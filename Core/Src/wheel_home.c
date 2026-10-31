#include "wheel_home.h"

/* 纯 C 归位状态机：不写 GPIO/PWM、不读定时器，只根据调用方给出的
 * 归一化位置/增量/按键/摇杆决定本轮应有的输出。 */

static int64_t wh_abs(int64_t value)
{
  return (value < 0) ? -value : value;
}

static int64_t wh_tolerance(uint32_t cpr)
{
  int64_t tol = (int64_t)cpr / 200;
  return (tol < 1) ? 1 : tol;
}

/* 静止/改善判据：CPR/1000，最小 1 个计数。 */
static int64_t wh_delta_limit(uint32_t cpr)
{
  int64_t limit = (int64_t)cpr / 1000;
  return (limit < 1) ? 1 : limit;
}

static int32_t wh_stick_offset(uint8_t value)
{
  int32_t offset = (int32_t)value - WHEEL_HOME_STICK_CENTER;
  return (offset < 0) ? -offset : offset;
}

static uint8_t wh_sticks_centered(const wheel_home_input_t *in)
{
  if (wh_stick_offset(in->lx) > WHEEL_HOME_STICK_DEADBAND ||
      wh_stick_offset(in->ly) > WHEEL_HOME_STICK_DEADBAND ||
      wh_stick_offset(in->rx) > WHEEL_HOME_STICK_DEADBAND ||
      wh_stick_offset(in->ry) > WHEEL_HOME_STICK_DEADBAND)
  {
    return 0U;
  }
  return 1U;
}

static uint8_t wh_sticks_active(const wheel_home_input_t *in)
{
  return (wh_sticks_centered(in) == 0U) ? 1U : 0U;
}

int64_t wheel_home_displacement(int64_t x, int64_t origin, uint32_t cpr)
{
  int64_t span;
  int64_t phase;
  int64_t displacement;

  if (cpr == 0U)
  {
    return 0;
  }

  span = (int64_t)cpr;
  phase = (x - origin) % span;
  if (phase < 0)
  {
    phase += span;
  }

  displacement = -phase;
  if (phase * 2 > span)
  {
    displacement += span;
  }
  return displacement;
}

void wheel_home_init(wheel_home_t *s, const uint32_t cpr[4])
{
  uint8_t w;

  for (w = 0U; w < 4U; w++)
  {
    s->cpr[w] = cpr[w];
    s->origin[w] = 0;
    s->target[w] = 0;
    s->best_abs_error[w] = 0;
    s->settle_ms[w] = 0U;
    s->progress_ms[w] = 0U;
    s->reverse_ms[w] = 0U;
    s->still_ms[w] = 0U;
    s->applied_direction[w] = 0;
    s->settle_active[w] = 0U;
    s->reverse_active[w] = 0U;
    s->progress_active[w] = 0U;
    s->still_active[w] = 0U;
  }

  s->press_ms = 0U;
  s->filter_ms = 0U;
  s->stable_frames = 0U;
  s->raw_select = 0U;
  s->frame_valid_now = 0U;
  s->have_frame = 0U;
  s->neutral_ms = 0U;
  s->stationary_ms = 0U;
  s->home_start_ms = 0U;
  s->last_valid_frame_ms = 0U;
  s->last_position_ms = 0U;
  s->reference_valid = 0U;
  s->long_consumed = 0U;
  s->press_active = 0U;
  s->filtered_select = 0U;
  s->candidate_select = 0U;
  s->filter_active = 0U;
  s->neutral_active = 0U;
  s->stationary_active = 0U;
  s->have_valid_frame = 0U;
  s->require_release = 0U;
  s->request_pending = 0U;
  s->request_wait_active = 0U;
  s->request_wait_ms = 0U;
  s->cancel_pending = 0U;
  s->mode = WHEEL_HOME_MANUAL;
  s->result = WHEEL_HOME_RESULT_NONE;
}

static void wh_zero_output(wheel_home_output_t *out)
{
  uint8_t w;

  for (w = 0U; w < 4U; w++)
  {
    out->pulse[w] = 0;
    out->error[w] = 0;
  }
  out->settled_mask = 0U;
}

static void wh_release_motion(wheel_home_t *s)
{
  uint8_t w;

  for (w = 0U; w < 4U; w++)
  {
    s->applied_direction[w] = 0;
    s->settle_active[w] = 0U;
    s->reverse_active[w] = 0U;
  }
}

/* 摇杆居中且四轮连续 300 ms 静止，才允许记录参考。 */
static uint8_t wh_still_ready(const wheel_home_t *s, const wheel_home_input_t *in,
                              uint32_t now)
{
  uint8_t w;

  if (wh_sticks_centered(in) == 0U)
  {
    return 0U;
  }
  for (w = 0U; w < 4U; w++)
  {
    if (s->still_active[w] == 0U)
    {
      return 0U;
    }
    if ((uint32_t)(now - s->still_ms[w]) < WHEEL_HOME_STILL_MS)
    {
      return 0U;
    }
  }
  return 1U;
}

static void wh_start_homing(wheel_home_t *s, const wheel_home_input_t *in, uint32_t now)
{
  uint8_t w;

  for (w = 0U; w < 4U; w++)
  {
    int64_t displacement = wheel_home_displacement(in->position[w], s->origin[w], s->cpr[w]);

    s->target[w] = in->position[w] + displacement;
    s->best_abs_error[w] = wh_abs(displacement);
    s->progress_ms[w] = now;
    s->applied_direction[w] = 0;
    s->settle_active[w] = 0U;
    s->reverse_active[w] = 0U;
  }

  s->mode = WHEEL_HOME_RUNNING;
  s->result = WHEEL_HOME_RESULT_NONE;
  s->request_pending = 0U;
  s->request_wait_active = 0U;
  s->home_start_ms = now;
}

void wheel_home_request_home(wheel_home_t *s)
{
  if (s == 0)
  {
    return;
  }

  /* 模式守卫：只有 MANUAL 模式才受理菜单请求。RUNNING 中忽略（不打断
   * 正在进行的归位、不覆写 result）；WAIT_NEUTRAL/BUTTON_HELD 同样忽略，
   * 等回到 MANUAL 后由页面重新发起。 */
  if (s->mode != WHEEL_HOME_MANUAL)
  {
    return;
  }

  /* 无参考：请求直接以 NO_REF 落地，不置 pending、不动轮。
   * 有参考：置 pending 并复用 require_release——提交键还按着的时候，
   * 按键事件一律不进状态机，避免菜单那一下按键泄漏成手势。 */
  if (s->reference_valid == 0U)
  {
    s->result = WHEEL_HOME_RESULT_NO_REF;
    return;
  }
  s->request_pending = 1U;
  s->request_wait_active = 0U;
  s->require_release = 1U;
}

static void wh_abort(wheel_home_t *s, wheel_home_result_t result, uint32_t now)
{
  s->mode = WHEEL_HOME_WAIT_NEUTRAL;
  s->result = result;
  s->neutral_active = 1U;
  s->neutral_ms = now;
  wh_release_motion(s);
}

/* 公开取消入口：本模块不持时钟，RUNNING 的 abrupt 收敛（wh_abort 需要
 * 真实毫秒）延到下一拍 wheel_home_step 用 in->now_ms 落地；pending 的
 * 请求与状态机无关，这里同步丢弃。 */
void wheel_home_cancel(wheel_home_t *s)
{
  if (s == 0)
  {
    return;
  }
  s->request_pending = 0U;
  s->request_wait_active = 0U;
  if (s->mode == WHEEL_HOME_RUNNING)
  {
    s->cancel_pending = 1U;
  }
}

static void wh_on_press(wheel_home_t *s, uint32_t now)
{
  if (s->require_release != 0U)
  {
    return;
  }
  if (s->press_active != 0U)
  {
    return;
  }

  s->press_active = 1U;
  s->press_ms = now;
  s->long_consumed = 0U;

  if (s->mode == WHEEL_HOME_RUNNING)
  {
    /* 再次按下即取消：不许同一周期落回摇杆驱动。 */
    wh_abort(s, WHEEL_HOME_RESULT_CANCELLED, now);
    s->press_active = 0U;
    s->long_consumed = 1U;
  }
  else if (s->mode == WHEEL_HOME_MANUAL)
  {
    s->mode = WHEEL_HOME_BUTTON_HELD;
  }
}

static void wh_on_release(wheel_home_t *s, const wheel_home_input_t *in, uint32_t now)
{
  uint8_t was_held = s->press_active;
  uint32_t held = (uint32_t)(now - s->press_ms);
  uint8_t held_mode = (s->mode == WHEEL_HOME_BUTTON_HELD) ? 1U : 0U;

  s->press_active = 0U;

  if (was_held == 0U || held_mode == 0U)
  {
    return;
  }

  if (s->long_consumed == 0U && held >= WHEEL_HOME_SHORT_PRESS_MIN_MS &&
      wh_sticks_centered(in) != 0U)
  {
    if (s->reference_valid != 0U)
    {
      wh_start_homing(s, in, now);
    }
    else
    {
      s->result = WHEEL_HOME_RESULT_NO_REF;
    }
  }

  if (s->mode == WHEEL_HOME_BUTTON_HELD)
  {
    s->mode = WHEEL_HOME_MANUAL;
  }
}

/* 手动页静止跟踪：必须每轮都喂，不能只在"用户请求记录"那一拍调用。
 * 连续静止是按时间累积出来的状态（WHEEL_HOME_STILL_MS，spec §10 明文
 * 300 ms；与归位路径共用同一宏，不占用归位逐轮到位的 SETTLE_MS），
 * 只在请求那一拍从零开始算，第一次调用必然不满足，用户松手速度稍快
 * 就记不上。
 * 返回值：1 = 四轮已连续静止满 300 ms 且本帧可用。 */
uint8_t wheel_home_track_manual_still(wheel_home_t *s,
                                      const wheel_home_input_t *in,
                                      uint32_t now)
{
  uint8_t w;
  uint8_t all_still = 1U;

  if (s == 0 || in == 0)
  {
    return 0U;
  }

  /* 帧/反馈无效、标定中、推着摇杆：一律打断静止计时，不让旧计时"复活"。 */
  if (in->frame_valid == 0U || in->feedback_valid == 0U ||
      in->calibration_mode != 0U || wh_sticks_active(in) != 0U)
  {
    for (w = 0U; w < 4U; w++)
    {
      s->record_still_active[w] = 0U;
    }
    return 0U;
  }

  /* CPR 为 0 无法归一化，也不该拿它当参考。 */
  for (w = 0U; w < 4U; w++)
  {
    if (s->cpr[w] == 0U)
    {
      s->record_still_active[w] = 0U;
      return 0U;
    }
  }

  for (w = 0U; w < 4U; w++)
  {
    int64_t limit = wh_delta_limit(s->cpr[w]);

    if (wh_abs((int64_t)in->delta[w]) > limit)
    {
      s->record_still_active[w] = 0U;
      all_still = 0U;
    }
    else
    {
      if (s->record_still_active[w] == 0U)
      {
        s->record_still_active[w] = 1U;
        s->record_still_ms[w] = now;
      }
      if ((uint32_t)(now - s->record_still_ms[w]) < WHEEL_HOME_STILL_MS)
      {
        all_still = 0U;
      }
    }
  }
  return all_still;
}

/* 手动对齐页里长按 SELECT 记录参考。
 * 静止状态由 wheel_home_track_manual_still 在每轮采样里累积，这里只做
 * 模式/帧有效性兜底和一次判定，保证"页面显示可以记录"和"真的写进去"
 * 用的是同一个累计量。
 * 返回 1 = 已保存，0 = 未保存（页面不显示 REF SET）。 */
uint8_t wheel_home_record_manual_reference(wheel_home_t *s,
                                           const wheel_home_input_t *in,
                                           uint32_t now)
{
  uint8_t w;

  if (s == 0 || in == 0)
  {
    return 0U;
  }

  /* 长按 SELECT 期间状态机在 BUTTON_HELD，正常路径必须也接受它。
   * 遥控驾驶中发现位移后才可能回 MANUAL，那时同样允许。 */
  if (s->mode != WHEEL_HOME_MANUAL && s->mode != WHEEL_HOME_BUTTON_HELD)
  {
    return 0U;
  }

  /* 本帧与反馈都必须有效：失联时记下的是过期位置。 */
  if (in->frame_valid == 0U || in->feedback_valid == 0U)
  {
    return 0U;
  }

  /* 标定模式不允许定零。 */
  if (in->calibration_mode != 0U)
  {
    return 0U;
  }

  /* 四轴回中：边推杆边记参考会把目标定在移动中的位置。 */
  if (wh_sticks_active(in) != 0U)
  {
    return 0U;
  }

  if (wheel_home_track_manual_still(s, in, now) == 0U)
  {
    return 0U;
  }

  for (w = 0U; w < 4U; w++)
  {
    s->origin[w] = in->position[w];
    s->target[w] = in->position[w];
  }
  s->reference_valid = 1U;
  return 1U;
}

static void wh_run_control(wheel_home_t *s, const wheel_home_input_t *in,
                           wheel_home_output_t *out, uint32_t now)
{
  uint8_t w;
  uint8_t all_settled = 1U;
  uint8_t stall = 0U;

  if (s->frame_valid_now != 0U && s->raw_select != 0U)
  {
    wh_abort(s, WHEEL_HOME_RESULT_CANCELLED, now);
    wh_zero_output(out);
    return;
  }
  if (wh_sticks_active(in) != 0U)
  {
    wh_abort(s, WHEEL_HOME_RESULT_CANCELLED, now);
    wh_zero_output(out);
    return;
  }
  if ((uint32_t)(now - s->home_start_ms) >= WHEEL_HOME_TIMEOUT_MS)
  {
    wh_abort(s, WHEEL_HOME_RESULT_TIMEOUT, now);
    wh_zero_output(out);
    return;
  }

  for (w = 0U; w < 4U; w++)
  {
    int64_t err = s->target[w] - in->position[w];
    int64_t tol = wh_tolerance(s->cpr[w]);
    int64_t limit = wh_delta_limit(s->cpr[w]);
    int8_t want = 0;
    int32_t pulse = 0;
    uint8_t settled = 0U;

    out->error[w] = err;
    if (err > 0)
    {
      want = 1;
    }
    else if (err < 0)
    {
      want = -1;
    }

    /* 越过目标：上采样时误差还在同号一侧，这一拍已经翻到另一侧，
     * 说明容差区被快速穿过。此时不论是不是才发现，都先零输出再等。
     * 关键：这里只置等待标志，不清 applied_direction。方向历史要一直保留到
     * 换向等待结束，否则下一拍会重新判定成"没有历史方向"而立刻反向通电。 */
    if (s->applied_direction[w] != 0 && want != 0 &&
        want != s->applied_direction[w])
    {
      if (s->reverse_active[w] == 0U)
      {
        s->reverse_active[w] = 1U;
        s->reverse_ms[w] = now;
      }
    }

    /* 任何反向输出都必须先过 40 ms 换向等待：容差区外直接翻符号也走这里。 */
    if (s->reverse_active[w] != 0U &&
        (uint32_t)(now - s->reverse_ms[w]) >= WHEEL_HOME_REVERSE_WAIT_MS)
    {
      /* 等待结束才允许丢弃旧方向，本拍起可以重新选方向。 */
      s->reverse_active[w] = 0U;
      s->applied_direction[w] = 0;
    }

    if (wh_abs(err) <= tol)
    {
      /* 这里不能清 applied_direction：进容差只是本拍零输出，
       * 电机刚刚还在朝上一个方向转。若就此丢掉方向历史，下一拍
       * 被推过目标到另一侧时会直接反向通电，绕过上面 40 ms 换向等待。
       * 方向由下面的"本拍是否真的反向通电"逻辑显式清零。 */
      if (s->settle_active[w] == 0U)
      {
        s->settle_active[w] = 1U;
        s->settle_ms[w] = now;
      }
      else if (wh_abs((int64_t)in->delta[w]) > limit)
      {
        s->settle_active[w] = 0U; /* 仍在漂移：停稳窗口重来 */
      }
      else if ((uint32_t)(now - s->settle_ms[w]) >= WHEEL_HOME_SETTLE_MS)
      {
        settled = 1U;
      }
      else
      {
        /* 停稳计时未满 */
      }
    }
    else
    {
      s->settle_active[w] = 0U;
      if (s->reverse_active[w] == 0U)
      {
        int64_t magnitude =
            (wh_abs(err) * (int64_t)WHEEL_HOME_FULL_SCALE) / (int64_t)s->cpr[w];

        if (magnitude > (int64_t)WHEEL_HOME_MAX_PULSE)
        {
          magnitude = (int64_t)WHEEL_HOME_MAX_PULSE;
        }
        if (magnitude < (int64_t)WHEEL_HOME_MIN_PULSE)
        {
          magnitude = (int64_t)WHEEL_HOME_MIN_PULSE;
        }
        s->applied_direction[w] = want;
        pulse = (want > 0) ? (int32_t)magnitude : -(int32_t)magnitude;
      }
    }

    /* 无进展只在真正施加了非零 PWM 的时间上累计。 */
    if (pulse != 0)
    {
      int64_t abs_err = wh_abs(err);

      if (abs_err < s->best_abs_error[w] - limit)
      {
        s->best_abs_error[w] = abs_err;
        s->progress_ms[w] = now;
      }
      else if ((uint32_t)(now - s->progress_ms[w]) >= WHEEL_HOME_NO_PROGRESS_MS)
      {
        stall = 1U;
      }
      else
      {
        /* 尚在观察窗口内 */
      }
    }
    else
    {
      s->progress_ms[w] = now;
    }

    if (settled != 0U)
    {
      out->settled_mask |= (uint8_t)(1U << w);
    }
    else
    {
      all_settled = 0U;
    }
    out->pulse[w] = pulse;
  }

  if (stall != 0U)
  {
    wh_abort(s, WHEEL_HOME_RESULT_STALL, now);
    wh_zero_output(out);
  }
  else if (all_settled != 0U)
  {
    s->mode = WHEEL_HOME_WAIT_NEUTRAL;
    s->result = WHEEL_HOME_RESULT_DONE;
    s->neutral_active = 1U;
    s->neutral_ms = now;
    wh_release_motion(s);
    wh_zero_output(out);
  }
  else
  {
    /* 继续归位 */
  }
}

static void wh_run_wait_neutral(wheel_home_t *s, const wheel_home_input_t *in,
                                uint32_t now)
{
  uint8_t released = (in->frame_valid != 0U && s->press_active == 0U &&
                      s->filtered_select == 0U) ? 1U : 0U;

  if (released != 0U && wh_sticks_centered(in) != 0U)
  {
    if (s->neutral_active == 0U)
    {
      s->neutral_active = 1U;
      s->neutral_ms = now;
    }
    else if ((uint32_t)(now - s->neutral_ms) >= WHEEL_HOME_RESUME_NEUTRAL_MS)
    {
      s->mode = WHEEL_HOME_MANUAL;
      s->neutral_active = 0U;
    }
    else
    {
      /* 继续等满 100 ms */
    }
  }
  else
  {
    s->neutral_active = 0U;
  }
}

void wheel_home_step(wheel_home_t *s, const wheel_home_input_t *in,
                     wheel_home_output_t *out)
{
  uint8_t w;
  uint32_t now = in->now_ms;
  uint8_t sample_ok = 1U;

  wh_zero_output(out);
  out->owns_motors = 1U;
  /* recovering 是"本拍"语义，必须每次调用重新判定；
   * 否则一旦置位就会永远对外报告恢复中。 */
  out->recovering = 0U;

  /* CPR 为 0 无法归一化：当反馈错误处理，绝不做除法。 */
  for (w = 0U; w < 4U; w++)
  {
    if (s->cpr[w] == 0U)
    {
      s->reference_valid = 0U;
      s->mode = WHEEL_HOME_MANUAL;
      s->result = WHEEL_HOME_RESULT_FEEDBACK_ERROR;
      s->press_active = 0U;
      s->long_consumed = 1U;
      wh_release_motion(s);
      out->mode = s->mode;
      out->result = s->result;
      out->reference_valid = 0U;
      return;
    }
  }

  /* 公开取消入口的落地拍：显式取消优先于本拍其它事件。abrupt 收敛到
   * WAIT_NEUTRAL/CANCELLED；手势清零并要求先看到释放（与帧失联取消
   * 同一口径）。非 RUNNING 时 cancel_pending 只是幂等清除。 */
  if (s->cancel_pending != 0U)
  {
    s->cancel_pending = 0U;
    if (s->mode == WHEEL_HOME_RUNNING)
    {
      wh_abort(s, WHEEL_HOME_RESULT_CANCELLED, now);
      s->press_active = 0U;
      s->long_consumed = 1U;
      s->filter_active = 0U;
      s->stable_frames = 0U;
      s->require_release = 1U;
    }
  }

  /* 采样新鲜度：间隔超过 50 ms 可能漏掉 16 位回绕，参考不可再用。 */
  if (in->feedback_valid == 0U)
  {
    sample_ok = 0U;
  }
  else if (s->have_valid_frame != 0U &&
           (uint32_t)(now - s->last_position_ms) > WHEEL_HOME_SAMPLE_TIMEOUT_MS)
  {
    sample_ok = 0U;
  }
  else
  {
    s->last_position_ms = now;
    s->have_valid_frame = 1U;
  }

  if (sample_ok == 0U)
  {
    /* 间隔超时：这一拍的位置可能已经漏掉 16 位回绕，旧参考的相位不再可信。
     * 必须真正作废参考；同时把计时基准钉到"现在"，否则下一拍算出的间隔
     * 依旧大于超时阈值，会永远卡在恢复分支里出不来。
     * 用户要重新长按记录参考后，归位才会恢复可用。 */
    s->press_active = 0U;
    s->long_consumed = 1U;
    wh_release_motion(s);
    s->reference_valid = 0U;
    for (w = 0U; w < 4U; w++)
    {
      s->settle_active[w] = 0U;
    }
    s->stationary_active = 0U;
    if (s->mode == WHEEL_HOME_RUNNING || s->mode == WHEEL_HOME_BUTTON_HELD)
    {
      wh_abort(s, WHEEL_HOME_RESULT_FEEDBACK_ERROR, now);
    }
    /* 本拍的样本已经被丢弃，不参与任何判断；但时间基准必须前进。 */
    s->last_position_ms = now;
    s->have_valid_frame = 1U;
    s->recovering = 1U;
    out->mode = s->mode;
    out->result = s->result;
    out->reference_valid = 0U;
    out->recovering = 1U;
    return;
  }

  if (s->recovering != 0U)
  {
    /* 恢复窗口：本拍只重新对时，不判按键、不停稳、不计无进展。 */
    s->recovering = 0U;
    out->mode = s->mode;
    out->result = s->result;
    out->reference_valid = s->reference_valid;
    out->recovering = 1U;
    return;
  }

  /* 标定模式优先于所有按键：电机保持睡眠，但不影响参考本身的有效性。 */
  if (in->calibration_mode != 0U)
  {
    if (s->mode == WHEEL_HOME_RUNNING || s->mode == WHEEL_HOME_BUTTON_HELD)
    {
      s->result = WHEEL_HOME_RESULT_CANCELLED;
    }
    s->mode = WHEEL_HOME_WAIT_NEUTRAL;
    s->neutral_active = 1U;
    s->neutral_ms = now;
    s->press_active = 0U;
    s->long_consumed = 1U;
    s->filter_active = 0U;
    s->stable_frames = 0U;
    s->require_release = 1U;
    wh_release_motion(s);
    out->mode = s->mode;
    out->result = s->result;
    out->reference_valid = s->reference_valid;
    return;
  }

  /* 逐轮静止跟踪：记录参考与停稳确认共用。 */
  for (w = 0U; w < 4U; w++)
  {
    if (wh_abs((int64_t)in->delta[w]) <= wh_delta_limit(s->cpr[w]))
    {
      if (s->still_active[w] == 0U)
      {
        s->still_active[w] = 1U;
        s->still_ms[w] = now;
      }
    }
    else
    {
      s->still_active[w] = 0U;
    }
  }

  /* 本周期帧状态：无效帧不产生任何按键事件。 */
  s->frame_valid_now = (in->frame_valid != 0U) ? 1U : 0U;
  if (s->frame_valid_now != 0U)
  {
    s->raw_select = ((in->buttons & (uint16_t)(1U << WHEEL_HOME_SELECT_BIT)) == 0U) ? 1U : 0U;
    s->last_valid_frame_ms = now;
    s->have_frame = 1U;
  }

  /* 有效帧中断 200 ms：归位立即取消，手势清零，重连后先看释放。 */
  if (s->have_frame != 0U &&
      (uint32_t)(now - s->last_valid_frame_ms) >= WHEEL_HOME_FRAME_LOSS_MS)
  {
    if (s->mode == WHEEL_HOME_RUNNING || s->mode == WHEEL_HOME_BUTTON_HELD)
    {
      wh_abort(s, WHEEL_HOME_RESULT_CANCELLED, now);
    }
    s->press_active = 0U;
    s->long_consumed = 1U;
    s->filter_active = 0U;
    s->stable_frames = 0U;
    s->require_release = 1U;
    out->mode = s->mode;
    out->result = s->result;
    out->reference_valid = s->reference_valid;
    out->owns_motors = (s->mode == WHEEL_HOME_MANUAL) ? 0U : 1U;
    return;
  }

  /* 按键滤波：只统计连续有效帧，坏帧既不推进也不清零已确认电平。 */
  if (s->frame_valid_now != 0U)
  {
    uint8_t raw = s->raw_select;

    if (s->filter_active == 0U)
    {
      s->filter_active = 1U;
      s->candidate_select = raw;
      s->stable_frames = 1U;
    }
    else if (raw != s->candidate_select)
    {
      s->candidate_select = raw;
      s->stable_frames = 1U;
    }
    else if (s->stable_frames < 255U)
    {
      s->stable_frames++;
    }
    else
    {
      /* 已饱和 */
    }

    if (s->stable_frames >= WHEEL_HOME_BUTTON_STABLE_FRAMES)
    {
      if (raw == 0U)
      {
        s->require_release = 0U;
      }
      if (raw != s->filtered_select)
      {
        s->filtered_select = raw;
        if (raw != 0U)
        {
          wh_on_press(s, now);
        }
        else
        {
          wh_on_release(s, in, now);
        }
      }
    }
  }

  /* 长按 2 s：任何页面都统一消费手势，防止长按的释放沿退化成短按归位。
   * 只有"记录参考"这个动作受手动标定页门控；
   * 不满足停稳条件也不退化成短按。 */
  if (s->mode == WHEEL_HOME_BUTTON_HELD && s->press_active != 0U &&
      s->long_consumed == 0U &&
      (uint32_t)(now - s->press_ms) >= WHEEL_HOME_LONG_PRESS_MS)
  {
    s->long_consumed = 1U;
    if (in->manual_calib_page != 0U && wh_still_ready(s, in, now) != 0U)
    {
      for (w = 0U; w < 4U; w++)
      {
        s->origin[w] = in->position[w];
      }
      s->reference_valid = 1U;
      s->result = WHEEL_HOME_RESULT_RECORDED;
    }
  }

  /* 菜单请求的归位：只在 MANUAL 模式推进放行门。
   * 四轴回中且 SELECT/L1/R1 全部释放连续满 100 ms 才进 RUNNING；
   * 提交键的释放发生在进 RUNNING 之前，天然不计为取消。 */
  if (s->request_pending != 0U && s->mode == WHEEL_HOME_MANUAL)
  {
    /* 按键低有效：掩码位全为 1 才是"全部释放"。 */
    uint16_t gate_released = (uint16_t)(in->buttons & WHEEL_HOME_REQUEST_GATE_MASK);

    if (in->frame_valid != 0U &&
        gate_released == WHEEL_HOME_REQUEST_GATE_MASK &&
        wh_sticks_centered(in) != 0U)
    {
      if (s->request_wait_active == 0U)
      {
        s->request_wait_active = 1U;
        s->request_wait_ms = now;
      }
      else if ((uint32_t)(now - s->request_wait_ms) >= WHEEL_HOME_RESUME_NEUTRAL_MS)
      {
        s->request_wait_active = 0U;
        s->request_pending = 0U;
        if (s->reference_valid != 0U)
        {
          wh_start_homing(s, in, now);
        }
        else
        {
          s->result = WHEEL_HOME_RESULT_NO_REF;
        }
      }
      else
      {
        /* 放行门计时未满 */
      }
    }
    else
    {
      /* 有按键没松开或摇杆出死区：重新等满 100 ms。 */
      s->request_wait_active = 0U;
    }
  }

  if (s->mode == WHEEL_HOME_RUNNING)
  {
    wh_run_control(s, in, out, now);
  }
  else if (s->mode == WHEEL_HOME_WAIT_NEUTRAL)
  {
    wh_run_wait_neutral(s, in, now);
    wh_zero_output(out);
  }
  else
  {
    /* MANUAL / BUTTON_HELD：四轮零输出，等 Ctrl 决定是否按摇杆驱动。 */
    wh_zero_output(out);
  }

  out->mode = s->mode;
  out->result = s->result;
  out->reference_valid = s->reference_valid;
  out->owns_motors = (s->mode == WHEEL_HOME_MANUAL) ? 0U : 1U;
}

/* 顶栏状态文本。全部 <=4 字符，配合 6x12 字库只出大写字母。
 * 任何未列出的组合都退化成 "WAIT"，避免显示未定义状态。 */
void wheel_home_status_text(char *buf, uint32_t size,
                            const wheel_home_output_t *out)
{
  const char *text = "WAIT";
  uint32_t i;

  if (buf == 0 || size == 0U)
  {
    return;
  }

  if (out != 0 && out->fault_mask != 0U)
  {
    /* 反馈故障优先于普通状态：锁存后输出已清零，必须让人看见。 */
    text = "FLT";
  }
  else if (out != 0)
  {
    switch (out->mode)
    {
      case WHEEL_HOME_MANUAL:
        text = (out->reference_valid != 0U) ? "RDY" : "NORF";
        break;
      case WHEEL_HOME_BUTTON_HELD:
        text = "HOLD";
        break;
      case WHEEL_HOME_RUNNING:
        text = "HOME";
        break;
      case WHEEL_HOME_WAIT_NEUTRAL:
      default:
        switch (out->result)
        {
          case WHEEL_HOME_RESULT_RECORDED:       text = "REC";  break;
          case WHEEL_HOME_RESULT_DONE:           text = "DONE"; break;
          case WHEEL_HOME_RESULT_CANCELLED:      text = "CANC"; break;
          case WHEEL_HOME_RESULT_NO_REF:         text = "NORF"; break;
          case WHEEL_HOME_RESULT_STALL:          text = "STAL"; break;
          case WHEEL_HOME_RESULT_TIMEOUT:        text = "TOUT"; break;
          case WHEEL_HOME_RESULT_FEEDBACK_ERROR: text = "FERR"; break;
          case WHEEL_HOME_RESULT_NONE:
          default:                               text = "WAIT"; break;
        }
        break;
    }
  }

  for (i = 0U; i + 1U < size && text[i] != '\0'; i++)
  {
    buf[i] = text[i];
  }
  buf[i] = '\0';
}
