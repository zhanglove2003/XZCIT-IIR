#include "selfcheck_service.h"

#include "drive_ctrl.h" /* drive_calibration_mode */

void selfcheck_service_init(selfcheck_service_t *s)
{
  uint8_t w;

  if (s == 0)
  {
    return;
  }
  s->phase = SELFCHECK_IDLE;
  s->wheel = 0U;
  s->subphase = 0U;
  for (w = 0U; w < 4U; w++)
  {
    s->state[w] = 0U;
    motor_test_reset(&s->result[w]);
  }
  s->passed = 0U;
  s->seq = 0U;
  s->phase_start_ms = 0U;
  s->phase_counts = 0;
}

void selfcheck_service_request(selfcheck_service_t *s, uint32_t now_ms)
{
  uint8_t w;

  if (s == 0 || s->phase == SELFCHECK_RUNNING)
  {
    return; /* 运行中重复请求不打断时序 */
  }
  s->phase = SELFCHECK_RUNNING;
  s->wheel = 0U;
  s->subphase = 0U;
  for (w = 0U; w < 4U; w++)
  {
    s->state[w] = 0U;
    motor_test_reset(&s->result[w]);
  }
  s->state[0] = 1U; /* A 轮进入运行 */
  s->passed = 0U;
  s->phase_start_ms = now_ms;
  s->phase_counts = 0;
}

void selfcheck_service_cancel(selfcheck_service_t *s)
{
  if (s == 0 || s->phase != SELFCHECK_RUNNING)
  {
    return;
  }
  /* 运行中的受试轮定格为失败（未测完 = 非通过）；已完成轮的
   * state/result 原样保留，结果页可回看。不清旧故障锁存。 */
  if (s->state[s->wheel] == 1U)
  {
    s->state[s->wheel] = 3U;
  }
  s->passed = 0U;
  s->phase = SELFCHECK_DONE;
}

void selfcheck_service_step(selfcheck_service_t *s, const int32_t fb_delta[4],
                            uint32_t now_ms, int32_t command[4])
{
  uint8_t w;
  uint32_t elapsed;

  if (s == 0 || command == 0)
  {
    return;
  }
  for (w = 0U; w < 4U; w++)
  {
    command[w] = 0;
  }
  if (s->phase != SELFCHECK_RUNNING || s->wheel >= 4U)
  {
    return;
  }

  /* 标定模式：立即放弃整轮——全轮 SKIP、state 保持等待、command 恒 0 */
  if (drive_calibration_mode != 0U)
  {
    for (w = 0U; w < 4U; w++)
    {
      motor_test_skip(&s->result[w]);
      s->state[w] = 0U;
    }
    s->passed = 0U;
    s->phase = SELFCHECK_DONE;
    return;
  }

  /* 受试轮增量先入账：边界拍的 fb 属于刚结束的段 */
  s->phase_counts += fb_delta[s->wheel];
  elapsed = now_ms - s->phase_start_ms; /* 无符号差值，回绕安全 */

  switch (s->subphase)
  {
  case 0U: /* 前转段 */
    if (elapsed >= (uint32_t)SELFCHECK_RUN_MS)
    {
      s->result[s->wheel].forward_counts = s->phase_counts;
      s->phase_counts = 0;
      s->subphase = 1U;
      s->phase_start_ms = now_ms;
    }
    break;
  case 1U: /* 前转后暂停 */
    if (elapsed >= (uint32_t)SELFCHECK_PAUSE_MS)
    {
      s->phase_counts = 0;
      s->subphase = 2U;
      s->phase_start_ms = now_ms;
    }
    break;
  case 2U: /* 反转段 */
    if (elapsed >= (uint32_t)SELFCHECK_RUN_MS)
    {
      s->result[s->wheel].reverse_counts = s->phase_counts;
      s->phase_counts = 0;
      s->subphase = 3U;
      s->phase_start_ms = now_ms;
    }
    break;
  default: /* 3U：反转后暂停 → 本轮收尾 */
    if (elapsed >= (uint32_t)SELFCHECK_PAUSE_MS)
    {
      s->phase_counts = 0;
      s->subphase = 0U;
      s->phase_start_ms = now_ms;
      /* 方向核对：前转段净计数须为正、反转为负；反号（轮子倒转）按该
       * 向失败处理——motor_test_record 只看幅值，方向由本服务把关。
       * result 里仍保留原始带符号净计数，供诊断回看。 */
      {
        int32_t fwd = s->result[s->wheel].forward_counts;
        int32_t rev = s->result[s->wheel].reverse_counts;

        motor_test_record(&s->result[s->wheel],
                          fwd > 0 ? fwd : 0,
                          rev < 0 ? rev : 0);
        s->result[s->wheel].forward_counts = fwd;
        s->result[s->wheel].reverse_counts = rev;
      }
      s->state[s->wheel] =
          motor_test_passed(&s->result[s->wheel]) != 0U ? 2U : 3U;
      s->wheel++;
      if (s->wheel >= 4U)
      {
        uint8_t all_passed = 1U;
        for (w = 0U; w < 4U; w++)
        {
          if (s->state[w] != 2U)
          {
            all_passed = 0U;
          }
        }
        s->passed = all_passed;
        s->seq++;
        s->phase = SELFCHECK_DONE;
      }
      else
      {
        s->state[s->wheel] = 1U; /* 下一轮进入运行 */
      }
    }
    break;
  }

  /* 段推进之后再落 command：边界拍输出的是新段的值 */
  if (s->phase == SELFCHECK_RUNNING)
  {
    if (s->subphase == 0U)
    {
      command[s->wheel] = (int32_t)SELFCHECK_PWM_PULSE; /* 前转为正 */
    }
    else if (s->subphase == 2U)
    {
      command[s->wheel] = -(int32_t)SELFCHECK_PWM_PULSE; /* 反转为负 */
    }
  }
}
