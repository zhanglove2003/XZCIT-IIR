/* Shortest-path wheel homing: maths, per-wheel proportional limiting,
 * settle confirmation, reverse wait, no-progress and timeout. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "wheel_home.h"
#include "wheel_feedback.h"

static const uint32_t CPR[4] = {213203U, 212664U, 214993U, 214348U};

typedef struct
{
  wheel_home_t s;
  wheel_home_output_t o;
  uint32_t now;
  int64_t pos[4];
  uint16_t buttons;
  uint8_t lx, ly, rx, ry;
  uint8_t frame_valid, feedback_valid, calibration_mode;
  uint8_t manual_calib_page;
} rig_t;

static rig_t rig;

static void rig_reset(const uint32_t cpr[4])
{
  memset(&rig, 0, sizeof rig);
  wheel_home_init(&rig.s, cpr);
  rig.now = 1000U;
  rig.buttons = 0xFFFFU;
  rig.lx = rig.ly = rig.rx = rig.ry = WHEEL_HOME_STICK_CENTER;
  rig.frame_valid = 1U;
  rig.feedback_valid = 1U;
}

static void step4(uint32_t dt, int32_t d0, int32_t d1, int32_t d2, int32_t d3)
{
  const int32_t d[4] = {d0, d1, d2, d3};
  wheel_home_input_t in;
  int w;

  rig.now += dt;
  for (w = 0; w < 4; w++) { rig.pos[w] += d[w]; }

  memset(&in, 0, sizeof in);
  in.now_ms = rig.now;
  for (w = 0; w < 4; w++) { in.position[w] = rig.pos[w]; in.delta[w] = d[w]; }
  in.frame_valid = rig.frame_valid;
  in.feedback_valid = rig.feedback_valid;
  in.calibration_mode = rig.calibration_mode;
  in.manual_calib_page = rig.manual_calib_page;
  in.buttons = rig.buttons;
  in.lx = rig.lx;
  in.ly = rig.ly;
  in.rx = rig.rx;
  in.ry = rig.ry;
  wheel_home_step(&rig.s, &in, &rig.o);
}

static void pump(uint32_t total_ms)
{
  uint32_t t;
  for (t = 0U; t < total_ms; t += 10U) { step4(10U, 0, 0, 0, 0); }
}

static void press_select(void) { rig.buttons = (uint16_t)(0xFFFFU & ~(1U << WHEEL_HOME_SELECT_BIT)); }
static void release_select(void) { rig.buttons = 0xFFFFU; }
static void hold_l1(void) { rig.buttons = (uint16_t)(rig.buttons & ~(1U << WHEEL_HOME_L1_BIT)); }
static void release_l1(void) { rig.buttons = (uint16_t)(rig.buttons | (1U << WHEEL_HOME_L1_BIT)); }

/* Long press on still wheels records the four origins.
 * 长按记录只属于手动标定页：辅助函数在页面作用域内完成记录。 */
static void record_reference(void)
{
  rig.manual_calib_page = 1U;
  press_select();
  pump(2050U);
  assert(rig.o.result == WHEEL_HOME_RESULT_RECORDED);
  assert(rig.o.reference_valid == 1U);
  release_select();
  pump(100U);
  rig.manual_calib_page = 0U;
  assert(rig.o.mode == WHEEL_HOME_MANUAL);
}

static uint32_t g_home_start;

static void tap_select(void)
{
  press_select();
  pump(50U);
  release_select();
  pump(40U);
}

static void short_press(void)
{
  press_select();
  pump(50U);
  release_select();
  pump(40U);
  assert(rig.o.mode == WHEEL_HOME_RUNNING);
  g_home_start = rig.s.home_start_ms; /* 与状态机自己的计时基准一致 */
}

static void test_displacement(void)
{
  assert(wheel_home_displacement(3500, 0, 1000U) == -500);
  assert(wheel_home_displacement(3250, 0, 1000U) == -250);
  assert(wheel_home_displacement(3750, 0, 1000U) == 250);
  assert(wheel_home_displacement(-250, 0, 1000U) == 250);
  assert(wheel_home_displacement(1000, 0, 1000U) == 0);
  assert(wheel_home_displacement(1000000003500LL, 0, 1000U) == -500);
  assert(wheel_home_displacement(100, 100, 1000U) == 0);
  /* Odd CPR: no exact half turn exists, so 500 -> -500 and 501 -> +500. */
  assert(wheel_home_displacement(500, 0, 1001U) == -500);
  assert(wheel_home_displacement(501, 0, 1001U) == 500);
  /* cpr == 0 has no phase; callers must reject it instead of dividing. */
  assert(wheel_home_displacement(1234, 0, 0U) == 0);
}

static void test_zero_cpr_rejected(void)
{
  const uint32_t bad[4] = {0U, 212664U, 214993U, 214348U};

  rig_reset(bad);
  step4(10U, 0, 0, 0, 0);
  assert(rig.o.result == WHEEL_HOME_RESULT_FEEDBACK_ERROR);
  assert(rig.o.reference_valid == 0U);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }
}

/* -0.5 / +0.25 / -0.1 / 0 turn errors produce independent signed outputs. */
static void test_proportional_per_wheel(void)
{
  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;   /* half turn -> reverse */
  rig.pos[1] -= (int64_t)CPR[1] / 4;   /* quarter turn -> forward */
  rig.pos[2] += (int64_t)CPR[2] / 10;  /* tenth turn -> reverse */
  short_press();

  assert(rig.o.mode == WHEEL_HOME_RUNNING);
  assert(rig.o.pulse[0] == -(int32_t)WHEEL_HOME_MAX_PULSE);
  assert(rig.o.pulse[1] == (int32_t)WHEEL_HOME_MAX_PULSE);
  assert(rig.o.pulse[2] < -1200 && rig.o.pulse[2] > -1400);
  assert(rig.o.pulse[3] == 0);
  assert(rig.o.error[0] == -(int64_t)CPR[0] / 2);
  assert(rig.o.error[3] == 0);
}

/* Tolerance is max(1, cpr/200) per wheel: 1065 counts is inside RF's band
 * but outside LF's, so the four wheels must not share one CPR. */
static void test_tolerance_uses_each_wheel_cpr(void)
{
  rig_reset(CPR);
  record_reference();

  for (int w = 0; w < 4; w++) { rig.pos[w] += 1065; }
  short_press();

  assert(rig.o.error[0] == -1065 && rig.o.error[1] == -1065);
  assert(rig.o.pulse[0] == 0);          /* RF: tol 1066 */
  assert(rig.o.pulse[1] == -1200);      /* LF: tol 1063 -> min pulse */
  assert(rig.o.pulse[2] == 0);          /* RR: tol 1074 */
  assert(rig.o.pulse[3] == 0);          /* LR: tol 1071 */
}

/* One wheel drives home, stops inside the band, and settles for 200 ms. */
static void test_settle_then_done(void)
{
  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;
  short_press();
  assert(rig.o.pulse[0] < 0);

  for (int guard = 0; guard < 400; guard++)
  {
    int64_t err = rig.s.target[0] - rig.pos[0];
    int32_t step;
    if (err > -((int64_t)CPR[0] / 200) && err < ((int64_t)CPR[0] / 200)) { break; }
    step = (err < 0) ? -10000 : 10000;
    if (err > -10000 && err < 10000) { step = (int32_t)err; }
    step4(10U, step, 0, 0, 0);
    assert(rig.o.pulse[1] == 0);
    assert(rig.o.pulse[2] == 0);
    assert(rig.o.pulse[3] == 0);
  }

  assert(rig.o.pulse[0] == 0);
  assert(rig.o.mode == WHEEL_HOME_RUNNING);

  pump(150U);
  assert(rig.o.mode == WHEEL_HOME_RUNNING);   /* 200 ms settle not reached */

  pump(100U);
  assert(rig.o.mode == WHEEL_HOME_WAIT_NEUTRAL);
  assert(rig.o.result == WHEEL_HOME_RESULT_DONE);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }
  assert(rig.o.owns_motors == 1U);

  /* Released and centred for 100 ms hands control back to the stick. */
  pump(150U);
  assert(rig.o.mode == WHEEL_HOME_MANUAL);
  assert(rig.o.owns_motors == 0U);
}

/* Crossing the target zeroes the output for 40 ms before reversing. */
static void test_reverse_wait(void)
{
  uint32_t waited = 0U;
  uint8_t reversed = 0U;

  rig_reset(CPR);
  record_reference();

  rig.pos[0] -= (int64_t)CPR[0] / 20;
  short_press();
  assert(rig.o.pulse[0] > 0);

  step4(10U, (int32_t)((int64_t)CPR[0] / 10), 0, 0, 0);
  assert(rig.o.error[0] < 0);
  assert(rig.o.pulse[0] == 0);

  for (int i = 0; i < 20 && reversed == 0U; i++)
  {
    waited += 10U;
    step4(10U, 0, 0, 0, 0);
    if (rig.o.pulse[0] < 0) { reversed = 1U; }
  }
  assert(reversed == 1U);
  assert(waited >= WHEEL_HOME_REVERSE_WAIT_MS);
  assert(waited <= WHEEL_HOME_REVERSE_WAIT_MS + 10U);
}

/* 容差区被一次快速越过（误差从 +20 直接跳到 -30，全程没落在容差内）：
 * 反向输出前必须仍然等满换向时间。 */
static void test_overshoot_outside_tolerance(void)
{
  uint32_t waited = 0U;
  uint8_t reversed = 0U;

  rig_reset(CPR);
  record_reference();

  rig.pos[0] -= (int64_t)CPR[0] / 20;
  short_press();
  assert(rig.o.pulse[0] > 0);

  /* 一次跨过目标，且落点仍在容差外（tol = CPR/200 ≈ 1066，落点取 2×tol）。 */
  {
    int64_t target = rig.s.target[0];
    int64_t overshoot = -(int64_t)(2U * CPR[0] / 200U); /* err = target - pos */
    int64_t jump = target - overshoot - rig.pos[0]; /* pos 越过到另一侧 */
    step4(10U, (int32_t)jump, 0, 0, 0);
    assert(rig.o.error[0] == overshoot);
  }
  assert(rig.o.pulse[0] == 0);

  for (int i = 0; i < 20 && reversed == 0U; i++)
  {
    waited += 10U;
    step4(10U, 0, 0, 0, 0);
    if (rig.o.pulse[0] < 0) { reversed = 1U; }
  }
  assert(reversed == 1U);
  assert(waited >= WHEEL_HOME_REVERSE_WAIT_MS);
}

/* 采样超时后必须能真正恢复：超时那一拍作废旧参考，下一拍新样本进来
 * 要把计时基准重新钉到当前时间，否则之后每一拍都会继续判超时（死循环）。 */
static void test_sample_timeout_recovers(void)
{
  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;
  short_press();
  assert(rig.o.pulse[0] < 0);
  assert(rig.o.reference_valid == 1U);

  /* 一拍落空：间隔 300 ms > SAMPLE_TIMEOUT(50)。 */
  step4(300U, 0, 0, 0, 0);
  assert(rig.o.recovering == 1U);
  /* 漏过回绕的参考不可再用。 */
  assert(rig.o.reference_valid == 0U);
  assert(rig.o.mode == WHEEL_HOME_WAIT_NEUTRAL);
  assert(rig.s.reference_valid == 0U);

  /* 下一拍紧跟着来：内部恢复标志清掉，但这一拍本身不作数（对外仍报 recovering）。 */
  step4(10U, 0, 0, 0, 0);
  assert(rig.o.recovering == 1U);
  assert(rig.s.recovering == 0U);

  /* 再下一拍必须完全恢复正常。 */
  step4(10U, 0, 0, 0, 0);
  assert(rig.o.recovering == 0U);
  assert(rig.s.recovering == 0U);

  /* 再连续跑一段：不得再次被判为超时。 */
  {
    uint8_t w2;
    for (w2 = 0U; w2 < 20U; w2++)
    {
      step4(10U, 0, 0, 0, 0);
      assert(rig.o.recovering == 0U);
    }
  }
}

/* 进容差区不得丢掉最后实际驱动方向：之后再反向，仍要等满换向时间。 */
static void test_tolerance_keeps_last_direction(void)
{
  uint32_t waited = 0U;
  uint8_t reversed = 0U;
  int64_t target;
  int8_t driven;

  rig_reset(CPR);
  record_reference();

  /* 起步偏在目标一侧，短按后按该侧方向输出。 */
  rig.pos[0] -= (int64_t)CPR[0] / 20;
  short_press();
  assert(rig.o.pulse[0] != 0);
  driven = (int8_t)((rig.o.pulse[0] > 0) ? 1 : -1);

  /* 一次落到容差内：输出归零，进入停稳计时。 */
  target = rig.s.target[0];
  step4(10U, (int32_t)(target - rig.pos[0]), 0, 0, 0);
  assert(rig.o.error[0] == 0);
  assert(rig.o.pulse[0] == 0);

  /* 还没停稳就被推过目标到另一侧：反向前必须等满换向时间。
   * 越过量要大于容差，才会产生反向输出请求。 */
  step4(10U, (int32_t)(driven * 4 * (int64_t)WHEEL_HOME_MIN_PULSE), 0, 0, 0);
  assert(rig.o.error[0] * driven < 0);
  assert(rig.o.pulse[0] == 0);
  /* 换向等待从这一拍开始计时，它自身已经消耗了一个采样周期。 */
  waited = 10U;

  for (int i = 0; i < 20 && reversed == 0U; i++)
  {
    step4(10U, 0, 0, 0, 0);
    if (rig.o.pulse[0] * driven < 0) { reversed = 1U; }
    else { waited += 10U; }
  }
  assert(reversed == 1U);
  assert(waited >= WHEEL_HOME_REVERSE_WAIT_MS);
}

/* 手动对齐页的记录入口必须自己核对安全前提，
 * 不能假定"页面已经检查过"就照写。 */
static void test_manual_record_checks_preconditions(void)
{
  wheel_home_input_t in;
  uint32_t t = 5000U;

  rig_reset(CPR);
  record_reference();

  memset(&in, 0, sizeof in);
  in.now_ms = t;
  in.frame_valid = 1U;
  in.feedback_valid = 1U;
  in.calibration_mode = 0U;
  in.buttons = (uint16_t)(0xFFFFU & ~(1U << WHEEL_HOME_SELECT_BIT));
  in.lx = in.ly = in.rx = in.ry = WHEEL_HOME_STICK_CENTER;
  for (int w = 0; w < 4; w++)
  {
    in.position[w] = 12345;
    in.delta[w] = 0;
  }

  /* 场景 A：长按 SELECT 时状态机在 BUTTON_HELD，这里必须仍然能记录，
   *        否则"正常长按"这条唯一路径永远写不进参考。 */
  assert(rig.s.mode == WHEEL_HOME_MANUAL);
  press_select();
  pump(50U);
  assert(rig.s.mode == WHEEL_HOME_BUTTON_HELD);

  /* 摇杆没回中 -> 拒绝。 */
  in.lx = (uint8_t)(WHEEL_HOME_STICK_CENTER + WHEEL_HOME_STICK_DEADBAND + 20U);
  assert(wheel_home_record_manual_reference(&rig.s, &in, t) == 0U);
  in.lx = WHEEL_HOME_STICK_CENTER;

  /* 反馈无效 -> 拒绝。 */
  in.feedback_valid = 0U;
  assert(wheel_home_record_manual_reference(&rig.s, &in, t) == 0U);
  in.feedback_valid = 1U;

  /* 帧无效 -> 拒绝。 */
  in.frame_valid = 0U;
  assert(wheel_home_record_manual_reference(&rig.s, &in, t) == 0U);
  in.frame_valid = 1U;

  /* 车轮还在转 -> 拒绝；连续静止够 200 ms 才接受。 */
  for (int w = 0; w < 4; w++) { in.delta[w] = (int32_t)(CPR[w] / 100); }
  in.now_ms = t;
  assert(wheel_home_record_manual_reference(&rig.s, &in, t) == 0U);
  for (int w = 0; w < 4; w++) { in.delta[w] = 0; }
  assert(wheel_home_record_manual_reference(&rig.s, &in, t + 100U) == 0U);
  assert(wheel_home_record_manual_reference(&rig.s, &in,
                                            t + 100U + WHEEL_HOME_SETTLE_MS) == 1U);
  assert(rig.s.reference_valid == 1U);
  for (int w = 0; w < 4; w++) { assert(rig.s.origin[w] == 12345); }

  /* 不在手动相关状态下拒绝。 */
  rig.s.mode = WHEEL_HOME_RUNNING;
  assert(wheel_home_record_manual_reference(&rig.s, &in,
                                            t + 500U + WHEEL_HOME_SETTLE_MS) == 0U);
}

/* Applied nonzero PWM with no improvement for 500 ms stops every wheel. */
static void test_stall(void)
{
  uint32_t elapsed = 0U;

  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;
  short_press();
  assert(rig.o.pulse[0] < 0);

  while ((uint32_t)(rig.now - g_home_start) < 700U &&
         rig.o.result != WHEEL_HOME_RESULT_STALL)
  {
    step4(10U, 0, 0, 0, 0);
  }

  elapsed = (uint32_t)(rig.now - g_home_start);
  assert(rig.o.result == WHEEL_HOME_RESULT_STALL);
  assert(elapsed >= WHEEL_HOME_NO_PROGRESS_MS);
  assert(elapsed <= WHEEL_HOME_NO_PROGRESS_MS + 30U);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }
  assert(rig.o.mode == WHEEL_HOME_WAIT_NEUTRAL);
}

/* Slow but real progress still trips the 8 s total timeout. */
static void test_timeout(void)
{
  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;
  short_press();
  assert(rig.o.pulse[0] < 0);

  while ((uint32_t)(rig.now - g_home_start) < WHEEL_HOME_TIMEOUT_MS)
  {
    assert(rig.o.result != WHEEL_HOME_RESULT_STALL);
    step4(10U, -13, 0, 0, 0);
    if ((uint32_t)(rig.now - g_home_start) < WHEEL_HOME_TIMEOUT_MS)
    {
      assert(rig.o.result != WHEEL_HOME_RESULT_TIMEOUT);
    }
  }

  assert(rig.o.result == WHEEL_HOME_RESULT_TIMEOUT);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }
}

/* HAL tick rollover must not look like a timeout, stall or settle restart. */
static void test_tick_wrap(void)
{
  rig_reset(CPR);
  rig.now = 0xFFFFFFFFU - 300U;

  rig.manual_calib_page = 1U;
  press_select();
  pump(2050U);
  assert(rig.o.result == WHEEL_HOME_RESULT_RECORDED);
  release_select();
  pump(100U);
  rig.manual_calib_page = 0U;

  rig.pos[0] += (int64_t)CPR[0] / 2;
  short_press();
  assert(rig.o.pulse[0] < 0);

  for (int guard = 0; guard < 400; guard++)
  {
    int64_t err = rig.s.target[0] - rig.pos[0];
    int32_t step;
    if (err > -((int64_t)CPR[0] / 200) && err < ((int64_t)CPR[0] / 200)) { break; }
    step = (err < 0) ? -10000 : 10000;
    if (err > -10000 && err < 10000) { step = (int32_t)err; }
    step4(10U, step, 0, 0, 0);
  }
  pump(240U);

  assert(rig.o.result == WHEEL_HOME_RESULT_DONE);
}

/* 20 ms 按下未满 30 ms 去抖：不启动归位，也不改变状态。 */
static void test_press_debounce(void)
{
  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;
  press_select();
  pump(20U);
  release_select();
  pump(60U);

  assert(rig.o.mode == WHEEL_HOME_MANUAL);
  assert(rig.o.result == WHEEL_HOME_RESULT_RECORDED);
  assert(rig.o.owns_motors == 0U);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }
}

/* 短按释放只触发一次，重复帧不再重设目标。 */
static void test_short_press_once(void)
{
  int64_t target0;

  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;
  short_press();
  target0 = rig.s.target[0];
  assert(rig.o.pulse[0] < 0);

  pump(40U);
  assert(rig.s.target[0] == target0);
  assert(rig.o.mode == WHEEL_HOME_RUNNING);
}

/* 长按只记录一次，释放不归位，也不清零 UI 累计净圈数。 */
static void test_long_press_records_once(void)
{
  rig_reset(CPR);
  rig.manual_calib_page = 1U;
  press_select();
  pump(2050U);

  assert(rig.o.result == WHEEL_HOME_RESULT_RECORDED);
  assert(rig.o.reference_valid == 1U);
  assert(rig.s.origin[0] == rig.pos[0]);
  pump(500U);
  assert(rig.o.result == WHEEL_HOME_RESULT_RECORDED);

  release_select();
  pump(120U);
  assert(rig.o.mode == WHEEL_HOME_MANUAL);
  assert(rig.o.owns_motors == 0U);
  assert(wheel_feedback_zero_request == 0U);
}

/* 没有参考时短按不转任何轮子。 */
static void test_short_press_without_reference(void)
{
  rig_reset(CPR);
  tap_select();

  assert(rig.o.result == WHEEL_HOME_RESULT_NO_REF);
  assert(rig.o.mode == WHEEL_HOME_MANUAL);
  assert(rig.o.reference_valid == 0U);
  assert(rig.o.owns_motors == 0U);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }
}

/* 长按两秒时静止不足 300 ms：不记录，且不延后、不退化成短按。 */
static void test_record_requires_still(void)
{
  rig_reset(CPR);
  press_select();
  pump(1900U);
  step4(10U, 5000, 5000, 5000, 5000);
  pump(100U);

  assert(rig.o.reference_valid == 0U);
  assert(rig.o.result != WHEEL_HOME_RESULT_RECORDED);

  pump(400U); /* 之后的静止时间再长也不补记 */
  assert(rig.o.reference_valid == 0U);

  release_select();
  pump(60U);
  assert(rig.o.mode == WHEEL_HOME_MANUAL);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }
}

/* 按下 SELECT 当周期立即停止归位，不能落回摇杆驱动。 */
static void test_press_stops_motion(void)
{
  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;
  short_press();
  assert(rig.o.pulse[0] < 0);

  press_select();
  step4(10U, 20000, 0, 0, 0);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }
  assert(rig.o.mode == WHEEL_HOME_WAIT_NEUTRAL);
  assert(rig.o.result == WHEEL_HOME_RESULT_CANCELLED);
  assert(rig.o.owns_motors == 1U);
}

/* 摇杆超出 ±16 取消；释放并回中持续 100 ms 才恢复手动。 */
static void test_stick_cancels(void)
{
  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;
  short_press();
  assert(rig.o.pulse[0] < 0);

  rig.ly = (uint8_t)(WHEEL_HOME_STICK_CENTER + 17);
  step4(10U, 0, 0, 0, 0);
  assert(rig.o.result == WHEEL_HOME_RESULT_CANCELLED);
  assert(rig.o.owns_motors == 1U);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }

  rig.ly = WHEEL_HOME_STICK_CENTER;
  pump(60U);
  assert(rig.o.mode == WHEEL_HOME_WAIT_NEUTRAL);
  assert(rig.o.owns_motors == 1U);
  pump(100U);
  assert(rig.o.mode == WHEEL_HOME_MANUAL);
  assert(rig.o.owns_motors == 0U);
}

/* 标定模式取消归位并保持零输出，但不使参考失效。 */
static void test_calibration_cancels(void)
{
  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;
  short_press();
  assert(rig.o.pulse[0] < 0);

  rig.calibration_mode = 1U;
  pump(50U);
  assert(rig.o.result == WHEEL_HOME_RESULT_CANCELLED);
  assert(rig.o.mode == WHEEL_HOME_WAIT_NEUTRAL);
  assert(rig.o.reference_valid == 1U);
  assert(rig.o.owns_motors == 1U);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }

  rig.calibration_mode = 0U;
  pump(150U);
  assert(rig.o.mode == WHEEL_HOME_MANUAL);
}

/* 有效帧中断 200 ms 取消归位并四轮置零。 */
static void test_frame_loss_cancels(void)
{
  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;
  short_press();
  assert(rig.o.pulse[0] < 0);

  rig.frame_valid = 0U;
  for (int i = 0; i < 19; i++) { step4(10U, 0, 0, 0, 0); }
  assert(rig.o.mode == WHEEL_HOME_RUNNING);

  for (int i = 0; i < 3; i++) { step4(10U, 0, 0, 0, 0); }
  assert(rig.o.mode == WHEEL_HOME_WAIT_NEUTRAL);
  assert(rig.o.result == WHEEL_HOME_RESULT_CANCELLED);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }
}

/* 重连时 SELECT 仍按住：先要求看到释放，才重新接受操作。 */
static void test_reconnect_requires_release(void)
{
  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;
  short_press();

  rig.frame_valid = 0U;
  pump(250U);
  assert(rig.o.mode == WHEEL_HOME_WAIT_NEUTRAL);

  press_select();
  rig.frame_valid = 1U;
  pump(200U);
  assert(rig.o.mode == WHEEL_HOME_WAIT_NEUTRAL);
  assert(rig.o.result == WHEEL_HOME_RESULT_CANCELLED);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }

  release_select();
  pump(200U);
  assert(rig.o.mode == WHEEL_HOME_MANUAL);

  press_select();
  pump(50U);
  assert(rig.o.mode == WHEEL_HOME_BUTTON_HELD);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }
  assert(rig.o.owns_motors == 1U);
}

/* 单个无效帧既不推进去抖，也不能伪造释放边沿。 */
static void test_single_bad_frame_no_edge(void)
{
  rig_reset(CPR);
  record_reference();

  press_select();
  pump(50U);
  assert(rig.o.mode == WHEEL_HOME_BUTTON_HELD);

  rig.frame_valid = 0U;
  step4(10U, 0, 0, 0, 0);
  assert(rig.o.mode == WHEEL_HOME_BUTTON_HELD);

  rig.frame_valid = 1U;
  pump(50U);
  assert(rig.o.mode == WHEEL_HOME_BUTTON_HELD);
  assert(rig.o.result != WHEEL_HOME_RESULT_NO_REF);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }
  assert(rig.o.owns_motors == 1U);
}


/* ---------- Task 7: 菜单请求归位与手动标定页作用域 ---------- */

/* 菜单 L1 提交归位请求后：按住 L1 不得进 RUNNING；
 * 释放且回中满 100 ms 后才进 RUNNING，且同一次提交的释放不算取消。 */
static void test_menu_request_waits_release_neutral(void)
{
  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;
  hold_l1();
  wheel_home_request_home(&rig.s);

  pump(90U);
  assert(rig.o.mode == WHEEL_HOME_MANUAL);
  assert(rig.o.result != WHEEL_HOME_RESULT_CANCELLED);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }

  release_l1();
  pump(90U);
  assert(rig.o.mode == WHEEL_HOME_MANUAL);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }

  pump(20U);
  assert(rig.o.mode == WHEEL_HOME_RUNNING);
  assert(rig.o.result != WHEEL_HOME_RESULT_CANCELLED);
  assert(rig.o.pulse[0] < 0);
}

/* 无参考时菜单请求直接以 NO_REF 落地：不动轮、不接管电机。 */
static void test_menu_request_without_reference(void)
{
  rig_reset(CPR);

  hold_l1();
  wheel_home_request_home(&rig.s);
  pump(10U);

  assert(rig.o.result == WHEEL_HOME_RESULT_NO_REF);
  assert(rig.o.mode == WHEEL_HOME_MANUAL);
  assert(rig.o.reference_valid == 0U);
  assert(rig.o.owns_motors == 0U);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }

  /* 之后也不会突然起转。 */
  release_l1();
  pump(150U);
  assert(rig.o.mode == WHEEL_HOME_MANUAL);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }
}

/* 菜单请求启动的归位仍受既有安全规则约束：摇杆越死区即取消。 */
static void test_menu_homing_stick_cancels(void)
{
  rig_reset(CPR);
  record_reference();

  rig.pos[0] += (int64_t)CPR[0] / 2;
  hold_l1();
  wheel_home_request_home(&rig.s);
  release_l1();
  pump(110U);
  assert(rig.o.mode == WHEEL_HOME_RUNNING);
  assert(rig.o.pulse[0] < 0);

  rig.ly = (uint8_t)(WHEEL_HOME_STICK_CENTER + 17);
  step4(10U, 0, 0, 0, 0);
  assert(rig.o.result == WHEEL_HOME_RESULT_CANCELLED);
  assert(rig.o.mode == WHEEL_HOME_WAIT_NEUTRAL);
  for (int w = 0; w < 4; w++) { assert(rig.o.pulse[w] == 0); }
}

/* 长按记录参考只属于手动标定页：驾驶页长按不再写参考。 */
static void test_long_press_record_gated_by_manual_page(void)
{
  rig_reset(CPR);

  /* 驾驶页（manual_calib_page == 0）：长按 2 s 不写参考。 */
  press_select();
  pump(2050U);
  assert(rig.o.reference_valid == 0U);
  assert(rig.o.result != WHEEL_HOME_RESULT_RECORDED);
  release_select();
  pump(60U);
  assert(rig.o.mode == WHEEL_HOME_MANUAL);

  /* 手动标定页（manual_calib_page == 1）：同样的长按写参考。 */
  rig.manual_calib_page = 1U;
  press_select();
  pump(2050U);
  assert(rig.o.result == WHEEL_HOME_RESULT_RECORDED);
  assert(rig.o.reference_valid == 1U);
  for (int w = 0; w < 4; w++) { assert(rig.s.origin[w] == rig.pos[w]); }
  release_select();
  pump(60U);
}

/* 顶栏状态文本：每种状态/结果都有确定文本，且绝不越过缓冲区。 */
static void test_status_text(void)
{
  static const struct
  {
    wheel_home_mode_t mode;
    wheel_home_result_t result;
    uint8_t ref;
    const char *want;
  } cases[] =
  {
    { WHEEL_HOME_MANUAL,       WHEEL_HOME_RESULT_NONE,     1U, "RDY"  },
    { WHEEL_HOME_MANUAL,       WHEEL_HOME_RESULT_NONE,     0U, "NORF" },
    { WHEEL_HOME_BUTTON_HELD,  WHEEL_HOME_RESULT_NONE,     1U, "HOLD" },
    { WHEEL_HOME_RUNNING,      WHEEL_HOME_RESULT_NONE,     1U, "HOME" },
    { WHEEL_HOME_WAIT_NEUTRAL, WHEEL_HOME_RESULT_RECORDED, 1U, "REC"  },
    { WHEEL_HOME_WAIT_NEUTRAL, WHEEL_HOME_RESULT_DONE,     1U, "DONE" },
    { WHEEL_HOME_WAIT_NEUTRAL, WHEEL_HOME_RESULT_CANCELLED,1U, "CANC" },
    { WHEEL_HOME_WAIT_NEUTRAL, WHEEL_HOME_RESULT_NO_REF,   0U, "NORF" },
    { WHEEL_HOME_WAIT_NEUTRAL, WHEEL_HOME_RESULT_STALL,    1U, "STAL" },
    { WHEEL_HOME_WAIT_NEUTRAL, WHEEL_HOME_RESULT_TIMEOUT,  1U, "TOUT" },
    { WHEEL_HOME_WAIT_NEUTRAL, WHEEL_HOME_RESULT_FEEDBACK_ERROR, 0U, "FERR" },
    { WHEEL_HOME_WAIT_NEUTRAL, WHEEL_HOME_RESULT_NONE,     0U, "WAIT" },
  };
  size_t i;

  for (i = 0U; i < sizeof cases / sizeof cases[0]; i++)
  {
    wheel_home_output_t o;
    char buf[WHEEL_HOME_STATUS_TEXT_MAX];

    memset(&o, 0, sizeof o);
    o.mode = cases[i].mode;
    o.result = cases[i].result;
    o.reference_valid = cases[i].ref;
    memset(buf, 'X', sizeof buf);
    wheel_home_status_text(buf, (uint32_t)sizeof buf, &o);
    assert(strcmp(buf, cases[i].want) == 0);
    /* 顶栏只有 7 个字符的格子，超出就压到 CAL 指示。 */
    assert(strlen(buf) <= WHEEL_HOME_STATUS_TEXT_MAX - 1U);
  }

  /* 锁存故障优先于一切普通状态：RUNNING 时也要显示 FLT。 */
  {
    wheel_home_output_t o;
    char buf[WHEEL_HOME_STATUS_TEXT_MAX + 4U];

    memset(&o, 0, sizeof o);
    o.mode = WHEEL_HOME_RUNNING;
    o.result = WHEEL_HOME_RESULT_DONE;
    o.fault_mask = 0x04U;
    wheel_home_status_text(buf, sizeof buf, &o);
    assert(strcmp(buf, "FLT") == 0);

    /* 逐轮仍可见：mask 不是全零就足够。 */
    o.mode = WHEEL_HOME_MANUAL;
    o.reference_valid = 1U;
    o.fault_mask = 0x0FU;
    wheel_home_status_text(buf, sizeof buf, &o);
    assert(strcmp(buf, "FLT") == 0);

    /* 清零后回到普通状态文本。 */
    o.fault_mask = 0U;
    wheel_home_status_text(buf, sizeof buf, &o);
    assert(strcmp(buf, "RDY") == 0);
  }

  /* 缓冲区边界：size 为 0/1 不写越界，截断后必须补 '\0'。 */
  {
    wheel_home_output_t o;
    char buf[WHEEL_HOME_STATUS_TEXT_MAX + 4U];

    memset(&o, 0, sizeof o);
    o.mode = WHEEL_HOME_WAIT_NEUTRAL;
    o.result = WHEEL_HOME_RESULT_DONE;

    memset(buf, 'X', sizeof buf);
    wheel_home_status_text(buf, 0U, &o);
    assert(buf[0] == 'X');

    memset(buf, 'X', sizeof buf);
    wheel_home_status_text(buf, 1U, &o);
    assert(buf[0] == '\0');
    assert(buf[1] == 'X');

    memset(buf, 'X', sizeof buf);
    wheel_home_status_text(buf, 3U, &o);
    assert(strcmp(buf, "DO") == 0);
    assert(buf[3] == 'X');

    memset(buf, 'X', sizeof buf);
    wheel_home_status_text(buf, (uint32_t)sizeof buf, &o);
    assert(strcmp(buf, "DONE") == 0);
    assert(buf[5] == 'X');

    /* 空指针不崩，也不写。 */
    wheel_home_status_text(0, 4U, &o);
    memset(buf, 'X', sizeof buf);
    wheel_home_status_text(buf, sizeof buf, 0);
    assert(strcmp(buf, "WAIT") == 0);
  }
}

int main(void)
{
  test_displacement();
  test_zero_cpr_rejected();
  test_proportional_per_wheel();
  test_tolerance_uses_each_wheel_cpr();
  test_settle_then_done();
  test_reverse_wait();
  test_overshoot_outside_tolerance();
  test_stall();
  test_timeout();
  test_sample_timeout_recovers();
  test_tolerance_keeps_last_direction();
  test_tick_wrap();
  test_press_debounce();
  test_short_press_once();
  test_long_press_records_once();
  test_short_press_without_reference();
  test_record_requires_still();
  test_manual_record_checks_preconditions();
  test_press_stops_motion();
  test_stick_cancels();
  test_calibration_cancels();
  test_frame_loss_cancels();
  test_reconnect_requires_release();
  test_single_bad_frame_no_edge();
  test_menu_request_waits_release_neutral();
  test_menu_request_without_reference();
  test_menu_homing_stick_cancels();
  test_long_press_record_gated_by_manual_page();
  test_status_text();
  puts("wheel home gesture / arbitration / shortest path / proportional / settle / reverse / stall / timeout OK");
  return 0;
}
