/* Ctrl 推进的逐轮自检服务：纯状态机，PWM/DIR 语义只落到 command[]。
 * 反馈用"上一拍 command"推导（fb 滞后一拍），段边界用注入 now 精确推进。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../Core/Inc/selfcheck_service.h"
#include "../Core/Inc/motor_test.h"

/* drive_calibration_mode 定义（drive_ctrl.h 只给 extern，host 测试在此落地） */
volatile uint8_t drive_calibration_mode = 0U;

/* 每轮反馈策略 */
typedef enum
{
  FB_FOLLOW = 0,   /* 跟随命令方向，正常轮 */
  FB_NONE,         /* 完全无反馈 */
  FB_NEG_ALWAYS,   /* 一动就向后转：前转段计数为负 → FWD ERR（反向段正常） */
  FB_POS_ALWAYS    /* 一动就向前转：反转段计数为正 → REV ERR（前转段正常） */
} fb_mode_t;
static fb_mode_t g_fb[4];

#define FB_STEP 50 /* 每 10 ms 命令下的计数增量 */

static void fb_from_cmd(const int32_t cmd[4], int32_t fb[4])
{
  uint8_t w;
  for (w = 0; w < 4U; w++)
  {
    int32_t want = cmd[w] > 0 ? FB_STEP : (cmd[w] < 0 ? -FB_STEP : 0);
    switch (g_fb[w])
    {
    case FB_FOLLOW:
      fb[w] = want;
      break;
    case FB_NEG_ALWAYS: /* 只要在动就向后转 */
      fb[w] = want != 0 ? -FB_STEP : 0;
      break;
    case FB_POS_ALWAYS: /* 只要在动就向前转 */
      fb[w] = want != 0 ? FB_STEP : 0;
      break;
    default: /* FB_NONE */
      fb[w] = 0;
      break;
    }
  }
}

/* 推进 n 个 10 ms 节拍；last_cmd 记录上一拍 command（供下一拍 fb 推导） */
static void pump(selfcheck_service_t *s, uint32_t *now, uint32_t steps,
                 int32_t last_cmd[4])
{
  uint32_t i;
  for (i = 0U; i < steps; i++)
  {
    int32_t fb[4] = {0, 0, 0, 0};
    int32_t cmd[4];
    fb_from_cmd(last_cmd, fb);
    *now += 10U;
    selfcheck_service_step(s, (const int32_t *)fb, *now, cmd);
    memcpy(last_cmd, cmd, sizeof cmd);
  }
}

static uint32_t g_done_at;
static uint8_t run_to_done(selfcheck_service_t *s, uint32_t *now,
                           int32_t last_cmd[4])
{
  uint32_t i;
  for (i = 0U; i < 1000U; i++) /* 上限 10 s，正常 5.6 s 收敛 */
  {
    int32_t fb[4] = {0, 0, 0, 0};
    int32_t cmd[4];
    fb_from_cmd(last_cmd, fb);
    *now += 10U;
    selfcheck_service_step(s, (const int32_t *)fb, *now, cmd);
    memcpy(last_cmd, cmd, sizeof cmd);
    if (s->phase == SELFCHECK_DONE)
    {
      g_done_at = *now;
      return 1U;
    }
  }
  return 0U;
}

static void all_fb(fb_mode_t m)
{
  uint8_t w;
  for (w = 0; w < 4U; w++)
  {
    g_fb[w] = m;
  }
}

int main(void)
{
  /* ---- 1. 正常轮：时序与 command 断言 ---- */
  {
    selfcheck_service_t s;
    uint32_t now = 1000U, start;
    int32_t cmd[4] = {0, 0, 0, 0};
    uint8_t w;

    selfcheck_service_init(&s);
    assert(s.phase == SELFCHECK_IDLE);
    assert(s.passed == 0U);
    assert(s.seq == 0U);
    /* IDLE 下 step：command 恒 0，状态不变 */
    selfcheck_service_step(&s, (const int32_t *)(int32_t[4]){0, 0, 0, 0}, now, cmd);
    assert(s.phase == SELFCHECK_IDLE);
    for (w = 0; w < 4U; w++)
    {
      assert(cmd[w] == 0);
    }

    all_fb(FB_FOLLOW);
    selfcheck_service_request(&s, now);
    assert(s.phase == SELFCHECK_RUNNING);
    assert(s.wheel == 0U && s.subphase == 0U);
    assert(s.state[0] == 1U && s.state[1] == 0U && s.state[2] == 0U &&
           s.state[3] == 0U);
    start = now;

    pump(&s, &now, 1U, cmd); /* now=1010：A 前转 */
    assert(cmd[0] == (int32_t)SELFCHECK_PWM_PULSE);
    assert(cmd[1] == 0 && cmd[2] == 0 && cmd[3] == 0);

    pump(&s, &now, 48U, cmd); /* now=1490：前转段仍在 */
    assert(cmd[0] == (int32_t)SELFCHECK_PWM_PULSE);

    pump(&s, &now, 1U, cmd); /* now=1500：500ms 到 → 停 */
    assert(cmd[0] == 0);

    pump(&s, &now, 19U, cmd); /* now=1690：暂停段 */
    assert(cmd[0] == 0);

    pump(&s, &now, 1U, cmd); /* now=1700：反转段 */
    assert(cmd[0] == -(int32_t)SELFCHECK_PWM_PULSE);

    pump(&s, &now, 49U, cmd); /* now=2190：反转段持续 */
    assert(cmd[0] == -(int32_t)SELFCHECK_PWM_PULSE);

    pump(&s, &now, 1U, cmd); /* now=2200：反转 500ms 到 → 停 */
    assert(cmd[0] == 0);

    pump(&s, &now, 19U, cmd); /* now=2390：尾暂停 */
    assert(cmd[0] == 0);

    pump(&s, &now, 1U, cmd); /* now=2400：轮 B 开跑 */
    assert(cmd[1] == (int32_t)SELFCHECK_PWM_PULSE);
    assert(cmd[0] == 0 && cmd[2] == 0 && cmd[3] == 0);
    assert(s.wheel == 1U && s.subphase == 0U);
    assert(s.state[0] == 2U && s.state[1] == 1U);

    /* RUNNING 中重复 request 必须被忽略（不打断时序） */
    selfcheck_service_request(&s, now);
    assert(s.wheel == 1U);

    /* 全程名义时长 8×(500+200)=5600 ms ±10 ms */
    assert(run_to_done(&s, &now, cmd) == 1U);
    {
      uint32_t span = g_done_at - start;
      assert(span >= 5590U && span <= 5610U);
    }
    assert(s.phase == SELFCHECK_DONE);
    assert(s.passed == 1U);
    assert(s.seq == 1U);
    for (w = 0; w < 4U; w++)
    {
      assert(s.state[w] == 2U);
      assert(s.result[w].tested == 1U);
      assert(s.result[w].forward_counts > 0);
      assert(s.result[w].reverse_counts < 0);
      assert(strcmp(motor_test_reason_text(&s.result[w]), "OK") == 0);
    }
  }

  /* ---- 2. 故障轮：缺反馈 NO FB / 反向 FWD ERR / REV ERR ---- */
  {
    selfcheck_service_t s;
    uint32_t now = 0U;
    int32_t cmd[4] = {0, 0, 0, 0};

    selfcheck_service_init(&s);
    g_fb[0] = FB_FOLLOW;     /* A 正常 */
    g_fb[1] = FB_NEG_ALWAYS; /* B 前转段倒转（计数反号）→ FWD ERR */
    g_fb[2] = FB_POS_ALWAYS; /* C 反转段倒转（计数反号）→ REV ERR */
    g_fb[3] = FB_NONE;       /* D 完全无反馈 → NO FB */
    selfcheck_service_request(&s, now);
    assert(run_to_done(&s, &now, cmd) == 1U);

    assert(s.phase == SELFCHECK_DONE);
    assert(s.seq == 1U); /* 跑完即 +1，与通过与否无关 */
    assert(s.passed == 0U);
    assert(s.state[0] == 2U);
    assert(s.state[1] == 3U && s.state[2] == 3U && s.state[3] == 3U);
    assert(strcmp(motor_test_reason_text(&s.result[0]), "OK") == 0);
    assert(strcmp(motor_test_reason_text(&s.result[1]), "FWD ERR") == 0);
    assert(s.result[1].forward_counts < 0); /* 原始带符号计数保留 */
    assert(s.result[1].reverse_counts < 0); /* 反向段正常，仍为负 */
    assert(strcmp(motor_test_reason_text(&s.result[2]), "REV ERR") == 0);
    assert(s.result[2].forward_counts > 0);
    assert(s.result[2].reverse_counts > 0); /* 反转段倒转，计数为正 */
    assert(strcmp(motor_test_reason_text(&s.result[3]), "NO FB") == 0);
    assert(s.result[3].forward_counts == 0);
    assert(s.result[3].reverse_counts == 0);
  }

  /* ---- 3. 标定模式：全轮 SKIP、state 保持等待、command 恒 0 ---- */
  {
    selfcheck_service_t s;
    uint32_t now = 5000U;
    int32_t cmd[4] = {0, 0, 0, 0};
    uint8_t w;

    selfcheck_service_init(&s);
    all_fb(FB_FOLLOW);
    drive_calibration_mode = 1U;
    selfcheck_service_request(&s, now);
    pump(&s, &now, 100U, cmd); /* 1 s，远超一段时长 */
    drive_calibration_mode = 0U;
    assert(s.phase == SELFCHECK_DONE);
    assert(s.passed == 0U);
    for (w = 0; w < 4U; w++)
    {
      assert(cmd[w] == 0);
      assert(s.state[w] == 0U); /* 保持等待 */
      assert(strcmp(motor_test_reason_text(&s.result[w]), "SKIP") == 0);
    }
  }

  /* ---- 4. 取消：已完成轮保留，未完成轮非通过 ---- */
  {
    selfcheck_service_t s;
    uint32_t now = 0U;
    int32_t cmd[4] = {0, 0, 0, 0};
    uint8_t w;

    selfcheck_service_init(&s);
    all_fb(FB_FOLLOW);
    selfcheck_service_request(&s, now);
    pump(&s, &now, 145U, cmd); /* now=1450：A 已过、B 运行中 */
    assert(s.state[0] == 2U && s.state[1] == 1U);
    selfcheck_service_cancel(&s);
    assert(s.phase == SELFCHECK_DONE);
    assert(s.passed == 0U);
    assert(s.seq == 0U); /* 没跑完不加 seq */
    pump(&s, &now, 20U, cmd);
    for (w = 0; w < 4U; w++)
    {
      assert(cmd[w] == 0); /* 取消后 command 恒 0 */
    }
    /* 已完成轮的 state/结果保留（结果页可回看） */
    assert(s.state[0] == 2U);
    assert(s.result[0].tested == 1U);
    assert(motor_test_passed(&s.result[0]) == 1U);
    assert(strcmp(motor_test_reason_text(&s.result[0]), "OK") == 0);
    /* 运行中的 B 不再显示"运行" */
    assert(s.state[1] != 1U);
    assert(motor_test_passed(&s.result[1]) == 0U);
  }

  /* ---- 5. 时序回绕：now 从 UINT32_MAX−1000 起跑完整轮 ---- */
  {
    selfcheck_service_t s;
    uint32_t now = 0xFFFFFFFFU - 1000U, start;
    int32_t cmd[4] = {0, 0, 0, 0};
    uint8_t w;

    selfcheck_service_init(&s);
    all_fb(FB_FOLLOW);
    selfcheck_service_request(&s, now);
    start = now;
    assert(run_to_done(&s, &now, cmd) == 1U);
    assert(s.phase == SELFCHECK_DONE);
    assert(s.passed == 1U);
    assert(s.seq == 1U);
    for (w = 0; w < 4U; w++)
    {
      assert(s.state[w] == 2U);
    }
    {
      uint32_t span = g_done_at - start; /* 无符号回绕差值 */
      assert(span >= 5590U && span <= 5610U);
    }
  }

  /* ---- 6. seq 每次完整跑只 +1；passed 仅全过 ---- */
  {
    selfcheck_service_t s;
    uint32_t now = 0U;
    int32_t cmd[4] = {0, 0, 0, 0};

    selfcheck_service_init(&s);
    all_fb(FB_FOLLOW);
    selfcheck_service_request(&s, now);
    assert(run_to_done(&s, &now, cmd) == 1U);
    assert(s.seq == 1U && s.passed == 1U);
    /* DONE 后可再次请求，seq 再 +1 */
    selfcheck_service_request(&s, now);
    assert(s.phase == SELFCHECK_RUNNING && s.wheel == 0U);
    assert(run_to_done(&s, &now, cmd) == 1U);
    assert(s.seq == 2U && s.passed == 1U);
    /* 失败跑：seq 照常 +1，passed 归 0 */
    g_fb[3] = FB_NONE;
    selfcheck_service_request(&s, now);
    assert(run_to_done(&s, &now, cmd) == 1U);
    assert(s.seq == 3U && s.passed == 0U);
  }

  puts("selfcheck service wheel sequence / timing / cancel / wrap OK");
  return 0;
}
