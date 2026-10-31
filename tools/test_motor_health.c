/* Task 7：运行期运动反馈监测与故障锁存（口径按 plan：PWM>=1200、300 ms 启动宽限、
 * 300 ms 窗口内净计数不足 64 判 NO FB；符号相反且幅度够判 DIR ERR）。 */
#include <assert.h>
#include <stdio.h>

#include "motor_health.h"

static void cyc(motor_health_t *h, uint32_t t, int32_t pwm, int32_t fb,
                uint8_t measurable, motor_health_out_t *out)
{
  motor_health_in_t in;

  for (int i = 0; i < 4; i++) { in.pwm[i] = pwm; in.fb[i] = fb; }
  in.now_ms = t;
  in.measurable = measurable;
  motor_health_step(h, &in, out);
}

int main(void)
{
  motor_health_t h;
  motor_health_out_t out;

  /* 1. 正常正转：窗口内净计数足够，不报错 */
  motor_health_init(&h);
  for (uint32_t t = 0U; t < 2000U; t += 10U) { cyc(&h, t, 2000, 40, 1U, &out); }
  assert(out.fault_mask == 0U);
  assert(out.stop_all == 0U);

  /* 2. 正常反转：同样不报错 */
  motor_health_init(&h);
  for (uint32_t t = 0U; t < 2000U; t += 10U) { cyc(&h, t, -2000, -40, 1U, &out); }
  assert(out.fault_mask == 0U);

  /* 3. 给 PWM 不动：宽限 300 ms 后，300 ms 窗口不足 64 计数 -> NO FB */
  motor_health_init(&h);
  for (uint32_t t = 0U; t < 700U; t += 10U) { cyc(&h, t, 2000, 0, 1U, &out); }
  assert(out.fault_mask == 0x0FU);
  assert(out.stop_all == 1U);

  /* 4. 抖动净零（每拍正负交替）同样不能通过 */
  motor_health_init(&h);
  {
    uint32_t t;
    for (t = 0U; t < 2000U; t += 10U)
    {
      cyc(&h, t, 2000, ((t / 10U) % 2U == 0U) ? 30 : -30, 1U, &out);
    }
  }
  assert(out.fault_mask == 0x0FU);

  /* 5. 反馈符号相反且幅度 >=64 -> DIR ERR（在窗口判定那一轮报出） */
  motor_health_init(&h);
  {
    motor_health_code_t seen = MOTOR_HEALTH_DISABLED;

    for (uint32_t t = 0U; t < 900U; t += 10U)
    {
      cyc(&h, t, 2000, -40, 1U, &out);
      if (out.code[0] == MOTOR_HEALTH_DIR_ERR)
      {
        seen = out.code[0];
      }
    }
    assert(seen == MOTOR_HEALTH_DIR_ERR);
  }
  assert(out.fault_mask == 0x0FU);

  /* 6. 零输出/PWM<1200/睡眠 均不判故障 */
  motor_health_init(&h);
  for (uint32_t t = 0U; t < 3000U; t += 10U) { cyc(&h, t, 0, 0, 1U, &out); }
  assert(out.fault_mask == 0U);
  for (uint32_t t = 0U; t < 3000U; t += 10U) { cyc(&h, t, 1100, 0, 1U, &out); }
  assert(out.fault_mask == 0U);
  for (uint32_t t = 0U; t < 3000U; t += 10U) { cyc(&h, t, 3000, 0, 0U, &out); }
  assert(out.fault_mask == 0U);

  /* 7. 符号改变要重新走 300 ms 宽限：换向后立刻按新方向计数不乱报 */
  motor_health_init(&h);
  for (uint32_t t = 0U; t < 1000U; t += 10U) { cyc(&h, t, 2000, 40, 1U, &out); }
  for (uint32_t t = 1000U; t < 1500U; t += 10U) { cyc(&h, t, -2000, -40, 1U, &out); }
  assert(out.fault_mask == 0U);

  /* 8. 锁存后不能被“看起来正常了”自动解除 */
  motor_health_init(&h);
  for (uint32_t t = 0U; t < 800U; t += 10U) { cyc(&h, t, 2000, 0, 1U, &out); }
  assert(out.fault_mask == 0x0FU);
  for (uint32_t t = 0U; t < 3000U; t += 10U) { cyc(&h, t, 2000, 40, 1U, &out); }
  assert(out.fault_mask == 0x0FU);
  assert(out.stop_all == 1U);

  /* 9. 清除只能来自“用户主动重新自检且四轮通过” */
  motor_health_clear_after_selfcheck(&h, 0U);   /* 自检没过：不许清 */
  assert(h.fault_mask == 0x0FU);
  motor_health_clear_after_selfcheck(&h, 1U);   /* 四轮全过：清 */
  assert(h.fault_mask == 0U);
  cyc(&h, 0U, 2000, 40, 1U, &out);
  assert(out.stop_all == 0U);

  /* 10. tick 回绕：同样的时间关系结果一致 */
  motor_health_init(&h);
  {
    uint32_t t = 0xFFFFFF00U;
    for (uint32_t i = 0U; i < 200U; i++) { cyc(&h, t, 2000, 40, 1U, &out); t += 10U; }
  }
  assert(out.fault_mask == 0U);

  puts("motor health: PWM floor / grace / window / DIR ERR / jitter / latch / clear OK");
  return 0;
}
