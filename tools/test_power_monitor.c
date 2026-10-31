/* power_monitor 纯逻辑主机测试。
 * ADC 行为全部通过 HAL_ADC_* 桩控制：vrefint 固定返回 1490
 * （配合 host_vrefint_cal=1490 得到 vdd=3300 mV），CH8 返回可设值。
 * 电池"已配置/欠压阈值"经弱符号注入（见 power_monitor.c 顶部说明）。 */
#include <assert.h>
#include <stdio.h>

#include "power_monitor.h"
#include "stm32f4xx_hal.h" /* 主机: tools/host_stubs 桩; 提供ADC类型/VREFINT_CAL_ADDR */

/* --- 注入点：覆盖 power_monitor.c 的弱默认实现 --- */
static uint8_t s_test_configured;
static uint32_t s_test_min_mv;
uint8_t power_monitor_battery_configured(void) { return s_test_configured; }
uint32_t power_monitor_battery_min_mv(void) { return s_test_min_mv; }

/* --- HAL 桩 --- */
ADC_HandleTypeDef hadc1;
uint16_t host_vrefint_cal = 1490U;

static uint32_t s_cur_ch;
static uint32_t s_vref_val = 1490U;
static uint32_t s_ch8_val;
static int s_cfg_fail, s_poll_fail, s_start_fail;
static int s_vref_batches; /* ConfigChannel(VREFINT) 次数 = vrefint 采样批数 */
static int s_ch8_gets;     /* CH8 实际取数次数 */

int HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c)
{
  (void)h;
  if (s_cfg_fail != 0) { return 1; }
  s_cur_ch = c->Channel;
  if (c->Channel == ADC_CHANNEL_VREFINT) { s_vref_batches++; }
  return 0;
}
int HAL_ADC_Start(ADC_HandleTypeDef *h) { (void)h; return s_start_fail ? 1 : 0; }
int HAL_ADC_PollForConversion(ADC_HandleTypeDef *h, uint32_t timeout_ms)
{
  (void)h; (void)timeout_ms;
  return s_poll_fail ? 1 : 0;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h)
{
  (void)h;
  if (s_cur_ch == ADC_CHANNEL_VREFINT) { return s_vref_val; }
  s_ch8_gets++;
  return s_ch8_val;
}
int HAL_ADC_Stop(ADC_HandleTypeDef *h) { (void)h; return 0; }

int main(void)
{
  const power_snapshot_t *s;

  /* 1. 纯换算：满量程、截断、大数不溢出（64 位中间值）、防 0 分母 */
  assert(power_monitor_battery_mv(4095U, 3300U) == 3300U);
  assert(power_monitor_battery_mv(2048U, 3300U) == 1650U);
  assert(power_monitor_battery_mv_div(4095U, 3300U, 100U, 1U) == 330000U);
  assert(power_monitor_battery_mv_div(4095U, 3300U, 2U, 1U) == 6600U);
  assert(power_monitor_battery_mv_div(4095U, 3300U, 1U, 0U) == 0U);

  /* 2. 未配置（宏 0）：不采 CH8，电池字段全零，不判欠压 */
  power_monitor_init();
  power_monitor_step(1000U);
  s = power_monitor_get();
  assert(s->vdd_valid == 1U && s->vdd_mv == 3300U);
  assert(s->battery_configured == 0U);
  assert(s->battery_valid == 0U);
  assert(s->battery_mv == 0U);
  assert(s->undervolt == 0U);
  assert(s_ch8_gets == 0);

  /* 3. 节拍：+499 ms 不重采，+501 ms 重采 */
  assert(s_vref_batches == 1);
  power_monitor_step(1499U);
  assert(s_vref_batches == 1);
  power_monitor_step(1501U);
  assert(s_vref_batches == 2);

  /* 4. 回绕点不卡：t0 靠近 2^32 边界 */
  power_monitor_init();
  s_vref_batches = 0;
  power_monitor_step(0xFFFFFF00U);
  assert(s_vref_batches == 1);
  power_monitor_step(0xFFFFFF00U + 499U); /* 回绕后 +499 */
  assert(s_vref_batches == 1);
  power_monitor_step(0xFFFFFF00U + 501U); /* 回绕后 +501 */
  assert(s_vref_batches == 2);

  /* 5. 配置后：欠压判 1，恢复判 0 */
  power_monitor_init();
  s_test_configured = 1U;
  s_test_min_mv = 3300U;
  s_ch8_val = 0U;
  power_monitor_step(2000U);
  s = power_monitor_get();
  assert(s->battery_configured == 1U);
  assert(s->battery_valid == 1U);
  assert(s->battery_mv == 0U);
  assert(s->undervolt == 1U);
  s_ch8_val = 4095U;
  power_monitor_step(2501U);
  s = power_monitor_get();
  assert(s->battery_mv == 3300U);
  assert(s->undervolt == 0U);

  /* 6. 配置后采样失败：battery_error 置位、undervolt 不误报，恢复后清零 */
  s_poll_fail = 1;
  power_monitor_step(3002U);
  s = power_monitor_get();
  assert(s->battery_valid == 0U);
  assert(s->battery_error == 1U);
  assert(s->undervolt == 0U);
  s_poll_fail = 0;
  power_monitor_step(3503U);
  s = power_monitor_get();
  assert(s->battery_valid == 1U);
  assert(s->battery_error == 0U);

  printf("power_monitor host test passed\n");
  return 0;
}
