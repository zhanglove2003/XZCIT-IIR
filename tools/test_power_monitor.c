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
static int s_vref_start_fail; /* 仅让 VREFINT 批次失败，CH8 照常成功 */
static int s_ch8_poll_fail;   /* 仅让 CH8 转换失败，VREFINT 照常成功 */
static int s_vref_batches; /* ConfigChannel(VREFINT) 次数 = vrefint 采样批数 */
static int s_ch8_gets;     /* CH8 实际取数次数 */

/* 审查项 #3：step 进行中的已发布快照捕获（撕䉤断言用）。
 * 置位后第一次走到 CH8 失败的 PollForConversion 时拍下 power_monitor_get()。 */
static uint8_t s_capture_mid;
static power_snapshot_t s_mid_snap;

int HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c)
{
  (void)h;
  if (s_cfg_fail != 0) { return 1; }
  s_cur_ch = c->Channel;
  if (c->Channel == ADC_CHANNEL_VREFINT) { s_vref_batches++; }
  return 0;
}
int HAL_ADC_Start(ADC_HandleTypeDef *h)
{
  (void)h;
  if (s_start_fail) { return 1; }
  if (s_vref_start_fail && s_cur_ch == ADC_CHANNEL_VREFINT) { return 1; }
  return 0;
}
int HAL_ADC_PollForConversion(ADC_HandleTypeDef *h, uint32_t timeout_ms)
{
  (void)h; (void)timeout_ms;
  if (s_poll_fail) { return 1; }
  if (s_ch8_poll_fail && s_cur_ch != ADC_CHANNEL_VREFINT)
  {
    if (s_capture_mid != 0U)
    {
      s_mid_snap = *power_monitor_get(); /* 采样中途的已发布快照 */
      s_capture_mid = 0U;
    }
    return 1;
  }
  return 0;
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

  /* 6. 配置后 CH8 采样失败：battery_error 置位、undervolt 不误报，恢复后清零 */
  s_ch8_poll_fail = 1;
  power_monitor_step(3002U);
  s = power_monitor_get();
  assert(s->vdd_valid == 1U); /* VREFINT 不受影响 */
  assert(s->battery_valid == 0U);
  assert(s->battery_error == 1U);
  assert(s->undervolt == 0U);
  s_ch8_poll_fail = 0;
  power_monitor_step(3503U);
  s = power_monitor_get();
  assert(s->battery_valid == 1U);
  assert(s->battery_error == 0U);

  /* 7. VREFINT 失败 + CH8 成功的组合：跳过电池换算，
   *    不得出现 battery_valid==1 且 battery_mv==0 的矛盾快照 */
  s_vref_start_fail = 1;
  power_monitor_step(4004U);
  s = power_monitor_get();
  assert(s->vdd_valid == 0U);
  assert(s->battery_valid == 0U);
  assert(s->battery_mv == 0U);
  assert(s->undervolt == 0U);
  assert(s->battery_error == 0U); /* CH8 未被采样，不算采样失败 */
  s_vref_start_fail = 0;
  power_monitor_step(4505U);
  s = power_monitor_get();
  assert(s->vdd_valid == 1U);
  assert(s->battery_valid == 1U);
  assert(s->battery_mv == 3300U);
  assert(s->undervolt == 0U);

  /* 8. vdd_ok：采样成功还必须 >= 3100 mV 才许运动/自检
   *    （3300*1490/1639 = 3000 mV 掉压；3300*1490/1585 = 3102 mV 达标） */
  power_monitor_init();
  s_test_configured = 0U;
  s_vref_val = 1639U;
  power_monitor_step(5000U);
  s = power_monitor_get();
  assert(s->vdd_valid == 1U && s->vdd_mv == 3000U);
  assert(s->vdd_ok == 0U);
  s_vref_val = 1585U;
  power_monitor_step(5501U);
  s = power_monitor_get();
  assert(s->vdd_mv == 3102U);
  assert(s->vdd_ok == 1U);

  /* 9. 发布原子性（审查项 #3：电源快照撕裂）：step 进行中（CH8 采样失败
   *    的 PollForConversion 里）读到的已发布快照必须仍是上一次完整快照，
   *    不得出现"vdd 新、battery 旧"的半提交混搭；发布后各字段自洽。 */
  power_monitor_init();
  s_test_configured = 1U;
  s_test_min_mv = 3300U;
  s_vref_val = 1490U;  /* vdd = 3300 mV */
  s_ch8_val = 0U;      /* battery_mv = 0 → undervolt = 1 */
  s_ch8_poll_fail = 0;
  power_monitor_step(6000U);
  s = power_monitor_get();
  assert(s->vdd_valid == 1U && s->vdd_mv == 3300U && s->vdd_ok == 1U);
  assert(s->battery_valid == 1U && s->battery_mv == 0U);
  assert(s->undervolt == 1U && s->battery_error == 0U);
  {
    power_snapshot_t prev = *s;

    s_vref_val = 1585U;  /* 本轮 vdd 变为 3102 mV */
    s_ch8_poll_fail = 1; /* CH8 失败 */
    s_capture_mid = 1U;
    power_monitor_step(6501U);
    s = power_monitor_get();
    /* 采样中途：已发布快照必须逐字段等于上一次完整快照 */
    assert(s_mid_snap.vdd_valid == prev.vdd_valid);
    assert(s_mid_snap.vdd_mv == prev.vdd_mv);
    assert(s_mid_snap.vdd_ok == prev.vdd_ok);
    assert(s_mid_snap.battery_configured == prev.battery_configured);
    assert(s_mid_snap.battery_valid == prev.battery_valid);
    assert(s_mid_snap.battery_mv == prev.battery_mv);
    assert(s_mid_snap.battery_error == prev.battery_error);
    assert(s_mid_snap.undervolt == prev.undervolt);
    /* 发布后：整体自洽（valid/error/undervolt/mv 成对一致） */
    assert(s->vdd_valid == 1U && s->vdd_mv == 3102U && s->vdd_ok == 1U);
    assert(s->battery_valid == 0U);
    assert(s->battery_error == 1U);
    assert(s->undervolt == 0U);
    assert(s->battery_mv == 0U);
    s_ch8_poll_fail = 0;
  }

  /* 10. 组合回归：vref 成功 + CH8 失败后恢复、未配置清空——发布后的
   *     快照各字段必须成对自洽（valid==1 ⟺ mv 可信；error==1 ⟹ valid==0；
   *     undervolt==1 ⟹ valid==1）。 */
  power_monitor_init();
  s_test_configured = 1U;
  s_test_min_mv = 3300U;
  s_vref_val = 1490U;
  s_ch8_val = 4095U;
  power_monitor_step(7000U);
  s = power_monitor_get();
  assert(s->battery_valid == 1U && s->battery_error == 0U &&
         s->undervolt == 0U && s->battery_mv == 3300U);
  s_ch8_poll_fail = 1;
  power_monitor_step(7501U);
  s = power_monitor_get();
  assert(!(s->battery_valid == 1U) ); /* error 路径 valid 必须为 0 */
  assert(s->battery_error == 1U && s->undervolt == 0U);
  s_test_configured = 0U;
  s_ch8_poll_fail = 0;
  power_monitor_step(8002U);
  s = power_monitor_get();
  assert(s->battery_configured == 0U);
  assert(s->battery_valid == 0U && s->battery_mv == 0U &&
         s->battery_error == 0U && s->undervolt == 0U);

  printf("power_monitor host test passed\n");
  return 0;
}
