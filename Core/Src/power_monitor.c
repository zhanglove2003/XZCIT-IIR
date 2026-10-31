/* power_monitor：低优先级电源监测服务（纯逻辑，500 ms 节拍）。
 *
 * ADC 访问收敛在本文件顶部的两个静态包装里，其余逻辑不直接碰 HAL_ADC_*：
 *   - adc_read_vrefint()：VREFINT 8 次均值，读完恢复 CH8（照抄
 *     boot_ui.c ui_read_vdd_mv() 的次序，保持 ADC 配置与初始化后一致）；
 *   - adc_read_ch8()：电池分压通道单次采样。
 * 主机测试通过桩 HAL_ADC_* 控制两者行为，不需要其他 stm32 HAL 桩。
 *
 * "电池已配置 / 欠压阈值"以弱函数形式读取（发布固件即编译期宏；
 * 主机测试用强符号覆盖注入，验证配置后的欠压路径）。 */
#include "power_monitor.h"

#include "adc.h"
#include "boot_ui.h"
#include "main.h"

#define POWER_STEP_MS          500U
#define POWER_VREFINT_SAMPLES  8U
#define POWER_ADC_FULL_SCALE   4095U
#define POWER_ADC_TIMEOUT_MS   10U

/* ---- 测试注入点（弱默认 = 编译期宏） ---- */
__attribute__((weak)) uint8_t power_monitor_battery_configured(void)
{
  return POWER_BATTERY_CONFIGURED;
}
__attribute__((weak)) uint32_t power_monitor_battery_min_mv(void)
{
  return POWER_BATT_MIN_MV;
}

/* ---- ADC 读取包装（主机测试经 HAL_ADC_* 桩控制） ---- */
static uint8_t adc_read_vrefint(uint32_t *sum, uint8_t *n);
static uint8_t adc_read_ch8(uint32_t *raw);

static uint8_t adc_read_vrefint(uint32_t *sum, uint8_t *n)
{
  ADC_ChannelConfTypeDef cfg = {0};
  uint8_t got = 0U;
  uint8_t i;

  *n = 0U;
  cfg.Channel = ADC_CHANNEL_VREFINT;
  cfg.Rank = 1U;
  cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK)
  {
    return 0U;
  }

  for (i = 0U; i < POWER_VREFINT_SAMPLES; i++)
  {
    if (HAL_ADC_Start(&hadc1) == HAL_OK)
    {
      if (HAL_ADC_PollForConversion(&hadc1, POWER_ADC_TIMEOUT_MS) == HAL_OK)
      {
        *sum += HAL_ADC_GetValue(&hadc1);
        got++;
      }
      HAL_ADC_Stop(&hadc1);
    }
  }

  /* 用完恢复电池分压通道，保持 ADC 状态与初始化后一致 */
  cfg.Channel = ADC_CHANNEL_8;
  (void)HAL_ADC_ConfigChannel(&hadc1, &cfg);

  *n = got;
  return (got == POWER_VREFINT_SAMPLES) ? 1U : 0U;
}

static uint8_t adc_read_ch8(uint32_t *raw)
{
  ADC_ChannelConfTypeDef cfg = {0};
  uint8_t ok = 0U;

  cfg.Channel = ADC_CHANNEL_8;
  cfg.Rank = 1U;
  cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_ADC_Start(&hadc1) == HAL_OK)
  {
    if (HAL_ADC_PollForConversion(&hadc1, POWER_ADC_TIMEOUT_MS) == HAL_OK)
    {
      *raw = HAL_ADC_GetValue(&hadc1);
      ok = 1U;
    }
    HAL_ADC_Stop(&hadc1);
  }
  return ok;
}

/* ---- 纯换算 ---- */
uint32_t power_monitor_battery_mv_div(uint32_t raw, uint32_t vdd_mv,
                                      uint32_t num, uint32_t den)
{
  uint64_t num_raw;

  if (den == 0U)
  {
    return 0U;
  }
  num_raw = (uint64_t)raw * (uint64_t)vdd_mv * (uint64_t)num;
  return (uint32_t)(num_raw / ((uint64_t)POWER_ADC_FULL_SCALE * (uint64_t)den));
}

uint32_t power_monitor_battery_mv(uint32_t raw, uint32_t vdd_mv)
{
  return power_monitor_battery_mv_div(raw, vdd_mv,
                                      POWER_BATT_DIV_NUM, POWER_BATT_DIV_DEN);
}

/* ---- 服务状态 ---- */
static power_snapshot_t s_snapshot;
static uint8_t s_have_t0;
static uint32_t s_t0;

void power_monitor_init(void)
{
  s_snapshot = (power_snapshot_t){0};
  s_have_t0 = 0U;
  s_t0 = 0U;
}

void power_monitor_step(uint32_t now_ms)
{
  uint32_t sum = 0U;
  uint8_t n = 0U;
  uint8_t configured;
  uint32_t min_mv;

  if (s_have_t0 == 0U)
  {
    s_t0 = now_ms;
    s_have_t0 = 1U;
  }
  else
  {
    uint32_t elapsed = (uint32_t)(now_ms - s_t0); /* 回绕安全 */
    if (elapsed < POWER_STEP_MS)
    {
      return;
    }
    s_t0 = now_ms;
  }

  /* VREFINT：8 次均值换算 3.3 V 轨 */
  if (adc_read_vrefint(&sum, &n) != 0U)
  {
    uint32_t mv = boot_ui_vdd_mv(sum / POWER_VREFINT_SAMPLES,
                                 *VREFINT_CAL_ADDR);
    if (mv > 0U)
    {
      s_snapshot.vdd_mv = mv;
      s_snapshot.vdd_valid = 1U;
    }
    else
    {
      s_snapshot.vdd_valid = 0U;
    }
  }
  else
  {
    s_snapshot.vdd_valid = 0U;
  }

  configured = power_monitor_battery_configured();
  s_snapshot.battery_configured = configured;
  if (configured != 0U)
  {
    uint32_t raw = 0U;
    if (adc_read_ch8(&raw) != 0U)
    {
      min_mv = power_monitor_battery_min_mv();
      s_snapshot.battery_mv = power_monitor_battery_mv_div(
          raw, s_snapshot.vdd_mv, POWER_BATT_DIV_NUM, POWER_BATT_DIV_DEN);
      s_snapshot.battery_valid = 1U;
      s_snapshot.battery_error = 0U;
      s_snapshot.undervolt =
          (s_snapshot.battery_mv < min_mv) ? 1U : 0U;
    }
    else
    {
      s_snapshot.battery_valid = 0U;
      s_snapshot.battery_error = 1U;
      s_snapshot.undervolt = 0U;
    }
  }
  else
  {
    s_snapshot.battery_mv = 0U;
    s_snapshot.battery_valid = 0U;
    s_snapshot.battery_error = 0U;
    s_snapshot.undervolt = 0U;
  }
}

const power_snapshot_t *power_monitor_get(void)
{
  return &s_snapshot;
}
