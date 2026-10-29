#ifndef BOOT_UI_H
#define BOOT_UI_H

#include <stdint.h>

/* 编码器一圈计数：4 倍频 × 500 线 × 27 减速比（MD36N_27 + GMR_500） */
#define WHEEL_CPR 54000U

/* VDD 通过阈值（mV）。3.3 V 轨实测约 3.28–3.30 V，按字面 >3300 会误判 */
#define BOOT_UI_VDD_MIN_MV 3100U

/* 转速采样：每 20 ms 读一次，累计 5 次得 100 ms 窗口 */
#define BOOT_UI_ENC_SAMPLE_MS 20U
#define BOOT_UI_ENC_SAMPLES    5U

/* 主页面 VDD 刷新间隔 */
#define BOOT_UI_VDD_REFRESH_MS 500U

static inline int32_t boot_ui_rpm_from_counts(int32_t counts_100ms, uint32_t cpr)
{
  if (cpr == 0U)
  {
    return 0;
  }
  return (counts_100ms * 600) / (int32_t)cpr;
}

static inline uint32_t boot_ui_vdd_mv(uint32_t raw_avg, uint16_t vrefint_cal)
{
  if (vrefint_cal == 0U || vrefint_cal == 0xFFFFU)
  {
    vrefint_cal = 1490U;
  }
  if (raw_avg == 0U)
  {
    return 0U;
  }
  return (3300U * (uint32_t)vrefint_cal) / raw_avg;
}

void boot_ui_run(void);

#endif
