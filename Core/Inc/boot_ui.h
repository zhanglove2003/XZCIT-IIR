#ifndef BOOT_UI_H
#define BOOT_UI_H

#include <stdint.h>

/* Manual full-turn calibration on 2026-10-30, measured per physical wheel.
 * RF uses four manual turns; LF/RR/LR use two manual turns each.
 * These references are not precision factory calibration.
 * Forward signs were established while rolling each wheel forward by hand. */
#define WHEEL_RF_CPR 213203U
#define WHEEL_LF_CPR 212664U
#define WHEEL_RR_CPR 214993U
#define WHEEL_LR_CPR 214348U
#define WHEEL_RF_FORWARD_SIGN (-1)
#define WHEEL_LF_FORWARD_SIGN 1
#define WHEEL_RR_FORWARD_SIGN (-1)
#define WHEEL_LR_FORWARD_SIGN 1

/* VDD 运动许可阈值 BOOT_UI_VDD_MIN_MV 已迁至 power_monitor.h（电源域）；
 * 兼容 include：让旧消费者（test_boot_math 等）继续按原名取到该宏。 */
#include "power_monitor.h"

/* 转速采样窗：Ctrl 任务每 20 ms 采样、累计 5 次得 100 ms 窗口 */
#define BOOT_UI_ENC_SAMPLE_MS 20U
#define BOOT_UI_ENC_SAMPLES    5U

/* 纯换算内联：power_monitor（VDD 换算）与主机测试共用。 */

static inline int32_t boot_ui_rpm_from_counts(int32_t counts_100ms, uint32_t cpr)
{
  if (cpr == 0U)
  {
    return 0;
  }
  return (counts_100ms * 600) / (int32_t)cpr;
}

static inline int32_t boot_ui_rpm_from_elapsed(int32_t counts, uint32_t cpr,
                                                uint32_t elapsed_ms)
{
  int64_t rpm;
  if (cpr == 0U || elapsed_ms == 0U)
  {
    return 0;
  }
  rpm = ((int64_t)counts * 60000) / (int64_t)cpr / (int64_t)elapsed_ms;
  if (rpm > INT32_MAX) { return INT32_MAX; }
  if (rpm < INT32_MIN) { return INT32_MIN; }
  return (int32_t)rpm;
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

/* boot_ui 已收敛为纯渲染层（spec §11.2/§12）：
 *   - 电机（含写零）的唯一出口是 Ctrl 任务的 ctrl_apply()；
 *   - 页面状态、编码器累计、自检/归位/标定动作全部在 Ctrl；
 *   - 本模块只做上电安全初始化（SLP 低 + PWM 占空比 0 + 编码器启动）、
 *     OLED 初始化，然后把 Ctrl 发布的 ui_snapshot 画到屏上。
 * 原双归属时代的 boot_ui_motor_owner/handoff_seq/selfcheck_seq/
 * selfcheck_passed/drive_enabled/diag 全局随之删除：
 * 自检结果消费 ui_snapshot_get()（selfcheck_passed/selfcheck_state 字段）。 */
void boot_ui_run(void);

/* OLED 初始化完成后置 1；渲染循环就绪的标志（不涉及电机归属）。 */
extern volatile uint8_t boot_ui_ready;

#endif
