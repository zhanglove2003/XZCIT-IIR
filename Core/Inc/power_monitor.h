#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <stdint.h>

/* 3.3 V 轨运动许可阈值（mV）。轨压实测 3.28-3.30 V，字面 >3300 会误判。
 * 原 BOOT_UI_VDD_MIN_MV 自 boot_ui.h 迁入：vdd 门槛属于电源域，
 * boot_ui.h 保留兼容 include（旧消费者不断链）。 */
#define BOOT_UI_VDD_MIN_MV 3100U

/* 电池监测配置。现场确认分压/规格/阈值后改 1 并填宏；
 * 未配置（0）时不采 CH8、battery_valid 恒 0、不判欠压。 */
#define POWER_BATTERY_CONFIGURED 0
#define POWER_BATT_DIV_NUM 1U
#define POWER_BATT_DIV_DEN 1U
#define POWER_BATT_MIN_MV  0U

typedef struct
{
  uint8_t vdd_valid;           /* 最近一次 VREFINT 采样成功且 vdd_mv>0 */
  uint8_t vdd_ok;              /* vdd_valid 且 vdd_mv >= BOOT_UI_VDD_MIN_MV：
                                  唯一允许运动/自检的电源许可位 */
  uint32_t vdd_mv;             /* 3.3 V 轨电压（mV） */
  uint8_t battery_configured;  /* 与 POWER_BATTERY_CONFIGURED 一致 */
  uint8_t battery_valid;       /* 最近一次 CH8 采样成功 */
  uint8_t battery_error;       /* 配置后采样失败（成功后清零） */
  uint8_t undervolt;           /* 配置且有效且 battery_mv < 阈值 */
  uint32_t battery_mv;         /* 未配置恒 0 */
} power_snapshot_t;

void power_monitor_init(void);
/* 500 ms 节拍（内部判断），回绕安全。 */
void power_monitor_step(uint32_t now_ms);
const power_snapshot_t *power_monitor_get(void);
/* 纯换算：battery_mv = raw * vdd_mv / 4095 * num / den（64 位中间值）。 */
uint32_t power_monitor_battery_mv(uint32_t raw, uint32_t vdd_mv);
/* 测试可注入分压比；power_monitor_battery_mv 用默认宏包装。den==0 返回 0。 */
uint32_t power_monitor_battery_mv_div(uint32_t raw, uint32_t vdd_mv,
                                      uint32_t num, uint32_t den);

#endif /* POWER_MONITOR_H */
