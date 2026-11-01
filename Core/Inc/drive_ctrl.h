#ifndef DRIVE_CTRL_H
#define DRIVE_CTRL_H

#include <stdint.h>

/* 摇杆死区半径（ly 距中心 128 的绝对偏差小于等于该值视为 0） */
#define DRIVE_CTRL_DEADZONE 16

/* 满速占空比：TIM8 ARR=16799 → 50% = 8400，与自检上限一致。
 * 同时是运行时输出上限的可调上顶点（50%）与上电默认值。 */
#define DRIVE_CTRL_MAX_PULSE 8400U

/* 运行时驾驶 PWM 输出上限下限：10% = 840（"输出上限"设置页可调范围 10%~50%） */
#define DRIVE_CTRL_MIN_PULSE 840

/* 运行时驾驶 PWM 输出上限（前进/后退混合总限幅）：
 * 默认 8400（50%）；设置页经 drive_ctrl_set_max_pulse 运行时调整，
 * 只存 RAM，断电恢复默认。转向分量恒 = drive_ctrl_max_pulse/2。
 * 仅约束驾驶混合（drive_ctrl_mix/map）；自检、归位、手动标定不受影响。
 * 单写者：Ctrl 任务（设置页模型 → set_max_pulse，同任务 mix 消费）。 */
extern int32_t drive_ctrl_max_pulse;

/* 设置运行时输出上限（比较值口径），夹取到 [DRIVE_CTRL_MIN_PULSE, DRIVE_CTRL_MAX_PULSE] */
void drive_ctrl_set_max_pulse(int32_t v);

/* 通用摇杆轴映射（0=推杆向上/负方向端点，128=中位，255=向下/正方向端点）：
 * 死区内为 0，死区外按 |偏差|-16 格线性铺满到 ±limit。
 * spec §8：正负两侧分别按各自有效行程归一化（上行 112 格、下行 111 格），
 * 两端对称到达 ±limit；raw < 128（前进/左转方向）为正，端点饱和于 ±limit。 */
int32_t drive_ctrl_map_axis(uint8_t raw, int32_t limit);

/* 双轴混合：v = map_axis(ly, drive_ctrl_max_pulse)（前进 + / 后退 -），
 * t = map_axis(rx, drive_ctrl_max_pulse/2)（左转 + / 右转 -）；
 * A/C（右轮）= v+t，B/D（左轮）= v-t；四值绝对值峰值超过
 * drive_ctrl_max_pulse 时整体等比缩放回限幅。out 下标 0..3 = A/B/C/D。 */
void drive_ctrl_mix(uint8_t ly, uint8_t rx, int32_t out[4]);

/* 左摇杆 Y 轴（0=推杆向上/前进，128=中位，255=向下/后退）
 * 映射为带符号 PWM 比较值：前进为正，后退为负，死区内为 0，
 * 端点饱和于 ±drive_ctrl_max_pulse。map_axis(ly, drive_ctrl_max_pulse) 的薄封装。 */
int32_t drive_ctrl_map(uint8_t ly);
/* Service mode: suspend motor commands while encoder position keeps updating. */
extern volatile uint8_t drive_calibration_mode;

#endif
