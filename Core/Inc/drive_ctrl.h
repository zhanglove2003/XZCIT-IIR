#ifndef DRIVE_CTRL_H
#define DRIVE_CTRL_H

#include <stdint.h>

/* 摇杆死区半径（ly 距中心 128 的绝对偏差小于等于该值视为 0） */
#define DRIVE_CTRL_DEADZONE 16

/* 满速占空比：TIM8 ARR=16799 → 50% = 8400，与自检上限一致 */
#define DRIVE_CTRL_MAX_PULSE 8400U

/* 转向轴（rx）满速占空比：混合时转向分量上限，取前进上限的一半 */
#define DRIVE_CTRL_TURN_MAX_PULSE 4200

/* 通用摇杆轴映射（0=推杆向上/负方向端点，128=中位，255=向下/正方向端点）：
 * 死区内为 0，死区外按 |偏差|-16 格线性铺满到 ±limit。
 * spec §8：正负两侧分别按各自有效行程归一化（上行 112 格、下行 111 格），
 * 两端对称到达 ±limit；raw < 128（前进/左转方向）为正，端点饱和于 ±limit。 */
int32_t drive_ctrl_map_axis(uint8_t raw, int32_t limit);

/* 双轴混合：v = map_axis(ly, DRIVE_CTRL_MAX_PULSE)（前进 + / 后退 -），
 * t = map_axis(rx, DRIVE_CTRL_TURN_MAX_PULSE)（左转 + / 右转 -）；
 * A/C（右轮）= v+t，B/D（左轮）= v-t；四值绝对值峰值超过
 * DRIVE_CTRL_MAX_PULSE 时整体等比缩放回限幅。out 下标 0..3 = A/B/C/D。 */
void drive_ctrl_mix(uint8_t ly, uint8_t rx, int32_t out[4]);

/* 左摇杆 Y 轴（0=推杆向上/前进，128=中位，255=向下/后退）
 * 映射为带符号 PWM 比较值：前进为正，后退为负，死区内为 0，
 * 端点饱和于 ±DRIVE_CTRL_MAX_PULSE。map_axis(ly, DRIVE_CTRL_MAX_PULSE) 的薄封装。 */
int32_t drive_ctrl_map(uint8_t ly);
/* Service mode: suspend motor commands while encoder position keeps updating. */
extern volatile uint8_t drive_calibration_mode;

#endif
