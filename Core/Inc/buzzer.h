#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

/* 蜂鸣器提示（PD11，纯 GPIO 输出，只能驱动有源蜂鸣器）。
 *
 * 极性：代码里没有任何地方主动拉高过 PD11，原理图也不在仓库里，
 * 所以"高电平响"是默认假设。如果上板发现响/不响正好相反，
 * 只改 BUZZER_ACTIVE_LEVEL 一处即可。
 *
 * 设计原则：
 *  - 非阻塞。控制器每轮调 buzzer_step() 推进，绝不在里面延时，
 *    否则会把 10 ms 的 Ctrl 周期拖长，影响手柄响应和反馈采样。
 *  - 一次故障只响一次。故障是锁存的，不去重就会一直响。
 *  - 蜂鸣只是提示，不参与任何安全判定，也不改变电机输出。 */

/* PD11：默认高电平有效。 */
#define BUZZER_ACTIVE_LEVEL   1U
#define BUZZER_IDLE_LEVEL     (1U - BUZZER_ACTIVE_LEVEL)

/* 一次提示的节奏：响 120 ms、停 80 ms，共 3 声。 */
#define BUZZER_BEEP_ON_MS     120U
#define BUZZER_BEEP_OFF_MS     80U
#define BUZZER_BEEP_COUNT       3U

void buzzer_init(void);
/* 请求一次故障提示。重复请求在提示播完之前会被合并，不会叠加。 */
void buzzer_alarm(void);
/* 每轮推进；now_ms 传 HAL_GetTick()。无提示时立即返回。 */
void buzzer_step(uint32_t now_ms);
/* 是否有提示正在播（测试用）。 */
uint8_t buzzer_busy(void);

#endif
