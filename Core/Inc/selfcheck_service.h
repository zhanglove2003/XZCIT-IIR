#ifndef SELFCHECK_SERVICE_H
#define SELFCHECK_SERVICE_H

#include <stdint.h>

#include "main.h" /* SELFCHECK_PWM_PULSE/RUN_MS/PAUSE_MS/CYCLES */
#include "motor_test.h"

/* Ctrl 推进的逐轮电机自检（spec §9.1）：A→B→C→D，
 * 每轮 前转 SELFCHECK_RUN_MS → 停 SELFCHECK_PAUSE_MS →
 * 反转 SELFCHECK_RUN_MS → 停 SELFCHECK_PAUSE_MS（subphase 0..3）。
 * 本模块是纯状态机：所有 PWM/DIR 语义只落到 command[]
 * （符号=前进为正，受试轮 ±SELFCHECK_PWM_PULSE，其余轮 0），
 * 硬件落地由 Ctrl 的 ctrl_apply() 完成（换向等待、唤醒延时天然复用）。
 * 每 10 ms 调一次 selfcheck_service_step()。 */

typedef enum
{
  SELFCHECK_IDLE = 0,
  SELFCHECK_RUNNING,
  SELFCHECK_DONE
} selfcheck_phase_t;

typedef struct
{
  selfcheck_phase_t phase;
  uint8_t wheel, subphase;   /* subphase: 0 前转 1 停 2 反转 3 停 */
  uint8_t state[4];          /* 快照用：0 等待 1 运行 2 通过 3 失败 */
  uint8_t passed;            /* 四轮全过 */
  uint32_t seq;              /* 每完成一轮 +1；Ctrl 据此清故障锁存 */
  uint32_t phase_start_ms;
  int32_t phase_counts;      /* 当前段净计数（回绕已由调用方增量抵消） */
  motor_test_result_t result[4];
} selfcheck_service_t;

void selfcheck_service_init(selfcheck_service_t *s);
/* 请求开始一轮自检（IDLE/DONE 下生效，RUNNING 中忽略） */
void selfcheck_service_request(selfcheck_service_t *s, uint32_t now_ms);
/* RUNNING 中取消：command 归 0、passed=0，已完成轮的 state/结果保留可回看 */
void selfcheck_service_cancel(selfcheck_service_t *s);
/* 每 10 ms 调用：fb_delta 为四轮前进为正的增量；command 四轮写 0 或受试轮
 * ±SELFCHECK_PWM_PULSE。运行中四轴越出驾驶死区由 Ctrl 检测并调 cancel。
 * 标定模式（drive_calibration_mode 非 0）下全轮 SKIP、state 保持等待、
 * command 恒 0。取消/失败不清旧故障锁存。 */
void selfcheck_service_step(selfcheck_service_t *s, const int32_t fb_delta[4],
                            uint32_t now_ms, int32_t command[4]);

#endif
