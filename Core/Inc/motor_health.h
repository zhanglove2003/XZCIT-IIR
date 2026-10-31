#ifndef MOTOR_HEALTH_H
#define MOTOR_HEALTH_H

#include <stdint.h>

/* 运行期运动反馈监测 + 故障锁存：只判定、只降级，不写硬件、不碰 RTOS。
 * 口径按 plan 固定：命令 PWM >= MOTOR_HEALTH_MIN_PWM 才开始算“给了命令”，
 * 启动后先给 MOTOR_HEALTH_STARTUP_GRACE_MS 宽限，再看 MOTOR_HEALTH_WINDOW_MS
 * 窗口内净计数是否达到 MOTOR_HEALTH_MIN_COUNTS。 */

#define MOTOR_HEALTH_MIN_PWM          1200   /* 低于此值不判故障（低速蠕行/换向） */
#define MOTOR_HEALTH_STARTUP_GRACE_MS 300U
#define MOTOR_HEALTH_WINDOW_MS        300U
#define MOTOR_HEALTH_MIN_COUNTS       64     /* 300 ms 窗口内的净计数门槛 */

typedef enum
{
  MOTOR_HEALTH_OK = 0,
  MOTOR_HEALTH_NO_FB,   /* 给了 PWM 但窗口内几乎不动：反馈异常，不等于证实离线 */
  MOTOR_HEALTH_DIR_ERR, /* 反馈符号与施加方向相反且幅度足够 */
  MOTOR_HEALTH_DISABLED /* 本轮不满足检测条件（零输出/低 PWM/睡眠/换向/UI 占权） */
} motor_health_code_t;

typedef struct
{
  int32_t pwm[4];  /* 实际施加的 signed PWM（forward-positive） */
  int32_t fb[4];   /* 本周期编码器增量（forward-positive 净计数） */
  uint32_t now_ms;
  uint8_t measurable; /* 0 = 睡眠/换向等待/手动对齐/未移交自检，本轮不监测 */
} motor_health_in_t;

typedef struct
{
  uint8_t fault_mask;      /* 已锁存故障轮 */
  uint8_t stop_all;        /* 1 = 本轮必须全部清零并睡眠 */
  motor_health_code_t code[4];
  uint8_t online_mask;     /* 只在通过窗口检测后才置位，不当作“确认在线”的充分证据 */
} motor_health_out_t;

typedef struct
{
  uint32_t cmd_start_ms[4];
  uint32_t win_start_ms[4];
  int64_t win_counts[4];
  int8_t dir[4];
  uint8_t window_active[4];
  uint8_t fault_mask;
  uint8_t online_mask;
} motor_health_t;

void motor_health_init(motor_health_t *h);
void motor_health_step(motor_health_t *h, const motor_health_in_t *in,
                       motor_health_out_t *out);
/* 清除只允许由“用户主动重新自检且四轮通过”触发，绝不由 SELECT 记录操作清除。 */
void motor_health_clear_after_selfcheck(motor_health_t *h, uint8_t selfcheck_passed);
/* 故障码 -> 短文本（日志用）。长度固定 7 字符，便于按列对齐。 */
const char *motor_health_code_text(motor_health_code_t code);

#endif
