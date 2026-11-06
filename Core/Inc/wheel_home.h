#ifndef WHEEL_HOME_H
#define WHEEL_HOME_H

#include <stdint.h>

/* 手柄一键恢复四轮初始朝向：纯 C 状态机，不含 HAL/RTOS。
 * 位置一律为“前进为正”的归一化计数（raw_counts * forward_sign），
 * 由调用方（Ctrl 任务）自己持有 wheel_feedback 实例累加，本模块不读定时器。 */

/* SELECT 为手柄原始按键最低位，低有效；上板前仍需只读采集确认。
 * L1/R1 按 PS2 标准位序（bit10/bit11），仅用于菜单请求归位的放行门。 */
#define WHEEL_HOME_SELECT_BIT        0U
#define WHEEL_HOME_L1_BIT            10U
#define WHEEL_HOME_R1_BIT            11U
/* 菜单请求归位进 RUNNING 前，这些提交按键必须全部释放。 */
#define WHEEL_HOME_REQUEST_GATE_MASK \
  ((uint16_t)((1U << WHEEL_HOME_SELECT_BIT) | (1U << WHEEL_HOME_L1_BIT) | \
              (1U << WHEEL_HOME_R1_BIT)))
#define WHEEL_HOME_BUTTON_STABLE_MS  30U
/* 10 ms 控制周期下 30 ms 去抖 = 3 个连续有效帧；无效帧不推进稳定计时。 */
#define WHEEL_HOME_BUTTON_STABLE_FRAMES 3U
#define WHEEL_HOME_FRAME_LOSS_MS     200U
#define WHEEL_HOME_SHORT_PRESS_MIN_MS 30U
#define WHEEL_HOME_LONG_PRESS_MS     2000U
#define WHEEL_HOME_RESUME_NEUTRAL_MS 100U

/* 摇杆中位与死区；归位期间任一轴超出即取消。 */
#define WHEEL_HOME_STICK_CENTER      128
#define WHEEL_HOME_STICK_DEADBAND    16

/* 归位 PWM：TIM8 ARR=16799，3360 约 20%。1/4 圈误差即达到上限。 */
#define WHEEL_HOME_MAX_PULSE         3360U
#define WHEEL_HOME_MIN_PULSE         1200U
#define WHEEL_HOME_FULL_SCALE        (WHEEL_HOME_MAX_PULSE * 4U)

/* 时序，全部使用真实毫秒。 */
#define WHEEL_HOME_SETTLE_MS         200U
#define WHEEL_HOME_STILL_MS          300U
#define WHEEL_HOME_REVERSE_WAIT_MS   40U
#define WHEEL_HOME_NO_PROGRESS_MS    500U
#define WHEEL_HOME_TIMEOUT_MS        8000U
#define WHEEL_HOME_SAMPLE_TIMEOUT_MS 50U

typedef enum
{
  WHEEL_HOME_MANUAL = 0,   /* Ctrl 按普通摇杆输出 */
  WHEEL_HOME_BUTTON_HELD,  /* SELECT 按住中：四轮零输出 */
  WHEEL_HOME_RUNNING,      /* 正在归位 */
  WHEEL_HOME_WAIT_NEUTRAL  /* 结束/取消后等释放与回中 */
} wheel_home_mode_t;

typedef enum
{
  WHEEL_HOME_RESULT_NONE = 0,
  WHEEL_HOME_RESULT_RECORDED,
  WHEEL_HOME_RESULT_DONE,
  WHEEL_HOME_RESULT_CANCELLED,
  WHEEL_HOME_RESULT_NO_REF,
  WHEEL_HOME_RESULT_STALL,
  WHEEL_HOME_RESULT_TIMEOUT,
  WHEEL_HOME_RESULT_FEEDBACK_ERROR
} wheel_home_result_t;

typedef struct
{
  uint32_t now_ms;
  int64_t position[4]; /* Forward-positive counts from Ctrl-owned trackers. */
  int32_t delta[4];    /* Forward-positive increments for this sample. */
  uint8_t frame_valid, feedback_valid, calibration_mode;
  uint8_t manual_calib_page; /* 1 = UI 在手动标定页（记录经显式 API，页面
                              * 门控由调用方把关；step 不再直接写参考） */
  uint16_t buttons;   /* Raw PS2, active low. */
  uint8_t lx, ly, rx, ry;
} wheel_home_input_t;

typedef struct
{
  wheel_home_mode_t mode;
  wheel_home_result_t result;
  uint8_t reference_valid, settled_mask, owns_motors;
  uint16_t buttons;    /* 本轮原始 PS2 按键（低有效），UI/手动页读取 SELECT。 */
  uint8_t frame_valid; /* 本轮手柄帧是否有效，失联期间为 0。 */
  uint8_t fault_mask;  /* 反馈监测锁存的故障轮（bit0..3 = A/B/C/D）。 */
  uint8_t recovering;  /* 1 = 本拍采样中断后的对时拍，UI 可提示但不判故障。 */
  int32_t pulse[4];   /* Signed, forward-positive. Zero on abort/error. */
  int64_t error[4];
  /* UI 侧判"手动页可以记录参考"要用的量，必须和 Ctrl 写参考时读的是同一批：
   * 累计位置、CRP、摇杆。缺了这些，页面只能瞎猜停稳与否。 */
  int64_t position[4];
  uint32_t cpr[4];
  uint8_t feedback_valid; /* 本轮反馈（编码器）是否可信。 */
  uint8_t lx, ly, rx, ry;
} wheel_home_output_t;

/* Fixed-size state, no malloc; timestamp validity uses explicit flags. */
typedef struct
{
  uint32_t cpr[4];
  int64_t origin[4], target[4], best_abs_error[4];
  uint32_t press_ms, filter_ms, neutral_ms, stationary_ms, home_start_ms;
  uint32_t last_valid_frame_ms;
  uint32_t settle_ms[4], progress_ms[4], reverse_ms[4];
  uint32_t still_ms[4];
  int8_t applied_direction[4];
  uint8_t settle_active[4], reverse_active[4], progress_active[4];
  uint8_t still_active[4];
  uint8_t reference_valid, long_consumed, press_active;
  uint8_t filtered_select, candidate_select, filter_active;
  uint8_t stable_frames;   /* consecutive valid frames at the same level */
  uint8_t raw_select;      /* raw SELECT level of this valid frame */
  uint8_t frame_valid_now; /* valid frame seen this cycle */
  uint8_t have_frame;      /* at least one valid frame ever seen */
  uint8_t neutral_active, stationary_active, have_valid_frame;
  uint8_t recovering; /* 采样超时后的一拍对时：本拍只重新对时，不判按键/停稳 */
  uint8_t require_release;
  /* 菜单请求的归位：pending 置位后在 MANUAL 下等放行门满 100 ms。
   * 提交键（L1）的释放不产生任何事件，进 RUNNING 只由放行门触发。 */
  uint8_t request_pending, request_wait_active;
  uint32_t request_wait_ms;
  /* 公开取消入口的落地请求：wheel_home_cancel 在 RUNNING 中置位，
   * 由下一拍 wheel_home_step 用状态机自己的时钟收敛（本模块不持时钟）。 */
  uint8_t cancel_pending;
  uint32_t last_position_ms;
  /* 手动对齐页记录的停稳跟踪：与 RUNNING 的 still_* 分开，
   * 因为手册页里状态机处于 BUTTON_HELD，共用会互相打断。 */
  uint32_t record_still_ms[4];
  uint8_t record_still_active[4];
  wheel_home_mode_t mode;
  wheel_home_result_t result;
} wheel_home_t;

/* Shortest signed displacement that puts x back on origin's phase,
 * in -cpr/2 .. cpr/2 (exact half turn selects the reverse direction).
 * Returns 0 when cpr is 0; callers must reject that as a feedback error. */
int64_t wheel_home_displacement(int64_t x, int64_t origin, uint32_t cpr);

void wheel_home_init(wheel_home_t *s, const uint32_t cpr[4]);
void wheel_home_step(wheel_home_t *s, const wheel_home_input_t *in,
                     wheel_home_output_t *out);

/* 菜单入口的归位请求（例如手动对齐页里的"归位"项）：
 * 置 pending，wheel_home_step 在 MANUAL 模式下等"四轴回中 +
 * SELECT/L1/R1 释放连续 WHEEL_HOME_RESUME_NEUTRAL_MS"才真正进 RUNNING；
 * 无参考时请求直接以 WHEEL_HOME_RESULT_NO_REF 落地，不动任何轮子。
 * SELECT 短按快捷入口不受影响。 */
void wheel_home_request_home(wheel_home_t *s);

/* 公开取消入口（spec §7/§10：R1/新按键/页面离开/取消请求都必须能停掉
 * 归位）：立即丢弃 pending 的菜单请求；RUNNING 中置 cancel_pending，
 * 由下一拍 wheel_home_step 落地为 WAIT_NEUTRAL/CANCELLED——当拍四轮
 * 零脉冲、摇杆/按键手势清零、恢复后要求先看到释放。非运行且无
 * pending 时幂等无操作，允许每拍调用。 */
void wheel_home_cancel(wheel_home_t *s);

/* Ctrl 任务发布的最新仲裁快照（临界区保护），供 UI 状态栏读取。 */
/* 手动对齐页：长按 SELECT 记录参考。
 * 安全前提由本函数自己核对，不假定调用方已经检查过：
 *   - 状态机处于 MANUAL 或 BUTTON_HELD（长按 SELECT 时模式是后者）；
 *   - 本帧有效且反馈有效；
 *   - 摇杆四轴回中（页面不应边推杆边定零）；
 *   - 四轮连续静止满 WHEEL_HOME_SETTLE_MS；
 *   - CPR 非 0。
 * 需要多拍调用才能满足停稳条件。返回 1 = 本拍已写入参考。 */
/* 每轮采样调用一次，累积"手动页四轮已持续静止"的状态。
 * 只在记录时调用会让计时从零开始，长按松手稍快就永远记不上。 */
uint8_t wheel_home_track_manual_still(wheel_home_t *s,
                                      const wheel_home_input_t *in,
                                      uint32_t now);

uint8_t wheel_home_record_manual_reference(wheel_home_t *s,
                                           const wheel_home_input_t *in,
                                           uint32_t now);

void wheel_home_get_status(wheel_home_output_t *out);

/* 顶栏短状态文本：7 个字符 + '\0' 上界，128 像素屏 8 字符单元内可控。
 * size 为缓冲区字节数（含 '\0'）；size 为 0 或 buf 为 0 时不写。 */
#define WHEEL_HOME_STATUS_TEXT_MAX 8U
void wheel_home_status_text(char *buf, uint32_t size,
                            const wheel_home_output_t *out);

#endif
