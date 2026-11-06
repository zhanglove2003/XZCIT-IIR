#ifndef MANUAL_REF_H
#define MANUAL_REF_H

#include <stdint.h>

/* 自检结束后的 USER 键窗口 + 手动对齐页：纯 C，不吞按键、不写硬件。
 * 只在真实毫秒上计时；按键去抖只数连续有效帧（无效帧既不推进也不清零已确认电平）。 */

/* 自检收尾后等待 USER 再次按下进入手动页的窗口 */
#define MANUAL_REF_WINDOW_MS          5000U
/* USER 按下去抖：10 ms 一轮 = 30 ms */
#define MANUAL_REF_PRESS_STABLE_FRAMES 3U
/* 手动页里 SELECT 长按多久才请求记录参考 */
#define MANUAL_REF_RECORD_MS          2000U
/* 手动页里一次按下到释放的完整按压，才视为“退出” */
#define MANUAL_REF_MIN_PRESS_MS       50U

typedef enum
{
  MANUAL_REF_STATE_WAITING = 0, /* 窗口计时中，等 USER 按下 */
  MANUAL_REF_STATE_MANUAL,      /* 手动对齐页 */
  MANUAL_REF_STATE_MAIN         /* 主界面，窗口已关闭 */
} manual_ref_state_t;

typedef struct
{
  uint32_t window_start_ms;
  uint32_t press_ms;      /* SELECT 本轮按下的起点，0 表示未按下 */
  uint32_t user_ms;       /* USER 本轮按下的起点 */
  uint8_t window_open;    /* 窗口尚未结束 */
  uint8_t window_armed;   /* 窗口内已经释放过一次，再次按下才开门 */
  uint8_t window_seeded;  /* 打开窗口时 USER 已按住的那一次已计入 */
  uint8_t user_filtered;  /* 已确认的 USER 电平 */
  uint8_t user_candidate; /* 正在计数的 USER 电平 */
  uint8_t user_frames;    /* 该电平连续有效帧数 */
  uint8_t select_filtered;
  uint8_t select_candidate;
  uint8_t select_frames;
  uint8_t record_requested;
  uint8_t record_consumed;
  uint8_t ref_shown;
  uint8_t ready_ok;       /* 停稳 + 手柄有效 + 摇杆回中 */
  uint8_t single_shot;    /* 保留：当前实现用 record_consumed 保证只发一次 */
  uint8_t swallow_release;   /* 开门那次按下的释放沿不算完整按压 */
  manual_ref_state_t state;
} manual_ref_t;

void manual_ref_open(manual_ref_t *s, uint32_t now_ms);
/* 菜单入口：跳过 USER 窗口直接进手动对齐页（open + enter_manual）。
 * 进页后 SELECT 仍按 enter_manual 的规矩视为“已按住”，
 * 必须先释放再按满 2 s 才请求记录参考。 */
void manual_ref_open_manual(manual_ref_t *s, uint32_t now_ms);
/* 进手动页：窗口里的按键必须重新释放才计时（防止进页瞬间就记参考）。 */
void manual_ref_enter_manual(manual_ref_t *s, uint32_t now_ms);
/* 记录参考的前置条件，由 Ctrl 每轮提供。 */
void manual_ref_set_ready(manual_ref_t *s, uint8_t still, uint8_t frame_valid,
                          uint8_t sticks_centered);

/* user_raw / select_raw 都是“按下为 1”的电平；无效帧请传 frame_valid=0。 */
void manual_ref_step(manual_ref_t *s, uint32_t now_ms, uint8_t user_raw,
                     uint8_t select_raw, uint8_t frame_valid);

manual_ref_state_t manual_ref_state(const manual_ref_t *s);
uint8_t manual_ref_record_requested(const manual_ref_t *s);
/* Ctrl 应答：success 为 1 才显示 REF SET。 */
void manual_ref_acknowledge(manual_ref_t *s, uint8_t success);
uint8_t manual_ref_ref_shown(const manual_ref_t *s);
uint8_t manual_ref_home_requested(const manual_ref_t *s);
uint32_t manual_ref_press_ms(const manual_ref_t *s);

#endif
