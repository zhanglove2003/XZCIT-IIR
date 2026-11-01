#ifndef UI_MODEL_H
#define UI_MODEL_H

#include <stdint.h>

/* OLED 分层菜单的纯 C 页面状态机：不含 HAL/OLED/RTOS。
 * 由 Ctrl 任务每 10 ms 用 ui_model_step() 推进；渲染层只读 page/焦点字段。
 * 键位（spec §7）：R1=协议位 11，L1=位 10，SELECT=位 0，低有效；
 * USER 为引脚电平，按下为 1。去抖沿用 wheel_home 的"连续有效帧"口径：
 * UI_BTN_STABLE_FRAMES(3) 帧同电平才翻转；frame_valid=0 冻结所有计数不清零。
 * 所有时间比较使用 (uint32_t)(now - t0) 形式，天然回绕安全。 */

typedef enum
{
  UI_PAGE_BRAND = 0,
  UI_PAGE_MENU,
  UI_PAGE_SUBMENU,
  UI_PAGE_DRIVE,
  UI_PAGE_CHECK_CONFIRM,
  UI_PAGE_CHECK_RUN,
  UI_PAGE_CHECK_RESULT,
  UI_PAGE_HOME,
  UI_PAGE_MANUAL_CAL
} ui_page_t;

#define UI_MENU_COUNT 2U /* 0=驾驶 1=检查与标定 */
#define UI_SUB_COUNT  4U /* 0=自检结果 1=重新自检 2=回到初始位 3=手动标定 */

#define UI_STICK_CENTER 128
#define UI_STICK_ENGAGE 48  /* 偏移达到 48 才移动 */
#define UI_STICK_RELEASE 24 /* 回到偏移 24 以内才复位 */

#define UI_NAV_FIRST_REPEAT_MS 400U
#define UI_NAV_REPEAT_MS       180U

#define UI_BTN_STABLE_FRAMES 3U
#define UI_BTN_BIT_R1     11U
#define UI_BTN_BIT_L1     10U
#define UI_BTN_BIT_SELECT 0U

/* 手动标定页 SELECT 长按记录参考；漂移/坏帧取消本次计时（spec §8）。 */
#define UI_SELECT_LONG_PRESS_MS 2000U
/* 确认页 5 s 倒计时自动启动（spec §4，2026-11-01）：
 * 归零当拍自发 selfcheck_start；前置不满足时冻结并置 confirm_blocked。
 * 摇杆右/左立即开始/取消阈值：lx ≥ 176 / lx ≤ 80（同导航 engage 48 偏移）。 */
#define UI_CONFIRM_COUNTDOWN_MS 5000U
#define UI_CONFIRM_START_LX     176U
#define UI_CONFIRM_CANCEL_LX    80U
/* 归位/自检期间摇杆越出驾驶死区（同 wheel_home 16 的口径）即取消动作。 */
#define UI_STICK_DRIFT_LIMIT 16
/* 行驶状态分类死区：|v|/|t| <= 该值视为 0（输入为映射后的脉宽）。 */
#define UI_DRV_STATUS_DEADBAND 1

/* 低有效原始键位；摇杆 0..255；user = USER 引脚电平(按下为 1) */
typedef struct
{
  uint16_t buttons;
  uint8_t lx, ly, rx, ry;
  uint8_t user_pressed, frame_valid;
  uint8_t selfcheck_passed;   /* Ctrl 回喂：本次上电自检是否已通过 */
  uint8_t selfcheck_ready;    /* Ctrl 回喂：自检前置全部满足（帧有效+四轴
                               * 回中+vdd_ok+非标定），0 时确认页倒计时冻结 */
  uint8_t action_busy;        /* Ctrl 回喂：自检/归位执行中 */
  uint8_t result_window_open; /* 自检刚结束的 5 s 标定入口窗口（Ctrl 计时） */
  uint32_t now_ms;
} ui_model_in_t;

/* 本拍动作请求。ui_model_step 每拍先清零全部字段再置位本拍触发的请求；
 * 调用方无须预清零。请求为单发语义（一次稳定按下只触发一拍）。 */
typedef struct
{
  uint8_t selfcheck_start, home_start, record_ref, cancel_motion;
} ui_model_req_t;

/* 菜单摇杆轴状态：迟滞 engage/release + 400/180 ms 连移。 */
typedef struct
{
  int8_t dir;       /* -1 / 0 / +1 当前接通方向 */
  uint32_t step_ms; /* 上次移动（或 engage）时刻锚点 */
  uint8_t fast;     /* 0=等 UI_NAV_FIRST_REPEAT_MS，1=等 UI_NAV_REPEAT_MS */
} ui_stick_axis_t;

typedef struct
{
  ui_page_t page, back_page;
  uint8_t menu_sel, sub_sel, sub_scroll, confirm_focus, result_view;
  uint32_t sel_since_ms; /* 焦点/滑动动画锚点 */
  /* 确认页 5 s 倒计时（spec §4）：进入页面时锚定，ready=0 冻结（锚点随
   * 剩余时间回推，恢复后继续走剩余）。confirm_focus 保留但不再参与启动
   * 裁决——启动只看 L1 边沿 / lx≥176 / 倒计时归零。 */
  uint32_t confirm_since_ms;
  uint16_t confirm_remain_ms;
  uint8_t confirm_blocked;
  /* 按键滤波：索引 0=R1 1=L1 2=SELECT 3=USER。 */
  uint8_t btn_filtered[4], btn_candidate[4], btn_frames[4];
  uint32_t btn_press_ms[4];  /* 稳定按下沿时刻（SELECT 长按锚点） */
  uint8_t btn_long_fired[4]; /* 长按单发锁存（仅 SELECT 使用） */
  ui_stick_axis_t axis_h;    /* 横向：主菜单卡片 / 结果页翻屏（lx） */
  ui_stick_axis_t axis_v;    /* 纵向：子菜单列表 / 确认页焦点（ly） */
  uint8_t run_seen_busy;     /* 动作页已见过 action_busy=1（防请求当拍误判完成） */
  uint8_t window_armed;      /* 自检运行进入结果页时 5 s 窗口是否开着 */
} ui_model_t;

void ui_model_init(ui_model_t *m, uint32_t now_ms);
void ui_model_step(ui_model_t *m, const ui_model_in_t *in, ui_model_req_t *req);

/* 行驶状态分类（Task 5 顶栏用）：V/T 死区内为 0。t>0 为左转。 */
typedef enum
{
  UI_DRV_STOP = 0,
  UI_DRV_FWD,
  UI_DRV_REV,
  UI_DRV_LEFT,
  UI_DRV_RIGHT,
  UI_DRV_FWD_LEFT,
  UI_DRV_FWD_RIGHT,
  UI_DRV_REV_LEFT,
  UI_DRV_REV_RIGHT
} ui_drive_status_t;

ui_drive_status_t ui_drive_status_classify(int32_t v, int32_t t);

#endif
