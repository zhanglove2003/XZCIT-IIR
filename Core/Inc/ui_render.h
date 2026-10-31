#ifndef UI_RENDER_H
#define UI_RENDER_H

#include <stdint.h>

#include "ui_model.h"

/* OLED 渲染层的输入快照：Ctrl 任务每 10 ms 用 seq-lock 发布一份，
 * 渲染任务用 ui_snapshot_get() 读取（重试 + 临界区兜底，模式同
 * wheel_home_get_status）。本头文件只定义快照结构与读取接口；
 * OLED 绘制函数由渲染层（Task 5）另行提供。 */
typedef struct {
  ui_page_t page;
  uint8_t menu_sel, sub_sel, sub_scroll, confirm_focus, result_view;
  uint32_t sel_since_ms;
  uint8_t drive_permitted;      /* Ctrl 仲裁：当前允许输出 */
  uint8_t action_busy, selfcheck_passed;
  uint8_t selfcheck_state[4];   /* 0 等待 1 运行 2 通过 3 失败 */
  uint8_t home_state[4];        /* 0 等待 1 运动 2 到位 3 失败 */
  uint8_t home_block_reason;    /* 0 无 1 无参考 2 等回中 3 故障 */
  uint8_t result_window_open;   /* 自检结束 5 s 标定入口窗口 */
  uint8_t fault_mask, reference_valid, frame_valid;
  ui_drive_status_t drive_status;
  int32_t wheel_rpm[4];
  int64_t wheel_milliturns[4];
  uint8_t wheel_sample_valid[4];
  uint8_t battery_valid, battery_error, undervolt, vdd_ok;
  uint32_t battery_mv;
} ui_snapshot_t;

void ui_snapshot_get(ui_snapshot_t *out);   /* seq-lock 读，实现在 freertos.c */

#endif
