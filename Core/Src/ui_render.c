/* ui_render：OLED 各页面绘制。只读快照、不碰硬件、不写任何模型状态；
 * clear + 按页绘制到 oled_gfx 帧缓冲，flush 由渲染循环（boot_ui）负责。
 * 排版数值来自 spec §5/§6；卡片跳动/滑动只由 now_ms 与快照锚点计算。
 * 本任务图标一律用 1px 边框占位，位图图标由视觉打磨任务补。 */
#include "ui_render.h"

#include <stdio.h>

#include "oled_font.h"
#include "oled_gfx.h"

#define CARD_W        92
#define CARD_H        32
#define CARD_HOME_X   18  /* 选中卡片居中起点（spec §5.2） */
#define CARD_HOME_Y   20
#define CARD_PITCH    100 /* 相邻卡片间距：92 + 8 px 间隙 */
#define HOP_PERIOD_MS 600U
#define HOP_STEP_MS   120U
#define SLIDE_MS      120U
#define SLIDE_MAX_PX  64

#define WHEEL_ROW_Y0 12
#define WHEEL_ROW_STEP 12
#define RPM_TEXT_X    12  /* RPM 域 x12..59（8 字符） */
#define TURNS_TEXT_X  66  /* 圈数域 x66..125（10 字符） */

/* ---------------- 格式化函数 ---------------- */

void ui_render_format_battery(char out[7], uint8_t valid, uint32_t mv)
{
  uint32_t tenths;

  if (valid == 0U)
  {
    out[0] = 'B';
    out[1] = '-';
    out[2] = '-';
    out[3] = '.';
    out[4] = '-';
    out[5] = 'V';
    out[6] = '\0';
    return;
  }
  tenths = mv / 100U;
  if (tenths > 999U)
  {
    out[0] = 'B';
    out[1] = ' ';
    out[2] = 'O';
    out[3] = 'V';
    out[4] = 'F';
    out[5] = '\0';
    return;
  }
  out[0] = 'B';
  if (tenths >= 100U)
  {
    out[1] = (char)('0' + (tenths / 100U));
    out[2] = (char)('0' + ((tenths / 10U) % 10U));
  }
  else
  {
    out[1] = (char)('0' + (tenths / 10U));
    out[2] = '.';
    out[3] = (char)('0' + (tenths % 10U));
    out[4] = 'V';
    out[5] = '\0';
    return;
  }
  out[3] = '.';
  out[4] = (char)('0' + (tenths % 10U));
  out[5] = 'V';
  out[6] = '\0';
}

void ui_render_format_rpm(char out[9], char wheel, int32_t rpm, uint8_t valid)
{
  uint32_t mag;
  uint32_t digits;
  int i;

  if (valid == 0U)
  {
    out[0] = wheel;
    out[1] = ' ';
    out[2] = ' ';
    out[3] = '-';
    out[4] = '-';
    out[5] = '\0';
    return;
  }
  if ((rpm < -9999) || (rpm > 9999))
  {
    out[0] = wheel;
    out[1] = ' ';
    out[2] = 'O';
    out[3] = 'V';
    out[4] = 'F';
    out[5] = '\0';
    return;
  }
  mag = (uint32_t)(rpm < 0 ? -rpm : rpm);
  i = 0;
  out[i] = wheel;
  i++;
  out[i] = (rpm < 0) ? '-' : '+';
  i++;
  digits = mag;
  {
    uint8_t started = 0U;
    uint32_t scale;

    for (scale = 1000U; scale >= 1U; scale /= 10U)
    {
      uint32_t d = digits / scale;

      if ((d > 0U) || (started != 0U) || (scale == 1U))
      {
        out[i] = (char)('0' + d);
        i++;
        started = 1U;
      }
      digits %= scale;
    }
  }
  out[i] = 'r';
  out[i + 1] = 'p';
  out[i + 2] = 'm';
  out[i + 3] = '\0';
}

void ui_render_format_turns(char out[11], int64_t milliturns, uint8_t valid)
{
  int64_t mag;
  uint32_t turns;
  uint32_t hundredths;

  if (valid == 0U)
  {
    out[0] = '-';
    out[1] = '-';
    out[2] = '\0';
    return;
  }
  mag = (milliturns < 0) ? -milliturns : milliturns;
  if (mag > 999999)
  {
    out[0] = 'O';
    out[1] = 'V';
    out[2] = 'F';
    out[3] = '\0';
    return;
  }
  turns = (uint32_t)(mag / 1000);
  hundredths = (uint32_t)((mag % 1000) / 10);
  out[0] = (milliturns < 0) ? '-' : '+';
  out[1] = (char)('0' + (turns / 100U));
  out[2] = (char)('0' + ((turns / 10U) % 10U));
  out[3] = (char)('0' + (turns % 10U));
  out[4] = '.';
  out[5] = (char)('0' + (hundredths / 10U));
  out[6] = (char)('0' + (hundredths % 10U));
  out[7] = 'R';
  out[8] = '\0';
}

/* ---------------- 动画采样点 ---------------- */

int8_t ui_render_hop_offset(uint32_t now_ms, uint32_t sel_since_ms)
{
  static const int8_t table[5] = {0, -1, -2, -1, 0};
  uint32_t elapsed = now_ms - sel_since_ms; /* 无符号差值，回绕安全 */

  return table[(elapsed / HOP_STEP_MS) % 5U];
}

int8_t ui_render_slide_offset(uint32_t now_ms, uint32_t sel_since_ms)
{
  uint32_t elapsed = now_ms - sel_since_ms;

  if (elapsed >= SLIDE_MS)
  {
    return 0;
  }
  /* 线性滑动，尾部进位到 1，保证 119 ms 处仍有位移 */
  return (int8_t)(-((int32_t)((SLIDE_MAX_PX * (SLIDE_MS - elapsed)) + 60U) / (int32_t)SLIDE_MS));
}

/* ---------------- 绘制辅助 ---------------- */

/* 逐轮状态短文本（自检与归位共用口径：0 等待 1 运行 2 通过 3 失败） */
static const char *state_text(uint8_t state)
{
  switch (state)
  {
    case 1U:
      return "RUN";
    case 2U:
      return "OK";
    case 3U:
      return "FAIL";
    default:
      return "WAIT";
  }
}

/* 驾驶状态短文本（spec §6） */
static const char *drive_status_text(ui_drive_status_t st)
{
  static const char *const names[9] = {
    "STOP", "FWD", "REV", "LEFT", "RIGHT", "FWD-L", "FWD-R", "REV-L", "REV-R"
  };

  return ((unsigned int)st < 9U) ? names[(unsigned int)st] : "STOP";
}

static void draw_centered_6x12(int16_t y, const char *text)
{
  oled_gfx_text6x12((int16_t)((OLED_WIDTH - (int16_t)oled_gfx_text6x12_width(text)) / 2), y, text);
}

/* 10×16 标题逐字绘制（未收录字形跳过，留空位） */
static void draw_title10(int16_t x, int16_t y, const char *ascii)
{
  int i;

  for (i = 0; ascii[i] != '\0'; i++)
  {
    const uint8_t *glyph = oled_font_title_lookup(ascii[i]);

    if (glyph != NULL)
    {
      oled_gfx_bitmap(x, y, 10, 16, glyph);
    }
    x = (int16_t)(x + 10);
  }
}

static void draw_name_row(int16_t y, char name, const char *text)
{
  char buf[2];

  buf[0] = name;
  buf[1] = '\0';
  oled_gfx_text6x12(0, y, buf);
  oled_gfx_text6x12(RPM_TEXT_X, y, text);
}

/* ---------------- 九个页面 ---------------- */

/* 品牌页：10×16 标题 + 6×12 副标题 + 底部一行提示/最高优先级异常 */
static void draw_brand(const ui_snapshot_t *s)
{
  const char *hint;

  draw_title10(14, 14, "XZCIT-MFSC");
  oled_gfx_text6x12(28, 36, "By Plucksnow");

  if (s->fault_mask != 0U)
  {
    hint = "MOTOR FAULT";
  }
  else if (s->frame_valid == 0U)
  {
    hint = "NO GAMEPAD";
  }
  else if (s->undervolt != 0U)
  {
    hint = "BATTERY LOW";
  }
  else if (s->vdd_ok == 0U)
  {
    hint = "VDD LOW";
  }
  else
  {
    hint = "R1:MENU  L1:CHECK";
  }
  draw_centered_6x12(52, hint);
}

/* 主菜单：两张 92×32 卡片，选中卡片居中轻跳，相邻卡片屏幕边缘露出 */
static void draw_menu(const ui_snapshot_t *s, uint32_t now_ms)
{
  static const char *const card_names[UI_MENU_COUNT] = {"DRIVE", "SELFCHECK"};
  int8_t hop = ui_render_hop_offset(now_ms, s->sel_since_ms);
  int8_t slide = ui_render_slide_offset(now_ms, s->sel_since_ms);
  int16_t x_sel = (int16_t)(CARD_HOME_X + slide);
  int16_t y_sel = (int16_t)(CARD_HOME_Y + hop);
  int16_t x_other;
  uint8_t i;

  if (s->menu_sel == 0U)
  {
    x_other = (int16_t)(x_sel + CARD_PITCH);
  }
  else
  {
    x_other = (int16_t)(x_sel - CARD_PITCH);
  }

  for (i = 0; i < UI_MENU_COUNT; i++)
  {
    int16_t cx = (i == s->menu_sel) ? x_sel : x_other;
    int16_t cy = (i == s->menu_sel) ? y_sel : CARD_HOME_Y;
    const char *name = card_names[i];
    uint16_t name_w = oled_gfx_text6x12_width(name);

    /* 图标占位：1px 边框（Task 9 视觉打磨补位图） */
    oled_gfx_rect((int16_t)(cx + (CARD_W - 20) / 2), (int16_t)(cy + 3), 20, 12);
    oled_gfx_text6x12((int16_t)(cx + ((int16_t)CARD_W - (int16_t)name_w) / 2), (int16_t)(cy + 18), name);
    /* 选中/未选中卡片均为 1px 边框；选中边框随跳动偏移，视觉打磨在 Task 9 */
    oled_gfx_rect(cx, cy, CARD_W, CARD_H);
  }
}

/* 子菜单：16px 标题 + 三行反白列表（滚动窗口）+ 窄滚动标记 */
static void draw_submenu(const ui_snapshot_t *s)
{
  static const char *const items[UI_SUB_COUNT] = {
    "RESULT VIEW", "RUN SELFCHECK", "GO HOME", "MANUAL CAL"
  };
  uint8_t r;
  int16_t y;
  int16_t thumb_y;

  oled_gfx_text16(0, 0, "检查与标定");
  for (r = 0; r < 3U; r++)
  {
    uint8_t idx = (uint8_t)(s->sub_scroll + r);

    if (idx >= UI_SUB_COUNT)
    {
      break;
    }
    y = (int16_t)(18 + r * 14);
    if (idx == s->sub_sel)
    {
      oled_gfx_text6x12_inv(0, y, items[idx]);
    }
    else
    {
      oled_gfx_text6x12(0, y, items[idx]);
    }
  }
  /* 窄滚动标记：x=126 一列，拇指位置随 sub_scroll 平移（可到顶/到底） */
  thumb_y = (int16_t)(18 + (int16_t)s->sub_scroll * 30);
  oled_gfx_fill_rect(126, thumb_y, 1, 14);
}

/* 驾驶仪表盘：顶栏 + 四轮行（固定数据布局，无动画） */
static void draw_drive(const ui_snapshot_t *s)
{
  static const char wheel_names[4] = {'A', 'B', 'C', 'D'};
  char buf[9];
  char turns_buf[11];
  const char *status;
  const char *right;
  uint8_t hint = (uint8_t)((s->drive_permitted == 0U) && (s->selfcheck_passed == 0U));
  uint8_t i;

  ui_render_format_battery(buf, s->battery_valid, s->battery_mv);
  oled_gfx_text6x12(0, 0, buf);

  /* 顶栏中部：未通过自检且未许可时显示提示（英文短句，中文字形 Task 9 补），
   * 否则显示行驶状态 */
  if (hint != 0U)
  {
    status = "NO SELFCHECK";
  }
  else
  {
    status = drive_status_text(s->drive_status);
  }
  oled_gfx_text6x12(42, 0, status);

  /* 顶栏右侧优先级（spec §6）：故障 > 失联 > 等待回中 > 无参考 > REF。
   * 未通过自检的提示占中部时，右侧仅在故障/失联时覆盖显示 */
  if (s->fault_mask != 0U)
  {
    right = "FAULT";
  }
  else if (s->frame_valid == 0U)
  {
    right = "LOST";
  }
  else if (hint != 0U)
  {
    right = NULL;
  }
  else if (s->drive_permitted == 0U)
  {
    right = "WAIT C"; /* 已过自检未许可：等回中/等条件 */
  }
  else if (s->reference_valid == 0U)
  {
    right = "NO REF";
  }
  else
  {
    right = "REF";
  }
  if (right != NULL)
  {
    oled_gfx_text6x12(90, 0, right);
  }

  for (i = 0; i < 4U; i++)
  {
    int16_t y = (int16_t)(WHEEL_ROW_Y0 + i * WHEEL_ROW_STEP);

    ui_render_format_rpm(buf, wheel_names[i], s->wheel_rpm[i], s->wheel_sample_valid[i]);
    /* RPM 字符串带轮名前缀，轮名单独画在 x=0，正文从 x=12 起（避免重名） */
    draw_name_row(y, wheel_names[i], &buf[1]);
    ui_render_format_turns(turns_buf, s->wheel_milliturns[i], s->wheel_sample_valid[i]);
    oled_gfx_text6x12(TURNS_TEXT_X, y, turns_buf);
  }
}

/* 自检启动确认：默认焦点在"取消" */
static void draw_check_confirm(const ui_snapshot_t *s)
{
  draw_centered_6x12(8, "START SELFCHECK");
  draw_centered_6x12(24, "WHEELS WILL SPIN");

  if (s->confirm_focus == 0U)
  {
    oled_gfx_text6x12_inv(28, 44, "CANCEL");
    oled_gfx_text6x12(84, 44, "START");
  }
  else
  {
    oled_gfx_text6x12(28, 44, "CANCEL");
    oled_gfx_text6x12_inv(84, 44, "START");
  }
}

/* 自检进度：逐轮 等待/运行/通过/失败；动作中右侧画占位标记 */
static void draw_check_run(const ui_snapshot_t *s)
{
  static const char wheel_names[4] = {'A', 'B', 'C', 'D'};
  uint8_t i;

  draw_centered_6x12(0, "SELFCHECK");
  for (i = 0; i < 4U; i++)
  {
    draw_name_row((int16_t)(12 + i * WHEEL_ROW_STEP), wheel_names[i],
                  state_text(s->selfcheck_state[i]));
  }
  if (s->action_busy != 0U)
  {
    oled_gfx_rect(114, 0, 14, 11); /* 动画占位（Task 9 补转圈位图） */
  }
}

/* 自检结果两屏：0=电机四轮 1=电池；左右翻屏由 ui_model 切 result_view */
static void draw_check_result(const ui_snapshot_t *s)
{
  static const char wheel_names[4] = {'A', 'B', 'C', 'D'};
  char line[32];

  if (s->result_view == 0U)
  {
    uint8_t i;

    oled_gfx_text6x12(0, 0, "MOTOR RESULT");
    for (i = 0; i < 4U; i++)
    {
      /* 快照无逐轮原因文本：失败只显示 FAIL，逐轮原因由 motor_test 日志查看 */
      draw_name_row((int16_t)(12 + i * WHEEL_ROW_STEP), wheel_names[i],
                    state_text(s->selfcheck_state[i]));
    }
  }
  else
  {
    oled_gfx_text6x12(0, 0, "BATTERY");
    if (s->battery_valid == 0U)
    {
      draw_centered_6x12(24, "NOT CONFIGURED");
    }
    else if (s->battery_error != 0U)
    {
      draw_centered_6x12(24, "SAMPLE ERROR");
    }
    else if (s->undervolt != 0U)
    {
      draw_centered_6x12(24, "UNDERVOLT");
    }
    else
    {
      /* 完整 mV 数值，不截高位（B OVF 时也在此显示全值） */
      (void)snprintf(line, sizeof(line), "%lu mV", (unsigned long)s->battery_mv);
      draw_centered_6x12(24, line);
    }
  }
}

/* 回到初始位：逐轮 等待/运动/到位/失败；不满足条件时显示原因 */
static void draw_home(const ui_snapshot_t *s)
{
  static const char wheel_names[4] = {'A', 'B', 'C', 'D'};
  const char *reason = NULL;
  uint8_t i;

  oled_gfx_text6x12(0, 0, "GO HOME");
  switch (s->home_block_reason)
  {
    case 1U:
      reason = "NO REF";
      break;
    case 2U:
      reason = "WAIT CENTER";
      break;
    case 3U:
      reason = "FAULT";
      break;
    default:
      reason = NULL;
      break;
  }
  if (reason != NULL)
  {
    oled_gfx_text6x12(54, 0, reason);
  }
  for (i = 0; i < 4U; i++)
  {
    draw_name_row((int16_t)(12 + i * WHEEL_ROW_STEP), wheel_names[i],
                  state_text(s->home_state[i]));
  }
}

/* 手动标定：对齐标记后长按 SELECT 2 s 记录参考 */
static void draw_manual_cal(const ui_snapshot_t *s)
{
  oled_gfx_text6x12(19, 0, "MANUAL CAL");
  draw_centered_6x12(20, "ALIGN WHEEL MARKS");
  draw_centered_6x12(36, "HOLD SELECT 2S");
  if (s->reference_valid != 0U)
  {
    draw_centered_6x12(52, "REF RECORDED");
  }
  else
  {
    draw_centered_6x12(52, "NO REFERENCE");
  }
}

void ui_render_draw(const ui_snapshot_t *snap, uint32_t now_ms)
{
  if (snap == NULL)
  {
    return;
  }
  oled_gfx_clear();
  switch (snap->page)
  {
    case UI_PAGE_BRAND:
      draw_brand(snap);
      break;
    case UI_PAGE_MENU:
      draw_menu(snap, now_ms);
      break;
    case UI_PAGE_SUBMENU:
      draw_submenu(snap);
      break;
    case UI_PAGE_DRIVE:
      draw_drive(snap);
      break;
    case UI_PAGE_CHECK_CONFIRM:
      draw_check_confirm(snap);
      break;
    case UI_PAGE_CHECK_RUN:
      draw_check_run(snap);
      break;
    case UI_PAGE_CHECK_RESULT:
      draw_check_result(snap);
      break;
    case UI_PAGE_HOME:
      draw_home(snap);
      break;
    case UI_PAGE_MANUAL_CAL:
      draw_manual_cal(snap);
      break;
    default:
      /* 未知页面：保持清屏，不越界写缓冲 */
      break;
  }
}
