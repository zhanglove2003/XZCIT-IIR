/* ui_render：OLED 各页面绘制。只读快照、不碰硬件、不写任何模型状态；
 * clear + 按页绘制到 oled_gfx 帧缓冲，flush 由渲染循环（boot_ui）负责。
 * 排版数值来自 spec §5/§6；卡片跳动/滑动只由 now_ms 与快照锚点计算。
 * 中文字形 16px 高：只能画进页面主体，顶栏一律 6×12（spec §6 顶栏行高 12px）。 */
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
  /* 8 字符域 "A+999rpm" 封顶：|rpm|>999 一律 OVF，不挤入列间隙 */
  if ((rpm < -999) || (rpm > 999))
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

/* 卡片图标 12×12（page 格式，2 页 × 12 列，bit0 为页内顶行），画进卡片左侧：
 * 驾驶卡 = 方向盘（外圈 + 横向中条 + 下辐条）；
 * 检查标定卡 = 开口扳手（顶部开口钳口 + 锥颈 + 直柄）。 */
static const uint8_t card_icon_drive[24] = {
    0x00, 0xFC, 0x62, 0x61, 0x61, 0xE1, 0xE1, 0x61, 0x61, 0x62, 0xFC, 0x00,
    0x00, 0x03, 0x04, 0x0C, 0x08, 0x0B, 0x0B, 0x08, 0x0C, 0x04, 0x03, 0x00,
};
static const uint8_t card_icon_wrench[24] = {
    0x00, 0x00, 0x08, 0x1F, 0xFF, 0xF8, 0xF8, 0xFF, 0x1F, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00,
};

#define ICON_W 12
#define ICON_H 12
#define ICON_X_OFF 5  /* 图标左缘距卡片左框 */
#define ICON_Y_OFF 10
#define NAME_X_OFF 20 /* 卡片中文名起点距卡片左框（图标右侧） */
#define NAME_Y_OFF 8

/* 逐轮状态短文本（自检页 6×12 数据行用短代码：0 等待 1 运行 2 通过 3 失败；
 * 四轮行行距 12px 放不下 16px 汉字，归位页的中文状态见 home_state_text） */
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

/* 归位页逐轮中文状态（2×2 状态格，16px） */
static const char *home_state_text(uint8_t state)
{
  switch (state)
  {
    case 1U:
      return "运动中";
    case 2U:
      return "到位";
    case 3U:
      return "失败";
    default:
      return "等待";
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

/* ---------------- 混排绘制（16px 汉字 + 6×12 ASCII 同行） ----------------
 * 汉字步进 18px（16px 字形 + 2px 字距），ASCII 步进 6px、在 16px 行内垂直居中
 * （y+2 起，12px 高）。用于"R1 菜单 L1 自检"、按USER取消 等中英混排提示行。 */
#define MIX_CN_STEP    18U
#define MIX_ASCII_STEP 6U

static uint16_t mixed16_width(const char *utf8)
{
  const uint8_t *p = (const uint8_t *)utf8;
  uint16_t width = 0U;
  uint8_t last_cn = 0U;

  while (*p != 0U)
  {
    if (*p < 0x80U)
    {
      p++;
      width = (uint16_t)(width + MIX_ASCII_STEP);
      last_cn = 0U;
    }
    else if ((*p & 0xF0U) == 0xE0U && p[1] != 0U && p[2] != 0U)
    {
      p += 3;
      width = (uint16_t)(width + MIX_CN_STEP);
      last_cn = 1U;
    }
    else
    {
      p++;
    }
  }
  /* 末尾汉字字距（18 - 16）不计入宽度，便于居中（与 oled_gfx_text16_width 一致） */
  if ((last_cn != 0U) && (width >= (MIX_CN_STEP - 16U)))
  {
    width = (uint16_t)(width - (MIX_CN_STEP - 16U));
  }
  return width;
}

static void draw_mixed16(int16_t x, int16_t y, const char *utf8)
{
  const uint8_t *p = (const uint8_t *)utf8;

  while (*p != 0U)
  {
    if (*p < 0x80U)
    {
      char c = (char)*p;
      char buf[2];

      if (c < ' ' || c > '~')
      {
        c = ' ';
      }
      buf[0] = c;
      buf[1] = '\0';
      oled_gfx_text6x12(x, (int16_t)(y + 2), buf);
      p++;
      x = (int16_t)(x + MIX_ASCII_STEP);
    }
    else if ((*p & 0xF0U) == 0xE0U && p[1] != 0U && p[2] != 0U)
    {
      const uint8_t *glyph = oled_font_cn_lookup((const char *)p);

      if (glyph != 0)
      {
        oled_gfx_bitmap(x, y, 16, 16, glyph);
      }
      p += 3;
      x = (int16_t)(x + MIX_CN_STEP);
    }
    else
    {
      p++;
    }
  }
}

/* 16px 中文/混排文本水平居中（16px 字形只能用于页面主体，不得进驾驶顶栏） */
static void draw_centered_mixed16(int16_t y, const char *utf8)
{
  draw_mixed16((int16_t)(((int16_t)OLED_WIDTH - (int16_t)mixed16_width(utf8)) / 2), y, utf8);
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

/* 品牌页：10×16 标题 + 6×12 副标题 + 底部 16px 中文提示/最高优先级异常行 */
static void draw_brand(const ui_snapshot_t *s)
{
  const char *hint;

  draw_title10(14, 14, "XZCIT-MFSC");
  oled_gfx_text6x12(28, 36, "By Plucksnow");

  if (s->fault_mask != 0U)
  {
    hint = "电机故障";
  }
  else if (s->frame_valid == 0U)
  {
    hint = "手柄未连接";
  }
  else if (s->undervolt != 0U)
  {
    hint = "电池欠压";
  }
  else if (s->vdd_ok == 0U)
  {
    hint = "供电不足";
  }
  else
  {
    hint = "R1 菜单 L1 自检";
  }
  draw_centered_mixed16(48, hint);
}

/* 主菜单：两张 92×32 卡片（图标 + 中文名），选中卡片居中轻跳，相邻卡片屏幕边缘露出 */
static void draw_menu(const ui_snapshot_t *s, uint32_t now_ms)
{
  static const char *const card_names[UI_MENU_COUNT] = {"驾驶", "检查标定"};
  static const uint8_t *const card_icons[UI_MENU_COUNT] = {
    card_icon_drive, card_icon_wrench
  };
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

    /* 图标：12×12 位图，卡片左侧（越出屏幕的部分由 oled_gfx 裁剪） */
    oled_gfx_bitmap((int16_t)(cx + ICON_X_OFF), (int16_t)(cy + ICON_Y_OFF),
                    ICON_W, ICON_H, card_icons[i]);
    /* 中文名：16px 字形，垂直居中（cy+8..cy+23），最右字尾 cx+20+70=cx+90 < 右框 cx+91 */
    oled_gfx_text16((int16_t)(cx + NAME_X_OFF), (int16_t)(cy + NAME_Y_OFF), card_names[i]);
    /* 选中/未选中卡片均为 1px 边框；选中边框随跳动偏移 */
    oled_gfx_rect(cx, cy, CARD_W, CARD_H);
  }
}

/* 子菜单：16px 标题 + 三行 16px 中文条目反白列表（行高 16、行距 0，
 * 标题 y0..15 + 窗口 y16..63 恰好占满 64px，滚动档位 16px）+ 窄滚动标记 */
static void draw_submenu(const ui_snapshot_t *s)
{
  static const char *const items[UI_SUB_COUNT] = {
    "自检结果", "重新自检", "回到初始位", "手动标定"
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
    y = (int16_t)(16 + r * 16);
    if (idx == s->sub_sel)
    {
      oled_gfx_text16_inv(0, y, items[idx]);
    }
    else
    {
      oled_gfx_text16(0, y, items[idx]);
    }
  }
  /* 窄滚动标记：x=126 一列，拇指位置随 sub_scroll 平移（16px/档） */
  thumb_y = (int16_t)(16 + (int16_t)s->sub_scroll * 16);
  oled_gfx_fill_rect(126, thumb_y, 1, 16);
}

/* 驾驶仪表盘：顶栏 + 四轮行（固定数据布局，无动画）。
 * 未通过自检时正文改为居中中文提示"请先自检"（16px，只能放页面主体，
 * 不得进顶栏——顶栏行高仅 12px），四轮数据此时未采认，不绘制。 */
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

  /* 顶栏中部：行驶状态（6×12，x42..83 域内） */
  status = drive_status_text(s->drive_status);
  oled_gfx_text6x12(42, 0, status);

  /* 顶栏右侧优先级（spec §6）：故障 > 失联 > 等待回中 > 无参考 > REF。
   * 未通过自检的提示占正文时，右侧仅在故障/失联时覆盖显示 */
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

  if (hint != 0U)
  {
    /* 中文提示居中于正文（"请先自检" 4 字 = 70px，x=29..98，y=26..41） */
    draw_centered_mixed16(26, "请先自检");
    return;
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

/* 自检启动确认：四条 16px 横带占满 64px（标题/倒计时/按键提示×2，行距 0）。
 * 第二行为 剩余秒数（6×12）+ "秒后自动开始"；前置不满足时冻结并把这半行
 * 换成原因（手柄未连接/请回中摇杆/供电不足），数字保留冻结值。 */
static void draw_check_confirm(const ui_snapshot_t *s)
{
  char digit[3];
  const char *row2;
  uint16_t w;
  int16_t x;

  draw_centered_mixed16(0, "即将转动四轮");

  (void)snprintf(digit, sizeof(digit), "%u",
                 (unsigned)(s->confirm_remain_ms / 1000U));
  if (s->confirm_blocked != 0U)
  {
    if (s->frame_valid == 0U)
    {
      row2 = "手柄未连接";
    }
    else if ((s->vdd_ok == 0U) || (s->undervolt != 0U))
    {
      row2 = "供电不足";
    }
    else
    {
      row2 = "请回中摇杆";
    }
  }
  else
  {
    row2 = "秒后自动开始";
  }
  /* 数字（6px）+ 2px 间隙 + 汉字行，整组水平居中：数字与汉字不重叠 */
  w = (uint16_t)(MIX_ASCII_STEP + 2U + mixed16_width(row2));
  x = (int16_t)(((int16_t)OLED_WIDTH - (int16_t)w) / 2);
  oled_gfx_text6x12(x, 18, digit);
  draw_mixed16((int16_t)(x + MIX_ASCII_STEP + 2), 16, row2);

  draw_centered_mixed16(32, "按USER取消");
  draw_centered_mixed16(48, "右移或L1立即");
}

/* 自检进度：16px 标题"正在自检" + 6×12 逐轮短代码行；
 * 动作中右上角转圈动画（12×12，100 ms/帧），与标题水平不重叠 */
static void draw_check_run(const ui_snapshot_t *s, uint32_t now_ms)
{
  static const char wheel_names[4] = {'A', 'B', 'C', 'D'};
  uint8_t i;

  draw_centered_mixed16(0, "正在自检");
  for (i = 0; i < 4U; i++)
  {
    draw_name_row((int16_t)(16 + i * WHEEL_ROW_STEP), wheel_names[i],
                  state_text(s->selfcheck_state[i]));
  }
  if (s->action_busy != 0U)
  {
    oled_gfx_bitmap(114, 0, 12, 12, oled_font_spinner[(now_ms / 100U) % 8U]);
  }
}

/* 自检结果两屏：0=电机四轮 1=电池；左右翻屏由 ui_model 切 result_view。
 * 标题行左侧 16px 屏名，右侧 16px 总体结论（全部通过/未通过）。 */
static void draw_check_result(const ui_snapshot_t *s)
{
  static const char wheel_names[4] = {'A', 'B', 'C', 'D'};
  char line[16];
  const char *verdict;
  uint8_t i;

  draw_mixed16(0, 0, (s->result_view == 0U) ? "电机" : "电池");
  verdict = (s->selfcheck_passed != 0U) ? "全部通过" : "未通过";
  draw_mixed16((int16_t)((int16_t)OLED_WIDTH - (int16_t)mixed16_width(verdict)), 0, verdict);

  if (s->result_view == 0U)
  {
    for (i = 0; i < 4U; i++)
    {
      /* 快照无逐轮原因文本：失败只显示 FAIL，逐轮原因由 motor_test 日志查看 */
      draw_name_row((int16_t)(16 + i * WHEEL_ROW_STEP), wheel_names[i],
                    state_text(s->selfcheck_state[i]));
    }
  }
  else
  {
    if (s->battery_valid == 0U)
    {
      draw_centered_mixed16(28, "未配置");
    }
    else if (s->battery_error != 0U)
    {
      draw_centered_mixed16(28, "采样错误");
    }
    else if (s->undervolt != 0U)
    {
      draw_centered_mixed16(28, "欠压");
    }
    else
    {
      /* 完整 mV 数值，不截高位（B OVF 时也在此显示全值） */
      (void)snprintf(line, sizeof(line), "%lu mV", (unsigned long)s->battery_mv);
      draw_centered_6x12(30, line);
    }
  }
}

/* 回到初始位：16px 标题 + 阻塞原因行（无参考/请回中/故障）+
 * 2×2 四轮状态格（16px 中文状态：等待/运动中/到位/失败）。
 * 四轮各 16px 状态无法排进 12px 行距的单列，故改两列两行网格。 */
static void draw_home(const ui_snapshot_t *s)
{
  static const char wheel_names[4] = {'A', 'B', 'C', 'D'};
  const char *reason = NULL;
  uint8_t i;

  draw_centered_mixed16(0, "回到初始位");
  switch (s->home_block_reason)
  {
    case 1U:
      reason = "无参考";
      break;
    case 2U:
      reason = "请回中";
      break;
    case 3U:
      reason = "故障";
      break;
    default:
      reason = NULL;
      break;
  }
  if (reason != NULL)
  {
    draw_centered_mixed16(16, reason);
  }
  for (i = 0; i < 4U; i++)
  {
    int16_t x = ((i % 2U) != 0U) ? 64 : 0; /* A B / C D 两列 */
    int16_t y = (i < 2U) ? 32 : 48;
    char buf[2];

    buf[0] = wheel_names[i];
    buf[1] = '\0';
    oled_gfx_text6x12(x, (int16_t)(y + 2), buf);
    draw_mixed16((int16_t)(x + 8), y, home_state_text(s->home_state[i]));
  }
}

/* 手动标定：四条 16px 横带（标题/对齐四轮/静止后长按SELECT/已记录或无参考） */
static void draw_manual_cal(const ui_snapshot_t *s)
{
  draw_centered_mixed16(0, "手动标定");
  draw_centered_mixed16(16, "对齐四轮");
  draw_centered_mixed16(32, "静止后长按SELECT");
  if (s->reference_valid != 0U)
  {
    draw_centered_mixed16(48, "已记录");
  }
  else
  {
    draw_centered_mixed16(48, "无参考");
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
      draw_check_run(snap, now_ms);
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
