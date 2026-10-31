/* ui_render 主机单测：格式化函数精确字符串、帧缓冲排版与动画采样点。
 * 不含 HAL/RTOS；通过 oled_gfx_buffer() 检查帧缓冲。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "oled_font.h"
#include "oled_gfx.h"
#include "ui_model.h"
#include "ui_render.h"

static int fb_bit(uint8_t x, uint8_t y)
{
  return (oled_gfx_buffer()[(y >> 3) * OLED_WIDTH + x] >> (y & 7)) & 1;
}

static int fb_ink_in(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
  int16_t x;
  int16_t y;
  int ink = 0;

  for (y = y0; y <= y1; y++)
  {
    for (x = x0; x <= x1; x++)
    {
      if (x >= 0 && y >= 0 && x < (int16_t)OLED_WIDTH && y < (int16_t)OLED_HEIGHT && fb_bit((uint8_t)x, (uint8_t)y) != 0)
      {
        ink++;
      }
    }
  }
  return ink;
}

static void snap_init(ui_snapshot_t *s, ui_page_t page)
{
  memset(s, 0, sizeof(*s));
  s->page = page;
  s->sel_since_ms = 0U;
  s->frame_valid = 1U;
}

/* 逐字符核对 UTF-8 中文串全部命中 oled_font_cn_lookup（防渲染时静默跳字） */
static void assert_cn_lookup(const char *utf8)
{
  int i;

  for (i = 0; utf8[i] != '\0'; i += 3)
  {
    assert(oled_font_cn_lookup(&utf8[i]) != NULL);
  }
}

/* 中文 16px 文本清单（驾驶页提示/结果页/卡片名/子菜单标题） */
static void test_cn_glyphs_used(void)
{
  assert_cn_lookup("请先自检");
  assert_cn_lookup("电池未配置");
  assert_cn_lookup("驾驶");
  assert_cn_lookup("检查标定");
  assert_cn_lookup("检查与标定");
}

/* 格式化三函数 */
static void test_format(void)
{
  char out[11];

  ui_render_format_battery(out, 1U, 24000U);
  assert(strcmp(out, "B24.0V") == 0);
  ui_render_format_battery(out, 1U, 0U);
  assert(strcmp(out, "B0.0V") == 0);
  ui_render_format_battery(out, 0U, 24000U);
  assert(strcmp(out, "B--.-V") == 0);
  ui_render_format_battery(out, 1U, 1200000U);
  assert(strcmp(out, "B OVF") == 0);

  ui_render_format_rpm(out, 'A', 120, 1U);
  assert(strcmp(out, "A+120rpm") == 0);
  ui_render_format_rpm(out, 'B', -45, 1U);
  assert(strcmp(out, "B-45rpm") == 0);
  ui_render_format_rpm(out, 'C', 0, 1U);
  assert(strcmp(out, "C+0rpm") == 0);
  ui_render_format_rpm(out, 'D', 0, 0U);
  assert(strcmp(out, "D  --") == 0);
  ui_render_format_rpm(out, 'A', 100000, 1U);
  assert(strcmp(out, "A OVF") == 0);
  /* 8 字符域边界：|rpm|<=999 全量显示，>=1000 必须 OVF（且不越界写，ASan 兜底） */
  ui_render_format_rpm(out, 'A', 999, 1U);
  assert(strcmp(out, "A+999rpm") == 0);
  ui_render_format_rpm(out, 'B', -999, 1U);
  assert(strcmp(out, "B-999rpm") == 0);
  ui_render_format_rpm(out, 'C', 1000, 1U);
  assert(strcmp(out, "C OVF") == 0);
  ui_render_format_rpm(out, 'D', -1000, 1U);
  assert(strcmp(out, "D OVF") == 0);
  ui_render_format_rpm(out, 'A', 9999, 1U);
  assert(strcmp(out, "A OVF") == 0);
  ui_render_format_rpm(out, 'B', -9999, 1U);
  assert(strcmp(out, "B OVF") == 0);

  ui_render_format_turns(out, 1500, 1U);
  assert(strcmp(out, "+001.50R") == 0);
  ui_render_format_turns(out, -2750, 1U);
  assert(strcmp(out, "-002.75R") == 0);
  ui_render_format_turns(out, 0, 1U);
  assert(strcmp(out, "+000.00R") == 0);
  ui_render_format_turns(out, 999999, 1U);
  assert(strcmp(out, "+999.99R") == 0);
  ui_render_format_turns(out, 1000000, 1U);
  assert(strcmp(out, "OVF") == 0);
  ui_render_format_turns(out, -1000000, 1U);
  assert(strcmp(out, "OVF") == 0);
  ui_render_format_turns(out, 1500, 0U);
  assert(strcmp(out, "--") == 0);
}

/* 动画采样点 */
static void test_anim(void)
{
  assert(ui_render_hop_offset(0U, 0U) == 0);
  assert(ui_render_hop_offset(119U, 0U) == 0);
  assert(ui_render_hop_offset(150U, 0U) == -1);
  assert(ui_render_hop_offset(300U, 0U) == -2);
  assert(ui_render_hop_offset(450U, 0U) == -1);
  assert(ui_render_hop_offset(500U, 0U) == 0);
  assert(ui_render_hop_offset(600U, 0U) == 0);
  /* 回绕安全：锚点在 now 之前（now 较小）按无符号差值计 */
  assert(ui_render_hop_offset(50U, 0xFFFFFFF0U) == 0);

  assert(ui_render_slide_offset(0U, 0U) == -64);
  assert(ui_render_slide_offset(60U, 0U) == -32);
  assert(ui_render_slide_offset(119U, 0U) == -1);
  assert(ui_render_slide_offset(120U, 0U) == 0);
  assert(ui_render_slide_offset(300U, 0U) == 0);
}

/* 品牌页：标题区有墨迹、底部仅一行提示 */
static void test_brand(void)
{
  ui_snapshot_t s;

  snap_init(&s, UI_PAGE_BRAND);
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(14, 14, 113, 29) > 0);        /* 标题 XZCIT-MFSC */
  assert(fb_ink_in(28, 36, 99, 47) > 0);         /* 副标题 */
  assert(fb_ink_in(0, 48, 127, 51) == 0);        /* 提示行之上留空 */
  assert(fb_ink_in(0, 52, 127, 63) > 0);         /* 底部一行提示 */

  /* 品牌页故障时底部显示异常而非操作提示 */
  s.fault_mask = 1U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 52, 127, 63) > 0);
}

/* 驾驶页：四行布局、列间隙、顶栏优先级、未自检提示 */
static void test_drive(void)
{
  ui_snapshot_t s;
  int x;
  int y;
  uint8_t fault_frame[128 * 8];
  uint8_t ref_frame[128 * 8];

  snap_init(&s, UI_PAGE_DRIVE);
  s.selfcheck_passed = 1U;
  s.drive_permitted = 1U;
  s.reference_valid = 1U;
  s.battery_valid = 1U;
  s.battery_mv = 24000U;
  for (y = 0; y < 4; y++)
  {
    s.wheel_rpm[y] = 120;
    s.wheel_milliturns[y] = 1500;
    s.wheel_sample_valid[y] = 1U;
  }
  ui_render_draw(&s, 0U);

  /* 四轮行均有墨迹，起始 y 固定在 12/24/36/48 */
  for (y = 12; y <= 48; y += 12)
  {
    assert(fb_ink_in(0, (int16_t)y, 59, (int16_t)(y + 11)) > 0);
  }
  /* RPM 域与圈数域之间的列间隙 x=60..65 全暗（y=12..59） */
  for (x = 60; x <= 65; x++)
  {
    for (y = 12; y <= 59; y++)
    {
      assert(fb_bit((uint8_t)x, (uint8_t)y) == 0);
    }
  }
  /* 圈数域有墨迹 */
  assert(fb_ink_in(66, 12, 125, 59) > 0);
  memcpy(ref_frame, oled_gfx_buffer(), sizeof(ref_frame));

  /* 无效采样：RPM 域显示 -- */
  for (y = 0; y < 4; y++)
  {
    s.wheel_sample_valid[y] = 0U;
  }
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(12, 12, 59, 59) > 0);

  /* 顶栏优先级：fault_mask=1 → 故障文本而非 REF */
  for (y = 0; y < 4; y++)
  {
    s.wheel_sample_valid[y] = 1U;
  }
  s.fault_mask = 1U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(90, 0, 125, 11) > 0);
  memcpy(fault_frame, oled_gfx_buffer(), sizeof(fault_frame));
  assert(memcmp(fault_frame, ref_frame, sizeof(fault_frame)) != 0);

  /* 失联优先级次于故障：fault=0、frame_valid=0 → 失联文本 */
  s.fault_mask = 0U;
  s.frame_valid = 0U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(90, 0, 125, 11) > 0);
  assert(memcmp(oled_gfx_buffer(), ref_frame, sizeof(ref_frame)) != 0);
  s.frame_valid = 1U;

  /* 未通过自检且未许可：正文改为居中中文提示"请先自检"（16px，只能放页面主体） */
  s.selfcheck_passed = 0U;
  s.drive_permitted = 0U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(42, 0, 83, 11) > 0);   /* 顶栏中部显示行驶状态（6×12） */
  assert(fb_ink_in(90, 0, 125, 11) == 0); /* 无故障/失联时右栏无覆盖 */
  assert(fb_ink_in(29, 26, 98, 41) > 0);  /* 中文提示正文内居中（x29..98, y26..41） */
  assert(fb_ink_in(0, 12, 127, 25) == 0); /* 提示之上无四轮数据 */
  assert(fb_ink_in(0, 42, 127, 59) == 0); /* 提示之下无四轮数据 */
  /* 提示与故障同现：右栏显示故障，正文提示不变，两段不重叠 */
  s.fault_mask = 1U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(29, 26, 98, 41) > 0);
  assert(fb_ink_in(84, 0, 89, 11) == 0);
  assert(fb_ink_in(90, 0, 125, 11) > 0);
  s.fault_mask = 0U;
  s.selfcheck_passed = 1U;
  s.drive_permitted = 1U;

  /* 圈数越界：显示 OVF 且不挤入间隙列 */
  for (y = 0; y < 4; y++)
  {
    s.wheel_milliturns[y] = 1000000000;
  }
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(66, 12, 125, 59) > 0);
  for (x = 60; x <= 65; x++)
  {
    for (y = 12; y <= 59; y++)
    {
      assert(fb_bit((uint8_t)x, (uint8_t)y) == 0);
    }
  }
}

/* 主菜单：选中卡片边框、图标位图、中文名、未选中卡片对侧露出 */
static void test_menu(void)
{
  ui_snapshot_t s;

  snap_init(&s, UI_PAGE_MENU);
  s.menu_sel = 0U;
  /* now 距锚点 1200 ms：滑动(120)与跳动(600)均已回到 0 偏移 */
  ui_render_draw(&s, 1200U);
  /* 选中卡片 92×32 边框：x=18..109, y=20..51 */
  assert(fb_ink_in(18, 20, 109, 20) == 92);
  assert(fb_ink_in(18, 51, 109, 51) == 92);
  assert(fb_ink_in(18, 20, 18, 51) == 32);
  assert(fb_ink_in(109, 20, 109, 51) == 32);
  /* 未选中卡片（右卡片）在对侧边缘露出 ≥1 列 */
  assert(fb_ink_in(118, 20, 127, 51) > 0);

  /* 图标 12×12 位图：选中卡片内 (23..34, 30..41) 有墨；
   * 图标不越卡片边框：图标四周与边框之间留白全暗 */
  assert(fb_ink_in(23, 30, 34, 41) > 0);
  assert(fb_ink_in(19, 21, 22, 50) == 0);  /* 图标左侧留白 */
  assert(fb_ink_in(35, 21, 37, 50) == 0);  /* 图标与名称之间留白 */
  assert(fb_ink_in(19, 42, 34, 50) == 0);  /* 图标下方留白 */
  assert(fb_ink_in(19, 21, 34, 29) == 0);  /* 图标上方留白 */
  /* 中文名 16px：x38..107, y28..43 有墨，右端不越右框 x=109 */
  assert(fb_ink_in(38, 28, 107, 43) > 0);
  assert(fb_ink_in(108, 21, 108, 50) == 0);

  /* 切到第二卡：扳手图标同样在卡片内、不越框 */
  s.menu_sel = 1U;
  ui_render_draw(&s, 1200U);
  assert(fb_ink_in(18, 20, 109, 51) > 0);
  assert(fb_ink_in(23, 30, 34, 41) > 0);
  assert(fb_ink_in(19, 21, 22, 50) == 0);
  assert(fb_ink_in(35, 21, 37, 50) == 0);
  assert(fb_ink_in(38, 28, 107, 43) > 0);
  /* 左卡片在对侧（屏幕左缘）露出 ≥1 列 */
  assert(fb_ink_in(0, 20, 9, 51) > 0);

  /* 跳动偏移非 0 时边框随 now_ms 移动（600 ms 周期内采样） */
  s.menu_sel = 0U;
  ui_render_draw(&s, 1200U + 300U);
  assert(fb_ink_in(0, 20, 127, 51) > 0);
}

/* 子菜单：16px 标题、反白列表、滚动窗口与窄滚动标记 */
static void test_submenu(void)
{
  ui_snapshot_t s;

  snap_init(&s, UI_PAGE_SUBMENU);
  s.sub_sel = 0U;
  s.sub_scroll = 0U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 0, 127, 15) > 0);          /* 16px 标题区 */
  assert(fb_ink_in(0, 18, 119, 29) > 0);         /* 首行反白条 */
  assert(fb_ink_in(126, 18, 127, 61) > 0);       /* 窄滚动标记 */

  /* 滚动窗口：scroll=1 时列表整体平移，标记位置改变 */
  s.sub_sel = 3U;
  s.sub_scroll = 1U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 46, 119, 57) > 0);
  assert(oled_gfx_buffer()[(48 >> 3) * OLED_WIDTH + 126] != 0); /* 标记移到窗口底部 */
  s.sub_sel = 0U;
  s.sub_scroll = 0U;
  ui_render_draw(&s, 0U);
  assert(oled_gfx_buffer()[(18 >> 3) * OLED_WIDTH + 126] != 0); /* 标记回到窗口顶部 */
}

/* 结果页两屏 */
static void test_result(void)
{
  ui_snapshot_t s;

  snap_init(&s, UI_PAGE_CHECK_RESULT);
  s.selfcheck_state[0] = 2U;
  s.selfcheck_state[1] = 3U;
  s.selfcheck_state[2] = 1U;
  s.selfcheck_state[3] = 0U;
  s.result_view = 0U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 0, 127, 11) > 0);          /* 标题 */
  assert(fb_ink_in(0, 12, 59, 59) > 0);          /* 四轮名与状态 */

  /* 电池屏：未配置 → 中文提示"电池未配置"（16px 居中于正文 x20..107, y24..39） */
  s.result_view = 1U;
  s.battery_valid = 0U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 12, 127, 63) > 0);
  assert(fb_ink_in(20, 24, 107, 39) > 0);
  {
    uint8_t not_config[128 * 8];
    memcpy(not_config, oled_gfx_buffer(), sizeof(not_config));

    /* 有效 → 完整 mV 数值，不截高位：12345 与 1234 显示不同 */
    s.battery_valid = 1U;
    s.battery_mv = 12345U;
    ui_render_draw(&s, 0U);
    assert(memcmp(oled_gfx_buffer(), not_config, sizeof(not_config)) != 0);
    s.battery_mv = 1234U;
    ui_render_draw(&s, 0U);
    assert(memcmp(oled_gfx_buffer(), not_config, sizeof(not_config)) != 0);
  }
}

/* 其余页面与未知页面：无越界、清屏后不残留 */
static void test_other_pages(void)
{
  ui_snapshot_t s;
  int i;

  snap_init(&s, UI_PAGE_CHECK_CONFIRM);
  s.confirm_focus = 0U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 0, 127, 63) > 0);

  snap_init(&s, UI_PAGE_CHECK_RUN);
  s.action_busy = 1U;
  s.selfcheck_state[0] = 1U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 0, 127, 63) > 0);
  assert(fb_ink_in(114, 0, 125, 11) > 0); /* 转圈动画帧在右上角 */

  snap_init(&s, UI_PAGE_HOME);
  s.reference_valid = 1U;
  s.home_state[0] = 2U;
  s.home_state[1] = 1U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 0, 127, 63) > 0);

  snap_init(&s, UI_PAGE_MANUAL_CAL);
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 0, 127, 63) > 0);

  /* 未知 page：不越界写缓冲（ASan 兜底），缓冲保持清屏状态 */
  snap_init(&s, (ui_page_t)99);
  ui_render_draw(&s, 0U);
  for (i = 0; i < (int)(OLED_WIDTH * OLED_HEIGHT / 8); i++)
  {
    assert(oled_gfx_buffer()[i] == 0);
  }
}

int main(void)
{
  test_format();
  test_anim();
  test_cn_glyphs_used();
  test_brand();
  test_drive();
  test_menu();
  test_submenu();
  test_result();
  test_other_pages();
  puts("ui_render OK");
  return 0;
}
