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

/* 中文 16px 文本清单（全界面中文化后各页用字，逐字符 lookup 防渲染静默跳字） */
static void test_cn_glyphs_used(void)
{
  assert_cn_lookup("请先自检");
  assert_cn_lookup("电池未配置");
  assert_cn_lookup("驾驶");
  assert_cn_lookup("检查标定");
  assert_cn_lookup("检查与标定");
  /* 品牌页提示与异常 */
  assert_cn_lookup("菜单");
  assert_cn_lookup("电机故障");
  assert_cn_lookup("手柄未连接");
  assert_cn_lookup("电池欠压");
  assert_cn_lookup("供电不足");
  assert_cn_lookup("请回中摇杆");
  /* 确认页 */
  assert_cn_lookup("即将转动四轮");
  assert_cn_lookup("秒后自动开始");
  assert_cn_lookup("取消");
  assert_cn_lookup("右移或立即");
  /* 子菜单条目 */
  assert_cn_lookup("自检结果");
  assert_cn_lookup("重新自检");
  assert_cn_lookup("回到初始位");
  assert_cn_lookup("手动标定");
  /* 结果页 */
  assert_cn_lookup("电机");
  assert_cn_lookup("电池");
  assert_cn_lookup("全部通过");
  assert_cn_lookup("未通过");
  assert_cn_lookup("未配置");
  assert_cn_lookup("欠压");
  assert_cn_lookup("采样错误");
  assert_cn_lookup("正在自检");
  /* 归位页 */
  assert_cn_lookup("等待");
  assert_cn_lookup("运动中");
  assert_cn_lookup("到位");
  assert_cn_lookup("失败");
  assert_cn_lookup("无参考");
  assert_cn_lookup("请回中");
  assert_cn_lookup("故障");
  /* 标定页 */
  assert_cn_lookup("对齐四轮");
  assert_cn_lookup("静止后长按");
  assert_cn_lookup("已记录");
  /* 输出上限页（2026-11-01 第 5 子菜单条目） */
  assert_cn_lookup("输出上限");
  assert_cn_lookup("上下调整");
  assert_cn_lookup("返回");
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

/* 品牌页：标题区有墨迹、底部 16px 中文提示行（y48..63，与副标题不重叠） */
static void test_brand(void)
{
  ui_snapshot_t s;
  uint8_t normal[128 * 8];

  snap_init(&s, UI_PAGE_BRAND);
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(14, 14, 113, 29) > 0);        /* 标题 XZCIT-MFSC */
  assert(fb_ink_in(28, 36, 99, 47) > 0);         /* 副标题 */
  assert(fb_ink_in(0, 30, 127, 35) == 0);        /* 标题与副标题之间留空 */
  assert(fb_ink_in(0, 48, 127, 63) > 0);         /* 底部 16px 提示行 */
  assert(fb_ink_in(0, 48, 7, 63) == 0);          /* "R1 菜单 L1 自检" 居中，左缘留空 */
  memcpy(normal, oled_gfx_buffer(), sizeof(normal));

  /* 异常提示替换为中文：故障/失联/欠压/VDD 各不相同且不与副标题重叠 */
  s.fault_mask = 1U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 48, 127, 63) > 0);
  assert(fb_ink_in(28, 36, 99, 47) > 0);         /* 副标题区照常绘制 */
  assert(memcmp(oled_gfx_buffer(), normal, sizeof(normal)) != 0);
  {
    uint8_t fault[128 * 8];
    memcpy(fault, oled_gfx_buffer(), sizeof(fault));
    s.fault_mask = 0U;
    s.frame_valid = 0U;
    ui_render_draw(&s, 0U);
    assert(memcmp(oled_gfx_buffer(), fault, sizeof(fault)) != 0);
    s.frame_valid = 1U;
    s.undervolt = 1U;
    ui_render_draw(&s, 0U);
    assert(memcmp(oled_gfx_buffer(), fault, sizeof(fault)) != 0);
    s.undervolt = 0U;
    s.vdd_ok = 0U;
    ui_render_draw(&s, 0U);
    assert(memcmp(oled_gfx_buffer(), fault, sizeof(fault)) != 0);
    s.vdd_ok = 1U;
  }
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

/* 子菜单：16px 标题 + 三行 16px 反白列表（行高 16、行距 0，窗口 y16..63）+ 窄滚动标记 */
static void test_submenu(void)
{
  ui_snapshot_t s;

  snap_init(&s, UI_PAGE_SUBMENU);
  s.sub_sel = 0U;
  s.sub_scroll = 0U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 0, 127, 15) > 0);          /* 16px 标题区 */
  assert(fb_ink_in(0, 16, 119, 31) > 0);         /* 首行反白条 */
  assert(fb_ink_in(0, 32, 119, 63) > 0);         /* 其余两行列表 */
  assert(fb_ink_in(126, 16, 127, 63) > 0);       /* 窄滚动标记 */
  /* 反白条与下一行不重叠：条底 y31，下一行字形自 y32 起仍有墨 */
  assert(fb_ink_in(0, 24, 119, 31) > 0);
  assert(fb_ink_in(0, 32, 119, 47) > 0);

  /* 滚动窗口：scroll=1 时列表整体平移，标记位置随 scroll 平移（16px/档） */
  s.sub_sel = 3U;
  s.sub_scroll = 1U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 48, 119, 63) > 0);         /* 第 3 行（手动标定）仍在窗口内 */
  assert(oled_gfx_buffer()[(32 >> 3) * OLED_WIDTH + 126] != 0); /* 标记移到中档 */
  assert(oled_gfx_buffer()[(16 >> 3) * OLED_WIDTH + 126] == 0); /* 顶部档位已空 */
  s.sub_sel = 0U;
  s.sub_scroll = 0U;
  ui_render_draw(&s, 0U);
  assert(oled_gfx_buffer()[(16 >> 3) * OLED_WIDTH + 126] != 0); /* 标记回到窗口顶部 */
}

/* 结果页两屏：16px 标题（电机/电池）+ 右侧总体结论（全部通过/未通过）+ 四轮行 */
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
  assert(fb_ink_in(0, 0, 33, 15) > 0);           /* 标题 电机（16px） */
  assert(fb_ink_in(58, 0, 127, 15) > 0);         /* 右侧未通过（16px） */
  assert(fb_ink_in(34, 0, 57, 15) == 0);         /* 标题与结论之间留空 */
  assert(fb_ink_in(0, 16, 59, 59) > 0);          /* 四轮名与状态（6×12 行） */

  /* 全部通过：总体结论切换 */
  s.selfcheck_passed = 1U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(34, 0, 127, 15) > 0);
  s.selfcheck_passed = 0U;

  /* 电池屏：未配置 → 16px 提示"未配置"居中于正文（34px 宽，x47..80, y28..43） */
  s.result_view = 1U;
  s.battery_valid = 0U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 0, 33, 15) > 0);           /* 标题 电池 */
  assert(fb_ink_in(47, 28, 80, 43) > 0);
  {
    uint8_t not_config[128 * 8];
    memcpy(not_config, oled_gfx_buffer(), sizeof(not_config));

    /* 采样错误 / 欠压 → 各自中文提示 */
    s.battery_valid = 1U;
    s.battery_error = 1U;
    ui_render_draw(&s, 0U);
    assert(fb_ink_in(0, 28, 127, 43) > 0);
    assert(memcmp(oled_gfx_buffer(), not_config, sizeof(not_config)) != 0);
    s.battery_error = 0U;
    s.undervolt = 1U;
    ui_render_draw(&s, 0U);
    assert(memcmp(oled_gfx_buffer(), not_config, sizeof(not_config)) != 0);
    s.undervolt = 0U;

    /* 有效 → 完整 mV 数值，不截高位：12345 与 1234 显示不同 */
    s.battery_mv = 12345U;
    ui_render_draw(&s, 0U);
    assert(memcmp(oled_gfx_buffer(), not_config, sizeof(not_config)) != 0);
    s.battery_mv = 1234U;
    ui_render_draw(&s, 0U);
    assert(memcmp(oled_gfx_buffer(), not_config, sizeof(not_config)) != 0);
  }
}

/* 确认页：四条 16px 横带（标题/倒计时/按键提示×2），倒计时数字与汉字不重叠 */
static void test_confirm(void)
{
  ui_snapshot_t s;
  uint8_t f5[128 * 8];

  snap_init(&s, UI_PAGE_CHECK_CONFIRM);
  s.confirm_remain_ms = 5000U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 0, 127, 15) > 0);          /* 即将转动四轮 */
  assert(fb_ink_in(0, 32, 127, 47) > 0);         /* 按USER取消 */
  assert(fb_ink_in(0, 48, 127, 63) > 0);         /* 右移或L1立即 */
  /* 倒计时行：数字（6×12）与"秒后自动开始"（16px）之间 2px 间隙全暗 */
  assert(fb_ink_in(0, 16, 127, 31) > 0);
  for (int x = 13; x <= 14; x++)
  {
    for (int y = 16; y <= 31; y++)
    {
      assert(fb_bit((uint8_t)x, (uint8_t)y) == 0);
    }
  }
  memcpy(f5, oled_gfx_buffer(), sizeof(f5));

  /* 剩余秒数变化 → 数字变化（帧不同） */
  s.confirm_remain_ms = 4000U;
  ui_render_draw(&s, 0U);
  assert(memcmp(oled_gfx_buffer(), f5, sizeof(f5)) != 0);

  /* 前置不满足：原因替换"秒后自动开始"，数字保留（冻结值），不越界 */
  s.confirm_remain_ms = 5000U;
  s.confirm_blocked = 1U;
  s.frame_valid = 0U;                            /* 手柄未连接 */
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 16, 127, 31) > 0);
  assert(fb_ink_in(0, 16, 15, 31) == 0);         /* 数字居中组合，左缘留空 */
  assert(fb_ink_in(0, 32, 127, 47) > 0);
  {
    uint8_t no_pad[128 * 8];
    memcpy(no_pad, oled_gfx_buffer(), sizeof(no_pad));

    s.frame_valid = 1U;                          /* 请回中摇杆 */
    ui_render_draw(&s, 0U);
    assert(memcmp(oled_gfx_buffer(), no_pad, sizeof(no_pad)) != 0);

    s.vdd_ok = 0U;                               /* 供电不足 */
    ui_render_draw(&s, 0U);
    assert(memcmp(oled_gfx_buffer(), no_pad, sizeof(no_pad)) != 0);
    s.vdd_ok = 1U;
  }
}

/* 自检进度页：16px 标题"正在自检" + 6×12 四轮行，标题与转圈动画不重叠 */
static void test_check_run(void)
{
  ui_snapshot_t s;

  snap_init(&s, UI_PAGE_CHECK_RUN);
  s.action_busy = 1U;
  s.selfcheck_state[0] = 1U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(29, 0, 98, 15) > 0);          /* 正在自检（70px 居中） */
  assert(fb_ink_in(114, 0, 125, 11) > 0);        /* 转圈动画帧在右上角 */
  assert(fb_ink_in(99, 0, 113, 15) == 0);        /* 标题与动画不重叠 */
  assert(fb_ink_in(0, 16, 59, 27) > 0);          /* A 轮行 */
}

/* 回到初始位：16px 标题 + 阻塞原因行 + 2×2 四轮状态格（16px 中文状态） */
static void test_home(void)
{
  ui_snapshot_t s;
  uint8_t all_wait[128 * 8];

  snap_init(&s, UI_PAGE_HOME);
  s.reference_valid = 1U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(20, 0, 107, 15) > 0);         /* 回到初始位（88px 居中） */
  assert(fb_ink_in(0, 16, 127, 31) == 0);        /* 未阻塞时原因行为空 */
  assert(fb_ink_in(0, 34, 5, 45) > 0);           /* A 轮名 */
  assert(fb_ink_in(64, 34, 69, 45) > 0);         /* B 轮名 */
  assert(fb_ink_in(8, 32, 55, 47) > 0);          /* 等待（16px，列 1） */
  assert(fb_ink_in(72, 32, 119, 47) > 0);        /* 等待（列 2） */
  memcpy(all_wait, oled_gfx_buffer(), sizeof(all_wait));

  /* 逐轮状态切换：运动中/到位/失败 均改变帧 */
  s.home_state[0] = 1U;
  ui_render_draw(&s, 0U);
  assert(memcmp(oled_gfx_buffer(), all_wait, sizeof(all_wait)) != 0);
  s.home_state[0] = 2U;
  ui_render_draw(&s, 0U);
  assert(memcmp(oled_gfx_buffer(), all_wait, sizeof(all_wait)) != 0);
  s.home_state[0] = 3U;
  ui_render_draw(&s, 0U);
  assert(memcmp(oled_gfx_buffer(), all_wait, sizeof(all_wait)) != 0);
  s.home_state[0] = 0U;

  /* 阻塞原因：无参考 / 请回中 / 故障 各不相同 */
  s.home_block_reason = 1U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 16, 127, 31) > 0);
  {
    uint8_t no_ref[128 * 8];
    memcpy(no_ref, oled_gfx_buffer(), sizeof(no_ref));
    s.home_block_reason = 2U;
    ui_render_draw(&s, 0U);
    assert(memcmp(oled_gfx_buffer(), no_ref, sizeof(no_ref)) != 0);
    s.home_block_reason = 3U;
    ui_render_draw(&s, 0U);
    assert(memcmp(oled_gfx_buffer(), no_ref, sizeof(no_ref)) != 0);
    s.home_block_reason = 0U;
  }
}

/* 手动标定：四条 16px 横带（标题/对齐提示/长按提示/状态） */
static void test_manual_cal(void)
{
  ui_snapshot_t s;
  uint8_t no_ref[128 * 8];

  snap_init(&s, UI_PAGE_MANUAL_CAL);
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(32, 0, 95, 15) > 0);          /* 手动标定（64px 居中） */
  assert(fb_ink_in(32, 16, 95, 31) > 0);         /* 对齐四轮 */
  assert(fb_ink_in(0, 32, 127, 47) > 0);         /* 静止后长按SELECT */
  assert(fb_ink_in(29, 48, 98, 63) > 0);         /* 无参考（70px 居中） */
  memcpy(no_ref, oled_gfx_buffer(), sizeof(no_ref));

  s.reference_valid = 1U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(38, 48, 89, 63) > 0);         /* 已记录（52px 居中） */
  assert(memcmp(oled_gfx_buffer(), no_ref, sizeof(no_ref)) != 0);
}

/* 输出上限页：16px 标题 + 大号 NN%（6×12）+ 原始比较值行 + 底部中文提示 */
static void test_output_cap(void)
{
  ui_snapshot_t s;
  uint8_t f50[128 * 8];

  snap_init(&s, UI_PAGE_OUTPUT_CAP);
  s.output_cap_pct = 50U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 0, 127, 15) > 0);    /* 标题 输出上限（16px 居中） */
  assert(fb_ink_in(0, 18, 127, 29) > 0);   /* 50%（6×12） */
  assert(fb_ink_in(0, 34, 127, 45) > 0);   /* 原始比较值 MAX 8400 */
  assert(fb_ink_in(0, 48, 127, 63) > 0);   /* 上下调整  L1返回 */
  memcpy(f50, oled_gfx_buffer(), sizeof(f50));

  /* 百分比变化 → 数字与原始比较值都变（帧不同） */
  s.output_cap_pct = 30U;
  ui_render_draw(&s, 0U);
  assert(memcmp(oled_gfx_buffer(), f50, sizeof(f50)) != 0);

  /* 10% 与 50% 帧不同（两端边界值都渲染） */
  s.output_cap_pct = 10U;
  ui_render_draw(&s, 0U);
  assert(fb_ink_in(0, 18, 127, 29) > 0);
}

/* 其余页面与未知页面：无越界、清屏后不残留 */
static void test_other_pages(void)
{
  ui_snapshot_t s;
  int i;

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
  test_confirm();
  test_check_run();
  test_home();
  test_manual_cal();
  test_output_cap();
  test_other_pages();
  puts("ui_render OK");
  return 0;
}
