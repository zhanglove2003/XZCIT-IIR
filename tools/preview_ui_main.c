/* 预览工具主机构建入口：按页面名构造 ui_snapshot_t 并调用 ui_render_draw，
 * 把 1 KB 帧缓冲（page 格式，与 oled_gfx_buffer 布局一致）原样写到 stdout，
 * 由 tools/render_preview.py 读入并转成放大 PNG。
 * 链接方式与 tools/test_ui_render.c 相同（无 HAL/RTOS 依赖）：
 * gcc -std=c11 -Wall -Wextra -Werror -ICore/Inc tools/preview_ui_main.c
 *     Core/Src/ui_render.c Core/Src/oled_gfx.c Core/Src/oled_font.c
 *     Core/Src/ui_model.c -o preview */
#include <stdio.h>
#include <string.h>

#include "oled_gfx.h"
#include "ui_model.h"
#include "ui_render.h"

int main(int argc, char **argv)
{
  ui_snapshot_t s;
  const char *name;

  if (argc != 2)
  {
    fprintf(stderr, "usage: preview brand|confirm|submenu\n");
    return 2;
  }
  name = argv[1];
  memset(&s, 0, sizeof(s));
  s.frame_valid = 1U;
  s.vdd_ok = 1U; /* 缺省渲染正常提示行；异常提示由目检者另行在板上看 */
  if (strcmp(name, "brand") == 0)
  {
    s.page = UI_PAGE_BRAND;
  }
  else if (strcmp(name, "confirm") == 0)
  {
    s.page = UI_PAGE_CHECK_CONFIRM;
    s.confirm_remain_ms = 5000U;
  }
  else if (strcmp(name, "submenu") == 0)
  {
    s.page = UI_PAGE_SUBMENU;
    s.sub_sel = 0U;
    s.sub_scroll = 0U;
  }
  else
  {
    fprintf(stderr, "unknown page: %s\n", name);
    return 2;
  }
  /* now_ms 取 1200：距动画锚点足够远，滑动/跳动偏移均为 0 的稳定帧 */
  ui_render_draw(&s, 1200U);
  fwrite(oled_gfx_buffer(), 1U, OLED_WIDTH * OLED_HEIGHT / 8U, stdout);
  return 0;
}
