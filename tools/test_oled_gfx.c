#include <assert.h>
#include <stdio.h>
#include "oled_gfx.h"
#include "oled_font.h"

static int fb_bit(uint8_t x, uint8_t y)
{
  return (oled_gfx_buffer()[(y >> 3) * OLED_WIDTH + x] >> (y & 7)) & 1;
}

int main(void)
{
  int ink;
  int i;

  oled_gfx_clear();
  assert(fb_bit(0, 0) == 0);

  oled_gfx_pixel(0, 0, 1);
  assert(oled_gfx_buffer()[0] == 0x01);
  oled_gfx_pixel(127, 63, 1);
  assert(oled_gfx_buffer()[8 * 128 - 1] == 0x80);

  oled_gfx_pixel(-1, 5, 1);
  oled_gfx_pixel(200, 5, 1);
  oled_gfx_pixel(5, 64, 1);
  assert(oled_gfx_buffer()[0] == 0x01);

  /* 8×8 位图贴在 y=4：跨两页，验证按像素落位 */
  oled_gfx_clear();
  {
    static const uint8_t bmp[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    oled_gfx_bitmap(0, 4, 8, 8, bmp);
    assert(oled_gfx_buffer()[0] == 0xF0);
    assert(oled_gfx_buffer()[OLED_WIDTH] == 0x0F);
  }

  /* ASCII '0'：与生成器验证过的 page 字节一致 */
  oled_gfx_clear();
  oled_gfx_text6x12(0, 0, "0");
  {
    static const uint8_t expect0[12] = {0xF8, 0x04, 0x04, 0x04, 0xF8, 0x00,
                                        0x01, 0x02, 0x02, 0x02, 0x01, 0x00};
    int c;
    for (c = 0; c < 6; c++)
    {
      assert(oled_gfx_buffer()[c] == expect0[c]);
      assert(oled_gfx_buffer()[OLED_WIDTH + c] == expect0[6 + c]);
    }
  }

  assert(oled_gfx_text6x12_width("RF") == 12);
  assert(oled_gfx_text6x12_width("") == 0);

  /* 汉字：16 px + 2 px 字距，且有墨迹 */
  assert(oled_gfx_text16_width("电机") == 34);
  oled_gfx_clear();
  oled_gfx_text16(0, 0, "电机");
  ink = 0;
  for (i = 0; i < (int)(OLED_WIDTH * OLED_HEIGHT / 8); i++)
  {
    ink += __builtin_popcount(oled_gfx_buffer()[i]);
  }
  assert(ink > 20);

  /* 未收录的字符不崩溃，按 6 px 跳过 */
  assert(oled_gfx_text16_width("X") == 6);
  oled_gfx_clear();
  oled_gfx_text16(0, 0, "X");

  puts("oled_gfx OK");
  return 0;
}
