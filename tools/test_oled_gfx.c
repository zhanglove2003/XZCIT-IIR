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

  /* 实心矩形 4×4 贴左上角 */
  oled_gfx_clear();
  oled_gfx_fill_rect(0, 0, 4, 4);
  assert(oled_gfx_buffer()[0] == 0x0F);
  assert(oled_gfx_buffer()[4] == 0x00);
  assert(fb_bit(3, 3) == 1);
  assert(fb_bit(4, 0) == 0);

  /* 空心矩形 6×6：只有边框 */
  oled_gfx_clear();
  oled_gfx_rect(10, 4, 6, 6);
  assert(fb_bit(10, 4) == 1);
  assert(fb_bit(15, 4) == 1);
  assert(fb_bit(10, 9) == 1);
  assert(fb_bit(15, 9) == 1);
  assert(fb_bit(15, 6) == 1);
  assert(fb_bit(12, 4) == 1);
  assert(fb_bit(12, 5) == 0);
  assert(fb_bit(12, 6) == 0);

  /* 负坐标与越界按像素裁剪，退化尺寸不落笔 */
  oled_gfx_clear();
  oled_gfx_fill_rect(-2, -2, 4, 4);
  assert(fb_bit(0, 0) == 1);
  assert(fb_bit(1, 1) == 1);
  assert(fb_bit(2, 2) == 0);
  oled_gfx_rect(126, 62, 10, 10);
  assert(fb_bit(126, 62) == 1);
  assert(fb_bit(127, 62) == 1);
  assert(fb_bit(126, 63) == 1);
  assert(fb_bit(127, 63) == 0);
  oled_gfx_clear();
  oled_gfx_fill_rect(0, 0, 0, 4);
  oled_gfx_rect(0, 0, 0, 4);
  oled_gfx_rect(0, 0, 1, 1);
  assert(fb_bit(0, 0) == 1);
  assert(fb_bit(1, 0) == 0);

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

  /* 10×16 标题字模：可查得、20 字节内有墨迹、未收录返回 NULL */
  {
    static const char title_chars[] = "XZCIT-MFSC";
    const uint8_t *glyph = oled_font_title_lookup('X');
    int bits = 0;

    assert(glyph != NULL);
    for (i = 0; i < 20; i++)
    {
      bits += __builtin_popcount(glyph[i]);
    }
    assert(bits > 0);
    assert(oled_font_title_lookup('Q') == NULL);
    for (i = 0; title_chars[i] != '\0'; i++)
    {
      assert(oled_font_title_lookup(title_chars[i]) != NULL);
    }
  }

  /* 反白 6×12：底为亮、笔画为暗，与普通绘制逐位互补 */
  oled_gfx_clear();
  oled_gfx_text6x12_inv(0, 0, "AB");
  assert(fb_bit(0, 0) == 1);
  {
    uint8_t inv[24];
    int c;
    int ones = 0;

    for (c = 0; c < 12; c++)
    {
      inv[c] = oled_gfx_buffer()[c];
      inv[12 + c] = oled_gfx_buffer()[OLED_WIDTH + c];
    }
    for (c = 0; c < 24; c++)
    {
      ones += __builtin_popcount(inv[c]);
    }
    assert(ones < 12 * 12); /* 笔画处为暗，不是整块全亮 */

    oled_gfx_clear();
    oled_gfx_text6x12(0, 0, "AB");
    for (c = 0; c < 12; c++)
    {
      /* 页 0：12 行区域的前 8 行，整字节互补 */
      assert((uint8_t)(inv[c] ^ oled_gfx_buffer()[c]) == 0xFF);
      /* 页 1：仅低 4 位属于 12 行区域 */
      assert((uint8_t)((inv[12 + c] ^ oled_gfx_buffer()[OLED_WIDTH + c]) & 0x0F) == 0x0F);
    }
  }

  puts("oled_gfx OK");
  return 0;
}
