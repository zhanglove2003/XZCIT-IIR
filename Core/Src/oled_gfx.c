#include "oled_gfx.h"

#include <string.h>

#include "oled_font.h"

#define TEXT16_STEP 18U

static uint8_t s_fb[OLED_HEIGHT / 8U][OLED_WIDTH];

void oled_gfx_clear(void)
{
  memset(s_fb, 0, sizeof(s_fb));
}

void oled_gfx_pixel(int16_t x, int16_t y, uint8_t on)
{
  if (x < 0 || y < 0 || x >= (int16_t)OLED_WIDTH || y >= (int16_t)OLED_HEIGHT)
  {
    return;
  }
  if (on)
  {
    s_fb[y >> 3][x] |= (uint8_t)(1U << (y & 7));
  }
  else
  {
    s_fb[y >> 3][x] &= (uint8_t)~(1U << (y & 7));
  }
}

static void bitmap_draw(int16_t x, int16_t y, uint8_t w, uint8_t h, const uint8_t *data, uint8_t on)
{
  uint8_t pages = (uint8_t)((h + 7U) / 8U);
  uint8_t page;
  uint8_t col;
  uint8_t bit;

  for (page = 0; page < pages; page++)
  {
    for (col = 0; col < w; col++)
    {
      uint8_t bits = data[page * w + col];
      for (bit = 0; bit < 8U; bit++)
      {
        uint8_t row = (uint8_t)(page * 8U + bit);
        if (row < h && (bits & (1U << bit)))
        {
          oled_gfx_pixel((int16_t)(x + col), (int16_t)(y + row), on);
        }
      }
    }
  }
}

void oled_gfx_bitmap(int16_t x, int16_t y, uint8_t w, uint8_t h, const uint8_t *data)
{
  bitmap_draw(x, y, w, h, data, 1U);
}

void oled_gfx_rect(int16_t x, int16_t y, int16_t w, int16_t h)
{
  int16_t col;
  int16_t row;

  if (w <= 0 || h <= 0)
  {
    return;
  }
  for (col = 0; col < w; col++)
  {
    oled_gfx_pixel((int16_t)(x + col), y, 1);
    oled_gfx_pixel((int16_t)(x + col), (int16_t)(y + h - 1), 1);
  }
  for (row = 1; row < (int16_t)(h - 1); row++)
  {
    oled_gfx_pixel(x, (int16_t)(y + row), 1);
    oled_gfx_pixel((int16_t)(x + w - 1), (int16_t)(y + row), 1);
  }
}

void oled_gfx_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h)
{
  int16_t col;
  int16_t row;

  for (row = 0; row < h; row++)
  {
    for (col = 0; col < w; col++)
    {
      oled_gfx_pixel((int16_t)(x + col), (int16_t)(y + row), 1);
    }
  }
}

void oled_gfx_text6x12(int16_t x, int16_t y, const char *ascii)
{
  char c;

  while (*ascii != '\0')
  {
    c = *ascii++;
    if (c < ' ' || c > '~')
    {
      c = ' ';
    }
    oled_gfx_bitmap(x, y, 6, 12, oled_font_ascii[(uint8_t)c - 32U]);
    x = (int16_t)(x + 6);
  }
}

uint16_t oled_gfx_text6x12_width(const char *ascii)
{
  return (uint16_t)(strlen(ascii) * 6U);
}

void oled_gfx_text6x12_inv(int16_t x, int16_t y, const char *ascii)
{
  int16_t cx = x;
  char c;

  oled_gfx_fill_rect(x, y, (int16_t)oled_gfx_text6x12_width(ascii), 12);
  while (*ascii != '\0')
  {
    c = *ascii++;
    if (c < ' ' || c > '~')
    {
      c = ' ';
    }
    bitmap_draw(cx, y, 6, 12, oled_font_ascii[(uint8_t)c - 32U], 0U);
    cx = (int16_t)(cx + 6);
  }
}

void oled_gfx_text16(int16_t x, int16_t y, const char *utf8)
{
  const uint8_t *p = (const uint8_t *)utf8;

  while (*p != 0U)
  {
    if (*p < 0x80U)
    {
      p++;
      x = (int16_t)(x + 6);
    }
    else if ((*p & 0xF0U) == 0xE0U && p[1] != 0U && p[2] != 0U)
    {
      const uint8_t *glyph = oled_font_cn_lookup((const char *)p);
      if (glyph != 0)
      {
        oled_gfx_bitmap(x, y, 16, 16, glyph);
      }
      p += 3;
      x = (int16_t)(x + TEXT16_STEP);
    }
    else
    {
      p++;
    }
  }
}

/* 16×16 汉字反白：白底条 + 黑字（子菜单选中行用）。
 * 未收录字形跳过（条底仍填充），ASCII 只推进光标，与 oled_gfx_text16 一致。 */
void oled_gfx_text16_inv(int16_t x, int16_t y, const char *utf8)
{
  int16_t cx = x;
  const uint8_t *p = (const uint8_t *)utf8;

  oled_gfx_fill_rect(x, y, (int16_t)oled_gfx_text16_width(utf8), 16);
  while (*p != 0U)
  {
    if ((*p & 0xF0U) == 0xE0U && p[1] != 0U && p[2] != 0U)
    {
      const uint8_t *glyph = oled_font_cn_lookup((const char *)p);
      if (glyph != 0)
      {
        bitmap_draw(cx, y, 16, 16, glyph, 0U);
      }
      p += 3;
      cx = (int16_t)(cx + TEXT16_STEP);
    }
    else
    {
      p++;
    }
  }
}

uint16_t oled_gfx_text16_width(const char *utf8)
{
  const uint8_t *p = (const uint8_t *)utf8;
  uint16_t width = 0U;
  uint16_t last_step = 0U;

  while (*p != 0U)
  {
    if (*p < 0x80U)
    {
      p++;
      width = (uint16_t)(width + 6U);
      last_step = 6U;
    }
    else if ((*p & 0xF0U) == 0xE0U && p[1] != 0U && p[2] != 0U)
    {
      p += 3;
      width = (uint16_t)(width + TEXT16_STEP);
      last_step = TEXT16_STEP;
    }
    else
    {
      p++;
    }
  }
  /* 末尾字距（18 - 16）不计入宽度，便于居中 */
  if (last_step == TEXT16_STEP && width >= (uint16_t)(TEXT16_STEP - 16U))
  {
    width = (uint16_t)(width - (uint16_t)(TEXT16_STEP - 16U));
  }
  return width;
}

const uint8_t *oled_gfx_buffer(void)
{
  return &s_fb[0][0];
}
