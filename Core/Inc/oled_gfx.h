#ifndef OLED_GFX_H
#define OLED_GFX_H

#include <stdint.h>

#define OLED_WIDTH   128U
#define OLED_HEIGHT   64U

void oled_gfx_clear(void);
void oled_gfx_pixel(int16_t x, int16_t y, uint8_t on);
void oled_gfx_bitmap(int16_t x, int16_t y, uint8_t w, uint8_t h, const uint8_t *data);
void oled_gfx_rect(int16_t x, int16_t y, int16_t w, int16_t h);
void oled_gfx_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h);
void oled_gfx_text6x12(int16_t x, int16_t y, const char *ascii);
void oled_gfx_text6x12_inv(int16_t x, int16_t y, const char *ascii);
void oled_gfx_text16(int16_t x, int16_t y, const char *utf8);
void oled_gfx_text16_inv(int16_t x, int16_t y, const char *utf8);
uint16_t oled_gfx_text6x12_width(const char *ascii);
uint16_t oled_gfx_text16_width(const char *utf8);
const uint8_t *oled_gfx_buffer(void);

#endif
