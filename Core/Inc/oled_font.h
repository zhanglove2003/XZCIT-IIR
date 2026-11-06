/* 自动生成：tools/font_convert.py —— 请勿手改 */
#ifndef OLED_FONT_H
#define OLED_FONT_H

#include <stdint.h>

#define OLED_FONT_CN_COUNT 96
#define OLED_FONT_TITLE_COUNT 9

/* 6×12 ASCII，page 格式（2 页 × 6 列），bit0 为页内顶行 */
extern const uint8_t oled_font_ascii[95][12];

/* 10×16 标题 ASCII，page 格式（2 页 × 10 列），由 6×12 最近邻放大 */
extern const uint8_t oled_font_title[OLED_FONT_TITLE_COUNT][20];

/* 16×16 汉字，page 格式（2 页 × 16 列），已旋转为正立 */
extern const uint8_t oled_font_cn[OLED_FONT_CN_COUNT][32];

/* 12×12 转圈动画，8 帧，page 格式 */
extern const uint8_t oled_font_spinner[8][24];

extern const uint8_t oled_font_check[32];
extern const uint8_t oled_font_cross[32];

const uint8_t *oled_font_cn_lookup(const char *utf8);
const uint8_t *oled_font_title_lookup(char c);

#endif
