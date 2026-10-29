#ifndef OLED_H
#define OLED_H

#include <stdint.h>

/* 上板若画面左右/上下颠倒，只改这两个宏 */
#define OLED_MIRROR_X 1
#define OLED_MIRROR_Y 1

void oled_init(void);
void oled_flush(void);

#endif
