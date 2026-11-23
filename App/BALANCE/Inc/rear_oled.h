#ifndef C50C_REAR_OLED_H
#define C50C_REAR_OLED_H

#include <stdint.h>

/* Five fixed-width ASCII characters, including sign and decimal point. */
static inline void rear_oled_tenths(char out[6], int value, int max)
{
    unsigned int mag;
    if (value > max || value < -max) {
        out[0] = value < 0 ? '<' : '>';
        out[1] = '9'; out[2] = '9'; out[3] = '.'; out[4] = '9'; out[5] = 0;
        return;
    }
    out[0] = value < 0 ? '-' : '+';
    mag = (unsigned int)(value < 0 ? -value : value);
    out[1] = (char)('0' + mag / 100u);
    out[2] = (char)('0' + (mag / 10u) % 10u);
    out[3] = '.';
    out[4] = (char)('0' + mag % 10u);
    out[5] = 0;
}

static inline void rear_oled_pwm(char out[6], int command)
{
    int tenths;
    if (command >= 16800 || command <= -16800) {
        rear_oled_tenths(out, command < 0 ? -1000 : 1000, 999);
        return;
    }
    tenths = command * 1000 / 16800;
    rear_oled_tenths(out, tenths, 999);
}

/* mm/s is numerically equal to tenths of cm/s. */
static inline void rear_oled_speed(char out[6], int mm_s, int valid)
{
    if (!valid) {
        out[0] = ' '; out[1] = '-'; out[2] = '-'; out[3] = '.';
        out[4] = '-'; out[5] = 0;
        return;
    }
    rear_oled_tenths(out, mm_s, 999);
}

static inline int rear_oled_fresh(int valid, uint32_t now_ms, uint32_t sample_ms)
{
    return valid && (uint32_t)(now_ms - sample_ms) <= 250u;
}

#endif
