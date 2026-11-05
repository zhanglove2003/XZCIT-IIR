#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "rear_oled.h"

int main(void)
{
    char b[6];
    rear_oled_pwm(b, 0);     assert(strcmp(b, "+00.0") == 0);
    rear_oled_pwm(b, 1500);  assert(strcmp(b, "+08.9") == 0);
    rear_oled_pwm(b, -1500); assert(strcmp(b, "-08.9") == 0);
    rear_oled_pwm(b, 16800); assert(strcmp(b, ">99.9") == 0);
    rear_oled_pwm(b, INT32_MIN); assert(strcmp(b, "<99.9") == 0);

    rear_oled_speed(b, 98, 1);   assert(strcmp(b, "+09.8") == 0);
    rear_oled_speed(b, -102, 1); assert(strcmp(b, "-10.2") == 0);
    rear_oled_speed(b, 0, 0);    assert(strcmp(b, " --.-") == 0);
    rear_oled_speed(b, 1000, 1); assert(strcmp(b, ">99.9") == 0);
    rear_oled_speed(b, -1000, 1); assert(strcmp(b, "<99.9") == 0);
    assert(rear_oled_fresh(1, 1000, 900));
    assert(!rear_oled_fresh(0, 1000, 900));
    assert(!rear_oled_fresh(1, 1151, 900));
    assert(rear_oled_fresh(1, 10, UINT32_MAX - 10));
    return 0;
}
