#!/usr/bin/env python3
"""Exercise the real OLED page with distinct left/right data and stale input."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
text = (root / 'BALANCE/show_task.c').read_bytes().decode('gb18030')
start = text.index('static void oled_4wd_debug_show(void)\n{')
end = text.index('\n}\n#endif\n//不同车型对应的显示页面', start) + 2
function = text[start:end]
pre = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "rear_odom.h"
#include "rear_oled.h"
#define REAR_WHEEL_DEBUG 1
#define ENCODER_DIAG 1
#define portTICK_PERIOD_MS 1u
typedef unsigned char u8;
typedef struct {int Output;} Motor;
static struct {Motor MOTOR_A, MOTOR_B;} robot;
volatile rear_odom_snapshot_t g_rear_odom_snapshot;
static uint32_t tick;
static uint32_t xTaskGetTickCount(void) { return tick; }
static int lock_depth;
#define taskENTER_CRITICAL() (++lock_depth)
#define taskEXIT_CRITICAL() (--lock_depth)
static char at[4][16][16];
static void OLED_ShowString(u8 x, u8 y, const u8 *s) {
    assert(lock_depth == 0 && (x % 8) == 0);
    assert(y == 12 || y == 24 || y == 36);
    assert(x + 8 * strlen((const char*)s) <= 128);
    unsigned row = (y - 12) / 12;
    assert(strlen((const char*)s) < sizeof(at[row][x / 8]));
    strcpy(at[row][x / 8], (const char*)s);
}
'''
tail = r'''
int main(void) {
    robot.MOTOR_A.Output = 1500;
    robot.MOTOR_B.Output = -750;
    g_rear_odom_snapshot.left_mm_s = -100;
    g_rear_odom_snapshot.right_mm_s = 200;
    g_rear_odom_snapshot.sample_ms = 900;
    g_rear_odom_snapshot.valid = 1;
    tick = 1000;
    oled_4wd_debug_show();
    assert(!strcmp(at[0][3], "PWM%") && !strcmp(at[0][9], "cm/s"));
    assert(!strcmp(at[1][0], "RL") && !strcmp(at[2][0], "RR"));
    assert(!strcmp(at[1][3], "-04.4") && !strcmp(at[2][3], "+08.9"));
    assert(!strcmp(at[1][9], "-10.0") && !strcmp(at[2][9], "+20.0"));
    tick = 1151;
    oled_4wd_debug_show();
    assert(!strcmp(at[1][9], " --.-") && !strcmp(at[2][9], " --.-"));
    return 0;
}
'''
with tempfile.TemporaryDirectory() as tmp:
    p = pathlib.Path(tmp)
    (p / 'test.c').write_text(pre + function + '\n' + tail)
    subprocess.run(['cc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    '-I', str(root / 'BALANCE/Inc'), str(p / 'test.c'),
                    '-o', str(p / 'test')], check=True)
    subprocess.run([str(p / 'test')], check=True)
print('rear OLED mapping, layout and stale-data handling: PASS')
