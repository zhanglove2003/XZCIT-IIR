/* Host-side integration test: drives the real oled_4wd_debug_show() extracted
   verbatim from App/BALANCE/show_task.c (REAR_WHEEL_DEBUG branch) with distinct
   left/right data and stale odomety input. Path anchors: App/BALANCE +
   App/BALANCE/Inc. */
#include <assert.h>
#include <stdio.h>
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
static void oled_4wd_debug_show(void)
{
#if defined(REAR_WHEEL_DEBUG) && defined(ENCODER_DIAG)
    char left_pwm[6], right_pwm[6], left_speed[6], right_speed[6];
    int command_left, command_right, speed_valid;
    uint32_t now_ms, sample_ms;
    int16_t speed_left, speed_right;
    uint8_t sample_valid;

    /* Snapshot the calibrated rear-wheel feedback and the software PWM command. */
    taskENTER_CRITICAL();
    now_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    command_left = robot.MOTOR_B.Output;
    command_right = robot.MOTOR_A.Output;
    speed_left = g_rear_odom_snapshot.left_mm_s;
    speed_right = g_rear_odom_snapshot.right_mm_s;
    sample_ms = g_rear_odom_snapshot.sample_ms;
    sample_valid = g_rear_odom_snapshot.valid;
    taskEXIT_CRITICAL();

    speed_valid = rear_oled_fresh(sample_valid, now_ms, sample_ms);
    rear_oled_pwm(left_pwm, command_left);
    rear_oled_pwm(right_pwm, command_right);
    rear_oled_speed(left_speed, speed_left, speed_valid);
    rear_oled_speed(right_speed, speed_right, speed_valid);
    OLED_ShowString(24, 12, (const u8*)"PWM%");
    OLED_ShowString(72, 12, (const u8*)"cm/s");
    OLED_ShowString(0, 24, (const u8*)"RL");
    OLED_ShowString(24, 24, (const u8*)left_pwm);
    OLED_ShowString(72, 24, (const u8*)left_speed);
    OLED_ShowString(0, 36, (const u8*)"RR");
    OLED_ShowString(24, 36, (const u8*)right_pwm);
    OLED_ShowString(72, 36, (const u8*)right_speed);
#else
	char b1[6], b2[6];
	int  t, e, i;

	//每轮一行: 车标 + 目标速度 + 实际速度(单位 cm/s, 取整)
	//车标按实车位置标注. 实车对应关系(用户实测):
	//  MOTOR_A = 右前 FR,  MOTOR_B = 右后 RR
	//  MOTOR_C = 左后 RL,  MOTOR_D = 左前 FL
	for( i = 0; i < 4; i++ )
	{
		uint8_t    row = (uint8_t)(10 + i*10);
		const u8*  lbl;

		if( 0 == i )      { lbl = (const u8*)"FL"; t = (int)(robot.MOTOR_D.Target*100); e = (int)(robot.MOTOR_D.Encoder*100); }
		else if( 1 == i ) { lbl = (const u8*)"FR"; t = (int)(robot.MOTOR_A.Target*100); e = (int)(robot.MOTOR_A.Encoder*100); }
		else if( 2 == i ) { lbl = (const u8*)"RL"; t = (int)(robot.MOTOR_C.Target*100); e = (int)(robot.MOTOR_C.Encoder*100); }
		else              { lbl = (const u8*)"RR"; t = (int)(robot.MOTOR_B.Target*100); e = (int)(robot.MOTOR_B.Encoder*100); }

		if( t >  999 ) t =  999;
		if( t < -999 ) t = -999;
		if( e >  999 ) e =  999;
		if( e < -999 ) e = -999;

		snprintf(b1, sizeof(b1), "%+4d", t);
		snprintf(b2, sizeof(b2), "%+4d", e);

		OLED_ShowString(0,  row, lbl);
		OLED_ShowString(16, row, (const u8*)b1);
		OLED_ShowString(56, row, (const u8*)b2);
	}
	//最后一行: 摇杆输入 / 报错标志
	{
		int ly = (int)Classic_PS2Gamepad.LY;
		int lx = (int)Classic_PS2Gamepad.LX;
		OLED_ShowString(0, 50, "L");
		snprintf(b1, sizeof(b1), "%3d", ly);
		OLED_ShowString(8, 50, (const u8*)b1);
		snprintf(b2, sizeof(b2), "%3d", lx);
		OLED_ShowString(40, 50, (const u8*)b2);
		if( 0 == robot_check.errorflag ) OLED_ShowString(72, 50, "OK");
		else                             OLED_ShowString(72, 50, "ER");
		snprintf(b1, sizeof(b1), "%d", (int)robot.type);
		OLED_ShowString(96, 50, (const u8*)b1);
	}
#endif
}
int main(void)
{
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
    puts("rear OLED mapping, layout and stale-data handling: PASS");
    return 0;
}
