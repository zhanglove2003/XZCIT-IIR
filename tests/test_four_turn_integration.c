
/* Host-side integration test: exercises the real REAR_WHEEL_DEBUG branch of
   ResponseControl() and the real Set_Pwm() guard extracted verbatim from
   App/BALANCE/balance_task.c (see SNIPPET/GUARD markers below). Path anchors
   point at the current tree: App/BALANCE + App/BALANCE/Inc. */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include "steer_debug.h"
#define REAR_WHEEL_DEBUG 1
#define _4WD_CAR 1
#ifndef FRONT_TEST_CHANNEL
#define FRONT_TEST_CHANNEL 6
#endif
#define FOUR_TURN_DEBUG 1
#define portTICK_PERIOD_MS 1u
#define pdMS_TO_TICKS(x) (x)
#define CONTROL_DELAY 1000
#define V1_0 0
#define V1_1 1
#define _PS2_Control 2
static uint32_t ticks;
static uint32_t xTaskGetTickCount(void){return ticks;}
static steer_debug_t g_steer_debug;
static int g_rear_ps2_valid;
static uint32_t g_rear_ps2_tick;
typedef struct {int Output; float Target;} Motor;
static struct {Motor MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;} robot;
static struct {int HardWare_Ver,Time_count;} SysVal;
static struct {int AllowRecharge;} charger;
static struct {int DeepCheck;} robot_check;
static struct {uint8_t LY,RX;} Classic_PS2Gamepad;
static struct {uint8_t LY,RX;} *GamePadInterface;
static int control_mode;
#define Get_Control_Mode(mask) (control_mode & (mask))
int AIN1,AIN2,BIN1,BIN2,CIN1,CIN2,DIN1,DIN2,V1_0_BIN1,V1_0_BIN2,PWMA,PWMB,PWMC,PWMD;
#define PWM_MIN_CMD 3360
#define PWM_MAX_CMD 8400
static int a,b,c,d;
static void Set_Pwm(int m_a,int m_b,int m_c,int m_d,int servo)
{
(void)servo;

#if defined(REAR_WHEEL_DEBUG) && defined(_4WD_CAR)
#if defined(FRONT_TEST_CHANNEL)
#if FRONT_TEST_CHANNEL != 6
    m_a = 0;
    m_b = 0;
#endif
#if FRONT_TEST_CHANNEL == 3
    m_d = 0;
#elif FRONT_TEST_CHANNEL == 4
    m_c = 0;
#endif
#else
    m_c = 0;
    m_d = 0;
#endif
#endif

		
		//电机控制
		if( m_a > 0 ) AIN1=0,AIN2=1;
#if defined(REAR_WHEEL_DEBUG) && defined(_4WD_CAR)
		else if (m_a < 0) AIN1=1,AIN2=0;
#else
		else          AIN1=1,AIN2=0;
#endif
		
		if(SysVal.HardWare_Ver==V1_0)
		{
			if( m_b > 0 ) V1_0_BIN1=0,V1_0_BIN2=1;
#if defined(REAR_WHEEL_DEBUG) && defined(_4WD_CAR)
			else if (m_b < 0) V1_0_BIN1=1,V1_0_BIN2=0;
#else
			else          V1_0_BIN1=1,V1_0_BIN2=0;
#endif
		}
		else if( SysVal.HardWare_Ver>=V1_1 )
		{
			if( m_b > 0 ) BIN1=0,BIN2=1;
			else          BIN1=1,BIN2=0;
		}
	
		//四驱方向修正: 原代码 C/D 的正向电平与 A/B 相反(10 vs 01),
		//导致同样符号的 PWM 让轮子转向相反. 这里统一为 m>0 -> 01.
		if( m_c > 0 ) CIN1=0,CIN2=1;
#if defined(FRONT_TEST_CHANNEL)
		else if (m_c < 0) CIN1=1,CIN2=0;
#else
		else          CIN1=1,CIN2=0;
#endif
		
		if( m_d > 0 ) DIN1=0,DIN2=1;
#if defined(FRONT_TEST_CHANNEL)
		else if (m_d < 0) DIN1=1,DIN2=0;
#else
		else          DIN1=1,DIN2=0;
#endif
		
		//[C50C] 起跳值与上限: 低速摩擦带不动, 非零指令抬到 3360(20%); 超过 8400(50%) 截断.
		{
			/* [C50C] m_a..m_d 为 int, abs() 返回 int; 原写 v 为 int 时 8000 应原样通过,
			   但实测两档都停在 3360, 故改用 long 并显式打印比较结果. */
			long v;
			v = abs(m_a); if( v != 0 ) v = ( v < PWM_MIN_CMD ) ? PWM_MIN_CMD : v; if( v > PWM_MAX_CMD ) v = PWM_MAX_CMD; PWMA = v;
			v = abs(m_b); if( v != 0 ) v = ( v < PWM_MIN_CMD ) ? PWM_MIN_CMD : v; if( v > PWM_MAX_CMD ) v = PWM_MAX_CMD; PWMB = v;
			v = abs(m_c); if( v != 0 ) v = ( v < PWM_MIN_CMD ) ? PWM_MIN_CMD : v; if( v > PWM_MAX_CMD ) v = PWM_MAX_CMD; PWMC = v;
			v = abs(m_d); if( v != 0 ) v = ( v < PWM_MIN_CMD ) ? PWM_MIN_CMD : v; if( v > PWM_MAX_CMD ) v = PWM_MAX_CMD; PWMD = v;
		}
		
a=m_a;b=m_b;c=m_c;d=m_d;
}
static void response(void)
{

#if defined(REAR_WHEEL_DEBUG)
    {
        uint32_t tick = (uint32_t)xTaskGetTickCount();
        uint32_t ms = tick * portTICK_PERIOD_MS;
#if defined(ROS_STRAIGHT_DRIVE)
        static uint8_t last_drive_mode;
        uint8_t drive_mode = Get_Control_Mode(_ROS_Control) ? _ROS_Control :
                             (Get_Control_Mode(_PS2_Control) ? _PS2_Control : 0);
        if (drive_mode != last_drive_mode) {
            /* Mode switch: clear both machines. A cleared machine is unarmed
               and holds zero output until its own input stays neutral for
               ARM_NEUTRAL_MS, so the entered source restarts from the neutral
               wait instead of resuming the state of the previous source. */
            static const steer_debug_t cleared;
            g_ros_steer_debug = cleared;
            g_steer_debug = cleared;
            last_drive_mode = drive_mode;
        }
        if (drive_mode == _ROS_Control) {
        /* The four wheel output comes from robot_control.Vx/Vz, never from the pad.
           When ros_ready is false the duties are forced to zero, so the last accepted
           speed cannot survive a mode switch or a stop request. */
        int ros_ready = SysVal.HardWare_Ver == V1_0 &&
                        Get_Control_Mode(_ROS_Control) &&
                        (uint32_t)(tick - robot_control.ros_cmd_tick) <= pdMS_TO_TICKS(ROS_CMD_TIMEOUT_MS) &&
                        !robot_check.DeepCheck && SysVal.Time_count > CONTROL_DELAY + 230;
        /* ROS closed loop: speed PI on encoder feedback, then split to wheels.
           Feedback is m/s, PI output is duty counts; start-up floor and ceiling
           are still applied by Set_Pwm. */
        ros_speed_loop(ros_ready);
        /* 中性判据用指令是否为 0，不用占空。闭环占空在运动时永不为 0，
           用它判中性会让 armed 一次失效后永久锁死。 */
        int ros_cmd_neutral = (robot_control.Vx == 0.0f) && (robot_control.Vz == 0.0f);
        g_ros_speed_diag_ready = ros_ready;
        g_ros_speed_diag_neutral = ros_cmd_neutral;
        steer_output_t drive = steer_debug_step_duty(&g_ros_steer_debug,
                                                    g_ros_speed_output,
                                                    ros_ready ? ros_drive_duty(robot_control.Vz) : 0,
                                                    ros_cmd_neutral,
                                                    ros_ready, ms);
        robot.MOTOR_A.Output = drive.right;
        robot.MOTOR_B.Output = -drive.left;
        robot.MOTOR_C.Output = drive.left;
        robot.MOTOR_D.Output = -drive.right;
        robot.MOTOR_A.Target = (float)drive.right / 3000.0f;
        robot.MOTOR_B.Target = (float)drive.left / 3000.0f;
        robot.MOTOR_C.Target = robot.MOTOR_D.Target = 0;
        /* [C50C 2026-11-15] Driver board reinstall swapped the rear wheel
           binding: measured per-channel with pwm_test frames (dir=0):
           chA=left-rear(reverse) chB=right-rear(reverse) chC=left-front(fwd)
           chD=right-front(fwd). Pre-reinstall calibration comment at the PID
           path said A=right-rear B=left-rear C=left-front D=right-front, i.e.
           the two rear motors moved between chA and chB. Negate A and B here
           (both the ROS branch and the PS2 FOUR_TURN_DEBUG branch). */
        Set_Pwm(drive.right, -drive.left, drive.left, -drive.right, 0);
        } else
#endif
#if defined(FOUR_TURN_DEBUG)
        {
        int ready = SysVal.HardWare_Ver == V1_0 &&
                    Get_Control_Mode(_PS2_Control) &&
                    !robot_check.DeepCheck && SysVal.Time_count > CONTROL_DELAY + 230 &&
                    g_rear_ps2_valid && (uint32_t)(tick - g_rear_ps2_tick) <= pdMS_TO_TICKS(150);
        uint8_t forward_axis = ready ? Classic_PS2Gamepad.LY : 127;
        uint8_t turn_axis = ready ? Classic_PS2Gamepad.RX : 127;
        steer_output_t drive = steer_debug_step(&g_steer_debug, forward_axis, turn_axis, ready, ms);
        robot.MOTOR_A.Output = drive.right;
        robot.MOTOR_B.Output = -drive.left;
        robot.MOTOR_C.Output = drive.left;
        robot.MOTOR_D.Output = -drive.right;
        robot.MOTOR_A.Target = (float)drive.right / 3000.0f;
        robot.MOTOR_B.Target = (float)drive.left / 3000.0f;
        robot.MOTOR_C.Target = robot.MOTOR_D.Target = 0;
        /* [C50C 2026-11-15] Driver board reinstall swapped the rear wheel
           binding: measured per-channel with pwm_test frames (dir=0):
           chA=left-rear(reverse) chB=right-rear(reverse) chC=left-front(fwd)
           chD=right-front(fwd). Pre-reinstall calibration comment at the PID
           path said A=right-rear B=left-rear C=left-front D=right-front, i.e.
           the two rear motors moved between chA and chB. Negate A and B here
           (both the ROS branch and the PS2 FOUR_TURN_DEBUG branch). */
        Set_Pwm(drive.right, -drive.left, drive.left, -drive.right, 0);
        }
#else
        uint8_t axis = ready ? Classic_PS2Gamepad.LY : 127;
        int command = rear_debug_step(&g_rear_debug, axis, ready, ms);
#if defined(FRONT_TEST_CHANNEL)
        robot.MOTOR_A.Output = robot.MOTOR_B.Output = FRONT_TEST_CHANNEL == 6 ? command : 0;
        robot.MOTOR_A.Target = robot.MOTOR_B.Target = FRONT_TEST_CHANNEL == 6 ? (float)command / 3000.0f : 0;
        robot.MOTOR_C.Output = (FRONT_TEST_CHANNEL == 3 || FRONT_TEST_CHANNEL == 5 || FRONT_TEST_CHANNEL == 6) ? command : 0;
        robot.MOTOR_D.Output = (FRONT_TEST_CHANNEL == 4 || FRONT_TEST_CHANNEL == 5 || FRONT_TEST_CHANNEL == 6) ? -command : 0;
        robot.MOTOR_C.Target = robot.MOTOR_D.Target = 0;
        Set_Pwm(-robot.MOTOR_A.Output, robot.MOTOR_B.Output, robot.MOTOR_C.Output, robot.MOTOR_D.Output, 0);
#else
        robot.MOTOR_A.Output = command;
        robot.MOTOR_B.Output = command;
        robot.MOTOR_C.Output = 0;
        robot.MOTOR_D.Output = 0;
        robot.MOTOR_A.Target = robot.MOTOR_B.Target = (float)command / 3000.0f;
        robot.MOTOR_C.Target = robot.MOTOR_D.Target = 0;
        Set_Pwm(-command, command, 0, 0, 0);
#endif
#endif
    }
    return;
#endif
	

}

int main(void)
{
    (void)charger;(void)robot_check;(void)GamePadInterface;
    GamePadInterface=(void *)&Classic_PS2Gamepad; SysVal.Time_count=1231; control_mode=2;
    g_rear_ps2_valid=1; Classic_PS2Gamepad.LY=127; Classic_PS2Gamepad.RX=127; ticks=1; g_rear_ps2_tick=ticks; response();
    ticks=101; g_rear_ps2_tick=ticks; response();
    Classic_PS2Gamepad.RX=255; ticks=111; g_rear_ps2_tick=ticks; response();
    /* Current source: RX=255 pivots with left=+MAX, right=-MAX; Set_Pwm maps
       A=right, B=-left, C=left, D=-right. MAX_DUTY was raised to 13440. */
    assert(a==-13440 && b==-13440 && c==13440 && d==13440);
    assert(robot.MOTOR_A.Output==-13440 && robot.MOTOR_B.Output==-13440);
    Classic_PS2Gamepad.RX=127; ticks=121; g_rear_ps2_tick=ticks; response();
    assert(a==0 && b==0 && c==0 && d==0);
    Classic_PS2Gamepad.LY=255; ticks=151; g_rear_ps2_tick=ticks; response();
    assert(a==13440 && b==-13440 && c==13440 && d==-13440);
    ticks=302; response(); assert(a==0 && b==0 && c==0 && d==0);
    /* Re-arm after a stale frame, then exercise both rotation directions. */
    Classic_PS2Gamepad.LY=127; Classic_PS2Gamepad.RX=127; ticks=310; g_rear_ps2_tick=ticks; response();
    ticks=410; g_rear_ps2_tick=ticks; response();
    Classic_PS2Gamepad.RX=0; ticks=411; g_rear_ps2_tick=ticks; response();
    assert(a==13440 && b==13440 && c==-13440 && d==-13440);
    Classic_PS2Gamepad.RX=255; ticks=412; g_rear_ps2_tick=ticks; response();
    assert(a==0 && b==0 && c==0 && d==0);
    ticks=432; g_rear_ps2_tick=ticks; response();
    assert(a==-13440 && b==-13440 && c==13440 && d==13440);
    Classic_PS2Gamepad.RX=127; ticks=433; g_rear_ps2_tick=ticks; response();
    assert(a==0 && b==0 && c==0 && d==0);
    Classic_PS2Gamepad.LY=255; Classic_PS2Gamepad.RX=255; ticks=453; g_rear_ps2_tick=ticks; response();
    assert(a==0 && b==-13440 && c==13440 && d==0);
    Classic_PS2Gamepad.LY=0; Classic_PS2Gamepad.RX=0; ticks=454; g_rear_ps2_tick=ticks; response();
    assert(a==0 && b==0 && c==0 && d==0);
    ticks=474; g_rear_ps2_tick=ticks; response();
    assert(a==0 && b==13440 && c==-13440 && d==0);
    g_rear_ps2_valid=0; ticks=475; response();
    assert(a==0 && b==0 && c==0 && d==0);
    puts("four wheel turning integration: PASS");
    return 0;
}
