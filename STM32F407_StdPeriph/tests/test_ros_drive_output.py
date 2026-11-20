#!/usr/bin/env python3
"""Offline test of the ROS straight line path.

The test lifts the real ResponseControl body, the real ros_speed_loop /
ros_measured_forward_mps / ros_drive_duty, and the real Set_Pwm prologue out
of BALANCE/balance_task.c, compiles them together with BALANCE/steer_debug.c
under ROS_STRAIGHT_DRIVE, and drives the four wheel outputs through a stub.

The stub Set_Pwm records (a, b, c, d) AFTER the m_b/m_c inversion prologue,
i.e. the signed values that reach the H-bridge direction pins. The expected
tuples follow the 2026-11-15 16:30 measured channel map (user confirmed):

    chA = 左后 (positive = forward)   chB = 右后 (negative = forward)
    chC = 左前 (positive = forward)   chD = 右前 (negative = forward)

So the behavioural signatures are:
    forward   -> (+v, -v, +v, -v)   all four wheels forward
    backward  -> (-v, +v, -v, +v)   all four wheels backward
    spin left -> (−T, −T, −T, −T)   左后/左前 backward, 右后/右前 forward
    spin right-> (+T, +T, +T, +T)   left side forward, right side backward

The 2026-11-15 regrouping fix (Set_Pwm(drive.left, drive.right, -drive.left,
-drive.right, 0)) is what makes the two spin signatures uniform; the old call
grouped the rear axle diagonally and the four wheels fought each other.

The stub records pre PWM_MIN_CMD/PWM_MAX_CMD values (that lift happens inside
the compiled-out AKM branch), so duties here are raw signed counts.

Covered:
  1. the wheels stay at zero until the steer machine has armed;
  2. a fresh ROS speed command drives all four wheels forward;
  3. the speed command alone never exceeds the per wheel duty limit;
  4. a stale command (timeout) zeros the wheels and disarms;
  5. stop / zero speed frames hold zero while staying armed;
  6. left/right spin commands produce the uniform channel signatures above
     (regression guard for the diagonal-grouping turning bug);
  7. forward arcs keep the turn on the correct side (Vz>0 -> right);
  8. the PS2 control mode keeps its own entry, arm window and limit, and the
     ROS path never reads the pad.
"""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
text = (root / 'BALANCE/balance_task.c').read_bytes().decode('gb18030')

base = text.index('static void ResponseControl(void)\n{')
start = text.index('#if defined(REAR_WHEEL_DEBUG)', base)
end = text.index('#ifdef WHEEL_DIR_CALIB', start)
response = text[start:end]

base = text.index('static void Set_Pwm(int m_a,int m_b,int m_c,int m_d,int servo)\n{')
start = text.index('{', base) + 1
end = text.index('\n\t#if defined AKM_CAR', start)
guard = text[start:end]

start = text.index('static int ros_drive_duty(float speed_mps)\n{')
end = text.index('\n}\n', start) + 3
helper = text[start:end]
assert 'ros_drive_duty' in helper and helper.count('{') == helper.count('}')

# ros_measured_forward_mps + ros_speed_loop sit directly above ResponseControl
speed_loop = text[text.index('static float ros_measured_forward_mps(void)'):
                  text.index('static void ResponseControl(void)')].rstrip()
assert 'g_ros_speed_output' in speed_loop

# sentinel that closes response() body and declares the pad reference
TAIL_MARK = '}\n'

head = '''
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#define REAR_WHEEL_DEBUG 1
#define _4WD_CAR 1
#define FRONT_TEST_CHANNEL 6
#define FOUR_TURN_DEBUG 1
#define ROS_STRAIGHT_DRIVE 1
#define ROS_DRIVE_MAX_DUTY 1680
#define portTICK_PERIOD_MS 1u
#define pdMS_TO_TICKS(x) (x)
#define CONTROL_DELAY 1000
#define ROS_CMD_TIMEOUT_MS 300u
#define V1_0 0
#define _PS2_Control 2
#define _ROS_Control 1
#define ROS_SPEED_TRIM_MAX 800
#define ROS_SPEED_TRIM_K 0.5f
static uint32_t ticks;
static uint32_t xTaskGetTickCount(void){return ticks;}
#include "steer_debug.h"
static steer_debug_t g_steer_debug;
static steer_debug_t g_ros_steer_debug;
static int g_rear_ps2_valid;
static uint32_t g_rear_ps2_tick __attribute__((unused));
typedef struct {float Bias; float LastBias; int Output; int kp; int ki;} PI_CONTROLLER;
static PI_CONTROLLER g_ros_speed_pi;
static int g_ros_speed_output;
static int g_ros_speed_diag_ready;
static int g_ros_speed_diag_neutral;
volatile uint32_t g_safety_probe[48];
typedef struct {int Output; float Target; float Encoder;} Motor;
static struct {Motor MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;} robot;
static struct {int HardWare_Ver,Time_count;} SysVal;
static struct {int AllowRecharge;} charger;
static struct {int DeepCheck;} robot_check;
static struct {float Vx,Vz; uint32_t ros_cmd_tick;} robot_control;
static struct {uint8_t LY,RX;} pad;
static struct {uint8_t LY,RX;} *GamePadInterface;
static int control_mode;
#define Get_Control_Mode(mask) (control_mode & (mask))
static int a,b,c,d;
static void Set_Pwm(int m_a,int m_b,int m_c,int m_d,int servo){
'''

helper_guard = '''
#ifdef ROS_STRAIGHT_DRIVE
'''
helper_end = '\n#endif\n'

pwm_mid = '''
a=m_a;b=m_b;c=m_c;d=m_d;(void)servo;(void)a;(void)b;(void)c;(void)d;
'''

tail = '''
static void send_command(float vx, float vz, uint32_t now){
  control_mode = _ROS_Control;
  robot_control.Vx = vx;
  robot_control.Vz = vz;
  robot_control.ros_cmd_tick = now;
  /* Perfect encoder tracking: the speed loop trim (0.5 * error_mm_s, capped
     at +-800) stays 0, so g_ros_speed_output == ros_drive_duty(vx) exactly
     and the expectations below only exercise duty mapping and steering. */
  robot.MOTOR_A.Encoder = vx;
  robot.MOTOR_B.Encoder = vx;
  ticks = now;
  response();
}
int main(void){
  GamePadInterface=(void *)&pad; SysVal.HardWare_Ver=0; SysVal.Time_count=8930; control_mode=_ROS_Control;
  g_rear_ps2_valid=1; pad.LY=127; pad.RX=127;

  /* 1: nothing moves before the steer machine arms (100 ms of neutral). */
  ticks=1; send_command(0.0f, 0.0f, 1);
  assert(a==0 && b==0 && c==0 && d==0);
  send_command(0.0f, 0.0f, 50);
  assert(a==0 && b==0 && c==0 && d==0);
  send_command(0.0f, 0.0f, 150);   /* 149 ms neutral: arms here */
  assert(a==0 && b==0 && c==0 && d==0);

  /* 2: a fresh forward command drives all four wheels FORWARD:
     the alternating tuple (+,-,+,-) under the measured channel map. */
  send_command(0.5f, 0.0f, 160);
  assert(a==1680 && b==-1680 && c==1680 && d==-1680);

  /* 3: the duty is limited per wheel: 9 m/s clamps to the same 1680. */
  send_command(9.0f, 0.0f, 170);
  assert(a==1680 && b==-1680 && c==1680 && d==-1680 && g_ros_steer_debug.left.running);

  /* 4: reverse inserts one stop step, then all four wheels BACKWARD. */
  send_command(-9.0f, 0.0f, 180);
  assert(a==0 && b==0 && c==0 && d==0);
  send_command(-9.0f, 0.0f, 210);
  assert(a==-1680 && b==1680 && c==-1680 && d==1680);

  /* 5: a zero command holds zero output while staying armed. */
  send_command(0.0f, 0.0f, 240);
  assert(a==0 && b==0 && c==0 && d==0);
  assert(g_ros_steer_debug.armed && !g_ros_steer_debug.left.running &&
         !g_ros_steer_debug.right.running);

  /* 6: LEFT spin (Vz<0): all four channels uniform negative —
     左后/左前 backward, 右后/右前 forward. Regression guard for the
     2026-11-15 diagonal-grouping bug (old firmware scrubbed diagonally).
     Both sides have been stopped since case 4 (>20 ms), so the direction
     flip takes effect on the first step. */
  send_command(0.0f, -0.05f, 260);
  assert(a==-600 && b==-600 && c==-600 && d==-600);

  /* 7: RIGHT spin (Vz>0): the left side is running, so its flip eats one
     20 ms stop step first; the right side keeps its direction. */
  send_command(0.0f, 0.05f, 300);
  assert(a==0 && b==0 && c==0 && d==0);            /* both sides flip: stop step */
  send_command(0.0f, 0.05f, 330);
  assert(a==600 && b==600 && c==600 && d==600);

  /* 8: forward arc RIGHT (Vz>0): left side forward, right side stopped. */
  send_command(0.05f, 0.05f, 350);
  assert(a==1200 && b==0 && c==1200 && d==0);

  /* 9: forward arc LEFT (Vz<0): right side forward, left side stopped. */
  send_command(0.05f, -0.05f, 380);
  assert(a==0 && b==-1200 && c==0 && d==-1200);

  /* 10: saturated mixed command rescales both sides proportionally. */
  send_command(0.4f, 0.8f, 420);
  assert(a==1680 && b==0 && c==1680 && d==0);

  /* 11: a stale command zeros the wheels and disarms. */
  ticks = 420 + 301;
  response();
  assert(a==0 && b==0 && c==0 && d==0);
  assert(!g_ros_steer_debug.armed);

  /* 12: re-arm on a held neutral, then forward again. */
  send_command(0.0f, 0.0f, 750);
  assert(a==0 && b==0 && c==0 && d==0);
  send_command(0.5f, 0.0f, 770);   /* non-neutral resets the wait */
  assert(a==0 && b==0 && c==0 && d==0);
  send_command(0.0f, 0.0f, 790);
  send_command(0.0f, 0.0f, 900);   /* 110 ms neutral: armed */
  send_command(0.5f, 0.0f, 910);
  assert(a==1680 && b==-1680 && c==1680 && d==-1680);

  /* 13: a mode switch to PS2 zeros the wheels and clears both machines. */
  control_mode = _PS2_Control; ticks=920; response();
  assert(a==0 && b==0 && c==0 && d==0);

  /* 14: the charger request and the deep self check gate the ROS path. */
  control_mode = _ROS_Control;
  send_command(0.0f, 0.0f, 930);
  send_command(0.0f, 0.0f, 1040);  /* re-armed */
  charger.AllowRecharge=1; send_command(0.5f, 0.0f, 1050);
  assert(a==0 && b==0 && c==0 && d==0);
  charger.AllowRecharge=0; robot_check.DeepCheck=1; send_command(0.5f, 0.0f, 1060);
  assert(a==0 && b==0 && c==0 && d==0);
  robot_check.DeepCheck=0;

  /* 15: the pad stick never reaches the ROS path: pushed stick + ROS
     reverse still gives exactly the reverse tuple. */
  pad.LY=255; pad.RX=255;
  send_command(0.0f, 0.0f, 1070);
  send_command(0.0f, 0.0f, 1180);  /* re-armed on neutral */
  send_command(-0.3f, 0.0f, 1190);
  assert(a==-1680 && b==1680 && c==-1680 && d==1680);
  pad.LY=127; pad.RX=127;

  /* 16: steer_debug_step_duty unit behaviour at the duty limit. */
  {
    steer_debug_t s = {0};
    steer_output_t out;
    (void)steer_debug_step(&s, 127, 127, 1, 1000);
    (void)steer_debug_step(&s, 127, 127, 1, 1200);
    out = steer_debug_step(&s, 255, 127, 1, 1201);
    assert(out.left == 13440 && out.right == 13440);
    out = steer_debug_step_duty(&s, 1680, 0, 1, 1, 1202);
    assert(out.left == 1680 && out.right == 1680);
    out = steer_debug_step_duty(&s, 99999, 0, 1, 1, 1203);
    assert(out.left == 1680 && out.right == 1680);
    out = steer_debug_step_duty(&s, 0, 0, 1, 0, 1204);
    assert(out.left == 0 && out.right == 0);
  }

  /* 17: the PS2 entry: full push drives all four forward (its own 13440
     limit), full spin right drives 左后/左前 forward and 右后/右前 backward. */
  control_mode = _PS2_Control; pad.LY=127; pad.RX=127;
  ticks=2000; g_rear_ps2_tick=ticks; response();
  ticks=2110; g_rear_ps2_tick=ticks; response();   /* neutral arm window */
  pad.LY=255; ticks=2120; g_rear_ps2_tick=ticks; response();
  assert(a==13440 && b==-13440 && c==13440 && d==-13440);
  pad.LY=127; pad.RX=255; ticks=2130; g_rear_ps2_tick=ticks; response();
  assert(a==13440 && b==0 && c==13440 && d==0);    /* right side eats a stop step */
  pad.LY=127; pad.RX=255; ticks=2160; g_rear_ps2_tick=ticks; response();
  assert(a==13440 && b==13440 && c==13440 && d==13440);
  pad.LY=127; pad.RX=127;

  /* 18: back to ROS: the cleared machine waits for its neutral window. */
  send_command(0.5f, 0.0f, 2200);
  assert(a==0 && b==0 && c==0 && d==0);
  send_command(0.0f, 0.0f, 2210);
  send_command(0.0f, 0.0f, 2320);  /* armed */
  send_command(0.5f, 0.0f, 2330);
  assert(a==1680 && b==-1680 && c==1680 && d==-1680);
  return 0;
}
'''

with tempfile.TemporaryDirectory() as tmp:
    p = pathlib.Path(tmp)
    # ros_drive_duty must sit above ros_speed_loop, which calls it;
    # both must sit above response().
    body = (head + guard + pwm_mid + '}\n' + helper_guard + helper + helper_end +
            '\n' + speed_loop + '\n'
            'static void response(void) {\n' + response.rstrip() + '\n' +
            TAIL_MARK + tail)
    (p / 't.c').write_text(body)
    subprocess.run(['cc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                    '-I', str(root / 'BALANCE/Inc'),
                    '-DROS_STRAIGHT_DRIVE=1', '-DROS_DRIVE_MAX_DUTY=1680',
                    str(p / 't.c'), str(root / 'BALANCE/steer_debug.c'),
                    '-o', str(p / 't')], check=True)
    subprocess.run([str(p / 't')], check=True)
print('ros straight drive output: PASS')
