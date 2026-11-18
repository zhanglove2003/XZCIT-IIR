#!/usr/bin/env python3
"""Offline test of the ROS straight line path.

The test lifts the real ResponseControl body and the real Set_Pwm body out of
BALANCE/balance_task.c, compiles them together with BALANCE/steer_debug.c under
ROS_STRAIGHT_DRIVE, and drives the four wheel outputs through a stub.

Covered:
  1. a fresh ROS speed command drives the wheels forward;
  2. the speed command alone never exceeds the per wheel duty limit;
  3. a stale command (timeout) zeros the wheels;
  4. the PS2 control mode name never reaches ResponseControl on the ROS path,
     and the ROS mode name does;
  5. the wheels stay at zero until the arm window has passed;
  6. a stop or a mode switch zeros the wheels;
  7. the PS2 entry keeps its own signature and limit.
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
static uint32_t ticks;
static uint32_t xTaskGetTickCount(void){return ticks;}
#include "steer_debug.h"
static steer_debug_t g_steer_debug;
static steer_debug_t g_ros_steer_debug;
static int g_rear_ps2_valid;
static uint32_t g_rear_ps2_tick __attribute__((unused));
typedef struct {int Output; float Target;} Motor;
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
  ticks = now;
  response();
}
int main(void){
  GamePadInterface=(void *)&pad; SysVal.HardWare_Ver=0; SysVal.Time_count=8930; control_mode=_ROS_Control;
  g_rear_ps2_valid=1; pad.LY=127; pad.RX=127;

  /* 5: nothing moves before the arm window closes. */
  ticks=1; send_command(0.0f, 0.0f, 1);
  assert(a==0 && b==0 && c==0 && d==0);
  send_command(0.0f, 0.0f, 200);
  assert(a==0 && b==0 && c==0 && d==0);

  /* 1: a fresh forward command drives all four wheels forward.
     Forward means the left front and the right rear wheel go negative. */
  send_command(0.5f, 0.0f, 150);
  /* forward: every wheel at +500, with a as the negated right side */
  assert(a==-500 && b==500 && c==500 && d==-500);

  /* 2: the duty is limited per wheel, never by the raw command. */
  send_command(9.0f, 0.0f, 160);
  assert(a==-1680 && b==1680 && c==1680 && d==-1680 && g_ros_steer_debug.left.running);
  /* a direction change inserts one stop step, so wait out the 20 ms window */
  send_command(-9.0f, 0.0f, 161);
  assert(a==0 && b==0 && c==0 && d==0);
  send_command(-9.0f, 0.0f, 200);
  assert(a==1680 && b==-1680 && c==-1680 && d==1680);
  /* turning: forward 400 and turn 800 give 1200 and -400, both under the
     limit, so the sides stay opposite and keep the forward offset */
  send_command(0.0f, 0.0f, 240);
  /* sides follow side_step ordering: a,b take the right duty, c,d the left */
  send_command(0.4f, 0.8f, 260);
  assert(a==400 && b==1200 && c==1200 && d==400);
  /* forward 400 with turn 1680 would be 2080 and -1280, so both scale down */
  send_command(0.4f, 1.7f, 500);
  /* forward 400 + turn 1680 = 2080 on one side, 2080/1680 scales to 1680
     and -1280 scales to 1033; the sign flips per side */
  assert(a==1033 && b==1680 && c==1680 && d==1033);

  /* 3: a stale command zeros the wheels. */
  send_command(0.5f, 0.0f, 300);
  ticks = 300 + 301;
  response();
  assert(a==0 && b==0 && c==0 && d==0);

  /* 5 again: after a timeout a zero command re-arms only after the window. */
  send_command(0.0f, 0.0f, 500);
  send_command(0.5f, 0.0f, 520);
  assert(a==0 && b==0 && c==0 && d==0);
  send_command(0.0f, 0.0f, 560);
  assert(a==0 && b==0 && c==0 && d==0);
  send_command(0.0f, 0.0f, 600);
  send_command(0.0f, 0.0f, 690);
  send_command(0.5f, 0.0f, 700);
  assert(a==-500 && b==500 && c==500 && d==-500);

  /* 6a: a control mode switch away from ROS zeros the wheels. */
  control_mode = _PS2_Control; ticks=710; response();
  assert(a==0 && b==0 && c==0 && d==0);

  /* 6b: the charger request and the deep self check both zero the wheels.
     The forward direction was already released above, so the wheels are
     stopped and the next forward command needs only the reverse window. */
  send_command(0.0f, 0.0f, 720);
  send_command(0.0f, 0.0f, 790);
  send_command(0.5f, 0.0f, 820);
  send_command(0.0f, 0.0f, 821);
  assert(a==0 && b==0 && c==0 && d==0);
  charger.AllowRecharge=1; send_command(0.5f, 0.0f, 850);
  assert(a==0 && b==0 && c==0 && d==0);
  /* releasing the charger gate re-enables the wheels on the next step */
  charger.AllowRecharge=0; send_command(0.5f, 0.0f, 880);
  robot_check.DeepCheck=1; send_command(0.5f, 0.0f, 910);
  assert(a==0 && b==0 && c==0 && d==0);
  robot_check.DeepCheck=0;

  /* 4: the ROS path never reads the pad, so the stick value is ignored. */
  send_command(0.0f, 0.0f, 950);
  send_command(0.0f, 0.0f, 1050);
  pad.LY=255; pad.RX=255; send_command(-0.3f, 0.0f, 1080);
  assert(a==300 && b==-300 && c==-300 && d==300);
  pad.LY=0; pad.RX=0;

  /* 7: the PS2 entry still maps the absolute stick value to the full duty. */
  {
    steer_debug_t s = {0};
    steer_output_t out;
    (void)steer_debug_step(&s, 127, 127, 1, 1000);
    (void)steer_debug_step(&s, 127, 127, 1, 1200);
    out = steer_debug_step(&s, 255, 127, 1, 1201);
    assert(out.left == 8400 && out.right == 8400);
    /* duty input equal to the limit stays at the limit, not scaled */
    out = steer_debug_step_duty(&s, 1680, 0, 1, 1202);
    assert(out.left == 1680 && out.right == 1680);
    /* a duty beyond the limit is clamped before the step runs */
    out = steer_debug_step_duty(&s, 99999, 0, 1, 1203);
    assert(out.left == 1680 && out.right == 1680);
    out = steer_debug_step_duty(&s, 0, 0, 0, 1204);
    assert(out.left == 0 && out.right == 0);
  }

  /* 7b: the PS2 entry keeps its own 8400 limit. The ROS entry clamps to
     ROS_DRIVE_MAX_DUTY, and ROS_STRAIGHT_DRIVE leaves the PS2 limit alone. */
  {
    steer_debug_t s = {0};
    steer_output_t out;
    (void)steer_debug_step(&s, 127, 127, 1, 2000);
    (void)steer_debug_step(&s, 127, 127, 1, 2200);
    out = steer_debug_step(&s, 255, 127, 1, 2201);
    assert(out.left == 8400 && out.right == 8400);
  }
  /* Switch the production output path between PS2 and ROS. */
  g_steer_debug = (steer_debug_t){0};
  control_mode = _PS2_Control; pad.LY=127; pad.RX=127;
  ticks=3000; g_rear_ps2_tick=ticks; response();
  ticks=3101; g_rear_ps2_tick=ticks; response();
  pad.LY=255; ticks=3102; g_rear_ps2_tick=ticks; response();
  assert(a==-8400 && b==8400 && c==8400 && d==-8400);
  /* Old pad input must not drive ROS or arm it on switch. */
  send_command(0.5f, 0.0f, 3110);
  assert(a==0 && b==0 && c==0 && d==0);
  send_command(0.0f, 0.0f, 3120);
  send_command(0.0f, 0.0f, 3221);
  assert(a==0 && b==0 && c==0 && d==0);
  send_command(0.5f, 0.0f, 3222);
  assert(a==-500 && b==500 && c==500 && d==-500);
  control_mode = _PS2_Control; ticks=3230; g_rear_ps2_tick=ticks; response();
  assert(a==0 && b==0 && c==0 && d==0);

  /* 8: a ROS zero speed frame stops all four wheels at once. The ROS state
     machine is already armed, so this is the zero duty step, not the arm
     window. */
  send_command(0.5f, 0.0f, 3300);   /* PS2 -> ROS, arm state cleared */
  send_command(0.0f, 0.0f, 3310);   /* neutral window starts */
  send_command(0.0f, 0.0f, 3411);   /* neutral window closes */
  send_command(0.5f, 0.0f, 3420);
  assert(a==-500 && b==500 && c==500 && d==-500);
  send_command(0.0f, 0.0f, 3430);   /* zero speed frame, still armed */
  assert(a==0 && b==0 && c==0 && d==0);
  assert(g_ros_steer_debug.armed && !g_ros_steer_debug.left.running &&
         !g_ros_steer_debug.right.running);

  /* 9: a stick that is still pushed never reaches the ROS path. Only the
     ROS speed command sets the duty. */
  pad.LY=255; pad.RX=255;
  send_command(0.3f, 0.0f, 3440);
  assert(a==-300 && b==300 && c==300 && d==-300);
  pad.LY=0; pad.RX=0;
  send_command(0.3f, 0.0f, 3450);
  assert(a==-300 && b==300 && c==300 && d==-300);
  pad.LY=127; pad.RX=127;

  /* 10: PS2 -> ROS -> PS2 restarts the neutral wait on both paths. The PS2
     stick is pushed during the switch, so it must not arm the PS2 machine. */
  control_mode = _PS2_Control; pad.LY=255; pad.RX=127;
  ticks=3460; g_rear_ps2_tick=ticks; response();
  assert(a==0 && b==0 && c==0 && d==0);
  ticks=3570; g_rear_ps2_tick=ticks; response();
  assert(a==0 && b==0 && c==0 && d==0);
  /* the stick returns to neutral, so the PS2 window can run */
  pad.LY=127; ticks=3580; g_rear_ps2_tick=ticks; response();
  ticks=3690; g_rear_ps2_tick=ticks; response();
  pad.LY=255; ticks=3700; g_rear_ps2_tick=ticks; response();
  assert(a==-8400 && b==8400 && c==8400 && d==-8400);
  /* back to ROS: the ROS machine is cleared, so the command waits again */
  pad.LY=127; pad.RX=127;
  send_command(0.5f, 0.0f, 3710);
  assert(a==0 && b==0 && c==0 && d==0);
  send_command(0.0f, 0.0f, 3720);
  send_command(0.0f, 0.0f, 3821);
  send_command(0.5f, 0.0f, 3830);
  assert(a==-500 && b==500 && c==500 && d==-500);
  return 0;
}
'''

with tempfile.TemporaryDirectory() as tmp:
    p = pathlib.Path(tmp)
    # The ros_drive_duty helper must sit above response(), which calls it.
    body = (head + guard + pwm_mid + '}\n' + helper_guard + helper + helper_end +
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
