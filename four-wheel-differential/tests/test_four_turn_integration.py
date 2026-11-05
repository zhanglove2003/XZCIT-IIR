#!/usr/bin/env python3
import pathlib, subprocess, tempfile
root = pathlib.Path(__file__).resolve().parents[1]
text = (root/'BALANCE/balance_task.c').read_bytes().decode('gb18030')
base=text.index('static void ResponseControl(void)\n{')
start=text.index('#if defined(REAR_WHEEL_DEBUG)',base)
end=text.index('#ifdef WHEEL_DIR_CALIB',start)
response=text[start:end]
base=text.index('static void Set_Pwm(int m_a,int m_b,int m_c,int m_d,int servo)\n{')
start=text.index('{',base)+1
end=text.index('\n\t#if defined AKM_CAR',start)
guard=text[start:end]
head='''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include "steer_debug.h"
#define REAR_WHEEL_DEBUG 1
#define _4WD_CAR 1
#define FRONT_TEST_CHANNEL 6
#define FOUR_TURN_DEBUG 1
#define portTICK_PERIOD_MS 1u
#define pdMS_TO_TICKS(x) (x)
#define CONTROL_DELAY 1000
#define V1_0 0
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
static struct {uint8_t LY,RX;} pad;
static struct {uint8_t LY,RX;} *GamePadInterface;
static int control_mode;
#define Get_Control_Mode(mask) (control_mode & (mask))
static int a,b,c,d;
static void Set_Pwm(int m_a,int m_b,int m_c,int m_d,int servo){
'''
mid='''
a=m_a;b=m_b;c=m_c;d=m_d;(void)servo;
}
static void response(void) {
'''
tail='''
}
int main(void){
  GamePadInterface=(void *)&pad; SysVal.Time_count=1231; control_mode=2;
  g_rear_ps2_valid=1; pad.LY=127; pad.RX=127; ticks=1; g_rear_ps2_tick=ticks; response();
  ticks=101; g_rear_ps2_tick=ticks; response();
  pad.RX=255; ticks=111; g_rear_ps2_tick=ticks; response();
  assert(a>0 && b>0 && c>0 && d>0);
  assert(robot.MOTOR_A.Output<0 && robot.MOTOR_B.Output>0);
  pad.RX=127; ticks=121; g_rear_ps2_tick=ticks; response();
  assert(a==0 && b==0 && c==0 && d==0);
  pad.LY=255; ticks=151; g_rear_ps2_tick=ticks; response();
  assert(a==-8400 && b==8400 && c==8400 && d==-8400);
  ticks=302; response(); assert(a==0 && b==0 && c==0 && d==0);
  /* Re-arm after a stale frame, then exercise both rotation directions. */
  pad.LY=127; pad.RX=127; ticks=310; g_rear_ps2_tick=ticks; response();
  ticks=410; g_rear_ps2_tick=ticks; response();
  pad.RX=0; ticks=411; g_rear_ps2_tick=ticks; response();
  assert(a<0 && b<0 && c<0 && d<0);
  pad.RX=255; ticks=412; g_rear_ps2_tick=ticks; response();
  assert(a==0 && b==0 && c==0 && d==0);
  ticks=432; g_rear_ps2_tick=ticks; response();
  assert(a>0 && b>0 && c>0 && d>0);
  pad.RX=127; ticks=433; g_rear_ps2_tick=ticks; response();
  assert(a==0 && b==0 && c==0 && d==0);
  pad.LY=255; pad.RX=255; ticks=453; g_rear_ps2_tick=ticks; response();
  assert(a==0 && b==8400 && c==8400 && d==0);
  pad.LY=0; pad.RX=0; ticks=454; g_rear_ps2_tick=ticks; response();
  assert(a==0 && b==0 && c==0 && d==0);
  ticks=474; g_rear_ps2_tick=ticks; response();
  assert(a==0 && b==-8400 && c==-8400 && d==0);
  g_rear_ps2_valid=0; ticks=475; response();
  assert(a==0 && b==0 && c==0 && d==0);
  return 0;
}
'''
with tempfile.TemporaryDirectory() as tmp:
 p=pathlib.Path(tmp); (p/'t.c').write_text(head+guard+mid+response+tail)
 subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror','-I',str(root/'BALANCE/Inc'),str(p/'t.c'),str(root/'BALANCE/steer_debug.c'),'-o',str(p/'t')],check=True)
 subprocess.run([str(p/'t')],check=True)
print('four wheel turning integration: PASS')
