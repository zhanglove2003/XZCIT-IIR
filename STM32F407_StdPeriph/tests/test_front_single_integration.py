#!/usr/bin/env python3
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
head = '''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include "rear_debug.h"
#define REAR_WHEEL_DEBUG 1
#define _4WD_CAR 1
#define portTICK_PERIOD_MS 1u
#define pdMS_TO_TICKS(x) (x)
#define CONTROL_DELAY 1000
#define V1_0 0
#define _PS2_Control 2
static uint32_t ticks;
static uint32_t xTaskGetTickCount(void){return ticks;}
static rear_debug_t g_rear_debug;
static int g_rear_ps2_valid;
static uint32_t g_rear_ps2_tick;
typedef struct {int Output; float Target;} Motor;
static struct {Motor MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;} robot;
static struct {int HardWare_Ver,Time_count;} SysVal;
static struct {int AllowRecharge;} charger;
static struct {int DeepCheck;} robot_check;
static struct {uint8_t LY;} pad;
static struct {uint8_t LY;} *GamePadInterface;
static int control_mode;
#define Get_Control_Mode(mask) (control_mode & (mask))
static int a,b,c,d;
static void Set_Pwm(int m_a,int m_b,int m_c,int m_d,int servo){
'''
mid = '''
a=m_a;b=m_b;c=m_c;d=m_d;(void)servo;
}
static void response(void) {
'''
tail = '''
}
int main(void){
  Set_Pwm(100,200,300,400,0);
#if FRONT_TEST_CHANNEL == 6
  assert(a==100 && b==200);
#else
  assert(a==0 && b==0);
#endif
#if FRONT_TEST_CHANNEL == 3
  assert(c==300 && d==0);
#elif FRONT_TEST_CHANNEL == 4
  assert(c==0 && d==400);
#else
  assert(c==300 && d==400);
#endif
  GamePadInterface=(void *)&pad; SysVal.Time_count=1231; control_mode=2;
  g_rear_ps2_valid=1; pad.LY=127; ticks=1; g_rear_ps2_tick=ticks; response();
  ticks=101; g_rear_ps2_tick=ticks; response();
  pad.LY=255; ticks=111; g_rear_ps2_tick=ticks; response();
#if FRONT_TEST_CHANNEL == 6
  assert(a==-1500 && b==1500);
#else
  assert(a==0 && b==0);
#endif
#if FRONT_TEST_CHANNEL == 3
  assert(c==1500 && d==0);
#elif FRONT_TEST_CHANNEL == 4
  assert(c==0 && d==-1500);
#else
  assert(c==1500 && d==-1500);
#endif
  pad.LY=0; ticks=121; g_rear_ps2_tick=ticks; response();
  assert(a==0 && b==0 && c==0 && d==0);
  ticks=141; g_rear_ps2_tick=ticks; response();
#if FRONT_TEST_CHANNEL == 3
  assert(a==0 && b==0 && c==-1500 && d==0);
#elif FRONT_TEST_CHANNEL == 4
  assert(a==0 && b==0 && c==0 && d==1500);
#elif FRONT_TEST_CHANNEL == 5
  assert(a==0 && b==0 && c==-1500 && d==1500);
#else
  assert(a==1500 && b==-1500 && c==-1500 && d==1500);
#endif
  ticks=292; response(); assert(a==0 && b==0 && c==0 && d==0);
  return 0;
}
'''
with tempfile.TemporaryDirectory() as tmp:
    p = pathlib.Path(tmp)
    (p/'t.c').write_text(head+guard+mid+response+tail)
    for ch in (3,4,5,6):
        exe=p/('t'+str(ch))
        subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror','-DFRONT_TEST_CHANNEL='+str(ch),'-I',str(root/'BALANCE/Inc'),str(p/'t.c'),str(root/'BALANCE/rear_debug.c'),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
print('front single channel control integration: PASS')
