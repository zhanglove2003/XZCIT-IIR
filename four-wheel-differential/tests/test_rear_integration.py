#!/usr/bin/env python3
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
text = (root / 'BALANCE/balance_task.c').read_bytes().decode('gb18030')
base = text.index('static void ResponseControl(void)\n{')
start = text.index('#if defined(REAR_WHEEL_DEBUG)', base)
end = text.index('#ifdef WHEEL_DIR_CALIB', start)
snippet = text[start:end]
head = '''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include "rear_debug.h"
#define REAR_WHEEL_DEBUG 1
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
static void Set_Pwm(int aa,int bb,int cc,int dd,int servo){a=aa;b=bb;c=cc;d=dd;(void)servo;}
'''
# Use the real function body from the feature branch in the firmware.
tail = '''
int main(void){
  GamePadInterface = (void *)&pad; SysVal.Time_count=1231; control_mode=2;
  g_rear_ps2_valid=1; pad.LY=127; ticks=1; g_rear_ps2_tick=ticks; response();
  ticks=101; g_rear_ps2_tick=ticks; response();
  pad.LY=255; ticks=111; g_rear_ps2_tick=ticks; response();
  assert(a==-1500 && b==1500 && c==0 && d==0);
  assert(robot.MOTOR_A.Output==1500 && robot.MOTOR_B.Output==1500);
  ticks=262; response(); assert(a==0 && b==0 && c==0 && d==0);
  return 0;
}
'''
with tempfile.TemporaryDirectory() as tmp:
    p = pathlib.Path(tmp)
    (p/'t.c').write_text(head+'\nstatic void response(void) {\n'+snippet+'\n}\n'+tail)
    subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror','-I',str(root/'BALANCE/Inc'),str(p/'t.c'),str(root/'BALANCE/rear_debug.c'),'-o',str(p/'t')],check=True)
    subprocess.run([str(p/'t')],check=True)
print('rear debug control integration: PASS')
