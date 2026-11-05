#!/usr/bin/env python3
import pathlib, subprocess, tempfile
ROOT = pathlib.Path(__file__).resolve().parents[1]

def extract_function(text, signature):
    start = text.index(signature)
    brace = text.index('{', start)
    depth = 0
    for i in range(brace, len(text)):
        depth += (text[i] == '{') - (text[i] == '}')
        if depth == 0:
            return text[start:i+1]
    raise ValueError(signature)

def run_c(source):
    with tempfile.TemporaryDirectory() as tmp:
        p = pathlib.Path(tmp)
        (p/'test.c').write_text(source)
        subprocess.run(['cc', '-std=c99', '-Wall', '-Werror', str(p/'test.c'), '-lm', '-o', str(p/'test')], check=True)
        subprocess.run([str(p/'test')], check=True)

text=(ROOT/'BALANCE/balance_task.c').read_bytes().decode('gb18030')
kin=extract_function(text, 'static void InverseKinematics_4wd(float Vx,float Vz)')
start=text.index('//纠偏系数计算', text.index('static void Drive_Motor(float T_Vx,float T_Vy,float T_Vz)\n{'))
end=text.index('\n}', start)
correction=text[start:end]
pre='''#include <assert.h>\n#include <math.h>\n#define _4WD_CAR 1\ntypedef struct {float Target;} Motor;\nstruct {Motor MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D; struct {float AxleSpacing,WheelSpacing;} HardwareParam;} robot;\nstruct {float limt_max_speed; int LineDiffParam;} robot_control;\nstatic float target_limit_float(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}\nstatic float wheelCoefficient(int p,int left){(void)p;return left?1.1f:0.9f;}\n'''
main='''\nstatic int eq(float a,float b){return fabsf(a-b)<0.0001f;}\nint main(void){ robot.HardwareParam.WheelSpacing=.23f; robot.HardwareParam.AxleSpacing=.20f;\nInverseKinematics_4wd(.5f,0); assert(eq(robot.MOTOR_A.Target,.5f)&&eq(robot.MOTOR_B.Target,.5f)&&eq(robot.MOTOR_C.Target,.5f)&&eq(robot.MOTOR_D.Target,.5f));\nInverseKinematics_4wd(0,1); assert(eq(robot.MOTOR_A.Target,robot.MOTOR_D.Target)); assert(eq(robot.MOTOR_B.Target,robot.MOTOR_C.Target)); assert(eq(robot.MOTOR_A.Target,-robot.MOTOR_B.Target));\nrobot.MOTOR_A.Target=robot.MOTOR_B.Target=robot.MOTOR_C.Target=robot.MOTOR_D.Target=1; robot_control.limt_max_speed=3; apply_correction();\nassert(eq(robot.MOTOR_A.Target,.9f)&&eq(robot.MOTOR_D.Target,.9f)); assert(eq(robot.MOTOR_B.Target,1.1f)&&eq(robot.MOTOR_C.Target,1.1f)); return 0;}\n'''
run_c(pre+kin+'\nstatic void apply_correction(void){\n'+correction+'\n}\n'+main)
print('4WD kinematics and side correction: PASS')
feedback_group=text.index('#elif defined MEC_CAR || defined _4WD_CAR', text.index('static void Get_Robot_FeedBack(void)\n{'))
feedback_start=text.index('        #if defined _4WD_CAR', text.index('robot_check.check_d', feedback_group))
feedback_end=text.index('//probe[10..13]', feedback_start)
feedback=text[feedback_start:feedback_end]
feedback_pre='''#include <assert.h>\n#include <math.h>\n#define _4WD_CAR 1\n#define BALANCE_TASK_RATE 100\ntypedef struct {float Encoder;} Motor;\nstruct {Motor MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D; struct {float Encoder_precision,Wheel_Circ;} HardwareParam;} robot;\n'''
feedback_main='''\nint main(void){float LeftWheelDiff=2,RightWheelDiff=4; short Encoder_A_pr=-1,Encoder_B_pr=-1,Encoder_C_pr=-1,Encoder_D_pr=-1; robot.HardwareParam.Encoder_precision=100;robot.HardwareParam.Wheel_Circ=1;\n'''+feedback+'''\nassert(fabsf(robot.MOTOR_A.Encoder-.25f)<.0001f);assert(fabsf(robot.MOTOR_D.Encoder-.25f)<.0001f);assert(fabsf(robot.MOTOR_B.Encoder-.5f)<.0001f);assert(fabsf(robot.MOTOR_C.Encoder-.5f)<.0001f);return 0;}'''
run_c(feedback_pre+feedback_main)
print('4WD encoder side correction: PASS')
serial_text=(ROOT/'BALANCE/data_task.c').read_bytes().decode('gb18030')
serial_func=extract_function(serial_text,'static void Usart1_SendTask(void)\n{')
serial_pre='''#include <assert.h>\n#include <stdint.h>\n#include <stdio.h>\n#include <string.h>\n#define C50C_USART1_TEXT 1\n#define RangerFRAME_LEN 19\ntypedef struct {float Target,Encoder; int Output;} Motor;\nstruct {Motor MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;} robot;\nstruct {int FlagStop,ControlMode;} robot_control;\nstruct {int HardWare_Ranger,HardWare_charger;} SysVal;\nstruct {unsigned char buffer[24];} Send_Data;\nstruct {unsigned char buffer[8];} Send_AutoCharge_Data;\nunsigned char rangerbuffer[19];\nstatic char wire[4096]; static unsigned int used;\nstatic void uart1_send(unsigned char b){assert(used<sizeof(wire)-1);wire[used++]=(char)b;wire[used]=0;}\n'''
serial_main='''\nint main(void){robot.MOTOR_A.Target=.503f;robot.MOTOR_A.Encoder=.250f;robot.MOTOR_A.Output=1500;\nfor(int i=0;i<24;i++){unsigned int before=used;Usart1_SendTask();assert(used-before<=16);}\nassert(strstr(wire,"C50C ")!=0);assert(strstr(wire,"A:503,250,1500")!=0);assert(strstr(wire,"\\r\\n")!=0);return 0;}\n'''
run_c(serial_pre+serial_func+serial_main)
print('USART1 readable, bounded diagnostic output: PASS')
legacy_pre=serial_pre.replace('#define C50C_USART1_TEXT 1','#define C50C_USART1_TEXT 0')
legacy_main='''\nint main(void){for(int i=0;i<24;i++)Send_Data.buffer[i]=(unsigned char)(i+1);Usart1_SendTask();assert(used==24);for(int i=0;i<24;i++)assert((unsigned char)wire[i]==(unsigned char)(i+1));return 0;}\n'''
run_c(legacy_pre+serial_func+legacy_main)
print('USART1 legacy binary mode: PASS')
report_text=(ROOT/'BALANCE/reportErr_task.c').read_bytes().decode('gb18030')
report_func=extract_function(report_text,'static void report_interface(char* buffer,uint16_t size)')
report_pre='''#include <stdint.h>\n#include <assert.h>\n#include <string.h>\n#define pdMS_TO_TICKS(x) (x)\nstatic char wire[16]; static unsigned used;\nstatic void Bluetooth_Transmit(uint8_t *buffer,uint8_t size,uint16_t timeout){(void)timeout;memcpy(wire+used,buffer,size);used+=size;}\nstatic void vTaskDelay(int ticks){(void)ticks;}\n'''
report_main='''\nint main(void){char msg[]={ 'A','B','C','X' };report_interface(msg,3);assert(used==3);assert(memcmp(wire,"ABC",3)==0);return 0;}\n'''
run_c(report_pre+report_func+report_main)
print('Bluetooth report preserves odd-length messages: PASS')
ps2_text=(ROOT/'HARDWARE/ps2_classic.c').read_bytes().decode('gb18030')
ps2_func=extract_function(ps2_text,'static u16 PS2_DataKey(void)\n{')
ps2_pre='''#include <stdint.h>\n#include <string.h>\n#include <assert.h>\ntypedef uint8_t u8; typedef uint16_t u16;\nstatic u8 Data[9], input[9]; static u16 Handkey;\nstatic u8 ps2_default_buf[9]={0,0x73,0,0xff,0xff,128,128,128,128};\nstatic void PS2_ClearData(void){memset(Data,0,sizeof(Data));}\nstatic void PS2_ReadData(void){memcpy(Data,input,sizeof(Data));}\nstatic void PS2_SetInit(void){}\n'''
ps2_main='''\nint main(void){input[1]=0x73;input[2]=0x5a;input[3]=51;PS2_DataKey();assert(memcmp(Data,input,9)==0);memset(input,0,9);for(int i=0;i<11;i++) PS2_DataKey();assert(memcmp(Data,ps2_default_buf,9)==0);return 0;}\n'''
run_c(ps2_pre+ps2_func+ps2_main)
print('PS2 nonzero frames and disconnect: PASS')
enc_text=(ROOT/'HARDWARE/encoder.c').read_bytes().decode('gb18030')
median=extract_function(enc_text,'static int32_t median3(int32_t a, int32_t b, int32_t c)')
read_raw=extract_function(enc_text,'static int32_t Read_Encoder_Raw(TIM_TypeDef* TIMx, int ch)')
enc_pre='''#include <stdint.h>\n#include <assert.h>\ntypedef struct {uint32_t CNT;} TIM_TypeDef;\nstatic struct {int32_t probe[64];int32_t guard[32];} state;\n#define g_enc_probe state.probe\nstatic uint32_t g_enc_ring_idx;\nstatic int32_t g_enc_raw_ring[32];\nstatic int32_t enc_hist[4][2];\n'''
run_c(enc_pre+median+read_raw+'''\nint main(void){TIM_TypeDef t={.CNT=1};for(int i=0;i<32;i++) state.guard[i]=12345;for(int i=0;i<32;i++){g_enc_ring_idx=(uint32_t)i;t.CNT=1;Read_Encoder_Raw(&t,0);}for(int i=0;i<32;i++)assert(state.guard[i]==12345);return 0;}''')
print('encoder diagnostics stay within bounds: PASS')
run_c(enc_pre+median+read_raw+'''\nint main(void){TIM_TypeDef t={.CNT=65535};assert(Read_Encoder_Raw(&t,1)==-1);t.CNT=65534;assert(Read_Encoder_Raw(&t,1)==-2);t.CNT=2;assert(Read_Encoder_Raw(&t,1)==2);return 0;}''')
print('encoder signed counter wrap: PASS')
feedback_check_start=text.index('if( robot_check.check_end == 0 )',feedback_group)
feedback_check_end=text.index('        #if defined _4WD_CAR',text.index('robot_check.check_d',feedback_check_start))
feedback_check=text[feedback_check_start:feedback_check_end]
check_pre='''#include <assert.h>\n#define _4WD_CAR 1\nstruct {int check_end,check_a,check_b,check_c,check_d;} robot_check;\n'''
check_main='''int main(void){short Encoder_A_pr=-3,Encoder_B_pr=-5,Encoder_C_pr=-7,Encoder_D_pr=-11;\n'''+feedback_check+'''\nassert(robot_check.check_a==3&&robot_check.check_b==5&&robot_check.check_c==7&&robot_check.check_d==11);return 0;}'''
run_c(check_pre+check_main)
print('4WD self-check uses calibrated feedback direction: PASS')
ps2_debug_pre = ps2_pre + '''\n#define REAR_WHEEL_DEBUG 1\n#define _4WD_CAR 1\nstatic u8 s_rear_ps2_raw_valid;\nstatic volatile u8 g_rear_ps2_valid;\n'''
ps2_debug_main = '''\nint main(void){input[1]=0x73;input[2]=0x5a;PS2_DataKey();assert(s_rear_ps2_raw_valid==1);g_rear_ps2_valid=1;memset(input,0,9);PS2_DataKey();assert(s_rear_ps2_raw_valid==0&&g_rear_ps2_valid==0);memset(input,0xff,9);g_rear_ps2_valid=1;PS2_DataKey();assert(s_rear_ps2_raw_valid==0&&g_rear_ps2_valid==0);return 0;}'''
run_c(ps2_debug_pre+ps2_func+ps2_debug_main)
print('rear debug rejects disconnected PS2 frames: PASS')
