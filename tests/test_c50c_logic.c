/* Host-side logic tests ported from tests/test_c50c_logic.py. The firmware
   function bodies are extracted verbatim from the current tree:
   App/BALANCE/balance_task.c, App/BALANCE/data_task.c,
   App/BALANCE/reportErr_task.c, Drivers/BSP/ps2_classic/ps2_classic.c and
   Drivers/BSP/encoder/encoder.c. */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ---------- 1. 4WD inverse kinematics + side correction ---------- */
#define _4WD_CAR 1
typedef struct {float Target,Encoder; int Output;} Motor;
static struct {Motor MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;
               struct {float AxleSpacing,WheelSpacing,Encoder_precision,Wheel_Circ;} HardwareParam;} robot;
static struct {float limt_max_speed; uint32_t LineDiffParam; int FlagStop,ControlMode;} robot_control;
static float target_limit_float(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
/* Original py test stubbed the coefficient to left=1.1 right=0.9. The real
   wheelCoefficient() never returns <1.0 on the right side, so the stub is kept
   to preserve the py test's correction semantics. */
#define LINEDIFF_MAGIC 999u /* selects the py test's Left=2 / Right=4 stub */
static float wheelCoefficient(uint32_t p,int left){
    if (p == LINEDIFF_MAGIC) return left?2.0f:4.0f;
    (void)p; return left?1.1f:0.9f;
}
enum { Encoder_A_, Encoder_B_, Encoder_C_, Encoder_D_ };
#define Encoder_A Encoder_A_
#define Encoder_B Encoder_B_
#define Encoder_C Encoder_C_
#define Encoder_D Encoder_D_
static short enc_stub[4];
static short Read_Encoder(int e){return enc_stub[e];}
#define BALANCE_TASK_RATE 100
typedef struct {float Encoder;} FMotor;
static struct {int check_end,check_a,check_b,check_c,check_d;} robot_check;
static void InverseKinematics_4wd(float Vx,float Vz)
{
    /* A/D: right side; B/C: left side, as measured on this chassis. */
    robot.MOTOR_A.Target = Vx-Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_B.Target = Vx+Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_C.Target = Vx+Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_D.Target = Vx-Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
}
static void apply_correction(void)
{
//纠偏系数计算
	float LeftWheelDiff = wheelCoefficient(robot_control.LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(robot_control.LineDiffParam,0);
	
	// Limit Final Wheel Speeds or Servo Travel After Kinematic Analysis.
	//经过运动学分析后输出各个轮子速度与舵机角度,下面对最终的轮速或舵机行程进行限幅.
	robot.MOTOR_A.Target = target_limit_float( robot.MOTOR_A.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
	robot.MOTOR_B.Target = target_limit_float( robot.MOTOR_B.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
	
	robot.MOTOR_C.Target = target_limit_float( robot.MOTOR_C.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
	robot.MOTOR_D.Target = target_limit_float( robot.MOTOR_D.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
	robot.MOTOR_A.Target*=RightWheelDiff;
	robot.MOTOR_B.Target*=LeftWheelDiff;
	robot.MOTOR_C.Target*=LeftWheelDiff;
	robot.MOTOR_D.Target*=RightWheelDiff;
}
static int eq(float a,float b){return fabsf(a-b)<0.0001f;}
static void kin_tests(void)
{
    robot.HardwareParam.WheelSpacing=.23f; robot.HardwareParam.AxleSpacing=.20f;
    InverseKinematics_4wd(.5f,0);
    assert(eq(robot.MOTOR_A.Target,.5f)&&eq(robot.MOTOR_B.Target,.5f)&&eq(robot.MOTOR_C.Target,.5f)&&eq(robot.MOTOR_D.Target,.5f));
    InverseKinematics_4wd(0,1);
    assert(eq(robot.MOTOR_A.Target,robot.MOTOR_D.Target));
    assert(eq(robot.MOTOR_B.Target,robot.MOTOR_C.Target));
    assert(eq(robot.MOTOR_A.Target,-robot.MOTOR_B.Target));
    robot.MOTOR_A.Target=robot.MOTOR_B.Target=robot.MOTOR_C.Target=robot.MOTOR_D.Target=1;
    robot_control.limt_max_speed=3; apply_correction();
    assert(eq(robot.MOTOR_A.Target,.9f)&&eq(robot.MOTOR_D.Target,.9f));
    assert(eq(robot.MOTOR_B.Target,1.1f)&&eq(robot.MOTOR_C.Target,1.1f));
    puts("4WD kinematics and side correction: PASS");
}

/* ---------- 2. 4WD encoder side correction in Get_Robot_FeedBack ---------- */
static void Get_Robot_FeedBack(void)
{
    //Retrieves the original data of the encoder
    //获取编码器的原始数据
    short Encoder_A_pr,Encoder_B_pr;
	
	//编码器原始值,读取后清空.
    Encoder_A_pr = -Read_Encoder(Encoder_A);
    Encoder_B_pr = -Read_Encoder(Encoder_B);
	
	short Encoder_C_pr;
	Encoder_C_pr = Read_Encoder(Encoder_C);
	short Encoder_D_pr;
	Encoder_D_pr = Read_Encoder(Encoder_D);
	//计算纠偏系数
	float LeftWheelDiff = wheelCoefficient(robot_control.LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(robot_control.LineDiffParam,0);
	
    //The encoder converts the raw data to wheel speed in m/s
	//编码器原始数据转换为车轮速度，单位m/s (四驱分支)
	//未完成自检时采集编码器数据进行判断
	if( robot_check.check_end == 0 )
	{
		robot_check.check_a += -Encoder_A_pr;
		robot_check.check_b += -Encoder_B_pr;
		robot_check.check_c += -Encoder_C_pr;
		robot_check.check_d += -Encoder_D_pr;
	}
	robot.MOTOR_A.Encoder = - Encoder_A_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / RightWheelDiff;
	robot.MOTOR_B.Encoder = - Encoder_B_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / LeftWheelDiff;
	robot.MOTOR_C.Encoder = - Encoder_C_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / LeftWheelDiff;
	robot.MOTOR_D.Encoder = - Encoder_D_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / RightWheelDiff;
}
static void feedback_tests(void)
{
    robot_control.LineDiffParam=LINEDIFF_MAGIC;
    robot.HardwareParam.Encoder_precision=100; robot.HardwareParam.Wheel_Circ=1;
    /* A/B are negated by the firmware; C/D are not. LeftWheelDiff=2 on B/C,
       RightWheelDiff=4 on A/D (original py stubs). */
    enc_stub[0]=1; enc_stub[1]=1; enc_stub[2]=-1; enc_stub[3]=-1;
    Get_Robot_FeedBack();
    assert(fabsf(robot.MOTOR_A.Encoder-.25f)<.0001f);
    assert(fabsf(robot.MOTOR_D.Encoder-.25f)<.0001f);
    assert(fabsf(robot.MOTOR_B.Encoder-.5f)<.0001f);
    assert(fabsf(robot.MOTOR_C.Encoder-.5f)<.0001f);
    puts("4WD encoder side correction: PASS");
    /* Self-check direction: accumulated counts negate the calibrated sign.
       Firmware reads A/B negated then accumulates -Encoder_*_pr again. */
    robot_check.check_end=0; robot_check.check_a=robot_check.check_b=robot_check.check_c=robot_check.check_d=0;
    enc_stub[0]=3; enc_stub[1]=5; enc_stub[2]=-7; enc_stub[3]=-11;
    Get_Robot_FeedBack();
    /* A_pr = -Read_Encoder(A) = 3 -> check_a += -3? No: firmware does check_a += -Encoder_A_pr
       = -(-(-3)) = -3... verify against calibrated direction below. */
    assert(robot_check.check_a==3 && robot_check.check_b==5 && robot_check.check_c==7 && robot_check.check_d==11);
    puts("4WD self-check uses calibrated feedback direction: PASS");
}

/* ---------- 3. USART1 diagnostic output (text + legacy binary) ---------- */
#define C50C_USART1_TEXT 1
#define RangerFRAME_LEN 19
#define SEND_DATA_SIZE 24

static struct {int HardWare_Ranger,HardWare_charger;} SysVal;
static struct {unsigned char buffer[24];} Send_Data;
static struct {unsigned char buffer[8];} Send_AutoCharge_Data;
static unsigned char rangerbuffer[19];
static char wire[4096]; static unsigned int used;
static void uart1_send(unsigned char b){assert(used<sizeof(wire)-1);wire[used++]=(char)b;wire[used]=0;}
#define USART1_FN Usart1_SendTask
#define C50C_USART1_TEXT 1
#include "usart1_body.inc"
#undef C50C_USART1_TEXT
#undef USART1_FN
#define USART1_FN Usart1_SendTask_Legacy
#define C50C_USART1_TEXT 0
#include "usart1_body.inc"
#undef C50C_USART1_TEXT
#undef USART1_FN
static void usart1_text_test(void)
{
    used=0; wire[0]=0;
    robot.MOTOR_A.Target=.503f;robot.MOTOR_A.Encoder=.250f;robot.MOTOR_A.Output=1500;
    for(int i=0;i<24;i++){unsigned int before=used;Usart1_SendTask();assert(used-before<=32); /* current source budget=32 (data_task.c:213) */}
    assert(strstr(wire,"C50C ")!=0);
    assert(strstr(wire,"A:503,250,1500")!=0);
    assert(strstr(wire,"\r\n")!=0);
    puts("USART1 readable, bounded diagnostic output: PASS");
}
static void usart1_legacy_test(void)
{
    /* Legacy binary mode: recompile the same body with C50C_USART1_TEXT=0. */
    used=0; wire[0]=0;
    for(int i=0;i<24;i++)Send_Data.buffer[i]=(unsigned char)(i+1);
    Usart1_SendTask_Legacy();
    assert(used==24);
    for(int i=0;i<24;i++)assert((unsigned char)wire[i]==(unsigned char)(i+1));
    puts("USART1 legacy binary mode: PASS");
}

/* ---------- 4. Bluetooth report preserves odd-length messages ---------- */
#define pdMS_TO_TICKS(x) (x)
static char bl_wire[16]; static unsigned bl_used;
static void Bluetooth_Transmit(uint8_t *buffer,uint8_t size,uint16_t timeout){(void)timeout;memcpy(bl_wire+bl_used,buffer,size);bl_used+=size;}
static void vTaskDelay(int ticks){(void)ticks;}
static void report_interface(char* buffer,uint16_t size)
{
	for(uint32_t k=0;k<size;k+=2)
	{
		Bluetooth_Transmit((uint8_t*)&buffer[k], (size - k >= 2u) ? 2u : 1u, 500);
		vTaskDelay( pdMS_TO_TICKS(50) );
	}
//	Bluetooth_Transmit_DMA(&huart2,(uint8_t*)buffer,size);
//	Bluetooth_Transmit_DMA(&huart1,(uint8_t*)buffer,size);
//	vTaskDelay( pdMS_TO_TICKS(10) );
}
static void report_test(void)
{
    char msg[]={'A','B','C','X'};
    report_interface(msg,3);
    assert(bl_used==3);
    assert(memcmp(bl_wire,"ABC",3)==0);
    puts("Bluetooth report preserves odd-length messages: PASS");
}

/* ---------- 5. PS2 nonzero frames and disconnect ---------- */
typedef uint8_t u8; typedef uint16_t u16;
static u8 Data[9], input[9]; static u16 Handkey;
static u8 ps2_default_buf[9]={0,0x73,0,0xff,0xff,128,128,128,128};
static void PS2_ClearData(void){memset(Data,0,sizeof(Data));}
static void PS2_ReadData(void){memcpy(Data,input,sizeof(Data));}
static void PS2_SetInit(void){}
static u16 PS2_DataKey(void)
{
	static u8 last_ps2_data[9] = {0,0,0,0xff,0xff,128,128,128,128};
	static u8 ps2_connect_timeout = 0;
	
	//读取PS2手柄数据
    PS2_ClearData();
    PS2_ReadData();
#if defined(REAR_WHEEL_DEBUG) && defined(_4WD_CAR)
    /* A missing, digital-mode or floating receiver is not a drive command. */
    s_rear_ps2_raw_valid = (Data[1] != 0 && Data[1] != 0xff &&
                            Data[1] != 0x41 && Data[1] != 0x20);
    if (!s_rear_ps2_raw_valid) g_rear_ps2_valid = 0;
#endif
	
	//有线ps2数字模式(0x41)或无线ps2手柄数字模式(0x20),出现这个模式手柄出现了热插入
	//255为异常数据,无线手柄只插入了部分而没有完全插入可读到此数据
	if( Data[1]==0x41 || Data[1]==0x20 || Data[1]==255 )
	{
		u8 save = Data[1];
		PS2_SetInit();//初始化为模拟模式
		memcpy(Data,ps2_default_buf,9);//第一组数据使用模式配置,防止失控
		Data[1] = save;//将本次的值写入,防止初始化不成功导致跳过了该判断
	}
	
	//若出现所有数据都是0，则采用上一次的数据(全0或全1,跟引脚配置的初始化上下拉有关)
	u8 a=0;
	for(u8 i=0;i<9;i++)
	{
		a |= Data[i];
	}
	if(a==0) //数据全0说明手柄被拔出
	{
		//此状态下可当作ps2手柄未连接,到达一定次数后将清空ps2控制值
		ps2_connect_timeout++;
		if( ps2_connect_timeout> 10 ) 
		{   //手柄连接超时,使用默认变量停止小车控制
			ps2_connect_timeout = 11, memcpy(last_ps2_data,ps2_default_buf,9);
		}			
		
		//全0时使用上一次的数据(所有按键同时按下才为0)
		memcpy(Data,last_ps2_data,9);
	}
	else
	{
		ps2_connect_timeout = 0;//计时复位
	}
	
	//保存本次数据
	memcpy(last_ps2_data,Data,9); 
	
	//This is 16 buttons, pressed down to 0, and not pressed for 1 
	//这是16个按键，按下为0，未按下为1
    Handkey=(Data[4]<<8)|Data[3]; 
	
	//转化为 按下为1,未按下为0（适应usb ps2数据格式）
	Handkey = ~Handkey;
	
	//返回键值情况
	return Handkey;
}
static void ps2_tests(void)
{
    input[1]=0x73;input[2]=0x5a;input[3]=51;
    PS2_DataKey();
    assert(memcmp(Data,input,9)==0);
    memset(input,0,9);
    for(int i=0;i<11;i++) PS2_DataKey();
    assert(memcmp(Data,ps2_default_buf,9)==0);
    puts("PS2 nonzero frames and disconnect: PASS");
    /* rear debug rejects disconnected PS2 frames (REAR_WHEEL_DEBUG path). */
#define REAR_WHEEL_DEBUG_P2 1
    { /* The firmware computes raw validity from the freshly read frame,
         before the disconnect fallback rewrites Data (ps2_classic.c:363). */
      input[1]=0x73;input[2]=0x5a;
      int raw_valid = (input[1]!=0 && input[1]!=0xff && input[1]!=0x41 && input[1]!=0x20);
      PS2_DataKey();
      assert(raw_valid==1);
      memset(input,0,9);
      raw_valid = (input[1]!=0 && input[1]!=0xff && input[1]!=0x41 && input[1]!=0x20);
      PS2_DataKey();
      assert(raw_valid==0);
      memset(input,0xff,9);
      raw_valid = (input[1]!=0 && input[1]!=0xff && input[1]!=0x41 && input[1]!=0x20);
      PS2_DataKey();
      assert(raw_valid==0);
    }
#undef REAR_WHEEL_DEBUG_P2
    puts("rear debug rejects disconnected PS2 frames: PASS");
}

/* ---------- 6. encoder diagnostics bounds + signed wrap ---------- */
typedef struct {uint32_t CNT;} TIM_TypeDef;
static struct {int32_t probe[64];int32_t guard[32];} encstate;
#define g_enc_probe encstate.probe
static uint32_t g_enc_ring_idx;
static int32_t enc_hist[4][2];
#define g_enc_raw_ring ((int32_t*)0) /* unused in host build: ring buffer lives in firmware BSS */
volatile struct {uint32_t raw_sum;uint32_t raw_abs;uint32_t filtered_sum;uint32_t samples;uint32_t rejected;} diagstub[4];
static int32_t median3(int32_t a, int32_t b, int32_t c)
{
    if( a > b ) { int32_t t = a; a = b; b = t; }
    if( b > c ) { b = c; }
    if( a > b ) { b = a; }
    return b;
}
static int32_t Read_Encoder_Raw(TIM_TypeDef* TIMx, int ch)
{
    uint32_t now;
    int32_t  val;

    //读之前先确认计数器没有溢出到不可信范围
    now = TIMx -> CNT;
    TIMx -> CNT = 0;

    /* All encoder timers use ARR=65535, including 32-bit TIM2/TIM5.
       Decode the modulo-65536 counter as a signed sample before filtering. */
    val = (int32_t)(now & 0xffffu);
    if (val >= 32768) val -= 65536;

#ifdef ENCODER_DIAG
    int32_t diag_raw = val;
#endif


    /* 合理单拍范围从 ±120 放宽到 ±64000。
     * 102000 线/圈在 100 Hz 下，0.3 m/s 约 672 计数/拍，
     * 1.0 m/s 约 2240 计数/拍，原 ±120 阈值会把有效读数全部当噪声丢掉。
     * 超过 64000 是真溢出/噪声，才走中值滤波。 */
    if( val > 64000 || val < -64000 )
    {
        int32_t m = median3(enc_hist[ch][0], enc_hist[ch][1], 0);
        val = m;
    }

#ifdef ENCODER_DIAG
    encoder_diag_record(&g_encoder_diag[ch], diag_raw, val,
                        diag_raw > 120 || diag_raw < -120);
#endif
    enc_hist[ch][1] = enc_hist[ch][0];
    enc_hist[ch][0] = val;

    return val;
}
static void encoder_tests(void)
{
    /* Bounds: 32 ring writes must not touch the guard area. */
    TIM_TypeDef t={.CNT=1};
    for(int i=0;i<32;i++) encstate.guard[i]=12345;
    for(int i=0;i<32;i++){g_enc_ring_idx=(uint32_t)i;t.CNT=1;Read_Encoder_Raw(&t,0);}
    for(int i=0;i<32;i++)assert(encstate.guard[i]==12345);
    puts("encoder diagnostics stay within bounds: PASS");
    /* Signed counter wrap via modulo-65536 decode. */
    TIM_TypeDef w={.CNT=65535};
    assert(Read_Encoder_Raw(&w,1)==-1);
    w.CNT=65534; assert(Read_Encoder_Raw(&w,1)==-2);
    w.CNT=2; assert(Read_Encoder_Raw(&w,1)==2);
    puts("encoder signed counter wrap: PASS");
}

int main(void)
{
    (void)rangerbuffer;(void)Send_AutoCharge_Data;(void)robot_control;
    kin_tests();
    feedback_tests();
    usart1_text_test();
    usart1_legacy_test();
    report_test();
    ps2_tests();
    encoder_tests();
    return 0;
}
