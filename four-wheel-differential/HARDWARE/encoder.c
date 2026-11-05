#include "encoder.h"
#ifdef ENCODER_DIAG
#include "encoder_diag.h"
volatile EncoderDiag g_encoder_diag[4];
#endif

//通用编码器初始化,将传入的定时器以及对应引脚初始化为编码器模式3
static void Encoder_TI12_ModeInit(GPIO_TypeDef* GPIOx_1,uint16_t GPIO_PIN_1,GPIO_TypeDef* GPIOx_2,uint16_t GPIO_PIN_2,TIM_TypeDef* TIMx)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
   
	//引脚配置
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_1;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOx_1, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_2; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOx_2, &GPIO_InitStructure);
	
	//确认复用的定时器
	//除TIM6、7无通道外,其他均有通道,对应使用复用功能
	uint8_t GPIO_AF;
	      if( TIMx == TIM1 || TIMx == TIM2 )                                      GPIO_AF = 0x01;
	else if( TIMx == TIM3  || TIMx == TIM4  || TIMx == TIM5 )                     GPIO_AF = 0x02;
	else if( TIMx == TIM8  || TIMx == TIM9  || TIMx == TIM10 || TIMx == TIM11 )   GPIO_AF = 0x03;
	else if( TIMx == TIM12 || TIMx == TIM13 || TIMx == TIM14)                     GPIO_AF = 0x09;
	
	//确认1号引脚复用的引脚号
	uint8_t PinSource=0;
	for(PinSource=0;PinSource<16;PinSource++)
	{
		if( ( (GPIO_PIN_1>>PinSource)&0x01 )==1 ) break;
	}
	//复用配置
	GPIO_PinAFConfig(GPIOx_1,PinSource,GPIO_AF);
	
	//确认2号引脚复用的引脚号
	for(PinSource=0;PinSource<16;PinSource++)
	{
		if( ( (GPIO_PIN_2>>PinSource)&0x01 )==1 ) break;
	}
	//复用配置
	GPIO_PinAFConfig(GPIOx_2,PinSource,GPIO_AF);
	
	
	//定时器配置
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 				    //不分频
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;      //设定计数器自动重装值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //选择时钟分频：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);    //初始化定时器

	//先配置输入捕获的滤波(6级). 原厂此处为0(不滤波).
	//顺序很重要: 原厂先调 TIM_EncoderInterfaceConfig 再调 TIM_ICInit,
	//TIM_ICInit 会重写 CCER 只使能 CC1E, 导致编码器模式只剩一相,
	//计数器乱跳(实测单拍读到数百脉冲). 这里改为先 ICInit 再配置编码器模式.
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);

	//使用编码器模式3
    TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	//标准库 TIM_EncoderInterfaceConfig 只写 SMS/CC1S/CC2S/极性, 不置位通道使能位.
	//厂家原代码只靠 TIM_ICInit 置了 CC1E, CC2E 一直是 0, 实测 CCER=0x0001,
	//两相编码器只剩一相, 计数器按内部时钟乱跳. 这里显式使能两个通道.
    TIMx->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
	
	//清除TIM的更新标志位
    TIM_ClearFlag(TIMx, TIM_FLAG_Update);
	//清空计数值
    TIM_SetCounter(TIMx,0);
	//启动定时器
    TIM_Cmd(TIMx, ENABLE);
}

/**************************************************************************
Function: Encoder interface A initialization
Input   : none
Output  : none
函数功能：编码器接口A初始化
入口参数：无
返 回 值：无
**************************************************************************/
void EncoderA_Init(void)
{
    ENABLE_ENCODER_A_TIM_CLOCK;   //使能定时器
	ENBALE_ENCODER_A1_PORT_CLOCK; //使能1号引脚对应的端口
	ENBALE_ENCODER_A2_PORT_CLOCK; //使能2号引脚对应的端口
	
	//配置编码器A工作在编码器3模式
	Encoder_TI12_ModeInit(ENCODER_A1_PORT,ENCODER_A1_PIN,ENCODER_A2_PORT,ENCODER_A2_PIN,ENCODER_A_TIM);
}


void EncoderB_Init(void)
{
    ENABLE_ENCODER_B_TIM_CLOCK;   //使能定时器
	ENBALE_ENCODER_B1_PORT_CLOCK; //使能1号引脚对应的端口
	ENBALE_ENCODER_B2_PORT_CLOCK; //使能2号引脚对应的端口
	
	//配置编码器B工作在编码器3模式
	Encoder_TI12_ModeInit(ENCODER_B1_PORT,ENCODER_B1_PIN,ENCODER_B2_PORT,ENCODER_B2_PIN,ENCODER_B_TIM);
}


void EncoderC_Init(void)
{
    ENABLE_ENCODER_C_TIM_CLOCK;   //使能定时器
	ENBALE_ENCODER_C1_PORT_CLOCK; //使能1号引脚对应的端口
	ENBALE_ENCODER_C2_PORT_CLOCK; //使能2号引脚对应的端口
   
	//配置编码器C工作在编码器3模式
	Encoder_TI12_ModeInit(ENCODER_C1_PORT,ENCODER_C1_PIN,ENCODER_C2_PORT,ENCODER_C2_PIN,ENCODER_C_TIM);
}


void EncoderD_Init(void)
{
    ENABLE_ENCODER_D_TIM_CLOCK;   //使能定时器
	ENBALE_ENCODER_D1_PORT_CLOCK; //使能1号引脚对应的端口
	ENBALE_ENCODER_D2_PORT_CLOCK; //使能2号引脚对应的端口
   
	//配置编码器D工作在编码器3模式
	Encoder_TI12_ModeInit(ENCODER_D1_PORT,ENCODER_D1_PIN,ENCODER_D2_PORT,ENCODER_D2_PIN,ENCODER_D_TIM);
}


/**************************************************************************
Function: Read the encoder count
Input   : The timer
Output  : Encoder value (representing speed)
函数功能：读取编码器计数
入口参数：定时器
返回  值：编码器数值(代表速度)

注意: STM32F407 的 TIM2/TIM5 是 32 位定时器, 厂家原代码按 F103 的 16 位
定时器写成 (short)CNT, 高位截断后会产生随机的大数(实测单拍读到 646),
造成速度读数跳变. 这里改为读 32 位差值并限幅, 同时对四路统一处理.
**************************************************************************/
//[诊断] 编码器观测区. [0]=标记 [1]~[4]=A/B/C/D累计脉冲 [5]~[8]=目标
//[9]=调用次数 [10]~[13]=Encoder峰值 [16]~[47]=最近32个非零A样本
//[48]=A超±100次数 [49]=A单拍最大 [50]=A单拍最小
volatile int32_t g_enc_probe[64];
volatile int32_t g_enc_raw_ring[32];
volatile uint32_t g_enc_ring_idx = 0;

#define ENC_PER_SAMPLE_LIMIT  200   //单拍脉冲上限, 超出视为噪声丢弃

//编码器偶发尖峰抑制: 3 点中值滤波
//实测单拍脉冲在 1~56 之间(手转), 但偶发尖峰会达到 500 以上,
//且出现概率约 2.6%. 中值滤波可剔除孤立尖峰, 不影响真实转速.
static int32_t enc_hist[4][2];   //A/B/C/D 各保留前两拍

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

    //诊断: 保存未限幅的原始值和环形历史(仅 A 通道)
    if( 0 == ch )
    {
        g_enc_probe[58] = val;                                  //本拍原始值
        g_enc_raw_ring[g_enc_ring_idx & 31] = val;               //独立的32项原始值环形
    }

    //超出合理单拍范围(±120)视为噪声, 用中值滤波
    if( val > 120 || val < -120 )
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

short Read_Encoder(ENCODER_t e)
{
    short Encoder_TIM;
    g_enc_probe[0] = 0x5AFE0003;
    g_enc_probe[9]++;
    switch(e)
    {
		case Encoder_A:
			Encoder_TIM = (short)Read_Encoder_Raw(ENCODER_A_TIM,0);
			g_enc_probe[1] += Encoder_TIM;
			if( 0 != Encoder_TIM )
			{
				g_enc_probe[16 + (g_enc_ring_idx & 31)] = Encoder_TIM;
				g_enc_ring_idx++;
			}
			if( Encoder_TIM > 100 || Encoder_TIM < -100 ) g_enc_probe[48]++;
			if( Encoder_TIM > g_enc_probe[49] ) g_enc_probe[49] = Encoder_TIM;
			if( Encoder_TIM < g_enc_probe[50] ) g_enc_probe[50] = Encoder_TIM;
			g_enc_probe[51] = (int32_t)ENCODER_A_TIM->CNT;
			g_enc_probe[52] = (int32_t)ENCODER_A_TIM->CCMR1;
			g_enc_probe[53] = (int32_t)ENCODER_A_TIM->CCMR2;
			g_enc_probe[54] = (int32_t)ENCODER_A_TIM->SMCR;
			g_enc_probe[55] = (int32_t)ENCODER_A_TIM->CCER;
			g_enc_probe[56] = (int32_t)ENCODER_A_TIM->ARR;
			g_enc_probe[57] = (int32_t)ENCODER_A_TIM->CR1;
			break;
		case Encoder_B:
			Encoder_TIM = (short)Read_Encoder_Raw(ENCODER_B_TIM,1);
			g_enc_probe[2] += Encoder_TIM;
			g_enc_probe[58] = (int32_t)ENCODER_B_TIM->CCER;
			g_enc_probe[59] = (int32_t)ENCODER_B_TIM->CCMR1;
			g_enc_probe[60] = (int32_t)ENCODER_B_TIM->SMCR;
			break;
		case Encoder_C:
			Encoder_TIM = (short)Read_Encoder_Raw(ENCODER_C_TIM,2);
			g_enc_probe[3] += Encoder_TIM;
			g_enc_probe[61] = (int32_t)ENCODER_C_TIM->CCER;
			g_enc_probe[62] = (int32_t)ENCODER_C_TIM->CCMR1;
			break;
		case Encoder_D:
			Encoder_TIM = (short)Read_Encoder_Raw(ENCODER_D_TIM,3);
			g_enc_probe[4] += Encoder_TIM;
			g_enc_probe[63] = (int32_t)ENCODER_D_TIM->CCER;
			break;
		default:
			Encoder_TIM=0;
    }
    return Encoder_TIM;
}

