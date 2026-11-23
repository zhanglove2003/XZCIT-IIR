#include "encoder.h"
#include "stm32f4xx_hal.h"		/* 批次3：HAL_TIM_Encoder_* 声明（encoder.h→sys.h 链不引 HAL 头） */
#ifdef ENCODER_DIAG
#include "encoder_diag.h"
volatile EncoderDiag g_encoder_diag[4];
#endif

//通用编码器初始化,将传入的定时器以及对应引脚初始化为编码器模式3
//批次5：参数名原为 GPIO_PIN_1/GPIO_PIN_2——启用 HAL_GPIO 后与 hal_gpio.h 的
//GPIO_PIN_1/GPIO_PIN_2 位掩码宏同名冲突（宏展开炸语法），改名 pin1/pin2。
static void Encoder_TI12_ModeInit(GPIO_TypeDef* GPIOx_1,uint16_t pin1,GPIO_TypeDef* GPIOx_2,uint16_t pin2,TIM_TypeDef* TIMx)
{
    TIM_HandleTypeDef       htim_encoder;
    TIM_Encoder_InitTypeDef TIM_EncoderConfig;
    GPIO_InitTypeDef GPIO_InitStructure;

	//引脚配置（批次5：迁 HAL——编码器 8 脚 AF+开漏+上拉，与 ioc OPEN_DRAIN+PULLUP
	//一致（蓝图明确项）；100MHz→GPIO_SPEED_FREQ_VERY_HIGH；AF 数值与原 if 链一致
	//(0x01/0x02/0x03/0x09)，经 Alternate 字段写入 AFR（等价原 GPIO_PinAFConfig））
    GPIO_InitStructure.Pin        = pin1;
    GPIO_InitStructure.Mode       = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Speed      = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pull       = GPIO_PULLUP;
	      if( TIMx == TIM1 || TIMx == TIM2 )                                      GPIO_InitStructure.Alternate = GPIO_AF1_TIM1;
	else if( TIMx == TIM3  || TIMx == TIM4  || TIMx == TIM5 )                     GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;
	else if( TIMx == TIM8  || TIMx == TIM9  || TIMx == TIM10 || TIMx == TIM11 )   GPIO_InitStructure.Alternate = GPIO_AF3_TIM8;
	else if( TIMx == TIM12 || TIMx == TIM13 || TIMx == TIM14)                     GPIO_InitStructure.Alternate = GPIO_AF9_TIM12;
    HAL_GPIO_Init(GPIOx_1, &GPIO_InitStructure);

    GPIO_InitStructure.Pin        = pin2;
    HAL_GPIO_Init(GPIOx_2, &GPIO_InitStructure);


	//批次3：原 TIM_TimeBaseInit(PSC=0/ARR=65535/CKD=DIV1/向上) + TIM_ICInit(滤波6级) +
	//       TIM_EncoderInterfaceConfig(TI12,双上升沿) + 手工 CC1E|CC2E 补位 → HAL_TIM_Encoder_Init。
	//       HAL 一次写全 PSC/ARR 与 SMS/CC1S/CC2S/CCER 极性，原 encoder.c:63-66 记录的
	//       "先 ICInit 再 EncoderInterfaceConfig 否则 CCER 只剩一相"顺序问题在 HAL 中不复存在；
	//       CC1E|CC2E 由下方 HAL_TIM_Encoder_Start(TIM_CHANNEL_ALL) 置位（等价原 77 行补位）。
	htim_encoder.Instance               = TIMx;
	htim_encoder.Init.Prescaler         = 0x0;                          //不分频
	htim_encoder.Init.CounterMode       = TIM_COUNTERMODE_UP;           //TIM向上计数
	htim_encoder.Init.Period            = ENCODER_TIM_PERIOD;           //设定计数器自动重装值(65535)
	htim_encoder.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;       //选择时钟分频：不分频
	htim_encoder.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; //等价复位默认 ARPE=0
	TIM_EncoderConfig.EncoderMode       = TIM_ENCODERMODE_TI12;         //使用编码器模式3
	TIM_EncoderConfig.IC1Polarity       = TIM_ICPOLARITY_RISING;        //上升沿,等价 TIM_ICPolarity_Rising
	TIM_EncoderConfig.IC1Selection      = TIM_ICSELECTION_DIRECTTI;     //CH1→IC1
	TIM_EncoderConfig.IC1Prescaler      = TIM_ICPSC_DIV1;               //不分频,等价 TIM_ICStructInit 默认
	TIM_EncoderConfig.IC1Filter         = 6;                            //滤波(6级),对应原 TIM_ICFilter=6
	TIM_EncoderConfig.IC2Polarity       = TIM_ICPOLARITY_RISING;
	TIM_EncoderConfig.IC2Selection      = TIM_ICSELECTION_DIRECTTI;     //CH2→IC2
	TIM_EncoderConfig.IC2Prescaler      = TIM_ICPSC_DIV1;
	TIM_EncoderConfig.IC2Filter         = 6;
	(void)HAL_TIM_Encoder_Init(&htim_encoder, &TIM_EncoderConfig);

	//清除TIM的更新标志位(等价原 TIM_ClearFlag(TIMx, TIM_FLAG_Update))
	__HAL_TIM_CLEAR_FLAG(&htim_encoder, TIM_FLAG_UPDATE);
	//清空计数值(等价原 TIM_SetCounter(TIMx,0))
	__HAL_TIM_SET_COUNTER(&htim_encoder, 0);

	//启动定时器(等价原 TIM_Cmd(TIMx,ENABLE) + encoder.c:77 手工 CC1E|CC2E 补位)
	(void)HAL_TIM_Encoder_Start(&htim_encoder, TIM_CHANNEL_ALL);
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

short Read_Encoder(ENCODER_t e)
{
    short Encoder_TIM;
    switch(e)
    {
		case Encoder_A:
			Encoder_TIM = (short)Read_Encoder_Raw(ENCODER_A_TIM,0);
			break;
		case Encoder_B:
			Encoder_TIM = (short)Read_Encoder_Raw(ENCODER_B_TIM,1);
			break;
		case Encoder_C:
			Encoder_TIM = (short)Read_Encoder_Raw(ENCODER_C_TIM,2);
			break;
		case Encoder_D:
			Encoder_TIM = (short)Read_Encoder_Raw(ENCODER_D_TIM,3);
			break;
		default:
			Encoder_TIM=0;
    }
    return Encoder_TIM;
}

