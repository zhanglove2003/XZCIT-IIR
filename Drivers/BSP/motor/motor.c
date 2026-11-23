#include "motor.h"
/* 批次4：PWM 输出走 HAL_TIM_PWM_*（stm32f4xx_tim.c 本批退场）。
 * 批次5：方向脚与 PWM 引脚 GPIO 迁移至 HAL_GPIO_Init（AF 数值用 hal_gpio_ex.h 宏，
 * 与原 StdPeriph GPIO_AF if 链数值一致）；方向脚 PBout/PCout 等位带宏（motor.h）保留。 */
#include "stm32f4xx_hal.h"

/* 批次4：HAL_TIM_PWM_* 需要 TIM_HandleTypeDef，而本文件接口以 TIM_TypeDef* 为键
 * （签名保持不动）。以静态池按 Instance 匹配借出 handle：
 * - 同一定时器的多次通道初始化复用同一 handle，每次 HAL_TIM_PWM_Init 重复写入相同
 *   base 配置（与原实现每通道重复调用 TIM_TimeBaseInit 写同值等价）；
 * - 本文件涉及的实例共 4 个：TIM8(四路电机)、TIM9(高配舵机)、TIM4(顶配舵机，与
 *   encoder.c 的编码器 handle 并存——Servo_Top_Init 当前无调用方，若运行期同时
 *   使用须由调用方保证不冲突)、TIM1(通用路径)。 */
static TIM_HandleTypeDef s_pwm_handle_pool[4];

static TIM_HandleTypeDef *PWM_Handle_Get(TIM_TypeDef *TIMx)
{
    int i;
    for (i = 0; i < 4; i++)
    {
        if (s_pwm_handle_pool[i].Instance == TIMx)
        {
            return &s_pwm_handle_pool[i];
        }
    }
    for (i = 0; i < 4; i++)
    {
        if (s_pwm_handle_pool[i].Instance == NULL)
        {
            s_pwm_handle_pool[i].Instance = TIMx;
            return &s_pwm_handle_pool[i];
        }
    }
    return NULL;
}

/**************************************************************************
Function: Motor orientation pin initialization
Input   : none
Output  : none
函数功能：电机方向引脚初始化
入口参数：无
返回  值：无
**************************************************************************/
//引脚通用配置：将传入的引脚设置为推挽输出模式
static void GPIO_Output_Init(GPIO_TypeDef* GPIOx,uint16_t PINx)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* 批次5：方向脚推挽输出，2MHz→GPIO_SPEED_FREQ_LOW（ioc 各方向脚 PD12=LOW/PC0=LOW
	 * 一致；PC12 ioc 为 NOPULL 而现码统一上拉，输出模式上拉不影响电平，保留现码）。 */
    GPIO_InitStructure.Pin   =  PINx;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStructure);
}

//定时器通用配置：将输入的定时器、对应的引脚，配置为PWM1输出模式 
void TIMx_PWM1_Mode_Init(GPIO_TypeDef* GPIOx,uint16_t PINx,TIM_TypeDef* TIMx,uint8_t TIM_Channel,u16 arr,u16 psc,u16 init_val)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	//确认复用的定时器
	//除TIM6、7无通道外,其他均有通道,对应使用复用功能
	//批次5：AF 数值与原 StdPeriph if 链一致（0x01/0x02/0x03/0x09），改用 hal_gpio_ex.h
	//宏并经 HAL_GPIO_Init 的 Alternate 字段写入 AFR（等价原 GPIO_PinAFConfig）。
	uint32_t GPIO_AF;
	      if( TIMx == TIM1 || TIMx == TIM2 )                                       GPIO_AF = GPIO_AF1_TIM1;
	else if( TIMx == TIM3  || TIMx == TIM4  || TIMx == TIM5 )                     GPIO_AF = GPIO_AF2_TIM3;
	else if( TIMx == TIM8  || TIMx == TIM9  || TIMx == TIM10 || TIMx == TIM11 )   GPIO_AF = GPIO_AF3_TIM8;
	else if( TIMx == TIM12 || TIMx == TIM13 || TIMx == TIM14)                     GPIO_AF = GPIO_AF9_TIM12;

	//配置对应引脚为复用功能
    GPIO_InitStructure.Pin        = PINx;
    GPIO_InitStructure.Mode       = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed      = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pull       = GPIO_PULLUP;
    GPIO_InitStructure.Alternate  = GPIO_AF;
    HAL_GPIO_Init(GPIOx,&GPIO_InitStructure);
	
    //Sets the value of the auto-reload register cycle for the next update event load activity
    //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    /* 批次4：时基等价映射。
     * 原 motor.c:66 TIM_ClockDivision 写字面值 1——StdPeriph TIM_TimeBaseInit 把该值
     * 直接 OR 进 CR1，1 落在 bit0=CEN（TIM_Cmd 随后同样置位），CKD 位域[9:8]实际保持
     * 0=DIV1，故 HAL 用 TIM_CLOCKDIVISION_DIV1 等价（蓝图确认）。
     * ARR 预装载等价 TIM_ARRPreloadConfig(TIMx,ENABLE)：HAL_TIM_Base_SetConfig 内
     * MODIFY_REG(CR1, TIM_CR1_ARPE, Init.AutoReloadPreload) 完整覆盖 ARPE 位。 */
    TIM_HandleTypeDef *htim = PWM_Handle_Get(TIMx);
    if (htim == NULL)
    {
        return;
    }
    htim->Init.Prescaler         = psc;
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;          //原 TIM_CounterMode_Up
    htim->Init.Period            = arr;
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;      //见上：原字面值 1 实效为 DIV1
    htim->Init.RepetitionCounter = 0x0;                         //原 TIM_TimeBaseInit 未写 RCR（复位默认 0）
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; //等价原 TIM_ARRPreloadConfig(TIMx,ENABLE)
    (void)HAL_TIM_PWM_Init(htim);                               //等价原 TIM_TimeBaseInit(arr/psc/CKD/UP)

    /* 批次4：OC 等价映射——保留原"安全修复"精神，全字段显式赋值（HAL 无 StructInit）。
     * 原实现 TIM_OCStructInit 先全量默认再覆盖，映射逐项对应：
     *   TIM_OCMode=PWM1 / TIM_OutputState=Enable / TIM_Pulse=0 /
     *   TIM_OCPolarity=High / TIM_OCNPolarity=High / TIM_OutputNState=Disable /
     *   TIM_OCIdleState=Reset / TIM_OCNIdleState=Reset。
     * OCFastMode 为 HAL 特有字段（F4 StdPeriph 无），置 TIM_OCFAST_DISABLE=复位默认
     * CCMRx.OCxFE=0，无行为差异。 */
    TIM_OC_InitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.OCMode       = TIM_OCMODE_PWM1;            //原 TIM_OCMode_PWM1
    /* 树内旧版 hal_tim.h 的 TIM_OC_InitTypeDef 无 OutputState/OutputNState 字段
     * （新版 ST 官方才有）：CCxE 在 TIM_OC1_SetConfig 中先清零、由 HAL_TIM_PWM_Start
     * 的 TIM_CCxChannelCmd 置位——等价原 TIM_OutputState_Enable + TIM_OCxInit 置
     * CCxE；CCxNE 复位默认 0=Disable，等价原 TIM_OutputNState_Disable。 */
    TIM_OCInitStructure.Pulse        = 0;                          //初始占空比为 0
    TIM_OCInitStructure.OCPolarity   = TIM_OCPOLARITY_HIGH;        //输出比较极性高
    TIM_OCInitStructure.OCNPolarity  = TIM_OCNPOLARITY_HIGH;       //互补极性高有效
    TIM_OCInitStructure.OCIdleState  = TIM_OCIDLESTATE_RESET;      //空闲时输出低
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;     //互补空闲输出低
    TIM_OCInitStructure.OCFastMode   = TIM_OCFAST_DISABLE;         //复位默认

    /* 通道号 1..4 → TIM_CHANNEL_1..4(0x00/0x04/0x08/0x0C)。
     * HAL_TIM_PWM_ConfigChannel 内部写 CCMRx.OCxPE（等价原 OCxPreloadConfig Enable）
     * 与 CCRx=Pulse；__HAL_TIM_SET_COMPARE 对应原 TIM_SetCompareX(init_val)。 */
    if( TIM_Channel==1 )
    {
        (void)HAL_TIM_PWM_ConfigChannel(htim, &TIM_OCInitStructure, TIM_CHANNEL_1);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, init_val);
        (void)HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    }
    else if( TIM_Channel==2 )
    {
        (void)HAL_TIM_PWM_ConfigChannel(htim, &TIM_OCInitStructure, TIM_CHANNEL_2);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, init_val);
        (void)HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    }
    else if( TIM_Channel==3 )
    {
        (void)HAL_TIM_PWM_ConfigChannel(htim, &TIM_OCInitStructure, TIM_CHANNEL_3);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, init_val);
        (void)HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    }
    else if( TIM_Channel==4 )
    {
        (void)HAL_TIM_PWM_ConfigChannel(htim, &TIM_OCInitStructure, TIM_CHANNEL_4);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, init_val);
        (void)HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
    }

    /* 原 TIM_CtrlPWMOutputs(TIMx,ENABLE)（TIM1/TIM8/TIM9 分支）：TIM8/TIM1 的 MOE 由
     * 上方 HAL_TIM_PWM_Start 内部 IS_TIM_BREAK_INSTANCE→__HAL_TIM_MOE_ENABLE 处理；
     * TIM9 无 BDTR 寄存器，原 StdPeriph 写入本就无效，HAL 不写，行为等价。
     * 原 TIM_ARRPreloadConfig(ENABLE) 已由 Init.AutoReloadPreload 覆盖；
     * 原 TIM_Cmd(TIMx,ENABLE) 的 CEN 已由 HAL_TIM_PWM_Start 内 __HAL_TIM_ENABLE 置位。
     * TIM8 四路即四次本函数调用，逐通道 Start（蓝图风险1：漏通道则输出不开）。 */
}

static void MiniBalance_Motor_Init(void)
{
	//开启对应引脚时钟
	ENABLE_AIN1_CLOCK;
	ENABLE_AIN2_CLOCK;
	ENABLE_BIN1_CLOCK;
	ENABLE_BIN2_CLOCK;
	ENABLE_CIN1_CLOCK;
	ENABLE_CIN2_CLOCK;
	ENABLE_DIN1_CLOCK;
	ENABLE_DIN2_CLOCK;
	
	//设置对应引脚推挽输出
	GPIO_Output_Init(AIN1_PORT,AIN1_PIN);
	GPIO_Output_Init(AIN2_PORT,AIN2_PIN);
	
	GPIO_Output_Init(BIN1_PORT,BIN1_PIN);
	GPIO_Output_Init(BIN2_PORT,BIN2_PIN);
	
	GPIO_Output_Init(CIN1_PORT,CIN1_PIN);
	GPIO_Output_Init(CIN2_PORT,CIN2_PIN);
	
	GPIO_Output_Init(DIN1_PORT,DIN1_PIN);
	GPIO_Output_Init(DIN2_PORT,DIN2_PIN);
	
	AIN1 = 0; AIN2 = 0;
	BIN1 = 0; BIN2 = 0;
	CIN1 = 0; CIN2 = 0;
	DIN1 = 0; DIN2 = 0;
}

static void V1_0_MiniBalance_Motor_Init(void)
{
	//开启对应引脚时钟
	ENABLE_AIN1_CLOCK;
	ENABLE_AIN2_CLOCK;
	V1_0_ENABLE_BIN1_CLOCK;
	V1_0_ENABLE_BIN2_CLOCK;
	ENABLE_CIN1_CLOCK;
	ENABLE_CIN2_CLOCK;
	ENABLE_DIN1_CLOCK;
	ENABLE_DIN2_CLOCK;
	
	//设置对应引脚推挽输出
	GPIO_Output_Init(AIN1_PORT,AIN1_PIN);
	GPIO_Output_Init(AIN2_PORT,AIN2_PIN);
	
	GPIO_Output_Init(V1_0_BIN1_PORT,V1_0_BIN1_PIN);
	GPIO_Output_Init(V1_0_BIN2_PORT,V1_0_BIN2_PIN);
	
	GPIO_Output_Init(CIN1_PORT,CIN1_PIN);
	GPIO_Output_Init(CIN2_PORT,CIN2_PIN);
	
	GPIO_Output_Init(DIN1_PORT,DIN1_PIN);
	GPIO_Output_Init(DIN2_PORT,DIN2_PIN);
	
	AIN1 = 0; AIN2 = 0;
	V1_0_BIN1 = 0; V1_0_BIN2 = 0;
	CIN1 = 0; CIN2 = 0;
	DIN1 = 0; DIN2 = 0;
}

/**************************************************************************
Function: The motor PWM initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
函数功能：电机PWM引脚初始化
入口参数：arr：自动重装值  psc：时钟预分频数
返回  值：无
**************************************************************************/
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{
	MiniBalance_Motor_Init();//电机方向引脚初始化
	
	//使能PWM对应的定时器与引脚端口时钟
	ENABLE_PWMA_TIM_CLOCK;
	ENABLE_PWMA_PORT_CLOCK;
	
	ENABLE_PWMB_TIM_CLOCK;
	ENABLE_PWMB_PORT_CLOCK;
	
	ENABLE_PWMC_TIM_CLOCK;
	ENABLE_PWMC_PORT_CLOCK;
	
	ENABLE_PWMD_TIM_CLOCK;
	ENABLE_PWMD_PORT_CLOCK;
	
	//将对应的定时器与对应的引脚配置为 PWM1 模式
	TIMx_PWM1_Mode_Init(PWMA_PORT,PWMA_PIN,PWMA_TIM,PWMA_Channel,arr,psc,0);
	TIMx_PWM1_Mode_Init(PWMB_PORT,PWMB_PIN,PWMB_TIM,PWMB_Channel,arr,psc,0);
	TIMx_PWM1_Mode_Init(PWMC_PORT,PWMC_PIN,PWMC_TIM,PWMC_Channel,arr,psc,0);
	TIMx_PWM1_Mode_Init(PWMD_PORT,PWMD_PIN,PWMD_TIM,PWMD_Channel,arr,psc,0);
	
}

void V1_0_MiniBalance_PWM_Init(u16 arr,u16 psc)
{
	V1_0_MiniBalance_Motor_Init();//电机方向引脚初始化
	
	//使能PWM对应的定时器与引脚端口时钟
	ENABLE_PWMA_TIM_CLOCK;
	ENABLE_PWMA_PORT_CLOCK;
	
	ENABLE_PWMB_TIM_CLOCK;
	ENABLE_PWMB_PORT_CLOCK;
	
	ENABLE_PWMC_TIM_CLOCK;
	ENABLE_PWMC_PORT_CLOCK;
	
	ENABLE_PWMD_TIM_CLOCK;
	ENABLE_PWMD_PORT_CLOCK;
	
	//将对应的定时器与对应的引脚配置为 PWM1 模式
	TIMx_PWM1_Mode_Init(PWMA_PORT,PWMA_PIN,PWMA_TIM,PWMA_Channel,arr,psc,0);
	TIMx_PWM1_Mode_Init(PWMB_PORT,PWMB_PIN,PWMB_TIM,PWMB_Channel,arr,psc,0);
	TIMx_PWM1_Mode_Init(PWMC_PORT,PWMC_PIN,PWMC_TIM,PWMC_Channel,arr,psc,0);
	TIMx_PWM1_Mode_Init(PWMD_PORT,PWMD_PIN,PWMD_TIM,PWMD_Channel,arr,psc,0);
}

/**************************************************************************
Function: Senior Ackerman model servo initialization
Input   : ARR: Automatic reload value  PSC: clock preset frequency
Output  : none
函数功能：高配阿克曼车型舵机初始化
入口参数：arr：自动重装值  psc：时钟预分频数
返回  值：无
**************************************************************************/
void Servo_Senior_Init(u16 arr,u16 psc,int servomid)
{
	ENABLE_SERVO_SENIOR_TIM_CLOCK;
	ENABLE_SERVO_SENIOR_PORT_CLOCK;
	TIMx_PWM1_Mode_Init(SERVO_SENIOR_PORT,SERVO_SENIOR_PIN,SERVO_SENIOR_TIM,SERVO_SENIOR_Channel,arr,psc,servomid);
}

void Servo_Top_Init(u16 arr,u16 psc,int servomid)
{
	ENABLE_SERVO_TOP_TIM_CLOCK;
	ENABLE_SERVO_TOP_PORT_CLOCK;
	TIMx_PWM1_Mode_Init(SERVO_TOP_PORT,SERVO_TOP_PIN,SERVO_TOP_TIM,SERVO_TOP_Channel,arr,psc,servomid);
}
