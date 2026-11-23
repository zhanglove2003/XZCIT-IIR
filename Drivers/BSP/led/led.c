#include "led.h"
/* 批次4：灯带 PWM 时基/OC 配置走 HAL_TIM_PWM_*（stm32f4xx_tim.c 本批退场）；
 * GPIO 与 LightStrip_R/G/B 直写 CCR 热路径宏（led.h）保持不变。 */
#include "stm32f4xx_hal.h"

/* 批次4：HAL 需要 TIM_HandleTypeDef，本文件以 TIM_TypeDef* 为键（函数签名不动）。
 * 静态池按 Instance 匹配借出：TIM12(R/B 两通道共用) 与 TIM9(G，与 motor.c 的
 * Servo_Senior_Init 高配舵机路径并存——该路径当前无调用方)。 */
static TIM_HandleTypeDef s_led_pwm_handle_pool[2];

static TIM_HandleTypeDef *LED_PWM_Handle_Get(TIM_TypeDef *TIMx)
{
    int i;
    for (i = 0; i < 2; i++)
    {
        if (s_led_pwm_handle_pool[i].Instance == TIMx)
        {
            return &s_led_pwm_handle_pool[i];
        }
    }
    for (i = 0; i < 2; i++)
    {
        if (s_led_pwm_handle_pool[i].Instance == NULL)
        {
            s_led_pwm_handle_pool[i].Instance = TIMx;
            return &s_led_pwm_handle_pool[i];
        }
    }
    return NULL;
}

/**************************************************************************
Function: LED interface initialization
Input   : none
Output  : none
函数功能：LED接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	ENABLE_LED_R_PIN_CLOCK;
	ENABLE_LED_G_PIN_CLOCK;
	ENABLE_LED_B_PIN_CLOCK;

	/* 批次5：迁 HAL——LED_R(PB15)/LED_G(PE6)/LED_B(PB14) 推挽输出，100MHz→VERY_HIGH。 */
	GPIO_InitStructure.Pin   =  LED_R_PIN;
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	HAL_GPIO_Init(LED_R_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin   =  LED_G_PIN;
	HAL_GPIO_Init(LED_G_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin   =  LED_B_PIN;
	HAL_GPIO_Init(LED_B_PORT, &GPIO_InitStructure);

	HAL_GPIO_WritePin(LED_R_PORT,LED_R_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_G_PORT,LED_G_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_B_PORT,LED_B_PIN,GPIO_PIN_SET);

	LED_SetColor(LED_Red);//设置LED颜色,默认为红色
}

static int16_t rgb_map_value(uint8_t user_val)
{
    // 计算输入范围的比例
    float scale = (float)(user_val) / 255.0f ;
    // 使用比例计算输出值
    int16_t result = (int16_t)(scale * 10000);
	
	if( result >=10000  ) result = 10000;
	
	return result;
}

//灯带颜色设置函数
void LightStrip_SetColor( uint8_t r,uint8_t g,uint8_t b )
{
	LightStrip_R = rgb_map_value(r);
	LightStrip_G = rgb_map_value(g);
	LightStrip_B = rgb_map_value(b);
}

//内部函数,初始化pwm的引脚
static void GPIO_PWM_Init(GPIO_TypeDef* GPIOx,uint16_t PINx,TIM_TypeDef* TIMx)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 批次5：迁 HAL——AF 数值与原 if 链一致，经 Alternate 字段写入 AFR
	 * （等价原 GPIO_PinAFConfig）；50MHz→GPIO_SPEED_FREQ_HIGH。 */
	uint32_t GPIO_AF;
	      if( TIMx == TIM1 || TIMx == TIM2 )                                       GPIO_AF = GPIO_AF1_TIM1;
	else if( TIMx == TIM3  || TIMx == TIM4  || TIMx == TIM5 )                     GPIO_AF = GPIO_AF2_TIM3;
	else if( TIMx == TIM8  || TIMx == TIM9  || TIMx == TIM10 || TIMx == TIM11 )   GPIO_AF = GPIO_AF3_TIM8;
	else if( TIMx == TIM12 || TIMx == TIM13 || TIMx == TIM14)                     GPIO_AF = GPIO_AF9_TIM12;

	GPIO_InitStructure.Pin        = PINx;
	GPIO_InitStructure.Mode       = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull       = GPIO_PULLUP;
	GPIO_InitStructure.Speed      = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Alternate  = GPIO_AF;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStructure);
}

//内部函数,pwm的定时器初始化
static void TIMx_PWM_Init(TIM_TypeDef* TIMx,uint8_t TIM_Channel,u16 arr,u16 psc,u16 init_val)
{
	/* 批次4：HAL 等价映射。原实现只赋值 OCMode/OutputState/Pulse/OCPolarity/
	 * OCNIdleState 五个字段（其余 OutputNState/OCNPolarity/OCIdleState 为栈上
	 * 垃圾值）——TIM9/TIM12 无互补输出通道，垃圾位不可观察；HAL 无 StructInit，
	 * 按无效果的复位默认值显式赋值（OutputNState=Disable/OCNPolarity=High/
	 * OCIdleState=Reset），可观察行为等价；OCNIdleState=Set 保留原赋值。
	 * 原 TIM_ClockDivision 写字面值 1（OR 进 CR1 落在 CEN 位，CKD 位域实效 0=DIV1），
	 * HAL 用 TIM_CLOCKDIVISION_DIV1 等价（同 motor.c 批次4 注释）。
	 * ARR 预装载由 Init.AutoReloadPreload=ENABLE 等价 TIM_ARRPreloadConfig(ENABLE)。 */
	TIM_HandleTypeDef *htim = LED_PWM_Handle_Get(TIMx);
	if (htim == NULL)
	{
		return;
	}

	htim->Init.Prescaler         = psc;   //PSC分频
	htim->Init.CounterMode       = TIM_COUNTERMODE_UP;             //计数模式
	htim->Init.Period            = arr;  //驱动灯带使用500Hz PWM信号，避免高频导致芯片发热
	htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;         //时钟分频系数（实效等价）
	htim->Init.RepetitionCounter = 0x0;
	htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;  //使能TIMx在ARR上的预装载寄存器
	(void)HAL_TIM_PWM_Init(htim);

	TIM_OC_InitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.OCMode       = TIM_OCMODE_PWM1;            //使用PWM1模式
	/* 树内旧版 hal_tim.h 无 OutputState/OutputNState 字段：CCxE 由 HAL_TIM_PWM_Start
	 * 置位（等价原 TIM_OutputState_Enable），CCxNE 复位默认 Disable（无互补通道）。 */
	TIM_OCInitStructure.Pulse        = 0;                          //初始比较值 0
	TIM_OCInitStructure.OCPolarity   = TIM_OCPOLARITY_HIGH;        //输出极性为高
	TIM_OCInitStructure.OCNPolarity  = TIM_OCNPOLARITY_HIGH;       //无互补通道，复位默认
	TIM_OCInitStructure.OCIdleState  = TIM_OCIDLESTATE_RESET;      //无 BDTR 空闲电平，复位默认
	TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_SET;       //保留原赋值
	TIM_OCInitStructure.OCFastMode   = TIM_OCFAST_DISABLE;         //复位默认

	if( TIM_Channel==1 )
	{
		(void)HAL_TIM_PWM_ConfigChannel(htim, &TIM_OCInitStructure, TIM_CHANNEL_1); //内部写 OC1PE 等价原 OC1PreloadConfig Enable
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, init_val);   //等价原 TIM_SetCompare1
		(void)HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);           //置 CC1E+CEN，等价原 TIM_Cmd(ENABLE)
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
	/* TIM9/TIM12 非 break 实例（IS_TIM_BREAK_INSTANCE 仅 TIM1/TIM8），Start 不写
	 * BDTR.MOE——原实现也未调 TIM_CtrlPWMOutputs，行为一致。 */
}

//用户pwm通用初始化函数
static void User_PWM_Init(GPIO_TypeDef* GPIOx,uint16_t PINx,TIM_TypeDef* TIMx,uint8_t TIM_Channel,u16 arr,u16 psc,u16 init_val)
{
	GPIO_PWM_Init(GPIOx,PINx,TIMx);
	TIMx_PWM_Init(TIMx,TIM_Channel,arr,psc,init_val);
}

//RGB灯带初始化
void RGB_LightStrip_Init(void)
{
	ENABLE_LightStrip_R_PORT_CLOCK;
	ENABLE_LightStrip_R_TIM_CLOCK;
	
	ENABLE_LightStrip_G_PORT_CLOCK;
	ENABLE_LightStrip_G_TIM_CLOCK;
	
	ENABLE_LightStrip_B_PORT_CLOCK;
	ENABLE_LightStrip_B_TIM_CLOCK;

	User_PWM_Init(LightStrip_R_PORT,LightStrip_R_PIN,LightStrip_R_TIM,LightStrip_R_Channel,10000-1,84-1 ,rgb_map_value(255));
	User_PWM_Init(LightStrip_G_PORT,LightStrip_G_PIN,LightStrip_G_TIM,LightStrip_G_Channel,10000-1,168-1,rgb_map_value(255));
	User_PWM_Init(LightStrip_B_PORT,LightStrip_B_PIN,LightStrip_B_TIM,LightStrip_B_Channel,10000-1,84-1 ,rgb_map_value(255));
	
	LED_SetColor(LED_Red);//设置LED颜色,默认为红色
}

static u8 led_buf[3]={0,0,0};

//设置LED颜色
void LED_SetColor(LED_COLOR color)
{
		  if(color==LED_Red)       led_buf[0] = LED_ON , led_buf[1] = LED_OFF , led_buf[2] = LED_OFF;
	else if( color==LED_Purple )  led_buf[0] = LED_ON , led_buf[1] = LED_OFF , led_buf[2] = LED_ON;
	else if( color==LED_Blue   )  led_buf[0] = LED_OFF , led_buf[1] = LED_OFF , led_buf[2] = LED_ON;
	else if( color==LED_Cyan   )  led_buf[0] = LED_OFF , led_buf[1] = LED_ON , led_buf[2] = LED_ON;
	else if( color==LED_Green  )  led_buf[0] = LED_OFF , led_buf[1] = LED_ON , led_buf[2] = LED_OFF;
	else if( color==LED_Yellow )  led_buf[0] = LED_ON , led_buf[1] = LED_ON , led_buf[2] = LED_OFF;
	else if( color==LED_White  )  led_buf[0] = LED_ON , led_buf[1] = LED_ON , led_buf[2] = LED_ON;
}

//设置LED亮灭
void LED_SetState(uint8_t mode)
{
	if( SysVal.HardWare_Ver >= V1_1 )
	{
		if( mode==1 )
		{
			//pwm模式下的rgb灯带与rgb灯珠控制
			u8 tmp_r=0,tmp_g=0,tmp_b=0;
			tmp_r = (led_buf[0]==LED_ON)?RGB_FULL_ON:RGB_FULL_OFF;
			tmp_g = (led_buf[1]==LED_ON)?RGB_FULL_ON:RGB_FULL_OFF;
			tmp_b = (led_buf[2]==LED_ON)?RGB_FULL_ON:RGB_FULL_OFF;
			
			LightStrip_SetColor(tmp_r,tmp_g,tmp_b);
		}
		else
		{
			LightStrip_SetColor(RGB_FULL_OFF,RGB_FULL_OFF,RGB_FULL_OFF);
		}
	}
}

/**************************************************************************
Function: The LED flashing
Input   : none
Output  : blink time
函数功能：LED闪烁
入口参数：闪烁时间
返 回 值：无
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
	  if(0==time) LED_R=0;
	  else		if(++temp==time)	LED_R=~LED_R,temp=0;
}

