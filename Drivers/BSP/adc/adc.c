#include "adc.h"
#include "motor.h"
#include "stm32f4xx_hal.h"		/* 批次3：HAL_ADC_* 声明（adc.h→sys.h 链不引 HAL 头） */

//阿克曼转向滑轨数据
AKM_SERVO_ADC Akm_Servo;

//批次3：ADC1 句柄（ADC1_Init 配置，Get_ADC1 转换路径使用）
static ADC_HandleTypeDef hadc1;

/**************************************************************************
Function: ADC initializes battery voltage detection
Input   : none
Output  : none
函数功能：ADC初始化电池电压检测
入口参数：无
返回  值：无
**************************************************************************/
static void ADC1_Pin_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	ENABLE_BATTERY_PIN_CLOCK;
	/* 批次5：PB0 模拟输入（ioc:141-144 BAT_ADC 一致）；AN 模式下 Speed/OType 无意义，
	 * 原实现未赋值（栈垃圾进 OSPEEDR 无效位），HAL 侧显式置复位默认 LOW 保持等价。 */
    GPIO_InitStructure.Pin   = Battery_PIN;
    GPIO_InitStructure.Mode  = GPIO_MODE_ANALOG;//模拟输入
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;//不带上下拉
    HAL_GPIO_Init(Battery_PORT, &GPIO_InitStructure);

}


void ADC1_Init(void)
{
	ADC1_Pin_Init();//使用ADC1引脚配置（批次5：GPIO 已迁 HAL_GPIO_Init）

	ADC_InitTypeDef ADC_InitStructure;

/* 批次6：RCC_*PeriphClockCmd → __HAL_RCC_*_CLK_ENABLE（写同 ENR 位，行为等价；StdPeriph 退场） */
	__HAL_RCC_ADC1_CLK_ENABLE(); //使能ADC1时钟

	//批次3：原 RCC_APB2PeriphResetCmd(ADC1,ENABLE/DISABLE) 复位序列在 HAL 无等价 API，
	//按规则用 CMSIS 寄存器直写保留同语义（等价 StdPeriph 内部 APB2RSTR |=/~ bit8）。
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;

	//批次3：原 ADC_CommonInit(独立模式/两采样间隔5c/DMA禁/Div4) + ADC_Init(12b/非扫描/
	//       非连续/软件触发/右对齐/1转换) + ADC_Cmd → HAL_ADC_Init + __HAL_ADC_ENABLE。
	//       等价说明：CCR 的 MODE/DMA/DLY 位组复位默认 0 与原 CommonInit 写入值相同，
	//       HAL 只动 ADCPRE(=Div4)；CR1/CR2 由 HAL 逐位写入同终值；EOCS 位置 1
	//       (ADC_EOC_SINGLE_CONV)，单 rank 单次转换下 EOC 置位时机与原 EOCS=0 一致。
	//       风险5：采样时间 480cycles 以现码为准（Get_ADC1 内 ConfigChannel），不采 ioc 的 3cycles。
	ADC_InitStructure.ClockPrescaler       = ADC_CLOCK_SYNC_PCLK_DIV4;      //预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz
	ADC_InitStructure.Resolution           = ADC_RESOLUTION_12B;            //12位模式
	ADC_InitStructure.DataAlign            = ADC_DATAALIGN_RIGHT;           //右对齐
	ADC_InitStructure.ScanConvMode         = DISABLE;                       //非扫描模式
	ADC_InitStructure.EOCSelection         = ADC_EOC_SINGLE_CONV;
	ADC_InitStructure.ContinuousConvMode   = DISABLE;                       //关闭连续转换
	ADC_InitStructure.DiscontinuousConvMode = DISABLE;
	ADC_InitStructure.NbrOfDiscConversion  = 1;
	ADC_InitStructure.NbrOfConversion      = 1;                             //1个转换在规则序列中
	ADC_InitStructure.ExternalTrigConv     = ADC_SOFTWARE_START;            //使用软件触发
	ADC_InitStructure.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; //禁止触发检测
	ADC_InitStructure.DMAContinuousRequests = DISABLE;
	hadc1.Instance = ADC1;
	(void)HAL_ADC_Init(&hadc1);

	__HAL_ADC_ENABLE(&hadc1);  //等价原 ADC_Cmd(ADC1, ENABLE)（置 CR2.ADON）
}

//阿克曼舵机参数初始化
void Akm_ServoParam_Init(AKM_SERVO_ADC* p)
{
	p->Max = 2000;
	p->Min = 1000;
	p->Mid = SERVO_INIT;
}

/**************************************************************************
Function: The AD sampling
Input   : The ADC channels
Output  : AD conversion results
函数功能：AD采样
入口参数：ADC的通道
返回  值：AD转换结果
**************************************************************************/
u16 Get_ADC1(volatile u8 ch)
{
    //批次3：原 ADC_RegularChannelConfig(ADC1,ch,1,480cycles) + ADC_SoftwareStartConv +
    //       while(!EOC) + ADC_GetConversionValue → HAL 四步等价（通道号数值直接参与
    //       SMPR/SQR 位写，Battery_Ch=8 与 HAL ADC_CHANNEL_8 数值相同）。
    ADC_ChannelConfTypeDef sConfig;

    //ADC1,ADC通道,采样时间为480周期（风险5：以现码 480cycles 为准,不采 ioc 的 3cycles）
    sConfig.Channel      = ch;                        //通道号,Battery_Ch=8==HAL ADC_CHANNEL_8
    sConfig.Rank         = 1;                         //一个序列
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;  //480周期
    sConfig.Offset       = 0;
    (void)HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    //使能指定的ADC1的软件转换启动功能(等价原 ADC_SoftwareStartConv)
    (void)HAL_ADC_Start(&hadc1);

    //等待转换结束(HAL_MAX_DELAY 无限等待,等价原 while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)))
    (void)HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

    //返回最近一次ADC1规则组转换的结果(等价原 ADC_GetConversionValue,读 DR 清 EOC)
    return (u16)HAL_ADC_GetValue(&hadc1);
}

/**************************************************************************
Function: Collect multiple ADC values to calculate the average function
Input   : ADC channels and collection times
Output  : AD conversion results
函数功能：采集多次ADC值求平均值函数
入口参数：ADC通道和采集次数
返 回 值：AD转换结果
**************************************************************************/
u16 Get_ADC1_Average(u8 chn, u8 times)
{
    u32 temp_val=0;
    u8 t;
    for(t=0; t<times; t++)
    {
        temp_val+=Get_ADC1(chn);
    }
    return temp_val/times;
}

/**************************************************************************
Function: Read the battery voltage
Input   : none
Output  : Battery voltage in V
函数功能：读取电池电压
入口参数：无
返回  值：电池电压，单位 v
**************************************************************************/
float Get_battery_volt(void)
{
    float Volt;
    //The resistance partial voltage can be obtained by simple analysis according to the schematic diagram
    //电阻分压，具体根据原理图简单分析可以得到
    Volt=Get_ADC1(Battery_Ch)*3.3*11.0/1.0/4096;
    return Volt;
}


