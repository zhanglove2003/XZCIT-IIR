#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"

//阿克曼转向结构 数值结构体
typedef struct{
	int Bias;                      //前轮转角零点值,可作为转向结构的纠正补偿调节
	int Mid;                       //舵机中值,用于矫正小车复位重启时舵机的方向
	int Max;                       //舵机转动的最大值
	int Min;                       //舵机转动的最小值
}AKM_SERVO_ADC;


/*--------ADC Interface Fun--------*/
//对外接口
void ADC1_Init(void);
void Akm_ServoParam_Init(AKM_SERVO_ADC* p);
u16 Get_ADC1(u8 ch);
u16 Get_ADC1_Average(u8 chn, u8 times);
float Get_battery_volt(void);
extern AKM_SERVO_ADC Akm_Servo;
/*----------------------------------*/

/*--------Battery_PIN config--------*/
//电池电压采集的引脚
/* 批次6：RCC_*PeriphClockCmd → __HAL_RCC_*_CLK_ENABLE（写同 ENR 位，行为等价；StdPeriph 退场） */
#define ENABLE_BATTERY_PIN_CLOCK __HAL_RCC_GPIOB_CLK_ENABLE()
#define Battery_PORT  GPIOB
#define Battery_PIN   GPIO_PIN_0
//批次3：原 StdPeriph ADC_Channel_8=0x08；HAL ADC_CHANNEL_8 数值同为 8，
//Get_ADC1 迁 HAL 后 sConfig.Channel 直作通道号
#define Battery_Ch    8
/*----------------------------------*/

#endif 


