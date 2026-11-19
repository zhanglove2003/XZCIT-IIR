#ifndef __ENCODER_H
#define __ENCODER_H 
#include "sys.h"

// No larger than 65535, because the timer of STM32F103 is 16 bit
//不可大于65535，因为STM32F103的定时器是16位的
#define ENCODER_TIM_PERIOD (u16)(65535)   

//编码器枚举
typedef enum{
	Encoder_A,
	Encoder_B,
	Encoder_C,
	Encoder_D
}ENCODER_t;

/*--------ENCODER Interface Fun --------*/
short Read_Encoder(ENCODER_t e);
/*----------------------------------*/


/*--------ENCODER_A config--------*/
/* 批次6：RCC_*PeriphClockCmd → __HAL_RCC_*_CLK_ENABLE（写同 ENR 位，行为等价）。
 * vendor 隐性 bug 申报：原 8 个 PORT_CLOCK 宏误用 RCC_APB1PeriphClockCmd 包
 * RCC_AHB1Periph_GPIOx——StdPeriph 数值上 GPIOA=0x1=TIM2EN、GPIOB=0x2=TIM3EN，
 * 实际写 APB1ENR（重复使能 TIM2/3）而非 GPIO 端口时钟；端口时钟一直由
 * systemInit 中先行的 motor/UART 初始化使能才未暴露。现按宏名意图迁
 * __HAL_RCC_GPIOx_CLK_ENABLE()，初始化顺序不变，全链完成后 AHB1ENR/APB1ENR
 * 终态与原实现一致（详见批次6 notes）。 */
#define ENABLE_ENCODER_A_TIM_CLOCK    __HAL_RCC_TIM2_CLK_ENABLE()
#define ENBALE_ENCODER_A1_PORT_CLOCK  __HAL_RCC_GPIOA_CLK_ENABLE()
#define ENBALE_ENCODER_A2_PORT_CLOCK  __HAL_RCC_GPIOB_CLK_ENABLE()

#define ENCODER_A_TIM     TIM2             //编码器A接口所使用的定时器

#define ENCODER_A1_PORT   GPIOA            //编码器A接口1号端口
#define ENCODER_A1_PIN    GPIO_PIN_15      //编码器A接口1号引脚

#define ENCODER_A2_PORT   GPIOB            //编码器A接口2号端口
#define ENCODER_A2_PIN    GPIO_PIN_3       //编码器A接口2号引脚
/*----------------------------------*/

/*--------ENCODER_A Interface Fun --------*/
void EncoderA_Init(void);
/*----------------------------------*/


/*--------ENCODER_B config--------*/
#define ENABLE_ENCODER_B_TIM_CLOCK    __HAL_RCC_TIM3_CLK_ENABLE()
#define ENBALE_ENCODER_B1_PORT_CLOCK  __HAL_RCC_GPIOA_CLK_ENABLE()
#define ENBALE_ENCODER_B2_PORT_CLOCK  __HAL_RCC_GPIOA_CLK_ENABLE()

#define ENCODER_B_TIM     TIM3             //编码器B接口所使用的定时器

#define ENCODER_B1_PORT   GPIOA            //编码器B接口1号端口
#define ENCODER_B1_PIN    GPIO_PIN_6       //编码器B接口1号引脚

#define ENCODER_B2_PORT   GPIOA            //编码器B接口2号端口
#define ENCODER_B2_PIN    GPIO_PIN_7       //编码器B接口2号引脚
/*----------------------------------*/

/*--------ENCODER_B Interface Fun --------*/
void EncoderB_Init(void);
/*----------------------------------*/

/*--------ENCODER_C config--------*/
#define ENABLE_ENCODER_C_TIM_CLOCK    __HAL_RCC_TIM4_CLK_ENABLE()
#define ENBALE_ENCODER_C1_PORT_CLOCK  __HAL_RCC_GPIOB_CLK_ENABLE()
#define ENBALE_ENCODER_C2_PORT_CLOCK  __HAL_RCC_GPIOB_CLK_ENABLE()

#define ENCODER_C_TIM     TIM4             //编码器C接口所使用的定时器

#define ENCODER_C1_PORT   GPIOB            //编码器C接口1号端口
#define ENCODER_C1_PIN    GPIO_PIN_6       //编码器C接口1号引脚

#define ENCODER_C2_PORT   GPIOB            //编码器C接口2号端口
#define ENCODER_C2_PIN    GPIO_PIN_7       //编码器C接口2号引脚
/*----------------------------------*/

/*--------ENCODER_C Interface Fun --------*/
void EncoderC_Init(void);
/*----------------------------------*/

/*--------ENCODER_D config--------*/
#define ENABLE_ENCODER_D_TIM_CLOCK    __HAL_RCC_TIM5_CLK_ENABLE()
#define ENBALE_ENCODER_D1_PORT_CLOCK  __HAL_RCC_GPIOA_CLK_ENABLE()
#define ENBALE_ENCODER_D2_PORT_CLOCK  __HAL_RCC_GPIOA_CLK_ENABLE()

#define ENCODER_D_TIM     TIM5             //编码器D接口所使用的定时器

#define ENCODER_D1_PORT   GPIOA            //编码器D接口1号端口
#define ENCODER_D1_PIN    GPIO_PIN_0       //编码器D接口1号引脚

#define ENCODER_D2_PORT   GPIOA            //编码器D接口2号端口
#define ENCODER_D2_PIN    GPIO_PIN_1       //编码器D接口2号引脚
/*----------------------------------*/

/*--------ENCODER_D Interface Fun --------*/
void EncoderD_Init(void);
/*----------------------------------*/


#endif
