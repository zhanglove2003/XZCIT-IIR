#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef struct { uint32_t ODR; } GPIO_TypeDef;
extern GPIO_TypeDef gpioa, gpiob, gpioc, gpiod, gpioe;
#define GPIOA (&gpioa)
#define GPIOB (&gpiob)
#define GPIOC (&gpioc)
#define GPIOD (&gpiod)
#define GPIOE (&gpioe)
#define GPIO_PIN_0 (1U << 0)
#define GPIO_PIN_1 (1U << 1)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_5 (1U << 5)
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_PIN_8 (1U << 8)
#define GPIO_PIN_9 (1U << 9)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_PIN_11 (1U << 11)
#define GPIO_PIN_12 (1U << 12)
#define GPIO_PIN_13 (1U << 13)
#define GPIO_PIN_14 (1U << 14)
#define GPIO_PIN_15 (1U << 15)
#define GPIO_PIN_SET 1U
#define GPIO_PIN_RESET 0U
typedef struct { uint32_t compare[4]; uint32_t counter; } TIM_HandleTypeDef;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->compare[(c)/4]=(v))
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint32_t, uint32_t);
#define __HAL_TIM_GET_COUNTER(t) ((t)->counter)
#define TIM_CHANNEL_ALL 0xffffU
#define HAL_OK 0
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_LOW 0
#define ADC_CHANNEL_VREFINT 17
#define ADC_CHANNEL_8 8
#define ADC_SAMPLETIME_480CYCLES 480
extern uint16_t host_vrefint_cal;
#define VREFINT_CAL_ADDR (&host_vrefint_cal)
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef struct { int unused; } ADC_HandleTypeDef;
typedef struct { uint32_t Channel, Rank, SamplingTime; } ADC_ChannelConfTypeDef;
uint32_t HAL_GetTick(void);
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
uint32_t HAL_GPIO_ReadPin(GPIO_TypeDef *, uint32_t);
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *, uint32_t);
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *, uint32_t);
int HAL_ADC_ConfigChannel(ADC_HandleTypeDef *, ADC_ChannelConfTypeDef *);
int HAL_ADC_Start(ADC_HandleTypeDef *);
int HAL_ADC_PollForConversion(ADC_HandleTypeDef *, uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *);
int HAL_ADC_Stop(ADC_HandleTypeDef *);
#endif
