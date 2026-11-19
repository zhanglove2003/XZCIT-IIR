#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"


/*--------KEY config--------*/
/* 批次6：RCC_*PeriphClockCmd → __HAL_RCC_*_CLK_ENABLE（写同 ENR 位，行为等价；StdPeriph 退场） */
#define ENABLE_USER_KEY_PIN_CLOCK  __HAL_RCC_GPIOC_CLK_ENABLE()
#define USER_KEY_PORT              GPIOC
#define USER_KEY_PIN               GPIO_PIN_5
#define KEY                        PCin(5)
/*----------------------------------*/


/*--------KEY Interface Fun --------*/
enum { //按键状态枚举
	key_stateless,
	single_click,
	double_click,
	long_click
};
void KEY_Init(void);
u8 KEY_Scan(u16 Frequency,u16 filter_times);
/*---------------------------------*/



#endif 
