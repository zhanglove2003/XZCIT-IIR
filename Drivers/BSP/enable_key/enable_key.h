#ifndef __ENABLE_KEY_H
#define __ENABLE_KEY_H
#include "sys.h"

/*--------EnableKey config--------*/
/* 批次6：RCC_*PeriphClockCmd → __HAL_RCC_*_CLK_ENABLE（写同 ENR 位，行为等价；StdPeriph 退场） */
#define ENABLE_EnableKEY_PIN_CLOCK  __HAL_RCC_GPIOE_CLK_ENABLE()
#define ENABLE_KEY_PORT              GPIOE
#define ENABLE_KEY_PIN               GPIO_PIN_4
#define EN     PEin(4)  
/*----------------------------------*/


/*--------EnableKey Interface Fun --------*/
void EnableKey_Init(void);
/*----------------------------------*/


#endif
