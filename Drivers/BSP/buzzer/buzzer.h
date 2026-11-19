#ifndef __BUZZER_H
#define __BUZZER_H
#include "sys.h"

enum{
	BEEP_OFF = 0,
	BEEP_ON  = 1
};

/*--------Buzzer config--------*/
/* 批次6：RCC_*PeriphClockCmd → __HAL_RCC_*_CLK_ENABLE（写同 ENR 位，行为等价；StdPeriph 退场） */
#define ENABLE_Buzzer_PIN_CLOCK  __HAL_RCC_GPIOD_CLK_ENABLE()
#define Buzzer_PORT              GPIOD
#define Buzzer_PIN               GPIO_PIN_11
#define Buzzer                   PDout(11)
/*----------------------------------*/


/*--------Buzzer Interface Fun --------*/
void Buzzer_Init(void); 

uint8_t Buzzer_AddTask(uint8_t num_time,uint8_t times);
void Buzzer_task(uint8_t rate);
/*----------------------------------*/


#endif
