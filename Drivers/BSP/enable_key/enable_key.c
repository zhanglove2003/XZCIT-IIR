#include "enable_key.h"
/* 批次5：GPIO 迁移至 HAL_GPIO_Init；ENABLE_KEY_PIN 宏（GPIO_PIN_4=0x0010）与
 * HAL 的 GPIO_PIN_4 数值一致。 */
#include "stm32f4xx_hal.h"

/**************************************************************************
Function: Enable switch pin initialization
Input   : none
Output  : none
函数功能：使能开关引脚初始化
入口参数：无
返回  值：无
**************************************************************************/
void EnableKey_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    ENABLE_EnableKEY_PIN_CLOCK;
    /* 批次5：等价映射 PE4 输入上拉（ioc:321-325 PULLUP 一致）。 */
    GPIO_InitStructure.Pin   = ENABLE_KEY_PIN;
    GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;//上拉
    HAL_GPIO_Init(ENABLE_KEY_PORT, &GPIO_InitStructure);
}
