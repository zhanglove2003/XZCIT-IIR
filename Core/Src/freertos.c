/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t selfCheckTaskHandle;
const osThreadAttr_t selfCheckTask_attributes = {
  .name = "selfCheckTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartSelfCheckTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 二分排查：临时禁用自检任务
  selfCheckTaskHandle = osThreadNew(StartSelfCheckTask, NULL, &selfCheckTask_attributes);
  */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* A轮(右前)上电自检：等 KEY_USER 按下后，正转3s → 停3s → 反转3s → 停3s，循环2次后
 * 驱动器回睡眠并回到等按键状态（任务常驻，不自删除——空闲任务清理自删除任务会卡死 tick） */
void StartSelfCheckTask(void *argument)
{
  uint8_t cycle;
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* A轮 SLP/EN (PB12)：本分支 .ioc 未配置，在此手动初始化，默认低 = 驱动睡眠 */
  HAL_GPIO_WritePin(MOTOR_RF_SLP_GPIO_Port, MOTOR_RF_SLP_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = MOTOR_RF_SLP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_RF_SLP_GPIO_Port, &GPIO_InitStruct);

  /* 上电即启动 PWM 输出（置 MOE），但占空比保持 0，电机不动 */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);

  for (;;)
  {
    /* 等待 USER 按键（PC5，按下为低），10ms 去抖确认 */
    for (;;)
    {
      if (HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin) == GPIO_PIN_RESET)
      {
        osDelay(10);
        if (HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin) == GPIO_PIN_RESET)
        {
          break;
        }
      }
      osDelay(10);
    }

    /* 唤醒 A 轮驱动器，给足上电稳定时间 */
    HAL_GPIO_WritePin(MOTOR_RF_SLP_GPIO_Port, MOTOR_RF_SLP_Pin, GPIO_PIN_SET);
    osDelay(10);

    for (cycle = 0; cycle < SELFCHECK_CYCLES; cycle++)
    {
      HAL_GPIO_WritePin(MOTOR_RF_DIR_GPIO_Port, MOTOR_RF_DIR_Pin, MOTOR_RF_DIR_FORWARD);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, SELFCHECK_PWM_PULSE);
      osDelay(SELFCHECK_RUN_MS);

      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
      osDelay(SELFCHECK_PAUSE_MS);

      HAL_GPIO_WritePin(MOTOR_RF_DIR_GPIO_Port, MOTOR_RF_DIR_Pin, MOTOR_RF_DIR_REVERSE);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, SELFCHECK_PWM_PULSE);
      osDelay(SELFCHECK_RUN_MS);

      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
      if (cycle < (SELFCHECK_CYCLES - 1))
      {
        osDelay(SELFCHECK_PAUSE_MS);
      }
    }

    /* 收尾：清占空比 → 停 PWM（清 MOE，引脚回低）→ 驱动器回睡眠 */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
    HAL_GPIO_WritePin(MOTOR_RF_SLP_GPIO_Port, MOTOR_RF_SLP_Pin, GPIO_PIN_RESET);

    /* 等按键松开再回到等待，防止松键抖动立刻重新触发 */
    while (HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin) == GPIO_PIN_RESET)
    {
      osDelay(10);
    }
    osDelay(500);
  }
}
/* USER CODE END Application */

