/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_SW_Pin GPIO_PIN_4
#define EN_SW_GPIO_Port GPIOE
#define RGB_G_Pin GPIO_PIN_6
#define RGB_G_GPIO_Port GPIOE
#define MOTOR_LF_DIR_Pin GPIO_PIN_0
#define MOTOR_LF_DIR_GPIO_Port GPIOC
#define ENC_D_A_Pin GPIO_PIN_0
#define ENC_D_A_GPIO_Port GPIOA
#define ENC_D_B_Pin GPIO_PIN_1
#define ENC_D_B_GPIO_Port GPIOA
#define ENC_B_A_Pin GPIO_PIN_6
#define ENC_B_A_GPIO_Port GPIOA
#define ENC_B_B_Pin GPIO_PIN_7
#define ENC_B_B_GPIO_Port GPIOA
#define KEY_USER_Pin GPIO_PIN_5
#define KEY_USER_GPIO_Port GPIOC
#define ADC_BATT_Pin GPIO_PIN_0
#define ADC_BATT_GPIO_Port GPIOB
#define MOTOR_RF_DIR_Pin GPIO_PIN_13
#define MOTOR_RF_DIR_GPIO_Port GPIOB
#define RGB_B_Pin GPIO_PIN_14
#define RGB_B_GPIO_Port GPIOB
#define RGB_R_Pin GPIO_PIN_15
#define RGB_R_GPIO_Port GPIOB
#define MOTOR_RR_DIR_Pin GPIO_PIN_10
#define MOTOR_RR_DIR_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_11
#define BUZZER_GPIO_Port GPIOD
#define MOTOR_LR_PWM_Pin GPIO_PIN_6
#define MOTOR_LR_PWM_GPIO_Port GPIOC
#define MOTOR_RR_PWM_Pin GPIO_PIN_7
#define MOTOR_RR_PWM_GPIO_Port GPIOC
#define MOTOR_LF_PWM_Pin GPIO_PIN_8
#define MOTOR_LF_PWM_GPIO_Port GPIOC
#define MOTOR_RF_PWM_Pin GPIO_PIN_9
#define MOTOR_RF_PWM_GPIO_Port GPIOC
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define ENC_A_A_Pin GPIO_PIN_15
#define ENC_A_A_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_10
#define BT_TX_GPIO_Port GPIOC
#define BT_RX_Pin GPIO_PIN_11
#define BT_RX_GPIO_Port GPIOC
#define MOTOR_LR_DIR_Pin GPIO_PIN_12
#define MOTOR_LR_DIR_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_6
#define OLED_SDA_GPIO_Port GPIOD
#define OLED_SCL_Pin GPIO_PIN_7
#define OLED_SCL_GPIO_Port GPIOD
#define ENC_A_B_Pin GPIO_PIN_3
#define ENC_A_B_GPIO_Port GPIOB
#define ENC_C_A_Pin GPIO_PIN_6
#define ENC_C_A_GPIO_Port GPIOB
#define ENC_C_B_Pin GPIO_PIN_7
#define ENC_C_B_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_8
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_9
#define I2C1_SDA_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_4
#define OLED_DC_GPIO_Port GPIOD
#define OLED_RES_Pin GPIO_PIN_5
#define OLED_RES_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
/* 四轮驱动 SLP/EN：高 = 使能，低 = 睡眠（板上 10kΩ 下拉，不拉高不转）
 * B 轮 EN=PB14 为实板核对结果（2026-10-29），PR #2 规划的 PE15 有误 */
#define MOTOR_RF_SLP_Pin GPIO_PIN_12
#define MOTOR_RF_SLP_GPIO_Port GPIOB
#define MOTOR_LF_SLP_Pin GPIO_PIN_14
#define MOTOR_LF_SLP_GPIO_Port GPIOB
#define MOTOR_RR_SLP_Pin GPIO_PIN_12
#define MOTOR_RR_SLP_GPIO_Port GPIOD
#define MOTOR_LR_SLP_Pin GPIO_PIN_8
#define MOTOR_LR_SLP_GPIO_Port GPIOA
/* 方向电平：装车后若某轮与实际前进方向相反，单独交换该轮的两个宏即可 */
#define MOTOR_RF_DIR_FORWARD GPIO_PIN_SET
#define MOTOR_RF_DIR_REVERSE GPIO_PIN_RESET
#define MOTOR_LF_DIR_FORWARD GPIO_PIN_RESET
#define MOTOR_LF_DIR_REVERSE GPIO_PIN_SET
#define MOTOR_RR_DIR_FORWARD GPIO_PIN_SET
#define MOTOR_RR_DIR_REVERSE GPIO_PIN_RESET
#define MOTOR_LR_DIR_FORWARD GPIO_PIN_RESET
#define MOTOR_LR_DIR_REVERSE GPIO_PIN_SET
/* 自检参数：TIM8 PSC=9/ARR=16799 → 1kHz，8400 ≈ 50% 占空比 */
#define SELFCHECK_PWM_PULSE 8400U
#define SELFCHECK_RUN_MS 500U
#define SELFCHECK_PAUSE_MS 200U
#define SELFCHECK_CYCLES 1U
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
