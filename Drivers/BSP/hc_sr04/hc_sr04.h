#ifndef __HC_SR04_H
#define __HC_SR04_H

#include "system.h"

/* [C50C 2026-11-15] Front HC-SR04 ultrasonic ranger (low-obstacle aid the
 * 2D lidar cannot see). Mounted front-center, about 5 cm above ground.
 * Wiring: Trig -> PD8, Echo -> PD9. Verified free pins: USART3 lives on
 * PB10/PB11 (uartx.h), PD5/6/7 are UART2/OLED; no driver code touches
 * PD8/PD9, and EXTI9_5_IRQn / EXTI_Line9 were unused.
 *
 * Measurement: 10 us Trig pulse -> Echo high pulse width T(us) ->
 * distance = T * 343 / 2000 (mm). Echo edges captured by EXTI9 with the
 * DWT cycle counter (168 MHz, wraps 25.6 s, unsigned delta handles wrap).
 * TIM2..5 are the four wheel encoders and TIM8 is the motor PWM, so no
 * timer peripheral is spent on this driver.
 */

#define HC_SR04_TRIG_PORT     GPIOD
#define HC_SR04_TRIG_PIN      GPIO_PIN_8    /* 批次5：0x0100 与 HAL GPIO_PIN_8 同值 */
/* 批次6：HC_SR04_TRIG_CLK（原 RCC_AHB1Periph_GPIOD）退役——
 * hc_sr04.c:39 改用 __HAL_RCC_GPIOD_CLK_ENABLE()（写同 AHB1ENR 位）。 */
#define HC_SR04_ECHO_PORT     GPIOD
#define HC_SR04_ECHO_PIN      GPIO_PIN_9    /* 批次5：0x0200 与 HAL GPIO_PIN_9 同值 */
/* 批次5：StdPeriph EXTI_Line9/EXTI_PortSourceGPIOD/EXTI_PinSource9 → HAL 命名；
 * EXTI_LINE_9 含 HAL 行类型属性位（EXTI_GPIO|0x09），仅用于 HAL_EXTI_SetConfigLine；
 * ISR 裸寄存器清/查挂起用 HC_SR04_ECHO_PR_MASK（纯 PR 位掩码，写 1 清除）。 */
#define HC_SR04_ECHO_EXTI_LINE   EXTI_LINE_9
#define HC_SR04_ECHO_EXTI_PORT   EXTI_GPIOD
#define HC_SR04_ECHO_PR_MASK     (1uL << 9u)
#define HC_SR04_ECHO_IRQn        EXTI9_5_IRQn

#define HC_SR04_TASK_PRIO     3              /* below BALANCE/DATA tasks (4) */
#define HC_SR04_STK_SIZE      256
#define HC_SR04_TASK_RATE     RATE_10_HZ     /* 100 ms cycle, spec needs >= 60 ms */

#define HC_SR04_MIN_MM        20u            /* spec: 2 cm minimum range */
#define HC_SR04_MAX_MM        4500u          /* spec: 450 cm maximum range */
#define HC_SR04_INVALID_MM    0xFFFFu        /* no echo / stale; ROS maps to max_range */
#define HC_SR04_ECHO_TIMEOUT_US 30000u       /* ~5.1 m, beyond spec max */

void HC_SR04_Init(void);
u16  HC_SR04_GetDistance_mm(void);
void HC_SR04_task(void *pvParameters);

#endif
