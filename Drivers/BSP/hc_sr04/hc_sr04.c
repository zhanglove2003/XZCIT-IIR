#include "hc_sr04.h"
#include "delay.h"
#include "stm32f4xx_hal.h"
/* 批次5：GPIO/EXTI 迁移至 HAL_GPIO_* + HAL_EXTI_SetConfigLine；裸 ISR 函数名
 * EXTI9_5_IRQHandler 与裸 EXTI->PR 保持（蓝图风险3：不走回调机制，避免守卫失效
 * 与 ISR 延迟增加）；DWT 测宽路径原样不动。 */

/* [C50C 2026-11-15] HC-SR04 front ultrasonic ranger driver. See hc_sr04.h.
 * The EXTI ISR measures the Echo high-pulse width with the DWT cycle
 * counter; the 10 Hz task only issues the Trig pulse. The ISR writes the
 * raw distance, the getter applies staleness + median-of-3 filtering.
 */

static volatile u16 g_hcsr04_raw_mm = HC_SR04_INVALID_MM;
static volatile u32 g_hcsr04_rise_cyc = 0;
static volatile u32 g_hcsr04_update_tick = 0;
static u16 g_hcsr04_hist[3];           /* median filter window */
static u8   g_hcsr04_hist_n = 0;

/* Extinct-by-construction guard: if any other EXTI9_5 handler ever appears
 * the linker will fail with a duplicate symbol instead of silently
 * aliasing. */
void EXTI9_5_IRQHandler(void);

static void HC_SR04_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void HC_SR04_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* 批次5：EXTI_InitTypeDef/NVIC_InitTypeDef → HAL_EXTI handle + HAL_NVIC。 */
    static EXTI_HandleTypeDef s_hcsr04_exti;
    EXTI_ConfigTypeDef exti_cfg;

/* 批次6：RCC_*PeriphClockCmd → __HAL_RCC_*_CLK_ENABLE（写同 ENR 位，行为等价；StdPeriph 退场） */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* Trig PD8: push-pull output, idle low（批次5：2MHz→SPEED_FREQ_LOW，
     * 下拉与 ioc PD8 PULLDOWN 一致） */
    GPIO_InitStructure.Pin   = HC_SR04_TRIG_PIN;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
    HAL_GPIO_Init(HC_SR04_TRIG_PORT, &GPIO_InitStructure);
    HAL_GPIO_WritePin(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN, GPIO_PIN_RESET);

    /* Echo PD9: digital input, module drives it（下拉与 ioc PD9 PULLDOWN 一致） */
    GPIO_InitStructure.Pin   = HC_SR04_ECHO_PIN;
    GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
    HAL_GPIO_Init(HC_SR04_ECHO_PORT, &GPIO_InitStructure);

    /* Both edges always on: ISR decides rise/fall by reading the pin, no
     * trigger-polarity reconfiguration race.
     * 批次5：SYSCFG_EXTILineConfig(PD,9) + EXTI_Init(双沿/中断) → HAL_EXTI
     * SetConfigLine 一体完成（内部写 SYSCFG->EXTICR[2] 的 GPIOD 选择与
     * EXTI IMR/RTSR/FTSR）；双沿属性与原码 hc_sr04.c:59 一致，不改单沿
     * （蓝图风险2），ioc:316-320 PD9=EXTI9 下拉一致。 */
    exti_cfg.Line    = HC_SR04_ECHO_EXTI_LINE;
    exti_cfg.Mode    = EXTI_MODE_INTERRUPT;
    exti_cfg.Trigger = EXTI_TRIGGER_RISING_FALLING;
    exti_cfg.GPIOSel = HC_SR04_ECHO_EXTI_PORT;
    (void)HAL_EXTI_SetConfigLine(&s_hcsr04_exti, &exti_cfg);

    /* 批次5：原 NVIC_Init(EXTI9_5_IRQn, 抢占7, 子0, 分组4) → HAL 等价调用
     * （批次2 同款换算：分组4 下 IPR=抢占<<4，抢占 7 原值传入、子优先级恒 0）。 */
    HAL_NVIC_SetPriority(HC_SR04_ECHO_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(HC_SR04_ECHO_IRQn);

    HC_SR04_DwtInit();
}

u16 HC_SR04_GetDistance_mm(void)
{
    u16 raw;
    u16 a, b, c, t;

    /* Stale (trig issued but no echo, e.g. free path beyond spec or sensor
     * unplugged): report invalid so consumers do not act on old data. */
    if ((xTaskGetTickCount() - g_hcsr04_update_tick) > pdMS_TO_TICKS(300))
        return HC_SR04_INVALID_MM;

    raw = g_hcsr04_raw_mm;
    if (raw == HC_SR04_INVALID_MM) {
        g_hcsr04_hist_n = 0;
        return HC_SR04_INVALID_MM;
    }

    /* Median-of-3 on the last valid readings; single-shot outliers
     * (multipath echoes) are dropped. Warm-up passes raw through. */
    if (g_hcsr04_hist_n < 3) {
        g_hcsr04_hist[g_hcsr04_hist_n++] = raw;
        return raw;
    }
    g_hcsr04_hist[0] = g_hcsr04_hist[1];
    g_hcsr04_hist[1] = g_hcsr04_hist[2];
    g_hcsr04_hist[2] = raw;
    a = g_hcsr04_hist[0]; b = g_hcsr04_hist[1]; c = g_hcsr04_hist[2];
    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }
    return b;
}

void HC_SR04_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();

    HC_SR04_Init();

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, F2T(HC_SR04_TASK_RATE));

        /* >=10 us Trig pulse; echo completes within ~30 ms, well inside the
         * 100 ms cycle, so the ISR result is ready before the next read.
         * 批次5：GPIO_SetBits/ResetBits → HAL_GPIO_WritePin。 */
        HAL_GPIO_WritePin(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN, GPIO_PIN_SET);
        delay_us(12);
        HAL_GPIO_WritePin(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN, GPIO_PIN_RESET);
    }
}

/* Echo edge capture. T_us * 343 / 2000 = distance in mm; 30000 us * 343
 * stays inside u32. Unsigned subtraction is wrap-safe for the 25.6 s
 * DWT cycle-counter wrap. */
void EXTI9_5_IRQHandler(void)
{
    u32 now;
    u32 cycles, us, mm;

    /* 批次5：EXTI_GetITStatus/ClearITPendingBit → 裸读/写 EXTI->PR（写 1 清除），
     * 保持裸 ISR 函数名与蓝图风险3 决策；DWT 测宽路径一行不改。 */
    if ((EXTI->PR & HC_SR04_ECHO_PR_MASK) != 0u)
    {
        now = DWT->CYCCNT;
        if (HAL_GPIO_ReadPin(HC_SR04_ECHO_PORT, HC_SR04_ECHO_PIN) != GPIO_PIN_RESET)
        {
            g_hcsr04_rise_cyc = now;                     /* echo rising edge */
        }
        else
        {
            cycles = now - g_hcsr04_rise_cyc;            /* echo falling edge */
            us = cycles / (SystemCoreClock / 1000000u);
            mm = us * 343u / 2000u;
            if (mm < HC_SR04_MIN_MM || mm > HC_SR04_MAX_MM)
                mm = HC_SR04_INVALID_MM;
            g_hcsr04_raw_mm = (u16)mm;
            g_hcsr04_update_tick = xTaskGetTickCountFromISR();
        }
        EXTI->PR = HC_SR04_ECHO_PR_MASK;
    }
}
