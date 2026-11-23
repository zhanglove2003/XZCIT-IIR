#include "hc_sr04.h"
#include "delay.h"

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
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(HC_SR04_TRIG_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Trig PD8: push-pull output, idle low */
    GPIO_InitStructure.GPIO_Pin   = HC_SR04_TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(HC_SR04_TRIG_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN);

    /* Echo PD9: digital input, module drives it */
    GPIO_InitStructure.GPIO_Pin   = HC_SR04_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(HC_SR04_ECHO_PORT, &GPIO_InitStructure);

    SYSCFG_EXTILineConfig(HC_SR04_ECHO_EXTI_PORT, HC_SR04_ECHO_EXTI_PIN);
    /* Both edges always on: ISR decides rise/fall by reading the pin, no
     * trigger-polarity reconfiguration race. */
    EXTI_InitStructure.EXTI_Line    = HC_SR04_ECHO_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = HC_SR04_ECHO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;  /* low: non-critical */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

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
         * 100 ms cycle, so the ISR result is ready before the next read. */
        GPIO_SetBits(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN);
        delay_us(12);
        GPIO_ResetBits(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN);
    }
}

/* Echo edge capture. T_us * 343 / 2000 = distance in mm; 30000 us * 343
 * stays inside u32. Unsigned subtraction is wrap-safe for the 25.6 s
 * DWT cycle-counter wrap. */
void EXTI9_5_IRQHandler(void)
{
    u32 now;
    u32 cycles, us, mm;

    if (EXTI_GetITStatus(HC_SR04_ECHO_EXTI_LINE) != RESET)
    {
        now = DWT->CYCCNT;
        if (GPIO_ReadInputDataBit(HC_SR04_ECHO_PORT, HC_SR04_ECHO_PIN) != 0)
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
        EXTI_ClearITPendingBit(HC_SR04_ECHO_EXTI_LINE);
    }
}
