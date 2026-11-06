#include "buzzer.h"
#include "main.h"

/* 提示时序状态。整机只有一个蜂鸣器，单实例即可。 */
typedef struct
{
  uint8_t  active;     /* 1 = 有提示正在播 */
  uint8_t  phase_on;   /* 1 = 当前处于"响"的相位 */
  uint8_t  done;       /* 已经播完的声数 */
  uint32_t phase_start;
} buzzer_t;

static buzzer_t s_buzzer;

static void buzzer_write(uint8_t level)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
                    (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void buzzer_silence(void)
{
  buzzer_write(BUZZER_IDLE_LEVEL);
}

void buzzer_init(void)
{
  s_buzzer.active = 0U;
  s_buzzer.phase_on = 0U;
  s_buzzer.done = 0U;
  s_buzzer.phase_start = 0U;
  buzzer_silence();
}

void buzzer_alarm(void)
{
  /* 已经在播就不重新开始：故障是锁存状态，调用方每轮都会调，
   * 若每次都重置就会一直响个不停。 */
  if (s_buzzer.active != 0U)
  {
    return;
  }

  s_buzzer.active = 1U;
  s_buzzer.phase_on = 1U;
  s_buzzer.done = 0U;
  s_buzzer.phase_start = HAL_GetTick();
  buzzer_write(BUZZER_ACTIVE_LEVEL);
}

void buzzer_step(uint32_t now_ms)
{
  uint32_t elapsed;

  if (s_buzzer.active == 0U)
  {
    return;
  }

  elapsed = now_ms - s_buzzer.phase_start;

  if (s_buzzer.phase_on != 0U)
  {
    if (elapsed >= BUZZER_BEEP_ON_MS)
    {
      s_buzzer.done++;
      s_buzzer.phase_start = now_ms;

      if (s_buzzer.done >= BUZZER_BEEP_COUNT)
      {
        s_buzzer.active = 0U;
        buzzer_silence();
      }
      else
      {
        s_buzzer.phase_on = 0U;
        buzzer_silence();
      }
    }
  }
  else if (elapsed >= BUZZER_BEEP_OFF_MS)
  {
    s_buzzer.phase_on = 1U;
    s_buzzer.phase_start = now_ms;
    buzzer_write(BUZZER_ACTIVE_LEVEL);
  }
  else
  {
    /* 仍在"停"的相位里 */
  }
}

uint8_t buzzer_busy(void)
{
  return s_buzzer.active;
}
