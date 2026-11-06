#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* main.h 会通过 host_stubs 引入 GPIO_TypeDef / GPIO_PIN_SET 等，
 * 这里只需要补上 HAL_GetTick / HAL_GPIO_WritePin 的实现。 */
#include "main.h"

static uint32_t s_tick;
/* main.h 把 BUZZER_GPIO_Port 定义成 &gpiod，这里补上实例。 */
GPIO_TypeDef gpiod;

uint32_t HAL_GetTick(void) { return s_tick; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, uint32_t value)
{
  if (value != 0U) { port->ODR |= pin; }
  else             { port->ODR &= ~pin; }
}

#include "../Core/Inc/buzzer.h"
#include "../Core/Src/buzzer.c"

static int sounding(void)
{
  uint32_t bit = (BUZZER_GPIO_Port->ODR & BUZZER_Pin) != 0U ? 1U : 0U;
  return bit == BUZZER_ACTIVE_LEVEL;
}

int main(void)
{
  uint32_t i;

  /* 1) 初始化后静音 */
  s_tick = 0U;
  buzzer_init();
  assert(!sounding());
  assert(!buzzer_busy());

  /* 2) 请求后立刻响，且进入忙状态 */
  buzzer_alarm();
  assert(buzzer_busy());
  assert(sounding());

  /* 3) 播完一声前重复请求不得重置计时（否则故障锁存会响个不停） */
  s_tick = BUZZER_BEEP_ON_MS - 1U;
  buzzer_alarm();
  s_tick = BUZZER_BEEP_ON_MS;
  buzzer_step(s_tick);
  assert(!sounding());          /* 说明计时没被重置 */
  assert(buzzer_busy());

  /* 4) 过了 OFF 时长重新响 */
  s_tick = BUZZER_BEEP_ON_MS + BUZZER_BEEP_OFF_MS;
  buzzer_step(s_tick);
  assert(sounding());

  /* 5) 播满 BUZZER_BEEP_COUNT 声后自动静音并退出忙状态 */
  for (i = 0U; i < BUZZER_BEEP_COUNT + 2U; i++)
  {
    s_tick += BUZZER_BEEP_ON_MS;
    buzzer_step(s_tick);
    s_tick += BUZZER_BEEP_OFF_MS;
    buzzer_step(s_tick);
  }
  assert(!buzzer_busy());
  assert(!sounding());

  /* 6) 播完之后可以再次触发 */
  buzzer_alarm();
  assert(buzzer_busy());
  assert(sounding());

  /* 7) 播完后再调 step 是安全的空操作 */
  for (i = 0U; i < 200U; i++)
  {
    s_tick += 10U;
    buzzer_step(s_tick);
  }
  assert(!buzzer_busy());
  assert(!sounding());

  /* 8) 空闲电平与有效电平必须相反 */
  assert(BUZZER_IDLE_LEVEL != BUZZER_ACTIVE_LEVEL);

  puts("buzzer: alarm once / non-blocking / dedupe / 3 beeps / silence OK");
  return 0;
}
