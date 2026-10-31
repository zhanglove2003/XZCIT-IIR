#include "ps2_gamepad.h"

#include "main.h"
#include "stm32f4xx_hal.h"

/* 时序与厂家 pstwo.c 一致：DO 建立后 CLK 高→低→高各一拍，
 * 位间隔 16us，半拍 5us；DI 在最后一次 CLK 上升沿之后采样 */
#define PS2_BIT_GAP_US 16U
#define PS2_HALF_US     5U

/* Data 布局：0=id 1=mode 2=0x5A 3/4=键位(低有效) 5=RX 6=RY 7=LX 8=LY */
#define PS2_DATA_LEN 9U

static uint8_t s_data[PS2_DATA_LEN];
volatile ps2_diag_t ps2_diag;
static uint32_t s_last_poll_tick;

static void ps2_delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);

  while ((DWT->CYCCNT - start) < ticks)
  {
  }
}

static void ps2_pin_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* DI=PE0 输入，厂家用下拉；空闲时接收器上拉会读到 1 */
  gpio.Pin = GPIO_PIN_0;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLDOWN;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &gpio);

  /* DO=PE1 / CS=PE2 / CLK=PE3 推挽输出，初始高 */
  gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &gpio);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);
}

static void ps2_dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void ps2_init(void)
{
  ps2_dwt_init();
  ps2_pin_init();
}

/* 收发一个字节：发送 cmd，同时把 DI 移入 *out */
static void ps2_byte(uint8_t cmd, uint8_t *out)
{
  uint16_t ref;
  uint8_t rx = 0U;

  for (ref = 0x01U; ref < 0x0100U; ref <<= 1)
  {
    if ((cmd & ref) != 0U)
    {
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    ps2_delay_us(PS2_BIT_GAP_US);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    ps2_delay_us(PS2_HALF_US);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    ps2_delay_us(PS2_HALF_US);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) != GPIO_PIN_RESET)
    {
      rx |= (uint8_t)ref;
    }
    ps2_delay_us(PS2_BIT_GAP_US);
  }

  *out = rx;
  ps2_delay_us(PS2_BIT_GAP_US);
}

/* 完整事务：CS 拉低，发 ncmd 个命令字节后补零收满 9 字节，CS 拉高 */
static void ps2_transfer(const uint8_t *cmd, uint8_t ncmd)
{
  uint8_t i;

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
  ps2_delay_us(PS2_BIT_GAP_US);

  for (i = 0U; i < PS2_DATA_LEN; i++)
  {
    ps2_byte(i < ncmd ? cmd[i] : 0x00U, &s_data[i]);
  }

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
  ps2_delay_us(PS2_BIT_GAP_US);
}

static void ps2_short_poll(void)
{
  const uint8_t cmd[5] = {0x01U, 0x42U, 0x00U, 0x00U, 0x00U};
  ps2_transfer(cmd, 5U);
}

static void ps2_enter_config(void)
{
  const uint8_t cmd[9] = {0x01U, 0x43U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  ps2_transfer(cmd, 9U);
}

static void ps2_analog_mode(void)
{
  /* 0x01=模拟模式，0x03=锁定，MODE 键不再切换 */
  const uint8_t cmd[9] = {0x01U, 0x44U, 0x00U, 0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U};
  ps2_transfer(cmd, 9U);
}

static void ps2_exit_config(void)
{
  const uint8_t cmd[9] = {0x01U, 0x43U, 0x00U, 0x00U, 0x5AU, 0x5AU, 0x5AU, 0x5AU, 0x5AU};
  ps2_transfer(cmd, 9U);
}

void ps2_config(void)
{
  ps2_diag.configs++;
  /* Receiver startup and configuration need time between transactions.
   * A live board probe reached 0x73 reliably with these 20 ms gaps. */
  ps2_short_poll();
  HAL_Delay(20U);
  ps2_short_poll();
  HAL_Delay(20U);
  ps2_short_poll();
  HAL_Delay(20U);
  ps2_enter_config();
  HAL_Delay(20U);
  ps2_analog_mode();
  HAL_Delay(20U);
  ps2_exit_config();
  HAL_Delay(20U);
}

uint8_t ps2_poll(ps2_state_t *out)
{
  const uint8_t cmd[2] = {0x01U, 0x42U};
  uint8_t i;
  uint8_t valid;
  uint32_t tick = HAL_GetTick();
  uint32_t start = DWT->CYCCNT;

  if (out == NULL)
  {
    return PS2_FRAME_BAD;
  }

  for (i = 0U; i < PS2_DATA_LEN; i++)
  {
    s_data[i] = 0x00U;
  }

  ps2_transfer(cmd, 2U);

  valid = ps2_decode_frame(s_data, out);
  ps2_diag.poll_us = (DWT->CYCCNT - start) / (SystemCoreClock / 1000000U);
  if (ps2_diag.poll_us > ps2_diag.max_poll_us)
  {
    ps2_diag.max_poll_us = ps2_diag.poll_us;
  }
  if (ps2_diag.polls != 0U && tick - s_last_poll_tick > ps2_diag.max_gap_ms)
  {
    ps2_diag.max_gap_ms = tick - s_last_poll_tick;
  }
  s_last_poll_tick = tick;
  ps2_diag.mode = s_data[1];
  ps2_diag.marker = s_data[2];
  for (i = 0U; i < PS2_DATA_LEN; i++)
  {
    ps2_diag.frame[i] = s_data[i];
  }
  ps2_diag.polls++;
  if (valid != 0U) { ps2_diag.good++; } else { ps2_diag.bad++; }
  return valid;
}
