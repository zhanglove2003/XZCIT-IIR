/* 四路麦轮电机驱动、编码器读取与 ROS 2 上位机串口通信 (USART1, 115200 8N1) */
#include "motor_drive.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

/* ======================== 电机驱动 ======================== */

/* DIR 引脚输出为该电平时电机正转。
 * 需对照 MD01 手册确认; 若方向相反, 改为 GPIO_PIN_RESET, 或在 ROS 端设置 motor_sign。 */
#define MOTOR_FORWARD_LEVEL GPIO_PIN_SET

typedef struct
{
  TIM_HandleTypeDef *enc_htim;    /* 编码器定时器 */
  uint32_t           pwm_channel; /* TIM8 PWM 通道 */
  GPIO_TypeDef      *dir_port;
  uint16_t           dir_pin;
  GPIO_TypeDef      *slp_port;    /* 驱动使能(高电平工作, 低电平睡眠) */
  uint16_t           slp_pin;
} wheel_hw_t;

static const wheel_hw_t wheel_hw[MOTOR_WHEEL_COUNT] = {
  /* RF 右前: PC9 / TIM8_CH4, 编码器 TIM2 */
  { &htim2, TIM_CHANNEL_4, MOTOR_RF_DIR_GPIO_Port, MOTOR_RF_DIR_Pin, MOTOR_RF_SLP_GPIO_Port, MOTOR_RF_SLP_Pin },
  /* LF 左前: PC8 / TIM8_CH3, 编码器 TIM3 */
  { &htim3, TIM_CHANNEL_3, MOTOR_LF_DIR_GPIO_Port, MOTOR_LF_DIR_Pin, MOTOR_LF_SLP_GPIO_Port, MOTOR_LF_SLP_Pin },
  /* RR 右后: PC7 / TIM8_CH2, 编码器 TIM4 */
  { &htim4, TIM_CHANNEL_2, MOTOR_RR_DIR_GPIO_Port, MOTOR_RR_DIR_Pin, MOTOR_RR_SLP_GPIO_Port, MOTOR_RR_SLP_Pin },
  /* LR 左后: PC6 / TIM8_CH1, 编码器 TIM5 */
  { &htim5, TIM_CHANNEL_1, MOTOR_LR_DIR_GPIO_Port, MOTOR_LR_DIR_Pin, MOTOR_LR_SLP_GPIO_Port, MOTOR_LR_SLP_Pin },
};

/* 上一次读取时的计数值, 用于计算增量 */
static uint16_t enc_last[MOTOR_WHEEL_COUNT];

void MotorDrive_Init(void)
{
  uint8_t i;

  for (i = 0U; i < MOTOR_WHEEL_COUNT; i++)
  {
    HAL_TIM_PWM_Start(&htim8, wheel_hw[i].pwm_channel);
    HAL_TIM_Encoder_Start(wheel_hw[i].enc_htim, TIM_CHANNEL_ALL);
    enc_last[i] = (uint16_t)__HAL_TIM_GET_COUNTER(wheel_hw[i].enc_htim);
    MotorDrive_SetDuty(i, 0);
  }

  /* TIM8 是高级定时器, 主输出使能位 MOE 需要手动置位 */
  __HAL_TIM_MOE_ENABLE(&htim8);

  MotorDrive_SetEnable(0U);
}

/* 置位或清除所有驱动的 SLP 引脚。SLP 为低时驱动睡眠, 电机没有输出。 */
void MotorDrive_SetEnable(uint8_t enable)
{
  uint8_t i;
  GPIO_PinState level = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;

  for (i = 0U; i < MOTOR_WHEEL_COUNT; i++)
  {
    HAL_GPIO_WritePin(wheel_hw[i].slp_port, wheel_hw[i].slp_pin, level);
  }
}

/* duty 单位为 ‰, 负值表示反转 */
void MotorDrive_SetDuty(uint8_t wheel, int16_t duty)
{
  uint32_t pulse;
  GPIO_PinState dir;

  if (wheel >= MOTOR_WHEEL_COUNT)
  {
    return;
  }

  if (duty > MOTOR_DUTY_MAX)
  {
    duty = MOTOR_DUTY_MAX;
  }
  if (duty < -MOTOR_DUTY_MAX)
  {
    duty = -MOTOR_DUTY_MAX;
  }

  /* ARR = 16799, 占空比 1000 对应比较值 16800(即 100%) */
  pulse = (uint32_t)(duty < 0 ? -duty : duty) * (htim8.Init.Period + 1U) / MOTOR_DUTY_MAX;

  if (duty >= 0)
  {
    dir = MOTOR_FORWARD_LEVEL;
  }
  else
  {
    dir = (MOTOR_FORWARD_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(wheel_hw[wheel].dir_port, wheel_hw[wheel].dir_pin, dir);
  __HAL_TIM_SET_COMPARE(&htim8, wheel_hw[wheel].pwm_channel, pulse);
}

/* 返回自上次调用以来的编码器计数变化量。
 * 计数器为 16 位(Period 65535), 用 uint16 相减后再转 int16, 可正确处理回绕。 */
int16_t MotorDrive_ReadEncoderDelta(uint8_t wheel)
{
  uint16_t now;
  int16_t delta;

  if (wheel >= MOTOR_WHEEL_COUNT)
  {
    return 0;
  }

  now = (uint16_t)__HAL_TIM_GET_COUNTER(wheel_hw[wheel].enc_htim);
  delta = (int16_t)(uint16_t)(now - enc_last[wheel]);
  enc_last[wheel] = now;
  return delta;
}

/* ======================== ROS 2 串口通信 ======================== */

/* 帧格式: [0xA5][0x5A][TYPE][LEN][PAYLOAD][CRC8], CRC8 多项式 0x07, 初值 0, 覆盖 TYPE..PAYLOAD */
#define RL_HDR1           0xA5U
#define RL_HDR2           0x5AU
#define RL_TYPE_CMD       0x01U /* 上位机 -> MCU: 四轮占空比 int16[4] */
#define RL_TYPE_FB        0x81U /* MCU -> 上位机: uint16 dt_ms + int16[4] 编码器增量 */
#define RL_CMD_PAYLOAD    8U
#define RL_FB_PAYLOAD     10U
#define RL_MAX_PAYLOAD    16U
#define RL_PERIOD_MS      20U   /* 控制与反馈周期, 50 Hz */
#define RL_CMD_TIMEOUT_MS 500U  /* 超过该时间没有有效指令则停车并断电 */

typedef enum
{
  ST_H1,
  ST_H2,
  ST_TYPE,
  ST_LEN,
  ST_PAYLOAD,
  ST_CRC
} rx_state_t;

/* 接收状态, 仅在串口中断中访问 */
static uint8_t rx_byte;
static rx_state_t rx_state = ST_H1;
static uint8_t rx_type;
static uint8_t rx_len;
static uint8_t rx_idx;
static uint8_t rx_crc;
static uint8_t rx_payload[RL_MAX_PAYLOAD];

/* 指令数据: 串口中断写入, 任务在临界区内读取 */
static volatile int16_t cmd_duty[MOTOR_WHEEL_COUNT];
static volatile uint32_t cmd_tick = 0U;
static volatile uint8_t cmd_valid = 0U;

static uint8_t crc8_update(uint8_t crc, uint8_t data)
{
  uint8_t i;

  crc ^= data;
  for (i = 0U; i < 8U; i++)
  {
    crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
  }
  return crc;
}

static void handle_frame(void)
{
  uint8_t i;
  int16_t duty;

  if ((rx_type != RL_TYPE_CMD) || (rx_len != RL_CMD_PAYLOAD))
  {
    return;
  }

  for (i = 0U; i < MOTOR_WHEEL_COUNT; i++)
  {
    duty = (int16_t)((uint16_t)rx_payload[2U * i] | ((uint16_t)rx_payload[2U * i + 1U] << 8));
    if (duty > MOTOR_DUTY_MAX)
    {
      duty = MOTOR_DUTY_MAX;
    }
    if (duty < -MOTOR_DUTY_MAX)
    {
      duty = -MOTOR_DUTY_MAX;
    }
    cmd_duty[i] = duty;
  }

  cmd_tick = HAL_GetTick();
  cmd_valid = 1U;
}

/* 逐字节状态机。遇到错误时回到 ST_H1, 从下一个字节重新寻找帧头。 */
static void rx_feed(uint8_t b)
{
  switch (rx_state)
  {
  case ST_H1:
    if (b == RL_HDR1)
    {
      rx_state = ST_H2;
    }
    break;

  case ST_H2:
    if (b == RL_HDR2)
    {
      rx_state = ST_TYPE;
    }
    else if (b != RL_HDR1)
    {
      rx_state = ST_H1;
    }
    break;

  case ST_TYPE:
    rx_type = b;
    rx_crc = crc8_update(0U, b);
    rx_state = ST_LEN;
    break;

  case ST_LEN:
    if (b > RL_MAX_PAYLOAD)
    {
      rx_state = ST_H1;
      break;
    }
    rx_len = b;
    rx_crc = crc8_update(rx_crc, b);
    rx_idx = 0U;
    rx_state = (b == 0U) ? ST_CRC : ST_PAYLOAD;
    break;

  case ST_PAYLOAD:
    rx_payload[rx_idx++] = b;
    rx_crc = crc8_update(rx_crc, b);
    if (rx_idx >= rx_len)
    {
      rx_state = ST_CRC;
    }
    break;

  case ST_CRC:
    if (b == rx_crc)
    {
      handle_frame();
    }
    rx_state = ST_H1;
    break;

  default:
    rx_state = ST_H1;
    break;
  }
}

static void send_feedback(uint16_t dt_ms, const int16_t *delta)
{
  uint8_t frame[4U + RL_FB_PAYLOAD + 1U];
  uint8_t *p = &frame[4];
  uint8_t i;
  uint8_t crc = 0U;
  uint16_t v;

  frame[0] = RL_HDR1;
  frame[1] = RL_HDR2;
  frame[2] = RL_TYPE_FB;
  frame[3] = RL_FB_PAYLOAD;

  p[0] = (uint8_t)(dt_ms & 0xFFU);
  p[1] = (uint8_t)(dt_ms >> 8);
  for (i = 0U; i < MOTOR_WHEEL_COUNT; i++)
  {
    v = (uint16_t)delta[i];
    p[2U + 2U * i] = (uint8_t)(v & 0xFFU);
    p[3U + 2U * i] = (uint8_t)(v >> 8);
  }

  for (i = 2U; i < 4U + RL_FB_PAYLOAD; i++)
  {
    crc = crc8_update(crc, frame[i]);
  }
  frame[4U + RL_FB_PAYLOAD] = crc;

  HAL_UART_Transmit(&huart1, frame, sizeof(frame), 10U);
}

void RosLink_Init(void)
{
  /* 优先级 6 低于 configMAX_SYSCALL_INTERRUPT_PRIORITY(5),
   * 这样临界区(taskENTER_CRITICAL)能屏蔽该中断, 保证任务读到的指令数据完整。 */
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

/* 每收到一个字节进入一次回调, 并重新挂起下一字节的接收 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    rx_feed(rx_byte);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
  }
}

/* 出错(例如溢出)时复位解析器并重新开始接收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    rx_state = ST_H1;
    HAL_UART_AbortReceive_IT(huart);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
  }
}

void RosLink_Task(void *argument)
{
  uint8_t i;
  int16_t duty[MOTOR_WHEEL_COUNT];
  int16_t delta[MOTOR_WHEEL_COUNT];
  uint32_t tick_copy;
  uint8_t valid_copy;
  uint32_t now;
  uint32_t last_ms;
  uint16_t dt_ms;

  (void)argument;
  last_ms = HAL_GetTick();

  for (;;)
  {
    osDelay(RL_PERIOD_MS);

    now = HAL_GetTick();
    dt_ms = (uint16_t)(now - last_ms);
    last_ms = now;

    /* 临界区内复制指令, 防止读到串口中断写到一半的数据 */
    taskENTER_CRITICAL();
    for (i = 0U; i < MOTOR_WHEEL_COUNT; i++)
    {
      duty[i] = cmd_duty[i];
    }
    tick_copy = cmd_tick;
    valid_copy = cmd_valid;
    taskEXIT_CRITICAL();

    if ((valid_copy != 0U) && ((now - tick_copy) < RL_CMD_TIMEOUT_MS))
    {
      MotorDrive_SetEnable(1U);
      for (i = 0U; i < MOTOR_WHEEL_COUNT; i++)
      {
        MotorDrive_SetDuty(i, duty[i]);
      }
    }
    else
    {
      /* 超时: 先清零占空比, 再拉低 SLP 让驱动睡眠 */
      for (i = 0U; i < MOTOR_WHEEL_COUNT; i++)
      {
        MotorDrive_SetDuty(i, 0);
      }
      MotorDrive_SetEnable(0U);
    }

    for (i = 0U; i < MOTOR_WHEEL_COUNT; i++)
    {
      delta[i] = MotorDrive_ReadEncoderDelta(i);
    }

    send_feedback(dt_ms, delta);
  }
}
