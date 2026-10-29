#include "boot_ui.h"

#include "adc.h"
#include "cmsis_os.h"
#include "main.h"
#include "oled.h"
#include "oled_font.h"
#include "oled_gfx.h"
#include "tim.h"

#define UI_BLINK_MS        500U
#define UI_KEY_POLL_MS      10U
#define UI_LOADING_MS     1000U
#define UI_START_MS        800U
#define UI_SPINNER_MS      125U
#define UI_VDD_RETRY         3U
#define UI_VDD_RETRY_MS    500U
#define UI_MAIN_HOLD_MS    600U

#define UI_ROW_MOTOR_Y      22
#define UI_ROW_VDD_Y        42
#define UI_ROW_SYMBOL_X     78

#define UI_STATE_RUN   0U
#define UI_STATE_OK    1U
#define UI_STATE_FAIL  2U

#define UI_TEXT6_WIDTH     6U
#define UI_TEXT16_STEP    18U

typedef struct
{
  TIM_HandleTypeDef *tim;
  const char *name;
} ui_wheel_t;

static const ui_wheel_t s_wheels[4] =
{
  { &htim2, "RF" },
  { &htim3, "LF" },
  { &htim4, "RR" },
  { &htim5, "LR" },
};

typedef struct
{
  GPIO_TypeDef *slpPort;
  uint32_t slpPin;
  GPIO_TypeDef *dirPort;
  uint32_t dirPin;
  uint32_t dirFwd;
  uint32_t dirRev;
  uint32_t pwmCh;
} ui_motor_t;

static const ui_motor_t s_motors[4] =
{
  { MOTOR_RF_SLP_GPIO_Port, MOTOR_RF_SLP_Pin, MOTOR_RF_DIR_GPIO_Port, MOTOR_RF_DIR_Pin,
    MOTOR_RF_DIR_FORWARD, MOTOR_RF_DIR_REVERSE, TIM_CHANNEL_4 }, /* A 右前 */
  { MOTOR_LF_SLP_GPIO_Port, MOTOR_LF_SLP_Pin, MOTOR_LF_DIR_GPIO_Port, MOTOR_LF_DIR_Pin,
    MOTOR_LF_DIR_FORWARD, MOTOR_LF_DIR_REVERSE, TIM_CHANNEL_3 }, /* B 左前 */
  { MOTOR_RR_SLP_GPIO_Port, MOTOR_RR_SLP_Pin, MOTOR_RR_DIR_GPIO_Port, MOTOR_RR_DIR_Pin,
    MOTOR_RR_DIR_FORWARD, MOTOR_RR_DIR_REVERSE, TIM_CHANNEL_2 }, /* C 右后 */
  { MOTOR_LR_SLP_GPIO_Port, MOTOR_LR_SLP_Pin, MOTOR_LR_DIR_GPIO_Port, MOTOR_LR_DIR_Pin,
    MOTOR_LR_DIR_FORWARD, MOTOR_LR_DIR_REVERSE, TIM_CHANNEL_1 }, /* D 左后 */
};

static void ui_text16_center(int16_t y, const char *text);
static void ui_text6_center(int16_t y, const char *text);
static void ui_checklist(uint8_t motor_state, uint8_t vdd_state, uint8_t frame);
static void ui_hold(uint32_t ms, uint8_t vdd_state, uint8_t *frame);
static void ui_motor_selfcheck(uint8_t *frame);
static uint8_t ui_vdd_check(uint32_t *vdd_mv, uint8_t *frame);
static uint32_t ui_read_vdd_mv(void);
static void ui_main_page(void);
static void ui_format_vdd(char *buf, uint32_t mv);
static void ui_format_rpm(char *buf, const char *name, int32_t rpm);

static void ui_text16_center(int16_t y, const char *text)
{
  uint16_t width = oled_gfx_text16_width(text);
  oled_gfx_text16((int16_t)((OLED_WIDTH - width) / 2U), y, text);
}

static void ui_text6_center(int16_t y, const char *text)
{
  uint16_t width = oled_gfx_text6x12_width(text);
  oled_gfx_text6x12((int16_t)((OLED_WIDTH - width) / 2U), y, text);
}

static void ui_row_symbol(int16_t y, uint8_t state, uint8_t frame)
{
  if (state == UI_STATE_RUN)
  {
    oled_gfx_bitmap(UI_ROW_SYMBOL_X, (int16_t)(y + 2), 12, 12,
                    oled_font_spinner[frame & 7U]);
  }
  else if (state == UI_STATE_OK)
  {
    oled_gfx_bitmap(UI_ROW_SYMBOL_X, y, 16, 16, oled_font_check);
  }
  else
  {
    oled_gfx_bitmap(UI_ROW_SYMBOL_X, y, 16, 16, oled_font_cross);
  }
}

static void ui_checklist(uint8_t motor_state, uint8_t vdd_state, uint8_t frame)
{
  oled_gfx_clear();
  ui_text16_center(0, "自检流程开始");
  oled_gfx_text16(0, UI_ROW_MOTOR_Y, "电机自检");
  oled_gfx_text16(0, UI_ROW_VDD_Y, "电池自检");
  ui_row_symbol(UI_ROW_MOTOR_Y, motor_state, frame);
  ui_row_symbol(UI_ROW_VDD_Y, vdd_state, frame);
  oled_flush();
}

static void ui_hold(uint32_t ms, uint8_t vdd_state, uint8_t *frame)
{
  while (ms > 0U)
  {
    ui_checklist(UI_STATE_RUN, vdd_state, (*frame)++);
    if (ms > UI_SPINNER_MS)
    {
      osDelay(UI_SPINNER_MS);
      ms -= UI_SPINNER_MS;
    }
    else
    {
      osDelay(ms);
      ms = 0U;
    }
  }
}

static void ui_wait_key(void)
{
  uint8_t blink = 1U;
  uint8_t i;

  for (;;)
  {
    oled_gfx_clear();
    ui_text16_center(16, "等待初始化");
    if (blink != 0U)
    {
      ui_text6_center(44, "PRESS KEY");
    }
    oled_flush();
    blink = (uint8_t)!blink;

    for (i = 0; i < (uint8_t)(UI_BLINK_MS / UI_KEY_POLL_MS); i++)
    {
      if (HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin) == GPIO_PIN_RESET)
      {
        osDelay(UI_KEY_POLL_MS);
        if (HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin) == GPIO_PIN_RESET)
        {
          return;
        }
      }
      osDelay(UI_KEY_POLL_MS);
    }
  }
}

static void ui_loading(void)
{
  uint8_t frame = 0U;
  uint32_t elapsed;
  uint16_t width;

  for (elapsed = 0U; elapsed < UI_LOADING_MS; elapsed += UI_SPINNER_MS)
  {
    oled_gfx_clear();
    ui_text16_center(24, "正在加载");
    width = oled_gfx_text16_width("正在加载");
    oled_gfx_bitmap((int16_t)((OLED_WIDTH - width) / 2U + width + 8U), 26,
                    12, 12, oled_font_spinner[frame & 7U]);
    oled_flush();
    frame++;
    osDelay(UI_SPINNER_MS);
  }

  oled_gfx_clear();
  ui_text16_center(24, "自检流程开始");
  oled_flush();
  osDelay(UI_START_MS);
}

/* 四轮顺序自检：A→B→C→D，每轮独立跑 正转3s → 停3s → 反转3s → 停3s ×2，50% 上限 */
static void ui_motor_selfcheck(uint8_t *frame)
{
  uint8_t cycle;
  uint8_t w;

  for (w = 0U; w < 4U; w++)
  {
    const ui_motor_t *m = &s_motors[w];

    HAL_GPIO_WritePin(m->slpPort, m->slpPin, GPIO_PIN_SET);
    osDelay(10U);

    for (cycle = 0U; cycle < SELFCHECK_CYCLES; cycle++)
    {
      HAL_GPIO_WritePin(m->dirPort, m->dirPin, m->dirFwd);
      __HAL_TIM_SET_COMPARE(&htim8, m->pwmCh, SELFCHECK_PWM_PULSE);
      ui_hold(SELFCHECK_RUN_MS, UI_STATE_RUN, frame);

      __HAL_TIM_SET_COMPARE(&htim8, m->pwmCh, 0U);
      ui_hold(SELFCHECK_PAUSE_MS, UI_STATE_RUN, frame);

      HAL_GPIO_WritePin(m->dirPort, m->dirPin, m->dirRev);
      __HAL_TIM_SET_COMPARE(&htim8, m->pwmCh, SELFCHECK_PWM_PULSE);
      ui_hold(SELFCHECK_RUN_MS, UI_STATE_RUN, frame);

      __HAL_TIM_SET_COMPARE(&htim8, m->pwmCh, 0U);
      ui_hold(SELFCHECK_PAUSE_MS, UI_STATE_RUN, frame);
    }

    /* 本轮收尾：清占空比 → 停 PWM（清 MOE）→ 驱动器回睡眠 */
    __HAL_TIM_SET_COMPARE(&htim8, m->pwmCh, 0U);
    HAL_TIM_PWM_Stop(&htim8, m->pwmCh);
    HAL_GPIO_WritePin(m->slpPort, m->slpPin, GPIO_PIN_RESET);
  }
}

static uint32_t ui_read_vdd_mv(void)
{
  static uint16_t s_cal = 0U;
  ADC_ChannelConfTypeDef cfg = {0};
  uint32_t sum = 0U;
  uint8_t i;

  if (s_cal == 0U)
  {
    s_cal = *VREFINT_CAL_ADDR;
  }

  cfg.Channel = ADC_CHANNEL_VREFINT;
  cfg.Rank = 1U;
  cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK)
  {
    return 0U;
  }

  for (i = 0U; i < 8U; i++)
  {
    if (HAL_ADC_Start(&hadc1) == HAL_OK)
    {
      if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK)
      {
        sum += HAL_ADC_GetValue(&hadc1);
      }
      HAL_ADC_Stop(&hadc1);
    }
  }

  /* 恢复电池分压通道，保持 ADC 状态与初始化后一致 */
  cfg.Channel = ADC_CHANNEL_8;
  (void)HAL_ADC_ConfigChannel(&hadc1, &cfg);

  return boot_ui_vdd_mv(sum / 8U, s_cal);
}

static uint8_t ui_vdd_check(uint32_t *vdd_mv, uint8_t *frame)
{
  uint8_t attempt;
  uint32_t value = 0U;

  for (attempt = 0U; attempt < UI_VDD_RETRY; attempt++)
  {
    ui_checklist(UI_STATE_OK, UI_STATE_RUN, (*frame)++);
    value = ui_read_vdd_mv();
    if (value >= BOOT_UI_VDD_MIN_MV)
    {
      *vdd_mv = value;
      return 1U;
    }
    osDelay(UI_VDD_RETRY_MS);
  }
  *vdd_mv = value;
  return 0U;
}

static void ui_format_vdd(char *buf, uint32_t mv)
{
  uint32_t cv = mv / 10U;

  if (cv > 9999U)
  {
    cv = 9999U;
  }
  buf[0] = 'V';
  buf[1] = 'D';
  buf[2] = 'D';
  buf[3] = ' ';
  buf[4] = (char)('0' + (cv / 100U) % 10U);
  buf[5] = '.';
  buf[6] = (char)('0' + (cv / 10U) % 10U);
  buf[7] = (char)('0' + cv % 10U);
  buf[8] = 'V';
  buf[9] = '\0';
}

static void ui_format_rpm(char *buf, const char *name, int32_t rpm)
{
  uint32_t mag = (rpm < 0) ? (uint32_t)(-rpm) : (uint32_t)rpm;

  if (mag > 9999U)
  {
    mag = 9999U;
  }
  buf[0] = name[0];
  buf[1] = name[1];
  buf[2] = ' ';
  buf[3] = (rpm < 0) ? '-' : '+';
  buf[4] = (char)('0' + (mag / 1000U) % 10U);
  buf[5] = (char)('0' + (mag / 100U) % 10U);
  buf[6] = (char)('0' + (mag / 10U) % 10U);
  buf[7] = (char)('0' + mag % 10U);
  buf[8] = ' ';
  buf[9] = 'r';
  buf[10] = 'p';
  buf[11] = 'm';
  buf[12] = '\0';
}

static void ui_main_page(void)
{
  uint16_t prev[4];
  int32_t sum[4];
  char line[20];
  uint32_t vdd_mv = 0U;
  uint32_t since_vdd = BOOT_UI_VDD_REFRESH_MS;
  uint8_t w;
  uint8_t s;

  for (w = 0U; w < 4U; w++)
  {
    prev[w] = (uint16_t)__HAL_TIM_GET_COUNTER(s_wheels[w].tim);
    sum[w] = 0;
  }

  for (;;)
  {
    for (w = 0U; w < 4U; w++)
    {
      sum[w] = 0;
    }

    for (s = 0U; s < BOOT_UI_ENC_SAMPLES; s++)
    {
      osDelay(BOOT_UI_ENC_SAMPLE_MS);
      for (w = 0U; w < 4U; w++)
      {
        uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(s_wheels[w].tim);
        sum[w] += (int32_t)(int16_t)(uint16_t)(now - prev[w]);
        prev[w] = now;
      }
    }

    if (since_vdd >= BOOT_UI_VDD_REFRESH_MS)
    {
      vdd_mv = ui_read_vdd_mv();
      since_vdd = 0U;
    }
    since_vdd += (uint32_t)(BOOT_UI_ENC_SAMPLE_MS * BOOT_UI_ENC_SAMPLES);

    oled_gfx_clear();
    ui_format_vdd(line, vdd_mv);
    oled_gfx_text6x12(0, 0, line);

    for (w = 0U; w < 4U; w++)
    {
      ui_format_rpm(line, s_wheels[w].name,
                    boot_ui_rpm_from_counts(sum[w], WHEEL_CPR));
      oled_gfx_text6x12(0, (int16_t)(12 * (w + 1U)), line);
    }
    oled_flush();
  }
}

/* 临时：面板点灯探测。阶段 1 = SSD1306 路径，阶段 2 = SH1106 路径，定位后删除 */
static void ui_panel_probe(void)
{
  uint8_t *fb = (uint8_t *)oled_gfx_buffer();
  uint32_t i;

  for (i = 0U; i < OLED_WIDTH * OLED_HEIGHT / 8U; i++)
  {
    fb[i] = 0xFFU;
  }
  oled_flush();
  osDelay(3000U);
  oled_gfx_clear();
  oled_flush();
  osDelay(800U);

  oled_sh1106_enable();
  for (i = 0U; i < OLED_WIDTH * OLED_HEIGHT / 8U; i++)
  {
    fb[i] = 0xFFU;
  }
  oled_flush();
  osDelay(3000U);
  oled_gfx_clear();
  oled_flush();
}

void boot_ui_run(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t frame = 0U;
  uint32_t vdd_mv = 0U;
  uint8_t w;

  /* 四轮 SLP/EN：.ioc 未配置，手动初始化，低电平 = 驱动睡眠 */
  for (w = 0U; w < 4U; w++)
  {
    HAL_GPIO_WritePin(s_motors[w].slpPort, s_motors[w].slpPin, GPIO_PIN_RESET);
    gpio.Pin = s_motors[w].slpPin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(s_motors[w].slpPort, &gpio);
  }

  /* 上电即置 MOE，但占空比 0，电机不动 */
  for (w = 0U; w < 4U; w++)
  {
    HAL_TIM_PWM_Start(&htim8, s_motors[w].pwmCh);
    __HAL_TIM_SET_COMPARE(&htim8, s_motors[w].pwmCh, 0U);
  }

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  oled_init();
  ui_panel_probe();

  ui_wait_key();
  ui_loading();

  ui_checklist(UI_STATE_RUN, UI_STATE_RUN, frame++);
  ui_motor_selfcheck(&frame);
  ui_checklist(UI_STATE_OK, UI_STATE_RUN, frame++);

  if (ui_vdd_check(&vdd_mv, &frame) == 0U)
  {
    ui_checklist(UI_STATE_OK, UI_STATE_FAIL, frame);
    for (;;)
    {
      osDelay(1000U);
    }
  }

  ui_checklist(UI_STATE_OK, UI_STATE_OK, frame);
  osDelay(UI_MAIN_HOLD_MS);
  ui_main_page();
}
