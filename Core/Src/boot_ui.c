/* boot_ui：OLED 初始化 + 纯渲染循环（spec §11.2/§12）。
 *
 * 单一职责：把 Ctrl 任务每 10 ms 发布的 ui_snapshot 画到屏上并 flush。
 * 不读手柄、不推进页面状态、不累计编码器、不写任何电机寄存器——
 * 电机（含写零）的唯一出口是 Ctrl 任务的 ctrl_apply()。
 *
 * 上电初始化也收敛在这里，但只建立"安全静止"的硬件起点，不产生运动：
 *   1. 四路 SLP 拉低（驱动睡眠；.ioc 未配置这些脚，手动初始化）；
 *   2. TIM8 四通道以占空比 0 启动（上电即有 PWM 外设、零波形）；
 *   3. 四路编码器定时器启动（16 位计数从此连续，回绕由 wheel_feedback 处理）；
 *   4. oled_init() 完成后置 boot_ui_ready=1，进入 20 ms 渲染循环。
 *
 * 原开机流程（等待按键/加载动画/UI 独立自检/结果停留页/USER 窗口/移交）
 * 全部移除：自检由 Ctrl 内的 selfcheck_service 推进，页面由 Ctrl 内的
 * ui_model 推进，本任务从第一帧起就只画快照。 */
#include "boot_ui.h"

#include "cmsis_os.h"
#include "main.h"
#include "oled.h"
#include "tim.h"
#include "ui_render.h"

/* 渲染周期 20 ms（spec §11.8：全屏刷新最多 20 Hz；驾驶数据 10 Hz、
 * 电压 2 Hz 由 Ctrl/power 任务的发布节拍决定，与本循环无关）。
 * 若单帧绘制+flush 超过 20 ms，则降速让出一拍，绝不忙等、不阻塞 Ctrl。 */
#define UI_RENDER_PERIOD_MS 20U

volatile uint8_t boot_ui_ready = 0U;

/* 通道顺序与轮位一致：A 右前=CH4、B 左前=CH3、C 右后=CH2、D 左后=CH1。 */
static const uint32_t s_boot_pwm_ch[4] = {
  TIM_CHANNEL_4, TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_1
};

static GPIO_TypeDef *const s_boot_slp_port[4] = {
  MOTOR_RF_SLP_GPIO_Port, MOTOR_LF_SLP_GPIO_Port,
  MOTOR_RR_SLP_GPIO_Port, MOTOR_LR_SLP_GPIO_Port
};

static const uint32_t s_boot_slp_pin[4] = {
  MOTOR_RF_SLP_Pin, MOTOR_LF_SLP_Pin, MOTOR_RR_SLP_Pin, MOTOR_LR_SLP_Pin
};

void boot_ui_run(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t w;

  /* 四轮 SLP/EN：低电平 = 驱动睡眠。电机自始至终归 Ctrl——这里只保证
   * "上电即睡眠"，之后唤醒/写零/写占空比全部由 ctrl_apply 接管。 */
  for (w = 0U; w < 4U; w++)
  {
    HAL_GPIO_WritePin(s_boot_slp_port[w], s_boot_slp_pin[w], GPIO_PIN_RESET);
    gpio.Pin = s_boot_slp_pin[w];
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(s_boot_slp_port[w], &gpio);
  }

  /* 上电即置 MOE，但占空比 0：定时器在跑、输出零波形，电机不动。
   * Ctrl 唤醒路径只需改 CCR，不需要再关心通道是否启动。 */
  for (w = 0U; w < 4U; w++)
  {
    (void)HAL_TIM_PWM_Start(&htim8, s_boot_pwm_ch[w]);
    __HAL_TIM_SET_COMPARE(&htim8, s_boot_pwm_ch[w], 0U);
  }

  /* 四路编码器：只启动计数，不读数（累计在 Ctrl 的 wheel_feedback 里）。 */
  (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  oled_init();
  /* 屏幕就绪后才置位：此后渲染循环提供的页面/数据才可信。 */
  boot_ui_ready = 1U;

  for (;;)
  {
    ui_snapshot_t snap;
    uint32_t frame_start = HAL_GetTick();
    uint32_t elapsed;
    uint32_t rest;

    /* seq-lock 读快照 → 按页绘制 → 整帧 flush。绘制函数只写
     * oled_gfx 帧缓冲，flush 是本任务唯一的屏幕输出。 */
    ui_snapshot_get(&snap);
    ui_render_draw(&snap, frame_start);
    oled_flush();

    /* 保持 ~20 ms 节拍：剩余时间睡眠；单帧超期时至少让出一拍
     * （osDelay 等价于 osDelayUntil 的相对形式，tick=1 ms）。 */
    elapsed = (uint32_t)(HAL_GetTick() - frame_start);
    rest = (elapsed < UI_RENDER_PERIOD_MS) ? (UI_RENDER_PERIOD_MS - elapsed)
                                           : 1U;
    osDelay(rest);
  }
}
