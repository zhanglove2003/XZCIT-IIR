/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "boot_ui.h"
#include "drive_ctrl.h"
#include "power_monitor.h"
#include "ps2_gamepad.h"
#include "selfcheck_service.h"
#include "tim.h"
#include "ui_model.h"
#include "ui_render.h"
#include "wheel_feedback.h"
#include "wheel_home.h"
#include "motor_health.h"
#include "buzzer.h"
#include "flog.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t selfCheckTaskHandle;
const osThreadAttr_t selfCheckTask_attributes = {
  .name = "selfCheckTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t powerTaskHandle;
const osThreadAttr_t powerTask_attributes = {
  .name = "powerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
osThreadId_t ctrlTaskHandle;
const osThreadAttr_t ctrlTask_attributes = {
  .name = "ctrlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartSelfCheckTask(void *argument);
void StartCtrlTask(void *argument);
void StartPowerTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* 电源监测状态先归零：powerTask 与 Ctrl/UI 都读这份快照，
   * 必须在两个任务启动之前初始化完毕。 */
  power_monitor_init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  powerTaskHandle = osThreadNew(StartPowerTask, NULL, &powerTask_attributes);
  selfCheckTaskHandle = osThreadNew(StartSelfCheckTask, NULL, &selfCheckTask_attributes);
  ctrlTaskHandle = osThreadNew(StartCtrlTask, NULL, &ctrlTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 开机自检引导 UI：等待按键 → 加载动画 → 电机/电压自检 → 主页面 */
void StartSelfCheckTask(void *argument)
{
  (void)argument;
  boot_ui_run();
}

/* 电源监测：ADC 采样收敛在 power_monitor 内部，这里只给 50 ms 节拍。
 * 低于 Ctrl 的优先级：采样偶尔迟到几拍不影响任何安全判定。 */
void StartPowerTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    power_monitor_step(HAL_GetTick());
    osDelay(50);
  }
}

/* PS2 手柄 + 分层菜单 → 四轮驱动。电机自始至终归 Ctrl（单归属，spec §11.2）：
 * boot_ui.c 只是渲染层，首帧起 ctrl_apply 就只有零命令 → 驱动睡眠，
 * 天然满足"开机不自动转轮"。归位状态机（wheel_home）只吃输入、不碰硬件；
 * 唯一输出路径是 ctrl_apply()。 */
typedef struct
{
  GPIO_TypeDef *slpPort;
  uint32_t slpPin;
  GPIO_TypeDef *dirPort;
  uint32_t dirPin;
  uint32_t dirFwd;
  uint32_t dirRev;
  uint32_t pwmCh;
} ctrl_motor_t;

static const ctrl_motor_t s_ctrl_motors[4] =
{
  { MOTOR_RF_SLP_GPIO_Port, MOTOR_RF_SLP_Pin, MOTOR_RF_DIR_GPIO_Port, MOTOR_RF_DIR_Pin,
    MOTOR_RF_DIR_FORWARD, MOTOR_RF_DIR_REVERSE, TIM_CHANNEL_4 }, /* A 右前 */
  { MOTOR_LF_SLP_GPIO_Port, MOTOR_LF_SLP_Pin, MOTOR_LF_DIR_GPIO_Port, MOTOR_LF_DIR_Pin,
    MOTOR_LF_DIR_FORWARD, MOTOR_LF_DIR_REVERSE, TIM_CHANNEL_3 }, /* B 左前 */
  { MOTOR_RR_SLP_GPIO_Port, MOTOR_RR_SLP_Pin, MOTOR_RR_DIR_GPIO_Port, MOTOR_RR_DIR_Pin,
    MOTOR_RR_DIR_FORWARD, MOTOR_RR_DIR_REVERSE, TIM_CHANNEL_2 }, /* C 右后（板上接口 D 丝印） */
  { MOTOR_LR_SLP_GPIO_Port, MOTOR_LR_SLP_Pin, MOTOR_LR_DIR_GPIO_Port, MOTOR_LR_DIR_Pin,
    MOTOR_LR_DIR_FORWARD, MOTOR_LR_DIR_REVERSE, TIM_CHANNEL_1 }, /* D 左后（板上接口 C 丝印） */
};

/* 编码器表：轮位顺序与 s_ctrl_motors 一致（A/B/C/D）。
 * 后轮按实物接线：C 读 TIM5、D 读 TIM4；乘前进符号得到“前进为正”的计数。 */
typedef struct
{
  TIM_HandleTypeDef *tim;
  int32_t forwardSign;
  uint32_t cpr;
} ctrl_encoder_t;

static const ctrl_encoder_t s_ctrl_encoders[4] =
{
  { &htim2, WHEEL_RF_FORWARD_SIGN, WHEEL_RF_CPR }, /* A 右前 */
  { &htim3, WHEEL_LF_FORWARD_SIGN, WHEEL_LF_CPR }, /* B 左前 */
  { &htim5, WHEEL_RR_FORWARD_SIGN, WHEEL_RR_CPR }, /* C 右后 */
  { &htim4, WHEEL_LR_FORWARD_SIGN, WHEEL_LR_CPR }, /* D 左后 */
};

#define CTRL_POLL_MS        10U
#define CTRL_LINK_LOSS_MS   200U  /* Elapsed time, independent of poll rate. */
#define CTRL_WAKE_DELAY_MS  10U
/* 手柄重连/冷启动后的再武装：摇杆必须连续回中且 SELECT 处于释放，
 * 持续满 CTRL_REARM_MS 才重新允许输出。
 * 用时间而不是帧数：帧数取决于轮询节奏，10 ms/帧 × 3 帧只有 30 ms，
 * 挡不住"按住 SELECT 时断连再重连"把旧按键带进来的情况。 */
#define CTRL_REARM_MS       100U
#define CTRL_STICK_CENTER   128U

/* Ctrl 自己持有的反馈状态；归位模块不读定时器。 */
static wheel_feedback_t s_ctrl_fb[4];
static wheel_home_t s_ctrl_home;
/* 归位状态快照：Ctrl 任务单写、UI 任务只读。
 * 用 seq-lock 而不是"写时关中断"：configUSE_PREEMPTION=1 时
 * taskENTER_CRITICAL() 只屏蔽 configMAX_SYSCALL_INTERRUPT_PRIORITY 及以上
 * 的中断，并不阻止任务切换。Ctrl 优先级高于 UI，即便写在临界区里，
 * 一次跨越结构体的多次字拷贝仍可能被中断/抢占切成两半，读者会拿到
 * "前半新、后半旧"的混搭快照（例如 mode 已 MANUAL 而 pulse 还是上一次的）。
 * 写者：seq++ (奇数, 表示写入中) -> 拷贝 -> seq++ (偶数, 表示稳定)。
 * 读者：读 seq，若不是偶数或读后 seq 变了就重读。 */
static volatile wheel_home_output_t s_ctrl_home_status;
static volatile uint32_t s_ctrl_home_seq;

static void ctrl_home_publish(const wheel_home_output_t *out)
{
  s_ctrl_home_seq = s_ctrl_home_seq + 1U; /* 变奇数：写入中 */
  s_ctrl_home_status = *out;
  s_ctrl_home_seq = s_ctrl_home_seq + 1U; /* 变偶数：稳定 */
}

/* 页面模型与 UI 快照：Ctrl 每 10 ms 推进 ui_model 并发布一份快照，
 * 渲染层（Task 5）只读。发布/读取模式与 s_ctrl_home_status 相同：seq-lock，
 * 原因见上——写时关中断挡不住任务切换，跨结构体拷贝可能被切成两半。 */
static ui_model_t s_ui_model;
static volatile ui_snapshot_t s_ui_snap;
static volatile uint32_t s_ui_snap_seq;

static void ctrl_ui_publish(const ui_snapshot_t *snap)
{
  s_ui_snap_seq = s_ui_snap_seq + 1U; /* 变奇数：写入中 */
  s_ui_snap = *snap;
  s_ui_snap_seq = s_ui_snap_seq + 1U; /* 变偶数：稳定 */
}

/* power_monitor_get() 返回的是 powerTask 正在写的内部指针：
 * 跨任务读必须用临界区做一次性结构体拷贝防撕裂。快照只有十来个字，
 * 关调度拷贝的代价远低于引入一套 seq-lock 的复杂度。 */
static void ctrl_power_read(power_snapshot_t *out)
{
  taskENTER_CRITICAL();
  *out = *power_monitor_get();
  taskEXIT_CRITICAL();
}

/* 驾驶许可的回中武装：进入驾驶页重新起算，摇杆出杆就重置计时，
 * 连续回中满 CTRL_REARM_MS 才武装；武装后出杆即驾驶（否则推杆本身
 * 永远打断计时，车没法开），离开驾驶页立即解除。 */
static uint8_t s_drive_center_armed;
static uint32_t s_drive_center_start;
/* 100 ms 转速采样窗：累计每轮编码器增量（前进为正），窗口闭合换算 RPM。
 * 窗口之间快照沿用上一次的 RPM 值；首个窗口闭合前 sample_valid=0。 */
static int32_t s_win_counts[4];
static int32_t s_win_rpm[4];
static uint32_t s_win_start;
static uint8_t s_win_started;
static uint8_t s_win_valid;
/* 运行期运动反馈监测：异常锁存后清零输出，只有“重新自检且四轮通过”才清除。 */
static motor_health_t s_ctrl_health;
static uint8_t s_ctrl_health_ready;  /* 0 = 尚未初始化 */
static volatile uint8_t s_ctrl_fault_mask;
/* 日志：只在状态变化时打一行，避免每 10 ms 刷屏占满串口。
 * 定时的心跳行每 FLOG_STATUS_MS 打一次，用来确认板子还活着。 */
#define FLOG_STATUS_MS 300U
#ifdef FLOG_ENABLE
static uint32_t s_flog_next_ms;
static uint8_t  s_flog_prev_fault;
static uint8_t  s_flog_prev_mode;
static uint8_t  s_flog_prev_linked;
/* 日志快照：本轮真正下发给电机的 pwm 与同轮编码器增量。
 * 分开存是因为 command[] 在 4b 段会被故障降级清零，
 * 而我们要看的是"判定那一刻"的值，不是清零后的值。 */
static int32_t s_flog_pwm[4];
static int32_t s_flog_delta[4];
#endif
static motor_health_code_t s_ctrl_health_code[4];
static int32_t s_ctrl_last_pwm[4];   /* 上一轮施加的 PWM，用于换向判定 */
static uint8_t s_ctrl_tracker_ready;

/* 菜单发起的自检服务：Ctrl 每轮推进，RUNNING 时独占 command。
 * seq 变化 = 一轮完整跑完（通过与否都算，标定模式不加）；
 * 每次进入 DONE（含取消）开 5 s 标定入口窗口，用真实毫秒计时。 */
static selfcheck_service_t s_ctrl_check;
static uint32_t s_ctrl_check_seq;           /* 已消费的服务轮次 */
static selfcheck_phase_t s_ctrl_check_prev; /* 上一拍 phase（DONE 沿检测） */
static uint32_t s_ctrl_check_done_ms;       /* 最近一次进入 DONE 的时刻 */
static uint8_t s_ctrl_action_busy;          /* 上一拍自检/归位是否在跑 */
static uint8_t s_ctrl_home_busy;            /* 上一拍归位状态机是否 RUNNING */
#define SELFCHECK_RESULT_WINDOW_MS 5000U

void wheel_home_get_status(wheel_home_output_t *out)
{
  uint32_t tries = 0U;
  uint32_t before;

  if (out == 0)
  {
    return;
  }

  /* seq-lock 读：重试有上限，避免写者持续更新时把 UI 卡死。
   * 超过上限就返回最近一次拷贝——UI 状态栏一帧的短暂不一致可以接受，
   * 但绝不能在这里无限自旋。 */
  for (tries = 0U; tries < 4U; tries++)
  {
    before = s_ctrl_home_seq;
    if ((before & 1U) != 0U)
    {
      continue; /* 正在写入，重试 */
    }
    *out = s_ctrl_home_status;
    if (s_ctrl_home_seq == before)
    {
      return; /* 读期间没被改写：一致 */
    }
  }
  /* 退回到带临界区的一次拷贝作为兜底。 */
  taskENTER_CRITICAL();
  *out = s_ctrl_home_status;
  taskEXIT_CRITICAL();
}

void ui_snapshot_get(ui_snapshot_t *out)
{
  uint32_t tries = 0U;
  uint32_t before;

  if (out == 0)
  {
    return;
  }

  /* seq-lock 读：与 wheel_home_get_status 同一套口径（4 次重试 +
   * 临界区兜底），避免写者持续更新时把渲染层卡死。 */
  for (tries = 0U; tries < 4U; tries++)
  {
    before = s_ui_snap_seq;
    if ((before & 1U) != 0U)
    {
      continue; /* 正在写入，重试 */
    }
    *out = s_ui_snap;
    if (s_ui_snap_seq == before)
    {
      return; /* 读期间没被改写：一致 */
    }
  }
  taskENTER_CRITICAL();
  *out = s_ui_snap;
  taskEXIT_CRITICAL();
}

static void ctrl_motors_sleep(void)
{
  uint8_t w;

  for (w = 0U; w < 4U; w++)
  {
    __HAL_TIM_SET_COMPARE(&htim8, s_ctrl_motors[w].pwmCh, 0U);
    HAL_GPIO_WritePin(s_ctrl_motors[w].slpPort, s_ctrl_motors[w].slpPin, GPIO_PIN_RESET);
  }
}

/* 唯一电机输出路径：手动、按键、归位、等待都从这里落地。
 * 唤醒时先以零占空比上电，等驱动就绪后再施加非零 PWM。 */
static void ctrl_apply(const int32_t command[4], uint8_t *awake)
{
  uint8_t w;
  uint8_t any = 0U;

  for (w = 0U; w < 4U; w++)
  {
    if (command[w] != 0)
    {
      any = 1U;
    }
  }

  /* 归位完成/部分故障停止/取消：命令全零且不由摇杆持有，
   * 就按停止流程关掉驱动使能，而不是只把 PWM 写零。 */
  if (any == 0U && *awake != 0U)
  {
    ctrl_motors_sleep();
    *awake = 0U;
    return;
  }

  if (any != 0U && *awake == 0U)
  {
    for (w = 0U; w < 4U; w++)
    {
      HAL_GPIO_WritePin(s_ctrl_motors[w].slpPort,
                        s_ctrl_motors[w].slpPin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim8, s_ctrl_motors[w].pwmCh, 0U);
      /* 必须在这里启动通道：UI 自检收尾时 HAL_TIM_PWM_Stop 关了四路，
       * 而 ctrl_link_service 的 PWM_Start 只在冷启动/重连跑一次，等电机
       * 移交过来时早已错过。只写 CCR 不启动通道 = 一点波形都没有，
       * 表现为"推摇杆完全没反应"。 */
      (void)HAL_TIM_PWM_Start(&htim8, s_ctrl_motors[w].pwmCh);
    }
    osDelay(CTRL_WAKE_DELAY_MS);
    *awake = 1U;
  }

  for (w = 0U; w < 4U; w++)
  {
    const ctrl_motor_t *m = &s_ctrl_motors[w];

    if (command[w] > 0)
    {
      HAL_GPIO_WritePin(m->dirPort, m->dirPin, m->dirFwd);
      __HAL_TIM_SET_COMPARE(&htim8, m->pwmCh, (uint32_t)command[w]);
    }
    else if (command[w] < 0)
    {
      HAL_GPIO_WritePin(m->dirPort, m->dirPin, m->dirRev);
      __HAL_TIM_SET_COMPARE(&htim8, m->pwmCh, (uint32_t)(-command[w]));
    }
    else
    {
      __HAL_TIM_SET_COMPARE(&htim8, m->pwmCh, 0U);
    }
  }
}

/* 采样四路编码器；所有模式都要做，漏采由状态机的 50 ms 判失效。 */
/* 四轴是否回中：直接给原始 0..255 值，手柄链路与状态机共用同一判据。 */
static uint8_t ctrl_sticks_centered_raw(uint8_t lx, uint8_t ly, uint8_t rx,
                                        uint8_t ry)
{
  int32_t dx[4];
  uint8_t i;
  uint8_t ok = 1U;

  dx[0] = (int32_t)lx - (int32_t)WHEEL_HOME_STICK_CENTER;
  dx[1] = (int32_t)ly - (int32_t)WHEEL_HOME_STICK_CENTER;
  dx[2] = (int32_t)rx - (int32_t)WHEEL_HOME_STICK_CENTER;
  dx[3] = (int32_t)ry - (int32_t)WHEEL_HOME_STICK_CENTER;
  for (i = 0U; i < 4U; i++)
  {
    if (dx[i] > (int32_t)WHEEL_HOME_STICK_DEADBAND ||
        dx[i] < -(int32_t)WHEEL_HOME_STICK_DEADBAND)
    {
      ok = 0U;
    }
  }
  return ok;
}

/* 摇杆是否回中（手动页记录参考的前置条件之一）。 */
static uint8_t ctrl_sticks_centered(const wheel_home_input_t *in)
{
  return ctrl_sticks_centered_raw(in->lx, in->ly, in->rx, in->ry);
}

static void ctrl_sample_encoders(wheel_home_input_t *in)
{
  uint8_t w;

  if (s_ctrl_tracker_ready == 0U)
  {
    uint32_t cpr[4];

    for (w = 0U; w < 4U; w++)
    {
      wheel_feedback_init(&s_ctrl_fb[w],
                          (uint16_t)__HAL_TIM_GET_COUNTER(s_ctrl_encoders[w].tim));
      cpr[w] = s_ctrl_encoders[w].cpr;
    }
    wheel_home_init(&s_ctrl_home, cpr);
    s_ctrl_tracker_ready = 1U;
  }

  for (w = 0U; w < 4U; w++)
  {
    int32_t delta = wheel_feedback_update(&s_ctrl_fb[w],
        (uint16_t)__HAL_TIM_GET_COUNTER(s_ctrl_encoders[w].tim));

    in->position[w] = s_ctrl_fb[w].counts * s_ctrl_encoders[w].forwardSign;
    in->delta[w] = delta * s_ctrl_encoders[w].forwardSign;
  }
}

/* 手柄链路：首次调用时点亮接收器并做一次配置；之后每轮取一帧。
 * 失联超过期限就睡觉（零输出 + SLP 低），重连后必须等摇杆回中若干帧
 * 才允许输出，避免"摇杆还偏着就通电"。
 * awake 指针由调用方持有：Ctrl 用它判断驱动是否已经上电。 */
static void ctrl_link_service(ps2_state_t *pad, uint8_t *linked,
                              uint8_t *frame_ok, uint32_t *last_good_tick,
                              uint8_t *awake)
{
  static uint32_t s_rearm_start_ms;
  static uint8_t s_rearm_active;

  *frame_ok = 0U;

  /* 标定模式不轮询（电机已睡眠，按键无效），但保持链路计时新鲜。 */
  if (drive_calibration_mode != 0U)
  {
    *last_good_tick = HAL_GetTick();
    return;
  }

  if (*linked == 0U)
  {
    ps2_init();
    ps2_config();

    /* 冷启动/重连：只关驱动，不启动通道。启动时机交给 ctrl_apply 的
     * 唤醒分支，那里才保证"立刻要输出"。 */
    ctrl_motors_sleep();
    *awake = 0U;
    *linked = 1U;
    *last_good_tick = HAL_GetTick();
    /* 重连/冷启动：先要求摇杆回中且 SELECT 释放，并保持满 CTRL_REARM_MS。 */
    s_rearm_active = 1U;
    s_rearm_start_ms = HAL_GetTick();
  }

  if (ps2_poll(pad) == PS2_FRAME_OK)
  {
    *last_good_tick = HAL_GetTick();
    if (s_rearm_active != 0U)
    {
      uint8_t centered =
          ctrl_sticks_centered_raw(pad->lx, pad->ly, pad->rx, pad->ry);
      /* SELECT 现在按着（低有效）就说明这个按键可能在断连前就按下了：
       * 不把它当成新操作，也不能让计时白白走完。 */
      uint8_t select_released = (uint8_t)(
          ((pad->buttons >> WHEEL_HOME_SELECT_BIT) & 1U) != 0U);

      if (centered == 0U || select_released == 0U)
      {
        /* 条件破掉就重新计时：必须"连续"满足满 100 ms。 */
        s_rearm_start_ms = HAL_GetTick();
        return;
      }
      if ((uint32_t)(HAL_GetTick() - s_rearm_start_ms) < CTRL_REARM_MS)
      {
        /* 还在等：本帧无效 => 状态机零输出、不被误判为操作。 */
        return;
      }
      s_rearm_active = 0U;
    }
    *frame_ok = 1U;
  }
  else if ((uint32_t)(HAL_GetTick() - *last_good_tick) >= CTRL_LINK_LOSS_MS)
  {
    ctrl_motors_sleep();
    *awake = 0U;
    *linked = 0U;
  }
  else
  {
    /* 单帧坏：保持链路，但本帧无效。 */
  }
}

void StartCtrlTask(void *argument)
{
  uint8_t linked = 0U;
  uint8_t awake = 0U;
  uint32_t last_good_tick = 0U;
  uint8_t w;

  (void)argument;

#ifdef FLOG_ENABLE
  /* 日志初始化放在 Ctrl 任务里：此时串口已经由 main 初始化完毕，
   * 而且只在单一任务上下文里调，不需要额外同步。 */
  flog_init();
  flog_printf("BOOT flog v1 usart3=115200 uart4=9600");
#endif

  /* 页面模型只被本任务推进：这里初始化一次，之后每轮 ui_model_step。 */
  ui_model_init(&s_ui_model, HAL_GetTick());
  /* 菜单自检服务同属本任务私有状态：先归零再进循环。 */
  selfcheck_service_init(&s_ctrl_check);
  s_ctrl_check_prev = SELFCHECK_IDLE;

  for (;;)
  {
    uint32_t cycle_start = HAL_GetTick();
    wheel_home_input_t in;
    wheel_home_output_t out;
    int32_t command[4];
    uint8_t frame_ok = 0U;
    ps2_state_t pad;
    ui_model_req_t ureq;
    power_snapshot_t pw;
    uint8_t ui_cancel_motion = 0U;
    uint8_t ui_action_busy = 0U;
    uint8_t drive_permitted = 0U;
    uint8_t result_window = 0U;

    /* 0) 单归属说明：boot_ui 已收敛为渲染层，不再有"电机归属/移交"概念。
     *    页面模型、输入事件、编码器累计与全部电机动作（含写零）都只在本任务；
     *    boot_ui_ready 只表示渲染管线就绪。上电首帧起本任务就用零命令走
     *    ctrl_apply → 驱动保持睡眠，满足"开机不自动转轮"。 */

    /* 手柄链路立刻建立：菜单/页面导航不依赖任何"自检结束"事件。
     * 冷启动分支会先把驱动按进睡眠（SLP 低 + CCR 0）。 */
    ctrl_link_service(&pad, &linked, &frame_ok, &last_good_tick, &awake);

    /* 电源快照：powerTask 并发写 power_monitor 的内部状态，跨任务读
     * 必须临界区拷贝（见 ctrl_power_read）。本拍供自检前置与 UI 快照用。 */
    ctrl_power_read(&pw);

    /* 1) 采样：所有模式都做（标定期间也要维持位置连续性）。
     *    ready=0 也要做——结果页的 5 秒窗口在 ui_main_page() 之前就开了，
     *    如果这里直接跳过，UI 拿到的快照永远 frame_valid=0，
     *    USER 入口被"启动条件"挡住。启动阶段发布的是有效按钮/摇杆 +
     *    空脉冲 + 禁止驾驶，不是放开普通摇杆输出。 */

    /* 自检未通过禁止驱动：可以看页面、可以记录参考，但不允许出力。
     * 这里只拦输出，不改状态机，避免把"没通过"混进归位模式。 */
    ctrl_sample_encoders(&in);
    in.now_ms = HAL_GetTick();
    in.frame_valid = 0U;
    in.feedback_valid = 1U;
    in.calibration_mode = drive_calibration_mode;
    in.manual_calib_page = 0U; /* 栈变量显式清 0，页面判出后下面按真值赋 */
    in.buttons = 0xFFFFU;
    in.lx = CTRL_STICK_CENTER;
    in.ly = CTRL_STICK_CENTER;
    in.rx = CTRL_STICK_CENTER;
    in.ry = CTRL_STICK_CENTER;

    /* 提示音推进：非阻塞，10 ms 为一拍，不影响手柄采样节奏。 */
    buzzer_step(HAL_GetTick());

    /* 串口发送推进：只把队列里攒的字节丢给 HAL 的中断发送，不在本任务里等。
     * 必须每轮都调，否则队列一满就开始丢日志。 */
#ifdef FLOG_ENABLE
    flog_poll();
#endif

    /* 2) 手柄：ctrl_link_service 已在本轮取过一帧，直接落到输入里。 */
    in.frame_valid = frame_ok;
    if (frame_ok != 0U)
    {
      in.buttons = pad.buttons;
      in.lx = pad.lx;
      in.ly = pad.ly;
      in.rx = pad.rx;
      in.ry = pad.ry;
    }
    else
    {
      /* 坏帧/失联：按键一律视为释放，摇杆回中，绝不拿旧值驱动。 */
      in.buttons = 0xFFFFU;
    }

    /* 2b) 页面模型推进：喂原始帧（按键/摇杆无论自检是否通过都要能导航）。
     *     req 每拍由模型清零（单发语义），必须在本拍立即消费。 */
    {
      ui_model_in_t uin;

      /* 自检 DONE 的 5 s 标定入口窗口：真实毫秒计时。
       * 进入 DONE 的沿在这里补记（cancel 发生在本块内，step 里的
       * 完成沿下一拍补记，误差一个控制周期，对 5 s 窗口无影响）。 */
      if (s_ctrl_check.phase != s_ctrl_check_prev)
      {
        if (s_ctrl_check.phase == SELFCHECK_DONE)
        {
          s_ctrl_check_done_ms = HAL_GetTick();
        }
        s_ctrl_check_prev = s_ctrl_check.phase;
      }
      result_window = (uint8_t)(
          (s_ctrl_check.phase == SELFCHECK_DONE) &&
          ((uint32_t)(HAL_GetTick() - s_ctrl_check_done_ms) <
           SELFCHECK_RESULT_WINDOW_MS)
              ? 1U
              : 0U);

      uin.buttons = in.buttons;
      uin.lx = in.lx;
      uin.ly = in.ly;
      uin.rx = in.rx;
      uin.ry = in.ry;
      uin.user_pressed = (uint8_t)(
          (HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin) == GPIO_PIN_RESET)
              ? 1U
              : 0U);
      uin.frame_valid = in.frame_valid;
      /* 回喂：自检服务结果、上一拍自检/归位占用、自检结果窗口。
       * selfcheck_passed 来自 Ctrl 自己的菜单自检服务（单归属）：
       * 只有四轮全过的完整一轮才为 1，重新发起/取消即归 0。 */
      uin.selfcheck_passed = s_ctrl_check.passed;
      /* 自检前置回喂（spec §4 倒计时冻结判据）：帧有效 + 四轴回中 +
       * 供电正常 + 非标定模式——与下方 selfcheck_start 消费门槛同一口径
       * （不含服务占用：RUNNING 时页面早已离开确认页，倒计时不再推进）。 */
      uin.selfcheck_ready = (uint8_t)(
          (in.frame_valid != 0U &&
           ctrl_sticks_centered_raw(in.lx, in.ly, in.rx, in.ry) != 0U &&
           pw.vdd_ok != 0U && drive_calibration_mode == 0U)
              ? 1U
              : 0U);
      uin.action_busy = s_ctrl_action_busy;
      uin.result_window_open = result_window;
      uin.now_ms = HAL_GetTick();
      ui_model_step(&s_ui_model, &uin, &ureq);
      ui_cancel_motion = ureq.cancel_motion;
      ui_action_busy = uin.action_busy;

      /* 手动标定页作用域：只有该页长按 SELECT 才允许记录参考。
       * 必须每轮显式赋值——in 是未清零的栈变量（审查确认项）。 */
      in.manual_calib_page =
          (s_ui_model.page == UI_PAGE_MANUAL_CAL) ? 1U : 0U;

      /* --- 请求消费（单发，当拍生效） ------------------------------ */
      /* 自检前置：帧有效 + 四轴回中 + 电源正常（vdd_ok：采样成功且
       * >= 3100 mV，掉压的轨不得自检）+ 非标定模式，
       * 且自检/归位都不在运行（服务独占电机，不叠加）。 */
      if (ureq.selfcheck_start != 0U)
      {
        if (in.frame_valid != 0U &&
            ctrl_sticks_centered_raw(in.lx, in.ly, in.rx, in.ry) != 0U &&
            pw.vdd_ok != 0U && drive_calibration_mode == 0U &&
            s_ctrl_check.phase != SELFCHECK_RUNNING &&
            s_ctrl_home_busy == 0U)
        {
          selfcheck_service_request(&s_ctrl_check, HAL_GetTick());
        }
      }
      /* 归位请求：无参考/非 MANUAL 模式由 wheel_home 自己拒绝落地；
       * vdd_ok=0（掉压）时同样不受理——spec §9.2 低压禁止一切运动。 */
      if (ureq.home_start != 0U && pw.vdd_ok != 0U)
      {
        wheel_home_request_home(&s_ctrl_home);
      }
      /* 取消时序：服务 cancel 必须先于本拍 step——模块 cancel 不清
       * 调用方缓冲，先取消后 step，本拍 command 天然归零（审查确认的
       * 一拍窗口在这里关死）。 */
      if (ureq.cancel_motion != 0U)
      {
        selfcheck_service_cancel(&s_ctrl_check);
      }
      /* 归位只在 HOME 页存活（spec §7/§10）：页面模型发起的归位——菜单
       * L1 或驾驶页 SELECT 快捷——都会转到 HOME 页；R1/新按键/摇杆越区
       * 离开页面时模型给出 cancel_motion。这里补两条取消路径：
       *   a) cancel_motion（含动作页摇杆越区、R1 取消）；
       *   b) pending/RUNNING 却不在 HOME 页（例如等待阶段 R1 退出、
       *      页面兜底转移）——请求一并丢弃，绝不从其它页面起转。
       * cancel 幂等；RUNNING 的停止在本拍稍后的 wheel_home_step 落地
       * （CANCELLED、四轮零脉冲、恢复要求先看到释放）。 */
      if (ureq.cancel_motion != 0U ||
          ((s_ctrl_home.mode == WHEEL_HOME_RUNNING ||
            s_ctrl_home.request_pending != 0U) &&
           s_ui_model.page != UI_PAGE_HOME))
      {
        wheel_home_cancel(&s_ctrl_home);
      }
      /* 手动标定页长按记录参考：帧有效 + 回中由这里核对，
       * 静止计时已由每轮的 track_manual_still 累积。 */
      if (ureq.record_ref != 0U)
      {
        /* 掉压时同样拒绝记录参考（写参考不改输出，但写进去的参考会在
         * 电源恢复后被用来驱动归位——低压下采的位置不可信）。 */
        if (in.frame_valid != 0U && pw.vdd_ok != 0U &&
            ctrl_sticks_centered(&in) != 0U)
        {
          (void)wheel_home_record_manual_reference(&s_ctrl_home, &in,
                                                   HAL_GetTick());
        }
      }

      /* 输出上限落地（单发新值请求，同其余 req 的当拍消费口径）：
       * 模型在 进页/页内步进实际改变百分比 的当拍置 output_cap_changed，
       * 这里落到 drive_ctrl 运行时上限。pct 为占空比百分比：pct×168 =
       * pct% × (ARR+1=16800)/100，50% = 8400 = 现状满输出（与默认一致，
       * 进页当拍为幂等写）；set_max_pulse 幂等夹取 [840, 15120]（90%）。同一任务
       * 内读写，无竞争；变更当拍之后的驾驶混合（第 4 步）即按新上限
       * 重映射。 */
      if (ureq.output_cap_changed != 0U)
      {
        drive_ctrl_set_max_pulse((int32_t)s_ui_model.output_cap_pct * 168);
      }
    }

    /* 2c) 驾驶许可仲裁：驾驶页 + 自检通过 + 无故障 + 回中武装。
     *     回中计时用独立变量（口径同 CTRL_REARM_MS）：页面重进即重新起算，
     *     出杆重置计时；武装后出杆即驾驶，离开驾驶页立即解除。 */
    {
      uint8_t centered = ctrl_sticks_centered_raw(in.lx, in.ly, in.rx, in.ry);
      uint32_t now = HAL_GetTick();

      if (s_ui_model.page != UI_PAGE_DRIVE)
      {
        s_drive_center_armed = 0U;
        s_drive_center_start = now;
      }
      else if (s_drive_center_armed == 0U)
      {
        if (centered != 0U)
        {
          if ((uint32_t)(now - s_drive_center_start) >= CTRL_REARM_MS)
          {
            s_drive_center_armed = 1U;
          }
        }
        else
        {
          s_drive_center_start = now; /* 出杆：重新计时，防止带杆进页窜车 */
        }
      }
      /* action_busy 由自检/归位状态回喂；本次上电未通过自检时驾驶页
       * 零输出并显示"请先自检"（快照 selfcheck_passed=0）。
       * vdd_ok 是供电异常的最高优先级闸（spec §11.4）：3.3 V 轨低于
       * 3100 mV 时即使其它条件全满足也零输出。电池域欠压/采样错误
       * （POWER_BATTERY_CONFIGURED=1 时才可能置位）同属供电故障停机
       * （spec §9.2）：与 vdd_ok 无关地关闭驾驶许可。 */
      drive_permitted = (uint8_t)(
          (s_ui_model.page == UI_PAGE_DRIVE) && (s_ctrl_check.passed != 0U) &&
          (s_ctrl_fault_mask == 0U) && (ui_action_busy == 0U) &&
          (pw.vdd_ok != 0U) && (pw.undervolt == 0U) &&
          (pw.battery_error == 0U) && (s_drive_center_armed != 0U));
    }

    /* 3) 仲裁：状态机只看输入，不写硬件。
     *    自检未通过时喂一个"无效帧"：按键/摇杆仍可被 UI 读走（快照照写），
     *    但状态机不会接受短按归位、长按记录等任何改变控制权的动作。 */
    {
      wheel_home_input_t arbitrate = in;

      if (s_ctrl_check.passed == 0U || pw.vdd_ok == 0U)
      {
        /* 自检未通过或掉压：喂"无效帧"（按键/摇杆仍可被 UI 读走）。 */
        arbitrate.frame_valid = 0U;
        arbitrate.buttons = 0xFFFFU;
      }
      /* spec §11.9：归位出现无效手柄帧即停止并取消——当拍走公开取消
       * 入口（cancel_pending 由本拍稍后的 wheel_home_step 落地：wh_abort
       * → CANCELLED、四轮零脉冲、恢复后要求先看到释放）。wheel_home 内部
       * 的 200 ms 帧失联规则保留作兜底（如 BUTTON_HELD 手势路径），
       * 但 RUNNING 的电机输出不再等它。 */
      if (arbitrate.frame_valid == 0U &&
          (s_ctrl_home.mode == WHEEL_HOME_RUNNING ||
           s_ctrl_home.mode == WHEEL_HOME_BUTTON_HELD))
      {
        wheel_home_cancel(&s_ctrl_home);
      }
      wheel_home_step(&s_ctrl_home, &arbitrate, &out);

      /* 手动页静止跟踪：每轮都喂，让"连续静止 SETTLE_MS"能真正累积。
       * 只在收到记录请求那一拍判定的话，计时从零开始，必然第一次就失败。 */
      (void)wheel_home_track_manual_still(&s_ctrl_home, &arbitrate,
                                          HAL_GetTick());
    }

    /* 3b) 菜单自检服务：驾驶仲裁之前推进。RUNNING 时 command 由服务独占
     *     （±SELFCHECK_PWM_PULSE，其余轮 0），第 4 步不得再混合驾驶输出。
     *     运行中摇杆越出驾驶死区兜底取消（页面模型在动作页已有一路）；
     *     出现无效手柄帧（坏帧/失联）同样立即取消——spec §11.9"归位或
     *     自检出现无效手柄帧即停止并取消；重连不自动续跑"（服务落 DONE，
     *     重连后只能重新从菜单发起）。取消当拍立即清缓冲——cancel 不清
     *     调用方的 command。 */
    if (s_ctrl_check.phase == SELFCHECK_RUNNING &&
        (frame_ok == 0U ||
         ctrl_sticks_centered_raw(in.lx, in.ly, in.rx, in.ry) == 0U))
    {
      selfcheck_service_cancel(&s_ctrl_check);
      for (w = 0U; w < 4U; w++)
      {
        command[w] = 0;
      }
    }
    else
    {
      selfcheck_service_step(&s_ctrl_check, in.delta, HAL_GetTick(), command);
    }
    /* 快照带上按键与帧有效性：UI 顶栏/手动页只读这里。 */
    out.buttons = in.buttons;
    out.frame_valid = in.frame_valid;
    out.fault_mask = s_ctrl_fault_mask;
    /* 页面判"能否记录参考"要和这里写参考用同一批量，否则两边会分叉。 */
    for (w = 0U; w < 4U; w++)
    {
      out.position[w] = in.position[w];
      out.cpr[w] = s_ctrl_home.cpr[w];
    }
    out.feedback_valid = in.feedback_valid;
    out.lx = in.lx;
    out.ly = in.ly;
    out.rx = in.rx;
    out.ry = in.ry;
    ctrl_home_publish(&out);

    /* 4) 输出仲裁：归位/等待用状态机结果，只有 MANUAL 且 drive_permitted
     *    才落双轴混合（v=ly 前进，t=rx 转向）。驾驶页仲裁未通过、手动
     *    标定页、cancel_motion 请求一律零输出（零命令经 ctrl_apply 走
     *    停止流程关使能）。原 boot_ui_drive_enabled 的"页面级许可"职责
     *    已由 drive_permitted 一并承担。 */
    if (ui_cancel_motion != 0U || s_ui_model.page == UI_PAGE_MANUAL_CAL ||
        pw.vdd_ok == 0U || pw.undervolt != 0U || pw.battery_error != 0U)
    {
      /* 掉压/欠压/采样错误当拍先清命令（归位脉冲也不例外）；wheel_home
       * 状态机随后经上面的无效帧路径落成 CANCELLED（spec §9.2 供电故障
       * 停机：POWER_BATTERY_CONFIGURED=0 时二者恒 0，接线到位待启用）。 */
      for (w = 0U; w < 4U; w++)
      {
        command[w] = 0;
      }
    }
    else if (s_ctrl_check.phase == SELFCHECK_RUNNING)
    {
      /* 自检独占电机：command 已由 3b 的服务本拍写入，驾驶混合与
       * 归位脉冲都不插手；取消当拍已由 3b/ui_model 清零。 */
    }
    else if (out.mode == WHEEL_HOME_RUNNING && s_ui_model.page != UI_PAGE_HOME)
    {
      /* 兜底：归位脉冲只允许出现在 HOME 页（spec §7"运动动作中先取消
       * 并停止，再返回"）。任何其它页面一律清零，绝不让 out.pulse 继续
       * 驱动电机到 8 s 超时。 */
      for (w = 0U; w < 4U; w++)
      {
        command[w] = 0;
      }
    }
    else if (out.mode == WHEEL_HOME_MANUAL)
    {
      if (drive_permitted != 0U)
      {
        drive_ctrl_mix(in.ly, in.rx, command);
      }
      else
      {
        for (w = 0U; w < 4U; w++)
        {
          command[w] = 0;
        }
      }
    }
    else
    {
      for (w = 0U; w < 4U; w++)
      {
        command[w] = out.pulse[w];
      }
    }

    /* 4b) 运动反馈监测：PWM 够大却不动 -> NO FB；方向相反 -> DIR ERR。
     *    只判定与本轮降级，硬件动作统一由 ctrl_apply() 执行。 */
    {
      motor_health_in_t hin;
      motor_health_out_t hout;
      uint8_t measurable = 1U;
      uint8_t reversing = 0U;

      if (s_ctrl_health_ready == 0U)
      {
        /* 静态区是零值，必须显式初始化（否则首轮就会误判/误锁）。 */
        motor_health_init(&s_ctrl_health);
        s_ctrl_health_ready = 1U;
      }

      for (w = 0U; w < 4U; w++)
      {
        hin.pwm[w] = command[w];
        hin.fb[w] = in.delta[w];

        /* 换向：符号与上一轮不同，重新给启动宽限。 */
        if (s_ctrl_last_pwm[w] != 0 && command[w] != 0 &&
            ((s_ctrl_last_pwm[w] > 0) != (command[w] > 0)))
        {
          reversing = 1U;
        }
      }
      hin.now_ms = HAL_GetTick();

      /* 睡眠 / 换向 / 手动标定页 / 手柄失联 / 渲染层未就绪：本轮不监测。 */
      if (awake == 0U || reversing != 0U ||
          s_ui_model.page == UI_PAGE_MANUAL_CAL || in.frame_valid == 0U ||
          boot_ui_ready == 0U)
      {
        measurable = 0U;
      }
      hin.measurable = measurable;

      /* 故障清除的唯一入口：菜单自检服务完整跑完一轮，且这一轮四轮全过。
       * 不接受长按/记录参考等操作来清故障，避免"黑掉一台车"当成修好。 */
      if (s_ctrl_check.seq != s_ctrl_check_seq)
      {
        s_ctrl_check_seq = s_ctrl_check.seq;
        motor_health_clear_after_selfcheck(&s_ctrl_health, s_ctrl_check.passed);
        s_ctrl_health_ready = 1U;
        if (s_ctrl_check.passed == 0U)
        {
          buzzer_alarm();
        }
      }

#ifdef FLOG_ENABLE
      /* 记下判定输入与结果，供日志使用（不影响判定本身）。 */
      for (w = 0U; w < 4U; w++)
      {
        s_flog_pwm[w] = hin.pwm[w];
        s_flog_delta[w] = hin.fb[w];
      }
#endif

      motor_health_step(&s_ctrl_health, &hin, &hout);
      s_ctrl_fault_mask = hout.fault_mask;
      for (w = 0U; w < 4U; w++)
      {
        s_ctrl_health_code[w] = hout.code[w];
      }

      /* 故障锁存时响一次提示。buzzer_alarm 自己会去重（播完之前不重开），
       * 所以这里每轮调用也不会变成持续长鸣。 */
      if (s_ctrl_fault_mask != 0U)
      {
        buzzer_alarm();
      }

      if (hout.stop_all != 0U && s_ctrl_check.phase != SELFCHECK_RUNNING)
      {
        /* 锁存后的降级：全部清零；上一轮 PWM 也归零，避免"换向"误判。
         * 例外：菜单自检运行中服务独占 command（spec §11.4"重新自检只
         * 放行其自身受限动作"）——若被 stop_all 清零，锁存的故障就永远
         * 无法通过重新自检清除了。服务结果由 selfcheck_service 自己把关。 */
        for (w = 0U; w < 4U; w++)
        {
          command[w] = 0;
        }
      }

      for (w = 0U; w < 4U; w++)
      {
        s_ctrl_last_pwm[w] = command[w];
      }
    }

    /* 4c) UI 快照发布（seq-lock）：渲染层只读，不阻塞控制。
     *     放在 4b 之后：fault_mask 取的是本轮判定完的最新值。 */
    {
      ui_snapshot_t snap;
      uint32_t now = HAL_GetTick();
      /* 行驶状态分类跟随运行时上限（转向分量恒 = 上限/2）。 */
      int32_t v = drive_ctrl_map_axis(in.ly, drive_ctrl_max_pulse);
      int32_t t = drive_ctrl_map_axis(in.rx, drive_ctrl_max_pulse / 2);

      snap.page = s_ui_model.page;
      snap.menu_sel = s_ui_model.menu_sel;
      snap.sub_sel = s_ui_model.sub_sel;
      snap.sub_scroll = s_ui_model.sub_scroll;
      snap.output_cap_pct = s_ui_model.output_cap_pct;
      snap.confirm_focus = s_ui_model.confirm_focus;
      snap.confirm_remain_ms = s_ui_model.confirm_remain_ms;
      snap.confirm_blocked = s_ui_model.confirm_blocked;
      snap.result_view = s_ui_model.result_view;
      snap.sel_since_ms = s_ui_model.sel_since_ms;
      snap.drive_permitted = drive_permitted;
      snap.action_busy = (uint8_t)(
          (s_ctrl_check.phase == SELFCHECK_RUNNING) ||
          (out.mode == WHEEL_HOME_RUNNING));
      s_ctrl_action_busy = snap.action_busy;
      s_ctrl_home_busy =
          (uint8_t)((out.mode == WHEEL_HOME_RUNNING) ? 1U : 0U);
      snap.selfcheck_passed = s_ctrl_check.passed;
      snap.result_window_open = result_window;
      snap.fault_mask = s_ctrl_fault_mask;
      snap.reference_valid = out.reference_valid;
      snap.frame_valid = in.frame_valid;
      snap.drive_status = ui_drive_status_classify(v, t);

      for (w = 0U; w < 4U; w++)
      {
        /* 自检逐轮状态：服务自己的快照（0 等待 1 运行 2 通过 3 失败）。 */
        snap.selfcheck_state[w] = s_ctrl_check.state[w];

        /* 归位逐轮状态：RUNNING 中按输出/到位/等待分类，
         * 结果为失败类时全部报 3，其余报 0 等待。 */
        if (out.mode == WHEEL_HOME_RUNNING)
        {
          if (out.pulse[w] > 0 || out.pulse[w] < 0)
          {
            snap.home_state[w] = 1U; /* 运动中 */
          }
          else if ((out.settled_mask & (uint8_t)(1U << w)) != 0U)
          {
            snap.home_state[w] = 2U; /* 已到位 */
          }
          else
          {
            snap.home_state[w] = 0U; /* 等待 */
          }
        }
        else if (out.result == WHEEL_HOME_RESULT_CANCELLED ||
                 out.result == WHEEL_HOME_RESULT_NO_REF ||
                 out.result == WHEEL_HOME_RESULT_STALL ||
                 out.result == WHEEL_HOME_RESULT_TIMEOUT ||
                 out.result == WHEEL_HOME_RESULT_FEEDBACK_ERROR)
        {
          snap.home_state[w] = 3U; /* 失败 */
        }
        else
        {
          snap.home_state[w] = 0U;
        }

        /* 圈数：s_ctrl_fb 的累计计数（含 16 位回绕处理）换算毫圈，
         * 乘前进符号得到"前进为正"；每轮都发，与页面无关，计数连续。 */
        snap.wheel_milliturns[w] =
            wheel_feedback_milliturns(&s_ctrl_fb[w], s_ctrl_encoders[w].cpr) *
            s_ctrl_encoders[w].forwardSign;

        /* 转速：累计 100 ms 窗口内的编码器增量，闭合时换算并沿用到下一窗。 */
        if (s_win_started == 0U)
        {
          s_win_start = now;
          s_win_started = 1U;
        }
        s_win_counts[w] += in.delta[w];
        snap.wheel_rpm[w] = s_win_rpm[w];
        snap.wheel_sample_valid[w] = s_win_valid;
      }

      /* 归位阻塞原因与电池域：电源域来自 power_monitor 的防撕裂快照。 */
      snap.home_block_reason = (uint8_t)((s_ctrl_fault_mask != 0U)
                                             ? 3U
                                             : ((out.reference_valid != 0U)
                                                    ? 0U
                                                    : 1U));
      snap.battery_valid = pw.battery_valid;
      snap.battery_error = pw.battery_error;
      snap.undervolt = pw.undervolt;
      snap.vdd_ok = pw.vdd_ok;
      snap.battery_mv = pw.battery_mv;

      if ((uint32_t)(now - s_win_start) >= BOOT_UI_ENC_SAMPLE_MS *
                                              BOOT_UI_ENC_SAMPLES)
      {
        uint32_t elapsed = now - s_win_start;

        for (w = 0U; w < 4U; w++)
        {
          s_win_rpm[w] = boot_ui_rpm_from_elapsed(s_win_counts[w],
                                                  s_ctrl_encoders[w].cpr,
                                                  elapsed);
          s_win_counts[w] = 0;
        }
        s_win_start = now;
        s_win_valid = 1U;
        /* 用刚闭合的窗口刷新本轮快照，避免渲染层看到一拍旧值。 */
        for (w = 0U; w < 4U; w++)
        {
          snap.wheel_rpm[w] = s_win_rpm[w];
          snap.wheel_sample_valid[w] = s_win_valid;
        }
      }

      ctrl_ui_publish(&snap);
    }

    /* 5) 电机输出（唯一归属）：Ctrl 自始至终独占电机（spec §11.2 单归属）。
     *    命令全零即走停止流程关使能：归位完成、部分故障停止、松杆、
     *    手动标定页都归入这里。 */
    ctrl_apply(command, &awake);

    /* 日志：状态行（定时）+ 关键事件（变化时）。
     * 放在硬件写完之后：这里打出来的 pwm 才是真正生效的那一份。
     * 只读 s_ctrl_* 与局部变量，不改变任何控制行为。 */
#ifdef FLOG_ENABLE
    {
      uint32_t now = HAL_GetTick();

      /* 故障锁存 / 解除：立刻打，带上四轮故障码与当时的 pwm、增量。
       * 这三行是排查"为什么报警"的第一手证据。 */
      if (s_ctrl_fault_mask != s_flog_prev_fault)
      {
        flog_printf("EVENT fault mask=%02x code=%s,%s,%s,%s pwm=%ld,%ld,%ld,%ld fb=%ld,%ld,%ld,%ld",
                    (unsigned)s_ctrl_fault_mask,
                    motor_health_code_text(s_ctrl_health_code[0]),
                    motor_health_code_text(s_ctrl_health_code[1]),
                    motor_health_code_text(s_ctrl_health_code[2]),
                    motor_health_code_text(s_ctrl_health_code[3]),
                    (long)s_flog_pwm[0], (long)s_flog_pwm[1],
                    (long)s_flog_pwm[2], (long)s_flog_pwm[3],
                    (long)s_flog_delta[0], (long)s_flog_delta[1],
                    (long)s_flog_delta[2], (long)s_flog_delta[3]);
        s_flog_prev_fault = s_ctrl_fault_mask;
      }

      /* 手柄连上/断开：推杆没反应的第一步就是查这个。 */
      if (awake != s_flog_prev_linked)
      {
        flog_printf("EVENT pad awake=%u linked=%u frame=%u sc_pass=%u",
                    (unsigned)awake, (unsigned)linked, (unsigned)frame_ok,
                    (unsigned)s_ctrl_check.passed);
        s_flog_prev_linked = awake;
      }

      /* 归位模式变化：进/出 RUNNING 是电机突然开始转动的时刻。 */
      if ((uint8_t)out.mode != s_flog_prev_mode)
      {
        flog_printf("EVENT home mode=%u page=%u manual_pg=%u",
                    (unsigned)out.mode, (unsigned)s_ui_model.page,
                    (unsigned)(s_ui_model.page == UI_PAGE_MANUAL_CAL));
        s_flog_prev_mode = (uint8_t)out.mode;
      }

      /* 定时状态行：每 FLOG_STATUS_MS 一行，看清"每一轮到底在干什么"。
       * 用 tick 差值判等，tick 回绕时也不会卡住。 */
      if ((int32_t)(now - s_flog_next_ms) >= 0)
      {
        s_flog_next_ms = now + FLOG_STATUS_MS;
        /* lk=链路是否建立（linked），aw=是否唤醒，fv=本帧是否有效。
         * st=摇杆四轴原始值：排查"推杆没反应"先看这里动没动。 */
        flog_printf("ST t=%lu lk=%u aw=%u fv=%u md=%u pg=%u st=%u,%u,%u,%u "
                    "pwm=%ld,%ld,%ld,%ld fb=%ld,%ld,%ld,%ld",
                    (unsigned long)now, (unsigned)linked, (unsigned)awake,
                    (unsigned)frame_ok, (unsigned)out.mode,
                    (unsigned)s_ui_model.page,
                    (unsigned)pad.lx, (unsigned)pad.ly,
                    (unsigned)pad.rx, (unsigned)pad.ry,
                    (long)s_flog_pwm[0], (long)s_flog_pwm[1],
                    (long)s_flog_pwm[2], (long)s_flog_pwm[3],
                    (long)s_flog_delta[0], (long)s_flog_delta[1],
                    (long)s_flog_delta[2], (long)s_flog_delta[3]);
      }
    }
#endif

    /* 手动标定页的记录请求已在 2b 的 record_ref 分支处理（Ctrl 单归属，
     * 页面模型在 Ctrl 内推进，不再经过 boot_ui_manual_* 接口）。 */

    if (drive_calibration_mode != 0U)
    {
      /* CAL 模式：驱动必须保持睡眠（标定上电不转轮，含重新上电后）。 */
      ctrl_motors_sleep();
      awake = 0U;
    }

    /* Include the PS2 transaction in the 10 ms period. Long setup/wake
     * cycles yield once rather than busy-looping or delaying another 10 ms. */
    {
      uint32_t elapsed = HAL_GetTick() - cycle_start;
      osDelay(elapsed < CTRL_POLL_MS ? CTRL_POLL_MS - elapsed : 1U);
    }
  }
}
/* USER CODE END Application */

