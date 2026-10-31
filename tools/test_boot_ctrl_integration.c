/* Host integration: real boot_ui render loop + real Ctrl task + real
 * selfcheck_service + real power_monitor, driven by a deterministic
 * ucontext scheduler. Only hardware and RTOS timing are simulated.
 * Each scenario runs in a fresh process (one scenario per argv[1]).
 *
 * 场景（plan Task 9 Step 2）：
 *   0 正常自检：每轮两个 ~500 ms 运动段、段间零输出、总时长 5.6 s±100 ms；
 *     通过后进驾驶页，回中门槛前零输出、门槛后满输出；电池未配置读数 B--.-V。
 *   1 缺反馈：全程无反馈 → 四轮 NO FB、不通过、故障锁存、驾驶零输出。
 *   2 反向：D 轮反转段冻结编码器 → REV ERR，其余轮通过。
 *   3 跳过：自检运行中进入标定模式 → 全轮 SKIP、无输出。
 *   4 取消：B 轮运行中推杆越区 → 取消，已完成轮结果保留、后续轮不出力。
 *   5 锁存：故障锁存后允许重新自检；失败不清锁存；全过 + 回中门槛才出力。
 *   6 归位/标定：无参考不转；L1 发起归位不被同次 L1 取消；手动标定
 *     （连续静止 + 新长按 2 s）成功、断连拒绝、长按释放不触发归位。
 */
#define _XOPEN_SOURCE 700
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <ucontext.h>
#include "main.h"
#include "cmsis_os.h"
#include "boot_ui.h"
#include "ps2_gamepad.h"
#include "ui_render.h"
#include "wheel_home.h"
#include "power_monitor.h"
#include "drive_ctrl.h"

void StartCtrlTask(void *argument);
void boot_ui_run(void);
static void scenario_step(void);

GPIO_TypeDef gpioa, gpiob, gpioc, gpiod, gpioe;
TIM_HandleTypeDef htim2, htim3, htim4, htim5, htim8;
ADC_HandleTypeDef hadc1;
uint16_t host_vrefint_cal = 1490;

typedef struct {
  ucontext_t context;
  unsigned char stack[131072];
  uint32_t wake;
} host_task_t;
static host_task_t tasks[3]; /* 0: Ctrl; 1: UI(boot_ui_run); 2: power */
static ucontext_t scheduler;
static uint32_t tick;
static int current_task, scenario;
static int finished;

/* ---- 观测量 ------------------------------------------------------------- */
static int ctrl_oled;          /* Ctrl 任务里出现 oled_flush = 越界 */
static int ui_motor_write;     /* UI 任务渲染循环写电机引脚 = 越界 */
static uint32_t powered_ms[4]; /* 有非零 PWM 且 SLP 使能的时段 */
static int nsegs;
static struct { uint32_t start, end; int8_t wheel, dir; } segs[32];
static int8_t seg_cur_dir[4];

static uint16_t s_btn = 0xFFFFU;
static uint8_t s_lx = 128U, s_ly = 128U;
static int st = -1;            /* 场景阶段机 */
static uint32_t st_since, mark;
static int cn;                 /* 帧计数（预热用） */
static uint8_t no_inject;      /* 1 = 编码器全程冻结（缺反馈类场景） */
static uint32_t bad_until;     /* 该时刻前 ps2_poll 返回坏帧（断连注入） */
static int script_idle;        /* 1 = 标定模式接管，脚本停摆，主循环守望 */

uint32_t HAL_GetTick(void) { return tick; }

static void report(int ok, const char *msg)
{
  finished = 1;
  printf("%s boot/ctrl integration scenario=%d stage=%d %s "
         "ui_write=%d ctrl_oled=%d\n",
         ok ? "PASS" : "FAIL", scenario, st, msg,
         ui_motor_write, ctrl_oled);
  exit(ok ? 0 : 1);
}

/* 物理轮序与接线（advance 内注入"前进为正"的编码器增量）。
 * 只统计"有非零 PWM 且 SLP 使能"的时段：任何多余的零/睡眠都会切断运动段。 */
static void advance(uint32_t ms)
{
  TIM_HandleTypeDef *enc[4] = {&htim2, &htim3, &htim5, &htim4};
  GPIO_TypeDef *dirport[4] = {GPIOB, GPIOC, GPIOD, GPIOC};
  uint32_t dirpin[4] = {GPIO_PIN_13, GPIO_PIN_0, GPIO_PIN_10, GPIO_PIN_12};
  GPIO_TypeDef *slpport[4] = {GPIOB, GPIOB, GPIOD, GPIOA};
  uint32_t slppin[4] = {GPIO_PIN_12, GPIO_PIN_14, GPIO_PIN_12, GPIO_PIN_8};
  int rawsign[4] = {-1, 1, -1, 1};
  int8_t now_dir[4];
  uint32_t t0 = tick;
  int w;

  for (w = 0; w < 4; ++w) {
    uint32_t cmp = htim8.compare[3 - w]; /* CH4=A, CH3=B, CH2=C, CH1=D */
    int powered = (cmp != 0U) && ((slpport[w]->ODR & slppin[w]) != 0U);
    int high = (dirport[w]->ODR & dirpin[w]) != 0;
    /* A/C 高电平前进；B/D 反极性（与 MOTOR_*_DIR_* 宏一致） */
    int forward = (w == 0 || w == 2) ? high : !high;

    now_dir[w] = (int8_t)(powered ? (forward ? 1 : -1) : 0);
  }
  /* 段开/关 + 反馈注入 */
  for (w = 0; w < 4; ++w) {
    if (now_dir[w] != seg_cur_dir[w]) {
      if (seg_cur_dir[w] != 0) {
        int k;
        for (k = nsegs - 1; k >= 0; --k) {
          if (segs[k].wheel == w && segs[k].end == 0) { segs[k].end = t0; break; }
        }
      }
      if (now_dir[w] != 0 && nsegs < 32) {
        segs[nsegs].start = t0;
        segs[nsegs].end = 0;
        segs[nsegs].wheel = (int8_t)w;
        segs[nsegs].dir = now_dir[w];
        nsegs++;
      }
      seg_cur_dir[w] = now_dir[w];
    }
    if (now_dir[w] != 0) {
      powered_ms[w] += ms;
      if (no_inject == 0U) {
        int freeze = (scenario == 2 && w == 3 && now_dir[w] < 0);
        if (!freeze) {
          int32_t d = (int)now_dir[w] * rawsign[w] * (int32_t)ms * 40;
          enc[w]->counter = (uint16_t)(enc[w]->counter + d);
        }
      }
    }
  }
  tick += ms;
}

void osDelay(uint32_t ms)
{
  tasks[current_task].wake = tick + (ms ? ms : 1);
  if (swapcontext(&tasks[current_task].context, &scheduler) != 0) abort();
}
void *osThreadNew(void (*fn)(void *), void *a, const osThreadAttr_t *attr)
{ (void)fn; (void)a; (void)attr; return NULL; }
void MX_USB_HOST_Init(void) {}
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g) { (void)p; (void)g; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint32_t bits, uint32_t v)
{
  /* 单归属边界（spec §11.2）：渲染任务在 boot_ui_ready 之后不得写任何
   * 电机引脚（SLP/DIR）；buzzer(PD11) 与其他引脚不在此列。 */
  if (current_task == 1 && boot_ui_ready != 0U) {
    if ((p == GPIOB && (bits & (GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14))) ||
        (p == GPIOC && (bits & GPIO_PIN_0)) ||
        (p == GPIOD && (bits & (GPIO_PIN_10 | GPIO_PIN_12))) ||
        (p == GPIOA && (bits & GPIO_PIN_8))) {
      ui_motor_write = 1;
    }
  }
  if (v) p->ODR |= bits; else p->ODR &= ~bits;
}
uint32_t HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint32_t bit)
{ (void)p; (void)bit; return GPIO_PIN_SET; } /* USER 键始终释放 */
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *t, uint32_t c) { (void)t; (void)c; return 0; }
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *t, uint32_t c)
{ t->compare[c / 4] = 0; return 0; }
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *t, uint32_t c) { (void)t; (void)c; return 0; }
int HAL_ADC_ConfigChannel(ADC_HandleTypeDef *a, ADC_ChannelConfTypeDef *c)
{ (void)a; (void)c; return 0; }
int HAL_ADC_Start(ADC_HandleTypeDef *a) { (void)a; return 0; }
int HAL_ADC_PollForConversion(ADC_HandleTypeDef *a, uint32_t ms)
{ (void)a; (void)ms; return 0; }
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *a) { (void)a; return 1490; }
int HAL_ADC_Stop(ADC_HandleTypeDef *a) { (void)a; return 0; }
void oled_init(void) {}
void oled_flush(void)
{
  if (current_task == 0) ctrl_oled = 1; /* Ctrl 不许碰屏幕 */
  osDelay(6); /* OLED 传输耗时：渲染循环必须让出而不是忙等 */
}
void ps2_init(void) {}
void ps2_config(void) { osDelay(120); }
uint8_t ps2_poll(ps2_state_t *pad)
{
  ++cn;
  advance(3);
  if (tick < bad_until) return PS2_FRAME_BAD;
  scenario_step();
  pad->buttons = s_btn;
  pad->lx = s_lx; pad->ly = s_ly; pad->rx = 128U; pad->ry = 128U;
  return PS2_FRAME_OK;
}

/* ---- 场景辅助 ------------------------------------------------------------ */

#define BTNS_NONE 0xFFFFU
#define BTNS_R1   0xF7FFU
#define BTNS_L1   0xFBFFU
#define BTNS_SEL  0xFFFEU

/* 阶段进入时记锚点；超时统一报失败。 */
#define ENTER(n) do { if (st != (n)) { st = (n); st_since = tick; } } while (0)
static void check_timeout(void)
{
  if ((uint32_t)(tick - st_since) > 25000U) report(0, "stage timeout");
}

/* 一段导航：按住 key 直到快照 page/选择条件成立（3-6 帧去抖已含在轮询里）。 */
static ui_snapshot_t snap;

/* 电池未配置断言：battery 域全零，格式化结果必须是 B--.-V，
 * 3.3 V（VDD 换算值）绝不允许出现在电池读数里。 */
static void assert_battery_unconfigured(void)
{
  char b[7];

  ui_snapshot_get(&snap);
  assert(snap.battery_valid == 0U);   /* 未配置：无电池读数 */
  assert(snap.battery_mv == 0U);      /* 3.3 V(VDD) 绝不冒充电池电压 */
  assert(snap.undervolt == 0U);
  ui_render_format_battery(b, snap.battery_valid, snap.battery_mv);
  assert(strcmp(b, "B--.-V") == 0);
  assert(snap.vdd_ok == 1U); /* VDD 正常，但与电池读数无关 */
}

/* 自检段统计：每轮 2 段（前/反），段长 ~500 ms，段间零输出。 */
static void assert_selfcheck_segments(void)
{
  int w, k;
  uint32_t first_start = 0, last_end = 0;

  assert(nsegs == 8); /* 4 轮 × (前转+反转) */
  for (w = 0; w < 4; ++w) {
    int fwd = -1, rev = -1;
    for (k = 0; k < nsegs; ++k) {
      if (segs[k].wheel == w) {
        if (segs[k].dir > 0) { assert(fwd < 0); fwd = k; }
        else { assert(rev < 0); rev = k; }
      }
    }
    assert(fwd >= 0 && rev >= 0 && rev == fwd + 1);
    assert(segs[fwd].end - segs[fwd].start >= 430U);
    assert(segs[fwd].end - segs[fwd].start <= 570U);
    assert(segs[rev].end - segs[rev].start >= 430U);
    assert(segs[rev].end - segs[rev].start <= 570U);
    /* 段间零输出：前转段结束到反转段开始之间没有 PWM */
    assert(segs[rev].start - segs[fwd].end >= 170U);
    assert(segs[rev].start - segs[fwd].end <= 300U);
    if (w == 0) first_start = segs[fwd].start;
    if (w == 3) last_end = segs[rev].end;
  }
  assert(last_end > first_start);
  assert(last_end - first_start >= 5100U);
  assert(last_end - first_start <= 5600U);
}

/* 公共前置：品牌页起，跑一轮完整自检（feedback 由 advance 的物理注入提供；
 * no_inject=1 时缺反馈），然后回到品牌页并把 menu_sel 复位到"驾驶"。
 * 返回 1 = 完成；*passed_out 写入自检是否全过。 */
static int pre_stage_n;
static uint32_t pre_mark;
static int pre_done, pre_passed;
static uint32_t pre_sc_t0, pre_sc_done;

static void pre_selfcheck_step(void)
{
  switch (pre_stage_n) {
  case 100: /* 冷启动回中性预热：链路 + 再武装 100 ms */
    s_btn = BTNS_NONE; s_lx = 128U; s_ly = 128U;
    if (cn >= 40) { pre_stage_n = 0; pre_mark = tick; }
    break;
  case 0: /* R1 → 主菜单 */
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { pre_stage_n = 1; pre_mark = tick; }
    break;
  case 1:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - pre_mark) >= 30U) { pre_stage_n = 2; pre_mark = tick; }
    break;
  case 2: /* 右移选"检查与标定" */
    s_btn = BTNS_NONE; s_lx = 208U;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 1U) { pre_stage_n = 3; pre_mark = tick; }
    break;
  case 3:
    s_btn = BTNS_NONE; s_lx = 128U;
    if ((uint32_t)(tick - pre_mark) >= 30U) { pre_stage_n = 4; pre_mark = tick; }
    break;
  case 4: /* L1 → 子菜单 */
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { pre_stage_n = 5; pre_mark = tick; }
    break;
  case 5:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - pre_mark) >= 30U) { pre_stage_n = 6; pre_mark = tick; }
    break;
  case 6: /* 下移到"重新自检" */
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.sub_sel == 1U) { s_ly = 128U; pre_stage_n = 7; pre_mark = tick; }
    else { s_ly = 208U; }
    break;
  case 7:
    s_btn = BTNS_NONE; s_ly = 128U;
    if ((uint32_t)(tick - pre_mark) >= 30U) { pre_stage_n = 8; pre_mark = tick; }
    break;
  case 8: /* L1 → 确认页 */
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_CONFIRM) { pre_stage_n = 9; pre_mark = tick; }
    break;
  case 9:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - pre_mark) >= 30U) { pre_stage_n = 10; pre_mark = tick; }
    break;
  case 10: /* 焦点移到"开始" */
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.confirm_focus == 1U) { s_ly = 128U; pre_stage_n = 11; pre_mark = tick; }
    else { s_ly = 208U; }
    break;
  case 11:
    s_btn = BTNS_NONE; s_ly = 128U;
    if ((uint32_t)(tick - pre_mark) >= 30U) { pre_stage_n = 12; pre_mark = tick; }
    break;
  case 12: /* L1 → 自检启动 */
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_RUN) { pre_stage_n = 13; pre_mark = tick; }
    break;
  case 13: /* 释放；服务开跑（state[0] 先变 1），记总时长起点 */
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.selfcheck_state[0] != 0U) {
      pre_sc_t0 = tick;
      pre_stage_n = 14;
    }
    break;
  case 14: /* 等服务跑完（physics 注入 → 全过；no_inject → 全 NO FB） */
    ui_snapshot_get(&snap);
    if (snap.action_busy == 0U && snap.selfcheck_state[0] != 1U &&
        snap.selfcheck_state[1] != 1U && snap.selfcheck_state[2] != 1U &&
        snap.selfcheck_state[3] != 1U) {
      pre_sc_done = tick;
      pre_passed = (snap.selfcheck_state[0] == 2U &&
                    snap.selfcheck_state[1] == 2U &&
                    snap.selfcheck_state[2] == 2U &&
                    snap.selfcheck_state[3] == 2U);
      pre_stage_n = 15; pre_mark = tick;
    }
    break;
  case 15: /* 页面落到结果页（晚服务完成一拍） */
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_RESULT) { pre_stage_n = 16; pre_mark = tick; }
    else {
      assert(snap.page == UI_PAGE_CHECK_RUN);
      assert((uint32_t)(tick - pre_mark) < 200U);
    }
    break;
  case 16: /* 结果页 R1 → 品牌页 */
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_BRAND) { pre_stage_n = 17; pre_mark = tick; }
    break;
  case 17:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - pre_mark) >= 30U) { pre_stage_n = 18; pre_mark = tick; }
    break;
  case 18: /* 品牌页 R1 → 主菜单，把 menu_sel 复位到"驾驶" */
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { pre_stage_n = 19; pre_mark = tick; }
    break;
  case 19:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - pre_mark) >= 30U) { pre_stage_n = 20; pre_mark = tick; }
    break;
  case 20:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 0U) { s_lx = 128U; pre_stage_n = 21; pre_mark = tick; }
    else { s_lx = 48U; }
    break;
  case 21:
    s_btn = BTNS_NONE; s_lx = 128U;
    if ((uint32_t)(tick - pre_mark) >= 30U) { pre_stage_n = 22; pre_mark = tick; }
    break;
  case 22: /* 菜单页 R1 → 品牌页 */
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_BRAND) { pre_stage_n = 23; pre_mark = tick; }
    break;
  case 23:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - pre_mark) >= 30U) { pre_done = 1; }
    break;
  default:
    break;
  }
}

/* 场景 0/1 共用：自检结束后进驾驶页，验证回中门槛。
 * 口径与 Ctrl 一致：出杆会打断武装计时，必须"推满(未武装零输出) →
 * 松杆回中 ≥100 ms 武装 → 再推满"才能看到出力。
 * dg_expect=1：武装后四轮 8400；dg_expect=0：武装后依旧零输出（未过自检）。 */
static int dg_stage, dg_expect;
static uint32_t dg_mark;

static void drive_gate_step(void)
{
  switch (dg_stage) {
  case 0: /* R1 → 主菜单 */
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { dg_stage = 1; dg_mark = tick; }
    break;
  case 1:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - dg_mark) >= 30U) { dg_stage = 2; dg_mark = tick; }
    break;
  case 2: /* L1（menu_sel==0）→ 驾驶页 */
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_DRIVE) { dg_stage = 3; dg_mark = tick; }
    break;
  case 3:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - dg_mark) >= 30U) { dg_stage = 4; dg_mark = tick; }
    break;
  case 4: /* 推满前进：回中武装满 100 ms 前必须零输出 */
    s_ly = 0U;
    ui_snapshot_get(&snap);
    if ((uint32_t)(tick - dg_mark) >= 60U && (uint32_t)(tick - dg_mark) < 100U) {
      assert(snap.drive_permitted == 0U);
      assert(htim8.compare[0] == 0U && htim8.compare[1] == 0U);
      assert(htim8.compare[2] == 0U && htim8.compare[3] == 0U);
    }
    if ((uint32_t)(tick - dg_mark) >= 120U) { dg_stage = 5; dg_mark = tick; }
    break;
  case 5: /* 松杆回中 ≥100 ms：武装 */
    s_ly = 128U;
    if ((uint32_t)(tick - dg_mark) >= 200U) { dg_stage = 6; dg_mark = tick; }
    break;
  case 6: /* 再推满：武装后按 expect 出力/不出力 */
    s_ly = 0U;
    if ((uint32_t)(tick - dg_mark) >= 60U) {
      ui_snapshot_get(&snap);
      assert(snap.drive_permitted == (dg_expect ? 1U : 0U));
      for (int i = 0; i < 4; i++) {
        assert(htim8.compare[i] == (dg_expect ? 8400U : 0U));
      }
      report(1, dg_expect ? "drive gate ok" : "drive blocked / gate ok");
    }
    break;
  default:
    break;
  }
}

/* ---- 各场景脚本 ---------------------------------------------------------- */

static void scenario_0(void)
{
  if (!pre_done) { pre_selfcheck_step(); return; }
  /* 自检总时长：服务是纯时间推进，8 段 × 700 ms = 5600 ms。 */
  if (st == -1) {
    assert(pre_passed);
    assert(pre_sc_done - pre_sc_t0 >= 5500U);
    assert(pre_sc_done - pre_sc_t0 <= 5700U);
    assert_selfcheck_segments();
    assert_battery_unconfigured();
    dg_expect = 1;
    st = 20; st_since = tick;
  }
  drive_gate_step();
}

static void scenario_1(void)
{
  if (!pre_done) { pre_selfcheck_step(); return; }
  if (st == -1) {
    ui_snapshot_t s2;

    assert(pre_passed == 0);
    ui_snapshot_get(&s2);
    for (int i = 0; i < 4; i++) assert(s2.selfcheck_state[i] == 3U); /* NO FB */
    assert(s2.fault_mask != 0U); /* 缺反馈 → 运行反馈监测锁存 */
    dg_expect = 0;
    st = 20; st_since = tick;
  }
  drive_gate_step(); /* 武装后依旧零输出：本次上电未通过自检 */
}

static void scenario_2(void)
{
  if (!pre_done) { pre_selfcheck_step(); return; }
  if (st == -1) {
    ui_snapshot_t s2;

    assert(pre_passed == 0);
    ui_snapshot_get(&s2);
    /* D 轮反转段被冻结 → REV ERR（state 3）；其余三轮通过 */
    assert(s2.selfcheck_state[0] == 2U && s2.selfcheck_state[1] == 2U &&
           s2.selfcheck_state[2] == 2U && s2.selfcheck_state[3] == 3U);
    assert(s2.selfcheck_passed == 0U);
    report(1, "reverse-error result ok");
  }
}

static void scenario_3(void)
{
  if (script_idle) return; /* 主循环守望负责收尾 */
  switch (st) {
  case -1:
    st = 100; st_since = tick;
    break;
  case 100:
    s_btn = BTNS_NONE; s_lx = 128U; s_ly = 128U;
    if (cn >= 40) { st = 0; st_since = tick; }
    break;
  case 0:
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { st = 1; st_since = tick; }
    break;
  case 1:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 2; st_since = tick; }
    break;
  case 2:
    s_btn = BTNS_NONE; s_lx = 208U;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 1U) { st = 3; st_since = tick; }
    break;
  case 3:
    s_btn = BTNS_NONE; s_lx = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 4; st_since = tick; }
    break;
  case 4:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { st = 5; st_since = tick; }
    break;
  case 5:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 6; st_since = tick; }
    break;
  case 6:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.sub_sel == 1U) { s_ly = 128U; st = 7; st_since = tick; }
    else { s_ly = 208U; }
    break;
  case 7:
    s_btn = BTNS_NONE; s_ly = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 8; st_since = tick; }
    break;
  case 8:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_CONFIRM) { st = 9; st_since = tick; }
    break;
  case 9:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 10; st_since = tick; }
    break;
  case 10:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.confirm_focus == 1U) { s_ly = 128U; st = 11; st_since = tick; }
    else { s_ly = 208U; }
    break;
  case 11:
    s_btn = BTNS_NONE; s_ly = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 12; st_since = tick; }
    break;
  case 12:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_RUN) { st = 13; st_since = tick; }
    break;
  case 13: /* 服务跑起来后切入标定模式 → 下一拍全轮 SKIP */
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.selfcheck_state[0] == 1U) {
      drive_calibration_mode = 1U;
      mark = tick;
      script_idle = 1;
    }
    break;
  default:
    break;
  }
}

static void scenario_4(void)
{
  switch (st) {
  case -1: st = 100; st_since = tick; break;
  case 100:
    s_btn = BTNS_NONE; s_lx = 128U; s_ly = 128U;
    if (cn >= 40) { st = 0; st_since = tick; }
    break;
  case 0:
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { st = 1; st_since = tick; }
    break;
  case 1:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 2; st_since = tick; }
    break;
  case 2:
    s_btn = BTNS_NONE; s_lx = 208U;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 1U) { st = 3; st_since = tick; }
    break;
  case 3:
    s_btn = BTNS_NONE; s_lx = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 4; st_since = tick; }
    break;
  case 4:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { st = 5; st_since = tick; }
    break;
  case 5:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 6; st_since = tick; }
    break;
  case 6:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.sub_sel == 1U) { s_ly = 128U; st = 7; st_since = tick; }
    else { s_ly = 208U; }
    break;
  case 7:
    s_btn = BTNS_NONE; s_ly = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 8; st_since = tick; }
    break;
  case 8:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_CONFIRM) { st = 9; st_since = tick; }
    break;
  case 9:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 10; st_since = tick; }
    break;
  case 10:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.confirm_focus == 1U) { s_ly = 128U; st = 11; st_since = tick; }
    else { s_ly = 208U; }
    break;
  case 11:
    s_btn = BTNS_NONE; s_ly = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 12; st_since = tick; }
    break;
  case 12:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_RUN) { st = 13; st_since = tick; }
    break;
  case 13: /* 等 A 轮完成、B 轮运行，然后推杆越区取消 */
    s_btn = BTNS_NONE; s_ly = 128U;
    ui_snapshot_get(&snap);
    if (snap.selfcheck_state[0] == 2U && snap.selfcheck_state[1] == 1U) {
      mark = tick;
      s_ly = 0U;
      st = 14; st_since = tick;
    }
    break;
  case 14: /* 取消当拍清输出，页面离开 CHECK_RUN */
    s_ly = 0U;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_CONFIRM) {
      assert(snap.selfcheck_state[0] == 2U); /* 已完成轮结果保留 */
      assert(snap.selfcheck_state[1] == 3U); /* 受试轮定格失败 */
      assert(snap.selfcheck_state[2] == 0U && snap.selfcheck_state[3] == 0U);
      for (int i = 0; i < 4; i++) assert(htim8.compare[i] == 0U);
      st = 15; st_since = tick;
    }
    break;
  case 15: /* 取消后不再有任何输出；结果窗口已开 */
    s_ly = 128U;
    ui_snapshot_get(&snap);
    if ((uint32_t)(tick - st_since) >= 300U) {
      assert(snap.result_window_open == 1U);
      assert(powered_ms[2] == 0U && powered_ms[3] == 0U);   /* C/D 没上过电 */
      assert(powered_ms[0] >= 900U);                        /* A 两段都跑完 */
      assert(powered_ms[1] < 450U);                         /* B 被截断 */
      report(1, "cancel keeps finished results / stops output");
    }
    break;
  default:
    break;
  }
}

/* 场景 5/6 的自检前置复用 pre_selfcheck_step（no_inject=0 → 全过）。 */

static void scenario_5(void)
{
  if (!pre_done) { pre_selfcheck_step(); return; }
  switch (st) {
  case -1:
    assert(pre_passed);
    st = 20; st_since = tick;
    break;
  case 20: /* 驾驶页推满 + 冻结反馈 → NO FB 锁存 */
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { st = 21; st_since = tick; }
    break;
  case 21:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 22; st_since = tick; }
    break;
  case 22:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_DRIVE) { st = 23; st_since = tick; }
    break;
  case 23:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 24; st_since = tick; }
    break;
  case 24: /* 推满：武装前确认零输出，然后松杆等待武装 */
    no_inject = 1U;
    s_ly = 0U;
    ui_snapshot_get(&snap);
    if ((uint32_t)(tick - st_since) >= 60U && (uint32_t)(tick - st_since) < 100U) {
      for (int i = 0; i < 4; i++) assert(htim8.compare[i] == 0U);
    }
    if ((uint32_t)(tick - st_since) >= 120U) { s_ly = 128U; st = 61; st_since = tick; }
    break;
  case 61: /* 回中 ≥100 ms：武装 */
    s_ly = 128U;
    if ((uint32_t)(tick - st_since) >= 200U) { s_ly = 0U; st = 62; st_since = tick; }
    break;
  case 62: /* 武装后出力 + 反馈冻结 → NO FB 锁存 */
    s_ly = 0U;
    ui_snapshot_get(&snap);
    if (snap.fault_mask != 0U) {
      for (int i = 0; i < 4; i++) assert(htim8.compare[i] == 0U);
      assert(!(gpiob.ODR & GPIO_PIN_12)); /* 锁存后停止流程关使能 */
      s_ly = 128U;
      st = 25; st_since = tick;
    }
    break;
  case 25: /* 锁存后重新自检（失败轮）：R1 → 菜单 → 子菜单"重新自检" */
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { st = 26; st_since = tick; }
    break;
  case 26:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 27; st_since = tick; }
    break;
  case 27: /* 前置把 menu_sel 复位成 0 了：先右移选"检查与标定" */
    s_btn = BTNS_NONE; s_lx = 208U;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 1U) { s_lx = 128U; st = 65; st_since = tick; }
    break;
  case 65:
    s_btn = BTNS_NONE; s_lx = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 28; st_since = tick; }
    break;
  case 28:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { st = 66; st_since = tick; }
    break;
  case 66:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 29; st_since = tick; }
    break;
  case 29:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.sub_sel == 1U) { s_ly = 128U; st = 30; st_since = tick; }
    else { s_ly = 208U; }
    break;
  case 30:
    s_btn = BTNS_NONE; s_ly = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 31; st_since = tick; }
    break;
  case 31:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_CONFIRM) { st = 32; st_since = tick; }
    break;
  case 32:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 33; st_since = tick; }
    break;
  case 33:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.confirm_focus == 1U) { s_ly = 128U; st = 34; st_since = tick; }
    else { s_ly = 208U; }
    break;
  case 34:
    s_btn = BTNS_NONE; s_ly = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 35; st_since = tick; }
    break;
  case 35: /* 发起失败的自检（反馈仍冻结） */
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_RUN) { st = 36; st_since = tick; }
    break;
  case 36:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.action_busy == 0U && snap.selfcheck_state[0] == 3U &&
        snap.selfcheck_state[1] == 3U && snap.selfcheck_state[2] == 3U &&
        snap.selfcheck_state[3] == 3U) {
      assert(snap.fault_mask != 0U);  /* 失败不清已锁存故障 */
      assert(snap.selfcheck_passed == 0U);
      st = 37; st_since = tick;
    }
    break;
  case 37: /* 结果页 R1 → 品牌页 → 菜单，再跑一轮全过的自检 */
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 38; st_since = tick; }
    break;
  case 38:
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_BRAND) { st = 39; st_since = tick; }
    break;
  case 39:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 40; st_since = tick; }
    break;
  case 40:
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { st = 41; st_since = tick; }
    break;
  case 41:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 42; st_since = tick; }
    break;
  case 42:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { st = 43; st_since = tick; }
    break;
  case 43:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 44; st_since = tick; }
    break;
  case 44:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.sub_sel == 1U) { s_ly = 128U; st = 45; st_since = tick; }
    else { s_ly = 208U; }
    break;
  case 45:
    s_btn = BTNS_NONE; s_ly = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 46; st_since = tick; }
    break;
  case 46:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_CONFIRM) { st = 47; st_since = tick; }
    break;
  case 47:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 48; st_since = tick; }
    break;
  case 48:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.confirm_focus == 1U) { s_ly = 128U; st = 49; st_since = tick; }
    else { s_ly = 208U; }
    break;
  case 49:
    s_btn = BTNS_NONE; s_ly = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 50; st_since = tick; }
    break;
  case 50: /* 发起通过的自检（恢复反馈注入） */
    no_inject = 0U;
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_RUN) { st = 51; st_since = tick; }
    break;
  case 51:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.action_busy == 0U && snap.selfcheck_state[0] == 2U &&
        snap.selfcheck_state[1] == 2U && snap.selfcheck_state[2] == 2U &&
        snap.selfcheck_state[3] == 2U) {
      assert(snap.fault_mask == 0U); /* 全过才清锁存 */
      st = 52; st_since = tick;
    }
    break;
  case 52: /* 回到驾驶页，验证"全过后仍须回中门槛才出力" */
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_BRAND) { st = 53; st_since = tick; }
    break;
  case 53:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 54; st_since = tick; }
    break;
  case 54:
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { st = 55; st_since = tick; }
    break;
  case 55:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 56; st_since = tick; }
    break;
  case 56:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 0U) { s_lx = 128U; st = 57; st_since = tick; }
    else { s_lx = 48U; }
    break;
  case 57:
    s_btn = BTNS_NONE; s_lx = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 58; st_since = tick; }
    break;
  case 58:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_DRIVE) { st = 59; st_since = tick; }
    break;
  case 59:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 60; st_since = tick; }
    break;
  case 60: /* 推满：武装前零输出，然后松杆武装 */
    s_ly = 0U;
    ui_snapshot_get(&snap);
    if ((uint32_t)(tick - st_since) >= 60U && (uint32_t)(tick - st_since) < 100U) {
      assert(snap.drive_permitted == 0U);
      for (int i = 0; i < 4; i++) assert(htim8.compare[i] == 0U);
    }
    if ((uint32_t)(tick - st_since) >= 120U) { s_ly = 128U; st = 63; st_since = tick; }
    break;
  case 63: /* 回中 ≥100 ms：武装 */
    s_ly = 128U;
    if ((uint32_t)(tick - st_since) >= 200U) { s_ly = 0U; st = 64; st_since = tick; }
    break;
  case 64: /* 再推满：全过 + 回中门槛后恢复出力（300 ms 宽限内断言） */
    s_ly = 0U;
    if ((uint32_t)(tick - st_since) >= 60U) {
      ui_snapshot_get(&snap);
      assert(snap.drive_permitted == 1U);
      for (int i = 0; i < 4; i++) assert(htim8.compare[i] == 8400U);
      report(1, "latch / recheck / neutral gate ok");
    }
    break;
  default:
    break;
  }
}

static void scenario_6(void)
{
  if (!pre_done) { pre_selfcheck_step(); return; }
  switch (st) {
  case -1:
    assert(pre_passed);
    st = 30; st_since = tick;
    break;
  case 30: /* R1 → 菜单 */
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { st = 31; st_since = tick; }
    break;
  case 31:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 32; st_since = tick; }
    break;
  case 32:
    s_btn = BTNS_NONE; s_lx = 208U;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 1U) { st = 33; st_since = tick; }
    break;
  case 33:
    s_btn = BTNS_NONE; s_lx = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 34; st_since = tick; }
    break;
  case 34:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { st = 35; st_since = tick; }
    break;
  case 35:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 36; st_since = tick; }
    break;
  case 36: /* 下移到"回到初始位" */
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.sub_sel == 2U) { s_ly = 128U; st = 37; st_since = tick; }
    else { s_ly = 208U; }
    break;
  case 37:
    s_btn = BTNS_NONE; s_ly = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 38; st_since = tick; }
    break;
  case 38: /* L1 发起归位（无参考） */
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_HOME) { st = 39; st_since = tick; }
    break;
  case 39: /* 同一次 L1 继续按住：请求不得被取消 */
    s_btn = BTNS_L1;
    if ((uint32_t)(tick - st_since) >= 60U) { st = 40; st_since = tick; }
    break;
  case 40: /* 释放；无参考 → NO_REF，一轮都不许转 */
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if ((uint32_t)(tick - st_since) >= 200U) {
      wheel_home_output_t hs;

      wheel_home_get_status(&hs);
      assert(hs.reference_valid == 0U);
      assert(hs.result == WHEEL_HOME_RESULT_NO_REF);
      assert(hs.mode != WHEEL_HOME_RUNNING);
      for (int i = 0; i < 4; i++) assert(htim8.compare[i] == 0U);
      st = 41; st_since = tick;
    }
    break;
  case 41: /* R1 回子菜单，进手动标定页 */
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { st = 42; st_since = tick; }
    break;
  case 42:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 43; st_since = tick; }
    break;
  case 43:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.sub_sel == 3U) { s_ly = 128U; st = 44; st_since = tick; }
    else { s_ly = 208U; }
    break;
  case 44:
    s_btn = BTNS_NONE; s_ly = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 45; st_since = tick; }
    break;
  case 45:
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MANUAL_CAL) { st = 46; st_since = tick; }
    break;
  case 46: /* 进页后必须先见 SELECT 释放 */
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 60U) { st = 47; st_since = tick; mark = tick; }
    break;
  case 47: /* 第一次长按：1 s 后断连 300 ms，继续按住到 3 s */
    s_btn = BTNS_SEL;
    if ((uint32_t)(tick - mark) >= 1000U && bad_until == 0U) {
      bad_until = tick + 300U;
    }
    if ((uint32_t)(tick - mark) >= 3000U) {
      s_btn = BTNS_NONE;
      st = 48; st_since = tick;
    }
    break;
  case 48: /* 断连拒绝：跨断连的长按不得写出参考 */
    if ((uint32_t)(tick - st_since) >= 300U) {
      wheel_home_output_t hs;

      wheel_home_get_status(&hs);
      assert(hs.reference_valid == 0U);
      st = 49; st_since = tick;
    }
    break;
  case 49: /* 静止等待（编码器无注入）+ 再武装，然后新长按 2 s */
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 500U) { st = 50; st_since = tick; mark = tick; }
    break;
  case 50:
    s_btn = BTNS_SEL;
    if ((uint32_t)(tick - mark) >= 2300U) {
      s_btn = BTNS_NONE;
      st = 51; st_since = tick;
    }
    break;
  case 51: /* 连续静止 + 新长按 → 记录成功 */
    if ((uint32_t)(tick - st_since) >= 100U) {
      wheel_home_output_t hs;

      wheel_home_get_status(&hs);
      assert(hs.reference_valid == 1U);
      mark = tick;
      st = 52; st_since = tick;
    }
    break;
  case 52: /* 长按后的释放不得触发短按归位 */
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 500U) {
      wheel_home_output_t hs;

      wheel_home_get_status(&hs);
      assert(hs.mode != WHEEL_HOME_RUNNING);
      for (int i = 0; i < 4; i++) assert(htim8.compare[i] == 0U);
      st = 53; st_since = tick;
    }
    break;
  case 53: /* 给 C/D 各注入 1/8 圈（分两拍，每拍 < 32768） */
    s_btn = BTNS_NONE;
    if (cn % 2 == 0) {
      htim5.counter = (uint16_t)(htim5.counter - 13437U); /* C 前向为负 raw */
      htim4.counter = (uint16_t)(htim4.counter + 13397U); /* D 前向为正 raw */
      static int inj;
      if (++inj >= 4) { st = 54; st_since = tick; }
    }
    break;
  case 54: /* R1 → 子菜单，选"回到初始位"，L1 发起归位 */
    s_btn = BTNS_R1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { st = 55; st_since = tick; }
    break;
  case 55:
    s_btn = BTNS_NONE;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 56; st_since = tick; }
    break;
  case 56:
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.sub_sel == 2U) { s_ly = 128U; st = 57; st_since = tick; }
    else { s_ly = 48U; }
    break;
  case 57:
    s_btn = BTNS_NONE; s_ly = 128U;
    if ((uint32_t)(tick - st_since) >= 30U) { st = 58; st_since = tick; }
    break;
  case 58: /* L1 发起归位（有参考） */
    s_btn = BTNS_L1;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_HOME) { st = 59; st_since = tick; }
    break;
  case 59: /* 同一次 L1 按住 60 ms：pending 不得被取消 */
    s_btn = BTNS_L1;
    if ((uint32_t)(tick - st_since) >= 60U) { st = 60; st_since = tick; }
    break;
  case 60: /* 释放 → 放行门 100 ms → RUNNING → C/D 出力 */
    s_btn = BTNS_NONE;
    ui_snapshot_get(&snap);
    if (snap.action_busy != 0U) {
      int moved = 0;
      for (int i = 0; i < 4; i++) {
        assert(htim8.compare[i] <= WHEEL_HOME_MAX_PULSE);
        if (htim8.compare[i] != 0U) moved = 1;
      }
      if (moved) {
        report(1, "home/cal/refusal/gate ok");
      }
    }
    break;
  default:
    break;
  }
}

static void scenario_step(void)
{
  check_timeout();
  switch (scenario) {
  case 0: scenario_0(); break;
  case 1: scenario_1(); break;
  case 2: scenario_2(); break;
  case 3: scenario_3(); break;
  case 4: scenario_4(); break;
  case 5: scenario_5(); break;
  case 6: scenario_6(); break;
  default: report(0, "unknown scenario"); break;
  }
}

/* 标定模式跳过场景的收尾守望（脚本停摆后由主循环调用）。 */
static void scenario_3_watch(void)
{
  ui_snapshot_t s2;

  ui_snapshot_get(&s2);
  if (script_idle && s2.action_busy == 0U &&
      s2.selfcheck_state[0] != 1U && s2.selfcheck_state[1] != 1U &&
      s2.selfcheck_state[2] != 1U && s2.selfcheck_state[3] != 1U) {
    assert((uint32_t)(tick - mark) < 1000U);        /* 下一拍即落地 */
    assert(powered_ms[0] < 100U);                   /* A 只上了一个周期的电 */
    assert(powered_ms[1] == 0U && powered_ms[2] == 0U && powered_ms[3] == 0U);
    assert(s2.selfcheck_state[0] == 0U && s2.selfcheck_state[1] == 0U &&
           s2.selfcheck_state[2] == 0U && s2.selfcheck_state[3] == 0U);
    assert(s2.selfcheck_passed == 0U);
    drive_calibration_mode = 0U;
    report(1, "calibration-mode skip / no output");
  }
}

static void power_runner(void)
{
  for (;;) {
    power_monitor_step(HAL_GetTick());
    osDelay(50);
  }
}

static void ctrl_runner(void) { StartCtrlTask(NULL); abort(); }
static void ui_runner(void) { boot_ui_run(); abort(); }

int main(int argc, char **argv)
{
  scenario = argc > 1 ? atoi(argv[1]) : 0;
  if (scenario == 1) no_inject = 1U; /* 缺反馈场景：编码器全程冻结 */

  for (int n = 0; n < 3; ++n) {
    if (getcontext(&tasks[n].context) != 0) return 2;
    tasks[n].context.uc_stack.ss_sp = tasks[n].stack;
    tasks[n].context.uc_stack.ss_size = sizeof tasks[n].stack;
    tasks[n].context.uc_link = &scheduler;
    makecontext(&tasks[n].context,
                n == 0 ? ctrl_runner : (n == 1 ? ui_runner : power_runner), 0);
    tasks[n].wake = 0;
  }
  while (!finished && tick < 150000U) {
    int selected = -1;
    for (int n = 0; n < 3; ++n) {
      if (tasks[n].wake <= tick) { selected = n; break; }
    }
    if (selected < 0) {
      uint32_t next = tasks[0].wake;
      for (int n = 1; n < 3; ++n) if (tasks[n].wake < next) next = tasks[n].wake;
      advance(next - tick);
      continue;
    }
    current_task = selected;
    if (swapcontext(&scheduler, &tasks[selected].context) != 0) return 2;
    if (scenario == 3) scenario_3_watch();
    if (tick > 130000U && !finished) report(0, "global timeout");
  }
  if (!finished) report(0, "scheduler exhausted");
  return finished ? 0 : 1;
}
