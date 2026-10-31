/* Real task: driver wake, 200 ms link deadline despite delayed polls,
 * reconnection, a HAL tick rollover, calibration sleep, and the homing
 * arbitration path end to end (rear wheels read TIM5/TIM4). */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "ps2_gamepad.h"
#include "boot_ui.h"
#include "drive_ctrl.h"
#include "power_monitor.h"
#include "ui_model.h"
#include "ui_render.h"
#include "wheel_home.h"
GPIO_TypeDef gpioa, gpiob, gpioc, gpiod, gpioe;
TIM_HandleTypeDef htim2, htim3, htim4, htim5, htim8;
volatile uint8_t boot_ui_ready=1;

/* --- power_monitor 桩 --------------------------------------------------- */
/* Ctrl 测试不链接 power_monitor.c：用可编程快照驱动 vdd_ok 门控
 * （菜单自检的前置条件之一），ADC 行为由 tools/test_power_monitor.c 覆盖。 */
static power_snapshot_t s_pm_stub;
void power_monitor_init(void) { memset(&s_pm_stub, 0, sizeof s_pm_stub); }
void power_monitor_step(uint32_t now_ms) { (void)now_ms; }
const power_snapshot_t *power_monitor_get(void) { return &s_pm_stub; }
static void pm_set_vdd_ok(uint8_t ok)
{
  s_pm_stub.vdd_valid = ok;
  s_pm_stub.vdd_mv = (ok != 0U) ? 3300U : 0U;
}

/* 自检 UX 的桩：Ctrl 任务测试只关心电机输出，不关心 OLED 页面。 */
void boot_ui_manual_begin(void) {}
uint8_t boot_ui_manual_run(void) { return 0U; }
uint8_t boot_ui_manual_active(void) { return 0U; }
uint8_t boot_ui_manual_take_record_request(void) { return 0U; }
void boot_ui_manual_ack(uint8_t success) { (void)success; }
volatile uint8_t boot_ui_selfcheck_passed=1;
volatile uint32_t boot_ui_selfcheck_seq=1;
/* 本测试不链接 boot_ui.c：直接让电机归 Ctrl，覆盖"已移交"之后的驾驶逻辑。
 * 自检独占期的行为由 tools/test_boot_ctrl_integration.c scenario=3 覆盖。 */
volatile boot_ui_motor_owner_t boot_ui_motor_owner=BOOT_UI_OWNER_CTRL;
volatile uint8_t boot_ui_drive_enabled=1;
volatile uint32_t boot_ui_handoff_seq=1;
/* --- fault-injection scenario (motor_health) --------------------------- */
/* W 键让四轮满输出、编码器纹丝不动：宽限 300 ms + 窗口 300 ms 后必须锁存，
 * 输出清零；之后即使反馈恢复正常也不许自动恢复，只有重新自检通过才清。 */
static int stop_test;
static int fault_test;
static uint32_t fault_t0;
static uint32_t fault_frames;
static int verbose_test;
static int reconnect_seen;
static uint8_t current_ly;
static int fault_cleared;

static jmp_buf done;
static int phase, reconnect_test, current_phase;
static uint32_t tick, first_poll_tick;

/* --- ui snapshot scenario ---------------------------------------------- */
/* 页面模型 + 驾驶页仲裁：品牌页起 → R1 进菜单 → L1 进驾驶页 → 满前进。
 * 断言：回中武装满 100 ms 前 drive_permitted==0 且输出全零；武装后四轮
 * compare=8400（双轴混合 v=+8400, t=0）；驾驶中按 R1 下一周期输出全零；
 * 编码器注入约 1 圈后 wheel_milliturns[0] ≈ ±1000（±5%），切页后计数连续。 */
static int ui_test;
static int ustage;
static uint32_t un;      /* ui 场景内的 ps2_poll 帧计数 */
static uint32_t uenc;    /* 剩余编码器注入帧数（每帧 +21320 计数） */
static int64_t u_mt_prev;/* 上一次记录的 milliturns，用于连续性断言 */

/* --- homing scenario state --------------------------------------------- */
static int home_test;
static int hstage;
static int hcycles;
static uint32_t hmark;
static uint16_t hbuttons = 0xFFFFU;
static uint8_t hly = 128U;
static uint8_t hlx = 128U;

void StartCtrlTask(void *);
void boot_ui_run(void) {}
void MX_USB_HOST_Init(void) {}
void *osThreadNew(void (*fn)(void *), void *arg, const osThreadAttr_t *attr) {(void)fn;(void)arg;(void)attr;return 0;}
/* 记录四路 PWM 通道是否真的开着。之前这里是空桩，所以"只写 CCR 不启动
 * 通道"这种 bug 在主机测试里完全测不出来，直到上板才发现推杆无反应。
 * 下面的驱动断言会同时检查"通道已启动"和"SLP 使能"。 */
/* 通道号是 0/4/8/12，低位移位后再 OR，才能区分四个通道。 */
static uint32_t pwm_ccer;
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t c) {(void)t;pwm_ccer|=(1U<<(c/4));return 0;}
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *t,uint32_t c) {(void)t;pwm_ccer&=~(1U<<(c/4));return 0;}
static int pwm_all_running(void){ return (pwm_ccer & 0xFU)==0xFU; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint32_t b,uint32_t v){if(v)p->ODR|=b;else p->ODR&=~b;}
uint32_t HAL_GetTick(void){return tick;}
static void active(void){
 /* Physical B uses the original polarity; loose wiring caused the fault. */
 assert(!(gpioc.ODR & GPIO_PIN_0));
 assert((gpiob.ODR&(GPIO_PIN_12|GPIO_PIN_14))==(GPIO_PIN_12|GPIO_PIN_14));
 /* 光写 CCR 不够：通道没启动时同样的 CCR 一个脉冲都不产生。 */
 assert(pwm_all_running());
 assert(gpiod.ODR&GPIO_PIN_12);assert(gpioa.ODR&GPIO_PIN_8);
 for(int i=0;i<4;i++)assert(htim8.compare[i]==8400);
}
static void asleep(void){
 assert(!(gpiob.ODR&(GPIO_PIN_12|GPIO_PIN_14)));
 assert(!(gpiod.ODR&GPIO_PIN_12));assert(!(gpioa.ODR&GPIO_PIN_8));
 for(int i=0;i<4;i++)assert(htim8.compare[i]==0);
}

/* Homing: navigate the menu to the manual-calibration page, long-press to
 * record the reference (manual_calib_page gated), then request homing from
 * the submenu with L1 — the same L1 press must not cancel it — and check a
 * quarter turn on the two rear encoders produces corrective pulses only on
 * those two channels. */
static void home_script(void){
  wheel_home_output_t st;
  ui_snapshot_t snap;

  hcycles++;
  switch(hstage){
  case 0: /* 回中武装预热：等链路再武装完成、状态机 MANUAL、页面 BRAND */
    hbuttons=0xFFFFU; hly=128U; hlx=128U;
    if(hcycles>=40){
      ui_snapshot_get(&snap);
      wheel_home_get_status(&st);
      if(st.mode==WHEEL_HOME_MANUAL && snap.page==UI_PAGE_BRAND){
        assert(htim8.compare[0]==0U && htim8.compare[1]==0U);
        assert(htim8.compare[2]==0U && htim8.compare[3]==0U);
        hstage=1; hmark=tick;
      }
    }
    break;
  case 1: /* R1：品牌页 → 主菜单 */
    hbuttons=0xF7FFU;
    ui_snapshot_get(&snap);
    if(snap.page==UI_PAGE_MENU){ hstage=2; hmark=tick; }
    break;
  case 2: /* 释放 R1 ≥3 稳定帧 */
    hbuttons=0xFFFFU;
    if((uint32_t)(tick-hmark)>=30U){ hstage=3; }
    break;
  case 3: /* 摇杆右移：选中"检查与标定" */
    hbuttons=0xFFFFU; hlx=208U;
    ui_snapshot_get(&snap);
    if(snap.menu_sel==1U){ hstage=4; hmark=tick; }
    break;
  case 4: /* 摇杆回中 */
    hbuttons=0xFFFFU; hlx=128U;
    if((uint32_t)(tick-hmark)>=30U){ hstage=5; }
    break;
  case 5: /* L1：主菜单 → 子菜单 */
    hbuttons=0xFBFFU;
    ui_snapshot_get(&snap);
    if(snap.page==UI_PAGE_SUBMENU){ hstage=6; hmark=tick; }
    break;
  case 6: /* 释放 L1 */
    hbuttons=0xFFFFU;
    if((uint32_t)(tick-hmark)>=30U){ hstage=7; }
    break;
  case 7: /* 摇杆下移到第 4 项（手动标定）：按住等 400/180ms 连移走完 */
    hbuttons=0xFFFFU; hly=208U;
    ui_snapshot_get(&snap);
    if(snap.sub_sel==3U){ hstage=8; hmark=tick; }
    break;
  case 8: /* 摇杆回中 */
    hbuttons=0xFFFFU; hly=128U;
    if((uint32_t)(tick-hmark)>=30U){ hstage=9; }
    break;
  case 9: /* L1：子菜单 → 手动标定页（此后 Ctrl 喂 manual_calib_page=1） */
    hbuttons=0xFBFFU;
    ui_snapshot_get(&snap);
    if(snap.page==UI_PAGE_MANUAL_CAL){ hstage=10; hmark=tick; }
    break;
  case 10: /* 释放 L1 */
    hbuttons=0xFFFFU;
    if((uint32_t)(tick-hmark)>=30U){ hstage=11; hmark=tick; }
    break;
  case 11: /* 长按 SELECT 2 s：手动标定页记录参考 */
    hbuttons=0xFFFEU;
    if((uint32_t)(tick-hmark)>=2500U){ hstage=12; hmark=tick; }
    break;
  case 12: /* 释放，确认参考已记录（驾驶页长按在此门控下不再写参考） */
    hbuttons=0xFFFFU;
    if((uint32_t)(tick-hmark)>=100U){
      wheel_home_get_status(&st);
      assert(st.reference_valid==1U);
      /* 后轮编码器：C 读 TIM5、D 读 TIM4，各转 1/4 圈。
       * 16 位计数一次跳变必须 < 32768，故分两拍累加。 */
      htim5.counter=20000U; htim4.counter=20000U;
      hstage=13; hcycles=0;
    }
    break;
  case 13: /* 后续几拍补足到 1/4 圈（每拍 < 32768） */
    hbuttons=0xFFFFU;
    if(hcycles==1){htim5.counter=40000U; htim4.counter=40000U;}
    else if(hcycles>=3){
      htim5.counter=(uint32_t)(WHEEL_RR_CPR/4U);
      htim4.counter=(uint32_t)(WHEEL_LR_CPR/4U);
      hstage=14; hmark=tick; hcycles=0;
    }
    break;
  case 14: /* 等采样看到新位置（MANUAL、四轮零输出） */
    hbuttons=0xFFFFU;
    if(hcycles>=3){
      wheel_home_get_status(&st);
      assert(st.mode==WHEEL_HOME_MANUAL);
      assert(htim8.compare[0]==0U && htim8.compare[2]==0U);
      hstage=15; hmark=tick;
    }
    break;
  case 15: /* R1：手动标定页 → 回子菜单（sub_sel 保留 3） */
    hbuttons=0xF7FFU;
    ui_snapshot_get(&snap);
    if(snap.page==UI_PAGE_SUBMENU){ hstage=16; hmark=tick; }
    break;
  case 16: /* 释放 R1 */
    hbuttons=0xFFFFU;
    if((uint32_t)(tick-hmark)>=30U){ hstage=17; }
    break;
  case 17: /* 摇杆上移一格：sub_sel 3 → 2（回到初始位） */
    hbuttons=0xFFFFU; hly=48U;
    ui_snapshot_get(&snap);
    if(snap.sub_sel==2U){ hstage=18; hmark=tick; }
    break;
  case 18: /* 摇杆回中 */
    hbuttons=0xFFFFU; hly=128U;
    if((uint32_t)(tick-hmark)>=30U){ hstage=19; }
    break;
  case 19: /* L1：子菜单提交归位（req.home_start），页面进 HOME */
    hbuttons=0xFBFFU;
    ui_snapshot_get(&snap);
    if(snap.page==UI_PAGE_HOME){ hstage=20; hmark=tick; }
    break;
  case 20: /* 继续按住 L1：请求已 pending，同一次 L1 不得取消它 */
    hbuttons=0xFBFFU;
    if((uint32_t)(tick-hmark)>=60U){ hstage=21; hmark=tick; }
    break;
  case 21: /* 释放 L1 → 放行门（全释放 + 回中 100 ms）→ RUNNING */
    hbuttons=0xFFFFU;
    wheel_home_get_status(&st);
    if(st.mode==WHEEL_HOME_RUNNING){
      assert(st.result!=WHEEL_HOME_RESULT_CANCELLED);
      /* 误差只出现在 C/D 两轮：证明后轮读 TIM5/TIM4 且符号正确。 */
      assert(st.error[0]==0 && st.error[1]==0);
      assert(st.error[2]==(int64_t)(WHEEL_RR_CPR/4U));
      assert(st.error[3]==-(int64_t)(WHEEL_LR_CPR/4U));
      /* 只有 C/D 通道出力（CH2/CH1），A/B 保持零。 */
      assert(htim8.compare[1]!=0U && htim8.compare[1]<=WHEEL_HOME_MAX_PULSE); /* CH2 = 右后 C */
      assert(htim8.compare[0]!=0U && htim8.compare[0]<=WHEEL_HOME_MAX_PULSE); /* CH1 = 左后 D */
      assert(htim8.compare[2]==0U);                     /* CH3 = 左前 B */
      assert(htim8.compare[3]==0U);                     /* CH4 = 右前 A */
      assert(gpiod.ODR&GPIO_PIN_10);                    /* C 前进 = SET */
      assert(gpioc.ODR&GPIO_PIN_12);                    /* D 后退 = SET */
      /* 归位已启动、C/D 都在出力：场景到此为止。
       * “停止后关驱动使能”由 fault 场景（锁存）与 stop 场景覆盖。 */
      longjmp(done,1);
    }
    if((uint32_t)(tick-hmark)>=1000U){
      puts("homing never started");assert(0);
    }
    break;
  default:
    break;
  }
}

void ps2_init(void) {}
void ps2_config(void){}
/* USER 引脚：测试里始终保持释放（高电平）。 */
uint32_t HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint32_t bit)
{ (void)p; (void)bit; return GPIO_PIN_SET; }

/* R1=位 11、L1=位 10，低有效。 */
#define UI_TEST_BTNS_NONE 0xFFFFU
#define UI_TEST_BTNS_R1   0xF7FFU
#define UI_TEST_BTNS_L1   0xFBFFU

static void ui_script(ps2_state_t *p)
{
  ui_snapshot_t snap;

  un++;
  p->lx = 128U; p->rx = 128U; p->ry = 128U;
  p->ly = 128U;
  p->buttons = UI_TEST_BTNS_NONE;
  switch (ustage)
  {
  case 0: /* 冷启动回中性预热；品牌页快照立即可读 */
    if (un >= 10U)
    {
      ui_snapshot_get(&snap);
      assert(snap.page == UI_PAGE_BRAND);
      assert(snap.drive_permitted == 0U);
    }
    if (un >= 14U)
    {
      /* 再武装完成（回中 ≥100 ms）后帧有效。 */
      ui_snapshot_get(&snap);
      assert(snap.frame_valid == 1U);
    }
    if (un >= 40U) ustage = 1;
    break;
  case 1: /* R1：品牌页 → 主菜单 */
    p->buttons = UI_TEST_BTNS_R1;
    if (un >= 46U)
    {
      ui_snapshot_get(&snap);
      assert(snap.page == UI_PAGE_MENU);
      ustage = 2;
    }
    break;
  case 2: /* 释放 R1（3 帧稳定释放后才能受理 L1） */
    if (un >= 50U) ustage = 3;
    break;
  case 3: /* L1：主菜单（menu_sel==0 驾驶） → 驾驶页 */
    p->buttons = UI_TEST_BTNS_L1;
    if (un >= 56U)
    {
      ui_snapshot_get(&snap);
      assert(snap.page == UI_PAGE_DRIVE);
      ustage = 4;
    }
    break;
  case 4: /* 释放 L1 */
    if (un >= 60U) ustage = 5;
    break;
  case 5: /* 驾驶页满前进：回中武装满 100 ms 前禁止输出 */
    p->ly = 0U;
    if (un >= 64U)
    {
      ui_snapshot_get(&snap);
      assert(snap.page == UI_PAGE_DRIVE);
      assert(snap.drive_status == UI_DRV_FWD);
      assert(snap.drive_permitted == 0U);
      assert(htim8.compare[0] == 0U && htim8.compare[1] == 0U);
      assert(htim8.compare[2] == 0U && htim8.compare[3] == 0U);
      ustage = 6;
      uenc = 10U; /* 约 1 圈：10 × 21320 = 213200 ≈ WHEEL_RF_CPR */
    }
    break;
  case 6: /* 回中等待武装；同时注入 A 轮编码器约 1 圈 */
    if (uenc > 0U)
    {
      htim2.counter = (uint16_t)(htim2.counter + 21320U);
      uenc--;
    }
    if (un >= 104U)
    {
      ui_snapshot_get(&snap);
      assert(snap.drive_permitted == 1U);
      assert(snap.wheel_sample_valid[0] == 1U);
      /* 前进符号 -1：注入正计数 → milliturns 为负，约 -1000 ±5% */
      assert(snap.wheel_milliturns[0] <= -950 && snap.wheel_milliturns[0] >= -1050);
      u_mt_prev = snap.wheel_milliturns[0];
      ustage = 7;
    }
    break;
  case 7: /* 武装后再推满：武装保持，四轮 compare=8400 */
    p->ly = 0U;
    if (un >= 108U)
    {
      int i;
      ui_snapshot_get(&snap);
      assert(snap.drive_permitted == 1U);
      assert(snap.drive_status == UI_DRV_FWD);
      for (i = 0; i < 4; i++) assert(htim8.compare[i] == 8400U);
      ustage = 8;
    }
    break;
  case 8: /* 驾驶中 R1：下一控制周期输出全零（不等 OLED） */
    p->buttons = UI_TEST_BTNS_R1;
    p->ly = 0U;
    if (un >= 114U)
    {
      int i;
      ui_snapshot_get(&snap);
      assert(snap.page == UI_PAGE_MENU);
      assert(snap.drive_permitted == 0U);
      for (i = 0; i < 4; i++) assert(htim8.compare[i] == 0U);
      ustage = 9;
      uenc = 5U; /* 切页后注入约半圈，验证计数连续 */
    }
    break;
  case 9: /* 切页后 milliturns 只沿运动方向累计，不归零 */
    if (uenc > 0U)
    {
      htim2.counter = (uint16_t)(htim2.counter + 21320U);
      uenc--;
    }
    else
    {
      /* 最后一帧注入要到下一轮采样才进 fb.counts，等两拍再断言。 */
      static uint32_t settle;
      settle++;
      if (settle >= 2U)
      {
        ui_snapshot_get(&snap);
        /* 累计 319800 计数 ≈ 1.5 圈 → 约 -1500 ±5%，且比切页前更负 */
        assert(snap.wheel_milliturns[0] <= -1425 &&
               snap.wheel_milliturns[0] >= -1575);
        assert(snap.wheel_milliturns[0] < u_mt_prev);
        longjmp(done, 1);
      }
    }
    break;
  default:
    break;
  }
}

/* --- menu selfcheck scenario ('c') -------------------------------------- */
/* 菜单发起自检：RUNNING 期间服务独占 command（驾驶推杆不出力），
 * 推满摇杆触发越区取消；重跑全过后 5 s 结果窗口打开并超时回品牌页。 */
static int selfcheck_test;
static int cstage;
static uint32_t cn;       /* 场景内 ps2_poll 帧计数 */
static uint32_t cmark;    /* 阶段 tick 锚点 */
static uint32_t crank_t0; /* 受试轮注入时序起点 */
static uint8_t crank_on;
static uint16_t cbuttons = 0xFFFFU;
static uint8_t clx = 128U, cly = 128U;

/* 自检反馈注入：按服务时序给受试轮"前进为正"的编码器增量。
 * 每轮 500ms 前转(+) / 200ms 停 / 500ms 反转(−) / 200ms 停；
 * raw 增量按各轮前进符号折算（RF/RR=-1，LF/LR=+1）。 */
static void selfcheck_inject(uint32_t elapsed)
{
  static TIM_HandleTypeDef *const tim[4] = {&htim2, &htim3, &htim5, &htim4};
  static const int sign[4] = {-1, 1, -1, 1};
  uint32_t wheel = elapsed / 1400U;
  uint32_t m = elapsed % 1400U;
  int32_t fwd;

  if (wheel >= 4U) { return; }
  if (m < 500U) { fwd = 24; }
  else if (m >= 700U && m < 1200U) { fwd = -24; }
  else { fwd = 0; }
  if (fwd != 0)
  {
    tim[wheel]->counter =
        (uint16_t)(tim[wheel]->counter + (uint32_t)(fwd * sign[wheel]));
  }
}

/* 注入中：除受试轮外任何通道都不得有输出（驾驶混合被服务独占挡住）。 */
static void assert_selfcheck_output_pattern(void)
{
  ui_snapshot_t snap;
  uint32_t wheel;
  int i;

  ui_snapshot_get(&snap);
  assert(snap.drive_permitted == 0U);
  wheel = (uint32_t)(tick - crank_t0) / 1400U;
  if (wheel >= 4U) { wheel = 3U; }
  for (i = 0; i < 4; i++)
  {
    /* 通道 i ↔ 轮 (3-i)：CH4=A(0)、CH3=B(1)、CH2=C(2)、CH1=D(3)。 */
    uint32_t wch = 3U - (uint32_t)i;
    if (wch != wheel)
    {
      assert(htim8.compare[i] == 0U);
    }
    else
    {
      assert(htim8.compare[i] == 0U || htim8.compare[i] == 8400U);
    }
  }
}

/* 导航辅助：品牌页到"重新自检"的确认页（SUBMENU sub_sel==1）。
 * 每步以快照为转移条件，帧间留足 3 帧去抖。 */
static void selfcheck_nav(int *stage, uint32_t *mark)
{
  ui_snapshot_t snap;

  switch (*stage)
  {
  case 0: /* R1 → 主菜单 */
    cbuttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { *stage = 1; *mark = tick; }
    break;
  case 1: /* 释放 R1 */
    cbuttons = 0xFFFFU;
    if ((uint32_t)(tick - *mark) >= 30U) { *stage = 2; *mark = tick; }
    break;
  case 2: /* 右移选中"检查与标定" */
    cbuttons = 0xFFFFU; clx = 208U;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 1U) { *stage = 3; *mark = tick; }
    break;
  case 3:
    cbuttons = 0xFFFFU; clx = 128U;
    if ((uint32_t)(tick - *mark) >= 30U) { *stage = 4; *mark = tick; }
    break;
  case 4: /* L1 → 子菜单 */
    cbuttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { *stage = 5; *mark = tick; }
    break;
  case 5:
    cbuttons = 0xFFFFU;
    if ((uint32_t)(tick - *mark) >= 30U) { *stage = 6; *mark = tick; }
    break;
  case 6: /* 下移到"重新自检"（sub_sel 可能被上一次导航保留为 1：先看快照再推杆，
           * 避免滞后一帧的快照让已到位的条目又被连移推过一格） */
    cbuttons = 0xFFFFU;
    ui_snapshot_get(&snap);
    if (snap.sub_sel == 1U)
    {
      cly = 128U;
      *stage = 7; *mark = tick;
    }
    else
    {
      cly = 208U;
    }
    break;
  case 7:
    cbuttons = 0xFFFFU; cly = 128U;
    if ((uint32_t)(tick - *mark) >= 30U) { *stage = 8; *mark = tick; }
    break;
  case 8: /* L1 → 确认页 */
    cbuttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_CONFIRM) { *stage = 9; *mark = tick; }
    break;
  case 9:
    cbuttons = 0xFFFFU;
    if ((uint32_t)(tick - *mark) >= 30U) { *stage = 10; *mark = tick; }
    break;
  case 10: /* 下移焦点到"开始" */
    cbuttons = 0xFFFFU; cly = 208U;
    ui_snapshot_get(&snap);
    if (snap.confirm_focus == 1U) { *stage = 11; *mark = tick; }
    break;
  case 11:
    cbuttons = 0xFFFFU; cly = 128U;
    if ((uint32_t)(tick - *mark) >= 30U) { *stage = 12; *mark = tick; }
    break;
  default:
    break;
  }
}

static void selfcheck_script(ps2_state_t *p)
{
  ui_snapshot_t snap;

  (void)p;
  cn++;
  switch (cstage)
  {
  case 0: /* 冷启动再武装预热；完成后进入 nav case 0（按住 R1 等进主菜单） */
    cbuttons = 0xFFFFU; clx = 128U; cly = 128U;
    if (cn >= 40U)
    {
      ui_snapshot_get(&snap);
      /* nav case 0 按住 R1 期间快照有一帧滞后：BRAND→MENU 的过渡帧
       * 两个页面都合法，其余页面视为跑飞。 */
      assert(snap.page == UI_PAGE_BRAND || snap.page == UI_PAGE_MENU);
      selfcheck_nav(&cstage, &cmark);
    }
    break;
  case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8:
  case 9: case 10: case 11:
    selfcheck_nav(&cstage, &cmark);
    break;
  case 12: /* L1：确认页"开始" → 自检启动（RUNNING） */
    cbuttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_RUN)
    {
      cstage = 13; cmark = tick;
      assert(snap.action_busy != 0U || snap.selfcheck_state[0] != 0U);
    }
    break;
  case 13: /* 释放 L1，等服务真跑起来（state[0]==1）后开始注入 */
    cbuttons = 0xFFFFU;
    ui_snapshot_get(&snap);
    if (snap.selfcheck_state[0] == 1U)
    {
      crank_on = 1U;
      crank_t0 = tick;
      cstage = 14; cmark = tick;
    }
    break;
  case 14: /* RUNNING 前段（回中推杆不出力）+ 后段推满（越区取消） */
    if ((uint32_t)(tick - cmark) < 300U)
    {
      selfcheck_inject((uint32_t)(tick - crank_t0));
      assert_selfcheck_output_pattern();
    }
    else if ((uint32_t)(tick - cmark) < 400U)
    {
      /* 推满前进：不得产生任何驾驶出力；页面模型随即取消自检。 */
      cly = 0U;
      selfcheck_inject((uint32_t)(tick - crank_t0));
      assert_selfcheck_output_pattern();
    }
    else
    {
      ui_snapshot_get(&snap);
      assert(snap.page == UI_PAGE_CHECK_CONFIRM); /* 越区取消已回退 */
      for (int i = 0; i < 4; i++)
      {
        assert(htim8.compare[i] == 0U);
        assert(snap.selfcheck_state[i] != 1U); /* 没有轮还在"运行" */
      }
      assert(snap.selfcheck_state[0] == 3U);   /* 受试轮定格失败 */
      cly = 128U;
      cstage = 20; cmark = tick;
    }
    break;
  case 20: /* 取消后从确认页重跑：等按键滤波/摇杆计时清干净 */
    cbuttons = 0xFFFFU; clx = 128U; cly = 128U;
    if ((uint32_t)(tick - cmark) >= 30U) { cstage = 21; }
    break;
  case 21: /* 确认页：焦点必须回到"开始"（满推杆可能已把它拨回取消） */
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_CONFIRM && snap.confirm_focus == 1U)
    {
      cstage = 22; cmark = tick;
    }
    else
    {
      cly = 208U; /* 下移焦点到"开始" */
      ui_snapshot_get(&snap);
      if (snap.confirm_focus == 1U) { cly = 128U; cstage = 22; cmark = tick; }
    }
    break;
  case 22: /* 摇杆回中片刻（自检前置要求四轴回中），L1 重新发起 */
    cbuttons = 0xFBFFU; cly = 128U;
    if ((uint32_t)(tick - cmark) >= 30U)
    {
      ui_snapshot_get(&snap);
      if (snap.page == UI_PAGE_CHECK_RUN) { cstage = 23; cmark = tick; }
    }
    break;
  case 23: /* 释放 L1，等服务真跑起来（state[0]==1）后开始注入 */
    cbuttons = 0xFFFFU;
    ui_snapshot_get(&snap);
    if (snap.selfcheck_state[0] == 1U)
    {
      crank_on = 1U;
      crank_t0 = tick;
      cstage = 24;
    }
    break;
  case 24: /* 全程注入直到四轮全部通过 */
    if (crank_on != 0U)
    {
      selfcheck_inject((uint32_t)(tick - crank_t0));
    }
    ui_snapshot_get(&snap);
    if (snap.selfcheck_state[0] == 2U && snap.selfcheck_state[1] == 2U &&
        snap.selfcheck_state[2] == 2U && snap.selfcheck_state[3] == 2U)
    {
      crank_on = 0U;
      cmark = tick;
      cstage = 26; /* 页面转移比服务完成晚一帧：先等 CHECK_RESULT 落地 */
    }
    break;
  case 26: /* 全过当拍窗口已开（Ctrl 同拍置位），页面下一拍自动转结果页 */
    ui_snapshot_get(&snap);
    assert(snap.result_window_open == 1U);
    if (snap.page == UI_PAGE_CHECK_RESULT)
    {
      cmark = tick;
      cstage = 25;
    }
    else
    {
      assert(snap.page == UI_PAGE_CHECK_RUN); /* 只允许这一帧滞后 */
      assert((uint32_t)(tick - cmark) < 200U);
    }
    break;
  case 25: /* 窗口内：5 s 后必须关闭并自动回品牌页 */
    cbuttons = 0xFFFFU;
    if ((uint32_t)(tick - cmark) < 4800U)
    {
      ui_snapshot_get(&snap);
      assert(snap.result_window_open == 1U);
    }
    else if ((uint32_t)(tick - cmark) >= 5400U)
    {
      ui_snapshot_get(&snap);
      assert(snap.result_window_open == 0U);
      assert(snap.page == UI_PAGE_BRAND);
      longjmp(done, 1);
    }
    break;
  default:
    break;
  }
}

/* --- failed-recheck latch scenario ('x') -------------------------------- */
/* 先制造 NO FB 锁存；菜单发起的自检失败（无反馈）不得清锁存、不得出力；
 * 全过 + 回到驾驶页回中后才恢复输出。 */
static int relatch_test;
static int xstage;
static uint32_t xn;
static uint32_t xmark;
static uint32_t xcrank_t0;
static uint8_t xcrank_on;
static uint16_t xbuttons = 0xFFFFU;
static uint8_t xlx = 128U, xly = 128U;

static void relatch_script(ps2_state_t *p)
{
  ui_snapshot_t snap;

  (void)p;
  xn++;
  switch (xstage)
  {
  case 0: /* 冷启动再武装预热 */
    xbuttons = 0xFFFFU; xlx = 128U; xly = 128U;
    if (xn >= 40U) { xstage = 1; }
    break;
  case 1: /* R1 → 主菜单 */
    xbuttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { xstage = 2; xmark = tick; }
    break;
  case 2:
    xbuttons = 0xFFFFU;
    if ((uint32_t)(tick - xmark) >= 30U) { xstage = 3; }
    break;
  case 3: /* L1（menu_sel==0）→ 驾驶页 */
    xbuttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_DRIVE) { xstage = 4; xmark = tick; }
    break;
  case 4:
    xbuttons = 0xFFFFU;
    if ((uint32_t)(tick - xmark) >= 30U) { xstage = 5; xmark = tick; }
    break;
  case 5: /* 驾驶页回中武装 ≥100 ms 后推满：反馈为零 → NO FB 锁存 */
    if ((uint32_t)(tick - xmark) < 150U) { /* 武装中 */ }
    else if ((uint32_t)(tick - xmark) < 300U)
    {
      xly = 0U;
      /* 比较器滞后一帧：满杆推入后的下一拍才检查出力。 */
      if ((uint32_t)(tick - xmark) >= 156U)
      {
        assert(htim8.compare[0] > 0U); /* 先确认真的在出力 */
      }
    }
    else if ((uint32_t)(tick - xmark) < 1200U)
    {
      xly = 0U; /* 宽限 300 + 窗口 300 内保持满输出零反馈 */
    }
    else
    {
      int i;
      xly = 128U;
      for (i = 0; i < 4; i++) { assert(htim8.compare[i] == 0U); }
      assert(!(gpiob.ODR & GPIO_PIN_12)); /* 锁存后停止流程关使能 */
      ui_snapshot_get(&snap);
      assert(snap.fault_mask != 0U);
      xstage = 6; xmark = tick;
    }
    break;
  case 6: /* R1：驾驶页 → 主菜单，然后走自检导航（菜单口径一致） */
    xbuttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { xstage = 7; xmark = tick; }
    break;
  case 7:
    xbuttons = 0xFFFFU;
    if ((uint32_t)(tick - xmark) >= 30U)
    {
      /* 复用自检导航：从主菜单起到确认页（stage 2..11）。 */
      cstage = 2; cmark = tick;
      cbuttons = 0xFFFFU; clx = 128U; cly = 128U;
      xstage = 8;
    }
    break;
  case 8: /* 导航推进（借 selfcheck_nav，输出借 c 变量） */
    selfcheck_nav(&cstage, &cmark);
    xbuttons = cbuttons; xlx = clx; xly = cly;
    if (cstage == 12) { xstage = 9; xmark = tick; }
    break;
  case 9: /* L1：发起"失败的自检"（不注入反馈） */
    xbuttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_RUN) { xstage = 10; xmark = tick; }
    break;
  case 10: /* 等失败自检跑完（约 5.6 s）：锁存必须还在 */
    xbuttons = 0xFFFFU;
    if ((uint32_t)(tick - xmark) >= 6300U)
    {
      ui_snapshot_get(&snap);
      for (int i = 0; i < 4; i++)
      {
        assert(snap.selfcheck_state[i] == 3U); /* 全轮 NO FB */
      }
      assert(snap.fault_mask != 0U);      /* 失败不清已锁存故障 */
      assert(snap.result_window_open == 1U);
      xstage = 11; xmark = tick;
    }
    break;
  case 11: /* 结果页 R1 → 品牌页（结果页 back_page 已被自动转移改成 BRAND） */
    xbuttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_BRAND) { xstage = 12; xmark = tick; }
    break;
  case 12:
    xbuttons = 0xFFFFU;
    if ((uint32_t)(tick - xmark) >= 30U) { xstage = 13; }
    break;
  case 13: /* 品牌页 R1 → 主菜单，再走一遍自检导航（这次注入反馈） */
    xbuttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { xstage = 14; xmark = tick; }
    break;
  case 14:
    xbuttons = 0xFFFFU;
    if ((uint32_t)(tick - xmark) >= 30U)
    {
      cstage = 2; cmark = tick;
      cbuttons = 0xFFFFU; clx = 128U; cly = 128U;
      xstage = 15;
    }
    break;
  case 15:
    selfcheck_nav(&cstage, &cmark);
    xbuttons = cbuttons; xlx = clx; xly = cly;
    if (cstage == 12) { xstage = 16; xmark = tick; }
    break;
  case 16: /* L1：发起"通过的自检" */
    xbuttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_RUN) { xstage = 17; xmark = tick; }
    break;
  case 17:
    xbuttons = 0xFFFFU;
    ui_snapshot_get(&snap);
    if (snap.selfcheck_state[0] == 1U)
    {
      xcrank_on = 1U;
      xcrank_t0 = tick;
      xstage = 18;
    }
    break;
  case 18: /* 全程注入直到四轮全过：此时故障锁存才被清 */
    if (xcrank_on != 0U)
    {
      selfcheck_inject((uint32_t)(tick - xcrank_t0));
    }
    ui_snapshot_get(&snap);
    if (snap.selfcheck_state[0] == 2U && snap.selfcheck_state[1] == 2U &&
        snap.selfcheck_state[2] == 2U && snap.selfcheck_state[3] == 2U)
    {
      xcrank_on = 0U;
      assert(snap.fault_mask == 0U); /* 全过才清锁存（清障与服务完成同拍） */
      xstage = 26; /* 结果窗口的 DONE 沿下一拍才被 Ctrl 补记：延迟一拍再断言 */
    }
    break;
  case 26:
    ui_snapshot_get(&snap);
    assert(snap.result_window_open == 1U);
    xstage = 19; xmark = tick;
    break;
  case 19: /* 结果页 R1 → 品牌页 → 主菜单 → 驾驶页 */
    xbuttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_BRAND) { xstage = 20; xmark = tick; }
    break;
  case 20:
    xbuttons = 0xFFFFU;
    if ((uint32_t)(tick - xmark) >= 30U) { xstage = 21; }
    break;
  case 21: /* R1 → 主菜单 */
    xbuttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { xstage = 22; xmark = tick; }
    break;
  case 22:
    xbuttons = 0xFFFFU;
    if ((uint32_t)(tick - xmark) >= 30U) { xstage = 27; }
    break;
  case 27: /* 菜单选择被之前的导航留在 1：左移回"驾驶" */
    xbuttons = 0xFFFFU;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 0U) { xstage = 28; xmark = tick; }
    else { xlx = 48U; }
    break;
  case 28: /* 摇杆回中 */
    xbuttons = 0xFFFFU; xlx = 128U;
    if ((uint32_t)(tick - xmark) >= 30U) { xstage = 23; }
    break;
  case 23: /* L1（menu_sel==0）→ 驾驶页 */
    xbuttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_DRIVE) { xstage = 24; xmark = tick; }
    break;
  case 24:
    xbuttons = 0xFFFFU;
    if ((uint32_t)(tick - xmark) >= 30U) { xstage = 25; xmark = tick; }
    break;
  case 25: /* 回中武装 ≥100 ms 后推满：输出必须已恢复 */
    if ((uint32_t)(tick - xmark) < 150U) { /* 武装中 */ }
    else
    {
      xly = 0U;
      /* 比较器滞后一帧：满杆推入后的下一拍才检查出力。 */
      if ((uint32_t)(tick - xmark) >= 156U)
      {
        assert(htim8.compare[0] > 0U); /* 全过 + 回中：恢复出力 */
        longjmp(done, 1);
      }
    }
    break;
  default:
    break;
  }
}

uint8_t ps2_poll(ps2_state_t *p){
 if(stop_test){
   /* 导航到驾驶页并等回中武装后推满出力，再回中让停止流程关使能。 */
   static uint32_t n;
   n++;
   p->buttons=0xFFFFU;p->lx=128U;p->rx=128U;p->ry=128U;
   p->ly=128U;
   if(n<=40U){ /* 冷启动再武装：回中且 SELECT 释放持续 100 ms */ }
   else if(n<=46U){ p->buttons=0xF7FFU; }             /* R1 → 主菜单 */
   else if(n<=48U){ }                                 /* 释放 R1 */
   else if(n<=54U){ p->buttons=0xFBFFU; }             /* L1 → 驾驶页 */
   else if(n<=66U){ }                                 /* 驾驶页回中武装 ≥100 ms */
   else if(n<=82U){ p->ly=0U; }                       /* 推满前进 */
   if(n==78U){
     assert(gpiob.ODR&GPIO_PIN_12);                   /* 推杆期间使能开着 */
     assert(htim8.compare[0]>0U);                     /* 确实在出力 */
   }
   /* n==83 起回中：mix 归零 → 停止流程关使能 */
   if(n==86U){
     for(int i=0;i<4;i++)assert(htim8.compare[i]==0U);
     assert(!(gpiob.ODR&GPIO_PIN_12));assert(!(gpiob.ODR&GPIO_PIN_14));
     assert(!(gpioa.ODR&GPIO_PIN_8));assert(!(gpiod.ODR&GPIO_PIN_12));
     longjmp(done,1);
   }
   tick+=3U;
   return PS2_FRAME_OK;
 }
 if(fault_test){
   static uint32_t nf;
   uint32_t dt = tick - fault_t0;
   nf++;
   p->buttons=0xFFFFU; p->lx=128U; p->rx=128U; p->ry=128U; p->ly=128U;
   /* 先导航到驾驶页并等回中武装，之后才允许满输出。 */
   if(nf<=30U){ /* 冷启动再武装：回中且 SELECT 释放持续 100 ms */ }
   else if(nf<=36U){ p->buttons=0xF7FFU; }            /* R1 → 主菜单 */
   else if(nf<=38U){ }                                /* 释放 R1 */
   else if(nf<=44U){ p->buttons=0xFBFFU; }            /* L1 → 驾驶页 */
   else if(nf<=56U){ }                                /* 驾驶页回中武装 ≥100 ms */
   else { p->ly=0U; }                                 /* 满输出：反馈为零 → NO FB */
   fault_frames++;
   if(dt < 1500U){
     /* 满输出、反馈为零：A 轮也应该走 NO FB 锁存 */
     tick+=3U;
     return PS2_FRAME_OK;
   }
   if(!fault_cleared){
     /* 已超过宽限+窗口：输出必须被清零（锁存生效） */
     for(int i=0;i<4;i++)assert(htim8.compare[i]==0U);
     /* 停止流程：锁存后驱动使能必须同时关闭，而不是只写零 PWM。 */
     assert(!(gpiob.ODR&GPIO_PIN_12));assert(!(gpiob.ODR&GPIO_PIN_14));
     fault_cleared=1;
     /* 模拟“用户重新自检且四轮全过” */
     boot_ui_selfcheck_seq++;
     boot_ui_selfcheck_passed=1U;
     tick+=3U;
     return PS2_FRAME_OK;
   }
   /* 清故障后必须恢复出力（锁存已解除）。清零那一刻驱动是睡眠的，
    * 恢复先要唤醒并等 10 ms；且驾驶命令受页面仲裁（用上一拍的
    * fault_mask），所以清完给几帧余量再断言。 */
   {
     static uint32_t rec;
     rec++;
     if(rec<3U){ tick+=3U; return PS2_FRAME_OK; }
   }
   for(int i=0;i<4;i++)assert(htim8.compare[i]!=0U);
   longjmp(done,1);
 }
 /* 冷启动再武装：摇杆回中且 SELECT 释放必须持续 100 ms（tick 每轮 +3 ms），
  * 因此冷启动场景先喂 PREARM_FRAMES 帧中性，之后才进被测流程。
  * 该预滚对所有冷启动场景生效（含 reconnect 的首段），避免推杆本身
  * 打断居中计时导致再武装永远无法完成。 */
 if(ui_test){ ui_script(p); tick+=3U; return PS2_FRAME_OK; }
 if(selfcheck_test){
   selfcheck_script(p);
   p->buttons=cbuttons;p->lx=clx;p->ly=cly;p->rx=128U;p->ry=128U;
   tick+=3U;return PS2_FRAME_OK;
 }
 if(relatch_test){
   relatch_script(p);
   p->buttons=xbuttons;p->lx=xlx;p->ly=xly;p->rx=128U;p->ry=128U;
   tick+=3U;return PS2_FRAME_OK;
 }
 if(!home_test && !fault_test && !reconnect_seen){
   static uint32_t preroll;
   if(preroll < 40U){
     preroll++;
     p->buttons=0xFFFFU;p->lx=128U;p->ly=128U;p->rx=128U;p->ry=128U;
     tick+=3U;
     return PS2_FRAME_OK;
   }
 }
 if(home_test){home_script();p->buttons=hbuttons;p->lx=hlx;p->ly=hly;
   p->rx=128U;p->ry=128U;tick+=3U;return PS2_FRAME_OK;}
 current_phase=phase++;
 if(current_phase==0)first_poll_tick=tick;
 assert(current_phase!=1 || (uint32_t)(tick-first_poll_tick)<=20U);
 tick+=3U; /* real PS2 transaction consumes about 3 ms */
  if(current_phase==0 && reconnect_test){
    /* 重连场景：第 0 帧只做预热，不推杆，也不让 active() 提前成立。 */
    p->buttons=0xFFFFU;p->lx=128U;p->ly=128U;p->rx=128U;p->ry=128U;
    return PS2_FRAME_OK;
  }
  if(current_phase>=1 && !reconnect_test){
    /* 冷启动先回中：预热 40 帧已过。先导航到驾驶页（R1→菜单、L1→驾驶），
     * 等驾驶页回中武装满 100 ms 后再推杆，断言唤醒+输出后结束场景。 */
    if(current_phase<=6){ p->buttons=0xF7FFU; p->ly=128U; }       /* R1 → 主菜单 */
    else if(current_phase<=8){ p->buttons=0xFFFFU; p->ly=128U; }  /* 释放 R1 */
    else if(current_phase<=14){ p->buttons=0xFBFFU; p->ly=128U; } /* L1 → 驾驶页 */
    else if(current_phase<=29){ p->buttons=0xFFFFU; p->ly=128U; } /* 回中武装 ≥100 ms */
    else { p->buttons=0xFFFFU; p->ly=0; }
    if(current_phase<=30)return PS2_FRAME_OK; /* 本帧只唤醒，下一帧才出力 */
    active();
    longjmp(done,1);
  }
 if(reconnect_test && current_phase==4)asleep();
 if(reconnect_test && current_phase>=5){
   /* 重连后再武装：回中且 SELECT 释放持续 100 ms（每轮 ≈ 13 ms ≈ 8 轮）。
    * 中性若干拍后导航到驾驶页，等回中武装完成再推杆断言。 */
   if(current_phase<=30){
     p->buttons=0xFFFFU;p->lx=128U;p->ly=128U;p->rx=128U;p->ry=128U;
     return PS2_FRAME_OK;
   }
   if(current_phase<=36){ p->buttons=0xF7FFU; p->ly=128U; }  /* R1 → 菜单 */
   else if(current_phase<=38){ p->buttons=0xFFFFU; p->ly=128U; }
   else if(current_phase<=44){ p->buttons=0xFBFFU; p->ly=128U; } /* L1 → 驾驶 */
   else if(current_phase<=56){ p->buttons=0xFFFFU; p->ly=128U; } /* 回中武装 */
   else { p->buttons=0xFFFFU; p->ly=0; }
   if(current_phase<=57)return PS2_FRAME_OK; /* 本帧只唤醒，下一帧才出力 */
   active();longjmp(done,1);
 }
 p->buttons=0xFFFFU; p->ly=0; p->lx=128U; p->rx=128U; p->ry=128U; current_ly=0;
 return reconnect_test && current_phase>=1 && current_phase<=3 ? PS2_FRAME_BAD : PS2_FRAME_OK;
}
void osDelay(uint32_t ms){
 if(drive_calibration_mode != 0U){asleep();longjmp(done,1); }
 tick+=ms;
 /* A scheduling stall takes us beyond the deadline with only three bad
  * frames. Counting ten bad frames would keep the old command too long. */
 if(reconnect_test && current_phase==2)tick+=200U;
}
int main(int argc, char **argv){
 if(argc>1 && argv[1][0]=='b'){ verbose_test=(argc>2); if(!setjmp(done))StartCtrlTask(0); puts("basic ok"); return 0; }
 if(argc>1 && argv[1][0]=='v'){ verbose_test=1; if(!setjmp(done))StartCtrlTask(0); puts("end"); return 0; }
 /* 归位场景单独一个进程：状态机是全静态的，与上面几个场景共享会
  * 带入 50 ms 采样新鲜度时间戳，混淆被测行为。 */
 if(argc>1 && argv[1][0]=='f'){
   fault_test=1;tick=0U;fault_t0=0U;fault_frames=0U;
   fault_t0=tick;
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl motor health latch / zero output / selfcheck clear OK");return 0;
 }
 if(argc>1 && argv[1][0]=='s'){
   stop_test=1;tick=0U;
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl stop closes drive enable");return 0;
 }
 if(argc>1 && argv[1][0]=='h'){
   home_test=1;tick=0U;phase=0;
   /* 复现上板现场：UI 自检收尾时 HAL_TIM_PWM_Stop 关掉四路通道，然后
    * 才移交 owner=CTRL。这里直接以"已移交"起始（owner 保持 CTRL），
    * 但 pwm_ccer 清零，保证 ctrl_link_service 的冷启动分支没机会抢先
    * 启动通道——这样才能测出"移交后需要有人启动通道"。
    * 若从 owner=UI 起步，Ctrl 会一直走无权跳过分支，归位相序根本推不动。 */
   boot_ui_drive_enabled=1U;
   pwm_ccer=0U;
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl homing arbitration / rear encoders OK");return 0;
 }
 if(argc>1 && argv[1][0]=='u'){
   ui_test=1;tick=0U;
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl ui snapshot / page-gated drive / dual-axis mix OK");return 0;
 }
 if(argc>1 && argv[1][0]=='c'){
   selfcheck_test=1;tick=0U;cn=0U;cstage=0;crank_on=0U;
   pm_set_vdd_ok(1U);
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl menu selfcheck / service-owned output / 5s window OK");return 0;
 }
 if(argc>1 && argv[1][0]=='x'){
   relatch_test=1;tick=0U;xn=0U;xstage=0;xcrank_on=0U;
   pm_set_vdd_ok(1U);
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl failed recheck keeps latch / pass+neutral restores OK");return 0;
 }
 if(!setjmp(done))StartCtrlTask(0);
 reconnect_test=1;reconnect_seen=1;phase=0;tick=1000;
 if(!setjmp(done))StartCtrlTask(0);
 phase=0;tick=UINT32_MAX-100U;
 if(!setjmp(done))StartCtrlTask(0);
 phase=0;drive_calibration_mode=1;
 if(!setjmp(done))StartCtrlTask(0);
 drive_calibration_mode=0;

 puts("ctrl wake / elapsed loss deadline / reconnect / tick wrap / cal sleep OK");
 return 0;
}
