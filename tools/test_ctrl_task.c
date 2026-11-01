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

/* --- power_monitor 桩 --------------------------------------------------- */
/* Ctrl 测试不链接 power_monitor.c：用可编程快照驱动 vdd_ok 门控
 * （菜单自检的前置条件之一），ADC 行为由 tools/test_power_monitor.c 覆盖。 */
static power_snapshot_t s_pm_stub;
void power_monitor_init(void) { memset(&s_pm_stub, 0, sizeof s_pm_stub); }
void power_monitor_step(uint32_t now_ms) { (void)now_ms; }
const power_snapshot_t *power_monitor_get(void) { return &s_pm_stub; }
static void pm_set_vdd_ok(uint8_t ok)
{
  /* ok=1：3.3 V 轨正常（3300 mV，vdd_ok=1）；
   * ok=0：模拟掉压到 3000 mV——采样仍成功（vdd_valid=1）但低于
   * BOOT_UI_VDD_MIN_MV，vdd_ok=0（运动/自检许可关闭）。 */
  s_pm_stub.vdd_valid = 1U;
  s_pm_stub.vdd_mv = (ok != 0U) ? 3300U : 3000U;
  s_pm_stub.vdd_ok = ok;
}

/* 自检 UX 已并入 Ctrl（boot_ui 收敛为渲染层）：不再有 boot_ui_manual_*
 * 接口和自检移交全局。本测试不链接 boot_ui.c，boot_ui_ready 直接以
 * "渲染层就绪"起步；自检结果由 Ctrl 内 selfcheck_service 的真实状态驱动。 */
volatile uint8_t boot_ui_ready=1;
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

/* 归位导航公共段（'h' 与 'n' 场景共用）：阶段 0..19 把页面从品牌页一路
 * 走到"子菜单 L1 提交归位进 HOME 页（L1 按住中）"，结束后 hstage==20。 */
static void home_nav(void){
  wheel_home_output_t st;
  ui_snapshot_t snap;

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
       * 16 位计数一次跳变必须 < 32768，故分两拍累加。
       * 自检前置给计数器留了任意残余：这里按相对增量注入，
       * 总位移与起点无关。 */
      htim5.counter=(uint16_t)(htim5.counter+20000U);
      htim4.counter=(uint16_t)(htim4.counter+20000U);
      hstage=13; hcycles=0;
    }
    break;
  case 13: /* 后续几拍补足到 1/4 圈（每拍 < 32768） */
    hbuttons=0xFFFFU;
    if(hcycles==1){htim5.counter=(uint16_t)(htim5.counter+20000U);
                   htim4.counter=(uint16_t)(htim4.counter+20000U);}
    else if(hcycles>=3){
      htim5.counter=(uint16_t)(htim5.counter+(uint32_t)(WHEEL_RR_CPR/4U-40000U));
      htim4.counter=(uint16_t)(htim4.counter+(uint32_t)(WHEEL_LR_CPR/4U-40000U));
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
  default:
    break;
  }
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
  if(hstage<20){ home_nav(); return; }
  switch(hstage){
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
      hstage=22; hmark=tick;
    }
    if((uint32_t)(tick-hmark)>=1000U){
      puts("homing never started");assert(0);
    }
    break;
  case 22: /* RUNNING 中按 R1：离开 HOME 页必须取消归位（spec §7/§10） */
    hbuttons=0xF7FFU;
    ui_snapshot_get(&snap);
    if(snap.page!=UI_PAGE_HOME){
      int i;
      /* 页面已离开：当拍取消落地，四轮 compare 全 0。 */
      for(i=0;i<4;i++) assert(htim8.compare[i]==0U);
      wheel_home_get_status(&st);
      assert(st.mode==WHEEL_HOME_WAIT_NEUTRAL ||
             st.mode==WHEEL_HOME_MANUAL);
      assert(st.result==WHEEL_HOME_RESULT_CANCELLED);
      hstage=23; hmark=tick;
    }
    if((uint32_t)(tick-hmark)>=1000U){
      puts("homing not cancelled by R1");assert(0);
    }
    break;
  case 23: /* 取消后保持停止：不允许任何归位脉冲复活 */
    hbuttons=0xFFFFU;
    if((uint32_t)(tick-hmark)>=150U){
      int i;
      wheel_home_get_status(&st);
      assert(st.mode!=WHEEL_HOME_RUNNING);
      assert(st.result==WHEEL_HOME_RESULT_CANCELLED);
      for(i=0;i<4;i++) assert(htim8.compare[i]==0U);
      longjmp(done,1);
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

/* --- output cap scenario ('o') ------------------------------------------ */
/* 子菜单第 5 条目"输出上限"页：进入时 50%（占空比口径 = 8400 = 现状满输出），
 * 摇杆下推到 40% 后运行时上限落到 drive_ctrl_max_pulse=6720；再上推连移到
 * 扩程后的新上限 90%（drive_ctrl_max_pulse=15120，fix round 2）；R1 回子菜单
 * 原条目；再导航到驾驶页武装推满：四轮 compare=15120（上限即时生效，
 * 无 8400 残留）。 */
static int cap_test;
static int ostage;
static uint32_t capn;   /* 场景内 ps2_poll 帧计数 */
static uint32_t omark;  /* 阶段 tick 锚点 */

static void cap_script(ps2_state_t *p)
{
  ui_snapshot_t snap;

  capn++;
  p->buttons = 0xFFFFU;
  p->lx = 128U; p->rx = 128U; p->ry = 128U; p->ly = 128U;
  switch (ostage)
  {
  case 0: /* 冷启动回中性预热（自检前置已由 ps2_poll 前导跑完一轮全过） */
    if (capn >= 40U) { ostage = 1; omark = tick; }
    break;
  case 1: /* R1：品牌页 → 主菜单 */
    p->buttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { ostage = 2; omark = tick; }
    break;
  case 2: /* 释放 R1 ≥3 稳定帧 */
    if ((uint32_t)(tick - omark) >= 30U) { ostage = 3; }
    break;
  case 3: /* 摇杆右移：选中"检查与标定" */
    p->lx = 208U;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 1U) { ostage = 4; omark = tick; }
    break;
  case 4: /* 摇杆回中 */
    if ((uint32_t)(tick - omark) >= 30U) { ostage = 5; }
    break;
  case 5: /* L1：主菜单 → 子菜单 */
    p->buttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { ostage = 6; omark = tick; }
    break;
  case 6: /* 释放 L1 */
    if ((uint32_t)(tick - omark) >= 30U) { ostage = 7; }
    break;
  case 7: /* 摇杆按住下移：连移走到第 5 项（输出上限） */
    p->ly = 208U;
    ui_snapshot_get(&snap);
    if (snap.sub_sel == 4U)
    {
      assert(snap.sub_scroll == 2U); /* 5 条目滚动窗口 */
      ostage = 8; omark = tick;
    }
    break;
  case 8: /* 摇杆回中 */
    p->ly = 128U;
    if ((uint32_t)(tick - omark) >= 30U) { ostage = 9; }
    break;
  case 9: /* L1：子菜单 → 输出上限页；进页当拍应用当前档 50%（=8400=默认，幂等） */
    p->buttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_OUTPUT_CAP)
    {
      assert(snap.output_cap_pct == 50U);
      assert(drive_ctrl_max_pulse == 50 * 168); /* 进页即生效，显示与实际一致 */
      ostage = 10; omark = tick;
    }
    break;
  case 10: /* 释放 L1 */
    if ((uint32_t)(tick - omark) >= 30U) { ostage = 11; omark = tick; }
    break;
  case 11: /* 摇杆按住下推：-5% 连移到 40%，快照与运行时上限同步 */
    p->ly = 208U;
    ui_snapshot_get(&snap);
    if (snap.output_cap_pct == 40U)
    {
      assert(drive_ctrl_max_pulse == 40 * 168); /* 40% × 168 = 6720 */
      ostage = 12; omark = tick;
    }
    if ((uint32_t)(tick - omark) >= 2000U) { puts("cap not adjusted"); assert(0); }
    break;
  case 12: /* 摇杆回中 */
    p->ly = 128U;
    if ((uint32_t)(tick - omark) >= 30U) { ostage = 13; omark = tick; }
    break;
  case 13: /* 摇杆按住上推：+5% 连移到扩程后的新上限 90%（fix round 2） */
    p->ly = 56U;
    ui_snapshot_get(&snap);
    if (snap.output_cap_pct == 90U)
    {
      assert(drive_ctrl_max_pulse == 90 * 168); /* 90% × 168 = 15120 */
      ostage = 14; omark = tick;
    }
    if ((uint32_t)(tick - omark) >= 6000U) { puts("cap not raised"); assert(0); }
    break;
  case 14: /* 摇杆回中 */
    p->ly = 128U;
    if ((uint32_t)(tick - omark) >= 30U) { ostage = 15; omark = tick; }
    break;
  case 15: /* R1：输出上限页 → 子菜单原条目（第 5 项） */
    p->buttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU)
    {
      assert(snap.sub_sel == 4U);
      assert(snap.output_cap_pct == 90U);
      ostage = 16; omark = tick;
    }
    if ((uint32_t)(tick - omark) >= 2000U) { puts("cap page not left"); assert(0); }
    break;
  case 16: /* 释放 R1 */
    if ((uint32_t)(tick - omark) >= 30U) { ostage = 17; omark = tick; }
    break;
  case 17: /* R1：子菜单 → 主菜单 */
    p->buttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { ostage = 18; omark = tick; }
    break;
  case 18: /* 释放 R1 */
    if ((uint32_t)(tick - omark) >= 30U) { ostage = 19; }
    break;
  case 19: /* 摇杆左移：选回"驾驶" */
    p->lx = 48U;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 0U) { ostage = 20; omark = tick; }
    break;
  case 20: /* 摇杆回中 */
    if ((uint32_t)(tick - omark) >= 30U) { ostage = 21; omark = tick; }
    break;
  case 21: /* L1：主菜单 → 驾驶页 */
    p->buttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_DRIVE) { ostage = 22; omark = tick; }
    break;
  case 22: /* 释放 L1，等回中武装 ≥100 ms */
    if ((uint32_t)(tick - omark) >= 150U) { ostage = 23; omark = tick; }
    break;
  case 23: /* 满前进：四轮 compare = 15120（90% 上限当拍生效） */
    p->ly = 0U;
    if ((uint32_t)(tick - omark) >= 30U)
    {
      int i;
      ui_snapshot_get(&snap);
      assert(snap.page == UI_PAGE_DRIVE);
      assert(snap.drive_permitted == 1U);
      assert(snap.output_cap_pct == 90U);
      for (i = 0; i < 4; i++)
      {
        assert(htim8.compare[i] == 15120U);
      }
      longjmp(done, 1);
    }
    break;
  default:
    break;
  }
}

/* --- drive rearm button-release gate scenario ('d') ---------------------- */
/* spec §7：驾驶页回中武装的条件含 R1/L1/SELECT 全部释放连续 100 ms。
 * 按住 L1 进入驾驶页并保持：即使四轴回中满 100 ms 也不武装（旧实现
 * 只看摇杆回中，会带着按键武装）；释放后重新计时，满 100 ms 才武装。 */
static int rearm_test;
static int dstage;
static uint32_t dn, dmark;

static void rearm_script(ps2_state_t *p)
{
  ui_snapshot_t snap;

  dn++;
  p->buttons = 0xFFFFU; p->lx = 128U; p->rx = 128U;
  p->ry = 128U; p->ly = 128U;
  switch (dstage)
  {
  case 0: /* 冷启动回中性预热（前置自检已由 preamble 跑完一轮全过） */
    if (dn >= 40U) { dstage = 1; dmark = tick; }
    break;
  case 1: /* R1 → 主菜单 */
    p->buttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { dstage = 2; dmark = tick; }
    break;
  case 2: /* 释放 R1 ≥3 稳定帧 */
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - dmark) >= 30U) { dstage = 3; dmark = tick; }
    break;
  case 3: /* 按住 L1：主菜单 → 驾驶页，并持续按住 */
    p->buttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_DRIVE) { dstage = 4; dmark = tick; }
    break;
  case 4: /* 按住 L1 超过 200 ms：四轴一直回中，也不许武装 */
    p->buttons = 0xFBFFU;
    if ((uint32_t)(tick - dmark) >= 200U)
    {
      int i;
      ui_snapshot_get(&snap);
      assert(snap.page == UI_PAGE_DRIVE);
      assert(snap.drive_permitted == 0U);
      for (i = 0; i < 4; i++) { assert(htim8.compare[i] == 0U); }
      dstage = 5; dmark = tick;
    }
    break;
  case 5: /* 释放 L1：重新计时；满 100 ms 前不武装 */
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - dmark) >= 150U)
    {
      ui_snapshot_get(&snap);
      assert(snap.drive_permitted == 1U); /* 释放后重新计满才武装 */
      dstage = 6; dmark = tick;
    }
    else if ((uint32_t)(tick - dmark) >= 60U &&
             (uint32_t)(tick - dmark) < 100U)
    {
      ui_snapshot_get(&snap);
      assert(snap.drive_permitted == 0U); /* 释放后 60~99 ms：还没武装 */
    }
    break;
  case 6: /* 武装后推满：出力 8400 */
    p->ly = 0U;
    if ((uint32_t)(tick - dmark) >= 30U)
    {
      int i;
      ui_snapshot_get(&snap);
      assert(snap.drive_permitted == 1U);
      for (i = 0; i < 4; i++) { assert(htim8.compare[i] == 8400U); }
      longjmp(done, 1);
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
 * 每步以快照为转移条件，帧间留足 3 帧去抖。
 * 确认页已是 5 s 倒计时新流程：无焦点选择，case 12 的 L1 直接启动。 */
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
  case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8: case 9:
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
  case 21: /* 确认页：新流程无焦点，摇杆已回中，直接进 L1 重跑 */
    cbuttons = 0xFFFFU; cly = 128U;
    if ((uint32_t)(tick - cmark) >= 30U) { cstage = 22; cmark = tick; }
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

/* --- selfcheck frame-loss scenario ('k') -------------------------------- */
/* spec §11.9：自检 RUNNING 中出现无效手柄帧（坏帧/失联）即停止并取消。
 * 断言：下一控制周期四轮 compare 全 0、受试轮定格失败（3）、服务退出
 * RUNNING（action_busy=0）、passed=0；重连不自动续跑。 */
static int scframe_test;
static int kstage;
static uint32_t kn, kmark, kbad;

static void selfcheck_frame_loss_script(ps2_state_t *p)
{
  ui_snapshot_t snap;

  kn++;
  switch (kstage)
  {
  case 0: /* 冷启动预热（preamble 已跑过一轮全过自检、页面回品牌页） */
    p->buttons = 0xFFFFU; p->lx = 128U; p->ly = 128U;
    if (kn >= 10U) { cstage = 0; cmark = tick; kstage = 1; }
    break;
  case 1: /* 复用自检导航：品牌页 → 确认页 */
    selfcheck_nav(&cstage, &cmark);
    p->buttons = cbuttons; p->lx = clx; p->ly = cly;
    if (cstage == 12) { kstage = 2; }
    break;
  case 2: /* L1：确认页"开始" → 自检启动 */
    p->buttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_RUN) { kstage = 3; kmark = tick; }
    break;
  case 3: /* 释放 L1，等服务真跑起来（state[0]==1）后开始注入 */
    p->buttons = 0xFFFFU;
    ui_snapshot_get(&snap);
    if (snap.selfcheck_state[0] == 1U)
    {
      crank_on = 1U;
      crank_t0 = tick;
      kstage = 4; kmark = tick;
    }
    break;
  case 4: /* RUNNING 注入反馈约 200 ms，确认服务独占输出 */
    selfcheck_inject((uint32_t)(tick - crank_t0));
    assert_selfcheck_output_pattern();
    if ((uint32_t)(tick - kmark) >= 200U)
    {
      crank_on = 0U;
      kstage = 5;
      kbad = 0U;
    }
    break;
  case 5: /* 帧无效（ps2_poll 改回 BAD）：两拍内必须已取消并清零输出 */
    kbad++;
    ui_snapshot_get(&snap);
    if (kbad >= 2U)
    {
      int i;
      for (i = 0; i < 4; i++) { assert(htim8.compare[i] == 0U); }
      assert(snap.selfcheck_state[0] == 3U); /* 受试轮定格失败 */
      assert(snap.action_busy == 0U);        /* 服务已退出 RUNNING */
      assert(snap.selfcheck_passed == 0U);   /* 取消不通过 */
      longjmp(done, 1);
    }
    break;
  default:
    break;
  }
}

/* --- homing frame-loss scenario ('n') ----------------------------------- */
/* spec §11.9：归位 RUNNING 中出现无效手柄帧当拍停止并取消——不再等
 * wheel_home 内部 200 ms 帧失联兜底。断言：首帧失效后两个控制周期内
 * 四轮 compare 全 0、result=CANCELLED。 */
static int homefl_test;
static int nstage;
static uint32_t nn, nmark;
static uint8_t nbad;

static void home_frame_loss_script(void)
{
  wheel_home_output_t st;

  nn++;
  switch (nstage)
  {
  case 0: /* 复用归位导航：L1 提交归位进 HOME 页（按住中） */
    hcycles++;
    if (hstage < 20) { home_nav(); }
    if (hstage >= 20) { nstage = 1; nmark = tick; }
    break;
  case 1: /* 继续按住 L1 60 ms（同一次提交不得取消请求） */
    if ((uint32_t)(tick - nmark) >= 60U) { nstage = 2; nmark = tick; }
    break;
  case 2: /* 释放 → 放行门 100 ms → RUNNING（C/D 出力） */
    hbuttons = 0xFFFFU;
    wheel_home_get_status(&st);
    if (st.mode == WHEEL_HOME_RUNNING)
    {
      int i, moved = 0;
      for (i = 0; i < 4; i++) { if (htim8.compare[i] != 0U) { moved = 1; } }
      assert(moved != 0); /* 先确认真的在出力 */
      nstage = 3; nmark = tick;
    }
    if ((uint32_t)(tick - nmark) >= 2000U)
    {
      puts("homing never started"); assert(0);
    }
    break;
  case 3: /* 首帧失效（ps2_poll 改回 BAD） */
    nbad = 1U;
    nstage = 4; nmark = tick;
    break;
  case 4: /* 两个控制周期内（远小于 200 ms 兜底）：必须已停并取消 */
    wheel_home_get_status(&st);
    if ((uint32_t)(tick - nmark) >= 6U)
    {
      int i;
      for (i = 0; i < 4; i++) { assert(htim8.compare[i] == 0U); }
      assert(st.mode != WHEEL_HOME_RUNNING);
      assert(st.result == WHEEL_HOME_RESULT_CANCELLED);
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
      /* 复用自检导航：从主菜单起到确认页（stage 2..9）。 */
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

/* --- 公共自检前置（真实菜单 + 真实 selfcheck_service）------------------- */
/* 新语义下驾驶出力要求"本次上电自检已通过"（Ctrl 内服务状态）。
 * 需要出力的场景先用真实菜单跑一轮全过的自检：品牌页 R1 → 菜单 →
 * 右移选"检查与标定" → L1 → 子菜单 → 下移"重新自检" → L1 → 确认页 →
 * 焦点"开始" → L1 → RUNNING 注入反馈 → 四轮全过 → 结果页 R1 回品牌页。
 * 复用 selfcheck_nav（cstage 0..11）与 selfcheck_inject；cn 作帧计数。 */
static int sc_pre_done;
static uint8_t sc_pre_crank;
static uint32_t sc_pre_crank_t0;

static void pre_reset(void)
{
  sc_pre_done = 0; sc_pre_crank = 0;
  cstage = 100; cmark = tick; cn = 0U; crank_on = 0U;
}

static void selfcheck_preamble_step(void)
{
  ui_snapshot_t snap;

  cn++;
  if (cstage <= 9) { selfcheck_nav(&cstage, &cmark); return; }
  switch (cstage)
  {
  case 100: /* 冷启动回中性预热：链路建立 + 再武装 100 ms */
    cbuttons = 0xFFFFU; clx = 128U; cly = 128U;
    if (cn >= 40U) { cstage = 0; cmark = tick; }
    break;
  case 12: /* L1：确认页"开始" → 自检启动 */
    cbuttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_RUN) { cstage = 13; cmark = tick; }
    break;
  case 13: /* 释放 L1，等服务真跑起来（state[0]==1）后开始注入 */
    cbuttons = 0xFFFFU;
    ui_snapshot_get(&snap);
    if (snap.selfcheck_state[0] == 1U)
    {
      sc_pre_crank = 1U;
      sc_pre_crank_t0 = tick;
      cstage = 14;
    }
    break;
  case 14: /* 全程注入直到四轮全过 */
    if (sc_pre_crank != 0U)
    {
      selfcheck_inject((uint32_t)(tick - sc_pre_crank_t0));
    }
    ui_snapshot_get(&snap);
    if (snap.selfcheck_state[0] == 2U && snap.selfcheck_state[1] == 2U &&
        snap.selfcheck_state[2] == 2U && snap.selfcheck_state[3] == 2U)
    {
      sc_pre_crank = 0U;
      assert(snap.selfcheck_passed == 1U);
      cstage = 15;
    }
    break;
  case 15: /* 等页面落到结果页（晚服务完成一拍，只允许这一帧滞后） */
    cbuttons = 0xFFFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_RESULT) { cstage = 16; cmark = tick; }
    else
    {
      assert(snap.page == UI_PAGE_CHECK_RUN);
      assert((uint32_t)(tick - cmark) < 200U);
    }
    break;
  case 16: /* 结果页 R1 → 品牌页 */
    cbuttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_BRAND) { cstage = 17; cmark = tick; }
    break;
  case 17: /* 释放 R1 ≥3 稳定帧 */
    cbuttons = 0xFFFFU;
    if ((uint32_t)(tick - cmark) >= 30U) { cstage = 18; cmark = tick; }
    break;
  case 18: /* 品牌页 R1 → 主菜单：把导航遗留的 menu_sel 复位到"驾驶" */
    cbuttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { cstage = 19; cmark = tick; }
    break;
  case 19:
    cbuttons = 0xFFFFU;
    if ((uint32_t)(tick - cmark) >= 30U) { cstage = 20; cmark = tick; }
    break;
  case 20: /* 左移到"驾驶"（menu_sel 0） */
    cbuttons = 0xFFFFU;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 0U) { clx = 128U; cstage = 21; cmark = tick; }
    else { clx = 48U; }
    break;
  case 21: /* 摇杆回中 */
    cbuttons = 0xFFFFU; clx = 128U;
    if ((uint32_t)(tick - cmark) >= 30U) { cstage = 22; cmark = tick; }
    break;
  case 22: /* 菜单页 R1 → 品牌页（后续场景都从品牌页起导航） */
    cbuttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_BRAND) { cstage = 23; cmark = tick; }
    break;
  case 23:
    cbuttons = 0xFFFFU;
    if ((uint32_t)(tick - cmark) >= 30U) { sc_pre_done = 1; }
    break;
  default:
    break;
  }
}

/* --- VDD 掉压闸门场景 ('p') --------------------------------------------- */
/* 前置（vdd_ok=1）：真实跑一轮全过自检 + 手动标定页记录参考。
 * 1) 驾驶页武装后推满出力 → 运行中注入 3000 mV：出力当拍切断并睡眠；
 * 2) 掉压下发起归位：请求不受理，放行门后也不进 RUNNING（不出力）；
 * 3) 轨恢复后归位进 RUNNING（C/D 出力）→ RUNNING 中再掉压：命令当拍清零，
 *    wheel_home 走既有 200 ms 帧失联取消路径落成 CANCELLED（立即停）；
 * 4) 掉压下发起自检：服务请求被拒（state 停留上一轮的 2，无 1/3）。 */
static int power_gate_test;
static int pstage;
static uint32_t pmark, pn;
static uint16_t p_frozen_remain; /* 掉压冻结时确认页剩余毫秒（不减断言用） */

static void power_gate_script(ps2_state_t *p)
{
  ui_snapshot_t snap;
  wheel_home_output_t hs;

  pn++;
  p->buttons = 0xFFFFU; p->lx = 128U; p->rx = 128U;
  p->ry = 128U; p->ly = 128U;
  switch (pstage) {
  /* ---- 前置：手动标定页记录参考（vdd 正常） ---- */
  case 0: /* R1 → 主菜单 */
    p->buttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { pstage = 1; pmark = tick; }
    break;
  case 1:
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 2; pmark = tick; }
    break;
  case 2: /* 右移选"检查与标定" */
    p->lx = 208U;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 1U) { p->lx = 128U; pstage = 3; pmark = tick; }
    break;
  case 3:
    p->lx = 128U;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 4; pmark = tick; }
    break;
  case 4: /* L1 → 子菜单 */
    p->buttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { pstage = 5; pmark = tick; }
    break;
  case 5:
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 6; pmark = tick; }
    break;
  case 6: /* 下移到"手动标定" */
    ui_snapshot_get(&snap);
    if (snap.sub_sel == 3U) { p->ly = 128U; pstage = 7; pmark = tick; }
    else { p->ly = 208U; }
    break;
  case 7:
    p->ly = 128U;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 8; pmark = tick; }
    break;
  case 8: /* L1 → 手动标定页 */
    p->buttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MANUAL_CAL) { pstage = 9; pmark = tick; }
    break;
  case 9: /* 进页先见 SELECT 释放 */
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 60U) { pstage = 10; pmark = tick; }
    break;
  case 10: /* 静止 ≥300 ms 后新长按 2 s：记录参考 */
    p->buttons = 0xFFFEU;
    if ((uint32_t)(tick - pmark) >= 2500U) {
      p->buttons = 0xFFFFU;
      pstage = 11; pmark = tick;
    }
    break;
  case 11:
    if ((uint32_t)(tick - pmark) >= 100U) {
      wheel_home_get_status(&hs);
      assert(hs.reference_valid == 1U);
      pstage = 12; pmark = tick;
    }
    break;
  /* ---- 驾驶页出力 → 运行中掉压：当拍切断 ---- */
  case 12: /* R1 → 子菜单（手动标定页返回第一跳） */
    p->buttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { pstage = 13; pmark = tick; }
    break;
  case 13:
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 14; pmark = tick; }
    break;
  case 14: /* R1 → 菜单 */
    p->buttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { pstage = 15; pmark = tick; }
    break;
  case 15:
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 16; pmark = tick; }
    break;
  case 16: /* 左移回"驾驶" */
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 0U) { p->lx = 128U; pstage = 17; pmark = tick; }
    else { p->lx = 48U; }
    break;
  case 17:
    p->lx = 128U;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 18; pmark = tick; }
    break;
  case 18: /* L1 → 驾驶页 */
    p->buttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_DRIVE) { pstage = 19; pmark = tick; }
    break;
  case 19:
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 20; pmark = tick; }
    break;
  case 20: /* 推满：武装前零输出 */
    p->ly = 0U;
    ui_snapshot_get(&snap);
    if ((uint32_t)(tick - pmark) >= 60U && (uint32_t)(tick - pmark) < 100U) {
      assert(snap.drive_permitted == 0U);
      for (int i = 0; i < 4; i++) assert(htim8.compare[i] == 0U);
    }
    if ((uint32_t)(tick - pmark) >= 120U) { pstage = 21; pmark = tick; }
    break;
  case 21: /* 松杆回中 200 ms：武装 */
    p->ly = 128U;
    if ((uint32_t)(tick - pmark) >= 200U) { pstage = 22; pmark = tick; }
    break;
  case 22: /* 再推满：vdd_ok=1 + 已过自检 → 出力 8400，随即注入电池欠压 */
    p->ly = 0U;
    if ((uint32_t)(tick - pmark) >= 60U) {
      ui_snapshot_get(&snap);
      assert(snap.drive_permitted == 1U);
      assert(snap.vdd_ok == 1U);
      for (int i = 0; i < 4; i++) assert(htim8.compare[i] == 8400U);
      /* 电池域欠压注入（vdd 轨仍正常）：spec §9.2 配置有效后的欠压
       * 必须进入供电故障停机——与 vdd_ok 无关地切断出力。 */
      s_pm_stub.battery_configured = 1U;
      s_pm_stub.battery_valid = 1U;
      s_pm_stub.battery_mv = 3000U;
      s_pm_stub.undervolt = 1U;
      pstage = 60; pmark = tick;
    }
    break;
  case 60: /* 欠压（vdd_ok 仍为 1）：驾驶输出当拍切断 */
    p->ly = 0U;
    ui_snapshot_get(&snap);
    if ((uint32_t)(tick - pmark) >= 30U) {
      assert(snap.undervolt == 1U);
      assert(snap.drive_permitted == 0U);
      for (int i = 0; i < 4; i++) assert(htim8.compare[i] == 0U);
      /* 还原电池域，继续原有 vdd_ok 掉压链路 */
      s_pm_stub.battery_configured = 0U;
      s_pm_stub.battery_valid = 0U;
      s_pm_stub.battery_mv = 0U;
      s_pm_stub.undervolt = 0U;
      pm_set_vdd_ok(0U); /* 运行中轨压掉到 3000 mV */
      pstage = 23; pmark = tick;
    }
    break;
  case 23: /* 供电异常：出力当拍切断并睡眠 */
    p->ly = 0U;
    ui_snapshot_get(&snap);
    if ((uint32_t)(tick - pmark) >= 150U) {
      assert(snap.vdd_ok == 0U);
      assert(snap.drive_permitted == 0U);
      for (int i = 0; i < 4; i++) assert(htim8.compare[i] == 0U);
      assert(!(gpiob.ODR & GPIO_PIN_12)); /* 停止流程关使能 */
      p->ly = 128U;
      pstage = 24; pmark = tick;
    }
    break;
  /* ---- 掉压下发起归位：请求不受理，放行门后也不进 RUNNING ---- */
  case 24: /* R1 → 主菜单 */
    p->buttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_MENU) { pstage = 25; pmark = tick; }
    break;
  case 25:
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 26; pmark = tick; }
    break;
  case 26: /* 右移选"检查与标定" */
    p->lx = 208U;
    ui_snapshot_get(&snap);
    if (snap.menu_sel == 1U) { p->lx = 128U; pstage = 27; pmark = tick; }
    break;
  case 27:
    p->lx = 128U;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 28; pmark = tick; }
    break;
  case 28: /* L1 → 子菜单 */
    p->buttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { pstage = 29; pmark = tick; }
    break;
  case 29:
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 30; pmark = tick; }
    break;
  case 30: /* 从"手动标定"(3) 上移一格到"回到初始位"(2) */
    ui_snapshot_get(&snap);
    if (snap.sub_sel == 2U) { p->ly = 128U; pstage = 31; pmark = tick; }
    else { p->ly = 48U; }
    break;
  case 31:
    p->ly = 128U;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 32; pmark = tick; }
    break;
  case 32: /* L1 → HOME 页（掉压：请求必须被拒） */
    p->buttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_HOME) { pstage = 33; pmark = tick; }
    break;
  case 33: /* 同次 L1 按住 60 ms 后释放 */
    p->buttons = 0xFBFFU;
    if ((uint32_t)(tick - pmark) >= 60U) { pstage = 34; pmark = tick; }
    break;
  case 34: /* 释放 500 ms：请求被拒 → 永远不进 RUNNING、不出力；
            * （若请求被受理，参考有效 + 放行门 100 ms 后必然 RUNNING） */
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 500U) {
      wheel_home_get_status(&hs);
      assert(hs.reference_valid == 1U); /* 掉压前记录的参考不受影响 */
      assert(hs.mode != WHEEL_HOME_RUNNING);
      for (int i = 0; i < 4; i++) assert(htim8.compare[i] == 0U);
      pstage = 35; pmark = tick;
    }
    break;
  /* ---- 轨恢复 → 归位 RUNNING（C/D 出力）→ RUNNING 中掉压：立即停 ---- */
  case 35: /* 轨恢复；给 C/D 各注入 1/8 圈（分拍 < 32768，相对增量） */
    pm_set_vdd_ok(1U);
    {
      static int inj;
      if ((pn % 2U) == 0U && inj < 4) {
        htim5.counter = (uint16_t)(htim5.counter - 13437U); /* C 前向为负 raw */
        htim4.counter = (uint16_t)(htim4.counter + 13397U); /* D 前向为正 raw */
        inj++;
      }
      if (inj >= 4) { pstage = 36; pmark = tick; }
    }
    break;
  case 36: /* R1 → 子菜单 */
    p->buttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { pstage = 37; pmark = tick; }
    break;
  case 37:
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 38; pmark = tick; }
    break;
  case 38: /* L1（sub_sel==2）→ HOME 页 + pending */
    p->buttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_HOME) { pstage = 39; pmark = tick; }
    break;
  case 39: /* 同次 L1 按住 60 ms（不得取消 pending） */
    p->buttons = 0xFBFFU;
    if ((uint32_t)(tick - pmark) >= 60U) { pstage = 40; pmark = tick; }
    break;
  case 40: /* 释放 → 放行门 100 ms → RUNNING（C/D 出力） */
    p->buttons = 0xFFFFU;
    wheel_home_get_status(&hs);
    if (hs.mode == WHEEL_HOME_RUNNING) {
      int moved = 0;
      for (int i = 0; i < 4; i++) {
        assert(htim8.compare[i] <= WHEEL_HOME_MAX_PULSE);
        if (htim8.compare[i] != 0U) { moved = 1; }
      }
      if (moved) { pstage = 41; pmark = tick; }
    } else if ((uint32_t)(tick - pmark) >= 2000U) {
      assert(0); /* 归位从未启动 */
    }
    break;
  case 41: /* RUNNING 中掉压：命令当拍清零，状态机随后落成 CANCELLED */
    pm_set_vdd_ok(0U);
    pstage = 42; pmark = tick;
    break;
  case 42:
    p->buttons = 0xFFFFU;
    wheel_home_get_status(&hs);
    if ((uint32_t)(tick - pmark) >= 120U) {
      for (int i = 0; i < 4; i++) assert(htim8.compare[i] == 0U); /* 立即停 */
    }
    if ((uint32_t)(tick - pmark) >= 500U) {
      assert(hs.mode != WHEEL_HOME_RUNNING);
      assert(hs.result == WHEEL_HOME_RESULT_CANCELLED);
      pstage = 43; pmark = tick;
    }
    break;
  /* ---- 掉压下发起自检：服务请求被拒 ---- */
  case 43: /* R1 → 子菜单 */
    p->buttons = 0xF7FFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_SUBMENU) { pstage = 44; pmark = tick; }
    break;
  case 44:
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 45; pmark = tick; }
    break;
  case 45: /* 上移到"重新自检" */
    ui_snapshot_get(&snap);
    if (snap.sub_sel == 1U) { p->ly = 128U; pstage = 46; pmark = tick; }
    else { p->ly = 48U; }
    break;
  case 46:
    p->ly = 128U;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 47; pmark = tick; }
    break;
  case 47: /* L1 → 确认页 */
    p->buttons = 0xFBFFU;
    ui_snapshot_get(&snap);
    if (snap.page == UI_PAGE_CHECK_CONFIRM) { pstage = 48; pmark = tick; }
    break;
  case 48:
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 49; pmark = tick; }
    break;
  case 49: /* ready=0（vdd 掉压）：倒计时冻结，仍停留确认页 */
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 30U) {
      ui_snapshot_get(&snap);
      assert(snap.page == UI_PAGE_CHECK_CONFIRM);
      assert(snap.confirm_blocked == 1U);
      p_frozen_remain = snap.confirm_remain_ms;
      pstage = 495; pmark = tick;
    }
    break;
  case 495: /* 冻结 600 ms：remain 不减、不自发启动 */
    if ((uint32_t)(tick - pmark) >= 600U) {
      ui_snapshot_get(&snap);
      assert(snap.page == UI_PAGE_CHECK_CONFIRM);
      assert(snap.confirm_blocked == 1U);
      assert(snap.confirm_remain_ms == p_frozen_remain);
      pstage = 50; pmark = tick;
    }
    break;
  case 50: /* ready=0 时 L1：模型不发请求、不转页（页面显示原因） */
    p->buttons = 0xFBFFU;
    if ((uint32_t)(tick - pmark) >= 60U) {
      ui_snapshot_get(&snap);
      assert(snap.page == UI_PAGE_CHECK_CONFIRM);
      pstage = 51; pmark = tick;
    }
    break;
  case 51: /* 释放 L1 */
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 30U) { pstage = 52; pmark = tick; }
    break;
  case 52: /* 1.5 s 后：服务从未启动——state 停留在上一轮全过的 2，
            * 不出现 1（运行）/3（失败），无任何输出，页面仍被挡 */
    p->buttons = 0xFFFFU;
    if ((uint32_t)(tick - pmark) >= 1500U) {
      ui_snapshot_get(&snap);
      assert(snap.page == UI_PAGE_CHECK_CONFIRM);
      assert(snap.confirm_blocked == 1U);
      assert(snap.action_busy == 0U);
      assert(snap.vdd_ok == 0U);
      for (int i = 0; i < 4; i++) {
        assert(snap.selfcheck_state[i] == 2U); /* 前置自检的保留结果 */
        assert(htim8.compare[i] == 0U);
      }
      longjmp(done, 1);
    }
    break;
  default:
    break;
  }
}

uint8_t ps2_poll(ps2_state_t *p){
 /* 需要出力的场景先跑一轮全过的真实自检（'c' 场景本身就在测自检，跳过）。 */
 if(!selfcheck_test && !sc_pre_done){
   selfcheck_preamble_step();
   p->buttons=cbuttons;p->lx=clx;p->ly=cly;p->rx=128U;p->ry=128U;
   tick+=3U;return PS2_FRAME_OK;
 }
 if(power_gate_test){ power_gate_script(p); tick+=3U; return PS2_FRAME_OK; }
 if(scframe_test){
   selfcheck_frame_loss_script(p);
   tick+=3U;
   /* kstage>=5：模拟坏帧/失联——本帧无效。 */
   return (kstage >= 5) ? PS2_FRAME_BAD : PS2_FRAME_OK;
 }
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
     /* “重新自检通过才清锁存”走的是真实的 selfcheck_service 路径，
      * 由本测试的 'x' 场景（relatch）覆盖；本场景到此确认锁存即可。 */
     tick+=3U;
     return PS2_FRAME_OK;
   }
   /* 锁存后保持静止若干帧，确认不会自行恢复出力。 */
   {
     static uint32_t rec;
     rec++;
     if(rec<10U){ tick+=3U; return PS2_FRAME_OK; }
   }
   for(int i=0;i<4;i++)assert(htim8.compare[i]==0U);
   longjmp(done,1);
 }
 /* 冷启动再武装：摇杆回中且 SELECT 释放必须持续 100 ms（tick 每轮 +3 ms），
  * 因此冷启动场景先喂 PREARM_FRAMES 帧中性，之后才进被测流程。
  * 该预滚对所有冷启动场景生效（含 reconnect 的首段），避免推杆本身
  * 打断居中计时导致再武装永远无法完成。 */
 if(ui_test){ ui_script(p); tick+=3U; return PS2_FRAME_OK; }
 if(cap_test){ cap_script(p); tick+=3U; return PS2_FRAME_OK; }
 if(rearm_test){ rearm_script(p); tick+=3U; return PS2_FRAME_OK; }
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
 if(homefl_test){home_frame_loss_script();p->buttons=hbuttons;p->lx=hlx;p->ly=hly;
   p->rx=128U;p->ry=128U;tick+=3U;
   return (nbad!=0U)?PS2_FRAME_BAD:PS2_FRAME_OK;}
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
 /* 自检前置（pw.vdd_valid）对所有需要出力的场景生效。 */
 pm_set_vdd_ok(1U);
 if(argc>1 && argv[1][0]=='b'){ verbose_test=(argc>2); pre_reset(); if(!setjmp(done))StartCtrlTask(0); puts("basic ok"); return 0; }
 if(argc>1 && argv[1][0]=='v'){ verbose_test=1; pre_reset(); if(!setjmp(done))StartCtrlTask(0); puts("end"); return 0; }
 /* 归位场景单独一个进程：状态机是全静态的，与上面几个场景共享会
  * 带入 50 ms 采样新鲜度时间戳，混淆被测行为。 */
 if(argc>1 && argv[1][0]=='f'){
   fault_test=1;tick=0U;fault_t0=0U;fault_frames=0U;
   fault_t0=tick;
   pre_reset();
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl motor health latch / zero output / stays latched OK");return 0;
 }
 if(argc>1 && argv[1][0]=='s'){
   stop_test=1;tick=0U;
   pre_reset();
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl stop closes drive enable");return 0;
 }
 if(argc>1 && argv[1][0]=='h'){
   home_test=1;tick=0U;phase=0;
   /* boot_ui 不再参与电机归属：从上电起 Ctrl 就独占电机（首帧零命令 →
    * 睡眠）。pwm_ccer 清零，保证 ctrl_apply 的唤醒分支必须真的启动
    * 通道——否则归位相序推不动（回归上板"只写 CCR 不启动通道"问题）。 */
   pwm_ccer=0U;
   pre_reset();
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl homing arbitration / rear encoders OK");return 0;
 }
 if(argc>1 && argv[1][0]=='u'){
   ui_test=1;tick=0U;
   pre_reset();
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl ui snapshot / page-gated drive / dual-axis mix OK");return 0;
 }
 if(argc>1 && argv[1][0]=='c'){
   selfcheck_test=1;tick=0U;cn=0U;cstage=0;crank_on=0U;
   pm_set_vdd_ok(1U);
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl menu selfcheck / service-owned output / 5s window OK");return 0;
 }
 if(argc>1 && argv[1][0]=='o'){
   cap_test=1;tick=0U;capn=0U;ostage=0;
   pm_set_vdd_ok(1U);
   pre_reset();
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl output cap settings page / runtime limit to compare OK");return 0;
 }
 if(argc>1 && argv[1][0]=='p'){
   power_gate_test=1;tick=0U;pn=0U;pstage=0;
   pre_reset();
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl vdd-undervolt gate / drive cut / selfcheck refused OK");return 0;
 }
 if(argc>1 && argv[1][0]=='k'){
   scframe_test=1;tick=0U;kn=0U;kstage=0;kbad=0U;crank_on=0U;
   pm_set_vdd_ok(1U);
   pre_reset();
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl selfcheck frame-loss cancels service / no resume OK");return 0;
 }
 if(argc>1 && argv[1][0]=='x'){
   relatch_test=1;tick=0U;xn=0U;xstage=0;xcrank_on=0U;
   pm_set_vdd_ok(1U);
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl failed recheck keeps latch / pass+neutral restores OK");return 0;
 }
 if(argc>1 && argv[1][0]=='n'){
   homefl_test=1;tick=0U;phase=0;pwm_ccer=0U;
   nstage=0;nn=0U;nbad=0U;hstage=0;hcycles=0;
   pm_set_vdd_ok(1U);
   pre_reset();
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl homing frame-loss cancels same-cycle OK");return 0;
 }
 if(argc>1 && argv[1][0]=='d'){
   rearm_test=1;tick=0U;dn=0U;dstage=0;
   pm_set_vdd_ok(1U);
   pre_reset();
   if(!setjmp(done))StartCtrlTask(0);
   puts("ctrl drive rearm requires button release OK");return 0;
 }
 pre_reset();
 if(!setjmp(done))StartCtrlTask(0);
 reconnect_test=1;reconnect_seen=1;phase=0;tick=1000;
 pre_reset();
 if(!setjmp(done))StartCtrlTask(0);
 phase=0;tick=UINT32_MAX-100U;
 pre_reset();
 if(!setjmp(done))StartCtrlTask(0);
 phase=0;drive_calibration_mode=1;
 pre_reset();
 if(!setjmp(done))StartCtrlTask(0);
 drive_calibration_mode=0;

 puts("ctrl wake / elapsed loss deadline / reconnect / tick wrap / cal sleep OK");
 return 0;
}
