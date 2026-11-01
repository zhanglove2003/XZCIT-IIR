/* Task 2：ui_model 纯页面状态机测试（assert 风格，同 test_manual_ref.c 组织）。
 * 全部确定性 tick：10 ms 一帧，按键 3 连续有效帧稳定，摇杆阈值 48/24、
 * 连移 400/180 ms。低有效键位：R1=bit11，L1=bit10，SELECT=bit0。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../Core/Inc/ui_model.h"

#define R1B  ((uint16_t)(1U << 11))
#define L1B  ((uint16_t)(1U << 10))
#define SELB ((uint16_t)(1U << 0))

typedef struct
{
  ui_model_t m;
  ui_model_in_t in;
  ui_model_req_t req;
  uint32_t now;
} rig_t;

static void r_step(rig_t *r)
{
  r->in.now_ms = r->now;
  ui_model_step(&r->m, &r->in, &r->req);
  r->now += 10U;
}

static void r_press(rig_t *r, uint16_t bits)
{
  r->in.buttons = (uint16_t)(r->in.buttons & (uint16_t)~bits);
}

static void r_release(rig_t *r, uint16_t bits)
{
  r->in.buttons = (uint16_t)(r->in.buttons | bits);
}

static void r_hold(rig_t *r, uint16_t bits, int n)
{
  r_press(r, bits);
  while (n-- > 0)
  {
    r_step(r);
  }
}

static void r_free(rig_t *r, uint16_t bits, int n)
{
  r_release(r, bits);
  while (n-- > 0)
  {
    r_step(r);
  }
}

static void rig_init(rig_t *r, uint32_t now)
{
  memset(r, 0, sizeof(*r));
  ui_model_init(&r->m, now);
  r->in.buttons = 0xFFFFu;
  r->in.lx = 128U;
  r->in.ly = 128U;
  r->in.rx = 128U;
  r->in.ry = 128U;
  r->in.frame_valid = 1U;
  r->in.now_ms = now;
  r->now = now;
}

static void req_all_zero(const rig_t *r)
{
  assert(r->req.selfcheck_start == 0U);
  assert(r->req.home_start == 0U);
  assert(r->req.record_ref == 0U);
  assert(r->req.cancel_motion == 0U);
}

/* BRAND -> SUBMENU，并把 sub_sel 移到 sel（前提：page==BRAND，menu_sel==0）。 */
static void goto_submenu(rig_t *r, uint8_t sel)
{
  r_hold(r, R1B, 3);
  r_free(r, R1B, 3);
  assert(r->m.page == UI_PAGE_MENU);
  r->in.lx = 176U;
  r_step(r);
  r->in.lx = 128U;
  r_step(r);
  assert(r->m.menu_sel == 1U);
  r_hold(r, L1B, 3);
  r_free(r, L1B, 3);
  assert(r->m.page == UI_PAGE_SUBMENU);
  while (r->m.sub_sel < sel)
  {
    r->in.ly = 200U;
    r_step(r);
    r->in.ly = 128U;
    r_step(r);
  }
}

/* 走到 CHECK_RUN（经子菜单"重新自检"→确认页 L1 立即开始），busy 未置位。 */
static void goto_check_run(rig_t *r)
{
  goto_submenu(r, 1U);
  r_hold(r, L1B, 3);
  r_free(r, L1B, 3);
  assert(r->m.page == UI_PAGE_CHECK_CONFIRM);
  r->in.selfcheck_ready = 1U; /* 前置满足：L1 才能立即启动 */
  r_hold(r, L1B, 3);
  r_free(r, L1B, 3);
  assert(r->m.page == UI_PAGE_CHECK_RUN);
}

/* 走到 CHECK_CONFIRM（经子菜单"重新自检"），ready 按给定值喂。 */
static void goto_confirm(rig_t *r, uint8_t ready)
{
  goto_submenu(r, 1U);
  r->in.selfcheck_ready = ready;
  r_hold(r, L1B, 3);
  r_free(r, L1B, 3);
  assert(r->m.page == UI_PAGE_CHECK_CONFIRM);
}

int main(void)
{
  /* 1. 导航：BRAND--R1-->MENU--L1-->DRIVE；SUBMENU R1 回 MENU 且 sub_sel 保留；
   *    CHECK_RESULT R1 回 SUBMENU 原条目。 */
  {
    rig_t r;
    rig_init(&r, 1000U);
    assert(r.m.page == UI_PAGE_BRAND);
    r_hold(&r, R1B, 2);
    assert(r.m.page == UI_PAGE_BRAND); /* 未满 3 帧 */
    r_step(&r);
    assert(r.m.page == UI_PAGE_MENU);
    r_free(&r, R1B, 3);
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_DRIVE); /* menu_sel==0 → 驾驶 */
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    assert(r.m.page == UI_PAGE_MENU);
    assert(r.m.menu_sel == 0U);
    r.in.lx = 176U;
    r_step(&r);
    assert(r.m.menu_sel == 1U);
    r.in.lx = 128U;
    r_step(&r);
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_SUBMENU);
    r.in.ly = 200U;
    r_step(&r);
    r.in.ly = 128U;
    r_step(&r);
    r.in.ly = 200U;
    r_step(&r);
    r.in.ly = 128U;
    r_step(&r);
    assert(r.m.sub_sel == 2U);
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    assert(r.m.page == UI_PAGE_MENU);
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_SUBMENU);
    assert(r.m.sub_sel == 2U); /* 保留 */
    r.in.ly = 56U;
    r_step(&r);
    r.in.ly = 128U;
    r_step(&r);
    r.in.ly = 56U;
    r_step(&r);
    r.in.ly = 128U;
    r_step(&r);
    assert(r.m.sub_sel == 0U);
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_CHECK_RESULT);
    assert(r.m.back_page == UI_PAGE_SUBMENU);
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    assert(r.m.page == UI_PAGE_SUBMENU);
    assert(r.m.sub_sel == 0U); /* 原条目 */
  }

  /* 2. L1 单发：MENU 按住 L1 连续 20 帧只进入一次；确认页 selfcheck_start 只发一帧。 */
  {
    rig_t r;
    int i;
    rig_init(&r, 1000U);
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    r_hold(&r, L1B, 2);
    assert(r.m.page == UI_PAGE_MENU);
    r_step(&r);
    assert(r.m.page == UI_PAGE_DRIVE);
    for (i = 0; i < 17; i++)
    {
      r_step(&r);
      assert(r.m.page == UI_PAGE_DRIVE);
      req_all_zero(&r);
    }
    r_free(&r, L1B, 3);
    for (i = 0; i < 10; i++)
    {
      r_step(&r);
      assert(r.m.page == UI_PAGE_DRIVE);
      req_all_zero(&r);
    }
    /* 确认页：按住 L1 只产生一帧 selfcheck_start */
    rig_init(&r, 1000U);
    r.in.selfcheck_ready = 1U;
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
    r_hold(&r, L1B, 2);
    assert(r.req.selfcheck_start == 0U);
    r_step(&r);
    assert(r.req.selfcheck_start == 1U);
    assert(r.m.page == UI_PAGE_CHECK_RUN);
    for (i = 0; i < 17; i++)
    {
      r_step(&r);
      assert(r.req.selfcheck_start == 0U);
      assert(r.m.page == UI_PAGE_CHECK_RUN);
    }
  }

  /* 3. 进页屏蔽：进 DRIVE 同帧 L1 仍按着 → 不触发 DRIVE 内任何动作
   *    （SELECT 快捷归位也因 L1 按住被抑制）。 */
  {
    rig_t r;
    int i;
    rig_init(&r, 1000U);
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    r_hold(&r, L1B, 3); /* 第 3 帧进 DRIVE，L1 继续按着 */
    assert(r.m.page == UI_PAGE_DRIVE);
    for (i = 0; i < 10; i++)
    {
      r_step(&r);
      req_all_zero(&r);
    }
    r_hold(&r, SELB, 3); /* SELECT 稳定按下：L1 仍按住 → 不发 home_start */
    assert(r.m.page == UI_PAGE_DRIVE);
    assert(r.req.home_start == 0U);
    req_all_zero(&r);
    r_free(&r, L1B | SELB, 3);
    assert(r.m.page == UI_PAGE_DRIVE);
  }

  /* 4. 优先级：R1 与 L1 同帧稳定 → 只出现返回，无确认副作用。 */
  {
    rig_t r;
    int i;
    rig_init(&r, 1000U);
    r_hold(&r, R1B | L1B, 2);
    assert(r.m.page == UI_PAGE_BRAND);
    r_step(&r);
    assert(r.m.page == UI_PAGE_MENU); /* R1 赢 */
    assert(r.m.page != UI_PAGE_CHECK_CONFIRM);
    for (i = 0; i < 5; i++)
    {
      r_step(&r); /* L1 仍按着：不得再触发确认 */
      assert(r.m.page == UI_PAGE_MENU);
      req_all_zero(&r);
    }
    r_free(&r, R1B | L1B, 3);
    assert(r.m.page == UI_PAGE_MENU);
  }

  /* 5. SELECT+确认同帧 → 无运动请求（selfcheck_start/home_start/record_ref 全 0）。 */
  {
    rig_t r;
    int i;
    rig_init(&r, 1000U);
    r.in.selfcheck_ready = 1U;
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
    r_hold(&r, SELB, 3);
    for (i = 0; i < 5; i++)
    {
      r_hold(&r, L1B, 3); /* SELECT 按住期间反复按 L1 */
      r_free(&r, L1B, 3);
      assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
      assert(r.req.selfcheck_start == 0U);
      assert(r.req.home_start == 0U);
      assert(r.req.record_ref == 0U);
    }
    r_free(&r, SELB, 3);
  }

  /* 6. 自检确认页：L1 立即发 selfcheck_start（新流程，无焦点选择）；
   *    busy 结束进结果页；selfcheck_passed=0 进 DRIVE 后 req 无运动请求。 */
  {
    rig_t r;
    int i;
    rig_init(&r, 1000U);
    r.in.selfcheck_ready = 1U;
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
    assert(r.m.confirm_focus == 0U); /* 焦点字段保留但不再参与选择 */
    r_hold(&r, L1B, 2);
    r_step(&r);
    assert(r.req.selfcheck_start == 1U);
    assert(r.m.page == UI_PAGE_CHECK_RUN);
    r_free(&r, L1B, 3);
    r.in.action_busy = 1U;
    r_step(&r);
    r_step(&r);
    r.in.action_busy = 0U;
    r.in.result_window_open = 1U;
    r_step(&r);
    assert(r.m.page == UI_PAGE_CHECK_RESULT);
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    assert(r.m.page == UI_PAGE_BRAND); /* 自检结束的结果页回品牌页 */
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    assert(r.m.page == UI_PAGE_MENU);
    r_hold(&r, L1B, 3); /* menu_sel==0 → DRIVE */
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_DRIVE);
    r.in.selfcheck_passed = 0U;
    for (i = 0; i < 10; i++)
    {
      r_step(&r);
      assert(r.m.page == UI_PAGE_DRIVE);
      assert(r.req.home_start == 0U);
      assert(r.req.selfcheck_start == 0U);
      assert(r.req.record_ref == 0U);
    }
  }

  /* 7. USER 键：BRAND 上稳定 USER → CHECK_CONFIRM；MANUAL_CAL 上 USER → 退回不记录。 */
  {
    rig_t r;
    rig_init(&r, 3000U);
    r.in.user_pressed = 1U;
    r_step(&r);
    r_step(&r);
    assert(r.m.page == UI_PAGE_BRAND);
    r_step(&r);
    assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
    r.in.user_pressed = 0U;
    r_step(&r);
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    assert(r.m.page == UI_PAGE_BRAND);
    goto_submenu(&r, 3U);
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_MANUAL_CAL);
    r.in.user_pressed = 1U;
    r_step(&r);
    r_step(&r);
    assert(r.m.page == UI_PAGE_MANUAL_CAL);
    r_step(&r);
    assert(r.m.page == UI_PAGE_SUBMENU); /* 退出回子菜单 */
    assert(r.req.record_ref == 0U);      /* 不记录 */
    r.in.user_pressed = 0U;
    r_step(&r);
  }

  /* 8. 5 s 标定窗口。 */
  {
    /* a) 结果页窗口内稳定 L1 → MANUAL_CAL */
    rig_t r;
    rig_init(&r, 1000U);
    goto_check_run(&r);
    r.in.action_busy = 1U;
    r_step(&r);
    r.in.action_busy = 0U;
    r.in.result_window_open = 1U;
    r_step(&r);
    assert(r.m.page == UI_PAGE_CHECK_RESULT);
    r_hold(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_MANUAL_CAL);
    r_free(&r, L1B, 3);
    assert(r.m.back_page == UI_PAGE_SUBMENU);

    /* b) 窗口字段清 0（Ctrl 超时）→ page=BRAND */
    rig_init(&r, 1000U);
    goto_check_run(&r);
    r.in.action_busy = 1U;
    r_step(&r);
    r.in.action_busy = 0U;
    r.in.result_window_open = 1U;
    r_step(&r);
    assert(r.m.page == UI_PAGE_CHECK_RESULT);
    r.in.result_window_open = 0U;
    r_step(&r);
    assert(r.m.page == UI_PAGE_BRAND);

    /* c) 结果页窗口内稳定 USER → MANUAL_CAL */
    rig_init(&r, 1000U);
    goto_check_run(&r);
    r.in.action_busy = 1U;
    r_step(&r);
    r.in.action_busy = 0U;
    r.in.result_window_open = 1U;
    r_step(&r);
    assert(r.m.page == UI_PAGE_CHECK_RESULT);
    r.in.user_pressed = 1U;
    r_step(&r);
    r_step(&r);
    r_step(&r);
    assert(r.m.page == UI_PAGE_MANUAL_CAL);
    r.in.user_pressed = 0U;
    r_step(&r);

    /* d) 从 SUBMENU 进入的结果页：back_page==SUBMENU，R1 回原条目，不受窗口影响 */
    rig_init(&r, 1000U);
    goto_submenu(&r, 0U);
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_CHECK_RESULT);
    assert(r.m.back_page == UI_PAGE_SUBMENU);
    r.in.result_window_open = 1U;
    r_step(&r);
    r_step(&r);
    assert(r.m.page == UI_PAGE_CHECK_RESULT); /* 不被窗口清 0 带走 */
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    assert(r.m.page == UI_PAGE_SUBMENU);
    assert(r.m.sub_sel == 0U);
  }

  /* 9. 摇杆阈值：47 不动 48 动；不回 24 以内不复位；400 ms 内一格、之后每 180 ms 一格。 */
  {
    rig_t r;
    int i;
    rig_init(&r, 1000U);
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    r.in.lx = 175U; /* 偏移 47 */
    for (i = 0; i < 5; i++)
    {
      r_step(&r);
      assert(r.m.menu_sel == 0U);
    }
    r.in.lx = 176U; /* 偏移 48 */
    r_step(&r);
    assert(r.m.menu_sel == 1U);
    r.in.lx = 160U; /* 偏移 32：未回 24 以内 */
    r_step(&r);
    assert(r.m.menu_sel == 1U);
    r.in.lx = 128U;
    r_step(&r);
    /* 子菜单连移时序 */
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_SUBMENU);
    assert(r.m.sub_sel == 0U);
    r.in.ly = 200U;
    r_step(&r); /* engage：动一格（锚点即本帧） */
    assert(r.m.sub_sel == 1U);
    for (i = 0; i < 39; i++) /* t0+10 .. t0+390 */
    {
      r_step(&r);
      assert(r.m.sub_sel == 1U);
    }
    r_step(&r); /* t0+400：连移第一格 */
    assert(r.m.sub_sel == 2U);
    for (i = 0; i < 17; i++) /* t0+410 .. t0+570 */
    {
      r_step(&r);
      assert(r.m.sub_sel == 2U);
    }
    r_step(&r); /* t0+580：再一格 */
    assert(r.m.sub_sel == 3U);
    for (i = 0; i < 5; i++)
    {
      r_step(&r);
      assert(r.m.sub_sel == 3U); /* 已到末项 */
    }
    /* 释放滞后：回到 24 以内才复位，复位后可再次触发 */
    r.in.ly = 56U;
    r_step(&r);
    assert(r.m.sub_sel == 2U);
    r.in.ly = 100U; /* 偏移 -28：未回 24 以内，不复位也不再动 */
    r_step(&r);
    assert(r.m.sub_sel == 2U);
    r.in.ly = 128U;
    r_step(&r);
    r.in.ly = 56U;
    r_step(&r);
    assert(r.m.sub_sel == 1U);
    r.in.ly = 128U;
    r_step(&r);
  }

  /* 10. 横向不回绕 + 主菜单记住上次选中项。 */
  {
    rig_t r;
    rig_init(&r, 1000U);
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    assert(r.m.page == UI_PAGE_MENU);
    r.in.lx = 56U; /* 左移：已在 0，不动 */
    r_step(&r);
    r_step(&r);
    assert(r.m.menu_sel == 0U);
    r.in.lx = 176U;
    r_step(&r);
    assert(r.m.menu_sel == 1U);
    r.in.lx = 176U; /* 右移：已在 1，不动 */
    r_step(&r);
    r_step(&r);
    assert(r.m.menu_sel == 1U);
    r.in.lx = 128U;
    r_step(&r);
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    assert(r.m.page == UI_PAGE_BRAND);
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    assert(r.m.page == UI_PAGE_MENU);
    assert(r.m.menu_sel == 1U); /* 记住 */
  }

  /* 11. 动作中取消：CHECK_RUN + busy，摇杆越出死区或 R1 → cancel_motion 且离开。 */
  {
    rig_t r;
    rig_init(&r, 1000U);
    goto_check_run(&r);
    r.in.action_busy = 1U;
    r_step(&r);
    r.in.lx = 0U; /* 越出死区 */
    r_step(&r);
    assert(r.req.cancel_motion == 1U);
    assert(r.m.page == UI_PAGE_CHECK_CONFIRM); /* 离开动作页 */
    r.in.lx = 128U;
    r.in.action_busy = 0U;
    r_step(&r);
    /* 再进一次，测 R1 取消 */
    assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_CHECK_RUN);
    r.in.action_busy = 1U;
    r_step(&r);
    r_hold(&r, R1B, 3);
    assert(r.req.cancel_motion == 1U);
    assert(r.m.page != UI_PAGE_CHECK_RUN);
    r_free(&r, R1B, 3);
  }

  /* 12. 坏帧：frame_valid=0 冻结去抖计数，恢复后按有效帧凑满 3 帧才出事件。 */
  {
    rig_t r;
    int i;
    rig_init(&r, 1000U);
    r_press(&r, R1B);
    r_step(&r); /* 有效帧 1 */
    assert(r.m.page == UI_PAGE_BRAND);
    r.in.frame_valid = 0U;
    for (i = 0; i < 5; i++)
    {
      r_step(&r);
      assert(r.m.page == UI_PAGE_BRAND); /* 冻结：不产生事件 */
      req_all_zero(&r);
    }
    r.in.frame_valid = 1U;
    r_step(&r); /* 有效帧 2 */
    assert(r.m.page == UI_PAGE_BRAND);
    r_step(&r); /* 有效帧 3 → 事件 */
    assert(r.m.page == UI_PAGE_MENU);
    r_release(&r, R1B);
    r_step(&r);
  }

  /* 13. 回绕：init 于 UINT32_MAX-100，推进跨回绕，导航与连移不变。 */
  {
    rig_t r;
    int i;
    rig_init(&r, UINT32_MAX - 100U);
    r_hold(&r, R1B, 3);
    r_free(&r, R1B, 3);
    assert(r.m.page == UI_PAGE_MENU);
    r.in.lx = 176U;
    r_step(&r);
    assert(r.m.menu_sel == 1U);
    r.in.lx = 128U;
    r_step(&r);
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_SUBMENU);
    r.in.ly = 200U;
    r_step(&r); /* engage（时间已贴着回绕点） */
    assert(r.m.sub_sel == 1U);
    for (i = 0; i < 39; i++)
    {
      r_step(&r);
      assert(r.m.sub_sel == 1U); /* 跨回绕后仍按 400 ms 连移节拍 */
    }
    r_step(&r);
    assert(r.m.sub_sel == 2U);
  }

  /* 14. ui_drive_status_classify 九值。 */
  {
    assert(ui_drive_status_classify(0, 0) == UI_DRV_STOP);
    assert(ui_drive_status_classify(100, 0) == UI_DRV_FWD);
    assert(ui_drive_status_classify(-100, 0) == UI_DRV_REV);
    assert(ui_drive_status_classify(0, 100) == UI_DRV_LEFT);
    assert(ui_drive_status_classify(0, -100) == UI_DRV_RIGHT);
    assert(ui_drive_status_classify(100, 50) == UI_DRV_FWD_LEFT);
    assert(ui_drive_status_classify(100, -50) == UI_DRV_FWD_RIGHT);
    assert(ui_drive_status_classify(-100, 50) == UI_DRV_REV_LEFT);
    assert(ui_drive_status_classify(-100, -50) == UI_DRV_REV_RIGHT);
    assert(ui_drive_status_classify(1, 1) == UI_DRV_STOP); /* 死区内 */
  }

  /* 15. MANUAL_CAL：SELECT 长按 2 s 记录一次；摇杆漂移取消计时。 */
  {
    rig_t r;
    int i;
    rig_init(&r, 5000U);
    goto_submenu(&r, 3U);
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_MANUAL_CAL);
    r_hold(&r, SELB, 3); /* 稳定按下，press_ms 锚定 */
    for (i = 0; i < 199; i++) /* 再 1990 ms：未满 2 s */
    {
      r_step(&r);
      assert(r.req.record_ref == 0U);
    }
    r_step(&r); /* 满 2 s：记录一帧 */
    assert(r.req.record_ref == 1U);
    for (i = 0; i < 10; i++)
    {
      r_step(&r);
      assert(r.req.record_ref == 0U); /* 单发 */
    }
    r_free(&r, SELB, 3);
    r.in.ly = 200U; /* 漂移中长按：计时被反复取消 */
    r_hold(&r, SELB, 3);
    for (i = 0; i < 210; i++)
    {
      r_step(&r);
      assert(r.req.record_ref == 0U);
    }
    r.in.ly = 128U; /* 回中后重新计满 2 s */
    for (i = 0; i < 199; i++)
    {
      r_step(&r);
      assert(r.req.record_ref == 0U);
    }
    r_step(&r);
    assert(r.req.record_ref == 1U);
    r_free(&r, SELB, 3);
    r.in.ly = 128U;
    r_step(&r);
  }

  /* 16. R1 按住期间新出现的 L1/USER 边沿被抑制；R1 释放后重新按下恢复正常。
   *     （审查修复：R1+L1 同按只受理返回/取消，含"R1 先按住再来的确认边沿"）。 */
  {
    rig_t r;
    rig_init(&r, 1000U);
    r.in.selfcheck_ready = 1U;
    r_hold(&r, L1B, 3); /* BRAND → CHECK_CONFIRM */
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
    r_hold(&r, R1B, 3); /* R1 边沿：返回 BRAND，并保持按住 */
    assert(r.m.page == UI_PAGE_BRAND);
    r_hold(&r, L1B, 3); /* R1 按住中 L1 稳定：不得产生确认副作用 */
    assert(r.m.page == UI_PAGE_BRAND);
    assert(r.req.selfcheck_start == 0U);
    assert(r.req.home_start == 0U);
    assert(r.req.record_ref == 0U);
    r_free(&r, R1B, 3);
    r_free(&r, L1B, 3);
    r_hold(&r, L1B, 3); /* R1 已释放，L1 重新按下：确认正常 */
    assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
    r_free(&r, L1B, 3);
    /* USER 同规则 */
    r_hold(&r, R1B, 3); /* CHECK_CONFIRM R1 → BRAND，保持按住 */
    assert(r.m.page == UI_PAGE_BRAND);
    r.in.user_pressed = 1U;
    r_step(&r);
    r_step(&r);
    r_step(&r); /* R1 按住中 USER 稳定：不进确认页 */
    assert(r.m.page == UI_PAGE_BRAND);
    r.in.user_pressed = 0U;
    r_free(&r, R1B, 3);
    r_step(&r);
    r.in.user_pressed = 1U;
    r_step(&r);
    r_step(&r);
    r_step(&r); /* R1 释放后 USER 重新按下：正常进确认页 */
    assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
    r.in.user_pressed = 0U;
    r_step(&r);
  }

  /* 17. 确认页 5 s 倒计时自动启动（spec §4 2026-11-01 新流程）：
   *     a) ready=1 归零当拍自发一帧 selfcheck_start 并进 CHECK_RUN；
   *     b) confirm_remain_ms 单调递减；c) ready=0 冻结不减、恢复后继续
   *     走剩余（不重置 5000）；d) ready=0 时 L1/摇杆右不发请求不转页；
   *     e) 进页即按住 USER 不算取消，先见释放后再按才回 BRAND；
   *     f) USER 新边沿取消回 BRAND；L1 立即启动；lx=200 立即启动；lx=60 取消。 */
  {
    rig_t r;
    int i;
    int fired;
    uint16_t prev;

    /* a) 归零当拍自发启动，只发一帧 */
    fired = 0;
    rig_init(&r, 1000U);
    goto_confirm(&r, 1U);
    for (i = 0; i < 520; i++)
    {
      r_step(&r);
      if (r.req.selfcheck_start != 0U)
      {
        fired++;
        assert(fired == 1); /* 单发 */
      }
      if (fired != 0)
      {
        assert(r.m.page == UI_PAGE_CHECK_RUN);
      }
      else
      {
        assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
      }
    }
    assert(fired == 1);

    /* b) remain 单调递减（递减到启动前为止） */
    rig_init(&r, 1000U);
    goto_confirm(&r, 1U);
    prev = r.m.confirm_remain_ms;
    for (i = 0; i < 495; i++)
    {
      r_step(&r);
      assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
      assert(r.m.confirm_blocked == 0U);
      assert(r.m.confirm_remain_ms < prev);
      prev = r.m.confirm_remain_ms;
    }

    /* c) 冻结：ready=0 时 remain 不减、blocked=1、5100 ms 不启动；
     *    走一段→冻结→恢复：继续走完剩余才启动（不重置为 5000）。 */
    rig_init(&r, 1000U);
    goto_confirm(&r, 0U);
    for (i = 0; i < 510; i++)
    {
      r_step(&r);
      assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
      assert(r.req.selfcheck_start == 0U);
      assert(r.m.confirm_blocked == 1U);
      assert(r.m.confirm_remain_ms == (uint16_t)UI_CONFIRM_COUNTDOWN_MS);
    }
    r.in.selfcheck_ready = 1U;
    for (i = 0; i < 200; i++) /* 走 2000 ms：remain → 3000 */
    {
      r_step(&r);
      assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
    }
    assert(r.m.confirm_remain_ms == (uint16_t)(UI_CONFIRM_COUNTDOWN_MS - 2000U));
    r.in.selfcheck_ready = 0U;
    for (i = 0; i < 150; i++) /* 冻结 1500 ms：进度保留 */
    {
      r_step(&r);
      assert(r.m.confirm_blocked == 1U);
      assert(r.m.confirm_remain_ms == (uint16_t)(UI_CONFIRM_COUNTDOWN_MS - 2000U));
    }
    r.in.selfcheck_ready = 1U;
    for (i = 0; i < 299; i++) /* 恢复后 2990 ms：仍未启动 */
    {
      r_step(&r);
      assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
      assert(r.req.selfcheck_start == 0U);
    }
    r_step(&r); /* 剩余走完：启动 */
    assert(r.req.selfcheck_start == 1U);
    assert(r.m.page == UI_PAGE_CHECK_RUN);

    /* d) ready=0：L1 与摇杆右都不发请求、不转页（页面只显示原因） */
    rig_init(&r, 1000U);
    goto_confirm(&r, 0U);
    r_hold(&r, L1B, 3);
    r_free(&r, L1B, 3);
    assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
    assert(r.req.selfcheck_start == 0U);
    r.in.lx = 200U;
    r_step(&r);
    assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
    assert(r.req.selfcheck_start == 0U);
    r.in.lx = 128U;
    r_step(&r);

    /* e) 进页那帧 USER 仍按住 → 不算取消；见释放后新按才回 BRAND */
    rig_init(&r, 1000U);
    r.in.user_pressed = 1U;
    r_step(&r);
    r_step(&r);
    r_step(&r);
    assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
    for (i = 0; i < 10; i++)
    {
      r_step(&r);
      assert(r.m.page == UI_PAGE_CHECK_CONFIRM); /* 按住不取消 */
    }
    r.in.user_pressed = 0U;
    for (i = 0; i < 3; i++) { r_step(&r); }
    r.in.user_pressed = 1U; /* 新边沿 → 取消 */
    r_step(&r);
    r_step(&r);
    r_step(&r);
    assert(r.m.page == UI_PAGE_BRAND);
    r.in.user_pressed = 0U;
    r_step(&r);

    /* f) L1 进入后 USER 新边沿同样取消回 BRAND */
    rig_init(&r, 1000U);
    goto_confirm(&r, 0U);
    r.in.user_pressed = 1U;
    r_step(&r);
    r_step(&r);
    r_step(&r);
    assert(r.m.page == UI_PAGE_BRAND);
    r.in.user_pressed = 0U;
    r_step(&r);
  }

  /* 18. 确认页立即启动/取消：L1 边沿、lx=200 启动；lx=60 取消回 BRAND。 */
  {
    rig_t r;

    /* L1 立即启动（经 USER 进页，ready=1） */
    rig_init(&r, 1000U);
    r.in.selfcheck_ready = 1U;
    r.in.user_pressed = 1U;
    r_step(&r);
    r_step(&r);
    r_step(&r);
    assert(r.m.page == UI_PAGE_CHECK_CONFIRM);
    r.in.user_pressed = 0U;
    r_step(&r);
    r_step(&r);
    r_step(&r);
    r_hold(&r, L1B, 2);
    assert(r.req.selfcheck_start == 0U);
    r_step(&r);
    assert(r.req.selfcheck_start == 1U);
    assert(r.m.page == UI_PAGE_CHECK_RUN);
    r_free(&r, L1B, 3);

    /* lx=200 立即启动 */
    rig_init(&r, 1000U);
    goto_confirm(&r, 1U);
    r.in.lx = 200U;
    r_step(&r);
    assert(r.req.selfcheck_start == 1U);
    assert(r.m.page == UI_PAGE_CHECK_RUN);

    /* lx=60 取消回 BRAND */
    rig_init(&r, 1000U);
    goto_confirm(&r, 1U);
    r.in.lx = 60U;
    r_step(&r);
    assert(r.m.page == UI_PAGE_BRAND);
    assert(r.req.selfcheck_start == 0U);
    r.in.lx = 128U;
    r_step(&r);
  }

  puts("ui_model pages / debounce / nav / window / classify OK");
  return 0;
}
