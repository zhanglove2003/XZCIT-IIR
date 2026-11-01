#include <stddef.h>

#include "ui_model.h"

/* 纯页面状态机实现（spec §4/§7）。不访问 OLED/HAL/RTOS。
 * 优先级：R1（返回/取消）> 确认键（L1/USER）；SELECT 与确认键同按不启动运动。
 * 确认键一次稳定按下只产生一个边沿事件，天然单发；进页时按住的键不会再出边沿。 */

#define UI_BTNIDX_R1   0U
#define UI_BTNIDX_L1   1U
#define UI_BTNIDX_SEL  2U
#define UI_BTNIDX_USER 3U
#define UI_BTN_COUNT   4U

void ui_model_init(ui_model_t *m, uint32_t now_ms)
{
  uint8_t i;
  m->page = UI_PAGE_BRAND;
  m->back_page = UI_PAGE_BRAND;
  m->menu_sel = 0U;
  m->sub_sel = 0U;
  m->sub_scroll = 0U;
  m->confirm_focus = 0U;
  m->result_view = 0U;
  m->sel_since_ms = now_ms;
  m->confirm_since_ms = now_ms;
  m->confirm_remain_ms = (uint16_t)UI_CONFIRM_COUNTDOWN_MS;
  m->confirm_blocked = 0U;
  for (i = 0U; i < UI_BTN_COUNT; i++)
  {
    m->btn_filtered[i] = 0U;
    m->btn_candidate[i] = 0U;
    m->btn_frames[i] = 0U;
    m->btn_press_ms[i] = now_ms;
    m->btn_long_fired[i] = 0U;
  }
  m->axis_h.dir = 0;
  m->axis_h.step_ms = now_ms;
  m->axis_h.fast = 0U;
  m->axis_v.dir = 0;
  m->axis_v.step_ms = now_ms;
  m->axis_v.fast = 0U;
  m->run_seen_busy = 0U;
  m->window_armed = 0U;
  m->output_cap_pct = UI_CAP_MAX_PCT; /* 上电默认 50% = 8400（RAM，不落盘） */
}

static int32_t ui_abs32(int32_t x)
{
  return (x < 0) ? -x : x;
}

static uint8_t btn_raw_pressed(const ui_model_in_t *in, uint8_t idx)
{
  switch (idx)
  {
  case UI_BTNIDX_R1:
    return ((in->buttons & (uint16_t)(1U << UI_BTN_BIT_R1)) == 0U) ? 1U : 0U;
  case UI_BTNIDX_L1:
    return ((in->buttons & (uint16_t)(1U << UI_BTN_BIT_L1)) == 0U) ? 1U : 0U;
  case UI_BTNIDX_SEL:
    return ((in->buttons & (uint16_t)(1U << UI_BTN_BIT_SELECT)) == 0U) ? 1U : 0U;
  default:
    return (in->user_pressed != 0U) ? 1U : 0U;
  }
}

/* 连续有效帧计数（同 wheel_home 口径）；只在本函数被调用时推进，
 * 坏帧由调用方整体跳过，计数冻结不清零。 */
static void btn_update(ui_model_t *m, uint8_t idx, uint8_t raw, uint32_t now,
                       uint8_t *press_edge)
{
  *press_edge = 0U;
  if (raw != m->btn_candidate[idx])
  {
    m->btn_candidate[idx] = raw;
    m->btn_frames[idx] = 1U;
  }
  else if (m->btn_frames[idx] < 255U)
  {
    m->btn_frames[idx]++;
  }

  if (m->btn_frames[idx] >= UI_BTN_STABLE_FRAMES && m->btn_filtered[idx] != raw)
  {
    m->btn_filtered[idx] = raw;
    if (raw != 0U)
    {
      m->btn_press_ms[idx] = now;
      m->btn_long_fired[idx] = 0U;
      *press_edge = 1U;
    }
  }
}

/* 菜单摇杆轴：迟滞 engage(48)/release(24)，engage 先走一格，
 * 400 ms 后开始连移、每 180 ms 一格。 */
static void axis_update(ui_stick_axis_t *a, uint32_t now, int32_t off, int8_t *ev)
{
  *ev = 0;
  if (a->dir == 0)
  {
    if (off >= UI_STICK_ENGAGE)
    {
      a->dir = 1;
      *ev = 1;
      a->step_ms = now;
      a->fast = 0U;
    }
    else if (off <= -UI_STICK_ENGAGE)
    {
      a->dir = -1;
      *ev = -1;
      a->step_ms = now;
      a->fast = 0U;
    }
  }
  else if (off <= UI_STICK_RELEASE && off >= -UI_STICK_RELEASE)
  {
    a->dir = 0; /* 回到 24 以内才复位 */
  }
  else if ((off > 0 && a->dir < 0) || (off < 0 && a->dir > 0))
  {
    a->dir = (off > 0) ? 1 : -1; /* 一次跨中：改向并重新起拍 */
    *ev = a->dir;
    a->step_ms = now;
    a->fast = 0U;
  }
  else
  {
    uint32_t interval = (a->fast != 0U) ? UI_NAV_REPEAT_MS : UI_NAV_FIRST_REPEAT_MS;
    if ((uint32_t)(now - a->step_ms) >= interval)
    {
      *ev = a->dir;
      a->step_ms = now;
      a->fast = 1U;
    }
  }
}

/* 任一摇杆轴越出死区（归位/自检取消、长按计时取消的判据）。 */
static uint8_t stick_drift(const ui_model_in_t *in)
{
  int32_t dx = (int32_t)in->lx - UI_STICK_CENTER;
  int32_t dy = (int32_t)in->ly - UI_STICK_CENTER;
  int32_t drx = (int32_t)in->rx - UI_STICK_CENTER;
  int32_t dry = (int32_t)in->ry - UI_STICK_CENTER;
  return (ui_abs32(dx) > UI_STICK_DRIFT_LIMIT ||
          ui_abs32(dy) > UI_STICK_DRIFT_LIMIT ||
          ui_abs32(drx) > UI_STICK_DRIFT_LIMIT ||
          ui_abs32(dry) > UI_STICK_DRIFT_LIMIT)
             ? 1U
             : 0U;
}

static void go_page(ui_model_t *m, ui_page_t p, uint32_t now)
{
  m->page = p;
  m->sel_since_ms = now;
  if (p == UI_PAGE_CHECK_CONFIRM)
  {
    /* 进入确认页即锚定 5 s 倒计时（含 R1/越区从动作页退回的重进）。 */
    m->confirm_since_ms = now;
    m->confirm_remain_ms = (uint16_t)UI_CONFIRM_COUNTDOWN_MS;
    m->confirm_blocked = 0U;
  }
}

/* 自检启动转移：发请求当拍进 CHECK_RUN；取消/越区回到本页重走倒计时。 */
static void start_selfcheck(ui_model_t *m, ui_model_req_t *req, uint32_t now)
{
  if (req != NULL)
  {
    req->selfcheck_start = 1U;
  }
  m->back_page = UI_PAGE_CHECK_CONFIRM;
  m->run_seen_busy = 0U;
  go_page(m, UI_PAGE_CHECK_RUN, now);
}

/* 确认页倒计时冻结（ready==0 与坏帧共用口径）：remain 不减、blocked=1，
 * 锚点按剩余时间回推——恢复后继续走剩余，绝不跳过剩余直接启动（spec §4）。 */
static void confirm_freeze(ui_model_t *m, uint32_t now)
{
  m->confirm_blocked = 1U;
  m->confirm_since_ms =
      now - (uint32_t)(UI_CONFIRM_COUNTDOWN_MS - m->confirm_remain_ms);
}

void ui_model_step(ui_model_t *m, const ui_model_in_t *in, ui_model_req_t *req)
{
  uint32_t now = in->now_ms;
  uint8_t press[UI_BTN_COUNT];
  uint8_t r1_edge, l1_edge, user_edge, sel_edge;
  uint8_t sel_held, l1_held, user_held, r1_held;
  int32_t offx, offy;
  int8_t evx, evy;
  uint8_t i;

  if (req != NULL)
  {
    req->selfcheck_start = 0U;
    req->home_start = 0U;
    req->record_ref = 0U;
    req->cancel_motion = 0U;
    req->output_cap_changed = 0U;
  }

  /* 坏帧：冻结全部去抖/摇杆/长按推进，不清零（spec §11 坏帧当轮不沿用旧指令，
   * 页面状态保持，等待恢复帧重新计数）。确认页倒计时同样冻结并显示原因：
   * 锚点必须在本早退内回推，否则恢复帧 elapsed 已超 5000 会跳过剩余直接
   * 自动启动（手柄重连瞬间意外转轮）。 */
  if (in->frame_valid == 0U)
  {
    if (m->page == UI_PAGE_MANUAL_CAL && m->btn_filtered[UI_BTNIDX_SEL] != 0U)
    {
      /* spec §8：坏帧取消本次长按计时 */
      m->btn_press_ms[UI_BTNIDX_SEL] = now;
      m->btn_long_fired[UI_BTNIDX_SEL] = 0U;
    }
    if (m->page == UI_PAGE_CHECK_CONFIRM)
    {
      confirm_freeze(m, now);
    }
    return;
  }

  for (i = 0U; i < UI_BTN_COUNT; i++)
  {
    btn_update(m, i, btn_raw_pressed(in, i), now, &press[i]);
  }
  r1_edge = press[UI_BTNIDX_R1];
  l1_edge = press[UI_BTNIDX_L1];
  sel_edge = press[UI_BTNIDX_SEL];
  user_edge = press[UI_BTNIDX_USER];

  sel_held = m->btn_filtered[UI_BTNIDX_SEL];
  l1_held = m->btn_filtered[UI_BTNIDX_L1];
  user_held = m->btn_filtered[UI_BTNIDX_USER];
  r1_held = m->btn_filtered[UI_BTNIDX_R1];

  offx = (int32_t)in->lx - UI_STICK_CENTER;
  offy = (int32_t)in->ly - UI_STICK_CENTER;
  axis_update(&m->axis_h, now, offx, &evx);
  axis_update(&m->axis_v, now, offy, &evy);

  /* 手动标定页：SELECT 长按 2 s 记录参考；漂移取消本次计时。 */
  if (m->page == UI_PAGE_MANUAL_CAL && sel_held != 0U)
  {
    if (stick_drift(in) != 0U)
    {
      m->btn_press_ms[UI_BTNIDX_SEL] = now;
      m->btn_long_fired[UI_BTNIDX_SEL] = 0U;
    }
    else if (m->btn_long_fired[UI_BTNIDX_SEL] == 0U &&
             (uint32_t)(now - m->btn_press_ms[UI_BTNIDX_SEL]) >= UI_SELECT_LONG_PRESS_MS)
    {
      m->btn_long_fired[UI_BTNIDX_SEL] = 1U;
      if (req != NULL)
      {
        req->record_ref = 1U;
      }
    }
  }

  /* 归位/自检期间摇杆越出驾驶死区：先取消动作并离开（spec §7）。 */
  if ((m->page == UI_PAGE_CHECK_RUN || m->page == UI_PAGE_HOME) &&
      in->action_busy != 0U && stick_drift(in) != 0U)
  {
    if (req != NULL)
    {
      req->cancel_motion = 1U;
    }
    go_page(m, m->back_page, now);
    m->run_seen_busy = 0U;
    m->window_armed = 0U;
    return;
  }

  /* 结果页 5 s 窗口超时：Ctrl 把 result_window_open 清 0 后回品牌页。 */
  if (m->page == UI_PAGE_CHECK_RESULT && m->window_armed != 0U &&
      in->result_window_open == 0U)
  {
    m->window_armed = 0U;
    go_page(m, UI_PAGE_BRAND, now);
  }

  /* 自检运行完成：必须先见过 busy，防止请求当拍误判为已完成。 */
  if (m->page == UI_PAGE_CHECK_RUN)
  {
    if (in->action_busy != 0U)
    {
      m->run_seen_busy = 1U;
    }
    else if (m->run_seen_busy != 0U)
    {
      m->page = UI_PAGE_CHECK_RESULT;
      m->back_page = UI_PAGE_BRAND; /* 自检结束的结果页 R1 回品牌页 */
      m->window_armed = (in->result_window_open != 0U) ? 1U : 0U;
      m->result_view = 0U;
      m->run_seen_busy = 0U;
      m->sel_since_ms = now;
    }
  }

  /* R1 优先：返回/取消；动作中先取消再转移（spec §7）。 */
  if (r1_edge != 0U)
  {
    switch (m->page)
    {
    case UI_PAGE_BRAND:
      go_page(m, UI_PAGE_MENU, now);
      break;
    case UI_PAGE_MENU:
      go_page(m, UI_PAGE_BRAND, now);
      break;
    case UI_PAGE_DRIVE:
      go_page(m, UI_PAGE_MENU, now); /* 撤销驾驶输出由 Ctrl 仲裁 */
      break;
    case UI_PAGE_SUBMENU:
      go_page(m, UI_PAGE_MENU, now); /* sub_sel 保留 */
      break;
    case UI_PAGE_CHECK_CONFIRM:
      go_page(m, m->back_page, now); /* 取消 */
      break;
    case UI_PAGE_CHECK_RUN:
    case UI_PAGE_HOME:
      if (in->action_busy != 0U && req != NULL)
      {
        req->cancel_motion = 1U;
      }
      go_page(m, m->back_page, now);
      m->run_seen_busy = 0U;
      break;
    case UI_PAGE_CHECK_RESULT:
      m->window_armed = 0U;
      go_page(m, m->back_page, now);
      break;
    case UI_PAGE_MANUAL_CAL:
      go_page(m, m->back_page, now);
      break;
    case UI_PAGE_OUTPUT_CAP:
      go_page(m, m->back_page, now); /* 回子菜单原条目（sub_sel 保留） */
      break;
    default:
      break;
    }
    return;
  }

  /* L1 确认；R1 按住期间不受理（R1+L1 同按只受理返回/取消）；
   * SELECT 按住期间不受理（互斥，spec §7）。 */
  if (l1_edge != 0U && sel_held == 0U && r1_held == 0U)
  {
    switch (m->page)
    {
    case UI_PAGE_BRAND:
      m->back_page = UI_PAGE_BRAND;
      m->confirm_focus = 0U;
      go_page(m, UI_PAGE_CHECK_CONFIRM, now);
      break;
    case UI_PAGE_MENU:
      m->back_page = UI_PAGE_MENU;
      if (m->menu_sel == 0U)
      {
        go_page(m, UI_PAGE_DRIVE, now);
      }
      else
      {
        go_page(m, UI_PAGE_SUBMENU, now);
      }
      break;
    case UI_PAGE_SUBMENU:
      switch (m->sub_sel)
      {
      case 0U: /* 自检结果：从子菜单进，R1 回原条目 */
        m->back_page = UI_PAGE_SUBMENU;
        m->window_armed = 0U;
        m->result_view = 0U;
        go_page(m, UI_PAGE_CHECK_RESULT, now);
        break;
      case 1U: /* 重新自检 */
        m->back_page = UI_PAGE_SUBMENU;
        m->confirm_focus = 0U;
        go_page(m, UI_PAGE_CHECK_CONFIRM, now);
        break;
      case 2U: /* 回到初始位：L1 只提交请求（spec §8） */
        if (req != NULL)
        {
          req->home_start = 1U;
        }
        m->back_page = UI_PAGE_SUBMENU;
        m->run_seen_busy = 0U;
        go_page(m, UI_PAGE_HOME, now);
        break;
      case UI_SUB_OUTPUT_CAP: /* 输出上限：进设置页（改动即时生效，无确认动作） */
        m->back_page = UI_PAGE_SUBMENU;
        if (req != NULL)
        {
          /* 进页当拍把当前百分比请求为新值：页面显示的 百分比/原始值
           * 与生效上限从第一拍起一致（默认档 50% = 8400 = 现状满输出，
           * 进页当拍为幂等写）。 */
          req->output_cap_changed = 1U;
        }
        go_page(m, UI_PAGE_OUTPUT_CAP, now);
        break;
      default: /* 手动标定 */
        m->back_page = UI_PAGE_SUBMENU;
        m->btn_long_fired[UI_BTNIDX_SEL] = 0U;
        go_page(m, UI_PAGE_MANUAL_CAL, now);
        break;
      }
      break;
    case UI_PAGE_CHECK_CONFIRM:
      /* 新流程：L1 边沿=立即开始，不再看焦点。前置不满足（ready=0）时
       * 不发请求、不转页——页面冻结倒计时并显示原因，避免落到无人运行
       * 的 CHECK_RUN 页（Ctrl 对请求的拒绝不回喂模型，见 spec §4）。 */
      if (in->selfcheck_ready != 0U)
      {
        start_selfcheck(m, req, now);
      }
      break;
    case UI_PAGE_CHECK_RESULT:
      if (in->result_window_open != 0U)
      {
        m->back_page = UI_PAGE_SUBMENU;
        m->window_armed = 0U;
        m->btn_long_fired[UI_BTNIDX_SEL] = 0U;
        go_page(m, UI_PAGE_MANUAL_CAL, now);
      }
      break;
    default: /* DRIVE/CHECK_RUN/HOME/MANUAL_CAL 无 L1 动作 */
      break;
    }
    return;
  }

  /* USER：品牌页进自检确认；确认页再按取消（进页那帧按住不出边沿，须先
   * 见释放——去抖口径天然满足）；结果页窗口内进标定；手动标定页备用退出。
   * R1 按住期间不受理（同 L1 的优先级规则）。 */
  if (user_edge != 0U && sel_held == 0U && r1_held == 0U)
  {
    switch (m->page)
    {
    case UI_PAGE_BRAND:
      m->back_page = UI_PAGE_BRAND;
      m->confirm_focus = 0U;
      go_page(m, UI_PAGE_CHECK_CONFIRM, now);
      break;
    case UI_PAGE_CHECK_CONFIRM:
      go_page(m, UI_PAGE_BRAND, now); /* 取消倒计时回品牌主页（spec §4） */
      break;
    case UI_PAGE_CHECK_RESULT:
      if (in->result_window_open != 0U)
      {
        m->back_page = UI_PAGE_SUBMENU;
        m->window_armed = 0U;
        m->btn_long_fired[UI_BTNIDX_SEL] = 0U;
        go_page(m, UI_PAGE_MANUAL_CAL, now);
      }
      break;
    case UI_PAGE_MANUAL_CAL:
      go_page(m, m->back_page, now); /* 退出不记录 */
      break;
    default:
      break;
    }
    return;
  }

  /* SELECT：驾驶页短按归位快捷入口；其他普通页面不触发运动。
   * 与确认键同按（L1/USER 按住）时不启动运动。 */
  if (sel_edge != 0U)
  {
    if (m->page == UI_PAGE_DRIVE && l1_held == 0U && user_held == 0U)
    {
      if (req != NULL)
      {
        req->home_start = 1U;
      }
      m->back_page = UI_PAGE_DRIVE;
      m->run_seen_busy = 0U;
      go_page(m, UI_PAGE_HOME, now);
    }
    return;
  }

  /* 摇杆导航：横向卡片与纵向列表各用一轴（spec §7）。 */
  switch (m->page)
  {
  case UI_PAGE_MENU:
    if (evx > 0 && m->menu_sel < (UI_MENU_COUNT - 1U))
    {
      m->menu_sel++;
      m->sel_since_ms = now;
    }
    else if (evx < 0 && m->menu_sel > 0U)
    {
      m->menu_sel--;
      m->sel_since_ms = now;
    }
    break;
  case UI_PAGE_SUBMENU:
    if (evy > 0 && m->sub_sel < (UI_SUB_COUNT - 1U))
    {
      m->sub_sel++;
      m->sel_since_ms = now;
    }
    else if (evy < 0 && m->sub_sel > 0U)
    {
      m->sub_sel--;
      m->sel_since_ms = now;
    }
    if (m->sub_sel < m->sub_scroll)
    {
      m->sub_scroll = m->sub_sel;
    }
    if (m->sub_sel > (uint8_t)(m->sub_scroll + 2U))
    {
      m->sub_scroll = (uint8_t)(m->sub_sel - 2U);
    }
    break;
  case UI_PAGE_CHECK_CONFIRM:
    /* 焦点字段保留（ly 仅更新显示用旧字段），不再参与启动/取消裁决：
     * 摇杆右（lx≥176）立即开始、摇杆左（lx≤80）取消回品牌页（spec §4）。 */
    if (evy > 0)
    {
      m->confirm_focus = 1U;
      m->sel_since_ms = now;
    }
    else if (evy < 0)
    {
      m->confirm_focus = 0U;
      m->sel_since_ms = now;
    }
    if (in->lx >= UI_CONFIRM_START_LX)
    {
      if (in->selfcheck_ready != 0U)
      {
        start_selfcheck(m, req, now);
      }
    }
    else if (in->lx <= UI_CONFIRM_CANCEL_LX)
    {
      go_page(m, UI_PAGE_BRAND, now);
    }
    break;
  case UI_PAGE_CHECK_RESULT:
    if (evx != 0)
    {
      m->result_view ^= 1U; /* 电机明细 / 电池 两屏切换 */
      m->sel_since_ms = now;
    }
    break;
  case UI_PAGE_OUTPUT_CAP:
    /* 摇杆上下 ±5% 步进（上=增/下=减），边界夹取不回绕；
     * 沿用导航轴的 engage/release/连移节奏。实际改变百分比的当拍发
     * output_cap_changed 新值请求（边界夹住不动则不发），Ctrl 落到
     * drive_ctrl 运行时上限，下一拍 mix 按新上限重算。 */
    if (evy != 0)
    {
      int32_t pct = (int32_t)m->output_cap_pct - (int32_t)evy * (int32_t)UI_CAP_STEP_PCT;

      if (pct < (int32_t)UI_CAP_MIN_PCT)
      {
        pct = (int32_t)UI_CAP_MIN_PCT;
      }
      if (pct > (int32_t)UI_CAP_MAX_PCT)
      {
        pct = (int32_t)UI_CAP_MAX_PCT;
      }
      if ((uint8_t)pct != m->output_cap_pct)
      {
        m->output_cap_pct = (uint8_t)pct;
        m->sel_since_ms = now;
        if (req != NULL)
        {
          req->output_cap_changed = 1U;
        }
      }
    }
    break;
  default:
    break;
  }

  /* 确认页 5 s 倒计时（spec §4）：上面各转移若已离开本页则跳过。
   * ready=0 冻结——remain 不减，锚点按剩余时间回推，恢复后继续走剩余；
   * 归零当拍自发 selfcheck_start（req 单发语义，本拍即被 Ctrl 消费）。 */
  if (m->page == UI_PAGE_CHECK_CONFIRM)
  {
    if (in->selfcheck_ready != 0U)
    {
      uint32_t elapsed = (uint32_t)(now - m->confirm_since_ms);

      m->confirm_blocked = 0U;
      if (elapsed >= UI_CONFIRM_COUNTDOWN_MS)
      {
        m->confirm_remain_ms = 0U;
        start_selfcheck(m, req, now);
      }
      else
      {
        m->confirm_remain_ms =
            (uint16_t)(UI_CONFIRM_COUNTDOWN_MS - elapsed);
      }
    }
    else
    {
      confirm_freeze(m, now);
    }
  }
}

ui_drive_status_t ui_drive_status_classify(int32_t v, int32_t t)
{
  uint8_t fwd = (v > UI_DRV_STATUS_DEADBAND) ? 1U : 0U;
  uint8_t rev = (v < -UI_DRV_STATUS_DEADBAND) ? 1U : 0U;
  uint8_t left = (t > UI_DRV_STATUS_DEADBAND) ? 1U : 0U;
  uint8_t right = (t < -UI_DRV_STATUS_DEADBAND) ? 1U : 0U;

  if (fwd != 0U)
  {
    return (left != 0U)   ? UI_DRV_FWD_LEFT
           : (right != 0U) ? UI_DRV_FWD_RIGHT
                           : UI_DRV_FWD;
  }
  if (rev != 0U)
  {
    return (left != 0U)   ? UI_DRV_REV_LEFT
           : (right != 0U) ? UI_DRV_REV_RIGHT
                           : UI_DRV_REV;
  }
  if (left != 0U)
  {
    return UI_DRV_LEFT;
  }
  if (right != 0U)
  {
    return UI_DRV_RIGHT;
  }
  return UI_DRV_STOP;
}
