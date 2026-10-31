#include "manual_ref.h"

static void mr_init_fields(manual_ref_t *s)
{
  s->window_start_ms = 0U;
  s->press_ms = 0U;
  s->user_ms = 0U;
  s->window_open = 0U;
  s->window_armed = 0U;   /* 必须先见过一次释放，才接受新按下 */
  s->window_seeded = 0U;
  s->user_filtered = 0U;
  s->user_candidate = 0U;
  s->user_frames = 0U;
  s->select_filtered = 0U;
  s->select_candidate = 0U;
  s->select_frames = 0U;
  s->record_requested = 0U;
  s->record_consumed = 0U;
  s->ref_shown = 0U;
  s->ready_ok = 0U;
  s->swallow_release = 0U;
  s->state = MANUAL_REF_STATE_WAITING;
}

void manual_ref_open(manual_ref_t *s, uint32_t now_ms)
{
  if (s == 0)
  {
    return;
  }
  mr_init_fields(s);
  s->window_start_ms = now_ms;
  s->window_open = 1U;
}

void manual_ref_open_manual(manual_ref_t *s, uint32_t now_ms)
{
  if (s == 0)
  {
    return;
  }
  manual_ref_open(s, now_ms);
  manual_ref_enter_manual(s, now_ms);
}

void manual_ref_enter_manual(manual_ref_t *s, uint32_t now_ms)
{
  if (s == 0 || s->state != MANUAL_REF_STATE_WAITING)
  {
    return;
  }
  s->window_open = 0U;
  s->state = MANUAL_REF_STATE_MANUAL;
  s->press_ms = 0U;
  s->record_requested = 0U;
  s->record_consumed = 0U;
  s->ref_shown = 0U;
  /* 进门那次按下的释放沿不算一次完整按压。 */
  s->swallow_release = 1U;
  s->user_ms = now_ms;
  /* 进页时 SELECT 视为“已按住”：必须先释放再按满 2 s，防误记参考。 */
  s->select_filtered = 1U;
  s->select_candidate = 1U;
  s->select_frames = MANUAL_REF_PRESS_STABLE_FRAMES;
}

void manual_ref_set_ready(manual_ref_t *s, uint8_t still, uint8_t frame_valid,
                          uint8_t sticks_centered)
{
  if (s == 0)
  {
    return;
  }
  s->ready_ok = (uint8_t)(still != 0U && frame_valid != 0U &&
                          sticks_centered != 0U);
}

/* 连续 N 个有效帧保持同一电平才改确认电平；只在确认电平真正翻转时产生事件。 */
static uint8_t mr_filter(uint8_t *filtered, uint8_t *candidate, uint8_t *frames,
                         uint8_t raw)
{
  if (*frames == 0U || raw != *candidate)
  {
    *candidate = raw;
    *frames = 1U;
  }
  else if (*frames < 255U)
  {
    (*frames)++;
  }
  else
  {
    /* 饱和 */
  }

  if (*frames >= MANUAL_REF_PRESS_STABLE_FRAMES && raw != *filtered)
  {
    *filtered = raw;
    return 1U;
  }
  return 0U;
}

static void mr_close(manual_ref_t *s)
{
  s->window_open = 0U;
  s->state = MANUAL_REF_STATE_MAIN;
}

static void mr_window_step(manual_ref_t *s, uint32_t now_ms, uint8_t user_event,
                           uint8_t user_raw)
{
  if (s->window_seeded == 0U)
  {
    s->window_seeded = 1U;
    /* 进窗口时 USER 还按着（自检结束那一下）：不许算入口，必须先释放。
     * 反过来，USER 本来就已经是释放态，之后不会再有"释放沿"事件送来——
     * 不能干等一个永远不来的边沿，否则第一次按下必须多按一次才生效。
     * 判据用当前这一帧的原始电平，而不是 user_filtered：后者在开头几帧
     * 还没完成去抖确认，此时它恒为 0，会把"按住"误判成"已释放"。 */
    s->window_armed = (uint8_t)((user_raw != 0U) ? 0U : 1U);
  }

  if (user_event == 0U)
  {
    return;
  }

  if (s->user_filtered == 0U)
  {
    /* 释放沿：之后再有按下就进页 */
    s->window_armed = 1U;
  }
  else if (s->window_armed != 0U)
  {
    manual_ref_enter_manual(s, now_ms);
  }
  else
  {
    /* 按住的是“进窗口那一下”，不算入口。 */
  }
}

static void mr_manual_step(manual_ref_t *s, uint32_t now_ms, uint8_t user_event,
                           uint8_t select_event, uint8_t sel)
{
  if (user_event != 0U)
  {
    if (s->user_filtered != 0U)
    {
      s->user_ms = now_ms;
      s->swallow_release = 0U;
    }
    else if (s->swallow_release != 0U)
    {
      s->swallow_release = 0U;
      s->user_ms = 0U;
    }
    else if (s->user_ms != 0U &&
             (uint32_t)(now_ms - s->user_ms) >= MANUAL_REF_PRESS_STABLE_FRAMES * 10U)
    {
      s->user_ms = 0U;
      mr_close(s);
    }
    else
    {
      s->user_ms = 0U;
    }
    return;
  }

  if (select_event == 0U)
  {
    return;
  }

  if (sel != 0U)
  {
    s->press_ms = now_ms;
    s->record_consumed = 0U;
  }
  else
  {
    /* 抬起：长按判定在按住期间进行，这里只需清掉计时 */
    s->press_ms = 0U;
  }
}

void manual_ref_step(manual_ref_t *s, uint32_t now_ms, uint8_t user_raw,
                     uint8_t select_raw, uint8_t frame_valid)
{
  uint8_t user_event;
  uint8_t select_event;

  if (s == 0 || s->state == MANUAL_REF_STATE_MAIN)
  {
    return;
  }

  /* 窗口按真实毫秒关闭，与按键无关。 */
  if (s->window_open != 0U &&
      (uint32_t)(now_ms - s->window_start_ms) >= MANUAL_REF_WINDOW_MS)
  {
    mr_close(s);
    return;
  }

  if (frame_valid == 0U)
  {
    return; /* 坏帧：不推进去抖，也不触发事件 */
  }

  user_event = mr_filter(&s->user_filtered, &s->user_candidate, &s->user_frames,
                         (uint8_t)(user_raw != 0U));
  select_event = mr_filter(&s->select_filtered, &s->select_candidate,
                           &s->select_frames, (uint8_t)(select_raw != 0U));

  if (s->state == MANUAL_REF_STATE_WAITING)
  {
    mr_window_step(s, now_ms, user_event, (uint8_t)(user_raw != 0U));
    return;
  }

  mr_manual_step(s, now_ms, user_event, select_event,
                 (uint8_t)(select_raw != 0U));

  /* 长按在按住期间判定，不依赖释放沿。 */
  if (s->state == MANUAL_REF_STATE_MANUAL && s->ready_ok != 0U &&
      s->record_consumed == 0U && s->press_ms != 0U &&
      (uint32_t)(now_ms - s->press_ms) >= MANUAL_REF_RECORD_MS)
  {
    s->record_consumed = 1U;
    s->record_requested = 1U;
  }
}

manual_ref_state_t manual_ref_state(const manual_ref_t *s)
{
  return (s == 0) ? MANUAL_REF_STATE_MAIN : s->state;
}

uint8_t manual_ref_record_requested(const manual_ref_t *s)
{
  return (s == 0) ? 0U : s->record_requested;
}

void manual_ref_acknowledge(manual_ref_t *s, uint8_t success)
{
  if (s == 0)
  {
    return;
  }
  s->record_requested = 0U;
  if (success != 0U)
  {
    s->ref_shown = 1U;
  }
}

uint8_t manual_ref_ref_shown(const manual_ref_t *s)
{
  return (s == 0) ? 0U : s->ref_shown;
}

uint8_t manual_ref_home_requested(const manual_ref_t *s)
{
  (void)s;
  return 0U;
}

uint32_t manual_ref_press_ms(const manual_ref_t *s)
{
  return (s == 0) ? 0U : s->press_ms;
}
