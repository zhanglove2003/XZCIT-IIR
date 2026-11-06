/* Task 6：自检结束后 USER 键窗口、手动对齐页的进入/退出条件。
 * 全部用确定性 tick，不起 OS：窗口计时只看真实毫秒，按键去抖只看连续有效帧。 */
#include <assert.h>
#include <stdio.h>

#include "../Core/Src/manual_ref.c"

/* 模拟人手：按住期间每 10 ms 一帧持续喂，而不是只在边沿喂一次。 */
static uint32_t hold(manual_ref_t *s, uint32_t now, uint8_t raw, uint32_t ms)
{
  uint32_t t;
  for (t = 0U; t < ms; t += 10U)
  {
    manual_ref_step(s, now + t, raw, 0U, 1U);
  }
  return now + ms;
}

static void feed(manual_ref_t *s, uint32_t now, uint8_t raw)
{
  manual_ref_step(s, now, raw, 0U, 1U);
}

static void feed_sel(manual_ref_t *s, uint32_t now, uint8_t sel)
{
  manual_ref_step(s, now, 0U, sel, 1U);
}

static uint32_t settle_sel(manual_ref_t *s, uint32_t now, uint8_t sel)
{
  uint8_t i;
  for (i = 0U; i < MANUAL_REF_PRESS_STABLE_FRAMES; i++)
  {
    feed_sel(s, now, sel);
    now += 10U;
  }
  return now;
}

/* 让按键在 now 时刻稳定到 raw，并返回之后的时间 */
static uint32_t settle(manual_ref_t *s, uint32_t now, uint8_t raw)
{
  uint8_t i;
  for (i = 0U; i < MANUAL_REF_PRESS_STABLE_FRAMES; i++)
  {
    feed(s, now, raw);
    now += 10U;
  }
  /* 真人按住不会立刻松手：再保持 100 ms，事件才有机会被消费 */
  return hold(s, now, raw, 100U);
}

int main(void)
{
  manual_ref_t s;

  /* 1. 打开窗口：无按压 5000 ms 后自动进主界面 */
  manual_ref_open(&s, 1000U);
  assert(manual_ref_state(&s) == MANUAL_REF_STATE_WAITING);
  manual_ref_step(&s, 5999U, 0U, 0U, 1U);
  assert(manual_ref_state(&s) == MANUAL_REF_STATE_WAITING);
  manual_ref_step(&s, 6000U, 0U, 0U, 1U);
  assert(manual_ref_state(&s) == MANUAL_REF_STATE_MAIN);

  /* 2. 初次启动按钮一直按住：窗口内不算“释放后按下”，到点直接进主界面 */
  manual_ref_open(&s, 0U);
  {
    uint32_t t;
    for (t = 0U; t <= MANUAL_REF_WINDOW_MS; t += 10U)
    {
      feed(&s, t, 1U);
    }
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MAIN);
  }

  /* 3. USER 释放后重新按下并稳定 30 ms，事件早于 5000 ms → 进手动页 */
  manual_ref_open(&s, 0U);
  {
    uint32_t t = settle(&s, 0U, 1U);  /* 先按住(初次启动) */
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_WAITING);
    t = settle(&s, t, 0U);            /* 释放 */
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_WAITING);
    t = settle(&s, t, 1U);            /* 重新按下 */
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
  }

  /* 4. 按下发生在 5000 ms 或之后：不进入 */
  manual_ref_open(&s, 0U);
  {
    uint32_t t = 0U;
    t = settle(&s, t, 1U);
    t = settle(&s, t, 0U);
    /* 拖到窗口边界之后才按下 */
    while (t < MANUAL_REF_WINDOW_MS + 10U) { feed(&s, t, 0U); t += 10U; }
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MAIN);
    settle(&s, t, 1U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MAIN);
  }

  /* 5. 30 ms 去抖：抖动 20 ms 不算按下 */
  manual_ref_open(&s, 0U);
  {
    uint32_t t = settle(&s, 0U, 1U);
    t = settle(&s, t, 0U);
    feed(&s, t, 1U);
    feed(&s, t + 10U, 0U);
    feed(&s, t + 20U, 1U);
    feed(&s, t + 30U, 0U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_WAITING);
  }

  /* 6. tick 回绕：同样的时间关系结果一致 */
  manual_ref_open(&s, 0xFFFFFF00U);
  {
    uint32_t t = 0xFFFFFF00U;
    t = settle(&s, t, 1U);
    t = settle(&s, t, 0U);
    t = settle(&s, t, 1U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
  }

  /* 7. 手动页：SELECT 长按 2 s 才请求记录；释放不触发归位 */
  manual_ref_open(&s, 0U);
  {
    uint32_t t = settle(&s, 0U, 1U);
    t = settle(&s, t, 0U);
    t = settle(&s, t, 1U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
    /* 松开 USER 不再算“完整按压”（进页那次 PIN 已被 consume） */
    t = settle(&s, t, 0U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
    manual_ref_set_ready(&s, 1U, 1U, 1U);

    /* 刚按下不到 2 s：不请求 */
    t = settle_sel(&s, t, 1U);
    assert(manual_ref_record_requested(&s) == 0U);
    t += 1500U;
    manual_ref_step(&s, t, 0U, 1U, 1U);
    assert(manual_ref_record_requested(&s) == 0U);
    t += 600U;  /* 累计 > 2 s */
    manual_ref_step(&s, t, 0U, 1U, 1U);
    assert(manual_ref_record_requested(&s) == 1U);

    /* 请求只发一次 */
    manual_ref_acknowledge(&s, 1U);
    t += 10U;
    manual_ref_step(&s, t, 0U, 1U, 1U);
    assert(manual_ref_record_requested(&s) == 0U);
    assert(manual_ref_ref_shown(&s) == 1U);

    /* 释放不得归位 */
    t = settle(&s, t, 0U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
    assert(manual_ref_home_requested(&s) == 0U);
  }

  /* 8. 摆位条件不满足(未停稳/手柄无效)时根本不发记录请求；全满足才发。
   *    摇杆没回中由 Ctrl 侧 already 拒绝，这里只测页面自己的两个前置条件。 */
  {
    uint8_t mask;
    for (mask = 0U; mask < 4U; mask++)  /* 每次只满足一个条件以外的组合 */
    {
      uint32_t t = 0U;
      manual_ref_open(&s, 0U);
      manual_ref_enter_manual(&s, t);
      manual_ref_set_ready(&s, (uint8_t)((mask & 1U) != 0U),
                           (uint8_t)((mask & 2U) != 0U), 1U);
      t = settle_sel(&s, t, 0U);   /* 进页时 SELECT 按着 -> 先释放 */
      t = settle_sel(&s, t, 1U);   /* 按下并保持 */
      {
        uint32_t i;
        for (i = 0U; i < 210U; i++)
        {
          manual_ref_step(&s, t, 0U, 1U, 1U);
          t += 10U;
        }
      }
      if (mask == 3U)
      {
        assert(manual_ref_record_requested(&s) == 1U);
      }
      else
      {
        assert(manual_ref_record_requested(&s) == 0U);
      }
      assert(manual_ref_ref_shown(&s) == 0U);
      assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
    }
  }

  /* 9. 进页面前 SELECT 已按住：必须先释放才能计时(防误记参考) */
  manual_ref_open(&s, 0U);
  {
    uint32_t t = 0U;
    manual_ref_enter_manual(&s, t);
    manual_ref_set_ready(&s, 1U, 1U, 1U);
    /* 进页那一轮 SELECT 已按着：先喂帧把它确认成“已按住” */
    t = settle_sel(&s, t, 1U);
    /* 保持 3 s（含去抖帧）也不请求，因为进页前就已按住 */
    {
      uint32_t i;
      for (i = 0U; i < 300U; i++)
      {
        manual_ref_step(&s, t, 0U, 1U, 1U);
        t += 10U;
      }
    }
    assert(manual_ref_record_requested(&s) == 0U);
    t = settle_sel(&s, t, 0U);  /* 必须先释放 */
    t += 3000U;
    manual_ref_step(&s, t, 0U, 0U, 1U);
    assert(manual_ref_record_requested(&s) == 0U);
    t = settle_sel(&s, t, 1U);  /* 再按 2 s 才有效 */
    {
      uint32_t i;
      for (i = 0U; i < 210U; i++)
      {
        manual_ref_step(&s, t, 0U, 1U, 1U);
        t += 10U;
      }
    }
    assert(manual_ref_record_requested(&s) == 1U);
  }

  /* 10. 记录失败(Ctrl 未确认)不显示 REF SET，但仍留在手动页 */
  manual_ref_open(&s, 0U);
  {
    uint32_t t = 0U;
    manual_ref_enter_manual(&s, t);
    manual_ref_set_ready(&s, 1U, 1U, 1U);
    t = settle_sel(&s, t, 0U);  /* 进页即“按着”，先释放 */
    t = settle_sel(&s, t, 1U);
    {
      uint32_t i;
      for (i = 0U; i < 210U; i++)
      {
        manual_ref_step(&s, t, 0U, 1U, 1U);
        t += 10U;
      }
    }
    assert(manual_ref_record_requested(&s) == 1U);
    manual_ref_acknowledge(&s, 0U);
    assert(manual_ref_ref_shown(&s) == 0U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
  }

  /* 11. USER 退出手动页：回主界面，不记录、不归位 */
  manual_ref_open(&s, 0U);
  {
    uint32_t t = settle(&s, 0U, 1U);
    t = settle(&s, t, 0U);
    t = settle(&s, t, 1U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
    t = settle(&s, t, 0U);  /* 松开进门那次（其释放沿被吞掉，不算按压） */
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
    t = settle(&s, t, 1U);  /* 手动页里第二次完整按压 */
    t = settle(&s, t, 0U);  /* 松开才退出 */
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MAIN);
    assert(manual_ref_record_requested(&s) == 0U);
    assert(manual_ref_home_requested(&s) == 0U);
  }

  /* 12. 无效帧不推进去抖，也不推进 2 s 计时 */
  manual_ref_open(&s, 0U);
  {
    uint32_t t = 0U;
    manual_ref_enter_manual(&s, t);
    manual_ref_set_ready(&s, 1U, 1U, 1U);
    for (int i = 0; i < 5; i++)
    {
      manual_ref_step(&s, t, 0U, 1U, 0U);  /* 无效帧 */
      t += 10U;
    }
    assert(manual_ref_press_ms(&s) == 0U);
  }

  /* 13. 正常释放态：窗口打开时 USER 就是释放的，第一次按下就该进页。
   *     不能要求"先按一下再松手"作为预热——滤波器初始状态就是释放，
   *     不会再有释放沿事件送来，干等边沿会让第一下永远被吞。 */
  manual_ref_open(&s, 0U);
  {
    uint32_t t = 0U;
    /* 保持释放 100 ms（真实场景：自检结束时手已经松开） */
    t = hold(&s, t, 0U, 100U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_WAITING);
    /* 第一次按下并保持 150 ms */
    t = settle(&s, t, 1U);
    for (int i = 0; i < 15; i++)
    {
      feed(&s, t, 1U);
      t += 10U;
    }
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
  }

  /* 14. 进窗口那一下仍按着 USER：必须先释放，再按才进页（不能直接进）。
   *     与 13 互补，确保修复没有把"自检结束那一下"也算成入口。 */
  manual_ref_open(&s, 0U);
  {
    uint32_t t = 0U;
    t = hold(&s, t, 1U, 150U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_WAITING);
    t = settle(&s, t, 0U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_WAITING);
    t = settle(&s, t, 1U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
  }

  /* 15. 菜单入口 open_manual：跳过 USER 窗口直接进手动页，立即可记录。
   *     进页时 SELECT 仍视为“按住”：先释放，再按满 2 s 即请求。 */
  manual_ref_open_manual(&s, 0U);
  {
    uint32_t t = 0U;
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
    manual_ref_set_ready(&s, 1U, 1U, 1U);
    t = settle_sel(&s, t, 0U);   /* 释放进门时按住的 SELECT */
    t = settle_sel(&s, t, 1U);   /* 按下并保持 */
    {
      uint32_t i;
      for (i = 0U; i < 210U; i++)
      {
        manual_ref_step(&s, t, 0U, 1U, 1U);
        t += 10U;
      }
    }
    assert(manual_ref_record_requested(&s) == 1U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
  }

  /* 16. 连续静止（ready 置位）+ 长按 2 s 才请求：静止不足时按满 3 s 也不请求。 */
  manual_ref_open_manual(&s, 0U);
  {
    uint32_t t = 0U;
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
    t = settle_sel(&s, t, 0U);   /* 释放进门时按住的 SELECT */
    manual_ref_set_ready(&s, 0U, 1U, 1U);   /* 尚未连续静止满 300 ms */
    t = settle_sel(&s, t, 1U);
    {
      uint32_t i;
      for (i = 0U; i < 300U; i++)
      {
        manual_ref_step(&s, t, 0U, 1U, 1U);
        t += 10U;
      }
    }
    assert(manual_ref_record_requested(&s) == 0U);

    manual_ref_set_ready(&s, 1U, 1U, 1U);   /* 连续静止满 300 ms 后 ready 置位 */
    t += 10U;
    manual_ref_step(&s, t, 0U, 1U, 1U);
    assert(manual_ref_record_requested(&s) == 1U);
  }

  /* 17. 回归：第一次合法请求应答成功即显示 REF SET；失败应答不显示成功，
   *     且失败后再次长按仍可重新请求。 */
  manual_ref_open_manual(&s, 0U);
  {
    uint32_t t = 0U;
    manual_ref_set_ready(&s, 1U, 1U, 1U);
    t = settle_sel(&s, t, 0U);
    t = settle_sel(&s, t, 1U);
    {
      uint32_t i;
      for (i = 0U; i < 210U; i++)
      {
        manual_ref_step(&s, t, 0U, 1U, 1U);
        t += 10U;
      }
    }
    assert(manual_ref_record_requested(&s) == 1U);
    manual_ref_acknowledge(&s, 0U);
    assert(manual_ref_ref_shown(&s) == 0U);
    assert(manual_ref_record_requested(&s) == 0U);

    t = settle_sel(&s, t, 0U);   /* 释放后重新按满 2 s */
    t = settle_sel(&s, t, 1U);
    {
      uint32_t i;
      for (i = 0U; i < 210U; i++)
      {
        manual_ref_step(&s, t, 0U, 1U, 1U);
        t += 10U;
      }
    }
    assert(manual_ref_record_requested(&s) == 1U);
    manual_ref_acknowledge(&s, 1U);
    assert(manual_ref_ref_shown(&s) == 1U);
    assert(manual_ref_state(&s) == MANUAL_REF_STATE_MANUAL);
  }

  puts("USER window / debounce / manual page / record conditions OK");
  return 0;
}
