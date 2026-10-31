#include "motor_health.h"

void motor_health_init(motor_health_t *h)
{
  uint8_t i;

  for (i = 0U; i < 4U; i++)
  {
    h->cmd_start_ms[i] = 0U;
    h->win_start_ms[i] = 0U;
    h->win_counts[i] = 0;
    h->dir[i] = 0;
    h->window_active[i] = 0U;
  }
  h->fault_mask = 0U;
  h->online_mask = 0U;
}

void motor_health_clear_after_selfcheck(motor_health_t *h, uint8_t selfcheck_passed)
{
  /* 四轮没全过就不清：故障必须保持到真正被修好并重新自检为止。 */
  if (selfcheck_passed != 0U)
  {
    motor_health_init(h);
  }
}

static int32_t mh_abs(int32_t v)
{
  return (v < 0) ? -v : v;
}

static int8_t mh_sign(int32_t v)
{
  if (v > 0)
  {
    return 1;
  }
  if (v < 0)
  {
    return -1;
  }
  return 0;
}

void motor_health_step(motor_health_t *h, const motor_health_in_t *in,
                       motor_health_out_t *out)
{
  uint8_t i;
  uint8_t any = 0U;

  for (i = 0U; i < 4U; i++)
  {
    int32_t pwm = in->pwm[i];
    int8_t dir = mh_sign(pwm);

    out->code[i] = MOTOR_HEALTH_DISABLED;

    /* 不满足检测条件：不累计、不判故障、也不记为在线。 */
    if (in->measurable == 0U || dir == 0 ||
        mh_abs(pwm) < MOTOR_HEALTH_MIN_PWM)
    {
      h->window_active[i] = 0U;
      h->win_counts[i] = 0;
      h->cmd_start_ms[i] = 0U;
      h->dir[i] = 0;
      continue;
    }

    if (h->window_active[i] != 0U && h->dir[i] != dir)
    {
      /* 符号变了（换向）：重新宽限，别把换向过渡算成故障。 */
      h->window_active[i] = 0U;
      h->cmd_start_ms[i] = 0U;
    }

    if (h->window_active[i] == 0U)
    {
      h->window_active[i] = 1U;
      h->dir[i] = dir;
      h->cmd_start_ms[i] = in->now_ms;
      h->win_start_ms[i] = in->now_ms;
      h->win_counts[i] = 0;
      out->code[i] = MOTOR_HEALTH_DISABLED;
      continue;
    }

    /* 启动宽限内不判：电机从静止起步需要时间。 */
    if ((uint32_t)(in->now_ms - h->cmd_start_ms[i]) < MOTOR_HEALTH_STARTUP_GRACE_MS)
    {
      h->win_counts[i] += in->fb[i];
      out->code[i] = MOTOR_HEALTH_DISABLED;
      continue;
    }

    h->win_counts[i] += in->fb[i];

    /* 窗口满：做一次判定，然后开下一个窗口（锁存后不再重复判）。 */
    if ((uint32_t)(in->now_ms - h->win_start_ms[i]) >= MOTOR_HEALTH_WINDOW_MS)
    {
      int64_t net = h->win_counts[i];

      if ((h->fault_mask & (uint8_t)(1U << i)) != 0U)
      {
        out->code[i] = MOTOR_HEALTH_NO_FB;
      }
      else if ((dir > 0 && net <= -(int64_t)MOTOR_HEALTH_MIN_COUNTS) ||
               (dir < 0 && net >= (int64_t)MOTOR_HEALTH_MIN_COUNTS))
      {
        /* 方向相反且幅度足够 */
        h->fault_mask |= (uint8_t)(1U << i);
        out->code[i] = MOTOR_HEALTH_DIR_ERR;
      }
      else if (net < (int64_t)MOTOR_HEALTH_MIN_COUNTS &&
               net > -(int64_t)MOTOR_HEALTH_MIN_COUNTS)
      {
        /* 抖动净零也算不动：反馈异常，不代表已证实离线。 */
        h->fault_mask |= (uint8_t)(1U << i);
        out->code[i] = MOTOR_HEALTH_NO_FB;
      }
      else
      {
        h->online_mask |= (uint8_t)(1U << i);
        out->code[i] = MOTOR_HEALTH_OK;
      }

      h->win_start_ms[i] = in->now_ms;
      h->win_counts[i] = 0;
    }
    else
    {
      out->code[i] = ((h->fault_mask & (uint8_t)(1U << i)) != 0U)
                         ? MOTOR_HEALTH_NO_FB
                         : MOTOR_HEALTH_OK;
    }
  }

  if (h->fault_mask != 0U)
  {
    any = 1U;
  }

  out->fault_mask = h->fault_mask;
  out->stop_all = any;
  out->online_mask = h->online_mask;
}

const char *motor_health_code_text(motor_health_code_t code)
{
  switch (code)
  {
    case MOTOR_HEALTH_OK:       return "ok";
    case MOTOR_HEALTH_NO_FB:    return "no-fb";
    case MOTOR_HEALTH_DIR_ERR:  return "direrr";
    case MOTOR_HEALTH_DISABLED: return "off";
    default:                    return "?";
  }
}
