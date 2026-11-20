#include "motor_safety.h"
#include <string.h>

/*--------------------------------------------------------------------------
 * 模块概况
 *
 * 本文件实现三项安全机制：
 * 1. 全局安全状态机，覆盖上电、急停、故障、失联与来源切换。
 * 2. PS2 来源门禁，按时序确认中位并要求 START。
 * 3. 输出仲裁与需求超时，保证没有任何来源索要运动时目标归零。
 *
 * 另有反馈故障监测。所有阈值都是待实车确认的建议值。
 *--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------
 * 1. 全局安全状态机
 *--------------------------------------------------------------------------*/
static MOTOR_SAFE_STATE s_state;
static uint16_t         s_reason;
static uint8_t          s_estop_latched; /* 急停需人工复位，防止自动恢复 */

void Motor_Safety_Init(void)
{
	s_state         = MOTOR_SAFE_INHIBIT;
	s_reason        = SAFE_REASON_POWERON;
	s_estop_latched = 0;
}

void Motor_Safety_Inhibit(uint16_t reason)
{
	s_state  = MOTOR_SAFE_INHIBIT;
	s_reason = reason;

	/* 急停与故障必须重新准备，不能自动回到等待中位 */
	if( reason & (SAFE_REASON_ESTOP | SAFE_REASON_ERROR | SAFE_REASON_FAULT) )
		s_estop_latched = 1;
}

/* 把状态强制拉回禁止。供失联、故障、需求超时等路径调用。 */
static void Safety_FallToInhibit(uint16_t reason)
{
	s_state  = MOTOR_SAFE_INHIBIT;
	s_reason = reason;
}

/*--------------------------------------------------------------------------
 * 安全状态推进
 * enable==1 表示 Turn_Off() 判定不允许运动.
 * allow   为来源门禁的结论, 1 表示当前来源已完成「中位 -> START -> 推杆」.
 *
 * 状态上升路径:
 *   INHIBIT --(enable==0 且无锁存)--> WAIT_NEUTRAL
 *   WAIT_NEUTRAL --(来源门禁放行)--> READY
 *   READY --(本周期仍有来源登记需求)--> RUN
 *   RUN --(本周期无需求)--> READY  (由通用超时路径处理)
 *--------------------------------------------------------------------------*/
MOTOR_SAFE_STATE Motor_Safety_Update(uint8_t enable)
{
	if( enable )
	{
		/* 失能: 记录原因, 不允许运动. 急停与故障需要人工确认后才解除锁存. */
		if( !(s_reason & (SAFE_REASON_ESTOP | SAFE_REASON_ERROR)) )
			Safety_FallToInhibit(SAFE_REASON_ESTOP);
		else
			s_state = MOTOR_SAFE_INHIBIT;
		return s_state;
	}

	/* 失能原因已解除, 允许回到等待中位. 来源侧仍需重新准备. */
	if( MOTOR_SAFE_INHIBIT == s_state && !s_estop_latched )
	{
		s_state  = MOTOR_SAFE_WAIT_NEUTRAL;
		s_reason = 0;
	}

	return s_state;
}

/*--------------------------------------------------------------------------
 * 来源门禁放行后推进状态. 每个控制周期在 Motor_Safety_Update 之后调用一次.
 * allow==1 表示当前来源已准备好, 可以进入准备态.
 *--------------------------------------------------------------------------*/
MOTOR_SAFE_STATE Motor_Safety_Allow(uint8_t allow)
{
	if( MOTOR_SAFE_WAIT_NEUTRAL == s_state && allow )
	{
		s_state  = MOTOR_SAFE_READY;
		s_reason = 0;
	}
	return s_state;
}

/* 本周期有来源登记了运动需求. 只有 READY 才能进入 RUN. */
MOTOR_SAFE_STATE Motor_Safety_MotionPresent(uint8_t present)
{
	if( present && MOTOR_SAFE_READY == s_state )
	{
		s_state  = MOTOR_SAFE_RUN;
		s_reason = 0;
	}
	else if( !present && MOTOR_SAFE_RUN == s_state )
	{
		/* 本周期没有运动需求: 回到准备态, 下个周期需要重新拿到需求. */
		s_state = MOTOR_SAFE_READY;
	}
	return s_state;
}

MOTOR_SAFE_STATE Motor_Safety_GetState(void)
{
	return s_state;
}

uint16_t Motor_Safety_GetReason(void)
{
	return s_reason;
}

uint8_t Motor_Safety_Output_Allowed(void)
{
	return ( MOTOR_SAFE_RUN == s_state ) ? 1u : 0u;
}

uint16_t Motor_Safety_NeutralHoldTicks(void)
{
	return SAFE_NEUTRAL_HOLD_TICKS;
}

uint16_t Motor_Safety_LostTicks(void)
{
	return SAFE_PS2_LOST_TICKS;
}

/*--------------------------------------------------------------------------
 * 2. PS2 来源门禁
 *
 * 进入顺序：有效帧 → 连续中位 ≥ SAFE_NEUTRAL_HOLD_TICKS → 按一次 START
 *           → 再次推杆才允许运动。
 * 失联或重新配对后撤销准备状态与 START 锁存。
 *--------------------------------------------------------------------------*/
enum
{
	PS2_SRC_INVALID = 0, /* 无有效帧 */
	PS2_SRC_NEUTRAL,     /* 帧有效，正在累计中位 */
	PS2_SRC_READY,       /* 中位已确认并已按 START */
	PS2_SRC_RUN          /* 已收到有效非中位指令 */
};

static uint16_t s_ps2_state;
static uint16_t s_ps2_reason;
static uint16_t s_ps2_neutral_ticks;

void Ps2_Safety_Init(void)
{
	s_ps2_state         = PS2_SRC_INVALID;
	s_ps2_reason        = SAFE_REASON_POWERON;
	s_ps2_neutral_ticks = 0;
}

/* 收到一帧通过校验的手柄数据。*/
void Ps2_Safety_OnValidFrame(uint8_t stick_at_neutral, uint8_t start_pressed)
{
	if( PS2_SRC_INVALID == s_ps2_state )
	{
		/* 重新接通. 此帧只表示手柄在通信, 摇杆是否可信由调用方判定.
		 * 先进入「等待中位」, 中位计数从 0 起算, 不在这里放行. */
		s_ps2_state         = PS2_SRC_NEUTRAL;
		s_ps2_reason        = 0;
		s_ps2_neutral_ticks = 0;
	}

	if( PS2_SRC_NEUTRAL == s_ps2_state )
	{
		if( stick_at_neutral )
		{
			if( s_ps2_neutral_ticks < SAFE_NEUTRAL_HOLD_TICKS )
				s_ps2_neutral_ticks++;

			if( s_ps2_neutral_ticks >= SAFE_NEUTRAL_HOLD_TICKS && start_pressed )
			{
				s_ps2_state  = PS2_SRC_READY;
				s_ps2_reason = 0;
			}
		}
		else
		{
			/* 中位被打断，重新计时。此前按下的 START 不算数。 */
			s_ps2_neutral_ticks = 0;
		}
		return;
	}

	if( PS2_SRC_READY == s_ps2_state )
	{
		if( !stick_at_neutral )
			s_ps2_state = PS2_SRC_RUN;
		return;
	}

	/* PS2_SRC_RUN：摇杆回中后回到已准备，等待下一次推杆。 */
	if( stick_at_neutral )
		s_ps2_state = PS2_SRC_READY;
}

/* 帧校验失败或超时。撤销准备状态和 START 锁存。 */
void Ps2_Safety_OnInvalidFrame(void)
{
	if( PS2_SRC_INVALID != s_ps2_state )
		s_ps2_reason = SAFE_REASON_LOST;

	s_ps2_state         = PS2_SRC_INVALID;
	s_ps2_neutral_ticks = 0;
}

uint8_t Ps2_Safety_SourceReady(void)
{
	return ( PS2_SRC_INVALID == s_ps2_state ) ? 0u : 1u;
}

uint8_t Ps2_Safety_MotionAllowed(void)
{
	return ( PS2_SRC_RUN == s_ps2_state ) ? 1u : 0u;
}

uint16_t Ps2_Safety_State(void)
{
	return s_ps2_state;
}

uint16_t Ps2_Safety_Reason(void)
{
	return s_ps2_reason;
}

/*--------------------------------------------------------------------------
 * 3. 输出仲裁
 *
 * 每个能触发运动的来源在收到新指令时登记「需求」。
 * 需求在超时后自动撤销。全部需求撤销后目标速度归零。
 * 不同来源各自计时，互不刷新。
 *--------------------------------------------------------------------------*/
#define OUTPUT_ARB_SRC_NUM 8u

struct output_requirement
{
	uint8_t  active;      /* 当前是否有需求 */
	uint8_t  ever_seen;   /* 本来源是否曾登记过需求 */
	uint8_t  lost;        /* 本来源是否曾超时 */
	uint16_t valid_ticks; /* 需求有效期 */	
	uint16_t none_ticks;  /* 无需求持续计时 */
	uint16_t age;         /* 需求已持续计数 */
};

static struct output_requirement s_req[OUTPUT_ARB_SRC_NUM];
static uint16_t s_lost_src_mask; /* 已锁存的失联来源 */
static uint8_t  s_arb_ever;      /* 任一来曾登记过需求 */
static uint16_t s_motion_ticks;  /* 最近一次需求的有效期 */
static uint16_t s_motion_age;    /* 距最近一次需求的周期数 */
static uint8_t  s_motion_timed_out;

/* 登记一条新指令。每收到一条指令调用一次。 */
void Output_Arb_SetRequirement(uint16_t src, uint16_t valid_ticks)
{
	if( src >= OUTPUT_ARB_SRC_NUM ) return;

	s_req[src].active      = 1;
	s_req[src].valid_ticks = valid_ticks;
	s_req[src].age         = 0;
	s_req[src].none_ticks  = 0;

	/* 本来源重新活动，清除它的失联锁存 */
	s_lost_src_mask   &= (uint16_t)~(1u << src);
	s_arb_ever         = 1;
	s_motion_ticks     = valid_ticks;
	s_motion_age       = 0;
	s_motion_timed_out = 0;
}

/* 来源主动撤销需求。none_ticks 为保持静默的周期数。 */
void Output_Arb_ClearRequirement(uint16_t src, uint16_t none_ticks)
{
	if( src >= OUTPUT_ARB_SRC_NUM ) return;

	if( s_req[src].active )
	{
		s_req[src].active = 0;
		s_lost_src_mask  |= (uint16_t)(1u << src);
	}
	s_req[src].none_ticks = none_ticks;
}

/* 每控制周期调用一次。 */
void Output_Arb_Tick(void)
{
	uint8_t i;
	uint8_t any_active = 0;

	/* 第一步: 推进每条需求的寿命, 并统计仍有几条需求在有效期内. */
	for( i = 0; i < OUTPUT_ARB_SRC_NUM; i++ )
	{
		if( !s_req[i].active ) continue;

		if( s_req[i].age < s_req[i].valid_ticks ) s_req[i].age++;

		if( s_req[i].age >= s_req[i].valid_ticks )
		{
			/* 寿命到期: 撤销并锁存失联. 该来源需要重新准备. */
			s_req[i].active = 0;
			s_lost_src_mask |= (uint16_t)(1u << i);
			continue;
		}

		any_active = 1;

		/* 第二步: 全局超时只跟随「最新一次指令」.
		 * 用每条来源自己的年龄减去全局计数, 等价于从每帧指令起算.
		 * 这样最后一条指令到期的那一拍, 全局计数同时到期,
		 * 不需要等寿命到期后再从零累加一遍. */
		if( s_req[i].age > s_motion_age ) s_motion_age = s_req[i].age;
	}

	if( any_active )
	{
		s_motion_timed_out = 0;
		return;
	}

	if( !s_arb_ever ) return;

	/* 全部来源无需求: 继续累计, 到 SAFETY_MOTION_TIMEOUT_TICKS 后判超时. */
	if( s_motion_age < s_motion_ticks ) s_motion_age++;
	if( s_motion_age >= s_motion_ticks ) s_motion_timed_out = 1;
}

/* 返回 1 表示执行机构需求已超时，运动目标必须归零。 */
uint16_t Output_Require_Timeout(void)
{
	return s_motion_timed_out ? 1u : 0u;
}

/* 返回 1 表示本周期至少有一个来源登记了运动需求。 */
uint8_t Output_Arb_Motion_Present(void)
{
	uint8_t i;

	for( i = 0; i < OUTPUT_ARB_SRC_NUM; i++ )
	{
		if( s_req[i].active ) return 1u;
	}
	return 0u;
}

/* 返回 1 表示某一来源已锁存失联。用于失联后要求重新准备。 */
uint8_t Output_Arb_SourceLost(uint16_t src)
{
	if( src >= OUTPUT_ARB_SRC_NUM ) return 0u;
	return ( s_lost_src_mask & (1u << src) ) ? 1u : 0u;
}

/*--------------------------------------------------------------------------
 * 4. 反馈故障监测
 *
 * 每路按目标方向符号比较反馈。目标明显非零、PWM 已达较大值、
 * 而反馈反号并持续 MOTOR_FAULT_NO_FEEDBACK_TICKS 时判故障。
 * 若完全没有反馈（绝对值很小）同样判故障。
 *--------------------------------------------------------------------------*/
struct motor_fault
{
	uint16_t reverse_ticks; /* 反馈与目标反号的持续计数 */
	uint16_t dead_ticks;    /* 无反馈的持续计数 */
};

static struct motor_fault s_fault[MOTOR_FAULT_WHEEL_NUM];
static uint16_t s_fault_reason;

#define MOTOR_FAULT_DUTY_MIN 1500        /* PWM 判定门限，低于此值不判故障 */
#define MOTOR_FAULT_TARGET_MIN 0.05f     /* 目标速度判定门限，m/s */
#define MOTOR_FAULT_FEEDBACK_MIN 0.02f   /* 反馈绝对值门限，m/s */
#define MOTOR_FAULT_ARM_TICKS 20u        /* 非零目标持续 200 ms 后才开始判 */

void Motor_Fault_Feed(uint8_t wheel, float target, float feedback, uint16_t pwm)
{
	struct motor_fault *f;

	if( wheel >= MOTOR_FAULT_WHEEL_NUM ) return;
	f = &s_fault[wheel];

	if( target > MOTOR_FAULT_TARGET_MIN || target < -MOTOR_FAULT_TARGET_MIN )
	{
		if( pwm > MOTOR_FAULT_DUTY_MIN )
		{
			/* 目标与非零反馈反号，视为未按预期转动 */
			if( ( target > 0 && feedback < -MOTOR_FAULT_FEEDBACK_MIN ) ||
			    ( target < 0 && feedback >  MOTOR_FAULT_FEEDBACK_MIN ) )
			{
				if( f->reverse_ticks < MOTOR_FAULT_NO_FEEDBACK_TICKS ) f->reverse_ticks++;
			}
			else if( feedback < MOTOR_FAULT_FEEDBACK_MIN && feedback > -MOTOR_FAULT_FEEDBACK_MIN )
			{
				if( f->dead_ticks < MOTOR_FAULT_NO_FEEDBACK_TICKS ) f->dead_ticks++;
			}
			else
			{
				f->reverse_ticks = 0;
				f->dead_ticks    = 0;
			}
		}
	}
	else
	{
		f->reverse_ticks = 0;
		f->dead_ticks    = 0;
	}

	if( f->reverse_ticks >= MOTOR_FAULT_NO_FEEDBACK_TICKS ||
	    f->dead_ticks    >= MOTOR_FAULT_NO_FEEDBACK_TICKS )
	{
		s_fault_reason |= (uint16_t)(1u << wheel);
	}
}

void Motor_Fault_Tick(uint8_t enable)
{
	if( enable )
	{
		memset(s_fault, 0, sizeof(s_fault));
		s_fault_reason = 0;
	}
}

uint8_t Motor_Fault_Active(void)
{
	return ( s_fault_reason != 0 ) ? 1u : 0u;
}

uint16_t Motor_Fault_Reason(void)
{
	return s_fault_reason;
}
