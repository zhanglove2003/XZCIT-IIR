#ifndef __MOTOR_SAFETY_H
#define __MOTOR_SAFETY_H

#include "system.h"
#include "balance_task.h"

/*--------------------------------------------------------------------------
 * 电机安全状态与输出门禁
 *
 * 计划依据：docs/superpowers/plans/2026-11-04-c50c-ps2-motor-safety.md
 * 任务 2「先实现统一安全停止」、任务 3「PS2 输入与模式切换」。
 *
 * 设计约束：
 * 1. 所有控制来源共用本模块，不允许各来源自行决定输出。
 * 2. 安全停止不给输出过程，直接清零并禁止输出。
 * 3. 只有完成「有效帧 → 连续中位 → START → 新推杆」后才允许运动。
 * 4. 本模块只做软件判定。驱动器物理失能仍需硬件措施，不能由本模块代替。
 *--------------------------------------------------------------------------*/

/* 安全状态 */
typedef enum
{
	MOTOR_SAFE_INHIBIT = 0, /* 禁止：上电、复位、急停、失联、故障 */
	MOTOR_SAFE_WAIT_NEUTRAL,/* 等待中位：已选来源，正在确认中位 */
	MOTOR_SAFE_READY,       /* 已准备：中位确认并已按 START，等待新推杆 */
	MOTOR_SAFE_RUN          /* 运动：已收到有效非中位指令 */
} MOTOR_SAFE_STATE;

/* 进入「禁止」状态的原因，按位保存 */
#define SAFE_REASON_POWERON    (1u<<0) /* 上电或复位 */
#define SAFE_REASON_ESTOP      (1u<<1) /* 急停开关或软件急停 */
#define SAFE_REASON_ERROR      (1u<<2) /* 自检报错 */
#define SAFE_REASON_LOST       (1u<<3) /* 有效帧超时 */
#define SAFE_REASON_SWITCH     (1u<<4) /* 控制来源切换 */
#define SAFE_REASON_NOTNEUTRAL (1u<<5) /* 选源时摇杆不在中位 */
#define SAFE_REASON_FAULT      (1u<<6) /* 反馈缺失或需求超时 */

/* PS2 时序参数，单位是 PS2 任务周期（标称 20 Hz，即 50 ms）
 * 两个值都是待实车验证的建议值，不能用本次断电试验推算。 */
#define SAFE_NEUTRAL_HOLD_TICKS  2u  /* 连续中位 ≥ 100 ms */
#define SAFE_PS2_LOST_TICKS      5u  /* 有效帧丢失 ≥ 250 ms 判失联 */
#define SAFE_REASON_INVALID      0u  /* Get_Actuator_Timeout 的默认值，表示该项不检查 */

/* 初始化。上电和软件复位后必须调用一次。 */
void Motor_Safety_Init(void);

/* 进入「禁止」。reason 为 SAFE_REASON_* 位或。 */
void Motor_Safety_Inhibit(uint16_t reason);

/* 实车是否允许运动。enable 由调用方根据 Turn_Off 结果填写，0 表示允许。
 * 返回下一控制周期的安全状态。 */
MOTOR_SAFE_STATE Motor_Safety_Update(uint8_t enable);
MOTOR_SAFE_STATE Motor_Safety_Allow(uint8_t allow);
MOTOR_SAFE_STATE Motor_Safety_MotionPresent(uint8_t present);

/* 读取当前安全状态与最近一次失能原因。 */
MOTOR_SAFE_STATE Motor_Safety_GetState(void);
uint16_t        Motor_Safety_GetReason(void);

/* 安全状态允许输出电机转矩时返回 1。 */
uint8_t Motor_Safety_Output_Allowed(void);

/* 供各控制来源查询的时序常量。 */
uint16_t Motor_Safety_NeutralHoldTicks(void);
uint16_t Motor_Safety_LostTicks(void);

/*--------------------------------------------------------------------------
 * PS2 来源门禁接口
 *--------------------------------------------------------------------------*/
void     Ps2_Safety_OnValidFrame(uint8_t stick_at_neutral, uint8_t start_pressed);
void     Ps2_Safety_OnInvalidFrame(void);
uint8_t  Ps2_Safety_SourceReady(void);
uint8_t  Ps2_Safety_MotionAllowed(void);
uint16_t Ps2_Safety_State(void);
uint16_t Ps2_Safety_Reason(void);
void     Ps2_Safety_Init(void);

/*--------------------------------------------------------------------------
 * 输出仲裁接口
 *--------------------------------------------------------------------------*/
void Output_Arb_SetRequirement(uint16_t src, uint16_t valid_ticks);
void Output_Arb_ClearRequirement(uint16_t src, uint16_t none_ticks);
void Output_Arb_Tick(void);
uint8_t Output_Arb_SourceLost(uint16_t src);
uint8_t Output_Arb_Motion_Present(void);

/* 输出仲裁来源编号。每个能触发运动的来源占用一个编号。 */
enum
{
	OUTPUT_SRC_APP   = 0,
	OUTPUT_SRC_RC    = 1,
	OUTPUT_SRC_PS2   = 2,
	OUTPUT_SRC_CAN   = 3,
	OUTPUT_SRC_UART  = 4,
	OUTPUT_SRC_ROS   = 5,
	OUTPUT_SRC_CHARGER = 6,
	OUTPUT_SRC_NUM   = 7
};

/*--------------------------------------------------------------------------
 * 故障监测接口
 *--------------------------------------------------------------------------*/
void Motor_Fault_Feed(uint8_t wheel, float target, float feedback, uint16_t pwm);
void Motor_Fault_Tick(uint8_t enable);
uint8_t Motor_Fault_Active(void);
uint16_t Motor_Fault_Reason(void);

/* 轮位编号，对应 motor.h 的 A、B、C、D 四路输出 */
#define MOTOR_FAULT_WHEEL_A 0u
#define MOTOR_FAULT_WHEEL_B 1u
#define MOTOR_FAULT_WHEEL_C 2u
#define MOTOR_FAULT_WHEEL_D 3u
#define MOTOR_FAULT_WHEEL_NUM 4u

/* 反馈丢失判定参数。时间和阈值需按编码器分辨率与实车数据确认，当前值为建议值。 */
#define MOTOR_FAULT_TICK_RATE_HZ     100u  /* 控制任务频率 */
#define MOTOR_FAULT_NO_FEEDBACK_TICKS 50u  /* 无反馈持续 500 ms 判故障 */
#define MOTOR_FAULT_COMPARE_HOLD_TICKS 30u /* 判决持续 300 ms 才确认，滤掉换向抖动 */

/*--------------------------------------------------------------------------
 * 执行机构需求超时接口
 *--------------------------------------------------------------------------*/
//一条运动指令的有效期, 单位控制周期. 20 对应主循环 100 Hz 下的 200 ms.
//来源每个周期重新登记, 所以只要指令持续存在就不会超时.
//一旦来源停止发送, 200 ms 后目标强制归零.
#define SAFETY_MOTION_TIMEOUT_TICKS 20u

/* 台架调试开关: 定义后, 安全门禁跳过来源检查, 允许直接输出.
 * 只用于台架验证电机链路, 严禁在实车/未固定轮子的情况下启用. */
//#define SAFETY_BENCH_BYPASS 1

uint16_t Output_Require_Timeout(void);
/*--------------------------------------------------------------------------
 * SWD 观测区: 把运行时状态写到固定 RAM 地址, 用 ST-Link 直接读回.
 * 地址选在 .bss 结束(0x20009de8)之后、栈底之前的空闲 RAM.
 * 调试完成后删除本段.
 *--------------------------------------------------------------------------*/
#define SAFETY_TRACE_ADDR   ((volatile uint32_t *)0x2001F000u)
#define SAFETY_TRACE_MAGIC  0x5A5A5A5Au

static inline void Safety_Trace(uint32_t idx, uint32_t val)
{
	SAFETY_TRACE_ADDR[idx] = val;
}

#endif /* __MOTOR_SAFETY_H */
