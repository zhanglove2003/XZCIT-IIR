#ifndef C50C_PWM_TEST_H
#define C50C_PWM_TEST_H

/* PWM 直通架空测试（只在定义 CHASSIS_PWM_TEST 时参与编译）。
 *
 * 目的：给架空工况一个固定占空比入口。10% 占空比 = 1680 个 PWM 计数
 * （定时器周期 16800 计数的 10%）。
 *
 * 契约来源：参考提交 7370195 的 pwm_test.h / pwm_test.c。帧类型、数据区
 * 布局、千分数单位、时长上限、函数前缀都与该提交一致。
 *
 * 帧格式（11 字节，帧头 0x7B，帧尾 0x7D，BCC = buffer[0..8] 异或）：
 *   buffer[0]     0x7B
 *   buffer[1]     0x10         帧类型 PWM_TEST_TYPE
 *   buffer[2]     0x00         保留，必须为 0
 *   buffer[3]     通道位图     bit0=A bit1=B bit2=C bit3=D，非 0 且在 0x0F 内
 *   buffer[4..5]  占空比千分数 16 位大端，范围 0..500
 *   buffer[6]     方向         0=正向 1=反向
 *   buffer[7..8]  持续时间毫秒 16 位大端，范围 1..10000
 *   buffer[9]     BCC          buffer[0..8] 的异或
 *   buffer[10]    0x7D
 *
 * 千分数转计数：duty_counts = permille * 16800 / 1000。10% 写作 100。
 *
 * 方向按 Set_Pwm 的入口符号解释。A、D 在 Set_Pwm 内部取反，所以通道 A 的
 * "正向"就是 Set_Pwm 的 -value，与四轮模式里 Set_Pwm(-right,...,-right,0)
 * 的约定一致。
 *
 * 安全约束：本模块不写 PWM。接收中断只装载参数（Pwm_Test_Arm）。控制任务
 * 每拍调用 Pwm_Test_ControlOutput(allow, out)，只有 allow 为真且命令有效时
 * 才回填非零数值，由控制任务在 FlagStop / Turn_Off / DeepCheck / EN 判断
 * 全部通过的分支里自行写 Set_Pwm。中断路径不持有写 PWM 的入口。
 *
 * 停止路径：持续时间到期清状态（保留）；Safe_Stop 调用 Pwm_Test_Stop 清状态
 * （保留）；160ms 无有效帧视为链路失效并清零（*扩展*，见下）；
 * 模式切到 PS2/APP/航模时直通条件不成立，输出交回原路径（行为同"不再写"）。
 * 上述停止只清状态，写零由控制任务下一拍的 ControlOutput 完成。
 *
 * 与 7370195 的差异（本基线额外加的部分，逐条列出）：
 *   D1 帧老化自停：7370195 没有。本基线跨总线到主机, 主机掉电时最后一帧
 *      会一直生效到时长用完, 所以加 STALE_MS=160 判定。160ms 取基线
 *      balance_task.c 里 PS2 输入失效窗口 150ms 的同一量级, 留 10ms 余量。
 *      测试 T7 覆盖。若不需要可删掉 tick 里的老化分支, 其余不受影响。
 *   D2 千分数 0 作为停止帧：7370195 的判决是 0 合法, 落到
 *      duty_counts=0。本实现同样先检查 0 并在 Arm 里归零, 等价。
 *   D3 帧类型号放在 buffer[1]：7370195 用 Chassis_Ack_CheckFrame 取类型,
 *      本基线没有 chassis_ack.c/.h, 所以在这里直接判 buffer[1]。
 *   D4 非法参数处理：7370195 返回 CHASSIS_RX_BAD_DATA=9。本基线没有该枚举,
 *      处理方式为"丢弃该帧、不写电机、不改既有输出、不改状态", 返回值
 *      只在模块内部使用（见 PwmTestRx）。
 *
 * 依赖：本模块不引用 chassis_ack.c/.h，也不需要该文件里的任何枚举。
 */

#include <stdint.h>

#define PWM_TEST_TYPE          0x10u

#define PWM_TEST_CH_A          0x01u
#define PWM_TEST_CH_B          0x02u
#define PWM_TEST_CH_C          0x04u
#define PWM_TEST_CH_D          0x08u
#define PWM_TEST_CH_MASK       0x0Fu

/* 定时器周期 16800 计数；50% 上限 8400 计数。 */
#define PWM_TEST_FULL_DUTY     16800u
#define PWM_TEST_PERMILLE_MAX  500u
#define PWM_TEST_COUNTS_MAX    8400u

#define PWM_TEST_MS_MIN        1u
#define PWM_TEST_MS_MAX        10000u
/* D1 扩展：无有效帧超过该毫秒数即归零。 */
#define PWM_TEST_STALE_MS      160u

typedef enum {
    PWM_TEST_IDLE   = 0,
    PWM_TEST_ACTIVE = 1
} PwmTestState;

typedef enum {
    PWM_TEST_RX_ACCEPT    = 0,
    PWM_TEST_RX_BAD_FRAME = 1,   /* 帧头、帧尾、BCC、长度不对 */
    PWM_TEST_RX_BAD_DATA  = 2    /* 帧结构正确但参数越界：丢弃，不写电机 */
} PwmTestRx;

/* 千分数转 PWM 计数，结果饱和到 8400。 */
uint16_t Pwm_Test_DutyCounts(uint16_t permille);

/* 帧数据区解析与合法性判定。不访问任何变量或硬件。
 * data 指向 buffer，len 为缓冲区长度，实际要求 len >= 11。 */
PwmTestRx Pwm_Test_Parse(const uint8_t *data, uint16_t len);

/* 启动或更新一次输出。参数必须已经过 Pwm_Test_Parse 判定。
 * permille 为 0 时等同于停止。now_ms 用于计算到期时刻。 */
void Pwm_Test_Arm(uint8_t mask, uint16_t permille, uint8_t reverse,
                  uint16_t duration_ms, uint32_t now_ms);

/* 立即停止。Safe_Stop 与到期都走这里。只清状态，不写 PWM。 */
void Pwm_Test_Stop(void);

/* 由 Balance_task 每拍调用，检查到期（D1 还检查帧老化）。 */
void Pwm_Test_Tick(uint32_t now_ms);

PwmTestState Pwm_Test_State(void);

/* 5 参数转发函数类型，与 static void Set_Pwm(int,int,int,int,int) 一致。
 * 本模块不再使用函数指针；保留该类型是为了不改变对外可见的类型集合。 */
typedef void (*set_pwm_fwd_f)(int, int, int, int, int);

/* 最近一次控制任务给出的四路入口值，A/B/C/D 顺序。 */
extern volatile int g_pwm_test_last[4];

/* 控制任务调用带 allow 的接口的累计次数，供测试断言调用来源。 */
extern volatile uint32_t g_pwm_test_ctrl_calls;

/* 按通道位图与方向一次性算四路 Set_Pwm 入口值。纯计算，不访问硬件。 */
void Pwm_Test_Output(uint8_t mask, uint16_t permille, uint8_t reverse,
                     int out[4]);

/* 控制任务每拍调用。allow 为 0 时一律回填四路零。
 * 返回 1 表示本拍请求了非零输出，调用方据此决定是否写 Set_Pwm。
 * 这个函数是唯一的输出来源；接收中断不调用它。 */
uint32_t Pwm_Test_ControlOutput(uint8_t allow, int out[4]);

/* 累计启动次数，供测试断言。 */
uint32_t Pwm_Test_ArmCount(void);

/* ---- 装载序号：写 PWM 与停止帧竞态的唯一判据 ----
 * 序号在每次改变输出含义时加一：Pwm_Test_Arm 接受新命令、千分数 0 的停止帧
 * （经 Pwm_Test_Stop）、以及所有 Pwm_Test_Stop 调用。 */

/* 读当前装载序号。 */
uint32_t Pwm_Test_Seq(void);

/* 控制任务进入"写 PWM"临界段时调用。先取序号，再声明窗口已打开。
 * 返回的序号交给 Pwm_Test_Commit 复核。 */
uint32_t Pwm_Test_BeginWrite(void);

/* 绑定 PWM 归零回调。回调只能写零，由控制任务提供；本模块不持有写非零的
 * 入口。 */
void Pwm_Test_BindSink(void (*sink)(void));

/* 控制任务写完本次 PWM 后调用，关闭窗口。 */
void Pwm_Test_EndWrite(void);

/* 写 PWM 前复核：序号未变且状态仍为 ACTIVE 时返回 1，否则返回 0。
 * 返回 0 表示本次算出的数值作废，调用方不得写非零值。 */
uint32_t Pwm_Test_Commit(uint32_t seq, const int out[4]);

#endif
