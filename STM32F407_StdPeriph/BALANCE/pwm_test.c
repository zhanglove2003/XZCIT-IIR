/* PWM 直通架空测试的实现。只在定义 CHASSIS_PWM_TEST 时编译。
 *
 * 安全约束：本文件不含任何写 PWM 的能力。它没有硬件头文件，也不持有
 * Set_Pwm 的函数指针。接收中断只通过 Pwm_Test_Arm 装载参数；真正的写出
 * 只发生在 Balance_task 的控制路径里，由 Pwm_Test_ControlOutput 给出数值，
 * 由控制任务在 FlagStop、DeepCheck 等判断全部通过后自行调用 Set_Pwm。
 * 这样中断路径在结构上无法越过安全分支。
 */

#ifdef CHASSIS_PWM_TEST

#include "pwm_test.h"

#define FRAME_HEADER_VAL  0x7Bu
#define FRAME_TAIL_VAL    0x7Du
#define IDX_TYPE          1u
#define IDX_RESERVED      2u
#define IDX_CHANNEL       3u
#define IDX_DUTY_HI       4u
#define IDX_DUTY_LO       5u
#define IDX_DIR           6u
#define IDX_MS_HI         7u
#define IDX_MS_LO         8u
#define IDX_CHECK         9u
#define IDX_TAIL          10u
#define FRAME_LEN         11u

/* 最近一次命令请求的四路入口值，供诊断读取。由控制任务每拍更新。 */
volatile int g_pwm_test_last[4];
/* 控制任务每拍调用带 allow 的接口时累加。测试用它确认调用来源。 */
volatile uint32_t g_pwm_test_ctrl_calls;

/* 装载序号: 每次改变输出含义 (启动、千分数 0 停止帧、Stop) 都加一。
 * 控制任务算出输出时取一次, 写 PWM 前再取一次; 两次不等说明中途有停止帧
 * 或新命令到来, 本次算出的数值作废。 */
static volatile uint32_t s_arm_seq;
/* 控制任务正在写 PWM 的闸门深度。大于 0 时 Stop/Tick 顺带清寄存器,
 * 以消除"算输出到写 PWM 之间的停止帧"造成的延时窗口。 */
static volatile uint32_t s_inflight;

static PwmTestState s_state = PWM_TEST_IDLE;
static uint32_t s_deadline_ms;
static uint32_t s_last_rx_ms;
static uint32_t s_arm_count;
/* PWM 归零回调。由控制任务在 Pwm_Test_BeginWrite 处经 Pwm_Test_BindSink
 * 提供一个只写零的函数; 本模块自己从不写 PWM。 */
static void (*s_sink)(void);

/* 已装载的命令参数。只在 Pwm_Test_Arm 里写，只在 ControlOutput 里读。 */
static uint8_t  s_mask;
static uint16_t s_permille;
static uint8_t  s_reverse;

static uint8_t frame_bcc(const uint8_t *d)
{
    uint8_t x = 0u;
    uint8_t i;
    for (i = 0u; i < IDX_CHECK; i++)
    {
        x ^= d[i];
    }
    return x;
}

uint16_t Pwm_Test_DutyCounts(uint16_t permille)
{
    uint32_t counts = (uint32_t)permille * (uint32_t)PWM_TEST_FULL_DUTY / 1000u;
    if (counts > (uint32_t)PWM_TEST_COUNTS_MAX) counts = (uint32_t)PWM_TEST_COUNTS_MAX;
    return (uint16_t)counts;
}

PwmTestRx Pwm_Test_Parse(const uint8_t *data, uint16_t len)
{
    uint16_t permille;
    uint16_t ms;

    if (data == 0 || len < FRAME_LEN) return PWM_TEST_RX_BAD_FRAME;
    if (data[0] != FRAME_HEADER_VAL) return PWM_TEST_RX_BAD_FRAME;
    if (data[IDX_TAIL] != FRAME_TAIL_VAL) return PWM_TEST_RX_BAD_FRAME;
    if (data[IDX_CHECK] != frame_bcc(data)) return PWM_TEST_RX_BAD_FRAME;
    if (data[IDX_TYPE] != PWM_TEST_TYPE) return PWM_TEST_RX_BAD_DATA;
    if (data[IDX_RESERVED] != 0u) return PWM_TEST_RX_BAD_DATA;

    if (data[IDX_CHANNEL] == 0u) return PWM_TEST_RX_BAD_DATA;
    if ((data[IDX_CHANNEL] & (uint8_t)~PWM_TEST_CH_MASK) != 0u)
        return PWM_TEST_RX_BAD_DATA;

    permille = (uint16_t)(((uint16_t)data[IDX_DUTY_HI] << 8) | data[IDX_DUTY_LO]);
    if (permille > PWM_TEST_PERMILLE_MAX) return PWM_TEST_RX_BAD_DATA;
    if (data[IDX_DIR] > 1u) return PWM_TEST_RX_BAD_DATA;

    /* 契约是 1..10000 ms。0 ms 必须在解析层拒绝: 若放行, Pwm_Test_Arm 会把
     * 到期时刻设成 now, Pwm_Test_Tick 要到下一拍才清状态, 同一拍的
     * ControlOutput 仍会给出非零值, 等于执行了一条零时长命令。 */
    ms = (uint16_t)(((uint16_t)data[IDX_MS_HI] << 8) | data[IDX_MS_LO]);
    if (ms < PWM_TEST_MS_MIN) return PWM_TEST_RX_BAD_DATA;
    if (ms > PWM_TEST_MS_MAX) return PWM_TEST_RX_BAD_DATA;

    return PWM_TEST_RX_ACCEPT;
}

void Pwm_Test_Arm(uint8_t mask, uint16_t permille, uint8_t reverse,
                  uint16_t duration_ms, uint32_t now_ms)
{
    /* 参数已由 Pwm_Test_Parse 判过；这里只装载，不写非零 PWM。 */
    s_last_rx_ms = now_ms;
    if (permille == 0u)
    {
        Pwm_Test_Stop();
        return;
    }
    s_mask     = mask;
    s_permille = permille;
    s_reverse  = reverse;
    s_deadline_ms = now_ms + (uint32_t)duration_ms;
    s_state = PWM_TEST_ACTIVE;
    s_arm_count++;
    s_arm_seq++;
}

void Pwm_Test_Stop(void)
{
    /* 只清状态。写零由控制任务在下一拍按 allow 路径完成，因此这里不碰 PWM。
     * 唯一的例外: 控制任务正处在"已取序号、未写 PWM"的窗口内时, 本函数
     * 顺带让回调把寄存器清掉, 不让停止帧多等到下一拍。 */
    s_state = PWM_TEST_IDLE;
    s_deadline_ms = 0u;
    s_permille = 0u;
    s_mask = 0u;
    s_reverse = 0u;
    s_arm_seq++;
    g_pwm_test_last[0] = 0;
    g_pwm_test_last[1] = 0;
    g_pwm_test_last[2] = 0;
    g_pwm_test_last[3] = 0;
    if (s_inflight != 0u && s_sink != 0)
    {
        s_sink();
    }
}

void Pwm_Test_Tick(uint32_t now_ms)
{
    if (s_state != PWM_TEST_ACTIVE) return;
    if (now_ms >= s_deadline_ms)
    {
        Pwm_Test_Stop();
        return;
    }
    /* D1 扩展：无有效帧超过 PWM_TEST_STALE_MS 即归零。 */
    if ((now_ms - s_last_rx_ms) > (uint32_t)PWM_TEST_STALE_MS)
    {
        Pwm_Test_Stop();
    }
}

PwmTestState Pwm_Test_State(void)
{
    return s_state;
}

uint32_t Pwm_Test_ArmCount(void)
{
    return s_arm_count;
}

void Pwm_Test_Output(uint8_t mask, uint16_t permille, uint8_t reverse,
                     int out[4])
{
    int value = (int)Pwm_Test_DutyCounts(permille);
    uint8_t i;

    if (reverse) value = -value;
    for (i = 0u; i < 4u; i++)
    {
        out[i] = ((mask >> i) & 1u) ? value : 0;
    }
    /* 位序 A、B、C、D 与 Set_Pwm 形参顺序一致。A、D 在 Set_Pwm 入口要
     * 取反，与四轮模式 Set_Pwm(-right,...,-right,0) 同号约定一致。 */
    out[0] = -out[0];
    out[3] = -out[3];
}

uint32_t Pwm_Test_ControlOutput(uint8_t allow, int out[4])
{
    uint32_t requested = 0u;
    uint8_t i;

    g_pwm_test_ctrl_calls++;

    if (allow && s_state == PWM_TEST_ACTIVE && s_permille != 0u)
    {
        Pwm_Test_Output(s_mask, s_permille, s_reverse, out);
        for (i = 0u; i < 4u; i++)
        {
            if (out[i] != 0) requested = 1u;
        }
    }
    else
    {
        for (i = 0u; i < 4u; i++) out[i] = 0;
    }

    g_pwm_test_last[0] = out[0];
    g_pwm_test_last[1] = out[1];
    g_pwm_test_last[2] = out[2];
    g_pwm_test_last[3] = out[3];
    return requested;
}

uint32_t Pwm_Test_Seq(void)
{
    return s_arm_seq;
}

uint32_t Pwm_Test_BeginWrite(void)
{
    uint32_t seq = s_arm_seq;
    s_inflight++;
    return seq;
}

void Pwm_Test_BindSink(void (*sink)(void))
{
    s_sink = sink;
}

void Pwm_Test_EndWrite(void)
{
    if (s_inflight != 0u) s_inflight--;
}

uint32_t Pwm_Test_Commit(uint32_t seq, const int out[4])
{
    uint32_t i;

    /* 本次输出仍然有效: 序号没变, 状态仍为 ACTIVE, 且四路都不为零。
     * 序号不等说明窗口内发生了停止帧、新命令或 Stop, 本次数值作废。 */
    if (seq != s_arm_seq || s_state != PWM_TEST_ACTIVE) return 0u;
    for (i = 0u; i < 4u; i++)
    {
        if (out[i] != 0) return 1u;
    }
    return 0u;
}

#endif /* CHASSIS_PWM_TEST */
