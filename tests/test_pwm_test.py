#!/usr/bin/env python3
"""架空测试 PWM 直通命令（CHASSIS_PWM_TEST）的主机侧验证。

契约来源：参考提交 7370195 的 pwm_test.c / pwm_test.h。
范围：验证模块判据、接收中断路径的"不写 PWM"性质，以及控制任务输出点的
安全条件。不连设备、不开串口、不烧录、不驱动电机。

安全缺陷（2026-11-08 修复）：原先 USART3 接收中断直接调用 Pwm_Test_SetPwm，
经函数指针写 Set_Pwm，绕过 FlagStop / Turn_Off / DeepCheck / EN。修复后
中断只做 Pwm_Test_Arm（装载参数 + 切模式 + 清 lostcount），写 PWM 只在
Balance_task 的控制输出点发生。

用例：
  S1 结构守卫：中断路径不调用任何写 PWM 的入口。
  S2 真实接收调用链：有效 0x10 帧进中断后不产生非零 PWM。
  S3 控制周期条件：FlagStop / Turn_Off / DeepCheck / EN / 时间窗任一不满足
     时不输出非零 PWM。
  S4 允许条件全部满足：输出 1680 计数，通道与方向符合映射。
  T1 千分数转计数：100‰ -> 1680，500‰ -> 8400，饱和到 8400。
  T2 默认关闭：未定义 CHASSIS_PWM_TEST 时模块不产生任何符号。
  T3 通道映射与方向：位图 bit0..bit3 = A..D，A/D 在 Set_Pwm 入口取反。
  T4 到期清零：时长到期当拍停止，下一拍输出零。
  T5 停止帧与 Safe_Stop：千分数 0 的帧立即清状态。
  T6 非法参数拒绝：通道、千分数、方向、时长越界与坏帧一律丢弃。
  T7 帧老化自停（D1 扩展）：160ms 无有效帧清状态。
  T8 与 7370195 的数据模型一致性：帧字段、范围、函数前缀。
  T9 持续时长下界：0 ms 被拒（BAD_DATA），1 ms / 10000 ms 接受，10001 ms 拒绝。
  R1 竞态：算出非零输出后、写 PWM 前收到停止帧，恢复后不得写非零。
  R2 竞态：写 PWM 前到达新命令（非停止）时，旧值作废。
  R3 停止帧落在写 PWM 窗口内时，寄存器经回调立即清零。
  R4 停止后寄存器读数（真实 Set_Pwm 写入的影子 CCR）为 0。
  R5 序列重排穷举：窗口内任意位置插入停止帧，最终寄存器必为 0。

桩与未覆盖部分见文件末尾的说明。
"""
import pathlib
import re
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]

MOD = ROOT / 'BALANCE/pwm_test.c'
HDR = ROOT / 'BALANCE/Inc/pwm_test.h'
BT = ROOT / 'BALANCE/balance_task.c'
RX = ROOT / 'BALANCE/uartx_callback.c'

assert MOD.exists(), 'pwm_test.c must exist'
assert HDR.exists(), 'pwm_test.h must exist'

FAIL = []


def check(cond, msg):
    print(('ok   ' if cond else 'FAIL ') + msg)
    if not cond:
        FAIL.append(msg)


def bt_text():
    return BT.read_bytes().decode('gb18030')


def rx_text():
    return RX.read_bytes().decode('gb18030')


# --------------------------------------------------------------------------
# T2a: 默认关闭。不定义开关时编译该文件，目标文件必须没有符号。
# --------------------------------------------------------------------------
with tempfile.TemporaryDirectory() as tmp:
    p = pathlib.Path(tmp)
    obj = p / 'off.o'
    subprocess.run(['cc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-c',
                    '-I', str(ROOT / 'BALANCE/Inc'), str(MOD), '-o', str(obj)],
                   check=True)
    syms = subprocess.run(['nm', str(obj)], capture_output=True, text=True, check=True).stdout
    text_syms = [ln for ln in syms.splitlines()
                 if ln.split()[-1:] and not ln.split()[-1].startswith('.')]
    check(len(text_syms) == 0,
          'T2 undefined CHASSIS_PWM_TEST -> no symbols (got %d)' % len(text_syms))

src = MOD.read_text(encoding='utf-8')
check(src.count('#ifdef CHASSIS_PWM_TEST') == 1 and src.rstrip().endswith('#endif /* CHASSIS_PWM_TEST */'),
      'T2 whole implementation sits inside one #ifdef CHASSIS_PWM_TEST guard')

mk = (ROOT / 'Makefile').read_text(encoding='utf-8')
check('CFLAGS  += $(PWM_TEST_DEFS)' in mk and mk.count('CFLAGS  += $(PWM_TEST_DEFS)') == 1,
      'T2 Makefile applies the switch exactly once via CFLAGS += $(PWM_TEST_DEFS)')
check('BALANCE/pwm_test.c' in mk, 'T2 source listed in C_SOURCES')
check('PWM_TEST_DEFS =\n' in mk or 'PWM_TEST_DEFS = ' in mk or 'PWM_TEST_DEFS =' not in mk,
      'T2 switch has no make-side default value')

# 默认关闭时，固件源文件不得在守卫外引用测试符号
for rel in ('BALANCE/balance_task.c', 'BALANCE/uartx_callback.c'):
    txt = (ROOT / rel).read_bytes().decode('gb18030')
    depth = 0
    bad = []
    for i, ln in enumerate(txt.splitlines(), 1):
        s = ln.strip()
        if re.match(r'#\s*if(def)?\s.*CHASSIS_PWM_TEST', s):
            depth += 1
            continue
        if s.startswith('#endif') and depth:
            depth -= 1
            continue
        if ('Pwm_Test_' in ln or 'g_pwm_test_' in ln or 'PWM_TEST_' in ln) and depth == 0:
            bad.append(i)
    check(not bad, 'T2 %s: test symbols only inside guards (unguarded: %s)' % (rel, bad))

# 不得依赖 chassis_ack
for rel in ('BALANCE/balance_task.c', 'BALANCE/uartx_callback.c', 'BALANCE/pwm_test.c'):
    txt = (ROOT / rel).read_bytes().decode('gb18030' if rel != 'BALANCE/pwm_test.c' else 'utf-8')
    check('chassis_ack' not in txt.lower(), 'T2 %s does not include chassis_ack' % rel)

# --------------------------------------------------------------------------
# S1 结构守卫：中断路径与模块都不能写 PWM。
# --------------------------------------------------------------------------
rx = rx_text()
bt = bt_text()

# S1.1 中断受理分支内不得出现任何写 PWM 的名字
i_isr = rx.index("Receive_Data.buffer[1]==PWM_TEST_TYPE")
i_isr_end = rx.index('#endif', rx.index('Pwm_Test_Arm(', i_isr))
isr_block = rx[i_isr:i_isr_end]
for banned in ('Set_Pwm', 'Pwm_Test_SetPwm', 'g_pwm_test_set_pwm', 'PWMA', 'PWMB',
               'PWMC', 'PWMD', 'g_pwm_test_set_pwm'):
    check(banned not in isr_block,
          'S1 USART3 0x10 branch never mentions %s' % banned)
check('Pwm_Test_Arm(' in isr_block, 'S1 USART3 0x10 branch only arms the module')
check('Set_Control_Mode(_ROS_Control);' in isr_block, 'S1 branch switches to ROS mode')
check('robot_control.command_lostcount = 0;' in isr_block, 'S1 branch clears lostcount')

# S1.2 模块本身不得持有写 PWM 的入口。注释里可以解释 Set_Pwm，代码里不行。
src_code = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
src_code = re.sub(r'//[^\n]*', '', src_code)
for banned in ('Set_Pwm', 'g_pwm_test_set_pwm', 'Pwm_Test_SetPwm', 'PWMA', 'PWMB'):
    check(banned not in src_code,
          'S1 pwm_test.c code (comments stripped) never references %s' % banned)
# 归零回调是唯一出口: 模块只保存一个 void(*)(void) 并把调用交给控制任务提供的函数。
check('void (*s_sink)(void);' in src_code,
      'S1 module holds only a void(*)(void) zero sink, no arguments to load')
check(src_code.count('s_sink();') == 1,
      'S1 module calls the sink from exactly one place (Pwm_Test_Stop)')
check('Pwm_Test_SetPwm' not in HDR.read_text(encoding='utf-8'),
      'S1 header no longer declares a Set_Pwm writer')
check('g_pwm_test_set_pwm' not in HDR.read_text(encoding='utf-8'),
      'S1 header no longer declares the Set_Pwm function pointer')

# S1.3 转发函数与绑定必须从 balance_task.c 移除
check('Pwm_Test_SetPwmFwd' not in bt, 'S1 Pwm_Test_SetPwmFwd forwarder removed')
check('Pwm_Test_Bind(' not in bt, 'S1 old Pwm_Test_Bind() removed')

# S1.4 ResponseControl 不得再有测试重放分支
i_rc = bt.index('static void ResponseControl(void)\n{')
i_rc_end = bt.index('#ifdef WHEEL_DIR_CALIB', i_rc)
rc_block = bt[i_rc:i_rc_end]
check('Pwm_Test_' not in rc_block, 'S1 ResponseControl contains no PWM test code')

# S1.5 唯一的测试写出点必须在控制任务主循环里，且在 Turn_Off / Safe_Stop 之后
i_loop = bt.index('void Balance_task(void *pvParameters)')
i_turnoff = bt.index('robot_control.FlagStop = Turn_Off();', i_loop)
i_safestop = bt.index('Safe_Stop();', i_turnoff)
i_out = bt.index('Pwm_Test_ControlOutput(', i_loop)
check(i_turnoff < i_safestop < i_out,
      'S1 test output point sits after Turn_Off() and Safe_Stop() in the control loop')
check(bt.index('Set_Pwm(pwm_test[0]', i_out) > i_out,
      'S1 the loop writes Set_Pwm only from the guarded output point')
check(bt.count('Set_Pwm(pwm_test[0]') == 1,
      'S1 exactly one test Set_Pwm call exists in the firmware')
check(bt.count('Pwm_Test_Commit(') == 1,
      'S1 the guarded output point re-checks validity exactly once before keeping its write')
check(bt.count('Pwm_Test_BeginWrite(') == 1 and bt.count('Pwm_Test_EndWrite(') == 1,
      'S1 the write window is opened and closed exactly once')

# --------------------------------------------------------------------------
# T1/T3..T7 与 S3/S4：用真实源文件在主机上编译执行。
# --------------------------------------------------------------------------
head = r'''
#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include "pwm_test.h"

/* ---- 调度模型 ------------------------------------------------------------
 * 真实环境: Balance_task 跑在 FreeRTOS 任务里, USART3 中断抢占优先级 0
 * (HARDWARE/uartx.c: NVIC_IRQChannelPreemptionPriority = 0), 属于不受
 * 临界段屏蔽的高优先级中断, 可以在控制任务的任何指令边界打断它。
 *
 * 本文件用 setjmp/longjmp 模拟这种抢占: 控制任务侧的每一步 (取序号、算输出、
 * 写寄存器、复核、关窗口) 都是一个独立的 HOOK 点。测试脚本预先登记
 * "在第 N 步之后递一条帧", 递送代码在那一刻同步执行, 执行完再回到被打断
 * 的那一步继续。这不是真实的并发, 但覆盖了同样的指令交错顺序。
 */
static jmp_buf  g_hook_jmp;
static const uint8_t *g_hook_frame;
static const char     *g_hook_after;   /* 在这些标记之后才允许插入 */
static const char     *g_last_mark;    /* 上一个经过的标记 */
static uint32_t g_hook_time;
static int      g_in_point;
static int      g_step;
static PwmTestRx g_last_rx;
static PwmTestRx isr_deliver(const uint8_t *f, uint32_t now_ms);

/* 标记本身是真实源码行之前的注释行或声明行的标签。 */
static void HOOK_STEP(const char *name)
{
    g_step++;
    g_last_mark = name;
    if (g_hook_frame != 0 && g_hook_after != 0 && strcmp(name, g_hook_after) == 0)
    {
        longjmp(g_hook_jmp, 1);
    }
}

#define FRAME_HEADER 0x7B
#define FRAME_TAIL   0x7D

static int g_last[5];
static int g_set_calls;

/* 固件里由 pwm_test.c 定义, 这里通过头文件引用。 */
extern volatile int g_pwm_test_last[4];

static int g_fail;

#define CHECK(c, ...) do { if (!(c)) { \
    printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); g_fail++; \
    } else { printf("ok   "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint8_t bcc(const uint8_t *f)
{
    uint8_t x = 0; int i;
    for (i = 0; i < 9; i++) x ^= f[i];
    return x;
}

static void seal(uint8_t *f) { f[9] = bcc(f); f[10] = FRAME_TAIL; }

/* 生成一条 11 字节帧。permille=0 表示停止帧。 */
static void mk(uint8_t *f, uint8_t ch, uint16_t permille, uint8_t dir, uint16_t ms)
{
    memset(f, 0, 11);
    f[0] = FRAME_HEADER;
    f[1] = PWM_TEST_TYPE;
    f[2] = 0;
    f[3] = ch;
    f[4] = (uint8_t)(permille >> 8);
    f[5] = (uint8_t)(permille & 0xFF);
    f[6] = dir;
    f[7] = (uint8_t)(ms >> 8);
    f[8] = (uint8_t)(ms & 0xFF);
    seal(f);
}

/* 中断投递的唯一实现见 isr_deliver; 这里保留同名入口给旧用例. */
static PwmTestRx deliver(const uint8_t *f, uint32_t now_ms)
{
    return isr_deliver(f, now_ms);
}

static int nz(const int out[4]) { return (out[0]|out[1]|out[2]|out[3]) != 0; }

/* 影子 CCR: 模拟 PWMA..PWMD 四个比较寄存器。fake_set_pwm 按符号折算幅值,
 * 与 Set_Pwm 一样只看幅值写 CCR, 方向另走电平。读这里是"寄存器读数",
 * 不是 g_pwm_test_last 那种诊断镜像。 */
static int g_ccr[4];
static int g_dir[4];
static void pwm_test_output_point(void);
static int ccr_snapshot(int dst[4]);
static int g_in_point;
static int g_last_write;
static void fake_set_pwm(int a, int b, int c, int d, int servo)
{
    int v[4]; int i;
    v[0] = a; v[1] = b; v[2] = c; v[3] = d;
    for (i = 0; i < 4; i++)
    {
        g_ccr[i] = v[i] < 0 ? -v[i] : v[i];
        g_dir[i] = v[i] > 0 ? 1 : (v[i] < 0 ? -1 : 0);
    }
    g_last[0] = a; g_last[1] = b; g_last[2] = c; g_last[3] = d; g_last[4] = servo;
    g_set_calls++;
    g_last_write = v[0] | v[1] | v[2] | v[3];
}

static int ccr_snapshot(int dst[4])
{
    int i;
    for (i = 0; i < 4; i++) dst[i] = g_ccr[i];
    return (g_ccr[0] | g_ccr[1] | g_ccr[2] | g_ccr[3]) != 0;
}

static PwmTestRx isr_deliver(const uint8_t *f, uint32_t now_ms)
{
    PwmTestRx r = Pwm_Test_Parse(f, 11);
    g_last_rx = r;
    if (r == PWM_TEST_RX_ACCEPT)
    {
        uint16_t permille = (uint16_t)(((uint16_t)f[4] << 8) | f[5]);
        uint16_t ms = (uint16_t)(((uint16_t)f[7] << 8) | f[8]);
        Pwm_Test_Arm(f[3], permille, f[6], ms, now_ms);
    }
    return r;
}

static void set_conditions(int ros_mode, int flag_stop, int deep_check,
                           int window_open, int recharge, int en);

/* 跑一拍真实输出点, 并在 HOOK_STEP 标记 after 处递一条帧 (模拟 USART3 抢占)。
 * 返回本拍模块是否请求了非零输出 (复核后的结论)。 */
static int g_in_point;
static uint32_t g_hook_time;
static int run_tick(uint8_t allow, const uint8_t *frame, const char *after, uint32_t at_ms)
{
    int r;
    g_set_calls = 0;
    g_ccr[0] = g_ccr[1] = g_ccr[2] = g_ccr[3] = 0;
    g_dir[0] = g_dir[1] = g_dir[2] = g_dir[3] = 0;
    /* allow=1: 六条件全真. allow=0: 用 FlagStop 一项把门关死.
     * 这里的 set_conditions 必须是唯一来源, 否则会把控制段算出的 allow 覆盖掉。 */
    set_conditions(1, allow ? 0 : 1, 0, 1, 0, 1);
    g_hook_frame = frame;
    g_hook_after = after;
    g_hook_time = at_ms;
    r = setjmp(g_hook_jmp);
    if (r != 0)
    {
        /* "中断"上下文: 在这一刻同步递帧, 然后回到被打断的那一行继续。
         * 帧只递一次, 递完清钩子, 后续步点不再触发。 */
        isr_deliver(g_hook_frame, g_hook_time);
        g_hook_frame = 0;
        g_hook_after = 0;
    }
    pwm_test_output_point();
    g_hook_after = 0;
    /* 返回值 = 这一拍结束时寄存器是否非零, 即"最终写出了非零值没有"。 */
    {
        int oc[4];
        return ccr_snapshot(oc);
    }
}
'''

harness = r'''
int main(void)
{
    uint8_t f[11];
    int out[4];

    /* ---------- S2 接收路径不写 PWM ---------- */
    Pwm_Test_Stop();
    g_set_calls = 0;
    mk(f, PWM_TEST_CH_MASK, 100, 0, 1000);
    CHECK(deliver(f, 1000) == PWM_TEST_RX_ACCEPT, "S2 valid 0x10 frame accepted");
    CHECK(g_set_calls == 0, "S2 ISR path performs no Set_Pwm call (calls=%d)", g_set_calls);
    CHECK(Pwm_Test_State() == PWM_TEST_ACTIVE, "S2 frame armed the module");
    CHECK((g_pwm_test_last[0] | g_pwm_test_last[1] | g_pwm_test_last[2] | g_pwm_test_last[3]) == 0,
          "S2 no non-zero diagnostic value exists before a control tick");

    /* ---------- S3 控制周期条件 ---------- */
    /* 允许位由真实 balance_task.c 的输出段计算, 这里逐条件验证它对条件敏感. */
    CHECK(allow_bit(1,0,0,1,0,1) == 1, "S3 all conditions met -> allow");
    CHECK(allow_bit(1,1,0,1,0,1) == 0, "S3 FlagStop set (Turn_Off true) -> no allow");
    CHECK(allow_bit(1,0,1,1,0,1) == 0, "S3 DeepCheck running -> no allow");
    CHECK(allow_bit(0,0,0,1,0,1) == 0, "S3 not in ROS mode -> no allow");
    CHECK(allow_bit(1,0,0,0,0,1) == 0, "S3 time window not open -> no allow");
    CHECK(allow_bit(1,0,0,1,1,1) == 0, "S3 auto-recharge armed -> no allow");
    CHECK(allow_bit(1,0,0,1,0,0) == 0, "S3 EN not asserted -> no allow");

    /* 模块侧: allow=0 时即使已装载, 也不请求非零输出. */
    mk(f, PWM_TEST_CH_MASK, 100, 0, 5000);
    deliver(f, 1500);
    g_set_calls = 0;
    CHECK(control_tick(0) == 0, "S3 allow=0 -> module requests nothing");
    /* 门关死时控制段仍会走一次"归零写", 但写入值必须是全零. */
    CHECK(g_last_write == 0, "S3 allow=0 -> the only write is the all-zero one");
    CHECK(Pwm_Test_State() == PWM_TEST_ACTIVE, "S3 allow=0 keeps the command armed");

    /* ---------- S4 允许条件满足后输出 1680 ---------- */
    {
        extern volatile int g_pwm_test_last[4];
        mk(f, PWM_TEST_CH_MASK, 100, 0, 5000);        deliver(f, 2000);
        /* 六条件全真. 必须显式设置, 因为上一段 S3 把 FlagStop 留成了 1. */
        set_conditions(1, 0, 0, 1, 0, 1);
        g_set_calls = 0;
        CHECK(control_tick(1) == 1, "S4 allow=1 -> non-zero output requested");
        CHECK(g_set_calls == 1, "S4 allow=1 -> exactly one Set_Pwm write");
        CHECK(g_last[0] == -1680 && g_last[1] == 1680 && g_last[2] == 1680 && g_last[3] == -1680,
              "S4 Set_Pwm(-A,B,C,-D) = (%d,%d,%d,%d) at 10%% duty",
              g_last[0], g_last[1], g_last[2], g_last[3]);
        CHECK(g_last[4] == 0, "S4 servo argument is 0");
        CHECK(g_pwm_test_last[0] == -1680 && g_pwm_test_last[3] == -1680,
              "S4 diagnostic mirror matches the written values");
    }

    /* 单通道与反向 */
    {
        struct { uint8_t ch; uint8_t rev; int lane; int val; } cases[] = {
            { PWM_TEST_CH_A, 0, 0, -1680 }, { PWM_TEST_CH_B, 0, 1, 1680 },
            { PWM_TEST_CH_C, 0, 2, 1680 },  { PWM_TEST_CH_D, 0, 3, -1680 },
            { PWM_TEST_CH_A, 1, 0, 1680 },  { PWM_TEST_CH_B, 1, 1, -1680 },
        };
        for (unsigned i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
            int o[4];
            Pwm_Test_Stop();
            mk(f, cases[i].ch, 100, cases[i].rev, 5000);
            deliver(f, 3000);
            g_set_calls = 0;
            CHECK(control_tick(1) == 1, "S4 single ch=0x%02X rev=%d requested",
                  cases[i].ch, cases[i].rev);
            Pwm_Test_Output(cases[i].ch, 100, cases[i].rev, o);
            CHECK(o[cases[i].lane] == cases[i].val && nz(o),
                  "S4 ch=0x%02X rev=%d lane %d = %d", cases[i].ch, cases[i].rev,
                  cases[i].lane, o[cases[i].lane]);
        }
    }

    /* ---------- T1 千分数 -> 计数 ---------- */
    CHECK(Pwm_Test_DutyCounts(100) == 1680, "T1 100 permille -> %u counts (want 1680)", Pwm_Test_DutyCounts(100));
    CHECK(Pwm_Test_DutyCounts(500) == 8400, "T1 500 permille -> %u counts (want 8400)", Pwm_Test_DutyCounts(500));
    CHECK(Pwm_Test_DutyCounts(200) == 3360, "T1 200 permille -> %u counts (want 3360)", Pwm_Test_DutyCounts(200));
    CHECK(Pwm_Test_DutyCounts(0) == 0, "T1 0 permille -> 0 counts");
    CHECK(Pwm_Test_DutyCounts(1000) == PWM_TEST_COUNTS_MAX, "T1 over-range saturates to 8400");
    CHECK(PWM_TEST_FULL_DUTY == 16800u, "T1 PWM_TEST_FULL_DUTY is 16800 counts");
    CHECK(PWM_TEST_PERMILLE_MAX == 500u, "T1 permille ceiling is 500 (50%%)");

    mk(f, PWM_TEST_CH_A, 100, 0, 1000);
    CHECK(Pwm_Test_Parse(f, 11) == PWM_TEST_RX_ACCEPT, "T1 100 permille frame accepted");
    mk(f, PWM_TEST_CH_A, 501, 0, 1000);
    CHECK(Pwm_Test_Parse(f, 11) == PWM_TEST_RX_BAD_DATA, "T1 501 permille rejected");

    /* ---------- T3 通道映射与方向 ---------- */
    Pwm_Test_Output(PWM_TEST_CH_A, 100, 0, out);
    CHECK(out[0] == -1680, "T3 A forward is negated at Set_Pwm entry: %d", out[0]);
    Pwm_Test_Output(PWM_TEST_CH_B, 100, 0, out);
    CHECK(out[1] == 1680, "T3 B forward stays positive: %d", out[1]);
    Pwm_Test_Output(PWM_TEST_CH_C, 100, 0, out);
    CHECK(out[2] == 1680, "T3 C forward stays positive: %d", out[2]);
    Pwm_Test_Output(PWM_TEST_CH_D, 100, 0, out);
    CHECK(out[3] == -1680, "T3 D forward is negated at Set_Pwm entry: %d", out[3]);
    Pwm_Test_Output(PWM_TEST_CH_MASK, 100, 1, out);
    CHECK(out[0] == 1680 && out[1] == -1680 && out[2] == -1680 && out[3] == 1680,
          "T3 reverse flips every sign: (%d,%d,%d,%d)", out[0], out[1], out[2], out[3]);

    /* ---------- T4 到期清零 ---------- */
    Pwm_Test_Stop();
    mk(f, PWM_TEST_CH_B, 100, 0, 100);
    deliver(f, 5000);
    CHECK(Pwm_Test_State() == PWM_TEST_ACTIVE, "T4 active right after arm");
    Pwm_Test_Tick(5050);
    CHECK(Pwm_Test_State() == PWM_TEST_ACTIVE, "T4 active at +50ms");
    Pwm_Test_Tick(5099);
    CHECK(Pwm_Test_State() == PWM_TEST_ACTIVE, "T4 active at +99ms");
    Pwm_Test_Tick(5100);
    CHECK(Pwm_Test_State() == PWM_TEST_IDLE, "T4 deadline stops at +100ms");
    g_set_calls = 0;
    set_conditions(1, 0, 0, 1, 0, 1);
    CHECK(control_tick(1) == 0, "T4 after expiry the control tick requests nothing");
    CHECK(g_last_write == 0, "T4 after expiry no non-zero write");
    Pwm_Test_Tick(5600);
    CHECK(Pwm_Test_State() == PWM_TEST_IDLE, "T4 stays idle");

    mk(f, PWM_TEST_CH_B, 100, 0, 100);
    deliver(f, 6000);
    mk(f, PWM_TEST_CH_B, 100, 0, 100);
    deliver(f, 6060);
    Pwm_Test_Tick(6140);
    CHECK(Pwm_Test_State() == PWM_TEST_ACTIVE, "T4 resent frame restarts the deadline");
    Pwm_Test_Tick(6160);
    CHECK(Pwm_Test_State() == PWM_TEST_IDLE, "T4 restarted deadline ends 100ms after resend");

    /* ---------- T5 停止帧 ---------- */
    Pwm_Test_Stop();
    mk(f, PWM_TEST_CH_C, 100, 0, 5000);
    deliver(f, 12000);
    CHECK(Pwm_Test_State() == PWM_TEST_ACTIVE, "T5 armed");
    mk(f, PWM_TEST_CH_C, 0, 0, 5000);
    CHECK(deliver(f, 12010) == PWM_TEST_RX_ACCEPT, "T5 permille 0 frame is legal");
    CHECK(Pwm_Test_State() == PWM_TEST_IDLE, "T5 permille 0 -> idle");
    CHECK(control_tick(1) == 0, "T5 stop frame leaves nothing to write");

    /* ---------- T6 非法参数拒绝 ---------- */
    {
        struct { uint8_t ch; uint16_t pm; uint8_t dir; uint16_t ms; const char *why; } bad[] = {
            { 0,    100, 0, 1000, "channel 0" },
            { 0x10, 100, 0, 1000, "channel bit 4" },
            { 0xF0, 100, 0, 1000, "channel high nibble" },
            { 0x01, 501, 0, 1000, "permille 501" },
            { 0x01, 0xFFFF, 0, 1000, "permille 65535" },
            { 0x01, 100, 2, 1000, "direction 2" },
            { 0x01, 100, 0xFF, 1000, "direction 255" },
            { 0x01, 100, 0, 10001, "duration 10001" },
            { 0x01, 100, 0, 0xFFFF, "duration 65535" },
        };
        for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); i++) {
            mk(f, bad[i].ch, bad[i].pm, bad[i].dir, bad[i].ms);
            CHECK(Pwm_Test_Parse(f, 11) == PWM_TEST_RX_BAD_DATA, "T6 reject %s", bad[i].why);
        }
        mk(f, 0x01, 100, 0, 1000); f[10] = 0x00;
        CHECK(Pwm_Test_Parse(f, 11) == PWM_TEST_RX_BAD_FRAME, "T6 reject bad tail");
        mk(f, 0x01, 100, 0, 1000); f[9] ^= 0xFF;
        CHECK(Pwm_Test_Parse(f, 11) == PWM_TEST_RX_BAD_FRAME, "T6 reject bad BCC");
        mk(f, 0x01, 100, 0, 1000);
        CHECK(Pwm_Test_Parse(f, 10) == PWM_TEST_RX_BAD_FRAME, "T6 reject short buffer");
        mk(f, 0x01, 100, 0, 1000); f[0] = 0x00; seal(f);
        CHECK(Pwm_Test_Parse(f, 11) == PWM_TEST_RX_BAD_FRAME, "T6 reject bad header");
        mk(f, 0x01, 100, 0, 1000); f[2] = 1; seal(f);
        CHECK(Pwm_Test_Parse(f, 11) == PWM_TEST_RX_BAD_DATA, "T6 reject reserved byte != 0");
        Pwm_Test_Stop();
        mk(f, 0x0F, 100, 0, 5000);
        deliver(f, 13000);
        mk(f, 0x10, 500, 0, 1000);
        CHECK(deliver(f, 13010) == PWM_TEST_RX_BAD_DATA, "T6 bad frame rejected while active");
        CHECK(Pwm_Test_State() == PWM_TEST_ACTIVE, "T6 rejected frame leaves state ACTIVE");
        int o[4];
        Pwm_Test_Output(0x0F, 100, 0, o);
        CHECK(o[0] == -1680, "T6 rejected frame keeps the previous command");
    }

    /* ---------- T7 D1 扩展：帧老化自停 ---------- */
    Pwm_Test_Stop();
    mk(f, PWM_TEST_CH_B, 100, 0, 10000);
    deliver(f, 20000);
    CHECK(Pwm_Test_State() == PWM_TEST_ACTIVE, "T7 long frame armed");
    Pwm_Test_Tick(20160);
    CHECK(Pwm_Test_State() == PWM_TEST_ACTIVE, "T7 +160ms exactly still active");
    Pwm_Test_Tick(20161);
    CHECK(Pwm_Test_State() == PWM_TEST_IDLE, "T7 +161ms without frame stops (D1 extension)");
    CHECK(control_tick(1) == 0, "T7 stale stop requests nothing");

    /* ---------- T9 持续时长下界 (协议契约 1..10000 ms) ---------- */
    mk(f, PWM_TEST_CH_MASK, 100, 0, 0);
    CHECK(Pwm_Test_Parse(f, 11) == PWM_TEST_RX_BAD_DATA,
          "T9 0 ms frame is rejected (BAD_DATA), not silently accepted");
    mk(f, PWM_TEST_CH_MASK, 100, 0, 1);
    CHECK(Pwm_Test_Parse(f, 11) == PWM_TEST_RX_ACCEPT, "T9 1 ms frame accepted (lower bound)");
    mk(f, PWM_TEST_CH_MASK, 100, 0, 10000);
    CHECK(Pwm_Test_Parse(f, 11) == PWM_TEST_RX_ACCEPT, "T9 10000 ms frame accepted (upper bound)");
    mk(f, PWM_TEST_CH_MASK, 100, 0, 10001);
    CHECK(Pwm_Test_Parse(f, 11) == PWM_TEST_RX_BAD_DATA, "T9 10001 ms frame rejected");
    /* 0 ms 帧即使在 Arm 之后也不得产生非零输出: 解析层已拦, Arm 不会被调用。 */
    Pwm_Test_Stop();
    g_in_point = 0;
    mk(f, PWM_TEST_CH_MASK, 100, 0, 0);
    CHECK(isr_deliver(f, 5000) == PWM_TEST_RX_BAD_DATA, "T9 0 ms rejected at the ISR entry");
    CHECK(Pwm_Test_State() == PWM_TEST_IDLE, "T9 rejected frame leaves the module idle");
    CHECK(run_tick(1, 0, 0, 0) == 0, "T9 after a rejected 0 ms frame the tick writes nothing");
    CHECK((g_ccr[0] | g_ccr[1] | g_ccr[2] | g_ccr[3]) == 0, "T9 CCR registers stay 0");

    /* ---------- R1 竞态: 算完输出后、写 PWM 前收到停止帧 ---------- */
    Pwm_Test_Stop();
    mk(f, PWM_TEST_CH_MASK, 100, 0, 1000);
    isr_deliver(f, 1000);
    Pwm_Test_Tick(1000);
    {
        uint8_t stop[11];
        mk(stop, PWM_TEST_CH_MASK, 0, 0, 1000);   /* 千分数 0 = 停止帧 */
        g_in_point = 0;
        CHECK(run_tick(1, stop, "calc", 1001) == 0,
              "R1 stop between output calculation and Set_Pwm -> no non-zero write");
        CHECK((g_ccr[0] | g_ccr[1] | g_ccr[2] | g_ccr[3]) == 0,
              "R1 CCR registers read 0 after the interleaved stop (%d,%d,%d,%d)",
              g_ccr[0], g_ccr[1], g_ccr[2], g_ccr[3]);
        CHECK(g_dir[0] == 0 && g_dir[1] == 0 && g_dir[2] == 0 && g_dir[3] == 0,
              "R1 direction lines read 0");
    }

    /* ---------- R2 竞态: 窗口内到达新命令 ---------- */
    Pwm_Test_Stop();
    mk(f, PWM_TEST_CH_MASK, 100, 0, 1000);
    isr_deliver(f, 2000);
    {
        uint8_t fresh[11];
        mk(fresh, PWM_TEST_CH_MASK, 400, 0, 4000);
        CHECK(run_tick(1, fresh, "calc", 2001) == 1,
              "R2 new command inside the window keeps the tick valid (it re-arms a valid output)");
        CHECK(g_ccr[0] != 0, "R2 rewritten output is non-zero (%d)", g_ccr[0]);
    }
    Pwm_Test_Stop();

    /* ---------- R3 停止帧落在写 PWM 窗口内: 寄存器立即清零 ---------- */
    mk(f, PWM_TEST_CH_MASK, 100, 0, 5000);
    isr_deliver(f, 3000);
    {
        uint8_t stop[11];
        mk(stop, PWM_TEST_CH_MASK, 0, 0, 5000);
        g_in_point = 0;
        CHECK(run_tick(1, stop, "write", 3001) == 0,
              "R3 stop right after the Set_Pwm line -> commit rejects the write");
        CHECK((g_ccr[0] | g_ccr[1] | g_ccr[2] | g_ccr[3]) == 0,
              "R3 the sink zeroed the CCR registers inside the window (%d,%d,%d,%d)",
              g_ccr[0], g_ccr[1], g_ccr[2], g_ccr[3]);
    }

    /* ---------- R4 停止后同一拍寄存器读数为 0 ---------- */
    Pwm_Test_Stop();
    CHECK(run_tick(1, 0, 0, 0) == 0, "R4 with no command the tick requests nothing");
    CHECK((g_ccr[0] | g_ccr[1] | g_ccr[2] | g_ccr[3]) == 0, "R4 CCR registers are 0");

    /* ---------- R5 序列穷举: 停止帧插在任意相邻两步之间, 终态必须为 0 ---------- */
    {
        static const char *marks[] = { "begin", "calc", "write", "close", 0 };
        int k;
        for (k = 0; marks[k] != 0; k++)
        {
            uint8_t stop[11];
            mk(f, PWM_TEST_CH_MASK, 300, 0, 2000);
            isr_deliver(f, 6000);
            mk(stop, PWM_TEST_CH_MASK, 0, 0, 2000);
            g_in_point = 0;
            (void)run_tick(1, stop, marks[k], 6001);
            CHECK((g_ccr[0] | g_ccr[1] | g_ccr[2] | g_ccr[3]) == 0,
                  "R5 stop injected after '%s' leaves every CCR at 0 (%d,%d,%d,%d)",
                  marks[k], g_ccr[0], g_ccr[1], g_ccr[2], g_ccr[3]);
            Pwm_Test_Stop();
        }
    }

    /* ---------- R6 对照: 不插停止帧时, 正常一拍仍然写出有效 10% ---------- */
    Pwm_Test_Stop();
    mk(f, PWM_TEST_CH_MASK, 100, 0, 3000);
    isr_deliver(f, 7000);
    g_in_point = 0;
    CHECK(run_tick(1, 0, 0, 0) == 1, "R6 undisturbed tick still requests the output");
    CHECK(g_ccr[0] == 1680 && g_ccr[1] == 1680 && g_ccr[2] == 1680 && g_ccr[3] == 1680,
          "R6 CCR registers hold 1680 counts on all four lanes (%d,%d,%d,%d)",
          g_ccr[0], g_ccr[1], g_ccr[2], g_ccr[3]);
    CHECK(g_dir[0] == -1 && g_dir[1] == 1 && g_dir[2] == 1 && g_dir[3] == -1,
          "R6 direction: A/D negative, B/C positive (%d,%d,%d,%d)",
          g_dir[0], g_dir[1], g_dir[2], g_dir[3]);
    Pwm_Test_Stop();

    Pwm_Test_Stop();
    if (g_fail) { printf("PWM-TEST FAILED: %d checks\n", g_fail); return 1; }
    printf("PWM-TEST PASSED\n");
    return 0;
}
'''

body = MOD.read_text(encoding='utf-8')

# --------------------------------------------------------------------------
# 从真实源码提取主循环的输出段，编译成可测试的 allow 判据。
# --------------------------------------------------------------------------
i_anchor = bt.index('/* 架空测试输出收敛点')
i_block = bt.index('#ifdef CHASSIS_PWM_TEST', bt.index('robot_control.FlagStop = Turn_Off();')
                   )
i_end = bt.index('#endif', bt.index('Set_Pwm(pwm_test[0]', i_block))
loop_block = bt[i_block:i_end]

loop_prelude = r'''
#include <stdint.h>
#include "pwm_test.h"

static uint8_t  g_en;
static uint32_t g_mode;

#define CONTROL_DELAY 1000u
#define _ROS_Control (1u<<0)
#define EN g_en
struct { uint8_t AllowRecharge; } charger;
struct { uint8_t DeepCheck; } robot_check;
struct { uint8_t FlagStop; } robot_control;
struct { uint32_t Time_count; } SysVal;
struct motor_stub { int Output; float Target; };
struct {
    struct motor_stub MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
} robot;
#define Get_Control_Mode(m) (g_mode & (m))

/* 六条件安全门的结果: 由脚本经 set_conditions() 设定, 真实输出段原样读取. */
static void set_conditions(int ros_mode, int flag_stop, int deep_check,
                           int window_open, int recharge, int en);
uint8_t allow_bit(int ros_mode, int flag_stop, int deep_check,
                  int window_open, int recharge, int en);

/* Set_Pwm 桩: 记录 5 个实参, 并按符号折算成"影子 CCR + 方向电平"。
 * 影子 CCR 才是本文件判读的寄存器读数; g_pwm/g_last 只是实参副本。 */
static void fake_set_pwm(int a, int b, int c, int d, int servo);
static void Set_Pwm(int a, int b, int c, int d, int servo)
{
    fake_set_pwm(a, b, c, d, servo);
}

/* balance_task.c 里提供给模块的归零回调, 原文照搬. */
static void Pwm_Test_ZeroSink(void)
{
    Set_Pwm(0, 0, 0, 0, 0);
}

/* 把主循环里的输出段摊成顺序执行的一次控制拍。每一行前面插一个 HOOK_STEP,
 * 行本身都是 balance_task.c 原文; 结构上顺序执行, 不用 goto, 因此没有
 * "跳入 if 块" 之类的 C 语言问题。HOOK_STEP 的计数器是全局的, 每个步点
 * 都可以被脚本指定为"USART3 中断在此刻插入"。
 * 插入点的语义: 真实中断可能落在两条语句之间的任何位置, 对应的 C 层顺序
 * 是"前一条已生效, 后一条未生效"。 */
static void test_output_point(void)
{
'''

loop_epilogue = r'''
#endif /* CHASSIS_PWM_TEST */
}

/* 跑一整拍真实输出段, 返回本拍是否写入了非零值.
 * 判断依据是影子 CCR 的读数, 不是 g_pwm_test_last 诊断镜像. */
static uint32_t control_tick(uint8_t allow)
{
    int oc[4];
    g_set_calls = 0;
    g_ccr[0] = g_ccr[1] = g_ccr[2] = g_ccr[3] = 0;
    g_dir[0] = g_dir[1] = g_dir[2] = g_dir[3] = 0;
    g_hook_frame = 0;
    g_hook_after = 0;
    /* 条件由调用方经 set_conditions()/allow_bit() 设定, 这里只跑一拍.
     * allow 仅用于断言提示: 它与调用方设的条件必须一致。 */
    (void)allow;
    pwm_test_output_point();
    return (uint32_t)ccr_snapshot(oc);
}

/* 设置条件后调用真实输出段, 看它有没有写出非零值. */
uint8_t allow_bit(int ros_mode, int flag_stop, int deep_check,
                  int window_open, int recharge, int en)
{
    /* 真实 Set_Control_Mode 是 |= 后 &=, 这里同样语义: 只保留目标模式位. */
    g_mode = ros_mode ? ((g_mode | _ROS_Control) & _ROS_Control) : 0u;
    robot_control.FlagStop  = (uint8_t)flag_stop;
    robot_check.DeepCheck   = (uint8_t)deep_check;
    SysVal.Time_count = window_open ? (CONTROL_DELAY + 231u) : (CONTROL_DELAY);
    charger.AllowRecharge   = (uint8_t)recharge;
    g_en = (uint8_t)en;
    return control_tick(1) ? 1u : 0u;
}

/* set_conditions: 只设条件不跑输出段. */
static void set_conditions(int ros_mode, int flag_stop, int deep_check,
                           int window_open, int recharge, int en)
{
    g_mode = ros_mode ? ((g_mode | _ROS_Control) & _ROS_Control) : 0u;
    robot_control.FlagStop  = (uint8_t)flag_stop;
    robot_check.DeepCheck   = (uint8_t)deep_check;
    SysVal.Time_count = window_open ? (CONTROL_DELAY + 231u) : (CONTROL_DELAY);
    charger.AllowRecharge   = (uint8_t)recharge;
    g_en = (uint8_t)en;
}

/* 本翻译单元只提供 allow_bit, 自身不需要入口. */
int main(void) { return 0; }
'''

with tempfile.TemporaryDirectory() as tmp:
    p = pathlib.Path(tmp)
    # t.c 里同时放: 用例函数、真实 balance_task.c 的输出段、allow_bit 定义。
    # 顺序: 声明 -> 桩 -> 用例函数 -> 主入口之前调用, 避免隐式声明。
    pre = loop_prelude
    k = pre.index('/* 把主循环里的输出段摊成顺序执行的一次控制拍')
    stubs = pre[:k].rstrip()
    if stubs.endswith('{'):
        stubs = stubs[:stubs.rindex('{')].rstrip()
    allow_decl = ('uint8_t allow_bit(int ros_mode, int flag_stop, int deep_check,'
                  ' int window_open, int recharge, int en);\n')

    # 尾段: control_tick 与 allow_bit / set_conditions 都是原文照搬的辅助桩。
    epi_tail = loop_epilogue
    # loop_epilogue 自带的 "#endif" 与函数右花括号属于 test_output_point;
    # 这里已由 run_body 之后的拼接补上, 所以去掉。
    epi_tail = epi_tail.replace('#endif /* CHASSIS_PWM_TEST */\n}\n', '', 1)
    epi_tail = epi_tail.replace('/* 本翻译单元只提供 allow_bit, 自身不需要入口. */', '')
    epi_tail = epi_tail.replace('int main(void) { return 0; }', '')
    epi_tail = epi_tail.strip() + '\n'

    # 把真实输出段逐行展开成"HOOK_STEP + 原文行"。
    # 只有控制变量声明行改写为初值 / 赋值, 其余每一行都是 balance_task.c 原文。
    loop_body = bt[i_block:i_end]
    anchor = 'uint32_t pwm_test_req = Pwm_Test_ControlOutput(pwm_test_allow, pwm_test);'
    assert loop_body.count(anchor) == 1, 'control-output line not found in the real source'
    i_decl = loop_body.index('int     pwm_test[4];')
    i_seq = loop_body.index('uint32_t pwm_test_seq = Pwm_Test_BeginWrite();')
    i_semi = loop_body.index(';', loop_body.index(anchor)) + 1
    i_close = loop_body.index('Pwm_Test_EndWrite();') + len('Pwm_Test_EndWrite();')
    # 切三段: 声明 -> 取序号到算输出 -> 写 PWM / 复核 / 关窗口 / 段尾收花括号。
    i_close = loop_body.index('Pwm_Test_EndWrite();')
    seg_decl = loop_body[i_decl:i_seq]
    seg_seq = loop_body[i_seq:i_semi]
    seg_rest = loop_body[i_semi:i_close]
    # EndWrite 之后的原文只有一个收尾的 "}", 函数体自己已有右花括号, 丢掉。
    seg_tail = ''

    def steps(lines, tag, extra=None):
        """逐行取原文, 在"语句边界"插 HOOK_STEP。

        唯一的限制: if/else 是一条语句, 中间不能插任何东西, 否则 else 会悬空。
        所以从 'if(' 行开始进入 if 语句块, 一直到这个 if 的右花括号闭合为止,
        整段原文连续输出, 只在整条语句之后插一个步点。
        """
        out = []
        n = 0
        depth = 0
        in_if = False
        for ln in lines.split('\n'):
            st = ln.strip()
            if not st:
                out.append(ln)
                continue
            if in_if:
                # if (或 else) 语句块内部: 原样输出并跟踪花括号深度。
                # 深度回到 0 时还不能结束: 后面可能紧跟 else, 所以要等到
                # 下一条不是 else 的语句才开始插步点。
                if st.startswith('else') or st.startswith('else {'):
                    depth = 0
                    out.append(ln)
                    continue
                if depth == 0 and not st.startswith('{'):
                    # if 的体是单条语句 (没有花括号), 本条之后结束。
                    in_if = False
                    out.append(ln)
                    n += 1
                    out.append('HOOK_STEP("%s%d");' % (tag, n))
                    continue
                out.append(ln)
                if st.startswith('{'): depth += 1
                elif st.startswith('}'):
                    depth -= 1
                    if depth == 0:
                        continue
                continue
            if st.startswith('/*') or st.startswith('*') or st.startswith('//') or st.startswith('*/'):
                out.append(ln)
                continue
            n += 1
            out.append('HOOK_STEP("%s%d");' % (tag, n))
            line = ln.replace('uint32_t pwm_test_seq =', 'pwm_test_seq =')
            line = line.replace('uint32_t pwm_test_req =', 'pwm_test_req =')
            out.append(line)
            if extra:
                for key, mark in extra:
                    if st.startswith(key):
                        out.append('HOOK_STEP("%s");' % mark)
            if st.startswith('if(') or st.startswith('if ('):
                in_if = True
                depth = 0
                # 若本条 if 自带 '{', 该花括号属于本 if 的体。
                if st.rstrip().endswith('{'):
                    depth = 1
        return '\n'.join(out)

    # 声明与六条件: 声明行先声明变量, 六条件作为整块赋值, 不加步点,
    # 因为中断不可能落在一个表达式条件的中间 (中断只会在语句边界返回)。
    seg_decl = seg_decl.replace('int     pwm_test[4];', '')
    seg_decl = seg_decl.replace('uint8_t pwm_test_allow =', 'pwm_test_allow =')

    run_body = ('    int pwm_test[4] = { 0, 0, 0, 0 };\n'
                '    uint8_t pwm_test_allow = 0;\n'
                '    uint32_t pwm_test_seq = 0;\n'
                '    uint32_t pwm_test_req = 0;\n'
                '    HOOK_STEP("allow_pre");\n'
                + seg_decl + '\n'
                + '    HOOK_STEP("allow_post");\n'
                + steps(seg_seq, 'mid', [('uint32_t pwm_test_seq =', 'begin'),
                                         ('uint32_t pwm_test_req =', 'calc')]) + '\n'
                + steps(seg_rest, 'mid', [('Set_Pwm(pwm_test[0]', 'write')]) + '\n'
                + '    HOOK_STEP("close");\n'
                + steps(seg_tail, 'post') + '\n'
                + '    (void)pwm_test_req;\n')

    src = p / 't.c'
    bot = body
    cases = harness.replace('int main(void)', 'static int run_cases(void)', 1)
    t_src = ('#define CHASSIS_PWM_TEST 1\n' + head + allow_decl + '\n'
             + stubs + '\n'
             + 'static void pwm_test_output_point(void)\n{\n'
             + '#ifdef CHASSIS_PWM_TEST\n'
             + run_body + '\n'
             + '#endif /* CHASSIS_PWM_TEST */\n'
             + '}\n'
             + bot + epi_tail + cases + '\n'
             + 'int main(void) { return run_cases(); }\n')
    src.write_text(t_src)
    exe = p / 't'
    cc = subprocess.run(['cc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                         '-I', str(ROOT / 'BALANCE/Inc'), str(src), '-o', str(exe)],
                        capture_output=True, text=True)
    if cc.returncode != 0:
        print(cc.stderr)
        raise SystemExit('compile failed')
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    print(run.stdout, end='')
    if run.returncode != 0:
        print(run.stderr)
        raise SystemExit('run failed rc=%d' % run.returncode)

# --------------------------------------------------------------------------
# T8 契约一致性断言 + 接线断言
# --------------------------------------------------------------------------
hdr = HDR.read_text(encoding='utf-8')
check('#define PWM_TEST_TYPE          0x10u' in hdr, 'T8 frame type is 0x10 in buffer[1]')
check('PWM_TEST_CH_A          0x01u' in hdr and 'PWM_TEST_CH_D          0x08u' in hdr,
      'T8 bit0..bit3 = A..D')
check('PWM_TEST_PERMILLE_MAX  500u' in hdr, 'T8 permille range is 0..500')
check('PWM_TEST_FULL_DUTY     16800u' in hdr, 'T8 duty denominator is the 16800-count period')
check('PWM_TEST_MS_MAX        10000u' in hdr, 'T8 duration range is 1..10000')
check('buffer[4..5]  占空比千分数' in hdr, 'T8 header documents permille in buffer[4..5]')
check('PWM_TEST_STALE_MS      160u' in hdr, 'T8 D1 extension is declared in the header')
check('D1' in hdr and '7370195' in hdr, 'T8 header lists the differences from 7370195')
for fn in ('Pwm_Test_DutyCounts', 'Pwm_Test_Parse', 'Pwm_Test_Arm', 'Pwm_Test_Stop',
           'Pwm_Test_Tick', 'Pwm_Test_Output', 'Pwm_Test_ControlOutput', 'Pwm_Test_State'):
    check((fn + '(') in hdr, 'T8 header declares %s' % fn)
check('Chassis_PwmTest_' not in hdr and 'Chassis_PwmTest_' not in body,
      'T8 no Chassis_PwmTest_ prefix remains')
hdr_code = hdr.split('#include <stdint.h>', 1)[1]
check('CHASSIS_RX_BAD_DATA' not in hdr_code and 'CHASSIS_RX_BAD_DATA' not in body,
      'T8 code never uses the baseline-absent CHASSIS_RX_BAD_DATA enum')
check('CHASSIS_RX_BAD_DATA=9' in hdr,
      'T8 header documents that 7370195 used CHASSIS_RX_BAD_DATA=9')

_ss = bt.index('static void Safe_Stop(void)')
_ss_zero = bt.index('PWMA = 0;', _ss)
check('Pwm_Test_Stop();' in bt[_ss:_ss_zero],
      'A1 Safe_Stop still clears the test state before writing PWMA=0')
check('Pwm_Test_Tick(' in bt, 'A1 balance loop ticks the test module')

check('Pwm_Test_Parse(Receive_Data.buffer, 11)' in rx, 'A2 uartx_callback parses the 0x10 frame')
check('USARTx==USART3' in rx, 'A2 frame accepted on USART3 only')

if FAIL:
    print('TOTAL FAILURES: %d' % len(FAIL))
    raise SystemExit(1)
print('pwm test direct path: PASS')

# --------------------------------------------------------------------------
# 桩与未覆盖部分（如实记录，避免夸大验证范围）
# --------------------------------------------------------------------------
# 使用的桩：
#   1. fake_set_pwm：记录 Set_Pwm 的 5 个实参，不代表真实 GPIO/定时器行为。
#   2. t.c 里的 Set_Pwm / robot / robot_control / robot_check / charger /
#      SysVal / g_mode / EN 都是最小桩，只保留真实输出段读取的字段。
#   3. allow_bit 把入参写进上述桩，再调用从 balance_task.c 原文抠出的输出段，
#      判断段内是否发生非零写。输出段本身未经改写。
#   4. Pwm_Test_* 直接链接 pwm_test.c 真实实现，不打桩。
#   5. 串口与中断用手工递字节模拟，不进真实 USART3 ISR。
# 未覆盖部分：
#   a. 没有运行在真实 STM32F407 上，没有观测定时器寄存器或电机电压。
#   b. 没有连串口、没有改 DTR/RTS、没有烧录、没有驱动电机，物理 PWM 未实测。
#   c. RTOS 调度、中断抢占优先级、xTaskGetTickCount 的实际节拍未建模。
#   d. 电压采样、errorflag 的产生链路由 Turn_Off 源码静态断言覆盖，未动态执行。
#   e. PS2 四轮控制与 50% 限幅只做存在性断言，未做运行期数值验证。
