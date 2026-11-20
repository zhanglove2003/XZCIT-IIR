"""本轮改动固件树的**逐字提取**模型：ROS 速度帧 → 三个轴 → 四轮输出 / 停车。

与 ``fw_ros_mode_model.py`` 的区别
----------------------------------
那个模型绑定 ``baseline-src/`` 的未改动基线，``ready`` 判据与输出映射是
手写 MODEL 替代。本模型改从**本轮实际修改的固件树**
``2026-11-08-50pct-migrate/`` 读取源码，并且**逐字插入真实函数体**：

1. ``BALANCE/uartx_callback.c``：``UartxControll_Callback``、
   ``XYZ_Target_Speed_transition`` —— 真实帧状态机与解码，含本轮新增的
   ``ros_cmd_tick`` 时间戳（在 ``#if defined(ROS_STRAIGHT_DRIVE)`` 里）。
2. ``BALANCE/balance_task.c``：``ResponseControl`` 的
   ``REAR_WHEEL_DEBUG`` + ``FOUR_TURN_DEBUG`` 段、``Turn_Off``、``Safe_Stop``
   —— 是**整段 Ctrl+A 复制**，不是重写。段内调用到的
   其它函数（``Drive_Motor``/``wheelCoefficient``/``Incremental_MOTOR``）在
   段外，用最小桩顶替，不在断言的路径上。
3. ``BALANCE/steer_debug.c``：``axis_duty`` / ``side_step`` / ``steer_debug_step``。
4. ``BALANCE/data_task.c``：``Check_BCC``。

因此"四轮输出"与"停车"这两条结论，来自本轮真实改动的固件源码，而不是
对它的再描述。

本模块仍然**只是模型**：不连串口、不烧录、不驱动电机。它证明的是
"给定这些帧，真实固件函数体算出什么占空比"，不证明板上执行。
"""
from __future__ import annotations

import hashlib
import pathlib
import subprocess
import tempfile

#: 本轮改动的固件工作目录（由另一个子代理并行修改，故不绑定整树哈希）。
FIRMWARE_DIR = (pathlib.Path(__file__).resolve().parents[4]
                / '2026-11-08-50pct-migrate')

#: 本模型依赖的四个文件，及本模型写作时它们的哈希。哈希不符即报错，
#: 迫使重新核对提取是否仍然对得上（固件一旦再改，本模型必须重审）。
RECORDED_SHA256 = {
    'BALANCE/uartx_callback.c':
        'b41f33d33b5df408df77c7b97ac92e4d26eb1a416927320984b5701a93b6c201',
    'BALANCE/balance_task.c':
        '4dbc3073d258874a0ee3449086284f26237fdf1179b3ce6147a0773c31069f29',
    'BALANCE/Inc/balance_task.h':
        '24825f14837211e073282fe1d908185453197c9e12a462cf306581ea47e298e4',
    'BALANCE/steer_debug.c':
        'd43e19defd61a01c8e5d6b0785788630bb483814f38205c888edb1d0d73e1c2a',
    'BALANCE/data_task.c':
        '1fbe665587444a9413c1562cb1cd602238a66ab572b42ab7eb8547e5a722e414',
}


def _load(rel: str) -> str:
    return (FIRMWARE_DIR / rel).read_bytes().decode('gb18030', errors='replace')


def verify_sources() -> dict:
    """校验本模型依赖的固件文件哈希；不符就报错，不静默继续。"""
    seen = {}
    for rel, want in RECORDED_SHA256.items():
        path = FIRMWARE_DIR / rel
        if not path.exists():
            raise AssertionError(f'缺少固件源文件 {rel}（目录 {FIRMWARE_DIR}）')
        got = hashlib.sha256(path.read_bytes()).hexdigest()
        if got != want:
            raise AssertionError(
                f'{rel} 哈希与本模型记录不一致: {got!r} != {want!r}；'
                '固件已变动，请重审提取')
        seen[rel] = got
    return seen


def _block(text: str, anchor: str, skip: int = 0) -> str:
    """截取以 ``anchor`` 开头、花括号配平的那一段源码。"""
    i = -1
    for _ in range(skip + 1):
        i = text.index(anchor, i + 1)
    j = text.index('{', i)
    depth = 0
    for k in range(j, len(text)):
        depth += (text[k] == '{') - (text[k] == '}')
        if depth == 0:
            return text[i:k + 1]
    raise ValueError(f'未配平: {anchor}')


#: 固件侧类型与常量，逐字取自 ``BALANCE/Inc/balance_task.h``、
#: ``USER/system.h``、``BALANCE/Inc/steer_debug.h``。
_PREAMBLE = r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;

#define FRAME_HEADER 0x7B
#define FRAME_TAIL   0x7D
#define RECEIVE_DATA_SIZE 11
#define BALANCE_TASK_RATE 100
#define CONTROL_DELAY 1000
#define V1_0 0

/* 控制模式位（BALANCE/Inc/balance_task.h:84-92），同名同值 */
enum {
    _ROS_Control = (1 << 0), _PS2_Control = (1 << 1), _APP_Control = (1 << 2),
    _RC_Control = (1 << 3), _CAN_Control = (1 << 4), _USART_Control = (1 << 5),
};
typedef void *USART_TypeDef;
static USART_TypeDef USART1 = (void *)1;
static USART_TypeDef USART3 = (void *)3;

#define Set_Control_Mode(mask) (robot_control.ControlMode |= (mask), \
                                robot_control.ControlMode &= (mask))
#define Get_Control_Mode(mask) (robot_control.ControlMode & (mask))

/* MODEL：FreeRTOS 的 tick 与 pdMS_TO_TICKS。真实 tick 周期 1 ms。 */
static uint32_t g_free_tick = 0;
static uint32_t xTaskGetTickCount(void) { return g_free_tick; }
#define pdMS_TO_TICKS(x) ((uint32_t)(x))

/* ROS_CMD_TIMEOUT_MS 与 ros_cmd_tick 取自真实头文件，这里用同样的宏定义
   从源码里抽出，保证模型与固件共用同一个阈值。若不定义 ROS_STRAIGHT_DRIVE，
   真实头文件就不含该字段，模型也如实失败。 */
#define ROS_STRAIGHT_DRIVE 1
/* 编译开关与固件 make OPT 一致：REAR_WHEEL_DEBUG + FOUR_TURN_DEBUG 才走
   四轮调试输出段；缺一就落到另一分支，模型如实失败而不是装作通过。 */
#define REAR_WHEEL_DEBUG 1
#define FOUR_TURN_DEBUG 1
#define portTICK_PERIOD_MS 1
/* 逐字提取 Chassis PWM test 块要用的枚举；值取自 pwm_test.h。 */
enum { PWM_TEST_RX_ACCEPT = 0 };
/* MODEL：Chassis PWM test 块只负责接管模式并装载参数。本模型只关心
 * "该块会不会吞掉基线速度帧"，所以用计数桩代替真实 Pwm_Test_Arm。 */
static int g_pwm_arm_calls = 0;
static void Pwm_Test_Arm(uint8_t a, uint16_t b, uint8_t c, uint16_t d,
                         uint32_t e) {
    (void)a; (void)b; (void)c; (void)d; (void)e;
    g_pwm_arm_calls++;
}
static int Pwm_Test_Parse(const uint8_t *buf, int len) {
    (void)buf; (void)len;
    return -1;   /* 除下面显式放行外，一律判为丢弃 */
}

typedef struct {
    unsigned char ControlMode;
    char   FlagStop;
    float  Vx, Vy, Vz;
    float  smooth_Vx, smooth_Vy, smooth_Vz;
    float  smooth_MotorStep, smooth_ServoStep, smooth_Servo;
    u8     ServoLow_flag;
    float  rc_speed;
    float  limt_max_speed;
    uint32_t LineDiffParam;
    int    command_lostcount;   /* 真实类型为 u8（balance_task.h:ROBOT_CONTROL_t） */
    u8     SoftWare_Stop;
@@ROS_FIELD@@
} ROBOT_CONTROL_t;
static ROBOT_CONTROL_t robot_control;

typedef struct { char AllowRecharge, NavWalk, RED_STATE;
                 float Up_MoveX, Up_MoveY, Up_MoveZ;
                 float Dock_MoveX, Dock_MoveY, Dock_MoveZ;
                 float Red_MoveX, Red_MoveY, Red_MoveZ; } CHARGER_t;
static CHARGER_t charger;

static u8 Last_IP[4], Received_IP[4], IP_Frame_Valid;
static unsigned IP_Valid_Count;
#define IP_CONFIRM_COUNT 50

struct { int Time_count; int SecurityLevel; int HardWare_Ver; } SysVal;
struct { int DeepCheck; int errorflag; } robot_check;
static int EN = 1;          /* 急停开关：1 = 未按下（balance_task.c 的 EN==0 判定） */

typedef struct { unsigned char buffer[RECEIVE_DATA_SIZE]; } RECEIVE_DATA_t;
static RECEIVE_DATA_t Receive_Data;

/* Safe_Stop 与 PI 路径用到的真实全局 */
typedef struct { float Bias, LastBias; int Output; } PI_CONTROLLER;
static PI_CONTROLLER PI_MotorA, PI_MotorB, PI_MotorC, PI_MotorD;
static void PI_Controller_Reset(PI_CONTROLLER *p) {
    p->Bias = 0; p->LastBias = 0; p->Output = 0; }
typedef struct { float Target; int Output, Encoder; } MOTOR_t;
static struct { MOTOR_t MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D; float voltage; } robot;
static struct { int start_clear, clear_state; } park;
static int PWMA = 0, PWMB = 0, PWMC = 0, PWMD = 0;

/* MODEL：ResponseControl 段后的 PI 主路径用到，段外桩，不在断言路径上。 */
static void Incremental_MOTOR_stub(void) { }
typedef struct { float Integral, LastError, Kp, Ki; } WHEEL_PI_t;
static WHEEL_PI_t PI_WheelA, PI_WheelB, PI_WheelC, PI_WheelD;
'''

#: 真实 ``Set_Pwm`` 的观察替身。真实实现换算占空比后写影子变量，
#: 定时器中断再搬运到寄存器；这里只记入参，不碰硬件。
#:
#: ``g_pwm`` 等价于"最近一次 Set_Pwm 的入参"，``PWMA..D`` 等价于"寄存器现值"。
#: ``Safe_Stop`` 直写 ``PWMA..PWMD``，所以失能后 ``g_pwm`` 不变、``PWMA..D`` 归零。
_OBSERVE = r'''
static int g_pwm[5];
static int g_set_pwm_calls = 0;
static int g_set_pwm_since_estop = 0;
static uint32_t g_last_cmd_tick_seen = 0;
static void Set_Pwm(int a, int b, int c, int d, int e) {
    g_last_cmd_tick_seen = robot_control.ros_cmd_tick;
    g_pwm[0] = a; g_pwm[1] = b; g_pwm[2] = c; g_pwm[3] = d; g_pwm[4] = e;
    PWMA = a; PWMB = b; PWMC = c; PWMD = d;
    g_set_pwm_calls++;
    g_set_pwm_since_estop++;
}

/* MODEL：手柄替身（不模拟 PS2 协议），字段与真实 GamePadInterface 同名 */
typedef struct { uint8_t LY, LX, RX, LT, RT, StartFlag; } PAD_t;
static PAD_t pad;
static PAD_t *GamePadInterface = &pad;
static int g_rear_ps2_valid = 0;
static uint32_t g_rear_ps2_tick = 0;
'''

#: ``steer_debug.h`` 结构体 + ``steer_debug.c`` 四个宏，逐字。
_STEER_HDR = r'''
#define DEADZONE 20
#define MAX_DUTY 8400
#define ARM_NEUTRAL_MS 100u
#define REVERSE_ZERO_MS 20u
typedef struct { uint32_t stopped_since; int8_t direction; uint8_t running; }
        steer_side_t;
typedef struct { uint32_t neutral_since; uint8_t armed, neutral_pending;
                 steer_side_t left, right; } steer_debug_t;
typedef struct { int left, right; } steer_output_t;
#ifndef ROS_DRIVE_MAX_DUTY
#define ROS_DRIVE_MAX_DUTY 8400
#endif
/* task-15：运行时 PWM 限幅的上界与变量。值取自
   ``BALANCE/Inc/steer_debug.h`` 的 ``#if defined(ROS_STRAIGHT_DRIVE)`` 段。 */
#define ROS_DRIVE_MAX_DUTY_HARD 8400
/* task-15：限幅命令字，值取自 ``BALANCE/Inc/uartx_callback.h``。 */
#define ROS_CMD_PWM_LIMIT 0x05u
'''

#: ``Balance_task`` 一拍：逐字对应 ``balance_task.c`` 失能判定与
#: ``ResponseControl``/``Safe_Stop`` 分派（第 179-200 行）。
_TICK = r'''
static void Balance_Tick(void) {
    uint32_t tick = (uint32_t)xTaskGetTickCount();
    (void)tick;
    robot_control.FlagStop = Turn_Off();
    if (0 == robot_control.FlagStop) {
        ResponseControl();
    } else {
        Safe_Stop();
        (void)steer_debug_step(&g_steer_debug, 127, 127, 0,
                               (uint32_t)xTaskGetTickCount());
    }
}

/* 逐字对应 balance_task.c 主循环的丢帧监督段（127-129 行）。 */
static void Supervise_LostCount(void) {
    if (SysVal.Time_count > CONTROL_DELAY + 230) {
        if (0 == SysVal.SecurityLevel) {
            robot_control.command_lostcount++;
            if (robot_control.command_lostcount > BALANCE_TASK_RATE) {
                robot_control.Vx = 0;
                robot_control.Vy = 0;
                robot_control.Vz = 0;
            }
        }
    }
}
'''

_DRIVER = r'''
static void host_init(void) {
    memset(&robot_control, 0, sizeof(robot_control));
    memset(&charger, 0, sizeof(charger));
    memset(&pad, 0, sizeof(pad));
    memset(&g_steer_debug, 0, sizeof(g_steer_debug));
    memset(&robot, 0, sizeof(robot));
    memset(&Receive_Data, 0, sizeof(Receive_Data));
    pad.LY = pad.LX = pad.RX = 127;
    g_pwm[0] = g_pwm[1] = g_pwm[2] = g_pwm[3] = g_pwm[4] = -1;
    g_set_pwm_calls = 0;
    g_last_cmd_tick_seen = 0;
    g_free_tick = 2000;
    g_set_pwm_calls = 0;
    g_set_pwm_since_estop = 0;
    g_rear_ps2_valid = 0;
    g_rear_ps2_tick = 0;
    robot.voltage = 12.6f;  /* 电压满足，Turn_Off 不因欠压失能 */
    robot_check.errorflag = 0;
    EN = 1;
    robot_control.ControlMode = _ROS_Control;
    robot_control.limt_max_speed = 3.5f;
    SysVal.HardWare_Ver = V1_0;
    SysVal.Time_count = CONTROL_DELAY + 400;
    SysVal.SecurityLevel = 0;
}

static void print_state(void) {
    printf("STATE mode=%d pwm=%d,%d,%d,%d,%d lostcount=%d "
           "vx=%.6f vy=%.6f vz=%.6f ma=%d mb=%d mc=%d md=%d "
           "reg=%d,%d,%d,%d last_cmd_at_setpwm=%u now=%u "
           "setpwm_since_estop=%d\n",
           (int)robot_control.ControlMode,
           g_pwm[0], g_pwm[1], g_pwm[2], g_pwm[3], g_pwm[4],
           robot_control.command_lostcount,
           robot_control.Vx, robot_control.Vy, robot_control.Vz,
           robot.MOTOR_A.Output, robot.MOTOR_B.Output,
           robot.MOTOR_C.Output, robot.MOTOR_D.Output,
           PWMA, PWMB, PWMC, PWMD,
           g_last_cmd_tick_seen, (unsigned)xTaskGetTickCount(),
           g_set_pwm_since_estop);
    fflush(stdout);
}

int main(void) {
    char line[8192];
    host_init();
    printf("READY\n"); fflush(stdout);
    while (fgets(line, sizeof(line), stdin)) {
        char *cmd = strtok(line, " \t\r\n");
        if (!cmd) continue;
        if (strcmp(cmd, "feed") == 0) {
            char *hx = strtok(NULL, " \t\r\n");
            int n = hx ? (int)(strlen(hx) / 2) : 0;
            for (int i = 0; i < n; i++) {
                char b[3] = {hx[2 * i], hx[2 * i + 1], 0};
                unsigned v = (unsigned)strtoul(b, NULL, 16);
                if (SysVal.Time_count < CONTROL_DELAY) continue;
                UartxControll_Callback(USART3, (uint8_t)v);
            }
            printf("OK %d\n", n);
        } else if (strcmp(cmd, "tick") == 0) {
            Balance_Tick();
            printf("OK tick\n");
        } else if (strcmp(cmd, "step") == 0) {
            /* 一拍：时钟前进 + 丢帧监督 + 一次 Balance_Tick */
            char *n = strtok(NULL, " \t\r\n");
            int ms = n ? atoi(n) : 0;
            g_free_tick += (uint32_t)ms;
            SysVal.Time_count += 1;
            Supervise_LostCount();
            Balance_Tick();
            printf("OK step\n");
        } else if (strcmp(cmd, "advance") == 0) {
            char *n = strtok(NULL, " \t\r\n");
            g_free_tick += n ? (uint32_t)atoi(n) : 0;
            printf("OK ms=%u\n", (unsigned)g_free_tick);
        } else if (strcmp(cmd, "pad") == 0) {
            char *a = strtok(NULL, " \t\r\n"), *b = strtok(NULL, " \t\r\n");
            if (a) pad.LY = (uint8_t)atoi(a);
            if (b) pad.RX = (uint8_t)atoi(b);
            printf("OK pad\n");
        } else if (strcmp(cmd, "ps2") == 0) {
            char *v = strtok(NULL, " \t\r\n");
            g_rear_ps2_valid = v ? atoi(v) : 1;
            /* 记录"最后一次手柄帧"的时刻，valid 由参数决定。
               失联置 0 时不推进时间戳：真实 ps2_classic.c 亦是如此。 */
            if (g_rear_ps2_valid) g_rear_ps2_tick = g_free_tick;
            printf("OK ps2\n");
        } else if (strcmp(cmd, "set_mode") == 0) {
            char *v = strtok(NULL, " \t\r\n");
            robot_control.ControlMode = (unsigned char)(v ? atoi(v) : 0);
            printf("OK mode=%d\n", (int)robot_control.ControlMode);
        } else if (strcmp(cmd, "estop") == 0) {
            /* MODEL：急停开关 EN（balance_task.c 的 Turn_Off 读 EN==0） */
            char *v = strtok(NULL, " \t\r\n");
            EN = v ? atoi(v) : 0;
            g_set_pwm_since_estop = 0;
            printf("OK en=%d\n", EN);
        } else if (strcmp(cmd, "calls") == 0) {
            printf("OK calls=%d\n", g_set_pwm_calls);
        } else if (strcmp(cmd, "state") == 0) {
            print_state();
        } else if (strcmp(cmd, "reset_all") == 0) {
            host_init();
            printf("OK reset_all\n");
        } else if (strcmp(cmd, "quit") == 0) {
            printf("OK bye\n"); fflush(stdout); return 0;
        } else {
            printf("ERR unknown %s\n", cmd);
        }
        fflush(stdout);
    }
    return 0;
}
'''


def _ros_field() -> str:
    """从真实头文件抽出 ROS 段，保证字段名与固件一致。"""
    hdr = _load('BALANCE/Inc/balance_task.h')
    i = hdr.index('#if defined(ROS_STRAIGHT_DRIVE)')
    j = hdr.index('#endif', i) + len('#endif')
    return hdr[i:j]


def build_source() -> str:
    """拼接完整模型源码（可人工审阅）。"""
    verify_sources()
    uartx = _load('BALANCE/uartx_callback.c')
    balance = _load('BALANCE/balance_task.c')
    steer = _load('BALANCE/steer_debug.c')
    datatask = _load('BALANCE/data_task.c')

    pre = _PREAMBLE.replace('@@ROS_FIELD@@', _ros_field())
    # task-15：g_ros_drive_max_duty 的定义在 steer_debug.c 文件作用域，
    # 不在任何被截取的函数体内。排在 _STEER_HDR 之后（宏已定义），
    # 在 uartx 与 ResponseControl 之前（两处都引用它）。
    parts = [pre, _STEER_HDR, 'int g_ros_drive_max_duty = ROS_DRIVE_MAX_DUTY;',
             _OBSERVE]

    # 0) 真实 BCC 校验
    parts.append(_block(datatask, 'uint8_t Check_BCC(const uint8_t *data'))
    # 1) 真实速度解码
    parts.append(_block(uartx, 'static float XYZ_Target_Speed_transition('))
    # 2) 真实帧状态机（去 static）
    cb = _block(uartx, 'static void UartxControll_Callback(')
    parts.append(cb.replace('static void UartxControll_Callback(',
                            'void UartxControll_Callback(', 1))
    # 3) 真实四轮混合
    for anchor in ('static int axis_duty(', 'static int side_step(',
                   'steer_output_t steer_debug_step(',
                   'steer_output_t steer_debug_step_duty('):
        body = _block(steer, anchor)
        if body.startswith('static '):
            body = body[len('static '):]
        parts.append(body)

    # 4) 四轮段调用的速度→占空比换算，也在 balance_task.c 里逐字提取。
    #    它必须先于四轮段定义，否则调用点走隐式声明，返回类型对不上。
    duty = _block(balance, 'static int ros_drive_duty(')
    parts.append(duty.replace('static int ros_drive_duty(', 'int ros_drive_duty(', 1))

    # 5) 真实 ResponseControl 四轮段：整段逐字，只补成独立函数。
    seg = _response_segment(balance)
    parts.append(seg)

    # 5) 真实 Turn_Off、Safe_Stop（跳过前置声明）
    for anchor in ('static u8 Turn_Off(void)', 'static void Safe_Stop(void)'):
        body = _block(balance, anchor, skip=1 if 'Safe_Stop' in anchor else 0)
        parts.append(body.replace(anchor, anchor.replace('static ', ''), 1))

    parts.extend([_TICK, _DRIVER])
    return '\n'.join(parts)


def _response_segment(balance: str) -> str:
    """从 ``ResponseControl`` 抽出 ``REAR_WHEEL_DEBUG`` 四轮段，原样保留。"""
    start = balance.index('static void ResponseControl(void)\n{')
    seg_start = balance.index('#if defined(REAR_WHEEL_DEBUG)', start)
    seg_end = balance.index('#ifdef WHEEL_DIR_CALIB', seg_start)
    seg = balance[seg_start:seg_end]
    if seg.count('#if') != seg.count('#endif'):
        raise AssertionError('ResponseControl 四轮段预处理块不配平')
    return ('static steer_debug_t g_steer_debug;\n'
            # 固件把 ROS 与 PS2 拆成两个独立状态机（g_ros_steer_debug 与
            # g_steer_debug），两者都在文件作用域声明。这里只截取函数体，
            # 所以按固件同名同类型补一个等价声明。
            'static steer_debug_t g_ros_steer_debug;\n'
            'static void ResponseControl(void)\n{\n' + seg + '\n}\n')


def build_program() -> pathlib.Path:
    """编译模型，返回可执行文件路径（按源码哈希缓存）。"""
    text = build_source()
    digest = hashlib.sha256(text.encode()).hexdigest()[:16]
    base = pathlib.Path(tempfile.gettempdir()) / 'c50c-fw-ros-drive-model'
    base.mkdir(exist_ok=True)
    cpath = base / f'model_{digest}.c'
    exe = base / f'model_{digest}'
    if not exe.exists():
        cpath.write_text(text)
        r = subprocess.run(['gcc', '-O0', '-w', '-o', str(exe), str(cpath)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError(f'ROS 直行提取模型编译失败:\n{r.stderr[:4000]}')
    return exe


class RosDriveModel:
    """逐字提取模型的常驻子进程。结论都是"模型判定，未上板"。"""

    def __init__(self, proc: subprocess.Popen) -> None:
        self._proc = proc

    @classmethod
    def start(cls) -> 'RosDriveModel':
        proc = subprocess.Popen(
            [str(build_program())],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True, bufsize=1)
        assert proc.stdout is not None
        assert proc.stdout.readline().strip() == 'READY'
        return cls(proc)

    def _cmd(self, line: str) -> str:
        assert self._proc.stdin is not None and self._proc.stdout is not None
        self._proc.stdin.write(line + '\n')
        self._proc.stdin.flush()
        return self._proc.stdout.readline().strip()

    # ---- 输入 ----
    def feed(self, data: bytes) -> None:
        out = self._cmd('feed ' + data.hex())
        if not out.startswith('OK'):
            raise RuntimeError(f'模型拒绝 feed: {out}')

    def step(self, ms: int = 10) -> None:
        """一拍：时钟前进 ``ms``、丢帧监督、一次 Balance_Tick。"""
        out = self._cmd(f'step {int(ms)}')
        if not out.startswith('OK'):
            raise RuntimeError(f'模型拒绝 step: {out}')

    def advance_ms(self, ms: int) -> None:
        self._cmd(f'advance {int(ms)}')

    def set_pad(self, ly: int = 127, rx: int = 127) -> None:
        self._cmd(f'pad {int(ly)} {int(rx)}')

    def mark_ps2_valid(self) -> None:
        self._cmd('ps2 1')

    def drop_ps2(self) -> None:
        """手柄失联：清 valid，并把时间戳拨回不可信区。

        真实 ``ps2_classic.c:304`` 只清 ``g_rear_ps2_valid``，不碰时间戳；
        ``ready`` 用 ``&&`` 短路，valid 为假时时间戳判据不参与求值，
        两种做法在固件里等效。这里同时把时间戳拨旧，让模型对顺序不敏感。
        """
        self._cmd('ps2 0')
        self._cmd('advance 400')

    def set_control_mode(self, mask: int) -> None:
        self._cmd(f'set_mode {int(mask)}')

    def reset_all(self) -> None:
        self._cmd('reset_all')

    # ---- 观测 ----
    def set_emergency_stop(self, pressed: bool) -> None:
        """MODEL：置急停开关；按下时真实 ``Turn_Off`` 读 ``EN==0`` → 返回 1。"""
        out = self._cmd(f'estop {0 if pressed else 1}')
        if not out.startswith('OK'):
            raise RuntimeError(f'模型拒绝 estop: {out}')

    def set_pwm_calls(self) -> int:
        out = self._cmd('calls')
        return int(out.split('=', 1)[1])

    def state(self) -> dict:
        line = self._cmd('state')
        fields: dict = {}
        for tok in line.split()[1:]:
            k, _, v = tok.partition('=')
            if k in ('pwm', 'reg'):
                fields[k] = tuple(int(x) for x in v.split(','))
                continue
            try:
                fields[k] = int(v) if '.' not in v else float(v)
            except ValueError:
                fields[k] = v
        return fields

    def pwm(self) -> tuple:
        return self.state()['pwm']

    def close(self) -> None:
        try:
            self._cmd('quit')
        except Exception:
            pass
        try:
            self._proc.kill()
        finally:
            self._proc.wait(timeout=5)


__all__ = ['FIRMWARE_DIR', 'RECORDED_SHA256', 'RosDriveModel',
           'build_program', 'build_source', 'verify_sources']
