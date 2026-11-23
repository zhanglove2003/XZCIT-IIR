"""基线 ``FOUR_TURN_DEBUG`` 四轮输出路径的**接收+仲裁+输出**提取模型。

这是**模型**，不是硬件、不是固件树、不是烧录产物。每条结论都必须标注
"模型判定，未上板"。本模块只做一件事：证明"ROS 模式能否进入四轮输出
路径"这件事在基线真实函数体上的判定结果。

为什么需要它
------------
基线 ``balance_task.c:1080-1083`` 的 ``ready`` 条件是::

    int ready = SysVal.HardWare_Ver == V1_0 && GamePadInterface != NULL &&
                Get_Control_Mode(_PS2_Control) && !charger.AllowRecharge &&
                !robot_check.DeepCheck && SysVal.Time_count > CONTROL_DELAY + 230 &&
                g_rear_ps2_valid && (tick - g_rear_ps2_tick) <= pdMS_TO_TICKS(150);

``Get_Control_Mode(_PS2_Control)`` 一项在 ROS 模式下恒假（``uartx_callback.c:60-64``
收到 ``buffer[1]==0`` 的速度帧时执行 ``Set_Control_Mode(_ROS_Control)``）。
``ready`` 为假 → ``steer_debug_step(..., valid=0, ...)`` → 输出全零。
所以**未改动的基线固件**在 ROS 模式下**不会**产生四轮非零输出。

三条容易被误读的机制，模型都照实复现：

1. 四轮调试输出轴来自**手柄** ``GamePadInterface->LY/RX``（``1085-1087``），
   不是串口三轴速度。串口帧只改 ``robot_control.Vx/Vy/Vz`` 与控制模式位。
   因此"ROS 模式四轮非零"的前提是**手柄摇杆被推动**；摇杆回中时输出为 0。
2. ``ready`` 还要求 ``g_rear_ps2_valid`` 新鲜（``ps2_classic.c:178,304`` 置位）。
   没有 PS2 接收机在位时，即使模式门放开也进不了输出路径。
3. 命令丢失保护（``balance_task.c:113-121``）只把 ``Vx/Vy/Vz`` 清零，
   ``FlagStop`` 不受影响；四轮调试输出继续跟随手柄。

本模型把这几条事实做成可执行断言，并给出"若把模式门改成 ROS 模式"时的
输出映射，供固件侧核对契约。

提取范围（逐条可核对）
----------------------
从交付基线 ``baseline-src/``（``source.tar.gz`` 解包，哈希见
:data:`RECORDED_SHA256`）**逐字提取**以下函数体，只做去 ``static`` 与改名：

1. ``BALANCE/uartx_callback.c`` ``UartxControll_Callback`` —— 真实帧状态机
   （帧头/帧尾/BCC/``buffer[1]`` 命令字分派/``Set_Control_Mode``/
   ``XYZ_Target_Speed_transition`` 三轴赋值/``command_lostcount=0``）。
2. ``BALANCE/uartx_callback.c`` ``XYZ_Target_Speed_transition`` —— 真实解码。
3. ``BALANCE/steer_debug.c`` ``axis_duty`` / ``side_step`` /
   ``steer_debug_step`` —— 真实四轮混合、死区、中位 arm、换向门。
4. ``BALANCE/balance_task.c`` ``Safe_Stop`` —— 真实失能清零。
5. ``BALANCE/data_task.c`` ``Check_BCC`` —— 真实校验。

**未**提取（在模型里以显式替代物出现，全部标注 ``MODEL``）：

- ``ResponseControl()`` 的整体控制流（``balance_task.c:1070-1200``）：只保留
  ``REAR_WHEEL_DEBUG``+``FOUR_TURN_DEBUG`` 那一段的 ``ready`` 判据与输出映射
  ``Set_Pwm(-drive.right, drive.left, drive.left, -drive.right, 0)``（``1095``），
  其余分支（PI、阿克曼舵机、麦轮）不进模型。
- ``Turn_Off()``：模型用可置位的 ``FlagStop`` 输入（``turn_off``）代替
  实际电压/自检/急停读取。
- ``Set_Pwm``：记录入参并写占空比变量，不驱动定时器、不写硬件寄存器。
- ``PS2_Control()``：模型用 ``set_mode``（置控制模式位）与 ``pad``
  （直接给 ``LY``/``RX``）代替手柄协议与模式切换。
- 编码器、PI、电机、PWM 定时器：全部不建模。

模型**不**证明的部分
--------------------
- 不证明任何真机行为，也不证明板上固件版本是基线还是候选。
- 不证明主机→固件方向对齐：本模型只解析主机**发来**的字节。
- 不证明 ``g_rear_ps2_valid`` 的真实时序（模型直接置位）。
- 不证明 ``Set_Pwm`` 之后电机动没动。

用法见 ``test/test_chassis_cmd/test_ros_mode_output.py``。
"""
from __future__ import annotations

import hashlib
import pathlib
import subprocess
import tempfile

#: 交付基线解包目录（本仓库内，只读）。与 ``fw_baseline_harness`` 同一个树。
BASELINE_DIR = pathlib.Path(__file__).resolve().parents[3] / 'baseline-src'

#: 参与本模型的文件哈希。**只对自己仓库内的 baseline-src 校验**；
#: 不引用、不绑定 ``2026-11-08-50pct-migrate`` 固件工作目录的任何哈希
#: （那个目录由另一个子代理并行修改，随时会变）。
RECORDED_SHA256 = {
    'BALANCE/uartx_callback.c':
        '76183d7069fd3bc0e41b1de2e499989e0bafe6d4fd7026c447e35974918034f9',
    'BALANCE/balance_task.c':
        'e49809f9db9ba792259079d743b28e725006c5df4f9dabe9e4af45924675e91b',
    'BALANCE/steer_debug.c':
        '222cc60e794977b04b29fe78311d5732aa373511c6d05976dbed4ddad4d4e09a',
    'BALANCE/data_task.c':
        '4eb45695ecf42090c3414b3ee0308414dbbc78828b539e85195ee9460a41378b',
}


def _load(rel: str) -> str:
    return (BASELINE_DIR / rel).read_bytes().decode('gb18030', errors='replace')


def verify_sources() -> dict:
    """校验基线源哈希；只认 ``RECORDED_SHA256`` 里列出的文件。"""
    seen = {}
    for rel, want in RECORDED_SHA256.items():
        path = BASELINE_DIR / rel
        if not path.exists():
            raise AssertionError(f'缺少基线源文件 {rel}（目录 {BASELINE_DIR}）')
        got = hashlib.sha256(path.read_bytes()).hexdigest()
        if got != want:
            raise AssertionError(
                f'{rel} 哈希与记录不一致: {got!r} != {want!r}')
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


_PREAMBLE = r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_HEADER 0x7B
#define FRAME_TAIL   0x7D
#define RECEIVE_DATA_SIZE 11
#define BALANCE_TASK_RATE 100
#define CONTROL_DELAY 1000

/* 控制方式位（BALANCE/Inc/balance_task.h:84-92），与基线同名同值 */
enum {
    _ROS_Control   = (1 << 0), _PS2_Control = (1 << 1), _APP_Control = (1 << 2),
    _RC_Control    = (1 << 3), _CAN_Control = (1 << 4), _USART_Control = (1 << 5),
};
typedef void* USART_TypeDef;
static USART_TypeDef USART1 = (void*)1;
static USART_TypeDef USART3 = (void*)3;

/* 与基线 #define 同义：置为恰好 mask */
#define Set_Control_Mode(mask) (robot_control.ControlMode |= (mask), \
                                robot_control.ControlMode &= (mask))
#define Get_Control_Mode(mask) (robot_control.ControlMode & (mask))

typedef struct {
    unsigned char ControlMode;
    char   FlagStop;
    float  Vx, Vy, Vz;
    float  smooth_Vx, smooth_Vy, smooth_Vz;
    int    command_lostcount;
    float  limt_max_speed;
} ROBOT_CONTROL_t;
static ROBOT_CONTROL_t robot_control;

typedef unsigned char u8;
typedef struct { char AllowRecharge, NavWalk, RED_STATE;
                 float Up_MoveX, Up_MoveY, Up_MoveZ;
                 float Dock_MoveX, Dock_MoveY, Dock_MoveZ;
                 float Red_MoveX, Red_MoveY, Red_MoveZ; } CHARGER_t;
static CHARGER_t charger;

static u8 Last_IP[4], Received_IP[4], IP_Frame_Valid;
static unsigned IP_Valid_Count;
#define IP_CONFIRM_COUNT 50

struct { int Time_count; int SecurityLevel; int HardWare_Ver; } SysVal;
struct { int DeepCheck; } robot_check;
#define V1_0 0

typedef struct { unsigned char buffer[RECEIVE_DATA_SIZE]; } RECEIVE_DATA_t;
static RECEIVE_DATA_t Receive_Data;

/* ---- Safe_Stop 用到的基线全局（真实定义见 balance_task.c） ---- */
typedef struct { float Bias, LastBias; int Output; } PI_CONTROLLER;
static PI_CONTROLLER PI_MotorA, PI_MotorB, PI_MotorC, PI_MotorD;
static void PI_Controller_Reset(PI_CONTROLLER *p) {
    p->Bias = 0; p->LastBias = 0; p->Output = 0; }
typedef struct { float Target; int Output, Encoder; } MOTOR_t;
static struct { MOTOR_t MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D; } robot;
static struct { int start_clear, clear_state; } park;
static uint32_t g_safe_stop_calls = 0;
/* ---- MODEL：Safe_Stop 第 4 步清零的占空比寄存器（基线为 TIM 比较值） ---- */
static int PWMA = 0, PWMB = 0, PWMC = 0, PWMD = 0;
static uint32_t g_pwm_register_zeroed = 0;
'''

#: 基线 ``steer_debug.h`` 的结构体 + ``steer_debug.c`` 的四个宏。
#: 宏值逐字取自 ``steer_debug.c`` 第 4-7 行。
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
'''

#: 四轮输出的 ``Set_Pwm`` 记录。**MODEL**：只记参数并写占空比变量，
#: 不碰定时器、不写寄存器。真实 ``Set_Pwm`` 把 5 个入参换算成比较值后
#: 赋给占空比影子变量，定时器中断再搬运；这里把影子变量直接记成入参。
_OBSERVE = r'''
static int g_pwm[5];
static int g_set_pwm_calls = 0;
static void Set_Pwm(int a, int b, int c, int d, int e) {
    g_pwm[0] = a; g_pwm[1] = b; g_pwm[2] = c; g_pwm[3] = d; g_pwm[4] = e;
    PWMA = a; PWMB = b; PWMC = c; PWMD = d;
    g_set_pwm_calls++;
}

/* ---- MODEL：PS2 摇杆替身（不模拟手柄协议） ---- */
typedef struct { uint8_t LY, LX, RX, LT, RT, StartFlag; } PAD_t;
static PAD_t pad;
static PAD_t *GamePadInterface = &pad;
static int g_rear_ps2_valid = 0;
static uint32_t g_rear_ps2_tick = 0;

/* ---- MODEL：PS2_control() 只把摇杆值转成三轴目标速度的替身 ---- */
static void PS2_control(void) { }
'''

#: ``ResponseControl`` 的 ``REAR_WHEEL_DEBUG``+``FOUR_TURN_DEBUG`` 段。
#: 输出映射与 ``ready`` 判据逐字取自 ``balance_task.c:1119-1133``；
#: 唯一改动是用 ``g_mode_gate()`` 替换 ``Get_Control_Mode(_PS2_Control)``，
#: 后者是一个可配置的**模式门策略**（见 ``--mode-gate``）。
_TURNER = r'''
static steer_debug_t g_steer_debug;

/* MODEL：模式门策略。
 *   0 = 基线原样：要求 _PS2_Control（未改动的固件行为）
 *   1 = 受控 ROS：要求 _ROS_Control
 */
static int g_mode_gate = 0;
static int g_tick_ms = 0;

static int gate_ok(void) {
    if (g_mode_gate == 0) return Get_Control_Mode(_PS2_Control) != 0;
    return Get_Control_Mode(_ROS_Control) != 0;
}

/* 逐字对应 balance_task.c:1118-1133 的 ready + 四轮输出 */
static void ResponseControl_FOUR_TURN(void) {
    uint32_t tick = (uint32_t)g_tick_ms;
    uint32_t ms = g_tick_ms;
    int ready = SysVal.HardWare_Ver == V1_0 && GamePadInterface != NULL &&
                gate_ok() && !charger.AllowRecharge &&
                !robot_check.DeepCheck && SysVal.Time_count > CONTROL_DELAY + 230 &&
                g_rear_ps2_valid && (uint32_t)(tick - g_rear_ps2_tick) <= 150u;
    uint8_t forward_axis = ready ? GamePadInterface->LY : 127;
    uint8_t turn_axis = ready ? GamePadInterface->RX : 127;
    steer_output_t drive = steer_debug_step(&g_steer_debug, forward_axis,
                                            turn_axis, ready, ms);
    robot.MOTOR_A.Output = drive.right;
    robot.MOTOR_B.Output = drive.left;
    robot.MOTOR_C.Output = drive.left;
    robot.MOTOR_D.Output = -drive.right;
    robot.MOTOR_A.Target = (float)drive.right / 3000.0f;
    robot.MOTOR_B.Target = (float)drive.left / 3000.0f;
    robot.MOTOR_C.Target = robot.MOTOR_D.Target = 0;
    Set_Pwm(-drive.right, drive.left, drive.left, -drive.right, 0);
}
'''

#: ``Balance_task`` 一拍：FlagStop/Turn_Off → ResponseControl 或 Safe_Stop。
#: **MODEL**：``Turn_Off()`` 用 ``g_turn_off`` 输入代替。
_TICK = r'''
static int g_turn_off = 0;      /* MODEL：Turn_Off() 的结果（1 = 应失能） */

static void Balance_Tick(void) {
    robot_control.FlagStop = (char)g_turn_off;
    if (0 == robot_control.FlagStop) {
        ResponseControl_FOUR_TURN();
    } else {
        Safe_Stop();
        /* balance_task.c:180-186：失能分支额外以轴 127/127、valid=0 调用，
           令下一步必须重新过 100ms 中位才重新 arm。 */
        (void)steer_debug_step(&g_steer_debug, 127, 127, 0,
                               (uint32_t)g_tick_ms);
    }
}

/* MODEL：基线 SecurityLevel==0 的丢帧保护（balance_task.c 监督段） */
static void Supervise_LostCount(void) {
    if (SysVal.Time_count > CONTROL_DELAY + 230) {
        if (0 == SysVal.SecurityLevel) {
            robot_control.command_lostcount++;
            if (robot_control.command_lostcount > BALANCE_TASK_RATE)
                robot_control.Vx = 0, robot_control.Vy = 0, robot_control.Vz = 0;
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
    pad.LY = pad.LX = pad.RX = 127;           /* 摇杆回中 */
    g_pwm[0] = g_pwm[1] = g_pwm[2] = g_pwm[3] = g_pwm[4] = -1;
    g_set_pwm_calls = 0;
    g_turn_off = 0;
    g_tick_ms = 2000;
    g_rear_ps2_valid = 0;
    g_rear_ps2_tick = 0;
    robot_control.ControlMode = _ROS_Control;   /* 基线初值 1，balance_task.c:2586 */
    robot_control.limt_max_speed = 3.5f;
    SysVal.HardWare_Ver = V1_0;
    SysVal.Time_count = CONTROL_DELAY + 400;
    SysVal.SecurityLevel = 0;
}

static void print_state(const char *tag) {
    printf("%s mode=%d pwm=%d,%d,%d,%d,%d lostcount=%d "
           "vx=%.6f vy=%.6f vz=%.6f ma=%d mb=%d mc=%d md=%d "
           "reg=%d,%d,%d,%d\n",
           tag, (int)robot_control.ControlMode,
           g_pwm[0], g_pwm[1], g_pwm[2], g_pwm[3], g_pwm[4],
           robot_control.command_lostcount,
           robot_control.Vx, robot_control.Vy, robot_control.Vz,
           robot.MOTOR_A.Output, robot.MOTOR_B.Output,
           robot.MOTOR_C.Output, robot.MOTOR_D.Output,
           PWMA, PWMB, PWMC, PWMD);
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
        } else if (strcmp(cmd, "ticks") == 0) {
            char *n = strtok(NULL, " \t\r\n");
            int c = n ? atoi(n) : 0;
            for (int i = 0; i < c; i++) { SysVal.Time_count++; Supervise_LostCount(); }
            printf("OK %d\n", c);
        } else if (strcmp(cmd, "advance") == 0) {
            char *n = strtok(NULL, " \t\r\n");
            g_tick_ms += n ? atoi(n) : 0;
            printf("OK ms=%d\n", g_tick_ms);
        } else if (strcmp(cmd, "pad") == 0) {
            char *a = strtok(NULL, " \t\r\n"), *b = strtok(NULL, " \t\r\n");
            if (a) pad.LY = (uint8_t)atoi(a);
            if (b) pad.RX = (uint8_t)atoi(b);
            printf("OK pad\n");
        } else if (strcmp(cmd, "ps2") == 0) {
            char *v = strtok(NULL, " \t\r\n");
            g_rear_ps2_valid = v ? atoi(v) : 1;
            g_rear_ps2_tick = (uint32_t)g_tick_ms;
            printf("OK ps2\n");
        } else if (strcmp(cmd, "mode_gate") == 0) {
            char *v = strtok(NULL, " \t\r\n");
            g_mode_gate = v ? atoi(v) : 0;
            printf("OK gate=%d\n", g_mode_gate);
        } else if (strcmp(cmd, "set_mode") == 0) {
            /* MODEL：替代 PS2_Control() 里的 Set_Control_Mode(_PS2_Control)。
               固件真实切换点只有两处：PS2_Control() 置 PS2 位、
               UartxControll_Callback 收到 buffer[1]==0 帧置 ROS 位。 */
            char *v = strtok(NULL, " \t\r\n");
            robot_control.ControlMode = v ? atoi(v) : 0;
            printf("OK mode=%d\n", (int)robot_control.ControlMode);
        } else if (strcmp(cmd, "turn_off") == 0) {
            char *v = strtok(NULL, " \t\r\n");
            g_turn_off = v ? atoi(v) : 1;
            printf("OK turn_off=%d\n", g_turn_off);
        } else if (strcmp(cmd, "safe_stop") == 0) {
            Safe_Stop();
            printf("OK safe_stop\n");
        } else if (strcmp(cmd, "state") == 0) {
            print_state("STATE");
        } else if (strcmp(cmd, "reset_all") == 0) {
            host_init();
            memset(&Receive_Data, 0, sizeof(Receive_Data));
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


def build_program() -> pathlib.Path:
    """编译提取模型，返回可执行文件路径（按拼接源码哈希缓存）。"""
    verify_sources()
    uartx = _load('BALANCE/uartx_callback.c')
    balance = _load('BALANCE/balance_task.c')
    steer = _load('BALANCE/steer_debug.c')
    datatask = _load('BALANCE/data_task.c')

    parts = [_PREAMBLE, _STEER_HDR, _OBSERVE]

    # 0) 真实 BCC 校验（data_task.c:73，逐字）
    parts.append(_block(datatask, 'uint8_t Check_BCC(const uint8_t *data'))

    # 1) 真实速度解码
    parts.append(_block(uartx, 'static float XYZ_Target_Speed_transition('))
    # 2) 真实帧状态机（去 static）
    cb = _block(uartx, 'static void UartxControll_Callback(')
    cb = cb.replace('static void UartxControll_Callback(',
                    'void UartxControll_Callback(', 1)
    parts.append(cb)

    # 3) 真实四轮混合（三个函数逐字，去 static）
    for anchor in ('static int axis_duty(', 'static int side_step(',
                   'steer_output_t steer_debug_step('):
        body = _block(steer, anchor)
        if body.startswith('static '):
            body = body[len('static '):]
        parts.append(body)

    # 4) 真实 Safe_Stop（跳过第 40 行的前置声明）
    safe = _block(balance, 'static void Safe_Stop(void)', skip=1)
    safe = safe.replace('static void Safe_Stop(void)', 'void Safe_Stop(void)', 1)
    parts.append(safe)

    parts.extend([_TURNER, _TICK, _DRIVER])
    text = '\n'.join(parts)

    digest = hashlib.sha256(text.encode()).hexdigest()[:16]
    base = pathlib.Path(tempfile.gettempdir()) / 'c50c-fw-ros-mode-model'
    base.mkdir(exist_ok=True)
    cpath = base / f'model_{digest}.c'
    exe = base / f'model_{digest}'
    if not exe.exists():
        cpath.write_text(text)
        r = subprocess.run(['gcc', '-O0', '-w', '-o', str(exe), str(cpath), '-lm'],
                           capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError(f'ROS 模式提取模型编译失败:\n{r.stderr[:3000]}')
    return exe


class RosModeModel:
    """提取模型的常驻子进程。所有结论都是"模型判定，未上板"。"""

    def __init__(self, proc: subprocess.Popen) -> None:
        self._proc = proc

    @classmethod
    def start(cls) -> 'RosModeModel':
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

    def tick(self) -> None:
        self._cmd('tick')

    def ticks(self, n: int = 1) -> None:
        self._cmd(f'ticks {int(n)}')

    def advance_ms(self, ms: int) -> None:
        self._cmd(f'advance {int(ms)}')

    def set_pad(self, ly: int = 127, rx: int = 127) -> None:
        self._cmd(f'pad {int(ly)} {int(rx)}')

    def mark_ps2_valid(self) -> None:
        self._cmd('ps2 1')

    def set_mode_gate(self, gate: int) -> None:
        """0 = 基线原样（要求 PS2 模式）；1 = 受控 ROS 门（要求 _ROS_Control）。"""
        out = self._cmd(f'mode_gate {int(gate)}')
        if not out.startswith('OK'):
            raise RuntimeError(f'模型拒绝 mode_gate: {out}')

    def set_control_mode(self, mask: int) -> None:
        """MODEL：直接置控制模式位（PS2 模式只有手柄连接路径能置）。"""
        self._cmd(f'set_mode {int(mask)}')

    def set_turn_off(self, on: bool) -> None:
        self._cmd(f'turn_off {1 if on else 0}')

    def safe_stop(self) -> None:
        self._cmd('safe_stop')

    def reset_all(self) -> None:
        self._cmd('reset_all')

    # ---- 观测 ----
    def state(self) -> dict:
        line = self._cmd('state')
        fields: dict = {}
        for tok in line.split()[1:]:
            k, _, v = tok.partition('=')
            if k == 'pwm':
                fields['pwm'] = tuple(int(x) for x in v.split(','))
                continue
            if k == 'reg':
                fields['reg'] = tuple(int(x) for x in v.split(','))
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


__all__ = ['BASELINE_DIR', 'RECORDED_SHA256', 'RosModeModel', 'build_program',
           'verify_sources']
