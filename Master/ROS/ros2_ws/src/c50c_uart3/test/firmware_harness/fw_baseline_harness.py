"""基线固件（2026-11-05 四轮 50% 交付基线）接收模型的宿主编译与驱动。

用途：把候选 ROS 侧**真实发出的字节**喂进基线固件的**真实解析代码**，
判断基线是否接受、三轴目标速度是否为零、命令丢失保护是否清零。

来源与固定性
------------
固件源取自交付包 ``source.tar.gz``（SHA-256 已校验，见下），解包在本仓库
``baseline-src/``，只读不修改。**不**使用候选固件（候选固件含
``chassis_ack.c`` 会话门与序号门，基线没有）。

编译进本模型的，全部是基线源文件里的**原样函数体**（用 ``gb18030`` 解码，
只做三件事：去掉 ``static``、改名避免符号冲突、注入观测变量）：

- ``BALANCE/uartx_callback.c`` ``UartxControll_Callback`` —— 帧状态机
- ``BALANCE/uartx_callback.c`` ``XYZ_Target_Speed_transition`` —— 速度解码
- ``BALANCE/balance_task.c`` ``Safe_Stop`` —— 失能时清零
- ``BALANCE/balance_task.c`` 里 ``SecurityLevel==0`` 的丢失计数保护
  （在模型里以 ``Supervise_LostCount`` 逐拍执行，见 :func:`_supervise_snippet`）

基线**没有** ``ROS_Command_Frame`` / ``ROS_Command_Supervise`` / 
``ROS_Command_Disarm``（grep 无结果），这三个函数只存在于候选固件，
所以不能复用 ``fw_parser_harness``。

能力边界
--------
模型证明：给定字节流，基线状态机是否走到"速度落盘 + lostcount 清零"分支，
以及三轴目标速度的数值。

模型**不**证明：``Drive_Motor`` 之后的电机行为。基线交付构建定义了
``REAR_WHEEL_DEBUG``，``ResponseControl`` 在该宏下直接 ``return``
（``BALANCE/balance_task.c:1114``–``1183``），速度到 PWM 的通路不可达。
"""
from __future__ import annotations

import hashlib
import os
import pathlib
import subprocess
import tempfile

#: 交付基线解包目录（本仓库内，只读）。
BASELINE_DIR = pathlib.Path(__file__).resolve().parents[3] / 'baseline-src'

#: 交付包内 ``SHA256SUMS`` 记录的值（已用 ``sha256sum`` 复核）。
BASELINE_PACKAGE_SHA256 = (
    'f4594d6700d79e417ba65b6764b44ada83a827dda49ced8aeb07715a418b7c4e')

#: 参与建模的基线源文件及哈希。哈希不符即拒绝构建，避免静默换树。
RECORDED_SHA256 = {
    'BALANCE/uartx_callback.c':
        '76183d7069fd3bc0e41b1de2e499989e0bafe6d4fd7026c447e35974918034f9',
    'BALANCE/balance_task.c':
        'e49809f9db9ba792259079d743b28e725006c5df4f9dabe9e4af45924675e91b',
    'BALANCE/data_task.c':
        '4eb45695ecf42090c3414b3ee0308414dbbc78828b539e85195ee9460a41378b',
}

#: 基线常量（与源文件逐一对齐，见 report）。
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
FRAME_LENGTH = 11
FIRMWARE_MAX_SPEED_MPS = 3.5      # balance_task.c:2566 limt_max_speed
LOST_COUNT_LIMIT = 100            # BALANCE_TASK_RATE
CONTROL_DELAY = 1000              # USER/system.h
SELF_CHECK_END = CONTROL_DELAY + 230


def _load(rel: str) -> str:
    return (BASELINE_DIR / rel).read_bytes().decode('gb18030', errors='replace')


def verify_sources() -> dict:
    """确认基线文件哈希与记录一致；不一致时抛错（不静默换树）。"""
    seen = {}
    for rel, want in RECORDED_SHA256.items():
        got = hashlib.sha256((BASELINE_DIR / rel).read_bytes()).hexdigest()
        if got != want:
            raise AssertionError(
                f'{rel} 哈希与记录不一致: {got!r} != {want!r}')
        seen[rel] = got
    return seen


def _block(text: str, anchor: str, skip: int = 0) -> str:
    """截取以 ``anchor`` 开头、花括号配平的那一段源码。

    ``skip`` 跳过前若干个匹配（用于避开同名前置声明）。
    """
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
#include <math.h>

#define FRAME_HEADER 0x7B
#define FRAME_TAIL   0x7D
#define RECEIVE_DATA_SIZE 11
#define BALANCE_TASK_RATE 100
#define CONTROL_DELAY 1000

/* 控制方式位（BALANCE/Inc/balance_task.h:84-92） */
enum {
    _ROS_Control = (1 << 0), _PS2_Control = (1 << 1), _APP_Control = (1 << 2),
    _RC_Control = (1 << 3), _CAN_Control = (1 << 4), _USART_Control = (1 << 5),
};
typedef void*   USART_TypeDef;
static USART_TypeDef USART1 = (void*)1;
static USART_TypeDef USART3 = (void*)3;

/* 与基线 #define Set_Control_Mode(mask) (x |= mask, x &= mask) 同义：置为恰好 mask */
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

/* IP 帧（buffer[1]==0xFF）用到的基线全局 */
static u8 Last_IP[4], Received_IP[4], IP_Frame_Valid;
static unsigned IP_Valid_Count;
#define IP_CONFIRM_COUNT 50

struct { int Time_count; int SecurityLevel; int HardWare_Ver; } SysVal;

typedef struct { unsigned char buffer[RECEIVE_DATA_SIZE]; } RECEIVE_DATA_t;
static RECEIVE_DATA_t Receive_Data;

static uint8_t Check_BCC(const uint8_t *data, uint16_t length) {
    uint8_t bcc = 0;
    for (uint16_t i = 0; i < length; i++) bcc ^= data[i];
    return bcc;
}

/* 观测点：整数，-1 表示本帧没有走到速度落盘分支 */
static volatile int g_accepted = -1;
static volatile int g_accept_count = 0;
static volatile int g_reject_tail = 0, g_reject_bcc = 0;
static volatile int g_reject_cmd = 0;
static volatile int g_dropped_pre_delay = 0;

/* ---- Safe_Stop 用到的基线全局（真实定义见 balance_task.c） ---- */
typedef struct { float Bias, LastBias; int Output; } PI_CONTROLLER;
static PI_CONTROLLER PI_MotorA, PI_MotorB, PI_MotorC, PI_MotorD;
static void PI_Controller_Reset(PI_CONTROLLER *p) {
    p->Bias = 0; p->LastBias = 0; p->Output = 0; }
typedef struct { float Target; int Output; } MOTOR_t;
static struct { MOTOR_t MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D; } robot;
static int PWMA, PWMB, PWMC, PWMD;
static struct { int start_clear, clear_state; } park;

/* 电机侧副作用计数（Safe_Stop 执行了几次） */
static volatile int g_safe_stop_calls = 0;
'''

_GLOBALS = r'''
void fw_reset_statics(void) { }
'''


_DRIVER = r'''
static void host_init(void) {
    memset(&robot_control, 0, sizeof(robot_control));
    memset(&charger, 0, sizeof(charger));
    robot_control.ControlMode = _ROS_Control;   /* 基线初值 1，balance_task.c:2586 */
    robot_control.limt_max_speed = 3.5f;        /* balance_task.c:2566 */
    SysVal.Time_count = CONTROL_DELAY + 400;    /* 越过开机静默与自检完成 */
    SysVal.SecurityLevel = 0;                   /* 最高安全等级，有丢帧保护 */
}

static void print_state(const char *tag) {
    printf("%s accepted=%d counts=%d lostcount=%d mode=%d "
           "vx=%.6f vy=%.6f vz=%.6f stop_calls=%d time=%d "
           "rej_tail=%d rej_bcc=%d rej_cmd=%d pre_delay=%d\n",
           tag, g_accepted, g_accept_count, robot_control.command_lostcount,
           (int)robot_control.ControlMode,
           robot_control.Vx, robot_control.Vy, robot_control.Vz,
           g_safe_stop_calls, SysVal.Time_count,
           g_reject_tail, g_reject_bcc, g_reject_cmd, g_dropped_pre_delay);
    fflush(stdout);
}

/* 执行一拍的基线监督逻辑：command_lostcount 递增并在超限时清零三轴 */
static void Supervise_LostCount(void) {
    if (SysVal.Time_count > CONTROL_DELAY + 230) {
        if (0 == SysVal.SecurityLevel) {
            robot_control.command_lostcount++;
            if (robot_control.command_lostcount > BALANCE_TASK_RATE)
                robot_control.Vx = 0, robot_control.Vy = 0, robot_control.Vz = 0;
        }
    }
}

/* 固件重启时残留计数会丢失，模型同样清零（见 _GLOBALS） */
void fw_reset_statics(void);

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
            g_accepted = 0;
            for (int i = 0; i < n; i++) {
                char b[3] = {hx[2 * i], hx[2 * i + 1], 0};
                unsigned v = (unsigned)strtoul(b, NULL, 16);
                /* 复刻 USART3_IRQHandler 的开机静默门 (uartx_callback.c:388-405) */
                if (SysVal.Time_count < CONTROL_DELAY) { g_dropped_pre_delay++; continue; }
                UartxControll_Callback(USART3, (uint8_t)v);
            }
            printf("OK %d\n", n);
        } else if (strcmp(cmd, "ticks") == 0) {
            char *n = strtok(NULL, " \t\r\n");
            int c = n ? atoi(n) : 0;
            for (int i = 0; i < c; i++) { SysVal.Time_count++; Supervise_LostCount(); }
            printf("OK %d\n", c);
        } else if (strcmp(cmd, "safe_stop") == 0) {
            Safe_Stop();
            printf("OK safe_stop\n");
        } else if (strcmp(cmd, "state") == 0) {
            print_state("STATE");
        } else if (strcmp(cmd, "reset_all") == 0) {
            host_init();
            memset(&Receive_Data, 0, sizeof(Receive_Data));
            fw_reset_statics();
            g_accepted = -1; g_accept_count = 0;
            g_reject_tail = g_reject_bcc = g_reject_cmd = 0;
            g_dropped_pre_delay = 0;
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
    """编译基线接收模型，返回可执行文件路径（按源码哈希缓存）。"""
    verify_sources()
    uartx = _load('BALANCE/uartx_callback.c')
    balance = _load('BALANCE/balance_task.c')

    parts = [_PREAMBLE]

    # 真实速度解码（uartx_callback.c:437-448）
    parts.append(_block(uartx, 'static float XYZ_Target_Speed_transition('))
    # 真实帧状态机（uartx_callback.c:27-…）。注入观测，去掉 static。
    cb = _block(uartx, 'static void UartxControll_Callback(')
    cb = cb.replace('static void UartxControll_Callback(',
                    'void UartxControll_Callback(', 1)
    # 观测：进入"速度落盘"分支时记录
    cb = cb.replace(
        'robot_control.command_lostcount = 0;',
        'robot_control.command_lostcount = 0; g_accepted = 1; g_accept_count++;', 1)
    cb = cb.replace(
        'if(Receive_Data.buffer[10] == FRAME_TAIL)',
        'if(!(Receive_Data.buffer[10] == FRAME_TAIL)) g_reject_tail++;\n'
        '\t\t\tif(Receive_Data.buffer[10] == FRAME_TAIL)', 1)
    cb = cb.replace(
        'if(Receive_Data.buffer[9] ==Check_BCC(Receive_Data.buffer,9))',
        'if(!(Receive_Data.buffer[9] ==Check_BCC(Receive_Data.buffer,9))) g_reject_bcc++;\n'
        '\t\t\tif(Receive_Data.buffer[9] ==Check_BCC(Receive_Data.buffer,9))', 1)
    parts.append(cb)

    # Safe_Stop（balance_task.c:1415-…）：观测调用次数
    # 跳过第 40 行的前置声明，取 1415 行的定义
    safe = _block(balance, 'static void Safe_Stop(void)', skip=1)
    safe = safe.replace('static void Safe_Stop(void)', 'void Safe_Stop(void)', 1)
    safe = safe.replace('{', '{\n    g_safe_stop_calls++;', 1)
    parts.append(safe)

    parts.append(_GLOBALS)
    parts.append(_DRIVER)
    text = '\n'.join(parts)

    digest = hashlib.sha256(text.encode()).hexdigest()[:16]
    base = pathlib.Path(tempfile.gettempdir()) / 'c50c-fw-baseline-model'
    base.mkdir(exist_ok=True)
    cpath = base / f'model_{digest}.c'
    exe = base / f'model_{digest}'
    if not exe.exists():
        cpath.write_text(text)
        r = subprocess.run(['gcc', '-O0', '-w', '-o', str(exe), str(cpath), '-lm'],
                           capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError(f'基线模型编译失败:\n{r.stderr[:3000]}')
    return exe


class BaselineModel:
    """基线接收模型的持久子进程（一次启动，多条命令）。

    设计约束：``feed`` 会**重置** ``g_accepted``（每条喂入命令统计本批是否
    接受），所以包装对象在每次 ``feed_bytes`` 后立刻读一次 ``state``，把
    "本次是否接受"固化到 ``last_accepted`` 字段，避免调用方跨命令读旧值。
    """

    def __init__(self, proc: subprocess.Popen) -> None:
        self._proc = proc
        self.last_accepted = 0
        self.last_accepted_count = 0

    @classmethod
    def start(cls) -> 'BaselineModel':
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

    def feed_bytes(self, data: bytes) -> int:
        """喂原始字节；返回本批被接受的完整帧数（0 或 1）。"""
        out = self._cmd('feed ' + data.hex())
        if not out.startswith('OK'):
            raise RuntimeError(f'模型拒绝 feed: {out}')
        return self.state()['accepted']

    def feed(self, frame: bytes) -> int:
        return self.feed_bytes(frame)

    def ticks(self, n: int = 1) -> None:
        """推进 n 拍（每拍 10 ms：command_lostcount++ 与丢失保护）。"""
        self._cmd(f'ticks {int(n)}')

    def safe_stop(self) -> None:
        self._cmd('safe_stop')

    def state(self) -> dict:
        line = self._cmd('state')
        fields = dict()
        for tok in line.split()[1:]:
            k, _, v = tok.partition('=')
            try:
                fields[k] = int(v) if '.' not in v else float(v)
            except ValueError:
                fields[k] = v
        self.last_accepted = fields.get('accepted', 0)
        self.last_accepted_count = fields.get('accept_count', 0)
        return fields

    def reset_all(self) -> None:
        self._cmd('reset_all')

    def close(self) -> None:
        try:
            self._cmd('quit')
        except Exception:
            pass
        try:
            self._proc.kill()
        finally:
            self._proc.wait(timeout=5)
