"""真实固件接收代码的宿主机构建与驱动（任务 B：固件侧观测）。

目的：给桥的每一帧找一个**真实的接收端**。片段提取方式沿用已审核候选
``/home/snow/robot-task-worktrees/c50c-ctrl-timeout.4kQ9s/worktree/src/tests/
test_ros_supervision.py`` 的做法——从真实固件源码里截取函数体，在主机上
用 gcc 编译，用真实状态机与真实监督逻辑判断"这帧是否被接受"。

只读来源（不修改）：

- ``/home/snow/robot-task-worktrees/c50c-ctrl-timeout.4kQ9s/worktree/src/BALANCE/
  uartx_callback.c``  —— ``UartxControll_Callback`` 帧状态机、``XYZ_Target_
  Speed_transition`` 解码
- ``.../src/BALANCE/balance_task.c``   —— ``ROS_Command_Frame`` / ``ROS_
  Command_Supervise`` / ``Safe_Stop`` / ``ROS_Command_Disarm``
- ``.../src/BALANCE/data_task.c``       —— ``Check_BCC``

**证据边界**：这是**固件候选源码**（``643498d``，未烧录）在主机的编译结果，
不是板上行为。它证明的是"桥写出的字节在候选固件解析器里会被接受/拒绝"，
不证明板上固件已接收、不证明电机动作、不证明实际停车效果。

解析器状态的跨运行保持（重连场景的关键）：``UartxControll_Callback`` 内部
用 ``static uint16_t Count`` 保存残帧位置。**桥重启不会重置固件**，所以
桥重启后送出的第一帧可能填进上一帧留下的残帧。:class:`FirmwareModel` 用
一个常驻子进程承载该静态变量：Python 侧每次 :meth:`feed` 只追加字节，
``reboot_host()`` 只重置**主机侧**状态、不碰 ``Count``——这正是要验证的
"固件侧状态跨主机重连存活"。

字段语义（FINAL 行）：armed / age / epoch / vx / vy / vz。``epoch`` 取自
``g_ctrl_epoch``（``Safe_Stop`` 递增）；本候选固件**没有** ROS 命令受理
纪元，请勿把它当作"命令已送达"的证据。
"""
from __future__ import annotations

import pathlib
import subprocess
import tempfile
from typing import NamedTuple

FIRMWARE_TREE = pathlib.Path(
    '/home/snow/robot-task-worktrees/c50c-ctrl-timeout.4kQ9s/worktree/src')
EXPECTED_SHA256 = {
    'BALANCE/uartx_callback.c':
        '7c2c5f1e017678447336811d26861b396cad8792fe9be173d2ab40a2d4447570',
    'BALANCE/balance_task.c':
        '448bb6f7d1f60d8607b2771e73a99d8d5227ad99e1f2e6d968ef253aa671535f',
}


def _load(rel: str) -> str:
    return (FIRMWARE_TREE / rel).read_bytes().decode('gb18030', errors='replace')


def _block(text: str, anchor: str) -> str:
    i = text.index(anchor)
    j = text.index('{', i)
    depth = 0
    for k in range(j, len(text)):
        depth += (text[k] == '{') - (text[k] == '}')
        if depth == 0:
            return text[i:k + 1]
    raise ValueError(anchor)


def firmware_hashes() -> dict:
    return {rel: (FIRMWARE_TREE / rel).read_bytes() for rel in ()}


C_PREAMBLE = r'''
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define BALANCE_TASK_RATE 100
#define CONTROL_DELAY 1000
#define _ROS_Control   1
#define _PS2_Control   2
#define _APP_Control   4
#define _USART_Control 32
typedef uint8_t u8;
typedef uint16_t u16;

static int test_primask;
#define __get_PRIMASK() (test_primask)
#define __disable_irq() (test_primask = 1)
#define __set_PRIMASK(v) (test_primask = (v))

typedef struct {
  uint8_t ControlMode, FlagStop, command_lostcount;
  float Vx, Vy, Vz, smooth_Vx, smooth_Vy, smooth_Vz;
} ROBOT_CONTROL_t;
static ROBOT_CONTROL_t robot_control;
static struct { int AllowRecharge, NavWalk, RED_STATE;
                float Up_MoveX, Up_MoveY, Up_MoveZ;
                float Dock_MoveX, Dock_MoveY, Dock_MoveZ; } charger;
#define IP_CONFIRM_COUNT 5
static u8 Last_IP[4], Received_IP[4], IP_Frame_Valid;
static u16 IP_Valid_Count;
static struct { int Time_count; } SysVal_test;
uint8_t ROS_Command_Frame(float, float, float);
static int __sysval_time_count = 1000 + 231;
static int __SysVal_Time_count(void){ return __sysval_time_count; }
static uint8_t Set_Control_Mode_impl(uint8_t m){ robot_control.ControlMode = m; return m; }
#define Set_Control_Mode(m) Set_Control_Mode_impl(m)
static int Recharge_Entry_Gate_impl(int from_ros){ (void)from_ros; return 0; }
#define Recharge_Entry_Gate(x) Recharge_Entry_Gate_impl(x)
static void Recharge_Command_Frame_impl(void){ }
#define Recharge_Command_Frame() Recharge_Command_Frame_impl()
typedef struct { float Bias, LastBias; int Output; } PI_CONTROLLER;
static PI_CONTROLLER PI_MotorA, PI_MotorB, PI_MotorC, PI_MotorD;
static void PI_Controller_Reset(PI_CONTROLLER *p){ p->Bias=0;p->LastBias=0;p->Output=0; }
typedef struct { float Target; int Output; } Motor;
static struct { Motor MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D; } robot;
static int PWMA, PWMB, PWMC, PWMD;
static struct { uint8_t start_clear; uint8_t clear_state; } park;
static volatile uint32_t g_ctrl_epoch = 0;
static volatile uint32_t g_tick_epoch = 0;
static volatile uint32_t g_ros_cmd_epoch = 0;
void ROS_Command_Disarm(void);
void Safe_Stop(void);
void Recharge_Mode_Exit(void);

struct RECEIVE_DATA { unsigned char buffer[11]; };
static struct RECEIVE_DATA Receive_Data;
#define USART1 ((void*)1u)
#define USART3 ((void*)3u)
typedef void* USART_TypeDef;
#define FRAME_HEADER 0x7B
#define FRAME_TAIL   0x7D
static uint8_t Check_BCC_impl(const uint8_t *data, uint16_t length) {
    uint8_t bcc = 0; for (uint16_t i = 0; i < length; i++) bcc ^= data[i]; return bcc;
}
#define Check_BCC Check_BCC_impl
'''

C_MAIN = r'''
/* 驱动：按字节把 16 进制流送进真实状态机，然后推进真实监督函数。
   argv[1] = 十六进制字节流（逗号或空格分隔）
   argv[2] = 可选：流送完后推进的 ROS_Command_Supervise 拍数
   每字节打印一行；流结束打印 FINAL 行。
   字段：idx hex armed age epoch vx vy vz */
int unused_batch_main(int argc, char **argv) {
    test_primask = 0;
    robot_control.ControlMode = _ROS_Control;
    robot_control.FlagStop = 0;
    robot_control.command_lostcount = 0;
    memset(&Receive_Data, 0, sizeof(Receive_Data));
    if (argc < 2) { return 2; }
    const char *s = argv[1];
    int idx = 0;
    while (*s) {
        while (*s == ',' || *s == ' ') s++;
        if (!*s) break;
        char lo = s[1] ? s[1] : '0';
        char buf[3] = {s[0], lo, 0};
        unsigned v = (unsigned)strtoul(buf, NULL, 16);
        UartxControll_Callback(USART3, (uint8_t)v);
        printf("%d %02x %d %d %u %.4f %.4f %.4f\n", idx, v,
               (int)g_ros_cmd_armed, (int)g_ros_cmd_age,
               (unsigned)g_ros_cmd_epoch,
               robot_control.Vx, robot_control.Vy, robot_control.Vz);
        idx++;
        s += 2;
    }
    int ticks = (argc > 2) ? atoi(argv[2]) : 0;
    for (int t = 0; t < ticks; t++) {
        ROS_Command_Supervise();
    }
    printf("FINAL %d %d %u %.4f %.4f %.4f\n", (int)g_ros_cmd_armed,
           (int)g_ros_cmd_age, (unsigned)g_ros_cmd_epoch,
           robot_control.Vx, robot_control.Vy, robot_control.Vz);
    return 0;
}
'''


PY_API_C = r'''
/* 常驻驱动：从 stdin 读指令行，向 stdout 回 OK/结果。
   feed <hexhex...>      逐字节喂入真实帧状态机
   ticks <n>             推进 n 拍真实 ROS_Command_Supervise
   state                 打印当前状态
   reboot_host           只重置主机侧（固件静态变量 Count 保持不变）
   reset_all             重置主机侧与��件状态（仅用于新板模拟）
   quit                  退出 */
static void print_state(const char *tag) {
    printf("%s %d %d %u %.4f %.4f %.4f\n", tag, (int)g_ros_cmd_armed,
           (int)g_ros_cmd_age, (unsigned)g_ctrl_epoch,
           robot_control.Vx, robot_control.Vy, robot_control.Vz);
    fflush(stdout);
}
static void host_init(void) {
    test_primask = 0;
    robot_control.ControlMode = _ROS_Control;
    robot_control.FlagStop = 0;
    robot_control.command_lostcount = 0;
    Received_IP[0] = 0; IP_Valid_Count = 0;
}
int main(void) {
    char line[4096];
    host_init();
    printf("READY\n"); fflush(stdout);
    while (fgets(line, sizeof(line), stdin)) {
        char *cmd = strtok(line, " \t\r\n");
        if (!cmd) continue;
        if (strcmp(cmd, "feed") == 0) {
            char *hx = strtok(NULL, " \t\r\n");
            int n = hx ? (int)strlen(hx) / 2 : 0;
            for (int i = 0; i < n; i++) {
                char b[3] = {hx[2*i], hx[2*i+1], 0};
                UartxControll_Callback(USART3, (uint8_t)strtoul(b, NULL, 16));
            }
            printf("OK %d\n", n);
        } else if (strcmp(cmd, "ticks") == 0) {
            char *n = strtok(NULL, " \t\r\n");
            int c = n ? atoi(n) : 0;
            for (int i = 0; i < c; i++) ROS_Command_Supervise();
            printf("OK %d\n", c);
        } else if (strcmp(cmd, "state") == 0) {
            print_state("STATE");
        } else if (strcmp(cmd, "reboot_host") == 0) {
            host_init();
            printf("OK reboot_host\n");
        } else if (strcmp(cmd, "reset_all") == 0) {
            host_init();
            g_ros_cmd_armed = 0; g_ros_cmd_age = 0; g_ctrl_epoch = 0;
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
    """编译固件片段并返回可执行文件路径（缓存在临时目录）。"""
    import hashlib
    import os

    src = [C_PREAMBLE]
    uartx = _load('BALANCE/uartx_callback.c')
    balance = _load('BALANCE/balance_task.c')
    region = balance[balance.index('/*------------------------- ROS 命令监督'):]
    src.append(_block(uartx, 'static float XYZ_Target_Speed_transition(u8 High,u8 Low)'))
    # 真实帧状态机不做改写：它内部的 static Count（残帧位置）就是要验证的状态。
    src.append(_block(uartx, 'static void UartxControll_Callback(USART_TypeDef* USARTx,uint8_t Usart_Receive)'))
    src.append(_block(balance, 'void Safe_Stop(void)\n{'))
    src.append(region[:region.index('uint8_t ROS_Command_Frame(float')])
    # 自检门控依赖 SysVal.Time_count（主循环里更新的运行状态）；用固定值
    # 1000+231（=CONTROL_DELAY+231，越过自检期）替换，只影响该门控。
    frame_fn = _block(region, 'uint8_t ROS_Command_Frame(float').replace(
        'SysVal.Time_count', '__SysVal_Time_count()')
    src.append(frame_fn)
    src.append(_block(region, 'void ROS_Command_Disarm(void)'))
    src.append(_block(region, 'static void ROS_Command_Supervise(void)'))
    src.append(_block(balance, 'void Recharge_Mode_Exit(void)'))
    src.append(C_MAIN)
    src.append(PY_API_C)
    text = '\n'.join(src)
    text = text.replace('static void UartxControll_Callback',
                        'void UartxControll_Callback')
    # 发射器：让 Write_Bytes 在固件侧打印收到的字节（真实 UartxControll_Callback
    # 里已有发送日志调用）。用于验证"桥写出的字节 == 固件收到的字节"。
    text = text.replace('__WIRE_SINK__', '')
    digest = hashlib.sha256(text.encode()).hexdigest()[:16]
    base = pathlib.Path(tempfile.gettempdir()) / f'c50c-fw-parser-{os.getpid()}'
    base.mkdir(exist_ok=True)
    c = base / f'fw_{digest}.c'
    exe = base / f'fw_{digest}'
    if not exe.exists():
        c.write_text(text)
        subprocess.run(['gcc', '-O0', '-o', str(exe), str(c), '-lm'],
                       check=True, capture_output=True)
    return exe


class FirmwareModel:
    """常驻固件解析器：真实状态机 + 真实监督，跑在子进程里。

    用法::

        m = FirmwareModel.start()
        m.feed(frame)          # 一个完整 11 字节速度帧
        m.feed_bytes(b'ab')    # 任意字节（用于构造残帧）
        m.tick(100)            # 推进 100 拍 ROS_Command_Supervise
        m.state                # -> State(armed, age, epoch, vx, vy, vz)
        m.reboot_host()        # 只重置主机侧；固件静态 Count 保留
        m.close()

    ``feed_bytes`` 是唯一原语；``feed`` 只是它的封装。固件**不知道**帧边界，
    接到什么字节就解析什么字节——这正是重连/残帧场景要验证的性质。
    """

    def __init__(self, proc: subprocess.Popen) -> None:
        self._proc = proc

    # ------------------------------------------------------------ 生命周期
    @classmethod
    def start(cls) -> 'FirmwareModel':
        exe = build_program()
        proc = subprocess.Popen([str(exe)], stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, text=True, bufsize=1)
        ready = proc.stdout.readline().strip()
        if ready != 'READY':
            proc.kill()
            raise RuntimeError(f'固件模型启动失败: {ready!r}')
        return cls(proc)

    def _cmd(self, line: str) -> list[str]:
        assert self._proc.stdin is not None and self._proc.stdout is not None
        self._proc.stdin.write(line + '\n')
        self._proc.stdin.flush()
        out = self._proc.stdout.readline().strip().split()
        if not out or out[0] == 'ERR':
            raise RuntimeError(f'固件模型拒绝命令 {line!r}: {out}')
        return out

    def close(self) -> None:
        try:
            self._cmd('quit')
        except Exception:   # noqa: BLE001
            pass
        self._proc.kill()
        self._proc.wait(timeout=5)

    # ------------------------------------------------------------ 数据面
    def feed_bytes(self, data: bytes) -> int:
        """把任意字节原样送进真实帧状态机，返回喂入字节数。"""
        if data:
            self._cmd('feed ' + data.hex())
        return len(data)

    def feed(self, frame: bytes) -> int:
        if len(frame) != 11:
            raise ValueError(f'速度帧必须是 11 字节，收到 {len(frame)}')
        return self.feed_bytes(frame)

    def tick(self, n: int = 1) -> None:
        """推进 n 拍监督（每拍对应 BALANCE_TASK_RATE=100 Hz）。"""
        if n > 0:
            self._cmd(f'ticks {n}')

    # ------------------------------------------------------------ 观测面
    @property
    def state(self) -> 'State':
        out = self._cmd('state')
        # out = ['STATE', armed, age, epoch, vx, vy, vz]
        return State(armed=int(out[1]) == 1, age=int(out[2]),
                     epoch=int(out[3]),
                     vx=float(out[4]), vy=float(out[5]), vz=float(out[6]))

    def reboot_host(self) -> None:
        """桥重启：只重置主机侧状态，固件静态变量（含残帧计数）保留。"""
        self._cmd('reboot_host')

    def reset_all(self) -> None:
        """模拟新板：主机侧与固件状态全清。"""
        self._cmd('reset_all')


class State(NamedTuple):
    armed: bool
    age: int
    epoch: int
    vx: float
    vy: float
    vz: float
