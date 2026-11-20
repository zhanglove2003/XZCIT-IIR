"""A 侧接收确认门（chassis_ack.c、data_task.c 候选）的宿主机构建与驱动。

与 :mod:`fw_parser_harness` 的区别
----------------------------------
:mod:`fw_parser_harness` 用的是**旧基线**（``c50c-ctrl-timeout.4kQ9s``），
不含 A 本轮新增的 ``chassis_ack.c``：它的 ``UartxControll_Callback`` 直接
把帧交给 ``ROS_Command_Frame``。本模块用 A 的**上机前候选**源码
（``c50c-fw-preboard.a1p2``），把三处真实代码原样提取进来：

1. ``Chassis_Ack_CheckFrame`` 及其全部依赖（``chassis_ack.c`` 全文）；
2. ``UartxControll_Callback`` 里的 USART3 门控：``ack != CHASSIS_RX_ACCEPT``
   即 ``return``（帧被丢弃，不进入速度处理）；
3. ``data_task.c`` 的会话帧发送节拍（每 ``DATA_TASK_RATE`` 拍一帧 0x7F，
   经 ``uart3_send`` 发出）与遥测逐字节发送（``Chassis_Ack_NoteTxByte``）。

**提取方式是文本提取 + 补齐桩**，不改写被测逻辑。与 A 自己的测试一致地
把 ``CHASSIS_SESSION_ENTROPY_ADDR`` 指回本地缓冲区（板上地址不可读）。

证据边界
--------
这是**固件候选源码**（``7ac8aef``，未烧录）的宿主机构建。它证明"主机写出的
字节在 A 的确认门里会被接受还是丢弃"，不证明板上行为、不证明电机动作。

``s_session`` 由熵源决定，每次构建都不同，所以**主机必须先从线上读会话帧**。
这就是本候选要验证的握手前提。
"""
from __future__ import annotations

import hashlib
import pathlib
import subprocess
import tempfile
from typing import NamedTuple

FIRMWARE_TREE = pathlib.Path(
    '/home/snow/robot-task-worktrees/c50c-fw-preboard.a1p2/src')
EXPECTED_SHA256 = {
    'BALANCE/chassis_ack.c':
        '0ffa27350e77d8a4a528cab9b374bd76e95184709109392b7d434498deb9cd65',
    'BALANCE/Inc/chassis_ack.h':
        '84f29dfac09d1b327f4f000fe61dc2e4180e9fc7090afe93c137480c90ae4381',
    'BALANCE/uartx_callback.c':
        'f08c57279f6c0d1d5c8e65de081a0d894eab05ac6abf6181fa795742cb87fe9a',
    'BALANCE/data_task.c':
        'f21e740db488e4b1de9333cb63ae46342e96273325f52f05e95e81bd9eeb4cf7',
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
    return {rel: hashlib.sha256((FIRMWARE_TREE / rel).read_bytes()).hexdigest()
            for rel in EXPECTED_SHA256}


def verify_firmware_hashes() -> None:
    """源码与记录不一致就失败：证据必须可追溯到具体字节。"""
    bad = {rel: (EXPECTED_SHA256[rel], digest)
           for rel, digest in firmware_hashes().items()
           if digest != EXPECTED_SHA256[rel]}
    if bad:
        raise RuntimeError(f'A 候选源码已变更，证据失效: {bad}')


C_PREAMBLE = r'''
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define BALANCE_TASK_RATE 100
#define DATA_TASK_RATE 20
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

/* ---- 熵源桩：把板上 SRAM 地址换成本地缓冲区（与 A 自己的测试一致） ---- */
static uint8_t entropy_stub[64];

/* 复位时把熵源填成固定但非常量的值：避免编译器把整个会话计算折叠成常数。
   板上这份 SRAM 上电内容不确定；这里固定值只为让宿主机构建可复现。 */
static void entropy_fill(void)
{
  uint32_t x = 0x2545F491u;
  for (uint32_t i = 0; i < sizeof(entropy_stub); i++)
  {
    x = x * 1664525u + 1013904223u;
    entropy_stub[i] = (uint8_t)(x >> 24);
  }
  entropy_stub[0] |= 0x01u;
}

typedef struct { uint32_t VAL; } SYSTICK_t;
static SYSTICK_t SysTick_stub = { 0x00012345u };
#define SysTick (&SysTick_stub)

typedef struct {
  uint8_t ControlMode, FlagStop, command_lostcount;
  float Vx, Vy, Vz, smooth_Vx, smooth_Vy, smooth_Vz;
} ROBOT_CONTROL_t;
static ROBOT_CONTROL_t robot_control;
static struct { int HardWare_Ranger; } SysVal;
static uint8_t Send_Data_buffer[24];

#define FRAME_HEADER 0x7B
#define FRAME_TAIL   0x7D
static uint8_t Check_BCC_impl(const uint8_t *data, uint16_t length) {
    uint8_t bcc = 0; for (uint16_t i = 0; i < length; i++) bcc ^= data[i]; return bcc;
}
#define Check_BCC Check_BCC_impl

struct RECEIVE_DATA { unsigned char buffer[11]; };
static struct RECEIVE_DATA Receive_Data;
#define USART1 ((void*)1u)
#define USART3 ((void*)3u)
typedef void* USART_TypeDef;

/* ---- 回调里用到的外设/业务桩（与 fw_parser_harness 一致，只替换副作用） ---- */
static struct { int AllowRecharge, NavWalk, RED_STATE;
                float Up_MoveX, Up_MoveY, Up_MoveZ;
                float Dock_MoveX, Dock_MoveY, Dock_MoveZ; } charger;
#define IP_CONFIRM_COUNT 5
static u8 Last_IP[4], Received_IP[4], IP_Frame_Valid;
static u16 IP_Valid_Count;
static int __sysval_time_count = 1000 + 231;
static int __SysVal_Time_count(void){ return __sysval_time_count; }
static uint8_t Set_Control_Mode_impl(uint8_t m){ robot_control.ControlMode = m; return m; }
#define Set_Control_Mode(m) Set_Control_Mode_impl(m)
static int Recharge_Entry_Gate_impl(int from_ros){ (void)from_ros; return 0; }
#define Recharge_Entry_Gate(x) Recharge_Entry_Gate_impl(x)
static void Recharge_Command_Frame_impl(void){ }
#define Recharge_Command_Frame() Recharge_Command_Frame_impl()
/* ---- 电机与 PI 桩：回调里只写这些字段，不产生任何输出 ---- */
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
void Recharge_Mode_Exit(void);
void ROS_Command_Disarm(void);
void Safe_Stop(void);

uint8_t ROS_Command_Frame(float, float, float);
'''


C_CHASSIS_ACK = r'''
/* ---- chassis_ack.h 的宏定义（与头文件一致） ---- */
#define CHASSIS_PROTO_VERSION   2
#define CHASSIS_FRAME_LEN       11
#define CHASSIS_IDX_TYPE        1
#define CHASSIS_IDX_SUB         2
#define CHASSIS_IDX_DATA        3
#define CHASSIS_IDX_CHECK       9
#define CHASSIS_IDX_TAIL        10
#define CHASSIS_TYPE_SPEED      0x00
#define CHASSIS_TYPE_RECHARGE   0x01
#define CHASSIS_TYPE_RECHARGE2  0x02
#define CHASSIS_TYPE_DOCK       0x03
#define CHASSIS_TYPE_SESSION    0x7F
#define CHASSIS_TYPE_IP         0xFF
#define CHASSIS_SESSION_SUB     0x01
#define CHASSIS_SESSION_SIZE    11
#define CHASSIS_CAP_HOST_ACK    0x80
#define CHASSIS_SESSION_ENTROPY_ADDR  (&entropy_stub[0])
#define CHASSIS_SESSION_ENTROPY_LEN   64U
#define CHASSIS_SESSION_ENTROPY_STRIDE 1U

typedef enum {
  CHASSIS_RX_ACCEPT      = 0,
  CHASSIS_RX_NOT_SESSION = 1,
  CHASSIS_RX_NO_SESSION  = 2,
  CHASSIS_RX_STALE       = 3,
  CHASSIS_RX_ECHO        = 4,
  CHASSIS_RX_BAD_TYPE    = 5,
  CHASSIS_RX_BAD_FRAME   = 6,
} ChassisRxResult;

ChassisRxResult Chassis_Ack_CheckFrame(const uint8_t *f);
void Chassis_Ack_NoteTxByte(uint8_t byte);
uint32_t Chassis_Ack_Session(void);
void Chassis_Ack_Stats(uint32_t*, uint32_t*, uint32_t*, uint32_t*, uint32_t*);
void Chassis_Ack_ResetSession(void);
void Chassis_Ack_EncodeSessionFrame(uint8_t out[CHASSIS_SESSION_SIZE]);
void Chassis_Ack_Init(void);
'''

PY_API_C = r'''
/* 常驻驱动。
   feed <hex>        把字节逐个送进真实 UartxControll_Callback(USART3, b)
   tick <n>          推进 n 拍 data_task 的发送节拍（产生会话帧与遥测）
   state             打印状态
   wire              打印本端从 USART3 发出的全部字节（十六进制）
   txcount           打印本端已发出的字节数
   quit              退出 */
/* 会话标识在固件里是私有的，固件也不提供读取口。这份构建额外导出原始值，
   只为让主机测试能从"固件自报"开始握手——它不改变任何门控逻辑。 */
uint32_t g_test_session_id(void);

static unsigned char g_wire[65536];
static int g_wire_len = 0;
static int g_session_tick = 0;

static void uart3_send(uint8_t b)
{
  if (g_wire_len < (int)sizeof(g_wire)) g_wire[g_wire_len++] = b;
  printf("TX %02x\n", b);
}

/* data_task.c 的会话帧节拍（原样提取自 SendTask，变量改为文件作用域） */
static void data_task_session_tick(void)
{
  if( ++g_session_tick >= (DATA_TASK_RATE) )
  {
    uint8_t sf[CHASSIS_SESSION_SIZE];
    uint8_t k;
    g_session_tick = 0;
    Chassis_Ack_EncodeSessionFrame(sf);
    for( k=0; k<CHASSIS_SESSION_SIZE; k++ ) uart3_send(sf[k]);
  }
}

/* data_task.c 的 Usart3_SendTask 逐字节部分（24 字节遥测） */
static void usart3_send_telemetry(void)
{
  unsigned char i = 0;
  for(i=0; i<24; i++)
  {
    Chassis_Ack_NoteTxByte(Send_Data_buffer[i]);
    uart3_send(Send_Data_buffer[i]);
  }
}

static void print_state(const char *tag)
{
  uint32_t a=0,s=0,e=0,n=0,bt=0;
  Chassis_Ack_Stats(&a,&s,&e,&n,&bt);
  printf("%s %d %d %u %.4f %.4f %.4f accept=%u stale=%u echo=%u no_session=%u bad_type=%u\n",
         tag, (int)g_ros_cmd_armed, (int)g_ros_cmd_age, (unsigned)g_ctrl_epoch,
         robot_control.Vx, robot_control.Vy, robot_control.Vz,
         a, s, e, n, bt);
  fflush(stdout);
}

int main(void)
{
  char line[65536];
  test_primask = 0;
  robot_control.ControlMode = _ROS_Control;
  robot_control.FlagStop = 0;
  robot_control.command_lostcount = 0;
  SysVal.HardWare_Ranger = 0;
  /* 遥测形状：0x7B 起、23 字节处为 0x7D（值固定，便于回显判据） */
  memset(Send_Data_buffer, 0, sizeof(Send_Data_buffer));
  Send_Data_buffer[0] = 0x7B;
  Send_Data_buffer[1] = 0x00;
  for (int i = 2; i < 22; i++) Send_Data_buffer[i] = (uint8_t)(0x40 + i);
  Send_Data_buffer[22] = Check_BCC(Send_Data_buffer, 22);
  Send_Data_buffer[23] = 0x7D;
  entropy_fill();
  Chassis_Ack_Init();
  /* 会话标识在固件里是私有的，固件也不提供读取口。固件不导出它——
     板上的主机同样拿不到，必须先读一帧线上会话帧。这里把原始值打到
     stderr，只为让测试能断言"主机采纳的 ID == 固件会话 ID"。 */
  fprintf(stderr, "SESSION_ID %08x\n", (unsigned)g_test_session_id());
  fflush(stderr);
  printf("READY\n");
  fflush(stdout);
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
    } else if (strcmp(cmd, "feedother") == 0) {
      char *hx = strtok(NULL, " \t\r\n");
      int n = hx ? (int)strlen(hx) / 2 : 0;
      for (int i = 0; i < n; i++) {
        char b[3] = {hx[2*i], hx[2*i+1], 0};
        UartxControll_Callback(USART1, (uint8_t)strtoul(b, NULL, 16));
      }
      printf("OK %d\n", n);
    } else if (strcmp(cmd, "tick") == 0) {
      char *n = strtok(NULL, " \t\r\n");
      int c = n ? atoi(n) : 0;
      for (int i = 0; i < c; i++) {
        /* data_task 一拍：先发遥测，再按节拍发会话帧，再推进监督 */
        usart3_send_telemetry();
        data_task_session_tick();
        ROS_Command_Supervise();
      }
      printf("OK %d\n", c);
    } else if (strcmp(cmd, "state") == 0) {
      print_state("STATE");
    } else if (strcmp(cmd, "wire") == 0) {
      printf("WIRE ");
      for (int i = 0; i < g_wire_len; i++) printf("%02x", g_wire[i]);
      printf("\n");
    } else if (strcmp(cmd, "txcount") == 0) {
      printf("TXCOUNT %d\n", g_wire_len);
    } else if (strcmp(cmd, "cleartx") == 0) {
      g_wire_len = 0;
      printf("OK cleartx\n");
    } else if (strcmp(cmd, "note_echo_telemetry") == 0) {
      usart3_send_telemetry();
      printf("OK note_echo_telemetry\n");
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
    """编译 A 候选的确认门 + 帧状态机，返回可执行文件路径。"""
    import os

    src = [C_PREAMBLE, C_CHASSIS_ACK]
    ack = _load('BALANCE/chassis_ack.c')
    # 去掉两行 include：宏与原型已在 C_CHASSIS_ACK 里给出。
    ack = '\n'.join(ln for ln in ack.splitlines()
                    if not ln.lstrip().startswith('#include'))
    src.append(ack)
    uartx = _load('BALANCE/uartx_callback.c')
    balance = _load('BALANCE/balance_task.c')
    region = balance[balance.index('/*------------------------- ROS 命令监督'):]
    src.append(_block(uartx, 'static float XYZ_Target_Speed_transition(u8 High,u8 Low)'))
    callback = _block(uartx, 'static void UartxControll_Callback(USART_TypeDef* USARTx,uint8_t Usart_Receive)')
    # 状态机的 Count/Last_Uartx 提到文件作用域（与 A 的测试做法一致），
    # 其余逐字保留：USART3 门控就是被测对象。
    callback = callback.replace('\tstatic uint16_t Count = 0;\n', '')
    callback = callback.replace('\tstatic USART_TypeDef* Last_Uartx;\n', '')
    callback = callback.replace('static void UartxControll_Callback',
                                'void UartxControll_Callback', 1)
    src.append('/* shim: 读取固件私有会话标识（仅供宿主机构建，不改门控） */\n'
               'uint32_t g_test_session_id(void){ return s_session; }\n')
    src.append('static uint16_t Count = 0;\nstatic USART_TypeDef* Last_Uartx;\n')
    src.append(callback)
    src.append(_block(balance, 'void Recharge_Mode_Exit(void)\n{'))
    src.append(_block(balance, 'void Safe_Stop(void)\n{'))
    src.append(region[:region.index('uint8_t ROS_Command_Frame(float')])
    frame_fn = _block(region, 'uint8_t ROS_Command_Frame(float').replace(
        'SysVal.Time_count', '__SysVal_Time_count()')
    src.append(frame_fn)
    src.append(_block(region, 'void ROS_Command_Disarm(void)'))
    src.append(_block(region, 'static void ROS_Command_Supervise(void)'))
    src.append(PY_API_C)
    text = '\n'.join(src)
    digest = hashlib.sha256(text.encode()).hexdigest()[:16]
    base = pathlib.Path(tempfile.gettempdir()) / f'c50c-fw-ack-{os.getpid()}'
    base.mkdir(exist_ok=True)
    c = base / f'fwack_{digest}.c'
    exe = base / f'fwack_{digest}'
    if not exe.exists():
        c.write_text(text)
        subprocess.run(['gcc', '-O0', '-o', str(exe), str(c), '-lm'],
                       check=True, capture_output=True)
    return exe


class AckState(NamedTuple):
    armed: bool
    age: int
    epoch: int
    vx: float
    vy: float
    vz: float
    accept: int
    stale: int
    echo: int
    no_session: int
    bad_type: int


class AckFirmware:
    """A 候选固件（含接收确认门）的常驻模型。

    用法::

        fw = AckFirmware.start()      # 记录固件自报的会话 ID
        fw.feed(frame, session_ack)   # 主机写出的字节送进真实回调
        fw.tick(1)                    # 推进一拍 data_task（可能产生会话帧）
        fw.wire                       # 固件从 USART3 发出的字节
        fw.state                      # AckState（含 accept/stale/echo 计数）
    """

    def __init__(self, proc: subprocess.Popen, session: int) -> None:
        self._proc = proc
        self.session = session

    @classmethod
    def start(cls) -> 'AckFirmware':
        exe = build_program()
        proc = subprocess.Popen([str(exe)], stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True, bufsize=1)
        assert proc.stderr is not None
        line = proc.stderr.readline().strip().split()
        if len(line) != 2 or line[0] != 'SESSION_ID':
            proc.kill()
            raise RuntimeError(f'A 候选固件模型启动失败: {line!r}')
        session = int(line[1], 16)
        if proc.stdout.readline().strip() != 'READY':
            proc.kill()
            raise RuntimeError('A 候选固件模型未发出 READY')
        return cls(proc, session)

    def _cmd(self, line: str) -> list[str]:
        assert self._proc.stdin is not None and self._proc.stdout is not None
        self._proc.stdin.write(line + '\n')
        self._proc.stdin.flush()
        while True:
            out = self._proc.stdout.readline().strip().split()
            if not out:
                raise RuntimeError('A 候选固件模型意外退出')
            if out[0] == 'TX':
                continue          # 发送日志由 wire 统一取，不在此解析
            if out[0] == 'ERR':
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
        if data:
            self._cmd('feed ' + data.hex())
        return len(data)

    def feed(self, frame: bytes) -> int:
        if len(frame) != 11:
            raise ValueError(f'速度帧必须是 11 字节，收到 {len(frame)}')
        return self.feed_bytes(frame)

    def feed_other_port(self, data: bytes) -> int:
        if data:
            self._cmd('feedother ' + data.hex())
        return len(data)

    def tick(self, n: int = 1) -> None:
        if n > 0:
            self._cmd(f'tick {n}')

    # ------------------------------------------------------------ 观测面
    @property
    def wire(self) -> bytes:
        out = self._cmd('wire')
        hexs = out[1] if len(out) > 1 else ''
        return bytes.fromhex(hexs) if hexs else b''

    @property
    def wire_length(self) -> int:
        return int(self._cmd('txcount')[1])

    def clear_wire(self) -> None:
        self._cmd('cleartx')

    @property
    def state(self) -> AckState:
        out = self._cmd('state')
        kv = dict(item.split('=') for item in out[7:])
        return AckState(armed=int(out[1]) == 1, age=int(out[2]),
                        epoch=int(out[3]), vx=float(out[4]), vy=float(out[5]),
                        vz=float(out[6]),
                        accept=int(kv['accept']), stale=int(kv['stale']),
                        echo=int(kv['echo']),
                        no_session=int(kv['no_session']),
                        bad_type=int(kv['bad_type']))
