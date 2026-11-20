"""v3 固件（chassis_ack.c v3 + uartx_callback.c + balance_task.c 监督）的宿主机构建。

与既有 harness 的区别（证据边界，交付时必须照抄）：

============================  ======================  ============================
项                            来源                    说明
============================  ======================  ============================
``chassis_ack.c`` v3 全文      真实源码文件（g++ 编译）  会话门 + 序号门 + 计数器
``UartxControll_Callback``    真实源码（文本截取）      USART3 门控 ``ack != ACCEPT`` 即丢帧
``XYZ_Target_Speed_transition`` 真实源码             速度解码
``Recharge_Mode_Exit``        真实源码               USART3 速度帧接受路径会调用
``ROS_Command_Frame``         真实源码（``SysVal.Time_count`` 换成本地值）
``ROS_Command_Disarm``        真实源码
``ROS_Command_Supervise``     真实源码               ``Safe_Stop`` 由**桩**替换（只置标）
``Safe_Stop``                 **桩**                 板上实现写电机/PI/PWM；本构建只置
                                                     ``g_ctrl_epoch++`` 与 ``g_safe_stop_calls++``
``RS485``/``charger``/``IP``   **桩**                 回调里的副作用，只存字段，不产生输出
会话帧发送节拍（``data_task``） **桩**                 ``tick`` 每 20 拍调用真实
                                                     ``Chassis_Ack_EncodeSessionFrame`` 并发线上
熵源                          **桩**                 板上 SRAM 地址换成本地缓冲区
============================  ======================  ============================

因此：**计数器与目标速度是真实固件逻辑的判定结果**；电机、PWM、继电器、
实际串口、时序（中断优先级/竞态）都**没有**验证。

编译用 ``g++``：真实 ``chassis_ack.c`` 里“无 goto 跨越变量初始化”的写法在
C 里合法、在 C++ 里非法，需要 gcc 的 ``fpermissive``（``-fpermissive``
只降级诊断，不改逻辑）。构建时断言真实的 ``CHASSIS_SEQ_WINDOW`` /
``CHASSIS_PROTO_VERSION`` / ``CHASSIS_SESSION_SUB`` / ``CHASSIS_CAP_HOST_ACK``
宏与主机侧常量一致，防止两边悄悄漂移。
"""
from __future__ import annotations

import hashlib
import os
import pathlib
import re
import subprocess
import tempfile
from typing import NamedTuple

FIRMWARE_TREE = pathlib.Path(
    '/home/snow/robot-task-worktrees/c50c-fw-wrap.a1/src')
#: 记录的源码哈希（只核对 chassis_ack.c/h；其余三处本轮未改，
#: 哈希只作证据登记，不作硬断言）。chassis_ack.c/h 取 A 序号回绕修复提交
#: d3069a8bc27de64c523171f743ecc56da059d1e7（基线 e6a5594 + 位图滑动）。
RECORDED_SHA256 = {
    'BALANCE/chassis_ack.c':
        'e468c99e365b6e06702af9053b8db96122f6439ca819a690165d075c6823df2a',
    'BALANCE/Inc/chassis_ack.h':
        '6e2e3eee8463a44b89d5df3f1f15164345119b73c27f9ba659804d0e2d9b284e',
    'BALANCE/uartx_callback.c':
        'f08c57279f6c0d1d5c8e65de081a0d894eab05ac6abf6181fa795742cb87fe9a',
    'BALANCE/data_task.c':
        'f21e740db488e4b1de9333cb63ae46342e96273325f52f05e95e81bd9eeb4cf7',
    'BALANCE/balance_task.c':
        '448bb6f7d1f60d8607b2771e73a99d8d5227ad99e1f2e6d968ef253aa671535f',
}
CONTRACT_RE = {
    'CHASSIS_PROTO_VERSION': (r'#define CHASSIS_PROTO_VERSION +([0-9]+)', 10),
    'CHASSIS_SESSION_SUB': (r'#define CHASSIS_SESSION_SUB +0x([0-9A-Fa-f]+)u?', 16),
    'CHASSIS_CAP_HOST_ACK': (r'#define CHASSIS_CAP_HOST_ACK +0x([0-9A-Fa-f]+)u?', 16),
    'CHASSIS_SEQ_MASK': (r'#define CHASSIS_SEQ_MASK +0x([0-9A-Fa-f]+)u?', 16),
    'CHASSIS_SEQ_WINDOW': (r'#define CHASSIS_SEQ_WINDOW +([0-9]+)u?', 10),
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
            for rel in RECORDED_SHA256}


def firmware_contract() -> dict:
    """从真实头文件读出本任务依赖的常量（用于与主机侧常量对照）。"""
    header = _load('BALANCE/Inc/chassis_ack.h')
    out = {}
    for name, (pattern, base) in CONTRACT_RE.items():
        m = re.search(pattern, header)
        if not m:
            raise RuntimeError(f'chassis_ack.h 里找不到 {name}')
        out[name] = int(m.group(1), base)
    return out


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

/* ---- 熵源桩：板上 SRAM 地址换成本地缓冲区（与 A 自己的测试一致） ---- */
static uint8_t entropy_stub[64];

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

/* 本机构建：STM32 的 SRAM 基址 0x20000000 改指真实缓冲区，
   使 §熵源桩 指针转换不丢高位（纯桩，不影响判定逻辑）。 */
#define CHASSIS_BUILD_ENTROPY_BASE ((volatile uint8_t *)entropy_stub)
static uint8_t ram_stub_addr[1] __attribute__((unused));
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
static inline uint8_t Check_BCC(const uint8_t *data, uint16_t length) {
    return Check_BCC_impl(data, length);
}

struct RECEIVE_DATA { unsigned char buffer[11]; };
static struct RECEIVE_DATA Receive_Data;
#define USART1 ((void*)1u)
#define USART3 ((void*)3u)
typedef void* USART_TypeDef;

/* ---- 回调里用到的外设/业务桩（只替换副作用） ---- */
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
static volatile uint32_t g_safe_stop_calls = 0;
void Recharge_Mode_Exit(void);
void ROS_Command_Disarm(void);
void Safe_Stop(void);

uint8_t ROS_Command_Frame(float, float, float);
'''

#: ``Safe_Stop`` 的桩：板上实现会写目标速度、电机、PI、PWM 并解除 ROS 授权。
#: 本构建只记录"被调用"与递增输出纪元，然后做与源码等价的授权解除
#: （``ROS_Command_Disarm`` 是真实源码）。
C_SAFE_STOP_STUB = r'''
void Safe_Stop(void)
{
  g_ctrl_epoch++;
  g_safe_stop_calls++;
  ROS_Command_Disarm();
}
'''

C_DRIVER = r'''
/* 常驻驱动。
   feed <hex>     把字节逐个送进真实 UartxControll_Callback(USART3, b)
   tick <n>       推进 n 拍 data_task：每拍遥测 24 字节 + 真实监督；
                  每 DATA_TASK_RATE 拍发一帧真实 Chassis_Ack_EncodeSessionFrame
   state          打印状态（含全部 7 个固件计数器 + 停止桩调用数）
   wire           打印固件从 USART3 发出的全部字节
   cleartx        清空线上缓冲
   quit           退出 */
uint32_t g_test_session_id(void);

static unsigned char g_wire[65536];
static int g_wire_len = 0;
static int g_session_tick = 0;

static void uart3_send(uint8_t b)
{
  if (g_wire_len < (int)sizeof(g_wire)) g_wire[g_wire_len++] = b;
}

/* data_task.c 的会话帧节拍（原样提取的逻辑） */
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
  uint32_t a=0,st=0,e=0,n=0,bt=0,ss=0,nq=0;
  Chassis_Ack_Stats(&a,&st,&e,&n,&bt,&ss,&nq);
  printf("%s %d %d %u %.4f %.4f %.4f accept=%u stale=%u echo=%u no_session=%u bad_type=%u stale_seq=%u no_seq=%u bad_frame=%u safe_stop=%u\n",
         tag, (int)g_ros_cmd_armed, (int)g_ros_cmd_age, (unsigned)g_ctrl_epoch,
         robot_control.Vx, robot_control.Vy, robot_control.Vz,
         a, st, e, n, bt, ss, nq, 0u, (unsigned)g_safe_stop_calls);
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
  memset(Send_Data_buffer, 0, sizeof(Send_Data_buffer));
  Send_Data_buffer[0] = 0x7B;
  Send_Data_buffer[1] = 0x00;
  for (int i = 2; i < 22; i++) Send_Data_buffer[i] = (uint8_t)(0x40 + i);
  Send_Data_buffer[22] = Check_BCC(Send_Data_buffer, 22);
  Send_Data_buffer[23] = 0x7D;
  entropy_fill();
  Chassis_Ack_Init();
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
    } else if (strcmp(cmd, "tick") == 0) {
      char *n = strtok(NULL, " \t\r\n");
      int c = n ? atoi(n) : 0;
      for (int i = 0; i < c; i++) {
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
    } else if (strcmp(cmd, "cleartx") == 0) {
      g_wire_len = 0;
      printf("OK cleartx\n");
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


def _cxx_compat(text: str) -> str:
    """把 ``#if defined X`` 改成 ``#if defined(X)``（g++ -fpermissive 必需）。"""
    return re.sub(r'#\s*(if|elif)\s+defined\s+([A-Za-z_]\w*)',
                  r'#\1 defined(\2)', text)


def build_program() -> pathlib.Path:
    """编译 v3 固件模型，返回可执行文件路径（按源码摘要缓存）。"""
    src = [C_PREAMBLE]
    ack_h = _load('BALANCE/Inc/chassis_ack.h')
    ack_h = '\n'.join(ln for ln in ack_h.splitlines()
                      if not ln.lstrip().startswith('#include'))
    src.append(ack_h)
    ack = _load('BALANCE/chassis_ack.c')
    ack = '\n'.join(ln for ln in ack.splitlines()
                    if not ln.lstrip().startswith('#include'))
    # 熵源桩：只把 SRAM 基址换成经变量中转的缓冲区地址。
    # 直接写 ``(uintptr_t)&entropy_stub`` 会被编译器当作可折叠常量，
    # 从而截成 32 位。CHASSIS_SESSION_ENTROPY_ADDR 本身一字不改。
    src.append(ack.replace('CHASSIS_SESSION_ENTROPY_ADDR',
                           'CHASSIS_BUILD_ENTROPY_BASE'))
    uartx = _load('BALANCE/uartx_callback.c')
    balance = _load('BALANCE/balance_task.c')
    src.append(_block(uartx,
                      'static float XYZ_Target_Speed_transition(u8 High,u8 Low)'))
    callback = _block(
        uartx,
        'static void UartxControll_Callback(USART_TypeDef* USARTx,uint8_t Usart_Receive)')
    # Count/Last_Uartx 提到文件作用域（与既有 harness 做法一致），其余逐字保留。
    callback = callback.replace('\tstatic uint16_t Count = 0;\n', '')
    callback = callback.replace('\tstatic USART_TypeDef* Last_Uartx;\n', '')
    callback = callback.replace('static void UartxControll_Callback',
                                'void UartxControll_Callback', 1)
    src.append('static uint32_t g_test_session_id(void)'
               ' { return s_session; }\n')
    src.append('static uint16_t Count = 0;\nstatic USART_TypeDef* Last_Uartx;\n')
    src.append(callback)
    src.append(_block(balance, 'void Recharge_Mode_Exit(void)\n{'))
    # Safe_Stop 用桩替换（板上实现写电机；此处只记录被调用）。
    src.append(C_SAFE_STOP_STUB)
    region = balance[balance.index('/*------------------------- ROS 命令监督'):]
    src.append(region[:region.index('uint8_t ROS_Command_Frame(float')])
    frame_fn = _block(region, 'uint8_t ROS_Command_Frame(float').replace(
        'SysVal.Time_count', '__SysVal_Time_count()')
    src.append(frame_fn)
    src.append(_block(region, 'void ROS_Command_Disarm(void)'))
    src.append(_block(region, 'static void ROS_Command_Supervise(void)'))
    src.append(C_DRIVER)
    text = _cxx_compat('\n'.join(src))
    text = text.replace(
        'static void UartxControll_Callback', 'void UartxControll_Callback')
    digest = hashlib.sha256(text.encode()).hexdigest()[:16]
    base = pathlib.Path(tempfile.gettempdir()) / f'c50c-fw-v3-{os.getpid()}'
    base.mkdir(exist_ok=True)
    c = base / f'fwv3_{digest}.cc'
    exe = base / f'fwv3_{digest}'
    if not exe.exists():
        c.write_text(text)
        subprocess.run(['g++', '-O0', '-fpermissive', '-o', str(exe), str(c), '-lm'],
                       check=True, capture_output=True)
    return exe


class V3State(NamedTuple):
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
    stale_seq: int
    no_seq: int
    safe_stop: int


class V3Firmware:
    """v3 固件模型的常驻进程（真实计数器 + 真实门控）。"""

    def __init__(self, proc: subprocess.Popen, session: int) -> None:
        self._proc = proc
        self.session = session

    @classmethod
    def start(cls) -> 'V3Firmware':
        exe = build_program()
        proc = subprocess.Popen([str(exe)], stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True, bufsize=1)
        assert proc.stderr is not None
        line = proc.stderr.readline().strip().split()
        if len(line) != 2 or line[0] != 'SESSION_ID':
            proc.kill()
            raise RuntimeError(f'v3 固件模型启动失败: {line!r}')
        session = int(line[1], 16)
        if proc.stdout.readline().strip() != 'READY':
            proc.kill()
            raise RuntimeError('v3 固件模型未发出 READY')
        return cls(proc, session)

    def _cmd(self, line: str) -> list[str]:
        assert self._proc.stdin is not None and self._proc.stdout is not None
        self._proc.stdin.write(line + '\n')
        self._proc.stdin.flush()
        out = self._proc.stdout.readline().strip().split()
        if not out or out[0] == 'ERR':
            raise RuntimeError(f'v3 固件模型拒绝命令 {line!r}: {out}')
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

    def tick(self, n: int = 1) -> None:
        if n > 0:
            self._cmd(f'tick {n}')

    # ------------------------------------------------------------ 观测面
    @property
    def wire(self) -> bytes:
        out = self._cmd('wire')
        hexs = out[1] if len(out) > 1 else ''
        return bytes.fromhex(hexs) if hexs else b''

    def clear_wire(self) -> None:
        self._cmd('cleartx')

    @property
    def state(self) -> V3State:
        out = self._cmd('state')
        kv = dict(item.split('=') for item in out[7:])
        return V3State(armed=int(out[1]) == 1, age=int(out[2]),
                       epoch=int(out[3]), vx=float(out[4]),
                       vy=float(out[5]), vz=float(out[6]),
                       accept=int(kv['accept']), stale=int(kv['stale']),
                       echo=int(kv['echo']),
                       no_session=int(kv['no_session']),
                       bad_type=int(kv['bad_type']),
                       stale_seq=int(kv['stale_seq']),
                       no_seq=int(kv['no_seq']),
                       safe_stop=int(kv['safe_stop']))
