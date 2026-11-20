"""基线固件 USART3 接收解析的**逐行复刻提取模型**（离线，不碰硬件）。

证据边界（必读）
================
1. 这是**按源码逐行复刻的提取模型**，不是时序仿真器。它在主机上重放
   ``2026-11-08-50pct-migrate/BALANCE/uartx_callback.c`` 的字节状态机。
   **本模型未在真实硬件上运行**：没有连接串口、没有改 DTR/RTS、没有烧录、
   没有部署、没有启动电机、没有发真实运动命令。
2. 本模型证明的是"给定这些字节，固件那几行代码会算出什么"。
   它**不**证明固件收到过任何帧，**也不**证明主机写入成功等于固件执行。
   主机写入成功 / 收到状态帧都不是固件已接受的证据。
3. 每次构造都核对四个源文件的 sha256（:data:`RECORDED_SHA256`）。固件一旦
   被改，模型立即报错，不静默沿用旧提取。

复刻范围（逐条对齐真实代码）
============================
来源 ``BALANCE/uartx_callback.c``：

- ``UartxControll_Callback``（第 30-200 行）的字节状态机：

  * ``Count`` 与 ``Last_Uartx`` 为函数内 ``static``，**跨字节保持**。
  * 换口（``USARTx != Last_Uartx`` 且 ``Count != 0``）→ ``Count = 0``。
  * 先写 ``buffer[Count] = 字节``，再判 ``字节 == 0x7B || Count > 0``：
    真则 ``Count++``，假则 ``Count = 0``。因此非帧头字节落在 ``buffer[0]``。
  * ``Count == 11`` 时先复位 ``Count``，再依次判帧尾 ``buffer[10]==0x7D``、
    BCC ``buffer[9]==Check_BCC(buffer,9)``。
  * 两项都过 → ``command_lostcount = 0``；``buffer[1]==0`` → 速度命令，
    ``Set_Control_Mode(_ROS_Control)``（仅 USART3）、``AllowRecharge = 0``、
    三轴赋值、``ros_cmd_tick`` 打时间戳。
  * ``buffer[1]==1`` / ``==2`` 分支本模型只记命令字，不建模充电器字段。
- ``XYZ_Target_Speed_transition``（第 470 行）：
  ``transition = (High << 8) + Low`` 赋给 ``short``（16 位补码截断）；
  ``return transition/1000 + (transition%1000)*0.001``。
  C 的 ``/`` 与 ``%`` 向零截断，与 Python ``//`` 不同，故逐位复刻。
- ``data_task.c`` ``Check_BCC``：前 9 字节逐字节 XOR。
- ``uartx_callback.c:432-435``：``SysVal.Time_count < CONTROL_DELAY`` 时
  中断直接返回，字节被丢弃（``CONTROL_DELAY = 1000``，见 balance_task.h）。

没有覆盖的固件行为（如实列出）
==============================
- **中断/时序**：真实入口是 USART3 RXNE 中断，本模型按字节同步喂入。丢字节、
  溢出、DMA、波特率误差一律不建模。
- **``Count < 11`` 的自然回退**：11 字节未满时不会重新对齐，模型照抄；但
  真实中断里 ``Receive_Data`` 是全局共享结构，多个串口同时写会互相覆盖，
  本模型只模拟单口 USART3。
- **``CHASSIS_PWM_TEST``**：``Makefile:49-51`` 默认不给
  ``PWM_TEST_DEFS``，未定义该宏。故 PWM 直通分支不参与编译，本模型也不
  复刻它（只在 :data:`UNMODELED` 里记录）。
- **``AKM_CAR``**：``DEFS`` 里只有 ``_4WD_CAR``，未定义 ``AKM_CAR``。
  故 ``Vz`` 不经 ``Akm_Vz_to_Angle`` 改写。
- **``command_lostcount`` 的超时清零**（``balance_task.c:127-129`` 及
  ``SecurityLevel``）不在此模型内，见已有的 ``fw_ros_drive_model.py``。
- **速度被接受之后是否真的驱动电机**（``balance_task.c`` 的
  ``ResponseControl`` / 限速 / 调试分支）不在此模型内。
- **``ros_cmd_tick`` 之后的四轮输出与超时降级**不在此模型内。
- **波特率、电平、DTR/RTS、USB 转串口**等一切物理层行为不建模。

与 ``fw_ros_drive_model.py`` 的区别
===================================
那个模型用 gcc 编译真实固件片段、观察 ``Set_Pwm`` 入参；本模型是纯 Python
的字节级重放，只回答"这 11 个字节会被解析成什么 Vx/Vy/Vz、被接受还是丢弃"。
两者互不替代。
"""
from __future__ import annotations

import hashlib
import pathlib

#: 固件源码目录。**只读**：本模型不写这个目录。
FIRMWARE_DIR = (pathlib.Path(__file__).resolve().parents[4]
                / '2026-11-08-50pct-migrate')

#: 模型依赖的源文件及提取时的 sha256。不符即报错。
RECORDED_SHA256 = {
    'BALANCE/uartx_callback.c':
        'b41f33d33b5df408df77c7b97ac92e4d26eb1a416927320984b5701a93b6c201',
    'BALANCE/Inc/uartx_callback.h':
        '16d3870067f22d91c2f335ee793cd92be06c0c7f987ada98f1518d071fc30b40',
    'BALANCE/data_task.c':
        '1fbe665587444a9413c1562cb1cd602238a66ab572b42ab7eb8547e5a722e414',
    'BALANCE/Inc/data_task.h':
        '7218e7ef183051be73c314c2fc42503a63dba6b82433093f09974c749e8438ab',
}

#: 真实常量（取自源码，不猜）。
FRAME_HEADER = 0x7B          # data_task.h:12
FRAME_TAIL = 0x7D            # data_task.h:13
RECEIVE_DATA_SIZE = 11       # uartx_callback.h:7
CONTROL_DELAY = 1000         # balance_task.h：开机 CONTROL_DELAY 拍前不处理
BCC_COVER_LEN = 9            # data_task.c:55 Check_BCC(buffer, 9)
CMD_SPEED = 0                # uartx_callback.c:92 buffer[1]==0 → 速度命令
CMD_CHARGE = (1, 2)          # uartx_callback.c:115 充电/导航字

#: 本模型**不覆盖**的固件行为（测试直接引用这一清单）。
UNMODELED = (
    'USART3 RXNE 中断时序、丢字节、溢出、波特率误差',
    '多串口并发写同一个 Receive_Data 结构',
    'CHASSIS_PWM_TEST 直通分支（Makefile 默认未定义该宏）',
    'AKM_CAR 的 Vz→前轮转角改写（DEFS 只定义 _4WD_CAR）',
    'command_lostcount 超时清零与 SecurityLevel 判定',
    'ResponseControl 的四轮输出、限速与调试分支',
    'ros_cmd_tick 之后的 ROS 超时降级',
    '物理层：电平、DTR/RTS、USB 转串口、烧录与上板执行',
)


def verify_sources() -> dict:
    """核对源文件哈希；不符就抛错，不静默沿用旧提取。"""
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


def check_bcc(data) -> int:
    """``data_task.c:73-78`` 逐字节 XOR。"""
    bcc = 0
    for b in data:
        bcc ^= b
    return bcc


def _c_short(value: int) -> int:
    """``transition = (High << 8) + Low`` 赋给 16 位 ``short`` 的截断语义。"""
    value &= 0xFFFF
    return value - 0x10000 if value >= 0x8000 else value


def _c_divmod_by_1000(transition: int) -> tuple[int, int]:
    """C 的 ``/`` 与 ``%`` 向零截断（负数与 Python 的 ``//`` 不同）。"""
    q = abs(transition) // 1000
    r = abs(transition) % 1000
    if transition < 0:
        q, r = -q, -r
    return q, r


def xyz_target_speed_transition(high: int, low: int) -> float:
    """``uartx_callback.c:470`` 的公式：``transition/1000 + (transition%1000)*0.001``。

    注意：真实固件里 ``transition`` 是 ``short``，本函数同样先做 16 位截断。
    """
    transition = _c_short((high << 8) + low)
    q, r = _c_divmod_by_1000(transition)
    return q + r * 0.001


class BaselineUartxModel:
    """``UartxControll_Callback`` 的字节级重放（USART3 单口）。

    调用 :meth:`feed_byte` 逐个喂字节，语义与真实函数逐行对应。
    """

    def __init__(self, *, now_ms: int = 0) -> None:
        # 真实是 static：跨字节保持
        self._count = 0
        self._last_uartx = None
        self.buffer = bytearray(RECEIVE_DATA_SIZE)
        # 可观测的固件侧变量
        self.command_lostcount = 0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.control_mode = 0          # Set_Control_Mode(_ROS_Control)
        self.allow_recharge = 1        # 真实初值 1（charger.AllowRecharge）
        self.ros_cmd_tick = None
        self.now_ms = int(now_ms)
        self.time_count = CONTROL_DELAY  # 默认开机延时已过
        # 统计
        self.accepted_speed_frames = 0
        self.accepted_other_frames = 0
        self.rejected_tail = 0
        self.rejected_bcc = 0
        self.dropped_by_startup_delay = 0
        # 最近一次被接受的字节（便于断言）
        self.last_frame = None

    # ---------------------------------------------------------- 真实变量
    def set_control_mode(self, mask: int) -> None:
        """``Set_Control_Mode`` 宏（balance_task.h）：先或后与。"""
        self.control_mode = (self.control_mode | mask) & mask

    # ---------------------------------------------------------- 字节入口
    def feed_byte(self, byte: int, *, uartx: str = 'USART3') -> None:
        """喂一个字节（对应 USART3 RXNE 中断里的一次调用）。"""
        # 中断层：SysVal.Time_count < CONTROL_DELAY 时直接返回
        if self.time_count < CONTROL_DELAY:
            self.dropped_by_startup_delay += 1
            return

        # ---- UartxControll_Callback 第 32-41 行
        if uartx != self._last_uartx and self._count != 0:
            self._count = 0
        self._last_uartx = uartx
        self.buffer[self._count] = byte & 0xFF

        # ---- 第 45-49 行
        if byte == FRAME_HEADER or self._count > 0:
            self._count += 1
        else:
            self._count = 0

        # ---- 第 51-56 行
        if self._count != RECEIVE_DATA_SIZE:
            return
        self._count = 0
        self._on_full_frame(uartx)

    def _on_full_frame(self, uartx: str) -> None:
        buf = self.buffer
        if buf[10] != FRAME_TAIL:                      # 第 56 行
            self.rejected_tail += 1
            return
        if buf[9] != check_bcc(buf[:BCC_COVER_LEN]):   # 第 58 行
            self.rejected_bcc += 1
            return

        # 第 86 行：合法帧先清丢帧计数
        self.command_lostcount = 0
        self.last_frame = bytes(buf)

        if buf[1] == CMD_SPEED:                        # 第 92 行
            if uartx == 'USART1':
                self.set_control_mode(1 << 5)          # _USART_Control
            elif uartx == 'USART3':
                self.set_control_mode(1)               # _ROS_Control
            self.allow_recharge = 0
            self.vx = xyz_target_speed_transition(buf[3], buf[4])
            self.vy = xyz_target_speed_transition(buf[5], buf[6])
            self.vz = xyz_target_speed_transition(buf[7], buf[8])
            # ROS_STRAIGHT_DRIVE 打开时才打时间戳
            self.ros_cmd_tick = self.now_ms
            self.accepted_speed_frames += 1
            return

        if buf[1] in CMD_CHARGE:                       # 第 115 行
            self.allow_recharge = 1
            self.accepted_other_frames += 1
            return

        # 其它命令字：本模型未建模其字段，只计为"被接受的非速度帧"
        self.accepted_other_frames += 1

    def feed(self, data: bytes, *, uartx: str = 'USART3') -> None:
        for b in data:
            self.feed_byte(b, uartx=uartx)

    # ---------------------------------------------------------- 观测
    def snapshot(self) -> dict:
        return {
            'vx': self.vx, 'vy': self.vy, 'vz': self.vz,
            'control_mode': self.control_mode,
            'allow_recharge': self.allow_recharge,
            'ros_cmd_tick': self.ros_cmd_tick,
            'command_lostcount': self.command_lostcount,
            'accepted_speed_frames': self.accepted_speed_frames,
            'accepted_other_frames': self.accepted_other_frames,
            'rejected_tail': self.rejected_tail,
            'rejected_bcc': self.rejected_bcc,
            'dropped_by_startup_delay': self.dropped_by_startup_delay,
        }


__all__ = ['BCC_COVER_LEN', 'BaselineUartxModel', 'FIRMWARE_DIR',
           'RECORDED_SHA256', 'UNMODELED', 'check_bcc',
           'verify_sources', 'xyz_target_speed_transition']
