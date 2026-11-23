"""基线固件（2026-11-05 c50c-four-wheel-50pct）的发送方言。

原由：候选线 ``session.py`` / ``seq.py`` 实现的是**候选固件**的 v3 契约
（会话帧 ``frame[1]=0x7F``、ack 子字节 ``frame[2]=0x80|seq``、接受序号
窗口）。已交付基线固件没有这些概念——见契约表：基线只按
``frame[1]==0`` 识别速度帧，``frame[2]`` 完全不读，无会话、无 ack、
无序号。基线接收路径（``BALANCE/uartx_callback.c``）：

1. 帧头 ``0x7B``（``FRAME_HEADER``），帧尾 ``0x7D``（``FRAME_TAIL``），
   长度固定 11（``RECEIVE_DATA_SIZE``）。
2. ``buffer[10] == FRAME_TAIL`` 与 ``buffer[9] == Check_BCC(buffer, 9)``
   同时成立才算合法帧；BCC 是前 9 字节的 XOR。
3. ``buffer[1]==0`` → 速度命令：``Vx=[3],[4]``、``Vy=[5],[6]``、
   ``Vz=[7],[8]``，大端 int16，单位 0.001 m/s。
4. 合法帧把 ``command_lostcount`` 清零；约 1 s 无合法帧，固件自己把三轴
   目标速度清零（``SecurityLevel==0``）。所以零速帧同时是心跳。

与 v3 编码的唯一差别是 ``frame[2]``：这里有意义地留 0（基线不读），
候选线写 ``0x80|seq``。两种写法基线都接受（``frame[2]`` 不参与判定），
但**写 0 才是"基线真的有这个概念"的样子**，也让字节流可以被
基线协议的解码器复现。

本模块只构造字节，不做 I/O，不打开任何设备。
"""
from __future__ import annotations

from . import (FIRMWARE_SPEED_LIMIT_MPS, FRAME_HEADER, FRAME_LENGTH,
               FRAME_TAIL, PAYLOAD_BYTES, WIRE_LIMIT_MPS, check_bcc,
               _to_int16_be)

#: 状态帧（固件 → 主机）：48 字节，见 ``BALANCE/data_task.c``。
#: task-15 起由 24 字节扩为 48 字节：``[22..37]`` 四轮编码器 int32 大端，
#: ``[38..45]`` TIM8->CCR1..CCR4 的 uint16 大端，BCC 移到位 46，帧尾到位 47。
STATUS_FRAME_LENGTH = 48
STATUS_BCC_COVER_LEN = 46
#: 扩帧起点与结尾偏移（供调用方取值）。
STATUS_ENCODER_OFFSET = 22
STATUS_PWM_OFFSET = 38

#: 状态帧新鲜度：固件每 ``DATA_TASK_RATE``（20 Hz）发一帧；容忍 3 s。
BASELINE_STATUS_STALE_S = 3.0


def encode_baseline_speed_frame(vx: float, vz: float, vy: float = 0.0) -> bytes:
    """构造基线固件接受的速度帧（11 字节）。

    ``vx``/``vy``/``vz`` 单位 m/s，与候选线 ``encode_speed_frame`` 同序
    （``vx, vz, vy``），避免调用点误解参数位置。超限抛 ``ValueError``，
    绝不静默钳位——钳位会让调用方以为发出去的值就是它要的值。
    """
    for name, value in (('vx', vx), ('vy', vy), ('vz', vz)):
        if abs(float(value)) > WIRE_LIMIT_MPS:
            raise ValueError(f'{name}={value} 超出线上可表示范围 ±{WIRE_LIMIT_MPS}')
        if abs(float(value)) > FIRMWARE_SPEED_LIMIT_MPS:
            raise ValueError(f'{name}={value} 超出固件限速 ±{FIRMWARE_SPEED_LIMIT_MPS} m/s')
    frame = bytearray(FRAME_LENGTH)
    frame[0] = FRAME_HEADER
    frame[1] = 0                      # CMD_SPEED：基线只认这一个命令字
    frame[2] = 0                      # 基线不读本字节（无 ack 位、无序号）
    for idx, v in ((3, vx), (5, vy), (7, vz)):
        hi, lo = _to_int16_be(int(round(float(v) * 1000)))
        frame[idx] = hi
        frame[idx + 1] = lo
    frame[9] = check_bcc(frame[:PAYLOAD_BYTES])
    frame[10] = FRAME_TAIL
    return bytes(frame)


def encode_baseline_zero_frame() -> bytes:
    """零速帧：既是受控停止，也是固件命令丢失保护的心跳。"""
    return encode_baseline_speed_frame(0.0, 0.0, 0.0)


def parse_status_frame(frame: bytes) -> dict | None:
    """解析状态帧（48 字节）。合法返回字段字典，否则 ``None``。

    用于把固件回传当作**接收方向证据**：主机只有在线上真的读到基线格式
    的状态帧，才能证明对方是基线固件且字节流已对齐。该证据不证明固件
    接受了主机的任何命令帧（基线不回复确认）——见交付说明的边界。
    """
    if not isinstance(frame, (bytes, bytearray)):
        return None
    if len(frame) != STATUS_FRAME_LENGTH:
        return None
    if frame[0] != FRAME_HEADER or frame[STATUS_FRAME_LENGTH - 1] != FRAME_TAIL:
        return None
    bcc = 0
    for b in frame[:STATUS_BCC_COVER_LEN]:
        bcc ^= b
    if frame[STATUS_BCC_COVER_LEN] != bcc:
        return None

    def i16(off: int) -> int:
        return int.from_bytes(frame[off:off + 2], 'big', signed=True)

    def i32(off: int) -> int:
        return int.from_bytes(frame[off:off + 4], 'big', signed=True)

    def u16(off: int) -> int:
        return int.from_bytes(frame[off:off + 2], 'big')

    enc_off = STATUS_ENCODER_OFFSET
    pwm_off = STATUS_PWM_OFFSET
    return {
        'stop_flag': frame[1],
        'velocity_mm_s': (i16(2), i16(4)),
        'yaw_rate_mrad_s': i16(6),
        'voltage_mv': int.from_bytes(frame[20:22], 'big'),
        # 四轮编码器：A/B/C/D，int32 大端。数值来自浮点轮速转 int，不是脉冲计数。
        'encoder_counts': tuple(i32(enc_off + i * 4) for i in range(4)),
        # 四轮 PWM 比较值：CCR1..CCR4，uint16 大端，来自 TIM8 实际输出。
        'pwm_counts': tuple(u16(pwm_off + i * 2) for i in range(4)),
    }


__all__ = [
    'BASELINE_STATUS_STALE_S', 'STATUS_BCC_COVER_LEN', 'STATUS_ENCODER_OFFSET',
    'STATUS_FRAME_LENGTH', 'STATUS_PWM_OFFSET',
    'encode_baseline_speed_frame', 'encode_baseline_zero_frame',
    'parse_status_frame',
]
