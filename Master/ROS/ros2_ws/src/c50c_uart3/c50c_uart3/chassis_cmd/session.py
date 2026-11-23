"""与固件 A 候选（会话帧 + 接收确认）对接的主机侧实现。

协议事实只来自 A 的固件源码（只读核对）：
``/home/snow/robot-task-worktrees/c50c-fw-preboard.a1p2/src/BALANCE/
Inc/chassis_ack.h`` 与 ``src/BALANCE/chassis_ack.c``。

本模块**只构造与解析字节，不做任何 I/O**，也不打开任何设备：

1. :func:`encode_session_ack` —— 主机确认帧。v3 的 ``take_session`` 要求
   ``frame[2]==0x01``（严格相等，能力位会使其失效）、``frame[3]==CHASSIS_PROTO_VERSION``、``frame[8]==0``、
   ``frame[4..7]`` 等于固件当前会话 ID（大端），BCC 覆盖 ``0..8``。
2. :func:`parse_session_frame` —— 解析固件主动发出的会话帧
   （``data_task.c`` 每 ``DATA_TASK_RATE`` 拍发一次，``frame[1]==0x7F``）。
3. :class:`SessionState` —— 只记录"从线上字节里读到了什么"。它**不是**
   接收确认：会话 ID 相同只能说明对方发过这一会话 ID，不能证明固件接受
   了本主机的任何命令帧。

为什么单独成模块：会话握手之前固件丢弃全部 USART3 速度帧
（``ChassisRxResult.NO_SESSION``），命令链若拿"写成功"当送达证据，就会
在固件全丢的情况下以为在走。本模块给链路提供一条**可观测的**依据：
只有真正在线上读到会话帧，才允许把 ``_rearmed`` 置位。
"""
from __future__ import annotations

import dataclasses

from . import FRAME_HEADER, FRAME_LENGTH, FRAME_TAIL, check_bcc

PROTO_VERSION = 3                 # chassis_ack.h: CHASSIS_PROTO_VERSION
CMD_SESSION = 0x7F                # chassis_ack.h: CHASSIS_TYPE_SESSION
SESSION_SUB = 0x01                # chassis_ack.h: CHASSIS_SESSION_SUB
IDX_TYPE = 1
IDX_SUB = 2
IDX_DATA = 3
IDX_CHECK = 9
IDX_TAIL = 10
SESSION_ID_BYTES = 4

# 会话帧的"新鲜度"：固件每 1 s 发一帧，容忍 3 帧丢失。
SESSION_STALE_S = 3.0


def encode_session_ack(session_id: int) -> bytes:
    """构造主机会话确认帧（11 字节）。

    会话帧 ``frame[2]`` **严格等于** :data:`SESSION_SUB`（0x01）：v3 固件的
    ``take_session`` 用 ``if (f[CHASSIS_IDX_SUB] != CHASSIS_SESSION_SUB) return;``
    整字节比对，任何能力位置位都会让确认帧被静默丢弃。能力位只属于
    速度帧（``frame[2] = 0x80 | seq``，见
    :func:`c50c_uart3.chassis_cmd.encode_speed_frame`），故本函数没有能力位参数。
    """
    if not isinstance(session_id, int) or not 0 <= session_id <= 0xFFFFFFFF:
        raise ValueError(f'会话 ID 必须是 32 位无符号整数，收到 {session_id!r}')
    frame = bytearray(FRAME_LENGTH)
    frame[0] = FRAME_HEADER
    frame[IDX_TYPE] = CMD_SESSION
    frame[IDX_SUB] = SESSION_SUB
    frame[IDX_DATA] = PROTO_VERSION
    for i in range(SESSION_ID_BYTES):
        frame[IDX_DATA + 1 + i] = (session_id >> (8 * (3 - i))) & 0xFF
    frame[IDX_DATA + 5] = 0
    frame[IDX_CHECK] = check_bcc(frame[:IDX_CHECK])
    frame[IDX_TAIL] = FRAME_TAIL
    return bytes(frame)


def decode_session_id(frame: bytes) -> int:
    """从会话帧取出 32 位会话 ID（大端 ``frame[4..7]``）。"""
    value = 0
    for i in range(SESSION_ID_BYTES):
        value = (value << 8) | frame[IDX_DATA + 1 + i]
    return value


def parse_session_frame(frame: bytes) -> int | None:
    """解析固件会话帧。合法返回会话 ID，否则返回 ``None``。

    只接受与 A 的发射格式一致的三件事：帧头帧尾正确、BCC 正确、
    ``frame[1]==0x7F`` 且 ``(frame[2] & 0x7F)==0x01``、``frame[8]==0``。
    版本号必须等于 :data:`PROTO_VERSION`；不匹配按"未知协议"拒绝，而不是
    猜一个值继续用。
    """
    if not isinstance(frame, (bytes, bytearray)) or len(frame) != FRAME_LENGTH:
        return None
    if frame[0] != FRAME_HEADER or frame[IDX_TAIL] != FRAME_TAIL:
        return None
    if frame[IDX_CHECK] != check_bcc(frame[:IDX_CHECK]):
        return None
    if frame[IDX_TYPE] != CMD_SESSION:
        return None
    if (frame[IDX_SUB] & 0x7F) != SESSION_SUB:
        return None
    if frame[IDX_DATA] != PROTO_VERSION:
        return None
    if frame[IDX_DATA + 5] != 0:
        return None
    return decode_session_id(frame)


class SessionExtractor:
    """从连续字节流里找会话帧。

    固件**按 1 s 间隔**主动发会话帧，而遥测帧（48 字节）与命令回显共用
    同一起始字节 ``0x7B``。本提取器只做一件事：在流里扫描以 ``0x7B``
    起始、长度为 11 且能通过 :func:`parse_session_frame` 的位置。
    它不假设帧边界，也不缓存跨调用的半个帧——接收由 A 的固件解析器负责。
    """

    def __init__(self) -> None:
        self.frames = 0
        self.bytes_seen = 0

    def feed(self, data: bytes) -> list[int]:
        ids: list[int] = []
        if not data:
            return ids
        self.bytes_seen += len(data)
        for i in range(len(data) - FRAME_LENGTH + 1):
            if data[i] != FRAME_HEADER:
                continue
            candidate = data[i:i + FRAME_LENGTH]
            session_id = parse_session_frame(candidate)
            if session_id is not None:
                ids.append(session_id)
                self.frames += 1
        return ids


@dataclasses.dataclass
class SessionObservation:
    """链路对固件会话的观测结果（不是确认，只描述看到过什么）。"""

    session_id: int | None = None
    observed_at: float | None = None
    frames_seen: int = 0
    acks_sent: int = 0

    @property
    def acknowledged(self) -> bool:
        return self.session_id is not None and self.acks_sent > 0

    def is_fresh(self, now: float) -> bool:
        if self.observed_at is None:
            return False
        return (now - self.observed_at) <= SESSION_STALE_S

    def describe(self) -> dict:
        return {
            'session_id': self.session_id,
            'session_frames_seen': self.frames_seen,
            'acks_sent': self.acks_sent,
            'observed_at': self.observed_at,
        }


__all__ = [
    'CMD_SESSION', 'PROTO_VERSION', 'SESSION_STALE_S',
    'SESSION_SUB', 'SessionExtractor', 'SessionObservation',
    'decode_session_id', 'encode_session_ack', 'parse_session_frame',
]
