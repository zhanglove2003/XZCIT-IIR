"""模块级共享测试数据：LD19 帧构造、时序工具。"""

import math
import struct
import types

# 与生产代码同一份 CRC8 表，避免测试重复抄写导致漂移
from opi_robot.lidar_node.codec import CRC_TABLE

FRAME_LEN = 47


def crc8(data):
    """按 LD19 查表算法计算 CRC8。"""
    c = 0
    for b in data:
        c = CRC_TABLE[(c ^ b) & 0xFF]
    return c


def build_frame(speed_ddeg=360, start_ddeg=0, end_ddeg=None,
                span_ddeg=None,
                points=None, bad_crc=False, speed=360):
    """构造一帧 47 字节 LD19 数据。

    start_ddeg/end_ddeg 单位为 0.01 度，end_ddeg 用 16 位打包，最大 65535
    （655.35 度）。speed 为帧头转速字段（0.01 度/秒）。
    不传 end_ddeg 时用 span_ddeg 推跨度；默认 100（1 度）是真实 LD19 的单包跨度。
    要构造整圈覆盖的帧时传 span_ddeg=36000。
    points 为 12 个距离（毫米），0 表示该点无效。
    """
    if points is None:
        points = [500] * 12
    if end_ddeg is None:
        # 未给跨度时按 12 个点各 1 度推算
        span = 12 * 100 if span_ddeg is None else int(span_ddeg)
        end_ddeg = int(start_ddeg) + span
    body = bytearray()
    body += b'\x54\x2c'
    body += struct.pack('<H', speed)
    body += struct.pack('<H', int(start_ddeg) % 36000)
    for d in points:
        body += struct.pack('<H', int(d)) + bytes([10])
    # 协议规定起止角都落在 [0, 36000)；整圈收尾帧超出时必须回绕
    body += struct.pack('<H', int(end_ddeg) % 36000)
    body += struct.pack('<H', 0)
    crc = crc8(bytes(body))
    if bad_crc:
        crc ^= 0x5A
    body.append(crc)
    assert len(body) == FRAME_LEN
    return bytes(body)


def frame_rotation(step_deg=30.0, distance_mm=1500, points=None):
    """生成覆盖整圈的帧序列。每帧跨度 = step_deg，帧起点间隔 step_deg 度。"""
    frames = []
    n = int(math.ceil(360.0 / step_deg))
    for i in range(n):
        start = i * step_deg
        pts = points if points is not None else [distance_mm] * 12
        # 每帧跨满 step_deg，起点间隔也是 step_deg，整圈无缝拼接
        frames.append(build_frame(start_ddeg=int(round(start * 100)),
                                  end_ddeg=int(round(start * 100))
                                  + int(round(step_deg * 100)),
                                  points=pts))
    return frames


class Clock:
    """手工推进的时钟，单位纳秒。"""

    def __init__(self, start_ns=1_000_000_000):
        self.ns = start_ns

    def advance(self, seconds):
        self.ns += int(round(seconds * 1e9))

    def now(self):
        return types.SimpleNamespace(nanoseconds=self.ns)

    @property
    def seconds(self):
        return self.ns / 1e9


_MONO_SOURCE = [1000.0]


def monotonic():
    """reader 用的单调时钟替身：与 rospy 逻辑时钟解耦，可单独推进。"""
    return _MONO_SOURCE[0]


def set_monotonic(seconds):
    _MONO_SOURCE[0] = float(seconds)


def advance(seconds):
    """推进单调时钟替身。"""
    _MONO_SOURCE[0] += float(seconds)


def approx(value, target, tol=1e-6):
    return abs(value - target) <= tol
