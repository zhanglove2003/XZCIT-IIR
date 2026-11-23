"""LD19 帧解码。

只做字节流到帧对象的转换，不碰串口、不碰 ROS，便于单测。

帧格式（47 字节）::

    0x54 0x2C | speed(2, 0.01 度/秒) | start_deg(2, 0.01 度)
    | 12 x [dist(2, mm) + intensity(1)]
    | end_deg(2, 0.01 度) | timestamp(2) | crc8(1)
"""

import struct

# 与现场 lidar_node.py 完全相同的 LD19 CRC8 查表，逐字节比对过
CRC_TABLE = bytes.fromhex(
    "004d9ad77934e3aef2bf68258bc6115ca9e4337ed09d4a075b16c18c226fb8f5"
    "1f5285c8662bfcb1eda0773a94d90e43b6fb2c61cf8255184409de933d70a7ea"
    "3e73a4e9470add90cc81561bb5f82f6297da0d40eea374396528ffb21c5186cb"
    "216cbbf65815c28fd39e4904aae7307d88c5125ff1bc6b267a37e0ad034e99d4"
    "7c31e6ab05489fd28ec31459f7ba6d20d5984f02ace1367b276abdf05e13c489"
    "632ef9b41a5780cd91dc0b46e8a5723fca87501db3fe29643875a2ef410cdb96"
    "420fd8953b76a1ecb0fd2a67c984531eeba6713c92df0845195483ce602dfab7"
    "5d10c78a2469bef3afe23578d69b4c01f4b96e238dc0175a064b9cd17f32e5a8"
)
FRAME_LEN = 47
HEADER0 = 0x54
HEADER1 = 0x2C
MAX_BUFFER_DEFAULT = 4096
MAX_SPAN_DDEG = 4000      # 单帧角度跨度上限 40 度，超出视为解析不可信


def crc8(data):
    """按 LD19 查表算法计算 CRC8。"""
    c = 0
    for b in data:
        c = CRC_TABLE[(c ^ b) & 0xFF]
    return c


class Point:
    """一帧内的一个点。angle_deg 为雷达原始角，dist_m 为米。"""

    __slots__ = ('angle_deg', 'dist_m', 'intensity')

    def __init__(self, angle_deg, dist_m, intensity):
        self.angle_deg = angle_deg
        self.dist_m = dist_m
        self.intensity = intensity

    def __repr__(self):
        return 'Point(%.2f deg, %.3f m)' % (self.angle_deg, self.dist_m)


class Frame:
    """一帧解析结果。"""

    __slots__ = ('speed_ddeg', 'start_deg', 'end_deg', 'points')

    def __init__(self, speed_ddeg, start_deg, end_deg, points):
        self.speed_ddeg = speed_ddeg
        self.start_deg = start_deg
        self.end_deg = end_deg
        self.points = points

    @property
    def span_deg(self):
        return (self.end_deg - self.start_deg) % 360.0


class FrameDecoder:
    """字节流解码器。"""

    def __init__(self, mirs=False, max_buffer=MAX_BUFFER_DEFAULT):
        self.mirs = bool(mirs)
        self.max_buffer = int(max_buffer)
        self.pending = bytearray()
        self.frames_ok = 0
        self.frames_bad = 0
        self.bytes_dropped = 0
        self.last_frames = 0

    def feed(self, chunk):
        """喂入字节，返回本次解出的帧列表。"""
        self.last_frames = 0
        if chunk:
            self.pending.extend(chunk)
        frames = []
        pos = 0
        # 缓冲超过上限时，先裁掉不可能属于完整帧的旧字节
        over = len(self.pending) - self.max_buffer
        if over > 0:
            self.bytes_dropped += over
            del self.pending[:over]
        n = len(self.pending)
        while pos + FRAME_LEN <= n:
            if (self.pending[pos] != HEADER0
                    or self.pending[pos + 1] != HEADER1):
                pos += 1
                continue
            raw = bytes(self.pending[pos:pos + FRAME_LEN])
            if crc8(raw[:FRAME_LEN - 1]) != raw[FRAME_LEN - 1]:
                self.frames_bad += 1
                pos += 1
                continue
            frame = self._decode(raw)
            if frame is None:
                self.frames_bad += 1
                pos += 1
                continue
            self.frames_ok += 1
            self.last_frames += 1
            frames.append(frame)
            pos += FRAME_LEN

        if pos:
            del self.pending[:pos]
        self._trim()
        return frames

    def _trim(self):
        """缓冲区超限时，从帧头开始保留尾部，丢弃杂字节并计数。"""
        if len(self.pending) <= self.max_buffer:
            return
        keep = self.max_buffer
        hidden = len(self.pending) - keep
        idx = self.pending.find(bytes([HEADER0, HEADER1]))
        if idx < 0 or idx > hidden:
            # 帧头在保留窗口之前，或根本没有帧头：从窗口起点裁
            idx = hidden
        self.bytes_dropped += idx
        del self.pending[:idx]

    def _decode(self, raw):
        speed = struct.unpack('<H', raw[2:4])[0]
        start = struct.unpack('<H', raw[4:6])[0]
        end = struct.unpack('<H', raw[42:44])[0]
        if start >= 36000 or end >= 36000:
            return None
        span = (end - start) % 36000
        # 协议帧角跨度不会超过 40 度；0 或过大都不可信
        if span == 0 or span > MAX_SPAN_DDEG:
            return None
        step = span / 11.0
        points = []
        for k in range(12):
            dist_mm = struct.unpack('<H', raw[6 + 3 * k:8 + 3 * k])[0]
            angle = (start + step * k) % 36000 / 100.0
            points.append(Point(angle, dist_mm / 1000.0, raw[8 + 3 * k]))
        if self.mirs:
            points.reverse()
        return Frame(speed / 100.0, start / 100.0, end / 100.0, points)
