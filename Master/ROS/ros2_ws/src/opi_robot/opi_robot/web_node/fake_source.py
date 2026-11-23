"""离线假数据源：不接任何硬件的合成状态帧、雷达扫描与摄像头图像。

用途：
- 页面演示与截图取证（不碰 /dev/tty*、不碰摄像头与雷达）。
- 前端渲染回归：合成数据覆盖正常值、零值、失效值。

设计约束：
- 只用标准库（``zlib``、``struct``、``math``）。
- 输出结构与真实数据路径完全一致，网页无法区分来源。
"""
from __future__ import annotations

import math
import struct
import zlib

# 一条覆盖直行、转向、停车、电压下坠的演示轨迹（周期 24 s）
CYCLE_S = 24.0

# 假房间尺寸，单位 m。雷达原点在房间中心偏后。
ROOM_W_M = 6.4
ROOM_H_M = 4.6
POLE_X_M = 1.9
POLE_Y_M = 0.7
POLE_R_M = 0.22

# 合成画面尺寸。摄像头面板与测试都引用这两个常量。
CAM_W = 384
CAM_H = 216


def png_bytes(width, height, rows):
    """把 RGB 行数据编码成 PNG（标准库，无第三方依赖）。"""
    raw = b''.join(b'\x00' + bytes(row) for row in rows)

    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))

    ihdr = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr)
            + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


def _sky_rows(width, height):
    rows = []
    horizon = int(height * 0.42)
    for y in range(height):
        row = bytearray()
        if y < horizon:
            t = y / max(1, horizon)
            r = int(18 + 30 * t)
            g = int(24 + 40 * t)
            b = int(38 + 62 * t)
            row += bytes((r, g, b)) * width
        else:
            t = (y - horizon) / max(1, height - horizon)
            base = int(38 + 46 * t)
            for x in range(width):
                # 透视地面网格：横线按 t 压缩，竖线向灭点收拢
                grid = 0
                if (y - horizon) % max(3, int(26 * (1.0 - t * 0.8))) == 0:
                    grid = 1
                cx = width / 2.0
                dx = (x - cx) / max(1.0, (y - horizon + 1) * 0.5)
                if abs(dx - round(dx)) < 0.055:
                    grid = 1
                v = base + (34 if grid else 0)
                row += bytes((v - 6, v, v + 8))
        rows.append(row)
    return rows


def camera_frame(width=CAM_W, height=CAM_H, phase=0.0,
                 target=True, horizon_ratio=0.42):
    """合成一帧"摄像头画面"：透视地面 + 移动目标块 + 取景框。"""
    rows = _sky_rows(width, height)
    horizon = int(height * 0.42)
    if target:
        # 目标块随相位左右移动，模拟场景里有人或箱子
        cx = int(width * (0.5 + 0.32 * math.sin(phase)))
        w = max(10, width // 12)
        h = max(16, height // 4)
        top = horizon - h + int(6 * math.sin(phase * 2.0))
        for y in range(max(0, top), min(height, top + h)):
            row = rows[y]
            for x in range(max(0, cx - w // 2), min(width, cx + w // 2)):
                shade = 200 - int(40 * (y - top) / max(1, h))
                row[x * 3:x * 3 + 3] = bytes((shade, 60, 52))
    # 取景十字与安全框
    cx = width // 2
    cy = int(height * horizon_ratio)
    for x in range(width):
        rows[cy][x * 3:x * 3 + 3] = bytes((77, 163, 255))
    for y in range(height):
        rows[y][cx * 3:cx * 3 + 3] = bytes((77, 163, 255))
    return png_bytes(width, height, rows)


def scan_points(phase=0.0, n=360):
    """合成 360 点扫描：矩形房间墙 + 一根柱子，带少量噪声。

    返回 ``[[deg, range], ...]``，与真实 scan 载荷同构。
    """
    half_w = ROOM_W_M / 2.0
    half_h = ROOM_H_M / 2.0
    pts = []
    for i in range(n):
        deg = i * 360.0 / n
        a = math.radians(deg)
        dx, dy = math.cos(a), math.sin(a)
        # 与四面墙求交，取最近正交点
        best = float('inf')
        for num, den in ((half_w, dx), (-half_w, dx),
                         (half_h, dy), (-half_h, dy)):
            if abs(den) < 1e-9:
                continue
            t = num / den
            if t > 0:
                best = min(best, t)
        # 与圆柱求交
        ox, oy = POLE_X_M, POLE_Y_M
        b = 2.0 * (dx * ox + dy * oy)
        c = ox * ox + oy * oy - POLE_R_M * POLE_R_M
        disc = b * b - 4.0 * c
        if disc > 0:
            t = (-b - math.sqrt(disc)) / 2.0
            if t > 0:
                best = min(best, t)
        rng = best if math.isfinite(best) else 0.0
        # 周期性遮挡，模拟真实雷达的点丢失
        if (i + int(phase * 12)) % 97 < 3:
            rng = 0.0
        if rng:
            rng = max(0.12, rng + 0.02 * math.sin(i * 1.7 + phase * 3.0))
        pts.append([round(deg, 2), round(rng, 3)])
    return pts


def chassis_status(phase=0.0):
    """合成一条底盘状态（字段与诊断键一致）。"""
    cycle = phase % CYCLE_S
    # 0-8 s 直行加速，8-14 s 匀速，14-18 s 转向，18-24 s 停车保持
    if cycle < 8.0:
        vx = 0.42 * (cycle / 8.0)
        wz = 0.0
        stop = 0
    elif cycle < 14.0:
        vx = 0.42
        wz = 0.0
        stop = 0
    elif cycle < 18.0:
        k = (cycle - 14.0) / 4.0
        vx = 0.42 * (1.0 - k)
        wz = 0.55 * math.sin(k * math.pi)
        stop = 0
    else:
        vx = 0.0
        wz = 0.0
        stop = 1
    # 电压随负载缓慢下坠，停车时回升
    load = abs(vx) / 0.42
    voltage = 12.24 - 0.34 * load - 0.05 * math.sin(phase * 0.7)
    if stop:
        voltage = 12.26 - 0.03 * math.sin(phase * 0.7)
    return {
        'voltage_mv': int(round(voltage * 1000)),
        'vx_mm_s': int(round(vx * 1000)),
        'vy_mm_s': 0,
        'yaw_rate_mrad_s': int(round(wz * 1000)),
        'stop_flag': stop,
        'level': 0,
        'message': 'synthetic',
    }


def imu_reading(phase=0.0):
    """合成 IMU 角速度（rad/s），供心跳与曲线使用。"""
    return {
        'wz': 0.08 * math.sin(phase * 3.1) + 0.01 * math.sin(phase * 11.0),
        'ax': 0.02 * math.sin(phase * 1.3),
    }


class FakeSource:
    """按时间步进产生合成数据。线程安全由调用方保证。"""

    def __init__(self, image_period=0.2, t0=0.0):
        self._t0 = t0
        self._image_period = image_period
        self._last_image = -1e9
        self._frame = None
        self._last_status = -1e9

    def phase(self, now):
        return max(0.0, now - self._t0)

    def reset(self, t0):
        self._t0 = t0
        self._last_image = -1e9
        self._frame = None

    def status(self, now):
        """返回合成底盘状态；调用方自行按 20 Hz 节流。"""
        return chassis_status(self.phase(now))

    def scan(self, now):
        return scan_points(self.phase(now))

    def image(self, now):
        """按周期生成 PNG；未到周期返回 None。"""
        if now - self._last_image < self._image_period:
            return None
        self._last_image = now
        self._frame = camera_frame(phase=self.phase(now))
        return self._frame
