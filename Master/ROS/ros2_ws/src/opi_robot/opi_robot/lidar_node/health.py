"""雷达健康状态机。

纯 Python，不依赖 ROS，可用假时钟直接测试。

四个对外状态：

- ``connecting``    串口已打开，但还没拿到足够覆盖的新扫描。
- ``healthy``       有新鲜有效帧，且整圈覆盖达到阈值。
- ``stale``         最后一次 CRC 正确帧已超时，覆盖累计已作废。
- ``disconnected``  串口打不开、读取出错，或设备文件消失。

状态原因字符串（``reason``）供诊断消息与网页直接显示。
"""

import math

from . import geometry

wrap_deg = geometry.wrap_deg
wrap_rad = geometry.wrap_rad

CONNECTING = 'connecting'
HEALTHY = 'healthy'
STALE = 'stale'
DISCONNECTED = 'disconnected'

ALL_STATES = (CONNECTING, HEALTHY, STALE, DISCONNECTED)

# diagnostic_msgs/DiagnosticStatus 等级
LEVEL_OK = 0
LEVEL_WARN = 1
LEVEL_ERROR = 2
LEVEL_STALE = 3

# 覆盖判定分辨率：全圆 360 度分 72 格，每格 5 度
COVER_SLOTS = 72
# 覆盖只在最近 N 帧内有效。必须大于一圈所需帧数，否则永远凑不满整圈：
# LD19 实测单帧跨度中位数 7.85 度（2026-11-09 板上直读，1268 帧），
# 一圈约 46 帧；取 60 留约 30% 余量，恢复延迟约 1.3 秒。
# 原值 24 只覆盖半圈，导致 coverage 卡在 0.58、永不 healthy。
COVER_WINDOW = 60


def diagnostic_level(state):
    """状态到诊断等级。"""
    if state == HEALTHY:
        return LEVEL_OK
    if state == CONNECTING:
        return LEVEL_WARN
    return LEVEL_ERROR


# 防止浮点误差把 15.000000000000002 算成下一个分箱
_EPS = 1e-6


def span_slots(start_deg, width_deg, slots):
    """一次扫描覆盖的全部分箱号。

    width_deg 为扫描宽度，0 到 360，单位度。起点和终点分箱都算在内。
    结果按角度从小到大列出，只在 360 度回绕时绕回 0。
    """
    slot_deg = 360.0 / slots
    a = wrap_deg(start_deg)
    start = int(math.floor(a / slot_deg + _EPS)) % slots
    end = int(math.floor((a + width_deg) / slot_deg + _EPS)) % slots
    n = (end - start) % slots
    return [(start + k) % slots for k in range(n + 1)]

class HealthMonitor:
    """雷达链路状态机。"""

    def __init__(self, frame_timeout, recovery_coverage=0.9,
                 slots=COVER_SLOTS, mirrored=True):
        self.frame_timeout = float(frame_timeout)
        self.recovery_coverage = float(recovery_coverage)
        self.slots = int(slots)
        self.mirrored = bool(mirrored)
        self._slot_deg = 360.0 / self.slots

        self.state = CONNECTING
        self.reason = 'startup'
        self.last_error = ''
        self.last_frame_t = None
        self.open_t = None
        self.frames_ok = 0
        self.frames_bad = 0
        self.bytes_dropped = 0
        self.reconnects = 0
        self._seen_open = False

        self._slot_frame = {}
        self._frame_no = 0

    # ---------- 时间 ----------

    def age(self, now):
        """距最后一次有效帧的秒数；无有效帧时为 inf。"""
        if self.last_frame_t is None:
            return float('inf')
        return max(0.0, now - self.last_frame_t)

    def is_fresh(self, now):
        """最后一次有效帧是否在超时之内。"""
        return self.age(now) < self.frame_timeout

    def uptime(self, now):
        """自本次打开端口以来的秒数；未打开时为 None。"""
        if self.open_t is None:
            return None
        return max(0.0, now - self.open_t)

    # ---------- 覆盖 ----------

    def mark_span(self, start_scan_deg, end_scan_deg):
        """按 LaserScan 角（度，0 到 360）标记一次扫描覆盖。

        帧宽取 (end - start) mod 360，由 span_slots 依序展开成整段分箱。
        ``coverage`` 按分箱求和。
        """
        self._frame_no += 1
        width = (wrap_deg(end_scan_deg) - wrap_deg(start_scan_deg)) % 360.0
        for slot in span_slots(start_scan_deg, width, self.slots):
            self._slot_frame[slot] = self._frame_no

    def coverage(self):
        """覆盖比例，0 到 1。只看最近 COVER_WINDOW 次扫描。"""
        if not self._slot_frame:
            return 0.0
        newest = self._frame_no
        hit = sum(1 for f in self._slot_frame.values()
                  if newest - f < COVER_WINDOW)
        return hit / float(self.slots)

    def reset_coverage(self):
        """清空覆盖累计。断线、重新打开、超时、失败都必须调用。"""
        self._slot_frame = {}
        self._frame_no = 0

    def coverage_view(self):
        """当前覆盖与帧号，用于排查。"""
        return len(self._slot_frame), self._frame_no

    # ---------- 事件 ----------

    def on_open_failure(self, now, error):
        """打开串口失败。"""
        self.last_error = error
        self.state = DISCONNECTED
        self.reason = 'open_error'
        self.last_frame_t = None
        self.open_t = None
        self.reset_coverage()

    def on_open_success(self, now):
        """串口打开成功。从断开状态重新连上时重连计数加一。

        进程启动后的第一次打开不计入重连。
        """
        was_disconnected = self.state == DISCONNECTED
        self.state = CONNECTING
        self.reason = 'awaiting_frame'
        self.open_t = now
        self.last_frame_t = None
        self.reset_coverage()
        if was_disconnected and self._seen_open:
            self.reconnects += 1
        self._seen_open = True

    def on_read_error(self, now, error):
        """读取出错：串口语义已坏，必须断开重开。"""
        self.last_error = error
        self.state = DISCONNECTED
        self.reason = 'read_error'
        self.last_frame_t = None
        self.open_t = None
        self.reset_coverage()

    def on_frame_timeout(self, now, error=None):
        """帧超时：串口仍在，但没有新数据。"""
        if error:
            self.last_error = error
        self.state = STALE
        self.reason = 'frame_timeout'
        self.reset_coverage()

    def on_device_missing(self, now):
        """设备文件不存在。"""
        self.state = DISCONNECTED
        self.reason = 'device_missing'
        self.last_frame_t = None
        self.open_t = None
        self.reset_coverage()

    def on_silence(self, now):
        """本轮没有解析出有效帧。

        还没拿到首帧时属于正常启动；已拿到首帧但帧已超时，则作废覆盖。
        只有在 healthy 时帧超时要保留由 tick 处理，这里不重复。
        """
        if self.state == STALE and not self.is_fresh(now):
            self.reset_coverage()

    def on_bad_frame(self, now, reason='crc'):
        self.frames_bad += 1
        self.last_error = reason

    def on_dropped_bytes(self, now, count):
        self.bytes_dropped += int(count)

    def on_scan(self, now, start_scan_deg, width_deg):
        """收到一次有效扫描。

        start_scan_deg 是这一帧在 LaserScan 角空间的起点，width_deg 是跨度。
        恢复条件：从 connecting 或 stale 出发，覆盖率达到阈值才进入
        healthy。覆盖累计在每次失败时清空，所以恢复必须来自新数据。
        已经在 healthy 时只刷新时间戳，不再要求一次扫描覆盖整圈。
        """
        self.frames_ok += 1
        recovery = self.state in (CONNECTING, STALE)
        self.mark_span(wrap_deg(start_scan_deg - width_deg),
                       wrap_deg(start_scan_deg))
        self.last_frame_t = now

        if recovery:
            if self.coverage() >= self.recovery_coverage:
                self.state = HEALTHY
                self.reason = 'ok'
            else:
                self.state = STALE
                self.reason = 'recovering'

    def tick(self, now):
        """周期检查：帧已超时就进入 stale 并作废覆盖。

        适用在 healthy 与 recovering 两种状态：两者都曾见过有效帧。
        connecting 尚未收到任何帧，超时由打开流程自己处理。
        """
        if self.state in (HEALTHY, STALE) and not self.is_fresh(now):
            self.state = STALE
            self.reason = 'frame_timeout'
            self.reset_coverage()

    def expire_if_stale(self, now):
        """返回超时是否刚刚被本函数确认。调用方据此释放串口。

        同一超时只应处理一次；重复调用返回 False。
        """
        if self.state in (HEALTHY, STALE) and not self.is_fresh(now):
            self.state = STALE
            self.reason = 'frame_timeout'
            self.reset_coverage()
            self.open_t = None
            return True
        return False

    def is_healthy(self, now):
        return self.state == HEALTHY and self.is_fresh(now)

    def snapshot(self, now, port=''):
        """诊断字段快照。age_s 在无有效帧时为 None。"""
        age = self.age(now)
        return {
            'state': self.state,
            'reason': self.reason,
            'age_s': None if age == float('inf') else round(age, 3),
            'frames_ok': self.frames_ok,
            'frames_bad': self.frames_bad,
            'bytes_dropped': self.bytes_dropped,
            'reconnects': self.reconnects,
            'coverage': round(self.coverage(), 3),
            'last_error': self.last_error,
            'port': port,
            'uptime_s': self.uptime(now),
        }


class ReconnectThrottle:
    """限制重连频率。"""

    def __init__(self, min_interval):
        self.min_interval = float(min_interval)
        self.last_t = None

    def wait(self, now):
        if self.last_t is None:
            return 0.0
        return max(0.0, self.min_interval - (now - self.last_t))

    def allow(self, now):
        return self.wait(now) <= 0.0

    def record(self, now):
        self.last_t = now

    def reset(self):
        self.last_t = None


class LogLimiter:
    """同一原因只打印一次；周期汇总单独节流。"""

    def __init__(self):
        self.last_key = None
        self.last_summary = None

    def should_log(self, key):
        if self.last_key == key:
            return False
        self.last_key = key
        return True

    def should_summary(self, now, interval):
        if self.last_summary is None or (now - self.last_summary) >= interval:
            self.last_summary = now
            return True
        return False
