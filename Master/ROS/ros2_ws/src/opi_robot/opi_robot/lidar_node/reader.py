"""雷达读取协调层。

把串口打开、帧解码、扫描聚合、健康判定串起来。
本模块不依赖 ROS，用假时钟与伪串口即可测试。

发布规则：只有 ``monitor.is_healthy(now)`` 为真时才返回扫描。
超时、断开、覆盖不足一律返回 None，调用方不得发布旧测距。

扫描聚合规则（按完整旋转）：
- 新扫到角度时清除该格旧值，同一格的有效点覆盖旧值。
- 超过 ``stale_point_age`` 秒没有新数据的格子输出 inf，近目标移开后点会消失。
- 时间用 ``time.monotonic()``，不依赖 ROS 时钟。
"""

import math
import time

from . import codec as C
from . import health as H
from . import geometry as G

BOOT_DELAY_S = 0.0
DEFAULT_STALE_POINT_AGE = 0.5


class LidarRunner:
    """一次性完成读串口、解帧、判健康、产出 LaserScan。"""

    def __init__(self, port, baud, frame_timeout, recovery_coverage,
                 reconnect_interval, log_interval, mirror, clock,
                 open_port, path_exists,
                 bins=360, angle_offset_deg=0.0, origin='mirror',
                 range_min=0.02, range_max=12.0,
                 stale_point_age=DEFAULT_STALE_POINT_AGE,
                 mono=None):
        self.port = port
        self.baud = int(baud)
        self.log_interval = float(log_interval)
        self.clock = clock
        # 单调时钟注入点。未注入时必须在使用时查 ``time.monotonic``：
        # 测试会整体替换 reader.time，构造期取值会把真实时钟固化进来
        self._mono_injected = mono
        self.open_port = open_port
        self.path_exists = path_exists
        # 单格测距最大保留时长（秒）；0 及以下表示不回退旧值，只保留本圈
        self.stale_point_age = float(stale_point_age)
        self.layout = G.ScanLayout(bins=bins,
                                   angle_offset_deg=angle_offset_deg,
                                   origin=origin,
                                   range_min=range_min,
                                   range_max=range_max)
        self.monitor = H.HealthMonitor(frame_timeout=frame_timeout,
                                       recovery_coverage=recovery_coverage)
        self.throttle = H.ReconnectThrottle(reconnect_interval)
        self.limiter = H.LogLimiter()

        self.serial = None
        self.decoder = C.FrameDecoder(mirs=False)
        self.ring = [math.inf] * self.layout.n_bins
        # ring_time[i] 是 ring[i] 的收到时刻，单位秒，取自单调时钟
        self.ring_time = [None] * self.layout.n_bins
        self._last_bin = None
        self.open_attempts = 0
        self.last_state = None
        self._read_frames = 0
        self._last_frames = []
        # 每帧的 (start, width) 缓存，避免每轮重复算三角
        self._frame_span_cache = {}
        self._last_poll_t = 0.0
        self.read_size = 4096
        self._log = None

    # ---------- 时间 ----------

    def now(self):
        """状态机与超时判定统一使用单调时钟。

        历史上这里用 ROS 时钟（``self.clock()``）。ROS 时钟可能是仿真时间，
        也会被 NTP 跳变影响；一旦回跳，帧龄计算变成负数，超时永远不会触发，
        旧测距就会一直被判成"新鲜"。超时语义必须与墙上时间解耦。
        """
        return self._mono()

    def _mono(self):
        """单调时钟秒。状态机、超时与测距过期判定只用这个时间。

        未注入时每次现查 ``time.monotonic``，保证替换 reader.time 的测试生效。
        """
        if self._mono_injected is not None:
            return self._mono_injected()
        return time.monotonic()

    def _clear_ring(self):
        self.ring = [math.inf] * self.layout.n_bins
        self.ring_time = [None] * self.layout.n_bins
        self._last_bin = None

    def _ring_max_age(self):
        return self.stale_point_age

    # ---------- 端口 ----------

    def _close_port(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
        self._clear_ring()
        self.decoder = C.FrameDecoder(mirs=False)

    def _try_open(self, now):
        """按节流规则尝试打开。返回是否已连接。"""
        if self.serial is not None:
            return True
        if not self.throttle.allow(now):
            return False
        if not self.path_exists(self.port):
            # 设备文件不在也算一次打开尝试，节流计数要如实反映
            self.open_attempts += 1
            self.monitor.on_device_missing(now)
            self.throttle.record(now)
            self._log_once('device_missing:%s' % self.port,
                           'error', '雷达串口不存在: %s' % self.port)
            return False
        self.open_attempts += 1
        self.throttle.record(now)
        try:
            serial = self.open_port(self.port, self.baud)
        except Exception as exc:  # 打开失败一律落回断开，不让异常打穿轮转
            self.monitor.on_open_failure(now, str(exc))
            self._log_once('open_error:%s' % exc, 'error',
                           '雷达串口打开失败: %s (%s)' % (self.port, exc))
            return False
        self.serial = serial
        self.monitor.on_open_success(now)
        self._log_once('open_ok:%s' % self.port, 'info',
                       '雷达串口已打开: %s @ %d' % (self.port, self.baud))
        return True

    # ---------- 读取与解析 ----------

    def _read(self, now):
        """读一次串口并解帧。返回本轮解出的有效帧数。"""
        self.decoder.last_frames = 0
        self._read_frames = 0
        self._last_frames = []
        try:
            # 只阻塞极短时间：没有新数据就返回，不让轮转卡住
            chunk = self.serial.read(self.read_size)
        except OSError as exc:
            self._close_port()
            self.monitor.on_read_error(now, str(exc))
            self._log_once('read_error:%s' % exc, 'error',
                           '雷达串口读取失败: %s，已关闭并准备重连' % exc)
            return 0
        if not chunk:
            return 0
        before = self.decoder.bytes_dropped
        frames = self.decoder.feed(chunk)
        dropped = self.decoder.bytes_dropped - before
        if dropped:
            self.monitor.on_dropped_bytes(now, dropped)
        if self.decoder.frames_bad:
            bad = self.decoder.frames_bad
            self.decoder.frames_bad = 0
            for _ in range(bad):
                self.monitor.on_bad_frame(now, 'crc')
        for frame in frames:
            self._apply_frame(frame)
        self._last_frames = list(frames)
        self._read_frames = len(frames)
        return self._read_frames

    def _apply_frame(self, frame):
        """把一个帧的点写进环形数组。

        进入新分箱时清除旧值；同一分箱可能横跨多个帧，零距离点不覆盖
        本次经过该分箱时收到的有效点。时间戳用单调时钟，整帧共用一个时刻。
        """
        t = self._mono()
        for pt in frame.points:
            scan_deg = self.layout.raw_deg_to_scan_deg(pt.angle_deg)
            idx = self.layout.index_for_scan_deg(scan_deg)
            if idx != self._last_bin:
                self.ring[idx] = math.inf
                self.ring_time[idx] = None
                self._last_bin = idx
            if pt.dist_m > 0.0:
                self.ring[idx] = pt.dist_m
                self.ring_time[idx] = t
        return t

    def _frame_span(self, frame):
        """一帧在扫描角空间的跨度。

        on_scan 内部按 mark_span(wrap(start - width), wrap(start)) 扩展，
        所以这里给的是扫描角空间里的高角端（原始起点在镜像下映射到高角端）。
        """
        key = (frame.start_deg, frame.end_deg, self.decoder.mirs)
        hit = self._frame_span_cache.get(key)
        if hit is not None:
            return hit
        width = self.layout.frame_span_deg(
            frame.start_deg, frame.end_deg, mirs=self.decoder.mirs)
        start = self.layout.raw_deg_to_scan_deg(frame.start_deg)
        if len(self._frame_span_cache) > 4096:
            self._frame_span_cache.clear()
        self._frame_span_cache[key] = (start, width)
        return start, width

    # ---------- 发布判定 ----------

    def _build_scan(self, now_mono=None):
        """把环形数组转成 LaserScan 字段字典。

        超过 stale_point_age 没有新数据的格、越界值和 NaN 一律记为 inf。
        ``max_point_age_s`` 是本帧所有**有效**测距值里最旧一个的年龄
        （单调时钟），供原始帧证据话题透出；一帧有效点都没有时为 None。
        LaserScan 消息本身没有逐点年龄字段，这是发布侧能给出的
        方向无关上界证据。
        """
        now_mono = self._mono() if now_mono is None else now_mono
        max_keep = self._ring_max_age()
        ranges = [math.inf] * self.layout.n_bins
        oldest_valid = None
        for i, r in enumerate(self.ring):
            t = self.ring_time[i]
            if t is None or (now_mono - t) > max_keep:
                continue
            # 超出量程的点按无效处理（inf），不塞成 0 或量程上限
            if r != r or r < self.layout.range_min or r > self.layout.range_max:
                continue
            ranges[i] = r
            age = now_mono - t
            if oldest_valid is None or age > oldest_valid:
                oldest_valid = age
        return {
            'ranges': ranges,
            'angle_min': self.layout.angle_min,
            'angle_max': self.layout.angle_max,
            'angle_increment': self.layout.angle_increment,
            'range_min': self.layout.range_min,
            'range_max': self.layout.range_max,
            'max_point_age_s': oldest_valid,
        }

    def poll(self, log=None):
        """读取一次。健康时返回 LaserScan 字段，否则返回 None。

        每轮只取一次时间，用它判新鲜度，避免边界上前后不一致。
        """
        self._log = log
        now = self.now()
        self._last_poll_t = now
        if not self._try_open(now):
            self.monitor.tick(now)
            self._watch_state(now)
            self._emit_summary(now)
            return None

        self._read(now)
        now = self.now()
        if self.serial is None:
            # 本轮到读取出错，已经在 _read 里断开
            self._watch_state(now)
            self._emit_summary(now)
            return None

        if self._read_frames:
            # 每个协议帧都单独计数与累计覆盖，不能只送最后一帧
            for frame in self._last_frames:
                a, width = self._frame_span(frame)
                self.monitor.on_scan(now, a, width)
        else:
            # 本轮没有新帧：只做超时判定，不动覆盖累计
            self.monitor.tick(now)

        result = None
        if self.monitor.is_healthy(now):
            result = self._build_scan()

        # tick 已负责把超时状态写成 stale；这里只做一次副作用：
        # 释放串口并清空 ring，避免旧测距被新时间戳伪装成新鲜数据
        if self.monitor.expire_if_stale(now):
            self._close_port()
            self._log_once('frame_timeout', 'error',
                           '雷达无有效帧超过 %.2fs，已释放串口'
                           % self.monitor.frame_timeout)
            result = None

        self._watch_state(now)
        self._emit_summary(now)
        return result

    def _watch_state(self, now):
        """状态迁移时打一行日志，避免每轮刷屏。"""
        state = self.monitor.state
        if state == self.last_state:
            return
        self.last_state = state
        if state == H.HEALTHY:
            self._log_once('state:healthy:%d' % self.monitor.reconnects,
                           'info',
                           '雷达状态: healthy，重连 %d 次'
                           % self.monitor.reconnects)
        elif state == H.DISCONNECTED:
            self._log_once('state:disconnected:%s' % self.monitor.reason,
                           'error',
                           '雷达状态: disconnected，原因 %s'
                           % self.monitor.reason)

    def reset_coverage(self):
        """测试与恢复用：清空覆盖累计。"""
        self.monitor.reset_coverage()

    def _emit_summary(self, now):
        if not self.limiter.should_summary(now, self.log_interval):
            return
        snap = self.snapshot()
        msg = ('雷达统计: 状态=%s 原因=%s 帧龄=%s 好帧=%d 坏帧=%d '
               '丢字节=%d 覆盖=%.2f 重连=%d'
               % (snap['state'], snap['reason'],
                  'n/a' if snap['age_s'] is None else '%.2fs' % snap['age_s'],
                  snap['frames_ok'], snap['frames_bad'],
                  snap['bytes_dropped'], snap['coverage'],
                  snap['reconnects']))
        self._log_once('summary', 'info', msg)

    def _log_once(self, key, level, text):
        if not self.limiter.should_log(key):
            return
        sink = self._log
        if sink is None:
            return
        if callable(sink):
            sink(text)
        else:
            sink.append(text)

    # ---------- 诊断 ----------

    def snapshot(self):
        now = self.now()
        snap = self.monitor.snapshot(now, port=self.port)
        snap['origin'] = self.layout.origin
        snap['baud'] = self.baud
        snap['connected'] = self.serial is not None
        snap['open_attempts'] = self.open_attempts
        snap['bins'] = self.layout.n_bins
        return snap
