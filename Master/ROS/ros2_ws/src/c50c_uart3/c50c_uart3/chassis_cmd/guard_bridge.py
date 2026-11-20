"""运动许可桥：MotionGuard 决策 → 基线 11 字节帧 → 所有者 fd，三分离记录。

ROS 接线全部在这里（本文件可以被 import 时不加载 rclpy：订阅由显式
``attach`` 创建）；纯逻辑判定在 ``c50c_uart3.motion_guard``。

每拍（``tick``）：

1. 可选 ``preconditions``：网页会话鉴权 + 显式 rearm 请求；
2. ``MotionGuard.decide_step`` 得到 ``(vx, vy, vz, source, zero_reason)``；
3. 只有**同时**满足以下条件才写非零帧：

   - 存在真实串口端口（``port.is_real`` 为真，即 ``FdPort`` 挂真实 UART；
     ``FakeFdPort`` 不算）；
   - ``drive_enable`` 为真；
   - ``MotionGuard`` 已 rearm 且本拍没有零速原因（``rearm`` 前提：网页会话
     鉴权 + 显式解停请求，两者缺一即拒绝；堵住"先 rearm 再续期"绕过前提）；
   - 底盘回传证据（状态帧）在时效内——没有接收方向证据时即使已 rearm
     也只发零速帧，避免"本地以为在走"。

   否则写基线零速帧（有端口时）或什么都不写（没有端口）。

三件事分开记录（``write_state`` / ``WriteRecord`` / ``counters``）：

- ``zero_issued``：本拍决定发出零速 / 非零（"发出零速"）；
- ``serial_write_ok``：``port.write_bytes`` 是否完整写入 11 字节
  （"串口写入成功"，**不**等于车轮动/停）；
- ``vehicle_stopped``：由底盘回传速度（``observe_status``）判定车辆是否
  实际停住，只能由回传证据置真；没有证据时为 ``None``（不可证）。

基线帧编码复用 ``baseline.encode_baseline_speed_frame`` /
``encode_baseline_zero_frame``（11 字节、``frame[2]=0``）。
"""
from __future__ import annotations

import json
import time

from ..motion_guard import (CHANNELS, License, MotionGuard, STOP_CAUSE,
                            ZERO_REASON, direction_sectors, motion_check,
                            scan_check, scan_stamp_to_mono, ultrasonic_state)
from .baseline import (encode_baseline_speed_frame, encode_baseline_zero_frame,
                       FRAME_LENGTH)

#: 零速原因 → 锁停原因（WriteRecord 对外只给一个"为什么停"）。
REASON_TO_CAUSE = {
    'license_expired': 'license_expired',
    'license_rejected': 'license_rejected',
    'ultrasonic_invalid': 'ultrasonic_invalid',
    'ultrasonic_stale': 'ultrasonic_stale',
    'scan_stale': 'scan_stale',
    'scan_blocked': 'scan_blocked',
    'global_stop': 'global_stop',
    'serial_reconnect': 'serial_reconnect',
    'rearm_pending': 'rearm_pending',
    'rearm_required': 'rearm_required',
    'no_status_evidence': 'no_status_evidence',
    # 没有可执行命令：guard 没有锁停，不伪造锁停原因（见 WriteRecord.zero_reason）。
    'no_demand': '',
}

LICENSE_TOPIC = 'motion_license'
ULTRASONIC_TOPIC = 'ultrasonic'
REARM_TOPIC = 'rearm_request'


class WriteRecord:
    """一拍的三分离结果。"""

    __slots__ = ('tick', 'now', 'decision', 'frame_kind', 'zero_reason',
                 'zero_issued', 'frame', 'serial_write_ok', 'written',
                 'vehicle_stopped', 'stop_cause')

    def __init__(self, *, tick, now, decision, frame_kind, zero_reason):
        self.tick = tick
        self.now = now
        self.decision = decision
        self.frame_kind = frame_kind      # 'speed' | 'zero' | 'none'
        self.zero_reason = zero_reason
        self.zero_issued = (frame_kind == 'zero')
        self.frame = None
        self.serial_write_ok = None       # None = 本拍未尝试写
        self.written = 0
        self.vehicle_stopped = None       # None = 无回传证据
        self.stop_cause = REASON_TO_CAUSE.get(zero_reason, '') if frame_kind != 'speed' else ''
        # 三件事分开：zero_issued / serial_write_ok / vehicle_stopped（见 as_dict）。

    @property
    def is_nonzero(self):
        return self.frame_kind == 'speed'

    def as_dict(self):
        vx, vy, vz, source, reason = self.decision
        return {
            'tick': self.tick,
            'now': self.now,
            'vx': vx, 'vy': vy, 'vz': vz,
            'source': source,
            'zero_reason': reason,
            'frame_kind': self.frame_kind,
            'zero_issued': self.zero_issued,
            'serial_write_ok': self.serial_write_ok,
            'written': self.written,
            'vehicle_stopped': self.vehicle_stopped,
            'stop_cause': self.stop_cause,
        }


class MotionLicenseBridge:
    """许可 + 四路仲裁 + 帧写入的一体化桥（不打开设备，不自建定时器）。"""

    def __init__(self, *, port=None, clock=time.monotonic, wall_clock=time.time,
                 drive_enable=False, status_stale_s=3.0,
                 license_timeout_s=0.5, license_expiry_grace_s=0.05,
                 ultrasonic_stale_s=0.3,
                 scan_stale_s=0.3, ultrasonic_max_range_m=4.5,
                 floor_filter_m=0.50, command_fresh_s=1.0,
                 scan_min_coverage=0.0, scan_strict=True):
        self.port = port
        self.clock = clock
        self.wall_clock = wall_clock
        self.drive_enable = bool(drive_enable)
        self.status_stale_s = float(status_stale_s)
        self.ultrasonic_max_range_m = float(ultrasonic_max_range_m)
        self.floor_filter_m = float(floor_filter_m)
        self.guard = MotionGuard(clock,
                                 license_timeout_s=license_timeout_s,
                                 license_expiry_grace_s=license_expiry_grace_s,
                                 ultrasonic_stale_s=ultrasonic_stale_s,
                                 ultrasonic_release_m=floor_filter_m,
                                 scan_stale_s=scan_stale_s,
                                 command_fresh_s=command_fresh_s,
                                 scan_min_coverage=scan_min_coverage,
                                 scan_strict=scan_strict)
        self._tick = 0
        self._last_write_at = None
        self._status_at = None
        self._status = {}
        self._ultra = (None, None)        # (range_m, recv_mono)
        self._scan = (None, None)         # (scan, stamp_mono)
        self._rearm_requested = False
        self.session_authorized = False
        self._subs = []
        self.records: list[WriteRecord] = []
        self.counters = {
            'ticks': 0,
            'speed_frames': 0,            # 决定写速度帧的次数
            'zero_frames': 0,             # 决定写零速帧的次数（含被拒写）
            'serial_write_ok': 0,         # 11 字节完整写入成功
            'serial_write_failed': 0,
            'vehicle_stopped_evidence': 0,  # 回传证明速度全零的拍数
            'vehicle_stopped_unproven': 0,  # 发零速但无回传证据的拍数
            'blocked_no_port': 0,
            'blocked_disabled': 0,
        }

    # ------------------------------------------------------------ 输入
    def on_license(self, msg):
        """``/motion_license``（JSON 文本或 String 消息）。"""
        raw = getattr(msg, 'data', msg)
        ok, reason = self.guard.accept_license_json(raw)
        self._note('license_rejected' if not ok else 'license_ok', reason)
        return ok

    def on_license_payload(self, payload):
        """已是 dict 的许可（测试/内部复用）。"""
        return self.guard.accept_license(payload)

    def on_ultrasonic(self, msg):
        """``sensor_msgs/Range``：无回波被上游映射为 ``max_range``，按无效处理。"""
        r = getattr(msg, 'range', msg)
        self._ultra = (float(r), float(self.clock()))

    def on_scan(self, msg):
        """``sensor_msgs/LaserScan``：帧龄按原始 ``header.stamp`` 换算。"""
        now_mono = float(self.clock())
        now_wall = float(self.wall_clock())
        stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
        stamp_mono = scan_stamp_to_mono(stamp, now_mono=now_mono,
                                        now_wall=now_wall)
        self._scan = (msg, stamp_mono)
        self.guard.on_scan(msg, stamp_mono)

    def on_command(self, channel, msg):
        """四路命令（Twist 或 ``(vx, vy, vz)``）。"""
        vx, vy, vz = _twist(msg)
        ok = self.guard.on_command(channel, vx, vy, vz)
        if self.guard.stopped:
            # 锁停中收到的命令不缓存（guard 已清缓存；这里只是显式表达）。
            pass
        if channel == 'manual':
            # 手动输入持续即 hold；显式零速/非法值视为松键。
            self.guard.set_manual_hold(bool(ok and (vx or vy or vz)))
        return ok

    def on_rearm_request(self, msg):
        """网页"请求解除锁停"：只登记意图。"""
        data = getattr(msg, 'data', msg)
        active = data if isinstance(data, bool) else (
            isinstance(data, str) and data.strip().lower() in ('true', '1', 'rearm'))
        if active:
            self._rearm_requested = True
            self.guard.request_rearm()
            # [2026-11-18 真机缺口] 已鉴权时立即执行显式 rearm，锁停就地
            # 解除——不再依赖调用方记得调 rearm_via_bridge。未鉴权时保持
            # 请求保留，等 authorize(True) 到达后执行一次。
            # rearm() 成功会清 _rearm_requested（一次性）；失败（preconditions
            # 竞态）时保留请求，让测试/调用方的显式 rearm() 仍可用。
            if getattr(self, 'session_authorized', False):
                ok, _reason = self.rearm()
                if not ok:
                    self._rearm_requested = True
        return active

    def on_status(self, info):
        """底盘回传（速度/编码器/PWM）：唯一的"车辆实际停住"证据来源。

        ``info`` 为 dict，可含 ``velocity_mm_s``、``vy_mm_s``、
        ``yaw_rate_mrad_s``、``pwm_counts``、``encoder_counts``。
        """
        self._status = dict(info or {})
        self._status_at = float(self.clock())
        return self.vehicle_stopped()

    def on_serial_reconnected(self):
        """串口重开：默认锁停（清缓存、丢掉回传证据、作废 rearm 请求）。

        ``MotionGuard`` 内部的许可状态保留（seq/代次不回退，重放仍被拒）。
        """
        self._status_at = None
        self._status = {}
        # 超声最后一条读数也作废：重连后必须等新读数，否则旧读数会"复活"。
        self._ultra = (None, None)
        self._scan = (None, None)
        self.guard.on_scan(None, None)
        self._rearm_requested = False
        self.guard.stop('serial_reconnect')
        self._note('serial_reconnect', 'latched')

    def set_drive_enable(self, enabled):
        self.drive_enable = bool(enabled)

    def preconditions_ok(self):
        """rearm 前提：网页会话鉴权通过（由 ``authorize`` 置）且显式请求解除。"""
        return bool(self._rearm_requested and getattr(self, 'session_authorized', False))

    def authorize(self, ok):
        self.session_authorized = bool(ok)
        if not ok:
            self._rearm_requested = False
        elif self._rearm_requested:
            self.rearm()

    def rearm(self):
        """显式 rearm：前提满足才解除锁停，否则拒绝。

        解停请求是一次性的：无论成败都清掉 ``_rearm_requested``，下次解停
        必须重新请求（防止"上一轮遗留请求"让后续 rearm 绕过前提）。解停后
        若前提瞬间又不成立，下一拍 ``decide`` 会立即重新锁停并给出真实故障
        原因——不会发出任何非零帧。
        """
        ok, reason = self.guard.rearm(self.preconditions_ok())
        self._rearm_requested = False
        self._note('rearm' if ok else 'rearm_refused', reason)
        return ok, reason

    # ------------------------------------------------------------ 每拍
    def sensor_state(self):
        """返回 ``(ultrasonic_state, range_m, ultra_age, scan_ok, scan_reason)``。"""
        r, at = self._ultra
        age = None if at is None else float(self.clock()) - at
        ustate, _ = ultrasonic_state(r, age,
                                     stale_s=self.guard.ultrasonic_stale_s,
                                     release_m=self.floor_filter_m)
        return ustate, r, age

    def tick(self, *, ultrasonic=None, scan=None, write_gap_s=0.0):
        """一拍：仲裁 → 帧 → 写入 → 三分离记录。返回 :class:`WriteRecord`。

        ``ultrasonic`` / ``scan`` 可选：传入时先经 ``on_ultrasonic`` /
        ``on_scan`` 处理（时间桩测试里让传感器与时钟同步推进，避免"时钟前
        进但只有一路传感器更新"造成的假故障）。

        ``write_gap_s`` > 0 时：距上一拍写入不足该时长则本拍**不写**（仍做
        决策并记录，``frame_kind='none'``、``serial_write_ok=None``），
        用于"读完状态帧后留一拍余量再写"，避免与读循环写调度抢占。
        """
        if ultrasonic is not None:
            self.on_ultrasonic(ultrasonic)
        if scan is not None:
            self.on_scan(scan)
        now = float(self.clock())
        self._tick += 1
        self.counters['ticks'] = self._tick
        ustate, _range_m, _uage = self.sensor_state()
        # 雷达随每拍重新喂给 guard：帧龄随 now 推进，按当前命令方向检查。
        self.guard.on_scan(*self._scan)
        decision = self.guard.decide(ultrasonic=ustate)
        record = WriteRecord(tick=self._tick, now=now, decision=decision,
                             frame_kind='zero', zero_reason=decision[4] or '')
        if not decision[4] and not self.guard.stopped:
            record.frame_kind = 'speed'
            record.zero_reason = ''
        if not self.drive_enable:
            # 写路径未启用：不写任何字节，只记录决策（"发出零速"=未发出）。
            record.frame_kind = 'none'
            record.zero_issued = False
            self.counters['blocked_disabled'] += 1
            self._append(record)
            return record
        # 三分离字段的每拍默认值：本拍没真的写零速帧 → zero_issued False；
        # 串口未尝试写 → None。写零速帧的分支在下面逐个置真。
        record.zero_issued = False
        record.serial_write_ok = None
        if record.frame_kind == 'speed' and not self.status_fresh():
            # 无回传证据：本拍不发速度帧，改发零速帧。
            record.frame_kind = 'zero'
            record.zero_reason = 'no_status_evidence'
            record.stop_cause = 'no_status_evidence'
        elif record.frame_kind == 'zero':
            record.stop_cause = REASON_TO_CAUSE.get(record.zero_reason, '')
        if (write_gap_s > 0 and self._last_write_at is not None
                and now - self._last_write_at < float(write_gap_s)):
            # 与上一拍写入太近：本拍只记录决策，不写任何字节。
            record.frame_kind = 'none'
            record.frame = None
            record.serial_write_ok = None
            self.counters['deferred_write_gap'] = (
                self.counters.get('deferred_write_gap', 0) + 1)
            self._append(record)
            return record
        if self.port is None or not bool(getattr(self.port, 'is_real', False)):
            # 没有真实串口：不伪造写入结果；只记录决定。
            record.frame_kind = 'none'
            record.frame = None
            record.serial_write_ok = None
            self.counters['blocked_no_port'] += 1
            self._append(record)
            return record
        if record.frame_kind == 'speed':
            frame = encode_baseline_speed_frame(decision[0], decision[2], decision[1])
            self.counters['speed_frames'] += 1
        else:
            frame = encode_baseline_zero_frame()
            self.counters['zero_frames'] += 1
        record.frame = frame
        record.zero_issued = (record.frame_kind == 'zero')
        self._last_write_at = now
        try:
            written = int(self.port.write_bytes(frame))
        except OSError:
            written = 0
        record.written = written
        record.serial_write_ok = (written == len(frame))
        if record.serial_write_ok:
            self.counters['serial_write_ok'] += 1
        else:
            self.counters['serial_write_failed'] += 1
        # 车辆是否真停只由回传证据决定，与写入结果无关。
        record.vehicle_stopped = self.vehicle_stopped()
        if record.vehicle_stopped is True:
            self.counters['vehicle_stopped_evidence'] += 1
        elif record.zero_issued:
            # 发了零速但本轮没有回传证明：单独计数（不能与"写成功"混谈）。
            self.counters['vehicle_stopped_unproven'] += 1
        self._append(record)
        return record

    def tick_record_only(self):
        """只决策不写（与另一条写入路径共用 fd 时）：返回记录，不写字节。"""
        saved = self.drive_enable
        self.drive_enable = False
        try:
            return self.tick()
        finally:
            self.drive_enable = saved

    def port_is_writable(self):
        """运行期写入门：真实端口 + ``drive_enable`` + fd 已授权。

        与 ``tick`` 内的判定同一口径；供节点侧每拍复核（重连后端口对象被
        换掉/撤权时立即停止写入）。
        """
        if self.port is None or not bool(getattr(self.port, 'is_real', False)):
            return False
        if not self.drive_enable:
            return False
        authorised = getattr(self.port, 'fd_authorised', True)
        return bool(authorised)

    def status_fresh(self):
        if self._status_at is None:
            return False
        return (float(self.clock()) - self._status_at) <= self.status_stale_s

    def vehicle_stopped(self):
        """回传证明车辆静止 → True；有回传证明在动 → False；无证据 → None。"""
        if not self.status_fresh() or not self._status:
            return None
        s = self._status
        vx = abs(float(s.get('velocity_mm_s', s.get('vx_mm_s', 0)) or 0))
        vy = abs(float(s.get('vy_mm_s', 0) or 0))
        wz = abs(float(s.get('yaw_rate_mrad_s', 0) or 0))
        pwm = [abs(int(v)) for v in (s.get('pwm_counts') or ())]
        idle = (vx == 0 and vy == 0 and wz == 0 and all(v == 0 for v in pwm))
        if not idle:
            # 固件速度/编码器非零，或有 PWM 输出：车辆在动。
            return False
        # 速度与 PWM 全零只是必要证据；设计说明要求"有回传证据"时才算停住。
        # 本函数只在 status_fresh() 时有意义（见 tick 的降级）。
        return True

    # ------------------------------------------------------------ ROS 接线
    def attach(self, node, *,
               license_topic=LICENSE_TOPIC,
               ultrasonic_topic=ULTRASONIC_TOPIC,
               scan_topic='scan',
               rearm_topic=REARM_TOPIC,
               command_topics=None,
               timer_period_s=None):
        """在节点上创建订阅（不打开设备、不建第二套写入路径）。

        ``command_topics`` 形如 ``{'manual': '/manual_cmd_vel', ...}``。
        定时器由所有者创建并调用 :meth:`tick`；本方法不创建定时器。
        返回 ``(record) -> None`` 日志回调可另行挂接。
        """
        from std_msgs.msg import String, Bool
        from sensor_msgs.msg import Range, LaserScan
        from geometry_msgs.msg import Twist
        from rclpy.qos import qos_profile_sensor_data

        subs = [
            node.create_subscription(String, license_topic,
                                     self.on_license, 10),
            node.create_subscription(Range, ultrasonic_topic,
                                     self.on_ultrasonic, 10),
            node.create_subscription(LaserScan, scan_topic, self.on_scan,
                                     qos_profile_sensor_data),
            node.create_subscription(Bool, rearm_topic,
                                     self.on_rearm_request, 10),
        ]
        for ch, topic in (command_topics or {}).items():
            if ch not in CHANNELS:
                raise KeyError(ch)
            subs.append(node.create_subscription(
                Twist, topic,
                (lambda m, _ch=ch: self.on_command(_ch, m)), 10))
        self._subs = subs
        return subs

    def detach(self):
        for sub in self._subs:
            destroy = getattr(sub, 'destroy', None)
            if callable(destroy):
                destroy()
        self._subs = []

    # ------------------------------------------------------------ 诊断
    def _note(self, kind, detail):
        self.guard._event(kind, detail)   # noqa: SLF001——同一对象的事件表

    def _append(self, record):
        self.records.append(record)
        if len(self.records) > 500:
            del self.records[:250]

    def last_zero_write_separately(self):
        """最近一拍的三个独立字段：``zero_issued`` / ``serial_write_ok`` /
        ``vehicle_stopped``。"""
        if not self.records:
            return None
        r = self.records[-1]
        return {'zero_issued': r.zero_issued,
                'serial_write_ok': r.serial_write_ok,
                'vehicle_stopped': r.vehicle_stopped}

    def snapshot(self):
        return {
            'guard': self.guard.snapshot(),
            'counters': dict(self.counters),
            'drive_enable': self.drive_enable,
            'port_kind': getattr(self.port, 'kind', None),
            'status_fresh': self.status_fresh(),
            'vehicle_stopped': self.vehicle_stopped() if self.status_fresh() else None,
            'ultrasonic': {'range_m': self._ultra[0], 'state': self.sensor_state()[0]},
            'scan_stamp_mono': self._scan[1],
            'last': self.records[-1].as_dict() if self.records else None,
            'separate': self.last_zero_write_separately(),
        }

    def log_line(self):
        """一行诊断（日志用）：三分离字段显式分开打印。"""
        snap = self.snapshot()
        last = snap['last'] or {}
        return ('motion: src=%s zero=%s | zero_issued=%s serial_write_ok=%s '
                'vehicle_stopped=%s | stop_cause=%s counters=%s'
                % (last.get('source'), last.get('zero_reason'),
                   last.get('zero_issued'), last.get('serial_write_ok'),
                   last.get('vehicle_stopped'), last.get('stop_cause'),
                   json.dumps(snap['counters'], sort_keys=True)))


def _twist(msg):
    if isinstance(msg, (tuple, list)) and len(msg) == 3:
        return float(msg[0]), float(msg[1]), float(msg[2])
    lin = getattr(msg, 'linear', None)
    ang = getattr(msg, 'angular', None)
    return (float(getattr(lin, 'x', 0.0)), float(getattr(lin, 'y', 0.0)),
            float(getattr(ang, 'z', 0.0)))


__all__ = ['MotionLicenseBridge', 'WriteRecord', 'REASON_TO_CAUSE',
           'LICENSE_TOPIC', 'ULTRASONIC_TOPIC', 'REARM_TOPIC',
           'FRAME_LENGTH', 'ZERO_REASON', 'STOP_CAUSE', 'scan_check',
           'direction_sectors']

