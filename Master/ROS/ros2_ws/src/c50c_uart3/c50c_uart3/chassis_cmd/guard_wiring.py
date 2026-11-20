"""把 ``MotionLicenseBridge``（guard 区）接到 ``uart3_node`` 的运行时路径上。

节点侧只在三处调用本模块（见 ``uart3_node.py``）：

- ``_attach_drive_bridge``：每个（重）连周期第一个定时器拍里
  ``ensure()`` —— 建桥、订 `/motion_license`、`/rearm_request`、
  `/manual_cmd_vel`、`/nav_cmd_vel`、`/auto_cmd_vel`、`/demo_cmd_vel`，
  并要求写入端就是本节点的 ``FdPort``（同一个 fd，不新开设备）；
- ``_on_serial_reconnected``：``on_serial_reconnected()`` —— 重连默认锁停，
  作废回传/传感器证据与解停请求；
- ``_handle_frame``：``on_status_frame(frame, now_mono)`` —— 状态帧是本节点
  唯一的底盘回传证据（速度/PWM/编码器），同时作为超声量测来源（``0xFFFF``
  无回波映射成 ``max_range``，与 ``_publish_ultrasonic`` 同口径）。

运行时写入门（节点每次 ``tick`` 前判定，不满足则不写**任何**字节）：

1. guard 已用真实 ``FdPort`` 建好（``port.is_real``）；
2. ``drive_enable`` 为真且端口获授权（``fd_authorised``）；
3. 固件方言为基线（本桥只发 11 字节基线帧——**本桥不会向 v3 固件写帧**）；
4. 没有另一套写入路径（直行命令桥）同时挂在同一个 fd 上；
5. 每拍发送会**覆盖**直行桥刚写入的字节，所以只有本节点没有任何其它
   订阅线程在发运动命令时才启用：检测方式 = 自己创建订阅器后，
   除自己以外每个命令话题的发布者数必须为 0（否则整桥不写，日志说明原因）。

授权（``/guard_authorize``，``std_msgs/Bool``，默认未授权）：会话鉴权由网页
侧给出。未授权时 ``rearm`` 一律被拒，车辆保持锁停。
"""
from __future__ import annotations

import json
import time

from ..motion_guard import CHANNELS

COMMAND_TOPICS = {
    'manual': '/manual_cmd_vel',
    'nav': '/nav_cmd_vel',
    'auto': '/auto_cmd_vel',
    'demo': '/demo_cmd_vel',
}
ULTRASONIC_TOPIC = '/ultrasonic_front'
SCAN_TOPIC = '/scan'
LICENSE_TOPIC = '/motion_license'
REARM_TOPIC = '/rearm_request'
AUTHORIZE_TOPIC = '/guard_authorize'
GUARD_STATUS_TOPIC = '/guard_status'

NO_ECHO_MM = 0xFFFF


class GuardWiring:
    """节点内的 guard 运行时接线（不在 __init__ 里建，避免启动竞态）。"""

    def __init__(self, node, *, port, device, firmware_protocol,
                 baseline_protocol, ultrasonic_max_range_m=4.5,
                 drive_enable=True, fd_authorised=True,
                 has_other_writer=False, write_gap_s=0.0,
                 coexist=False, clock=time.monotonic,
                 ultrasonic_stale_s=1.5, scan_stale_s=1.0,
                 license_timeout_s=1.5, license_expiry_grace_s=1.0):
        self.node = node
        self.port = port
        self.device = device
        self.firmware_protocol = firmware_protocol
        self.baseline_protocol = baseline_protocol
        self.drive_enable = bool(drive_enable)
        self.fd_authorised = bool(fd_authorised)
        self.has_other_writer = bool(has_other_writer)
        # [B3 修复 2026-11-18] coexist：与直行命令桥共用同一 fd 的共存模式。
        # True 时 guard 仍正常挂载、正常决策，但桥的 drive_enable 被关掉
        # （每拍只记录 frame_kind='none'，不写字节），由调用方走
        # record_only 分支并经 forward_stop_to_arbiter 注入停车。
        self.coexist = bool(coexist)
        self.write_gap_s = float(write_gap_s)
        self.ultrasonic_max_range_m = float(ultrasonic_max_range_m)
        self.clock = clock
        self.ultrasonic_stale_s = float(ultrasonic_stale_s)
        self.scan_stale_s = float(scan_stale_s)
        self.license_timeout_s = float(license_timeout_s)
        self.license_expiry_grace_s = float(license_expiry_grace_s)
        self.bridge = None
        self.subs = []
        self._status_pub = None
        self._last_status_key = None
        self._last_status_t = None
        self.authorized = False
        self._reported = set()

    # ------------------------------------------------------------ 建桥
    def _blocked_reason(self):
        if not bool(getattr(self.port, 'is_real', False)):
            return 'port_not_real'
        if not self.drive_enable:
            return 'drive_enable_false'
        if not self.fd_authorised:
            return 'fd_not_authorised'
        if self.firmware_protocol != self.baseline_protocol:
            return 'firmware_not_baseline'
        if self.has_other_writer:
            return 'other_writer_present'
        return ''

    def ensure(self):
        """第一个定时器拍调用；已有桥或门不满足时返回 ``None``。"""
        if self.bridge is not None:
            return self.bridge
        why = self._blocked_reason()
        if why:
            self._report_once(why,
                              f'guard 区未挂载（{why}）：本次连接不写任何'
                              '运动许可帧，车辆保持基线行为/锁停')
            return None
        from .guard_bridge import MotionLicenseBridge
        # 时间换算链必须一致：scan_stamp_to_mono 要求"同一时刻采样的单调
        # 时钟 + 墙钟"，换算出的 stamp_mono 也落在单调时基上，因此桥的内部
        # 时钟必须是单调时钟（默认 time.monotonic），不能换成节点 ROS 时钟。
        bridge = MotionLicenseBridge(
            port=self.port, clock=self.clock,
            wall_clock=time.time,
            # [B3 修复 2026-11-18] 共存模式（coexist=True）时桥的写路径
            # 关闭：每拍只记录决策（frame_kind='none'），永不写字节——
            # guard 不是第二个写者。独占模式行为不变。
            drive_enable=self.drive_enable and not self.coexist,
            ultrasonic_max_range_m=self.ultrasonic_max_range_m,
            ultrasonic_stale_s=self.ultrasonic_stale_s,
            scan_stale_s=self.scan_stale_s,
            license_timeout_s=self.license_timeout_s,
            license_expiry_grace_s=self.license_expiry_grace_s)
        bridge.authorize(self.authorized)
        self.subs = bridge.attach(
            self.node,
            license_topic=LICENSE_TOPIC,
            ultrasonic_topic=ULTRASONIC_TOPIC,
            scan_topic=SCAN_TOPIC,
            rearm_topic=REARM_TOPIC,
            command_topics=dict(COMMAND_TOPICS))
        from std_msgs.msg import Bool, String
        self.subs.append(self.node.create_subscription(
            Bool, AUTHORIZE_TOPIC, self._on_authorize, 10))
        if callable(getattr(self.node, 'create_publisher', None)):
            try:
                self._status_pub = self.node.create_publisher(
                    String, GUARD_STATUS_TOPIC, 10)
            except Exception:
                self._status_pub = None
        self.bridge = bridge
        self.node.get_logger().warn(
            'guard 区已挂载：运动许可/四路仲裁/方向-帧龄停车生效；'
            '启动默认锁停，rearm 需要 /guard_authorize 鉴权 + /rearm_request')
        return bridge

    def _on_authorize(self, msg):
        self.authorized = bool(getattr(msg, 'data', False))
        self.bridge.authorize(self.authorized)
        if not self.authorized:
            self.bridge.guard.stop('global_stop')
        self.node.get_logger().warn(
            f'guard 鉴权 = {self.authorized}'
            + ('' if self.authorized else '：锁停并作废解停请求'))

    def rearm_via_bridge(self):
        """显式解停：前提 = 已鉴权 + 已收到解停请求（bridge.preconditions_ok）。"""
        if self.bridge is None:
            return False, 'not_attached'
        if not self.authorized:
            return False, 'preconditions'
        return self.bridge.rearm()

    # ------------------------------------------------ 共存：向直行桥注入停车
    def forward_stop_to_arbiter(self, chain):
        """共存模式（drive_input_enable=true，guard 只决策不写）：把 guard
        的锁停状态注入直行命令桥的仲裁器。

        guard 解除锁停时只解除 guard 自己的停车状态。外部监督停车仍由
        独立状态保持；任一来源停车都会丢弃缓存的非零命令。

        ``chain`` 是直行桥的 ``StraightCommandChain``（可能为 None 或
        桩，测试环境无仲裁器时跳过）。
        """
        if chain is None:
            return
        arbiter = getattr(chain, '_arbiter', None)
        if arbiter is None:
            return
        stopped = bool(self.bridge is not None and self.bridge.guard.stopped)
        if stopped:
            arbiter.on_guard_stop(True)
            self._forwarded_stop = True
        elif getattr(self, '_forwarded_stop', False):
            arbiter.on_guard_stop(False)
            self._forwarded_stop = False

    # ------------------------------------------------------------ 事件
    def on_serial_reconnected(self):
        """重连：默认锁停（不依赖桥是否已建）。

        桥未建时没有可锁的对象——下一次 ``ensure()`` 建的桥本身就是
        "启动默认锁停"。桥已建则清缓存、回传/传感器证据与解停请求。
        """
        if self.bridge is not None:
            self.bridge.on_serial_reconnected()
            self.node.get_logger().warn('guard：串口重连，已锁停并清缓存/证据')

    def on_status_frame(self, frame, now_mono):
        """状态帧 → 回传证据 + 超声量测（与 ``_publish_ultrasonic`` 同口径）。"""
        if self.bridge is None:
            return
        mm = int(getattr(frame, 'ultrasonic_front_mm', 0))
        range_m = (self.ultrasonic_max_range_m if mm == NO_ECHO_MM
                   else mm / 1000.0)
        self.bridge.on_status({
            'velocity_mm_s': int(frame.chassis_velocity_mm_s[0]),
            'vy_mm_s': int(frame.chassis_velocity_mm_s[1]),
            'yaw_rate_mrad_s': int(frame.chassis_yaw_rate_mrad_s),
            'pwm_counts': tuple(int(p) for p in frame.pwm_counts),
            'encoder_counts': tuple(int(c) for c in frame.encoder_counts),
        })
        self.bridge.on_ultrasonic(_SimpleRange(range_m, msg_max_range=(
            self.ultrasonic_max_range_m)))
        self._status_now = now_mono

    # ------------------------------------------------------------ 每拍
    def foreign_publishers(self):
        """命令话题上**任何**发布者（本桥只订阅、从不发布，因此非空即他人）。

        返回 ``{topic: n}``；查询失败记为 ``{'error': ...}``（保守：视为有）。
        """
        if self.bridge is None:
            return {'not_attached': 1}
        out = {}
        for topic in COMMAND_TOPICS.values():
            try:
                n = int(self.node.count_publishers(topic))
            except Exception as exc:  # noqa: BLE001——ROS API 异常不得打断驱动
                out['error:%s' % exc] = 1
                continue
            if n:
                out[topic] = n
        return out

    def can_write(self):
        """``(ok, why)``：见模块文档的写入门 1–5。"""
        if self.bridge is None:
            return False, 'not_attached'
        if not self.bridge.port_is_writable():
            return False, 'port'
        if not self.authorized:
            return False, 'unauthorized'
        if self.foreign_publishers():
            return False, 'other_publisher'
        return True, ''

    def guard_tick_record_only(self):
        """只决策不写（与直行桥共用 fd 的共存模式）：结果只记入诊断。"""
        rec = self.bridge.tick_record_only()
        self._last_record_only = rec
        return rec

    def tick(self):
        """guard 可写时返回本拍 ``WriteRecord``，否则 ``None``（调用方不写）。

        ``None`` 的语义是"本拍一个字节都不写"——既不是基线写入，也不是
        伪造的零速；调用方据此决定是否执行自己的（基线）动作。
        """
        ok, why = self.can_write()
        if not ok:
            self._report_once('tick_' + why,
                              f'guard 每拍不写（{why}）：保持基线写入路径/不写')
            return None
        # 与读循环/直行桥共用同一个 fd：距上一拍写入不足一个周期则本拍不写
        return self.bridge.tick(write_gap_s=self.write_gap_s)

    def publish_status(self, now_mono=None):
        """发布底盘 guard 锁停快照到 ``/guard_status``（边沿立即发，否则 5 Hz）。"""
        if self._status_pub is None or self.bridge is None:
            return False
        now = float(self.clock()) if now_mono is None else float(now_mono)
        snap = self.bridge.guard.snapshot()
        ustate, range_m, _uage = self.bridge.sensor_state()
        last_rec = getattr(self, '_last_record_only', None)
        zero_reason = getattr(last_rec, 'zero_reason', '') if last_rec is not None else ''
        key = (bool(snap['latched']), str(snap['stop_cause'] or ''),
               bool(snap['rearm_pending']), str(ustate or ''), str(zero_reason or ''))
        if (key == self._last_status_key and self._last_status_t is not None
                and (now - self._last_status_t) < 0.2):
            return False
        payload = {
            'latched': bool(snap['latched']),
            'stop_cause': snap['stop_cause'] or '',
            'stop_text': snap['stop_text'] or '',
            'rearm_pending': bool(snap['rearm_pending']),
            'last_fault': snap['last_fault'] or '',
            'ultrasonic_state': ustate or '',
            'ultrasonic_range_m': range_m,
            'zero_reason': zero_reason or '',
            'ts': round(now, 3),
        }
        try:
            from std_msgs.msg import String
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
            self._status_pub.publish(msg)
            self._last_status_key = key
            self._last_status_t = now
            return True
        except Exception:
            return False

    def _report_once(self, key, text):
        if key in self._reported:
            return
        self._reported.add(key)
        self.node.get_logger().error(text)


class _SimpleRange:
    """``sensor_msgs/Range`` 的最小替身（只提供 bridge 读取的 ``range``）。"""

    def __init__(self, range_m, *, msg_max_range):
        self.range = float(range_m)
        self.max_range = float(msg_max_range)


__all__ = ['GuardWiring', 'COMMAND_TOPICS', 'GUARD_STATUS_TOPIC', 'NO_ECHO_MM']
