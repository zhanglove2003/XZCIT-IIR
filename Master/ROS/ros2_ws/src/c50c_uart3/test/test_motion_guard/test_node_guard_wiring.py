"""节点运行时接线测试（本机无 rclpy：用桩 rclpy / 桩消息包，进程内跑真节点代码）。

覆盖：复核 A（guard 未接到节点运行时路径）——
- ``_on_serial_reconnected`` 调用 guard 重连锁停；
- ``_handle_frame`` 把状态帧作为回传证据与超声量测喂给 guard；
- ``_drive_cycle`` 在 guard 可写时由 guard 写 11 字节帧，不可写时**一个字节
  都不写**（不伪造、不退化成未受许可约束的直行桥）；
- 非基线方言 / fd 未授权 / 有另一条写入路径 / 未鉴权 / 有外部命令发布者 /
  端口不是真实 FdPort → 一律不写，只记一次日志；
- guard 与直行桥共用 fd：每拍最多写一帧（``write_gap_s`` 跳过写入但记录决策）。

只构造节点对象、不启动 rclpy 上下文；串口 fd 是一个**未授权**的临时文件描述
符（``FdPort`` 构造只对真实 UART 设备要求授权，/tmp 临时文件写入只落在磁盘）。
"""
import math
import os
import sys
import time
import tempfile
import types
import pytest

from c50c_uart3.chassis_cmd import guard_wiring as gw
from c50c_uart3.chassis_cmd.baseline import encode_baseline_zero_frame
from c50c_uart3.chassis_cmd.serial_port import FdPort


# ---------------------------------------------------------------- 桩
class Clock:
    def __init__(self, t=100.0):
        self.t = float(t)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)


class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Stamp:
    def __init__(self, sec, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class Scan:
    def __init__(self, fill=5.0, n=628):
        self.n = n
        self.angle_min = 0.0
        self.angle_increment = 2.0 * math.pi / n
        self.angle_max = self.angle_increment * (n - 1)
        self.range_min = 0.02
        self.range_max = 12.0
        self.ranges = [fill] * n
        self.header = Msg(stamp=Stamp(0))


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(('info', m))

    def warn(self, m):
        self.lines.append(('warn', m))

    def warning(self, m):
        self.lines.append(('warn', m))

    def error(self, m):
        self.lines.append(('error', m))

    def debug(self, m):
        pass


class StubNode:
    """节点侧最小替身：订阅、发布者计数、日志。"""

    def __init__(self):
        self.subs = []
        self.logger = Log()
        self.publishers = {}
        self.counts = {}
        self.executor = None

    def create_subscription(self, msg_type, topic, cb, qos):
        sub = Msg(msg_type=msg_type, topic_name=topic, callback=cb,
                  is_publisher=False)
        self.subs.append(sub)
        return sub

    def get_logger(self):
        return self.logger

    def count_publishers(self, topic):
        return int(self.publishers.get(topic, self.counts.get(topic, 0)))

    def get_clock(self):
        return Msg(now=lambda: Msg(to_msg=lambda: Stamp(1700000000, 0)))

    def create_timer(self, period, cb):
        return Msg(period=period, cb=cb)


class Range:
    def __init__(self, r):
        self.range = r
        self.max_range = 4.5


def tmp_fd_port(authorised=True):
    f = tempfile.NamedTemporaryFile(delete=False)
    f.close()
    fd = os.open(f.name, os.O_RDWR | os.O_APPEND)
    port = FdPort(fd, f.name, fd_authorised=True)   # 非 UART 设备，无需授权
    port.fd_authorised = authorised
    return fd, f.name, port


def _stamp_at_wall(w):
    """把"假单调时钟"当墙钟用：节点 stamp 取 ``w.clock()`` 秒。

    bridge 的换算要 ``now_mono`` 与 ``now_wall`` 同时刻采样；这里让
    ``now_wall`` 就是假时钟、``now_mono`` 用真实单调时钟加固定偏移，
    使 stamp_mono 落在假时钟时基上（与桥内部 ``clock()`` 一致）。
    """
    fake_now = w.clock()
    offset = []

    def bridge_clock():
        if not offset:
            offset.append(time.monotonic() - fake_now)
        return time.monotonic() - offset[0]

    w.bridge.clock = bridge_clock
    w.bridge.wall_clock = bridge_clock
    return Msg(now=lambda: Msg(to_msg=lambda: Stamp(
        int(w.clock()), int((w.clock() % 1.0) * 1e9))))


def stamped(w, scan):
    """节点 LaserScan 的 header.stamp 来自节点时钟；这里取假时钟秒（stamp 0 = 未填）。"""
    scan.header.stamp = Stamp(int(w.clock()), 0)
    return scan


def frame(vx=0, vy=0, wz=0, pwm=(0, 0, 0, 0), ultra_mm=300):
    return Msg(chassis_velocity_mm_s=(vx, vy), chassis_yaw_rate_mrad_s=wz,
               pwm_counts=tuple(pwm), encoder_counts=(0, 0, 0, 0),
               ultrasonic_front_mm=ultra_mm)


def stub_msg_packages(monkeypatch):
    """无 ROS 环境：给 bridge.attach 延迟导入的消息包补桩（只在本文件内生效）。"""
    qos = types.ModuleType('rclpy.qos')
    qos.qos_profile_sensor_data = 10
    monkeypatch.setitem(sys.modules, 'rclpy.qos', qos)
    for pkg, names in (('std_msgs', ('String', 'Bool')),
                       ('sensor_msgs', ('Range', 'LaserScan')),
                       ('geometry_msgs', ('Twist',))):
        pkgmod = types.ModuleType(pkg)
        msgmod = types.ModuleType(pkg + '.msg')
        for n in names:
            setattr(msgmod, n, type(n, (Msg,), {}))
        pkgmod.msg = msgmod
        monkeypatch.setitem(sys.modules, pkg, pkgmod)
        monkeypatch.setitem(sys.modules, pkg + '.msg', msgmod)


@pytest.fixture(autouse=True)
def _msg_stubs(monkeypatch):
    stub_msg_packages(monkeypatch)


@pytest.fixture
def wiring():
    node = StubNode()
    clock = Clock()
    fd, path, port = tmp_fd_port()
    w = gw.GuardWiring(node, port=port, device=path,
                       firmware_protocol='baseline',
                       baseline_protocol='baseline',
                       ultrasonic_max_range_m=4.5,
                       has_other_writer=False, write_gap_s=0.02, clock=clock)
    w.port_path = path
    w.clock_obj = clock
    yield w, node, clock, port
    os.close(fd)
    os.unlink(path)


def written(path):
    with open(path, 'rb') as fh:
        return fh.read()


def subs_by_topic(node):
    return {s.topic_name: s.callback for s in node.subs}


# ---------------------------------------------------------------- 建桥/门
def test_ensure_attaches_expected_topics_and_starts_latched(wiring):
    w, node, clock, port = wiring
    assert w.ensure() is not None
    assert subs_by_topic(node).keys() >= {
        '/motion_license', '/ultrasonic_front', '/scan', '/rearm_request',
        '/manual_cmd_vel', '/nav_cmd_vel', '/auto_cmd_vel', '/demo_cmd_vel',
        '/guard_authorize'}
    assert w.bridge.guard.stopped               # 启动默认锁停
    assert w.ensure() is w.bridge               # 不重复建


@pytest.mark.parametrize('kw,why', [
    (dict(firmware_protocol='v3'), 'firmware_not_baseline'),
    (dict(has_other_writer=True), 'other_writer_present'),
    (dict(fd_authorised=False), 'fd_not_authorised'),
    (dict(drive_enable=False), 'drive_enable_false'),
])
def test_gate_refuses_without_writing(wiring, kw, why):
    w, node, clock, port = wiring
    for k, v in kw.items():
        setattr(w, k, v)
    assert w.ensure() is None
    assert w.tick() is None
    assert written(w.port_path) == b''
    assert any(why in m for _, m in node.logger.lines)


def test_non_real_port_refused(wiring):
    from c50c_uart3.chassis_cmd.serial_port import FakeFdPort
    w, node, clock, _ = wiring
    w.port = FakeFdPort()
    assert w.ensure() is None
    assert w.tick() is None
    assert w.port.stream == b''


# ---------------------------------------------------------------- 运行时行为
def test_status_frame_feeds_evidence_and_ultrasonic(wiring):
    w, node, clock, port = wiring
    w.ensure()
    w.on_status_frame(frame(vx=0, pwm=(0, 0, 0, 0), ultra_mm=0xFFFF), clock())
    assert w.bridge.vehicle_stopped() is True          # 速度/PWM 全零 = 停住
    assert w.bridge._ultra[0] == 4.5                   # 0xFFFF → max_range
    # [2026-11-18 用户定案] 无回波 = 远处无约束，不再 invalid 锁车
    assert w.bridge.sensor_state()[0] == 'ok'
    w.on_status_frame(frame(vx=120, pwm=(300, 300, 300, 300), ultra_mm=300),
                      clock())
    assert w.bridge.vehicle_stopped() is False
    assert w.bridge._ultra[0] == pytest.approx(0.30)
    assert w.bridge.sensor_state()[0] == 'ok'


def test_unauthorized_never_writes(wiring):
    w, node, clock, port = wiring
    w.ensure()
    w.on_status_frame(frame(ultra_mm=300), clock())
    w.on_serial_reconnected()
    subs_by_topic(node)['/motion_license'](Msg(data=(
        '{"session": 1, "seq": 0, "expires_mono": 200.0}')))
    subs_by_topic(node)['/rearm_request'](Msg(data=True))
    assert w.rearm_via_bridge() == (False, 'preconditions')
    rec = w.tick()
    assert rec is None and written(w.port_path) == b''


def test_authorized_but_latched_writes_zero_frame(wiring):
    """默认严格口径下：已鉴权、许可在线、超声有效、雷达缺失 → 锁停写零速帧。"""
    w, node, clock, port = wiring
    w.ensure()
    subs_by_topic(node)['/guard_authorize'](Msg(data=True))
    subs_by_topic(node)['/motion_license'](Msg(data=(
        '{"session": 1, "seq": 0, "expires_mono": 200.0}')))
    w.on_status_frame(frame(ultra_mm=300), clock())
    rec = w.tick()
    assert rec is not None
    assert rec.frame_kind == 'zero' and rec.zero_issued is True
    assert rec.serial_write_ok is True
    assert written(w.port_path) == encode_baseline_zero_frame()


def test_forward_command_blocked_by_empty_front_sector_at_runtime(wiring):
    """运行时路径：前方扇区无有效点 → 前进命令被扣，写出的是零速帧。"""
    w, node, clock, port = wiring
    w.ensure()
    node.get_clock = _stamp_at_wall(w)     # 帧戳取当下墙钟（否则换算出负帧龄 → 被扣成 scan_stale）
    cb = subs_by_topic(node)
    cb['/guard_authorize'](Msg(data=True))
    cb['/motion_license'](Msg(data=(
        '{"session": 1, "seq": 0, "expires_mono": 200.0}')))
    w.on_status_frame(frame(ultra_mm=300), clock())
    scan = Scan(fill=float('nan'))                  # 一圈都没有有效点
    cb['/scan'](stamped(w, scan))
    cb['/rearm_request'](Msg(data=True))
    # [2026-11-18] 已鉴权时请求就地 auto-rearm；仍锁停才补显式 rearm
    if w.bridge.guard.stopped and w.bridge._rearm_requested:
        assert w.rearm_via_bridge() == (True, 'ok')
    cb['/auto_cmd_vel'](Msg(linear=Msg(x=0.2, y=0.0), angular=Msg(z=0.0)))
    w.on_status_frame(frame(ultra_mm=300), clock())
    rec = w.tick()
    assert rec.decision == (0.0, 0.0, 0.0, None, 'scan_blocked')
    assert rec.frame_kind == 'zero'
    assert written(w.port_path) == encode_baseline_zero_frame()


def test_clear_front_sector_allows_forward_speed_frame_at_runtime(wiring):
    """对照组：同一运行时路径，前方扇区有有效点 → 前进命令写出速度帧。"""
    w, node, clock, port = wiring
    w.ensure()
    node.get_clock = _stamp_at_wall(w)
    cb = subs_by_topic(node)
    cb['/guard_authorize'](Msg(data=True))
    cb['/motion_license'](Msg(data=(
        '{"session": 1, "seq": 0, "expires_mono": 200.0}')))
    w.on_status_frame(frame(ultra_mm=300), clock())
    cb['/scan'](stamped(w, Scan()))
    cb['/rearm_request'](Msg(data=True))
    # [2026-11-18] 已鉴权时请求就地 auto-rearm；仍锁停才补显式 rearm
    if w.bridge.guard.stopped and w.bridge._rearm_requested:
        assert w.rearm_via_bridge() == (True, 'ok')
    cb['/auto_cmd_vel'](Msg(linear=Msg(x=0.2, y=0.0), angular=Msg(z=0.0)))
    w.on_status_frame(frame(ultra_mm=300), clock())
    rec = w.tick()
    assert rec.frame_kind == 'speed' and rec.decision[3] == 'auto'
    assert rec.serial_write_ok is True
    assert written(w.port_path)[0:3] == bytes([0x7B, 0x00, 0x00])


def test_scan_stale_by_age_at_runtime(wiring):
    w, node, clock, port = wiring
    w.ensure()
    cb = subs_by_topic(node)
    cb['/guard_authorize'](Msg(data=True))
    cb['/motion_license'](Msg(data=(
        '{"session": 1, "seq": 0, "expires_mono": 200.0}')))
    w.on_status_frame(frame(ultra_mm=300), clock())
    cb['/scan'](Scan())
    cb['/rearm_request'](Msg(data=True))
    # [2026-11-18] 已鉴权时请求就地 auto-rearm；仍锁停才补显式 rearm
    if w.bridge.guard.stopped and w.bridge._rearm_requested:
        assert w.rearm_via_bridge() == (True, 'ok')
    cb['/auto_cmd_vel'](Msg(linear=Msg(x=0.2, y=0.0), angular=Msg(z=0.0)))
    clock.advance(0.31)                             # 同一帧，帧龄 0.31 s
    w.on_status_frame(frame(ultra_mm=300), clock())
    rec = w.tick()
    assert rec.frame_kind == 'zero' and rec.decision[4] == 'scan_stale'


def test_serial_reconnect_relatches(wiring):
    w, node, clock, port = wiring
    w.ensure()
    cb = subs_by_topic(node)
    cb['/guard_authorize'](Msg(data=True))
    cb['/motion_license'](Msg(data=(
        '{"session": 1, "seq": 0, "expires_mono": 200.0}')))
    w.on_status_frame(frame(ultra_mm=300), clock())
    cb['/scan'](Scan())
    cb['/rearm_request'](Msg(data=True))
    # [2026-11-18] 已鉴权：请求就地 auto-rearm（旧 rearm_via_bridge 语义）
    if w.bridge.guard.stopped and w.bridge._rearm_requested:
        assert w.rearm_via_bridge() == (True, 'ok')
    assert not w.bridge.guard.stopped
    w.on_serial_reconnected()
    assert w.bridge.guard.stopped
    assert w.bridge.guard.stop_cause == 'serial_reconnect'
    assert w.bridge._ultra == (None, None)
    assert w.bridge.vehicle_stopped() is None


def test_foreign_publisher_blocks_writes(wiring):
    w, node, clock, port = wiring
    w.ensure()
    assert w.can_write() == (False, 'unauthorized')
    subs_by_topic(node)['/guard_authorize'](Msg(data=True))
    node.counts['/nav_cmd_vel'] = 1                   # 别的节点在发导航命令
    assert w.can_write() == (False, 'other_publisher')
    assert w.tick() is None
    assert written(w.port_path) == b''


def test_write_gap_skips_write_but_records_decision(wiring):
    w, node, clock, port = wiring
    w.ensure()
    node.counts['/nav_cmd_vel'] = 0
    cb = subs_by_topic(node)
    cb['/guard_authorize'](Msg(data=True))
    cb['/motion_license'](Msg(data=(
        '{"session": 1, "seq": 0, "expires_mono": 200.0}')))
    w.on_status_frame(frame(ultra_mm=300), clock())
    first = w.tick()
    assert first.frame_kind == 'zero' and len(written(w.port_path)) == 11
    clock.advance(0.005)                            # < write_gap_s=0.02
    w.on_status_frame(frame(ultra_mm=300), clock())
    second = w.tick()
    assert second is not None and second.frame_kind == 'none'
    assert second.serial_write_ok is None
    assert len(written(w.port_path)) == 11          # 没写第二帧
    clock.advance(0.02)
    w.on_status_frame(frame(ultra_mm=300), clock())
    assert w.tick().frame_kind == 'zero'
    assert len(written(w.port_path)) == 22


def test_record_only_never_writes(wiring):
    w, node, clock, port = wiring
    w.ensure()
    node.counts['/nav_cmd_vel'] = 0
    subs_by_topic(node)['/guard_authorize'](Msg(data=True))
    w.on_status_frame(frame(ultra_mm=300), clock())
    rec = w.guard_tick_record_only()
    assert rec is not None and rec.frame_kind == 'none'
    assert written(w.port_path) == b''


# ---------------------------------------------------------------- 节点 call-site
@pytest.fixture
def node_cls(monkeypatch):
    """导入真 ``uart3_node``：补桩 rclpy / 消息包 / c50c_odom（本机没有 ROS）。"""
    def stub_mod(name, **attrs):
        mod = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(mod, k, v)
        monkeypatch.setitem(sys.modules, name, mod)
        return mod

    class Node:
        def __init__(self, name):
            self._params = {'ultrasonic_max_range_m': 4.5}
            self.logger = Log()

        def declare_parameter(self, name, value):
            self._params.setdefault(name, value)

        def get_parameter(self, name):
            return Msg(value=self._params[name])

        def get_logger(self):
            return self.logger

        def create_publisher(self, *a, **k):
            return Msg(publish=lambda m: None)

        def create_subscription(self, *a, **k):
            return Msg()

        def create_timer(self, period, cb):
            return Msg(period=period, cb=cb)

    stub_mod('rclpy', init=lambda **k: None, ok=lambda: True,
             spin_once=lambda *a, **k: None, try_shutdown=lambda: None)
    stub_mod('rclpy.node', Node=Node)
    stub_mod('rclpy.qos', qos_profile_sensor_data=10)
    stub_mod('rclpy.signals', SignalHandlerOptions=Msg(NO=0))
    stub_mod('tf2_ros', TransformBroadcaster=object)
    for pkg, names in (('diagnostic_msgs', ('DiagnosticArray', 'DiagnosticStatus', 'KeyValue')),
                       ('geometry_msgs', ('TransformStamped',)),
                       ('nav_msgs', ('Odometry',)),
                       ('sensor_msgs', ('Imu', 'Range'))):
        pkgmod = stub_mod(pkg)
        msgmod = stub_mod(pkg + '.msg', **{n: type(n, (Msg,), {}) for n in names})
        pkgmod.msg = msgmod
    stub_mod('c50c_odom', PlanarOdomIntegrator=object)
    stub_mod('c50c_odom.odometry', PlanarOdomIntegrator=object,
             StepReport=type('StepReport', (), {}),
             twist_from_parsed=lambda *a, **k: None)
    import importlib
    sys.modules.pop('c50c_uart3.uart3_node', None)
    uart3_node = importlib.import_module('c50c_uart3.uart3_node')
    yield uart3_node


def make_node(uart3_node, tmp_path):
    node = uart3_node.C50CUart3Node.__new__(uart3_node.C50CUart3Node)
    node._params = {'ultrasonic_max_range_m': 4.5}
    node.logger = Log()
    node._odom_candidate = None
    node._guard_wiring = None
    node._drive_bridge = Msg(cycle=lambda: (None, None))
    node._drive_wiring = None
    node._drive_enabled = True
    node._drive_fd_authorised = True
    node._drive_input_enable = False
    node._drive_firmware_protocol = 'baseline'
    node._drive_tick_period_s = 0.02
    node._parser = Msg(reset=lambda: None)
    node._drive_handshake = None
    node._last_frame_monotonic = None
    node._frame_gap_warn_s = 0.25
    node._publish_imu = lambda *a: None
    node._publish_ultrasonic = lambda *a: None
    node._publish_chassis_diagnostics = lambda *a: None
    node._notify_drive_status_frame = lambda: None
    node._on_serial_reconnected = lambda: uart3_node.C50CUart3Node._on_serial_reconnected(node)
    fd, path, port = tmp_fd_port()
    node._device = path
    node._ultrasonic_max_range_m = 4.5
    node.get_clock = lambda: Msg(now=lambda: Msg(to_msg=lambda: Stamp(1700000000, 0)))
    node._drive_port = port
    node._test_path = path
    node._test_fd = fd
    # 节点上真正的 create_subscription（__new__ 绕过基类，绑到桩节点）
    stub_node = StubNode()
    node.create_subscription = stub_node.create_subscription
    node._stub_node = stub_node
    return node


def test_node_callsites_attach_reconnect_and_status(node_cls, monkeypatch):
    node = make_node(node_cls, None)
    try:
        created = []

        class Wiring:
            def __init__(self, nd, **kw):
                created.append((nd, kw))
                self.bridge = None
                self.stopped = False
                self.frames = []

            def ensure(self):
                return self

            def can_write(self):
                return (False, 'unauthorized')

            def tick(self):
                return Msg(frame_kind='zero')

            def guard_tick_record_only(self):
                return Msg(frame_kind='none')

            def on_serial_reconnected(self):
                self.stopped = True

            def on_status_frame(self, frame, now):
                self.frames.append((frame, now))

        monkeypatch.setattr(gw, 'GuardWiring', Wiring)
        # 第一个定时器拍：建 guard
        node._drive_cycle()
        assert len(created) == 1
        kw = created[0][1]
        assert kw['firmware_protocol'] == 'baseline'
        assert kw['has_other_writer'] is False
        assert kw['write_gap_s'] == pytest.approx(0.02)
        w = node._guard_wiring
        # 状态帧 → guard
        node._handle_frame(frame(ultra_mm=0xFFFF))
        assert len(w.frames) == 1 and w.frames[0][0].ultrasonic_front_mm == 0xFFFF
        # 重连 → guard 锁停
        node._on_serial_reconnected()
        assert w.stopped is True
    finally:
        os.close(node._test_fd)
        os.unlink(node._test_path)


def test_node_drive_cycle_uses_guard_frame_and_never_both(node_cls, monkeypatch):
    node = make_node(node_cls, None)
    try:
        calls = []

        class Wiring:
            def __init__(self, nd, **kw):
                self.can = True

            def ensure(self):
                return self

            def can_write(self):
                return (self.can, '' if self.can else 'unauthorized')

            def tick(self):
                calls.append('guard')
                return Msg(frame_kind='zero')

            def guard_tick_record_only(self):
                calls.append('guard_ro')
                return Msg(frame_kind='none')

            def forward_stop_to_arbiter(self, chain):
                # 与真实 GuardWiring.forward_stop_to_arbiter 同签名：
                # record_only 共存模式下节点会把 guard 锁停以边沿注入
                # 直行桥仲裁器；链为 None 时真实实现静默跳过。
                calls.append(('fwd_stop', chain))

        bridge_calls = []
        node._drive_bridge = Msg(cycle=lambda: (bridge_calls.append(1), (None, None))[1])
        monkeypatch.setattr(gw, 'GuardWiring', Wiring)
        node._drive_cycle()                       # guard 可写
        assert calls == ['guard'] and bridge_calls == []
        node._drive_wiring = Msg(refresh_publishers=lambda: None)  # 共存模式
        node._drive_cycle()
        # record_only 分支：先 guard 只决策，再把锁停边沿注入直行桥仲裁器
        assert calls == ['guard', 'guard_ro', ('fwd_stop', None)]
        assert bridge_calls == [1]
    finally:
        os.close(node._test_fd)
        os.unlink(node._test_path)


def test_drive_probe_reports_guard_latch_and_scan_state(node_cls, tmp_path):
    from c50c_uart3.chassis_cmd.bridge import StraightDriveBridge
    from c50c_uart3.chassis_cmd.guard_bridge import MotionLicenseBridge

    node = make_node(node_cls, tmp_path)
    try:
        node._drive_bridge = StraightDriveBridge(
            node._drive_port, device='/dev/pts/9', enabled=True,
            firmware_protocol='baseline')
        node._guard_wiring = Msg(bridge=MotionLicenseBridge())
        node._guard_wiring.bridge.guard.stop('scan_stale')
        node._drive_probe_ticks = 249
        node._run_straight_bridge()

        lines = [line for _, line in node.logger.lines if line.startswith('guard probe:')]
        assert len(lines) == 1
        assert 'latched=True' in lines[0]
        assert 'cause=scan_stale' in lines[0]
        assert 'scan_received=False' in lines[0]
        assert 'rearms=0' in lines[0] and 'rearm_refused=0' in lines[0]
    finally:
        os.close(node._test_fd)
        os.unlink(node._test_path)



def test_default_authorization_cannot_rearm(wiring):
    w, node, clock, port = wiring
    b = w.ensure()
    assert b.session_authorized is False
    subs_by_topic(node)['/rearm_request'](Msg(data=True))
    assert b.guard.stopped and b.guard.rearms == 0
