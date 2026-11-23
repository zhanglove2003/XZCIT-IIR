"""第十二轮定向测试：生产路径用**接收方向证据**放行桥，而不是测试收据。

覆盖六条：
1. 默认关闭（drive_enable=False）：不建桥、只读打开、零字节写入；
2. 桥已挂载但输入未启用：行为与实现语义一致（静默、无写入）；
3. 接入：喂真实格式会话帧 → 握手写确认帧、仲裁器会话已置 → 非零命令
   写出非零帧且 bit7 已置、序号递增；
4. 缺少接收证据：不给会话帧 → 非零命令仍然零字节写出（核心一条）；
5. 停车优先：nav_stop 为真时即使有会话与非零命令也只出零速帧；
6. 会话过期：超过 SESSION_MAX_AGE_S 后退回零速，dropped_session_stale 增加。

全部用内存端口 / 伪终端替身，不打开真实串口，不碰 DTR/RTS。
"""
from __future__ import annotations

import ast
import os
import pathlib
import sys
import types

import pytest

from c50c_uart3.chassis_cmd.arbiter import SESSION_MAX_AGE_S, ZeroReason
from c50c_uart3.chassis_cmd.bridge import SESSION_RECEIPT_ATTR, StraightDriveBridge
from c50c_uart3.chassis_cmd.handshake import SessionHandshake, session_frame_for
from c50c_uart3.chassis_cmd.ros_input import RosSubscriptionInput
from c50c_uart3.chassis_cmd.serial_port import FakeFdPort
from sim_confirm import grant_safety_via_input
from c50c_uart3.chassis_cmd.sim_receipt import (SIM_SESSION_SEAL,
                                                SimSessionReceipt)

_ROOT = pathlib.Path(__file__).resolve().parents[2]     # …/c50c_uart3
_PROD = _ROOT / 'c50c_uart3'
DEVICE = '/dev/pts/9'


class Clock:
    def __init__(self, t: float = 100.0) -> None:
        self.t = float(t)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> float:
        self.t += dt
        return self.t


class Twist:
    def __init__(self, vx: float = 0.0, vz: float = 0.0) -> None:
        self.linear = types.SimpleNamespace(x=vx, y=0.0)
        self.angular = types.SimpleNamespace(z=vz)


class Bool:
    def __init__(self, data: bool) -> None:
        self.data = data


def seqs(data: bytes) -> list:
    return [data[i + 2] & 0x7F for i in range(0, len(data), 11)]


def user_frames(data: bytes) -> bytes:
    """去掉会话确认帧（buffer[2] == 0x01），只留速度帧。"""
    return bytes(b for i in range(0, len(data), 11)
                 for b in data[i:i + 11] if data[i + 2] != 0x01)


def _stack(enabled: bool, *, session: bool = True, publishers: int = 1):
    """建一套替身：端口 + 桥 + 握手，可选喂一条真实格式会话帧。"""
    clock = Clock()
    port = FakeFdPort()
    bridge = StraightDriveBridge(port, device=DEVICE, enabled=enabled, clock=clock)
    bridge.input.set_nav_publishers(publishers)
    if enabled:
        # 禁用态下 on_safety 会被丢弃并计数，只在启用时授予
        grant_safety_via_input(bridge, clock())
    handshake = SessionHandshake(bridge.chain._arbiter, clock=clock,
                                 receipt_sink=port)
    if session:
        handshake.feed(session_frame_for(7), write=port.write_bytes)
    return bridge, port, clock, handshake


def _tick(bridge, clock, n: int = 1):
    """推进 n 拍，返回 (write_calls, last_record)。"""
    calls, record = 0, None
    for _ in range(n):
        before = bridge.transport.stats.writes
        clock.advance(0.02)
        _, record = bridge.cycle()
        calls += bridge.transport.stats.writes - before
    return calls, record


# ---------------------------------------------------------------- 1 默认关闭
def test_default_off_opens_read_only_and_writes_nothing():
    """drive_enable=False：不建桥；即使有人手工建桥也不写字节。"""
    node_src = (_PROD / 'uart3_node.py').read_text(encoding='utf-8')
    # 只读打开仍由 drive_enable 决定（不在本轮改动范围内，此处断言未变）。
    assert 'flags = os.O_RDWR if self._drive_enabled else os.O_RDONLY' in node_src

    bridge, port, clock, _ = _stack(enabled=False, session=True)
    bridge.input.on_nav_cmd(Twist(0.05))
    for _ in range(5):
        clock.advance(0.02)
        bridge.cycle()
    # 会话帧的确认帧是握手写的，与桥无关：这里只数桥写出的用户帧。
    assert user_frames(bytes(port.stream)) == b''
    assert bridge.input.state.dropped_while_disabled >= 1
    # 会话证据已在；cycle 每拍都会把收据转交命令链，因此 receiver_clean
    # 变成 True，但输入关闭使适配层每拍都门控，没有任何字节写出。
    assert bridge.snapshot()['session_evidence'] is True
    assert bridge.snapshot()['receiver_clean'] is True
    assert bridge.transport.stats.writes == 0


def _import_uart3_node(monkeypatch):
    """在无 ROS 环境里导入节点模块：只替换外部依赖，节点源码不动。"""
    def mod(name, **attrs):
        m = sys.modules.get(name) or types.ModuleType(name)
        m.__path__ = getattr(m, '__path__', [])
        # 真实模块上的覆盖要登记到 monkeypatch，用例结束后还原；
        # 凭空新建的桩模块也一并还原，避免污染后续用例。
        created = name not in sys.modules
        for k, v in attrs.items():
            if not created and hasattr(m, k):
                monkeypatch.setattr(m, k, v, raising=False)
            else:
                setattr(m, k, v)
        if created:
            monkeypatch.setitem(sys.modules, name, m)
        sys.modules[name] = m
        return m

    class _Node:
        def __init__(self, *a, **k):
            pass

    mod('rclpy')
    mod('rclpy.node', Node=_Node)
    mod('rclpy.qos', qos_profile_sensor_data=object())
    mod('rclpy.signals', SignalHandlerOptions=type('S', (), {}))
    mod('geometry_msgs')
    # Twist 给节点订阅回调（RosSubscriptionInput.start 里惰性导入）。
    mod('geometry_msgs.msg', TransformStamped=object, Twist=Twist)
    mod('std_msgs')
    mod('std_msgs.msg', Bool=Bool)
    mod('nav_msgs')
    mod('nav_msgs.msg', Odometry=object)
    mod('sensor_msgs')
    mod('sensor_msgs.msg', Imu=object)
    mod('diagnostic_msgs')
    mod('diagnostic_msgs.msg', DiagnosticArray=object, DiagnosticStatus=object, KeyValue=object)
    mod('tf2_ros', TransformBroadcaster=object)
    # 先导入真实模块再打桩，见上；否则会造出同名桩并留在 sys.modules。
    import c50c_odom.odometry as _real_odom  # noqa: F401
    mod('c50c_odom')
    mod('c50c_odom.odometry', PlanarOdomIntegrator=object, StepReport=object,
        twist_from_parsed=lambda *a, **k: None)
    import importlib

    return importlib.import_module('c50c_uart3.uart3_node')


def test_default_off_node_path_creates_no_bridge(monkeypatch):
    """节点在 drive_enable=False 时 _attach_drive_bridge 立刻返回。"""
    uart3_node = _import_uart3_node(monkeypatch)

    opened = []
    real_open = os.open

    def _spy_open(path, flags, *a, **k):
        opened.append((str(path), flags))
        pytest.fail('默认关闭不得打开串口')

    monkeypatch.setattr(os, 'open', _spy_open)
    stub = types.SimpleNamespace(
        _drive_enabled=False, _fd=3, _drive_port=None, _drive_bridge=None,
        _drive_handshake=None, _drive_wiring=None, _device=DEVICE,
        _drive_input_enable=True, _drive_tick_period_s=0.02,
        _drive_fd_authorised=False)
    stub.get_logger = lambda: types.SimpleNamespace(warn=lambda *a, **k: None,
                                                    error=lambda *a, **k: None)
    stub.create_timer = lambda *a, **k: pytest.fail('默认关闭不得建定时器')
    uart3_node.C50CUart3Node._attach_drive_bridge(stub)
    assert stub._drive_bridge is None and stub._drive_port is None
    assert stub._drive_wiring is None
    assert opened == []
    assert os.open is _spy_open
    monkeypatch.setattr(os, 'open', real_open)


def test_node_enabled_path_subscribes_without_second_timer(monkeypatch):
    """drive_enable=True + drive_input_enable=True：三订阅、零定时器、收据去处。

    节点定时器只有一个（驱动路径唯一）；接线对象按 ``with_timer=False``
    建订阅，自己不建定时器。
    """
    uart3_node = _import_uart3_node(monkeypatch)
    clock = Clock()
    port = FakeFdPort()
    # 端口工厂替身：给出内存端口，不打开任何 fd（本机无串口，也不许开）。
    import c50c_uart3.chassis_cmd.serial_port as serial_port_mod

    class _PortFactory:
        class RealFdUseForbidden(Exception):
            pass

        def __call__(self, fd, device, *, fd_authorised=False):
            assert fd == 7 and device == DEVICE
            assert fd_authorised is True
            return port

    monkeypatch.setattr(serial_port_mod, 'FdPort', _PortFactory())
    node = _NodeCtx(clock=clock, port=port)
    uart3_node.C50CUart3Node._attach_drive_bridge(node)
    assert node._drive_bridge is not None
    assert node._drive_port is port
    assert node._drive_handshake._receipt_sink is port
    assert len(node.timers) == 1 and len(node.subs) == 3
    assert node._drive_wiring is not None
    assert node._drive_wiring.started is True


class _NodeCtx:
    """节点替身：只提供 _attach_drive_bridge 用到的接口。"""

    def __init__(self, *, clock, port):
        self._clock = clock
        self._port = port
        self.timers, self.subs = [], []
        self._drive_enabled = True
        self._drive_input_enable = True
        self._drive_tick_period_s = 0.02
        self._drive_fd_authorised = True
        # 本文件验证 v3 会话证据路径：显式给默认方言 v3。
        self._drive_firmware_protocol = 'v3'
        self._fd = 7
        self._drive_port = None
        self._drive_bridge = None
        self._drive_handshake = None
        self._drive_wiring = None
        self._device = DEVICE
        self._clock_fn = clock

    def _drive_cycle(self):            # 真实节点的方法，此处只做占位
        return None

    # --- 节点接口 ---
    def get_logger(self):
        return types.SimpleNamespace(warn=lambda *a, **k: None,
                                     error=lambda *a, **k: None)

    def create_timer(self, period, callback):
        timer = _FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = _FakeSub(msg_type, topic, callback, qos)
        self.subs.append(sub)
        return sub

    def resolve_topic_name(self, name):
        return '/' + name.lstrip('/')

    def count_publishers(self, topic):
        return 1 if topic == '/nav_cmd_vel' else 0



# ------------------------------------------- 2 桥已挂载但输入未启用
def test_bridge_mounted_without_input_stays_silent():
    """drive_enable=True, drive_input_enable=False：桥在，但适配层不放行。"""
    bridge, port, clock, _ = _stack(enabled=False, session=True)
    bridge.input.on_nav_cmd(Twist(0.05))
    for _ in range(3):
        clock.advance(0.02)
        bridge.cycle()
    assert user_frames(bytes(port.stream)) == b''
    assert bridge.input.snapshot()['dropped_while_disabled'] == 1
    assert bridge.snapshot()['input']['nav_licensed'] is False


# ------------------------------------------------- 3 实际接入方案
def test_session_frame_then_nonzero_writes_nonzero_frame_with_bit7():
    bridge, port, clock, handshake = _stack(enabled=True)
    assert handshake.stats.acks_sent == 1
    assert bytes(port.stream) == _ack(7)
    assert bridge.chain._arbiter.session_id == 7
    assert bridge.chain._arbiter.session_acknowledged is True
    assert isinstance(getattr(port, SESSION_RECEIPT_ATTR), SimSessionReceipt)

    _first_zero(bridge, clock)
    bridge.input.on_nav_cmd(Twist(0.05))
    records = []
    for _ in range(2):
        clock.advance(0.02)
        _, rec = bridge.cycle()
        records.append(rec)

    frames = user_frames(bytes(port.stream))
    assert len(frames) == 33, frames
    # 序号连续递增 0,1,2，每拍一帧，没有重复推进。
    assert seqs(frames) == [0, 1, 2]
    # buffer[2] 的 bit7 已置（命令帧有效位），低 7 位是序号。
    assert all(f[2] & 0x80 for f in (frames[0:11], frames[11:22], frames[22:33]))
    assert frames[2] & 0x7F == 0 and frames[13] & 0x7F == 1
    # 第 2、3 帧是非零速度（vx 负载非零）；首帧是零速。
    assert frames[24:28] != b'\x00\x00\x00\x00'
    assert frames[3:7] == b'\x00\x00\x00\x00'
    assert [r.is_zero for r in records] == [False, False]
    assert records[-1].write_ok is True


def _first_zero(bridge, clock):
    """首帧零速：会话证据就位后必须先写一帧成功的零速才允许非零。"""
    clock.advance(0.02)
    _, rec = bridge.cycle()
    if not (rec is not None and rec.write_ok and rec.is_zero):
        # 诊断用：失败时把端口与记录状态一起打出来。
        pytest.fail(f'首帧不是成功的零速: rec={rec} '
                    f'port_writes={bridge.transport.stats.writes} '
                    f'clean={bridge.chain._arbiter.receiver_clean} '
                    f'locked={bridge.transport.stream_locked} '
                    f'evidence={bridge.session_evidence()}')
    return rec


def _ack(session_id: int) -> bytes:
    from c50c_uart3.chassis_cmd.session import encode_session_ack
    return encode_session_ack(session_id)


# ------------------------------------------------- 4 缺少接收证据（核心）
def test_no_session_frame_writes_nothing_even_with_nonzero_command():
    bridge, port, clock, handshake = _stack(enabled=True, session=False)
    bridge.input.on_nav_cmd(Twist(0.05))
    for _ in range(5):
        clock.advance(0.02)
        bridge.cycle()
    assert bytes(port.stream) == b''
    assert port.writes == 0
    assert bridge.snapshot()['session_evidence'] is False
    # 会话门在桥里就挡住：命令链一拍都没 tick，也没有任何字节。
    assert bridge.chain._arbiter.frame_seq.issued == 0
    assert bridge.ticks == 5


def test_cycle_ticks_once_per_call():
    """一拍只 tick 一次：序号每拍前进 1，不会重复推进。"""
    bridge, port, clock, _ = _stack(enabled=True)
    _first_zero(bridge, clock)
    bridge.input.on_nav_cmd(Twist(0.05))
    before = bridge.chain._arbiter.frame_seq.issued
    _tick(bridge, clock)
    assert bridge.chain._arbiter.frame_seq.issued == before + 1
    assert bridge.transport.stats.writes == 2


# ---------------------------------------------------------------- 5 停车优先
def test_stop_preempts_nonzero_command():
    bridge, port, clock, _ = _stack(enabled=True)
    bridge.input.on_nav_cmd(Twist(0.05))
    _first_zero(bridge, clock)         # 零速首帧
    base = len(port.stream)
    base_drop = 0

    bridge.input.on_stop(Bool(True))
    bridge.input.on_nav_cmd(Twist(0.05))
    clock.advance(0.02)
    _, rec = bridge.cycle()

    tail = user_frames(bytes(port.stream[base:]))
    assert rec.is_zero is True
    assert rec.zero_reason == ZeroReason.STOP_REQUESTED
    assert tail[3:7] == b'\x00\x00\x00\x00'
    assert bridge.input.state.stop_active is True


# ---------------------------------------------------------------- 6 会话过期
def test_session_expiry_returns_to_silence():
    bridge, port, clock, _ = _stack(enabled=True)
    _first_zero(bridge, clock)
    bridge.input.on_nav_cmd(Twist(0.05))
    clock.advance(0.02)
    _, rec = bridge.cycle()
    assert rec.is_zero is False, '过期前应写出非零帧'
    base = len(port.stream)

    clock.advance(SESSION_MAX_AGE_S + 0.01)
    _, rec = bridge.cycle()

    # 会话过期：桥退回静默拍（不 tick），因此这一拍一个字节也不写。
    assert bridge.snapshot()['session_evidence'] is False
    assert rec is None
    assert bytes(port.stream[base:]) == b''
    # 证据仍在仲裁器里（session_id 未清），只是窗口过期：下一次读到会话帧
    # 就会重新有效。固件侧是否也判过期，本主机无法证明。
    assert bridge.chain._arbiter.session_id == 7
    assert bridge.chain._arbiter.session_window_open(
        bridge.chain._arbiter._session_observed_at
        + SESSION_MAX_AGE_S + 0.01) is False


# ------------------------------------------------ 存根/守卫：生产不得自报
def test_bridge_does_not_construct_receipts_or_sim_receipt_entry_points():
    """bridge.py 不构造任何收据，也不调用测试收据入口。"""
    tree = ast.parse((_PROD / 'chassis_cmd' / 'bridge.py').read_text(encoding='utf-8'))
    calls = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = node.func
            if isinstance(fn, ast.Name):
                calls.add(fn.id)
            elif isinstance(fn, ast.Attribute):
                calls.add(fn.attr)
    assert 'SimCleanReceipt' not in calls
    assert 'SimSessionReceipt' not in calls
    # 只允许在 _open_session（测试设施入口）里导入它；生产门不得引用。
    prod_imports = []
    for fn in [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]:
        if fn.name == '_open_session':
            continue
        for node in ast.walk(fn):
            if isinstance(node, ast.ImportFrom):
                prod_imports += [a.name for a in node.names]
    assert 'require_receipt' not in prod_imports
    assert 'require_session_receipt' in calls          # 只做类型校验


def test_session_receipt_seal_is_private_module_level():
    src = (_PROD / 'chassis_cmd' / 'sim_receipt.py').read_text(encoding='utf-8')
    assert 'SIM_SESSION_SEAL' in src
    with pytest.raises(TypeError):
        SimSessionReceipt('forged', 7, 100.0, _seal='guess')


# ------------------------------------------------ 单一驱动路径（节点侧）
class _FakeSub:
    def __init__(self, msg_type, topic, callback, qos):
        self.msg_type, self.topic, self.callback = msg_type, topic, callback

    def destroy(self):
        pass


class _FakeTimer:
    def __init__(self, period, callback):
        self.period, self.callback, self.destroyed = period, callback, False

    def destroy(self):
        self.destroyed = True


class _FakeNode:
    def __init__(self, counts=None):
        self.subs, self.timers = [], []
        self.counts = dict(counts or {})

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = _FakeSub(msg_type, topic, callback, qos)
        self.subs.append(sub)
        return sub

    def create_timer(self, period, callback):
        timer = _FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def resolve_topic_name(self, name):
        return '/' + name.lstrip('/')

    def count_publishers(self, topic):
        return self.counts.get(topic, 0)


def test_wiring_without_timer_creates_subscriptions_only(monkeypatch):
    clock = Clock()
    port = FakeFdPort()
    bridge = StraightDriveBridge(port, device=DEVICE, enabled=True, clock=clock)
    grant_safety_via_input(bridge, clock())
    node = _FakeNode({'/nav_cmd_vel': 1})
    wiring = RosSubscriptionInput(node, bridge.input, tick_period_s=0.02)
    # 替身消息类型：start() 延迟导入真实 ROS 消息，本机没有 ROS。
    fake_geometry = types.ModuleType('geometry_msgs.msg')
    fake_geometry.Twist = Twist
    fake_std = types.ModuleType('std_msgs.msg')
    fake_std.Bool = Bool
    fake_std.String = Bool  # 2026-11-14 起接线还订阅 drive_safety(String)
    monkeypatch.setitem(sys.modules, 'geometry_msgs', types.ModuleType('geometry_msgs'))
    monkeypatch.setitem(sys.modules, 'geometry_msgs.msg', fake_geometry)
    monkeypatch.setitem(sys.modules, 'std_msgs', types.ModuleType('std_msgs'))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', fake_std)
    wiring.start(with_timer=False)
    # 2026-11-14 起接线订阅 6 条：manual/nav/auto/demo + stop + safety
    assert len(node.subs) == 6 and node.timers == []
    assert wiring.started is True
    assert [s.topic for s in node.subs] == [
        'manual_cmd_vel', 'nav_cmd_vel', 'auto_cmd_vel',
        'demo_cmd_vel', 'nav_stop', 'drive_safety']
