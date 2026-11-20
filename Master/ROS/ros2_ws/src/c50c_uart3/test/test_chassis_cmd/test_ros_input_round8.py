"""第八轮 C：禁用/重启用语义、发布者计数门控、订阅接线的定向反例。

全部纯内存，无 ROS、无真实串口。测试里出现的"独立证据"（端口干净、
固件确认）都只由测试设施构造，适配层不提供也不转调这些入口。
"""
import ast
import io

import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame, ros_input
from c50c_uart3.chassis_cmd.arbiter import Arbiter, ZeroReason
from c50c_uart3.chassis_cmd.chain import StraightCommandChain
from c50c_uart3.chassis_cmd.ros_input import (RosStraightInput,
                                              RosSubscriptionInput)
from sim_confirm import (mark_chain_clean, mark_endpoint_clean,
                         new_receipt as _new_receipt,
                         grant_safety, TEST_SAFETY_MAX_AGE_S)

_SIM = _new_receipt('ros-input')
from c50c_uart3.chassis_cmd.transport import MemoryTransport

ZERO = encode_speed_frame(0.0, 0.0, 0.0)

def zero(seq: int = 0) -> bytes:
    """协议 v3 的零速帧：buffer[2] = 0x80 | 序号。"""
    return encode_speed_frame(0.0, 0.0, 0.0, seq=seq)



class _Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)


class _Twist:
    def __init__(self, vx=0.0, vy=0.0, wz=0.0):
        self.linear = _Vec(vx, vy, 0.0)
        self.angular = _Vec(0.0, 0.0, wz)


class _Bool:
    def __init__(self, data):
        self.data = data


class _Clock:
    def __init__(self, start=100.0):
        self.t = float(start)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)
        return self.t


@pytest.fixture
def clock():
    return _Clock()


@pytest.fixture
def port():
    p = MemoryTransport()
    mark_endpoint_clean(p, receipt=_SIM)
    return p


@pytest.fixture
def chain(port, clock):
    arbiter = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5,
                      safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arbiter, clock())
    return StraightCommandChain(arbiter, port)


@pytest.fixture
def ros(chain, clock):
    return RosStraightInput(chain, enabled=True, clock=clock)


def _authorise(chain, clock):
    mark_chain_clean(chain, receipt=_SIM)
    chain.firmware_confirm(True)
    rec = chain.tick(clock())
    assert rec.is_zero and rec.frame == ZERO


# ------------------------------------------------- 1 禁用 / 重新启用
def test_disable_stops_old_nonzero_from_being_output(ros, chain, clock):
    """反例：先给 vx=0.05，再 disable()，下一拍不得输出非零。"""
    ros.set_nav_publishers(1)
    ros.set_demo_publishers(0)
    _authorise(chain, clock)
    ros.on_nav_cmd(_Twist(vx=0.05))
    ros.disable()
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert rec.is_zero, rec.frame
    assert chain.nonzero_frames() == []


def test_disable_then_enable_does_not_restore_old_command(ros, chain, clock):
    """重新启用也不得恢复禁用前的命令。"""
    ros.set_nav_publishers(1)
    _authorise(chain, clock)
    ros.on_nav_cmd(_Twist(vx=0.05))
    ros.disable()
    ros.enable()
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert rec.is_zero
    assert chain.nonzero_frames() == []
    # 启用之后新到的命令才可能被输出
    ros.on_nav_cmd(_Twist(vx=0.04))
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert not rec.is_zero
    # 序号：授权零速(0) → 禁用期零速(1) → 本条命令(2)。
    assert rec.frame == encode_speed_frame(0.04, 0.0, 0.0, seq=2)


def test_stop_priority_survives_disable_enable(ros, chain, clock):
    """停车激活期间禁用再启用，仍不得输出非零。"""
    ros.set_nav_publishers(1)
    _authorise(chain, clock)
    ros.on_stop(_Bool(True))
    ros.on_nav_cmd(_Twist(vx=0.05))
    ros.disable()
    ros.enable()
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert rec.is_zero and rec.zero_reason == ZeroReason.STOP_REQUESTED
    assert chain.nonzero_frames() == []


def test_cycle_gate_blocks_bytes_while_disabled(ros, chain, clock):
    """禁用期间周期回调不写任何字节。"""
    ros.set_nav_publishers(1)
    _authorise(chain, clock)
    before = len(chain.records)
    ros.disable()
    gated, rec = ros.on_cycle(lambda: chain.tick(clock()))
    assert gated is True and rec is None
    assert len(chain.records) == before


# ------------------------------------------------- 2 发布者计数
def test_unknown_publishers_block_nonzero(ros, chain, clock):
    """发布者数未知（默认 0 且未确认）时输入命令也不得放行非零。"""
    _authorise(chain, clock)
    assert ros.on_nav_cmd(_Twist(vx=0.05)) is False
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert rec.is_zero
    assert chain.nonzero_frames() == []
    assert ros.snapshot()['publishers_known'] is False


def test_single_publisher_allows_nonzero(ros, chain, clock):
    """一个发布者是可放行区间（对照）。"""
    ros.set_nav_publishers(1)
    _authorise(chain, clock)
    assert ros.on_nav_cmd(_Twist(vx=0.05)) is True
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert not rec.is_zero


def test_publisher_drop_to_zero_revokes_and_does_not_resurrect(ros, chain,
                                                               clock):
    """1 → 0 后不得输出；恢复为 1 也不复活先前命令。"""
    ros.set_nav_publishers(1)
    _authorise(chain, clock)
    ros.on_nav_cmd(_Twist(vx=0.05))
    ros.set_nav_publishers(0)
    clock.advance(0.02)
    assert chain.tick(clock()).is_zero
    ros.set_nav_publishers(1)
    clock.advance(0.02)
    assert chain.tick(clock()).is_zero
    # 恢复之后的新命令仍可放行
    ros.on_nav_cmd(_Twist(vx=0.03))
    clock.advance(0.02)
    assert not chain.tick(clock()).is_zero
    # 序号：授权零速(0) → 两次来源不明零速(1,2) → 本条命令(3)。
    assert chain.nonzero_frames() == [encode_speed_frame(0.03, 0.0, 0.0, seq=3)]


def test_multiple_publishers_block_nonzero(ros, chain, clock):
    """多于一个发布者：仲裁器判定来源不明。"""
    ros.set_nav_publishers(1)
    _authorise(chain, clock)
    ros.on_nav_cmd(_Twist(vx=0.05))
    ros.set_nav_publishers(2)
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert rec.is_zero
    assert chain.nonzero_frames() == []


def test_publishers_unknown_flag_blocks_cycle(ros, chain, clock):
    ros.set_nav_publishers(1)
    _authorise(chain, clock)
    ros.on_nav_cmd(_Twist(vx=0.05))
    ros.publishers_unknown()
    gated, rec = ros.on_cycle(lambda: chain.tick(clock()))
    assert gated is True and rec is None
    assert chain.nonzero_frames() == []


# ------------------------------------------------- 3 重连
def test_reconnect_revokes_licence(ros, chain, clock):
    ros.set_nav_publishers(1)
    _authorise(chain, clock)
    ros.on_nav_cmd(_Twist(vx=0.05))
    ros.on_serial_reconnect()
    assert ros.cmd_licensed('nav') is False


# ------------------------------------------------- 4 订阅接线候选
class _FakeSub:
    def __init__(self, msg_type, topic, callback, qos):
        self.msg_type, self.topic, self.callback, self.qos = (
            msg_type, topic, callback, qos)
        self.destroyed = False

    def destroy(self):
        self.destroyed = True


class _FakeTimer:
    def __init__(self, period, callback):
        self.period, self.callback = period, callback
        self.destroyed = False

    def destroy(self):
        self.destroyed = True


class _FakeNode:
    """最小 rclpy 节点替身：记录订阅、定时器与发布者数量查询。"""

    def __init__(self, counts=None):
        self.subs = []
        self.timers = []
        self.counts = dict(counts or {})
        self.raise_on_count = False

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
        if self.raise_on_count:
            raise RuntimeError('graph query failed')
        return self.counts.get(topic, 0)


def _wire(node, adapter, **kw):
    return RosSubscriptionInput(node, adapter, **kw)


def test_subscription_wiring_is_explicit_and_default_off(chain, clock):
    node = _FakeNode()
    adapter = RosStraightInput(chain, enabled=False, clock=clock)
    wiring = _wire(node, adapter)
    assert wiring.started is False
    assert node.subs == [] and node.timers == []
    assert wiring.snapshot()['started'] is False


def test_subscription_wiring_disabled_adapter_writes_nothing(chain, clock):
    """接线已建，但适配层关闭时周期回调不写字节。"""
    node = _FakeNode({'/nav_cmd_vel': 1})
    adapter = RosStraightInput(chain, enabled=False, clock=clock)
    wiring = _wire(node, adapter)
    wiring.cycle()
    assert wiring.ticks_run == 0
    assert chain.records == []


def test_graph_query_failure_treated_as_unknown(chain, clock):
    node = _FakeNode({'/nav_cmd_vel': 1})
    node.raise_on_count = True
    adapter = RosStraightInput(chain, enabled=True, clock=clock)
    wiring = _wire(node, adapter)
    wiring.refresh_publishers()
    assert adapter.snapshot()['publishers_known'] is False
    gated, _ = adapter.on_cycle(lambda: chain.tick(clock()))
    assert gated is True


def test_wiring_imports_no_serial_and_no_confirm_helpers():
    """生产接线模块不得引入串口打开点或模拟确认入口。"""
    src = io.open(ros_input.__file__, encoding='utf-8').read()
    tree = ast.parse(src)
    called = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = node.func
            called.add(getattr(fn, 'attr', None) or getattr(fn, 'id', None))
    for name in ('mark_clean_state_verified', 'firmware_confirm',
                 'mark_endpoint_clean_for_test', 'on_firmware_confirm',
                 'open_transport', 'mark_receiver_clean'):
        assert name not in called, name
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [a.name for a in node.names] + [getattr(node, 'module', '') or '']
            assert not any('serial' in n for n in names), names
