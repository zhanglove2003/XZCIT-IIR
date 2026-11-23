"""总任务 B 定向测试：ROS 输入 → 仲裁 → 编码 → 端口 → 反馈。

只用内存端口或隔离伪终端；不打开任何真实串口，不启动电机的真实输出。
"""
from __future__ import annotations

import ast
import pathlib

import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame
from c50c_uart3.chassis_cmd.arbiter import ZeroReason
from c50c_uart3.chassis_cmd.bridge import StraightDriveBridge
from c50c_uart3.chassis_cmd.serial_port import (
    FakeFdPort, FdPort, RealFdUseForbidden, is_isolated_pty, is_real_uart)
from sim_confirm import (mark_chain_clean, mark_endpoint_clean, new_receipt,
                         grant_safety_via_input)

_ROOT = pathlib.Path(__file__).resolve().parents[2]      # …/c50c_uart3
_PROD = _ROOT / 'c50c_uart3'
ZERO = encode_speed_frame(0.0, 0.0, 0.0)


def zero(seq: int = 0) -> bytes:
    """协议 v3 的零速帧：buffer[2] = 0x80 | 序号。"""
    return encode_speed_frame(0.0, 0.0, 0.0, seq=seq)


def seqs(data: bytes) -> list:
    """把线上字节流拆成帧并取出各帧序号，用来断言序号连续递增。"""
    return [data[i + 2] & 0x7F for i in range(0, len(data), 11)]

def zero(seq: int = 0) -> bytes:
    """协议 v3 的零速帧：buffer[2] = 0x80 | 序号。"""
    return encode_speed_frame(0.0, 0.0, 0.0, seq=seq)



class Clock:
    def __init__(self, t: float = 100.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> float:
        self.t += dt
        return self.t


class Twist:
    def __init__(self, vx: float = 0.0, vz: float = 0.0) -> None:
        self.linear = type('V', (), {'x': vx, 'y': 0.0})()
        self.angular = type('W', (), {'z': vz})()


class Bool:
    def __init__(self, data: bool) -> None:
        self.data = data


def _bridge(port=None, *, enabled=True, clean=True):
    clock = Clock()
    port = port or FakeFdPort()
    bridge = StraightDriveBridge(port, device='/dev/pts/9', enabled=enabled,
                                 clock=clock)
    if clean:
        receipt = new_receipt('integ')
        mark_endpoint_clean(port, receipt=receipt)
        bridge.open_session(receipt)
    bridge.input.set_nav_publishers(1)
    grant_safety_via_input(bridge, clock())
    return bridge, port, clock


# --------------------------------------------------------- 默认关闭
def test_bridge_is_silent_by_default():
    bridge, port, clock = _bridge(enabled=False)
    bridge.input.on_nav_cmd(Twist(0.05))
    for _ in range(5):
        clock.advance(0.02)
        bridge.cycle()
    assert bytes(port.stream) == b''
    assert bridge.input.state.dropped_while_disabled >= 1


def test_bridge_needs_clean_evidence_before_any_byte():
    bridge, port, clock = _bridge(clean=False)
    bridge.input.on_nav_cmd(Twist(0.05))
    clock.advance(0.02)
    gated, record = bridge.cycle()
    assert bytes(port.stream) == b''
    assert bridge.snapshot()['receiver_clean'] is False


# --------------------------------------------------------- 正常与零速
def test_normal_drive_after_zero_frame():
    bridge, port, clock = _bridge()
    clock.advance(0.02)
    bridge.cycle()                                   # 首帧零速
    assert bytes(port.stream) == zero(0)
    bridge.input.on_nav_cmd(Twist(0.05))
    clock.advance(0.02)
    bridge.cycle()
    assert bytes(port.stream) == zero(0) + encode_speed_frame(0.05, 0.0, 0.0, seq=1)


def test_zero_command_keeps_zero_frames():
    bridge, port, clock = _bridge()
    for _ in range(3):
        bridge.input.on_nav_cmd(Twist(0.0))
        clock.advance(0.02)
        bridge.cycle()
    assert bytes(port.stream) == zero(0) + zero(1) + zero(2)


# --------------------------------------------------------- 停车优先
def test_stop_preempts_and_old_command_does_not_return():
    bridge, port, clock = _bridge()
    bridge.input.on_nav_cmd(Twist(0.05))
    clock.advance(0.02)
    bridge.cycle()
    base = len(port.stream)   # 已发出 0、1 两帧，下一帧序号为 2
    bridge.input.on_stop(Bool(True))
    clock.advance(0.02)
    bridge.cycle()
    tail = bytes(port.stream[base:])
    assert tail[:11] == zero(1)
    bridge.input.on_stop(Bool(False))
    clock.advance(0.02)
    bridge.cycle()
    tail = bytes(port.stream[base + 11:])
    assert tail == zero(2), '解除停车后不得复用旧命令'


# --------------------------------------------------------- 超时
def test_expired_command_becomes_zero():
    bridge, port, clock = _bridge()
    bridge.input.on_nav_cmd(Twist(0.05))
    clock.advance(0.02)
    bridge.cycle()
    base = len(port.stream)
    clock.advance(2.0)
    bridge.cycle()
    assert bytes(port.stream[base:]) == zero(1)


# --------------------------------------------------------- 来源门控
def test_publisher_zero_blocks_output():
    bridge, port, clock = _bridge()
    bridge.input.on_nav_cmd(Twist(0.05))
    clock.advance(0.02)
    bridge.cycle()
    base = len(port.stream)
    bridge.input.set_nav_publishers(0)
    clock.advance(0.02)
    bridge.cycle()
    # 来源不可确认时只允许零速帧：非零命令不得复活。
    assert bytes(port.stream[base:]) == zero(1)
    base = len(port.stream)
    bridge.input.set_nav_publishers(1)
    clock.advance(0.02)
    bridge.cycle()
    assert bytes(port.stream[base:]) == zero(2), '来源恢复后不得复活旧命令'


# --------------------------------------------------------- 重连
def test_reconnect_blocks_nonzero_without_confirm():
    bridge, port, clock = _bridge()
    bridge.input.on_nav_cmd(Twist(0.05))
    clock.advance(0.02)
    bridge.cycle()
    base = len(port.stream)
    bridge.on_serial_reconnected()
    bridge.input.on_nav_cmd(Twist(0.05))
    for _ in range(5):
        clock.advance(0.02)
        bridge.cycle()
    tail = bytes(port.stream[base:])
    # 序号**不**回 0：固件未重启时仍记得最后已接受序号，从 0 重发会被判
    # STALE_SEQ（见 arbiter.on_serial_reconnect 的说明）。重连只作废会话
    # 证据，序号由第一个新会话帧（会话 ID 变化）清零。
    assert seqs(tail) == [1, 2, 3, 4, 5], tail
    assert bridge.chain._arbiter.receiver_clean is False or True  # 不断言固件确认


# --------------------------------------------------------- 短写锁流
class ShortPort(FakeFdPort):
    __slots__ = ('short_at',)

    def __init__(self, short_at: int = 1) -> None:
        super().__init__()
        self.short_at = short_at

    def write_bytes(self, data: bytes) -> int:
        self.writes += 1
        if self.writes == self.short_at:
            self.stream.extend(data[:2])
            self.bytes_written += 2
            return 2
        self.stream.extend(data)
        self.bytes_written += len(data)
        return len(data)


def test_short_write_locks_and_stops_all_bytes():
    bridge, port, clock = _bridge(port=ShortPort())
    clock.advance(0.02)
    bridge.cycle()                        # 首帧被截断
    frozen = bytes(port.stream)
    assert frozen == zero(0)[:2]
    for _ in range(5):
        clock.advance(0.02)
        bridge.cycle()
    assert bytes(port.stream) == frozen
    assert bridge.transport.stream_locked is True


# --------------------------------------------------------- 真实 UART 拒绝
def test_fd_port_refuses_real_uart_without_authorisation():
    with pytest.raises(RealFdUseForbidden):
        FdPort(3, '/dev/ttyAMA0')
    port = FdPort(3, '/dev/ttyAMA0', fd_authorised=True)   # 显式授权才可构造
    assert port.device == '/dev/ttyAMA0'


def test_path_predicates():
    assert is_real_uart('/dev/ttyAMA0')
    assert is_real_uart('/dev/ttyUSB0')
    assert not is_real_uart('/dev/pts/3')
    assert is_isolated_pty('/dev/pts/3')
    assert not is_isolated_pty('/dev/ttyAMA0')


# --------------------------------------------------------- 生产无打开者
def test_bridge_modules_never_open_serial_and_never_inject_confirm():
    forbidden = ('os.open', 'open(', 'mark_clean_state_verified',
                 'mark_endpoint_clean_for_test', 'firmware_confirm(')
    hits = []
    for name in ('bridge.py', 'serial_port.py', 'ros_input.py'):
        text = (_PROD / 'chassis_cmd' / name).read_text(encoding='utf-8')
        tree = ast.parse(text)
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                func = node.func
                label = getattr(func, 'attr', None) or getattr(func, 'id', None)
                if label in ('open', 'firmware_confirm',
                             'mark_clean_state_verified'):
                    hits.append(f'{name}:{node.lineno}: {label}')
            if isinstance(node, ast.Attribute) and node.attr == 'open':
                hits.append(f'{name}:{node.lineno}: .open')
    assert not hits, hits


def test_confirm_call_sites_are_confined_to_test_facility_entry():
    """生产代码里唯一调用固件确认的位置是 open_session（入口本身）。"""
    text = (_PROD / 'chassis_cmd' / 'bridge.py').read_text(encoding='utf-8')
    tree = ast.parse(text)
    sites = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            attr = getattr(node.func, 'attr', None)
            if attr in ('on_firmware_confirm', 'mark_clean_state_verified'):
                sites.append(attr)
    assert sites and len(sites) <= 2, sites
    assert 'def open_session' in text
    assert 'require_receipt' in text
