"""WP2 回归：基线状态帧必须同时打开仲裁器的证据门和传输层的字节流。

缺口（任务书 1.2 第 1 条）：基线固件没有会话帧，唯一的接收方向证据是
24 字节状态帧。``StraightDriveBridge.notify_status_frame()`` 原来只更新
仲裁器，``OwnerSerialTransport`` 仍保持锁定，于是：

1. 写入被拒（``write refused: stream locked``）；
2. 命令链把这次失败按"流毒"上报，仲裁器永久静默（连零速帧都不再产生）。

本文件锁定修复后的顺序：状态帧到达 → 传输层解锁 → 首帧零速 → 非零。

不打开真实串口：端口用测试替身，不产生任何系统调用。
"""
from __future__ import annotations

import types

import pytest

from c50c_uart3.chassis_cmd.arbiter import PROTOCOL_BASELINE, PROTOCOL_V3
from c50c_uart3.chassis_cmd.baseline import (encode_baseline_speed_frame,
                                            encode_baseline_zero_frame)
from c50c_uart3.chassis_cmd.bridge import StraightDriveBridge
from c50c_uart3.chassis_cmd.serial_port import FakeFdPort
from sim_confirm import grant_safety_via_input

DEVICE = '/dev/pts/9'


class Clock:
    """可控单调时钟。"""

    def __init__(self, t: float = 500.0) -> None:
        self.t = float(t)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += float(dt)


class _ShortWritePort(FakeFdPort):
    """模拟部分写入：声称写成功但字节数不足，即帧被截断。"""

    def write_bytes(self, data: bytes) -> int:
        self.writes += 1
        return max(0, len(data) - 1)


class _ShortWriteOncePort(FakeFdPort):
    """只短写第一次，之后正常。

    用于区分"每次写都短写（锁被不断刷新）"与"真短写一次后残帧证据
    永久保留"这两种情况——后者才是生产语义。
    """

    def __init__(self, device: str = DEVICE) -> None:
        super().__init__(device)
        self.calls = 0

    def write_bytes(self, data: bytes) -> int:
        self.calls += 1
        if self.calls == 1:
            self.writes += 1
            self.stream += data[:5]
            return 5
        self.writes += 1
        self.stream += data
        return len(data)


def _status_frame() -> bytes:
    """构造一帧结构合法的 48 字节状态帧（内容全零 + 正确 BCC）。

    task-15 把状态帧从 24 字节扩到 48 字节：BCC 移到位 46，帧尾移到位 47。
    """
    frame = bytearray(48)
    frame[0] = 0x7B
    frame[47] = 0x7D
    bcc = 0
    for b in frame[:46]:
        bcc ^= b
    frame[46] = bcc
    return bytes(frame)


def _twist(vx: float, vy: float = 0.0, wz: float = 0.0):
    return types.SimpleNamespace(
        linear=types.SimpleNamespace(x=vx, y=vy, z=0.0),
        angular=types.SimpleNamespace(x=0.0, y=0.0, z=wz))


def _bridge(clock, protocol=PROTOCOL_BASELINE, port=None):
    port = port if port is not None else FakeFdPort(DEVICE)
    bridge = StraightDriveBridge(port, device=DEVICE, enabled=True,
                                 tick_period_s=0.02, clock=clock,
                                 firmware_protocol=protocol)
    bridge.input.set_nav_publishers(1)
    bridge.input.set_demo_publishers(0)
    grant_safety_via_input(bridge, clock())
    return bridge, port


def _bcc_ok(frame: bytes) -> bool:
    bcc = 0
    for b in frame[:9]:
        bcc ^= b
    return frame[9] == bcc


# ------------------------------------------------------------------ 解锁
def test_baseline_status_frame_unlocks_transport():
    """状态帧到达前传输层锁定；到达后解锁，且通知本身不写字节。"""
    clock = Clock()
    bridge, port = _bridge(clock)
    assert bridge.transport.stream_locked is True
    assert bridge.transport.receiver_synced is False

    bridge.notify_status_frame()

    assert bridge.transport.stream_locked is False
    assert bridge.transport.receiver_synced is True
    assert bytes(port.stream) == b''      # 解锁不是靠写成功换来的


def test_short_write_latches_and_later_status_frame_cannot_wash_out():
    """一次真短写之后，后续状态帧不得洗白：帧边界错位不可恢复。

    与上一个用例的区别：这里端口只短写**一次**，之后正常。若实现把
    "收到状态帧"当解除条件，则第二次写会成功并在这里暴露。
    """
    clock = Clock()
    bridge, port = _bridge(clock, port=_ShortWriteOncePort(DEVICE))
    bridge.notify_status_frame()
    assert bridge.transport.stream_locked is False

    clock.advance(0.02)
    _gated, first = bridge.cycle()        # 首帧零速 → 短写 5/11
    assert first is not None and first.write_ok is False
    assert bridge.transport.stream_locked is True
    reason = bridge.transport.stream_lock_reason
    assert reason is not None and '5/11' in reason
    after_short = len(port.stream)

    bridge.notify_status_frame()          # 再收到状态帧
    assert bridge.transport.stream_locked is True, '短写证据不得被状态帧清除'

    clock.advance(0.02)
    bridge.input.on_nav_cmd(_twist(0.05))
    clock.advance(0.02)
    _gated, record = bridge.cycle()
    assert len(port.stream) == after_short, '短写后不得再写出任何字节'
    assert record is None or record.frame is None


def test_status_frame_bytes_are_verified_when_supplied():
    """给字节时本类自己复核：合法帧解锁，坏帧不解锁。"""
    clock = Clock()
    bridge, port = _bridge(clock)

    bad_tail = bytearray(_status_frame())
    bad_tail[47] = 0x7C
    bridge.notify_status_frame(bytes(bad_tail))
    assert bridge.transport.stream_locked is True, '帧尾错不得解锁'

    bad_bcc = bytearray(_status_frame())
    bad_bcc[46] ^= 0xFF
    bridge.notify_status_frame(bytes(bad_bcc))
    assert bridge.transport.stream_locked is True, 'BCC 错不得解锁'

    short = _status_frame()[:47]
    bridge.notify_status_frame(short)
    assert bridge.transport.stream_locked is True, '长度不足不得解锁'

    bridge.notify_status_frame(_status_frame())
    assert bridge.transport.stream_locked is False
    assert bytes(port.stream) == b''


def test_short_write_cannot_be_washed_by_verified_bytes_either():
    """给了合法字节也一样：短写证据优先于任何状态帧证据。"""
    clock = Clock()
    bridge, port = _bridge(clock, port=_ShortWriteOncePort(DEVICE))
    bridge.notify_status_frame(_status_frame())
    clock.advance(0.02)
    bridge.cycle()                        # 短写锁流
    assert bridge.transport.stream_locked is True
    bridge.notify_status_frame(_status_frame())
    assert bridge.transport.stream_locked is True


def test_v3_notification_raises_and_keeps_stream_locked():
    """v3 下状态帧不是会话证据：抛错，且传输层不被放开。"""
    clock = Clock()
    bridge, port = _bridge(clock, protocol=PROTOCOL_V3)
    assert bridge.transport.stream_locked is True

    with pytest.raises(RuntimeError):
        bridge.notify_status_frame()

    assert bridge.transport.stream_locked is True
    assert bridge.transport.receiver_synced is False
    assert bytes(port.stream) == b''


# ------------------------------------------------------------------ 出帧顺序
def test_notification_then_zero_frame_then_nonzero():
    """通知之后：第一帧零速，第二帧才是非零（首帧零速规则）。"""
    clock = Clock()
    bridge, port = _bridge(clock)
    bridge.notify_status_frame()

    clock.advance(0.02)
    _gated, first = bridge.cycle()
    assert first is not None and first.write_ok is True
    assert first.is_zero is True
    stream = bytes(port.stream)
    assert stream == encode_baseline_zero_frame()

    clock.advance(0.02)
    bridge.input.on_nav_cmd(_twist(0.05))
    clock.advance(0.02)
    _gated, second = bridge.cycle()
    assert second is not None and second.write_ok is True
    assert second.is_zero is False
    tail = bytes(port.stream)[len(stream):]
    assert tail == encode_baseline_speed_frame(0.05, 0.0, 0.0)
    assert _bcc_ok(tail) and tail[1] == 0 and tail[2] == 0
    assert tail[3:5] == (50).to_bytes(2, 'big', signed=True)


def test_stale_status_window_stops_output_without_poisoning_stream():
    """状态帧过期只关门，不锁流；新状态帧到达后可恢复。"""
    clock = Clock()
    bridge, port = _bridge(clock)
    bridge.notify_status_frame()
    clock.advance(0.02)
    bridge.cycle()

    clock.advance(3.5)                    # 超出 3 s 窗口
    bridge.input.on_nav_cmd(_twist(0.08))
    clock.advance(0.02)
    written_before = len(port.stream)
    gated, record = bridge.cycle()
    assert gated is True and record is None
    assert len(port.stream) == written_before
    assert bridge.transport.stream_locked is False
    assert bridge.chain._arbiter.stream_poisoned is False

    bridge.notify_status_frame()          # 固件恢复正常发帧
    clock.advance(0.02)
    bridge.input.on_nav_cmd(_twist(0.08))
    clock.advance(0.02)
    _gated, record = bridge.cycle()
    assert record is not None and record.write_ok is True
    tail = bytes(port.stream)[written_before:]
    assert tail == encode_baseline_speed_frame(0.08, 0.0, 0.0)


# ------------------------------------------------------- R2：调用点静态守卫
def test_only_the_frozen_node_calls_notify_status_frame():
    """生产代码里 ``notify_status_frame(`` 的调用点只允许 uart3_node.py 一处。

    理由：不传 ``frame=`` 的形态依赖"调用方已用节点解析器校验过字节"这一
    前提（见 ``bridge.notify_status_frame`` 文档）。``uart3_node.py`` 本轮
    冻结，所以只能维持无参形态；一旦别处新增调用点，那个前提就没有保证了。
    冻结解除后把节点改成传 ``frame=``，本守卫也应随之放宽或删除。
    """
    import ast
    from pathlib import Path

    import c50c_uart3

    pkg = Path(c50c_uart3.__file__).resolve().parent
    allowed = {pkg / 'uart3_node.py'}
    allowed_nodes = {pkg / 'chassis_cmd' / 'bridge.py'}    # 定义处
    offenders = []
    for path in sorted(pkg.rglob('*.py')):
        if path in allowed_nodes:
            continue
        tree = ast.parse(path.read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            name = getattr(func, 'attr', None) or getattr(func, 'id', None)
            if name != 'notify_status_frame':
                continue
            if path in allowed:
                # 冻结的节点必须仍是"无参"形态；改成传 frame= 时本断言会提醒
                # 复核者更新本守卫。
                assert node.args == [] and node.keywords == [], (
                    f'{path.name}:{node.lineno} 传了参数，请同步更新本守卫')
                continue
            offenders.append(f'{path.relative_to(pkg)}:{node.lineno}')
    assert offenders == [], offenders
