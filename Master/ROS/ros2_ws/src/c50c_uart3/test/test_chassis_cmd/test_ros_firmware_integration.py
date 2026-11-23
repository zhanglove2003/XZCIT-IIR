"""总任务 B 定向集成测试：ROS → 仲裁 → 编码 → 伪终端 → 真实固件模型。

只使用隔离伪终端与主机编译的真实解析器；不打开任何真实串口。
"""
from __future__ import annotations

import os
import signal
import sys
import tty
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'firmware_harness'))

from fw_parser_harness import FirmwareModel          # noqa: E402
from c50c_uart3.chassis_cmd import is_zero_frame     # noqa: E402
from c50c_uart3.chassis_cmd.bridge import StraightDriveBridge   # noqa: E402
from c50c_uart3.chassis_cmd.serial_port import PTS_PATTERN       # noqa: E402
from sim_confirm import (mark_endpoint_clean, new_receipt,       # noqa: E402
                         grant_safety_via_input)


class Clock:
    def __init__(self) -> None:
        self.t = 100.0

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


class PtyBridgePort:
    """把 writes 落到隔离伪终端主端；固件侧从 O_RDWR 打开的从端读。"""

    __slots__ = ('master', 'reader', 'device', 'writes', 'bytes_written',
                 'refused')

    def __init__(self, master: int, reader: int, device: str) -> None:
        self.master = master
        self.reader = reader
        self.device = device
        self.writes = 0
        self.bytes_written = 0
        self.refused = 0

    def write_bytes(self, data: bytes) -> int:
        self.writes += 1
        try:
            n = os.write(self.master, data)
        except OSError:
            return 0
        self.bytes_written += n
        return n

    def read_some(self, size: int = 256) -> bytes:
        import select
        out = bytearray()
        while True:
            ready, _, _ = select.select([self.reader], [], [], 0.05)
            if not ready:
                return bytes(out)
            try:
                chunk = os.read(self.reader, size)
            except OSError:
                return bytes(out)
            if not chunk:
                return bytes(out)
            out += chunk

    def reopen(self) -> None:
        return None


@pytest.fixture()
def pty_pair():
    # 本机内核实测：接收端必须以 O_RDWR 打开从端，否则主端写入的字节不会
    # 挂到读队列，读取会一直阻塞（见 pty_bridge_run.py 的同一注释）。
    master, slave = os.openpty()
    tty.setraw(master)
    device = os.ttyname(slave)
    reader = os.open(device, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(reader)
    assert PTS_PATTERN.match(device)
    old = signal.signal(signal.SIGHUP, signal.SIG_IGN)
    try:
        yield master, reader, device
    finally:
        signal.signal(signal.SIGHUP, old)
        for fd in (reader, slave, master):
            try:
                os.close(fd)
            except OSError:
                pass


@pytest.fixture()
def firmware():
    model = FirmwareModel.start()
    try:
        yield model
    finally:
        model.close()


def _run(model: FirmwareModel, data: bytes) -> None:
    for i in range(0, len(data), 11):
        model.feed(data[i:i + 11])
    model.tick(1)


def test_normal_drive_reaches_firmware(pty_pair, firmware):
    master, reader, device = pty_pair
    port = PtyBridgePort(master, reader, device)
    clock = Clock()
    bridge = StraightDriveBridge(port, device=device, enabled=True, clock=clock)
    receipt = new_receipt('pty-integ')
    mark_endpoint_clean(port, receipt=receipt)
    bridge.open_session(receipt)
    bridge.input.set_nav_publishers(1)
    grant_safety_via_input(bridge, clock())
    grant_safety_via_input(bridge, clock())

    clock.advance(0.02)
    gated, rec = bridge.cycle()
    assert gated is False and rec.is_zero is True
    _run(firmware, port.read_some())
    state = firmware.state
    assert state.armed is True and state.vx == 0.0

    bridge.input.on_nav_cmd(Twist(0.05))
    clock.advance(0.02)
    bridge.cycle()
    _run(firmware, port.read_some())
    assert firmware.state.vx == pytest.approx(0.05, abs=1e-6)


def test_zero_rate_command_is_zero_on_wire(pty_pair, firmware):
    master, reader, device = pty_pair
    port = PtyBridgePort(master, reader, device)
    clock = Clock()
    bridge = StraightDriveBridge(port, device=device, enabled=True, clock=clock)
    receipt = new_receipt('pty-zero')
    mark_endpoint_clean(port, receipt=receipt)
    bridge.open_session(receipt)
    bridge.input.set_nav_publishers(1)
    grant_safety_via_input(bridge, clock())
    received = b''
    for _ in range(3):
        bridge.input.on_nav_cmd(Twist(0.0))
        clock.advance(0.02)
        bridge.cycle()
        received += port.read_some()
    assert received and is_zero_frame(received[:11])
    _run(firmware, received)
    assert firmware.state.armed is True


def test_stop_and_release_do_not_revive_old_command(pty_pair, firmware):
    master, reader, device = pty_pair
    port = PtyBridgePort(master, reader, device)
    clock = Clock()
    bridge = StraightDriveBridge(port, device=device, enabled=True, clock=clock)
    receipt = new_receipt('pty-stop')
    mark_endpoint_clean(port, receipt=receipt)
    bridge.open_session(receipt)
    bridge.input.set_nav_publishers(1)
    grant_safety_via_input(bridge, clock())
    clock.advance(0.02)
    bridge.cycle()
    bridge.input.on_nav_cmd(Twist(0.05))
    clock.advance(0.02)
    bridge.cycle()
    _run(firmware, port.read_some())
    assert firmware.state.vx == pytest.approx(0.05, abs=1e-6)

    bridge.input.on_stop(Bool(True))
    clock.advance(0.02)
    bridge.cycle()
    _run(firmware, port.read_some())
    assert firmware.state.vx == 0.0

    bridge.input.on_stop(Bool(False))
    for _ in range(3):
        clock.advance(0.02)
        bridge.cycle()
        data = port.read_some()
        if data:
            assert is_zero_frame(data), '停车解除后不得再发旧的非零命令'
    assert firmware.state.vx == 0.0


def test_expiry_keeps_firmware_at_zero(pty_pair, firmware):
    master, reader, device = pty_pair
    port = PtyBridgePort(master, reader, device)
    clock = Clock()
    bridge = StraightDriveBridge(port, device=device, enabled=True, clock=clock)
    receipt = new_receipt('pty-expire')
    mark_endpoint_clean(port, receipt=receipt)
    bridge.open_session(receipt)
    bridge.input.set_nav_publishers(1)
    grant_safety_via_input(bridge, clock())
    bridge.input.on_nav_cmd(Twist(0.05))
    clock.advance(0.02)
    bridge.cycle()
    _run(firmware, port.read_some())
    clock.advance(1.0)
    bridge.cycle()
    _run(firmware, port.read_some())
    assert firmware.state.vx == 0.0


def test_default_off_writes_nothing(pty_pair):
    master, reader, device = pty_pair
    port = PtyBridgePort(master, reader, device)
    bridge = StraightDriveBridge(port, device=device, clock=Clock())
    bridge.input.on_nav_cmd(Twist(0.05))
    for _ in range(4):
        bridge.cycle()
    assert port.bytes_written == 0


def test_real_uart_path_is_rejected():
    from c50c_uart3.chassis_cmd.serial_port import FdPort, RealFdUseForbidden
    with pytest.raises(RealFdUseForbidden):
        FdPort(0, '/dev/ttyAMA0')
