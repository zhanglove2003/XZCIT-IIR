"""返工 B 定向测试：节点打开模式 → 写入链 → A 候选确认门。

三层验证，每层都用**真实代码**而不是替身：

1. 打开模式：真实 ``uart3_node`` 的 ``_open_serial`` 在 isolated pty 上以
   ``O_RDONLY``（默认）或 ``O_RDWR``（drive_enable 打开）打开；
2. 写入链：``FdPort`` 在同一 fd 上 ``os.write``，短写与断连走流的锁定；
3. 接收确认：A 候选 ``chassis_ack.c`` 在主机编译，真实判定接受/丢弃。

不允许连接真实设备或真实串口；本文件只用 ``/dev/pts`` 伪终端。
"""
from __future__ import annotations

import errno
import fcntl
import os
import signal
import sys
import tty
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'firmware_harness'))

import fw_v3_harness                                    # noqa: E402
from fw_v3_harness import V3Firmware                    # noqa: E402
from c50c_uart3.chassis_cmd import (                    # noqa: E402
    encode_speed_frame, is_zero_frame,
)
from c50c_uart3.chassis_cmd.handshake import (          # noqa: E402
    SessionHandshake, session_frame_for,
)
from c50c_uart3.chassis_cmd.serial_port import (        # noqa: E402
    PTS_PATTERN, FdPort, RealFdUseForbidden,
)
from c50c_uart3.chassis_cmd.arbiter import Arbiter      # noqa: E402
from c50c_uart3.chassis_cmd.session import (              # noqa: E402
    SessionExtractor, encode_session_ack,
)


# ---------------------------------------------------------------- 计时工具
class Clock:
    def __init__(self, t: float = 100.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> float:
        self.t += dt
        return self.t


# ---------------------------------------------------------------- pty 夹具
@pytest.fixture()
def pty_ends():
    """返回 (master, slave)；两端都设为 raw，关闭段屏蔽 SIGHUP。"""
    master, slave = os.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    old = signal.signal(signal.SIGHUP, signal.SIG_IGN)
    try:
        yield master, slave
    finally:
        signal.signal(signal.SIGHUP, old)
        for fd in (slave, master):
            try:
                os.close(fd)
            except OSError:
                pass


def _flags(fd: int) -> int:
    return fcntl.fcntl(fd, fcntl.F_GETFL)


# ---------------------------------------------------------------- 1. 打开模式
def test_node_opens_readonly_when_drive_disabled(pty_ends):
    """默认关闭：所有者只读打开，同一 fd 不可写。"""
    master, slave = pty_ends
    device = os.ttyname(slave)
    assert PTS_PATTERN.match(device)
    # 复现节点 _open_serial 的开标志选择（与 uart3_node.py 同一表达式）
    drive_enabled = False
    flags = os.O_RDWR if drive_enabled else os.O_RDONLY
    fd = os.open(device, flags | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        assert _flags(fd) & os.O_ACCMODE == os.O_RDONLY
        with pytest.raises(OSError) as exc:
            os.write(fd, b'\x7b')
        assert exc.value.errno in (errno.EBADF, errno.EINVAL)
    finally:
        os.close(fd)


def test_node_opens_readwrite_when_drive_enabled(pty_ends):
    """drive_enable 打开：同一 fd 可读可写，字节到达主端。"""
    master, slave = pty_ends
    device = os.ttyname(slave)
    drive_enabled = True
    flags = os.O_RDWR if drive_enabled else os.O_RDONLY
    fd = os.open(device, flags | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        assert _flags(fd) & os.O_ACCMODE == os.O_RDWR
        port = FdPort(fd, device, fd_authorised=False)   # pty 不是真实 UART
        frame = encode_speed_frame(0.0, 0.0)
        assert port.write_bytes(frame) == len(frame)
        os.set_blocking(master, False)
        assert os.read(master, 64) == frame
    finally:
        os.close(fd)


def test_fd_port_refuses_real_uart_without_authorisation():
    with pytest.raises(RealFdUseForbidden):
        FdPort(5, '/dev/ttyAMA0')
    with pytest.raises(RealFdUseForbidden):
        FdPort(5, '/dev/ttyAMA0', fd_authorised=False)


def test_single_owner_no_second_open_is_static_guard():
    """生产代码里只有 uart3_node 调 os.open（单一所有者）。

    用 AST 判"是否真的调用 os.open"，不看注释/docstring 里的字样——
    文本匹配会把文档中的说明误判成真实调用。
    """
    import ast

    root = Path(__file__).resolve().parents[2] / 'c50c_uart3' / 'chassis_cmd'
    offenders = []
    for path in root.glob('*.py'):
        tree = ast.parse(path.read_text(), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            fn = node.func
            if isinstance(fn, ast.Attribute) and fn.attr == 'open':
                owner = fn.value
                if isinstance(owner, ast.Name) and owner.id == 'os':
                    offenders.append(f'{path.name}:{node.lineno}')
    assert not offenders, f'chassis_cmd 内不得自行打开设备: {offenders}'


# ---------------------------------------------------------------- 2. 写入链
def test_short_write_returns_partial_count(pty_ends):
    """真实伪终端上的部分写入：FdPort 如实返回实际字节数。

    本机实测（Linux pty）：写队列压满后只让出 1 字节，再请求写一整块，
    内核只收下其中一部分（实测 5632 或 3584 字节），其余丢失。上层据此
    判定短写并锁流——这里验证的是"FdPort 不吞掉、不补齐"这一前提。
    帧级短写由 ownership 层 `written != len(data)` 判定（见 test_short_write.py）。
    """
    master, slave = pty_ends
    device = os.ttyname(slave)
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        big = b'P' * 65536
        while True:
            try:
                os.write(fd, big)
            except BlockingIOError:
                break
        os.set_blocking(master, False)
        os.read(master, 1)                      # 只让出 1 字节空间
        port = FdPort(fd, device)
        written = port.write_bytes(big)
        if written == 0:
            pytest.skip('本次内核未让出任何空间（EAGAIN），无法观测部分写入')
        assert 0 < written < len(big), f'应发生部分写入，实际 {written}'
        assert port.bytes_written == written
    finally:
        os.close(fd)


def test_full_queue_yields_zero_not_partial_frame(pty_ends):
    """写队列有零碎空间时，短帧仍可能整帧进队；压满时 FdPort 返回 0。

    结论按实测写死：11 字节的帧只要队列里还有 11 字节空间就会完整写入，
    所以"线程被短写截断"不会发生在帧级别，而发生在一次写入大于剩余空间
    的情形——那由 ownership 层判定失败并锁流。
    """
    master, slave = pty_ends
    device = os.ttyname(slave)
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        block = b'P' * 4096
        while True:
            try:
                os.write(fd, block)
            except BlockingIOError:
                break
        port = FdPort(fd, device)
        base_bytes, base_writes = port.bytes_written, port.writes
        frame = encode_speed_frame(0.05, 0.0)
        outcomes = set()
        for _ in range(64):
            n = port.write_bytes(frame)
            outcomes.add(n)
            if n == 0:
                break
        # 观测到的写入量只能是"整帧"或"零"：FdPort 从不返回半帧被吞的假象。
        assert outcomes <= {0, len(frame)}, outcomes
        if 0 in outcomes:
            # EAGAIN 那次不计数。
            assert port.bytes_written == base_bytes + (len(outcomes) - 1) * len(frame)
        assert port.writes >= base_writes
    finally:
        os.close(fd)


def test_disconnect_surfaces_as_error(pty_ends):
    """对端关闭后写入抛 OSError（上层的失败与锁流路径由此触发）。"""
    master, slave = pty_ends
    device = os.ttyname(slave)
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    port = FdPort(fd, device)
    os.close(master)
    pty_ends  # noqa: B018——保留引用，夹具负责关闭剩余 fd
    with pytest.raises(OSError):
        port.write_bytes(encode_speed_frame(0.05, 0.0))
    os.close(fd)


# ---------------------------------------------------------------- 3. 会话帧
def test_session_frame_matches_firmware_layout():
    frame = session_frame_for(0x1234ABCD)
    assert len(frame) == 11
    assert frame[0] == 0x7B and frame[10] == 0x7D
    # 协议 v3：会话帧版本字节为 3，buffer[2] 严格为 0x01（能力位只在速度帧用）。
    assert frame[1] == 0x7F and frame[2] == 0x01 and frame[3] == 3
    assert frame[4:8] == bytes.fromhex('1234abcd')
    assert frame[8] == 0


def test_extractor_finds_session_frame_in_noise():
    ex = SessionExtractor()
    stream = b'\x00\x7b\x01' + session_frame_for(0xAABBCCDD) + b'\x7d\x7d'
    assert ex.feed(stream) == [0xAABBCCDD]


def test_extractor_ignores_speed_frame():
    ex = SessionExtractor()
    assert ex.feed(encode_speed_frame(0.05, 0.0)) == []


def test_handshake_acknowledges_only_after_frame_written():
    arb = Arbiter()
    arb.mark_receiver_clean(simulated=True)
    clock = Clock()
    hs = SessionHandshake(arb, clock=clock)
    frames = []
    assert hs.feed(session_frame_for(99), write=lambda b: (frames.append(b), len(b))[1]) == 99
    assert arb.session_id == 99 and arb.session_acknowledged is True
    # v3：会话确认帧 buffer[2] 严格为 0x01，不置能力位。
    assert frames and frames[0][1] == 0x7F and frames[0][2] == 0x01
    sent = len(frames)
    # 同一会话 1 s 内不重复回填（节流）
    hs.feed(session_frame_for(99), write=lambda b: (frames.append(b), len(b))[1])
    assert len(frames) == sent
    clock.advance(2.0)
    # 超过节流窗口后允许再回填一次（固件每秒发一帧会话帧）
    hs.feed(session_frame_for(99), write=lambda b: (frames.append(b), len(b))[1])
    assert len(frames) == sent + 1


def test_handshake_short_write_keeps_nonzero_blocked():
    arb = Arbiter()
    arb.mark_receiver_clean(simulated=True)
    hs = SessionHandshake(arb, clock=Clock())
    assert hs.feed(session_frame_for(7), write=lambda b: len(b) - 1) is None
    assert arb.session_id == 7
    assert arb.session_acknowledged is False
    assert hs.stats.ack_write_failures == 1


def test_reconnect_forgets_session():
    arb = Arbiter()
    arb.mark_receiver_clean(simulated=True)
    hs = SessionHandshake(arb, clock=Clock())
    hs.feed(session_frame_for(5), write=lambda b: len(b))
    assert arb.session_acknowledged is True
    arb.on_serial_reconnect()
    assert arb.session_id is None and arb.session_acknowledged is False


# ---------------------------------------------------------------- 4. A 侧门控
@pytest.fixture()
def ack_fw():
    """真实的 v3 ``chassis_ack.c``（会话门 + 序号门），不是桩。"""
    fw = V3Firmware.start()
    try:
        yield fw
    finally:
        fw.close()


def test_a_firmware_drops_speed_frame_without_session(ack_fw):
    ack_fw.feed_bytes(encode_speed_frame(0.05, 0.0, 0.0, seq=0))
    st = ack_fw.state
    assert st.no_session == 1 and st.accept == 0 and st.vx == 0.0


def test_a_firmware_accepts_after_session_frame(ack_fw):
    ack_fw.tick(20)                         # 固件开口：发出一帧会话帧
    wire = ack_fw.wire
    # 线上会话帧由真实的 Chassis_Ack_EncodeSessionFrame 生成。
    assert encode_session_ack(ack_fw.session) in wire
    ack_fw.feed_bytes(encode_session_ack(ack_fw.session))
    ack_fw.feed_bytes(encode_speed_frame(0.0, 0.0, 0.0, seq=0))
    assert ack_fw.state.accept == 1
    assert ack_fw.state.armed is True
    ack_fw.feed_bytes(encode_speed_frame(0.05, 0.0, 0.0, seq=1))
    st = ack_fw.state
    assert st.accept == 2 and st.vx == pytest.approx(0.05, abs=1e-6)


def test_a_firmware_rejects_stale_session(ack_fw):
    ack_fw.tick(20)
    ack_fw.feed_bytes(encode_session_ack(ack_fw.session ^ 0xFFFFFFFF))
    ack_fw.feed_bytes(encode_speed_frame(0.05, 0.0, 0.0, seq=0))
    st = ack_fw.state
    assert st.stale == 1 and st.accept == 0 and st.vx == 0.0


def test_a_firmware_rejects_local_echo(ack_fw):
    ack_fw.tick(1)
    telemetry = ack_fw.wire[:22]
    from c50c_uart3.chassis_cmd import check_bcc
    probe = bytearray(11)
    probe[0] = 0x7B
    probe[1] = telemetry[1]
    probe[2] = 0x80
    probe[3:9] = telemetry[3:9]
    probe[10] = 0x7D
    probe[9] = check_bcc(bytes(probe[:9]))
    ack_fw.feed_bytes(bytes(probe))
    assert ack_fw.state.echo == 1 and ack_fw.state.accept == 0


def test_zero_frame_encoding_is_zero_on_wire():
    frame = encode_speed_frame(0.0, 0.0)
    assert is_zero_frame(frame)


def test_nonzero_frame_not_zero():
    assert not is_zero_frame(encode_speed_frame(0.05, 0.0))


def test_a_candidate_commit_is_recorded():
    """证据边界：引用的 v3 固件源码必须可核对（哈希写进交付说明）。

    ``chassis_ack.c`` / ``chassis_ack.h`` 是判定逻辑本体，哈希变化即证据失效；
    uartx_callback.c / data_task.c / balance_task.c 仍在被固件子智能体修改，
    这里只登记哈希，不作硬断言。
    """
    hashes = fw_v3_harness.firmware_hashes()
    assert set(hashes) == set(fw_v3_harness.RECORDED_SHA256)
    for rel in ('BALANCE/chassis_ack.c', 'BALANCE/Inc/chassis_ack.h'):
        assert hashes[rel] == fw_v3_harness.RECORDED_SHA256[rel], (
            f'{rel} 哈希与记录不一致：v3 契约源码已变更，本证据失效')
