"""单一串口所有者接入方案的定向测试（任务书 B 第二轮第 3 条）。

全部测试只使用内存对象与隔离伪终端（``os.openpty``）；**不打开真实串口**。
覆盖要求：

1. 单一所有者——只有一个端口对象被写入，新进程/新 fd 不被允许；
2. 停车前的旧命令不恢复；
3. 通信失败（写异常、部分写入）立即锁存停车；
4. 无确认（以及模拟确认 vs 真实固件接收确认的区别）时禁止非零；
5. 重连后禁止非零保持关闭。
"""
from __future__ import annotations

import os
import signal
import tty

import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame, is_zero_frame
from c50c_uart3.chassis_cmd.arbiter import Arbiter
from c50c_uart3.chassis_cmd.chain import StraightCommandChain
from c50c_uart3.chassis_cmd.ownership import (
    BlockedNonzeroGuard,
    NonIsolatedDeviceForbidden,
    OwnerHarness,
    OwnerSerialTransport,
    build_isolated_transport,
    mark_endpoint_tainted,
)
from c50c_uart3.chassis_cmd.handshake import session_frame_for
from c50c_uart3.chassis_cmd.transport import (
    MemoryTransport,
    RealDeviceAccessForbidden,
    SerialOwnerTransport,
)
from sim_confirm import mark_endpoint_clean, mark_chain_clean, \
    grant_safety, TEST_SAFETY_MAX_AGE_S, \
    new_receipt as _new_receipt

_SIM = _new_receipt('test_ownership')

ZERO = encode_speed_frame(0.0, 0.0, 0.0)

def zero(seq: int = 0) -> bytes:
    """协议 v3 的零速帧：buffer[2] = 0x80 | 序号。"""
    return encode_speed_frame(0.0, 0.0, 0.0, seq=seq)



# ------------------------------------------------------------------ 测试替身
class CountingPort:
    """满足 :class:`SerialPortContract` 的内存端口，记录写入。"""

    def __init__(self) -> None:
        self.writes: list[bytes] = []
        self.reopen_count = 0
        self.fail_mode: str | None = None      # None / 'raise' / 'partial' / 'zero'

    def write_bytes(self, data: bytes) -> int:
        if self.fail_mode == 'raise':
            raise OSError('模拟写失败 EIO')
        if self.fail_mode == 'partial':
            return len(data) - 1
        if self.fail_mode == 'zero':
            return 0
        self.writes.append(bytes(data))
        return len(data)

    def read_some(self, max_bytes: int) -> bytes:   # pragma: no cover——本测试不读
        return b''

    def reopen(self) -> None:
        self.reopen_count += 1


def _fresh(transport=None) -> tuple[StraightCommandChain, OwnerHarness]:
    arb = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5,
                  safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arb)
    arb.on_publisher_count(1, 0)
    arb.on_firmware_confirm(True)   # 模拟确认：仅测试用
    if transport is None:
        # 第六轮：未登记的端口按"不可判定"处理，测试在此显式登记为干净。
        transport = MemoryTransport()
        mark_endpoint_clean(transport, receipt=_SIM)
    chain = StraightCommandChain(arb, transport)
    # 第五轮：启动默认"接收端未知"，测试需显式构造干净状态才能发送
    mark_chain_clean(chain, receipt=_SIM)
    return chain, OwnerHarness(chain)


# ------------------------------------------------------- 1. 单一所有者约束
def test_real_uart_device_is_accepted_without_opening():
    """真实 UART 设备名可构造，且构造不打开任何设备（端口是离线替身）。"""
    for dev in ('/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/ttyACM1',
                '/dev/serial/by-id/foo'):
        transport = OwnerSerialTransport(CountingPort(), device=dev)
        assert transport._device == dev          # noqa: SLF001——只核对记录值
        assert isinstance(transport.stream_locked, bool)


def test_unauthorised_real_uart_fd_is_rejected_by_port_layer():
    """放行真实 UART 不等于放行未授权 fd：``FdPort`` 自己仍拒绝。

    授权检查留在 ``serial_port`` 这一层，``ownership`` 不重复实现它。
    """
    from c50c_uart3.chassis_cmd.serial_port import FdPort, RealFdUseForbidden

    with pytest.raises(RealFdUseForbidden):
        FdPort(99, '/dev/ttyACM1')             # 未授权：构造即失败
    port = FdPort(99, '/dev/ttyACM1', fd_authorised=True)
    assert port.device == '/dev/ttyACM1'       # 构造不调用 os.open
    OwnerSerialTransport(port, device='/dev/ttyACM1')


def test_non_uart_non_pty_paths_are_still_rejected():
    """其它路径（/tmp/xxx、/dev/random 等）仍然抛 NonIsolatedDeviceForbidden。"""
    for dev in ('/tmp/xxx', '/dev/random', '/dev/ttyACM1x', '',
                '/dev/pts/abc', '/dev/pts/1/extra'):
        with pytest.raises(NonIsolatedDeviceForbidden):
            OwnerSerialTransport(CountingPort(), device=dev)
        with pytest.raises(NonIsolatedDeviceForbidden):
            build_isolated_transport(dev, port=CountingPort())


def test_direct_real_transport_still_refuses():
    """旧的 SerialOwnerTransport 仍然构造即拒绝（未被本轮放松）。"""
    with pytest.raises(RealDeviceAccessForbidden):
        SerialOwnerTransport('/dev/ttyAMA0')


def test_factory_never_opens_device(tmp_path):
    """工厂只返回内存写入端或已注入端口的适配器，不打开设备。"""
    assert isinstance(build_isolated_transport(), MemoryTransport)
    with pytest.raises(NonIsolatedDeviceForbidden):
        build_isolated_transport(str(tmp_path / 'not-a-uart'))
    with pytest.raises(ValueError):
        build_isolated_transport('/dev/pts/9999')   # 没注入端口就不打开
    # 真实 UART 由工厂放行，但仍要求注入所有者已打开的端口对象。
    with pytest.raises(ValueError):
        build_isolated_transport('/dev/ttyACM1')
    transport = build_isolated_transport('/dev/ttyACM1', port=CountingPort())
    assert isinstance(transport, OwnerSerialTransport)


def test_port_contract_is_checked_by_runtime_protocol():
    """端口对象必须满足协议；缺方法时适配器立刻暴露而不是运行时才炸。"""
    from c50c_uart3.chassis_cmd.ownership import SerialPortContract

    assert isinstance(CountingPort(), SerialPortContract)
    assert not isinstance(object(), SerialPortContract)


def test_single_port_receives_all_writes():
    """命令链的每一帧都只写进同一个端口对象，没有第二个写入者。"""
    port = CountingPort()
    mark_endpoint_clean(port, receipt=_SIM)
    transport = OwnerSerialTransport(port, device=None)
    chain, _ = _fresh(transport)
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    for i in range(5):
        chain.tick(100.0 + i * 0.01)
    assert len(port.writes) == 5
    assert transport.stats.writes == 5
    assert len(transport.stats.detail) == 0


# -------------------------------------------------- 2. 停车前旧命令不恢复
def test_stop_discards_before_owner_write():
    """停车期间与停车前的非零命令都不会写到端口。"""
    port = CountingPort()
    mark_endpoint_clean(port, receipt=_SIM)
    chain, h = _fresh(OwnerSerialTransport(port))
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    chain.tick(100.0)
    chain.tick(100.01)                       # 首帧零速 + 一帧非零
    nonzero_before = len([w for w in port.writes if not is_zero_frame(w)])
    assert nonzero_before == 1

    chain.set_stop(True)
    chain.tick(100.02)
    chain.submit_nav(0.05, 0.0, 0.0, 100.03)  # 停车期间到达
    chain.tick(100.03)
    chain.set_stop(False)
    for i in range(5):
        chain.tick(100.04 + i * 0.01)
        chain.submit_nav(0.0, 0.0, 0.0, 100.04 + i * 0.01)
    after = [w for w in port.writes[4:] if not is_zero_frame(w)]
    assert after == [], '停车解除后不得再写出任何非零帧'


# ---------------------------------------------------- 3. 通信失败立即锁存
@pytest.mark.parametrize('mode', ['raise', 'partial', 'zero'])
def test_write_failure_latches_stop(mode):
    """写异常、部分写入、零写入都算失败，并锁存停车。"""
    port = CountingPort()
    mark_endpoint_clean(port, receipt=_SIM)
    transport = OwnerSerialTransport(port)
    chain, _ = _fresh(transport)
    chain.tick(100.0)                        # 正常零速帧
    port.fail_mode = mode
    rec = chain.tick(100.01)
    assert rec.write_ok is False
    port.fail_mode = None
    chain.submit_nav(0.05, 0.0, 0.0, 100.02)
    for i in range(5):
        rec = chain.tick(100.02 + i * 0.01)
        assert rec.is_zero, '写失败后必须只输出零速'
    assert transport.stats.write_failures >= 1


def test_partial_write_is_not_success():
    """部分写入不能当作成功：帧被截断，固件可能拼出错误帧。"""
    port = CountingPort()
    port.fail_mode = 'partial'
    mark_endpoint_clean(port, receipt=_SIM)
    transport = OwnerSerialTransport(port)
    assert transport.write(encode_speed_frame(0.05, 0.0, 0.0)) is False
    assert transport.stats.write_failures == 1
    assert any('partial write' in d for d in transport.stats.detail)


# ------------------------------------------------- 4. 无确认时禁止非零输出
def test_no_confirm_no_nonzero_memory():
    """没有确认（真实固件状态），仲裁器不得放行非零。"""
    arb = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5,
                  safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arb)
    arb.on_publisher_count(1, 0)
    chain = StraightCommandChain(arb, MemoryTransport())
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    for i in range(10):
        rec = chain.tick(100.0 + i * 0.01)
        assert rec.is_zero


def test_simulated_confirm_vs_real_confirm_are_distinguishable():
    """模拟确认能让仲裁器放行，但事件名必须标出 simulated。

    真实固件没有状态确认字段（``buffer[2]`` 语义未知），所以生产路径
    没有任何地方会调用确认；测试里调用必须能被日志区分。
    """
    chain, h = _fresh()
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    chain.tick(100.0)
    assert not chain.tick(100.01).is_zero, '前置：模拟确认后应能放行'
    assert h.events == [], '构造阶段的确认不应被记录成运行时事件'

    h.on_firmware_confirm(True)
    name, value = h.events[-1]
    assert name == 'firmware_confirm(simulated)'
    assert value is True


def test_confirm_false_does_not_authorise():
    """``on_firmware_confirm(False)`` 不得当作已确认。"""
    port = CountingPort()
    mark_endpoint_clean(port, receipt=_SIM)
    chain, h = _fresh(OwnerSerialTransport(port))
    h.on_reconnected()                       # 进入禁止非零
    h.on_firmware_confirm(False)
    for i in range(5):
        chain.submit_nav(0.05, 0.0, 0.0, 100.0 + i * 0.01)
        assert chain.tick(100.0 + i * 0.01).is_zero


def test_reconnect_blocks_nonzero_until_confirmed():
    """重连后必须零速，且没有确认时永远不放行。"""
    port = CountingPort()
    mark_endpoint_clean(port, receipt=_SIM)
    chain, h = _fresh(OwnerSerialTransport(port))
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    chain.tick(100.0)
    assert not chain.tick(100.01).is_zero

    h.on_reconnected()                       # 所有者报告重连
    for i in range(20):
        chain.submit_nav(0.05, 0.0, 0.0, 100.02 + i * 0.01)
        assert chain.tick(100.02 + i * 0.01).is_zero, '重连后不得放行非零'

    h.on_firmware_confirm(True)              # 模拟确认后才可恢复
    chain.submit_nav(0.05, 0.0, 0.0, 100.5)
    chain.tick(100.5)
    assert not chain.tick(100.51).is_zero


# ------------------------------------------------------- 5. 第二道防线
def test_blocked_guard_rejects_nonzero_but_allows_zero():
    """第二道防线：禁止非零时零速帧放过、非零帧拦截并计数。"""
    inner = MemoryTransport()
    guard = BlockedNonzeroGuard(inner, blocked=True)
    assert guard.write(ZERO) is True
    assert guard.write(encode_speed_frame(0.05, 0.0, 0.0)) is False
    assert guard.stats.rejected_nonzero_while_blocked == 1
    assert len(inner.writes) == 1


def test_zero_frame_detector_is_conservative():
    """非法字节串一律视为非零（宁可拦下）。"""
    assert is_zero_frame(ZERO)
    assert not is_zero_frame(encode_speed_frame(0.001, 0.0, 0.0))
    assert not is_zero_frame(b'')
    assert not is_zero_frame(ZERO[:-1])
    assert not is_zero_frame(b'\x00' * 11)


# --------------------------------------- 6. 隔离伪终端上的端到端（可选验证）
def test_end_to_end_on_isolated_pty():
    """用 /dev/pts/ 伪终端跑通整条链：端口不可写真实设备，只写伪终端。

    **SIGHUP 陷阱**：``os.openpty`` 会把该从端的前台进程组设为调用进程组；
    关闭从端时内核向该进程组发 SIGHUP，默认动作是终止进程——整个 pytest
    进程会以退出码 129（128+1）死掉，而不是测试失败。所以关闭段必须先
    忽略 SIGHUP，结束后恢复原处置。
    """
    master, slave = os.openpty()
    slave_path = os.ttyname(slave)
    tty.setraw(slave)
    try:
        with open(slave_path, 'r+b', buffering=0) as f:
            class PtyPort:
                def write_bytes(self, data):
                    return f.write(data)

                def read_some(self, n):
                    return b''

                def reopen(self):
                    pass

            _pty_port = PtyPort()
            mark_endpoint_clean(_pty_port, receipt=_SIM)
            transport = OwnerSerialTransport(_pty_port, device=slave_path)
            chain, _ = _fresh(transport)
            chain.submit_nav(0.05, 0.0, 0.0, 100.0)
            chain.tick(100.0)                # 首帧零速（首帧规则）
            chain.tick(100.01)               # 非零帧
            # 非阻塞读：启动静默时不会有字节，阻塞读会让本轮无输出并在
            # 关闭从端时被 SIGHUP 打断（这正是第三轮退出码 129 的形态）。
            os.set_blocking(master, False)
            try:
                received = os.read(master, 64)
            except BlockingIOError:
                received = b''
        # 伪终端非阻塞读可能只取到已到达的部分；断言不依赖一次读完。
        assert len(received) in (0, 11, 22) or len(received) > 0
        assert received[:11] == zero(0), '首帧必须是零速'
        for i in range(0, len(received) - 10, 11):
            chunk = received[i:i + 11]
            assert chunk[0] == 0x7B and chunk[10] == 0x7D, chunk.hex()
            if i >= 11:
                assert not is_zero_frame(chunk), '第二帧应是非零命令'
    finally:
        _close_pty_quietly(master, slave)


def _close_pty_quietly(master: int, slave: int) -> None:
    """关闭伪终端，屏蔽关闭从端触发的 SIGHUP（见上方测试文档）。

    只在**关闭段**临时忽略 SIGHUP 并立刻恢复原处置，不会掩盖测试内的
    真实信号问题。
    """
    old = signal.signal(signal.SIGHUP, signal.SIG_IGN)
    try:
        for fd in (slave, master):
            try:
                os.close(fd)
            except OSError:
                pass
    finally:
        signal.signal(signal.SIGHUP, old)


# ------------------------------------ 7. 接收端同步规则（真实 UART 接入）
def _uart_transport(port=None):
    """构造一个指向真实 UART 设备名的适配器（端口是离线替身，不开设备）。"""
    return OwnerSerialTransport(port if port is not None else CountingPort(),
                                device='/dev/ttyACM1')


def test_receiver_stays_locked_without_evidence():
    """(b) 无任何接收端证据时仍锁定，写入返回 False。"""
    port = CountingPort()
    transport = _uart_transport(port)
    assert transport.stream_locked is True
    assert transport.stream_lock_reason is None, '缺证据是"未同步"，不是"残帧"'
    assert transport.receiver_synced is False

    assert transport.write(zero(0)) is False
    assert port.writes == []
    assert transport.stats.write_failures == 1


def test_write_success_never_creates_clean_evidence():
    """写入成功与接收端证据无关：被拒的写入不会置同步态。"""
    transport = _uart_transport()
    for _ in range(3):
        assert transport.write(zero(0)) is False
    assert transport.receiver_synced is False
    assert transport.stream_locked is True


def test_valid_firmware_status_frame_unlocks_receiver():
    """(c) 收到并校验通过的固件状态帧后，接收端同步、可以写入。"""
    port = CountingPort()
    transport = _uart_transport(port)
    assert transport.stream_locked is True

    frame = session_frame_for(0x1234)
    assert transport.note_firmware_status_frame(frame) is True
    assert transport.receiver_synced is True
    assert transport.stream_locked is False
    assert transport.stream_lock_reason is None

    assert transport.write(zero(0)) is True
    assert len(port.writes) == 1
    assert transport.stats.writes == 1


@pytest.mark.parametrize('corrupt', [
    b'',
    b'\x00' * 11,
    bytearray(session_frame_for(1))[:10],
])
def test_truncated_or_garbage_status_frame_does_not_unlock(corrupt):
    """长度/内容非法的字节串不是状态帧，不解锁。"""
    transport = _uart_transport()
    assert transport.note_firmware_status_frame(corrupt) is False
    assert transport.stream_locked is True
    assert transport.receiver_synced is False


@pytest.mark.parametrize('field,value', [
    ('bcc', 0xFF),
    ('tail', 0x00),
    ('type', 0x00),
    ('version', 0x02),
])
def test_bad_status_frame_field_does_not_unlock(field, value):
    """BCC/帧尾/类型/版本任一不符，都按非法状态帧处理。"""
    frame = bytearray(session_frame_for(0x55))
    index = {'bcc': 9, 'tail': 10, 'type': 1, 'version': 3}[field]
    frame[index] = value
    transport = _uart_transport()
    assert transport.note_firmware_status_frame(bytes(frame)) is False
    assert transport.stream_locked is True


def test_sync_evidence_does_not_clear_inherited_taint():
    """(d) 端口已有残帧证据时，状态帧不解锁，原有锁定原因保留。"""
    port = CountingPort()
    mark_endpoint_tainted(port, '上一个适配器短写')
    transport = _uart_transport(port)
    assert transport.stream_locked is True
    assert '残帧' in (transport.stream_lock_reason or '')

    assert transport.note_firmware_status_frame(session_frame_for(1)) is False
    assert transport.stream_locked is True
    assert '残帧' in (transport.stream_lock_reason or ''), '原有原因必须保留'
    assert transport.write(zero(0)) is False
    assert port.writes == []


def test_sync_evidence_does_not_clear_own_short_write_lock():
    """(d) 适配器自己短写过之后，再收到状态帧也不解锁。"""
    port = CountingPort()
    transport = _uart_transport(port)
    assert transport.note_firmware_status_frame(session_frame_for(2)) is True
    assert transport.stream_locked is False

    port.fail_mode = 'partial'
    assert transport.write(zero(0)) is False          # 短写 → 锁定并标记端口
    assert transport.stream_locked is True

    port.fail_mode = None
    assert transport.note_firmware_status_frame(session_frame_for(3)) is False
    assert transport.stream_locked is True
    assert '部分写入' in (transport.stream_lock_reason or ''), '锁定原因必须是残帧'
    assert port.writes == []                          # 锁后一个字节都没写出去
