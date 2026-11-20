"""串口生命周期与并发验证（任务书 B 第三轮第 1、2 条）。

本轮测试对象是**生命周期契约**，不是真实串口：

- 所有者（``uart3_node`` 的 ``_run`` / ``destroy_node``）负责 fd 的关闭与重连；
- 写入端只持有端口对象，**从不关闭 fd**；
- 关闭与写入竞争时，写入必须失败并被锁存，而不是"看起来写成功"；
- 重连后旧命令不能恢复，无真实确认时非零门保持关闭。

全部使用内存端口或 ``os.openpty`` 伪终端，不打开真实串口。
"""
from __future__ import annotations

import os
import signal
import threading
import time
import tty

import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame, is_zero_frame
from c50c_uart3.chassis_cmd.arbiter import Arbiter
from c50c_uart3.chassis_cmd.chain import StraightCommandChain
from c50c_uart3.chassis_cmd.ownership import (
    BlockedNonzeroGuard,
    OwnerHarness,
    OwnerSerialTransport,
)
from sim_confirm import mark_endpoint_clean, mark_chain_clean, \
    grant_safety, TEST_SAFETY_MAX_AGE_S, \
    new_receipt as _new_receipt

_SIM = _new_receipt('test_lifecycle')

ZERO = encode_speed_frame(0.0, 0.0, 0.0)

def zero(seq: int = 0) -> bytes:
    """协议 v3 的零速帧：buffer[2] = 0x80 | 序号。"""
    return encode_speed_frame(0.0, 0.0, 0.0, seq=seq)



# --------------------------------------------------------------- 生命周期端口
class ClosingPort:
    """模拟"所有者正在关闭 fd"的端口。

    ``close()`` 之后任何写入都抛 ``OSError(EBADF)``，与真实 fd 关闭后再写
    的行为一致。``reopen()`` 恢复可写并递增世代号，用来区分新旧 fd。
    """

    def __init__(self) -> None:
        self.closed = False
        self.generation = 0
        self.writes: list[bytes] = []
        self.length = 11

    def write_bytes(self, data: bytes) -> int:
        if self.closed:
            raise OSError(9, 'Bad file descriptor')   # EBADF
        if len(data) != self.length:
            raise OSError(5, 'Input/output error')    # 模拟帧错
        self.writes.append(bytes(data))
        return len(data)

    def read_some(self, max_bytes: int) -> bytes:
        return b''

    def reopen(self) -> None:
        self.closed = False
        self.generation += 1

    def close(self) -> None:
        self.closed = True


def fresh(port):
    """返回 (chain, harness)，确认走 harness 记录路径。"""
    arb = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5,
                  safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arb)
    arb.on_publisher_count(1, 0)
    mark_endpoint_clean(port, receipt=_SIM)
    tr = OwnerSerialTransport(port)
    chain = StraightCommandChain(arb, tr)
    mark_chain_clean(chain, receipt=_SIM)  # 第五轮：构造干净接收端
    harness = OwnerHarness(chain)
    chain.firmware_confirm(True)            # 模拟确认
    return chain, harness, tr


# ---------------------------------------------- 1. 所有者为唯一关闭方
def test_writer_never_closes_the_port():
    """写入端不拥有 fd：写入端没有任何关闭路径。"""
    port = ClosingPort()
    mark_endpoint_clean(port, receipt=_SIM)
    tr = OwnerSerialTransport(port)
    chain, _, _ = fresh(port)
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    chain.tick(100.0)
    chain.tick(100.01)
    assert port.closed is False, '命令链不得关闭端口'
    assert tr.write(ZERO) is True


def test_close_then_write_fails_and_latches():
    """所有者关闭 fd 后，写入失败并锁存停车。

    **时序（实测）**：``tick`` 先求决策、再写、写完才回填
    ``on_write_result``。所以失败那一拍仍持有原决策（非零），但那一帧
    **没有写出去**（``write_ok=False``），固件收不到非零；锁存从下一拍
    生效。安全性由"没写出去"保证，不由这一拍的 ``is_zero`` 保证。
    """
    port = ClosingPort()
    chain, _, tr = fresh(port)
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    chain.tick(100.0)
    assert not chain.tick(100.01).is_zero

    writes_before_close = len(port.writes)
    port.close()                            # 所有者关闭
    rec = chain.tick(100.02)
    assert rec.write_ok is False, '关闭后写入必须判失败'
    assert len(port.writes) == writes_before_close, \
        '关闭后端口不得记录任何新写入（失败帧没离开进程）'

    chain.submit_nav(0.05, 0.0, 0.0, 100.03)
    for i in range(5):
        rec = chain.tick(100.03 + i * 0.01)
        assert rec.is_zero, '锁存后只输出零速'
        # 第四轮：关闭即"已发字节不可知"，按流毒处理——连零速帧也不写
        assert rec.zero_reason.name in ('WRITE_FAILED', 'STREAM_POISONED')
        assert rec.frame is None, '流毒后不得再产生任何帧'
    assert tr.stats.write_failures >= 1


def test_close_is_observable_as_ebadf():
    """关闭后的异常是 EBADF，写入端记录下来而不是静默忽略。"""
    port = ClosingPort()
    port.close()
    mark_endpoint_clean(port, receipt=_SIM)
    tr = OwnerSerialTransport(port)
    assert tr.write(ZERO) is False
    assert any('OSError' in d or 'Bad file descriptor' in d
               for d in tr.stats.detail), tr.stats.detail


def test_write_during_concurrent_close_never_silently_succeeds():
    """关闭与写入并发：每一拍的结论必须与端口实际状态一致。

    端口在另一个线程里被关闭；主线程持续写入。断言不出现
    "端口已关闭但写入报告成功"的情况。
    """
    port = ClosingPort()
    chain, _, tr = fresh(port)
    stop = threading.Event()
    closed_flag = {'value': False}

    def closer():
        time.sleep(0.02)
        port.close()
        closed_flag['value'] = True
        stop.set()

    t = threading.Thread(target=closer, daemon=True)
    t.start()
    mismatches = 0
    now = 100.0
    while not stop.is_set() and now < 100.5:
        chain.submit_nav(0.05, 0.0, 0.0, now)
        rec = chain.tick(now)
        if closed_flag['value'] and rec.write_ok:
            mismatches += 1
        now += 0.005
    t.join(timeout=1.0)
    assert mismatches == 0, '关闭后不得报告写入成功'


def test_reopen_generation_is_tracked():
    """重连产生新世代；旧世代写入不能冒充新世代的成功。"""
    port = ClosingPort()
    mark_endpoint_clean(port, receipt=_SIM)
    tr = OwnerSerialTransport(port)
    assert tr.write(ZERO) is True
    gen_before = port.generation
    port.close()
    assert tr.write(ZERO) is False
    tr.reconnect()
    assert port.generation == gen_before + 1, '重连应递增世代'
    # 第四轮改判：关闭后写入的已发字节数不可知，字节流可能被切开，
    # 所以重连**不**解除流锁。恢复输出必须重建端口对象（新适配器）。
    assert tr.stream_locked is True, '关闭/异常写入后必须锁流'
    assert tr.write(ZERO) is False, '流锁下重连不得恢复写入'

    # 第五轮改判：**同一端口**上的新适配器仍看得到残帧，必须继续锁流。
    # 换内存对象不等于接收端重新同步，见 test_stream_epoch.py。
    mark_endpoint_clean(port, receipt=_SIM)
    tr2 = OwnerSerialTransport(port)
    assert tr2.stream_locked is True, '同一端口的新适配器必须继承残帧锁定'
    assert tr2.write(ZERO) is False, '继承锁定后不得写入'

    # 只有换一个**新端口对象**（新接收端，测试构造的干净状态）才恢复可写。
    port_fresh = ClosingPort()
    mark_endpoint_clean(port_fresh, receipt=_SIM)
    tr3 = OwnerSerialTransport(port_fresh)
    assert tr3.stream_locked is False
    assert tr3.write(ZERO) is True, '新端口对象可写'


# ------------------------------ 2. 重连后旧命令不恢复 / 非零门保持关闭
def test_reconnect_clears_pending_nav_and_blocks_nonzero():
    """重连：清空历史命令，20 拍内不出现非零，且旧值不复活。"""
    port = ClosingPort()
    chain, h, _ = fresh(port)
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    chain.tick(100.0)
    assert not chain.tick(100.01).is_zero

    port.close()
    port.reopen()                           # 所有者重新打开
    h.on_reconnected()
    for i in range(20):
        chain.submit_nav(0.0, 0.0, 0.0, 100.02 + i * 0.01)   # 只发零速
        rec = chain.tick(100.02 + i * 0.01)
        assert rec.is_zero, '重连后不得写出非零'
    # 重连后序号从 0 重新开始；只断言帧语义（零速）而不写死序号。
    assert port.writes[-1][1:2] == b'\x00' and port.writes[-1][3:9] == b'\x00' * 6
    assert h.events[-1][0] == 'reconnected'


def test_reconnect_without_confirm_stays_closed():
    """重连后即使有非零命令持续到达，无确认也永久禁止。"""
    port = ClosingPort()
    arb = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5,
                  safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arb)
    arb.on_publisher_count(1, 0)
    arb.on_firmware_confirm(True)
    arb.mark_receiver_clean(simulated=True)  # 第五轮：干净接收端
    mark_endpoint_clean(port, receipt=_SIM)
    tr = OwnerSerialTransport(port)
    chain = StraightCommandChain(arb, tr)
    mark_chain_clean(chain, receipt=_SIM)  # 第五轮：构造干净接收端
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    chain.tick(100.0)
    assert not chain.tick(100.01).is_zero

    port.close()
    port.reopen()
    chain.on_transport_reconnect()
    for i in range(30):
        chain.submit_nav(0.05, 0.0, 0.0, 100.02 + i * 0.01)
        assert chain.tick(100.02 + i * 0.01).is_zero, '无确认不得放行'


def test_second_line_guard_still_counts_after_reconnect():
    """第二道防线在重连后仍然拦截并计数（可观测证据）。"""
    port = ClosingPort()
    mark_endpoint_clean(port, receipt=_SIM)
    inner = OwnerSerialTransport(port)
    guard = BlockedNonzeroGuard(inner, blocked=True)
    guard.reconnect()
    assert guard.write(ZERO) is True
    assert guard.write(encode_speed_frame(0.05, 0.0, 0.0)) is False
    assert guard.stats.rejected_nonzero_while_blocked == 1


# ------------------------------------- 3. 短写与异常在关闭前后的一致行为
@pytest.mark.parametrize('mode', ['short', 'raise', 'zero'])
def test_short_or_exception_paths_are_failures(mode):
    """短写、异常、零写入在任何阶段都判失败。"""
    class Flaky:
        def write_bytes(self, data):
            if mode == 'short':
                return len(data) - 3
            if mode == 'raise':
                raise OSError(5, 'Input/output error')
            return 0

        def read_some(self, n):
            return b''

        def reopen(self):
            pass

    port = Flaky()
    mark_endpoint_clean(port, receipt=_SIM)
    tr = OwnerSerialTransport(port)
    assert tr.write(ZERO) is False
    assert tr.stats.write_failures == 1


# ---------------------------------------- 4. 真实伪终端生命周期（隔离）
def test_pty_lifecycle_close_while_reader_active():
    """隔离伪终端：从端关闭后写入失败，且进程不因 SIGHUP 退出。

    ``os.openpty`` 把从端前台进程组设为调用进程组；关闭从端会向该组
    发 SIGHUP。这里在关闭段忽略 SIGHUP，与 ``test_ownership.py`` 的
    处理一致。
    """
    master, slave = os.openpty()
    slave_path = os.ttyname(slave)
    tty.setraw(slave)
    old_hup = signal.signal(signal.SIGHUP, signal.SIG_IGN)
    try:
        f = open(slave_path, 'r+b', buffering=0)

        class PtyPort:
            def write_bytes(self, data):
                return f.write(data)

            def read_some(self, n):
                return b''

            def reopen(self):
                pass

        _p_port = PtyPort()
        mark_endpoint_clean(_p_port, receipt=_SIM)
        tr = OwnerSerialTransport(_p_port, device=slave_path)
        assert tr.write(ZERO) is True
        f.close()                            # 所有者关闭从端
        assert tr.write(ZERO) is False, '关闭后写入必须失败'
        os.read(master, 64)                  # 排空，不阻塞
    finally:
        for fd in (slave, master):
            try:
                os.close(fd)
            except OSError:
                pass
        signal.signal(signal.SIGHUP, old_hup)


def test_pty_viable_bytes_go_out_only_on_open_port():
    """端口可用时字节确实离开进程；端口关闭后不再有新字节。"""
    master, slave = os.openpty()
    slave_path = os.ttyname(slave)
    tty.setraw(slave)
    old_hup = signal.signal(signal.SIGHUP, signal.SIG_IGN)
    try:
        f = open(slave_path, 'r+b', buffering=0)

        class PtyPort:
            def write_bytes(self, data):
                return f.write(data)

            def read_some(self, n):
                return b''

            def reopen(self):
                pass

        _p_port = PtyPort()
        mark_endpoint_clean(_p_port, receipt=_SIM)
        tr = OwnerSerialTransport(_p_port, device=slave_path)
        assert tr.write(ZERO) is True
        first = os.read(master, 64)
        assert first == ZERO

        f.close()
        assert tr.write(ZERO) is False
        f2 = open(slave_path, 'r+b', buffering=0)   # 重新打开同一从端
        assert tr.write(ZERO) is False, '适配器不缓存 fd，关闭后不复用'
        f2.close()
    finally:
        for fd in (slave, master):
            try:
                os.close(fd)
            except OSError:
                pass
        signal.signal(signal.SIGHUP, old_hup)
