"""第五轮：重建对象不得解除流锁（审核反例复现与修复验证）。

审核反例：内存端口第一次只写出 ``7b 00``，重建 ``Arbiter`` 与
``OwnerSerialTransport`` 后复用同一端口，第一个 ``tick()`` 又写完整零速帧，
字节流从 ``7b00`` 增长为 ``7b000000000000007b7d`` 等。

根因：流锁状态只存在适配器对象里。重建对象 = 换一个没有记忆的适配器，
于是"锁"没了，但**接收端的残帧还在**。

修法：把"接收端状态是否已知安全"挂到**共享接收端**上（端口对象），
而不是适配器上。任何新适配器拿到同一端口，都能看到残帧标记。

测试只用内存对象与隔离伪终端，不打开真实串口。
"""
from __future__ import annotations

import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame
from c50c_uart3.chassis_cmd.arbiter import Arbiter
from c50c_uart3.chassis_cmd.ownership import (
    BlockedNonzeroGuard,
    OwnerSerialTransport,
)
from sim_confirm import mark_endpoint_clean, mark_chain_clean, \
    grant_safety, TEST_SAFETY_MAX_AGE_S, \
    new_receipt as _new_receipt

_SIM = _new_receipt('test_stream_epoch')

ZERO = encode_speed_frame(0.0, 0.0, 0.0)

def zero(seq: int = 0) -> bytes:
    """协议 v3 的零速帧：buffer[2] = 0x80 | 序号。"""
    return encode_speed_frame(0.0, 0.0, 0.0, seq=seq)



# ------------------------------------------------------------------ 共享接收端
class SharedPort:
    """带**接收端状态**的内存端口，模拟"串口另一端仍留着残帧"。

    端口对象代表接收端，重建适配器 ≠ 重建端口。刻意在端口上记录
    ``wire_tainted``：字节流被切开后，接收端不再干净。
    """

    def __init__(self, short_at: int = 1, short_bytes: int = 2) -> None:
        self.stream = bytearray()
        self.writes = 0
        self.short_at = short_at
        self.short_bytes = short_bytes
        self.received_bytes = 0          # "固件侧已收到的字节总数"
        self.wire_tainted = False        # 接收端字节流是否已被切开

    def write_bytes(self, data: bytes) -> int:
        self.writes += 1
        if self.writes == self.short_at:
            n = max(0, min(self.short_bytes, len(data)))
            self.stream.extend(data[:n])
            self.received_bytes += n
            self.wire_tainted = True     # 只发出一部分 → 接收端流被切开
            return n
        self.stream.extend(data)
        self.received_bytes += len(data)
        return len(data)

    def read_some(self, max_bytes: int) -> bytes:
        return b''

    def reopen(self) -> None:
        pass


def make(port, *, confirm=False, clean=True):
    """构造一套全新的适配器 + 仲裁器 + 命令链（模拟对象重建）。

    ``clean=True`` 时同时注入"接收端已证明干净"的**模拟**状态，
    这只是测试构造，不是生产确认。
    """
    from c50c_uart3.chassis_cmd.chain import StraightCommandChain
    arb = Arbiter(safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arb)
    arb.on_publisher_count(1, 0)
    mark_endpoint_clean(port, receipt=_SIM)
    tr = OwnerSerialTransport(port)
    chain = StraightCommandChain(arb, tr)
    if confirm:
        chain.firmware_confirm(True)
    if clean:
        mark_chain_clean(chain, receipt=_SIM)  # 测试构造的干净状态
    return chain, tr, arb


# ============================================================== 反例复现
def test_rebuilt_objects_with_shared_port_do_not_write_again():
    """审核反例：重建适配器与仲裁器、复用同一端口，不得再写任何字节。"""
    port = SharedPort(short_at=1, short_bytes=2)
    chain, tr, _ = make(port, confirm=True)
    mark_chain_clean(chain, receipt=_SIM)   # 反例前置：干净状态
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    chain.tick(100.0)                       # 首帧零速 → 只发出 7b 00
    after_short = bytes(port.stream)
    assert after_short == b'\x7b\x00', f'反例前置：应为 7b00，实际 {after_short.hex()}'

    # 重建整套对象，端口（接收端）不变
    chain2, tr2, arb2 = make(port, confirm=True)
    chain2.submit_nav(0.05, 0.0, 0.0, 200.0)
    chain2.tick(200.0)
    assert bytes(port.stream) == after_short, (
        '重建对象后不得再写字节；新增：'
        f'{bytes(port.stream)[len(after_short):].hex()}')
    assert tr2.stream_locked is True, '新适配器必须继承接收端的残帧状态'
    assert arb2.stream_poisoned is True or chain2.tick(201.0).frame is None


def test_rebuild_after_exception_also_inherits_lock():
    """写入异常（字节数不明）后重建对象同样不得恢复写入。"""
    class RaisingPort(SharedPort):
        def write_bytes(self, data: bytes) -> int:
            self.writes += 1
            if self.writes == 1:
                self.wire_tainted = True
                self.received_bytes += 3
                raise OSError(5, 'Input/output error')
            return super().write_bytes(data)

    port = RaisingPort()
    chain, _, _ = make(port, confirm=True)
    mark_chain_clean(chain, receipt=_SIM)
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    chain.tick(100.0)
    frozen = bytes(port.stream)
    # 异常端口未向内存流追加字节，但接收端已收到部分字节 —— 这正是"不可知"。
    chain2, tr2, _ = make(port, confirm=True)
    for i in range(5):
        chain2.tick(200.0 + i * 0.01)
    assert bytes(port.stream) == frozen
    assert tr2.stream_locked is True


def test_simulated_process_restart_keeps_lock():
    """模拟进程重启：新进程只能拿到同一端口对象，锁随端口一起留下。"""
    port = SharedPort(short_at=1, short_bytes=2)
    chain, _, _ = make(port, confirm=True)
    mark_chain_clean(chain, receipt=_SIM)
    chain.tick(100.0)
    frozen = bytes(port.stream)
    assert frozen, '前置：短写前缀应已在流里'

    # "重启" = 丢弃所有内存对象，只保留端口（接收端）
    del chain
    chain_new, tr_new, _ = make(port)
    chain_new.submit_nav(0.05, 0.0, 0.0, 300.0)
    for i in range(10):
        chain_new.tick(300.0 + i * 0.01)
    assert bytes(port.stream) == frozen, '重启进程不得重置接收端残帧状态'
    assert tr_new.stream_locked is True


def test_close_and_reopen_on_port_keeps_taint():
    """关闭重开：端口对象仍在，残帧标记不因重开消失。"""
    port = SharedPort(short_at=1, short_bytes=5)
    chain, _, _ = make(port, confirm=True)
    mark_chain_clean(chain, receipt=_SIM)
    chain.tick(100.0)
    frozen = bytes(port.stream)
    port.reopen()
    chain2, tr2, _ = make(port, confirm=True)
    chain2.tick(200.0)
    assert bytes(port.stream) == frozen
    assert tr2.stream_locked is True


def test_clear_by_operator_does_not_clear_taint():
    """人工清错只清仲裁器写失败锁存，不清端口残帧标记。"""
    port = SharedPort(short_at=1, short_bytes=2)
    chain, tr, arb = make(port, confirm=True)
    mark_chain_clean(chain, receipt=_SIM)
    chain.tick(100.0)
    frozen = bytes(port.stream)
    arb.clear_write_fault(by_operator=True)
    assert arb.stream_poisoned is True, '清错不得清除流毒'
    chain2, tr2, _ = make(port, confirm=True)
    chain2.tick(200.0)
    assert bytes(port.stream) == frozen


def test_repeated_rebuilds_are_idempotent():
    """反复重建对象不累积出任何一次可写机会。"""
    port = SharedPort(short_at=1, short_bytes=2)
    chain, _, _ = make(port, confirm=True)
    mark_chain_clean(chain, receipt=_SIM)
    chain.tick(100.0)
    frozen = bytes(port.stream)
    assert frozen
    for i in range(6):
        c, _, _ = make(port, confirm=True)
        c.submit_nav(0.05, 0.0, 0.0, 200.0 + i)
        c.tick(200.0 + i)
    assert bytes(port.stream) == frozen


# ================================================== 启动静默（安全前提）
def test_startup_is_silent_until_clean_state_confirmed():
    """启动/恢复时无法证明接收端安全 → 静默且禁止非零。

    旧行为：启动第一拍就写零速帧（"首帧零速"规则）。
    第五轮改判：首帧零速只适用于**已满足安全发送前提**的情形；
    启动时接收端状态未知，故先静默。
    """
    port = SharedPort()
    chain, tr, _ = make(port, confirm=True, clean=False)   # 即便给了模拟确认
    for i in range(5):
        chain.submit_nav(0.05, 0.0, 0.0, 100.0 + i * 0.01)
        rec = chain.tick(100.0 + i * 0.01)
        assert rec.frame is None, '启动时接收端状态未知，必须静默'
    assert len(port.stream) == 0, '启动静默期间不得写出任何字节'


def test_first_frame_zero_still_applies_after_clean_state():
    """构造明确干净的模拟状态后，"首帧零速"规则仍然生效。"""
    port = SharedPort()
    port.wire_tainted = False                  # 测试构造的干净状态
    chain, tr, _ = make(port, confirm=True)
    mark_chain_clean(chain, receipt=_SIM)
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    rec = chain.tick(100.0)
    assert rec.frame is not None and rec.is_zero, '干净状态后首帧应为零速'
    assert rec.zero_reason.name == 'BRIDGE_REARMING'


def test_clean_state_injection_requires_simulated_flag():
    """第八轮：生产模块上不再有公开注入入口；构造必须走测试设施凭证。"""
    chain, _, _ = make(SharedPort())
    assert not hasattr(chain, 'mark_clean_state_verified')
    # 测试设施的构造入口拒绝裸布尔/无凭证调用
    with pytest.raises(TypeError):
        mark_chain_clean(chain, receipt=True)
    with pytest.raises(TypeError):
        mark_chain_clean(chain)
    with pytest.raises(TypeError):
        mark_endpoint_clean(SharedPort(), receipt='simulated')
    # 私有入口仍保留 simulated 校验（防止内部误用）
    with pytest.raises(TypeError):
        chain._mark_clean_state_verified()
    with pytest.raises(ValueError):
        chain._mark_clean_state_verified(simulated=False)


# ==================================================== 组合与适配器兼容
class PlainTransport:
    """没有 ``stream_locked`` 属性的旧式适配器。"""

    def __init__(self) -> None:
        self.stream = bytearray()
        self.writes = 0

    def write(self, data: bytes) -> bool:
        self.writes += 1
        if self.writes == 1:
            self.stream.extend(data[:2])
            return False                       # 短写
        self.stream.extend(data)
        return True


def test_transport_without_stream_locked_locks_on_failure():
    """没有 stream_locked 属性的适配器短写时，也必须锁定。"""
    from c50c_uart3.chassis_cmd.chain import StraightCommandChain
    arb = Arbiter(safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arb)
    arb.on_publisher_count(1, 0)
    tr = PlainTransport()
    chain = StraightCommandChain(arb, tr)
    chain.firmware_confirm(True)
    mark_chain_clean(chain, receipt=_SIM)
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    chain.tick(100.0)                          # 首帧零速短写
    frozen = bytes(tr.stream)
    for i in range(5):
        chain.tick(100.01 + i * 0.01)
    assert bytes(tr.stream) == frozen, '无 stream_locked 时必须保守锁定'
    # 没有 stream_locked 属性的适配器：chain 按保守默认处理，写失败即判流毒。
    assert arb.stream_poisoned is True or arb.receiver_clean is False


def test_guard_passes_lock_to_arbiter_and_blocks_underlying():
    """BlockedNonzeroGuard 包住 OwnerSerialTransport 时，锁能传到仲裁器。"""
    from c50c_uart3.chassis_cmd.chain import StraightCommandChain
    port = SharedPort(short_at=1, short_bytes=2)
    mark_endpoint_clean(port, receipt=_SIM)
    tr = OwnerSerialTransport(port)
    guard = BlockedNonzeroGuard(tr, blocked=False)
    arb = Arbiter(safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arb)
    arb.on_publisher_count(1, 0)
    chain = StraightCommandChain(arb, guard)
    chain.firmware_confirm(True)
    mark_chain_clean(chain, receipt=_SIM)
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    chain.tick(100.0)                          # 短写
    frozen = bytes(port.stream)
    assert guard.stream_locked is True, 'guard 必须透出内层流锁'
    for i in range(5):
        chain.tick(100.01 + i * 0.01)
    assert bytes(port.stream) == frozen, '锁定期间底层不得再收到字节'
    assert arb.stream_poisoned is True
    assert any('locked' in d or 'partial' in d or 'OSError' in d
               for d in tr.stats.detail), tr.stats.detail


def test_guard_reports_consistent_state():
    """诊断状态与实际一致：guard.stream_locked == 内层 == 仲裁器。"""
    port = SharedPort(short_at=1, short_bytes=2)
    mark_endpoint_clean(port, receipt=_SIM)
    tr = OwnerSerialTransport(port)
    guard = BlockedNonzeroGuard(tr, blocked=False)
    assert guard.stream_locked is False
    tr.write(ZERO)                             # 第 1 次即短写
    assert guard.stream_locked is True
    assert tr.stream_locked is True
    assert guard.stream_lock_reason is not None
