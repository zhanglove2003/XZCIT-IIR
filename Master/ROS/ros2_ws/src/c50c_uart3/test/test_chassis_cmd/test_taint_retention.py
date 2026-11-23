"""第六轮：标记无法保存时的流锁缺口。

审核反例：端口用 ``__slots__`` 拒绝任意属性。第五轮把残帧标记 ``setattr``
到端口上，对这类端口抛出 ``AttributeError`` 并被吞掉，结果重建适配器即丢锁。

本文件先把审核反例写成失败测试，再验证修复。

隔离模型边界：这里所有"接收端"都是内存对象，共享的是 Python 对象本身。
真实进程重启、真实 fd 重开**没有**在目标机上执行，本文件不代表真实串口行为。
"""
from __future__ import annotations

import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame
from c50c_uart3.chassis_cmd.arbiter import Arbiter
from c50c_uart3.chassis_cmd.chain import StraightCommandChain
from c50c_uart3.chassis_cmd.ownership import (
    OwnerSerialTransport,
    endpoint_is_tainted,
    mark_endpoint_tainted,
)
from sim_confirm import mark_endpoint_clean, mark_chain_clean, \
    new_receipt as _new_receipt

_SIM = _new_receipt('test_taint_retention')

ZERO = encode_speed_frame(0.0, 0.0, 0.0)

def zero(seq: int = 0) -> bytes:
    """协议 v3 的零速帧：buffer[2] = 0x80 | 序号。"""
    return encode_speed_frame(0.0, 0.0, 0.0, seq=seq)



# ------------------------------------------------------------- 不可写属性端口
class SlottedPort:
    """用 ``__slots__`` 禁止任意属性——模拟审核反例里的端口。"""

    __slots__ = ('stream', 'writes', 'short_at', 'short_bytes', 'generation')

    def __init__(self, short_at: int = 1, short_bytes: int = 2) -> None:
        self.stream = bytearray()
        self.writes = 0
        self.short_at = short_at
        self.short_bytes = short_bytes
        self.generation = 0

    def write_bytes(self, data: bytes) -> int:
        self.writes += 1
        if self.writes == self.short_at:
            n = max(0, min(self.short_bytes, len(data)))
            self.stream.extend(data[:n])
            return n
        self.stream.extend(data)
        return len(data)

    def read_some(self, max_bytes: int) -> bytes:
        return b''

    def reopen(self) -> None:
        self.generation += 1


class RaisingSlottedPort(SlottedPort):
    __slots__ = ()

    def write_bytes(self, data: bytes) -> int:
        self.writes += 1
        if self.writes == 1:
            self.stream.extend(data[:2])
            raise OSError(5, 'Input/output error')
        return super().write_bytes(data)


def make(port, *, clean=True):
    """构造一整套新对象。

    ``clean=True`` 表示"测试显式声明这个端口状态已知干净"。第六轮改判后，
    端口默认是**不可判定**（按锁定处理），所以必须用
    :func:`c50c_uart3.chassis_cmd.ownership.mark_endpoint_clean_for_test`
    给出证据，否则一个字节也不会写出。
    """
    if clean:
        mark_endpoint_clean(port, receipt=_SIM)
    arb = Arbiter()
    arb.on_publisher_count(1, 0)
    arb.on_firmware_confirm(True)
    tr = OwnerSerialTransport(port)
    chain = StraightCommandChain(arb, tr)
    # clean=True 表示测试显式声明该端口干净；否则只做固件确认，
    # 端口仍不可判定（保持静默），用于验证"注入压不过不可判定"。
    mark_chain_clean(chain, receipt=_SIM, mark_port=clean)
    return chain, tr, arb


# =========================================================== 反例 1：短写
def test_slotted_port_short_write_cannot_be_rearmed_by_rebuild():
    """审核反例：slotted 端口短写后重建适配器，必须仍锁定且不再写字节。"""
    port = SlottedPort(short_at=1, short_bytes=2)
    chain, tr, _ = make(port)
    chain.tick(100.0)
    assert bytes(port.stream) == b'\x7b\x00'
    frozen = bytes(port.stream)

    chain2, tr2, arb2 = make(port, clean=False)
    chain2.tick(200.0)
    assert bytes(port.stream) == frozen, (
        '标记无法保存时，重建对象也不得再写；新增：'
        f'{bytes(port.stream)[len(frozen):].hex()}')
    assert tr2.stream_locked is True, '无法证明干净时必须拒绝使用该端口'
    assert tr2.write(ZERO) is False


def test_slotted_port_exception_cannot_be_rearmed_by_rebuild():
    """异常路径同样：重建对象不得恢复写入。"""
    port = RaisingSlottedPort()
    chain, _, _ = make(port)
    chain.tick(100.0)
    frozen = bytes(port.stream)
    chain2, tr2, _ = make(port, clean=False)
    for i in range(5):
        chain2.tick(200.0 + i * 0.01)
    assert bytes(port.stream) == frozen
    assert tr2.stream_locked is True


def test_slotted_port_rebuild_is_idempotent():
    """重复重建不累积出任何一次可写机会。"""
    port = SlottedPort(short_at=1, short_bytes=5)
    chain, _, _ = make(port)
    chain.tick(100.0)
    frozen = bytes(port.stream)
    for i in range(5):
        c, _, _ = make(port, clean=True)   # 即便再次注入"干净"也不得放行
        c.tick(200.0 + i)
    assert bytes(port.stream) == frozen


# ================================================== 两个包装对象 / 同一接收端
class SharedWire:
    """模拟接收端：两个不同 Python 端口对象指向同一份字节流。"""

    def __init__(self) -> None:
        self.bytes = bytearray()
        self.writes = 0
        self.short_at = 1
        self.short_bytes = 2
        self.generation = 0


class Wrapper:
    """指向共享接收端的一个 Python 包装对象（可写属性）。"""

    def __init__(self, wire: SharedWire) -> None:
        self.wire = wire

    def write_bytes(self, data: bytes) -> int:
        w = self.wire
        w.writes += 1
        if w.writes == w.short_at:
            n = min(w.short_bytes, len(data))
            w.bytes.extend(data[:n])
            return n
        w.bytes.extend(data)
        return len(data)

    def read_some(self, max_bytes: int) -> bytes:
        return b''

    def reopen(self) -> None:
        self.wire.generation += 1

    @property
    def stream_bytes(self) -> bytes:
        return bytes(self.wire.bytes)


def test_two_wrapper_objects_on_same_receiver_do_not_rearm():
    """反例 2：两个不同包装对象指向同一接收端，第二个不得写字节。"""
    wire = SharedWire()
    w1 = Wrapper(wire)
    chain, _, _ = make(w1)
    chain.tick(100.0)
    frozen = bytes(wire.bytes)
    assert frozen == b'\x7b\x00'

    w2 = Wrapper(wire)                      # 新包装对象，同一接收端
    # 关键：第二个包装对象不能再拿到"干净"声明——它没有证据。
    chain2, tr2, _ = make(w2, clean=False)
    chain2.tick(200.0)
    assert bytes(wire.bytes) == frozen, (
        f'新包装对象不得追加字节：{bytes(wire.bytes)[len(frozen):].hex()}')


def test_process_loss_with_receiver_residue_stays_silent():
    """进程状态完全丢失、接收端残帧仍在：默认静默。"""
    wire = SharedWire()
    w1 = Wrapper(wire)
    chain, _, _ = make(w1)
    chain.tick(100.0)
    frozen = bytes(wire.bytes)
    del chain, w1                            # 丢弃全部 Python 状态
    w2 = Wrapper(wire)                       # 新进程只能拿到同一接收端
    chain2, tr2, _ = make(w2, clean=False)   # 默认（无注入）
    for i in range(10):
        chain2.tick(300.0 + i * 0.01)
    assert bytes(wire.bytes) == frozen, '进程状态丢失时不得追加字节'
    assert tr2.stream_locked is True


def test_close_and_reopen_same_wrapper_keeps_lock():
    """关闭重开（同一包装对象）：重开不等于接收端已同步。"""
    wire = SharedWire()
    w = Wrapper(wire)
    chain, tr, _ = make(w)
    chain.tick(100.0)
    frozen = bytes(wire.bytes)
    w.reopen()
    chain2, tr2, _ = make(w, clean=True)
    chain2.tick(200.0)
    assert bytes(wire.bytes) == frozen
    assert tr2.stream_locked is True


# ================================================== 标记接口本身
def test_mark_endpoint_tainted_falls_back_on_slotted_port():
    """属性写不进去时退回对象登记，仍判为脏；未登记的端口判不可判定。"""
    port = SlottedPort()
    assert endpoint_is_tainted(port) is None, '未登记 = 不可判定，不是干净'
    mark_endpoint_tainted(port, 'test')
    assert endpoint_is_tainted(port) is True, '登记后必须判脏'


def test_untracked_port_is_treated_as_locked():
    """不可判定（None）必须按锁定处理，不得被当成干净。"""
    tr = OwnerSerialTransport(SlottedPort())
    assert tr.stream_locked is True
    assert tr.write(ZERO) is False


def test_unknown_state_blocks_even_with_injection():
    """注入"干净"也不能压过端口不可判定：仍在启动态静默。"""
    port = SlottedPort()                     # 未登记干净
    arb = Arbiter()
    arb.on_publisher_count(1, 0)
    arb.on_firmware_confirm(True)
    tr = OwnerSerialTransport(port)
    chain = StraightCommandChain(arb, tr)
    mark_chain_clean(chain, receipt=_SIM, mark_port=False)
    assert arb.receiver_clean is False, '不可判定的端口不得被注入放行'
    chain.tick(100.0)
    assert bytes(port.stream) == b''


def test_unwritable_port_is_rejected_after_first_failure():
    """无法记录状态时，适配器拒绝继续使用该端口。"""
    port = SlottedPort(short_at=1, short_bytes=2)
    mark_endpoint_clean(port, receipt=_SIM)       # 先有干净证据才能写出第一帧
    tr = OwnerSerialTransport(port)
    assert tr.stream_locked is False
    assert tr.write(ZERO) is False           # 短写
    assert tr.stream_locked is True
    assert any('stream locked' in d for d in tr.stats.detail), tr.stats.detail
    # 重建
    tr2 = OwnerSerialTransport(port)
    assert tr2.stream_locked is True


# ============================================ 生产候选无无条件放行入口
ALLOWED_PRODUCTION_CALLS = {
    # 唯一允许的转发点：测试专用入口，且内部有 simulated=True 校验与
    # 端口锁定检查。它不是无条件放行入口。
    'c50c_uart3/chassis_cmd/chain.py',
}


def test_production_modules_have_no_unconditional_release_call():
    """干净状态注入在生产模块中只有一处受控转发，且无无条件放行。"""
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    hits = []
    for p in (root / 'c50c_uart3').rglob('*.py'):
        if 'test' in p.parts:
            continue
        text = p.read_text(encoding='utf-8')
        for name in ('mark_receiver_clean', 'mark_clean_state_verified'):
            if name in text:
                # 允许定义，不允许调用
                for i, line in enumerate(text.splitlines(), 1):
                    s = line.strip()
                    if (name in s and not s.startswith('def ')
                            and str(p.relative_to(root)) not in ALLOWED_PRODUCTION_CALLS
                            and 'bridge.py' not in p.name):
                        hits.append(f'{p.relative_to(root)}:{i}: {s}')
    assert not hits, '生产模块出现干净状态注入调用：\n' + '\n'.join(hits)


def test_clean_state_injection_rejects_non_simulated():
    chain, _, arb = make(SlottedPort(), clean=False)
    assert not hasattr(chain, 'mark_clean_state_verified')
    with pytest.raises(TypeError):
        mark_chain_clean(chain, receipt=True)
    with pytest.raises(ValueError):
        chain._mark_clean_state_verified(simulated=False)
    with pytest.raises(TypeError):
        chain._mark_clean_state_verified()
    assert arb.receiver_clean is False, '未登记干净的端口不得被放行'
    mark_chain_clean(chain, receipt=_SIM, mark_port=True)   # 显式构造干净后才可以
    assert arb.receiver_clean is True
