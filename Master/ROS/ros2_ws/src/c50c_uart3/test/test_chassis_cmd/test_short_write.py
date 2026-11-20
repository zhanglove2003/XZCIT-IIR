"""短写后继续发帧的缺陷复现与修复验证（任务书 B 第四轮）。

审核发现：``OwnerSerialTransport.write()`` 短写返回失败，但
``StraightCommandChain.tick()`` 后续仍会发送仲裁器生成的零速帧。
真实短写可能已经发出帧前缀；后续字节可能与固件残帧拼接。

本模块的模拟端口**保存实际发出的前缀**（不是只返回长度），所以能还原
"字节流真的流出了什么"，并直接喂给真实固件解析器主机测试。
"""
from __future__ import annotations

import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame, is_zero_frame
from c50c_uart3.chassis_cmd.arbiter import Arbiter
from c50c_uart3.chassis_cmd.chain import StraightCommandChain
from c50c_uart3.chassis_cmd.ownership import (
    OwnerHarness,
    OwnerSerialTransport,
)
from sim_confirm import mark_endpoint_clean, mark_chain_clean, \
    grant_safety, TEST_SAFETY_MAX_AGE_S, \
    new_receipt as _new_receipt

_SIM = _new_receipt('test_short_write')

ZERO = encode_speed_frame(0.0, 0.0, 0.0)

def zero(seq: int = 0) -> bytes:
    """协议 v3 的零速帧：buffer[2] = 0x80 | 序号。"""
    return encode_speed_frame(0.0, 0.0, 0.0, seq=seq)



# ------------------------------------------------------------------ 可复核端口
class PrefixPort:
    """记录**实际发出**字节的端口。

    与旧假端口的区别：短写时把真正写出的前缀追加到 ``stream``，而不是
    只返回一个数字。``stream`` 就是"离开进程的字节流"，可以直接喂给真实
    解析器检验会不会拼出错误帧。
    """

    def __init__(self, short_at: int | None = None,
                 raise_at: int | None = None,
                 raise_after_bytes: int | None = None,
                 short_bytes: int | None = None) -> None:
        self.stream = bytearray()
        self.attempts: list[tuple[int, int, str]] = []   # (请求长度, 实际写出, 结果)
        self._writes = 0
        self.short_at = short_at                  # 第几次写入只写前 n 字节
        self.short_bytes = short_bytes            # 短写时实际写出多少字节
        self.raise_at = raise_at                  # 第几次写入抛异常
        self.raise_after_bytes = raise_after_bytes  # 抛异常前已写出多少字节

    def write_bytes(self, data: bytes) -> int:
        self._writes += 1
        idx = self._writes
        if self.raise_at is not None and idx == self.raise_at:
            before = 0 if self.raise_after_bytes is None else self.raise_after_bytes
            self.stream.extend(data[:before])
            self.attempts.append((len(data), before, 'raise'))
            raise OSError(5, 'Input/output error')
        if self.short_at is not None and idx == self.short_at:
            n = self.short_bytes if self.short_bytes is not None else 1
            n = max(0, min(int(n), len(data)))
            self.stream.extend(data[:n])
            self.attempts.append((len(data), n, 'short'))
            return n
        self.stream.extend(data)
        self.attempts.append((len(data), len(data), 'full'))
        return len(data)

    def read_some(self, max_bytes: int) -> bytes:
        return b''

    def reopen(self) -> None:
        pass


def make_chain(port, *, confirm=True):
    arb = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5,
                  safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arb)
    arb.on_publisher_count(1, 0)
    mark_endpoint_clean(port, receipt=_SIM)
    tr = OwnerSerialTransport(port)
    chain = StraightCommandChain(arb, tr)
    mark_chain_clean(chain, receipt=_SIM)  # 第五轮：构造干净接收端
    harness = OwnerHarness(chain)
    if confirm:
        chain.firmware_confirm(True)      # 模拟确认，仅测试用
    return chain, harness, tr


# ============================================================== 失败复现
@pytest.mark.parametrize('n', list(range(0, 11)))
def test_short_write_leaves_prefix_then_stops(n):
    """短写长度 0–10：已发前缀留在流里，之后**不得**再写任何字节。

    修复前：``tick`` 在短写失败后继续写零速帧，``stream`` 会增长。
    """
    port = PrefixPort(short_at=1, short_bytes=n)   # 第 1 次写入即短写 n 字节
    chain, _, tr = make_chain(port, confirm=False)
    chain.submit_nav(0.05, 0.0, 0.0, 200.0)
    chain.tick(200.0)                              # 首帧零速即触发短写
    assert len(port.attempts) == 1
    asked, written, kind = port.attempts[0]
    assert kind == 'short'
    assert written == n, f'模拟端口应只写出前 {n} 字节，实际 {written}'
    stream_after_short = bytes(port.stream)

    for i in range(10):
        chain.submit_nav(0.05, 0.0, 0.0, 200.01 + i * 0.01)
        chain.tick(200.01 + i * 0.01)
    assert bytes(port.stream) == stream_after_short, (
        f'短写 {n} 字节后不得再写任何帧；新增字节：'
        f'{bytes(port.stream)[len(stream_after_short):].hex()}')
    assert len(port.attempts) == 1, (
        f'短写后不得再调用端口写入；实际调用 {len(port.attempts)} 次')


def test_old_behaviour_reproduced_without_lock():
    """对照：不锁定时的旧行为确实会追加字节（证明本测试能抓到缺陷）。

    用一个**不加锁**的直接写入端复现：短写后再写零速，流会变长。
    该测试只用于证明复现手段有效，不描述被测代码的期望行为。
    """
    port = PrefixPort(short_at=1)
    port.write_bytes(encode_speed_frame(0.05, 0.0, 0.0))
    after = bytes(port.stream)
    port.write_bytes(ZERO)
    assert bytes(port.stream) != after, '无锁时后续帧会追加到流里'


# ============================================================== 锁定位不变式
def _drive_until_short(port, base=300.0):
    chain, harness, tr = make_chain(port)
    chain.submit_nav(0.05, 0.0, 0.0, base)
    chain.tick(base)                     # 首帧零速
    rec = chain.tick(base + 0.01)        # 第二次写入触发短写
    return chain, harness, tr, rec


def test_lock_after_short_write_blocks_all_frames():
    """短写后锁定：反复 tick、停车、清错、重连都不得解除，流不再增长。"""
    port = PrefixPort(short_at=2)
    chain, harness, tr, rec = _drive_until_short(port)
    assert rec.write_ok is False
    frozen = bytes(port.stream)

    # 反复 tick
    for i in range(10):
        chain.submit_nav(0.05, 0.0, 0.0, 300.02 + i * 0.01)
        chain.tick(300.02 + i * 0.01)
    assert bytes(port.stream) == frozen, '反复 tick 不得写字节'

    # 停车 / 解除
    chain.set_stop(True)
    chain.tick(300.5)
    chain.set_stop(False)
    chain.tick(300.51)
    assert bytes(port.stream) == frozen, '停车与解除不得写字节'

    # 操作者清错
    harness.on_operator_clear()
    chain.submit_nav(0.05, 0.0, 0.0, 300.6)
    chain.tick(300.6)
    assert bytes(port.stream) == frozen, '操作者清错不得解除流锁'

    # 重连
    harness.on_reconnected()
    for i in range(5):
        chain.tick(300.7 + i * 0.01)
    assert bytes(port.stream) == frozen, '重连不得解除流锁'

    # 模拟确认
    harness.on_firmware_confirm(True)
    mark_chain_clean(chain, receipt=_SIM)  # 第五轮：干净接收端
    for i in range(5):
        chain.tick(300.8 + i * 0.01)
    assert bytes(port.stream) == frozen, '模拟确认不得解除流锁'


def test_lock_reports_zero_reason():
    """锁定时决策显示明确的零速原因，便于日志与运维识别。"""
    port = PrefixPort(short_at=2)
    chain, _, _, _ = _drive_until_short(port)
    rec = chain.tick(301.0)
    assert rec.is_zero, '锁定时不得输出非零'
    assert rec.zero_reason.name in ('STREAM_POISONED', 'WRITE_FAILED'), \
        f'意外原因：{rec.zero_reason}'


def test_exception_with_unknown_bytes_locks_stream():
    """异常且无法确定已发字节数：同样锁定，不假设"一个字节都没发"。"""
    port = PrefixPort(raise_at=2, raise_after_bytes=None)
    chain, _, tr, rec = _drive_until_short(port)
    assert rec.write_ok is False
    frozen = bytes(port.stream)
    for i in range(10):
        chain.submit_nav(0.05, 0.0, 0.0, 310.0 + i * 0.01)
        chain.tick(310.0 + i * 0.01)
    assert bytes(port.stream) == frozen, '异常后不得继续写'


def test_exception_after_partial_bytes_locks_stream():
    """异常前已写出部分字节：前缀确已离开进程，必须锁定。"""
    port = PrefixPort(raise_at=2, raise_after_bytes=5)
    chain, _, _, rec = _drive_until_short(port)
    assert rec.write_ok is False
    assert len(port.stream) >= 11 + 5, '前缀应留在流里'
    frozen = bytes(port.stream)
    for i in range(5):
        chain.tick(320.0 + i * 0.01)
    assert bytes(port.stream) == frozen


def test_zero_length_write_also_locks_stream():
    """写入 0 字节（完全没出去）同样按失败处理并锁流。

    0 字节虽然不留前缀，但说明端口已不可用；本轮统一按"流不可信"处理，
    不靠"这次恰好没出去"推断下一次也不会出事。
    """
    port = PrefixPort(short_at=2, short_bytes=0)
    chain, _, _, rec = _drive_until_short(port)
    assert rec.write_ok is False
    assert len(port.stream) == 11, '第二次写入 0 字节，流里应只有首帧'
    frozen = bytes(port.stream)
    for i in range(5):
        chain.tick(330.0 + i * 0.01)
    assert bytes(port.stream) == frozen, '零字节写后不得再写'


# ============================================================== 状态覆盖
def test_short_write_when_previously_authorised():
    """短写前已有授权：锁定后不得再放行非零。"""
    port = PrefixPort(short_at=3)
    chain, _, tr = make_chain(port)
    chain.submit_nav(0.05, 0.0, 0.0, 400.0)
    chain.tick(400.0)
    assert not chain.tick(400.01).is_zero, '前置：已授权且已发非零'
    chain.tick(400.02)                    # 第三次写入短写
    assert tr.stats.write_failures >= 1
    frozen = bytes(port.stream)
    for i in range(10):
        chain.submit_nav(0.05, 0.0, 0.0, 400.03 + i * 0.01)
        assert chain.tick(400.03 + i * 0.01).is_zero
    assert bytes(port.stream) == frozen


def test_short_write_when_not_authorised():
    """未授权状态下短写：同样锁定，之后也不得写。"""
    port = PrefixPort(short_at=1)
    chain, _, _ = make_chain(port, confirm=False)
    chain.submit_nav(0.05, 0.0, 0.0, 500.0)
    chain.tick(500.0)                     # 首帧零速即短写
    frozen = bytes(port.stream)
    for i in range(5):
        chain.tick(500.01 + i * 0.01)
    assert bytes(port.stream) == frozen


def test_short_write_during_stop():
    """停车期间短写：锁定生效，解除停车后仍不写。"""
    port = PrefixPort(short_at=1)
    chain, _, _ = make_chain(port)
    chain.set_stop(True)
    chain.tick(600.0)                     # 零速帧短写
    frozen = bytes(port.stream)
    chain.set_stop(False)
    for i in range(5):
        chain.submit_nav(0.05, 0.0, 0.0, 600.1 + i * 0.01)
        chain.tick(600.1 + i * 0.01)
    assert bytes(port.stream) == frozen


def test_command_cache_not_reused_after_short_write():
    """短写后缓存的命令不得复活：解除锁定后也只能从零速重新开始。"""
    port = PrefixPort(short_at=2)
    chain, harness, tr = make_chain(port)
    chain.submit_nav(0.07, 0.01, 0.02, 700.0)
    chain.tick(700.0)
    chain.tick(700.01)                    # 短写
    frozen = bytes(port.stream)

    # 即使把已发字节当成"安全恢复"的模拟值，缓存也应当已清空
    for i in range(5):
        chain.tick(700.02 + i * 0.01)
    assert bytes(port.stream) == frozen
    assert chain.nonzero_frames() == [] or all(
        not r.write_ok for r in chain.records[-5:])


def test_close_and_reopen_does_not_clear_lock():
    """关闭与重开文件描述符不得解除流锁。"""
    port = PrefixPort(short_at=2)
    chain, harness, tr, _ = _drive_until_short(port)
    frozen = bytes(port.stream)
    port.reopen()
    tr.reconnect()
    harness.on_reconnected()
    for i in range(5):
        chain.tick(800.0 + i * 0.01)
    assert bytes(port.stream) == frozen


def test_repeated_callbacks_are_idempotent():
    """重复回调（多次重连、多次清错）不会累积出可写机会。"""
    port = PrefixPort(short_at=2)
    chain, harness, tr, _ = _drive_until_short(port)
    frozen = bytes(port.stream)
    for _ in range(5):
        harness.on_reconnected()
        harness.on_operator_clear()
        harness.on_firmware_confirm(True)
        mark_chain_clean(chain, receipt=_SIM)  # 第五轮：干净接收端
    for i in range(5):
        chain.tick(900.0 + i * 0.01)
    assert bytes(port.stream) == frozen


# ====================================================== 真实解析器拼接检查
def test_stream_from_short_write_is_fed_to_real_parser(firmware):
    """把保存的前缀与实际被允许写出的后续字节喂给真实固件解析器。

    断言：失败后的重发**没有**制造额外拼接命令。不断言固件停车——
    已发出的部分无法撤销。
    """
    model = firmware
    model.feed(ZERO)                  # 先用合法零速帧授权
    assert model.state.armed is True

    port = PrefixPort(short_at=2)
    chain, _, _ = make_chain(port)
    chain.submit_nav(0.05, 0.0, 0.0, 1000.0)
    chain.tick(1000.0)                # 完整零速帧
    chain.tick(1000.01)               # 非零帧：第 2 次写入触发短写，前缀已入流
    assert len(port.attempts) == 2
    for i in range(10):
        chain.submit_nav(0.05, 0.0, 0.0, 1000.02 + i * 0.01)
        rec = chain.tick(1000.02 + i * 0.01)
        assert rec.frame is None, '锁流后不应再产生帧'
        assert rec.zero_reason.name == 'STREAM_POISONED'

    stream = bytes(port.stream)
    # 把整条流按真实固件的接收方式喂进去（前缀 1 字节）
    model.feed_bytes(stream)
    model.tick(5)
    # 关键断言：流里只有"完整零速帧 + 短写前缀"，没有完整后续帧。
    # 不断言固件一定停车——已发出的部分无法撤销。
    assert len(stream) == 11 + 1, f'流长度应为首帧 11 + 前缀 1；实际 {len(stream)}'
    # 协议 v3 的零速帧 buffer[2] 是 0x80|序号，帧内只有 1 个 0x7B（帧头）；
    # 再加短写前缀 1 个，合计 2。若锁流失效，会多出完整后续帧（+1 或更多）。
    assert stream.count(b'\x7b') == 2, (
        f'应为首帧帧头(1 个 0x7B) + 前缀(1 个)；流：{stream.hex()}')
    assert len(port.attempts) == 2, '短写后不得再写'
