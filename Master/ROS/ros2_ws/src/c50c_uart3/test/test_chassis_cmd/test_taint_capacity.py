"""B 第七轮：流锁登记表容量与淘汰策略的绕过（审核反例）。

反例：端口 p 先被注入模拟干净状态并登记为干净；短写后 p 被标记污染。
但 p 的属性写不进去（``__slots__``），污染只能进登记表。此后向登记表
再塞入 512 个端口，p 的污染记录被先进先出淘汰，而 p 的**干净记录仍在**。
于是重建适配器时 ``endpoint_is_tainted(p)`` 返回"干净"，桥继续写字节。

本文件的断言只依赖内存端口，不打开任何真实串口。
"""

from __future__ import annotations

import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame, ownership
from c50c_uart3.chassis_cmd.arbiter import Arbiter, ZeroReason
from c50c_uart3.chassis_cmd.chain import StraightCommandChain
from c50c_uart3.chassis_cmd.ownership import (
    OwnerSerialTransport,
    endpoint_is_tainted,
)
from sim_confirm import mark_endpoint_clean, mark_chain_clean, \
    new_receipt as _new_receipt

_SIM = _new_receipt('test_taint_capacity')

#: 协议 v3 零速帧：buffer[2] = 0x80 | 序号。
ZERO = encode_speed_frame(0.0, 0.0, 0.0, seq=0)


@pytest.fixture(autouse=True)
def _isolate_registry():
    """每个用例独占登记表：污染表、干净表、饱和标志都清空后还原。"""
    # 修复前的基线只有列表登记表；用 getattr 兼容，便于复跑"修复前"证据。
    ids = getattr(ownership, '_TAINTED_IDS', None)
    clean_list = getattr(ownership, '_CLEAN_OBJECTS', None)
    before_ids = dict(ids) if ids is not None else None
    before_clean = list(clean_list) if clean_list is not None else None
    before_saturated = getattr(ownership, '_saturated', False)

    def restore():
        if ids is not None:
            ids.clear()
            ids.update(before_ids)
        if clean_list is not None:
            clean_list[:] = before_clean
        ownership._saturated = before_saturated

    if ids is not None:
        ids.clear()
    if clean_list is not None:
        clean_list.clear()
    ownership._saturated = False
    yield
    restore()


class SlottedPort:
    """短写端口：只写出前 ``short_bytes`` 个字节，且不能写属性。"""

    __slots__ = ('stream', 'writes', 'short_at', 'short_bytes', 'generation')

    def __init__(self, short_at: int = 1, short_bytes: int = 2) -> None:
        self.stream = bytearray()
        self.writes = 0
        self.short_at = short_at
        self.short_bytes = short_bytes
        self.generation = 0

    def write_bytes(self, data: bytes) -> int:
        self.writes += 1
        if self.short_at is not None and self.writes >= self.short_at:
            n = min(self.short_bytes, len(data))
        else:
            n = len(data)
        self.stream.extend(data[:n])
        return n

    def reopen(self) -> None:
        self.generation += 1


def _build(port: SlottedPort, *, clean: bool = True):
    """造一条命令链。``clean=True`` 时先给端口注入模拟干净状态。"""
    if clean:
        mark_endpoint_clean(port, receipt=_SIM)
    arb = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5)
    arb.on_publisher_count(1, 0)
    arb.on_firmware_confirm(True)          # 模拟确认：仅测试用
    tr = OwnerSerialTransport(port)
    chain = StraightCommandChain(arb, tr)
    mark_chain_clean(chain, receipt=_SIM)
    return chain, tr, arb


def _fill_registry_with_other_ports(count: int) -> None:
    """向污染登记表塞入其它端口，触发按容量淘汰。"""
    for _ in range(count):
        other = SlottedPort()
        ownership.mark_endpoint_tainted(other, '容量填充')


def _short_write(port: SlottedPort) -> bytes:
    """让端口发生一次短写，返回此时已发出的字节。"""
    chain, tr, arb = _build(port)
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    chain.tick(100.0)                      # 首帧零速
    chain.tick(100.01)                     # 非零帧 → 短写
    assert tr.stream_locked is True, '前置：短写后适配器必须锁定'
    return bytes(port.stream)


# ---------------------------------------------------- 1. 审核反例本身
def test_review_counterexample_registry_eviction_does_not_unlock():
    """512 项淘汰后，污染端口不得被判为干净，也不得再写字节。"""
    port = SlottedPort(short_at=2, short_bytes=2)
    frozen = _short_write(port)
    # 第一次 tick 写出完整零速帧，第二次 tick 短写出 2 字节前缀。
    assert frozen == ZERO + b'{\x00', frozen

    _fill_registry_with_other_ports(ownership._TAINTED_CAPACITY + 1)

    chain2, tr2, _arb2 = _build(port, clean=False)
    chain2.submit_nav(0.05, 0.0, 0.0, 200.0)
    chain2.tick(200.0)
    chain2.tick(200.01)

    assert bytes(port.stream) == frozen, (
        '登记表淘汰后仍不得追加字节；新增：'
        f'{bytes(port.stream)[len(frozen):].hex()}')
    assert tr2.stream_locked is True, '污染记录丢失时必须按锁定处理'


def test_review_counterexample_isolation_hex():
    """用隔离的十六进制标记复刻审核观察：6162 → 616258595a。

    把审核的 ``61 62`` 当作已被短写切开的残帧前缀，再触发登记表淘汰，
    重建适配器后不得再追加任何字节。
    """
    port = SlottedPort(short_at=1, short_bytes=2)
    port.stream.extend(b'ab')
    chain, tr, _arb = _build(port)
    chain.tick(300.0)                       # 一次短写，只写出 b'{\x00'
    assert bytes(port.stream) == b'ab{\x00', bytes(port.stream)
    assert tr.stream_locked is True

    _fill_registry_with_other_ports(ownership._TAINTED_CAPACITY + 1)
    chain2, _tr2, _arb2 = _build(port, clean=False)
    chain2.submit_nav(0.05, 0.0, 0.0, 301.0)
    chain2.tick(301.0)
    chain2.tick(301.01)
    assert bytes(port.stream).hex() == '61627b00', bytes(port.stream).hex()


# ------------------------------------------- 2. 清洁 / 污染记录的优先级
def test_taint_outranks_stale_clean_record():
    """同一对象既有旧干净记录又有新污染记录时，必须以污染为准。"""
    port = SlottedPort()
    mark_endpoint_clean(port, receipt=_SIM)     # 先有干净记录
    assert endpoint_is_tainted(port) is False
    ownership.mark_endpoint_tainted(port)  # 再有污染记录（属性写不进去）
    assert endpoint_is_tainted(port) is True


def test_clean_marker_after_taint_cannot_reset_without_explicit_reset():
    """测试注入干净也不能压过已污染对象，除非显式清除。"""
    port = SlottedPort()
    ownership.mark_endpoint_tainted(port)
    mark_endpoint_clean(port, receipt=_SIM)
    assert endpoint_is_tainted(port) is True


def test_repeated_marking_is_idempotent_under_capacity():
    """重复标记不得靠占位把旧记录挤出去。"""
    port = SlottedPort()
    ownership.mark_endpoint_tainted(port)
    for _ in range(ownership._TAINTED_CAPACITY * 2):
        ownership.mark_endpoint_tainted(port)
    _fill_registry_with_other_ports(ownership._TAINTED_CAPACITY)
    assert endpoint_is_tainted(port) is True


# ------------------------------------------------------ 3. 容量边界
def test_registry_overflow_keeps_taint_decision_safe():
    """登记表装满后，被淘汰的对象必须落到"不可判定"，不是"干净"。"""
    port = SlottedPort()
    mark_endpoint_clean(port, receipt=_SIM)
    ownership.mark_endpoint_tainted(port)
    _fill_registry_with_other_ports(ownership._TAINTED_CAPACITY)
    # 允许实现选择"继续保留"或"记为不可判定"，但绝不允许返回 False。
    assert endpoint_is_tainted(port) is not False


def test_capacity_boundary_below_limit_still_blocks():
    """未达容量时，污染端口必须仍被锁定。"""
    port = SlottedPort()
    mark_endpoint_clean(port, receipt=_SIM)
    ownership.mark_endpoint_tainted(port)
    _fill_registry_with_other_ports(ownership._TAINTED_CAPACITY - 2)
    assert endpoint_is_tainted(port) is True


# ------------------------------------------------ 4. 组合路径（适配器）
def test_new_adapter_on_evicted_port_stays_locked():
    """淘汰后重建的适配器必须锁定，且不得写任何帧。"""
    port = SlottedPort(short_at=2, short_bytes=2)
    frozen = _short_write(port)
    _fill_registry_with_other_ports(ownership._TAINTED_CAPACITY + 1)
    tr = OwnerSerialTransport(port)
    assert tr.stream_locked is True
    assert tr.write(ZERO) is False
    assert bytes(port.stream) == frozen


def test_wrapped_adapter_on_evicted_port_stays_locked():
    """不同包装对象同样不得解除锁定。"""
    from c50c_uart3.chassis_cmd.ownership import BlockedNonzeroGuard

    port = SlottedPort(short_at=2, short_bytes=2)
    frozen = _short_write(port)
    _fill_registry_with_other_ports(ownership._TAINTED_CAPACITY + 1)
    inner = OwnerSerialTransport(port)
    guard = BlockedNonzeroGuard(inner, blocked=False)
    assert guard.stream_locked is True
    assert guard.write(ZERO) is False
    assert bytes(port.stream) == frozen


def test_arbiter_stays_poisoned_across_registry_eviction():
    """仲裁器不得因为登记表淘汰而回到可发送状态。"""
    port = SlottedPort(short_at=2, short_bytes=2)
    chain, tr, arb = _build(port)
    chain.submit_nav(0.05, 0.0, 0.0, 100.0)
    chain.tick(100.0)
    chain.tick(100.01)
    assert arb.stream_poisoned is True
    _fill_registry_with_other_ports(ownership._TAINTED_CAPACITY + 1)
    rec = chain.tick(100.5)
    assert rec.frame is None
    assert rec.zero_reason is ZeroReason.STREAM_POISONED


def test_restart_simulation_after_eviction_stays_silent():
    """模拟重启（新建全部对象）在残帧未清除时仍须静默。"""
    port = SlottedPort(short_at=2, short_bytes=2)
    frozen = _short_write(port)
    _fill_registry_with_other_ports(ownership._TAINTED_CAPACITY + 1)
    arb = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5)
    arb.on_publisher_count(1, 0)
    chain = StraightCommandChain(arb, OwnerSerialTransport(port))
    chain.submit_nav(0.05, 0.0, 0.0, 400.0)
    for i in range(10):
        rec = chain.tick(400.0 + i * 0.01)
        assert rec.frame is None
    assert bytes(port.stream) == frozen


# --------------------------------------- 5. 干净注入与生产入口的隔离
def test_simulated_injection_requires_explicit_and_never_overrides_taint():
    with pytest.raises(ValueError):
        Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5).mark_receiver_clean(
            simulated=False)


def test_no_production_module_calls_clean_injection():
    """生产模块只允许 chain.py 为受控转发，不得有其它无条件放行调用。"""
    from pathlib import Path

    pkg = Path(ownership.__file__).resolve().parent
    allowed = {'chain.py'}
    offenders = []
    for path in sorted(pkg.glob('*.py')):
        text = path.read_text(encoding='utf-8')
        if path.name == 'ownership.py':
            # 该模块是定义处，不算调用点。
            continue
        if 'mark_endpoint_clean_for_test(' in text and path.name not in allowed:
            offenders.append(f'{path.name}:mark_endpoint_clean_for_test')
        if 'mark_receiver_clean(' in text and path.name not in {'arbiter.py', 'chain.py'}:
            offenders.append(f'{path.name}:mark_receiver_clean')
    assert offenders == [], offenders


def test_test_only_entry_points_are_not_used_by_production_code():
    """仅供测试的入口不得被生产模块引用，也不得出现在 __init__ 导出面。"""
    from pathlib import Path
    import c50c_uart3.chassis_cmd as pkg_root

    test_only = ('mark_endpoint_clean_for_test', '_forget_endpoint_state',
                 'registry_saturated')
    pkg = Path(pkg_root.__file__).resolve().parent
    init_text = (pkg / '__init__.py').read_text(encoding='utf-8')
    star = [line.strip() for line in init_text.splitlines()
            if line.strip().startswith(('from ', 'import ', '__all__'))]
    exposed = [line for line in star if 'ownership' in line and '*' in line]
    assert exposed == [], f'不得星号导出 ownership：{exposed}'
    offenders = []
    for path in sorted(pkg.glob('*.py')):
        if path.name == 'ownership.py':
            continue                      # 定义处
        text = path.read_text(encoding='utf-8')
        for name in test_only:
            if f'{name}(' in text:
                offenders.append(f'{path.name}:{name}')
    assert offenders == [], offenders
