"""第八轮定向测试：模拟确认入口已被隔离在测试设施内。

审核要点（任务书第八轮 B）：
1. 生产模块上不能再有公开的"模拟干净 / 模拟固件确认"入口。
2. 不能用一个调用者自报的布尔值证明接收端干净。
3. 未知接收状态仍保持静默；第七轮的容量修复不受影响。
4. 测试设施仍能在明确的内存模型里构造干净状态。
"""
from __future__ import annotations

import ast
import pathlib

import pytest

from c50c_uart3.chassis_cmd import chain as chain_mod
from c50c_uart3.chassis_cmd import ownership as ownership_mod
from c50c_uart3.chassis_cmd.arbiter import Arbiter
from c50c_uart3.chassis_cmd.chain import StraightCommandChain
from c50c_uart3.chassis_cmd.ownership import OwnerSerialTransport, endpoint_is_tainted
from sim_confirm import SimCleanReceipt, mark_chain_clean, mark_endpoint_clean, new_receipt

_SIM = new_receipt('r8')

ROOT = pathlib.Path(__file__).resolve().parents[2]          # …/c50c_uart3
PRODUCTION = ROOT / 'c50c_uart3'

FORBIDDEN_PUBLIC = (
    'mark_endpoint_clean_for_test',
    'mark_clean_state_verified',
)


class Port:
    """可写属性的普通内存端口。"""

    def __init__(self) -> None:
        self.stream = bytearray()
        self.writes = 0

    def write_bytes(self, data: bytes) -> int:
        self.writes += 1
        self.stream.extend(data)
        return len(data)

    def read_some(self, max_bytes: int) -> bytes:
        return b''

    def reopen(self) -> None:
        pass


class SlottedPort:
    """禁止 setattr 的端口，走进程内登记表。"""

    __slots__ = ('stream', 'writes')

    def __init__(self) -> None:
        self.stream = bytearray()
        self.writes = 0

    def write_bytes(self, data: bytes) -> int:
        self.writes += 1
        self.stream.extend(data)
        return len(data)

    def read_some(self, max_bytes: int) -> bytes:
        return b''

    def reopen(self) -> None:
        pass


def _chain(port):
    arb = Arbiter()
    arb.on_publisher_count(1, 0)
    tr = OwnerSerialTransport(port)
    return StraightCommandChain(arb, tr), arb


# ------------------------------------------------ 1. 生产模块没有公开入口
def test_production_modules_expose_no_public_sim_entry():
    for module in (ownership_mod, chain_mod):
        for name in FORBIDDEN_PUBLIC:
            assert not hasattr(module, name), f'{module.__name__} 仍有公开入口 {name}'


def test_chain_instance_has_no_public_sim_method():
    chain, _ = _chain(Port())
    for name in FORBIDDEN_PUBLIC:
        assert not hasattr(chain, name)


GATE_MODULE = 'c50c_uart3/chassis_cmd/bridge.py'


def test_production_source_never_calls_private_entry():
    """生产源码里两个私有入口唯一允许的调用点是桥的收据门。

    桥的 ``_open_session(receipt)`` 是受控入口：先校验测试收据，再调用
    私有入口。除该文件外，任何生产模块调用私有入口都算违规。
    """
    hits = []
    for path in PRODUCTION.rglob('*.py'):
        rel = str(path.relative_to(ROOT))
        tree = ast.parse(path.read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            name = getattr(func, 'attr', None) or getattr(func, 'id', None)
            if name in ('_mark_endpoint_clean_for_test', '_mark_clean_state_verified'):
                if rel == GATE_MODULE:
                    continue
                hits.append(f'{rel}:{node.lineno}: {name}')
    assert not hits, '生产源码调用了测试注入入口：\n' + '\n'.join(hits)


def test_gate_module_is_the_only_private_entry_caller():
    """核对受控入口本身：调用前必须校验收据。"""
    tree = ast.parse((ROOT / GATE_MODULE).read_text(encoding='utf-8'))
    openers = [n for n in ast.walk(tree)
               if isinstance(n, ast.FunctionDef) and n.name in ('_open_session',)]
    assert openers, '桥缺少 _open_session 入口'
    body = ast.get_source_segment(
        (ROOT / GATE_MODULE).read_text(encoding='utf-8'), openers[0]) or ''
    assert 'require_receipt(receipt)' in body
    assert body.index('require_receipt') < body.index('_mark_clean_state_verified')


def test_no_production_import_of_test_facility():
    """生产模块不得 import 测试设施（``test_chassis_cmd/sim_confirm``）。

    生产侧只允许引用 ``chassis_cmd/sim_receipt.py``——那只是收据类型，
    本身不能构造合法收据（构造权在 test 目录）。
    """
    hits = []
    for path in PRODUCTION.rglob('*.py'):
        text = path.read_text(encoding='utf-8')
        if 'sim_confirm' in path.name:
            continue
        if 'test_chassis_cmd' in text or 'sim_confirm import' in text:
            hits.append(str(path.relative_to(ROOT)))
    assert not hits, '生产模块引用了测试设施：' + ', '.join(hits)


# ---------------------------------- 2. 自报布尔值不能证明接收端干净
@pytest.mark.parametrize('bad', [True, False, 1, 'simulated', None])
def test_receipt_rejects_self_reported_booleans(bad):
    port = Port()
    with pytest.raises(TypeError):
        mark_endpoint_clean(port, receipt=bad)
    chain, _ = _chain(port)
    with pytest.raises(TypeError):
        mark_chain_clean(chain, receipt=bad)


def test_receipt_requires_non_empty_label():
    with pytest.raises(ValueError):
        SimCleanReceipt('')


def test_private_entry_still_rejects_bare_simulated():
    chain, _ = _chain(SlottedPort())
    with pytest.raises(TypeError):
        chain._mark_clean_state_verified()          # noqa: SLF001
    with pytest.raises(ValueError):
        chain._mark_clean_state_verified(simulated=False)   # noqa: SLF001


# ------------------------------- 3. 未知状态仍静默（注入不绕过不可判定）
def test_unknown_port_stays_silent_even_with_test_receipt():
    port = SlottedPort()                # 从未登记干净 → 不可判定
    chain, arb = _chain(port)
    mark_chain_clean(chain, receipt=_SIM, mark_port=False)
    assert arb.receiver_clean is False
    rec = chain.tick(100.0)
    assert rec.frame is None
    assert bytes(port.stream) == b''


def test_receipt_on_port_unlocks_only_after_explicit_mark():
    port = Port()
    chain, arb = _chain(port)
    mark_chain_clean(chain, receipt=_SIM, mark_port=False)
    assert arb.receiver_clean is False
    mark_endpoint_clean(port, receipt=_SIM)
    mark_chain_clean(chain, receipt=_SIM)
    assert arb.receiver_clean is True
    rec = chain.tick(100.0)
    assert rec.frame is not None and rec.is_zero


# ---------------------------- 4. 测试设施仍可构造，且容量修复仍生效
def test_facility_can_construct_clean_state_in_memory():
    port = Port()
    assert endpoint_is_tainted(port) is None
    mark_endpoint_clean(port, receipt=new_receipt('memory'))
    assert endpoint_is_tainted(port) is False


def test_capacity_fix_still_holds_after_isolation():
    """第七轮修复的前提（登记表容量）仍存在，未被本轮改动破坏。"""
    assert getattr(ownership_mod, '_TAINTED_CAPACITY', None) == 512
    assert hasattr(ownership_mod, 'registry_saturated')
