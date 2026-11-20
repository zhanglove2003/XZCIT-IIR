"""测试设施：模拟"接收端干净"与"固件确认"的唯一构造函数。

为什么单独成模块：第八轮审核指出，`mark_endpoint_clean_for_test` /
`mark_clean_state_verified(simulated=True)` 原先挂在生产模块上，生产接入
代码可以直接 import 调用，把测试注入当成真实确认。现在：

1. 生产模块里的入口改名为私有的 ``_mark_endpoint_clean_for_test`` /
   ``_mark_clean_state_verified``；
2. 本模块位于 ``test/`` 目录，不进安装包；
3. 构造时必须显式给出 ``receipt=SimCleanReceipt(...)``，用类型区分"测试
   构造"与"调用者自报的布尔值"——裸 ``simulated=True`` 不被接受。
"""
from __future__ import annotations

import hashlib

from c50c_uart3.chassis_cmd import ownership as _ownership


from c50c_uart3.chassis_cmd.sim_receipt import SimCleanReceipt as _ProductionReceipt


class SimCleanReceipt(_ProductionReceipt):
    """测试构造的凭证。只有 test/ 里的代码会创建它。

    继承生产模块里的同名类型：生产入口只认该类型（``isinstance``），
    而构造收据的权限留在 test/ 目录。
    """

    __slots__ = ()


def _require(receipt) -> None:
    if not isinstance(receipt, SimCleanReceipt):
        raise TypeError('必须传入测试构造的 SimCleanReceipt，不接受布尔或其它自报值')


def mark_endpoint_clean(port, *, receipt) -> None:
    """声明该端口接收端"模拟干净"。写属性失败退回对象登记。"""
    _require(receipt)
    _ownership._mark_endpoint_clean_for_test(port)


def resolve_port(transport):
    """按适配器链逐层解开，取真正的接收端口对象（与 chain 的解析规则一致）。"""
    layer = transport
    port = getattr(layer, '_port', None)
    for _ in range(8):
        if port is not None or not hasattr(layer, '_inner'):
            break
        layer = layer._inner
        port = getattr(layer, '_port', None)
    return port if port is not None else layer


def mark_chain_clean(chain, *, receipt, mark_port: bool = True) -> None:
    """为命令链构造"模拟干净接收端 + 模拟固件确认"。

    等价于旧写法：先标记端口干净，再 ``mark_clean_state_verified(simulated=True)``。
    ``mark_port=False`` 只做固件确认，用来验证"无法判定的端口不得被放行"。
    """
    _require(receipt)
    transport = getattr(chain, '_transport', None)
    if mark_port and transport is not None:
        _ownership._mark_endpoint_clean_for_test(resolve_port(transport))
    chain._mark_clean_state_verified(simulated=True)   # noqa: SLF001


def new_receipt(label: str = 'test') -> SimCleanReceipt:
    return SimCleanReceipt(label)


# 2026-11-14 起仲裁器有安全许可门（无许可 = 禁非零）。既有测试验证的是
# 许可**之后**的仲裁/传输行为，这里提供统一的"模拟 fusion 在线且安全"
# 构造：时限放大到测试尺度，生产默认（0.5 s）不受影响；许可门本身的
# 拒绝/过期/钳制行为由 test_safety_gate.py 单独验证。
TEST_SAFETY_MAX_AGE_S = 3600.0


def grant_safety(arbiter, now: float = 0.0) -> None:
    """模拟一条有效安全许可（生产路径只能来自 /drive_safety 话题）。"""
    arbiter.on_safety_permission(True, 'test_granted',
                                 TEST_SAFETY_MAX_AGE_S, now)


def grant_safety_via_input(bridge, now: float | None = None) -> None:
    """经**生产路径**给桥授一条安全许可：RosStraightInput.on_safety。

    桥测试不直接改仲裁器内部状态，而是喂一条与 fusion_node
    ``drive_safety`` 话题同格式的 JSON 消息，验证接线本身。桥内仲裁器的
    许可时限上限放大到测试尺度（生产默认 0.5 s，见 test_safety_gate.py）。
    """
    import json as _json
    from types import SimpleNamespace as _NS
    arb = bridge.chain._arbiter                      # noqa: SLF001——测试助手
    arb._safety_max_age_s = TEST_SAFETY_MAX_AGE_S    # noqa: SLF001
    payload = _json.dumps({'v': 1, 'allowed': True, 'reason': 'test_granted',
                           'seq': 1, 'ttl_s': TEST_SAFETY_MAX_AGE_S},
                          separators=(',', ':'))
    bridge.input.on_safety(_NS(data=payload))


def digest(note: str) -> str:
    """把构造说明折成短摘要，便于日志里核对使用的是模拟注入。"""
    return hashlib.sha256(note.encode('utf-8')).hexdigest()[:12]
