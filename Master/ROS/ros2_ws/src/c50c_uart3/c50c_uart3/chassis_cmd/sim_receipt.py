"""测试构造凭证：唯一一种"生产入口接受的、只由测试构造"的类型。

生产模块只做类型校验；收据由测试目录里的 sim_confirm 模块的
``new_receipt()`` 构造。生产路径没有任何构造收据的入口。

两种收据对应两种语义，**不可互相替代**：

- :class:`SimCleanReceipt` —— "测试构造的干净接收端"。它只能证明
  ``endpoint_is_tainted`` 的端口状态，不能证明固件收到过任何东西。
- :class:`SimSessionReceipt` —— "测试构造的会话观察"。它代表测试里
  **确实从隔离伪终端的固件侧读到了会话帧**；用于在没有真实板子时
  贯通"会话握手后放行非零"的一段链路。生产路径同样没有构造入口，
  所以生产链路仍停在 ``RECEIVER_UNKNOWN``。
"""
from __future__ import annotations

#: 私有封印。模块级名字被改名（``_`` 前缀）后，从本模块 ``import *`` 拿不到；
#: 目前唯一的签发者是 :class:`~.handshake.SessionHandshake`，它只在真的读到
#: 线上会话帧之后构造收据。
SIM_SESSION_SEAL = 'sim-session'


class SimCleanReceipt:
    """测试干净的接收端收据。只有测试设施构造合法实例。"""

    __slots__ = ('label', '_seal')

    def __init__(self, label: str, *, _seal: str = 'sim-confirm') -> None:
        if not isinstance(label, str) or not label:
            raise ValueError('label 必须是非空字符串')
        if _seal != 'sim-confirm':
            raise TypeError('SimCleanReceipt 只能由测试设施构造')
        self.label = label
        self._seal = _seal


class SimSessionReceipt:
    """测试构造的会话观察收据（携带会话 ID 与观察时刻）。"""

    __slots__ = ('label', 'session_id', 'observed_at', 'frames_seen', '_seal')

    def __init__(self, label: str, session_id: int, observed_at: float, *,
                 frames_seen: int = 1, _seal: str = SIM_SESSION_SEAL) -> None:
        if not isinstance(label, str) or not label:
            raise ValueError('label 必须是非空字符串')
        if not isinstance(session_id, int) or not 0 <= session_id <= 0xFFFFFFFF:
            raise ValueError(f'session_id 必须是 32 位无符号整数，收到 {session_id!r}')
        if _seal != SIM_SESSION_SEAL:
            raise TypeError('SimSessionReceipt 只能由测试设施构造')
        self.label = label
        self.session_id = session_id
        self.observed_at = float(observed_at)
        self.frames_seen = int(frames_seen)
        self._seal = _seal


def require_receipt(receipt: object) -> SimCleanReceipt:
    """校验干净接收端收据；不接受布尔、字符串或任何自报值。"""
    if not isinstance(receipt, SimCleanReceipt):
        raise TypeError(
            '必须传入测试构造的 SimCleanReceipt，不接受布尔或其它自报值')
    return receipt


def require_session_receipt(receipt: object) -> SimSessionReceipt:
    """校验会话观察收据；不接受布尔、字符串或任何自报值。"""
    if not isinstance(receipt, SimSessionReceipt):
        raise TypeError(
            '必须传入测试构造的 SimSessionReceipt，不接受布尔或其它自报值')
    return receipt


__all__ = ['SimCleanReceipt', 'SimSessionReceipt', 'require_receipt',
           'require_session_receipt']
