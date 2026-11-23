"""直行命令链的组装（纯逻辑，不依赖 ROS、不依赖 rclpy）。

链路：命令输入 → 仲裁（校验/限速/过期/停车/重连）→ 编码 → 写入端。

本模块把仲裁器与写入端绑定成一个可测试对象；ROS 节点只是把消息转成
:meth:`StraightCommandChain.submit_nav` 等调用。**没有任何路径会打开或
写真实设备**：写入端要么是内存假串口，要么是测试脚本注入的伪终端。

写入节奏（``tick``）：每个周期最多写一帧。零速帧用于受控停止与心跳；
非零帧只在仲裁放行时写出。写失败立即通过 ``on_write_result(False)``
锁存，后续只可能输出零速直到操作者复位。
"""
from __future__ import annotations

from dataclasses import dataclass

from .arbiter import Arbiter, Decision, ZeroReason
from .ownership import endpoint_is_tainted
from .transport import MemoryTransport


@dataclass
class TickRecord:
    """一次写入周期的可观测结果（测试与日志用）。"""

    tick_index: int
    now: float
    frame: bytes | None
    is_zero: bool
    zero_reason: ZeroReason
    write_ok: bool


class StraightCommandChain:
    """仲裁器 + 写入端的组合对象。"""

    def __init__(self, arbiter: Arbiter, transport=None) -> None:
        self._arbiter = arbiter
        self._transport = transport if transport is not None else MemoryTransport()
        self._tick_index = 0
        self.records: list[TickRecord] = []

    # 输入转发（ROS 节点或测试调用）
    def submit_nav(self, vx: float, vy: float, vz: float, now: float) -> None:
        self._arbiter.on_nav_cmd(vx, vy, vz, now)

    def submit_demo(self, vx: float, vy: float, vz: float, now: float) -> None:
        self._arbiter.on_demo_cmd(vx, vy, vz, now)

    def submit_manual(self, vx: float, vy: float, vz: float, now: float) -> None:
        """网页手动命令（/manual_cmd_vel）。"""
        self._arbiter.on_manual_cmd(vx, vy, vz, now)

    def submit_auto(self, vx: float, vy: float, vz: float, now: float) -> None:
        """自动巡航命令（/auto_cmd_vel）。"""
        self._arbiter.on_auto_cmd(vx, vy, vz, now)

    def set_safety_permission(self, allowed: bool, reason: str, ttl_s: float,
                              now: float) -> None:
        """安全许可（/drive_safety）。转发给仲裁器的有效期门。"""
        self._arbiter.on_safety_permission(allowed, reason, ttl_s, now)

    def set_sources(self, nav_publishers: int, demo_publishers: int,
                    manual_publishers: int | None = None,
                    auto_publishers: int | None = None) -> None:
        self._arbiter.on_publisher_count(nav_publishers, demo_publishers,
                                         manual_publishers, auto_publishers)

    def set_stop(self, active: bool) -> None:
        self._arbiter.on_stop(active)

    def on_transport_reconnect(self) -> None:
        """由唯一串口所有者调用：串口重新打开。"""
        self._arbiter.on_serial_reconnect()
        reconnect = getattr(self._transport, 'reconnect', None)
        if callable(reconnect):
            reconnect()

    def operator_clear(self) -> None:
        """操作者显式复位（清除写失败锁存与重连禁非零）。"""
        self._arbiter.clear_write_fault(by_operator=True)

    def firmware_confirm(self, confirmed: bool) -> None:
        self._arbiter.on_firmware_confirm(confirmed)

    def _mark_clean_state_verified(self, *, simulated: bool) -> None:
        """仅构造明确的**模拟**干净接收端状态（必须 ``simulated=True``）。

        这不是真实固件确认，也不能替代接收端同步。若适配器已声明流被锁定，
        或端口状态不可判定，本方法不会放行——调用后仍可能静默。
        """
        if simulated is not True:
            raise ValueError('干净状态只能由测试以 simulated=True 构造')
        # 适配器显式声明锁定时不放行。没有 stream_locked 属性的适配器
        # 由端口层面的 endpoint_is_tainted 判定（None = 不可判定 = 按脏处理）。
        if getattr(self._transport, 'stream_locked', False):
            return
        # 包装适配器（如 BlockedNonzeroGuard）把端口存在内层：逐层解开再判定，
        # 否则会把包装对象本身当成端口，得到"不可判定"。
        layer = self._transport
        port = getattr(layer, '_port', None)
        for _ in range(8):
            if port is not None or not hasattr(layer, '_inner'):
                break
            layer = layer._inner
            port = getattr(layer, '_port', None)
        if port is None:
            port = layer
        if endpoint_is_tainted(port) is None:
            return
        self._arbiter.mark_receiver_clean(simulated=simulated)

    # 输出
    def tick(self, now: float) -> TickRecord:
        decision: Decision = self._arbiter.evaluate(now)
        write_ok = False
        if decision.frame is not None:
            try:
                write_ok = bool(self._transport.write(decision.frame))
            except Exception:   # noqa: BLE001——写入异常同样视为写失败
                write_ok = False
            # 传输层声明流已被切开（短写/写入异常）时，仲裁器须锁死整个流：
            # 之后连零速帧也不许写，避免残帧与新帧拼接。
            # 没有 stream_locked 属性的适配器：无法证明"字节没出去"，
            # 写失败一律按流毒处理（保守），不假设未发字节。
            poisoned = bool(getattr(self._transport, 'stream_locked', True))
            self._arbiter.on_write_result(write_ok,
                                          stream_poisoned=poisoned and not write_ok)
        rec = TickRecord(self._tick_index, now, decision.frame,
                         decision.is_zero, decision.zero_reason, write_ok)
        self._tick_index += 1
        self.records.append(rec)
        return rec

    @property
    def counts(self) -> dict:
        return dict(self._arbiter._counts)  # noqa: SLF001——诊断只读快照

    def nonzero_frames(self) -> list[bytes]:
        return [r.frame for r in self.records if r.frame is not None and not r.is_zero]
