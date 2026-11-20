"""直行会话握手：把固件会话帧与本机确认帧接到命令链上。

目的（上机前返工 B）：A 的候选固件在**没有有效会话**时丢弃全部 USART3
速度帧（``ChassisRxResult.NO_SESSION``），而"wirte 返回成功"只说明字节
出了本进程。因此链路需要一条**接收方向**的证据：只有真正在线上读到固件
的会话帧（``frame[1]==0x7F``），才允许把非零放行。

本握手只做四件事，不打开设备、不改变任何既有安全门：

1. 从所有者已读到的字节里提取会话帧（:class:`SessionExtractor`）；
2. 记录观测时刻，交 :meth:`Arbiter.on_session_observed` 作会话时效门；
3. 生成主机确认帧，交所有者用**同一个 fd** 写出（不新增打开者）；
4. 确认帧写失败时通知仲裁器保持禁非零（不因"尝试过"而放行）。

第十二轮补充：真的读到会话帧（``stats.session_frames_seen > 0``）之后，
本握手用私有封印构造一张 :class:`~.sim_receipt.SimSessionReceipt`，交给
所有者挂到端口对象上。这张收据是**接收方向证据的唯一生产来源**：桥只
认它的类型，不接受任何自报布尔值。收据在每次读到会话帧时重新签发，
重连（本对象被重建）后旧收据随端口一起作废。

本轮仍未实现的部分（写进交付）：A 的固件**不会**回送"确认已收到"的
任何字段，所以"确认帧被接受"无法由主机证明。本握手只证明"会话帧确实
从线上来了"，不证明"固件接受了本主机的确认"。
"""
from __future__ import annotations

from dataclasses import dataclass

from .bridge import SESSION_RECEIPT_ATTR
from .ownership import _remember_clean
from .session import (
    PROTO_VERSION,
    SessionExtractor,
    SessionObservation,
    encode_session_ack,
    parse_session_frame,
)
from .sim_receipt import SIM_SESSION_SEAL, SimSessionReceipt

#: A 的固件每 1 s 发一帧会话帧（``data_task.c``）。本握手在此基础上只做
#: "收到即记录"，不主动探测时延。
SESSION_ACK_REPEAT_S = 1.0


@dataclass
class HandshakeStats:
    session_frames_seen: int = 0
    acks_sent: int = 0
    ack_write_failures: int = 0
    bytes_received: int = 0
    receipt_attach_failures: int = 0

    def snapshot(self) -> dict:
        return {
            'session_frames_seen': self.session_frames_seen,
            'acks_sent': self.acks_sent,
            'ack_write_failures': self.ack_write_failures,
            'bytes_received': self.bytes_received,
            'receipt_attach_failures': self.receipt_attach_failures,
        }


class SessionHandshake:
    """会话握手（纯逻辑，写入动作由调用方注入的 writer 完成）。

    参数
    ----
    arbiter : 需要接收会话观测的仲裁器。
    clock : 单调时钟（秒），默认沿用调用方传入，不自行取系统时间。
    """

    def __init__(self, arbiter, *, clock, receipt_sink=None) -> None:
        self._arbiter = arbiter
        self._clock = clock
        self._extractor = SessionExtractor()
        self.stats = HandshakeStats()
        self._last_ack_at: float | None = None
        self._acked_session: int | None = None
        # 收据去处：通常为端口对象；None 表示不签发（既有测试的用法）。
        self._receipt_sink = receipt_sink

    # ------------------------------------------------------------ 接收方向
    def feed(self, data: bytes, *, write=None) -> int | None:
        """处理所有者读到的一批字节，返回本次新确认的会话 ID（无则 None）。

        ``write`` 是可调用对象（通常为端口的 ``write_bytes``），只在**首次**
        见到某个会话 ID 时调用一次，用来回填确认帧。``write`` 返回写入
        字节数；不足一帧视为失败，仲裁器保持禁非零。
        """
        if not data:
            return None
        self.stats.bytes_received += len(data)
        ids = self._extractor.feed(data)
        if not ids:
            return None
        session_id = ids[-1]
        self.stats.session_frames_seen += len(ids)
        now = float(self._clock())
        self._issue_receipt(session_id, now)
        if session_id != self._acked_session:
            # 先记录会话（仍禁非零），回填确认成功后才放行。
            self._arbiter.on_session_observed(session_id, now, accepted=False)
            self._acked_session = session_id
            self._last_ack_at = None
        if write is None:
            return None
        if (self._last_ack_at is not None
                and (now - self._last_ack_at) < SESSION_ACK_REPEAT_S):
            return None
        frame = encode_session_ack(session_id)
        try:
            written = write(frame)
        except Exception:       # noqa: BLE001——写入异常一律按失败处理
            written = 0
        self._last_ack_at = now
        if isinstance(written, int) and written == len(frame):
            self.stats.acks_sent += 1
            self._arbiter.on_session_observed(session_id, now, accepted=True)
            return session_id
        self.stats.ack_write_failures += 1
        self._arbiter.on_session_ack_failed()
        return None

    def _issue_receipt(self, session_id: int, now: float) -> None:
        """用私有封印签发接收方向收据，挂到所有者给的端口对象上。

        这是本模块**唯一**的收据构造点：构造条件只有"真的在线上读到会话
        帧"。挂不上（``__slots__`` 端口）时记录一条失败计数，不放宽任何门。
        """
        sink = self._receipt_sink
        if sink is None:
            return
        receipt = SimSessionReceipt('session-handshake', session_id, now,
                                    frames_seen=self.stats.session_frames_seen,
                                    _seal=SIM_SESSION_SEAL)
        try:
            setattr(sink, SESSION_RECEIPT_ATTR, receipt)
        except (AttributeError, TypeError):
            self.stats.receipt_attach_failures += 1
            return
        # 接收端已在线上证明对齐（会话帧通过头/尾/BCC/版本校验）：按
        # ``ownership`` 的接收端登记声明干净。``endpoint_is_tainted`` 的污染
        # 记录优先，因此曾短写过/异常过的端口仍然是脏的，这里的声明无效。
        _remember_clean(sink)

    # ------------------------------------------------------------ 诊断
    @property
    def observation(self) -> SessionObservation:
        return SessionObservation(
            session_id=self._arbiter.session_id,
            observed_at=self._arbiter.session_observed_at,
            frames_seen=self.stats.session_frames_seen,
            acks_sent=self.stats.acks_sent,
        )

    def snapshot(self) -> dict:
        data = self.stats.snapshot()
        data.update(self.observation.describe())
        data['expected_proto_version'] = PROTO_VERSION
        return data


def session_frame_for(session_id: int) -> bytes:
    """构造一条与固件同格式的会话帧（仅供测试替身使用）。"""
    from . import FRAME_HEADER, FRAME_LENGTH, FRAME_TAIL, check_bcc
    frame = bytearray(FRAME_LENGTH)
    frame[0] = FRAME_HEADER
    frame[1] = 0x7F
    frame[2] = 0x01
    frame[3] = PROTO_VERSION
    for i in range(4):
        frame[4 + i] = (session_id >> (8 * (3 - i))) & 0xFF
    frame[8] = 0
    frame[9] = check_bcc(frame[:9])
    frame[10] = FRAME_TAIL
    return bytes(frame)


__all__ = [
    'HandshakeStats', 'SESSION_ACK_REPEAT_S', 'SessionHandshake',
    'session_frame_for',
]
