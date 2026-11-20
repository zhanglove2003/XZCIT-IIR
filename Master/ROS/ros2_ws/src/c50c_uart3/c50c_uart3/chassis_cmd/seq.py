"""逐帧序号（v3 帧序号门）：唯一的序号状态归属对象。

固件契约（``chassis_ack.c`` 的 ``check_seq``，只读核对）：

- 速度帧 ``frame[2]`` 的 bit7 是能力位 ``CHASSIS_CAP_HOST_ACK``（0x80），
  未置位即 ``CHASSIS_RX_NO_SEQ`` 拒绝；
- bit0..6 是本会话内的逐帧序号，取值 0..127；
- 会话内第一帧接受任意序号，其后只接受"前进且未用过"的序号，
  前进窗口 ``CHASSIS_SEQ_WINDOW``（64），回退、重放、越窗都拒绝。

主机侧因此必须知道"本会话已用到第几号"。本对象是**唯一的**序号状态，
由 :class:`~c50c_uart3.chassis_cmd.arbiter.Arbiter` 与本次会话的授权状态
放在一起持有，并在会话变化/重连/复位时一并清零——固件在会话内没有
重新开始的入口，主机侧会话重置即对应固件侧的新会话。

**不复用同一序号**：序号随之递增，跨过一次会话后仍不回到 0 以外的
旧值——每次会话重置都从 :data:`SEQ_START` 开始，与"新会话首帧接受任意
序号"一致。
"""
from __future__ import annotations

SEQ_MASK = 0x7F                   # chassis_ack.h: CHASSIS_SEQ_MASK
CAP_HOST_ACK = 0x80               # chassis_ack.h: CHASSIS_CAP_HOST_ACK
SEQ_START = 0                     # 新会话第一帧取 0（固件首帧接受任意序号）
SEQ_MAX = SEQ_MASK                # 0..127 循环
SEQ_WINDOW = 64                   # chassis_ack.h: CHASSIS_SEQ_WINDOW


class FrameSeq:
    """本会话的逐帧序号。会话重置即 :meth:`reset`。"""

    __slots__ = ('_next', 'issued')

    def __init__(self) -> None:
        self._next = SEQ_START
        self.issued = 0

    def take(self) -> int:
        """取下一帧的序号 0..127，并前进。"""
        value = self._next
        self._next = (value + 1) & SEQ_MASK
        self.issued += 1
        return value

    def reset(self) -> None:
        """新会话从 :data:`SEQ_START` 重新开始（固件新会话首帧接受任意序号）。"""
        self._next = SEQ_START
        self.issued = 0

    @property
    def next_value(self) -> int:
        return self._next


__all__ = ['CAP_HOST_ACK', 'FrameSeq', 'SEQ_MASK', 'SEQ_MAX', 'SEQ_START',
           'SEQ_WINDOW']
