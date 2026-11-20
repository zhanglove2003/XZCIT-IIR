"""唯一串口所有者持有的 fd 适配器（不打开设备）。

谁用它：``uart3_node`` 自己持有的那个文件描述符。适配器只**借用**调用方
传入的 fd 做 ``os.write`` / ``os.read``，从不 ``os.open`` 设备。
因此本模块不会产生第二个串口打开者——这正是「单一所有者」的实现方式。

安全约束（沿用总任务书与第八轮结论）：

1. 默认不允许被真实 UART 使用：构造时必须显式给出 ``fd_authorised=True``，
   且设备路径必须属于真实 UART 白名单；未授权即抛
   :class:`RealFdUseForbidden`。
2. ``fd_authorised`` **不是接收确认**。它只表示"所有者允许使用这个 fd 写"，
   不表示固件已接收、解析器已同步。端口在``endpoint_is_tainted`` 里的
   状态仍是"不可判定"，需要所有者给出可验证的干净证据才会放行非零。
3. 部分写入视为帧被截断：返回实际字节数，由
   :class:`~c50c_uart3.chassis_cmd.ownership.OwnerSerialTransport` 判定失败
   并锁定整条流。
"""
from __future__ import annotations

import errno
import os
import re

# 真实 UART 的危险路径（只在显式授权时允许）。
REAL_UART_PATTERN = re.compile(
    r'^/dev/(ttyAMA\d+|ttyS\d+|ttyUSB\d+|ttyACM\d+|serial/by-id/[^/]+)$')
# 隔离伪终端。
PTS_PATTERN = re.compile(r'^/dev/pts/\d+$')


class RealFdUseForbidden(PermissionError):
    """未获显式授权就试图用真实 UART 的 fd 写入。"""


def is_real_uart(device: str) -> bool:
    return isinstance(device, str) and bool(REAL_UART_PATTERN.match(device))


def is_isolated_pty(device: str) -> bool:
    return isinstance(device, str) and bool(PTS_PATTERN.match(device))


class FdPort:
    """在调用方已打开的 fd 上读写。不拥有 fd，不负责关闭它。

    参数
    ----
    fd : 所有者已打开的整数文件描述符（读循环与写路径共用同一个 fd）。
    device : 设备路径，只用于授权判定与诊断。
    fd_authorised : 必须显式为 ``True`` 才允许写真实 UART。
    """

    # 不用 __slots__：会话握手要在端口对象上挂接收方向收据，`__slots__`
    # 类无法 setattr，收据挂不上就只能停在 RECEIVER_UNKNOWN。
    kind = 'owner-fd'
    #: 是否为真实串口端口。写路径只认 True（FakeFdPort 为 False），
    #: 让"决定写零速"与"真的写进串口"在记录里可区分。
    is_real = True

    def __init__(self, fd: int, device: str, *,
                 fd_authorised: bool = False) -> None:
        if not isinstance(fd, int) or fd < 0:
            raise TypeError('fd 必须是非负整数')
        if is_real_uart(device) and fd_authorised is not True:
            raise RealFdUseForbidden(
                f'{device!r} 是真实 UART；未获显式 fd_authorised=True 授权，'
                '不得用该 fd 写入（本轮授权不含真实串口写）')
        self.fd = fd
        self.device = device
        # 授权标志留档：运行时写入门（guard_bridge.port_is_writable）每拍复核
        self.fd_authorised = fd_authorised is True
        self.writes = 0
        self.bytes_written = 0
        self.refused = 0

    # ------------------------------------------------------------ 端口契约
    def write_bytes(self, data: bytes) -> int:
        """向已打开的 fd 写入。返回实际写入字节数（可能小于请求长度）。"""
        try:
            written = os.write(self.fd, data)
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                # 非阻塞 fd 不可写：按 0 字节处理，由上层判定失败。
                return 0
            raise
        self.writes += 1
        self.bytes_written += written
        return written

    def read_some(self, max_bytes: int) -> bytes:
        try:
            return os.read(self.fd, max_bytes)
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return b''
            raise

    def reopen(self) -> None:
        """所有者重开设备后调用。本对象只记录事实；不解除任何流锁。"""
        return None

    # ------------------------------------------------------------ 诊断
    def snapshot(self) -> dict:
        return {
            'device': self.device,
            'fd': self.fd,
            'writes': self.writes,
            'bytes_written': self.bytes_written,
        }


class FakeFdPort(FdPort):
    """测试替身：不触碰真实系统调用，把字节留在内存里。"""

    kind = 'fake-fd'
    is_real = False

    def __init__(self, device: str = '/dev/pts/0') -> None:
        # 借用一条假 fd 值；从不真正 write(2)。
        self.stream = bytearray()
        self.fd = -1
        self.device = device
        self.writes = 0
        self.bytes_written = 0
        self.refused = 0

    def write_bytes(self, data: bytes) -> int:
        self.writes += 1
        self.stream.extend(data)
        self.bytes_written += len(data)
        return len(data)

    def read_some(self, max_bytes: int) -> bytes:
        return b''


__all__ = [
    'FdPort',
    'FakeFdPort',
    'RealFdUseForbidden',
    'is_isolated_pty',
    'is_real_uart',
]
