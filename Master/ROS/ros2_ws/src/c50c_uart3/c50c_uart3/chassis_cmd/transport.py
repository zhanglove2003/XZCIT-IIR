"""串口写入端：**默认拒绝任何真实设备**。

安全约束（任务书 B 公共边界第 1、3 条）：

- 本轮不打开真实串口，不向任何真实设备写数据；
- 不允许第二个进程另行打开同一 UART；串口只能有一个所有者。

因此本模块的正常实现只有一个：:class:`MemoryTransport`（内存假串口），
它记录写入字节供测试断言，不做任何 I/O。真实写入端
:class:`SerialOwnerTransport` 只作为**未启用的占位设计**存在：构造即
抛出 :class:`RealDeviceAccessForbidden`，并说明以后如何交给唯一串口
所有者（见 :class:`SerialOwnership`）。

所有写入都必须经过 :func:`assert_isolated_target` 校验：路径不在隔离
白名单内即拒绝。白名单只接受 ``/dev/pts/N``（伪终端）。伪终端本身就是
隔离目标；真实 UART（``/dev/ttyUSB*``、``/dev/ttyAMA*``、``/dev/serial/*``）
一律拒绝，无论调用者是否是测试。
"""
from __future__ import annotations

import os
import re

# 只允许伪终端：/dev/pts/<N>。其它路径（含 /dev/tty*, /dev/serial/*）一律拒绝。
PTS_PATTERN = re.compile(r'^/dev/pts/\d+$')


class RealDeviceAccessForbidden(RuntimeError):
    """试图访问真实设备时抛出（不是警告——构造即失败）。"""


def assert_isolated_target(path: str) -> None:
    """校验写入目标是否为隔离伪终端。不是就抛异常，绝不返回警告。"""
    if not isinstance(path, str) or not PTS_PATTERN.match(path):
        raise RealDeviceAccessForbidden(
            f'{path!r} 不是隔离伪终端（只允许 /dev/pts/N）。'
            '本候选不得打开真实串口或向真实设备写数据。')


class SerialOwnership:
    """唯一串口所有者的**设计说明**（不打开任何设备）。

    现状（只读核对）：

    - 串口所有者：``c50c_uart3`` 的 ``uart3_node`` 用
      ``os.open(O_RDWR | O_NOCTTY | O_NONBLOCK)`` + ``TIOCEXCL`` 打开
      UART3（``drive_enable`` 关闭时仍为 ``O_RDONLY``，不写任何字节）。
      ``TIOCEXCL`` 只阻止**之后**的新打开，不能排除先打开者。
    - 写入方：节点自己，在同一个 fd 上写。
    - 历史架构建议与已审核方案都要求**单一串口所有者**；不允许新桥
      另开同一设备（方案 §3.2.1 第 2 条）。

    本候选的选择：**扩展现有读取方**，不新增第二个设备打开者。

    具体做法（返工轮已实现）：

    1. ``uart3_node`` 用 ``drive_enable`` 选择打开模式：关闭时 ``O_RDONLY``，
       开启时 ``O_RDWR``；写入一律借用**同一个 fd**（``FdPort``），
       读循环与写路径共用一个文件描述符。
       ``TIOCEXCL`` 继续阻止其他进程随后打开。
    2. 仲裁器（:mod:`c50c_uart3.chassis_cmd.arbiter`）运行在节点内，
       写节拍由节点定时器驱动，不是第二个进程。
    3. 写失败与重连事件直接复用节点的 ``_on_serial_reconnected`` 钩子：
       重连 = 调用 ``Arbiter.on_serial_reconnect()``。
    4. 备选方案（需审核）：独立的 ``chassis_cmd_bridge`` 节点，由它成为
       唯一所有者（读、写都归它），``uart3_node`` 退化为纯话题消费者。
       在审核通过前，不允许两个进程同时持有 UART3。

    两条路都必须满足：一个设备只被一个进程打开；写入路径与读取路径
    共享同一 fd 或同一所有者的进程内队列。
    """

    def __init__(self, device: str) -> None:
        # 构造只做路径校验，不打开设备
        assert_isolated_target(device)
        self.device = device


class MemoryTransport:
    """内存假串口：记录写入字节，不做任何 I/O。测试与默认候选使用。"""

    kind = 'memory'

    def __init__(self) -> None:
        self.writes: list[bytes] = []
        self.fail_next = 0        # 让接下来 N 次写入失败（模拟写失败）
        self.closed = False

    def write(self, data: bytes) -> bool:
        if self.closed:
            raise RuntimeError('写入已关闭的假串口')
        if self.fail_next > 0:
            self.fail_next -= 1
            return False
        self.writes.append(bytes(data))
        return True

    def reconnect(self) -> None:
        """模拟串口重新打开：保留已写记录，模拟器本身不断开连接概念。"""
        self.closed = False


class SerialOwnerTransport:
    """真实写入端占位：**构造即拒绝**。

    保留它是为了让"以后怎么接唯一串口所有者"在代码里显式可见，而不是
    藏在文档里。真正的实现必须回到 :class:`SerialOwnership` 两个方案之一，
    并经审核；在此之前任何人调用都会立即失败。
    """

    kind = 'real-forbidden'

    def __init__(self, device: str) -> None:
        raise RealDeviceAccessForbidden(
            '本轮禁止打开真实串口。真实写入端必须由唯一串口所有者'
            f'（uart3_node 内部或独立桥节点）实现，设备 {device!r} '
            '不在本候选的可用范围内。')


def open_transport(device: str) -> MemoryTransport:
    """按设备路径选择写入端。

    - 空字符串、``memory://`` 或以 ``mem:`` 开头 → 内存假串口；
    - ``/dev/pts/N`` → 也返回内存假串口之外的**独立伪终端**实现留给
      测试脚本；本模块不直接打开，交给调用方注入；
    - 其它任何路径（真实设备）→ 拒绝。

    本函数永远不会打开真实设备；返回值只有 :class:`MemoryTransport`。
    """
    if device in ('', 'memory://') or device.startswith('mem:'):
        return MemoryTransport()
    if PTS_PATTERN.match(device):
        # 伪终端由测试脚本自己创建并注入到 write callable；这里不做打开。
        return MemoryTransport()
    raise RealDeviceAccessForbidden(
        f'设备 {device!r} 不在隔离白名单（只允许内存假串口与 /dev/pts/N）；'
        '本候选不得打开真实串口。')


def is_pts(path: str) -> bool:
    return bool(PTS_PATTERN.match(path)) and os.path.exists(path)
