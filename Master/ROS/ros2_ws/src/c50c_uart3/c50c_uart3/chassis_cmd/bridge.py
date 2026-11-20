"""直行命令桥：把 ROS 输入、仲裁器与**所有者提供的** fd 串成一个节点内组件。

本模块负责接线和设置仲裁器限速，不修改其安全判定逻辑：

- 输入来自 :class:`~c50c_uart3.chassis_cmd.ros_input.RosStraightInput`
  （默认关闭）；
- 判定全部由 :class:`~c50c_uart3.chassis_cmd.arbiter.Arbiter` 完成；
- 写入经 :class:`~c50c_uart3.chassis_cmd.ownership.OwnerSerialTransport`，
  该适配器**不自带 fd**，只转发给所有者传入的端口对象。

关键性质（与总任务书 B 项 2、3 对齐）：

1. 本模块不调用 ``os.open``，因此不会成为第二个串口打开者。
2. 本模块不调用任何"模拟干净/模拟固件确认"入口。默认状态下链路停在
   ``RECEIVER_UNKNOWN``，一个字节也不写。
3. 串口重连只作废缓存命令，**不**附带任何确认；重连后非零仍然关闭。
4. 失效静默不等于车辆已停车：静默后固件是否超时停车取决于固件自身监督，
   本模块不能证明。
5. 生产放行改用**接收方向证据**（第十二轮）：本模块按仲裁器已有的会话
   证据判定本拍是否驱动命令链，并用端口上的会话收据放行
   ``_receiver_clean``。两者都只能由 :meth:`SessionHandshake.feed` 在真的
   读到线上会话帧之后产生；本模块自己**不能**制造会话证据，也不接受
   自报的布尔值。测试收据 ``SimCleanReceipt`` 路径保留给既有测试。
"""
from __future__ import annotations

import time

from .arbiter import PROTOCOL_BASELINE
from .chain import StraightCommandChain
from .ownership import OwnerSerialTransport
from .ros_input import RosStraightInput

#: 默认方言：保持不变（v3），基线只作为显式开关。
DEFAULT_FIRMWARE_PROTOCOL = 'v3'

DEFAULT_TICK_PERIOD_S = 0.02
#: 端口对象上存放会话收据的属性名。收据由 ``SessionHandshake`` 在真的读到
#: 线上会话帧之后签发；本模块只读取，不构造。
SESSION_RECEIPT_ATTR = '_c50c_session_receipt'


class StraightDriveBridge:
    """命令链 + ROS 输入 + 所有者端口的一体化接线。

    参数
    ----
    port : 所有者持有的端口对象（需提供 ``write_bytes`` 等契约）。
    device : 设备路径，只用于诊断与隔离校验。
    enabled : 传入 :class:`RosStraightInput` 的初始开关；默认 ``False``。
    tick_period_s : 周期驱动间隔（秒）。
    clock : 单调时钟函数，默认 :func:`time.monotonic`。
    """

    def __init__(self, port, *, device: str | None = None,
                 enabled: bool = False,
                 tick_period_s: float = DEFAULT_TICK_PERIOD_S,
                 clock=time.monotonic,
                 firmware_protocol: str = DEFAULT_FIRMWARE_PROTOCOL) -> None:
        if tick_period_s <= 0:
            raise ValueError('周期必须为正')
        if tick_period_s > 0.1:
            raise ValueError('周期必须小于固件监督超时（约 1 s）的安全上界')
        self.firmware_protocol = firmware_protocol
        self.transport = OwnerSerialTransport(port, device)
        self.chain = StraightCommandChain(_new_arbiter(firmware_protocol),
                                          self.transport)
        self.input = RosStraightInput(self.chain, enabled=enabled, clock=clock,
                                      nonzero_gate=self._allow_nonzero)
        # 会话放行门。两个条件同时成立才允许写非零：
        #   1) 仲裁器侧有可验证的会话证据：``session_id`` 已由
        #      SessionHandshake 从线上会话帧写入，且会话窗口未过期；
        #   2) 同一会话内本桥已经写出过一帧成功的零速帧。
        # ``_session_open`` 只保留给测试收据入口（_open_session）；生产路径
        # 不依赖它，本模块自己不能制造会话证据。
        self._session_open = False
        self._session_zero_written = False
        self.period = float(tick_period_s)
        self.clock = clock
        self.ticks = 0
        # 订阅回调计数：门控全开却一直 ALL_SOURCES_ZERO 时，用它区分
        # "消息没到桥"与"消息到了但缓存被清"。
        self.nav_msgs = 0
        self.last_msg_vx = 0.0
        self.last_msg_vz = 0.0
        self.last_allow = None

    def count_nav_msg(self, _msg=None) -> None:
        """订阅回调旁路计数（只计数，不改变仲裁行为）。"""
        self.nav_msgs += 1
        self.last_msg_vx = float(getattr(getattr(_msg, 'linear', None), 'x', 0.0))
        self.last_msg_vz = float(getattr(getattr(_msg, 'angular', None), 'z', 0.0))
        self.last_allow = self._allow_nonzero()

    # ------------------------------------------------------------ 周期驱动
    def cycle(self):
        """一拍：先会话门，再来源门控，最后仲裁与写入。返回 ``(gated, record)``。

        本拍是否驱动命令链由 :meth:`session_evidence` 决定。没有接收方向
        证据时只做"不发字节"的静默拍：推进本桥计数，不调用
        :meth:`StraightCommandChain.tick`，因此也不产生任何写入。
        """
        if not self._evidence_gate_open():
            self.ticks += 1
            return (True, None)
        self._arm_receiver_from_evidence()
        gated, record = self.input.on_cycle(lambda: self.chain.tick(self.clock()))
        self.ticks += 1
        if record is not None and record.write_ok and record.is_zero:
            self._session_zero_written = True
            if (self.firmware_protocol == PROTOCOL_BASELINE
                    and not self.chain._arbiter._rearmed):   # noqa: SLF001
                # 基线固件：首帧零速已写出，且接收方向状态帧仍在时效内。
                # 基线不回复确认，"写成功"是唯一可得证据；它证明不了固件
                # 接受，只证明字节离开了进程（见交付说明边界）。
                if self.baseline_evidence():
                    self.chain._arbiter.on_baseline_arm(self.clock())  # noqa: SLF001
        return gated, record

    def _evidence_gate_open(self) -> bool:
        """本拍是否允许驱动命令链（只读判定，不制造证据）。"""
        if self._session_open:
            return True
        if self.firmware_protocol == PROTOCOL_BASELINE:
            return self.baseline_evidence()
        return self.session_evidence()

    def baseline_evidence(self) -> bool:
        """基线固件：线上最近一帧合法状态帧是否仍在时效内。"""
        return self.chain._arbiter.baseline_window_open(self.clock())  # noqa: SLF001

    def notify_status_frame(self, frame: bytes | None = None) -> None:
        """总线读取侧调用：刚刚解出一帧合法基线状态帧。

        这是基线固件下唯一的接收方向证据来源。协议不是基线时抛错。

        两处状态必须同时更新，否则链路永远不出字节：

        - 仲裁器（``on_baseline_status_frame``）负责放行判定；
        - 传输层（``note_baseline_status_frame``）负责解锁字节流。
          ``OwnerSerialTransport`` 默认锁定，唯一解锁入口是接收方向证据；
          基线的证据是 48 字节状态帧，与 v3 的 11 字节会话帧分属不同入口，
          ``note_firmware_status_frame`` 不认这帧（会回 ``None`` 而拒绝）。

        ``frame`` 可选：给字节时传输层会用同一解析器复核（唯一能自证的
        形态）；不给时按"调用方已校验"处理。生产节点
        （``uart3_node.py`` 的 ``_notify_drive_status_frame``）本轮冻结，
        仍走无参形态。

        更新顺序：非基线方言交给仲裁器抛错（传输层不被改动）；基线方言下
        先让传输层判定（字节校验、残帧锁存），**只有**它接受时才把证据记到
        仲裁器。反过来的顺序会让被锁存的适配器在日志里同时出现"已接受"和
        "已忽略"两句互相矛盾的话——状态没写错，但证据链会说谎。
        """
        if self.firmware_protocol != PROTOCOL_BASELINE:
            # 仲裁器在第一步即抛 RuntimeError，两层状态都不被改动。
            self.chain._arbiter.on_baseline_status_frame(self.clock())  # noqa: SLF001
            return
        if self.transport.note_baseline_status_frame(frame):
            self.chain._arbiter.on_baseline_status_frame(self.clock())  # noqa: SLF001

    def session_evidence(self) -> bool:
        """仲裁器侧是否已有未过期的会话证据（只读判定，不制造证据）。"""
        arbiter = self.chain._arbiter                    # noqa: SLF001
        if arbiter.session_id is None:
            return False
        return bool(arbiter.session_window_open(self.clock()))

    def _session_receipt(self):
        """端口链上的会话收据；没有则 ``None``。"""
        return _find_session_receipt(self.transport)

    def _arm_receiver_from_evidence(self) -> None:
        """用端口上的接收方向收据放行 ``_receiver_clean``（不发字节）。

        收据只能由 :class:`SessionHandshake` 在真的读到会话帧之后签发；
        本方法只把它转交命令链，命令链仍会按端口污染状态否决。没有收据
        时不做任何事：链路保持 ``RECEIVER_UNKNOWN``。
        """
        from .sim_receipt import require_session_receipt
        receipt = self._session_receipt()
        if receipt is None:
            return
        try:
            require_session_receipt(receipt)
        except TypeError:
            return
        self.chain._mark_clean_state_verified(simulated=True)   # noqa: SLF001

    def _open_session(self, receipt) -> None:
        """测试设施入口：用收据打开会话，并把固件放行条件显式置为满足。

        本方法不是生产入口：它在生产路径没有调用点，且默认拒绝任何非
        :class:`sim_confirm.SimCleanReceipt` 的收据。本方法**不是**固件
        接收确认——它只表示"测试构造了干净的接收端"。
        """
        from .sim_receipt import require_receipt
        require_receipt(receipt)
        self.chain._mark_clean_state_verified(simulated=True)   # noqa: SLF001
        # 固件契约：未授权时丢弃首个非零帧。测试构造的干净接收端给了
        # "授权有效"这一前提；本桥仍必须先发一帧零速，见 cycle()。
        self.chain._arbiter.on_firmware_confirm(True)           # noqa: SLF001
        self._session_open = True
        self._session_zero_written = False

    def open_session(self, receipt) -> None:
        """打开会话（公开名）。仍需测试收据，见 :meth:`_open_session`。"""
        self._open_session(receipt)

    def _allow_nonzero(self) -> bool:
        """输入层是否允许把来源命令标为"有输出许可"。

        按方言分派：

        - 基线固件没有会话帧、没有确认位，``_session_zero_written`` 永远不可能
          由会话建立，因此不能用它当放行条件。基线只用同一桥已有的
          :meth:`baseline_evidence` 门；首帧零速规则由仲裁器的
          ``_baseline_nonzero_block`` / ``_sent_zero_since_arm`` 统一执行，
          本方法不重复承担，也不会让状态帧自身变成放行口子。
        - v3 方言行为保持不变，仍要求 ``_session_zero_written``。
        """
        if self.firmware_protocol == PROTOCOL_BASELINE:
            return self.baseline_evidence()
        return self._evidence_gate_open() and self._session_zero_written

    # ------------------------------------------------------------ 生命周期
    def on_serial_reconnected(self) -> None:
        """所有者重新打开串口后调用：清缓存并禁非零，**不带确认**。"""
        self.input.on_serial_reconnect()

    def snapshot(self) -> dict:
        return {
            'ticks': self.ticks,
            'period_s': self.period,
            'input': self.input.snapshot(),
            'transport': {
                'kind': self.transport.kind,
                'stream_locked': self.transport.stream_locked,
                'stream_lock_reason': self.transport.stream_lock_reason,
                'stats': self.transport.stats.__dict__,
            },
            'receiver_clean': self.chain._arbiter.receiver_clean,   # noqa: SLF001
            'session_evidence': self.session_evidence(),
            'session_open_by_receipt': self._session_open,
            'session_id': self.chain._arbiter.session_id,           # noqa: SLF001
        }


def _find_session_receipt(transport):
    """沿 ``_inner`` 注册链找到端口对象上由握手签发的会话收据。

    找不到（端口不挂收据、或收据已被新一轮重连作废）时返回 ``None``。
    """
    layers = transport
    for _ in range(8):
        receipt = getattr(layers, SESSION_RECEIPT_ATTR, None)
        if receipt is not None:
            return receipt
        # OwnerSerialTransport 用 ``_port``，命令链的包装层用 ``_inner``。
        layers = getattr(layers, '_inner', None) or getattr(layers, '_port', None)
        if layers is None:
            return None
    return None


def _new_arbiter(firmware_protocol: str = DEFAULT_FIRMWARE_PROTOCOL):
    from .arbiter import Arbiter
    if firmware_protocol == PROTOCOL_BASELINE:
        # 2026-11-14 用户确认：速度策略由网页挡位定义——空档 0 / 巡航挡
        # 0.4 / 运动挡 0.8。上限钉在**最高挡**：挡位之外的速度一律拒绝
        # （转向同样取挡位值，故角速度上限一致）。旧的 1.0/1.3 直行测试
        # 上限撤销——它会放行超过运动挡的速度。
        return Arbiter(firmware_protocol=firmware_protocol, max_linear_mps=0.8,
                       max_forward_mps=0.8, max_angular_rad_s=0.8)
    return Arbiter(firmware_protocol=firmware_protocol)


__all__ = ['StraightDriveBridge', 'DEFAULT_FIRMWARE_PROTOCOL',
           'DEFAULT_TICK_PERIOD_S', 'SESSION_RECEIPT_ATTR']
