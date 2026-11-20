"""ROS 直行输入适配层（默认关闭，无 ROS 导入依赖）。

职责单一：把 ROS 侧的三类消息（导航命令、演示命令、监督停车、发布者数量）
转成 :class:`c50c_uart3.chassis_cmd.chain.StraightCommandChain` 的方法调用。
**不编码帧、不打开串口、不选择写入端**——那些全部由既有链路负责。

设计要点（任务书第七轮 C 第 1、2 条）：

1. **无 ROS 依赖**：本模块只接受"鸭子类型"的数值对象（含 ``linear``、
   ``angular`` 属性，或 ``data`` 布尔量）。ROS 节点负责把
   ``geometry_msgs/Twist`` 与 ``std_msgs/Bool`` 传进来即可，测试用轻量
   替身对象（见 ``test_ros_input.py`` 的 ``TwistLike`` / ``BoolLike``）。
   因此适配层测试不需要 ROS 2 环境。
2. **默认关闭**：``RosStraightInput(enabled=False)`` 构造后所有 ``on_*``
   入口直接忽略输入，命令链拿不到任何命令。开启需显式 ``enabled=True``。
3. **不绕过仲裁**：仅调用 ``submit_nav`` / ``submit_demo`` /
   ``set_sources`` / ``set_stop``。限速、过期、停车优先、来源冲突、重连
   禁非零全部仍由 :class:`~c50c_uart3.chassis_cmd.arbiter.Arbiter` 判定。
4. **不伪装固件确认**：本模块不提供、也不转调任何"注入干净状态"或
   "标记接收端已确认"的入口（连字符会触发生产模块的静态检查，此处
   不写具体名字）。ROS 层收到消息与固件确认是两件事；前者不能推导
   后者（见 ``RosInputState`` 里 ``received_*`` 与 ``firmware_confirmed``
   分离计数）。
5. **时间用单调时钟**：命令的 ``now`` 取适配层注入的 ``clock``（默认
   ``time.monotonic``），不取 ROS 消息时间戳——消息戳可被
   ``use_sim_time`` 影响，会让过期判定失效。

已知限制（交付中同样写明）：

- 真实接收确认无协议手段：本适配层只能观察到"ROS 消息到达"，无法证明
  固件收到并接受。因此 ``enabled=True`` 且经由真实节点启动时，链路仍会
  停在 ``RECEIVER_UNKNOWN`` 的静默状态，不输出任何字节。
- 话题名沿用已审核方案的约定（``/nav_cmd_vel``、``/demo_cmd_vel``、
  ``~/nav_stop``），见 ``c50c-nav-wiring-plan.md`` §3.2.1。真实 Nav2
  配置与本仓现有 launch 尚未接入，接入前需复核。
- 2026-11-14 新增三个输入（任务书 T1/T2）：``/manual_cmd_vel``（网页手动）、
  ``/auto_cmd_vel``（自动巡航）、``/drive_safety``（安全许可，String JSON，
  来自 fusion_node，带接收端可判的有效期）。``/nav_cmd_vel`` 保留给外部
  程序，语义为手动遥控，发布者上限 1。

第八轮修复（审核复现的两处门控缺口）：

- **禁用不残留命令**：``disable()`` 立即向命令链提交一次零速命令
  （``submit_nav(0,0,0)``），并作废本条命令的"可输出许可"。重新启用
  不会恢复禁用前的任何值；``enable()`` 先提交零速，再等**启用之后**
  到达的新命令。旧值即使还留在仲裁器缓存里，也因为许可作废而无法输出。
- **发布者数量未知不放行**：适配层的发布者计数有"未知"状态
  （``publishers_known=False``）。未知、为 0、或从 1 降到 0 时一律禁
  非零；来源重新确认后也不复活先前命令（该命令的许可在该段时间已作废）。
- 新增 :class:`RosSubscriptionInput`：把已有回调层接到 rclpy 订阅与周期
  驱动的候选接线（默认关闭）。**不新增任何串口打开点**，不使用任何模拟
  确认入口；真实 rclpy 运行未验证。

ROS 图事件时序边界：rclpy 的订阅回调与图事件回调都在执行器线程里串行
执行，所以"命令到达"与"发布者数变化"不会并发。但一次发布者变化事件
可能晚于该发布者已经发出的命令——适配层对此保持保守：发布者数变化时
作废已有命令的许可，下一次非零必须来自变化之后到达的命令。
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any

DEFAULT_NAV_TOPIC = 'nav_cmd_vel'
DEFAULT_DEMO_TOPIC = 'demo_cmd_vel'
DEFAULT_STOP_TOPIC = 'nav_stop'
DEFAULT_MANUAL_TOPIC = 'manual_cmd_vel'
DEFAULT_AUTO_TOPIC = 'auto_cmd_vel'
DEFAULT_SAFETY_TOPIC = 'drive_safety'

# 允许的最大发布者数：超过即视为来源不明（仲裁器据此禁非零）。
# 2026-11-14 起网页手动与自动巡航分离到 /manual_cmd_vel、/auto_cmd_vel
#（各限 1 个发布者），/nav_cmd_vel 只保留给外部程序（手动遥控语义），
# 上限从 2 收敛为 1——原 2 个常驻发布者互相覆盖、许可被反复撤销的
# 不一致由此消除：放行条件与撤销条件共用同一组上限。
MAX_TRACKED_PUBLISHERS = 1

# 安全许可 JSON 的 schema 版本（与 fusion_node.safety_state 一致）。
SAFETY_PERMISSION_SCHEMA = 1


class RosInputDisabled(RuntimeError):
    """禁用状态下尝试直接写入命令链时抛出（防御性，正常路径不触发）。"""


@dataclass
class RosInputState:
    """适配层自身的可观测状态（与仲裁器状态分开，便于区分两层）。"""

    enabled: bool = False
    received_nav: int = 0          # 收到并已转发的导航消息数
    received_demo: int = 0
    received_stop: int = 0
    received_manual: int = 0       # 网页手动（/manual_cmd_vel）
    received_auto: int = 0         # 自动巡航（/auto_cmd_vel）
    received_safety: int = 0       # 安全许可（/drive_safety）
    safety_malformed: int = 0      # 无法解析的许可消息数
    dropped_while_disabled: int = 0
    rejected_malformed: int = 0    # 缺字段或非数值的消息
    stop_active: bool = False
    # 发布者计数：``publishers_known=False`` 表示还没拿到 ROS 图证据，
    # 此时计数不构成"来源唯一"的证据，非零一律不放行（第八轮修复）。
    publishers_known: bool = False
    nav_publishers: int = 0
    demo_publishers: int = 0
    manual_publishers: int = 0
    auto_publishers: int = 0
    # **固件确认**：本适配层从不写入。保留字段只为让"ROS 收到 ≠ 固件确认"
    # 在状态对象里显式可见，任何值为 False 都是实际状态。
    firmware_confirmed: bool = False
    last_nav_monotonic: float | None = None

    def snapshot(self) -> dict:
        return {
            'enabled': self.enabled,
            'received_nav': self.received_nav,
            'received_demo': self.received_demo,
            'received_stop': self.received_stop,
            'received_manual': self.received_manual,
            'received_auto': self.received_auto,
            'received_safety': self.received_safety,
            'safety_malformed': self.safety_malformed,
            'dropped_while_disabled': self.dropped_while_disabled,
            'rejected_malformed': self.rejected_malformed,
            'stop_active': self.stop_active,
            'publishers_known': self.publishers_known,
            'nav_publishers': self.nav_publishers,
            'demo_publishers': self.demo_publishers,
            'manual_publishers': self.manual_publishers,
            'auto_publishers': self.auto_publishers,
            'firmware_confirmed': self.firmware_confirmed,
            'last_nav_monotonic': self.last_nav_monotonic,
        }


def _extract_twist(msg: Any) -> tuple[float, float, float] | None:
    """从 ``Twist`` 或鸭子类型对象取 (vx, vy, vz)。取不到返回 None。

    只读 ``linear.x`` / ``linear.y`` / ``angular.z``；其余轴忽略（4WD 固件
    只用 Vx 与 Vz，见 ``chassis_cmd/__init__.py`` 的编码说明）。
    """
    linear = getattr(msg, 'linear', None)
    angular = getattr(msg, 'angular', None)
    if linear is None or angular is None:
        return None
    try:
        vx = float(getattr(linear, 'x'))
        vy = float(getattr(linear, 'y'))
        vz = float(getattr(angular, 'z'))
    except (AttributeError, TypeError, ValueError):
        return None
    return vx, vy, vz


def _extract_bool(msg: Any) -> bool | None:
    """从 ``std_msgs/Bool`` 或鸭子类型对象取布尔值。取不到返回 None。"""
    if isinstance(msg, bool):
        return msg
    data = getattr(msg, 'data', None)
    if data is None:
        return None
    if isinstance(data, bool):
        return data
    # 只接受 0/1 等价的数值，其它按无法判定处理（保守：不当作停车激活）
    try:
        value = float(data)
    except (TypeError, ValueError):
        return None
    if value not in (0.0, 1.0):
        return None
    return value != 0.0


class RosStraightInput:
    """把 ROS 消息转成命令链调用的适配器（默认关闭）。

    参数
    ----
    chain : 具备 ``submit_nav`` / ``submit_demo`` / ``set_sources`` /
        ``set_stop`` 的对象（通常为
        :class:`~c50c_uart3.chassis_cmd.chain.StraightCommandChain`）。
    enabled : 是否启用。默认 ``False``：启用前所有输入被丢弃并计数。
    clock : 单调时钟可调用对象，默认 ``time.monotonic``。
    """

    def __init__(self, chain: Any, *, enabled: bool = False,
                 clock: Any = None, nonzero_gate: Any = None) -> None:
        self._chain = chain
        self._clock = clock if callable(clock) else time.monotonic
        # 可选的"非零放行门"：无参可调用对象，返回 True 才允许把来源命令
        # 标为有输出许可。缺省（None）表示按第十轮之前的行为，只依赖来源
        # 检查——桥会传入自己的会话门，见 bridge.StraightDriveBridge。
        self._nonzero_gate = nonzero_gate
        self.state = RosInputState(enabled=bool(enabled))
        # 每条来源一个"许可标志"：只有置位期间收到、且之后仍置位的命令
        # 才可能被仲裁器输出。禁用、来源不可确认、重连都会清掉它。
        self._nav_licensed = False
        self._demo_licensed = False
        self._manual_licensed = False
        self._auto_licensed = False

    # ------------------------------------------------------------ 内部动作
    def _revoke(self, kind: str) -> None:
        """作废某条来源的命令许可并请求零速。

        只作废许可还不够：仲裁器缓存里可能还有禁用前的旧值。因此这里向
        命令链提交一次零速命令，让缓存里的非零值被明确替换掉。零速命令是
        固件的授权帧，本身不是非零动作，提交它不需要任何额外授权。
        """
        now = float(self._clock())
        if kind == 'nav':
            self._nav_licensed = False
            self._chain.submit_nav(0.0, 0.0, 0.0, now)
        elif kind == 'manual':
            self._manual_licensed = False
            self._chain.submit_manual(0.0, 0.0, 0.0, now)
        elif kind == 'auto':
            self._auto_licensed = False
            self._chain.submit_auto(0.0, 0.0, 0.0, now)
        else:
            self._demo_licensed = False
            self._chain.submit_demo(0.0, 0.0, 0.0, now)

    def _revoke_all(self) -> None:
        self._revoke('nav')
        self._revoke('demo')
        self._revoke('manual')
        self._revoke('auto')

    def _gate_open(self) -> bool:
        """额外的非零放行门（缺省放行）。见 ``__init__`` 的 ``nonzero_gate``。"""
        gate = self._nonzero_gate
        if gate is None:
            return True
        return bool(gate())

    def _channel_safe(self, kind: str) -> bool:
        """单条来源是否可放行：发布者数已知且该通道恰好 1（demo ≤1）。

        与 :meth:`_set_publishers` 的 per-channel unsafe 判据共用同一
        判定——2026-11-14 修复的不一致点：旧实现放行条件允许 nav≤2、
        撤销条件却把 nav≠1 当不安全，两个常驻发布者下许可被反复撤销。
        数量为 0 同样不可放行：发布者消失后，缓存里的旧命令没有在世的
        来源，不得继续被输出。
        """
        if not self.state.publishers_known:
            return False
        if kind == 'nav':
            return self.state.nav_publishers == MAX_TRACKED_PUBLISHERS
        if kind == 'manual':
            return self.state.manual_publishers == 1
        if kind == 'auto':
            return self.state.auto_publishers == 1
        return self.state.demo_publishers <= 1

    def _outputs_allowed(self) -> bool:
        """整体门（on_cycle 用）：数量已知且没有任何通道超上限。

        这里只决定"本拍是否驱动命令链"（未知/超限时静默）；单条来源的
        放行由 :meth:`_channel_safe` 判定。
        """
        if not self.state.publishers_known:
            return False
        return (self.state.nav_publishers <= MAX_TRACKED_PUBLISHERS
                and self.state.demo_publishers <= 1
                and self.state.manual_publishers <= 1
                and self.state.auto_publishers <= 1)

    # ------------------------------------------------------------ 生命周期
    @property
    def enabled(self) -> bool:
        return self.state.enabled

    def enable(self) -> None:
        """显式开启。

        先作废两条来源的旧命令并请求零速，再置启用位：重新启用后只有
        **启用之后**到达的新命令可能进入后续安全门（第八轮修复）。
        """
        if not self.state.enabled:
            self._revoke_all()
            self.state.enabled = True

    def disable(self) -> None:
        """显式关闭。立即作废旧命令并请求零速；关闭期间到达的消息一律
        丢弃（不缓存）。重新启用不会恢复禁用前的任何命令。"""
        if self.state.enabled:
            self._revoke_all()
        self.state.enabled = False
        self.state.stop_active = False

    # ------------------------------------------------------------ 输入入口
    def on_nav_cmd(self, msg: Any) -> bool:
        """导航命令回调（外部程序遥控语义，上限 1 个发布者）。"""
        return self._on_twist(msg, 'nav')

    def on_demo_cmd(self, msg: Any) -> bool:
        """演示命令回调（默认不使用；保留以便集成时对齐既有接口）。"""
        return self._on_twist(msg, 'demo')

    def on_manual_cmd(self, msg: Any) -> bool:
        """网页手动命令回调（/manual_cmd_vel）。"""
        return self._on_twist(msg, 'manual')

    def on_auto_cmd(self, msg: Any) -> bool:
        """自动巡航命令回调（/auto_cmd_vel）。"""
        return self._on_twist(msg, 'auto')

    def on_safety(self, msg: Any) -> bool:
        """安全许可回调（/drive_safety，String JSON）。

        契约（与 fusion_node.safety_state 一致）：``v`` 必须等于
        :data:`SAFETY_PERMISSION_SCHEMA`；``allowed`` 必须是布尔；
        ``ttl_s`` 必须是正数。任何违反都按「明确拒绝」转发——损坏的
        许可不是没有许可（由仲裁器过期兜底），而是已知不可信。
        ``seq`` 仅用于诊断，接收端不靠它判断新旧。
        """
        if not self.state.enabled:
            self.state.dropped_while_disabled += 1
            return False
        raw = getattr(msg, 'data', None)
        try:
            payload = json.loads(raw)
        except (TypeError, ValueError, UnicodeDecodeError):
            payload = None
        if (not isinstance(payload, dict)
                or payload.get('v') != SAFETY_PERMISSION_SCHEMA
                or not isinstance(payload.get('allowed'), bool)
                or isinstance(payload.get('ttl_s'), bool)
                or not isinstance(payload.get('ttl_s'), (int, float))
                or payload.get('ttl_s') <= 0):
            self.state.safety_malformed += 1
            self._chain.set_safety_permission(False, 'malformed_permission',
                                              0.0, float(self._clock()))
            return False
        self.state.received_safety += 1
        allowed = payload['allowed']
        reason = str(payload.get('reason') or ('ok' if allowed else 'denied'))
        ttl = float(payload['ttl_s'])
        self._chain.set_safety_permission(allowed, reason, ttl,
                                          float(self._clock()))
        return True

    def _on_twist(self, msg: Any, kind: str) -> bool:
        if not self.state.enabled:
            self.state.dropped_while_disabled += 1
            return False
        twist = _extract_twist(msg)
        if twist is None:
            self.state.rejected_malformed += 1
            return False
        vx, vy, vz = twist
        now = float(self._clock())
        # 来源可疑（未知 / 0 / 多于上限）时，命令照样写进仲裁器缓存（仲裁器
        # 自己会判来源不明），但立刻作废本来源许可并请求零速——这样"来源
        # 恢复"后仲裁器缓存里只剩零速，没有旧命令可复活（第八轮修复）。
        licensed = self._channel_safe(kind) and self._gate_open()
        if kind == 'nav':
            self.state.received_nav += 1
            self.state.last_nav_monotonic = now
            self._chain.submit_nav(vx, vy, vz, now)
            self._nav_licensed = licensed
        elif kind == 'manual':
            self.state.received_manual += 1
            self._chain.submit_manual(vx, vy, vz, now)
            self._manual_licensed = licensed
        elif kind == 'auto':
            self.state.received_auto += 1
            self._chain.submit_auto(vx, vy, vz, now)
            self._auto_licensed = licensed
        else:
            self.state.received_demo += 1
            self._chain.submit_demo(vx, vy, vz, now)
            self._demo_licensed = licensed
        if not licensed and not (vx == 0.0 and vy == 0.0 and vz == 0.0):
            self._revoke(kind)
        return licensed

    def on_stop(self, msg: Any) -> bool:
        """监督停车回调。``Bool=True`` 激活停车，``False`` 解除。

        停车优先由仲裁器保证：激活边沿会立即丢弃缓存非零命令。本方法只
        转发，不参与优先级判断。
        """
        if not self.state.enabled:
            self.state.dropped_while_disabled += 1
            return False
        active = _extract_bool(msg)
        if active is None:
            self.state.rejected_malformed += 1
            return False
        self.state.received_stop += 1
        self.state.stop_active = active
        self._chain.set_stop(active)
        return True

    def set_nav_publishers(self, count: int) -> None:
        """更新导航话题的发布者数量（由 ROS 图事件或启动检查提供）。

        第八轮修复：数量未知、为 0、或从 1 降到 0 时都**不放行非零**，
        并且作废已有命令的许可——来源恢复后也不复活先前命令。
        """
        self._set_publishers(nav=count)

    def set_demo_publishers(self, count: int) -> None:
        self._set_publishers(demo=count)

    def publishers_unknown(self) -> None:
        """标记发布者数量未知（图事件丢失、查询失败、启动未完成）。

        未知即不放行非零；已有命令立即作废。
        """
        self.state.publishers_known = False
        self.state.nav_publishers = 0
        self.state.demo_publishers = 0
        self.state.manual_publishers = 0
        self.state.auto_publishers = 0
        self._revoke_all()
        if self.state.enabled:
            self._chain.set_sources(0, 0, 0, 0)

    def set_manual_publishers(self, count: int) -> None:
        self._set_publishers(manual=count)

    def set_auto_publishers(self, count: int) -> None:
        self._set_publishers(auto=count)

    def _set_publishers(self, *, nav: int | None = None,
                        demo: int | None = None,
                        manual: int | None = None,
                        auto: int | None = None) -> None:
        """更新各通道发布者数量（由 ROS 图事件或周期复核提供）。

        判据与 :meth:`_channel_safe` 一致并**按通道**独立处理：进入或
        离开"该通道可放行"区间（含 1→0 的发布者消失）都作废该通道的
        旧命令，来源恢复后只能依赖恢复之后到达的新命令（第八轮修复）。
        未发生转换的通道不受牵连——这是对旧实现"一次撤销清空全部来源"
        的修正，也是两常驻发布者互相覆盖问题的根源之一。
        """
        new_nav = self.state.nav_publishers if nav is None else int(nav)
        new_demo = self.state.demo_publishers if demo is None else int(demo)
        new_manual = (self.state.manual_publishers if manual is None
                      else int(manual))
        new_auto = self.state.auto_publishers if auto is None else int(auto)
        new_nav = max(new_nav, 0)
        new_demo = max(new_demo, 0)
        new_manual = max(new_manual, 0)
        new_auto = max(new_auto, 0)
        cap = MAX_TRACKED_PUBLISHERS
        was_known = self.state.publishers_known
        # 每个通道的"可放行"状态：nav/manual/auto 恰好 1，demo ≤1。
        now_safe = {
            'nav': new_nav == cap,
            'demo': new_demo <= 1,
            'manual': new_manual == 1,
            'auto': new_auto == 1,
        }
        was_safe = {
            'nav': self.state.nav_publishers == cap,
            'demo': self.state.demo_publishers <= 1,
            'manual': self.state.manual_publishers == 1,
            'auto': self.state.auto_publishers == 1,
        }
        self.state.nav_publishers = new_nav
        self.state.demo_publishers = new_demo
        self.state.manual_publishers = new_manual
        self.state.auto_publishers = new_auto
        self.state.publishers_known = True
        for kind in ('nav', 'demo', 'manual', 'auto'):
            if not was_known or now_safe[kind] != was_safe[kind]:
                # 进入或离开"来源不可放行"区间都作废该通道旧命令：来源
                # 恢复后只能依赖恢复之后到达的新命令（第八轮修复）。
                self._revoke(kind)
        if self.state.enabled:
            self._chain.set_sources(new_nav, new_demo, new_manual, new_auto)

    # ------------------------------------------------------------ 重连
    def on_serial_reconnect(self) -> None:
        """串口重新打开。转发给命令链（清缓存并禁非零）。

        **不会**附带任何固件确认：重连后放行仍需独立的接收端证据。
        重连同时作废各来源的许可；重连之后到达的新命令仍需重新通过
        来源检查，不能凭重连前的许可复活。
        """
        self._nav_licensed = False
        self._demo_licensed = False
        self._manual_licensed = False
        self._auto_licensed = False
        self.state.received_nav = 0
        self.state.received_demo = 0
        self.state.last_nav_monotonic = None
        self._chain.on_transport_reconnect()

    def cmd_licensed(self, kind: str) -> bool:
        """该来源当前的命令是否仍有输出许可（诊断用）。"""
        if kind == 'nav':
            return self._nav_licensed
        if kind == 'manual':
            return self._manual_licensed
        if kind == 'auto':
            return self._auto_licensed
        return self._demo_licensed

    # ------------------------------------------------------------ 周期驱动
    def on_cycle(self, step: Any) -> Any:
        """周期回调：把"适配层门控"与"命令链一次 tick"绑在一起。

        ``step`` 是无参可调用对象（通常为 ``lambda: chain.tick(now)``）。
        返回 ``(gated, record)``：``gated`` 为 True 表示本拍因为来源不可确认
        或适配层已禁用而没有执行 ``step``（不写任何字节）。
        """
        if not self.state.enabled:
            return (True, None)
        if not self.state.publishers_known:
            return (True, None)
        if not self._outputs_allowed():
            # 来源不可确认：本适配层连零速都不发（保持静默）。
            return (True, None)
        # 没有来源持有输出许可时仍执行 ``step``。许可只表示"允许非零"，
        # 不表示"允许驱动"：无许可输入下仲裁器只给零速帧或静默。若在此
        # 早退，固件要求的首帧零速永远发不出去，桥会一直零字节。
        return (False, step())

    # ------------------------------------------------------------ 诊断
    def snapshot(self) -> dict:
        """适配层状态快照（不含仲裁器内部计数）。"""
        snap = self.state.snapshot()
        snap['nav_licensed'] = self._nav_licensed
        snap['demo_licensed'] = self._demo_licensed
        snap['manual_licensed'] = self._manual_licensed
        snap['auto_licensed'] = self._auto_licensed
        return snap


# ---------------------------------------------------------------- 订阅接线候选
class RosSubscriptionInput:
    """把 :class:`RosStraightInput` 接到 rclpy 订阅与周期驱动的候选接线。

    **默认关闭**。构造本对象不会创建任何节点、订阅或定时器；只有调用
    :meth:`start` 才会在**调用方提供的节点**上创建。本类从不打开串口，
    也不调用任何模拟确认入口。

    参数
    ----
    node : ``rclpy.node.Node`` 实例；本类只向它挂订阅与定时器。
    adapter : :class:`RosStraightInput` 实例。
    nav_topic / demo_topic / manual_topic / auto_topic : 命令话题（相对名，
        跟随节点命名空间）。``manual`` 接网页手动，``auto`` 接自动巡航，
        ``nav`` 保留给外部程序（手动遥控语义）。
    stop_topic : 监督停车（std_msgs/Bool）。
    safety_topic : 安全许可（std_msgs/String JSON，来自 fusion_node）。
    tick_period_s : 周期驱动间隔。
    """

    def __init__(self, node: Any, adapter: RosStraightInput, *,
                 nav_topic: str = DEFAULT_NAV_TOPIC,
                 demo_topic: str = DEFAULT_DEMO_TOPIC,
                 stop_topic: str = DEFAULT_STOP_TOPIC,
                 manual_topic: str = DEFAULT_MANUAL_TOPIC,
                 auto_topic: str = DEFAULT_AUTO_TOPIC,
                 safety_topic: str = DEFAULT_SAFETY_TOPIC,
                 tick_period_s: float = 0.05) -> None:
        if tick_period_s <= 0:
            raise ValueError('周期必须为正')
        self._node = node
        self._adapter = adapter
        self._nav_topic = nav_topic
        self._demo_topic = demo_topic
        self._stop_topic = stop_topic
        self._manual_topic = manual_topic
        self._auto_topic = auto_topic
        self._safety_topic = safety_topic
        self._period = float(tick_period_s)
        self._subs: list = []
        self._timer = None
        self._last_record = None
        self.ticks_gated = 0
        self.ticks_run = 0

    @property
    def started(self) -> bool:
        return bool(self._subs)

    def start(self, *, with_timer: bool = True) -> None:
        """创建订阅（幂等）；``with_timer`` 决定是否同时建周期定时器。

        订阅的发布者数量由 :meth:`refresh_publishers` 在每次周期回调里
        复核；查询失败一律按"未知"处理，不放行非零。

        ``with_timer=False`` 给"所有者已有一个周期回调"的节点用：
        :meth:`cycle` 由所有者调用，避免两处定时器同时 tick。
        """
        if self._subs:
            return
        # 延迟导入：本模块在无 ROS 环境下也必须可导入。
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool, String

        self._subs = [
            self._node.create_subscription(Twist, self._manual_topic,
                                           self._adapter.on_manual_cmd, 10),
            self._node.create_subscription(Twist, self._nav_topic,
                                           self._adapter.on_nav_cmd, 10),
            self._node.create_subscription(Twist, self._auto_topic,
                                           self._adapter.on_auto_cmd, 10),
            self._node.create_subscription(Twist, self._demo_topic,
                                           self._adapter.on_demo_cmd, 10),
            self._node.create_subscription(Bool, self._stop_topic,
                                           self._adapter.on_stop, 10),
            self._node.create_subscription(String, self._safety_topic,
                                           self._adapter.on_safety, 10),
        ]
        if with_timer:
            self._timer = self._node.create_timer(self._period, self.cycle)

    def refresh_publishers(self) -> None:
        """查询各命令话题的发布者数量；失败按未知处理（不放行）。"""
        try:
            resolve = self._node.resolve_topic_name
            nav = self._node.count_publishers(resolve(self._nav_topic))
            demo = self._node.count_publishers(resolve(self._demo_topic))
            manual = self._node.count_publishers(resolve(self._manual_topic))
            auto = self._node.count_publishers(resolve(self._auto_topic))
        except Exception:      # noqa: BLE001——查询失败等同"未知"
            self._adapter.publishers_unknown()
            return
        self._adapter.set_nav_publishers(int(nav))
        self._adapter.set_demo_publishers(int(demo))
        self._adapter.set_manual_publishers(int(manual))
        self._adapter.set_auto_publishers(int(auto))

    def cycle(self) -> None:
        """周期回调：复核来源 → 门控 → 命令链 tick。"""
        self.refresh_publishers()
        chain = getattr(self._adapter, '_chain', None)   # noqa: SLF001
        clock = getattr(self._adapter, '_clock', time.monotonic)  # noqa: SLF001
        if chain is None:
            return
        gated, record = self._adapter.on_cycle(lambda: chain.tick(clock()))
        if gated:
            self.ticks_gated += 1
            return
        self.ticks_run += 1
        self._last_record = record

    def stop_timer(self) -> None:
        """只销毁周期定时器，保留订阅。

        给"由所有者统一定时"的节点用：订阅回调照常喂消息，tick 由所有者
        唯一的定时器驱动，避免两处同时 cycle 导致一拍写两帧。
        """
        if self._timer is not None:
            destroy = getattr(self._timer, 'destroy', None)
            if callable(destroy):
                destroy()
        self._timer = None

    def stop(self) -> None:
        """销毁订阅与定时器（不触碰命令链状态）。"""
        for sub in self._subs:
            destroy = getattr(sub, 'destroy', None)
            if callable(destroy):
                destroy()
        if self._timer is not None:
            destroy = getattr(self._timer, 'destroy', None)
            if callable(destroy):
                destroy()
        self._subs = []
        self._timer = None

    def snapshot(self) -> dict:
        return {
            'started': self.started,
            'nav_topic': self._nav_topic,
            'demo_topic': self._demo_topic,
            'stop_topic': self._stop_topic,
            'manual_topic': self._manual_topic,
            'auto_topic': self._auto_topic,
            'safety_topic': self._safety_topic,
            'tick_period_s': self._period,
            'ticks_gated': self.ticks_gated,
            'ticks_run': self.ticks_run,
            'adapter': self._adapter.snapshot(),
        }
