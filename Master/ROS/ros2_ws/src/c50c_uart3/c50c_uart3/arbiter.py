"""受控直行的命令仲裁（纯逻辑，无 ROS、无串口、无 I/O）。

设计依据（只读核对）：已审核方案
``/home/snow/robot-task-worktrees/c50c-nav-wiring.3hd8/worktree/task-results/
c50c-nav-wiring-plan.md`` §3.2.1/§3.2.2 与任务书 B 第 3 条。本模块只实现
**第一阶段直行所需的最小子集**，不实现导航、避障、建图。

核心规则（每条都有对应测试）：

1. **来源只认独立输入**：导航命令 ``/nav_cmd_vel``、演示命令
   ``/demo_cmd_vel``、监督停车 ``~/nav_stop`` 三个独立输入。同一话题上
   出现多个发布者即视为来源不明——不允许非零输出。
2. **停车优先且不可被非零覆盖**：停车状态由假变真时立即丢弃缓存的非零
   命令；停车期间到达的非零命令不入缓存。因此解除停车后没有任何"旧命令"
   可被恢复，只有解除之后新到达的命令才是候选（不排队、不改写）。
3. **过期不重发**：导航命令超过 ``nav_timeout_s`` 没有新消息，仲裁器
   **不重发最后一条非零命令**；输出降为零速帧（受控停止 + 心跳）。
   非零命令还带 ``nav_cmd_max_age_s``：单条命令自身过旧时即使是新到达的
   也不放行（防止上游阻塞后一次性冲出的旧命令）。
4. **写失败立即锁存停车**：串口写入失败后必须人工/监督显式复位，不能
   仅凭下一帧成功就恢复。
5. **重启清零**：仲裁器所有状态在构造时为零；重启后第一条输出必为零速，
   未重置的停车锁存（来自磁盘续跑）同样只给零速。
6. **重连禁止非零**：重连事件把仲裁器推到"禁止非零"，放行需要
   :meth:`Arbiter.on_firmware_confirm` 给出**固件侧显式确认**。
   延迟、写入成功、重复零速帧都**不能**触发放行（任务书 B 第 3 条）。
7. **零速豁免**：零速帧**不受以上非零门控限制**（除"写失败锁存"外），
   因为零速帧是固件的授权帧，也是唯一安全的输出。零速帧放行时
   ``zero_reason`` 记录它为什么为零。

时间一律用调用方传入的**单调时钟**秒值（``monotonic`` 语义），不比较
消息时间戳——消息戳来自 ROS 时钟，可被 ``use_sim_time`` 影响。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from . import is_zero_command, validate_speed_command
from .ros_input import MAX_TRACKED_PUBLISHERS

# 仲裁器判「来源不明」用的发布者上限，与桥输入层放行上限保持一致。
# 导航允许 2 个（Web GUI 手动控制与自主壁障节点并存），演示只允许 1 个。
MAX_NAV_PUBLISHERS = MAX_TRACKED_PUBLISHERS
MAX_DEMO_PUBLISHERS = 1
from .seq import FrameSeq


class ZeroReason(str, Enum):
    """零速输出的来源，用于日志与诊断（每条都对应一个失效或状态）。"""

    NONE = 'none'                       # 不是零速（正常输出非零）
    STOP_REQUESTED = 'stop_requested'   # 监督停车激活
    NO_NAV_COMMAND = 'no_nav_command'   # 从未收到过导航命令
    NAV_EXPIRED = 'nav_expired'         # 导航命令过期（未重发）
    NAV_STALE = 'nav_stale'             # 新到达但自身已过旧
    AMBIGUOUS_SOURCES = 'ambiguous_sources'  # 同一输入多个发布者
    NAV_INVALID = 'nav_invalid'         # 数值非法或超限
    ALL_SOURCES_ZERO = 'all_sources_zero'    # 来源明确命令本身为零
    OUTPUT_BLOCKED = 'output_blocked'   # 重连/重启后未获确认，禁止非零
    WRITE_FAILED = 'write_failed'       # 串口写入失败锁存
    BRIDGE_REARMING = 'bridge_rearming' # 桥没有明确的输出授权（启动态）
    STREAM_POISONED = 'stream_poisoned' # 字节流已有残帧，禁止再写任何帧
    RECEIVER_UNKNOWN = 'receiver_unknown'  # 接收端状态未证明安全，保持静默
    NO_SESSION = 'no_session'           # 固件会话未确认（或尚未握手）
    SESSION_STALE = 'session_stale'     # 会话帧长时间未再出现，授权失效


# 会话帧由固件每 1 s 发一次（data_task.c：每 DATA_TASK_RATE 拍一帧）。
# 容忍连续 3 帧丢失后判定会话失效：再久就无法证明固件仍在当前会话里。
SESSION_MAX_AGE_S = 3.0

#: 候选固件（chassis_ack.c）：要求会话帧 + 逐帧序号，``frame[2]=0x80|seq``。
PROTOCOL_V3 = 'v3'
#: 已交付基线（2026-11-05 c50c-four-wheel-50pct）：**没有**会话帧、没有序号、
#: 没有 ack 位；``frame[1]==0`` 即速度帧，``frame[2]`` 不被读取。
PROTOCOL_BASELINE = 'baseline'
FIRMWARE_PROTOCOLS = (PROTOCOL_V3, PROTOCOL_BASELINE)


@dataclass
class Command:
    vx: float
    vy: float
    vz: float
    received_monotonic: float


@dataclass
class Decision:
    """一次仲裁结果。``frame`` 为 ``None`` 表示本周期不发送任何字节。"""

    frame: bytes | None
    is_zero: bool
    zero_reason: ZeroReason
    keepalive: bool = False             # 零速帧是否会刷新固件监督计时（取决于固件是否接受）
    # 诊断计数（累计），便于测试与日志
    counts: dict = field(default_factory=dict)


class Arbiter:
    """第一阶段直行候选的命令仲裁器。全部状态在内存，进程退出即消失。"""

    def __init__(self,
                 nav_timeout_s: float = 0.3,
                 nav_cmd_max_age_s: float = 0.5,
                 max_linear_mps: float | None = None,
                 max_angular_rad_s: float | None = None,
                 session_max_age_s: float = SESSION_MAX_AGE_S,
                 firmware_protocol: str = PROTOCOL_V3,
                 *, max_forward_mps: float | None = None,
                 ) -> None:
        if nav_timeout_s <= 0 or nav_cmd_max_age_s <= 0:
            raise ValueError('超时阈值必须为正')
        if session_max_age_s <= 0:
            raise ValueError('会话时效必须为正')
        if firmware_protocol not in FIRMWARE_PROTOCOLS:
            raise ValueError(f'未知固件协议 {firmware_protocol!r}；'
                             f'可选 {sorted(FIRMWARE_PROTOCOLS)}')
        self._firmware_protocol = firmware_protocol
        self._nav_timeout_s = float(nav_timeout_s)
        self._nav_cmd_max_age_s = float(nav_cmd_max_age_s)
        self._session_max_age_s = float(session_max_age_s)
        self._last_reject = ''
        self._last_gate = ''
        self._max_linear = max_linear_mps
        self._max_angular = max_angular_rad_s
        self._max_forward = max_forward_mps

        self._nav: Command | None = None
        self._demo: Command | None = None
        self._nav_publishers = 0
        self._demo_publishers = 0
        self._stop_active = False
        self._write_failed = False
        self._stream_poisoned = False
        # 启动/复位/重连后还没有输出过零速帧；为 False 时只允许零速。
        self._sent_zero_since_arm = False
        # 接收端是否已被证明处于安全（可解析对齐）状态。
        # 第五轮改判：默认 **False = 未知**。首帧零速只适用于已满足
        # 安全发送前提的情形，不能用来证明前提成立。
        self._receiver_clean = False
        self._rearmed = False
        # 会话握手（A 候选协议）。``_session_id`` 只在线上真的读到会话帧
        # 时被赋值；``_session_ack`` 表示本进程已回填过该会话 ID 的确认帧。
        self._session_id: int | None = None
        self._session_ack = False
        self._session_observed_at: float | None = None
        self._session_stale_events = 0
        # 基线固件方言（PROTOCOL_BASELINE）的接收方向证据：线上最近一帧
        # 合法状态帧的时刻与计数。只在基线方言下被赋值。
        self._baseline_observed_at: float | None = None
        self._baseline_frames_seen = 0
        # v3 逐帧序号：唯一状态归属对象，与上面的会话/授权状态并列，
        # 随会话重置、重连、操作者复位一起清零（固件新会话首帧接受任意
        # 序号，所以从 0 重新开始与契约一致）。
        self._seq = FrameSeq()
        self._counts = {
            'sent_nonzero': 0, 'sent_zero': 0, 'dropped_stop': 0,
            'dropped_blocked': 0, 'dropped_invalid': 0,
            'dropped_ambiguous': 0, 'dropped_write_failed': 0,
            'reconnect_events': 0, 'restarts': 1,
            'dropped_stream_poisoned': 0, 'dropped_receiver_unknown': 0,
            'dropped_no_session': 0, 'dropped_session_stale': 0,
            'dropped_baseline_gate': 0,
        }

    # ------------------------------------------------------------ 输入事件
    def on_nav_cmd(self, vx: float, vy: float, vz: float, now: float) -> None:
        cmd = Command(float(vx), float(vy), float(vz), float(now))
        self._nav = self._drop_nonzero(cmd) if self._stop_active else cmd

    def on_demo_cmd(self, vx: float, vy: float, vz: float, now: float) -> None:
        cmd = Command(float(vx), float(vy), float(vz), float(now))
        self._demo = self._drop_nonzero(cmd) if self._stop_active else cmd

    def on_publisher_count(self, nav_publishers: int, demo_publishers: int) -> None:
        self._nav_publishers = int(nav_publishers)
        self._demo_publishers = int(demo_publishers)

    def on_stop(self, active: bool) -> None:
        """监督停车输入。由假变真时**立即丢弃**缓存的非零命令。

        必须在**边沿**处理，不能只在 :meth:`evaluate` 里清理：若停车激活与
        解除之间没有求值调用（周期调度被阻塞、单拍停车），缓存会留到解除
        之后，旧命令在新命令到达前仍可能被输出。停车的唯一安全语义是
        "解除后只能靠解除之后新收到的命令重新起步"。

        重复的 ``on_stop(True)``（状态未变化）**不**再清除缓存，避免把解除
        停车后正常到达的命令一并清掉。停车期间到达的非零命令由
        :meth:`_drop_nonzero` 在缓存处丢弃，不可能成为解除后的候选。
        """
        active = bool(active)
        if active and not self._stop_active:
            self._stop_active = True
            self._drop_cached_nonzero()
        else:
            self._stop_active = active

    def _drop_nonzero(self, cmd: Command | None) -> Command | None:
        """停车期间到达的非零命令不入缓存（零速命令仍保留，用于判零）。"""
        if cmd is None or is_zero_command(cmd.vx, cmd.vy, cmd.vz):
            return cmd
        self._counts['dropped_stop'] += 1
        return None

    def _drop_cached_nonzero(self) -> None:
        """丢弃全部缓存的非零命令；零速命令保留（它不是可执行的候选）。"""
        for attr in ('_nav', '_demo'):
            setattr(self, attr, self._drop_nonzero(getattr(self, attr)))

    def on_serial_reconnect(self) -> None:
        """串口重新打开。**清除所有历史命令与会话证据**，并禁止非零输出。

        串口重开后固件可能已重启（新会话 ID），也可能保持原状。两种情况
        都不能假定：会话观测一并作废，必须重新在线上读到会话帧。

        **不动逐帧序号**：固件未重启时它仍记得最后一个已接受序号，
        主机从 0 重发会被 ``check_seq`` 判 STALE_SEQ（delta = 128 - last
        ≥ 64），重连后的零速帧同样被拒。序号的唯一重置条件是会话 ID
        变化，由 :meth:`on_session_observed` 负责（它先于第一条命令帧
        被调用：主机必须先读到会话帧才能回填确认）。
        """
        self._nav = None
        self._demo = None
        self._rearmed = False
        self._sent_zero_since_arm = False
        self._session_id = None
        self._session_ack = False
        self._session_observed_at = None
        self._counts['reconnect_events'] += 1

    def on_session_observed(self, session_id: int, now: float, *,
                            accepted: bool = False) -> bool:
        """从线上读到固件会话帧（``frame[1]==0x7F``）时调用。

        这是本候选**唯一**会把 ``_rearmed`` 置真的路径：它依据的是接收
        方向真实发生的字节，而不是主机自己写出去的东西。

        ``accepted=False``（收到帧、尚未回填确认）→ 只记录会话，仍禁非零；
        回填确认帧成功后再以 ``accepted=True`` 调用一次 → 放行。

        返回 ``True`` 表示这次调用放行了非零。
        """
        if not isinstance(session_id, int):
            raise TypeError('会话 ID 必须是整数')
        changed = session_id != self._session_id
        self._session_id = session_id
        self._session_observed_at = float(now)
        if changed:
            # 会话 ID 变了：固件已重启或换了会话。旧授权不能沿用，
            # 逐帧序号也从新会话的第一帧重新开始。
            self._session_ack = False
            self._rearmed = False
            self._seq.reset()
        if accepted:
            self._session_ack = True
            self._rearmed = True
            return True
        return False

    def on_session_ack_failed(self) -> None:
        """确认帧没写出去：保持禁非零（不因为"尝试过"而放行）。"""
        self._session_ack = False
        self._rearmed = False

    def on_baseline_status_frame(self, now: float) -> None:
        """基线固件：从线上读到一帧合法 48 字节状态帧。

        基线固件不发会话帧、不回确认，唯一的接收方向证据就是它自己的
        状态帧（``data_task.c`` 每 20 Hz 一帧）。这里只把这一事实记为
        "接收方向已同步"，**不**把 ``_rearmed`` 置真——放行由
        :meth:`on_baseline_arm` 在首帧零速**写成功**之后给出。

        协议不是基线方言时抛错：在 v3 固件上，状态帧不能替代会话帧。
        """
        if self._firmware_protocol != PROTOCOL_BASELINE:
            raise RuntimeError(
                'on_baseline_status_frame 只适用于基线固件方言；'
                f'当前为 {self._firmware_protocol!r}')
        self._baseline_observed_at = float(now)
        self._baseline_frames_seen += 1
        self._receiver_clean = True

    def on_baseline_arm(self, now: float) -> None:
        """基线固件：首帧零速已被接收端接受，放行非零。"""
        if self._firmware_protocol != PROTOCOL_BASELINE:
            raise RuntimeError('on_baseline_arm 只适用于基线固件方言')
        if self._baseline_observed_at is None:
            raise RuntimeError('还没有接收方向证据，不能放行非零')
        if not self.baseline_window_open(now):
            raise RuntimeError('基线状态窗口已过期，不能放行非零')
        self._rearmed = True

    def baseline_window_open(self, now: float) -> bool:
        """基线状态帧的时效窗口是否仍打开（与 v3 的 3 s 同义）。"""
        if self._baseline_observed_at is None:
            return False
        return (now - self._baseline_observed_at) <= self._session_max_age_s

    @property
    def firmware_protocol(self) -> str:
        return self._firmware_protocol

    @property
    def baseline_frames_seen(self) -> int:
        return self._baseline_frames_seen

    def session_window_open(self, now: float) -> bool:
        """会话窗口是否仍然打开（线上最近一条会话帧未过期）。

        没有写入断点的旧测试设施只调用 ``on_firmware_confirm(True)``，
        那条路径不携带会话观测时刻。此时按"未观测到会话过期"处理：
        一旦 :meth:`on_session_observed` 被调用过，就严格按 3 s 时效判定。
        """
        if not self._rearmed:
            return False
        if self._session_observed_at is None:
            return True
        return (now - self._session_observed_at) <= self._session_max_age_s

    @property
    def session_id(self) -> int | None:
        return self._session_id

    @property
    def session_acknowledged(self) -> bool:
        return self._session_ack

    @property
    def session_observed_at(self) -> float | None:
        return self._session_observed_at

    @property
    def session_stale_events(self) -> int:
        return self._session_stale_events

    @property
    def frame_seq(self) -> FrameSeq:
        """本会话逐帧序号的唯一归属对象（每出一个速度帧前进一）。"""
        return self._seq

    def on_firmware_confirm(self, confirmed: bool) -> None:
        """固件侧显式确认（历史入口，保留兼容）。

        第十一轮起，放行改由 :meth:`on_session_observed` 依据**线上会话帧**
        驱动，本方法不再是生产放行路径。它仍保留，因为测试设施用它构造
        受控状态；生产路径没有调用点。
        """
        if confirmed:
            self._rearmed = True

    def on_write_result(self, success: bool, stream_poisoned: bool = False) -> None:
        """串口写入结果。失败即锁存，直到 :meth:`clear_write_fault`。

        ``stream_poisoned=True`` 表示本次失败可能是**部分写入或写入异常**，
        即已写出的字节数不可知、字节流已被切开。此时除写失败锁存外，还要
        置流毒标记：**此后不再允许任何帧（包括零速帧）离开进程**，因为任何
        后续帧都可能与残帧拼接成错误帧。该标记比写失败锁存更强，操作者清错
        也不解除（见 :meth:`clear_write_fault`）。
        """
        if not success:
            self._write_failed = True
            # 写失败后不再信任任何缓存命令，并重新要求零速首帧
            self._nav = None
            self._demo = None
            self._sent_zero_since_arm = False
            if stream_poisoned:
                self._stream_poisoned = True

    def mark_receiver_clean(self, *, simulated: bool) -> None:
        """声明接收端处于已证明安全的状态。

        **当前没有已验证的同步机制**，所以本方法只接受 ``simulated=True``，
        即仅供测试构造明确的模拟干净状态。生产路径没有调用点；若将来实现
        真实同步，必须先有独立的接收端证据（例如解析器确认），届时再开新入口。
        """
        if simulated is not True:
            raise ValueError('接收端安全状态只能由测试以 simulated=True 构造')
        self._receiver_clean = True

    @property
    def receiver_clean(self) -> bool:
        return self._receiver_clean

    @property
    def stream_poisoned(self) -> bool:
        """字节流是否已被判毒。

        解除条件：所有者换一个新的接收端对象，并由测试或未来已授权的真实
        同步入口重新证明接收端干净。重建适配器、重连、清错都不解除。
        """
        return self._stream_poisoned

    def clear_write_fault(self, by_operator: bool) -> None:
        """清除写失败锁存。只允许操作者显式复位（受控阶段的人工门）。

        **不清除流毒标记**：操作者清错只表达"知道这次失败了"，不能证明
        残帧已消失。要恢复输出必须先换一个新的接收端对象，再重新证明接收端
        干净；当前没有真实同步机制，所以生产路径只能保持静默。
        """
        if by_operator:
            self._write_failed = False
            self._nav = None
            self._demo = None
            self._rearmed = False
            self._sent_zero_since_arm = False
            self._seq.reset()

    # ------------------------------------------------------------ 仲裁
    def _sources(self) -> list[tuple[str, Command | None, int, int]]:
        """(来源名, 缓存命令, 当前发布者数, 该来源允许的发布者上限)。"""
        return [('nav', self._nav, self._nav_publishers, MAX_NAV_PUBLISHERS),
                ('demo', self._demo, self._demo_publishers, MAX_DEMO_PUBLISHERS)]

    def evaluate(self, now: float) -> Decision:
        """按优先级仲裁一次。``now`` 为单调时钟秒值。"""
        # 1) 流毒：字节流里可能有残帧，任何帧（含零速）都不得再写。
        #    这是最强制约——写零速帧同样可能与残帧拼接，故返回 frame=None。
        if self._stream_poisoned:
            self._counts['dropped_stream_poisoned'] += 1
            d = Decision(frame=None, is_zero=True,
                         zero_reason=ZeroReason.STREAM_POISONED,
                         counts=dict(self._counts))
            d.keepalive = False
            return d

        # 2) 接收端状态未知：不发任何字节（含零速）。
        #    零速帧同样会把残帧的错位固定下来，故静默是唯一安全动作。
        if not self._receiver_clean:
            self._counts['dropped_receiver_unknown'] += 1
            d = Decision(frame=None, is_zero=True,
                         zero_reason=ZeroReason.RECEIVER_UNKNOWN,
                         counts=dict(self._counts))
            d.keepalive = False
            return d

        # 3) 写失败锁存：一切非零禁止（零速仍可输出，用于受控停止）
        if self._write_failed:
            return self._zero(ZeroReason.WRITE_FAILED, now)

        # 2) 监督停车：最高优先级。非零命令在到达时已被丢弃（见 _drop_nonzero），
        #    此处不再计数，也不保留任何可恢复的候选。
        if self._stop_active:
            return self._zero(ZeroReason.STOP_REQUESTED, now)

        # 3) 来源不明：某个输入的发布者数超过各自上限就不允许非零
        #    （DDS 不保证顺序，来源也无法区分——不猜测）。
        #    导航上限为 MAX_TRACKED_PUBLISHERS：Web GUI 手动控制与自主壁障节点
        #    共存时常态就是 2 个发布者，上限若按 1 判，车会永远只发零速。
        ambiguous = any(pub > cap for _, _, pub, cap in self._sources())

        # 4) 选择一个非零候选：导航优先于演示（演示默认关闭）
        candidate: tuple[str, Command] | None = None
        for name, cmd, _, _ in self._sources():
            if cmd is None:
                continue
            if is_zero_command(cmd.vx, cmd.vy, cmd.vz):
                continue
            candidate = (name, cmd)
            break

        if candidate is None:
            return self._zero(ZeroReason.NO_NAV_COMMAND if self._nav is None
                              else ZeroReason.ALL_SOURCES_ZERO, now)

        name, cmd = candidate
        if ambiguous:
            self._counts['dropped_ambiguous'] += 1
            return self._zero(ZeroReason.AMBIGUOUS_SOURCES, now)
        if self._firmware_protocol == PROTOCOL_BASELINE:
            # 基线固件方言：没有会话概念，改用接收方向状态帧证据。
            block = self._baseline_nonzero_block(now)
            if block is not None:
                self._counts['dropped_baseline_gate'] += 1
                return self._zero(block, now)
        elif self._v3_session_block(now) is not None:
            return self._zero(self._v3_session_block(now), now)

        age = now - cmd.received_monotonic
        # 诊断：记下最近一次拒绝原因与命令年龄，供 drive probe 打印。
        self._last_gate = ('age=%.3f src=%s cmd=(%.3f,%.3f,%.3f)'
                           % (age, name, cmd.vx, cmd.vy, cmd.vz))
        if age > self._nav_cmd_max_age_s:
            self._counts['dropped_invalid'] += 1
            self._last_reject = 'NAV_STALE ' + self._last_gate
            return self._zero(ZeroReason.NAV_STALE, now)
        if name == 'nav' and age > self._nav_timeout_s:
            # 新到的命令但已超时：不重发，按过期处理
            self._last_reject = 'NAV_EXPIRED ' + self._last_gate
            return self._zero(ZeroReason.NAV_EXPIRED, now)

        reason = validate_speed_command(cmd.vx, cmd.vy, cmd.vz,
                                        *(self._limits()),
                                        max_forward=self._max_forward)
        if reason is not None:
            self._counts['dropped_invalid'] += 1
            self._last_reject = ('NAV_INVALID %s %s' % (reason, self._last_gate))
            return self._zero(ZeroReason.NAV_INVALID, now, detail=reason)

        # 7) 首帧零速规则：桥在启动、复位、重连后的**第一条输出**必须是零速帧。
        #    固件只有收到零速帧才授权 ROS 控制；若桥开局直接发非零，固件会
        #    丢弃该帧，桥却以为在走。故未发过零速帧前一律先发零速。
        if not self._sent_zero_since_arm:
            self._sent_zero_since_arm = True
            return self._zero(ZeroReason.BRIDGE_REARMING, now)

        from . import encode_speed_frame
        from .baseline import encode_baseline_speed_frame
        if self._firmware_protocol == PROTOCOL_BASELINE:
            # 基线不读 frame[2]，因此这里没有序号也没有 ack 位。
            frame = encode_baseline_speed_frame(cmd.vx, cmd.vz, cmd.vy)
        else:
            frame = encode_speed_frame(cmd.vx, cmd.vz, cmd.vy,
                                       seq=self._seq.take())
        self._counts['sent_nonzero'] += 1
        return Decision(frame=frame, is_zero=False, zero_reason=ZeroReason.NONE,
                        counts=dict(self._counts))

    def _v3_session_block(self, now: float) -> ZeroReason | None:
        """v3 固件的会话门（原内联逻辑，抽出以便按方言分派）。"""
        # 会话门（第十一轮）：非零必须在"本会话已见过且未过期"时才放行。
        # 固件在没有有效会话时丢弃全部 USART3 速度帧（ChassisRxResult
        # NO_SESSION），所以"写成功"不等于"被接受"。会话窗口过期说明线上
        # 已经一段时间没读到会话帧——按失效处理，退回零速。
        # 说明：本门判定"见过会话"，不判定"固件确认过主机的确认帧"。
        # 唯一例外是测试设施的历史入口 on_firmware_confirm(True)，它不带来
        # 会话观测，按"未观测到会话过期"处理（见 session_window_open）。
        if self._session_id is None and not self._rearmed:
            self._counts['dropped_no_session'] += 1
            return ZeroReason.NO_SESSION
        if not self.session_window_open(now):
            self._session_stale_events += 1
            self._counts['dropped_session_stale'] += 1
            return ZeroReason.SESSION_STALE
        if not self._rearmed:
            self._counts['dropped_blocked'] += 1
            return ZeroReason.OUTPUT_BLOCKED
        return None

    def _baseline_nonzero_block(self, now: float) -> ZeroReason | None:
        """基线固件的非零门。

        基线**没有会话帧、没有 ack 位、没有序号**，所以 v3 的会话门在这里
        恒不成立。可用的接收方向证据只有固件自己的状态帧（20 Hz）：
        没有状态帧就无法证明对端在、也无法证明对齐，按 ``NO_SESSION``
        处理（语义等价：接收方向未同步）；状态帧过期按 ``SESSION_STALE``。
        首帧零速规则（``_sent_zero_since_arm``）在两种方言下一致保留。
        """
        if self._baseline_observed_at is None:
            self._counts['dropped_no_session'] += 1
            return ZeroReason.NO_SESSION
        if not self.baseline_window_open(now):
            self._session_stale_events += 1
            self._counts['dropped_session_stale'] += 1
            return ZeroReason.SESSION_STALE
        # 不再额外要求 ``_rearmed``：基线不回复确认，没有"等固件确认"的
        # 语义。首帧零速规则（``_sent_zero_since_arm``）在下面统一执行，
        # 已保证重新写满一帧零速之前不会放出非零。
        return None

    def _encode_zero(self) -> bytes:
        from . import encode_speed_frame
        from .baseline import encode_baseline_zero_frame
        if self._firmware_protocol == PROTOCOL_BASELINE:
            return encode_baseline_zero_frame()
        return encode_speed_frame(0.0, 0.0, 0.0, seq=self._seq.take())

    def _limits(self) -> tuple[float, float]:
        from . import PLACEHOLDER_MAX_ANGULAR_RAD_S, PLACEHOLDER_MAX_LINEAR_MPS
        return (self._max_linear if self._max_linear is not None
                else PLACEHOLDER_MAX_LINEAR_MPS,
                self._max_angular if self._max_angular is not None
                else PLACEHOLDER_MAX_ANGULAR_RAD_S)

    def _zero(self, reason: ZeroReason, now: float, detail: str = '') -> Decision:
        # 零速帧一旦真正发出（写成功与否由调用方的 on_write_result 判定），
        # 就视为"本授权周期已发过零速"；写入失败会在 on_write_result 里清掉。
        self._sent_zero_since_arm = True
        self._counts['sent_zero'] += 1
        d = Decision(frame=self._encode_zero(),
                     is_zero=True,
                     zero_reason=reason, counts=dict(self._counts))
        # 零速帧是否刷新固件监督计时取决于固件是否**接受**该帧；
        # 写失败锁存时本帧也未必写得出去，故 keepalive 标为不确定。
        d.keepalive = reason is not ZeroReason.WRITE_FAILED
        if detail:
            d.counts['last_detail'] = detail
        return d


def monotonic_now() -> float:
    """默认时钟：单调时钟秒值。"""
    import time
    return time.monotonic()


def is_finite(v: float) -> bool:
    return isinstance(v, (int, float)) and math.isfinite(v)
