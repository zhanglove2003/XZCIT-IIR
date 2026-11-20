"""单一串口所有者的接入接口（**只定义契约，不打开任何设备**）。

本模块是任务书 B（第二轮）第 2 条要求的"最小接入接口"的可执行形式。
它不实现桥、不打开串口、不驱动定时器；它只定义：

- 一个端口对象必须提供什么（:class:`SerialPortContract`）；
- 命令链如何借用端口（:class:`OwnerSerialTransport`）；
- 重连、写失败、确认缺失如何通知仲裁器（:class:`OwnerHarness`）。

设计前提（只读核对，见 ``task-results`` 报告 §1）：

- 当前 UART3 的唯一打开者是 ``uart3_node``（``_open_serial``），
  ``drive_enable`` 关闭时为只读 ``O_RDONLY`` + ``TIOCEXCL``；开启时为
  ``O_RDWR``，仍由同一进程、同一 fd 读写；
- 候选命令链（``chassis_cmd``）只面向 :class:`MemoryTransport`，
  真实写入端 ``SerialOwnerTransport`` 构造即拒绝；
- 因此接入方式必须是**扩展现有读取方**，让同一进程、同一 fd 同时读写，
  而不是新开第二个进程去抢同一个设备。

安全约束（任务书公共边界；第九轮修正见下）：

- 放行**两类**目标：隔离伪终端 ``/dev/pts/N``，或真实 UART
  （:func:`~.serial_port.is_real_uart`，例如 ``/dev/ttyACM0``）。
  真实 UART 的写权限仍由 :class:`~.serial_port.FdPort` 自己的
  ``fd_authorised`` 检查把关：未授权时 :class:`~.serial_port.FdPort`
  构造即抛 :class:`~.serial_port.RealFdUseForbidden`。其它路径一律抛
  :class:`NonIsolatedDeviceForbidden`。
- 写失败、重连、确认缺失一律走"禁止非零"，不得因写入返回成功而放行。
- 接收端解锁只有一条路径：收到**并校验通过**的固件状态帧，见
  :meth:`OwnerSerialTransport.note_firmware_status_frame`。缺证据时
  保持锁定。
"""
from __future__ import annotations

from dataclasses import dataclass, field
import os
from typing import Protocol, runtime_checkable

from .serial_port import is_real_uart
from .transport import PTS_PATTERN, MemoryTransport, RealDeviceAccessForbidden


class NonIsolatedDeviceForbidden(RealDeviceAccessForbidden):
    """目标不是隔离伪终端时抛出（继承原拒绝类型，便于统一捕获）。"""


#: 残留标记属性名。挂在**端口对象**（接收端）上，而不是适配器上。
RETAINED_TAINT_ATTR = '_c50c_wire_tainted'
RETAINED_SEQ_ATTR = '_c50c_stream_epoch'

#: 接收端"已同步"的显式状态名。只由**收到并校验通过的固件状态帧**置位
#: （:meth:`OwnerSerialTransport.note_firmware_status_frame`）。它不回写端口
#: 的干净登记，因此重建适配器后需要重新取得证据——证据缺失时就保持锁定。
RECEIVER_SYNCED_ATTR = '_c50c_receiver_synced'


#: 进程内的"已污染对象"登记表。
#:
#: 端口属性的唯一保证是：它只存在于该 Python 对象上，进程退出即消失。
#: 对 ``__slots__`` 等无法写属性的端口，属性根本不生效；这里用对象身份
#: 登记作为补充，处理"对象还活着但属性写不进去"的情况。
#:
#: 边界（不得夸大）：
#:
#: - 用字典存对象本身并以 ``is`` 比对身份，避免要求可哈希，也避免 ``id()``
#:   复用到新对象时误判。
#: - 真实进程重启、真实 fd 重开，本表必然丢失。本表**不提供**任何
#:   跨进程或"新包装对象指向同一接收端"的保证；后者由
#:   :func:`endpoint_is_tainted` 返回"不可判定"保守处理。
#:
#: 第七轮修正（审核反例）：污染一旦记录，**永不静默淘汰**。旧实现按先进
#: 先出淘汰污染表，而干净表仍留着同一对象的干净记录，于是被淘汰的污染端口
#: 会被重新判成"干净"并继续写字节。现在的规则：
#:
#: 1. 污染记录先于干净记录判定，两者同时存在时以污染为准；
#: 2. 登记表满且无法再保留新的污染时，置全局"饱和"标志，此后所有未显式
#:    标记干净的对象一律按"不可判定"处理；
#: 3. 干净表永不淘汰，只按对象身份删除被污染的对象。
_TAINTED_OBJECTS: list = []          # 保留名字供测试读取计数（长度恒为 0 或 1）
_TAINTED_IDS: dict = {}              # id -> (obj, reason)
_CLEAN_OBJECTS: list = []            # 测试声明过的干净对象，永不淘汰
_TAINTED_CAPACITY = 512
_saturated = False


def _object_is_tainted(port) -> bool:
    entry = _TAINTED_IDS.get(id(port))
    return entry is not None and entry[0] is port


def _is_remembered_clean(port) -> bool:
    for obj in _CLEAN_OBJECTS:
        if obj is port:
            return True
    return False


def _forget_clean(port) -> None:
    for index, obj in enumerate(_CLEAN_OBJECTS):
        if obj is port:
            del _CLEAN_OBJECTS[index]
            return


def _remember_clean(port) -> None:
    _forget_clean(port)
    _CLEAN_OBJECTS.append(port)


def _remember_tainted(port, reason: str) -> None:
    """登记污染。登记表满时进入"饱和"状态，而不是淘汰旧污染记录。"""
    global _saturated
    entry = _TAINTED_IDS.get(id(port))
    if entry is not None and entry[0] is port:
        return
    if len(_TAINTED_IDS) >= _TAINTED_CAPACITY:
        # 不淘汰旧记录：丢一条污染就可能让某个残帧流被判为干净。
        _saturated = True
        return
    _TAINTED_IDS[id(port)] = (port, reason)
    _forget_clean(port)


def registry_saturated() -> bool:
    """登记表是否已满，无法再保证新的污染记录。"""
    return _saturated


def _forget_endpoint_state(port) -> None:
    """仅供测试显式清除某对象的污染与干净记录（模拟新接收端）。"""
    _TAINTED_IDS.pop(id(port), None)
    _forget_clean(port)


def mark_endpoint_tainted(port, reason: str = '') -> None:
    """把"接收端字节流已被切开"标记到端口对象自身。

    为什么挂在端口上：重建适配器 / 重建仲裁器 / 重启进程都只丢弃内存对象，
    **接收端里的残帧不会因此消失**。若把锁只记在适配器上，重建就等于解锁，
    这是第五轮审核反例的根因。
    """
    try:
        setattr(port, RETAINED_TAINT_ATTR, True)
        setattr(port, RETAINED_SEQ_ATTR, getattr(port, RETAINED_SEQ_ATTR, 0) + 1)
        notes = getattr(port, '_c50c_taint_notes', None)
        if notes is None:
            notes = []
            setattr(port, '_c50c_taint_notes', notes)
        notes.append(reason)
        return
    except (AttributeError, TypeError):
        pass
    # 属性写不进去（__slots__、只读属性、禁止 setattr）：退回进程内登记表。
    # 登记只对**同一 Python 对象实例**有效，进程退出即消失。
    _remember_tainted(port, reason)
    if _object_is_tainted(port) or _saturated:
        # 饱和时无法记录本对象，但全局受控：任何未显式标记干净的对象都按脏处理。
        return
    raise AttributeError('端口无法保存残帧标记，且对象登记失败')


RETAINED_CLEAN_ATTR = '_c50c_wire_clean'


def _mark_endpoint_clean_for_test(port) -> None:
    """**仅供测试**声明该端口接收状态已知干净。

    生产路径绝不调用。写属性失败（``__slots__``）时退回对象登记。
    没有证据就没有干净；本函数是唯一可给出证据的入口，且只在 test/ 使用。
    """
    try:
        setattr(port, RETAINED_CLEAN_ATTR, True)
    except (AttributeError, TypeError):
        pass
    _remember_clean(port)


def endpoint_is_tainted(port):
    """端口自身的接收端是否已被标记为脏。

    - ``True``：本对象已被标记；
    - ``None``：**无法判定**——不是"干净"。调用方必须按脏处理。
    """
    # 1. 污染优先：任何污染证据都压过干净记录。
    if getattr(port, RETAINED_TAINT_ATTR, None):
        return True
    if _object_is_tainted(port):
        return True
    # 2. 登记表饱和：无法证明"这个对象没被污染过"，一律不可判定。
    if _saturated:
        return None
    # 3. 仅当没有污染证据时才采信干净记录。
    if getattr(port, RETAINED_CLEAN_ATTR, None) or _is_remembered_clean(port):
        return False
    return None



def _parse_status_frame(frame):
    """校验固件状态帧，合法返回会话 ID，否则返回 ``None``。

    复用 :func:`~.session.parse_session_frame`：它逐项校验帧头、帧尾、
    BCC、类型字节 ``0x7F``、子类型、协议版本与保留字节。这里延迟导入，
    避免包初始化阶段的模块环。
    """
    from .session import parse_session_frame

    return parse_session_frame(frame)


@runtime_checkable
class SerialPortContract(Protocol):
    """唯一所有者必须提供的**端口能力**（协议，不实现）。

    真实实现只允许出现在唯一所有者的进程内，且本轮不得启用。
    """

    def write_bytes(self, data: bytes) -> int:
        """向**已由所有者打开的同一个 fd** 写入。返回写入字节数。

        实现约束：

        - 不得自行 ``open()`` 设备；
        - 抛出 :class:`OSError` 表示通信失败（由调用方锁存停车）；
        - 返回小于 ``len(data)`` 视为失败（部分写入 = 帧被截断）。
        """
        ...

    def read_some(self, max_bytes: int) -> bytes:
        """从同一 fd 读取。返回空字节表示连接已断。"""
        ...

    def reopen(self) -> None:
        """所有者重开设备后调用。**不**代表固件已同步。"""
        ...


@dataclass
class PortStats:
    """端口统计（诊断用，不影响放行判定）。"""

    writes: int = 0
    bytes_written: int = 0
    write_failures: int = 0
    reopens: int = 0
    rejected_nonzero_while_blocked: int = 0
    detail: list[str] = field(default_factory=list)


class OwnerSerialTransport:
    """把命令链接到**唯一所有者的端口对象**上的适配器。

    与 :class:`MemoryTransport` 的关系：对外接口相同（``write`` 返回
    bool、可选 ``reconnect``），所以 :class:`~.chain.StraightCommandChain`
    不需要改动即可使用。区别在于：

    - 不自带 fd，不打开设备，只转发给注入的端口对象；
    - 端口对象必须指向隔离伪终端、真实 UART 或内存对象，否则构造即拒绝；
    - 部分写入算失败（帧被截断 = 固件可能拼出错误帧）；
    - 任何异常都转成 ``False``，由命令链锁存停车。

    **接收端同步规则（第九轮）**：新建适配器默认**锁定**，因为此时没有
    任何接收端证据。解锁只有一条路径：所有者把**校验通过的固件状态帧**
    交给 :meth:`note_firmware_status_frame`。该帧证明接收端按帧边界对齐，
    因此可作为"接收端已同步"的证据。若端口此前已有残帧证据
    （:data:`RETAINED_TAINT_ATTR` 或本适配器自己的锁），状态帧**不**解锁，
    并保留原锁定原因。写入成功永远不产生干净证据：本类没有任何
    "写成功即解锁"的路径。

    **流锁（第四轮）**：一旦发生部分写入或写入异常，已写出的字节数可能
    不可知，字节流可能在帧中间被切开。此时**不再向该端口写任何字节**
    （包括零速帧），因为任何后续帧都可能与残帧拼接成错误帧。锁只能由
    所有者**换一个新的接收端对象**并重新证明干净来解除；重连/清错/重复回调/
    重建适配器/重建仲裁器都不解除。注意：同一 Python 对象上的标记只在本
    进程有效，真实进程重启后本进程无法证明干净，只能保持静默。
    见 :attr:`stream_locked`。
    """

    kind = 'owner-adapted'

    def __init__(self, port, device: str | None = None) -> None:
        if device is not None:
            _assert_isolated(device)
        self._port = port
        self._device = device
        self.stats = PortStats()
        self._stream_lock_reason: str | None = None
        # 接收端同步状态：默认"未同步"。只由校验通过的固件状态帧置位。
        self._receiver_synced = False
        self._receiver_sync_frames = 0
        self._receiver_sync_session: int | None = None
        # 第五轮：接收端若已被标记（例如上一个适配器短写过），新适配器
        # 一开始就是锁定态。重建对象不等于重新同步。
        if endpoint_is_tainted(port) is True:
            self._stream_lock_reason = '接收端已标记残帧（继承自端口）'

    @property
    def stream_locked(self) -> bool:
        """字节流是否已被判定不可信（此后不再写任何字节）。

        判据按优先级排列：

        1. 本适配器的锁定原因非空（自己短写/异常，或构造时继承的残帧
           标记）→ 锁定；
        2. ``endpoint_is_tainted(port) is True``（端口有残帧证据）→ 锁定；
        3. ``endpoint_is_tainted(port) is False``（端口有干净登记）→ 放行；
        4. 其余情况（``None`` = 不可判定）→ 仅在
           :meth:`note_firmware_status_frame` 已置"接收端已同步"时放行。

        第 4 条是本轮（第九轮）修正。缺证据时行为不变：仍然锁定。
        """
        if self._stream_lock_reason is not None:
            return True
        tainted = endpoint_is_tainted(self._port)
        if tainted is True:
            return True
        if tainted is False:
            return False
        return not self._receiver_synced

    @property
    def receiver_synced(self) -> bool:
        """接收端是否已由**校验通过的固件状态帧**证明同步。"""
        return self._receiver_synced

    def note_firmware_status_frame(self, frame) -> bool:
        """把一条固件状态帧作为接收端同步证据。

        这是本类唯一的解锁入口，判据如下：

        1. 帧必须通过 :func:`~.session.parse_session_frame` 的全部校验
           （帧头、帧尾、BCC、``frame[1]==0x7F``、子类型、协议版本、
           保留字节）。不通过 → 不改变任何状态，返回 ``False``。
        2. 端口已有残帧证据（:func:`endpoint_is_tainted` 返回 ``True``）
           或本适配器已因短写/异常锁定 → **不**解锁，保留原锁定原因，
           返回 ``False``。残帧证据压过状态帧证据。
        3. 否则置"接收端已同步"，返回 ``True``。

        本方法不写端口、不做 I/O、不改变仲裁器状态；写入成功与调用本
        方法之间没有任何因果联系。
        """
        session_id = _parse_status_frame(frame)
        if session_id is None:
            self.stats.detail.append('status frame rejected: 校验未通过')
            return False
        if self._stream_lock_reason is not None:
            self.stats.detail.append(
                'status frame ignored: 本适配器已有残帧证据，不解锁'
                f'（reason={self._stream_lock_reason}）')
            return False
        if endpoint_is_tainted(self._port) is True:
            self.stats.detail.append(
                'status frame ignored: 端口已标记残帧，不解锁')
            return False
        self._receiver_synced = True
        self._receiver_sync_frames += 1
        self._receiver_sync_session = session_id
        self.stats.detail.append(
            f'status frame accepted: receiver synced (session={session_id:#010x})')
        return True

    def note_baseline_status_frame(self, frame: bytes | None = None) -> bool:
        """把"已从线上读到并校验通过一帧基线状态帧"作为接收端同步证据。

        与 :meth:`note_firmware_status_frame` 的分工（两者互不替代）：

        - v3 固件：11 字节会话帧（``frame[1]==0x7F``）由
          :meth:`note_firmware_status_frame` 逐字节校验；
        - 基线固件：没有会话帧，唯一的接收方向证据是 48 字节状态帧（task-15 起由 24 扩为 48）。

        两种调用形态，强度不同，**必须分清**：

        1. ``frame`` 给字节（推荐）：本方法用
           :func:`~.baseline.parse_status_frame` 亲自复核帧头 ``0x7B``、
           帧尾 ``0x7D``、前 46 字节 XOR BCC 与长度。校验不过 → 不解锁。
           这是唯一"本类自己能证明"的形态。
        2. ``frame`` 为 ``None``：调用方声明它已经校验过。本类**无法**
           复核字节，只能核对锁定状态。生产节点（``uart3_node.py`` 的
           ``_notify_drive_status_frame``）走这一形态，因为它在
           ``self._parser.feed()`` 之后才通知，且该文件本轮冻结不可改。
           这条路径的保证来自节点，不来自本类。

        判据（与 v3 入口一致，残帧证据压过状态帧证据）：

        1. 给了字节但校验不过 → 不解锁，返回 ``False``；
        2. 本适配器已有残帧证据（短写/异常）→ 不解锁，返回 ``False``；
        3. 端口有残帧证据（``endpoint_is_tainted`` 为 ``True``）→ 不解锁；
        4. 否则置"接收端已同步"，返回 ``True``。

        第 2 条不因后续状态帧而放宽：短写是**不可恢复**的帧边界错位，
        一次短写之后无论再收到多少状态帧，本适配器都不再写任何字节。

        本方法不写端口、不做 I/O、不改变仲裁器状态：写入成功与本方法之间
        没有因果联系。解锁只表示"接收方向的字节流按帧边界对齐"，不表示
        固件收到过任何命令帧（基线不回复确认）。
        """
        if frame is not None:
            from .baseline import parse_status_frame

            status = parse_status_frame(bytes(frame))
            if status is None:
                self.stats.detail.append(
                    'baseline status frame rejected: 字节校验未通过，不解锁'
                    f'（{len(bytes(frame))} bytes）')
                return False
        if self._stream_lock_reason is not None:
            self.stats.detail.append(
                'baseline status frame ignored: 本适配器已有残帧证据，不解锁'
                f'（reason={self._stream_lock_reason}）')
            return False
        if endpoint_is_tainted(self._port) is True:
            self.stats.detail.append(
                'baseline status frame ignored: 端口已标记残帧，不解锁')
            return False
        self._receiver_synced = True
        self._receiver_sync_frames += 1
        form = 'bytes-verified' if frame is not None else 'caller-verified'
        self.stats.detail.append(
            'baseline status frame accepted: receiver synced '
            f'(protocol=baseline, frames={self._receiver_sync_frames}, {form})')
        return True

    @property
    def stream_lock_reason(self) -> str | None:
        return self._stream_lock_reason

    def _lock_stream(self, reason: str) -> None:
        if self._stream_lock_reason is None:
            self._stream_lock_reason = reason
            self.stats.detail.append(f'stream locked: {reason}')
        # 标记持久化到接收端：重建适配器/仲裁器/进程都不能清除
        try:
            mark_endpoint_tainted(self._port, reason)
        except (AttributeError, TypeError) as exc:
            self.stats.detail.append(
                f'无法在端口上持久化残帧标记：{exc!r}——本适配器仍保持锁定')

    def write(self, data: bytes) -> bool:
        if self.stream_locked:
            self.stats.write_failures += 1
            self.stats.detail.append(
                'write refused: stream locked '
                f'({len(data)} bytes, reason={self._stream_lock_reason})')
            return False
        try:
            written = self._port.write_bytes(data)
        except OSError as exc:
            self.stats.write_failures += 1
            self.stats.detail.append(f'write OSError: {exc!r}')
            # 异常时已写出多少字节不可知 → 流不可信
            self._lock_stream(f'OSError: {exc!r}')
            return False
        except Exception as exc:      # noqa: BLE001——任何异常都按失败处理
            self.stats.write_failures += 1
            self.stats.detail.append(f'write error: {exc!r}')
            self._lock_stream(f'{type(exc).__name__}: {exc!r}')
            return False
        try:
            written = int(written)
        except (TypeError, ValueError):
            self.stats.write_failures += 1
            self.stats.detail.append(f'write returned non-integer: {written!r}')
            self._lock_stream('写入返回值不可解释')
            return False
        if written != len(data):
            self.stats.write_failures += 1
            self.stats.detail.append(
                f'partial write: {written}/{len(data)} bytes——帧被截断')
            # 前缀已离开进程：后续任何帧都可能拼接
            self._lock_stream(f'部分写入 {written}/{len(data)} 字节')
            return False
        self.stats.writes += 1
        self.stats.bytes_written += len(data)
        return True

    def reconnect(self) -> None:
        """仅由所有者重连时调用；只重置适配器统计，**不**解除流锁。

        端口上的残帧标记同样保留：重开 fd 不会清空接收端缓冲。
        """
        self.stats.reopens += 1
        reopen = getattr(self._port, 'reopen', None)
        if callable(reopen):
            reopen()


class BlockedNonzeroGuard:
    """包住命令链的写入端，在"禁止非零"状态下计数并拦截非零帧。

    用途：验证"重连后 / 无确认时"确实没有非零字节离开进程——即使仲裁器
    将来被改错，这一层也会拦住并计数。它不是主防线（主防线是仲裁器），
    是**可观察的第二道**。
    """

    def __init__(self, inner, blocked: bool = True) -> None:
        self._inner = inner
        self._blocked = bool(blocked)
        self.stats = PortStats()

    def set_blocked(self, blocked: bool) -> None:
        self._blocked = bool(blocked)

    @property
    def blocked(self) -> bool:
        return self._blocked

    @property
    def stream_locked(self) -> bool:
        """透出内层流锁状态，使其能被命令链读到并告知仲裁器。

        第五轮：不带这个属性时，外层包装会**吞掉**短写事实，
        命令链以为只是普通写失败，仲裁器因而仍允许写零速帧。
        """
        return bool(getattr(self._inner, 'stream_locked', True))

    @property
    def stream_lock_reason(self) -> str | None:
        return getattr(self._inner, 'stream_lock_reason', None)

    def write(self, data: bytes) -> bool:
        from . import is_zero_frame

        if self._blocked and not is_zero_frame(data):
            self.stats.rejected_nonzero_while_blocked += 1
            return False
        return self._inner.write(data)

    def reconnect(self) -> None:
        reconnect = getattr(self._inner, 'reconnect', None)
        if callable(reconnect):
            reconnect()


class OwnerHarness:
    """把"所有者事件"映射成仲裁器/命令链调用（纯转发，无 I/O）。

    **重连的非零门保持关闭**：``on_reconnected`` 只调用
    ``chain.on_transport_reconnect()``，后者会清除历史命令并禁止非零。
    本类**不提供**任何"写入成功即放行"的路径。
    """

    def __init__(self, chain) -> None:
        self._chain = chain
        self.events: list[tuple[str, bool]] = []

    def on_reconnected(self) -> None:
        self._chain.on_transport_reconnect()
        self.events.append(('reconnected', False))

    def on_write_result(self, ok: bool) -> None:
        # 命令链自身的 tick() 已转发写结果；这里只在外部端口另行观测时调用
        self.events.append(('write_result', bool(ok)))

    def on_operator_clear(self) -> None:
        self._chain.operator_clear()
        self.events.append(('operator_clear', True))

    def on_firmware_confirm(self, confirmed: bool) -> None:
        """**模拟确认**：仅用于测试仲裁器放行路径。

        真实固件没有确认字段（``buffer[2]`` 语义未知），本方法在生产路径
        没有调用点；调用它必须显式标注为模拟。
        """
        self._chain.firmware_confirm(bool(confirmed))
        self.events.append(('firmware_confirm(simulated)', bool(confirmed)))


def _assert_isolated(device: str) -> None:
    """放行两类目标：隔离伪终端 ``/dev/pts/N``，或真实 UART。

    真实 UART 的判据复用 :func:`~.serial_port.is_real_uart`；写权限由
    :class:`~.serial_port.FdPort` 的 ``fd_authorised`` 检查负责——未授权时
    端口对象根本构造不出来。其它路径仍然拒绝。
    """
    if isinstance(device, str) and PTS_PATTERN.match(device):
        return
    # by-path/by-id 符号链接先解析成真实设备路径再判：start_chassis.sh
    # 2026-11-14 起传 by-path 链接（抗 USB 重枚举换号），链接指向的仍是
    # /dev/ttyACM* 真实 UART；realpath 对普通路径是恒等变换。
    if is_real_uart(os.path.realpath(device)
                    if isinstance(device, str) else device):
        return
    raise NonIsolatedDeviceForbidden(
        f'{device!r} 既不是隔离伪终端（/dev/pts/N），'
        '也不是已授权的真实 UART（/dev/ttyACM*、/dev/ttyUSB* 等）。')


def build_isolated_transport(device: str | None = None, port=None):
    """工厂：返回可在测试或所有者进程内使用的写入端。

    - ``device is None`` → :class:`MemoryTransport`；
    - ``device`` 为 ``/dev/pts/N`` 或真实 UART → 需要注入 ``port``，返回
      :class:`OwnerSerialTransport`（真实 UART 的授权检查在
      :class:`~.serial_port.FdPort` 构造时已完成）；
    - 其它路径 → 抛 :class:`NonIsolatedDeviceForbidden`。

    本函数**从不**打开设备。
    """
    if device is None:
        return MemoryTransport()
    _assert_isolated(device)
    if port is None:
        raise ValueError(
            '目标需要注入 port 对象（隔离伪终端或所有者已打开的 fd）；'
            '本工厂不打开设备')
    return OwnerSerialTransport(port, device=device)
