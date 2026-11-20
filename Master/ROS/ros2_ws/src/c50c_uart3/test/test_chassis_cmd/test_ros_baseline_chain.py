"""生产链路 → 基线固件解析模型的端到端离线验证。

链路（全部是生产代码）::

    RosStraightInput.on_nav_cmd / on_stop        （ROS 消息适配层）
      → StraightCommandChain.submit_* / tick    （命令链）
        → Arbiter.evaluate                       （仲裁：限速/过期/停车/来源）
          → encode_baseline_speed_frame          （production 编码器）
            → OwnerSerialTransport.write_bytes   （唯一所有者适配器）
              → FakeFdPort.stream                （内存假端口，无真实 fd）
                → BaselineUartxModel.feed        （固件字节状态机的提取模型）

模型边界（与"已验证"区分）
==========================
1. ``fw_baseline_parse_model.py`` 是**按 ``uartx_callback.c`` 逐行复刻的
   Python 提取模型**。**它没有在真实硬件上运行**：本文件不连串口、不改
   DTR/RTS、不烧录、不部署、不启动电机、不发真实运动命令。构造模型时只核对
   固件源文件的 sha256。
2. 「主机写入成功」只说明字节进了内存假端口。本文件**不**把它写成固件已
   接收；「收到状态帧」同理。唯一可断言的是：**给定这些字节，固件的解析
   代码会算出什么**。
3. 模型未覆盖的固件行为见 ``fw_baseline_parse_model.UNMODELED``，并在
   :func:`test_unmodeled_behavior_list_is_explicit` 里逐条列出，避免读者
   误以为本文件证明了全部固件行为。
"""
from __future__ import annotations

import ast
import inspect

import pytest

from c50c_uart3.chassis_cmd.arbiter import (Arbiter, PROTOCOL_BASELINE,
                                            ZeroReason)
from c50c_uart3.chassis_cmd.baseline import encode_baseline_speed_frame
from c50c_uart3.chassis_cmd.serial_port import FakeFdPort
from c50c_uart3.chassis_cmd.transport import MemoryTransport
from c50c_uart3.frame_parser import FRAME_HEADER as STATUS_HEADER
from c50c_uart3.frame_parser import FRAME_LEN as STATUS_LEN
from c50c_uart3.frame_parser import FRAME_TAIL as STATUS_TAIL
from fw_baseline_parse_model import (BaselineUartxModel, UNMODELED,
                                     check_bcc,
                                     xyz_target_speed_transition)
from sim_confirm import (mark_chain_clean, mark_endpoint_clean, new_receipt,
                         grant_safety, TEST_SAFETY_MAX_AGE_S)

DEVICE = '/dev/pts/9'
_SIM = new_receipt('ros-baseline-chain')

#: 模型量化步长 1 mm/s。断言容差取它。
MM_S = 0.001


class Clock:
    """可控单调时钟：只在显式 ``advance`` 时前进。"""

    def __init__(self, t: float = 100.0) -> None:
        self.t = float(t)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> float:
        self.t += float(dt)
        return self.t


class _Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)


class _Twist:
    """``geometry_msgs/Twist`` 的鸭子类型替身。"""

    def __init__(self, vx=0.0, vy=0.0, wz=0.0):
        self.linear = _Vec(vx, vy, 0.0)
        self.angular = _Vec(0.0, 0.0, wz)


class _Bool:
    def __init__(self, data):
        self.data = data


def _status_frame() -> bytes:
    """48 字节基线状态帧（头/BCC/尾同 ``data_task.c``）。

    它只用来让仲裁器的基线证据门打开。产出它不需要连串口。
    """
    body = bytearray()
    body.append(STATUS_HEADER)
    body.append(0)
    body += b'\x00' * (STATUS_LEN - 4)   # 到 [45]，含编码器与 PWM 区
    bcc = 0
    for b in body:
        bcc ^= b
    body.append(bcc)                    # [46]
    body.append(STATUS_TAIL)            # [47]
    assert len(body) == STATUS_LEN
    return bytes(body)


class Chain:
    """生产链路 + 固件解析模型（纯内存，无真实设备）。

    用真实 ``Arbiter`` 与 ``StraightCommandChain``，把内存传输端作为
    ``StraightDriveBridge`` 的做法里那层所有者适配器，再把写出的字节原样
    喂给 :class:`BaselineUartxModel`。
    """

    def __init__(self, *, transport=None) -> None:
        self.clock = Clock()
        self.port = transport if transport is not None else MemoryTransport()
        self.arbiter = Arbiter(firmware_protocol=PROTOCOL_BASELINE,
                               safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
        grant_safety(self.arbiter, self.clock())
        from c50c_uart3.chassis_cmd.chain import StraightCommandChain
        self.chain = StraightCommandChain(self.arbiter, self.port)
        mark_endpoint_clean(self.port, receipt=_SIM)
        mark_chain_clean(self.chain, receipt=_SIM)
        from c50c_uart3.chassis_cmd.ros_input import RosStraightInput
        self.input = RosStraightInput(self.chain, enabled=True,
                                      clock=self.clock)
        self.fw = BaselineUartxModel(now_ms=2000)
        self.frames: list[dict] = []
        # 声明来源（``ros_input.on_cycle`` 要求发布者数已知）
        self.input.set_nav_publishers(1)
        self.input.set_demo_publishers(0)

    # ---- 生产链路入口 ----
    def status_evidence(self) -> None:
        """打开基线证据门（模拟主机读到一帧状态帧，**不是**固件确认）。"""
        self.arbiter.on_baseline_status_frame(self.clock())

    def nav(self, vx: float, vz: float = 0.0) -> bool:
        return self.input.on_nav_cmd(_Twist(vx, 0.0, vz))

    def stop(self, active: bool) -> bool:
        return self.input.on_stop(_Bool(active))

    def publishers(self, nav: int, demo: int = 0) -> None:
        self.input.set_nav_publishers(nav)
        self.input.set_demo_publishers(demo)

    def tick(self, dt: float = 0.02) -> dict:
        """一拍：推进时钟 → 适配层周期 → 把新写出的字节喂进固件模型。"""
        self.clock.advance(dt)
        before = len(self.port.writes)
        gated, record = self.input.on_cycle(
            lambda: self.chain.tick(self.clock()))
        new = b''.join(self.port.writes[before:])
        fw_before = self.fw.accepted_speed_frames
        if new:
            self.fw.feed(new)
        item = {
            'gated': gated,
            'record': record,
            'bytes': new,
            'fw_accepted': self.fw.accepted_speed_frames - fw_before,
        }
        self.frames.append(item)
        return item

    # ---- 观测 ----
    def nonzero(self) -> list[bytes]:
        return [f['bytes'] for f in self.frames
                if f['bytes'] and f['record'] is not None
                and not f['record'].is_zero]

    def zeros(self) -> list[bytes]:
        return [f['bytes'] for f in self.frames
                if f['bytes'] and f['record'] is not None
                and f['record'].is_zero]

    def reason(self, index: int = -1):
        rec = self.frames[index]['record']
        return None if rec is None else rec.zero_reason


@pytest.fixture
def chain():
    c = Chain()
    yield c
    c.input.disable()


def _armed(c: Chain, n: int = 2) -> None:
    """让链路进入"已发过零速帧"的稳态。

    基线非零需要状态帧证据在时效内，且本授权周期已写出过一帧零速。
    """
    c.status_evidence()
    c.nav(0.0)
    for _ in range(n):
        c.tick()


# =====================================================================
# 0. 模型自身的自证：公式与常量逐一对应源码
# =====================================================================
def test_model_source_hashes_match_recorded_firmware():
    """模型依赖的四个固件源文件哈希与记录一致（固件一改就报错）。"""
    from fw_baseline_parse_model import verify_sources
    seen = verify_sources()
    assert len(seen) == 4
    assert all(len(h) == 64 for h in seen.values())


def test_unmodeled_behavior_list_is_explicit():
    """未覆盖项必须显式列出，不能靠读者猜。"""
    text = ' '.join(UNMODELED)
    for keyword in ('中断时序', 'CHASSIS_PWM_TEST', 'AKM_CAR',
                    'command_lostcount', 'ResponseControl',
                    'ros_cmd_tick', '物理层'):
        assert keyword in text, f'未覆盖项清单缺少 {keyword}'
    assert len(UNMODELED) >= 8


def test_xyz_transition_matches_c_truncation_semantics():
    """``XYZ_Target_Speed_transition`` 是 ``short`` 截断 + 向零取整。"""
    assert xyz_target_speed_transition(0x00, 0x00) == 0.0
    assert xyz_target_speed_transition(0x00, 0x32) == pytest.approx(0.05)
    assert xyz_target_speed_transition(0xFF, 0xCE) == pytest.approx(-0.05)
    # -1 毫单位：C 的 / 与 % 向零截断 → 0 + (-1)*0.001 = -0.001
    assert xyz_target_speed_transition(0xFF, 0xFF) == pytest.approx(-0.001)
    assert xyz_target_speed_transition(0x7F, 0xFF) == pytest.approx(32.767)
    assert xyz_target_speed_transition(0x80, 0x00) == pytest.approx(-32.768)


def test_model_rejects_wrong_tail_and_wrong_bcc():
    """帧尾 / BCC 任一不成立 → 丢弃，不改三轴（``uartx_callback.c:56-58``）。"""
    good = encode_baseline_speed_frame(0.1, 0.0)
    bad_tail = bytearray(good)
    bad_tail[10] = 0x00
    bad_bcc = bytearray(good)
    bad_bcc[9] ^= 0xFF

    m = BaselineUartxModel()
    m.feed(bytes(bad_tail))
    assert (m.vx, m.rejected_tail, m.accepted_speed_frames) == (0.0, 1, 0)

    m = BaselineUartxModel()
    m.feed(bytes(bad_bcc))
    assert m.vx == 0.0
    assert m.rejected_bcc == 1
    assert m.accepted_speed_frames == 0


def test_model_resyncs_after_garbage_bytes():
    """垃圾字节后仍能重新对齐到帧头（真实状态机的 ``Count`` 复位语义）。"""
    m = BaselineUartxModel()
    m.feed(b'\x01\x02\x03\x04')
    m.feed(encode_baseline_speed_frame(0.07, 0.0))
    assert m.accepted_speed_frames == 1
    assert m.vx == pytest.approx(0.07, abs=MM_S)


def test_model_drops_bytes_during_startup_delay():
    """``SysVal.Time_count < CONTROL_DELAY`` 时中断直接返回（丢字节）。"""
    m = BaselineUartxModel()
    m.time_count = 0
    m.feed(encode_baseline_speed_frame(0.1, 0.0))
    assert m.accepted_speed_frames == 0
    assert m.dropped_by_startup_delay == 11


# =====================================================================
# 1. 非零直行：生产链路字节 → 固件解析模型三轴一致
# =====================================================================
@pytest.mark.parametrize('vx', [0.05, 0.10, -0.10, 0.037])
def test_nonzero_straight_reaches_firmware_axes(chain, vx):
    """发布 vx → 生产链路写出 → 固件模型解出同量级 Vx（误差 ≤ 1 mm/s）。"""
    _armed(chain)
    chain.nav(vx)
    f = chain.tick()
    assert f['fw_accepted'] == 1, f'本拍字节 {f["bytes"].hex()}'

    assert chain.fw.vx == pytest.approx(vx, abs=MM_S)
    assert chain.fw.vy == pytest.approx(0.0, abs=MM_S)
    assert chain.fw.vz == pytest.approx(0.0, abs=MM_S)
    # 固件侧模式被切到 _ROS_Control（uartx_callback.c:94）
    assert chain.fw.control_mode == 1
    assert chain.fw.allow_recharge == 0


def test_turn_in_place_maps_vz_only(chain):
    """纯角速度命令只改 Vz，Vx 保持零。"""
    _armed(chain)
    chain.nav(0.0, 0.25)
    chain.tick()
    assert chain.fw.vz == pytest.approx(0.25, abs=MM_S)
    assert chain.fw.vx == pytest.approx(0.0, abs=MM_S)


def test_written_frame_is_the_production_encoder_output(chain):
    """链路写出的字节与 ``encode_baseline_speed_frame`` 逐字节一致。"""
    _armed(chain)
    chain.nav(0.08)
    chain.tick()
    written = chain.nonzero()[-1]
    assert written == encode_baseline_speed_frame(0.08, 0.0)
    assert written[1] == 0 and written[2] == 0 and written[10] == 0x7D
    assert written[9] == check_bcc(written[:9])


def test_negative_vz_is_encoded_signed_in_firmware(chain):
    """反向角速度在固件模型里解出负 Vz（int16 补码经 C 语义转换）。"""
    _armed(chain)
    chain.nav(0.0, -0.30)
    chain.tick()
    assert chain.fw.vz == pytest.approx(-0.30, abs=MM_S)


# =====================================================================
# 2. 零速：命令为零时固件模型三轴全零
# =====================================================================
def test_zero_command_yields_zero_axes_in_firmware(chain):
    _armed(chain)
    chain.nav(0.0)
    chain.tick()
    assert chain.fw.vx == 0.0 and chain.fw.vy == 0.0 and chain.fw.vz == 0.0
    assert chain.zeros(), '零速命令应产出零速帧'


# =====================================================================
# 3. nav_stop：停车后固件模型不再收到非零
# =====================================================================
def test_nav_stop_prevents_any_nonzero_from_reaching_firmware(chain):
    """停车期间固件模型只收到**零速**速度帧；三轴恒为零。"""
    _armed(chain)
    chain.nav(0.10)
    chain.tick()
    assert chain.fw.vx == pytest.approx(0.10, abs=MM_S)
    already = len(chain.nonzero())

    assert chain.stop(True) is True

    for _ in range(5):
        chain.nav(0.10)          # 停车期间持续发布非零
        f = chain.tick()
        assert f['record'] is not None
        assert f['record'].is_zero, '停车期间只允许零速帧'
        assert f['record'].zero_reason is ZeroReason.STOP_REQUESTED
        assert f['bytes'] == encode_baseline_speed_frame(0.0, 0.0)
        assert chain.fw.vx == pytest.approx(0.0, abs=MM_S), \
            '固件模型的 Vx 只能被零速帧写成 0'
    assert len(chain.nonzero()) == already, '停车期间不得新增非零帧'

    # 解除停车后先发零速，再恢复非零
    chain.stop(False)
    chain.tick()
    chain.nav(0.10)
    chain.tick()
    assert chain.fw.vx == pytest.approx(0.10, abs=MM_S)

    # 解除停车后先发零速，再恢复非零
    chain.stop(False)
    chain.tick()
    chain.nav(0.10)
    chain.tick()
    assert chain.fw.vx == pytest.approx(0.10, abs=MM_S)


# =====================================================================
# 4. 来源不唯一：多个发布者 → 固件模型只看到零速
# =====================================================================
def test_ambiguous_sources_keep_firmware_at_zero(chain):
    """适配层判来源不可放行时保持静默，仲裁器层面改判零速。

    两层防守都要验：
    1. ``ros_input`` 看到 2 个发布者 → 不驱动命令链（``gated``，零字节）；
    2. 即便直接给仲裁器一个非零候选（这里用链上原语模拟"绕过适配层"），
       仲裁器也返回 ``AMBIGUOUS_SOURCES`` 零速帧，固件三轴不变。
    """
    _armed(chain)
    chain.publishers(2)              # 同一话题两个发布者
    chain.nav(0.10)
    gated = chain.tick()
    assert gated['gated'] is True
    assert gated['bytes'] == b''
    assert chain.fw.vx == pytest.approx(0.0, abs=MM_S)
    assert not chain.nonzero()

    # 仲裁器层面的第二道门：直接注入非零候选
    chain.publishers(1)
    chain.clock.advance(0.02)
    chain.chain.submit_nav(0.10, 0.0, 0.0, chain.clock())
    chain.chain.set_sources(2, 0)    # 只给仲裁器看，跳过适配层门控
    rec = chain.chain.tick(chain.clock())
    assert rec.is_zero
    assert rec.zero_reason is ZeroReason.AMBIGUOUS_SOURCES
    assert rec.frame is not None
    assert rec.frame == encode_baseline_speed_frame(0.0, 0.0)
    chain.fw.feed(rec.frame)
    assert chain.fw.vx == pytest.approx(0.0, abs=MM_S)


# =====================================================================
# 5. 状态帧缺失 / 过期
# =====================================================================
def test_without_status_evidence_no_nonzero_frame_is_written(chain):
    """没有状态帧证据时仲裁器只给零速并标 ``NO_SESSION``；绝不写非零。"""
    chain.publishers(1)
    chain.nav(0.10)
    for _ in range(4):
        f = chain.tick()
        assert f['record'] is not None
        assert f['record'].is_zero
        assert f['record'].zero_reason is ZeroReason.NO_SESSION
        if f['bytes']:
            assert f['bytes'] == encode_baseline_speed_frame(0.0, 0.0)
    assert not chain.nonzero()
    assert chain.fw.vx == 0.0


def test_status_evidence_expiry_downgrades_to_zero(chain):
    """状态帧证据过期（> 3 s）后非零被降级为 ``SESSION_STALE`` 零速。"""
    _armed(chain)
    chain.nav(0.10)
    chain.tick()
    assert chain.fw.vx == pytest.approx(0.10, abs=MM_S)

    chain.clock.advance(4.0)         # 越过 baseline 3 s 窗口
    chain.nav(0.10)
    f = chain.tick()
    assert f['record'].is_zero
    assert f['record'].zero_reason is ZeroReason.SESSION_STALE
    assert f['fw_accepted'] == 1     # 零速帧本身仍是合法速度帧
    assert chain.fw.vx == pytest.approx(0.0, abs=MM_S)


# =====================================================================
# 6. 链路首帧：先零后非零（固件授权帧）
# =====================================================================
def test_first_frame_is_zero_before_any_nonzero(chain):
    """生产链路的第一条输出必须是零速帧，之后才可能非零。"""
    chain.status_evidence()
    chain.nav(0.10)
    first = chain.tick()
    assert first['record'].is_zero
    assert first['fw_accepted'] == 1
    assert chain.fw.vx == 0.0

    second = chain.tick()
    assert not second['record'].is_zero
    assert chain.fw.vx == pytest.approx(0.10, abs=MM_S)


# =====================================================================
# 7. 证据边界：本文件没有证明的东西
# =====================================================================
def test_model_does_not_claim_hardware_execution():
    """模型模块的文档必须写明"逐行复刻""未在真实硬件上运行"与"不证明接收"。"""
    import fw_baseline_parse_model as model
    doc = model.__doc__ or ''
    assert '逐行复刻' in doc
    assert '未在真实硬件上运行' in doc
    assert '证明固件收到' in doc


def test_this_test_has_no_real_serial_open_point():
    """本测试只用 ``FakeFdPort`` 与 ``MemoryTransport``：不打开设备节点。

    用 ``ast`` 检查真实 import 与调用，避免文档字符串里的同名文字误报。
    """
    tree = ast.parse(inspect.getsource(inspect.getmodule(_status_frame)))
    banned_modules = {'serial', 'termios', 'fcntl', 'tty'}
    banned_calls = {'open', 'fdopen'}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                assert alias.name.split('.')[0] not in banned_modules, \
                    f'不得 import {alias.name}'
        elif isinstance(node, ast.ImportFrom):
            assert (node.module or '').split('.')[0] not in banned_modules, \
                f'不得 from {node.module} import'
        elif isinstance(node, ast.Call):
            func = node.func
            name = getattr(func, 'id', None) or getattr(func, 'attr', None)
            assert name not in banned_calls, f'不得调用 {name}()'
        elif isinstance(node, ast.Attribute):
            assert node.attr not in {'Serial', 'open_port'}, \
                f'不得引用 {node.attr}'
