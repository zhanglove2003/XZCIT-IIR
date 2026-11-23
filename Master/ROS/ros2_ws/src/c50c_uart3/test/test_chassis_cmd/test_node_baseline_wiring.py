"""节点侧最小接线：``drive_firmware_protocol`` 开关 + 状态帧通知。

本文件只验证两件事，对应任务的两条目标：

1. ``C50CUart3Node`` 构造直行命令桥时**显式**选择基线协议；不传参数时
   仍是 v3（既存行为不变），非法值在启动时直接拒绝。
2. 节点收到 48 字节状态帧后调用既有的 ``notify_status_frame()``，使基线
   证据门可用。

证据边界（与 ``_notify_drive_status_frame`` 的 docstring 一致）：
状态帧只证明**主机在线上读到了格式合法的一帧基线状态帧**。它不证明板上
固件版本、不证明主机→固件方向已对齐、也不证明固件收到或执行了主机的任何
命令——基线不回复确认。写入成功不是接收确认。

不打开真实串口，不碰 DTR/RTS，不连串口，不烧录。
"""
from __future__ import annotations

import ast
import pathlib
import sys
import types

import pytest

from c50c_uart3.chassis_cmd.arbiter import (FIRMWARE_PROTOCOLS,
                                            PROTOCOL_BASELINE, PROTOCOL_V3)
from c50c_uart3.chassis_cmd.bridge import (DEFAULT_FIRMWARE_PROTOCOL,
                                           StraightDriveBridge)
from c50c_uart3.chassis_cmd.serial_port import FakeFdPort
from c50c_uart3.frame_parser import FRAME_HEADER, FRAME_LEN, FRAME_TAIL
from sim_confirm import mark_chain_clean, mark_endpoint_clean, new_receipt

_ROOT = pathlib.Path(__file__).resolve().parents[2]     # …/c50c_uart3
_PROD = _ROOT / 'c50c_uart3'
DEVICE = '/dev/pts/9'
_SIM = new_receipt('node-baseline-wiring')


class Clock:
    def __init__(self, t: float = 100.0) -> None:
        self.t = float(t)

    def __call__(self) -> float:
        return self.t


def _import_uart3_node(monkeypatch):
    """无 ROS 环境导入节点模块：只替换外部依赖，节点源码不动。"""

    def mod(name, **attrs):
        m = sys.modules.get(name) or types.ModuleType(name)
        m.__path__ = getattr(m, '__path__', [])
        # 若目标模块是真实包（例如 c50c_odom.odometry），覆盖属性必须登记到
        # monkeypatch，否则本测试结束后真实模块仍留着 object 桩，后续用例
        # 构造 StepReport 就会 TypeError: object() takes no arguments。
        # 新建的桩模块整条塞进 sys.modules，同样要在用例结束时还原。
        created = name not in sys.modules
        for k, v in attrs.items():
            if not created and hasattr(m, k):
                monkeypatch.setattr(m, k, v, raising=False)
            else:
                setattr(m, k, v)
        if created:
            monkeypatch.setitem(sys.modules, name, m)
        sys.modules[name] = m
        return m

    class _Node:
        def __init__(self, *a, **k):
            pass

    mod('rclpy')
    mod('rclpy.node', Node=_Node)
    mod('rclpy.qos', qos_profile_sensor_data=object())
    mod('rclpy.signals', SignalHandlerOptions=type('S', (), {}))
    mod('geometry_msgs')
    mod('geometry_msgs.msg', TransformStamped=object,
        Twist=type('Twist', (), {}))
    mod('std_msgs')
    mod('std_msgs.msg', Bool=type('Bool', (), {}))
    mod('nav_msgs')
    mod('nav_msgs.msg', Odometry=object)
    mod('sensor_msgs')
    mod('sensor_msgs.msg', Imu=object)
    mod('diagnostic_msgs')
    mod('diagnostic_msgs.msg', DiagnosticArray=object, DiagnosticStatus=object, KeyValue=object)
    mod('tf2_ros', TransformBroadcaster=object)
    # c50c_odom.odometry 是真实模块（本机可用），先导入再打桩，
    # 这样打桩落在真模块上、由 monkeypatch 在用例结束时还原；
    # 若不先导入，这里会凭空造一个同名桩留在 sys.modules 里。
    import c50c_odom.odometry as _real_odom  # noqa: F401
    mod('c50c_odom')
    mod('c50c_odom.odometry', PlanarOdomIntegrator=object, StepReport=object,
        twist_from_parsed=lambda *a, **k: None)
    import importlib

    return importlib.import_module('c50c_uart3.uart3_node')


def _bind_notify(uart3_node) -> None:
    """把真实 ``_notify_drive_status_frame`` 绑到替身上（不复制实现）。"""
    _NodeCtx._notify_drive_status_frame = (
        uart3_node.C50CUart3Node._notify_drive_status_frame)


def _status_frame(vx_mm_s: int = 0) -> bytes:
    """组一帧 48 字节基线状态帧（头/BCC/尾同 ``data_task.c``）。"""
    body = bytearray()
    body.append(FRAME_HEADER)
    body.append(0)                                   # buffer[1] stop 标志
    body += int(vx_mm_s).to_bytes(2, 'big', signed=True)
    body += b'\x00' * 18                             # 其余 9 个 16 位字段，到 [21]
    body += b'\x00' * 16                             # [22..37] 四轮编码器 int32
    body += b'\x00' * 8                              # [38..45] 四轮 PWM uint16
    assert len(body) == 46
    bcc = 0
    for b in body:
        bcc ^= b
    body.append(bcc)                                  # [46]
    body.append(FRAME_TAIL)                           # [47]
    assert len(body) == FRAME_LEN
    return bytes(body)


def _baseline_bridge(clock):
    port = FakeFdPort()
    bridge = StraightDriveBridge(port, device=DEVICE, enabled=True,
                                 firmware_protocol=PROTOCOL_BASELINE,
                                 clock=clock)
    mark_endpoint_clean(port, receipt=_SIM)
    mark_chain_clean(bridge.chain, receipt=_SIM)
    return bridge, port


# ------------------------------------------------------------------ 静态契约
def test_default_protocol_keeps_v3_and_is_declared_as_parameter():
    """默认值是 v3；节点用同一个常量声明 ROS 参数（不给就保持 v3）。"""
    assert DEFAULT_FIRMWARE_PROTOCOL == PROTOCOL_V3
    assert PROTOCOL_BASELINE in FIRMWARE_PROTOCOLS

    tree = ast.parse((_PROD / 'uart3_node.py').read_text(encoding='utf-8'))
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and _callee(n) == 'declare_parameter']
    decl = [c for c in calls
            if c.args and isinstance(c.args[0], ast.Constant)
            and c.args[0].value == 'drive_firmware_protocol']
    assert decl, '节点必须声明 drive_firmware_protocol 参数'
    default = decl[0].args[1] if len(decl[0].args) > 1 else None
    assert isinstance(default, ast.Name) and default.id == 'DEFAULT_FIRMWARE_PROTOCOL', (
        '参数默认值必须直接取自 DEFAULT_FIRMWARE_PROTOCOL，避免第二处默认值')


def test_startup_rejects_unknown_protocol():
    """非法协议值在启动时直接报错，不回退、不兜底。"""
    tree = ast.parse((_PROD / 'uart3_node.py').read_text(encoding='utf-8'))
    inits = [n for n in ast.walk(tree)
             if isinstance(n, ast.FunctionDef) and n.name == '__init__']
    assert inits, '节点必须有 __init__'
    text = ast.dump(inits[0])
    assert 'FIRMWARE_PROTOCOLS' in text, '启动时必须校验协议名'
    assert 'RuntimeError' in text, '非法协议值必须抛 RuntimeError（不回退）'
    assert 'NotIn' in text, '校验形式应为「不在允许集合内即拒绝」'


def _callee(call) -> str:
    fn = call.func
    if isinstance(fn, ast.Name):
        return fn.id
    if isinstance(fn, ast.Attribute):
        return fn.attr
    return ''


# ------------------------------------------------------------------ 选协议
def test_baseline_selection_builds_baseline_bridge_without_handshake(
        monkeypatch):
    """选 baseline：桥按基线方言建，且**不建**会话握手。"""
    uart3_node = _import_uart3_node(monkeypatch)
    import c50c_uart3.chassis_cmd.serial_port as serial_port_mod

    port = FakeFdPort()
    warns = []

    class _PortFactory:
        class RealFdUseForbidden(Exception):
            pass

        def __call__(self, fd, device, *, fd_authorised=False):
            assert fd == 7 and device == DEVICE
            assert fd_authorised is True
            return port

    monkeypatch.setattr(serial_port_mod, 'FdPort', _PortFactory())
    node = _NodeCtx(protocol=PROTOCOL_BASELINE)
    node.warns = warns
    uart3_node.C50CUart3Node._attach_drive_bridge(node)

    assert node._drive_bridge is not None
    assert node._drive_bridge.firmware_protocol == PROTOCOL_BASELINE
    assert node._drive_handshake is None, '基线没有会话帧，不得建握手'
    assert len(node.timers) == 1 and len(node.subs) == 3
    assert any('基线方言' in w for w in warns), '必须记录基线方言的降级说明'


def test_v3_selection_keeps_session_handshake(monkeypatch):
    """默认 v3：行为与改动前一致，仍建会话握手。"""
    uart3_node = _import_uart3_node(monkeypatch)
    import c50c_uart3.chassis_cmd.serial_port as serial_port_mod

    port = FakeFdPort()

    class _PortFactory:
        class RealFdUseForbidden(Exception):
            pass

        def __call__(self, fd, device, *, fd_authorised=False):
            return port

    monkeypatch.setattr(serial_port_mod, 'FdPort', _PortFactory())
    node = _NodeCtx(protocol=PROTOCOL_V3)
    uart3_node.C50CUart3Node._attach_drive_bridge(node)

    assert node._drive_bridge.firmware_protocol == PROTOCOL_V3
    assert node._drive_handshake is not None
    assert node._drive_handshake._receipt_sink is port


# --------------------------------------------------------------- 状态帧通知
def test_status_frame_notification_opens_baseline_window(monkeypatch):
    """收到状态帧 → 调既有通知 → 基线证据窗口打开。"""
    uart3_node = _import_uart3_node(monkeypatch)
    _bind_notify(uart3_node)
    clock = Clock()
    bridge, _port = _baseline_bridge(clock)
    node = _NodeCtx(protocol=PROTOCOL_BASELINE)
    node._drive_bridge = bridge

    assert bridge.chain._arbiter.baseline_window_open(clock()) is False
    uart3_node.C50CUart3Node._notify_drive_status_frame(node)
    assert bridge.chain._arbiter.baseline_window_open(clock()) is True


def test_status_frame_notification_is_noop_under_v3(monkeypatch):
    """v3 下不得把状态帧当会话证据：什么也不做，且不抛错。"""
    uart3_node = _import_uart3_node(monkeypatch)
    _bind_notify(uart3_node)
    clock = Clock()
    port = FakeFdPort()
    bridge = StraightDriveBridge(port, device=DEVICE, enabled=True,
                                 firmware_protocol=PROTOCOL_V3, clock=clock)
    node = _NodeCtx(protocol=PROTOCOL_V3)
    node._drive_bridge = bridge

    uart3_node.C50CUart3Node._notify_drive_status_frame(node)
    assert bridge.chain._arbiter.baseline_window_open(clock()) is False


def test_status_frame_notification_tolerates_missing_bridge(monkeypatch):
    """桥未挂载（drive_enable 关闭）时通知静默返回。"""
    uart3_node = _import_uart3_node(monkeypatch)
    _bind_notify(uart3_node)
    node = _NodeCtx(protocol=PROTOCOL_BASELINE)
    node._drive_bridge = None
    uart3_node.C50CUart3Node._notify_drive_status_frame(node)


def test_handle_frame_path_notifies_bridge(monkeypatch):
    """整条收帧路径：解析器吐出 48 字节帧 → ``_handle_frame`` → 证据窗口打开。"""
    uart3_node = _import_uart3_node(monkeypatch)
    _bind_notify(uart3_node)
    from c50c_uart3.frame_parser import StatusFrameParser

    clock = Clock()
    bridge, _port = _baseline_bridge(clock)
    node = _NodeCtx(protocol=PROTOCOL_BASELINE)
    node._drive_bridge = bridge
    node._odom_candidate = None

    frames = StatusFrameParser().feed(_status_frame())
    assert len(frames) == 1, '构造的状态帧应被解析器接受'
    uart3_node.C50CUart3Node._handle_frame(node, frames[0])

    assert bridge.chain._arbiter.baseline_window_open(clock()) is True


class _NodeCtx:
    """节点替身：只提供接线用到的接口，方法本体用真实节点函数。"""

    def __init__(self, *, protocol: str) -> None:
        self.timers, self.subs = [], []
        self.warns = []
        self._drive_enabled = True
        self._drive_input_enable = True
        self._drive_tick_period_s = 0.02
        self._drive_fd_authorised = True
        self._drive_firmware_protocol = protocol
        self._fd = 7
        self._drive_port = None
        self._drive_bridge = None
        self._drive_handshake = None
        self._drive_wiring = None
        self._device = DEVICE
        self._last_frame_monotonic = None
        self._frame_gap_warn_s = 0.25
        self._odom_candidate = None

    def _drive_cycle(self):
        return None

    def _publish_imu(self, frame, stamp):
        return None

    def _publish_chassis_diagnostics(self, frame, stamp):
        return None

    def get_logger(self):
        warns = self.warns

        def warn(msg, *a, **k):
            warns.append(str(msg))

        return types.SimpleNamespace(warn=warn,
                                     error=lambda *a, **k: None)

    def get_clock(self):
        class _Now:
            @staticmethod
            def to_msg():
                return object()

        return types.SimpleNamespace(now=lambda: _Now())

    def create_timer(self, period, callback):
        timer = types.SimpleNamespace(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = types.SimpleNamespace(msg_type=msg_type, topic=topic,
                                    callback=callback, qos=qos)
        self.subs.append(sub)
        return sub

    def resolve_topic_name(self, name):
        return name


