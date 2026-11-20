"""雷达端口选择与启动路径测试。

覆盖：
- 0 个候选保持空、状态 missing；
- 1 个候选自动使用；
- 2 个候选拒绝盲选，日志含候选列表；
- 显式 lidar_port 不扫描；
- 启动无设备、之后出现，下一次重连能拿到路径并打开；
- 其他 USB 串口同时存在时不误选；
- launch 默认路径是现场实测值。

按教训：用 ``types.SimpleNamespace`` 替换 ``LN.glob`` 与 ``LN.os``，
不整体替换 ``os.path``。
"""

import ast
import os
import types

import pytest

import fake_ros
import lidar_fixtures
from opi_robot.lidar_node import lidar_node as LN

LIDAR_PORT = '/dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0'
OTHER_PORT = '/dev/serial/by-path/platform-fc8c0000.usb-usb-0:2:1.0-port0'
TTYUSB = '/dev/serial/by-path/platform-fc8c0000.usb-usb-0:3:1.0-port0'
CAM_BY_ID = ('/dev/v4l/by-id/usb-HD_720P_Webcam_HD_720P_Webcam_20130901'
             '-video-index0')
EMPTY_BYTES = b''

LAUNCH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__))), 'launch', 'robot.launch.py')


# 共享实例表放在模块级；每个用例在 fixture 里清空，避免跨用例污染。
SERIALS = []


class FakeSerial:
    def __init__(self, path, baud):
        self.path = path
        self.baud = baud
        self.chunks = bytearray()
        self.closed = False
        SERIALS.append(self)

    def read(self, n=4096):
        if not self.chunks:
            return EMPTY_BYTES
        data = bytes(self.chunks[:n])
        del self.chunks[:n]
        return data

    def close(self):
        self.closed = True


@pytest.fixture
def world(monkeypatch):
    del SERIALS[:]
    w = World()
    w.install(monkeypatch)
    return w


class World:
    """伪造的设备文件系统。候选列表可随时改变，模拟热插拔。"""

    def __init__(self, candidates=(), existing=None):
        self.candidates = list(candidates)
        self.existing = set(existing if existing is not None else candidates)
        self.glob_calls = []
        self.exists_calls = []

    def glob(self, pattern):
        self.glob_calls.append(pattern)
        return list(self.candidates)

    def exists(self, path):
        self.exists_calls.append(path)
        return path in self.existing

    def install(self, monkeypatch):
        monkeypatch.setattr(LN, '_open_serial',
                            lambda p, b: FakeSerial(p, b))
        monkeypatch.setattr(LN, 'glob',
                            types.SimpleNamespace(glob=self.glob))
        monkeypatch.setattr(
            LN, 'os', types.SimpleNamespace(
                path=types.SimpleNamespace(exists=self.exists)))
        # 教训：整体替换 os.path 会递归，这里只替换 glob 与 os.path.exists。
        # 单调时钟不在这里替换：conftest 已把 reader.time 换成可控替身，
        # 推进热插拔用 lidar_fixtures.advance。
        monkeypatch.setattr(LN, 'time', types.SimpleNamespace(
            sleep=lambda s: None, monotonic=lidar_fixtures.monotonic))


def tick(seconds):
    """推进单调时钟。设备树重扫与重连节流都看它。"""
    lidar_fixtures.advance(seconds)


def make_node(monkeypatch, port=''):
    """构造节点并显式设置 lidar_port（fake_ros 无法在构造前声明参数）。

    conftest 的 autouse fixture 已把单调时钟归零，这里只管参数注入。
    """
    orig = LN.LidarNode.__init__

    def patched(self):
        orig(self)
        if port:
            # Node.__init__ 之后 _params 才存在，此时覆写与 launch 传参等价
            self.set_param('lidar_port', port)
            self.explicit_port = port
            self.port_status = 'explicit'
            self.runner.port = port

    monkeypatch.setattr(LN.LidarNode, '__init__', patched)
    try:
        return LN.LidarNode()
    finally:
        monkeypatch.setattr(LN.LidarNode, '__init__', orig)


def errors(node):
    return [t for lv, t in node.get_logger().lines if lv == 'error']


def diag_kv(node, key):
    node.publish_diag()
    status = node.publishers['diagnostics'].messages[-1].status[0]
    for item in status.values:
        if item.key == key:
            return item.value
    raise KeyError(key)


class TestResolvePort:
    def test_zero_candidates(self):
        assert LN._resolve_port([]) == ('', 'missing')

    def test_one_candidate(self):
        assert LN._resolve_port([LIDAR_PORT]) == (LIDAR_PORT, 'unique')

    def test_two_candidates_refuses(self):
        port, status = LN._resolve_port([LIDAR_PORT, OTHER_PORT])
        assert port == ''
        assert status == 'ambiguous'

    def test_default_pattern_never_picks_first(self):
        owned = [LIDAR_PORT, OTHER_PORT]
        # 用一个假的 glob 源验证，避免依赖本机设备树
        saved = LN.glob
        try:
            LN.glob = types.SimpleNamespace(glob=lambda pat: owned)
            assert LN._default_port_pattern() == ''
        finally:
            LN.glob = saved


class TestZeroCandidates:
    def test_keeps_empty_and_missing(self, world, monkeypatch):
        node = make_node(monkeypatch)
        assert node.runner.port == ''
        assert node.port_status == 'missing'
        node.on_poll()
        assert node.runner.port == ''
        assert node.port_status == 'missing'
        assert diag_kv(node, 'port') == ''
        assert diag_kv(node, 'port_status') == 'missing'
        assert diag_kv(node, 'state') != 'healthy'
        assert not SERIALS

    def test_logs_missing_once(self, world, monkeypatch):
        node = make_node(monkeypatch)
        for _ in range(5):
            node.on_poll()
        joined = '\n'.join(errors(node))
        assert '未找到 USB 串口' in joined


class TestOneCandidate:
    def test_uses_it(self, world, monkeypatch):
        """唯一候选：第一次 poll 就选定并使用。"""
        world.candidates = [LIDAR_PORT]
        node = make_node(monkeypatch)
        node.on_poll()
        assert node.runner.port == LIDAR_PORT
        assert node.port_status == 'scanning'
        joined = '\n'.join(node.get_logger().texts())
        assert '雷达串口自动选定' in joined

    def test_opens_serial(self, world, monkeypatch):
        world.candidates = [LIDAR_PORT]
        world.existing = {LIDAR_PORT}
        node = make_node(monkeypatch)
        node.on_poll()
        assert SERIALS
        assert SERIALS[-1].path == LIDAR_PORT


class TestTwoCandidates:
    def test_refuses_blind_pick(self, world, monkeypatch):
        world.candidates = [LIDAR_PORT, OTHER_PORT]
        node = make_node(monkeypatch)
        node.on_poll()
        assert node.port_status == 'ambiguous'
        assert node.runner.port == ''
        node.on_poll()
        assert node.runner.port == ''
        assert not SERIALS, '多候选时不得打开任何一个'

    def test_log_lists_candidates(self, world, monkeypatch):
        world.candidates = [LIDAR_PORT, OTHER_PORT]
        node = make_node(monkeypatch)
        node.on_poll()
        joined = '\n'.join(errors(node))
        assert '拒绝盲选' in joined
        assert LIDAR_PORT in joined
        assert OTHER_PORT in joined

    def test_diag_reports_ambiguous(self, world, monkeypatch):
        world.candidates = [LIDAR_PORT, OTHER_PORT]
        node = make_node(monkeypatch)
        node.on_poll()
        node.publish_diag()
        status = node.publishers['diagnostics'].messages[-1].status[0]
        values = {kv.key: kv.value for kv in status.values}
        assert values['port_status'] == 'ambiguous'
        assert values['port'] == ''
        assert values['connected'] == '0'


class TestExplicitPort:
    def test_does_not_scan(self, world, monkeypatch):
        world.candidates = [LIDAR_PORT]
        node = make_node(monkeypatch, port=OTHER_PORT)
        for _ in range(3):
            node.on_poll()
        assert world.glob_calls == [], '显式端口时不得扫描'
        assert node.runner.port == OTHER_PORT

    def test_explicit_used_even_when_zero_candidates(self, world, monkeypatch):
        world.candidates = []
        world.existing = {OTHER_PORT}
        node = make_node(monkeypatch, port=OTHER_PORT)
        node.on_poll()
        assert node.runner.port == OTHER_PORT
        assert SERIALS
        assert SERIALS[-1].path == OTHER_PORT
        assert node.port_status == 'explicit'


class TestHotPlug:
    def test_device_appears_later(self, world, monkeypatch):
        """启动时无设备，之后设备出现：下一次重连能拿到路径并打开。"""
        node = make_node(monkeypatch)
        assert node.runner.port == ''
        node.on_poll()
        tick(2.0)
        node.get_clock().advance(2.0)
        node.on_poll()
        assert not SERIALS

        # 设备插上
        world.candidates = [LIDAR_PORT]
        world.existing = {LIDAR_PORT}
        tick(2.0)
        node.get_clock().advance(2.0)
        node.on_poll()

        assert node.runner.port == LIDAR_PORT, '设备出现后必须拿到路径'
        assert SERIALS, '设备出现后必须打开串口'
        assert SERIALS[-1].path == LIDAR_PORT

    def test_ambiguous_then_disambiguated(self, world, monkeypatch):
        world.candidates = [LIDAR_PORT, OTHER_PORT]
        node = make_node(monkeypatch)
        node.on_poll()
        assert node.runner.port == ''
        # 拔掉一个别的串口，只剩雷达
        world.candidates = [LIDAR_PORT]
        world.existing = {LIDAR_PORT}
        tick(2.0)
        node.get_clock().advance(2.0)
        node.on_poll()
        assert node.runner.port == LIDAR_PORT
        assert SERIALS[-1].path == LIDAR_PORT

    def test_rescan_only_on_reconnect_attempt(self, world, monkeypatch):
        """重扫只跟着真正的重连尝试走，不跟着轮询周期走。

        启动时没有设备：第一次 poll 扫一次，抛出的空端口就停在 missing。
        之后端口仍为空，每次 poll 都会调 ``_try_open``，但节流会挡住真正的
        打开尝试。重扫只跟尝试走，否则 100Hz 轮询会打爆设备树。
        """
        node = make_node(monkeypatch)
        node.on_poll()
        n = len(world.glob_calls)
        assert n >= 1
        for _ in range(20):
            tick(0.01)
            node.get_clock().advance(0.01)
            node.on_poll()
        assert len(world.glob_calls) == n

    def test_not_rescanned_within_throttle_window(self, world, monkeypatch):
        """节流窗口内设备出现，端口不会立刻变；跨过窗口才重扫。"""
        node = make_node(monkeypatch)
        node.on_poll()
        world.candidates = [LIDAR_PORT]
        world.existing = {LIDAR_PORT}
        tick(0.2)
        node.on_poll()
        assert node.runner.port == ''
        assert not SERIALS
        tick(2.0)
        node.on_poll()
        assert node.runner.port == LIDAR_PORT
        assert SERIALS[-1].path == LIDAR_PORT

    def test_rescan_after_interval(self, world, monkeypatch):
        """跨过重连间隔后重扫一次，节流不是永久关闭。"""
        node = make_node(monkeypatch)
        node.on_poll()
        n = len(world.glob_calls)
        tick(2.0)
        node.get_clock().advance(2.0)
        node.on_poll()
        assert len(world.glob_calls) == n + 1


class TestOtherUsbSerialPort:
    def test_other_serial_does_not_hijack(self, world, monkeypatch):
        """其他 USB 串口同时存在时不得误选：显式端口仍指向雷达。"""
        world.candidates = [TTYUSB, LIDAR_PORT]
        world.existing = {TTYUSB, LIDAR_PORT}
        node = make_node(monkeypatch, port=LIDAR_PORT)
        node.on_poll()
        assert node.runner.port == LIDAR_PORT
        assert SERIALS[-1].path == LIDAR_PORT
        assert TTYUSB not in [s.path for s in SERIALS]

    def test_auto_mode_refuses_when_other_serial_present(
            self, world, monkeypatch):
        world.candidates = [TTYUSB, LIDAR_PORT]
        world.existing = {TTYUSB, LIDAR_PORT}
        node = make_node(monkeypatch)
        node.on_poll()
        assert node.runner.port == '', '两个候选时不得打开任何一个'
        assert not SERIALS


def _launch_defaults():
    """从 launch 文件读 DeclareLaunchArgument 的默认值，不导入 launch。"""
    with open(LAUNCH, 'r', encoding='utf-8') as f:
        tree = ast.parse(f.read())
    defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = getattr(func, 'id', None) or getattr(func, 'attr', None)
        if name != 'DeclareLaunchArgument':
            continue
        arg_name = None
        default = None
        for kw in node.keywords:
            if kw.arg == 'default_value':
                try:
                    default = ast.literal_eval(kw.value)
                except ValueError:
                    parts = [ast.literal_eval(v) for v in
                             getattr(kw.value, 'values', [])]
                    default = ''.join(parts)
        if node.args:
            arg_name = ast.literal_eval(node.args[0])
        if arg_name:
            defaults[arg_name] = default
    return defaults


class TestLaunchDefaults:
    def test_lidar_port_is_field_value(self):
        assert _launch_defaults()['lidar_port'] == LIDAR_PORT

    def test_video_device_is_by_id(self):
        assert _launch_defaults()['video_device'] == CAM_BY_ID

    def test_lidar_uses_by_path_not_ttyusb(self):
        value = _launch_defaults()['lidar_port']
        assert value.startswith('/dev/serial/by-path/')
        assert '/dev/ttyUSB' not in value
        assert 'fc880000' not in value

    def test_camera_does_not_use_video0(self):
        value = _launch_defaults()['video_device']
        assert value.startswith('/dev/v4l/by-id/')
        assert value != '/dev/video0'

    def test_cmd_vel_still_disabled(self):
        assert _launch_defaults()['enable_cmd_vel'] == 'false'
