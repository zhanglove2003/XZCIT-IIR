"""lidar_node 节点级测试：诊断发布、健康前不发布扫描、断线停发。

用假 rclpy 与伪串口驱动真实节点代码，不连硬件。
"""

import math
import types

import pytest

import fake_ros
from opi_robot.lidar_node import health as H
from opi_robot.lidar_node import lidar_node as LN
import lidar_fixtures
from lidar_fixtures import build_frame, frame_rotation
from opi_robot import diagnostic_level as DL

PORT = '/dev/serial/by-path/platform-x-usb-0:1.2:1.0-port0'


class FakeSerial:
    instances = []

    def __init__(self, path, baud):
        self.path = path
        self.baud = baud
        self.chunks = bytearray()
        self.closed = False
        FakeSerial.instances.append(self)

    def push(self, data):
        self.chunks.extend(data)

    def read(self, n=4096):
        if not self.chunks:
            return b''
        data = bytes(self.chunks[:n])
        del self.chunks[:n]
        return data

    def close(self):
        self.closed = True


@pytest.fixture
def node(monkeypatch):
    FakeSerial.instances = []

    def fake_open(path, baud):
        return FakeSerial(path, baud)

    def fake_exists(path):
        return True

    def fake_glob(pattern):
        return [PORT]

    monkeypatch.setattr(LN, '_open_serial', fake_open)
    monkeypatch.setattr(LN, 'glob', types.SimpleNamespace(glob=fake_glob))
    monkeypatch.setattr(LN, 'os',
                        types.SimpleNamespace(path=types.SimpleNamespace(
                            exists=lambda p: True)))
    monkeypatch.setattr(LN, 'time', types.SimpleNamespace(
        sleep=lambda s: None, monotonic=lidar_fixtures.monotonic))
    return LN.LidarNode()


def diag_status(node):
    msgs = node.publishers['diagnostics'].messages
    assert msgs, '未发布诊断'
    return msgs[-1].status[0]


def kv(status, key):
    for item in status.values:
        if item.key == key:
            return item.value
    raise KeyError(key)


def feed_full_rotation(node):
    serial = FakeSerial.instances[-1]
    for frame in frame_rotation():
        serial.push(frame)
        node.get_clock().advance(0.02)
        lidar_fixtures.advance(0.02)
        node.on_poll()


class TestStartup:
    def test_uses_stable_by_path_port(self, node):
        # 端口扫描推迟到第一次 poll：那时 runner 已存在，逻辑只剩一条路径
        assert node.runner.port == ''
        node.on_poll()
        assert node.runner.port == PORT
        assert '/dev/ttyUSB' not in node.runner.port

    def test_diagnostics_before_any_frame(self, node):
        node.on_poll()
        node.publish_diag()
        st = diag_status(node)
        assert st.name == 'lidar'
        assert kv(st, 'state') == H.CONNECTING
        # 真实消息里 level 是单字节 bytes，用解码函数比较
        assert DL.decode_level(st.level) == H.LEVEL_WARN

    def test_no_scan_before_coverage(self, node):
        node.on_poll()
        FakeSerial.instances[-1].push(build_frame())
        node.get_clock().advance(0.02)
        node.on_poll()
        assert node.publishers['scan'].messages == []


class TestHealthy:
    def test_publishes_scan_when_healthy(self, node):
        node.on_poll()
        feed_full_rotation(node)
        node.on_poll()
        scans = node.publishers['scan'].messages
        assert scans, '健康状态下必须发布扫描'
        assert len(scans[-1].ranges) == 360
        node.publish_diag()
        assert kv(diag_status(node), 'state') == H.HEALTHY

    def test_scan_frame_id_and_stamp(self, node):
        node.on_poll()
        feed_full_rotation(node)
        node.on_poll()
        msg = node.publishers['scan'].messages[-1]
        assert msg.header.frame_id == 'laser'
        assert msg.header.stamp is not None

    def test_front_point_present(self, node):
        node.on_poll()
        feed_full_rotation(node)
        node.on_poll()
        ranges = node.publishers['scan'].messages[-1].ranges
        assert math.isfinite(ranges[0])
        assert ranges[0] == pytest.approx(1.5, abs=1e-6)

    def test_default_expires_old_angle_while_other_frames_arrive(self, node):
        node.on_poll()
        feed_full_rotation(node)
        assert math.isfinite(node.publishers['scan'].messages[-1].ranges[0])
        serial = FakeSerial.instances[-1]
        for _ in range(6):
            serial.push(build_frame(start_ddeg=9000, end_ddeg=10100))
            lidar_fixtures.advance(0.1)
            node.on_poll()
        assert math.isinf(node.publishers['scan'].messages[-1].ranges[0])


class TestDisconnect:
    def _healthy(self, node):
        node.on_poll()
        feed_full_rotation(node)
        node.on_poll()
        assert node.publishers['scan'].messages

    def test_stops_publishing_after_timeout(self, node):
        self._healthy(node)
        before = len(node.publishers['scan'].messages)
        # 先推进到超过 frame_timeout，此后再采样，数量必须不再增长
        for _ in range(60):
            node.get_clock().advance(0.01)
            lidar_fixtures.advance(0.01)
            node.on_poll()
        frozen = len(node.publishers['scan'].messages)
        for _ in range(60):
            node.get_clock().advance(0.01)
            lidar_fixtures.advance(0.01)
            node.on_poll()
        assert len(node.publishers['scan'].messages) == frozen
        assert frozen >= before
        node.publish_diag()
        st = diag_status(node)
        # 超时后串口会被释放并重开，状态可能是 stale、disconnected
        # 或重连中的 connecting；三者都不健康，都不得发扫描
        assert kv(st, 'state') in (H.STALE, H.DISCONNECTED, H.CONNECTING)
        assert DL.decode_level(st.level) != H.LEVEL_OK

    def test_no_stale_scan_with_new_stamp(self, node):
        """断线后不得出现"旧测距 + 新时间戳"。"""
        self._healthy(node)
        old_stamp = node.publishers['scan'].messages[-1].header.stamp.sec
        node.get_clock().advance(5.0)
        lidar_fixtures.advance(5.0)
        for _ in range(20):
            node.on_poll()
        stamps = [m.header.stamp.sec
                  for m in node.publishers['scan'].messages]
        assert all(s == old_stamp for s in stamps)

    def test_diagnostics_keep_publishing_while_down(self, node):
        self._healthy(node)
        for _ in range(80):
            node.get_clock().advance(0.01)
            node.on_poll()
        n0 = len(node.publishers['diagnostics'].messages)
        node.publish_diag()
        node.publish_diag()
        assert len(node.publishers['diagnostics'].messages) == n0 + 2


class TestDiagFields:
    def test_required_keys(self, node):
        node.on_poll()
        node.publish_diag()
        st = diag_status(node)
        keys = {v.key for v in st.values}
        for key in ('state', 'reason', 'age_s', 'frames_ok', 'frames_bad',
                    'coverage', 'reconnects', 'port', 'origin', 'connected'):
            assert key in keys

    def test_counters_increase(self, node):
        node.on_poll()
        feed_full_rotation(node)
        node.on_poll()
        node.publish_diag()
        st = diag_status(node)
        assert int(kv(st, 'frames_ok')) >= 12
