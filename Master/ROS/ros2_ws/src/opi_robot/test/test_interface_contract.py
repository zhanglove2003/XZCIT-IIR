"""接口契约测试：话题名、消息字段、诊断 key 与交付文档一致。

这些名字一旦和文档不一致，现场排查会直接卡住。
"""

import math

import fake_ros
from opi_robot.lidar_node.lidar_node import LidarNode

PORT = '/dev/serial/by-path/platform-x-usb-0:1.2:1.0-port0'
from opi_robot.fusion_node.fusion_node import FusionNode
from opi_robot.web_node import web_debug as W


def test_lidar_topics():
    node = LidarNode()
    assert node.publishers['scan'].topic == 'scan'
    assert node.publishers['diagnostics'].topic == 'diagnostics'


def test_fusion_topics():
    node = FusionNode()
    # nav_cmd_vel / obstacle_report 是第一阶段话题；drive_safety 是
    # 2026-11-14 安全许可新增（T1），同时约束手动与自动。
    assert set(node.publishers) == {'auto_cmd_vel', 'obstacle_report',
                                    'drive_safety'}


def test_fusion_subscriptions():
    node = FusionNode()
    assert 'scan' in node.subscriptions
    assert 'diagnostics' in node.subscriptions
    assert 'image_raw/compressed' in node.subscriptions
    assert 'scan_raw_evidence' in node.subscriptions


def test_diag_keys_match_doc():
    node = LidarNode()
    node.on_poll()
    node.publish_diag()
    status = node.publishers['diagnostics'].messages[-1].status[0]
    keys = {kv.key for kv in status.values}
    documented = {
        'state', 'reason', 'age_s', 'frames_ok', 'frames_bad',
        'bytes_dropped', 'coverage', 'reconnects', 'port',
        'origin', 'connected', 'last_error',
    }
    assert documented <= keys, '文档字段缺失: %s' % (documented - keys)
    assert status.name == 'lidar'


def test_scan_message_shape(monkeypatch):
    from opi_robot.lidar_node import lidar_node as LN
    import types
    from test_lidar_node import FakeSerial
    from lidar_fixtures import frame_rotation

    FakeSerial.instances = []
    monkeypatch.setattr(LN, '_open_serial', lambda p, b: FakeSerial(p, b))
    monkeypatch.setattr(LN, 'glob',
                        types.SimpleNamespace(glob=lambda pat: [PORT]))
    monkeypatch.setattr(LN, 'os',
                        types.SimpleNamespace(path=types.SimpleNamespace(
                            exists=lambda p: True)))
    monkeypatch.setattr(LN, 'time', types.SimpleNamespace(
        sleep=lambda s: None, monotonic=lambda: 0.0))
    node = LN.LidarNode()
    node.on_poll()
    assert FakeSerial.instances, '伪串口未被打开'
    serial = FakeSerial.instances[-1]
    for frame in frame_rotation():
        serial.push(frame)
        node.get_clock().advance(0.02)
        node.on_poll()
    node.on_poll()
    scan = node.publishers['scan'].messages[-1]
    assert len(scan.ranges) == 360
    assert scan.angle_min == 0.0
    assert scan.angle_increment == pytest_approx(2 * math.pi / 360)
    assert scan.range_min == 0.02
    assert scan.range_max == 12.0


def pytest_approx(v):
    import pytest
    return pytest.approx(v)


def test_status_payload_fields_match_page():
    """web_debug.status_payload 的字段必须覆盖页面读取的键。"""
    payload = W.status_payload(now=1.0)
    for key in ('t', 'lidar_diag', 'lidar_state', 'scan', 'image',
                'lidar_age', 'image_age', 'lidar_trusted'):
        assert key in payload, '页面读取的字段缺失: %s' % key
