"""web_debug 桥接层测试：订阅写入共享状态、状态载荷随新鲜度变化。

不启动 HTTP 服务，只测 Bridge 与 status_payload。
"""

import time

import pytest

import fake_ros
from opi_robot.web_node import web_debug as W
from opi_robot.web_node import sensors as S


@pytest.fixture(autouse=True)
def clean_state():
    W.reset_state()
    yield
    W.reset_state()


def scan_msg(ranges):
    import math
    n = len(ranges)
    return fake_ros.LaserScan(
        ranges=list(ranges), angle_min=0.0,
        angle_increment=2 * math.pi / n,
        range_min=0.02, range_max=12.0)


def test_bridge_subscribes_diagnostics():
    bridge = W.Bridge()
    assert 'diagnostics' in bridge.subscriptions
    assert 'scan' in bridge.subscriptions
    assert 'image_raw/compressed' in bridge.subscriptions


def test_image_updates_state():
    bridge = W.Bridge()
    bridge.on_image(fake_ros.CompressedImage(data=b'\xff\xd8' * 50))
    assert W._state['image_seq'] == 1
    assert W._state['image_t'] > 0.0


def test_scan_updates_state_and_counts_valid():
    bridge = W.Bridge()
    bridge.on_scan(scan_msg([1.0, float('inf'), float('nan'), 3.0]))
    assert W._state['scan_frames'] == 1
    assert W._state['scan_valid'] == 2
    pts = W._state['scan']
    assert pts[1][1] == 0.0
    assert pts[2][1] == 0.0


def test_status_payload_unknown_when_empty():
    payload = W.status_payload(now=1000.0)
    assert payload['t'] == 'status'
    assert payload['scan'] == S.UNKNOWN
    assert payload['image'] == S.UNKNOWN
    assert payload['lidar_trusted'] is False


def test_status_payload_marks_stale_after_timeout():
    bridge = W.Bridge()
    bridge.on_image(fake_ros.CompressedImage(data=b'\xff\xd8'))
    bridge.on_diagnostics(W.S.parse_diagnostic and fake_ros.DiagnosticArray(
        status=[fake_ros.DiagnosticStatus.make(
            level=0, name='lidar', message='healthy',
            values=[fake_ros.KeyValue(key='state', value='healthy')])]))
    t0 = time.time()
    bridge.on_scan(scan_msg([1.0, 2.0]))
    fresh = W.status_payload(now=time.time())
    assert fresh['scan'] == S.FRESH
    stale = W.status_payload(now=time.time() + 5.0)
    assert stale['scan'] == S.STALE
    assert stale['lidar_trusted'] is False
    assert t0 > 0


def test_status_payload_error_after_disconnect():
    bridge = W.Bridge()
    bridge.on_diagnostics(fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(
            level=2, name='lidar', message='disconnected (read_error)',
            values=[fake_ros.KeyValue(key='state', value='disconnected')])]))
    bridge.on_scan(scan_msg([1.0, 2.0]))
    payload = W.status_payload(now=time.time())
    assert payload['lidar_diag'] == 'error'
    assert payload['scan'] == S.STALE
    assert payload['lidar_state'] == 'disconnected'


def test_payload_is_json_safe():
    import json
    W.Bridge().on_scan(scan_msg([float('inf'), 2.0]))
    json.dumps(W.status_payload(now=time.time()), allow_nan=False)
