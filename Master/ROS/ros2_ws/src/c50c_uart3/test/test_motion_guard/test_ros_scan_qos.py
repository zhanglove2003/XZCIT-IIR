"""Exercise the guard's scan subscription with a real ROS 2 publisher."""
import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range

from c50c_uart3.chassis_cmd.guard_bridge import MotionLicenseBridge


def test_guard_receives_best_effort_scan_and_stops_when_it_stales():
    rclpy.init()
    node = rclpy.create_node('guard_scan_qos_test')
    try:
        bridge = MotionLicenseBridge(license_timeout_s=2.0)
        bridge.attach(node, scan_topic='/test_guard_scan_qos')
        publisher = node.create_publisher(
            LaserScan, '/test_guard_scan_qos', qos_profile_sensor_data)
        scan = LaserScan()
        scan.angle_min = 0.0
        scan.angle_increment = 2.0 * math.pi / 360
        scan.angle_max = scan.angle_increment * 359
        scan.range_min = 0.02
        scan.range_max = 12.0
        scan.ranges = [2.0] * 360

        deadline = time.monotonic() + 3.0
        while bridge._scan[0] is None and time.monotonic() < deadline:
            scan.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(scan)
            rclpy.spin_once(node, timeout_sec=0.02)
        assert bridge._scan[0] is not None, 'guard did not receive best-effort scan'

        bridge.on_ultrasonic(Range(range=1.0))
        assert bridge.on_license_payload({
            'session': 1, 'seq': 1, 'expires_mono': time.monotonic() + 2.0,
        }) == (True, 'ok')
        bridge.authorize(True)
        bridge.on_rearm_request(True)
        assert bridge.tick_record_only().decision[4] == 'no_demand'
        assert not bridge.guard.stopped

        time.sleep(0.35)
        bridge.on_ultrasonic(Range(range=1.0))
        assert bridge.tick_record_only().decision[4] == 'scan_stale'
        assert bridge.guard.stopped
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_guard_receives_best_effort_ultrasonic_over_topic():
    """底盘超声订阅必须与 uart3 的 BEST_EFFORT 发布端匹配。

    [2026-11-19] 上一版 ``create_subscription(Range, ultrasonic_topic,
    self.on_ultrasonic, 10)`` 用整数深度 ⇒ rclpy 默认 RELIABLE，而发布端是
    ``qos_profile_sensor_data``（BEST_EFFORT），DDS 层不匹配，订阅恒收 0 条。
    同一 attach() 里 scan 用了 sensor data，唯独超声漏改。
    """
    rclpy.init()
    node = rclpy.create_node('guard_ultrasonic_qos_test')
    try:
        bridge = MotionLicenseBridge(license_timeout_s=2.0)
        bridge.attach(node, ultrasonic_topic='/test_guard_ultra_qos')
        publisher = node.create_publisher(
            Range, '/test_guard_ultra_qos', qos_profile_sensor_data)

        msg = Range()
        msg.range = 1.25
        deadline = time.monotonic() + 3.0
        while bridge._ultra[0] is None and time.monotonic() < deadline:
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)
        assert bridge._ultra[0] == pytest.approx(1.25), (
            'guard 未从 BEST_EFFORT 发布端收到超声读数（QoS 不匹配）')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stale_topic_reading_does_not_overwrite_newer_status_frame():
    """迟到的旧话题消息不得覆盖较新的状态帧读数。

    [2026-11-19] 两条路径写同一个 ``_ultra``。状态帧直读带串口帧自身的
    单调时刻，话题消息多绕一趟 DDS 才到，天然更旧；若只按到达顺序覆盖，
    一条旧读数会把更新的状态帧读数冲掉（stale-wins）。
    """
    bridge = MotionLicenseBridge(license_timeout_s=2.0)

    # 状态帧直读：串口帧在 t=10.0 采样，读到 0.30 m。
    bridge.on_ultrasonic(Range(range=0.30), source='status_frame', stamp_mono=10.0)
    assert bridge._ultra == (pytest.approx(0.30), pytest.approx(10.0))

    # 话题消息在 t=10.05 才到，携带的是更早采样(9.98)的 0.45 m 读数。
    bridge.on_ultrasonic(Range(range=0.45), source='topic', stamp_mono=9.98)

    assert bridge._ultra[0] == pytest.approx(0.30), '旧话题读数覆盖了较新的状态帧读数'
    assert bridge._ultra[1] == pytest.approx(10.0)


def test_equal_timestamp_from_topic_is_ignored_against_status_frame():
    """同一采样时刻，状态帧口径优先，避免同帧双写抖动。"""
    bridge = MotionLicenseBridge(license_timeout_s=2.0)
    bridge.on_ultrasonic(Range(range=0.30), source='status_frame', stamp_mono=10.0)
    bridge.on_ultrasonic(Range(range=0.45), source='topic', stamp_mono=10.0)
    assert bridge._ultra[0] == pytest.approx(0.30)
