"""真实 ROS 话题下验证鉴权和解除锁停的两种到达顺序。"""
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import Bool

from c50c_uart3.chassis_cmd.guard_bridge import MotionLicenseBridge


@pytest.mark.parametrize('authorize_first', [True, False])
def test_rearm_authorize_ros_arrival_orders(authorize_first):
    rclpy.init()
    node = rclpy.create_node('guard_rearm_order_test')
    try:
        bridge = MotionLicenseBridge()
        bridge.attach(node, rearm_topic='/test_guard_rearm')
        node.create_subscription(
            Bool, '/test_guard_authorize',
            lambda msg: bridge.authorize(msg.data), 10)
        auth = node.create_publisher(Bool, '/test_guard_authorize', 10)
        rearm = node.create_publisher(Bool, '/test_guard_rearm', 10)
        deadline = time.monotonic() + 3.0
        while not (auth.get_subscription_count() and rearm.get_subscription_count()):
            assert time.monotonic() < deadline, 'DDS 发现超时'
            rclpy.spin_once(node, timeout_sec=0.02)

        first, second = (auth, rearm) if authorize_first else (rearm, auth)
        first.publish(Bool(data=True))
        deadline = time.monotonic() + 3.0
        while not (bridge.session_authorized if authorize_first else bridge._rearm_requested):
            assert time.monotonic() < deadline, '第一条消息未收到'
            rclpy.spin_once(node, timeout_sec=0.02)
        assert bridge.guard.stopped
        second.publish(Bool(data=True))
        deadline = time.monotonic() + 3.0
        while bridge.guard.stopped:
            assert time.monotonic() < deadline, '第二条消息未解除锁停'
            rclpy.spin_once(node, timeout_sec=0.02)
        assert bridge.guard.rearms == 1
        assert not bridge._rearm_requested
    finally:
        node.destroy_node()
        rclpy.shutdown()
