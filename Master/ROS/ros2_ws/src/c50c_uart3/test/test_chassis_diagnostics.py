"""在隔离 ROS 域中以伪串口验证底盘诊断，不连接车辆。"""
import os
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from diagnostic_msgs.msg import DiagnosticArray

from test_frame_parser import build_frame


def test_valid_status_frame_reaches_chassis_diagnostics():
    from c50c_uart3.uart3_node import C50CUart3Node
    from opi_robot.web_node.chassis import parse_chassis_status, chassis_view
    from opi_robot.web_node import web_debug

    master, slave = os.openpty()
    rclpy.init(args=['--ros-args', '-p', f'device:={os.ttyname(slave)}'])
    node = None
    web = None
    executor = None
    spin_thread = None
    try:
        node = C50CUart3Node()
        web = web_debug.Bridge()
        received = []
        node.create_subscription(DiagnosticArray, '/diagnostics', received.append, 10)
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(web)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        deadline = time.monotonic() + 5
        while (node._fd is None and time.monotonic() < deadline):
            time.sleep(0.02)
        assert node._fd is not None
        while (node.count_publishers('/diagnostics') < 1
               and time.monotonic() < deadline):
            time.sleep(0.02)
        assert node.count_publishers('/diagnostics') == 1, '串口所有者未发布底盘诊断'

        invalid = bytearray(build_frame(voltage_mv=24500))
        invalid[20] ^= 1  # 校验和不再匹配；不能把坏帧发布为电压。
        os.write(master, invalid)
        time.sleep(0.2)
        assert not received
        os.write(master, build_frame(
            stop=0, vx=123, vy=-45, wz=230, voltage_mv=24500,
            encoder=(12, -34, 56, -78), pwm=(3000, 3100, 3200, 3300)))
        deadline = time.monotonic() + 5
        while not received and time.monotonic() < deadline:
            time.sleep(0.02)
        assert received, '校验通过的状态帧没有产生诊断'
        message = received[-1]
        assert message.header.stamp.sec > 0
        (status,) = message.status
        assert status.name == 'chassis'
        values = {kv.key: kv.value for kv in status.values}
        assert values['voltage_mv'] == '24500'
        assert [values[f'encoder_{name}_reported'] for name in 'abcd'] == [
            '12', '-34', '56', '-78']
        assert [values[f'pwm_ch{i}'] for i in range(1, 5)] == [
            '3000', '3100', '3200', '3300']
        parsed = parse_chassis_status(message)
        view = chassis_view(parsed, 10.0, 10.5)
        assert view['voltage_v'] == 24.5
        assert view['vx_m_s'] == 0.123
        assert view['vy_m_s'] == -0.045
        assert view['wz_rad_s'] == 0.23
        assert chassis_view(parsed, 10.0, 12.1)['voltage_v'] is None
        deadline = time.monotonic() + 5
        while web_debug.status_payload()['chassis']['voltage_v'] != 24.5:
            assert time.monotonic() < deadline, '网页桥没有接收底盘电压'
            time.sleep(0.02)
        payload = web_debug.status_payload()
        assert payload['battery']['voltage_v'] == 24.5
        assert payload['battery']['percent'] is None
        assert payload['chassis']['drive_ready'] is None
        stale = web_debug.status_payload(now=time.time() + 3.0)
        assert stale['battery']['voltage_v'] is None
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=3)
        if web is not None:
            web.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)
