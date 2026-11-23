"""ROS 2 节点集成测试（伪串口 pty，不接触真实 UART3、不发送任何命令）。

本模块依赖 rclpy 与 sensor_msgs；在没有 ROS 2 的环境中整体跳过，
**不用任何伪 ROS 结果替代**。在 ROS 2 Humble 上运行时它验证：

- 节点可启动并从串口设备解析状态帧、发布 imu/data_raw；
- 消息类型为 sensor_msgs/Imu，frame_id、时间戳（主机接收时刻）正确；
- FlagStop!=0 时角速度置零且 angular_velocity_covariance[0] = -1；
- 姿态恒不可用（orientation_covariance[0] = -1，四元数全零）；
- 单位换算（LSB → m/s²、rad/s）正确。
"""
import math
import os
import signal
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

from test_frame_parser import build_frame  # noqa: E402


def test_node_publishes_imu_from_pty():
    from c50c_uart3.uart3_node import C50CUart3Node

    master_fd, slave_fd = os.openpty()
    slave_name = os.ttyname(slave_fd)
    # 通过 CLI 参数注入串口设备路径（节点自身从参数读取 device）
    rclpy.init(args=['--ros-args', '-p', f'device:={slave_name}'])
    try:
        node = C50CUart3Node()
        # 订阅端必须与发布端同为 sensor data（best-effort）QoS
        received = []
        node.create_subscription(Imu, 'imu/data_raw', received.append,
                                 qos_profile_sensor_data)
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        spin = threading.Thread(target=executor.spin, daemon=True)
        spin.start()
        try:
            # 等节点完成串口打开（tcflush 之后）且订阅匹配，避免第 1 帧
            # 在打开前被 tcflush 丢弃或在匹配前被 best-effort 丢弃
            deadline = time.monotonic() + 10.0
            while (getattr(node, '_fd', None) is None
                   and time.monotonic() < deadline):
                time.sleep(0.05)
            assert node._fd is not None, '节点应在超时前打开伪终端'
            while (node.count_subscribers('imu/data_raw') < 1
                   and time.monotonic() < deadline):
                time.sleep(0.05)
            # 使能态一帧 + 失能态一帧，分两次写入模拟分段到达
            os.write(master_fd, build_frame(stop=0, vx=100, az=16384, gz=3755))
            _wait_for(received, 1)
            os.write(master_fd, build_frame(stop=1, vx=200, ax=-8192, gz=9999))
            _wait_for(received, 2)
        finally:
            executor.shutdown()
            spin.join(timeout=3.0)
            node.destroy_node()
            # 节点退出后应释放伪终端（节点自身 fd 已关闭）
            from c50c_uart3.serial_util import find_device_holders
            assert find_device_holders(slave_name) == [], \
                '节点退出后不应再持有伪终端'

        assert len(received) == 2, '应在超时前收到两帧 IMU 消息'
        m1, m2 = received

        # 消息类型与姿态不可用约定
        assert isinstance(m1, Imu)
        assert m1.orientation_covariance[0] == -1.0
        assert (m1.orientation.x, m1.orientation.y,
                m1.orientation.z, m1.orientation.w) == (0.0, 0.0, 0.0, 0.0)
        # 主机接收时间戳：不为零（帧内无采样时间，应为节点时钟接收时刻）
        assert m1.header.stamp.sec > 0

        # 使能帧：角速度按 ±500 dps 量程换算（3755 LSB ≈ 1.0 rad/s）
        assert m1.angular_velocity_covariance[0] >= 0.0
        assert math.isclose(m1.angular_velocity.z, 3755 / 3754.9, rel_tol=1e-3)
        # Z 通道含 +1g 基准：16384 LSB ÷ 1671.84 ≈ 9.79998 m/s²（固件 1g≈9.8 约定）
        assert math.isclose(m1.linear_acceleration.z, 16384 / 1671.84, rel_tol=1e-9)

        # 失能帧：角速度置零且标记不可用；加速度不受失能影响
        assert m2.angular_velocity_covariance[0] == -1.0
        assert (m2.angular_velocity.x, m2.angular_velocity.y,
                m2.angular_velocity.z) == (0.0, 0.0, 0.0)
        assert math.isclose(m2.linear_acceleration.x, -8192 / 1671.84,
                            rel_tol=1e-6)
    finally:
        rclpy.shutdown()
        os.close(master_fd)
        os.close(slave_fd)


def _wait_for(received, count, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while len(received) < count and time.monotonic() < deadline:
        time.sleep(0.05)


def test_sigint_during_reception_shuts_down_cleanly(tmp_path):
    """持续接收数据时对节点进程发 SIGINT：进程退出、读取线程结束、
    伪终端释放，且不出现 context 已关闭后的 publish 报错。

    子进程方式启动真实节点入口（python -m），与 main() 的信号处理路径
    完全一致；使用隔离 ROS 域，不接触真实 UART3。"""
    import subprocess
    import sys
    from pathlib import Path

    from c50c_uart3.serial_util import find_device_holders

    package_root = Path(__file__).resolve().parents[1]
    master_fd, slave_fd = os.openpty()
    slave_name = os.ttyname(slave_fd)
    env = dict(os.environ)
    env['PYTHONPATH'] = (str(package_root) + os.pathsep
                         + env.get('PYTHONPATH', ''))
    # 隔离域：避免与本机其他 ROS 图通信
    env['ROS_DOMAIN_ID'] = env.get('ROS_DOMAIN_ID_TEST_ISOLATED', '188')

    log_path = tmp_path / 'node_sigint_test.log'
    proc = subprocess.Popen(
        [sys.executable, '-m', 'c50c_uart3.uart3_node',
         '--ros-args', '-p', f'device:={slave_name}'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
        cwd=str(tmp_path))
    stop_writing = threading.Event()
    writer = threading.Thread(
        target=_feed_frames, args=(master_fd, stop_writing), daemon=True)
    log_lines = []

    try:
        writer.start()
        # 等节点打开串口并开始收帧（日志出现打开字样，且持续喂帧已写入）
        deadline = time.monotonic() + 15.0
        opened = False
        while time.monotonic() < deadline:
            line = proc.stdout.readline()
            if not line:
                break
            log_lines.append(line)
            if '已打开串口'.encode() in line:
                opened = True
                break
        assert opened, (f'节点应在超时前打开伪终端；日志见 {log_path}:\n'
                        + b''.join(log_lines).decode(errors='replace'))

        # 持续接收状态下发送 SIGINT（此时写线程仍在向 pty 喂帧）
        time.sleep(0.3)
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5.0)
            raise AssertionError('SIGINT 后节点未在 15 s 内退出')
        stop_writing.set()
        writer.join(timeout=3.0)
        # 排空剩余输出，与前面日志一起落盘供失败诊断
        rest = proc.stdout.read()
        log_lines.append(rest)
        log_path.write_bytes(b''.join(log_lines))

        output = log_path.read_text(encoding='utf-8', errors='replace')
        for bad in ('RCLError', 'context is invalid', 'Exception in thread',
                    '读取线程 3 s 内未退出'):
            assert bad not in output, \
                f'退出日志不应出现 {bad!r}；日志:\n{output}'
        assert proc.returncode == 0, f'应正常退出，实际 {proc.returncode}:\n{output}'
        # 读取线程与串口都已释放
        assert find_device_holders(slave_name) == [], \
            '节点退出后不应再持有伪终端'
    finally:
        stop_writing.set()
        if writer.is_alive():
            writer.join(timeout=2.0)
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=5.0)
        for fd in (master_fd, slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass


def _feed_frames(master_fd, stop_event, period_s=0.05):
    """持续向 pty 主端写入使能态状态帧，模拟固件 20 Hz 发送。"""
    frame = build_frame(stop=0, vx=0, az=16384, gz=0)
    while not stop_event.is_set():
        try:
            os.write(master_fd, frame)
        except OSError:
            return
        stop_event.wait(period_s)
