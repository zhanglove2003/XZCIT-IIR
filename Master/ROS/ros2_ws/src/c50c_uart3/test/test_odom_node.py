#!/usr/bin/env python3
"""UART3 节点里程计影子输出集成测试（伪串口 pty，真实 ROS 2，不接触真实 UART3）。

依赖 rclpy / nav_msgs；无 ROS 环境时整体跳过（与 test_uart3_node 同策略）。
覆盖（对应任务书"准备 UART3 里程计影子联跑"）：

1. 默认关闭：不创建影子话题发布器，IMU 输出不受接线影响；
2. 开启模式：Odometry 内容（frames、编码器 wz ≠ IMU 陀螺、协方差布局、
   单调时间基准）、IMU 话题同帧照常发布；
3. 断流间隙：dt > odom_max_interval_s 时不跨间隙积分，诊断计数暴露；
4. 重连：显式重置积分时间基准（被动判定之外的双保险）；
5. 节点退出：开启状态下 SIGINT 正常退出、释放伪终端。

速度标度未标定、协方差为占位值——测试只断言结构与方向，不断言绝对
标度精度。整车偏航绝对方向、速度标度、正式 /odom / TF 均不在本次范围。
"""
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')

from test_frame_parser import build_frame, craft_splice_prefix  # noqa: E402

SHADOW_TOPIC = '/odom_candidate'


def _frame(vx=0, wz=0, gz=0, stop=0):
    """使能态状态帧；gz 故意给显眼值，断言它不进入里程计 twist。"""
    return build_frame(stop=stop, vx=vx, vy=0, wz=wz,
                       ax=0, ay=0, az=16384, gx=0, gy=0, gz=gz,
                       voltage_mv=4060)


def _open_pty():
    master_fd, slave_fd = os.openpty()
    return master_fd, slave_fd, os.ttyname(slave_fd)


def _wait(cond, timeout_s=10.0, interval=0.05):
    deadline = time.monotonic() + timeout_s
    while not cond():
        if time.monotonic() > deadline:
            return False
        time.sleep(interval)
    return True


def _start_node_inprocess(slave_name, extra_params):
    """进程内启动节点（device 经 CLI 注入，其余里程计参数经 extra_params）。"""
    args = ['--ros-args', '-p', f'device:={slave_name}']
    for kv in extra_params:
        args += ['-p', kv]
    rclpy.init(args=args)
    from c50c_uart3.uart3_node import C50CUart3Node
    return C50CUart3Node()


def _spin_with_executor(node):
    """后台执行器线程；返回 (executor, spin_thread)。"""
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    return executor, spin


def _close(executor, spin, node, master_fd, slave_fd, slave_name):
    executor.shutdown()
    spin.join(timeout=3.0)
    node.destroy_node()
    from c50c_uart3.serial_util import find_device_holders
    assert find_device_holders(slave_name) == [], '节点退出后不应再持有伪终端'
    rclpy.shutdown()
    os.close(master_fd)
    os.close(slave_fd)


# --------------------------------------------------------------- 1. 默认关闭

def test_odom_disabled_by_default_no_shadow_topic_imu_unaffected():
    """默认参数：无影子话题发布器；IMU 输出与串口独占行为不变。"""
    from sensor_msgs.msg import Imu

    master_fd, slave_fd, slave_name = _open_pty()
    node = _start_node_inprocess(slave_name, [])
    received = []
    node.create_subscription(Imu, '/imu/data_raw', received.append,
                             rclpy.qos.qos_profile_sensor_data)
    executor, spin = _spin_with_executor(node)
    try:
        assert _wait(lambda: node._fd is not None), '节点应打开伪终端'
        assert _wait(lambda: node.count_subscribers('/imu/data_raw') >= 1)
        os.write(master_fd, _frame(vx=500, wz=1000, gz=9999))
        assert _wait(lambda: len(received) >= 1), 'IMU 输出应不受接线影响'

        # 影子话题不存在：默认关闭时不创建发布器
        assert node._odom_candidate is None
        assert node.count_publishers(SHADOW_TOPIC) == 0
        assert node._odom_diagnostics is None
    finally:
        _close(executor, spin, node, master_fd, slave_fd, slave_name)

    # IMU 内容回归抽查：编码器 wz 不应串到 IMU 角速度
    imu = received[0]
    import math
    assert imu.orientation_covariance[0] == -1.0
    assert math.isclose(imu.angular_velocity.z, 9999 / 3754.9, rel_tol=1e-3)


# ------------------------------------------------------- 2. 开启模式：消息内容

def test_odom_enabled_shadow_topic_message_content():
    """开启后：影子话题发布 Odometry；编码器 wz 生效、陀螺 z 不混入；
    协方差布局符合候选模块约定；IMU 同帧照常发布。"""
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    master_fd, slave_fd, slave_name = _open_pty()
    node = _start_node_inprocess(slave_name, ['odom_enable:=true'])
    odom_msgs, imu_msgs = [], []
    node.create_subscription(Odometry, SHADOW_TOPIC, odom_msgs.append, 10)
    node.create_subscription(Imu, '/imu/data_raw', imu_msgs.append,
                             rclpy.qos.qos_profile_sensor_data)
    executor, spin = _spin_with_executor(node)
    try:
        assert _wait(lambda: node._fd is not None)
        assert _wait(lambda: node.count_subscribers(SHADOW_TOPIC) >= 1)
        assert _wait(lambda: node.count_subscribers('/imu/data_raw') >= 1)
        # 编码器 vx=500mm/s、wz=1000mrad/s；IMU 陀螺 z=9999 LSB（显眼值）
        for _ in range(3):
            os.write(master_fd, _frame(vx=500, wz=1000, gz=9999))
            time.sleep(0.06)
        assert _wait(lambda: len(odom_msgs) >= 3), \
            f'应收到影子话题消息，实际 {len(odom_msgs)}'
        assert _wait(lambda: len(imu_msgs) >= 3)
    finally:
        _close(executor, spin, node, master_fd, slave_fd, slave_name)

    import math
    m = odom_msgs[-1]
    assert isinstance(m, Odometry)
    assert m.header.frame_id == 'odom'
    assert m.child_frame_id == 'base_link'
    assert m.header.stamp.sec > 0
    # 接收时刻单调 → 消息戳不倒退
    stamps = [o.header.stamp.sec + o.header.stamp.nanosec * 1e-9
              for o in odom_msgs]
    assert stamps == sorted(stamps), '消息戳应随接收单调'

    # 编码器 wz：1000 mrad/s = 1.0 rad/s；陀螺 9999 LSB ≈ 2.66 rad/s 不混入
    assert math.isclose(m.twist.twist.angular.z, 1.0, rel_tol=1e-12)
    assert abs(m.twist.twist.angular.z - 9999 / 3754.9) > 1.0
    assert math.isclose(m.twist.twist.linear.x, 0.5, rel_tol=1e-12)
    assert m.twist.twist.linear.y == 0.0
    assert m.twist.twist.angular.x == 0.0 and m.twist.twist.angular.y == 0.0

    # 协方差布局：占位对角项 + 未测量维度大方差（1e6），非对角线为 0
    assert m.pose.covariance[0] == 0.01 and m.pose.covariance[7] == 0.01
    assert m.pose.covariance[35] == 0.01
    for idx in (14, 21, 28):
        assert m.pose.covariance[idx] == 1e6
    assert m.twist.covariance[0] == 0.01 and m.twist.covariance[35] == 0.01
    for idx in (7, 14, 21, 28):
        assert m.twist.covariance[idx] == 1e6
    assert all(m.pose.covariance[i] == 0.0 for i in range(36) if i % 7)
    assert all(m.twist.covariance[i] == 0.0 for i in range(36) if i % 7)

    # 积分：vx>0、wz>0（首帧只建基准，后两帧各积分 ~0.06s）
    assert m.pose.pose.position.x > 0.0
    assert m.pose.pose.position.x < 0.3
    assert 0.0 < m.pose.pose.orientation.z  # wz>0 → yaw 增大
    q = m.pose.pose.orientation
    yaw = 2 * math.atan2(q.z, q.w)
    assert 0.0 < yaw <= 0.3

    # 同帧 IMU 回归：陀螺换算不受接线影响
    imu = imu_msgs[-1]
    assert math.isclose(imu.angular_velocity.z, 9999 / 3754.9, rel_tol=1e-3)
    assert imu.orientation_covariance[0] == -1.0

    # 诊断计数暴露
    d = node._odom_diagnostics
    assert d['accepted_steps'] >= 2 and d['rejected_gap'] == 0


# ------------------------------------------------------------ 3. 断流间隙

def test_odom_gap_not_crossed_and_diagnostics():
    """dt > odom_max_interval_s：间隙不跨积分，位姿不跳变，诊断计数。"""
    from nav_msgs.msg import Odometry

    master_fd, slave_fd, slave_name = _open_pty()
    node = _start_node_inprocess(
        slave_name, ['odom_enable:=true', 'odom_max_interval_s:=0.15'])
    odom_msgs = []
    node.create_subscription(Odometry, SHADOW_TOPIC, odom_msgs.append, 10)
    executor, spin = _spin_with_executor(node)
    try:
        assert _wait(lambda: node._fd is not None)
        assert _wait(lambda: node.count_subscribers(SHADOW_TOPIC) >= 1)
        os.write(master_fd, _frame(vx=500))
        time.sleep(0.06)
        os.write(master_fd, _frame(vx=500))
        assert _wait(lambda: len(odom_msgs) >= 2)
        x_before = odom_msgs[-1].pose.pose.position.x

        time.sleep(0.5)  # 静默 > 0.15 s：断流间隙

        os.write(master_fd, _frame(vx=500))
        time.sleep(0.06)
        os.write(master_fd, _frame(vx=500))
        assert _wait(lambda: len(odom_msgs) >= 4)
    finally:
        _close(executor, spin, node, master_fd, slave_fd, slave_name)

    x_after = odom_msgs[-1].pose.pose.position.x
    # 只积分了恢复后 ~0.06s 的一段（≈0.03 m）；跨间隙会多出 ≈0.5×0.5=0.25 m
    assert 0.0 < x_after - x_before < 0.10
    d = node._odom_diagnostics
    assert d['rejected_gap'] >= 1, '断流间隙应记入诊断计数'
    assert d['accepted_steps'] >= 2  # gap 恢复后的首帧只重置基准不积分


# ---------------------------------------------------------------- 4. 重连

def test_odom_reconnect_resets_time_base_explicitly():
    """重连显式重置时间基准：重连后等待一段小于间隔阈值的时间，再发非零
    速度首帧——首帧只重建基准、位姿不变，第二帧才开始积分。"""
    from nav_msgs.msg import Odometry

    master_fd, slave_fd, slave_name = _open_pty()
    # 被动 gap 阈值 5 s：下述 ~0.7 s 间隙若不显式重置就会被跨积分
    node = _start_node_inprocess(
        slave_name, ['odom_enable:=true', 'odom_max_interval_s:=5.0'])
    odom_msgs = []
    node.create_subscription(Odometry, SHADOW_TOPIC, odom_msgs.append, 10)
    executor, spin = _spin_with_executor(node)
    try:
        assert _wait(lambda: node._fd is not None)
        assert _wait(lambda: node.count_subscribers(SHADOW_TOPIC) >= 1)
        os.write(master_fd, _frame(vx=500))
        time.sleep(0.06)
        os.write(master_fd, _frame(vx=500))
        assert _wait(lambda: len(odom_msgs) >= 2)
        x_before = odom_msgs[-1].pose.pose.position.x

        time.sleep(0.6)  # 模拟断流（0.6 s < 5 s 被动阈值）
        node._on_serial_reconnected()  # 真实路径：_run 在重新打开后调用
        # 非零速度首帧：只能重建基准，不得产生位移
        os.write(master_fd, _frame(vx=500))
        assert _wait(lambda: len(odom_msgs) >= 3)
        x_first = odom_msgs[2].pose.pose.position.x
        time.sleep(0.06)
        os.write(master_fd, _frame(vx=500))
        assert _wait(lambda: len(odom_msgs) >= 4)
    finally:
        _close(executor, spin, node, master_fd, slave_fd, slave_name)

    x_after = odom_msgs[-1].pose.pose.position.x
    assert x_first == x_before, \
        f'重连后首帧不得产生位移（x_before={x_before} x_first={x_first}）'
    # 第二帧积分了 ~0.06 s ≈ 0.03 m；跨间隙会多出 ≈0.5×0.7=0.35 m
    assert 0.0 < x_after - x_first < 0.10
    d = node._odom_diagnostics
    assert d['explicit_resets'] == 1, '重连应显式重置一次'
    assert d['rejected_gap'] == 0, '0.7s 间隙 < 5s 阈值，不应走被动 gap'
    assert d['accepted_steps'] == 2  # 断流前 1 步 + 重连后第 2 帧起 1 步


# --------------------------------------- 5. 开启状态下的进程退出（SIGINT）

def test_odom_enabled_sigint_clean_exit(tmp_path):
    """开启影子输出的节点进程在持续收帧时收到 SIGINT：正常退出、释放 pty。"""
    package_root = Path(__file__).resolve().parents[1]
    master_fd, slave_fd, slave_name = _open_pty()
    env = dict(os.environ)
    env['PYTHONPATH'] = str(package_root) + os.pathsep + env.get('PYTHONPATH', '')
    env['ROS_DOMAIN_ID'] = env.get('ROS_DOMAIN_ID_TEST_ISOLATED', '188')

    proc = subprocess.Popen(
        [sys.executable, '-m', 'c50c_uart3.uart3_node',
         '--ros-args', '-p', f'device:={slave_name}', '-p', 'odom_enable:=true',
         '-p', 'odom_tf_enable:=true'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
        cwd=str(tmp_path))
    stop_writing = threading.Event()
    writer = threading.Thread(target=_feed_frames, args=(master_fd, stop_writing),
                              daemon=True)
    log_lines = []
    try:
        writer.start()
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
        assert opened, ('节点应在超时前打开伪终端：\n'
                        + b''.join(log_lines).decode(errors='replace'))
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
        rest = proc.stdout.read()
        log_lines.append(rest)
        output = b''.join(log_lines).decode(errors='replace')
        for bad in ('RCLError', 'context is invalid', 'Exception in thread',
                    '读取线程 3 s 内未退出', 'publisher\'s context'):
            assert bad not in output, f'退出日志不应出现 {bad!r}；日志:\n{output}'
        assert proc.returncode == 0, f'应正常退出，实际 {proc.returncode}:\n{output}'
        from c50c_uart3.serial_util import find_device_holders
        assert find_device_holders(slave_name) == [], '退出后不应再持有伪终端'
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


# ------------------- 7. 默认（里程计关闭）模式的重连解析器清理 -------------------

def test_default_mode_reconnect_drops_splice_prefix(tmp_path):
    """默认 odom_enable=false：重连仍必须丢弃解析器残余半帧。

    修复前重连钩子只在里程计开启时被调用，默认模式下断开前的可拼接
    残余会与新数据误拼成通过校验的假帧并污染 IMU 输出。本测试走完整
    真实路径（设备符号链接换接 → 读端 EIO → 重新打开 → 重连钩子），
    断言重连后 IMU 数值只来自完整新帧。"""
    from sensor_msgs.msg import Imu

    package_root = Path(__file__).resolve().parents[1]
    master1, slave1 = os.openpty()
    pts1 = os.ttyname(slave1)
    link = tmp_path / 'uart3'
    os.symlink(pts1, link)

    env = dict(os.environ)
    env['PYTHONPATH'] = str(package_root) + os.pathsep + env.get('PYTHONPATH', '')
    env['ROS_DOMAIN_ID'] = env.get('ROS_DOMAIN_ID_TEST_ISOLATED', '188')
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']  # 测试进程订阅者同域

    proc = subprocess.Popen(
        [sys.executable, '-m', 'c50c_uart3.uart3_node',
         '--ros-args', '-p', f'device:={link}'],  # 默认参数：里程计关闭
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
        cwd=str(tmp_path))
    log_lines = []
    imu_msgs = []
    writer_stop = threading.Event()
    writer = None
    sub_executor = sub_spin = sub_node = None
    master2 = slave2 = None
    try:
        rclpy.init()
        sub_node = rclpy.create_node('default_reconnect_test_sub')
        sub_node.create_subscription(Imu, '/imu/data_raw', imu_msgs.append,
                                     rclpy.qos.qos_profile_sensor_data)
        sub_executor = rclpy.executors.SingleThreadedExecutor()
        sub_executor.add_node(sub_node)
        sub_spin = threading.Thread(target=sub_executor.spin, daemon=True)
        sub_spin.start()

        def _pump():
            for line in proc.stdout:
                log_lines.append(line)

        threading.Thread(target=_pump, daemon=True).start()

        def _log_has(text):
            return any(text.encode() in l for l in log_lines)

        assert _wait(lambda: _log_has('已打开串口'), 15.0), \
            ('节点应在超时前打开伪终端；日志:\n'
             + b''.join(log_lines).decode(errors='replace'))
        assert _wait(lambda: sub_node.count_publishers('/imu/data_raw') >= 1), \
            'IMU 发布器应被发现'

        # 阶段 1：正常收帧
        writer = threading.Thread(
            target=_feed_frames, args=(master1, writer_stop), daemon=True)
        writer.start()
        assert _wait(lambda: len(imu_msgs) >= 2), \
            f'应有 IMU 消息，实际 {len(imu_msgs)}'

        # 阶段 2：断开前留可拼接残余半帧，换接设备后断开旧读端
        writer_stop.set()
        writer.join(timeout=3.0)
        writer = None
        new_frame = build_frame(vx=100, az=0x407D, gz=3755, voltage_mv=4060)
        prefix, new_frame = craft_splice_prefix(new_frame)
        os.write(master1, prefix)
        time.sleep(0.3)  # > select 超时 0.25 s，确保残余已进解析器缓冲
        master2, slave2 = os.openpty()
        pts2 = os.ttyname(slave2)
        new_link = tmp_path / 'uart3.new'
        os.symlink(pts2, new_link)
        os.replace(new_link, link)
        os.close(master1)
        assert _wait(lambda: _log_has('串口已重新打开'), 10.0), \
            ('默认模式也应在日志中看到重连清理；日志:\n'
             + b''.join(log_lines).decode(errors='replace'))
        reconnect_line = next(l for l in log_lines if '串口已重新打开'.encode() in l)
        assert '里程计'.encode() not in reconnect_line, \
            '默认关闭里程计的重连日志不应声称里程计在等待首帧'

        # 阶段 3：新设备上发完整新帧——IMU 数值必须只来自它
        os.write(master2, new_frame)
        prev_len = len(imu_msgs)
        assert _wait(lambda: len(imu_msgs) >= prev_len + 1), \
            ('重连后应继续收到 IMU 消息；日志:\n'
             + b''.join(log_lines).decode(errors='replace'))
        first_after = imu_msgs[prev_len]
        import math
        # 完整新帧（craft 只改了 az 低字节为 0x7D、重算 BCC）：
        az_new = int.from_bytes(new_frame[12:14], 'big', signed=True)
        gz_new = int.from_bytes(new_frame[18:20], 'big', signed=True)
        # 若残余未清理：拼接假帧的 az/gz 取自 vx 与 voltage 字节
        az_fake = int.from_bytes(new_frame[2:4], 'big', signed=True)
        gz_fake = int.from_bytes(new_frame[8:10], 'big', signed=True)
        assert az_fake != az_new and gz_fake != gz_new, '判别值不应重合（反例构造失效）'
        assert math.isclose(first_after.linear_acceleration.z,
                            az_new / 1671.84, rel_tol=1e-9), \
            '重连后 IMU 加速度应来自完整新帧，而不是误拼假帧'
        assert math.isclose(first_after.angular_velocity.z,
                            gz_new / 3754.9, rel_tol=1e-3), \
            '重连后 IMU 角速度应来自完整新帧，而不是误拼假帧'

        # 阶段 4：SIGINT 干净退出
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5.0)
            raise AssertionError('SIGINT 后节点未在 15 s 内退出')
        output = b''.join(log_lines).decode(errors='replace')
        for bad in ('RCLError', 'context is invalid', 'Exception in thread',
                    '读取线程 3 s 内未退出'):
            assert bad not in output, \
                f'退出日志不应出现 {bad!r}；日志:\n{output}'
        assert proc.returncode == 0, \
            f'应正常退出，实际 {proc.returncode}:\n{output}'
        from c50c_uart3.serial_util import find_device_holders
        assert find_device_holders(pts2) == [], '退出后不应再持有伪终端'
    finally:
        writer_stop.set()
        if writer is not None and writer.is_alive():
            writer.join(timeout=2.0)
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=5.0)
        if sub_executor is not None:
            sub_executor.shutdown()
        if sub_spin is not None:
            sub_spin.join(timeout=3.0)
        if sub_node is not None:
            sub_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        for fd in (master1, slave1, master2, slave2):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass


# ----------------------- 8. 候选 TF 输出（odom_tf_enable，默认关闭） -------

def test_odom_tf_default_off_no_tf_publisher():
    """默认（含 odom_enable=true 但未开 TF）：节点不创建 /tf 发布器。"""
    from nav_msgs.msg import Odometry

    master_fd, slave_fd, slave_name = _open_pty()
    node = _start_node_inprocess(slave_name, ['odom_enable:=true'])
    odom_msgs = []
    node.create_subscription(Odometry, SHADOW_TOPIC, odom_msgs.append, 10)
    executor, spin = _spin_with_executor(node)
    try:
        assert _wait(lambda: node._fd is not None)
        assert _wait(lambda: node.count_subscribers(SHADOW_TOPIC) >= 1)
        os.write(master_fd, _frame(vx=500))
        assert _wait(lambda: len(odom_msgs) >= 1), '影子消息应照常发布'
        assert node._odom_tf_publisher is None
        assert node.count_publishers('/tf') == 0, '未开 TF 开关不应有 /tf 发布器'
        assert node._tf_existing_publishers is None
    finally:
        _close(executor, spin, node, master_fd, slave_fd, slave_name)


def test_odom_tf_without_odom_enable_rejected():
    """odom_tf_enable=true 但 odom_enable=false：启动即拒绝（TF 无积分来源）。"""
    from c50c_uart3.uart3_node import C50CUart3Node

    rclpy.init(args=['--ros-args', '-p', 'device:=/nonexistent-uart3',
                     '-p', 'odom_tf_enable:=true'])
    try:
        with pytest.raises(RuntimeError, match='odom_enable'):
            C50CUart3Node()
    finally:
        rclpy.shutdown()


def test_odom_tf_same_frame_pose_stamp_as_message():
    """开启后：TF 与 Odometry 消息同帧、同位姿、同 ROS 时间戳。"""
    import math

    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage

    master_fd, slave_fd, slave_name = _open_pty()
    node = _start_node_inprocess(
        slave_name, ['odom_enable:=true', 'odom_tf_enable:=true'])
    odom_msgs, tf_msgs = [], []
    node.create_subscription(Odometry, SHADOW_TOPIC, odom_msgs.append, 10)
    node.create_subscription(TFMessage, '/tf', tf_msgs.append, 10)
    executor, spin = _spin_with_executor(node)
    try:
        assert _wait(lambda: node._fd is not None)
        assert _wait(lambda: node.count_subscribers(SHADOW_TOPIC) >= 1)
        assert _wait(lambda: node.count_subscribers('/tf') >= 1)
        for i in range(4):
            # 有移动也有转动：vx=250mm/s、wz=250mrad/s
            os.write(master_fd, _frame(vx=250, wz=250))
            time.sleep(0.06)
        assert _wait(lambda: len(odom_msgs) >= 4), f'实际 {len(odom_msgs)}'
        assert _wait(lambda: len(tf_msgs) >= 4), f'实际 {len(tf_msgs)}'
    finally:
        _close(executor, spin, node, master_fd, slave_fd, slave_name)

    # TFMessage 每条含一个 TransformStamped；按时间戳与 Odometry 配对
    tfs = [t.transforms[0] for t in tf_msgs]
    stamps_tf = [t.header.stamp.sec + t.header.stamp.nanosec * 1e-9 for t in tfs]
    stamps_od = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in odom_msgs]
    assert stamps_tf == sorted(stamps_tf) and stamps_od == sorted(stamps_od)
    # 每条 TF 都能在 Odometry 序列里找到同时间戳的同位姿记录
    by_stamp = {round(s, 6): m for s, m in zip(stamps_od, odom_msgs)}
    assert len(by_stamp) >= 3
    for t, s in zip(tfs, stamps_tf):
        m = by_stamp[round(s, 6)]
        assert t.header.frame_id == m.header.frame_id == 'odom'
        assert t.child_frame_id == m.child_frame_id == 'base_link'
        assert t.transform.translation.x == m.pose.pose.position.x
        assert t.transform.translation.y == m.pose.pose.position.y
        assert t.transform.translation.z == m.pose.pose.position.z
        q, qo = t.transform.rotation, m.pose.pose.orientation
        assert (q.x, q.y, q.z, q.w) == (qo.x, qo.y, qo.z, qo.w)
    # 位姿确有变化（积分在动，TF 不是恒等）：yaw 随 wz>0 增大
    yaw = [2 * math.atan2(t.transform.rotation.z, t.transform.rotation.w)
           for t in tfs]
    assert yaw[-1] > yaw[0] >= 0.0
    # 守卫/提示字段
    assert node._tf_existing_publishers == 0  # 隔离域内启动时无其他 TF 发布者


def test_odom_tf_conflict_hint_on_existing_tf_publisher():
    """受控互斥检查：启动时已有其他 TF 发布者（模拟恒等 odom TF）→
    记录既有发布者数量并告警。该检查是尽力而为的配置审查提示，
    无法消除所有外部发布者竞争。"""
    from tf2_msgs.msg import TFMessage
    from tf2_ros import StaticTransformBroadcaster
    from geometry_msgs.msg import TransformStamped

    master_fd, slave_fd, slave_name = _open_pty()
    rclpy.init(args=['--ros-args', '-p', f'device:={slave_name}',
                     '-p', 'odom_enable:=true', '-p', 'odom_tf_enable:=true'])
    try:
        # 先制造一个既有 TF 发布者（等同 use_identity_odom:=true 的静态 TF）
        other = rclpy.create_node('fake_identity_odom_tf')
        sb = StaticTransformBroadcaster(other)
        st = TransformStamped()
        st.header.frame_id = 'odom'
        st.child_frame_id = 'base_link'
        sb.sendTransform(st)  # 恒等变换
        def _spin():
            rclpy.spin(other)
        spin_thread = threading.Thread(target=_spin, daemon=True)
        spin_thread.start()
        time.sleep(0.5)

        from c50c_uart3.uart3_node import C50CUart3Node
        node = C50CUart3Node()
        assert node._tf_existing_publishers is not None \
            and node._tf_existing_publishers >= 1, \
            '启动时应记录到既有 TF 发布者（互斥提示的触发依据）'
        node.destroy_node()
        other.destroy_node()
    finally:
        rclpy.shutdown()
        os.close(master_fd)
        os.close(slave_fd)


def _feed_frames(master_fd, stop_event, period_s=0.05):
    """持续向 pty 主端写入使能态状态帧，模拟固件 20 Hz 发送。"""
    frame = _frame(vx=500, wz=100)
    while not stop_event.is_set():
        try:
            os.write(master_fd, frame)
        except OSError:
            return
        stop_event.wait(period_s)


# --------------------------------- 6. 真实重连路径（设备消失 → 重新打开）

def test_shadow_topic_guard_rejects_formal_odom_name():
    """影子话题守卫：odom_topic 参数值或外部 --remap 解析为全局 /odom 时，
    节点拒绝启动（RuntimeError），不创建任何发布器。串口不会被打开。"""
    from c50c_uart3.uart3_node import C50CUart3Node

    cases = [
        ('参数值直指 /odom',
         ['-p', 'device:=/nonexistent-uart3', '-p', 'odom_enable:=true',
          '-p', 'odom_topic:=/odom']),
        ('外部重映射到 /odom',
         ['-p', 'device:=/nonexistent-uart3', '-p', 'odom_enable:=true',
          '--remap', 'odom_candidate:=/odom']),
        ('相对名 + 重映射到 /odom',
         ['-p', 'device:=/nonexistent-uart3', '-p', 'odom_enable:=true',
          '-p', 'odom_topic:=odom_candidate2',
          '--remap', 'odom_candidate2:=/odom']),
    ]
    for label, args in cases:
        rclpy.init(args=['--ros-args'] + args)
        try:
            with pytest.raises(RuntimeError, match='/odom'):
                C50CUart3Node()
        finally:
            rclpy.shutdown()
        # 非真实设备路径 + 守卫在打开串口之前触发：无需清理伪终端


def test_odom_reconnect_via_device_swap_no_cross_gap(tmp_path):
    """真实验证 _run 的重连路径：设备消失（读端 EIO）→ 节点重新打开设备
    → 显式重置积分时间基准 → 不跨未知间隙积分。

    通过换接符号链接指向的新 pty 模拟"重新插上串口"；被动 gap 阈值调大
    （10 s），使显式重置成为唯一保护，若接线缺失位姿会跨 ~1 s 间隙跳变。
    隔离 ROS 域内子进程真实节点 + 测试进程订阅者，不接触真实 UART3。"""
    from nav_msgs.msg import Odometry

    package_root = Path(__file__).resolve().parents[1]
    master1, slave1 = os.openpty()
    pts1 = os.ttyname(slave1)
    link = tmp_path / 'uart3'
    os.symlink(pts1, link)

    env = dict(os.environ)
    env['PYTHONPATH'] = str(package_root) + os.pathsep + env.get('PYTHONPATH', '')
    env['ROS_DOMAIN_ID'] = env.get('ROS_DOMAIN_ID_TEST_ISOLATED', '188')
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']  # 测试进程订阅者同域

    proc = subprocess.Popen(
        [sys.executable, '-m', 'c50c_uart3.uart3_node',
         '--ros-args', '-p', f'device:={link}',
         '-p', 'odom_enable:=true', '-p', 'odom_max_interval_s:=10.0'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
        cwd=str(tmp_path))
    log_lines = []
    sub_msgs = []
    writer_stop = threading.Event()
    writer_stop2 = threading.Event()
    writer = None
    sub_executor = sub_spin = sub_node = None
    master2 = slave2 = None
    try:
        rclpy.init()
        sub_node = rclpy.create_node('odom_reconnect_test_sub')
        sub_node.create_subscription(Odometry, SHADOW_TOPIC,
                                     sub_msgs.append, 10)
        sub_executor = rclpy.executors.SingleThreadedExecutor()
        sub_executor.add_node(sub_node)
        sub_spin = threading.Thread(target=sub_executor.spin, daemon=True)
        sub_spin.start()

        def _pump():
            for line in proc.stdout:
                log_lines.append(line)

        pump = threading.Thread(target=_pump, daemon=True)
        pump.start()

        def _log_has(text):
            return any(text.encode() in l for l in log_lines)

        assert _wait(lambda: _log_has('已打开串口'), 15.0), \
            ('节点应在超时前打开伪终端；日志:\n'
             + b''.join(log_lines).decode(errors='replace'))
        assert _wait(lambda: sub_node.count_publishers(SHADOW_TOPIC) >= 1), \
            '影子话题发布器应被发现'

        # 阶段 1：正常收帧
        writer = threading.Thread(
            target=_feed_frames, args=(master1, writer_stop), daemon=True)
        writer.start()
        assert _wait(lambda: len(sub_msgs) >= 3), \
            f'应有影子话题消息，实际 {len(sub_msgs)}'
        x_before = sub_msgs[-1].pose.pose.position.x

        # 阶段 2：设备换接（备好新设备并换接链接后，停写、断开旧读端）
        writer_stop.set()
        writer.join(timeout=3.0)
        writer = None
        # 断开前留一个残余半帧（含旧 vx=7777 的可拼接前缀）：重连时必须
        # 被丢弃，不得与新数据误拼成通过校验的假帧
        splice_prefix, splice_frame = craft_splice_prefix(
            build_frame(vx=100, az=0x407D, voltage_mv=4060))
        os.write(master1, splice_prefix)
        time.sleep(0.3)  # > select 超时 0.25 s，确保节点已把残余读进解析器
        master2, slave2 = os.openpty()
        pts2 = os.ttyname(slave2)
        new_link = tmp_path / 'uart3.new'
        os.symlink(pts2, new_link)
        os.replace(new_link, link)
        os.close(master1)
        # 节点读到 EIO → 关闭 → 重新打开 link（已指向 pts2）→ 重连钩子
        assert _wait(lambda: _log_has('串口已重新打开'), 10.0), \
            ('应在日志中看到重连重置；日志:\n'
             + b''.join(log_lines).decode(errors='replace'))

        # 阶段 3：新设备上继续收帧（用全新 stop 事件——writer_stop 已置位）。
        # 首帧发完整 splice_frame（vx=100）：若解析器未在重连时丢弃残余，
        # "残余前缀 + 本帧前 14 字节"会拼成 vx=7777 的假帧被误接受。
        os.write(master2, splice_frame)
        prev_len = len(sub_msgs)
        assert _wait(lambda: len(sub_msgs) >= prev_len + 1), \
            ('重连后应继续收到影子话题消息；节点日志:\n'
             + b''.join(log_lines).decode(errors='replace'))
        first_after = sub_msgs[prev_len]
        assert first_after.twist.twist.linear.x == pytest.approx(0.1), \
            ('重连后首帧 vx 应来自完整新帧(0.1 m/s)，而不是误拼假帧(7.777)；'
             '节点日志:\n' + b''.join(log_lines).decode(errors='replace'))
        writer = threading.Thread(
            target=_feed_frames, args=(master2, writer_stop2), daemon=True)
        writer.start()
        prev_len = len(sub_msgs)
        assert _wait(lambda: len(sub_msgs) >= prev_len + 3), \
            ('重连后应继续收到影子话题消息；节点日志:\n'
             + b''.join(log_lines).decode(errors='replace'))
        writer_stop2.set()
        writer.join(timeout=3.0)
        writer = None

        x_after = sub_msgs[-1].pose.pose.position.x
        # 间隙 ~1 s @ 0.5 m/s：跨积分会多出 ≈0.5 m；正确处理只多 ~0.1 m
        assert 0.0 <= x_after - x_before < 0.30, \
            f'重连间隙不应被跨积分（x_before={x_before:.3f} x_after={x_after:.3f}）'

        # 阶段 4：SIGINT 干净退出
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5.0)
            raise AssertionError('SIGINT 后节点未在 15 s 内退出')
        output = b''.join(log_lines).decode(errors='replace')
        for bad in ('RCLError', 'context is invalid', 'Exception in thread',
                    '读取线程 3 s 内未退出', '里程计积分遇到断流间隙'):
            assert bad not in output, \
                f'日志不应出现 {bad!r}（间隙应由显式重置处理）；日志:\n{output}'
        assert proc.returncode == 0, \
            f'应正常退出，实际 {proc.returncode}:\n{output}'
        from c50c_uart3.serial_util import find_device_holders
        assert find_device_holders(pts2) == [], '退出后不应再持有伪终端'
    finally:
        writer_stop.set()
        writer_stop2.set()
        if writer is not None and writer.is_alive():
            writer.join(timeout=2.0)
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=5.0)
        if sub_executor is not None:
            sub_executor.shutdown()
        if sub_spin is not None:
            sub_spin.join(timeout=3.0)
        if sub_node is not None:
            sub_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        for fd in (master1, slave1, master2, slave2):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
