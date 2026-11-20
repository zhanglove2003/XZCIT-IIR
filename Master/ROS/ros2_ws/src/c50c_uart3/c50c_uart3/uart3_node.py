#!/usr/bin/env python3
"""C50C UART3 状态帧 → ROS 2 IMU 只读桥接节点。

行为约定（详见包 README 与 task-results 交付报告）：

- 订阅对象只有串口：本节点只读打开 UART3 设备，不发送任何命令。
  **串口占用语义（重要）**：``TIOCEXCL`` 只能阻止**本节点打开之后**的
  新打开请求（后开者得到 EBUSY），**不能关闭或发现所有已有占用者**；
  且本节点的 termios 配置（raw 模式、波特率）作用于 tty 线路本身，
  会被所有共享该设备的进程感知。因此**现场启动顺序必须是：先关闭
  UART3 监视器和其他读取程序，再启动本节点**；已有进程打开设备时，
  不能把本节点当作无影响地并行读取。节点每次（重）打开串口前会做一次
  ``/proc`` 扫描提示疑似占用者——那是尽力而为的提示（有竞态、覆盖不全），
  不是**无竞态的独占保证。
- 发布 ``~/imu/data_raw``（默认 ``/imu/data_raw``，sensor_msgs/Imu）：
  - 姿态：固件不提供四元数，四元数保持全零且
    ``orientation_covariance[0] = -1``（按消息约定姿态不可用）。
  - 角速度：``FlagStop != 0`` 时固件将陀螺 Z 强制置零，该帧角速度整体
    不可用——测量值置零并置 ``angular_velocity_covariance[0] = -1``。
  - 线加速度：按 ±2 g 量程换算为 m/s²。注意 Z 通道含固件补回的 +1 g
    基准，X/Y 的静置重力分量被上电零偏扣除吸收（上电时假定水平、Z 朝天），
    即本话题是"相对上电姿态的线性加速度 + Z 重力"，REP 145 意义上的
    raw 约定以此为准，现场需复核。
- 时间戳：帧内无采样时间戳，使用主机接收时刻（节点时钟 ``now()``）。
  20 Hz 发送 + 串口/调度延迟，抖动不补偿——这是已声明的限制。
- 上电校准：固件上电前 10 s 采集零偏，期间帧内为未扣零偏原始值，且
  接收端无从判断 C50C 何时上电。节点**不猜测**校准窗口是否结束，仅照实
  转发并记录该未确认状态。
- 底盘速度字段（编码器里程计 vx/vy/wz）：经 ``OdomCandidate`` 接入**默认
  关闭**的里程计候选输出（``odom_enable=False``）。开启后仅发布可配置的
  **影子话题**（默认 ``/odom_candidate``，nav_msgs/Odometry），积分使用
  主机单调接收时刻、消息戳用节点（ROS）时钟；串口重连（首帧只重建基准、
  不产生位移，并丢弃解析器残余半帧）或 >``odom_max_interval_s`` 的断流
  间隙均不跨积分。**仍不发布正式 `/odom`、不发布任何 TF**：``odom_topic``
  参数与外部重映射解析出的最终话题名若为全局 ``/odom``，节点在启动时
  拒绝启动（``RuntimeError``），不会静默发布；带命名空间的同名话题
  （如 ``/ns/odom``）不在守卫范围，需启动配置审查。
  速度标度（轮径/轮距/编码器方向）未标定，协方差为占位值——均为已声明
  限制，不是标定结果。角速度来源是帧 offset 6-7 的**编码器差速解算 wz**，
  与 IMU 陀螺 Z（offset 18-19，FlagStop≠0 时被固件钳零）严格区分、不混用。
- 可选候选 TF（``odom_tf_enable``，默认关闭）：与候选 Odometry 同帧、
  同位姿、同 ROS 时间戳发布 ``odom→base_link`` 变换（TF 直接取自已构造
  消息，无第二套积分器）。**与建图 launch 的 ``use_identity_odom:=true``
  互斥**——不得让两个发布者同时用于同一条 TF 边；启动时对 /tf 与
  /tf_static 既有发布者数量做尽力而为的提示，无法消除所有外部竞争。
  这是候选位姿的 TF，不是真实里程计 TF（偏航方向/标度未验收）。
"""
from __future__ import annotations

import fcntl
import os
import select
import threading
import termios
import time

import rclpy
from c50c_odom.odometry import PlanarOdomIntegrator
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, Range
from tf2_ros import TransformBroadcaster

from .chassis_cmd.arbiter import FIRMWARE_PROTOCOLS, PROTOCOL_BASELINE
from .chassis_cmd.bridge import DEFAULT_FIRMWARE_PROTOCOL
from .conversions import imu_measurements
from .frame_parser import StatusFrame, StatusFrameParser
from .gyro_bias import GyroBiasCorrector, GyroZBiasEstimator
from .odom_msg import OdomCandidate, OdomCandidateConfig
from .serial_util import find_device_holders

BAUD_RATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
}


class C50CUart3Node(Node):
    def __init__(self) -> None:
        super().__init__('c50c_uart3_node')
        self.declare_parameter('device', '')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        # 协方差为占位参数：固件不提供任何协方差，待现场标定后修改。
        self.declare_parameter('accel_variance', 0.01)
        self.declare_parameter('gyro_variance', 0.001)
        # 陀螺 Z 轴静止残差修正：默认关闭（行为与既有版本完全一致）。
        # 只修正 angular_velocity.z，估计的是固件扣除之后的残差，判据见
        # gyro_bias 模块文档。开启前须经现场评审。
        self.declare_parameter('gyro_bias_enable', False)
        self.declare_parameter('gyro_bias_window_s', 30.0)
        self.declare_parameter('gyro_bias_max_rad_s', 0.01)
        # 帧间隔监测：标称 20 Hz（50 ms），超时仅告警，无序列号无法精确计数丢帧。
        self.declare_parameter('frame_gap_warn_s', 0.25)
        # 串口看门狗：USB CDC 断开（固件断电/重枚举）不一定唤醒 select——
        # 旧 fd 可能永远静默（2026-11-14 实测：桥进程活着但状态帧断流，
        # 只能人工重启服务）。静默达到该阈值即主动退出读循环，由 _run
        # 重开串口；0 = 关闭看门狗。
        self.declare_parameter('serial_starve_reopen_s', 5.0)

        # ---- 里程计候选（影子）输出：默认关闭，开启也只发影子话题 ----
        # 协方差/速度标度均为占位或未标定值——见 odom_msg 模块文档。
        self.declare_parameter('odom_enable', False)
        self.declare_parameter('odom_topic', 'odom_candidate')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('odom_child_frame_id', 'base_link')
        # 断流间隙阈值：与固件 rear_odom_step 的 dt>500ms 丢弃语义一致
        self.declare_parameter('odom_max_interval_s', 0.5)
        self.declare_parameter('odom_pose_variance_x', 0.01)
        self.declare_parameter('odom_pose_variance_y', 0.01)
        self.declare_parameter('odom_pose_variance_yaw', 0.01)
        self.declare_parameter('odom_twist_variance_linear', 0.01)
        self.declare_parameter('odom_twist_variance_angular', 0.01)
        # 未测量维度（pose z/roll/pitch、twist vy/vz/vroll/vpitch）的方差：
        # 大方差表达"不可融合"，不是标定值，详见 odom_msg 模块文档
        self.declare_parameter('odom_unmeasured_variance', 1e6)
        # 候选 TF 输出（odom→base_link）：默认关闭；必须与 odom_enable 同开。
        # 与建图 launch 的 use_identity_odom:=true 互斥——不得两个发布者
        # 同时用于同一条 TF 边（启动期提示为尽力而为，见下）。
        self.declare_parameter('odom_tf_enable', False)

        # 直行命令桥（默认关闭）：本节点是 UART3 的唯一所有者；开启后
        # 由本节点用**同一个 fd** 写入命令帧。未获授权不得开启。
        self.declare_parameter('drive_enable', False)
        self.declare_parameter('drive_input_enable', False)
        self.declare_parameter('drive_tick_period_s', 0.02)
        self.declare_parameter('drive_fd_authorised', False)
        # 固件发送方言（显式开关）：'v3'（默认，保持既有行为）或 'baseline'
        # （已交付 2026-11-05 基线：无会话帧、无 ack 位、无序号）。
        # 默认不开基线，避免改变既有 v3 路径；没有自动探测、没有回退。
        self.declare_parameter('drive_firmware_protocol',
                               DEFAULT_FIRMWARE_PROTOCOL)

        device = self.get_parameter('device').value
        if not device:
            raise RuntimeError('c50c_uart3_node 需要 device 参数指向 UART3 串口设备')
        self._baudrate = int(self.get_parameter('baudrate').value)
        if self._baudrate not in BAUD_RATES:
            raise RuntimeError(f'不支持的波特率 {self._baudrate}，可用: {sorted(BAUD_RATES)}')
        self._device = device
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._accel_variance = float(self.get_parameter('accel_variance').value)
        self._gyro_variance = float(self.get_parameter('gyro_variance').value)
        self._gyro_bias = None
        self._gyro_bias_logged = None
        if bool(self.get_parameter('gyro_bias_enable').value):
            self._gyro_bias = GyroBiasCorrector(GyroZBiasEstimator(
                window_s=float(self.get_parameter('gyro_bias_window_s').value),
                max_bias=float(self.get_parameter('gyro_bias_max_rad_s').value)))
        self._frame_gap_warn_s = float(self.get_parameter('frame_gap_warn_s').value)
        self._starve_reopen_s = float(
            self.get_parameter('serial_starve_reopen_s').value)

        self._publisher = self.create_publisher(Imu, 'imu/data_raw',
                                                qos_profile_sensor_data)
        self._chassis_publisher = self.create_publisher(DiagnosticArray,
                                                        '/diagnostics', 10)
        # [C50C 2026-11-15] 前置 HC-SR04 超声波（50 字节状态帧 [46..47]）。
        # frame_id 为传感器自身坐标系；base_link→ultrasonic_front 的静态 TF
        # 随 Nav2 集成另行部署（安装位车头正中、离地约 5 cm，待实测补外参）。
        self.declare_parameter('ultrasonic_frame_id', 'ultrasonic_front')
        self.declare_parameter('ultrasonic_max_range_m', 4.5)
        self._ultrasonic_frame_id = str(
            self.get_parameter('ultrasonic_frame_id').value)
        self._ultrasonic_max_range_m = float(
            self.get_parameter('ultrasonic_max_range_m').value)
        self._ultrasonic_publisher = self.create_publisher(
            Range, 'ultrasonic_front', qos_profile_sensor_data)
        self._parser = StatusFrameParser()
        self._last_frame_monotonic: float | None = None

        # 里程计候选：默认关闭（无发布器、无积分状态）；开启后只发影子话题
        self._odom_enabled = bool(self.get_parameter('odom_enable').value)
        odom_tf_enable = bool(self.get_parameter('odom_tf_enable').value)
        if odom_tf_enable and not self._odom_enabled:
            raise RuntimeError(
                'odom_tf_enable=true 需要 odom_enable=true：候选 TF 由同一'
                '候选积分位姿派生，没有候选积分就没有 TF')
        self._odom_candidate: OdomCandidate | None = None
        self._odom_publisher = None
        self._odom_tf_publisher = None
        self._odom_diagnostics: dict | None = None
        self._tf_existing_publishers: int | None = None
        self._serial_open_count = 0
        self._drive_enabled = bool(self.get_parameter('drive_enable').value)
        self._drive_input_enable = bool(
            self.get_parameter('drive_input_enable').value)
        self._drive_tick_period_s = float(
            self.get_parameter('drive_tick_period_s').value)
        self._drive_fd_authorised = bool(
            self.get_parameter('drive_fd_authorised').value)
        # 发送方言在启动时校验一次：非法值直接拒绝启动，运行期不再判断，
        # 也不存在"失败后退回 v3"的分支（回退会让两种方言证据混用）。
        self._drive_firmware_protocol = str(
            self.get_parameter('drive_firmware_protocol').value)
        if self._drive_firmware_protocol not in FIRMWARE_PROTOCOLS:
            raise RuntimeError(
                f'drive_firmware_protocol={self._drive_firmware_protocol!r} '
                f'不是已知固件方言；可选 {sorted(FIRMWARE_PROTOCOLS)}')
        self._drive_bridge = None
        self._drive_port = None
        self._drive_timer = None
        self._drive_handshake = None
        # guard 区（运动许可/四路仲裁/方向-帧龄停车）：与直行命令桥共用
        # 同一个 FdPort；每（重）连周期在 _drive_cycle 里懒建，见
        # chassis_cmd/guard_wiring.py 的写入门。
        self._guard_wiring = None
        if self._odom_enabled:
            odom_topic = str(self.get_parameter('odom_topic').value)
            # 影子话题守卫：解析(含参数值与外部 --remap 规则)后的最终话题名
            # 若为全局正式话题 /odom,拒绝启动而不是发布——保证任何配置/重映射
            # 组合都不会把影子输出落到正式 /odom(要么发影子名,要么拒绝启动)。
            # 带命名空间的同名话题(如 /ns/odom)不在本守卫范围,需启动配置审查。
            resolved_topic = self.resolve_topic_name(odom_topic)
            if resolved_topic == '/odom':
                raise RuntimeError(
                    f'odom_topic={odom_topic!r} 解析为正式话题 {resolved_topic}：'
                    '本轮不发布正式 /odom，拒绝启动；请改用影子话题名'
                    '（如 odom_candidate）')
            config = OdomCandidateConfig(
                frame_id=str(self.get_parameter('odom_frame_id').value),
                child_frame_id=str(self.get_parameter('odom_child_frame_id').value),
                pose_covariance_x=float(
                    self.get_parameter('odom_pose_variance_x').value),
                pose_covariance_y=float(
                    self.get_parameter('odom_pose_variance_y').value),
                pose_covariance_yaw=float(
                    self.get_parameter('odom_pose_variance_yaw').value),
                twist_covariance_linear=float(
                    self.get_parameter('odom_twist_variance_linear').value),
                twist_covariance_angular=float(
                    self.get_parameter('odom_twist_variance_angular').value),
                unmeasured_variance=float(
                    self.get_parameter('odom_unmeasured_variance').value),
            )
            integrator = PlanarOdomIntegrator(
                max_interval_s=float(
                    self.get_parameter('odom_max_interval_s').value))
            self._odom_candidate = OdomCandidate(config, integrator)
            # 影子话题用默认可靠 QoS：20 Hz 低速率，便于 rviz/EKF 工具直接订阅
            self._odom_publisher = self.create_publisher(Odometry, odom_topic, 10)
            self._odom_topic = odom_topic

            # ---- 候选 TF 输出（odom_tf_enable，默认关闭）----
            # 与候选 Odometry 同帧、同位姿、同 ROS 时间戳：TF 直接由已构造的
            # 消息派生，不引入第二套积分器。开启时是 odom→base_link 边的
            # 一个发布者——不得与 use_identity_odom:=true 等其他同边发布者
            # 同时使用（见启动期尽力而为的竞争提示，无法消除所有外部竞争）。
            if odom_tf_enable:
                # 尽力而为的竞争提示：创建自己的发布器之前统计 /tf 与
                # /tf_static 上的既有发布者数量。无法判断它们发布哪条边，
                # 也不能消除之后加入的外部发布者——只作启动配置审查提示。
                existing_tf_pubs = (self.count_publishers('/tf')
                                    + self.count_publishers('/tf_static'))
                self._tf_existing_publishers = existing_tf_pubs
                if existing_tf_pubs:
                    self.get_logger().warn(
                        f'启动时已有 {existing_tf_pubs} 个 TF 发布者'
                        '（/tf + /tf_static 合计）：无法判断其中是否已有人发布'
                        f' {config.frame_id}→{config.child_frame_id}；若同时启用'
                        '恒等里程计 TF（如建图 use_identity_odom:=true），会有'
                        '两个发布者竞争同一条 TF 边——两者必须互斥，不能同时'
                        '用于同一条边')
                self._odom_tf_publisher = TransformBroadcaster(self)
                self.get_logger().info(
                    f'候选 TF 输出开启：随影子话题同帧发布 '
                    f'{config.frame_id}→{config.child_frame_id}（与 Odometry '
                    '同位姿、同时间戳）；不是真实里程计 TF')
            self.get_logger().info(
                f'里程计候选输出开启：影子话题 {odom_topic}；积分用主机单调接收'
                '时刻，消息戳用节点时钟；速度标度未标定、协方差为占位值；'
                '不发布 /odom'
                + ('' if odom_tf_enable else '、不发 TF'))

        self._stop_event = threading.Event()
        self._fd: int | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ 串口
    def _open_serial(self) -> int:
        """打开串口并声明独占（TIOCEXCL），8N1、无流控。

        打开模式取决于 ``drive_enable``（默认关闭）：

        - 关闭（默认）：``O_RDONLY``，与历史行为一致，本节点只读不写；
        - 开启：``O_RDWR``。写入端（命令桥 / 会话握手）必须与读循环共用
          同一个 fd —— 只读 fd 上的 ``os.write`` 必然失败（EBADF），而上机
          前返工前的版本正好踩了这一点。共用 fd 也保证"一个设备只有一个
          进程打开"：本节点仍是唯一所有者，不新增任何打开者。

        注意：TIOCEXCL 只阻止之后的新打开请求，不能排除打开在本节点之前
        的进程；termios 配置修改线路，对共享设备的已有进程可见。
        """
        self._warn_if_device_in_use()
        flags = os.O_RDWR if self._drive_enabled else os.O_RDONLY
        fd = os.open(self._device, flags | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.ioctl(fd, termios.TIOCEXCL)
            attrs = termios.tcgetattr(fd)
            attrs[0] = attrs[1] = attrs[3] = 0  # iflag/oflag/lflag: raw
            attrs[2] = (attrs[2]
                        & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB
                            | getattr(termios, 'CRTSCTS', 0))
                        | termios.CS8 | termios.CREAD | termios.CLOCAL)
            attrs[4] = attrs[5] = BAUD_RATES[self._baudrate]
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIFLUSH)
        except Exception:
            os.close(fd)
            raise
        return fd

    def _warn_if_device_in_use(self) -> None:
        """尽力而为的占用提示（适用范围见 serial_util 模块文档）：
        扫描 /proc 提示疑似占用者，有竞态、覆盖不全，不是独占保证；
        真正的独占语义只有 TIOCEXCL 对之后新打开的限制。"""
        holders = find_device_holders(self._device)
        if holders:
            self.get_logger().warn(
                f'检测到其他进程可能已打开 {self._device} '
                f'(PID: {", ".join(holders)})；该检查有竞态且覆盖不全，'
                '仅作提示。请先关闭 UART3 监视器/其他读取程序，再启动本节点。')

    # ------------------------------------------------------------ 接收主循环
    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._fd = self._open_serial()
            except OSError as exc:
                self.get_logger().error(
                    f'打开串口 {self._device} 失败: {exc}；2 s 后重试')
                if self._stop_event.wait(2.0):
                    return
                continue
            mode = '读写' if self._drive_enabled else '只读'
            self.get_logger().info(
                f'已打开串口并设置后续打开限制 {self._device} @ '
                f'{self._baudrate} 8N1（{mode}）')
            self._serial_open_count += 1
            if self._serial_open_count > 1:
                # 与里程计开关无关：解析器残余在任何模式下都必须丢弃
                self._on_serial_reconnected()
            # 重连之后才挂桥：这样桥收到的是"已重连"的状态，而不是在
            # 旧状态上新建后再补一次重连通知。
            self._attach_drive_bridge()
            try:
                self._read_loop()
            except OSError as exc:
                self.get_logger().error(f'串口读取异常: {exc}；尝试重新打开')
            except Exception as exc:  # noqa: BLE001——先记录再退出线程，不做无提示死亡
                self.get_logger().error(f'接收线程异常退出: {exc!r}')
                return
            finally:
                if self._fd is not None:
                    # 先解除独占再关闭：TIOCEXCL 不随 close 自动清除（内核
                    # 不在 tty_release 里复位），不解除的话本节点自己下一次
                    # open 会被 EBUSY 挡住（pty 上实测；USB 设备重枚举虽然
                    # 是新 tty 实例不受影响，但显式解除在语义上总是对的）
                    try:
                        fcntl.ioctl(self._fd, termios.TIOCNXCL)
                    except OSError:
                        pass
                    os.close(self._fd)
                    self._fd = None

    def _reset_drive_handshake(self) -> None:
        """串口重开后作废会话证据：必须重新在线上读到会话帧。"""
        if self._drive_handshake is None:
            return
        from .chassis_cmd.handshake import SessionHandshake
        self._drive_handshake = SessionHandshake(
            self._drive_bridge.chain._arbiter,     # noqa: SLF001
            clock=time.monotonic)

    def _on_serial_reconnected(self) -> None:
        """串口（重新）打开成功且非首次时调用，与里程计开关无关。

        - 解析器残余（任何模式都执行）：断开前的半帧字节与新连接的数据
          可能误拼成恰好通过 BCC 的假帧（帧无序列号、BCC 是弱校验），
          必须丢弃（诊断计数保留）。
        - 积分时间基准（仅里程计开启时）：断流期间速度未知，重连后**首帧
          只建立新基准、不产生位移**，从第二帧起按常规间隔策略积分；位姿
          保留断流前的值。这是被动 dt>max_interval 判定之外的双保险
          （被动判定依赖间隔超过阈值，重连场景由本方法明确触发）。
        """
        self._parser.reset()
        self._reset_drive_handshake()
        if self._guard_wiring is not None:
            # 重连默认锁停：清缓存、回传/传感器证据与解停请求
            self._guard_wiring.on_serial_reconnected()
        if self._odom_candidate is not None:
            self._odom_candidate.reset_time_base()
            self.get_logger().warn(
                '串口已重新打开：已丢弃断流前的残余半帧字节；'
                '里程计积分待首帧重建基准（首帧不产生位移）')
        else:
            self.get_logger().warn(
                '串口已重新打开：已丢弃断流前的残余半帧字节')

    def _read_loop(self) -> None:
        assert self._fd is not None
        # 看门狗基准：以"最近一次收到字节"计时，而不是最近一帧——字节级
        # 静默才是链路死亡（断电/重枚举）的信号，帧缺失可能只是固件问题。
        last_rx = time.monotonic()
        while not self._stop_event.is_set():
            if not select.select([self._fd], [], [], 0.25)[0]:
                if (self._starve_reopen_s > 0
                        and time.monotonic() - last_rx >= self._starve_reopen_s):
                    self.get_logger().error(
                        f'串口静默超过 {self._starve_reopen_s:.1f}s：判定链路'
                        '死亡（固件断电或 USB 重枚举），主动重开串口')
                    return
                continue
            chunk = os.read(self._fd, 1024)
            if not chunk:
                raise OSError('serial disconnected')
            last_rx = time.monotonic()
            self._feed_drive_bridge(chunk)
            for frame in self._parser.feed(chunk):
                self._handle_frame(frame)

    # ------------------------------------------------------------------ 发布
    def _handle_frame(self, frame: StatusFrame) -> None:
        now_monotonic = time.monotonic()
        if (self._last_frame_monotonic is not None
                and now_monotonic - self._last_frame_monotonic > self._frame_gap_warn_s):
            self.get_logger().warn(
                '状态帧间隔超过 '
                f'{self._frame_gap_warn_s:.3f}s（标称 50ms）：可能丢帧或固件未发送；'
                '帧无序列号，无法精确计数')
        self._last_frame_monotonic = now_monotonic

        # 帧内无采样时间戳：消息戳统一用主机接收时刻（节点时钟）。20 Hz 发送 +
        # 串口/调度延迟，抖动不补偿——已声明的限制。里程计积分的时间基准
        # 另用单调时钟（见下），二者同帧取得。
        stamp = self.get_clock().now().to_msg()

        self._notify_drive_status_frame()
        self._publish_imu(frame, stamp)
        self._publish_ultrasonic(frame, stamp)
        self._publish_chassis_diagnostics(frame, stamp)
        if self._odom_candidate is not None:
            self._update_odom(frame, now_monotonic, stamp)
        if self._guard_wiring is not None:
            # 状态帧是本节点唯一的底盘回传证据；超声量测同源（0xFFFF→max_range）
            self._guard_wiring.on_status_frame(frame, now_monotonic)

    def _publish_chassis_diagnostics(self, frame: StatusFrame, stamp) -> None:
        """发布同一校验帧；电压和编码器标度仍需实车确认。"""
        values = {
            'raw_frame_hex': frame.frame_bytes.hex(),
            'parser_frames_accepted_total': self._parser.frames_accepted,
            'parser_candidates_rejected_total': self._parser.candidates_rejected,
            'voltage_mv': frame.voltage_mv,
            'vx_mm_s': frame.chassis_velocity_mm_s[0],
            'vy_mm_s': frame.chassis_velocity_mm_s[1],
            'yaw_rate_mrad_s': frame.chassis_yaw_rate_mrad_s,
            'stop_flag': frame.stop_flag,
        }
        for name, value in zip('abcd', frame.encoder_counts):
            values[f'encoder_{name}_reported'] = value
        for channel, value in enumerate(frame.pwm_counts, start=1):
            values[f'pwm_ch{channel}'] = value
        values['ultrasonic_front_mm'] = frame.ultrasonic_front_mm
        status = DiagnosticStatus()
        status.name = 'chassis'
        status.hardware_id = 'c50c_uart3'
        status.level = DiagnosticStatus.WARN
        status.message = '底盘状态帧已接收；电压和速度待标定'
        status.values = [KeyValue(key=key, value=str(value))
                         for key, value in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = stamp
        msg.status = [status]
        self._chassis_publisher.publish(msg)

    def _notify_drive_status_frame(self) -> None:
        """把一帧已由解析器校验通过的 48 字节状态帧告知命令桥。

        只在基线方言下通知：v3 固件用会话帧（``frame[1]==0x7F``）作为接收
        方向证据，状态帧不能替代它（仲裁器会抛错）。桥未挂载时什么也不做。

        本通知只表示"主机在线上读到了格式合法的一帧基线状态帧"。它**不**
        证明板上固件版本、不证明主机→固件方向已对齐、也不证明固件收到了
        主机的任何命令帧——基线不回复确认。
        """
        if self._drive_bridge is None:
            return
        if self._drive_firmware_protocol != PROTOCOL_BASELINE:
            return
        self._drive_bridge.notify_status_frame()

    def _log_gyro_bias(self) -> None:
        est = self._gyro_bias.estimator
        if not est.converged:
            return
        last = self._gyro_bias_logged
        if last is None or abs(est.bias - last) >= 0.0005:
            self._gyro_bias_logged = est.bias
            self.get_logger().info(
                'IMU z 静止残差修正: %.5f rad/s（%.4f °/s）'
                % (est.bias, est.bias * 57.29578))

    def _publish_imu(self, frame: StatusFrame, stamp) -> None:
        measurements = imu_measurements(frame)
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id

        # 姿态：固件不提供四元数 → 显式置为全零四元数（rclpy 默认是
        # (0,0,0,1) 单位姿态，会向下游暗示有效姿态）并按消息约定标记不可用
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = measurements.gyro_rad_s[0]
        msg.angular_velocity.y = measurements.gyro_rad_s[1]
        msg.angular_velocity.z = measurements.gyro_rad_s[2]
        if self._gyro_bias is not None:
            msg.angular_velocity.z = self._gyro_bias.process(
                frame, measurements, time.monotonic())
            self._log_gyro_bias()
        if measurements.angular_velocity_available:
            msg.angular_velocity_covariance[0] = self._gyro_variance
            msg.angular_velocity_covariance[4] = self._gyro_variance
            msg.angular_velocity_covariance[8] = self._gyro_variance
        else:
            # FlagStop != 0：陀螺 Z 被固件钳零，整组角速度不可信
            msg.angular_velocity_covariance[0] = -1.0

        msg.linear_acceleration.x = measurements.accel_m_s2[0]
        msg.linear_acceleration.y = measurements.accel_m_s2[1]
        msg.linear_acceleration.z = measurements.accel_m_s2[2]
        msg.linear_acceleration_covariance[0] = self._accel_variance
        msg.linear_acceleration_covariance[4] = self._accel_variance
        msg.linear_acceleration_covariance[8] = self._accel_variance

        self._publisher.publish(msg)
        self.get_logger().debug(
            f'stop={frame.stop_flag} vx={frame.chassis_velocity_mm_s[0]}mm/s '
            f'vy={frame.chassis_velocity_mm_s[1]}mm/s '
            f'wz={frame.chassis_yaw_rate_mrad_s}mrad/s '
            f'voltage={frame.voltage_mv}mV')

    def _publish_ultrasonic(self, frame: StatusFrame, stamp) -> None:
        """前置 HC-SR04 距离（50 字节状态帧 [46..47]，mm）。

        固件侧已完成 3 点中值滤波与 2–450 cm 规格域校验；``0xFFFF`` =
        无回波/数据陈旧（目标超出量程或传感器未接），按声呐惯例映射为
        max_range（=路径无障碍），不缓存旧值。

        注意：传感器离地约 5 cm、波束角 ~15°，约 40 cm 外地面回波可能
        混入（读数贴地假阳性）——消费者（导航层）应结合安装高度过滤，
        本节点只透传物理测量。
        """
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = self._ultrasonic_frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26          # 模块规格 ~15°
        msg.min_range = 0.02
        msg.max_range = self._ultrasonic_max_range_m
        if frame.ultrasonic_front_mm == 0xFFFF:
            msg.range = msg.max_range
        else:
            msg.range = frame.ultrasonic_front_mm / 1000.0
        self._ultrasonic_publisher.publish(msg)

    def _update_odom(self, frame: StatusFrame,
                     now_monotonic: float, stamp) -> None:
        """编码器里程计候选：积分 + 影子话题发布（仅 odom_enable 时被调用）。

        - 积分时间基准 = 主机单调接收时刻（time.monotonic），消息戳用节点
          时钟（与 IMU 同帧取得）；首帧只建基准，重复/倒退时间与断流间隙
          （dt>odom_max_interval_s，含串口重连的显式重置）均不积分。
        - 角速度来源是帧 offset 6-7 的**编码器差速解算 wz**（见
          OdomCandidate.update_from_frame），与 IMU 陀螺 Z 无关。
        - 速度标度未标定、协方差占位——消息数值是固件标度原样。
        """
        assert self._odom_candidate is not None
        assert self._odom_publisher is not None
        report = self._odom_candidate.update_from_frame(frame, now_monotonic)
        self._odom_diagnostics = self._odom_candidate.integrator_diagnostics()
        if report.reason == 'gap':
            self.get_logger().warn(
                '里程计积分遇到断流间隙（dt > '
                f'{float(self.get_parameter("odom_max_interval_s").value):.3f}s）：'
                '间隙内速度未知，不跨间隙积分，时间基准已重置')
        elif report.reason == 'nonpositive_dt':
            self.get_logger().warn(
                '里程计积分遇到重复/倒退的接收时刻：本样本拒绝，位姿不变')

        odom_msg = self._odom_candidate.build_msg(stamp)
        self._odom_publisher.publish(odom_msg)
        if self._odom_tf_publisher is not None:
            # 候选 TF：与 Odometry 同帧、同位姿、同 ROS 时间戳——直接取自
            # 已构造的消息，不引入第二套积分器。odom_tf_enable 默认关闭；
            # 这是候选位姿的 TF，不是真实里程计 TF（标度/方向未验收）。
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = odom_msg.header.frame_id
            tf_msg.child_frame_id = odom_msg.child_frame_id
            tf_msg.transform.translation.x = odom_msg.pose.pose.position.x
            tf_msg.transform.translation.y = odom_msg.pose.pose.position.y
            tf_msg.transform.translation.z = odom_msg.pose.pose.position.z
            tf_msg.transform.rotation = odom_msg.pose.pose.orientation
            self._odom_tf_publisher.sendTransform(tf_msg)
        d = self._odom_diagnostics
        self.get_logger().debug(
            f'odom[{report.reason}] x={odom_msg.pose.pose.position.x:.3f} '
            f'y={odom_msg.pose.pose.position.y:.3f} '
            f'accepted={d["accepted_steps"]} gap={d["rejected_gap"]} '
            f'npdt={d["rejected_nonpositive_dt"]} resets={d["explicit_resets"]}')

    # ------------------------------------------------------------ 直行命令桥
    def _attach_drive_bridge(self) -> None:
        """在已打开的 fd 上挂载命令桥（默认关闭）。

        只在 ``drive_enable`` 为真时创建。写入端借用**本节点自己的 fd**，
        不新开设备，因此不会出现第二个串口打开者。创建成功后链路仍停在
        ``RECEIVER_UNKNOWN``：没有可验证的接收端干净证据，一个字节也不写。
        """
        if not self._drive_enabled or self._fd is None:
            return
        from .chassis_cmd.bridge import StraightDriveBridge
        from .chassis_cmd.serial_port import FdPort, RealFdUseForbidden
        try:
            self._drive_port = FdPort(self._fd, self._device,
                                      fd_authorised=self._drive_fd_authorised)
        except RealFdUseForbidden as exc:
            self.get_logger().error(f'直行命令桥拒绝启用：{exc}')
            self._drive_enabled = False
            return
        try:
            self._drive_bridge = StraightDriveBridge(
                self._drive_port,
                device=self._device,
                enabled=self._drive_input_enable,
                tick_period_s=self._drive_tick_period_s,
                firmware_protocol=self._drive_firmware_protocol)
        except ValueError as exc:
            self.get_logger().error(f'直行命令桥参数不合法：{exc}')
            self._drive_enabled = False
            self._drive_bridge = None
            return
        # 会话握手：只有真正从线上读到固件会话帧才放行非零（A 候选协议）。
        # 握手写确认帧用的是同一个 fd（见 _feed_drive_bridge）。
        # 基线方言**没有**会话帧，也不回复任何确认：基线态下不建握手，
        # 接收方向证据改由状态帧提供（见 _notify_drive_status_frame）。
        self._drive_handshake = None
        if self._drive_firmware_protocol == PROTOCOL_BASELINE:
            self.get_logger().warn(
                '直行命令桥使用基线方言：无会话帧、无 ack、无序号；'
                '接收方向证据只来自 48 字节状态帧')
        else:
            from .chassis_cmd.handshake import SessionHandshake
            self._drive_handshake = SessionHandshake(
                self._drive_bridge.chain._arbiter,     # noqa: SLF001——同一仲裁器
                clock=time.monotonic,
                # 收到会话帧后就在端口上签发接收方向收据，桥据此放行
                # _receiver_clean；本节点不碰该收据。
                receipt_sink=self._drive_port)
        # 唯一的驱动路径：节点定时器调 _drive_cycle 驱动桥；输入接线若启用，
        # 它只建订阅、不建定时器（见 _drive_cycle 与 with_timer=False）。
        self._drive_wiring = None
        self._drive_timer = self.create_timer(self._drive_tick_period_s,
                                              self._drive_cycle)
        self.get_logger().warn(
            f'直行命令桥已挂载在既有 fd 上（device={self._device}，'
            f'input_enabled={self._drive_input_enable}，'
            f'protocol={self._drive_firmware_protocol}）；'
            '等待接收方向证据；证据到达前链路静默，不发送任何命令帧')
        if not self._drive_input_enable:
            return
        # 订阅 nav_cmd_vel / demo_cmd_vel / nav_stop。接线只建订阅，不建
        # 自己的定时器（with_timer=False）：本节点唯一的 _drive_timer 负责
        # 每拍复核来源并驱动桥，否则两处同时 tick 会一拍写两帧。
        from .chassis_cmd.ros_input import RosSubscriptionInput
        self._drive_wiring = RosSubscriptionInput(
            self, self._drive_bridge.input,
            tick_period_s=self._drive_tick_period_s)
        self._drive_wiring.start(with_timer=False)
        self.get_logger().warn(
            '直行命令输入已订阅 nav_cmd_vel/demo_cmd_vel/nav_stop；'
            '发布者数量每拍复核，来源不唯一时只发零速')

    def _feed_drive_bridge(self, chunk: bytes) -> None:
        """把读到的字节交给会话握手；握手用同一 fd 回填确认帧。"""
        if self._drive_handshake is None or self._drive_port is None:
            return
        self._drive_handshake.feed(
            chunk, write=self._write_session_ack)

    def _write_session_ack(self, frame: bytes) -> int:
        """用所有者自己的 fd 写会话确认帧（不新增打开者、不新增锁）。"""
        if self._drive_port is None:
            return 0
        return self._drive_port.write_bytes(frame)

    def _drive_cycle(self) -> None:
        """唯一驱动路径：复核来源 → 会话门 → 仲裁 → 写入（每拍一次）。

        无论输入接线是否启用都只走这一条；每拍最多写一帧：
        guard 区已挂载且可写时本拍由 guard 写入；否则走既有直行命令桥。
        两者共用同一 fd，不会同拍写两帧。
        """
        if self._drive_bridge is None:
            return
        guard_record = self._guard_cycle(
            record_only=self._drive_wiring is not None)
        if guard_record is not None:
            self._drive_last = (guard_record.frame_kind == 'speed', guard_record)
            return
        self._run_straight_bridge()

    def _run_straight_bridge(self) -> None:
        """既有直行命令桥一拍（guard 未挂载/不可写时的唯一写入路径）。"""
        if self._drive_wiring is not None:
            self._drive_wiring.refresh_publishers()
        gated, record = self._drive_bridge.cycle()
        self._drive_last = (gated, record)
        # 每 5 秒一行链路诊断：只看现象（PWM 恒 0）无法区分是哪一道门卡住，
        # 这一行把门控结果与零速原因直接打出来。
        self._drive_probe_ticks = getattr(self, '_drive_probe_ticks', 0) + 1
        if self._drive_probe_ticks % max(1, int(5.0 / max(
                self._drive_tick_period_s, 1e-3))) == 0:
            reason = getattr(getattr(record, 'zero_reason', None), 'name', None)
            snap = (self._drive_wiring.snapshot()
                    if self._drive_wiring is not None else {})
            counts = self._drive_bridge.chain.counts
            adapter = snap.get('adapter', {})
            # T8 发布者告警（纯观测）：任一命令通道发布者数 >1 时升级为
            # warning。只读 wiring snapshot 的 *_publishers 字段，不触碰
            # 适配层私有属性；_set_publishers 的作废/放行语义零改动——
            # 多来源场景由既有 per-channel revoke 自行处理，这里只负责
            # 让现场能直接看到"谁在多播"。
            _chan_topics = {
                'nav': '/nav_cmd_vel',
                'demo': '/demo_cmd_vel',
                'manual': '/manual_cmd_vel',
                'auto': '/auto_cmd_vel',
            }
            for _chan, _limit in (('nav', 1), ('demo', 1),
                                  ('manual', 1), ('auto', 1)):
                _count = adapter.get(_chan + '_publishers')
                if not isinstance(_count, int) or _count <= _limit:
                    continue
                self.get_logger().warning(
                    '%s 通道发布者数=%d（应=1）：多来源互相覆盖并触发'
                    '作废，非零命令会被扣；排查：ros2 topic info %s'
                    % (_chan, _count, _chan_topics[_chan]))
            self.get_logger().info(
                'drive probe: gated=%s record=%s zero_reason=%s '
                'sent_nonzero=%s sent_zero=%s dropped=%s '
                'allowed=%s gate=%s enabled=%s pubs_known=%s nav_pubs=%s '
                'nav_msgs=%s'
                % (gated, record is not None, reason,
                   counts.get('sent_nonzero'), counts.get('sent_zero'),
                   {k: v for k, v in counts.items()
                    if k.startswith('dropped_') and v},
                   adapter.get('outputs_allowed'), adapter.get('gate_open'),
                   adapter.get('enabled'), adapter.get('publishers_known'),
                   adapter.get('nav_publishers'),
                   self._drive_bridge.nav_msgs))
            nav = self._drive_bridge.chain._arbiter._nav   # noqa: SLF001
            arb = self._drive_bridge.chain._arbiter
            navc = arb._nav   # noqa: SLF001
            self.get_logger().info(
                'drive probe reject: %s | gate: %s | nav=(%s) stop=%s sends=%s'
                ' safety=%s'
                % (getattr(arb, '_last_reject', ''), getattr(arb, '_last_gate', ''),
                   'None' if navc is None else '%.3f,%.3f,%.3f' % (navc.vx, navc.vy, navc.vz),
                   arb._stop_active, self._drive_bridge.nav_msgs,
                   arb.safety_state(self._drive_bridge.clock())))
            cache_desc = ('None' if nav is None else
                          '(%.4f, %.4f, %.4f)' % (nav.vx, nav.vy, nav.vz))
            self.get_logger().info(
                'drive probe nav cache: %s msg_vx=%.3f msg_vz=%.3f allow_at_msg=%s allow_now=%s ev=%s lic=%s'
                % (cache_desc, self._drive_bridge.last_msg_vx,
                   self._drive_bridge.last_msg_vz,
                   self._drive_bridge.last_allow,
                   self._drive_bridge._allow_nonzero(),        # noqa: SLF001
                   self._drive_bridge.chain._arbiter.baseline_window_open(  # noqa: SLF001
                       self._drive_bridge.clock()),
                   self._drive_bridge.input._nav_licensed)     # noqa: SLF001
                + ' dis=%d mal=%d recv_nav=%d'
                % (self._drive_bridge.input.state.dropped_while_disabled,
                   self._drive_bridge.input.state.rejected_malformed,
                   self._drive_bridge.input.state.received_nav))
            if self._guard_wiring is not None and self._guard_wiring.bridge is not None:
                bridge = self._guard_wiring.bridge
                guard = bridge.guard.snapshot()
                license_state = guard['license']
                scan_stamp = bridge._scan[1]
                scan_age = None if scan_stamp is None else bridge.clock() - scan_stamp
                self.get_logger().info(
                    'guard probe: latched=%s cause=%s pending=%s rearms=%s '
                    'rearm_refused=%s scan_received=%s scan_age=%s '
                    'license_accepted=%s license_rejected=%s last_reject=%s events=%s'
                    % (guard['latched'], guard['stop_cause'], guard['rearm_pending'],
                       guard['rearms'], guard['rearm_refused'], bridge._scan[0] is not None,
                       scan_age, license_state['accepted'], license_state['rejected'],
                       license_state['last_reject'], guard['events'][-3:]))

    def _guard_cycle(self, *, record_only):
        """guard 区每拍。返回 ``WriteRecord`` 表示本拍已由 guard 决定写入；
        ``None`` 表示未挂载/不可写，由调用方走原有路径（或什么都不写）。

        - 第一个定时器拍（或每次重连后的第一拍）懒建 guard；
        - 直行桥的导航输入接线（``drive_input_enable``）同时启用时，两条写入
          路径会在同一 fd 上互相覆盖：guard **只决策不写**（``record_only``），
          不改变既有直行桥行为；
        - 写入与上一拍至少间隔 ``drive_tick_period_s``（``write_gap_s``），
          给读循环/状态帧留出串口余量；被跳过的拍只记录决策。
        """
        if not self._drive_enabled or self._drive_port is None:
            return None
        if self._guard_wiring is None:
            from .chassis_cmd.arbiter import PROTOCOL_BASELINE
            from .chassis_cmd.guard_wiring import GuardWiring
            # [B3 修复 2026-11-18] drive_input_enable=true（默认导航部署）时
            # 直行命令桥与本 guard 区共用同一个 fd：guard 仍要挂载做决策，
            # 但**永不成为第二个写者**——coexist=True 让桥每拍只记录
            # （frame_kind='none'），节点侧本拍走 record_only 分支，并把
            # guard 的锁停以边沿注入直行桥仲裁器。挂载门（真实端口/fd
            # 授权/基线方言）不变。
            coexist = bool(self._drive_input_enable)
            self._guard_wiring = GuardWiring(
                self, port=self._drive_port, device=self._device,
                firmware_protocol=self._drive_firmware_protocol,
                baseline_protocol=PROTOCOL_BASELINE,
                ultrasonic_max_range_m=self._ultrasonic_max_range_m,
                drive_enable=self._drive_enabled,
                fd_authorised=self._drive_fd_authorised,
                has_other_writer=False,
                coexist=coexist,
                write_gap_s=self._drive_tick_period_s)
        if self._guard_wiring.ensure() is None:
            return None
        if record_only:
            # 共存模式：guard 只决策；其停车状态注入直行桥仲裁器。
            # guard 解除锁停不会解除外部监督停车。
            self._guard_wiring.guard_tick_record_only()
            self._guard_wiring.forward_stop_to_arbiter(
                getattr(self._drive_bridge, 'chain', None))
            return None
        if not self._guard_wiring.can_write()[0]:
            return None
        return self._guard_wiring.tick()

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._thread.join(timeout=3.0)
        if self._thread.is_alive():
            # 不能静默当作正常关闭：线程仍持有串口 fd，且其后续 publish
            # 可能与正在进行的 context 关闭竞争。
            self.get_logger().error(
                '读取线程 3 s 内未退出，节点将带着未停止的线程继续关闭；'
                f'串口 {self._device} 可能不会释放，请检查残留进程')
        return super().destroy_node()


def main(args=None) -> None:
    # 禁用 rclpy 默认 SIGINT 处理器：它会在 spin 抛出异常**之前**就关闭
    # context，导致读线程在 context 失效后仍尝试 publish（现场复现过
    # RCLError: publisher's context is invalid）。改为：SIGINT 在主线程
    # 转成 KeyboardInterrupt，context 由本函数在读线程停止**之后**再显式
    # 关闭（destroy_node 内先 join 线程）。
    rclpy.init(args=args,
               signal_handler_options=SignalHandlerOptions.NO)
    node = C50CUart3Node()
    try:
        # 不用无限阻塞的 spin：SignalHandlerOptions.NO 下没有信号线程唤醒
        # rcl_wait，rmw 的 EINTR 重试会吞掉 SIGINT，KeyboardInterrupt 可能
        # 永远不触发（定向测试复现过）。0.1 s 轮询保证主线程周期性回到
        # Python 层处理信号。
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
