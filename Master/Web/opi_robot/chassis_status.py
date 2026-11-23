"""底盘只读状态：车速、里程、电压，以及"哪儿有动的东西"。

为什么要这个模块
----------------
``c50c_uart3_node`` 目前只把串口状态帧转成 ``/imu/data_raw`` 发布，
电压、底盘速度、编码器计数都不进任何 ROS 话题。网页于是永远显示
"无数据"。本模块**只读**地补上这一路，不改 ``c50c_uart3`` 包、不碰
串口写入、不发布任何运动命令。

它做两件事：

1. **底盘诊断**：把状态帧解成车速/电压，软件积分里程，发布
   ``/diagnostics``（name = ``chassis_status``）。网页的底盘卡和电池卡
   都读这一帧。
2. **移动目标标记**：按角度分箱比较连续两圈扫描，把"这一圈还在、上一圈
   不在附近"的角度标成移动目标，发布 ``std_msgs/UInt8MultiArray``
   （``/robot/moving_bins``）。网页只用它给点云上色，其余点冻结不动——
   静止的墙和柱子不再每帧抖动，动的东西仍然会亮起来。

数据来源之二选一
----------------
- ``bytes_topic``（默认 ``chassis/status_bytes``）: 另一进程把底盘字节
  转发的只读副本。
- ``device``: 直接 O_RDONLY 打开串口。**仅当 c50c_uart3_node 已停用**
  时可用，两个进程同读会互相抢字节。

为什么车上目前不接
------------------
底盘桥 ``c50c_uart3_node`` 以 ``TIOCEXCL`` 独占串口，第二个进程再 open
会得到 EBUSY，``tee -p`` 复制读取缓冲同样被拒。所以车上**不设数据来源**，
网页底盘卡/电池卡显示"无数据"；要接线需先在底盘桥侧加一条只读诊断话题。

状态帧字段：``c50c_uart3/c50c_uart3/frame_parser.py``（``StatusFrame``）。
"""
from __future__ import annotations

import os
import time

STATUS_NAME = 'chassis_status'
STATUS_TOPIC = 'chassis/status_bytes'
MOVING_TOPIC = 'robot/moving_bins'
SCAN_TOPIC = 'scan'
BINS = 120

# 里程落盘位置与周期。同目录下由 web_node.console_views 读写，格式一致。
MILEAGE_FILE = os.environ.get(
    'OPI_MILEAGE_FILE', os.path.expanduser('~/.cache/opi-robot/trip'))
MILEAGE_SAVE_S = 5.0

# 单次积分步长上限（秒）。超过就视为断流，这一段不积分，
# 避免掉线几分钟后靠一次大 dt 凭空"跑"出几公里。
MAX_STEP_S = 0.5


# --------------------------------------------------------------------------
# 里程
# --------------------------------------------------------------------------
class MileageIntegrator:
    """用底盘 vx 积分出里程（m）。

    里程是**软件积分**，不是轮式里程计：只对车速求时间积分。停车不增长，
    丢帧期间（dt 超过 ``max_step_s``）不计入。
    """

    def __init__(self, max_step_s=MAX_STEP_S):
        self._max_step = float(max_step_s)
        self.reset()

    def reset(self):
        self._last_t = None
        self._last_vx = None
        self.distance_m = 0.0
        self.speed_m_s = 0.0

    def update(self, vx_mm_s, now):
        """喂一帧车速（mm/s）。now 与上次的差值用作积分步长。"""
        vx = float(vx_mm_s) / 1000.0
        if self._last_t is not None:
            dt = now - self._last_t
            if 0.0 <= dt <= self._max_step:
                prev = self._last_vx if self._last_vx is not None else vx
                self.distance_m += abs(0.5 * (prev + vx)) * dt
        self._last_t = now
        self._last_vx = vx
        self.speed_m_s = vx

    def trip(self):
        return self.distance_m


class TripStore:
    """把里程积分器落到文件，让桥/网页重启后接着累计。

    换进程后单调时钟零点变了，所以文件里存的是墙上时钟：重启时用两次
    墙钟差乘上最后一次车速，补上停机这段的时间。
    """

    def __init__(self, path=None, wall_now=None, mono_now=None):
        self.path = path or MILEAGE_FILE
        self.wall_now = wall_now or time.time
        self.mono_now = mono_now or time.monotonic

    def load(self, integrator):
        """读盘并接上；没有存档就保持 0。返回读到的里程（m）或 None。"""
        try:
            with open(self.path) as fh:
                trip_m, saved_wall = (float(x) for x in fh.read().split()[:2])
        except (OSError, ValueError):
            return None
        elapsed = max(0.0, self.wall_now() - saved_wall)
        integrator.distance_m = trip_m + abs(integrator.speed_m_s) * elapsed
        return trip_m

    def save(self, integrator):
        """原子写盘；失败只抛 OSError，由调用方决定是否忽略。"""
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as fh:
            fh.write('%.3f %.3f\n' % (integrator.trip(), self.wall_now()))
        os.replace(tmp, self.path)


# --------------------------------------------------------------------------
# 移动目标检测
# --------------------------------------------------------------------------
class MovingDetector:
    """按角度分箱找"上一圈这里还没有、这一圈出现了"的角区。

    只在**车位基本静止**时判定。车在动的时候整张图都在变，把所有点都
    标成"移动"没有意义，而且会让静止点跟着抖。
    """

    def __init__(self, bins=BINS, threshold_m=0.35):
        self.bins = int(bins)
        self.threshold_m = float(threshold_m)
        self._prev = None

    def reset(self):
        self._prev = None

    def feed(self, ranges, rmin, rmax):
        """ranges 为该圈按角度递增的距离数组，返回移动角区索引列表。"""
        cur = []
        for r in ranges:
            r = float(r)
            ok = (r == r) and rmin <= r <= rmax
            cur.append(r if ok else None)
        if self._prev is None or len(cur) != len(self._prev) or not cur:
            # 首圈、长度变化或整圈无有效点：只建基准，不报移动。
            # 首圈全是 nan 时也算"还没见过"，否则第二圈所有点都会因为
            # old is None 被误报成移动。
            if all(r is None for r in cur):
                return []
            self._prev = cur
            return []
        step = len(cur) / float(self.bins)
        moving = []
        for b in range(self.bins):
            i0 = int(b * step)
            i1 = max(i0 + 1, int((b + 1) * step))
            changed = 0
            seen = 0
            for i in range(i0, min(i1, len(cur))):
                r = cur[i]
                if r is None:
                    continue
                seen += 1
                old = self._prev[i]
                if old is None or abs(r - old) > self.threshold_m:
                    changed += 1
            if seen >= max(1, (i1 - i0) // 2) and changed * 2 >= seen:
                moving.append(b)
        self._prev = cur
        return moving


# --------------------------------------------------------------------------
# 诊断载荷
# --------------------------------------------------------------------------
def _fmt(value, digits=0):
    return ('%.' + str(digits) + 'f') % value


def build_diagnostics(frame, mileage_m, speed_m_s, age_s=None,
                      frames=0, rejects=0, hz=None, stale_reason=None):
    """把一帧状态和里程打成 ``(level, message, values)``。

    values 的键名与 ``chassis.py`` 读取的候选名一致（``voltage_mv`` /
    ``vx_mm_s`` …）。frame 为 None 时只发状态项，不编造数值。
    """
    if frame is None:
        state = 'stale' if stale_reason else 'unknown'
        values = [
            ('state', state),
            ('reason', stale_reason or '从未收到状态帧'),
            ('frames', str(frames)),
            ('rejects', str(rejects)),
            ('mileage_m', _fmt(mileage_m, 2)),
        ]
        level = 3 if stale_reason else 2
        message = stale_reason or '底盘状态帧未到达'
        return level, message, values

    values = [
        ('state', 'ok'),
        ('voltage_mv', str(int(frame.voltage_mv))),
        ('vx_mm_s', str(int(frame.chassis_velocity_mm_s[0]))),
        ('vy_mm_s', str(int(frame.chassis_velocity_mm_s[1]))),
        ('yaw_rate_mrad_s', str(int(frame.chassis_yaw_rate_mrad_s))),
        ('stop_flag', str(int(frame.stop_flag))),
        ('speed_m_s', _fmt(speed_m_s, 3)),
        ('mileage_m', _fmt(mileage_m, 2)),
        ('frames', str(frames)),
        ('rejects', str(rejects)),
    ]
    if age_s is not None:
        values.append(('age_s', _fmt(age_s, 2)))
    if hz is not None:
        values.append(('hz', _fmt(hz, 1)))
    level = 0 if int(frame.stop_flag) == 0 else 1
    message = '车速 %.2f m/s，里程 %.1f m' % (speed_m_s, mileage_m)
    if level:
        message = '软件失能；' + message
    return level, message, values


# --------------------------------------------------------------------------
# ROS 包装
# --------------------------------------------------------------------------
def _ros_imports():
    from rclpy.node import Node
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from std_msgs.msg import UInt8MultiArray
    from sensor_msgs.msg import LaserScan
    # 逐级退让：安装后的入口脚本以 opi_robot.chassis_status 导入时用相对
    # 导入；脚本直接跑（top-level 模式）时包没有父级，改用绝对导入。
    try:
        from ..diagnostic_level import encode_level
        from .. import qos as Q
    except ImportError:
        from opi_robot.diagnostic_level import encode_level
        from opi_robot import qos as Q
    from c50c_uart3.frame_parser import StatusFrameParser
    return (Node, DiagnosticArray, DiagnosticStatus, KeyValue,
            UInt8MultiArray, LaserScan, encode_level, Q, StatusFrameParser)


class SerialByteReader:
    """以 O_RDONLY 打开串口，只读、不写，供直连模式使用。

    用 ``os.open`` 而不是 pyserial：不引入额外依赖，也保证本节点拿不到
    写句柄。打开失败按间隔重试并在日志里说明原因。
    """

    def __init__(self, device, logger, retry_s=1.0):
        self.device = device
        self._log = logger
        self._retry_s = retry_s
        self._fd = None
        self._next_try = 0.0

    def read_available(self):
        import os
        import select
        import tty
        now = time.monotonic()
        if self._fd is None:
            if now < self._next_try:
                return b''
            self._next_try = now + self._retry_s
            try:
                self._fd = os.open(self.device, os.O_RDONLY | os.O_NOCTTY)
                tty.setraw(self._fd)
                self._log.info('串口只读打开成功: %s' % self.device)
            except OSError as exc:
                self._fd = None
                self._log.warn('串口只读打开失败 %s: %s' % (self.device, exc))
                return b''
        try:
            ready, _, _ = select.select([self._fd], [], [], 0)
            if not ready:
                return b''
            return os.read(self._fd, 4096)
        except OSError as exc:
            self._log.warn('串口读失败，稍后重开 %s: %s' % (self.device, exc))
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
            return b''


def build_ros_node_class():
    """拼出真实 ROS 节点类。只在 import rclpy 成功时调用。"""
    (Node, DiagnosticArray, DiagnosticStatus, KeyValue,
     UInt8MultiArray, LaserScan, encode_level, Q, StatusFrameParser) = \
        _ros_imports()

    class RosChassisStatusNode(Node):
        """订阅底盘字节流与雷达扫描，发布诊断与移动角区。"""

        def __init__(self, context=None):
            super().__init__('chassis_status_node', context=context)
            self.declare_parameter('bytes_topic', STATUS_TOPIC)
            self.declare_parameter('scan_topic', SCAN_TOPIC)
            self.declare_parameter('diag_period', 0.5)
            self.declare_parameter('stale_s', 1.0)
            self.declare_parameter('moving_threshold_m', 0.35)
            self.declare_parameter('moving_min_speed_m_s', 0.05)
            self.declare_parameter('device', '')
            topic = str(self.get_parameter('bytes_topic').value or '')
            scan_topic = str(self.get_parameter('scan_topic').value or '')
            self._stale_s = float(self.get_parameter('stale_s').value)
            self._min_speed = float(
                self.get_parameter('moving_min_speed_m_s').value)
            self._parser = StatusFrameParser()
            self._mileage = MileageIntegrator()
            self._detector = MovingDetector(
                threshold_m=self.get_parameter('moving_threshold_m').value)
            self._last_frame = None
            self._last_t = None
            self._last_publish_t = None
            self._hz_base = 0
            self._hz = None
            self._frames = 0
            self._moving_pub = None

            self._diag = self.create_publisher(DiagnosticArray, 'diagnostics',
                                               Q.status_qos())
            dev = str(self.get_parameter('device').value or '')
            self._reader = None
            if dev:
                self._reader = SerialByteReader(dev, self.get_logger())
                self.create_timer(0.02, self._poll_device)
            elif topic:
                self.create_subscription(UInt8MultiArray, topic,
                                         self.on_bytes, Q.status_qos())
            else:
                self.get_logger().error(
                    'bytes_topic 与 device 都为空：收不到任何底盘数据')
            if scan_topic:
                self._moving_pub = self.create_publisher(
                    UInt8MultiArray, MOVING_TOPIC, Q.status_qos())
                self.create_subscription(LaserScan, scan_topic, self.on_scan,
                                         Q.sensor_qos())
            self.create_timer(
                max(0.05, float(self.get_parameter('diag_period').value)),
                self.publish_diag)
            self.get_logger().info(
                'chassis_status_node 启动: bytes=%s device=%s scan=%s'
                % (topic or '无', dev or '未指定', scan_topic or '无'))

        # ---- 底盘字节 ----
        def _poll_device(self):
            if self._reader is None:
                return
            data = self._reader.read_available()
            if data:
                self._ingest(data)

        def on_bytes(self, msg):
            self._ingest(bytes(msg.data))

        def _ingest(self, data):
            if not data:
                return
            frames = self._parser.feed(data)
            if not frames:
                return
            now = time.monotonic()
            for frame in frames:
                self._last_frame = frame
                self._last_t = now
                self._mileage.update(frame.chassis_velocity_mm_s[0], now)
            self._frames += len(frames)
            if self._last_publish_t is not None:
                window = max(0.2, now - self._last_publish_t)
                self._hz = (self._parser.frames_accepted - self._hz_base) / window

        def publish_diag(self):
            now = time.monotonic()
            self._last_publish_t = now
            self._hz_base = self._parser.frames_accepted
            age = None if self._last_t is None else now - self._last_t
            fresh = age is not None and age <= self._stale_s
            reason = None
            if age is not None and not fresh:
                reason = '底盘状态帧已过期 %.1fs' % age
            elif age is None:
                reason = '从未收到状态帧'
            level, message, values = build_diagnostics(
                self._last_frame if fresh else None,
                self._mileage.trip(), self._mileage.speed_m_s,
                age_s=age if fresh else None,
                frames=self._frames,
                rejects=self._parser.candidates_rejected,
                hz=self._hz, stale_reason=reason)
            status = DiagnosticStatus()
            status.level = encode_level(level)
            status.name = STATUS_NAME
            status.message = message
            status.hardware_id = 'c50c'
            status.values = [KeyValue(key=k, value=str(v))
                             for k, v in values]
            self._diag.publish(DiagnosticArray(status=[status]))

        # ---- 雷达：只标移动角区 ----
        def on_scan(self, msg):
            if self._moving_pub is None:
                return
            # 车在动时整图都在变，不做移动标记，避免静止点跟着抖
            if abs(self._mileage.speed_m_s) > self._min_speed:
                self._detector.reset()
                moving = []
            else:
                moving = self._detector.feed(msg.ranges, msg.range_min,
                                             msg.range_max)
            out = UInt8MultiArray()
            out.data = list(moving)
            self._moving_pub.publish(out)

    return RosChassisStatusNode


def main(argv=None):
    """入口：起一个只读节点，里程每 5 秒落盘一次。"""
    import rclpy
    rclpy.init(args=argv)
    node = build_ros_node_class()()
    store = TripStore()
    try:
        saved = store.load(node._mileage)
        if saved is not None:
            node.get_logger().info('里程续接: 存档 %.2f m，当前 %.2f m'
                                   % (saved, node._mileage.trip()))

        def _save():
            try:
                store.save(node._mileage)
            except OSError as exc:
                node.get_logger().warn('里程写盘失败: %s' % exc)
        saver = node.create_timer(MILEAGE_SAVE_S, _save)
        rclpy.spin(node)
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        try:
            store.save(node._mileage)
        except OSError:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
