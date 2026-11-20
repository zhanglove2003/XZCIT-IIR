"""测试用的最小 ROS 替身。

本机没有安装 rclpy。用本模块提供 diagnostic_msgs、sensor_msgs、
geometry_msgs、std_msgs 的最小对象，并替换 Node 基类。

生产代码的 ROS 调用不变，只替换消息工厂与 Node 基类。
"""

import types


class _Msg:
    """可带默认字段的消息基类。

    BYTE_FIELDS 列出在真实 ROS 2 里声明为 `byte` 的字段。这些字段
    只接受长度为 1 的 bytes，赋值其它类型时抛出与真实 ROS 相同
    形式的 AssertionError。伪 ROS 若不做这项检查，就会把
    "level 用整数赋值" 这类真实环境必崩的写法当成通过。
    """

    FIELDS = ()
    DEFAULTS = {}
    BYTE_FIELDS = ()

    def __setattr__(self, name, value):
        if name in self.BYTE_FIELDS:
            if not isinstance(value, (bytes, bytearray)) or len(value) != 1:
                raise AssertionError(
                    "The '%s' field must be of type 'bytes' or "
                    "'ByteString' with length 1" % name)
            value = bytes(value)
        object.__setattr__(self, name, value)

    def __init__(self, **kwargs):
        for name in self.FIELDS:
            # byte 字段没有安全默认值，一律等 kwargs 给；不给就保持未设置
            if name in self.BYTE_FIELDS:
                continue
            default = self.DEFAULTS.get(name)
            setattr(self, name, default() if callable(default) else default)
        for key, value in kwargs.items():
            setattr(self, key, value)

    def __repr__(self):
        parts = ['%s=%r' % (k, getattr(self, k, None)) for k in self.FIELDS]
        return '%s(%s)' % (type(self).__name__, ', '.join(parts))


class Header(_Msg):
    FIELDS = ('stamp', 'frame_id')


class Time(_Msg):
    FIELDS = ('sec', 'nanosec')


def _header_with_stamp():
    # 真实 sensor_msgs/LaserScan 的 header.stamp 是 stamp: builtin_interfaces/Time
    # （全零表示"未填"），而不是 None。[2026-11-17 复核] 默认值保持全零
    # （与真实 rclpy ``Time()`` 默认一致），**不再**偷偷填节点时钟：
    # 那会把"时间戳不可用 → 方向门 stale"的真实路径从所有测试里抽掉。
    # 需要新鲜帧的节点级测试显式写明时间戳（见 test_motion_gate._stamp）。
    return Header(stamp=Time(sec=0, nanosec=0))


class LaserScan(_Msg):
    FIELDS = ('header', 'angle_min', 'angle_max', 'angle_increment',
              'time_increment', 'scan_time', 'range_min', 'range_max',
              'ranges', 'intensities')
    # 与真实 sensor_msgs/LaserScan 一致：header.stamp 是 stamp: Time。
    DEFAULTS = {'header': _header_with_stamp,
                'ranges': list, 'intensities': list}


class CompressedImage(_Msg):
    FIELDS = ('header', 'format', 'data')
    DEFAULTS = {'header': Header, 'data': bytes}


class _Vec(_Msg):
    FIELDS = ('x', 'y', 'z')
    DEFAULTS = {'x': 0.0, 'y': 0.0, 'z': 0.0}


class Twist(_Msg):
    FIELDS = ('linear', 'angular')
    DEFAULTS = {'linear': _Vec, 'angular': _Vec}


class KeyValue(_Msg):
    FIELDS = ('key', 'value')


class DiagnosticStatus(_Msg):
    FIELDS = ('level', 'name', 'message', 'hardware_id', 'values')
    DEFAULTS = {'values': list}
    # 真实 ROS 2 Humble 实测：level 是单字节 byte 字段
    BYTE_FIELDS = ('level',)

    @classmethod
    def make(cls, level=0, **kwargs):
        """按真实字段类型构造：level 自动编码成单字节 bytes。

        测试里写 `DiagnosticStatus.make(level=2)` 与真实环境赋值等价，
        而不是把整数直接塞进 byte 字段。
        """
        if isinstance(level, (bytes, bytearray)):
            raw = bytes(level)
        else:
            raw = bytes([int(level)])
        return cls(level=raw, **kwargs)
    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3


class DiagnosticArray(_Msg):
    FIELDS = ('header', 'status')
    DEFAULTS = {'header': Header, 'status': list}


class String(_Msg):
    FIELDS = ('data',)


class Bool(_Msg):
    """std_msgs/Bool。巡航开关用它；默认 False，与真实 ROS 一致。"""

    FIELDS = ('data',)
    DEFAULTS = {'data': False}


class Float32MultiArray(_Msg):
    FIELDS = ('layout', 'data')
    DEFAULTS = {'data': list}


class MultiArrayLayout(_Msg):
    FIELDS = ('dim', 'data_offset')
    DEFAULTS = {'dim': list}


class MultiArrayDimension(_Msg):
    FIELDS = ('label', 'size', 'stride')


class Point(_Msg):
    FIELDS = ('x', 'y', 'z')
    DEFAULTS = {'x': 0.0, 'y': 0.0, 'z': 0.0}


class Quaternion(_Msg):
    FIELDS = ('x', 'y', 'z', 'w')
    DEFAULTS = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}


class Pose(_Msg):
    FIELDS = ('position', 'orientation')
    DEFAULTS = {'position': Point, 'orientation': Quaternion}


class Imu(_Msg):
    """sensor_msgs/Imu。控制台只用它做心跳，不读数值。"""

    FIELDS = ('header', 'orientation', 'orientation_covariance',
              'angular_velocity', 'angular_velocity_covariance',
              'linear_acceleration', 'linear_acceleration_covariance')
    DEFAULTS = {'header': Header, 'orientation': Quaternion,
                'angular_velocity': _Vec, 'linear_acceleration': _Vec,
                'orientation_covariance': lambda: [0.0] * 9,
                'angular_velocity_covariance': lambda: [0.0] * 9,
                'linear_acceleration_covariance': lambda: [0.0] * 9}


class Range(_Msg):
    """sensor_msgs/Range。前置 HC-SR04（ultrasonic_front）用。

    常量与真实生成码一致挂在类上：ULTRASOUND=0 / INFRARED=1，
    发布端写 ``msg.radiation_type = Range.ULTRASOUND``。
    """

    ULTRASOUND = 0
    INFRARED = 1

    FIELDS = ('header', 'radiation_type', 'field_of_view',
              'min_range', 'max_range', 'range')
    DEFAULTS = {'header': Header, 'radiation_type': 0,
                'field_of_view': 0.0, 'min_range': 0.0, 'max_range': 0.0,
                'range': 0.0}


class MapMetaData(_Msg):
    """nav_msgs/MapMetaData：占据栅格的尺寸与原点。"""

    FIELDS = ('resolution', 'width', 'height', 'origin')
    DEFAULTS = {'origin': Pose}


class OccupancyGrid(_Msg):
    """nav_msgs/OccupancyGrid。SLAM 地图（web_debug 地图页）用。

    data 真实类型是 int8[]（-1=未知, 0..100），桩里用 list 按元素存。
    """

    FIELDS = ('header', 'info', 'data')
    DEFAULTS = {'header': Header, 'info': MapMetaData, 'data': list}


class _ActionFuture:
    """rclpy ActionClient 的 future 替身：同步已完成的 future。

    刻意**不继承 _Msg**：future 字段里有函数/异常对象，继承消息基类会
    触发 __setattr__ 的字节字段断言，还会与真实 rclpy 的 future 形态不符。
    """

    def __init__(self, result=None, error=None):
        self._result = result
        self._error = error

    def add_done_callback(self, fn):
        fn(self)

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result


class PoseStamped(_Msg):
    FIELDS = ('header', 'pose')
    DEFAULTS = {'header': Header, 'pose': Pose}


class Detect(_Msg):
    FIELDS = ('class_id', 'score', 'x', 'y', 'width', 'height')


class Detection2D(_Msg):
    FIELDS = ('header', 'results', 'id', 'bbox')
    DEFAULTS = {'header': Header, 'results': list}


class Detection2DArray(_Msg):
    FIELDS = ('header', 'detections')
    DEFAULTS = {'header': Header, 'detections': list}


class ObjectHypothesisWithPose(_Msg):
    FIELDS = ('hypothesis', 'pose')
    DEFAULTS = {'pose': Pose}


class ObjectHypothesis(_Msg):
    FIELDS = ('class_id', 'score')


# ---- 模块对象 ----
diagnostic_msgs_msg = types.ModuleType('diagnostic_msgs.msg')
diagnostic_msgs_msg.DiagnosticArray = DiagnosticArray
diagnostic_msgs_msg.DiagnosticStatus = DiagnosticStatus
diagnostic_msgs_msg.KeyValue = KeyValue

sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
sensor_msgs_msg.LaserScan = LaserScan
sensor_msgs_msg.CompressedImage = CompressedImage
sensor_msgs_msg.Imu = Imu
sensor_msgs_msg.Range = Range

nav_msgs_msg = types.ModuleType('nav_msgs.msg')
nav_msgs_msg.OccupancyGrid = OccupancyGrid
nav_msgs_msg.MapMetaData = MapMetaData

geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
geometry_msgs_msg.Twist = Twist
# 点图导航/巡航发目标用 geometry_msgs/PoseStamped
geometry_msgs_msg.PoseStamped = PoseStamped
geometry_msgs_msg.Pose = Pose
geometry_msgs_msg.Quaternion = Quaternion
geometry_msgs_msg.PointStamped = type('PointStamped', (_Msg,), {
    'FIELDS': ('header', 'point'),
    'DEFAULTS': {'header': Header, 'point': Point}})

std_msgs_msg = types.ModuleType('std_msgs.msg')
std_msgs_msg.String = String
std_msgs_msg.Float32MultiArray = Float32MultiArray
# 巡航开关用 std_msgs/Bool；缺它时 web_debug 必须降级为"不可用"
std_msgs_msg.Bool = Bool

# [巡航] nav2_msgs/action/NavigateToPose 与 rclpy.action.ActionClient 的最小替身。
# 巡航（patrol）经 nav_goal 一次一个目标；离线用例要让"接受/结果/取消失败"
# 可编排，所以 ActionClient 把 send_goal_async 的返回交给测试给定的策略。
class NavigateToPoseGoal(_Msg):
    FIELDS = ('pose',)
    DEFAULTS = {'pose': PoseStamped}


class NavigateToPose:
    Goal = NavigateToPoseGoal


nav2_msgs_msg = types.ModuleType('nav2_msgs.msg')
nav2_msgs_msg.PoseStamped = PoseStamped
nav2_msgs_action = types.ModuleType('nav2_msgs.action')
nav2_msgs_action.NavigateToPose = NavigateToPose


class FakeGoalHandle:
    """被接受的目标句柄：记录取消次数；结果由测试用 ``finish()`` 触发。"""

    def __init__(self, client, status=5):
        self.client = client
        self.accepted = True
        self.cancelled = 0
        self.done = False
        self._result_status = status
        self._result_future = None

    def cancel_goal_async(self):
        self.cancelled += 1
        if _TRACE:
            import traceback
            print('CANCEL#%d' % self.cancelled, ''.join(traceback.format_stack()[-6:-1]))
        resp = _ActionFuture(result=types.SimpleNamespace(goals_canceling=[]))
        if self.client.cancel_raises:
            # 与真实 rclpy 一致：取消应答是 future，失败在应答里（不是同步抛）
            return _ActionFuture(error=RuntimeError('cancel response lost'))
        return resp

    def get_result_async(self):
        fut = _DeferredFuture(_Msg(status=self._result_status), done=self.done)
        self._result_future = fut
        return fut

    def finish(self, status):
        """Nav2 侧给出结果（4 成功 / 5 取消 / 6 中止）。"""
        self.done = True
        self._result_status = status
        if getattr(self, '_result_future', None) is not None:
            self._result_future.finish(_Msg(status=status))


class _DeferredFuture:
    """send_goal_async 的 future（result() 在收到应答前不可用）。

    与 rclpy 一致：``add_done_callback`` 在已完成的 future 上立即回调一次，
    回调拿到的是**已完成的 future 视图**（带最终 status），不是首次登记
    时的对象。
    """

    def __init__(self, result, done=False):
        self._result = result
        self.done = done
        self._cbs = []

    def add_done_callback(self, fn):
        self._cbs.append(fn)
        if self.done:
            fn(_DoneFuture(self._result))

    def result(self):
        if not self.done:
            raise RuntimeError('future not ready')
        return self._result

    def finish(self, result):
        """模拟结果到达：置完成并回调（每个回调拿到已完成视图）。"""
        self.done = True
        self._result = result
        for fn in list(self._cbs):
            fn(_DoneFuture(result))

    def _cbs_attached(self):
        return bool(self._cbs)


class _DoneFuture:
    """已完成 future 的视图。

    回调只接收一个参数（future），与 rclpy 一致：生产代码不能靠
    回调参数个数区分"结果"与"句柄"。返回值刻意不复用 future 本身，
    以免有代码把回调收到的东西当成发送时拿到的 future。
    """

    def __init__(self, result):
        self._result = result
        self.done = True

    def add_done_callback(self, fn):
        fn(self)

    def result(self):
        return self._result


class _PendingAcceptFuture(_DeferredFuture):
    """接受应答挂起：等测试 ``deliver()`` 再给结果。"""

    def __init__(self, handle):
        super().__init__(None, done=False)
        self._handle = handle

    def result(self):
        if not self.done:
            raise RuntimeError('future not ready')
        return self._handle


class FakeActionClient:
    """记录发出的目标；接受与结果由测试显式触发。

    - ``reject=True``：目标被拒绝（accepted=False 的句柄，结果即"失败"）。
    - ``async_accept=True``：send_goal_async 的 future 先挂起，等测试
      ``deliver()`` 再返回句柄——用来模拟"停车之后目标才被接受"。
    - ``cancel_raises``：cancel_goal_async 返回失败的 future（取消失败）。
    """

    def __init__(self, node, action_type, action_name):
        self.node = node
        self.action_type = action_type
        self.action_name = action_name
        self.goals = []
        self.handles = []
        self.reject = False
        self.async_accept = False
        self.cancel_raises = False
        self.send_raises = None
        self._deferred = []

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        if self.send_raises is not None:
            raise self.send_raises
        handle = FakeGoalHandle(self, status=5)
        handle.accepted = not self.reject
        self.handles.append(handle)
        if self.async_accept or getattr(self, 'waiting', False):
            fut = _PendingAcceptFuture(handle)
            self._deferred.append(fut)
            return fut
        return _ActionFuture(result=handle)

    def deliver(self):
        """把挂起的接受回调放行（模拟迟到接受）。"""
        pending, self._deferred = self._deferred, []
        for fut in pending:
            fut.done = True
            for fn in list(fut._cbs):
                fn(_DoneFuture(fut.result()))


_TRACE = False
vision_msgs_msg = types.ModuleType('vision_msgs.msg')
vision_msgs_msg.Detection2DArray = Detection2DArray
vision_msgs_msg.Detection2D = Detection2D
vision_msgs_msg.ObjectHypothesisWithPose = ObjectHypothesisWithPose


class _Logger:
    def __init__(self):
        self.lines = []

    def _log(self, level, text):
        self.lines.append((level, text))

    def info(self, text):
        self._log('info', text)

    def warn(self, text):
        self._log('warn', text)

    def warning(self, text):
        self._log('warn', text)

    def error(self, text):
        self._log('error', text)

    def debug(self, text):
        self._log('debug', text)

    def count(self, level=None):
        if level is None:
            return len(self.lines)
        return sum(1 for lv, _ in self.lines if lv == level)

    def texts(self):
        return [t for _, t in self.lines]


class FakeClock:
    """手工推进的时钟，单位纳秒。"""

    def __init__(self, start_ns=1_000_000_000):
        self.ns = start_ns

    def advance(self, seconds):
        self.ns += int(round(seconds * 1e9))

    def now(self):
        ns = self.ns
        t = types.SimpleNamespace(nanoseconds=ns)
        t.to_msg = lambda: Time(sec=ns // 1_000_000_000,
                                nanosec=ns % 1_000_000_000)
        return t

    def now_msg(self):
        ns = self.ns
        return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    def to_msg(self):
        return self.now_msg()


def describe_qos(qos):
    q = normalize_qos(qos)
    return '%r,%r,depth=%r' % (getattr(q, 'reliability', '?'),
                               getattr(q, 'history', '?'),
                               getattr(q, 'depth', '?'))


class QoSIncompatibleError(RuntimeError):
    """发布端与订阅端 QoS 不兼容。真实 ROS 下表现为收不到消息。"""


class FakeNode:
    """记录参数、发布器和订阅的 Node 替身。

    ``create_subscription`` 时会检查同话题上已有发布端的 QoS 是否兼容。
    不兼容时把细节追加到 ``self.qos_conflicts``；``raise_on_qos_conflict``
    为真时直接抛 ``QoSIncompatibleError``。
    """

    raise_on_qos_conflict = False

    def __init__(self, name, context=None):
        self.name = name
        # 真实 rclpy 的 Node 接受 context 关键字；这里只记录，不模拟域隔离
        self.context = context
        self._params = {}
        self._logger = _Logger()
        self.publishers = {}
        self.subscriptions = {}
        self.qos_conflicts = []
        self.timers = []
        self.clock = FakeClock()
        self.destroyed = False

    # 参数
    def declare_parameter(self, name, value=None):
        self._params[name] = value

    def get_parameter(self, name):
        if name not in self._params:
            raise KeyError('未声明参数: %s' % name)
        return types.SimpleNamespace(value=self._params[name])

    def set_param(self, name, value):
        self._params[name] = value

    # ROS 接口
    def get_logger(self):
        return self._logger

    def create_publisher(self, msg_type, topic, qos):
        profile = normalize_qos(qos)
        pub = types.SimpleNamespace(msg_type=msg_type, topic=topic,
                                    qos=profile, messages=[])
        pub.publish = lambda msg: pub.messages.append(msg)
        self.publishers[topic] = pub
        _topic_endpoints.setdefault(topic, []).append(
            {'role': 'pub', 'qos': profile, 'node': self})
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        profile = normalize_qos(qos)
        sub = types.SimpleNamespace(msg_type=msg_type, topic=topic,
                                    callback=callback, qos=profile)
        self.subscriptions[topic] = sub
        conflicts = self._check_qos(topic, profile)
        _topic_endpoints.setdefault(topic, []).append(
            {'role': 'sub', 'qos': profile, 'node': self})
        if conflicts and self.raise_on_qos_conflict:
            raise QoSIncompatibleError(conflicts[0]['detail'])
        return sub

    def _check_qos(self, topic, sub_qos):
        """登记与同话题发布端的不兼容项。"""
        found = []
        for ep in _topic_endpoints.get(topic, []):
            if ep['role'] != 'pub':
                continue
            ok, reason = qos_compatible(ep['qos'], sub_qos)
            if ok:
                continue
            detail = ('QoS 不兼容: 话题 %s 发布端(%s) 订阅端(%s): %s'
                      % (topic, describe_qos(ep['qos']),
                         describe_qos(sub_qos), reason))
            item = {'topic': topic, 'publisher': ep['qos'],
                    'subscriber': sub_qos, 'reason': reason,
                    'detail': detail}
            self.qos_conflicts.append(item)
            self._logger.error(detail)
            found.append(item)
        return found

    def create_timer(self, period, callback):
        node = self

        # 真实 rclpy 的 Timer 有 cancel()；生产代码用 timer.cancel()，
        # 替身缺这个方法会让 halt_timers() 静默跳过，测试查出 0 个。
        class _Timer:
            def __init__(self):
                self.period = period
                self.callback = callback
                self.cancelled = False

            def cancel(self):
                self.cancelled = True
                if self in node.timers:
                    node.timers.remove(self)

            def reset(self):
                self.cancelled = False

            def is_canceled(self):
                return self.cancelled

        timer = _Timer()
        self.timers.append(timer)
        return timer

    def get_clock(self):
        return self.clock

    def destroy_timer(self, timer):
        # 真实 rclpy 会取消并销毁；这里标记一下，便于断言确实停过
        timer.cancelled = True
        if timer in self.timers:
            self.timers.remove(timer)

    def destroy_node(self):
        self.destroyed = True


class _Policy:
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return self.name

    def __eq__(self, other):
        return isinstance(other, _Policy) and other.name == self.name

    def __hash__(self):
        return hash(self.name)


# 与真实 rclpy 默认值一致：depth=10、RELIABLE、KEEP_LAST、VOLATILE
DEFAULT_DEPTH = 10


class _QosProfile:
    def __init__(self, depth=DEFAULT_DEPTH, **kw):
        self.depth = depth
        self.reliability = _RELIABLE
        self.history = _KEEP_LAST
        self.durability = _VOLATILE
        for k, v in kw.items():
            setattr(self, k, v)

    def __repr__(self):
        return '_QosProfile(depth=%r, reliability=%r, history=%r, durability=%r)' % (
            self.depth, self.reliability, self.history, self.durability)


_RELIABLE = _Policy('RELIABLE')
_BEST_EFFORT = _Policy('BEST_EFFORT')
_KEEP_LAST = _Policy('KEEP_LAST')
_KEEP_ALL = _Policy('KEEP_ALL')
_VOLATILE = _Policy('VOLATILE')
_TRANSIENT_LOCAL = _Policy('TRANSIENT_LOCAL')


def normalize_qos(qos):
    """把 int、None 或 QoSProfile 统一成带 policy 的 profile。

    真实 rclpy 里 ``create_subscription(..., 10)`` 等价于默认
    QoSProfile(depth=10)，即 RELIABLE + KEEP_LAST + VOLATILE。
    """
    if isinstance(qos, _QosProfile):
        return qos
    if qos is None:
        return _QosProfile()
    if isinstance(qos, int):
        return _QosProfile(depth=qos)
    return qos


def reliability_of(qos):
    return normalize_qos(qos).reliability


def qos_compatible(pub_qos, sub_qos):
    """按 ROS 2 规则判断发布端与订阅端能否通信。

    - RELIABLE 发布者 + BEST_EFFORT 订阅者：兼容。
    - BEST_EFFORT 发布者 + RELIABLE 订阅者：不兼容，收不到任何消息。
    - TRANSIENT_LOCAL 订阅者 + VOLATILE 发布者：不兼容。
    返回 ``(ok, reason)``。
    """
    pub = normalize_qos(pub_qos)
    sub = normalize_qos(sub_qos)
    if (getattr(pub, 'reliability', None) == _BEST_EFFORT
            and getattr(sub, 'reliability', None) == _RELIABLE):
        return False, 'RELIABILITY 不兼容: 发布端 BEST_EFFORT 订阅端 RELIABLE'
    if (getattr(sub, 'durability', None) == _TRANSIENT_LOCAL
            and getattr(pub, 'durability', None) == _VOLATILE):
        return False, 'DURABILITY 不兼容: 发布端 VOLATILE 订阅端 TRANSIENT_LOCAL'
    return True, 'ok'


# 话题级注册表：跨节点共享，用来模拟同一话题上的所有端点
_topic_endpoints = {}


def reset_topics():
    """测试用：清空话题端点表。"""
    _topic_endpoints.clear()


def topic_endpoints(topic):
    """返回该话题上的端点列表 [{'role':'pub'|'sub','qos':...,'node':...}]。"""
    return list(_topic_endpoints.get(topic, []))


rclpy_qos = types.ModuleType('rclpy.qos')
rclpy_qos.QoSProfile = _QosProfile
rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(
    RELIABLE=_RELIABLE, BEST_EFFORT=_BEST_EFFORT)
rclpy_qos.HistoryPolicy = types.SimpleNamespace(
    KEEP_LAST=_KEEP_LAST, KEEP_ALL=_KEEP_ALL)
rclpy_qos.DurabilityPolicy = types.SimpleNamespace(
    VOLATILE=_VOLATILE, TRANSIENT_LOCAL=_TRANSIENT_LOCAL)

rclpy_node = types.ModuleType('rclpy.node')
rclpy_node.Node = FakeNode

rclpy_action = types.ModuleType('rclpy.action')
rclpy_action.ActionClient = FakeActionClient
rclpy_action.GoalStatus = types.SimpleNamespace(
    STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6)

rclpy_executors = types.ModuleType('rclpy.executors')


class ExternalShutdownException(Exception):
    """外部触发关闭时抛出，与真实 rclpy 行为一致。"""


rclpy_executors.ExternalShutdownException = ExternalShutdownException

class Context:
    """替身 context。真实 rclpy 的 Node 必须挂在已初始化的 context 上。"""

    _next_id = [0]

    def __init__(self, *a, **k):
        Context._next_id[0] += 1
        self.id = Context._next_id[0]
        self.ok = lambda: True

    def __repr__(self):
        return 'Context(%d)' % self.id


class _FakeNodeWithContext:
    """与 FakeNode 相同，只是构造签名接受 context。"""

    def __new__(cls, name, context=None, **kw):
        return FakeNode(name, context=context)


rclpy = types.ModuleType('rclpy')
rclpy.init = lambda *a, **k: None
rclpy.spin = lambda *a, **k: None
rclpy.shutdown = lambda *a, **k: None
rclpy.try_shutdown = lambda *a, **k: None
rclpy.ok = lambda *a, **k: True
rclpy.time = types.ModuleType('rclpy.time')


class _Time:
    """rclpy.time.Time：to_msg() 给全零 stamp（与真实 ROS 同一形状）。"""

    def __init__(self, seconds=0.0):
        self._s = float(seconds)

    def to_msg(self):
        ns = int(round(self._s * 1e9))
        return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


rclpy.time.Time = _Time
rclpy.Time = _Time
rclpy.Context = Context
rclpy.node = rclpy_node
rclpy.executors = rclpy_executors


def _create_node(name, context=None, **kw):
    """替身 rclpy.create_node。供检查脚本按话题建临时节点用。"""
    node = FakeNode(name, context=context)
    node.get_publishers_info_by_topic = lambda topic: _endpoint_infos(
        'pub', topic)
    node.get_subscriptions_info_by_topic = lambda topic: _endpoint_infos(
        'sub', topic)
    return node


rclpy.create_node = _create_node


class _EndpointInfo:
    def __init__(self, profile):
        self.qos_profile = profile


# 话题 -> {'pub': [QoSProfile], 'sub': [QoSProfile]}，由测试填写
# 检查脚本读取；真实环境里这些来自图发现。
_endpoints = {}


def reset_endpoints():
    _endpoints.clear()


def set_endpoints(topic, pubs=(), subs=()):
    _endpoints[topic] = {'pub': list(pubs), 'sub': list(subs)}


def _endpoint_infos(kind, topic):
    return [_EndpointInfo(p) for p in _endpoints.get(topic, {}).get(kind, [])]


def install():
    """把替身装进 sys.modules 并清空话题端点表。"""
    import sys
    reset_topics()
    reset_endpoints()
    for name, mod in (
            ('diagnostic_msgs', diagnostic_msgs_msg),
            ('sensor_msgs', sensor_msgs_msg),
            ('geometry_msgs', geometry_msgs_msg),
            ('std_msgs', std_msgs_msg),
            ('vision_msgs', vision_msgs_msg),
            ('nav_msgs', nav_msgs_msg),
            ('nav2_msgs', nav2_msgs_msg),
            ('nav2_msgs.action', nav2_msgs_action)):
        pkg = sys.modules.setdefault(name, types.ModuleType(name))
        pkg.msg = mod
        sys.modules[name + '.msg'] = mod
        # 动作包（nav2_msgs.action）：production 写 `from nav2_msgs.action
        # import NavigateToPose`，把对象挂到包上，而不是再建子模块。
        if name.endswith('.action'):
            for attr in dir(mod):
                if not attr.startswith('_'):
                    setattr(pkg, attr, getattr(mod, attr))
    rclpy.qos = rclpy_qos
    sys.modules['rclpy'] = rclpy
    sys.modules['rclpy.node'] = rclpy_node
    sys.modules['rclpy.qos'] = rclpy_qos
    sys.modules['rclpy.executors'] = rclpy_executors
    sys.modules['rclpy.action'] = rclpy_action
