"""全包统一的 QoS 约定。

只在两处使用 QoS：

- 传感器流：``LaserScan``、``CompressedImage``。用 BEST_EFFORT，
  丢掉一帧旧数据比阻塞等重传好。
- 状态消息：``DiagnosticArray``。用 RELIABLE，诊断丢失的代价高。

ROS 2 的 RELIABILITY 不兼容会让发布者与订阅者完全收不到数据。
所以发布端与订阅端必须共用本模块，禁止各自手写。

``SENSOR_QOS_SPEC`` 是不依赖 rclpy 的普通字典，测试可直接读。
``sensor_qos()`` 只依赖 ``rclpy.qos``，在真实 ROS 与伪 ROS 下都可用。
"""

SENSOR_QOS_SPEC = {
    'reliability': 'BEST_EFFORT',
    'history': 'KEEP_LAST',
    'depth': 10,
    'durability': 'VOLATILE',
}

# 状态消息的可靠性。只用于 ``/diagnostics`` 这类不能丢的话题。
STATUS_RELIABILITY = 'RELIABLE'
STATUS_QOS_DEPTH = 10


def _policy_module():
    from rclpy import qos as q
    return q


def sensor_qos(depth=None):
    """按传感器约定构造 QoSProfile。

    depth 缺省取 ``SENSOR_QOS_SPEC['depth']``。调用方不应改返回值里的
    可靠性字段：发布端与订阅端必须一致。
    """
    q = _policy_module()
    profile = q.QoSProfile(depth=SENSOR_QOS_SPEC['depth'] if depth is None
                           else int(depth))
    profile.reliability = getattr(q.ReliabilityPolicy,
                                  SENSOR_QOS_SPEC['reliability'])
    profile.history = getattr(q.HistoryPolicy, SENSOR_QOS_SPEC['history'])
    durability = getattr(q.DurabilityPolicy, 'VOLATILE', None)
    if durability is not None:
        profile.durability = durability
    return profile


def status_qos(depth=None):
    """按状态约定构造 QoSProfile：RELIABLE + KEEP_LAST。"""
    q = _policy_module()
    profile = q.QoSProfile(depth=STATUS_QOS_DEPTH if depth is None
                           else int(depth))
    profile.reliability = getattr(q.ReliabilityPolicy, STATUS_RELIABILITY)
    profile.history = getattr(q.HistoryPolicy, 'KEEP_LAST')
    durability = getattr(q.DurabilityPolicy, 'VOLATILE', None)
    if durability is not None:
        profile.durability = durability
    return profile


def describe(profile):
    """把 QoSProfile 转成可比较的字符串，用于日志与断言。"""
    if profile is None:
        return 'default(RELIABLE,KEEP_LAST)'
    return '%s,%s,depth=%s' % (
        getattr(profile, 'reliability', '?'),
        getattr(profile, 'history', '?'),
        getattr(profile, 'depth', '?'))
