"""下游失效处理（fail-closed）的纯逻辑部分。

从 fusion_node 抽出来，便于用假时钟做单元测试，不依赖 rclpy。

核心原则：**未知不等于空闲**。
扫描陈旧、雷达不健康、扇区覆盖不足时，对应方向返回 ``None``（未知），
绝不返回 ``inf``（空旷）。速度输出在任一未知条件下都必须为零。
"""

import math

UNKNOWN = None


class ScanFreshness:
    """扫描新鲜度与健康判定的纯逻辑。"""

    def __init__(self, scan_timeout=1.0):
        self.scan_timeout = float(scan_timeout)
        self.last_scan_time = None

    def mark(self, now):
        self.last_scan_time = now

    def age(self, now):
        if self.last_scan_time is None:
            return None
        return now - self.last_scan_time

    def is_fresh(self, now):
        age = self.age(now)
        return age is not None and age < self.scan_timeout


def sector_values(msg, center_rad, half_rad):
    """取一个扇区内所有有效测距值。

    无效值直接丢弃：
    - 非有限值（inf/nan）：该角度没有返回；
    - ``<= 0``：无效量；
    - 超出 ``range_min``/``range_max``：传感器标称量程外的读数不可信。

    最后一条很关键：驱动把量程外刻度写成有限值时（例如 0.001 m 的
    "遮挡错误码"或 999 m 的"超量程码"），不做范围检查会让这些伪距离
    参与决策，可能凭空产生停车或凭空判定空旷。

    返回 list[float]，调用方据此判断"扇区是否未知"。
    """
    if msg is None:
        return []
    ranges = getattr(msg, 'ranges', None)
    if not ranges:
        return []
    angle_min = msg.angle_min
    inc = msg.angle_increment
    rmin = float(getattr(msg, 'range_min', 0.0) or 0.0)
    rmax = float(getattr(msg, 'range_max', 0.0) or 0.0)
    has_range = rmax > 0.0 and rmax > rmin
    out = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        if has_range and (r < rmin or r > rmax):
            continue
        a = angle_min + i * inc
        d = math.atan2(math.sin(a - center_rad), math.cos(a - center_rad))
        if abs(d) <= half_rad:
            out.append(r)
    return out


def sector_distance(msg, center_rad, half_rad, min_points=1):
    """扇区最近距离。点数为 0 时返回 UNKNOWN（None），不是 inf。"""
    vals = sector_values(msg, center_rad, half_rad)
    if len(vals) < min_points:
        return UNKNOWN
    return min(vals)


def classify(front, left, right,
             stop_distance=0.35, slow_distance=0.9):
    """根据三个方向的已知/未知给出动作。

    任一方向未知时，动作都是 unknown 且速度为零。
    左右未知时即便前方空旷也不能输出前进：本阶段不做转向，
    但"不知道旁边有什么"仍不允许车动。
    """
    if front is UNKNOWN or left is UNKNOWN or right is UNKNOWN:
        return 'unknown', 0.0, 0.0
    if front < stop_distance:
        return 'stop', 0.0, 0.0
    if front < slow_distance:
        ratio = (front - stop_distance) / max(slow_distance - stop_distance,
                                              1e-6)
        return 'slow', max(0.0, min(1.0, ratio)), 0.0
    return 'clear', 1.0, 0.0


def apply_limits(action, ratio, max_linear=0.25):
    """把动作比换算成线速度。角速度恒为 0（本阶段不转向）。"""
    if action in ('unknown', 'stop'):
        return 0.0, 0.0
    return max(0.0, ratio) * max_linear, 0.0


def describe(front, left, right, action):
    """生成可读状态文本，未知方向写 unknown，不写 inf。"""
    def fmt(v):
        return 'unknown' if v is UNKNOWN else '%.2fm' % v

    return ('front=%s left=%s right=%s action=%s'
            % (fmt(front), fmt(left), fmt(right), action))
