"""角度换算。不依赖 ROS，可用纯 Python 测试。

必须区分的角度空间：

1. 雷达原始角：LD19 帧的 start/end 角，单位 0.01 度，原始顺时针增大。
2. 帧内角：一帧内 12 个点，从 start 到 end 线性插值。
3. ROS ``LaserScan`` 角：索引 i 对应 ``angle_min + i * angle_increment``。
4. ``base_link`` 方向：x 向前，y 向左，逆时针为正。

换算规则（origin=mirror，与 LD19 协议一致）：
雷达原始角加 ``angle_offset_deg`` 后取负号，得到 LaserScan 角。
origin=raw 时不取负号，仅用于现场核对方向。

C50C 现场纸箱对照：LaserScan 中车头约 0°，左约 +90°，右约 −90°。
"""

import math


# 取整保护：浮点误差会让 15.000000000000002 落进下一个分箱
_EPS = 1e-6


def wrap_deg(deg):
    """角度归一到 [0, 360)。"""
    return deg % 360.0


def wrap_rad(rad):
    """弧度归一到 (-pi, pi]。"""
    r = math.fmod(rad, 2.0 * math.pi)
    if r > math.pi:
        r -= 2.0 * math.pi
    elif r <= -math.pi:
        r += 2.0 * math.pi
    return r


class ScanLayout:
    """LaserScan 分箱布局。"""

    def __init__(self, bins=360, angle_offset_deg=0.0, origin='mirror',
                 range_min=0.02, range_max=12.0):
        self.n_bins = int(bins)
        self.angle_offset_deg = float(angle_offset_deg)
        self.origin = origin
        self.range_min = float(range_min)
        self.range_max = float(range_max)
        self.angle_min = 0.0
        self.angle_increment = 2.0 * math.pi / self.n_bins
        self.angle_max = self.angle_min + self.angle_increment * (self.n_bins - 1)

    # ---- 索引与角度 ----

    def center_angle(self, idx):
        """索引对应的 LaserScan 角（弧度，归一到 [0, 2pi)）。"""
        return (self.angle_min + idx * self.angle_increment) % (2.0 * math.pi)

    def center_deg(self, idx):
        return math.degrees(self.center_angle(idx))

    def index_for_scan_rad(self, rad):
        rel = (rad - self.angle_min) % (2.0 * math.pi)
        k = int(math.floor(rel / self.angle_increment + _EPS))
        return k % self.n_bins

    def index_for_scan_deg(self, deg):
        return self.index_for_scan_rad(math.radians(deg))

    def raw_deg_to_scan_deg(self, raw_deg):
        """雷达原始角转 LaserScan 角，结果在 [0, 360)。"""
        shifted = wrap_deg(raw_deg) + self.angle_offset_deg
        if self.origin == 'raw':
            return wrap_deg(shifted)
        return wrap_deg(-shifted)

    def scan_deg_to_raw_deg(self, scan_deg):
        """LaserScan 角反推雷达原始角。"""
        if self.origin == 'raw':
            return wrap_deg(scan_deg - self.angle_offset_deg)
        return wrap_deg(-scan_deg - self.angle_offset_deg)

    def signed_scan_rad(self, raw_deg):
        """雷达原始角对应的 LaserScan 角，归一到 (-pi, pi]。"""
        return wrap_rad(math.radians(self.raw_deg_to_scan_deg(raw_deg)))

    def index_for_raw_deg(self, raw_deg):
        return self.index_for_scan_deg(self.raw_deg_to_scan_deg(raw_deg))

    # ---- 帧覆盖 ----

    def frame_index_range(self, start_deg, end_deg, mirs=False):
        """一帧覆盖的起止索引。输入为雷达原始角（度）。"""
        if mirs:
            start_deg, end_deg = end_deg, start_deg
        return (self.index_for_raw_deg(start_deg),
                self.index_for_raw_deg(end_deg))

    def frame_span_deg(self, start_deg, end_deg, mirs=False):
        """一帧覆盖的扫描角跨度（度，正值）。

        先按原始角算宽度，再换算到扫描角。宽度不随镜像变化，
        所以不能直接对起止角各自换算后相减。
        """
        if mirs:
            start_deg, end_deg = end_deg, start_deg
        return wrap_deg(end_deg - start_deg)

    def frame_scan_span(self, start_deg, end_deg, mirs=False):
        """一帧覆盖的 LaserScan 角起止（度，0 到 360），宽度见 frame_span_deg。"""
        a = self.raw_deg_to_scan_deg(end_deg if mirs else start_deg)
        b = self.raw_deg_to_scan_deg(start_deg if mirs else end_deg)
        return a, b


class Sector:
    """以 LaserScan 角为中心的方向扇区。"""

    def __init__(self, center_deg, half_deg):
        self.center_deg = float(center_deg)
        self.half_deg = float(half_deg)
        self.center = math.radians(self.center_deg)
        self.half = math.radians(self.half_deg)

    def contains(self, rad):
        return abs(wrap_rad(rad - self.center)) <= self.half + 1e-9

    def indices(self, layout):
        return [i for i in range(layout.n_bins)
                if self.contains(layout.center_angle(i))]


def is_valid_range(r, range_min, range_max):
    """距离是否有效：非 NaN 且在量程内。inf 视为无效。"""
    if r != r:            # NaN
        return False
    return range_min <= r <= range_max


def sector_coverage(layout, ranges, half_deg, range_min, range_max,
                    center_deg=0.0):
    """扇区的有效点覆盖率。"""
    sector = Sector(center_deg, half_deg)
    idxs = sector.indices(layout)
    if not idxs:
        return 0.0
    good = sum(1 for i in idxs if is_valid_range(ranges[i], range_min, range_max))
    return good / float(len(idxs))


def front_coverage(layout, ranges, half_deg, range_min, range_max):
    """前方扇区（0 度为中心）的有效点覆盖率。"""
    return sector_coverage(layout, ranges, half_deg, range_min, range_max, 0.0)


SECTOR_FRONT = Sector(0.0, 30.0)
SECTOR_LEFT = Sector(90.0, 30.0)
SECTOR_RIGHT = Sector(-90.0, 30.0)
SECTOR_REAR = Sector(180.0, 30.0)
