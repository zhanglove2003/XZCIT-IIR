"""按车体轮廓与运动方向判定扇区最近有效障碍（纯逻辑，无 rclpy）。

用途：自动巡航与导航命令的前置门。给定一帧 ``LaserScan``、原始帧龄
（``header.stamp`` 帧龄）、车体轮廓几何与运动方向，回答两件事：

1. 前/后/左/右四个扇区各自的"最近有效障碍距离"——无效点（NaN/inf、
   量程外、落在车体轮廓之内）一律丢弃；扇区无有效点时结果为 ``None``
   （未知），绝不返回 ``inf``（空旷）。
2. ``obstacle_blocked(direction, ranges, age, geometry)`` 是否应拦停，
   以及原因字符串。

角度约定与 :mod:`opi_robot.lidar_node.geometry` 一致（``geometry.Sector``，
LaserScan 角空间，弧度）：

    车头 0°，左侧 +90°，右侧 −90°，车后 ±180°

方向常量直接复用 ``geometry.SECTOR_*``，不另立一套角度。

车体轮廓（``geometry`` 参数）由调用方注入，本模块不写死底盘尺寸：

    geometry = {
        'half_length': 0.25,   # 车头/车尾到中心的距离（m）
        'half_width': 0.15,    # 左侧/右侧到中心的距离（m）
        'center_x': 0.0,       # 车体中心相对雷达的纵向位置（m），向前为正
        'half_deg': 30.0,      # 扇区半角（度），与 geometry.Sector.half_deg 同义
        'margin': 0.0,         # 轮廓外扩：点在 [轮廓, 轮廓+margin] 内不可信
        'range_min': 0.02,     # 雷达最小有效量程（m）
        'range_max': 12.0,     # 雷达最大有效量程（m）
    }

轮廓判定取扫描射线从雷达原点到矩形边缘的距离。前后边缘分别离原点
``half_length + center_x`` 和 ``half_length - center_x``，侧边仍为
``half_width``。距离不大于
``轮廓 + margin`` 的点视为打到车身/支架，不计入障碍统计——否则车体
自身回波会让每个方向恒为"贴脸障碍"。

``age``：原始雷达帧龄（秒，由节点用 ``msg.header.stamp`` 与当前时钟
算出的差；本模块不做记时）。``None`` = 无帧；超过 ``stale_s`` 视为陈旧。

判定顺序（先到先返回，原因串互斥）：

    no_frame      从未收到扫描
    stale         age 为 None 或 age > stale_s（雷达陈旧）
    <dir>_no_geometry
                 half_length/half_width 未注入或不合法（未标定不许动）
    no_ranges     消息没有 ranges
    <dir>_unknown 运动方向扇区无任何有效点（不知道前方有什么）
    <dir>_blocked 运动方向扇区最近障碍 < stop_distance
    <dir>_gap_blocked
                 扇区间空隙（half_deg < 45° 时存在）里的近点 < stop_distance

未知一律拦停（fail-closed）：无帧、陈旧、扇区无有效点都算 blocked。
传入 ``dict`` 形式的 ranges 时只判该方向；若同时带 ``'scan'`` 键
（节点层主动附上原始帧），空隙照判。

**几何缺失即拦停。** ``half_length`` / ``half_width`` 必须由节点参数显式
注入（未标定就不许动），缺项或非正时所有方向判为 ``<dir>_no_geometry``，
不使用任何猜测尺寸的默认值。``DEFAULT_GEOMETRY`` 只用于测试与
``make_geometry`` 的缺省合并。

比较一律用 Python 原生浮点运算符（``strictly_below`` / ``outside_range``
是唯一出口）。早先版本曾因误诊"解释器比较在 0 附近不可信"而引入
整数微度 + ``Fraction`` 比较层，那层在每个扇区判定里都往 stderr 打
``DBGM`` 调试行、并且 ``_deg_units`` 把负角度直接丢弃，导致左/右/后扇区
永远为空（``left_unknown`` 等）。2026-11-17 已整体移除。
"""

import math

from . import geometry as G

FRONT = 'front'
REAR = 'rear'
LEFT = 'left'
RIGHT = 'right'
DIRECTIONS = (FRONT, REAR, LEFT, RIGHT)

#: 方向 → geometry.Sector（中心角按度数约定，见 geometry.SECTOR_*）
BASE_SECTORS = {
    FRONT: G.SECTOR_FRONT,      # 0°
    LEFT: G.SECTOR_LEFT,        # +90°
    RIGHT: G.SECTOR_RIGHT,      # −90°
    REAR: G.SECTOR_REAR,        # ±180°
}

DEFAULT_GEOMETRY = {
    'half_length': 0.25,
    'half_width': 0.15,
    'center_x': 0.0,
    'half_deg': 30.0,
    # 轮廓外扩默认 0：只丢矩形轮廓之内（r <= 轮廓半径）的点。
    # 车体角（45°）外再多给余量会把 45° 方向的真实近障碍一起裁掉。
    'margin': 0.0,
    'range_min': 0.02,
    'range_max': 12.0,
}

#: 默认停车距离，与 fusion_node ``stop_distance`` 同级
#: （现场定为 0.4 m，由节点参数注入；此处只作纯逻辑默认）。
STOP_DISTANCE_M = 0.4
STALE_S = 0.5


# ------------------------------------------------------------ 比较出口
def strictly_below(value, limit):
    """``value < limit``，结果恒为 bool；非有限值一律 False。"""
    try:
        v = float(value)
        l = float(limit)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(v) or not math.isfinite(l):
        return False
    return v < l


def outside_range(r, rmin, rmax):
    """``r < rmin or r > rmax``，结果恒为 bool；边界含入（== 视为有效）。"""
    return strictly_below(r, rmin) or strictly_below(rmax, r)


# ------------------------------------------------------------ 几何
#: 必须由节点注入的轮廓字段（无默认值：未标定不许动）。
REQUIRED_GEOMETRY = ('half_length', 'half_width')


def geometry_missing(geometry):
    """轮廓是否缺失/不合法：返回字段名列表（空列表 = 可用）。"""
    if not isinstance(geometry, dict):
        return list(REQUIRED_GEOMETRY)
    bad = []
    for name in REQUIRED_GEOMETRY:
        value = geometry.get(name)
        try:
            value = float(value)
        except (TypeError, ValueError):
            bad.append(name)
            continue
        if not (value > 0.0):
            bad.append(name)
    if 'half_length' not in bad:
        try:
            center_x = float(geometry.get('center_x', 0.0))
        except (TypeError, ValueError):
            bad.append('center_x')
        else:
            if not abs(center_x) < float(geometry['half_length']):
                bad.append('center_x')
    return bad


def make_geometry(**overrides):
    """合并默认几何与覆盖项，返回新 dict（不改 DEFAULT_GEOMETRY）。"""
    geom = dict(DEFAULT_GEOMETRY)
    geom.update(overrides)
    return geom


def _geom(value):
    """取几何值：dict 缺项按默认补齐；非 dict 视为配置错误。"""
    if value is None:
        return dict(DEFAULT_GEOMETRY)
    if not isinstance(value, dict):
        raise TypeError('geometry 必须是 dict，得到 %r' % (type(value),))
    merged = dict(DEFAULT_GEOMETRY)
    merged.update(value)
    return merged


def sector_for(direction, geometry=None):
    """运动方向对应的扇区（``geometry.Sector``；中心角取自 ``SECTOR_*``）。"""
    if direction not in BASE_SECTORS:
        raise ValueError('未知方向: %r（应为 %s）' % (direction,
                                                    '/'.join(DIRECTIONS)))
    geom = _geom(geometry)
    base = BASE_SECTORS[direction]
    return G.Sector(base.center_deg, float(geom['half_deg']))


def body_radius(bearing_rad, geometry=None):
    """返回沿扫描方位角到轴对齐矩形边缘的距离（m）。"""
    geom = _geom(geometry)
    c = math.cos(bearing_rad)
    s = abs(math.sin(bearing_rad))
    if c == 0.0:
        return float(geom['half_width'])
    center_x = float(geom['center_x'])
    edge = float(geom['half_length']) + (center_x if c > 0.0 else -center_x)
    if s == 0.0:
        return edge
    r_len = edge / abs(c)
    r_wid = float(geom['half_width']) / s
    return r_len if r_len < r_wid else r_wid


def _wrap(a):
    """角度归一到 (-π, π]（同 geometry.wrap_rad）。"""
    return math.atan2(math.sin(a), math.cos(a))


def _valid_point(r, bearing, geom, sector):
    """单个点是否为扇区内有效障碍距离。"""
    r = float(r)
    if not math.isfinite(r) or not (r > 0.0):
        return False
    if outside_range(r, float(geom['range_min']), float(geom['range_max'])):
        return False      # 量程外（边界含入）
    if abs(_wrap(bearing - sector.center)) > sector.half:
        return False      # 不在扇区（原生浮点，不在 0 附近做比较）
    outline = body_radius(bearing, geom) + float(geom['margin'])
    if not (r > outline):
        return False      # 落在车体轮廓（含 margin）之内：车身自身回波
    return True


def _bearings(msg):
    angle_min = float(msg.angle_min)
    inc = float(msg.angle_increment)
    n = len(msg.ranges or [])
    for i, r in enumerate(msg.ranges or []):
        try:
            r = float(r)
        except (TypeError, ValueError):
            continue
        # 索引角度：与 geometry.ScanLayout.center_angle 同式（先乘后加），
        # 保证第 i 个点与该扇区 indices() 选的 bin 一一对应、不差 1 ulp。
        yield r, angle_min + inc * i


def sector_values(msg, direction, geometry=None):
    """扇区内"有效障碍"距离列表。

    丢弃：非有限值、<=0、量程外（range_min/range_max 取几何配置）、
    落在车体轮廓 + margin 内的点（车身自身回波）。
    """
    if msg is None or geometry_missing(geometry):
        return []
    geom = _geom(geometry)
    sector = sector_for(direction, geom)
    return [r for r, b in _bearings(msg) if _valid_point(r, b, geom, sector)]


def sector_nearest(msg, direction, geometry=None):
    """扇区最近有效障碍距离；无有效点返回 ``None``（未知，不是 inf）。"""
    vals = sector_values(msg, direction, geometry)
    return min(vals) if vals else None


def sector_ranges(msg, geometry=None):
    """四个扇区一次算全：``{'front': x|None, 'rear': ..., ...}``。

    节点层每拍调用一次，把结果同时交给门、``/drive_safety``、
    ``obstacle_report``——所有消费者读同一份结果，不各自重算。
    """
    return {d: sector_nearest(msg, d, geometry) for d in DIRECTIONS}


def gap_nearest(msg, direction, geometry=None):
    """运动方向所在半圆内、不属于任何扇区轮廓的有效点最近距离。

    无这类点返回 ``None``。扇区在完整半圆内互不重叠时（half_deg >= 45）
    恒为 ``None``——此时没有空隙可漏。
    """
    if msg is None or geometry_missing(geometry):
        return None
    geom = _geom(geometry)
    sectors = [sector_for(d, geom) for d in DIRECTIONS]
    if all(s.half_deg >= 45.0 for s in sectors):
        return None
    base = BASE_SECTORS[direction]
    half = G.Sector(base.center_deg, 90.0)
    out = []
    for r, bearing in _bearings(msg):
        if any(abs(_wrap(bearing - s.center)) <= s.half for s in sectors):
            continue
        if _valid_point(r, bearing, geom, half):
            out.append(r)
    return min(out) if out else None


def sector_nearest_many(msg, directions, geometry=None):
    """多个方向扇区各自最近有效障碍：``{direction: 距离或 None}``（单遍）。"""
    if msg is None or geometry_missing(geometry):
        return {d: None for d in directions}
    geom = _geom(geometry)
    sectors = [(d, sector_for(d, geom)) for d in directions]
    out = {d: None for d in directions}
    for r, b in _bearings(msg):
        for d, sector in sectors:
            if _valid_point(r, b, geom, sector):
                if out[d] is None or r < out[d]:
                    out[d] = r
    return out


def side_nearest(msg, geometry=None):
    """当前运动中"转弯一侧"最近障碍：左扇区与右扇区取近者（单遍）。

    直线前进时车身朝向不变，但左右任一侧 90° 扇区里的近点决定能否继续
    前进+转向；只判前扇区会漏掉这一侧。值语义与 ``sector_ranges`` 相同：
    无有效点 = ``None``（未知）。
    """
    if msg is None or geometry_missing(geometry):
        return None
    geom = _geom(geometry)
    left = sector_for(LEFT, geom)
    right = sector_for(RIGHT, geom)
    best = None
    for r, b in _bearings(msg):
        for sector in (left, right):
            if _valid_point(r, b, geom, sector):
                if best is None or r < best:
                    best = r
                break
    return best


_UNSET = object()


def obstacle_blocked(direction, ranges, age, geometry=None, *,
                     stop_distance=STOP_DISTANCE_M, stale_s=STALE_S,
                     nearest=_UNSET, side=_UNSET):
    """自动/导航命令的前置门。返回 ``(blocked, reason)``。

    参数
    ----
    direction : 'front'|'rear'|'left'|'right' —— 本拍的运动方向。
    ranges : 本拍的 ``LaserScan`` 消息；或预先算好的 ``dict``
        ``{direction: 最近距离或 None}``（与 ``sector_ranges`` 同形）。
        None = 尚未收到任何扫描。
    age : 原始 ``header.stamp`` 帧龄（秒）或 None（无帧）。
    geometry : 车体轮廓 dict（见模块 docstring）。
    stop_distance : 安全距离（m），严格小于才拦停。
    stale_s : 帧龄上限（秒），严格大于才判陈旧。

    blocked 为 True 时 reason 一定非空且不是 'ok'。
    """
    if direction not in BASE_SECTORS:
        raise ValueError('未知方向: %r' % (direction,))
    if ranges is None:
        return True, 'no_frame'
    try:
        age = None if age is None else float(age)
    except (TypeError, ValueError):
        age = None
    if age is None or strictly_below(stale_s, age):
        return True, 'stale'
    limit = float(stop_distance)
    if geometry_missing(geometry):
        return True, '%s_no_geometry' % (direction,)
    checked = True          # 本调用是否已做过"前进判两侧"
    if nearest is _UNSET:
        if isinstance(ranges, dict) and 'side' not in ranges:
            checked = False
        nearest = ranges if isinstance(ranges, dict) else None
    if nearest is None:
        if not (getattr(ranges, 'ranges', None) or []):
            return True, 'no_ranges'
        nearest = sector_ranges(ranges, geometry)
    value = nearest.get(direction)
    if value is None or not strictly_below(0.0, value):
        return True, '%s_unknown' % (direction,)
    if strictly_below(value, limit):
        return True, '%s_blocked' % (direction,)
    # 转弯一侧：前进方向只在两侧都开阔时才放行。前进时车体横向擦碰的是
    # 左右两侧，不是仅前方 90°；两侧任一近点 < stop_distance 即拦停。
    # 倒车不判侧向（原地转向/后退擦碰由底盘桥与角度命令本身约束）。
    if direction == FRONT and checked:
        # 节点层（已知是前进命令）判两侧；纯 dict 调用只在附带原始帧时判。
        if side is _UNSET and isinstance(ranges, dict):
            side = side_nearest(ranges['scan'], geometry) \
                if ranges.get('scan') is not None else None
        if side is not _UNSET:
            if side is None or strictly_below(side, limit):
                return True, ('side_blocked' if side is not None
                              else 'side_unknown')
    # 半角 < 45° 时相邻扇区之间留有空隙（0.25×0.15 车体的 45° 车体角
    # 恰在空隙里）；落在运动中半球、又不属于任何扇区的近点同样拦停。
    # dict 形式（节点层 `sector_ranges` 的缓存结果）没有原始点，必须回到
    # 本节点缓存的原始帧再判空隙——否则空隙里的近点会绕开门。
    raw = ranges.get('scan') if isinstance(ranges, dict) else ranges
    if raw is not None:
        gap = gap_nearest(raw, direction, geometry)
        if gap is not None and strictly_below(gap, limit):
            return True, '%s_gap_blocked' % (direction,)
    return False, 'ok'


__all__ = ['BASE_SECTORS', 'DEFAULT_GEOMETRY', 'DIRECTIONS', 'FRONT',
           'LEFT', 'REAR', 'RIGHT', 'REQUIRED_GEOMETRY', 'STALE_S',
           'STOP_DISTANCE_M', 'body_radius', 'gap_nearest', 'geometry_missing',
           'make_geometry', 'obstacle_blocked',
           'outside_range', 'sector_for', 'sector_nearest',
           'sector_nearest_many', 'sector_ranges',
           'sector_values', 'side_nearest', 'strictly_below']
