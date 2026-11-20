"""lidar_node.motion_check 纯逻辑测试：扇区距离、方向、轮廓、帧龄、原因。

不依赖 rclpy：扫描用最小替身对象（angle_min/angle_increment/ranges）。
"""

import math

import pytest

from opi_robot.lidar_node import geometry as G
from opi_robot.lidar_node import motion_check as M


class _Scan:
    def __init__(self, ranges, angle_min=0.0):
        self.ranges = list(ranges)
        self.angle_min = angle_min
        self.angle_increment = (2.0 * math.pi / len(ranges)
                                if ranges else 2.0 * math.pi / 360)
        self.range_min = 0.02
        self.range_max = 12.0


def field(fill=math.inf, **by_index):
    ranges = [fill] * 360
    for k, v in by_index.items():
        ranges[int(k) % 360] = v
    return ranges


def blob(center_deg, half_deg, distance, base=math.inf):
    ranges = field(base)
    for d in range(center_deg - half_deg, center_deg + half_deg + 1):
        ranges[d % 360] = distance
    return ranges


GEOM = M.make_geometry()        # 0.25 × 0.17，半角 30°，默认 margin 0.0
MARGINED = M.make_geometry(margin=0.05)


def test_strictly_below_matches_real_order():
    """边界与正负号（原生浮点比较；早先的 Fraction 比较层已移除）。"""
    cases = [(0.45, 0.4, False), (0.39, 0.4, True), (0.4, 0.4, False),
             (1.7, 3.0, True), (12.0, 3.0, False), (3.0, 1.7, False),
             (1e-300, 1e300, True), (1e300, -1e300, False),
             (-1.0, -0.5, True), (-0.5, -1.0, False)]
    for value, limit, expected in cases:
        assert M.strictly_below(value, limit) is expected, (value, limit)
    assert M.strictly_below(float('nan'), 1.0) is False
    assert M.strictly_below(1.0, float('inf')) is False
    assert M.strictly_below(float('inf'), 1.0) is False


def test_outside_range_bounds_inclusive():
    assert M.outside_range(0.02, 0.02, 12.0) is False
    assert M.outside_range(12.0, 0.02, 12.0) is False
    assert M.outside_range(0.019, 0.02, 12.0) is True
    assert M.outside_range(12.001, 0.02, 12.0) is True


# ---------------------------------------------------------------- 角度约定
def test_directions_follow_geometry_sector_convention():
    """车头 0°、左 +90°、右 −90°、后 180°（geometry.Sector 约定）。"""
    assert M.sector_for('front').center_deg == 0.0
    assert M.sector_for('left').center_deg == G.SECTOR_LEFT.center_deg == 90.0
    assert M.sector_for('right').center_deg == G.SECTOR_RIGHT.center_deg == -90.0
    assert abs(abs(M.sector_for('rear').center_deg) - 180.0) < 1e-9
    assert M.sector_for('front', {'half_deg': 20.0}).half_deg == 20.0


def test_unknown_direction_rejected():
    with pytest.raises(ValueError):
        M.sector_for('up')


# ---------------------------------------------------------------- 扇区距离
@pytest.mark.parametrize('direction,center', [
    ('front', 0), ('left', 90), ('right', -90), ('rear', 180)])
def test_each_direction_sees_only_its_own_blob(direction, center):
    scan = _Scan(blob(center, 20, 1.7))
    assert M.sector_nearest(scan, direction, GEOM) == pytest.approx(1.7)
    for other in M.DIRECTIONS:
        if other == direction:
            continue
        assert M.sector_nearest(scan, other, GEOM) is None, (
            '%s 方向的点被 %s 看到' % (direction, other))


def test_nearest_wins():
    ranges = field()
    ranges[0] = 5.0
    ranges[20] = 2.5
    ranges[-20 % 360] = 1.1
    assert M.sector_nearest(_Scan(ranges), 'front', GEOM) == pytest.approx(1.1)


def test_behind_is_not_front():
    scan = _Scan(blob(180, 10, 0.5))
    assert M.obstacle_blocked('front', scan, 0.0, GEOM) == (True, 'front_unknown')
    assert M.obstacle_blocked('rear', scan, 0.0, GEOM) == (False, 'ok')


# ---------------------------------------------------------------- 无效点
def test_invalid_points_ignored_then_unknown():
    ranges = field()
    ranges[0] = float('nan')
    ranges[1] = float('inf')
    ranges[2] = 0.0
    ranges[3] = -1.0
    ranges[4] = 50.0            # 超 range_max
    ranges[5] = 0.001           # 低于 range_min
    assert M.sector_values(_Scan(ranges), 'front', GEOM) == []
    assert M.sector_nearest(_Scan(ranges), 'front', GEOM) is None


def test_empty_sector_is_unknown_never_inf():
    assert M.sector_nearest(_Scan(field()), 'front', GEOM) is None
    assert M.obstacle_blocked('front', _Scan(field()), 0.0, GEOM) == (
        True, 'front_unknown')


def test_no_ranges_message():
    assert M.obstacle_blocked('front', _Scan([]), 0.0, GEOM) == (
        True, 'no_ranges')
    assert M.obstacle_blocked('front', [], 0.0, GEOM) == (True, 'no_ranges')


# ---------------------------------------------------------------- 车体轮廓
def test_body_points_inside_outline_ignored():
    """0.2 m：比车头外轮廓+margin（0.30 m）近，属车身自身回波。"""
    scan = _Scan(blob(0, 30, 0.2))      # 整个前进扇区贴车身
    assert M.sector_values(scan, 'front', GEOM) == []
    assert M.sector_nearest(scan, 'front', GEOM) is None


def test_point_just_outside_outline_counts():
    # 0.25 + 0.05 = 0.30 m 之外才算障碍；0.40 m 外部障碍可见
    scan = _Scan(blob(0, 5, 0.40))
    assert M.sector_nearest(scan, 'front', GEOM) == pytest.approx(0.40)


def test_body_radius_rectangle():
    assert M.body_radius(0.0, GEOM) == pytest.approx(0.25)
    assert M.body_radius(math.pi / 2, GEOM) == pytest.approx(0.15)
    # 45°：矩形边界在侧边 y=0.15 与车头 x=0.25 两条边上取近者：0.15/sin45°
    assert M.body_radius(math.pi / 4, GEOM) == pytest.approx(
        0.15 / math.sin(math.pi / 4))
    assert M.body_radius(30.0 * math.pi / 180, GEOM) == pytest.approx(
        0.25 / math.cos(30.0 * math.pi / 180))


def test_geometry_is_injectable_not_hardcoded():
    wide = M.make_geometry(half_length=0.5)
    assert M.body_radius(0.0, wide) == pytest.approx(0.5)
    assert M.body_radius(0.0, GEOM) == pytest.approx(0.25)


@pytest.mark.parametrize('direction,angle,body,obstacle', [
    ('front', 0, 0.11, 0.15),
    ('rear', 180, 0.25, 0.29),
    ('left', 90, 0.19, 0.21),
    ('right', -90, 0.19, 0.21),
])
def test_offset_outline_filters_body_but_keeps_near_obstacle(
        direction, angle, body, obstacle):
    geom = M.make_geometry(half_length=0.20, half_width=0.20, center_x=-0.08)
    assert M.sector_nearest(_Scan(field(**{str(angle): body})), direction, geom) is None
    scan = _Scan(field(**{str(angle): obstacle}))
    assert M.sector_nearest(scan, direction, geom) == pytest.approx(obstacle)
    assert M.obstacle_blocked(direction, scan, 0.0, geom) == (
        True, f'{direction}_blocked')


def test_offset_outline_keeps_near_obstacle_in_front_gap():
    geom = M.make_geometry(half_length=0.20, half_width=0.20, center_x=-0.08)
    scan = _Scan(field(**{'0': 3.0, '45': 0.20}))
    assert M.obstacle_blocked('front', scan, 0.0, geom) == (True, 'front_gap_blocked')


@pytest.mark.parametrize('center_x', [-0.20, 0.21, math.nan])
def test_center_x_must_keep_laser_inside_outline(center_x):
    geom = M.make_geometry(half_length=0.20, half_width=0.20, center_x=center_x)
    assert M.obstacle_blocked('front', _Scan(field(**{'0': 3.0})), 0.0, geom) == (
        True, 'front_no_geometry')


# ---------------------------------------------------------------- 帧龄
def test_stale_frame_blocks():
    scan = _Scan(blob(0, 20, 3.0))
    assert M.obstacle_blocked('front', scan, 0.6, GEOM) == (True, 'stale')
    assert M.obstacle_blocked('front', scan, 0.5, GEOM) == (False, 'ok')
    assert M.obstacle_blocked('front', scan, None, GEOM) == (True, 'stale')


def test_stale_checked_before_distance():
    scan = _Scan(blob(0, 20, 0.1))
    assert M.obstacle_blocked('front', scan, 9.0, GEOM)[1] == 'stale'


def test_no_frame_and_stale_threshold_injectable():
    assert M.obstacle_blocked('front', None, 0.0, GEOM) == (True, 'no_frame')
    scan = _Scan(blob(0, 20, 3.0))
    assert M.obstacle_blocked('front', scan, 0.3, GEOM, stale_s=0.2) == (
        True, 'stale')


# ---------------------------------------------------------------- 停车距离
def test_stop_distance_boundary():
    scan = _Scan(blob(0, 20, 0.39))
    assert M.obstacle_blocked('front', scan, 0.0, GEOM) == (
        True, 'front_blocked')
    scan = _Scan(blob(0, 20, 0.40))
    assert M.obstacle_blocked('front', scan, 0.0, GEOM) == (False, 'ok')


def test_stop_distance_injectable():
    scan = _Scan(blob(0, 20, 0.6))
    assert M.obstacle_blocked('front', scan, 0.0, GEOM) == (False, 'ok')
    assert M.obstacle_blocked('front', scan, 0.0, GEOM,
                              stop_distance=0.8) == (True, 'front_blocked')


@pytest.mark.parametrize('direction,center', [
    ('rear', 180), ('left', 90), ('right', -90)])
def test_reverse_and_side_directions_block(direction, center):
    far = _Scan(blob(center, 20, 0.9))
    assert M.obstacle_blocked(direction, far, 0.0, GEOM) == (False, 'ok')
    near = _Scan(blob(center, 20, 0.3))
    assert M.obstacle_blocked(direction, near, 0.0, GEOM) == (
        True, '%s_blocked' % direction)


def test_margin_crops_points_near_outline():
    """侧向 0.14 m / 0.12 m 都落在轮廓（0.15）之内，含 margin 0.05 更裁。"""
    on_outline = _Scan(blob(-90, 20, 0.14))
    assert M.sector_nearest(on_outline, 'right', GEOM) is None
    near = _Scan(blob(-90, 20, 0.12))
    # 无 margin 时 0.12 m 小于侧向外轮廓 0.15 m：车身自身回波，不计
    assert M.sector_nearest(near, 'right', GEOM) is None
    assert M.sector_nearest(near, 'right', MARGINED) is None
    assert M.obstacle_blocked('right', near, 0.0, MARGINED) == (
        True, 'right_unknown')


# ---------------------------------------------------------------- 扇区间隙
def test_corner_gap_obstacle_blocks_front():
    """半角 30° 时 45° 车体角在各扇区之外；0.45 m 的近点必须拦停前进。

    同时铺正前方远处覆盖，证明拦停原因来自“隙”而不是“前方未知”。
    """
    ranges = blob(45, 3, 0.35, base=3.0)
    scan = _Scan(ranges)
    assert M.body_radius(math.radians(45), GEOM) + GEOM['margin'] < 0.35
    assert M.sector_nearest(scan, 'front', GEOM) == pytest.approx(3.0)
    assert M.obstacle_blocked('front', scan, 0.0, GEOM) == (
        True, 'front_gap_blocked')


def test_gap_far_obstacle_does_not_block():
    scan = _Scan(blob(45, 3, 0.9, base=3.0))
    assert M.obstacle_blocked('front', scan, 0.0, GEOM) == (False, 'ok')


def test_gap_not_applied_to_opposite_half():
    """45° 的近点在前进半圆内，不拦倒车。"""
    scan = _Scan(blob(45, 3, 0.35))
    # 倒车时后方扇区无点 → rear_unknown（仍是 fail-closed）
    assert M.obstacle_blocked('rear', scan, 0.0, GEOM) == (True, 'rear_unknown')


def test_wide_half_angle_has_no_gap():
    wide = M.make_geometry(half_deg=45.0)
    scan = _Scan(blob(45, 3, 0.35, base=3.0))
    assert M.gap_nearest(scan, 'front', wide) is None
    # 45° 恰在 front 扇区边界内：由扇区本身看到并拦停，不走隙检查
    assert M.obstacle_blocked('front', scan, 0.0, wide) == (
        True, 'front_blocked')


# ---------------------------------------------------------------- 汇总
def test_sector_ranges_shape_and_sharing():
    ranges = field()
    ranges[0] = 1.0
    ranges[90] = 2.0
    ranges[180] = 3.0
    ranges[270] = 4.0
    out = M.sector_ranges(_Scan(ranges), GEOM)
    assert set(out) == {'front', 'rear', 'left', 'right'}
    # 左侧 +90° ⇒ 90°，右侧 −90° ⇒ 270°
    expected = {'front': 1.0, 'left': 2.0, 'rear': 3.0, 'right': 4.0}
    for direction, value in expected.items():
        assert out[direction] == pytest.approx(value), direction


def test_missing_geometry_blocks_every_direction():
    """未注入轮廓（未标定）时任何方向都拦停，不用猜测尺寸。"""
    scan = _Scan(blob(0, 20, 3.0))
    for geom in (None, {}, {'half_length': 0.25}, {'half_length': -0.2,
                                                   'half_width': 0.15},
                 'not-a-dict'):
        for direction in M.DIRECTIONS:
            assert M.obstacle_blocked(direction, scan, 0.0, geom) == (
                True, '%s_no_geometry' % direction), (geom, direction)
    # geometry_missing 拦在计算之前：缺 half_width 也不允许出距离
    side = _Scan(blob(-90, 20, 0.1))
    assert M.sector_nearest(side, 'right', GEOM) is None
    assert M.geometry_missing({'half_length': 0.25}) == ['half_width']
    assert M.sector_values(side, 'right', {'half_length': 0.25}) == []


def test_dict_ranges_accepted():
    vals = {'front': 0.2, 'rear': None, 'left': 2.0, 'right': 2.0}
    assert M.obstacle_blocked('front', vals, 0.0, GEOM) == (
        True, 'front_blocked')
    assert M.obstacle_blocked('rear', vals, 0.0, GEOM) == (True, 'rear_unknown')
    assert M.obstacle_blocked('left', vals, 0.0, GEOM) == (False, 'ok')


def test_blocked_always_has_reason():
    scan = _Scan(blob(0, 20, 3.0))
    for direction in M.DIRECTIONS:
        blocked, reason = M.obstacle_blocked(direction, scan, 0.0, GEOM)
        assert isinstance(blocked, bool)
        assert reason
        assert (reason != 'ok') == blocked
