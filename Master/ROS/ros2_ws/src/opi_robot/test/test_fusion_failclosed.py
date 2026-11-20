"""下游 fail-closed 测试：无数据、陈旧、全无效、部分扇区未知、恢复。

只测纯逻辑（opi_robot.fusion_node.failclosed），不依赖 rclpy。
"""

import math

import pytest

from opi_robot.fusion_node import failclosed as F


class Msg:
    """构造 LaserScan 风格的假消息。"""

    def __init__(self, ranges, angle_min=0.0, angle_increment=None):
        self.ranges = list(ranges)
        self.angle_min = angle_min
        n = len(self.ranges)
        self.angle_increment = (2 * math.pi / n if angle_increment is None
                                else angle_increment)
        self.range_min = 0.02
        self.range_max = 12.0


def ring(distance_by_deg):
    """360 个分箱，按角度给距离；未给出的角度为 inf。"""
    ranges = [math.inf] * 360
    for deg, d in distance_by_deg.items():
        ranges[int(deg) % 360] = d
    return Msg(ranges)


class TestFreshness:
    def test_no_scan_ever(self):
        f = F.ScanFreshness(scan_timeout=1.0)
        assert f.age(10.0) is None
        assert f.is_fresh(10.0) is False

    def test_fresh_then_stale(self):
        f = F.ScanFreshness(scan_timeout=0.5)
        f.mark(100.0)
        assert f.is_fresh(100.2) is True
        assert f.is_fresh(100.6) is False

    def test_boundary_is_stale(self):
        f = F.ScanFreshness(scan_timeout=0.5)
        f.mark(1.0)
        assert f.is_fresh(1.5) is False


class TestSector:
    def test_empty_msg_is_unknown(self):
        assert F.sector_distance(None, 0.0, math.radians(30)) is F.UNKNOWN

    def test_all_inf_is_unknown(self):
        msg = Msg([math.inf] * 360)
        assert F.sector_distance(msg, 0.0, math.radians(30)) is F.UNKNOWN

    def test_all_nan_is_unknown(self):
        msg = Msg([math.nan] * 360)
        assert F.sector_distance(msg, 0.0, math.radians(30)) is F.UNKNOWN

    def test_zero_range_dropped(self):
        msg = Msg([0.0] * 360)
        assert F.sector_distance(msg, 0.0, math.radians(30)) is F.UNKNOWN

    def test_front_point_found(self):
        msg = ring({0: 1.2, 5: 0.8, 350: 2.0})
        assert F.sector_distance(msg, 0.0, math.radians(30)) == pytest.approx(0.8)

    def test_point_outside_sector_ignored(self):
        msg = ring({90: 0.5})
        assert F.sector_distance(msg, 0.0, math.radians(30)) is F.UNKNOWN

    def test_partial_sector_unknown(self):
        """前方有数据、右侧全空：右侧必须是未知，不是 inf。"""
        msg = ring({0: 1.5})
        front = F.sector_distance(msg, 0.0, math.radians(30))
        right = F.sector_distance(msg, math.radians(90), math.radians(30))
        assert front == pytest.approx(1.5)
        assert right is F.UNKNOWN

    def test_left_and_right_not_swapped(self):
        """左 -90°、右 +90°，距离不同时不得混淆。"""
        msg = ring({270: 0.6, 90: 3.0})
        left = F.sector_distance(msg, math.radians(-90), math.radians(30))
        right = F.sector_distance(msg, math.radians(90), math.radians(30))
        assert left == pytest.approx(0.6)
        assert right == pytest.approx(3.0)

    def test_angle_min_offset(self):
        """angle_min 非零时扇区判定要按物理角，不是下标。"""
        n = 90
        ranges = [math.inf] * n
        ranges[10] = 2.0
        msg = Msg(ranges, angle_min=-math.pi, angle_increment=2 * math.pi / n)
        phys = -math.pi + 10 * (2 * math.pi / n)
        got = F.sector_distance(msg, phys, math.radians(10))
        assert got == pytest.approx(2.0)

    def test_angle_increment_degenerate(self):
        """只有一个点、增量为 0 时不得除零或死循环。"""
        msg = Msg([1.0], angle_min=0.0, angle_increment=0.0)
        assert F.sector_distance(msg, 0.0, math.radians(30)) == pytest.approx(1.0)


class TestClassify:
    def test_unknown_front_stops(self):
        assert F.classify(None, 5.0, 5.0) == ('unknown', 0.0, 0.0)

    def test_unknown_front_with_unknown_sides_stops(self):
        assert F.classify(None, None, None) == ('unknown', 0.0, 0.0)

    def test_stop_close_and_both_sides_free(self):
        """旧代码会原地转向；现在必须停且角速度为 0。"""
        action, ratio, ang = F.classify(0.2, 9.0, 9.0)
        assert action == 'stop'
        assert ratio == 0.0
        assert ang == 0.0

    def test_stop_close_and_left_unknown(self):
        # 侧向未知时动作升级为 unknown（比 stop 更保守），速度仍为零
        action, ratio, ang = F.classify(0.2, None, 9.0)
        assert (action, ratio, ang) == ('unknown', 0.0, 0.0)

    def test_stop_close_and_right_unknown(self):
        action, ratio, ang = F.classify(0.2, 9.0, None)
        assert (action, ratio, ang) == ('unknown', 0.0, 0.0)

    def test_slow_zone(self):
        action, ratio, ang = F.classify(0.6, 9.0, 9.0)
        assert action == 'slow'
        assert 0.0 < ratio < 1.0
        assert ang == 0.0

    def test_clear(self):
        action, ratio, ang = F.classify(5.0, 9.0, 9.0)
        assert (action, ratio, ang) == ('clear', 1.0, 0.0)

    def test_never_turns(self):
        for front in (None, 0.1, 0.5, 3.0):
            for left in (None, 0.1, 5.0):
                for right in (None, 0.1, 5.0):
                    assert F.classify(front, left, right)[2] == 0.0


class TestLimits:
    def test_unknown_gives_zero_velocity(self):
        assert F.apply_limits('unknown', 1.0, max_linear=0.25) == (0.0, 0.0)

    def test_stop_gives_zero_velocity(self):
        assert F.apply_limits('stop', 1.0, max_linear=0.25) == (0.0, 0.0)

    def test_clear_uses_max_linear(self):
        lin, ang = F.apply_limits('clear', 1.0, max_linear=0.25)
        assert lin == pytest.approx(0.25)
        assert ang == 0.0

    def test_slow_scaled(self):
        lin, _ = F.apply_limits('slow', 0.5, max_linear=0.4)
        assert lin == pytest.approx(0.2)

    def test_negative_ratio_clamped(self):
        lin, _ = F.apply_limits('slow', -1.0, max_linear=0.4)
        assert lin == 0.0


class TestDescribe:
    def test_unknown_written_as_unknown(self):
        text = F.describe(None, None, 1.0, 'unknown')
        assert 'unknown' in text
        assert 'inf' not in text

    def test_known_values_formatted(self):
        text = F.describe(1.234, 2.0, 3.0, 'clear')
        assert 'front=1.23m' in text
        assert 'left=2.00m' in text
        assert 'action=clear' in text


class TestEndToEnd:
    def test_stale_scan_never_moves(self):
        """陈旧扫描：距离可为任何值，速度必须为零。"""
        fresh = F.ScanFreshness(scan_timeout=0.5)
        fresh.mark(1.0)
        assert fresh.is_fresh(2.0) is False
        action, ratio, ang = F.classify(None, 5.0, 5.0)
        lin, ang2 = F.apply_limits(action, ratio)
        assert (lin, ang2) == (0.0, 0.0)

    def test_recovery_after_new_data(self):
        """恢复后左右也必须有有效覆盖，才允许给出前进。"""
        msg = ring({0: 2.0, 90: 2.0, 270: 2.0})
        front = F.sector_distance(msg, 0.0, math.radians(30))
        left = F.sector_distance(msg, math.radians(90), math.radians(30))
        right = F.sector_distance(msg, math.radians(-90), math.radians(30))
        action, ratio, _ = F.classify(front, left, right)
        assert action == 'clear'
        assert F.apply_limits(action, ratio)[0] > 0.0

    def test_recovery_partial_sides_still_unknown(self):
        """只有正前方有数据、左右无覆盖时仍然不算恢复。"""
        msg = ring({0: 2.0})
        front = F.sector_distance(msg, 0.0, math.radians(30))
        left = F.sector_distance(msg, math.radians(90), math.radians(30))
        right = F.sector_distance(msg, math.radians(-90), math.radians(30))
        action, ratio, _ = F.classify(front, left, right)
        assert action == 'unknown'
        assert F.apply_limits(action, ratio)[0] == 0.0


# ---------- Codex 审核第 4 项、第 6 项：左右未知与量程外读数 ----------


class TestSideUnknown:
    """左右任一未知都不得下发前进。

    Codex 复现过的问题：``classify`` 只看 front，左右为 None 时
    仍然可能给出 ``clear`` 或非零比例。未知方向不得被当成空闲。
    """

    @pytest.mark.parametrize('left,right', [
        (None, 1.0),
        (1.0, None),
        (None, None),
    ])
    def test_any_side_unknown_means_no_motion(self, left, right):
        action, ratio, ang = F.classify(2.0, left, right)
        assert action == 'unknown'
        assert ratio == 0.0
        assert ang == 0.0

    def test_unknown_side_blocks_clear(self):
        # 前方 5 m 远，但右侧未知：不能报 clear，也不能前进
        action, ratio, _ = F.classify(5.0, 5.0, None)
        assert action == 'unknown'
        assert F.apply_limits(action, ratio)[0] == 0.0

    def test_all_known_still_works(self):
        action, ratio, ang = F.classify(5.0, 3.0, 3.0)
        assert action == 'clear'
        assert ratio == 1.0
        assert ang == 0.0


class TestRangeLimits:
    """量程外的有限值不得参与决策。

    驱动可能把量程外刻度写成有限数（0.001 m 遮挡码、999 m 超量程码）。
    不检查 ``range_min``/``range_max`` 会让伪距离凭空触发停车或空闲。
    """

    def test_below_range_min_ignored(self):
        msg = ring({})
        msg.ranges[0] = 0.001          # 小于 range_min=0.02
        assert F.sector_values(msg, 0.0, math.radians(30)) == []

    def test_above_range_max_ignored(self):
        msg = ring({})
        msg.ranges[0] = 999.0          # 大于 range_max=12.0
        assert F.sector_values(msg, 0.0, math.radians(30)) == []

    def test_below_range_min_does_not_produce_stop(self):
        msg = ring({})
        for i in range(354, 360):      # 前方扇区塞满 0.001 m 伪值
            msg.ranges[i] = 0.001
        for i in range(0, 7):
            msg.ranges[i] = 0.001
        assert F.sector_distance(msg, 0.0, math.radians(30)) is None

    def test_in_range_value_kept(self):
        msg = ring({0: 1.5})
        vals = F.sector_values(msg, 0.0, math.radians(30))
        assert vals == [1.5]

    def test_zero_range_max_disables_check(self):
        # range_max 缺失或为 0 时不做上限过滤，避免误丢全部读数
        msg = ring({0: 100.0})
        msg.range_max = 0.0
        assert F.sector_values(msg, 0.0, math.radians(30)) == [100.0]


class TestLeftRightAngleConvention:
    """左右扇区角度约定（Codex 审核第 4 项）。

    现场事实：旧节点把雷达原始角直接当 LaserScan 角输出时，
    车左落在约 -90°、车右约 +90°。

    新节点用 ``origin='mirror'`` 发布，LaserScan 角 = (-原始角) mod 360。
    因此车左应落在 +90°，车右应落在 -90°。这与 REP-103 一致。

    该结论尚未在实车复测。本测试只保证代码内部的换算与常量自洽：
    一旦有人把 fusion 的角度常量改回旧值，这里会失败。
    """

    def test_constants_match_mirror_convention(self):
        import math as m
        from opi_robot.fusion_node import fusion_node as FN
        from opi_robot.lidar_node import geometry as G

        layout = G.ScanLayout(bins=360, origin='mirror')
        # 旧节点观察到的原始角：车左 -90°、车右 +90°（等价 270°/90°）
        left_scan = layout.raw_deg_to_scan_deg(-90.0)
        right_scan = layout.raw_deg_to_scan_deg(90.0)
        assert abs((left_scan - 90.0) % 360.0) < 1e-6 or \
            abs((left_scan - 90.0) % 360.0 - 360.0) < 1e-6
        assert abs((right_scan - 270.0) % 360.0) < 1e-6 or \
            abs((right_scan - 270.0) % 360.0 - 360.0) < 1e-6
        # 常量必须与换算结果同号
        assert FN.LEFT_DEG == 90.0
        assert FN.RIGHT_DEG == -90.0
        assert m.radians(FN.LEFT_DEG) == m.pi / 2

    def test_asymmetric_scene_picks_correct_side(self):
        """左侧有箱子、右侧空旷时，left 必须命中，right 必须未知。"""
        from opi_robot.fusion_node import fusion_node as FN
        msg = ring({})
        for i in range(85, 96):        # 扫描角约 +90°
            msg.ranges[i] = 0.6
        left = F.sector_distance(msg, math.radians(FN.LEFT_DEG),
                                 math.radians(30.0))
        right = F.sector_distance(msg, math.radians(FN.RIGHT_DEG),
                                  math.radians(30.0))
        assert left is not None and abs(left - 0.6) < 1e-9
        assert right is None

    def test_right_box_lands_on_right(self):
        from opi_robot.fusion_node import fusion_node as FN
        msg = ring({})
        for i in range(265, 276):      # 扫描角约 270°（即 -90°）
            msg.ranges[i] = 0.8
        left = F.sector_distance(msg, math.radians(FN.LEFT_DEG),
                                 math.radians(30.0))
        right = F.sector_distance(msg, math.radians(FN.RIGHT_DEG),
                                  math.radians(30.0))
        assert right is not None and abs(right - 0.8) < 1e-9
        assert left is None
