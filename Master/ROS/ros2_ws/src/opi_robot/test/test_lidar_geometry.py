"""角度映射测试。

区分雷达原始角、ROS LaserScan 角和 base_link 方向。
现场事实：车头约 0°，左约 −90°，右约 +90°，后约 ±180°。
被测模块：opi_robot.lidar_node.geometry
"""

import math

import pytest

from opi_robot.lidar_node import geometry as G


def deg(a):
    return math.degrees(a)


def near(a, b, tol=1e-6):
    return abs(deg(a) - b) <= tol


class TestLayout:
    def test_full_circle(self):
        layout = G.ScanLayout(bins=360)
        assert layout.angle_min == 0.0
        assert layout.angle_increment == pytest.approx(2 * math.pi / 360)
        assert layout.angle_max == pytest.approx(2 * math.pi - layout.angle_increment)
        assert layout.n_bins == 360

    def test_integer_degrees_round_trip(self):
        # 每个整数度都必须落到自己的分箱，不能被浮点误差挤到隔壁
        layout = G.ScanLayout(bins=360)
        for d in range(360):
            idx = layout.index_for_scan_deg(float(d))
            assert layout.center_deg(idx) == pytest.approx(float(d), abs=1e-6)

    def test_center_angle(self):
        # 索引 i 的中心角恰为 i 度，分箱边界落在整数度上
        layout = G.ScanLayout(bins=360)
        assert near(layout.center_angle(0), 0.0)
        assert near(layout.center_angle(90), 90.0)

    def test_offset_shifts_center(self):
        # angle_offset 只改原始角到索引的映射，不改索引自身的几何角
        layout = G.ScanLayout(bins=360, angle_offset_deg=180.0)
        assert near(layout.center_angle(0), 0.0)
        assert layout.raw_deg_to_scan_deg(0.0) == pytest.approx(180.0)


class TestRawToScan:
    def test_raw_angle_is_mirrored(self):
        """原始角 90° 映射到 LaserScan −90°，即现场左侧。"""
        layout = G.ScanLayout(bins=360, angle_offset_deg=0.0)
        idx = layout.index_for_raw_deg(90.0)
        assert near(layout.center_angle(idx), 270.0)

    def test_front_raw_zero_maps_to_zero(self):
        layout = G.ScanLayout(bins=360, angle_offset_deg=0.0)
        idx = layout.index_for_raw_deg(0.0)
        assert near(layout.center_angle(idx), 0.0)

    def test_rear_raw_180_maps_to_180(self):
        layout = G.ScanLayout(bins=360, angle_offset_deg=0.0)
        idx = layout.index_for_raw_deg(180.0)
        assert near(layout.center_angle(idx), 180.0)

    def test_raw_mode_is_identity(self):
        layout = G.ScanLayout(bins=360, angle_offset_deg=0.0, origin='raw')
        idx = layout.index_for_raw_deg(90.0)
        assert near(layout.center_angle(idx), 90.0)

    def test_offset_rotates(self):
        layout = G.ScanLayout(bins=360, angle_offset_deg=180.0)
        idx = layout.index_for_raw_deg(0.0)
        assert near(layout.center_angle(idx), 180.0)

    def test_roundtrip(self):
        layout = G.ScanLayout(bins=360, angle_offset_deg=17.0)
        for raw in (0.0, 45.0, 123.4, 359.9):
            scan = layout.raw_deg_to_scan_deg(raw)
            back = layout.scan_deg_to_raw_deg(scan)
            assert back == pytest.approx(raw, abs=1e-6)

    def test_signed_angle_wraps_to_left(self):
        """LaserScan 270° 等价于 −90°，即左侧。"""
        layout = G.ScanLayout(bins=360)
        rad = layout.signed_scan_rad(90.0)
        assert math.degrees(rad) == pytest.approx(-90.0, abs=1e-6)

    def test_frame_range(self):
        layout = G.ScanLayout(bins=360)
        # 原始 90°→LaserScan 270°，原始 92°→LaserScan 268°
        start, end = layout.frame_index_range(90.0, 92.0)
        assert near(layout.center_angle(start), 270.0)
        assert near(layout.center_angle(end), 268.0)

    def test_frame_range_wraps_start_idx_greater_than_end(self):
        layout = G.ScanLayout(bins=360)
        # 原始 359°→LaserScan 1°，原始 1°→LaserScan 359°；1 小于 359
        start, end = layout.frame_index_range(359.0, 1.0)
        assert start < end

    def test_frame_scan_span(self):
        layout = G.ScanLayout(bins=360)
        # 镜像后帧内角度递减：起点 90°→270°，终点 92°→268°
        a, b = layout.frame_scan_span(90.0, 92.0)
        assert a == pytest.approx(270.0)
        assert b == pytest.approx(268.0)

    def test_mirs_gives_increasing_scan_angle(self):
        """mirs=True 时帧内序号反向，LaserScan 角沿帧序增加。"""
        layout = G.ScanLayout(bins=360)
        # 不反向：起点 0°、终点 358°，跨 0 度绕回
        a, b = layout.frame_scan_span(0.0, 2.0, mirs=False)
        assert (b - a) % 360 == pytest.approx(358.0)
        # 反向：起点 358°、终点 0°，宽度 2 度
        a2, b2 = layout.frame_scan_span(0.0, 2.0, mirs=True)
        assert (b2 - a2) % 360 == pytest.approx(2.0)


class TestSectors:
    def test_left_is_plus_90(self):
        assert G.SECTOR_LEFT.contains(math.radians(90)) is True
        assert G.SECTOR_LEFT.contains(math.radians(60)) is True
        assert G.SECTOR_LEFT.contains(math.radians(120)) is True
        assert G.SECTOR_LEFT.contains(math.radians(125)) is False

    def test_right_is_minus_90(self):
        assert G.SECTOR_RIGHT.contains(math.radians(-90)) is True
        assert G.SECTOR_RIGHT.contains(math.radians(-60)) is True
        assert G.SECTOR_RIGHT.contains(math.radians(-125)) is False

    def test_front_and_rear(self):
        assert G.SECTOR_FRONT.contains(0.0) is True
        assert G.SECTOR_FRONT.contains(math.radians(29)) is True
        assert G.SECTOR_FRONT.contains(math.radians(31)) is False
        assert G.SECTOR_REAR.contains(math.radians(170)) is True
        assert G.SECTOR_REAR.contains(math.radians(-170)) is True

    def test_indices_count(self):
        # -30..30 含端点共 61 个分箱
        layout = G.ScanLayout(bins=360)
        assert len(G.SECTOR_FRONT.indices(layout)) == 61
        assert len(G.SECTOR_LEFT.indices(layout)) == 61
        assert len(G.SECTOR_RIGHT.indices(layout)) == 61
        # 后方 180° 处的分箱边界：±30° 含端点共 61 个
        assert len(G.SECTOR_REAR.indices(layout)) == 61


class TestCoverage:
    def _layout(self, ranges_by_scan_deg):
        layout = G.ScanLayout(bins=360)
        ranges = [float('inf')] * 360
        for d in ranges_by_scan_deg:
            ranges[layout.index_for_scan_deg(d)] = 1.0
        return layout, ranges

    def test_full_front(self):
        # 分箱中心角为整数度，扇区边界含端点，全部 61 个分箱都要填上
        layout, ranges = self._layout([float(d) for d in range(-30, 31)])
        cov = G.front_coverage(layout, ranges, 30.0, 0.02, 12.0)
        assert cov == pytest.approx(1.0, abs=1e-9)

    def test_missing_edge_bin_lowers_coverage(self):
        layout, ranges = self._layout([float(d) for d in range(-29, 31)])
        cov = G.front_coverage(layout, ranges, 30.0, 0.02, 12.0)
        assert cov == pytest.approx(60 / 61.0, abs=1e-9)

    def test_empty_front(self):
        layout, ranges = self._layout([])
        assert G.front_coverage(layout, ranges, 30.0, 0.02, 12.0) == 0.0

    def test_half_front_below_threshold(self):
        layout, ranges = self._layout([float(d) for d in range(-29, 0)])
        assert G.front_coverage(layout, ranges, 30.0, 0.02, 12.0) < 0.6

    def test_out_of_range_not_counted(self):
        layout = G.ScanLayout(bins=360)
        ranges = [13.0] * 360
        assert G.front_coverage(layout, ranges, 30.0, 0.02, 12.0) == 0.0

    def test_nan_not_counted(self):
        layout = G.ScanLayout(bins=360)
        ranges = [1.0] * 360
        ranges[0] = float('nan')
        assert G.front_coverage(layout, ranges, 30.0, 0.02, 12.0) < 1.0

    def test_sector_coverage_center_left(self):
        layout, ranges = self._layout([-90.0, -100.0, -80.0])
        cov = G.sector_coverage(layout, ranges, 30.0, 0.02, 12.0,
                                center_deg=-90.0)
        assert cov > 0.0


class TestValidRange:
    def test_bounds(self):
        assert G.is_valid_range(1.0, 0.02, 12.0) is True
        assert G.is_valid_range(0.01, 0.02, 12.0) is False
        assert G.is_valid_range(13.0, 0.02, 12.0) is False
        assert G.is_valid_range(float('inf'), 0.02, 12.0) is False
        assert G.is_valid_range(float('-inf'), 0.02, 12.0) is False
        assert G.is_valid_range(float('nan'), 0.02, 12.0) is False
