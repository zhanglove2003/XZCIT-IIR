"""fusion_node 节点级测试：用假 rclpy 驱动真实节点代码。

验证 fail-closed：无诊断、诊断过期、雷达不健康、扫描陈旧、前方无点,
以及恢复后确有数据才放行；角速度全程为零。
"""

import json
import math

import pytest

import fake_ros  # 由 conftest 装载到 sys.modules
from opi_robot.fusion_node import fusion_node
from opi_robot.fusion_node.fusion_node import FusionNode


def scan(ranges):
    msg = fake_ros.LaserScan(ranges=list(ranges), angle_min=0.0,
                             angle_increment=2 * math.pi / len(ranges),
                             angle_max=2 * math.pi - 2 * math.pi / len(ranges),
                             range_min=0.02, range_max=12.0)
    return msg


def ring(**by_deg):
    ranges = [math.inf] * 360
    for key, val in by_deg.items():
        ranges[int(key) % 360] = val
    return scan(ranges)


def all_around(distance, front=None, half=30):
    """前方、左侧（+90°）、右侧（-90°）三个扇区都铺满。

    新规格要求左右任一未知即判 unknown，所以验证 clear/slow/stop
    必须把两侧也铺上有效覆盖。
    """
    f = distance if front is None else front
    data = {str(d): f for d in range(-half, half + 1)}
    for center in (90, -90):
        for d in range(center - half, center + half + 1):
            data[str(d)] = distance
    return data

def diag(level, state='healthy'):
    return fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(level=level, name='lidar', message=state,
                                  values=[fake_ros.KeyValue(key='state',
                                                            value=state)])
    ])


def make_node():
    node = FusionNode()
    return node


def last_report(node):
    msgs = node.publishers['obstacle_report'].messages
    assert msgs, '未发布 obstacle_report'
    return json.loads(msgs[-1].data)


class TestDefaultConfig:
    def test_cmd_vel_disabled_by_default(self):
        node = make_node()
        assert node.enable_cmd_vel is False

    def test_subscribes_expected_topics(self):
        node = make_node()
        for topic in ('scan', 'image_raw/compressed', 'diagnostics'):
            assert topic in node.subscriptions

    def test_publishes_no_visual_target(self):
        node = make_node()
        assert 'visual_target' not in node.publishers


class TestFailClosed:
    def test_no_diagnostics_means_unknown(self):
        node = make_node()
        node.on_scan(ring(**{'0': 2.0}))
        node.decide()
        rep = last_report(node)
        assert rep['action'] == 'unknown'
        assert rep['reason'] == 'no_diagnostics'
        assert rep['front'] is None
        assert rep['linear'] == 0.0
        assert rep['angular'] == 0.0

    def test_no_scan_means_unknown(self):
        node = make_node()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        node.decide()
        rep = last_report(node)
        assert rep['action'] == 'unknown'
        assert rep['reason'] == 'scan_stale'

    def test_unhealthy_lidar_is_unknown(self):
        node = make_node()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.ERROR,
                                 state='disconnected'))
        node.on_scan(ring(**{'0': 2.0}))
        node.decide()
        rep = last_report(node)
        assert rep['action'] == 'unknown'
        assert rep['reason'].startswith('lidar_')
        assert rep['linear'] == 0.0

    def test_stale_diagnostics_is_unknown(self):
        node = make_node()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        node.on_scan(ring(**{'0': 2.0}))
        node.get_clock().advance(3.0)
        node.on_scan(ring(**{'0': 2.0}))
        node.decide()
        rep = last_report(node)
        assert rep['reason'] in ('diagnostics_stale', 'scan_stale')
        assert rep['linear'] == 0.0

    def test_all_invalid_scan_is_unknown(self):
        node = make_node()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        node.on_scan(scan([math.inf] * 360))
        node.decide()
        rep = last_report(node)
        assert rep['action'] == 'unknown'
        assert rep['front'] is None
        assert rep['linear'] == 0.0

    def test_too_few_front_points_is_unknown(self):
        node = make_node()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        node.on_scan(ring(**{'0': 3.0, '1': 3.0}))
        node.decide()
        rep = last_report(node)
        assert rep['action'] == 'unknown'
        assert rep['reason'] == 'front_no_points'

    def test_never_emits_angular_velocity(self):
        node = make_node()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        for r in (0.1, 0.5, 3.0):
            node.on_scan(ring(**{str(d): r for d in range(-5, 6)}))
            node.decide()
            assert last_report(node)['angular'] == 0.0


class TestHealthyBehaviour:
    def _healthy_node(self):
        node = make_node()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        return node

    def _front(self, distance, half=30):
        return {str(d): distance for d in range(-half, half + 1)}


    def test_clear_ahead(self):
        node = self._healthy_node()
        node.on_scan(ring(**all_around(3.0)))
        node.decide()
        rep = last_report(node)
        assert rep['action'] == 'clear'
        # max_linear 上限已统一为 0.4（与桥和航线参数一致）。
        assert rep['linear'] == pytest.approx(0.4)

    def test_stop_close(self):
        node = self._healthy_node()
        node.on_scan(ring(**all_around(3.0, front=0.2)))
        node.decide()
        rep = last_report(node)
        assert rep['action'] == 'stop'
        assert rep['linear'] == 0.0
        assert rep['angular'] == 0.0

    def test_stop_does_not_turn_even_if_sides_free(self):
        """旧实现会原地转向；本阶段必须停。"""
        node = self._healthy_node()
        data = self._front(0.2)
        data.update({str(d): 5.0 for d in range(60, 121)})
        data.update({str(d): 0.3 for d in range(240, 301)})
        node.on_scan(ring(**data))
        node.decide()
        rep = last_report(node)
        assert rep['action'] == 'stop'
        assert rep['angular'] == 0.0

    def test_cmd_vel_not_published_when_disabled(self):
        node = self._healthy_node()
        node.on_scan(ring(**self._front(3.0)))
        node.decide()
        assert node.publishers['auto_cmd_vel'].messages == []

    def test_cmd_vel_zero_when_unknown(self):
        node = make_node()
        node.enable_cmd_vel = True
        node.decide()
        msgs = node.publishers['auto_cmd_vel'].messages
        assert msgs
        assert msgs[-1].linear.x == 0.0
        assert msgs[-1].angular.z == 0.0


class TestCruiseDuration:
    """巡航运行时长（2026-11-14 用户定：无限运行直到手动停止）。

    cruise_timeout ≤ 0 = 不限时；障碍/雷达失效/扫描停更照常停车。
    """

    def _cruising_node(self):
        node = make_node()
        node.enable_cmd_vel = True
        node.body_geometry['half_length'] = 0.25   # 方向门默认启用需标定
        node.body_geometry['half_width'] = 0.15
        node.on_ultrasonic(fake_ros.Range(range=0.8, min_range=0.02,
                                          max_range=4.5))
        node.on_cruise(fake_ros.Bool(data=True))
        return node

    def _fresh(self, node, front=3.0):
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        # 带节点时钟的 header 时间戳（方向门读原始 header.stamp 帧龄；
        # fake_ros 默认 0 = 未填 → stale）
        msg = ring(**all_around(front))
        stamp = node.get_clock().now_msg()
        msg.header.stamp = fake_ros.Time(sec=stamp.sec, nanosec=stamp.nanosec)
        node.on_scan(msg)
        # 2026-11-17 超声门 fail-closed：没有有效读数时许可一律拒绝。
        node.on_ultrasonic(fake_ros.Range(range=0.8, min_range=0.02,
                                          max_range=4.5))
        # 身体轮廓标定：方向门默认启用，未标定会以 front_no_geometry 拦停。
        node.body_geometry['half_length'] = 0.25
        node.body_geometry['half_width'] = 0.15

    def test_runs_indefinitely(self):
        node = self._cruising_node()
        for _ in range(6):                       # 30 s，远超旧 10 s 窗口
            node.get_clock().advance(5.0)
            self._fresh(node)
            # 时间推进 5 s：超声门 fail-closed，必须补一条新读数
            # （实车 10Hz 常态；仿真时钟跳 5 s 不等于传感器断流）。
            node.on_ultrasonic(fake_ros.Range(range=0.8, min_range=0.02,
                                              max_range=4.5))
            node.get_clock().advance(0.1)
            node.decide()
        msgs = node.publishers['auto_cmd_vel'].messages
        assert msgs and msgs[-1].linear.x == pytest.approx(0.4)

    def test_manual_stop_ends_cruise(self):
        node = self._cruising_node()
        self._fresh(node)
        node.decide()
        assert node.publishers['auto_cmd_vel'].messages[-1].linear.x == pytest.approx(0.4)
        node.on_cruise(fake_ros.Bool(data=False))   # 网页"巡航停止"
        self._fresh(node)
        node.decide()
        assert node.publishers['auto_cmd_vel'].messages[-1].linear.x == 0.0

    def test_run_limit_stops_at_deadline_despite_heartbeats(self, monkeypatch):
        elapsed = 10.0
        monkeypatch.setattr(fusion_node, 'monotonic', lambda: elapsed, raising=False)
        node = self._cruising_node()
        node.cruise_max_run_s = 1.0
        node.cruise_timeout = 0.5
        node.on_cruise(fake_ros.Bool(data=False))
        node.on_cruise(fake_ros.Bool(data=True))
        msgs = node.publishers['auto_cmd_vel'].messages

        elapsed = 10.75
        node.on_cruise(fake_ros.Bool(data=True))
        self._fresh(node)
        node.decide()
        assert msgs[-1].linear.x > 0.0

        elapsed = 11.0
        for _ in range(2):
            node.on_cruise(fake_ros.Bool(data=True))
            node.decide()
            assert (msgs[-1].linear.x, msgs[-1].angular.z) == (0.0, 0.0)
            assert last_report(node)['cruise_stop_reason'] == 'run_limit'
            elapsed += 0.25

        node.on_cruise(fake_ros.Bool(data=False))
        node.on_cruise(fake_ros.Bool(data=True))
        node.decide()
        assert msgs[-1].linear.x > 0.0

    def test_obstacle_stops_within_indefinite_cruise(self):
        """无限巡航中遇 0.4 m 内障碍停车（双层：速度置零 + 许可拒绝）。"""
        node = self._cruising_node()
        node.get_clock().advance(5.0)
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        node.on_ultrasonic(fake_ros.Range(range=0.8, min_range=0.02,
                                          max_range=4.5))
        msg = ring(**all_around(3.0, front=0.3))          # < 0.4 新门限
        stamp = node.get_clock().now_msg()
        msg.header.stamp = fake_ros.Time(sec=stamp.sec, nanosec=stamp.nanosec)
        node.on_scan(msg)
        node.on_evidence(fake_ros.String(data=json.dumps(
            {'v': 1, 'epoch': 7, 'frames_ok': 100, 'max_point_age_ms': None})))
        node.on_evidence(fake_ros.String(data=json.dumps(
            {'v': 1, 'epoch': 7, 'frames_ok': 105, 'max_point_age_ms': None})))
        node.decide()
        msgs = node.publishers['auto_cmd_vel'].messages
        assert msgs and msgs[-1].linear.x == 0.0
        st = json.loads(node.publishers['drive_safety'].messages[-1].data)
        assert st['allowed'] is False and st['reason'] == 'obstacle_stop'

    def test_old_025_boundary_now_slows_not_stops(self):
        """原 0.25 m 门限撤销：0.3 m 处障碍在新门限 0.4 m 下停车、
        而按旧门限本不会停——守卫门限变更不被无意回退。"""
        node = self._cruising_node()
        node.get_clock().advance(2.0)
        node.on_ultrasonic(fake_ros.Range(range=0.8, min_range=0.02,
                                          max_range=4.5))
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        msg = ring(**all_around(3.0, front=0.3))
        stamp = node.get_clock().now_msg()
        msg.header.stamp = fake_ros.Time(sec=stamp.sec, nanosec=stamp.nanosec)
        node.on_scan(msg)
        node.on_evidence(fake_ros.String(data=json.dumps(
            {'v': 1, 'epoch': 7, 'frames_ok': 100, 'max_point_age_ms': None})))
        node.on_evidence(fake_ros.String(data=json.dumps(
            {'v': 1, 'epoch': 7, 'frames_ok': 105, 'max_point_age_ms': None})))
        node.decide()
        assert node.publishers['auto_cmd_vel'].messages[-1].linear.x == 0.0

    def test_scan_stale_mid_cruise_also_stops(self):
        """巡航中途扫描停更（>1 s）：未知 → 停车（fail-closed）。"""
        node = self._cruising_node()
        node.on_scan(ring(**all_around(3.0)))
        node.get_clock().advance(2.0)            # 扫描过期、未补新帧
        node.decide()
        msgs = node.publishers['auto_cmd_vel'].messages
        assert msgs and msgs[-1].linear.x == 0.0


class TestImageState:
    def test_unknown_before_first_frame(self):
        node = make_node()
        node.decide()
        assert last_report(node)['image'] == 'unknown'

    def test_fresh_then_stale(self):
        node = make_node()
        node.on_image(fake_ros.CompressedImage(data=b'\xff\xd8' * 100))
        node.decide()
        assert last_report(node)['image'] == 'fresh'
        node.get_clock().advance(2.0)
        node.decide()
        assert last_report(node)['image'] == 'stale'

    def test_no_vision_result_fabricated(self):
        node = make_node()
        node.on_image(fake_ros.CompressedImage(data=b'\xff\xd8' * 100))
        node.decide()
        rep = last_report(node)
        assert rep['vision'] == 'not_implemented'
        assert 'visual_level' not in rep


class TestRecovery:
    def test_recovers_only_with_new_scan(self):
        node = make_node()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        node.on_scan(ring(**all_around(3.0)))
        node.decide()
        assert last_report(node)['action'] == 'clear'

        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.ERROR,
                                 state='disconnected'))
        node.decide()
        assert last_report(node)['action'] == 'unknown'

        node.get_clock().advance(5.0)
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        node.decide()
        # 诊断恢复但扫描仍陈旧，必须继续未知
        assert last_report(node)['action'] == 'unknown'

        node.on_scan(ring(**all_around(3.0)))
        node.decide()
        assert last_report(node)['action'] == 'clear'
