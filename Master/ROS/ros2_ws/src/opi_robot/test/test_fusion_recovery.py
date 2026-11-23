"""第四次审核阻断 1 与阻断 2 的回归测试。

阻断 1：雷达从 connecting 恢复到 healthy 时，旧扫描仍新鲜，
        旧实现直接返回 clear。恢复必须看到 healthy 之后的新扫描。
阻断 2：新诊断缺少 state 时，旧实现沿用上一条的 healthy。
        状态必须来自当前这条诊断。
"""

import json
import math

import fake_ros
from opi_robot import diagnostic_level as DL
from opi_robot.fusion_node.fusion_node import FusionNode


def all_around(distance, half=30):
    """前方、左 +90°、右 -90° 三个扇区都铺满有限值。"""
    ranges = [math.inf] * 360
    for d in range(-half, half + 1):
        ranges[d % 360] = distance
    for center in (90, -90):
        for d in range(center - half, center + half + 1):
            ranges[d % 360] = distance
    return fake_ros.LaserScan(
        ranges=ranges, angle_min=0.0,
        angle_increment=2 * math.pi / 360,
        angle_max=2 * math.pi - 2 * math.pi / 360,
        range_min=0.02, range_max=12.0)


def diag(level, state=None, extra=None):
    """构造雷达诊断。state 为 None 时不带 state 键。"""
    values = []
    if state is not None:
        values.append(fake_ros.KeyValue(key='state', value=state))
    if extra:
        values.extend(extra)
    return fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(
            level=level, name='lidar', message=state or '',
            values=values)
    ])


def report(node):
    msgs = node.publishers['obstacle_report'].messages
    assert msgs, '未发布 obstacle_report'
    return json.loads(msgs[-1].data)


class TestRecoveryNeedsNewScan:
    """阻断 1：旧扫描仍新鲜时，不能靠它恢复。"""

    def test_connecting_then_healthy_without_new_scan_stays_unknown(self):
        node = FusionNode()
        # 1. 正常健康
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        assert report(node)['action'] == 'clear'

        # 2. 进入 connecting。扫描仍在 scan_timeout 窗口内（不推进时钟）。
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.decide()
        assert report(node)['action'] == 'unknown'

        # 3. 直接回到 healthy，但没有新扫描。
        #    旧扫描还在 scan_timeout 内，仍然新鲜。
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown', rep
        assert rep['linear'] == 0.0
        assert rep['angular'] == 0.0
        assert 'recovery' in rep['reason'], rep['reason']

    def test_new_scan_after_healthy_recovers(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.decide()
        # 先恢复 healthy，再来新扫描
        node.get_clock().advance(0.01)
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        assert report(node)['action'] == 'unknown'
        node.get_clock().advance(0.01)
        node.on_scan(all_around(2.0))
        node.decide()
        assert report(node)['action'] == 'clear'

    def test_scan_before_healthy_is_accepted(self):
        """先到新扫描、后到 healthy 诊断：扫描晚于进入不可信时刻，算恢复。

        判据是"扫描晚于进入不可信那一刻"，不是"晚于 healthy 诊断"。
        扫描与诊断是两个独立话题，同一轮里两个顺序都可能出现。
        """
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.decide()

        node.get_clock().advance(0.01)
        node.on_scan(all_around(2.0))          # 扫描先到
        node.get_clock().advance(0.01)
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))   # 诊断后到
        node.decide()
        assert report(node)['action'] == 'clear'

    def test_scan_from_before_losing_trust_is_rejected(self):
        """进入不可信之前到达的扫描，即使还新鲜也不算恢复。"""
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()

        node.get_clock().advance(0.01)
        node.on_scan(all_around(2.0))          # 这一帧在失信任之前
        node.get_clock().advance(0.01)
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.decide()                          # 此刻进入不可信
        node.get_clock().advance(0.01)
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown', rep
        assert rep['reason'] == 'recovery_no_new_scan', rep['reason']

    def test_recovery_reason_no_healthy(self):
        """扫描新鲜但从未收到 healthy 诊断时，原因明确。"""
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown'
        assert rep['reason'] == 'no_diagnostics', rep['reason']

    def test_cmd_vel_zero_during_recovery_gap(self):
        node = FusionNode()
        node.enable_cmd_vel = True
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.decide()
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        tw = node.publishers['auto_cmd_vel'].messages[-1]
        assert tw.linear.x == 0.0
        assert tw.angular.z == 0.0

    def test_recovery_survives_scan_timeout_window(self):
        """旧扫描过期后恢复，同样需要新扫描。"""
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.decide()
        node.get_clock().advance(5.0)               # 扫描与诊断都过期
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown'
        assert rep['linear'] == 0.0
        node.on_scan(all_around(2.0))
        node.decide()
        assert report(node)['action'] == 'clear'


class TestStateMustComeFromCurrentDiagnostic:
    """阻断 2：缺 state 时不能沿用上一条。"""

    def test_missing_state_after_healthy_is_unknown(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        assert report(node)['action'] == 'clear'

        # 新诊断等级 OK，但没有 state 键
        node.on_diagnostics(diag(DL.LEVEL_OK, None))
        assert node.lidar_state == '', node.lidar_state
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown', rep
        assert rep['reason'] == 'state_missing', rep['reason']
        assert rep['linear'] == 0.0

    def test_empty_state_is_unknown(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        for bad in ('', '   ', 'HEALTHY', 'Health', 'booting'):
            node.on_diagnostics(diag(DL.LEVEL_OK, bad))
            node.decide()
            rep = report(node)
            assert rep['action'] == 'unknown', (bad, rep)
            assert rep['linear'] == 0.0

    def test_duplicate_conflicting_state_is_unknown(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy', extra=[
            fake_ros.KeyValue(key='state', value='connecting')]))
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown', rep
        assert node.lidar_state == ''

    def test_duplicate_same_state_is_accepted(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy', extra=[
            fake_ros.KeyValue(key='state', value='healthy')]))
        node.decide()
        assert node.lidar_state == 'healthy'
        assert report(node)['action'] == 'clear'

    def test_unknown_state_value_is_ignored(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        node.on_diagnostics(diag(DL.LEVEL_OK, 'robot_disconnected'))
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown'
        assert rep['reason'] == 'state_robot_disconnected', rep['reason']

    def test_non_lidar_status_does_not_clear_state(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        # 只有别的组件的诊断：不改变雷达状态，也不刷新雷达时间
        other = fake_ros.DiagnosticArray(status=[
            fake_ros.DiagnosticStatus.make(
                level=DL.LEVEL_ERROR, name='camera', message='down',
                values=[fake_ros.KeyValue(key='state', value='disconnected')])
        ])
        node.on_diagnostics(other)
        node.decide()
        assert report(node)['action'] == 'clear'

    def test_multiple_lidar_statuses_conflict_is_unknown(self):
        msg = fake_ros.DiagnosticArray(status=[
            fake_ros.DiagnosticStatus.make(
                level=DL.LEVEL_OK, name='lidar', message='healthy',
                values=[fake_ros.KeyValue(key='state', value='healthy')]),
            fake_ros.DiagnosticStatus.make(
                level=DL.LEVEL_OK, name='lidar', message='connecting',
                values=[fake_ros.KeyValue(key='state', value='connecting')]),
        ])
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(msg)
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown', rep
        assert node.lidar_status is None
        assert node.lidar_state == ''

    def test_multiple_lidar_statuses_agree_is_accepted(self):
        mk = lambda: fake_ros.DiagnosticStatus.make(  # noqa: E731
            level=DL.LEVEL_OK, name='lidar', message='healthy',
            values=[fake_ros.KeyValue(key='state', value='healthy')])
        msg = fake_ros.DiagnosticArray(status=[mk(), mk()])
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(msg)
        node.decide()
        assert report(node)['action'] == 'clear'


class TestUnverifiedGap:
    """第五次审核阻断 1：两条诊断之间没有 decide 调用时，失信不能被抹掉。

    复现顺序（审核报告原文）：
      1. 收到扫描和 OK/healthy，执行一次 decide，得到 clear。
      2. 收到 WARN/connecting；不执行 decide。
      3. 收到 OK/healthy；中间没有任何新扫描，也不执行 decide。
      4. 执行 decide。
    要求：unknown，线速度与角速度都为 0；满足恢复规则的新扫描到达后才可恢复。
    """

    def test_connecting_overwritten_before_decide(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        assert report(node)['action'] == 'clear'

        # 第二条与第三条诊断之间没有 decide
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))

        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown', rep
        assert rep['linear'] == 0.0
        assert rep['angular'] == 0.0
        assert rep['reason'] == 'recovery_no_new_scan', rep['reason']

    def test_recovery_after_new_scan(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        assert report(node)['action'] == 'unknown'

        # 新扫描到达，满足恢复规则
        node.on_scan(all_around(2.0))
        node.decide()
        assert report(node)['action'] == 'clear'

    def test_cmd_vel_publishes_zero_when_enabled(self):
        node = FusionNode()
        node.enable_cmd_vel = True
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        tw = node.publishers['auto_cmd_vel'].messages[-1]
        assert tw.linear.x == 0.0
        assert tw.angular.z == 0.0

    def test_multiple_bad_diagnostics_keep_earliest_boundary(self):
        """连续多条无效诊断，边界取最早那条，不能往后推。"""
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        first = node.lost_seq if hasattr(node, 'lost_seq') else None
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        bound1 = node.lost_seq
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.on_diagnostics(diag(DL.LEVEL_ERROR, 'disconnected'))
        assert node.lost_seq == bound1, (node.lost_seq, bound1)
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        assert report(node)['action'] == 'unknown'
        # 边界之后的新扫描才算恢复
        node.on_scan(all_around(2.0))
        node.decide()
        assert report(node)['action'] == 'clear'

    def test_missing_state_mid_gap(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        node.on_diagnostics(diag(DL.LEVEL_OK, None))   # 缺 state
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown', rep
        assert rep['reason'] == 'recovery_no_new_scan', rep['reason']

    def test_scan_before_and_after_healthy(self):
        """扫描先于 healthy、但晚于失信边界：允许恢复。"""
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))   # 边界在此
        node.on_scan(all_around(2.0))                             # 晚于边界
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        assert report(node)['action'] == 'clear'

    def test_diagnostics_timeout_latches_boundary(self):
        """诊断超时也会锁存边界，之后必须先有新扫描。

        只让诊断过期（lidar_diag_timeout=2.0），把扫描刷新在
        scan_timeout（1.0）之内，否则先返回的是 scan_stale。
        """
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        node.get_clock().advance(1.5)              # 扫描过期
        node.on_scan(all_around(2.0))              # 扫描重新新鲜
        node.get_clock().advance(0.8)              # 诊断年龄 2.3s > 2.0
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown', rep
        assert rep['reason'] == 'diagnostics_stale', rep['reason']
        assert node.lost_seq is not None

        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown', rep
        assert rep['reason'] == 'recovery_no_new_scan', rep['reason']
        node.on_scan(all_around(2.0))
        node.decide()
        assert report(node)['action'] == 'clear'

    def test_scan_timeout_latches_boundary(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        node.get_clock().advance(5.0)
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))   # 只刷新诊断
        node.decide()
        assert report(node)['action'] == 'unknown'
        node.on_scan(all_around(2.0))
        node.decide()
        assert report(node)['action'] == 'clear'

    def test_clean_start_needs_no_recovery(self):
        """启动后首次就健康，不要求先失信任。"""
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        assert report(node)['action'] == 'clear'
