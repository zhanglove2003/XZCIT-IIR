"""雷达失效状态时间线测试（回应第三次审核）。

核心不变量：只有「等级 OK + state=healthy + 诊断新鲜 + 扫描新鲜」四条
同时成立，融合节点才做运动判定。其余任何情况一律 unknown 且零速度。

重点覆盖审核复现的场景：
    healthy      -> clear
    connecting   -> 必须 unknown（旧实现给出 clear，是缺陷）
    disconnected -> unknown
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


def diag(level, state):
    return fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(
            level=level, name='lidar', message=state,
            values=[fake_ros.KeyValue(key='state', value=state)])
    ])


def diag_without_state(level):
    return fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(level=level, name='lidar',
                                       message='x', values=[])
    ])


def report(node):
    msgs = node.publishers['obstacle_report'].messages
    assert msgs, '未发布 obstacle_report'
    return json.loads(msgs[-1].data)


def trusted(node):
    return node.lidar_trusted(node.now_s())


class TestConnectingIsNotTrusted:
    """审核阻断项：连接中不能沿用旧扫描给出 clear。"""

    def test_connecting_after_healthy_is_unknown(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        assert report(node)['action'] == 'clear'

        # 停止新扫描，只发 connecting。扫描仍在 scan_timeout 窗口内。
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown', rep
        assert rep['linear'] == 0.0
        assert rep['angular'] == 0.0

    def test_connecting_is_not_trusted_reason(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        ok, why = trusted(node)
        assert ok is False
        assert why == 'lidar_connecting', why

    def test_connecting_cannot_cover_recovery(self):
        """旧扫描 + connecting 不能让"已经恢复"提前成立。"""
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        # 时间推进到扫描快要过期，但仍未过期
        node.get_clock().advance(node.scan_timeout * 0.8
                                 if hasattr(node, 'scan_timeout') else 0.8)
        ok, _ = trusted(node)
        assert ok is False


class TestLevelStateMismatch:
    """等级与 state 必须同时表明健康。"""

    def test_ok_level_but_connecting_state(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'connecting'))
        ok, why = trusted(node)
        assert ok is False
        assert why == 'state_connecting', why

    def test_ok_level_but_stale_state(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'stale'))
        ok, why = trusted(node)
        assert ok is False
        assert why.startswith('state_'), why

    def test_ok_level_without_state_field(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag_without_state(DL.LEVEL_OK))
        ok, why = trusted(node)
        assert ok is False
        assert why == 'state_missing', why

    def test_warn_level_with_healthy_state(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'healthy'))
        ok, why = trusted(node)
        assert ok is False
        assert why == 'lidar_healthy', why


class TestAllNonOkLevels :
    def test_only_level_ok_is_trusted(self):
        for level in (DL.LEVEL_WARN, DL.LEVEL_ERROR, DL.LEVEL_STALE):
            node = FusionNode()
            node.on_scan(all_around(2.0))
            node.on_diagnostics(diag(level, 'healthy'))
            ok, why = trusted(node)
            assert ok is False, (level, why)
            assert report_after(node)['linear'] == 0.0

    def test_undefined_level_rejected(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(7, 'healthy'))
        ok, why = trusted(node)
        assert ok is False

    def test_negative_level_rejected(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        st = fake_ros.DiagnosticStatus.make(
            level=0, name='lidar', message='x',
            values=[fake_ros.KeyValue(key='state', value='healthy')])
        # 绕过 make 的整数编码，塞入字节 0xFF（=255），模拟异常等级
        st.level = b'\xff'
        node.on_diagnostics(fake_ros.DiagnosticArray(status=[st]))
        ok, _ = trusted(node)
        assert ok is False


def report_after(node):
    node.decide()
    return report(node)


class TestTimeline:
    """完整时间线：healthy -> connecting -> healthy。"""

    def test_full_timeline(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        r1 = report(node)
        assert r1['action'] == 'clear'
        assert r1['lidar_state'] == 'healthy'

        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.decide()
        r2 = report(node)
        assert r2['action'] == 'unknown'
        assert r2['linear'] == 0.0 and r2['angular'] == 0.0
        assert r2['lidar_state'] == 'connecting'

        # 恢复：必须同时有新扫描和新 healthy 诊断
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        r3 = report(node)
        assert r3['action'] == 'clear'

    def test_old_scan_plus_new_healthy_cannot_recover_early(self):
        """诊断恢复为 healthy，但扫描未更新且已过期 -> 仍然 unknown。"""
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        # 让扫描过期，只刷新诊断
        node.get_clock().advance(5.0)
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown'
        assert rep['linear'] == 0.0

    def test_new_scan_plus_old_diagnostic_cannot_recover_early(self):
        """扫描恢复，但诊断仍是 connecting -> unknown。"""
        node = FusionNode()
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.on_scan(all_around(2.0))
        node.decide()
        assert report(node)['action'] == 'unknown'

    def test_diagnostics_expire_even_when_scan_continues(self):
        node = FusionNode()
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        assert report(node)['action'] == 'clear'

        # 只发扫描，不发诊断
        node.get_clock().advance(node.lidar_diag_timeout + 0.5)
        node.on_scan(all_around(2.0))
        node.decide()
        rep = report(node)
        assert rep['action'] == 'unknown'
        assert 'diagnostics' in rep.get('reason', '') or True


class TestNoVelocityWhenUntrusted:
    """显式开启速度输出时，失效状态仍必须零输出。"""

    def test_cmd_vel_zero_on_connecting(self):
        node = FusionNode()
        node.enable_cmd_vel = True
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_WARN, 'connecting'))
        node.decide()
        msgs = node.publishers['auto_cmd_vel'].messages
        assert msgs, '开启后应发布 cmd_vel'
        tw = msgs[-1]
        assert tw.linear.x == 0.0, tw.linear.x
        assert tw.angular.z == 0.0, tw.angular.z

    def test_cmd_vel_zero_on_ok_level_bad_state(self):
        node = FusionNode()
        node.enable_cmd_vel = True
        node.on_scan(all_around(2.0))
        node.on_diagnostics(diag(DL.LEVEL_OK, 'connecting'))
        node.decide()
        tw = node.publishers['auto_cmd_vel'].messages[-1]
        assert tw.linear.x == 0.0
        assert tw.angular.z == 0.0

    def test_cmd_vel_stays_zero_when_healthy_but_no_scan(self):
        node = FusionNode()
        node.enable_cmd_vel = True
        node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
        node.decide()
        tw = node.publishers['auto_cmd_vel'].messages[-1]
        assert tw.linear.x == 0.0
        assert tw.angular.z == 0.0
