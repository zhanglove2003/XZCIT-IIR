"""QoS 约定与兼容性测试。

背景：现场实测 BEST_EFFORT 发布者 + 默认 RELIABLE 订阅者收到 0 条消息，
伪 ROS 旧版不检查 QoS，所以单测全绿但融合节点收不到雷达。

这里做三件事：
1. 断言统一约定本身的值。
2. 断言各节点实际使用的 QoS 与约定一致，且发布端与订阅端兼容。
3. 断言冲突检测本身有效：构造不兼容对，必须被抓到。
"""

import pytest

import fake_ros
from fake_ros import (FakeNode, QoSIncompatibleError, normalize_qos,
                      qos_compatible)
from opi_robot import qos as Q


def _rel(profile):
    return normalize_qos(profile).reliability.name


def _history(profile):
    return normalize_qos(profile).history.name


def _depth(profile):
    return normalize_qos(profile).depth


class TestSpec:
    def test_spec_values(self):
        assert Q.SENSOR_QOS_SPEC['reliability'] == 'BEST_EFFORT'
        assert Q.SENSOR_QOS_SPEC['history'] == 'KEEP_LAST'
        assert Q.SENSOR_QOS_SPEC['depth'] == 10
        assert Q.SENSOR_QOS_SPEC['durability'] == 'VOLATILE'

    def test_sensor_qos_matches_spec(self):
        q = Q.sensor_qos()
        assert _rel(q) == 'BEST_EFFORT'
        assert _history(q) == 'KEEP_LAST'
        assert _depth(q) == 10
        assert q.durability.name == 'VOLATILE'

    def test_sensor_qos_depth_override_keeps_reliability(self):
        q = Q.sensor_qos(depth=1)
        assert _depth(q) == 1
        assert _rel(q) == 'BEST_EFFORT'

    def test_status_qos_is_reliable(self):
        q = Q.status_qos()
        assert _rel(q) == 'RELIABLE'
        assert _history(q) == 'KEEP_LAST'
        assert _depth(q) == 10


class TestFusionSubscriptions:
    def _nodes(self):
        from opi_robot.fusion_node.fusion_node import FusionNode
        from opi_robot.lidar_node.lidar_node import LidarNode
        from opi_robot.web_node.web_debug import Bridge
        return FakeNode('probe'), LidarNode(), FusionNode(), Bridge()

    def test_fusion_scan_reliability_matches_lidar_publisher(self):
        """融合节点订阅 /scan 的可靠性必须与雷达发布端一致。"""
        _, lidar, fusion, _ = self._nodes()
        pub_rel = _rel(lidar.publishers['scan'].qos)
        sub_rel = _rel(fusion.subscriptions['scan'].qos)
        assert pub_rel == sub_rel
        ok, reason = qos_compatible(lidar.publishers['scan'].qos,
                                    fusion.subscriptions['scan'].qos)
        assert ok, reason

    def test_lidar_publisher_uses_sensor_spec(self):
        _, lidar, _, _ = self._nodes()
        assert _rel(lidar.publishers['scan'].qos) == 'BEST_EFFORT'
        assert _depth(lidar.publishers['scan'].qos) == 10

    def test_fusion_image_subscription_uses_sensor_spec(self):
        _, _, fusion, _ = self._nodes()
        assert _rel(fusion.subscriptions['image_raw/compressed'].qos) \
            == 'BEST_EFFORT'

    def test_web_bridge_uses_sensor_spec(self):
        _, _, _, bridge = self._nodes()
        assert _rel(bridge.subscriptions['scan'].qos) == 'BEST_EFFORT'
        assert _rel(bridge.subscriptions['image_raw/compressed'].qos) \
            == 'BEST_EFFORT'

    def test_diagnostics_reliable_on_both_ends(self):
        _, lidar, fusion, bridge = self._nodes()
        pub = lidar.publishers['diagnostics'].qos
        assert _rel(pub) == 'RELIABLE'
        for sub in (fusion.subscriptions['diagnostics'].qos,
                    bridge.subscriptions['diagnostics'].qos):
            assert _rel(sub) == 'RELIABLE'
            ok, reason = qos_compatible(pub, sub)
            assert ok, reason

    def test_no_conflicts_across_full_node_set(self):
        """四个节点同时在场时，任何话题都不得记录 QoS 冲突。"""
        from opi_robot.fusion_node.fusion_node import FusionNode
        from opi_robot.lidar_node.lidar_node import LidarNode
        from opi_robot.web_node.web_debug import Bridge
        lidar = LidarNode()
        fusion = FusionNode()
        bridge = Bridge()
        for node in (lidar, fusion, bridge):
            assert node.qos_conflicts == [], node.qos_conflicts

    def test_old_default_reliable_subscription_is_flagged(self):
        """回归：退回整数深度（默认 RELIABLE）必须被检测出来。"""
        from opi_robot.lidar_node.lidar_node import LidarNode
        lidar = LidarNode()
        probe = FakeNode('probe')
        probe.create_subscription(fake_ros.LaserScan, 'scan',
                                  lambda m: None, 10)
        assert probe.qos_conflicts, '默认 RELIABLE 订阅未被检测为不兼容'
        assert probe.qos_conflicts[0]['topic'] == 'scan'
        assert 'RELIABILITY' in probe.qos_conflicts[0]['reason']


class TestConflictDetection:
    def test_best_effort_pub_vs_reliable_sub_detected(self):
        reset = FakeNode('p')
        reset.create_publisher(fake_ros.LaserScan, '/isolated_qos',
                               Q.sensor_qos())
        sub = FakeNode('s')
        sub.create_subscription(fake_ros.LaserScan, '/isolated_qos',
                                lambda m: None, 10)
        assert sub.qos_conflicts
        assert sub.qos_conflicts[0]['topic'] == '/isolated_qos'

    def test_best_effort_pub_vs_best_effort_sub_ok(self):
        FakeNode('p').create_publisher(fake_ros.LaserScan, '/ok',
                                       Q.sensor_qos())
        sub = FakeNode('s')
        sub.create_subscription(fake_ros.LaserScan, '/ok', lambda m: None,
                                Q.sensor_qos())
        assert sub.qos_conflicts == []

    def test_reliable_pub_vs_best_effort_sub_ok(self):
        FakeNode('p').create_publisher(fake_ros.LaserScan, '/ok2', 10)
        sub = FakeNode('s')
        sub.create_subscription(fake_ros.LaserScan, '/ok2', lambda m: None,
                                Q.sensor_qos())
        assert sub.qos_conflicts == []

    def test_raise_mode(self):
        FakeNode('p').create_publisher(fake_ros.LaserScan, '/strict',
                                       Q.sensor_qos())
        node = FakeNode('s')
        node.raise_on_qos_conflict = True
        with pytest.raises(QoSIncompatibleError):
            node.create_subscription(fake_ros.LaserScan, '/strict',
                                     lambda m: None, 10)

    def test_cross_topic_not_flagged(self):
        FakeNode('p').create_publisher(fake_ros.LaserScan, '/topic_a',
                                       Q.sensor_qos())
        sub = FakeNode('s')
        sub.create_subscription(fake_ros.LaserScan, '/topic_b',
                                lambda m: None, 10)
        assert sub.qos_conflicts == []

    def test_qos_compatible_pure_function(self):
        assert qos_compatible(Q.sensor_qos(), Q.sensor_qos())[0] is True
        assert qos_compatible(Q.sensor_qos(), 10)[0] is False
        assert qos_compatible(Q.status_qos(), 10)[0] is True
