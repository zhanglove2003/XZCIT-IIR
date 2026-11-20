"""模拟运行验证：按真实节奏推帧，检查诊断与扫描的时间线一致。

用假时钟模拟 30 帧正常 → 断开 2 秒 → 恢复 30 帧，逐步检查：
- 断线期间 scan 不再发布
- 诊断持续发布且状态为非健康
- 恢复后必须来自新帧
- 全过程不出现"旧测距配新时间戳"
"""

import types

import fake_ros
from opi_robot.lidar_node import health as H
from opi_robot.lidar_node import lidar_node as LN
import lidar_fixtures
from lidar_fixtures import frame_rotation
from test_lidar_node import FakeSerial

PORT = '/dev/serial/by-path/platform-x-usb-0:1.2:1.0-port0'


def make_node(monkeypatch):
    FakeSerial.instances = []
    monkeypatch.setattr(LN, '_open_serial',
                        lambda p, b: FakeSerial(p, b))
    monkeypatch.setattr(LN, 'glob',
                        types.SimpleNamespace(glob=lambda pat: [PORT]))
    monkeypatch.setattr(LN, 'os',
                        types.SimpleNamespace(path=types.SimpleNamespace(
                            exists=lambda p: True)))
    # 单调时钟替身由 conftest 的 autouse fixture 安装到 reader 模块；
    # 模拟里显式取用它，让"时间推进"同时作用于逻辑时钟与测距过期判定
    monkeypatch.setattr(LN, 'time', types.SimpleNamespace(
        sleep=lambda s: None, monotonic=lidar_fixtures.monotonic))

    return LN.LidarNode()


def run(node, seconds, feed=True, step=0.02):
    """按 step 推进假时钟，可选持续推帧。"""
    serial = FakeSerial.instances[-1] if FakeSerial.instances else None
    frames = frame_rotation()
    i = 0
    n = int(round(seconds / step))
    for _ in range(n):
        if feed and serial is not None and i % 3 == 0:
            serial.push(frames[(i // 3) % len(frames)])
        i += 1
        node.get_clock().advance(step)
        lidar_fixtures.advance(step)
        node.on_poll()


def diag(node):
    node.publish_diag()
    st = node.publishers['diagnostics'].messages[-1].status[0]
    return {kv.key: kv.value for kv in st.values}


def scans(node):
    return node.publishers['scan'].messages


class TestTimeline:
    def test_healthy_then_disconnect_then_recover(self, monkeypatch):
        node = make_node(monkeypatch)
        node.on_poll()

        # 阶段 1：连续供帧
        run(node, 2.0, feed=True)
        healthy_count = len(scans(node))
        assert healthy_count > 5, '健康期扫描过少: %d' % healthy_count
        d = diag(node)
        assert d['state'] == H.HEALTHY

        # 阶段 2：停帧。帧龄 < frame_timeout 时数据仍新鲜，可以继续发布；
        # 超过 0.5s 后必须停发，且状态不得为 healthy
        run(node, 0.5, feed=False)      # 越过超时边界
        frozen = len(scans(node))
        run(node, 1.5, feed=False)      # 再停 1.5s
        assert len(scans(node)) == frozen, '断线超时后仍在发布扫描'
        d = diag(node)
        assert d['state'] in (H.STALE, H.CONNECTING, H.DISCONNECTED)
        assert d['state'] != H.HEALTHY

        # 阶段 3：恢复供帧
        run(node, 1.0, feed=True)
        recovered = len(scans(node))
        assert recovered > frozen, '恢复后没有新扫描'
        d = diag(node)
        assert d['state'] == H.HEALTHY

    def test_no_stale_scan_with_fresh_stamp(self, monkeypatch):
        node = make_node(monkeypatch)
        node.on_poll()
        run(node, 1.5, feed=True)
        run(node, 0.6, feed=False)   # 越过超时
        before = [(m.header.stamp.sec, m.header.stamp.nanosec)
                  for m in scans(node)]
        run(node, 3.0, feed=False)
        after = [(m.header.stamp.sec, m.header.stamp.nanosec)
                 for m in scans(node)]
        # 超时后不得产生新扫描；已有扫描的时间戳不变
        assert len(after) == len(before)
        assert after == before

    def test_scan_values_change_only_with_new_frames(self, monkeypatch):
        node = make_node(monkeypatch)
        node.on_poll()
        run(node, 1.0, feed=True)
        run(node, 0.6, feed=False)
        last = list(scans(node)[-1].ranges)
        run(node, 2.0, feed=False)
        # 断线后最后一条扫描的测距值必须与断线前一致，不能被清空后重发
        assert list(scans(node)[-1].ranges) == last

    def test_diagnostics_keep_flowing_while_down(self, monkeypatch):
        node = make_node(monkeypatch)
        node.on_poll()
        run(node, 1.0, feed=True)
        n0 = len(node.publishers['diagnostics'].messages)
        run(node, 2.0, feed=False)
        for _ in range(4):
            node.publish_diag()
        assert len(node.publishers['diagnostics'].messages) > n0

    def test_recovery_requires_new_frames(self, monkeypatch):
        node = make_node(monkeypatch)
        node.on_poll()
        run(node, 1.0, feed=True)
        run(node, 2.0, feed=False)
        assert diag(node)['state'] != H.HEALTHY
        # 只推进时间、不给帧，状态不得自己变好
        node.get_clock().advance(10.0)
        node.on_poll()
        assert diag(node)['state'] != H.HEALTHY


class TestFusionUnderSimulation:
    def test_no_cmd_vel_by_default(self, monkeypatch):
        """默认 enable_cmd_vel=false：一条都不发，而不是发零速。"""
        from opi_robot.fusion_node.fusion_node import FusionNode
        fus = FusionNode()
        for _ in range(5):
            fus.decide()
        assert fus.publishers['auto_cmd_vel'].messages == []

    def test_zero_speed_when_cmd_vel_enabled(self, monkeypatch):
        """显式开启速度输出后，雷达缺失时线速度与角速度都必须为 0。"""
        from opi_robot.fusion_node.fusion_node import FusionNode
        fus = FusionNode()
        fus.enable_cmd_vel = True
        for _ in range(3):
            fus.decide()
        msgs = fus.publishers['auto_cmd_vel'].messages
        assert msgs, '开启后仍未发布 cmd_vel'
        for m in msgs:
            assert m.linear.x == 0.0
            assert m.angular.z == 0.0

    def test_report_says_unknown(self, monkeypatch):
        from opi_robot.fusion_node.fusion_node import FusionNode
        fus = FusionNode()
        fus.decide()
        assert fus.publishers['obstacle_report'].messages, '未发布报告'
        text = fus.publishers['obstacle_report'].messages[-1].data
        assert 'unknown' in text
