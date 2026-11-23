"""drive_safety 节点级测试：安全许可的发布时机与状态矩阵（假 ROS）。

覆盖任务书 T1 的四个必测项：
1. 原始帧停止后缓存 /scan 继续重发，超时限后许可必须翻为拒绝；
2. 障碍/雷达断开/诊断丢失·冲突·无有效点/覆盖未知 × 许可全为拒绝；
3. 恢复只有旧缓存或旧诊断时继续拒绝，真实新数据（证据增长 + 新扫描）
   才重新放行；
4. epoch 变更（节点重启）与计数回退不把旧数据当新数据。
"""

import json
import math
import types

import fake_ros  # 由 conftest 装载到 sys.modules
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
    f = distance if front is None else front
    data = {str(d): f for d in range(-half, half + 1)}
    for center in (90, -90):
        for d in range(center - half, center + half + 1):
            data[str(d)] = distance
    return ring(**data)


def diag(level, state='healthy'):
    return fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(level=level, name='lidar',
                                       message=state,
                                       values=[fake_ros.KeyValue(
                                           key='state', value=state)])
    ])


def evidence_json(epoch=7, count=100, max_age=None):
    return json.dumps({'v': 1, 'epoch': epoch, 'frames_ok': count,
                       'max_point_age_ms': max_age}, separators=(',', ':'))


def feed_evidence(node, epoch=7, count=100, max_age=None):
    node.on_evidence(fake_ros.String(data=evidence_json(epoch, count, max_age)))


def last_safety(node):
    msgs = node.publishers['drive_safety'].messages
    assert msgs, '未发布 drive_safety'
    return json.loads(msgs[-1].data)


def us_range(value):
    """sensor_msgs/Range 替身：``ultrasonic_front``（uart3_node 发布）。"""
    return fake_ros.Range(range=value, min_range=0.02, max_range=4.5)


def make_flowing_node(front=3.0):
    """健康诊断 + 新扫描 + 证据增长两次 → 许可应为允许。

    2026-11-17 起超声波门 fail-closed：从未收到 /ultrasonic_front 一律
    拒绝，所以"应放行"的夹具必须先喂一条有效的远距读数（0.8 m）。
    """
    node = FusionNode()
    node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
    node.on_scan(all_around(front))
    node.on_ultrasonic(us_range(0.8))
    feed_evidence(node, count=100)
    feed_evidence(node, count=105)
    node.decide()
    return node


class TestPermissionBasics:
    def test_flowing_and_clear_is_allowed(self):
        node = make_flowing_node()
        st = last_safety(node)
        assert st['allowed'] is True
        assert st['reason'] == 'ok'
        assert st['seq'] == 1
        assert st['ttl_s'] > 0
        assert st['raw']['flowing'] is True

    def test_no_evidence_denies_even_when_scan_looks_good(self):
        node = FusionNode()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        node.on_scan(all_around(3.0))
        node.decide()
        st = last_safety(node)
        assert st['allowed'] is False
        assert st['reason'] == 'no_evidence'

    def test_single_evidence_message_denies(self):
        """首条证据只建立基线：没有增长证明就不放行。"""
        node = FusionNode()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        node.on_scan(all_around(3.0))
        node.on_ultrasonic(us_range(0.8))
        feed_evidence(node, count=100)
        node.decide()
        assert last_safety(node)['reason'] == 'evidence_no_increase'

    def test_seq_monotonic_and_every_decide_publishes(self):
        node = make_flowing_node()
        seqs = [json.loads(m.data)['seq']
                for m in node.publishers['drive_safety'].messages]
        node.decide()
        node.decide()
        seqs += [json.loads(m.data)['seq']
                 for m in node.publishers['drive_safety'].messages[1:]]
        assert seqs == sorted(seqs)
        assert len(set(seqs)) == len(seqs)


class TestCachedRepublishWindow:
    """必测项 1：原始帧停止，缓存圈继续重发。"""

    def test_frozen_counter_past_deadline_denies(self):
        node = make_flowing_node()
        node.get_clock().advance(0.3)          # 仍在证据时限内
        feed_evidence(node, count=105)         # 缓存重发：计数冻结
        node.on_scan(all_around(3.0))          # lidar 仍重发缓存圈
        node.decide()
        assert last_safety(node)['allowed'] is True
        node.get_clock().advance(0.25)         # 距最后一次增长 0.55 s
        feed_evidence(node, count=105)
        node.on_scan(all_around(3.0))
        node.decide()
        st = last_safety(node)
        assert st['allowed'] is False
        assert st['reason'] == 'raw_frames_stalled'

    def test_evidence_stops_entirely_denies(self):
        node = make_flowing_node()
        node.get_clock().advance(0.55)         # 证据消息不再到达
        node.decide()
        st = last_safety(node)
        assert st['allowed'] is False
        assert st['reason'] == 'evidence_stale'

    def test_report_carries_safety_fields(self):
        node = make_flowing_node()
        rep = json.loads(node.publishers['obstacle_report'].messages[-1].data)
        assert rep['safety_allowed'] is True
        assert rep['raw_evidence']['flowing'] is True


class TestStateMatrix:
    """必测项 2：每种未知/危险状态都必须拒绝许可。"""

    def _denied(self, node):
        st = last_safety(node)
        assert st['allowed'] is False, st
        return st['reason']

    def test_obstacle_detected(self):
        node = make_flowing_node(front=0.2)
        assert self._denied(node) == 'obstacle_stop'

    def test_lidar_disconnected(self):
        node = make_flowing_node()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.ERROR,
                                 state='disconnected'))
        node.on_scan(all_around(3.0))
        feed_evidence(node, count=110)
        node.decide()
        assert self._denied(node).startswith('lidar_')

    def test_diagnostics_stale(self):
        node = make_flowing_node()
        node.get_clock().advance(2.5)          # 超过 lidar_diag_timeout
        node.on_scan(all_around(3.0))
        feed_evidence(node, count=110)
        node.decide()
        assert self._denied(node) in ('diagnostics_stale', 'scan_stale')

    def test_diagnostics_conflict(self):
        node = make_flowing_node()
        conflict = fake_ros.DiagnosticArray(status=[
            fake_ros.DiagnosticStatus.make(level=0, name='lidar',
                                           message='healthy',
                                           values=[fake_ros.KeyValue(
                                               key='state', value='healthy')]),
            fake_ros.DiagnosticStatus.make(level=2, name='lidar',
                                           message='disconnected',
                                           values=[fake_ros.KeyValue(
                                               key='state',
                                               value='disconnected')]),
        ])
        node.on_diagnostics(conflict)
        node.on_scan(all_around(3.0))
        feed_evidence(node, count=110)
        node.decide()
        assert self._denied(node) != 'ok'

    def test_front_no_points(self):
        node = FusionNode()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        node.on_scan(all_around(3.0, front=math.inf))
        node.on_ultrasonic(us_range(0.8))
        feed_evidence(node, count=100)
        feed_evidence(node, count=105)
        node.decide()
        assert self._denied(node) == 'front_no_points'

    def test_side_coverage_unknown(self):
        """左右扇区缺覆盖：判 unknown，拒绝许可。"""
        node = FusionNode()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        data = {str(d): 3.0 for d in range(-30, 31)}   # 只有前方
        node.on_scan(ring(**data))
        node.on_ultrasonic(us_range(0.8))
        feed_evidence(node, count=100)
        feed_evidence(node, count=105)
        node.decide()
        assert self._denied(node) == 'front_no_points'


class TestRecovery:
    """必测项 3：旧缓存/旧诊断不解锁；真实新数据才重新放行。"""

    def test_recovery_requires_new_evidence_and_scan(self):
        node = make_flowing_node()
        # 雷达断开
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.ERROR,
                                 state='disconnected'))
        node.decide()
        assert last_safety(node)['allowed'] is False
        node.get_clock().advance(3.0)
        # 恢复尝试 A：只重放旧诊断，无新证据 → 继续拒绝
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        node.decide()
        st = last_safety(node)
        assert st['allowed'] is False
        assert st['reason'] == 'evidence_stale'
        # 恢复尝试 B：证据恢复但扫描仍是断开前的旧缓存（未再 on_scan）
        feed_evidence(node, count=120)
        node.decide()
        st = last_safety(node)
        assert st['allowed'] is False
        # 恢复尝试 C：新证据 + 断开之后到达的新扫描 → 重新放行
        # （超声门 fail-closed：断开期间读数停更超时 → 需重新启动）
        node.on_scan(all_around(3.0))
        node.us_gate.rearm()
        node.on_ultrasonic(us_range(0.8))
        feed_evidence(node, count=125)
        node.decide()
        assert last_safety(node)['allowed'] is True

    def test_cached_scan_alone_never_unlocks(self):
        """证据冻结时不断喂旧扫描，许可不得回到允许。"""
        node = make_flowing_node()
        node.get_clock().advance(0.6)
        for _ in range(5):                     # 缓存圈连发
            node.on_scan(all_around(3.0))
            node.on_ultrasonic(us_range(0.8))  # 超声在持续出数也不能解锁
            node.decide()
        assert last_safety(node)['allowed'] is False


class TestRestartAndRegression:
    """必测项 4：epoch 变更与计数回退不把旧数据当新数据。"""

    def test_epoch_change_rebuilds(self):
        node = make_flowing_node()             # epoch 7, count 105
        feed_evidence(node, epoch=8, count=0)  # lidar 节点重启
        node.decide()
        st = last_safety(node)
        assert st['allowed'] is False
        assert st['raw']['epoch'] == 8
        feed_evidence(node, epoch=8, count=3)
        node.on_scan(all_around(3.0))
        node.decide()
        assert last_safety(node)['allowed'] is True

    def test_count_regression_denies_until_regrowth(self):
        node = make_flowing_node()             # count 105
        feed_evidence(node, count=90)          # 回退：不可能的递减
        node.decide()
        st = last_safety(node)
        assert st['allowed'] is False
        assert st['raw']['regressions'] == 1
        feed_evidence(node, count=91)
        node.decide()
        assert last_safety(node)['allowed'] is True

    def test_malformed_evidence_counts_not_trusted(self):
        node = make_flowing_node()
        node.on_evidence(fake_ros.String(data='{broken'))
        node.decide()
        st = last_safety(node)
        # 损坏消息不影响已有时限，但被如实计数
        assert st['allowed'] is True
        node.get_clock().advance(0.6)
        node.decide()
        assert last_safety(node)['allowed'] is False


class TestLidarEvidencePublication:
    """lidar_node 侧：每次发布 /scan 同轮发布证据，计数只随新原始帧增长。"""

    def _node_with_fake_serial(self, monkeypatch):
        from opi_robot.lidar_node import lidar_node as LN
        from test_lidar_node import FakeSerial
        from lidar_fixtures import frame_rotation
        PORT = '/dev/serial/by-path/platform-x-usb-0:1.2:1.0-port0'
        FakeSerial.instances = []
        monkeypatch.setattr(LN, '_open_serial',
                            lambda p, b: FakeSerial(p, b))
        monkeypatch.setattr(LN, 'glob', types.SimpleNamespace(
            glob=lambda pat: [PORT]))
        monkeypatch.setattr(LN, 'os', types.SimpleNamespace(
            path=types.SimpleNamespace(exists=lambda p: True)))
        monkeypatch.setattr(LN, 'time', types.SimpleNamespace(
            sleep=lambda s: None, monotonic=lambda: 0.0))
        node = LN.LidarNode()
        return node, FakeSerial, frame_rotation

    def test_evidence_published_alongside_scan(self, monkeypatch):
        node, FakeSerial, frame_rotation = self._node_with_fake_serial(
            monkeypatch)
        node.on_poll()
        serial = FakeSerial.instances[-1]
        for frame in frame_rotation():
            serial.push(frame)
            node.get_clock().advance(0.02)
            node.on_poll()
        node.on_poll()
        scans = node.publishers['scan'].messages
        evidences = node.publishers['scan_raw_evidence'].messages
        assert scans and evidences
        assert len(evidences) == len(scans)
        payload = json.loads(evidences[-1].data)
        assert payload['v'] == 1
        assert payload['frames_ok'] > 0
        assert isinstance(payload['epoch'], int)

    def test_frozen_counter_when_no_new_frames(self, monkeypatch):
        node, FakeSerial, frame_rotation = self._node_with_fake_serial(
            monkeypatch)
        node.on_poll()
        serial = FakeSerial.instances[-1]
        for frame in frame_rotation():
            serial.push(frame)
            node.get_clock().advance(0.02)
            node.on_poll()
        before = json.loads(
            node.publishers['scan_raw_evidence'].messages[-1].data)
        # 不再喂新帧，只推进时钟重发缓存圈：证据条数仍与扫描条数一致
        #（健康窗口内两者同轮发布），但 frames_ok 冻结。
        for _ in range(10):
            node.get_clock().advance(0.02)
            node.on_poll()
        evidences = node.publishers['scan_raw_evidence'].messages
        scans = node.publishers['scan'].messages
        after = json.loads(evidences[-1].data)
        assert after['frames_ok'] == before['frames_ok']
        assert len(evidences) == len(scans)
