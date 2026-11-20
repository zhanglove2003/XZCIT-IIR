"""雷达扫描聚合测试：按完整旋转生成扫描、单角度过期、逐帧累计。

覆盖两个 P1 缺陷：

- 缺陷 A：ring 只保留历史最小值，导致近目标移开后近距离长期残留。
  现在每格保存"最近一次距离 + 收到时刻"，超过 ``stale_point_age`` 输出 inf。
- 缺陷 B：一次 read 解出多个协议帧时只把最后一帧送进健康状态机。
  现在每个帧都调用 ``monitor.on_scan``。

被测模块：opi_robot.lidar_node.reader（LidarRunner）
"""

import math
import os
import runpy
import sys
import types

import pytest

from opi_robot.lidar_node import health as H
from opi_robot.lidar_node import reader as R
from lidar_fixtures import Clock, build_frame, frame_rotation

PORT = '/dev/serial/by-path/platform-x-usb-0:1.2:1.0-port0'

# 覆盖阈值放宽到 0.3，便于在单帧场景下进入 healthy 后观察扫描内容
LOOSE_COVERAGE = 0.3


class _Serial:
    """伪串口：可注入数据、抛错误。"""

    def __init__(self, path, baud):
        self.path = path
        self.baud = baud
        self.chunks = bytearray()
        self.closed = False
        _Serial.instances.append(self)

    def push(self, data):
        self.chunks.extend(data)

    def read(self, n=4096):
        if not self.chunks:
            return b''
        data = bytes(self.chunks[:n])
        del self.chunks[:n]
        return data

    def close(self):
        self.closed = True


_Serial.instances = []


def make_runner(clock, mono, **kwargs):
    # 本测试模块独立实例表，避免与其他模块的 FakeSerial 互相干扰
    params = dict(
        port=PORT,
        baud=230400,
        frame_timeout=0.5,
        recovery_coverage=LOOSE_COVERAGE,
        reconnect_interval=1.0,
        log_interval=10.0,
        mirror=False,
    )
    params.update(kwargs)
    return R.LidarRunner(
        clock=clock.now,
        open_port=lambda p, b: _Serial(p, b),
        path_exists=lambda p: True,
        **params)


@pytest.fixture(autouse=True)
def reset_serials():
    _Serial.instances = []
    yield
    _Serial.instances = []


@pytest.fixture
def mono(monkeypatch):
    """可控单调时钟：只替换 reader 模块内的 time。"""
    clk = Clock(start_ns=5_000_000_000)

    def monotonic():
        return clk.seconds

    monkeypatch.setattr(R, 'time',
                        types.SimpleNamespace(monotonic=monotonic))
    return clk


def _push_rotation(serial, runner, clock):
    for frame in frame_rotation():
        serial.push(frame)
        clock.advance(0.02)
        runner.poll()


class TestLatestDistance:
    def test_newer_farther_value_overwrites(self, mono):
        """同一角度先 0.5 m 再 2.0 m，输出必须是 2.0 m。"""
        clock = Clock()
        runner = make_runner(clock, mono)
        runner.poll()
        serial = _Serial.instances[-1]

        _push_rotation(serial, runner, clock)
        clock.advance(0.01)
        assert runner.poll() is not None
        assert runner.monitor.state == H.HEALTHY

        # 只重扫前方 0 度这一格：先在 0.5 m，再在 2.0 m
        near = build_frame(start_ddeg=0, end_ddeg=100, points=[500] + [9999] * 11)
        serial.push(near)
        clock.advance(0.02)
        mono.advance(0.02)
        runner.poll()

        far = build_frame(start_ddeg=0, end_ddeg=100, points=[2000] + [9999] * 11)
        serial.push(far)
        clock.advance(0.02)
        mono.advance(0.02)
        scan = runner.poll()

        # mirror=False 时原始 0.01 度落在分箱 0
        assert scan['ranges'][0] == pytest.approx(2.0, abs=1e-6), (
            'ring 仍在保留历史最小值：%r' % (scan['ranges'][:3],))

    def test_partial_refresh_keeps_fresh_angles(self, mono):
        """有新数据的角度保留新值，未刷新的角度在期限内保留旧值。"""
        clock = Clock()
        runner = make_runner(clock, mono)
        runner.poll()
        serial = _Serial.instances[-1]
        _push_rotation(serial, runner, clock)
        clock.advance(0.01)
        assert runner.poll() is not None

        serial.push(build_frame(start_ddeg=0, end_ddeg=100,
                                points=[3000] * 12))
        clock.advance(0.02)
        mono.advance(0.2)
        scan = runner.poll()
        assert scan['ranges'][0] == pytest.approx(3.0, abs=1e-6)
        # 其他角度还在 stale_point_age 内，仍输出旧值
        finite = [r for r in scan['ranges'] if math.isfinite(r)]
        assert len(finite) > 12


class TestPointExpiry:
    def test_target_moved_away_becomes_inf(self, mono):
        """近目标移开后，超过 stale_point_age 该角度必须变 inf。"""
        clock = Clock()
        runner = make_runner(clock, mono, stale_point_age=1.5)
        runner.poll()
        serial = _Serial.instances[-1]
        _push_rotation(serial, runner, clock)
        clock.advance(0.01)
        assert runner.poll() is not None

        serial.push(build_frame(start_ddeg=0, end_ddeg=100,
                                points=[500] * 12))
        clock.advance(0.02)
        mono.advance(0.02)
        scan = runner.poll()
        assert scan['ranges'][0] == pytest.approx(0.5, abs=1e-6)

        # 目标移开：该角度不再有数据，但雷达仍有其他角度的新帧
        clock.advance(0.02)
        mono.advance(1.6)
        serial.push(build_frame(start_ddeg=9000, end_ddeg=9100,
                                points=[4000] * 12))
        clock.advance(0.02)
        scan = runner.poll()
        assert scan is not None
        assert scan['ranges'][0] == math.inf, (
            '近目标移开后 0 度仍在输出 %r' % (scan['ranges'][0],))

    def test_partial_angles_expire_others_do_not(self, mono):
        """只有部分角度持续有数据时，其余角度超时后为 inf。"""
        clock = Clock()
        runner = make_runner(clock, mono, stale_point_age=0.5)
        runner.poll()
        serial = _Serial.instances[-1]
        _push_rotation(serial, runner, clock)
        clock.advance(0.01)
        assert runner.poll() is not None

        # 只持续刷新原始 90 度附近，其余角度不再有数据
        for _ in range(5):
            serial.push(build_frame(start_ddeg=9000, end_ddeg=9030,
                                    points=[2000] * 12))
            clock.advance(0.02)
            mono.advance(0.2)
            scan = runner.poll()
            assert scan is not None

        live = [i for i, r in enumerate(scan['ranges']) if math.isfinite(r)]
        assert live, '持续刷新的角度也过期了'
        # 只有持续刷新的那一帧范围内的角度存活，其余全部过期
        assert len(live) <= 20, (
            '未刷新的角度仍在输出旧值：存活 %d 格 %r' % (len(live), live))

    def test_endless_repeat_blocked(self, mono):
        """没有新帧时，超过 stale_point_age 后不得重复发布旧扫描。"""
        clock = Clock()
        runner = make_runner(clock, mono, stale_point_age=0.3,
                             frame_timeout=10.0)
        runner.poll()
        serial = _Serial.instances[-1]
        _push_rotation(serial, runner, clock)
        clock.advance(0.01)
        assert runner.poll() is not None

        finite_counts = []
        for _ in range(40):
            clock.advance(0.02)
            mono.advance(0.02)
            scan = runner.poll()
            if scan is None:
                finite_counts.append(0)
                continue
            finite_counts.append(
                sum(1 for r in scan['ranges'] if math.isfinite(r)))
        # 健康期内（frame_timeout=10s）仍返回 dict，但点必须全部过期。
        # 只有单调时钟推进过期判定，所以 clock.advance 之后点必须还在。
        assert finite_counts[-1] == 0, (
            '无新帧仍在重复发布旧点：%r' % (finite_counts[-5:],))
        assert finite_counts[0] > 0, 'ROS 时钟推进就触发过期，说明没用单调时钟'

    def test_default_expires_unvisited_angle_after_half_second(self, mono):
        clock = Clock()
        runner = make_runner(clock, mono, origin='raw', frame_timeout=10.0)
        runner.poll()
        serial = _Serial.instances[-1]
        _push_rotation(serial, runner, clock)
        assert math.isfinite(runner.poll()['ranges'][5])

        mono.advance(0.51)
        serial.push(build_frame(start_ddeg=9000, end_ddeg=10100))
        scan = runner.poll()
        assert math.isinf(scan['ranges'][5])
        assert math.isfinite(scan['ranges'][90])

    def test_parameter_file_expires_unvisited_angle_after_half_second(self, mono):
        yaml = pytest.importorskip('yaml')
        config_file = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                   'config', 'params.yaml')
        with open(config_file, encoding='utf-8') as source:
            params = yaml.safe_load(source)['lidar_node']['ros__parameters']
        clock = Clock()
        runner = make_runner(clock, mono, origin='raw', frame_timeout=10.0,
                             stale_point_age=params['stale_point_age'])
        runner.poll()
        serial = _Serial.instances[-1]
        _push_rotation(serial, runner, clock)
        assert math.isfinite(runner.poll()['ranges'][5])
        mono.advance(0.51)
        serial.push(build_frame(start_ddeg=9000, end_ddeg=10100))
        assert math.isinf(runner.poll()['ranges'][5])

    def test_launch_expires_unvisited_angle_after_half_second(
            self, mono, monkeypatch):
        launch = types.ModuleType('launch')
        launch.LaunchDescription = lambda actions: actions
        actions = types.ModuleType('launch.actions')
        actions.DeclareLaunchArgument = lambda *args, **kwargs: None
        substitutions = types.ModuleType('launch.substitutions')
        substitutions.LaunchConfiguration = lambda name: name
        launch_ros = types.ModuleType('launch_ros')
        ros_actions = types.ModuleType('launch_ros.actions')
        ros_actions.Node = lambda **kwargs: types.SimpleNamespace(**kwargs)
        for module in (launch, actions, substitutions, launch_ros, ros_actions):
            monkeypatch.setitem(sys.modules, module.__name__, module)

        launch_file = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                   'launch', 'robot.launch.py')
        nodes = runpy.run_path(launch_file)['generate_launch_description']()
        lidar = next(n for n in nodes if n and n.executable == 'lidar_node')
        clock = Clock()
        runner = make_runner(clock, mono, origin='raw', frame_timeout=10.0,
                             stale_point_age=lidar.parameters[0]['stale_point_age'])
        runner.poll()
        serial = _Serial.instances[-1]
        _push_rotation(serial, runner, clock)
        assert math.isfinite(runner.poll()['ranges'][5])
        mono.advance(0.51)
        serial.push(build_frame(start_ddeg=9000, end_ddeg=10100))
        assert math.isinf(runner.poll()['ranges'][5])


class TestZeroDistanceRefresh:
    def test_zero_distance_clears_old_bin_but_keeps_other_angles(self, mono):
        clock = Clock()
        runner = make_runner(clock, mono, origin='raw')
        runner.poll()
        serial = _Serial.instances[-1]
        _push_rotation(serial, runner, clock)
        assert runner.poll()['ranges'][5] == pytest.approx(1.5)

        points = [2000] * 12
        points[5] = 0
        serial.push(build_frame(start_ddeg=0, end_ddeg=1100, points=points))
        mono.advance(0.02)
        scan = runner.poll()
        assert math.isinf(scan['ranges'][5])
        assert scan['ranges'][6] == pytest.approx(2.0)
        assert scan['ranges'][180] == pytest.approx(1.5)

    @pytest.mark.parametrize('valid_index', [0, 11])
    def test_valid_and_zero_in_same_bin_keep_valid_point(self, mono, valid_index):
        clock = Clock()
        runner = make_runner(clock, mono, origin='raw')
        runner.poll()
        serial = _Serial.instances[-1]
        _push_rotation(serial, runner, clock)

        points = [0] * 12
        points[valid_index] = 1200
        serial.push(build_frame(start_ddeg=500, end_ddeg=555, points=points))
        mono.advance(0.02)
        assert runner.poll()['ranges'][5] == pytest.approx(1.2)

    def test_valid_point_survives_zeroes_in_same_bin_across_frames(self, mono):
        clock = Clock()
        runner = make_runner(clock, mono, origin='raw')
        runner.poll()
        serial = _Serial.instances[-1]
        _push_rotation(serial, runner, clock)

        serial.push(build_frame(start_ddeg=500, end_ddeg=555,
                                points=[0] * 11 + [1200]))
        mono.advance(0.02)
        assert runner.poll()['ranges'][5] == pytest.approx(1.2)
        serial.push(build_frame(start_ddeg=556, end_ddeg=599,
                                points=[0] * 12))
        mono.advance(0.02)
        assert runner.poll()['ranges'][5] == pytest.approx(1.2)

    def test_complete_rotation_keeps_all_directions_covered(self, mono):
        clock = Clock()
        runner = make_runner(clock, mono, origin='raw')
        runner.poll()
        _push_rotation(_Serial.instances[-1], runner, clock)
        ranges = runner.poll()['ranges']
        for center in (0, 90, 180, 270):
            count = sum(math.isfinite(ranges[(center + delta) % 360])
                        for delta in range(-15, 16))
            assert count >= 5, (center, count)

    @pytest.mark.parametrize(('origin', 'expected'), [('raw', 359),
                                                      ('mirror', 300)])
    def test_rotation_with_mixed_zeroes_keeps_valid_coverage(
            self, mono, origin, expected):
        clock = Clock()
        runner = make_runner(clock, mono, origin=origin)
        runner.poll()
        serial = _Serial.instances[-1]
        serial.push(b''.join(frame_rotation(
            step_deg=6.0, points=[1500, 0] * 6)))
        scan = runner.poll()
        # 镜像分箱会合并部分有效点；raw 末尾的零点会清除 0 度旧值。
        assert sum(math.isfinite(r) for r in scan['ranges']) == expected

    def test_reconnect_clears_old_points_and_accepts_new_scan(self, mono):
        clock = Clock()
        runner = make_runner(clock, mono, origin='raw')
        runner.poll()
        serial = _Serial.instances[-1]
        _push_rotation(serial, runner, clock)
        assert runner.poll()['ranges'][5] == pytest.approx(1.5)

        mono.advance(0.51)
        assert runner.poll() is None
        assert serial.closed
        mono.advance(1.0)
        assert runner.poll() is None
        serial = _Serial.instances[-1]
        serial.push(b''.join(frame_rotation(points=[0] * 12)))
        scan = runner.poll()
        assert all(math.isinf(r) for r in scan['ranges'])

        serial.push(build_frame(start_ddeg=500, end_ddeg=555,
                                points=[0] * 11 + [1200]))
        assert runner.poll()['ranges'][5] == pytest.approx(1.2)


class TestPerFrameAccounting:
    def test_three_frames_in_one_read(self, mono):
        """一次 read 解出 3 帧：frames_ok 加 3，覆盖率按 3 帧累计。"""
        clock = Clock()
        runner = make_runner(clock, mono, recovery_coverage=0.25)
        runner.poll()
        serial = _Serial.instances[-1]

        # 协议单帧跨度上限 40 度，所以用整圈 12 帧里等距的 3 帧：
        # 起点 0/120/240 度，各跨 30+ 度，覆盖互补，合计接近整圈
        frames = [build_frame(start_ddeg=d, end_ddeg=d + 3000,
                              points=[1500] * 12)
                  for d in (0, 12000, 24000)]
        serial.push(b''.join(frames))
        clock.advance(0.02)
        mono.advance(0.02)
        runner.poll()

        assert runner.monitor.frames_ok == 3
        # 3 帧各占 72 个 5 度槽位中的 7 个，合计 21/72 >= 0.25。
        # 只送最后一帧时只有 7/72，达不到阈值
        assert runner.monitor.coverage() == pytest.approx(21 / 72.0, abs=1e-6)
        assert runner.monitor.state == H.HEALTHY

    def test_last_frame_only_would_fail_expansion(self, mono):
        """回归：只送最后一帧时覆盖率只有 1/3，达不到 0.9。"""
        clock = Clock()
        runner = make_runner(clock, mono, recovery_coverage=0.9)
        runner.poll()
        serial = _Serial.instances[-1]
        third = [build_frame(start_ddeg=d, end_ddeg=d + 3000,
                             points=[1500] * 12)
                 for d in (0, 12000, 24000)]
        serial.push(third[2])
        clock.advance(0.02)
        mono.advance(0.02)
        runner.poll()
        assert runner.monitor.frames_ok == 1
        assert runner.monitor.coverage() < 0.9
        assert runner.monitor.state != H.HEALTHY

        # 补上另外两帧：三帧合计覆盖 21/72，仍低于 0.9，不得判 healthy
        serial.push(b''.join(third[0:2]))
        clock.advance(0.02)
        mono.advance(0.02)
        runner.poll()
        assert runner.monitor.frames_ok == 3
        assert runner.monitor.coverage() >= 21 / 72.0 - 1e-6
        assert runner.monitor.state == H.STALE

        # 再送整圈（12 帧一次 read），覆盖率达标后恢复 healthy
        serial.push(b''.join(frame_rotation()))
        clock.advance(0.02)
        mono.advance(0.02)
        runner.poll()
        assert runner.monitor.frames_ok == 15
        assert runner.monitor.coverage() >= 0.9
        assert runner.monitor.state == H.HEALTHY


class TestMonotonicClock:
    def test_ros_clock_rollback_does_not_affect_expiry(self, mono):
        """ROS 时钟回退后，过期判定仍按单调时钟推进。"""
        clock = Clock()
        runner = make_runner(clock, mono, stale_point_age=0.5,
                             frame_timeout=10.0)
        runner.poll()
        serial = _Serial.instances[-1]
        _push_rotation(serial, runner, clock)
        clock.advance(0.01)
        assert runner.poll() is not None

        # ROS 时钟推进 5 s，单调时钟只走 0.05 s：不得过期
        clock.advance(5.0)
        mono.advance(0.05)
        scan = runner.poll()
        assert scan is not None
        assert any(math.isfinite(r) for r in scan['ranges'])

        # ROS 时钟回退 100 s，单调时钟推进 0.6 s：必须过期
        clock.advance(-100.0)
        mono.advance(0.6)
        scan = runner.poll()
        assert scan is not None
        assert all(not math.isfinite(r) for r in scan['ranges']), (
            'ROS 时钟回退后过期判定错误：仍有 %d 个有限点'
            % sum(1 for r in scan['ranges'] if math.isfinite(r)))

    def test_ring_time_recorded_with_monotonic(self, mono):
        clock = Clock()
        runner = make_runner(clock, mono)
        runner.poll()
        serial = _Serial.instances[-1]
        serial.push(frame_rotation()[0])
        clock.advance(0.02)
        mono.advance(0.02)
        runner.poll()
        stamps = [t for t in runner.ring_time if t is not None]
        assert stamps
        assert all(t == pytest.approx(mono.seconds) for t in stamps)


class TestContract:
    def test_scan_fields_unchanged(self, mono):
        clock = Clock()
        runner = make_runner(clock, mono)
        runner.poll()
        _push_rotation(_Serial.instances[-1], runner, clock)
        clock.advance(0.1)
        mono.advance(0.1)
        scan = runner.poll()
        # max_point_age_s 是 2026-11-14 新增（原始帧证据用），其余字段不变
        assert set(scan) == {'ranges', 'angle_min', 'angle_max',
                             'angle_increment', 'range_min', 'range_max',
                             'max_point_age_s'}
        # 默认 origin='mirror'，角度约定不变
        assert scan['angle_min'] == pytest.approx(0.0)
        assert scan['range_min'] == pytest.approx(0.02)
        assert scan['range_max'] == pytest.approx(12.0)
        assert len(scan['ranges']) == 360
        # 超出量程仍按无效处理
        assert all(not math.isfinite(r) or r <= 12.0 for r in scan['ranges'])

    def test_unhealthy_returns_none(self, mono):
        clock = Clock()
        runner = make_runner(clock, mono)
        assert runner.poll() is None
        clock.advance(0.02)
        mono.advance(0.02)
        assert runner.poll() is None
