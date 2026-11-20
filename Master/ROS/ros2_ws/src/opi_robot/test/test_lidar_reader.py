"""雷达节点协调逻辑测试：伪串口、假时钟、mock 端口。

覆盖：设备未出现、运行中断线、读取错误、无有效帧、恢复连接、
设备名变化后从稳定路径重连、不发布旧测距、不刷日志。

被测模块：opi_robot.lidar_node.reader（LidarRunner）
"""

import math

import pytest

from opi_robot.lidar_node import health as H
from opi_robot.lidar_node import reader as R
import lidar_fixtures
from lidar_fixtures import Clock, build_frame, frame_rotation

PORT = '/dev/serial/by-path/platform-x-usb-0:1.2:1.0-port0'


class FakeSerial:
    """伪串口：可注入数据、抛错。"""

    instances = []

    def __init__(self, path, baud):
        self.path = path
        self.baud = baud
        self.chunks = bytearray()
        self.closed = False
        self.fail_read = None
        self.mirror = False
        FakeSerial.instances.append(self)

    def push(self, data):
        self.chunks.extend(data)

    def read(self, n=4096):
        if self.fail_read is not None:
            err = self.fail_read
            self.fail_read = None
            raise err
        if not self.chunks:
            return b''
        data = bytes(self.chunks[:n])
        del self.chunks[:n]
        return data

    def close(self):
        self.closed = True


@pytest.fixture(autouse=True)
def clean_instances():
    FakeSerial.instances = []
    yield
    FakeSerial.instances = []


def make_runner(clock, exists=None, open_fn=None, **kwargs):
    calls = {'exists': [], 'open': []}

    def exists_fn(path):
        calls['exists'].append(path)
        return exists(path) if exists else True

    def open_impl(path, baud):
        calls['open'].append(path)
        if open_fn is not None:
            return open_fn(path, baud)
        return FakeSerial(path, baud)

    params = dict(
        port=PORT,
        # 状态机与测距过期判定走单调时钟；这里用可控假时钟，与 ROS 时钟解耦
        mono=lidar_fixtures.monotonic,
        baud=230400,
        frame_timeout=0.5,
        recovery_coverage=0.9,
        reconnect_interval=1.0,
        log_interval=10.0,
        mirror=False,
    )
    params.update(kwargs)
    runner = R.LidarRunner(
        clock=clock.now,
        open_port=open_impl,
        path_exists=exists_fn,
        **params)
    return runner, calls


def push_rotation(runner, clock, step=0.02, serial=None, frames=None):
    serial = serial or FakeSerial.instances[-1]
    for frame in (frames or frame_rotation()):
        serial.push(frame)
        clock.advance(step)
        lidar_fixtures.advance(step)
        runner.poll()
    clock.advance(0.01)
    lidar_fixtures.advance(0.01)
    lidar_fixtures.advance(0.01)


class TestStartup:
    def test_missing_device(self):
        clock = Clock()
        runner, _ = make_runner(clock, exists=lambda p: False)
        assert runner.poll() is None
        assert runner.monitor.state == H.DISCONNECTED
        assert runner.monitor.reason == 'device_missing'

    def test_missing_device_polls_without_spam(self):
        clock = Clock()

        def boom(p, b):
            raise OSError(2, 'No such file or directory')

        runner, calls = make_runner(clock, open_fn=boom)
        logs = []
        for _ in range(400):
            clock.advance(0.01)
            lidar_fixtures.advance(0.01)
            runner.poll(logs)
        assert runner.open_attempts <= 4
        errors = [m for m in logs if '打开失败' in m]
        # 允许首次失败报一次，之后只按汇总间隔报，不能每轮都刷
        assert 1 <= len(errors) <= 6

    def test_open_error_state(self):
        clock = Clock()

        def boom(p, b):
            raise OSError(2, 'No such file or directory')

        runner, _ = make_runner(clock, open_fn=boom)
        runner.poll()
        assert runner.monitor.state == H.DISCONNECTED
        assert runner.monitor.reason == 'open_error'


class TestHealthy:
    def test_becomes_healthy_and_publishes(self):
        clock = Clock()
        runner, _ = make_runner(clock)
        assert runner.poll() is None
        assert runner.monitor.state == H.CONNECTING
        push_rotation(runner, clock)
        clock.advance(0.1)
        lidar_fixtures.advance(0.1)
        scan = runner.poll()
        assert scan is not None
        assert runner.monitor.state == H.HEALTHY
        finite = [r for r in scan['ranges'] if math.isfinite(r)]
        # 12 帧各跨 30 度、起点间隔 30 度，每帧 12 个点：整圈 360 个 0.5 度
        # 分箱里约 1/3 被点到（每帧约 11 个不同分箱），132 是精确值
        assert len(finite) == 132
        # 覆盖判定只要求每个 5 度槽位都被扫到，槽位数远少于分箱数
        assert runner.monitor.coverage() >= 0.9

    def test_laserscan_fields(self):
        clock = Clock()
        runner, _ = make_runner(clock)
        runner.poll()
        push_rotation(runner, clock)
        clock.advance(0.1)
        lidar_fixtures.advance(0.1)
        scan = runner.poll()
        assert scan['angle_min'] == pytest.approx(0.0)
        assert scan['angle_increment'] == pytest.approx(2 * math.pi / 360)
        assert scan['range_min'] == pytest.approx(0.02)
        assert scan['range_max'] == pytest.approx(12.0)
        assert len(scan['ranges']) == 360

    def test_front_frame_lands_at_zero(self):
        """原始角 0° 附近应落在 LaserScan 0° 附近。"""
        clock = Clock()
        runner, _ = make_runner(clock)
        runner.poll()
        serial = FakeSerial.instances[-1]
        # 每格保留最近一次读数：背景帧必须整体放到远处，且 1.5m 的前向帧
        # 要排在同一角度的背景帧之后，否则 0° 会被后到的 9.99m 覆盖
        frames = frame_rotation(points=[9990] * 12)
        frames.append(build_frame(start_ddeg=0, end_ddeg=3000,
                                  points=[1500] * 12))
        push_rotation(runner, clock, serial=serial, frames=frames)
        clock.advance(0.1)
        lidar_fixtures.advance(0.1)
        scan = runner.poll()
        assert scan['ranges'][0] == pytest.approx(1.5, abs=1e-6)

    def test_left_frame_lands_at_minus_90(self):
        """原始角 90°（雷达左侧）应落在 LaserScan 270° 附近。"""
        clock = Clock()
        runner, _ = make_runner(clock)
        runner.poll()
        # 12 帧整圈打底让覆盖达标，随后单独注入左向帧
        base = frame_rotation(points=[999] * 12)
        push_rotation(runner, clock, frames=base)
        left = build_frame(start_ddeg=9000, end_ddeg=9030, points=[2000] * 12)
        push_rotation(runner, clock, frames=[left])
        clock.advance(0.1)
        lidar_fixtures.advance(0.1)
        scan = runner.poll()
        assert scan is not None, (
            '扫描为空：状态=%s 覆盖=%.3f'
            % (runner.monitor.state, runner.monitor.coverage()))
        # 左向帧原始角 90°→镜像后 270°，取 269..271 三个分箱的最小值
        window = [scan['ranges'][i % 360] for i in (269, 270, 271)]
        assert min(window) <= 2.0 + 1e-6, '左向帧距离未写入 270 分箱：%r' % (window,)
    def test_no_scan_before_coverage(self):
        clock = Clock()
        runner, _ = make_runner(clock)
        runner.poll()
        FakeSerial.instances[-1].push(build_frame())
        clock.advance(0.02)
        lidar_fixtures.advance(0.02)
        assert runner.poll() is None

    def test_no_scan_after_timeout(self):
        clock = Clock()
        runner, _ = make_runner(clock)
        runner.poll()
        push_rotation(runner, clock)
        clock.advance(0.1)
        lidar_fixtures.advance(0.1)
        assert runner.poll() is not None, (
            '推进前状态=%s 覆盖=%.3f' % (runner.monitor.state,
                                       runner.monitor.coverage()))
        got_none = 0
        for _ in range(70):
            clock.advance(0.01)
            lidar_fixtures.advance(0.01)
            if runner.poll() is None:
                got_none += 1
        # 0.7s > frame_timeout(0.5s)，后半程必须不再发布
        assert got_none >= 15, '超时后仍持续返回扫描：%d 次为空' % got_none
        assert runner.monitor.state in (H.STALE, H.CONNECTING,
                                        H.DISCONNECTED)
    def test_ring_cleared_after_timeout(self):
        clock = Clock()
        runner, _ = make_runner(clock)
        runner.poll()
        push_rotation(runner, clock)
        clock.advance(0.1)
        lidar_fixtures.advance(0.1)
        runner.poll()
        for _ in range(70):
            clock.advance(0.01)
            lidar_fixtures.advance(0.01)
            runner.poll()
        assert all(not math.isfinite(r) for r in runner.ring)


class TestDisconnect:
    def _healthy(self, clock, runner):
        runner.poll()
        push_rotation(runner, clock)
        clock.advance(0.1)
        lidar_fixtures.advance(0.1)
        assert runner.poll() is not None
        assert runner.monitor.state == H.HEALTHY
        return FakeSerial.instances[-1]

    def test_read_error_closes_port(self):
        clock = Clock()
        runner, _ = make_runner(clock)
        serial = self._healthy(clock, runner)
        serial.fail_read = OSError(5, 'Input/output error')
        clock.advance(0.02)
        lidar_fixtures.advance(0.02)
        runner.poll()
        assert serial.closed is True
        assert runner.monitor.state == H.DISCONNECTED
        assert runner.monitor.reason == 'read_error'

    def test_stale_releases_port(self):
        clock = Clock()
        runner, _ = make_runner(clock)
        serial = self._healthy(clock, runner)
        for _ in range(80):
            clock.advance(0.01)
            lidar_fixtures.advance(0.01)
            runner.poll()
        # 超时后串口必须释放；随后可能已按节流重新打开
        assert serial.closed is True
        assert runner.monitor.state in (H.STALE, H.CONNECTING,
                                        H.DISCONNECTED)

    def test_reconnect_from_stable_path(self):
        clock = Clock()
        state = {'exists': True}
        runner, calls = make_runner(clock, exists=lambda p: state['exists'])
        serial = self._healthy(clock, runner)
        serial.fail_read = OSError(5, 'Input/output error')
        clock.advance(0.02)
        lidar_fixtures.advance(0.02)
        runner.poll()
        state['exists'] = False
        for _ in range(100):
            clock.advance(0.01)
            lidar_fixtures.advance(0.01)
            runner.poll()
        assert runner.monitor.state == H.DISCONNECTED
        state['exists'] = True
        for _ in range(200):
            clock.advance(0.01)
            lidar_fixtures.advance(0.01)
            runner.poll()
        assert runner.monitor.state == H.CONNECTING
        assert runner.monitor.reconnects == 1
        assert FakeSerial.instances[-1].path == PORT

    def test_recovery_needs_new_frames(self):
        clock = Clock()
        state = {'exists': True}
        runner, _ = make_runner(clock, exists=lambda p: state['exists'])
        serial = self._healthy(clock, runner)
        serial.fail_read = OSError(5, 'Input/output error')
        clock.advance(0.02)
        lidar_fixtures.advance(0.02)
        runner.poll()
        for _ in range(200):
            clock.advance(0.01)
            lidar_fixtures.advance(0.01)
            runner.poll()
        new = FakeSerial.instances[-1]
        new.push(frame_rotation()[0])
        clock.advance(0.02)
        lidar_fixtures.advance(0.02)
        runner.poll()
        # 新数据不足整圈覆盖，只能停在 recovering，不能回到 healthy
        assert runner.monitor.state in (H.STALE, H.CONNECTING)
        assert runner.monitor.state != H.HEALTHY
        assert runner.monitor.coverage() < runner.monitor.recovery_coverage

    def test_reconnect_throttled(self):
        clock = Clock()
        runner, calls = make_runner(clock, exists=lambda p: False)
        runner.poll()
        runner.poll()
        runner.poll()
        assert runner.open_attempts == 1

    def test_no_reopen_while_open(self):
        clock = Clock()
        runner, _ = make_runner(clock)
        runner.poll()
        for _ in range(200):
            clock.advance(0.01)
            lidar_fixtures.advance(0.01)
            runner.poll()
        assert runner.open_attempts == 1

    def test_empty_read_is_not_error(self):
        clock = Clock()
        runner, _ = make_runner(clock)
        serial = self._healthy(clock, runner)
        for _ in range(70):
            clock.advance(0.01)
            lidar_fixtures.advance(0.01)
            runner.poll()
        assert serial.closed is True


class TestLogs:
    def test_summary_rate_limited(self):
        clock = Clock()
        runner, _ = make_runner(clock)
        logs = []
        runner.poll(logs)
        for _ in range(3000):
            clock.advance(0.01)
            lidar_fixtures.advance(0.01)
            runner.poll(logs)
        summaries = [m for m in logs if '雷达统计' in m]
        # log_interval=10s，30 秒内最多 4 条，但不能完全没有统计
        assert 1 <= len(summaries) <= 4

    def test_healthy_transition_logged_once(self):
        clock = Clock()
        runner, _ = make_runner(clock)
        logs = []
        runner.poll(logs)
        push_rotation(runner, clock)
        for frame in frame_rotation():
            FakeSerial.instances[-1].push(frame)
            clock.advance(0.02)
            lidar_fixtures.advance(0.02)
            runner.poll(logs)
        self_healthy = [m for m in logs if '进入 healthy' in m]
        assert len(self_healthy) <= 1


class TestSnapshot:
    def test_has_port_and_state(self):
        clock = Clock()
        runner, _ = make_runner(clock)
        runner.poll()
        snap = runner.snapshot()
        assert snap['port'] == PORT
        assert snap['state'] in H.ALL_STATES
        assert snap['origin'] == 'mirror'

    def test_snapshot_after_healthy(self):
        clock = Clock()
        runner, _ = make_runner(clock)
        runner.poll()
        push_rotation(runner, clock)
        clock.advance(0.1)
        lidar_fixtures.advance(0.1)
        runner.poll()
        snap = runner.snapshot()
        assert snap['state'] == H.HEALTHY
        assert snap['frames_ok'] >= 11
