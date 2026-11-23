"""状态机与重连节流测试。

被测模块：opi_robot.lidar_node.health
"""

import pytest

from opi_robot.lidar_node import health as H
from lidar_fixtures import Clock

# 每段 30 度，起点的扫描角与宽度（on_scan 参数）。
FULL_SCANS = [(i * 30.0, 30.0) for i in range(12)]


def feed_full(mon, now):
    for a, width in FULL_SCANS:
        mon.on_scan(now, a, width)


class TestStates:
    def test_state_names(self):
        assert H.CONNECTING == 'connecting'
        assert H.HEALTHY == 'healthy'
        assert H.STALE == 'stale'
        assert H.DISCONNECTED == 'disconnected'
        assert set(H.ALL_STATES) == {'connecting', 'healthy', 'stale',
                                     'disconnected'}

    def test_initial_state(self):
        mon = H.HealthMonitor(frame_timeout=0.5, recovery_coverage=0.9)
        assert mon.state == H.CONNECTING
        assert mon.reason == 'startup'

    def test_age_infinite_without_frame(self):
        mon = H.HealthMonitor(frame_timeout=0.5)
        assert mon.age(10.0) == float('inf')

    def test_not_healthy_before_first_scan(self):
        mon = H.HealthMonitor(frame_timeout=0.5)
        assert mon.is_healthy(10.0) is False


class TestCoverageGate:
    def test_single_scan_not_healthy(self):
        mon = H.HealthMonitor(frame_timeout=0.5, recovery_coverage=0.9)
        mon.on_open_success(1.0)
        mon.on_scan(1.1, 0.0, 30.0)
        assert mon.state != H.HEALTHY
        assert mon.coverage() < 0.9

    def test_full_rotation_becomes_healthy(self):
        mon = H.HealthMonitor(frame_timeout=0.5, recovery_coverage=0.9)
        mon.on_open_success(1.0)
        feed_full(mon, 5.0)
        assert mon.state == H.HEALTHY
        assert mon.reason == 'ok'
        assert mon.coverage() == pytest.approx(1.0)

    def test_gap_blocks_healthy(self):
        mon = H.HealthMonitor(frame_timeout=0.5, recovery_coverage=0.9)
        mon.on_open_success(1.0)
        for i in range(12):
            if 3 <= i < 6:
                continue
            mon.on_scan(5.0, i * 30.0 + 30.0, 30.0)
        assert mon.coverage() < 0.9
        assert mon.state != H.HEALTHY

    def test_recovery_needs_new_coverage(self):
        """断线后旧覆盖作废，恢复要重新积累。"""
        mon = H.HealthMonitor(frame_timeout=0.5, recovery_coverage=0.9)
        mon.on_open_success(1.0)
        feed_full(mon, 5.0)
        assert mon.state == H.HEALTHY
        mon.on_frame_timeout(5.6)
        assert mon.state == H.STALE
        assert mon.coverage() == pytest.approx(0.0)
        mon.on_open_success(6.0)
        assert mon.state == H.CONNECTING
        mon.on_scan(6.1, 0.0, 30.0)
        assert mon.state == H.STALE
        assert mon.reason == 'recovering'
        feed_full(mon, 6.2)
        assert mon.state == H.HEALTHY


class TestTimeout:
    def test_tick_marks_stale(self):
        mon = H.HealthMonitor(frame_timeout=0.5, recovery_coverage=0.9)
        mon.on_open_success(1.0)
        feed_full(mon, 2.0)
        assert mon.state == H.HEALTHY
        mon.tick(2.6)
        assert mon.state == H.STALE
        assert mon.reason == 'frame_timeout'

    def test_is_healthy_false_after_timeout(self):
        mon = H.HealthMonitor(frame_timeout=0.5, recovery_coverage=0.9)
        mon.on_open_success(1.0)
        feed_full(mon, 2.0)
        assert mon.is_healthy(2.1) is True
        assert mon.is_healthy(2.6) is False

    def test_tick_does_not_flip_connecting(self):
        mon = H.HealthMonitor(frame_timeout=0.5, recovery_coverage=0.9)
        mon.on_open_success(1.0)
        mon.tick(1.4)
        assert mon.state == H.CONNECTING


class TestFailures:
    def test_open_failure(self):
        mon = H.HealthMonitor(frame_timeout=0.5)
        mon.on_open_failure(1.0, 'ENOENT')
        assert mon.state == H.DISCONNECTED
        assert mon.reason == 'open_error'
        assert mon.last_error == 'ENOENT'

    def test_read_error(self):
        mon = H.HealthMonitor(frame_timeout=0.5)
        mon.on_open_success(1.0)
        mon.on_read_error(2.0, 'EIO')
        assert mon.state == H.DISCONNECTED
        assert mon.reason == 'read_error'

    def test_device_missing(self):
        mon = H.HealthMonitor(frame_timeout=0.5)
        mon.on_device_missing(1.0)
        assert mon.state == H.DISCONNECTED
        assert mon.reason == 'device_missing'

    def test_failure_clears_last_frame_time(self):
        mon = H.HealthMonitor(frame_timeout=0.5, recovery_coverage=0.9)
        mon.on_open_success(1.0)
        feed_full(mon, 2.0)
        mon.on_read_error(3.0, 'EIO')
        assert mon.age(3.1) == float('inf')
        assert mon.snapshot(3.1)['age_s'] is None


class TestCounters:
    def test_good_bad_and_dropped(self):
        mon = H.HealthMonitor(frame_timeout=0.5)
        mon.on_bad_frame(1.0)
        mon.on_bad_frame(1.1)
        mon.on_dropped_bytes(1.2, 5)
        assert mon.frames_bad == 2
        assert mon.bytes_dropped == 5

    def test_first_open_is_not_a_reconnect(self):
        mon = H.HealthMonitor(frame_timeout=0.5)
        mon.on_open_success(1.0)
        assert mon.reconnects == 0

    def test_reconnect_count_after_disconnect(self):
        mon = H.HealthMonitor(frame_timeout=0.5)
        mon.on_open_success(1.0)
        mon.on_read_error(2.0, 'EIO')
        mon.on_open_failure(2.5, 'ENOENT')
        mon.on_open_success(3.0)
        assert mon.reconnects == 1


class TestThrottle:
    def test_first_attempt_allowed(self):
        th = H.ReconnectThrottle(1.0)
        assert th.allow(100.0) is True

    def test_rate_limited(self):
        th = H.ReconnectThrottle(1.0)
        th.record(100.0)
        assert th.allow(100.5) is False
        assert th.allow(101.0) is True

    def test_wait_remaining(self):
        th = H.ReconnectThrottle(2.0)
        th.record(10.0)
        assert th.wait(10.0) == pytest.approx(2.0)
        assert th.wait(11.5) == pytest.approx(0.5)
        assert th.wait(12.0) == pytest.approx(0.0)

    def test_reset(self):
        th = H.ReconnectThrottle(5.0)
        th.record(10.0)
        th.reset()
        assert th.allow(10.1) is True


class TestLogLimiter:
    def test_same_reason_once(self):
        lim = H.LogLimiter()
        assert lim.should_log('read_error:EIO') is True
        assert lim.should_log('read_error:EIO') is False
        assert lim.should_log('open_error:ENOENT') is True
        assert lim.should_log('read_error:EIO') is True

    def test_summary_interval(self):
        lim = H.LogLimiter()
        assert lim.should_summary(10.0, 10.0) is True
        assert lim.should_summary(15.0, 10.0) is False
        assert lim.should_summary(20.0, 10.0) is True


class TestSnapshot:
    def test_fields(self):
        mon = H.HealthMonitor(frame_timeout=0.5)
        mon.on_open_success(1.0)
        snap = mon.snapshot(1.2, port='/dev/serial/by-path/x')
        required = {'state', 'reason', 'age_s', 'frames_ok', 'frames_bad',
                    'reconnects', 'coverage', 'last_error', 'port',
                    'uptime_s'}
        assert required <= set(snap)
        assert snap['age_s'] is None
        assert snap['uptime_s'] == pytest.approx(0.2, abs=1e-6)

    def test_age_after_scan(self):
        mon = H.HealthMonitor(frame_timeout=0.5, recovery_coverage=0.9)
        mon.on_open_success(1.0)
        feed_full(mon, 1.5)
        assert mon.snapshot(2.0, 'p')['age_s'] == pytest.approx(0.5)


class TestDiagnosticLevel:
    def test_levels(self):
        assert H.diagnostic_level(H.HEALTHY) == 0
        assert H.diagnostic_level(H.CONNECTING) == 1
        assert H.diagnostic_level(H.STALE) == 2
        assert H.diagnostic_level(H.DISCONNECTED) == 2


def test_clock_helper_advances():
    c = Clock()
    t0 = c.seconds
    c.advance(1.5)
    assert c.seconds == pytest.approx(t0 + 1.5)
