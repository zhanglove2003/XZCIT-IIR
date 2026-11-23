"""网页端新鲜度逻辑测试。

覆盖：断线后不显示旧帧为实时、诊断解析、雷达可信性组合判定。
"""

import pytest

import fake_ros
from opi_robot.web_node import sensors as S


def make_diag(state='healthy', level=S.LEVEL_OK, coverage='1.0',
              age='0.01', reconnects='0'):
    return fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(
            level=level, name='lidar', message=state,
            values=[
                fake_ros.KeyValue(key='state', value=state),
                fake_ros.KeyValue(key='reason', value='ok'),
                fake_ros.KeyValue(key='age_s', value=age),
                fake_ros.KeyValue(key='coverage', value=coverage),
                fake_ros.KeyValue(key='reconnects', value=reconnects),
                fake_ros.KeyValue(key='port', value='/dev/serial/by-path/x'),
                fake_ros.KeyValue(key='frames_ok', value='100'),
                fake_ros.KeyValue(key='frames_bad', value='2'),
            ])
    ])


class TestFreshness:
    def test_never_seen(self):
        assert S.freshness(None, 10.0, 1.0) == S.UNKNOWN

    def test_fresh(self):
        assert S.freshness(9.8, 10.0, 1.0) == S.FRESH

    def test_stale_after_timeout(self):
        assert S.freshness(8.0, 10.0, 1.0, ever_seen=True) == S.STALE

    def test_stale_never_returns_fresh(self):
        for dt in (1.0, 2.0, 100.0):
            assert S.freshness(10.0 - dt, 10.0, 0.5, ever_seen=True) == S.STALE

    def test_clock_backwards_not_fresh(self):
        assert S.freshness(11.0, 10.0, 1.0) == S.STALE


class TestParseDiagnostic:
    def test_none_message(self):
        assert S.parse_diagnostic(None) is None

    def test_no_lidar_status(self):
        msg = fake_ros.DiagnosticArray(status=[
            fake_ros.DiagnosticStatus.make(level=0, name='other')
        ])
        assert S.parse_diagnostic(msg) is None

    def test_fields(self):
        d = S.parse_diagnostic(make_diag(state='stale', level=S.LEVEL_ERROR,
                                         coverage='0.5', reconnects='3'))
        assert d['state'] == 'stale'
        assert d['level'] == S.LEVEL_ERROR
        assert d['coverage'] == 0.5
        assert d['reconnects'] == 3
        assert d['frames_ok'] == 100

    def test_bad_numbers_do_not_crash(self):
        d = S.parse_diagnostic(make_diag(age='', coverage='n/a'))
        assert d['age_s'] is None
        assert d['coverage'] is None


class TestLevelMapping:
    def test_ok(self):
        assert S.level_to_state(S.LEVEL_OK) == 'ok'

    def test_warn(self):
        assert S.level_to_state(S.LEVEL_WARN) == 'warn'

    def test_error_and_stale(self):
        assert S.level_to_state(S.LEVEL_ERROR) == 'error'
        assert S.level_to_state(S.LEVEL_STALE) == 'error'


class TestSensorView:
    def test_all_unknown_at_start(self):
        v = S.sensor_view(None, None, None, None, 10.0)
        assert v['lidar_diag'] == S.UNKNOWN
        assert v['scan'] == S.UNKNOWN
        assert v['image'] == S.UNKNOWN
        assert v['lidar_trusted'] is False

    def test_all_fresh(self):
        v = S.sensor_view(S.parse_diagnostic(make_diag()), 9.9, 9.9, 9.9, 10.0)
        assert v['lidar_diag'] == 'ok'
        assert v['scan'] == S.FRESH
        assert v['image'] == S.FRESH
        assert v['lidar_trusted'] is True

    def test_old_frame_not_shown_as_live(self):
        v = S.sensor_view(S.parse_diagnostic(make_diag()), 9.9, 1.0, 9.9, 10.0)
        assert v['scan'] == S.STALE
        assert v['lidar_trusted'] is False
        assert v['lidar_age'] == pytest.approx(9.0, abs=1e-6)

    def test_image_expires_independently(self):
        v = S.sensor_view(S.parse_diagnostic(make_diag()), 9.9, 9.9, 5.0, 10.0)
        assert v['image'] == S.STALE
        assert v['scan'] == S.FRESH

    def test_unhealthy_diagnostic_downgrades_scan(self):
        v = S.sensor_view(
            S.parse_diagnostic(make_diag(state='disconnected',
                                         level=S.LEVEL_ERROR)),
            9.9, 9.9, 9.9, 10.0)
        assert v['lidar_diag'] == 'error'
        assert v['scan'] == S.STALE
        assert v['lidar_trusted'] is False

    def test_stale_diagnostics_downgrades_scan(self):
        v = S.sensor_view(S.parse_diagnostic(make_diag()), 1.0, 9.9, 9.9, 10.0)
        assert v['lidar_diag'] == S.STALE
        assert v['lidar_trusted'] is False

    def test_json_safe(self):
        import json
        v = S.sensor_view(S.parse_diagnostic(make_diag()), 9.9, 9.9, 9.9, 10.0)
        json.dumps(v, allow_nan=False)


class TestDescribe:
    def test_known_states(self):
        assert S.describe_state(S.FRESH) == '实时'
        assert S.describe_state(S.STALE) == '已过期'
        assert S.describe_state(S.UNKNOWN) == '无数据'

    def test_unknown_state_passthrough(self):
        assert S.describe_state('weird') == 'weird'
