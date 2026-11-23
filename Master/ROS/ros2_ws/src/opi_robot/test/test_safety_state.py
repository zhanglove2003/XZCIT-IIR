"""safety_state 纯逻辑测试：原始帧证据追踪与许可判定（假时钟）。"""

import json

import pytest

from opi_robot.fusion_node import safety_state as SS


def ev(epoch=7, count=100, max_age=None):
    return {'epoch': epoch, 'frames_ok': count, 'max_point_age_ms': max_age}


def evidence_msg(epoch=7, count=100, max_age=None):
    payload = {'v': SS.RAW_EVIDENCE_SCHEMA, 'epoch': epoch,
               'frames_ok': count, 'max_point_age_ms': max_age}
    return json.dumps(payload, separators=(',', ':'))


class TestRawEvidenceTracker:
    def test_no_evidence_is_not_flowing(self):
        t = SS.RawEvidenceTracker(0.5)
        st = t.status(10.0)
        assert st['flowing'] is False
        assert st['reason'] == 'no_evidence'

    def test_first_message_only_establishes_baseline(self):
        t = SS.RawEvidenceTracker(0.5)
        t.feed(**ev(), now=10.0)
        st = t.status(10.05)
        assert st['flowing'] is False
        assert st['reason'] == 'evidence_no_increase'

    def test_increase_proves_flow(self):
        t = SS.RawEvidenceTracker(0.5)
        t.feed(**ev(count=100), now=10.0)
        t.feed(**ev(count=105), now=10.1)
        st = t.status(10.2)
        assert st['flowing'] is True
        assert st['reason'] == 'ok'
        assert st['increases'] == 1

    def test_stalled_counter_within_grace_is_flowing(self):
        """缓存圈重发期：消息继续、计数冻结，时限内仍算流动。"""
        t = SS.RawEvidenceTracker(0.5)
        t.feed(**ev(count=100), now=10.0)
        t.feed(**ev(count=105), now=10.1)
        t.feed(**ev(count=105), now=10.4)
        st = t.status(10.55)
        assert st['flowing'] is True
        assert st['idle_s'] == pytest.approx(0.45)

    def test_stalled_counter_past_deadline_denies(self):
        """原始帧停止后，超过时限必须判失效（T1 必测项 1 的核心）。"""
        t = SS.RawEvidenceTracker(0.5)
        t.feed(**ev(count=100), now=10.0)
        t.feed(**ev(count=105), now=10.1)
        # 原始帧停止；证据消息（缓存圈）继续到达，计数不变
        t.feed(**ev(count=105), now=10.55)
        st = t.status(10.61)
        assert st['flowing'] is False
        assert st['reason'] == 'raw_frames_stalled'

    def test_evidence_stream_stops_denies(self):
        t = SS.RawEvidenceTracker(0.5)
        t.feed(**ev(count=100), now=10.0)
        t.feed(**ev(count=105), now=10.1)
        st = t.status(10.1 + 0.5 + 0.01)
        assert st['flowing'] is False
        assert st['reason'] == 'evidence_stale'

    def test_epoch_change_requires_rebuild(self):
        """节点重启（epoch 变更）：旧增长历史作废，重建后才放行。"""
        t = SS.RawEvidenceTracker(0.5)
        t.feed(**ev(epoch=7, count=5000), now=10.0)
        t.feed(**ev(epoch=7, count=5005), now=10.1)
        assert t.status(10.2)['flowing'] is True
        # lidar 节点重启：frames_ok 从 0 重新计数
        t.feed(**ev(epoch=8, count=0), now=11.0)
        st = t.status(11.05)
        assert st['flowing'] is False
        assert st['reason'] == 'evidence_no_increase'
        assert st['epoch_changes'] == 1
        t.feed(**ev(epoch=8, count=3), now=11.1)
        assert t.status(11.2)['flowing'] is True

    def test_count_regression_requires_rebuild(self):
        """同 epoch 计数回退：不能当新数据，必须重新建立。"""
        t = SS.RawEvidenceTracker(0.5)
        t.feed(**ev(count=100), now=10.0)
        t.feed(**ev(count=150), now=10.1)
        assert t.status(10.2)['flowing'] is True
        t.feed(**ev(count=140), now=10.3)
        st = t.status(10.4)
        assert st['flowing'] is False
        assert st['regressions'] == 1
        # 以回退后的值为基线，重新增长才恢复
        t.feed(**ev(count=141), now=10.5)
        assert t.status(10.6)['flowing'] is True

    def test_restart_counts_below_old_value_stay_denied(self):
        """重启后 frames_ok 长期低于旧值：持续拒绝（fail-closed）。"""
        t = SS.RawEvidenceTracker(0.5)
        t.feed(**ev(epoch=7, count=10000), now=10.0)
        t.feed(**ev(epoch=7, count=10010), now=10.1)
        # 新 epoch 计数从 0 爬升，但 100 ms 内不会超过 10010
        t.feed(**ev(epoch=8, count=0), now=11.0)
        t.feed(**ev(epoch=8, count=46), now=11.1)
        st = t.status(11.2)
        # epoch 8 内已有 1 次增长 → 放行交由上层（诊断/恢复门槛）兜底
        assert st['flowing'] is True
        assert st['epoch'] == 8

    def test_feed_rejects_bad_types(self):
        t = SS.RawEvidenceTracker(0.5)
        with pytest.raises(ValueError):
            t.feed(epoch=True, frames_ok=1, now=0.0)
        with pytest.raises(ValueError):
            t.feed(epoch=1, frames_ok=-1, now=0.0)
        with pytest.raises(ValueError):
            t.feed(epoch=1.5, frames_ok=1, now=0.0)

    def test_invalid_timeout_rejected(self):
        with pytest.raises(ValueError):
            SS.RawEvidenceTracker(0)
        with pytest.raises(ValueError):
            SS.RawEvidenceTracker(-1)


class TestParseEvidenceJson:
    def test_valid(self):
        parsed = SS.parse_evidence_json(evidence_msg(7, 12345, 120))
        assert parsed == {'epoch': 7, 'frames_ok': 12345,
                          'max_point_age_ms': 120}

    def test_null_max_age_allowed(self):
        parsed = SS.parse_evidence_json(evidence_msg(7, 1, None))
        assert parsed['max_point_age_ms'] is None

    def test_rejects_wrong_schema(self):
        assert SS.parse_evidence_json(evidence_msg().replace(
            '"v":1', '"v":2')) is None

    def test_rejects_malformed_json(self):
        assert SS.parse_evidence_json('{not json') is None

    def test_rejects_non_object(self):
        assert SS.parse_evidence_json('[1,2,3]') is None

    def test_rejects_bool_as_int(self):
        payload = {'v': 1, 'epoch': True, 'frames_ok': 5,
                   'max_point_age_ms': None}
        assert SS.parse_evidence_json(json.dumps(payload)) is None

    def test_rejects_negative_and_missing(self):
        payload = {'v': 1, 'epoch': -1, 'frames_ok': 5, 'max_point_age_ms': 0}
        assert SS.parse_evidence_json(json.dumps(payload)) is None
        payload = {'v': 1, 'epoch': 1, 'max_point_age_ms': 0}
        assert SS.parse_evidence_json(json.dumps(payload)) is None

    def test_rejects_non_string_input(self):
        assert SS.parse_evidence_json(None) is None
        assert SS.parse_evidence_json(12345) is None


class TestPermission:
    def test_evidence_failure_wins(self):
        allowed, reason = SS.permission(
            False, 'raw_frames_stalled', False, 'scan_stale', 'stop', '')
        assert allowed is False
        assert reason == 'raw_frames_stalled'

    def test_trusted_failure_second(self):
        allowed, reason = SS.permission(
            True, 'ok', False, 'diagnostics_stale', 'stop', '')
        assert allowed is False
        assert reason == 'diagnostics_stale'

    def test_obstacle_stop(self):
        allowed, reason = SS.permission(
            True, 'ok', True, 'ok', 'stop', '')
        assert allowed is False
        assert reason == 'obstacle_stop'

    def test_unknown_coverage(self):
        allowed, reason = SS.permission(
            True, 'ok', True, 'ok', 'unknown', 'front_no_points')
        assert allowed is False
        assert reason == 'front_no_points'

    def test_clear_and_slow_allowed(self):
        for action in ('clear', 'slow'):
            allowed, reason = SS.permission(
                True, 'ok', True, 'ok', action, 'ok')
            assert allowed is True
            assert reason == 'ok'

    def test_unknown_action_enum_denied(self):
        allowed, reason = SS.permission(
            True, 'ok', True, 'ok', 'emergebcy', '')
        assert allowed is False
        assert reason.startswith('unknown_action:')
