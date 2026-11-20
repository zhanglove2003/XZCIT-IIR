"""超声波补充停车门（safety_state.UltrasonicGate）与许可接入的定向测试。

只测纯逻辑（opi_robot.fusion_node.safety_state），不依赖 rclpy。
2026-11-15 前置 HC-SR04 接入融合安全门时新增。
"""

import math

import pytest

from opi_robot.fusion_node import safety_state as SS


def make_gate(stop_d=0.30, release_d=0.35, timeout_s=0.5, invalid_at=None):
    return SS.UltrasonicGate(stop_d, release_d, timeout_s, invalid_at=invalid_at)


class TestGateConfig:
    def test_bad_thresholds_rejected(self):
        with pytest.raises(ValueError):
            make_gate(0.35, 0.30)          # stop >= release
        with pytest.raises(ValueError):
            make_gate(-0.1, 0.30)          # stop <= 0
        with pytest.raises(ValueError):
            make_gate(0.30, 0.0)           # release <= 0

    def test_bad_timeout_rejected(self):
        with pytest.raises(ValueError):
            make_gate(timeout_s=0.0)
        with pytest.raises(ValueError):
            make_gate(timeout_s=-1)


class TestGateHysteresis:
    def test_never_fed_blocks(self):
        # fail-closed：从未收到读数 = 不知道，不是"畅通"
        gate = make_gate()
        stop, rng, ok, age = gate.verdict(100.0)
        assert stop is True and rng is None and ok is False and age is None
        assert gate.fault_reason == 'never'

    def test_enter_at_stop_distance(self):
        gate = make_gate()
        gate.feed(0.29, 1.0)
        assert gate.verdict(1.1)[0] is True

    def test_enter_at_exact_stop_distance(self):
        gate = make_gate()
        gate.feed(0.30, 1.0)
        assert gate.verdict(1.1)[0] is True

    def test_above_stop_no_entry(self):
        gate = make_gate()
        gate.feed(0.31, 1.0)
        stop, rng, ok, age = gate.verdict(1.1)
        assert stop is False and rng == pytest.approx(0.31) and ok is True

    def test_hysteresis_holds_between_thresholds(self):
        gate = make_gate()
        gate.feed(0.25, 1.0)               # 进入
        gate.feed(0.32, 2.0)               # 0.30-0.35 之间：保持
        assert gate.verdict(2.1)[0] is True

    def test_exit_at_release_distance(self):
        gate = make_gate()
        gate.feed(0.25, 1.0)               # 进入
        gate.feed(0.34, 1.2)               # 迟滞区：保持
        gate.feed(0.35, 1.4)               # 恰在退出阈值：退出（见下）
        stop, rng, ok, age = gate.verdict(1.45)
        # [2026-11-17 复核] 缺省 invalid_at 不再等于 release_d，读数恰好等于
        # release_d 时正常退出迟滞，且为有效读数。
        assert stop is False and ok is True and gate.fault is False

    def test_exit_below_release_keeps_latch(self):
        gate = make_gate()
        gate.feed(0.25, 1.0)
        gate.feed(0.34, 2.0)
        assert gate.verdict(2.1)[0] is True

    def test_reenter_after_exit(self):
        gate = make_gate()
        gate.feed(0.25, 1.0)
        gate.feed(0.40, 2.0)               # 退出
        gate.feed(0.20, 3.0)               # 再进
        assert gate.verdict(3.1)[0] is True

    def test_far_readings_never_latch(self):
        # 地面回波区读数（< max_range）不得触发停车。
        # 缺省 invalid_at = 4.5（传感器规格上限）：0.40/0.92 都是量程内读数
        gate = make_gate()
        for r in (0.40, 0.92):
            gate.feed(r, 1.0)
            assert gate.verdict(1.1)[0] is False
        assert gate.fault is False
        # [2026-11-18 用户定案] invalid_at 现在是"远处无约束"界：
        # >= invalid_at（含 0xFFFF→max_range）= 远处，解除锁存、不停车。
        # 阈值边界用 NaN/0 坏数据与 release_m 近距停车验证。
        wide = make_gate(invalid_at=4.5)
        for r in (0.40, 0.92, 4.5, 4.6):
            wide.feed(r, 1.0)
            stop, rng, ok, age = wide.verdict(1.1)
            assert stop is False and ok is True, r
        assert wide.fault is False
        # 近距照常停车
        near = make_gate(invalid_at=4.5)
        near.feed(0.30, 1.0)
        assert near.verdict(1.1)[0] is True
        # 坏数据仍锁停
        bad = make_gate(invalid_at=4.5)
        bad.feed(float('nan'), 1.0)
        assert bad.verdict(1.1)[0] is True


class TestGateFailClosed:
    """[2026-11-18 用户定案] 坏数据(NaN/0/负)/断流 = 无效 = 停车；
    无回波（0xFFFF→max_range）与超 invalid_at = "远处无约束"，不锁车。"""

    def test_no_echo_max_range_clears_and_does_not_stop(self):
        gate = make_gate(invalid_at=4.5)
        gate.feed(0.20, 1.0)               # 先锁存近距停车
        gate.feed(4.5, 1.1)                # 0xFFFF 映射值 = 波束打空
        stop, rng, ok, age = gate.verdict(1.2)
        assert stop is False and ok is True   # 不锁车、有效（远处）
        assert gate.latched is False          # 近距锁存被解除
        assert gate.fault is False

    def test_no_echo_from_clear_state_stays_clear(self):
        gate = make_gate(invalid_at=4.5)
        gate.feed(2.0, 1.0)
        assert gate.verdict(1.1)[0] is False
        gate.feed(4.5, 1.2)
        assert gate.verdict(1.3)[0] is False   # 远处仍畅通

    def test_beyond_invalid_at_is_far_not_invalid(self):
        gate = make_gate(invalid_at=4.5)
        gate.feed(4.6, 1.0)
        assert gate.verdict(1.1)[0] is False

    def test_nan_zero_negative_still_invalid(self):
        for r in (float('nan'), float('inf'), 0.0, -0.1):
            gate = make_gate()
            gate.feed(r, 1.0)
            stop, rng, ok, age = gate.verdict(1.1)
            assert stop is True and ok is False, r
            assert gate.fault_reason == 'no_echo' 

    def test_stale_reading_fails_closed(self):
        gate = make_gate(timeout_s=0.5)
        gate.feed(0.20, 1.0)               # 锁存停车
        assert gate.verdict(1.4)[0] is True    # 时限内仍停车
        stop, rng, ok, age = gate.verdict(1.6)     # 0.6s 无新读数
        assert stop is True and rng is None and ok is False
        assert age == pytest.approx(0.6)
        assert gate.fault_reason == 'stale'

    def test_stale_after_clear_still_blocks(self):
        gate = make_gate(timeout_s=0.5)
        gate.feed(2.0, 1.0)                # 2.0 >= release 0.35 ⇒ no_echo
        gate2 = make_gate(timeout_s=0.5, invalid_at=4.5)
        gate2.feed(2.0, 1.0)
        assert gate2.verdict(1.1)[0] is False
        assert gate2.verdict(1.9)[0] is True

    def test_recovery_requires_rearm_not_new_reading(self):
        # 用户要求：坏数据(NaN)无效即停，恢复后需重新启动；有效读数自己
        # 不得解除无效锁（远处读数本来就不触发无效锁，见新口径）
        gate = make_gate(timeout_s=0.5, invalid_at=4.5)
        gate.feed(float('nan'), 1.0)       # 坏数据
        assert gate.verdict(1.1)[0] is True
        for k in range(4):                 # 全在 0.5 s 时限内（不触发 stale）
            gate.feed(2.0, 1.1 + 0.1 * k)  # 量程内读数也不能自行解除
            assert gate.verdict(1.15 + 0.1 * k)[0] is True, k
        assert gate.fault is True
        gate.rearm(1.7)
        assert gate.fault is False and gate.last_rx_t is None
        gate.feed(2.0, 1.8)
        stop, rng, ok, age = gate.verdict(1.85)
        # [2026-11-17 复核修复] 复位后由新读数按两阈值判定：2.0 m > release
        # → 放行且有效。复位后、新读数前调 verdict 会重锁 never 故障（见
        # test_rearm_before_new_reading_still_stops），故不在读数前调。
        assert stop is False and ok is True and rng == pytest.approx(2.0)
        assert gate.rearms == 1

    @pytest.mark.parametrize('now_first', [False, True])
    def test_rearm_then_readings(self, now_first):
        """repro3：带/不带 now 的 rearm 之后，正向读数必须能解除停车。"""
        gate = make_gate()
        gate.feed(float('nan'), 1.0)       # [2026-11-18] 坏数据 → 无效锁
        assert gate.verdict(1.1)[2] is False
        gate.rearm(1.2 if now_first else None)
        gate.feed(0.9, 1.3)
        stop, rng, ok, _age = gate.verdict(1.4)
        assert stop is False and ok is True
        assert rng == pytest.approx(0.9)
        # 近距再进停车，仍按阈值
        gate.feed(0.2, 1.5)
        assert gate.verdict(1.6)[0] is True
        gate.feed(0.9, 1.7)                # 0.9 > release 0.35 → 退出迟滞
        assert gate.verdict(1.8)[0] is False

    def test_rearm_before_new_reading_still_stops(self):
        """复位后等待新读数，定时器不能重新锁存 never。"""
        gate = make_gate()
        gate.feed(float('nan'), 1.0)   # [2026-11-18] 坏数据触发无效锁
        gate.rearm(1.5)
        stop, _rng, ok, _age = gate.verdict(1.6)
        assert stop is True and ok is False
        assert gate.fault is False
        assert gate.fault_reason == ''
        # 不再次复位，新读数应能恢复。
        gate.feed(1.0, 1.8)
        stop, rng, ok, _age = gate.verdict(1.9)
        assert stop is False and ok is True and rng == pytest.approx(1.0)

    def test_rearm_still_stops_if_near(self):
        gate = make_gate()
        gate.feed(float('nan'), 1.0)   # [2026-11-18] 坏数据触发无效锁
        gate.rearm(1.5)
        gate.feed(0.25, 2.0)
        assert gate.verdict(2.1)[0] is True

    def test_latch_survives_timeout_with_rearm(self):
        gate = make_gate(timeout_s=0.5)
        gate.feed(0.20, 1.0)
        gate.verdict(2.0)                  # 断流超时 → 无效
        gate.rearm(2.05)
        gate.feed(0.20, 2.1)               # 恢复且仍近 → 立即再停
        assert gate.verdict(2.2)[0] is True

    def test_invalid_at_must_exceed_release(self):
        # [2026-11-17 复核] 必须严格大于 release_d：等于时"读数恰好 =
        # release_d（应退出迟滞）"会被判 no_echo。
        with pytest.raises(ValueError):
            make_gate(invalid_at=0.30)
        with pytest.raises(ValueError):
            make_gate(invalid_at=0.35)
        gate = make_gate(invalid_at=0.40)
        assert gate.invalid_at == pytest.approx(0.40)
        assert make_gate().invalid_at == pytest.approx(4.5)   # 缺省 = max_range


class TestGateInputHygiene:
    def test_nan_invalid(self):
        gate = make_gate()
        gate = make_gate(invalid_at=4.5)
        gate.feed(2.0, 1.0)
        gate.feed(float('nan'), 2.0)       # 不更新锁存，也不刷新新鲜度
        stop, rng, ok, age = gate.verdict(2.1)
        assert stop is True and rng is None and ok is False
        # 时间戳没被 NaN 刷新：1.1s > 0.5s 超时；最后可采信读数保留
        assert gate.snapshot()['range_m'] == pytest.approx(2.0)
        assert age == pytest.approx(1.1)

    def test_inf_invalid(self):
        gate = make_gate()
        gate.feed(float('inf'), 1.0)
        stop, rng, ok, age = gate.verdict(1.1)
        assert stop is True and rng is None and ok is False and age is None

    def test_none_discarded_numeric_string_accepted(self):
        gate = make_gate()
        gate.feed(None, 1.0)               # None → TypeError → 无效
        assert gate.verdict(1.1)[0] is True
        gate.rearm(1.2)
        gate.feed("0.2", 1.3)              # 数字字符串按 float() 语义接受
        assert gate.verdict(1.4)[0] is True


class TestPermissionIntegration:
    BASE = dict(evidence_ok=True, evidence_reason='ok',
                trusted_ok=True, trusted_why='ok',
                action='clear', action_reason='ok')

    def test_us_stop_denies(self):
        allowed, reason = SS.permission(us_stop=True, us_reason='ultrasonic_stop',
                                        **self.BASE)
        assert allowed is False and reason == 'ultrasonic_stop'

    def test_us_default_off_keeps_old_behaviour(self):
        allowed, reason = SS.permission(**self.BASE)
        assert allowed is True and reason == 'ok'

    def test_lidar_stop_priority_above_us(self):
        # 雷达停车优先：原因取 obstacle_stop（两者都拒绝，仅证据链更清晰）
        allowed, reason = SS.permission(action='stop', action_reason='x',
                                        us_stop=True, us_reason='ultrasonic_stop',
                                        **{k: v for k, v in self.BASE.items()
                                           if not k.startswith('action')})
        assert allowed is False and reason == 'obstacle_stop'

    def test_us_stop_beats_unknown(self):
        # 雷达前方无点（unknown）时超声波近距回波给出更准确的原因
        allowed, reason = SS.permission(action='unknown', action_reason='front_no_points',
                                        us_stop=True, us_reason='ultrasonic_stop',
                                        **{k: v for k, v in self.BASE.items()
                                           if not k.startswith('action')})
        assert allowed is False and reason == 'ultrasonic_stop'

    def test_us_reason_default(self):
        allowed, reason = SS.permission(us_stop=True, **self.BASE)
        assert allowed is False and reason == 'ultrasonic_stop'

    def test_evidence_failure_still_dominates(self):
        allowed, reason = SS.permission(
            evidence_ok=False, evidence_reason='evidence_stale',
            us_stop=True, us_reason='ultrasonic_stop',
            **{k: v for k, v in self.BASE.items()
               if not k.startswith('evidence')})
        assert allowed is False and reason == 'evidence_stale'


@pytest.mark.parametrize('reading', [0.2, 0.9, 4.5])
def test_rearm_timer_before_sample_recovers(reading):
    gate = make_gate()
    gate.feed(1.0, 1.0)
    assert gate.verdict(2.0)[2] is False
    gate.rearm(2.1)
    for t in (2.2, 2.3):
        assert gate.verdict(t) == (True, None, False, None)
    gate.feed(reading, 2.4)
    stop, rng, valid, age = gate.verdict(2.5)
    assert valid is True and rng == pytest.approx(reading)
    assert stop is (reading <= 0.30)


def test_bad_sample_after_rearm_still_requires_another_rearm():
    gate = make_gate()
    gate.rearm(1.0)
    gate.verdict(1.1)
    gate.feed(float('nan'), 1.2)
    gate.feed(0.9, 1.3)
    assert gate.fault is True
    assert gate.verdict(1.4)[2] is False
