"""旧命令重放与新鲜度（任务书 T3 必测项 5）。

场景：上游失效后，下游仍持续收到**重复的旧非零命令**（例如发布者把
同一 Twist 反复重发、消息带原始时间戳）。要求：旧帧不能刷新有效命令
计时，也不能续期手动占用窗——"串口完全没数据"之外的失效形态。

时间语义：命令的 ``now`` 由上游携带（到达时刻），仲裁器用它计算年龄；
重复的旧帧不推进年龄，过期判定照常触发。
"""

import pytest

from c50c_uart3.chassis_cmd.arbiter import Arbiter, ZeroReason


def _armed(clock, t0=100.0):
    a = Arbiter(safety_max_age_s=3600.0)
    a.on_publisher_count(1, 0, manual_publishers=1, auto_publishers=1)
    a.mark_receiver_clean(simulated=True)
    a.on_firmware_confirm(True)
    a.on_safety_permission(True, 'test_granted', 3600.0, t0)
    a.evaluate(t0)                       # 首帧零速
    return a


class TestStaleReplay:
    def test_repeated_old_frame_never_refreshes_timer(self, clock):
        """同一旧命令重复到达：年龄不刷新，输出永远零速。"""
        a = _armed(clock)
        old_t = clock() - 2.0            # 命令产生于 2 s 前
        for _ in range(10):
            a.on_manual_cmd(0.2, 0.0, 0.0, old_t)   # 重复同一旧帧
            clock.advance(0.02)
            d = a.evaluate(clock())
            assert d.is_zero
            assert d.zero_reason is ZeroReason.NAV_EXPIRED

    def test_replayed_frame_does_not_extend_hold_window(self, clock):
        """重复旧帧不能续期手动占用窗：窗过期后 auto 可接管。"""
        a = _armed(clock)
        old_t = clock() - 2.0
        for _ in range(10):
            a.on_manual_cmd(0.2, 0.0, 0.0, old_t)   # 旧帧重放
            clock.advance(0.02)
            a.evaluate(clock())
        # 占用窗由 on_manual_cmd 的 now（旧时刻）决定，早已过期
        a.on_auto_cmd(0.4, 0.0, 0.0, clock())
        clock.advance(0.02)
        d = a.evaluate(clock())
        assert not d.is_zero, '占用窗必须由消息携带的时刻计算，旧帧不得续期'
        assert d.frame[3:5] == b'\x01\x90'   # vx = 0.4 m/s

    def test_stale_arrival_is_dropped_not_cached(self, clock):
        """到达时已超时的命令：不缓存为可执行候选（不重发语义）。"""
        a = _armed(clock)
        a.on_manual_cmd(0.2, 0.0, 0.0, clock() - 0.4)
        clock.advance(0.02)
        d = a.evaluate(clock())
        assert d.is_zero
        assert d.zero_reason is ZeroReason.NAV_EXPIRED

    def test_new_frame_after_silence_reauthorises_motion(self, clock):
        """对照：静默之后真正新鲜的命令可以放行（不误伤正常操作）。"""
        a = _armed(clock)
        clock.advance(2.0)               # 长时间无命令
        a.on_manual_cmd(0.2, 0.0, 0.0, clock())
        clock.advance(0.02)
        d = a.evaluate(clock())
        assert not d.is_zero

    def test_replay_while_gate_closed_leaves_no_resurrectable_cache(self, clock):
        """许可失效期间重放旧非零：不入缓存；恢复后旧速度不复活。"""
        a = _armed(clock)
        a.on_safety_permission(False, 'obstacle_stop', 0.0, clock())
        clock.advance(0.02)
        a.evaluate(clock())
        old_t = clock() - 2.0
        for _ in range(5):
            a.on_manual_cmd(0.2, 0.0, 0.0, old_t)   # 失效期间旧帧重放
            clock.advance(0.02)
            assert a.evaluate(clock()).zero_reason is ZeroReason.SAFETY_DENIED
        a.on_safety_permission(True, 'ok', 3600.0, clock())
        clock.advance(0.02)
        d = a.evaluate(clock())
        assert d.is_zero
        assert d.zero_reason is ZeroReason.NO_NAV_COMMAND
