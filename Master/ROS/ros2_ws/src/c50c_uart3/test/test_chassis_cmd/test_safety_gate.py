"""安全许可门与手动/自动分离（任务书 T2 必测项 1-6）。

许可门语义：拒绝 / 过期 / 从未收到都不允许非零；deny 边沿与过期边沿
清缓存；发布端 ttl 只能缩短接收端上限（无效 ttl 的消息整条忽略）。
手动/自动分离：manual > nav > auto > demo，手动占用窗内 auto 非零
一律不放行（手动失联不得被自动接管）。

对应必测项：
1. 手动和自动同时存在：只有选中的来源生效；
2. 手动非零时出现障碍/雷达失效/状态未知：下一次安全决策非零禁止；
3. STOP 保持期间发非零，解除后旧命令不恢复；
4. 安全状态发布者消失/过期/停在旧"安全"：进入停车，旧消息不解锁；
5. 接收端未知/写流污染：不绕过串口门控（沿用既有门，本文件只验证
   许可门不打开它们）；
6. enable_cmd_vel=false 时发布者仍存在是 opi_robot 侧现状（见
   test_drive_safety.py 与交付报告），本文件验证桥不依赖该参数。
"""

import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame
from c50c_uart3.chassis_cmd.arbiter import Arbiter, ZeroReason


def grant(arb, now, ttl=3600.0, reason='ok'):
    arb.on_safety_permission(True, reason, ttl, now)


class TestSafetyGateBasics:
    def test_no_permission_ever_blocks_nonzero(self, clock):
        """从未收到许可：非零一律禁止（fail-closed 默认）。"""
        a = Arbiter()
        a.on_publisher_count(1, 0)
        a.mark_receiver_clean(simulated=True)
        a.on_firmware_confirm(True)
        a.on_nav_cmd(0.05, 0.0, 0.0, clock())
        d = a.evaluate(clock())
        assert d.is_zero
        assert d.zero_reason is ZeroReason.SAFETY_UNKNOWN

    def test_grant_then_nonzero_flows(self, clock):
        a = Arbiter(safety_max_age_s=3600.0)
        a.on_publisher_count(1, 0)
        a.mark_receiver_clean(simulated=True)
        a.on_firmware_confirm(True)
        grant(a, clock())
        a.evaluate(clock())                      # 首帧零速
        a.on_nav_cmd(0.05, 0.0, 0.0, clock())
        d = a.evaluate(clock())
        assert not d.is_zero

    def test_deny_message_blocks_and_drops_cache(self, clock):
        """必测项 2：手动非零时许可翻拒绝——下一次决策非零禁止。"""
        a = Arbiter(safety_max_age_s=3600.0)
        a.on_publisher_count(1, 0)
        a.mark_receiver_clean(simulated=True)
        a.on_firmware_confirm(True)
        grant(a, clock())
        a.evaluate(clock())
        a.on_manual_cmd(0.2, 0.0, 0.0, clock())
        d = a.evaluate(clock())
        assert not d.is_zero                     # 许可有效，正常输出
        a.on_safety_permission(False, 'obstacle_stop', 0.0, clock())
        clock.advance(0.02)
        d = a.evaluate(clock())
        assert d.is_zero
        assert d.zero_reason is ZeroReason.SAFETY_DENIED
        assert a._counts['dropped_safety'] == 1

    def test_nonzero_during_deny_is_not_cached(self, clock):
        """拒绝期间到达的非零命令不入缓存，解除后不复活。"""
        a = Arbiter(safety_max_age_s=3600.0)
        a.on_publisher_count(1, 0)
        a.mark_receiver_clean(simulated=True)
        a.on_firmware_confirm(True)
        grant(a, clock())
        a.evaluate(clock())
        a.on_safety_permission(False, 'obstacle_stop', 0.0, clock())
        clock.advance(0.02)
        a.evaluate(clock())
        a.on_manual_cmd(0.2, 0.0, 0.0, clock())  # 停车期间的非零
        a.on_safety_permission(True, 'ok', 3600.0, clock())
        clock.advance(0.02)
        d = a.evaluate(clock())
        assert d.is_zero
        assert d.zero_reason is ZeroReason.NO_NAV_COMMAND

    def test_permission_expiry_blocks_and_drops(self, clock):
        """必测项 4：发布者停发，许可过期后进入停车，缓存被清。"""
        a = Arbiter(safety_max_age_s=0.5)
        a.on_publisher_count(1, 0)
        a.mark_receiver_clean(simulated=True)
        a.on_firmware_confirm(True)
        grant(a, clock(), ttl=0.5)
        a.evaluate(clock())
        a.on_manual_cmd(0.2, 0.0, 0.0, clock())
        d = a.evaluate(clock())
        assert not d.is_zero
        clock.advance(0.51)                      # 超过有效期
        d = a.evaluate(clock())
        assert d.is_zero
        assert d.zero_reason is ZeroReason.SAFETY_EXPIRED
        assert a._counts['dropped_safety'] == 1

    def test_receiver_clamps_publisher_ttl(self, clock):
        """发布端 ttl 不能延长接收端上限（0.5 s 钳制）。"""
        a = Arbiter(safety_max_age_s=0.5)
        grant(a, clock(), ttl=100.0)
        assert a.safety_state(clock())['remaining_s'] <= 0.5

    def test_invalid_ttl_message_is_ignored(self, clock):
        """非法 ttl（<=0）：整条消息无效，此前状态保持。

        全新仲裁器没有任何许可 → 仍按从未收到处理（SAFETY_UNKNOWN）。
        """
        a = Arbiter()
        a.on_publisher_count(1, 0)
        a.mark_receiver_clean(simulated=True)
        a.on_firmware_confirm(True)
        grant(a, clock(), ttl=0.0)
        a.evaluate(clock())
        a.on_manual_cmd(0.2, 0.0, 0.0, clock())
        d = a.evaluate(clock())
        assert d.is_zero
        assert d.zero_reason is ZeroReason.SAFETY_UNKNOWN
        assert a._counts['safety_invalid'] == 1

    def test_invalid_ttl_does_not_extend_existing_grant(self, clock):
        """已有许可时收到非法 ttl：不延长（等效发布端静默，时限兜底）。"""
        a = Arbiter(safety_max_age_s=0.5)
        grant(a, clock(), ttl=0.4)
        first = a.safety_state(clock())['remaining_s']
        grant(a, clock(), ttl=0.0)
        assert a.safety_state(clock())['remaining_s'] == first
        assert a._counts['safety_invalid'] == 1

    def test_stale_safe_permission_cannot_unlock(self, clock):
        """必测项 4：发布端消失后，旧"安全"过期即失效，不得解锁。"""
        a = Arbiter(safety_max_age_s=0.5)
        a.on_publisher_count(1, 0)
        a.mark_receiver_clean(simulated=True)
        a.on_firmware_confirm(True)
        grant(a, clock(), ttl=0.3)
        a.evaluate(clock())
        clock.advance(1.0)
        d = a.evaluate(clock())
        assert d.is_zero
        assert d.zero_reason is ZeroReason.SAFETY_EXPIRED

    def test_zero_speed_still_flows_without_permission(self, clock):
        """零速是唯一安全输出：无许可时零速帧照发（受控停止 + 心跳）。"""
        a = Arbiter()
        a.on_publisher_count(1, 0)
        a.mark_receiver_clean(simulated=True)
        d = a.evaluate(clock())
        assert d.is_zero and d.frame is not None

    def test_permission_survives_serial_reconnect(self, clock):
        """许可表征避障状态，与串口无关：重连不清许可（清命令缓存）。"""
        a = Arbiter(safety_max_age_s=3600.0)
        a.on_publisher_count(1, 0)
        a.mark_receiver_clean(simulated=True)
        a.on_firmware_confirm(True)
        grant(a, clock())
        a.on_serial_reconnect()
        assert a.safety_state(clock())['ok'] is True


class TestManualAutoSeparation:
    """必测项 1：手动与自动同时存在时只有被选中的来源生效。"""

    def _armed(self, clock):
        a = Arbiter(safety_max_age_s=3600.0)
        a.on_publisher_count(1, 0, manual_publishers=1, auto_publishers=1)
        a.mark_receiver_clean(simulated=True)
        a.on_firmware_confirm(True)
        grant(a, clock())
        a.evaluate(clock())                      # 首帧零速（seq 0）
        return a

    def test_manual_wins_over_auto(self, clock):
        a = self._armed(clock)
        a.on_manual_cmd(0.2, 0.0, 0.0, clock())
        a.on_auto_cmd(0.4, 0.0, 0.0, clock())
        d = a.evaluate(clock())
        assert not d.is_zero
        assert d.frame == encode_speed_frame(0.2, 0.0, 0.0, seq=1)

    def test_auto_value_not_used_while_manual_holding(self, clock):
        """手动占用窗内 auto 非零被抑制：自动不得在手动会话中输出。"""
        a = self._armed(clock)
        a.on_manual_cmd(0.0, 0.0, 0.0, clock())   # 手动会话存活（零速也刷新窗）
        a.on_auto_cmd(0.4, 0.0, 0.0, clock())
        d = a.evaluate(clock())
        assert d.is_zero
        assert d.zero_reason is ZeroReason.MANUAL_HOLD
        # 手动零速消息继续刷新占用窗：auto 仍被抑制
        a.on_manual_cmd(0.0, 0.0, 0.0, clock())
        clock.advance(0.2)
        d = a.evaluate(clock())
        assert d.is_zero

    def test_auto_takes_over_only_after_hold_window(self, clock):
        """占用窗过期后 auto 才参与选择（巡航是显式开启的）。"""
        a = self._armed(clock)
        a.on_manual_cmd(0.2, 0.0, 0.0, clock())
        a.evaluate(clock())
        clock.advance(0.4)                       # > manual_hold_s(0.35)
        a.on_auto_cmd(0.4, 0.0, 0.0, clock())
        d = a.evaluate(clock())
        assert not d.is_zero
        assert d.frame == encode_speed_frame(0.4, 0.0, 0.0, seq=2)

    def test_stale_auto_command_expired(self, clock):
        """auto 命令同样受 0.3 s 超时约束：过期不重发。"""
        a = self._armed(clock)
        a.on_auto_cmd(0.4, 0.0, 0.0, clock())
        assert not a.evaluate(clock()).is_zero
        clock.advance(0.34)
        d = a.evaluate(clock())
        assert d.is_zero
        assert d.zero_reason is ZeroReason.NAV_EXPIRED

    def test_manual_deny_covers_all_directions(self, clock):
        """必测项 2 的手动矩阵：前/后/左/右非零在拒绝下全部禁止。"""
        for vx, vz in ((0.2, 0.0), (-0.2, 0.0), (0.0, 0.2), (0.0, -0.2)):
            a = Arbiter(safety_max_age_s=3600.0)
            a.on_publisher_count(1, 0, manual_publishers=1)
            a.mark_receiver_clean(simulated=True)
            a.on_firmware_confirm(True)
            grant(a, clock())
            a.evaluate(clock())
            a.on_safety_permission(False, 'obstacle_stop', 0.0, clock())
            a.on_manual_cmd(vx, 0.0, vz, clock())
            clock.advance(0.02)
            d = a.evaluate(clock())
            assert d.is_zero, (vx, vz)
            assert d.zero_reason is ZeroReason.SAFETY_DENIED, (vx, vz)

    def test_auto_deny_covers_obstacle_and_lidar_states(self, clock):
        """必测项 2 的自动矩阵：障碍与雷达失效都让自动输出为零。"""
        for reason in ('obstacle_stop', 'lidar_disconnected',
                       'raw_frames_stalled', 'evidence_stale'):
            clock_t = [100.0]

            def clk():
                return clock_t[0]

            a = Arbiter(safety_max_age_s=3600.0)
            a.on_publisher_count(1, 0, auto_publishers=1)
            a.mark_receiver_clean(simulated=True)
            a.on_firmware_confirm(True)
            grant(a, clk())
            a.evaluate(clk())
            a.on_auto_cmd(0.4, 0.0, 0.0, clk())
            assert not a.evaluate(clk()).is_zero
            a.on_safety_permission(False, reason, 0.0, clk())
            clock_t[0] += 0.02
            d = a.evaluate(clk())
            assert d.is_zero, reason
            assert d.zero_reason is ZeroReason.SAFETY_DENIED, reason


class TestStopInteraction:
    def test_stop_during_permission_still_blocks(self, clock):
        """监督停车与许可门相互独立：两者任一有效都禁非零。"""
        a = Arbiter(safety_max_age_s=3600.0)
        a.on_publisher_count(1, 0)
        a.mark_receiver_clean(simulated=True)
        a.on_firmware_confirm(True)
        grant(a, clock())
        a.evaluate(clock())
        a.on_manual_cmd(0.2, 0.0, 0.0, clock())
        a.on_stop(True)
        d = a.evaluate(clock())
        assert d.is_zero
        assert d.zero_reason is ZeroReason.STOP_REQUESTED

    def test_nonzero_during_stop_not_replayed_after_release(self, clock):
        """必测项 3：STOP 期间的非零在解除后不恢复。"""
        a = Arbiter(safety_max_age_s=3600.0)
        a.on_publisher_count(1, 0)
        a.mark_receiver_clean(simulated=True)
        a.on_firmware_confirm(True)
        grant(a, clock())
        a.evaluate(clock())
        a.on_stop(True)
        a.on_manual_cmd(0.2, 0.0, 0.0, clock())  # 停车期间到达
        a.on_stop(False)
        clock.advance(0.02)
        d = a.evaluate(clock())
        assert d.is_zero
        assert d.zero_reason is ZeroReason.NO_NAV_COMMAND
        # 解除后到达的新命令才生效
        a.on_manual_cmd(0.2, 0.0, 0.0, clock())
        clock.advance(0.02)
        d = a.evaluate(clock())
        assert not d.is_zero


class TestGearCeiling:
    """挡位上限（2026-11-14 用户确认：空档 0 / 巡航挡 0.4 / 运动挡 0.8）。

    基线桥的仲裁上限钉在最高挡 0.8——挡位之外的速度（前后与转向同口径）
    在桥层拒绝，旧的 1.0/1.3 直行测试上限撤销。
    """

    def _baseline_armed(self, clock):
        a = Arbiter(firmware_protocol='baseline', safety_max_age_s=3600.0,
                    max_linear_mps=0.8, max_forward_mps=0.8,
                    max_angular_rad_s=0.8)
        grant(a, clock())
        a.mark_receiver_clean(simulated=True)
        a.on_baseline_status_frame(clock())
        a.evaluate(clock())                  # 首帧零速
        a.on_baseline_arm(clock())
        return a

    def test_bridge_wiring_pins_gear_ceiling(self):
        """桥装配的基线仲裁器上限必须是挡位上限（0.8/0.8/0.8）。"""
        from c50c_uart3.chassis_cmd.bridge import _new_arbiter
        arb = _new_arbiter('baseline')
        assert arb._limits() == (0.8, 0.8)
        assert arb._max_forward == 0.8

    def test_top_gear_passes_and_above_is_rejected(self, clock):
        a = self._baseline_armed(clock)
        a.on_manual_cmd(0.8, 0.0, 0.0, clock())
        d = a.evaluate(clock())
        assert not d.is_zero                 # 运动挡恰好放行
        a.on_manual_cmd(0.81, 0.0, 0.0, clock())
        d = a.evaluate(clock())
        assert d.is_zero
        assert d.zero_reason is ZeroReason.NAV_INVALID

    def test_turn_uses_same_gear_ceiling(self, clock):
        a = self._baseline_armed(clock)
        a.on_manual_cmd(0.0, 0.0, -0.8, clock())   # 左转取挡位值
        d = a.evaluate(clock())
        assert not d.is_zero
        a.on_manual_cmd(0.0, 0.0, -0.81, clock())
        d = a.evaluate(clock())
        assert d.is_zero and d.zero_reason is ZeroReason.NAV_INVALID

    def test_cruise_gear_below_ceiling(self, clock):
        a = self._baseline_armed(clock)
        a.on_manual_cmd(0.4, 0.0, 0.0, clock())
        assert not a.evaluate(clock()).is_zero


class TestTransportGatesUnaffected:
    """必测项 5：许可门不打开接收端未知 / 流毒等串口门控。"""

    def test_receiver_unknown_silence_unchanged_with_grant(self, clock):
        a = Arbiter(safety_max_age_s=3600.0)
        a.on_publisher_count(1, 0)
        grant(a, clock())
        a.on_manual_cmd(0.2, 0.0, 0.0, clock())
        d = a.evaluate(clock())
        assert d.frame is None
        assert d.zero_reason is ZeroReason.RECEIVER_UNKNOWN

    def test_stream_poisoned_silence_unchanged_with_grant(self, clock):
        a = Arbiter(safety_max_age_s=3600.0)
        a.on_publisher_count(1, 0)
        a.mark_receiver_clean(simulated=True)
        grant(a, clock())
        a.on_manual_cmd(0.2, 0.0, 0.0, clock())
        a.on_write_result(False, stream_poisoned=True)
        d = a.evaluate(clock())
        assert d.frame is None
        assert d.zero_reason is ZeroReason.STREAM_POISONED
