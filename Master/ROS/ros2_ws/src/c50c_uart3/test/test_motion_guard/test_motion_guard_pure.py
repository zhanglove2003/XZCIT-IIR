"""motion_guard 纯逻辑测试（无 ROS、注入时钟）。

覆盖：四路仲裁、锁停清缓存、松键不锁停、许可重放/旧代次/过期/500ms
中断、启动与重连默认锁停、rearm 前提、超声 invalid/stale、雷达帧龄与
方向扇区（前/后/左/右）、几何复用。
"""
import json
import math

import pytest

from c50c_uart3 import motion_guard as mg
from c50c_uart3.motion_guard import (License, MotionGuard, direction_sectors,
                                     scan_check, scan_stamp_to_mono,
                                     sector_points, ultrasonic_state)


class Clock:
    def __init__(self, t=100.0):
        self.t = float(t)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)
        return self.t


class Scan:
    """完整一圈 0.01 rad 的 LaserScan 替身；默认全量程（空场）。"""

    def __init__(self, fill=5.0):
        self.n = 628
        self.angle_min = 0.0
        self.angle_increment = 2.0 * math.pi / self.n
        self.range_min = 0.02
        self.range_max = 12.0
        self.ranges = [fill] * self.n

    def clear(self, center_deg, half_deg=35.0):
        """把某扇区设成"无有效点"（NaN）。"""
        for i in range(self.n):
            if abs(_wrap(_scan_rad(self, i) - math.radians(center_deg))) \
                    <= math.radians(half_deg):
                self.ranges[i] = float('nan')


def _scan_rad(scan, i):  # noqa: D401
    return (scan.angle_min + i * scan.angle_increment) % (2.0 * math.pi)


def _wrap(a):
    return ((a + math.pi) % (2.0 * math.pi)) - math.pi


def lic(session=1, seq=0, exp=None, now=100.0):
    return json.dumps({'session': session, 'seq': seq,
                       'expires_mono': now + 1.0 if exp is None else exp})


def armed(clock, *, scan=None, stamp=None, feed=True):  # noqa: D401
    """许可在线 + 已 rearm + 雷达新鲜。返回 guard。

    ``feed=True``：每拍给一帧新鲜雷达（``stamp`` 给定时改为每拍固定重发
    该 stamp）；``feed=False``：帧不再更新，用来验证帧龄/方向判定。
    """
    g = MotionGuard(clock)
    assert g.accept_license_json(lic(now=clock()))[0]
    assert g.rearm(True)[0]
    g._scan = scan if scan is not None else Scan()   # noqa: SLF001
    fixed = None if stamp is None else float(stamp)
    if feed:
        g._feed = lambda: g.on_scan(g._scan, fixed if fixed is not None   # noqa: SLF001
                                    else clock() - 0.05)
        g._feed()  # noqa: SLF001
    return g


def drive(g, range_m=0.30, age=0.0, feed=True):
    if feed and getattr(g, '_feed', None) is not None and getattr(
            g, '_scan', None) is not None:
        g._feed()  # noqa: SLF001——模拟每拍都有新一帧雷达
    state, _ = ultrasonic_state(range_m, age)
    return g.decide(ultrasonic=state)


# ---------------------------------------------------------------- 许可
def test_license_replay_is_rejected_and_does_not_advance_seq():
    c = Clock()
    lic_obj = License()
    assert lic_obj.accept(License.parse(lic(seq=5)), c())[0]
    assert lic_obj.accept(License.parse(lic(seq=5)), c()) == (False, 'seq_replay')
    assert lic_obj.seq == 5
    assert lic_obj.accept(License.parse(lic(seq=4)), c()) == (False, 'seq_replay')
    assert lic_obj.accept(License.parse(lic(seq=6)), c())[0]


def test_license_old_session_rejected():
    c = Clock()
    g = armed(c)
    assert g.accept_license_json(lic(session=2, seq=100, now=c())) \
        == (False, 'session_mismatch')
    # 旧代次消息不推进原会话的序号：seq=0 仍被判重放
    assert g.accept_license_json(lic(session=1, seq=0, now=c())) \
        == (False, 'seq_replay')
    # 旧代次属于"明确的坏消息"→锁停；重放不锁停（见下一用例）
    assert g.stopped and g.stop_cause == 'license_rejected'
    g.rearm(True)
    assert g.accept_license_json(lic(session=1, seq=0, now=c())) \
        == (False, 'seq_replay')
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g)[3] == 'auto'


def test_license_expired_by_clock_rejected():
    c = Clock()
    g = MotionGuard(c)
    assert g.accept_license_json(
        lic(seq=1, exp=c() - mg.LICENSE_EXPIRY_GRACE_S - 0.001)) == (False, 'expired')
    # 恰好等于 now：落在发送/接收延迟宽限内，接受；宽限外拒绝
    assert g.accept_license_json(lic(seq=1, exp=c()))[0]
    assert g.accept_license_json(
        lic(seq=2, exp=c() - mg.LICENSE_EXPIRY_GRACE_S - 0.001)) == (False, 'expired')
    assert g.accept_license_json(lic(seq=2, exp=c() + 0.01))[0]


def test_license_malformed_or_bad_fields_rejected_and_latches():
    c = Clock()
    g = armed(c)
    assert g.accept_license_json('{"session": 1, "seq": 9}') \
        == (False, 'expires_invalid')
    assert g.stopped and g.stop_cause == 'license_rejected'
    g.rearm(True)
    assert g.accept_license_json('not json') == (False, 'malformed')
    assert g.stopped
    g.rearm(True)
    assert g.accept_license_json(json.dumps(
        {'session': True, 'seq': 10, 'expires_mono': 200}))[0] is False
    assert g.stopped


def test_500ms_renewal_gap_latches_all_four_and_clears_cache():
    c = Clock()
    g = armed(c)
    for ch, v in (('manual', 0.1), ('nav', 0.2), ('auto', 0.3), ('demo', 0.4)):
        g.on_command(ch, v, 0, 0)
    g.set_manual_hold(True)
    assert g.accept_license_json(lic(seq=1, now=c()))[0]   # 续期
    c.advance(0.49)
    assert drive(g)[4] == ''
    c.advance(0.02)                                        # 0.51 s > 500 ms
    assert drive(g) == (0.0, 0.0, 0.0, None, 'license_expired')
    assert g.stopped
    assert all(g.cached(ch) == [0.0, 0.0, 0.0] for ch in mg.CHANNELS)
    # 中断期只写过零速帧：同代次合法续期自动解停（唯一自动路径），
    # 但锁停期间到达的命令未缓存，必须等续期后的新命令。
    # 没有自动解停：许可续期也不解除锁停，必须显式 rearm（补丁后语义）
    assert g.accept_license_json(lic(seq=2, now=c()))[0]
    assert g.stopped and g.stop_cause == 'license_expired'
    assert drive(g) == (0.0, 0.0, 0.0, None, 'license_expired')
    g.on_command('auto', 0.3, 0, 0)
    assert drive(g) == (0.0, 0.0, 0.0, None, 'license_expired')
    assert g.rearm(True)[0]
    assert drive(g) == (0.0, 0.0, 0.0, None, 'no_demand')   # rearm 清缓存
    g.on_command('auto', 0.3, 0, 0)
    assert drive(g) == (0.3, 0.0, 0.0, 'auto', '')


def test_license_resume_does_not_release_startup_or_reconnect_latch():
    c = Clock()
    g = MotionGuard(c)
    g.on_scan(Scan(), c())
    assert g.accept_license_json(lic(seq=0, now=c()))[0]
    assert drive(g)[4] == 'global_stop'             # 启动锁停
    assert g.accept_license_json(lic(seq=1, now=c()))[0]
    assert drive(g)[4] == 'global_stop'
    assert g.rearm(True)[0]
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g)[3] == 'auto'
    g.stop('serial_reconnect')
    assert g.accept_license_json(lic(seq=2, now=c()))[0]
    assert drive(g)[4] == 'serial_reconnect'        # 重连锁停不自动解除
    assert g.rearm(True)[0]
    assert drive(g)[4] == 'no_demand'               # rearm 清缓存，旧命令未复活


# ---------------------------------------------------------------- 仲裁
def test_four_way_arbitration_and_hold_release_is_not_global_stop():
    c = Clock()
    g = armed(c)
    g.on_command('demo', 0.1, 0, 0)
    assert drive(g)[3] == 'demo'
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g)[3] == 'auto'
    g.on_command('nav', 0.25, 0, 0)
    assert drive(g)[3] == 'nav'
    g.on_command('manual', 0.3, 0, 0)
    g.set_manual_hold(True)
    assert drive(g) == (0.3, 0.0, 0.0, 'manual', '')
    g.set_manual_hold(False)                # 普通松键
    out = drive(g)
    assert out[3] == 'nav' and out[4] == '' and not g.stopped
    # 松键后连"手动"都没有时，其它来源照常
    for ch in ('nav', 'auto', 'demo'):
        g.on_command(ch, 0.0, 0, 0)
    out = drive(g)
    assert out[4] == 'no_demand' and not g.stopped


def test_lock_clears_cache_and_rearm_does_not_revive_old_commands():
    c = Clock()
    g = armed(c)
    g.on_command('manual', 0.3, 0, 0)
    g.set_manual_hold(True)
    g.stop()
    assert drive(g) == (0.0, 0.0, 0.0, None, 'global_stop')
    assert all(g.cached(ch) == [0.0, 0.0, 0.0] for ch in mg.CHANNELS)
    g.set_manual_hold(True)
    g.rearm(True)
    assert drive(g) == (0.0, 0.0, 0.0, None, 'no_demand')
    assert not g.stopped


def test_rearm_requires_explicit_precondition():
    c = Clock()
    g = MotionGuard(c)
    assert g.stopped and g.stop_cause == 'global_stop'   # 启动默认锁停
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g) == (0.0, 0.0, 0.0, None, 'global_stop')
    g.request_rearm()
    assert g.rearm()[0] is False
    assert drive(g)[4] == 'rearm_pending'
    assert drive(g) == (0.0, 0.0, 0.0, None, 'rearm_pending')
    assert g.rearm(True)[0]
    assert not g.stopped


def test_serial_reconnect_default_latch():
    c = Clock()
    g = armed(c)
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g)[3] == 'auto'
    g.stop('serial_reconnect')
    assert g.stopped
    assert drive(g) == (0.0, 0.0, 0.0, None, 'serial_reconnect')


# ---------------------------------------------------------------- 超声
ULTRASONIC_TABLE = [
    # [2026-11-18 用户定案] 无回波(max_range)/超释放门限 = 远处无约束，
    # 不再 invalid 锁车；坏数据(NaN/inf/0)仍 invalid。
    (4.5, 0.0, 'ok'),            # uart3_node 把 0xFFFF 映射为 max_range → 远处
    (0.50, 0.0, 'ok'),           # >= 0.50 m = 远处无约束（不再锁车）
    (float('nan'), 0.0, 'invalid'),
    (float('inf'), 0.0, 'invalid'),
    (0.0, 0.0, 'invalid'),
    (0.49, 0.0, 'ok'),
    (0.30, 0.301, 'stale'),      # 断流 > 0.3 s
    (0.30, None, 'stale'),
    (None, 0.0, 'invalid'),
]


def test_ultrasonic_state_table():
    for r, age, state in ULTRASONIC_TABLE:
        assert ultrasonic_state(r, age)[0] == state, (r, age)


@pytest.mark.parametrize('channel', mg.CHANNELS)
def test_ultrasonic_bad_data_stops_all_four_and_needs_rearm(channel):
    c = Clock()
    g = armed(c)
    g.on_command(channel, -0.2, 0, 0)
    g.set_manual_hold(channel == 'manual')
    assert drive(g)[4] == ''
    assert drive(g, range_m=float('nan')) == (0.0, 0.0, 0.0, None, 'ultrasonic_invalid')
    assert g.stopped
    assert all(g.cached(ch) == [0.0, 0.0, 0.0] for ch in mg.CHANNELS)
    assert drive(g, range_m=0.3)[4] == 'rearm_required'   # [B6] 恢复但未 rearm
    g.rearm(True)
    g.on_command(channel, -0.2, 0, 0)
    g.set_manual_hold(channel == 'manual')
    assert drive(g, range_m=0.3)[:4] == (-0.2, 0.0, 0.0, channel)


def test_sensor_recovery_after_request_unlock_still_needs_rearm():
    c = Clock()
    g = armed(c)
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g, range_m=float('nan'))[4] == 'ultrasonic_invalid'
    assert g.stopped
    g.request_rearm()                       # 网页请求：只登记意图，不解除锁停
    assert g.stopped
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g, range_m=0.30) == (0.0, 0.0, 0.0, None, 'rearm_pending')
    assert g.stopped
    assert g.rearm(True)[0]
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g, range_m=0.30)[3] == 'auto'


def test_request_does_not_unlock_startup_or_reconnect_latch():
    c = Clock()
    g = MotionGuard(c)
    g.on_scan(Scan(), c())
    assert g.accept_license_json(lic(now=c()))[0]
    g.request_rearm()
    assert drive(g) == (0.0, 0.0, 0.0, None, 'rearm_pending')
    assert g.rearm(True)[0]
    g.stop('serial_reconnect')
    g.request_rearm()
    assert drive(g)[4] == 'rearm_pending'
    assert g.stopped


def test_scan_recovery_after_request_unlock_still_needs_rearm():
    """复核 R2-1：雷达 scan_blocked / scan_stale 锁停后，网页请求再恢复帧也不发速度。"""
    for blocked in (0.0,):
        c = Clock()
        scan = Scan()
        scan.clear(blocked)
        g = armed(c, scan=scan)
        g.on_command('auto', 0.2, 0, 0)
        assert drive(g)[4] == 'scan_blocked'
        g.request_rearm()                       # 网页请求解停：不解除锁停
        assert g.stopped
        scan.ranges = [5.0] * scan.n            # 障碍移开（全向恢复）
        g.on_command('auto', 0.2, 0, 0)
        assert drive(g) == (0.0, 0.0, 0.0, None, 'rearm_pending')
        assert g.stopped
        assert g.rearm(True)[0]
        g.on_command('auto', 0.2, 0, 0)
        assert drive(g)[:4] == (0.2, 0.0, 0.0, 'auto')
    # 陈旧 → 请求 → 新帧到达
    c = Clock()
    g = armed(c, stamp=c() - 0.31)
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g)[4] == 'scan_stale'
    g.request_rearm()
    c.advance(0.05)                             # 新帧到达（arming 时 _feed 的 0.05 s 偏移）


def test_scan_stale_after_request_and_fresh_frame_still_needs_rearm():
    c = Clock()
    g = armed(c, stamp=c() - 0.31)
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g)[4] == 'scan_stale'
    g.request_rearm()
    c.advance(0.05)
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g) == (0.0, 0.0, 0.0, None, 'rearm_pending')
    g.on_scan(g._scan, c())                     # 恢复后的新帧：仍要 rearm
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g, feed=False)[4] == 'rearm_pending'
    assert g.rearm(True)[0]
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g, feed=False)[:4] == (0.2, 0.0, 0.0, 'auto')


def test_license_request_then_renew_does_not_resume_without_rearm():
    """复核 R2-3：许可过期 → 请求解停 → 合法续期，不得直接恢复。"""
    c = Clock()
    g = armed(c)
    g.on_command('auto', 0.2, 0, 0)
    c.advance(1.2)                              # 断流 > 500 ms（且消息已过期）
    assert drive(g)[4] == 'license_expired'
    g.request_rearm()
    assert g.accept_license_json(lic(seq=1, now=c()))[0]
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g) == (0.0, 0.0, 0.0, None, 'rearm_pending')
    assert g.stopped
    assert g.rearm(True)[0]
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g)[3] == 'auto'


def test_ultrasonic_stale_latches():
    c = Clock()
    g = armed(c)
    g.on_command('auto', 0.1, 0, 0)
    assert drive(g, age=0.31)[4] == 'ultrasonic_stale'
    assert g.stopped


# ---------------------------------------------------------------- 雷达
def test_scan_stamp_to_mono_uses_original_stamp():
    class S:
        sec = 1000
        nanosec = 250_000_000
    # 墙钟 1000.5，单调 500.0 → 帧在 0.25 s 前
    assert scan_stamp_to_mono(S(), now_mono=500.0, now_wall=1000.5) \
        == pytest.approx(499.75)
    S.sec, S.nanosec = 0, 0
    assert scan_stamp_to_mono(S(), now_mono=1.0, now_wall=1.0) is None


def test_scan_age_over_300ms_is_stale_and_latches():
    c = Clock()
    g = armed(c, stamp=c() - 0.299)
    g.on_command('auto', 0.1, 0, 0)
    assert drive(g, feed=False)[4] == ''     # 帧龄 0.299 s
    c.advance(0.01)                          # 同一帧：0.309 s
    assert drive(g, feed=False) == (0.0, 0.0, 0.0, None, 'scan_stale')
    assert g.stopped
    g.rearm(True)
    g.on_command('auto', 0.1, 0, 0)
    assert drive(g, feed=False)[4] == 'scan_stale'   # 同一旧帧：rearm 也走不了


def test_scan_stale_judged_by_header_stamp_not_arrival():
    stamp_mono = scan_stamp_to_mono(
        type('S', (), {'sec': 1000, 'nanosec': 0})(), now_mono=100.4,
        now_wall=1000.4)
    assert stamp_mono == pytest.approx(100.0)
    ok, why, _ = scan_check(Scan(), stamp_mono, 100.4, 0.1, 0.0)
    assert (ok, why) == (False, 'scan_stale')


@pytest.mark.parametrize('vx,vz,blocked_dir', [
    (0.2, 0.0, 0.0),        # 前进查前
    (-0.2, 0.0, 180.0),     # 后退查后
    (0.0, 0.5, -90.0),       # B4 实测：命令层 vz>0 = 顺时针右转查右
    (0.0, -0.5, 90.0),     # vz<0 = 左转查左
    (0.2, 0.5, -90.0),       # 前进+右转：扫过区域含右侧
])
def test_direction_check_blocks_when_travel_sector_empty(vx, vz, blocked_dir):
    c = Clock()
    scan = Scan()
    scan.clear(blocked_dir)
    g = armed(c, scan=scan)
    g.on_command('auto', vx, 0, vz)
    out = drive(g)
    assert out == (0.0, 0.0, 0.0, None, 'scan_blocked'), (vx, vz, blocked_dir)
    assert g.stopped
    assert all(g.cached(ch) == [0.0, 0.0, 0.0] for ch in mg.CHANNELS)


def test_opposite_direction_empty_does_not_block():
    c = Clock()
    scan = Scan()
    scan.clear(180.0)                   # 后方无点
    g = armed(c, scan=scan)
    g.on_command('auto', 0.2, 0, 0)     # 前进：只查前
    assert drive(g)[3] == 'auto'
    c2 = Clock()
    scan2 = Scan()
    scan2.clear(0.0)                    # 前方无点
    g2 = armed(c2, scan=scan2)
    g2.on_command('auto', -0.2, 0, 0)   # 后退：只查后
    assert drive(g2)[3] == 'auto'


def test_turn_uses_side_sector_only():
    c = Clock()
    scan = Scan()
    scan.clear(0.0)                     # 前方无点
    g = armed(c, scan=scan)
    g.on_command('auto', 0.0, 0, -0.5)  # 原地左转（B4：-vz = 左）：查左侧
    assert drive(g)[3] == 'auto'


def test_direction_sectors_and_geometry_reuse():
    # B4 用户实测定案：命令层 vz>0 = 顺时针（REP-103 反向），轮差速 m/s。
    assert direction_sectors(0.1, 0.0) == [0.0]
    assert direction_sectors(-0.1, 0.0) == [180.0]
    assert direction_sectors(0.0, 0.1) == [-90.0]     # +vz 右转查右
    assert direction_sectors(0.0, -0.1) == [90.0]   # -vz 左转查左
    assert direction_sectors(0.1, 0.1) == [0.0, -45.0, -90.0]    # 前右弧含扫过
    assert direction_sectors(0.1, -0.1) == [0.0, 45.0, 90.0]  # 前左弧
    assert direction_sectors(-0.1, 0.1) == [180.0, -135.0, -90.0]  # 右后：后与右之间的弧
    assert direction_sectors(-0.1, -0.1) == [180.0, 135.0, 90.0]  # 左后：后与左之间的弧
    assert direction_sectors(float('nan'), 0.0) is None
    scan = Scan(fill=1.0)
    pts = sector_points(scan, 0.0, half_deg=30.0)
    expected = sum(1 for i in range(scan.n)
                   if mg.Sector(0.0, 30.0).contains(_scan_rad(scan, i)))
    assert len(pts) == expected > 0
    assert mg.is_valid_range(0.1, 0.02, 12.0) is True
    assert sector_points(Scan(fill=float('inf')), 0.0) == []


def test_single_valid_point_passes_zero_blocks_and_rearm_does_not_help():
    scan = Scan(fill=float('nan'))
    idx = min(range(scan.n), key=lambda i: abs(_wrap(_scan_rad(scan, i)
                                                     - math.radians(10.0))))
    scan.ranges[idx] = 1.2
    c = Clock()
    g = armed(c, scan=scan)
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g) == (0.2, 0.0, 0.0, 'auto', '')
    scan.ranges[idx] = float('nan')
    assert drive(g)[4] == 'scan_blocked'
    assert g.stopped
    g.rearm(True)
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g)[4] == 'scan_blocked'     # 仍无点：rearm 也不能走


def test_no_rclpy_import_in_pure_module():
    import sys
    assert 'rclpy' not in sys.modules
    src = open(mg.__file__, encoding='utf-8').read()
    assert 'import rclpy' not in src and 'from rclpy' not in src


# ---------------------------------------------------------------- B6 rearm_required
# 故障类锁停（超声/雷达）后传感器恢复有效、但未经显式 rearm：输出
# rearm_required（零速、锁停维持），而不是继续报已恢复的旧故障原因。
FAULT_CAUSES = ('ultrasonic_invalid', 'ultrasonic_stale',
                'scan_stale', 'scan_blocked')


@pytest.mark.parametrize('channel', mg.CHANNELS)
def test_fault_latch_recovery_without_rearm_reports_rearm_required(channel):
    c = Clock()
    g = armed(c)
    g.on_command(channel, -0.2, 0, 0)
    g.set_manual_hold(channel == 'manual')
    assert drive(g)[4] == ''
    assert drive(g, range_m=float('nan'))[4] == 'ultrasonic_invalid'   # 故障锁停
    assert g.stopped
    g.on_command(channel, -0.2, 0, 0)
    g.set_manual_hold(channel == 'manual')
    # 传感器恢复有效：锁停维持，原因 = rearm_required（不是旧故障原因）
    rec = drive(g, range_m=0.30)
    assert rec == (0.0, 0.0, 0.0, None, 'rearm_required')
    assert g.stopped
    assert g.stop_cause == 'ultrasonic_invalid'   # 真实原因仍在 stop_cause
    assert all(g.cached(ch) == [0.0, 0.0, 0.0] for ch in mg.CHANNELS)


def test_scan_stale_recovery_without_rearm_reports_rearm_required():
    c = Clock()
    g = armed(c)
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g, feed=False, age=0.0)[4] == ''   # 先确保正常
    # 雷达帧龄超限 → scan_stale 锁停（decide 走 no_demand 分支的 _reject_scan）
    g2_scan = g._scan
    g2_stamp = c() - 1.0
    g._last_scan = (g2_scan, g2_stamp)  # noqa: SLF001
    assert drive(g, feed=False, range_m=0.30)[4] in ('scan_stale', 'scan_blocked')
    assert g.stopped
    # 新鲜帧到达（传感器恢复），未 rearm → rearm_required
    g._feed()  # noqa: SLF001
    rec = drive(g, feed=False, range_m=0.30)
    assert rec == (0.0, 0.0, 0.0, None, 'rearm_required')
    assert g.stopped


def test_fault_latch_recovery_with_request_reports_rearm_pending():
    """已显式请求解停（rearm_pending）时优先报 rearm_pending。"""
    c = Clock()
    g = armed(c)
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g, range_m=float('nan'))[4] == 'ultrasonic_invalid'
    g.request_rearm()
    assert drive(g, range_m=0.30)[4] == 'rearm_pending'


def test_non_fault_latch_keeps_original_reason():
    """启动默认锁停（global_stop）不伪装成 rearm_required。"""
    c = Clock()
    g = MotionGuard(c)
    assert g.accept_license_json(lic(now=c()))[0]
    assert g.decide(ultrasonic='ok')[4] == 'global_stop'


def test_rearm_required_until_explicit_rearm():
    """rearm_required 持续输出，直到显式 rearm 解锁。"""
    c = Clock()
    g = armed(c)
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g, range_m=float('nan'))[4] == 'ultrasonic_invalid'
    for _ in range(3):
        assert drive(g, range_m=0.30)[4] == 'rearm_required'
        assert g.stopped
    assert g.rearm(True)[0]
    g.on_command('auto', 0.2, 0, 0)
    assert drive(g, range_m=0.30)[3] == 'auto'


@pytest.mark.parametrize('gap,ttl', [(0.51, 10.0), (0.2, 0.1)])
def test_expired_license_allows_new_session_but_requires_rearm(gap, ttl):
    c = Clock()
    g = armed(c)
    assert g.accept_license_json(lic(seq=9, exp=c() + ttl))[0]
    g.on_command('auto', 0.2, 0, 0)
    c.advance(gap)
    assert g.accept_license_json(lic(session=2, seq=0, now=c())) == (True, 'ok')
    assert g.license.session == 2 and g.license.seq == 0
    assert g.stopped
    assert g.cached('auto') == [0.0, 0.0, 0.0]
    assert drive(g)[:3] == (0.0, 0.0, 0.0)
    g.on_command('auto', 0.3, 0, 0)
    assert g.rearm(True)[0]
    assert drive(g)[:3] == (0.0, 0.0, 0.0)


def test_invalid_takeover_does_not_replace_expired_session():
    c = Clock()
    obj = License()
    assert obj.accept(License.parse(lic(seq=9)), c())[0]
    c.advance(2)
    assert obj.accept(License.parse(lic(session=2, exp=c()-1)), c()) == (False, 'expired')
    assert obj.session == 1 and obj.seq == 9


def test_takeover_preserves_serial_reconnect_stop():
    c = Clock()
    g = armed(c)
    g.stop('serial_reconnect')
    c.advance(2)
    assert g.accept_license_json(lic(session=2, now=c()))[0]
    assert g.stopped and g.stop_cause == 'serial_reconnect'
