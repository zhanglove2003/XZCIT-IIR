"""guard 链路测试（本轮新增）：纯逻辑 + 桥，注入时钟、桩消息，无 ROS。

对照验收点逐条复现（含上轮复核遗留问题）：

1. 四路仲裁、锁停清缓存、普通松键不锁停；
2. 许可：重放 / 旧代次 / 过期 expires_mono 被拒；500 ms 续期中断后四路零速
   且清缓存；不存在任何自动解停（续期也不放行，必须显式 rearm）；
3. 启动 / 串口重连默认锁停，只有显式 rearm 且前提满足才解停；堵住
   "先 request_rearm/rearm 再续期"绕过前提；rearm 请求一次性；
4. 超声 max_range（无回波）与 >= 0.50 m 视为无效并停车；雷达按
   header.stamp 帧龄 > 0.3 s 视为陈旧；恢复后仍需 rearm；传感器无证据时
   默认停车；
5. 方向检查覆盖 前 / 后 / 左转 / 右转 / 扫过弧；
6. 写入层三分离：zero_issued / serial_write_ok / vehicle_stopped。
"""
import json
import math
import sys
import types

import pytest

from c50c_uart3 import motion_guard as mg
from c50c_uart3.chassis_cmd import guard_bridge as gb
from c50c_uart3.chassis_cmd.baseline import encode_baseline_zero_frame
from c50c_uart3.chassis_cmd.serial_port import FdPort, FakeFdPort


# ---------------------------------------------------------------- 桩
class Clock:
    def __init__(self, t=100.0):
        self.t = float(t)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)
        return self.t


class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Stamp:
    def __init__(self, sec, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class Scan:
    """LD19 风格 628 束、0.01 rad 步长的整圈 LaserScan 桩。"""

    def __init__(self, fill=5.0, n=628):
        self.n = n
        self.angle_min = 0.0
        self.angle_increment = 2.0 * math.pi / n
        self.angle_max = self.angle_min + self.angle_increment * (n - 1)
        self.range_min = 0.02
        self.range_max = 12.0
        self.ranges = [fill] * n
        self.header = Msg(stamp=Stamp(0))

    def angle_deg(self, i):
        return math.degrees((self.angle_min + i * self.angle_increment + math.pi)
                            % (2.0 * math.pi) - math.pi)

    def member_indices(self, center_deg, half_deg=30.0):
        """与 motion_guard/geometry 同一判定（Sector.contains）的束索引。"""
        sec = mg.Sector(center_deg, half_deg)
        return [i for i in range(self.n)
                if sec.contains(math.radians(self.angle_deg(i)))]

    def clear(self, center_deg, half_deg=30.0, value=float('nan')):
        for i in self.member_indices(center_deg, half_deg):
            self.ranges[i] = value

    def mark_valid(self, center_deg, half_deg=30.0, value=5.0):
        for i in self.member_indices(center_deg, half_deg):
            self.ranges[i] = value


class FakeRealPort:
    """满足 FdPort 写契约的"真实端口"替身；fail_after=N 表示第 N+1 次起写 0 字节。"""
    is_real = True
    kind = 'test-real'

    def __init__(self, fail_after=None):
        self.frames = []
        self.fail_after = fail_after

    def write_bytes(self, data):
        self.frames.append(bytes(data))
        if self.fail_after is not None and len(self.frames) > self.fail_after:
            return 0
        return len(data)


WALL_OFFSET = 900.0     # 墙钟 = 单调 + 900 s


def lic(session=1, seq=0, exp=None, now=100.0, ttl=5.0):
    payload = {'session': session, 'seq': seq,
               'expires_mono': (now + ttl) if exp is None else exp}
    return json.dumps(payload)


def twist(vx, vz, vy=0.0):
    return Msg(linear=Msg(x=vx, y=vy), angular=Msg(z=vz))


def bridge(clock, *, port=None, drive_enable=True, status=True, scan=True,
           ultra=0.30, authorize=True):
    b = gb.MotionLicenseBridge(port=port, clock=clock,
                               wall_clock=lambda: clock() + WALL_OFFSET,
                               drive_enable=drive_enable)
    if authorize:
        b.authorize(True)
    b.on_ultrasonic(Msg(range=ultra)) if ultra is not None else None
    if scan:
        b.on_scan(fresh_scan(clock))
    if status:
        b.on_status({'velocity_mm_s': 0, 'vy_mm_s': 0,
                     'yaw_rate_mrad_s': 0, 'pwm_counts': (0, 0, 0, 0)})
    return b


def fresh_scan(clock, lag=0.0):
    sc = Scan()
    t = clock() + WALL_OFFSET - lag
    sc.header = Msg(stamp=Stamp(int(t), int(round((t - int(t)) * 1e9)) % 10 ** 9))
    return sc


def armed(b, clock, seq_start=0):
    assert b.on_license(Msg(data=lic(seq=seq_start, now=clock())))
    b.on_rearm_request(Msg(data=True))
    # [2026-11-18] auto-rearm 已在请求内执行；仅当仍锁停且请求在时补 rearm。
    if b.guard.stopped and b._rearm_requested:
        assert b.rearm() == (True, 'ok')
    return b


def pure_armed(clock, scan=None):
    """纯逻辑：许可 + 显式 rearm + 新鲜雷达。"""
    g = mg.MotionGuard(clock)
    assert g.accept_license_json(lic(now=clock()))[0]
    assert g.rearm(True)[0]
    g.on_scan(scan if scan is not None else Scan(), clock())
    return g


def pure_drive(g, range_m=0.30, age=0.0, clock=None, refresh_scan=True):
    if refresh_scan and clock is not None:
        g.on_scan(Scan() if g._last_scan[0] is None else g._last_scan[0], clock())
    return g.decide(ultrasonic=mg.ultrasonic_state(range_m, age)[0])


CH = mg.CHANNELS


# ---------------------------------------------------------------- 1 仲裁
def test_p1_four_way_arbitration_priority_and_hold_release():
    c = Clock()
    g = pure_armed(c)
    for ch, v in (('demo', 0.1), ('auto', 0.2), ('nav', 0.25)):
        g.on_command(ch, v, 0, 0)
        assert pure_drive(g, clock=c)[3] == ch
    g.on_command('manual', 0.3, 0, 0)
    # manual 未按下（hold=False）不占优先
    assert pure_drive(g, clock=c)[3] == 'nav'
    g.set_manual_hold(True)
    assert pure_drive(g, clock=c) == (0.3, 0.0, 0.0, 'manual', '')
    g.set_manual_hold(False)                       # 普通松键
    out = pure_drive(g, clock=c)
    assert out[3] == 'nav' and out[4] == ''
    assert not g.stopped and g.stop_cause == ''    # 不触发全局锁停


def test_p1_lock_clears_all_four_caches():
    c = Clock()
    g = pure_armed(c)
    for ch, v in (('manual', 0.1), ('nav', 0.2), ('auto', 0.3), ('demo', 0.4)):
        g.on_command(ch, v, 0, 0)
    g.set_manual_hold(True)
    g.stop()
    out = pure_drive(g, clock=c)
    assert out == (0.0, 0.0, 0.0, None, 'global_stop')
    assert all(g.cached(ch) == [0.0, 0.0, 0.0] for ch in CH)
    assert g.manual_hold is True      # 松键与锁停是两个事件，不被锁停改写


def test_p1_stale_command_is_not_executed():
    c = Clock()
    g = pure_armed(c)
    g.on_command('auto', 0.3, 0, 0)
    for _ in range(12):                            # 1.2 s，许可/雷达每 0.1 s 更新
        c.advance(0.1)
        g.accept_license_json(lic(seq=g.license.seq + 1, now=c()))
        g.on_scan(Scan(), c())
    assert g.decide(ultrasonic='ok') == (0.0, 0.0, 0.0, None, 'no_demand')


# ---------------------------------------------------------------- 2 许可
def test_p2_replay_old_session_and_expired_rejected():
    c = Clock()
    g = mg.MotionGuard(c)
    assert g.accept_license_json(lic(seq=5, now=c()))[0]
    assert g.accept_license_json(lic(seq=5, now=c())) == (False, 'seq_replay')
    assert g.accept_license_json(lic(seq=4, now=c())) == (False, 'seq_replay')
    assert g.accept_license_json(lic(session=0, seq=7, now=c())) \
        == (False, 'session_invalid')
    assert g.accept_license_json(lic(session=2, seq=8, now=c())) \
        == (False, 'session_mismatch')
    assert g.accept_license_json(
        lic(seq=6, exp=c() - mg.LICENSE_EXPIRY_GRACE_S - 0.01, now=c())) \
        == (False, 'expired')
    snap = g.license.snapshot()
    assert (snap['session'], snap['seq']) == (1, 5)   # 被拒不推进


def test_p2_license_expires_between_renewals_latches_even_if_arrival_is_recent():
    """expires_mono 一过宽限就锁停：不因"刚到 10 ms"而放行已过期的许可。"""
    c = Clock()
    port = FakeRealPort()
    b = armed(bridge(c, port=port), c)
    b.on_command('auto', twist(0.2, 0.0))
    assert b.tick().frame_kind == 'speed'
    assert b.on_license(Msg(data=lic(seq=1, exp=c() + 0.02, now=c())))
    c.advance(0.10)                       # 距接收 0.10 s < 0.5 s，但 expires 已过
    rec = b.tick()
    assert rec.decision[4] == 'license_expired' and rec.frame_kind == 'zero'
    assert b.guard.stopped


def test_p2_500ms_renewal_gap_zeroes_all_four_and_clears_cache():
    c = Clock()
    port = FakeRealPort()
    b = armed(bridge(c, port=port), c)
    for ch in CH:                         # manual 同时 hold 住
        b.on_command(ch, twist(0.1, 0.0))
    for seq in (1, 2, 3):
        c.advance(0.05)
        b.on_status({'velocity_mm_s': 0, 'pwm_counts': (0, 0)})
        assert b.on_license(Msg(data=lic(seq=seq, now=c())))
        assert b.tick().frame_kind == 'speed'
    b.on_ultrasonic(Msg(range=0.30))
    b.on_scan(fresh_scan(c))
    c.advance(0.24)
    assert b.on_license(Msg(data=lic(seq=4, now=c())))   # 0.24 s 内续期：仍放行
    assert b.tick().frame_kind == 'speed'
    c.advance(0.51)                       # 0.51 s 无续期（且许可已过宽限）
    rec = b.tick()
    assert rec.frame_kind == 'zero' and rec.decision[4] == 'license_expired'
    assert rec.frame == encode_baseline_zero_frame()
    assert all(b.guard.cached(ch) == [0.0, 0.0, 0.0] for ch in CH)
    for ch in CH:                         # 锁停期间新命令也不缓存
        b.on_command(ch, twist(0.2, 0.0))
    assert b.tick().decision[4] == 'license_expired'
    assert all(b.guard.cached(ch) == [0.0, 0.0, 0.0] for ch in CH)
    # 续期恢复许可本身，但锁停不解：仍零速，直到显式 rearm
    assert b.on_license(Msg(data=lic(seq=5, now=c())))
    assert b.tick().frame_kind == 'zero'
    assert b.guard.stopped


def test_p2_no_automatic_resume_on_renewal_after_any_latch():
    c = Clock()
    g = pure_armed(c)
    # 先让手上的续期在宽限外过期，且新许可 seq 必须更大、也未过期
    assert g.accept_license_json(lic(seq=1, exp=c() + 0.05))[0]
    c.advance(0.60)
    assert g.decide(ultrasonic='ok')[4] == 'license_expired'
    for seq in range(2, 5):
        assert g.accept_license_json(lic(seq=seq, now=c()))[0]
        assert g.decide(ultrasonic='ok') == (0.0, 0.0, 0.0, None, 'license_expired')
    assert g.stopped


# ---------------------------------------------------------------- 3 rearm
def test_p3_startup_default_latched_until_explicit_rearm():
    c = Clock()
    port = FakeRealPort()
    b = bridge(c, port=port)
    b.on_command('auto', twist(0.2, 0.0))
    rec = b.tick()
    assert rec.decision[4] == 'global_stop' and rec.frame_kind == 'zero'
    assert b.on_license(Msg(data=lic(seq=0, now=c())))
    assert b.tick().decision[4] == 'global_stop'     # 许可不能解除启动锁停
    assert b.rearm() == (False, 'preconditions')     # 没有请求
    b.on_rearm_request(Msg(data=True))               # 有请求、有鉴权（bridge 默认未鉴权？见下）
    b.authorize(False)                               # 显式撤销鉴权 → 拒绝并清请求
    assert b.rearm() == (False, 'preconditions')
    b.authorize(True)
    b.on_rearm_request(Msg(data=True))
    # [2026-11-18] auto-rearm 已在请求内就地解停；仅当仍锁停且请求在时补 rearm。
    if b.guard.stopped and b._rearm_requested:
        assert b.rearm() == (True, 'ok')
    b.on_command('auto', twist(0.2, 0.0))
    assert b.tick().frame_kind == 'speed'


def test_p3_serial_reconnect_relatches_and_requires_new_request():
    c = Clock()
    port = FakeRealPort()
    b = armed(bridge(c, port=port), c)
    b.on_command('auto', twist(0.2, 0.0))
    assert b.tick().frame_kind == 'speed'
    b.on_serial_reconnected()                        # 重开串口应调本方法
    assert b.guard.stop_cause == 'serial_reconnect'
    rec = b.tick()
    assert rec.frame_kind == 'zero' and rec.stop_cause == 'serial_reconnect'
    assert rec.vehicle_stopped is None               # 回传证据已作废
    assert b.on_license(Msg(data=lic(seq=1, now=c())))
    b.on_ultrasonic(Msg(range=0.3))
    b.on_scan(fresh_scan(c))
    b.on_status({'velocity_mm_s': 0, 'pwm_counts': (0, 0)})
    b.on_command('auto', twist(0.2, 0.0))
    assert b.tick().frame_kind == 'zero'             # 旧请求不复活
    assert b.rearm() == (False, 'preconditions')
    b.on_rearm_request(Msg(data=True))               # 新显式请求 → 就地 auto-rearm
    assert not b.guard.stopped
    b.on_command('auto', twist(0.2, 0.0))
    assert b.tick().frame_kind == 'speed'


def test_p3_rearm_before_license_renewal_is_refused_by_next_tick():
    """上轮复核 R2-3 / R3 遗留：先 rearm 再续期不得放行。"""
    c = Clock()
    port = FakeRealPort()
    b = armed(bridge(c, port=port), c)
    b.on_command('auto', twist(0.2, 0.0))
    assert b.tick().frame_kind == 'speed'
    c.advance(0.51)                                  # 许可续期中断（> 500 ms）
    b.on_status({'velocity_mm_s': 0, 'pwm_counts': (0, 0)})
    b.on_ultrasonic(Msg(range=0.30))
    b.on_scan(fresh_scan(c))
    assert b.tick().decision[4] == 'license_expired'
    b.on_rearm_request(Msg(data=True))
    # [2026-11-18] 已鉴权：请求就地 auto-rearm（解停）
    assert not b.guard.stopped
    rec = b.tick()                                   # 但许可未续期：立即重锁
    assert rec.decision[4] == 'license_expired' and rec.frame_kind == 'zero'
    assert b.guard.stopped and b.guard.stop_cause == 'license_expired'
    ok = b.on_license(Msg(data=lic(seq=1, now=c())))     # 续期在同一锁停内到达
    assert ok
    assert b.guard.stopped                           # 续期不自动解停
    b.on_rearm_request(Msg(data=True))               # 新显式请求 → 就地 auto-rearm
    assert not b.guard.stopped
    b.on_command('auto', twist(0.2, 0.0))
    _refresh(c, b)
    rec = b.tick()                                   # 许可已续期 + 已解停：速度帧恢复
    assert rec.frame_kind == 'speed'
    b.on_status({'velocity_mm_s': 0, 'pwm_counts': (0, 0)})
    b.on_command('auto', twist(0.2, 0.0))
    assert b.tick().frame_kind == 'speed'


def _refresh(c, b):
    """刷新许可/超声/雷达/回传（不动命令），使本拍只看命令缓存。"""
    assert b.on_license(Msg(data=lic(seq=b.guard.license.seq + 1, now=c())))
    b.on_ultrasonic(Msg(range=0.30))
    b.on_scan(fresh_scan(c))
    b.on_status({'velocity_mm_s': 0, 'pwm_counts': (0, 0)})


def test_p3_rearm_request_is_one_shot():
    c = Clock()
    b = bridge(c)
    b.on_rearm_request(Msg(data=True))
    # [2026-11-18] 已鉴权：请求就地 auto-rearm 并消费（一次性语义不变）
    assert b.rearm() == (False, 'preconditions')     # 请求已被 auto-rearm 用掉
    assert not b.guard.stopped
    b.on_serial_reconnected()                        # 重连重新锁停
    assert b.rearm() == (False, 'preconditions')     # 重连也作废请求


def test_p3_rearm_request_alone_never_unlocks():
    c = Clock()
    g = mg.MotionGuard(c)
    g.on_scan(Scan(), c())
    for _ in range(3):
        g.request_rearm()
        assert g.stopped
        assert g.decide(ultrasonic='ok') == (0.0, 0.0, 0.0, None, 'rearm_pending')
    g.stop('serial_reconnect')
    g.request_rearm()
    assert g.stopped and g.decide(ultrasonic='ok')[4] == 'rearm_pending'
    assert g.rearm() == (False, 'preconditions')     # 默认拒绝
    assert g.stopped
    assert g.rearm(True)[0] and not g.stopped


# ---------------------------------------------------------------- 4 超声/雷达
@pytest.mark.parametrize('r,state', [
    # [2026-11-18 用户定案] 无回波(max_range)/超释放门限 = 远处无约束，
    # 不再 invalid 锁车；坏数据(NaN/0/None)仍 invalid。
    (4.5, 'ok'),             # uart3_node.py:537 把 0xFFFF 映射为 max_range → 远处
    (0.50, 'ok'),            # 恰好 0.50 m = 远处无约束（不再锁车）
    (3.99, 'ok'),
    (float('nan'), 'invalid'),
    (0.0, 'invalid'),
    (None, 'invalid'),
    (0.499, 'ok'),
    (0.05, 'ok'),
])
def test_p4_ultrasonic_reading_table(r, state):
    assert mg.ultrasonic_state(r, 0.0)[0] == state


@pytest.mark.parametrize('ch', CH)
@pytest.mark.parametrize('bad', [(float('nan'), 0.0, 'ultrasonic_invalid'),
                                 (0.0, 0.0, 'ultrasonic_invalid'),
                                 (0.30, 0.301, 'ultrasonic_stale')])
def test_p4_ultrasonic_bad_stops_manual_and_auto_and_needs_rearm(ch, bad):
    c = Clock()
    port = FakeRealPort()
    b = armed(bridge(c, port=port), c)
    b.on_command(ch, twist(0.2, 0.0))
    if ch == 'manual':
        b.guard.set_manual_hold(True)
    assert b.tick().frame_kind == 'speed'
    r, age, reason = bad
    b.on_ultrasonic(Msg(range=r))
    c.advance(age)
    rec = b.tick()
    assert rec.frame_kind == 'zero' and rec.decision[4] == reason
    assert rec.zero_issued is True and rec.serial_write_ok is True
    # 写成功 ≠ 车停住：这一拍只有"速度/PWM 全零"一次回传，就是唯一的停住证据
    assert rec.vehicle_stopped is True
    # 读数恢复也不自动放行
    b.on_ultrasonic(Msg(range=0.30))
    b.on_status({'velocity_mm_s': 0, 'pwm_counts': (0, 0, 0, 0)})
    b.on_command(ch, twist(0.2, 0.0))
    # [B6] 传感器恢复但未经显式 rearm：报 rearm_required（真实原因保留在 stop_cause）
    assert b.tick().decision[4] == 'rearm_required'
    assert b.guard.stop_cause in ('ultrasonic_stale', 'ultrasonic_invalid')
    # [2026-11-18] 已鉴权时 on_rearm_request 就地 auto-rearm：锁停解除，
    # 下一拍（同命令仍在）直接恢复速度帧。
    b.on_rearm_request(Msg(data=True))
    b.on_command(ch, twist(0.2, 0.0))
    _refresh(c, b)                          # 重置回传龄/雷达龄：本拍只看有效读数
    assert b.tick().frame_kind == 'speed'


def test_p4_missing_sensor_evidence_defaults_to_stop():
    g = mg.MotionGuard(Clock())
    g.accept_license_json(lic(now=100.0))
    g.rearm(True)
    g.on_command('auto', 0.2, 0, 0)
    g.on_scan(Scan(), 100.0)
    assert g.decide_step() == (0.0, 0.0, 0.0, None, 'ultrasonic_stale')
    # 桥同样按"无证据即停车"：先正常跑起，再让超声样本丢失
    c = Clock()
    port = FakeRealPort()
    b = armed(bridge(c, port=port), c)
    b.on_command('auto', twist(0.2, 0.0))
    assert b.tick(scan=fresh_scan(c)).frame_kind == 'speed'
    b._ultra = (None, None)                         # 超声样本丢失
    rec = b.tick(scan=fresh_scan(c))
    assert rec.frame_kind == 'zero' and rec.decision[4] == 'ultrasonic_stale'
    assert rec.frame == encode_baseline_zero_frame()


def test_p4_scan_stale_by_header_stamp_then_rearm_required():
    c = Clock()
    port = FakeRealPort()
    b = armed(bridge(c, port=port), c)
    b.on_command('auto', twist(0.2, 0.0))
    assert b.tick().frame_kind == 'speed'
    b.on_scan(fresh_scan(c, lag=0.299))
    assert b.tick().frame_kind == 'speed'
    b.on_scan(fresh_scan(c, lag=0.301))             # 刚到的新帧，但 stamp 落后 0.301 s
    rec = b.tick()
    assert rec.frame_kind == 'zero' and rec.decision[4] == 'scan_stale'
    assert rec.stop_cause == 'scan_stale'
    # 新帧到达 + 恢复：仍需 rearm
    b.on_scan(fresh_scan(c))
    b.on_status({'velocity_mm_s': 0, 'pwm_counts': (0, 0)})
    b.on_command('auto', twist(0.2, 0.0))
    # [B6] 新帧到达（恢复）但未 rearm：rearm_required
    assert b.tick().decision[4] == 'rearm_required'
    b.on_rearm_request(Msg(data=True))    # 就地 auto-rearm（已鉴权）
    b.on_command('auto', twist(0.2, 0.0))
    assert b.tick().frame_kind == 'speed'


def test_p4_scan_frame_age_grows_between_frames():
    c = Clock()
    g = pure_armed(c)
    g.on_command('auto', 0.2, 0, 0)
    assert g.decide(ultrasonic='ok')[3] == 'auto'
    c.advance(0.31)
    assert g.decide(ultrasonic='ok') == (0.0, 0.0, 0.0, None, 'scan_stale')


def test_p4_zero_stamp_means_no_evidence():
    assert mg.scan_stamp_to_mono(Stamp(0, 0), now_mono=5.0, now_wall=5.0) is None
    g = pure_armed(Clock())
    g.on_command('auto', 0.2, 0, 0)
    g.on_scan(Scan(), None)
    assert g.decide(ultrasonic='ok') == (0.0, 0.0, 0.0, None, 'scan_stale')


# ---------------------------------------------------------------- 5 方向
@pytest.mark.parametrize('vx,vz,blocked', [
    (0.2, 0.0, 0.0),       # 前进 → 前
    (-0.2, 0.0, 180.0),    # 后退 → 后
    (0.0, 0.5, -90.0),      # B4：vz>0 = 顺时针右转 → 右
    (0.0, -0.5, 90.0),    # vz<0 = 左转 → 左
    (0.2, 0.5, -45.0),      # 右前：前进+右转的扫过弧
    (0.2, -0.5, 45.0),    # 左前
    (-0.2, 0.5, -135.0),    # 右后
    (-0.2, -0.5, 135.0),  # 左后
])
def test_p5_direction_sector_empty_blocks(vx, vz, blocked):
    c = Clock()
    # 全 NaN 起点，只在前/后/左/右/四个 45° 方向上各放一段有效点，
    # 再清掉"本方向需要查的那个扇区"——没有任何构造余量。
    scan = Scan(fill=float('nan'))
    for center in (0.0, 180.0, -90.0, 90.0, -45.0, 45.0, -135.0, 135.0):
        scan.mark_valid(center)
    scan.clear(blocked)
    g = pure_armed(c, scan)
    g.on_command('auto', vx, 0, vz)
    assert g.decide(ultrasonic='ok') == (0.0, 0.0, 0.0, None, 'scan_blocked')
    assert g.stopped and all(g.cached(ch) == [0.0, 0.0, 0.0] for ch in CH)


@pytest.mark.parametrize('vx,vz', [
    (0.2, 0.0), (-0.2, 0.0), (0.0, 0.5), (0.0, -0.5), (0.2, 0.5), (0.2, -0.5),
    (-0.2, 0.5), (-0.2, -0.5),
])
def test_p5_unrelated_sector_empty_does_not_block(vx, vz):
    """只在本方向需要查的扇区放有效点，其余扇区全 NaN（含 -135°/180°）。"""
    c = Clock()
    scan = Scan(fill=float('nan'))
    use = mg.direction_sectors(vx, vz)
    for center in use:
        scan.mark_valid(center)
    g = pure_armed(c, scan)
    g.on_command('auto', vx, 0, vz)
    assert g.decide(ultrasonic='ok')[:4] == (vx, 0.0, vz, 'auto')


def test_p5_each_of_four_channels_gets_direction_check():
    for ch in CH:
        c = Clock()
        scan = Scan()
        scan.clear(0.0)
        g = pure_armed(c, scan)
        g.on_command(ch, 0.2, 0, 0)
        g.set_manual_hold(ch == 'manual')
        assert g.decide(ultrasonic='ok')[4] == 'scan_blocked', ch


def test_p5_all_inf_sector_counts_as_nothing_visible():
    """LD19 无回波输出 inf：geometry.is_valid_range 会把 inf 当"在量程内"。"""
    c = Clock()
    scan = Scan(fill=float('inf'))
    g = pure_armed(c, scan)
    g.on_command('auto', 0.2, 0, 0)
    assert mg.sector_points(scan, 0.0) == []
    assert g.decide(ultrasonic='ok')[4] == 'scan_blocked'
    # NaN 同理；普通读到量程上限的点算有效
    scan.ranges = [float('nan')] * scan.n
    g.on_scan(scan, c())
    g.rearm(True)
    g.on_command('auto', 0.2, 0, 0)
    assert g.decide(ultrasonic='ok')[4] == 'scan_blocked'
    scan.ranges = [12.0] * scan.n
    g.on_scan(scan, c())
    g.rearm(True)
    g.on_command('auto', 0.2, 0, 0)
    assert g.decide(ultrasonic='ok')[3] == 'auto'


def test_p5_default_is_strict_no_valid_point_blocks():
    """复核 A9：任务字面口径是默认——扇区里没有有效点就停车。"""
    scan = Scan(fill=float('nan'))
    idx = min(range(scan.n), key=lambda i: abs(scan.angle_deg(i)))
    scan.ranges[idx] = 1.0                          # 前方只有 1 个点
    assert mg.motion_check(scan, 10.0, 10.0, 0.2, 0.0)[:2] == (True, 'ok')
    scan.ranges[idx] = float('nan')                 # 前方一个有效点都没有
    assert mg.motion_check(scan, 10.0, 10.0, 0.2, 0.0)[:2] == (False, 'scan_blocked')
    assert mg.scan_check(scan, 10.0, 10.0, 0.2, 0.0)[:2] == (False, 'scan_blocked')
    # guard / 桥默认也是严格口径
    assert mg.MotionGuard(Clock()).scan_strict is True
    assert gb.MotionLicenseBridge().guard.scan_strict is True


def test_p5_lenient_coverage_mode_is_opt_in():
    scan = Scan(fill=float('nan'))
    idx = min(range(scan.n), key=lambda i: abs(scan.angle_deg(i)))
    scan.ranges[idx] = 1.0
    assert mg.motion_check(scan, 10.0, 10.0, 0.2, 0.0,
                           strict=False)[:2] == (True, 'ok')
    assert mg.motion_check(scan, 10.0, 10.0, 0.2, 0.0, strict=False,
                           min_coverage=0.5)[:2] == (False, 'scan_blocked')


def test_p5_narrow_wall_ahead_blocks_turn_and_forward_guard():
    """复核 A9 复现：前方 ±5°、0.25 m 窄墙，扇区其余 NaN → 默认必须停车。"""
    scan = Scan(fill=float('nan'))
    for i in range(scan.n):
        if abs(scan.angle_deg(i)) <= 5.0:
            scan.ranges[i] = 0.25
    assert mg.motion_check(scan, 10.0, 10.0, 0.2, 0.0)[:2] == (True, 'ok')
    scan.ranges = [float('nan')] * scan.n           # 再把窄墙去掉
    c = Clock()
    g = pure_armed(c, scan)
    g.on_command('auto', 0.2, 0, 0)
    assert g.decide(ultrasonic='ok') == (0.0, 0.0, 0.0, None, 'scan_blocked')


# ---------------------------------------------------------------- 6 三分离
def test_p6_three_results_are_independent():
    c = Clock()
    port = FakeRealPort()
    b = bridge(c, port=port, status=False)
    assert b.tick().decision[4] == 'global_stop'
    rec = b.tick()
    assert (rec.zero_issued, rec.serial_write_ok, rec.vehicle_stopped) == \
        (True, True, None)                          # 发出且写入成功，停没停不可证
    port.fail_after = len(port.frames)              # 下一拍写 0 字节
    b.on_status({'velocity_mm_s': 150, 'pwm_counts': (400, 400, 400, 400)})
    rec = b.tick()
    assert (rec.zero_issued, rec.serial_write_ok, rec.vehicle_stopped) == \
        (True, False, False)                        # 发零速、写失败、车还在动
    port.fail_after = None
    b.on_status({'velocity_mm_s': 0, 'pwm_counts': (0, 0, 0, 0)})
    rec = b.tick()
    assert (rec.zero_issued, rec.serial_write_ok, rec.vehicle_stopped) == \
        (True, True, True)
    sep = b.last_zero_write_separately()
    assert sep == {'zero_issued': True, 'serial_write_ok': True,
                   'vehicle_stopped': True}
    assert 'zero_issued=True' in b.log_line()
    assert 'serial_write_ok=True' in b.log_line()
    assert 'vehicle_stopped=True' in b.log_line()
    assert b.counters['serial_write_failed'] == 1
    assert b.counters['vehicle_stopped_unproven'] >= 1


def test_p6_speed_frame_then_write_failure_is_not_zero_issued():
    c = Clock()
    port = FakeRealPort()
    b = armed(bridge(c, port=port), c)
    b.on_command('auto', twist(0.2, 0.0))
    port.fail_after = 0
    rec = b.tick()
    assert rec.frame_kind == 'speed'
    assert (rec.zero_issued, rec.serial_write_ok, rec.vehicle_stopped) == \
        (False, False, True)


def test_p6_drive_disabled_and_fake_port_write_nothing():
    c = Clock()
    b = armed(bridge(c, port=FakeFdPort()), c)
    b.on_command('auto', twist(0.2, 0.0))
    rec = b.tick()
    assert rec.frame_kind == 'none' and rec.serial_write_ok is None
    assert rec.zero_issued is False
    assert b.port.stream == b''                     # FakeFdPort 没被写
    assert FdPort.is_real is True and FakeFdPort.is_real is False
    c2 = Clock()
    b2 = armed(bridge(c2, port=FakeRealPort(), drive_enable=False), c2)
    assert b2.tick().frame_kind == 'none'


def test_p6_stop_cause_mapping_is_honest():
    for reason in ('global_stop', 'serial_reconnect', 'rearm_pending',
                   'rearm_required', 'license_expired', 'license_rejected',
                   'ultrasonic_invalid', 'ultrasonic_stale', 'scan_stale',
                   'scan_blocked', 'no_status_evidence'):
        assert gb.REASON_TO_CAUSE[reason] == reason
    assert gb.REASON_TO_CAUSE['no_demand'] == ''
    for reason in mg.ZERO_REASON:
        if reason == 'none':            # 哨兵值，不是零速原因
            continue
        assert reason in gb.REASON_TO_CAUSE, reason


# ---------------------------------------------------------------- 约束
def test_no_rclpy_in_pure_module_and_bridge_side_effect_free():
    import importlib
    for name in [m for m in sys.modules if m.startswith('rclpy')]:
        del sys.modules[name]
    importlib.reload(mg)
    src = open(mg.__file__, encoding='utf-8').read()
    assert 'import rclpy' not in src and 'from rclpy' not in src
    gb_src = open(gb.__file__, encoding='utf-8').read()
    assert '\nimport rclpy' not in gb_src and '\nfrom rclpy' not in gb_src
    assert 'rclpy' not in sys.modules


def test_attach_with_stub_message_modules(monkeypatch):
    """ROS 接线：用桩模块验证话题与回调，不需要 rclpy。"""
    qos = types.ModuleType('rclpy.qos')
    qos.qos_profile_sensor_data = 10
    monkeypatch.setitem(sys.modules, 'rclpy.qos', qos)
    for name, attrs in (('std_msgs', ('String', 'Bool')),
                        ('sensor_msgs', ('Range', 'LaserScan')),
                        ('geometry_msgs', ('Twist',))):
        pkg = types.ModuleType(name)
        pkgmsg = types.ModuleType(name + '.msg')
        for a in attrs:
            setattr(pkgmsg, a, type(a, (Msg,), {}))
        pkg.msg = pkgmsg
        monkeypatch.setitem(sys.modules, name, pkg)
        monkeypatch.setitem(sys.modules, name + '.msg', pkgmsg)
    node = Msg(subs=[])
    node.create_subscription = lambda t, topic, cb, q: (
        node.subs.append((topic, cb)) or Msg())
    c = Clock()
    b = bridge(c)
    b.attach(node, command_topics={'manual': '/manual_cmd_vel',
                                   'auto': '/auto_cmd_vel'})
    topics = [t for t, _ in node.subs]
    assert topics == ['motion_license', 'ultrasonic', 'scan', 'rearm_request',
                      '/manual_cmd_vel', '/auto_cmd_vel']
    cbs = dict(node.subs)
    cbs['/manual_cmd_vel'](twist(0.3, 0.0))
    assert b.guard.manual_hold is True
    cbs['motion_license'](Msg(data=lic(seq=0, now=c())))
    assert b.guard.license.seq == 0
