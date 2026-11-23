"""MotionLicenseBridge 测试：桩消息 + 注入时钟 + 内存端口，不加载 rclpy。

覆盖：启动/重连默认锁停与显式 rearm；许可 500ms 中断后写零速帧；
无真实端口 / drive_enable 关闭 / 无回传证据时不发速度帧；11 字节基线
帧；"发出零速 / 串口写入成功 / 车辆实际停住"三分离；ROS 接线用桩节点。
"""
import json
import math
import sys
import types

import pytest

from c50c_uart3.chassis_cmd import guard_bridge as gb
from c50c_uart3.chassis_cmd.baseline import parse_status_frame
from c50c_uart3.chassis_cmd.serial_port import FdPort, FakeFdPort


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


class Range:
    def __init__(self, r):
        self.range = r
        self.max_range = 4.5


class Header:
    def __init__(self, stamp):
        self.stamp = stamp


class Stamp:
    def __init__(self, sec, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class Scan:
    def __init__(self, fill=1.0, n=360):
        self.angle_min = 0.0
        self.angle_increment = 2.0 * math.pi / n
        self.range_min = 0.02
        self.range_max = 12.0
        self.ranges = [fill] * n
        self.header = Header(Stamp(0))

    def clear(self, center_deg, half_deg=35.0):
        for i in range(len(self.ranges)):
            a = (self.angle_min + i * self.angle_increment) % (2 * math.pi)
            d = abs(((math.radians(center_deg) - a + math.pi) % (2 * math.pi)) - math.pi)
            if d <= math.radians(half_deg):
                self.ranges[i] = float('nan')


class FakeRealPort:
    """满足 FdPort 契约的"真实"端口替身（只用于测试判定分支）。"""

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


def lic_msg(session=1, seq=0, exp=None, now=100.0, ttl=1.0):
    return Msg(data=json.dumps({'session': session, 'seq': seq,
                               'expires_mono': (now + ttl) if exp is None else exp}))


def twist(vx, vz, vy=0.0):
    return Msg(linear=Msg(x=vx, y=vy), angular=Msg(z=vz))


def make(clock, *, port=None, drive_enable=True, status=True, scan=True,
         ultra=0.30):
    b = gb.MotionLicenseBridge(port=port, clock=clock,
                               wall_clock=lambda: clock() + 900.0,
                               drive_enable=drive_enable)
    b.authorize(True)
    if scan:
        sc = Scan()
        sc.header = Header(Stamp(clock() + 900.0))
        b.on_scan(sc)
    if ultra is not None:
        b.on_ultrasonic(Range(ultra))
    if status:
        b.on_status({'velocity_mm_s': 0, 'vy_mm_s': 0,
                     'yaw_rate_mrad_s': 0, 'pwm_counts': (0, 0, 0, 0)})
    return b


def online_and_armed(b, clock, seq=0):
    assert b.on_license(lic_msg(seq=seq, now=clock()))
    b.on_rearm_request(Msg(data=True))
    # [2026-11-18] on_rearm_request 已鉴权时会就地 auto-rearm（真机缺口修复）；
    # 此时锁停已解除，只有请求仍登记（未消费）时才补一次显式 rearm。
    if b.guard.stopped and b._rearm_requested:
        ok, _ = b.rearm()
        assert ok
    return b


# ---------------------------------------------------------------- 启动/重连
def test_startup_default_latched_and_tick_writes_zero_frame():
    c = Clock()
    port = FakeRealPort()
    b = make(c, port=port)
    b.on_command('auto', twist(0.2, 0.0))
    rec = b.tick()
    assert rec.frame_kind == 'zero' and rec.zero_issued is True
    assert rec.decision[4] == 'global_stop'
    assert rec.frame == bytes([0x7B, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0x7B, 0x7D])
    assert len(rec.frame) == 11 and rec.frame[2] == 0     # 基线帧：frame[2]=0
    assert rec.serial_write_ok is True
    assert b.guard.stopped


def test_serial_reconnect_relatches_and_drops_status_evidence():
    c = Clock()
    b = online_and_armed(make(c, port=FakeRealPort()), c)
    b.on_command('auto', twist(0.2, 0.0))
    assert b.tick().frame_kind == 'speed'
    b.on_serial_reconnected()
    assert b.guard.stopped and b.guard.stop_cause == 'serial_reconnect'
    rec = b.tick()
    assert rec.frame_kind == 'zero' and rec.decision[4] == 'serial_reconnect'
    assert b.vehicle_stopped() is None      # 回传证据已丢弃


def test_rearm_without_explicit_request_or_session_is_refused():
    c = Clock()
    b = make(c, port=FakeRealPort())
    b.on_license(lic_msg(now=c()))
    assert b.rearm() == (False, 'preconditions')
    b.authorize(False)
    b.on_rearm_request(Msg(data=True))     # 有请求但没有会话鉴权
    assert b.rearm() == (False, 'preconditions')
    assert b.tick().decision[4] == 'rearm_pending'   # 已请求、未解停
    assert b.guard.stopped


# ---------------------------------------------------------------- 许可
def test_license_gap_over_500ms_then_zero_frames_for_all_sources():
    c = Clock()
    port = FakeRealPort()
    b = online_and_armed(make(c, port=port), c)
    for ch in ('manual', 'nav', 'auto', 'demo'):
        b.on_command(ch, twist(0.1, 0.0))
    for seq in (1, 2, 3, 4, 5):
        c.advance(0.05)
        b.on_license(lic_msg(seq=seq, now=c() - 0.05))
        assert b.tick().frame_kind == 'speed'
    c.advance(0.55)                      # 不再续期，超过 500 ms
    rec = b.tick()
    assert rec.frame_kind == 'zero'
    assert rec.decision[4] == 'license_expired'
    assert rec.frame == port.frames[-1]
    b.on_command('auto', twist(0.3, 0.0))
    assert b.tick().decision[4] == 'license_expired'
    assert b.guard.cached('auto') == [0.0, 0.0, 0.0]


def test_license_replay_and_old_session_and_expiry_rejected():
    c = Clock()
    b = make(c, port=FakeRealPort())
    assert b.on_license(lic_msg(seq=5, now=c()))
    assert not b.on_license(lic_msg(seq=5, now=c()))
    assert not b.on_license(lic_msg(session=2, seq=9, now=c()))
    assert not b.on_license(lic_msg(seq=6, exp=c() - 1))
    snap = b.guard.snapshot()['license']
    assert snap['seq'] == 5 and snap['session'] == 1


# ---------------------------------------------------------------- 三分离
def test_three_results_separated_no_status_evidence_means_zero_frame_not_speed():
    c = Clock()
    port = FakeRealPort()
    b = online_and_armed(make(c, port=port, status=False), c)
    b.on_command('auto', twist(0.2, 0.0))
    rec = b.tick()
    assert rec.frame_kind == 'zero'
    assert rec.zero_reason == 'no_status_evidence'
    assert rec.zero_issued is True
    assert rec.serial_write_ok is True
    assert rec.vehicle_stopped is None       # 无回传：不可证明
    sep = b.last_zero_write_separately()
    assert set(sep) == {'zero_issued', 'serial_write_ok', 'vehicle_stopped'}
    assert b.counters['vehicle_stopped_unproven'] == 1


def test_zero_issued_but_write_failed_and_vehicle_moving_are_three_results():
    c = Clock()
    port = FakeRealPort()
    b = online_and_armed(make(c, port=port, status=False), c)
    port.fail_after = 0                     # 本拍写 0 字节
    rec = b.tick()
    assert rec.zero_issued is True          # 发出零速
    assert rec.serial_write_ok is False     # 串口写入失败
    assert rec.written == 0
    assert rec.vehicle_stopped is None      # 车辆状态不可证
    port.fail_after = None                  # 下一拍写入恢复正常
    b.on_status({'velocity_mm_s': 120, 'vy_mm_s': 0,
                 'yaw_rate_mrad_s': 0, 'pwm_counts': (300, 300, 300, 300)})
    assert b.vehicle_stopped() is False
    rec2 = b.tick()
    assert rec2.zero_issued is True
    assert rec2.serial_write_ok is True
    assert rec2.vehicle_stopped is False    # 写成功 ≠ 停住


def test_drive_disable_writes_nothing_but_records_decision():
    c = Clock()
    port = FakeRealPort()
    b = online_and_armed(make(c, port=port, drive_enable=False), c)
    b.on_command('auto', twist(0.2, 0.0))
    rec = b.tick()
    assert rec.frame_kind == 'none' and port.frames == []
    assert rec.serial_write_ok is None and rec.zero_issued is False


def test_fake_port_is_not_a_real_serial_and_nothing_is_written():
    c = Clock()
    fake = FakeFdPort()
    assert fake.is_real is False and FdPort.is_real is True
    b = online_and_armed(make(c, port=fake), c)
    b.on_command('auto', twist(0.2, 0.0))
    rec = b.tick()
    assert rec.frame_kind == 'none' and fake.stream == b''
    assert b.counters['blocked_no_port'] == 1


def test_speed_frame_uses_baseline_encoder_with_expected_bytes():
    c = Clock()
    port = FakeRealPort()
    b = online_and_armed(make(c, port=port), c)
    b.on_command('auto', twist(0.2, -0.5))
    rec = b.tick()
    assert rec.frame_kind == 'speed'
    assert rec.frame[:3] == bytes([0x7B, 0x00, 0x00])
    # vx=200 (0x00C8), vy=0, vz=-500 (int16 0xFE0C)
    assert rec.frame[3:9] == bytes([0x00, 0xC8, 0x00, 0x00, 0xFE, 0x0C])
    assert rec.frame[9] == (0x7B ^ 0x00 ^ 0xC8 ^ 0xFE ^ 0x0C)
    assert rec.frame[10] == 0x7D
    assert rec.serial_write_ok is True and rec.vehicle_stopped is True


# ---------------------------------------------------------------- 传感器
# [2026-11-18 用户定案] 4.5/0.50/0.62 = 远处无约束，不再锁车；
# 坏数据(NaN)仍 ultrasonic_invalid 锁停。改名并拆参数。
@pytest.mark.parametrize('ultra', [float('nan')])
def test_ultrasonic_bad_reading_writes_zero_and_latches(ultra):
    c = Clock()
    port = FakeRealPort()
    b = online_and_armed(make(c, port=port), c)
    b.on_command('auto', twist(0.2, 0.0))
    assert b.tick().frame_kind == 'speed'
    b.on_ultrasonic(Range(ultra))
    rec = b.tick()
    assert rec.frame_kind == 'zero' and rec.decision[4] == 'ultrasonic_invalid'
    assert b.guard.stopped
    # [2026-11-18] 已鉴权：请求就地 auto-rearm；同命令在，下一拍恢复速度
    b.on_rearm_request(Msg(data=True))
    b.on_ultrasonic(Range(0.30))
    b.on_command('auto', twist(0.2, 0.0))
    assert b.tick().frame_kind == 'speed'


def fresh_scan_like(b):
    """与 b 上一次雷达帧同构造的新帧（stamp 全零 → 只校验新鲜度）。"""
    return Scan()


def test_ultrasonic_stop_stream_over_300ms_zero_frames():
    c = Clock()
    b = online_and_armed(make(c, port=FakeRealPort()), c)
    b.on_command('auto', twist(0.2, 0.0))
    c.advance(0.31)                 # 超声消息龄超限；雷达同拍有新鲜帧
    rec = b.tick(scan=fresh_scan_like(b))
    assert rec.frame_kind == 'zero' and rec.decision[4] == 'ultrasonic_stale'


def test_scan_stale_by_header_stamp_latches():
    c = Clock()
    b = online_and_armed(make(c, port=FakeRealPort()), c)
    b.on_command('auto', twist(0.2, 0.0))
    sc = Scan()
    # 帧 stamp 比墙钟落后 0.4 s：到达再新也按 stamp 计龄
    sc.header = Header(Stamp(c() + 900.0 - 0.4))
    rec = b.tick(ultrasonic=Range(0.30), scan=sc)
    assert rec.frame_kind == 'zero' and rec.decision[4] == 'scan_stale'


def test_scan_direction_rear_blocked_changes_outcome_for_backward_command():
    c = Clock()
    b = online_and_armed(make(c, port=FakeRealPort()), c)
    sc = Scan()
    sc.clear(180.0)
    sc.header = Header(Stamp(c() + 900.0))
    b.on_scan(sc)
    b.on_command('auto', twist(0.2, 0.0))       # 前进不受后方空洞影响
    assert b.tick().frame_kind == 'speed'
    b.on_command('auto', twist(-0.2, 0.0))      # 后退：后扇区无点
    rec = b.tick()
    assert rec.frame_kind == 'zero' and rec.decision[4] == 'scan_blocked'


# ---------------------------------------------------------------- ROS 接线
class StubNode:
    def __init__(self):
        self.subs = []

    def create_subscription(self, msg_type, topic, cb, qos_depth):
        sub = Msg(topic=topic, cb=cb, msg_type=msg_type, qos=qos_depth)
        self.subs.append(sub)
        return sub


def test_attach_creates_subscriptions_via_stub_modules(monkeypatch):
    """attach 只做延迟导入：用桩模块冒充 ROS 消息包，验证订阅话题与回调。"""
    src = open(gb.__file__, encoding='utf-8').read()
    assert '\nimport rclpy' not in src      # rclpy 只在 attach 内延迟导入
    pure = open(gb.__file__.replace('chassis_cmd/guard_bridge.py',
                                    'motion_guard.py'), encoding='utf-8').read()
    assert 'import rclpy' not in pure
    made = {}

    def stub(modname, attr, cls):
        mod = sys.modules.get(modname) or types.ModuleType(modname)
        setattr(mod, attr, cls)
        monkeypatch.setitem(sys.modules, modname, mod)
        made[attr] = cls

    stub('std_msgs.msg', 'String', type('String', (Msg,), {}))
    stub('std_msgs.msg', 'Bool', type('Bool', (Msg,), {}))
    stub('sensor_msgs.msg', 'Range', type('Range', (Msg,), {}))
    stub('sensor_msgs.msg', 'LaserScan', type('LaserScan', (Msg,), {}))
    stub('geometry_msgs.msg', 'Twist', type('Twist', (Msg,), {}))
    qos = types.ModuleType('rclpy.qos')
    qos.qos_profile_sensor_data = object()
    monkeypatch.setitem(sys.modules, 'rclpy.qos', qos)
    node = StubNode()
    b = gb.MotionLicenseBridge(port=None, clock=lambda: 0.0)
    b.attach(node, command_topics={'manual': '/manual_cmd_vel',
                                   'auto': '/auto_cmd_vel'})
    topics = [s.topic for s in node.subs]
    assert topics[:4] == ['motion_license', 'ultrasonic', 'scan',
                          'rearm_request']
    assert '/manual_cmd_vel' in topics and '/auto_cmd_vel' in topics
    assert node.subs[2].qos is qos.qos_profile_sensor_data
    assert all(isinstance(s.qos, int) for s in node.subs if s.topic != 'scan')


@pytest.mark.parametrize('authorize_first', [True, False])
def test_rearm_authorize_arrival_orders(authorize_first):
    b = gb.MotionLicenseBridge(port=None, clock=Clock())
    if authorize_first:
        b.authorize(True)
        b.on_rearm_request(Msg(data=True))
    else:
        b.on_rearm_request(Msg(data=True))
        assert b.guard.stopped
        b.authorize(True)
    assert not b.guard.stopped
    assert b.guard.rearms == 1
    b.authorize(True)
    assert b.guard.rearms == 1


def test_authorize_revoke_discards_pending_rearm():
    b = gb.MotionLicenseBridge(port=None, clock=Clock())
    b.on_rearm_request(Msg(data=True))
    b.authorize(False)
    b.authorize(True)
    assert b.guard.stopped and b.guard.rearms == 0
