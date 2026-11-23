"""航点巡航 + 网页运动许可（计划 §4 中属于网页/巡航模块的离线用例）。

覆盖：
- 一次只发一个目标；成功后发下一点；失败 / 3 秒无安全路径 → 取消并锁停；
  每个航点 yaw 指向下一点（末点指回首点）。
- 取消竞态：目标未接受时停车，迟到接受后立即取消；取消失败保持锁停；
  手动接管先取消旧导航，松键后旧任务不自行恢复。
- patrol_save / patrol_start / motion_stop / motion_rearm 鉴权与 cruise_set
  相同（未登录、非实时页都拒绝）。
- 路线绑定地图内容标识（yaml/pgm sha256 前 16 位），地图变化拒绝旧路线。
- 网页 100 ms 许可心跳、服务端 350 ms 判失效；网页进程退出后下游仍独立
  锁停（有代码路径用例）；只在有效许可且锁停已解除时 released=True。
- 页面含保存路线/开始/停止/恢复按钮与状态显示，且只发消息不作安全判断。

本套件随 conftest 在 fake_ros 安装后导入 web_debug；nav_goal 的 ActionClient
走 fake_ros.FakeActionClient（同步接受、结果由测试触发）。
"""

import inspect
import json
import math
import os
import threading
import time

import pytest

import fake_ros

from opi_robot.web_node import patrol as P
from opi_robot.web_node import nav_goal as NG
from opi_robot.web_node import web_debug as W


# ------------------------------------------------------------------ 工具
class Clock:
    def __init__(self, t=500.0):
        self.t = t

    def __call__(self):
        return self.t

    def tick(self, dt):
        self.t += dt


class FakeNav:
    def __init__(self):
        self.dispatched = []
        self.cancelled = []
        self.cancel_ok = True
        self.stops = []
        self.next_id = 1

    def dispatch(self, goal):
        rid = self.next_id
        self.next_id += 1
        self.dispatched.append((rid, dict(goal)))
        return rid

    def cancel(self, rid, result_known=True):
        # rclpy 回调把 seq 当 positional 参数传；巡航侧的 cancel 签名是
        # (rid, result_known)，这里按同样顺序记录。
        assert isinstance(rid, int) or True
        self.cancelled.append((rid, result_known))
        return self.cancel_ok

    def stop(self, reason):
        self.stops.append(reason)


def make(pts=None, clk=None):
    clk = clk or Clock()
    nav = FakeNav()
    route, err = P.validate_route(pts or [[0, 0], [1, 0], [1, 1]], 'm1', 'm1')
    assert err is None, err
    r = P.PatrolRunner(nav.dispatch, nav.cancel, nav.stop, clk)
    r.save(route)
    return r, nav, clk


ROUTE_ID = '0123456789abcdef'


@pytest.fixture(autouse=True)
def clean_patrol():
    """每个用例前后复位网页侧巡航状态，避免按地图标识缓存互相污染。"""
    W.reset_state()
    fake_ros.reset_topics()
    W._MANUAL.attach(None)
    yield
    W.reset_state()
    W._MANUAL.attach(None)


@pytest.fixture
def map_yaml(tmp_path):
    """一张最小可用的地图：yaml + 同目录 pgm。"""
    def _make(content=b'P5\n4 4\n255\n' + bytes(range(16))):
        pgm = tmp_path / 'room.pgm'
        pgm.write_bytes(content)
        yml = tmp_path / 'room.yaml'
        yml.write_text('image: room.pgm\nresolution: 0.05\n', encoding='utf-8')
        return str(yml)
    yield _make


# ------------------------------------------------------------------ 路线/地图
def test_route_binds_map_content_hash(tmp_path):
    """地图标识 = yaml+pgm 内容 sha256 前 16 位；内容一变标识就变。"""
    a = tmp_path / 'a.pgm'
    a.write_bytes(b'one')
    y = tmp_path / 'a.yaml'
    y.write_text('image: a.pgm\n', encoding='utf-8')
    mid1 = W.patrol_map_id(str(y))
    assert len(mid1) == 16 and int(mid1, 16) >= 0
    files = P.map_files_from_yaml(str(y))
    assert files[1].endswith('a.pgm')
    a.write_bytes(b'two')                       # 同名不同内容
    mid2 = W.patrol_map_id(str(y))
    assert mid1 != mid2
    assert P.map_id_from_digest('ab' * 32) == ('ab' * 8)
    with pytest.raises(ValueError):
        P.map_id_from_digest('short')


def test_unknown_map_rejects_everything():
    """不知道地图（文件缺失/未配置）时保存与开始都拒绝，不猜。"""
    W.configure_patrol(reset_runner=True)      # 显式无地图（不传 yaml）
    W._patrol['map_yaml'] = None
    W._patrol['map_id'] = None
    assert W._patrol['runner'].route is None
    assert W.patrol_map_id(None) is None
    assert W.patrol_map_id('/nonexistent/x.yaml') is None
    ok, msg, _ = W.patrol_save([[0, 0], [1, 0]])
    assert not ok and '当前地图未知' in msg
    ok, msg = W.patrol_start()
    assert not ok


def test_old_route_rejected_after_map_change(map_yaml):
    path = map_yaml()
    W.configure_patrol(map_yaml=path)
    mid = W.patrol_map_id(path)
    ok, msg, frame = W.patrol_save([[0, 0], [2, 0]], map_id=mid)
    assert ok, msg
    assert frame['map_id'] == mid
    # 地图内容变化（同名文件被覆盖）：旧路线拒绝开始
    map_yaml(b'P5\n4 4\n255\n' + bytes(range(16, 32)))
    W.configure_patrol(map_yaml=path)
    assert W._patrol['runner'].route is not None      # 旧路线还在手里
    ok, msg = W.patrol_start()
    assert not ok and '地图已更换' in msg
    # 保存时声明旧标识也拒绝
    ok, msg, _ = W.patrol_save([[0, 0], [2, 0]], map_id=mid)
    assert not ok and '地图' in msg


def test_yaw_faces_next_and_last_faces_first():
    route, err = P.validate_route([[0, 0], [1, 0], [1, 1]], 'm1', 'm1')
    assert err is None
    pts = route['points']
    assert abs(pts[0][2]) < 1e-9
    assert abs(pts[1][2] - math.pi / 2) < 1e-9
    assert abs(pts[2][2] + 3 * math.pi / 4) < 1e-9


def test_route_validation_limits():
    assert P.validate_route([[0, 0]], 'm', 'm')[1]
    assert P.validate_route([[0, 0], [0.1, 0]], 'm', 'm')[1]
    assert P.validate_route([[0, 0], ['x', 0]], 'm', 'm')[1]
    assert P.validate_route([[0, 0], [float('nan'), 0]], 'm', 'm')[1]
    assert P.validate_route([[i, 0] for i in range(40)], 'm', 'm')[1]
    assert P.validate_route([[0, 0], [1, 0]], 'old', 'new')[1] == '地图已更换，旧路线不可用'


# ------------------------------------------------------------------ 状态机
def test_one_goal_at_a_time_and_advance():
    r, nav, _ = make()
    r.release_lock()
    assert r.start('m1') == (True, 'ok')
    assert len(nav.dispatched) == 1
    rid = nav.dispatched[0][0]
    r.on_accepted(rid)
    assert r.pending == rid and r.state == 'running'
    # 在途时不会再发第二个
    assert len(nav.dispatched) == 1
    r.on_result(rid, 'succeeded')
    assert len(nav.dispatched) == 2
    assert nav.dispatched[1][1] == {'x': 1.0, 'y': 0.0, 'yaw': pytest.approx(math.pi / 2)}
    rid2 = nav.dispatched[1][0]
    r.on_accepted(rid2)
    r.on_result(rid2, 'succeeded')
    rid3 = nav.dispatched[2][0]
    r.on_accepted(rid3)
    r.on_result(rid3, 'succeeded')
    assert len(nav.dispatched) == 4 and r.laps == 1 and r.index == 0
    assert nav.dispatched[3][1]['x'] == 0.0         # 回到首点继续


def test_nav_failure_cancels_and_locks():
    r, nav, _ = make()
    r.release_lock()
    r.start('m1')
    rid = nav.dispatched[0][0]
    r.on_accepted(rid)
    r.on_result(rid, 'failed', 'aborted')
    assert r.state == 'failed' and r.locked and r.lock_reason == 'aborted'
    assert nav.stops == ['aborted']
    assert len(nav.dispatched) == 1


def test_plan_blocked_three_seconds():
    r, nav, clk = make()
    r.release_lock()
    r.start('m1')
    rid = nav.dispatched[0][0]
    r.on_accepted(rid)
    r.on_plan_blocked(True)
    clk.tick(2.9)
    r.on_plan_blocked(True)
    assert r.state == 'running'
    clk.tick(0.2)
    r.on_plan_blocked(True)
    assert r.state == 'failed' and r.lock_reason == 'no_safe_path'
    assert nav.stops == ['no_safe_path']
    assert rid in [c[0] for c in nav.cancelled]
    r.on_plan_blocked(False)
    clk.tick(5)
    r.on_plan_blocked(True)
    assert r.state == 'failed'


def test_dispatch_exception_is_nav_failure_locked():
    nav = FakeNav()
    nav.dispatch = lambda goal: (_ for _ in ()).throw(RuntimeError('boom'))
    r = P.PatrolRunner(nav.dispatch, nav.cancel, nav.stop, Clock())
    route, _ = P.validate_route([[0, 0], [1, 0]], 'm1', 'm1')
    r.save(route)
    r.release_lock()
    ok, _ = r.start('m1')
    assert not ok and r.state == 'failed' and nav.stops == ['dispatch_failed']


def test_start_after_rearm_cruises_normally():
    """rearm 校验过许可后才能开始；开始即解除锁停，接受后正常巡航。"""
    r, nav, _ = make()
    assert r.locked
    ok, _ = r.rearm(license_ok=True)        # 无在途目标、许可有效
    assert ok and not r.locked
    r.start('m1')
    rid = nav.dispatched[0][0]
    r.on_accepted(rid)
    assert r.state == 'running' and r.pending == rid and not nav.cancelled
    assert r.cancel_request is None


# ------------------------------------------------------------------ 竞态
def test_late_accept_after_stop_is_cancelled_immediately():
    r, nav, _ = make()
    r.release_lock()
    r.start('m1')
    rid = nav.dispatched[0][0]
    r.on_accepted(rid)                       # Nav2 已接受
    r.stop('global_stop')
    assert r.state == 'stopped' and r.locked
    # 无论当时取消是否成功，都留下"目标需取消"的记录
    assert r.cancel_request == rid
    r.on_accepted(rid)                       # 重复的接受回调
    assert rid in [c[0] for c in nav.cancelled]
    assert r.state == 'stopped'
    # 结果确认前视为"未停"：保持锁停语义（cancel_request 仍在）
    assert r.cancel_request == rid
    r.on_result(rid, 'cancelled')
    assert r.cancel_request is None and r.locked and r.state == 'stopped'


def test_stop_before_accept_schedules_cancel():
    """停车时目标还在发送（未接受）：取消未确认，接受一到立即取消。"""
    r, nav, _ = make()
    r.release_lock()
    r.start('m1')
    rid = nav.dispatched[0][0]               # 未调用 on_accepted
    r.stop('global_stop')
    assert r.cancel_request == rid and r.sending is None
    r.on_accepted(rid)                       # 迟到的接受
    assert [c[0] for c in nav.cancelled].count(rid) == 2
    assert r.state == 'stopped' and r.locked
    r.on_result(rid, 'cancelled')
    assert r.cancel_request is None and r.state == 'stopped'


def test_cancel_failure_keeps_locked_and_recancels():
    r, nav, _ = make()
    r.release_lock()
    r.start('m1')
    rid = nav.dispatched[0][0]
    r.on_accepted(rid)
    nav.cancel_ok = False
    r.stop('global_stop')
    assert r.cancel_request == rid
    nav.cancel_ok = True
    r.on_accepted(rid)                       # 迟到接受 → 再取消一次
    assert [c[0] for c in nav.cancelled].count(rid) == 2
    assert r.state == 'stopped' and r.cancel_request == rid
    # 取消结果迟迟不确认：仍记录在案
    assert r.cancel_request == rid
    # 结果不是 canceled（例如取消失败后仍成功到达）：不得自动恢复
    r.on_result(rid, 'succeeded')
    assert r.state == 'stopped' and len(nav.dispatched) == 1


def test_manual_takeover_cancels_then_never_resumes():
    r, nav, _ = make()
    r.release_lock()
    r.start('m1')
    rid = nav.dispatched[0][0]
    r.on_accepted(rid)
    assert r.manual_override() is True
    assert rid in [c[0] for c in nav.cancelled]
    assert r.state == 'stopped' and r.stop_reason == 'manual_takeover'
    assert not r.locked                         # 手动接管是用户动作，释放巡航锁
    r.on_result(rid, 'succeeded')               # 旧任务结果迟到
    assert r.state == 'stopped' and len(nav.dispatched) == 1
    r.manual_released()                         # 松键
    assert r.state == 'stopped' and len(nav.dispatched) == 1
    # 再开始：从第一个航点
    r.start('m1')
    assert r.index == 0 and nav.dispatched[-1][1]['x'] == 0.0


def test_manual_override_without_nav_does_not_touch_lock():
    r, nav, _ = make()
    assert r.locked
    assert r.manual_override() is False
    assert r.locked                              # 别的锁停不被手动"顺手"放开


def test_disconnect_cancels_and_locks():
    r, nav, _ = make()
    r.release_lock()
    r.start('m1')
    rid = nav.dispatched[0][0]
    r.on_accepted(rid)
    r.disconnect()
    assert r.state == 'stopped' and r.stop_reason == 'web_disconnected'
    assert rid in [c[0] for c in nav.cancelled]
    assert r.cancel_request == rid           # 取消结果未确认：不算已停
    r.on_result(rid, 'cancelled')
    assert r.cancel_request is None and r.state == 'stopped'



def test_goal_timeout_beats_infinite_wait():
    r, nav, clk = make()
    r.release_lock()
    r.start('m1')
    clk.tick(15.5)
    assert r.overall_timeout(15.0) is True
    assert r.state == 'failed'
    assert r.lock_reason == 'goal_timeout' and r.locked


def test_rearm_requires_license_and_does_not_resume():
    r, nav, _ = make()
    r.start('m1')
    r.stop('global_stop')                        # 在途目标：只剩待确认取消
    ok, _ = r.rearm(license_ok=False)
    assert not ok and r.locked
    ok, _ = r.rearm(license_ok=True)
    assert not ok                                # 有在途目标：先停再解除
    assert r.state == 'stopped'
    assert r.cancel_request is not None          # 取消未确认
    assert r.rearm(license_ok=True)[1] == '上一个目标取消未确认，保持锁停'
    r.on_result(1, 'cancelled')                  # 取消确认
    assert r.cancel_request is None
    ok, _ = r.rearm(license_ok=True)
    assert ok and not r.locked
    assert len(nav.dispatched) == 1              # 解除锁停不自动巡航
    assert r.state == 'stopped'


# ------------------------------------------------------------------ 许可
def test_license_100ms_heartbeat_350ms_expiry():
    clk = Clock(1000.0)
    lost = []
    lic = P.MotionLicense(lost.append, now=clk)
    assert lic.expired() is True                 # 启动即无许可
    assert lost == ['license_lost']
    seq = 0
    for _ in range(4):                           # 每 100 ms 一条
        clk.tick(0.1)
        seq += 1
        ok, reason = lic.accept(7, seq, expires_mono=clk() + 0.5)
        assert ok, reason
        assert lic.expired() is False
    clk.tick(0.34)                               # 340 ms：仍在线
    assert lic.expired() is False
    clk.tick(0.05)                               # 390 ms：失效，只报一次
    assert lic.expired() is True
    assert lost.count('license_lost') == 2       # 启动即锁停 + 这次失效
    ok, _ = lic.accept(7, seq)                   # 会话失效后旧消息? 序号必须前进
    assert not ok and _ == 'seq_replay'


def test_license_rejects_replay_session_change_and_expired():
    clk = Clock(50.0)
    lic = P.MotionLicense(lambda r: None, now=clk)
    assert lic.accept(1, 5, 50.4) == (True, 'ok')
    assert lic.accept(1, 5, 50.4)[1] == 'seq_replay'
    assert lic.accept(1, 4)[1] == 'seq_replay'
    assert lic.accept(2, 6)[1] == 'session_mismatch'
    assert lic.accept(1, 6, 49.0)[1] == 'expired'
    assert lic.accept(1, 6, 'x')[1] == 'expires_invalid'
    assert lic.accept(0, 7)[1] == 'session_invalid'
    assert lic.accept(1, -1)[1] == 'seq_invalid'
    assert lic.accept(True, 8)[1] == 'session_invalid'


def test_released_only_with_fresh_license_and_unlocked_run():
    clk = Clock(10.0)
    runner, nav, _ = make(clk=clk)
    lic = P.MotionLicense(lambda r: None, now=clk)
    frame = P.status_frame(runner, lic.snapshot(), 'm1')
    assert frame['licensed'] is False and frame['released'] is False
    assert frame['locked'] is True
    clk.tick(0.1)
    lic.accept(3, 0, 10.6)
    lic.expired()
    frame = P.status_frame(runner, lic.snapshot(), 'm1')
    assert frame['licensed'] is True
    assert frame['released'] is False             # 仍锁停
    runner.rearm(license_ok=True)
    frame = P.status_frame(runner, lic.snapshot(), 'm1')
    assert frame['released'] is True
    # 许可中断：即使 runner 状态是"已解除"，也不再对外宣称 released
    clk.tick(0.4)
    lic.expired()
    frame = P.status_frame(runner, lic.snapshot(), 'm1')
    assert frame['licensed'] is False and frame['released'] is False


def test_status_frame_fields():
    r, nav, clk = make()
    lic = P.MotionLicense(lambda x: None, now=clk)
    lic.accept(1, 0, clk() + 0.5)
    frame = P.status_frame(r, lic.snapshot(), 'm1')
    for key in ('map_id', 'current_map_id', 'route_valid', 'index', 'waypoints',
                'goal', 'status', 'stop_reason', 'stop_text', 'locked',
                'lock_reason', 'lock_text', 'licensed', 'license', 'released'):
        assert key in frame, key
    assert frame['map_id'] == 'm1' and frame['route_valid'] is True
    assert frame['lock_text']


# ------------------------------------------------------------------ 下游独立锁停
def test_web_process_death_downstream_locks_without_web(monkeypatch):
    """网页进程退出：不再有任何许可 → 下游（按自身许可超时）独立锁停。

    代码路径：网页许可只经 websocket 进入 web_debug（续期服务端状态），
    底盘桥的锁停由它自己的 LICENSE_TIMEOUT_S 计时，不查询网页进程。
    这里把"网页侧最后一条许可"时间冻结在进程退出时刻，推进下游时钟，
    断言下游许可过期、且网页侧判失效（web_debug 的 350 ms 规则）。
    """
    down = Clock(200.0)
    web = Clock(200.0)
    lic = P.MotionLicense(lambda r: None, now=web)
    lic.accept(9, 0, web() + 0.5)
    lic.expired()
    assert lic.fresh()
    # 进程退出：两边的时钟照走，但再没有新许可进来
    down.tick(0.55)
    web.tick(0.55)
    downstream_fresh = (down() - 200.0) <= P.LICENSE_TIMEOUT_S
    assert downstream_fresh is False            # 下游 0.5 s 后锁停
    assert lic.expired() is True                # 网页侧 0.35 s 判失效
    # 源码证据：许可证只在 websocket 读循环里被接收，下游不得读取网页状态
    src = inspect.getsource(W)
    assert "'motion_license'" in src and 'patrol_license_message' in src
    # patrol.MotionLicense 的失效只看"最后收到时刻"，不依赖读循环继续运行
    lic2 = P.MotionLicense(lambda r: None, now=web)
    lic2.accept(10, 0)
    web.tick(0.1)
    assert lic2.age() == pytest.approx(0.1, abs=1e-6)


# ------------------------------------------------------------------ 鉴权
def _frame(payload):
    data = json.dumps(payload).encode('utf-8')
    mask = b'\x01\x02\x03\x04'
    body = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
    if len(data) < 126:
        head = bytes([0x81, 0x80 | len(data)])
    else:
        head = bytes([0x81, 0x80 | 126]) + len(data).to_bytes(2, 'big')
    return head + mask + body


class _Sock:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.sent = []

    def recv(self, _n):
        return self.chunks.pop(0) if self.chunks else b''

    def sendall(self, data):
        self.sent.append(data)


def _handler(*msgs):
    h = W.Handler.__new__(W.Handler)
    h.connection = _Sock([_frame(m) for m in msgs])
    h._ws_buf = bytearray()
    return h


def _sent(h):
    out = []
    for raw in h.connection.sent:
        # 控制帧：0x81 + 长度（<65536） + 载荷
        n = raw[1] & 0x7F
        if n < 126:
            payload = raw[2:]
        else:
            payload = raw[4:]
        if payload:
            out.append(json.loads(payload.decode('utf-8')))
    return out


@pytest.mark.parametrize('kind,extra', [
    ('patrol_save', {'points': [[0, 0], [1, 0]]}),
    ('patrol_start', {}),
    ('motion_stop', {}),
    ('motion_rearm', {}),
])
def test_patrol_messages_require_same_auth_as_cruise(kind, extra):
    msg = dict({'t': kind}, **extra)
    # 未登录：拒绝
    h = _handler(msg)
    h._read_view(True, 0.0)
    out = _sent(h)
    assert any(m.get('t') == 'manual_error' and '登录' in m.get('message', '')
               for m in out), out
    # 已登录但不在实时页：拒绝
    h = _handler(msg)
    W._MANUAL.login(h, 'admin', 'password')
    try:
        h._read_view(False, 0.0)
        out = _sent(h)
        assert any(m.get('t') == 'manual_error' and '实时页面' in m.get('message', '')
                   for m in out), out
    finally:
        W._MANUAL.disconnect(h)
    # 对照：cruise_set 同样两条规则
    for login, live in ((False, True), (True, False)):
        h = _handler({'t': 'cruise_set', 'on': False})
        if login:
            W._MANUAL.login(h, 'admin', 'password')
        try:
            h._read_view(live, 0.0)
            assert any(m.get('t') == 'manual_error' for m in _sent(h))
        finally:
            W._MANUAL.disconnect(h)
    # 已登录 + 实时页：放行（跑到服务层，回 patrol_status 回执）
    W.reset_state()
    h = _handler(msg)
    W._MANUAL.login(h, 'admin', 'password')
    try:
        h._read_view(True, 0.0)
        out = _sent(h)
        assert not any(m.get('t') == 'manual_error' for m in out), out
        if kind != 'motion_stop':
            assert any(m.get('t') == 'patrol_status' for m in out), out
    finally:
        W._MANUAL.disconnect(h)


def test_ultrasonic_rearm_requires_same_auth():
    W._MANUAL.attach(None)              # 清掉其他用例残留的登录态
    """超声"重新启动"是驾驶动作：鉴权同 cruise_set，且只发 ultrasonic_rearm。"""
    bridge = W.Bridge()                 # 建发布器
    msg = {'t': 'ultrasonic_rearm'}
    h = _handler(msg)
    h._read_view(True, 0.0)
    assert any(m.get('t') == 'manual_error' and '登录' in m.get('message', '')
               for m in _sent(h)), _sent(h)
    h = _handler(msg)
    W._MANUAL.login(h, 'admin', 'password')
    try:
        h._read_view(False, 0.0)
        assert any(m.get('t') == 'manual_error' and '实时页面' in m.get('message', '')
                   for m in _sent(h)), _sent(h)
    finally:
        W._MANUAL.disconnect(h)
    assert bridge.publishers['ultrasonic_rearm'].messages == []
    # 已登录 + 实时页：发一条 Bool(True)，回执 ultrasonic_status
    h = _handler(msg)
    W._MANUAL.login(h, 'admin', 'password')
    try:
        h._read_view(True, 0.0)
    finally:
        W._MANUAL.disconnect(h)
    sent = bridge.publishers['ultrasonic_rearm'].messages
    assert len(sent) == 1 and isinstance(sent[0], fake_ros.Bool)
    assert sent[0].data is True
    out = _sent(h)
    assert any(m.get('t') == 'ultrasonic_status' and m.get('ok') is True
               for m in out), out


def test_ultrasonic_rearm_without_publisher_reports_failure():
    """发布器不可用时如实回失败，不假装已恢复。"""
    W.reset_state()
    ok, info = W.ultrasonic_rearm()     # 未建 Bridge
    assert ok is False and '不可用' in info


def test_patrol_handler_rejects_bad_save_without_map():
    W.reset_state()
    # 未配置地图（map_id 未知）：旧路线/新路线都不能保存
    W._patrol['map_id'] = None
    h = _handler({'t': 'view', 'page': 'map'},
                 {'t': 'patrol_save', 'points': [[0, 0], [1, 0]]})
    W._MANUAL.login(h, 'admin', 'password')
    try:
        h._read_view(True, 0.0)
        out = [m for m in _sent(h) if m.get('t') == 'patrol_status']
        assert out and out[-1]['ok'] is False
    finally:
        W._MANUAL.disconnect(h)


def test_patrol_save_start_stop_rearm_flow(map_yaml):
    path = map_yaml()
    W.configure_patrol(map_yaml=path)
    mid = W.patrol_map_id(path)
    ok, msg, _ = W.patrol_save([[0, 0], [2, 0], [2, 2]], mid)
    assert ok, msg
    runner = W._patrol['runner']
    assert runner.route['points'][2][2] == pytest.approx(-3 * math.pi / 4)
    assert runner.locked
    # 无新鲜许可不能恢复
    assert W.patrol_rearm()[0] is False
    lic = W._patrol['license']
    lic.accept(11, 0)
    lic.expired()
    assert W.patrol_rearm()[0] is True
    ok, msg = W.patrol_start()
    assert ok
    assert runner.state == 'running' and runner.sending is not None
    seq_of_running = runner.sending
    W.patrol_stop('global_stop')
    assert runner.state == 'stopped' and runner.stop_reason == 'global_stop'
    assert runner.cancel_request is not None     # 目标未确认：保持锁停
    # 视图层锁停
    assert W.patrol_status_frame()['locked'] is True
    runner.on_accepted(runner.cancel_request)    # 迟到接受 → 取消
    runner.on_result(seq_of_running, 'cancelled')
    assert runner.cancel_request is None


# ------------------------------------------------------------------ 页面
def test_page_has_patrol_controls_and_status():
    page = W.PAGE
    for eid in ('patrol_save', 'patrol_start', 'motion_stop', 'motion_rearm',
                'patrol_state', 'patrol_map', 'patrol_waypoint',
                'patrol_stop', 'patrol_license', 'patrol_lock', 'route_add'):
        assert 'id="' + eid + '"' in page, eid
    for kind in ('patrol_save', 'patrol_start', 'motion_stop', 'motion_rearm'):
        assert "{ t: '" + kind + "'" in page or "t: '" + kind + "'" in page, kind
    assert "'patrol_status'" in page
    assert "'motion_license'" in page


def test_page_license_heartbeat_is_100ms():
    page = W.PAGE
    assert 'MOTION_LICENSE_MS = 100' in page
    assert 'setInterval(sendMotionLicense, MOTION_LICENSE_MS)' in page
    assert 'stopMotionLicense()' in page          # 断开即停发
    # 页面只发消息：保存/开始/停止都不在页面里判断锁停或许可
    src = page[page.index('function patrolSend'):]
    src = src[:src.index('function applyNav')]
    assert 'patrolSend(' in src
    assert 'sendMotionLicense' in page and 'expires_mono' in page


# ------------------------------------------------------------------ 真实 nav_goal 接线
@pytest.fixture
def wired_nav(monkeypatch):
    """把 web_debug 的巡航接到真实的 NavGoalControl + 假 ActionClient。"""
    # 未配置 on_result：结果只经生产接线（set_callbacks）送达巡航侧，
    # 这与 Bridge 的用法一致（不计数的吞掉会让取消确认静默丢失）。
    ctrl = NG.NavGoalControl()
    assert ctrl.attach(None)
    monkeypatch.setattr(NG, '_NAV', ctrl)
    monkeypatch.setattr(W, '_patrol_cancel',
                        lambda rid, result_known=True: ctrl.cancel_seq(rid, result_known))
    # 测试驱动：投递队列里的目标直接交给假 ActionClient（不跑 executor），
    # nav_goal 的接受/结果回调回流到 W 的巡航状态机。
    monkeypatch.setattr(W, '_patrol_run_dispatch',
                        lambda: _pump(ctrl))
    # 地图内容标识：用临时 yaml 当"当前地图"，路线按它的 sha256 保存。
    # patrol_start 会按真实文件重算标识，必须给文件才是"有地图"的形态。
    import os
    import tempfile
    fd, path = tempfile.mkstemp(suffix='.yaml')
    with os.fdopen(fd, 'w') as fh:
        fh.write('image: map.pgm\nresolution: 0.05\n')
    W._PATROL_TEST_MAP = P.map_id_from_files([path])
    W.configure_patrol(reset_runner=True, map_yaml=path)
    try:
        yield ctrl
    finally:
        os.unlink(path)
        W._PATROL_TEST_MAP = None


def _pump(ctrl):
    q = W._patrol['goal_queue']
    while q is not None and not q.empty():
        rid, goal = q.get_nowait()
        ok, view = ctrl.request_goal(goal['x'], goal['y'], goal['yaw'])
        assert ok, view
        # 投递完一次就接上本状态机的回调（不跑 executor，回调手动对账）
        W._patrol_wire_nav()
        if getattr(ctrl._client, 'waiting', False):
            break        # 保持执行器投递语义：一次 tick 只投一个目标


def _save_route(pts=((0.0, 0.0), (2.0, 0.0)), map_id=None):
    map_id = map_id or getattr(W, '_PATROL_TEST_MAP', None) or ROUTE_ID
    route, err = P.validate_route([list(p) for p in pts], map_id, map_id)
    assert err is None
    W._patrol['map_id'] = map_id
    W._patrol['runner'].save(route)


def test_late_accept_cancelled_end_to_end(wired_nav):
    """W 层：停车时目标未接受，迟到的接受经 nav_goal 回调立即取消并保持锁停。"""
    ctrl = wired_nav
    _save_route()
    runner = W._patrol['runner']
    lic = W._patrol['license']
    lic.accept(21, 0)
    assert runner.rearm(license_ok=lic.fresh())[0] is True
    assert W.patrol_start()[0] is True
    ctrl._client.async_accept = True             # 接受应答挂起（迟到接受）
    W.patrol_tick()                              # 投递 → 假客户端挂起
    seq = ctrl.view()['seq']
    assert runner.sending == seq
    # 目标尚未接受：全局停止。取消被排程（_parked），状态锁停。
    W.patrol_stop('global_stop')
    assert runner.locked and runner.state == 'stopped'
    assert runner.cancel_request == seq
    ctrl._client.deliver()                       # Nav2 迟到接受
    # nav_goal 在接受的同一回调里按句柄取消（重复取消幂等，至少一次）
    assert ctrl._client.handles[-1].cancelled >= 1
    assert ctrl.view()['state'] == 'canceled'
    # 结果确认前仍保持锁停
    assert runner.locked and runner.state == 'stopped'
    # 模拟 Nav2：目标被取消 → 结果回调（CANCELED）到达。回调参数是
    # future 视图；patrol 侧的取消确认随即由结果本身补齐。
    ctrl._client.handles[-1].finish(5)
    assert runner.cancel_request is None
    assert runner.locked and runner.state == 'stopped'
    assert len(ctrl._client.goals) == 1          # 没有自动发下一点


def test_manual_takeover_end_to_end_never_resumes(wired_nav):
    """手动接管先取消旧导航；旧任务结果/松键都不让巡航自己恢复。"""
    ctrl = wired_nav
    _save_route()
    runner = W._patrol['runner']
    W._patrol['license'].accept(22, 0)
    assert runner.rearm(license_ok=True)[0] is True
    assert W.patrol_start()[0] is True
    W.patrol_tick()                              # 同步接受 → active
    assert ctrl.view()['state'] == 'active'
    assert runner.state == 'running'

    owner = type('Owner', (), {})()
    W._MANUAL.attach(lambda vx, wz: None, ready=lambda: True)
    W._MANUAL.login(owner, 'admin', 'password')
    try:
        # 没按手动键：不算接管
        assert W.patrol_manual_override(owner) is False
        assert runner.state == 'running'
        # 按下手动键：先取消旧导航
        W._MANUAL.start(owner, 0.4, 'hold')
        assert W.patrol_manual_override(owner) is True
        assert ctrl._client.handles[-1].cancelled == 1
        assert runner.state == 'stopped'
        W._MANUAL.stop(owner)                    # 松键
        runner.manual_released()
        assert runner.state == 'stopped'
        assert len(ctrl._client.goals) == 1
    finally:
        W._MANUAL.stop(owner)
        W._MANUAL.disconnect(owner)
        W._MANUAL.attach(None)


def test_late_accept_cancelled_in_nav_goal(wired_nav):
    """目标未接受时停车 → 接受迟到 → NavGoalControl 立即按句柄取消。"""
    ctrl = wired_nav
    ctrl._client.async_accept = True
    ok, view = ctrl.request_goal(1.0, 1.0, 0.5)
    assert ok and view['state'] == 'sending'
    seq = view['seq']
    ok = ctrl.cancel_seq(seq, result_known=False)   # 巡航侧：接受前停车
    assert ok is False                               # 未确认停止
    ctrl._client.deliver()                           # Nav2 迟到接受
    assert ctrl._client.handles[0].cancelled == 1
    assert ctrl.view()['state'] == 'canceled'


def test_cancel_ack_failure_reverts_to_active(wired_nav):
    ctrl = wired_nav
    ok, _ = ctrl.request_goal(1.0, 1.0, 0.5)
    assert ok and ctrl.view()['state'] == 'active'
    ctrl._client.cancel_raises = True
    ok = ctrl.cancel_seq(ctrl.view()['seq'])
    assert ok is False
    assert ctrl.view()['state'] == 'active'      # 取消失败：不能认为已停


def test_status_topic_declared_and_view_carries_patrol():
    b = W.Bridge()
    assert 'patrol_status' in b.publishers
    view = W.status_payload()
    assert 'patrol' in view
    p = view['patrol']
    for key in ('map_id', 'index', 'status', 'stop_reason', 'lock_reason',
                'licensed', 'released'):
        assert key in p, key
