"""巡航区复核修补用例（本轮）：会话断线、许可会话接管、停车原因、页面契约。

每条用例都对应一个本轮复现并修掉的问题（修复前用同一代码路径跑出失败）：

1. 唯一的网页连接断开（或心跳超时）后，巡航若没有 ``cancel_request``
   （目标已被接受）此前不会被停——``_read_view`` 的 finally 只做了
   ``_MANUAL.disconnect``。现在走 ``patrol_session_closed``。
2. 许可会话被顶替（新页面用新 session）后旧会话永续期：``MotionLicense``
   从不判失效、新会话永被拒、"恢复"永远失败；页面重开后也回不到试探态。
   现在旧会话 350 ms 后判失效且允许新会话接管，页面用 ``claim`` 换号。
3. 页面 ``expires_mono`` 把服务端 monotonic（开机起算）与
   ``performance.now()``（页面加载起算）相加，期限恒为 −开机秒，
   每一帧都被判过期。现在用 (perf0, mono0) 锚点换算。
4. 四个按钮只发动作、不带 ``t:'view'``：服务端按本连接最近一次 view 判
   "在实时页"，空闲后会被当成非实时页拒掉。
"""

import json
import time

import pytest

import fake_ros

from opi_robot.web_node import nav_goal as NG
from opi_robot.web_node import patrol as P
from opi_robot.web_node import web_debug as W


# ------------------------------------------------------------------ 工具
class Clock:
    def __init__(self, t=500.0):
        self.t = t

    def __call__(self):
        return self.t

    def tick(self, dt):
        self.t += dt


class _Nav:
    """dispatch / cancel / lock_stop 三件套（与 web_debug 注入形状一致）。"""

    def __init__(self):
        self.dispatched = []
        self.cancelled = []
        self.stops = []
        self.next_id = 1

    def dispatch(self, goal):
        rid = self.next_id
        self.next_id += 1
        self.dispatched.append((rid, dict(goal)))
        return rid

    def cancel(self, rid, result_known=True):
        self.cancelled.append((rid, result_known))
        return True

    def lock_stop(self, reason):
        self.stops.append(reason)


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
    # 握手阶段已取过 rfile 缓冲（见 web_debug._read_view 的 _ws_drained）
    h._ws_drained = True
    h._ws_page = None
    # 连接开始时刻：与 _ws_loop 一致（心跳超时用它比较）
    h._next_msg_t = time.time() + W.CLIENT_TIMEOUT
    return h


def _beat(h, next_msg_t=None):
    """模拟一次 _ws_loop 迭代：传入上一轮返回的 next_msg_t，返回新的。"""
    live, nt, refused = h._read_view(True, h._next_msg_t)
    h._next_msg_t = nt
    h._ws_live = live
    return live, nt, refused


def _sent(h):
    out = []
    for raw in h.connection.sent:
        n = raw[1] & 0x7F
        payload = raw[2:] if n < 126 else raw[4:]
        if payload:
            out.append(json.loads(payload.decode('utf-8')))
    return out


@pytest.fixture
def send_ok(monkeypatch):
    """离线容器没有 nav2_msgs：把"发目标"这一步换成"已接受"。

    生产路径（真的 send_goal_async + 接受回调）在容器里不可用；被替换的只是
    "把目标交给 Nav2 并收到接受"这一步，巡航状态机的竞态/会话路径全部是
    真实代码。取消恒为"已发出"。
    """
    def _drain():
        q = W._patrol['goal_queue']
        while q is not None and not q.empty():
            q.get_nowait()             # 已由 _start_running 手动标成"已接受"
    monkeypatch.setattr(W, '_patrol_run_dispatch', _drain)
    monkeypatch.setattr(W, '_patrol_cancel',
                        lambda rid, result_known=True: True)
    return True


@pytest.fixture(autouse=True)
def _clean():
    W.reset_state()
    fake_ros.reset_topics()
    W._MANUAL.attach(None)
    W.patrol_session_reset()
    yield
    W._MANUAL.attach(None)
    W.patrol_session_reset()
    W.reset_state()


def _route_runner(map_id='mapA'):
    W._patrol['map_id'] = map_id
    route, err = P.validate_route([[0, 0], [2, 0]], map_id, map_id)
    assert err is None
    runner = W._patrol['runner']
    runner.save(route)
    return runner


def _start_running(map_id='mapA', tx=None):
    """W 层：许可有效 → 恢复 → 开始 → 目标已被 Nav2 接受（running）。

    离线容器没有 nav2_msgs，真正的 ``_patrol_run_dispatch`` 会按发送失败
    处理（这是正确的失败即停行为）；这里直接把目标标成"已接受"，验证
    其后与断线/许可相关的路径。
    """
    runner = _route_runner(map_id)
    _licensed()
    assert W.patrol_rearm()[0] is True
    assert W.patrol_start()[0] is True
    rid = runner.sending
    assert rid is not None
    if tx is not None:
        tx[0] = rid                  # 记下这次发目标用的请求编号
    runner.on_accepted(rid)
    assert runner.state == 'running' and runner.pending == rid
    return runner, rid


def _licensed(sid=77, seq=0):
    lic = W._patrol['license']
    ok, reason = lic.accept(sid, seq)
    assert ok, reason
    lic.expired()


# ------------------------------------------------------------------ 1. 会话断线
def test_last_session_closed_cancels_accepted_cruise(send_ok):
    """最后一个实时页连接走掉：已被接受的在途目标取消、锁停、不自动恢复。"""
    tx = [None]
    runner, rid = _start_running(tx=tx)
    assert W.patrol_status_frame()['released'] is True

    W._MANUAL.attach(lambda *a: None, ready=lambda: True)
    h1, h2 = _handler(), _handler()
    for h in (h1, h2):
        W._MANUAL.login(h, 'admin', 'password')
        _beat(h)
    try:
        assert W.patrol_session_hooks()['live'] == 2
        W.patrol_session_closed(h1)                  # 还有一个页面：不算失效
        assert W.patrol_session_hooks()['live'] == 1
        assert W.patrol_session_hooks()['lost'] == ''
        assert runner.state == 'running'
        W.patrol_session_closed(h2)                  # 最后一个也走了
    finally:
        W._MANUAL.disconnect(h1)
        W._MANUAL.disconnect(h2)
    assert W.patrol_session_hooks()['lost'] == 'web_disconnected'
    assert runner.state == 'stopped' and runner.stop_reason == 'web_disconnected'
    assert runner.locked and runner.lock_reason == 'web_disconnected'
    # 取消已发出，但结果未确认前仍记为"待取消"（不能认为已停）
    assert runner.cancel_request == tx[0]
    frame = W.patrol_status_frame()
    assert frame['released'] is False and frame['locked'] is True
    # 旧目标之后成功返回：不得发下一点、不得解锁
    runner.on_result(tx[0], 'succeeded')
    assert runner.state == 'stopped' and runner.locked


def test_read_view_timeout_drops_map_session(send_ok):
    """实时页超过 CLIENT_TIMEOUT 没有控制帧：会话退出，巡航按失效停。"""
    runner, rid = _start_running()
    h = _handler({'t': 'view', 'page': 'map'})
    _beat(h)                                  # 进入地图页（免登录）
    assert W.patrol_session_hooks()['live'] == 1
    _beat(h)                                  # 1 s 一次的心跳：未超时
    assert runner.state == 'running'
    h._next_msg_t = time.time() - 1.0         # 心跳早已超时
    _beat(h)
    assert W.patrol_session_hooks()['live'] == 0
    assert W.patrol_session_hooks()['lost'] == 'web_disconnected'
    assert runner.state == 'stopped' and runner.locked


def test_license_expiry_stops_running_cruise(send_ok):
    """许可失效（350 ms 无新许可）：取消并锁停，旧目标结果不复活巡航。"""
    runner, rid = _start_running()
    W._patrol['license']._now = lambda: 1e6        # 时钟跳远：许可失效
    W.patrol_tick(now=1e6)
    assert runner.state == 'stopped'
    assert runner.lock_reason == 'license_lost'
    frame = W.patrol_status_frame()
    assert frame['licensed'] is False and frame['released'] is False
    assert frame['license_state'] == 'lock_stop'
    n_before = len(W._patrol['goal_queue'].queue) if W._patrol['goal_queue'] else 0
    runner.on_result(rid, 'succeeded')
    assert runner.state == 'stopped' and runner.locked
    assert runner.sending is None and runner.pending is None
    n_after = len(W._patrol['goal_queue'].queue) if W._patrol['goal_queue'] else 0
    assert n_after == n_before                     # 没有自动发下一点


def test_watchdog_thread_runs_without_read_loop(send_ok):
    """看护线程独立判失效：即使读循环卡死/网页断开，也执行取消并锁停。"""
    runner, rid = _start_running()
    sleep_calls = []
    orig_sleep = W._sleep

    def fake_sleep(_s):
        sleep_calls.append(_s)
        W._patrol['license']._now = lambda: 1e6     # 下一次看护：许可已失效
        stop[0] = True

    stop = [False]
    W._sleep = fake_sleep
    try:
        W.patrol_session_watch(lambda: stop[0])
    finally:
        W._sleep = orig_sleep
    assert sleep_calls == [W.SESSION_POLL_S]
    assert runner.state == 'stopped' and runner.locked
    assert runner.lock_reason == 'license_lost'


# ------------------------------------------------------------------ 2. 许可会话接管
def test_superseded_session_expires_and_new_session_takes_over():
    """旧会话停止送许可后：判失效 → 锁停；新页面（新 session）可接管。"""
    clk = Clock(300.0)
    stops = []
    lic = P.MotionLicense(lambda r: stops.append(r), now=clk)
    assert lic.accept(100, 0, 300.5) == (True, 'ok')
    clk.tick(0.1)
    assert lic.fresh() is True
    # 新页面来报到，旧会话仍在送许可：拒绝（防两条页面互抢）
    assert lic.accept(200, 0) == (False, 'session_mismatch')
    clk.tick(0.4)                                 # 旧会话不再续期（>350 ms）
    assert lic.expired() is True
    assert 'license_lost' in stops
    # 旧会话已停止送许可：新页面接管，序号从头开始
    assert lic.accept(200, 0) == (True, 'ok')
    assert lic.session == 200 and lic.fresh() is True
    # 接管之后，原旧会话的迟到消息一律拒绝（不能再把许可抢回去）
    assert lic.accept(100, 99)[1] == 'session_mismatch'


def test_session_lost_marks_license_stale_immediately():
    """最后一个页面走掉：即使许可还在 350 ms 内，状态帧也不许自称"在线"。"""
    lic = P.MotionLicense(lambda r: None, now=Clock(10.0))
    assert lic.accept(5, 0)[0]
    assert lic.fresh() is True
    lic.mark_stale()
    assert lic.fresh() is False
    assert lic.snapshot()['session_dead'] is True
    assert lic.accept(5, 1)[0]                    # 同一会话再来就能恢复
    assert lic.fresh() is True


def test_license_message_claim_and_confirm():
    """``session=0`` 取时钟基准并可同时声明 claim；被接受即回 session 确认。"""
    W.reset_state()
    W.patrol_session_reset()
    assert W._patrol['license'].session is None
    out = W.patrol_license_message({'t': 'motion_license', 'session': 0,
                                    'seq': 1, 'claim': 4242})
    kinds = [m['t'] for m in out]
    assert kinds == ['motion_license_session', 'motion_clock']
    assert out[0]['session'] == 4242
    assert W._patrol['license'].session == 4242
    # 正常许可：被接受即确认本页号码
    ok = W.patrol_license_message({'t': 'motion_license', 'session': 4242,
                                   'seq': 2})
    assert ok == {'t': 'motion_license_session', 'session': 4242}
    # 被别处占用：nak 带 owner（被拒页面据此换号）
    nak = W.patrol_license_message({'t': 'motion_license', 'session': 999,
                                    'seq': 0})
    assert nak['reason'] == 'session_mismatch'
    texts = [json.loads(t) for t in W._session_msg_text(nak)]
    assert texts[0]['session'] == 999 and texts[0]['owner'] == 4242


def test_license_handler_sends_clock_and_confirmation():
    h = _handler({'t': 'motion_license', 'session': 0, 'seq': 1, 'claim': 31})
    _beat(h)
    kinds = [m['t'] for m in _sent(h)]
    assert 'motion_clock' in kinds and 'motion_license_session' in kinds
    W.patrol_session_reset()


# ------------------------------------------------------------------ 3. 停车原因/锁停
def test_stop_cancels_accepted_goal_and_reports_latest_reason():
    nav = _Nav()
    r = P.PatrolRunner(nav.dispatch, nav.cancel, nav.lock_stop, Clock())
    route, _ = P.validate_route([[0, 0], [2, 0]], 'm', 'm')
    r.save(route)
    r.release_lock()
    r.start('m')
    r.on_accepted(1)
    assert r.pending == 1
    r.stop('global_stop', user_action=True)
    assert nav.cancelled and nav.cancelled[0][0] == 1     # 已被接受也取消
    assert r.pending is None and r.state == 'stopped'
    assert r.snapshot()['stop_reason'] == 'global_stop'
    assert r.snapshot()['lock_reason'] == 'global_stop'
    # 第二次停车：显示的是最近一次用户动作，而不是上一个失败原因
    r.release_lock()
    r._fail('dispatch_failed')
    assert r.lock_reason == 'dispatch_failed'
    r.stop('global_stop', user_action=True)
    assert r.lock_reason == 'global_stop'
    assert r.snapshot()['stop_reason'] == 'global_stop'


def test_manual_takeover_does_not_release_global_stop_lock():
    """锁停来自全局停车时，按手动键接管不解除该锁停。"""
    nav = _Nav()
    r = P.PatrolRunner(nav.dispatch, nav.cancel, nav.lock_stop, Clock())
    route, _ = P.validate_route([[0, 0], [2, 0]], 'm', 'm')
    r.save(route)
    r.release_lock()
    r.start('m')
    r.on_accepted(1)
    r.stop('global_stop')
    assert r.locked and r.lock_reason == 'global_stop'
    assert r.manual_override() is False          # 已无在途目标：不构成"接管"
    assert r.locked and r.lock_reason == 'global_stop'


def test_rearm_while_running_stops_and_keeps_lock():
    """没有锁停但旧目标取消未确认时点恢复：先全局停车，不叠加新目标。"""
    nav = _Nav()
    r = P.PatrolRunner(nav.dispatch, nav.cancel, nav.lock_stop, Clock())
    route, _ = P.validate_route([[0, 0], [2, 0]], 'm', 'm')
    r.save(route)
    r.release_lock()
    r.start('m')
    r.on_accepted(1)
    ok, msg = r.rearm(license_ok=True)
    assert ok is False
    assert r.state == 'stopped' and r.locked and r.lock_reason == 'global_stop'
    assert (1, True) in nav.cancelled
    assert len(nav.dispatched) == 1


def test_status_frame_license_state_and_gate():
    clk = Clock(10.0)
    nav = _Nav()
    runner = P.PatrolRunner(nav.dispatch, nav.cancel, nav.lock_stop, clk)
    route, _ = P.validate_route([[0, 0], [2, 0]], 'm', 'm')
    runner.save(route)
    lic = P.MotionLicense(lambda r: None, now=clk)
    lic.accept(1, 0, 10.6)
    frame = P.status_frame(runner, lic.snapshot(), 'm')
    assert frame['map_id'] == 'm' and frame['current_map_id'] == 'm'
    assert frame['route_valid'] is True
    assert frame['licensed'] is True and frame['released'] is False
    assert frame['pending_cancel'] is None
    clk.tick(0.4)
    lic.expired()
    frame = P.status_frame(runner, lic.snapshot(), 'm')
    assert frame['licensed'] is False and frame['released'] is False


# ------------------------------------------------------------------ 4. 页面契约
def test_page_buttons_send_view_first():
    page = W.PAGE
    i = page.index('function patrolSend(')
    seg = page[i:i + 1200]
    assert seg.index("t: 'view'") < seg.index('ws.send(JSON.stringify(msg))')
    for kind in ('patrol_save', 'patrol_start', 'motion_stop', 'motion_rearm'):
        j = page.index("$('" + kind + "').onclick")
        assert "t: '" + kind + "'" in page[j:j + 400], kind


def test_page_license_clock_is_anchored_not_summed():
    """expires_mono 必须按 (perf0, mono0) 锚点换算；不能把 mono 与 perf 相加。"""
    page = W.PAGE
    i = page.index('function motionNowMs(')
    body = page[i:page.index('}', i) + 1]
    assert 'motionMonoAnchor' in body
    assert 'performance.now() + motionClockSkew' not in body
    clock = page[page.index('function onMotionClock('):]
    assert 'motionMonoAnchorPerf = performance.now()' in clock[:600]


def test_page_has_patrol_controls_and_state_rows():
    page = W.PAGE
    for eid in ('patrol_save', 'patrol_start', 'motion_stop', 'motion_rearm',
                'patrol_state', 'patrol_map', 'patrol_waypoint', 'patrol_stop',
                'patrol_license', 'patrol_gate', 'patrol_lock'):
        assert 'id="' + eid + '"' in page, eid
    for key in ('license_state', 'lock_text', 'stop_text', 'current_map_id'):
        assert key in page, key


# ------------------------------------------------------------------ 6. 看门狗挂到控制连接
def test_control_websocket_starts_and_stops_session_watch(monkeypatch):
    """控制连接建立时就挂上会话看门狗，连接收尾时置停止位。

    读循环卡住（代理半开、客户端不发 RST）时，只有这条独立线程能按
    wall clock 判会话失效——它必须真的被 _websocket 启动，而不是只定义。
    """
    h = _handler()
    h.connection.settimeout = lambda _t: None
    h.connection.setblocking = lambda _b: None
    h.connection.sendall = lambda data: h.connection.sent.append(data)
    h.headers = {}
    h.path = '/ws'
    h.server = type('S', (), {'stats': {}})()
    h.wfile = type('W', (), {})()
    h.close_connection = False
    started = []
    real_thread = W.threading.Thread

    class _Thread:
        def __init__(self, target, args=(), **kw):
            started.append((target, args, kw))

        def start(self):
            pass

    monkeypatch.setattr(W.threading, 'Thread', _Thread)
    monkeypatch.setattr(W, '_take_buffered', lambda _h: b'', raising=False)

    def boom(*_a):
        raise RuntimeError('stop loop')

    monkeypatch.setattr(W.Handler, '_read_view', boom)
    try:
        h._websocket()
    except RuntimeError:
        pass
    finally:
        monkeypatch.setattr(W.threading, 'Thread', real_thread)
    watch = [(t, a) for t, a, _ in started if t is W.patrol_session_watch]
    assert watch, '控制连接未启动 patrol_session_watch'
    _target, args = watch[0]
    assert args[1] is h
    # 连接收尾后停止位已置：看门狗不会再判会话
    assert args[0]() is True


# ------------------------------------------------------------------ 7. 地图页"保存路线"入口
def test_map_page_save_button_has_route_entry():
    """地图页能真正把路线存下去：录入开关存在，且"保存路线"真正发出消息。

    复现：没有录入开关时 routeDraft 恒空，点"保存路线"只提示"至少需要
    两个航点"、一个字节都不发，"保存路线"按钮等于摆设。
    """
    import os
    path = os.path.join(os.path.dirname(W.__file__), 'console_page.html')
    page = open(path, encoding='utf-8').read()
    assert 'id="route_add"' in page
    assert 'id="patrol_save"' in page
    # 保存按钮的消息体（不是 return 前只提示）
    save = page[page.index("$('patrol_save').onclick"):]
    save = save[:save.index('};') + 2]
    assert "t: 'patrol_save'" in save and 'routeDraft' in save


# ================================================================== 复核轮修补
# ------------------------------------------------------------------ 1. 规划受阻 3s（按墙钟，不按"相隔两次调用"）
def test_plan_blocked_fires_on_wall_clock_not_call_gap(send_ok):
    """10 Hz 每拍都喂 blocked=True：3 s 后必须取消并锁停。

    旧实现按"相邻两次调用相差 ≥3 s"计时，而生产每 20–100 ms 调一次，
    3 s 永远不会触发（连续 4 s 受阻仍是 running）。
    """
    clk = Clock(1000.0)
    W.configure_patrol(reset_runner=True, now=clk)
    runner, rid = _start_running()
    for _ in range(40):                  # 4 s @10 Hz
        runner.on_plan_blocked(True)
        clk.tick(0.1)
    assert runner.state == 'failed'
    assert runner.locked and runner.lock_reason == 'no_safe_path'


def test_plan_blocked_resets_when_goal_accepted():
    """刚接受时没有反馈不算受阻：计时从接受时刻起。"""
    clk = Clock(50.0)
    nav = _Nav()
    r = P.PatrolRunner(nav.dispatch, nav.cancel, nav.lock_stop, clk)
    route, _ = P.validate_route([[0, 0], [2, 0]], 'm', 'm')
    r.save(route)
    r.release_lock()
    r.start('m')                         # 目标在发送中：受阻不累计
    for _ in range(50):
        r.on_plan_blocked(True)
        clk.tick(0.1)
    assert r.state == 'running' and not r.locked
    rid = nav.dispatched[0][0]
    r.on_accepted(rid)                   # 接受：重新起算
    assert r.plan_blocked_since is None
    r.on_plan_blocked(True)              # 第一个"受阻"采样：只起算
    clk.tick(2.9)
    r.on_plan_blocked(True)
    assert r.state == 'running'
    clk.tick(0.2)
    r.on_plan_blocked(True)
    assert r.state == 'failed' and r.lock_reason == 'no_safe_path'


def test_plan_blocked_uses_nav2_error_code():
    """Nav2 明确报规划失败（error_code）即判受阻；刚接受无反馈不判。"""
    nav = {'state': 'active', 'feedback': None, 'feedback_age': None,
           'plan_error': '', 'error': ''}
    assert W._patrol_plan_blocked(nav) is False
    nav['plan_error'] = 'error_code=50(NO_VALID_PATH)'
    assert W._patrol_plan_blocked(nav) is True
    nav = {'state': 'active', 'feedback': None, 'feedback_age': None,
           'plan_error': 'error_code=0()', 'error': ''}
    assert W._patrol_plan_blocked(nav) is False


# ------------------------------------------------------------------ 2. 真实入口：manual_start 触发接管
def test_manual_start_message_cancels_old_nav(send_ok):
    """ws 收到 manual_start（已登录、在控制页）：先取消旧导航再发手动速度。

    旧实现在 _MANUAL.start 之前判 holds(owner)，那一刻 owner 还没拿到
    手动控制，接管永远不执行；现在钩子在 start 内部、发布第一条速度前。
    """
    runner, rid = _start_running()
    published = []
    W._MANUAL.attach(lambda vx, wz: published.append((vx, wz)),
                     ready=lambda: True)
    h = _handler()
    h._ws_live = True
    W._MANUAL.login(h, 'admin', 'password')      # 消息里的 self 就是 owner
    try:
        W.Handler._manual_message(h, {'t': 'manual_start', 'speed': 0.4,
                                      'mode': 'hold', 'direction': 'forward',
                                      'id': 1}, True)
        assert runner.state == 'stopped'
        assert runner.stop_reason == 'manual_takeover'
        assert published and published[-1][0] > 0
        # 松键 + 旧任务迟到成功：不恢复巡航，不再发下一点
        W._MANUAL.stop(h)
        runner.on_result(rid, 'succeeded')
        assert runner.state == 'stopped'
        # 不再发新目标（队列里那一条是 _start_running 故意没消费掉的）
        assert len(W._patrol['goal_queue'].queue) == 1
    finally:
        W._MANUAL.disconnect(h)
        W._MANUAL.attach(None)


# ------------------------------------------------------------------ 3. 取消未确认不当作已停
def test_manual_override_cancel_failure_keeps_locked():
    nav = _Nav()
    nav.cancel = lambda rid, result_known=True: False     # 取消被拒
    clk = Clock(10.0)
    r = P.PatrolRunner(nav.dispatch, nav.cancel, nav.lock_stop, clk)
    route, _ = P.validate_route([[0, 0], [2, 0]], 'm', 'm')
    r.save(route)
    r.release_lock()
    r.start('m')
    rid = nav.dispatched[0][0]
    r.on_accepted(rid)
    assert r.manual_override() is True
    assert r.state == 'stopped' and r.locked          # 取消未确认：仍锁停
    assert r.cancel_request == rid and r.cancel_confirmed is False
    # 取消最终成功（结果 canceled）：撤掉手动接管设的锁停，但巡航不恢复
    r.on_result(rid, 'cancelled')
    assert not r.locked and r.state == 'stopped'
    assert len(nav.dispatched) == 1


# ------------------------------------------------------------------ 5. 生产 launch 传地图
def test_launch_passes_patrol_map_yaml():
    import os
    path = os.path.join(os.path.dirname(W.__file__), '..', '..', 'launch',
                        'robot.launch.py')
    src = open(os.path.normpath(path), encoding='utf-8').read()
    assert "DeclareLaunchArgument(\n            'patrol_map_yaml'" in src
    web = src[src.index("executable='web_debug'"):]
    web = web[:web.index(')') + 1]
    assert 'patrol_map_yaml' in web


# ------------------------------------------------------------------ 6. 运行中换图不被缓存骗过
def test_start_rehashes_map_file(tmp_path):
    yaml = tmp_path / 'm.yaml'
    pgm = tmp_path / 'm.pgm'
    yaml.write_text('image: m.pgm\n')
    pgm.write_bytes(b'P5 1 1 255\x00')
    W.configure_patrol(reset_runner=True, map_yaml=str(yaml))
    first = W._patrol['map_id']
    assert first
    route, _ = P.validate_route([[0, 0], [2, 0]], first, first)
    W._patrol['runner'].save(route)
    W._patrol['license'].accept(7, 0)
    assert W._patrol['runner'].rearm(license_ok=True)[0]
    pgm.write_bytes(b'P5 1 1 255\x01')             # 地图内容变了
    ok, msg = W.patrol_start()
    assert not ok and '地图' in msg
    assert W._patrol['map_id'] != first


# ------------------------------------------------------------------ 7. 换图后重存旧路线被拒
def test_resave_after_map_change_rejected(send_ok):
    W.configure_patrol(reset_runner=True)
    W._patrol['map_id'] = 'old'
    old_route, _ = P.validate_route([[0, 0], [2, 0]], 'old', 'old')
    W._patrol['runner'].save(old_route)
    W._patrol['map_id'] = 'new'
    ok, msg, _frame = W.patrol_save([[0, 0], [1, 0]], map_id='old')
    assert not ok and '地图已更换' in msg
    ok, _msg, _frame = W.patrol_save([[0, 0], [1, 0]], map_id=None)
    assert ok and W._patrol['runner'].route['map_id'] == 'new'


# ------------------------------------------------------------------ 8. 断线原因不被许可失效改写
def test_disconnect_reason_survives_next_tick(send_ok):
    clk = Clock(time.monotonic())
    # 状态机与许可用同一个时钟重建（独立的测试，不依赖用例顺序）
    W._patrol['map_yaml'] = None
    W.configure_patrol(reset_runner=True, now=clk)
    runner = _route_runner('mapA')         # 路线绑定"当前"地图标识
    W._patrol['license'].accept(77, 0)     # 时钟固定：许可一直新鲜
    assert W.patrol_rearm()[0] is True
    ok, info = W.patrol_start()
    assert ok, (info, runner.locked, runner.lock_reason)
    rid = runner.sending
    runner.on_accepted(rid)
    assert runner.state == 'running'
    W.patrol_session_lost('web_disconnected')
    assert runner.locked and runner.lock_reason == 'web_disconnected'
    lic = W._patrol['license']
    clk.tick(2.0)                         # 2 s 没有新许可：许可自己会判过期
    W.patrol_tick()
    assert runner.lock_reason == 'web_disconnected'
    # 页面拿到新许可：fresh 必须能重新变真（断开≠接管，不标 stale）
    sid = lic.session or 99
    ok, _r = lic.accept(sid, 999, clk() + 0.5)
    assert ok and lic.fresh()


# ------------------------------------------------------------------ 9. rearm 文案
def test_rearm_message_matches_state():
    clk = Clock(1.0)
    nav = _Nav()
    r = P.PatrolRunner(nav.dispatch, nav.cancel, nav.lock_stop, clk)
    route, _ = P.validate_route([[0, 0], [2, 0]], 'm', 'm')
    r.save(route)
    r.release_lock()
    r.start('m')                           # sending 未接受
    r.stop('global_stop', user_action=True)
    ok, msg = r.rearm(license_ok=True)
    assert not ok and '取消' in msg         # 确实有未确认的取消
    rid = nav.dispatched[0][0]
    r.on_result(rid, 'cancelled')
    r.pending, r.sending = 77, None        # 只有待清算的已接受目标
    ok, msg = r.rearm(license_ok=True)
    assert not ok and '取消未确认' not in msg


# ------------------------------------------------------------------ 4. 下游通道
class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(getattr(msg, 'data', msg))


class _Node:
    def __init__(self):
        self.pubs = {}

    def create_publisher(self, typ, topic, qos):
        pub = _Pub()
        self.pubs[topic] = pub
        return pub


def test_downstream_license_authorize_rearm_topics(send_ok):
    """许可/鉴权/解停真的发到下游话题；锁停撤回鉴权。"""
    node = _Node()
    W.configure_patrol(reset_runner=True)
    ds = W.patrol_downstream_attach(node)
    if ds['license'] is None or ds['authorize'] is None:
        pytest.skip('本环境没有 std_msgs（假模式）：下游通道为 no-op')
    runner = W._patrol['runner']
    W._patrol['license'].accept(4242, 0)
    W.patrol_license_message({'t': 'motion_license', 'session': 4242,
                              'seq': 1,
                              'expires_mono': W._patrol_now() + 0.5})
    lic_msgs = list(node.pubs[W.LICENSE_TOPIC].sent)
    assert lic_msgs and json.loads(lic_msgs[-1])['session'] == 4242
    assert node.pubs[W.AUTHORIZE_TOPIC].sent[-1] is True
    runner.lock_stop('nav_failed')         # 锁停 → 撤回鉴权、不再转发许可
    assert node.pubs[W.AUTHORIZE_TOPIC].sent[-1] is False
    runner.locked = False
    runner.lock_reason = ''
    ok, _m = W.patrol_rearm()
    assert ok and node.pubs[W.REARM_TOPIC].sent == [True]


def test_runner_default_lock_sink_is_noop_not_none(send_ok):
    W.configure_patrol(reset_runner=True)
    assert callable(W._patrol['lock_sink'])
    W._patrol['runner'].lock_stop('x')     # 不能抛


# ------------------------------------------------------------------ 5. B1 断链复现
def _make_node_record():
    """记录 create_publisher 调用的最小节点替身（复用 _Pub/_Node）。"""
    node = _Node()
    return node, node.pubs


def test_b1_license_forward_survives_bool_missing(monkeypatch, send_ok):
    """[B1 复现] Bool/rclpy 不可用时，/motion_license 转发不许跟着断。

    修复前：attach 的三个 publisher 在同一个 try 里，且每条先求值
    ``Q.status_qos()``——rclpy 缺失（假模式/降级环境）时 qos 就地抛
    ModuleNotFoundError，整个 try 被吞，license publisher 也建不出来，
    patrol_downstream_license 恒 return False，下游运动门永远
    license_expired。
    """
    W.configure_patrol(reset_runner=True)
    node, pubs = _make_node_record()
    # 模拟降级环境：Bool 缺失 + qos 构造需要 rclpy 而不可用
    monkeypatch.setattr(W, '_Bool', None, raising=False)

    def _boom(depth=None):
        raise ModuleNotFoundError("No module named 'rclpy'")

    monkeypatch.setattr(W.Q, 'status_qos', _boom)
    ds = W.patrol_downstream_attach(node)
    assert ds['license'] is not None, \
        'Bool/rclpy 缺失时 license publisher 也必须建出来（B1 断链）'
    assert W.LICENSE_TOPIC in pubs
    W._patrol['license'].accept(4242, 0)
    out = W.patrol_license_message({'t': 'motion_license', 'session': 4242,
                                    'seq': 1,
                                    'expires_mono': W._patrol_now() + 0.5})
    assert out == {'t': 'motion_license_session', 'session': 4242}
    lic_pub = [p for t, p in pubs.items() if t == W.LICENSE_TOPIC]
    assert lic_pub and lic_pub[0].sent, \
        '页面许可必须被转发到 /motion_license（B1）'
    assert json.loads(lic_pub[0].sent[-1])['session'] == 4242
