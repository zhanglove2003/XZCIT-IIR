"""控制台新增能力的测试：底盘视图、假数据源、状态载荷扩展。

只测纯逻辑与本地合成的部分。真实话题链路需要 ROS 硬件，不在此覆盖。
"""
import base64
import json
import zlib
import math
import os
import re
import sys
import time

import pytest

import fake_ros as fake_ros_mod

from opi_robot.web_node import chassis as C
from opi_robot.web_node import fake_source as F
from opi_robot.web_node import web_debug as W


@pytest.fixture
def fake_ros():
    """与 test_web_bridge 同款：伪 ROS 已由 conftest 装好，这里只做引用。"""
    return fake_ros_mod


@pytest.fixture(autouse=True)
def clean_state():
    W.reset_state()
    yield
    W.reset_state()


# ---- 底盘视图 ----

def test_true_wheel_speed_straight():
    """直行时左右轮组同速，等于车体速度。"""
    left, right = C.true_wheel_speed(0.5, 0.0)
    assert left == pytest.approx(0.5)
    assert right == pytest.approx(0.5)


def test_true_wheel_speed_turn_uses_sum_of_half_levers():
    """左转：左右轮差等于 2*wz*(半轮距+半轴距)。"""
    left, right = C.true_wheel_speed(0.0, 1.0)
    lever = C.HALF_WHEEL_SPACING_M + C.HALF_AXLE_SPACING_M
    assert left == pytest.approx(-lever)
    assert right == pytest.approx(lever)


def test_wheel_rpm_matches_circumference():
    """轮速换算与轮周长一致：1 圈/秒 = 1*周长 m/s = 60 rpm。"""
    rpm = C.wheel_rpm(C.WHEEL_CIRCUMFERENCE_M)
    assert rpm == pytest.approx(60.0, abs=0.01)


def test_wheel_view_marks_estimate_not_per_wheel():
    """必须标出「推算」且「非单轮」，不能让界面看着像实测。"""
    view = C.wheel_view(0.4, 0.2)
    assert view['estimated'] is True
    assert view['per_wheel'] is False
    assert view['note']
    assert view['turn_lever_m'] == pytest.approx(0.43, abs=1e-6)


def test_chassis_view_unknown_without_data():
    view = C.chassis_view(None, None, now=100.0)
    assert view['state'] == 'unknown'
    assert view['voltage_v'] is None
    assert view['wheels'] is None


def test_chassis_view_stale_drops_values():
    """过期后不显示旧值，避免把停更数据当实时。"""
    data = {'voltage_mv': 12100, 'vx_mm_s': 500, 'vy_mm_s': 0,
            'yaw_rate_mrad_s': 0, 'stop_flag': 0, 'level': 0,
            'message': 'ok'}
    view = C.chassis_view(data, 100.0, now=200.0, timeout=2.0)
    assert view['state'] == 'stale'
    assert view['voltage_v'] is None
    assert view['vx_m_s'] is None


def test_chassis_view_fresh_converts_units():
    data = {'voltage_mv': 12100, 'vx_mm_s': 500, 'vy_mm_s': -20,
            'yaw_rate_mrad_s': 250, 'stop_flag': 0, 'level': 0,
            'message': 'ok'}
    view = C.chassis_view(data, 100.0, now=100.5, timeout=2.0)
    assert view['state'] == 'fresh'
    assert view['voltage_v'] == pytest.approx(12.1)
    assert view['vx_m_s'] == pytest.approx(0.5)
    assert view['vy_m_s'] == pytest.approx(-0.02)
    assert view['wz_rad_s'] == pytest.approx(0.25)
    assert view['drive_ready'] is None
    assert view['stop_text'] == '软件未失能；就绪未知'
    assert view['wheels'] is not None


def test_chassis_view_stop_flag_text():
    data = {'voltage_mv': 12100, 'vx_mm_s': 0, 'vy_mm_s': 0,
            'yaw_rate_mrad_s': 0, 'stop_flag': 1, 'level': 0,
            'message': 'ok'}
    view = C.chassis_view(data, 100.0, now=100.2, timeout=2.0)
    assert view['stop_text'] == '软件失能'
    assert view['drive_ready'] is False


def test_parse_chassis_status_from_diagnostics(fake_ros):
    """只有已知名称的诊断项才算底盘状态，其它项忽略。"""
    def kv(k, v):
        return fake_ros.KeyValue(key=k, value=v)

    msg = fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(
            level=0, name='lidar', message='ok',
            values=[kv('voltage_mv', '1')]),
        fake_ros.DiagnosticStatus.make(
            level=0, name='chassis', message='run',
            values=[kv('voltage_mv', '11900'), kv('vx_mm_s', '420'),
                    kv('vy_mm_s', '0'), kv('yaw_rate_mrad_s', '30'),
                    kv('stop_flag', '0')]),
    ])
    parsed = C.parse_chassis_status(msg)
    assert parsed['voltage_mv'] == 11900
    assert parsed['vx_mm_s'] == 420
    assert parsed['stop_flag'] == 0


def test_parse_chassis_status_ignores_unrelated():
    assert C.parse_chassis_status(None) is None


def test_safety_view_normalises_values():
    view = C.safety_view({'drive_enable': 'true',
                          'drive_input_enable': 'false',
                          'drive_firmware_protocol': 'v3'}, '在线节点')
    assert view['drive_enable'] is True
    assert view['drive_input_enable'] is False
    assert view['drive_firmware_protocol'] == 'v3'
    assert view['source'] == '在线节点'


def test_cmd_view_expires():
    class _Twist:
        class linear:
            x = 0.3
        class angular:
            z = 0.1
    view = C.cmd_view(_Twist(), 100.0, now=100.2, timeout=1.0)
    assert view['state'] == 'fresh'
    assert view['vx_m_s'] == pytest.approx(0.3)
    stale = C.cmd_view(_Twist(), 100.0, now=105.0, timeout=1.0)
    assert stale['state'] == 'stale'
    assert stale['vx_m_s'] is None


def test_cmd_view_without_topic():
    view = C.cmd_view(None, None, now=100.0)
    assert view['state'] == 'unknown'


# ---- 假数据源 ----

def test_fake_source_scan_is_json_safe():
    source = F.FakeSource(t0=0.0)
    pts = source.scan(0.0)
    assert len(pts) > 100
    assert all(isinstance(d, float) and isinstance(r, float) for d, r in pts)
    assert all(r >= 0 for _, r in pts)
    json.dumps(pts, allow_nan=False)


def test_fake_source_scan_hits_room_walls():
    """最短距离不能小于房间半宽，说明点确实落在墙上。"""
    source = F.FakeSource(t0=0.0)
    pts = source.scan(1.0)
    radii = [r for _, r in pts if r > 0]
    assert min(radii) > 1.0
    assert max(radii) < 12.0


def test_fake_source_status_cycles_through_phases():
    """整圈演示里必须同时出现使能与停车两种状态。"""
    source = F.FakeSource(t0=0.0)
    stops = set()
    for i in range(0, 24):
        status = source.status(float(i))
        stops.add(status['stop_flag'])
    assert stops == {0, 1}


def test_fake_source_status_fields():
    source = F.FakeSource(t0=0.0)
    status = source.status(0.0)
    for key in ('voltage_mv', 'vx_mm_s', 'vy_mm_s', 'yaw_rate_mrad_s',
                'stop_flag'):
        assert key in status
    json.dumps(status, allow_nan=False)


def test_fake_source_image_is_png_with_expected_size():
    source = F.FakeSource(t0=0.0)
    img = source.image(0.0)
    assert img is not None
    assert img[:8] == b'\x89PNG\r\n\x1a\n'
    w, h = W._image_size(img)
    assert (w, h) == (F.CAM_W, F.CAM_H)


def test_fake_source_image_rate_limited():
    """按 image_period 节流：同一时刻连续取图只第一帧有数据。"""
    source = F.FakeSource(t0=0.0)
    assert source.image(0.0) is not None
    assert source.image(0.01) is None


# ---- 状态载荷扩展 ----

def test_status_payload_has_console_sections(fake_ros):
    bridge = W.Bridge()
    payload = W.status_payload(now=time.time())
    assert payload['t'] == 'status'
    for key in ('chassis', 'imu', 'cmd', 'safety', 'mode'):
        assert key in payload
    assert payload['cmd']['nav']['state'] == 'unknown'
    json.dumps(payload, allow_nan=False)
    assert bridge.subscriptions


def test_status_payload_carries_srv_epoch(fake_ros):
    """进程纪元随载荷下发：页面据此发现服务端重启并整页自刷新。

    2026-11-15 现场：web_debug 重启后旧页面的控制 WS 已自愈（显示
    "已连接"），但视频重连链死在后台标签页的定时器节流里——"等待画面"
    永不出图。纪元比对让页面识别这种情况并自愈。
    """
    payload = W.status_payload(now=time.time())
    epoch = payload['srv_epoch']
    assert isinstance(epoch, int) and epoch > 0
    assert epoch == W.SRV_EPOCH
    # 同进程内稳定：页面用相等判定"没重启"，纪元抖动会造成无限刷新
    assert W.status_payload(now=time.time())['srv_epoch'] == epoch


def test_console_page_wires_srv_epoch_reload(fake_ros):
    """页面必须消费 srv_epoch 并带一次性刷新守卫，防载荷与页面漂移。"""
    assert 'srv_epoch' in W.PAGE
    assert '__srvReloading' in W.PAGE


def test_on_map_stores_grid_snapshot(fake_ros):
    """占据栅格回调：尺寸/分辨率/原点入共享状态，data 转 bytes。"""
    W.reset_state()
    bridge = W.Bridge()
    grid = fake_ros.nav_msgs_msg.OccupancyGrid()
    grid.info.width = 3
    grid.info.height = 2
    grid.info.resolution = 0.05
    grid.info.origin.position.x = 1.0
    grid.info.origin.position.y = -2.0
    grid.data = [0, 100, -1, 50, 0, -1]
    bridge.on_map(grid)
    snap = W._state['map_grid']
    assert snap['w'] == 3 and snap['h'] == 2 and snap['res'] == 0.05
    assert snap['ox'] == 1.0 and snap['oy'] == -2.0
    # -1（未知）转 0xFF，其余原值
    assert snap['data'] == bytes([0, 100, 0xFF, 50, 0, 0xFF])
    assert W._state['map_t'] is not None


def test_map_frame_json_roundtrip(fake_ros):
    """地图帧：zlib+base64 可还原原始栅格，位姿与年龄随帧携带。"""
    grid = {'seq': 7, 'w': 2, 'h': 1, 'res': 0.05,
            'ox': 1.0, 'oy': 2.0, 'data': bytes([0, 100])}
    now = time.time()
    text = W._map_frame_json(grid, now - 2.0,
                             {'x': 0.5, 'y': -0.5, 'th': 0.25}, now - 0.5)
    assert text is not None
    frame = json.loads(text)
    assert frame['t'] == 'map' and frame['seq'] == 7
    raw = zlib.decompress(base64.b64decode(frame['data']))
    assert raw == bytes([0, 100])
    assert frame['pose']['th'] == 0.25
    assert frame['age'] == pytest.approx(2.0, abs=0.05)
    assert frame['pose_age'] == pytest.approx(0.5, abs=0.05)


def test_console_page_has_map_section(fake_ros):
    """地图页三要素：区块/画布/解压路径（DecompressionStream）。"""
    for needle in ('id="page-map"', 'id="mapcanvas"', 'DecompressionStream',
                   "id=\"us_big\"", "'map'"):
        assert needle in W.PAGE


def test_bridge_subscribes_to_imu_and_commands(fake_ros):
    """控制台依赖这三个话题做心跳与命令显示。"""
    bridge = W.Bridge()
    for topic in ('imu/data_raw', 'nav_cmd_vel', 'demo_cmd_vel'):
        assert topic in bridge.subscriptions


def test_bridge_on_imu_updates_heartbeat(fake_ros):
    W.reset_state()
    bridge = W.Bridge()
    before = W._state['imu_t']
    bridge.on_imu(fake_ros.Imu())
    assert W._state['imu_t'] is not None
    assert before is None
    payload = W.status_payload(now=time.time())
    assert payload['imu']['state'] in ('fresh', 'unknown')


def test_bridge_on_cmd_records_twist(fake_ros):
    W.reset_state()
    bridge = W.Bridge()
    twist = fake_ros.Twist()
    twist.linear.x = 0.25
    twist.angular.z = -0.5
    bridge.on_cmd('nav', twist)
    payload = W.status_payload(now=time.time())
    assert payload['cmd']['nav']['vx_m_s'] == pytest.approx(0.25)
    assert payload['cmd']['nav']['wz_rad_s'] == pytest.approx(-0.5)


def test_bridge_records_chassis_diagnostics(fake_ros):
    W.reset_state()
    bridge = W.Bridge()

    def kv(k, v):
        return fake_ros.KeyValue(key=k, value=v)

    msg = fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(
            level=0, name='chassis', message='run',
            values=[kv('voltage_mv', '12050'), kv('vx_mm_s', '400'),
                    kv('vy_mm_s', '0'), kv('yaw_rate_mrad_s', '0'),
                    kv('stop_flag', '0')]),
    ])
    bridge.on_diagnostics(msg)
    payload = W.status_payload(now=time.time())
    assert payload['chassis']['state'] == 'fresh'
    assert payload['chassis']['voltage_v'] == pytest.approx(12.05)
    assert payload['chassis']['wheels'] is not None


def test_bridge_safety_uses_local_parameters(fake_ros):
    """没有参数服务时用本地启动参数，来源要写明。"""
    W.reset_state()
    bridge = W.Bridge()
    payload = W.status_payload(now=time.time())
    safety = payload['safety']
    assert safety['drive_enable'] is False
    assert safety['drive_firmware_protocol'] == 'baseline'
    assert '本地' in safety['source']


# ---- 假数据模式 ----

def test_fake_enabled_from_env(monkeypatch):
    monkeypatch.delenv('OPI_WEB_FAKE', raising=False)
    assert W.fake_enabled(argv=[]) is False
    assert W.fake_enabled(argv=['--fake']) is True
    monkeypatch.setenv('OPI_WEB_FAKE', '1')
    assert W.fake_enabled(argv=[]) is True
    monkeypatch.setenv('OPI_WEB_FAKE', '0')
    assert W.fake_enabled(argv=[]) is False


def test_fake_bridge_fills_state_without_ros(fake_ros):
    """假模式必须产出全部五路数据，页面才不留空。"""
    W.reset_state()
    bridge = W.FakeBridge()
    try:
        bridge.start()
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if (W._state['scan'] is not None
                    and W._state['image_b64']
                    and W._state['chassis'] is not None):
                break
            time.sleep(0.05)
        payload = W.status_payload(now=time.time())
        assert payload['mode'] == 'fake'
        assert payload['chassis']['state'] == 'fresh'
        assert payload['chassis']['voltage_v'] > 10.0
        assert payload['scan'] == 'fresh'
        assert payload['image'] == 'fresh'
        assert payload['imu']['state'] == 'fresh'
        assert payload['safety']['source'] == '假数据模式'
        json.dumps(payload, allow_nan=False)
    finally:
        bridge.stop()


def test_page_has_no_external_resources():
    """零外部依赖：不许出现 CDN、npm 包或外网字体。

    先去掉 data: URI，再查协议。内联 favicon 用的是 data:image/svg+xml，
    它的 xmlns 里含 http://www.w3.org/2000/svg，那是命名空间不是请求。
    """
    page = W.PAGE
    stripped = re.sub(r'"data:[^"]*"', '"data:"', page)
    for bad in ('http://', 'https://', 'cdn.', 'unpkg', 'jsdelivr',
                'googleapis', '@import'):
        assert bad not in stripped, bad
    # src/href 只允许三种：空、data: 内联、页内 hash 路由
    for attr in re.findall(r'(?:src|href)="([^"]*)"', page):
        assert (attr == '' or attr.startswith('data:')
                or attr.startswith('#/')), attr


def test_page_declares_console_elements():
    """关键元素 id 必须在页面里，截图与前端回归都依赖它们。"""
    page = W.PAGE
    for element_id in (
            # 公共
            'mode', 'loglist', 'pageTitle',
            # 控制页：自动巡航开关与状态
            'cruise-start', 'cruise-stop', 'cruise-state',
            # 概览：底盘只要车速与里程，电池单独一张卡
            'ov_ch_led', 'ov_ch_vx', 'ov_ch_kmh', 'ov_ch_mile',
            'ov_bat_pct', 'ov_bat_volt', 'ov_bat_bar',
            # 概览：主机环形仪表
            'ring_cpu', 'ring_mem', 'ring_disk', 'ring_temp',
            'ov_cpu', 'ov_mem', 'ov_disk', 'ov_temp',
            # 概览：节点呼吸灯、任务、网络、开机倒计时
            'nd_cam', 'nd_lidar', 'nd_bridge', 'nd_chassis', 'nd_web',
            'nd_imu', 'task_text', 'task_led', 'boot', 'boot_count',
            'ov_net_ssid', 'ov_net_ip', 'ov_net_bars',
            # 控制页
            'manual-login', 'manual-content', 'manual-stop',
            'pad-fwd', 'pad-left', 'pad-right', 'pad-rev',
            'manual-timed', 'manual-duration', 'manual-direction',
            'cam', 'radar', 'scanstate', 'cam_badge', 'cam_empty'):
        assert 'id="' + element_id + '"' in page, element_id


def test_page_has_three_routed_pages():
    """三页必须都在，且每页都有一个对应的侧边栏入口。"""
    page = W.PAGE
    for name in ('overview', 'control', 'logs'):
        assert 'id="page-' + name + '"' in page, name
        assert 'data-page="' + name + '"' in page, name
    assert 'id="page-sensors"' not in page
    assert "location.replace('#/overview')" in page


def test_page_drops_removed_fields():
    """用户删掉的字段不能被新页面带回来：雷达帧龄/已接受、安全开关读数。"""
    page = W.PAGE
    for gone in ('ov_safety_drive', 'ov_safety_input', 'sen_frames_ok',
                 'sen_frames_bad', 'ov_hb_scan'):
        assert gone not in page, gone


def test_page_stop_button_is_emergency_stop():
    """按钮文案是「紧急停车」，发的仍是零速命令。"""
    page = W.PAGE
    assert '<button class="estop" id="manual-stop"' in page
    assert '紧急停车' in page
    assert 'manualSend({ t: \'manual_stop\' })' in page


def test_page_gates_live_stream_to_control():
    """实时流只跟实时页走，离开即发 idle，避免概览页占带宽。

    2026-11-15 起"看/开"分离：地图页只读、免登录即开视频连接；
    控制页（驾驶）仍须 manualAuthenticated，摄像头只推给已登录会话。
    """
    page = W.PAGE
    assert 'sendView' in page
    # 地图分支不查 manualAuthenticated；控制分支保留鉴权门
    assert "if (name === 'map') {" in page
    assert "} else if (name === 'control' && manualAuthenticated) {" in page
    assert "sendView(name)" in page
    assert "sendView('idle')" in page
    assert "sendView(curPage === 'map' ? 'map' : 'control')" in page
    assert 'function sendView' in page
    # 1 秒一次的心跳：弱网丢一两包也不会被判成离线
    assert 'VIEW_BEAT_MS = 1000' in page


def test_page_freezes_static_radar_points():
    """静止点冻结：每个角度格保持原距离，超过阈值才更新。"""
    page = W.PAGE
    assert 'HOLD_M' in page
    assert 'Math.abs(r - old) > HOLD_M' in page
    assert 'POLAR = 720' in page


def test_websocket_defaults_to_no_image():
    """控制连接不推图像/点云；视频连接里摄像头以 _live_controls 为闸。

    2026-11-14 控制与视频分离（T4）：img/scan 只在 /ws/video 的
    ``_video_loop`` 里推。2026-11-15"看/开"分离：scan/map 免登录直推
    （地图页公开只读），仅摄像头仍等 _live_controls（已登录控制页）。
    """
    import inspect
    source = inspect.getsource(W.Handler._websocket)
    # 控制连接：不再承载图像与点云
    assert "'t': 'img'" not in source
    assert "'t': 'scan'" not in source
    video = inspect.getsource(W.Handler._video_loop)
    # 摄像头闸：无已登录控制页会话时不推 img
    assert ('if (seq != last_img and b64 and now >= next_img_t\n'
            '                        and _live_controls):') in video
    assert 'pts is not None and sframes != last_scan' in video
    # 雷达 10 Hz 节流：100 Hz 原始扫描不能整帧往弱网推
    assert 'scan_period = 0.1' in video


def _view_frame(payload):
    data = json.dumps(payload).encode('utf-8')
    mask = b'\x01\x02\x03\x04'
    body = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
    return bytes([0x81, 0x80 | len(data)]) + mask + body


class _FakeSock:
    """最小套接字替身：_read_view 走 recv + select。"""

    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.sent = []

    def recv(self, _n):
        return self.chunks.pop(0) if self.chunks else b''

    def sendall(self, data):
        self.sent.append(data)


def _new_handler(chunks):
    handler = W.Handler.__new__(W.Handler)
    handler.connection = _FakeSock(chunks)
    handler._ws_buf = bytearray()
    # 不设 _ws_drained：替身没有 rfile，_take_buffered 返回空串，
    # 随后照常走 recv。设成 True 会让第一次 recv 被跳过，控制帧丢失。
    return handler


def test_read_view_switches_live_flag():
    """_read_view 只认 view 消息：control 打开，其余关闭。"""
    handler = _new_handler([_view_frame({'t': 'view', 'page': 'control'})])
    # 浏览器连接的身份就是 handler 本身，登录与心跳必须同一个对象
    assert W._MANUAL.login(handler, 'admin', 'password') is True
    try:
        live, next_t, _ = handler._read_view(False, 0.0)
        assert live is True
        assert next_t >= time.time()

        live2, _, _ = handler._read_view(True, 0.0)
        assert live2 is False
    finally:
        W._MANUAL.disconnect(handler)


def test_read_view_rejects_control_without_login():
    """未登录请求实时画面：不推流，并回一条 manual_error。"""
    handler = _new_handler([_view_frame({'t': 'view', 'page': 'control'})])
    live, _, _ = handler._read_view(False, time.time() + 30)
    assert live is False
    frames = b''.join(handler.connection.sent)
    assert b'manual_error' in frames


def test_read_view_allows_map_without_login():
    """地图页公开只读（2026-11-15"看/开"分离）：未登录也能看。

    live 保持 True（切到地图页不打断手动会话），但不回 manual_error、
    不登记 _live_controls（摄像头不推给地图页）。
    """
    handler = _new_handler([_view_frame({'t': 'view', 'page': 'map'})])
    live, next_t, _ = handler._read_view(False, 0.0)
    assert live is True
    assert next_t >= time.time()
    assert handler.connection.sent == []          # 不提示登录
    assert handler not in W._live_controls        # 摄像头闸不开
    assert getattr(handler, '_ws_page', None) == 'map'
    # 切回控制页才进摄像头闸
    handler.connection.chunks.append(
        _view_frame({'t': 'view', 'page': 'control'}))
    live2, _, _ = handler._read_view(True, 0.0)
    assert live2 is False                         # 未登录，拒绝
    assert handler not in W._live_controls
    # 登录后切控制页：登记
    assert W._MANUAL.login(handler, 'admin', 'password') is True
    try:
        handler.connection.chunks.append(
            _view_frame({'t': 'view', 'page': 'control'}))
        live3, _, _ = handler._read_view(False, time.time() + 30)
        assert live3 is True
        assert handler in W._live_controls
        # 切到地图页：仍是实时页，但摄像头闸关闭
        handler.connection.chunks.append(
            _view_frame({'t': 'view', 'page': 'map'}))
        live4, _, _ = handler._read_view(True, time.time() + 30)
        assert live4 is True
        assert handler not in W._live_controls
    finally:
        W._MANUAL.disconnect(handler)


def test_handshake_drains_rfile_into_ws_buf():
    """握手时 rfile 已缓冲的控制帧必须直接进 _ws_buf。

    浏览器把"请求头 + 登录帧"一次 sendall，BaseHTTPRequestHandler 的
    rfile.readline 会把登录帧一起读进 64 KB 缓冲。此后 socket 上没有
    可读字节，select 永远不命中，只在 _read_view 里取 rfile 就永远取不到。
    """
    src = open(W.__file__, encoding='utf-8').read()
    start = src.index('    def _websocket(self, video=False):')
    end = src.index('    def _read_view(self', start)
    body = src[start:end]
    assert '_ws_buf = bytearray(_take_buffered(self))' in body
    assert 'self._ws_drained = True' in body


def test_ws_log_records_manual_login_without_password():
    """控制帧日志记一条 manual_login，但不能落明文密码。"""
    handler = _new_handler(
        [_view_frame({'t': 'manual_login', 'username': 'admin',
                      'password': 'password'})])
    handler._read_view(False, 0.0)
    items = W.ws_log_snapshot()['items']
    rx = [it for it in items if it['kind'] == 'rx'
          and it['detail'].get('t') == 'manual_login']
    assert rx, items
    # 只留字段名，不留字段值：清单里有 password 这个名字是对的，
    # 但它的值不能出现在日志里。
    detail = rx[-1]['detail']
    assert detail['fields'] == ['password', 't', 'username']
    assert set(detail) == {'t', 'fields'}


def test_ws_log_marks_stale_after_frames():
    """收到过帧以后 last_frame_age 必须是数字，供 /api/wslog 判断。"""
    handler = _new_handler([_view_frame({'t': 'view', 'page': 'idle'})])
    handler._read_view(False, 0.0)
    age = W.ws_log_snapshot()['last_frame_age']
    assert age is not None and age >= 0


def test_read_view_stops_after_timeout():
    """超过 CLIENT_TIMEOUT 没有任何消息：按离开控制页处理。"""
    handler = _new_handler([])
    assert W._MANUAL.login(handler, 'admin', 'password') is True
    try:
        live, _, _ = handler._read_view(True, time.time() - 1.0)
        assert live is False
    finally:
        W._MANUAL.disconnect(handler)


# ---- 巡航开关（服务端） ----

def _cruise_frame(on):
    return _view_frame({'t': 'cruise_set', 'on': on})


def test_cruise_publisher_declared_on_bridge(fake_ros):
    """发布器在 __init__ 里建，话题就是 fusion_node 读的 /cruise_enable。"""
    bridge = W.Bridge()
    assert 'cruise_enable' in bridge.publishers
    assert W.cruise_view()['available'] is True


def test_cruise_set_reaches_server(fake_ros):
    """cruise_set 到达服务端：状态保存，并向 /cruise_enable 发 Bool。"""
    bridge = W.Bridge()
    handler = _new_handler([_view_frame({'t': 'view', 'page': 'control'}),
                            _cruise_frame(True)])
    assert W._MANUAL.login(handler, 'admin', 'password') is True
    try:
        live, _, _ = handler._read_view(False, 0.0)
        assert live is True
    finally:
        W._MANUAL.disconnect(handler)
    view = W.cruise_view()
    assert view['on'] is True
    assert view['available'] is True
    sent = bridge.publishers['cruise_enable'].messages
    assert len(sent) == 1
    assert isinstance(sent[0], fake_ros.Bool)
    assert sent[0].data is True
    # 关闭也走同一条路
    handler2 = _new_handler([_cruise_frame(False)])
    assert W._MANUAL.login(handler2, 'admin', 'password') is True
    try:
        handler2._read_view(True, time.time() + 30)
    finally:
        W._MANUAL.disconnect(handler2)
    assert W.cruise_view()['on'] is False
    assert bridge.publishers['cruise_enable'].messages[-1].data is False


def test_cruise_set_requires_login(fake_ros):
    """未登录不能开巡航：回一条 manual_error，状态保持关闭。"""
    bridge = W.Bridge()
    handler = _new_handler([_cruise_frame(True)])
    handler._read_view(True, time.time() + 30)
    frames = b''.join(handler.connection.sent)
    assert b'manual_error' in frames
    assert W.cruise_view()['on'] is False
    assert bridge.publishers['cruise_enable'].messages == []


def test_cruise_status_frame_carries_state(fake_ros):
    """状态帧必须带 cruise 字段，页面据此显示巡航中 / 已停止。"""
    W.Bridge()
    payload = W.status_payload(now=time.time())
    assert payload['cruise'] == {'on': False, 'available': True}


def test_cruise_degrades_without_publisher(fake_ros, monkeypatch):
    """发布器缺失时置 None：服务端不崩，按钮显示不可用。"""
    monkeypatch.setattr(W, '_Bool', None)
    W.reset_state()
    bridge = W.Bridge()
    assert W.cruise_view()['available'] is False
    handler = _new_handler([_view_frame({'t': 'view', 'page': 'control'}),
                            _cruise_frame(True)])
    assert W._MANUAL.login(handler, 'admin', 'password') is True
    try:
        handler._read_view(False, 0.0)
    finally:
        W._MANUAL.disconnect(handler)
    view = W.cruise_view()
    assert view['on'] is False
    assert view['available'] is False
    assert payload_cruise(W)['available'] is False
    assert bridge is not None


def payload_cruise(module):
    return module.status_payload(now=time.time())['cruise']



def test_image_size_reads_png(fake_ros):
    img = F.FakeSource(t0=0.0).image(0.0)
    assert W._image_size(img) == (F.CAM_W, F.CAM_H)
    assert W._sniff_mime(img) == 'image/png'


def test_image_size_rejects_garbage():
    assert W._image_size(b'') == (0, 0)


def test_load_page_covers_installed_layout(tmp_path):
    """安装布局下 console_page.html 在 lib/opi_robot/web_node/，必须能找到。

    模块实际位于 lib/python3.10/site-packages/opi_robot/web_node/，页面在
    lib/opi_robot/web_node/。路径算错时服务会静默退回兜底页（只剩原始
    JSON），车上就漏过一次，所以这里按真实目录层级构造。
    """
    pkg = tmp_path / 'lib' / 'python3.10' / 'site-packages' / 'opi_robot' / 'web_node'
    pkg.mkdir(parents=True)
    page = tmp_path / 'lib' / 'opi_robot' / 'web_node'
    page.mkdir(parents=True)
    (page / 'console_page.html').write_text('<html>installed-page</html>',
                                            encoding='utf-8')
    cands = W._page_candidates(str(pkg))
    assert str(page / 'console_page.html') in cands

# ---- 自动巡航 ----
# 巡航只发开关：速度由 fusion_node 按 0.4 m/s 走，网页不发自动驾驶速度。

def test_page_declares_cruise_elements():
    """巡航两个按钮与状态位必须在页面里。"""
    page = W.PAGE
    for element_id in ('cruise-start', 'cruise-stop', 'cruise-state'):
        assert 'id="' + element_id + '"' in page, element_id
    assert '>自动巡航<' in page
    assert '>停车<' in page


def test_page_sends_cruise_set_on_and_off():
    """两个按钮分别发 on=true 与 on=false。"""
    page = W.PAGE
    assert "manualSend({ t: 'cruise_set', on: true })" in page
    assert "manualSend({ t: 'cruise_set', on: false })" in page


def test_page_direction_keys_drop_cruise_first():
    """方向键按下时若巡航在开，先关巡航再发手动。

    两个来源同时写 /nav_cmd_vel 会抢速度，所以必须先关。
    """
    page = W.PAGE
    assert 'function cruiseReleaseForManual' in page
    assert 'cruiseReleaseForManual();' in page
    start = page.index('function cruiseReleaseForManual')
    body = page[start:start + 260]
    assert "manualSend({ t: 'cruise_set', on: false })" in body
    # 方向键处理里必须先关、再进入 manualStart
    hook = page.index('cruiseReleaseForManual();')
    assert page.index("manualStart('hold'") > hook


def test_page_shows_cruise_state_text():
    """状态回显三种：巡航中 / 已停止 / 雷达不可信。"""
    page = W.PAGE
    assert '巡航中' in page
    assert '已停止' in page
    assert '雷达不可信' in page
    assert 'function applyCruise' in page

