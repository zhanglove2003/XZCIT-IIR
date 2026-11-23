"""底盘只读状态的纯逻辑测试：里程积分、移动角区、诊断载荷、页面视图、
里程落盘续接。

真正的 ROS 节点需要串口与 rclpy，不在这里覆盖；这里只保证"给定一帧
状态，算出来的数和文案是对的"。
"""
from types import SimpleNamespace

import pytest

from opi_robot import chassis_status as CS
from opi_robot.web_node import console_views as V

def _frame(vx=0, vy=0, wz=0, volt=12100, stop=0):
    return SimpleNamespace(
        chassis_velocity_mm_s=(vx, vy),
        chassis_yaw_rate_mrad_s=wz,
        voltage_mv=volt,
        stop_flag=stop,
    )


# ---- 里程 ----

def test_mileage_first_frame_does_not_count():
    m = CS.MileageIntegrator()
    m.update(500, 10.0)
    assert m.trip() == 0.0


def test_mileage_integrates_trapezoid():
    """20 Hz 下 500 mm/s 走 1 秒 = 0.5 m。"""
    m = CS.MileageIntegrator()
    for i in range(21):
        m.update(500, 10.0 + i * 0.05)
    assert m.trip() == pytest.approx(0.5)


def test_mileage_counts_reverse_as_distance():
    """倒车也增加里程，里程不是位移。"""
    m = CS.MileageIntegrator()
    for i in range(21):
        m.update(-400, 10.0 + i * 0.05)
    assert m.trip() == pytest.approx(0.4)


def test_mileage_skips_long_gap():
    """断流超过 max_step 的那一段不积分，避免凭空跑出几公里。"""
    m = CS.MileageIntegrator(max_step_s=0.5)
    m.update(1000, 10.0)
    m.update(1000, 60.0)
    assert m.trip() == 0.0


def test_mileage_reset_zeroes_trip():
    m = CS.MileageIntegrator()
    for i in range(11):
        m.update(1000, 1.0 + i * 0.05)
    assert m.trip() > 0
    m.reset()
    assert m.trip() == 0.0


# ---- 移动角区 ----

def _circle(r=3.0, n=360):
    return [r] * n


def test_detector_first_scan_has_no_moving():
    det = CS.MovingDetector()
    assert det.feed(_circle(), 0.1, 12.0) == []


def test_detector_static_scene_stays_empty():
    """墙不动就不该报移动：这是"点为什么一直动"的根因。"""
    det = CS.MovingDetector()
    det.feed(_circle(), 0.1, 12.0)
    for _ in range(5):
        assert det.feed(_circle(), 0.1, 12.0) == []


def test_detector_flags_changed_sector():
    det = CS.MovingDetector(bins=120, threshold_m=0.3)
    base = _circle()
    det.feed(base, 0.1, 12.0)
    moved = list(base)
    # 90 度附近（第 90 个点）向前挪 1 m：3 度宽的那一格必须被标出来
    for i in range(88, 93):
        moved[i] = 2.0
    bins = det.feed(moved, 0.1, 12.0)
    # 每格 3 度；点 88..92 落在第 29、30 格，两格都变了
    assert bins == [29, 30]


def test_detector_ignores_invalid_ranges():
    det = CS.MovingDetector()
    det.feed([float('nan')] * 360, 0.1, 12.0)
    assert det.feed(_circle(), 0.1, 12.0) == []


# ---- 诊断载荷 ----

def test_diagnostics_without_frame_is_not_fake_data():
    level, message, values = CS.build_diagnostics(None, 0.0, 0.0)
    keys = dict(values)
    assert keys['state'] == 'unknown'
    assert 'voltage_mv' not in keys
    assert 'vx_mm_s' not in keys
    assert level == 2
    assert message


def test_diagnostics_stale_reason_sets_error_level():
    level, message, values = CS.build_diagnostics(
        None, 1.5, 0.0, stale_reason='底盘状态帧已过期 3.0s')
    assert level == 3
    assert '过期' in message
    assert dict(values)['state'] == 'stale'


def test_diagnostics_keys_match_console_parser():
    """网页 chassis.py 读的是这几个键名，名字对不上就永远显示无数据。"""
    level, message, values = CS.build_diagnostics(
        _frame(vx=420, wz=30, volt=12050), 12.3, 0.42, age_s=0.05, hz=20.0)
    keys = dict(values)
    assert level == 0
    assert int(keys['voltage_mv']) == 12050
    assert int(keys['vx_mm_s']) == 420
    assert int(keys['yaw_rate_mrad_s']) == 30
    assert keys['stop_flag'] == '0'
    assert float(keys['mileage_m']) == pytest.approx(12.3)
    assert '0.42' in message


def test_diagnostics_stop_flag_marks_warning():
    level, message, _ = CS.build_diagnostics(_frame(stop=1), 0.0, 0.0)
    assert level == 1
    assert '失能' in message


# ---- 开机倒计时 ----
# marker 存墙上时钟；monotonic 零点各进程不同，所以测试里显式给 wall_now。

def _boot(path, boot_epoch, now_mono=105.0, wall_now=100.0 + 5.0):
    return V.boot_view(str(path), now_mono, wall_now=wall_now)


def test_boot_view_no_marker(tmp_path):
    view = _boot(tmp_path / 'missing', 100.0)
    assert view['initializing'] is False
    assert view['remain_s'] == 0.0


def test_boot_view_counts_down(tmp_path):
    path = tmp_path / 'boot_hold'
    V.write_boot_marker(str(path), 100.0)
    # 墙钟走了 5 秒 => 单调时钟也走了 5 秒
    view = _boot(path, 100.0, now_mono=1005.0, wall_now=105.0)
    assert view['initializing'] is True
    assert view['remain_s'] == pytest.approx(15.0)
    assert view['hold_s'] == pytest.approx(20.0)


def test_boot_view_counts_down_with_unrelated_monotonic_base(tmp_path):
    """写 marker 的进程和读 marker 的进程单调时钟零点不同，仍要算对。"""
    path = tmp_path / 'boot_hold'
    V.write_boot_marker(str(path), 1_760_000_000.0)
    view = _boot(path, 0.0, now_mono=3.5, wall_now=1_760_000_003.5)
    assert view['remain_s'] == pytest.approx(16.5)


def test_boot_view_ends_after_hold(tmp_path):
    path = tmp_path / 'boot_hold'
    V.write_boot_marker(str(path), 100.0)
    assert _boot(path, 100.0, now_mono=21.0, wall_now=121.0)['initializing'] is False


def test_boot_view_ignores_marker_from_previous_boot(tmp_path):
    """标记是上次开机留下的（时间戳比现在还晚）：当作已结束，不显示负数。"""
    path = tmp_path / 'boot_hold'
    V.write_boot_marker(str(path), 500.0)
    view = _boot(path, 500.0, now_mono=10.0, wall_now=10.0)
    assert view['initializing'] is False


def test_boot_marker_is_atomic_and_readable(tmp_path):
    path = tmp_path / 'boot_hold'
    V.write_boot_marker(str(path), 42.5)
    assert float(path.read_text().strip()) == pytest.approx(42.5)


# ---- 任务状态 ----

def test_task_idle_without_command():
    state, text = V.task_state(False, None, None, None)
    assert state == V.STATE_IDLE
    assert text == '无任务'


def test_task_zero_command_is_not_running():
    """零速命令不算任务在跑。"""
    state, _ = V.task_state(False, 0.0, 0.0, 0.1)
    assert state == V.STATE_IDLE


def test_task_nav_text():
    state, text = V.task_state(True, 0.3, 0.0, 0.1)
    assert state == V.STATE_NAV
    assert text == '导航前往任务目标中'


def test_task_manual_text():
    state, text = V.task_state(True, 0.0, 0.5, 0.1, manual_active=True)
    assert state == V.STATE_MANUAL
    assert text == '手动控制中'


def test_task_stale_command_is_idle():
    state, _ = V.task_state(True, 0.3, 0.0, 5.0)
    assert state == V.STATE_IDLE


def test_task_return_has_priority():
    state, text = V.task_state(True, 0.3, 0.0, 0.1, battery_low=True)
    assert state == V.STATE_RETURN
    assert text == '低电量回充中'


# ---- 电池 ----

def test_battery_invalid_voltage_is_unknown():
    view = V.battery_view('not-a-voltage')
    assert view['voltage_v'] is None
    assert view['percent'] is None
    assert view['text'] == '无数据'


def test_battery_shows_voltage_without_guessing_charge():
    view = V.battery_view(24.456)
    assert view['voltage_v'] == 24.46
    assert view['percent'] is None
    assert view['text'] == '电量未标定'


def test_battery_missing_voltage_is_unknown():
    view = V.battery_view(None)
    assert view['percent'] is None
    assert view['state'] == 'unknown'
    assert view['text'] == '无数据'


# ---- 里程落盘续接 ----

def test_trip_store_saves_and_loads(tmp_path):
    path = str(tmp_path / 'trip')
    store = CS.TripStore(path, wall_now=lambda: 1000.0)
    m = CS.MileageIntegrator()
    m.distance_m = 12.5
    m.speed_m_s = 0.0
    store.save(m)

    m2 = CS.MileageIntegrator()
    saved = CS.TripStore(path, wall_now=lambda: 1060.0).load(m2)
    assert saved == pytest.approx(12.5)
    assert m2.trip() == pytest.approx(12.5)   # 停车时停机，里程不增长


def test_trip_store_adds_distance_while_powered_off(tmp_path):
    """停机前还在跑：补上停机这段按最后车速估算的里程，并标明是估算。"""
    path = str(tmp_path / 'trip')
    m = CS.MileageIntegrator()
    m.distance_m = 10.0
    m.speed_m_s = 0.5
    CS.TripStore(path, wall_now=lambda: 2000.0).save(m)

    m2 = CS.MileageIntegrator()
    m2.speed_m_s = 0.5        # 存档里的车速跟着一起落地，读回时用它补停机里程
    CS.TripStore(path, wall_now=lambda: 2004.0).load(m2)
    assert m2.trip() == pytest.approx(12.0)


def test_trip_store_missing_file_keeps_zero(tmp_path):
    m = CS.MileageIntegrator()
    assert CS.TripStore(str(tmp_path / 'none')).load(m) is None
    assert m.trip() == 0.0


def test_trip_store_ignores_future_timestamp(tmp_path):
    """存档时间是"未来"（改过表）：不补负数里程。"""
    path = str(tmp_path / 'trip')
    with open(path, 'w') as fh:
        fh.write('5.0 9999.0\n')
    m = CS.MileageIntegrator()
    m.speed_m_s = 1.0
    CS.TripStore(path, wall_now=lambda: 1000.0).load(m)
    assert m.trip() == pytest.approx(5.0)
