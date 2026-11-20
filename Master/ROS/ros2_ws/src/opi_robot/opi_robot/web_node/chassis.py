"""底盘状态视图：把 24 字节状态帧的字段整理成可直接渲染的显示值。

本模块是纯逻辑，不 import rclpy，可在没有 ROS 的机器上单独测试。

数据来源（全部只读引用，不改动 c50c_uart3 与固件）：

- 帧字段偏移：``c50c_uart3/c50c_uart3/frame_parser.py:53-67``。
- 电压字段 ``frame[20:22]`` 大端 mV：
  ``c50c_uart3/c50c_uart3/frame_parser.py:148`` 与
  ``c50c_uart3/c50c_uart3/chassis_cmd/baseline.py:95``。
- 四驱运动学反解：``project/C50C_gcc/09_Balance/balance_task.c:822-827``
  （``InverseKinematics_4wd``）。
- 四驱运动学正解：``project/C50C_gcc/09_Balance/data_task.c:343-352``
  （``Kinematics_mec_4wd``）。
- 车型几何常量：``project/C50C_gcc/08_CarType/Inc/robot_init.h:169-190``，
  本车取 ``case 2``（``4wd_robot_init.c:31``）；取值依据见
  ``reviews/2026-11-04-c50c-safety/docs/HANDOVER-2026-11-05-c50c-4wd.md:202``。

重要限制：车轮速度是**推算值**，不是实测值。状态帧只给车体三轴速度，
固件把左右两侧各两个轮视为同速（``A=B``、``C=D``），所以只能还原
"左轮组"与"右轮组"两个数值，不是四个独立轮速。参数未经 ROS 现场标定，
详见 ``EV/console/console-report.md``。
"""
from __future__ import annotations

import math

from . import sensors as S

# ---- 本车几何参数（robot.type = 2，四驱高配独立悬挂 + MD36N_27）----
# robot_init.h:170,177 -> SENIOR_4WD_DL_wheelspacing / _axlespacing
HALF_WHEEL_SPACING_M = 0.230
HALF_AXLE_SPACING_M = 0.200
# robot_init.h:186 -> WheelDiameter_4WD_152
WHEEL_DIAMETER_M = 0.152
# robot_init.h:78 -> MD36N_27
GEAR_RATIO = 27.0
# robot_init.h:71 -> GMR_500
ENCODER_LINES = 500
# 四驱反解把两个半量相加后直接乘角速度（balance_task.c:824-827）
TURN_LEVER_M = HALF_WHEEL_SPACING_M + HALF_AXLE_SPACING_M

WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M

# 诊断里可能使用的底盘状态名。任一命中即视为底盘状态源。
CHASSIS_NAMES = ('chassis', 'uart3', 'c50c_uart3', 'c50c')

# 数值字段 -> 诊断 KeyValue 键名候选
_KEYS = {
    'voltage_mv': ('voltage_mv', 'voltage', 'battery_mv'),
    'vx_mm_s': ('vx_mm_s', 'velocity_x', 'chassis_velocity_x'),
    'vy_mm_s': ('vy_mm_s', 'velocity_y', 'chassis_velocity_y'),
    'yaw_rate_mrad_s': ('yaw_rate_mrad_s', 'yaw_rate', 'chassis_yaw_rate'),
    'stop_flag': ('stop_flag',),
    # [C50C 2026-11-15] 前置 HC-SR04 超声波距离（mm；0xFFFF=无回波/陈旧）
    'ultrasonic_front_mm': ('ultrasonic_front_mm',),
}

GEOMETRY_NOTE = '车轮速度由车体速度反解，参数未现场标定'


def parse_chassis_status(msg):
    """从 DiagnosticArray 取底盘状态，返回字典或 None。

    认不出任何已知名称时返回 None，不猜测。
    """
    if msg is None:
        return None
    for status in getattr(msg, 'status', []) or []:
        name = getattr(status, 'name', '') or ''
        if name.lower() not in CHASSIS_NAMES:
            continue
        values = {}
        for kv in getattr(status, 'values', []) or []:
            values[str(kv.key).lower()] = kv.value
        out = {}
        for field, candidates in _KEYS.items():
            out[field] = None
            for key in candidates:
                if key in values:
                    out[field] = (_to_int(values[key]) if field in
                                  ('voltage_mv', 'stop_flag', 'vx_mm_s',
                                   'vy_mm_s', 'yaw_rate_mrad_s',
                                   'ultrasonic_front_mm')
                                  else _to_float(values[key]))
                    break
        out['level'] = status.level
        out['message'] = getattr(status, 'message', '') or ''
        return out
    return None


def _to_int(text):
    try:
        return int(float(text))
    except (TypeError, ValueError):
        return None


def _to_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def true_wheel_speed(vx_m_s, wz_rad_s):
    """四驱反解：左右轮组速度，单位 m/s。

    依据 ``balance_task.c:824-827``：
    ``left = Vx - Vz*(WheelSpacing+AxleSpacing)``，
    ``right = Vx + Vz*(WheelSpacing+AxleSpacing)``。
    """
    lever = wz_rad_s * TURN_LEVER_M
    return vx_m_s - lever, vx_m_s + lever


def wheel_rpm(speed_m_s):
    """轮速 m/s -> rpm。"""
    if WHEEL_CIRCUMFERENCE_M <= 0:
        return 0.0
    return speed_m_s / WHEEL_CIRCUMFERENCE_M * 60.0


def wheel_view(vx_m_s, wz_rad_s):
    """左右轮组的显示值。标记为推算，不冒充实测。"""
    left, right = true_wheel_speed(vx_m_s, wz_rad_s)
    return {
        'left_m_s': round(left, 4),
        'right_m_s': round(right, 4),
        'left_rpm': round(wheel_rpm(left), 1),
        'right_rpm': round(wheel_rpm(right), 1),
        'per_wheel': False,
        'estimated': True,
        'turn_lever_m': round(TURN_LEVER_M, 3),
        'wheel_diameter_m': WHEEL_DIAMETER_M,
        'half_wheel_spacing_m': HALF_WHEEL_SPACING_M,
        'half_axle_spacing_m': HALF_AXLE_SPACING_M,
        'note': GEOMETRY_NOTE,
    }


def chassis_view(chassis, chassis_t, now, timeout=2.0):
    """汇总底盘卡片。超时后返回 stale，绝不继续显示旧值当实时。"""
    state = S.freshness(chassis_t, now, timeout, chassis_t is not None)
    view = {
        'state': state,
        'age': None if chassis_t is None else round(now - chassis_t, 2),
        'voltage_v': None,
        'vx_m_s': None,
        'vy_m_s': None,
        'wz_rad_s': None,
        'stop_flag': None,
        'stop_text': '未知',
        'drive_ready': None,
        'mileage_m': None,
        'ultrasonic_m': None,
        'wheels': None,
    }
    if chassis is None or state != S.FRESH:
        return view

    voltage_mv = chassis.get('voltage_mv')
    if voltage_mv is not None:
        view['voltage_v'] = round(voltage_mv / 1000.0, 2)

    vx_mm = chassis.get('vx_mm_s')
    vy_mm = chassis.get('vy_mm_s')
    wz_mrad = chassis.get('yaw_rate_mrad_s')
    if vx_mm is not None:
        view['vx_m_s'] = round(vx_mm / 1000.0, 4)
    if vy_mm is not None:
        view['vy_m_s'] = round(vy_mm / 1000.0, 4)
    if wz_mrad is not None:
        view['wz_rad_s'] = round(wz_mrad / 1000.0, 4)

    stop = chassis.get('stop_flag')
    view['stop_flag'] = stop
    if stop == 0:
        view['stop_text'] = '软件未失能；就绪未知'
    elif stop is None:
        view['stop_text'] = '未知'
    else:
        view['stop_text'] = '软件失能'
        view['drive_ready'] = False

    mileage_m = chassis.get('mileage_m')
    if mileage_m is not None:
        view['mileage_m'] = round(float(mileage_m), 2)

    # 前置 HC-SR04 距离 mm→m；0xFFFF = 无回波/数据陈旧（固件约定），不显示旧值
    us_mm = chassis.get('ultrasonic_front_mm')
    if us_mm is not None and us_mm != 0xFFFF:
        view['ultrasonic_m'] = round(us_mm / 1000.0, 3)

    if view['vx_m_s'] is not None and view['wz_rad_s'] is not None:
        view['wheels'] = wheel_view(view['vx_m_s'], view['wz_rad_s'])
    return view


def imu_view(last_t, hz, now, timeout=1.0):
    """IMU 心跳卡片。频率来自订阅回调的实测计数。"""
    state = S.freshness(last_t, now, timeout, last_t is not None)
    return {
        'state': state,
        'age': None if last_t is None else round(now - last_t, 2),
        'hz': round(hz, 1) if hz is not None else None,
    }


def cmd_view(twist, t, now, timeout=1.0):
    """速度命令卡片：显示当前下发的 vx/wz 与新鲜度。"""
    state = S.freshness(t, now, timeout, t is not None)
    view = {
        'state': state,
        'age': None if t is None else round(now - t, 2),
        'vx_m_s': None,
        'wz_rad_s': None,
    }
    if twist is None or state != S.FRESH:
        return view
    # 命令既可以是 ROS 消息，也可以是 web_debug 复制出的字典。
    view['vx_m_s'] = round(_vec_component(twist, 'linear', 'x'), 4)
    view['wz_rad_s'] = round(_vec_component(twist, 'angular', 'z'), 4)
    return view


def _vec_component(twist, group, axis):
    """取 linear.x 或 angular.z，字典与消息两种形态都支持。"""
    if isinstance(twist, dict):
        if axis == 'x':
            return float(twist.get('vx', 0.0))
        return float(twist.get('wz', 0.0))
    vec = getattr(twist, group, None)
    return float(getattr(vec, axis, 0.0)) if vec is not None else 0.0


def safety_view(params, source):
    """安全开关卡片。params 是名字到值的映射。"""
    def as_bool(name):
        value = params.get(name)
        if value is None:
            return None
        if isinstance(value, bool):
            return value
        text = str(value).strip().lower()
        if text in ('true', '1', 'yes', 'on'):
            return True
        if text in ('false', '0', 'no', 'off'):
            return False
        return None

    protocol = params.get('drive_firmware_protocol')
    return {
        'drive_enable': as_bool('drive_enable'),
        'drive_input_enable': as_bool('drive_input_enable'),
        'drive_firmware_protocol': (None if protocol is None
                                    else str(protocol)),
        'source': source,
    }
