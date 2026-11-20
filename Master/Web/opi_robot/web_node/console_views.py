"""控制台页面用的纯逻辑：开机静止倒计时、导航/巡逻状态、数值格式化。

全部不依赖 rclpy，可在没有 ROS 的机器上单测。

开机静止倒计时
--------------
``c50c-chassis.service`` 启动时先 ``sleep 20`` 再拉起底盘桥；系统里用
一份标记文件记录这段时间的起点，避免只靠前端本地计时（刷新页面、换
设备就对不上了）。标记文件存在说明正在初始化；超时或服务已进入运行
状态就不再显示倒计时。

时间基准：倒计时本身用 **单调时钟** 推进，墙钟被 NTP 校正时不会突跳；
marker 里存的是墙上时钟（写 marker 的启动脚本与读它的网页是两个进程，
单调时钟零点不同），读取时换算一次偏移即可。
"""
from __future__ import annotations

import os
import time

# 与 c50c-chassis.service 的 ExecStartPre sleep 20 对应（见 start_chassis.sh）。
BOOT_HOLD_S = 20.0
STATE_IDLE = 'idle'
STATE_NAV = 'nav'
STATE_PATROL = 'patrol'
STATE_RETURN = 'return'
STATE_MANUAL = 'manual'

_TEXT = {
    STATE_IDLE: '无任务',
    STATE_NAV: '导航前往任务目标中',
    STATE_PATROL: '巡逻中',
    STATE_RETURN: '低电量回充中',
    STATE_MANUAL: '手动控制中',
}


def _wall_clock_now():
    return time.time()


def read_boot_marker(marker_path, now_mono, hold_s=BOOT_HOLD_S,
                     wall_now=None):
    """返回 marker 里的开机时刻，换算到调用方的单调时钟。

    marker 写的是**墙上时钟**（time.time()），因为写它的脚本和读它的网页
    是两个进程，单调时钟的零点各不相同，跨进程不可比。monotonic 零点与
    墙上时钟的偏移在本进程内取一次，之后就不受系统改时间影响。
    """
    if not marker_path:
        return None
    try:
        with open(marker_path) as fh:
            boot_epoch = float(fh.read().strip())
    except (OSError, ValueError):
        return None
    if wall_now is None:
        wall_now = _wall_clock_now()
    return now_mono - (wall_now - boot_epoch)


def boot_view(marker_path, now_mono, hold_s=BOOT_HOLD_S, wall_now=None):
    """返回开机初始化视图。

    marker 文件内容是一个墙上时钟时间戳（秒，浮点）。文件不存在、内容
    读不出或已经超时，都按"初始化已结束"返回 ``initializing=False``。
    这样删掉标记文件（服务启动完成时由脚本删除）也能立即结束倒计时。
    """
    view = {
        'initializing': False,
        'remain_s': 0.0,
        'hold_s': float(hold_s),
    }
    if not marker_path:
        return view
    start = read_boot_marker(marker_path, now_mono, hold_s, wall_now)
    if start is None:
        return view
    elapsed = now_mono - start
    if elapsed < 0:
        # 时钟对不上（比如文件是上次开机留下的）：当成已结束，不显示负数
        return view
    remain = float(hold_s) - elapsed
    if remain <= 0:
        return view
    view['initializing'] = True
    view['remain_s'] = round(remain, 1)
    return view


def write_boot_marker(path, now_mono):
    """原子地写下倒计时起点。写失败直接抛，由调用方决定是否致命。"""
    tmp = path + '.tmp'
    with open(tmp, 'w') as fh:
        fh.write('%.3f\n' % now_mono)
    os.replace(tmp, path)


def task_state(nav_fresh, nav_vx, nav_wz, nav_age, manual_active=False,
               battery_low=False, patrol_fresh=False):
    """判断当前任务状态，返回 ``(state, text)``。

    - nav_fresh 且命令不是零速 → 导航/手动控制中；零速命令不算"在跑"。
    - patrol_fresh → 巡逻中（当前没有巡逻节点，预留给后续接入）。
    - battery_low → 低电量回充中。
    其余一律"无任务"，不猜。
    """
    if battery_low:
        return STATE_RETURN, _TEXT[STATE_RETURN]
    if patrol_fresh:
        return STATE_PATROL, _TEXT[STATE_PATROL]
    if nav_fresh and nav_age is not None and nav_age <= 1.0:
        moving = (abs(nav_vx or 0.0) > 1e-6) or (abs(nav_wz or 0.0) > 1e-6)
        if moving and manual_active:
            return STATE_MANUAL, _TEXT[STATE_MANUAL]
        if moving:
            return STATE_NAV, _TEXT[STATE_NAV]
    return STATE_IDLE, _TEXT[STATE_IDLE]


# 电池规格（用户 2026-11-20 现场重新标定）：6S 锂离子，正常满电 24.5V，
# 超过 25.0V 判定为充电中，停用线 21.0V（3.5V/串）——低于停用线继续放电会
# 永久损伤电芯。百分比 = 电压在 [停用线, 24.5V 满电] 的线性映射（超 24.5V 钳位 100%）：
# 锂电开路曲线非线性、带载有瞬时压降，此值为仪表估计，不是库仑计量。
_BAT_CELLS = 6
_BAT_FULL_V = 24.5
_BAT_CHARGING_V = 25.0
_BAT_EMPTY_V = 21.0
_BAT_WARN_V = 22.2   # 低于标称电压即提示充电


def battery_view(voltage_v, level=None):
    """显示固件上报电压，按现场标定（24.5V 满电 / >25.0V 充电中）推算电量与状态。"""
    view = {
        'voltage_v': None,
        'percent': None,
        'charging': False,
        'state': 'unknown',
        'text': '无数据',
    }
    if voltage_v is None:
        return view
    try:
        voltage = float(voltage_v)
    except (TypeError, ValueError):
        return view
    view['voltage_v'] = round(voltage, 2)
    percent = (voltage - _BAT_EMPTY_V) / (_BAT_FULL_V - _BAT_EMPTY_V) * 100.0
    view['percent'] = int(round(max(0.0, min(100.0, percent))))
    view['charging'] = voltage > _BAT_CHARGING_V
    if voltage > _BAT_CHARGING_V:
        view['state'] = 'ok'
        view['text'] = '充电中'
    elif voltage <= _BAT_EMPTY_V:
        view['state'] = 'bad'
        view['text'] = '低于停用线，立即停车充电'
    elif voltage < _BAT_WARN_V:
        view['state'] = 'warn'
        view['text'] = '电量低，请充电'
    else:
        view['state'] = 'ok'
        view['text'] = '正常'
    return view

