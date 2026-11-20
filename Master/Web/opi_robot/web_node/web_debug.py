#!/usr/bin/env python3
"""小车控制台：运维概览 / 手动控制 / 日志 三页。

浏览器通过 WebSocket 收数据，登录后可以发前进/后退/转向与紧急停车。

页面用 hash 路由切换（``#/overview``、``#/control``、``#/logs``），
页面本体在 ``console_page.html``（源码目录加载，缺失时用包内兜底页）。
概览只回答"现在还正常吗"：底盘车速与里程、电池、主机环形仪表、节点
呼吸灯、任务状态、网络；传感器明细不单独分页，统一进日志。实时画面
与点云只在手动控制页推送，离开即停。

数据与能力（不删既有能力）：

- 底盘状态：从 ``/diagnostics`` 读取固件上报的电压和车速；车速待标定，
  如有里程数据则为软件积分（页面标注）。
- 电池：显示固件上报电压；电池规格未确认前不推算电量百分比。
- 在线心跳：节点在线用呼吸灯表达；IMU、状态帧、雷达、图像按新鲜度判定。
- 安全开关：drive_enable / drive_input_enable / drive_firmware_protocol
  的当前生效值，优先从在线节点读参数服务，读不到时退回本地启动参数。
- 速度命令：nav_cmd_vel / demo_cmd_vel 的当前下发值，用于判断有无任务。
- 开机静止：底盘服务启动后 20 秒内显示倒计时（标记文件见 console_views）。
- 假数据模式：设置环境变量 ``OPI_WEB_FAKE=1`` 或传 ``--fake``，
  不订阅任何 ROS 话题即可展示完整页面。

数据来源（只读引用）：

- 状态帧解析：``c50c_uart3/c50c_uart3/frame_parser.py``。
- IMU 话题：``c50c_uart3/c50c_uart3/uart3_node.py``（``imu/data_raw``）。
- 速度命令话题：``c50c_uart3/c50c_uart3/chassis_cmd/ros_input.py``。
- 四驱几何：``project/C50C_gcc/08_CarType/Inc/robot_init.h``。

已知缺口（页面上如实显示，不假装有数据）：

- 「最后写入结果」没有发布话题，控制页不能确认车辆是否执行命令。
- 雷达诊断里的重连次数、好帧、坏帧取决于是否有诊断发布者。
"""

from __future__ import annotations

import base64
import hashlib
import json
import math
import os
import select
import shutil
import socket
import subprocess
import sys
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:                                    # 假数据模式不需要 rclpy
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, Imu, LaserScan
    from geometry_msgs.msg import Twist
    from diagnostic_msgs.msg import DiagnosticArray
    # 移动角区话题是可选增强：缺 std_msgs 时点云照常，只是没有高亮
    try:
        from std_msgs.msg import UInt8MultiArray as _U8Arr
    except ImportError:
        _U8Arr = None
    # 巡航开关用 std_msgs/Bool；缺它时巡航按钮显示不可用，其余功能照常
    try:
        from std_msgs.msg import Bool as _Bool
    except ImportError:
        _Bool = None
    # [巡航] String（许可转发）在文件后段统一导入到 _String，这里不重复
    # [C50C 2026-11-15] 地图页：SLAM 占据栅格（nav_msgs/OccupancyGrid）与
    # map→base_link 位姿（tf2_ros）。缺任一时地图页只显示点云与超声波。
    try:
        from nav_msgs.msg import OccupancyGrid as _OccGrid
    except ImportError:
        _OccGrid = None
    try:
        from tf2_ros import Buffer as _TfBuffer
        from tf2_ros import TransformListener as _TfListener
    except Exception:                    # tf2_ros 可能缺，或被测试垫片遮蔽
        _TfBuffer = _TfListener = None
    _HAVE_ROS = True
except ImportError:                     # pragma: no cover - 仅在假模式触发
    rclpy = None
    _U8Arr = None
    _Bool = None
    _OccGrid = None
    _TfBuffer = _TfListener = None

    class Node:                         # type: ignore[no-redef]
        pass

    class _Stub:
        """缺失 ROS 时的话题占位：订阅照建，回调永不触发。"""

        def __init__(self, *args, **kwargs):
            pass

    CompressedImage = Imu = LaserScan = Twist = DiagnosticArray = _Stub
    UInt8MultiArray = _Stub
    _HAVE_ROS = False

try:
    from rcl_interfaces.srv import GetParameters, ListParameters
    _HAVE_PARAM_SRV = True
except ImportError:                     # pragma: no cover
    GetParameters = ListParameters = None
    _HAVE_PARAM_SRV = False

from .. import qos as Q
from . import chassis as C
from . import sensors as S
from . import fake_source as F
from .manual_forward import ManualForwardControl
from . import nav_goal as NG
from . import patrol as P
from . import console_views as V
from opi_robot.chassis_status import MileageIntegrator

# 模块加载即建一次巡航状态机：状态帧/鉴权在 Bridge 启动前也要正常返回
# （Bridge 启动时再按地图参数重建）。放在所有 import 之后。

try:
    from std_msgs.msg import String as _String
except ImportError:                     # pragma: no cover
    class _String:                      # 假模式：仅占位
        def __init__(self, **kwargs):
            self.data = kwargs.get('data', '')

# [巡航] 下游运动门输入。std_msgs/Bool 缺失（假模式）时发布器根本建不出来，
# 下游通道保持 None（退化 no-op），不需要再补桩。

_MANUAL = ManualForwardControl()

# 自动驾驶巡航开关。网页只发开关，速度由 fusion_node 按 0.4 m/s 下发。
# 话题名必须与 fusion_node 的订阅一致：/cruise_enable（std_msgs/Bool）。
CRUISE_TOPIC = 'cruise_enable'
# 巡航开关状态。on 是最近一次成功下发的值；available 表示发布器就绪。
_cruise = {'on': False, 'available': False, 'publisher': None}


def cruise_view():
    """巡航开关视图，进状态帧。发布器缺失时 available 为假。"""
    with _lock:
        return {'on': bool(_cruise['on']),
                'available': bool(_cruise['available'])}


def set_cruise(on):
    """保存巡航开关并下发 /cruise_enable。

    发布器缺失时抛 RuntimeError：调用方回 manual_error，页面显示不可用。
    返回实际生效的值。
    """
    with _lock:
        publisher = _cruise['publisher']
    if publisher is None:
        raise RuntimeError('巡航通道不可用，检查 std_msgs 依赖')
    value = bool(on)
    publisher.publish(_Bool(data=value))
    with _lock:
        _cruise['on'] = value
    return value

# ---- 航点巡航与网页运动许可（计划 §3/§4）----------------------------
# 路线绑定地图内容标识：地图 yaml/pgm 的 sha256 前 16 位。yaml 由
# 启动参数 patrol_map_yaml 指定；不指定或文件缺失时 map_id 为 None，
# 保存/开始巡航一律拒绝（宁可不巡航，不碰"不知道是哪张图"的路线）。
PATROL_STATUS_TOPIC = 'patrol_status'
PATROL_GOAL_TIMEOUT_S = 15.0
# 与 c50c_uart3/chassis_cmd/guard_wiring.py 的 LICENSE_TOPIC / REARM_TOPIC /
# AUTHORIZE_TOPIC 同名（不带前导斜杠，按本节点命名空间解析）。
LICENSE_TOPIC = 'motion_license'
REARM_TOPIC = 'rearm_request'
AUTHORIZE_TOPIC = 'guard_authorize'
PATROL_PERIOD = 0.1


def _patrol_now():
    return time.monotonic()



def patrol_map_id(path):
    """地图文件 → 16 位内容标识；文件缺失返回 None。"""
    if not path:
        return None
    try:
        files = P.map_files_from_yaml(path)
        return P.map_id_from_files(files)
    except (OSError, ValueError):
        return None


_patrol = {
    'runner': None,
    'license': None,
    'route': None,
    'map_yaml': None,
    'map_id': None,
    'goal_queue': None,          # dispatch 投递队列（executor 线程发目标）
    'goal_seq': 0,
    'publisher': None,
    'last_frame_t': 0.0,
    'live_t': None,              # 上一次确认"网页会话还在"的时刻
    'session_lost': '',          # 最近一次判会话失效的原因
    # runner 的锁停通道：Bridge 启动时换成发下游的版本（见
    # patrol_downstream_attach），之前是 no-op。
    'lock_sink': lambda reason: None,
}

# 会话存活判定的节拍（秒）。与 select 轮询周期同量级，不额外占 CPU。
SESSION_POLL_S = 0.02
# 测试可替换：等待节拍用的 sleep（默认 time.sleep）。
_sleep = time.sleep


def _patrol_wire_nav():
    """把现行 NavGoalControl 的接受/结果回调接到巡航状态机。

    每次取目标前都重接一次：nav_goal 的模块级实例被替换（测试）或
    Bridge 晚于其它组件启动时，回调都不会丢。
    """
    try:
        NG.set_callbacks(_patrol_accepted, _patrol_result, control=NG._NAV)
    except Exception:
        pass
    # runner 的锁停也要落到当前下游通道（configure_patrol 之后 Bridge
    # 才建发布器的情形）。
    runner = _patrol['runner']
    if runner is not None:
        runner._lock_stop = _patrol['lock_sink']


def _patrol_dispatch(goal):
    """巡航状态机 → 发目标队列。返回请求编号。"""
    q = _patrol['goal_queue']
    if q is None:
        raise RuntimeError('patrol_dispatch_unavailable')
    _patrol['goal_seq'] += 1
    rid = _patrol['goal_seq']
    q.put((rid, dict(goal)))
    return rid


def _patrol_cancel(rid, result_known=True):
    return NG.cancel_nav_seq(rid, result_known=result_known)


def _patrol_run_dispatch():
    """executor 定时器里调：把队首目标交给 NavGoalControl。

    发目标放在 executor 线程，ws 线程只投递：与 nav_goal 的线程模型一致。
    """
    q = _patrol['goal_queue']
    if q is None or q.empty():
        return
    try:
        rid, goal = q.get_nowait()
    except Exception:
        return
    _patrol_wire_nav()
    ok, view = NG.request_nav_goal(goal['x'], goal['y'], goal['yaw'])
    if not ok:
        runner = _patrol['runner']
        if runner is not None:
            _patrol_result(rid, 'failed', view.get('error') or 'request_failed')


def _patrol_accepted(rid):
    try:
        _patrol['runner'].on_accepted(rid)
    except Exception:
        _patrol_log_exc('patrol_accepted')


def _patrol_result(rid, status, reason=''):
    try:
        _patrol_wire_nav()       # 结果也走生产接线（与发目标同一入口）
        _patrol['runner'].on_result(rid, status, reason or '')
    except Exception:
        _patrol_log_exc('patrol_result')


def _patrol_log_exc(where):
    try:
        import traceback
        sys.stderr.write('%s: exception in %s\n%s' % (
            time.time(), where, traceback.format_exc()))
    except Exception:
        pass


def _patrol_plan_blocked(nav=None):
    """目标是否连续没有安全路径/合法控制轨迹（"此刻仍受阻"的当前值）。

    判据（页面不做此判断）：

    1. Nav2 明确给了规划/控制失败（nav_goal 把 action result 的
       error_code 转成 ``plan_failed`` / ``control_failed``，或 abort 的
       reason 说明没有可用路径/轨迹）；
    2. feedback 推断：目标已被接受且正在跟踪，但 feedback 断开超过
       ``PLAN_BLOCK_S``。

    刚被接受、还没收到第一帧反馈时不判受阻——那一刻只是"还没开始规划"，
    ``PatrolRunner.on_plan_blocked`` 的计时也以接受时刻重新起算。
    """
    nav = NG.nav_view() if nav is None else nav
    if nav.get('state') != 'active':
        return False
    # Nav2 明确给的原因码 / 失败文本（见 nav_goal 的 plan_error/error）
    cause = '%s %s' % (nav.get('plan_error') or '', nav.get('error') or '')
    cause = cause.lower()
    if any(k in cause for k in _PLAN_FAIL_KEYS):
        return True
    fb = nav.get('feedback')
    age = nav.get('feedback_age')
    if fb is None and age is None:
        return False               # 还没有任何反馈：还没开始，不算受阻
    if fb is not None and (age is not None and age <= P.PLAN_BLOCK_S):
        return False
    return True


# Nav2 error_code 的文本化（nav_goal 写成 'error_code=N(NAME)'）+ abort 文案。
# 只认"规划/控制确实失败"，不把普通的导航失败（例如被拒、超时）算进来。
_PLAN_FAIL_KEYS = ('no_valid_path', 'no valid path', 'no_path', 'no path',
                   'plan_failed', 'planner failed', 'no_global_plan',
                   'planner_timed_out', 'controller_failed',
                   'no valid control', '规划/控制失败')


def patrol_tick(now=None):
    """executor 定时器（10 Hz）：超时/许可/规划判停/状态发布。"""
    runner = _patrol['runner']
    lic = _patrol['license']
    if runner is None or lic is None:
        return
    try:
        _patrol_run_dispatch()
        lic.expired()          # 350 ms 无许可：判失效并上报 license_lost
        # 地图文件被替换后，下一拍就按新内容判（start 用这里的值比路线）
        _mid = patrol_map_id(_patrol['map_yaml'])
        if _mid is not None:
            _patrol['map_id'] = _mid
        # 网页会话看护：许可超时、或在途巡航时断线（最后一个实时页连接
        # 走掉）→ 取消并锁停。许可仍在而页面已经不在了（只断 ws、进程
        # 还活着）时许可会一直续期，所以必须按会话判，不能只等许可超时。
        # 顺序：超时 → 会话看护 → 规划判停。已经因超时失败的目标不再被
        # 会话看护改写成"断线"，锁停原因保持首个真实原因。
        runner.overall_timeout(PATROL_GOAL_TIMEOUT_S)
        snap = lic.snapshot()
        _patrol_session_guard(snap)
        runner.on_plan_blocked(_patrol_plan_blocked())
        now = _patrol_now() if now is None else now
        if now - _patrol['last_frame_t'] >= PATROL_PERIOD:
            _patrol['last_frame_t'] = now
            out = patrol_status_frame()
            pub = _patrol['publisher']
            text = json.dumps(out, allow_nan=False)
            if pub is not None:
                try:
                    pub.publish(_String(data=text))
                except Exception:
                    pass
            # 活跃的驾驶会话同步收到一帧（页面侧不用等 5 Hz 状态帧）
            for h in list(_live_controls):
                try:
                    _send_text(h, text)
                except Exception:
                    pass
    except Exception:
        _patrol_log_exc('patrol_tick')


def _patrol_session_guard(license_snapshot):
    """判定网页会话是否失效（每个看护节拍调用一次）。

    - 许可超时：``MotionLicense.expired`` 已经把原因写成 license_lost。
    - 在途巡航 + 许可新鲜（或本次会话从没拿到过许可）+ 超过存活窗口没有
      任何确认：判会话失效（web_disconnected），取消并锁停。
    - 实时页心跳仍在：续期，会话照常在（许可状态由 ``licensed`` 单独表达）。
    """
    runner = _patrol['runner']
    if runner is None:
        return
    active = (runner.state in ('running', 'holding_goal')
              or runner.cancel_request is not None)
    if not active:
        return                             # 没在跑：锁停态本身就是停
    if not license_snapshot.get('fresh'):
        # 许可超时 / 会话被顶替 / 重放：会话已不可续期，立即停（不等窗口）。
        # 原因是许可（license_lost）；只有页面连接集合才知道"页面走了"。
        # 已经停住（例如上一拍因断线停的）就不再改写锁停原因：会话失效
        # 只是一次，原因取第一次那个。
        if runner.state in ('running', 'holding_goal'):
            _patrol_session_lost('license_lost')
        return
    window = SESSION_WINDOW.window_s
    last_t = _patrol['live_t']
    if last_t is not None:
        # 有实时页心跳记录：心跳停了（页关掉/挂起/断网）即判会话失效。
        if _patrol_now() - last_t > window:
            patrol_session_closed(None)
        return
    # 从没收到过实时页心跳（页面只发许可，例如旧页面或测试驱动）：用
    # 最近一条有效许可的到达时刻当续期点——许可按 100 ms 节奏续期就说明
    # 会话还活着；许可窗口也断了同样判会话失效。
    age = license_snapshot.get('age_s')
    if age is not None and age > window:
        patrol_session_closed(None)


def patrol_status_frame(now=None):
    """``/patrol_status`` 帧。许可不新鲜时 released 必为 False。"""
    runner = _patrol['runner']
    lic = _patrol['license']
    if runner is None or lic is None:
        return {'t': 'patrol_status', 'licensed': False, 'released': False,
                'locked': True, 'lock_reason': 'global_stop',
                'stop_cause': 'global_stop', 'status': 'idle',
                'rearm_state': P.rearm_state(True, False, False, None),
                'map_id': None, 'current_map_id': _patrol['map_id'],
                'route_valid': False, 'session': patrol_session_hooks(),
                'license_state': 'absent', 'gate': 'lock_stop'}
    license_state = lic.snapshot()
    frame = P.status_frame(runner, license_state, _patrol['map_id'])
    try:
        frame['nav'] = NG.nav_view()
    except Exception:
        frame['nav'] = None
    frame['map_yaml'] = _patrol['map_yaml']
    # 航线（供页面重绘）
    route = runner.route
    frame['route'] = (None if route is None else
                      [[round(p[0], 3), round(p[1], 3)] for p in route['points']])
    frame['session'] = patrol_session_hooks()
    # 许可状态细分（运动门结论直投）：lock_stop=锁停；license_lost=许可不可
    # 续期（会话失效/顶替）；unlicensed=运行中但没有有效许可；ok。
    if frame['locked']:
        frame['license_state'] = 'lock_stop'
        frame['gate'] = 'lock_stop'
    elif not frame['licensed']:
        frame['license_state'] = 'license_lost'
        frame['gate'] = 'license_lost'
    else:
        frame['license_state'] = 'ok'
        # 需求侧允许：许可有效 + 锁停已解除 + 没有待确认取消。执行侧（桥）
        # 的传感器门与仲裁不在这里重复判断。
        frame['gate'] = ('ok' if frame['released']
                         and frame['pending_cancel'] is None
                         else 'not_released')
    return frame


def configure_patrol(reset_runner=False, map_yaml=None, goal_queue=None,
                     now=None):
    """建立/重建巡航状态机与许可监视（Bridge 启动、测试复位时调）。"""
    if map_yaml is not None:
        _patrol['map_yaml'] = map_yaml
    if goal_queue is not None:
        _patrol['goal_queue'] = goal_queue
        _patrol['goal_seq'] = 0
    elif reset_runner or _patrol['goal_queue'] is None:
        import queue as _queue
        _patrol['goal_queue'] = _queue.Queue()
        _patrol['goal_seq'] = 0
    if reset_runner or _patrol['runner'] is None:
        # 锁停请求：本进程的 runner 状态；真正的零速由底盘桥按许可超时
        # 独立完成（见 patrol.MotionLicense 文档），不依赖网页进程。
        # 锁停请求必须真的往下游发（撤回 /guard_authorize），而不是写进
        # 一个空 lambda：那会让"内层锁停、下游照走"成为可能。锁停通道
        # 在 Bridge 启动时接上；之前的调用落在 no-op 里（离线环境）。
        _patrol['runner'] = P.PatrolRunner(
            _patrol_dispatch, _patrol_cancel,
            _patrol['lock_sink'], now or _patrol_now)
        _patrol['route'] = None
        _patrol['live_t'] = None
        _patrol['session_lost'] = ''
        # 新 runner 的许可监视也换新（旧监视的 on_lost 指向旧 runner）
        _patrol['license'] = None
    if reset_runner or _patrol['license'] is None:
        _patrol['license'] = P.MotionLicense(
            lambda reason: _patrol['runner'].lock_stop(reason),
            now=now or _patrol_now)
    # 复位/测试时旧连接不再算"实时页会话"；运行中改地图不动会话。
    with _live_sessions_lock:
        if reset_runner:
            _live_sessions.clear()
    _patrol['map_id'] = patrol_map_id(_patrol['map_yaml'])
    return _patrol['runner']


def patrol_save(points, map_id=None):
    """保存路线（绑定当前地图标识）。返回 (ok, message|route, frame)。"""
    runner = _patrol['runner']
    cur = _patrol['map_id']
    if cur is None:
        return False, '当前地图未知（未指定地图文件），不能保存路线', patrol_status_frame()
    old = runner.route
    if old is not None and old.get('map_id') not in (None, cur):
        # 换图后旧路线仍挂在 runner 上：不先清掉就"重新保存"，等于拿
        # 换图前的路线名头给新地图背书。要求调用方显式传空地图标识
        # （= 放弃旧路线，按当前地图新建），否则拒绝。
        if map_id:
            return False, '地图已更换：旧路线作废，请先清空路线再保存', patrol_status_frame()
        runner.save(None)
        _patrol['route'] = None
        old = None
    route, err = P.validate_route(points, map_id or cur, cur)
    if err is not None:
        return False, err, patrol_status_frame()
    runner.save(route)
    _patrol['route'] = route
    return True, '已保存 %d 个航点' % len(route['points']), patrol_status_frame()


def patrol_start():
    runner = _patrol['runner']
    if runner is None:
        return False, '巡航状态机未就绪'
    # 地图内容可能在上次保存后变过：按当前文件重算标识再比，不能只比
    # 启动时缓存的那一个（生产上 configure_patrol 只在启动时调一次）。
    # 重算结果只用于与路线比对；真实地图文件每次要重读+哈希，常态刷新
    # 交给 patrol_tick，切换地图文件后下一次点击就按新内容判。
    # 没有地图文件（None/文件缺失）时保留当前标识：那是"未知地图"，不是
    # "新地图"，拿 None 去比会把刚保存的路线误判成换图。
    _mid = patrol_map_id(_patrol['map_yaml'])
    if _mid is not None:
        _patrol['map_id'] = _mid
    if runner.route is None:
        return False, '没有已保存的路线'
    ok, info = runner.start(_patrol['map_id'])
    if ok:
        # 开始巡航的这条消息本身证明网页会话在；之后靠许可/心跳续期。
        _patrol_live_touch()
    return ok, info


def patrol_stop(reason='global_stop'):
    return _patrol['runner'].stop(reason, user_action=True)


def patrol_rearm():
    runner = _patrol['runner']
    lic = _patrol['license']
    ok, info = runner.rearm(license_ok=bool(lic and lic.fresh()))
    if ok:
        # 解除锁停动作本身即确认网页会话在（按键的一定是活着的页面）。
        _patrol_live_touch()
        # 本进程锁停已解除、用户显式恢复：再向下游请求一次解停。
        # 下游只认"已鉴权 + 显式请求"（guard_bridge.preconditions_ok），
        # 请求一次即消费，下一轮必须重新点。
        # 顺序必须先 authorize 再 rearm：锁停/会话失效会把下游鉴权撤回成
        # False（patrol_downstream_lock_stop / _patrol_session_lost），
        # 恢复时若只发 /rearm_request 不补鉴权，下游 preconditions_ok
        # 永远 False → rearm 被拒，guard 区车辆永远停在 rearm_pending。
        patrol_downstream_authorize(True, force=True)
        patrol_downstream_rearm()
    return ok, info


def ultrasonic_rearm():
    """网页"重新启动超声波"：清 fusion_node 超声门的无效锁。

    超声门 fail-closed（无回波/断流 → 许可拒绝），读数本身永远不能自动
    清除无效锁，必须由人明确动作恢复。发布到 ``ultrasonic_rearm`` 话题
    （``std_msgs/Bool``，status QoS），fusion_node 收到即 ``us_gate.rearm()``。
    发布器不可用时如实返回失败，不假装已恢复。
    """
    pub = _US_REARM_PUB
    if pub is None:
        return False, '超声恢复通道不可用'
    try:
        pub.publish(_Bool(data=True))
    except Exception as exc:                           # noqa: BLE001
        return False, '超声恢复发布失败: %r' % (exc,)
    _ws_log_add('tx', {'t': 'ultrasonic_rearm'})
    return True, '已请求重新启动超声波'


def patrol_manual_override(owner):
    """手动接管：仅当本连接真的在按手动键（或即将按下）时取消旧导航。"""
    runner = _patrol['runner']
    try:
        holds = _MANUAL.holds(owner)
    except Exception:
        holds = False
    if handles_nav(runner) and holds:
        runner.manual_override()
    return holds


def handles_nav(runner):
    return runner is not None and runner.state in ('running', 'holding_goal')


def _manual_takeover_hook(owner):
    """ManualForwardControl 在发布第一条非零速度前调用。

    在这里取消旧导航（而不是在消息处理里、在 ``_MANUAL.start`` 之前）：
    那时 owner 还没拿到手动控制，``holds(owner)`` 必为 False，接管永远
    不执行；等 start 把 owner 设好之后旧目标已经在跑。钩子放在 start
    内部，保证"先取消旧导航，再发手动速度"的顺序。
    """
    runner = _patrol['runner']
    if handles_nav(runner):
        try:
            runner.manual_override()
        except Exception:          # noqa: BLE001——接管失败也不能卡住手动输入
            _patrol_log_exc('patrol_manual_override')


def patrol_session_lost(reason='license_lost'):
    """网页会话失联（许可超时 / 最后一个实时页连接走掉）：取消并锁停。"""
    return _patrol_session_lost(reason)


def patrol_license_message(msg):
    """``{t:'motion_license', session, seq, expires_mono [, claim]}``。

    - ``session=0``：页面还没从服务端拿到时钟基准。回一条 ``motion_clock``
      （页面据本帧到达时刻换算）；若同时带 ``claim``（本页想用的会话号），
      按接管规则试一下这个号码，被接受就回 ``motion_license_session``——
      没有这一步，旧页面留下的会话号会让新页面永远被判失效。
    - 其余按运动许可规则校验；被接受即回 ``motion_license_session``（幂等，
      页面据此确认本页号码有效）。
    """
    session = msg.get('session')
    lic = _patrol['license']
    claim = msg.get('claim')
    if session == 0:
        out = []
        if isinstance(claim, int) and not isinstance(claim, bool):
            ok, _reason = lic.accept(claim, msg.get('seq') or 0)
            if ok:
                out.append({'t': 'motion_license_session', 'session': claim})
                _patrol_live_touch()
                patrol_downstream_authorize(True)
        out.append({'t': 'motion_clock', 'seq': msg.get('seq'),
                    'mono': round(_patrol_now(), 6), 'addr': time.time()})
        return out
    ok, reason = lic.accept(
        session, msg.get('seq'), msg.get('expires_mono'))
    if not ok:
        return {'t': 'motion_license_nak', 'reason': reason, 'session': session}
    _patrol_live_touch()
    # 同一套 session/seq/expires_mono 原文转给下游运动门（100 ms 一条）。
    # 下游据此判"网页还在"；本进程退出即断流，下游 0.5 s 后自行锁停。
    patrol_downstream_license(json.dumps(
        {'session': session, 'seq': msg.get('seq'),
         'expires_mono': msg.get('expires_mono')},
        separators=(',', ':')))
    patrol_downstream_authorize(True)
    return {'t': 'motion_license_session', 'session': session}


def _session_msg_text(out):
    """``patrol_license_message`` 的返回值 → 要回给页面的文本列表。"""
    items = out if isinstance(out, list) else ([] if out is None else [out])
    for item in items:
        if item.get('reason') == 'session_mismatch':
            # 告诉被拒页面"服务器现在认哪个号"：号码不是本页的，说明本页
            # 是被顶替的一方 → 页面换新会话号（旧号已结束，接管规则会接收）。
            item['owner'] = _patrol['license'].session
    return [json.dumps(item) for item in items]


class PatrolSessionDeadline:
    """网页会话存活窗口：最后一条有效许可（或一次实时页心跳）之后的 N 秒。

    网页每 100 ms 发一条 /motion_license；(view, page=map/control) 心跳每
    1 s 一条。两者任一到达即续期。窗口取 350 ms 与心跳周期的较大者＋余量：
    许可断了但实时页心跳还在（例如该页 mtime 落后的旧页面不发许可）时，
    不能把正在巡的任何页面判死；反过来页面进程退出时许可与心跳同时停，
    350 ms 后照样判会话失效。真正判失效的门槛仍是 350 ms 的许可。
    """

    def __init__(self, license_timeout_s, beat_s):
        self.license_timeout_s = float(license_timeout_s)
        self.beat_s = float(beat_s)
        # 心跳 1 s：窗口 2.2 s，单次许可丢帧、一次心跳抖动都不误判；页面进程
        # 退出时许可与心跳同时停，350 ms 的许可判定先触发。
        self.window_s = max(self.license_timeout_s, self.beat_s * 2.0 + 0.2)


SESSION_WINDOW = PatrolSessionDeadline(P.SESSION_TIMEOUT_S, 1.0)   # 1.0 = VIEW_BEAT_S


def _patrol_live_touch():
    """确认网页会话还在：在途目标照常推进，清掉"会话失效"记录。

    只更新时间戳，**不**解除锁停——会话恢复不等于用户按了"恢复"。
    """
    _patrol['live_t'] = _patrol_now()
    _patrol['session_lost'] = ''


def patrol_session_live(now=None):
    """实时页心跳（view=map/control）算一次会话存活。"""
    _patrol_live_touch()


def patrol_session_reset():
    """测试/重启用：清空实时页会话集合与失效记录。"""
    with _live_sessions_lock:
        _live_sessions.clear()
    _patrol['live_t'] = None
    _patrol['session_lost'] = ''


def patrol_session_closed(handler):
    """一个控制连接结束（断开/换页/心跳超时）。最后一个实时页连接走掉即判失效。

    ``handler=None``：不看连接集合直接判失效（许可超时/看护线程用）。
    返回 ``True`` 表示本次调用触发了会话失效。
    """
    if handler is not None:
        with _live_sessions_lock:
            _live_sessions.discard(handler)
            others = len(_live_sessions)
        _live_controls.discard(handler)
        if others:
            return False
    else:
        with _live_sessions_lock:
            others = len(_live_sessions)
        if others:
            return False
    return _patrol_session_lost('web_disconnected')


def _patrol_session_lost(reason):
    """网页会话失效：取消并在途导航并锁停（页面重连后须用户显式恢复）。"""
    lic = _patrol['license']
    # 下游先撤回鉴权：即使本进程之后卡死/被打断，下游也会立即锁停。
    patrol_downstream_authorize(False)
    # 本节拍内到达的状态帧不许再自称 licensed/released：会话已经没了；
    # 同时把旧会话标成"已结束"，下一个页面（新会话号）可立即接管许可。
    if lic is not None:
        try:
            # 只上报一次失效（本节拍 status_frame 显示 licensed=False，
            # 并置 _lost 免得下一拍 license_lost 再盖掉本次的真实原因）。
            # 不 mark_stale——mark_stale 是"旧会话松开、新页面可接管"的
            # 语义，会话断开不是接管；标了它，本页拿到新许可也 fresh=False。
            lic.report_lost()
        except Exception:          # noqa: BLE001
            pass
    _patrol['live_t'] = None
    _patrol['session_lost'] = reason
    runner = _patrol['runner']
    if runner is None:
        return True
    if runner.state in ('running', 'holding_goal') or runner.cancel_request:
        runner.stop(reason, user_action=True)   # 在途：取消并锁停
    elif not runner.locked:
        # 没在跑也没锁停（例如空闲中锁停已解除）：补一道锁停。
        runner.lock_stop(reason)
    # 已经锁停（可能是更早、更真实的原因如导航失败/派发失败/用户停车）：
    # 不覆盖原因，只保持锁停——会话失效不需要换掉屏幕上的首个真实原因。
    return True


def patrol_session_hooks():
    """状态帧里的会话字段（测试与页面显示用）。"""
    with _live_sessions_lock:
        n = len(_live_sessions)
    return {'live': n, 'lost': _patrol['session_lost']}


def patrol_session_watch(stop, handler=None):
    """看护一个网页会话的连接。

    线程模型：控制连接在自己的线程里跑读循环；本函数在**另一个**线程里
    每 ``SESSION_POLL_S`` 检查"网页会话是否还在"。判失效后走与
    :func:`patrol_session_closed` 相同的路径（取消并锁停），**不依赖**
    该连接的读循环、也不依赖 web_debug 主进程：底盘桥在自身许可超时
    （0.5 s）后独立零速，网页进程整个退出时由它兜底。
    """
    while True:
        try:
            patrol_tick()          # 先查再等：启动即判一次，不等第一个节拍
        except Exception:          # noqa: BLE001
            _patrol_log_exc('patrol_session_watch')
        if stop():
            break
        _sleep(SESSION_POLL_S)
    if handler is not None:
        patrol_session_closed(handler)


# 手动输入的巡航接管钩子在这里登记：_MANUAL 在文件前段就建好了，
# 也必须保证任何 Bridge/测试路径下都已挂上（不依赖谁先 import）。
_MANUAL.takeover_hook(_manual_takeover_hook)


HTTP_PORT = 8088
# 图像推送上限（帧/秒）。Wi-Fi 实测仅 5.5 Mbit/s，原图 200KB 推不到 30fps。
IMG_HZ = 5.0
# 状态推送周期（秒）。状态与数据分开推，断流时也能立刻显示过期。
# 2026-11-13 用户要求加快：0.5 秒在手动加速时明显看得到速度表滞后。
# 0.2 秒即 5 Hz，与 status_payload 的采样开销相当，Wi-Fi 带宽不是瓶颈。
STATUS_PERIOD = 0.2
SCAN_TIMEOUT = 1.5
# 图像过期判定：5 Hz 推图在弱网下经常晚 1 秒才到，1 秒阈值会来回跳
# "过期/实时"。3 秒内都算实时，只有真断流才标过期。
IMAGE_TIMEOUT = 3.0
DIAG_TIMEOUT = 3.0
CHASSIS_TIMEOUT = 2.0
IMU_TIMEOUT = 2.0
CMD_TIMEOUT = 1.0
#: /drive_safety 的许可有效期由发布端随消息给出（ttl_s）；没收到也不
#: 默认放行——页面只显示"无许可"，不做自己的判定。
SAFETY_STATUS_TOPIC = 'drive_safety'
#: 超声波"重新启动"话题（网页 → fusion_node）；发布端在 WebNode 初始化时建。
ULTRASONIC_REARM_TOPIC = 'ultrasonic_rearm'
#: 最近创建的 WebNode 的超声恢复发布器（None = 不可用）。
_US_REARM_PUB = None
# [巡航] 下游运动门（c50c_uart3 guard_bridge）的输入通道：
#   /motion_license  —— 与页面发给本进程同一套规则转出的许可（下游据此判
#                       网页还活着；网页进程整个退出 → 不再有许可 → 0.5 s
#                       后下游自行锁停并给真实原因，不依赖本进程）；
#   /guard_authorize —— 许可有效即"已鉴权"、失效/会话结束即撤回；撤回时
#                       下游会立即锁停并作废未决的解停请求；
#   /rearm_request   —— 只在用户点"恢复"且本进程锁停已解除时才发一次。
# 发布器在 Bridge 启动时建；离线/测试环境没有 rclpy 时保持 None，
# 全部退化为 no-op（不影响本进程的巡航状态机）。
_patrol_downstream = {
    'license': None,      # publisher[_String]
    'authorize': None,    # publisher[_Bool]
    'rearm': None,        # publisher[_Bool]
    'authorized': False,  # 最近一次发出去的鉴权状态
    'last_forward': None, # 最近一次转发的许可（去重，免得每拍都发）
    'forwards': 0,
    'rearmsg': 0,
    # runner 最近一次锁停经 guard_stop_cause 映射后的下游口径（B5）。
    'last_stop_cause': '',
}


def patrol_downstream_attach(node):
    """在 Bridge 上建下游三个通道并接上 runner 的锁停回调。

    [B1] 许可链是安全相关通道：license publisher 的创建必须独立于
    authorize/rearm。原先三个 publisher 共用一个 try，且每条都先求值
    ``Q.status_qos()``——rclpy 缺失（假模式/降级环境）时 qos 就地抛
    ``ModuleNotFoundError``，整个 try 被吞掉，license publisher 也建不
    出来，``patrol_downstream_license`` 恒 return False，下游运动门永远
    license_expired。现在：qos 构造失败退化为默认 depth（真实 rclpy 与
    fake_ros 的 create_publisher 都接受 int），license 通道单独建。
    """
    _String = globals().get('_String')
    _Bool = globals().get('_Bool')
    _patrol_downstream['license'] = None
    _patrol_downstream['authorize'] = None
    _patrol_downstream['rearm'] = None
    _patrol_downstream['authorized'] = False
    _patrol_downstream['last_forward'] = None

    def _qos(depth=10):
        try:
            return Q.status_qos(depth)
        except Exception:          # noqa: BLE001——rclpy 缺失等坏环境
            return depth           # 默认 QoS（与 fake_ros/真实 rclpy 兼容）

    # [B1] license 通道独立 try：authorize/rearm 建不成不许拖垮许可转发。
    try:
        if _String is not None:
            _patrol_downstream['license'] = node.create_publisher(
                _String, LICENSE_TOPIC, _qos())
    except Exception:              # noqa: BLE001——坏环境
        pass
    try:
        if _Bool is not None:
            _patrol_downstream['authorize'] = node.create_publisher(
                _Bool, AUTHORIZE_TOPIC, _qos())
            _patrol_downstream['rearm'] = node.create_publisher(
                _Bool, REARM_TOPIC, _qos())
    except Exception:              # noqa: BLE001——坏环境
        pass
    runner = _patrol['runner']
    if runner is not None:
        runner._lock_stop = patrol_downstream_lock_stop
        _patrol['lock_sink'] = patrol_downstream_lock_stop
    return _patrol_downstream


def _pub_bool(topic_key, value):
    pub = _patrol_downstream.get(topic_key)
    if pub is None:
        return False
    try:
        pub.publish(globals()['_Bool'](data=bool(value)))
        return True
    except Exception:              # noqa: BLE001
        return False


def patrol_downstream_authorize(ok, *, force=False):
    """同步下游鉴权；显式恢复可强制重发，许可续期仍按值去重。"""
    ok = bool(ok)
    if not force and ok == _patrol_downstream['authorized']:
        return
    if _pub_bool('authorize', ok):
        _patrol_downstream['authorized'] = ok


def patrol_downstream_license(text):
    """把页面许可原文（同一套 session/seq/expires_mono）转发给下游运动门。

    下游的规则比本进程更严（会话不匹配即锁存），所以只转发"本进程已
    接受的那条原文"，不自己另外生成号码。
    """
    pub = _patrol_downstream.get('license')
    if pub is None or not isinstance(text, str):
        return False
    try:
        pub.publish(globals()['_String'](data=text))
    except Exception:              # noqa: BLE001
        return False
    _patrol_downstream['last_forward'] = text
    _patrol_downstream['forwards'] += 1
    return True


def patrol_downstream_rearm():
    """用户点"恢复"：本进程已解除锁停后再向下游请求一次解停。"""
    ok = _pub_bool('rearm', True)
    if ok:
        _patrol_downstream['rearmsg'] += 1
    return ok


def patrol_downstream_lock_stop(reason):
    """runner 锁停 → 撤回下游鉴权：让下游立即锁停并记录真实原因。

    下游需要的不是"原因文本"而是"网页许可已不可信"这一事实：撤回
    /guard_authorize 后下游按全局停止锁停，直到下一次用户显式恢复。
    原因文本按 :func:`patrol.guard_stop_cause` 映射成 guard ``stop_cause``
    枚举后记入 ``last_stop_cause``（B5：与下游 WriteRecord.stop_cause 同
    一套字面，页面/日志可对账），不改变撤回行为本身。
    """
    _patrol_downstream['last_forward'] = None
    _patrol_downstream['last_stop_cause'] = P.guard_stop_cause(reason)
    patrol_downstream_authorize(False)
    return True
# 控制页心跳周期。前端每 1 秒重发一次 view=control。
# [2026-11-17] 曾定义在 SESSION_WINDOW（模块级更早处使用）之后，
# 模块导入即 NameError：整个网页模块（含 test_web_*）无法加载。
VIEW_BEAT_S = 1.0
# 浏览器多久没发控制消息，就停止给它推图像（秒）。页面崩溃也能自己停。
CLIENT_TIMEOUT = 20.0
# ---- 2026-11-14 控制与视频分离（任务书 T4）----
# 显示帧发送的可写等待时限（秒）。超时未可写就丢弃这一帧显示数据：
# 显示数据可丢，停车状态与控制消息不允许被发送队列无限期占住。
DISPLAY_SEND_TIMEOUT_S = 0.5
# 视频连接连续丢弃多少帧后主动断开（慢客户端降级）。前端会自动重连。
VIDEO_DROP_LIMIT = 60
# 当前"已登录且在控制页"的控制连接登记。视频连接只在登记非空时推送，
# 与旧实现"未登录不推画面"的边界一致。
_live_controls = set()
# 当前在实时页（view=map/control）的控制连接。巡航只在"网页会话还在"时
# 运行：最后一个实时页连接断开（或心跳超时）即判会话失效 → 取消并锁停
# （计划 §3"任何断线不自动恢复"）。与 _live_controls 的区别只是不要求
# 登录——地图页免登录也能看巡航状态，但同样算"网页在"。
_live_sessions = set()
_live_sessions_lock = threading.Lock()
# 每次 select 命中后最多读几包。浏览器会把 view 与控制帧并进同一个包，
# 只读一包会漏掉后面的登录或巡航开关。
_WS_READS_PER_TICK = 16
# 控制帧日志：最近若干条控制消息与发出的回复。排查"点了没反应"时
# 用 /api/wslog 只读查看，确认帧到底有没有进 _ws_buf。
WS_LOG_MAX = 50
_ws_log = []
# 握手后多久还没有任何控制帧就记一条警告（秒）。
WS_RX_WARN_S = 5.0
# 最近一条控制帧到达的时刻（monotonic）。0 表示还没收到过任何帧。
_ws_last_frame_t = [0.0]
# 本连接收到的控制帧条数。多连接共用 _ws_last_frame_t 不能判断
# 当前这条连接有没有收到帧，所以按 handler 计数。
_ws_rx_count = [0]


def _ws_conn_begin():
    """新连接开始收帧：清空上一个连接留下的"无帧"警告标记。"""
    with _lock:
        _ws_rx_count[0] = 0


def ws_log_snapshot():
    """返回控制帧日志副本，供 /api/wslog 只读查询。"""
    with _lock:
        return {'last_frame_age':
                None if not _ws_last_frame_t[0]
                else round(time.monotonic() - _ws_last_frame_t[0], 3),
                'items': list(_ws_log)}


def _ws_log_add(kind, detail):
    """记一条控制帧日志。环形覆盖，最多 WS_LOG_MAX 条。"""
    with _lock:
        _ws_log.append({'t': round(time.time(), 3),
                        'kind': kind, 'detail': detail})
        if len(_ws_log) > WS_LOG_MAX:
            del _ws_log[:len(_ws_log) - WS_LOG_MAX]

# 提供安全参数的目标节点。节点名必须与 launch / start_chassis.sh 一致：
# c50c_uart3_node。写错名字时查询直接返回 Node not found，页面会拿本地
# 参数冒充在线值（2026-11-11 已确认的一次数据错配）。
SAFETY_NODE = 'c50c_uart3_node'
SAFETY_PARAMS = ('drive_enable', 'drive_input_enable',
                 'drive_firmware_protocol')
# 在线探测名单：只列本控制台关心的节点。
NODE_WATCH = ('web_bridge', 'camera_node', 'lidar_node', 'c50c_uart3_node',
              'chassis_status_node')
# 开机静止倒计时标记文件（见 start_chassis.sh / c50c-chassis.service）。
BOOT_MARKER = os.environ.get(
    'OPI_BOOT_MARKER', os.path.expanduser('~/.cache/opi-robot/boot_hold'))
# [C50C 2026-11-15] web_debug 进程纪元：每次进程重启都换新值，随状态载荷
# 发给页面。页面发现纪元变了＝服务端已重启而自己还是旧进程时代加载的
# ——旧页面控制 WS 可能已自愈重连（所以显示"已连接"），但视频重连链
# 可能死在后台标签页的定时器节流里（videoWs.onclose 只在控制页排重试，
# openVideo 对卡在 CONNECTING 的半死连接直接返回），表现就是"等待画面…"
# 永不出图。页面据此整页自刷新一次拿全新会话。
SRV_EPOCH = int(time.time())

# ==== 共享状态 ====
_lock = threading.Lock()
_state = {
    'image_b64': '',
    'image_w': 0,
    'image_h': 0,
    'image_mime': 'image/png',
    'image_seq': 0,
    'image_t': None,
    'image_fps': 0.0,
    'scan': None,
    'scan_t': None,
    'scan_hz': 0.0,
    'scan_frames': 0,
    'scan_valid': 0,
    'diag': None,
    'diag_t': None,
    'chassis': None,
    'chassis_t': None,
    'chassis_hz': 0.0,
    'imu_t': None,
    'imu_hz': 0.0,
    'cmd_nav': None,
    'cmd_nav_t': None,
    'cmd_demo': None,
    'cmd_demo_t': None,
    'cmd_manual': None,
    'cmd_manual_t': None,
    'cmd_auto': None,
    'cmd_auto_t': None,
    'status_seq': 0,
    'mode': 'ros',
    'safety': None,
    # [2026-11-17] /drive_safety 原文（fusion_node 唯一判定点）：页面
    # 与导航状态只读这一份，不各自重算雷达/超声结论。
    'drive_safety': None,
    'drive_safety_t': None,
    'nav_seen_t': None,
    'nodes_seen': {},
    'moving': [],
    'moving_t': None,
    # [C50C 2026-11-15] 地图页：占据栅格快照与 map→base_link 位姿。
    # map_grid 是 dict（w/h/res/ox/oy/data/seq），data 为 255=未知、
    # 0..100=占用度的 bytes；pose 为 {'x','y','th'} 或 None。
    'map_grid': None,
    'map_t': None,
    'map_seq': 0,
    'pose': None,
    'pose_t': None,
}

_INITIAL = {k: v for k, v in _state.items() if k != 'mode'}


def reset_state():
    """测试用：把共享状态恢复成初值。

    巡航开关一并复位；发布器留原样，由 Bridge() 重新登记。超声"重新
    启动"发布器登记为不可用（与 Bridge() 未建时一致），避免其他用例的
    Bridge 残留发布器，让"通道不可用"的降级路径可测。
    """
    global _US_REARM_PUB
    _US_REARM_PUB = None
    with _lock:
        for key, value in _INITIAL.items():
            _state[key] = value
        _cruise['on'] = False
        _cruise['publisher'] = None
        _cruise['available'] = False
    configure_patrol(reset_runner=True)


# 模块加载即建巡航状态机（configure_patrol 定义见上）。
configure_patrol(reset_runner=True)


class RateMeter:
    """按固定窗口统计回调频率。窗口到期才更新，避免抖动。"""

    def __init__(self, window=1.0):
        self._window = window
        self._n = 0
        self._t0 = None
        self._hz = 0.0

    def tick(self, now):
        """记一次到达，返回当前频率（可能与上次相同）。"""
        if self._t0 is None:
            self._t0 = now
        self._n += 1
        dt = now - self._t0
        if dt >= self._window:
            self._hz = round(self._n / dt, 1)
            self._n = 0
            self._t0 = now
        return self._hz

    def value(self):
        return self._hz

    def reset(self):
        self._n = 0
        self._t0 = None
        self._hz = 0.0


class Bridge(Node):
    """订阅图像、雷达、诊断、IMU 与速度命令，写入共享状态。"""

    def __init__(self, context=None):
        # context 用于测试脚本在独立域里建节点（见 scripts/ros_isolation.py）。
        # 生产启动走 launch，不传 context，行为不变。
        super().__init__('web_bridge', context=context)
        # 图像与扫描用统一传感器约定，与发布端一致，不得改用默认 RELIABLE
        self.create_subscription(
            CompressedImage, 'image_raw/compressed', self.on_image,
            Q.sensor_qos())
        self.create_subscription(
            LaserScan, 'scan', self.on_scan, Q.sensor_qos())
        self.create_subscription(
            DiagnosticArray, 'diagnostics', self.on_diagnostics,
            Q.status_qos())
        # IMU 心跳：只用来判断节点是否还在出数据，不使用数值
        self.create_subscription(
            Imu, 'imu/data_raw', self.on_imu, Q.sensor_qos())
        # 雷达移动角区：chassis_status_node 标出的"这一圈新出现的角度"，
        # 网页只给它上色，其余点冻结。没有这个发布者时点云照常显示。
        if _U8Arr is not None:
            self.create_subscription(
                _U8Arr, 'robot/moving_bins', self.on_moving,
                Q.status_qos())
        # [C50C 2026-11-20] /map 改为 RELIABLE+TRANSIENT_LOCAL：map_server 以
        # TRANSIENT_LOCAL 发布静态栅格，web_debug 重启后即可自动收到历史地图，
        # 不再依赖手动重调 /map_server/load_map。
        if _OccGrid is not None:
            map_qos = Q.status_qos(1)
            try:
                from rclpy.qos import DurabilityPolicy as _Dur
                map_qos.durability = _Dur.TRANSIENT_LOCAL
            except Exception:
                pass
            self.create_subscription(
                _OccGrid, 'map', self.on_map, map_qos)
        if _TfBuffer is not None:
            try:
                self._tf_buffer = _TfBuffer()
                self._tf_listener = _TfListener(self._tf_buffer, self)
                self.create_timer(0.2, self._pose_tick)
            except Exception:            # pragma: no cover - 垫片/坏环境兜底
                self._tf_buffer = None
        else:
            self._tf_buffer = None
        # 速度命令：显示当前下发值。话题名与 c50c_uart3 的 ros_input 一致。
        # 2026-11-14 起手动与自动分离：本节点自己的手动命令发 /manual_cmd_vel，
        # fusion 自动发 /auto_cmd_vel；/nav_cmd_vel 保留给外部程序。
        self.create_subscription(
            Twist, 'nav_cmd_vel', lambda m: self.on_cmd('nav', m),
            Q.sensor_qos())
        self.create_subscription(
            Twist, 'demo_cmd_vel', lambda m: self.on_cmd('demo', m),
            Q.sensor_qos())
        self.create_subscription(
            Twist, 'manual_cmd_vel', lambda m: self.on_cmd('manual', m),
            Q.sensor_qos())
        self.create_subscription(
            Twist, 'auto_cmd_vel', lambda m: self.on_cmd('auto', m),
            Q.sensor_qos())
        # 安全许可（fusion_node 每拍发布，status_qos 与发布端一致）
        if _String is not None:
            self.create_subscription(
                _String, SAFETY_STATUS_TOPIC, self.on_drive_safety,
                Q.status_qos())
        self._manual_publisher = self.create_publisher(Twist, 'manual_cmd_vel', Q.status_qos())
        # [巡航] 地图文件（路线内容标识的来源）。空 = 不知道地图，拒绝巡航。
        # /patrol_status 发布器与 10Hz 定时器（超时/许可/规划判停）随后建。
        # 真机 declare_parameter 返回 Parameter；测试替身返回 None。
        _pp = self.declare_parameter('patrol_map_yaml', '')
        _pv = getattr(_pp, 'value', '')
        self._patrol_map_yaml = _pv if isinstance(_pv, str) and _pv else None
        configure_patrol(reset_runner=True, map_yaml=self._patrol_map_yaml)
        _patrol['publisher'] = None
        try:
            _patrol['publisher'] = self.create_publisher(
                _String, PATROL_STATUS_TOPIC, Q.status_qos())
        except Exception:                # pragma: no cover
            _patrol['publisher'] = None
        # 下游运动门的输入通道（许可/鉴权/解停）与 runner 的锁停回调。
        # 建失败也不能挡住网页启动：通道为 None 时全部退化 no-op。
        try:
            patrol_downstream_attach(self)
        except Exception:              # pragma: no cover - 坏环境
            _patrol_log_exc('patrol_downstream_attach')
        self.create_timer(PATROL_PERIOD, patrol_tick)
        # 巡航开关：发布器在这里建。缺 std_msgs 时置 None，按钮显示不可用。
        self._cruise_publisher = None
        if _Bool is not None:
            try:
                self._cruise_publisher = self.create_publisher(
                    _Bool, CRUISE_TOPIC, Q.status_qos())
            except Exception:            # pragma: no cover - 仅在坏环境触发
                self._cruise_publisher = None
        with _lock:
            _cruise['publisher'] = self._cruise_publisher
            _cruise['available'] = self._cruise_publisher is not None
        # 超声波"重新启动"发布器（fusion_node 订阅 ultrasonic_rearm）。
        global _US_REARM_PUB
        _US_REARM_PUB = None
        if _Bool is not None:
            try:
                _US_REARM_PUB = self.create_publisher(
                    _Bool, ULTRASONIC_REARM_TOPIC, Q.status_qos())
            except Exception:            # pragma: no cover
                _US_REARM_PUB = None
        _MANUAL.attach(self._publish_manual, ready=self._manual_ready)
        self.create_timer(0.05, _MANUAL.tick)
        # [C50C 阶段 D 2026-11-16] 网页点图导航：/navigate_to_pose action。
        # 缺 nav2_msgs 时 nav_view 的 client 为 false，页面显示不可用。
        self._nav_attached = NG.attach_nav(self)
        _patrol_wire_nav()
        self._img_n = 0
        self._img_t0 = time.time()
        self._sc_n = 0
        self._sc_t0 = time.time()
        self._chassis_rate = RateMeter()
        self._imu_rate = RateMeter()
        # 真机里程积分器：喂固件上报的 vx_mm_s
        self._mileage = MileageIntegrator()

        # 安全开关：先声明本地参数（launch 未传时用默认值），
        # 再尝试读在线节点的参数服务拿真实生效值。
        self.declare_parameter('drive_enable', False)
        self.declare_parameter('drive_input_enable', False)
        self.declare_parameter('drive_firmware_protocol', 'baseline')
        self._list_cli = None
        self._get_cli = None
        self._probe_busy = False
        self._probe_stale = 0
        self._safety_seen_t = None
        self._watch_busy = False
        # 给 chassis_status_node 用的整圈快照服务：它只读订阅不到
        # LaserScan 的内部状态，通过这个服务按需取最近一圈。
        self._scan_srv = None
        if _HAVE_PARAM_SRV:
            try:
                self._list_cli = self.create_client(
                    ListParameters, '/%s/list_parameters' % SAFETY_NODE)
                self._get_cli = self.create_client(
                    GetParameters, '/%s/get_parameters' % SAFETY_NODE)
                self.create_timer(2.0, self.probe_safety)
                self.create_timer(3.0, self.probe_nodes)
            except Exception:
                self._list_cli = None
        self._publish_safety()

    def _manual_ready(self):
        """手动话题有底盘订阅端时就绪；命令桥和 guard 可同时订阅。"""
        subscribers = self.get_subscriptions_info_by_topic('/manual_cmd_vel')
        return any(item.node_name == 'c50c_uart3_node' and item.node_namespace == '/'
                   for item in subscribers)

    def _publish_manual(self, vx, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self._manual_publisher.publish(msg)

    # ---- 订阅回调 ----
    def on_image(self, msg):
        """把 JPEG/PNG 编码为 base64 并记录尺寸。"""
        data = bytes(msg.data)
        w, h = _image_size(data)
        b64 = base64.b64encode(data).decode('ascii')
        now = time.time()
        self._img_n += 1
        dt = now - self._img_t0
        fps = self._img_n / dt if dt > 0.5 else _state['image_fps']
        if dt > 2.0:
            self._img_n = 0
            self._img_t0 = now
        with _lock:
            _state['image_b64'] = b64
            _state['image_w'] = w
            _state['image_h'] = h
            _state['image_mime'] = _sniff_mime(data)
            _state['image_seq'] += 1
            _state['image_t'] = now
            _state['image_fps'] = round(fps, 1)

    def on_scan(self, msg):
        """把 LaserScan 转成角度-距离数组。inf/nan 一律记 0，保证 JSON 合法。"""
        n = len(msg.ranges)
        rmin = msg.range_min
        rmax = msg.range_max
        pts = []
        valid = 0
        inc = msg.angle_increment
        a0 = msg.angle_min
        for i in range(n):
            r = msg.ranges[i]
            deg = round(math.degrees(a0 + inc * i), 2)
            if not (r == r) or r < rmin or r > rmax:
                pts.append([deg, 0.0])
                continue
            valid += 1
            pts.append([deg, round(float(r), 3)])
        now = time.time()
        with _lock:
            _state['scan'] = pts
            _state['scan_t'] = now
            _state['scan_frames'] += 1
            _state['scan_valid'] = valid

        self._sc_n += 1
        dt = now - self._sc_t0
        if dt > 2.0:
            with _lock:
                _state['scan_hz'] = round(self._sc_n / dt, 2)
            self._sc_n = 0
            self._sc_t0 = now

    def on_map(self, msg):
        """存最新占据栅格（地图页 1Hz 增量推送用）。

        ``data`` 里的 int8（-1=未知, 0..100=占用度）转成字节：未知→255，
        其余原值。空栅格（SLAM 刚启动）跳过。
        """
        info = msg.info
        if not info.width or not info.height:
            return
        grid = {
            'w': int(info.width),
            'h': int(info.height),
            'res': float(info.resolution),
            'ox': float(info.origin.position.x),
            'oy': float(info.origin.position.y),
            'data': bytes(v & 0xFF for v in msg.data),
        }
        with _lock:
            _state['map_seq'] += 1
            grid['seq'] = _state['map_seq']
            _state['map_grid'] = grid
            _state['map_t'] = time.time()

    def _pose_tick(self):
        """查 map→base_link 位姿存入共享状态（0.5s 一次）。

        TF 没备好（SLAM 未出第一帧、map 坐标系还没发布）就保持旧值，
        页面按 pose_age 自行判断新鲜度，不显示过期位姿。
        """
        if self._tf_buffer is None:
            return
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link',
                                                 rclpy.time.Time())
        except Exception:
            return
        q = t.transform.rotation
        th = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with _lock:
            _state['pose'] = {
                'x': round(float(t.transform.translation.x), 3),
                'y': round(float(t.transform.translation.y), 3),
                'th': round(th, 4),
            }
            _state['pose_t'] = time.time()

    def on_diagnostics(self, msg):
        """记录最新雷达诊断与底盘状态。解析失败时保持上一次记录不清空。"""
        parsed = S.parse_diagnostic(msg)
        chassis = C.parse_chassis_status(msg)
        now = time.time()
        with _lock:
            if parsed is not None:
                _state['diag'] = parsed
                _state['diag_t'] = now
            if chassis is not None:
                # 里程：对固件上报的 vx 做软件积分。停车不增长，断流不计入。
                vx_mm_s = chassis.get('vx_mm_s')
                if vx_mm_s is not None:
                    self._mileage.update(vx_mm_s, now)
                    chassis['mileage_m'] = round(self._mileage.trip(), 2)
                _state['chassis'] = chassis
                _state['chassis_t'] = now
                _state['chassis_hz'] = self._chassis_rate.tick(now)

    def on_imu(self, msg):
        """只记心跳，不读数值。"""
        now = time.time()
        with _lock:
            _state['imu_t'] = now
            _state['imu_hz'] = self._imu_rate.tick(now)

    def on_moving(self, msg):
        """记录移动角区（0..119 索引，每格 3 度）。"""
        now = time.time()
        with _lock:
            _state['moving'] = [int(b) for b in msg.data]
            _state['moving_t'] = now

    def on_drive_safety(self, msg):
        """``/drive_safety``（String JSON）：只保存，不重算。

        解析失败按"无许可"记录（``raw=False``），页面显示拒绝。
        """
        now = time.time()
        raw = getattr(msg, 'data', None)
        payload = None
        if _String is not None and isinstance(raw, (str, bytes, bytearray)):
            try:
                payload = json.loads(raw)
            except (TypeError, ValueError, UnicodeDecodeError):
                payload = None
        with _lock:
            _state['drive_safety'] = payload if isinstance(payload, dict) else None
            _state['drive_safety_t'] = now

    def on_cmd(self, which, msg):
        now = time.time()
        key = 'cmd_%s' % which
        with _lock:
            _state[key] = _copy_twist(msg)
            _state[key + '_t'] = now
            if which == 'nav':
                _state['nav_seen_t'] = now

    # ---- 安全参数 ----
    def _local_safety(self):
        values = {}
        for name in SAFETY_PARAMS:
            try:
                values[name] = self.get_parameter(name).value
            except Exception:
                values[name] = None
        return values

    def _publish_safety(self, values=None, source='本地启动参数',
                        online=False):
        payload = C.safety_view(values or self._local_safety(), source)
        payload['online'] = bool(online)
        payload['checked_t'] = time.time()
        with _lock:
            _state['safety'] = payload

    # service_is_ready 连续 False 达到该轮数后重建 client：DDS 图缓存
    # 可能漏掉桥重启（USB 重枚举）后的发现事件，长命 client 会永远
    # ready=False，而新建节点立刻能看到服务（2026-11-14 实测）。
    # 2s 探测 × 15 = 30s 自愈上限；桥真离线时重建也无害（开销可忽略）。
    _PROBE_RECREATE_AFTER = 15

    def _recreate_safety_clients(self):
        """销毁并重建参数服务 client，强制重新查询 DDS 图缓存。"""
        for attr in ('_list_cli', '_get_cli'):
            cli = getattr(self, attr)
            if cli is not None:
                try:
                    self.destroy_client(cli)
                except Exception:
                    pass
                setattr(self, attr, None)
        try:
            self._list_cli = self.create_client(
                ListParameters, '/%s/list_parameters' % SAFETY_NODE)
            self._get_cli = self.create_client(
                GetParameters, '/%s/get_parameters' % SAFETY_NODE)
        except Exception:
            self._list_cli = None
        self._probe_stale = 0

    def probe_safety(self):
        """向在线节点读安全参数。读不到就把在线状态标成离线。"""
        if self._list_cli is None or self._probe_busy:
            return
        try:
            if not self._list_cli.service_is_ready():
                # 参数服务不在：底盘桥进程没起来。立刻降级，不沿用上一次
                # 的"在线值"——那会让页面显示一个已经不存在的节点状态。
                self._publish_safety(source='底盘桥离线，显示本地启动参数',
                                     online=False)
                self._probe_stale += 1
                if self._probe_stale >= self._PROBE_RECREATE_AFTER:
                    self._recreate_safety_clients()
                return
            self._probe_stale = 0
            self._probe_busy = True
            future = self._list_cli.call_async(ListParameters.Request())
            future.add_done_callback(self._on_list_params)
        except Exception:
            self._probe_busy = False

    def _on_list_params(self, future):
        try:
            response = future.result()
            names = list(getattr(response, 'result', None).names)
        except Exception:
            self._probe_busy = False
            self._publish_safety(source='参数查询失败，显示本地启动参数',
                                 online=False)
            return
        if not names:
            # 在线节点只声明了参数描述、没声明参数值。直接 get 会卡住
            # 整条探测链，所以不发起查询，如实显示本地值。
            self._probe_busy = False
            self._publish_safety(source='节点在线但未声明参数值',
                                 online=True)
            return
        wanted = [n for n in names if n in SAFETY_PARAMS]
        if not wanted or self._get_cli is None:
            self._probe_busy = False
            return
        try:
            request = GetParameters.Request()
            request.names = wanted
            fut = self._get_cli.call_async(request)
            fut.add_done_callback(self._on_get_params)
        except Exception:
            self._probe_busy = False

    def _on_get_params(self, future):
        try:
            values = list(future.result().values)
        except Exception:
            self._probe_busy = False
            self._publish_safety(source='参数读取失败，显示本地启动参数',
                                 online=False)
            return
        merged = self._local_safety()
        for name, param_value in zip(SAFETY_PARAMS, values):
            kind = getattr(param_value, 'type', 0)
            if kind == 1:               # bool
                merged[name] = bool(param_value.bool_value)
            elif kind == 4:             # string
                merged[name] = param_value.string_value
        self._safety_seen_t = time.time()
        self._publish_safety(merged, '在线节点 %s' % SAFETY_NODE, online=True)
        self._probe_busy = False

    # ---- 在线探测 ----
    def probe_nodes(self):
        """按话题端点与参数服务判断各节点是否在线，结果写进 _state。

        Bridge 只在真 ROS 模式下创建；假模式由 FakeBridge 的定时器调用。
        """
        if self._watch_busy:
            return
        self._watch_busy = True
        now = time.time()
        seen = {}
        try:
            endpoints = {
                'camera_node': self.get_publishers_info_by_topic(
                    '/image_raw/compressed'),
                'lidar_node': self.get_publishers_info_by_topic('/scan'),
                'chassis_status_node': self.get_publishers_info_by_topic(
                    '/diagnostics'),
            }
        except Exception:
            endpoints = {}
        for node_name, infos in endpoints.items():
            alive = any(getattr(i, 'node_name', '') == node_name
                        for i in infos)
            if alive:
                seen[node_name] = True
        if self._safety_seen_t is not None and now - self._safety_seen_t < 6.0:
            seen['c50c_uart3_node'] = True
        seen['web_bridge'] = True
        with _lock:
            _state['nodes_seen'] = {k: (now if v else None)
                                    for k, v in seen.items()}
        self._watch_busy = False


def _copy_twist(msg):
    """把 Twist 复制成简单容器，避免持有 ROS 消息对象。"""
    linear = getattr(msg, 'linear', None)
    angular = getattr(msg, 'angular', None)
    return {
        'vx': float(getattr(linear, 'x', 0.0)) if linear is not None else 0.0,
        'vy': float(getattr(linear, 'y', 0.0)) if linear is not None else 0.0,
        'wz': float(getattr(angular, 'z', 0.0)) if angular is not None else 0.0,
    }


class FakeBridge:
    """假数据模式：不建 ROS 节点，按 20 Hz 填共享状态。"""

    def __init__(self, t0=None):
        self._t0 = time.time() if t0 is None else t0
        self._source = F.FakeSource(t0=self._t0)
        self._stop = threading.Event()
        self._thread = None
        self._rate = RateMeter()
        self._imu_rate = RateMeter()
        self._mileage = MileageIntegrator()

    def start(self):
        with _lock:
            _state['mode'] = 'fake'
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _run(self):
        period = 1.0 / 20.0
        cycle = 0
        while not self._stop.is_set():
            now = time.time()
            status = self._source.status(now)
            with _lock:
                _state['chassis'] = status
                _state['chassis_t'] = now
                _state['chassis_hz'] = self._rate.tick(now)
                # 假模式没有真实里程，按同一套规则积分，页面才不是空
                self._mileage.update(status['vx_mm_s'], now)
                status['mileage_m'] = round(self._mileage.trip(), 2)
                # 假 IMU 按 50 Hz 记心跳，与真实 imu 话题频率同量级
                _state['imu_t'] = now
                _state['imu_hz'] = self._imu_rate.tick(now)
            # 雷达 10 Hz，与常见的 RPLIDAR 一致
            if cycle % 2 == 0:
                pts = self._source.scan(now)
                with _lock:
                    _state['scan'] = pts
                    _state['scan_t'] = time.time()
                    _state['scan_frames'] += 1
                    _state['scan_valid'] = sum(1 for _, r in pts if r > 0)
                    _state['scan_hz'] = 10.0
                    _state['diag'] = {
                        'level': 0, 'message': 'synthetic', 'state': 'healthy',
                        'reason': '', 'age_s': 0.0, 'coverage': 0.97,
                        'reconnects': 0, 'port': '/dev/fake', 'frames_ok': 1,
                        'frames_bad': 0}
                    _state['diag_t'] = time.time()
            img = self._source.image(now)
            if img is not None:
                w, h = _image_size(img)
                with _lock:
                    _state['image_b64'] = base64.b64encode(img).decode('ascii')
                    _state['image_w'] = w
                    _state['image_h'] = h
                    _state['image_mime'] = 'image/png'
                    _state['image_seq'] += 1
                    _state['image_t'] = time.time()
                    _state['image_fps'] = 5.0
            # 命令侧：假模式下没有真的下发者。保持"无命令"，
            # 否则概览页会一直显示"导航前往任务目标中"，跟实车行为不符。
            with _lock:
                _state['cmd_nav'] = None
                _state['cmd_nav_t'] = None
                _state['cmd_demo'] = None
                _state['cmd_manual'] = None
                _state['cmd_auto'] = None
                safety = C.safety_view(
                    {'drive_enable': True, 'drive_input_enable': True,
                     'drive_firmware_protocol': 'baseline'},
                    '假数据模式')
                safety['online'] = True
                safety['checked_t'] = time.time()
                _state['safety'] = safety
                # 假模式没有 ROS 发布者，节点灯按"数据在流"点亮，
                # 否则整块节点卡在演示时全是灰的。
                _state['nodes_seen'] = {
                    'camera_node': now, 'lidar_node': now,
                    'chassis_status_node': now, 'c50c_uart3_node': now,
                    'web_bridge': now}
            cycle += 1
            time.sleep(period)


NET_CACHE_S = 5.0                      # 网络信息缓存，避免每帧都开进程
_net_cache = {'t': 0.0, 'data': None}


def _net_cmd(argv):
    """跑一条只读网络命令，失败一律当空输出处理。"""
    try:
        p = subprocess.run(argv, stdout=subprocess.PIPE,
                           stderr=subprocess.DEVNULL, timeout=2.0)
    except (OSError, subprocess.SubprocessError):
        return ''
    return p.stdout.decode('utf-8', 'replace')


def _wireless_iface():
    """找一块无线网卡：能连 WiFi 就不会没有。"""
    base = '/sys/class/net'
    try:
        names = sorted(os.listdir(base))
    except OSError:
        return ''
    for name in names:
        if os.path.isdir(os.path.join(base, name, 'wireless')):
            return name
    return ''


def _parse_iw_link(text):
    """解析 `iw dev <if> link` 的文本输出。"""
    out = {}
    for line in text.splitlines():
        line = line.strip()
        if line.startswith('SSID:'):
            out['ssid'] = line.split(':', 1)[1].strip()
        elif line.startswith('freq:'):
            try:
                out['freq_mhz'] = int(float(line.split(':', 1)[1]))
            except ValueError:
                pass
        elif line.startswith('signal:'):
            try:
                out['signal_dbm'] = int(float(
                    line.split(':', 1)[1].replace('dBm', '').strip()))
            except ValueError:
                pass
        elif line.startswith('tx bitrate:'):
            try:
                out['tx_mbps'] = float(line.split(':', 1)[1].split()[0])
            except (ValueError, IndexError):
                pass
        elif line.startswith('rx bitrate:'):
            try:
                out['rx_mbps'] = float(line.split(':', 1)[1].split()[0])
            except (ValueError, IndexError):
                pass
    return out


def _parse_ipv4(text, iface):
    """从 `ip -4 -o addr show` 里取指定网卡的 IPv4。"""
    for line in text.splitlines():
        parts = line.split()
        if iface in parts and 'inet' in parts:
            return parts[parts.index('inet') + 1]
    return None


def _parse_ipv4_list(text, iface):
    """取指定网卡的全部 IPv4（去掩码）。一个网卡可能挂了多个地址。"""
    out = []
    for line in text.splitlines():
        parts = line.split()
        if iface in parts and 'inet' in parts:
            out.append(parts[parts.index('inet') + 1].split('/')[0])
    return out


def network_info(now=None):
    """采集无线网络状态。结果缓存 NET_CACHE_S 秒。"""
    now = time.time() if now is None else now
    data = _net_cache['data']
    if data is not None and now - _net_cache['t'] < NET_CACHE_S:
        return data
    iface = _wireless_iface()
    info = {
        'iface': iface or None,
        'state': 'down',
        'ssid': None,
        'ip': None,
        'ips': [],
        'mac': None,
        'signal_dbm': None,
        'quality': None,
        'band': None,
        'tx_mbps': None,
        'rx_mbps': None,
    }
    if iface:
        try:
            with open('/sys/class/net/%s/address' % iface) as fh:
                info['mac'] = fh.read().strip().upper()
        except OSError:
            pass
        link = _net_cmd(['iw', 'dev', iface, 'link'])
        if 'Connected to' in link or 'SSID:' in link:
            info['state'] = 'up'
            info.update(_parse_iw_link(link))
        info['ip'] = _parse_ipv4(_net_cmd(['ip', '-4', '-o', 'addr', 'show',
                                           'dev', iface]), iface)
        info['ips'] = _parse_ipv4_list(_net_cmd(['ip', '-4', '-o', 'addr',
                                                  'show', 'dev', iface]), iface)
        dbm = info.get('signal_dbm')
        if dbm is not None:
            info['quality'] = max(0, min(100, 2 * (dbm + 100)))
        freq = info.get('freq_mhz')
        if freq:
            info['band'] = '5 GHz' if freq >= 5000 else '2.4 GHz'
        if info['state'] == 'up' and not info.get('ssid'):
            info['state'] = '有信号但读不到名称'
    _net_cache['t'] = now
    _net_cache['data'] = info
    return info


HOST_CACHE_S = 1.0
_host_cache = {'t': 0.0, 'data': None, 'prev': None}
_THERMAL_ROOT = '/sys/class/thermal'


def _read_text(path):
    try:
        with open(path) as fh:
            return fh.read()
    except OSError:
        return ''


def _cpu_times(text):
    """从 /proc/stat 首行取 (total, idle) 累计值。"""
    parts = text.split()
    if len(parts) < 5 or parts[0] != 'cpu':
        return None
    try:
        vals = [int(x) for x in parts[1:9]]
    except ValueError:
        return None
    total = sum(vals)
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    return total, idle


def _mem_values(text):
    """从 /proc/meminfo 取 (total_kb, available_kb)。"""
    total = avail = None
    for line in text.splitlines():
        if line.startswith('MemTotal:'):
            total = int(line.split()[1])
        elif line.startswith('MemAvailable:'):
            avail = int(line.split()[1])
    return total, avail


def _max_temp_c():
    """取 thermal_zone 里最高的温度，单位 ℃。"""
    try:
        names = [n for n in os.listdir(_THERMAL_ROOT)
                 if n.startswith('thermal_zone')]
    except OSError:
        return None
    best = None
    for name in names:
        raw = _read_text(os.path.join(_THERMAL_ROOT, name, 'temp')).strip()
        try:
            val = int(raw)
        except ValueError:
            continue
        val = val / 1000.0 if val > 1000 else float(val)
        if best is None or val > best:
            best = val
    return best


def host_info(now=None):
    """采集主机 CPU、温度、内存、磁盘。结果缓存 HOST_CACHE_S 秒。"""
    now = time.time() if now is None else now
    data = _host_cache['data']
    if data is not None and now - _host_cache['t'] < HOST_CACHE_S:
        return data

    info = {
        'cpu_pct': None,
        'load1': None,
        'temp_c': None,
        'mem_used_pct': None,
        'mem_used_mb': None,
        'mem_total_mb': None,
        'disk_used_pct': None,
        'disk_total_gb': None,
        'disk_free_gb': None,
    }

    cur = _cpu_times(_read_text('/proc/stat'))
    prev = _host_cache['prev']
    if cur is not None and prev is not None:
        dt = cur[0] - prev[0]
        di = cur[1] - prev[1]
        if dt > 0:
            info['cpu_pct'] = max(0.0, min(100.0, 100.0 * (dt - di) / dt))
    if cur is not None:
        _host_cache['prev'] = cur

    raw_load = _read_text('/proc/loadavg').split()
    if raw_load:
        try:
            info['load1'] = float(raw_load[0])
        except ValueError:
            pass

    info['temp_c'] = _max_temp_c()

    total_kb, avail_kb = _mem_values(_read_text('/proc/meminfo'))
    if total_kb:
        used_kb = max(0, total_kb - (avail_kb or 0))
        info['mem_total_mb'] = int(total_kb / 1024)
        info['mem_used_mb'] = int(used_kb / 1024)
        info['mem_used_pct'] = round(100.0 * used_kb / total_kb, 1)

    try:
        st = os.statvfs('/')
    except OSError:
        st = None
    if st is not None and st.f_blocks:
        unit = st.f_frsize or st.f_bsize or 4096
        total_b = st.f_blocks * unit
        free_b = st.f_bavail * unit
        used_b = total_b - free_b
        info['disk_total_gb'] = round(total_b / 1e9, 1)
        info['disk_free_gb'] = round(free_b / 1e9, 1)
        info['disk_used_pct'] = round(100.0 * used_b / total_b, 1)

    _host_cache['t'] = now
    _host_cache['data'] = info
    return info


def status_payload(now=None):
    """按新鲜度生成状态载荷。过期数据不会显示为实时。"""
    now = time.time() if now is None else now
    with _lock:
        diag = _state['diag']
        diag_t = _state['diag_t']
        scan_t = _state['scan_t']
        image_t = _state['image_t']
        chassis = _state['chassis']
        chassis_t = _state['chassis_t']
        chassis_hz = _state['chassis_hz']
        imu_t = _state['imu_t']
        imu_hz = _state['imu_hz']
        cmd_nav = _state['cmd_nav']
        cmd_nav_t = _state['cmd_nav_t']
        cmd_demo = _state['cmd_demo']
        cmd_demo_t = _state['cmd_demo_t']
        cmd_manual = _state['cmd_manual']
        cmd_manual_t = _state['cmd_manual_t']
        cmd_auto = _state['cmd_auto']
        cmd_auto_t = _state['cmd_auto_t']
        safety = _state['safety']
        scan_hz = _state['scan_hz']
        image_fps = _state['image_fps']
        scan_frames = _state['scan_frames']
        scan_valid = _state['scan_valid']
        mode = _state['mode']
        drive_safety = _state.get('drive_safety')
        drive_safety_t = _state.get('drive_safety_t')
        nodes_seen = dict(_state['nodes_seen'])
        nav_seen_t = _state['nav_seen_t']
        moving = list(_state['moving'])
        moving_t = _state.get('moving_t')
    view = S.sensor_view(diag, diag_t, scan_t, image_t, now,
                         scan_timeout=SCAN_TIMEOUT,
                         image_timeout=IMAGE_TIMEOUT,
                         diag_timeout=DIAG_TIMEOUT)
    view['t'] = 'status'
    view['mode'] = mode
    view['chassis'] = C.chassis_view(chassis, chassis_t, now, CHASSIS_TIMEOUT)
    view['chassis']['hz'] = chassis_hz
    view['imu'] = C.imu_view(imu_t, imu_hz, now, IMU_TIMEOUT)
    view['cmd'] = {
        'nav': C.cmd_view(cmd_nav, cmd_nav_t, now, CMD_TIMEOUT),
        'demo': C.cmd_view(cmd_demo, cmd_demo_t, now, CMD_TIMEOUT),
        'manual': C.cmd_view(cmd_manual, cmd_manual_t, now, CMD_TIMEOUT),
        'auto': C.cmd_view(cmd_auto, cmd_auto_t, now, CMD_TIMEOUT),
    }
    view['safety'] = safety
    # 许可与运动方向门的原文（只读，不在页面侧重算）；同一份对象
    # 交给导航状态（nav relay 侧），三处消费者读同一判断结果。
    ds_view = drive_safety_view(drive_safety, drive_safety_t, now)
    view['drive_safety'] = ds_view
    view['nav'] = NG.nav_view(ds_view)
    view['scan_hz'] = scan_hz
    view['image_fps'] = image_fps
    view['scan_frames'] = scan_frames
    view['scan_valid'] = scan_valid
    view['network'] = network_info(now)
    view['host'] = host_info(now)
    view['boot'] = V.boot_view(BOOT_MARKER, time.monotonic())
    view['srv_epoch'] = SRV_EPOCH
    # 底盘在线：诊断帧新鲜，或底盘桥参数服务可达
    chassis_up = view['chassis']['state'] == S.FRESH
    bridge_up = bool(safety and safety.get('online'))
    view['nodes'] = node_view(nodes_seen, now, chassis_up, bridge_up,
                              bool(scan_frames) and not nodes_seen.get(
                                  'lidar_node'))
    nav = view['cmd']['nav']
    view['task'] = _task_view(nav, chassis, safety, now, nav_seen_t)
    view['battery'] = V.battery_view(view['chassis']['voltage_v'])
    # 巡航开关：on 是脚本侧已下发的值，available 表示发布器就绪。
    # 雷达能否让车走由 fusion_node 判，不在这里猜。
    view['cruise'] = cruise_view()
    # [巡航] 航点巡航 + 运动许可（地图标识/当前航点/运行状态/停车原因/
    # 锁停原因/许可状态）。许可不新鲜时 released 必为 False。
    view['patrol'] = patrol_status_frame(now=now)
    # 移动标记只保留 2 秒：雷达停更后旧标记不能一直亮着
    view['moving'] = (moving if moving_t is not None
                      and now - moving_t < 2.0 else [])
    return view


def drive_safety_view(payload, last_t, now):
    """``/drive_safety`` 的展示视图。不做任何判定，只做新鲜度与字段转述。

    过期（超过消息自带 ttl_s，或没带 ttl 时按 1 s）时 ``state='stale'``，
    ``allowed=None``——过期许可不等于允许，也不等于拒绝，页面显示"无许可"。
    """
    if payload is None or last_t is None:
        return {'state': S.UNKNOWN, 'allowed': None, 'reason': None,
                'age': None, 'motion': None, 'ultrasonic': None}
    age = now - last_t
    ttl = payload.get('ttl_s')
    try:
        ttl = float(ttl)
    except (TypeError, ValueError):
        ttl = 1.0
    fresh = age <= ttl
    return {
        'state': S.FRESH if fresh else S.STALE,
        'allowed': payload.get('allowed') if fresh else None,
        'reason': payload.get('reason') if fresh else None,
        'age': round(age, 2),
        'seq': payload.get('seq'),
        # fusion_node 唯一判定点给出的运动方向结论（与自动命令同源）
        'motion': payload.get('motion'),
        'ultrasonic': payload.get('ultrasonic'),
    }


def node_view(seen, now, chassis_up, bridge_up, scan_but_no_node):
    """各节点在线状态。返回 {name: 'online'|'offline'}。

    chassis_status_node 与 c50c_uart3_node 不是同一回事：前者只读发诊断，
    后者持有串口。两个都要能单独显示，避免"一个绿就全绿"。
    """
    def online(name, timeout=6.0):
        t = (seen or {}).get(name)
        return t is not None and now - t < timeout

    return {
        'camera_node': 'online' if online('camera_node') else 'offline',
        'lidar_node': 'online' if online('lidar_node') else 'offline',
        'c50c_uart3_node': 'online' if bridge_up else 'offline',
        'chassis_status_node': ('online' if online('chassis_status_node')
                                else 'offline'),
        'web_bridge': 'online',
        'scan_without_node': bool(scan_but_no_node),
    }


def _task_view(nav, chassis, safety, now, nav_seen_t):
    """导航/巡逻/回充/无任务。目前没有导航与巡逻节点，如实显示无任务。"""
    manual_active = _MANUAL.snapshot().get('active', False)
    manual_debug = _MANUAL.snapshot_debug()
    moving = (nav.get('state') == S.FRESH
              and ((abs(nav.get('vx_m_s') or 0.0) > 1e-6)
                   or (abs(nav.get('wz_rad_s') or 0.0) > 1e-6)))
    state, text = V.task_state(moving, nav.get('vx_m_s'),
                               nav.get('wz_rad_s'),
                               nav.get('age'), manual_active,
                               battery_low=False, patrol_fresh=False)
    return {
        'state': state,
        'text': text,
        'cmd_vx': nav.get('vx_m_s'),
        'cmd_wz': nav.get('wz_rad_s'),
        # 命令通道是否就绪：底盘状态帧新鲜，或底盘桥参数服务可达
        'channel': bool((chassis or {}).get('state') == S.FRESH
                        or (safety or {}).get('online')),

        'manual_stops': manual_debug.get('stops'),
        'manual_owner': manual_debug.get('owner'),
        'manual_beats': manual_debug.get('beats'),
        'manual_pubbed': manual_debug.get('pubbed'),
        'manual_gap': manual_debug.get('gap'),
        'manual_starts': manual_debug.get('starts'),
    }


def _sniff_mime(data):
    """按魔数判断图像类型。"""
    if data[:8] == b'\x89PNG\r\n\x1a\n':
        return 'image/png'
    return 'image/jpeg'


def _image_size(data):
    """按魔数读宽高，JPEG 与 PNG 都支持。"""
    if data[:8] == b'\x89PNG\r\n\x1a\n' and len(data) >= 24:
        w = int.from_bytes(data[16:20], 'big')
        h = int.from_bytes(data[20:24], 'big')
        return w, h
    return _jpeg_size(data)


def _jpeg_size(data):
    """从 JPEG 头读宽高。"""
    i = 2
    while i < len(data) - 9:
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker in (0xC0, 0xC1, 0xC2, 0xC3):
            h = (data[i + 5] << 8) | data[i + 6]
            w = (data[i + 7] << 8) | data[i + 8]
            return w, h
        seg = (data[i + 2] << 8) | data[i + 3]
        i += 2 + seg
    return 0, 0


_HERE = os.path.dirname(os.path.abspath(__file__))


def _page_candidates(here=None):
    """页面候选路径：模块同级优先，其次任意祖先下 lib/opi_robot/web_node/。"""
    here = here or _HERE
    out = [os.path.join(here, 'console_page.html')]
    cur = here
    for _ in range(6):
        cur = os.path.dirname(cur)
        if not cur or cur == '/':
            break
        out.append(os.path.join(cur, 'lib', 'opi_robot', 'web_node',
                                'console_page.html'))
    return tuple(dict.fromkeys(out))


def load_page():
    """读取页面文件；源码目录与原位安装两种布局都覆盖。

    页面单独放 console_page.html，便于前端改动不碰后端逻辑；
    文件缺失时退回包内嵌的 _PAGE_FALLBACK，保证服务仍能启动。
    """
    for path in _page_candidates():
        try:
            with open(path, 'r', encoding='utf-8') as fp:
                text = fp.read()
        except OSError:
            continue
        if text.strip():
            return text
    return _PAGE_FALLBACK


# 兜底页面：console_page.html 缺失时使用，内容与新页面同源但极简，
# 只保证状态可读，不依赖任何外部资源。
_PAGE_FALLBACK = r"""<!doctype html>
<html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>小车控制台</title></head>
<body id="page-overview">
<p>页面文件 console_page.html 未找到，当前为兜底视图。</p>
<pre id="raw">等待数据…</pre>
<script>
fetch('/api/state').then(r => r.json()).then(d => {
  document.getElementById('raw').textContent = JSON.stringify(d, null, 2);
});
</script>
</body></html>
"""


PAGE = load_page()



class Handler(BaseHTTPRequestHandler):
    """HTTP 与 WebSocket 共用同一端口。"""

    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):
        """关闭默认日志，避免刷屏。"""
        return

    def version_string(self):
        """不向局域网暴露 Python/BaseHTTP 版本号。"""
        return 'opi-robot'

    def do_GET(self):
        """返回页面、JSON 状态或升级到 WebSocket。"""
        if self.path == '/favicon.ico':
            # 浏览器总是自己请求图标。回 204，避免控制台出现 404 报错。
            self.send_response(204)
            self.send_header('Content-Length', '0')
            self.end_headers()
            return
        if self.path in ('/', '/index.html'):
            body = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            # 禁缓存：否则浏览器拿旧页面跑旧脚本，改完刷新也不生效。
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == '/api/wslog':
            # 只读排障入口：最近收到的控制帧与发出的握手回复。
            # 现场"点了没反应"时不用改代码就能分清：帧没到、还是回复没发。
            body = json.dumps(ws_log_snapshot(), separators=(',', ':'),
                              allow_nan=False).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == '/api/state':
            # 便于离线验证断言：不需要 WebSocket 也能读到当前状态
            body = json.dumps(status_payload(), separators=(',', ':'),
                              allow_nan=False).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == '/ws/video' and self.headers.get('Upgrade', '').lower() == 'websocket':
            # 2026-11-14 起图像与点云走独立连接：慢的视频客户端不能占住
            # 控制消息的收发路径（任务书 T4）。
            self._websocket(video=True)
            return
        if self.path == '/ws' and self.headers.get('Upgrade', '').lower() == 'websocket':
            self._websocket()
            return
        self.send_error(404)

    def _websocket(self, video=False):
        """完成握手后循环推送数据与状态。

        ``video=False``：控制连接——承载控制消息（登录/手动/巡航）与小体积
        状态帧。图像与点云不再走这条连接（2026-11-14 分离，任务书 T4）。
        ``video=True``：视频连接——只推图像与点云，显示数据可丢弃，慢客户端
        连续超限后降级断开；它读不到控制消息也不影响控制连接。
        """
        key = self.headers.get('Sec-WebSocket-Key', '')
        accept = base64.b64encode(
            hashlib.sha1(
                (key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()
            ).digest()).decode()
        # 握手响应手工拼字节，不走 send_response()。
        # BaseHTTPRequestHandler 会给 1xx 响应补 Transfer-Encoding: chunked，
        # 之后 wfile 会在每个 flush 追加 "0\r\n\r\n" 结束块，
        # 混进 WebSocket 帧流里就是坏帧，浏览器报 Could not decode UTF-8 并断开。
        resp = (
            'HTTP/1.1 101 Switching Protocols\r\n'
            'Upgrade: websocket\r\n'
            'Connection: Upgrade\r\n'
            'Sec-WebSocket-Accept: %s\r\n'
            '\r\n' % accept
        ).encode('ascii')
        # 只要握手成功，HTTP 层的响应处理就结束了，后面全是 WebSocket 帧。
        self.close_connection = True
        self.connection.sendall(resp)
        # 握手后显式恢复阻塞模式：服务端 push 大帧（图像约 70 KB）
        # 依赖阻塞 sendall 一次写完整帧，非阻塞会中途失败。
        try:
            self.connection.settimeout(None)
        except OSError:
            pass
        # 不能把 wfile 置 None：socketserver 的 finish() 与
        # handle_one_request() 都会访问 self.wfile.closed / flush()，
        # 置 None 会让每个连接结束时抛 AttributeError 并污染日志。
        # 保留 wfile，但改用 socket 直发（_send_text 走 raw），
        # 这样既绕开缓冲，也不破坏标准生命周期。
        # 每连接一把发送锁：保证一帧的帧头与载荷不被其它帧插队。
        self._ws_lock = threading.Lock()
        # 读取自己维护字节缓冲，不再用 rfile：
        # rfile 是 http.server 的 64 KB BufferedReader，
        # 给它设读超时后一旦超时就作废，后到的消息永远读不出来。
        # 这里只用原始 recv 收字节，帧边界自己解析（见 _decode_ws_frames）。
        # 握手时 rfile 会把"请求头 + 紧跟的控制帧"一起读进 64 KB 缓冲。
        # 那一刻 socket 上已经没有可读字节，select 永远不命中，
        # 只在 _read_view 里等 select 再取 rfile 就拿不到首帧。
        # 所以在 socket 刚交给 WebSocket、还没开始推送时先取一次。
        # peek 在缓冲为空时会读 socket，临时设为非阻塞避免等待首帧。
        self.connection.setblocking(False)
        try:
            self._ws_buf = bytearray(_take_buffered(self))
        finally:
            self.connection.setblocking(True)
        self._ws_drained = True
        if video:
            self._video_loop()
            return
        # 连接建立时刻：超过 WS_RX_WARN_S 还没有任何控制帧就记一条警告。
        # 浏览器握完手一定会先发 view=idle，长时间收不到多半是握手包
        # 把首帧吃进了 rfile 或者公网/反代改了缓冲，不记就查不到。
        self._ws_enter_t = time.monotonic()
        self._ws_rx_warned = False
        _ws_conn_begin()
        next_status_t = 0.0
        # 是否在实时页（控制/地图）。地图页免登录也保持 live——手动会话
        # 与心跳超时语义不变；但只有"已登录且在控制页"的会话才登记进
        # _live_controls（视频连接据此推摄像头）。2026-11-15 起"看"与
        # "开"分离：栅格/点云免登录直推，驾驶鉴权在 _manual_message。
        live = False
        # 未登录却请求实时画面时只提示一次；登录成功或切走后复位
        refused = False
        next_msg_t = time.time() + CLIENT_TIMEOUT
        # [巡航] 会话看门狗：读循环卡住（代理半开、客户端不发 RST）时
        # 仍按 wall clock 判会话失效并锁停，不依赖本条连接的读循环。
        _watch_stop = threading.Event()
        _watch = threading.Thread(
            target=patrol_session_watch,
            args=(_watch_stop.is_set, self),
            name='patrol-session-watch', daemon=True)
        _watch.start()
        try:
            while True:
                # 读浏览器发来的控制消息：切到控制页要实时数据，切走就停。
                live, next_msg_t, refused = self._read_view(
                    live, next_msg_t, refused)
                if (not getattr(self, '_ws_rx_warned', False)
                        and _ws_rx_count[0] == 0
                        and time.monotonic() - getattr(
                            self, '_ws_enter_t', time.monotonic())
                        > WS_RX_WARN_S):
                    self._ws_rx_warned = True
                    print('websocket %.0f 秒未收到任何控制帧'
                          '（浏览器 view=idle 都没到），检查握手与代理缓冲'
                          % WS_RX_WARN_S, flush=True)
                now = time.time()
                # 状态按固定周期推，断流时也能让网页立刻显示过期。
                # 状态是小体积显示数据：客户端收不动时丢弃本帧（可丢），
                # 不允许阻塞控制消息的读取。
                if now >= next_status_t:
                    next_status_t = now + STATUS_PERIOD
                    # 先序列化再发：JSON 失败只丢这一帧状态，不能打断整条连接。
                    # allow_nan=False 时任何 nan/inf 都会抛 ValueError，
                    # 雷达距离或 IMU 四元数一旦出现 nan 就会踩到。
                    try:
                        payload = self._status_msg(now)
                        text = json.dumps(payload,
                                          separators=(',', ':'),
                                          allow_nan=False)
                    except (ValueError, TypeError) as exc:
                        self.server.stats['status_bad'] += 1
                        if self.server.stats['status_bad'] <= 3:
                            print('状态帧序列化失败（已跳过）: %r' % (exc,),
                                  flush=True)
                    else:
                        _send_display(self, text)
                time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return
        except Exception:
            import traceback
            traceback.print_exc()
            return
        finally:
            # 看门狗先停：它自己的收尾也会判会话结束，避免与本处重复/竞争。
            _watch_stop.set()
            _live_controls.discard(self)
            _MANUAL.disconnect(self)
            # 最后一个实时页连接的巡航会话结束（取消并锁停，不自动恢复）。
            try:
                patrol_session_closed(self)
            except Exception:          # noqa: BLE001
                _patrol_log_exc('patrol_session_closed')

    def _video_loop(self):
        """视频连接主循环：只推图像与点云。

        套接字带 :data:`DISPLAY_SEND_TIMEOUT_S` 发送时限：客户端收不动时
        select 预检先丢帧；`sendall` 仍超时说明发生了**部分写入**——字节流
        已可能被切开，此时唯一安全动作是断开本视频连接（慢客户端降级，
        前端自动重连）。本循环不处理任何控制消息——控制与停车不经此路。
        """
        try:
            self.connection.settimeout(DISPLAY_SEND_TIMEOUT_S)
        except OSError:
            pass
        last_img = -1
        last_scan = -1
        last_map_seq = -1
        last_pose_t = None
        # 扫描点整圈约 8 KB，100 Hz 后台每秒能跑出 100 份，Wi-Fi 扛不住。
        # 画面上限 10 Hz 足够顺滑，也避免积压把图像挤掉。
        scan_period = 0.1
        # 地图推送：SLAM 栅格变化不快，1Hz 且仅在 seq 变化时推。
        # 静态地图(AMCL)下栅格 seq 不再变，用独立轻量 pose 帧按 5Hz 推实时位姿。
        map_period = 1.0
        pose_period = 0.2
        next_map_t = 0.0
        next_pose_t = 0.0
        # 图像节流：Wi-Fi 带宽窄，图像最多每秒推 IMG_HZ 帧。
        img_period = 1.0 / max(1.0, IMG_HZ)
        next_img_t = 0.0
        next_scan_t = 0.0
        drops = 0
        try:
            while True:
                with _lock:
                    seq = _state['image_seq']
                    b64 = _state['image_b64']
                    w = _state['image_w']
                    h = _state['image_h']
                    mime = _state['image_mime']
                    fps = _state['image_fps']
                    pts = _state['scan']
                    hz = _state['scan_hz']
                    sframes = _state['scan_frames']
                    valid = _state['scan_valid']
                now = time.time()
                # 画面推送边界（2026-11-15 重定"看/开分离"）：点云与占据
                # 栅格是只读遥测，地图页免登录可看——推给任何连上来的
                # 视频客户端；摄像头只在控制页显示，仍以 _live_controls
                # （已登录的控制页会话）为闸，无人观看时不白耗相机带宽。
                if (seq != last_img and b64 and now >= next_img_t
                        and _live_controls):
                    last_img = seq
                    next_img_t = now + img_period
                    try:
                        text = json.dumps({
                            't': 'img', 'd': b64, 'w': w, 'h': h, 'fps': fps,
                            'mime': mime,
                        }, separators=(',', ':'), allow_nan=False)
                    except (ValueError, TypeError) as exc:
                        self.server.stats['img_bad'] += 1
                        if self.server.stats['img_bad'] <= 3:
                            print('图像帧序列化失败（已跳过）: %r' % (exc,),
                                  flush=True)
                    else:
                        if _send_display(self, text):
                            drops = 0
                        else:
                            drops += 1
                if (pts is not None and sframes != last_scan
                        and now >= next_scan_t):
                    last_scan = sframes
                    next_scan_t = now + scan_period
                    try:
                        text = json.dumps({
                            't': 'scan', 'p': pts, 'hz': hz,
                            'n': valid, 'all': len(pts),
                        }, separators=(',', ':'), allow_nan=False)
                    except (ValueError, TypeError) as exc:
                        self.server.stats['scan_bad'] += 1
                        if self.server.stats['scan_bad'] <= 3:
                            print('扫描帧序列化失败（已跳过）: %r' % (exc,),
                                  flush=True)
                    else:
                        if _send_display(self, text):
                            drops = 0
                        else:
                            drops += 1
                # [C50C 2026-11-15] 地图帧：栅格 seq 变化才推，1Hz 上限。
                with _lock:
                    grid = _state['map_grid']
                    grid_t = _state['map_t']
                    pose = _state['pose']
                    pose_t = _state['pose_t']
                if (grid is not None and grid['seq'] != last_map_seq
                        and now >= next_map_t):
                    last_map_seq = grid['seq']
                    next_map_t = now + map_period
                    last_pose_t = pose_t
                    next_pose_t = now + pose_period
                    text = _map_frame_json(grid, grid_t, pose, pose_t)
                    if text is not None and not _send_display(self, text):
                        drops += 1
                elif (pose is not None and pose_t is not None
                        and pose_t != last_pose_t and now >= next_pose_t):
                    last_pose_t = pose_t
                    next_pose_t = now + pose_period
                    text = _pose_frame_json(pose, pose_t, grid_t)
                    if text is not None and not _send_display(self, text):
                        drops += 1
                if drops >= VIDEO_DROP_LIMIT:
                    print('视频客户端连续 %d 帧不可写，主动断开（慢客户端'
                          '降级；页面会自动重连）' % drops, flush=True)
                    self.server.stats['video_closed'] = \
                        self.server.stats.get('video_closed', 0) + 1
                    break
                time.sleep(0.02)
        except socket.timeout:
            # 带时限的 sendall 超时：可能有部分字节已写出，字节流不再可信，
            # 只能断开。控制消息不经本连接，停车不受影响。
            print('视频客户端发送超时（收不动），已断开该视频连接', flush=True)
            self.server.stats['video_closed'] = \
                self.server.stats.get('video_closed', 0) + 1
            return
        except (BrokenPipeError, ConnectionResetError, OSError):
            return
        except Exception:
            import traceback
            traceback.print_exc()
            return


    def _status_msg(self, now=None):
        """状态帧。在只读字段原样保留的前提下，附加雷达诊断明细。

        ``status_payload()`` 的字段一个不动；这里只是把 diagnosics 里
        已经解出的覆盖率 / 好帧 / 坏帧 / 重连次数 / 串口补进消息，
        供传感器页显示。缺失就是 None，前端显示「无数据」。
        """
        payload = status_payload(now)
        with _lock:
            diag = _state['diag']
        payload['lidar_detail'] = {
            'coverage': None if diag is None else diag.get('coverage'),
            'frames_ok': None if diag is None else diag.get('frames_ok'),
            'frames_bad': None if diag is None else diag.get('frames_bad'),
            'reconnects': None if diag is None else diag.get('reconnects'),
            'port': None if diag is None else diag.get('port'),
        }
        return payload

    def _read_view(self, live, next_msg_t, already_refused=False):
        """读浏览器发来的控制消息，返回（是否要实时数据, 下次超时时刻）。

        浏览器在控制页每 ``VIEW_BEAT_S`` 秒发一次 ``{"t":"view","page":"control"}``
        （弱网丢一次也不影响），离开时发 ``{"t":"view","page":"idle"}``。
        超过 CLIENT_TIMEOUT 没收到任何消息才按「不要实时数据」处理，避免
        页面崩溃后还一直推图像。
        """
        now = time.time()
        buf = getattr(self, '_ws_buf', None)
        if buf is None:
            buf = self._ws_buf = bytearray()
        # 用 select 探测可读，绝不把套接字改成非阻塞或加读超时。
        # 两个坑都踩过：
        # 1) settimeout(0.0) 会让套接字一直非阻塞，随后 70 KB 的图像帧
        #    sendall 只能写一部分，抛 BlockingIOError，被当成对端断开；
        # 2) 给 rfile（BufferedReader）设读超时后，超时一次它就作废，
        #    后到的 view 消息再也读不出来。
        # 因此这里只用 select 判断，再用原始 recv 收字节，自己解析帧。
        for _ in range(_WS_READS_PER_TICK):
            # 每次接收前都检查，避免等待尚未到达的下一帧。
            try:
                ready, _, _ = select.select([self.connection], [], [], 0)
            except (OSError, ValueError, TypeError):
                ready = True          # 测试替身没有 fileno，直接读
            if not ready:
                break
            try:
                chunk = b''
                if not getattr(self, '_ws_drained', False):
                    # 兜底：握手时已经取过一次 rfile，这里再取一次，
                    # 防止别的分支把控制帧读进缓冲。只取一次。
                    self._ws_drained = True
                    chunk = _take_buffered(self)
                if not chunk:
                    chunk = self.connection.recv(65536)
            except (BlockingIOError, TimeoutError):
                chunk = b''
            except OSError:
                raise ConnectionResetError('socket closed')
            if not chunk:
                break
            buf.extend(chunk)
        msgs = []
        if not buf and not ready:
            # 没有可读字节就直接返回：日志只记真实到达的帧，
            # 否则每秒 50 次空转会把环形日志刷满。
            return live, next_msg_t, already_refused
        for data in _decode_ws_frames(buf):
            try:
                msg = json.loads(data.decode('utf-8'))
            except (ValueError, UnicodeDecodeError):
                continue
            if isinstance(msg, dict):
                msgs.append(msg)
        if msgs:
            with _lock:
                _ws_last_frame_t[0] = time.monotonic()
                _ws_rx_count[0] += len(msgs)
            for msg in msgs:
                kind = msg.get('t')
                # 只记类型与非空字段名，不落密码：日志要能贴进工单。
                if kind == 'manual_login':
                    _ws_log_add('rx', {'t': kind,
                                       'fields': sorted(msg.keys())})
                else:
                    _ws_log_add('rx', msg)
        # 先处理 view，再处理手动消息：同一批里"进入控制页 + 心跳"
        # 必须按这个顺序生效，否则第一条 manual_start 会因 live 还是 False 被拒。
        for msg in msgs:
            if msg.get('t') != 'view':
                continue
            # 2026-11-15 边界重定："看"与"开"分离。地图页只读、免登录
            # （不登记 _live_controls，栅格/点云由视频连接直接推）；控制页
            # 未登录仍提示一次。live 保持"在实时页"语义（切到地图页不打断
            # 手动会话）；驾驶鉴权由 _manual_message 复查，不依赖本标志。
            page = msg.get('page')
            self._ws_page = page
            requested_live = page in ('control', 'map')
            if page == 'control' and not _MANUAL.authenticated(self):
                # 心跳每 1 秒一次；连续请求都没登录时不必每条都回错误，
                # 回一次就够，避免把连接刷满。但进入控制页必须提示。
                if live or not already_refused:
                    already_refused = True
                    _send_text(self, json.dumps(
                        {'t': 'manual_error', 'message': '请先登录手动控制'}))
                live = False
                continue
            live = requested_live
            refused = False        # 明确要了 idle 就复位提示
            if not live:
                _MANUAL.disconnect(self)
            next_msg_t = time.time() + CLIENT_TIMEOUT
            # 本批先处理了 view：按新的页面与 live 保存（之前这里还是上一批
            # 的 None/idle，会先按非实时页拒掉同一批里的驾驶消息）。
            self._ws_page = page
        for msg in msgs:
            if msg.get('t') != 'view':
                self._manual_message(msg, live)
        # 登记当前"已登录且在控制页"的连接：视频连接据此推摄像头。
        # 地图页虽是实时页（live=True）但不登记——它不显示摄像头画面。
        if live and getattr(self, '_ws_page', None) == 'control':
            _live_controls.add(self)
        else:
            _live_controls.discard(self)
        had_session = self in _live_sessions
        if live:
            patrol_session_live()
            with _live_sessions_lock:
                _live_sessions.add(self)
        else:
            with _live_sessions_lock:
                _live_sessions.discard(self)
        if live and time.time() > next_msg_t:
            # 控制消息停了（页面崩溃/挂起）：已登录会话按原语义断开；
            # 地图页（免登录）也一起退出实时会话——巡航随后按会话失效停。
            live = False
            _live_controls.discard(self)
            _MANUAL.disconnect(self)
            with _live_sessions_lock:
                _live_sessions.discard(self)
            if had_session:
                patrol_session_closed(self)
        return live, next_msg_t, already_refused

    def _manual_message(self, msg, live):
        kind = msg.get('t')
        if kind == 'ping':
            # RTT 探测：原样回显时间戳，客户端自己算往返延迟。不鉴权、
            # 不进 ws 日志（5s 一次，进日志全是噪音）。
            _send_text(self, json.dumps({'t': 'pong', 'ts': msg.get('ts')}))
            return
        if kind == 'motion_license':
            # 网页运动许可：每 100 ms 一条。**不要求登录**——许可只决定
            # "下游是否认为网页还活着"，不代表驾驶授权；驾驶消息另有鉴权。
            # 网页进程退出 → 不再有许可 → 服务端 350 ms 判失效并锁停，
            # 底盘桥再按自己的许可超时独立零速（不依赖本进程）。
            for text in _session_msg_text(patrol_license_message(msg)):
                _send_text(self, text)
            return
        if kind == 'manual_login':
            ok = _MANUAL.login(self, msg.get('username'), msg.get('password'))
            out = json.dumps({'t': 'manual_auth', 'ok': ok})
            _ws_log_add('tx', out)
            _send_text(self, out)
        elif kind in ('manual_start', 'manual_heartbeat', 'manual_stop'):
            try:
                if not _MANUAL.authenticated(self):
                    raise PermissionError('请先登录手动控制')
                if kind == 'manual_start':
                    if not live:
                        raise RuntimeError('请先进入手动控制页面')
                    # [巡航] 手动接管：先取消旧导航，松键后旧任务不自行恢复。
                    # 由 ManualForwardControl 的接管钩子执行（见
                    # _manual_takeover_hook）：此刻本连接还没拿到手动控制，
                    # 在这里判"是否在按手动键"永远为假。
                    _MANUAL.start(self, msg.get('speed'), msg.get('mode'), msg.get('duration'),
                                  direction=msg.get('direction', 'forward'))
                    _send_text(self, json.dumps({'t': 'manual_started', 'id': msg.get('id')}))
                elif kind == 'manual_heartbeat':
                    _MANUAL.heartbeat(self)
                else:
                    _MANUAL.stop(self)
            except (PermissionError, RuntimeError, ValueError) as exc:
                _send_text(self, json.dumps({'t': 'manual_error', 'message': str(exc)}))
        elif kind == 'cruise_set':
            # 巡航开关：只发开关，不发速度。必须在控制页，避免后台误开。
            try:
                if not _MANUAL.authenticated(self):
                    raise PermissionError('请先登录手动控制')
                if not live:
                    raise RuntimeError('请先进入手动控制页面')
                if not isinstance(msg.get('on'), bool):
                    raise ValueError('巡航开关缺少 on 布尔字段')
                set_cruise(msg.get('on'))
            except (PermissionError, RuntimeError, ValueError) as exc:
                _send_text(self, json.dumps({'t': 'manual_error', 'message': str(exc)}))
        elif kind == 'ultrasonic_rearm':
            # [2026-11-17] 超声 fail-closed 后唯一的恢复入口：鉴权同巡航。
            try:
                _require_drive_auth(self, live)
                ok, info = ultrasonic_rearm()
            except (PermissionError, RuntimeError, ValueError) as exc:
                ok, info = False, str(exc)
            _send_text(self, json.dumps({'t': 'manual_error' if not ok
                                         else 'ultrasonic_status',
                                         'ok': ok, 'message': info}))
        elif kind in ('patrol_save', 'patrol_start', 'motion_stop',
                      'motion_rearm'):
            # [巡航] 保存路线 / 开始巡航 / 全局停止 / 恢复。鉴权与 cruise_set
            # 完全相同：已登录 + 在实时页（地图页免登录可看，不能动）。
            try:
                _require_drive_auth(self, live)
                if kind == 'patrol_save':
                    ok, info, frame = patrol_save(
                        msg.get('points'), msg.get('map_id'))
                elif kind == 'patrol_start':
                    ok, info = patrol_start()
                elif kind == 'motion_stop':
                    ok, info = True, '已全局停止'
                    patrol_stop('global_stop')
                else:
                    ok, info = patrol_rearm()
                frame = patrol_status_frame()
                frame['t'] = 'patrol_status'
                frame['ok'] = ok
                frame['message'] = info if isinstance(info, str) else ''
                _ws_log_add('tx', {'t': 'patrol_status', 'ok': ok})
                _send_text(self, json.dumps(frame, allow_nan=False))
            except (PermissionError, RuntimeError, ValueError) as exc:
                _send_text(self, json.dumps({'t': 'manual_error',
                                             'message': str(exc)}))
        elif kind in ('nav_goal', 'nav_cancel'):
            # [C50C 阶段 D 2026-11-16] 网页点图导航：发 /navigate_to_pose
            # 或取消。鉴权同 cruise_set——导航是驾驶行为，必须登录且在
            # 实时页（地图页免登录可看，但不能发目标）。
            try:
                if not _MANUAL.authenticated(self):
                    raise PermissionError('请先登录手动控制')
                if not live:
                    raise RuntimeError('请先进入实时页面再导航')
                if kind == 'nav_cancel':
                    ok, view = NG.cancel_nav()
                    out = json.dumps({'t': 'nav_status', **view})
                    _ws_log_add('tx', out)
                    _send_text(self, out)
                    if not ok:
                        _send_text(self, json.dumps(
                            {'t': 'manual_error', 'message': view.get('error', '')}))
                else:
                    x, y = msg.get('x'), msg.get('y')
                    th = msg.get('th')
                    ok, view = NG.request_nav_goal(x, y, th)
                    out = json.dumps({'t': 'nav_status', **view})
                    _ws_log_add('tx', out)
                    _send_text(self, out)
                    if not ok:
                        _send_text(self, json.dumps(
                            {'t': 'manual_error',
                             'message': view.get('error', '目标被拒绝')}))
            except (PermissionError, RuntimeError, ValueError) as exc:
                _send_text(self, json.dumps({'t': 'manual_error', 'message': str(exc)}))


def _require_drive_auth(handler, live):
    """驾驶类消息的统一鉴权（cruise_set / patrol_* 共用）。

    已登录 + 在实时页（控制页或地图页）。地图页免登录只读，但发目标、
    保存路线、开始巡航都是驾驶行为，必须登录。
    """
    if not _MANUAL.authenticated(handler):
        raise PermissionError('请先登录手动控制')
    if not live:
        raise RuntimeError('请先进入实时页面（控制页或地图页）')


def _take_buffered(handler):
    """取出 rfile 里已经缓冲的原始字节（没有就返回 b''）。

    http.server 握手时用 rfile 读请求头；如果浏览器在同一包里紧接着发了
    控制帧，那些字节已经在缓冲里。只取一次：之后全靠 recv 收。
    """
    rfile = getattr(handler, 'rfile', None)
    peek = getattr(rfile, 'peek', None)
    if peek is None:
        return b''
    try:
        data = peek(65536)
        if not data:
            return b''
        return bytes(rfile.read(len(data)))
    except (OSError, ValueError):
        return b''


def _decode_ws_frames(buf):
    """从字节缓冲里取出所有完整的文本帧，返回已解掩码的载荷列表。

    从 buf 头部消费掉已解析的字节，剩下的半帧留在缓冲里等下次
    调用。收到关闭帧（opcode 0x8）时抛 ConnectionResetError。
    """
    out = []
    while len(buf) >= 2:
        opcode = buf[0] & 0x0F
        masked = buf[1] & 0x80
        n = buf[1] & 0x7F
        pos = 2
        if n == 126:
            if len(buf) < 4:
                break
            n = int.from_bytes(bytes(buf[2:4]), 'big')
            pos = 4
        elif n == 127:
            if len(buf) < 10:
                break
            n = int.from_bytes(bytes(buf[2:10]), 'big')
            pos = 10
        mlen = 4 if masked else 0
        if len(buf) < pos + mlen + n:
            break
        mask = bytes(buf[pos:pos + 4]) if masked else b'\0\0\0\0'
        raw = bytes(buf[pos + mlen:pos + mlen + n])
        del buf[:pos + mlen + n]
        if opcode == 0x8:                      # 客户端关闭
            raise ConnectionResetError('client closed')
        if opcode != 0x1:
            continue
        out.append(bytes(b ^ mask[i % 4] for i, b in enumerate(raw)))
    return out


def _send_text(handler, text):
    """按 RFC 6455 发送文本帧。

    整帧（帧头 + 载荷）必须一次写进套接字：先写帧头再写载荷时，
    两次写之间可能被其它帧插入，浏览器收到的是拼接后的乱序字节流，
    会报 “Could not decode a text frame as UTF-8” 并断开连接。
    所以这里一律用 connection.sendall 一次发出，并加锁串行化。
    """
    data = text.encode('utf-8')
    head = bytearray([0x81])
    n = len(data)
    if n < 126:
        head.append(n)
    elif n < 65536:
        head.append(126)
        head += n.to_bytes(2, 'big')
    else:
        head.append(127)
        head += n.to_bytes(8, 'big')
    frame = bytes(head) + data
    lock = getattr(handler, '_ws_lock', None)
    if lock is None:
        handler.connection.sendall(frame)
        return
    with lock:
        handler.connection.sendall(frame)


def _send_display(handler, text, timeout_s=DISPLAY_SEND_TIMEOUT_S):
    """有界发送一帧显示数据（状态/图像/点云）。

    先用 select 限时等待套接字可写：超时未可写就**丢弃本帧**并返回
    False——显示数据可丢，发送队列不允许无限期占住连接（任务书 T4）。
    可写时仍用整帧 sendall（与 _send_text 相同的原子性保证）。
    控制回执不走这里：登录/手动确认必须送达，仍走阻塞的 _send_text。
    """
    sock = getattr(handler, 'connection', None)
    if sock is None:
        return False
    try:
        _, writable, _ = select.select([], [sock], [], timeout_s)
    except (OSError, ValueError, TypeError):
        # 测试替身没有 fileno，或套接字已关闭：按可写处理，由 sendall
        # 的异常路径兜底。
        writable = [sock]
    if not writable:
        return False
    _send_text(handler, text)
    return True


def _map_frame_json(grid, grid_t, pose, pose_t):
    """占据栅格 → 视频连接推送帧（地图页用，2026-11-15）。

    ``data`` 用 zlib + base64：整幅栅格压缩后只有几 KB~几十 KB，
    1Hz 推送对 Wi-Fi 无压力；浏览器端用 DecompressionStream('deflate')
    原生解压（'deflate' 即 zlib 格式，与 Python zlib.compress 对应）。
    位姿与栅格同帧携带，页面免二次同步。序列化失败返回 None。
    """
    try:
        payload = {
            't': 'map', 'seq': grid['seq'],
            'w': grid['w'], 'h': grid['h'], 'res': grid['res'],
            'ox': grid['ox'], 'oy': grid['oy'],
            'age': None if grid_t is None else round(time.time() - grid_t, 2),
            'data': base64.b64encode(zlib.compress(grid['data'], 6)).decode('ascii'),
            'pose': pose,
            'pose_age': (None if pose_t is None
                         else round(time.time() - pose_t, 2)),
        }
        return json.dumps(payload, separators=(',', ':'), allow_nan=False)
    except (ValueError, TypeError, KeyError, zlib.error):
        return None


def _pose_frame_json(pose, pose_t, grid_t):
    """静态地图模式下单独推实时位姿（5Hz，避免重复发送整幅栅格）。"""
    try:
        payload = {
            't': 'pose',
            'pose': pose,
            'pose_age': (None if pose_t is None
                         else round(time.time() - pose_t, 2)),
            'map_age': None if grid_t is None else round(time.time() - grid_t, 2),
        }
        return json.dumps(payload, separators=(',', ':'), allow_nan=False)
    except (ValueError, TypeError, KeyError):
        return None


def fake_enabled(argv=None):
    """判断是否进入假数据模式。环境变量或 --fake 任一命中即为真。"""
    argv = sys.argv[1:] if argv is None else argv
    if '--fake' in argv:
        return True
    return os.environ.get('OPI_WEB_FAKE', '').strip().lower() in (
        '1', 'true', 'yes', 'on')


def main():
    """启动数据源与 HTTP 服务。"""
    fake = fake_enabled()
    node = None
    fake_bridge = None
    with _lock:
        _state['mode'] = 'fake' if fake else 'ros'

    if fake:
        fake_bridge = FakeBridge()
        fake_bridge.start()
        print('web_debug fake mode: 合成数据，不订阅 ROS 话题', flush=True)
    else:
        rclpy.init()
        node = Bridge()
        spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin.start()

    port = int(os.environ.get('OPI_WEB_PORT', HTTP_PORT))
    srv = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    srv.daemon_threads = True
    # 跳帧计数：序列化失败时只丢那一帧，不打断 WebSocket 连接。
    srv.stats = {'status_bad': 0, 'scan_bad': 0, 'img_bad': 0}
    print('web_debug on http://0.0.0.0:%d' % port, flush=True)
    try:
        srv.serve_forever()
    except BaseException:
        # 任何异常都必须打印：静默退出会让 HTTP 端口无声消失，现场排障极难。
        import traceback
        traceback.print_exc()
        raise
    finally:
        if fake_bridge is not None:
            fake_bridge.stop()
        if node is not None:
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
