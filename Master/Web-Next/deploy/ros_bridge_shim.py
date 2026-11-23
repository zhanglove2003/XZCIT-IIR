#!/usr/bin/env python3
"""C50C Web-Next 桥接垫片（板端 opi_robot/web_node/bridge_shim.py 的源文件）。

职责边界（契约 ownership）：只做话题 I/O 与消息转换——订阅/QoS、TF 轮询、
字节级转换、七个发布器 + 两个 10Hz 重发循环、NavigateToPose action 客户端、
节点/参数探测、HTTP 客户端与令牌。不做任何许可/仲裁/状态机决策（旧设计
同构：web_debug.py:910-914「下游的规则比本进程更严，只转发本进程已接受的
原文」）。垫片进程退出 → 许可断流 → hub 与下游 guard 双保险锁停。

逐字复刻旧 web_debug.py Bridge（web_debug.py:1134-1256）的订阅与转换：
- QoS 三件套照抄 qos.py:13-70（sensor=BEST_EFFORT/KEEP_LAST/10/VOLATILE、
  status=RELIABLE/KEEP_LAST/10/VOLATILE）+ /map 的 TRANSIENT_LOCAL 覆盖
  （web_debug.py:1159-1170：qos.py 两个默认都是 VOLATILE，漏改则永远收不到
  map_server 的 latched 静态图）；
- on_image/on_scan/on_map/位姿四元数→yaw 仅做 web_debug.py 已验证的字节级
  变换（web_debug.py:1295-1393），不新增业务判断；
- 诊断键名候选匹配是实测沉淀（chassis.py:46-57，含 voltage_mv/battery_mv
  别名、0xFFFF 无回波语义 chassis.py:191-194），不得「清理」；
- 手动命令 0.05s tick / 0.10s 重发周期 / 停止即发零（manual_forward.py:18
  与 web_debug.py:1242；manual_forward.py:159-174,199-210）；
- patrol_status 10Hz 重发 hub 缓存帧（web_debug.py:357-366）；
- navigate_to_pose 目标构造逐字对齐 nav_goal.py:128-150：frame_id='map'、
  stamp=零时间戳（nav_goal.py:157-159 返回 rclpy.time.Time().to_msg()，
  Nav2 据此取最新 TF 变换——不得改用当前时钟，会改变实车已验证的 TF 语义）、
  yaw→quaternion(z=sin(θ/2),w=cos(θ/2))、wait_for_server 2.0s；取消=句柄
  cancel_goal_async，取消应答失败回滚 active（nav_goal.py:366-384）；
- 探测：3s 图 API + 2s 参数服务（web_debug.py:1497-1597），SAFETY_NODE=
  'c50c_uart3_node'（web_debug.py:1011）。

部署（deploy/c50c-web-bridge.service）：systemctl --user 用户单元，进程
归属 orangepi（unit 内不得写 User=，dds-watchdog.service:10-12 注释）；
ExecStart 必须经 bash -lc source /opt/ros/humble/setup.bash 与
/home/orangepi/ros2_ws/install/setup.bash（c50c-web.service ExecStart 同款，
否则 systemctl --user 无登录 shell、不继承 AMENT_PREFIX_PATH/PYTHONPATH，
裸 python3 会在 import rclpy 处直接死）；禁设任何 DDS domain 变量
（dds-watchdog.service:16 注释 C1）；linger 由 loginctl enable-linger
orangepi 提供（dds-watchdog.service:13-15，非 unit 文件能力）。

python3.10 stdlib + rclpy，无第三方依赖（无 websockets/aiohttp/numpy）。
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import pickle
import socket
import subprocess
import sys
import threading
import time
import zlib
from collections import deque
from http.client import HTTPConnection
from urllib.parse import urlparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import CompressedImage, Imu, LaserScan
from std_msgs.msg import Bool, String as RosString

try:  # /map 属 nav_msgs（web_debug.py:1169 同款）；缺包时跳过订阅，地图照常显示旧栅格
    from nav_msgs.msg import OccupancyGrid
except ImportError:  # pragma: no cover - 坏环境兜底
    OccupancyGrid = None

try:  # moving_bins 可选：类型不可用时跳过订阅，点云照常（web_debug.py:1154）
    from sensor_msgs.msg import UInt8MultiArray
except ImportError:  # pragma: no cover - 坏环境兜底
    UInt8MultiArray = None

try:  # TF 可选：缺 tf2_ros 时位姿帧不发，hub 保旧值由 pose_age 判新鲜
    import tf2_ros
except ImportError:  # pragma: no cover
    tf2_ros = None

try:  # 参数服务探测
    from rcl_interfaces.srv import GetParameters, ListParameters
except ImportError:  # pragma: no cover
    GetParameters = ListParameters = None

try:  # 点选导航/巡航派发
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover
    NavigateToPose = None

try:  # AMCL 无运动刷新（发布 /initialpose 后触发粒子重算，web_debug.py:88-90）
    from std_srvs.srv import Empty as EmptySrv
except ImportError:  # pragma: no cover
    EmptySrv = None

try:  # 定位校准后清代价地图（清掉 AMCL 漂移期墙体残影，web_debug.py:94-97）
    from nav2_msgs.srv import ClearEntireCostmap as ClearCostmapSrv
except ImportError:  # pragma: no cover
    ClearCostmapSrv = None

try:  # numpy 全局重定位（旧栈 web_node/relocalize.py 纯 numpy 实现，零 scipy）；
    # 垫片与 relocalize.py 同目录部署，缺 numpy 时降级（回执报错，不假装成功）
    import relocalize as RLOC
except ImportError:  # pragma: no cover
    RLOC = None

# ---------------------------------------------------------------------------
# QoS（照抄 opi_robot/qos.py:16-61，不手写第二套）
# ---------------------------------------------------------------------------

SENSOR_QOS_SPEC = {
    'reliability': 'BEST_EFFORT',
    'history': 'KEEP_LAST',
    'depth': 10,
    'durability': 'VOLATILE',
}
STATUS_RELIABILITY = 'RELIABLE'
STATUS_QOS_DEPTH = 10


def sensor_qos(depth=None) -> QoSProfile:
    q = QoSProfile(depth=SENSOR_QOS_SPEC['depth'] if depth is None else int(depth))
    q.reliability = getattr(ReliabilityPolicy, SENSOR_QOS_SPEC['reliability'])
    q.history = getattr(HistoryPolicy, SENSOR_QOS_SPEC['history'])
    q.durability = getattr(DurabilityPolicy, SENSOR_QOS_SPEC['durability'])
    return q


def status_qos(depth=None) -> QoSProfile:
    q = QoSProfile(depth=STATUS_QOS_DEPTH if depth is None else int(depth))
    q.reliability = getattr(ReliabilityPolicy, STATUS_RELIABILITY)
    q.history = HistoryPolicy.KEEP_LAST
    q.durability = DurabilityPolicy.VOLATILE
    return q


def map_qos() -> QoSProfile:
    """status_qos(1) + TRANSIENT_LOCAL（web_debug.py:1159-1170 硬前提）。"""
    profile = status_qos(1)
    try:
        profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
    except Exception:  # pragma: no cover
        pass
    return profile


# ---------------------------------------------------------------------------
# 诊断解析（chassis.py / sensors.py 的逐字移植）
# ---------------------------------------------------------------------------

# 诊断里可能使用的底盘状态名。任一命中即视为底盘状态源（chassis.py:46）。
CHASSIS_NAMES = ('chassis', 'uart3', 'c50c_uart3', 'c50c')

# 数值字段 -> 诊断 KeyValue 键名候选（chassis.py:49-57 实测沉淀）。
_KEYS = {
    'voltage_mv': ('voltage_mv', 'voltage', 'battery_mv'),
    'vx_mm_s': ('vx_mm_s', 'velocity_x', 'chassis_velocity_x'),
    'vy_mm_s': ('vy_mm_s', 'velocity_y', 'chassis_velocity_y'),
    'yaw_rate_mrad_s': ('yaw_rate_mrad_s', 'yaw_rate', 'chassis_yaw_rate'),
    'stop_flag': ('stop_flag',),
    # [C50C 2026-11-15] 前置 HC-SR04 超声波距离（mm；0xFFFF=无回波/陈旧）
    'ultrasonic_front_mm': ('ultrasonic_front_mm',),
}
_INT_FIELDS = ('voltage_mv', 'stop_flag', 'vx_mm_s', 'vy_mm_s',
               'yaw_rate_mrad_s', 'ultrasonic_front_mm')


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


def parse_chassis_status(msg):
    """从 DiagnosticArray 取底盘状态，返回字典或 None（chassis.py:62-90）。

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
                    out[field] = (_to_int(values[key]) if field in _INT_FIELDS
                                  else _to_float(values[key]))
                    break
        out['level'] = getattr(status, 'level', None)
        out['message'] = getattr(status, 'message', '') or ''
        return out
    return None


def parse_lidar_diag(msg):
    """从 DiagnosticArray 取 lidar 状态，返回字典或 None（sensors.py:39-63）。"""
    if msg is None:
        return None
    for status in getattr(msg, 'status', []) or []:
        if getattr(status, 'name', '') != 'lidar':
            continue
        values = {}
        for kv in getattr(status, 'values', []) or []:
            values[kv.key] = kv.value
        return {
            # 真实消息的 level 是单字节 bytes；统一解码成整数，无法识别给 None
            'level': _decode_level(getattr(status, 'level', None)),
            'message': getattr(status, 'message', '') or '',
            'state': values.get('state', 'unknown'),
            'reason': values.get('reason', ''),
            'age_s': _to_float(values.get('age_s')),
            'coverage': _to_float(values.get('coverage')),
            'reconnects': _to_int(values.get('reconnects')),
            'port': values.get('port', ''),
            'frames_ok': _to_int(values.get('frames_ok')),
            'frames_bad': _to_int(values.get('frames_bad')),
        }
    return None


def _decode_level(level):
    """DiagnosticStatus.level 单字节/bytes → 整数（sensors.py:50-52 语义）。"""
    if level is None:
        return None
    if isinstance(level, (bytes, bytearray)):
        return level[0] if len(level) else None
    try:
        return int(level)
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------------------
# 图像嗅探（web_debug.py:2104-2135 _sniff_mime/_image_size/_jpeg_size）
# ---------------------------------------------------------------------------

def sniff_mime(data: bytes) -> str:
    if data[:8] == b'\x89PNG\r\n\x1a\n':
        return 'image/png'
    return 'image/jpeg'


def image_size(data: bytes):
    """按魔数读宽高，JPEG 与 PNG 都支持；读不出如实置 null。"""
    try:
        if data[:8] == b'\x89PNG\r\n\x1a\n' and len(data) >= 24:
            w = int.from_bytes(data[16:20], 'big')
            h = int.from_bytes(data[20:24], 'big')
            return w, h
        return _jpeg_size(data)
    except Exception:
        return None, None


def _jpeg_size(data: bytes):
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
    return None, None


# ---------------------------------------------------------------------------
# 常数
# ---------------------------------------------------------------------------

SAFETY_NODE = 'c50c_uart3_node'          # web_debug.py:1011
SAFETY_PARAMS = ('drive_enable', 'drive_input_enable',
                 'drive_firmware_protocol')
CRUISE_TOPIC = 'cruise_enable'           # web_debug.py:138
LICENSE_TOPIC = 'motion_license'         # web_debug.py:868-873
AUTHORIZE_TOPIC = 'guard_authorize'      # web_debug.py:874-881
REARM_TOPIC = 'rearm_request'            # web_debug.py:175
ULTRASONIC_REARM_TOPIC = 'ultrasonic_rearm'  # web_debug.py:1237-1238
PATROL_STATUS_TOPIC = 'patrol_status'    # web_debug.py:1210-1211
INITIALPOSE_TOPIC = 'initialpose'        # relocalize 出口（AMCL 订阅）
NOMOTION_SERVICE = 'request_nomotion_update'
CLEAR_GLOBAL_COSTMAP_SERVICE = '/global_costmap/clear_entirely_global_costmap'
CLEAR_LOCAL_COSTMAP_SERVICE = '/local_costmap/clear_entirely_local_costmap'

MANUAL_TICK_S = 0.05                     # manual_forward.py web_debug.py:1242
MANUAL_PUBLISH_PERIOD_S = 0.10           # manual_forward.py:18
PATROL_RESEND_S = 0.10                   # web_debug.py:357-366（10Hz 重发）
BATCH_FRAMES = 32                        # 契约 endpointContract①
BATCH_WINDOW_S = 0.2                     # 200ms 或满 32 帧成批
IMG_PASSTHROUGH_BYTES = 64 * 1024        # img 大帧直通不攒批
RECONNECT_BACKOFF_S = 1.0                # 失败重连退避（契约 shimOutline⑤）
PROBE_NODES_S = 3.0                      # probe 周期必须小于 hub 10s 活性窗口
PROBE_SAFETY_S = 2.0

# 看门狗（HANDOVER 坑 16/19）：
# - A（publish 上下文坏死，坑 19）：rclpy context 不可重建，只能重启进程；
#   systemd 单元 Restart=always 会 3s 拉起。滑窗统计 RCLError("context is
#   invalid") 类异常，持续成簇（≈10Hz 心跳全失败）才自杀，杜绝瞬时误杀。
PUB_FAIL_WINDOW_S = 30.0
PUB_FAIL_LIMIT = 20           # 30s 窗口内 ≥20 次（10Hz 下约 2s 即触发）
PUB_CONTEXT_FAIL_MARKERS = (
    'context is invalid', 'context is destroyed', 'context is shutdown',
)
# - B（command_thread 轮询卡死，坑 16）：一次轮询最长 wait_ms=20000 + 余量，
#   超 25s 未完成即判定卡死，由 main 线程强制断开连接让 command_thread 抛
#   异常走重连路径（不动上行线程 / 不自杀进程——上行是健康的）。
CMD_POLL_STALL_S = 25.0

# 目标范围限制（nav_goal.py:26-31；hub 侧 ros::request_nav_goal 同款校验）
GOAL_MIN_X, GOAL_MAX_X = -8.0, 22.0
GOAL_MIN_Y, GOAL_MAX_Y = -8.0, 8.0

RESULT_TEXT = {
    4: '到达目标',                    # SUCCEEDED（nav_goal.py:290-295）
    5: '已取消',                      # CANCELED
    6: '导航中止（规划/控制失败）',    # ABORTED
}


def log(msg: str) -> None:
    sys.stderr.write('[bridge_shim %s] %s\n' % (time.strftime('%H:%M:%S'), msg))
    sys.stderr.flush()


def load_token(explicit_file: str | None) -> str:
    """令牌：环境变量优先；文件形态须属主本人、权限 0600（契约 shimOutline⑨：
    root 属主 0600 垫片读不到，会拿到空 token 或启动失败）。只进请求头、
    不进任何日志。"""
    env = os.environ.get('OPI_BRIDGE_TOKEN', '').strip()
    if env:
        return env
    path = explicit_file or os.path.expanduser('~/.config/opi-robot/bridge-token')
    try:
        st = os.stat(path)
        uid = os.getuid()
        if st.st_uid != uid or (st.st_mode & 0o077):
            log('令牌文件 %s 属主/权限不合规（须属主本人且 0600），拒绝读取' % path)
            return ''
        with open(path, encoding='utf-8') as fh:
            token = fh.read().strip()
        if token:
            return token
    except OSError as exc:
        log('令牌文件 %s 不可读：%r' % (path, exc))
    log('警告：OPI_BRIDGE_TOKEN 未设置且令牌文件缺失——Web-Next 将拒绝一切请求（401）')
    return ''


class BridgeShim(Node):
    """订阅/发布/转换/探测一体的垫片节点（节点名 web_bridge_shim，与旧
    web_bridge 区分）。"""

    def __init__(self, server_url: str, token: str):
        super().__init__('web_bridge_shim')
        self._token = token
        parsed = urlparse(server_url if '//' in server_url else 'http://' + server_url)
        self._host = parsed.hostname or '127.0.0.1'
        self._port = parsed.port or 8088
        self._conn_lock = threading.Lock()

        # 上行帧队列：转换回调只入队，HTTP 线程成批发送。
        self._frames = deque(maxlen=1024)
        self._frames_lock = threading.Lock()
        self._connected = False

        # 发布器（QoS 全 status_qos，web_debug.py:1200,1210,1225,1237,868-881）
        self._pub_manual = self.create_publisher(Twist, 'manual_cmd_vel', status_qos())
        self._pub_cruise = self.create_publisher(Bool, CRUISE_TOPIC, status_qos())
        self._pub_license = self.create_publisher(RosString, LICENSE_TOPIC, status_qos())
        self._pub_authorize = self.create_publisher(Bool, AUTHORIZE_TOPIC, status_qos())
        self._pub_rearm = self.create_publisher(Bool, REARM_TOPIC, status_qos())
        self._pub_us_rearm = self.create_publisher(Bool, ULTRASONIC_REARM_TOPIC, status_qos())
        self._pub_patrol = self.create_publisher(RosString, PATROL_STATUS_TOPIC, status_qos())
        # 重定位出口：AMCL 订阅 /initialpose（Publisher=0 等待喂初始位姿）
        self._pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, INITIALPOSE_TOPIC, status_qos(1))
        self._cli_nomotion = None
        if EmptySrv is not None:
            try:
                self._cli_nomotion = self.create_client(EmptySrv, NOMOTION_SERVICE)
            except Exception:  # pragma: no cover
                self._cli_nomotion = None
        self._cli_clear_global_cm = None
        self._cli_clear_local_cm = None
        if ClearCostmapSrv is not None:
            try:
                self._cli_clear_global_cm = self.create_client(
                    ClearCostmapSrv, CLEAR_GLOBAL_COSTMAP_SERVICE)
                self._cli_clear_local_cm = self.create_client(
                    ClearCostmapSrv, CLEAR_LOCAL_COSTMAP_SERVICE)
            except Exception:  # pragma: no cover
                self._cli_clear_global_cm = None
                self._cli_clear_local_cm = None

        # 手动命令重发状态（manual_forward.py 语义：target/active 由命令流
        # 设置，0.05s tick、0.10s 周期重发，停止即发零）
        self._manual_lock = threading.Lock()
        self._manual_active = False
        self._manual_cmd = (0.0, 0.0)
        self._manual_last_pub = 0.0
        self.create_timer(MANUAL_TICK_S, self._manual_tick)

        # patrol_status 10Hz 重发缓存帧（web_debug.py:357-366）
        self._patrol_lock = threading.Lock()
        self._patrol_frame = None
        self.create_timer(PATROL_RESEND_S, self._patrol_tick)

        # 订阅清单 10 项 = frameMap 各行（moving_bins 可选）
        self.create_subscription(CompressedImage, 'image_raw/compressed',
                                 self.on_image, sensor_qos())
        self.create_subscription(LaserScan, 'scan', self.on_scan, sensor_qos())
        self.create_subscription(DiagnosticArray, 'diagnostics',
                                 self.on_diagnostics, status_qos())
        self.create_subscription(Imu, 'imu/data_raw', self.on_imu, sensor_qos())
        self._moving_ok = False
        if UInt8MultiArray is not None:
            try:
                self.create_subscription(UInt8MultiArray, 'robot/moving_bins',
                                         self.on_moving, status_qos())
                self._moving_ok = True
            except Exception as exc:  # pragma: no cover
                log('moving_bins 订阅建失败，点云照常：%r' % (exc,))
        if OccupancyGrid is not None:
            try:
                self.create_subscription(OccupancyGrid, 'map', self.on_map, map_qos())
            except Exception as exc:  # pragma: no cover
                log('map 订阅建失败，地图显示旧栅格：%r' % (exc,))
        for which, topic in (('nav', 'nav_cmd_vel'), ('demo', 'demo_cmd_vel'),
                             ('manual', 'manual_cmd_vel'), ('auto', 'auto_cmd_vel')):
            self.create_subscription(
                Twist, topic, lambda m, w=which: self.on_cmd(w, m), sensor_qos())
        self.create_subscription(RosString, 'drive_safety', self.on_drive_safety,
                                 status_qos())

        # TF：map→base_link 0.2s 定时（web_debug.py:1175,1371-1393）
        self._tf_buffer = None
        if tf2_ros is not None:
            try:
                self._tf_buffer = tf2_ros.Buffer()
                tf2_ros.TransformListener(self._tf_buffer, self)
                self.create_timer(0.2, self._pose_tick)
            except Exception as exc:  # pragma: no cover - 坏环境兜底
                log('TF 初始化失败，位姿帧不发：%r' % (exc,))
                self._tf_buffer = None

        # 雷达/相机/IMU 实测计数（web_debug.py:1281-1292,1316-1346 同构）
        self._img_n = 0
        self._img_t0 = time.time()
        self._scan_n = 0
        self._scan_t0 = time.time()
        self._scan_last_emit = 0.0  # 100Hz 源节流到 20Hz：点云桶驻留渲染用不到全速率
        self._imu_n = 0
        self._imu_t0 = time.time()
        self._imu_hz = None
        self._map_seq = 0
        # numpy 全局重定位（relocalize.py）输入缓存：最新栅格 + 最近一批
        # 有效 scan（on_map/on_scan 各自喂；锁保护，匹配在独立线程跑）
        self._reloc_lock = threading.Lock()
        self._map_grid = None
        self._map_frame = None
        self._scan_pts = None
        self._scan_pts_t = 0.0
        # 重定位器（懒建；numpy 缺失时保持 None，降级回当前位姿重播种）
        self._relocalizer = None
        self._reloc_busy = False   # numpy 匹配器防重入（前端 relocal_busy 双保险）

        # 探测：3s 图 API + 2s 参数服务（web_debug.py:1497-1597 精简版）
        self._nodes_seen = {'camera_node': False, 'lidar_node': False,
                            'chassis_status_node': False}
        self._safety_values = {name: None for name in SAFETY_PARAMS}
        self._safety_online = False
        self._safety_source = '尚未探测'
        self._safety_seen_t = None
        self._chassis_t = None             # /diagnostics 底盘帧最近一次解析成功时刻
        self._probe_busy = False
        self._probe_t = 0.0                # 本次参数调用发起时刻（看门狗用）
        if GetParameters is not None:
            self._list_cli = self.create_client(
                ListParameters, '/%s/list_parameters' % SAFETY_NODE)
            self._get_cli = self.create_client(
                GetParameters, '/%s/get_parameters' % SAFETY_NODE)
            self.create_timer(PROBE_SAFETY_S, self.probe_safety)
        else:
            self._list_cli = None
            self._get_cli = None
        self.create_timer(PROBE_NODES_S, self.probe_nodes)

        # navigate_to_pose action 客户端（nav_goal.py:68-81 挂节点方式）
        self._nav_client = None
        self._nav_ok = False
        if NavigateToPose is not None:
            try:
                self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
                self._nav_ok = True
            except Exception as exc:  # pragma: no cover
                log('NavigateToPose 客户端建失败：%r' % (exc,))
        # rid → 目标状态（发出去未接受 / 已接受未结束）
        self._nav_lock = threading.Lock()
        self._nav_goals = {}   # rid -> {'handle': handle|None, 'parked': bool,
                               #            'canceled': bool, 'active': bool}
        self._nav_seq = 0

        # 看门狗状态（HANDOVER 坑 16/19）：
        # - A：publish context 坏死失败时间戳滑窗（health_lock 保护）
        # - B：command_thread 最近一次长轮询发起时刻 + 下行连接引用
        #   （cmd_lock 保护；main 线程读、command_thread 写）
        self._health_lock = threading.Lock()
        self._pub_fail_times = deque(maxlen=64)
        self._cmd_lock = threading.Lock()
        self._cmd_conn = None
        self._cmd_poll_t = None

    # ---- 帧入队 ----
    def emit(self, frame: dict, direct: bool = False) -> None:
        with self._frames_lock:
            self._frames.append((direct, frame))

    def hello_frame(self) -> dict:
        return {
            't': 'bridge_hello',
            'topics': {
                'subscriptions': ['image_raw/compressed', 'scan', 'diagnostics',
                                  'imu/data_raw', 'map', 'drive_safety']
                                + (['robot/moving_bins'] if self._moving_ok else []),
                'publishers': ['manual_cmd_vel', CRUISE_TOPIC, LICENSE_TOPIC,
                               AUTHORIZE_TOPIC, REARM_TOPIC,
                               ULTRASONIC_REARM_TOPIC, PATROL_STATUS_TOPIC,
                               INITIALPOSE_TOPIC],
                'moving_bins': self._moving_ok,
                'tf': self._tf_buffer is not None,
                'nav_action': self._nav_ok,
            },
        }

    # ---- 订阅回调：仅做 web_debug.py 已验证的字节级变换 ----
    def on_image(self, msg) -> None:
        """原始编码字节 base64，不重编码；魔数嗅探 mime 与宽高
        （on_image web_debug.py:1295-1314）。"""
        data = bytes(msg.data)
        w, h = image_size(data)
        b64 = base64.b64encode(data).decode('ascii')
        self._img_n += 1
        now = time.time()
        dt = now - self._img_t0
        if dt > 2.0:
            self._img_n = 0
            self._img_t0 = now
        # 大帧直通不攒批（契约 shimOutline⑤）
        self.emit({'t': 'img', 'd': b64, 'w': w, 'h': h,
                   'mime': sniff_mime(data)}, direct=len(data) > IMG_PASSTHROUGH_BYTES)

    def on_scan(self, msg) -> None:
        """deg=round(degrees(angle_min+inc*i),2)，nan/inf/超程点记 0.0 保证
        JSON 合法，有效点 round(r,3)（on_scan web_debug.py:1316-1346）。
        源实测 ~100Hz，全速率逐点转换会吃满半个核，20Hz 足够桶驻留渲染。"""
        now0 = time.time()
        if now0 - self._scan_last_emit < 0.05:
            return
        self._scan_last_emit = now0
        ranges = list(msg.ranges)
        n = len(ranges)
        rmin = float(msg.range_min)
        rmax = float(msg.range_max)
        inc = float(msg.angle_increment)
        a0 = float(msg.angle_min)
        pts = []
        valid = 0
        for i in range(n):
            r = ranges[i]
            deg = round(math.degrees(a0 + inc * i), 2)
            if not (r == r) or r < rmin or r > rmax:
                pts.append([deg, 0.0])
                continue
            valid += 1
            pts.append([deg, round(float(r), 3)])
        self._scan_n += 1
        now = time.time()
        if now - self._scan_t0 > 2.0:
            self._scan_n = 0
            self._scan_t0 = now
        # numpy 全局重定位缓存：保持最近一批完整 scan（含无效点 0.0，
        # relocalize._extract_scan 会按 MIN_RANGE_M 过滤；锁保护防并发读）
        with self._reloc_lock:
            self._scan_pts = pts
            self._scan_pts_t = now
        self.emit({'t': 'scan', 'p': pts, 'n': valid, 'all': n})

    def on_map(self, msg) -> None:
        """int8 -1→255（v&0xFF）其余原值；zlib+base64 与 _map_frame_json
        同编码；seq 到达自增（on_map web_debug.py:1348-1369）。缓存原始栅格
        供 numpy 全局重定位（relocalize.py）用。"""
        info = msg.info
        if not info.width or not info.height:
            return
        data = bytes(v & 0xFF for v in msg.data)
        self._map_seq += 1
        if self._map_seq == 1:
            log('收到首帧 /map：%dx%d res=%.3f（全局重定位栅格缓存就绪）'
                % (int(info.width), int(info.height), float(info.resolution)))
        map_grid = {
            'w': int(info.width),
            'h': int(info.height),
            'res': float(info.resolution),
            'ox': float(info.origin.position.x),
            'oy': float(info.origin.position.y),
            'data': data,
            'seq': self._map_seq,
        }
        map_frame = {
            't': 'map',
            'seq': self._map_seq,
            'w': int(info.width),
            'h': int(info.height),
            'res': float(info.resolution),
            'ox': float(info.origin.position.x),
            'oy': float(info.origin.position.y),
            'd': base64.b64encode(zlib.compress(data, 6)).decode('ascii'),
        }
        with self._reloc_lock:
            self._map_grid = map_grid
            self._map_frame = map_frame
        self.emit(map_frame)

    def cached_map_frame(self) -> dict | None:
        """返回最近一次缓存的 /map 帧副本（web_next 重启重连后立即补发）。"""
        with self._reloc_lock:
            return dict(self._map_frame) if self._map_frame is not None else None

    def _pose_tick(self) -> None:
        """查 map→base_link 位姿（0.2s 一次）。TF 没备好就保持旧值不发帧，
        hub 保旧值由 pose_age 自行判断新鲜度，不显示过期位姿
        （web_debug.py:1371-1393 注释）。yaw=atan2(2(wz+xy),1-2(y²+z²))。"""
        if self._tf_buffer is None:
            return
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return  # lookup 失败不发帧
        q = t.transform.rotation
        th = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.emit({'t': 'pose', 'pose': {
            'x': round(float(t.transform.translation.x), 3),
            'y': round(float(t.transform.translation.y), 3),
            'th': round(th, 4),
        }})

    def on_diagnostics(self, msg) -> None:
        """拆两帧：chassis（parse_chassis_status）+ lidar_diag
        （parse_lidar_diag）。解析失败不发帧 → hub 保持上一次记录不清空
        （web_debug.py:1395-1411）。"""
        chassis = parse_chassis_status(msg)
        if chassis is not None:
            self._chassis_t = time.time()
            self.emit({'t': 'chassis',
                       'voltage_mv': chassis['voltage_mv'],
                       'vx_mm_s': chassis['vx_mm_s'],
                       'vy_mm_s': chassis['vy_mm_s'],
                       'yaw_rate_mrad_s': chassis['yaw_rate_mrad_s'],
                       'stop_flag': chassis['stop_flag'],
                       'ultrasonic_front_mm': chassis['ultrasonic_front_mm'],
                       'level': _decode_level(chassis['level']),
                       'message': chassis['message']})
        diag = parse_lidar_diag(msg)
        if diag is not None:
            self.emit({'t': 'lidar_diag', **diag})

    def on_imu(self, _msg) -> None:
        """只做心跳不读数值：垫片本地计数、1s 窗口实测 hz、1Hz 聚合一帧
        （on_imu web_debug.py:1414-1419 + RateMeter）。"""
        self._imu_n += 1
        now = time.time()
        dt = now - self._imu_t0
        if dt >= 1.0:
            self._imu_hz = round(self._imu_n / dt, 1)
            self._imu_n = 0
            self._imu_t0 = now
            self.emit({'t': 'imu', 'hz': self._imu_hz})

    def on_moving(self, msg) -> None:
        """移动角区（web_debug.py:1421-1426 on_moving）。"""
        self.emit({'t': 'moving', 'bins': [int(b) for b in msg.data]})

    def on_drive_safety(self, msg) -> None:
        """JSON 解析失败记 null（页面显示无许可；on_drive_safety
        web_debug.py:1428-1443）。hub 按 ttl_s（缺省 1s）做新鲜度。"""
        raw = getattr(msg, 'data', None)
        payload = None
        if isinstance(raw, (str, bytes, bytearray)):
            try:
                text = raw.decode('utf-8') if isinstance(raw, (bytes, bytearray)) else raw
                payload = json.loads(text)
            except (TypeError, ValueError, UnicodeDecodeError):
                payload = None
        self.emit({'t': 'drive_safety', 'raw': payload})

    def on_cmd(self, which: str, msg) -> None:
        """_copy_twist web_debug.py:1600-1608；manual 回声是旧 cmd.manual
        卡片的数据源（垫片自己发 manual_cmd_vel 会收到自己发的帧，无害且
        必需，hub 落库接受并存）。"""
        linear = getattr(msg, 'linear', None)
        angular = getattr(msg, 'angular', None)
        self.emit({'t': 'cmd', 'which': which,
                   'vx': float(getattr(linear, 'x', 0.0)) if linear is not None else 0.0,
                   'vy': float(getattr(linear, 'y', 0.0)) if linear is not None else 0.0,
                   'wz': float(getattr(angular, 'z', 0.0)) if angular is not None else 0.0})

    # ---- 探测（web_debug.py:1497-1597 精简版）----
    def probe_nodes(self) -> None:
        """camera/lidar/chassis_status 按 get_publishers_info_by_topic 判
        （web_debug.py:1576-1592）。3s 一帧 → hub 的 10s 桥活性窗口留足
        余量防抖。"""
        seen = {}
        try:
            endpoints = {
                'camera_node': self.get_publishers_info_by_topic('/image_raw/compressed'),
                'lidar_node': self.get_publishers_info_by_topic('/scan'),
                'chassis_status_node': self.get_publishers_info_by_topic('/diagnostics'),
            }
            for node_name, infos in endpoints.items():
                seen[node_name] = any(getattr(i, 'node_name', '') == node_name
                                      for i in infos)
        except Exception:
            seen = {'camera_node': False, 'lidar_node': False,
                    'chassis_status_node': False}
        self._nodes_seen = seen
        # c50c_uart3_node 在线 = 参数服务 6s 内可达（web_debug.py:1591-1592）
        # 由 probe_safety 维护 _safety_seen_t。
        param_alive = (self._safety_seen_t is not None
                       and time.time() - self._safety_seen_t < 6.0)
        # 底盘活性双源：参数服务可达，或 /diagnostics 底盘帧 3s 内有解析成功。
        # 参数调用 future 卡死时（曾实测，探测永久停摆）遥测仍新鲜，不能据此
        # 把有真数据的底盘判成离线；反过来遥测断了参数还在也不该报离线。
        chassis_alive = (self._chassis_t is not None
                         and time.time() - self._chassis_t < 3.0)
        # 本车栈没有名为 chassis_status_node 的节点（/diagnostics 实测 7 个
        # 发布者无它，2026-11-21），底盘遥测实际走 c50c_uart3_node：
        # 底盘节点活性改挂 uart3 参数服务存活，与「底盘数据超时」同源，
        # 不再对着不存在的节点名常挂假离线。
        self._nodes_seen['chassis_status_node'] = bool(param_alive or chassis_alive)
        self.emit({'t': 'probe',
                   'nodes_seen': dict(self._nodes_seen),
                   'safety': {
                       'drive_enable': self._safety_values['drive_enable'],
                       'drive_input_enable': self._safety_values['drive_input_enable'],
                       'drive_firmware_protocol': self._safety_values['drive_firmware_protocol'],
                       'online': bool(param_alive),
                       'source': self._safety_source,
                       'checked_t': time.time(),
                   }})

    def probe_safety(self) -> None:
        """向在线节点读安全参数（web_debug.py:1501-1563）。读不到就把在线
        状态标成离线，不沿用上一次的「在线值」。"""
        if self._list_cli is None:
            return
        if self._probe_busy:
            # 看门狗：服务调用 future 可能永不回调（服务端重启/发现抖动），
            # 卡住会永久停止探测 → 概览常挂「底盘离线」，只能重启垫片。
            if time.time() - self._probe_t < 10.0:
                return
            log('参数探测超时 10s 无回调，重置后重试')
            self._probe_busy = False
        try:
            if not self._list_cli.service_is_ready():
                self._safety_online = False
                self._safety_source = '底盘桥离线，无参数'
                return
            self._probe_busy = True
            self._probe_t = time.time()
            fut = self._list_cli.call_async(ListParameters.Request())
            fut.add_done_callback(self._on_list_params)
        except Exception as exc:
            self._probe_busy = False
            log('参数探测异常：%r' % (exc,))

    def _on_list_params(self, fut) -> None:
        try:
            response = fut.result()
            names = list(getattr(getattr(response, 'result', None), 'names', []) or [])
        except Exception:
            self._probe_busy = False
            self._safety_online = False
            self._safety_source = '参数查询失败'
            return
        wanted = [n for n in names if n in SAFETY_PARAMS]
        if not wanted or self._get_cli is None:
            self._probe_busy = False
            self._safety_online = bool(names) or self._safety_online
            self._safety_source = '节点在线但未声明参数值' if names else self._safety_source
            return
        try:
            req = GetParameters.Request()
            req.names = wanted
            fut = self._get_cli.call_async(req)
        except Exception:
            self._probe_busy = False
            return
        fut.add_done_callback(lambda f: self._on_get_params(f, wanted))

    def _on_get_params(self, fut, wanted) -> None:
        try:
            values = list(fut.result().values)
        except Exception:
            self._probe_busy = False
            self._safety_online = False
            self._safety_source = '参数读取失败'
            return
        for name, param_value in zip(wanted, values):
            kind = getattr(param_value, 'type', 0)
            if kind == 1:               # PARAMETER_BOOL
                self._safety_values[name] = bool(param_value.bool_value)
            elif kind == 4:             # PARAMETER_STRING
                self._safety_values[name] = param_value.string_value
            elif kind == 2:             # PARAMETER_INTEGER
                self._safety_values[name] = int(param_value.integer_value)
        self._safety_seen_t = time.time()
        was_online = self._safety_online
        self._safety_online = True
        self._safety_source = '在线节点 %s' % SAFETY_NODE
        self._probe_busy = False
        if not was_online:
            log('参数探测恢复：/%s 参数服务已可达' % SAFETY_NODE)

    # ---- 下游命令发布 ----
    def _publish_twist(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._pub_manual.publish(msg)

    def _run_relocalize_subprocess(self, grid: dict, scan_pts, timeout_s: float = 30.0) -> dict:
        """在短生命周期子进程执行 relocalize.py，避开 rclpy 四线程回调与 GIL 争用。"""
        reloc_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'relocalize.py')
        if os.path.isfile(reloc_path):
            payload = pickle.dumps((grid, scan_pts), protocol=pickle.HIGHEST_PROTOCOL)
            proc = subprocess.run(
                [sys.executable, reloc_path],
                input=payload,
                capture_output=True,
                timeout=timeout_s,
            )
            out = proc.stdout.decode('utf-8', errors='replace').strip()
            if proc.returncode != 0:
                err_tail = proc.stderr.decode('utf-8', errors='replace').strip()[:200]
                raise RuntimeError('重定位子进程异常退出（rc=%d）：%s' % (proc.returncode, err_tail))
            if not out:
                raise RuntimeError('重定位子进程无输出')
            res = json.loads(out)
            if not res.get('ok'):
                raise ValueError(res.get('error') or '重定位匹配失败')
            return res
        if self._relocalizer is None:
            self._relocalizer = RLOC.ScanMapRelocalizer()
        return self._relocalizer.relocalize(grid, scan_pts)

    def _trigger_post_relocalize_services(self) -> tuple[bool, int]:
        """发布 /initialpose 后刷新 AMCL 粒子并清理 Nav2 代价地图残影。"""
        nomotion_ok = False
        if self._cli_nomotion is not None and EmptySrv is not None:
            try:
                if self._cli_nomotion.service_is_ready():
                    self._cli_nomotion.call_async(EmptySrv.Request())
                    nomotion_ok = True
            except Exception:
                pass
        cleared = 0
        if ClearCostmapSrv is not None:
            for cli in (self._cli_clear_global_cm, self._cli_clear_local_cm):
                if cli is None:
                    continue
                try:
                    if cli.service_is_ready():
                        cli.call_async(ClearCostmapSrv.Request())
                        cleared += 1
                except Exception:
                    pass
        return nomotion_ok, cleared

    def _relocalize_global(self, cmd: dict) -> None:
        """numpy 全局扫描匹配重定位（旧栈 web_debug.py::relocalize_pose 的
        真实现，非「当前位姿重播种」）：用缓存的 /map 栅格 + 最近一次 /scan，
        做 360° 粗搜→±0.25m/±6°精搜→±0.06m/±2°微调三阶段匹配，出
        (x, y, yaw) 与命中率，发布 /initialpose 让 AMCL 置种子并触发
        request_nomotion_update 与代价地图清理，再经 cmd_ack 回执真实结果。"""
        cmd_id = cmd.get('id')
        if self._reloc_busy:
            self._ack_detail(cmd_id, 'relocalize', False,
                             '重定位正在执行中，请稍候', None)
            return
        if RLOC is None:
            self._ack_detail(cmd_id, 'relocalize', False,
                             '重定位模块不可用（缺 numpy/relocalize）', None)
            return
        with self._reloc_lock:
            grid = self._map_grid
            scan_pts = self._scan_pts
            scan_t = self._scan_pts_t
        if not grid or not grid.get('data'):
            self._ack_detail(cmd_id, 'relocalize', False,
                             '尚未收到静态地图，无法重定位', None)
            return
        if not scan_pts or (time.time() - scan_t) > 2.5:
            self._ack_detail(cmd_id, 'relocalize', False,
                             '激光雷达扫描已过期或缺失，无法重定位', None)
            return
        self._reloc_busy = True

        def _run():
            try:
                res = self._run_relocalize_subprocess(grid, scan_pts, timeout_s=30.0)
                # 发布 /initialpose：格式照旧栈 relocalize_pose 744-765
                # （stamp 零时间戳是 AMCL 采纳关键）
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = 'map'
                yaw = float(res['yaw_rad'])
                msg.pose.pose.position.x = float(res['x'])
                msg.pose.pose.position.y = float(res['y'])
                msg.pose.pose.position.z = 0.0
                msg.pose.pose.orientation.x = 0.0
                msg.pose.pose.orientation.y = 0.0
                msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
                msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
                cov = [0.0] * 36
                cov[0] = 0.09
                cov[7] = 0.09
                cov[35] = 0.0685
                msg.pose.covariance = cov
                self._pub_initialpose.publish(msg)
                nomotion_ok, cleared_cm = self._trigger_post_relocalize_services()
                log('relocalize 全局匹配：x=%.3f y=%.3f yaw=%.4f 命中率=%.0f%%'
                    ' LUT=%dms 匹配=%dms nomotion=%s costmap_clear=%d'
                    % (res['x'], res['y'], res['yaw_rad'],
                       res['hit_ratio'] * 100.0, res.get('lut_ms', 0.0),
                       res['match_ms'], nomotion_ok, cleared_cm))
                self._ack_detail(cmd_id, 'relocalize', True, None, {
                    'x': res['x'], 'y': res['y'],
                    'yaw_deg': res['yaw_deg'],
                    'hit_ratio': res['hit_ratio'],
                    'match_ms': res['match_ms'],
                    'valid_beams': res['valid_beams'],
                })
            except subprocess.TimeoutExpired:
                log('relocalize 全局匹配超时（>30s）')
                self._ack_detail(cmd_id, 'relocalize', False,
                                 '全局重定位超时（>30s），请重试', None)
            except Exception as exc:  # noqa: BLE001
                log('relocalize 全局匹配失败：%r' % (exc,))
                self._ack_detail(cmd_id, 'relocalize', False, str(exc), None)
            finally:
                self._reloc_busy = False

        threading.Thread(target=_run, name='relocalize-worker',
                         daemon=True).start()

    def _ack_detail(self, cmd_id, kind, ok, error, detail) -> None:
        """带 detail 的 cmd_ack（relocalize 用它回传匹配结果）。"""
        frame = {'t': 'cmd_ack', 'id': cmd_id, 'c': kind,
                 'ok': bool(ok), 'error': error if error is not None else None}
        if detail is not None:
            frame['detail'] = detail
        self.emit(frame)

    def _manual_tick(self) -> None:
        """0.05s tick、0.10s 周期重发当前手动命令（manual_forward.py::tick
        的移植）；manual_zero 停止即发零一次。"""
        with self._manual_lock:
            if not self._manual_active:
                return
            now = time.monotonic()
            if now - self._manual_last_pub < MANUAL_PUBLISH_PERIOD_S:
                return
            self._manual_last_pub = now
            vx, wz = self._manual_cmd
        self._publish_twist(vx, wz)

    def _patrol_tick(self) -> None:
        """10Hz 重发 hub 缓存的 patrol_status 帧（web_debug.py:357-366）。"""
        with self._patrol_lock:
            frame = self._patrol_frame
        if frame is None:
            return
        try:
            self._pub_patrol.publish(
                RosString(data=json.dumps(frame, separators=(',', ':'),
                                          allow_nan=False)))
        except Exception:
            pass

    # ---- 看门狗（HANDOVER 坑 16/19 防复发）----
    def record_publish_failure(self, kind: str, exc: Exception) -> None:
        """命令派发 publish 失败打点。context 坏死（rclpy 上下文不可重建）
        类异常累积到滑窗阈值即返回 True（由 dispatch_command 触发自杀）。
        其他偶发异常只记日志不计数——不能因为单条发布失败就杀进程。"""
        text = str(exc)
        if not any(m in text for m in PUB_CONTEXT_FAIL_MARKERS):
            return
        now = time.monotonic()
        with self._health_lock:
            self._pub_fail_times.append(now)
            while (self._pub_fail_times and
                   now - self._pub_fail_times[0] > PUB_FAIL_WINDOW_S):
                self._pub_fail_times.popleft()
            return len(self._pub_fail_times) >= PUB_FAIL_LIMIT

    def cmd_poll_begin(self, conn) -> None:
        """command_thread 每次发起长轮询问：记录连接与开始时刻（main 线程
        巡检用）。"""
        with self._cmd_lock:
            self._cmd_conn = conn
            self._cmd_poll_t = time.monotonic()

    def cmd_poll_done(self) -> None:
        """一次轮询完整结束（读到响应并派发完）：清除巡检标记。"""
        with self._cmd_lock:
            self._cmd_conn = None
            self._cmd_poll_t = None

    def cmd_watchdog(self) -> None:
        """main 线程每 1s 巡检：若 command_thread 已超过 CMD_POLL_STALL_S
        未完成一次轮询（长轮询最长 20s，25s 即为卡死，坑 16），强制断开
        下行连接——command_thread 会在 getresponse/read 处抛异常走退避
        重连，全新连接/全新 HTTP 状态，无需自杀（上行线程健康，不受影响）。"""
        with self._cmd_lock:
            conn, t = self._cmd_conn, self._cmd_poll_t
        if conn is None or t is None:
            return
        if time.monotonic() - t <= CMD_POLL_STALL_S:
            return
        log('看门狗：command_thread 轮询卡死 >%gs，强制断开下行连接重连'
            % CMD_POLL_STALL_S)
        try:
            conn.close()
        except Exception:
            pass

    def dispatch_command(self, cmd: dict) -> None:
        """命令派发：六个发布器 + 两个重发循环 + action 客户端。
        发布失败经由 cmd_ack 如实回报（ultrasonic_rearm / cruise_enable），
        不假装已恢复（web_debug.py:570-580）。"""
        kind = cmd.get('c')
        if kind not in ('license_forward', 'guard_authorize', 'manual_cmd',
                        'manual_zero', 'patrol_status', 'nav_goal',
                        'nav_cancel', 'cmd'):
            log('派发命令 %r id=%r' % (kind, cmd.get('id')))
        cmd_id = cmd.get('id')
        try:
            if kind == 'manual_cmd':
                with self._manual_lock:
                    self._manual_active = True
                    self._manual_cmd = (float(cmd.get('vx', 0.0)),
                                        float(cmd.get('wz', 0.0)))
                    self._manual_last_pub = time.monotonic()
                vx, wz = self._manual_cmd
                self._publish_twist(vx, wz)
            elif kind == 'manual_zero':
                with self._manual_lock:
                    self._manual_active = False
                    self._manual_cmd = (0.0, 0.0)
                self._publish_twist(0.0, 0.0)  # 停止即发零（manual_forward.py:199-210）
            elif kind == 'license_forward':
                # 转发本进程（hub）已接受的原文，紧凑分隔符，不自造号码
                # （web_debug.py:909-924：下游的规则比本进程更严）。
                frame = cmd.get('frame') or {}
                self._pub_license.publish(RosString(
                    data=json.dumps(frame, separators=(',', ':'))))
            elif kind == 'guard_authorize':
                self._pub_authorize.publish(Bool(data=bool(cmd.get('value'))))
            elif kind == 'rearm_request':
                # 一次性边沿触发：下游「请求一次即消费、无论成败清标记」
                # （guard_bridge.py:258-261）；垫片只发不重放。
                self._pub_rearm.publish(Bool(data=True))
            elif kind == 'ultrasonic_rearm':
                self._ack(cmd_id, 'ultrasonic_rearm',
                          self._publish_bool_checked(self._pub_us_rearm))
            elif kind == 'cruise_enable':
                self._ack(cmd_id, 'cruise_enable',
                          self._publish_bool_checked(self._pub_cruise,
                                                     bool(cmd.get('on'))))
            elif kind == 'nav_goal':
                self._nav_dispatch(cmd)
            elif kind == 'nav_cancel':
                self._nav_cancel(cmd.get('rid'))
            elif kind == 'relocalize':
                # numpy 全局扫描匹配重定位（旧栈真实现），结果经 cmd_ack
                # 回执；一次性边沿命令，不重放。
                self._relocalize_global(cmd)
            elif kind == 'patrol_status':
                with self._patrol_lock:
                    self._patrol_frame = cmd.get('frame')
            else:
                log('未知命令类型 %r，忽略' % (kind,))
        except Exception as exc:
            log('命令 %r 派发异常：%r' % (kind, exc))
            if self.record_publish_failure(kind, exc):
                # 坑 19：rclpy context 坏死不可自愈，publish 全崩。立即自杀
                # 让 systemd（Restart=always）3s 拉起——新进程全新 rclpy
                # context；继续跑只会让许可断流锁停一直挂着。
                log('看门狗：publish 连续失败（rclpy context 疑似坏死），'
                    '自杀重启：%r' % (exc,))
                os._exit(1)
            if cmd_id is not None and kind in ('ultrasonic_rearm', 'cruise_enable'):
                self._ack(cmd_id, kind, (False, '垫片内部异常: %r' % (exc,)))

    @staticmethod
    def _publish_bool_checked(publisher, value: bool = True):
        """发布一帧 Bool；发布异常如实返回失败。"""
        try:
            publisher.publish(Bool(data=bool(value)))
            return True, None
        except Exception as exc:
            return False, '发布失败: %r' % (exc,)

    def _ack(self, cmd_id, kind, result) -> None:
        ok, error = result
        self.emit({'t': 'cmd_ack', 'id': cmd_id, 'c': kind,
                   'ok': bool(ok), 'error': error})

    # ---- navigate_to_pose（nav_goal.py 的桥接版）----
    def _nav_dispatch(self, cmd: dict) -> None:
        rid = cmd.get('rid')
        if rid is None or self._nav_client is None:
            self.emit({'t': 'nav_result', 'rid': rid, 'status': 'failed',
                       'text': '导航客户端不可用（缺 nav2_msgs 或客户端未建）',
                       'plan_error': ''})
            return
        try:
            x = float(cmd.get('x'))
            y = float(cmd.get('y'))
        except (TypeError, ValueError):
            self.emit({'t': 'nav_result', 'rid': rid, 'status': 'failed',
                       'text': '目标坐标必须是数字', 'plan_error': ''})
            return
        yaw = cmd.get('yaw')
        yaw = 0.0 if yaw is None else float(yaw)
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        # stamp=零时间戳（nav_goal.py:134,157-159：rclpy.time.Time().to_msg()
        # 即 sec=0/nanosec=0，Nav2 据此取最新 TF 变换；不得改用当前时钟）。
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        with self._nav_lock:
            self._nav_seq += 1
            self._nav_goals[rid] = {'handle': None, 'parked': False,
                                    'canceled': False, 'active': False}
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            # 服务发现没就绪：按发送失败处理（nav_goal.py:140-143 语义）
            with self._nav_lock:
                self._nav_goals.pop(rid, None)
            self.emit({'t': 'nav_result', 'rid': rid, 'status': 'failed',
                       'text': '导航服务不可达（wait_for_server 2.0s 超时）',
                       'plan_error': ''})
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        try:
            fut = self._nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        except Exception as exc:
            with self._nav_lock:
                self._nav_goals.pop(rid, None)
            self.emit({'t': 'nav_result', 'rid': rid, 'status': 'failed',
                       'text': str(exc), 'plan_error': ''})
            return
        fut.add_done_callback(lambda f, r=rid: self._on_goal_response(f, r))

    def _on_goal_response(self, future, rid: str) -> None:
        with self._nav_lock:
            entry = self._nav_goals.get(rid)
        try:
            handle = future.result()
        except Exception as exc:
            with self._nav_lock:
                self._nav_goals.pop(rid, None)
            self.emit({'t': 'nav_result', 'rid': rid, 'status': 'failed',
                       'text': str(exc), 'plan_error': ''})
            return
        if handle is None or not getattr(handle, 'accepted', False):
            with self._nav_lock:
                self._nav_goals.pop(rid, None)
            self.emit({'t': 'nav_result', 'rid': rid, 'status': 'failed',
                       'text': '目标被拒绝（规划器不可达或正在执行其他任务）',
                       'plan_error': ''})
            return
        parked = bool(entry and entry.get('parked'))
        canceled = bool(entry and entry.get('canceled'))
        with self._nav_lock:
            if rid in self._nav_goals:
                self._nav_goals[rid]['handle'] = handle
                self._nav_goals[rid]['active'] = True
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f, r=rid: self._on_result(f, r))
        self.emit({'t': 'nav_result', 'rid': rid, 'status': 'accepted',
                   'text': '', 'plan_error': ''})
        if parked or canceled:
            # 停车/取消时目标还没被接受：迟到的接受 → 立即取消
            # （nav_goal.py:193-212 + patrol.py:521-541）
            self._nav_cancel(rid)

    def _on_feedback(self, fb) -> None:
        """feedback 字段=nav_goal.py:248-269（distance_remaining /
        estimated_time_remaining / current_pose）；rid 由活跃目标反查。"""
        try:
            goal_handle = getattr(fb, 'goal_handle', None)
            fbm = getattr(fb, 'feedback', None)
            rid = None
            with self._nav_lock:
                for r, entry in self._nav_goals.items():
                    if entry.get('active') and entry.get('handle') is goal_handle:
                        rid = r
                        break
            if rid is None or fbm is None:
                return
            dr = getattr(fbm, 'distance_remaining', None)
            et = getattr(fbm, 'estimated_time_remaining', None)
            cp = getattr(fbm, 'current_pose', None)
            self.emit({'t': 'nav_feedback', 'rid': rid,
                       'dist': None if dr is None else round(float(dr), 2),
                       'eta': None if et is None else round(
                           float(et.sec + et.nanosec * 1e-9), 1),
                       'x': None if cp is None else round(
                           float(cp.pose.position.x), 2),
                       'y': None if cp is None else round(
                           float(cp.pose.position.y), 2)})
        except Exception:
            pass

    def _on_result(self, future, rid: str) -> None:
        try:
            result = future.result()
        except Exception as exc:
            with self._nav_lock:
                self._nav_goals.pop(rid, None)
            self.emit({'t': 'nav_result', 'rid': rid, 'status': 'failed',
                       'text': str(exc), 'plan_error': ''})
            return
        status = getattr(result, 'status', None)
        code = int(status) if status is not None else 0
        text = RESULT_TEXT.get(code, '导航结束（状态 %s）' % status)
        plan_error = ''
        err_code = getattr(result, 'error_code', None)
        try:
            err_code = int(err_code) if err_code is not None else 0
        except (TypeError, ValueError):
            err_code = 0
        if err_code:
            plan_error = 'error_code=%d(%s)' % (err_code, 'nav2')
        with self._nav_lock:
            self._nav_goals.pop(rid, None)
        nav_status = ('succeeded' if code == 4
                      else 'cancelled' if code == 5 else 'failed')
        self.emit({'t': 'nav_result', 'rid': rid, 'status': nav_status,
                   'text': text, 'plan_error': plan_error})

    def _nav_cancel(self, rid) -> None:
        if rid is None:
            return
        with self._nav_lock:
            entry = self._nav_goals.get(rid)
            handle = entry.get('handle') if entry else None
            if entry is not None and handle is None:
                # 尚未被接受：记进 parked，接受回调一到立即取消
                # （nav_goal.py:336-341）
                entry['parked'] = True
                entry['canceled'] = True
                return
            if entry is not None:
                entry['canceled'] = True
        if handle is None:
            return
        try:
            fut = handle.cancel_goal_async()
        except Exception as exc:
            # 同步抛异常 ≠ 取消失败：与旧行为一致按已发出处理
            log('取消同步异常（按已发出处理）：%r' % (exc,))
            return
        fut.add_done_callback(lambda f, r=rid, h=handle: self._on_cancel_response(f, r, h))

    def _on_cancel_response(self, future, rid: str, handle) -> None:
        """取消应答：失败 → 回滚 active（nav_goal.py:366-384），不发任何
        结束帧——hub 保持锁停，等真结果。"""
        ok = True
        response = None
        try:
            response = future.result()
        except Exception:
            ok = False                 # 应答丢了/失败：视为取消未生效
        if ok and response is not None:
            rc = getattr(response, 'return_value', True)
            ok = bool(rc) if isinstance(rc, bool) else True
        if ok:
            return
        with self._nav_lock:
            entry = self._nav_goals.get(rid)
            if entry is not None:
                entry['active'] = True   # 回滚为 active（不能认为已停）
                entry['canceled'] = False
                entry['handle'] = handle
        log('取消未被接受（rid=%r）：回滚 active' % (rid,))


# ---------------------------------------------------------------------------
# HTTP 上行 / 命令下行线程
# ---------------------------------------------------------------------------

def upstream_thread(node: BridgeShim, token: str) -> None:
    """单条持久连接，200ms 或满 32 帧成批 POST /bridge/frames；img 大帧
    直通不攒批；失败重连退避 1s；（重）连成功先发 bridge_hello。"""
    conn = None
    while True:
        try:
            if conn is None:
                conn = HTTPConnection(node._host, node._port, timeout=30)
            # 成批：等首批（最长 200ms），再收集到 32 帧；img 直通单独发。
            batch = []
            deadline = time.monotonic() + BATCH_WINDOW_S
            while len(batch) < BATCH_FRAMES:
                remaining = deadline - time.monotonic()
                frame = None
                direct = False
                with node._frames_lock:
                    if node._frames:
                        direct, frame = node._frames.popleft()
                if frame is None:
                    if batch:
                        break
                    if remaining > 0:
                        time.sleep(min(remaining, 0.02))
                        continue
                    break
                if direct and batch:
                    # 大帧直通：先把手上的小批发出去
                    break
                batch.append(frame)
                if direct:
                    break
            if not batch:
                continue
            if not node._connected:
                batch.insert(0, node.hello_frame())
                cached_map = node.cached_map_frame()
                if cached_map is not None:
                    batch.insert(1, cached_map)
                    log('已连接 Web-Next %s:%d，发送 bridge_hello + 缓存 /map'
                        % (node._host, node._port))
                else:
                    log('已连接 Web-Next %s:%d，发送 bridge_hello'
                        % (node._host, node._port))
                node._connected = True
            body = json.dumps({
                'mono_ms': int(time.monotonic() * 1000),
                'frames': batch,
            }, separators=(',', ':'), allow_nan=False)
            headers = {'Content-Type': 'application/json',
                       'Content-Length': str(len(body.encode('utf-8')))}
            if token:
                headers['X-Bridge-Token'] = token
            conn.request('POST', '/bridge/frames', body=body, headers=headers)
            resp = conn.getresponse()
            resp.read()
            if resp.status != 200:
                log('POST /bridge/frames → HTTP %s，重连' % resp.status)
                raise OSError('http %s' % resp.status)
        except Exception as exc:
            if node._connected:
                log('上行断开：%r，%ss 后重连' % (exc, RECONNECT_BACKOFF_S))
            node._connected = False
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass
            conn = None
            time.sleep(RECONNECT_BACKOFF_S)


def command_thread(node: BridgeShim, token: str) -> None:
    """GET /bridge/commands 长轮询（wait_ms=20000），收到即派发；cursor 跨
    重连保持（同进程内）；断线退避 1s。"""
    cursor = 0
    conn = None
    while True:
        try:
            if conn is None:
                conn = HTTPConnection(node._host, node._port, timeout=60)
            path = '/bridge/commands?wait_ms=20000&cursor=%d' % cursor
            headers = {}
            if token:
                headers['X-Bridge-Token'] = token
            node.cmd_poll_begin(conn)
            try:
                conn.request('GET', path, headers=headers)
                resp = conn.getresponse()
                data = resp.read()
            finally:
                node.cmd_poll_done()
            if resp.status != 200:
                raise OSError('http %s' % resp.status)
            payload = json.loads(data.decode('utf-8'))
            for cmd in payload.get('cmds', []):
                node.dispatch_command(cmd)
            cursor = int(payload.get('cursor', cursor))
        except Exception as exc:
            log('命令轮询断开：%r，%ss 后重连' % (exc, RECONNECT_BACKOFF_S))
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass
            conn = None
            time.sleep(RECONNECT_BACKOFF_S)


def main() -> int:
    parser = argparse.ArgumentParser(description='C50C Web-Next 桥接垫片')
    parser.add_argument('--server', default=os.environ.get('OPI_BRIDGE_SERVER',
                                                           'http://127.0.0.1:8088'),
                        help='Web-Next 服务地址（默认 http://127.0.0.1:8088）')
    parser.add_argument('--token-file', default=None,
                        help='令牌文件（缺省 ~/.config/opi-robot/bridge-token，'
                             '须属主本人且 0600；环境变量 OPI_BRIDGE_TOKEN 优先）')
    args = parser.parse_args()

    token = load_token(args.token_file)
    rclpy.init()
    node = BridgeShim(args.server, token)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    log('垫片启动：server=%s:%d token=%s tf=%s nav=%s' % (
        node._host, node._port, '已配置' if token else '缺失',
        node._tf_buffer is not None, node._nav_ok))
    up = threading.Thread(target=upstream_thread, args=(node, token), daemon=True)
    cmd = threading.Thread(target=command_thread, args=(node, token), daemon=True)
    up.start()
    cmd.start()
    try:
        while True:
            time.sleep(1.0)
            node.cmd_watchdog()
    except KeyboardInterrupt:
        log('收到 Ctrl-C，退出')
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
