"""下游融合节点：只在数据可信时给出动作，否则一律按未知处理。

第一阶段边界：
- 只显示与记录，不驱动车辆。``enable_cmd_vel`` 默认 False。
- 不发布任何自动转向。线速度与角速度在未知条件下都为 0。
- 雷达诊断不健康、扫描陈旧、前方扇区无有效覆盖时，动作记为 unknown。
- 图像仅记录新鲜度，不产生任何"检测结果"。
- 另发布 ``drive_safety`` 安全许可（每拍一次）：原始帧证据在流动 ∧
  雷达可信 ∧ 动作不是 stop/unknown ∧ 超声波不处于补充停车（2026-11-15
  接入前置 HC-SR04；2026-11-17 起无回波/断流 fail-closed，恢复需
  网页"重新启动超声波"，见 ``on_ultrasonic_rearm``）。该许可同时约束
  手动与自动——消费方是底盘桥仲裁器（``c50c_uart3`` 侧），本节点的
  cmd_vel 路径只覆盖自动巡航。证据追踪与许可判定见 ``safety_state``
  模块。

QoS：``/scan`` 与 ``/image_raw/compressed`` 必须用发布端的传感器约定
（:func:`opi_robot.qos.sensor_qos`），否则 RELIABILITY 不兼容会收不到任何消息。
``/diagnostics`` 保持 RELIABLE。
"""

import json
import math
from time import monotonic

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, CompressedImage, Range, Imu
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool, String

from .. import qos as Q
from .. import diagnostic_level as DL
from ..lidar_node import motion_check as MC
from ..lidar_node.motion_check import obstacle_blocked as obstacle_blocked_checked
from . import failclosed as F
from . import safety_state as SS

def wrap_angle(a):
    """角度归一到 (-π, π]。"""
    return math.atan2(math.sin(a), math.cos(a))


def frame_age_from_stamp(stamp, now):
    """原始 ``header.stamp`` 帧龄（秒）；未填/不可用返回 None。

    - ``stamp`` 缺失、字段缺失、非有限 → None（由调用方按陈旧处理）；
    - 全零时间戳 = rclpy ``Time()`` 的默认值（本车 lidar 固件未给时间时
      就是它），**不代表 1970 年** → None；
    - 帧龄远大于任何合法链路延迟（时间戳来自另一个时钟域）→ None；
      这不是"陈旧证据"，而是"时间戳不可用"，不能拿它冒充新鲜。
    """
    if stamp is None:
        return None
    try:
        sec = getattr(stamp, 'sec', None)
        nsec = getattr(stamp, 'nanosec', None)
        if sec is None or nsec is None:
            return None
        t = float(sec) + float(nsec) * 1e-9
    except (TypeError, ValueError, AttributeError):
        return None
    if not math.isfinite(t):
        return None
    if t == 0.0:
        return None
    age = float(now) - t
    if not math.isfinite(age) or abs(age) > MAX_PLAUSIBLE_FRAME_AGE_S:
        return None
    return age


def motion_direction_for(linear, angular, threshold=0.0):
    """本拍运动方向（``motion_check`` 方向名）。

    - 横向为主（|linear| <= threshold）或静止：按角速度方向判
      ``left``（负）/ ``right``（正）；命令层 +angular.z = 顺时针右转；
    - 否则 ``linear > 0`` 是 ``front``，``linear < 0`` 是 ``rear``；
    - 完全静止（两者都为 0）：``front``（无运动，取默认方向）。
    """
    try:
        lin = float(linear)
        ang = float(angular)
    except (TypeError, ValueError):
        return 'front'
    if abs(lin) <= threshold:
        # 命令层 angular.z 与 REP-103 反向（+ = 顺时针右转，见
        # manual_forward / heading_correction 的实车记录），所以 + 是右。
        # [B7 2026-11-18] 本口径与 guard 区 direction_sectors（B4 修复后，
        # c50c_uart3/motion_guard.py:263 vz>0 → 查右 −90°）一致；对账用例
        # test/test_motion_gate.py::test_b7_*（与 guard 同命令同侧）。
        if ang > 0.0:
            return 'right'
        if ang < 0.0:
            return 'left'
        return 'front'
    return 'front' if lin > 0.0 else 'rear'


def heading_correction(ref_yaw, yaw, gain, max_cmd, deadband, integral=0.0):
    """[B3] 航向保持 PI 控制器：直线行走的转向输出（纯函数，可单测）。

    误差 = ref - yaw，REP-103 逆时针为正（IMU gz 实测确认）。
    命令层 angular.z 与 REP-103 **反向**（+0.8 → 实测顺时针右转，
    见 c50c 符号链记录），故命令取负号：误差为正 = 车头偏右
    （yaw 掉到基准下面）= 需要左转 = 发负命令。

    量纲：本车 angular.z 命令 = 左右轮差速线速度 m/s，不是 rad/s
    （实测 0.8 命令 ≈ 1.93 rad/s 实际转速，比值 ~2.4）。gain 按
    "5°（0.087 rad）误差给 ~0.17 命令 ≈ 0.4 rad/s 实际回正"定标，
    默认 2.0。输出钳制 ±max_cmd；死区内 P 项回 0，防 IMU 噪声画龙。

    [B3 PI 2026-11-16] 纯 P 的稳态偏差实车实锤：地面/电池造成的恒定
    右拉力让 err 稳在 +2.3°（cmd −0.08 恒定顶住拉力），0.4 m/s 下
    每秒横移 ~1.6cm，几米就是 20cm。积分项把稳态偏差慢慢吃到零，
    由积分顶着恒定拉力；顺带吸收陀螺零偏残余（0.69cm/m 地板）。
    integral 由节点层维护（带钳位/冻结/清零纪律），本函数只负责
    "P 项受死区门控、积分项直通、总和钳制"。
    """
    err = wrap_angle(ref_yaw - yaw)
    cmd = integral
    if abs(err) > deadband:
        cmd -= gain * err
    if cmd > max_cmd:
        cmd = max_cmd
    elif cmd < -max_cmd:
        cmd = -max_cmd
    return cmd, err


HEALTHY_STATE = 'healthy'
#: 方向门帧龄上限（秒）：与 ``ScanFreshness``/``motion_check.STALE_S`` 同口径。
DEFAULT_SCAN_STALE_S = 0.5
#: header 帧龄超过它即判"时间戳不可用"（时钟域不符），不是"未填"。
MAX_PLAUSIBLE_FRAME_AGE_S = 60.0
# 动作层使用现场纸箱验证的 LaserScan 左 +90°、右 −90°。
LEFT_DEG = 90.0
RIGHT_DEG = -90.0


def _extract_state(values):
    """从诊断的 KeyValue 列表里取 state。

    返回状态字符串；以下情况返回空串，由上层判为不可信：
      - 没有 state 键；
      - state 值为空或全是空白；
      - 出现多个 state 键且取值不一致。
    重复但取值相同视为一条，不算冲突。
    """
    found = [kv.value for kv in values
             if getattr(kv, 'key', '') == 'state']
    if not found:
        return ''
    cleaned = {str(v).strip() for v in found}
    if len(cleaned) != 1:
        return ''
    value = cleaned.pop()
    return value


class FusionNode(Node):

    def __init__(self, context=None):
        # context 用于测试脚本在独立域里建节点（见 scripts/ros_isolation.py）。
        # 生产启动走 launch，不传 context，行为不变。
        super().__init__('fusion_node', context=context)

        # 停车距离 0.4 m（2026-11-14 用户定，原 0.25 m）。
        self.declare_parameter('stop_distance', 0.4)
        self.declare_parameter('slow_distance', 0.9)
        self.declare_parameter('front_half_angle_deg', 30.0)
        self.declare_parameter('side_half_angle_deg', 30.0)
        # 与底盘桥 PLACEHOLDER_MAX_LINEAR_MPS 一致，都是 0.4。
        # 超限会被桥判 NAV_INVALID 并改发零速。
        self.declare_parameter('max_linear', 0.4)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('image_timeout', 1.0)
        self.declare_parameter('lidar_diag_timeout', 2.0)
        self.declare_parameter('min_front_points', 3)
        self.declare_parameter('enable_cmd_vel', False)
        # 自动巡航开关：只由网页显式打开，默认关闭。
        # 打开前不发任何速度，保证上电后车不会自己动。
        self.declare_parameter('cruise_topic', 'cruise_enable')
        # 巡航运行时长（2026-11-14 用户定）：≤0 表示无限运行，直到
        # 网页手动停止、节点重启或安全条件不允许。正值为超时秒数。
        self.declare_parameter('cruise_timeout', 0.0)
        self.declare_parameter('cruise_max_run_s', 0.0)
        # 巡航前进速度，单位 m/s。实际取它与 max_linear 的较小值。
        self.declare_parameter('cruise_linear', 0.4)
        # 输出话题：2026-11-14 起自动巡航与网页手动分离，本节点发
        # /auto_cmd_vel；底盘桥把 auto 通道与手动通道分开缓存、分开仲裁。
        # 手动失联期间桥会抑制 auto 非零（不得自动接管）。
        self.declare_parameter('cmd_vel_topic', 'auto_cmd_vel')
        self.declare_parameter('decide_period', 0.1)
        self.declare_parameter('report_period', 1.0)
        # 原始帧证据失效时限：与 lidar frame_timeout（0.5 s）同级。
        # 原始帧停止后（含缓存圈重发期内 frames_ok 冻结），最多再过该时限
        # 安全许可必须翻为拒绝。判定用节点单调时钟。
        self.declare_parameter('raw_frame_timeout', 0.5)
        # 安全许可有效期：本节点每 decide_period 重评重发；接收端（底盘桥）
        # 还要用它自己的上限钳制，不能由发布端单方面延长。
        self.declare_parameter('safety_ttl_s', 0.4)
        self.declare_parameter('evidence_topic', 'scan_raw_evidence')
        self.declare_parameter('safety_topic', 'drive_safety')
        # [C50C 2026-11-15] 前置 HC-SR04 超声波补充停车（雷达 22cm 高看
        # 不到的低矮障碍）。只增停不解除：结果只会让许可更严。
        # 信任上限 us_release_distance 的几何依据：传感器离地 h、波束半角
        # 7.5°，地面回波最早出现在 h/tan(7.5°) ≈ 6.76h 处（h=5cm → 38cm），
        # 更远的读数（含无回波的 max_range）不采信。
        # [C50C 2026-11-15 傍晚] 传感器移装离地 10cm（用户现场操作）：
        # 地面回波最早 ~76cm，阈值随之放宽到 0.40/0.50。
        self.declare_parameter('us_topic', 'ultrasonic_front')
        self.declare_parameter('us_stop_distance', 0.40)
        self.declare_parameter('us_release_distance', 0.50)
        self.declare_parameter('us_timeout', 0.6)
        # 发布端 max_range：0xFFFF（无回波/陈旧）就被映射成它。>= 该值
        # 的读数判无效（fail-closed，见 safety_state.UltrasonicGate）。
        self.declare_parameter('us_max_range', 4.5)
        # [2026-11-17] 车体轮廓：运动方向前置门用。默认 0 = 未标定，
        # motion_check 对所有方向判 no_geometry、/drive_safety 拒绝；
        # 必须由 config/params.yaml 用实测尺寸显式打开（与 nav2 的
        # robot_radius 同一口径核对）。
        self.declare_parameter('body_half_length', 0.0)
        self.declare_parameter('body_half_width', 0.0)
        # 车体中心在雷达前方为正，后方为负（m）。
        self.declare_parameter('body_center_x', 0.0)
        self.declare_parameter('body_margin', 0.0)
        # 慢行区最低速度比例：1.0 = 不降速（旧行为）。
        self.declare_parameter('slow_min_ratio', 0.5)
        # [C50C 2026-11-15 B3] 航向保持（直线行走）：巡航打开瞬间锁存
        # IMU 积分航向为基准。已收过 IMU 数据后断流超过 imu_timeout，
        # 自动巡航锁停，等下一次关→开。
        self.declare_parameter('imu_topic', 'imu/data_raw')
        self.declare_parameter('steer_gain', 2.0)
        self.declare_parameter('steer_max', 0.5)
        # 死区（2026-11-15 实车首测定案 0.01→0.002）：恒定右漂下，死区
        # 内完全不修、攒满才修，锯齿式净偏移实测 8-10m 偏 ~10cm。
        # 0.002 rad ≈ IMU 积分噪声地板（σ≈0.0017 rad/10s），再小只抖不修。
        self.declare_parameter('steer_deadband_rad', 0.002)
        # [B3 PI 2026-11-16] 积分项：稳态偏差（地面/电池恒定拉力 + 陀螺
        # 零偏残余）实车实测 2.3°→0.4m/s 每秒横移 1.6cm。ki=0.8/s 配
        # gain=2.0 → 闭环阻尼 ζ≈0.6，拉力 ~3-6s 吃完；积分钳位 0.15
        # （≈max_cmd 的 30%）防饱和猛甩。
        self.declare_parameter('steer_ki', 0.8)
        self.declare_parameter('steer_integral_max', 0.15)
        self.declare_parameter('imu_timeout', 0.5)

        gp = self.get_parameter
        self.stop_d = gp('stop_distance').value
        self.slow_d = gp('slow_distance').value
        front_half = math.radians(gp('front_half_angle_deg').value)
        side_half = math.radians(gp('side_half_angle_deg').value)
        self.front_half = front_half
        self.side_half = side_half
        self.max_linear = gp('max_linear').value
        # 自动巡航固定按巡航档 0.4 m/s 前进（2026-11-13 用户指定）。
        # 这里取两者较小值，保证巡航不会超过 max_linear。
        self.cruise_linear = min(float(gp('cruise_linear').value),
                                 float(self.max_linear))
        self.image_timeout = gp('image_timeout').value
        self.lidar_diag_timeout = gp('lidar_diag_timeout').value
        self.min_front_points = int(gp('min_front_points').value)
        self.enable_cmd_vel = bool(gp('enable_cmd_vel').value)
        self.cruise_timeout = float(gp('cruise_timeout').value)
        self.cruise_max_run_s = float(gp('cruise_max_run_s').value)
        if not math.isfinite(self.cruise_max_run_s) or self.cruise_max_run_s < 0.0:
            raise ValueError('cruise_max_run_s 必须是有限的非负秒数')
        # 巡航状态：False 时不发速度。last_cruise_time 用于超时保护。
        self.cruise_on = False
        self._cruise_imu_timed_out = False
        self._cruise_run_deadline = None
        self.last_cruise_time = None
        # [B3] 航向保持状态：imu_yaw 为 gz 积分（REP-103 逆时针为正）。
        # 零偏维护（用户定则 2026-11-15：使用前必须重置、必须证实平衡态）：
        # 只在巡航关 ∧ 四轮编码器全 0 ∧ 窗口方差小 ∧ |均值|<0.05 时刷新。
        # 实测静止偏置 ~0.002 rad/s、噪声 ~0.0005 rad/s。
        self.imu_topic = str(gp('imu_topic').value)
        self.steer_gain = float(gp('steer_gain').value)
        self.steer_max = float(gp('steer_max').value)
        self.steer_deadband = float(gp('steer_deadband_rad').value)
        self.steer_ki = float(gp('steer_ki').value)
        self.steer_integral_max = float(gp('steer_integral_max').value)
        self.imu_timeout = float(gp('imu_timeout').value)
        self.imu_yaw = 0.0
        self.imu_last_t = None      # 墙钟，判 IMU 断流
        self.imu_bias = None        # None = 零偏未收敛（静止证据不足）
        self._gz_window = []        # 零偏滚动窗口（20 样本 ≈ 1s）
        self.last_encoders = None   # 编码器四轮瞬时速度（静止证据）
        self._encoders_t = None     # [B8] 最近一条编码器诊断到达时刻
        self._imu_stamp = None      # 头时间戳，算积分步长
        self.cruise_yaw_ref = None  # 巡航开启瞬间的航向基准
        self.last_steer = None      # 观测用：{'err_deg','imu_age','cmd','integ'}
        # [B3 PI 2026-11-16] 积分项状态：P 控制留稳态偏差（直行右偏 20cm
        # 的根因），积分项把恒定拉力慢慢吃掉。只在车体真实移动时积分
        # （_body_still 时冻结，与零偏维护同一静止证据），dt 用自增时钟
        # 差分并钳位，防单次大步长打爆。
        self.steer_integral = 0.0
        self._steer_integral_t = None

        self.fresh = F.ScanFreshness(scan_timeout=gp('scan_timeout').value)
        self.latest_scan = None
        self.last_image_time = None
        self.lidar_status = None
        self.lidar_status_time = None
        self.lidar_state = ''
        # 恢复门槛（回应第四次、第五次审核阻断 1）：
        #   一旦出现不可信，回到可信必须看到"信任失效之后到达的新扫描"。
        #
        #   失信边界 lost_seq 在**收到不可信诊断的那一刻**锁存（见
        #   _note_untrusted），不是在判定时锁存。第五次审核证明：
        #   两条诊断之间如果没有 decide 调用，connecting 会被随后的
        #   healthy 覆盖，判定时看不到它，旧扫描就能绕过门槛。
        #
        #   判据用序号而不是时间戳：同一轮循环里两个回调的时间戳可能相等，
        #   时序无法分辨；序号对每次回调唯一递增，可以可靠比较先后。
        #
        #   参照点选"信任失效的那一刻"，不是"healthy 诊断时刻"：
        #   扫描与诊断是两个独立话题，同一轮里扫描可能先到也可能后到，
        #   用诊断时刻做参照会把合法的恢复顺序判错。
        #
        #   已知限制：序号只能证明本节点回调的先后。
        #   跨话题的真实到达顺序、以及节点根本没收到的诊断，
        #   本地序号无法证明——见 reports/stage1-delivery.md 已知限制。
        self.lost_seq = None
        self._seq = 0
        self.scan_seq = None
        self.last_scan_time = None
        self.last_report = 0.0

        self.cmd_vel_topic = str(gp('cmd_vel_topic').value)
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_report = self.create_publisher(String, 'obstacle_report', 10)

        # ---- 安全许可（T1）：原始帧证据 + 雷达可信 + 避障动作 → 单一布尔 ----
        # 许可对手动与自动同时生效：消费方是底盘桥仲裁器（T2 接线），
        # 不经本节点的 cmd_vel 路径。
        self.safety_topic = str(gp('safety_topic').value)
        self.evidence_topic = str(gp('evidence_topic').value)
        self.safety_ttl_s = float(gp('safety_ttl_s').value)
        if self.safety_ttl_s <= 0:
            raise ValueError('safety_ttl_s 必须为正')
        self.raw_evidence = SS.RawEvidenceTracker(
            float(gp('raw_frame_timeout').value))
        self.safety_seq = 0
        # ---- 超声波补充停车状态 ----
        self.us_stop_d = float(gp('us_stop_distance').value)
        self.us_release_d = float(gp('us_release_distance').value)
        self.us_timeout = float(gp('us_timeout').value)
        if not (0.0 < self.us_stop_d < self.us_release_d):
            raise ValueError('需要 0 < us_stop_distance < us_release_distance')
        self.us_topic = str(gp('us_topic').value)
        # 迟滞/超时/信任上限全部在纯逻辑门里（见 safety_state.UltrasonicGate）
        self.us_max_range = float(gp('us_max_range').value)
        self.us_gate = SS.UltrasonicGate(self.us_stop_d, self.us_release_d,
                                         self.us_timeout,
                                         invalid_at=self.us_max_range)
        # ---- 运动方向前置门（motion_check）：自动/导航/许可同源 ----
        self.body_geometry = {
            'half_length': float(gp('body_half_length').value),
            'half_width': float(gp('body_half_width').value),
            'center_x': float(gp('body_center_x').value),
            'half_deg': float(gp('front_half_angle_deg').value),
            'margin': float(gp('body_margin').value),
            'range_min': 0.02,
            'range_max': 12.0,
        }
        self.slow_min_ratio = float(gp('slow_min_ratio').value)
        if self.slow_min_ratio <= 0.0:
            raise ValueError('slow_min_ratio 必须为正')
        self.motion_direction = MC.FRONT
        # [2026-11-17] 方向门开关（节点级隐藏参数，默认 True）。旧版本把
        # 方向门写成硬编码 False，导致自动/导航命令绕开运动方向判据、
        # /drive_safety 的 motion 永远 gate_disabled。"未标定"由
        # body_half_length/width = 0 → <dir>_no_geometry 拦停，不需要
        # （也不允许）用关闭门来"兼容"。
        self.declare_parameter('motion_gate_enabled', True)
        self.motion_gate = bool(gp('motion_gate_enabled').value)
        # 本拍唯一一份雷达结论：四扇区最近距离 + 运动方向门结果
        # （含参与判定的那一帧，用于同拍复用，避免二次计算）。
        self._motion = None
        self._motion_scan = None
        # 上一条实际发出的命令（无巡航意图时给方向门定向用）。
        self._last_cmd = (0.0, 0.0)
        # 最近一次 evaluate 的可信判定，供许可层复用（避免二次判定的
        # lost_seq 副作用）。
        self._last_trust = (False, 'no_diagnostics')
        self.pub_safety = self.create_publisher(String, self.safety_topic,
                                                Q.status_qos())
        # 证据与 /scan 同源（BEST_EFFORT）：丢一条证据只损失一次增长样本，
        # 时限判定不依赖单条消息必达。
        self.create_subscription(String, self.evidence_topic, self.on_evidence,
                                 Q.sensor_qos())

        # 传感器流用统一约定：与雷达、摄像头发布端同源，不得改用默认 RELIABLE
        #
        # 壁障判定**只用雷达**：stop / slow / clear 全部来自 /scan 的距离，
        # 由 failclosed.classify(front, left, right) 决定。
        self.create_subscription(LaserScan, 'scan', self.on_scan,
                                 Q.sensor_qos())
        # [C50C 2026-11-15] 超声波补充停车：Range 发布端（uart3_node）用
        # sensor_data QoS（BEST_EFFORT），必须同约定否则一条也收不到。
        # 只消费"有无近距回波"，不参与任何前进许可的放宽。
        self.create_subscription(Range, self.us_topic, self.on_ultrasonic,
                                 Q.sensor_qos())
        # [2026-11-17] 超声门 fail-closed 后唯一的恢复入口：网页按键 →
        # ultrasonic_rearm=True → us_gate.rearm()。读数本身永远不能解除。
        self.create_subscription(Bool, 'ultrasonic_rearm',
                                 self.on_ultrasonic_rearm, Q.status_qos())
        # 摄像头订阅只为上报"图像是否在刷新"，不参与任何动作判定。
        # 本车摄像头不是深度相机，没有距离信息可用；即使此处没数据，
        # evaluate() 仍照常按雷达给结论。
        self.create_subscription(CompressedImage, 'image_raw/compressed',
                                 self.on_image, Q.sensor_qos())
        # 诊断是状态消息，丢失代价高，保持 RELIABLE
        self.create_subscription(DiagnosticArray, 'diagnostics',
                                 self.on_diagnostics, Q.status_qos())
        # 巡航开关来自网页。用 status_qos（RELIABLE）保证开关指令不丢。
        self.create_subscription(Bool, str(gp('cruise_topic').value),
                                 self.on_cruise, Q.status_qos())
        # [B3] IMU：底盘桥用 qos_profile_sensor_data（BEST_EFFORT）发布
        # imu/data_raw，订阅端必须同用传感器 QoS，否则可靠性不兼容收不到。
        self.create_subscription(Imu, self.imu_topic, self.on_imu,
                                 Q.sensor_qos())

        # 包一层：回调异常若不在此捕获，rclpy 会抛出并中止整条回调链，
        # 话题从此静默却仍在图上显示为发布者，很难排查。
        self.decide_timer = self.create_timer(gp('decide_period').value,
                                              self._decide_safe)
        self.report_timer = self.create_timer(gp('report_period').value,
                                              self.log_status)

        self.get_logger().info(
            'fusion_node 启动: enable_cmd_vel=%s（恒为安全默认）；'
            '[B3] 航向保持 PI 已装（gain=%s/max=%s/ki=%s/integ_max=%s），'
            '仅巡航开启且 IMU 新鲜时转向'
            % (self.enable_cmd_vel, self.steer_gain, self.steer_max,
               self.steer_ki, self.steer_integral_max))

    def halt_timers(self):
        """取消所有定时器。供测试脚本与异常退出使用。

        与雷达节点同名同义：返回实际取消的数量，方便调用方断言。
        定时器不停就销毁节点，销毁后仍可能有回调在跑。
        """
        count = 0
        for name in ('decide_timer', 'report_timer'):
            timer = getattr(self, name, None)
            if timer is None:
                continue
            try:
                self.destroy_timer(timer)
            except Exception:                       # noqa: BLE001
                continue
            setattr(self, name, None)
            count += 1
        return count

    # ---------- 时间 ----------

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---------- 订阅 ----------

    def _next_seq(self):
        self._seq += 1
        return self._seq

    def on_scan(self, msg):
        now = self.now_s()
        self.latest_scan = msg
        self.last_scan_time = now
        self.scan_seq = self._next_seq()
        self.fresh.mark(now)

    def on_image(self, msg):
        # 只记录到达时间。JPEG 字节数不是亮度，也不是检测结果。
        # 壁障不用它：摄像头不是深度相机，取不到距离。本回调**不得**
        # 影响 evaluate() 的结论，也不得作为允许前进的条件。
        self.last_image_time = self.now_s()

    def on_ultrasonic(self, msg):
        """前置 HC-SR04 距离回调（sensor_msgs/Range）。

        发布端约定（``c50c_uart3/uart3_node.py:_publish_ultrasonic``）：
        0xFFFF 无回波/数据陈旧映射为 ``range = max_range``。2026-11-17 起
        ``>= us_max_range`` 的读数与 NaN/inf/0/负数一样判无效 → 停车
        （fail-closed），恢复需 ``rearm()``。见
        ``safety_state.UltrasonicGate.feed``。
        """
        try:
            r = float(msg.range)
        except (TypeError, ValueError):
            return
        self.us_gate.feed(r, self.now_s())

    def on_ultrasonic_rearm(self, msg):
        """"重新启动超声波"：只接受显式 True，清除无效锁（需人明确动作）。"""
        if not bool(getattr(msg, 'data', False)):
            return
        self.us_gate.rearm(self.now_s())
        self.get_logger().warning('超声波门已重新启动（清除无效锁）')

    def on_evidence(self, msg):
        """雷达原始帧证据回调（``scan_raw_evidence``）。

        解析失败按「无证据」计数，不刷新任何时限——损坏的证据不能当
        好证据用。判定全部用本节点单调时钟（``now_s``）。
        """
        parsed = SS.parse_evidence_json(getattr(msg, 'data', None))
        if parsed is None:
            self.raw_evidence.note_malformed()
            return
        self.raw_evidence.feed(parsed['epoch'], parsed['frames_ok'],
                               self.now_s(), parsed['max_point_age_ms'])

    def on_diagnostics(self, msg):
        """解析雷达诊断。

        state 必须来自当前这条匹配诊断，不得沿用上一条的值
        （回应第四次审核阻断 2）。缺失、空值、重复冲突、
        不认识的取值一律记为空串，由 lidar_trusted 判为不可信。

        一条消息里出现多个 name=='lidar' 的 status 时：
          取最后一个，因为它是最新写入的状态；但如果多个的
          level 或 state 互相不一致，则整体置信度为未知。
        """
        # [B3] 车身静止证据：桥把四轮瞬时轮速放诊断键 encoder_{a..d}_reported，
        # 轮停字段回 0（实测：编码器是瞬时速度，勿用首尾差分析）。
        # 哪条 status 带编码器键就收哪条，与雷达状态解耦——必须放在
        # lidar 早退之前：纯底盘诊断没有 lidar status，不能被它挡掉。
        enc = {}
        for status in msg.status:
            for kv in status.values:
                key = getattr(kv, 'key', '')
                if key.startswith('encoder_') and key.endswith('_reported'):
                    enc[key] = getattr(kv, 'value', '')
        if enc:
            self.last_encoders = enc
            # [B8] 静止证据到达时刻：encoders_still 用它判证据是否新鲜，
            # 超过 ENCODER_STILL_TIMEOUT_S 的"全 0"不能再当停住证据。
            self._encoders_t = self.now_s()

        matches = [st for st in msg.status if st.name == 'lidar']
        if not matches:
            # 没有任何雷达诊断：不是"没有变化"，而是没有可用信息
            return

        parsed = []
        for status in matches:
            # 真实消息里 level 是单字节 bytes，伪 ROS 里是整数。
            # 统一解码成整数；无法识别时记 None，按"未知"fail closed。
            level = DL.decode_level(status.level)
            parsed.append((level, _extract_state(status.values)))

        levels = {lv for lv, _ in parsed}
        states = {st for _, st in parsed}
        if len(levels) > 1 or len(states) > 1:
            # 同一条消息内自相矛盾：不能挑一个用，整体判未知
            self.lidar_status = None
            self.lidar_state = ''
        else:
            self.lidar_status = parsed[-1][0]
            self.lidar_state = parsed[-1][1]

        self.lidar_status_time = self.now_s()
        # 关键：在**收到诊断时**就锁存失信边界，不等到判定时才记。
        # 两条诊断之间可能没有 decide 调用，只在判定时锁存会让
        # connecting 被随后的 healthy 覆盖，旧扫描就能绕过恢复门槛。
        if not self._diag_trustworthy():
            self._note_untrusted()

    def _diag_trustworthy(self):
        """当前这条诊断本身是否可信（不看扫描）。"""
        if self.lidar_status is None:
            return False
        if self.lidar_status != DL.LEVEL_OK:
            return False
        return self.lidar_state == HEALTHY_STATE

    def _note_untrusted(self):
        """锁存信任失效的序号。已在失效状态时保持最早的边界。"""
        if self.lost_seq is None:
            self.lost_seq = self._next_seq()

    # ---------- 判定 ----------

    def lidar_trusted(self, now):
        """雷达可用于下游判定。

        条件全部满足才返回 True：
          1. 收到过诊断，且诊断未超时；
          2. 等级严格等于 LEVEL_OK（WARN 及 worse 一律不可信）；
          3. 诊断里的 state 字段存在且严格等于 'healthy'；
          4. 最近一帧扫描新鲜；
          5. 若信任曾经失效，必须已经收到失效边界之后的新扫描。

        规则是 fail-closed：任何一个条件无法确认，都返回 False。
        第 5 条回应第四次、第五次审核阻断 1：旧扫描还在 scan_timeout
        窗口内，不能因为"还没过期"就当作恢复。恢复必须证明雷达真的
        在出新数据，且该新数据出现在信任失效之后。

        扫描与诊断的到达顺序不做要求：扫描可以先于 healthy 诊断到达，
        只要它晚于失信边界就算有效。两个话题独立调度，强求某一顺序
        会把合法恢复判错。

        已知限制（不能在本地证明的部分）：
          - 序号只反映本节点回调的先后，不反映消息在传输层的真实顺序；
          - 节点根本没收到某条不可信诊断时，本地无从得知，也就不会
            锁存边界；
          - 两类情形都依赖上游可靠发布诊断来兜底，本身不是安全保证。
        """
        ok, why = self._trust_reason(now)
        if ok:
            # 本次判定可信且已满足恢复条件，恢复过程结束
            self.lost_seq = None
        else:
            # 判定不可信时也要锁存边界（覆盖诊断超时、扫描过期等
            # 不来自 on_diagnostics 的失信情形）。已锁存时保持最早值。
            self._note_untrusted()
        return ok, why

    def _trust_reason(self, now):
        if self.lidar_status_time is None:
            return False, 'no_diagnostics'
        if now - self.lidar_status_time > self.lidar_diag_timeout:
            return False, 'diagnostics_stale'
        if self.lidar_status is None:
            # 收到了诊断但等级无法解析。不能当成 OK。
            return False, 'diagnostics_unreadable'
        # 只有 OK 才算健康。WARN（含 connecting）也必须拒绝。
        # 旧实现用 >= LEVEL_ERROR 判坏，导致连接中状态沿用旧扫描。
        if self.lidar_status != DL.LEVEL_OK:
            return False, 'lidar_%s' % (self.lidar_state or 'not_ok')
        # 等级与 state 必须一致。缺 state 或 state 不是 healthy 时拒绝，
        # 防止等级被误写成 OK 而实际状态仍未知。
        if self.lidar_state != HEALTHY_STATE:
            return False, 'state_%s' % (self.lidar_state or 'missing')
        if not self.fresh.is_fresh(now):
            return False, 'scan_stale'
        # 恢复门槛：出现过不可信之后，必须看到失效边界之后的新扫描。
        if self.lost_seq is not None:
            if self.scan_seq is None or self.scan_seq <= self.lost_seq:
                return False, 'recovery_no_new_scan'
        return True, 'ok'

    def evaluate(self, now):
        trusted, why = self.lidar_trusted(now)
        # 记录本次判定，供许可层（_evaluate_safety）复用：重复调用
        # lidar_trusted 会再次改写 lost_seq，产生判定之外的副作用。
        self._last_trust = (trusted, why)
        if not trusted:
            return {
                'action': 'unknown', 'reason': why,
                'front': None, 'left': None, 'right': None,
                'linear': 0.0, 'angular': 0.0,
                # 雷达不可信时不判方向门（不知道帧是否新鲜），许可由
                # 雷达可信度那一关拒绝；这里不伪造"已判过且空旷"。
                'motion': self.motion_verdict(),
            }
        msg = self.latest_scan
        front = F.sector_distance(msg, 0.0, self.front_half,
                                  min_points=self.min_front_points)
        left = F.sector_distance(msg, math.radians(LEFT_DEG), self.side_half)
        right = F.sector_distance(msg, math.radians(RIGHT_DEG), self.side_half)
        action, ratio, _ = F.classify(front, left, right,
                                      stop_distance=self.stop_d,
                                      slow_distance=self.slow_d)
        # 运动方向门始终计算（页面状态要看到）；巡航关闭时不拦停——
        # 自动命令此刻恒为零，方向门只在"有自动运动意图"时生效。
        motion = self.motion_verdict(msg, active=self.cruise_active(now))
        self._motion = motion
        if action == 'unknown':
            result = {
                'action': 'unknown', 'reason': 'front_no_points',
                'front': None, 'left': left, 'right': right,
                'linear': 0.0, 'angular': 0.0, 'motion': motion,
            }
            result.update(self._command(motion, 0.0, 0.0))
            return result
        lin, ang = F.apply_limits(action, ratio, max_linear=self.max_linear)
        if action == 'slow':
            # [2026-11-17] 旧代码下发 cruise_linear 恒速，慢行区的
            # 计算限速被丢弃；这里把慢行区限速落到 linear 上。
            lin = lin * self.slow_min_ratio
        elif action == 'clear':
            # 运动方向门虽然放行但方向扇区贴得比 slow_distance 近时，
            # 仍按同一比例限速（门与限速读同一个 motion）。
            nearest = motion['ranges'].get(motion['direction'])
            if nearest is not None and nearest < self.slow_d:
                lin = lin * self.slow_min_ratio
        result = {
            'action': action, 'reason': 'ok',
            'front': front, 'left': left, 'right': right,
            'linear': lin, 'angular': ang, 'motion': motion,
        }
        return result

    def _command(self, motion, lin, ang):
        """按本拍最终命令给运动方向门定向，并用同一份判定回写 motion。

        ``motion`` 由 ``motion_verdict(..., active=...)`` 产出，方向取
        ``motion_direction``（默认前）；这里按最终 ``lin``/``ang`` 覆盖方向
        并**原地**改同一个 dict——evaluate 结果、cmd_vel 门、/drive_safety
        共享这一份，不各自重算。最终命令为零（停车/巡航关）时不拦停
        （无运动意图），但仍标出方向以便观察。
        """
        lin = float(lin)
        ang = float(ang)
        moving = abs(lin) > 0.0 or abs(ang) > 0.0
        if not moving:
            return {'motion_linear': 0.0, 'motion_angular': 0.0}
        direction = motion_direction_for(lin, ang)
        if direction != motion['direction']:
            fresh = self.motion_verdict(msg=self._motion_scan,
                                        active=motion.get('active', True),
                                        direction=direction)
            fresh.pop('scan', None)        # 内部字段不进 result/JSON
            motion.clear()
            motion.update(fresh)
        return {'motion_linear': lin, 'motion_angular': ang}

    def decide(self):
        now = self.now_s()
        self._check_cruise_imu_timeout(now)
        result = self.evaluate(now)
        # [B3] 航向保持转向：只在巡航生效、已有航向基准、IMU 新鲜时
        # 输出（heading_correction 纯函数，符号链见其 docstring）。
        # 其余情况一律 0——没有依据时绝不打方向。
        steer = 0.0
        steer_err = None
        imu_fresh = (self.imu_last_t is not None
                     and now - self.imu_last_t <= self.imu_timeout)
        if not (self.cruise_active(now)
                and self.cruise_yaw_ref is not None):
            # 巡航关/无基准：积分清零，下次开启（重锁）从头再学。
            self.steer_integral = 0.0
            self._steer_integral_t = None
        elif imu_fresh:
            # [B3 PI 2026-11-16] 先用当前积分算本周期命令（离散 PI：
            # u[k] = I[k] + P(e[k])，再用 e[k] 更新 I[k+1]），积分项
            # 把 P 控制的稳态偏差（实测 +2.3°，右偏 20cm 根因）在
            # 3-6s 内吃掉。
            steer, steer_err = heading_correction(
                self.cruise_yaw_ref, self.imu_yaw,
                self.steer_gain, self.steer_max, self.steer_deadband,
                self.steer_integral)
            # 积分更新：dt 取自增时钟差分并钳位 (0, 0.5]——时钟回拨
            # 或长时间无 decide 时本周期不积分，但不丢已有状态；车体
            # 静止时冻结（与零偏维护同一静止证据，防停车累积）。
            dt = None
            if self._steer_integral_t is not None:
                dt = now - self._steer_integral_t
                if not 0.0 < dt <= 0.5:
                    dt = None
            self._steer_integral_t = now
            if dt is not None and not self._body_still():
                self.steer_integral -= self.steer_ki * steer_err * dt
                if self.steer_integral > self.steer_integral_max:
                    self.steer_integral = self.steer_integral_max
                elif self.steer_integral < -self.steer_integral_max:
                    self.steer_integral = -self.steer_integral_max
        # 基线上 ``F.apply_limits`` 的角速度恒为 0、转向全靠本块覆盖；
        # 保留该块为线速度来源，但转向输出**始终**取航向保持结果
        # （及下方停车归零），不能把整块丢给常量 0。
        self.last_steer = None if steer_err is None else {
            'err_deg': round(math.degrees(steer_err), 1),
            'cmd': round(steer, 3),
            'integ': round(self.steer_integral, 3),
            'imu_age': round(now - self.imu_last_t, 2) if
            self.imu_last_t is not None else None,
        }
        # 本拍最终命令（含航向保持转向）已定：据此给运动方向门定向，并
        # 原地改写同一份 motion——evaluate 结果、cmd_vel 门、/drive_safety
        # 共享这一份，不各自重算。文件顺序与基线的
        # _evaluate_safety → 发 cmd_vel 保持一致。
        result['angular'] = steer
        if 'motion' in result:
            result.update(self._command(result['motion'], result['linear'],
                                        result['angular']))
        if result['action'] in ('unknown', 'stop'):
            result['linear'] = 0.0
            result['angular'] = 0.0    # 停车时不打方向
            # [B3 PI] 停车/失智即弃积分：停车期间地面条件可能已变，
            # 陈旧积分会在恢复瞬间突甩方向，代价只是几秒重新学习。
            self.steer_integral = 0.0
            self._steer_integral_t = None
        # [B8] 先发本拍命令，再发许可：stop_stage 的 issued/zero_issued
        # 必须按"实际送出的命令"记录，因此 /drive_safety 的发布
        # （_evaluate_safety）必须排在发布分支之后，否则许可里的
        # stop_stage 永远是发布前的旧值（issued 恒 False）。
        enc_still = SS.encoders_still(self.last_encoders, now,
                                      self._encoders_t)
        zero_issued = True
        issued = False
        if self.enable_cmd_vel:
            twist = Twist()
            # 只有巡航打开才给速度。巡航关闭时仍发零速，
            # 让桥收到明确停车命令，避免沿用上一条非零命令。
            # [2026-11-17] 线速度 = min(cruise_linear, 计算限速)：
            # 旧实现恒发 cruise_linear，慢行区/贴障碍的限速被丢弃。
            # 本拍最终命令同时决定运动方向门的方向（motion_verdict），
            # 方向门拦停与 /drive_safety 拒绝读的是同一份 motion 结果。
            limit = min(self.cruise_linear, float(result['linear']))
            motion_ok = (self.cruise_active(now) and limit > 0.0
                         and not result['motion']['blocked'])
            twist.linear.x = limit if motion_ok else 0.0
            twist.angular.z = (result['motion_angular'] if motion_ok
                               else 0.0)
            zero_issued = (twist.linear.x == 0.0 and twist.angular.z == 0.0)
            issued = True
            self._last_cmd = (twist.linear.x, twist.angular.z)
            self.pub_cmd.publish(twist)
        # [B8] 三阶段停状态（enable_cmd_vel=False 时 issued=False：
        # "没发命令"不伪装成"已发停车命令"）。回传证据三态见
        # SS.encoders_still，与 guard_bridge.vehicle_stopped 同口径。
        result['stop_stage'] = SS.stop_stage_record(zero_issued, issued,
                                                    enc_still)
        self._stop_stage = result['stop_stage']
        self._evaluate_safety(now, result)
        self.publish_report(now, result)

    # ---------- 运动方向前置门（唯一判定点） ----------

    def raw_frame_age(self, now):
        """原始 ``header.stamp`` 帧龄（秒）；无帧或时间戳不可用返回 None。

        ``now`` 是节点时钟。``/scan`` 在雷达节点健康窗口内会把缓存圈
        整圈重发并写新 header 时间，因此这是"雷达声称的这一帧多新"，
        原始帧是否真的在流动仍由 ``scan_raw_evidence`` 证据链兜底。

        时间戳不可用（未填 / 全零 / 时钟域不符）返回 None，方向门据此
        判 ``stale``——不拿坏时间戳冒充新鲜，也与"刚发的一帧"区分开：
        ``age=None`` 交给 ``motion_check.obstacle_blocked`` 即拦停，
        方向门宁停不走。
        """
        msg = self.latest_scan
        if msg is None:
            return None
        stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
        return frame_age_from_stamp(stamp, now)

    def motion_verdict(self, msg=None, active=True, direction=None):
        """本拍的运动方向判定：自动命令、``/drive_safety``、页面状态共用。

        返回 ``motion_check.obstacle_blocked`` 的输出加上上下文
        （四扇区距离、帧龄、方向、安全距离）。一次 ``evaluate`` 只算一次，
        结果同时放进 ``result['motion']`` 与 ``/drive_safety``；页面与
        桥只读这两处，不各自重算。

        ``ranges`` 由 ``motion_check.sector_ranges`` 逐扇区判定（含量程、
        车体轮廓裁剪），这个有序循环在本地容器的整数比较不可信时也不
        受影响；方向门的最终 ``blocked/reason`` 以该原语输出为准。
        """
        now = self.now_s()
        full = msg is None or msg is self.latest_scan
        if msg is None:
            msg = self.latest_scan
        age = self.raw_frame_age(now) if full else None
        if msg is None:
            ranges = None
        else:
            # 四扇区 + 转弯一侧（左右取近者）一次算全。
            ranges = MC.sector_nearest_many(msg, MC.DIRECTIONS,
                                            self.body_geometry)
            ranges['side'] = MC.side_nearest(msg, self.body_geometry)
            if full:
                # 本拍先用纯扇区距离判一次、按最终命令定向；随后 _command
                # 会用原始帧重判（含扇区间隙）。原始帧只挂在内部缓存上，
                # 不进 /drive_safety（那里必须是 JSON 可序列化的）。
                ranges = dict(ranges, scan=msg)
        # 缓存命中：同一帧、同一生效状态、方向一致，才复用本拍已算过的
        # 结果（evaluate 与 _evaluate_safety、_command 不各算一次）。
        cached = (self._motion if full and self._motion is not None
                  and msg is getattr(self, '_motion_scan', None) else None)
        explicit = direction is not None
        dir_reason = 'explicit' if explicit else 'default'
        if cached is not None and not explicit and \
                not active and self._motion.get('reason') == 'inactive':
            return dict(self._motion)       # 无意图：不判方向
        if not explicit:
            if (cached is not None
                    and cached.get('direction_source') == 'explicit'):
                # 本拍已按最终命令定向：沿用，不能被默认方向覆盖。
                direction = cached['direction']
            elif self.cruise_active(now):
                # 巡航 = 自动前进意图：方向固定为前（会同时判两侧）。
                direction = self.motion_direction
            else:
                # 无自动运动意图时按"上一条实际命令"定向（转角命令里那一侧
                # 的近点会立刻让许可拒绝），不改变 active 语义。
                last = getattr(self, '_last_cmd', None) or (0.0, 0.0)
                direction = motion_direction_for(*last)
        if (cached is not None
                and direction == cached.get('direction')
                and active == cached.get('active')
                and self._motion_scan is msg):
            return dict(cached)
        if not self.motion_gate:
            # 方向门被节点参数显式关闭：明确标出来，不伪装成"已判过且空旷"。
            blocked, reason = False, 'gate_disabled'
        elif not active:
            # 无自动运动意图（巡航关闭）：不拦停，标出原因。
            blocked, reason = False, 'inactive'
        else:
            raw = ranges['scan'] if (isinstance(ranges, dict)
                                     and ranges.get('scan') is not None) \
                else None
            raw_dirs = {} if raw is None else MC.sector_nearest_many(
                raw, MC.DIRECTIONS, self.body_geometry)
            raw_side = None if raw is None else MC.side_nearest(
                raw, self.body_geometry)
            try:
                blocked, reason = obstacle_blocked_checked(
                    direction, ranges if raw is None else raw, age,
                    self.body_geometry, stop_distance=self.stop_d,
                    stale_s=DEFAULT_SCAN_STALE_S,
                    nearest=raw_dirs if raw is not None else None,
                    side=raw_side if raw is not None else None)
            except Exception as exc:                 # noqa: BLE001
                # 判定原语本身出错：按未知拦停，把原因带出去（fail-closed）。
                blocked, reason = True, 'motion_check_error:%r' % (exc,)
        verdict = {
            'blocked': bool(blocked),
            'reason': reason,
            'direction': direction,
            'direction_source': dir_reason,
            'active': bool(active),
            'ranges': (None if ranges is None
                       else {d: v for d, v in ranges.items() if d != 'scan'}),
            'age': None if age is None else round(age, 3),
            'stop_distance': self.stop_d,
            'geometry': dict(self.body_geometry),
            'stamp_age': age is not None,
            # 内部用：参与判定的原始帧（JSON 输出时经 ranges 白名单剔除）。
            'scan': msg if full else None,
        }
        if full:
            self._motion = verdict
            self._motion_scan = msg
        return verdict

    def us_stop_active(self, now):
        """超声波补充停车判定（供许可层调用）。

        直接转发纯逻辑门 ``UltrasonicGate.verdict``：超时/从未收到返回
        无约束（fail-open）；本判定只会让许可更严，不会解除雷达的任何
        停车结论。
        """
        return self.us_gate.verdict(now)

    def _evaluate_safety(self, now, result):
        """计算安全许可并发布 ``drive_safety``（每拍一次，denied 也发）。

        许可 = 原始帧证据在流动 ∧ 雷达可信 ∧ 避障动作不是 stop/unknown
        ∧ 超声波有效且不在补充停车 ∧ 运动方向门放行。任一无法确认都
        拒绝（全部 fail-closed；2026-11-17 起超声波无回波/断流也拒绝）。
        持续发布的原因：接收端按「许可过期 = 不允许运动」处理，停止
        发布本身就必须导致停车。

        运动方向门只读 ``evaluate()`` 本拍算出的 ``result['motion']``
        （同一帧、同一最终命令方向），不在这里另算一份。
        """
        ev = self.raw_evidence.status(now)
        trusted_ok, trusted_why = self._last_trust
        us_stop, us_range, us_ok, us_age = self.us_stop_active(now)
        motion = result.get('motion') or self._motion or self.motion_verdict()
        # 超声波输入无效（无回波/断流/从未收到）必须单独成原因：无效态
        # 下 verdict 同时返回 stop=True（fail-closed），不能因此被误报成
        # "近距障碍 ultrasonic_stop"，页面/桥也就不会把它读成"畅通"。
        us_deny = us_stop or not us_ok
        us_reason = 'ultrasonic_stop' if us_ok else 'ultrasonic_invalid'
        allowed, reason = SS.permission(
            ev['flowing'], ev['reason'], trusted_ok, trusted_why,
            result['action'], result['reason'],
            us_stop=us_deny, us_reason=us_reason,
            motion_blocked=motion['blocked'],
            motion_reason='motion_%s' % (motion['reason'],))
        self.safety_seq += 1
        payload = {
            'v': SS.PERMISSION_SCHEMA,
            'allowed': allowed,
            'reason': reason,
            'seq': self.safety_seq,
            'ttl_s': self.safety_ttl_s,
            'raw': {'flowing': ev['flowing'], 'age': ev['age'],
                    'epoch': ev['epoch'], 'frames_ok': ev['frames_ok'],
                    'regressions': ev['regressions'],
                    'max_point_age_ms': ev['max_point_age_ms']},
            # 超声波原始证据：排查"车为什么不动"时先看这里
            'ultrasonic': {'stop': us_stop, 'deny': us_deny, 'valid': us_ok,
                           'range_m': us_range,
                           'fault': self.us_gate.fault,
                           'fault_reason': self.us_gate.fault_reason,
                           'age': None if us_age is None
                           else round(max(0.0, us_age), 3),
                           'stop_distance': self.us_stop_d,
                           'release_distance': self.us_release_d,
                           'invalid_at': self.us_max_range,
                           'timeout_s': self.us_timeout},
            # 运动方向门（与自动命令同源）：页面/桥/nav relay 只读这里
            'motion': {'blocked': motion['blocked'],
                       'reason': motion['reason'],
                       'direction': motion['direction'],
                       'direction_source': motion.get('direction_source'),
                       'active': motion.get('active'),
                       'ranges': motion['ranges'],
                       'age': motion['age'],
                       'stop_distance': motion['stop_distance']},
            # [B8] 三阶段停状态：zero_issued=本拍是否发零速命令；
            # issued=命令是否实际送出（含发布通道关闭时为 False）；
            # vehicle_stopped=底盘回传证实停住（True/False/None）。
            # 三件事分开记录，与 guard_bridge.write_record 同一口径，
            # 页面从这一处即可判断车辆真实状态，不再各处各说各话。
            'stop_stage': result.get('stop_stage'),
        }
        msg = String(data=json.dumps(payload, ensure_ascii=False,
                                     separators=(',', ':')))
        self.pub_safety.publish(msg)
        result['safety_allowed'] = allowed
        result['safety_reason'] = reason
        result['raw_evidence'] = ev
        result['ultrasonic'] = payload['ultrasonic']
        result['motion'] = payload['motion']

    def _body_still(self):
        """[B3] 车身静止判定：编码器四轮都回 0。

        桥把瞬时轮速放诊断键 encoder_{a..d}_reported，轮停回 0。
        四键不全或从未收到诊断 → 未知，按"不在静止"处理（保守：
        宁可不刷新零偏，不可拿运动中的 gz 当零偏）。
        """
        enc = self.last_encoders
        if not enc or len(enc) < 4:
            return False
        try:
            return all(float(v) == 0.0 for v in enc.values())
        except (TypeError, ValueError):
            return False

    def _check_cruise_imu_timeout(self, now):
        if (self.cruise_on and not self._cruise_imu_timed_out
                and self.imu_last_t is not None
                and now - self.imu_last_t > self.imu_timeout):
            self._cruise_imu_timed_out = True
            self.get_logger().warning('IMU 超时，自动巡航已锁停，请关闭后重新开启')

    def on_imu(self, msg):
        """[B3] gz 积分航向 + 零偏维护（用户定则：使用前必须重置）。

        零偏只在"被证实静止"时刷新，三重条件缺一不可：
          1. 巡航关闭——行驶中的 gz 是真旋转，绝不能混进零偏；
          2. 四轮编码器全 0——车在动（含被手推）编码器会说话，
             比只看 gz 方差可靠（匀速旋转骗得过方差检查）；
          3. 窗口方差小 且 |均值| < 0.05 rad/s——停着的车零偏只可能
             是微小值，均值大说明车正被搬动/旋转，拒绝刷新。
        条件满足时用滚动窗口（20 样本 ≈1s）均值持续刷新——车什么时候
        静过，零偏就什么时候是新鲜的，顺带跟随温漂。零偏未收敛时按
        0 积分：积分是相对量，基准只在收敛后允许锁定，污染进不了
        控制回路。积分步长用头时间戳，跨步异常（≤0 或 >0.5s）跳过。
        """
        now = self.now_s()
        self._check_cruise_imu_timeout(now)
        gz = float(msg.angular_velocity.z)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        window = self._gz_window
        window.append(gz)
        if len(window) > 20:
            del window[0]
        if (not self.cruise_on and self._body_still()
                and len(window) >= 20):
            mean = sum(window) / len(window)
            var = sum((s - mean) ** 2 for s in window) / len(window)
            if var ** 0.5 < 0.01 and abs(mean) < 0.05:
                if self.imu_bias is None:
                    self.get_logger().info(
                        'IMU 零偏校准完成: %.4f rad/s' % mean)
                self.imu_bias = mean
        last = self._imu_stamp
        self._imu_stamp = t
        if last is not None and 0.0 < t - last < 0.5:
            bias = self.imu_bias if self.imu_bias is not None else 0.0
            self.imu_yaw += (gz - bias) * (t - last)
        self.imu_last_t = now

    def on_cruise(self, msg):
        """网页巡航开关回调。

        打开发速度，关闭发零速。开关带超时：网页若不持续发心跳，
        超时后按关闭处理，防止网页掉线后车一直前进。
        """
        was_on = self.cruise_on
        self.cruise_on = bool(msg.data)
        self.last_cruise_time = self.now_s()
        # [B3] 打开瞬间锁存当前 IMU 航向作为直线基准。只在关→开沿
        # 重锁：巡航中网页重复发 True 不打断基准。锁存前提（用户定则
        # 2026-11-15）：零偏已收敛且车身正被编码器证实静止——不在
        # 平衡态就不锁，宁可本次没有航向保持，不可带着脏零偏开车。
        if self.cruise_on and not was_on:
            self._cruise_imu_timed_out = False
            self._cruise_run_deadline = (
                monotonic() + self.cruise_max_run_s
                if self.cruise_max_run_s > 0.0 else None)
            # [B3 PI 2026-11-16] 重锁基准的同时积分清零——基准换了，
            # 旧积分对新基准毫无意义。
            self.steer_integral = 0.0
            self._steer_integral_t = None
            if self.imu_bias is not None and self._body_still():
                self.cruise_yaw_ref = self.imu_yaw
                self.get_logger().info(
                    '航向基准锁定: %.3f rad（零偏 %.4f，PI 积分清零）'
                    % (self.imu_yaw, self.imu_bias))
            else:
                self.cruise_yaw_ref = None
                self.get_logger().warning(
                    'IMU 未收敛或车身未静止，本次巡航不做航向保持')
        self.get_logger().info('自动巡航开关: %s' % ('开' if self.cruise_on
                                                   else '关'))

    def _cruise_run_expired(self):
        return (self._cruise_run_deadline is not None
                and monotonic() >= self._cruise_run_deadline)

    def cruise_active(self, now):
        """当前是否允许自动前进。

        cruise_timeout 检查最近一次心跳。cruise_max_run_s 从开启时起算，
        到期后必须关→开才能重新计时。两项为 0 时不限制运行时间。
        """
        if not self.cruise_on or self._cruise_imu_timed_out:
            return False
        if self._cruise_run_expired():
            return False
        if self.last_cruise_time is None:
            return False
        if self.cruise_timeout <= 0:
            return True
        return (now - self.last_cruise_time) <= self.cruise_timeout

    def image_state(self, now):
        if self.last_image_time is None:
            return 'unknown'
        if now - self.last_image_time > self.image_timeout:
            return 'stale'
        return 'fresh'

    def publish_report(self, now, result):
        cruise_stop_reason = ''
        if self.cruise_on:
            if self._cruise_imu_timed_out:
                cruise_stop_reason = 'imu_timeout'
            elif self._cruise_run_expired():
                cruise_stop_reason = 'run_limit'
        payload = {
            'action': result['action'],
            'reason': result['reason'],
            'front': result['front'],
            'left': result['left'],
            'right': result['right'],
            'linear': result['linear'],
            'angular': result['angular'],
            'scan_age': self.fresh.age(now),
            'lidar_state': self.lidar_state or 'unknown',
            'image': self.image_state(now),
            'cmd_vel_enabled': self.enable_cmd_vel,
            'cruise': self.cruise_active(now),
            'cruise_stop_reason': cruise_stop_reason,
            # 安全许可与原始帧证据状态：排查"车为什么不动"时先看这里
            'safety_allowed': result.get('safety_allowed'),
            'safety_reason': result.get('safety_reason'),
            'raw_evidence': result.get('raw_evidence'),
            'ultrasonic': result.get('ultrasonic'),
            # 与 /drive_safety.motion 同一份对象：页面不必重算
            'motion': result.get('motion'),
            # [B8] 三阶段停状态（与 /drive_safety.stop_stage 同一份）
            'stop_stage': result.get('stop_stage'),
            'body': {'half_length': self.body_geometry['half_length'],
                     'half_width': self.body_geometry['half_width'],
                     'center_x': self.body_geometry['center_x'],
                     'missing': MC.geometry_missing(self.body_geometry)},
            # [B3] 航向保持观测：err_deg=偏离基准角（正=需左转回正），
            # cmd=转向命令，imu_age=IMU 数据年龄。排查"巡航走不直"先看这。
            'steer': self.last_steer,
            # 视觉相位未实现，明确标未知，不伪造检测结果
            'vision': 'not_implemented',
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_report.publish(msg)

    def _decide_safe(self):
        """周期回调的异常屏障：出错只记日志，不让节点失去发布能力。"""
        try:
            self.decide()
        except Exception as exc:  # noqa: BLE001——周期回调必须吞掉异常
            self.get_logger().error('decide 失败: %r' % (exc,))

    def log_status(self):
        now = self.now_s()
        result = self.evaluate(now)
        # [B3] 航向保持追踪：err 停在死区附近=控制器没修到车（执行层
        # 吃掉了）；err≈0 而车仍漂=陀螺积分自身在漂（零偏残余）。
        # [B3 PI 2026-11-16] integ=积分项输出：稳态时 err→0、cmd≈integ，
        # integ 顶到 ±0.15 钳位=拉力超出积分能力，看数值即知。
        steer = self.last_steer
        steer_txt = ''
        if steer is not None:
            steer_txt = ' 航向: err=%+.2f° cmd=%+.3f integ=%+.3f' % (
                steer['err_deg'], steer['cmd'], steer.get('integ', 0.0))
        self.get_logger().info(
            '融合状态: %s action=%s reason=%s 图像=%s%s'
            % (F.describe(result['front'], result['left'], result['right'],
                          result['action']),
               result['action'], result['reason'], self.image_state(now),
               steer_txt))


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
