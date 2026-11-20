"""C50C 运动许可 + 全局锁停（纯逻辑，不依赖 ROS / rclpy）。

本模块决定"这一拍四个来源（manual / nav / auto / demo）里谁的命令可以
出去"；桥（``chassis_cmd/guard_bridge.py``）只按返回的
``(vx, vy, vz, source, zero_reason)`` 决定写速度帧还是零速帧。本模块不做
I/O，只用注入的时钟 ``now``（单调时钟秒），因此可在无 ROS 环境下测试。

规则（按优先级从上到下判定，命中即输出零速）：

1. **锁停**（``latched``）：一律四路零速并清缓存。启动、串口（重）连、
   人工停车、许可被拒、超声/雷达故障都会锁停。**解除只认显式**
   ``rearm(preconditions_ok=True)``；前提不满足或解停后下一拍前提再次
   不成立，立刻重新锁停（不保留任何自动解停路径）。
2. **等待 rearm**：已发出解停请求、尚未 rearm → ``rearm_pending`` 零速。
3. **许可**：``/motion_license`` 为 ``std_msgs/String`` JSON
   ``{"session": int>=1, "seq": int>=0, "expires_mono": float}``。
   ``seq`` 严格递增（重放/乱序拒绝）；``session`` 与已锁定代次不符即
   拒绝（含旧代次，且不推进序号）；``expires_mono <= now`` 拒绝。
   桥端 ``LICENSE_TIMEOUT_S``（初始设计值 0.5 s，可调）内没有有效续期即
   锁停；``expires_mono + LICENSE_EXPIRY_GRACE_S`` 都早于 now 视为过期。
4. **超声**：无回波被桥映射成 ``max_range``（``uart3_node.py``
   ``_publish_ultrasonic``），按无效读数处理；``>= 0.50 m``（地面回波/
   超量程）同样不采信；消息龄 > 0.3 s（断流）停车。手动与自动同等处理。
5. **雷达**：帧龄按原始 ``LaserScan.header.stamp`` 换算（``scan_stamp_to_mono``），
   > 0.3 s 视为陈旧。按**即将执行命令的运动方向**检查（``scan_check``）：
   前进查前扇区、后退查后扇区、转向查扫过区域（左转查左、右转查右）。
   角度换算复用 ``opi_robot.lidar_node.geometry`` 的 ``Sector`` /
   ``is_valid_range``。
6. **仲裁**：``PRIORITY``（manual > nav > auto > demo）选一条命令。
   手动只在 ``manual_hold`` 期间占优先；"普通松键"（``set_manual_hold(False)``）
   只结束手动，不锁停。

传感器/许可故障后读数恢复**不会**自动放行：用户先 ``request_rearm()``，
再显式 ``rearm(preconditions_ok=True)``。

边界（本模块不判定）：

- "车辆实际停住"需要底盘回传证据（编码器/速度），本模块不判定；三分离
  记录见 ``chassis_cmd/guard_bridge.py``。
- 激光在 ``base_link`` 中的安装方向由 geometry 的 LaserScan 角空间约定
  承担（车头 0°、左 +90°、右 -90°、后 180°，见 ``geometry.SECTOR_*``）。
"""

import json
import math
import time

try:  # 包内导入；测试时把 opi_robot 包放到 PYTHONPATH 上亦可。
    from opi_robot.lidar_node.geometry import SECTOR_LEFT, SECTOR_RIGHT, Sector, is_valid_range
except ImportError:  # pragma: no cover - 仅在布置缺包时触发
    raise ImportError(
        'motion_guard 需要 opi_robot.lidar_node.geometry 的 Sector/is_valid_range；'
        '请把 opi_robot 包放进 PYTHONPATH')


# ---------------------------------------------------------------- 常量
HEARTBEAT_PERIOD_S = 0.100      # 网页许可发送间隔（设计值）
LICENSE_TIMEOUT_S = 0.500       # 桥端许可续期中断即锁停（初始设计值，可调）
LICENSE_EXPIRY_GRACE_S = 0.050  # expires_mono 过期宽限（容忍发送到接收的延迟；设计值，可调）
SCAN_STALE_S = 0.300            # 雷达帧龄上限（按 header.stamp）
ULTRASONIC_STALE_S = 0.300      # 超声消息龄上限（断流）
ULTRASONIC_RELEASE_M = 0.50     # >= 该值不采信（地面回波/超量程）
COMMAND_FRESH_S = 1.0           # 单路命令龄上限（与既有 RosStraightInput 一致）
NAV_FRESH_S = 1.0               # nav 命令龄上限（沿用既有口径）
SCAN_MIN_POINTS = 1             # 扇区内至少几个有效点才放行
SECTOR_HALF_DEG = 30.0          # 方向扇区半角（与 geometry.SECTOR_FRONT 一致）
FORWARD_DIR_DEG = 0.0
REAR_DIR_DEG = 180.0
LEFT_DIR_DEG = SECTOR_LEFT.center_deg
RIGHT_DIR_DEG = SECTOR_RIGHT.center_deg

CHANNELS = ('manual', 'nav', 'auto', 'demo')
#: 优先级（左高右低）：手动是人工接管优先，nav 是外部程序，其次。
PRIORITY = ('manual', 'nav', 'auto', 'demo')

#: [B6] 故障类锁停原因：这些原因导致的锁停在传感器恢复、但未经显式
#: rearm 时，``decide`` 输出 ``rearm_required``（零速、锁停维持）。
_FAULT_CAUSES = frozenset((
    'ultrasonic_invalid', 'ultrasonic_stale', 'scan_stale', 'scan_blocked'))

ZERO_REASON = {
    'none': '',
    'global_stop': '全局锁停（启动/串口重连/人工停车，未 rearm）',
    'serial_reconnect': '串口重连默认锁停',
    'rearm_pending': '已请求解除锁停，等待显式 rearm',
    'rearm_required': '故障已恢复，等待显式 rearm',
    'license_expired': '许可续期中断或许可过期',
    'license_rejected': '许可消息无效（重放/旧代次/字段非法）',
    'ultrasonic_invalid': '超声无效或无回波',
    'ultrasonic_stale': '超声断流',
    'scan_stale': '雷达陈旧',
    'scan_blocked': '运动方向无有效点（雷达盲区）',
    'no_demand': '四路都没有可执行命令',
    'no_status_evidence': '缺少底盘回传证据（只允许零速帧）',
}

#: 锁停原因 → 说明。
STOP_CAUSE = {
    'global_stop': '启动/人工停车默认锁停',
    'serial_reconnect': '串口重连默认锁停',
    'rearm_pending': '等待显式 rearm',
    'rearm_required': '故障后等待显式 rearm',
    'license_expired': '许可续期中断或过期',
    'license_rejected': '许可被拒',
    'ultrasonic_invalid': '超声无效或无回波',
    'ultrasonic_stale': '超声断流',
    'scan_stale': '雷达陈旧',
    'scan_blocked': '运动方向受阻/无有效点',
}


def _finite(*vals):
    for v in vals:
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return False
        if not math.isfinite(float(v)):
            return False
    return True


def _int_or_none(v):
    """JSON 数值 → int；布尔/非整数/NaN/inf 返回 None。"""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    if isinstance(v, float):
        if not math.isfinite(v) or v != int(v):
            return None
        return int(v)
    return v


# ---------------------------------------------------------------- 许可
class License:
    """许可解析与"最近一条"状态。"""

    def __init__(self, *, timeout_s=LICENSE_TIMEOUT_S,
                 expiry_grace_s=LICENSE_EXPIRY_GRACE_S):
        self.timeout_s = float(timeout_s)
        self.expiry_grace_s = float(expiry_grace_s)
        self.session = None
        self.seq = None
        self.expires_mono = None
        self.received_at = None
        self.accepted = 0
        self.rejected = 0
        self.last_reject = ''

    @staticmethod
    def parse(raw):
        """``str``/``bytes`` JSON → dict；已是 dict 时原样返回；否则 ``None``。"""
        if isinstance(raw, dict):
            return raw
        try:
            payload = json.loads(raw)
        except (TypeError, ValueError):
            return None
        return payload if isinstance(payload, dict) else None

    def check(self, payload, now):
        """判定一条已解析许可：返回 ``(ok, session, seq, expires, reason)``。

        只读，不修改状态：被拒的消息不推进 ``seq``，重放仍然被拒。
        """
        if not isinstance(payload, dict):
            return False, None, None, None, 'malformed'
        session = _int_or_none(payload.get('session'))
        seq = _int_or_none(payload.get('seq'))
        exp_raw = payload.get('expires_mono')
        if session is None or session <= 0:
            return False, session, seq, None, 'session_invalid'
        if seq is None or seq < 0:
            return False, session, seq, None, 'seq_invalid'
        if isinstance(exp_raw, bool) or not isinstance(exp_raw, (int, float)) \
                or not math.isfinite(float(exp_raw)):
            return False, session, seq, None, 'expires_invalid'
        exp = float(exp_raw)
        if self.session is not None and session != self.session and self.fresh(now):
            return False, session, seq, exp, 'session_mismatch'
        if session == self.session and self.seq is not None and seq <= self.seq:
            return False, session, seq, exp, 'seq_replay'
        if exp + self.expiry_grace_s < now:
            return False, session, seq, exp, 'expired'
        return True, session, seq, exp, 'ok'

    def accept(self, payload, now):
        ok, session, seq, exp, reason = self.check(payload, now)
        if not ok:
            self.rejected += 1
            self.last_reject = reason
            return False, reason
        self.session = session
        self.seq = seq
        self.expires_mono = exp
        self.received_at = now
        self.accepted += 1
        return True, 'ok'

    def fresh(self, now):
        """续期未中断且未过期：``received_at`` 与 ``expires_mono`` 都不过期。"""
        if self.received_at is None or self.expires_mono is None:
            return False
        if now - self.received_at > self.timeout_s:
            return False
        return now <= self.expires_mono + self.expiry_grace_s

    def snapshot(self):
        return {
            'session': self.session,
            'seq': self.seq,
            'expires_mono': self.expires_mono,
            'received_at': self.received_at,
            'accepted': self.accepted,
            'rejected': self.rejected,
            'last_reject': self.last_reject,
        }


# ---------------------------------------------------------------- 超声
def ultrasonic_state(range_m, msg_age_s=None, *, stale_s=ULTRASONIC_STALE_S,
                     release_m=ULTRASONIC_RELEASE_M):
    """返回 ``(state, detail)``，state ∈ ``ok/invalid/stale``。

    - 消息龄 None/NaN/负/超过 ``stale_s``：``stale``（断流）。
    - 读数 None/NaN/inf/<=0/>= ``release_m``：``invalid``。无回波在桥上
      被映射成 ``max_range``（默认 4.5 m），因此落入 invalid。
    """
    if msg_age_s is None:
        return 'stale', 'no_sample'
    try:
        age = float(msg_age_s)
    except (TypeError, ValueError):
        return 'stale', 'age_unparsable'
    if age != age or age < 0.0 or age > stale_s:
        return 'stale', 'age=%s' % ('nan' if age != age else '%.3f' % age)
    if range_m is None:
        return 'invalid', 'no_range'
    try:
        r = float(range_m)
    except (TypeError, ValueError):
        return 'invalid', 'range_unparsable'
    if not math.isfinite(r):
        return 'invalid', 'no_echo'
    if r <= 0.0:
        return 'invalid', 'nonpositive'
    # [2026-11-18 用户定案] 无回波（桥把 0xFFFF 映射成 max_range）与超信任
    # 上限（地面回波）= "远处无约束"，不再当 invalid 锁车：超声波是 0.4m 级
    # 近距补充门，波束打空 ≠ 有障碍。近距停车仍由 release_m 以下的有效读数
    # 驱动迟滞；坏数据（NaN/<=0）仍判 invalid（fail-closed）。
    return 'ok', ''


# ---------------------------------------------------------------- 雷达
def direction_sectors(vx, vz):
    """按命令方向返回需检查的 LaserScan 角中心（度）；非有限值返回 None。

    已取负后的轮差速——``vz > 0`` = 顺时针右转查右 -90°；``vz < 0`` =
    左转查左 +90°（B4 用户实测定案：命令层 + = 顺时针，REP-103 反向）。
    guard 收到的就是这条命令层 vz，不得再按 REP-103 取反。
    ``vx > 0`` 查前 0°；``vx < 0`` 查后 180°。同时平移+转向时，**将要扫过
    的弧**（平移方向与转向侧之间的 45° 方向，例如前左 +45°）也算扫过区域。
    中心角按 ``Sector.contains``（``wrap_rad`` 口径）取值：右后为 -135°、
    左后为 +135°。
    """
    if not _finite(vx, vz):
        return None
    fwd = FORWARD_DIR_DEG if vx > 0.0 else (REAR_DIR_DEG if vx < 0.0 else None)
    side = RIGHT_DIR_DEG if vz > 0.0 else (LEFT_DIR_DEG if vz < 0.0 else None)
    centers = []
    if fwd is not None:
        centers.append(fwd)
    if side is not None:
        if fwd is not None:
            diag = (fwd + side) / 2.0
            if fwd == REAR_DIR_DEG and side < 0.0:
                diag -= 180.0            # 后右弧中点：+45° → 等效 -135°
            elif diag > REAR_DIR_DEG:    # 只有钝角（后侧 135°）需要等价换算
                diag -= 360.0
            elif diag <= -REAR_DIR_DEG:
                diag += 360.0
            centers.append(diag)
        centers.append(side)
    return centers


def _scan_angle(scan, i):
    """索引 i 的 LaserScan 角（与 geometry.ScanLayout.center_angle 同口径）。"""
    return (float(scan.angle_min) + i * float(scan.angle_increment)) % (2.0 * math.pi)


def sector_points(scan, center_deg, *, half_deg=SECTOR_HALF_DEG):
    """扇区内有效点距离列表；scan 缺字段/缺 ``angle_max`` 返回 ``None``。

    ``scan`` 为鸭子类型（``angle_min`` / ``angle_increment`` / ``ranges`` /
    ``range_min`` / ``range_max``，``angle_max`` 可选）。有效性由
    geometry.is_valid_range 判定（NaN/inf/超量程均无效）；``inf`` 在
    geometry 中同时满足 ``r <= range_max``，这里显式排除。
    """
    for name in ('angle_min', 'angle_increment', 'ranges'):
        if getattr(scan, name, None) is None:
            return None
    n = len(scan.ranges)
    amin = float(scan.angle_min)
    inc = float(scan.angle_increment)
    amax = getattr(scan, 'angle_max', None)
    if amax is None:  # 无 angle_max 时按索引推算
        amax = amin + inc * max(n - 1, 0)
    rmin = float(getattr(scan, 'range_min', 0.0))
    rmax = float(getattr(scan, 'range_max', float('inf')))
    sector = Sector(float(center_deg), float(half_deg))
    out = []
    for i, r in enumerate(scan.ranges):
        if not sector.contains(_scan_angle(scan, i)):
            continue
        try:
            rf = float(r)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(rf):
            continue
        if is_valid_range(rf, rmin, rmax):
            out.append(rf)
    return out


def _continuous_rad(amin, amax):
    """geometry.ScanLayout 同口径的"索引换算用"角连续体（不含粗粒度折行）。"""
    if amax < amin:
        amax += 2.0 * math.pi
    return amin, amax


def sector_coverage(scan, center_deg, *, half_deg=SECTOR_HALF_DEG):
    """扇区内可测束覆盖比例 ``(good, expected, spans)``；scan 不完整返回 ``None``。

    ``expected``：几何上应落进扇区的扫描索引数（``Sector.indices`` 口径）；
    ``good``：其中既在扇区又有效（有限、在量程内）的索引数。

    "有效点"= 有限距离且量程内——``inf``（无回波）与 NaN 都不算。对驱动
    本身（LD19 按反射强度丢弃弱回波）而言，"扇区里连一个有效点都没有"
    在无目标时同样成立，会误停车；因此桥默认用本函数与
    ``SCAN_MIN_COVERAGE`` 判"前方完全看不到东西"，``scan_check``（无有效
    点即停）仍作为可选的严格模式。
    """
    for name in ('angle_min', 'angle_increment', 'ranges'):
        if getattr(scan, name, None) is None:
            return None
    n = len(scan.ranges)
    if n == 0:
        return None
    amin = float(scan.angle_min)
    inc = float(scan.angle_increment)
    if inc == 0.0 or not math.isfinite(inc):
        return None
    amax_raw = getattr(scan, 'angle_max', None)
    amax = amin + inc * (n - 1) if amax_raw is None else float(amax_raw)
    amin_c, amax_c = _continuous_rad(amin, amax)
    if amax_c - amin_c >= 2.0 * math.pi:
        # 整圈（如 628 束 0.01 rad）：索引归属与 scan_check 的取模口径一致。
        angle_of = lambda i: (amin + i * inc) % (2.0 * math.pi)  # noqa: E731
    else:
        # 扫描弧 < 2π（如 LD19 分帧）：索引角不折行，否则跨越 0/2π 的束会
        # 被错误地算进对面扇区（geometry.ScanLayout 同理）。
        angle_of = lambda i: amin + i * inc  # noqa: E731
    layout = type('_Layout', (), {'n_bins': n, 'center_angle': staticmethod(angle_of)})
    spans = Sector(float(center_deg), float(half_deg)).indices(layout)
    if not spans:
        return None
    rmin = float(getattr(scan, 'range_min', 0.0))
    rmax = float(getattr(scan, 'range_max', float('inf')))
    good = 0
    for i in spans:
        try:
            rf = float(scan.ranges[i])
        except (TypeError, ValueError):
            continue
        if math.isfinite(rf) and is_valid_range(rf, rmin, rmax):
            good += 1
    return good, len(spans), (amin_c, amax_c)


def scan_check(scan, stamp_mono, now, vx, vz, *,
               stale_s=SCAN_STALE_S, half_deg=SECTOR_HALF_DEG,
               min_points=SCAN_MIN_POINTS):
    """按运动方向检查雷达（严格模式：扇区无有效点即阻塞）。

    返回 ``(ok, reason, detail)``，reason ∈ ``ok/scan_stale/scan_blocked``。
    ``stamp_mono`` 由桥用 ``scan_stamp_to_mono`` 从原始 ``header.stamp``
    换算。方向全零时只校验新鲜度。
    """
    if scan is None or stamp_mono is None:
        return False, 'scan_stale', 'no_scan'
    try:
        age = float(now) - float(stamp_mono)
    except (TypeError, ValueError):
        return False, 'scan_stale', 'stamp_unparsable'
    if age != age or age > stale_s:
        return False, 'scan_stale', 'age=%s' % ('nan' if age != age else '%.3f' % age)
    centers = direction_sectors(vx, vz)
    if centers is None:
        return False, 'scan_stale', 'non_finite_direction'
    for c in centers:
        pts = sector_points(scan, c, half_deg=half_deg)
        if pts is None:
            return False, 'scan_stale', 'scan_missing_fields'
        if len(pts) < min_points:
            return False, 'scan_blocked', 'sector_%+.0f_deg' % c
    return True, 'ok', ''


def motion_check(scan, stamp_mono, now, vx, vz, *,
                 stale_s=SCAN_STALE_S, half_deg=SECTOR_HALF_DEG,
                 min_coverage=0.0, strict=True):
    """按运动方向检查雷达：``(ok, reason, detail)``。

    - 帧龄（``now - stamp_mono``）> ``stale_s`` → ``scan_stale``；
    - ``strict=True``（**默认**，任务字面口径）：等价于 :func:`scan_check`，
      运动方向扇区内没有有效点就阻塞（``scan_blocked``）；
    - ``strict=False``（可选的宽松模式）：用 :func:`sector_coverage`，只有
      覆盖率 < ``min_coverage`` 时才阻塞（默认 0，即"连一个有效点都没有"才停）。
    """
    if strict:
        return scan_check(scan, stamp_mono, now, vx, vz, stale_s=stale_s,
                          half_deg=half_deg)
    if scan is None or stamp_mono is None:
        return False, 'scan_stale', 'no_scan'
    try:
        age = float(now) - float(stamp_mono)
    except (TypeError, ValueError):
        return False, 'scan_stale', 'stamp_unparsable'
    if age != age or age > stale_s:
        return False, 'scan_stale', 'age=%s' % ('nan' if age != age else '%.3f' % age)
    centers = direction_sectors(vx, vz)
    if centers is None:
        return False, 'scan_stale', 'non_finite_direction'
    for c in centers:
        cov = sector_coverage(scan, c, half_deg=half_deg)
        if cov is None:
            return False, 'scan_stale', 'scan_missing_fields'
        good, expected, _spans = cov
        if good == 0 or (good / float(expected)) < float(min_coverage):
            return False, 'scan_blocked', ('sector_%+.0f_deg good=%d/%d'
                                           % (c, good, expected))
    return True, 'ok', ''


def scan_stamp_to_mono(stamp, *, now_mono, now_wall):
    """把 ``header.stamp``（sec/nanosec）换算成单调时钟秒。

    ``now_mono`` 与 ``now_wall`` 在同一时刻采样。未填（0）的时间戳返回
    ``None``——没有帧龄证据就按陈旧处理。

    边界（**未实测**）：本换算以"``header.stamp`` 与墙钟同源"为前提，
    只对 ``RCL_SYSTEM_TIME``（墙钟）成立。节点若用 ``RCL_STEADY_TIME``
    打戳，stamp 与墙钟不同源，换算结果无意义；桥不读取节点的时钟源，
    因此这里既不断言也不自动纠正——接线方必须在节点上用 SYSTEM_TIME 打
    雷达 stamp，否则帧龄判定不可信（参见 ``guard_bridge`` 的接线说明）。
    """
    sec = getattr(stamp, 'sec', None)
    nsec = getattr(stamp, 'nanosec', None)
    if sec is None or nsec is None:
        return None
    if float(sec) == 0.0 and float(nsec) == 0.0:
        return None
    stamp_wall = float(sec) + float(nsec) * 1e-9
    return float(now_mono) - (float(now_wall) - stamp_wall)


# ---------------------------------------------------------------- 主类
class MotionGuard:
    """四路命令的许可与锁停裁决器（调用方保证串行调用）。"""

    def __init__(self, now=None, *, license_timeout_s=LICENSE_TIMEOUT_S,
                 license_expiry_grace_s=LICENSE_EXPIRY_GRACE_S,
                 scan_stale_s=SCAN_STALE_S,
                 ultrasonic_stale_s=ULTRASONIC_STALE_S,
                 ultrasonic_release_m=ULTRASONIC_RELEASE_M,
                 command_fresh_s=COMMAND_FRESH_S, nav_fresh_s=NAV_FRESH_S,
                 scan_min_points=SCAN_MIN_POINTS, scan_min_coverage=0.0,
                 scan_strict=True,
                 half_deg=SECTOR_HALF_DEG):
        self._now = now or time.monotonic
        self.license_timeout_s = float(license_timeout_s)
        self.scan_stale_s = float(scan_stale_s)
        self.ultrasonic_stale_s = float(ultrasonic_stale_s)
        self.ultrasonic_release_m = float(ultrasonic_release_m)
        self.command_fresh_s = float(command_fresh_s)
        self.nav_fresh_s = float(nav_fresh_s)
        self.scan_min_points = int(scan_min_points)
        self.scan_min_coverage = float(scan_min_coverage)
        self.scan_strict = bool(scan_strict)
        self.half_deg = float(half_deg)
        self.license = License(timeout_s=self.license_timeout_s,
                               expiry_grace_s=license_expiry_grace_s)
        self._cmds = {ch: (0.0, 0.0, 0.0) for ch in CHANNELS}
        self._cmd_at = {ch: None for ch in CHANNELS}
        self._manual_hold = False
        self._stop_cause = 'global_stop'
        self._latched = True             # 启动默认锁停
        self._rearm_pending = False
        self._last_scan = (None, None)   # (scan, stamp_mono)
        self._last_fault = ''
        self._events = []
        self.rearms = 0
        self.rearm_refused = 0

    # ------------------------------------------------------------ 命令
    def on_command(self, channel, vx, vy, vz):
        """接收一路命令。非有限值清该路缓存并返回 False。"""
        if channel not in self._cmds:
            raise KeyError(channel)
        now = self._now()
        if not _finite(vx, vy, vz):
            self._cmds[channel] = (0.0, 0.0, 0.0)
            self._cmd_at[channel] = now
            return False
        self._cmds[channel] = (float(vx), float(vy), float(vz))
        self._cmd_at[channel] = now
        return True

    def set_manual_hold(self, holding):
        """手动输入是否持续（松键即 False）。只影响仲裁，不锁停。"""
        self._manual_hold = bool(holding)

    @property
    def manual_hold(self):
        return self._manual_hold

    def command_age(self, channel):
        at = self._cmd_at.get(channel)
        return float('inf') if at is None else self._now() - at

    def cached(self, channel):
        return list(self._cmds[channel])

    # ------------------------------------------------------------ 许可
    def accept_license_json(self, raw):
        """接收 ``/motion_license`` 原文，返回 ``(ok, reason)``。"""
        return self.accept_license(License.parse(raw))

    def accept_license(self, payload):
        """接收一条许可。

        - 被拒：结构/代次/序号/期限字段非法 → 立即锁停 ``license_rejected``；
          重放与"这条本身已过期"不改变当前许可仍然有效的事实（由
          ``License.fresh`` 自然到期锁停）。
        - 被接受**不**解除任何锁停：解停只认显式 ``rearm``。
        """
        now = self._now()
        previous_session = self.license.session
        ok, reason = self.license.accept(payload, now)
        if not ok:
            self._event('license_rejected', reason)
            if reason in ('malformed', 'session_invalid', 'seq_invalid',
                          'expires_invalid', 'session_mismatch'):
                self._latch('license_rejected')
            return False, reason
        if previous_session is not None and self.license.session != previous_session:
            # 会话接管只能在旧许可失效后发生；清旧命令并要求显式解停。
            self._latch(self._stop_cause if self._latched else 'license_expired')
        self._event('license_ok', self.license.seq)
        return True, 'ok'

    # ------------------------------------------------------------ 锁停
    def _latch(self, cause):
        was = self._latched
        if not was:
            self._event('latch', cause)
        self._latched = True
        self._stop_cause = cause or self._stop_cause
        self._rearm_pending = False
        self.clear_commands()

    def stop(self, cause='global_stop'):
        """全局锁停：四路零速并清缓存，直到显式 rearm。"""
        self._event('stop', cause)
        self._latched = True
        self._stop_cause = cause or 'global_stop'
        self._rearm_pending = False
        self.clear_commands()

    def rearm(self, preconditions_ok=False):
        """显式 rearm，返回 ``(ok, reason)``。

        默认拒绝：``preconditions_ok`` 必须由调用方（网页会话鉴权 + 用户
        明确动作）显式给出。前提满足只解除锁停；许可/传感器状态不变，
        ``decide`` 下一拍按实际状态重新判定（仍无效则立刻重新锁停）。
        """
        if preconditions_ok is not True:
            self.rearm_refused += 1
            self._event('rearm_refused', 'preconditions')
            return False, 'preconditions'
        self._latched = False
        self._stop_cause = ''
        self._rearm_pending = False
        self.rearms += 1
        self.clear_commands()
        self._event('rearm', 'ok')
        return True, 'ok'

    def request_rearm(self):
        """网页"请求解除锁停"：只登记意图，**不**解除任何锁停。

        登记后 ``decide`` 输出 ``rearm_pending`` 零速，直到显式
        ``rearm(preconditions_ok=True)``。
        """
        if self._latched:
            self._rearm_pending = True
            self._event('rearm_request', self._stop_cause)
        return self._rearm_pending

    @property
    def rearm_pending(self):
        return self._rearm_pending

    @property
    def stopped(self):
        return self._latched

    @property
    def stop_cause(self):
        return self._stop_cause if self._latched else ''

    def clear_commands(self):
        for ch in CHANNELS:
            self._cmds[ch] = (0.0, 0.0, 0.0)
            self._cmd_at[ch] = None
        # 只清命令缓存；manual_hold 由输入侧（松键/按下）单独维护，不在锁停
        # 时顺带改写——否则松键与锁停两个事件无法区分。

    # ------------------------------------------------------------ 传感器
    def on_scan(self, scan, stamp_mono):
        """记录最近一帧雷达（桥按 header.stamp 换算 stamp_mono）。"""
        self._last_scan = (scan, stamp_mono)

    def scan_for_direction(self, vx, vz):
        scan, stamp = self._last_scan
        return motion_check(scan, stamp, self._now(), vx, vz,
                            stale_s=self.scan_stale_s, half_deg=self.half_deg,
                            min_coverage=self.scan_min_coverage,
                            strict=self.scan_strict)

    # ------------------------------------------------------------ 仲裁
    def _pick(self, now):
        """按 PRIORITY（manual > nav > auto > demo）选一条可执行命令。

        - 手动只在 ``manual_hold`` 期间占优先；松键后回落，不锁停。
        - 返回 ``(channel, cmd)``，没有可执行命令时返回 ``None``。
        """
        order = [ch for ch in PRIORITY
                 if not (ch == 'manual' and not self._manual_hold)]
        for ch in order:
            at = self._cmd_at[ch]
            if at is None:
                continue
            limit = self.nav_fresh_s if ch == 'nav' else self.command_fresh_s
            if (now - at) > limit:
                continue
            cmd = self._cmds[ch]
            if any(abs(v) > 0.0 for v in cmd):
                return ch, cmd
        return None

    def _zero(self, reason):
        self.clear_commands()
        return (0.0, 0.0, 0.0, None, reason)

    def _reject_scan(self, why):
        reason = 'scan_stale' if why == 'scan_stale' else 'scan_blocked'
        self._last_fault = reason
        self._latch(reason)
        return self._zero(reason)

    def decide(self, *, ultrasonic='ok'):
        """一拍输出：``(vx, vy, vz, source, zero_reason)``。

        - 锁停或许可失效：四路零速并清缓存。
        - 超声 ``invalid``/``stale`` 或雷达按方向不 ok：锁停并清缓存
          （手动与自动同等）。``ultrasonic`` 缺省 ``'ok'``；调用方没有
          传感器证据时必须显式传 ``'invalid'``/``'stale'``，
          :meth:`decide_step` 负责这一步。
        - 否则按 PRIORITY 选命令，再按**该命令的运动方向**检查雷达；
          方向不 ok 同样锁停。
        - 没有来源持有非零命令时输出零速 ``no_demand``（不锁停）。
        """
        now = self._now()
        if self._latched:
            if self._rearm_pending:
                reason = 'rearm_pending'
            elif self._stop_cause in _FAULT_CAUSES:
                # [B6] 故障类锁停：传感器恢复与否以"显式 rearm"为唯一解停
                # 通道。恢复后不再重复报已消失的旧故障原因（那会让上层把
                # 一次恢复误读为"故障仍在持续"），改报 rearm_required：
                # 仍是零速、锁停维持；真实原因保留在 stop_cause/snapshot。
                reason = 'rearm_required'
            else:
                reason = self._stop_cause or 'global_stop'
            return self._zero(reason)
        if not self.license.fresh(now):
            self._last_fault = 'license_expired'
            self._latch('license_expired')
            return self._zero('license_expired')
        if ultrasonic == 'stale':
            self._last_fault = 'ultrasonic_stale'
            self._latch('ultrasonic_stale')
            return self._zero('ultrasonic_stale')
        if ultrasonic != 'ok':
            self._last_fault = 'ultrasonic_invalid'
            self._latch('ultrasonic_invalid')
            return self._zero('ultrasonic_invalid')
        if self._rearm_pending:
            # 已请求解停、但 `_latched` 因显式 rearm 已解除：不输出非零，也不
            # 把原因当故障（锁停与否以 `_latched` 为准）。许可/超声/雷达故障
            # 反而优先：它们会让 `_latch` 重新置位，下一拍就是权威原因。
            return self._zero('rearm_pending')
        picked = self._pick(now)
        if picked is None:
            # 四路都没有可执行命令：只校验雷达新鲜度（零速帧不需要方向扇区）。
            ok, why, _detail = self.scan_for_direction(0.0, 0.0)
            if not ok:
                return self._reject_scan(why)
            return self._zero('no_demand')
        ch, (vx, vy, vz) = picked
        ok, why, _detail = self.scan_for_direction(vx, vz)
        if not ok:
            return self._reject_scan(why)
        return (vx, vy, vz, ch, '')

    def decide_step(self, *, ultrasonic_range=None, ultrasonic_age_s=None,
                    scan=None, scan_stamp_mono=None):
        """桥每拍入口：喂传感器后 ``decide``。

        没有超声样本（参数缺省）按 ``stale`` 处理 → 锁停，避免无订阅时
        误放行。
        """
        if scan is not None or scan_stamp_mono is not None:
            self.on_scan(scan, scan_stamp_mono)
        state, _ = ultrasonic_state(ultrasonic_range, ultrasonic_age_s,
                                    stale_s=self.ultrasonic_stale_s,
                                    release_m=self.ultrasonic_release_m)
        return self.decide(ultrasonic=state)

    # ------------------------------------------------------------ 诊断
    def _event(self, kind, detail):
        self._events.append((round(self._now(), 3), kind, detail))
        if len(self._events) > 200:
            del self._events[:100]

    def snapshot(self):
        return {
            'latched': self._latched,
            'stop_cause': self.stop_cause,
            'stop_text': STOP_CAUSE.get(self._stop_cause, '') if self._latched else '',
            'rearm_pending': self._rearm_pending,
            'last_fault': self._last_fault,
            'rearms': self.rearms,
            'rearm_refused': self.rearm_refused,
            'manual_hold': self._manual_hold,
            'license': self.license.snapshot(),
            'commands': {ch: list(self._cmds[ch]) for ch in CHANNELS},
            'events': list(self._events[-10:]),
        }


__all__ = [
    'CHANNELS', 'COMMAND_FRESH_S', 'FORWARD_DIR_DEG', 'HEARTBEAT_PERIOD_S',
    'LEFT_DIR_DEG', 'LICENSE_EXPIRY_GRACE_S', 'LICENSE_TIMEOUT_S', 'License',
    'MotionGuard', 'NAV_FRESH_S', 'PRIORITY', 'REAR_DIR_DEG',
    'RIGHT_DIR_DEG', 'SCAN_MIN_POINTS', 'SCAN_STALE_S', 'SECTOR_HALF_DEG',
    'STOP_CAUSE', 'ULTRASONIC_RELEASE_M', 'ULTRASONIC_STALE_S', 'ZERO_REASON',
    'direction_sectors', 'motion_check', 'scan_check', 'scan_stamp_to_mono',
    'sector_coverage', 'sector_points', 'ultrasonic_state',
]
