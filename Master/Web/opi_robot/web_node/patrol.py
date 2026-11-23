"""网页航点巡航的纯逻辑：路线保存、地图绑定、一次一个目标、全局停止与锁停。

不依赖 ROS，也不依赖 web_debug：``web_debug`` 与离线测试共用本模块；
真正发目标仍是 :class:`opi_robot.web_node.nav_goal.NavGoalControl`
（``NavigateToPose`` 一次一个）。

参考实现：``c50c-auto-nav-20261117/patrol_logic.py``；在其上增加本模块
需要的三处（计划 §3/§4）：

- 路线绑定地图内容标识（地图 yaml/pgm 的 sha256 前 16 位）；地图变了拒绝
  旧路线（``validate_route`` / ``PatrolRunner.start`` 每次启动都复查）。
- 迟到的接受必须能被发现：取消请求**不论当时是否失败**都记录成
  ``cancel_request``，接受回调到达时立即再取消；结果状态为 CANCELED 才
  清掉记录。取消失败 / 结果未确认 → 保持锁停，不认为"已停"。
- 手动接管先取消旧导航；松键/旧任务成功返回都不得自行恢复
  （``_manual_override`` 一直保持到用户再次 ``start``）。

安全边界：巡航失败、规划 3 s 无安全路径、导航结果不成功、网页断线、
网页许可（``/motion_license``）超时——一律经过 :meth:`PatrolRunner.lock_stop`
请求全局锁停。锁停由下游（本 runner 的只读状态 + 底盘桥的许可超时）独立
保持，不依赖网页进程。
"""

import hashlib
import math
import os
import threading
import time


MIN_WAYPOINTS = 2
MIN_SPACING_M = 0.30
MAX_WAYPOINTS = 32
# 规划连续多久没有安全路径/合法控制轨迹就取消并锁停。
PLAN_BLOCK_S = 3.0

STATUS = ('idle', 'running', 'holding_goal', 'cancelling', 'stopped', 'failed')

# 网页许可：前端每 100 ms 发一条；服务端 350 ms 没有新许可即判会话失效。
LICENSE_PERIOD_S = 0.100
SESSION_TIMEOUT_S = 0.350
# 许可消息自带的期限：超过即不再续期（防止一条旧消息被反复使用）。
LICENSE_GRANT_S = 1.0
# 下游底盘桥在许可断供多久后锁停（与 motion_guard.LICENSE_TIMEOUT_S 同值）。
LICENSE_TIMEOUT_S = 0.500

LOCK_REASON_TEXT = {
    'global_stop': '用户全局停车',
    'no_safe_path': '规划连续 3 秒无安全路径',
    'nav_failed': '导航失败',
    'nav_rejected': '导航目标被拒绝',
    'dispatch_failed': '目标发送失败',
    'goal_timeout': '目标迟迟未被接受',
    'web_disconnected': '网页断线',
    'license_lost': '网页许可中断',
    'manual_takeover': '手动接管',
    'map_changed': '地图已更换，请重新保存路线',
}

# ---------------------------------------------------------------- guard 口径
# B5（reviews/tree-r1-reviewer.md）：patrol 的锁停原因与 guard 的
# ZERO_REASON/STOP_CAUSE 不是一套枚举。下游底盘桥（c50c_uart3
# guard_bridge.WriteRecord.stop_cause）只认 guard 枚举里的字面；本映射是
# patrol 原因 → guard stop_cause 的唯一权威转换：
#   - license_lost 本质是"许可续期中断"→ guard 的 license_expired；
#   - 其余 patrol 侧的显式停车（用户停车/断线/接管/导航流程失败）对下游
#     语义相同：撤回鉴权并全局锁停，等下一次显式 rearm → global_stop。
# 下游不猜原因文本；页面显示仍用 LOCK_REASON_TEXT 的 patrol 原文。
PATROL_TO_GUARD_CAUSE = {
    'license_lost': 'license_expired',
}
# guard 枚举（只做子集校验的字符串集合；c50c_uart3 为独立包，这里不 import）。
GUARD_STOP_CAUSES = frozenset((
    'global_stop', 'serial_reconnect', 'rearm_pending', 'rearm_required',
    'license_expired', 'license_rejected', 'ultrasonic_invalid',
    'ultrasonic_stale', 'scan_stale', 'scan_blocked', 'no_status_evidence',
))


def guard_stop_cause(reason):
    """patrol 锁停原因 → guard ``stop_cause`` 枚举（未知原因兜底 global_stop）。"""
    cause = PATROL_TO_GUARD_CAUSE.get(reason, 'global_stop')
    if cause not in GUARD_STOP_CAUSES:     # 防御：映射表写错也不出枚举
        cause = 'global_stop'
    return cause


# ---------------------------------------------------------------- B6 口径
# B6（reviews/tree-r1-reviewer.md:81-86）：guard 枚举里的 ``rearm_required``
# （故障已恢复、等待显式 rearm）与 ``no_status_evidence``（缺底盘回传证据）
# 在 guard/桥一侧"声明了但没人产生"，网页只能看到 lock_stop / license_lost
# 两档，无法回答"要不要人按恢复"。网页侧在 /patrol_status 里给出
# ``rearm_state`` 展示口径（只转述，不替代下游判定；下游 guard 侧何时真正
# 产出这两个字面见 reviews/b6-rearm-noevidence-fix.md 的 open 项）：
#   - 'rearm_required'：许可有效 + 锁停未解除 + 用户已点过恢复（本进程
#     rearm 成功）但下游仍锁停——下游在等显式解停请求/鉴权；
#   - 'rearm_pending'：锁停未解除、用户尚未点恢复——等用户动作；
#   - 'not_required'：锁停已解除，无需恢复。
GUARD_REARM_STATES = frozenset(
    ('rearm_required', 'rearm_pending', 'not_required'))


def rearm_state(locked, licensed, released, pending_cancel=None):
    """``/patrol_status`` 的恢复状态口径（B6）。只做展示转述。"""
    if not locked:
        state = 'not_required'
    elif not licensed:
        # 许可不新鲜：先恢复许可，恢复按钮此时必然失败（patrol.rearm）。
        state = 'rearm_pending'
    elif pending_cancel is not None:
        state = 'rearm_required'           # 有待确认取消：许可在、锁未解
    elif released:
        state = 'not_required'
    else:
        # 许可有效 + 锁停未解除：可按恢复，但用户还没按（按过仍锁停的
        # 情况由 pending_cancel 分支覆盖）。
        state = 'rearm_required'
    if state not in GUARD_REARM_STATES:    # 防御：字面写错也不出枚举
        state = 'rearm_pending'
    return state


# ---------------------------------------------------------------- 地图标识
def map_id_from_digest(digest):
    """地图内容标识：sha256 十六进制前 16 位。"""
    if not isinstance(digest, str) or len(digest) < 16:
        raise ValueError('需要完整 sha256')
    return digest[:16].lower()


def digest_files(paths):
    """对一组地图文件求内容摘要（按给定顺序拼接字节再哈希）。

    返回十六进制摘要；任一文件不可读时抛 ``OSError``。文件名不参与
    摘要——改名不算换图，改内容才算（计划 §3：不能"同名不同内容"）。
    """
    h = hashlib.sha256()
    for path in paths:
        with open(path, 'rb') as fp:
            for chunk in iter(lambda: fp.read(1 << 16), b''):
                h.update(chunk)
        # 分隔符防止两个文件首尾拼接后与另一组文件冲突
        h.update(b'\x00')
    return h.hexdigest()


def map_id_from_files(paths):
    """地图文件 → 16 位内容标识。"""
    return map_id_from_digest(digest_files(paths))


_YAML_KEYS = ('image', 'yaml', 'map', 'file', 'grid', 'pgm', 'data')


def map_files_from_yaml(yaml_path):
    """从地图 yaml 找栅格文件；找不到时退回同目录同名 .pgm。"""
    out = [yaml_path]
    image = None
    try:
        with open(yaml_path, 'r', encoding='utf-8') as fp:
            for line in fp:
                key, sep, rest = line.partition(':')
                if sep and key.strip().lower() in _YAML_KEYS:
                    value = rest.strip().strip('"\'')
                    if value:
                        image = value
                        break
    except OSError:
        raise
    base = os.path.dirname(os.path.abspath(yaml_path))
    if image:
        cand = image if os.path.isabs(image) else os.path.join(base, image)
        if os.path.exists(cand):
            out.append(cand)
            return tuple(out)
    stem = os.path.splitext(yaml_path)[0]
    for ext in ('.pgm', '.png'):
        cand = stem + ext
        if os.path.exists(cand):
            out.append(cand)
            break
    return tuple(out)


# ---------------------------------------------------------------- 几何
def wrap(a):
    r = math.fmod(a, 2.0 * math.pi)
    if r > math.pi:
        r -= 2.0 * math.pi
    elif r <= -math.pi:
        r += 2.0 * math.pi
    return r


def facing_next(points):
    """每个航点的 yaw = 从该点指向下一点；末点指向首点（闭环）。"""
    n = len(points)
    out = []
    for i, (x, y) in enumerate(points):
        nx, ny = points[(i + 1) % n]
        if (nx, ny) == (x, y):
            yaw = 0.0
        else:
            yaw = math.atan2(ny - y, nx - x)
        out.append((x, y, yaw))
    return out


def validate_route(raw_points, map_id, current_map_id):
    """校验并规范化一条路线。返回 ``(route, error)``。

    ``raw_points``：``[[x, y], ...]``。``map_id`` 是保存方声明的地图标识，
    与当前地图标识不一致即拒绝。
    """
    if not map_id or not current_map_id or map_id != current_map_id:
        return None, '地图已更换，旧路线不可用'
    if not isinstance(raw_points, list) or not all(
            isinstance(p, (list, tuple)) and len(p) >= 2 for p in raw_points):
        return None, '航点格式不正确'
    pts = []
    for p in raw_points:
        try:
            x, y = float(p[0]), float(p[1])
        except (TypeError, ValueError):
            return None, '航点坐标不是数字'
        if not (math.isfinite(x) and math.isfinite(y)):
            return None, '航点坐标非有限值'
        pts.append((x, y))
    if len(pts) > MAX_WAYPOINTS:
        return None, '航点不能超过 %d 个' % MAX_WAYPOINTS
    if len(pts) < MIN_WAYPOINTS:
        return None, '至少需要 %d 个航点' % MIN_WAYPOINTS
    for i in range(len(pts)):
        j = (i + 1) % len(pts)
        d = math.hypot(pts[j][0] - pts[i][0], pts[j][1] - pts[i][1])
        if d < MIN_SPACING_M:
            return None, ('第 %d 与第 %d 个航点重合（<%.2f m）'
                          % (i + 1, j + 1, MIN_SPACING_M))
    return {'map_id': map_id, 'points': [list(p) for p in facing_next(pts)]}, None


# ---------------------------------------------------------------- 巡航状态机
class PatrolRunner:
    """一次一个目标的巡航状态机。

    外部依赖（构造时注入，便于离线测试竞态）：

    - ``dispatch(goal)`` → 请求编号；``goal`` 为 ``{'x','y','yaw'}``。
    - ``cancel(request_id, result_known)`` → bool：结果已知时按句柄取消，
      结果未知（接受前的迟到目标）时按 seq 取消。返回是否已发出取消。
    - ``lock_stop(reason)``：请求全局锁停（下游独立保持）。
    - ``now()``：单调时钟。

    本类只决定"下一步做什么"；发目标、取消、发布状态都由 web_debug 做。
    """

    def __init__(self, dispatch, cancel, lock_stop, now):
        self._dispatch = dispatch
        self._cancel = cancel
        self._lock_stop = lock_stop
        self._now = now
        self.route = None
        self.state = 'idle'
        self.index = 0
        self.laps = 0
        self.stop_reason = ''
        # 启动即锁停：与 motion_guard 一致，需明确 rearm（计划 §1）。
        self.locked = True
        self.lock_reason = 'global_stop'
        self.pending = None            # 已发出、已确认接受的请求编号
        self.sending = None            # 已发出、尚未确认接受
        self.sending_since = None
        self.last_rid = None           # 最近一次发出的请求编号（迟到的接受据此识别）
        self.cancel_request = None     # 停车后仍未确认取消的目标
        self.plan_blocked_since = None
        self._manual_override = False
        self._lock_by = None           # 锁停来源标记：'manual'=手动接管的防误判锁
        self._cancel_busy = set()

    # -- 用户动作 ------------------------------------------------------
    def save(self, route):
        self.route = route

    def start(self, current_map_id):
        """开始巡航：从第一个航点起。地图变了拒绝旧路线。"""
        if self.route is None:
            return False, '没有已保存的路线'
        if self.route.get('map_id') != current_map_id:
            self.lock_stop('global_stop')      # 锁停状态保持，不因为换图而放开
            self.state = 'stopped'
            self.stop_reason = 'map_changed'
            return False, '地图已更换，请重新保存路线'
        if self.cancel_request is not None:
            if not self._cancel_now(self.cancel_request, result_known=False):
                # 上一个迟到的目标还没确认取消：不允许在上面叠新目标。
                return False, '上一个目标尚未确认取消，请稍后重试'
        self.cancel_request = None
        self.state = 'running'
        self.index = 0
        self.laps = 0
        self.stop_reason = ''
        self.locked = False           # rearm 已校验许可：开始了就不是锁停态
        self.lock_reason = ''
        self.pending = None
        self.sending = None
        self.sending_since = None
        self.plan_blocked_since = None
        self.cancel_confirmed = None
        ok = self._issue()
        if not ok:
            return False, '目标发送失败'
        return True, 'ok'

    def _set_locked(self, reason):
        """写锁停原因并通知下游（不经 lock_stop 的"保留首因"规则）。"""
        self.locked = True
        self.lock_reason = reason or 'global_stop'
        try:
            self._lock_stop(self.lock_reason)
        except Exception:          # noqa: BLE001
            pass

    def stop(self, reason='global_stop', user_action=False):
        """全局停车：取消在途/待接受目标并锁停。

        ``user_action=True``：用户/会话显式发起的停车（全局停止按钮、断线、
        许可失效），原因取这一次的动作；否则（内部流程触发的停车）已经锁停
        时保留首个更真实的原因（派发失败/导航失败/超时）。
        取消请求无论当时成败都记下来；目标迟到被接受时再取消一次。
        结果未确认（CANCELED）前 ``cancel_request`` 不清，状态保持锁停。
        """
        self.state = 'stopped'
        self._manual_override = False
        if user_action or not self.locked or not self.stop_reason:
            self._set_locked(reason)
            self.stop_reason = reason
        else:
            self.lock_stop(reason)     # 已锁停：保持原原因
        if not self.stop_reason:
            self.stop_reason = reason
        # 已接受的目标：取消返回 True 才算"已停"；返回 False（取消没被
        # 接受）保留 cancel_request，结果回来再对账 / 迟到接受再取消一次。
        rid = self.pending
        if rid is not None:
            # 取消"已被接受"与"结果已确认"是两件事：前者只说明取消请求
            # 没被拒，目标仍可能随后到达；真实结果由 on_result 对上。
            # 取消请求成功时记 _cancel_by：之后结果确认只撤它设的锁，
            # 不牵扯别的原因（例如会话断线）。
            self.cancel_confirmed = self._cancel_now(rid, result_known=True)
            self.cancel_request = rid
        # 还没被接受的：取消会抛（没有句柄），只记下来，接受一到立即取消。
        if self.sending is not None:
            self.cancel_request = self.sending
            self._cancel_now(self.sending, result_known=False)
        self.pending = None
        self.sending = None
        self.sending_since = None
        self.plan_blocked_since = None
        return True

    def manual_override(self):
        """手动接管：先取消旧导航；松键后旧任务不得自行恢复。

        只有确实有在途导航时才动作；没有任何导航时不动锁停（避免把手动
        控制里"全局停车"留下的锁停顺手放开）。
        """
        was_active = (self.state in ('running', 'holding_goal')
                      and (self.pending is not None or self.sending is not None))
        if not was_active:
            return False
        already_locked = self.locked
        self.stop('manual_takeover', user_action=True)
        self._manual_override = True
        # 手动接管只撤掉"巡航自己的"锁停（全局停车/会话失效/导航失败留下
        # 的锁停不在这里放开：那些只能由用户点"恢复"解除）。巡航本身
        # 不恢复，松键后停在原地。
        if already_locked:
            # 接管前就锁停：原因与锁都保持（接管不是"恢复"）。
            return True
        if self.cancel_request is not None and not self.cancel_confirmed:
            # 取消根本没被接受（或目标还没发到 Nav2）：结果回来前保持锁停
            # （宁可过停），由 on_result 对账后再撤。
            self._lock_by = 'manual'
            self.lock_reason = self.lock_reason or 'manual_takeover'
            return True
        # 旧目标已停（取消已被接受或本来还没发出）：接管动作写的"锁停"
        # 只是停车，撤掉；旧任务即便迟到成功也不会再发下一点。
        self._lock_by = 'manual'
        self.release_lock('manual_takeover')
        return True

    def _manual_cancel_confirmed(self, reason_was):
        """手动接管的"待确认取消"已确认：撤掉它设的锁停（条件严格）。

        ``reason_was``：确认之前记下的锁停原因。只有仍是 manual_takeover
        （没有被会话断线/失败等更严重的事件改写）才撤。
        """
        if self._lock_by != 'manual':
            return
        self._lock_by = None
        if self.pending is not None or self.sending is not None:
            return
        if self.cancel_request is not None:
            return                          # 还有未确认的取消：锁停保留
        if reason_was not in ('manual_takeover', ''):
            return                          # 原因已被更严重的事件改写
        if self.lock_reason not in ('manual_takeover', ''):
            return
        self.release_lock('manual_takeover')

    def manual_released(self):
        """手动松键：只结束手动输入，巡航与锁停都不自动恢复。"""
        return self.state == 'stopped'

    def lock_stop(self, reason):
        """请求锁停。已锁停时保留首个原因（派发失败/导航失败/超时等更贴近
        事实的原因），后续的会话失效/断线只保持锁停、不覆盖显示。"""
        already = self.locked and bool(self.lock_reason)
        self.locked = True
        if not already:
            self.lock_reason = reason or 'global_stop'
        try:
            self._lock_stop(self.lock_reason)
        except Exception:              # noqa: BLE001——锁停请求失败也不能抛穿
            pass

    def release_lock(self, _by='rearm'):
        self.locked = False
        self.lock_reason = ''

    def rearm(self, *, license_ok):
        """解除锁停。必须先拿到新鲜许可；解除后仍停在原地，不自动巡航。

        解除锁停不能夹带巡航：有在途目标/待确认取消时先取消并返回失败，
        由用户确认停止后再解除。
        """
        if not license_ok:
            return False, '网页许可无效，不能解除锁停'
        # 先看有没有在途/待确认的取消：已经在跑（没有锁停）时点恢复，
        # 不能只"顺手解锁"——旧目标仍可能被接受并开走；一律按全局停车先取消。
        if self.cancel_request is not None or self.pending is not None \
                or self.sending is not None:
            if self.state in ('running', 'holding_goal'):
                self.stop('global_stop', user_action=True)
            if self.cancel_request is not None:
                return False, '上一个目标取消未确认，保持锁停'
            return False, '上次巡航已停止，仍有未清算的目标；请稍后重试'
        if not self.locked:
            return True, 'ok'              # 幂等：本来就未锁停
        if self.state in ('running', 'holding_goal'):
            self.stop('global_stop', user_action=True)
            # 锁停保持，原因回到用户看到的"巡航中点了恢复"：按用户意图是
            # 全局停车，不是导航失败。
            self.lock_reason = 'global_stop'
            return False, '巡航已停止，锁停保持；确认后再次解除'
        self.release_lock('rearm')
        self.stop_reason = ''
        return True, 'ok'

    # -- 内部 ----------------------------------------------------------
    def _goal(self):
        x, y, yaw = self.route['points'][self.index]
        return {'x': x, 'y': y, 'yaw': yaw}

    def _cancel_now(self, rid, result_known):
        # 同时只挂一个取消请求：巡逻侧 bus 模块（web_debug._patrol_cancel）
        # 可能在取消失败的回滚回调里再调进来，递归会无界放大。
        if rid in self._cancel_busy:
            return False           # 同一目标的取消正在途：不重入
        self._cancel_busy.add(rid)
        try:
            # 取消结果的确认由调用方（stop/manual_override/start）落到
            # cancel_request 上：这里只负责发出取消、返回 Nav2 是否确认。
            try:
                return bool(self._cancel(rid, result_known))
            except Exception:          # noqa: BLE001
                return False
        finally:
            self._cancel_busy.discard(rid)

    def _issue(self):
        try:
            rid = self._dispatch(self._goal())
        except Exception:              # noqa: BLE001——发送失败等同导航失败
            self._fail('dispatch_failed')
            return False
        if rid is None:
            self._fail('dispatch_failed')
            return False
        self.last_rid = rid
        self.sending = rid
        self.sending_since = self._now()
        self.plan_blocked_since = None
        return True

    def _fail_keep_lock(self, request_id, reason):
        """锁停保持，只补记停车原因（不覆盖原锁停原因）。"""
        self.state = 'stopped'
        if not self.stop_reason:
            self.stop_reason = reason

    def _fail(self, reason):
        self.state = 'failed'
        self.stop_reason = reason
        if self.pending is not None:
            self._cancel_now(self.pending, result_known=True)
        if self.sending is not None:
            self._cancel_now(self.sending, result_known=False)
        self.pending = None
        self.sending = None
        self.plan_blocked_since = None
        self.lock_stop(reason)

    # -- 外部事件（web_debug 从 NavGoalControl 回调转发）----------------
    def on_accepted(self, request_id):
        """目标被 Nav2 接受。

        停车后迟到的接受（包括取消失败后迟到、以及上一趟 parked 之后
        才到达的接受）一律立即取消；结果确认前保持锁停。
        """
        if self.cancel_request == request_id:
            reason_was = self.lock_reason
            if self._cancel_now(request_id, result_known=True):
                self.cancel_confirmed = True
                self._manual_lift_if_late_result(reason_was)
            return
        if request_id != self.sending:
            if (self.state == 'stopped'
                    and request_id == self.last_rid):
                self._cancel_now(request_id, result_known=True)
            return
        self.sending = None
        self.sending_since = None
        if self.locked or self.cancel_request is not None:
            # 锁停中绝不继续巡航：接受即取消，锁停保持
            self._cancel_now(request_id, result_known=True)
            self._fail_keep_lock(request_id, 'global_stop')
            return
        if self.state in ('stopped', 'failed'):
            self._cancel_now(request_id, result_known=True)
            return
        self.pending = request_id
        self.state = 'running'
        self.plan_blocked_since = None     # 规划受阻从"目标被接受"重新起算

    def on_result(self, request_id, status, reason=''):
        """导航结果。``status``：``'succeeded' | 'failed' | 'cancelled'``。

        ``request_id`` 只接受整数编号：调用方（回调转发）传错对象时必须
        报错，而不能因为对象比较恒假就当成"无关结果"默默丢弃。
        """
        if isinstance(request_id, bool) or not isinstance(request_id, int):
            raise TypeError('导航结果缺少整数请求编号: %r' % (request_id,))
        if self.cancel_request == request_id:
            reason_was = self.lock_reason
            # 结果已确认：取消生效，锁停继续（不能因为取消成功就认为可恢复）
            if status == 'cancelled':
                # Nav2 明确回"取消成功"才是结果确认；failed/succeeded 是
                # 旧目标自己跑完/失败，取消本身并没有被确认，记录保留。
                self.cancel_request = None
            self._manual_lift_if_late_result(reason_was)
            return
        if request_id == self.sending:
            # 接受前就返回结果：视为失败（不重发）
            self.sending = None
            self.sending_since = None
            self._fail(reason or status or 'nav_failed')
            return
        if request_id != self.pending:
            return
        self.pending = None
        if self.state in ('stopped', 'failed'):
            return                 # 手动接管/停车后的旧结果不复活巡航
        if status != 'succeeded':
            self._fail(reason or status or 'nav_failed')
            return
        self.index += 1
        if self.index >= len(self.route['points']):
            self.index = 0
            self.laps += 1
        self._issue()

    def _manual_lift_if_late_result(self, reason_was):
        """迟到接受/结果到达后重新评估手动接管的"待确认取消"锁。

        ``reason_was`` 是进入本回调前的锁停原因：回调里 ``_cancel_now``
        可能先触发 Nav2 的回滚（重新锁停），不能拿回调之后的原因判断。
        """
        self._manual_cancel_confirmed(reason_was)

    def on_plan_blocked(self, blocked):
        """规划连续 ``PLAN_BLOCK_S`` 秒无安全路径/无合法控制轨迹 → 取消并锁停。

        ``blocked`` 是"此刻仍然受阻"的**当前值**，不是调用沿：本函数被
        10 Hz 定时器、会话看门狗和读循环各调一次，按"相隔两次调用"计时
        会把 3 s 变成几十秒。计时只从"首次看到受阻"开始，并在目标被接受
        （:meth:`on_accepted`）时重新起算——目标刚接受时还没有反馈，不能
        把等待第一帧反馈的时间算成规划失败。
        """
        if not blocked:
            self.plan_blocked_since = None
            return
        if self.state != 'running' or self.pending is None:
            self.plan_blocked_since = None
            return
        now = self._now()
        if self.plan_blocked_since is None:
            self.plan_blocked_since = now
            return
        if now - self.plan_blocked_since >= PLAN_BLOCK_S:
            self._fail('no_safe_path')

    def overall_timeout(self, limit_s):
        """在途目标超时未接受：取消并锁停，防止无限等待。"""
        if self.sending_since is None or not limit_s:
            return False
        if self._now() - self.sending_since > limit_s:
            self._fail('goal_timeout')
            return True
        return False

    def disconnect(self):
        """网页断线：取消并锁停，不自动恢复。"""
        if self.state in ('running', 'holding_goal') or self.cancel_request:
            self.stop('web_disconnected', user_action=True)

    def snapshot(self):
        goal = None
        if self.route is not None and self.route['points']:
            g = self._goal()
            goal = {'x': g['x'], 'y': g['y'], 'yaw': round(g['yaw'], 4)}
        return {
            'status': self.state,
            'map_id': None if self.route is None else self.route['map_id'],
            'index': self.index,
            'waypoints': 0 if self.route is None else len(self.route['points']),
            'laps': self.laps,
            'goal': goal,
            'stop_reason': self.stop_reason,
            'stop_text': LOCK_REASON_TEXT.get(self.stop_reason, self.stop_reason),
            'locked': self.locked,
            'lock_reason': self.lock_reason,
            'lock_text': LOCK_REASON_TEXT.get(self.lock_reason, self.lock_reason),
            'pending_cancel': self.cancel_request,
            'awaiting_accept': self.sending,
            'manual_override': self._manual_override,
        }


# ---------------------------------------------------------------- 网页许可
class MotionLicense:
    """网页运动许可（``/motion_license``）的服务端侧。

    - 前端每 ``LICENSE_PERIOD_S``（100 ms）发一条 ``{session, seq,
      expires_mono}``；``session`` 是网页页面加载时生成的随机正整数，
      ``seq`` 在同一会话号内严格递增，``expires_mono`` 是**服务端**单调
      时钟上的期限（页面用校准帧 ``motion_clock`` 的 (perf0, mono0) 锚点
      换算，见 console_page.html 的 ``motionNowMs``）。
    - 服务端 ``SESSION_TIMEOUT_S``（350 ms）没收到新许可即判会话失效：
      既不看网页发来的时间戳做判断，也不依赖 websocket 读循环（进程卡死
      时循环不跑，但 :meth:`expired` 按"最后收到时刻"照样判失效）。
    - 判失效时只请求一次锁停（``PatrolRunner.lock_stop('license_lost')``）；
      下游底盘桥在自身 ``LICENSE_TIMEOUT_S`` 后独立零速——网页进程退出后
      不再转发许可，下游照样锁停（见 test_patrol 里的说明用例）。

    "只有在收到有效许可且锁停已解除时转发锁停状态"由
    :func:`status_frame` 落实：许可不新鲜时 ``licensed=False``，
    不再把"已解除"对外宣称。
    """

    def __init__(self, on_lost, now=None):
        self._on_lost = on_lost
        self._now = now or time.monotonic
        self._lock = threading.Lock()
        self.session = None
        self.seq = -1
        self.last_rx = None
        self.expires = None
        self.rejected = 0
        self.lost_reports = 0
        self.takeovers = 0
        # 启动即丢失（无许可、锁停），但"失效"在上游侧从没复述过，
        # 所以第一次判失效也要上报一次（让锁停原因如实是 license_lost）。
        self._lost = False
        # 会话是否被判定结束（最后一个网页连接走掉 / 旧会话被新会话顶替）：
        # 即使最后一条许可还在 350 ms 窗口内，也不许再对外宣称"在线"。
        # 启动时为 False——没收到过许可是"从未开始"，由 age=inf 判失效。
        self._stale = False

    def mark_stale(self):
        """记一条"会话已结束"：现有许可视为失效，等下一页重新送来许可。"""
        with self._lock:
            self._stale = True

    def _session_ended(self, now):
        """已锁定的会话是否已经结束（可以允许新代次接管）。

        - 被显式标记（最后一个网页连接走掉 / 会话失效）；
        - 或超过 350 ms 没有任何新许可（旧页面关掉后不再送许可）。

        两条都要：页面只断开 ws（TCP 半开）时仍可能在发许可，必须靠
        会话连接集合显式标记，否则"旧会话永续期、新会话永被拒"。
        """
        if self._stale:
            return True
        return self.last_rx is None or now - self.last_rx > SESSION_TIMEOUT_S

    def accept(self, session, seq, expires_mono=None):
        """返回 ``(ok, reason)``。

        会话代次第一次收到时锁定；旧会话**已经停止送许可**（超过 350 ms）
        后允许新代次接管——否则被关掉/刷新的页面留下的旧代次会让新页面
        永远被判 ``session_mismatch``，许可永远不新鲜、``rearm`` 永远失败。
        """
        now = self._now()
        with self._lock:
            if isinstance(session, bool) or not isinstance(session, int) or session <= 0:
                self.rejected += 1
                return False, 'session_invalid'
            if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
                self.rejected += 1
                return False, 'seq_invalid'
            if self.session is None:
                self.session = session
            elif session != self.session:
                if not self._session_ended(now):
                    self.rejected += 1
                    return False, 'session_mismatch'
                # 旧代次已停止送许可：新页面接管，序号从头开始。旧会话
                # 的状态仍按"已结束"计（_stale），直到新页面送来第一条许可。
                self.session = session
                self.seq = -1
                self.takeovers += 1
                self._event_stale()
            if seq <= self.seq:
                self.rejected += 1
                return False, 'seq_replay'
            exp = None
            if expires_mono is not None:
                try:
                    exp = float(expires_mono)
                except (TypeError, ValueError):
                    self.rejected += 1
                    return False, 'expires_invalid'
                if not math.isfinite(exp) or exp <= now or exp - now > LICENSE_GRANT_S * 5:
                    self.rejected += 1
                    return False, 'expired'
            self.seq = seq
            self.last_rx = now
            self.expires = exp
            self._stale = False
            return True, 'ok'

    def age(self):
        with self._lock:
            if self.last_rx is None:
                return float('inf')
            return self._now() - self.last_rx

    def fresh(self):
        age = self.age()
        if age > SESSION_TIMEOUT_S:
            return False
        with self._lock:
            if self._stale:
                return False
            exp = self.expires
        return exp is None or self._now() <= exp

    def _event_stale(self):
        """顶替发生时立刻判一次失效（不等 350 ms），只报一次。"""
        if not self._stale:
            self._stale = True
            self._lost = True
            self.lost_reports += 1
            try:
                self._on_lost('license_lost')
            except Exception:          # noqa: BLE001
                pass

    def expired(self):
        """判会话失效（350 ms 无新许可 / 被顶替 / 页面已走）。

        由定时器反复调用，失效只上报一次；重新收到有效许可后恢复在线
        （但不解除锁停）。

        被顶替/页面走掉时 :meth:`mark_stale` 先置位（新会话可立即接管）；
        这里只负责"350 ms 没有新许可"的常规失效上报。
        """
        return self.report_lost()

    def report_lost(self):
        """按当前许可状态上报一次失效（不改变许可本身）。

        会话守护用这个：它已把真实原因（断线/会话结束）写进锁停，
        不能让许可守护再用 ``license_lost`` 盖掉。
        """
        if self.fresh():
            self._lost = False
            return False
        with self._lock:
            first = not self._lost
            self._lost = True
        if first:
            self.lost_reports += 1
            try:
                self._on_lost('license_lost')
            except Exception:          # noqa: BLE001
                pass
        return True

    def snapshot(self):
        age = self.age()
        return {
            'session': self.session,
            'seq': self.seq,
            'age_s': None if age == float('inf') else round(age, 3),
            'fresh': self.fresh(),
            'rejected': self.rejected,
            'takeovers': self.takeovers,
            'session_dead': self._stale or self.age() > SESSION_TIMEOUT_S,
        }


def status_frame(runner, license_state, map_id, now=None):
    """``/patrol_status`` 状态帧。

    服务端只在"收到有效许可"时才算在线；锁停是否已解除（``released``）
    也只在许可有效时才为 True——网页进程退出后下游独自锁停，本帧不会
    再对外宣称已解除。
    """
    snap = runner.snapshot()
    licensed = bool(license_state.get('fresh'))
    return {
        't': 'patrol_status',
        'map_id': snap['map_id'],
        'current_map_id': map_id,
        'route_valid': snap['map_id'] is not None and snap['map_id'] == map_id,
        'index': snap['index'],
        'waypoints': snap['waypoints'],
        'laps': snap['laps'],
        'goal': snap['goal'],
        'status': snap['status'],
        'stop_reason': snap['stop_reason'],
        'stop_text': snap['stop_text'],
        'locked': snap['locked'],
        'lock_reason': snap['lock_reason'],
        'lock_text': snap['lock_text'],
        # guard 口径的停车原因（B5）：锁停时与下游底盘桥的
        # WriteRecord.stop_cause 同一套枚举，页面可与下游记录对账。
        'stop_cause': guard_stop_cause(snap['lock_reason'] or
                                       snap['stop_reason']),
        # 恢复状态口径（B6）：页面据此区分"等用户按恢复"/"已按过等下游"/
        # "无需恢复"，不再只能笼统显示锁停。
        'rearm_state': rearm_state(snap['locked'], licensed,
                                   bool(licensed and not snap['locked']),
                                   snap['pending_cancel']),
        'pending_cancel': snap['pending_cancel'],
        'manual_override': snap['manual_override'],
        'licensed': licensed,
        'license': license_state,
        'released': bool(licensed and not snap['locked']),
        'nav': None,
    }
