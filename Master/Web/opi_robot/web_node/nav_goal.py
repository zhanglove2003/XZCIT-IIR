"""[C50C 阶段 D 2026-11-16] 网页点图导航：/navigate_to_pose action 客户端。

职责边界（与 manual_forward 同型）：
- 纯逻辑 + rclpy ActionClient 包装；web_debug 的节点提供建 publisher/
  client 的载体（action client 不能在 Node 外独立建）。
- 鉴权在 web_debug._manual_message 里做（复用 _MANUAL.authenticated），
  本模块只管"目标是否合法、是否可发"。

安全链（实车已验证）：Nav2 → relay(取负÷2.41) → /nav_cmd_vel → 桥
arbiter（/drive_safety 安全门，ultrasonic_stop/obstacle_stop 拦停）。
本模块不重复安全判断，只做目标合法性（限幅在已扫区）。

线程模型：web_debug 是单线程 executor（timer + ws 线程分离）。action
的 feedback/result 用 executor 回调；goal 句柄的取消也走 executor。
ws 线程只调 request_goal()/cancel()——两者都只是"投递"，不阻塞。
"""

import math
import sys
import threading
import time
import traceback

# 目标范围限制：地图栅格是 584×256 @ 0.05m，原点 -8.28,-8.03；
# 允许目标落在 [min, max] 内（覆盖已扫区，留余量）。
GOAL_MIN_X = -8.0
GOAL_MAX_X = 22.0
GOAL_MIN_Y = -8.0
GOAL_MAX_Y = 8.0
# 目标与车的最小间距：太近 DWB 直接判定到达或振荡。
GOAL_MIN_DIST_M = 0.3


class NavGoalControl:
    """网页点图导航的状态机：idle → sending → active → (done/canceled)。"""

    def __init__(self, on_accepted=None, on_result=None):
        # 巡航（patrol.py）据此知道"目标何时被接受/结束"：迟到目标在
        # 被接受的那一刻必须能立即取消（本轮竞态修复）。
        # 构造参数只是初值；运行时接线一律用 set_callbacks()。回调存在
        # 独立字段里：不能写 self._on_result/_on_accepted——那会遮蔽同名
        # 方法（self._on_result(fut, seq) 的方法体），结果回调直接
        # TypeError，取消确认静默丢失、锁停永不解除。
        self._result_cb = on_result
        self._accepted_cb = on_accepted
        self._lock = threading.Lock()
        self._client = None          # rclpy ActionClient（节点 attach 时建）
        self._goal_handle = None
        self._goal_future = None
        # 已发出、尚未被接受的目标：{seq: handle-or-None}
        # 停车时若目标还没被接受，就记进 _parked；接受回调一到立即取消。
        self._pending_seq = None
        self._parked = None          # 被停车后仍需取消的 seq（无论当时取消成败）
        self._cancel_futs = []
        self._result_futs = []
        self._state = 'idle'         # idle/sending/active/done/canceled/failed
        self._goal = None            # {'x','y','th'}
        self._goal_t = None
        self._feedback = None        # 最近一次 feedback 的剩余距离
        self._feedback_t = None      # 最近一次 feedback 的单调时刻
        self._result_text = ''
        self._plan_error = ''
        self._send_error = ''
        self._seq = 0

    # ---------- 接线 ----------

    def attach(self, node):
        """在 web_debug 节点上建 action client（rclpy 要求挂在 node 上）。"""
        try:
            from rclpy.action import ActionClient
            from nav2_msgs.action import NavigateToPose
        except ImportError:
            return False
        try:
            self._client = ActionClient(node, NavigateToPose,
                                        'navigate_to_pose')
        except Exception:
            self._client = None
            return False
        return True

    def attached(self):
        return self._client is not None

    # ---------- ws 调用入口 ----------

    def request_goal(self, x, y, th=None):
        """网页发来的目标点。返回 (ok, 状态字典)。

        th=None 表示保持当前朝向（点图导航语义：直行过去，
        到达后不原地转向——开环差速车原地转容易打滑堆位姿误差）。
        """
        try:
            x = float(x)
            y = float(y)
        except (TypeError, ValueError):
            return False, self._snapshot('目标坐标必须是数字')
        if not (math.isfinite(x) and math.isfinite(y)):
            return False, self._snapshot('目标坐标必须有限')
        if not (GOAL_MIN_X <= x <= GOAL_MAX_X
                and GOAL_MIN_Y <= y <= GOAL_MAX_Y):
            return False, self._snapshot(
                '目标超出地图范围（%.0f~%.0f, %.0f~%.0f）'
                % (GOAL_MIN_X, GOAL_MAX_X, GOAL_MIN_Y, GOAL_MAX_Y))
        with self._lock:
            if self._state in ('sending', 'active'):
                # 注意：_snapshot 内部也拿 self._lock（不可重入），在锁内
                # 必须用 _snapshot_locked，否则自死锁（首测踩过）。
                return False, self._snapshot_locked(
                    '已有导航任务进行中，先取消再发新目标')
            goal = {'x': round(x, 3), 'y': round(y, 3),
                    'th': None if th is None else round(float(th), 4)}
            self._goal = goal
            self._state = 'sending'
            self._goal_t = time.time()
            self._feedback = None
            self._result_text = ''
            self._send_error = ''
            self._seq += 1
            seq = self._seq
            self._pending_seq = seq
        # 发送在 executor 线程外投递：rclpy 的 ActionClient.async_send_goal
        # 线程安全（内部走 DDS），这里直接调。
        ok = self._send(goal, seq)
        return ok, self._snapshot()

    def _send(self, goal, seq):
        try:
            from nav2_msgs.action import NavigateToPose
            from geometry_msgs.msg import PoseStamped
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self._node_now()
            msg.pose.position.x = float(goal['x'])
            msg.pose.position.y = float(goal['y'])
            yaw = 0.0 if goal['th'] is None else float(goal['th'])
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
            if self._client.wait_for_server is not None:
                # 服务发现没就绪时 send_goal_async 会抛 RuntimeError；
                # 先在 executor 线程上等一小段，仍连不上按发送失败处理。
                self._client.wait_for_server(timeout_sec=2.0)
            g = NavigateToPose.Goal()
            g.pose = msg
            self._goal_future = self._client.send_goal_async(
                g, feedback_callback=self._on_feedback)
            self._goal_future.add_done_callback(
                lambda f, s=seq, ctl=self: ctl._on_goal_response(f, s))
            return True
        except Exception as exc:                     # pragma: no cover
            with self._lock:
                self._state = 'failed'
                self._send_error = str(exc)
            return False

    def _node_now(self):
        import rclpy
        return rclpy.time.Time().to_msg() if rclpy.ok() else None

    def _on_goal_response(self, future, seq):
        try:
            handle = future.result()
        except Exception as exc:
            with self._lock:
                parked = self._parked == seq
                if self._pending_seq == seq:
                    self._pending_seq = None
                if self._seq == seq and self._state == 'sending':
                    self._state = 'failed'
                    self._send_error = str(exc)
            if parked:
                with self._lock:
                    if self._parked == seq:
                        self._parked = None
            self._notify_result(seq, 'failed', str(exc))
            return
        if handle is None or not handle.accepted:
            with self._lock:
                parked = self._parked == seq
                if self._pending_seq == seq:
                    self._pending_seq = None
                if self._parked == seq:
                    self._parked = None
                if self._seq == seq and self._state == 'sending':
                    self._state = 'failed'
                    self._send_error = '目标被拒绝（规划器不可达或正在执行其他任务）'
                elif self._seq == seq and parked:
                    self._state = 'canceled'      # 停车后目标被拒：已结束
                    self._result_text = '目标被拒绝'
            self._notify_result(seq, 'failed', '目标被拒绝')
            return
        cancel_now = False
        with self._lock:
            if self._pending_seq == seq:
                self._pending_seq = None
            if self._parked == seq:
                # 停车时目标还没被接受：现在迟到的接受 → 立即取消
                self._parked = None
                cancel_now = True
        if cancel_now:
            try:
                fut = handle.cancel_goal_async()
            except Exception:
                fut = None      # 取消失败：patrol 侧记录，保持锁停
            if fut is not None and hasattr(fut, 'add_done_callback'):
                with self._lock:
                    self._state = 'canceled'
                    self._goal_handle = handle
                self._cancel_futs.append(fut)
                fut.add_done_callback(
                    lambda f, h=handle, ctl=self: ctl._on_cancel(f, seq, h))
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f, s=seq, ctl=self: ctl._on_result(
                f, s))  # 见 _result_cb：不依赖可被替换的实例属性
        self._result_futs.append(result_future)
        with self._lock:
            if self._seq == seq and self._state == 'sending' and not cancel_now:
                self._goal_handle = handle
                self._state = 'active'
            elif self._seq == seq and cancel_now:
                self._goal_handle = handle
                self._state = 'canceled'
        self._notify_result(seq, 'accepted', '')

    def _notify_result(self, seq, status, text):
        """把"已接受/已结束"转给巡航侧（web_debug 注入）。"""
        # 用构造时登记的真实回调（实例属性可能被测试/接线替换为 None）。
        cb = (self._accepted_cb if status == 'accepted'
              else self._result_cb)
        if cb is None:
            return
        try:
            if status == 'accepted':
                cb(seq)
            else:
                # 位置参数传结果：巡航侧（及测试替身）按
                # on_result(请求编号, 状态, 说明) 接收。
                cb(seq, status, text or '')
        except Exception:
            # 回调里的异常不能吞掉：巡航状态机报错（比如编号不对）时
            # 必须留下痕迹，而不是让停车/取消竞态悄悄失效。
            sys.stderr.write('%s: nav_goal callback %s(seq=%r, %r) failed:\n%s'
                             % (time.time(), getattr(cb, '__name__', cb),
                                seq, status, traceback.format_exc()))

    def _on_feedback(self, fb):
        try:
            fbm = fb.feedback
            # NavigateToPose feedback: current_pose / distance_remaining /
            # estimated_time_remaining / number_of_replans
            dr = getattr(fbm, 'distance_remaining', None)
            et = getattr(fbm, 'estimated_time_remaining', None)
            cp = getattr(fbm, 'current_pose', None)
            with self._lock:
                self._feedback_t = time.monotonic()
                self._feedback = {
                    'dist': (None if dr is None
                             else round(float(dr), 2)),
                    'eta': (None if et is None
                            else round(float(et.sec + et.nanosec * 1e-9), 1)),
                    'x': (None if cp is None
                          else round(float(cp.pose.position.x), 2)),
                    'y': (None if cp is None
                          else round(float(cp.pose.position.y), 2)),
                }
        except Exception:
            pass

    def _on_result(self, future, seq):
        try:
            result = future.result()
        except Exception as exc:
            with self._lock:
                parked = self._parked == seq
                if self._seq == seq:
                    self._state = 'failed'
                    self._send_error = str(exc)
                    self._goal_handle = None
                    if self._pending_seq == seq:
                        self._pending_seq = None
                    if parked:
                        self._parked = None
            # 停车后等待的目标拿不到结果：按"未确认"（failed）上报，
            # 巡航侧保持锁停，不会因为取消没确认就认为已停。
            self._notify_result(seq, 'failed', str(exc))
            return
        status = getattr(result, 'status', None)
        text = {
            4: '到达目标',                    # SUCCEEDED
            5: '已取消',                      # CANCELED
            6: '导航中止（规划/控制失败）',    # ABORTED
        }.get(int(status) if status is not None else 0, '导航结束（状态 %s）'
              % status)
        code = int(status) if status is not None else 0
        # Nav2 的 result 带 error_code（int 枚举）：规划/控制失败的原因码
        # 是巡航判"无安全路径"的明确依据，比 feedback 断流更准。只取
        # 非零码，转成带名字的文本给 view()['plan_error']。
        err_code = getattr(result, 'error_code', None)
        plan_error = ''
        try:
            err_code = int(err_code) if err_code is not None else 0
        except (TypeError, ValueError):
            err_code = 0
        if err_code:
            plan_error = 'error_code=%d(%s)' % (
                err_code, getattr(err_code, 'name', '') or 'nav2')
        with self._lock:
            if self._seq != seq:
                return
            self._plan_error = plan_error
            self._state = ('done' if code == 4
                           else 'canceled' if code == 5
                           else 'failed')
            self._result_text = text
            self._goal_handle = None
            if self._pending_seq == seq:
                self._pending_seq = None
            parked = self._parked == seq
            if parked:
                self._parked = None
        self._notify_result(seq, 'succeeded' if code == 4
                            else 'cancelled' if code == 5 else 'failed', text)

    def cancel(self):
        """取消。无任务时返回 ``(False, view)``。

        两种目标都要处理：
        - 已被接受（有句柄）：按句柄取消，发出即返回 True；取消应答
          失败会回滚为 active 并清掉 ``_parked``（不能认为已停）。
        - 尚未被接受：记进 ``_parked``，接受回调一到立即取消；返回 False
          （"尚未确认停止"），状态为 canceled。
        """
        with self._lock:
            if self._pending_seq is not None:
                seq = self._pending_seq
                self._parked = seq
                if self._seq == seq:
                    self._state = 'canceled'
                view = self._snapshot_locked('目标尚未被接受，取消将在接受后立即执行')
            else:
                handle = self._goal_handle
                if handle is None or self._state not in ('sending', 'active'):
                    return False, self._snapshot_locked('当前没有进行中的导航任务')
                seq = self._seq
                self._state = 'canceled'
                self._parked = seq          # 取消发出但结果未确认前仍记录
                view = None
        if view is not None:
            return False, view
        fut = None
        try:
            fut = handle.cancel_goal_async()
        except Exception as exc:
            with self._lock:
                self._send_error = '取消失败：%s' % exc
        # 同步抛异常 ≠ 取消失败：rclpy 在"没有活跃目标"时也可能抛，
        # 与旧行为一致按已发出处理；真正的取消失败由应答回调回滚。
        # 真正的取消应答是 future：失败时回滚为 active（巡航侧据此保持锁停）。
        if fut is not None and hasattr(fut, 'add_done_callback'):
            fut.add_done_callback(
                lambda f, h=handle, ctl=self: ctl._on_cancel(f, seq, h))
        return True, self._snapshot()

    def _on_cancel(self, future, seq, handle):
        """取消应答：失败 → 回到 active（不能认为已停）。"""
        ok = True
        try:
            r = future.result()
        except Exception:
            ok = False                 # 应答丢了/失败：视为取消未生效
        if ok and r is not None:
            # 真实 rclpy 的 CancelResponse 没有布尔真值语义：以 return_value 为准。
            rc = getattr(r, 'return_value', True)
            ok = rc if isinstance(rc, bool) else True
        if ok:
            return
        with self._lock:
            if self._seq == seq and self._state == 'canceled':
                self._state = 'active'
                self._goal_handle = handle
                self._parked = None
                self._send_error = '取消未被接受'

    def cancel_seq(self, seq, result_known=True):
        """按请求编号取消（巡航用）。返回是否"已发出取消"。

        - 目标尚未被接受：记进 ``_parked``，接受回调一到立即取消，返回
          True（取消已排程）。
        - 已被接受：按句柄取消失败（同步抛异常 / 应答失败）返回 False，
          ``_parked`` 清掉——巡航侧据此保持锁停，等结果确认或迟到接受。
        """
        with self._lock:
            if self._pending_seq == seq:
                # 接受前停车：清掉发送态。目标已在发送途中、只是我们还没
                # 收到接受应答——保守地视为"可能已被接受"：返回 False
                # （取消未确认），等接受回调到达再按句柄取消。
                self._pending_seq = None
                self._parked = seq
                if self._seq == seq:
                    self._state = 'canceled'
                return False
            handle = self._goal_handle
            if self._seq != seq or handle is None:
                return False
            self._parked = seq
            self._state = 'canceled'
        fut = None
        try:
            fut = handle.cancel_goal_async()
        except Exception as exc:
            with self._lock:
                self._send_error = '取消失败：%s' % exc
            with self._lock:
                if self._seq == seq and self._state == 'canceled':
                    self._state = 'active'       # 取消失败：不能认为已停
                    self._parked = None
            return False
        sent = True
        if fut is not None and hasattr(fut, 'add_done_callback'):
            self._cancel_futs.append(fut)
            fut.add_done_callback(
                lambda f, h=handle, ctl=self: ctl._on_cancel(f, seq, h))
        # 应答可能同步到达（离线替身/已完成的 future）：回调里已回滚为
        # active 就说明取消失败。
        if self._cancel_failed(seq):
            sent = False
        return sent

    def _cancel_failed(self, seq):
        """取消应答是否已判失败（同步应答失败时 add_done_callback 会先回滚）。"""
        with self._lock:
            return self._seq == seq and self._state not in ('canceled',)

    # ---------- 视图 ----------

    def view(self):
        return self._snapshot()

    def _snapshot(self, error=None):
        with self._lock:
            return self._snapshot_locked(error)

    def _snapshot_locked(self, error=None):
        """持锁版本：调用方已持有 self._lock 时用这个（锁不可重入）。"""
        now = time.time()
        return {
            'state': self._state,
            'goal': dict(self._goal) if self._goal else None,
            'age': (None if self._goal_t is None
                    else round(now - self._goal_t, 1)),
            'feedback': dict(self._feedback) if self._feedback else None,
            'feedback_age': (None if self._feedback_t is None
                             else round(time.monotonic() - self._feedback_t, 2)),
            'result': self._result_text,
            'error': error or self._send_error,
            # 规划/控制失败的原因（Nav2 error_code），巡航据此判"无安全路径"
            'plan_error': getattr(self, '_plan_error', '') or '',
            'client': self._client is not None,
            'seq': self._seq,
        }


_NAV = NavGoalControl()

#: /drive_safety 原文视图（web_debug 收到消息时注入）。导航状态只转述
#: fusion_node 给出的运动方向门结论，不在这里另算。
_SAFETY_VIEW = {'state': 'unknown', 'allowed': None, 'reason': None,
                'motion': None}


def set_safety_view(view):
    """web_debug：把 drive_safety_view(...) 的同一份结果交给导航状态。"""
    global _SAFETY_VIEW
    _SAFETY_VIEW = dict(view or {})
    return _SAFETY_VIEW


def set_callbacks(on_accepted=None, on_result=None, control=None):
    """注入巡航回调（web_debug 的 Bridge 启动时调）。

    只写在回调字段里，不覆盖同名方法 ``_on_result``（结果回调入口）。
    """
    target = control if control is not None else _NAV
    target._accepted_cb = on_accepted
    target._result_cb = on_result


def cancel_nav_seq(seq, result_known=True):
    return _NAV.cancel_seq(seq, result_known)


def nav_view(safety_view=None):
    """状态帧视图（web_debug 的 status_payload 调）。

    ``safety`` 是 ``/drive_safety`` 的同一份视图（含 motion 门），由
    web_debug 传入（``drive_safety_view(...)``）；不传时用最近一次注入值。
    导航本体不据此另做判定——桥的仲裁器才是执行点。
    """
    view = _NAV.view()
    view['safety'] = dict(_SAFETY_VIEW if safety_view is None else safety_view)
    return view


def attach_nav(node):
    """web_debug 节点启动时接线。"""
    return _NAV.attach(node)


def request_nav_goal(x, y, th=None):
    return _NAV.request_goal(x, y, th)


def cancel_nav():
    return _NAV.cancel()
