#!/usr/bin/env python3
"""nav_relay：Nav2 速度 → 底盘命令层 的唯一换算点（候选版，未部署）。

链路：controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed
      → nav_relay → /nav_cmd_vel → 桥 arbiter(/nav_cmd_vel, /manual_cmd_vel,
      /auto_cmd_vel, /drive_safety) → c50c_uart3 固件帧。

为什么订阅 /cmd_vel_smoothed 而不是 /cmd_vel_nav：
  Humble 的 nav2_bringup/launch/navigation_launch.py（容器内 /opt/ros/humble，
  第 182-183 行与 241-242 行）把 velocity_smoother 的 ('cmd_vel_smoothed','cmd_vel')
  重映射掉了——smoother 实际发布到 /cmd_vel，所以此前"没看到 /cmd_vel_smoothed"
  （probe-board.txt `/cmd_vel_smoothed info` 行为空）。本区的
  speed_chain.launch.py 用 ['/tf','tf'], ['/tf_static','tf_static'],
  ('cmd_vel_smoothed','/cmd_vel_smoothed') 覆盖该重映射，smoother 真正发到
  /cmd_vel_smoothed，本 relay 的默认输入才成立。

量纲/符号（交接文档 §4，实测常数）：
  - Nav2 输出 REP-103：linear.x = m/s，angular.z = rad/s。
  - 底盘命令层 angular.z 量纲 = 轮差速（左/右轮速差，m/s），且符号与 REP-103 相反。
    实测：一条 +0.8 的命令层 vz ≈ 1.93 rad/s 车体转向（增益 2.41），
    rad/s = 2.41 × 命令层_vz。
  - linear.x：开环效率 0.78（cmd 0.4 → 实走 0.31 m/s），故命令层 vx = nav_vx / 0.78。
  - 命令层 vz = -nav_wz / 2.41（取负 + 增益补偿；若漏取负，则命令层 -0.8 → +1.93 rad/s，
    是相反方向的旋转——首次试验必须实测确认这一符号）。

安全策略：
  - 只做"补偿换算"，不做放大：|nav linear.x| 与 |nav angular.z| 相加不得超过 MAX_OUT_MPS
    （0.15），否则整条消息拒发；换算后仍超 SOLO 上限（vx/wz 各 0.15）也整条拒发。
    这样单条命令不会因为 1/0.78 被放大到 >0.15 m/s。
  - 上游消息超过 STALE_TIMEOUT_S 未更新 → 发一次零速后静默（smoother 是 20Hz 持续发布，
    停发即视为上游异常）。
  - 退出（SIGINT/SIGTERM/异常）时发零速（尽力而为），失败只记日志。
  - 启动时要求 ROS 环境已 source：rclpy 导入失败 → 打印可诊断错误并 exit 2。
"""
from __future__ import annotations

import argparse
import logging
import math
import os
import signal
import sys
import time

# ---- 实测常数（交接文档 §4）------------------------------------------------
LIN_EFF = 0.78          # 开环效率：命令层 vx = nav_vx / 0.78
ANG_GAIN = 2.41         # rad/s → 命令层轮差速：命令层 vz = -nav_wz / 2.41
# 初次调试硬上限，全部以 Nav2 REP-103 量纲计：
#   上游 linear.x（m/s）≤ 0.15 → 补偿后命令层 vx ≤ 0.15/0.78 = 0.192（车实走 ≈0.15 m/s）
#   上游 angular.z（rad/s）≤ 0.15 → 命令层 |vz| ≤ 0.15/2.41 = 0.062
# 含义：以“实走等效速度”为上限，而不是把命令层数值钳在 0.15。
MAX_OUT_MPS = 0.15
MAX_OUT_RADPS = 0.80
STALE_TIMEOUT_S = 1.0
ZERO_EPS = 1e-6

LOG = logging.getLogger('nav_relay')


def convert(nav_vx: float, nav_wz: float, *,
            lin_eff: float = LIN_EFF, ang_gain: float = ANG_GAIN,
            max_out: float = MAX_OUT_MPS,
            max_ang: float | None = None,
            turn_priority_wz: float = 0.0,
            force_pivot: bool = False,
            allow_reverse: bool = False
            ) -> tuple[float, float, str | None]:
    """Nav2 (m/s, rad/s) → 底盘命令层 (vx m/s, vz 轮差速 m/s)。

    线性速度上限始终按 max_out（默认 0.15 m/s）；角速度上限按 max_ang
    （未指定时沿用 max_out，兼容既有单测；传入 --max-ang 时独立放宽到
    <= MAX_OUT_RADPS 以便克服底层 PWM_MIN_CMD=3360 转向死区）。
    当 turn_priority_wz > 0 且 |nav_wz| >= turn_priority_wz 时，压零前进
    vx（只保留原地转向），避免底层 PWM_MIN_CMD=3360 把左/右轮同时抬平
    成 0.23 m/s 直冲进障碍物。
    换算本身只补偿开环效率/转向增益（vx=nav_vx/0.78、vz=-nav_wz/2.41）。

    倒车（nav_vx < 0）默认拒发：倒车方向未验证（与 nav2_params.yaml 的
    min_vel_x=0 / velocity_smoother min_velocity[0]=0 同一口径），需显式
    allow_reverse=True（--allow-reverse）才放行。

    返回 (vx, vz, reject_reason)；reject_reason 非 None 时调用方必须发零速。
    纯函数、无 ROS 依赖，供 check_nav2.py 与单测直接调用。
    """
    ang_limit = max_out if max_ang is None else max_ang
    for name, v in (('nav_vx', nav_vx), ('nav_wz', nav_wz)):
        if not math.isfinite(v):
            return 0.0, 0.0, f'{name} 非有限值: {v!r}'
    if lin_eff <= 0.0 or ang_gain <= 0.0 or max_out <= 0.0 or ang_limit <= 0.0:
        return 0.0, 0.0, '换算常数非法'
    if not allow_reverse and nav_vx < -ZERO_EPS:
        return 0.0, 0.0, (f'倒车方向未验证：nav_vx={nav_vx:.3f} m/s < 0'
                          f'（需 --allow-reverse 才放行）')
    if abs(nav_vx) > max_out:
        return 0.0, 0.0, f'上游 linear.x={nav_vx:.3f} 超出首调上限 ±{max_out} m/s'
    if abs(nav_wz) > ang_limit:
        return 0.0, 0.0, f'上游 angular.z={nav_wz:.3f} rad/s 超出首调上限 ±{ang_limit} rad/s'
    in_pivot = force_pivot or (turn_priority_wz > 0.0 and abs(nav_wz) >= turn_priority_wz)
    eff_vx = 0.0 if in_pivot else nav_vx
    vx = eff_vx / lin_eff
    wz = -nav_wz / ang_gain
    return vx, wz, None


def _diag_no_ros(exc) -> None:
    sys.stderr.write(
        '[nav_relay][FATAL] 无法导入 ROS 模块: %s\n'
        '  原因：进程启动前没有 source ROS 环境，rclpy 不在 PYTHONPATH 中。\n'
        '  修复：source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash\n'
        '  当前 PYTHONPATH=%s\n'
        '  当前 AMENT_PREFIX_PATH=%s\n'
        % (exc, os.environ.get('PYTHONPATH', '<未设置>'),
           os.environ.get('AMENT_PREFIX_PATH', '<未设置>'))
    )


def _import_ros(args):
    """导入 ROS；失败时给出可诊断信息并以 2 退出。"""
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
    except ModuleNotFoundError as exc:
        _diag_no_ros(exc)
        sys.exit(2)
    return rclpy, Node, Twist


class NavRelay:
    def __init__(self, node, Twist, args):
        self.node = node
        self.Twist = Twist
        self.args = args
        self.pub = node.create_publisher(Twist, args.out, 10)
        # 用 rclpy 默认 QoS（RELIABLE、VOLATILE、depth 10）。smoother 发布端 QoS 未在
        # 容器内核对（无源码）；部署预检须 `ros2 topic info /cmd_vel_smoothed -v`
        # 确认两端 QoS 对齐后再动车，未对齐时此处要显式改成 BEST_EFFORT。
        self.sub = node.create_subscription(Twist, args.input, self.on_cmd, 10)
        self.zero_sent_after_stale = False
        self.last_rx = None
        self.in_turn_pivot = False
        self.counts = {'rx': 0, 'sent': 0, 'rejected': 0, 'zero': 0}
        node.create_timer(0.2, self.watchdog)
        LOG.info('nav_relay 启动：sub=%s pub=%s LIN_EFF=%.3f ANG_GAIN=%.3f '
                 'MAX_OUT=%.3f m/s MAX_ANG=%.3f rad/s TURN_PRIO=%.3f TURN_REL=%.3f STALE=%.1fs zero_on_exit=%s allow_reverse=%s',
                 args.input, args.out, args.lin_eff, args.ang_gain,
                 args.max_out,
                 args.max_out if args.max_ang is None else args.max_ang,
                 getattr(args, 'turn_priority_wz', 0.0),
                 getattr(args, 'turn_release_wz', 0.0),
                 args.stale_timeout, args.zero_on_exit,
                 getattr(args, 'allow_reverse', False))

    # ---- 内部 -------------------------------------------------------------
    def _publish(self, vx: float, wz: float) -> None:
        msg = self.Twist()
        msg.linear.x = float(vx)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)
        self.pub.publish(msg)

    def publish_zero(self, why: str) -> None:
        self._publish(0.0, 0.0)
        self.counts['zero'] += 1
        LOG.info('发送零速 nav_cmd_vel（原因：%s）', why)

    # ---- 回调 -------------------------------------------------------------
    def on_cmd(self, msg) -> None:
        self.counts['rx'] += 1
        self.last_rx = time.monotonic()
        self.zero_sent_after_stale = False
        nav_vx = float(msg.linear.x)
        nav_wz = float(msg.angular.z)
        tp = float(getattr(self.args, 'turn_priority_wz', 0.0) or 0.0)
        tr = float(getattr(self.args, 'turn_release_wz', 0.0) or 0.0)
        if tp > 0.0:
            rel = tr if (0.0 < tr < tp) else tp
            if abs(nav_wz) >= tp:
                self.in_turn_pivot = True
            elif abs(nav_wz) <= rel:
                self.in_turn_pivot = False
        else:
            self.in_turn_pivot = False
        vx, wz, reason = convert(nav_vx, nav_wz,
                                 lin_eff=self.args.lin_eff,
                                 ang_gain=self.args.ang_gain,
                                 max_out=self.args.max_out,
                                 max_ang=getattr(self.args, 'max_ang', None),
                                 turn_priority_wz=tp,
                                 force_pivot=self.in_turn_pivot,
                                 allow_reverse=getattr(self.args, 'allow_reverse', False))
        if reason is not None:
            self.counts['rejected'] += 1
            LOG.warning('拒发并归零：%s', reason)
            self.publish_zero('上游命令超限/未验证方向')
            return
        self._publish(vx, wz)
        self.counts['sent'] += 1
        if self.counts['sent'] % 20 == 1:
            LOG.info('nav_in=(%.3f m/s, %.3f rad/s) -> 命令层=(%.3f, %.3f) 轮差单位见 README '
                     '[rx=%d sent=%d rejected=%d]',
                     nav_vx, nav_wz, vx, wz,
                     self.counts['rx'], self.counts['sent'], self.counts['rejected'])

    def watchdog(self) -> None:
        if self.last_rx is None or self.zero_sent_after_stale:
            return
        if time.monotonic() - self.last_rx > self.args.stale_timeout:
            self.zero_sent_after_stale = True
            self.in_turn_pivot = False
            LOG.warning('上游 %s 已 %.1fs 无消息 → 发零速并静默（relay 只在收到消息时转发）',
                        self.args.input, self.args.stale_timeout)
            self.publish_zero('上游超时')

    def shutdown_zero(self, why: str) -> None:
        if not self.args.zero_on_exit:
            return
        try:
            self.publish_zero(why)
        except Exception as exc:  # noqa: BLE001
            LOG.error('退出时零速发布失败：%s', exc)


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Nav2 速度中继：/cmd_vel_smoothed → /nav_cmd_vel（含单位/符号换算）')
    # 所有默认值都显式写出并在启动日志里回显，避免"默认值不明"
    p.add_argument('--in', dest='input', default='/cmd_vel_smoothed',
                   help='订阅话题（默认 /cmd_vel_smoothed = velocity_smoother 的真实输出）')
    p.add_argument('--out', default='/nav_cmd_vel',
                   help='发布话题（默认 /nav_cmd_vel，桥 arbiter 导航通道）')
    p.add_argument('--lin-eff', type=float, default=LIN_EFF,
                   help='开环效率（默认 %.2f，实测）' % LIN_EFF)
    p.add_argument('--ang-gain', type=float, default=ANG_GAIN,
                   help='rad/s→轮差速增益（默认 %.2f，实测；符号内部取负）' % ANG_GAIN)
    p.add_argument('--max-out', type=float, default=MAX_OUT_MPS,
                   help='线速度硬上限 m/s（默认 %.2f）' % MAX_OUT_MPS)
    p.add_argument('--max-ang', type=float, default=None,
                   help='角速度上限 rad/s（默认跟随 --max-out；最高 %.2f）' % MAX_OUT_RADPS)
    p.add_argument('--turn-priority-wz', dest='turn_priority_wz', type=float, default=0.0,
                   help='转向优先进入阈值 rad/s（|wz|>=阈值时压零前进 vx；0=禁用）')
    p.add_argument('--turn-release-wz', dest='turn_release_wz', type=float, default=0.0,
                   help='转向优先退出迟滞阈值 rad/s（|wz|<=阈值才恢复直行，减少顿转抖动）')
    p.add_argument('--stale-timeout', dest='stale_timeout', type=float,
                   default=STALE_TIMEOUT_S,
                   help='上游超时秒数（默认 %.1f）' % STALE_TIMEOUT_S)
    p.add_argument('--allow-reverse', dest='allow_reverse', action='store_true',
                   help='放行倒车（linear.x<0）。默认关闭：倒车方向未验证')
    p.add_argument('--no-zero-on-exit', dest='zero_on_exit', action='store_false',
                   help='退出时不发零速（默认发）')
    p.set_defaults(zero_on_exit=True, allow_reverse=False)
    p.add_argument('--verbose', action='store_true', help='DEBUG 日志')
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format='[nav_relay][%(levelname)s] %(message)s')
    # 环境自检（在导入 ROS 之前，便于 diagnose）
    if not os.environ.get('AMENT_PREFIX_PATH'):
        # 环境变量缺失 → 与"未 source 即启动"同一可诊断错误路径（exit 2）
        _diag_no_ros('AMENT_PREFIX_PATH 未设置（未 source /opt/ros/humble/setup.bash）')
        return 2
    if not hasattr(args, 'stale_timeout') or args.stale_timeout <= 0:
        sys.stderr.write('[nav_relay][FATAL] --stale-timeout 必须为正数\n')
        return 2
    if args.max_out > MAX_OUT_MPS:
        LOG.warning('--max-out=%.3f 超出首调上限 %.2f，已钳到上限', args.max_out, MAX_OUT_MPS)
        args.max_out = MAX_OUT_MPS
    if args.max_ang is not None and args.max_ang > MAX_OUT_RADPS:
        LOG.warning('--max-ang=%.3f 超出角速度上限 %.2f，已钳到上限', args.max_ang, MAX_OUT_RADPS)
        args.max_ang = MAX_OUT_RADPS
    try:
        rclpy, Node, Twist = _import_ros(args)
    except SystemExit:
        raise
    except ImportError as exc:   # 部分安装缺共享库时抛 ImportError 而非 ModuleNotFoundError
        _diag_no_ros(exc)
        return 2
    rclpy.init(args=None)
    node = Node('nav_relay')
    try:
        # 构造放在 try 内：任何构造期异常都要先发零速 + 非零退出，而不是裸崩
        relay = NavRelay(node, Twist, args)
    except BaseException as exc:  # noqa: BLE001
        LOG.error('relay 构造失败：%s: %s', type(exc).__name__, exc)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 3
    stop = {'flag': False}

    def _sig(signum, _frame):
        stop['flag'] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    try:
        while not stop['flag'] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as exc:  # noqa: BLE001
        LOG.error('主循环异常退出：%s', exc)
        relay.shutdown_zero('异常退出')
        return 1
    finally:
        relay.shutdown_zero('收到停止信号')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
