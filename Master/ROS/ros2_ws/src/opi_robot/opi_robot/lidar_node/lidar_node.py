"""雷达节点：发布 LaserScan 与独立的雷达健康诊断。

第一阶段改造要点：
- 串口用稳定的 ``/dev/serial/by-path/`` 路径，不依赖 ``/dev/ttyUSB0`` 编号。
- 读写错误、帧超时后关闭旧描述符，按节流间隔重开。
- 只有 ``LidarRunner`` 判定为 healthy 时才发布 ``/scan``。
  断线期间停发扫描，持续发布 ``/diagnostics``。
- 绝不把上一轮的旧测距配上新时间戳再发布。
- 每次发布 ``/scan`` 时同步发布 ``scan_raw_evidence``（String JSON：
  epoch / frames_ok / max_point_age_ms）。健康窗口内重发的缓存圈
  ``frames_ok`` 不增长，消费方据此区分「原始帧在流动」与「缓存重发」。
  epoch 在节点构造时生成，重启后变更，消费方据此重建基线。

端口选择规则：

- ``lidar_port`` 非空：只用它，不扫描。
- ``lidar_port`` 为空：每次重连尝试前重新扫一次 ``/dev/serial/by-path/*usb*``。
  0 个候选保持空并停在 missing；恰好 1 个候选才采用；
  多个候选拒绝盲选，打错误日志列出候选并保持 disconnected。

QoS：``/scan`` 用 :func:`opi_robot.qos.sensor_qos`，``/diagnostics`` 用
:func:`opi_robot.qos.status_qos`。发布端与订阅端必须同源。

诊断话题为 ``/diagnostics``（``diagnostic_msgs/DiagnosticArray``），
状态名为 ``lidar``，发布周期由 ``diag_period`` 参数控制。
"""

import glob
import json
import os
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .. import qos as Q
from .. import diagnostic_level as DL
from . import health as H
from .reader import DEFAULT_STALE_POINT_AGE, LidarRunner

PORT_GLOB = '/dev/serial/by-path/*usb*'


def _scan_port_candidates():
    """列出 by-path 下的 USB 串口候选，按路径排序。

    找不到时返回空列表。只返回路径，不挑选。
    """
    return sorted(glob.glob(PORT_GLOB))


def _resolve_port(candidates):
    """按候选列表决定端口。

    返回 ``(port, status)``。``status`` 取值：

    - ``'explicit'``   调用方给了显式端口，未扫描
    - ``'missing'``    0 个候选，port 为空
    - ``'unique'``     恰好 1 个候选，采用它
    - ``'ambiguous'``  多个候选，拒绝盲选，port 为空
    """
    cands = list(candidates)
    if not cands:
        return '', 'missing'
    if len(cands) == 1:
        return cands[0], 'unique'
    return '', 'ambiguous'


def _default_port_pattern():
    """按 by-path 规则选 USB 串口。只在恰好一个候选时返回路径。"""
    port, _ = _resolve_port(_scan_port_candidates())
    return port


class LidarNode(Node):

    def __init__(self, context=None):
        # context 用于测试脚本在独立域里建节点（见 scripts/ros_isolation.py）。
        # 生产启动走 launch，不传 context，行为不变。
        super().__init__('lidar_node', context=context)
        # 单调时钟来源；测试可在构造前替换成假时钟
        self._mono_source = time.monotonic

        self.declare_parameter('lidar_port', '')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_timeout', 0.5)
        self.declare_parameter('recovery_coverage', 0.9)
        self.declare_parameter('reconnect_interval', 1.0)
        self.declare_parameter('log_interval', 10.0)
        self.declare_parameter('diag_period', 1.0)
        self.declare_parameter('poll_period', 0.01)
        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('origin', 'mirror')
        self.declare_parameter('bins', 360)
        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('stale_point_age', DEFAULT_STALE_POINT_AGE)
        self.declare_parameter('frame_id', 'laser')

        gp = self.get_parameter
        self.explicit_port = str(gp('lidar_port').value or '').strip()
        self.port_status = 'explicit' if self.explicit_port else 'missing'
        port = self.explicit_port
        if not self.explicit_port:
            # 构造期只记录状态，实际扫描交给第一次 poll：
            # 那时 runner 已存在，逻辑只剩一条路径
            port = ''
        elif not os.path.exists(port):
            self.get_logger().error(
                '雷达串口 %s 当前不存在，节点会按重连间隔重试' % port)

        self.frame_id = gp('frame_id').value
        # 首次 poll 必扫一次
        self._first_poll = not self.explicit_port
        self.runner = LidarRunner(
            port=port,
            baud=gp('baudrate').value,
            frame_timeout=gp('frame_timeout').value,
            recovery_coverage=gp('recovery_coverage').value,
            reconnect_interval=gp('reconnect_interval').value,
            log_interval=gp('log_interval').value,
            mirror=gp('origin').value == 'mirror',
            clock=self._clock,
            open_port=_open_serial,
            path_exists=os.path.exists,
            bins=int(gp('bins').value),
            angle_offset_deg=gp('angle_offset_deg').value,
            origin=gp('origin').value,
            range_min=gp('range_min').value,
            range_max=gp('range_max').value,
            stale_point_age=float(gp('stale_point_age').value),
            # 状态机与测距过期判定一律走单调时钟：ROS 时钟可能是仿真时间，
            # 也会被 NTP 跳变影响，回跳时超时会永远不触发
            mono=self._mono,
        )

        self.pub_scan = self.create_publisher(LaserScan, 'scan', Q.sensor_qos())
        self.pub_diag = self.create_publisher(DiagnosticArray, 'diagnostics',
                                              Q.status_qos())
        # 原始帧证据话题：与 /scan 同源 QoS（BEST_EFFORT），只在发布 /scan
        # 的同一轮回调里发布，保证两条话题可用同一对时限关联。
        # epoch 用本节点单调时钟（毫秒，掩 31 位）：lidar 节点重启后必然
        # 变化，消费方据此重建基线；测试可整体替换时钟源。
        self._evidence_epoch = int(self._mono() * 1000.0) & 0x7FFFFFFF
        self.pub_evidence = self.create_publisher(String, 'scan_raw_evidence',
                                                  Q.sensor_qos())

        # 保存句柄：测试脚本需要取消定时器，避免后台轮询与诊断继续跑。
        self.poll_timer = self.create_timer(gp('poll_period').value,
                                            self.on_poll)
        self.diag_timer = self.create_timer(gp('diag_period').value,
                                            self.publish_diag)
        self.get_logger().info('lidar_node 启动: port=%s baud=%d'
                               % (port, int(gp('baudrate').value)))

    # ---------- 端口 ----------

    def _scan_port(self):
        """扫一次 by-path，按规则更新 runner.port。返回当前端口。"""
        cands = _scan_port_candidates()
        port, status = _resolve_port(cands)
        previous = getattr(self, 'port_status', None)
        self.port_status = status
        if status == 'unique':
            if previous != 'unique' or self._current_port() != port:
                self.get_logger().info('雷达串口自动选定: %s' % port)
        elif status == 'ambiguous':
            self.get_logger().error(
                '找到 %d 个 USB 串口候选，拒绝盲选: %s；请用 lidar_port 明确指定'
                % (len(cands), ', '.join(cands)))
        elif status == 'missing':
            self.get_logger().error(
                '未找到 USB 串口 by-path 路径（模式 %s），保持未连接，'
                '每次重连重新扫描' % PORT_GLOB)
        return port

    def _current_port(self):
        return self.runner.port if self.runner is not None else ''

    def _current_status(self):
        """当前端口状态。诊断消息用，便于现场区分 missing 与 ambiguous。"""
        if self.explicit_port:
            return 'explicit'
        if self._current_port():
            return 'scanning'
        return self.port_status

    def _rescan_if_needed(self):
        """只在真正进入一次重连尝试时重扫一次设备树。

        判据是 runner 的重连节流：节流允许打开时才算一次尝试。不能用
        ``open_attempts`` 计数，它在 poll 里递增，节点读到的是上一轮的值，
        重扫会刚好晚一个周期，设备后插上要多等一拍。

        顺序必须是先重扫再 poll：扫描结果写进 runner.port 后，同一轮的
        ``_try_open`` 才能用上新路径。

        已连上时 ``runner.serial`` 非空，``_try_open`` 直接返回，不扫描。
        """
        if self.explicit_port:
            return
        if self._first_poll:
            # 构造期没扫过：首次 poll 无条件扫一次
            self._first_poll = False
            self.runner.port = self._scan_port()
            return
        throttle = self.runner.throttle
        if not throttle.allow(self.runner.now()):
            return
        self.runner.port = self._scan_port()

    # ---------- 时钟 ----------

    def _clock(self):
        """保留 ROS 时钟，只用于消息头时间戳。

        状态机、帧超时、测距过期的判定一律用单调时钟，
        不要把这个返回值传给 LidarRunner.clock()。
        """
        return self.get_clock().now()

    def _mono(self):
        """单调时钟秒。测试可替换 ``self._mono_source`` 注入假时钟。"""
        return self._mono_source()

    # ---------- 定时器 ----------

    def on_poll(self):
        # 空端口时每次重连前重新扫描，设备后插上也能拿到路径
        self._rescan_if_needed()
        scan = self.runner.poll(self.get_logger().info)
        self.port_status = self._current_status()
        if scan is None:
            # 不健康时停发扫描，旧测距不会配上新时间戳流出去
            return
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = scan['angle_min']
        msg.angle_max = scan['angle_max']
        msg.angle_increment = scan['angle_increment']
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = scan['range_min']
        msg.range_max = scan['range_max']
        msg.ranges = scan['ranges']
        msg.intensities = []
        self.pub_scan.publish(msg)
        self._publish_evidence(scan.get('max_point_age_s'))

    def _publish_evidence(self, max_point_age_s=None):
        """发布原始帧证据（与刚发出的 /scan 同一 poll 轮次）。

        ``frames_ok`` 是健康监视器的原始好帧累计数：只在解出新原始帧时
        增长，缓存圈重发不增长。``max_point_age_ms`` 透传
        ``_build_scan`` 算出的有效点最大年龄；无有效点时为 None。
        """
        payload = {
            'v': 1,
            'epoch': self._evidence_epoch,
            'frames_ok': int(self.runner.monitor.frames_ok),
            'max_point_age_ms': (None if max_point_age_s is None
                                 else int(round(max_point_age_s * 1000.0))),
        }
        msg = String(data=json.dumps(payload, separators=(',', ':')))
        self.pub_evidence.publish(msg)

    def halt_timers(self):
        """取消所有定时器并关闭串口。供测试脚本与异常退出使用。

        返回被取消的定时器数量，方便调用方断言确实停了。
        """
        count = 0
        for name in ('poll_timer', 'diag_timer'):
            timer = getattr(self, name, None)
            if timer is None:
                continue
            try:
                timer.cancel()
                count += 1
            except Exception:                    # noqa: BLE001
                pass
        try:
            self.runner._close_port()
        except Exception:                        # noqa: BLE001
            pass
        return count

    def publish_diag(self):
        snap = self.runner.snapshot()
        status = DiagnosticStatus()
        status.name = 'lidar'
        status.hardware_id = snap['port'] or 'none'
        # 真实 diagnostic_msgs 的 level 是单字节 byte 字段，必须用
        # bytes 赋值；直接给整数会在序列化时抛 AssertionError。
        status.level = DL.encode_level(H.diagnostic_level(snap['state']))
        status.message = '%s (%s)' % (snap['state'], snap['reason'])
        pairs = [
            ('state', snap['state']),
            ('reason', snap['reason']),
            ('age_s', '' if snap['age_s'] is None else '%.3f'
             % snap['age_s']),
            ('frames_ok', str(snap['frames_ok'])),
            ('frames_bad', str(snap['frames_bad'])),
            ('bytes_dropped', str(snap['bytes_dropped'])),
            ('coverage', '%.3f' % snap['coverage']),
            ('reconnects', str(snap['reconnects'])),
            ('port', snap['port']),
            ('origin', snap['origin']),
            ('connected', '1' if snap['connected'] else '0'),
            ('last_error', snap['last_error']),
            ('port_status', getattr(self, 'port_status', 'explicit')),
        ]
        status.values = [KeyValue(key=k, value=v) for k, v in pairs]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self.pub_diag.publish(arr)


def _open_serial(port, baud):
    """打开串口。只在有真实 pyserial 时可用，测试用替身替换。"""
    import serial  # 延迟导入，避免纯逻辑测试依赖 pyserial
    return serial.Serial(port, baud, timeout=0.0, write_timeout=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
