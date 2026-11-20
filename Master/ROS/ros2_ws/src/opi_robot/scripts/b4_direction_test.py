#!/usr/bin/env python3
"""B4 入口。默认静检；现场确认后，操作者用 --drive 执行原方向测试。"""
import argparse
import json
import re
import time


def new_session():
    return time.monotonic_ns()


class B4Test:
    def __init__(self, node):
        from std_msgs.msg import Bool, String
        from geometry_msgs.msg import Twist
        from rcl_interfaces.msg import Log
        self.node = node
        self.session = new_session()
        self.seq = 0
        self.licensing = False
        self.error = ''
        self.guard = None
        self.guard_at = None
        self.safety = None
        self.safety_at = None
        self.Bool, self.String, self.Twist = Bool, String, Twist
        self.license_pub = node.create_publisher(String, '/motion_license', 10)
        self.auth_pub = node.create_publisher(Bool, '/guard_authorize', 10)
        self.rearm_pub = node.create_publisher(Bool, '/rearm_request', 10)
        self.cmd_pub = node.create_publisher(Twist, '/nav_cmd_vel', 10)
        node.create_subscription(String, '/motion_license', self.on_license, 10)
        node.create_subscription(String, '/drive_safety', self.on_safety, 10)
        node.create_subscription(Log, '/rosout', self.on_log, 10)
        node.create_timer(0.1, self.publish_license)

    def on_license(self, msg):
        payload = json.loads(msg.data)
        if payload['session'] != self.session:
            self.error = '其他许可会话仍活动；先关闭网页控制会话和其他测试脚本'

    def on_safety(self, msg):
        self.safety = json.loads(msg.data)
        self.safety_at = time.monotonic()

    def on_log(self, msg):
        if msg.name != 'c50c_uart3_node' or not msg.msg.startswith('guard probe:'):
            return
        fields = dict(re.findall(r'(latched|rearms|license_accepted|license_rejected)=([^ ]+)', msg.msg))
        self.guard = {'latched': fields['latched'] == 'True',
                      'rearms': int(fields['rearms']),
                      'accepted': int(fields['license_accepted']),
                      'rejected': int(fields['license_rejected'])}
        self.guard_at = time.monotonic()

    def publish_license(self):
        if self.licensing:
            self.seq += 1
            self.license_pub.publish(self.String(data=json.dumps({
                'session': self.session, 'seq': self.seq,
                'expires_mono': time.monotonic() + 0.6})))

    def subscribers_ready(self):
        return all(pub.get_subscription_count() > 0 for pub in
                   (self.license_pub, self.auth_pub, self.rearm_pub, self.cmd_pub))

    def wait(self, condition, timeout, reason):
        import rclpy
        deadline = time.monotonic() + timeout
        while not condition():
            if self.error:
                raise RuntimeError(self.error)
            if time.monotonic() >= deadline:
                raise RuntimeError(reason)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        if self.error:
            raise RuntimeError(self.error)

    def prepare(self):
        self.wait(self.subscribers_ready, 3.0, 'DDS 发现超时；检查底盘订阅端')
        self.wait(lambda: self.guard is not None, 6.0, '未收到 guard 诊断，不能确认接收结果')
        if self.safety is None or not self.safety['allowed']:
            raise RuntimeError('安全门未放行；检查 /drive_safety，不自动复位超声故障')
        baseline = dict(self.guard)
        self.licensing = True
        self.wait(lambda: self.guard['accepted'] > baseline['accepted'], 6.0,
                  'guard 未确认接受本次许可')
        if self.guard['rejected'] != baseline['rejected']:
            raise RuntimeError('guard 拒绝了许可；保持停车并检查日志')
        before_rearm = dict(self.guard)
        self.auth_pub.publish(self.Bool(data=True))
        self.rearm_pub.publish(self.Bool(data=True))
        self.wait(lambda: not self.guard['latched'] and
                  self.guard['rearms'] > before_rearm['rearms'], 6.0,
                  'guard 未确认解停；保持停车并检查鉴权、许可和传感器')
        if self.guard['rejected'] != baseline['rejected']:
            raise RuntimeError('解停期间许可被拒绝；保持停车')
        print(json.dumps({'ready': True, 'session': self.session,
                          'guard': self.guard}, ensure_ascii=False), flush=True)

    def drive(self, vx, vz, seconds, label):
        import rclpy
        print(label, flush=True)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            now = time.monotonic()
            if self.error:
                raise RuntimeError(self.error)
            if (self.safety is None or not self.safety['allowed'] or
                    now-self.safety_at > self.safety['ttl_s']):
                raise RuntimeError('安全许可失效，结束测试')
            if self.guard['latched'] or now-self.guard_at > 6.0:
                raise RuntimeError('guard 锁停或诊断超时，结束测试')
            msg = self.Twist()
            msg.linear.x, msg.angular.z = vx, vz
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def finish(self):
        import rclpy
        self.licensing = False
        self.cmd_pub.publish(self.Twist())
        self.auth_pub.publish(self.Bool(data=False))
        end = time.monotonic()+0.3
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def run(self, drive=False):
        try:
            self.prepare()
            if drive:
                self.drive(0.15, 0.0, 2.0, 'PHASE 1：前进，观察实际方向')
                self.drive(0.0, 0.0, 1.5, 'PHASE 1：零速')
                self.drive(0.0, 0.15, 3.0, 'PHASE 2：正向转向，记录实际方向')
                self.drive(0.0, 0.0, 1.5, 'PHASE 2：零速')
        finally:
            self.finish()


def main(argv=None):
    import rclpy
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true', help='由现场操作者显式启动带动力方向测试')
    args = parser.parse_args(argv)
    rclpy.init()
    node = rclpy.create_node('b4_direction_test')
    try:
        B4Test(node).run(drive=args.drive)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
