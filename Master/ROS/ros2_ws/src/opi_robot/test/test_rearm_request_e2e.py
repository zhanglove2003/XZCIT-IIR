"""B2 复核用例：网页"恢复（解除锁停）"按钮必须产生 /rearm_request 消息。

整树复核 R1 阻塞项 B2 称 ``patrol_downstream_rearm`` 只有定义没有调用点，
guard 永远停在 ``global_stop``/``rearm_pending``。本文件从**页面协议层**
（``motion_rearm`` ws 消息）到**下游通道发布器**做端到端验证：

1. 页面发 ``{'t': 'motion_rearm'}`` → web_debug 消息分支 → ``patrol_rearm``
   → 锁停解除成功时必须调用 ``patrol_downstream_rearm``；
2. ``patrol_downstream_rearm`` 必须在 ``/rearm_request`` 话题上发布
   ``Bool(data=True)``——这是下游 guard_bridge（guard_bridge.py:193
   ``on_rearm_request``）唯一的解停触发器。
"""

import json
import threading
import time
import unittest

from opi_robot.web_node import web_debug


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(getattr(msg, 'data', msg))


class _Node:
    def __init__(self):
        self.pubs = {}

    def create_publisher(self, typ, topic, qos):
        pub = _Pub()
        self.pubs[topic] = pub
        return pub


class RearmRequestE2ETests(unittest.TestCase):
    """B2：/rearm_request 必须有生产者（页面恢复按钮 → 下游话题）。"""

    def setUp(self):
        web_debug.configure_patrol(reset_runner=True)
        self.node = _Node()
        self.ds = web_debug.patrol_downstream_attach(self.node)
        if self.ds['rearm'] is None:
            self.skipTest('本环境没有 std_msgs（假模式）：下游通道为 no-op')
        self.addCleanup(self._reset_patrol)

    def _reset_patrol(self):
        web_debug.configure_patrol(reset_runner=True)

    # ------------------------------------------------ 静态结构
    def test_patrol_rearm_calls_downstream_rearm_on_success(self):
        """patrol_rearm 成功解除锁停后必须调用 patrol_downstream_rearm。"""
        runner = web_debug._patrol['runner']
        lic = web_debug._patrol['license']
        lic.accept(4242, 0)
        runner.release_lock('test')          # 锁停已解除：rearm 幂等成功
        ok, info = web_debug.patrol_rearm()
        assert ok, info
        assert self.ds['rearmsg'] >= 1, (
            'patrol_rearm 成功后未调用 patrol_downstream_rearm：'
            '/rearm_request 没有生产者（B2）')
        assert self.node.pubs[web_debug.REARM_TOPIC].sent == [True]

    def test_patrol_rearm_failure_does_not_forward(self):
        """许可无效时 rearm 被拒：不得向下游发解停请求（fail-closed）。"""
        runner = web_debug._patrol['runner']
        runner.release_lock('test')
        before = self.ds['rearmsg']
        ok, _info = web_debug.patrol_rearm()   # license 未 accept → fresh=False
        assert not ok
        assert self.ds['rearmsg'] == before

    def test_lock_stop_revokes_authorize_then_rearm_reauthorizes(self):
        """完整时序：锁停撤回鉴权 → 用户恢复 → 鉴权+解停都发到下游。"""
        runner = web_debug._patrol['runner']
        lic = web_debug._patrol['license']
        lic.accept(4242, 0)
        # 先让"网页会话有效"发到下游（license_message 入口会转许可+鉴权），
        # 这样 lock_stop 撤回鉴权才有False可发（去重逻辑只在变化时发）。
        web_debug.patrol_license_message(
            {'t': 'motion_license', 'session': 4242, 'seq': 1,
             'expires_mono': web_debug._patrol_now() + 0.5})
        runner.release_lock('test')
        web_debug.patrol_rearm()               # 第一次恢复
        runner.lock_stop('nav_failed')         # 导航失败锁停 → 撤回鉴权
        assert self.node.pubs[web_debug.AUTHORIZE_TOPIC].sent[-1] is False
        runner.release_lock('test')
        web_debug.patrol_rearm()               # 用户再点恢复
        pubs = self.node.pubs
        assert pubs[web_debug.REARM_TOPIC].sent[-1] is True
        assert pubs[web_debug.AUTHORIZE_TOPIC].sent[-1] is True


if __name__ == '__main__':
    unittest.main()


def test_explicit_rearm_resends_cached_authorization():
    """底盘重启或外部撤销后，显式恢复必须再次发送 True。"""
    web_debug.configure_patrol(reset_runner=True)
    try:
        node = _Node()
        web_debug.patrol_downstream_attach(node)
        web_debug._patrol['license'].accept(4242, 0)
        web_debug._patrol['runner'].release_lock('test')
        assert web_debug.patrol_rearm()[0]
        auth = node.pubs[web_debug.AUTHORIZE_TOPIC]
        rearm = node.pubs[web_debug.REARM_TOPIC]
        before_auth = len(auth.sent)
        before_rearm = len(rearm.sent)
        assert web_debug.patrol_rearm()[0]
        assert len(auth.sent) == before_auth + 1
        assert auth.sent[-1] is True
        assert len(rearm.sent) == before_rearm + 1
    finally:
        web_debug.configure_patrol(reset_runner=True)
