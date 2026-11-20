"""手动会话失效保护（任务书 T3 必测项 1、2）：假时钟驱动 ManualForwardControl。

覆盖：
- 心跳超过 0.35 s 有效期：下一次调度产生停车（零速发布一次），之后没有
  非零重发；
- timed 模式到期停车；
- 页面失焦/隐藏/关闭由前端发 manual_stop，服务端 stop() 与 disconnect()
  都进入停车；
- 连接中断（handler 对象消失）：disconnect 后 owner 清空，重新连接不能
  复用旧会话（必须重新登录）；
- 只有会话存活期间心跳才续期；停车后心跳不再触发发布。
"""

import pytest

from opi_robot.web_node.manual_forward import ManualForwardControl


class FakeClock:
    def __init__(self, t0=100.0):
        self.t = float(t0)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)
        return self.t


@pytest.fixture
def ctl():
    """就绪的手动控制器：publish 记录每次发布，ready 恒真。"""
    clk = FakeClock()
    c = ManualForwardControl(now=clk)
    published = []
    c.attach(lambda vx, wz: published.append((round(vx, 4), round(wz, 4))),
             ready=lambda: True)
    assert c.login('owner1', 'admin', 'password')
    return c, published, clk


def test_start_publishes_command_once(ctl):
    c, published, _clk = ctl
    c.start('owner1', 0.4, 'hold', direction='forward')
    assert published == [(0.4, 0.0)]


def test_heartbeat_within_window_keeps_moving(ctl):
    c, published, clk = ctl
    c.start('owner1', 0.4, 'hold', direction='forward')
    for _ in range(5):
        clk.advance(0.1)                 # 100 ms 心跳周期
        c.heartbeat('owner1')
        c.tick()
    assert published[-1] == (0.4, 0.0)
    assert all(vx == 0.4 for vx, _ in published)


def test_heartbeat_timeout_stops_and_no_nonzero_replay(ctl):
    """必测项 1：心跳超过 0.35 s，下一次调度产生停车，之后无非零重发。"""
    c, published, clk = ctl
    c.start('owner1', 0.4, 'hold', direction='forward')
    clk.advance(0.4)                     # 超过 HEARTBEAT_TIMEOUT=0.35
    c.heartbeat('owner1')                # 迟到的心跳：触发停车而不是续期
    assert published[-1] == (0.0, 0.0)
    assert c.snapshot()['active'] is False
    # 之后任何调度都不再发布非零
    for _ in range(5):
        clk.advance(0.1)
        c.heartbeat('owner1')
        c.tick()
    assert all(vx == 0.0 for vx, _ in published[1:])


def test_tick_alone_detects_dead_heartbeat(ctl):
    """心跳线程/连接消失时，周期调度自身必须能停车（不依赖新心跳）。"""
    c, published, clk = ctl
    c.start('owner1', 0.4, 'hold', direction='forward')
    clk.advance(0.4)                     # 没有任何心跳到达
    c.tick()
    assert published[-1] == (0.0, 0.0)
    assert c.snapshot()['active'] is False


def test_timed_mode_stops_at_deadline(ctl):
    c, published, clk = ctl
    c.start('owner1', 0.4, 'timed', duration=2.0, direction='forward')
    for _ in range(10):                  # 1 s 内正常心跳
        clk.advance(0.1)
        c.heartbeat('owner1')
        c.tick()
    assert published[-1] == (0.4, 0.0)
    clk.advance(1.5)                     # 超过 2 s 截止
    c.tick()
    assert published[-1] == (0.0, 0.0)


def test_manual_stop_publishes_zero(ctl):
    c, published, _clk = ctl
    c.start('owner1', 0.4, 'hold', direction='forward')
    c.stop('owner1')                     # 前端失焦/隐藏/关闭都走这条
    assert published[-1] == (0.0, 0.0)
    assert c.snapshot()['active'] is False


def test_disconnect_stops_and_revokes_session(ctl):
    """必测项 2：连接中断（handler 消失）→ 停车 + 会话作废。"""
    c, published, _clk = ctl
    c.start('owner1', 0.4, 'hold', direction='forward')
    c.disconnect('owner1')
    assert published[-1] == (0.0, 0.0)
    assert c.authenticated('owner1') is False
    assert c.snapshot()['active'] is False


def test_reconnect_cannot_reuse_old_session(ctl):
    """重连后旧 owner 身份不复用：未登录状态不能再次驱动。"""
    c, published, clk = ctl
    c.start('owner1', 0.4, 'hold', direction='forward')
    c.disconnect('owner1')
    with pytest.raises(PermissionError):
        c.start('owner1', 0.4, 'hold', direction='forward')
    # 失效会话的心跳是静默 no-op（服务端在调用前已检查登录），不得续期
    clk.advance(0.1)
    c.heartbeat('owner1')
    c.tick()
    assert c.snapshot()['active'] is False
    assert all(vx == 0.0 for vx, _ in published[1:])


def test_second_owner_rejected_while_session_active(ctl):
    c, _published, _clk = ctl
    c.start('owner1', 0.4, 'hold', direction='forward')
    assert c.login('owner2', 'admin', 'password')
    with pytest.raises(RuntimeError):
        c.start('owner2', 0.4, 'hold', direction='forward')


def test_stop_from_non_owner_only_requires_authentication(ctl):
    """停车请求不要求是 owner：任何已登录会话都能叫停（安全方向）。"""
    c, published, _clk = ctl
    c.start('owner1', 0.4, 'hold', direction='forward')
    assert c.login('owner2', 'admin', 'password')
    c.stop('owner2')
    assert published[-1] == (0.0, 0.0)
    assert c.snapshot()['active'] is False


class TestGearCeiling:
    """挡位上限（2026-11-14 用户确认）：服务端输入上限 = 最高挡 0.8。"""

    def test_top_gear_accepted(self, ctl):
        c, published, _clk = ctl
        c.start('owner1', 0.8, 'hold', direction='forward')
        assert published == [(0.8, 0.0)]

    def test_above_top_gear_rejected(self, ctl):
        """1.3 旧上限撤销：挡位之外的速度在服务端拒绝。"""
        c, published, _clk = ctl
        with pytest.raises(ValueError):
            c.start('owner1', 0.85, 'hold', direction='forward')
        with pytest.raises(ValueError):
            c.start('owner1', 0.85, 'hold', direction='left')
        assert published == []
