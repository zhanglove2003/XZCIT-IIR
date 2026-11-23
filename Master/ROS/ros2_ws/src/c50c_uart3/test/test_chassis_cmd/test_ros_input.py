"""ROS 直行输入适配层测试（无 ROS 依赖，纯内存）。

覆盖任务书第七轮 C 第 2 条要求的六类场景：

1. 命令传递：ROS 消息 → 命令链 → 编码帧 → 内存假串口字节。
2. 无命令 / 过期：无消息时不输出非零；命令过期后降为零速。
3. 停车优先：停车激活期间非零命令不入缓存，解除后不能恢复。
4. 来源冲突：导航话题出现多个发布者时禁非零。
5. 禁用：``enabled=False`` 时所有输入被丢弃，链路拿不到命令。
6. 失联 / 重连：重连后禁非零，且适配层不会附带任何固件确认。

另有"不伪装确认"专项：断言适配层源码与状态里不存在把 ROS 收消息当成
固件确认的路径。
"""
import ast
import inspect

import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame, ros_input
from c50c_uart3.chassis_cmd.arbiter import Arbiter, ZeroReason
from c50c_uart3.chassis_cmd.chain import StraightCommandChain
from c50c_uart3.chassis_cmd.ros_input import MAX_TRACKED_PUBLISHERS, RosStraightInput
from c50c_uart3.chassis_cmd.transport import MemoryTransport
from sim_confirm import (mark_chain_clean, mark_endpoint_clean,
                         new_receipt as _new_receipt,
                         grant_safety, TEST_SAFETY_MAX_AGE_S)

_SIM = _new_receipt('ros-input')

ZERO = encode_speed_frame(0.0, 0.0, 0.0)

def zero(seq: int = 0) -> bytes:
    """协议 v3 的零速帧：buffer[2] = 0x80 | 序号。"""
    return encode_speed_frame(0.0, 0.0, 0.0, seq=seq)



# ---------------------------------------------------------------- 替身对象
class _Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)


class TwistLike:
    """``geometry_msgs/msg/Twist`` 的最小替身（无 ROS 依赖）。"""

    def __init__(self, vx=0.0, vy=0.0, wz=0.0):
        self.linear = _Vec(vx, vy, 0.0)
        self.angular = _Vec(0.0, 0.0, wz)


class BoolLike:
    """``std_msgs/msg/Bool`` 的最小替身。"""

    def __init__(self, data):
        self.data = data


class _Clock:
    def __init__(self, t=100.0):
        self.t = float(t)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)
        return self.t


# ---------------------------------------------------------------- 夹具
@pytest.fixture
def clock():
    return _Clock()


@pytest.fixture
def port():
    p = MemoryTransport()
    # 测试显式构造"端口无残帧"的证据；生产路径没有这个入口。
    mark_endpoint_clean(p, receipt=_SIM)
    return p


@pytest.fixture
def chain(port, clock):
    arbiter = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5,
                      safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arbiter, clock())
    return StraightCommandChain(arbiter, port)


@pytest.fixture
def ros(chain, clock):
    """启用状态下的适配层。注意：**没有**任何固件确认动作。

    第八轮起发布者数未知时适配层不放行非零，所以这里显式给出
    "导航话题恰好一个发布者"的图证据（第三、四节的自定义用例除外）。
    """
    adapter = RosStraightInput(chain, enabled=True, clock=clock)
    adapter.set_nav_publishers(1)
    adapter.set_demo_publishers(0)
    return adapter


def _authorise(chain, clock, vx=0.05):
    """走到与非零输出相邻的稳态：接收端干净 + 首拍零速 + 人工放行。

    注意：``firmware_confirm(True)`` 与 ``mark_clean_state_verified``
    都只在**测试**里调用，代表"已存在的独立证据"，不是本适配层提供的。
    """
    mark_chain_clean(chain, receipt=_SIM)
    chain.firmware_confirm(True)          # 测试构造的独立证据
    first = chain.tick(clock())
    assert first.is_zero and first.frame == ZERO
    return first


# ---------------------------------------------------------------- 1 命令传递
def test_nav_message_reaches_frame_bytes(ros, chain, port, clock):
    """ROS 消息 → 非零帧 → 内存假串口字节。"""
    _authorise(chain, clock)
    assert ros.on_nav_cmd(TwistLike(vx=0.05)) is True
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert not rec.is_zero
    # _authorise 已发出首帧零速(序号 0)，这是序号 1。
    assert rec.frame == encode_speed_frame(0.05, 0.0, 0.0, seq=1)
    assert port.writes[-1] == rec.frame
    assert ros.snapshot()['received_nav'] == 1


def test_demo_message_reaches_chain(ros, chain, clock, port):
    _authorise(chain, clock)
    assert ros.on_demo_cmd(TwistLike(vx=0.03)) is True
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert not rec.is_zero
    assert port.writes[-1] == encode_speed_frame(0.03, 0.0, 0.0, seq=1)


def test_vy_and_extra_axes_are_ignored(ros, chain, clock):
    """只取 linear.x / linear.y / angular.z；其它轴不影响编码。"""
    _authorise(chain, clock)
    msg = TwistLike(vx=0.0, vy=0.0, wz=0.0)
    msg.linear.z = 9.9
    msg.angular.x = 9.9
    ros.on_nav_cmd(msg)
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert rec.is_zero          # 三轴全零 → 零速，不是非法值


# ---------------------------------------------------------------- 2 无命令/过期
def test_no_message_stays_zero(chain, clock):
    _authorise(chain, clock)
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert rec.is_zero
    # 2026-11-14 起启用/撤销按通道提交零速，缓存非空但全为零。
    assert rec.zero_reason in (ZeroReason.NO_NAV_COMMAND,
                               ZeroReason.ALL_SOURCES_ZERO)


def test_expired_message_drops_to_zero(ros, chain, clock):
    _authorise(chain, clock)
    ros.on_nav_cmd(TwistLike(vx=0.05))
    clock.advance(0.02)
    assert not chain.tick(clock()).is_zero
    # 超过 nav_timeout_s（0.3 s）没有新消息：不重发
    clock.advance(0.4)
    before = len(chain.nonzero_frames())
    rec = chain.tick(clock())
    assert rec.is_zero
    assert rec.zero_reason in (ZeroReason.NAV_EXPIRED, ZeroReason.NAV_STALE)
    # 过期之后不能再写出任何非零帧（不重发旧命令）
    assert len(chain.nonzero_frames()) == before


def test_stale_on_arrival_is_rejected(ros, chain, clock):
    """命令自身过旧（到达时已超 nav_cmd_max_age_s）不放行。"""
    _authorise(chain, clock)
    old_now = clock()
    clock.advance(0.6)                  # 超过 nav_cmd_max_age_s = 0.5
    ros._clock = lambda: old_now        # 模拟上游阻塞后一次性冲出的旧命令
    ros.on_nav_cmd(TwistLike(vx=0.05))
    ros._clock = clock
    rec = chain.tick(clock())
    assert rec.is_zero
    # 2026-11-14 起过期来源在候选选择时跳过，统一报 NAV_EXPIRED
    #（0.6 s 同时超过 0.3 s 超时与 0.5 s 自身时效，后者不再单独可达）。
    assert rec.zero_reason is ZeroReason.NAV_EXPIRED


# ---------------------------------------------------------------- 3 停车优先
def test_stop_blocks_nonzero_immediately(ros, chain, clock, port):
    _authorise(chain, clock)
    ros.on_nav_cmd(TwistLike(vx=0.05))
    assert ros.on_stop(BoolLike(True)) is True
    rec = chain.tick(clock())
    assert rec.is_zero
    assert rec.zero_reason == ZeroReason.STOP_REQUESTED


def test_nonzero_during_stop_is_not_cached(ros, chain, clock):
    """停车期间到达的非零命令，解除停车后不能被恢复。"""
    _authorise(chain, clock)
    ros.on_stop(BoolLike(True))
    chain.tick(clock())
    ros.on_nav_cmd(TwistLike(vx=0.05))     # 停车期间到达 → 丢弃
    clock.advance(0.02)
    ros.on_stop(BoolLike(False))           # 解除
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert rec.is_zero
    # 按通道撤销后缓存非空但全为零：两种理由都表示没有非零出去。
    assert rec.zero_reason in (ZeroReason.NO_NAV_COMMAND,
                               ZeroReason.ALL_SOURCES_ZERO)


def test_new_command_after_release_moves(ros, chain, clock):
    _authorise(chain, clock)
    ros.on_stop(BoolLike(True))
    chain.tick(clock())
    ros.on_stop(BoolLike(False))
    clock.advance(0.02)
    ros.on_nav_cmd(TwistLike(vx=0.04))     # 解除之后新到达
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert not rec.is_zero
    # 序号：首帧零速(0) → 停车零速(1) → 本条命令(2)。
    assert rec.frame == encode_speed_frame(0.04, 0.0, 0.0, seq=2)


# ---------------------------------------------------------------- 4 来源冲突
def test_two_publishers_block_nonzero(ros, chain, clock):
    _authorise(chain, clock)
    ros.set_nav_publishers(MAX_TRACKED_PUBLISHERS + 1)
    ros.on_nav_cmd(TwistLike(vx=0.05))
    rec = chain.tick(clock())
    assert rec.is_zero
    # 第八轮起适配层在来源可疑时立即请求零速，所以理由可能是
    # ALL_SOURCES_ZERO；两种都表示"没有非零出去"。
    assert rec.zero_reason in (ZeroReason.AMBIGUOUS_SOURCES,
                              ZeroReason.ALL_SOURCES_ZERO)


def test_single_publisher_allows_nonzero(ros, chain, clock):
    _authorise(chain, clock)
    ros.set_nav_publishers(1)
    ros.on_nav_cmd(TwistLike(vx=0.05))
    clock.advance(0.02)
    assert not chain.tick(clock()).is_zero


# ---------------------------------------------------------------- 5 禁用
def test_disabled_input_is_dropped(chain, clock):
    ros = RosStraightInput(chain, enabled=False, clock=clock)
    _authorise(chain, clock)
    assert ros.on_nav_cmd(TwistLike(vx=0.05)) is False
    rec = chain.tick(clock())
    assert rec.is_zero
    assert rec.zero_reason == ZeroReason.NO_NAV_COMMAND
    assert ros.snapshot()['dropped_while_disabled'] == 1


def test_disabled_stop_is_dropped(chain, clock):
    ros = RosStraightInput(chain, enabled=False, clock=clock)
    assert ros.on_stop(BoolLike(True)) is False
    assert ros.snapshot()['stop_active'] is False


def test_disable_drops_later_messages(ros, chain, clock):
    _authorise(chain, clock)
    ros.disable()
    assert ros.on_nav_cmd(TwistLike(vx=0.05)) is False
    clock.advance(0.02)
    assert chain.tick(clock()).is_zero


def test_malformed_message_is_rejected(ros, chain, clock):
    _authorise(chain, clock)
    assert ros.on_nav_cmd(object()) is False           # 缺 linear/angular
    assert ros.on_stop(BoolLike(2.0)) is False         # 非法布尔
    assert ros.snapshot()['rejected_malformed'] == 2
    clock.advance(0.02)
    assert chain.tick(clock()).is_zero


# ---------------------------------------------------------------- 6 重连
def test_reconnect_blocks_nonzero(ros, chain, clock):
    _authorise(chain, clock)
    ros.on_nav_cmd(TwistLike(vx=0.05))
    ros.on_serial_reconnect()
    ros.on_nav_cmd(TwistLike(vx=0.05))
    clock.advance(0.02)
    rec = chain.tick(clock())
    assert rec.is_zero
    assert rec.zero_reason in (ZeroReason.OUTPUT_BLOCKED,
                               ZeroReason.BRIDGE_REARMING,
                               ZeroReason.NO_SESSION)


def test_reconnect_clears_ros_counters(ros, chain, clock):
    _authorise(chain, clock)
    ros.on_nav_cmd(TwistLike(vx=0.05))
    assert ros.snapshot()['received_nav'] == 1
    ros.on_serial_reconnect()
    snap = ros.snapshot()
    assert snap['received_nav'] == 0
    assert snap['last_nav_monotonic'] is None
    assert snap['firmware_confirmed'] is False


# ---------------------------------------------------------------- 不伪装确认
def test_adapter_source_has_no_confirm_calls():
    """源码层面断言：适配层不调用固件确认或模拟干净入口。

    用 AST 检查调用点，而不是文本匹配——模块文档会在说明里**提到**这些
    名字（写明"不调用"），文本匹配会把说明误判成调用。
    """
    tree = ast.parse(inspect.getsource(ros_input))
    called: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = node.func
            if isinstance(fn, ast.Attribute):
                called.add(fn.attr)
            elif isinstance(fn, ast.Name):
                called.add(fn.id)
    for forbidden in ('firmware_confirm', 'mark_clean_state_verified',
                      'mark_endpoint_clean_for_test', 'on_firmware_confirm',
                      'operator_clear'):
        assert forbidden not in called, forbidden


def test_adapter_state_is_not_firmware_confirmation(ros, chain, clock):
    """ROS 收到消息不改变 firmware_confirmed。"""
    _authorise(chain, clock)
    ros.on_nav_cmd(TwistLike(vx=0.05))
    assert ros.snapshot()['received_nav'] == 1
    assert ros.snapshot()['firmware_confirmed'] is False
