"""仲裁行为：停车、过期、来源不明、非法值、限速、零速豁免。

每个测试只验证一条规则（对应任务书 B 第 3 条与已审核方案 §3.2.1）。
"""
import math

import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame
from c50c_uart3.chassis_cmd.arbiter import Arbiter, ZeroReason

ZERO = encode_speed_frame(0.0, 0.0, 0.0)

def zero(seq: int = 0) -> bytes:
    """协议 v3 的零速帧：buffer[2] = 0x80 | 序号。"""
    return encode_speed_frame(0.0, 0.0, 0.0, seq=seq)



def fresh(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5):
    """已授权的仲裁器：先输出一帧零速（首帧零速规则），再交给测试。

    真实桥没有 ``on_firmware_confirm(True)`` 的调用点；这里显式调用是
    为了让"授权之后"的行为可测，并在测试名里标明这是模拟授权。
    2026-11-14 起还有安全许可门：本助手同时模拟"fusion 在线且安全"
    （生产许可来自 /drive_safety；许可门行为由 test_safety_gate.py 验证）。
    """
    a = Arbiter(nav_timeout_s=nav_timeout_s, nav_cmd_max_age_s=nav_cmd_max_age_s,
                safety_max_age_s=3600.0)
    a.on_safety_permission(True, 'test_granted', 3600.0, 0.0)
    a.on_publisher_count(1, 0)
    a.on_firmware_confirm(True)
    a.mark_receiver_clean(simulated=True)  # 第五轮：干净接收端
    first = a.evaluate(0.0)
    assert first.is_zero
    return a


def test_bridge_starts_with_zero_and_blocks_nonzero(clock):
    """启动：接收端未知 → 静默（frame=None）；证明干净后首帧才是零速。"""
    a = Arbiter()
    a.on_publisher_count(1, 0)
    a.mark_receiver_clean(simulated=True)
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    d = a.evaluate(clock())
    assert d.is_zero and d.frame == ZERO
    # 第十一轮：会话未观测时先报 NO_SESSION；会话已确认但未放行时报
    # OUTPUT_BLOCKED。两者都是"不放行非零"。2026-11-14 起安全许可门在
    # 会话门之前：未收到许可时报 SAFETY_UNKNOWN，同样不放行非零。
    assert d.zero_reason in (ZeroReason.OUTPUT_BLOCKED, ZeroReason.NO_SESSION,
                             ZeroReason.SAFETY_UNKNOWN)


def test_forward_command_encodes_after_authorisation(clock):
    a = fresh()
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    d = a.evaluate(clock())
    assert not d.is_zero
    # fresh() 已发出首帧零速(序号 0)，这条命令是序号 1。
    assert d.frame == encode_speed_frame(0.05, 0.0, 0.0, seq=1)


def test_stop_drops_nonzero_and_emits_zero(clock):
    a = fresh()
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    a.on_stop(True)
    d = a.evaluate(clock())
    # fresh() 只发出首帧零速(序号 0)；非零命令在停车边沿被丢弃、未编码，
    # 所以这条停车帧是序号 1。
    assert d.is_zero and d.frame == zero(1)
    assert d.zero_reason is ZeroReason.STOP_REQUESTED
    assert a._counts['dropped_stop'] == 1


def test_stop_wins_over_pre_stop_command(clock):
    """停车前到达的非零命令在停车**边沿**即被丢弃并计数（只计一次）。

    返工任务书 B：计数从"每个周期计数"改为"每次丢弃计数"，因为缓存已经
    在边沿丢弃，周期内不再有可计数的非零命令。
    """
    a = fresh()
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    a.on_stop(True)
    assert a._counts['dropped_stop'] == 1
    for _ in range(3):
        clock.advance(0.02)
        d = a.evaluate(clock())
        assert d.is_zero and d.zero_reason is ZeroReason.STOP_REQUESTED
    assert a._counts['dropped_stop'] == 1, '缓存已丢弃，不应继续计数' 


def test_stop_period_longer_than_timeout_ends_nonzero(clock):
    """停车超过 nav_timeout 后解除：旧命令已被丢弃，理由不再是 NAV_EXPIRED。

    返工前该测试要求旧缓存保留到过期，靠 ``NAV_EXPIRED`` 兜住。返工后缓存
    在停车边沿就丢弃，``NAV_EXPIRED`` 无从产生；此处按任务书 B 第 5 条
    更新为 ``NO_NAV_COMMAND``（缓存为空），且不为了保留旧理由再存旧命令。
    """
    a = fresh()
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    a.on_stop(True)
    clock.advance(0.35)
    assert a.evaluate(clock()).zero_reason is ZeroReason.STOP_REQUESTED
    a.on_stop(False)
    d = a.evaluate(clock())
    assert d.is_zero
    assert d.zero_reason is ZeroReason.NO_NAV_COMMAND


def test_nav_expiry_never_resends_nonzero(clock):
    a = fresh()
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    assert not a.evaluate(clock()).is_zero
    clock.advance(0.29)
    assert not a.evaluate(clock()).is_zero      # 未过期，仍可输出
    clock.advance(0.05)                         # 0.34 s > 0.3 s
    d = a.evaluate(clock())
    assert d.is_zero and d.zero_reason is ZeroReason.NAV_EXPIRED
    # 继续过期：一直零速，绝不出现非零
    for _ in range(5):
        clock.advance(0.1)
        assert a.evaluate(clock()).is_zero


def test_stale_new_command_rejected(clock):
    """自身过旧的新命令（上游阻塞后冲出）不放行。"""
    a = fresh()
    a.on_nav_cmd(0.05, 0.0, 0.0, clock() - 0.6)   # 迟到 0.6 s > 0.5 s
    d = a.evaluate(clock())
    # 2026-11-14 起过期来源在候选选择时跳过，统一报 NAV_EXPIRED
    assert d.is_zero and d.zero_reason is ZeroReason.NAV_EXPIRED


def test_ambiguous_sources_block_nonzero(clock):
    """发布者数超过上限才算来源不明。

    上限为 2（Web GUI 手动控制与自主壁障节点并存），所以 2 个允许，
    3 个才判不明。
    """
    from c50c_uart3.chassis_cmd.arbiter import MAX_NAV_PUBLISHERS
    a = fresh()
    a.on_publisher_count(MAX_NAV_PUBLISHERS + 1, 0)
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    d = a.evaluate(clock())
    assert d.is_zero and d.zero_reason is ZeroReason.AMBIGUOUS_SOURCES


def test_two_nav_publishers_are_allowed(clock):
    """两个导航发布者（网页手动 + 自主壁障）是常态，不得判来源不明。"""
    from c50c_uart3.chassis_cmd.arbiter import MAX_NAV_PUBLISHERS
    a = fresh()
    a.on_publisher_count(MAX_NAV_PUBLISHERS, 0)
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    d = a.evaluate(clock())
    assert not d.is_zero





@pytest.mark.parametrize('vx,vz', [(math.nan, 0.0), (0.0, math.inf),
                                   ('over', 0.0), (0.0, 'over')])
def test_invalid_or_overlimit_rejected(clock, vx, vz):
    """超限值按占位上限推算，不写死数字：改上限不应让本测试失效。"""
    from c50c_uart3.chassis_cmd import (PLACEHOLDER_MAX_ANGULAR_RAD_S,
                                        PLACEHOLDER_MAX_LINEAR_MPS)
    if vx == 'over':
        vx = PLACEHOLDER_MAX_LINEAR_MPS + 0.01
    if vz == 'over':
        vz = PLACEHOLDER_MAX_ANGULAR_RAD_S + 0.01
    a = fresh()
    # on_nav_cmd 的签名是 (vx, vy, vz, now)：角速度落在第 3 位。
    a.on_nav_cmd(vx, 0.0, vz, clock())
    d = a.evaluate(clock())
    assert d.is_zero and d.zero_reason is ZeroReason.NAV_INVALID


def test_placeholder_limit_is_not_a_calibrated_ceiling(clock):
    """占位限速只用于拒绝过大值，不代表已标定速度。"""
    from c50c_uart3.chassis_cmd import PLACEHOLDER_MAX_LINEAR_MPS
    a = fresh()
    a.on_nav_cmd(PLACEHOLDER_MAX_LINEAR_MPS, 0.0, 0.0, clock())
    assert not a.evaluate(clock()).is_zero
    a.on_nav_cmd(PLACEHOLDER_MAX_LINEAR_MPS + 0.001, 0.0, 0.0, clock())
    assert a.evaluate(clock()).is_zero


def test_zero_command_is_always_allowed(clock):
    """零速不受授权/过期门控；但接收端未知时同样静默（第五轮改判）。"""
    a = Arbiter()
    a.mark_receiver_clean(simulated=True)
    a.on_nav_cmd(0.0, 0.0, 0.0, clock())
    d = a.evaluate(clock())
    assert d.is_zero and d.frame == ZERO


def test_write_failure_latches_until_operator_clear(clock):
    a = fresh()
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    a.on_write_result(False)
    d = a.evaluate(clock())
    assert d.is_zero and d.zero_reason is ZeroReason.WRITE_FAILED
    # 后续写入成功也不能自动恢复
    a.on_write_result(True)
    assert a.evaluate(clock()).zero_reason is ZeroReason.WRITE_FAILED
    # 操作者显式复位后才允许再次评估
    a.clear_write_fault(by_operator=True)
    assert a.evaluate(clock()).zero_reason is ZeroReason.NO_NAV_COMMAND
