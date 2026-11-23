"""定向测试：停车解除后不得恢复旧非零命令（返工任务书 B）。

覆盖维度：

- 输入来源：导航（``nav``）与演示（``demo``）
- 命令时序：停车**前**收到，停车**期间**收到
- 求值时序：停车激活与解除之间**调用过** ``evaluate()``，以及**没有调用过**
- 正向对照：解除停车后**新**收到的命令可以输出
- 重复停车通知：不误清新命令

授权一律由 ``fresh()`` 的 ``on_firmware_confirm(True)`` 模拟。它只证明
检测模型行为，**不是固件确认**。
"""
import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame
from c50c_uart3.chassis_cmd.arbiter import Arbiter, ZeroReason
from sim_confirm import TEST_SAFETY_MAX_AGE_S, grant_safety

ZERO = encode_speed_frame(0.0, 0.0, 0.0)

def zero(seq: int = 0) -> bytes:
    """协议 v3 的零速帧：buffer[2] = 0x80 | 序号。"""
    return encode_speed_frame(0.0, 0.0, 0.0, seq=seq)



def fresh(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5):
    """与 ``test_arbiter.fresh`` 完全一致的助手（测试目录不是包，不能导入）。

    它调用 ``on_firmware_confirm(True)`` 模拟授权：只证明检测模型行为，
    不是固件确认。真实桥没有该调用点。
    """
    a = Arbiter(nav_timeout_s=nav_timeout_s, nav_cmd_max_age_s=nav_cmd_max_age_s,
                safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(a)
    a.on_publisher_count(1, 0)
    a.on_firmware_confirm(True)
    # 第五轮：启动默认"接收端未知"，测试需构造干净状态
    a.mark_receiver_clean(simulated=True)
    assert a.evaluate(0.0).is_zero   # 首帧零速，把桥推进到"已发过零速"
    return a


def _drive(a, clock, submit, vx=0.05):
    submit(a, vx, 0.0, 0.0, clock())
    d = a.evaluate(clock())
    assert not d.is_zero, '前置条件：停车前应能输出非零'
    return d


# --------------------------------------------------------------- 停车前收到
@pytest.mark.parametrize('source', ['nav', 'demo'])
def test_short_stop_release_does_not_replay_old_command(clock, source):
    """停车前收到的命令，解除后不得恢复（示例测试的参数化版本）。"""
    a = fresh()
    submit = (lambda arb, *args: arb.on_nav_cmd(*args)) if source == 'nav' \
        else (lambda arb, *args: arb.on_demo_cmd(*args))
    if source == 'demo':
        a.on_publisher_count(0, 1)
    _drive(a, clock, submit)

    a.on_stop(True)
    assert a.evaluate(clock()).zero_reason is ZeroReason.STOP_REQUESTED
    clock.advance(0.02)                      # 远小于 nav_timeout_s=0.3
    a.on_stop(False)
    assert a.evaluate(clock()).is_zero
    clock.advance(0.02)
    assert a.evaluate(clock()).is_zero
    clock.advance(0.02)
    assert a.evaluate(clock()).is_zero


@pytest.mark.parametrize('source', ['nav', 'demo'])
def test_stop_without_evaluate_between_still_discards(clock, source):
    """停车激活与解除之间**没有**调用 evaluate()，缓存仍须被丢弃。"""
    a = fresh()
    if source == 'demo':
        a.on_publisher_count(0, 1)
    submit = a.on_nav_cmd if source == 'nav' else a.on_demo_cmd
    submit(0.05, 0.0, 0.0, clock())
    assert not a.evaluate(clock()).is_zero

    a.on_stop(True)
    a.on_stop(False)                         # 期间一次 evaluate 都没调用
    assert a.evaluate(clock()).is_zero
    assert a.evaluate(clock()).is_zero


@pytest.mark.parametrize('source', ['nav', 'demo'])
def test_stop_state_round_trips_before_any_evaluate(clock, source):
    """停车状态反复切换，只要期间没收到新命令，解除后仍为零速。"""
    a = fresh()
    if source == 'demo':
        a.on_publisher_count(0, 1)
    submit = a.on_nav_cmd if source == 'nav' else a.on_demo_cmd
    submit(0.05, 0.0, 0.0, clock())
    assert not a.evaluate(clock()).is_zero

    for _ in range(5):
        a.on_stop(True)
        a.on_stop(False)
    for _ in range(3):
        clock.advance(0.01)
        assert a.evaluate(clock()).is_zero


# --------------------------------------------------------------- 停车期间收到
@pytest.mark.parametrize('source', ['nav', 'demo'])
def test_command_arriving_during_stop_is_not_a_candidate(clock, source):
    """停车期间到达的非零命令不得成为解除后的候选。"""
    a = fresh()
    if source == 'demo':
        a.on_publisher_count(0, 1)
    submit = a.on_nav_cmd if source == 'nav' else a.on_demo_cmd
    submit(0.05, 0.0, 0.0, clock())
    assert not a.evaluate(clock()).is_zero

    a.on_stop(True)
    assert a.evaluate(clock()).is_zero
    clock.advance(0.02)
    submit(0.08, 0.0, 0.0, clock())          # 停车期间仍有命令流
    assert a.evaluate(clock()).zero_reason is ZeroReason.STOP_REQUESTED
    clock.advance(0.02)
    submit(0.09, 0.0, 0.0, clock())
    assert a.evaluate(clock()).is_zero

    a.on_stop(False)                         # 解除
    d = a.evaluate(clock())
    assert d.is_zero, '停车期间收到的命令被当成了解除后的候选'
    clock.advance(0.02)
    assert a.evaluate(clock()).is_zero


def test_command_arriving_during_stop_with_mixed_sources(clock):
    """两类来源在停车期间都有输入，解除后都不得恢复。"""
    a = fresh()
    a.on_publisher_count(1, 1)
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    assert not a.evaluate(clock()).is_zero

    a.on_stop(True)
    a.evaluate(clock())
    a.on_nav_cmd(0.06, 0.0, 0.0, clock())
    a.on_demo_cmd(0.07, 0.0, 0.0, clock())
    a.evaluate(clock())

    a.on_stop(False)
    assert a.evaluate(clock()).is_zero
    assert a.evaluate(clock()).is_zero


# --------------------------------------------------------------- 正向对照
@pytest.mark.parametrize('source', ['nav', 'demo'])
def test_new_command_after_release_is_a_valid_candidate(clock, source):
    """解除停车后**新**收到、仍满足全部门控的命令可以输出。"""
    a = fresh()
    if source == 'demo':
        a.on_publisher_count(0, 1)
    submit = a.on_nav_cmd if source == 'nav' else a.on_demo_cmd
    submit(0.05, 0.0, 0.0, clock())
    assert not a.evaluate(clock()).is_zero

    a.on_stop(True)
    a.evaluate(clock())
    a.on_stop(False)
    assert a.evaluate(clock()).is_zero

    clock.advance(0.02)
    submit(0.04, 0.0, 0.0, clock())          # 解除之后才到达
    d = a.evaluate(clock())
    assert not d.is_zero, '解除后新命令应当可以输出'
    # 序号：授权零速(0) → 非零(1) → 停车零速(2,3) → 本条命令(4)。
    assert d.frame == encode_speed_frame(0.04, 0.0, 0.0, seq=4)


def test_new_command_after_release_still_needs_authorisation(clock):
    """去掉模拟授权后，解除停车后的新命令仍被 OUTPUT_BLOCKED 拦下。"""
    a = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5,
                safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(a)
    a.on_publisher_count(1, 0)
    a.mark_receiver_clean(simulated=True)   # 第五轮：接收端需先证明干净
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    a.on_stop(True)
    a.evaluate(clock())
    a.on_stop(False)
    clock.advance(0.02)
    a.on_nav_cmd(0.04, 0.0, 0.0, clock())
    assert a.evaluate(clock()).zero_reason in (ZeroReason.OUTPUT_BLOCKED,
                                              ZeroReason.NO_SESSION)


def test_new_command_after_release_still_obeys_expiry(clock):
    """解除停车后新命令自身过旧，仍按 NAV_STALE 拒绝。"""
    a = fresh()
    a.on_stop(True)
    a.evaluate(clock())
    a.on_stop(False)
    clock.advance(0.6)                       # 超过 nav_cmd_max_age_s=0.5
    a.on_nav_cmd(0.05, 0.0, 0.0, clock() - 0.6)
    # 过期来源跳过选择后统一报 NAV_EXPIRED（原 NAV_STALE 不再单独可达）
    assert a.evaluate(clock()).zero_reason is ZeroReason.NAV_EXPIRED


def test_new_command_after_release_still_obeys_limit(clock):
    """解除停车后新命令超占位限速，仍按 NAV_INVALID 拒绝。"""
    a = fresh()
    a.on_stop(True)
    a.evaluate(clock())
    a.on_stop(False)
    clock.advance(0.02)
    from c50c_uart3.chassis_cmd import PLACEHOLDER_MAX_LINEAR_MPS
    over = PLACEHOLDER_MAX_LINEAR_MPS + 0.05
    a.on_nav_cmd(over, 0.0, 0.0, clock())     # 超过占位上限
    assert a.evaluate(clock()).zero_reason is ZeroReason.NAV_INVALID


def test_new_command_after_release_still_obeys_ambiguity(clock):
    """解除停车后出现来源不明，仍按 AMBIGUOUS_SOURCES 拒绝。"""
    a = fresh()
    a.on_stop(True)
    a.evaluate(clock())
    a.on_stop(False)
    clock.advance(0.02)
    from c50c_uart3.chassis_cmd.arbiter import MAX_NAV_PUBLISHERS
    a.on_publisher_count(MAX_NAV_PUBLISHERS + 1, 0)
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    assert a.evaluate(clock()).zero_reason is ZeroReason.AMBIGUOUS_SOURCES


# --------------------------------------------------------------- 重复通知
def test_repeated_stop_set_does_not_clear_new_command(clock):
    """停车期间的重复 on_stop(True) 不得吞掉解除后**新**到达的命令。

    停车期间到达的命令按规则不入缓存，因此解除后的重启只能靠解除之后
    新到的命令；重复的停车通知不得影响这条路径。
    """
    a = fresh()
    a.on_stop(True)
    a.evaluate(clock())
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())    # 停车期间：按规则丢弃
    for _ in range(3):
        a.on_stop(True)                      # 重复通知（状态未变化）
        assert a.evaluate(clock()).zero_reason is ZeroReason.STOP_REQUESTED
    a.on_stop(False)
    assert a.evaluate(clock()).is_zero       # 解除后首次求值必为零速
    assert a.evaluate(clock()).is_zero       # 期间命令不得恢复
    clock.advance(0.02)
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())    # 解除之后才到达
    # 首帧零速规则由停车期间那几帧零速输出满足，故新命令本拍即可输出。
    d = a.evaluate(clock())
    assert not d.is_zero, '重复的停车通知不应吞掉解除后正常到达的新命令'
    # 序号：授权零速(0) → 停车零速 4 帧(1..4) → 解除后两帧(5,6) → 本条(7)。
    assert d.frame == encode_speed_frame(0.05, 0.0, 0.0, seq=7)


def test_repeated_stop_release_is_idempotent(clock):
    """停车已解除时的重复 on_stop(False) 不得恢复任何旧命令。"""
    a = fresh()
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    assert not a.evaluate(clock()).is_zero
    a.on_stop(True)
    a.on_stop(False)
    for _ in range(3):
        a.on_stop(False)
    assert a.evaluate(clock()).is_zero
    assert a.evaluate(clock()).is_zero


# --------------------------------------------------------------- 组合门控
def test_stop_discard_survives_write_failure_cycle(clock):
    """停车丢缓存与写失败丢缓存互不干扰：两条路径之后都只输出零速。"""
    a = fresh()
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    assert not a.evaluate(clock()).is_zero
    a.on_stop(True)
    a.on_stop(False)
    a.on_write_result(False)                 # 写失败锁存
    assert a.evaluate(clock()).zero_reason is ZeroReason.WRITE_FAILED
    a.clear_write_fault(by_operator=True)
    assert a.evaluate(clock()).is_zero       # 人工复位后仍不得恢复旧命令


def test_stop_discard_survives_reconnect(clock):
    """重连后再解除停车，只能零速。"""
    a = fresh()
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    assert not a.evaluate(clock()).is_zero
    a.on_stop(True)
    a.on_serial_reconnect()
    a.on_stop(False)
    # 重连已清空命令；解除停车后不得因空缓存而改变判定，仍禁止非零。
    d = a.evaluate(clock())
    assert d.is_zero and d.zero_reason in (ZeroReason.NO_NAV_COMMAND,
                                           ZeroReason.OUTPUT_BLOCKED)
