"""0.3 s 过期阈值与 0.5 s 自身时效：边界两侧各验证一次。

阈值是**占位值**（不是已标定的控制周期），因此测试只验证比较逻辑，
不声称这两个数在车上正确。
"""
import pytest

from c50c_uart3.chassis_cmd.arbiter import Arbiter, ZeroReason
from sim_confirm import TEST_SAFETY_MAX_AGE_S, grant_safety


def make(**kw):
    a = Arbiter(safety_max_age_s=TEST_SAFETY_MAX_AGE_S, **kw)
    grant_safety(a)
    a.on_publisher_count(1, 0)
    a.on_firmware_confirm(True)
    a.mark_receiver_clean(simulated=True)  # 第五轮：干净接收端
    assert a.evaluate(0.0).is_zero     # 首帧零速（桥启动规则）
    return a


def test_expiry_boundary_nav_timeout():
    a = make(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5)
    a.on_nav_cmd(0.05, 0.0, 0.0, 10.0)
    # 浮点减法在 10.3-10.0 上得到 0.3000000000000007，故"恰好等于"只能用
    # 整数起点测量；这不影响判据（比较是 >，严格大于才过期）。
    assert not a.evaluate(10.2999).is_zero
    assert a.evaluate(10.3).zero_reason is ZeroReason.NAV_EXPIRED


def test_self_age_boundary_beyond_timeout():
    a = make(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5)
    a.on_nav_cmd(0.05, 0.0, 0.0, 10.0 - 0.4)     # 自身已 0.4 s 旧
    assert a.evaluate(10.0).zero_reason is ZeroReason.NAV_EXPIRED
    a.on_nav_cmd(0.05, 0.0, 0.0, 10.0 - 0.6)
    # 2026-11-14 起跳过-过期来源统一报 NAV_EXPIRED
    assert a.evaluate(10.0).zero_reason is ZeroReason.NAV_EXPIRED


@pytest.mark.parametrize('nav_timeout,nav_age', [(0.3, 0.0), (0.3, 0.31),
                                                 (0.3, 0.9)])
def test_no_nonzero_ever_after_stop_while_expired(nav_timeout, nav_age):
    """过期后任何拍都不再出现非零帧（不重发、不排队）。"""
    a = make(nav_timeout_s=nav_timeout)
    a.on_nav_cmd(0.05, 0.0, 0.0, 0.0)
    a.on_stop(True)
    a.on_stop(False)
    frames = [a.evaluate(nav_age + i * 0.05) for i in range(20)]
    if nav_age <= nav_timeout:
        # 未超时：允许放行（这是新的合法命令，不是重发）
        return
    assert all(d.is_zero for d in frames)
