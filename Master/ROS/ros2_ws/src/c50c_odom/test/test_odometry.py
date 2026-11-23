#!/usr/bin/env python3
"""平面里程计积分器定向测试(纯 Python,无 ROS 依赖)。

覆盖任务书要求:静止、直行、原地转向、异常时间间隔;另补圆弧(与细步长
数值积分交叉验证)、vy 侧移、角度规整、twist 单位换算。
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from c50c_odom.odometry import (
    BodyTwist,
    OdomState,
    PlanarOdomIntegrator,
    integrate_twist,
    twist_from_parsed,
    wrap_angle,
)

DT = 0.05  # 20 Hz 标称间隔,固件 data_task.c:53


def feed(ig, twists, dt=DT, t0=100.0):
    """按固定间隔喂入一串速度,返回最终状态与各步 reason 列表。"""
    reasons = []
    for i, tw in enumerate(twists):
        rep = ig.step(t0 + i * dt, tw)
        reasons.append(rep.reason)
    return ig.state, reasons


# ---------- 静止 ----------

def test_static_zero_velocity_keeps_pose():
    ig = PlanarOdomIntegrator()
    st, reasons = feed(ig, [BodyTwist()] * 40)
    assert reasons[0] == "first"
    assert all(r == "ok" for r in reasons[1:])
    assert st.x == 0.0 and st.y == 0.0 and st.yaw == 0.0
    assert st.twist == BodyTwist()
    assert ig.accepted_steps == 39 and ig.rejected_gap == 0


def test_first_sample_does_not_integrate():
    ig = PlanarOdomIntegrator()
    rep = ig.step(100.0, BodyTwist(vx=1.0))
    assert rep.reason == "first" and not rep.integrated
    assert rep.state.x == 0.0  # 即使速度非零,首样本也不产生位移


# ---------- 直行 ----------

def test_straight_line_exact_distance():
    ig = PlanarOdomIntegrator()
    st, _ = feed(ig, [BodyTwist(vx=0.5)] * 100)  # 首样本不积分:99 步 × 0.05 s × 0.5 m/s
    assert math.isclose(st.x, 99 * DT * 0.5, rel_tol=1e-9)
    assert abs(st.y) < 1e-9
    assert st.yaw == 0.0


def test_straight_line_at_initial_yaw():
    # 从 yaw=pi/2 出发直行:x 不动,y 前进(此时 +x 机体方向即世界 +y)
    st = integrate_twist(OdomState(yaw=math.pi / 2), BodyTwist(vx=1.0), 2.0)
    assert math.isclose(st.x, 0.0, abs_tol=1e-12)
    assert math.isclose(st.y, 2.0, rel_tol=1e-12)


# ---------- 原地转向 ----------

def test_in_place_rotation():
    ig = PlanarOdomIntegrator()
    # 9 个样本 = 首样本 + 8 步;每步 0.5 s × (pi/4) = pi/8,共 pi
    st, reasons = feed(ig, [BodyTwist(wz=math.pi / 4)] * 9, dt=0.5)
    assert reasons[0] == "first" and reasons[1:] == ["ok"] * 8
    assert abs(st.x) < 1e-12 and abs(st.y) < 1e-12
    assert math.isclose(st.yaw, math.pi, abs_tol=1e-9)
    # 再转 pi(同样首样本只重置时间基准)→ 2π 规整回 0
    st2, _ = feed(ig, [BodyTwist(wz=math.pi / 4)] * 9, dt=0.5, t0=200.0)
    assert math.isclose(st2.yaw, 0.0, abs_tol=1e-9)


def test_negative_rotation_wraps():
    st = OdomState(yaw=math.pi)
    st2 = integrate_twist(st, BodyTwist(wz=-0.01), 1.0)  # pi - 0.01
    assert st2.yaw == math.pi - 0.01
    st3 = integrate_twist(OdomState(yaw=-math.pi + 0.005), BodyTwist(wz=-0.01), 1.0)
    # 越过 -pi 边界 → 规整到接近 +pi
    assert st3.yaw > 3.0


# ---------- 圆弧:与数值积分交叉验证 ----------

def _numeric_reference(twist, total_t, steps=200000):
    """中点法数值积分参考(二阶,400k 步下位置误差 ~1e-10)。"""
    x = y = yaw = 0.0
    h = total_t / steps
    for _ in range(steps):
        xm = (twist.vx * math.cos(yaw) - twist.vy * math.sin(yaw))
        ym = (twist.vx * math.sin(yaw) + twist.vy * math.cos(yaw))
        yaw_m = yaw + twist.wz * h / 2.0
        x += (twist.vx * math.cos(yaw_m) - twist.vy * math.sin(yaw_m)) * h
        y += (twist.vx * math.sin(yaw_m) + twist.vy * math.cos(yaw_m)) * h
        yaw += twist.wz * h
    return x, y, yaw


def test_arc_matches_numeric_integration():
    # 采样栅格精确落在整圈上:每步转角 2π/16,dt=0.5 → wz=π/4,半径 vx/wz
    twist = BodyTwist(vx=1.0, wz=math.pi / 4)
    ig = PlanarOdomIntegrator()
    st, _ = feed(ig, [twist] * 17, dt=0.5)  # 首样本 + 16 步 = 恰好一整圈
    # 整圈回原点,朝向复原
    assert abs(st.x) < 1e-9 and abs(st.y) < 1e-9
    assert math.isclose(st.yaw, 0.0, abs_tol=1e-9)
    # 与细步长数值积分交叉验证(参考覆盖同样的 16×0.5 s)
    ref_x, ref_y, ref_yaw = _numeric_reference(twist, 16 * 0.5, steps=400000)
    assert math.isclose(st.x, ref_x, abs_tol=1e-6)
    assert math.isclose(st.y, ref_y, abs_tol=1e-6)
    assert math.isclose(st.yaw, wrap_angle(ref_yaw), abs_tol=1e-9)


def test_arc_with_lateral_velocity_matches_numeric():
    twist = BodyTwist(vx=0.8, vy=0.2, wz=0.3)  # 侧移项当前固件为 0,但公式须正确
    ig = PlanarOdomIntegrator()
    n = 120
    st, _ = feed(ig, [twist] * n)
    ref_x, ref_y, ref_yaw = _numeric_reference(twist, (n - 1) * DT, steps=400000)
    assert math.isclose(st.x, ref_x, abs_tol=1e-6)
    assert math.isclose(st.y, ref_y, abs_tol=1e-6)
    assert math.isclose(st.yaw, wrap_angle(ref_yaw), abs_tol=1e-9)


# ---------- 异常时间间隔 ----------

def test_zero_dt_rejected():
    ig = PlanarOdomIntegrator()
    ig.step(100.0, BodyTwist(vx=1.0))
    rep = ig.step(100.0, BodyTwist(vx=1.0))  # dt=0
    assert rep.reason == "nonpositive_dt" and not rep.integrated
    assert rep.state.x == 0.0


def test_negative_dt_rejected_and_last_t_not_advanced():
    ig = PlanarOdomIntegrator()
    ig.step(100.0, BodyTwist(vx=1.0))
    rep = ig.step(90.0, BodyTwist(vx=1.0))  # 时间倒退 10 s
    assert rep.reason == "nonpositive_dt"
    assert rep.state.x == 0.0
    # last_t 未前移:下一个正常样本按 100.0 起算,只积 0.05 s
    rep2 = ig.step(100.05, BodyTwist(vx=1.0))
    assert rep2.reason == "ok"
    assert math.isclose(rep2.state.x, 0.05, rel_tol=1e-9)


def test_gap_not_integrated_across():
    ig = PlanarOdomIntegrator()
    ig.step(100.0, BodyTwist(vx=1.0))
    ig.step(100.05, BodyTwist(vx=1.0))
    rep = ig.step(102.0, BodyTwist(vx=1.0))  # 1.95 s 断流 > 0.5 s 阈值
    assert rep.reason == "gap" and not rep.integrated
    assert math.isclose(rep.state.x, 0.05, abs_tol=1e-12)  # 断流前的位移保留
    # 间隙后重新起算,不把 1.95 s 一次积分进去
    rep2 = ig.step(102.05, BodyTwist(vx=1.0))
    assert rep2.reason == "ok"
    assert math.isclose(rep2.state.x, 0.10, rel_tol=1e-9)
    assert ig.rejected_gap == 1


def test_max_interval_boundary_integrates():
    ig = PlanarOdomIntegrator(max_interval_s=0.5)
    ig.step(100.0, BodyTwist(vx=1.0))
    rep = ig.step(100.5, BodyTwist(vx=1.0))  # dt == 阈值,不算 gap
    assert rep.reason == "ok"
    assert math.isclose(rep.state.x, 0.5, rel_tol=1e-9)


# ---------- 单位换算 ----------

def test_twist_from_parsed_units():
    tw = twist_from_parsed(300, 0, -1250)
    assert math.isclose(tw.vx, 0.3, abs_tol=1e-12)
    assert tw.vy == 0.0
    assert math.isclose(tw.wz, -1.25, abs_tol=1e-12)
    # int16 满量程往返无损
    tw2 = twist_from_parsed(-32768, 0, 32767)
    assert math.isclose(tw2.vx, -32.768, abs_tol=1e-12)
    assert math.isclose(tw2.wz, 32.767, abs_tol=1e-12)


def test_wrap_angle_range():
    assert math.isclose(wrap_angle(math.pi), math.pi, abs_tol=1e-12)
    assert math.isclose(wrap_angle(-math.pi), math.pi, abs_tol=1e-12)  # -pi 归到 +pi
    assert math.isclose(wrap_angle(3.0 * math.pi), math.pi, abs_tol=1e-12)
    assert math.isclose(wrap_angle(0.25), 0.25, abs_tol=1e-12)


# ---------- 断流/重连显式重置 ----------

def test_reset_time_base_pending_first_no_displacement():
    """重连重置后：首样本只建立新基准(不积分、不位移)，第二样本才积分。

    重连→首帧之间没有速度样本，不能按首帧速度补算这段时间的位移。"""
    ig = PlanarOdomIntegrator()
    ig.step(100.0, BodyTwist(vx=1.0))              # first：只建基准
    ig.step(100.1, BodyTwist(vx=1.0))              # 积分 0.1 s → x=0.1
    assert math.isclose(ig.state.x, 0.1, rel_tol=1e-12)

    ig.reset_time_base()
    assert ig.explicit_resets == 1

    # 重连后首帧（即使远在将来）：只建基准，位姿不变
    rep = ig.step(200.0, BodyTwist(vx=1.0))
    assert rep.reason == "first" and not rep.integrated
    assert math.isclose(ig.state.x, 0.1, rel_tol=1e-12)
    # 第二帧起按常规间隔策略积分
    rep = ig.step(200.1, BodyTwist(vx=1.0))
    assert rep.integrated
    assert math.isclose(ig.state.x, 0.2, rel_tol=1e-12)
    assert ig.explicit_resets == 1  # 重置只在显式调用时计数


def test_reset_before_any_sample_is_harmless():
    ig = PlanarOdomIntegrator()
    ig.reset_time_base()
    rep = ig.step(10.0, BodyTwist(vx=2.0))
    assert rep.reason == "first"
    assert ig.explicit_resets == 1
    rep = ig.step(10.5, BodyTwist(vx=2.0))
    assert rep.integrated and math.isclose(rep.state.x, 1.0, rel_tol=1e-12)
