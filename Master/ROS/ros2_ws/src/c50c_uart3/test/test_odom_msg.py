#!/usr/bin/env python3
"""OdomCandidate:编码器里程计候选 → nav_msgs/msg/Odometry 定向测试。

- ``update_from_frame`` / 诊断计数:纯 Python,本机即可运行;
- ``build_msg`` 消息构造:需要 nav_msgs,无 ROS 环境时跳过
  (本机无 ROS;完整消息验证在香橙派 Humble 隔离域执行)。

关键断言:角速度来源是**编码器解算** chassis_yaw_rate,不是 IMU 陀螺 Z。
"""
import math
import sys
import time as _time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from c50c_uart3.frame_parser import StatusFrame
from c50c_uart3.odom_msg import OdomCandidate, OdomCandidateConfig

try:
    from nav_msgs.msg import Odometry
    HAVE_NAV_MSGS = True
except ImportError:
    HAVE_NAV_MSGS = False


def _frame(vx=0, vy=0, wz=0, gyro_z=999, stop=0):
    """构造 StatusFrame:gyro_z 故意给一个显眼值,断言它不出现在 twist 里。"""
    return StatusFrame(
        stop_flag=stop,
        chassis_velocity_mm_s=(vx, vy),
        chassis_yaw_rate_mrad_s=wz,
        accel_raw_lsb=(0, 0, 16384),
        gyro_raw_lsb=(0, 0, gyro_z),
        voltage_mv=4060,
        encoder_counts=(0, 0, 0, 0),
        pwm_counts=(0, 0, 0, 0),
        ultrasonic_front_mm=0xFFFF,
        frame_bytes=b"\x7b" + b"\x00" * 22 + b"\x00" * 16 + b"\x00" * 10
                    + b"\x7d",
    )


# ---------- 积分接入(无 ROS 依赖) ----------

def test_update_uses_encoder_wz_not_gyro():
    cand = OdomCandidate()
    t = 100.0
    cand.update_from_frame(_frame(), t)  # first: 只建时间基准
    rep = cand.update_from_frame(_frame(vx=500, wz=-250, gyro_z=999), t + 0.05)
    assert rep.reason == "ok" and rep.integrated
    st = cand.state
    # wz 用编码器值 −0.25 rad/s(不是陀螺 999 LSB);vx=0.5 m/s 经 0.05 s
    # SE(2) 精确积分:yaw 0→−0.0125,x = vx/wz·sin(Δyaw) = 0.024999…(非直线 0.025)
    th1 = -0.25 * 0.05
    x_exact = (0.5 / -0.25) * math.sin(th1)
    assert math.isclose(st.x, x_exact, rel_tol=1e-12)
    assert math.isclose(st.yaw, th1, rel_tol=1e-12)
    assert math.isclose(st.twist.wz, -0.25, rel_tol=1e-12)
    assert st.twist.vx == 0.5 and st.twist.vy == 0.0


def test_time_base_is_host_receive_time():
    """dt 由调用方提供的接收时刻决定;dt>0.5s 记为断流间隙。"""
    cand = OdomCandidate()
    cand.update_from_frame(_frame(), 10.0)          # first,不积分
    cand.update_from_frame(_frame(vx=1000), 10.05)  # dt=0.05,接受
    cand.update_from_frame(_frame(vx=1000), 10.2)   # dt=0.15,接受
    cand.update_from_frame(_frame(vx=1000), 11.0)   # dt=0.8 > 0.5 → gap,不积分
    d = cand.integrator_diagnostics()
    assert d["rejected_gap"] == 1 and d["accepted_steps"] == 2
    # 位姿只含两段接受步:0.05+0.15=0.2 m,gap 的 0.8 s 未跨积分
    assert math.isclose(cand.state.x, 0.2, rel_tol=1e-9)


def test_nonpositive_dt_rejected():
    cand = OdomCandidate()
    cand.update_from_frame(_frame(), 10.0)
    rep = cand.update_from_frame(_frame(vx=500), 10.0)  # 重复时刻
    assert rep.reason == "nonpositive_dt" and not rep.integrated
    rep = cand.update_from_frame(_frame(vx=500), 9.9)   # 时间倒退
    assert rep.reason == "nonpositive_dt" and not rep.integrated
    assert cand.state.x == 0.0


# ---------- 消息构造(需 nav_msgs) ----------

def test_build_msg_fields_and_covariance():
    if not HAVE_NAV_MSGS:
        import pytest
        pytest.skip("无 nav_msgs(本机无 ROS);消息字段验证在香橙派 Humble 执行")
    cand = OdomCandidate(OdomCandidateConfig(
        pose_covariance_yaw=0.05, twist_covariance_angular=0.02))
    t = _time.monotonic()
    cand.update_from_frame(_frame(), t)
    cand.update_from_frame(_frame(vx=250, wz=100), t + 0.05)
    from builtin_interfaces.msg import Time as TimeMsg
    stamp = TimeMsg(sec=1794044000, nanosec=500000000)
    msg = cand.build_msg(stamp)
    st = cand.state
    assert msg.header.stamp == stamp
    assert msg.header.frame_id == "odom"
    assert msg.child_frame_id == "base_link"
    assert math.isclose(msg.pose.pose.position.x, st.x, rel_tol=1e-12)
    q = msg.pose.pose.orientation
    assert math.isclose(q.z, math.sin(st.yaw / 2), rel_tol=1e-12)
    assert math.isclose(q.w, math.cos(st.yaw / 2), rel_tol=1e-12)
    assert math.isclose(q.x, 0.0, abs_tol=1e-15)
    assert math.isclose(q.y, 0.0, abs_tol=1e-15)
    tw = msg.twist.twist
    assert tw.linear.x == st.twist.vx and tw.linear.y == 0.0
    assert tw.angular.z == st.twist.wz
    assert tw.angular.x == 0.0 and tw.angular.y == 0.0
    # 6×6 行主序协方差:占位对角项 + 未测量维度大方差
    assert msg.pose.covariance[0] == 0.01      # x(占位)
    assert msg.pose.covariance[7] == 0.01      # y(占位)
    assert msg.pose.covariance[14] == 1e6      # z(未测量)
    assert msg.pose.covariance[21] == 1e6      # roll(未测量)
    assert msg.pose.covariance[28] == 1e6      # pitch(未测量)
    assert msg.pose.covariance[35] == 0.05     # yaw(占位,行 5,列 5)
    assert msg.twist.covariance[0] == 0.01     # vx(占位)
    assert msg.twist.covariance[7] == 1e6      # vy(固件结构零,未测量)
    assert msg.twist.covariance[14] == 1e6     # vz(未测量)
    assert msg.twist.covariance[21] == 1e6     # vroll(未测量)
    assert msg.twist.covariance[28] == 1e6     # vpitch(未测量)
    assert msg.twist.covariance[35] == 0.02    # vyaw(占位)
    # 非对角线保持 0(各轴独立假设)
    offdiag = [i for i in range(36) if i % 7 != 0]
    assert all(msg.pose.covariance[i] == 0.0 for i in offdiag)
    assert all(msg.twist.covariance[i] == 0.0 for i in offdiag)


def test_unmeasured_variance_configurable():
    """未测量维度方差可配置,占位项与未测量项独立设置。"""
    if not HAVE_NAV_MSGS:
        import pytest
        pytest.skip("无 nav_msgs(本机无 ROS);消息字段验证在香橙派 Humble 执行")
    cand = OdomCandidate(OdomCandidateConfig(
        unmeasured_variance=555.0, pose_covariance_x=0.2))
    t = _time.monotonic()
    cand.update_from_frame(_frame(), t)
    cand.update_from_frame(_frame(vx=100), t + 0.05)
    from builtin_interfaces.msg import Time as TimeMsg
    msg = cand.build_msg(TimeMsg(sec=0, nanosec=0))
    assert msg.pose.covariance[0] == 0.2    # 占位 x:自定义
    assert msg.pose.covariance[14] == 555.0  # z:未测量方差自定义
    assert msg.twist.covariance[7] == 555.0  # vy:未测量方差自定义


def test_config_rejects_invalid_variance():
    """方差非法值(0/负数/-1/nan/inf)直接拒绝,不生成误导性消息。"""
    import pytest
    for bad in (0.0, -1.0, -1e-9, float("nan"), float("inf"), float("-inf")):
        with pytest.raises(ValueError):
            OdomCandidateConfig(unmeasured_variance=bad)
        with pytest.raises(ValueError):
            OdomCandidateConfig(pose_covariance_x=bad)
    # 合法值不受影响
    OdomCandidateConfig(unmeasured_variance=1e6, pose_covariance_x=0.01)
