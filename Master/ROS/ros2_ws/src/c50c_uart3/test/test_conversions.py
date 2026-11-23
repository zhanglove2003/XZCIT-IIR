"""单位换算与 FlagStop 可用性语义定向测试（离线，不依赖 ROS）。"""
import math

import pytest

from c50c_uart3.conversions import (
    ACCEL_LSB_PER_M_S2,
    GYRO_LSB_PER_RAD_S,
    accel_lsb_to_m_s2,
    gyro_lsb_to_rad_s,
    imu_measurements,
)
from c50c_uart3.frame_parser import StatusFrameParser
from test_frame_parser import build_frame


def parse_frame(**kwargs):
    """构造一帧已知内容并解析为 StatusFrame，再进入换算层。"""
    (frame,) = StatusFrameParser().feed(build_frame(**kwargs))
    return frame


def test_conversion_constants_match_audit_report():
    # 审计报告 §3.3 / 固件 balance_task.c：加速度 1671.84 LSB/(m/s²)；
    # 陀螺 65.536 LSB/(°/s) × 180/π ≈ 3754.9 LSB/(rad/s)
    assert ACCEL_LSB_PER_M_S2 == 1671.84
    assert math.isclose(GYRO_LSB_PER_RAD_S, 3754.9, rel_tol=1e-4)


def test_accel_1g_mapping():
    # 16384 LSB ÷ 1671.84 ≈ 9.7997 m/s²（固件按 1 g ≈ 9.8 m/s² 约定）
    assert math.isclose(accel_lsb_to_m_s2(16384), 16384 / 1671.84, rel_tol=1e-9)
    assert math.isclose(accel_lsb_to_m_s2(-16384), -16384 / 1671.84, rel_tol=1e-9)
    assert accel_lsb_to_m_s2(0) == 0.0


def test_gyro_1rad_s_mapping():
    assert math.isclose(gyro_lsb_to_rad_s(GYRO_LSB_PER_RAD_S), 1.0, rel_tol=1e-9)
    # 1000 LSB ≈ 0.2663 rad/s ≈ 15.26 °/s
    assert math.isclose(gyro_lsb_to_rad_s(1000), 1000 / 3754.9, rel_tol=1e-4)


def test_flagstop_nonzero_marks_angular_velocity_unavailable():
    # FlagStop != 0：固件强制陀螺 Z=0，整组角速度按不可用处理并置零
    frame = parse_frame(stop=1, gx=100, gy=-200, gz=0,
                        ax=1000, ay=-1000, az=16384)
    m = imu_measurements(frame)
    assert m.gyro_rad_s == (0.0, 0.0, 0.0)
    assert m.angular_velocity_available is False
    # 加速度不受 FlagStop 影响（固件只钳零陀螺 Z）
    assert math.isclose(m.accel_m_s2[0], 1000 / ACCEL_LSB_PER_M_S2, rel_tol=1e-9)
    assert math.isclose(m.accel_m_s2[2], 16384 / ACCEL_LSB_PER_M_S2, rel_tol=1e-9)


def test_flagstop_zero_keeps_measured_angular_velocity():
    frame = parse_frame(stop=0, gx=655, gy=-655, gz=3754)
    m = imu_measurements(frame)
    assert m.angular_velocity_available is True
    assert math.isclose(m.gyro_rad_s[0], 655 / GYRO_LSB_PER_RAD_S, rel_tol=1e-9)
    assert math.isclose(m.gyro_rad_s[2], 3754 / GYRO_LSB_PER_RAD_S, rel_tol=1e-9)


def test_stop_flag_zero_is_not_treated_as_unavailable_when_gyro_z_zero():
    # 使能状态下陀螺 Z 恰好为 0 是合法实测值，不应被判为不可用
    frame = parse_frame(stop=0, gz=0)
    m = imu_measurements(frame)
    assert m.angular_velocity_available is True
    assert m.gyro_rad_s[2] == 0.0


def test_no_orientation_produced():
    # 固件不提供姿态四元数；换算层也不得伪造
    m = imu_measurements(parse_frame())
    assert not hasattr(m, 'orientation')


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__]))
