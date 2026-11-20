"""陀螺 Z 轴静止残差估计器定向测试（离线，不依赖 ROS，不接板）。

背景：固件上电后前 10 秒确定陀螺偏置并扣除，ROS 侧静止时仍剩约
+0.0025 rad/s（约 0.146 °/s）残差，被 EKF 积分成航向漂移。估计器只学习
"固件扣除之后的残差"，所以不会与固件标定重复扣除。
"""
import math
import random

import pytest

from c50c_uart3.conversions import imu_measurements
from c50c_uart3.frame_parser import StatusFrameParser
from c50c_uart3.gyro_bias import (
    GyroBiasCorrector,
    GyroZBiasEstimator,
    frame_is_still,
)
from test_frame_parser import build_frame

DT = 0.05          # 20 Hz 状态帧
G = 9.8            # 静止加速度模长（近似）
RESIDUAL = 0.00255  # 实测 60 秒 z 均值，rad/s
NOISE = 0.0005     # 实测静止 z 噪声量级（范围 0.001~0.004）


def run(est, seconds, t0, gz_mean=RESIDUAL, noise=NOISE, still=True,
        available=True, rng=None, accel=G):
    """以 20 Hz 喂入 `seconds` 秒样本，返回结束时刻。"""
    rng = rng or random.Random(1)
    n = int(round(seconds / DT))
    t = t0
    for _ in range(n):
        t += DT
        est.update(t, rng.gauss(gz_mean, noise), accel + rng.gauss(0, 0.01),
                   still, available)
    return t


def test_1_stable_residual_converges_and_drift_is_small():
    est = GyroZBiasEstimator()
    t = run(est, 60.0, 0.0)
    assert est.converged
    assert est.bias == pytest.approx(RESIDUAL, abs=0.0003)
    # 修正后再静止 60 秒：积分漂移应远小于当前 8.7°
    rng = random.Random(7)
    integral = 0.0
    for _ in range(int(60 / DT)):
        t += DT
        gz = rng.gauss(RESIDUAL, NOISE)
        est.update(t, gz, G, True, True)
        integral += est.correct(gz) * DT
    assert abs(math.degrees(integral)) < 0.5


@pytest.mark.parametrize('rate', [0.05, -0.05, 0.3, -0.3])
def test_2_turning_freezes_estimate_and_keeps_sign(rate):
    est = GyroZBiasEstimator()
    t = run(est, 60.0, 0.0)
    bias = est.bias
    t = run(est, 20.0, t, gz_mean=rate + RESIDUAL, still=False)
    assert est.bias == bias                      # 转向期间冻结
    out = est.correct(rate + RESIDUAL)
    assert math.copysign(1.0, out) == math.copysign(1.0, rate)
    assert out == pytest.approx(rate, abs=0.0003)   # 无死区，小角速度不被压成 0


def test_3_startup_passthrough_and_raw_offset_not_learned():
    est = GyroZBiasEstimator()
    # 保持时间内：不学习，原样透传
    t = run(est, 10.0, 0.0)
    assert not est.converged
    assert est.correct(0.123) == 0.123
    # 固件 10 秒窗口内 z 还没扣除偏置：大偏移不得被学成偏置
    est2 = GyroZBiasEstimator()
    run(est2, 120.0, 0.0, gz_mean=0.2)
    assert not est2.converged
    assert est2.bias == 0.0


def test_4_step_change_reconverges_without_double_subtraction():
    est = GyroZBiasEstimator()
    t = run(est, 60.0, 0.0)
    assert est.bias == pytest.approx(RESIDUAL, abs=0.0003)
    # 固件重启或双击重标定后残差跳到 -0.001
    t = run(est, 200.0, t, gz_mean=-0.001)
    assert est.bias == pytest.approx(-0.001, abs=0.0004)
    assert abs(est.bias - (-0.0035)) > 0.002     # 不是二次扣除后的 -0.0035


def test_5a_rotation_with_still_encoders_is_rejected():
    # 编码器报 0（still=True）但陀螺显示真实旋转 0.3 rad/s
    est = GyroZBiasEstimator()
    run(est, 120.0, 0.0, gz_mean=0.3, noise=0.01, still=True)
    assert not est.converged
    assert est.bias == 0.0


def test_5b_slow_rotation_is_bounded_by_max_bias():
    # 已知限制：低于 max_bias 的真实慢转无法与偏置区分，只能限幅。
    est = GyroZBiasEstimator(max_bias=0.01)
    run(est, 120.0, 0.0, gz_mean=0.008)
    assert abs(est.bias) <= 0.01
    est2 = GyroZBiasEstimator(max_bias=0.01)
    run(est2, 120.0, 0.0, gz_mean=0.02)           # 超过上限：不学
    assert not est2.converged


def test_5c_accel_disturbance_blocks_learning():
    est = GyroZBiasEstimator()
    rng = random.Random(3)
    t = 0.0
    for _ in range(int(120 / DT)):
        t += DT
        est.update(t, rng.gauss(RESIDUAL, NOISE),
                   G + rng.gauss(0, 0.8), True, True)   # 被推动/晃动
    assert not est.converged


def test_6_motion_stops_updates_and_resets_window():
    est = GyroZBiasEstimator()
    t = run(est, 60.0, 0.0)
    bias = est.bias
    t = run(est, 10.0, t, gz_mean=0.0, still=False)
    assert est.bias == bias
    # 恢复静止后，窗口攒满之前不得更新
    t = run(est, 20.0, t, gz_mean=-0.004, still=True)
    assert est.bias == bias
    t = run(est, 40.0, t, gz_mean=-0.004, still=True)
    assert est.bias != bias                       # 攒满后才更新


def test_7_imu_unavailable_is_not_learned():
    est = GyroZBiasEstimator()
    run(est, 120.0, 0.0, gz_mean=0.0, available=False)
    assert not est.converged
    assert est.bias == 0.0


def test_9_timestamp_anomaly_resets_window():
    est = GyroZBiasEstimator()
    t = run(est, 14.0, 0.0)
    t += 1.0                                      # 间隔 > 0.5 s
    t = run(est, 20.0, t)
    assert not est.converged                      # 窗口被清空，需重新攒
    t = run(est, 30.0, t)
    assert est.converged
    est2 = GyroZBiasEstimator()
    est2.update(10.0, RESIDUAL, G, True, True)
    est2.update(9.0, RESIDUAL, G, True, True)     # 时间倒退不得抛异常
    assert not est2.converged


def test_bias_clamped_to_max_bias():
    est = GyroZBiasEstimator(max_bias=0.004)
    run(est, 120.0, 0.0, gz_mean=0.0035)
    assert abs(est.bias) <= 0.004


def parse(**kwargs):
    (frame,) = StatusFrameParser().feed(build_frame(**kwargs))
    return frame


def test_frame_is_still_requires_all_zero_motion_fields():
    assert frame_is_still(parse(az=16384))
    assert not frame_is_still(parse(az=16384, encoder=(0, 0, 3, 0)))
    assert not frame_is_still(parse(az=16384, vx=10))
    assert not frame_is_still(parse(az=16384, vy=-4))
    assert not frame_is_still(parse(az=16384, wz=50))
    assert not frame_is_still(parse(az=16384, stop=1))


def feed_frames(corrector, seconds, t0, **frame_kwargs):
    t = t0
    out = None
    for _ in range(int(round(seconds / DT))):
        t += DT
        frame = parse(**frame_kwargs)
        out = corrector.process(frame, imu_measurements(frame), t)
    return t, out


def test_8_flagstop_forced_zero_is_passthrough_and_not_learned():
    corrector = GyroBiasCorrector(GyroZBiasEstimator())
    # 先在正常静止下收敛（gz 约 9.6 LSB，加噪）
    rng = random.Random(5)
    t = 0.0
    for _ in range(int(60 / DT)):
        t += DT
        frame = parse(az=16384, gz=round(rng.gauss(9.6, 2.0)))
        corrector.process(frame, imu_measurements(frame), t)
    assert corrector.estimator.converged
    bias = corrector.estimator.bias
    # FlagStop != 0：固件把 z 置 0，整组标记不可用
    t, out = feed_frames(corrector, 30.0, t, az=16384, stop=1)
    assert out == 0.0                              # 不得变成 -bias
    assert corrector.estimator.bias == bias


def test_corrector_subtracts_bias_only_on_z():
    est = GyroZBiasEstimator()
    corrector = GyroBiasCorrector(est)
    rng = random.Random(9)
    t = 0.0
    for _ in range(int(60 / DT)):
        t += DT
        frame = parse(az=16384, gx=-13, gy=-11, gz=round(rng.gauss(9.6, 2.0)))
        corrector.process(frame, imu_measurements(frame), t)
    assert est.converged
    frame = parse(az=16384, gx=-13, gy=-11, gz=10)
    m = imu_measurements(frame)
    z = corrector.process(frame, m, t + DT)
    assert z == pytest.approx(m.gyro_rad_s[2] - est.bias)
    assert abs(z) < 0.001                          # 静止残差基本被抵消
