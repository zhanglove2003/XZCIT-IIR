#!/usr/bin/env python3
"""陀螺 Z 轴静止残差估计与修正（纯 Python，不依赖 ROS）。

背景（见 evidence/.../yaw-drift-readonly-20261119/固件调用链复核-20261119.md）：
参考固件在上电后前 10 秒确定陀螺偏置并在上报前扣除，之后固定不变。
2026-11-19 实测静止时 ROS 侧 z 仍有约 +0.00255 rad/s（0.146 °/s）残差，
EKF 只融合 IMU z 角速度，把它积分成约 8.7°/分钟的航向漂移。

本模块只估计"固件扣除之后的残差"，所以不会与固件的启动标定重复扣除；
固件重启或双击重标定使残差突变时，估计器靠滑动窗口重新收敛。

静止证据缺一不可，宁可不学习，不可把运动中的 gz 当偏置：
  1. 同帧内编码器四轮、底盘速度、底盘偏航率全为 0，且 stop_flag == 0；
  2. 角速度可用（固件 FlagStop != 0 时 z 被钳零，整组标记不可用）；
  3. 窗口内 gz 标准差小、加速度模长标准差小（被推动/晃动则拒绝）；
  4. 窗口均值不超过 max_bias（上电 10 秒窗口内固件未扣除的大偏移不会被学走）。

已声明的限制：低于 max_bias 且恒定的真实慢转，与偏置无法区分，只能靠
max_bias 限幅。固件把编码器 m/s 直接取整时（低于 1 m/s 恒为 0），
"编码器为 0"不等于静止，所以必须叠加 3、4。
"""
from __future__ import annotations

import math
from collections import deque


def frame_is_still(frame) -> bool:
    """同一状态帧内的运动字段全为 0，且软件使能（stop_flag == 0）。"""
    return (frame.stop_flag == 0
            and all(v == 0 for v in frame.encoder_counts)
            and all(v == 0 for v in frame.chassis_velocity_mm_s)
            and frame.chassis_yaw_rate_mrad_s == 0)


def _mean_std(values):
    n = len(values)
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n
    return mean, math.sqrt(var)


class GyroZBiasEstimator:
    """静止窗口内的 z 轴残差估计。bias 在收敛前恒为 0（原样透传）。"""

    def __init__(self, window_s: float = 30.0, startup_hold_s: float = 15.0,
                 max_bias: float = 0.01, gyro_std_max: float = 0.003,
                 accel_std_max: float = 0.1, max_gap_s: float = 0.5,
                 max_slew_rad_s2: float = 1e-4, min_samples: int = 100):
        self.window_s = float(window_s)
        self.startup_hold_s = float(startup_hold_s)
        self.max_bias = float(max_bias)
        self.gyro_std_max = float(gyro_std_max)
        self.accel_std_max = float(accel_std_max)
        self.max_gap_s = float(max_gap_s)
        self.max_slew_rad_s2 = float(max_slew_rad_s2)
        self.min_samples = int(min_samples)
        self._bias = 0.0
        self._converged = False
        self._t_first = None       # 第一个样本时刻，用于启动保持
        self._last_t = None
        self._run_start = None     # 当前连续静止段的起点
        self._last_update_t = None
        self._samples = deque()    # (t, gz, accel_norm)

    @property
    def bias(self) -> float:
        return self._bias

    @property
    def converged(self) -> bool:
        return self._converged

    def correct(self, gz: float) -> float:
        """未收敛时原样返回，不置零。"""
        return gz - self._bias if self._converged else gz

    def reset_window(self) -> None:
        self._samples.clear()
        self._run_start = None

    def update(self, t: float, gz: float, accel_norm: float,
               still: bool, available: bool) -> None:
        if not (math.isfinite(t) and math.isfinite(gz)
                and math.isfinite(accel_norm)):
            self.reset_window()
            return
        if self._last_t is not None and (
                t <= self._last_t or t - self._last_t > self.max_gap_s):
            self.reset_window()        # 时间倒退或断流：窗口作废
        self._last_t = t
        if self._t_first is None:
            self._t_first = t
        if not (still and available):
            self.reset_window()
            return
        if t - self._t_first < self.startup_hold_s:
            return
        if self._run_start is None:
            self._run_start = t
        self._samples.append((t, gz, accel_norm))
        while self._samples and t - self._samples[0][0] > self.window_s:
            self._samples.popleft()
        if t - self._run_start < self.window_s:
            return
        if len(self._samples) < self.min_samples:
            return
        gzs = [s[1] for s in self._samples]
        acc = [s[2] for s in self._samples]
        mean, std = _mean_std(gzs)
        _, accel_std = _mean_std(acc)
        if (abs(mean) > self.max_bias or std > self.gyro_std_max
                or accel_std > self.accel_std_max):
            return
        if not self._converged:
            self._bias = max(-self.max_bias, min(self.max_bias, mean))
            self._converged = True
        else:
            dt = t - self._last_update_t if self._last_update_t else 0.0
            limit = self.max_slew_rad_s2 * dt
            step = max(-limit, min(limit, mean - self._bias))
            self._bias = max(-self.max_bias,
                             min(self.max_bias, self._bias + step))
        self._last_update_t = t


class GyroBiasCorrector:
    """把估计器接到状态帧上：只修正 z 轴，x、y 不动。"""

    def __init__(self, estimator: GyroZBiasEstimator):
        self.estimator = estimator

    def process(self, frame, measurements, t: float) -> float:
        """返回应发布的 angular_velocity.z（rad/s）。"""
        gz = measurements.gyro_rad_s[2]
        accel = measurements.accel_m_s2
        norm = math.sqrt(sum(a * a for a in accel))
        available = measurements.angular_velocity_available
        self.estimator.update(t, gz, norm, frame_is_still(frame), available)
        if not available:
            return gz              # 固件钳零的 0 不得变成 -bias
        return self.estimator.correct(gz)
