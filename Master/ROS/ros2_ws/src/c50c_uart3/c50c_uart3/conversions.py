#!/usr/bin/env python3
"""C50C 状态帧原始值 → ROS 单位换算。

换算系数来源（依据 c50c-imu-audit 审计报告 §3.3，均绑定固件初始化量程，
见 c50c-firmware-ref/BALANCE/MPU6050/MPU6050.c ``MPU6050_initialize()``；
量程若在现场被改动，本文件系数必须随之更新）：

- 加速度：±2 g 量程 → 16384 LSB/g；固件内部换算使用 ``imu.accel.z/1671.84f``
  （``balance_task.c``），审计报告 §3.3 记为 1671.84 LSB/(m/s²)，即按
  1 g ≈ 9.8 m/s² 取值。本模块沿用 1671.84 以与固件保持一致
  （若按标准重力 9.80665 m/s² 则为 1670.97，差约 0.05%，已记录）。
- 角速度：±500 dps 量程 → 65.536 LSB/(°/s)；1 °/s = π/180 rad/s，
  得 3754.93 LSB/(rad/s)（审计报告记为 3754.9）。

轴映射说明：固件组帧（``data_transition()``）已把芯片轴映射为
"ROS X 前、Y 左、Z 上" 约定（Accel.X=+芯片Y、Accel.Y=-芯片X 等）。
本模块**不做二次旋转**，只做单位换算。芯片在 C50C 板上的实际安装
朝向无归档证据，物理轴向待现场逐轴激励实测（尤其 D500 倒装后）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .frame_parser import StatusFrame

ACCEL_LSB_PER_G = 16384.0          # MPU-6050 ±2 g 量程（MPU6050.c:327）
ACCEL_LSB_PER_M_S2 = 1671.84       # 固件换算系数（balance_task.c /1671.84f），
                                   # 即 16384 LSB/g ÷ 9.8 m/s²（非标准重力 9.80665）

GYRO_LSB_PER_DPS = 65.536          # MPU-6050 ±500 dps 量程（MPU6050.c:325）
DEG_PER_RAD = 180.0 / math.pi
GYRO_LSB_PER_RAD_S = GYRO_LSB_PER_DPS * DEG_PER_RAD   # ≈ 3754.93


def accel_lsb_to_m_s2(raw_lsb: int) -> float:
    """加速度 LSB → m/s²（z 通道含固件补回的 +16384 即 +1 g 基准）。"""
    return raw_lsb / ACCEL_LSB_PER_M_S2


def gyro_lsb_to_rad_s(raw_lsb: int) -> float:
    """角速度 LSB → rad/s。"""
    return raw_lsb / GYRO_LSB_PER_RAD_S


@dataclass(frozen=True)
class ImuMeasurements:
    """按 sensor_msgs/Imu 约定整理后的 IMU 测量值。

    ``angular_velocity_available``: False 表示本帧角速度不可作为实测值使用
    （固件 FlagStop!=0 时陀螺 Z 被强制置零；sensor_msgs/Imu 无法表达单轴
    不可用，按消息约定整体标记）。发布方应将角速度置零并把
    ``angular_velocity_covariance[0]`` 置为 -1。
    """

    accel_m_s2: tuple[float, float, float]
    gyro_rad_s: tuple[float, float, float]
    angular_velocity_available: bool


def imu_measurements(frame: StatusFrame) -> ImuMeasurements:
    """从一帧状态数据得出可发布的 IMU 测量。

    - 加速度三轴始终为实测零偏扣除值（固件只在 FlagStop!=0 时钳零陀螺 Z，
      不动加速度），照常换算发布。
    - ``stop_flag != 0``：陀螺 Z 是固件写死的 0，不能区分"真实静止"与
      "数据被压零"，故整组角速度按不可用处理，测量值置零。
    - ``stop_flag == 0``：三轴角速度均为实测值，正常换算。
    """
    accel = tuple(accel_lsb_to_m_s2(v) for v in frame.accel_raw_lsb)
    if frame.stop_flag != 0:
        return ImuMeasurements(
            accel_m_s2=accel,
            gyro_rad_s=(0.0, 0.0, 0.0),
            angular_velocity_available=False,
        )
    gyro = tuple(gyro_lsb_to_rad_s(v) for v in frame.gyro_raw_lsb)
    return ImuMeasurements(
        accel_m_s2=accel,
        gyro_rad_s=gyro,
        angular_velocity_available=True,
    )
