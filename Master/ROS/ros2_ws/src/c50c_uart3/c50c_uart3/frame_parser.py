#!/usr/bin/env python3
"""C50C UART3 50 字节状态帧解析器。

协议依据（本仓库 `c50c-firmware-ref/` 只读存档，SHA-256 见 task-results 交付报告；
2026-11-15 固件在 48 字节基础上追加超声波字段扩到 50 字节）：

- 帧常量: `c50c-firmware-ref/BALANCE/Inc/data_task.h`
  ``FRAME_HEADER=0x7B``、``FRAME_TAIL=0x7D``、``SEND_DATA_SIZE=50``。
- 组帧: `c50c-firmware-ref/BALANCE/data_task.c` 的 ``data_transition()``（20 Hz，
  ``DATA_TASK_RATE=RATE_20_HZ``）与 ``Usart3_SendTask()``。
  数值字段为大端 16 位有符号（固件 ``short`` 高字节在前，``buffer[2]=X_speed>>8`` 等）。
- 校验: ``Send_Data.buffer[48]=Check_BCC(Send_Data.buffer,48)``，前 48 字节 XOR。
- 40~45 字节块（task-15 新增）: ``[22..37]`` 四轮 ``robot.MOTOR_A..D.Encoder``
  （int32 大端，**数值是浮点轮速转 int**，不是原始脉冲计数）、``[38..45]``
  ``TIM8->CCR1..CCR4``（uint16 大端，PWM 比较值真实输出）。
- 46~47 字节块（2026-11-15 超声波新增）: ``[46..47]`` 前置 HC-SR04 距离
  （uint16 大端，mm；``0xFFFF`` = 无回波/数据陈旧；传感器规格 2–450 cm，
  安装位置车头正中、离地约 5 cm）。
- 参考解码器: `c50c-firmware-ref/tools/c50c_uart3_monitor.py` 的 ``StatusFrameParser``。
  本模块的同步/校验/重同步语义与该已验证解码器一致：搜索帧头 → 取 50 字节 →
  校验帧尾与 BCC → 失败则从头字节起滑动 1 字节重新同步。

固件侧同一串口还可能交错发送其他帧（``Usart3_SendTask()``）：

- 19 字节超声波帧：本基线 ``SysVal.HardWare_Ranger`` 初始化为 0，不发送
  （超声波数据已改走 50 字节状态帧 [46..47]，不使用该无校验帧）；
- 8 字节回充帧（头 0x7C、尾 0x7F，无校验）：``HardWare_charger`` 或
  ``HardWare_Ranger`` 为 1 时才发送，本基线为 0，不发送。

两种帧都没有校验字段，无法被可靠识别；本解析器不做特判，依靠
"帧头候选 + 帧尾/BCC 校验 + 滑动重同步" 消化任何交错字节。若现场固件版本
打开了这些可选帧，解析器仍应恢复同步，但建议现场核对固件配置。

本模块不做任何单位换算（换算见 `conversions` 模块），不提供姿态四元数，
不构造采样时间戳——帧内没有时间戳与序列号，接收时刻由调用方（ROS 节点）
用主机时钟打戳，属已记录的限制。
"""
from __future__ import annotations

from dataclasses import dataclass

FRAME_LEN = 50
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
BCC_COVER_LEN = 48

# 字段偏移（data_task.c buffer[N] 赋值，大端 16 位有符号除注明外）
OFF_STOP = 1
OFF_VEL_X = 2
OFF_VEL_Y = 4
OFF_YAW_RATE = 6
OFF_ACCEL_X = 8
OFF_ACCEL_Y = 10
OFF_ACCEL_Z = 12
OFF_GYRO_X = 14
OFF_GYRO_Y = 16
OFF_GYRO_Z = 18
OFF_VOLTAGE = 20
OFF_BCC = 48
OFF_ENCODER = 22
OFF_PWM = 38
OFF_ULTRASONIC = 46
OFF_FRAME_TAIL = 49


@dataclass(frozen=True)
class StatusFrame:
    """一帧解码后的 C50C 状态数据，字段名与单位可追溯。

    三类数据严格区分（来源见 data_task.c ``data_transition()``）：

    - ``chassis_*``: 底盘速度/偏航角速度，来自后轮编码器里程计差速解算，
      **不是 IMU 数据**；编码器方向、轮径、轮距未完成 ROS 现场验证。
    - ``accel_raw_lsb`` / ``gyro_raw_lsb``: MPU-6050 原始差值（LSB），
      固件在组帧时已做芯片轴 → ROS 轴映射（chipY→X、-chipX→Y、chipZ→Z）。
      ``gyro_raw_lsb[2]`` 在 ``stop_flag != 0`` 时被固件强制置零，不可信。
    - ``stop_flag``: ``robot_control.FlagStop``（0=使能，非 0=软件失能）。
    """

    stop_flag: int
    chassis_velocity_mm_s: tuple[int, int]  # (vx, vy)，mm/s，编码器里程计
    chassis_yaw_rate_mrad_s: int            # mrad/s，编码器差速解算，非 IMU
    accel_raw_lsb: tuple[int, int, int]     # (x, y, z)，LSB（±2g 量程）
    gyro_raw_lsb: tuple[int, int, int]      # (x, y, z)，LSB（±500 dps 量程）
    voltage_mv: int                         # 电池电压，mV
    encoder_counts: tuple[int, int, int, int]  # (A, B, C, D)，int32，浮点轮速转 int
    pwm_counts: tuple[int, int, int, int]      # (CCR1..CCR4)，uint16，PWM 真实比较值
    ultrasonic_front_mm: int                 # 前置 HC-SR04 距离 mm，0xFFFF=无回波/陈旧
    frame_bytes: bytes                      # 原始 50 字节，供追溯/日志


class StatusFrameParser:
    """增量式帧解析器：支持分段输入、连续多帧与坏数据后重新同步。

    解析语义与 ``c50c-firmware-ref/tools/c50c_uart3_monitor.py`` 的
    ``StatusFrameParser`` 保持一致（该工具经 task-results 静态交叉校验
    14 项全部通过，见 c50c-imu-audit 报告第 6 节）。
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        # 诊断计数：丢帧/坏校验不可由协议本身检出（无序列号，BCC 对偶数
        # 位翻转不敏感），这些计数仅反映本地候选帧被拒的情况。
        self.frames_accepted = 0
        self.candidates_rejected = 0

    def reset(self) -> None:
        """丢弃缓冲区中的残余半帧/垃圾字节，诊断计数保留累计值。

        串口断流重连时必须调用：断开前的残余字节会与新连接的数据拼接，
        且"残余前缀 + 新数据前缀"**可能恰好通过 BCC 与帧尾校验**拼成假帧
        （BCC 只是 48 字节 XOR，碰撞概率约 1/256 每候选，测试
        ``test_reset_prevents_cross_reconnect_splice`` 构造了确定性反例）；
        重连后的字节流也不保证连续（设备可能重启、丢字节），残余不可信。
        """
        self._buffer.clear()

    def feed(self, data: bytes | bytearray | memoryview) -> list[StatusFrame]:
        """送入任意长度的原始串口字节，返回其中完整解出的状态帧列表。"""
        self._buffer.extend(data)
        frames: list[StatusFrame] = []
        while True:
            try:
                start = self._buffer.index(FRAME_HEADER)
            except ValueError:
                # 缓冲区里没有任何帧头字节：全部为垃圾，丢弃
                self._buffer.clear()
                break
            if start > 0:
                del self._buffer[:start]
            if len(self._buffer) < FRAME_LEN:
                break  # 帧未收全，等待后续数据（分段接收）
            frame = bytes(self._buffer[:FRAME_LEN])
            if frame[OFF_BCC] != _bcc(frame) or frame[FRAME_LEN - 1] != FRAME_TAIL:
                # 校验失败：滑过这个帧头候选，从下一字节重新同步
                self.candidates_rejected += 1
                del self._buffer[0]
                continue
            frames.append(_decode(frame))
            del self._buffer[:FRAME_LEN]
            self.frames_accepted += 1
        return frames


def _bcc(frame: bytes) -> int:
    """前 48 字节 XOR，与固件 ``Check_BCC(buffer, 48)`` 一致。"""
    checksum = 0
    for b in frame[:BCC_COVER_LEN]:
        checksum ^= b
    return checksum


def _decode(frame: bytes) -> StatusFrame:
    def i16(offset: int) -> int:
        return int.from_bytes(frame[offset:offset + 2], 'big', signed=True)

    return StatusFrame(
        stop_flag=frame[OFF_STOP],
        chassis_velocity_mm_s=(i16(OFF_VEL_X), i16(OFF_VEL_Y)),
        chassis_yaw_rate_mrad_s=i16(OFF_YAW_RATE),
        accel_raw_lsb=(i16(OFF_ACCEL_X), i16(OFF_ACCEL_Y), i16(OFF_ACCEL_Z)),
        gyro_raw_lsb=(i16(OFF_GYRO_X), i16(OFF_GYRO_Y), i16(OFF_GYRO_Z)),
        voltage_mv=int.from_bytes(frame[OFF_VOLTAGE:OFF_VOLTAGE + 2], 'big'),
        encoder_counts=tuple(
            int.from_bytes(frame[OFF_ENCODER + i * 4:OFF_ENCODER + i * 4 + 4], 'big', signed=True)
            for i in range(4)
        ),
        pwm_counts=tuple(
            int.from_bytes(frame[OFF_PWM + i * 2:OFF_PWM + i * 2 + 2], 'big')
            for i in range(4)
        ),
        ultrasonic_front_mm=int.from_bytes(
            frame[OFF_ULTRASONIC:OFF_ULTRASONIC + 2], 'big'),
        frame_bytes=frame,
    )
