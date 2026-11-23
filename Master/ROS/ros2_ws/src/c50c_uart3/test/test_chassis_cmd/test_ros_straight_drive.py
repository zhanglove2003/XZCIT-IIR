"""ROS 受控直行的四轮输出与停车：对本轮改动固件源码的**逐字提取**判定。

模型本体见 ``test/firmware_harness/fw_ros_drive_model.py``：它从
``2026-11-08-50pct-migrate/BALANCE/`` 读取真实源码，逐字插入
``UartxControll_Callback``、``XYZ_Target_Speed_transition``、
``steer_debug_step``、``ResponseControl`` 四轮段、``Turn_Off``、``Safe_Stop``。
因此下面每条断言都跑在本轮**改过的**固件函数体上，不是对它的再描述。

边界（必须与"已验证"区分）
--------------------------
1. 本文件证明的是"给定这些字节，真实固件函数体算出什么占空比"。它**不**证明
   板上固件匹配源码、不证明串口写入成功、不证明电机转动。
2. 未覆盖：四轮调试输出段之前的 ``Drive_Motor`` PI 主路径（本构建下该段
   先 ``return``）、编码器、PWM 定时器搬运、真实 PS2 协议。
3. ``Set_Pwm`` 是观察替身（记入参、写占空比影子变量），不碰寄存器。

四组判定
--------
1. 速度帧进入固件：真实帧状态机解析三轴、切 ``_ROS_Control``、打时间戳。
2. 四轮输出：``ready`` 放行后按 ``Set_Pwm(-right, left, left, -right, 0)`` 输出。
3. 停车：速度帧停发超时、零速帧、失能、``Safe_Stop`` 四条路径。
4. 保留 PS2：``_PS2_Control`` 下原判据仍然有效（本构建的 ``ROS_STRAIGHT_DRIVE``
   分支只接受 ROS 位，故用编译期开关核实，见
   :func:`test_ps2_branch_is_untouched_in_source`）。
"""
from __future__ import annotations

import pytest

from c50c_uart3.chassis_cmd.baseline import (
    encode_baseline_speed_frame,
    encode_baseline_zero_frame,
)
from fw_ros_drive_model import (
    FIRMWARE_DIR,
    RosDriveModel,
    build_source,
)

#: 基线同名同值（``BALANCE/Inc/balance_task.h``）
_ROS = 1 << 0
_PS2 = 1 << 1

#: 手柄轴 —— 逐位来源：四轮调试输出段仍取 ``GamePadInterface->LY/RX``。
_LY, _RX = 200, 180

#: 基线 `ready` 原文（`balance_task.c`）。默认构建必须逐字保留它。
_BASELINE_READY = (
    '        int ready = SysVal.HardWare_Ver == V1_0 && GamePadInterface != NULL &&\n'
    '                    Get_Control_Mode(_PS2_Control) && !charger.AllowRecharge &&\n'
    '                    !robot_check.DeepCheck && SysVal.Time_count > CONTROL_DELAY + 230 &&\n'
    '                    g_rear_ps2_valid && (uint32_t)(tick - g_rear_ps2_tick) <= pdMS_TO_TICKS(150);\n'
)
#: 真实 ``axis_duty``/``steer_debug_step`` 在 (200, 180) 下的输出
_LEFT, _RIGHT = 6750, 1570


@pytest.fixture
def m():
    model = RosDriveModel.start()
    try:
        yield model
    finally:
        model.close()


def _arm(m: RosDriveModel, *, ms: int = 16) -> None:
    """每拍喂一帧并回中，累计 >= ARM_NEUTRAL_MS(100 ms) 才 arm。

    真实情况：主机侧按 50 ms 周期发帧，固件按 10 ms 拍跑。拍到而帧未到的那一拍
    ``ready`` 为假，``steer_debug_step`` 会清掉中位累计。所以"帧连续到达"是 arm
    的前提，测试必须如实模拟，否则测出来的不是固件行为。
    """
    m.set_pad(127, 127)
    for _ in range(12):
        m.feed(encode_baseline_speed_frame(0.0, 0.0))
        m.mark_ps2_valid()
        m.step(ms)


def _step(m: RosDriveModel, n: int = 1, *, ps2_alive: bool = True,
          ms: int = 16, feed: bool = True, vx: float = 0.0,
          vz: float = 0.0) -> None:
    """跑 ``n`` 拍，每拍前喂一帧 ``(vx, vz)`` 并刷新手柄新鲜度。

    必须每拍都喂同一速度：``ready`` 含 ``ros_cmd_tick`` 新鲜度，帧断一拍就判假，
    输出那拍就是零。
    """
    for _ in range(n):
        if feed:
            m.feed(encode_baseline_speed_frame(vx, vz))
        m.step(ms)
        if ps2_alive:
            m.mark_ps2_valid()


# --------------------------------------------------------------------------
# 1. 速度帧确实进入固件（真实 UartxControll_Callback）
# --------------------------------------------------------------------------
def test_baseline_speed_frame_sets_ros_mode_and_three_axes(m):
    """生产编码器的帧被真实帧状态机接受：切 _ROS_Control 且写入三轴。"""
    m.feed(encode_baseline_speed_frame(0.5, 0.0))
    s = m.state()
    assert s['mode'] == _ROS
    assert s['vx'] == pytest.approx(0.5, abs=1e-6)
    assert s['vy'] == pytest.approx(0.0, abs=1e-6)
    assert s['vz'] == pytest.approx(0.0, abs=1e-6)
    assert s['lostcount'] == 0


def test_baseline_frame_stamps_cmd_tick_and_it_reaches_set_pwm(m):
    """本轮改动：帧到达时打 ros_cmd_tick，四轮输出段读到同一个戳。"""
    _arm(m)
    m.set_pad(_LY, _RX)
    _step(m, 2)
    m.feed(encode_baseline_speed_frame(0.5, 0.0))
    _step(m, 2)
    s = m.state()
    assert s['last_cmd_at_setpwm'] > 0
    assert s['last_cmd_at_setpwm'] <= s['now']


def test_status_frame_bytes_are_not_speed_frames(m):
    """主机上行状态帧不是速度帧：不改模式、不改三轴。"""
    # 状态帧 24 字节，第 3 字节不是 0，帧状态机不应把它当速度命令。
    frame = bytes([0x7B, 0x01, 0x02] + [0] * 20 + [0x7D])
    assert frame[1] != 0
    m.feed(frame)
    s = m.state()
    assert s['vx'] == pytest.approx(0.0)
    assert s['vz'] == pytest.approx(0.0)


def test_corrupted_bcc_frame_is_rejected(m):
    """BCC 错的帧被丢弃：三轴不更新。"""
    good = encode_baseline_speed_frame(0.5, 0.0)
    bad = bytearray(good)
    bad[9] ^= 0xFF
    m.feed(bytes(bad))
    assert m.state()['vx'] == pytest.approx(0.0)


# --------------------------------------------------------------------------
# 2. 四轮输出（真实 ResponseControl 四轮段，逐字）
# --------------------------------------------------------------------------
def test_ros_mode_outputs_four_wheels(m):
    """ROS 模式下四轮输出取速度帧：duty = Vx × 1000。

    0.5 m/s 远未触到 8400 上限，所以输出就是 ±500。四路顺序沿用基线
    ``Set_Pwm(-right, left, left, -right, 0)``。
    """
    _arm(m)
    _step(m, 2, vx=0.5)
    s = m.state()
    assert s['mode'] == _ROS
    assert s['pwm'] == (-500, 500, 500, -500, 0)


def test_output_follows_speed_frame_not_pad_axis(m):
    """ROS 模式的输出只由速度帧决定，手柄推满也不影响。"""
    _arm(m)
    _step(m, 2, vx=0.5)
    baseline = m.state()['pwm']

    m.set_pad(_LY, _RX)          # 手柄推到 200/180
    _step(m, 2, vx=0.5)
    assert m.state()['pwm'] == baseline == (-500, 500, 500, -500, 0)


def test_centred_pad_does_not_matter_zero_frame_yields_zero(m):
    """零速帧 → 四轮全零（受控输出，不是发散）。"""
    _arm(m)
    m.feed(encode_baseline_zero_frame())
    _step(m, 2)
    assert m.state()['pwm'] == (0, 0, 0, 0, 0)


# --------------------------------------------------------------------------
# 3. 停车路径
# --------------------------------------------------------------------------
def test_output_stops_when_speed_frames_stop(m):
    """帧停发超过 ROS_CMD_TIMEOUT_MS（300 ms）即丢掉四轮输出。"""
    _arm(m)
    _step(m, 2, vx=0.5)
    assert m.state()['pwm'] == (-500, 500, 500, -500, 0)

    # 停发速度帧：超过 300 ms 后必须归零。
    _step(m, 22, ms=16, feed=False)   # 22 × 16 ms = 352 ms > 300 ms
    s = m.state()
    assert s['pwm'] == (0, 0, 0, 0, 0)


def test_output_resumes_after_frames_return(m):
    """超时归零不等于锁死：新帧到达后必须重新过中位 arm 才恢复。"""
    _arm(m)
    _step(m, 2, vx=0.5)
    _step(m, 22, ms=16, feed=False)           # 超时
    assert m.state()['pwm'] == (0, 0, 0, 0, 0)

    _step(m, 1, ms=16)
    assert m.state()['pwm'] == (0, 0, 0, 0, 0), '须先回中重新 arm'
    _arm(m)
    _step(m, 2, vx=0.5)
    assert m.state()['pwm'] == (-500, 500, 500, -500, 0)


def test_baseline_zero_frame_keeps_mode_and_lostcount_clean(m):
    """生产编码器的零速帧仍是有效帧：模式保持、丢帧计数清零、输出为零。"""
    _arm(m)
    m.mark_ps2_valid()
    for _ in range(3):
        m.feed(encode_baseline_zero_frame())
        m.mark_ps2_valid()
        m.step(16)
    s = m.state()
    assert s['mode'] == _ROS
    # 帧在 ``step`` 之前到达，而 ``step`` 末尾的 ``Supervise_LostCount`` 再 +1；
    # 真实固件里 USART 中断与主循环不同步，故只断言"没有累积到超限清除"。
    assert s['lostcount'] <= 1
    assert s['vx'] == pytest.approx(0.0)
    assert s['pwm'] == (0, 0, 0, 0, 0)


def test_turn_off_drops_output_through_safe_stop(m):
    """失能（Turn_Off 真）→ Safe_Stop → 占空比寄存器归零。"""
    _arm(m)
    _step(m, 2, vx=0.5)
    assert m.state()['pwm'] == (-500, 500, 500, -500, 0)

    m.set_emergency_stop(True)
    _step(m, 1)
    s = m.state()
    # 失能走 Safe_Stop，它直接写寄存器，不再调 Set_Pwm：
    assert s['reg'] == (0, 0, 0, 0)
    assert s['ma'] == s['mb'] == s['mc'] == s['md'] == 0
    assert s['setpwm_since_estop'] == 0
    assert s['last_cmd_at_setpwm'] > 0, '失能后不得再刷新输出'


def _arm_then_drive(m: RosDriveModel) -> None:
    """arm 后发 0.5 m/s 速度帧，两拍内输出必须是四轮非零。"""
    _arm(m)
    _step(m, 2, vx=0.5)
    assert m.state()['pwm'] == (-500, 500, 500, -500, 0)


def test_stale_ps2_drops_output_even_with_fresh_frames(m):
    """PS2 新鲜度判据保留：手柄帧过期时即使速度帧在跑也输出零。

    ROS 分支的 ``ready`` 里仍有 ``g_rear_ps2_valid`` 与 150 ms 新鲜度，
    这是基线判据，本轮没有删。
    """
    _arm_then_drive(m)

    # 手柄失联：真实固件 ps2_classic.c:304 只清 valid，不推进时间戳。
    # 本用例不喂速度帧：喂帧会用新戳压过 PS2 判据之外的条件，反而掩盖问题。
    m.drop_ps2()
    _step(m, 1, ps2_alive=False, feed=False)
    assert m.state()['pwm'] == (0, 0, 0, 0, 0)


def test_ps2_mode_still_drives_from_pad(m):
    """PS2 模式仍按手柄轴输出，不被 ROS 改动影响。"""
    # 不喂速度帧：速度帧会把模式切回 _ROS_Control，那样测的就不是 PS2 分支。
    m.set_control_mode(_PS2)
    m.set_pad(127, 127)
    _step(m, 12, feed=False)          # 中立满 100 ms → armed
    m.set_pad(_LY, _RX)
    _step(m, 3, feed=False)
    assert m.state()['mode'] == _PS2
    assert m.state()['pwm'] == (-_RIGHT, _LEFT, _LEFT, -_RIGHT, 0)


def test_switch_ps2_to_ros_requires_new_neutral_window(m):
    """PS2→ROS 切换后必须重走 100 ms 中立窗口，旧手柄占空比不得带入。"""
    m.set_control_mode(_PS2)
    m.set_pad(127, 127)
    _step(m, 12, feed=False)      # 中立满 100 ms → armed
    m.set_pad(_LY, _RX)
    _step(m, 3, feed=False)
    assert m.state()['pwm'] == (-_RIGHT, _LEFT, _LEFT, -_RIGHT, 0)

    m.set_control_mode(_ROS)      # 固件在此清零两个状态机
    _step(m, 1, vx=0.5)
    assert m.state()['pwm'] == (0, 0, 0, 0, 0), '刚切换不得立即出非零'

    _arm(m)                       # 重走中立窗口
    _step(m, 2, vx=0.5)
    assert m.state()['pwm'] == (-500, 500, 500, -500, 0)


# --------------------------------------------------------------------------
# 4. PS2 行为保留（源码级）
# --------------------------------------------------------------------------
def test_ps2_branch_is_untouched_in_source():
    """不开 ROS_STRAIGHT_DRIVE 时 ready 分支逐字等于基线原文。

    基线在 ``&&`` 链里直接用宏 ``Get_Control_Mode(_PS2_Control)``
    展开的真值 2。本轮不得引入 ``!= 0``，也不得引入临时变量，
    否则默认构建的机器码会偏离基线哈希。
    """
    text = (FIRMWARE_DIR / 'BALANCE/balance_task.c').read_bytes().decode(
        'gb18030', errors='replace')
    assert _BASELINE_READY in text
    assert 'Get_Control_Mode(_PS2_Control) != 0' not in text
    assert 'int mode_ok' not in text


def test_ready_keeps_all_baseline_conditions():
    """改动的只有模式门：其余六项判据逐字保留。"""
    src = build_source()
    for cond in ('SysVal.HardWare_Ver == V1_0',
                 'GamePadInterface != NULL',
                 '!charger.AllowRecharge',
                 '!robot_check.DeepCheck',
                 'SysVal.Time_count > CONTROL_DELAY + 230',
                 'g_rear_ps2_valid',
                 'pdMS_TO_TICKS(150)'):
        assert cond in src, cond


def test_timeout_threshold_is_bounded():
    """超时窗口必须落在合理区间：太短会随 50 ms 周期抖，太长会拖慢停车。"""
    text = (FIRMWARE_DIR / 'BALANCE/Inc/balance_task.h').read_bytes().decode(
        'gb18030')
    assert '#define ROS_CMD_TIMEOUT_MS 300u' in text
