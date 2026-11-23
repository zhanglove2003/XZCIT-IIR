"""四轮输出路径的**提取模型**判定：PS2 模式、受控 ROS 模式、停止路径。

模型本体见 ``test/firmware_harness/fw_ros_mode_model.py``（逐字提取基线
``uartx_callback.c`` / ``steer_debug.c`` / ``balance_task.c`` / ``data_task.c``
的真实函数体）。**每条结论都只是"模型判定，未上板"**：没有连接串口、
没有烧录、没有电机动作，也不证明板上固件版本。

本文件覆盖任务要求的四组判定：

1. PS2 模式下姿态仍由 PS2 驱动（手柄轴 → 四轮占空比）；
2. 未改动的基线固件在 ROS 模式下**进不了**四轮输出路径（根因）；
3. 受控 ROS 门放行后，合法速度帧切到 ``_ROS_Control``，四轮输出按
   ``Set_Pwm(-right, left, left, -right, 0)`` 产生非零；
4. ``FlagStop`` / ``Turn_Off`` / 失联 / ``Safe_Stop`` 的归零范围。

第 4 组里有一处模型判定与任务预期**不一致**，如实写成断言并在测试名里点明：
失联保护只清 ``Vx/Vy/Vz``，不清四轮调试输出（见 :func:`test_lost_frames_...`）。
"""
from __future__ import annotations

import pytest

from c50c_uart3.chassis_cmd.baseline import encode_baseline_speed_frame
from fw_ros_mode_model import RosModeModel

#: 基线同名同值（``BALANCE/Inc/balance_task.h:84-92``）
_PS2 = 1 << 1
_ROS = 1 << 0

#: 摇杆推到底的中间量：LY=200、RX=180（``axis_duty`` 死区 20、上限 8400）
_LY, _RX = 200, 180
#: 上两组轴值经真实 ``axis_duty``/``steer_debug_step`` 算出的期望占空比
_LEFT, _RIGHT = 6750, 1570


@pytest.fixture
def model():
    m = RosModeModel.start()
    try:
        yield m
    finally:
        m.close()


def _step(m: RosModeModel, *, ps2_alive: bool = True, ms: int = 20) -> None:
    """推进一拍：先走时钟再补 PS2 新鲜戳（真实固件里手柄帧每拍都刷新）。"""
    m.advance_ms(ms)
    if ps2_alive:
        m.mark_ps2_valid()
    m.tick()


def _arm(m: RosModeModel, *, ps2_alive: bool = True) -> None:
    """先过 100ms 中位（``ARM_NEUTRAL_MS``）才允许输出。"""
    m.set_pad(127, 127)
    for _ in range(7):
        _step(m, ps2_alive=ps2_alive)


def test_ps2_mode_four_wheel_output_follows_pad(model):
    """模型判定，未上板：PS2 模式下姿态仍由 PS2 摇杆驱动。"""
    model.set_mode_gate(0)          # 基线原样：ready 要求 _PS2_Control
    model.set_control_mode(_PS2)
    model.mark_ps2_valid()
    _arm(model)
    model.set_pad(_LY, _RX)
    _step(model)

    state = model.state()
    assert state['mode'] == _PS2, '模型判定，未上板：PS2 模式下控制模式应保持 PS2 位'
    assert state['pwm'] == (-_RIGHT, _LEFT, _LEFT, -_RIGHT, 0), (
        '模型判定，未上板：PS2 模式四轮输出应为 Set_Pwm(-right,left,left,-right,0)')
    assert state['ma'] == _RIGHT and state['mb'] == _LEFT
    assert state['mc'] == _LEFT and state['md'] == -_RIGHT


def test_ps2_mode_pad_centre_gives_zero(model):
    """模型判定，未上板：手柄回中即零输出（输出轴就是手柄轴）。"""
    model.set_mode_gate(0)
    model.set_control_mode(_PS2)
    model.mark_ps2_valid()
    model.set_pad(_LY, _RX)
    _step(model)
    model.set_pad(127, 127)
    _step(model)

    assert model.state()['pwm'] == (0, 0, 0, 0, 0), (
        '模型判定，未上板：手柄回中应输出零')


def test_baseline_firmware_ros_mode_cannot_enter_output_path(model):
    """模型判定，未上板：未改动基线在 ROS 模式下进不了四轮输出路径（根因）。"""
    model.set_mode_gate(0)          # 基线原样
    _arm(model)
    model.feed(encode_baseline_speed_frame(0.3, 0.0))

    state = model.state()
    assert state['mode'] == _ROS, (
        '模型判定，未上板：buffer[1]==0 的速度帧应把控制模式切到 _ROS_Control')
    assert state['vx'] == pytest.approx(0.3, abs=1e-6), (
        '模型判定，未上板：XYZ_Target_Speed_transition 应解出 0.3 m/s')
    assert state['lostcount'] == 0, '模型判定，未上板：合法帧应清丢帧计数'

    model.set_pad(_LY, _RX)
    _step(model)
    assert model.state()['pwm'] == (0, 0, 0, 0, 0), (
        '模型判定，未上板：ready 要求 _PS2_Control，ROS 模式必须输出零')


def test_ros_gate_enters_output_path_with_declared_setpwm_mapping(model):
    """模型判定，未上板：模式门放开后 ROS 模式四轮输出非零，映射不变。"""
    model.set_mode_gate(1)          # 受控 ROS 门：ready 改判 _ROS_Control
    model.mark_ps2_valid()
    _arm(model)
    model.feed(encode_baseline_speed_frame(0.3, 0.0))
    model.set_pad(_LY, _RX)
    _step(model)

    state = model.state()
    assert state['mode'] == _ROS
    assert state['pwm'] == (-_RIGHT, _LEFT, _LEFT, -_RIGHT, 0), (
        '模型判定，未上板：受控 ROS 门放行后应沿用 '
        'Set_Pwm(-right,left,left,-right,0)')
    # 映射自洽：第 1/4 参同为 -right、第 2/3 参同为 left、第 5 参恒 0
    pwm = state['pwm']
    assert pwm[0] == pwm[3] and pwm[1] == pwm[2] and pwm[4] == 0


def test_ros_gate_still_needs_live_ps2_link(model):
    """模型判定，未上板：ready 还要求 g_rear_ps2_valid，无手柄链路仍为零。"""
    model.set_mode_gate(1)
    model.feed(encode_baseline_speed_frame(0.3, 0.0))   # 帧到了，但没有 PS2 链路
    _arm(model, ps2_alive=False)
    model.set_pad(_LY, _RX)
    _step(model, ps2_alive=False)

    state = model.state()
    assert state['mode'] == _ROS
    assert state['pwm'] == (0, 0, 0, 0, 0), (
        '模型判定，未上板：没有 PS2 新鲜戳时 ready 为假，输出必须为零')


def test_corrupt_frame_does_not_switch_mode(model):
    """模型判定，未上板：BCC 不符的帧不改控制模式。"""
    model.set_mode_gate(0)
    model.set_control_mode(_PS2)
    frame = bytearray(encode_baseline_speed_frame(0.3, 0.0))
    frame[3] ^= 0xFF                     # 破坏载荷，BCC 失配
    model.feed(bytes(frame))

    state = model.state()
    assert state['mode'] == _PS2, '模型判定，未上板：坏 BCC 帧不得切模式'
    assert state['vx'] == 0.0, '模型判定，未上板：坏帧不得写入目标速度'


def test_flag_stop_zeroes_outputs_and_speeds(model):
    """模型判定，未上板：FlagStop（Turn_Off 为真）走 Safe_Stop，输出归零。"""
    model.set_mode_gate(1)
    model.mark_ps2_valid()
    _arm(model)
    model.feed(encode_baseline_speed_frame(0.3, 0.0))
    model.set_pad(_LY, _RX)
    _step(model)
    assert model.state()['ma'] != 0

    model.set_turn_off(True)
    _step(model)

    state = model.state()
    assert (state['ma'], state['mb'], state['mc'], state['md']) == (0, 0, 0, 0), (
        '模型判定，未上板：失能分支应把四路 Output 清零')
    assert state['vx'] == 0.0 and state['vy'] == 0.0 and state['vz'] == 0.0
    assert state['reg'] == (0, 0, 0, 0), (
        '模型判定，未上板：Safe_Stop 第 4 步应把占空比清零')


def test_release_after_flag_stop_requires_rearm(model):
    """模型判定，未上板：失能分支带 valid=0 调用，释放后必须重新过中位。"""
    model.set_mode_gate(1)
    model.mark_ps2_valid()
    _arm(model)
    model.set_pad(_LY, _RX)
    _step(model)
    model.set_turn_off(True)
    _step(model)

    model.set_turn_off(False)
    model.set_pad(_LY, _RX)
    _step(model)
    assert model.state()['pwm'] == (0, 0, 0, 0, 0), (
        '模型判定，未上板：未重新 arm 前推杆不得产生输出')

    _arm(model)
    model.set_pad(_LY, _RX)
    _step(model)
    assert model.state()['pwm'] == (-_RIGHT, _LEFT, _LEFT, -_RIGHT, 0), (
        '模型判定，未上板：重新过 100ms 中位后应恢复输出')


def test_safe_stop_clears_every_motion_quantity(model):
    """模型判定，未上板：Safe_Stop 一次清目标速度、输出、占空比、丢帧计数。"""
    model.set_mode_gate(1)
    model.mark_ps2_valid()
    _arm(model)
    model.feed(encode_baseline_speed_frame(0.3, 0.0))
    model.set_pad(_LY, _RX)
    _step(model)

    model.safe_stop()

    state = model.state()
    assert state['vx'] == 0.0 and state['vy'] == 0.0 and state['vz'] == 0.0
    assert (state['ma'], state['mb'], state['mc'], state['md']) == (0, 0, 0, 0)
    assert state['reg'] == (0, 0, 0, 0)
    assert state['lostcount'] == 0, '模型判定，未上板：Safe_Stop 应清丢帧计数'


def test_lost_frames_zero_target_speeds_only_not_four_wheel_output(model):
    """模型判定，未上板：失联只清 Vx/Vy/Vz，不清四轮调试输出。

    这是本模型与任务预期**不一致**的一条，照实记录：
    ``balance_task.c:113-121`` 的丢失保护只写三轴目标速度，``FlagStop``
    不受影响，而四轮调试输出读的是手柄轴。因此"超时失联 → 四轮归零"
    在基线上**不成立**；只有 ``FlagStop`` 为真（电压/急停/自检/软停）
    才会走 ``Safe_Stop`` 归零。
    """
    model.set_mode_gate(1)
    model.mark_ps2_valid()
    _arm(model)
    model.feed(encode_baseline_speed_frame(0.3, 0.0))
    model.set_pad(_LY, _RX)
    _step(model)
    assert model.state()['vx'] == pytest.approx(0.3, abs=1e-6)

    model.ticks(101)                 # 超过 BALANCE_TASK_RATE=100
    model.tick()

    state = model.state()
    assert state['lostcount'] > 100
    assert state['vx'] == 0.0 and state['vy'] == 0.0 and state['vz'] == 0.0, (
        '模型判定，未上板：丢帧保护应把三轴目标速度清零')
    assert state['pwm'] == (-_RIGHT, _LEFT, _LEFT, -_RIGHT, 0), (
        '模型判定，未上板：四轮调试输出不读三轴速度，失联后仍跟随手柄')
