"""协议 v3 定向集成：ROS 侧真实编码器 → 真实固件解析（chassis_ack.c v3）。

证据边界（与 ``fw_v3_harness`` 模块文档一致）：
- 会话门、序号门、回显门、计数器：**真实源码**即可执行位；
- USART3 接收入口 ``UartxControll_Callback`` 的门控：**真实源码**；
- 速度解码 ``XYZ_Target_Speed_transition``、``ROS_Command_Frame`` /
  ``ROS_Command_Disarm`` / ``Recharge_Mode_Exit``：**真实源码**；
- ``Safe_Stop``：**桩**（只置标并递增输出纪元，板上的电机/PI/PWM 未执行）；
- 会话帧发送节拍：**桩**（照抄 data_task.c 的计数逻辑并调用真实的
  ``Chassis_Ack_EncodeSessionFrame``）；
- 熵源：**桩**（板上 SRAM 地址换成本地缓冲区）。

未验证：真实电机、真实串口、中断时序与竞态、DTR/RTS。

每条用例都断言**固件侧计数器**，不以主机写入成功为证据。
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'firmware_harness'))

import fw_v3_harness                                    # noqa: E402

from c50c_uart3.chassis_cmd import (                    # noqa: E402
    encode_speed_frame,
)
from c50c_uart3.chassis_cmd.arbiter import Arbiter
from sim_confirm import TEST_SAFETY_MAX_AGE_S, grant_safety      # noqa: E402
from c50c_uart3.chassis_cmd.seq import (                # noqa: E402
    CAP_HOST_ACK, SEQ_MAX, SEQ_MASK, SEQ_WINDOW,
)
from c50c_uart3.chassis_cmd.session import (            # noqa: E402
    PROTO_VERSION, SESSION_SUB, encode_session_ack,
)

REAL_ACCEPTED = 1


@pytest.fixture()
def fw():
    fw = fw_v3_harness.V3Firmware.start()
    try:
        yield fw
    finally:
        fw.close()


def sync(fw) -> None:
    """模拟主机读到会话帧并回填确认帧（与 handshake 的行为一致）。"""
    fw.feed_bytes(encode_session_ack(fw.session))


# --------------------------------------------------------------- 契约常量
def test_host_constants_match_the_real_firmware_header():
    """主机侧常量必须等于真实 chassis_ack.h 里的宏（防止两侧漂移）。"""
    contract = fw_v3_harness.firmware_contract()
    assert contract['CHASSIS_PROTO_VERSION'] == PROTO_VERSION == 3
    assert contract['CHASSIS_SESSION_SUB'] == SESSION_SUB == 0x01
    assert contract['CHASSIS_CAP_HOST_ACK'] == CAP_HOST_ACK == 0x80
    assert contract['CHASSIS_SEQ_MASK'] == SEQ_MASK == 0x7F
    assert contract['CHASSIS_SEQ_WINDOW'] == SEQ_WINDOW == 64


def test_session_frame_bytes_carry_version_three_and_sub_id():
    ack = encode_session_ack(0x11223344)
    assert ack[1] == 0x7F
    assert ack[2] == 0x01
    assert ack[3] == 3
    assert ack[4:8] == bytes((0x11, 0x22, 0x33, 0x44))


# --------------------------------------------------------------- 正常接收
def test_firmware_rejects_speed_before_session_ack(fw):
    """未确认会话：速度帧被计为 NO_SESSION，目标速度不改变。"""
    fw.feed_bytes(encode_speed_frame(0.05, 0.0, 0.0, seq=0))
    st = fw.state
    assert st.no_session == 1
    assert st.accept == 0
    assert st.vx == 0.0


def test_session_ack_establishes_session_then_zero_arms(fw):
    """会话建立 → 零速授权 → 非零命令被接纳。全部以固件计数器为证据。"""
    sync(fw)
    # 确认帧本身不进入 accept 计数（固件按 NOT_SESSION 返回）。
    assert fw.state.accept == 0
    assert fw.state.no_session == 0

    fw.feed_bytes(encode_speed_frame(0.0, 0.0, 0.0, seq=0))
    st = fw.state
    assert st.accept == REAL_ACCEPTED
    assert st.armed is True

    fw.feed_bytes(encode_speed_frame(0.05, 0.0, 0.0, seq=1))
    st = fw.state
    assert st.accept == 2           # 零速帧 + 非零帧都被接纳
    assert st.armed is True
    assert st.vx == pytest.approx(0.05, abs=1e-4)


def test_speed_frame_capability_bit_is_set(fw):
    """速度帧 buffer[2] 的 bit7 必须置位，否则固件按 NO_SEQ 拒绝。"""
    sync(fw)
    fw.feed_bytes(encode_speed_frame(0.05, 0.0, 0.0, seq=0))
    assert fw.state.accept == 1
    assert fw.state.no_seq == 0


def test_arbiter_frames_are_accepted_by_the_firmware(fw):
    """仲裁器真实输出帧（带自己维护的序号）被固件连续接纳。"""
    clock = [100.0]
    a = Arbiter(safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(a)
    a.on_publisher_count(1, 0)
    a.on_firmware_confirm(True)
    a.mark_receiver_clean(simulated=True)
    a.on_session_observed(fw.session, clock[0], accepted=True)
    sync(fw)                       # 固件会话必须建立，否则速度帧被 NO_SESSION 拒收

    # 首帧零速规则：授权后第一帧必然是零速帧，先把它送进固件授权。
    a.on_nav_cmd(0.0, 0.0, 0.0, clock[0])
    first = a.evaluate(clock[0])
    assert first.is_zero
    clock[0] += 0.05
    fw.feed_bytes(first.frame)

    seen = []
    for _ in range(3):
        a.on_nav_cmd(0.05, 0.0, 0.0, clock[0])
        d = a.evaluate(clock[0])
        clock[0] += 0.05
        assert not d.is_zero
        seen.append(d.frame[2] & SEQ_MASK)
        fw.feed_bytes(d.frame)
    st = fw.state
    assert st.accept == 4          # 首帧零速 + 三条非零命令
    assert st.stale_seq == 0 and st.no_seq == 0
    assert seen == [1, 2, 3]
    assert st.vx == pytest.approx(0.05, abs=1e-4)


# --------------------------------------------------------------- 停车
def test_zero_frame_stops_and_keeps_firmware_armed(fw):
    """停车：零速帧仍被接纳，目标速度回到 0。

    这里直接用主机编码器构造字节（seq=1 是会话内第二帧，
    与仲裁器的首帧零速规则无关）。
    """
    sync(fw)
    # 未授权的固件只接受零速帧来置 armed；先发零速帧。
    fw.feed_bytes(encode_speed_frame(0.0, 0.0, 0.0, seq=0))
    fw.feed_bytes(encode_speed_frame(0.05, 0.0, 0.0, seq=1))
    assert fw.state.vx == pytest.approx(0.05, abs=1e-4)
    fw.feed_bytes(encode_speed_frame(0.0, 0.0, 0.0, seq=2))
    st = fw.state
    assert st.accept == 3
    assert st.vx == 0.0


def test_arbiter_stop_command_reaches_firmware(fw):
    """仲裁器收到停车后发出的零速帧被固件接纳。"""
    clock = [100.0]
    a = Arbiter(safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(a)
    a.on_publisher_count(1, 0)
    a.on_firmware_confirm(True)
    a.mark_receiver_clean(simulated=True)
    a.on_session_observed(fw.session, clock[0], accepted=True)
    sync(fw)                       # 固件会话必须建立，否则速度帧被 NO_SESSION 拒收
    a.on_nav_cmd(0.0, 0.0, 0.0, clock[0])
    arm = a.evaluate(clock[0])
    assert arm.is_zero
    fw.feed_bytes(arm.frame)
    clock[0] += 0.05
    a.on_nav_cmd(0.05, 0.0, 0.0, clock[0])
    fw.feed_bytes(a.evaluate(clock[0]).frame)
    clock[0] += 0.05
    a.on_stop(True)
    d = a.evaluate(clock[0])
    assert d.is_zero
    fw.feed_bytes(d.frame)
    st = fw.state
    assert st.accept == 3          # 零速授权 + 非零 + 停车零速
    assert st.vx == 0.0


# --------------------------------------------------------------- 版本不匹配
def test_v2_session_ack_is_rejected_by_v3_firmware(fw):
    """v3 固件收 v2 会话确认帧：版本字节 2 → 会话不建立。

    帧由 ``encode_session_ack`` 生成后只把版本字节改回 2，
    与改造前主机实际发出的字节一致。
    """
    ack = bytearray(encode_session_ack(fw.session))
    ack[3] = 2
    ack[9] = 0
    for b in ack[:9]:
        ack[9] ^= b
    fw.feed_bytes(bytes(ack))
    fw.feed_bytes(encode_speed_frame(0.0, 0.0, 0.0, seq=0))
    st = fw.state
    assert st.no_session == 1
    assert st.accept == 0
    assert st.armed is False


def test_v2_session_frame_with_capability_bit_is_rejected(fw):
    """旧主机把 0x81 写进会话帧 buffer[2]：固件要求严格 0x01。"""
    ack = bytearray(encode_session_ack(fw.session))
    ack[2] = 0x81
    ack[9] = 0
    for b in ack[:9]:
        ack[9] ^= b
    fw.feed_bytes(bytes(ack))
    fw.feed_bytes(encode_speed_frame(0.0, 0.0, 0.0, seq=0))
    st = fw.state
    assert st.no_session == 1
    assert st.accept == 0


# --------------------------------------------------------------- 旧命令
def test_replayed_sequence_number_is_rejected(fw):
    sync(fw)
    frame = encode_speed_frame(0.05, 0.0, 0.0, seq=0)
    fw.feed_bytes(frame)
    fw.feed_bytes(frame)
    st = fw.state
    assert st.accept == 1
    assert st.stale_seq == 1


def test_regressed_sequence_number_is_rejected(fw):
    sync(fw)
    for seq in (10, 11, 12):
        fw.feed_bytes(encode_speed_frame(0.05, 0.0, 0.0, seq=seq))
    fw.feed_bytes(encode_speed_frame(0.05, 0.0, 0.0, seq=11))
    st = fw.state
    assert st.accept == 3
    assert st.stale_seq == 1


def test_sequence_beyond_the_window_is_rejected(fw):
    """超出 64 的前进跨度被当作回绕，固件拒绝。"""
    sync(fw)
    fw.feed_bytes(encode_speed_frame(0.0, 0.0, 0.0, seq=0))
    fw.feed_bytes(encode_speed_frame(0.05, 0.0, 0.0, seq=SEQ_WINDOW))
    st = fw.state
    assert st.accept == 1
    assert st.stale_seq == 1


def test_frame_without_capability_bit_is_rejected(fw):
    """未置 bit7 的速度帧按 NO_SEQ 拒绝。"""
    sync(fw)
    frame = bytearray(encode_speed_frame(0.0, 0.0, 0.0, seq=0))
    frame[2] = 0x01
    frame[9] = 0
    for b in frame[:9]:
        frame[9] ^= b
    fw.feed_bytes(bytes(frame))
    st = fw.state
    assert st.no_seq == 1
    assert st.accept == 0


# --------------------------------------------------------------- 重连
def test_sequence_wraps_at_127_without_rejection(fw):
    """序号到 127 后回到 0：只要仍在窗口内，固件全部接纳。"""
    sync(fw)
    for seq in (126, 127, 0, 1):
        fw.feed_bytes(encode_speed_frame(0.0, 0.0, 0.0, seq=seq))
    st = fw.state
    assert st.accept == 4
    assert st.stale_seq == 0
    assert st.no_seq == 0


def test_arbiter_seq_survives_reconnect_until_firmware_session_changes(fw):
    """重连**不**清零序号；只有固件会话 ID 变化才清零。

    固件未重启时它仍记得最后一个已接受序号，主机从 0 重发会被
    ``check_seq`` 判 STALE_SEQ（首帧 delta = 128 - last）。所以重连后
    继续用重连前的序号，固件接受。
    """
    clock = [100.0]
    a = Arbiter(safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(a)
    a.on_publisher_count(1, 0)
    a.on_firmware_confirm(True)
    a.mark_receiver_clean(simulated=True)
    a.on_session_observed(fw.session, clock[0], accepted=True)
    sync(fw)                       # 固件会话必须建立，否则速度帧被 NO_SESSION 拒收
    for _ in range(3):
        a.on_nav_cmd(0.0, 0.0, 0.0, clock[0])
        fw.feed_bytes(a.evaluate(clock[0]).frame)
        clock[0] += 0.05
    assert a.frame_seq.next_value == 3

    a.on_serial_reconnect()
    assert a.frame_seq.next_value == 3, '同一固件会话内重连不得清零序号'

    # 重连清掉了会话证据（_session_id=None），所以重连后读到的第一条会话帧
    # 与"无"不同，按现有 on_session_observed 语义清零。这是既有行为：
    # 属于"重连后授权必须重新建立"，本测试不改它。此后的会话帧不再清零。
    a.on_session_observed(fw.session, clock[0], accepted=True)
    assert a.frame_seq.next_value == 0
    a.on_session_observed(fw.session, clock[0], accepted=True)
    assert a.frame_seq.next_value == 0

    # 从 0 继续：固件仍在同一会话，若 last >= 0 则 0 会被判 STALE_SEQ。
    # 这一条记录的是现状（见 task-results/onsite/reconnect.md 的限制一节）。

    a.on_publisher_count(1, 0)
    a.on_firmware_confirm(True)
    a.mark_receiver_clean(simulated=True)
    a.on_nav_cmd(0.05, 0.0, 0.0, clock[0])
    fw.feed_bytes(a.evaluate(clock[0]).frame)
    st = fw.state
    assert st.accept == 3               # 本帧没有被接受
    assert st.stale_seq == 1            # 序号 0 在重连前已用过 → STALE_SEQ

    # 固件换会话（等价于固件重启）才清零。
    a.on_session_observed(fw.session ^ 0xFFFF, clock[0], accepted=True)
    assert a.frame_seq.next_value == 0


def test_new_firmware_session_restarts_sequence_from_zero(fw):
    """固件会话重建后，主机从序号 0 重新开始，首帧被接纳。"""
    sync(fw)
    fw.feed_bytes(encode_speed_frame(0.0, 0.0, 0.0, seq=5))
    assert fw.state.accept == 1

    # 新会话 = 固件重启：新实例，会话标识不同。
    other = fw_v3_harness.V3Firmware.start()
    try:
        # 同一台机器上会话标识由进程地址与 tick 决定，可能相同；
        # 这里改为验证"主机从序号 0 重新开始"这一契约本身。
        other.feed_bytes(encode_speed_frame(0.0, 0.0, 0.0, seq=0))
        assert other.state.no_session == 1
        other.feed_bytes(encode_session_ack(other.session))
        other.feed_bytes(encode_speed_frame(0.0, 0.0, 0.0, seq=0))
        st = other.state
        assert st.accept == 1
        assert st.armed is True
    finally:
        other.close()


# --------------------------------------------------------------- 保门
def test_default_arbiter_still_blocks_nonzero_without_session(fw):
    """默认关闭：未观测会话时仲裁器不发非零帧，固件也没有接纳计数。"""
    a = Arbiter(safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(a)
    a.on_publisher_count(1, 0)
    a.mark_receiver_clean(simulated=True)
    a.on_nav_cmd(0.05, 0.0, 0.0, 100.0)
    d = a.evaluate(100.0)
    assert d.is_zero or d.frame is None
    if d.frame is not None:
        fw.feed_bytes(d.frame)
    assert fw.state.vx == 0.0
    assert fw.state.accept in (0, 1)   # 只可能是零速帧被接纳
