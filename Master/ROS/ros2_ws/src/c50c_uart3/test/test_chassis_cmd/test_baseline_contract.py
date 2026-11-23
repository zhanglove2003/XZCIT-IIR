"""基线固件（2026-11-05 交付基线）与候选 ROS 线的契约验证。

本文件是**契约证据**，不是新功能：它用基线 ``BALANCE/uartx_callback.c``
的真实解析代码（``fw_baseline_harness``）判定候选 ROS 侧写出的字节是否
被接受、速度是多少、停车是否生效。

覆盖（每条一个测试）：

1. 候选 v3 帧（``frame[2]=0x80|seq``）基线**接受**——基线不读 ``frame[2]``；
   候选线担心的 ack 位与序号在基线上不存在，所以它不是"坏字节"。
2. 但候选 v3 的**会话帧**（``frame[1]=0x7F``）在基线上是 ``buffer[1]==0xFF``
   以外的未知命令：基线不认，也不会回确认。
3. 候选桥在基线方言下**默认一口字节都不写**（会话证据永不存在）。
4. 切到基线方言后：零速帧接受、三轴为 0、``command_lostcount`` 清零。
5. 直行命令被接受，固件三轴目标速度等于命令值（量化 0.001 m/s）。
6. 停车（零速帧）被接受并把三轴目标速度清零。
7. 命令丢失保护：约 1 s 没有合法帧，基线自己把三轴清零。
8. 重连后**无接收方向证据时不写非零**；有状态帧证据时先发零速，再放行。
9. 重连后残留的半帧会把下一帧吞掉——"写成功"不等于"被接受"。
"""
import pytest

from fw_baseline_harness import BaselineModel

from c50c_uart3.chassis_cmd import encode_speed_frame
from c50c_uart3.chassis_cmd.arbiter import (Arbiter, PROTOCOL_BASELINE,
                                            PROTOCOL_V3, ZeroReason)
from c50c_uart3.chassis_cmd.baseline import (encode_baseline_speed_frame,
                                             encode_baseline_zero_frame,
                                             parse_status_frame)
from c50c_uart3.chassis_cmd.bridge import StraightDriveBridge
from c50c_uart3.chassis_cmd.serial_port import FakeFdPort
from sim_confirm import (mark_chain_clean, mark_endpoint_clean, new_receipt,
                         grant_safety_via_input, grant_safety,
                         TEST_SAFETY_MAX_AGE_S)

_SIM = new_receipt('baseline-contract')


class Clock:
    def __init__(self, start: float = 100.0) -> None:
        self.t = float(start)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> float:
        self.t += float(dt)
        return self.t


@pytest.fixture
def model():
    m = BaselineModel.start()
    try:
        yield m
    finally:
        m.close()


def _state(model) -> dict:
    return model.state()


def _baseline_bridge(clock):
    """基线方言的桥：假 fd 端口 + 测试收据放行接收端。"""
    port = FakeFdPort()
    bridge = StraightDriveBridge(port, device='/dev/pts/9', enabled=True,
                                 firmware_protocol=PROTOCOL_BASELINE,
                                 clock=clock)
    mark_endpoint_clean(port, receipt=_SIM)
    mark_chain_clean(bridge.chain, receipt=_SIM)
    grant_safety_via_input(bridge, clock())
    bridge.input.set_nav_publishers(1)
    return bridge, port


def _frames(port) -> list:
    data = bytes(port.stream)
    return [data[i:i + 11] for i in range(0, len(data) - len(data) % 11, 11)]


def test_baseline_sources_are_the_delivered_baseline():
    """构建模型的源文件必须是交付基线，且哈希与记录一致。"""
    import fw_baseline_harness as h
    hashes = h.verify_sources()
    assert hashes == h.RECORDED_SHA256


def test_candidate_v3_speed_frame_is_accepted_by_baseline(model):
    """候选 v3 帧：基线不读 frame[2]，因此 ack 位与序号都被忽略。"""
    frame = encode_speed_frame(0.05, 0.0, 0.0, seq=37)
    assert frame[2] == (0x80 | 37)
    assert model.feed_bytes(frame) == 1
    st = _state(model)
    assert st['lostcount'] == 0
    assert st['vx'] == pytest.approx(0.05, abs=1e-6)


def test_candidate_session_frame_is_not_a_baseline_command(model):
    """候选会话帧（frame[1]=0x7F）在基线上不是速度命令，也不被回确认。"""
    from c50c_uart3.chassis_cmd.session import encode_session_ack
    assert model.feed_bytes(encode_baseline_zero_frame()) == 1
    before = _state(model)['counts']
    assert model.feed_bytes(encode_session_ack(0x12345678)) == 1
    st = _state(model)
    # 帧格式合法（帧头/帧尾/BCC 都对），所以命令丢失计数被清零；
    # 但 buffer[1]==0x7F 不是速度命令，三轴速度没有被改写。
    assert st['counts'] == before + 1
    assert st['vx'] == 0.0
    assert st['mode'] == 1          # 仍是 _ROS_Control，没有被会话帧改变


def test_candidate_bridge_writes_nothing_on_baseline_by_default():
    """候选桥（默认 v3 方言）在基线上永远等不到会话证据：一个字节都不写。"""
    port = FakeFdPort()
    bridge = StraightDriveBridge(port, device='/dev/pts/9', enabled=True,
                                 clock=Clock())
    for _ in range(50):
        bridge.cycle()
    assert bytes(port.stream) == b''
    assert bridge.ticks == 50


def test_baseline_dialect_zero_frame_is_accepted_and_zeroes(model):
    frame = encode_baseline_zero_frame()
    assert frame[2] == 0            # 基线没有 ack 位
    assert model.feed_bytes(frame) == 1
    st = _state(model)
    assert (st['vx'], st['vy'], st['vz']) == (0.0, 0.0, 0.0)
    assert st['lostcount'] == 0


def test_baseline_dialect_straight_command_reaches_firmware(model):
    assert model.feed_bytes(encode_baseline_zero_frame()) == 1
    cmd = encode_baseline_speed_frame(0.05, 0.0)
    assert model.feed_bytes(cmd) == 1
    st = _state(model)
    assert st['vx'] == pytest.approx(0.05, abs=1e-6)
    assert st['vy'] == 0.0 and st['vz'] == 0.0


def test_baseline_stop_frame_zeroes_moving_targets(model):
    assert model.feed_bytes(encode_baseline_speed_frame(0.08, 0.0)) == 1
    assert _state(model)['vx'] == pytest.approx(0.08, abs=1e-6)
    assert model.feed_bytes(encode_baseline_zero_frame()) == 1
    st = _state(model)
    assert (st['vx'], st['vy'], st['vz']) == (0.0, 0.0, 0.0)


def test_baseline_command_loss_guard_zeroes_after_about_one_second(model):
    assert model.feed_bytes(encode_baseline_speed_frame(0.08, 0.0)) == 1
    assert _state(model)['vx'] == pytest.approx(0.08, abs=1e-6)
    model.ticks(100)                 # BALANCE_TASK_RATE 拍，未超限
    assert _state(model)['vx'] == pytest.approx(0.08, abs=1e-6)
    model.ticks(2)                   # 超限：固件自己清零三轴
    st = _state(model)
    assert st['lostcount'] > 100
    assert (st['vx'], st['vy'], st['vz']) == (0.0, 0.0, 0.0)


def test_arbiter_baseline_requires_status_evidence_for_nonzero():
    """基线方言：没有状态帧证据时非零被拒（NO_SESSION），零速仍可发。"""
    clock = Clock()
    arb = Arbiter(firmware_protocol=PROTOCOL_BASELINE,
                  safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arb, clock())
    arb.mark_receiver_clean(simulated=True)
    arb.on_nav_cmd(0.05, 0.0, 0.0, clock())
    d = arb.evaluate(clock())
    assert d.is_zero and d.zero_reason is ZeroReason.NO_SESSION
    assert d.frame == encode_baseline_zero_frame()


def test_arbiter_v3_dialect_still_requires_session_frame():
    """默认方言未变：v3 在没有会话时仍是 NO_SESSION。"""
    clock = Clock()
    arb = Arbiter(safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arb, clock())
    assert arb.firmware_protocol == PROTOCOL_V3
    arb.mark_receiver_clean(simulated=True)
    arb.on_nav_cmd(0.05, 0.0, 0.0, clock())
    d = arb.evaluate(clock())
    assert d.zero_reason is ZeroReason.NO_SESSION


def test_arbiter_baseline_arms_only_after_successful_zero_write():
    clock = Clock()
    arb = Arbiter(firmware_protocol=PROTOCOL_BASELINE,
                  safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arb, clock())
    arb.mark_receiver_clean(simulated=True)
    arb.on_baseline_status_frame(clock())
    arb.on_nav_cmd(0.05, 0.0, 0.0, clock())
    first = arb.evaluate(clock())
    assert first.is_zero and first.zero_reason is ZeroReason.BRIDGE_REARMING
    arb.on_baseline_arm(clock())
    second = arb.evaluate(clock())
    assert not second.is_zero
    assert second.frame == encode_baseline_speed_frame(0.05, 0.0)


def test_arbiter_baseline_status_window_expires():
    clock = Clock()
    arb = Arbiter(firmware_protocol=PROTOCOL_BASELINE,
                  safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arb, clock())
    arb.mark_receiver_clean(simulated=True)
    arb.on_baseline_status_frame(clock())
    arb.on_baseline_arm(clock())
    clock.advance(3.5)
    arb.on_nav_cmd(0.05, 0.0, 0.0, clock())
    d = arb.evaluate(clock())
    assert d.is_zero and d.zero_reason is ZeroReason.SESSION_STALE


def test_baseline_rejects_nonzero_without_status_evidence_after_reconnect(model):
    """重连问题复现：无接收方向证据时不写非零（且不误以为在走）。"""
    clock = Clock()
    bridge, port = _baseline_bridge(clock)
    bridge.notify_status_frame()
    bridge.chain.submit_nav(0.05, 0.0, 0.0, clock())   # 首帧零速
    clock.advance(0.05)
    for _ in range(8):
        bridge.chain.submit_nav(0.05, 0.0, 0.0, clock())
        bridge.cycle()
        clock.advance(0.05)
    moved = [f for f in _frames(port) if f[3] or f[4]]
    assert moved, '应当已放行直行帧'
    assert model.feed_bytes(moved[0]) == 1

    # 断线：接收方向证据消失（状态帧窗口过期）
    bridge.on_serial_reconnected()
    port.stream.clear()
    clock.advance(4.0)
    bridge.chain.submit_nav(0.05, 0.0, 0.0, clock())
    for _ in range(10):
        bridge.cycle()
        clock.advance(0.05)
    assert bytes(port.stream) == b'', '重连后无状态帧证据时不得写出任何字节'


def test_baseline_bridge_sends_zero_before_nonzero_after_reconnect(model):
    """有状态帧证据时，重连后必须零速先行，然后才放行非零，且都被接受。"""
    clock = Clock()
    bridge, port = _baseline_bridge(clock)
    bridge.on_serial_reconnected()
    bridge.notify_status_frame()
    port.stream.clear()
    bridge.chain.submit_nav(0.05, 0.0, 0.0, clock())
    for _ in range(4):
        bridge.chain.submit_nav(0.05, 0.0, 0.0, clock())
        bridge.cycle()
        clock.advance(0.02)
    frames = _frames(port)
    assert frames, '重连后应至少写出首帧'
    assert frames[0] == encode_baseline_zero_frame(), '首帧必须是零速'
    accepted = 0
    for f in frames:
        accepted += model.feed_bytes(f)
    st = _state(model)
    assert accepted >= 1, '重连后的帧必须被固件接受'
    assert st['vx'] == pytest.approx(0.05, abs=1e-6)


def test_baseline_residue_swallows_first_frame_after_reconnect(model):
    """残帧会吞掉一帧：写成功不等于被接受（基线版本的已知边界）。"""
    model.feed_bytes(b'\x7b\x00')                     # 残留半帧
    assert model.feed_bytes(encode_baseline_speed_frame(0.05, 0.0)) == 0, \
        '残帧应吞掉紧随的一帧'
    assert model.feed_bytes(encode_baseline_zero_frame()) == 1, \
        '下一帧应重新对齐'


def test_status_frame_parser_matches_baseline_layout():
    """状态帧解析器（48 字节）能解出固件真实发送的字段。"""
    body = bytearray(48)
    body[0] = 0x7B
    body[1] = 0                                  # FlagStop
    body[2:4] = (100).to_bytes(2, 'big')         # vx mm/s
    body[20:22] = (12400).to_bytes(2, 'big')     # 电压 mV
    body[22:26] = (500).to_bytes(4, 'big', signed=True)      # 编码器 A
    body[26:30] = (500).to_bytes(4, 'big', signed=True)      # 编码器 B
    body[30:34] = (500).to_bytes(4, 'big', signed=True)      # 编码器 C
    body[34:38] = (-500).to_bytes(4, 'big', signed=True)     # 编码器 D
    body[38:40] = (500).to_bytes(2, 'big')                   # CCR1
    body[40:42] = (500).to_bytes(2, 'big')                   # CCR2
    body[42:44] = (500).to_bytes(2, 'big')                   # CCR3
    body[44:46] = (500).to_bytes(2, 'big')                   # CCR4
    bcc = 0
    for b in body[:46]:
        bcc ^= b
    body[46] = bcc
    body[47] = 0x7D
    parsed = parse_status_frame(bytes(body))
    assert parsed == {'stop_flag': 0, 'velocity_mm_s': (100, 0),
                      'yaw_rate_mrad_s': 0, 'voltage_mv': 12400,
                      'encoder_counts': (500, 500, 500, -500),
                      'pwm_counts': (500, 500, 500, 500)}
    body[46] ^= 0xFF
    assert parse_status_frame(bytes(body)) is None


def test_status_frame_rejects_legacy_24_byte_frame():
    """旧 24 字节帧必须被拒绝：改长度后它不再是一个合法帧。

    这条测试防止"固件已扩帧、主机仍按旧长度解析"的错配悄悄通过。
    """
    legacy = bytearray(24)
    legacy[0] = 0x7B
    bcc = 0
    for b in legacy[:22]:
        bcc ^= b
    legacy[22] = bcc
    legacy[23] = 0x7D
    assert parse_status_frame(bytes(legacy)) is None


def test_baseline_bridge_ros_input_with_simulated_clean_endpoint():
    """Production bridge with a simulated clean endpoint forwards ROS input."""
    from types import SimpleNamespace
    clock = Clock()
    port = FakeFdPort()
    bridge = StraightDriveBridge(port, device='/dev/pts/9', enabled=True,
                                 firmware_protocol=PROTOCOL_BASELINE, clock=clock)
    mark_endpoint_clean(port, receipt=_SIM)  # simulated clean endpoint only
    bridge.input.set_nav_publishers(1)
    bridge.input.set_demo_publishers(0)
    grant_safety_via_input(bridge, clock())
    bridge.notify_status_frame()
    _, first = bridge.cycle()
    assert first and first.write_ok and first.is_zero
    msg = SimpleNamespace(linear=SimpleNamespace(x=0.05, y=0.0, z=0.0),
                          angular=SimpleNamespace(x=0.0, y=0.0, z=0.0))
    assert bridge.input.on_nav_cmd(msg)
    clock.advance(0.02)
    _, second = bridge.cycle()
    assert second and second.write_ok and not second.is_zero
    assert second.frame == encode_baseline_speed_frame(0.05, 0.0)
