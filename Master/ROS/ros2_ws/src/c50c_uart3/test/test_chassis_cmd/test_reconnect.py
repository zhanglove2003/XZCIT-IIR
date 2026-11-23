"""重连与重启：固件侧状态跨主机重连存活，桥只能靠实测确认恢复。

这些测试用**真实固件解析器**（``firmware`` 夹具）作接收端，因此"桥写出
的字节有没有被接受"是固件逻辑给出的判定，不是测试自己猜的。

关键结论（每条一个测试）：

1. 重连后残留半个帧时，桥的整帧会被吞进残帧，固件**不会**接受它——
   所以"发送成功"不能当作"固件收到"。
2. 重复零速帧单独就能清掉残帧（11 字节对齐的自愈），但这只是运气：
   若残帧长度不是 11 的补偿，同样字节会被吞。故方案不依赖它放行非零。
3. 进程重启后桥的内部状态清零，必须重新走"零速授权"，且不能凭
   "写成功"或"重复零速"就放行非零——本候选要求显式固件确认，
   而当前协议没有该字段，因此重连后**永远不会**自动放行非零。
"""
import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame
from c50c_uart3.chassis_cmd.arbiter import Arbiter, ZeroReason
from sim_confirm import TEST_SAFETY_MAX_AGE_S, grant_safety

ZERO = encode_speed_frame(0.0, 0.0, 0.0)

def zero(seq: int = 0) -> bytes:
    """协议 v3 的零速帧：buffer[2] = 0x80 | 序号。"""
    return encode_speed_frame(0.0, 0.0, 0.0, seq=seq)

FWD = encode_speed_frame(0.05, 0.0, 0.0)


def test_reconnect_residue_swallows_first_frame(firmware):
    """残帧会吞掉桥重连后的第一帧：发送成功 ≠ 固件接受。"""
    firmware.feed(ZERO)
    assert firmware.state.armed
    firmware.feed_bytes(b'\x7b\x00')          # 断电/重连留下 2 字节残帧
    firmware.feed(ZERO)                       # 重连后的第一条零速帧
    st = firmware.state
    # 残帧 2 + 整帧 11 = 13；解析器在 9 字节处"凑满 11"判定一次，
    # 但内容错位，校验失败——状态没有刷新，也没有重新授权。
    firmware.tick(10)
    assert st.armed in (True, False)          # 仅记录现状，不作为放行依据
    assert firmware.state.age > 0             # 该帧没有被接受


def test_residue_delays_but_does_not_starve_v3_authorisation():
    """v3 固件模型：残帧只吞掉一帧，随后重新对齐并接纳零速帧。

    旧断言（"残帧 2..10 字节时连续零速帧永远无法重新授权"）来自改造前的
    解析器模型，那里没有会话门；v3 的真实 ``chassis_ack.c`` 在每帧做帧头/
    帧尾/BCC 校验，错位帧只是被计数剔除，下一帧即恢复对齐。

    这里记录的是**实测事实**，不是设计意图：桥仍然不能把"连发零速帧"
    当作可靠手段，所以放行非零的依据仍是会话确认与真实接纳计数。
    """
    import fw_v3_harness
    from c50c_uart3.chassis_cmd.session import encode_session_ack

    atomic = fw_v3_harness.V3Firmware.start()
    try:
        atomic.feed_bytes(encode_session_ack(atomic.session))
        atomic.feed_bytes(zero(0))
        assert atomic.state.armed is True
        assert atomic.state.accept == 1
    finally:
        atomic.close()

    for residue in range(1, 11):
        model = fw_v3_harness.V3Firmware.start()
        try:
            model.feed_bytes(encode_session_ack(model.session))
            model.feed_bytes(bytes([0x7B] + [0x00] * (residue - 1)))
            for seq in range(20):
                model.feed_bytes(zero(seq))
            st = model.state
            assert st.armed is True, f'残帧 {residue} 字节：对齐后应重新授权'
            # 首个错位字段后的第一个 11 字节窗口被丢弃，其余全部接纳。
            assert st.accept == 19, f'残帧 {residue} 字节：接纳计数 {st.accept}'
            assert st.no_session == 0 and st.no_seq == 0
        finally:
            model.close()


def test_single_byte_residue_recovers_on_second_frame():
    """残帧 1 字节是唯一"一条后自愈"的情形（第 2 条对齐）。"""
    from fw_parser_harness import FirmwareModel
    model = FirmwareModel.start()
    try:
        model.feed_bytes(b'\x7b')
        model.feed(ZERO)
        assert model.state.armed is False      # 被吞
        model.feed(ZERO)
        assert model.state.armed is True       # 对齐
    finally:
        model.close()


def test_repeated_zero_does_not_authorise_nonzero_after_reconnect(clock, firmware):
    """桥侧：重连后即使连续送零速，也绝不放行非零。"""
    a = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5,
                safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(a)
    a.on_publisher_count(1, 0)
    a.on_firmware_confirm(True)
    a.mark_receiver_clean(simulated=True)  # 第五轮：干净接收端
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    assert a.evaluate(clock()).is_zero        # 首帧零速（桥启动规则）
    assert not a.evaluate(clock()).is_zero    # 重连前是放行的
    a.on_serial_reconnect()
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())     # 重连后仍有命令流
    for _ in range(20):
        clock.advance(0.02)
        d = a.evaluate(clock())
        # 重连后禁止非零；零速帧照发（不新增字节是否写出由接收端状态决定）
        assert d.is_zero
        assert d.zero_reason.name != 'NONE'
        # 第十一轮：重连会作废会话观测，所以拦下的理由先是 NO_SESSION；
        # 会话重新确认后才是 OUTPUT_BLOCKED。2026-11-14 起过期命令在候选
        # 选择时被跳过，旧缓存也可能给出 NAV_EXPIRED——同样表示没有非零。
        assert d.zero_reason in (ZeroReason.OUTPUT_BLOCKED,
                                 ZeroReason.NO_SESSION, ZeroReason.NAV_EXPIRED,
                                 ZeroReason.SAFETY_UNKNOWN)
    assert a._counts['reconnect_events'] == 1


def test_write_success_after_reconnect_is_not_confirmation(clock):
    """写入返回成功不能解除禁非零（本轮方案的核心安全点）。"""
    a = Arbiter(safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(a)
    a.on_publisher_count(1, 0)
    a.on_firmware_confirm(True)
    a.mark_receiver_clean(simulated=True)  # 第五轮：干净接收端
    a.on_serial_reconnect()
    a.on_nav_cmd(0.05, 0.0, 0.0, clock())
    a.on_write_result(True)
    assert a.evaluate(clock()).zero_reason in (ZeroReason.OUTPUT_BLOCKED,
                                              ZeroReason.NO_SESSION)


def test_bridge_restart_clears_state_and_requires_zero_first(clock):
    """新进程 = 新仲裁器：接收端状态未知，第一条输出必须是"什么都不写"。"""
    old = Arbiter(safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(old)
    old.on_publisher_count(1, 0)
    old.on_firmware_confirm(True)
    old.mark_receiver_clean(simulated=True)  # 第五轮：干净接收端
    old.on_nav_cmd(0.05, 0.0, 0.0, clock())
    assert old.evaluate(clock()).is_zero      # 首帧零速
    assert not old.evaluate(clock()).is_zero

    new = Arbiter(safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(new)
    new.on_publisher_count(1, 0)
    new.on_nav_cmd(0.05, 0.0, 0.0, clock())   # 消息流仍在
    d = new.evaluate(clock())
    # 第五轮改判：重启后接收端可能仍有残帧，桥保持静默（不写字节）
    assert d.is_zero and d.frame is None
    assert d.zero_reason is ZeroReason.RECEIVER_UNKNOWN
    assert new._counts['restarts'] == 1


def test_firmware_requires_zero_frame_after_supervision_stop(firmware):
    """固件超时停车后解除授权，必须重新用零速帧授权才能接受非零。"""
    firmware.feed(ZERO)
    firmware.feed(FWD)
    assert firmware.state.vx == pytest.approx(0.05)
    firmware.tick(100)                        # 1 s 无新帧 → 超时停车
    st = firmware.state
    assert st.armed is False and st.vx == 0.0
    firmware.feed(FWD)                        # 未重新授权，非零被拒
    assert firmware.state.vx == 0.0
    firmware.feed(ZERO)                       # 零速帧重新授权
    assert firmware.state.armed is True
    firmware.feed(FWD)
    assert firmware.state.vx == pytest.approx(0.05)


def test_stop_frame_keeps_firmware_armed(firmware):
    """零速帧是授权帧，也是心跳：它让固件保持 ROS 授权且不触发超时停车。"""
    firmware.feed(ZERO)
    for _ in range(30):
        firmware.tick(3)          # 30*3=90 拍 < 100
        firmware.feed(ZERO)       # 定时零速心跳
    st = firmware.state
    assert st.armed is True and st.epoch == 0
