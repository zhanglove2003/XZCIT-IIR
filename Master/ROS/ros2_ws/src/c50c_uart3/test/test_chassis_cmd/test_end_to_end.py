"""端到端：仲裁 + 编码 + 内存假串口 + 真实固件解析器。

这些测试把桥的**实际写出字节**喂给固件模型，因此验证的是"桥这一拍会
让固件接受/拒绝"，而不是桥自己的说法。
"""
import pytest

from c50c_uart3.chassis_cmd import encode_speed_frame
from sim_confirm import mark_endpoint_clean, mark_chain_clean, \
    new_receipt as _new_receipt

_SIM = _new_receipt('test_end_to_end')

ZERO = encode_speed_frame(0.0, 0.0, 0.0)

def zero(seq: int = 0) -> bytes:
    """协议 v3 的零速帧：buffer[2] = 0x80 | 序号。"""
    return encode_speed_frame(0.0, 0.0, 0.0, seq=seq)



CHAIN_CLOCK = [100.0]


@pytest.fixture(autouse=True)
def _reset_clock():
    CHAIN_CLOCK[0] = 100.0
    yield


def _step(chain, firmware, dt=0.02, ticks=2):
    """推进一拍：先让固件跑监督，再把桥这一拍的字节喂给固件。"""
    CHAIN_CLOCK[0] += dt
    firmware.tick(ticks)
    rec = chain.tick(CHAIN_CLOCK[0])
    assert rec.frame is not None
    firmware.feed(rec.frame)
    return rec


def _authorise(chain, firmware, vx=0.05):
    """走到"已获授权且固件已 armed"的稳态：第一拍零速，第二拍才是非零。"""
    chain.set_sources(1, 0)
    # 第五轮：启动默认接收端未知；测试显式构造干净状态才能发送
    mark_chain_clean(chain, receipt=_SIM)
    chain.submit_nav(vx, 0.0, 0.0, CHAIN_CLOCK[0])
    # 桥在启动/复位后的第一拍必须输出零速帧（授权帧 + 心跳）。
    first = chain.tick(CHAIN_CLOCK[0])
    assert first.is_zero and first.frame == ZERO
    firmware.feed(first.frame)
    assert firmware.state.armed is True
    # 之后的拍才输出非零；固件接受并写入目标速度。
    second = _step(chain, firmware)
    assert not second.is_zero
    assert firmware.state.vx == pytest.approx(vx)
    return first, second


def test_bridge_alone_authorises_and_drives(firmware, chain):
    chain.firmware_confirm(True)      # 审核期的人工门（真实桥无调用点）
    _authorise(chain, firmware)
    assert firmware.state.vx == pytest.approx(0.05)
    assert firmware.state.epoch == 0         # 未触发 Safe_Stop


def test_nav_stream_ended_bridge_holds_still_and_firmware_stops(firmware, chain):
    """命令流中断：桥降为零速；即使零速帧也断了，固件 1 s 后自行停车。"""
    chain.firmware_confirm(True)
    mark_chain_clean(chain, receipt=_SIM)  # 第五轮：干净接收端
    _authorise(chain, firmware)

    CHAIN_CLOCK[0] += 0.35                    # 超过 nav_timeout
    rec = chain.tick(CHAIN_CLOCK[0])
    # 序号：授权零速(0) → 非零(1) → 本条超时零速(2)。
    assert rec.is_zero and rec.frame == zero(2)
    firmware.feed(rec.frame)
    assert firmware.state.vx == 0.0
    assert firmware.state.armed is True       # 零速帧保持授权

    # 桥完全停发（例如进程被杀），固件自行超时停车
    firmware.tick(100)
    assert firmware.state.armed is False
    assert firmware.state.epoch == 1


def test_stop_request_drives_firmware_to_still(firmware, chain):
    chain.firmware_confirm(True)
    mark_chain_clean(chain, receipt=_SIM)  # 第五轮：干净接收端
    _authorise(chain, firmware)
    chain.set_stop(True)
    rec = _step(chain, firmware)
    assert rec.is_zero
    assert firmware.state.vx == 0.0
    for _ in range(5):
        assert _step(chain, firmware).is_zero
    assert chain.counts['dropped_stop'] >= 1
    assert chain.nonzero_frames() and all(f != ZERO for f in chain.nonzero_frames())


def test_short_stop_release_does_not_drive_firmware_again(firmware, chain):
    """短停车解除后，固件目标速度保持零，直到桥收到新命令。

    返工任务书 B 的端到端对照：桥侧不得恢复旧命令，固件侧因此也看不到
    任何非零帧。
    """
    chain.firmware_confirm(True)
    mark_chain_clean(chain, receipt=_SIM)  # 第五轮：干净接收端
    _authorise(chain, firmware)
    chain.set_sources(1, 0)
    chain.submit_nav(0.05, 0.0, 0.0, CHAIN_CLOCK[0])
    _step(chain, firmware)
    _step(chain, firmware)
    assert firmware.state.vx == pytest.approx(0.05)

    before = len(chain.nonzero_frames())
    chain.set_stop(True)
    assert _step(chain, firmware).is_zero
    CHAIN_CLOCK[0] += 0.02                    # 远小于 nav_timeout_s
    chain.set_stop(False)
    for _ in range(5):
        rec = _step(chain, firmware)
        assert rec.is_zero
        assert firmware.state.vx == 0.0
    assert len(chain.nonzero_frames()) == before, '解除停车后不得再输出非零'


def test_reconnect_blocks_nonzero_forever_without_confirmation(firmware, chain):
    """重连后桥只发零速，固件保持非行驶状态。"""
    chain.firmware_confirm(True)
    mark_chain_clean(chain, receipt=_SIM)  # 第五轮：干净接收端
    _authorise(chain, firmware)

    chain.on_transport_reconnect()
    for _ in range(30):
        chain.submit_nav(0.05, 0.0, 0.0, CHAIN_CLOCK[0])
        rec = _step(chain, firmware)
        assert rec.is_zero
        assert firmware.state.vx == 0.0
    assert chain.counts['reconnect_events'] == 1
    assert firmware.state.armed is True       # 零速帧仍保持授权（心跳）


def test_write_failure_stops_output_and_needs_operator(firmware, chain, transport):
    chain.firmware_confirm(True)
    mark_chain_clean(chain, receipt=_SIM)  # 第五轮：干净接收端
    _authorise(chain, firmware)
    transport.fail_next = 1
    rec = _step(chain, firmware)              # 这一拍的写入失败
    assert rec.write_ok is False
    # 第五轮：写失败（字节数不明）后锁死字节流，此后连零速帧也不再写出
    for _ in range(5):
        CHAIN_CLOCK[0] += 0.02
        rec = chain.tick(CHAIN_CLOCK[0])
        assert rec.is_zero
        assert rec.frame is None, '锁流后不得再产生帧'
        assert rec.zero_reason.name == 'STREAM_POISONED'
    # 操作者清错只清写失败锁存，**不清流毒**；流锁需重建端口对象才解除。
    chain.operator_clear()
    chain.submit_nav(0.05, 0.0, 0.0, CHAIN_CLOCK[0])
    for _ in range(5):
        CHAIN_CLOCK[0] += 0.02
        rec = chain.tick(CHAIN_CLOCK[0])
        assert rec.is_zero
        assert rec.frame is None, '清错不得解除流锁'
    # 锁流后不再发送任何帧，固件目标速度停在失败前的值。
    # 这不能证明车已停下——接收链未被证明安全，见报告 §6。
    assert firmware.state.vx == 0.05, '固件保持最后一次被接受的目标速度' 


def test_wire_bytes_equal_bridge_bytes(firmware, chain, transport):
    """固件收到的字节就是桥写出的字节（假串口不丢不改）。"""
    chain.firmware_confirm(True)
    mark_chain_clean(chain, receipt=_SIM)  # 第五轮：干净接收端
    _authorise(chain, firmware)
    assert transport.writes == [r.frame for r in chain.records if r.write_ok]
