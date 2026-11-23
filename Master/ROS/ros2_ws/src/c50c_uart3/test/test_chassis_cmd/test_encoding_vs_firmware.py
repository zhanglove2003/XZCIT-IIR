"""命令编码：字节与真实固件解析器的双向核对。

断言两类事实：

1. 桥编码的字节符合同一协议（帧头/命令字/大端毫单位/BCC/帧尾）；
2. 真实固件解析器**接受**这些字节，并解出与量化一致的数值。
"""
import pytest

from c50c_uart3.chassis_cmd import (FRAME_HEADER, FRAME_TAIL, check_bcc,
                                    encode_speed_frame)


def test_zero_frame_bytes():
    """协议 v3：buffer[2] = 0x80 | 序号。首帧序号 0 → 0x80。"""
    frame = encode_speed_frame(0.0, 0.0, 0.0)
    assert frame == bytes.fromhex('7b0080000000000000fb7d')
    assert frame[2] == 0x80


def test_sequence_number_occupies_low_seven_bits():
    """buffer[2] = 能力位 0x80 | 序号(0..127)。"""
    assert encode_speed_frame(0.0, 0.0, 0.0, seq=1)[2] == 0x81
    assert encode_speed_frame(0.0, 0.0, 0.0, seq=127)[2] == 0xFF


@pytest.mark.parametrize('bad', [-1, 128, 1.0, True, '3', None])
def test_sequence_number_out_of_range_is_refused(bad):
    with pytest.raises(ValueError):
        encode_speed_frame(0.0, 0.0, 0.0, seq=bad)


def test_frame_layout_and_bcc():
    frame = encode_speed_frame(0.10, 0.0, 0.0)
    assert len(frame) == 11
    assert frame[0] == FRAME_HEADER and frame[10] == FRAME_TAIL
    assert frame[1] == 0x00
    assert frame[9] == check_bcc(frame[:9])
    # 0.10 m/s = 100 毫单位 = 0x0064，大端在第 3、4 字节
    assert frame[3:5] == bytes([0x00, 0x64])


def test_negative_axis_uses_twos_complement():
    frame = encode_speed_frame(-0.05, 0.0, 0.0)
    # -50 毫单位 = 0xFFCE
    assert frame[3:5] == bytes([0xFF, 0xCE])


@pytest.mark.parametrize('vx,vz', [(0.10, 0.0), (0.05, 0.0), (-0.05, 0.0),
                                   (0.0, 0.30), (0.0, -0.30)])
def test_firmware_accepts_encoded_frames(firmware, vx, vz):
    """固件先接受零速授权帧，再接受非零帧，并解出量化值。"""
    firmware.feed(encode_speed_frame(0.0, 0.0, 0.0))
    assert firmware.state.armed is True
    assert firmware.state.age == 0

    firmware.feed(encode_speed_frame(vx, vz, 0.0))
    st = firmware.state
    assert st.armed is True
    assert st.age == 0            # 帧被接受，监督计时被清零
    assert st.vx == pytest.approx(vx, abs=1e-6)
    assert st.vz == pytest.approx(vz, abs=1e-6)


def test_firmware_rejects_corrupted_bcc(firmware):
    firmware.feed(encode_speed_frame(0.0, 0.0, 0.0))
    bad = bytearray(encode_speed_frame(0.05, 0.0, 0.0))
    bad[9] ^= 0xFF
    firmware.feed(bytes(bad))
    # 校验失败：不刷新年龄，也不写入目标速度
    firmware.tick(5)
    assert firmware.state.age == 5
    assert firmware.state.vx == 0.0


def test_nonzero_frame_before_zero_is_rejected(firmware):
    """未授权时非零帧不被接受（固件要求先用零速帧授权）。"""
    firmware.feed(encode_speed_frame(0.05, 0.0, 0.0))
    assert firmware.state.armed is False
    assert firmware.state.vx == 0.0
