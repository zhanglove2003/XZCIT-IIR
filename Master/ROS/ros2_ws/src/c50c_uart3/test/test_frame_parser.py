"""C50C 50 字节状态帧解析器定向测试（离线，不依赖 ROS、不接触串口）。

帧构造按 c50c-firmware-ref/BALANCE/data_task.c ``data_transition()`` 的
buffer 赋值顺序与 data_task.h 常量反向生成，作为已知输入。
（2026-11-15：48→50 字节，[46..47] 追加前置 HC-SR04 超声波 mm。） 
"""
import pytest

from c50c_uart3.frame_parser import (
    FRAME_HEADER,
    FRAME_LEN,
    FRAME_TAIL,
    StatusFrame,
    StatusFrameParser,
)


def bcc(payload: bytes) -> int:
    checksum = 0
    for b in payload:
        checksum ^= b
    return checksum


def build_frame(stop=0, vx=0, vy=0, wz=0, ax=0, ay=0, az=0,
                gx=0, gy=0, gz=0, voltage_mv=0,
                encoder=(0, 0, 0, 0), pwm=(0, 0, 0, 0),
                ultrasonic_mm=0) -> bytes:
    def i16(v: int) -> bytes:
        return int(v).to_bytes(2, 'big', signed=True)

    def i32(v: int) -> bytes:
        return int(v).to_bytes(4, 'big', signed=True)

    def u16(v: int) -> bytes:
        return int(v).to_bytes(2, 'big', signed=False)

    assert len(encoder) == 4 and len(pwm) == 4
    body = bytearray()
    body.append(FRAME_HEADER)
    body.append(stop & 0xFF)
    body += i16(vx) + i16(vy) + i16(wz)
    body += i16(ax) + i16(ay) + i16(az)
    body += i16(gx) + i16(gy) + i16(gz)
    body += int(voltage_mv).to_bytes(2, 'big')
    assert len(body) == 22, len(body)
    for v in encoder:
        body += i32(v)          # [22..37] A/B/C/D
    for v in pwm:
        body += u16(v)          # [38..45] CCR1..CCR4
    body += u16(ultrasonic_mm)  # [46..47] 前置 HC-SR04 距离 mm
    assert len(body) == FRAME_LEN - 2, len(body)
    body.append(bcc(bytes(body)))   # [48]
    body.append(FRAME_TAIL)         # [49]
    assert len(body) == FRAME_LEN
    return bytes(body)


def test_known_frame_field_values_and_signs():
    frame = build_frame(stop=1, vx=120, vy=-45, wz=-3000,
                        ax=16384, ay=-8192, az=16384,
                        gx=100, gy=-50, gz=1234, voltage_mv=24000)
    parser = StatusFrameParser()
    (out,) = parser.feed(frame)
    assert isinstance(out, StatusFrame)
    assert out.stop_flag == 1
    assert out.chassis_velocity_mm_s == (120, -45)
    assert out.chassis_yaw_rate_mrad_s == -3000
    assert out.accel_raw_lsb == (16384, -8192, 16384)
    assert out.gyro_raw_lsb == (100, -50, 1234)
    assert out.voltage_mv == 24000
    assert out.frame_bytes == frame
    assert parser.frames_accepted == 1


def test_int16_extremes_big_endian():
    frame = build_frame(vx=-32768, vy=32767, wz=-1,
                        ax=-32768, ay=32767, az=-2,
                        gx=32766, gy=-32767, gz=32765,
                        voltage_mv=65535)
    (out,) = StatusFrameParser().feed(frame)
    assert out.chassis_velocity_mm_s == (-32768, 32767)
    assert out.chassis_yaw_rate_mrad_s == -1
    assert out.accel_raw_lsb == (-32768, 32767, -2)
    assert out.gyro_raw_lsb == (32766, -32767, 32765)
    assert out.voltage_mv == 65535


def test_ultrasonic_field_round_trip():
    # 2026-11-15 新增 [46..47]：前置 HC-SR04 距离 mm，0xFFFF=无回波
    (out,) = StatusFrameParser().feed(build_frame(ultrasonic_mm=1234))
    assert out.ultrasonic_front_mm == 1234
    (out,) = StatusFrameParser().feed(build_frame(ultrasonic_mm=0xFFFF))
    assert out.ultrasonic_front_mm == 0xFFFF
    (out,) = StatusFrameParser().feed(build_frame(ultrasonic_mm=20))
    assert out.ultrasonic_front_mm == 20
    # 大端序自证：构造 0x0102 → 帧内 [46]=0x01、[47]=0x02
    frame = build_frame(ultrasonic_mm=0x0102)
    assert frame[46] == 0x01 and frame[47] == 0x02
    (out,) = StatusFrameParser().feed(frame)
    assert out.ultrasonic_front_mm == 0x0102


def test_segmented_byte_by_byte_feed():
    frame = build_frame(stop=0, vx=500, az=16384, gz=-999)
    parser = StatusFrameParser()
    collected = []
    for i in range(len(frame)):
        collected.extend(parser.feed(frame[i:i + 1]))
        assert parser.frames_accepted == (1 if i == len(frame) - 1 else 0)
    assert len(collected) == 1
    assert collected[0].gyro_raw_lsb[2] == -999


def test_multiple_consecutive_frames_in_single_chunk():
    f1 = build_frame(stop=0, vx=1)
    f2 = build_frame(stop=1, vx=2)
    f3 = build_frame(stop=0, vx=3)
    (out1, out2, out3) = StatusFrameParser().feed(f1 + f2 + f3)
    assert [f.chassis_velocity_mm_s[0] for f in (out1, out2, out3)] == [1, 2, 3]
    assert [f.stop_flag for f in (out1, out2, out3)] == [0, 1, 0]


def test_leading_garbage_then_resync():
    frame = build_frame(vx=777)
    parser = StatusFrameParser()
    (out,) = parser.feed(b'\x00\x11\x22' + frame)
    assert out.chassis_velocity_mm_s == (777, 0)
    assert parser.frames_accepted == 1


def test_bad_checksum_rejects_and_resyncs():
    good = build_frame(vx=888)
    corrupted = bytearray(good)
    corrupted[10] ^= 0x40  # 破坏加速度 Y 低字节 → BCC 失败
    parser = StatusFrameParser()
    out = parser.feed(bytes(corrupted) + good)
    assert len(out) == 1
    assert out[0].chassis_velocity_mm_s == (888, 0)
    assert parser.candidates_rejected == 1


def test_bad_tail_rejects_and_resyncs():
    good = build_frame(vx=555)
    corrupted = bytearray(good)
    corrupted[FRAME_LEN - 1] = 0x7E  # 帧尾错误
    out = StatusFrameParser().feed(bytes(corrupted) + good)
    assert len(out) == 1
    assert out[0].chassis_velocity_mm_s == (555, 0)


def test_garbage_containing_header_byte_resyncs():
    # 垃圾数据里含 0x7B：候选帧校验失败后必须滑动越过它恢复同步
    good = build_frame(vx=321)
    noise = b'\x7b' + b'\xaa\xbb\xcc\xdd' * 6  # 25 字节，无合法帧尾/BCC
    out = StatusFrameParser().feed(noise + good)
    assert len(out) == 1
    assert out[0].chassis_velocity_mm_s == (321, 0)


def test_corrupted_split_across_feeds():
    # 坏帧与好帧跨多次 feed 到达，重同步逻辑不受分段影响
    good = build_frame(vx=64)
    bad = bytearray(build_frame(vx=0))
    bad[18] ^= 0x01  # 陀螺 Z 字节翻转
    parser = StatusFrameParser()
    out = []
    data = bytes(bad) + good
    out.extend(parser.feed(data[:7]))
    out.extend(parser.feed(data[7:20]))
    out.extend(parser.feed(data[20:]))
    assert [f.chassis_velocity_mm_s[0] for f in out] == [64]
    # 全零坏帧的 BCC 恰为 0x7B（等于帧头字节），被滑过后还会形成一个
    # 假候选再被拒一次：candidates_rejected 为 2 属预期。
    assert parser.candidates_rejected == 2


def test_interleaved_autocharge_frame_is_skipped():
    # 固件 Usart3_SendTask 在配置回充装备时会交错发送 8 字节回充帧
    #（头 0x7C、尾 0x7F，data_task.h AutoCharge_* 常量）。本基线不发送，
    # 但解析器必须能消化这类交错字节并恢复同步。
    good1 = build_frame(vx=11)
    good2 = build_frame(vx=22)
    autocharge = bytes([0x7C, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x7F])
    out = StatusFrameParser().feed(good1 + autocharge + good2)
    assert [f.chassis_velocity_mm_s[0] for f in out] == [11, 22]


def test_incomplete_frame_waits_for_more_data():
    frame = build_frame(vx=9)
    parser = StatusFrameParser()
    assert parser.feed(frame[:-1]) == []
    (out,) = parser.feed(frame[-1:])
    assert out.chassis_velocity_mm_s == (9, 0)


def test_pure_garbage_discarded_without_frames():
    parser = StatusFrameParser()
    assert parser.feed(b'\x01\x02\x03' * 100) == []
    assert parser.frames_accepted == 0


def test_no_timestamp_or_orientation_fabricated():
    # StatusFrame 只承载帧内字段：无姿态、无采样时间戳（固件不提供）。
    frame = build_frame()
    (out,) = StatusFrameParser().feed(frame)
    assert not hasattr(out, 'orientation')
    assert not hasattr(out, 'timestamp')


# ---------- 重连残余处理（任务书：断开前后字节不得误拼成有效帧） ----------

def craft_splice_prefix(new_frame: bytes, old_vx_mm_s=7777, az_high=0x40):
    """构造断流前的残余前缀（10 字节），使"前缀 + new_frame[0:FRAME_LEN-10]"拼成一个
    恰好通过帧尾+BCC 校验的假帧，且假帧 vx 字段取自前缀（旧数据）。

    这不是期望行为，而是**危害确定性反例**：证明解析器若在重连时不丢弃
    残余，断开前后字节可能被误拼成有效状态帧——这正是节点重连必须调用
    ``StatusFrameParser.reset()`` 的原因。

    候选帧布局（缓冲区 = prefix + new_frame，候选取前 FRAME_LEN 字节）：
    候选[FRAME_LEN-1] = new_frame[FRAME_LEN-11]（须为 0x7D → 用 new_frame 当帧尾）；
    候选[FRAME_LEN-2] = new_frame[FRAME_LEN-12]（BCC 字节，自由选择）；
    BCC 条件 XOR(prefix) ^ XOR(new_frame[0:FRAME_LEN-12]) == 候选[FRAME_LEN-2]
    化简为 prefix[9] = 候选[FRAME_LEN-2] ^ XOR(new_frame[0:FRAME_LEN-12])
    ^ 0x7B ^ XOR(prefix[1:9])。
    """
    assert len(new_frame) == FRAME_LEN
    tail_idx = FRAME_LEN - 11  # prefix 10 字节 + new_frame[0:FRAME_LEN-10]；候选帧尾落在 new_frame[tail_idx]
    frame = bytearray(new_frame)
    frame[FRAME_LEN - 12] = az_high    # 候选[FRAME_LEN-2]，BCC 位
    frame[tail_idx] = 0x7D # 候选[FRAME_LEN-1]
    assert frame[tail_idx] == 0x7D

    prefix = bytearray(10)
    prefix[0] = FRAME_HEADER          # 候选从缓冲区第一个 0x7B 开始
    prefix[2:4] = old_vx_mm_s.to_bytes(2, 'big', signed=True)
    # 解 prefix[9]；prefix[1..8] 先固定，若解出 0x7B 则换一个 az_high 重解
    # （保持前缀内无第二个帧头歧义源）
    x = FRAME_HEADER
    for b in prefix[1:9]:
        x ^= b
    for ah in range(az_high, 0x80):
        frame[FRAME_LEN - 12] = ah
        candidate_last = ah ^ bcc(bytes(frame[:FRAME_LEN - 12])) ^ x
        if candidate_last != FRAME_HEADER:
            prefix[9] = candidate_last
            break
    else:
        raise AssertionError('未能构造无歧义前缀')
    frame[FRAME_LEN - 2] = bcc(bytes(frame[:FRAME_LEN - 2]))  # 重算 new_frame 自身 BCC
    frame = bytes(frame)
    # 自检：候选 = prefix + frame[0:FRAME_LEN-10] 必须通过帧尾+BCC，且 vx 来自前缀
    candidate = bytes(prefix) + frame[:FRAME_LEN - 10]
    assert candidate[FRAME_LEN - 1] == FRAME_TAIL
    assert candidate[FRAME_LEN - 2] == bcc(candidate[:FRAME_LEN - 2])
    assert int.from_bytes(candidate[2:4], 'big', signed=True) == old_vx_mm_s
    return bytes(prefix), frame


def test_reset_drops_partial_buffer_and_keeps_counters():
    p = StatusFrameParser()
    half = build_frame(vx=500, az=16384)[:10]
    p.feed(half)
    assert p.frames_accepted == 0
    p.reset()
    (out,) = p.feed(build_frame(vx=100, az=16384))
    assert out.chassis_velocity_mm_s == (100, 0)
    assert p.frames_accepted == 1 and p.candidates_rejected == 0


def test_reset_prevents_cross_reconnect_splice():
    """确定性反例：不 reset 时断开前残余可与重连后数据拼成有效假帧；
    reset 后同样输入只解出新帧——这正是节点重连必须 reset 的依据。"""
    new_frame = build_frame(vx=100, az=0x407D, voltage_mv=4060)
    prefix, new_frame = craft_splice_prefix(new_frame)

    # 危害演示（修复前行为）：残余 + 新数据 → 假帧被接受，vx 是旧数据
    p_bad = StatusFrameParser()
    p_bad.feed(prefix)                      # 断流前残余（半帧）
    frames = p_bad.feed(new_frame)          # 重连后的新数据
    assert any(f.chassis_velocity_mm_s[0] == 7777 for f in frames), \
        '构造的反例应能拼出通过校验的假帧（否则反例失效）'

    # 修复后：reset 丢弃残余 → 只解出新帧
    p_good = StatusFrameParser()
    p_good.feed(prefix)
    p_good.reset()
    (out,) = p_good.feed(new_frame)
    assert out.chassis_velocity_mm_s == (100, 0)
    assert p_good.candidates_rejected == 0


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__]))
