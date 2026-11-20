"""与固件参考解码器 c50c_uart3_monitor.py 的字段级一致性交叉测试。

参考解码器位于 c50c-firmware-ref/tools/c50c_uart3_monitor.py（SHA-256
与审计报告第 10 节记录一致），其解析语义已通过 task-results 静态交叉校验。
本测试在随机字节流（合法帧 + 垃圾 + 随机分段）上断言两个解析器给出的
帧数量与字段值完全一致，确保新解析器没有偏离已验证的协议定义。

注意：c50c-firmware-ref/ 不在 Python 包路径上，通过 importlib 按文件加载。

task-15 把生产状态帧从 24 字节扩到 48 字节，而 c50c-firmware-ref/ 是 11-05
基线快照（只读、哈希在案、解码器写死 FRAME_LEN = 24），不随新协议更新。
本测试因此只比对新旧协议共有的前 22 字节字段（stop/速度/加速度/角速度）；
编码器与 PWM 字段由 test_frame_parser.py 与 test_baseline_contract.py 覆盖。
旧协议帧在流中的位置分布与新帧不同，所以**不比对帧数量**。
"""
import importlib.util
import random
from pathlib import Path

import pytest

from c50c_uart3.frame_parser import StatusFrameParser
from test_frame_parser import build_frame

REF_PATH = (Path(__file__).resolve().parents[2] / 'c50c-firmware-ref'
            / 'tools' / 'c50c_uart3_monitor.py')


def load_reference_parser_class():
    if not REF_PATH.exists():
        pytest.skip('c50c-firmware-ref 参考解码器不存在')
    spec = importlib.util.spec_from_file_location('ref_monitor', REF_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.StatusFrameParser


def test_fields_match_reference_decoder_on_mixed_stream():
    RefParser = load_reference_parser_class()

    rng = random.Random(20261107)
    stream = bytearray()
    expected = []
    for _ in range(20):
        stream += bytes(rng.randrange(256) for _ in range(rng.randrange(0, 12)))
        record = dict(
            stop=rng.choice([0, 1]),
            vx=rng.randrange(-3000, 3000), vy=rng.randrange(-2000, 2000),
            wz=rng.randrange(-5000, 5000),
            ax=rng.randrange(-20000, 20000), ay=rng.randrange(-20000, 20000),
            az=16384,
            gx=rng.randrange(-1000, 1000), gy=rng.randrange(-1000, 1000),
            gz=rng.randrange(-1000, 1000),
        )
        record['voltage_mv'] = rng.randrange(20000, 26000)
        stream += build_frame(stop=record['stop'], vx=record['vx'],
                              vy=record['vy'], wz=record['wz'], ax=record['ax'],
                              ay=record['ay'], az=record['az'], gx=record['gx'],
                              gy=record['gy'], gz=record['gz'],
                              voltage_mv=record['voltage_mv'])
        expected.append(record)

    # 随机分段切块，喂给两个解析器并各自收集输出
    mine, ref = StatusFrameParser(), RefParser()
    mine_out, ref_out = [], []
    pos = 0
    while pos < len(stream):
        size = rng.randrange(1, 40)
        chunk = bytes(stream[pos:pos + size])
        pos += size
        mine_out += mine.feed(chunk)
        ref_out += ref.feed(chunk)

    # 新解析器必须完整解出每一个构造的帧，且前 22 字节字段与构造值逐项相同
    assert len(mine_out) == len(expected)
    for m, e in zip(mine_out, expected):
        assert m.stop_flag == e['stop']
        assert m.chassis_velocity_mm_s == (e['vx'], e['vy'])
        assert m.chassis_yaw_rate_mrad_s == e['wz']
        assert m.accel_raw_lsb == (e['ax'], e['ay'], e['az'])
        assert m.gyro_raw_lsb == (e['gx'], e['gy'], e['gz'])

    # 公共字段（前 22 字节）的解析语义仍要与参考解码器一致。参考解码器写死
    # 24 字节窗口，喂 48 字节帧它一帧也解不出（新旧协议帧长不同），所以这里
    # 用同一批字段值再建一条**旧格式 24 字节流**单独喂它，逐项比对共享字段。
    legacy = bytearray()
    for e in expected:
        w = bytearray()
        w.append(0x7B)
        w.append(e['stop'])
        w += int(e['vx']).to_bytes(2, 'big', signed=True)
        w += int(e['vy']).to_bytes(2, 'big', signed=True)
        w += int(e['wz']).to_bytes(2, 'big', signed=True)
        w += int(e['ax']).to_bytes(2, 'big', signed=True)
        w += int(e['ay']).to_bytes(2, 'big', signed=True)
        w += int(e['az']).to_bytes(2, 'big', signed=True)
        w += int(e['gx']).to_bytes(2, 'big', signed=True)
        w += int(e['gy']).to_bytes(2, 'big', signed=True)
        w += int(e['gz']).to_bytes(2, 'big', signed=True)
        w += int(e['voltage_mv']).to_bytes(2, 'big')
        c = 0
        for b in w:
            c ^= b
        w.append(c)
        w.append(0x7D)
        assert len(w) == 24
        legacy += w
    legacy_out = RefParser().feed(bytes(legacy))
    assert len(legacy_out) == len(expected)
    for m, r, e in zip(mine_out, legacy_out, expected):
        assert m.stop_flag == r['stop'] == e['stop']
        assert m.chassis_velocity_mm_s == tuple(r['velocity_mm_s']) == (e['vx'], e['vy'])
        assert m.chassis_yaw_rate_mrad_s == r['yaw_mrad_s'] == e['wz']
        assert m.accel_raw_lsb == tuple(r['accel_raw']) == (e['ax'], e['ay'], e['az'])
        assert m.gyro_raw_lsb == tuple(r['gyro_raw']) == (e['gx'], e['gy'], e['gz'])
        assert m.voltage_mv == r['voltage_mv'] == e['voltage_mv']

    # 混合流上参考解码器解出的帧数不再是判据（协议已变），只断言它不崩、不误报
    assert ref_out == []


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__]))
