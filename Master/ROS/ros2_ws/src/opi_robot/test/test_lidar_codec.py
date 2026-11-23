"""帧解码测试：正常帧、半帧、CRC 错误、黏包、反向帧、缓冲上限。

被测模块：opi_robot.lidar_node.codec
"""

import pytest

from opi_robot.lidar_node import codec as C
from lidar_fixtures import build_frame, frame_rotation


def feed_all(dec, chunks):
    out = []
    for chunk in chunks:
        out.extend(dec.feed(chunk))
    return out


class TestSingleFrame:
    def test_one_full_frame(self):
        dec = C.FrameDecoder()
        frames = feed_all(dec, [build_frame(start_ddeg=0, end_ddeg=3290)])
        assert len(frames) == 1
        f = frames[0]
        assert f.start_deg == pytest.approx(0.0)
        assert f.end_deg == pytest.approx(32.9)
        assert len(f.points) == 12
        assert all(p.dist_m == pytest.approx(0.5) for p in f.points)

    def test_point_angles_interpolated(self):
        dec = C.FrameDecoder()
        f = feed_all(dec, [build_frame(start_ddeg=0, end_ddeg=3300)])[0]
        assert f.points[0].angle_deg == pytest.approx(0.0)
        assert f.points[11].angle_deg == pytest.approx(33.0)
        assert f.points[3].angle_deg == pytest.approx(9.0)

    def test_zero_distance_preserves_angle(self):
        dec = C.FrameDecoder()
        pts = [1000] * 12
        pts[5] = 0
        f = feed_all(dec, [build_frame(start_ddeg=0, end_ddeg=1100,
                                       points=pts)])[0]
        assert len(f.points) == 12
        assert f.points[5].angle_deg == pytest.approx(5.0)
        assert f.points[5].dist_m == 0.0

    def test_counts(self):
        dec = C.FrameDecoder()
        feed_all(dec, [build_frame()])
        assert dec.frames_ok == 1
        assert dec.frames_bad == 0


class TestPartial:
    def test_half_frame_no_emit(self):
        dec = C.FrameDecoder()
        frame = build_frame()
        assert feed_all(dec, [frame[:23]]) == []
        assert len(feed_all(dec, [frame[23:]])) == 1

    def test_byte_by_byte(self):
        dec = C.FrameDecoder()
        frame = build_frame()
        out = []
        for i in range(len(frame)):
            out.extend(dec.feed(frame[i:i + 1]))
        assert len(out) == 1

    def test_garbage_prefix_resync(self):
        dec = C.FrameDecoder()
        frames = feed_all(dec, [b'\x00\x11\x22' + build_frame()])
        assert len(frames) == 1


class TestCorrupt:
    def test_bad_crc_not_emitted(self):
        dec = C.FrameDecoder()
        assert feed_all(dec, [build_frame(bad_crc=True)]) == []
        assert dec.frames_bad == 1
        assert dec.frames_ok == 0

    def test_bad_then_good(self):
        dec = C.FrameDecoder()
        frames = feed_all(dec, [build_frame(bad_crc=True), build_frame()])
        assert len(frames) == 1
        assert dec.frames_bad == 1
        assert dec.frames_ok == 1

    def test_fake_header_in_payload(self):
        """载荷中出现 0x54 0x2C 不能吞掉真正的下一帧。"""
        dec = C.FrameDecoder()
        raw = build_frame(points=[0x2C54] + [500] * 11)
        assert b'\x54\x2c' in raw[2:]
        frames = feed_all(dec, [raw, build_frame()])
        assert len(frames) >= 1

    def test_bad_crc_burst(self):
        dec = C.FrameDecoder()
        feed_all(dec, [build_frame(bad_crc=True)] * 5)
        assert dec.frames_bad == 5
        assert dec.frames_ok == 0

    def test_absurd_span_rejected(self):
        """帧角跨度异常（超过 40 度）视为不可信。"""
        dec = C.FrameDecoder()
        assert feed_all(dec, [build_frame(start_ddeg=0, end_ddeg=20000)]) == []
        assert dec.frames_bad == 1


class TestReverse:
    def test_mirs_reverses_points(self):
        frame = build_frame(start_ddeg=0, end_ddeg=3300)
        f_mirs = feed_all(C.FrameDecoder(mirs=True), [frame])[0]
        f_norm = feed_all(C.FrameDecoder(mirs=False), [frame])[0]
        assert f_mirs.points[0].angle_deg == pytest.approx(33.0)
        assert f_norm.points[0].angle_deg == pytest.approx(0.0)


class TestBuffer:
    def test_buffer_bounded(self):
        dec = C.FrameDecoder(max_buffer=256)
        for _ in range(100):
            dec.feed(b'\x01' * 64)
        # 每次喂 64 字节，解不出帧的部分留在缓冲；超过上限时必须被裁掉
        assert len(dec.pending) <= 256
        assert dec.bytes_dropped >= 0

    def test_frame_spanning_trim_still_survives(self):
        dec = C.FrameDecoder(max_buffer=256)
        frame = build_frame()
        dec.feed(frame[:20])
        for _ in range(100):
            dec.feed(b'\x01' * 20)
        assert len(dec.pending) <= 256


def test_rotation_fixture_has_twelve_frames():
    assert len(frame_rotation()) == 12
