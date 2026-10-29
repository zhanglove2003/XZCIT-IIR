#!/usr/bin/env python3
"""font_convert 的单元测试：python3 tools/test_font_convert.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import font_convert as fc


def test_rot90cw():
    img = [[0] * 16 for _ in range(16)]
    img[0][0] = 1
    img[1][0] = 1
    out = fc.rot90cw(img)
    assert out[0][15] == 1
    assert out[0][14] == 1


def test_ascii_table_complete():
    assert len(fc.ASCII_RAW) == 95
    for glyph in fc.ASCII_RAW:
        assert len(glyph) == 12


def test_ascii_zero():
    page = fc.ascii_to_page(fc.ascii_raw('0'))
    assert list(page) == [0xF8, 0x04, 0x04, 0x04, 0xF8, 0x00,
                          0x01, 0x02, 0x02, 0x02, 0x01, 0x00]


def test_cn_duplicates():
    assert fc.CN_RAW['wait'][3] == fc.CN_RAW['start'][5]    # 始
    assert fc.CN_RAW['start'][0] == fc.CN_RAW['items'][2]   # 自
    assert fc.CN_RAW['start'][1] == fc.CN_RAW['items'][3]   # 检
    assert fc.CN_RAW['items'][0] == fc.CN_RAW['items'][4]   # 电


def test_cn_glyphs():
    assert len(fc.CN_GLYPHS) == 17
    for ch, data in fc.CN_GLYPHS.items():
        assert len(data) == 32, ch
    img = fc.cn_bitmap('等')
    assert len(img) == 16 and len(img[0]) == 16
    assert img[0][6] == 1 and img[0][7] == 0
    assert any(any(row) for row in img)


def test_assets():
    for frame in fc.spinner_frames():
        assert len(frame) == 24
        assert sum(bin(b).count('1') for b in frame) == 12
    assert sum(bin(b).count('1') for b in fc.CHECK_16) > 20
    assert sum(bin(b).count('1') for b in fc.CROSS_16) > 20


def test_generate():
    header, source = fc.generate()
    assert '#define OLED_FONT_CN_COUNT 17' in header
    assert 'oled_font_cn[OLED_FONT_CN_COUNT][32]' in source
    assert 'oled_font_cn_lookup' in source
    assert source.count('0x') > 1500


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('PASS', name)
    print('ALL TESTS PASSED')
