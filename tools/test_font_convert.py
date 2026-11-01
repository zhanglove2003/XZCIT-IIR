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
    assert len(fc.CN_GLYPHS) == 28 + 60
    for ch, data in fc.CN_GLYPHS.items():
        assert len(data) == 32, ch
    img = fc.cn_bitmap('等')
    assert len(img) == 16 and len(img[0]) == 16
    assert img[0][9] == 1 and img[0][8] == 0
    assert any(any(row) for row in img)


def test_cn_extra_glyphs():
    """Task 9b 视觉打磨增补的 11 个字形：有墨、尺寸正确、来自 extra 源。"""
    extra = '请先查与标定未配置驾驶'
    assert len(extra) == 11
    for ch in extra:
        assert ch in fc.CN_GLYPHS, ch
        assert fc.CN_INDEX[ch] == ('extra', fc.CN_INDEX[ch][1]), ch
        data = fc.CN_GLYPHS[ch]
        assert len(data) == 32
        assert sum(bin(b).count('1') for b in data) > 20, ch
        img = fc.cn_bitmap(ch)
        assert any(any(row) for row in img), ch


# Task 2 全界面中文化增补的 60 个字形（文案清单逐字对照见 task-2-report.md）
TASK2_CHARS = ('菜单即将转动四轮秒后按取消右移或立'
               '手柄连接回中摇杆供不足结果全部通'
               '过欠压采样错误到位失败无参考重新'
               '对齐静止长已记录运故障')


def test_cn_task2_glyphs():
    """中文化文案清单新增字形：来自 extra2 源、有墨、尺寸正确。"""
    assert len(TASK2_CHARS) == 60
    assert len(set(TASK2_CHARS)) == 60
    for ch in TASK2_CHARS:
        assert ch in fc.CN_GLYPHS, ch
        assert fc.CN_INDEX[ch][0] == 'extra2', ch
        data = fc.CN_GLYPHS[ch]
        assert len(data) == 32
        assert sum(bin(b).count('1') for b in data) > 20, ch
        img = fc.cn_bitmap(ch)
        assert any(any(row) for row in img), ch


def test_regen_idempotent():
    """font_convert 重新生成幂等：磁盘上的 oled_font.c/h 与 generate() 一致。"""
    header, source = fc.generate()
    c_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'Core', 'Src', 'oled_font.c')
    h_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'Core', 'Inc', 'oled_font.h')
    with open(c_path, encoding='utf-8') as f:
        assert f.read() == source, 'oled_font.c 与生成器输出不一致（请重跑 font_convert.py）'
    with open(h_path, encoding='utf-8') as f:
        assert f.read() == header, 'oled_font.h 与生成器输出不一致（请重跑 font_convert.py）'


def test_cn_radicals_are_on_the_correct_side():
    # 手工核对：初的“刀”右侧竖笔在 x=13，不能镜像到 x=2。
    # 待的“彳”在左边，不能出现在右边。
    chu = fc.cn_bitmap('初')
    dai = fc.cn_bitmap('待')
    assert chu[4][13] == 1 and chu[4][2] == 0
    assert dai[4][1] == 1 and dai[4][14] == 0


def test_assets():
    for frame in fc.spinner_frames():
        assert len(frame) == 24
        assert sum(bin(b).count('1') for b in frame) == 12
    assert sum(bin(b).count('1') for b in fc.CHECK_16) > 20
    assert sum(bin(b).count('1') for b in fc.CROSS_16) > 20


def test_title_glyphs():
    assert fc.TITLE_CHARS == 'XZCIT-MFS'
    assert len(fc.TITLE_GLYPHS) == 9
    for data in fc.TITLE_GLYPHS:
        assert len(data) == 20
        assert sum(bin(b).count('1') for b in data) > 0


def test_generate():
    header, source = fc.generate()
    assert '#define OLED_FONT_CN_COUNT 88' in header
    assert 'oled_font_cn[OLED_FONT_CN_COUNT][32]' in source
    assert 'oled_font_cn_lookup' in source
    assert source.count('0x') > 1500


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('PASS', name)
    print('ALL TESTS PASSED')
