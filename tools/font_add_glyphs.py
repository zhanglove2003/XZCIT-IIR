#!/usr/bin/env python3
"""用系统 Noto Sans CJK SC 渲染 oled_font 缺失的汉字字形，
生成 tools/font_src/CNADD16x16.h（行列式·横向字节·逆向·16x16，
与 font_convert.py 的 _parse_glyphs/decode_page 约定一致）。

一次性生成工具：产物 .h 已入库，作为 font_convert.py 的字形来源；
仅当需要增删字形时重跑。需要 Pillow。

用法：python3 tools/font_add_glyphs.py [字符集]
默认字符集 = 视觉打磨任务（Task 9b）增补的 11 个字。
"""
import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'tools', 'font_src', 'CNADD16x16.h')

# Task 9b 视觉打磨增补：请先自检 / 电池未配置 / 卡片名 驾驶·检查标定 / 子菜单标题 检查与标定
DEFAULT_CHARS = '请先查与标定未配置驾驶'

# Noto Sans CJK Bold ttc：index 2 = Noto Sans CJK SC
FONT_PATH = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
FONT_INDEX = 2
FONT_SIZE = 32
THRESHOLD = 110


def render(ch, font):
    """渲染单字到 16x16 的 0/1 矩阵：2 倍尺寸绘制，裁剪墨迹包围盒，
    等比缩到 ≤16×16 后抗锯齿阈值化，最后在 16×16 内居中。"""
    canvas = Image.new('L', (96, 96), 0)
    d = ImageDraw.Draw(canvas)
    d.text((24, 24), ch, font=font, fill=255)
    bbox = canvas.getbbox()
    if bbox is None:
        raise SystemExit('字形渲染为空: %s' % ch)
    img = canvas.crop(bbox)
    if img.width > 16 or img.height > 16:
        scale = 16.0 / max(img.width, img.height)
        img = img.resize((max(1, int(img.width * scale)), max(1, int(img.height * scale))),
                         Image.LANCZOS)
    grid = [[0] * 16 for _ in range(16)]
    ox = (16 - img.width) // 2
    oy = (16 - img.height) // 2
    px = img.load()
    for y in range(img.height):
        for x in range(img.width):
            grid[oy + y][ox + x] = 1 if px[x, y] > THRESHOLD else 0
    return grid


def encode(grid):
    """正立像素矩阵 → 行列式·逆向 16x16 字模（32 字节）。
    约定（与 CN*16x16.h 现有字模一致，经 font_convert.decode_page 转置还原）：
    data[y] = 第 y 行左 8 列（bit0=x0）；data[16+y] = 第 y 行右 8 列（bit0=x8）。"""
    out = []
    for y in range(16):
        left = 0
        right = 0
        for x in range(8):
            left |= grid[y][x] << x
            right |= grid[y][8 + x] << x
        out.append(left)
    for y in range(16):
        right = 0
        for x in range(8):
            right |= grid[y][8 + x] << x
        out.append(right)
    return out


def main():
    chars = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CHARS
    out_name = sys.argv[2] if len(sys.argv) > 2 else 'CNADD16x16.h'
    arr_name = out_name[:-2] if out_name.endswith('.h') else out_name
    font = ImageFont.truetype(FONT_PATH, FONT_SIZE, index=FONT_INDEX)
    lines = ['/* 补充汉字字模（阴码 · 行列式（横向字节，8 列一条）· 逆向（低位在前，首点在 bit0）· 16x16）',
             ' * 由 tools/font_add_glyphs.py 从 Noto Sans CJK SC Bold 16px 渲染生成，供 tools/font_convert.py 增补使用。 */',
             'const unsigned char %s[%d][32] = {' % (arr_name, len(chars))]
    for ch in chars:
        grid = render(ch, font)
        art = [' * ' + ''.join('#' if v else '.' for v in row) for row in grid]
        vals = encode(grid)
        hexes = ', '.join('0x%02X' % v for v in vals)
        lines.append('    {' + hexes + '}, /* %s' % ch)
        lines.extend(l + ' */' if i == len(art) - 1 else l + '' for i, l in enumerate(art))
    lines.append('};')
    out_path = os.path.join(os.path.dirname(OUT), out_name)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print('written', out_path, len(chars), 'glyphs')


if __name__ == '__main__':
    main()
