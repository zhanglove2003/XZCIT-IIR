#!/usr/bin/env python3
"""主机端 OLED 预览：把全部中文字形栅格与品牌/确认/子菜单三个页面渲染成 PNG，
供上板前人工目检字形粗细与页面排版。需要 Pillow。

用法：
  python3 tools/render_preview.py glyphs out.png
      88 个 16x16 中文字形（oled_font_cn 顺序）4 倍放大 + 字标注，拼成栅格图。
  python3 tools/render_preview.py page brand|confirm|submenu out.png
      主机构建（preview_ui_main.c + oled_gfx + ui_render + oled_font + ui_model，
      链接方式同 tools/test_ui_render.c）渲染整页帧缓冲，4 倍放大输出 1bit PNG。
"""
import os
import subprocess
import sys
import tempfile

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCALE = 4
LABEL_FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
LABEL_INDEX = 2  # Noto Sans CJK SC


def glyph_tile(ch):
    """单个 16x16 字形 → 放大 tile（顶部留一行字标注）。"""
    import font_convert as fc
    img = fc.cn_bitmap(ch)
    tile = Image.new('1', (16 * SCALE, 16 * SCALE + 18), 0)
    d = ImageDraw.Draw(tile)
    d.text((1, 0), ch, font=ImageFont.truetype(LABEL_FONT, 12, index=LABEL_INDEX), fill=1)
    for y in range(16):
        for x in range(16):
            if img[y][x]:
                d.rectangle((x * SCALE, 18 + y * SCALE,
                             (x + 1) * SCALE - 1, 18 + (y + 1) * SCALE - 1), fill=1)
    return tile


def render_glyphs(out_path):
    import font_convert as fc
    chars = [spec[0] for spec in fc.CN_GLYPHS_SPEC]
    cols = 11
    rows = (len(chars) + cols - 1) // cols
    tw = 16 * SCALE + 4
    th = 16 * SCALE + 22
    sheet = Image.new('1', (cols * tw + 4, rows * th + 4), 0)
    for i, ch in enumerate(chars):
        x = 4 + (i % cols) * tw
        y = 4 + (i // cols) * th
        sheet.paste(glyph_tile(ch), (x, y))
    sheet.save(out_path)
    print('written %s (%d glyphs)' % (out_path, len(chars)))


def build_preview_bin(bin_path):
    cmd = ['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-ICore/Inc',
           'tools/preview_ui_main.c',
           'Core/Src/ui_render.c', 'Core/Src/oled_gfx.c',
           'Core/Src/oled_font.c', 'Core/Src/ui_model.c',
           '-o', bin_path]
    subprocess.run(cmd, cwd=ROOT, check=True)


def render_page(name, out_path):
    with tempfile.TemporaryDirectory() as td:
        bin_path = os.path.join(td, 'preview')
        build_preview_bin(bin_path)
        raw = subprocess.run([bin_path, name], check=True,
                             stdout=subprocess.PIPE).stdout
    if len(raw) != 128 * 64 // 8:
        raise SystemExit('帧缓冲长度异常: %d' % len(raw))
    img = Image.new('1', (128 * SCALE, 64 * SCALE), 0)
    d = ImageDraw.Draw(img)
    for y in range(64):
        for x in range(128):
            byte = raw[(y >> 3) * 128 + x]
            if (byte >> (y & 7)) & 1:
                d.rectangle((x * SCALE, y * SCALE,
                             (x + 1) * SCALE - 1, (y + 1) * SCALE - 1), fill=1)
    img.save(out_path)
    print('written %s (page %s)' % (out_path, name))


def main():
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    mode = sys.argv[1]
    out = sys.argv[-1]
    if not os.path.isabs(out):
        out = os.path.join(ROOT, out)
    if mode == 'glyphs':
        sys.path.insert(0, os.path.join(ROOT, 'tools'))
        render_glyphs(out)
    elif mode == 'page':
        if len(sys.argv) != 4:
            raise SystemExit(__doc__)
        render_page(sys.argv[2], out)
    else:
        raise SystemExit(__doc__)


if __name__ == '__main__':
    main()
