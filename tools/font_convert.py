#!/usr/bin/env python3
"""把导出字模重组为正立 page 格式，生成 Core/Src/oled_font.c 与 Core/Inc/oled_font.h。

原始汉字字模按 page 格式解码后需要转置才能还原正立字形。
仅顺时针旋转 90° 会把每个汉字左右镜像，造成偏旁位置颠倒。
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(ROOT, 'tools', 'font_src')
OUT_C = os.path.join(ROOT, 'Core', 'Src', 'oled_font.c')
OUT_H = os.path.join(ROOT, 'Core', 'Inc', 'oled_font.h')

CN_SOURCES = {
    'wait': 'CNINT16x16.h',
    'loading': 'CNST16x16.h',
    'start': 'CNZJ16x16.h',
    'items': 'CNZJ16x16_1.h',
}

# (汉字, 来源, 序号)；17 个不重复字形（始 只在 wait 中登记，start 里的 始 与它逐字节相同）
CN_GLYPHS_SPEC = [
    ('等', 'wait', 0), ('待', 'wait', 1), ('初', 'wait', 2), ('始', 'wait', 3), ('化', 'wait', 4),
    ('正', 'loading', 0), ('在', 'loading', 1), ('加', 'loading', 2), ('载', 'loading', 3),
    ('自', 'start', 0), ('检', 'start', 1), ('流', 'start', 2), ('程', 'start', 3),
    ('开', 'start', 4),
    ('电', 'items', 0), ('机', 'items', 1), ('池', 'items', 5),
]


def _parse_glyphs(path, width):
    text = open(path, encoding='utf-8', errors='ignore').read()
    m = re.search(r'\[\s*\d+\s*\]\s*\[\s*%d\s*\]\s*=\s*\{(.*?)\n\};' % width, text, re.S)
    if not m:
        raise SystemExit('未找到字模数组: %s' % path)
    glyphs = []
    for block in re.findall(r'\{([^{}]*)\}', m.group(1)):
        vals = [int(v, 16) for v in re.findall(r'0x[0-9A-Fa-f]+', block)]
        if len(vals) != width:
            raise SystemExit('字形长度异常 %s: %d' % (path, len(vals)))
        glyphs.append(vals)
    return glyphs


def _load():
    ascii_raw = _parse_glyphs(os.path.join(SRC_DIR, 'ascii1206.h'), 12)
    cn_raw = {key: _parse_glyphs(os.path.join(SRC_DIR, name), 32)
              for key, name in CN_SOURCES.items()}
    return ascii_raw, cn_raw


ASCII_RAW, CN_RAW = _load()


def decode_page(data):
    """16×16 page 格式 → 像素矩阵 [row][col]"""
    img = [[0] * 16 for _ in range(16)]
    for page in range(2):
        for col in range(16):
            byte = data[page * 16 + col]
            for bit in range(8):
                img[page * 8 + bit][col] = (byte >> bit) & 1
    return img


def rot90cw(img):
    return [[img[15 - c][r] for c in range(16)] for r in range(16)]


def page_encode(img):
    """像素矩阵 → page 格式（行数向上取整到 8，bit0 为页内顶行）"""
    h = len(img)
    w = len(img[0])
    out = bytearray(((h + 7) // 8) * w)
    for page in range((h + 7) // 8):
        for col in range(w):
            byte = 0
            for bit in range(8):
                row = page * 8 + bit
                if row < h and img[row][col]:
                    byte |= 1 << bit
            out[page * w + col] = byte
    return bytes(out)


def ascii_raw(ch):
    return ASCII_RAW[ord(ch) - 32]


def ascii_to_page(raw):
    """6 列 × 16 位（bit15 为顶行，仅用 12 行）→ page 格式 12 字节"""
    img = [[0] * 6 for _ in range(12)]
    for col in range(6):
        value = (raw[2 * col] << 8) | raw[2 * col + 1]
        for row in range(12):
            img[row][col] = (value >> (15 - row)) & 1
    return page_encode(img)


def cn_bitmap(ch):
    for name, key, idx in CN_GLYPHS_SPEC:
        if name == ch:
            img = decode_page(CN_RAW[key][idx])
            return [[img[col][row] for col in range(16)] for row in range(16)]
    raise KeyError(ch)


CN_INDEX = {ch: (key, idx) for ch, key, idx in CN_GLYPHS_SPEC}
CN_GLYPHS = {ch: page_encode(cn_bitmap(ch)) for ch, _key, _idx in CN_GLYPHS_SPEC}

# 10×16 标题字模字符集（OLED 品牌页标题用）；由 6×12 最近邻放大生成
TITLE_CHARS = 'XZCIT-MFS'


def title_bitmap(raw):
    """6×12 原始位图最近邻放大到 10×16 → page 格式 20 字节"""
    img = [[0] * 6 for _ in range(12)]
    for col in range(6):
        value = (raw[2 * col] << 8) | raw[2 * col + 1]
        for row in range(12):
            img[row][col] = (value >> (15 - row)) & 1
    big = [[0] * 10 for _ in range(16)]
    for row in range(16):
        for col in range(10):
            big[row][col] = img[row * 12 // 16][col * 6 // 10]
    return page_encode(big)


TITLE_GLYPHS = [title_bitmap(ascii_raw(ch)) for ch in TITLE_CHARS]

SPINNER_POS = [(10, 6), (9, 9), (6, 10), (3, 9), (2, 6), (3, 3), (6, 2), (9, 3)]


def spinner_frames():
    frames = []
    for k in range(8):
        img = [[0] * 12 for _ in range(12)]
        for i in range(3):
            x, y = SPINNER_POS[(k + i) % 8]
            for dy in range(2):
                for dx in range(2):
                    img[y + dy][x + dx] = 1
        frames.append(page_encode(img))
    return frames


def _line(img, x0, y0, x1, y1, size, w, h):
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        for by in range(size):
            for bx in range(size):
                x, y = x0 + bx, y0 + by
                if 0 <= x < w and 0 <= y < h:
                    img[y][x] = 1
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def _check_image():
    img = [[0] * 16 for _ in range(16)]
    _line(img, 2, 10, 6, 13, 2, 16, 16)
    _line(img, 6, 13, 13, 3, 2, 16, 16)
    return img


def _cross_image():
    img = [[0] * 16 for _ in range(16)]
    _line(img, 3, 3, 12, 12, 2, 16, 16)
    _line(img, 12, 3, 3, 12, 2, 16, 16)
    return img


CHECK_16 = page_encode(_check_image())
CROSS_16 = page_encode(_cross_image())


def _bytes(values, per_line=12):
    lines = []
    for i in range(0, len(values), per_line):
        chunk = ', '.join('0x%02X' % v for v in values[i:i + per_line])
        lines.append('    ' + chunk + ',')
    return '\n'.join(lines)


def _header():
    return '''/* 自动生成：tools/font_convert.py —— 请勿手改 */
#ifndef OLED_FONT_H
#define OLED_FONT_H

#include <stdint.h>

#define OLED_FONT_CN_COUNT 17
#define OLED_FONT_TITLE_COUNT 9

/* 6×12 ASCII，page 格式（2 页 × 6 列），bit0 为页内顶行 */
extern const uint8_t oled_font_ascii[95][12];

/* 10×16 标题 ASCII，page 格式（2 页 × 10 列），由 6×12 最近邻放大 */
extern const uint8_t oled_font_title[OLED_FONT_TITLE_COUNT][20];

/* 16×16 汉字，page 格式（2 页 × 16 列），已旋转为正立 */
extern const uint8_t oled_font_cn[OLED_FONT_CN_COUNT][32];

/* 12×12 转圈动画，8 帧，page 格式 */
extern const uint8_t oled_font_spinner[8][24];

extern const uint8_t oled_font_check[32];
extern const uint8_t oled_font_cross[32];

const uint8_t *oled_font_cn_lookup(const char *utf8);
const uint8_t *oled_font_title_lookup(char c);

#endif
'''


def _source():
    lines = ['/* 自动生成：tools/font_convert.py —— 请勿手改 */',
             '#include "oled_font.h"',
             '',
             'const uint8_t oled_font_ascii[95][12] = {']
    for i, raw in enumerate(ASCII_RAW):
        lines.append('    {' + ', '.join('0x%02X' % v for v in ascii_to_page(raw)) + '}, /* %d */' % (32 + i))
    lines.append('};')
    lines.append('')
    lines.append('const uint8_t oled_font_title[OLED_FONT_TITLE_COUNT][20] = {')
    for ch, data in zip(TITLE_CHARS, TITLE_GLYPHS):
        lines.append('    {' + ', '.join('0x%02X' % v for v in data) + '}, /* %c */' % ch)
    lines.append('};')
    lines.append('')
    lines.append('const uint8_t oled_font_cn[OLED_FONT_CN_COUNT][32] = {')
    for ch, _key, _idx in CN_GLYPHS_SPEC:
        data = CN_GLYPHS[ch]
        lines.append('    {' + ', '.join('0x%02X' % v for v in data) + '}, /* %s */' % ch)
    lines.append('};')
    lines.append('')
    lines.append('const uint8_t oled_font_spinner[8][24] = {')
    for frame in spinner_frames():
        lines.append('    {' + ', '.join('0x%02X' % v for v in frame) + '},')
    lines.append('};')
    lines.append('')
    lines.append('const uint8_t oled_font_check[32] = {')
    lines.append(_bytes(CHECK_16))
    lines.append('};')
    lines.append('')
    lines.append('const uint8_t oled_font_cross[32] = {')
    lines.append(_bytes(CROSS_16))
    lines.append('};')
    lines.append('')
    lines.append('typedef struct')
    lines.append('{')
    lines.append('  uint8_t utf8[3];')
    lines.append('  uint8_t index;')
    lines.append('} oled_font_cn_entry_t;')
    lines.append('')
    lines.append('static const oled_font_cn_entry_t s_cn_map[OLED_FONT_CN_COUNT] = {')
    for i, (ch, _key, _idx) in enumerate(CN_GLYPHS_SPEC):
        u = ch.encode('utf-8')
        lines.append('    {{0x%02X, 0x%02X, 0x%02X}, %d}, /* %s */' % (u[0], u[1], u[2], i, ch))
    lines.append('};')
    lines.append('')
    lines.append('const uint8_t *oled_font_cn_lookup(const char *utf8)')
    lines.append('{')
    lines.append('  uint8_t i;')
    lines.append('  for (i = 0; i < OLED_FONT_CN_COUNT; i++)')
    lines.append('  {')
    lines.append('    if (s_cn_map[i].utf8[0] == (uint8_t)utf8[0] &&')
    lines.append('        s_cn_map[i].utf8[1] == (uint8_t)utf8[1] &&')
    lines.append('        s_cn_map[i].utf8[2] == (uint8_t)utf8[2])')
    lines.append('    {')
    lines.append('      return oled_font_cn[i];')
    lines.append('    }')
    lines.append('  }')
    lines.append('  return 0;')
    lines.append('}')
    lines.append('')
    lines.append('static const char s_title_chars[OLED_FONT_TITLE_COUNT] = {')
    lines.append('    ' + ', '.join("'%c'" % ch for ch in TITLE_CHARS))
    lines.append('};')
    lines.append('')
    lines.append('const uint8_t *oled_font_title_lookup(char c)')
    lines.append('{')
    lines.append('  uint8_t i;')
    lines.append('  for (i = 0; i < OLED_FONT_TITLE_COUNT; i++)')
    lines.append('  {')
    lines.append('    if (s_title_chars[i] == c)')
    lines.append('    {')
    lines.append('      return oled_font_title[i];')
    lines.append('    }')
    lines.append('  }')
    lines.append('  return 0;')
    lines.append('}')
    lines.append('')
    return '\n'.join(lines)


def generate():
    return _header(), _source()


if __name__ == '__main__':
    header, source = generate()
    with open(OUT_H, 'w', encoding='utf-8', newline='\n') as f:
        f.write(header)
    with open(OUT_C, 'w', encoding='utf-8', newline='\n') as f:
        f.write(source)
    print('written:', OUT_H)
    print('written:', OUT_C)
