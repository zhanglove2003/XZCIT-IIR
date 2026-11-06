#!/usr/bin/env python3
"""用系统 Noto Sans CJK SC 渲染 oled_font 缺失的汉字字形，
生成 tools/font_src/CNADD*.h（行列式·横向字节·逆向·16x16，
与 font_convert.py 的 _parse_glyphs/decode_page 约定一致）。

需要 Pillow。

两种模式：

1. 固定参数（默认，历史行为）：Bold 32px、固定阈值 110 渲染，
   用于 Task 9b 的 CNADD16x16.h（勿重跑覆盖，产物已入库）。
   用法：python3 tools/font_add_glyphs.py [字符集 [输出文件名]]

2. 墨水密度自动匹配（--auto，2026-11-01 字体粗细匹配）：
   以 oled_font_cn 前 17 个原始点阵字模的 ink/256 均值 μ 为目标，
   逐字在多字重 × 多字号 × 多阈值（含形态学膨胀/收缩兜底）的
   搜索空间里选 ink 落在 [0.85μ, 1.15μ] 且最接近 μ 的参数档；
   全部新字形 ink 极差须 ≤ 0.20μ，无法达标则直接报错（不许硬凑）。
   选用参数写入字模行注释（如“菜 DemiLight/30/132”）便于复查。
   用法：python3 tools/font_add_glyphs.py --auto [字符集 [输出文件名]]
"""
import bisect
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

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

# ---- 墨水密度自动匹配（--auto）----

CJK_DIR = '/usr/share/fonts/opentype/noto'
# index 2 = Noto Sans CJK SC；按字重排序（细 -> 粗），DemiLight 为观感基准档
CJK_WEIGHTS = (
    ('Light', 'NotoSansCJK-Light.ttc'),
    ('DemiLight', 'NotoSansCJK-DemiLight.ttc'),
    ('Regular', 'NotoSansCJK-Regular.ttc'),
    ('Medium', 'NotoSansCJK-Medium.ttc'),
    ('Bold', 'NotoSansCJK-Bold.ttc'),
)
WEIGHT_CENTER = 1                     # 观感基准：DemiLight（与原点阵字模笔画最接近）
AUTO_SIZES = (26, 28, 30, 32, 34, 36, 38)
AUTO_THRESHOLDS = tuple(range(40, 220, 2))
AUTO_MORPHS = (None, 'erode', 'dilate')
MIN_INK = 25                          # 低于此墨量的 16x16 字形笔画已断裂，不可用

# 打分：ink 拟合为主项，其余为同墨量时的观感平手裁决（总和 < 1 px 墨量差，
# 即绝不因观感项牺牲 ink 拟合，只在拟合度相同时偏好 AA 中点阈值/中间字重）
SCORE_THR = 0.02      # 阈值离 128（AA 中点）越远，笔画边缘越毛糙/发虚
SCORE_WEIGHT = 0.25   # 字重离 DemiLight 越远，风格差异越大
SCORE_SIZE = 0.10     # 字号离 32（原管道基准）越远，比例越异常
SCORE_MORPH = 0.60    # 形态学兜底比纯阈值化更不自然


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
    return _binarize(img, THRESHOLD)


def _binarize(img, threshold):
    w = img.width
    h = img.height
    grid = [[0] * 16 for _ in range(16)]
    ox = (16 - w) // 2
    oy = (16 - h) // 2
    px = img.load()
    for y in range(h):
        for x in range(w):
            grid[oy + y][ox + x] = 1 if px[x, y] > threshold else 0
    return grid


def _render_gray(ch, font, size):
    """固定参数渲染的前半段：大画布绘制 → 裁剪 → LANCZOS 缩到 ≤16×16 灰度。
    返回灰度小图（≤16×16，居中信息可由尺寸推出），供多档阈值复用。"""
    pad = size * 3
    canvas = Image.new('L', (pad, pad), 0)
    d = ImageDraw.Draw(canvas)
    d.text((size * 3 // 4, size * 3 // 4), ch, font=font, fill=255)
    bbox = canvas.getbbox()
    if bbox is None:
        raise SystemExit('字形渲染为空: %s' % ch)
    img = canvas.crop(bbox)
    if img.width > 16 or img.height > 16:
        scale = 16.0 / max(img.width, img.height)
        img = img.resize((max(1, int(img.width * scale)), max(1, int(img.height * scale))),
                         Image.LANCZOS)
    return img


def _gray_to_grid(img, threshold):
    """灰度小图 → 16×16 内居中的 0/1 矩阵（> threshold 记墨）。"""
    return _binarize(img, threshold)


def _ink(grid):
    return sum(sum(row) for row in grid)


def _erode(grid):
    """4 邻域收缩：仅当自身与四邻全为墨时保留（贴边的笔画端点收 1 px）。"""
    out = [[0] * 16 for _ in range(16)]
    for y in range(16):
        for x in range(16):
            if grid[y][x]:
                if (x > 0 and grid[y][x - 1] and x < 15 and grid[y][x + 1] and
                        y > 0 and grid[y - 1][x] and y < 15 and grid[y + 1][x]):
                    out[y][x] = 1
    return out


def _dilate(grid):
    """4 邻域膨胀：自身或任一四邻为墨即置墨。"""
    out = [[0] * 16 for _ in range(16)]
    for y in range(16):
        for x in range(16):
            if grid[y][x]:
                out[y][x] = 1
            else:
                if ((x > 0 and grid[y][x - 1]) or (x < 15 and grid[y][x + 1]) or
                        (y > 0 and grid[y - 1][x]) or (y < 15 and grid[y + 1][x])):
                    out[y][x] = 1
    return out


def _apply_morph(grid, morph):
    if morph == 'erode':
        return _erode(grid)
    if morph == 'dilate':
        return _dilate(grid)
    return grid


def original_mu():
    """oled_font_cn 前 17 项（原始点阵字模）的 ink/256 均值 μ。"""
    import font_convert as fc
    ratios = []
    for ch, _key, _idx in fc.CN_GLYPHS_SPEC[:17]:
        img = fc.cn_bitmap(ch)
        ratios.append(sum(sum(row) for row in img) / 256.0)
    return sum(ratios) / len(ratios)


def auto_params(chars, verbose=False):
    """逐字搜索 ink ∈ [0.85μ, 1.15μ] 且最接近 μ 的渲染参数档。

    返回 {ch: (字重, 字号, 阈值, 形态学)}；搜完全局校验极差 ≤ 0.20μ，
    有字无法达标（含形态学兜底）则 SystemExit 列出，绝不硬凑。"""
    mu = original_mu()
    target = mu * 256.0
    lo = 0.85 * target
    hi = 1.15 * target
    fonts = {(w, s): ImageFont.truetype(os.path.join(CJK_DIR, f), s, index=FONT_INDEX)
             for w, f in CJK_WEIGHTS for s in AUTO_SIZES}
    params = {}
    report = []
    for ch in chars:
        grays = {}
        for w, _f in CJK_WEIGHTS:
            for s in AUTO_SIZES:
                grays[(w, s)] = _render_gray(ch, fonts[(w, s)], s)
        best = None
        for w, _f in CJK_WEIGHTS:
            wi = CJK_WEIGHTS.index((w, _f))
            for s in AUTO_SIZES:
                vals = sorted(grays[(w, s)].getdata())
                n = len(vals)
                for thr in AUTO_THRESHOLDS:
                    ink = n - bisect.bisect_right(vals, thr)
                    if ink < MIN_INK:
                        continue
                    score = (abs(ink - target) + SCORE_THR * abs(thr - 128) +
                             SCORE_WEIGHT * abs(wi - WEIGHT_CENTER) +
                             SCORE_SIZE * abs(s - 32))
                    if best is None or score < best[0]:
                        best = (score, w, s, thr, None, ink)
        # 兜底：纯阈值化无法入带时再做形态学收缩/膨胀（仅对网格级操作）
        if best is None or not lo <= best[5] <= hi:
            for w, _f in CJK_WEIGHTS:
                wi = CJK_WEIGHTS.index((w, _f))
                for s in AUTO_SIZES:
                    for thr in AUTO_THRESHOLDS[::2]:
                        base = _gray_to_grid(grays[(w, s)], thr)
                        for morph in ('erode', 'dilate'):
                            ink = _ink(_apply_morph(base, morph))
                            if ink < MIN_INK:
                                continue
                            score = (abs(ink - target) + SCORE_THR * abs(thr - 128) +
                                     SCORE_WEIGHT * abs(wi - WEIGHT_CENTER) +
                                     SCORE_SIZE * abs(s - 32) + SCORE_MORPH)
                            if best is None or score < best[0]:
                                best = (score, w, s, thr, morph, ink)
        if best is None or not lo <= best[5] <= hi:
            got = '无候选' if best is None else 'ink=%d (ratio %.4f)' % (best[5], best[5] / 256.0)
            raise SystemExit('字形 %s 无法达到墨水密度目标 [%.4f, %.4f] (μ=%.4f): %s'
                             % (ch, lo / 256.0, hi / 256.0, mu, got))
        _score, w, s, thr, morph, ink = best
        params[ch] = (w, s, thr, morph)
        report.append((ch, w, s, thr, morph, ink, ink / 256.0))
    ratios = [r[6] for r in report]
    spread = max(ratios) - min(ratios)
    if spread > 0.20 * mu:
        worst = max(report, key=lambda r: r[6])
        best_r = min(report, key=lambda r: r[6])
        raise SystemExit('全部字形 ink 极差 %.4f > 0.20μ=%.4f（%s=%.4f / %s=%.4f）'
                         % (spread, 0.20 * mu, worst[0], worst[6], best_r[0], best_r[6]))
    if verbose:
        print('mu=%.4f band=[%.4f, %.4f] spread=%.4f (limit %.4f)'
              % (mu, lo / 256.0, hi / 256.0, spread, 0.20 * mu))
        for ch, w, s, thr, morph, ink, ratio in report:
            print('%s weight=%-9s size=%d thr=%3d morph=%-6s ink=%3d ratio=%.4f'
                  % (ch, w, s, thr, morph or '-', ink, ratio))
    return params


def render_grid(ch, weight, size, thr, morph=None):
    """按选定参数渲染一个字的 16×16 网格。"""
    font = ImageFont.truetype(os.path.join(CJK_DIR, dict(CJK_WEIGHTS)[weight]),
                              size, index=FONT_INDEX)
    grid = _gray_to_grid(_render_gray(ch, font, size), thr)
    return _apply_morph(grid, morph)


def encode(grid):
    """正立像素矩阵 → 行列式·逆向 16x16 字模（32 字节）。
    约定（与 CN*16x16.h 现有字模一致，经 font_convert.decode_page 转置还原）：
    data[y] = 第 y 行左 8 列（bit0=x0）；data[16+y] = 第 y 行右 8 列（bit0=x8）。"""
    out = []
    for y in range(16):
        left = 0
        for x in range(8):
            left |= grid[y][x] << x
        out.append(left)
    for y in range(16):
        right = 0
        for x in range(8):
            right |= grid[y][8 + x] << x
        out.append(right)
    return out


def _glyph_block(ch, grid, tag):
    art = [' * ' + ''.join('#' if v else '.' for v in row) for row in grid]
    hexes = ', '.join('0x%02X' % v for v in encode(grid))
    head = '    {' + hexes + '}, /* %s' % ch
    if tag:
        head += ' ' + tag
    return [head] + [l + ' */' if i == len(art) - 1 else l for i, l in enumerate(art)]


def build_content(chars, params):
    """按 {ch: (字重, 字号, 阈值, 形态学)} 组装 CNADD*.h 文件内容。"""
    lines = ['/* 补充汉字字模（阴码 · 行列式（横向字节，8 列一条）· 逆向（低位在前，首点在 bit0）· 16x16）',
             ' * 由 tools/font_add_glyphs.py --auto 从 Noto Sans CJK SC 渲染生成，供 tools/font_convert.py 增补使用。',
             ' * 逐字渲染参数（字重/字号/阈值[+形态学]）按原始 17 字模墨水密度均值 μ 自动匹配（见行注释）。 */']
    chars = list(chars)
    lines.append('const unsigned char CNADD_AUTO[%d][32] = {' % len(chars))
    for ch in chars:
        w, s, thr, morph = params[ch]
        grid = render_grid(ch, w, s, thr, morph)
        tag = '%s/%d/%d' % (w, s, thr)
        if morph:
            tag += '+' + morph
        lines.extend(_glyph_block(ch, grid, tag))
    lines.append('};')
    return '\n'.join(lines) + '\n'


def build_content_named(chars, params, arr_name):
    """同 build_content，但数组名按输出文件名推导（font_convert 按文件名解析）。"""
    return build_content(chars, params).replace('CNADD_AUTO', arr_name, 1)


def main():
    argv = [a for a in sys.argv[1:] if a != '--auto']
    auto = '--auto' in sys.argv[1:]
    chars = argv[0] if argv else (DEFAULT_CHARS if not auto else _task2_chars())
    out_name = argv[1] if len(argv) > 1 else ('CNADD2_16x16.h' if auto else 'CNADD16x16.h')
    arr_name = out_name[:-2] if out_name.endswith('.h') else out_name
    if auto:
        params = auto_params(chars, verbose=True)
        content = build_content_named(chars, params, arr_name)
    else:
        font = ImageFont.truetype(FONT_PATH, FONT_SIZE, index=FONT_INDEX)
        chars = list(chars)
        lines = ['/* 补充汉字字模（阴码 · 行列式（横向字节，8 列一条）· 逆向（低位在前，首点在 bit0）· 16x16）',
                 ' * 由 tools/font_add_glyphs.py 从 Noto Sans CJK SC Bold 16px 渲染生成，供 tools/font_convert.py 增补使用。 */',
                 'const unsigned char %s[%d][32] = {' % (arr_name, len(chars))]
        for ch in chars:
            grid = render(ch, font)
            lines.extend(_glyph_block(ch, grid, ''))
        lines.append('};')
        content = '\n'.join(lines) + '\n'
    out_path = os.path.join(os.path.dirname(OUT), out_name)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(content)
    print('written', out_path, len(chars), 'glyphs')


def _task2_chars():
    import font_convert as fc
    return fc.TASK2_CHARS


if __name__ == '__main__':
    main()
