# OLED 开机自检引导界面 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 上电后由 0.96" OLED（SSD1306，软 SPI，无 CS）引导完成「等待按键 → 加载动画 → 电机 24 s 自检 + VDD 自检并打勾 → 主页面显示四轮转速与板级电压」的完整流程。

**Architecture:** 单个 FreeRTOS 任务顺序跑状态机（`boot_ui.c`），底层分为可主机测试的纯 C 帧缓冲绘制层（`oled_gfx.c`）与 HAL 软 SPI 硬件层（`oled.c`）；字库由 Python 工具从导出字模重新编码并生成 C 数组，随仓库提交。

**Tech Stack:** STM32F407VET6 + STM32 HAL + FreeRTOS（CMSIS_V2）、arm-none-eabi-gcc + Makefile、Python 3（字模转换）、主机 gcc（单元测试）。

**Spec:** `docs/superpowers/specs/2026-10-29-oled-boot-ui-design.md`

## Global Constraints

- 编译/烧录前必须 `source /mnt/data/project-ai/Embedded_Development/toolchain/env.sh`（工具链不在默认 PATH）。
- 构建：`make -j8`；产物 `build/hal_f407_mecanum_4motor_src.elf`。
- 不改 `.ioc`、不重新生成 CubeMX 代码；`main.h` 里已有的 `MOTOR_RF_*` / `SELFCHECK_*` 宏保留并使用。
- 新增的每个 `.c` 必须手动加入 Makefile 的 `C_SOURCES`（Makefile 不会自动扫描）。
- 固件内禁止 `printf`/`sprintf`/浮点格式化；数字用整数除法手工拼字符。
- OLED 四个 GPIO（PD7/PD4/PE0/PE1）已在 `gpio.c` 初始化，驱动层不得重复初始化 GPIO。
- 文本坐标可为任意像素（非 8 的倍数），绘制层按像素写入帧缓冲。
- 代码注释用中文，标识符用英文；提交信息用英文 `type: summary` 格式。

## 文件结构

| 文件 | 责任 |
|---|---|
| `tools/font_src/*.h` | 字模原始数据（用户导出的 4 个汉字文件 + 从厂商表提取的 ASCII 表），只读 |
| `tools/font_convert.py` | 解码 → 旋转/编码 → 生成 `oled_font.c/h`，也可被测试导入 |
| `tools/test_font_convert.py` | 转换器单元测试（python3 直接运行） |
| `Core/Inc/oled_font.h`、`Core/Src/oled_font.c` | **生成文件**：ASCII 95×12、汉字 18×32、转圈 8×24、✓/✗ 32、UTF-8 查找 |
| `Core/Inc/oled_gfx.h`、`Core/Src/oled_gfx.c` | 纯 C 绘制层（无 HAL）：帧缓冲、像素、位图、6×12/16×16 文本 |
| `tools/test_oled_gfx.c` | 绘制层主机测试（gcc） |
| `Core/Inc/oled.h`、`Core/Src/oled.c` | SSD1306 软 SPI：复位、初始化、flush、方向宏 |
| `Core/Inc/boot_ui.h`、`Core/Src/boot_ui.c` | 状态机 + VDD + 转速；头文件放纯数学内联函数 |
| `tools/test_boot_math.c` | 电压/转速换算主机测试 |
| `Core/Src/freertos.c` | 任务入口调用 `boot_ui_run()`，栈 512×4 |
| `Makefile` | 加入 4 个新源文件 |

## 任务依赖

Task 0 → Task 1 → Task 2 → Task 3 → Task 4 → Task 5（顺序执行，后一个依赖前一个的产物）。

---

### Task 0: 提交现有 A 轮自检基线

当前工作区有未提交的 A 轮自检实现（`main.h` 宏 + `freertos.c` 任务），它是本计划的基础，先落成基线提交，后续差异才干净。

**Files:**
- Modify: 无（仅提交现有改动）

**Interfaces:**
- Consumes: 无
- Produces: 干净的 `git status`，后续任务只提交自己的文件

- [ ] **Step 1: 确认改动内容只有自检相关**

Run: `git status --short && git diff --stat`
Expected: 只有 `Core/Inc/main.h`、`Core/Src/freertos.c`（gpio.c 若出现修改需先向用户确认）

- [ ] **Step 2: 提交**

```bash
git add Core/Inc/main.h Core/Src/freertos.c
git commit -m "feat: add RF wheel self-check task with SLP and PWM control"
```

- [ ] **Step 3: 确认工作区干净**

Run: `git status --short`
Expected: 无输出

---

### Task 1: 字模工具与生成数据

把用户导出的 4 个汉字字模（列行式 + 整体旋转 90°）和厂商 6×12 ASCII 表转换成「正立 page 格式」C 数组，并生成 UTF-8 查找表。

**Files:**
- Create: `tools/font_src/CNINT16x16.h`、`tools/font_src/CNST16x16.h`、`tools/font_src/CNZJ16x16.h`、`tools/font_src/CNZJ16x16_1.h`、`tools/font_src/ascii1206.h`
- Create: `tools/font_convert.py`
- Test: `tools/test_font_convert.py`
- Generated: `Core/Inc/oled_font.h`、`Core/Src/oled_font.c`

**Interfaces:**
- Consumes: 用户文件 `/home/snow/下载/CNINT16x16.h`、`CNST16x16.h`、`CNZJ16x16.h`、`CNZJ16x16 (1).h`
- Produces:
  - `const uint8_t oled_font_ascii[95][12]` — 6×12，page 格式（2 页 × 6 列），bit0 为页内顶行
  - `const uint8_t oled_font_cn[17][32]` — 16×16，page 格式
  - `const uint8_t oled_font_spinner[8][24]` — 12×12，page 格式
  - `const uint8_t oled_font_check[32]` / `oled_font_cross[32]` — 16×16
  - `const uint8_t *oled_font_cn_lookup(const char *utf8)` — 传入 3 字节 UTF-8 起始地址，找不到返回 NULL
  - `#define OLED_FONT_CN_COUNT 17`

- [ ] **Step 1: 复制原始字模到仓库**

```bash
mkdir -p tools/font_src
cp "/home/snow/下载/CNINT16x16.h"        tools/font_src/CNINT16x16.h
cp "/home/snow/下载/CNST16x16.h"         tools/font_src/CNST16x16.h
cp "/home/snow/下载/CNZJ16x16.h"         tools/font_src/CNZJ16x16.h
cp "/home/snow/下载/CNZJ16x16 (1).h"     tools/font_src/CNZJ16x16_1.h
```

- [ ] **Step 2: 从厂商表提取 6×12 ASCII 字模**

厂商这套 `oled_asc2_1206` 是斜体小字号设计：E/F/H/I/L/N/T/X 等字母本来带斜笔和衬线端点，
渲染出来“不像方正的 E”属字库原样式，不是解码错误——用 `0`、`!`、`A` 三个字形即可判断解码是否正确。

```bash
python3 - <<'PY'
import re
src = ('/mnt/data/project-ai/Embedded_Development/project/vendor_C50C/'
       '3.STM32运动底盘源码/1.1 STM32运动底盘源码_ROS机器人控制板_C50C（默认使用该程序）/'
       'WHEELTEC_C50X_2026.09.15/HARDWARE/Inc/oledfont.h')
text = open(src, encoding='utf-8', errors='ignore').read()
m = re.search(r'(const unsigned char oled_asc2_1206\[95\]\[12\]\s*=\s*\{.*?\n\};)', text, re.S)
assert m, '未找到 oled_asc2_1206'
out = '/* 从厂商 oledfont.h 提取：6×12 ASCII，6 列 × 2 字节/列，bit15 为顶行 */\n' + m.group(1) + '\n'
open('tools/font_src/ascii1206.h', 'w', encoding='utf-8', newline='\n').write(out)
print('OK', len(m.group(1)))
PY
```
Expected: 打印 `OK` 及约 12000 的字符数

- [ ] **Step 3: 写失败的测试**

Create `tools/test_font_convert.py`:

```python
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
```

- [ ] **Step 4: 运行测试确认失败**

Run: `python3 tools/test_font_convert.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'font_convert'`

- [ ] **Step 5: 实现转换器**

Create `tools/font_convert.py`:

```python
#!/usr/bin/env python3
"""把导出字模重组为正立 page 格式，生成 Core/Src/oled_font.c 与 Core/Inc/oled_font.h。

原始汉字字模是「列行式 + 整体旋转 90°」，文件头注释的「行列式（横向字节）」
与实际不符；这里按实际解码（page 格式解码 → 顺时针旋转 90°）还原正立字形。
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
            return rot90cw(decode_page(CN_RAW[key][idx]))
    raise KeyError(ch)


CN_INDEX = {ch: (key, idx) for ch, key, idx in CN_GLYPHS_SPEC}
CN_GLYPHS = {ch: page_encode(cn_bitmap(ch)) for ch, _key, _idx in CN_GLYPHS_SPEC}

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

/* 6×12 ASCII，page 格式（2 页 × 6 列），bit0 为页内顶行 */
extern const uint8_t oled_font_ascii[95][12];

/* 16×16 汉字，page 格式（2 页 × 16 列），已旋转为正立 */
extern const uint8_t oled_font_cn[OLED_FONT_CN_COUNT][32];

/* 12×12 转圈动画，8 帧，page 格式 */
extern const uint8_t oled_font_spinner[8][24];

extern const uint8_t oled_font_check[32];
extern const uint8_t oled_font_cross[32];

const uint8_t *oled_font_cn_lookup(const char *utf8);

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
```

- [ ] **Step 6: 运行测试确认通过**

Run: `python3 tools/test_font_convert.py`
Expected: 7 行 `PASS ...` + `ALL TESTS PASSED`

- [ ] **Step 7: 生成字库文件**

Run: `python3 tools/font_convert.py`
Expected: 打印两个 `written:` 路径

- [ ] **Step 8: 检查生成的汉字数组顺序**

Run: `grep -n "0x.*}, /\*" Core/Src/oled_font.c | head -20`
Expected: 依次是 等 待 初 始 化 正 在 加 载 自 检 流 程 开 电 机 池，共 17 行

- [ ] **Step 9: 提交**

```bash
git add tools/font_src tools/font_convert.py tools/test_font_convert.py Core/Inc/oled_font.h Core/Src/oled_font.c
git commit -m "feat: add OLED font pipeline and generated 6x12/16x16 glyph data"
```

---

### Task 2: 帧缓冲绘制层（主机可测）

纯 C 的 128×64 帧缓冲与文本绘制，不依赖 HAL；在主机上用 gcc 编译测试。

**Files:**
- Create: `Core/Inc/oled_gfx.h`、`Core/Src/oled_gfx.c`
- Test: `tools/test_oled_gfx.c`

**Interfaces:**
- Consumes: `oled_font.h`（Task 1 生成）
- Produces:
  - `#define OLED_WIDTH 128U`、`#define OLED_HEIGHT 64U`
  - `void oled_gfx_clear(void)`
  - `void oled_gfx_pixel(int16_t x, int16_t y, uint8_t on)`
  - `void oled_gfx_bitmap(int16_t x, int16_t y, uint8_t w, uint8_t h, const uint8_t *data)` — data 为 page 格式
  - `void oled_gfx_text6x12(int16_t x, int16_t y, const char *ascii)`
  - `void oled_gfx_text16(int16_t x, int16_t y, const char *utf8)` — 字距 2 px，步进 18 px
  - `uint16_t oled_gfx_text6x12_width(const char *ascii)` — 6 px/字符
  - `uint16_t oled_gfx_text16_width(const char *utf8)` — `n*18-2`，n=0 时 0
  - `const uint8_t *oled_gfx_buffer(void)` — 1024 字节，`fb[y>>3][x]`，bit0 为页内顶行

- [ ] **Step 1: 写失败的测试**

Create `tools/test_oled_gfx.c`:

```c
#include <assert.h>
#include <stdio.h>
#include "oled_gfx.h"
#include "oled_font.h"

static int fb_bit(uint8_t x, uint8_t y)
{
  return (oled_gfx_buffer()[(y >> 3) * OLED_WIDTH + x] >> (y & 7)) & 1;
}

int main(void)
{
  int ink;
  int i;

  oled_gfx_clear();
  assert(fb_bit(0, 0) == 0);

  oled_gfx_pixel(0, 0, 1);
  assert(oled_gfx_buffer()[0] == 0x01);
  oled_gfx_pixel(127, 63, 1);
  assert(oled_gfx_buffer()[8 * 128 - 1] == 0x80);

  oled_gfx_pixel(-1, 5, 1);
  oled_gfx_pixel(200, 5, 1);
  oled_gfx_pixel(5, 64, 1);
  assert(oled_gfx_buffer()[0] == 0x01);

  /* 8×8 位图贴在 y=4：跨两页，验证按像素落位 */
  oled_gfx_clear();
  {
    static const uint8_t bmp[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    oled_gfx_bitmap(0, 4, 8, 8, bmp);
    assert(oled_gfx_buffer()[0] == 0xF0);
    assert(oled_gfx_buffer()[OLED_WIDTH] == 0x0F);
  }

  /* ASCII '0'：与生成器验证过的 page 字节一致 */
  oled_gfx_clear();
  oled_gfx_text6x12(0, 0, "0");
  {
    static const uint8_t expect0[12] = {0xF8, 0x04, 0x04, 0x04, 0xF8, 0x00,
                                        0x01, 0x02, 0x02, 0x02, 0x01, 0x00};
    int c;
    for (c = 0; c < 6; c++)
    {
      assert(oled_gfx_buffer()[c] == expect0[c]);
      assert(oled_gfx_buffer()[OLED_WIDTH + c] == expect0[6 + c]);
    }
  }

  assert(oled_gfx_text6x12_width("RF") == 12);
  assert(oled_gfx_text6x12_width("") == 0);

  /* 汉字：16 px + 2 px 字距，且有墨迹 */
  assert(oled_gfx_text16_width("电机") == 34);
  oled_gfx_clear();
  oled_gfx_text16(0, 0, "电机");
  ink = 0;
  for (i = 0; i < (int)(OLED_WIDTH * OLED_HEIGHT / 8); i++)
  {
    ink += __builtin_popcount(oled_gfx_buffer()[i]);
  }
  assert(ink > 20);

  /* 未收录的字符不崩溃，按 6 px 跳过 */
  assert(oled_gfx_text16_width("X") == 6);
  oled_gfx_clear();
  oled_gfx_text16(0, 0, "X");

  puts("oled_gfx OK");
  return 0;
}
```

- [ ] **Step 2: 运行测试确认失败**

Run: `gcc -std=c99 -Wall -Wextra -ICore/Inc tools/test_oled_gfx.c Core/Src/oled_gfx.c Core/Src/oled_font.c -o build/test_oled_gfx`
Expected: FAIL — `oled_gfx.h: No such file or directory`

- [ ] **Step 3: 实现头文件**

Create `Core/Inc/oled_gfx.h`:

```c
#ifndef OLED_GFX_H
#define OLED_GFX_H

#include <stdint.h>

#define OLED_WIDTH   128U
#define OLED_HEIGHT   64U

void oled_gfx_clear(void);
void oled_gfx_pixel(int16_t x, int16_t y, uint8_t on);
void oled_gfx_bitmap(int16_t x, int16_t y, uint8_t w, uint8_t h, const uint8_t *data);
void oled_gfx_text6x12(int16_t x, int16_t y, const char *ascii);
void oled_gfx_text16(int16_t x, int16_t y, const char *utf8);
uint16_t oled_gfx_text6x12_width(const char *ascii);
uint16_t oled_gfx_text16_width(const char *utf8);
const uint8_t *oled_gfx_buffer(void);

#endif
```

- [ ] **Step 4: 实现绘制层**

Create `Core/Src/oled_gfx.c`:

```c
#include "oled_gfx.h"

#include <string.h>

#include "oled_font.h"

#define TEXT16_STEP 18U

static uint8_t s_fb[OLED_HEIGHT / 8U][OLED_WIDTH];

void oled_gfx_clear(void)
{
  memset(s_fb, 0, sizeof(s_fb));
}

void oled_gfx_pixel(int16_t x, int16_t y, uint8_t on)
{
  if (x < 0 || y < 0 || x >= (int16_t)OLED_WIDTH || y >= (int16_t)OLED_HEIGHT)
  {
    return;
  }
  if (on)
  {
    s_fb[y >> 3][x] |= (uint8_t)(1U << (y & 7));
  }
  else
  {
    s_fb[y >> 3][x] &= (uint8_t)~(1U << (y & 7));
  }
}

void oled_gfx_bitmap(int16_t x, int16_t y, uint8_t w, uint8_t h, const uint8_t *data)
{
  uint8_t pages = (uint8_t)((h + 7U) / 8U);
  uint8_t page;
  uint8_t col;
  uint8_t bit;

  for (page = 0; page < pages; page++)
  {
    for (col = 0; col < w; col++)
    {
      uint8_t bits = data[page * w + col];
      for (bit = 0; bit < 8U; bit++)
      {
        uint8_t row = (uint8_t)(page * 8U + bit);
        if (row < h && (bits & (1U << bit)))
        {
          oled_gfx_pixel((int16_t)(x + col), (int16_t)(y + row), 1);
        }
      }
    }
  }
}

void oled_gfx_text6x12(int16_t x, int16_t y, const char *ascii)
{
  char c;

  while (*ascii != '\0')
  {
    c = *ascii++;
    if (c < ' ' || c > '~')
    {
      c = ' ';
    }
    oled_gfx_bitmap(x, y, 6, 12, oled_font_ascii[(uint8_t)c - 32U]);
    x = (int16_t)(x + 6);
  }
}

uint16_t oled_gfx_text6x12_width(const char *ascii)
{
  return (uint16_t)(strlen(ascii) * 6U);
}

void oled_gfx_text16(int16_t x, int16_t y, const char *utf8)
{
  const uint8_t *p = (const uint8_t *)utf8;

  while (*p != 0U)
  {
    if (*p < 0x80U)
    {
      p++;
      x = (int16_t)(x + 6);
    }
    else if ((*p & 0xF0U) == 0xE0U && p[1] != 0U && p[2] != 0U)
    {
      const uint8_t *glyph = oled_font_cn_lookup((const char *)p);
      if (glyph != 0)
      {
        oled_gfx_bitmap(x, y, 16, 16, glyph);
      }
      p += 3;
      x = (int16_t)(x + TEXT16_STEP);
    }
    else
    {
      p++;
    }
  }
}

uint16_t oled_gfx_text16_width(const char *utf8)
{
  const uint8_t *p = (const uint8_t *)utf8;
  uint16_t width = 0U;
  uint16_t last_step = 0U;

  while (*p != 0U)
  {
    if (*p < 0x80U)
    {
      p++;
      width = (uint16_t)(width + 6U);
      last_step = 6U;
    }
    else if ((*p & 0xF0U) == 0xE0U && p[1] != 0U && p[2] != 0U)
    {
      p += 3;
      width = (uint16_t)(width + TEXT16_STEP);
      last_step = TEXT16_STEP;
    }
    else
    {
      p++;
    }
  }
  /* 末尾字距（18 - 16）不计入宽度，便于居中 */
  if (last_step == TEXT16_STEP && width >= (uint16_t)(TEXT16_STEP - 16U))
  {
    width = (uint16_t)(width - (uint16_t)(TEXT16_STEP - 16U));
  }
  return width;
}

const uint8_t *oled_gfx_buffer(void)
{
  return &s_fb[0][0];
}
```

- [ ] **Step 5: 运行测试确认通过**

Run: `gcc -std=c99 -Wall -Wextra -ICore/Inc tools/test_oled_gfx.c Core/Src/oled_gfx.c Core/Src/oled_font.c -o build/test_oled_gfx && ./build/test_oled_gfx`
Expected: `oled_gfx OK`

- [ ] **Step 6: 提交**

```bash
git add Core/Inc/oled_gfx.h Core/Src/oled_gfx.c tools/test_oled_gfx.c
git commit -m "feat: add host-testable OLED framebuffer graphics layer"
```

---

### Task 3: OLED 硬件驱动（软 SPI）

SSD1306 复位、初始化、整屏 flush；不改 GPIO 初始化（`gpio.c` 已完成）。

**Files:**
- Create: `Core/Inc/oled.h`、`Core/Src/oled.c`
- Modify: `Makefile:38-83`（`C_SOURCES`）

**Interfaces:**
- Consumes: `oled_gfx.h`（`oled_gfx_buffer()`、`OLED_WIDTH/HEIGHT`）、`main.h` 的 `OLED_*_Pin/_GPIO_Port`
- Produces: `void oled_init(void)`、`void oled_flush(void)`；方向宏 `OLED_MIRROR_X`、`OLED_MIRROR_Y`

- [ ] **Step 1: 写头文件**

Create `Core/Inc/oled.h`:

```c
#ifndef OLED_H
#define OLED_H

#include <stdint.h>

/* 上板若画面左右/上下颠倒，只改这两个宏 */
#define OLED_MIRROR_X 1
#define OLED_MIRROR_Y 1

void oled_init(void);
void oled_flush(void);

#endif
```

- [ ] **Step 2: 写驱动实现**

Create `Core/Src/oled.c`:

```c
#include "oled.h"

#include "main.h"
#include "oled_gfx.h"

#define OLED_SCL_HIGH()  (OLED_SCL_GPIO_Port->BSRR = (uint32_t)OLED_SCL_Pin)
#define OLED_SCL_LOW()   (OLED_SCL_GPIO_Port->BSRR = ((uint32_t)OLED_SCL_Pin) << 16)
#define OLED_SDA_HIGH()  (OLED_SDA_GPIO_Port->BSRR = (uint32_t)OLED_SDA_Pin)
#define OLED_SDA_LOW()   (OLED_SDA_GPIO_Port->BSRR = ((uint32_t)OLED_SDA_Pin) << 16)
#define OLED_DC_CMD()    (OLED_DC_GPIO_Port->BSRR = ((uint32_t)OLED_DC_Pin) << 16)
#define OLED_DC_DATA()   (OLED_DC_GPIO_Port->BSRR = (uint32_t)OLED_DC_Pin)
#define OLED_RES_LOW()   (OLED_RES_GPIO_Port->BSRR = ((uint32_t)OLED_RES_Pin) << 16)
#define OLED_RES_HIGH()  (OLED_RES_GPIO_Port->BSRR = (uint32_t)OLED_RES_Pin)

static void oled_spi_byte(uint8_t value)
{
  uint8_t i;

  for (i = 0; i < 8U; i++)
  {
    OLED_SCL_LOW();
    if ((value & 0x80U) != 0U)
    {
      OLED_SDA_HIGH();
    }
    else
    {
      OLED_SDA_LOW();
    }
    value = (uint8_t)(value << 1);
    OLED_SCL_HIGH();
  }
}

static void oled_write_cmd(uint8_t cmd)
{
  OLED_DC_CMD();
  oled_spi_byte(cmd);
}

static void oled_write_data(const uint8_t *data, uint16_t len)
{
  OLED_DC_DATA();
  while (len-- > 0U)
  {
    oled_spi_byte(*data++);
  }
}

static void oled_delay(volatile uint32_t cycles)
{
  while (cycles-- > 0U)
  {
  }
}

static void oled_hw_reset(void)
{
  OLED_RES_LOW();
  oled_delay(40000U);   /* ≈ 1 ms @168 MHz */
  OLED_RES_HIGH();
  oled_delay(400000U);  /* ≈ 10 ms，等电荷泵稳定 */
}

void oled_init(void)
{
  oled_hw_reset();

  oled_write_cmd(0xAE);        /* 关显示 */
  oled_write_cmd(0xD5); oled_write_cmd(0x80);
  oled_write_cmd(0xA8); oled_write_cmd(0x3F);
  oled_write_cmd(0xD3); oled_write_cmd(0x00);
  oled_write_cmd(0x40);        /* 起始行 0 */
  oled_write_cmd(0x8D); oled_write_cmd(0x14);  /* 电荷泵 */
  oled_write_cmd(0x20); oled_write_cmd(0x00);  /* 水平寻址 */
#if OLED_MIRROR_X
  oled_write_cmd(0xA1);
#else
  oled_write_cmd(0xA0);
#endif
#if OLED_MIRROR_Y
  oled_write_cmd(0xC8);
#else
  oled_write_cmd(0xC0);
#endif
  oled_write_cmd(0xDA); oled_write_cmd(0x12);
  oled_write_cmd(0x81); oled_write_cmd(0xCF);
  oled_write_cmd(0xD9); oled_write_cmd(0xF1);
  oled_write_cmd(0xDB); oled_write_cmd(0x40);
  oled_write_cmd(0xA4);
  oled_write_cmd(0xA6);
  oled_write_cmd(0x2E);        /* 关滚动 */
  oled_write_cmd(0xAF);        /* 开显示 */

  oled_gfx_clear();
  oled_flush();
}

void oled_flush(void)
{
  oled_write_cmd(0x21); oled_write_cmd(0x00); oled_write_cmd(0x7F);
  oled_write_cmd(0x22); oled_write_cmd(0x00); oled_write_cmd(0x07);
  oled_write_data(oled_gfx_buffer(), (uint16_t)(OLED_WIDTH * OLED_HEIGHT / 8U));
}
```

- [ ] **Step 3: 加入 Makefile**

在 `C_SOURCES` 的 `Core/Src/tim.c \` 之后插入三行：

```make
Core/Src/oled.c \
Core/Src/oled_gfx.c \
Core/Src/oled_font.c \
```
注意：`boot_ui.c` 在 Task 4 才创建，本步不要加，否则编译失败。

- [ ] **Step 4: 编译验证**

Run: `source /mnt/data/project-ai/Embedded_Development/toolchain/env.sh && make -j8 2>&1 | tail -20`
Expected: 编译成功，最后打印 `arm-none-eabi-size` 三行（text/data/bss），无 warning

- [ ] **Step 5: 提交**

```bash
git add Core/Inc/oled.h Core/Src/oled.c Makefile
git commit -m "feat: add SSD1306 software-SPI driver and register new sources"
```

---

### Task 4: boot_ui 状态机与 FreeRTOS 集成

电压/转速换算先做主机测试，再写硬件相关状态机。

**Files:**
- Create: `Core/Inc/boot_ui.h`、`Core/Src/boot_ui.c`
- Test: `tools/test_boot_math.c`
- Modify: `Core/Src/freertos.c:44-53`（栈大小）、`:112-118`（Includes）、`:118-124`（FunctionPrototypes 保持）、`:178-238`（任务函数体）
- Modify: `Makefile`（加 `Core/Src/boot_ui.c`）

**Interfaces:**
- Consumes: `oled.h`、`oled_gfx.h`、`oled_font.h`、`main.h`（`MOTOR_RF_*`、`SELFCHECK_*`、`KEY_USER_*`）、`tim.h`（`htim2..5`、`htim8`）、`adc.h`（`hadc1`）
- Produces:
  - `void boot_ui_run(void)` — 永不返回
  - `static inline int32_t boot_ui_rpm_from_counts(int32_t counts_100ms, uint32_t cpr)`
  - `static inline uint32_t boot_ui_vdd_mv(uint32_t raw_avg, uint16_t vrefint_cal)`
  - 宏 `WHEEL_CPR 54000U`、`BOOT_UI_VDD_MIN_MV 3100U`

- [ ] **Step 1: 写失败的换算测试**

Create `tools/test_boot_math.c`:

```c
#include <assert.h>
#include <stdio.h>

#include "boot_ui.h"

int main(void)
{
  assert(WHEEL_CPR == 54000U);
  assert(BOOT_UI_VDD_MIN_MV == 3100U);

  assert(boot_ui_rpm_from_counts(9000, WHEEL_CPR) == 100);
  assert(boot_ui_rpm_from_counts(-9000, WHEEL_CPR) == -100);
  assert(boot_ui_rpm_from_counts(27000, WHEEL_CPR) == 300);
  assert(boot_ui_rpm_from_counts(0, WHEEL_CPR) == 0);
  assert(boot_ui_rpm_from_counts(12345, 0U) == 0);

  assert(boot_ui_vdd_mv(1490U, 1490U) == 3300U);
  assert(boot_ui_vdd_mv(1600U, 1490U) == 3073U);
  assert(boot_ui_vdd_mv(1400U, 1490U) == 3512U);
  assert(boot_ui_vdd_mv(0U, 1490U) == 0U);
  assert(boot_ui_vdd_mv(1500U, 0U) == 3278U);

  puts("boot math OK");
  return 0;
}
```

- [ ] **Step 2: 运行测试确认失败**

Run: `gcc -std=c99 -Wall -Wextra -ICore/Inc tools/test_boot_math.c -o build/test_boot_math`
Expected: FAIL — `boot_ui.h: No such file or directory`

- [ ] **Step 3: 写头文件**

Create `Core/Inc/boot_ui.h`:

```c
#ifndef BOOT_UI_H
#define BOOT_UI_H

#include <stdint.h>

/* 编码器一圈计数：4 倍频 × 500 线 × 27 减速比（MD36N_27 + GMR_500） */
#define WHEEL_CPR 54000U

/* VDD 通过阈值（mV）。3.3 V 轨实测约 3.28–3.30 V，按字面 >3300 会误判 */
#define BOOT_UI_VDD_MIN_MV 3100U

/* 转速采样：每 20 ms 读一次，累计 5 次得 100 ms 窗口 */
#define BOOT_UI_ENC_SAMPLE_MS 20U
#define BOOT_UI_ENC_SAMPLES    5U

/* 主页面 VDD 刷新间隔 */
#define BOOT_UI_VDD_REFRESH_MS 500U

static inline int32_t boot_ui_rpm_from_counts(int32_t counts_100ms, uint32_t cpr)
{
  if (cpr == 0U)
  {
    return 0;
  }
  return (counts_100ms * 600) / (int32_t)cpr;
}

static inline uint32_t boot_ui_vdd_mv(uint32_t raw_avg, uint16_t vrefint_cal)
{
  if (vrefint_cal == 0U || vrefint_cal == 0xFFFFU)
  {
    vrefint_cal = 1490U;
  }
  if (raw_avg == 0U)
  {
    return 0U;
  }
  return (3300U * (uint32_t)vrefint_cal) / raw_avg;
}

void boot_ui_run(void);

#endif
```

- [ ] **Step 4: 运行测试确认通过**

Run: `gcc -std=c99 -Wall -Wextra -ICore/Inc tools/test_boot_math.c -o build/test_boot_math && ./build/test_boot_math`
Expected: `boot math OK`

- [ ] **Step 5: 写状态机实现**

Create `Core/Src/boot_ui.c`:

```c
#include "boot_ui.h"

#include "adc.h"
#include "cmsis_os.h"
#include "main.h"
#include "oled.h"
#include "oled_font.h"
#include "oled_gfx.h"
#include "tim.h"

#define UI_BLINK_MS        500U
#define UI_KEY_POLL_MS      10U
#define UI_LOADING_MS     1000U
#define UI_START_MS        800U
#define UI_SPINNER_MS      125U
#define UI_VDD_RETRY         3U
#define UI_VDD_RETRY_MS    500U
#define UI_MAIN_HOLD_MS    600U

#define UI_ROW_MOTOR_Y      22
#define UI_ROW_VDD_Y        42
#define UI_ROW_SYMBOL_X     78

#define UI_STATE_RUN   0U
#define UI_STATE_OK    1U
#define UI_STATE_FAIL  2U

#define UI_TEXT6_WIDTH     6U
#define UI_TEXT16_STEP    18U

#define VREFINT_CAL_ADDR 0x1FFF7A2AU

typedef struct
{
  TIM_HandleTypeDef *tim;
  const char *name;
} ui_wheel_t;

static const ui_wheel_t s_wheels[4] =
{
  { &htim2, "RF" },
  { &htim3, "LF" },
  { &htim4, "RR" },
  { &htim5, "LR" },
};

static void ui_text16_center(int16_t y, const char *text);
static void ui_text6_center(int16_t y, const char *text);
static void ui_checklist(uint8_t motor_state, uint8_t vdd_state, uint8_t frame);
static void ui_hold(uint32_t ms, uint8_t vdd_state, uint8_t *frame);
static void ui_motor_selfcheck(uint8_t *frame);
static uint8_t ui_vdd_check(uint32_t *vdd_mv, uint8_t *frame);
static uint32_t ui_read_vdd_mv(void);
static void ui_main_page(void);
static void ui_format_vdd(char *buf, uint32_t mv);
static void ui_format_rpm(char *buf, const char *name, int32_t rpm);

static void ui_text16_center(int16_t y, const char *text)
{
  uint16_t width = oled_gfx_text16_width(text);
  oled_gfx_text16((int16_t)((OLED_WIDTH - width) / 2U), y, text);
}

static void ui_text6_center(int16_t y, const char *text)
{
  uint16_t width = oled_gfx_text6x12_width(text);
  oled_gfx_text6x12((int16_t)((OLED_WIDTH - width) / 2U), y, text);
}

static void ui_row_symbol(int16_t y, uint8_t state, uint8_t frame)
{
  if (state == UI_STATE_RUN)
  {
    oled_gfx_bitmap(UI_ROW_SYMBOL_X, (int16_t)(y + 2), 12, 12,
                    oled_font_spinner[frame & 7U]);
  }
  else if (state == UI_STATE_OK)
  {
    oled_gfx_bitmap(UI_ROW_SYMBOL_X, y, 16, 16, oled_font_check);
  }
  else
  {
    oled_gfx_bitmap(UI_ROW_SYMBOL_X, y, 16, 16, oled_font_cross);
  }
}

static void ui_checklist(uint8_t motor_state, uint8_t vdd_state, uint8_t frame)
{
  oled_gfx_clear();
  ui_text16_center(0, "自检流程开始");
  oled_gfx_text16(0, UI_ROW_MOTOR_Y, "电机自检");
  oled_gfx_text16(0, UI_ROW_VDD_Y, "电池自检");
  ui_row_symbol(UI_ROW_MOTOR_Y, motor_state, frame);
  ui_row_symbol(UI_ROW_VDD_Y, vdd_state, frame);
  oled_flush();
}

static void ui_hold(uint32_t ms, uint8_t vdd_state, uint8_t *frame)
{
  while (ms > 0U)
  {
    ui_checklist(UI_STATE_RUN, vdd_state, (*frame)++);
    if (ms > UI_SPINNER_MS)
    {
      osDelay(UI_SPINNER_MS);
      ms -= UI_SPINNER_MS;
    }
    else
    {
      osDelay(ms);
      ms = 0U;
    }
  }
}

static void ui_wait_key(void)
{
  uint8_t blink = 1U;
  uint8_t i;

  for (;;)
  {
    oled_gfx_clear();
    ui_text16_center(16, "等待初始化");
    if (blink != 0U)
    {
      ui_text6_center(44, "PRESS KEY");
    }
    oled_flush();
    blink = (uint8_t)!blink;

    for (i = 0; i < (uint8_t)(UI_BLINK_MS / UI_KEY_POLL_MS); i++)
    {
      if (HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin) == GPIO_PIN_RESET)
      {
        osDelay(UI_KEY_POLL_MS);
        if (HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin) == GPIO_PIN_RESET)
        {
          return;
        }
      }
      osDelay(UI_KEY_POLL_MS);
    }
  }
}

static void ui_loading(void)
{
  uint8_t frame = 0U;
  uint32_t elapsed;
  uint16_t width;

  for (elapsed = 0U; elapsed < UI_LOADING_MS; elapsed += UI_SPINNER_MS)
  {
    oled_gfx_clear();
    ui_text16_center(24, "正在加载");
    width = oled_gfx_text16_width("正在加载");
    oled_gfx_bitmap((int16_t)((OLED_WIDTH - width) / 2U + width + 8U), 26,
                    12, 12, oled_font_spinner[frame & 7U]);
    oled_flush();
    frame++;
    osDelay(UI_SPINNER_MS);
  }

  oled_gfx_clear();
  ui_text16_center(24, "自检流程开始");
  oled_flush();
  osDelay(UI_START_MS);
}

static void ui_motor_selfcheck(uint8_t *frame)
{
  uint8_t cycle;

  HAL_GPIO_WritePin(MOTOR_RF_SLP_GPIO_Port, MOTOR_RF_SLP_Pin, GPIO_PIN_SET);
  osDelay(10U);

  for (cycle = 0U; cycle < SELFCHECK_CYCLES; cycle++)
  {
    HAL_GPIO_WritePin(MOTOR_RF_DIR_GPIO_Port, MOTOR_RF_DIR_Pin, MOTOR_RF_DIR_FORWARD);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, SELFCHECK_PWM_PULSE);
    ui_hold(SELFCHECK_RUN_MS, UI_STATE_RUN, frame);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0U);
    ui_hold(SELFCHECK_PAUSE_MS, UI_STATE_RUN, frame);

    HAL_GPIO_WritePin(MOTOR_RF_DIR_GPIO_Port, MOTOR_RF_DIR_Pin, MOTOR_RF_DIR_REVERSE);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, SELFCHECK_PWM_PULSE);
    ui_hold(SELFCHECK_RUN_MS, UI_STATE_RUN, frame);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0U);
    if (cycle < (SELFCHECK_CYCLES - 1U))
    {
      ui_hold(SELFCHECK_PAUSE_MS, UI_STATE_RUN, frame);
    }
  }

  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0U);
  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
  HAL_GPIO_WritePin(MOTOR_RF_SLP_GPIO_Port, MOTOR_RF_SLP_Pin, GPIO_PIN_RESET);
}

static uint32_t ui_read_vdd_mv(void)
{
  static uint16_t s_cal = 0U;
  ADC_ChannelConfTypeDef cfg = {0};
  uint32_t sum = 0U;
  uint8_t i;

  if (s_cal == 0U)
  {
    s_cal = *(const volatile uint16_t *)VREFINT_CAL_ADDR;
  }

  cfg.Channel = ADC_CHANNEL_VREFINT;
  cfg.Rank = 1U;
  cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK)
  {
    return 0U;
  }

  for (i = 0U; i < 8U; i++)
  {
    if (HAL_ADC_Start(&hadc1) == HAL_OK)
    {
      if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK)
      {
        sum += HAL_ADC_GetValue(&hadc1);
      }
      HAL_ADC_Stop(&hadc1);
    }
  }

  /* 恢复电池分压通道，保持 ADC 状态与初始化后一致 */
  cfg.Channel = ADC_CHANNEL_8;
  (void)HAL_ADC_ConfigChannel(&hadc1, &cfg);

  return boot_ui_vdd_mv(sum / 8U, s_cal);
}

static uint8_t ui_vdd_check(uint32_t *vdd_mv, uint8_t *frame)
{
  uint8_t attempt;
  uint32_t value = 0U;

  for (attempt = 0U; attempt < UI_VDD_RETRY; attempt++)
  {
    ui_checklist(UI_STATE_OK, UI_STATE_RUN, (*frame)++);
    value = ui_read_vdd_mv();
    if (value >= BOOT_UI_VDD_MIN_MV)
    {
      *vdd_mv = value;
      return 1U;
    }
    osDelay(UI_VDD_RETRY_MS);
  }
  *vdd_mv = value;
  return 0U;
}

static void ui_format_vdd(char *buf, uint32_t mv)
{
  uint32_t cv = mv / 10U;

  if (cv > 9999U)
  {
    cv = 9999U;
  }
  buf[0] = 'V';
  buf[1] = 'D';
  buf[2] = 'D';
  buf[3] = ' ';
  buf[4] = (char)('0' + (cv / 100U) % 10U);
  buf[5] = '.';
  buf[6] = (char)('0' + (cv / 10U) % 10U);
  buf[7] = (char)('0' + cv % 10U);
  buf[8] = 'V';
  buf[9] = '\0';
}

static void ui_format_rpm(char *buf, const char *name, int32_t rpm)
{
  uint32_t mag = (rpm < 0) ? (uint32_t)(-rpm) : (uint32_t)rpm;

  if (mag > 9999U)
  {
    mag = 9999U;
  }
  buf[0] = name[0];
  buf[1] = name[1];
  buf[2] = ' ';
  buf[3] = (rpm < 0) ? '-' : '+';
  buf[4] = (char)('0' + (mag / 1000U) % 10U);
  buf[5] = (char)('0' + (mag / 100U) % 10U);
  buf[6] = (char)('0' + (mag / 10U) % 10U);
  buf[7] = (char)('0' + mag % 10U);
  buf[8] = ' ';
  buf[9] = 'r';
  buf[10] = 'p';
  buf[11] = 'm';
  buf[12] = '\0';
}

static void ui_main_page(void)
{
  uint16_t prev[4];
  int32_t sum[4];
  char line[20];
  uint32_t vdd_mv = 0U;
  uint32_t since_vdd = BOOT_UI_VDD_REFRESH_MS;
  uint8_t w;
  uint8_t s;

  for (w = 0U; w < 4U; w++)
  {
    prev[w] = (uint16_t)__HAL_TIM_GET_COUNTER(s_wheels[w].tim);
    sum[w] = 0;
  }

  for (;;)
  {
    for (w = 0U; w < 4U; w++)
    {
      sum[w] = 0;
    }

    for (s = 0U; s < BOOT_UI_ENC_SAMPLES; s++)
    {
      osDelay(BOOT_UI_ENC_SAMPLE_MS);
      for (w = 0U; w < 4U; w++)
      {
        uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(s_wheels[w].tim);
        sum[w] += (int32_t)(int16_t)(uint16_t)(now - prev[w]);
        prev[w] = now;
      }
    }

    if (since_vdd >= BOOT_UI_VDD_REFRESH_MS)
    {
      vdd_mv = ui_read_vdd_mv();
      since_vdd = 0U;
    }
    since_vdd += (uint32_t)(BOOT_UI_ENC_SAMPLE_MS * BOOT_UI_ENC_SAMPLES);

    oled_gfx_clear();
    ui_format_vdd(line, vdd_mv);
    oled_gfx_text6x12(0, 0, line);

    for (w = 0U; w < 4U; w++)
    {
      ui_format_rpm(line, s_wheels[w].name,
                    boot_ui_rpm_from_counts(sum[w], WHEEL_CPR));
      oled_gfx_text6x12(0, (int16_t)(12 * (w + 1U)), line);
    }
    oled_flush();
  }
}

void boot_ui_run(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t frame = 0U;
  uint32_t vdd_mv = 0U;

  /* A 轮 SLP/EN (PB12)：.ioc 未配置，手动初始化，低电平 = 驱动睡眠 */
  HAL_GPIO_WritePin(MOTOR_RF_SLP_GPIO_Port, MOTOR_RF_SLP_Pin, GPIO_PIN_RESET);
  gpio.Pin = MOTOR_RF_SLP_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_RF_SLP_GPIO_Port, &gpio);

  /* 上电即置 MOE，但占空比 0，电机不动 */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0U);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  oled_init();

  ui_wait_key();
  ui_loading();

  ui_checklist(UI_STATE_RUN, UI_STATE_RUN, frame++);
  ui_motor_selfcheck(&frame);
  ui_checklist(UI_STATE_OK, UI_STATE_RUN, frame++);

  if (ui_vdd_check(&vdd_mv, &frame) == 0U)
  {
    ui_checklist(UI_STATE_OK, UI_STATE_FAIL, frame);
    for (;;)
    {
      osDelay(1000U);
    }
  }

  ui_checklist(UI_STATE_OK, UI_STATE_OK, frame);
  osDelay(UI_MAIN_HOLD_MS);
  ui_main_page();
}
```

- [ ] **Step 6: 集成到 FreeRTOS**

修改 `Core/Src/freertos.c`：

1. `USER CODE BEGIN Includes` 中把 `#include "tim.h"` 换成 `#include "boot_ui.h"`（tim.h 由 boot_ui.c 自己包含）。
2. `selfCheckTask_attributes` 的 `.stack_size` 从 `256 * 4` 改为 `512 * 4`。
3. `USER CODE BEGIN Application` 里的整个 `StartSelfCheckTask` 函数体替换为：

```c
/* 开机自检引导 UI：等待按键 → 加载动画 → 电机/电压自检 → 主页面 */
void StartSelfCheckTask(void *argument)
{
  (void)argument;
  boot_ui_run();
}
```
4. 删除原来函数内的注释块和局部变量（`cycle`、`GPIO_InitStruct`），并删除 `/* USER CODE BEGIN Application */` 中与新函数体无关的旧注释。

- [ ] **Step 7: 加入 Makefile**

在 `C_SOURCES` 中（Task 3 已加的三行之后）加入：

```make
Core/Src/boot_ui.c \
```

- [ ] **Step 8: 编译验证**

Run: `source /mnt/data/project-ai/Embedded_Development/toolchain/env.sh && make -j8 2>&1 | tail -25`
Expected: 编译成功，无 warning；`arm-none-eabi-size` 显示 text 增加约 3–4 KB（字库 + 代码）

- [ ] **Step 9: 复查旧逻辑已完全移除**

Run: `grep -n "HAL_GPIO_Init\|SELFCHECK_CYCLES\|osThreadTerminate" Core/Src/freertos.c`
Expected: 无输出（自检逻辑已全部搬到 boot_ui.c；`osThreadTerminate` 不再需要）

- [ ] **Step 10: 提交**

```bash
git add Core/Inc/boot_ui.h Core/Src/boot_ui.c Core/Src/freertos.c tools/test_boot_math.c Makefile
git commit -m "feat: add OLED boot UI state machine with VDD check and wheel RPM page"
```

---

### Task 5: 烧录与上板验证

硬件行为人工核对；发现方向/符号问题只改宏或映射表，不改流程。

**Files:**
- Modify（如需）: `Core/Inc/oled.h`（`OLED_MIRROR_X/Y`）、`Core/Inc/boot_ui.h`（`WHEEL_CPR`）、`Core/Src/boot_ui.c`（`s_wheels` 顺序）

**Interfaces:**
- Consumes: Task 1–4 的全部产物
- Produces: 通过验证的固件

- [ ] **Step 1: 编译并烧录**

```bash
source /mnt/data/project-ai/Embedded_Development/toolchain/env.sh
make -j8
ocd-flash build/hal_f407_mecanum_4motor_src.elf
```
Expected: OpenOCD 输出 `** Programming Finished **`、`** Verified OK **`、`** Resetting Target **`

- [ ] **Step 2: 上电检查屏幕 ①**

Expected: 显示「等待初始化」+ 闪烁的「PRESS KEY」，字迹正立、不镜像。
若左右镜像 → `OLED_MIRROR_X` 改 0；上下颠倒 → `OLED_MIRROR_Y` 改 0；重新编译烧录。

- [ ] **Step 3: 按键进入自检**

Expected: 按 KEY_USER 后出现「正在加载」+ 转圈 1 s → 「自检流程开始」0.8 s → 自检列表（电机行转圈、电池行转圈）。

- [ ] **Step 4: 确认电机时序**

Expected: A 轮（右前）正转 3 s → 停 3 s → 反转 3 s → 停 3 s，共两轮 24 s；期间转圈动画连续；结束后该行变 ✓ 且驱动器回睡眠（电机松手可转）。

- [ ] **Step 5: 确认电压自检**

Expected: 电机打勾后电池行很快变 ✓（<0.1 s）；主页面出现。

- [ ] **Step 6: 核对主页面电压**

Expected: 第一行 `VDD 3.2xV`–`3.3xV`，与万用表量 3.3 V 轨相差 ≤0.05 V。

- [ ] **Step 7: 核对四轮转速**

手转右前轮一圈（慢慢转），Expected: 主页面 `RF` 行出现非零值且符号随方向翻转；另外三个轮子不动时保持 `+0000`。
若行与轮子不对应 → 调整 `s_wheels` 顺序；若数值约为实际 1/2 或 2 倍 → 调整 `WHEEL_CPR`。

- [ ] **Step 8: 一圈计数复核（可选）**

用调试器/串口读 TIM2 CNT，手转一圈前后差值 Expected ≈ 54000（±5%）。偏差大说明电机是别的减速比/线数，按实测改 `WHEEL_CPR`。

- [ ] **Step 9: 长时间观察**

Expected: 主页面 10 Hz 刷新无闪烁、无残影；静置 1 分钟数值稳定（±1）。

- [ ] **Step 10: 提交上板修正（如有）**

```bash
git add -A Core/Inc/oled.h Core/Inc/boot_ui.h Core/Src/boot_ui.c
git commit -m "fix: correct OLED orientation and wheel mapping after bench test"
```

---

## 自检清单（规格覆盖）

| 规格条目 | 对应任务 |
|---|---|
| 屏幕 ① 等待初始化 + 闪烁提示 + 去抖 | Task 4 `ui_wait_key` |
| 屏幕 ② 加载动画 1 s + 自检流程开始 0.8 s | Task 4 `ui_loading` |
| 屏幕 ③ 两项自检 + 转圈 + ✓ | Task 4 `ui_checklist` / `ui_motor_selfcheck` / `ui_vdd_check` |
| 电机 24 s（3/3/3/3 ×2）、50%、SLP 时序 | Task 4 `ui_motor_selfcheck` |
| VDD 用 VREFINT、8 次平均、≥3100 mV、失败重试 3 次后停留 | Task 4 `ui_read_vdd_mv` / `ui_vdd_check` |
| 主页面 VDD + 四轮 RPM、10 Hz | Task 4 `ui_main_page` |
| 6×12 ASCII（实测位序） | Task 1 `ascii_to_page` + Task 2 |
| 17 个汉字旋转修正 | Task 1 `rot90cw` / `page_encode` |
| 转圈 8 帧、✓、✗ | Task 1 `spinner_frames` / `CHECK_16` / `CROSS_16` |
| 软 SPI、无 CS、方向宏 | Task 3 |
| 编码器启动、int16 差值、CPR 54000 | Task 4 `ui_main_page` / `boot_ui.h` |
| Makefile 注册新源文件 | Task 3/4 |
| 主机测试 | Task 1/2/4 的 test 文件 |
