# 字体粗细修正 + 驾驶 PWM 上限动态设置 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** ①2026-11-01 上板反馈：新增 60 个 Noto 阈值化字形偏粗且彼此粗细不一，按原有 17 字模的墨水密度自动匹配阈值重新生成，并提供主机端 PNG 预览；②新增"输出上限"子菜单页，运行时动态设置驾驶 PWM 上限（10%~50%，RAM 生效，断电回默认）。

**Architecture:** 字体侧改 tools/font_add_glyphs.py 的渲染-阈值管道（按 ink-ratio 目标自动选阈值）+ 新增 tools/render_preview.py（oled_gfx 帧缓冲 → PNG）；PWM 侧 drive_ctrl 增加运行时上限变量与 setter，ui_model 增加第 5 个子菜单条目与调整页，freertos 接线。

**Tech Stack:** 同主计划；预览脚本用 Pillow（已是 font_add_glyphs.py 依赖）。

**Spec:** docs/superpowers/specs/2026-10-31-oled-menu-driving-design.md §5.3"输出上限"条目（已更新）。

## Global Constraints

- PWM 上限：**百分比一律为占空比口径（pct×168 = pct% × (ARR+1=16800)/100）：50% = 8400 = 现状满输出 = 上电默认**。setter 夹取 [840, 8400] 不变（840 = 5% 占空比为绝对下夹取点，页面档位 10% ↔ 1680）；只影响驾驶混合（drive_ctrl_mix 与其内部 map_axis 调用）；SELFCHECK_PWM_PULSE(8400)、WHEEL_HOME_MAX_PULSE(3360) 不受影响；存 RAM 不写 Flash。
- 调整页交互：摇杆上下 ±5% 步进（占空比口径 = ±840），阈值 48/24 与 400/180ms 连移沿用；L1 返回子菜单（原条目），R1 同；改动即时生效（改的是全局运行时变量，无"保存"动作）。
- 字形：全部仍经 font_convert.py extra 路径生成；既有 17 个原始字模文件不得修改；重新生成后 oled_font.c 中原始 17 字形与 ascii/title 段逐字节不变。
- 预览图：tools/render_preview.py 渲染全部中文字形栅格 + 品牌页/确认页/子菜单三页到 PNG，供上板前人工目检。
- 回归门槛：bash tools/test_motion.sh 全绿、bash tools/test_boot_ctrl_integration.sh 7/7、python3 tools/test_font_convert.py 通过、ARM make 零告警并更新构建指纹。

## Review Focus

1. 上限运行时变量的并发访问：Ctrl 任务单写（设置页请求经 req），drive_ctrl 消费在同任务——无跨任务竞争；但 snapshot/渲染读百分比时用 seq-lock 快照字段，不裸读全局。
2. 上限变更瞬间正在输出的驾驶命令：下一控制周期按新上限重新映射（mix 每拍重算，天然生效），不得出现超过新上限的残留输出——测试钉住。
3. 范围夹取：摇杆连移快速越过边界时停在 10%/50%，不回绕、不溢出（int32）。
4. 字体阈值匹配：新字形 ink-ratio 与原始 17 字形均值偏差 ≤15%，且彼此极差 ≤20%（预览图 + 程序化断言双验证）。
5. 第 5 子菜单条目进入后：滚动窗口/滚动标记/焦点记忆仍正确（4→5 条目的窗口逻辑回归）。

---

### Task 1: 字体粗细匹配 + 主机预览工具

**Files:**
- Modify: `tools/font_add_glyphs.py`（阈值自动匹配）、`tools/font_convert.py`（如需）、`tools/font_src/CNADD2_16x16.h`（重新生成）
- Create: `tools/render_preview.py`
- Test: `tools/test_font_convert.py`

**Interfaces:**
- Produces: 重新生成的 oled_font.c（60 个新字形墨水密度匹配原始集）、`tools/render_preview.py glyphs out.png` 与 `tools/render_preview.py page brand|confirm|submenu out.png`（链接 oled_gfx + ui_render 的主机构建，或直接读字模自绘——实现者选最省方案并报告）。
- 墨水密度目标：原始 17 字形（oled_font_cn 前 17 项）每字 ink/256 比值的均值 μ 与极差；新字形逐字调节阈值使 ink 落在 [0.85μ, 1.15μ]；全部新字形极差 ≤0.20μ。若某字无法达标记 fail 并报告。

- [ ] **Step 1: 写失败测试**（test_font_convert.py）：新字形 ink 比值全部在 [0.85μ, 1.15μ] 且极差 ≤0.20μ；重新生成幂等。
- [ ] **Step 2: 跑失败**。
- [ ] **Step 3: 实现**（font_add_glyphs.py 对每字在多档阈值/权重下渲染并选最接近目标者；重生成 CNADD2_16x16.h → font_convert.py → oled_font.c）+ render_preview.py + 三张预览 PNG 输出到 docs/superpowers/plans/preview-*.png。
- [ ] **Step 4: 跑通过**（含 bash tools/test_motion.sh 回归——oled_font.c 变更不破坏 ui_render 断言）。
- [ ] **Step 5: Commit**：`feat: match added glyph stroke weight to original set and add png preview`

### Task 2: 输出上限动态设置

**Files:**
- Modify: `Core/Inc/drive_ctrl.h`、`Core/Src/drive_ctrl.c`、`Core/Inc/ui_model.h`、`Core/Src/ui_model.c`、`Core/Inc/ui_render.h`、`Core/Src/ui_render.c`、`Core/Src/freertos.c`
- Test: `tools/test_drive_ctrl.c`、`tools/test_ui_model.c`、`tools/test_ctrl_task.c`

**Interfaces:**
- drive_ctrl：`extern int32_t drive_ctrl_max_pulse;`（默认 DRIVE_CTRL_MAX_PULSE=8400）+ `void drive_ctrl_set_max_pulse(int32_t v);`（夹取 [840, 8400]）；`drive_ctrl_map`/`drive_ctrl_mix`/`map_axis` 全部改读运行时变量；转向 limit = drive_ctrl_max_pulse/2（int32 整除）。
- ui_model：`UI_SUB_OUTPUT_CAP=4`（第 5 条目）；新页 `UI_PAGE_OUTPUT_CAP`；子菜单 L1 进入；页内摇杆上下 ±5%（engage/release/连移沿用）；L1/R1 回子菜单原条目；快照字段 `uint8_t output_cap_pct;`（渲染与集成测试消费）。
- freertos：设置页把模型产出的新值（req 单发口径：进页/实际步进当拍置位）落到 `drive_ctrl_set_max_pulse(pct × 168)`（pct 为占空比百分比：50% = 8400 = 现状满输出 = 默认，±5% 步进 = ±840；fix round 1 修正，初稿 ×84/420 为 8400 口径误写）；快照发布 output_cap_pct。
- 渲染：新页显示 `输出上限`（16px）+ `NN%`（6×12 大数字区）+ 原始比较值 + 提示 `上下调整  L1返回`（中文，字形已有则用，缺则加进 Task 1 字形管道后统一生成——若 Task 1 先行完成则直接用新管道补字）。

- [x] **Step 1: 写失败测试**：drive_ctrl——set_max_pulse 夹取边界（839→840、9000→8400）、mix 在 50% 上限下满杆=8400（占空比口径，fix round 1 修正：初稿误写 4200/×84）、上限变更后下一拍 mix 立即按新值、死区/对称不变；ui_model——子菜单第 5 条目导航/滚动窗口回归、进入/返回、±5% 步进与边界夹取、连移节奏；test_ctrl_task——设置页调整后四轮 compare 反映新上限、snapshot output_cap_pct 正确。
- [x] **Step 2: 跑失败**。
- [x] **Step 3: 实现**。
- [x] **Step 4: 跑通过**（bash tools/test_motion.sh + test_boot_ctrl_integration.sh 7/7 + ARM make 零告警 + 构建指纹更新到本 plan 末尾）。
- [x] **Step 5: Commit**：`feat: runtime drive pwm output cap with settings page`

---

## 构建记录（Task 2 Step 4，2026-11-01）

- 工具链：Makefile 自带 `arm-none-eabi-gcc`（-Og -Wall，零告警零错误）。
- `text 63128 / data 132 / bss 54496`（较 Task 1 基线 +3416 text：新页面 + 8 字形 + 运行时上限）。
- `build/hal_f407_mecanum_4motor_src.bin`：size 63268 bytes，
  sha256 `7cfd41cc80eeb50513f6f42c3ab97eff044085e83043447df7290f83b53ed5b8`。
- 回归：`bash tools/test_motion.sh` 全绿（新增 ctrl 'o' 输出上限场景）、
  `bash tools/test_boot_ctrl_integration.sh` 7/7 PASS、`python3 tools/test_font_convert.py` 全过
  （96 字形墨水密度带内、CNADD3 幂等）。
- 预览：`docs/superpowers/plans/preview-output-cap.png`（新页目检）、
  `preview-glyphs.png` 重生成（96 字形）、`preview-submenu.png` 重生成（5 条目）。
- 渲染参数：新增 8 字形 输出上限下调整返（CNADD3_16x16.h）——
  输 DemiLight/30/158、出 DemiLight/28/128、上 Bold/32/160、限 Light/30/134、
  下 Medium/32/110、调 Light/32/118、整 Light/34/146、返 Light/32/122；
  ink 全部 65~66 px（ratio 0.2539/0.2578），带内极差 0.0039 ≪ 0.20μ。

### Fix round 1（2026-11-01 裁决修正：占空比口径）

- 修正：百分比换算 pct×84 → **pct×168**（占空比口径）：50% 档 = 8400 = 现状
  满输出 = 上电默认；页面档位 10%~50% ↔ [1680, 8400]；±5% 步进 = ±840；
  setter 夹取 [840, 8400] 不变（840 = 5% 占空比，绝对下夹取点）。
- 同步：test_drive_ctrl（50% 档满杆=8400）、test_ctrl_task（40% → compare 6720）、
  ui_model.h/freertos/ui_render 注释与换算；plan 接口段与 Global Constraints 更正。
- 重跑全绿：test_motion.sh 全绿、boot_ctrl 7/7、test_font_convert.py 全过、
  ARM make 零告警。
- `build/hal_f407_mecanum_4motor_src.bin`（fix 后）：size 63268 bytes，
  sha256 `545ed202368a1b645a2d56098eada7c2e46aeb1370dc8f43645ab443c2c15fea`
  （text/data/bss 不变，63128/132/54496）。
