# 自检 5 秒倒计时启动 + 全界面中文化 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 按用户 2026-11-01 上板反馈改造：①自检启动改为 USER 触发的 5 秒倒计时（USER 再按取消、L1/摇杆右立即开始、前置不满足显示原因）；②除仪表盘短代码外全部界面中文化。

**Architecture:** ui_model 的 CHECK_CONFIRM 页改为带倒计时的确认状态（模型内计时，Ctrl 回喂 `selfcheck_ready`）；ui_render 全部英文占位替换为 16×16 汉字（经 tools/font_convert.py 增补字形）；仪表盘顶栏短代码（FWD/STOP/REF/FAULT 等）与品牌副标题保留英文。

**Tech Stack:** 同主计划（C11/FreeRTOS/主机 gcc 测试）。

**Spec:** `docs/superpowers/specs/2026-10-31-oled-menu-driving-design.md` §1（中文化范围）与 §4（倒计时启动流程，已更新）。

## Global Constraints

- 倒计时时长 5000 ms；归零自动发 `selfcheck_start`；启动前置（frame_valid、四轴回中、vdd_ok、非标定模式）不满足时倒计时暂停并显示原因，恢复后继续走完剩余时间。
- USER 取消任何时候有效（含倒计时中）；进入确认页的那次 USER 按下不算取消（须先见释放）。
- L1 与摇杆右移（lx ≥ 176）立即开始；摇杆左移（lx ≤ 80）取消回主页。
- 自检进行中按键行为不变（R1 取消、结果页规则不变）。
- 中文字形一律 16×16、由 tools/font_convert.py 的 extra 路径生成，不许手写字模；既有字形与 title/ascii 段不得变化。
- 仪表盘顶栏（y 0..11）不得放 16px 汉字；四轮数据行格式不变。
- 回归门槛：`bash tools/test_motion.sh` 全绿、`bash tools/test_boot_ctrl_integration.sh` 7/7 PASS；改动后 ARM `make` 零告警并重记构建指纹。

## Review Focus

1. 倒计时走时用真实毫秒回绕安全差值；暂停（前置不满足）后恢复必须从剩余时间继续，不重置为 5000。
2. 进入确认页那帧 USER 仍按住 → 不得立即算"再按取消"（复用 seen_release 口径）。
3. 倒计时归零自动启动的请求必须当拍被 Ctrl 消费（req 每拍清零），且前置拒绝时页面有可见原因、模型不重复轰炸请求。
4. 新增字形渲染后不得越界/重叠（确认页双行中文 + 倒计时数字同屏）；帧缓冲断言覆盖。
5. 既有页面（品牌/菜单/子菜单/结果/标定）中文化后布局不越域、不与图形元素（卡片边框、滚动标记）重叠。

---

### Task 1: CHECK_CONFIRM 5 秒倒计时启动流程

**Files:**
- Modify: `Core/Inc/ui_model.h`、`Core/Src/ui_model.c`、`Core/Src/freertos.c`
- Test: `tools/test_ui_model.c`、`tools/test_ctrl_task.c`

**Interfaces:**
- `ui_model_in_t` 增加 `uint8_t selfcheck_ready;`（Ctrl 计算：frame_valid && 四轴回中 && vdd_ok && 非标定模式）。
- `ui_model_t` 增加 `uint32_t confirm_since_ms; uint16_t confirm_remain_ms;`（渲染层读剩余毫秒画倒计时）。
- 行为：BRAND 上 USER 稳定边沿 → CHECK_CONFIRM 且 `confirm_since_ms=now`；页内每拍若 `selfcheck_ready==0` 冻结剩余时间并置 `confirm_blocked=1`（渲染显示原因），恢复后继续；剩余归零 → `req.selfcheck_start=1` 并回 BRAND？**否——启动后进入 CHECK_RUN 页（现状不变）**。页内 USER 新边沿 → 取消回 BRAND；L1 或 lx≥176 → `req.selfcheck_start=1` 立即进 CHECK_RUN；lx≤80 → 回 BRAND。

- [ ] **Step 1: 写失败测试**（test_ui_model.c 新增节）：进入确认页即按住 USER 不产生取消（须先释放）；ready=1 时 5000ms 归零产生一次 selfcheck_start 且进 CHECK_RUN；ready=0 冻结（5100ms 后仍不启动，恢复 ready 后走完剩余才启动）；USER 新边沿取消回 BRAND；L1 立即启动；lx=200 立即启动；lx=60 取消；`confirm_remain_ms` 数值单调递减且冻结时不减。test_ctrl_task.c：前置不满足（无手柄）时模型自发的 selfcheck_start 被 Ctrl 拒绝且不产生输出。
- [ ] **Step 2: 跑失败**（test_motion.sh 的 uimodel 行）。
- [ ] **Step 3: 实现**（ui_model 状态机 + freertos 喂 `selfcheck_ready`）。
- [ ] **Step 4: 跑通过**（bash tools/test_motion.sh + test_boot_ctrl_integration.sh 7/7）。
- [ ] **Step 5: Commit**：`git commit -m "feat: selfcheck starts via 5s USER countdown on confirm page"`

### Task 2: 全界面中文化（除仪表盘短代码）

**Files:**
- Modify: `tools/font_convert.py`、`tools/font_src/`（增补源字模）、`Core/Inc/oled_font.h`、`Core/Src/oled_font.c`、`Core/Src/ui_render.c`、`Core/Src/ui_model.c`（如含页面文本常量）、`Core/Inc/ui_render.h`（如需）
- Test: `tools/test_font_convert.py`、`tools/test_ui_render.c`

**Interfaces:**
- 中文文案清单（全部 16×16）：品牌页底部提示（如 `R1 菜单 L1 自检`）；卡片名 `驾驶`/`检查标定`；子菜单标题 `检查与标定`；条目 `自检结果`/`重新自检`/`回到初始位`/`手动标定`；确认页 `即将转动四轮`/`秒后自动开始`/`按USER取消`/`右移或L1立即`；原因文案 `手柄未连接`/`请回中摇杆`/`供电不足`；结果页 `自检结果`/`电机`/`电池`/`全部通过`/`未通过`/`未配置`/`欠压`/`采样错误`/`正在自检`；归位页 `回到初始位`/`等待`/`运动中`/`到位`/`失败`/`无参考`/`请回中`；标定页 `手动标定`/`对齐四轮`/`静止后长按SELECT`/`已记录`。顶栏短代码与 `By Plucksnow` 保留。
- `oled_font_cn_lookup()` 对新字符返回非 NULL；渲染入口全部改为中文常量。

- [ ] **Step 1: 写失败测试**：文案清单逐字符 `oled_font_cn_lookup` 非 NULL；确认页帧缓冲含"即将转动四轮"墨迹且倒计时数字不与汉字重叠；卡片/子菜单/结果页中文化后不越 128×64、不与卡片边框/滚动标记重叠（沿用既有 fb_ink 断言风格）。
- [ ] **Step 2: 跑失败**。
- [ ] **Step 3: 实现**（增补字模源 → font_convert.py 重新生成 → ui_render 替换文案）。
- [ ] **Step 4: 跑通过**（bash tools/test_motion.sh + test_boot_ctrl_integration.sh 7/7 + python3 tools/test_font_convert.py）。
- [ ] **Step 5: ARM 构建**：make 零告警，更新 plan 末尾构建指纹。
- [ ] **Step 6: Commit**：`git commit -m "feat: full Chinese UI text with expanded 16x16 glyphs"`

## 构建记录

### Task 1（88b9dbf）

- ARM `make` 零告警（text 59712 / bss 54496）；构建指纹按计划留待 Task 2 一并重记（Task 1 无 ARM 步骤）。

### Task 2（全界面中文化 + 16×16 字形扩充）

- 工具链：arm-none-eabi-gcc (GNU Tools for STM32 14.3.rel1.20251027-0700) 14.3.1 20250623
- 告警：0（make clean 全量重编，-Wall）
- size：text=62444 data=128 bss=54496 dec=117068
- bin：62580 字节，sha256 = 78aa8c90aaae60b296cb134c55bb22e9ced44b50f03fb8947370033cd4bf74a1
- 主机回归：python3 tools/test_font_convert.py 全绿；bash tools/test_motion.sh 全绿（含 ui_render/ui_model/oled_gfx）；bash tools/test_boot_ctrl_integration.sh 7/7 PASS
