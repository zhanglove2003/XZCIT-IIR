# OLED 分层菜单、驾驶仪表盘与转向 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 按设计文档实现 OLED 页面状态机（品牌页/卡片主菜单/子菜单）、驾驶仪表盘、前后+转向混合、菜单发起的自检/归位/手动标定，以及独立电源服务。

**Architecture:** 新增四个纯/半纯模块 `ui_model`（页面逻辑，Ctrl 任务 10 ms 推进）、`ui_render`（绘制，UI 任务 20 ms 只读快照）、`selfcheck_service`（Ctrl 内推进的自检动作生成）、`power_monitor`（低优先级 ADC 服务）；`boot_ui.c` 收敛为 OLED 初始化 + 渲染循环，不再写电机、不再独立累计编码器；`freertos.c` 的 Ctrl 任务持有页面模型、动作仲裁和唯一电机出口 `ctrl_apply()`。

**Tech Stack:** C11（ARM GCC 交叉编译 + 主机 gcc 测试）、STM32F4 HAL、FreeRTOS（CMSIS-RTOS2）、SSD1306 软件 SPI、主机桩（`tools/host_stubs/`）。

**Spec:** `docs/superpowers/specs/2026-10-31-oled-menu-driving-design.md`（本计划逐条引用其章节号；实现者须同时阅读 spec §7、§8、§11、§14）。

## Global Constraints

以下数值逐字来自 spec，所有任务默认包含本节：

- 轮序与硬件（spec §14）：A 右前=TIM2/CH4/符号-1/CPR 213203；B 左前=TIM3/CH3/+1/212664；C 右后=TIM5/CH2/-1/214993；D 左后=TIM4/CH1/+1/214348。宏 `WHEEL_*_CPR`、`WHEEL_*_FORWARD_SIGN` 已在 `Core/Inc/boot_ui.h`，不得改值。OLED PD7/6/5/4 与镜像方向不得改。
- 混合（spec §8）：死区 16、中心 128、正负两侧按有效行程 112 归一化；前进上限 `DRIVE_CTRL_MAX_PULSE=8400`；转向分量上限 4200；`A=V+T, B=V−T, C=V+T, D=V−T`；`peak>8400` 时四个值同乘 8400/peak；32 位有符号中间值（12600×8400 < 2³¹，int32 够用）；左转为正，右摇杆右推=车体右转（T 为负）。
- 按键（spec §7）：R1=协议位 11，L1=位 10，SELECT=位 0（复用 `WHEEL_HOME_SELECT_BIT`），低有效；30 ms 稳定（10 ms 周期 3 连续有效帧）；R1+L1 同按只受理 R1；SELECT 与确认键同按不启动运动/不记录；进入页面/重连后必须先见到确认键释放。
- 菜单摇杆阈值：偏移 ≥48 触发、回到 ≤24 复位；按住 400 ms 后连移，每 180 ms 一步；横向/纵向各只用一个轴。
- 卡片（spec §5.2）：92×32，选中卡片中心区起点 x=18、y=20，跳动偏移查表 `{0,−1,−2,−1,0}` 周期 600 ms，滑动 120 ms。
- 品牌页（spec §5.1）：标题 `XZCIT-MFSC` 10×16 字模 x=14,y=14；副标题 `By Plucksnow` 6×12 x=28,y=36；底部 y=52 一行提示/最高优先级异常。
- 驾驶仪表盘（spec §6）：顶栏电池 x0..35、行驶状态 x42..83、参考/故障 x90..125；四轮行 y=12/24/36/48，轮名 x=0、RPM 域 x12..59（≤8 字符）、圈数域 x66..125（≤10 字符）；越界显示 `OVF`，无效采样 `--`；电池未配置显示 `B--.-V`。
- 自检（spec §9.1）：A→B→C→D，前转 500 ms/停 200 ms/后转 500 ms/停 200 ms（`SELFCHECK_*` 常量在 `Core/Inc/main.h:148-151`）；判据沿用 MOTOR_TEST_MIN_COUNTS=32 与 MOTOR_HEALTH_MIN_PWM=1200 / 宽限 300 ms / 窗口 300 ms / 64 计数；结果文本沿用 `motor_test_reason_text()`（OK/SKIP/NO FB/FWD ERR/REV ERR/FB ERR）。
- 电池（spec §9.2）：`battery_mV = raw × VDD_mV × divider_num / (4095 × divider_den)`，中间乘法 64 位；分压/规格/阈值现场配置，未配置时不判欠压、不显示百分比、不得把 VDD 当电池电压。MCU VDD 阈值沿用 `BOOT_UI_VDD_MIN_MV=3100`。
- 归位/标定（spec §10）：归位请求只由 L1 提交，等回中+释放 100 ms 才启动；无参考不转；沿用 500 ms 无进展、8 s 总超时、`WHEEL_HOME_STILL_MS=300`、整半圈走反向、SELECT 长按 2000 ms（`WHEEL_HOME_LONG_PRESS_MS`/`MANUAL_REF_RECORD_MS`）。
- 边界（spec §11）：Ctrl 是页面逻辑、输入、编码器累计、电机动作的唯一写入者；UI 只画屏。所有电机写（含写零）必须走 Ctrl 的 `ctrl_apply()`。多字段快照用 seq-lock（沿用 `ctrl_home_publish()` 模式），不用裸 volatile 结构体拷贝。所有计时用真实毫秒 + 回绕安全差值（`(uint32_t)(now - start)` 形式）。
- 开机不自动转轮；进入驾驶页而本次上电未通过自检时零输出并显示"请先自检"。
- 回归命令：`bash tools/test_motion.sh`、`bash tools/test_boot_ctrl_integration.sh`。主机测试一律 `-std=c11 -Wall -Wextra -Werror`，内存类测试加 `-fsanitize=address,undefined`。

## Review Focus

spec 暗示但没有单条测试直接覆盖、最可能咬人的五类输入（每条已在对应任务里钉了测试）：

1. **HAL tick 回绕**：所有 `now - start` 判时在 32 位回绕点必须正常。→ Task 2（测试把 `now_ms` 置于 UINT32_MAX 附近推进）、Task 6、Task 9。
2. **坏帧/失联冻结按键**：`frame_valid=0` 时去抖计数冻结、不产生新确认事件、动作取消。→ Task 2、Task 7（wheel_home 已有 200 ms 规则，回归确认）。
3. **同按优先级**：R1+L1 只返回不确认；SELECT+确认键不启动运动不记录。→ Task 2。
4. **切页不丢 64 位累计圈数**：UI 不读硬件 CNT、不重置 `wheel_feedback`，累计只存于 Ctrl 的 `s_ctrl_fb`。→ Task 4、Task 9（集成测试切页前后读快照比对）。
5. **混合限幅与电池换算溢出**：peak 缩放不越 int32；电池乘法必须 64 位（raw×VDD×num 可达 4.5×10¹⁰）。→ Task 1、Task 8。

---

### Task 1: drive_ctrl 双轴混合

**Files:**
- Modify: `Core/Inc/drive_ctrl.h`、`Core/Src/drive_ctrl.c`
- Test: `tools/test_drive_ctrl.c`（追加用例）

**Interfaces:**
- Produces: `int32_t drive_ctrl_map_axis(uint8_t raw, int32_t limit);`、`void drive_ctrl_mix(uint8_t ly, uint8_t rx, int32_t out[4]);`、宏 `DRIVE_CTRL_TURN_MAX_PULSE 4200`。`out` 下标 0..3 = A/B/C/D。Task 5/9 消费。现有 `drive_ctrl_map()` 保留且改为 `drive_ctrl_map_axis(ly, DRIVE_CTRL_MAX_PULSE)` 的薄封装（现有用例不得回归）。

- [ ] **Step 1: 写失败测试**（追加到 `tools/test_drive_ctrl.c`）

```c
/* 轴映射：死区/端点对称/极限值 */
assert(drive_ctrl_map_axis(128, 8400) == 0);
assert(drive_ctrl_map_axis(144, 8400) == 0);   /* 死区 16 内 */
assert(drive_ctrl_map_axis(0,   8400) ==  8400);
assert(drive_ctrl_map_axis(255, 8400) == -8400);
assert(drive_ctrl_map_axis(0,   4200) ==  4200);
assert(drive_ctrl_map_axis(255, 4200) == -4200);
/* 混合：out = A,B,C,D */
int32_t m[4];
drive_ctrl_mix(128, 128, m);              /* 全零 */
for (i=0;i<4;i++) assert(m[i]==0);
drive_ctrl_mix(0, 128, m);                /* 满前进 → 四轮 +8400 */
drive_ctrl_mix(255, 128, m);              /* 满后退 → 四轮 -8400 */
drive_ctrl_mix(128, 0, m);                /* 摇杆左=左转(T=+4200)：右轮 A/C=+4200，左轮 B/D=-4200 */
assert(m[0]==4200 && m[2]==4200 && m[1]==-4200 && m[3]==-4200);
drive_ctrl_mix(128, 255, m);              /* 右转：A/C=-4200，B/D=+4200 */
drive_ctrl_mix(0, 255, m);                /* 前进+右转：peak=12600 缩放 → A=C=2800, B=D=8400 */
assert(m[0]==2800 && m[2]==2800 && m[1]==8400 && m[3]==8400);
drive_ctrl_mix(255, 0, m);                /* 后退+左转：A=C=-2800, B=D=-8400 */
```

- [ ] **Step 2: 跑测试确认失败**：`gcc -std=c11 -Wall -Wextra -Werror -ICore/Inc tools/test_drive_ctrl.c Core/Src/drive_ctrl.c -o /tmp/map && /tmp/map` → 编译失败（函数未定义）。
- [ ] **Step 3: 实现**。`drive_ctrl_map_axis`：`delta=raw−128`，`|delta|≤16 → 0`，否则 `over=|delta|−16`，`pulse = over*limit/112`（int32，112×8400=940800 无溢出），符号随 delta。`drive_ctrl_mix` 按 Global Constraints 公式：`v=map_axis(ly,8400)`、`t=map_axis(rx,4200)`，`peak` 取四值绝对值最大，`peak>8400` 时 `out[i] = out[i]*8400/peak`。`drive_ctrl_map` 改为封装。
- [ ] **Step 4: 跑测试通过**（同 Step 2 命令，期望 PASS）。
- [ ] **Step 5: Commit**：`git add Core/Inc/drive_ctrl.h Core/Src/drive_ctrl.c tools/test_drive_ctrl.c && git commit -m "feat: add dual-axis drive mixing with turn limit"`

### Task 2: ui_model 纯页面状态机

**Files:**
- Create: `Core/Inc/ui_model.h`、`Core/Src/ui_model.c`
- Test: `tools/test_ui_model.c`（新文件）

**Interfaces:**
- Produces（Task 4/5/7/9 消费）:

```c
typedef enum { UI_PAGE_BRAND=0, UI_PAGE_MENU, UI_PAGE_SUBMENU, UI_PAGE_DRIVE,
  UI_PAGE_CHECK_CONFIRM, UI_PAGE_CHECK_RUN, UI_PAGE_CHECK_RESULT,
  UI_PAGE_HOME, UI_PAGE_MANUAL_CAL } ui_page_t;
#define UI_MENU_COUNT 2U   /* 0=驾驶 1=检查与标定 */
#define UI_SUB_COUNT  4U   /* 0=自检结果 1=重新自检 2=回到初始位 3=手动标定 */
#define UI_STICK_ENGAGE 48
#define UI_STICK_RELEASE 24
#define UI_NAV_FIRST_REPEAT_MS 400U
#define UI_NAV_REPEAT_MS 180U
#define UI_BTN_STABLE_FRAMES 3U
#define UI_BTN_BIT_R1 11U
#define UI_BTN_BIT_L1 10U
/* 低有效原始键位；摇杆 0..255；user = USER 引脚电平(按下为 1) */
typedef struct {
  uint16_t buttons; uint8_t lx, ly, rx, ry;
  uint8_t user_pressed, frame_valid;
  uint8_t selfcheck_passed;   /* Ctrl 回喂：本次上电自检是否已通过 */
  uint8_t action_busy;        /* Ctrl 回喂：自检/归位执行中 */
  uint8_t result_window_open; /* 自检刚结束的 5 s 标定入口窗口（Ctrl 计时） */
  uint32_t now_ms;
} ui_model_in_t;
typedef struct {
  uint8_t selfcheck_start, home_start, record_ref, cancel_motion;
} ui_model_req_t;
typedef struct {
  ui_page_t page, back_page;
  uint8_t menu_sel, sub_sel, sub_scroll, confirm_focus, result_view;
  uint32_t sel_since_ms;      /* 焦点/滑动动画锚点 */
  /* 内部：每键 filtered/candidate/frames/press_ms/seen_release；菜单与子菜单
     摇杆轴的 engage/release/连移计时 —— 字段布局由实现者定，全部静态定长 */
} ui_model_t;
void ui_model_init(ui_model_t *m, uint32_t now_ms);
void ui_model_step(ui_model_t *m, const ui_model_in_t *in, ui_model_req_t *req);
/* 行驶状态分类（Task 5 顶栏用）：V/T 死区内为 0 */
typedef enum { UI_DRV_STOP=0, UI_DRV_FWD, UI_DRV_REV, UI_DRV_LEFT, UI_DRV_RIGHT,
  UI_DRV_FWD_LEFT, UI_DRV_FWD_RIGHT, UI_DRV_REV_LEFT, UI_DRV_REV_RIGHT } ui_drive_status_t;
ui_drive_status_t ui_drive_status_classify(int32_t v, int32_t t);
```

- [ ] **Step 1: 写失败测试** `tools/test_ui_model.c`。用例（每条一个 assert 组，坐标/键位用真实值）：
  - 导航：BRAND 上 R1 → MENU；MENU 上 L1 进 DRIVE；SUBMENU R1 回 MENU 且 `sub_sel` 保留；CHECK_RESULT R1 回 SUBMENU 原条目。
  - L1 单发：MENU 按住 L1 连续 step 20 帧只产生一次进入（`req` 只在第一帧非零）；释放后不再触发。
  - 进页屏蔽：从 MENU 进入 DRIVE 的同帧 L1 仍按着 → 不触发 DRIVE 内任何动作。
  - 优先级：R1 与 L1 同帧稳定 → 只出现返回（page 变化），无确认副作用。
  - SELECT+确认同帧 → 无运动请求（`selfcheck_start/home_start/record_ref` 全 0）。
  - 自检确认页默认焦点取消；摇杆下移（ly>176）到"开始"，L1 产生 `selfcheck_start=1`；`selfcheck_passed=0` 时进入 DRIVE 后 `req` 无运动类请求，由渲染层显示"请先自检"（模型只负责 page 状态）。
  - USER 键：BRAND 上稳定 USER → CHECK_CONFIRM（与 L1 同义）；MANUAL_CAL 上 USER → 退出回 SUBMENU 且不记录。
  - 5 s 标定窗口：CHECK_RESULT 且 `result_window_open=1` 时稳定 L1 或 USER → page=MANUAL_CAL（须先见释放）；窗口字段清 0 后（Ctrl 超时）page=BRAND，且不清异常状态。从 SUBMENU 进入的结果页 `back_page==SUBMENU`，R1 回原条目，不受窗口影响。
  - 摇杆阈值：偏移 47 不动、48 动一格；不回到 24 以内不复位；按住 400 ms 内只动一格，之后每 180 ms 动一格（用 `now_ms` 步进模拟）。
  - 横向不回绕：MENU `menu_sel=0` 再左移不变；=1 再右移不变。主菜单记住上次选中项（返回后仍是 1）。
  - 动作中取消：`action_busy=1` 且 page==CHECK_RUN，R1 → `cancel_motion=1` 且 page 离开；摇杆越出死区同样 `cancel_motion=1`。
  - 坏帧：`frame_valid=0` 连续多帧，去抖计数冻结，恢复有效帧后需重新满 3 帧才产生事件。
  - 回绕：`ui_model_init(m, UINT32_MAX - 100)` 后推进 200 ms，导航/连移行为不变（无回绕卡死）。
  - `ui_drive_status_classify`：`(0,0)→STOP`、`(+,0)→FWD`、`(0,+)→LEFT`、`(+,−)→FWD_RIGHT`、`(−,+)→REV_LEFT` 等 9 值。
- [ ] **Step 2: 跑失败**：`gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc tools/test_ui_model.c Core/Src/ui_model.c -o /tmp/uimodel && /tmp/uimodel` → 编译失败。
- [ ] **Step 3: 实现**。按键稳定判定沿用 wheel_home 的"连续有效帧"口径（3 帧）；`frame_valid=0` 冻结所有计数不清零。页面转移表照 spec §4/§7 实现。R1 在 `action_busy=1` 时先发 `cancel_motion` 再转移。所有时间比较用 `(uint32_t)(now - t0)` 形式。
- [ ] **Step 4: 跑通过**（同 Step 2）。
- [ ] **Step 5: 注册进 `tools/test_motion.sh`**（新增一行 gcc+运行，编译选项同 Step 2），并确认 `bash tools/test_motion.sh` 全绿。
- [ ] **Step 6: Commit**：`git add Core/Inc/ui_model.h Core/Src/ui_model.c tools/test_ui_model.c tools/test_motion.sh && git commit -m "feat: add pure OLED page state machine"`

### Task 3: 10×16 标题字体与反白文字

**Files:**
- Modify: `tools/font_convert.py`、`Core/Inc/oled_font.h`、`Core/Src/oled_font.c`、`Core/Inc/oled_gfx.h`、`Core/Src/oled_gfx.c`
- Test: `tools/test_oled_gfx.c`（追加）、`tools/test_font_convert.py`（如需）

**Interfaces:**
- Produces: `#define OLED_FONT_TITLE_COUNT 9`、`extern const uint8_t oled_font_title[OLED_FONT_TITLE_COUNT][20];`（10×16：2 页 × 10 列，page 格式同现有字模）、`const uint8_t *oled_font_title_lookup(char c);`（未含字符返回 NULL）、`void oled_gfx_text6x12_inv(int16_t x, int16_t y, const char *ascii);`（整格反白：先填实再画反相点阵）。Task 5 消费。

- [ ] **Step 1: 写失败测试**（追加到 `tools/test_oled_gfx.c`）：
  - `oled_font_title_lookup('X') != NULL` 且 20 字节内至少 1 位为 1；`lookup('Q')==NULL`；"XZCIT-MFSC" 10 个字符全部可查得。
  - `oled_gfx_text6x12_inv(0,0,"AB")` 后缓冲区 (0,0) 为亮（反白底）且字形笔画处为暗；与普通 `text6x12` 区域按位互补（逐字节 assert）。
- [ ] **Step 2: 跑失败**（`bash tools/test_motion.sh` 中 test_oled_gfx 现有行）→ 编译失败。
- [ ] **Step 3: 实现**。扩展 `tools/font_convert.py` 生成 10×16 字模（源字体取 `tools/font_src/` 现有 ASCII 位图放大或整型替换；无现成 10×16 源则按现有 6×12 字形在脚本内用最近邻放大到 10×16），重跑脚本再生 `oled_font.c` 的 title 段；`oled_gfx.c` 实现 `_inv`（复用 6×12 点阵，先 `fill_rect` 6×12 再以 `on=0` 画点）。
- [ ] **Step 4: 跑通过**（`bash tools/test_motion.sh`）。
- [ ] **Step 5: Commit**：`git add tools/font_convert.py Core/Inc/oled_font.h Core/Src/oled_font.c Core/Inc/oled_gfx.h Core/Src/oled_gfx.c tools/test_oled_gfx.c && git commit -m "feat: add 10x16 title font and inverted 6x12 text"`

### Task 4: ui_snapshot 快照结构与 Ctrl 发布

**Files:**
- Modify: `Core/Inc/ui_render.h`（新建，仅快照部分）、`Core/Src/freertos.c`
- Test: `tools/test_ctrl_task.c`（追加用例）

**Interfaces:**
- Consumes: Task 1 的 `drive_ctrl_mix`、Task 2 的 `ui_drive_status_t`、既有 `wheel_feedback_*`、`motor_health_*`。
- Produces（Task 5/6/8/9 消费）:

```c
/* Core/Inc/ui_render.h（本任务只放快照，绘制函数 Task 5 加） */
typedef struct {
  ui_page_t page;
  uint8_t menu_sel, sub_sel, sub_scroll, confirm_focus, result_view;
  uint32_t sel_since_ms;
  uint8_t drive_permitted;      /* Ctrl 仲裁：当前允许输出 */
  uint8_t action_busy, selfcheck_passed;
  uint8_t selfcheck_state[4];   /* 0 等待 1 运行 2 通过 3 失败 */
  uint8_t home_state[4];        /* 0 等待 1 运动 2 到位 3 失败 */
  uint8_t home_block_reason;    /* 0 无 1 无参考 2 等回中 3 故障 */
  uint8_t result_window_open;   /* 自检结束 5 s 标定入口窗口 */
  uint8_t fault_mask, reference_valid, frame_valid;
  ui_drive_status_t drive_status;
  int32_t wheel_rpm[4];
  int64_t wheel_milliturns[4];
  uint8_t wheel_sample_valid[4];
  uint8_t battery_valid, battery_error, undervolt, vdd_ok;
  uint32_t battery_mv;
} ui_snapshot_t;
void ui_snapshot_get(ui_snapshot_t *out);   /* seq-lock 读，实现在 freertos.c */
```

Ctrl 侧：新增 `static ui_model_t s_ui_model;`、`static volatile ui_snapshot_t s_ui_snap; static volatile uint32_t s_ui_snap_seq;`、发布函数（照抄 `ctrl_home_publish` 的 seq-lock 写法）与 `ui_snapshot_get()` 读侧（照抄 `wheel_home_get_status` 含 4 次重试 + 临界区兜底）。

- [ ] **Step 1: 写失败测试**（追加到 `tools/test_ctrl_task.c`，新增 scenario）：
  - Ctrl 跑起来后 `ui_snapshot_get()` 返回 `page==UI_PAGE_BRAND`；喂一帧把左摇杆推满前进的合法帧 + 模拟 menu 导航后 page==UI_PAGE_DRIVE 且 `drive_status==UI_DRV_FWD`（回中 100 ms 前 `drive_permitted==0`，之后置 1，四轮 compare=8400）。
  - 摇杆回中放宽后立刻 R1：下一控制周期四轮 compare 全 0（不等 OLED）。
  - 快照 64 位字段：编码器注入累计转动 N 圈 → `wheel_milliturns[0]` 约 ±N×1000（±5%）；切页（R1 进菜单再回）后数值只增不减（计数连续）。
- [ ] **Step 2: 跑失败**（`tools/test_motion.sh` 的 test_ctrl_task 行）。
- [ ] **Step 3: 实现**。freertos.c：每轮 `ui_model_step()`（放在手柄采样后、仲裁前）；`drive_permitted` 仲裁 = `page==UI_PAGE_DRIVE && selfcheck_passed && 回中持续≥100ms && fault_mask==0 && !action_busy`（回中计时沿用 `CTRL_REARM_MS` 机制，单独变量）；驾驶命令 = `drive_ctrl_mix(in.ly, in.rx, command)`（替换现有四轮同值的 `drive_ctrl_map` 循环）；`drive_status = ui_drive_status_classify(v, t)`；RPM/圈数沿用现有 `s_ctrl_fb` 累计（rpm 用 `boot_ui_rpm_from_elapsed`，100 ms 窗口照旧）。`ui_model` 请求在本任务只处理 `cancel_motion`（清命令），其余请求字段 Task 6/7 接。
- [ ] **Step 4: 跑通过**（`bash tools/test_motion.sh`）。
- [ ] **Step 5: Commit**：`git add Core/Inc/ui_render.h Core/Src/freertos.c tools/test_ctrl_task.c && git commit -m "feat: publish UI snapshot and gate drive output by page state"`

### Task 5: ui_render 页面绘制

**Files:**
- Modify: `Core/Inc/ui_render.h`、新建 `Core/Src/ui_render.c`
- Test: `tools/test_ui_render.c`（新文件）

**Interfaces:**
- Consumes: Task 3 字体/反白、Task 4 `ui_snapshot_t`、`oled_gfx_*`。
- Produces: `void ui_render_draw(const ui_snapshot_t *snap, uint32_t now_ms);`（clear + 按页绘制，不 flush）；格式化函数与动画函数（均可主机测试）：

```c
void ui_render_format_battery(char out[7], uint8_t valid, uint32_t mv); /* B24.0V/B--.-V/B OVF */
void ui_render_format_rpm(char out[9], char wheel, int32_t rpm, uint8_t valid);   /* A+120rpm / A  --   */
void ui_render_format_turns(char out[11], int64_t milliturns, uint8_t valid);     /* +001.50R / OVF / -- */
int8_t ui_render_hop_offset(uint32_t now_ms, uint32_t sel_since_ms);   /* {0,-1,-2,-1,0} 周期600ms */
int8_t ui_render_slide_offset(uint32_t now_ms, uint32_t sel_since_ms); /* 120ms 线性滑动查表 */
```

- [ ] **Step 1: 写失败测试** `tools/test_ui_render.c`（链接 `oled_gfx.c oled_font.c ui_render.c`，通过 `oled_gfx_buffer()` 检查帧缓冲）：
  - 三个格式化函数的精确字符串断言（含 `B OVF`、负圈数 `-002.75R`、`OVF`、`--` 分支）。
  - 品牌页：`page==BRAND` 绘制后 (14..113, 14..29) 有点亮像素（标题区）且 y≥52 区域只有一行提示。
  - 驾驶页：四轮行文本起始 y∈{12,24,36,48}，RPM 域与圈数域绘制后互不重叠（对 `x=60..65` 竖条断言全暗——列间隙）。顶栏故障位时右侧显示优先级文本（fault_mask=1 → 显示故障提示而非 REF）。
  - 菜单页：选中卡片边框存在（x=18,y=20 起 92×32 有边框像素）；未选中卡片在屏幕对侧边缘露出 ≥1 列像素。
  - 动画：`hop_offset(0,0)==0`、`hop_offset(150,0)==-1`、`hop_offset(300,0)==-2`、`hop_offset(600,0)==0`；`slide_offset(0,0)==-?(刚切换的位移大端)`、`slide_offset(120,0)==0`。跨页/未知 page 不越界写缓冲（ASan 保证）。
  - 子菜单页：16 px 标题"检查与标定"，下方三行反白列表（`text6x12_inv`），滚动窗口随 `sub_scroll` 平移，底部窄滚动标记（x≥126 有 1 列像素）；四个条目分别可到顶/到底。
  - 结果页两屏：`result_view==0` 绘制四轮名 + `motor_test` 原因文本占位（快照 state=3 时显示失败原因区）；`result_view==1` 显示"电池未配置"（`battery_valid==0`）或完整 mV 数值，不截高位。
- [ ] **Step 2: 跑失败**：`gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc tools/test_ui_render.c Core/Src/ui_render.c Core/Src/oled_gfx.c Core/Src/oled_font.c Core/Src/ui_model.c -o /tmp/uirender && /tmp/uirender`。
- [ ] **Step 3: 实现**。九个页面各一个 static 绘制函数 + 顶栏状态优先级（故障>失联>等待回中>无参考>REF，spec §6）；卡片跳动/滑动只由 `now_ms` 与快照锚点计算，无循环等待。图标本任务先用 1px 边框占位（Task 9 补）。
- [ ] **Step 4: 跑通过**；注册进 `tools/test_motion.sh`。
- [ ] **Step 5: Commit**：`git add Core/Inc/ui_render.h Core/Src/ui_render.c tools/test_ui_render.c tools/test_motion.sh && git commit -m "feat: render OLED pages from read-only snapshot"`

### Task 6: selfcheck_service（Ctrl 推进的自检）

**Files:**
- Create: `Core/Inc/selfcheck_service.h`、`Core/Src/selfcheck_service.c`
- Modify: `Core/Src/freertos.c`（请求接入 + 动作生成接入 `ctrl_apply`）
- Test: `tools/test_selfcheck_service.c`（新文件）

**Interfaces:**
- Consumes: `motor_test_*`、`SELFCHECK_*`（main.h）、Task 2 的确认请求。
- Produces:

```c
#include "main.h"   /* SELFCHECK_PWM_PULSE/RUN_MS/PAUSE_MS/CYCLES */
typedef enum { SELFCHECK_IDLE=0, SELFCHECK_RUNNING, SELFCHECK_DONE } selfcheck_phase_t;
typedef struct {
  selfcheck_phase_t phase;
  uint8_t wheel, subphase;          /* subphase: 0 前转 1 停 2 反转 3 停 */
  uint8_t state[4];                 /* 快照用：0 等待 1 运行 2 通过 3 失败 */
  uint8_t passed;                   /* 四轮全过 */
  uint32_t seq;                     /* 每完成一轮 +1；Ctrl 据此清故障锁存 */
  uint32_t phase_start_ms;
  int32_t  phase_counts;            /* 当前段净计数（回绕已由调用方增量抵消） */
  motor_test_result_t result[4];
} selfcheck_service_t;
void selfcheck_service_init(selfcheck_service_t *s);
void selfcheck_service_request(selfcheck_service_t *s, uint32_t now_ms);
void selfcheck_service_cancel(selfcheck_service_t *s);
/* 每 10 ms 调用：fb_delta 为四轮前进为正的增量；command 四轮写 0 或受试轮 ±SELFCHECK_PWM_PULSE。
 * 运行中四轴越出驾驶死区由 Ctrl 检测并调 cancel。取消/失败不清旧故障锁存。 */
void selfcheck_service_step(selfcheck_service_t *s, const int32_t fb_delta[4],
                            uint32_t now_ms, int32_t command[4]);
```

- [ ] **Step 1: 写失败测试**：
  - 正常轮：注入每 10 ms ±50 计数增量 → 走完 500/200/500/200 → `state==2`；`command` 前转段 == +8400、暂停段 == 0、反转段 == −8400；其他轮恒 0；全程（四轮正常）名义时长 = 8×(500+200) ms ±10 ms。
  - 缺反馈轮：增量 0 → `state==3` 且 `motor_test_reason_text` 报 `NO FB`；反向轮：增量与命令反号 → `FWD ERR`/`REV ERR`；标定模式（`drive_calibration_mode=1`）→ 全轮 SKIP 且 command 恒 0。
  - 取消：RUNNING 中 `cancel` → command 全 0、`phase==DONE`、`passed==0`，且此前轮的 `state` 保留（结果页可回看）。
  - 时序回绕：`now_ms` 从 UINT32_MAX−1000 开始跑完整轮，段切换正确。
  - 完整通过时 `seq` 只 +1 一次；`passed==1` 仅当四轮 `state==2`。
- [ ] **Step 2: 跑失败**：`gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc -Itools/host_stubs tools/test_selfcheck_service.c Core/Src/selfcheck_service.c Core/Src/motor_test.c -o /tmp/scs && /tmp/scs`。
- [ ] **Step 3: 实现**。逐轮状态机照 spec §9.1；所有 PWM/DIR 语义只落到 `command[]`（符号=前进为正），硬件由 Ctrl 的 `ctrl_apply` 落地（换向等待、唤醒延时天然复用）。
- [ ] **Step 4: 跑通过**；注册进 `tools/test_motion.sh`。
- [ ] **Step 5: 接入 freertos.c**：`ui_model_req_t.selfcheck_start` 且满足前置（frame_valid、四轴回中、`vdd_ok`、非标定模式）→ `selfcheck_service_request`；每轮 `selfcheck_service_step(..., command)` 在驾驶仲裁之前，RUNNING 期间 command 由服务独占；seq 变化时沿用 `motor_health_clear_after_selfcheck(&s_ctrl_health, passed)`；`passed==0` 时 `buzzer_alarm()`。服务 DONE 后 Ctrl 打开 5 s 标定窗口（快照 `result_window_open=1`，超时清 0；窗口内 L1/USER 进标定由 ui_model 按 Task 2 规则处理）。测试：`tools/test_ctrl_task.c` scenario 断言"重新自检失败不清已锁存故障；全过 + 回中后才恢复输出"。
- [ ] **Step 6: 全量回归** `bash tools/test_motion.sh` PASS。
- [ ] **Step 7: Commit**：`git add Core/Inc/selfcheck_service.h Core/Src/selfcheck_service.c Core/Src/freertos.c tools/test_selfcheck_service.c tools/test_ctrl_task.c tools/test_motion.sh && git commit -m "feat: move motor selfcheck into Ctrl-owned service"`

### Task 7: 归位与手动标定的菜单接入

**Files:**
- Modify: `Core/Inc/wheel_home.h`、`Core/Src/wheel_home.c`、`Core/Inc/manual_ref.h`、`Core/Src/manual_ref.c`、`Core/Src/freertos.c`
- Test: `tools/test_wheel_home.c`、`tools/test_manual_ref.c`（各追加用例）

**Interfaces:**
- Produces:
  - `void wheel_home_request_home(wheel_home_t *s);` — 菜单 L1 置 pending 请求；`wheel_home_step` 在 `MANUAL` 模式下等"四轴回中 + R1/L1/SELECT 释放连续 `WHEEL_HOME_RESUME_NEUTRAL_MS`(100 ms)"才进 RUNNING；提交那次 L1 的释放不计为取消。无参考时请求直接以 `WHEEL_HOME_RESULT_NO_REF` 落地，不动轮。SELECT 短按快捷入口保留。
  - `void manual_ref_open_manual(manual_ref_t *s, uint32_t now_ms);` — 跳过 USER 窗口直接进手动对齐页（`open` + `enter_manual`）。
  - `wheel_home_input_t` 增加字段 `uint8_t manual_calib_page;` — 仅该页为 1 时 `WHEEL_HOME_BUTTON_HELD` 下的长按 SELECT 才允许走记录参考路径（现 `wheel_home.c:774` 的长按记录入口加此门控）；驾驶页长按 SELECT 不再写参考，短按归位快捷保留。

- [ ] **Step 1: 写失败测试**（追加）：
  - `test_wheel_home.c`：菜单请求后按住 L1 → 100 ms 内不进 RUNNING；释放且回中满 100 ms 后进 RUNNING 且不被同次按键取消；无参考 → `NO_REF` 且 `pulse` 全 0；运行中摇杆越出死区 → 取消停止（回归现有规则）。
  - `test_manual_ref.c`：`open_manual` 后立即可记录（无需 USER 窗口）；连续静止 300 ms + 长按 2 s → `record_requested`；第一次合法请求即可成功、失败应答不显示成功（回归）。
  - `test_ctrl_task.c`：驾驶页长按 SELECT 3 s → 参考不写入；进入手动标定页（page==UI_PAGE_MANUAL_CAL 且 `manual_ref` 活跃）后同样长按 → 参考写入、`boot_ui_manual_ack` 等价应答路径成功。
- [ ] **Step 2: 跑失败**（`bash tools/test_motion.sh` 对应用例行）。
- [ ] **Step 3: 实现**。`wheel_home.c` 加 pending 标志与进入前等待计时（复用 `require_release`/`neutral_ms` 机制）；`freertos.c`：`home_start` 请求 → `wheel_home_request_home`；`record_ref` 请求 → 走现有 `manual_ref` + `wheel_home_track_manual_still` + `wheel_home_record_manual_reference` 路径（Ctrl 侧应答照旧）；`page==UI_PAGE_MANUAL_CAL` 时命令强制零 + `ctrl_motors_sleep`，Ctrl 持续采样。
- [ ] **Step 4: 跑通过**（`bash tools/test_motion.sh`）。
- [ ] **Step 5: Commit**：`git add Core/Inc/wheel_home.h Core/Src/wheel_home.c Core/Inc/manual_ref.h Core/Src/manual_ref.c Core/Src/freertos.c tools/test_wheel_home.c tools/test_manual_ref.c tools/test_ctrl_task.c && git commit -m "feat: add menu-requested homing and manual-calibration page scope"`

### Task 8: power_monitor 电源服务

**Files:**
- Create: `Core/Inc/power_monitor.h`、`Core/Src/power_monitor.c`
- Modify: `Core/Src/freertos.c`（新建低优先级任务）
- Test: `tools/test_power_monitor.c`（新文件）

**Interfaces:**
- Consumes: `boot_ui_vdd_mv()`（boot_ui.h 内联，保持不动）、ADC1。
- Produces:

```c
/* 现场确认分压/规格/阈值后改 1 并填宏；未配置时不判欠压 */
#define POWER_BATTERY_CONFIGURED 0
#define POWER_BATT_DIV_NUM 1U
#define POWER_BATT_DIV_DEN 1U
#define POWER_BATT_MIN_MV  0U
typedef struct {
  uint8_t vdd_valid; uint32_t vdd_mv;
  uint8_t battery_configured, battery_valid, battery_error, undervolt;
  uint32_t battery_mv;   /* 未配置恒 0 */
} power_snapshot_t;
void power_monitor_init(void);
void power_monitor_step(uint32_t now_ms);      /* 500 ms 节拍，内部判断 */
const power_snapshot_t *power_monitor_get(void);
uint32_t power_monitor_battery_mv(uint32_t raw, uint32_t vdd_mv); /* 纯换算，64 位中间值 */
/* 测试可注入分压比；公开函数用默认宏包装 */
uint32_t power_monitor_battery_mv_div(uint32_t raw, uint32_t vdd_mv,
                                      uint32_t num, uint32_t den);
```

- [ ] **Step 1: 写失败测试**：
  - `power_monitor_battery_mv(4095, 3300) == 3300`（num=den=1）；`power_monitor_battery_mv(2048, 3300) == 1650`（截断）；大数值不溢出：`power_monitor_battery_mv(4095, 3300)` 在 num=100 时 == 330000（测试里临时用可注入分母的内部实现或把分母作参数——实现为 `power_monitor_battery_mv_div(raw, vdd, num, den)` 供测试，公开函数包默认宏）。
  - step 节拍：喂 `now_ms` 隔 499 ms 不重采（stub 计数不变）、隔 501 ms 重采；回绕点不卡。
  - 未配置（宏 0）：`battery_valid==0 && battery_mv==0 && undervolt==0`；配置为 1 + 欠压阈值 → `undervolt==1`。
- [ ] **Step 2: 跑失败**：`gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc -Itools/host_stubs tools/test_power_monitor.c Core/Src/power_monitor.c -o /tmp/pw && /tmp/pw`（ADC 调用在文件内以弱桩隔离：把 `HAL_ADC_*` 调用收敛到 power_monitor.c 顶部可被测试替换的包装函数）。
- [ ] **Step 3: 实现**。500 ms 节拍：VREFINT 8 次均值 → vdd_mv（复用 `boot_ui_vdd_mv`）；`POWER_BATTERY_CONFIGURED` 为 1 时再采 ADC1_IN8 并换算，采样失败置 `battery_error`；通道配置切换照抄现 `ui_read_vdd_mv()` 的"用完恢复 CH8"次序。freertos.c 新建 `powerTask`（`osPriorityBelowNormal`、栈 256×4），循环 `power_monitor_step(HAL_GetTick()); osDelay(50);`。Ctrl 侧把 `vdd_ok`（VDD<3100 禁止运动）改读 `power_monitor_get()`；`boot_ui.c` 的 `ui_read_vdd_mv` 在 Task 9 删除。
- [ ] **Step 4: 跑通过**；注册进 `tools/test_motion.sh`。
- [ ] **Step 5: Commit**：`git add Core/Inc/power_monitor.h Core/Src/power_monitor.c Core/Src/freertos.c tools/test_power_monitor.c tools/test_motion.sh && git commit -m "feat: add low-priority power monitoring service"`

### Task 9: boot_ui 收敛、集成测试与全量回归

**Files:**
- Modify: `Core/Src/boot_ui.c`、`Core/Inc/boot_ui.h`、`Core/Src/freertos.c`、`Makefile`、`tools/test_boot_ctrl_integration.c`、`tools/test_boot_ctrl_integration.sh`、`tools/test_motion.sh`、`tools/test_ctrl_task.c`（桩同步）
- Create: `Core/Src/ui_model.c` 等（已在 Makefile 注册）

**Interfaces:**
- Consumes: Task 2-8 全部。本任务是最终装配。

- [ ] **Step 1: boot_ui.h/boot_ui.c 收敛**（spec §11.2、§12）：
  - 删除：`s_motors` 表与全部 PWM/DIR/SLP 写入、`ui_motor_selfcheck`、`ui_run_with_feedback`、`ui_read_vdd_mv`、`ui_main_page` 的独立编码器累计（`boot_ui_diag` 中位置/圈数数据源改为快照）、`boot_ui_motor_owner/handoff_seq/selfcheck_seq/selfcheck_passed/drive_enabled` 全部全局（单归属后不再需要；`boot_ui_ready` 保留，OLED 初始化完成后置 1）。
  - `boot_ui_run()` 新流程：SLP 引脚低 + PWM 通道启动占空比 0 + 编码器启动 + `oled_init()` → `boot_ui_ready=1` → 20 ms 循环：`ui_snapshot_get(&snap); ui_render_draw(&snap, HAL_GetTick()); oled_flush();`。自检/确认/结果/标定页全部由快照驱动（Ctrl 里的 `ui_model` + 服务推进）。
  - `freertos.c`：`boot_ui_motor_owner` 判断移除（电机自始至终归 Ctrl；写零走 `ctrl_apply` 不再伤自检——自检命令也来自 `ctrl_apply`）。开机首帧起 `ctrl_apply` 只有零命令 → 驱动睡眠，满足"开机不自动转轮"。
- [ ] **Step 2: 改集成测试** `tools/test_boot_ctrl_integration.c`：
  - scenario 重写：UI+Ctrl+自检服务一起跑（ucontext 双任务），断言——每轮两个 500 ms 运动段、段间零输出、总时长 5.6 s±100 ms；正常/缺反馈/反向/跳过/取消五类结果；上一轮故障锁存时允许重新自检、失败不清锁存、全过后仍须回中门槛才出力；菜单 L1 发起归位不被同次 L1 取消；无参考不转；手动标定"连续静止 300 ms + 新长按 2 s"成功、断连拒绝、长按释放不触发归位；`3.3 V` 不得出现在电池读数（未配置 → `B--.-V`）。
- [ ] **Step 3: 跑失败 → 实现装配 → 跑通过**：`bash tools/test_boot_ctrl_integration.sh && bash tools/test_motion.sh` 全绿。Makefile `C_SOURCES` 追加 `Core/Src/ui_model.c`、`Core/Src/ui_render.c`、`Core/Src/selfcheck_service.c`、`Core/Src/power_monitor.c`。
- [ ] **Step 4: 视觉打磨**（spec §5.2/实施次序 7）：菜单卡片图标位图（`ui_render.c` 内 static 数组）、跳动/滑动启用（Task 5 已实现函数，此处把 `sel_since_ms` 锚点从模型接到绘制）、中文提示字形核对（`oled_font_cn` 现有 17 字不够时经 `tools/font_convert.py` 增补）。跑 `test_oled_gfx`/`test_ui_render` 确认无越界。
- [ ] **Step 5: ARM 构建记录**：用固定工具链（现 Makefile 的 `arm-none-eabi-gcc`）全量编译，记录版本、告警数、固件大小、`sha256sum build/*.elf`，追加到 `docs/superpowers/plans/2026-10-31-oled-menu-driving.md` 末尾"构建记录"一节。
- [ ] **Step 6: Commit**：`git add -A && git commit -m "feat: converge boot_ui into render loop, wire full menu/drive flow"`

---

## 上板验收（不在本计划内，现场另行安排）

按 spec §13 顺序：电机断电只读核对按键/摇轴 → 电池接线与分压核对 → 低速方向确认（保留硬件极性，不猜反转 B）→ 菜单停机/回中恢复/断连停机/自检重跑/标定/归位 → 采样至输出延迟测量。
