# 四轮最短路径归位 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 用户指定由 DeepSeek 编码；本文只交付方案，不由当前助手执行固件开发。

**Goal:** 长按 SELECT 记录四轮参考朝向，短按后每轮沿最短路径低速回到该朝向。

**Architecture:** 在现有 Ctrl 任务中集成纯 C 归位状态机，保留唯一电机写入者。Ctrl 独立采样编码器，UI 通过快照显示结果，避免使用低频 UI 计数做闭环。

**Tech Stack:** STM32F407、STM32 HAL、CMSIS-RTOS2/FreeRTOS、C11 主机测试、ARM GCC。

**Spec:** `/mnt/data/project-ai/Embedded_Development/basic_project/hal_f407_mecanum_4motor_src/docs/superpowers/specs/2026-10-30-wheel-angle-home-design.md`，先全文阅读。

**补充 Spec（必须同时实现）：** `/mnt/data/project-ai/Embedded_Development/basic_project/hal_f407_mecanum_4motor_src/docs/superpowers/specs/2026-10-30-selfcheck-reference-flow-design.md`。后文 Task 5–7 加入自检结束页、手动对齐入口及运行监测。若尚未开始开发，先完成 Task 5 的反馈结果结构，再做 Task 1–4，并在真实 Ctrl 接入时一起完成 Task 6–7，避免先做出引导页不读取手柄的版本。

## Global Constraints

- 用户确认的是恢复初始朝向，不是退回累计零点；最多走最近半圈。
- SELECT 长按 2000 ms 记录，短按释放触发；无参考不运动，断电参考失效。
- 索引顺序 RF/LF/RR/LR；编码器 TIM2/TIM3/TIM5/TIM4，不能交换后轮。
- B 方向保持 PC0 前进 RESET、后退 SET；之前方向异常由掉线引起。
- CPR 为 213203/212664/214993/214348，前进编码器符号 -1/+1/-1/+1。
- 10 ms 目标控制周期，失联 200 ms 停止，首次驱动唤醒零 PWM 等 10 ms。
- 用户现有工作区有大量未提交文件，不 reset/clean/全量暂存；不执行自动烧录或转轮测试。
- 归位 PWM 初始限制 1200…3360；误差在 CPR/200 内置零，200 ms 停稳后到位；参数待上板验证。
- 归位中任一摇杆动作、SELECT 再按、标定模式均取消；故障停止全部轮子。

## 文件职责与拟定接口

新增 `Core/Inc/wheel_home.h`、`Core/Src/wheel_home.c`：无 HAL/RTOS 的最短路径计算和归位状态机。现有 `wheel_feedback` 继续承担 16 位回绕累积；不要复制另一份回绕算法。

新增接口固定如下，字段可补充但含义不能漂移：

```c
typedef enum { HOME_MANUAL, HOME_BUTTON_HELD, HOME_RUNNING,
               HOME_WAIT_NEUTRAL } wheel_home_mode_t;
typedef enum { HOME_NONE, HOME_RECORDED, HOME_DONE, HOME_CANCELLED,
               HOME_NO_REF, HOME_STALL, HOME_TIMEOUT,
               HOME_FEEDBACK_ERROR } wheel_home_result_t;
typedef struct {
  uint32_t now_ms;
  int64_t position[4]; /* Forward-positive counts from Ctrl-owned trackers. */
  int32_t delta[4];    /* Forward-positive increments for this sample. */
  uint8_t frame_valid, feedback_valid, calibration_mode;
  uint16_t buttons;   /* Raw PS2, active low. */
  uint8_t lx, ly, rx, ry;
} wheel_home_input_t;
typedef struct {
  wheel_home_mode_t mode;
  wheel_home_result_t result;
  uint8_t reference_valid, settled_mask, owns_motors;
  int32_t pulse[4];   /* Signed, forward-positive. Zero on abort/error. */
  int64_t error[4];
} wheel_home_output_t;
/* Fixed-size state, no malloc; timestamp validity uses explicit flags. */
typedef struct {
  uint32_t cpr[4];
  int64_t origin[4], target[4], best_abs_error[4];
  uint32_t press_ms, filter_ms, neutral_ms, stationary_ms, home_start_ms;
  uint32_t last_valid_frame_ms;
  uint32_t settle_ms[4], progress_ms[4], reverse_ms[4];
  int8_t applied_direction[4];
  uint8_t settle_active[4], reverse_active[4], progress_active[4];
  uint8_t reference_valid, long_consumed, press_active;
  uint8_t filtered_select, candidate_select, filter_active;
  uint8_t neutral_active, stationary_active, have_valid_frame;
  wheel_home_mode_t mode;
  wheel_home_result_t result;
} wheel_home_t;
int64_t wheel_home_displacement(int64_t x, int64_t origin, uint32_t cpr);
void wheel_home_init(wheel_home_t *s, const uint32_t cpr[4]);
void wheel_home_step(wheel_home_t *s, const wheel_home_input_t *in,
                     wheel_home_output_t *out);
```

`owns_motors=0` 只允许在 MANUAL 且没有按钮/等待条件时出现。其余路径必须明确输出零或归位 PWM，不能沿用上一轮的未初始化结果。状态机自身不写 GPIO/PWM。

修改 `Core/Src/freertos.c`：输入采样、手柄和模式仲裁、统一输出、状态快照。修改 `Core/Src/boot_ui.c`：显示状态，不新增电机命令。`Makefile` 加源文件，`tools/test_motion.sh` 加新测试及链接依赖。

### Task 1：最短路径与逐轮控制

**Files:** 新建 wheel_home.h/.c、`tools/test_wheel_home.c`；修改 Makefile 和 test_motion.sh。

**Consumes:** 规范中的归一化计数/CPR。**Produces:** 上述三个纯 C API。

- [ ] 先写表驱动失败测试，再实现 `wheel_home_displacement`。关键断言如下：

```c
assert(wheel_home_displacement(3500, 0, 1000) == -500);
assert(wheel_home_displacement(3250, 0, 1000) == -250);
assert(wheel_home_displacement(3750, 0, 1000) == 250);
assert(wheel_home_displacement(-250, 0, 1000) == 250);
assert(wheel_home_displacement(1000, 0, 1000) == 0);
assert(wheel_home_displacement(1000000003500LL, 0, 1000) == -500);
assert(wheel_home_displacement(100, 100, 1000) == 0);
/* For cpr=1001: phase 500 -> -500; phase 501 -> +500. */
```

- [ ] 加 CPR=0 输入拒绝测试（反馈错误、零输出），不得进行除零。测试四轮不同 CPR，不能偷用 RF 默认 CPR。
- [ ] 实现触发时锁定 target，逐轮比例限幅、容差零输出和 200 ms 停稳；测试初始误差 -0.5/+0.25/-0.1/0 圈产生负/正/负/零独立输出，已到位轮子保持零。
- [ ] 测试误差跨零时先零输出 40 ms 才可反向；停稳中漂离则恢复纠偏，不重新选择整圈目标。
- [ ] 测试 500 ms 无进展、8000 ms 总超时、错误方向导致无改善全部停止；时间计算覆盖 uint32 tick 回绕。
- [ ] 按先失败后通过执行：`gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc tools/test_wheel_home.c Core/Src/wheel_home.c -o /tmp/test_wheel_home && /tmp/test_wheel_home`。

### Task 2：按键手势和控制权状态机

**Files:** wheel_home.h/.c、test_wheel_home.c。

**Consumes/Produces:** `wheel_home_step()` 输入/输出接口；状态定义与规范一致。

- [ ] 先构造有效 PS2 帧序列，低有效 bit0 按下/释放：按 100 ms 释放仅启动一次；按 2000 ms 仅记录一次，释放不启动；未设参考短按始终零输出。
- [ ] 测试按下即停止、30 ms 去抖、长按失败不转成短按、300 ms 停稳才可记录、记录后不改 UI 累计净圈数。
- [ ] 实现 MANUAL/BUTTON_HELD/RUNNING/WAIT_NEUTRAL 仲裁；在输出对象全清零后填本次结果，明确 `owns_motors`。
- [ ] 测试四个摇杆任一超 ±16、SELECT 再按、CAL、无效帧达 200 ms 均终止归位并四轮置零；单个短暂坏帧不伪造按键边沿。
- [ ] 测试取消当周期不得落回摇杆驱动；释放+居中有效持续 100 ms 后才恢复手动。重连时按住 SELECT 不能补发指令。
- [ ] 重跑 Task 1 命令并确认所有状态测试通过；不要通过增加阻塞延时实现手势。

### Task 3：真实任务接入与硬件反馈

**Files:** freertos.c、`tools/test_ctrl_task.c`、host_stubs、test_motion.sh，必要时新增 `tools/test_home_task.c`。

**Consumes:** 状态机 API、现有 `wheel_feedback_init/update`。**Produces:** 真实任务里的唯一电机输出路径、线程安全显示快照。

- [ ] 先扩展主机模拟定时器支持 TIM2/3/4/5、编码器位置与方向，加入集成失败用例：后轮读取 TIM5/TIM4、B 前进 PC0=0、后退 PC0=1、四轮独立 PWM。
- [ ] Ctrl ready 后初始化四个反馈实例；每循环读 CNT/update，再乘前进符号。所有手动、按键、归位、等待模式持续采样；采样间隔 >50 ms 标记失效并使参考无效。不要从 `boot_ui_diag.position_counts` 读取归位位置。
- [ ] 采样失效后立即零输出、丢弃旧参考，四个反馈实例以当前 CNT 重新初始化；停稳重新记录才能再次归位。故障解除并满足居中/释放条件后仍允许普通手动驾驶，不能永久锁死。重初始化只是 Ctrl 私有计数，不写硬件 CNT 或 UI 累计位置。
- [ ] `ps2_poll` 后把有效性、真实 tick、全部摇杆及 buttons 传状态机。先检查停机条件，再做归位或手动输出；统一使用现有 motor table/SLP 唤醒流程。归位无效帧不能重复使用旧按键状态生成新事件。
- [ ] 将驱动输出整合到单一函数，禁止普通摇杆分支覆盖归位结果；所有零/取消路径显式写四路 CCR=0。把实际施加 PWM 和换向等待与状态机时序保持一致，唤醒期间不计入无进展时间。
- [ ] 重连配置前先停机；配置阻塞超过计数期限使参考失效，不在错误位置继续归位。CAL 优先于所有按键；上电自检前不动电机。
- [ ] 用紧凑状态快照接口 `void wheel_home_get_status(wheel_home_output_t *out)` 发布状态，临界区只做数据复制。UI 不直接读无保护的 64 位共享变量。
- [ ] 回归 `bash tools/test_motion.sh`，保留唤醒、失联 200 ms、tick 回绕、CAL 禁运动、PS2 模拟帧、自检、转速/圈数等现有验证；新增链接源文件必须同步到脚本。

### Task 4：UI 与交付验证

**Files:** boot_ui.c、UI 主机测试、knowledge/pinmap.md、功能使用说明与实测记录。

**Consumes:** `wheel_home_get_status()` 快照。**Produces:** 可观察状态、完整构建、待上板固件和测试清单。

- [ ] 先写 UI 状态文本和缓冲区边界测试，再添加顶栏短状态；保留 RPM 与累计净圈数含义，不挤掉 VDD、不突破 128 像素，不增加归位显示页阻塞控制。
- [ ] 运行 `bash tools/test_motion.sh` 和 `python3 tools/test_font_convert.py`；若该字库文件要求 pytest，按文件现有入口执行，记录命令与结果。对新增纯 C 测试使用 ASan/UBSan。
- [ ] 构建：`make -j4 GCC_PATH=/home/snow/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin`。用 `git -c core.whitespace=cr-at-eol diff --check` 检查更改。审查仅相关变更，不全量暂存或提交用户未提交工作。
- [ ] 将具体新增文件、测试结果、剩余上板步骤交给用户；功能未上板时明确标记，不声称“已精确归位”。

## 上板操作顺序：由现场用户配合，不能在烧录后自动触发

1. 先备份当前 Flash，确认主板稳定连接；保持零输出写入并验证，再读回比较。符号地址从新 ELF 的 nm 输出取，不能沿用本记录旧调试地址。
2. 只读采集 SELECT 按下/释放的 buttons，确认确实 bit0 低有效；若不是，修正按钮掩码及主机测试后重新构建，不能带着猜测触发电机。
3. 四轮悬空、用户确认空间和摇杆居中。未设参考短按应不动并提示 NO REF。
4. 手动摆好四轮标记，长按记录。验证长按释放不会自动转；累计净圈数显示不被清零。
5. 用诊断轮掩码先只允许 A 归位，手转约 1/4 圈测试最短方向、减速和到位，随后 B/C/D 分别验证；测试掩码默认全轮，在现场使用的临时值必须记录并最终恢复，不能偷偷留下单轮固件。
6. 四轮分别偏移 +1/4、-1/4、+1/2、0 圈，短按归位；记录目标计数、实际结束计数/误差、用时和机械标记。然后测跨整圈（3.5 圈）、负方向及奇数 CPR 情况。
7. 检查拨杆取消、再按取消、手柄失联、卡轮/无反馈、超时均停全部轮子，取消后不突然执行手动输出。卡轮测试不长时间强堵转，软件模拟先覆盖故障。
8. 再次观察正常手柄响应；归位功能不得把普通轮询退回几十毫秒额外延时。报告机械误差和参数调试记录，不能把编码器容差当机械精度。

### Task 5：自检逐轮异常结果

**Files:** boot_ui.c、boot_ui.h（或新增 `motor_health.h/.c`）、test_selfcheck.c、UI 测试。

**Consumes:** 现有正/反转 500 ms 的回绕净计数。**Produces:** 稳定保存的逐轮自检结果，供引导页/主页面使用。

- [ ] 先写失败测试：两向正常、前向缺反馈、后向缺反馈、双向缺反馈、±1 抖动、主动跳过。要求每轮保存前后两段净计数，不能只留下一个汇总 bool。
- [ ] 新增 `motor_test_result_t`，字段至少包含 `tested`、`forward_ok`、`reverse_ok`、`forward_counts`、`reverse_counts`、`reason_mask`。跳过时 `tested=0`，不得 PASS；原因掩码区分前/后缺反馈和采样异常。
- [ ] 修改 `ui_run_with_feedback()` 返回本段净计数和是否满足运动阈值；继续在运行期间多次读 CNT，而非只读开始/结束造成漏回绕。
- [ ] 将结果传入 OLED，失败轮子明确显示 `NO FB/FWD ERR/REV ERR`，保留到主页面；自检结束文案用 CHECK END，全部通过才显示 ALL OK。不用“离线”伪装已经定位硬件原因。
- [ ] 运行 `bash tools/test_motion.sh`。更新测试链接时保留旧自检卡轮、单方向失败、最终全停、CAL 跳过和时长断言。

### Task 6：5 秒 USER 入口与手动对齐页

**Files:** boot_ui.c、freertos.c、boot_ui.h/drive_ctrl.h、test_ctrl_task.c、UI 测试。

**Consumes:** Task 5 结果、Task 1–3 参考记录状态机。**Produces:** 自检后的结果窗口、可读取 SELECT 的手动初始位页面。

- [ ] 先写确定性 tick/按键测试：无 USER 事件 5000 ms 后进入主页面；USER 释放后新按下稳定 30 ms、事件 <5000 ms 才进入手动页；在 5000 ms 或之后不进入；初次启动按钮一直按住不能误触发；tick 回绕结果一致。
- [ ] 增加启动阶段状态与控制权移交。检查实际代码中的 boot_ui_ready/drive_calibration_mode 分支：引导阶段仍轮询 PS2，任何摇杆/PWM 命令均被零输出覆盖，自检任务移交前 Ctrl 不能驱动。
- [ ] 自检收尾零输出、SLP 睡眠并绘制结果页，之后启动窗口计时；每 10–20 ms 处理 USER，显示按需刷新，不用阻塞五秒等待。
- [ ] 手动页持续记录编码器变化但不建立自动参考。SELECT 在有效、停稳、居中条件下长按两秒才由 Ctrl 记录四轮位置，收到成功应答后才显示 REF SET。
- [ ] 测试进入页面前 SELECT 已按住必须先释放；长按释放不得归位；未停稳/手柄无效不能记录；用户手转十几秒仍留在手动页；USER 退出不记录。退出后等待按钮释放、居中 100 ms，故障未清除则继续零输出。
- [ ] 自检失败仍可进入页面，但记录不清故障；超时到主页面保留具体故障且禁止驾驶/归位。NO REF 与 MOTOR ERR 分开，不能让无故障但未记录参考的用户也不能普通驾驶。
- [ ] 添加 UI 文字/布局/状态切换测试，运行 ASan/UBSan 和完整回归。

### Task 7：持续运动反馈监测与故障锁存

**Files:** 新增 motor_health.h/.c（若 Task 5 尚未新建）、`tools/test_motor_health.c`；修改 Ctrl 唯一输出路径、UI 快照、test_motion.sh。

**Consumes:** 真实时间、实际已施加 signed PWM、SLP/换向等待状态、归一化净计数。**Produces:** 每轮运行故障码与全轮停止请求；不直接写 PWM。

- [ ] 先写纯 C 失败测试：实际 PWM>=1200，300 ms 启动宽限后，300 ms 窗口不足 64 净计数 -> NO FB；正常连续正/反反馈 -> 不报错；反馈符号反且幅度>=64 -> DIR ERR；抖动净零 -> 不能通过。
- [ ] 增加输出零、PWM<1200、睡眠、换向、手动对齐、自检未移交时不判运行故障的测试。不要把不满足检测条件显示为“确认在线”。
- [ ] 测试符号改变重启宽限，tick 回绕正确；输出要求零时立即回报停止，不等待 UI 刷新。
- [ ] 实现每轮窗口/故障锁存。一个轮子故障使全部归位或手动命令归零并睡眠；下一帧出现计数也不能自动解除。清除仅由用户处理后主动重新自检且四轮通过，不能使用 SELECT 记录操作清故障。
- [ ] 将故障出口与归位卡轮/超时/采样异常统一仲裁；测试没有普通摇杆分支在故障当周期重新写入非零 PWM。
- [ ] 整体构建和回归后，更新使用说明：NO FB 是反馈异常而非已证实离线；参数未实测前不得声称已经精准检测任何负载下卡轮。
- [ ] 上板由用户配合：先只读核对两种按键、验证页面零输出，再测试故障显示、接线修复后的重新自检、记录和最短归位，保留原始诊断数据。
