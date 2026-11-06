# 自检输出权限复查（2026-10-31）

## 结论

用户提供的本轮修复证据已独立确认，但仍有一个阻止上板验收的 P1 问题：Ctrl 在自检期间反复写零 PWM，截断自检动作。此次仅修改主机集成测试并增加报告，没有修改生产代码，也没有烧录或操作电机。

## 已确认通过的证据

- 原三项真实接口集成测试全部通过：窗口超时、USER 入口、SELECT 记录确认。
- 原 `bash tools/test_motion.sh` 的 14 个输出场景通过。
- 使用 GCC 10.3.1 在独立目录 `/tmp/events-deepseek-review4-gcc10` 全新构建：0 warning/0 error，bin=51868 字节，MD5 `b945d2419effe81c8787618dbcc9908f`，与 build 产物逐字节一致。
- 手动页测试只按一次 USER 是正确的：第二次完整按压属于退出操作，不能用它测试“保持页内并记录”。

## [P1] 写零也是写电机，自检期间仍被 Ctrl 截断

代码位置：`Core/Src/freertos.c:615-629,715-716`；`ctrl_apply` 的零输出分支。对应自检在 `Core/Src/boot_ui.c::ui_motor_selfcheck()`。

1. 本轮修复删除了 Ctrl 的启动 continue，让启动期间发布状态快照。这个方向合理。
2. 启动阶段仲裁将四路 command 设为零，但后面仍无条件调用 ctrl_apply。
3. 即使 awake=0，ctrl_apply 也遍历四个通道并把 CCR 写成零。
4. 同时 UI 自检把当前轮 CCR 写成非零，计划维持正向 500 ms、反向 500 ms。下一次 Ctrl 循环即将其清零。

所以“启动时禁止驾驶”不能用“Ctrl 持续向硬件写零”替代。自检持有电机时，Ctrl 不应写 CCR、SLP、启动或停止 PWM；正常 handoff 后才由 Ctrl 独占电机。仅发布输入/位置快照不需要修改这些寄存器。

## 新增复现测试

在真实双任务集成测试中增加 scenario=3：保持完整启动和自检，累计每个轮子实际有非零 PWM 且 SLP 使能的时间。要求至少 `2 * SELFCHECK_RUN_MS`，当前为 1000 ms；OLED 模拟耗时只会延长自检，因此这是保守下限。

运行 `bash tools/test_boot_ctrl_integration.sh`：

```text
PASS scenario=0
PASS scenario=1
PASS scenario=2
selfcheck powered_ms A=10 B=10 C=10 D=10 minimum_each=1000
FAIL scenario=3
```

这是模拟调度/硬件下的真实代码接入结果，不是主板电机通电时长实测。原自检单模块测试没有同时运行 Ctrl；前三项集成测试也没有检查通电时长。因此此前测试可以通过。

## 修复要求

- 明确自检电机权限与引导/驾驶电机权限，不要用 boot_ui_selfcheck_passed 同时表示“结果通过”和“已完成控制移交”。自检失败后也需要可显示/可记录但禁止运动的明确阶段。
- 自检尚未移交时，只读取手柄、采样和发布状态。禁止 ctrl_apply、ctrl_motors_sleep、HAL_TIM_PWM_Start/Stop 修改自检输出。
- UI 自检结束必须先四轮 PWM=0、SLP 低，再发布移交事件。结果窗口/手动页由 Ctrl 维持零输出，正常页面才允许驾驶。
- 驱动权限变量、移交时间和 passed 状态应分别明确；链路重连初始化和 CAL 分支也必须服从同一权限。
- 保留原三项集成和新第四项通电时长断言，不删检查、不将失败变成预期通过。必要时增加每轮前/后方向分别持续 500 ms 的断言，以及自检期间接收器失联/重连不影响电机输出的测试。
- 修复后重新运行两个脚本，使用同一编译器构建并核对产物，之后再决定烧录。

## 当前硬件连接状态

本轮只尝试了一次 100 kHz HLA 只读连接：ST-LINK 可识别，电压 3.238959 V，OpenOCD 报 init mode failed。没有继续改频率、复位配置或写 Flash。

不能把 ctrl_motors_sleep() 本身当作 SWD 失联的已知原因，它只操作电机输出，未修改 SWD 引脚配置。报告中“跑飞”也不是本轮读到的芯片状态。此前读回已确认 Flash 启动区为空，必须保留该证据，恢复连接后再次核对再解释当前启动行为。

---

# 修复记录（同日，第二轮）

## 已落地

权限模型拆成三个互不代替的量（`Core/Inc/boot_ui.h`）：

| 变量 | 含义 | 谁写 |
|---|---|---|
| `boot_ui_motor_owner` | 当前谁独占 CCR/SLP/DIR | UI：自检开始置 `OWNER_UI`，移交后置 `OWNER_CTRL` |
| `boot_ui_selfcheck_passed` | 自检结果是否通过（只决定能否清故障锁存） | UI，自检结束时写 |
| `boot_ui_drive_enabled` | 是否允许驾驶（结果页/手动页为 0，主页面置 1） | UI，`ui_main_page()` 写 |
| `boot_ui_handoff_seq` | 移交发生轮次，Ctrl 据此复位自身输出认知 | UI 移交时 +1 |

Ctrl 侧（`Core/Src/freertos.c`）：

- `ctrl_link_service()` 的 `allow_output` 改为 `boot_ui_motor_owner == CTRL`，不再用 `selfcheck_passed`。
- 新增**电机归属闸门**：`owner != CTRL` 时整段跳过 `ctrl_apply()`，连写零都不做；`awake` 强制为 0。
- 移交轮次变化时复位 `awake`、`s_ctrl_tracker_ready`、`s_ctrl_last_pwm[]`，避免把 UI 留下的 CCR/SLP 当成自己的状态。
- CAL 分支的 `ctrl_motors_sleep()` 也加了归属判断。
- 输出仲裁改用 `boot_ui_drive_enabled`，不再用 `selfcheck_passed`。

UI 侧（`Core/Src/boot_ui.c`）：

- 自检结束后、开 5 s 窗口之前，先把四轮 `CCR=0`、`HAL_TIM_PWM_Stop`、`SLP` 拉低，**然后**才发布 `owner=CTRL` 并 `handoff_seq++`。
- `ui_main_page()` 进入时置 `boot_ui_drive_enabled = 1`。

## 验证结果

```
PASS scenario=0
PASS scenario=1
PASS scenario=2
selfcheck powered_ms A=1160 B=1160 C=1160 D=1160 minimum_each=1000
PASS scenario=3
```

- `bash tools/test_boot_ctrl_integration.sh` → EXIT=0
- `bash tools/test_motion.sh` → 14 组全绿
- 原三项集成测试保留未改，scenario=3 的通电时长断言保留未放宽。

## 新增的两条断言（按报告要求）

1. **每轮前/后方向分别持续 500 ms**：`tools/test_boot_ctrl_integration.c` 新增
   `fwd_ms[4]` / `rev_ms[4]`，只统计“非零 PWM 且 SLP 使能”的时段，分别要求
   `>= SELFCHECK_RUN_MS`。总量够但方向被切碎同样判失败。
2. **自检期间接收器失联/重连不影响电机输出**：scenario 3 的 `ps2_poll()` 每 700 ms
   返回 300 ms 的 `PS2_FRAME_BAD`，全程 6 次，覆盖 Ctrl 的失联关驱动与重连
   `HAL_TIM_PWM_Start + ctrl_motors_sleep` 两条会写输出的路径。

修复后的输出：

```text
selfcheck fwd_ms A=580 B=580 C=580 D=583 rev_ms A=583 B=580 C=580 D=580 min_each=500
selfcheck powered_ms A=1163 B=1160 C=1160 D=1163 minimum_each=1000
PASS scenario=3
```

## 变异验证（证明断言不是空转）

把 `Core/Src/freertos.c` 的归属闸门 `if (boot_ui_motor_owner == BOOT_UI_OWNER_CTRL)`
临时改成 `if (1)` 后重跑：

```text
selfcheck fwd_ms A=10 B=45 C=4 D=9 rev_ms A=4 B=10 C=5 D=120 min_each=500
selfcheck powered_ms A=14 B=55 C=9 D=129 minimum_each=1000
FAIL scenario=3
```

恢复闸门后回到 580/1160 ms 并 PASS。说明该断言确实能捕捉“别人写零截断自检”。

## 构建

干净构建：0 warning / 0 error，`build/hal_f407_mecanum_4motor_src.bin`
**52092 字节，MD5 `fbc9b5406a3a059312efd5f558c274eb`**。

## 仍未验证

- 上板：SWD 连接自上一轮起 `init mode failed`（ST-LINK 可识别，电压 3.238 V）。移交时序与通电时长均为主机模拟结果，需上板复核。
