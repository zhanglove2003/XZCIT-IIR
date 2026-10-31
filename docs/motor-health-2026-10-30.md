# 本机反馈监测与故障锁存（motor_health）

2026-10-30 · 对应 plan `docs/superpowers/plans/2026-10-30-wheel-angle-home.md` Task 7

## 它做什么

`Core/Src/motor_health.c` 每个控制周期（10 ms）看一次“给了多少 PWM、编码器实际走了多少”：

| 判定 | 条件 |
|---|---|
| **NO FB** | PWM ≥ 1200，300 ms 启动宽限后，300 ms 窗口内净计数 < 64 |
| **DIR ERR** | 反馈方向与施加方向相反，且反向幅度 ≥ 64 |
| **不判** | PWM < 1200、零输出、驱动睡眠、换向后的宽限期、手动对齐页、手柄失联、自检未移交 |

任意一轮出现 NO FB / DIR ERR，就**锁存**该轮：`stop_all = 1`，Ctrl 把四轮 PWM 全部清零。顶栏状态文本变成 `FLT`（优先级高于 HOME / RDY / NORF 等所有普通状态）。

## 它不做什么（重要）

**NO FB 只说明“给了力却没看到反馈”，不等于已证实该电机断线、脱离或已损坏。**
负载过大、轮子被卡住、驱动供电不足、编码器线松、齿轮打滑都会表现为 NO FB。
系统不会声称“已精准检测到任何负载下的卡轮”；它只承诺“在 PWM 足够大且持续 300 ms 的情况下，反馈窗口不足 64 计数”。

DIR ERR 同理：只比符号，不判断闭环正确性。

## 怎么解除

锁存之后**不会自动恢复**——即使反馈看起来正常了也继续禁止输出。

唯一的解除路径：**用户主动重新自检，且这一轮四轮全部通过**
（`boot_ui_selfcheck_seq` 递增且 `boot_ui_selfcheck_passed != 0`，接线在 `Core/Src/freertos.c:523-529`）。

- 长按 SELECT 记录参考**不会**清故障。
- 手动对齐页里记录参考**不会**清故障。
- 重新上电会清（内存状态归零）。

## 上板验证要点

1. 正常行驶（PWM > 1200）：顶栏保持原状态，不出现 `FLT`。
2. 拔掉任一编码器信号线后推杆：宽限 + 窗口内顶栏变 `FLT`，四轮立刻停。
3. 插回编码器：顶栏仍是 `FLT`，推杆不动（锁存未解除）。
4. 重新上电并走完自检：`FLT` 消失，恢复行驶。
5. 低速蠕行（PWM < 1200）与原地换向：不应出现 `FLT`。

## 主机侧验证

```bash
bash tools/test_motion.sh
```

其中三组与本功能相关：

- `motor health: PWM floor / grace / window / DIR ERR / jitter / latch / clear OK`（`tools/test_motor_health.c`）
- `ctrl motor health latch / zero output / selfcheck clear OK`（`tools/test_ctrl_task.c` 的 `fault` 场景：满输出零反馈 → 锁存清零 → 自检通过 → 恢复）
- `wheel home ...`（含 `FLT` 优先级与缓冲区边界断言）
