# DeepSeek 最新版本复查与烧录恢复记录

日期：2026-10-31。结论：暂不烧录此版本。用户要求先复查，再决定是否烧录。本次没有修改固件源码，没有擦除或编程 Flash。

## 主板当前状态

重新上电后，OpenOCD 0.11.0 HLA 在 100 kHz 成功连接，无需 NRST。

- 芯片标识 `DBGMCU_IDCODE=0x10076413`。
- Flash 容量寄存器 `0x0200`，即 512 KiB。
- FLASH_SR=0，FLASH_CR=0x80000000，FLASH_OPTCR=0x0fffaaed。
- RDP 字节为 0xAA，Level 0。
- 启动向量读出全 FF。完整 512 KiB 备份进一步确认前 64 KiB 全 FF；64–128 KiB 还有 33902 个非 FF 字节。不能将之前烧录失败解释为 Flash 完全未改动。
- 该状态没有有效的应用启动向量，足以解释上电后原 OLED 应用不启动；未发现能证明 OLED 硬件损坏的证据。

当前 Flash 备份：`/mnt/data/project-ai/Embedded_Development/firmware-recovery/2026-10-31/flash-after-failed-write-512KiB.bin`，524288 字节，SHA256 `b6a96525f721fce3d10b85c99a866f81f1fc120cb9f38d009bb66f2350d7fe36`。

恢复候选另存同目录：`known-tested-manual-drive-44132.bin`（此前现场验证版本）和 `deepseek-current-51052.bin`（用户报告中的构建产物）。不能在未确认候选前自动写入。

## 软件验证

- `bash tools/test_motion.sh` 全部通过，14 个输出场景。
- 独立构建目录 `/tmp/events-deepseek-review2-build` 完整构建成功，无编译警告/错误；text=50828，data=128，bss=50984。
- 本次源码重新构建出的 bin 为 50964 字节，SHA256 `90c8b4119478dce4858edb356698156be4204d7aa41ffd3cd326c352a5b82750`。
- 原 `build/...bin` 为 51052 字节，SHA256 `a2d686750b522ce8912602667da87f95025611381a0c9c57d941136e2caff783`。两个文件不同；现有 build 产物不能直接当作本次审查源码的产物。下一次烧录应使用修复后干净构建并校验的产物。
- 上次“自检失败禁止输出”和“全零输出让驱动睡眠”已增加代码处理。其余问题如下。

## 仍需修复的问题

### 1. [P1] 5 秒入口仍拿不到有效快照

`freertos.c:478-481` 仍在 ready=0 时跳过状态机和快照发布。虽然前面新增 PS2 读取，但 `out.frame_valid` 的发布仍在此 gate 之后。`boot_ui.c:583` 只有进入主页面才置 ready，而结果窗口在它之前。因此 UI 读到的 home.frame_valid 始终为 0，`manual_ref_step` 不接受 USER。读取手柄提前不等于快照和控制阶段已经正确接入。

修复须分开“自检持有电机”“引导页可读输入”“正常驾驶允许输出”。添加真实启动→结果窗口→USER→手动页的集成测试，不使用空桩替换页面接口。

### 2. [P1] 提前读取手柄时，同时提前修改了电机输出

`freertos.c:474-480` 在 ready gate 前调用 `ctrl_link_service`。该 helper 的 `410-415` 初始化路径会启动 PWM 并执行全轮 sleep；`440-444` 失联路径也会清零 CCR/SLP。自检期间如果接收器未连接/重连，Ctrl 会抢先关闭 UI 自检正在测试的电机，破坏自检结果。

修复须将只读手柄服务与电机操作分开；自检正式移交前，Ctrl 不得写 PWM、SLP 或启动/停止电机通道。

### 3. [P1] 超时恢复分支仍不可达，并错误保留旧参考

`wheel_home.c:509-537` 超时后不更新 last_position_ms，也不重置 have_valid_frame。新增 recovering 变量不能解决入口一直超时的问题。旧 reference_valid 现在还被保留，违反丢回绕后参考必须作废的要求。

当前源码的临时主机测试：t=0 正常，t=60/70/80 ms 均 last_position_ms=0、recovering=1、reference_valid=1。

修复：停止、作废参考、重建 Ctrl 私有计数跟踪，再恢复正常新样本；重新记录后才能归位。

### 4. [P1] 经过容差区后仍绕过换向等待

`wheel_home.c:350-352` 进入容差仍清 applied_direction。临时主机测试（CPR=1000，target=100）：t=0/position=0/PWM=+1344；t=10/position=100/PWM=0；t=20/position=110/PWM=-1200，reverse_active=0。仍未满足 40 ms 零输出等待。

修复：用最后实际驱动方向和零输出时间判断，不能在进入容差时丢失方向历史。

### 5. [P2] 手动页参考记录仍不能完成，新增入口检查也不足

`boot_ui.c:692` 仍以 settled_mask!=0 判静止，但非归位状态 mask 始终为零。因此页面不会请求记录。`wheel_home.c:267-269` 的新增记录函数只允许 MANUAL，而正常长按 SELECT 已进入 BUTTON_HELD，仍会拒绝。独立长按分支可能悄悄记录，但 UI 不会显示应答成功。

此外该新增函数本身不核对 feedback_valid/连续停稳/CPR，只核对 frame_valid；不能把安全前置条件全部假定为 UI 已检查。应提供 Ctrl 单一记录入口，明确检查全部四轮、居中、有效反馈、连续停稳，并使长按手势只执行一次记录。

### 6. [P2] 正常释放状态下的第一次 USER 按下仍被忽略

`manual_ref.c:105-109` 第一次调用就清 window_armed，但初始 filtered=0 时持续释放不会产生释放事件，无法重新置位。窗口打开时 USER 已释放的正常场景仍需第二次按下。

重新编译当前源码复现：窗口打开、保持释放 100 ms、按下保持 150 ms，结果 WAITING，预期 MANUAL。不要只测试“窗口打开时已经按住 USER”的情况。本地 USER 也不应依赖无线手柄有效帧。

### 7. [P2] 重连只等待三帧居中，未等待 SELECT 释放与 100 ms

`freertos.c:425-438` 收到三帧居中就允许后续帧，约 30 ms，不检查 SELECT 释放。状态机 MANUAL 的失联分支仍不进入 WAIT_NEUTRAL。比上一版有所改善，但不符合设计的释放/居中持续 100 ms。

### 8. [P2] UI 状态机与 OLED 仍由两个任务同时调用

`freertos.c:472` 与 `boot_ui.c:873` 都调用 `boot_ui_manual_run()`；该函数会修改同一个 manual_ref 状态并执行 `oled_flush`。在修复有效快照后，两任务会同时推进去抖/长按、绘制同一缓冲并发送 OLED 数据，且高优先级 Ctrl 因屏幕传输阻塞。

修复：UI 单任务推进本地 USER、页面与 OLED；Ctrl 只发布输入/运动快照并处理记录请求。请求/应答用清晰消息或短临界区交换，Ctrl 不调用显示函数。

## 复查门槛

先修复以上问题，并增加对应真实集成测试。原测试把 boot_ui_manual_run/active/take_record_request/ack 全替换为空函数，仍然不能证明真实引导流程正确。通过后重新干净构建，核对源码与产物，再请用户决定上板。当前保持不烧录。
