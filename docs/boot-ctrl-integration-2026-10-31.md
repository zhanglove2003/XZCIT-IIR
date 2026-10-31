# 真实 UI/Ctrl 接口集成测试（2026-10-31）

结论：先修复两项集成失败，再决定上板。没有修改生产代码，没有操作主板。

## 测试方法与边界

新增 `tools/test_boot_ctrl_integration.c` 和 `tools/test_boot_ctrl_integration.sh`。

运行：`bash tools/test_boot_ctrl_integration.sh`。

链接真实 `freertos.c`、`boot_ui.c`、`manual_ref.c`、`wheel_home.c`、反馈和健康检测模块，保留真实 `boot_ui_manual_begin/run/active/take_record_request/ack` 与 `wheel_home_get_status`。图形绘制和字库也使用实际代码。

仅模拟硬件接口、手柄输入、定时器和任务调度。使用 ucontext 提供两个任务栈，通过 osDelay 调度，任务同时可运行时先运行 Ctrl；OLED 刷新模拟 20 ms，手柄配置模拟 120 ms，单次读取模拟 3 ms。模拟电机计数按现有四轮实际映射、前进符号更新。使用 UBSan；ucontext 切栈不适合直接宣称 ASan 完整支持。

这是主机真实软件接口集成测试，不是 FreeRTOS 内核时序认证，也不能替代上板机械验证。

## 当前实测结果

```text
PASS scenario=0 window=1 entry=0 record_ack=0 ref=0 ready=1
FAIL scenario=1 window=1 entry=0 record_ack=0 ref=0 ready=1
FAIL scenario=2 window=1 entry=1 record_ack=0 ref=1 ready=1
```

三项场景均未检测到引导窗口/手动页中的意外电机输出；Ctrl 没有调用 OLED。场景 0/1 从真实 `boot_ui_run` 和 `StartCtrlTask` 开始，ready 初始为 0。场景 2 隔离手动页记录接口，模拟已完成控制移交，ready/pass 初始为 1，调用真实 begin/run，不替换任何记录或快照接口；它不声称完整启动已经成功。

### 场景 0：无人操作窗口超时，通过

真实启动、自检、结果窗口，5 秒无人按 USER，进入主页面。确认没有自动设置参考。

### 场景 1：真实启动后 USER 入口，失败

结果窗口开始后按 USER，失败后又补按一次，始终没有进入手动页。根因仍是 `freertos.c:542-545` 在 ready=0 时跳过状态快照发布。`boot_ui_ready` 只有 `ui_main_page()` 设置，结果窗口早于该函数。链路虽然已经读取有效帧，UI 接口取到的快照仍无效。必须分离输入可用和驾驶使能，不能简单提前放开普通摇杆输出。

新增回归要求：从 ready=0 的真实启动流程到 USER 进入手动页，测试应变绿；引导阶段任何摇杆动作都不得出力。

### 场景 2：手动页 SELECT 记录与应答，失败

在有效移交条件下，真实 USER 输入能进入手动页。长按 SELECT 后归位模块将参考设为有效，但 manual_ref 未收到成功确认，页面仍不是 REF SET。

`boot_ui.c:692` 仍以 `home.settled_mask!=0` 判断是否可记录。该字段在 MANUAL/BUTTON_HELD 分支被清零，不能代替四轮连续静止。页面因此没有发出正常的记录请求；独立 wheel_home 长按路径却保存参考，造成两个状态机的结果不一致。

修复应统一记录手势/请求/应答：UI 显示与 Ctrl 捕获四轮参考使用同一次操作，Ctrl 核对所有轮子停稳、反馈有效、摇杆居中后记录并应答。参考已设有效与页面提示成功必须一致。

## 既有测试与构建

- `bash tools/test_motion.sh`：现有 14 个输出场景全部通过，说明原模块测试不足以覆盖上述集成。
- 用户文件为 51452 字节，MD5 `989dbfd9bf6786af5bbeebb4c490b861`。
- 检查 build ELF 的 `.comment` 后确认编译器为 GCC 10.3.1。
- 使用相同 GCC 10.3.1 在独立目录 `/tmp/events-deepseek-review3-gcc10` 干净构建：51452 字节，MD5 完全一致，0 warning/0 error，与现有 bin 逐字节一致。
- GCC 14.3.1 的独立构建为 51348 字节，MD5 不同，这是不同编译器产物；不能仅以大小/哈希不同判断文件过期。此前报告中的产物不一致结论缺少编译器条件，本次明确更正。此前版本的历史文件未按同一编译器重新验证，不能据此反推它们一定匹配或过期。
- `git diff --check` 发现 Makefile 原有生成文本的多处行末空格，本次未修改 Makefile。新增测试文件没有该问题。

## 交付规则

两项失败保留为真实验收条件，不改为“期望失败通过”，不删除断言让脚本变绿。当前未将它们并入旧 test_motion.sh，提供独立脚本供修复复测。修复后应同时运行两个脚本，再使用同一编译器干净构建并核对产物，之后由用户决定是否烧录。
