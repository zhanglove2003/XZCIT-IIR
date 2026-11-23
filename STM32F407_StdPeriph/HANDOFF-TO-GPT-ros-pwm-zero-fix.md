# C50C ROS2 直行控制 PWM 恒零故障 — 移交修复任务提示词

> 用途：把本文件完整复制给另一个 AI（GPT）执行修复。本文件只做约束与证据
> 交接，**不包含确定的修复方案**——根因调查尚未完成，禁止跳过调查直接改代码。

## 0. 一句话症状

C50C 四驱底盘，ROS 2（Humble，OrangePi）直行（Phase 1）控制链路，固件侧
四路 PWM 恒为 0，轮子不转。同一套固件/ROS 进程，2026-11-09 13:30 左右能
转，之后不能转，期间没有人为重新上下电（是否重新烧录了固件待核实，见
第 4 节第 6 条）。

## 1. 架构速览（三层，必须都打开才有非零 PWM）

### 1.1 固件（STM32F407，FreeRTOS）

源码根：`/mnt/data/project-ai/Embedded_Development/ROS2/work/2026-11-08-50pct-migrate`
构建：`make -f Makefile`（默认已含 4 个宏：`REAR_WHEEL_DEBUG
ROS_STRAIGHT_DRIVE FOUR_TURN_DEBUG FRONT_TEST_CHANNEL=6`，见 `Makefile:25`）
PWM 输出口：`BALANCE/balance_task.c` 第 1152-1196 行，`_4WD_CAR` +
`REAR_WHEEL_DEBUG` + `ROS_STRAIGHT_DRIVE` 分支。

**7 条 `ros_ready` 门（全部为真才有非零 PWM，任何一条为假都静默清零，外部
无报错）：**

```c
int ros_ready = SysVal.HardWare_Ver == V1_0 && GamePadInterface != NULL &&
                Get_Control_Mode(_ROS_Control) &&
                (uint32_t)(tick - robot_control.ros_cmd_tick) <= pdMS_TO_TICKS(ROS_CMD_TIMEOUT_MS) &&
                !charger.AllowRecharge &&
                !robot_check.DeepCheck && SysVal.Time_count > CONTROL_DELAY + 230;
```

另有 `steer_debug_step_duty()`（`BALANCE/steer_debug.c`）自带的 100ms
中立臂定时器（`ARM_NEUTRAL_MS`），`ros_ready` 为真后还要先经过一次
`(0,0)` 中立窗口才能真正出力；`drive_mode` 在 `_ROS_Control` /
`_PS2_Control` / 0 之间切换会清空该状态机重新计时。

**关于 `GamePadInterface != NULL` 这一条（已查实，不是本次卡点的首要怀
疑对象）：** `HARDWARE/bsp_gamepad.c:48` 定义
`GamePadType_t* GamePadInterface = 0;` 初值 NULL，但
`USER/system.c:52-73` 的 `V1_0` 分支里调用 `PS2_Init()` →
`PS2_Key_Param_Init()`（`HARDWARE/ps2_classic.c:88-90`）会把它赋值为
`&Classic_PS2Gamepad`（非 NULL）。只要板子真的被识别为 `V1_0`
（MPU6050 存在），`systemInit()` 启动阶段就会把 `GamePadInterface` 置
为非 NULL，和有没有真实手柄插着无关。**结论：如果
`SysVal.HardWare_Ver == V1_0` 为真，`GamePadInterface != NULL` 几乎必
然也为真；这两条应合并看待，不是两个独立失败点。** 若板子实际是
V1_1/V1_2（ICM20948），`USER/system.c` 的那个分支完全不初始化
`GamePadInterface`、也不设 `V1_0`，`ros_ready` 会永久为假——这是需要
现场核实的关键硬件事实（见第 4 节第 1 条）。

### 1.2 ROS 侧（OrangePi，`c50c_uart3` 包，`chassis_cmd/` 子模块）

本机对应源码目录：
`/mnt/data/project-ai/Embedded_Development/ROS2/work/ros-straight-migrate/c50c_uart3/c50c_uart3/chassis_cmd/`

板端实际跑的路径（用户原话，本机不可达，需现场核实其内容是否与本机源
码一致）：`/home/orangepi/ros2_ws/src/c50c_uart3/...`。

关键文件与各自职责：

- `ros_input.py`——`RosStraightInput`：把 ROS `Twist`/`Bool` 消息转成
  命令链调用，默认 `enabled=False`；自带"许可"机制
  （`_nav_licensed`/`_demo_licensed`），与仲裁器判断分开；发布者数量
  未知/`0`/多于 1 时不放行非零，且作废已有许可。
- `bridge.py`——`StraightDriveBridge`：接线 ROS 输入 + `Arbiter` +
  串口 fd；**额外的会话门**：非零输出要求 (a) `Arbiter` 已从真实状态
  帧里由 `SessionHandshake` 读出 session_id，(b) 同一会话内已经成功写
  出过至少一帧零速帧。这两条缺任何一条，桥会保持静默（不发送任何字
  节），即使固件侧 `ros_ready` 已经为真。
- `arbiter.py`（580 行）：限速、过期、停车优先、来源冲突判定——未读
  完，是尚需核查的模块。
- `handshake.py`（182 行）：从真实状态帧产生 session 证据——未读完。
- `chain.py`（124 行）：周期 tick、编帧。

### 1.3 两层门的叠加效应

固件 `ros_ready`（7 条）与 ROS 侧会话门（2 条）互相独立，**必须同时全
部满足**才会有非零 PWM。从外部（电机不转）无法分辨是哪一层、哪一条失
败——这是本次调查最大的难点，必须靠分层探针而不是猜测。

## 2. 已查实的新证据（本次会话新增）

### 2.1 固件源码当前处于"未提交的修改"状态

```
$ git status
位于分支 main
尚未暂存以备提交的变更：
	修改：     BALANCE/Inc/data_task.h
	修改：     BALANCE/Inc/steer_debug.h
	修改：     BALANCE/Inc/uartx_callback.h
	修改：     BALANCE/balance_task.c
	修改：     BALANCE/data_task.c
	修改：     BALANCE/steer_debug.c
	修改：     BALANCE/uartx_callback.c
	修改：     Makefile
未跟踪的文件:
	build/
```

`git log` 只有一个提交 `4af5e40 基线：C50C 四轮固件（2026-11-08 迁移候
选）`。也就是说**所有 ROS 直行相关改动目前都没有 commit，全部是工作
区未提交状态**，没有历史 commit 可供 `git diff` 对比"13:30 能转"与
"现在不能转"之间的差异。这意味着这个回归**不能用 git 历史复原**，必
须靠文件 mtime、备份目录、现场日志来重建时间线。

### 2.2 文件 mtime 显示 `data_task.c` 是本次调查范围内最后被改动的文件

```
$ ls -la --time-style=full-iso BALANCE/balance_task.c BALANCE/data_task.c \
    BALANCE/Inc/steer_debug.h BALANCE/Inc/uartx_callback.h BALANCE/steer_debug.c \
    BALANCE/uartx_callback.c Makefile build/Diff_Car.bin
-rw-rw-r-- 1 snow snow 112056 2026-11-09 11:51:43 BALANCE/balance_task.c
-rw-rw-r-- 1 snow snow  18807 2026-11-09 14:06:09 BALANCE/data_task.c   <- 最新
-rw-rw-r-- 1 snow snow   1414 2026-11-09 11:51:24 BALANCE/Inc/steer_debug.h
-rw-rw-r-- 1 snow snow   1620 2026-11-09 11:52:01 BALANCE/Inc/uartx_callback.h
-rw-rw-r-- 1 snow snow   4767 2026-11-09 11:51:33 BALANCE/steer_debug.c
-rw-rw-r-- 1 snow snow  22080 2026-11-09 11:53:16 BALANCE/uartx_callback.c
-rwxrwxr-x 1 snow snow  89384 2026-11-09 14:06:09 build/Diff_Car.bin      <- 最新，和 data_task.c 同时刻
```

`build/Diff_Car.bin` 的 mtime 与 `data_task.c` 完全一致（14:06:09），
说明**这个 bin 是在 `data_task.c` 被改动之后、由同一次 `make` 重新产
出的**。而用户此前汇报的"13:30 能转"是在这次重编之前。**这个
`build/Diff_Car.bin` 很可能还没有被重新烧录到板子上**——需要现场确认
实际烧录动作的时间戳是否晚于 14:06。

### 2.3 一份停手报告直接解释了 `data_task.c` 为什么被改、改到了什么程度

文件：`/mnt/data/project-ai/Embedded_Development/reviews/2026-11-06-c50c-ros2-plan/交付/第一阶段直行/现场准备-审核返工/logs/data_task-48byte-recover-20261109.md`

核心结论（原文已读，摘录要点）：

- 任务目标：让反馈帧从 24 字节扩到 48 字节（新增四轮编码器 16 字节 +
  四路 PWM 读回 8 字节），并让重建后的 `data_task.c` 编译产物与"已交
  付镜像"`Diff_Car-task15.bin`（89416 B，sha256 `22dc883c...`）字节级
  一致。
- **结论：未达到字节一致。** 本机重建出两个变体：
  - 变体 A：89384 B，sha256 `26b17aa8...`（与当前
    `build/Diff_Car.bin` 的 89384 B 大小一致，疑似同一产物，需要
    GPT 现场重新 `sha256sum` 核对，不能直接采信旧记录）
  - 变体 B：89376 B，sha256 `f8e63332...`
  - 均比"已交付镜像"少 32~40 字节，差异定位在 `data_task` 函数内部
    （编码器打包方式不同：ref 用寄存器自增循环 + 栈上 `float[4]`，
    变体 A 直接写结构体字段再打包，语义等价但生成的机器码字节数不
    同）。
- **已确认的功能正确性**（反汇编验证，偏移量对）：编码器写
  `buffer[22..37]`、PWM 写 `buffer[38..45]`、BCC 在 `buffer[46]`、帧
  尾在 `buffer[47]`，与 `BALANCE/Inc/data_task.h` 的
  `SEND_DATA_SIZE=48`/`SEND_DATA_BCC_LEN=46`/
  `SEND_DATA_ENCODER_OFFSET=22`/`SEND_DATA_PWM_OFFSET=38` 一致。
- **报告自己列出的风险点（原文）**：
  1. 无法字节复现，不能证明"重建源码 == 已烧录镜像的源码"。
  2. 变体 A 会写 `Send_Data.Sensor_Str.Encoder[]`，而 ref 不写该字
     段（该字段当前无人读取，暂无影响，但是行为差异）。
  3. **发送循环改了两处**（`Usart1_SendTask`、`Usart3_SendTask` 都
     从 24 字节改成了 `SEND_DATA_SIZE`）。报告原话："lead 原指令说
     发送循环『只改一处』，实际改了两处……若已烧录镜像只改了一处，
     则 USART1 侧行为不同。此点无法从 ref bin 直接判定。"**这是一
     个未消除的不确定性：USART1 和 USART3 现在的帧长是否真的应该一
     样，没有被证实。**
  4. **报告本身的最后一句话："未上机：未烧录、未开串口、未驱动电
     机。功能正确性仅由反汇编偏移与宏定义对齐证明，未在真实设备上
     验证。"**

**这份报告的时间戳（14:07:05）紧跟在 `build/Diff_Car.bin` 重编
（14:06:09）之后，强烈提示："13:30 能转"到"现在不能转"之间，工作区
至少经历了一次 `data_task.c` 的结构性改写 + 重新编译，而这次改写从未
在真实硬件上验证过。**

### 2.4 同一天还有另一份"板端重编报告"，范围只限 ROS 侧 Python 包，与本次固件改动无关

文件：`.../现场准备-审核返工/logs/板端重编报告.md`（已读）。内容是在
OrangePi 上对 `c50c_odom`/`c50c_uart3` 做 `colcon build` + 只读 import
验证，**明确声明未启动任何节点、未打开任何串口、未驱动电机**。此报告
与 PWM 恒零无直接因果关系，只用于排除"ROS 包编译失败"这一可能性——
该报告显示编译 0 error、0 warning，13 个 `chassis_cmd` 子模块全部可
import。**这份报告不能证明"运行时"行为正常，只证明"能 import"。**

### 2.5 `GamePadInterface` 赋值链已核实完整（见 1.1 节），不是本次卡点的首要怀疑对象

已排除的早前假设："`GamePadInterface == NULL` 导致 `ros_ready` 恒
假"——这个假设需要先确认板子不是 `V1_0` 才成立，而 `V1_0` 路径里
`PS2_Init()` 必然会把它设为非 NULL。**除非板子实际硬件版本不是
`V1_0`，这一条不是独立故障点。**

## 3. 当前最可信的假设排序（按可信度，均未经现场验证，禁止直接当结论）

1. **（最可信）`build/Diff_Car.bin` 是 `data_task.c` 48 字节改写之后
   的新编译产物，而该改写从未上机验证过。** 如果现场把这个新 bin 重
   新烧录到板子上，`data_task.c` 里任何一个运行时问题（不是编译错
   误，报告说编译 EXIT=0），或者 `Usart1_SendTask`/
   `Usart3_SendTask` 循环上界改动引入的时序/阻塞问题，都可能干扰到
   同一个 `balance_task` 任务周期内的 PWM 输出路径，需要逐行核对，
   报告原文承认"没有在真实设备上验证"。
2. **ROS 侧与固件侧的帧格式版本错配**：固件状态帧已从 24 字节改成
   48 字节（`SEND_DATA_SIZE`），如果 ROS 侧实际运行的
   `frame_parser.py`/`baseline.py` 还是旧版本（仍按 24 字节解析），
   `handshake.py` 会拿到错的 BCC/帧尾校验，永远无法建立 session 证
   据，桥会保持静默。**反之如果固件还没有真正烧录新 bin（仍是老的
   24 字节帧），但 ROS 侧已经按 48 字节解析，同样会失配。** 这是最
   该优先验证的"双边版本错配"场景，而且与第 1 条高度相关——两者可
   能是同一个根因的两个侧面。
3. `robot_control.ros_cmd_tick` 超时（`ROS_CMD_TIMEOUT_MS`，约
   500ms）——如果上一条的帧解析失败导致 ROS 侧停止发送合法速度帧，
   固件侧会在超时窗口后自然判定超时，表现出的症状也是"PWM 恒零"，
   但这只是上游问题的下游症状，不是独立根因。
4. 其余 `ros_ready` 条件（`!charger.AllowRecharge`、
   `!robot_check.DeepCheck`、`SysVal.Time_count > CONTROL_DELAY +
   230`、`Get_Control_Mode(_ROS_Control)`）——没有证据支持，但必须
   作为探针逐条打印排除，不能假设它们恒为真。

## 4. 必须先做、不能跳过的现场核实（给 GPT 的硬性前置步骤）

**在修改任何一行代码之前，必须先拿到以下证据，否则任何"修复"都是盲
改：**

1. **当前板子的真实硬件版本**：读 48 字节状态帧里固件填的
   `SysVal.HardWare_Ver`（如果状态帧里还没有暴露这个字段，先加一个
   只读探针字段，不要动其他逻辑），确认是不是 `V1_0`。如果不是
   `V1_0`，`ros_ready` 第一、二条永久为假，根因在这里，后面全部不
   用查。
2. **板子上实际运行/已烧录的固件是哪一版**：对比板子里实际烧录的
   bin 校验值，和本机 `build/Diff_Car.bin` 现场重新算出的
   sha256（不要直接采信本文档里记录的旧哈希，因为文件之后可能还有
   改动）。**如果板子上是旧的 24 字节帧固件，而 ROS 侧已经切到 48
   字节解析器，这就是根因，直接对齐版本即可，不需要改任何业务逻
   辑。**
3. **ROS 侧 `frame_parser.py`/`chassis_cmd/baseline.py` 当前
   `FRAME_LEN`/`STATUS_FRAME_LENGTH` 实际值**：本机源码树（1.2 节路
   径）里现在是多少，板子上 `/home/orangepi/ros2_ws/...` 实际跑的安
   装副本是多少，两者是否一致。**这是第 3 节假设 2 的直接验证手
   段。**
4. **固件侧加一次性诊断探针，不要改动任何判定逻辑**，把 `ros_ready`
   的 7 个子条件逐个作为独立字段写进现有的 `g_safety_probe[40]`
   （`BALANCE/balance_task.c:271` 起，该数组已存在并有现成注释约
   定："小观察点，写入 .bss 之后的固定地址，用 ST-Link 读取；验证
   完成后删除"），逐条打印：
   - `SysVal.HardWare_Ver`
   - `GamePadInterface != NULL`
   - `Get_Control_Mode(_ROS_Control)`
   - `tick - robot_control.ros_cmd_tick` 的实际差值（而不是只判定
     是否超时）
   - `charger.AllowRecharge`
   - `robot_check.DeepCheck`
   - `SysVal.Time_count`（和 `CONTROL_DELAY + 230` 比较后的差值）
   这一步只读，不改变任何行为，验证完成后必须删除。
5. **ROS 侧加一次性诊断日志**，把 `bridge.py` 的
   `_session_open`/`_session_zero_written`、`ros_input.py` 的
   `snapshot()`（已有现成方法）定期打印到日志里，确认到底是哪一层
   哪一条没过。
6. **确认"13:30 能转、后来不能转"之间，现场到底做了哪些操作**：有
   没有重新烧录固件、有没有重启 ROS 节点、有没有切换过串口、有没有
   人改过 `ros_cmd_tick` 相关的发送频率。这段时间线目前没有记录，必
   须靠现场人员回忆或系统日志（`journalctl`/进程启动时间）重建。

## 5. 对 GPT 的硬性约束（修复纪律）

1. **禁止在完成第 4 节全部 6 条现场核实之前提出或实施任何代码修
   复。** 本任务此前的教训就是"没有调查就动手改"（见第 2.3 节的
   `data_task.c` 停手报告，那次改写本身就是在没有上机验证的情况下
   做的，现在反而成了新的怀疑对象）。
2. **诊断探针必须明确标记为临时**，验证完成后要在代码注释里写清楚
   "验证完成后删除"，并真正删除，不要把诊断代码遗留在生产路径里
   （代码里已有先例，`g_safety_probe` 数组本身就是这么用的，照此惯
   例做）。
3. **任何对 `data_task.c`、`Usart1_SendTask`、`Usart3_SendTask` 的改
   动，必须先确认当前这次改写（48 字节扩帧）到底有没有被上机验证
   过。** 如果没有，优先考虑"回退到已知在 13:30 能转的那个版本"作
   为第一道防线，而不是继续往前修——除非能明确证明 13:30 之后的改
   动与 PWM 恒零无关。
4. **`steer_debug.c` 的 `g_ros_drive_max_duty` 和 `ARM_NEUTRAL_MS`
   100ms 中立窗逻辑不要改**，这是已经过离线验证通过的逻辑（见"固件
   打通与扩帧报告.md"第三部分），不是本次怀疑对象。
5. **一次只改一个变量。** 如果同时怀疑"硬件版本不对"和"帧格式版本
   不对"，先验证硬件版本（成本最低、最快排除），再验证帧格式。不
   要同时改两处再上机测试，否则无法判断到底是哪个起作用。
6. **`ros_cmd_tick`/`ROS_CMD_TIMEOUT_MS` 的超时窗口不要放宽来"掩
   盖"问题。** 如果怀疑是超时导致的，先证明"ROS 侧确实没有按时发
   帧"（有日志证据），再去看为什么没发（大概率是上游的帧解析失败
   或会话门没开，不是超时阈值本身设错）。放宽超时阈值是典型的"治标
   不治本"，被明确禁止。
7. **任何涉及 USART1/USART3 帧长同步的改动，必须同时更新固件端和
   ROS 端，并重新跑 ROS 侧现有的 pytest 套件**（"固件打通与扩帧报
   告.md"记录的基线是 `1 failed, 440 passed, 4 skipped`，那 1 个失
   败是另一个任务的既有偏差，与本任务无关，不需要也不要去修它）。
   **修复后必须重新跑一遍这个套件，确认不低于这个基线。**
8. **完成修复后，必须能清楚回答**："13:30 能转、后来不能转"中间，
   到底是哪一层（固件 7 条门之一 / ROS 侧会话门 / 帧格式版本错配 /
   别的）出了问题，并且有现场读出来的具体数值作为证据，不能只给出
   "应该是这个原因"的推测性结论。

## 6. 本机可达路径汇总（给 GPT 定位文件用）

- 固件源码：`/mnt/data/project-ai/Embedded_Development/ROS2/work/2026-11-08-50pct-migrate`
  （git 仓库，HEAD 唯一提交 `4af5e40`，当前有未提交修改，见第 2.1
  节）
- ROS 侧源码（本机开发树，与板子上 `/home/orangepi/ros2_ws/...` 需
  要现场核对是否一致）：
  `/mnt/data/project-ai/Embedded_Development/ROS2/work/ros-straight-migrate/`
  （内含 `c50c_uart3/`、`c50c_odom/`）
- 本次调查已读到的两份关键报告：
  - `/mnt/data/project-ai/Embedded_Development/reviews/2026-11-06-c50c-ros2-plan/交付/第一阶段直行/现场准备-审核返工/logs/data_task-48byte-recover-20261109.md`
  - `/mnt/data/project-ai/Embedded_Development/reviews/2026-11-06-c50c-ros2-plan/交付/第一阶段直行/现场准备-审核返工/logs/固件打通与扩帧报告.md`
- 同目录下还有大量现场日志（`_watch.log`、`_watch2.log`、
  `full-chain-raw.log`、`stop-timeout-raw.log` 等），尚未逐一读取，
  GPT 接手后应优先查这些日志里是否已经包含"13:30 之后"的运行时证
  据，而不是重新采集。
- 本机长期记忆文档（架构总览，供背景参考）：
  `/home/snow/.claude/projects/-mnt-data-project-ai-Embedded-Development-new/memory/project_c50c_ros2.md`

---

**再次强调给 GPT 的第一步**：先做第 4 节的 6 条现场核实，把结果填回
这份文档（或新开一份证据记录），再进入第 5 节约束下的修复阶段。不要
跳步。
