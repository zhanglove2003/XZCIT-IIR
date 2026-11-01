# XZCIT-MFSC 四轮麦克纳姆轮车固件

OLED 品牌页显示 `XZCIT-MFSC / By Plucksnow`。本项目是整车固件：OLED 分层菜单、驾驶仪表盘、双轴驾驶混合、逐轮自检、一键归位、手动标定与电源监控。

**状态**：已实车验收（2026-11-01），PR #4 合并至 main。设计文档与全部实施计划在 `docs/superpowers/`。

## 硬件配置

### 轮序与驱动（勿改，实车验证过）

| 物理轮 | 位置 | 编码器 | TIM8 PWM | 前进反馈符号 | CPR（手动标定） | SLP/EN | DIR |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A | 右前 | TIM2 | CH4 | **-1** | 213203 | PB12 | PB13 |
| B | 左前 | TIM3 | CH3 | +1 | 212664 | PB14 | PC0 |
| C | 右后 | TIM5 | CH2 | **-1** | 214993 | PD12 | PD10 |
| D | 左后 | TIM4 | CH1 | +1 | 214348 | PA8 | PC12 |

- TIM8 ARR=16799（周期 16800 计数）；PWM 比较值即"每周期高电平计数"。
- SLP 高=使能、低=睡眠（板上 10kΩ 下拉，不使能不转）。
- 编码器 16 位计数，采样间隔必须 <32768 计数（Ctrl 每 10ms 采样）。
- GPIO 配置同步在 `hal_f407_mecanum_4motor_src.ioc`（CubeMX 可重新生成；SLP 引脚已于 2026-11-01 补入）。

### OLED（SSD1306 128×64，软件 SPI）

PD7=SCL、PD6=SDA、PD5=RES、PD4=DC；镜像方向已校准勿改。全中文界面（16×16 汉字 88 字），仪表盘顶栏用 6×12 ASCII 短代码（FWD/STOP/REF/FAULT…）。

### 手柄与按键

- PS2 手柄（模拟模式）：PE0-PE3 位带时序；按键低有效——SELECT=位0、L1=位10、R1=位11。
- USER 实体按键：PC5（低有效）。

### 电源

- 电池分压：PB0（ADC1_IN8）。**换算未配置**（`POWER_BATTERY_CONFIGURED=0`），分压比/额定电压/欠压阈值需现场核对后填入 `Core/Inc/power_monitor.h` 并置 1。未配置时界面显示"电池未配置"，不判欠压。
- MCU VDD 经 VREFINT 监测，<3100mV 禁止一切运动输出。

## 操作说明

| 按键 | 作用 |
| --- | --- |
| R1 | 主页开菜单；其他页返回一级；动作中先取消再返回 |
| L1 | 确认；输出上限页返回子菜单 |
| USER | 主页进自检确认（5 秒倒计时）；倒计时中再按取消；标定页退出 |
| SELECT 短按 | 驾驶页发起归位（最短朝向归位快捷键） |
| SELECT 长按 2s | 仅手动标定页记录参考（驾驶页无效） |
| 左摇杆 | 菜单上下/左右选择；驾驶页 Y 轴前后 |
| 右摇杆 | 仅驾驶页 X 轴转向（右推=右转） |

页面结构：品牌主页 →（R1）主菜单卡片【驾驶 / 检查标定】→ 检查与标定子菜单【自检结果 / 重新自检 / 回到初始位 / 手动标定 / 输出上限】。

### 关键安全行为

- 开机不转轮；进入驾驶页而本次上电未通过自检 → 显示"请先自检"，零输出。
- 自检确认页 5 秒倒计时自动开始；手柄未连接/未回中/电压不足时冻结并显示原因，恢复后续走剩余时间。
- 故障锁存（NO FB/DIR ERR）：输出立即清零，只有完整重跑自检且四轮全过才清除。
- 手柄失联 200ms 停机；坏帧当拍丢弃；归位/自检中失联立即取消。
- 电机唯一写入路径是 Ctrl 任务的 `ctrl_apply()`；UI 只读快照（seq-lock）。

## 驾驶输出

双轴混合（前进 V=左摇杆 Y，转向 T=右摇杆 X；死区 16、中心 128、两端对称归一化）：

```
A(右前)=V+T  B(左前)=V-T  C(右后)=V+T  D(左后)=V-T
peak = max(|A|,|B|,|C|,|D|)；peak>上限 → 四值同乘 上限/peak
```

- **输出上限**：检查与标定 → 输出上限页，10%~90% 占空比（±5% 步进），即时生效，断电恢复默认 50%（=8400，历史满值）。转向分量恒为上限一半。只影响驾驶；自检(8400)/归位(3360) 固定。
- >50% 档已于 2026-11-01 实车验收通过。

## 构建

```bash
make -j8          # arm-none-eabi-gcc，当前零告警
# 产物：build/hal_f407_mecanum_4motor_src.{elf,bin}
```

## 主机测试（改代码必跑）

```bash
bash tools/test_motion.sh                  # 24 项模块测试（ASan/UBSan）
bash tools/test_boot_ctrl_integration.sh   # 7 场景 UI+Ctrl+服务集成（ucontext）
python3 tools/test_font_convert.py         # 字模生成管道
```

测试约定：先写失败测试再实现；禁止用空函数桩替代真实 UI/Ctrl 协同测试。

## 烧录（已验证流程）

ST-LINK V2 + STM32CubeProgrammer 2.23.0（NRST 未接也能烧）：

```bash
CP=~/.local/share/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI
# 1. 连接+暂停+写入+校验（擦除失败重试一次即可，USB 口要直连主机）
$CP -c port=SWD freq=100 mode=HOTPLUG -halt -w build/hal_f407_mecanum_4motor_src.bin 0x08000000 -v
# 2. 独立读回逐字节比对（长度=实际 bin 大小，用 stat 取）
$CP -c port=SWD freq=100 mode=HOTPLUG -u 0x08000000 <size> /tmp/rb.bin && cmp build/*.bin /tmp/rb.bin
# 3. 软复位运行
$CP -c port=SWD freq=100 mode=HOTPLUG reset=SWrst -rst -run -score
```

排查顺序：USB 识别（`lsusb -d 0483:3748`）→ 低速连接 → 短读取 → 备份 → 暂停 → 写入校验 → 读回 → 复位。USB hub/长线会间歇超时，直插主机口。

## 代码结构

| 模块 | 职责 |
| --- | --- |
| `Core/Src/ui_model.c` | 页面状态机（纯 C，无 HAL/OLED）；Ctrl 任务 10ms 推进 |
| `Core/Src/ui_render.c` | 九页绘制 + 动画；只读快照，UI 任务 20ms |
| `Core/Src/freertos.c` | Ctrl 任务：输入采样、页面推进、动作仲裁、唯一电机出口 `ctrl_apply()`、seq-lock 快照发布 |
| `Core/Src/boot_ui.c` | OLED 初始化 + 20ms 渲染循环（不写电机） |
| `Core/Src/selfcheck_service.c` | 逐轮自检动作生成（A→B→C→D，500/200ms 时序） |
| `Core/Src/wheel_home.c` | 最短朝向归位状态机 + 手动记录门控 |
| `Core/Src/manual_ref.c` | （预留）USER 窗口标定流程，当前由 ui_model 接管 |
| `Core/Src/power_monitor.c` | 低优先级 ADC 服务（VDD/电池分离） |
| `Core/Src/drive_ctrl.c` | 双轴混合 + 运行时输出上限 |
| `Core/Src/motor_health.c` | 运行期反馈监测 + 故障锁存 |
| `tools/font_convert.py` | 字模生成（font_src 源 → oled_font.c，勿手改生成文件） |
| `tools/render_preview.py` | OLED 帧缓冲 PNG 预览（上板前目检字体/布局） |

## 文档索引

- 设计 spec：`docs/superpowers/specs/2026-10-31-oled-menu-driving-design.md`
- 实施计划与构建指纹：`docs/superpowers/plans/`
- 字体预览：`docs/superpowers/plans/preview-*.png`

## 待办（下一迭代）

- 电池分压参数现场核对后启用欠压停机（`power_monitor.h` 三处宏）
- `manual_ref.c` 与 ui_model 标定流程合并去重
- 输出上限断电记忆（需 Flash 参数区）
