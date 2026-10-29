# OLED 开机自检引导界面 · 设计文档

日期：2026-10-29
分支：`qoder/oled`
目标板：C50C-V4.3 / STM32F407VET6，HAL + FreeRTOS（CMSIS_V2）

## 1. 目标

上电后由 OLED 引导完成开机自检流程：等待按键 → 加载动画 → 逐项自检（电机、板级电压）并打勾 → 进入主页面显示四轮转速与板子电压。

限制前提：

- 无电池。所谓「电池自检」实际检查单片机自身的 3.3 V 供电（VDD），不用 PB0 分压。
- OLED 为 0.96" SSD1306 128×64，4 线软 SPI，**无 CS 引脚**（模块上 CS 已接地）。
- 中文仅使用已导出的 17 个不重复 16×16 字形；数字/字母使用 6×12 ASCII 字模。

## 2. 硬件接口（引自 knowledge/pinmap.md）

| 信号 | 引脚 | 说明 |
|---|---|---|
| OLED SCL | PD7 | 输出，初始高 |
| OLED SDA | PD4 | 输出，初始高 |
| OLED DC | PE0 | 低=命令，高=数据 |
| OLED RES | PE1 | 低脉冲复位，平时高 |
| KEY_USER | PC5 | 上拉输入，按下为低 |
| 电机 PWM | PC9 / TIM8_CH4 | 1 kHz，PSC 9 / ARR 16799 |
| 电机 DIR | PB13 | 高=正转（`MOTOR_RF_DIR_FORWARD`） |
| 电机 SLP | PB12 | 低=驱动睡眠，高=唤醒 |
| 编码器 A/B/C/D | TIM2/3/4/5 | 编码器模式 TI12，对应轮位 RF/LF/RR/LR |
| ADC | ADC1_IN17 | 内部 VREFINT，测 VDD |

OLED 四脚已在 `MX_GPIO_Init()` 中初始化为推挽输出（gpio.c:113-131），驱动层不再重复初始化 GPIO。PB12 不在 .ioc 中，沿用现有做法在任务内手动初始化。

## 3. 屏幕流程

### 3.1 屏幕 ①：等待初始化

- 第一行居中显示「等待初始化」（5 字 × 16 px = 80 px，y=16）。
- 下方居中显示 ASCII「PRESS KEY」（6×12，y=40），500 ms 亮 / 500 ms 灭循环。
- 按 KEY_USER：连续两次 10 ms 采样均为低才确认（沿用现有去抖）。

### 3.2 屏幕 ②：加载动画（1.0 s）

- 居中显示「正在加载」+ 12×12 转圈动画，8 帧 × 125 ms。
- 1.0 s 后切换到「自检流程开始」屏（居中，0.8 s），随后进入屏幕 ③。

### 3.3 屏幕 ③：逐项自检

```
自检流程开始        ← y=0，16 px 高
电机自检  ◐        ← y=22
电池自检  ◐        ← y=42
```

- 正在执行的项目后缀显示转圈动画；完成后原位替换为 ✓。
- **电机自检 24 s**：3 s 正转 → 3 s 停 → 3 s 反转 → 3 s 停，循环 2 次。PWM 占空比 8400/16799 ≈ 50%，方向由 PB13 控制。
- **电池自检瞬时完成（<0.1 s）**：VREFINT 8 次采样后判定。
- 两项均打 ✓ 后进入主页面。不实现大对勾完成屏。
- 驱动使能时序：上电到按键确认前 PB12 保持低（睡眠）；进入电机自检时置高唤醒，24 s 结束后回低。电机自检收尾：CCR 清零 → `HAL_TIM_PWM_Stop()`（清 MOE，PC9 回低）→ PB12 回低。

### 3.4 屏幕 ④：主页面

每 100 ms 刷新一次，整屏 10 Hz：

```
VDD 3.29V
RF  +123 rpm
LF  +121 rpm
RR  +124 rpm
LR  +122 rpm
```

- 5 行 × 12 px，y = 0/12/24/36/48。
- 电压保留两位小数。
- 转速为有符号整数，正负号表示方向；上限显示 4 位数字。

## 4. 软件架构

单任务前台状态机：一个 FreeRTOS 任务顺序执行整个流程，电机时序阻塞在该任务内，不使用任务间通信。

状态：`WAIT_KEY → LOADING → MOTOR_CHECK → VOLT_CHECK → MAIN`。

| 文件 | 职责 |
|---|---|
| `Core/Inc/oled.h` / `Core/Src/oled.c` | SSD1306 软 SPI 硬件层：复位、初始化序列、整屏 flush、方向宏 |
| `Core/Inc/oled_gfx.h` / `Core/Src/oled_gfx.c` | 纯 C 帧缓冲与绘制（不依赖 HAL，可在主机测试）：清屏、像素、位图、6×12/16×16 文本 |
| `Core/Inc/oled_font.h` / `Core/Src/oled_font.c` | 6×12 ASCII 95 字、17 个汉字 16×16（正立存储）、12×12 转圈 8 帧、✓ / ✗，由工具生成 |
| `Core/Inc/boot_ui.h` / `Core/Src/boot_ui.c` | 状态机、按键去抖、电机时序、VDD 测量、转速采样，入口 `boot_ui_run()`；头文件含可在主机测试的纯数学内联函数 |
| `Core/Src/freertos.c` | `StartSelfCheckTask` 改为调用 `boot_ui_run()`；任务栈 256×4 → 512×4 |
| `Makefile` | `C_SOURCES` 加入 oled.c、oled_gfx.c、oled_font.c、boot_ui.c |
| `tools/font_convert.py` + `tools/font_src/*.h` | 字模重组工具及原始导出文件，保证字库可重现 |
| `tools/test_font_convert.py`、`tools/test_oled_gfx.c`、`tools/test_boot_math.c` | 主机侧测试（python3 / gcc），不依赖开发板 |

`main.h`、`gpio.c`、`adc.c`、`tim.c` 不改动。`main.c` 不改动（编码器启动放在 boot_ui 初始化中）。

## 5. 显示驱动

- 写时序：SCL/SDA 直接用 BSRR 置位/清零；每字节 MSB 先发；DC 选择命令/数据。无 CS。
- 初始化：RES 低 20 ms → 高 20 ms，再发 SSD1306 标准 128×64 序列（0xAE, 0xD5/0x80, 0xA8/0x3F, 0xD3/0x00, 0x40, 0x8D/0x14, 0x20/0x00, 0xA1, 0xC8, 0xDA/0x12, 0x81/0xCF, 0xD9/0xF1, 0xDB/0x40, 0xA4, 0xA6, 0xAF）。
- 帧缓冲：`uint8_t fb[8][128]`，bit0 为页内顶行；flush 用水平寻址（0x21 0 127 / 0x22 0 7）后连续写 1024 B。
- 方向：`OLED_MIRROR_X`（0xA0/0xA1）与 `OLED_MIRROR_Y`（0xC0/0xC8）两个宏，首次上板若画面颠倒只改这两个宏。
- 全部绘制函数只写帧缓冲，不直接写屏。

## 6. 字库

### 6.1 ASCII 6×12

取自厂商 `oledfont.h` 的 `oled_asc2_1206[95][12]`。已逐个解码验证实际布局：

- 6 列 × 2 字节/列，列优先；
- 每列高字节在前，bit15 = 顶行，仅用到 bit15..bit4（12 px）；
- 例：`A` → 顶点在中间列顶部、横杠第 8 行；`0` → 6×8 方框；`M`/`V` 均正确。

渲染时按上述位序直接拆成两个 pages 写入帧缓冲，不做预处理。

### 6.2 汉字 16×16

用户导出的 4 个 .h 文件为 **列行式且整体旋转 90°**（文件头注释「行列式（横向字节）」与实际不符，以实际解码为准）。`tools/font_convert.py` 读取原件、按页解码后顺时针旋转 90°、按正立 page 格式重新输出为 `oled_font.c`。

17 个不重复字形：等 待 初 始 化 / 正 在 加 载 / 自 检 流 程 开 始 / 电 机 池（「始」在「初始」「开始」中各用一次）。导出文件中的「通信自检」不进入固件。

### 6.3 动画与符号

转圈 8 帧（12×12，圆弧按 45° 递增）、✓、✗（均 16×16）在 `tools/font_convert.py` 中按程序生成，与汉字一起输出到 `oled_font.c`，不在运行时计算。

## 7. 电压（VDD）

- ADC1 配置为单次、软件触发、1 转换、480 周期，与现有配置一致；测量前用 `HAL_ADC_ConfigChannel()` 切到 `ADC_CHANNEL_VREFINT`（HAL 会自动置 TSVREFE）。
- 本项目所用 HAL 版本未提供 `HAL_ADCEx_Calibration_Start`（已在 Drivers 中确认），因此不做 ADC 自校准；剩下的偏移误差对本用途（3.1 V 阈值与两位小数显示）可忽略。
- 连续 8 次单次转换取平均，`VDD_mV = 3300 × Vrefint_CAL / raw`，`Vrefint_CAL` 读自 0x1FFF7A2A。
- 判定：`VDD_mV ≥ 3100` 为通过。若按字面「>3.3 V」判定，真实 3.3 V 轨（读数约 3.28–3.30 V）会误判失败。
- `Vrefint_CAL` 为 0 或 0xFFFF（芯片未校准）时按 1490 处理，保证不会出现除零。
- 失败：显示 ✗，每 500 ms 重测，最多 3 次；仍失败则停留在自检页。无电池场景下电压异常即板级故障，主页面数值同样可直接观察。
- 测量结束后把通道恢复为 ADC_CHANNEL_8（电池分压），保持 ADC 状态与之前一致。

## 8. 转速

- 在 boot_ui 初始化时对 TIM2–TIM5 调用 `HAL_TIM_Encoder_Start(&htimX, TIM_CHANNEL_ALL)`（此前只配置未启动）。
- 每 20 ms 读一次 CNT，`int16_t` 差值处理 16 位回绕，累加 5 次得到 100 ms 计数。
- `RPM = 累计计数 × 600 / WHEEL_CPR`，`WHEEL_CPR = 54000`（4 × 500 线 × 27 减速比，MD36N_27 + GMR_500 出厂配置）。
- 宏 `WHEEL_CPR` 与 20 ms/5 次采样参数集中在 `boot_ui.h`。
- 20 ms 内计数上限约 7200（≈4000 RPM），`int16_t` 不溢出；100 ms 累计用 `int32_t`。
- 轮位映射固定：TIM2=RF、TIM3=LF、TIM4=RR、TIM5=LR；装车后手转单轮复核符号与数值。

## 9. 文件变更清单

新增：

- `Core/Inc/oled.h`、`Core/Src/oled.c`
- `Core/Inc/oled_gfx.h`、`Core/Src/oled_gfx.c`
- `Core/Inc/oled_font.h`、`Core/Src/oled_font.c`
- `Core/Inc/boot_ui.h`、`Core/Src/boot_ui.c`
- `tools/font_convert.py`、`tools/font_src/`（4 个原始字模文件 + 从厂商表提取的 ASCII 字模）
- `tools/test_font_convert.py`、`tools/test_oled_gfx.c`、`tools/test_boot_math.c`
- 本文档

修改：

- `Core/Src/freertos.c`：任务入口改为 `boot_ui_run()`，栈 512×4
- `Makefile`：新增 4 个源文件

## 10. 验证

1. `make` 编译通过，无新增警告。
2. ST-LINK 烧录后上电：① 屏正常显示且字迹正立（否则调方向宏）。
3. 按 KEY_USER：去抖正常，② 动画流畅，③ 各项按序打勾。
4. 电机 24 s 时序与方向正确，结束后驱动器回睡眠（PB12 低）。
5. 主页面 VDD ≈ 3.3 V，与万用表一致（±0.05 V 内）。
6. 手转每个轮子：对应行 RPM 符号与数值变化，一圈计数 ≈ 54000（可用调试器读 CNT 复核）。
7. 主页面连续刷新无闪烁、无残影。

## 11. 上板待复核项

| 项目 | 当前取值 | 复核方法 |
|---|---|---|
| OLED 方向 | 0xA1 + 0xC8 | 目视，颠倒则改宏 |
| `WHEEL_CPR` | 54000 | 手转一圈读 TIMx CNT 增量 |
| 轮位对应 | TIM2=RF … TIM5=LR | 手转单轮看主页面行 |
| VDD 阈值 | ≥3100 mV | 万用表对比主页面读数 |
