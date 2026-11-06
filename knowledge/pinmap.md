# 引脚配置表 · C50C-V4.3 / STM32F407VET6

本表由 `hal_f407_mecanum_4motor_src.ioc` 导出，是固件与硬件接线之间的唯一对照。
改引脚先改 `.ioc`，再由 CubeMX 生成代码，最后回来更新本表。

---

## 1. 电机驱动（TIM8 + MD01 原生排针 P1/P2）

| 轮位 | PWM | TIM8 | DIR | SLP/EN | CS | 板上排针 |
|---|---|---|---|---|---|---|
| 右前 RF (A) | PC9 | CH4 | PB13 | PB12 | CR1 | P1/P2 |
| 左前 LF (B) | PC8 | CH3 | PC0 | **PB14**（实板核对 2026-10-29，纠正 PR #2 规划的 PE15；PB14 是排针空闲脚，与 RGB 无关） | CR2 | P1/P2 |
| 右后 RR (C) | PC7 | CH2 | PD10 | PD12 | CR3 | P1/P2 |
| 左后 LR (D) | PC6 | CH1 | PC12 | PA8 | CR4 | P1/P2 |

- PWM：PSC 9 / ARR 16799 → **1 kHz**，PWM mode 1，初始占空比 0。AOE（自动输出）已在 `MX_TIM8_Init()` 中配置，但 **MOE 默认不置位**——要等 `HAL_TIM_PWM_Start()` 才会置 MOE、使能通道并启动计数器。也就是说初始化完成后定时器是停的，四个 PWM 脚保持低电平
- DIR：`GPIO_Output`，**High speed**，初始低电平；方向翻转只影响电平，不影响频率
- SLP：`GPIO_Output` / **Low speed** / 初始**低电平** = 驱动睡眠（板上 10 kΩ 下拉，悬空即睡眠）。⚠️ 这四个脚的初始化代码在 `feat/motor-slp-pins` 分支（PR #2）才加入；只合入本文档所在分支时，基线里还没有它们。⚠️ PR #2 的 `.ioc` 把 B 轮 SLP 规划成了 PE15，**实板核对为 PB14**，合入 PR #2 时必须把 `MOTOR_LF_SLP` 改成 PB14
- ⚠️ ~~PB14 冲突~~ **已澄清（2026-10-29）**：RGB 灯区是板载 LED1，焊死在板内、不占排针；PB14 是排针空闲脚，实接 B 轮 EN。**RGB 蓝灯实际引脚未定**（厂商文档"蓝=PB14"有误），红色 PB15、绿色 PE6 已确认，蓝色待原理图复核。本分支 `.ioc` 里 `RGB_B=PB14` 的标注是错的，合并 SLP 配置时需一并改正
- CS：电流采样输出，当前未接、未配置
- 编号约定：A/B/C/D 是厂商电机编号，**代码里一律用轮位 RF/LF/RR/LR**，不用字母

> 注意：TIM8 的 CH1N–CH4N 与 BKIN **必须保持禁用**（会占用 PA7/PB0/PB14/PA6，与编码器 B、电池 ADC、RGB 蓝冲突）。

## 2. 编码器（TIM2–TIM5，编码器模式 TI1 and TI2）

| 编码器 | A 相 | B 相 | 定时器 | 对应轮位 | 接口 |
|---|---|---|---|---|---|
| A | PA15 | PB3 | TIM2 | 右前 | U23（XH2.54-4P） |
| B | PA6 | PA7 | TIM3 | 左前 | U22 |
| C（接口标签） | PB6 | PB7 | TIM4 | **左后 D（当前实接）** | U21 |
| D（接口标签） | PA0 | PA1 | TIM5 | **右后 C（当前实接）** | U12 |

- 计数器：PSC 0 / Period 65535 / TI12（四倍频）/ 上下拉 **上拉** / 无中断
- TIM2、TIM5 本身是 32 位，仍统一用 65535，方便四路用同一套 int16 差值运算
- 插座 pin1=A、pin2=B、pin3=3V3、pin4=GND；**插座供电是 3.3 V，5 V 编码器不能直插**
- PA15/PB3 是 JTDI/JTDO，之所以能用，是因为 SYS Debug 选了 **Serial Wire**（JTAG 已关闭）
- **2026-10-30 手转逐轮复核**：A→TIM2、B→TIM3、实际右后 C→TIM5、实际左后 D→TIM4。当前接线的后轮编码器 C/D 互换，`boot_ui.c` 已按真实轮位映射；`.ioc` / `ENC_C_*`、`ENC_D_*` 名称仍表示板上接口与定时器，不代表装车轮位。
- 每轮手转一圈参考计数与方向符号见 `Core/Inc/boot_ui.h`，为单次人工标定，精度受标记对齐影响。

## 3. 通信

| 功能 | 引脚 | 外设 | 参数 | 主机侧 |
|---|---|---|---|---|
| 调试串口 | PB10 TX / PB11 RX | USART3 | 115200 8N1 | 板载 CH340 → `/dev/ttyUSB*` |
| 上位机 | PA9 TX / PA10 RX | USART1 | 115200 8N1 | 板载 CH9102 → `/dev/ttyACM*` |
| 蓝牙 | PC10 TX / PC11 RX | UART4 | **9600** 8N1 | 蓝牙模块插座 |
| IMU | PB8 SCL / PB9 SDA | I2C1 | 100 kHz 标准模式 | ICM-20948（1.8 V 侧，经电平转换） |
| 电池电压 | PB0 | ADC1_IN8 | 12 bit，480 cycles，软件触发单次 | 分压电阻 |
| PS2 无线接收器（当前使用） | PE0 DI / PE1 DO / PE2 CS / PE3 CLK | GPIO 软件时序 | 0x73/0x79 模拟模式，约 20 ms 轮询 | 板上 PS2 接收器 |
| USB 外设（当前手柄不使用） | PA11 DM / PA12 DP | USB_OTG_FS | Host_Only + HID | USB-A 座 CN5 |
| 调试口 | PA13 SWDIO / PA14 SWCLK | SYS | Serial Wire | 4 针 SWD 座 |

- USART3 走 **PB10/PB11**（板载 CH340 的 TXD/RXD），不是 PD8/PD9
- UART4 的 CubeMX 默认引脚是 PA0/PA1，**必须手工改成 PC10/PC11**，否则会抢编码器 D 的引脚
- USB Host 需要 PLL Q=7（336/7 = 48 MHz），Q 错了不会报错，只会静默不工作
- I2C1 不需要开内部上拉：板上 R63/R64 已有 10 kΩ 上拉到 3V3V

## 4. 人机交互

| 功能 | 引脚 | 标签 | 方向 | 初始电平 / 说明 |
|---|---|---|---|---|
| OLED SCL | PD7 | `OLED_SCL` | 输出 | 高（软件 SPI） |
| OLED SDA | PD6 | `OLED_SDA` | 输出 | 高 |
| OLED RES | PD5 | `OLED_RES` | 输出 | 高（释放复位） |
| OLED DC | PD4 | `OLED_DC` | 输出 | 低 = 命令模式 |
| RGB 红 | PB15 | `RGB_R` | 输出 | 低 |
| RGB 绿 | PE6 | `RGB_G` | 输出 | 低 |
| RGB 蓝 | **未定** | `RGB_B` | 输出 | 板载 LED1 焊死在板内；厂商文档"PB14"有误（PB14 实为 B 轮 EN），待原理图复核 |
| 用户按键 | PC5 | `KEY_USER` | 输入 | 上拉，按下为低 |
| 急停/使能 | PE4 | `EN_SW` | 输入 | 上拉，1 = 允许，0 = 急停（板上开关未装） |
| 蜂鸣器 | PD11 | `BUZZER` | 输出 | 低 = 响（NPN 驱动，有跳线） |

## 5. 时钟与时基

| 项目 | 配置 |
|---|---|
| HSE | 8 MHz 晶振 |
| PLL | M 8 / N 336 / P 2 / Q 7 |
| SYSCLK / HCLK | 168 MHz |
| APB1 / APB2 | 42 MHz / 84 MHz |
| Flash latency | 5 WS |
| HAL 时基 | TIM6（FreeRTOS 占用 SysTick） |
| FreeRTOS | CMSIS_V2，tick 1000 Hz，heap 40960 字节，heap_4 |

## 6. 空闲 / 预留引脚

| 引脚 | 状态 | 备注 |
|---|---|---|
| PA2 / PA3 | 空闲 | 备用 USART2，在排针上 |
| PD0 / PD1 | 空闲 | CAN1 备用引脚 |
| PE0 / PE1 / PE2 / PE3 | PS2 接收器占用 | 已与厂家 pstwo.h 核对；不可按空闲脚使用 |
| PD3 | 空闲 | ICM-20948 的 INT，当前不用 |
| PE5 / PE7 / PE8 / PE9 | 空闲 | 原 TIM1 电机方案遗留，已弃用 |
| PE10–PE14 | 空闲 | 在排针上 |

## 7. 禁用清单（踩过的坑）

| 引脚 | 原因 |
|---|---|
| PD5 / PD6 | OLED RES / SDA；2026-10-29 根据实板丝印及调试测量修正，原 RS485 标注错误 |
| PA7 / PB0 / PB14 / PA6 | 编码器 B / 电池 ADC / RGB 蓝 / 编码器 B，TIM8 互补通道必须禁用 |
| PA15 / PB3 | 只有 JTAG 关闭（Serial Wire）时才可用 |
| PC6–PC9 | 必须先解绑 TIM3_CH1–CH4，才能给 TIM8 使用 |
| PA8 / PA9 | USB 的 SOF / VBUS 检测会占用，两个都不能开 |

现场接线复核（2026-10-30）：用户确认 B 的原配置正确，异常来自线掉落。已撤回临时方向交换，恢复 PC0 前进 RESET、后退 SET，自检和手柄共用。编码器手转前进符号仍为 +1，四轮圈数参考不变。
