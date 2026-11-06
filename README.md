# XZCIT-IIR · 智能巡检机器人

四轮移动机器人平台。主控 STM32F407VET6，HAL + FreeRTOS，OLED 本地界面，PS2 手柄操控。

> 仓库代号 `IIR` = Intelligent Inspection Robot。
> 历史名 `MFSC`（Mecanum Four-wheel Smart Car）沿用为麦轮底盘子项目的目录名。

## 一、项目定位

面向室内轻载巡检场景的四轮移动底盘。整车自持：本地 OLED 完成菜单、驾驶、自检、标定，不依赖上位机即可运行。

设计约束：

- **本地优先**：断网、无 PC 也能完成开机自检与驾驶。
- **单一电机出口**：所有电机写入收敛到一个函数，便于审计与急停。
- **可实测**：每个功能都要在实车上验证，不靠仿真打勾。

## 二、立项信息

| 项目 | 内容 |
|---|---|
| 项目代号 | XZCIT-IIR |
| 前身 | XZCIT-MFSC（麦轮固件） |
| 主控 | STM32F407VET6，LQFP100，168 MHz |
| 板卡 | C50C-V4.3 |
| 固件基线 | 始于 2026-10-29 的 CubeMX HAL 骨架 |
| 首个实车验收 | 2026-11-01，麦轮底盘完整固件 |
| 授权方式 | 未声明（见「待办」） |

## 三、底盘形态

同一套硬件抽象支持两种底盘。二者代码独立，互不引用。

| 底盘 | 分支 | 目录 | 运动方式 |
|---|---|---|---|
| 麦克纳姆轮 | `feat/mecanum` | 根目录 `Core/` | 全向平移 + 原地旋转 |
| 四轮差速 | `feat/differential` | `four-wheel-differential/` | 前后 + 差速转向 |

麦轮与差速的轮序、编码器映射、PWM 通道不同。切换底盘必须整分支切换，不可混用配置。

## 四、硬件摘要

只列全局信息。完整引脚表见 [knowledge/pinmap.md](knowledge/pinmap.md)。

| 项目 | 配置 |
|---|---|
| HSE | 8 MHz 晶振 |
| PLL | M8 / N336 / P2 / Q7（Q=7 供 USB 48 MHz） |
| SYSCLK | 168 MHz，Flash 5 WS |
| 定时器 | TIM8 四路 PWM 1 kHz；TIM2–TIM5 编码器模式 TI12 |
| 通信 | USART1 上位机、USART3 调试、UART4 蓝牙 9600 |
| 外设 | I2C1（IMU）、ADC1_IN8（电池）、USB_OTG_FS Host HID |
| 人机 | OLED SSD1306 128×64 软 SPI、PS2 手柄、RGB、蜂鸣器 |
| 时基 | HAL 用 TIM6，FreeRTOS 用 SysTick |

## 五、仓库结构

```
XZCIT-IIR/
├── README.md                    本文件：项目介绍与立项
├── .github/workflows/ci.yml     CI 流水线
├── docs/
│   ├── architecture.md          固件分层与任务模型
│   ├── workflow.md              分支模型与 CI 触发规则
│   └── roadmap.md               里程碑与待办
└── knowledge/
    └── pinmap.md                引脚配置表（唯一硬件对照）
```

**main 只放文档与硬件定义，不含源码、CubeMX 工程与构建文件。** 固件源码在各车型正式分支上。

## 六、构建

main 不含构建文件。要编译固件，切到对应车型的正式分支：

```bash
git switch feat/mecanum
make -j8
# 产物：build/hal_f407_mecanum_4motor_src.{elf,hex,bin}
```

需要 `arm-none-eabi-gcc`（本项目验证版本 14.3.1）与 `make`。

## 七、分支与集成

**每个车型各有一条专属 `develop` 分支。** 日常开发在专属分支进行，不触发完整流水线。推送到该车型的正式分支时才触发 CI。

| 分支 | 角色 | 触发 CI |
|---|---|---|
| `mecanum/develop` | 麦轮日常开发 | 否 |
| `differential/develop` | 差分日常开发 | 否 |
| `feat/mecanum` | 麦轮正式分支 | 是 |
| `feat/differential` | 差分正式分支 | 是 |
| `main` | 项目门面与发布 | 是（只做文档检查） |

`main` 只放文档、硬件定义与发布产物，不放日常业务代码。`main` 的 CI 不编译、不跑测试。

详细规则见 [docs/workflow.md](docs/workflow.md)。

## 八、文档索引

| 文档 | 内容 |
|---|---|
| [docs/architecture.md](docs/architecture.md) | 分层、任务、模块职责 |
| [docs/workflow.md](docs/workflow.md) | 分支模型、CI 触发、合并规则 |
| [docs/roadmap.md](docs/roadmap.md) | 里程碑、已完成、待办 |
| [knowledge/pinmap.md](knowledge/pinmap.md) | 引脚对照表与禁用清单 |

固件操作手册（菜单、按键、烧录、调试坑）在各车型正式分支的 README 中。

## 九、待办

- 选择并添加 LICENSE。
- 补齐差速底盘的操作手册。
- 电池分压参数现场核对后启用欠压停机。
