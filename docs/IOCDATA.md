# IOCDATA.md — `C50C_HAL.ioc` 字段 ↔ 引脚对照表

生成日期：2026-11-19。`.ioc` 为手写（依据 `docs/PINOUT.md` + 现有 `C50C_StdPeriph.ioc` 命令记录 + 固件时钟源码），
**未经 CubeMX 实际打开/保存验证**。首次用 STM32CubeMX 打开时请按本表逐项核对，尤其是下表标 ⚠ 的字段；
CubeMX 重存一次后以它生成的文件为准。

文件：`st32f407/C50C_HAL.ioc`（STM32F407VETx，LQFP100，FW_F4 V1.28.3，Makefile 工具链）

## 1. 时钟树（来源：`Core/Src/system_stm32f4xx.c:316-325` PLL_M=8/N=336/P=2/Q=7；PPRE1=DIV4、PPRE2=DIV2；`Core/Src/system.c:36` `delay_init(168)`；HSE=8 MHz 来自原理图 `C50C-STM32F407-StdPeriph/docs/引脚配置与核对说明.md:38`）

| .ioc 字段 | 值 | 含义 | 核对 |
|---|---|---|---|
| RCC.HSE_VALUE | 8000000 | 外部晶体 8 MHz | ⚠ CubeMX Clock Configuration 页确认 HSE=8MHz、PLL Source=MUL by PREDIV(A)=HSE |
| RCC.PLLM / PLLN / PLLQ | 8 / 336 / 7 | VCO = 8/8*336 = 336 MHz | ⚠ 页面上对应 /M、×N、/Q |
| RCC.SYSCLKSource | PLLCLK | SYSCLK=168 MHz（PLL_P=2 在 CubeMX 里由 SYSCLK=VCO/2 隐含） | ⚠ 确认 /P=2 |
| RCC.APB1Freq_Value | 42000000 | AHB/4，APB1 定时器倍频后 84 MHz | ⚠ |
| RCC.APB2Freq_Value | 84000000 | AHB/2，APB2 定时器倍频后 168 MHz | ⚠ |
| RCC.PLLQCLKFreq_Value | 48000000 | USB/RNG 48 MHz（当前未用 USB） | — |

注意：`.ioc` 不含 PLL_P 字段，168 MHz 与 VCO=336 的组合在 CubeMX 中自动取 /P=2；打开后若 Clock 页报红线请回看本节。

## 2. 引脚 ↔ .ioc 字段对照（Signal/模式/上下拉/速率，全部来自 `docs/PINOUT.md` 总表）

| 引脚 | PINOUT.md 功能 | .ioc Signal | 模式/上下拉/速率 | 核对 |
|---|---|---|---|---|
| PC9 | TIM8_CH4 PWM（PWMA） | `PC9.Signal=TIM8_CH4` | AF，上拉，High | ⚠ TIM8 页确认 4 通道 PWM、ARR=16799（`TIM8.Period`）、PSC=0 |
| PC8 | TIM8_CH3（PWMB） | `PC8.Signal=TIM8_CH3` | 同上 | ⚠ |
| PC7 | TIM8_CH2（PWMC） | `PC7.Signal=TIM8_CH2` | 同上 | ⚠ |
| PC6 | TIM8_CH1（PWMD） | `PC6.Signal=TIM8_CH1` | 同上 | ⚠ |
| PB13/PB12/PC0/PE15/PD10/PD12/PC12/PA8 | 电机方向 AIN1..DIN2 | `GPIO_Output` | 推挽，上拉，Low(2MHz) | — |
| PA15/PB3 | TIM2 编码器 A | `TIM2_CH1/CH2` | Encoder Mode TI12，上拉 | ⚠ CubeMX 里编码器 GPIO 默认无上下拉，需在 TIM2 GPIO 设置改 Pull-up |
| PA6/PA7 | TIM3 编码器 B | `TIM3_CH1/CH2` | 同上 | ⚠ |
| PB6/PB7 | TIM4 编码器 C | `TIM4_CH1/CH2` | 同上 | ⚠ |
| PA0-WKUP/PA1 | TIM5 编码器 D | `TIM5_CH1/CH2` | 同上 | ⚠ PA0 在 CubeMX 显示为 `PA0-WKUP` |
| PA9/PA10 | USART1 115200 | `USART1_TX/RX` | AF7，上拉，Medium(50MHz) | ⚠ USART1 页 BaudRate=115200 |
| PB10/PB11 | USART3 115200 | `USART3_TX/RX` | 同上 | ⚠ |
| PC10/PC11 | UART4 9600 | `UART4_TX/RX` | AF8，上拉，Medium | ⚠ UART4 页 BaudRate=9600（无线版需 230400） |
| PB9/PB8 | 软件 I2C SCL/SDA | `GPIO_Output` | 推挽，上拉，High(100MHz) | ⚠ SDA 输入/输出切换由用户代码控制，HAL 生成仅初始输出 |
| PD8 | 超声波 Trig | `GPIO_Output` | 推挽，下拉，Low(2MHz) | — |
| PD9 | 超声波 Echo（EXTI9） | `PD9.Signal=EXTI9` | 上升沿 EXTI，下拉，Low | ⚠ NVIC 页确认 EXTI9_5 已使能（本 ioc 已写 `NVIC.EXTI9_5_IRQn`） |
| PE0 | PS2 DI | `GPIO_Input` | 下拉，High | — |
| PE1/PE2/PE3 | PS2 DO/CS/CLK | `GPIO_Output` | 推挽，上拉，Low | — |
| PB15/PE6/PB14 | LED R/G/B（当前生效版） | `GPIO_Output` | 推挽，上拉，High | — |
| PD11 | 蜂鸣器 | `GPIO_Output` | 推挽，上拉，High | — |
| PC5 | 用户按键 | `GPIO_Input` | 上拉 | — |
| PE4 | 使能开关 | `GPIO_Input` | 上拉 | — |
| PD7/PD6/PD5/PD4 | OLED SCL/SDA/RST/DC（旧版映射） | `GPIO_Output` | 推挽，无上下拉，Low | ⚠ 本 ioc 未为这 4 脚写 GPIO_PuPd（CubeMX 默认 No pull） |
| PB0 | ADC1_IN8 电池电压 | `ADC1_IN8`，ADC1 Analog | 模拟，无上下拉 | ⚠ ADC1 页使能 IN8；本 ioc 未写采样参数 |
| PC2 | ADC1_IN12 CarMode 电位器 | `ADC1_IN12` | 同上 | ⚠ |
| PA13/PA14 | SWDIO/SWCLK | `SYS_JTMS-SWDIO/JTCK-SWCLK` | Serial Wire | — |
| PH0/PH1 | HSE 晶体 | `RCC_OSC_IN/OUT` | HSE-External-Oscillator | ⚠ RCC 页模式选 Crystal/Ceramic Resonator |

## 3. 其他顶层字段

| .ioc 字段 | 值 | 说明 | 核对 |
|---|---|---|---|
| FREERTOS CMSIS_V1 | `VP_FREERTOS_VS_CMSIS_V1.Mode=CMSIS_V1` | 与现有任务 API 对齐（现有 App 用 `vTaskDelay/vTaskDelayUntil/xTaskGetSchedulerState`，`App/BALANCE/led_task.c:33,122` 等为 FreeRTOS 原生 API，CMSIS_V1 包装层兼容） | ⚠ Middleware 页确认勾选 CMSIS_V1；TICK_RATE 与 `Drivers/BSP/delay/delay.c:42` `delay_init` 的 `configTICK_RATE_HZ` 对齐 |
| FREERTOS.configTOTAL_HEAP_SIZE | 30720 | 默认值，需按 4 任务 + 队列评估 | ⚠ |
| NVIC.PriorityGroup | 4（全抢占） | 与 FreeRTOS 惯例一致 | ⚠ |
| NVIC.PendSV/SysTick | 15（最低） | FreeRTOS 要求 | ⚠ 确认 FreeRTOS 页 "Timebase Source" 选了非 SysTick 定时器（如 TIM6/TIM7）——**本 ioc 未启用 HAL timebase 定时器，需在 CubeMX 图形化设置** |
| TIM2..TIM5 Encoder 模式 | Encoder Mode TI12 | 对应 `encoder.c` TIM_EncoderMode_TI12；CNT 滤波、计数周期等参数本 ioc 未写 | ⚠ 编码器各页 Counter Period/IC Filter 按源码补 |
| TIM8 Period/Prescaler | 16799 / 0 | `system.c:59` `V1_0_MiniBalance_PWM_Init(16799,0)` | ⚠ |
| UART 波特率 | 115200/115200/9600 | `system.c:45,89-96` | ⚠ |
| ProjectManager.LibraryCopy | 0（Copy only necessary files） | — | — |
| ProjectManager.TargetToolchain | Makefile | 若用 CubeIDE/CMake 请改 | ⚠ |

## 4. 与现有固件的已知差异 / 未覆盖项

- V1_0 板 LED（PD14/PD13/PD15，`system.c:58`）未配置——本 ioc 采用 PINOUT.md 总表的 PB15/PE6/PB14 版本；两版不能共存，需按实物板版本取舍。
- UART4 无线版 230400 波特率按编译宏分支决定，.ioc 固定写 9600。
- 软件 I2C（PB8/PB9）、软件 SPI OLED、PS2 均为 GPIO 模拟，HAL 生成代码仅为初始电平，时序逻辑在用户代码。
- PB7 舵机复用冲突（PINOUT.md §1）当前四驱车型无舵机，未配置。
- 引脚的 GPIO_Label（如 MOTOR_A_IN1）未写入本 ioc，可在 CubeMX 中按 `C50C-STM32F407-StdPeriph/docs/CubeMX引脚配置命令记录.txt` 的标签补。
