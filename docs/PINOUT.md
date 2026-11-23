# ST32F407 (WHEELTEC C50C) 引脚分配表

依据：本表全部来自对 `Drivers/BSP`、`Core/Src/system.c` 源码的实读（源文件为 GB2312/GB18030 编码，用 `iconv -f GB18030` 解码后逐行核对）。标注说明：
- 【实读】= 该引脚出现在 GPIO_Init / GPIO_PinAFConfig / RCC 时钟使能等初始化代码或其直接引用的宏定义中；
- 【宏定义】= 仅在模块头文件的 `#define` 中出现（未在该次编译路径的初始化代码中实际执行，例如 `uartx.h` 的 `#else` 分支、V1_0 备用配置）。

当前固件实际生效的配置由 `systemInit()`（`Core/Src/system.c:30-157`）决定：V1_0 旧版板走 `V1_0_LED_Init()` + `V1_0_MiniBalance_PWM_Init()`；UART 引脚由 `uartx.h` 中 `#if 1` 分支选定（PA9/PA10、PB10/PB11、PC10/PC11）。

## 总表

| 引脚 | 复用功能/方向 | 速率/上下拉 | 所属模块 | 用途 |
|---|---|---|---|---|
| PC9 | TIM8_CH4 复用推挽输出 AF3 | 100MHz, 上拉 | motor.c | 电机 A PWM（PWMA） |
| PC8 | TIM8_CH3 复用推挽输出 AF3 | 100MHz, 上拉 | motor.c | 电机 B PWM（PWMB） |
| PC7 | TIM8_CH2 复用推挽输出 AF3 | 100MHz, 上拉 | motor.c | 电机 C PWM（PWMC） |
| PC6 | TIM8_CH1 复用推挽输出 AF3 | 100MHz, 上拉 | motor.c | 电机 D PWM（PWMD） |
| PB13 | GPIO 输出推挽 | 2MHz, 上拉 | motor.c | AIN1 电机 A 方向 1 |
| PB12 | GPIO 输出推挽 | 2MHz, 上拉 | motor.c | AIN2 电机 A 方向 2 |
| PC0 | GPIO 输出推挽 | 2MHz, 上拉 | motor.c | BIN1 电机 B 方向 1 |
| PE15 | GPIO 输出推挽 | 2MHz, 上拉 | motor.c | BIN2 电机 B 方向 2 |
| PD10 | GPIO 输出推挽 | 2MHz, 上拉 | motor.c | CIN1 电机 C 方向 1 |
| PD12 | GPIO 输出推挽 | 2MHz, 上拉 | motor.c | CIN2 电机 C 方向 2 |
| PC12 | GPIO 输出推挽 | 2MHz, 上拉 | motor.c | DIN1 电机 D 方向 1 |
| PA8 | GPIO 输出推挽 | 2MHz, 上拉 | motor.c | DIN2 电机 D 方向 2 |
| PA15 | TIM2_CH1 复用开漏 AF1 | 100MHz, 上拉 | encoder.c | 编码器 A 相 1 |
| PB3 | TIM2_CH2 复用开漏 AF1 | 100MHz, 上拉 | encoder.c | 编码器 A 相 2 |
| PA6 | TIM3_CH1 复用开漏 AF2 | 100MHz, 上拉 | encoder.c | 编码器 B 相 1 |
| PA7 | TIM3_CH2 复用开漏 AF2 | 100MHz, 上拉 | encoder.c | 编码器 B 相 2 |
| PB6 | TIM4_CH1 复用开漏 AF2 | 100MHz, 上拉 | encoder.c | 编码器 C 相 1 |
| PB7 | TIM4_CH2 复用开漏 AF2 | 100MHz, 上拉 | encoder.c | 编码器 C 相 2 |
| PA0 | TIM5_CH1 复用开漏 AF2 | 100MHz, 上拉 | encoder.c | 编码器 D 相 1 |
| PA1 | TIM5_CH2 复用开漏 AF2 | 100MHz, 上拉 | encoder.c | 编码器 D 相 2 |
| PA9 | USART1_TX 复用 AF7 | 50MHz, 上拉 | uartx.c | 串口1 发送，115200 |
| PA10 | USART1_RX 复用 AF7 | 50MHz, 上拉 | uartx.c | 串口1 接收，115200 |
| PB10 | USART3_TX 复用 AF7 | 50MHz, 上拉 | uartx.c | 串口3 发送，115200（ROS 通信默认口） |
| PB11 | USART3_RX 复用 AF7 | 50MHz, 上拉 | uartx.c | 串口3 接收，115200 |
| PC10 | UART4_TX 复用 AF8 | 50MHz, 上拉 | uartx.c | 串口4 发送，9600（蓝牙 APP；无线版 230400） |
| PC11 | UART4_RX 复用 AF8 | 50MHz, 上拉 | uartx.c | 串口4 接收 |
| PB9 | GPIO 输出推挽（软件 I2C SCL） | 100MHz, 上拉 | I2C.c | IIC 时钟线（IMU 总线） |
| PB8 | GPIO 输出推挽/输入切换（软件 I2C SDA） | 100MHz, 上拉 | I2C.c | IIC 数据线 |
| PD8 | GPIO 输出推挽 | 2MHz, 下拉 | hc_sr04.c | 超声波 Trig |
| PD9 | GPIO 输入 + EXTI9 | 2MHz, 下拉 | hc_sr04.c | 超声波 Echo（EXTI_Line9, EXTI9_5_IRQn） |
| PE0 | GPIO 输入（下拉） | 100MHz, 下拉 | ps2_classic.c | PS2 手柄 DI（数据输入） |
| PE1 | GPIO 输出推挽 | 2MHz, 上拉 | ps2_classic.c | PS2 手柄 DO（命令输出） |
| PE2 | GPIO 输出推挽 | 2MHz, 上拉 | ps2_classic.c | PS2 手柄 CS |
| PE3 | GPIO 输出推挽 | 2MHz, 上拉 | ps2_classic.c | PS2 手柄 CLK |
| PB15 | GPIO 输出推挽 | 100MHz, 上拉 | led.c | LED_R 红灯（当前生效版） |
| PE6 | GPIO 输出推挽 | 100MHz, 上拉 | led.c | LED_G 绿灯 |
| PB14 | GPIO 输出推挽 | 100MHz, 上拉 | led.c | LED_B 蓝灯 |
| PD11 | GPIO 输出推挽 | 100MHz, 上拉 | buzzer.c | 蜂鸣器 |
| PC5 | GPIO 输入上拉 | 100MHz, 上拉 | key.c | 用户按键 KEY |
| PE4 | GPIO 输入上拉 | 100MHz, 上拉 | enable_key.c | 使能开关 EN |
| PD7 | GPIO 输出推挽 | 2MHz, 无上下拉 | oled.c | OLED SCL（旧版引脚映射） |
| PD6 | GPIO 输出推挽 | 2MHz, 无上下拉 | oled.c | OLED SDA |
| PD5 | GPIO 输出推挽 | 2MHz, 无上下拉 | oled.c | OLED RST(RES) |
| PD4 | GPIO 输出推挽 | 2MHz, 无上下拉 | oled.c | OLED DC(RS) |
| PB0 | ADC1_IN8 模拟输入 | 无上下拉 | adc.c | 电池电压采集 |
| PC2 | ADC1_IN12 模拟输入 | 无上下拉 | adc.c | 车型调节电位器 CarMode |

## 按模块分组

### 1. 电机 PWM（TIM8 四通道 + 方向脚）

来源：`Drivers/BSP/motor/motor.h`（宏定义）+ `Drivers/BSP/motor/motor.c`（`TIMx_PWM1_Mode_Init` 实读配置：GPIO_Mode_AF、推挽、GPIO_Speed_100MHz、上拉，`motor.c` 约 120-130 行；方向脚 `GPIO_Output_Init`：Mode_OUT、推挽、2MHz、上拉，`motor.c:16-24`）。

| 电机 | PWM 引脚 | TIM8 通道 | 方向引脚 1 | 方向引脚 2 |
|---|---|---|---|---|
| A | PC9 | CH4 | PB13 (AIN1) | PB12 (AIN2) |
| B | PC8 | CH3 | PC0 (BIN1) | PE15 (BIN2) |
| C | PC7 | CH2 | PD10 (CIN1) | PD12 (CIN2) |
| D | PC6 | CH1 | PC12 (DIN1) | PA8 (DIN2) |

- PWM 模式 PWM1，ARR 由 `system.c:59` 调用 `V1_0_MiniBalance_PWM_Init(16799,0)`（V1_0 板）传入，满占空比 `FULL_DUTYCYCLE 16800`（motor.h）。
- 另有 V1_0 备用 B 电机方向配置 PB14/V1_0_BIN2（motor.h【宏定义】，V1_0 init 用的是宏 `V1_0_BIN1/2`，而 `V1_0_ENABLE_BIN2_CLOCK` 指向 GPIOB，即 V1_0 板 BIN2=PB14）。
- 舵机配置（阿克曼车型）：SERVO_SENIOR = PA2/TIM9_CH1，SERVO_TOP = PB7/TIM4_CH2（motor.h【宏定义】；`system.c` 中注明"阿克曼舵机初始化已随车型裁剪删除"，当前四驱车型未初始化）。注意 PB7 与编码器 C 相 2 复用冲突，仅在舵机车型下相关。

### 2. 编码器（四路，定时器编码器模式 TI12）

来源：`Drivers/BSP/encoder/encoder.h`（宏）+ `encoder.c` `Encoder_TI12_ModeInit`（实读：GPIO_Mode_AF、开漏、100MHz、上拉；TIM_EncoderMode_TI12，滤波 6，`encoder.c:8-76`）。

| 编码器 | 定时器 | 相 1 引脚 | 相 2 引脚 |
|---|---|---|---|
| A | TIM2 (APB1) | PA15 | PB3 |
| B | TIM3 (APB1) | PA6 | PA7 |
| C | TIM4 (APB1) | PB6 | PB7 |
| D | TIM5 (APB1) | PA0 | PA1 |

四路均在 `systemInit()`（system.c:110-123）初始化。

### 3. 串口

来源：`Drivers/BSP/uartx/uartx.h`（宏，`#if 1` 分支生效）+ `uartx.c`（实读：GPIO_Mode_AF、50MHz、上拉）+ `Core/Src/system.c:45/89-96`（波特率）。

| 串口 | TX | RX | 波特率 | 用途 |
|---|---|---|---|---|
| USART1 | PA9 | PA10 | 115200 | ROS/调试通信 |
| USART3 | PB10 | PB11 | 115200 | ROS 通信默认串口 |
| UART4 | PC10 | PC11 | 9600（`VECT_TAB_OFFSET==0`）；无线烧录版 230400 | 蓝牙 APP |

`uartx.h` 中的备选引脚组（USART1→PB6/PB7、USART3→PD8/PD9、UART4→PA0/PA1）在 `#else` 分支【宏定义】，当前未编译。注意 PD8/PD9 备选与超声波引脚冲突，hc_sr04.h 注释已确认 USART3 用 PB10/PB11，PD8/PD9 留给超声波。

### 4. IMU（I2C，软件模拟）

来源：`Drivers/BSP/I2C/I2C.h`（宏）+ `I2C.c` `I2C_GPIOInit`（实读：Mode_OUT 推挽、100MHz、上拉，SDA 通过 MODER 在输入/输出间切换）。

| 引脚 | 功能 |
|---|---|
| PB9 | IIC_SCL |
| PB8 | IIC_SDA（可切输入 READ_SDA） |

MPU6050 通过该软件 I2C 读写（`Drivers/BSP/MPU6050/MPU6050.c` 全部经 `I2C_ReadOneByte/I2C_WriteOneByte`，无独立 GPIO）。`system.c:40-42` 最先初始化 IIC，用 MPU6050 WHO_AM_I 区分新旧板。

### 5. 超声波（HC-SR04）

来源：`Drivers/BSP/hc_sr04/hc_sr04.h`（宏）+ `hc_sr04.c:34-52`（实读）。

| 引脚 | 方向 | 配置 | 功能 |
|---|---|---|---|
| PD8 | 输出推挽 | 2MHz, 下拉 | Trig |
| PD9 | 输入 + EXTI_Line9（EXTI_PortSourceGPIOD, EXTI9_5_IRQn） | 2MHz, 下拉 | Echo |

### 6. PS2 手柄（四线，GPIO 模拟时序）

来源：`Drivers/BSP/ps2_classic/pstwo.h`（宏）+ `ps2_classic.c` `PS2_Init`（实读：DI 输入下拉 100MHz；DO/CS/CLK 输出推挽 2MHz 上拉）。

| 引脚 | 方向 | 功能 |
|---|---|---|
| PE0 | 输入，下拉 | DI 数据输入 |
| PE1 | 输出推挽 | DO 命令输出 |
| PE2 | 输出推挽 | CS 片选 |
| PE3 | 输出推挽 | CLK 时钟 |

### 7. OLED（软件 SPI，双版本引脚映射）

来源：`Drivers/BSP/oled/oled.c:22-33`（实读，`oled_pins_old` / `oled_pins_v1_2` 两套映射，配置均为输出推挽、2MHz、无上下拉）。

| 功能 | 旧版 oled_pins_old | V1.2 oled_pins_v1_2 |
|---|---|---|
| SCL | PD7 | PD7 |
| SDA | PD6 | PD4 |
| RST | PD5 | PE1 |
| DC | PD4 | PE0 |

- 旧版为当前默认（`oled_pins = &oled_pins_old`，`OLED_SelectHardware()` 按 `SysVal.HardWare_Ver == V1_2` 切换）。
- 注意：V1.2 映射的 PE1/PE0 与 PS2 手柄 DI/CS 引脚重叠，两套外设不能同时启用该映射。

### 8. LED / 蜂鸣器 / 按键

来源：`Drivers/BSP/led/led.{h,c}`、`buzzer.{h,c}`、`key.{h,c}`、`enable_key.{h,c}`（均为宏 + 实读初始化，全部 GPIO_Mode_OUT 推挽 100MHz 上拉或 GPIO_Mode_IN 100MHz 上拉）。

| 引脚 | 方向 | 模块 | 用途 |
|---|---|---|---|
| PB15 | 输出推挽, 100MHz, 上拉 | led.c | LED_R 红灯 |
| PE6 | 输出推挽, 100MHz, 上拉 | led.c | LED_G 绿灯 |
| PB14 | 输出推挽, 100MHz, 上拉 | led.c | LED_B 蓝灯 |
| PD14 / PD13 / PD15 | 输出推挽 | led.c | V1_0 版 R/G/B（`V1_0_LED_Init()`，system.c:58 生效于旧版板） |
| PD11 | 输出推挽, 100MHz, 上拉 | buzzer.c | 蜂鸣器 |
| PC5 | 输入, 100MHz, 上拉 | key.c | 用户按键 |
| PE4 | 输入, 100MHz, 上拉 | enable_key.c | 使能开关 |

注：当前 `systemInit()` 走 V1_0 分支时 LED 用 PD14/PD13/PD15；PB15/PE6/PB14 为另一版配置（`LED_Init` 亦在 led.c 中实现）。RGB 灯带（LightStrip）配置：PB15/TIM12_CH2、PE6/TIM9_CH2、PB14/TIM12_CH1（led.h【宏定义】，bsp_RGBLight 模块）。

### 9. ADC（补充，电池电压/车型选择）

来源：`Drivers/BSP/adc/adc.h`（宏）+ `adc.c:16-43`（实读：GPIO_Mode_AN、无上下拉）。

| 引脚 | 通道 | 用途 |
|---|---|---|
| PB0 | ADC1_IN8 | 电池电压 |
| PC2 | ADC1_IN12 | 车型调节电位器 CarMode |
| PC1 | ADC2_IN11 | 阿克曼转向滑轨 SLIDE（【宏定义】adc.h，本车型未用） |
| PC3 | ADC2_IN13 | 舵机调整电位器 ServoAdj（【宏定义】adc.h，本车型未用） |

## 来源核对说明

- 以上所有"实读"配置均通过 `iconv -f GB18030 -t UTF-8//TRANSLIT -c <file>` 逐文件读取并 grep `GPIO_Init/GPIO_PinAFConfig/RCC_*PeriphClockCmd` 核对；本会话未做编译或烧录验证。
- 未在本表列入凭空推测的引脚；`uartx.h`、`motor.h`、`led.h`、`adc.h` 中未生效的 `#else`/备用宏已单独标注【宏定义】。
- 波特率来源于 `Core/Src/system.c:45`（UART1 115200）、`:89/:91`（UART4 230400/9600，由 `VECT_TAB_OFFSET` 编译分支决定）、`:96`（UART3 115200）。
