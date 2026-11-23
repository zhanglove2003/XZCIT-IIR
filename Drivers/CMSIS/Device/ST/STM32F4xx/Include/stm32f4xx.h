#ifndef __STM32F4xx_H
#define __STM32F4xx_H

/* 批次6：CMSIS 设备头 shim 切换。
 * 本文件原为 StdPeriph V1.4.0 胖头（9176 行，legacy 器件门 STM32F40_41xxx，
 * 无 _Pos/_Msk 全集）。StdPeriph 随批次1-5 迁移、批次6 退场后，保留文件名
 * "stm32f4xx.h" 以维持树内既有 include 面（hal_def.h、sys.h、system.h、
 * stm32f4xx_it.h、LL 头等十余处），内容整体切换到同目录 Cube 版
 * stm32f407xx.h（FW_F4 1.28.x 格式，自带全部 _Pos/_Msk、ADC123_COMMON、
 * 32 位 BSRR、IRQn 枚举）。全树批量改 include 属高风险（nel/GBK 文件），
 * 蓝图裁定不做。
 * -DSTM32F407xx（Makefile DEFS）继续供 HAL _ex 模块器件宏分支使用。 */

#include "stm32f407xx.h"

/* 批次6：原 StdPeriph 胖头 stm32f4xx.h:473 被 vendor 加入 #include "arm_math.h"——
 * 全树经此隐式获得 PI 等 CMSIS-DSP 数学宏（-DARM_MATH_CM4 即为其准备）。原样保留。 */
#include "arm_math.h"

/* 批次6：原 StdPeriph 胖头 stm32f4xx.h:495-497 内嵌的 vendor 缩写 typedef 原样
 * 补回（全树 BSP/App 头函数签名使用；与旧头逐行同义，非本批新增语义）：
 *   typedef uint32_t u32; typedef uint16_t u16; typedef uint8_t u8;
 * shim 化后 stm32f407xx.h 不再提供这三个名字。 */
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

/* 批次6：原 StdPeriph 胖头 stm32f4xx.h:511-516 的状态枚举 typedef 原样补回——
 * 树内旧版 HAL 头（hal_adc.h 等）仍在使用 FunctionalState/FlagStatus/ITStatus/
 * ErrorStatus；Cube 头不提供。 */
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

/* 批次6：原 StdPeriph 胖头 stm32f4xx.h:9144-9156 尾部自带的 CMSIS 通用寄存器宏
 * 原样补回——树内 core_cm4.h 为旧版（无此宏区），Cube 头 stm32f407xx.h 亦不提供；
 * HAL 源（hal_adc.c 等）与 LL 头依赖它们。#ifndef 包裹，与未来升级的 CMSIS 冲突安全。 */
#ifndef SET_BIT
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#endif

#ifndef CLEAR_BIT
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#endif

#ifndef READ_BIT
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#endif

#ifndef CLEAR_REG
#define CLEAR_REG(REG)        ((REG) = (0x0))
#endif

#ifndef WRITE_REG
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#endif

#ifndef READ_REG
#define READ_REG(REG)         ((REG))
#endif

#ifndef MODIFY_REG
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#endif

#endif /* __STM32F4xx_H */
