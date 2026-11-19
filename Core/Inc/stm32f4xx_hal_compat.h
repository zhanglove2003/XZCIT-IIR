/**
  ******************************************************************************
  * @file    stm32f4xx_hal_compat.h
  * @brief   StdPeriph V1.4.0 CMSIS 头与 STM32F4xx HAL 1.8.5 的兼容层（批次0）。
  *
  * 背景：本工程 CMSIS 设备头（Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h）
  * 是 StdPeriph V1.4.0 版本，寄存器位定义只给位掩码（如 RCC_PLLCFGR_PLLN=0x7FC0），
  * 没有 Cube 风格的 _Pos 位偏移宏；而 HAL 1.8.5 源码按 Cube 风格使用 _Pos 宏，
  * 且 Cube 版 stm32f4xx.h 本应声明 SystemCoreClock / AHBPrescTable 全局量。
  * 本头文件按旧头文件中的位掩码换算补齐位偏移，数值与
  * STM32Cube FW_F4 V1.28.x 的 stm32f407xx.h 一致。全部用 #ifndef 保护，可被覆盖。
  *
  * 由 Core/Inc/stm32f4xx_hal_conf.h 顶部 include，先于所有 HAL 模块头生效。
  * MPU/SCB/SysTick 的 _Pos/_Msk 由 core_cm4.h 提供；GPIO_MODE_Pos 等由
  * stm32f4xx_hal_gpio.h 自带，均不在此重复。
  ******************************************************************************
  */
#ifndef __STM32F4XX_HAL_COMPAT_H
#define __STM32F4XX_HAL_COMPAT_H

#include <stdint.h>   /* uint32_t/uint8_t：本头先于 stm32f4xx.h 被 include */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- RCC 位偏移（换算自旧头 stm32f4xx.h 位掩码） --------------- */
#ifndef RCC_CR_HSITRIM_Pos            /* RCC_CR_HSITRIM       0x000000F8 */
#define RCC_CR_HSITRIM_Pos            (3U)
#endif
#ifndef RCC_CFGR_SWS_Pos              /* RCC_CFGR_SWS         0x0000000C */
#define RCC_CFGR_SWS_Pos              (2U)
#endif
#ifndef RCC_CFGR_HPRE_Pos             /* RCC_CFGR_HPRE        0x000000F0 */
#define RCC_CFGR_HPRE_Pos             (4U)
#endif
#ifndef RCC_CFGR_PPRE1_Pos            /* RCC_CFGR_PPRE1       0x00000700 */
#define RCC_CFGR_PPRE1_Pos            (8U)
#endif
#ifndef RCC_CFGR_PPRE2_Pos            /* RCC_CFGR_PPRE2       0x00003800 */
#define RCC_CFGR_PPRE2_Pos            (11U)
#endif
#ifndef RCC_PLLCFGR_PLLM_Pos          /* RCC_PLLCFGR_PLLM     0x0000003F */
#define RCC_PLLCFGR_PLLM_Pos          (0U)
#endif
#ifndef RCC_PLLCFGR_PLLN_Pos          /* RCC_PLLCFGR_PLLN     0x00007FC0 */
#define RCC_PLLCFGR_PLLN_Pos          (6U)
#endif
#ifndef RCC_PLLCFGR_PLLP_Pos          /* RCC_PLLCFGR_PLLP     0x00030000 */
#define RCC_PLLCFGR_PLLP_Pos          (16U)
#endif
#ifndef RCC_PLLCFGR_PLLQ_Pos          /* RCC_PLLCFGR_PLLQ     0x0F000000 */
#define RCC_PLLCFGR_PLLQ_Pos          (24U)
#endif
#ifndef RCC_PLLCFGR_PLLR_Pos          /* F407 无 PLLR 位，仅为满足 F446 分支编译 */
#define RCC_PLLCFGR_PLLR_Pos          (28U)
#endif
#ifndef RCC_PLLI2SCFGR_PLLI2SM_Pos    /* RCC_PLLI2SCFGR_PLLI2SM 0x0000003F */
#define RCC_PLLI2SCFGR_PLLI2SM_Pos    (0U)
#endif
#ifndef RCC_PLLI2SCFGR_PLLI2SN_Pos    /* RCC_PLLI2SCFGR_PLLI2SN 0x00007FC0 */
#define RCC_PLLI2SCFGR_PLLI2SN_Pos    (6U)
#endif
#ifndef RCC_PLLI2SCFGR_PLLI2SP_Pos    /* F407 无 PLLI2SP 位，仅为满足编译 */
#define RCC_PLLI2SCFGR_PLLI2SP_Pos    (16U)
#endif
#ifndef RCC_PLLI2SCFGR_PLLI2SQ_Pos    /* RCC_PLLI2SCFGR_PLLI2SQ 0x0F000000 */
#define RCC_PLLI2SCFGR_PLLI2SQ_Pos    (24U)
#endif
#ifndef RCC_PLLI2SCFGR_PLLI2SR_Pos    /* RCC_PLLI2SCFGR_PLLI2SR 0x70000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_Pos    (28U)
#endif
#ifndef RCC_PLLSAICFGR_PLLSAIM_Pos    /* F407 PLLSAICFGR 位布局与 PLLI2SCFGR 同构 */
#define RCC_PLLSAICFGR_PLLSAIM_Pos    (0U)
#endif
#ifndef RCC_PLLSAICFGR_PLLSAIN_Pos
#define RCC_PLLSAICFGR_PLLSAIN_Pos    (6U)
#endif
#ifndef RCC_PLLSAICFGR_PLLSAIP_Pos
#define RCC_PLLSAICFGR_PLLSAIP_Pos    (16U)
#endif
#ifndef RCC_PLLSAICFGR_PLLSAIQ_Pos
#define RCC_PLLSAICFGR_PLLSAIQ_Pos    (24U)
#endif
#ifndef RCC_PLLSAICFGR_PLLSAIR_Pos
#define RCC_PLLSAICFGR_PLLSAIR_Pos    (28U)
#endif
#ifndef RCC_DCKCFGR_PLLI2SDIVQ_Pos    /* RCC_DCKCFGR_PLLI2SDIVQ 0x0000001F */
#define RCC_DCKCFGR_PLLI2SDIVQ_Pos    (0U)
#endif
#ifndef RCC_DCKCFGR_PLLSAIDIVQ_Pos    /* RCC_DCKCFGR_PLLSAIDIVQ 0x00001F00 */
#define RCC_DCKCFGR_PLLSAIDIVQ_Pos    (8U)
#endif
#ifndef RCC_DCKCFGR_PLLSAIDIVR_Pos    /* RCC_DCKCFGR_PLLSAIDIVR 0x00030000 */
#define RCC_DCKCFGR_PLLSAIDIVR_Pos    (16U)
#endif
#ifndef RCC_DCKCFGR_TIMPRE_Pos        /* RCC_DCKCFGR_TIMPRE     0x01000000 */
#define RCC_DCKCFGR_TIMPRE_Pos        (24U)
#endif

/* ---------------- FLASH / PWR / SYSCFG 位偏移 ------------------------------ */
#ifndef FLASH_CR_SNB_Pos              /* FLASH_CR_SNB          0x000000F8 */
#define FLASH_CR_SNB_Pos              (3U)
#endif
#ifndef PWR_CR_PVDE_Pos               /* PWR_CR_PVDE           0x00000010 */
#define PWR_CR_PVDE_Pos               (4U)
#endif
#ifndef PWR_CR_DBP_Pos                /* PWR_CR_DBP            0x00000100 */
#define PWR_CR_DBP_Pos                (8U)
#endif
#ifndef PWR_CR_VOS_Pos                /* PWR_CR_VOS            0x0000C000 */
#define PWR_CR_VOS_Pos                (14U)
#endif
#ifndef PWR_CSR_EWUP_Pos              /* PWR_CSR_EWUP          0x00000100 */
#define PWR_CSR_EWUP_Pos              (8U)
#endif
#ifndef PWR_CSR_UDRDY                 /* F42x/43x Under-Drive ready 位（F407 保留位），
                                         旧头无此宏但 ll_pwr.h 的 UD 分支引用
                                         （PWR_CR_UDEN 在旧头已定义），取 Cube F42x 数值 */
#define PWR_CSR_UDRDY                 (PWR_CSR_UDRDY_0 | PWR_CSR_UDRDY_1)
#endif
#ifndef PWR_CSR_UDRDY_0
#define PWR_CSR_UDRDY_0               (0x00000010U)
#endif
#ifndef PWR_CSR_UDRDY_1
#define PWR_CSR_UDRDY_1               (0x00000020U)
#endif
#ifndef SYSCFG_CMPCR_CMP_PD_Pos       /* SYSCFG_CMPCR_CMP_PD   0x00000001 */
#define SYSCFG_CMPCR_CMP_PD_Pos       (0U)
#endif
#ifndef SYSCFG_MEMRMP_UFB_MODE_Pos    /* UFB_MODE 位（F42x/43x 分支用） */
#define SYSCFG_MEMRMP_UFB_MODE_Pos    (8U)
#endif
#ifndef SYSCFG_MCHDLYCR_BSCKSEL_Pos   /* F410/446 分支用，F407 无此寄存器 */
#define SYSCFG_MCHDLYCR_BSCKSEL_Pos   (26U)
#endif

/* --------------- HAL 源码引用的全局量（Cube 设备头本应声明） ---------------- */
extern uint32_t SystemCoreClock;              /* 核心时钟频率(HCLK)，定义于 system_stm32f4xx.c */
extern const uint8_t  AHBPrescTable[16];      /* AHB 分频换算表，定义于 system_stm32f4xx.c */
extern const uint8_t  APBPrescTable[8];       /* APB 分频换算表（HAL_RCC_GetPCLK1/2Freq 用），
                                                 Cube 设备头本应定义，取 FW_F4 V1.28.x 数值 */

/* --------------- Cube 设备头本应定义的基址（RM0090，F40x/41x） --------------- */
#ifndef UID_BASE                      /* 96 位唯一设备 ID 基址（HAL_GetUIDw0/1/2 用） */
#define UID_BASE                      (0x1FFF7A18UL)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4XX_HAL_COMPAT_H */
