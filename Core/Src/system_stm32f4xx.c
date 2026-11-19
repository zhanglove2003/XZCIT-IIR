/**
  ******************************************************************************
  * @file    system_stm32f4xx.c
  * @brief   CMSIS Cortex-M4 设备层系统源文件（批次0：时钟树与 RCC 迁移到 HAL）。
  *
  * 迁移说明（StdPeriph -> HAL 1.8.5）：
  *  - 原手写 SetSysClock()（寄存器级 HSE/PLL/FLASH/SYSCLK 配置）与 PLL_M/N/P/Q
  *    宏定义已删除，改用 HAL_RCC_OscConfig() + HAL_RCC_ClockConfig()：
  *      HSE 8MHz -> PLL(M=8, N=336, P=2, Q=7) -> SYSCLK=168MHz,
  *      AHB=168MHz, APB1=42MHz(÷4), APB2=84MHz(÷2), FLASH_LATENCY_5(5WS)。
  *      与原配置逐寄存器等价（原 PLLCFGR = 8 | 336<<6 | 0<<16 | PLLSRC_HSE | 7<<24）。
  *  - HSE_VALUE=8000000 定义于 stm32f4xx_hal_conf.h（与旧 stm32f4xx.h:123 一致，
  *    板载 8MHz 晶振；头文件注释里的 25MHz 从未生效）。
  *  - PWR 调压器 Scale1 由本文件显式设置（等价旧 system_stm32f4xx.c:594-595 的
  *    PWR->CR |= PWR_CR_VOS）。注意：HAL 1.8.5 的 HAL_RCC_ClockConfig() 并不
  *    内建 VOS 设置（源码 stm32f4xx_hal_rcc.c 全函数无 VOS 操作），故不能省略。
  *  - SystemInit() 保留原有 FPU 使能与向量表重定位；RCC 复位块保留（寄存器级，
  *    与原行为一致）；其余初始化交由 HAL_Init()/HAL_RCC_*。
  *  - SystemCoreClock/AHBPrescTable 定义保留，声明由 stm32f4xx_hal_compat.h 提供。
  ******************************************************************************
  */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_pwr.h"

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f4xx_system
  * @{
  */

/** @addtogroup STM32F4xx_System_Private_Defines
  * @{
  */

/* 向量表偏移：与 Core/Inc/system.h:5 的定义保持一致（旧文件经 system.h 间接引入） */
#define VECT_TAB_OFFSET  0x00000

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Variables
  * @{
  */
  uint32_t SystemCoreClock = 168000000;  /* HCLK = SYSCLK = 168MHz（批次0 起由 HAL 维护） */

  const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

  /* APB 分频换算表：Cube FW_F4 V1.28.x 设备头数值（HAL_RCC_GetPCLK1/2Freq 引用），
     旧 StdPeriph CMSIS 头中无此定义 */
  const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_FunctionPrototypes
  * @{
  */
static void SystemClock_Config(void);

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU, reset the RCC clock configuration to the default
  *         reset state, configure the system clock via HAL and relocate the
  *         vector table. Called from startup before main().
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif

  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* （保留原 system_stm32f4xx.c:423-440 的寄存器级复位块，行为等价） */
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x24003010;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;

  /* HAL 公共初始化：Flash 预取/指令缓存/数据缓存（等价旧 FLASH->ACR 的
     PRFTEN|ICEN|DCEN）、NVIC 分组 4（等价旧 NVIC_PriorityGroupConfig(Group_4)）、
     以 SystemCoreClock 配置 1ms SysTick 时间基准（HAL_RCC_* 的超时依赖 uwTick，
     uwTick 由 Drivers/BSP/delay/delay.c 的 SysTick_Handler 调 HAL_IncTick() 维护） */
  HAL_Init();

  /* Configure the System clock source, PLL Multiplier and Divider factors,
     AHB/APBx prescalers and Flash settings ----------------------------------*/
  SystemClock_Config();

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif
}

/**
  * @brief  Configures the System clock via HAL (replaces the old register-level
  *         SetSysClock(), system_stm32f4xx.c:564-660 of the StdPeriph tree).
  *         HSE 8MHz -> PLL(M=8,N=336,P=2,Q=7) -> SYSCLK=168MHz,
  *         AHB=/1, APB1=/4 (42MHz), APB2=/2 (84MHz), FLASH 5WS.
  * @Note   Must be called once after the RCC reset done in SystemInit().
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Enable PWR clock and select regulator voltage output Scale 1 mode.
     等价旧代码 RCC->APB1ENR |= RCC_APB1ENR_PWREN; PWR->CR |= PWR_CR_VOS;
     （system_stm32f4xx.c:593-595）。HAL 1.8.5 的 stm32f4xx_hal_pwr.h 未提供
     __HAL_PWR_VOLTAGESCALING_CONFIG，故用 LL_PWR 等价实现；
     LL_PWR_REGU_VOLTAGE_SCALE1 == PWR_CR_VOS（见 stm32f4xx_ll_pwr.h:85-92）。 */
  __HAL_RCC_PWR_CLK_ENABLE();
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

  /* Enable HSE and configure the main PLL.
     PLL_VCO = 8MHz/8 * 336 = 336MHz; SYSCLK = 336MHz/2 = 168MHz;
     USB OTG FS/SDIO/RNG = 336MHz/7 = 48MHz（与旧 PLL_Q=7 一致）。 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 8;
  RCC_OscInitStruct.PLL.PLLN       = 336;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ       = 7;
  /* HSE 起振失败时旧代码同样继续运行（错误分支为空），故此处不中断启动 */
  (void)HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure HCLK/APB1/APB2.
     等价旧代码 HPRE=/1, PPRE1=/4, PPRE2=/2, FLASH_LATENCY_5WS, SW=PLL。 */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  (void)HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  /* HAL_RCC_ClockConfig 末尾会按新时钟重算 SystemCoreClock=168MHz 并重配 SysTick，
     delay_init(168) 的 fac_us=168 依赖保持成立（主频不变）。 */
}

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         逻辑与原实现（system_stm32f4xx.c:494-554）逐行等价：读 RCC->CFGR/
  *         RCC->PLLCFGR 重算。HSE_VALUE=8000000 来自 stm32f4xx.h:123，
  *         与 stm32f4xx_hal_conf.h:99 一致。
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;

  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock source */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock source */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08:  /* PLL used as system clock source */
       /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_P
         */
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

      if (pllsource != 0)
      {
        /* HSE used as PLL clock source */
        pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }
      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
      SystemCoreClock = pllvco/pllp;
      break;
    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* Compute HCLK frequency --------------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK frequency */
  SystemCoreClock >>= tmp;
}

/**
  * @}
  */

/**
  * @}
  */
