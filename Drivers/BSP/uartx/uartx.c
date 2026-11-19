#include "uartx.h"
#include "stm32f4xx_hal.h"		/* 批次2：HAL_UART_Init/HAL_NVIC_* 声明（uartx.h→sys.h 链不引 HAL 头） */

//app按键定义
APP_CONTROL_t appkey;

void APPKey_Param_Init(APP_CONTROL_t *p)
{
	p->DirectionFlag = 0;
	p->ParamSaveFlag = 0;
	p->ParamSendflag = 0;
	p->TurnFlag = 0;
	p->TurnPage = 0;
}

/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
函数功能：串口1初始化
入口参数：无
返 回 值：无
**************************************************************************/
void UART1_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_HandleTypeDef UART1_Handle; /* 批次2：HAL 句柄（仅初始化期使用，ISR 为寄存器级不依赖） */

    //对应引脚、对应串口时钟开启
    ENABLE_UART1_TX_PIN_CLOCK;
    ENABLE_UART1_RX_PIN_CLOCK;
    ENABLE_UART1_CLOCK;

    //批次5：AF 经 HAL_GPIO_Init 的 Alternate 字段写入（等价原 GPIO_PinAFConfig；
    //GPIO_AF_USART1=0x07 改用 HAL 宏 GPIO_AF7_USART1 同值）；50MHz→SPEED_FREQ_HIGH。
    //对应引脚端口初始化
    GPIO_InitStructure.Pin       = UART1_TX_PIN;
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pull      = GPIO_PULLUP;
    GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(UART1_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin       = UART1_RX_PIN;
    GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(UART1_RX_PORT, &GPIO_InitStructure);

    //UsartNVIC configuration //UsartNVIC配置
    //批次2：原 NVIC_Init(USART1_IRQn, 抢占4, 子0, 分组4) → HAL 等价调用。
    //分组4（4位抢占/0位子）下 IPR 寄存器 = 抢占<<4，StdPeriph 与 NVIC_EncodePriority
    //编码一致，抢占 4 原值传入、子优先级恒 0，勿按子优先级平移。
    HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    //USART Initialization Settings 初始化设置
    //批次2：原 USART_Init(8N1, TX_RX) + USART_Cmd → HAL_UART_Init 等价（UE 由 HAL 置位）。
    UART1_Handle.Instance          = USART1;
    UART1_Handle.Init.BaudRate     = bound;                 //Port rate //串口波特率
    UART1_Handle.Init.WordLength   = UART_WORDLENGTH_8B;    //字长8位，等价 USART_WordLength_8b
    UART1_Handle.Init.StopBits     = UART_STOPBITS_1;       //一个停止位
    UART1_Handle.Init.Parity       = UART_PARITY_NONE;      //无校验
    UART1_Handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;   //无硬件流控
    UART1_Handle.Init.Mode         = UART_MODE_TX_RX;       //收发模式
    UART1_Handle.Init.OverSampling = UART_OVERSAMPLING_16;  //等价 StdPeriph 默认 OVER8=0
    HAL_UART_Init(&UART1_Handle);                           //初始化串口1

    __HAL_UART_ENABLE_IT(&UART1_Handle, UART_IT_RXNE);      //开启串口接受中断，等价 USART_ITConfig(RXNE, ENABLE)
}

/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void uart1_send(u8 data)
{
    USART1->DR = data;
    while((USART1->SR&0x40)==0);
}


/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
函数功能：串口3初始化
入口参数：无
返回  值：无
**************************************************************************/
void UART3_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_HandleTypeDef UART3_Handle; /* 批次2：HAL 句柄（仅初始化期使用，ISR 为寄存器级不依赖） */

    //对应引脚、对应串口时钟开启
    ENABLE_UART3_TX_PIN_CLOCK;
    ENABLE_UART3_RX_PIN_CLOCK;
    ENABLE_UART3_CLOCK;

    //批次5：AF 经 HAL_GPIO_Init 的 Alternate 字段写入（等价原 GPIO_PinAFConfig；
    //GPIO_AF_USART3=0x07 改用 HAL 宏 GPIO_AF7_USART3 同值）；50MHz→SPEED_FREQ_HIGH。
    //对应引脚端口初始化
    GPIO_InitStructure.Pin       = UART3_TX_PIN;
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pull      = GPIO_PULLUP;
    GPIO_InitStructure.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(UART3_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin       = UART3_RX_PIN;
    GPIO_InitStructure.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(UART3_RX_PORT, &GPIO_InitStructure);

    //UsartNVIC configuration //UsartNVIC配置
    //批次2：原 NVIC_Init(USART3_IRQn, 抢占0, 子0, 分组4) → HAL 等价调用（编码同上说明）。
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    //USART Initialization Settings 初始化设置
    //批次2：原 USART_Init(8N1, TX_RX) + USART_Cmd → HAL_UART_Init 等价。
    UART3_Handle.Instance          = USART3;
    UART3_Handle.Init.BaudRate     = bound;
    UART3_Handle.Init.WordLength   = UART_WORDLENGTH_8B;
    UART3_Handle.Init.StopBits     = UART_STOPBITS_1;
    UART3_Handle.Init.Parity       = UART_PARITY_NONE;
    UART3_Handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    UART3_Handle.Init.Mode         = UART_MODE_TX_RX;
    UART3_Handle.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&UART3_Handle);                  //Initialize serial port 3 //初始化串口3

    __HAL_UART_ENABLE_IT(&UART3_Handle, UART_IT_RXNE); //开启串口接受中断，等价 USART_ITConfig(RXNE, ENABLE)
}

/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口3发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void uart3_send(u8 data)
{
    USART3->DR = data;
    while((USART3->SR&0x40)==0);
}

/**************************************************************************
Function: Serial port 4 initialization
Input   : none
Output  : none
函数功能：串口4初始化
入口参数：无
返回  值：无
**************************************************************************/
void UART4_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_HandleTypeDef UART4_Handle; /* 批次2：HAL 句柄（仅初始化期使用，ISR 为寄存器级不依赖） */

    //对应引脚、对应串口时钟开启
    ENABLE_UART4_TX_PIN_CLOCK;
    ENABLE_UART4_RX_PIN_CLOCK;
    ENABLE_UART4_CLOCK;

    //批次5：AF 经 HAL_GPIO_Init 的 Alternate 字段写入（等价原 GPIO_PinAFConfig；
    //GPIO_AF_UART4=0x08 改用 HAL 宏 GPIO_AF8_UART4 同值）；50MHz→SPEED_FREQ_HIGH。
    //对应引脚端口初始化
    GPIO_InitStructure.Pin       = UART4_TX_PIN;
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pull      = GPIO_PULLUP;
    GPIO_InitStructure.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(UART4_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin       = UART4_RX_PIN;
    GPIO_InitStructure.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(UART4_RX_PORT, &GPIO_InitStructure);

    //UsartNVIC configuration //UsartNVIC配置
    //批次2：原 NVIC_Init(UART4_IRQn, 抢占3, 子0, 分组4) → HAL 等价调用（编码同上说明）。
    HAL_NVIC_SetPriority(UART4_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    //USART Initialization Settings 初始化设置
    //批次2：原 USART_Init(8N1, TX_RX) + USART_Cmd → HAL_UART_Init 等价。
    UART4_Handle.Instance          = UART4;
    UART4_Handle.Init.BaudRate     = bound;
    UART4_Handle.Init.WordLength   = UART_WORDLENGTH_8B;
    UART4_Handle.Init.StopBits     = UART_STOPBITS_1;
    UART4_Handle.Init.Parity       = UART_PARITY_NONE;
    UART4_Handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    UART4_Handle.Init.Mode         = UART_MODE_TX_RX;
    UART4_Handle.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&UART4_Handle);                  //Initialize serial port 4 //初始化串口4

    __HAL_UART_ENABLE_IT(&UART4_Handle, UART_IT_RXNE); //开启串口接受中断，等价 USART_ITConfig(RXNE, ENABLE)
}




/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口4发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void uart4_send(u8 data)
{
    UART4->DR = data;
    while((UART4->SR&0x40)==0);
}
