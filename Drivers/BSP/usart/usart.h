#ifndef __USART_H
#define __USART_H
#include "stdio.h"
/* 批次6：stm32f4xx_conf.h 随 StdPeriph 退场删除，此处换 stm32f4xx_hal.h——
 * 本头只用 USART_TypeDef（stm32f407xx.h shim 已提供），HAL 头保持与 uartx.c
 * 批次2 同款显式 include 习惯；usart.c 为 printf 重定向文件，不碰外设 API。 */
#include "stm32f4xx_hal.h"
#include "sys.h"

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
void any_printf(USART_TypeDef* USARTx,char *format,...);
#endif


