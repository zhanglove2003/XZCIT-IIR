#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用	  
#endif  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((UART4->SR&0X40)==0);//循环发送,直到发送完毕   
	UART4->DR = (u8) ch;      
	return ch;

//	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
//	USART1->DR = (u8) ch;      
//	return ch;
	
//	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
//	USART3->DR = (u8) ch;      
//	return ch;

}

//任意串口printf
#include "stdarg.h"
uint8_t Ux_TxBuff[256];     //串口x发送缓冲区
void any_printf(USART_TypeDef* USARTx,char *format,...)
{	
	uint8_t i;                                           //用于for循环
	
	va_list listdata;                                     //建立一个va_list变量listdata
	va_start(listdata,format);                            //向listdata加载...代表的不定长的参数
	vsprintf((char *)Ux_TxBuff,format,listdata);          //格式化输出到缓冲区U0_TxBuff
	va_end(listdata);                                     //释放listdata

	for(i=0;i<strlen((const char*)Ux_TxBuff);i++){        //根据U0_TxBuff缓冲区数据量，一个字节一个字节的循环发送
		while((USARTx->SR&0x40)==0);
		USARTx->DR = Ux_TxBuff[i];
			
	}
	while((USARTx->SR&0x40)==0);	     //等到最后一个字节数据发送完毕，再退出函数
}

#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记
/* 批次2：死码 uart_init() 已删除——全树无调用者（grep uart_init 仅本文件定义与
 * usart.h:13 声明），且与 UART1_Init(uartx.c) 重复配置 USART1（但抢占优先级为 3
 * 与 UART1_Init 的 4 相互冲突）；接收 ISR 由 uartx_callback.c 的
 * USART1_IRQHandler 提供。 */
#endif





 



