#ifndef FLOG_H
#define FLOG_H

#include <stdint.h>

/* 只前向声明：头文件不依赖 usart.h，主机测试也就不用提供整套 HAL UART 类型。
 * 真正的 UART_HandleTypeDef 由 flog.c（真机）或测试桩（主机）给出。 */
struct __UART_HandleTypeDef;

/* 运行期日志：非阻塞环形缓冲 + 多路串口输出。
 *
 * 为什么不用 printf：Ctrl 任务周期是 10 ms，阻塞发一帧 40 字节要 3.5 ms，
 * 会把控制周期拖长，而我们要排查的恰恰是反馈判定——不能用会干扰
 * 被观测对象的手段去观测它。所以这里只把字节塞进 RAM 队列，
 * 发送交给 HAL_UART_Transmit_IT 和中断推进。
 *
 * 两路同时输出，因为不知道板子上的 Type-C 口接到哪个 UART：
 *   - USART3 (PB10/PB11, 115200)  约 0.4 ms / 40 字节
 *   - UART4  (PC10/PC11,  9600)   约 4 ms / 40 字节
 * 两路队列各自独立，任何一路发不动（对端没接/没开）都只是丢自己那路的字节，
 * 绝不会阻塞或拖动另一路，也不会影响控制循环。
 *
 * 关掉：Makefile 里去掉 -DFLOG_ENABLE 即可，编译后不占任何代码体和 RAM。 */

#define FLOG_LINE_MAX   120U
#define FLOG_QUEUE_SIZE 1024U

void flog_init(void);
/* 每轮调用：把队列里攒的字节推给各串口。必须在 Ctrl 循环里高频调用。 */
void flog_poll(void);
/* 写一行（自动补 \r\n）。超长会被截断，不会越界。 */
void flog_printf(const char *fmt, ...);
/* 某一路上还有多少字节待发（测试用）。 */
uint32_t flog_pending(uint8_t route);
/* HAL_UART_TxCpltCallback 里调用：清掉该路的"在途"标记。
 * 必须声明在头文件里——它由 stm32f4xx_it.c 的中断上下文调用。 */
void flog_tx_done(struct __UART_HandleTypeDef *huart);

#define FLOG_ROUTE_USART1 0U   /* PA9(TX)/PA10(RX) 串口1-一键下载口 */
#define FLOG_ROUTE_USART3 1U   /* PB10(TX)/PB11(RX) 串口3-Type-C口 */
#define FLOG_ROUTE_UART4  2U   /* PC10(TX)/PC11(RX) 蓝牙口 */
#define FLOG_ROUTE_COUNT  3U

#endif
