#include "flog.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>

/* 每路一个独立环形缓冲：一路发不出去不能拖住另一路。 */
typedef struct
{
  uint8_t  buf[FLOG_QUEUE_SIZE];
  volatile uint32_t head;      /* 生产者（flog_printf）写位置 */
  volatile uint32_t tail;      /* 已启动发送的字节之后的位置（可回收） */
  volatile uint8_t  tx_busy;   /* HAL_UART_Transmit_IT 是否在途 */
  UART_HandleTypeDef *uart;
} flog_route_t;

static flog_route_t s_routes[FLOG_ROUTE_COUNT];
/* 两路共用同一份格式化结果：日志内容必须完全一致，也省一次 vsnprintf。 */
static uint8_t s_line[FLOG_LINE_MAX];
static uint8_t s_ready;

void flog_init(void)
{
  uint8_t r;

  for (r = 0U; r < FLOG_ROUTE_COUNT; r++)
  {
    s_routes[r].head = 0U;
    s_routes[r].tail = 0U;
    s_routes[r].tx_busy = 0U;
  }
  s_routes[FLOG_ROUTE_USART1].uart = &huart1;
  s_routes[FLOG_ROUTE_USART3].uart = &huart3;
  s_routes[FLOG_ROUTE_UART4].uart = &huart4;
  s_ready = 1U;
}

static uint32_t flog_used(const flog_route_t *rt)
{
  uint32_t head = rt->head;
  uint32_t tail = rt->tail;

  return (head >= tail) ? (head - tail)
                        : (FLOG_QUEUE_SIZE - tail + head);
}

static void flog_push(flog_route_t *rt, const uint8_t *data, uint32_t len)
{
  uint32_t i;
  uint32_t free_bytes;

  if (len >= FLOG_QUEUE_SIZE)
  {
    return;
  }

  /* 剩余可写字节（尾巴留一格区分满/空）。 */
  free_bytes = (rt->tail + FLOG_QUEUE_SIZE - rt->head - 1U) % FLOG_QUEUE_SIZE;
  free_bytes += (free_bytes == 0U) ? FLOG_QUEUE_SIZE - 1U : 0U;

  /* 放不下整行就整行丢掉，不写半截：
   * 截断的行比没有这行更误导人（例如 pwm=12,3 看起来像合法值）。 */
  if (len > free_bytes)
  {
    return;
  }

  for (i = 0U; i < len; i++)
  {
    rt->buf[rt->head] = data[i];
    rt->head = (rt->head + 1U) % FLOG_QUEUE_SIZE;
  }
}

void flog_poll(void)
{
  uint8_t r;

  if (s_ready == 0U)
  {
    return;
  }

  for (r = 0U; r < FLOG_ROUTE_COUNT; r++)
  {
    flog_route_t *rt = &s_routes[r];
    uint32_t len;

    /* 上一笔还没发完：本路暂停取新数据。
     * HAL 发送读的是这块内存，回绕后覆盖会发出乱码。 */
    if ((rt->tx_busy != 0U) || (rt->uart == NULL))
    {
      continue;
    }

    len = flog_used(rt);
    if (len == 0U)
    {
      continue;
    }

    /* 一次最多推一整块连续区，跨环尾就截断到这里，剩的下次再发。 */
    if (rt->tail + len > FLOG_QUEUE_SIZE)
    {
      len = FLOG_QUEUE_SIZE - rt->tail;
    }

    rt->tx_busy = 1U;
    if (HAL_UART_Transmit_IT(rt->uart, &rt->buf[rt->tail],
                             (uint16_t)len) != HAL_OK)
    {
      /* 启动失败（大多是对端没接、外设没开）：清空本路队列，不要卡死。
       * 另一路的队列不受影响。 */
      rt->tx_busy = 0U;
      rt->tail = 0U;
      rt->head = 0U;
      continue;
    }

    /* 数据已交给 HAL，立刻推进 tail：这段空间不能再被 flog_push 写，
     * 但"这一段正在发"由 tx_busy 保证——busy 期间本路不会再取新数据。 */
    rt->tail = (rt->tail + len) % FLOG_QUEUE_SIZE;
  }
}

/* HAL 在发送完成回调里调到这里。只清在途标记，tail 已在启动时推进。 */
void flog_tx_done(struct __UART_HandleTypeDef *huart)
{
  uint8_t r;

  if (s_ready == 0U)
  {
    return;
  }

  for (r = 0U; r < FLOG_ROUTE_COUNT; r++)
  {
    if (s_routes[r].uart == huart)
    {
      s_routes[r].tx_busy = 0U;
      return;
    }
  }
}

void flog_printf(const char *fmt, ...)
{
  uint8_t r;
  va_list ap;
  int n;
  uint32_t total;

  if (s_ready == 0U)
  {
    return;
  }

  va_start(ap, fmt);
  /* 预留 2 字节给 \r\n：容量给 FLOG_LINE_MAX-2，vsnprintf 最多写这么多（含结束符），
   * 所以正文最多 FLOG_LINE_MAX-3 字节，不会和后面补的 CRLF 打架。 */
  n = vsnprintf((char *)s_line, FLOG_LINE_MAX - 2U, fmt, ap);
  va_end(ap);

  if (n < 0)
  {
    return;
  }
  if ((uint32_t)n > (FLOG_LINE_MAX - 3U))
  {
    n = (int)(FLOG_LINE_MAX - 3U);
  }

  s_line[n] = (uint8_t)'\r';
  s_line[n + 1] = (uint8_t)'\n';
  total = (uint32_t)n + 2U;

  for (r = 0U; r < FLOG_ROUTE_COUNT; r++)
  {
    flog_push(&s_routes[r], s_line, total);
  }
}

uint32_t flog_pending(uint8_t route)
{
  if (route >= FLOG_ROUTE_COUNT)
  {
    return 0U;
  }
  return flog_used(&s_routes[route]);
}
