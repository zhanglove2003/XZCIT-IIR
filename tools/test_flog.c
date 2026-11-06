#define FLOG_HOST_TEST 1
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* flog.c 依赖 HAL 的 UART 句柄与 IT 发送。这里给最小桩：
 * Transmit_IT 把数据拷进"线路"，等测试手动调用完成回调。 */
typedef struct {
  uint8_t  buf[4096];
  uint32_t len;
  uint8_t  busy;
  uint8_t  fail_next;   /* 1 = 下一次 Transmit_IT 返回错误 */
} fake_uart_t;

#define HAL_OK   0
#define HAL_ERROR 1

static fake_uart_t s_usart1;
static fake_uart_t s_usart3;
static fake_uart_t s_uart4;

/* 供 flog.c 链接的句柄。名字必须与 flog.h 前向声明的 struct 一致，
 * 否则 flog_tx_done(struct __UART_HandleTypeDef*) 与实际类型不兼容。 */
struct __UART_HandleTypeDef
{
  fake_uart_t *fake;
  int kind;
};
typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

UART_HandleTypeDef huart1 = { &s_usart1, 1 };
UART_HandleTypeDef huart3 = { &s_usart3, 3 };
UART_HandleTypeDef huart4 = { &s_uart4, 4 };

int HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *data, uint16_t size)
{
  if (h->fake->fail_next != 0U)
  {
    h->fake->fail_next = 0U;
    return HAL_ERROR;
  }
  if (h->fake->len + size > sizeof(h->fake->buf))
  {
    return HAL_ERROR;
  }
  if (h->fake->busy != 0U)
  {
    return HAL_ERROR;   /* 复用同一缓冲时重复启动就是 bug */
  }
  memcpy(&h->fake->buf[h->fake->len], data, size);
  h->fake->len += size;
  h->fake->busy = 1U;
  return HAL_OK;
}

#include "../Core/Inc/flog.h"
#include "../Core/Src/flog.c"

/* 模拟一次发送完成中断。真实硬件里完成中断总会到来，
 * 所以只要当前有在途数据就清掉，不依赖测试时序。 */
static void complete(fake_uart_t *f, UART_HandleTypeDef *h)
{
  f->busy = 0U;
  flog_tx_done(h);
}

static uint8_t route_of(const UART_HandleTypeDef *h)
{
  if (h == &huart1) { return FLOG_ROUTE_USART1; }
  if (h == &huart3) { return FLOG_ROUTE_USART3; }
  return FLOG_ROUTE_UART4;
}

/* 把某一路彻底推进：反复 poll + complete 直到队列空。 */
static void drain(fake_uart_t *f, UART_HandleTypeDef *h)
{
  int guard = 0;

  /* 队列空了不代表发完了：最后一笔可能还在"在途"。两者都要等。 */
  while ((flog_pending(route_of(h)) > 0U) ||
         (f->busy != 0U))
  {
    flog_poll();
    complete(f, h);
    guard++;
    if (guard > 2000)
    {
      assert(!"flog: drain did not terminate");
    }
  }
}

static int contains(const fake_uart_t *f, const char *needle)
{
  uint32_t n = (uint32_t)strlen(needle);
  uint32_t i;

  for (i = 0U; i + n <= f->len; i++)
  {
    if (memcmp(&f->buf[i], needle, n) == 0)
    {
      return 1;
    }
  }
  return 0;
}

int main(void)
{
  uint32_t i;

  memset(&s_usart1, 0, sizeof(s_usart1));
  memset(&s_usart3, 0, sizeof(s_usart3));
  memset(&s_uart4, 0, sizeof(s_uart4));
  flog_init();

  /* 1) 一行同时进两路队列，且带 \r\n */
  flog_printf("hello %d", 42);
  assert(flog_pending(FLOG_ROUTE_USART3) == 10U);   /* 8 字符 + CRLF */
  assert(flog_pending(FLOG_ROUTE_UART4) == 10U);

  /* 2) 推进后两路都收到同样内容 */
  drain(&s_usart3, &huart3);
  drain(&s_uart4, &huart4);
  assert(contains(&s_usart3, "hello 42\r\n"));
  assert(contains(&s_uart4, "hello 42\r\n"));
  assert(s_usart3.len == s_uart4.len);

  /* 3) 一路卡住不影响另一路：只推进 USART3，UART4 一直不完成 */
  memset(&s_usart3, 0, sizeof(s_usart3));
  memset(&s_uart4, 0, sizeof(s_uart4));
  flog_printf("only-usart3");
  flog_poll();                       /* 两路都启动发送 */
  complete(&s_usart3, &huart3);      /* 只完成 USART3 */
  assert(contains(&s_usart3, "only-usart3"));
  assert(flog_pending(FLOG_ROUTE_USART3) == 0U);
  /* UART4 的数据已取走、卡在"在途"，队列里是空的——但它没有拖住 USART3 */
  assert(flog_pending(FLOG_ROUTE_UART4) == 0U);
  assert(s_uart4.busy == 1U);

  /* UART4 在途期间再来一行：USART3 照常发完，UART4 排队等 */
  flog_printf("while-stuck");
  for (i = 0U; i < 50U; i++) { flog_poll(); complete(&s_usart3, &huart3); }
  assert(contains(&s_usart3, "while-stuck"));
  assert(flog_pending(FLOG_ROUTE_USART3) == 0U);
  assert(flog_pending(FLOG_ROUTE_UART4) == 13U);   /* 11 字符 + CRLF */
  complete(&s_uart4, &huart4);      /* UART4 终于完成 */
  drain(&s_uart4, &huart4);
  assert(contains(&s_uart4, "while-stuck"));

  /* 4) 一路启动失败（对端没接）只清自己那路，不拖累另一路 */
  memset(&s_usart3, 0, sizeof(s_usart3));
  memset(&s_uart4, 0, sizeof(s_uart4));
  flog_printf("after-fail");
  s_uart4.fail_next = 1U;            /* UART4 下一次启动失败 */
  flog_poll();
  assert(flog_pending(FLOG_ROUTE_UART4) == 0U);      /* 本路被清空 */
  assert(flog_pending(FLOG_ROUTE_USART3) == 0U);     /* USART3 已在途 */
  complete(&s_usart3, &huart3);
  assert(contains(&s_usart3, "after-fail"));         /* 内容照常发出 */
  /* 恢复后 UART4 仍可继续工作 */
  s_uart4.busy = 0U;
  flog_tx_done(&huart4);
  flog_printf("recovered");
  drain(&s_uart4, &huart4);
  assert(contains(&s_uart4, "recovered"));

  /* 5) 长时间不动 poll：缓冲满了只丢新字节，不覆盖旧字节、不越界 */
  memset(&s_usart3, 0, sizeof(s_usart3));
  memset(&s_uart4, 0, sizeof(s_uart4));
  for (i = 0U; i < 500U; i++)
  {
    flog_printf("filler %03u", (unsigned)i);
  }
  assert(flog_pending(FLOG_ROUTE_USART3) <= FLOG_QUEUE_SIZE - 1U);
  drain(&s_usart3, &huart3);
  assert(contains(&s_usart3, "filler 000"));        /* 最早那行还在 */
  assert(!contains(&s_usart3, "filler 499"));       /* 最新的被丢 */

  /* 6) 超长行被截断且不越界 */
  memset(&s_usart3, 0, sizeof(s_usart3));
  memset(&s_uart4, 0, sizeof(s_uart4));
  flog_printf("%0200d", 7);
  drain(&s_usart3, &huart3);
  assert(s_usart3.len <= FLOG_LINE_MAX);
  assert(s_usart3.len > 0U);

  /* 7) 缓冲快满时整行丢弃，不得写出半截行（否则 pwm=12,3 这种会被误读） */
  memset(&s_usart3, 0, sizeof(s_usart3));
  memset(&s_uart4, 0, sizeof(s_uart4));
  {
    /* 先塞满几乎整圈，制造"只剩几字节空间"的局面 */
    for (i = 0U; i < 200U; i++)
    {
      flog_printf("pad-%04u-xxxxxxxxxxxxxxxxxxxxxxxxxx", (unsigned)i);
    }
    drain(&s_usart3, &huart3);
    /* 留下的尾巴里，最后一条必须是完整行（以 CRLF 结尾） */
    assert(s_usart3.len >= 2U);
    assert(s_usart3.buf[s_usart3.len - 2U] == (uint8_t)'\r');
    assert(s_usart3.buf[s_usart3.len - 1U] == (uint8_t)'\n');
  }

  puts("flog: dual route / non-blocking / isolation / full-drop / truncate OK");
  return 0;
}
