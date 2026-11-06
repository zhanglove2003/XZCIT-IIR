#include <stdint.h>
typedef void *osThreadId_t;
typedef int osPriority_t;
#define osPriorityNormal 0
#define osPriorityBelowNormal 2
#define osPriorityAboveNormal 1
struct osThreadAttr_t { const char *name; uint32_t stack_size; osPriority_t priority; };
typedef struct osThreadAttr_t osThreadAttr_t;
void *osThreadNew(void (*fn)(void *), void *, const osThreadAttr_t *);
void osDelay(uint32_t);
typedef void *xTaskHandle;
/* The host controls task scheduling via osDelay. */
/* Host: no ISR to mask, critical sections are no-ops. */
#define taskENTER_CRITICAL()
#define taskEXIT_CRITICAL()
