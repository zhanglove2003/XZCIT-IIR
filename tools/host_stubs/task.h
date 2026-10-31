/* The host controls task scheduling via osDelay. */
/* Host: no ISR to mask, critical sections are no-ops. */
#define taskENTER_CRITICAL()
#define taskEXIT_CRITICAL()
