#ifndef WHEEL_FEEDBACK_H
#define WHEEL_FEEDBACK_H
#include <stdint.h>
/* Net displacement relative to init; sample before a half-counter wrap.
 * A 16-bit timer requires fewer than 32768 counts between samples. */
typedef struct
{
  int64_t counts;
  uint16_t previous;
} wheel_feedback_t;
/* Request a new reference without touching the hardware timer counter. */
extern volatile uint8_t wheel_feedback_zero_request;
void wheel_feedback_init(wheel_feedback_t *wheel, uint16_t counter);
int32_t wheel_feedback_update(wheel_feedback_t *wheel, uint16_t counter);
int64_t wheel_feedback_milliturns(const wheel_feedback_t *wheel, uint32_t cpr);
#endif
