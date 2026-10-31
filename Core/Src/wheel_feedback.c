#include "wheel_feedback.h"
volatile uint8_t wheel_feedback_zero_request = 0U;
void wheel_feedback_init(wheel_feedback_t *wheel, uint16_t counter)
{
  wheel->counts = 0;
  wheel->previous = counter;
}
int32_t wheel_feedback_update(wheel_feedback_t *wheel, uint16_t counter)
{
  int32_t delta = (int32_t)counter - (int32_t)wheel->previous;
  if (delta > 32767) { delta -= 65536; }
  else if (delta < -32768) { delta += 65536; }
  wheel->counts += delta;
  wheel->previous = counter;
  return delta;
}
int64_t wheel_feedback_milliturns(const wheel_feedback_t *wheel, uint32_t cpr)
{
  if (cpr == 0U) { return 0; }
  return (wheel->counts / (int64_t)cpr) * 1000 +
         ((wheel->counts % (int64_t)cpr) * 1000) / (int64_t)cpr;
}
