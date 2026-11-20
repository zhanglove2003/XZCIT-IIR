#include "rear_debug.h"

/* Forward/reverse commissioning only. No speed feedback is used. */
#define NEUTRAL_DEADZONE 20
#define MAX_DUTY 1500
#define ARM_NEUTRAL_MS 100u
#define REVERSE_ZERO_MS 20u

int rear_debug_step(rear_debug_t *s, uint8_t axis, int valid, uint32_t now_ms)
{
    int delta = (int)axis - 127;
    int magnitude = delta < 0 ? -delta : delta;
    int direction = delta < 0 ? -1 : 1;
    int duty;

    if (!valid) {
        if (s->running) s->stopped_since = now_ms;
        s->running = 0;
        s->armed = 0;
        s->neutral_pending = 0;
        return 0;
    }
    if (!s->armed) {
        if (magnitude <= NEUTRAL_DEADZONE) {
            if (!s->neutral_pending) {
                s->neutral_since = now_ms;
                s->neutral_pending = 1;
            }
            if ((uint32_t)(now_ms - s->neutral_since) >= ARM_NEUTRAL_MS)
                s->armed = 1;
        } else {
            s->neutral_pending = 0;
        }
        return 0;
    }
    if (magnitude <= NEUTRAL_DEADZONE) {
        if (s->running) s->stopped_since = now_ms;
        s->running = 0;
        return 0;
    }
    if (s->direction && direction != s->direction) {
        if (s->running) {
            s->stopped_since = now_ms;
            s->running = 0;
        }
        if ((uint32_t)(now_ms - s->stopped_since) < REVERSE_ZERO_MS)
            return 0;
    }
    duty = (magnitude - NEUTRAL_DEADZONE) * MAX_DUTY / (127 - NEUTRAL_DEADZONE);
    if (duty > MAX_DUTY) duty = MAX_DUTY;
    s->direction = (int8_t)direction;
    s->running = 1;
    return direction * duty;
}
