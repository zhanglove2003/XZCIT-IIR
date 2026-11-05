#include "steer_debug.h"

/* Commissioning only: open-loop PWM, not measured wheel velocity. */
#define DEADZONE 20
#define MAX_DUTY 8400 /* 50% of the 16800-count PWM period. */
#define ARM_NEUTRAL_MS 100u
#define REVERSE_ZERO_MS 20u

static int axis_duty(uint8_t axis)
{
    int delta = (int)axis - 127;
    int magnitude = delta < 0 ? -delta : delta;
    int duty;
    if (magnitude <= DEADZONE) return 0;
    if (magnitude > 127) magnitude = 127;
    duty = (magnitude - DEADZONE) * MAX_DUTY / (127 - DEADZONE);
    return delta < 0 ? -duty : duty;
}

static int side_step(steer_side_t *side, int desired, uint32_t now_ms)
{
    int direction = desired < 0 ? -1 : 1;
    if (desired == 0) {
        if (side->running) side->stopped_since = now_ms;
        side->running = 0;
        return 0;
    }
    if (side->direction && direction != side->direction) {
        if (side->running) {
            side->stopped_since = now_ms;
            side->running = 0;
        }
        if ((uint32_t)(now_ms - side->stopped_since) < REVERSE_ZERO_MS)
            return 0;
    }
    side->direction = (int8_t)direction;
    side->running = 1;
    return desired;
}

steer_output_t steer_debug_step(steer_debug_t *s, uint8_t forward_axis,
                                uint8_t turn_axis, int valid, uint32_t now_ms)
{
    steer_output_t out = {0, 0};
    int forward = axis_duty(forward_axis);
    int turn = axis_duty(turn_axis);
    int left, right, max, magnitude;

    if (!valid) {
        (void)side_step(&s->left, 0, now_ms);
        (void)side_step(&s->right, 0, now_ms);
        s->armed = 0;
        s->neutral_pending = 0;
        return out;
    }
    if (!s->armed) {
        if (forward == 0 && turn == 0) {
            if (!s->neutral_pending) {
                s->neutral_since = now_ms;
                s->neutral_pending = 1;
            }
            if ((uint32_t)(now_ms - s->neutral_since) >= ARM_NEUTRAL_MS)
                s->armed = 1;
        } else {
            s->neutral_pending = 0;
        }
        return out;
    }

    /* Right stick right: left wheels forward, right wheels backward. */
    left = forward + turn;
    right = forward - turn;
    max = left < 0 ? -left : left;
    magnitude = right < 0 ? -right : right;
    if (magnitude > max) max = magnitude;
    if (max > MAX_DUTY) {
        left = left * MAX_DUTY / max;
        right = right * MAX_DUTY / max;
    }
    out.left = side_step(&s->left, left, now_ms);
    out.right = side_step(&s->right, right, now_ms);
    return out;
}
