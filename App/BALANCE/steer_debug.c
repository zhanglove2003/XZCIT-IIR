#include "steer_debug.h"

/* Commissioning only: open-loop PWM, not measured wheel velocity. */
#define DEADZONE 20
#define MAX_DUTY 13440 /* 80% of the 16800-count PWM period (raised from 50% on 2026-11-13). */
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

#if defined(ROS_STRAIGHT_DRIVE)
/* Runtime ROS PWM duty limit, in PWM counts. Starts at the compile time value
   and is changed at run time by CMD_PWM_LIMIT on USART3. Every clamp and every
   rescale below reads this variable, so the limits stay consistent.
   Range enforced by the command handler: 1..ROS_DRIVE_MAX_DUTY_HARD. */
int g_ros_drive_max_duty = ROS_DRIVE_MAX_DUTY;

/* ROS entry. The caller builds PWM counts from the speed command:
   duty = speed[m/s] * 1000. Each input is clamped to +-g_ros_drive_max_duty here,
   and the sum on one side is clamped the same way. This is an open loop
   conversion, not a calibrated wheel speed. */
steer_output_t steer_debug_step_duty(steer_debug_t *s, int forward_duty,
                                     int turn_duty, int neutral, int valid,
                                     uint32_t now_ms)
{
    int left, right, max, magnitude;
    int limit = g_ros_drive_max_duty;
    steer_output_t out = {0, 0};

    if (limit < 1) limit = 1;
    if (limit > ROS_DRIVE_MAX_DUTY_HARD) limit = ROS_DRIVE_MAX_DUTY_HARD;

    if (forward_duty > limit) forward_duty = limit;
    if (forward_duty < -limit) forward_duty = -limit;
    if (turn_duty > limit) turn_duty = limit;
    if (turn_duty < -limit) turn_duty = -limit;

    if (!valid) {
        (void)side_step(&s->left, 0, now_ms);
        (void)side_step(&s->right, 0, now_ms);
        s->armed = 0;
        s->neutral_pending = 0;
        return out;
    }
    if (!s->armed) {
        /* Rearm on a neutral COMMAND, never on a zero duty. The closed loop
           holds a non-zero duty while the car is moving, so a duty based test
           would block rearming after any single invalid step. */
        if (neutral) {
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

    left = forward_duty + turn_duty;
    right = forward_duty - turn_duty;
    max = left < 0 ? -left : left;
    magnitude = right < 0 ? -right : right;
    if (magnitude > max) max = magnitude;
    if (max > limit) {
        left = left * limit / max;
        right = right * limit / max;
    }
    out.left = side_step(&s->left, left, now_ms);
    out.right = side_step(&s->right, right, now_ms);
    return out;
}
#endif
