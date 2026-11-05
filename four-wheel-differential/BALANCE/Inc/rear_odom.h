#ifndef C50C_REAR_ODOM_H
#define C50C_REAR_ODOM_H

#include <stdint.h>
#include <limits.h>

/* Provisional value: measured at about 102k counts per rear-wheel revolution. */
#define REAR_ENCODER_COUNTS_PER_REV 102000.0f

typedef struct {
    uint32_t left_count, right_count, time_ms;
    uint8_t initialized;
} rear_odom_t;

typedef struct {
    int16_t vx_mm_s, vy_mm_s, wz_mrad_s;
    int16_t left_mm_s, right_mm_s;
    uint8_t valid;
} rear_odom_result_t;

typedef struct {
    int16_t left_mm_s, right_mm_s;
    uint32_t sample_ms;
    uint8_t valid;
} rear_odom_snapshot_t;

extern volatile rear_odom_snapshot_t g_rear_odom_snapshot;

static inline int32_t rear_odom_delta(uint32_t current, uint32_t previous)
{
    uint32_t diff = current - previous;
    if (diff <= INT32_MAX) return (int32_t)diff;
    return (int32_t)(-(int64_t)(UINT32_MAX - diff + 1u));
}

static inline int16_t rear_odom_saturate(float value)
{
    if (value >= 32767.0f) return INT16_MAX;
    if (value <= -32768.0f) return INT16_MIN;
    return (int16_t)value;
}

static inline rear_odom_result_t rear_odom_step(rear_odom_t *s,
        uint32_t raw_b, uint32_t raw_c, uint32_t now_ms,
        float wheel_circ_m, float half_wheel_spacing_m)
{
    rear_odom_result_t out = {0};
    uint32_t dt = now_ms - s->time_ms;
    int32_t left_delta, right_raw_delta;
    float left_m_s, right_m_s;

    if (!s->initialized || dt > 500u ||
        !(wheel_circ_m > 0.0f && wheel_circ_m < 10.0f) ||
        !(half_wheel_spacing_m > 0.0f && half_wheel_spacing_m < 10.0f)) {
        s->left_count = raw_b;
        s->right_count = raw_c;
        s->time_ms = now_ms;
        s->initialized = 1;
        return out;
    }
    if (dt == 0u) return out;

    left_delta = rear_odom_delta(raw_b, s->left_count);
    right_raw_delta = rear_odom_delta(raw_c, s->right_count);
    s->left_count = raw_b;
    s->right_count = raw_c;
    s->time_ms = now_ms;

    left_m_s = (float)left_delta * wheel_circ_m * 1000.0f /
               (REAR_ENCODER_COUNTS_PER_REV * (float)dt);
    right_m_s = -(float)right_raw_delta * wheel_circ_m * 1000.0f /
                (REAR_ENCODER_COUNTS_PER_REV * (float)dt);
    out.left_mm_s = rear_odom_saturate(left_m_s * 1000.0f);
    out.right_mm_s = rear_odom_saturate(right_m_s * 1000.0f);
    out.vx_mm_s = rear_odom_saturate((left_m_s + right_m_s) * 500.0f);
    out.wz_mrad_s = rear_odom_saturate((right_m_s - left_m_s) * 1000.0f /
                                       (2.0f * half_wheel_spacing_m));
    out.valid = 1;
    return out;
}

#endif
