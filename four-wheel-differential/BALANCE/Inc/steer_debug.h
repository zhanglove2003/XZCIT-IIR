#ifndef C50C_STEER_DEBUG_H
#define C50C_STEER_DEBUG_H
#include <stdint.h>

typedef struct {
    uint32_t stopped_since;
    int8_t direction;
    uint8_t running;
} steer_side_t;

typedef struct {
    uint32_t neutral_since;
    uint8_t armed, neutral_pending;
    steer_side_t left, right;
} steer_debug_t;

typedef struct { int left, right; } steer_output_t;
steer_output_t steer_debug_step(steer_debug_t *s, uint8_t forward_axis,
                                uint8_t turn_axis, int valid, uint32_t now_ms);
#endif
