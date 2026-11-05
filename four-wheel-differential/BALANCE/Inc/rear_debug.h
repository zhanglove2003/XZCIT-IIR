#ifndef C50C_REAR_DEBUG_H
#define C50C_REAR_DEBUG_H
#include <stdint.h>
typedef struct { uint32_t neutral_since, stopped_since; int8_t direction; uint8_t armed, neutral_pending, running; } rear_debug_t;
int rear_debug_step(rear_debug_t *s, uint8_t axis, int valid, uint32_t now_ms);
#endif
