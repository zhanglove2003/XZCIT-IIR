#include <assert.h>
#include <stdint.h>
#include "rear_debug.h"
int main(void) {
    rear_debug_t s = {0};
    assert(rear_debug_step(&s, 127, 1, 0) == 0);
    assert(rear_debug_step(&s, 127, 1, 99) == 0);
    assert(rear_debug_step(&s, 127, 1, 100) == 0);
    assert(rear_debug_step(&s, 255, 1, 110) == 1500);
    assert(rear_debug_step(&s, 0, 1, 120) == 0);
    assert(rear_debug_step(&s, 0, 1, 139) == 0);
    assert(rear_debug_step(&s, 0, 1, 140) == -1500);
    assert(rear_debug_step(&s, 0, 0, 141) == 0);
    assert(rear_debug_step(&s, 0, 1, 300) == 0);
    assert(rear_debug_step(&s, 127, 1, 301) == 0);
    assert(rear_debug_step(&s, 127, 1, 401) == 0);
    assert(rear_debug_step(&s, 0, 1, 402) == -1500);
    assert(rear_debug_step(&s, 127, 1, 403) == 0);
    /* A held stick cannot arm on startup or after signal loss. */
    rear_debug_t fresh = {0};
    assert(rear_debug_step(&fresh, 255, 1, 0) == 0);
    assert(rear_debug_step(&fresh, 255, 1, 1000) == 0);
    assert(rear_debug_step(&fresh, 127, 1, 1001) == 0);
    assert(rear_debug_step(&fresh, 127, 1, 1101) == 0);
    assert(rear_debug_step(&fresh, 255, 1, 1102) == 1500);
    assert(rear_debug_step(&fresh, 255, 0, 1103) == 0);
    assert(rear_debug_step(&fresh, 255, 1, 1500) == 0);
    /* A direction change remains blanked even across tick wrap. */
    rear_debug_t wrap = { .direction = 1, .armed = 1, .running = 1 };
    assert(rear_debug_step(&wrap, 0, 1, UINT32_MAX - 9) == 0);
    assert(rear_debug_step(&wrap, 0, 1, 5) == 0);
    assert(rear_debug_step(&wrap, 0, 1, 10) == -1500);
    return 0;
}
