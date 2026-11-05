#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "steer_debug.h"

static steer_output_t step(steer_debug_t *s, int f, int t, int valid, uint32_t ms)
{
    return steer_debug_step(s, (uint8_t)f, (uint8_t)t, valid, ms);
}
static void eq(steer_output_t o, int l, int r)
{
    assert(o.left == l && o.right == r);
}
static void arm(steer_debug_t *s, uint32_t start)
{
    eq(step(s, 127, 127, 1, start), 0, 0);
    eq(step(s, 127, 127, 1, start + 100), 0, 0);
}
int main(void)
{
    steer_debug_t s = {0};
    eq(step(&s, 255, 127, 1, 0), 0, 0);
    eq(step(&s, 127, 255, 1, 1000), 0, 0);
    eq(step(&s, 127, 127, 1, 1010), 0, 0);
    eq(step(&s, 127, 127, 1, 1109), 0, 0);
    eq(step(&s, 255, 127, 1, 1110), 0, 0);
    arm(&s, 1200);
    eq(step(&s, 255, 127, 1, 1301), 8400, 8400);
    eq(step(&s, 127, 127, 1, 1302), 0, 0);
    eq(step(&s, 0, 127, 1, 1322), -8400, -8400);
    eq(step(&s, 127, 127, 1, 1323), 0, 0);
    eq(step(&s, 127, 255, 1, 1343), 8400, -8400);
    eq(step(&s, 127, 0, 1, 1344), 0, 0);
    eq(step(&s, 127, 0, 1, 1363), 0, 0);
    eq(step(&s, 127, 0, 1, 1364), -8400, 8400);
    eq(step(&s, 127, 127, 1, 1365), 0, 0);
    eq(step(&s, 255, 255, 1, 1385), 8400, 0);
    eq(step(&s, 255, 0, 1, 1386), 0, 8400);
    eq(step(&s, 127, 127, 1, 1387), 0, 0);
    eq(step(&s, 107, 147, 1, 1388), 0, 0);
    eq(step(&s, 255, 127, 1, 1407), 8400, 8400);
    eq(step(&s, 0, 127, 0, 1408), 0, 0);
    eq(step(&s, 255, 127, 1, 1608), 0, 0);
    arm(&s, 1700);
    eq(step(&s, 255, 127, 1, 1801), 8400, 8400);
    /* Right side reverses while left side remains forward. */
    { steer_output_t o = step(&s, 200, 255, 1, 1802); assert(o.left > 0 && o.right == 0); }
    { steer_output_t o = step(&s, 200, 255, 1, 1822); assert(o.left > 0 && o.right < 0); }
    /* The elapsed-time comparisons must survive uint32_t wraparound. */
    { steer_debug_t w = {0}; uint32_t start = UINT32_MAX - 110u;
      arm(&w, start);
      eq(step(&w, 255, 127, 1, UINT32_MAX - 9u), 8400, 8400);
      eq(step(&w, 0, 127, 1, UINT32_MAX - 4u), 0, 0);
      eq(step(&w, 0, 127, 1, 14u), 0, 0);
      eq(step(&w, 0, 127, 1, 15u), -8400, -8400); }
    puts("steering control: PASS");
    return 0;
}
