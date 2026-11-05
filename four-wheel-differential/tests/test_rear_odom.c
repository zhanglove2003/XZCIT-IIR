#include <assert.h>
#include <stdint.h>
#include <math.h>
#include "rear_odom.h"

int main(void)
{
    rear_odom_t s = {0};
    rear_odom_result_t r = rear_odom_step(&s, 0, 0, 1000, 0.477f, 0.23f);
    assert(!r.valid && r.vx_mm_s == 0);

    /* Left input B counts forward positive; right input C counts negative. */
    r = rear_odom_step(&s, 10200, (uint32_t)-10200, 1100, 0.477f, 0.23f);
    assert(r.valid && r.vx_mm_s == 477 && r.vy_mm_s == 0 && r.wz_mrad_s == 0);
    assert(r.left_mm_s == 477 && r.right_mm_s == 477);
    r = rear_odom_step(&s, 0, 0, 1200, 0.477f, 0.23f);
    assert(r.valid && r.vx_mm_s == -477 && r.wz_mrad_s == 0);
    assert(r.left_mm_s == -477 && r.right_mm_s == -477);

    /* 4WD configuration stores the half track: 0.23 m means 0.46 m full track. */
    r = rear_odom_step(&s, 0, (uint32_t)-10200, 1300, 0.477f, 0.23f);
    assert(r.valid && r.vx_mm_s >= 238 && r.vx_mm_s <= 239);
    assert(r.wz_mrad_s >= 1036 && r.wz_mrad_s <= 1038);
    assert(r.left_mm_s == 0 && r.right_mm_s == 477);

    /* Unsigned counter and clock wrap must preserve a small signed delta. */
    rear_odom_t wrap = {0};
    rear_odom_step(&wrap, UINT32_MAX - 5099u, 5099u, UINT32_MAX - 49u, 0.477f, 0.23f);
    r = rear_odom_step(&wrap, 5100u, (uint32_t)-5101, 50u, 0.477f, 0.23f);
    assert(r.valid && r.vx_mm_s == 477 && r.wz_mrad_s == 0);

    /* A delayed update is a fresh baseline, never an exaggerated speed. */
    r = rear_odom_step(&s, 100000, 0, 2000, 0.477f, 0.23f);
    assert(!r.valid && r.vx_mm_s == 0);
    r = rear_odom_step(&s, 100000, 0, 2100, 0.477f, 0.23f);
    assert(r.valid && r.vx_mm_s == 0);
    r = rear_odom_step(&s, 100001, 0, 2100, 0.477f, 0.23f);
    assert(!r.valid);
    r = rear_odom_step(&s, 100001, 0, 2200, 0.477f, 0.23f);
    assert(r.valid && r.vx_mm_s == 0);
    r = rear_odom_step(&s, 100001, 0, 2300, INFINITY, 0.23f);
    assert(!r.valid);
    r = rear_odom_step(&s, 100001, 0, 2400, 0.477f, NAN);
    assert(!r.valid);
    assert(rear_odom_saturate(40000.0f) == INT16_MAX);
    assert(rear_odom_saturate(-40000.0f) == INT16_MIN);
    return 0;
}
