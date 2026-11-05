#include <assert.h>
#include <stdint.h>
#include "encoder_diag.h"

int main(void)
{
    EncoderDiag d = {0};
    encoder_diag_record(&d, 5, 5, 0);
    encoder_diag_record(&d, -3, -3, 0);
    encoder_diag_record(&d, -130, 2, 1);
    assert(d.raw_sum == (uint32_t)-128);
    assert(d.raw_abs == 138u);
    assert(d.filtered_sum == 4u);
    assert(d.samples == 3u);
    assert(d.rejected == 1u);
    d.raw_sum = UINT32_MAX;
    encoder_diag_record(&d, 1, 1, 0);
    assert(d.raw_sum == 0u);
    encoder_diag_record(&d, -32768, 0, 1);
    assert(d.raw_abs == 32907u);
    encoder_diag_record(&d, INT32_MIN, 0, 1);
    assert(d.raw_abs == 32907u + UINT32_C(2147483648));
    return 0;
}
