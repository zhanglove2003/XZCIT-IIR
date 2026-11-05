#ifndef C50C_ENCODER_DIAG_H
#define C50C_ENCODER_DIAG_H

#include <stdint.h>

typedef struct {
    uint32_t raw_sum;
    uint32_t raw_abs;
    uint32_t filtered_sum;
    uint32_t samples;
    uint32_t rejected;
} EncoderDiag;

/* Sums use defined modulo-2^32 arithmetic so a host can subtract snapshots. */
static inline void encoder_diag_record(volatile EncoderDiag *d, int32_t raw,
                                       int32_t filtered, int rejected)
{
    d->raw_sum += (uint32_t)raw;
    d->raw_abs += raw < 0 ? 0u - (uint32_t)raw : (uint32_t)raw;
    d->filtered_sum += (uint32_t)filtered;
    d->samples++;
    if (rejected) d->rejected++;
}

extern volatile EncoderDiag g_encoder_diag[4];

#endif
