/* Host-side C port of tests/test_uart3_monitor.py: a faithful C reimplementation
   of the StatusFrameParser in tools/c50c_uart3_monitor.py:15-46 (24-byte frames,
   0x7B marker, XOR checksum over bytes 0..21, 0x7D tail, big-endian signed
   16-bit fields, bounded resynchronization). Pure logic, no serial I/O. */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define FRAME_LEN 24

typedef struct {
    uint8_t buffer[512];   /* bounded, like the py bytearray */
    size_t len;
} Parser;

typedef struct {
    uint8_t stop;
    int16_t velocity_mm_s[2];
    int16_t yaw_mrad_s;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    uint16_t voltage_mv;
} StatusItem;

static void parser_init(Parser *p) { p->len = 0; }

static int16_t be16s(const uint8_t *b)
{
    int16_t v = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    return v;
}

/* feed(): appends data, returns number of parsed items (max 8 per call) */
static int parser_feed(Parser *p, const uint8_t *data, size_t n, StatusItem *out)
{
    int count = 0;
    /* py: buffer.extend(data) with the py test feeding at most ~10k bytes;
       our bounded buffer simply refuses to grow past its capacity, which the
       10000-byte noise test never exceeds because noise is drained. */
    for (size_t i = 0; i < n && p->len < sizeof(p->buffer); i++) p->buffer[p->len++] = data[i];
    for (;;) {
        if (p->len == 0) break;
        /* find 0x7b */
        size_t start = p->len;
        for (size_t i = 0; i < p->len; i++) {
            if (p->buffer[i] == 0x7b) { start = i; break; }
        }
        if (start == p->len) { p->len = 0; break; }  /* no marker: clear */
        if (start > 0) { memmove(p->buffer, p->buffer + start, p->len - start); p->len -= start; }
        if (p->len < FRAME_LEN) break;
        uint8_t *frame = p->buffer;
        uint8_t checksum = 0;
        for (int i = 0; i < 22; i++) checksum ^= frame[i];
        if (frame[23] != 0x7d || frame[22] != checksum) {
            memmove(p->buffer, p->buffer + 1, p->len - 1);
            p->len--;
            continue;
        }
        StatusItem *it = &out[count++];
        it->stop = frame[1];
        it->velocity_mm_s[0] = be16s(frame + 2);
        it->velocity_mm_s[1] = be16s(frame + 4);
        it->yaw_mrad_s = be16s(frame + 6);
        it->accel_raw[0] = be16s(frame + 8);
        it->accel_raw[1] = be16s(frame + 10);
        it->accel_raw[2] = be16s(frame + 12);
        it->gyro_raw[0] = be16s(frame + 14);
        it->gyro_raw[1] = be16s(frame + 16);
        it->gyro_raw[2] = be16s(frame + 18);
        it->voltage_mv = (uint16_t)(((uint16_t)frame[20] << 8) | frame[21]);
        memmove(p->buffer, p->buffer + FRAME_LEN, p->len - FRAME_LEN);
        p->len -= FRAME_LEN;
    }
    return count;
}

static size_t build_frame(uint8_t *out)
{
    static const int16_t vals[10] = {123, -45, 67, 1, -2, 3, 4, -5, 6, 12345};
    size_t n = 0;
    out[n++] = 0x7B;
    out[n++] = 0;
    for (int i = 0; i < 10; i++) {
        uint16_t u = (uint16_t)vals[i];
        out[n++] = (uint8_t)(u >> 8);
        out[n++] = (uint8_t)(u & 0xff);
    }
    uint8_t checksum = 0;
    for (size_t i = 0; i < n; i++) checksum ^= out[i];
    out[n++] = checksum;
    out[n++] = 0x7D;
    return n;
}

int main(void)
{
    uint8_t frame[FRAME_LEN];
    size_t flen = build_frame(frame);
    assert(flen == FRAME_LEN);

    /* Stream split, corruption and resynchronization. */
    Parser p; parser_init(&p);
    StatusItem items[8];
    uint8_t broken[FRAME_LEN];
    memcpy(broken, frame, FRAME_LEN);
    broken[8] ^= 0x20;

    uint8_t stream[512];
    size_t n = 0;
    memcpy(stream + n, "noise", 5); n += 5;
    memcpy(stream + n, broken, FRAME_LEN); n += FRAME_LEN;
    memcpy(stream + n, frame, 9); n += 9;
    assert(parser_feed(&p, stream, n, items) == 0);

    n = 0;
    memcpy(stream + n, frame + 9, FRAME_LEN - 9); n += FRAME_LEN - 9;
    memcpy(stream + n, frame, FRAME_LEN); n += FRAME_LEN;
    int got = parser_feed(&p, stream, n, items);
    assert(got == 2);
    assert(items[0].velocity_mm_s[0] == 123);
    assert(items[0].velocity_mm_s[1] == -45);
    assert(items[0].yaw_mrad_s == 67);
    assert(items[0].accel_raw[0] == 1 && items[0].accel_raw[1] == -2 && items[0].accel_raw[2] == 3);
    assert(items[0].gyro_raw[0] == 4 && items[0].gyro_raw[1] == -5 && items[0].gyro_raw[2] == 6);
    assert(items[0].voltage_mv == 12345);
    puts("uart3 frame split, corruption and resynchronization: PASS");

    /* Unfinished noise is bounded. */
    Parser q; parser_init(&q);
    uint8_t noise[256];
    memset(noise, 0x7b, sizeof(noise));
    for (int i = 0; i < 10000 / 256; i++) {
        StatusItem dummy[8];
        parser_feed(&q, noise, sizeof(noise), dummy);
    }
    assert(q.len <= 23);
    puts("uart3 unfinished noise is bounded: PASS");
    return 0;
}
