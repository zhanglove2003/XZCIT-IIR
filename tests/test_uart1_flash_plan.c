/* Host-side C port of the pure-logic part of tests/test_uart1_flash_plan.py:
   a faithful C reimplementation of tools/c50c_uart1_flash.py prepare_image()
   (BASE/FLASH_SIZE/PARAM_OFFSET/SECTOR_ENDS/validate_image/prepare_image read
   from tools/c50c_uart1_flash.py:9-47). The subprocess/workflow tests
   (invoke(), main() with fake stm32flash) depend on spawning the real host
   tool and pyserial-free process mocking; they are out of scope for a pure C
   port and are NOT rewritten (declared in the run notes). */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define BASE 0x08000000u
#define FLASH_SIZE 0x80000u
#define PARAM_OFFSET 0x60000u
static const uint32_t SECTOR_ENDS[] = {0x4000, 0x8000, 0xC000, 0x10000, 0x20000, 0x40000, 0x60000};
#define N_SECTORS 7

/* returns 0 ok, nonzero = ValueError equivalent */
static int validate_image(const uint8_t *image, size_t len)
{
    if (len < 8 || len > PARAM_OFFSET) return 1;
    uint32_t sp = (uint32_t)image[0] | ((uint32_t)image[1] << 8) |
                  ((uint32_t)image[2] << 16) | ((uint32_t)image[3] << 24);
    uint32_t reset = (uint32_t)image[4] | ((uint32_t)image[5] << 8) |
                     ((uint32_t)image[6] << 16) | ((uint32_t)image[7] << 24);
    if (!(sp > 0x20000000u && sp <= 0x20020000u && sp % 4 == 0)) return 2;
    if (!(reset & 1u && BASE <= (reset & ~1u) && (reset & ~1u) < BASE + len)) return 3;
    return 0;
}

/* returns 0 ok; on success fills sectors[] (count), payload (malloc, caller frees), payload_len */
static int prepare_image(const uint8_t *image, size_t image_len,
                         const uint8_t *backup, size_t backup_len,
                         int *sector_count, uint8_t **payload, size_t *payload_len)
{
    int rc = validate_image(image, image_len);
    if (rc) return rc;
    if (backup_len != FLASH_SIZE) return 10;
    size_t end = 0;
    for (int i = 0; i < N_SECTORS; i++) {
        if (SECTOR_ENDS[i] >= image_len) { end = SECTOR_ENDS[i]; *sector_count = i + 1; break; }
    }
    *payload_len = end;
    *payload = malloc(end);
    assert(*payload);
    memcpy(*payload, image, image_len);
    memcpy(*payload + image_len, backup + image_len, end - image_len);
    return 0;
}

static void fill_image(uint8_t *b, size_t len)
{
    memset(b, 0x5a, len);
    b[0] = 0x00; b[1] = 0x20; b[2] = 0x00; b[3] = 0x20; /* sp = 0x20020000 LE */
    b[4] = 0x01; b[5] = 0x01; b[6] = 0x00; b[7] = 0x08; /* reset = 0x08000101 LE */
}

int main(void)
{
    /* Preserves bytes outside the image and inside sector 7. */
    uint8_t *backup = malloc(FLASH_SIZE);
    for (size_t i = 0; i < FLASH_SIZE; i++) backup[i] = (uint8_t)(i % 251);
    size_t img_len = 88596;
    uint8_t *img = malloc(img_len);
    fill_image(img, img_len);
    int sectors = -1; uint8_t *payload = NULL; size_t payload_len = 0;
    assert(prepare_image(img, img_len, backup, FLASH_SIZE, &sectors, &payload, &payload_len) == 0);
    assert(sectors == 5); /* [0,1,2,3,4] */
    assert(payload_len == 0x20000);
    assert(memcmp(payload, img, img_len) == 0);
    assert(memcmp(payload + img_len, backup + img_len, 0x20000 - img_len) == 0);
    free(payload);
    puts("flash plan preserves bytes outside image and inside sector 7: PASS");

    /* Rejects parameter-sector overlap. */
    uint8_t *big = malloc(0x60001);
    fill_image(big, 0x60001);
    assert(prepare_image(big, 0x60001, backup, FLASH_SIZE, &sectors, &payload, &payload_len) == 1);
    free(big);
    puts("flash plan rejects parameter sector overlap: PASS");

    /* Rejects invalid vectors: all-zero image. */
    uint8_t zero[1024]; memset(zero, 0, sizeof(zero));
    assert(prepare_image(zero, sizeof(zero), backup, FLASH_SIZE, &sectors, &payload, &payload_len) != 0);
    puts("flash plan rejects invalid vectors: PASS");

    /* Rejects short backup. */
    uint8_t small[1024]; fill_image(small, sizeof(small));
    assert(prepare_image(small, sizeof(small), backup, FLASH_SIZE - 1,
                         &sectors, &payload, &payload_len) == 10);
    puts("flash plan rejects short backup: PASS");

    free(img); free(backup);
    return 0;
}
