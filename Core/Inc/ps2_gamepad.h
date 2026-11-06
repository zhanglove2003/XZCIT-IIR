#ifndef PS2_GAMEPAD_H
#define PS2_GAMEPAD_H

#include <stdint.h>

/* 手柄状态：buttons 为原始键位（1=未按），摇杆 0..255，中位 128 */
typedef struct
{
  uint16_t buttons;
  uint8_t lx;
  uint8_t ly;
  uint8_t rx;
  uint8_t ry;
} ps2_state_t;

/* 一次完整读取是否成功（模拟模式 0x73/0x79，收到 0x5A 帧头） */
#define PS2_FRAME_OK 1U
#define PS2_FRAME_BAD 0U

/* Completed-transaction diagnostics, readable over SWD without stopping PWM. */
typedef struct
{
  uint32_t polls, good, bad, configs;
  uint32_t poll_us, max_poll_us, max_gap_ms;
  uint32_t mode, marker;
  uint8_t frame[9];
} ps2_diag_t;
extern volatile ps2_diag_t ps2_diag;

/* Decode the nine bytes returned by a poll; leave output unchanged on error. */
static inline uint8_t ps2_decode_frame(const uint8_t data[9], ps2_state_t *out)
{
  if (out == 0 || data[2] != 0x5AU ||
      (data[1] != 0x73U && data[1] != 0x79U))
  {
    return PS2_FRAME_BAD;
  }
  out->buttons = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
  out->rx = data[5];
  out->ry = data[6];
  out->lx = data[7];
  out->ly = data[8];
  return PS2_FRAME_OK;
}

/* 初始化 PE0-PE3 与 DWT 微秒延时 */
void ps2_init(void);

/* 进入并锁定模拟模式（短轮询 ×3 → 进配置 → 模拟模式 → 退出） */
void ps2_config(void);

/* 读取一帧；返回 PS2_FRAME_OK 时 *out 有效 */
uint8_t ps2_poll(ps2_state_t *out);

#endif
