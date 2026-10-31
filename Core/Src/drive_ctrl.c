#include "drive_ctrl.h"

#define DRIVE_CTRL_CENTER 128

volatile uint8_t drive_calibration_mode = 0U;

int32_t drive_ctrl_map_axis(uint8_t raw, int32_t limit)
{
  int32_t delta = (int32_t)raw - DRIVE_CTRL_CENTER;
  int32_t over;
  int32_t pulse;

  if (delta < 0)
  {
    delta = -delta;
  }

  if (delta <= DRIVE_CTRL_DEADZONE)
  {
    return 0;
  }

  over = delta - DRIVE_CTRL_DEADZONE;

  /* (128-16)=112 档线性铺满到 limit：112*8400=940800 无 int32 溢出 */
  pulse = over * limit / (DRIVE_CTRL_CENTER - DRIVE_CTRL_DEADZONE);

  if (pulse > limit)
  {
    pulse = limit;
  }

  /* raw < 128（推杆向上/前进、摇杆向左/左转方向）为正 */
  return (raw < DRIVE_CTRL_CENTER) ? pulse : -pulse;
}

int32_t drive_ctrl_map(uint8_t ly)
{
  return drive_ctrl_map_axis(ly, (int32_t)DRIVE_CTRL_MAX_PULSE);
}

void drive_ctrl_mix(uint8_t ly, uint8_t rx, int32_t out[4])
{
  int32_t v = drive_ctrl_map_axis(ly, (int32_t)DRIVE_CTRL_MAX_PULSE);
  int32_t t = drive_ctrl_map_axis(rx, (int32_t)DRIVE_CTRL_TURN_MAX_PULSE);
  int32_t wheel[2]; /* [0]=A/C 右侧 = v+t，[1]=B/D 左侧 = v-t */
  int32_t peak = 0;
  int i;

  wheel[0] = v + t;
  wheel[1] = v - t;

  for (i = 0; i < 2; i++)
  {
    int32_t mag = (wheel[i] < 0) ? -wheel[i] : wheel[i];

    if (mag > peak)
    {
      peak = mag;
    }
  }

  if (peak > (int32_t)DRIVE_CTRL_MAX_PULSE)
  {
    /* 等比缩放回限幅：峰值 12600*8400 < 2^31 无溢出 */
    out[0] = out[2] = wheel[0] * (int32_t)DRIVE_CTRL_MAX_PULSE / peak;
    out[1] = out[3] = wheel[1] * (int32_t)DRIVE_CTRL_MAX_PULSE / peak;
  }
  else
  {
    out[0] = out[2] = wheel[0];
    out[1] = out[3] = wheel[1];
  }
}
