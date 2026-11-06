#include "drive_ctrl.h"

#define DRIVE_CTRL_CENTER 128
/* spec §8：正负两侧分别按有效行程归一化，两端对称到达 ±limit。
 * 上行（raw<128）有效行程 128-16=112 格；下行（raw>128）为 255-128-16=111 格。 */
#define DRIVE_CTRL_AXIS_SPAN_UP 112
#define DRIVE_CTRL_AXIS_SPAN_DOWN 111

volatile uint8_t drive_calibration_mode = 0U;

/* 运行时驾驶 PWM 输出上限：默认 50% = 8400；设置页可调至
 * [10%, 90%] = [1680, 15120]（setter 绝对夹取 [840, 15120]）。
 * 只在本任务（Ctrl）读写，无跨任务竞争。 */
int32_t drive_ctrl_max_pulse = (int32_t)DRIVE_CTRL_MAX_PULSE;

void drive_ctrl_set_max_pulse(int32_t v)
{
  if (v < DRIVE_CTRL_MIN_PULSE)
  {
    v = DRIVE_CTRL_MIN_PULSE;
  }
  if (v > (int32_t)DRIVE_CTRL_ABS_MAX_PULSE)
  {
    v = (int32_t)DRIVE_CTRL_ABS_MAX_PULSE;
  }
  drive_ctrl_max_pulse = v;
}

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

  /* 两侧各自线性铺满到 limit：over≤span 时 112*8400=940800 无 int32 溢出 */
  pulse = over * limit / ((raw < DRIVE_CTRL_CENTER) ? DRIVE_CTRL_AXIS_SPAN_UP
                                                    : DRIVE_CTRL_AXIS_SPAN_DOWN);

  if (pulse > limit)
  {
    pulse = limit;
  }

  /* raw < 128（推杆向上/前进、摇杆向左/左转方向）为正 */
  return (raw < DRIVE_CTRL_CENTER) ? pulse : -pulse;
}

int32_t drive_ctrl_map(uint8_t ly)
{
  return drive_ctrl_map_axis(ly, drive_ctrl_max_pulse);
}

void drive_ctrl_mix(uint8_t ly, uint8_t rx, int32_t out[4])
{
  /* 每拍重读运行时上限：上限变更瞬间正在输出的驾驶命令下一拍即按
   * 新上限重映射（mix 每拍整体重算，无残留）。转向分量恒 = 上限/2。 */
  int32_t v = drive_ctrl_map_axis(ly, drive_ctrl_max_pulse);
  int32_t t = drive_ctrl_map_axis(rx, drive_ctrl_max_pulse / 2);
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

  if (peak > drive_ctrl_max_pulse)
  {
    /* 等比缩放回限幅：峰值 22680*15120 < 2^31 无溢出（90% 档满前进+满转向） */
    out[0] = out[2] = wheel[0] * drive_ctrl_max_pulse / peak;
    out[1] = out[3] = wheel[1] * drive_ctrl_max_pulse / peak;
  }
  else
  {
    out[0] = out[2] = wheel[0];
    out[1] = out[3] = wheel[1];
  }
}
