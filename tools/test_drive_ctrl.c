#include <assert.h>
#include <stdio.h>

#include "drive_ctrl.h"

int main(void)
{
  assert(DRIVE_CTRL_MAX_PULSE == 8400);
  assert(DRIVE_CTRL_DEADZONE == 16);

  /* 死区：中心 128 ± 16 内全部为 0 */
  assert(drive_ctrl_map(128) == 0);
  assert(drive_ctrl_map(112) == 0);
  assert(drive_ctrl_map(144) == 0);
  assert(drive_ctrl_map(120) == 0);
  assert(drive_ctrl_map(136) == 0);

  /* 前进：推杆向上 ly < 128 → 正占空比，比例系数 8400/112 = 75/单位 */
  assert(drive_ctrl_map(64) == 3600);
  assert(drive_ctrl_map(16) == 7200);
  assert(drive_ctrl_map(0) == 8400);

  /* 后退：推杆向下 ly > 128 → 负占空比，与前进对称；
   * uint8 端点不对称（向下最大偏差 127），满杆后退 = 111*75 = 8325 */
  assert(drive_ctrl_map(192) == -3600);
  assert(drive_ctrl_map(240) == -7200);
  assert(drive_ctrl_map(255) == -8325);

  /* ===== Task 1: 双轴映射与混合 ===== */

  /* 轴映射：死区/端点/极限值。
   * 注：uint8 端点不对称——向上满杆 raw=0 偏差 128 → over=112 → +limit；
   * 向下满杆 raw=255 偏差仅 127 → over=111 → -111*limit/112（与上方
   * drive_ctrl_map(255)==-8325 的既有结论一致）。 */
  assert(drive_ctrl_map_axis(128, 8400) == 0);
  assert(drive_ctrl_map_axis(144, 8400) == 0);   /* 死区 16 内 */
  assert(drive_ctrl_map_axis(0,   8400) ==  8400);
  assert(drive_ctrl_map_axis(255, 8400) == -8325);
  assert(drive_ctrl_map_axis(0,   4200) ==  4200);
  assert(drive_ctrl_map_axis(255, 4200) == -4162);  /* 111*4200/112 = 4162.5 截断 */

  /* 混合：out 下标 0..3 = A/B/C/D；T=+ 为左转（rx<128），A/C 右轮=+T，B/D 左轮=-T */
  {
    int i;
    int32_t m[4];

    drive_ctrl_mix(128, 128, m);              /* 全零 */
    for (i = 0; i < 4; i++) assert(m[i] == 0);

    drive_ctrl_mix(0, 128, m);                /* 满前进 → 四轮 +8400 */
    for (i = 0; i < 4; i++) assert(m[i] == 8400);

    drive_ctrl_mix(255, 128, m);              /* 满后退 → 四轮 -8325 */
    for (i = 0; i < 4; i++) assert(m[i] == -8325);

    drive_ctrl_mix(128, 0, m);                /* 摇杆左=左转(T=+4200)：A/C=+4200, B/D=-4200 */
    assert(m[0] == 4200 && m[2] == 4200 && m[1] == -4200 && m[3] == -4200);

    drive_ctrl_mix(128, 255, m);              /* 右转(T=-4162)：A/C=-4162, B/D=+4162 */
    assert(m[0] == -4162 && m[2] == -4162 && m[1] == 4162 && m[3] == 4162);

    drive_ctrl_mix(0, 255, m);                /* 前进+右转：peak=12562 缩放 → A=C=2833, B=D=8400 */
    assert(m[0] == 2833 && m[2] == 2833 && m[1] == 8400 && m[3] == 8400);

    drive_ctrl_mix(255, 0, m);                /* 后退+左转：peak=12525 缩放 → A=C=-2766, B=D=-8400 */
    assert(m[0] == -2766 && m[2] == -2766 && m[1] == -8400 && m[3] == -8400);
  }

  puts("drive ctrl OK");
  return 0;
}
