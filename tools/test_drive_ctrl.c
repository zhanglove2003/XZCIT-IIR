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

  /* 后退：推杆向下 ly > 128 → 负占空比；spec §8：两端对称归一化，
   * 下行按有效行程 111 格归一（48*8400/111=3632、96*8400/111=7264、满杆 -8400） */
  assert(drive_ctrl_map(192) == -3632);
  assert(drive_ctrl_map(240) == -7264);
  assert(drive_ctrl_map(255) == -8400);  /* spec §8：两端对称归一化 */

  /* ===== Task 1: 双轴映射与混合 ===== */

  /* 轴映射：死区/端点对称/极限值。
   * spec §8：正负两侧分别按有效行程归一化（上行 112 格、下行 111 格），
   * 两端对称到达 ±limit。 */
  assert(drive_ctrl_map_axis(128, 8400) == 0);
  assert(drive_ctrl_map_axis(144, 8400) == 0);   /* 死区 16 内 */
  assert(drive_ctrl_map_axis(0,   8400) ==  8400);
  assert(drive_ctrl_map_axis(255, 8400) == -8400);
  assert(drive_ctrl_map_axis(0,   4200) ==  4200);
  assert(drive_ctrl_map_axis(255, 4200) == -4200);

  /* 混合：out 下标 0..3 = A/B/C/D；T=+ 为左转（rx<128），A/C 右轮=+T，B/D 左轮=-T */
  {
    int i;
    int32_t m[4];

    drive_ctrl_mix(128, 128, m);              /* 全零 */
    for (i = 0; i < 4; i++) assert(m[i] == 0);

    drive_ctrl_mix(0, 128, m);                /* 满前进 → 四轮 +8400 */
    for (i = 0; i < 4; i++) assert(m[i] == 8400);

    drive_ctrl_mix(255, 128, m);              /* 满后退 → 四轮 -8400 */
    for (i = 0; i < 4; i++) assert(m[i] == -8400);

    drive_ctrl_mix(128, 0, m);                /* 摇杆左=左转(T=+4200)：A/C=+4200, B/D=-4200 */
    assert(m[0] == 4200 && m[2] == 4200 && m[1] == -4200 && m[3] == -4200);

    drive_ctrl_mix(128, 255, m);              /* 右转(T=-4200)：A/C=-4200, B/D=+4200 */
    assert(m[0] == -4200 && m[2] == -4200 && m[1] == 4200 && m[3] == 4200);

    drive_ctrl_mix(0, 255, m);                /* 前进+右转：peak=12600 缩放 → A=C=2800, B=D=8400 */
    assert(m[0] == 2800 && m[2] == 2800 && m[1] == 8400 && m[3] == 8400);

    drive_ctrl_mix(255, 0, m);                /* 后退+左转：peak=12600 缩放 → A=C=-2800, B=D=-8400 */
    assert(m[0] == -2800 && m[2] == -2800 && m[1] == -8400 && m[3] == -8400);
  }

  puts("drive ctrl OK");
  return 0;
}
