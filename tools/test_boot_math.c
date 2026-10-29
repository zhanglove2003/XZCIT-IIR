#include <assert.h>
#include <stdio.h>

#include "boot_ui.h"

int main(void)
{
  assert(WHEEL_CPR == 54000U);
  assert(BOOT_UI_VDD_MIN_MV == 3100U);

  assert(boot_ui_rpm_from_counts(9000, WHEEL_CPR) == 100);
  assert(boot_ui_rpm_from_counts(-9000, WHEEL_CPR) == -100);
  assert(boot_ui_rpm_from_counts(27000, WHEEL_CPR) == 300);
  assert(boot_ui_rpm_from_counts(0, WHEEL_CPR) == 0);
  assert(boot_ui_rpm_from_counts(12345, 0U) == 0);

  assert(boot_ui_vdd_mv(1490U, 1490U) == 3300U);
  assert(boot_ui_vdd_mv(1600U, 1490U) == 3073U);
  assert(boot_ui_vdd_mv(1400U, 1490U) == 3512U);
  assert(boot_ui_vdd_mv(0U, 1490U) == 0U);
  assert(boot_ui_vdd_mv(1500U, 0U) == 3278U);

  puts("boot math OK");
  return 0;
}
