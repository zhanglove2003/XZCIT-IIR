#include <assert.h>
#include <stdio.h>

#include "boot_ui.h"

int main(void)
{
  assert(BOOT_UI_VDD_MIN_MV == 3100U);

  assert(boot_ui_rpm_from_counts(34886, 209314U) == 100);
  assert(boot_ui_rpm_from_counts(-34886, 209314U) == -100);
  assert(boot_ui_rpm_from_counts(104657, 209314U) == 300);
  assert(boot_ui_rpm_from_counts(0, 209314U) == 0);
  assert(boot_ui_rpm_from_counts(12345, 0U) == 0);

  assert(boot_ui_vdd_mv(1490U, 1490U) == 3300U);
  assert(boot_ui_vdd_mv(1600U, 1490U) == 3073U);
  assert(boot_ui_vdd_mv(1400U, 1490U) == 3512U);
  assert(boot_ui_vdd_mv(0U, 1490U) == 0U);
  assert(boot_ui_vdd_mv(1500U, 0U) == 3278U);

  assert(boot_ui_rpm_from_elapsed(209314, 209314U, 1200U) == 50);
  assert(boot_ui_rpm_from_elapsed(-209314, 209314U, 1200U) == -50);
  assert(boot_ui_rpm_from_elapsed(209314, 209314U, 1000U) == 60);
  assert(boot_ui_rpm_from_elapsed(0, 209314U, 120U) == 0);
  assert(boot_ui_rpm_from_elapsed(209314, 0U, 100U) == 0);
  assert(boot_ui_rpm_from_elapsed(209314, 209314U, 0U) == 0);
  puts("boot math OK");
  return 0;
}
