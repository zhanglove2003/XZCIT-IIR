#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT
# ucontext changes stacks. Use UBSan; ASan does not fully support this API.
gcc -std=c11 -Wall -Wextra -Werror -Wno-unused-parameter \
  -fsanitize=undefined -ffunction-sections -fdata-sections \
  -ICore/Inc -Itools/host_stubs \
  tools/test_boot_ctrl_integration.c Core/Src/freertos.c \
  Core/Src/boot_ui.c Core/Src/ui_render.c \
  Core/Src/drive_ctrl.c Core/Src/ui_model.c Core/Src/wheel_home.c \
  Core/Src/wheel_feedback.c Core/Src/selfcheck_service.c \
  Core/Src/power_monitor.c \
  Core/Src/manual_ref.c Core/Src/motor_test.c Core/Src/motor_health.c \
  Core/Src/buzzer.c \
  Core/Src/oled_gfx.c Core/Src/oled_font.c -Wl,--gc-sections \
  -o "$test_dir/boot_ctrl"
failed=0
for scenario in 0 1 2 3 4 5 6; do
  if ! "$test_dir/boot_ctrl" "$scenario"; then failed=1; fi
done
exit "$failed"
