#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT
common=(-std=c11 -Wall -Wextra -Werror -ICore/Inc)
gcc "${common[@]}" -Wno-unused-parameter -Itools/host_stubs tools/test_ctrl_task.c Core/Src/freertos.c Core/Src/drive_ctrl.c Core/Src/wheel_home.c Core/Src/wheel_feedback.c Core/Src/motor_health.c Core/Src/buzzer.c -o "$test_dir/ctrl"
"$test_dir/ctrl" homing
"$test_dir/ctrl" fault
"$test_dir/ctrl" s
"$test_dir/ctrl"
gcc "${common[@]}" tools/test_ps2_frame.c -o "$test_dir/ps2"
"$test_dir/ps2"
gcc "${common[@]}" -ffunction-sections -fdata-sections -Itools/host_stubs tools/test_selfcheck.c Core/Src/drive_ctrl.c Core/Src/motor_test.c Core/Src/oled_gfx.c Core/Src/oled_font.c -Wl,--gc-sections -o "$test_dir/selfcheck"
"$test_dir/selfcheck"
gcc "${common[@]}" tools/test_oled_gfx.c Core/Src/oled_gfx.c Core/Src/oled_font.c -o "$test_dir/oledgfx"
"$test_dir/oledgfx"
gcc "${common[@]}" tools/test_drive_ctrl.c Core/Src/drive_ctrl.c -o "$test_dir/map"
"$test_dir/map"
gcc "${common[@]}" tools/test_boot_math.c -o "$test_dir/math"
"$test_dir/math"
gcc "${common[@]}" -fsanitize=address,undefined -ffunction-sections -fdata-sections -Itools/host_stubs tools/test_ui_sampling.c Core/Src/drive_ctrl.c Core/Src/motor_test.c Core/Src/wheel_feedback.c Core/Src/wheel_home.c Core/Src/oled_gfx.c Core/Src/oled_font.c -Wl,--gc-sections -o "$test_dir/ui"
"$test_dir/ui"
gcc "${common[@]}" tools/test_wheel_feedback.c Core/Src/wheel_feedback.c -o "$test_dir/feedback"
"$test_dir/feedback"
gcc "${common[@]}" -fsanitize=address,undefined -ICore/Inc tools/test_wheel_home.c Core/Src/wheel_home.c Core/Src/wheel_feedback.c -o "$test_dir/home"
"$test_dir/home"
gcc "${common[@]}" -fsanitize=address,undefined -ICore/Inc tools/test_motor_test.c -o "$test_dir/motortest"
"$test_dir/motortest"
gcc "${common[@]}" -fsanitize=address,undefined -ICore/Inc tools/test_manual_ref.c -o "$test_dir/manualref"
gcc "${common[@]}" -fsanitize=address,undefined -ICore/Inc tools/test_ui_model.c Core/Src/ui_model.c -o "$test_dir/uimodel"
gcc "${common[@]}" -fsanitize=address,undefined -ICore/Inc -Itools/host_stubs tools/test_buzzer.c -o "$test_dir/buzzer"
"$test_dir/manualref"
"$test_dir/uimodel"
"$test_dir/buzzer"
gcc "${common[@]}" -fsanitize=address,undefined -ICore/Inc tools/test_motor_health.c Core/Src/motor_health.c -o "$test_dir/mhealth"
"$test_dir/mhealth"
