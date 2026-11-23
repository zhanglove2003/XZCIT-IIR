# Host-side unit/integration tests. Each target builds and runs one test.
# All paths are relative to the repo root (st32f407/), matching the current
# tree: App/BALANCE, App/BALANCE/Inc, Drivers/BSP.
#
# Vendor (STM32/CMSIS) headers are not warning-clean on the host, so tests that
# compile firmware sources including them use -w for the vendor part; the test
# files themselves stay -Wall -Werror clean.

CC       ?= cc
CSTD     = -std=c99
BUILD    = build/tests
ROOT     = ..

# Current-tree include chains
INC_APP       = -I$(ROOT)/App/BALANCE
INC_APP_INC   = -I$(ROOT)/App/BALANCE/Inc
INC_ENC       = -I$(ROOT)/Drivers/BSP/encoder
# 批次3：原 INC_VENDOR/STDPERIPH_SRC 链（host 编 encoder.c+StdPeriph 源）已随
# encoder.c 迁移到 HAL_TIM_Encoder_* 而废弃——固件 HAL 头链无法在 host 编译，
# 且 test_encoder_diag 只测 encoder_diag.h 的 encoder_diag_record 纯逻辑，
# 从未使用 encoder.c 的符号（改前该链接即历史遗留）。

TESTS = test_c50c_logic \
        test_encoder_diag \
        test_four_turn_integration \
        test_front_single_integration \
        test_rear_debug \
        test_rear_integration \
        test_rear_odom \
        test_rear_oled \
        test_rear_oled_integration \
        test_steer_debug \
        test_uart1_flash_plan \
        test_uart3_monitor

# test_front_single_integration builds four variants (one per channel).
FRONT_CHS = 3 4 5 6

all: $(addprefix $(BUILD)/,$(TESTS))

$(BUILD):
	mkdir -p $(BUILD)

# --- integration tests embedding firmware snippets (clean -Wall -Werror) ---
$(BUILD)/test_rear_integration: test_rear_integration.c $(ROOT)/App/BALANCE/rear_debug.c | $(BUILD)
	$(CC) $(CSTD) -Wall -Wextra -Werror $(INC_APP) test_rear_integration.c $(ROOT)/App/BALANCE/rear_debug.c -lm -o $@ && ./$@

$(BUILD)/test_four_turn_integration: test_four_turn_integration.c $(ROOT)/App/BALANCE/steer_debug.c | $(BUILD)
	$(CC) $(CSTD) -Wall -Wextra -Werror $(INC_APP) test_four_turn_integration.c $(ROOT)/App/BALANCE/steer_debug.c -lm -o $@ && ./$@

$(BUILD)/test_front_single_integration: test_front_single_integration.c $(ROOT)/App/BALANCE/rear_debug.c | $(BUILD)
	@set -e; for ch in $(FRONT_CHS); do \
	  $(CC) $(CSTD) -Wall -Wextra -Werror -DFRONT_TEST_CHANNEL=$$ch $(INC_APP) \
	    test_front_single_integration.c $(ROOT)/App/BALANCE/rear_debug.c -lm -o $(BUILD)/front_ch$$ch; \
	  ./$(BUILD)/front_ch$$ch; \
	done; touch $@

$(BUILD)/test_rear_oled_integration: test_rear_oled_integration.c | $(BUILD)
	$(CC) $(CSTD) -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all $(INC_APP_INC) test_rear_oled_integration.c -lm -o $@ && ./$@

# --- c50c logic (firmware bodies extracted verbatim; needs Inc + encoder diag header) ---
$(BUILD)/test_c50c_logic: test_c50c_logic.c usart1_body.inc $(ROOT)/App/BALANCE/balance_task.c | $(BUILD)
	$(CC) $(CSTD) -Wall -Wextra -Werror -Itests $(INC_APP_INC) $(INC_ENC) test_c50c_logic.c -lm -o $@ && ./$@

# --- module unit tests ---
$(BUILD)/test_encoder_diag: test_encoder_diag.c $(ROOT)/Drivers/BSP/encoder/encoder_diag.h | $(BUILD)
	$(CC) $(CSTD) -Wall -Wextra -Werror $(INC_ENC) test_encoder_diag.c -lm -o $@ && ./$@

$(BUILD)/test_rear_debug: test_rear_debug.c $(ROOT)/App/BALANCE/rear_debug.c | $(BUILD)
	$(CC) $(CSTD) -Wall -Werror $(INC_APP) test_rear_debug.c $(ROOT)/App/BALANCE/rear_debug.c -lm -o $@ && ./$@

$(BUILD)/test_rear_odom: test_rear_odom.c | $(BUILD)
	$(CC) $(CSTD) -Wall -Werror $(INC_APP_INC) test_rear_odom.c -lm -o $@ && ./$@

$(BUILD)/test_rear_oled: test_rear_oled.c | $(BUILD)
	$(CC) $(CSTD) -Wall -Werror $(INC_APP_INC) test_rear_oled.c -lm -o $@ && ./$@

$(BUILD)/test_steer_debug: test_steer_debug.c $(ROOT)/App/BALANCE/steer_debug.c | $(BUILD)
	$(CC) $(CSTD) -Wall -Werror $(INC_APP) test_steer_debug.c $(ROOT)/App/BALANCE/steer_debug.c -lm -o $@ && ./$@

# --- host tool pure-logic ports ---
$(BUILD)/test_uart1_flash_plan: test_uart1_flash_plan.c | $(BUILD)
	$(CC) $(CSTD) -Wall -Wextra -Werror test_uart1_flash_plan.c -lm -o $@ && ./$@

$(BUILD)/test_uart3_monitor: test_uart3_monitor.c | $(BUILD)
	$(CC) $(CSTD) -Wall -Wextra -Werror test_uart3_monitor.c -lm -o $@ && ./$@

clean:
	rm -rf $(BUILD)

.PHONY: all clean
