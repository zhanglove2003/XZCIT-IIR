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
# Firmware sources pulling vendor headers need this chain:
INC_VENDOR    = -I$(ROOT)/Core/Inc -I$(ROOT)/Drivers/BSP/encoder -I$(ROOT)/Drivers/BSP/sys \
                -I$(ROOT)/Drivers/BSP/usart -I$(ROOT)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
                -I$(ROOT)/Drivers/CMSIS/Include -I$(ROOT)/Drivers/STM32F4xx_StdPeriph_Driver/inc \
                -DSTM32F407xx -DSTM32F40_41xxx -D__FPU_PRESENT=1 -DARM_MATH_CM4
STDPERIPH_SRC = $(ROOT)/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
                $(ROOT)/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c \
                $(ROOT)/Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c

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
$(BUILD)/test_encoder_diag: test_encoder_diag.c $(ROOT)/Drivers/BSP/encoder/encoder.c | $(BUILD)
	$(CC) $(CSTD) -Wall -Werror -Wno-error -w $(INC_VENDOR) -include stm32f4xx_conf.h \
	  test_encoder_diag.c $(ROOT)/Drivers/BSP/encoder/encoder.c $(STDPERIPH_SRC) -lm -o $@ && ./$@

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
