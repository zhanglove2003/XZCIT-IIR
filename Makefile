# WHEELTEC C50C Diff_Car 的 GNU Make 构建
# 目标芯片：STM32F407VET6（FLASH 512K / SRAM 128K）
# 与原 Keil 工程 Diff_Car Target 等价：宏 DIFF_CAR，RVDS 端口已改为 GCC 内联汇编版本。

TARGET   = Diff_Car
BUILD    = build
PREFIX   = arm-none-eabi-

CC       = $(PREFIX)gcc
AS       = $(PREFIX)gcc -x assembler-with-cpp
CP       = $(PREFIX)objcopy
SZ       = $(PREFIX)size

CPU      = -mcpu=cortex-m4
FPU      = -mfpu=fpv4-sp-d16
FLOAT-ABI= -mfloat-abi=hard
MCU      = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

DEFS     = -DUSE_STDPERIPH_DRIVER -D__FPU_PRESENT=1 -D__TARGET_FPU_VFP \
           -DARM_MATH_CM4 -DUSE_USB_OTG_FS -D_4WD_CAR -DSTM32F40_41xxx -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -D__align="__attribute__((aligned(4)))" -D__inline="inline" -D__forceinline="inline __attribute__((always_inline))" -D__nop="__NOP"
# 四轮 ROS 直行（task-15）：这 4 个宏是 balance_task.c 的 #error 守卫要求的最小充分集合。
# REAR_WHEEL_DEBUG 打开四轮调试输出路径；ROS_STRAIGHT_DRIVE 打开 ros_cmd_tick 与
# steer_debug_step_duty；FOUR_TURN_DEBUG 打开 PS2 四轮转向分支（不加则走引用
# g_rear_debug 的旧分支，编译失败）；FRONT_TEST_CHANNEL=6 是 FOUR_TURN_DEBUG 的前置条件。
DEFS     += -DREAR_WHEEL_DEBUG -DROS_STRAIGHT_DRIVE -DFOUR_TURN_DEBUG -DFRONT_TEST_CHANNEL=6


C_INCLUDES = -IDrivers/CMSIS/Include -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
             -IDrivers/BSP/delay \
             -IDrivers/BSP/sys \
             -IDrivers/BSP/usart \
             -ICore/Src -ICore/Inc \
             -IDrivers/STM32F4xx_StdPeriph_Driver/inc \
             -IMiddlewares/Third_Party/FreeRTOS/include \
             -IMiddlewares/Third_Party/FreeRTOS/portable/RVDS/ARM_CM4F \
             -IApp/CarType/Inc \
             -IApp/BALANCE/Inc \
             -IDrivers/BSP/Inc \
             -IDrivers/BSP/MPU6050 \
             -IDrivers/BSP/ICM20948 \
             -IMiddlewares/STM32_USB_Host_Library/Class/HID/Inc \
             -IMiddlewares/STM32_USB_Host_Library/Core/Inc \
             -IMiddlewares/USB_Host/App \
             -IMiddlewares/USB_Host/Target

OPT      = -O2 -g3
CFLAGS   = $(MCU) $(DEFS) $(C_INCLUDES) $(OPT) -Wall \
           -ffunction-sections -fdata-sections -fno-common \
           -std=gnu99 -Wno-unused-parameter -Wno-unused-variable \
           -Wno-unused-but-set-variable -Wno-missing-field-initializers \
           -Wno-parentheses -Wno-pointer-sign -Wno-implicit-fallthrough \
           -Wno-address-of-packed-member

# 架空测试开关. 默认关闭. 命令行给 PWM_TEST_DEFS=-DCHASSIS_PWM_TEST 才编进测试代码.
CFLAGS  += $(PWM_TEST_DEFS)

# ROS 直行驱动开关. 默认关闭, 关闭时不生成 ROS 专用代码.
#   ROS_STRAIGHT_DRIVE=1   编译 ROS 速度帧驱动四轮的路径
#   ROS_DRIVE_MAX_DUTY=N   ROS 路径 PWM 占空比上限. 默认 8400. 10% 架空用 1680.
# 两个开关都只追加预处理器定义, 不改默认构建.
ifneq ($(ROS_STRAIGHT_DRIVE),)
ROS_DRIVE_DEFS = -DROS_STRAIGHT_DRIVE
endif
ifneq ($(ROS_DRIVE_MAX_DUTY),)
ROS_DRIVE_DEFS += -DROS_DRIVE_MAX_DUTY=$(ROS_DRIVE_MAX_DUTY)
endif
CFLAGS  += $(ROS_DRIVE_DEFS)

ASFLAGS  = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections
LDSCRIPT = WHEELTEC_C50C.ld
LDFLAGS  = $(MCU) -T$(LDSCRIPT) -specs=nano.specs -specs=nosys.specs \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map,--cref \
           -Wl,--print-memory-usage -lc -lm -lnosys

C_SOURCES = \
  Drivers/STM32F4xx_StdPeriph_Driver/src/misc.c \
  Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
  Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
  Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c \
  Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c \
  Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
  Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c \
  Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_can.c \
  Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c \
  Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c \
  Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
  Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hal_hcd.c \
  Drivers/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_ll_usb.c \
  Middlewares/Third_Party/FreeRTOS/croutine.c \
  Middlewares/Third_Party/FreeRTOS/event_groups.c \
  Middlewares/Third_Party/FreeRTOS/list.c \
  Middlewares/Third_Party/FreeRTOS/queue.c \
  Middlewares/Third_Party/FreeRTOS/tasks.c \
  Middlewares/Third_Party/FreeRTOS/timers.c \
  Middlewares/Third_Party/FreeRTOS/portable/RVDS/ARM_CM4F/port.c \
  Middlewares/Third_Party/FreeRTOS/portable/MemMang/heap_4.c \
  Core/Src/main.c \
  Core/Src/system.c \
  Core/Src/stm32f4xx_it.c \
  Core/Src/system_stm32f4xx.c \
  Drivers/BSP/delay/delay.c \
  Drivers/BSP/sys/sys.c \
  Drivers/BSP/usart/usart.c \
  Drivers/BSP/adc.c \
  Drivers/BSP/auto_recharge.c \
  Drivers/BSP/can.c \
  Drivers/BSP/buzzer.c \
  Drivers/BSP/encoder.c \
  Drivers/BSP/enable_key.c \
  Drivers/BSP/I2C.c \
  Drivers/BSP/ICM20948/ICM20948.c \
  Drivers/BSP/key.c \
  Drivers/BSP/LED.c \
  Drivers/BSP/motor.c \
  Drivers/BSP/MPU6050/MPU6050.c \
  Drivers/BSP/oled.c \
  Drivers/BSP/remote.c \
  Drivers/BSP/stmflash.c \
  Drivers/BSP/uartx.c \
  Drivers/BSP/hc_sr04.c \
  Drivers/BSP/driver_d50a.c \
  Drivers/BSP/bsp_gamepad.c \
  Drivers/BSP/ps2_classic.c \
  Drivers/BSP/bsp_RGBLight.c \
  App/CarType/4wd_robot_init.c \
  App/BALANCE/balance_task.c \
  App/BALANCE/rear_debug.c \
  App/BALANCE/steer_debug.c \
  App/BALANCE/imu_task.c \
  App/BALANCE/show_task.c \
  App/BALANCE/led_task.c \
  App/BALANCE/ps2_task.c \
  App/BALANCE/data_task.c \
  App/BALANCE/uartx_callback.c \
  App/BALANCE/pwm_test.c \
  App/BALANCE/can_callback.c \
  App/BALANCE/timx_callback.c \
  App/BALANCE/reportErr_task.c \
  Middlewares/STM32_USB_Host_Library/Core/Src/usbh_core.c \
  Middlewares/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.c \
  Middlewares/STM32_USB_Host_Library/Core/Src/usbh_ioreq.c \
  Middlewares/STM32_USB_Host_Library/Core/Src/usbh_pipes.c \
  Middlewares/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.c \
  Middlewares/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.c \
  Middlewares/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.c \
  Middlewares/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.c \
  Middlewares/USB_Host/Target/usbh_conf.c \
  Middlewares/USB_Host/App/usb_host.c \
  Middlewares/USB_Host/App/usbh_os_middleware.c \
  Middlewares/USB_Host/App/usbh_hid_GamePad.c \
  Middlewares/USB_Host/App/xbox360_gamepad.c \
  Middlewares/USB_Host/App/WiredPS2_gamepad.c \
  Core/Startup/startup_gcc.s

# C 源与汇编源分开收集，各自单独生成目标文件
CSRC     = $(filter %.c,$(C_SOURCES))
ASM_SRC  = $(filter %.s,$(C_SOURCES))
OBJECTS  = $(addprefix $(BUILD)/,$(notdir $(CSRC:.c=.o)))
ASM_OBJ  = $(addprefix $(BUILD)/,$(notdir $(ASM_SRC:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(C_SOURCES)))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c Makefile | $(BUILD)
	@$(CC) -c $(CFLAGS) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o $@ $<

$(BUILD)/%.o: %.s Makefile | $(BUILD)
	@$(AS) -c $(ASFLAGS) -o $@ $<

$(BUILD)/$(TARGET).elf: $(OBJECTS) $(ASM_OBJ) Makefile
	$(CC) $(OBJECTS) $(ASM_OBJ) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(CP) -O ihex $< $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(CP) -O binary -S $< $@

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)

.PHONY: all clean
