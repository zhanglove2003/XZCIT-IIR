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


C_INCLUDES = -ICORE \
             -ISYSTEM/delay \
             -ISYSTEM/sys \
             -ISYSTEM/usart \
             -IUSER \
             -IFWLIB/inc \
             -IFreeRTOS/include \
             -IFreeRTOS/portable/RVDS/ARM_CM4F \
             -ICarType/Inc \
             -IBALANCE/Inc \
             -IHARDWARE/Inc \
             -IHARDWARE/MPU6050 \
             -IHARDWARE/ICM20948 \
             -IMiddleWares/STM32_USB_Host_Library/Class/HID/Inc \
             -IMiddleWares/STM32_USB_Host_Library/Core/Inc \
             -IUSB_HOST/App \
             -IUSB_HOST/Target

OPT      = -O2 -g3
CFLAGS   = $(MCU) $(DEFS) $(C_INCLUDES) $(OPT) -Wall \
           -ffunction-sections -fdata-sections -fno-common \
           -std=gnu99 -Wno-unused-parameter -Wno-unused-variable \
           -Wno-unused-but-set-variable -Wno-missing-field-initializers \
           -Wno-parentheses -Wno-pointer-sign -Wno-implicit-fallthrough \
           -Wno-address-of-packed-member
ASFLAGS  = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections
LDSCRIPT = WHEELTEC_C50C.ld
LDFLAGS  = $(MCU) -T$(LDSCRIPT) -specs=nano.specs -specs=nosys.specs \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map,--cref \
           -Wl,--print-memory-usage -lc -lm -lnosys

C_SOURCES = \
  FWLIB/src/misc.c \
  FWLIB/src/stm32f4xx_gpio.c \
  FWLIB/src/stm32f4xx_rcc.c \
  FWLIB/src/stm32f4xx_syscfg.c \
  FWLIB/src/stm32f4xx_usart.c \
  FWLIB/src/stm32f4xx_adc.c \
  FWLIB/src/stm32f4xx_can.c \
  FWLIB/src/stm32f4xx_tim.c \
  FWLIB/src/stm32f4xx_pwr.c \
  FWLIB/src/stm32f4xx_dma.c \
  FWLIB/src/stm32f4xx_hal_hcd.c \
  FWLIB/src/stm32f4xx_ll_usb.c \
  FreeRTOS/croutine.c \
  FreeRTOS/event_groups.c \
  FreeRTOS/list.c \
  FreeRTOS/queue.c \
  FreeRTOS/tasks.c \
  FreeRTOS/timers.c \
  FreeRTOS/portable/RVDS/ARM_CM4F/port.c \
  FreeRTOS/portable/MemMang/heap_4.c \
  USER/main.c \
  USER/system.c \
  USER/stm32f4xx_it.c \
  USER/system_stm32f4xx.c \
  SYSTEM/delay/delay.c \
  SYSTEM/sys/sys.c \
  SYSTEM/usart/usart.c \
  HARDWARE/adc.c \
  HARDWARE/auto_recharge.c \
  HARDWARE/can.c \
  HARDWARE/buzzer.c \
  HARDWARE/encoder.c \
  HARDWARE/enable_key.c \
  HARDWARE/I2C.c \
  HARDWARE/ICM20948/ICM20948.c \
  HARDWARE/key.c \
  HARDWARE/LED.c \
  HARDWARE/motor.c \
  HARDWARE/MPU6050/MPU6050.c \
  HARDWARE/oled.c \
  HARDWARE/remote.c \
  HARDWARE/stmflash.c \
  HARDWARE/uartx.c \
  HARDWARE/driver_d50a.c \
  HARDWARE/bsp_gamepad.c \
  HARDWARE/ps2_classic.c \
  HARDWARE/bsp_RGBLight.c \
  CarType/4wd_robot_init.c \
  BALANCE/balance_task.c \
  BALANCE/rear_debug.c \
  BALANCE/steer_debug.c \
  BALANCE/imu_task.c \
  BALANCE/show_task.c \
  BALANCE/led_task.c \
  BALANCE/ps2_task.c \
  BALANCE/data_task.c \
  BALANCE/uartx_callback.c \
  BALANCE/can_callback.c \
  BALANCE/timx_callback.c \
  BALANCE/reportErr_task.c \
  MiddleWares/STM32_USB_Host_Library/Core/Src/usbh_core.c \
  MiddleWares/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.c \
  MiddleWares/STM32_USB_Host_Library/Core/Src/usbh_ioreq.c \
  MiddleWares/STM32_USB_Host_Library/Core/Src/usbh_pipes.c \
  MiddleWares/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.c \
  MiddleWares/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.c \
  MiddleWares/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.c \
  MiddleWares/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.c \
  USB_HOST/Target/usbh_conf.c \
  USB_HOST/App/usb_host.c \
  USB_HOST/App/usbh_os_middleware.c \
  USB_HOST/App/usbh_hid_GamePad.c \
  USB_HOST/App/xbox360_gamepad.c \
  USB_HOST/App/WiredPS2_gamepad.c \
  CORE/startup_gcc.s

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
