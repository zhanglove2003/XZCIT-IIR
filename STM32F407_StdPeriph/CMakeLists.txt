cmake_minimum_required(VERSION 3.21)

# Keep the existing Makefile toolchain as the default for IDE users.
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/arm-none-eabi.cmake"
        CACHE FILEPATH "Arm GCC cross-compilation toolchain")
endif()

project(C50C_Firmware LANGUAGES C ASM)

find_program(C50C_OBJCOPY NAMES arm-none-eabi-objcopy REQUIRED)
find_program(C50C_SIZE NAMES arm-none-eabi-size REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# The source list follows the current Makefile's default 4WD ROS build.
add_executable(Diff_Car
    FWLIB/src/misc.c
    FWLIB/src/stm32f4xx_gpio.c
    FWLIB/src/stm32f4xx_rcc.c
    FWLIB/src/stm32f4xx_syscfg.c
    FWLIB/src/stm32f4xx_usart.c
    FWLIB/src/stm32f4xx_adc.c
    FWLIB/src/stm32f4xx_can.c
    FWLIB/src/stm32f4xx_tim.c
    FWLIB/src/stm32f4xx_pwr.c
    FWLIB/src/stm32f4xx_dma.c
    FWLIB/src/stm32f4xx_hal_hcd.c
    FWLIB/src/stm32f4xx_ll_usb.c
    FreeRTOS/croutine.c
    FreeRTOS/event_groups.c
    FreeRTOS/list.c
    FreeRTOS/queue.c
    FreeRTOS/tasks.c
    FreeRTOS/timers.c
    FreeRTOS/portable/RVDS/ARM_CM4F/port.c
    FreeRTOS/portable/MemMang/heap_4.c
    USER/main.c
    USER/system.c
    USER/stm32f4xx_it.c
    USER/system_stm32f4xx.c
    SYSTEM/delay/delay.c
    SYSTEM/sys/sys.c
    SYSTEM/usart/usart.c
    HARDWARE/adc.c
    HARDWARE/auto_recharge.c
    HARDWARE/can.c
    HARDWARE/buzzer.c
    HARDWARE/encoder.c
    HARDWARE/enable_key.c
    HARDWARE/I2C.c
    HARDWARE/ICM20948/ICM20948.c
    HARDWARE/key.c
    HARDWARE/LED.c
    HARDWARE/motor.c
    HARDWARE/MPU6050/MPU6050.c
    HARDWARE/oled.c
    HARDWARE/remote.c
    HARDWARE/stmflash.c
    HARDWARE/uartx.c
    HARDWARE/driver_d50a.c
    HARDWARE/bsp_gamepad.c
    HARDWARE/ps2_classic.c
    HARDWARE/bsp_RGBLight.c
    CarType/4wd_robot_init.c
    BALANCE/balance_task.c
    BALANCE/rear_debug.c
    BALANCE/steer_debug.c
    BALANCE/imu_task.c
    BALANCE/show_task.c
    BALANCE/led_task.c
    BALANCE/ps2_task.c
    BALANCE/data_task.c
    BALANCE/uartx_callback.c
    BALANCE/pwm_test.c
    BALANCE/can_callback.c
    BALANCE/timx_callback.c
    BALANCE/reportErr_task.c
    MiddleWares/STM32_USB_Host_Library/Core/Src/usbh_core.c
    MiddleWares/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.c
    MiddleWares/STM32_USB_Host_Library/Core/Src/usbh_ioreq.c
    MiddleWares/STM32_USB_Host_Library/Core/Src/usbh_pipes.c
    MiddleWares/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.c
    MiddleWares/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.c
    MiddleWares/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.c
    MiddleWares/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.c
    USB_HOST/Target/usbh_conf.c
    USB_HOST/App/usb_host.c
    USB_HOST/App/usbh_os_middleware.c
    USB_HOST/App/usbh_hid_GamePad.c
    USB_HOST/App/xbox360_gamepad.c
    USB_HOST/App/WiredPS2_gamepad.c
    CORE/startup_gcc.s
)
set_target_properties(Diff_Car PROPERTIES SUFFIX ".elf")

target_include_directories(Diff_Car PRIVATE
    CORE
    SYSTEM/delay
    SYSTEM/sys
    SYSTEM/usart
    USER
    FWLIB/inc
    FreeRTOS/include
    FreeRTOS/portable/RVDS/ARM_CM4F
    CarType/Inc
    BALANCE/Inc
    HARDWARE/Inc
    HARDWARE/MPU6050
    HARDWARE/ICM20948
    MiddleWares/STM32_USB_Host_Library/Class/HID/Inc
    MiddleWares/STM32_USB_Host_Library/Core/Inc
    USB_HOST/App
    USB_HOST/Target
)

target_compile_definitions(Diff_Car PRIVATE
    USE_STDPERIPH_DRIVER
    __FPU_PRESENT=1
    __TARGET_FPU_VFP
    ARM_MATH_CM4
    USE_USB_OTG_FS
    _4WD_CAR
    STM32F40_41xxx
    "__weak=__attribute__((weak))"
    "__packed=__attribute__((__packed__))"
    "__align=__attribute__((aligned(4)))"
    "__inline=inline"
    "__forceinline=inline __attribute__((always_inline))"
    "__nop=__NOP"
    REAR_WHEEL_DEBUG
    ROS_STRAIGHT_DRIVE
    FOUR_TURN_DEBUG
    FRONT_TEST_CHANNEL=6
)

set(C50C_MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

target_compile_options(Diff_Car PRIVATE
    ${C50C_MCU_FLAGS}
    -O2 -g3 -Wall -ffunction-sections -fdata-sections
    $<$<COMPILE_LANGUAGE:ASM>:-xassembler-with-cpp>
    # Match Makefile's relative __FILE__ strings in firmware assertions.
    "$<$<COMPILE_LANGUAGE:C>:-fmacro-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}/=>"
    $<$<COMPILE_LANGUAGE:C>:-fno-common>
    $<$<COMPILE_LANGUAGE:C>:-Wno-unused-parameter>
    $<$<COMPILE_LANGUAGE:C>:-Wno-unused-variable>
    $<$<COMPILE_LANGUAGE:C>:-Wno-unused-but-set-variable>
    $<$<COMPILE_LANGUAGE:C>:-Wno-missing-field-initializers>
    $<$<COMPILE_LANGUAGE:C>:-Wno-parentheses>
    $<$<COMPILE_LANGUAGE:C>:-Wno-pointer-sign>
    $<$<COMPILE_LANGUAGE:C>:-Wno-implicit-fallthrough>
    $<$<COMPILE_LANGUAGE:C>:-Wno-address-of-packed-member>
)

target_link_options(Diff_Car PRIVATE
    ${C50C_MCU_FLAGS}
    "-T${CMAKE_CURRENT_SOURCE_DIR}/WHEELTEC_C50C.ld"
    -specs=nano.specs
    -specs=nosys.specs
    -Wl,--gc-sections
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/Diff_Car.map,--cref"
    -Wl,--print-memory-usage
)
target_link_libraries(Diff_Car PRIVATE c m nosys)
set_property(TARGET Diff_Car APPEND PROPERTY LINK_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/WHEELTEC_C50C.ld")

add_custom_command(TARGET Diff_Car POST_BUILD
    COMMAND "${C50C_OBJCOPY}" -O ihex
            "$<TARGET_FILE:Diff_Car>" "${CMAKE_CURRENT_BINARY_DIR}/Diff_Car.hex"
    COMMAND "${C50C_OBJCOPY}" -O binary -S
            "$<TARGET_FILE:Diff_Car>" "${CMAKE_CURRENT_BINARY_DIR}/Diff_Car.bin"
    COMMAND "${C50C_SIZE}" "$<TARGET_FILE:Diff_Car>"
    VERBATIM
)
