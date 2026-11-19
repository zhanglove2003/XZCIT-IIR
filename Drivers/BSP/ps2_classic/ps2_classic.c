#include "pstwo.h"
#include "FreeRTOS.h"
#include "task.h"
#if defined(REAR_WHEEL_DEBUG) && defined(_4WD_CAR)
volatile uint8_t g_rear_ps2_valid;
volatile uint32_t g_rear_ps2_tick;
static uint8_t s_rear_ps2_raw_valid;
#endif

/* USB Host 整链移除后, 按键枚举与手柄类型由 pstwo.h 直接提供. */

//经典PS2手柄
GamePadType_t Classic_PS2Gamepad = {
	.LX = 127,
	.LY = 127,
	.RX = 127,
	.RY = 127,
	.LT = 0,
	.RT = 0,
	.StartFlag = 0,
	.Vib_EN = 0
};


__weak void Classic_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	printf("%d,%d\r\n",keyid,event);
}
