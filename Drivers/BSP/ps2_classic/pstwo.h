#ifndef __PSTWO_H
#define __PSTWO_H
#include "sys.h"

/* 经典 GPIO 时序 PS2 手柄的最小手柄模型.
 * USB Host 整链移除后, 原bsp_gamepad.h的通用手柄类型收敛到这里,
 * 仅保留 ps2_classic.c / balance_task.c 实际用到的字段与按键枚举. */

//PS2按键位置枚举(bit0~bit15分别为下面的0~15)
enum
{
	PS2KEY_SELECT	   = 0, //选择按键
	PS2KEY_LROCKER      , //左右摇杆按下键值
	PS2KEY_RROCKER      ,
	PS2KEY_START        , //开始按键
	PS2KEY_UP           , //左按键区域
	PS2KEY_RIGHT        ,
	PS2KEY_DOWN         ,
	PS2KEY_LEFT         ,
	PS2KEY_L2           ,	//左右扳机按键值
	PS2KEY_R2           ,
	PS2KEY_L1           ,
	PS2KEY_R1           ,
	PS2KEY_1GREEN       , //右按键区域
	PS2KEY_2RED         ,
	PS2KEY_3BLUE        ,
	PS2KEY_4PINK
};

//游戏手柄按键通用事件
typedef enum{
    GamePadKeyEvent_NONE,         //无按键事件
    GamePadKeyEvent_SINGLECLICK,  //单击事件
    GamePadKeyEvent_DOUBLECLICK,  //双击事件
    GamePadKeyEvent_LONGCLICK     //长按事件
}GamePadKeyEventType_t;

//游戏手柄通用按键状态
typedef enum{
	GamePadKeyState_Release = 0,  //松开
	GamePadKeyState_Press = 1     //按下
}GamePadKeyStateType_t;

//通用手柄按键检测状态机
typedef enum{
    WaitToPress = 0,
    WaitToRelease ,
    KEYPress    ,
    KEYUp       ,
    LONG_CLICK  ,
}GamePad_StateMachineType_t;

//通用手柄单个按键属性
typedef struct
{
    uint8_t keystate;
    uint32_t timebase;
    uint32_t statetime;
    GamePad_StateMachineType_t statemachine;
}GamePad_CheckEventType_t;

//游戏手柄按键通用检测时间,单位ms
#define GamePad_LONGPRESS_TIEM 1000 //长按检测时间
#define GamePad_CLICK_TIME     400  //单双击检测时间
#define GamePad_KEYFILTER_TIME 50   //按键滤波时间

//通用游戏手柄对象
typedef struct{
	uint8_t LX;  //4个方向摇杆值,取值0~255
	uint8_t LY;
	uint8_t RX;
	uint8_t RY;
	uint8_t LT;  //左右两个油门扳机
	uint8_t RT;
	uint8_t StartFlag;
	uint8_t Vib_EN;
}GamePadType_t;

//对外接口
#if defined(REAR_WHEEL_DEBUG) && defined(_4WD_CAR)
extern volatile uint8_t g_rear_ps2_valid;
extern volatile uint32_t g_rear_ps2_tick;
#endif

//经典GPIO时序PS2手柄对象(定义在ps2_classic.c)
extern GamePadType_t Classic_PS2Gamepad;

//摇杆中位死区. 手柄模拟量中位为127
#define PS2_STICK_CENTER        127
#define PS2_NEUTRAL_THRESHOLD   20

//取摇杆偏离中位的绝对值
static inline unsigned char PS2_Stick_Offset(unsigned char raw)
{
	if( raw >= PS2_STICK_CENTER ) return (unsigned char)(raw - PS2_STICK_CENTER);
	return (unsigned char)(PS2_STICK_CENTER - raw);
}

#endif





