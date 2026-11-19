#include "ps2_task.h"
#include "pstwo.h"

/**************************************************************************
Function: Ps2 handle task
Input   : none
Output  : none
函数功能：PS2手柄任务
入口参数：无
返回  值：无
**************************************************************************/
void pstwo_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();

    while(1)
    {
        vTaskDelayUntil(&lastWakeTime, F2T(PS2_TASK_RATE));
		
		//PS2_Init/PS2_Read 已随 PS2 外设定案移除, Classic_PS2Gamepad 恒为中位, 不再进入 PS2 控制模式.

		//游戏手柄模式启动
		//原厂条件要求 LY>150(即左摇杆向后推到底)才进入PS2模式, 向前推永远进不去.
		//改为: 按过START键, 且任意摇杆离开中位(偏移超过死区), 即进入PS2模式.
		if( Classic_PS2Gamepad.StartFlag == 1 && Get_Control_Mode(_PS2_Control)==0
		 && SysVal.Time_count>=CONTROL_DELAY
		 && ( PS2_Stick_Offset(Classic_PS2Gamepad.LX) > PS2_NEUTRAL_THRESHOLD
		   || PS2_Stick_Offset(Classic_PS2Gamepad.LY) > PS2_NEUTRAL_THRESHOLD
		   || PS2_Stick_Offset(Classic_PS2Gamepad.RX) > PS2_NEUTRAL_THRESHOLD
		   || PS2_Stick_Offset(Classic_PS2Gamepad.RY) > PS2_NEUTRAL_THRESHOLD ) )
			Set_Control_Mode(_PS2_Control);

    }
}

