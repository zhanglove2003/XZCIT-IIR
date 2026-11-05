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
		
		if( SysVal.HardWare_Ver == V1_0 ) // V1.0硬件版本 使用常规ps2手柄
		{
			//Read the ps2 data
			//读取PS2的数据
			PS2_Read();		
		}
		else if( SysVal.HardWare_Ver >= V1_1 ) //V1.1硬件版本 使用usb手柄
		{
			//V1.1硬件使用USB手柄,读取任务不在此函数
		}
		
		//游戏手柄模式启动
		//原厂条件要求 LY>150(即左摇杆向后推到底)才进入PS2模式, 向前推永远进不去.
		//改为: 按过START键, 且任意摇杆离开中位(偏移超过死区), 即进入PS2模式.
		if( GamePadInterface != 0 && GamePadInterface->StartFlag == 1 && Get_Control_Mode(_PS2_Control)==0
		 && SysVal.Time_count>=CONTROL_DELAY
		 && ( PS2_Stick_Offset(GamePadInterface->LX) > PS2_NEUTRAL_THRESHOLD
		   || PS2_Stick_Offset(GamePadInterface->LY) > PS2_NEUTRAL_THRESHOLD
		   || PS2_Stick_Offset(GamePadInterface->RX) > PS2_NEUTRAL_THRESHOLD
		   || PS2_Stick_Offset(GamePadInterface->RY) > PS2_NEUTRAL_THRESHOLD ) )
			Set_Control_Mode(_PS2_Control);
		
    }
}

