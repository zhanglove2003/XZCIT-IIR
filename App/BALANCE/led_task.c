#include "led_task.h"

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//BSP Include File
#include "bsp_RGBLight.h"

void RGBControl_task(void *pvParameters);

/**************************************************************************
Function: LED light flashing task
Input   : none
Output  : none
函数功能：LED灯闪烁任务
入口参数：无
返回  值：无
**************************************************************************/
void led_task(void *pvParameters)
{
    while(1)
    {
        static u8 led_state=0;//LED亮灭设置

        //LED闪烁任务
        led_state = !led_state;
        LED_SetState(led_state);
		
        //The LED flicker task is very simple, requires low frequency accuracy, and uses the relative delay function
        //LED闪烁任务非常简单，对频率精度要求低，使用相对延时函数
        vTaskDelay(SysVal.LED_delay);
    }
}


//灯带控制优先级由高到低
enum{
	RGB_LowPower = 2,                //电量低
	RGB_RangerWarning = 3,           //超声波识别障碍物警示
	RGB_UserDefined = 4,             //用户自定义
	RGB_Default = 5                  //默认状态
};

//用户自定义RGB状态
uint8_t userdefine_rgb[4]={ 0 };

static uint8_t RGB_PriorityManagement(void)
{
	//电量低判断
	if( robot.LowPower ) return RGB_LowPower; 
	
	//超声波警示判断
	if( s21c_board.rangerA < 0.5f ||s21c_board.rangerB  < 0.5f || s21c_board.rangerC  < 0.5f ||
		s21c_board.rangerD < 0.5f || s21c_board.rangerE < 0.5f || s21c_board.rangerF < 0.5f )
	{
		return RGB_RangerWarning;
	}
	
	//用户自定义
	if( userdefine_rgb[0]==1 ) 
	{
		return RGB_UserDefined;
	}
	
	//默认状态
	return RGB_Default;

}

void RGBControl_task(void *pvParameters)
{
	const uint16_t TaskFreq = 5;
	
	//获取时基,用于辅助任务能按固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//RGB灯带对象
	pRGBLightInterface_t rgb = &UserRGBLight;
	
	//实时状态机
	uint8_t RGB_Sm = RGB_Default;
	
	//灯带初始化与自检
	rgb->init();
	rgb->SetColor(255,0,0);
	vTaskDelay(1000);
	rgb->SetColor(0,255,0);
	vTaskDelay(1000);
	rgb->SetColor(0,0,255);
	vTaskDelay(1000);
	
	while(1)
	{
		//获取RGB状态灯状态
		RGB_Sm = RGB_PriorityManagement();
		
		switch( RGB_Sm )
		{
			case RGB_LowPower:
				//紫灯低频闪烁
				rgb->SetBlink(100,0,128,2000);
				break;
			case RGB_RangerWarning:
				//黄灯警示
				rgb->SetColorFade(255,128,0);
				break;
			case RGB_UserDefined:
				//用户自定义
				rgb->SetColorFade(userdefine_rgb[1],userdefine_rgb[2],userdefine_rgb[3]);
				break;
			case RGB_Default:
				//设置灯带默认状态
				rgb->turnoff();
				break;
			default:
				break;
		}	
		
		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}




