#include "data_task.h"
#include "hc_sr04.h"
#include <stdio.h>

/* USART1: readable diagnostics; set to 0 for the legacy binary protocol. */
#ifndef C50C_USART1_TEXT
#define C50C_USART1_TEXT 1
#endif

//用于发送数据的结构体
SEND_DATA Send_Data;

S21C_SensorData_t s21c_board = { 
	.rangerA = 5,
	.rangerB = 5,
	.rangerC = 5,
	.rangerD = 5,
	.rangerE = 5,
	.rangerF = 5,
};

//超声波数据
#define RangerFRAME_HEAD 0xFA
#define RangerFRAME_TAIL 0xFC
#define RangerFRAME_LEN 19
static uint8_t rangerbuffer[RangerFRAME_LEN];

uint8_t Check_BCC(const uint8_t *data, uint16_t length) {
    uint8_t bcc = 0;
    for (uint16_t i = 0; i < length; i++) {
        bcc ^= data[i];
    }
    return bcc;
}
	
/**************************************************************************
Function: Robot Data Transmission Task: Sending robot status, IMU, speed, and other information to various interfaces.
Input   : none
Output  : none
函数功能： 机器人数据发送任务,向各个接口发送机器人的状态,imu,速度等信息
入口参数：无
返回  值：无
**************************************************************************/
TaskHandle_t data_TaskHandle = NULL;

void data_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();
	
   while(1)
    {	
			//The task is run at 20hz
			//此任务以20Hz的频率运行
			vTaskDelayUntil(&lastWakeTime, F2T(DATA_TASK_RATE));
			//Assign the data to be sent
			//对要进行发送的数据进行赋值
			data_transition(); 
			Usart1_SendTask();
			Usart3_SendTask();
		}
}

/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
static void data_transition(void)
{
	unsigned char i;
	//根据车型的不同运行相对应的运动学正解函数,获取机器人3轴计算速度
	float *vel;
	vel = Kinematics_mec_4wd(robot.MOTOR_A.Encoder,robot.MOTOR_B.Encoder,robot.MOTOR_C.Encoder,robot.MOTOR_D.Encoder);
	
	//Forward kinematics solution, from the current speed of each wheel to calculate the current speed of the three axis
	//运动学正解，从各车轮当前速度求出三轴当前速度
	Send_Data.Sensor_Str.Vel.X_speed = vel[0]*1000; //小车x轴速度,扩大1000倍发送
	Send_Data.Sensor_Str.Vel.Y_speed = vel[1]*1000; //小车y轴速度,扩大1000倍发送
	Send_Data.Sensor_Str.Vel.Z_speed = vel[2]*1000; //小车z轴速度,扩大1000倍发送
	//The acceleration of the triaxial acceleration //加速度计三轴加速度
	Send_Data.Sensor_Str.Accelerometer.X_data= imu.accel.y; //The accelerometer Y-axis is converted to the ros coordinate X axis //加速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Accelerometer.Y_data=-imu.accel.x; //The accelerometer X-axis is converted to the ros coordinate y axis //加速度计X轴转换到ROS坐标Y轴
	Send_Data.Sensor_Str.Accelerometer.Z_data= imu.accel.z; //The accelerometer Z-axis is converted to the ros coordinate Z axis //加速度计Z轴转换到ROS坐标Z轴
	
	//The Angle velocity of the triaxial velocity //角速度计三轴角速度
	Send_Data.Sensor_Str.Gyroscope.X_data= imu.gyro.y; //The Y-axis is converted to the ros coordinate X axis //角速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Gyroscope.Y_data=-imu.gyro.x; //The X-axis is converted to the ros coordinate y axis //角速度计X轴转换到ROS坐标Y轴
	
	if( 0 == robot_control.FlagStop ) 
		//If the motor control bit makes energy state, the z-axis velocity is sent normall
	  //如果电机控制位使能状态，那么正常发送Z轴角速度
		Send_Data.Sensor_Str.Gyroscope.Z_data=imu.gyro.z;  
	else  
		//If the robot is static (motor control dislocation), the z-axis is 0
    //如果机器人是静止的（电机控制位失能），那么发送的Z轴角速度为0		
		Send_Data.Sensor_Str.Gyroscope.Z_data=0;  
	
	//Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	//电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)
	Send_Data.Sensor_Str.Power_Voltage = robot.voltage*1000; 
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade //帧头
	Send_Data.buffer[1]=robot_control.FlagStop; //Car software loss marker //小车软件失能标志位
	
	//The three-axis speed of / / car is split into two eight digit Numbers
	//小车三轴速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[2]=Send_Data.Sensor_Str.Vel.X_speed >>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.Vel.X_speed ;    
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Vel.Y_speed>>8;  
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Vel.Y_speed;     
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Vel.Z_speed >>8; 
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Vel.Z_speed ;    
	
	//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	//IMU加速度计三轴加速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; 
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;   
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	//The axis of the triaxial velocity of the / /imu is divided into two eight digits
	//IMU角速度计三轴角速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	//Battery voltage, split into two 8 digit Numbers
	//电池电压,拆分为两个8位数据发送
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8; 
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; 

  //Data check digit calculation, Pattern 1 is a data check
  //数据校验位计算，模式1是发送数据校验
	//四轮编码器计数 A/B/C/D，int32 大端
	/* 原厂缺陷: robot.MOTOR_x.Encoder 单位是 m/s(float), 直接 (int) 截断后
	 * 只有满 1.0 m/s 才非零, 低速下恒发 0. 这里改为 mm/s 整数上报,
	 * 主机侧按 mm/s 解析即可。 */
	Send_Data.Sensor_Str.Encoder[0]=(int)(robot.MOTOR_A.Encoder * 1000.0f);
	Send_Data.Sensor_Str.Encoder[1]=(int)(robot.MOTOR_B.Encoder * 1000.0f);
	Send_Data.Sensor_Str.Encoder[2]=(int)(robot.MOTOR_C.Encoder * 1000.0f);
	Send_Data.Sensor_Str.Encoder[3]=(int)(robot.MOTOR_D.Encoder * 1000.0f);
	//四路 PWM 比较值 CCR1..CCR4，uint16 大端
	Send_Data.Sensor_Str.Pwm[0]=(unsigned short)TIM8->CCR1;
	Send_Data.Sensor_Str.Pwm[1]=(unsigned short)TIM8->CCR2;
	Send_Data.Sensor_Str.Pwm[2]=(unsigned short)TIM8->CCR3;
	Send_Data.Sensor_Str.Pwm[3]=(unsigned short)TIM8->CCR4;
	for(i=0;i<4;i++)
	{
		Send_Data.buffer[SEND_DATA_ENCODER_OFFSET+i*4+0]=(unsigned char)(Send_Data.Sensor_Str.Encoder[i]>>24);
		Send_Data.buffer[SEND_DATA_ENCODER_OFFSET+i*4+1]=(unsigned char)(Send_Data.Sensor_Str.Encoder[i]>>16);
		Send_Data.buffer[SEND_DATA_ENCODER_OFFSET+i*4+2]=(unsigned char)(Send_Data.Sensor_Str.Encoder[i]>>8);
		Send_Data.buffer[SEND_DATA_ENCODER_OFFSET+i*4+3]=(unsigned char)(Send_Data.Sensor_Str.Encoder[i]);
	}
	for(i=0;i<4;i++)
	{
		Send_Data.buffer[SEND_DATA_PWM_OFFSET+i*2]  =(unsigned char)(Send_Data.Sensor_Str.Pwm[i]>>8);
		Send_Data.buffer[SEND_DATA_PWM_OFFSET+i*2+1]=(unsigned char)(Send_Data.Sensor_Str.Pwm[i]);
	}
	/* [C50C 2026-11-15] Front HC-SR04 range, uint16 big-endian mm;
	 * 0xFFFF = no echo / stale (ROS side maps it to max_range). */
	Send_Data.Sensor_Str.Ultrasonic_mm = HC_SR04_GetDistance_mm();
	Send_Data.buffer[SEND_DATA_ULTRASONIC_OFFSET+0]=(unsigned char)(Send_Data.Sensor_Str.Ultrasonic_mm>>8);
	Send_Data.buffer[SEND_DATA_ULTRASONIC_OFFSET+1]=(unsigned char)(Send_Data.Sensor_Str.Ultrasonic_mm);
	Send_Data.buffer[SEND_DATA_BCC_LEN]=Check_BCC(Send_Data.buffer,SEND_DATA_BCC_LEN);
	Send_Data.buffer[SEND_DATA_SIZE-1]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //帧尾

	////////////// 超声波数据帧 /////////////
	rangerbuffer[0] = RangerFRAME_HEAD;
	rangerbuffer[RangerFRAME_LEN-1] = RangerFRAME_TAIL;
	rangerbuffer[1] = (short)(s21c_board.rangerA*1000)>>8;
	rangerbuffer[2] = (short)(s21c_board.rangerA*1000);
	rangerbuffer[3] = (short)(s21c_board.rangerB*1000)>>8;
	rangerbuffer[4] = (short)(s21c_board.rangerB*1000);
	rangerbuffer[5] = (short)(s21c_board.rangerC*1000)>>8;
	rangerbuffer[6] = (short)(s21c_board.rangerC*1000);
	rangerbuffer[7] = (short)(s21c_board.rangerD*1000)>>8;
	rangerbuffer[8] = (short)(s21c_board.rangerD*1000);
	rangerbuffer[9] = (short)(s21c_board.rangerE*1000)>>8;
	rangerbuffer[10] = (short)(s21c_board.rangerE*1000);
	rangerbuffer[11] = (short)(s21c_board.rangerF*1000)>>8;
	rangerbuffer[12] = (short)(s21c_board.rangerF*1000);
	rangerbuffer[13] = 0;
	rangerbuffer[14] = 0;
	rangerbuffer[15] = 0;
	rangerbuffer[16] = 0;
	rangerbuffer[17] = Check_BCC(rangerbuffer,17);
	////////////// 超声波数据帧 /////////////
}

/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
函数功能：串口1发送数据
入口参数：无
返回  值：无
**************************************************************************/
static void Usart1_SendTask(void)
{
#if C50C_USART1_TEXT
    /* Send at most 16 bytes per 50 ms cycle; do not block the control task
       for an entire formatted line at 115200 baud. */
    static char line[512];
    static unsigned int pos, len;
    unsigned int budget = 32;
    if (pos == len) {
        int n = snprintf(line, sizeof(line),
            "C50C stop=%u mode=%u A:%ld,%ld,%d B:%ld,%ld,%d C:%ld,%ld,%d D:%ld,%ld,%d\r\n",
            (unsigned)robot_control.FlagStop, (unsigned)robot_control.ControlMode,
            (long)(robot.MOTOR_A.Target * 1000.0f), (long)(robot.MOTOR_A.Encoder * 1000.0f), robot.MOTOR_A.Output,
            (long)(robot.MOTOR_B.Target * 1000.0f), (long)(robot.MOTOR_B.Encoder * 1000.0f), robot.MOTOR_B.Output,
            (long)(robot.MOTOR_C.Target * 1000.0f), (long)(robot.MOTOR_C.Encoder * 1000.0f), robot.MOTOR_C.Output,
            (long)(robot.MOTOR_D.Target * 1000.0f), (long)(robot.MOTOR_D.Encoder * 1000.0f), robot.MOTOR_D.Output);
        if (n <= 0 || (unsigned int)n >= sizeof(line)) { pos = len = 0; return; }
        pos = 0;
        len = (unsigned int)n;
    }
    while (pos < len && budget-- > 0) uart1_send((unsigned char)line[pos++]);
#else

  unsigned char i = 0;	
	
	for(i=0; i<SEND_DATA_SIZE; i++)
	{
		uart1_send(Send_Data.buffer[i]);
	}	 
	
	if( SysVal.HardWare_Ranger==1 )
	{
		for(i=0; i<RangerFRAME_LEN; i++)
		{
			uart1_send(rangerbuffer[i]);
		}
	}
#endif
}

/**************************************************************************
Function: Serial port 3 sends data
Input   : none
Output  : none
函数功能：串口3发送数据
入口参数：无
返回  值：无
**************************************************************************/
static void Usart3_SendTask(void)
{
  unsigned char i = 0;	
	for(i=0; i<SEND_DATA_SIZE; i++)
	{
		uart3_send(Send_Data.buffer[i]);
	}	 
	
	if( SysVal.HardWare_Ranger==1 )
	{
		for(i=0; i<RangerFRAME_LEN; i++)
		{
			uart3_send(rangerbuffer[i]);
		}
	}
}

/*-------------------- Positive kinematics correlation function for each vehicle model ------------------------*/
/*--------------------------------         各车型运动学正解相关函数          ------------------------------------*/
/**************************************************************************
Function Purpose: akm/diff Kinematic Analysis
Input Parameters: Left wheel speed, Right wheel speed, in meters per second (m/s)
Return Value: Robot's x, y, z velocities
Author: WHEELTEC
函数功能：阿克曼/差速 运动学正解
入口参数：左轮速度、右轮速度,单位 m/s
返回  值：机器人x、y、z三轴速度
作    者：WHEELTEC
**************************************************************************/
/**************************************************************************
Function Purpose: mec/4wd Kinematic Analysis
Input Parameters: Robot's four-wheel speeds, in meters per second (m/s).
Return Value: Robot's x, y, z velocities
Author: WHEELTEC
函数功能：麦轮/四驱 运动学正解
入口参数：机器人四个轮的速度,单位 m/s
返回  值：机器人x、y、z三轴速度
作    者：WHEELTEC
**************************************************************************/
static float* Kinematics_mec_4wd(float motorA,float motorB,float motorC,float motorD)
{
	static float vel[3];
	//xyz三轴计算速度
	vel[0] = (motorA+motorB+motorC+motorD)/4.0f;
	vel[1] = (motorA-motorB+motorC-motorD)/4.0f;
	vel[2] = (-motorA-motorB+motorC+motorD)/4.0f/( robot.HardwareParam.WheelSpacing + robot.HardwareParam.AxleSpacing );
	
	return vel;
}


