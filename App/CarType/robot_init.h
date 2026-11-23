#ifndef __ROBOT_INIT_H
#define __ROBOT_INIT_H
#include "sys.h"
#include "system.h"

//Parameter structure of robot
//机器人结构相关参数
typedef struct  
{
	float WheelSpacing;           //Wheelspacing, Mec_Car is half wheelspacing //轮距参数 麦轮、四驱为半轮距
	float AxleSpacing;            //Axlespacing, Mec_Car is half axlespacing   //轴距参数 麦轮、四驱为半轴距	
	float Wheel_Circ;             //Wheel circumference                        //主动轮周长
	float GearRatio;              //Motor_gear_ratio                           //电机减速比参数
	float Encoder_precision;      //车轮转一圈对应的编码器数值
	uint16_t   EncoderAccuracy;   //Number_of_encoder_lines                    //编码器精度(编码器线数)
	
	
}Robot_Parament_InitTypeDef;

//Robot servo motor related parameters
//机器人伺服电机相关参数
typedef struct  
{
	float Target;                //Control the target speed of the motor            //电机目标速度值，控制电机目标速度
	float Encoder;               //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	int Output;                  //Output PWM Values to Motors for Controlling Rotation //输出到电机的PWM值,用于控制电机转动
}Moto_parameter;

typedef struct
{
	Robot_Parament_InitTypeDef HardwareParam;//机器人的结构参数
	
	Moto_parameter MOTOR_A;
	Moto_parameter MOTOR_B;
	Moto_parameter MOTOR_C;
	Moto_parameter MOTOR_D;
	
	
	//Robot model 
	//小车型号
	uint8_t    type;      
	
	//机器人电池电压
	float voltage;
	
	//用于控制机器人运动的pid参数
	int V_KP;
	int V_KI;
	
	uint8_t LowPower;
}ROBOT_t;


//对外接口
extern ROBOT_t robot;
void Robot_Select(void);

//Motor_gear_ratio
//电机减速比
#define   MD36N_5_18  5.18
#define   MD36N_27    27
#define   MD36N_51    51
#define   MD36N_71    71
#define   MD60N_18    18
#define   MD60N_47    47

//Number_of_encoder_lines
//编码器精度
#define		GMR_500  500
#define	    Hall_13  13

//The encoder octave depends on the encoder initialization Settings
//编码器倍频数，取决于编码器初始化设置
#define   EncoderMultiples 4

//默认PID参数
#define VEL_KP 300
#define VEL_KI 300

//不同类别车型下的参数
	//half wheelspacing //半轮距
	#define SENIOR_4WD_BS_wheelspacing   0.176 
	#define SENIOR_4WD_DL_wheelspacing   0.230
	#define TOP_4WD_BS_wheelspacing      0.311
	#define TOP_4WD_DL_wheelspacing      0.295
	#define FlagShip_4WD_BS_wheelspacing 0.286
	#define FlagShip_4WD_DL_wheelspacing 0.24

	//half axlespacing //半轴距
	#define SENIOR_4WD_BS_axlespacing   0.150
	#define SENIOR_4WD_DL_axlespacing   0.20 
	#define TOP_4WD_BS_axlespacing      0.308
	#define TOP_4WD_DL_axlespacing      0.201
	#define FlagShip_4WD_BS_axlespacing 0.167
	#define FlagShip_4WD_DL_axlespacing 0.14

	//4WD wheel tire diameter series   (145 = C50C measured 2026-11-12)
	//四驱车轮胎直径
	#define  WheelDiameter_4WD_152  0.152f
	#define  WheelDiameter_4WD_225  0.225f
	#define	 WheelDiameter_4WD_250  0.250f
	#define  WheelDiameter_4WD_145  0.145f
	/* C50C 实测: 编码器 GMR_500 x 减速比 51 x 4 倍频 = 102000 计数/轮圈 */
	#define  EncoderPrecision_C50C_102000  102000
	
#endif
