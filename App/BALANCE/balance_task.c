#include "balance_task.h"
#ifdef CHASSIS_PWM_TEST
#include "pwm_test.h"
#endif
#if defined(FOUR_TURN_DEBUG) && (!defined(REAR_WHEEL_DEBUG) || !defined(_4WD_CAR) || !defined(FRONT_TEST_CHANNEL) || FRONT_TEST_CHANNEL != 6)
#error "FOUR_TURN_DEBUG requires the four-wheel debug configuration"
#endif
#if defined(FRONT_TEST_CHANNEL) && (!defined(REAR_WHEEL_DEBUG) || !defined(_4WD_CAR) || (FRONT_TEST_CHANNEL != 3 && FRONT_TEST_CHANNEL != 4 && FRONT_TEST_CHANNEL != 5 && FRONT_TEST_CHANNEL != 6))
#error "FRONT_TEST_CHANNEL requires REAR_WHEEL_DEBUG, _4WD_CAR and channel 3, 4, both (5), or four wheels (6)"
#endif
#if defined(REAR_WHEEL_DEBUG) && defined(_4WD_CAR)
#include "rear_debug.h"
#if defined(WHEEL_DIR_CALIB)
#error "REAR_WHEEL_DEBUG cannot be combined with WHEEL_DIR_CALIB"
#endif
#if defined(FOUR_TURN_DEBUG) || defined(ROS_STRAIGHT_DRIVE)
#include "steer_debug.h"
/* PS2 path state machine. */
static steer_debug_t g_steer_debug;
#if defined(ROS_STRAIGHT_DRIVE)
/* ROS path state machine, independent of the PS2 one. A mode switch clears
   both, so neither path can inherit the arm state or the wheel direction of
   the other one. */
static steer_debug_t g_ros_steer_debug;
#endif
#else
static rear_debug_t g_rear_debug;
#endif
#endif

//四驱调试期占空比上限. 满量程 FULL_DUTYCYCLE = 16800, 这里约 54%.
#define FOURWD_DUTY_LIMIT   9000

//与小车控制相关
ROBOT_CONTROL_t robot_control;

//与小车自检相关变量
ROBOT_SELFCHECK_t robot_check;

//机器人驻车模式相关变量
static ROBOT_PARKING_t park;

//增量式PI控制器
PI_CONTROLLER PI_MotorA,PI_MotorB,PI_MotorC,PI_MotorD,PI_Servo;

int PengZhuang=0;

//Safety: 统一安全停止.实现在 Turn_Off() 之后, Balance_task() 之前需要声明.
#ifdef CHASSIS_PWM_TEST
/* 架空测试: 交给 pwm_test 模块的 PWM 归零回调。只写零, 不读任何参数。
 * 模块用它在中途收到停止帧时把寄存器立即清零。 */
static void Pwm_Test_ZeroSink(void)
{
    Set_Pwm(0, 0, 0, 0, 0);
}
#endif

static void Safe_Stop(void);


/* [C50C] Forward declarations for the diagnostic points gathered at the end
 * of the control loop. The definitions live in the ROS speed loop section. */
static int  g_ros_speed_output;
static PI_CONTROLLER g_ros_speed_pi;
static int  g_ros_speed_diag_ready;
static int  g_ros_speed_diag_neutral;

//[诊断] 编码器原始脉冲观测区, 定义在 HARDWARE/encoder.c

AKM_SERVO_UNLOCK_t ServoState;//舵机非自锁控制器(异车型链已删,仅保留定义供诊断显示)
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/

void Balance_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
	
    while(1)
    {
        // This task runs at a frequency of 100Hz (10ms control once)
        //此任务以100Hz的频率运行（10ms控制一次）
        vTaskDelayUntil(&lastWakeTime, F2T(BALANCE_TASK_RATE));

#ifdef CHASSIS_PWM_TEST
        /* 架空测试: 到期与 USART3 帧失效都在这里判, 到期后输出归零. */
        Pwm_Test_Tick((uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS);
#endif

        //Time count is no longer needed after 30 seconds
        //时间计数，30秒后不再需要
        if(SysVal.Time_count<3000) SysVal.Time_count++;
		
        //and convert to transposition international units
        //获取编码器数据反馈、舵机数据反馈
        Get_Robot_FeedBack();


        //Function to Scan User Key Press Status
        //扫描用户按键状态
        UserKey_Scan( BALANCE_TASK_RATE );

		//Robot parking status monitoring, clearing residual motor control signals. \
		  Reduce power consumption, reduce motor noise.
		//机器人驻车状态监测,清除残余电机控制量.降低功耗,降低电机噪音.
		Robot_ParkingCheck();
		
		//机器人上电自检
        //等陀螺仪初始化完成后,检测机器人型号是否选择错误
        //When the gyroscope is initialized, check whether the robot model is selected incorrectly
        if(CONTROL_DELAY<SysVal.Time_count && SysVal.Time_count<CONTROL_DELAY+200)
        {
            /* 安全改动：原厂上电会以 0.2 m/s 前进 2 秒做车型自检，轮子会转。
             * 本车已取消电池电压检测（阈值降到 3.3V），FlagStop 恒为 0，
             * 这道自检不再被电压保护拦住，上电即转动有伤人风险，故强制给 0。 */
            Drive_Motor(0,0,0);//上电不做前进测试
			//TODO:自检需要测试各种车型自检
            robot_mode_check();  //Detection function //检测函数
        }
        else if(CONTROL_DELAY+200<SysVal.Time_count && SysVal.Time_count<CONTROL_DELAY+230)
        {
			robot_check.check_end = 1;
            Drive_Motor(0,0,0); //The stop forward control is completed //检测完成停止前进控制
			SysVal.LED_delay = 500;
        }
		
		//After the self-check is completed, obtain the control commands for the robot.
        //自检结束后，获取机器人的控制命令
        if(SysVal.Time_count>CONTROL_DELAY+230)
        {
			if( 0 == SysVal.SecurityLevel )//0为最高安全等级,带有机器人控制命令丢失保护
			{
				//TODO:监测丢失,不同控制方式,监测丢失的频率不同,以实测生效为准.
				//uart,ros,can,usb ps2,安卓app 可用,苹果app无法使用,航模需要其他实现
				//TODO:安全等级只对uart,ros,can生效. ps2，app，航模信号丢失需要无条件停止
				robot_control.command_lostcount++;
				if( robot_control.command_lostcount>BALANCE_TASK_RATE )
					robot_control.Vx = 0 , robot_control.Vy = 0 , robot_control.Vz = 0;
			}
			
			//正常模式控制命令
			{
				if      ( Get_Control_Mode(_APP_Control) )    Get_APPcmd();      //Handle the APP remote commands //处理APP遥控命令
				else if (Get_Control_Mode(_PS2_Control)  )    PS2_control();     //Handle PS2 controller commands //处理PS2手柄控制命令
				
				//CAN, Uart x control can directly get the 3 axis target speed, 
				//without additional processing
				//CAN、串口x 控制直接得到3轴目标速度，无须额外处理
				else    Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
			}
        }
		
		//深度自检模式,由用户需要时进入
		if( 1 == robot_check.DeepCheck && SysVal.Time_count>CONTROL_DELAY+300)
		{
			if( 1 == Deep_SelfCheck( BALANCE_TASK_RATE ) )
			{
				robot_check.DeepCheck = 0;
			}
			continue;
		}
		//Robot Operation Status Inspection. Inspection Items: Check if the voltage is too low, \
		if there are any self-test errors, and if the emergency stop switch has been pressed.   \
		The robot will not be allowed to be controlled if any of these conditions are not met.
		//机器人运行状态检测.检测内容：电压是否过低、自检是否出错、急停开关是否被按下.不符合条件时将不允许机器人被控制
		robot_control.FlagStop = Turn_Off();
		
		//机器人允许被控制
		if( 0 == robot_control.FlagStop )
		{
			//执行控制,根据车型、型号不同,进行不同的控制.
			ResponseControl();
		}
		else //失能控制.清空控制量.
		{
			//Safety fix: 原来调用 UnResponseControl(UN_LOCK), 它走平滑减速,
			//输出要若干拍才降到 0. 安全场景不能让减速过程产生残余位移,
			//改为 Safe_Stop() 立即把目标速度、PI 历史、PWM 占空比全部归零.
			Safe_Stop();
#if defined(REAR_WHEEL_DEBUG) && defined(_4WD_CAR)
#if defined(FOUR_TURN_DEBUG)
            (void)steer_debug_step(&g_steer_debug, 127, 127, 0,
                                   (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS);
#else
            (void)rear_debug_step(&g_rear_debug, 127, 0,
                                  (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS);
#endif
#endif
		}

#ifdef CHASSIS_PWM_TEST
		/* 架空测试输出收敛点. 位置选在这里是有意的:
		 *   1. 上面的 Deep_SelfCheck 分支执行时已 continue, 自检期间到不了本段;
		 *   2. 本段在 Turn_Off() 之后, robot_control.FlagStop 就是 Turn_Off 的结果,
		 *      已包含 EN==0、电压、errorflag、SoftWare_Stop;
		 *   3. 本段在 ResponseControl / Safe_Stop 之后, 本拍不覆盖它们的输出.
		 * 只有 allow 全真时才请求非零值; 否则回填零, 是否写 Set_Pwm 由下面决定. */
		{
			int     pwm_test[4];
			uint8_t pwm_test_allow =
				( 0 == robot_control.FlagStop )
				&& ( 0 == robot_check.DeepCheck )
				&& ( SysVal.Time_count > CONTROL_DELAY + 230 )
				&& Get_Control_Mode(_ROS_Control)
				&& ( 1 == EN );
			/* 竞态防护: 取序号 -> 算输出 -> 写 PWM -> 复核 -> 关窗口.
			 * USART3 停止帧(或任何 Pwm_Test_Stop)会加序号; 复核不过就一律
			 * 写零, 不写本次缓存值. 窗口内收到的停止帧还会由模块回调立即
			 * 清零寄存器, 所以停止帧到寄存器清零不依赖任务调度. */
			uint32_t pwm_test_seq = Pwm_Test_BeginWrite();
			uint32_t pwm_test_req = Pwm_Test_ControlOutput(pwm_test_allow, pwm_test);
			robot.MOTOR_A.Output = pwm_test[0];
			robot.MOTOR_B.Output = pwm_test[1];
			robot.MOTOR_C.Output = pwm_test[2];
			robot.MOTOR_D.Output = pwm_test[3];
			robot.MOTOR_A.Target = robot.MOTOR_B.Target = 0;
			robot.MOTOR_C.Target = robot.MOTOR_D.Target = 0;
			Pwm_Test_BindSink(Pwm_Test_ZeroSink);
			Set_Pwm(pwm_test[0], pwm_test[1], pwm_test[2], pwm_test[3], 0);
			if( pwm_test_req && Pwm_Test_Commit(pwm_test_seq, pwm_test) )
			{
				/* 复核通过: 本次写入仍然有效, 保持. */
			}
			else
			{
				/* 窗口内发生过期/停止/新命令, 本次数值作废, 立即归零. */
				Set_Pwm(0, 0, 0, 0, 0);
			}
			Pwm_Test_EndWrite();
		}
#endif
		
		

		//保存FLash参数,内容包含阿克曼参数、纠偏系数等
		uint8_t flash_check = 0;
		flash_check = FlashParam_Save( &appkey.ParamSaveFlag ); //保存Flash参数
		if( 1 == flash_check ) Buzzer_AddTask(1,100);      //保存成功,正常蜂鸣提示1秒
		else if( 1 < flash_check ) Buzzer_AddTask(10,11); //保存失败,快速蜂鸣10声警告

    }
}

/*-------------------------------- Functions Related to PI Controller ------------------------------------*/
/*--------------------------------           PI 控制器相关函数          ------------------------------------*/
/**************************************************************************
Functionality: Set PID Control Parameters - Configures the proportional (Kp) and integral (Ki) parameters for PID control.
Input Parameters: Kp parameter, Ki parameter.
Return Value: None.
Author: WHEELTEC
函数功能：设置PID控制的参数
入口参数：Kp参数、Ki参数
返回  值：无
作    者：WHEELTEC
**************************************************************************/
static void PI_SetParam(PI_CONTROLLER* p,int kp,int ki)
{
	//PID参数不可为负,也不可为0.
	if( kp > 0 ) 
	{
		p->kp = kp;
	}
	if( ki > 0 ) 
	{
		p->ki = ki;
	}
}

void Set_Robot_PI_Param(int kp,int ki)
{
	if( kp > 0 ) robot.V_KP = kp;
	if( ki > 0 ) robot.V_KI = ki;
	
	//设置四路pi参数
	PI_SetParam(&PI_MotorA,robot.V_KP,robot.V_KI);
	PI_SetParam(&PI_MotorB,robot.V_KP,robot.V_KI);
	PI_SetParam(&PI_MotorC,robot.V_KP,robot.V_KI);
	PI_SetParam(&PI_MotorD,robot.V_KP,robot.V_KI);
}

/**************************************************************************
Functionality: Incremental PI Controller Reset - Resets an incremental PI (Proportional-Integral) controller to its initial state.
Input Parameters: PI controller.
Return Value: None.
Author: WHEELTEC
函数功能：增量式PI控制器复位
入口参数：PI控制器
返回  值：无
作    者：WHEELTEC
**************************************************************************/
static void PI_Controller_Reset(PI_CONTROLLER *p)
{
	p->Bias = 0;
	p->LastBias = 0;
	p->Output = 0;
}


/**************************************************************************
Functionality: Incremental PI Control - Implements the control logic for an 
               incremental PI (Proportional-Integral) controller using feedback and target values.
Input Parameters: PI controller, feedback value, target value.
Return Value: Control output result.
Author: WHEELTEC
函数功能：增量式PI控制
入口参数：PI控制器,反馈值,目标值
返回  值：控制量结果输出
作    者：WHEELTEC
**************************************************************************/
static int Incremental_MOTOR(PI_CONTROLLER* p,float current,float target)
{
	//计算偏差
	p->Bias = target - current;
	
	//计算输出
	p->Output += p->kp * ( p->Bias - p->LastBias ) + p->ki * p->Bias;
	
	//输出限幅
	if( p->Output >   FULL_DUTYCYCLE ) p->Output =   FULL_DUTYCYCLE;
	if( p->Output < - FULL_DUTYCYCLE ) p->Output = - FULL_DUTYCYCLE;
	
	//保存本次偏差
	p->LastBias = p->Bias;
	
	//输出
	return p->Output;
}

/**************************************************************************
Functionality: PI Controller Clearing Function - Clears the output residue of a PI (Proportional-Integral) 
               controller when the robot is disabled, aiming to reduce power consumption and motor noise.
Input Parameters: PI controller.
Return Value: None.
Author: WHEELTEC
函数功能：PI控制器清除函数,在机器人禁止时清空输出残余，降低功耗以及电机的噪音
入口参数：PI控制器
返回  值：无
作    者：WHEELTEC
**************************************************************************/
static uint8_t PI_Clear_Output(PI_CONTROLLER* p)
{
	u8 state = 0;//清除标志位
	
	if( p->Output > 0 ) p->Output -- ;
	if( p->Output < 0 ) p->Output ++ ;
	
	//接近0时,直接置0
	if( p->Output < 2.0f && p->Output > -2.0f ) p->Output = 0 , state = 1;
	
	//返回清除结果. 0:未完成 1:清除完毕
	return state;
}

/*-------------------------------- Function for Inverse Kinematics Solution ------------------------------------*/
/*--------------------------------         各车型运动学逆解相关函数          ------------------------------------*/
/**************************************************************************
Function function: inverse kinematics solution of four-wheel drive robot
Entrance parameters: X-axis velocity (in m/s), Y-axis velocity (in m/s), Z-axis velocity (in rad/s)
Return value: None
Author  :WHEELTEC
函数功能：四驱机器人运动学逆解
入口参数：X轴速度(单位m/s)、Y轴速度(m/s)、Z轴速度(rad/s)
返回  值：无
作    者：WHEELTEC
**************************************************************************/
	
static void InverseKinematics_4wd(float Vx,float Vz)
{
    /* A/D: right side; B/C: left side, as measured on this chassis. */
    robot.MOTOR_A.Target = Vx-Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_B.Target = Vx+Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_C.Target = Vx+Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_D.Target = Vx-Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
}


/*-------------------------------- Control class related functions ------------------------------------*/
/*--------------------------------        控制类相关函数          ------------------------------------*/
//计算纠偏系数
static float wheelCoefficient(uint32_t diffparam,uint8_t isLeftWheel)
{
	if( 1 == isLeftWheel ) //左轮纠偏,对应50~100对应1.0~1.2倍的纠偏系数
	{
		if( diffparam>=50 )
			return 1.0f + 0.004f*(diffparam-50);
	}
	else //右轮纠偏,50~0对应1.0~1.2倍的纠偏系数
	{
		if( diffparam<=50 )
			return 1.0f + 0.004f*(50-diffparam);
	}
	
	return 1.0f;//不满足条件时,默认是1.
}

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：执行运动学逆解函数，根据三轴目标速度计算各车轮目标转速
入口参数：X、Y、Z三轴速度
返回  值：无
**************************************************************************/
static void Drive_Motor(float T_Vx,float T_Vy,float T_Vz)
{
	// Function to Limit Maximum Target Speed Input 
	//对输入的目标速度进行最大值限幅,单位m/s
	T_Vx = target_limit_float( T_Vx, -robot_control.limt_max_speed , robot_control.limt_max_speed );
	T_Vy = target_limit_float( T_Vy, -robot_control.limt_max_speed , robot_control.limt_max_speed );
	T_Vz = target_limit_float( T_Vz, -robot_control.limt_max_speed , robot_control.limt_max_speed );
	
	//Smoothly control the target speed input.
	//对输入的目标速度进行平滑控制
	{
		robot_control.smooth_Vx = Vel_SmoothControl(robot_control.smooth_Vx,T_Vx,robot_control.smooth_MotorStep);
		robot_control.smooth_Vy = Vel_SmoothControl(robot_control.smooth_Vy,T_Vy,robot_control.smooth_MotorStep);

		robot_control.smooth_Vz = Vel_SmoothControl(robot_control.smooth_Vz,T_Vz,robot_control.smooth_MotorStep);
	}

	// Call Inverse Kinematics Function for Corresponding Vehicle Model to Obtain Target Values for Wheels and Steering Angle
	//调用四驱车型的运动学逆解函数,获得各个轮子的目标值
	InverseKinematics_4wd(robot_control.smooth_Vx,robot_control.smooth_Vz);

	//纠偏系数计算
	float LeftWheelDiff = wheelCoefficient(robot_control.LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(robot_control.LineDiffParam,0);
	
	// Limit Final Wheel Speeds or Servo Travel After Kinematic Analysis.
	//经过运动学分析后输出各个轮子速度与舵机角度,下面对最终的轮速或舵机行程进行限幅.
	robot.MOTOR_A.Target = target_limit_float( robot.MOTOR_A.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
	robot.MOTOR_B.Target = target_limit_float( robot.MOTOR_B.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
	
	robot.MOTOR_C.Target = target_limit_float( robot.MOTOR_C.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
	robot.MOTOR_D.Target = target_limit_float( robot.MOTOR_D.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
	robot.MOTOR_A.Target*=RightWheelDiff;
	robot.MOTOR_B.Target*=LeftWheelDiff;
	robot.MOTOR_C.Target*=LeftWheelDiff;
	robot.MOTOR_D.Target*=RightWheelDiff;
}

/**************************************************************************
Function function: Control the corresponding function. After inverse kinematics solution, 
obtain the target values of each executing mechanism of the robot, and control the motion 
of the executing mechanism through PID control
Entrance parameters: None
Return value: None
Author: WHEELTEC
函数功能：控制相应函数.经过运动学逆解后得出机器人各个执行机构的目标值,通过PID控制执行机构运动
入口参数：无
返回  值：无
作    者：WHEELTEC
**************************************************************************/
#if defined(ROS_STRAIGHT_DRIVE)
/* ROS speed [m/s] to PWM duty: 1 m/s maps to 12000 counts.
   So 0.4 m/s is 4800 counts (cruise gear) and 0.8 m/s is 9600 counts
   (sport gear). The web console sends those two values, one value per gear.
   The turn channel uses the same conversion.
   steer_debug_step_duty() then limits each wheel to +-g_ros_drive_max_duty
   (runtime PWM limit, changed by CMD_PWM_LIMIT).
   This is a fixed conversion, not a calibrated wheel speed: the control is
   open-loop duty; real wheel speed must be read from the encoders.
   The firmware never derives a duty below 3360 for a non-zero command,
   because Set_Pwm lifts any non-zero duty to PWM_MIN_CMD. */
static int ros_drive_duty(float speed_mps)
{
	/* [C50C] 基准占空换算. 实测: 3360 占空约 0.23 m/s, 4000 占空约 0.30 m/s,
	   曲线近似线性, 斜率约 1 m/s = 12000 占空. 原系数 1000 让 0.4/0.8 两档
	   只差 278 占空, 两档跑出同一速度. 改为 12000 后 0.4 档给 4800,
	   0.8 档给 9600, 由 Set_Pwm 的 80% 上限(13440)截断. */
	return (int)(speed_mps * 12000.0f);
}
#endif

/* ---- ROS 速度闭环 ----
 * 反馈取四轮编码器换算出的 m/s（robot.MOTOR_x.Encoder），目标取 robot_control.Vx。
 * PI 输出单位是占空计数，和 ros_drive_duty() 的开环输出同量纲，
 * 因此可以原样喂给 steer_debug_step_duty()，起跳值与上限仍由 Set_Pwm 统一处理。
 * 增益沿用车型表里的 VEL_KP / VEL_KI，上电后必须显式装载一次，
 * 否则结构体清零会让 kp/ki 保持 0，闭环退化成纯开环直通。 */
#define ROS_SPEED_LOOP_READY_MS   500u      /* 上电后先开环一段时间再接管，避免静摩擦把积分顶满 */
#define ROS_SPEED_LOOP_INTEGRAL_MAX 12000  /* 积分项单独限幅，防止堵转时积到满占空 */
#define ROS_SPEED_LOOP_OUTPUT_MIN (16800 / 5)   /* 20% 起跳占空，见 Set_Pwm 的 PWM_MIN_CMD */
#define ROS_SPEED_TRIM_MAX   800                /* 闭环修正量限幅，约 ±4.8% 占空 */
#define ROS_SPEED_TRIM_K     0.5f               /* 每 1 mm/s 误差修正 0.5 占空 */

/* g_ros_speed_output / g_ros_speed_pi are declared near the top of this file
   so the end-of-loop diagnostic block can read them. */
static uint32_t g_ros_speed_loop_start = 0;
static int  g_ros_speed_loop_loaded = 0;
static uint32_t g_ros_not_ready_since = 0;

static void ros_speed_loop_init(void)
{
	g_ros_speed_loop_loaded = 1;
	g_ros_speed_loop_start = (uint32_t)xTaskGetTickCount();
	g_ros_speed_pi.Bias = 0.0f;
	g_ros_speed_pi.LastBias = 0.0f;
	g_ros_speed_pi.Output = 0;
	g_ros_speed_pi.kp = robot.V_KP > 0 ? robot.V_KP : VEL_KP;
	g_ros_speed_pi.ki = robot.V_KI > 0 ? robot.V_KI : VEL_KI;
	g_ros_speed_output = 0;
}


static float ros_measured_forward_mps(void)
{
	/* A/B 为前侧一对，C/D 为后侧一对；取两侧平均当作车体前进速度。
	   [C50C] 返回毫米量级。VEL_KP/VEL_KI(=300) 是原厂按 mm/s 调的增量，
	   若直接送 m/s (0.3) 进去，微分项会在一拍内把输出推到限幅，导致正反方向抖动。 */
	return 1000.0f * 0.5f * (robot.MOTOR_A.Encoder + robot.MOTOR_B.Encoder);
}




static void ros_speed_loop(int ros_ready)
{
	if( !ros_ready || robot_control.Vx == 0.0f )
	{
		/* [C50C] 不留残余输出：增量式 PI 残留会让电机一通电就被满占空驱动。 */
		g_ros_speed_output = 0;
		g_ros_speed_pi.Output = 0;
		g_ros_speed_pi.Bias = 0.0f;
		g_ros_speed_pi.LastBias = 0.0f;
		return;
	}
	/* [C50C] 基准占空加小幅修正，替代增量式 PI。
	   基准按档位给：0.4 m/s 得 4800，0.8 m/s 得 9600（1 m/s = 12000 占空）。
	   修正量按编码器测到的车速做比例调节，限在 +-ROS_SPEED_TRIM_MAX 内。
	   这样输出不会撞占空上限，也不会因积分饱和来回换向。
	   修正只用比例，不用积分：静摩擦与负载变化靠基准覆盖。 */
	int ros_base = ros_drive_duty(robot_control.Vx);
	float ros_err = 1000.0f * robot_control.Vx - ros_measured_forward_mps();
	int ros_trim = (int)(ros_err * ROS_SPEED_TRIM_K);

	if( ros_trim >  ROS_SPEED_TRIM_MAX ) ros_trim =  ROS_SPEED_TRIM_MAX;
	if( ros_trim < -ROS_SPEED_TRIM_MAX ) ros_trim = -ROS_SPEED_TRIM_MAX;

	g_ros_speed_pi.Bias = ros_err;
	g_ros_speed_pi.LastBias = ros_err;
	g_ros_speed_pi.Output = ros_base + ros_trim;
	g_ros_speed_output = g_ros_speed_pi.Output;
}


static void ResponseControl(void)
{
	//四驱车型执行路径(异车型分支已删)

#if defined(REAR_WHEEL_DEBUG)
    {
        uint32_t tick = (uint32_t)xTaskGetTickCount();
        uint32_t ms = tick * portTICK_PERIOD_MS;
#if defined(ROS_STRAIGHT_DRIVE)
        static uint8_t last_drive_mode;
        uint8_t drive_mode = Get_Control_Mode(_ROS_Control) ? _ROS_Control :
                             (Get_Control_Mode(_PS2_Control) ? _PS2_Control : 0);
        if (drive_mode != last_drive_mode) {
            /* Mode switch: clear both machines. A cleared machine is unarmed
               and holds zero output until its own input stays neutral for
               ARM_NEUTRAL_MS, so the entered source restarts from the neutral
               wait instead of resuming the state of the previous source. */
            static const steer_debug_t cleared;
            g_ros_steer_debug = cleared;
            g_steer_debug = cleared;
            last_drive_mode = drive_mode;
        }
        if (drive_mode == _ROS_Control) {
        /* The four wheel output comes from robot_control.Vx/Vz, never from the pad.
           When ros_ready is false the duties are forced to zero, so the last accepted
           speed cannot survive a mode switch or a stop request. */
        int ros_ready = SysVal.HardWare_Ver == V1_0 &&
                        Get_Control_Mode(_ROS_Control) &&
                        (uint32_t)(tick - robot_control.ros_cmd_tick) <= pdMS_TO_TICKS(ROS_CMD_TIMEOUT_MS) &&
                        !robot_check.DeepCheck && SysVal.Time_count > CONTROL_DELAY + 230;
        /* ROS closed loop: speed PI on encoder feedback, then split to wheels.
           Feedback is m/s, PI output is duty counts; start-up floor and ceiling
           are still applied by Set_Pwm. */
        ros_speed_loop(ros_ready);
        /* 中性判据用指令是否为 0，不用占空。闭环占空在运动时永不为 0，
           用它判中性会让 armed 一次失效后永久锁死。 */
        int ros_cmd_neutral = (robot_control.Vx == 0.0f) && (robot_control.Vz == 0.0f);
        g_ros_speed_diag_ready = ros_ready;
        g_ros_speed_diag_neutral = ros_cmd_neutral;
        steer_output_t drive = steer_debug_step_duty(&g_ros_steer_debug,
                                                    g_ros_speed_output,
                                                    ros_ready ? ros_drive_duty(robot_control.Vz) : 0,
                                                    ros_cmd_neutral,
                                                    ros_ready, ms);
        robot.MOTOR_A.Output = drive.right;
        robot.MOTOR_B.Output = -drive.left;
        robot.MOTOR_C.Output = drive.left;
        robot.MOTOR_D.Output = -drive.right;
        robot.MOTOR_A.Target = (float)drive.right / 3000.0f;
        robot.MOTOR_B.Target = (float)drive.left / 3000.0f;
        robot.MOTOR_C.Target = robot.MOTOR_D.Target = 0;
        /* [C50C 2026-11-15] Driver board reinstall swapped the rear wheel
           binding: measured per-channel with pwm_test frames (dir=0):
           chA=left-rear(reverse) chB=right-rear(reverse) chC=left-front(fwd)
           chD=right-front(fwd). Pre-reinstall calibration comment at the PID
           path said A=right-rear B=left-rear C=left-front D=right-front, i.e.
           the two rear motors moved between chA and chB. Negate A and B here
           (both the ROS branch and the PS2 FOUR_TURN_DEBUG branch). */
        Set_Pwm(drive.right, -drive.left, drive.left, -drive.right, 0);
        } else
#endif
#if defined(FOUR_TURN_DEBUG)
        {
        int ready = SysVal.HardWare_Ver == V1_0 &&
                    Get_Control_Mode(_PS2_Control) &&
                    !robot_check.DeepCheck && SysVal.Time_count > CONTROL_DELAY + 230 &&
                    g_rear_ps2_valid && (uint32_t)(tick - g_rear_ps2_tick) <= pdMS_TO_TICKS(150);
        uint8_t forward_axis = ready ? Classic_PS2Gamepad.LY : 127;
        uint8_t turn_axis = ready ? Classic_PS2Gamepad.RX : 127;
        steer_output_t drive = steer_debug_step(&g_steer_debug, forward_axis, turn_axis, ready, ms);
        robot.MOTOR_A.Output = drive.right;
        robot.MOTOR_B.Output = -drive.left;
        robot.MOTOR_C.Output = drive.left;
        robot.MOTOR_D.Output = -drive.right;
        robot.MOTOR_A.Target = (float)drive.right / 3000.0f;
        robot.MOTOR_B.Target = (float)drive.left / 3000.0f;
        robot.MOTOR_C.Target = robot.MOTOR_D.Target = 0;
        /* [C50C 2026-11-15] Driver board reinstall swapped the rear wheel
           binding: measured per-channel with pwm_test frames (dir=0):
           chA=left-rear(reverse) chB=right-rear(reverse) chC=left-front(fwd)
           chD=right-front(fwd). Pre-reinstall calibration comment at the PID
           path said A=right-rear B=left-rear C=left-front D=right-front, i.e.
           the two rear motors moved between chA and chB. Negate A and B here
           (both the ROS branch and the PS2 FOUR_TURN_DEBUG branch). */
        Set_Pwm(drive.right, -drive.left, drive.left, -drive.right, 0);
        }
#else
        uint8_t axis = ready ? Classic_PS2Gamepad.LY : 127;
        int command = rear_debug_step(&g_rear_debug, axis, ready, ms);
#if defined(FRONT_TEST_CHANNEL)
        robot.MOTOR_A.Output = robot.MOTOR_B.Output = FRONT_TEST_CHANNEL == 6 ? command : 0;
        robot.MOTOR_A.Target = robot.MOTOR_B.Target = FRONT_TEST_CHANNEL == 6 ? (float)command / 3000.0f : 0;
        robot.MOTOR_C.Output = (FRONT_TEST_CHANNEL == 3 || FRONT_TEST_CHANNEL == 5 || FRONT_TEST_CHANNEL == 6) ? command : 0;
        robot.MOTOR_D.Output = (FRONT_TEST_CHANNEL == 4 || FRONT_TEST_CHANNEL == 5 || FRONT_TEST_CHANNEL == 6) ? -command : 0;
        robot.MOTOR_C.Target = robot.MOTOR_D.Target = 0;
        Set_Pwm(-robot.MOTOR_A.Output, robot.MOTOR_B.Output, robot.MOTOR_C.Output, robot.MOTOR_D.Output, 0);
#else
        robot.MOTOR_A.Output = command;
        robot.MOTOR_B.Output = command;
        robot.MOTOR_C.Output = 0;
        robot.MOTOR_D.Output = 0;
        robot.MOTOR_A.Target = robot.MOTOR_B.Target = (float)command / 3000.0f;
        robot.MOTOR_C.Target = robot.MOTOR_D.Target = 0;
        Set_Pwm(-command, command, 0, 0, 0);
#endif
#endif
    }
    return;
#endif
	
#ifdef WHEEL_DIR_CALIB
	//逐路标定: 四路轮流转, 每路正输出 3 秒, 不受摇杆和零目标守门影响.
	{
		static uint32_t calib_cnt = 0;
		uint32_t calib_ch = (calib_cnt++ / 300u) % 4u;
		int pA = 0, pB = 0, pC = 0, pD = 0;
		if( calib_ch == 0 ) pA = 1500;
		if( calib_ch == 1 ) pB = 1500;
		if( calib_ch == 2 ) pC = 1500;
		if( calib_ch == 3 ) pD = 1500;
		Set_Pwm( pA, pB, pC, pD, 0 );
	}
	return;
#endif
	
	//Safety fix: 四路Target全为0时, 不跑PI, 直接输出0.
	//原因: 编码器上电野值与目标值0相互作用会让增量式PI的Output冲高,
	//之后只能靠PI_Clear_Output每拍减1慢清(约90秒), 期间电机自转.
	if( 0.0f == robot.MOTOR_A.Target && 0.0f == robot.MOTOR_B.Target
	 && 0.0f == robot.MOTOR_C.Target && 0.0f == robot.MOTOR_D.Target )
	{
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
		robot.MOTOR_A.Output = 0;
		robot.MOTOR_B.Output = 0;
		robot.MOTOR_C.Output = 0;
		robot.MOTOR_D.Output = 0;
		Set_Pwm( 0 , 0 , 0 , 0 , 0 );
		return;
	}
	
	
	robot.MOTOR_A.Output = Incremental_MOTOR(&PI_MotorA,robot.MOTOR_A.Encoder,robot.MOTOR_A.Target);
	robot.MOTOR_B.Output = Incremental_MOTOR(&PI_MotorB,robot.MOTOR_B.Encoder,robot.MOTOR_B.Target);
	robot.MOTOR_C.Output = Incremental_MOTOR(&PI_MotorC,robot.MOTOR_C.Encoder,robot.MOTOR_C.Target);
	robot.MOTOR_D.Output = Incremental_MOTOR(&PI_MotorD,robot.MOTOR_D.Encoder,robot.MOTOR_D.Target);
	
	//限占空比: 9000 / 16800 约为 54%.
	if( robot.MOTOR_A.Output >   FOURWD_DUTY_LIMIT ) robot.MOTOR_A.Output =   FOURWD_DUTY_LIMIT;
	if( robot.MOTOR_A.Output < - FOURWD_DUTY_LIMIT ) robot.MOTOR_A.Output = - FOURWD_DUTY_LIMIT;
	if( robot.MOTOR_B.Output >   FOURWD_DUTY_LIMIT ) robot.MOTOR_B.Output =   FOURWD_DUTY_LIMIT;
	if( robot.MOTOR_B.Output < - FOURWD_DUTY_LIMIT ) robot.MOTOR_B.Output = - FOURWD_DUTY_LIMIT;
	if( robot.MOTOR_C.Output >   FOURWD_DUTY_LIMIT ) robot.MOTOR_C.Output =   FOURWD_DUTY_LIMIT;
	if( robot.MOTOR_C.Output < - FOURWD_DUTY_LIMIT ) robot.MOTOR_C.Output = - FOURWD_DUTY_LIMIT;
	if( robot.MOTOR_D.Output >   FOURWD_DUTY_LIMIT ) robot.MOTOR_D.Output =   FOURWD_DUTY_LIMIT;
	if( robot.MOTOR_D.Output < - FOURWD_DUTY_LIMIT ) robot.MOTOR_D.Output = - FOURWD_DUTY_LIMIT;
	
	//四驱方向: 方向电平已统一为 m>0 -> 01, 各路符号由标定得出.
	//逐路标定结果: A=右后(正输出反转) B=左后(正) C=左前(正) D=右前(正输出反转).
	//故 A、D 取反, B、C 保持.
	Set_Pwm(-robot.MOTOR_A.Output, robot.MOTOR_B.Output, robot.MOTOR_C.Output,-robot.MOTOR_D.Output,0);
	
}


/**************************************************************************
Functionality: Non-Control Response Function - Executes when the robot encounters an error, prohibiting the robot's movement.
Input Parameters: Set the robot's stop mode: 0: Set to unclamp axes, allowing the robot to be freely pushed. 
                  1: Set to clamp axes, locking all actuators.
Return Value: None
Author: WHEELTEC
函数功能：非控制响应函数,当机器人存在报错时,将执行此函数.禁止机器人的运动
入口参数：设置机器人停止运动的方式: 0:设置为解轴,机器人可以随意推动  1:设置为锁轴,机器人锁住所有执行机构
返回  值：无
作    者：WHEELTEC
**************************************************************************/
static void UnResponseControl(uint8_t mode)
{
	//数据有效性校验
	if(mode!=UN_LOCK && mode!=LOCK) mode = UN_LOCK;
	
	//解轴失能
	if(mode==UN_LOCK)
	{
		//复位PI控制器,消除积累
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
		
		//发送0停止电机的转动
		Set_Pwm( 0 , 0 , 0 , 0 , 0 );
	}
	
	//锁轴失能,无法推动小车
	else if( mode==LOCK )
	{
		Drive_Motor(0,0,0);
		ResponseControl();
	}
}

/**************************************************************************
Functionality: Send Control Values to Actuators - Distributes control values to each actuator (A, B, C, D wheels and servo).
Input Parameters: Control value for A wheel motor, control value for B wheel motor, control value for C wheel motor, 
                   control value for D wheel motor, control value for servo.
Return Value: None.
Author: WHEELTEC
函数功能：下发控制量到各个执行机构
入口参数：A轮电机控制量、B轮电机控制量、C轮电机控制量、D轮电机控制量、舵机控制量
返回  值：无
作    者：WHEELTEC
**************************************************************************/
/* [C50C] PWM 起跳值/上限. 周期 16800 计数:
   PWM_MIN_CMD 3360 = 20% 占空比(低于此值电机克服不了静摩擦)
   PWM_MAX_CMD 13440 = 80% 占空比(2026-11-13 用户要求从 50% 提高到 80%) */
#define PWM_PERIOD_COUNTS   16800
/* 起跳值必须保持 20%: 低于此值时电机克服不了自身摩擦与地面摩擦, 轮子转不动. 不要为了测开环速度表而降低它. */
#define PWM_MIN_CMD         (PWM_PERIOD_COUNTS / 5)
#define PWM_MAX_CMD         (PWM_PERIOD_COUNTS * 4 / 5)

static void Set_Pwm(int m_a,int m_b,int m_c,int m_d,int servo)
{
#if defined(REAR_WHEEL_DEBUG) && defined(_4WD_CAR)
#if defined(FRONT_TEST_CHANNEL)
#if FRONT_TEST_CHANNEL != 6
    m_a = 0;
    m_b = 0;
#endif
#if FRONT_TEST_CHANNEL == 3
    m_d = 0;
#elif FRONT_TEST_CHANNEL == 4
    m_c = 0;
#endif
#else
    m_c = 0;
    m_d = 0;
#endif
#endif

		
		//电机控制
		if( m_a > 0 ) AIN1=0,AIN2=1;
#if defined(REAR_WHEEL_DEBUG) && defined(_4WD_CAR)
		else if (m_a < 0) AIN1=1,AIN2=0;
#else
		else          AIN1=1,AIN2=0;
#endif
		
		if(SysVal.HardWare_Ver==V1_0)
		{
			if( m_b > 0 ) V1_0_BIN1=0,V1_0_BIN2=1;
#if defined(REAR_WHEEL_DEBUG) && defined(_4WD_CAR)
			else if (m_b < 0) V1_0_BIN1=1,V1_0_BIN2=0;
#else
			else          V1_0_BIN1=1,V1_0_BIN2=0;
#endif
		}
		else if( SysVal.HardWare_Ver>=V1_1 )
		{
			if( m_b > 0 ) BIN1=0,BIN2=1;
			else          BIN1=1,BIN2=0;
		}
	
		//四驱方向修正: 原代码 C/D 的正向电平与 A/B 相反(10 vs 01),
		//导致同样符号的 PWM 让轮子转向相反. 这里统一为 m>0 -> 01.
		if( m_c > 0 ) CIN1=0,CIN2=1;
#if defined(FRONT_TEST_CHANNEL)
		else if (m_c < 0) CIN1=1,CIN2=0;
#else
		else          CIN1=1,CIN2=0;
#endif
		
		if( m_d > 0 ) DIN1=0,DIN2=1;
#if defined(FRONT_TEST_CHANNEL)
		else if (m_d < 0) DIN1=1,DIN2=0;
#else
		else          DIN1=1,DIN2=0;
#endif
		
		//[C50C] 起跳值与上限: 低速摩擦带不动, 非零指令抬到 3360(20%); 超过 8400(50%) 截断.
		{
			/* [C50C] m_a..m_d 为 int, abs() 返回 int; 原写 v 为 int 时 8000 应原样通过,
			   但实测两档都停在 3360, 故改用 long 并显式打印比较结果. */
			long v;
			v = abs(m_a); if( v != 0 ) v = ( v < PWM_MIN_CMD ) ? PWM_MIN_CMD : v; if( v > PWM_MAX_CMD ) v = PWM_MAX_CMD; PWMA = v;
			v = abs(m_b); if( v != 0 ) v = ( v < PWM_MIN_CMD ) ? PWM_MIN_CMD : v; if( v > PWM_MAX_CMD ) v = PWM_MAX_CMD; PWMB = v;
			v = abs(m_c); if( v != 0 ) v = ( v < PWM_MIN_CMD ) ? PWM_MIN_CMD : v; if( v > PWM_MAX_CMD ) v = PWM_MAX_CMD; PWMC = v;
			v = abs(m_d); if( v != 0 ) v = ( v < PWM_MIN_CMD ) ? PWM_MIN_CMD : v; if( v > PWM_MAX_CMD ) v = PWM_MAX_CMD; PWMD = v;
		}
		
}


/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
static u8 Turn_Off(void)
{
    u8 temp = 0;
	static uint8_t saveflag = 0;
	
	//电压不满足要求：电压低于20V,且不是自动回充状态.(若是进入自动回充状态,则忽略电压)
	if( robot.voltage < 3.3f ) temp = 1;

	//急停开关被按下
	if( EN==0 ) temp = 1;
	
	//机器人自检出错
	if( robot_check.errorflag==1 ) temp = 1;

	//机器人通过软件设置了急停
	if( robot_control.SoftWare_Stop==1 ) temp = 1;
	
	//小车由失能变为使能的过程,清空目标速度.
	if(temp==0 && saveflag==1)
	{
		robot_control.Vx = 0;
		robot_control.Vy = 0;
		robot_control.Vz = 0;
		robot_control.smooth_Vx = 0;
		robot_control.smooth_Vy = 0;
		robot_control.smooth_Vz = 0;
	}
	saveflag = temp;
	
	return temp;
}

/**************************************************************************
Function: Unified safe stop. Clears every motion quantity in one pass.
Input   : none
Output  : none
函数功能：统一安全停止.一次把所有运动量清零
入口参数：无
返回  值：无
说明：与 UnResponseControl() 的区别:
      1.UnResponseControl 走平滑减速, 输出逐拍递减, 需要若干拍才归零.
      2.Safe_Stop 立即把目标速度、PI 历史、PWM 占空比全部归零.
      安全场景要用 2, 不能让减速过程产生残余位移.
**************************************************************************/
static void Safe_Stop(void)
{
	//1.目标速度与平滑值
	robot_control.Vx = 0;
	robot_control.Vy = 0;
	robot_control.Vz = 0;
	robot_control.smooth_Vx = 0;
	robot_control.smooth_Vy = 0;
	robot_control.smooth_Vz = 0;
	robot_control.command_lostcount = 0;

#ifdef CHASSIS_PWM_TEST
	//1b.架空测试: 停车路径优先, 任何固定占空比都在这里归零.
	Pwm_Test_Stop();
#endif

	//2.四个轮子的目标与输出
	robot.MOTOR_A.Target = 0;  robot.MOTOR_A.Output = 0;
	robot.MOTOR_B.Target = 0;  robot.MOTOR_B.Output = 0;
	robot.MOTOR_C.Target = 0;  robot.MOTOR_C.Output = 0;
	robot.MOTOR_D.Target = 0;  robot.MOTOR_D.Output = 0;

	//3.清 PI 历史, 防止下次使能时积分项继续累加产生突跳
	PI_Controller_Reset(&PI_MotorA);
	PI_Controller_Reset(&PI_MotorB);
	PI_Controller_Reset(&PI_MotorC);
	PI_Controller_Reset(&PI_MotorD);

	//4.PWM 占空比清零
	PWMA = 0;
	PWMB = 0;
	PWMC = 0;
	PWMD = 0;

	//5.清驻车状态, 防止沿旧路径递减
	park.start_clear = 0;
	park.clear_state = (1<<7);
}

/**************************************************************************
Function: Processes the command sent by APP through usart x
Input   : none
Output  : none
函数功能：对APP通过串口x发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
static void Get_APPcmd(void)
{
	/*  APP摇杆与速度方向
	     A
	     ^  +Vx       ^ +Vz
   H     |       B    ）
	     |
   G----------> -Vy  C
	     |
   F     |    D
	     E
	*/
	
	short m_sign = 1;         //符号变量
	float base_vz= PI/4.0f;   //Z轴速度基准
	
	//四驱小车: 方向键映射到 Vx/Vz
	switch( appkey.DirectionFlag ) 
	{
		case 1: robot_control.Vx = robot_control.rc_speed , robot_control.Vz = 0               ; break; //A
		case 2: robot_control.Vx = robot_control.rc_speed , robot_control.Vz =-base_vz         ; break; //B
		case 3: robot_control.Vx = 0 ,                      robot_control.Vz =-base_vz         ; break; //C
		case 4: robot_control.Vx =-robot_control.rc_speed , robot_control.Vz = (base_vz)*m_sign; break; //D
		case 5: robot_control.Vx =-robot_control.rc_speed , robot_control.Vz = 0               ; break; //E
		case 6: robot_control.Vx =-robot_control.rc_speed , robot_control.Vz =-(base_vz)*m_sign; break; //F
		case 7: robot_control.Vx = 0 ,                      robot_control.Vz = base_vz         ; break; //G
		case 8: robot_control.Vx = robot_control.rc_speed , robot_control.Vz = base_vz         ; break; //H
		
		default : robot_control.Vx=0;robot_control.Vy=0;robot_control.Vz=0; break;
	}
	
	

	//单位转换 mm/s -> m/s
	robot_control.Vx = robot_control.Vx/1000.0f;
	robot_control.Vy = robot_control.Vy/1000.0f;
	robot_control.Vz = robot_control.Vz * ( robot_control.rc_speed/500.0f );//Z轴速度根据遥控速度调整.

    //Control target value is obtained and kinematics analysis is performed
    //得到控制目标值，进行运动学分析
    Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
}


/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
/* USB Host 整链移除, 仅保留经典GPIO时序PS2手柄回调. */
//经典PS2手柄回调函数,非USB款
void Classic_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	//按下start按键
	if( keyid == PS2KEY_START && event == GamePadKeyEvent_SINGLECLICK )
		Classic_PS2Gamepad.StartFlag = 1;

	//手柄加减速
	else if( keyid == PS2KEY_L2 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		robot_control.rc_speed -= 50;
	else if( keyid == PS2KEY_L1 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		robot_control.rc_speed += 50;

	if( robot_control.rc_speed < 0 ) robot_control.rc_speed = 0;
}


static void PS2_control(void)
{
	float LX=127,LY=127,RX=127;
	float ThrottleTri = 255;
	float base_vz = PI/4.0f ;//Z轴速度基准

	//前进摇杆
	LY = Classic_PS2Gamepad.LY - 127;

	//左右横移
	LX = 127 - Classic_PS2Gamepad.LX;

	//顺逆时针
	RX = 127 - Classic_PS2Gamepad.RX;

	//摇杆微小幅度过滤
	if( fabs(LY)<20 ) LY = 0;
	if( fabs(LX)<20 ) LX = 0;
	if( fabs(RX)<20 ) RX = 0;


	robot_control.Vx = (LY/127.0f) * robot_control.rc_speed * (ThrottleTri/255.0f);
	robot_control.Vy = (LX/127.0f) * robot_control.rc_speed;
	robot_control.Vz = base_vz * (RX/127.0f) * ( robot_control.rc_speed/500.0f );
	
	if( robot_control.Vx<0 ) robot_control.Vz = -robot_control.Vz;

    //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s
    robot_control.Vx = robot_control.Vx/1000.0f;
	robot_control.Vy = robot_control.Vy/1000.0f;

    //Control target value is obtained and kinematics analysis is performed
    //得到控制目标值，进行运动学分析
    Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
	
		
//	//TODO:适配PS2控制
//    float LX,LY,RY;
//    int Yuzhi=20; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
//	
//	float base_vz = PI/4.0f ;//Z轴速度基准
//	
//	//针对 usb有线ps2手柄 数字模式控制转换
//	if( SysVal.HardWare_Ver==V1_1 )
//	{
//		if( ps2_type == Normal_PS2 )
//		{
//			if( ps2_val.RX==128 && ps2_val.RY==128 )
//			{
//				if( Read_PS2_KEY(R_4PINK )==PS2_KEY_ON )  ps2_val.RX  = 0;
//				if( Read_PS2_KEY(R_2RED  )==PS2_KEY_ON )  ps2_val.RX = 255;
//				if( Read_PS2_KEY(R_1GREEN)==PS2_KEY_ON )  ps2_val.RY = 0;
//				if( Read_PS2_KEY(R_3BLUE )==PS2_KEY_ON )  ps2_val.RY = 255;
// 			}
//		}
//	}
//	
//    //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
//    //128为中值。PS2坐标系与ROS坐标系对X、Y的定义不一样
//    LY=-(ps2_val.LX-128);//取值[-128,128]
//    LX=-(ps2_val.LY-128);
//    RY=-(ps2_val.RX-128);

//    //Ignore small movements of the joystick //忽略摇杆小幅度动作
//    if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
//    if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
//    if(RY>-Yuzhi&&RY<Yuzhi)RY=0;
//	
//	static uint8_t low_fre = 0;
//	if(++low_fre==10)//降低按键扫描频率
//	{
//		low_fre = 0;
//		//PS2手柄,左扳机1号可加速,2号可减速
//			 if( Read_PS2_KEY(L1_KEY)==PS2_KEY_ON ) robot_control.rc_speed+=20;
//		else if( Read_PS2_KEY(L2_KEY)==PS2_KEY_ON ) robot_control.rc_speed-=20;
//		
//		//速度最大最小值限制
//		robot_control.rc_speed = target_limit_float(robot_control.rc_speed,0,robot_control.limt_max_speed*1000);
//	}

//	#if defined AKM_CAR
//		base_vz = angle_to_rad(30.0f);//阿克曼Z轴基准角度
//	#endif
//	
//    //Handle PS2 controller control commands
//    //对PS2手柄控制命令进行处理
//	robot_control.Vx = (LX/128.0f) * robot_control.rc_speed;
//	robot_control.Vy = (LY/128.0f) * robot_control.rc_speed;
//	robot_control.Vz = base_vz * (RY/128.0f) * ( robot_control.rc_speed/500.0f );
//	
//	#if !defined AKM_CAR
//		if( robot_control.Vx<0 ) robot_control.Vz = -robot_control.Vz;
//	#endif

//    //Unit conversion, mm/s -> m/s
//    //单位转换，mm/s -> m/s
//    robot_control.Vx = robot_control.Vx/1000.0f;
//	robot_control.Vy = robot_control.Vy/1000.0f;

//    //Control target value is obtained and kinematics analysis is performed
//    //得到控制目标值，进行运动学分析
//    Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
}

/**************************************************************************
Functionality: Robot Self-Check Function - Performs a self-check on the robot to verify if there are any errors in the wiring of motors, encoders, and drivers.
Input Parameters: None.
Return Value: None.
Author: WHEELTEC
函数功能： 机器人自检函数,检查机器人电机、编码器、驱动器是否存在错误接线
入口参数：无
返回  值：无
作    者：WHEELTEC
**************************************************************************/
static void robot_mode_check(void)
{
	//TODO:自检程序,同向电机接反难判断.
	
	#define OVERLOAD_PWM 8000 //超载PWM预警值
	#define ENCODER_LOST_PWM 5500 //编码器未接时,PWM报错阈值
	
    static u8 init=0;//初始化
    if( 0 == init ) 
	{
		ROBOT_SELFCHECK_t_Init(&robot_check);
		init=1;
	}

    if( EN==1 && robot_check.errorflag ==0 ) //保留可以使用急停开关跳过自检的功能
    {

			static int last_a=0,last_b=0,last_c=0,last_d=0;
			static u8 err_time = 0;
			
			//1、任意车轮反转 1/4 圈 认为是编码器线或驱动线接反,或者是车型选错.
			if( robot_check.check_a < -robot.HardwareParam.Encoder_precision/4  || \
				robot_check.check_b < -robot.HardwareParam.Encoder_precision/4  || \
				robot_check.check_c < -robot.HardwareParam.Encoder_precision/4  || \
				robot_check.check_d < -robot.HardwareParam.Encoder_precision/4  ) 
			{
				robot_check.errorflag = 1;
				Buzzer_AddTask(1,20);
			}
			
			//2、任意电机PWM输出到达阈值,对应的编码器数据很小(轮子转动不到12°)或无数据,则认为编码器线未接
			if( ( abs( robot.MOTOR_A.Output ) > ENCODER_LOST_PWM && robot_check.check_a < robot.HardwareParam.Encoder_precision/30 ) || \
				( abs( robot.MOTOR_B.Output ) > ENCODER_LOST_PWM && robot_check.check_b < robot.HardwareParam.Encoder_precision/30 ) || \
				( abs( robot.MOTOR_C.Output ) > ENCODER_LOST_PWM && robot_check.check_c < robot.HardwareParam.Encoder_precision/30 ) || \
				( abs( robot.MOTOR_D.Output ) > ENCODER_LOST_PWM && robot_check.check_d < robot.HardwareParam.Encoder_precision/30 )  )
			{
				robot_check.errorflag = 1;
				Buzzer_AddTask(2,20);
			}
			
			//3、任意电机PWM输出超过更大阈值,则认为小车过载
			if( abs( robot.MOTOR_A.Output ) > OVERLOAD_PWM || abs( robot.MOTOR_B.Output ) > OVERLOAD_PWM || \
				abs( robot.MOTOR_C.Output ) > OVERLOAD_PWM || abs( robot.MOTOR_D.Output ) > OVERLOAD_PWM	)
			{
				robot_check.errorflag = 1;
				Buzzer_AddTask(3,20);
			}
			
			//4、出现多次累计值倒退,除了1中的异常,也可能是多种错误累计叠加的表现.
			if( abs(robot_check.check_a) < abs(last_a) - 50 || abs(robot_check.check_b) < abs(last_b) - 50 || abs(robot_check.check_c) < abs(last_c) - 50|| abs(robot_check.check_d) < abs(last_d) - 50 )
			{
				err_time++;
				if( err_time > 20 )  robot_check.errorflag = 1 , Buzzer_AddTask(4,20);	
			}
		
			last_a = robot_check.check_a;
			last_b = robot_check.check_b;
			last_c = robot_check.check_c;
			last_d = robot_check.check_d;
    }
}

/**************************************************************************
Functionality: Read Feedback Values - Retrieves feedback values from each actuator of the robot.
Input Parameters: None.
Return Value: None.
Author: WHEELTEC
函数功能：读取机器各个执行机构的反馈值
入口参数：无
返回  值：无
作    者：WHEELTEC
**************************************************************************/
static void Get_Robot_FeedBack(void)
{
    //Retrieves the original data of the encoder
    //获取编码器的原始数据
    short Encoder_A_pr,Encoder_B_pr;
	
	//编码器原始值,读取后清空.
    Encoder_A_pr = -Read_Encoder(Encoder_A);
    Encoder_B_pr = -Read_Encoder(Encoder_B);
	
	short Encoder_C_pr;
	Encoder_C_pr = Read_Encoder(Encoder_C);
	short Encoder_D_pr;
	Encoder_D_pr = Read_Encoder(Encoder_D);
	//计算纠偏系数
	float LeftWheelDiff = wheelCoefficient(robot_control.LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(robot_control.LineDiffParam,0);
	
    //The encoder converts the raw data to wheel speed in m/s
	//编码器原始数据转换为车轮速度，单位m/s (四驱分支)
	//未完成自检时采集编码器数据进行判断
	if( robot_check.check_end == 0 )
	{
		robot_check.check_a += -Encoder_A_pr;
		robot_check.check_b += -Encoder_B_pr;
		robot_check.check_c += -Encoder_C_pr;
		robot_check.check_d += -Encoder_D_pr;
	}
	robot.MOTOR_A.Encoder = - Encoder_A_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / RightWheelDiff;
	robot.MOTOR_B.Encoder = - Encoder_B_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / LeftWheelDiff;
	robot.MOTOR_C.Encoder = - Encoder_C_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / LeftWheelDiff;
	robot.MOTOR_D.Encoder = - Encoder_D_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / RightWheelDiff;
}

/*-------------------------------- Robot state detection related functions ------------------------------------*/
/*--------------------------------         机器人状态检测相关函数          ------------------------------------*/
/**************************************************************************
Function Description: The function clears residual motor control signals when the robot \
  remains stationary on a non-sloping surface for a certain period of time. This helps  \
  reduce power consumption and motor noise of the robot.
Input Parameters: None
Return Value: None
Author: WHEELTEC
函数功能：小车在非斜坡上静止超过一定时间,清除电机残余的控制量.降低机器人功耗以及降低电机的噪音
入口参数：无
返回  值：无
作    者：WHEELTEC
**************************************************************************/
static void Robot_ParkingCheck(void)
{
	#define PUSH_PWM 500 //推车累计的PWM阈值,超出将执行清除
	
    //小车姿态简易判断
    float y_accle = (float)(imu.accel.y/1671.84f);//Y轴加速度实际值
    float z_accle = (float)(imu.accel.z/1671.84f);//Z轴加速度实际值
    float diff;

    //计算Y、Z加速度融合值，该值越接近9.8，表示小车姿态越水平
    if( y_accle > 0 ) diff  = z_accle - y_accle;
    else diff  = z_accle + y_accle;
	
	if( robot.MOTOR_A.Target != 0 || robot.MOTOR_B.Target!=0 || \
		robot.MOTOR_C.Target != 0 || robot.MOTOR_D.Target!=0)
	{   //小车被控制中,设置等待标志位.复位其他检测变量
		park.wait_check = 1;  //等待检测
		park.timecore = 0;    //复位检测相关变量
		park.clear_state = 0;
		park.start_clear = 0;
	}
	else 
	{
		if( park.wait_check==1 ) //速度非0 -> 0 触发检测
		{
			park.timecore++;
			if( park.timecore >= BALANCE_TASK_RATE*5 ) //小车静止时间超过5秒
			{
				park.wait_check=0;
				park.timecore = 0;
				
				//若机器人不在斜面上,则进入驻车模式,清空电机控制量
				if( diff > 8.8f ) park.start_clear = 1 , park.clear_state = 0;
				else  park.clear_state |= 1<<7 ; //在斜坡上,不可清除控制量.标记清除已完成
			}
		}
	}
	
	//检查是否已完成清除.完成清除后将会监测小车是否被动积累控制量(推车行为)
	if( ((park.clear_state>>7)&0x01)==1  )
	{
		if(diff > 8.8f )//小车不在斜坡上作为前提.
		{
			//出现推车行为
			if( abs(robot.MOTOR_A.Output) > PUSH_PWM || abs(robot.MOTOR_B.Output) > PUSH_PWM || \
				abs(robot.MOTOR_C.Output) > PUSH_PWM || abs(robot.MOTOR_D.Output) > PUSH_PWM)
			{
				park.timecore++;
				if( park.timecore >= BALANCE_TASK_RATE*10 ) //小车pwm累计时间超过10秒
				{
					park.timecore = 0;
					//进入驻车模式清除控制量
					park.start_clear = 1 , park.clear_state = 0;
				}
			}
		}
	}
	
	//开启驻车模式,执行清除任务
	if( park.start_clear==1 )
	{
		if( 1 == PI_Clear_Output(&PI_MotorA) ) park.clear_state |= 1<<0 ;
		else                                   park.clear_state &= ~(1<<0);
		
		if( 1 == PI_Clear_Output(&PI_MotorB) ) park.clear_state |= 1<<1 ;
		else                                   park.clear_state &= ~(1<<1);
		
		if( 1 == PI_Clear_Output(&PI_MotorC) ) park.clear_state |= 1<<2 ;
		else                                   park.clear_state &= ~(1<<2);
		
		if( 1 == PI_Clear_Output(&PI_MotorD) ) park.clear_state |= 1<<3 ;
		else                                   park.clear_state &= ~(1<<3);
		
		//4个电机完成清除
		if( (park.clear_state&0xff)==0x0f ) 
		{
			park.start_clear = 0;    //清除任务结束
			park.clear_state |= 1<<7;//标记清除完成
		}
	}
}

/**************************************************************************
Function function: Scan user buttons and perform different functions through different button states
Entry parameter: Frequency of executing scanning tasks
Return value: None
Author: SHEELTEC
函数功能：扫描用户按键,通过不同的按键状态执行不同的功能
入口参数：执行扫描的任务频率
返回  值：无
作    者：WHEELTEC
**************************************************************************/
static void UserKey_Scan(u16 rate)
{
	u8 state;

	//板子上的用户按键扫描
	state = KEY_Scan(rate,0);
	if( state==double_click )//双击重新标定imu数值
	{
		ImuData_copy(&imu.Deviation_gyro,&imu.Original_gyro);
		ImuData_copy(&imu.Deviation_accel,&imu.Original_accel);
	}

	else if( state==long_click ) //长按
	{
		oled.page++;
		if( oled.page > oled.MAX_PAGE ) oled.page = 1;
	}

}


/*-------------------------------- Function auxiliary type related functions ------------------------------------*/
/*--------------------------------         功能辅助类相关函数          ------------------------------------*/

/**************************************************************************
Function function: Save the steering variables of the Ackermann model's servo to Flash
Entry parameter: Flag bit for executing save
Return value: 1: Successfully saved 0: Not saved or failed to save
Author: WHEELTEC
函数功能：保存阿克曼车型的舵机转向变量到Flash
入口参数：是否执行保存的标志位
返回  值：1:保存成功 0:不存在保存需求 >1:flash写入错误
作    者：WHEELTEC
**************************************************************************/
static uint8_t FlashParam_Save(uint8_t *flag)
{
	u8 check=0;
	
	if(*flag==1)
	{
		*flag = 0;
		check = 1;
		taskENTER_CRITICAL();//操作FLash进入临界，保证数据安全
		int buf[7]={0};
		buf[0] = Akm_Servo.Min;
		buf[1] = Akm_Servo.Mid;
		buf[2] = Akm_Servo.Max;
		buf[3] = robot_control.LineDiffParam;
		buf[4] = *((int32_t*)&robot_control.rc_speed);
		buf[5] = robot.V_KP;
		buf[6] = robot.V_KI;
		check += Write_Flash( (u32*)buf , 7);
		
		taskEXIT_CRITICAL();//退出临界
		//若全部写入成功,check==1
	}

	return check;
}

void FlashParam_Read(void)
{
	int read;
	read = Read_Flash(0);//读取下标为0的数据
	if( read!=0xffffffff ) Akm_Servo.Min = read;
	
	read = Read_Flash(1);//读取下标为1的数据
	if( read!=0xffffffff ) Akm_Servo.Mid = read;
	
	read = Read_Flash(2);//读取下标为2的数据
	if( read!=0xffffffff ) Akm_Servo.Max = read;
	
	read = Read_Flash(3); //纠偏系数
	if( read!=0xffffffff ) robot_control.LineDiffParam = read;
	
	
	read = Read_Flash(4);//速度
	if( read!=0xffffffff ) robot_control.rc_speed = *((float*)&read);
	if( robot_control.rc_speed < 0 || robot_control.rc_speed > 10000 )//异常速度数据过滤
		robot_control.rc_speed = 500;
	
	
	read = Read_Flash(5);//KP参数
	if( read!=0xffffffff ) 
	{
		Set_Robot_PI_Param(read,-1);
	}
	
	read = Read_Flash(6);//KI参数
	if( read!=0xffffffff ) 
	{
		Set_Robot_PI_Param(-1,read);
	}
}

//static uint8_t Akm_SaveServo_Param(uint8_t *flag)
//{
//	u8 check=0;
//	
//	if(*flag==1)
//	{
//		*flag = 0;
//		check = 1;
//		taskENTER_CRITICAL();//操作FLash进入临界，保证数据安全
//		int buf[4]={Akm_Servo.Min,Akm_Servo.Mid,Akm_Servo.Max,robot_control.LineDiffParam};
//		check += Write_Flash( (u32*)buf , 4);
//		taskEXIT_CRITICAL();//退出临界
//		//若全部写入成功,check==1
//	}

//	return check;
//}

//void Akm_ReadServo_Param(void)
//{
//	int read;
//	read = Read_Flash(0);//读取下标为0的数据
//	if( read!=0xffffffff ) Akm_Servo.Min = read;
//	
//	read = Read_Flash(1);//读取下标为1的数据
//	if( read!=0xffffffff ) Akm_Servo.Mid = read;
//	
//	read = Read_Flash(2);//读取下标为2的数据
//	if( read!=0xffffffff ) Akm_Servo.Max = read;
//	
//	read = Read_Flash(3); //纠偏系数
//	if( read!=0xffffffff ) robot_control.LineDiffParam = read;
//	
//}

/**************************************************************************
Functionality: Velocity Smoothing Function - Gradually adjusts the speed to the target speed using a specified step value.
Input Parameters: Current speed, target speed, smoothing step value.
Return Value: Smoothed speed.
Author: WHEELTEC
函数功能：速度平滑函数,让速度以设置的步进值到达目标速度
入口参数：当前速度、目标速度、平滑的步进值
返回  值：平滑后的速度
作    者：WHEELTEC
**************************************************************************/
static float Vel_SmoothControl(float now_speed,float targetSpeed,float step)
{
	if( now_speed > targetSpeed )
	{
		now_speed -= step;
		if( now_speed<=targetSpeed ) now_speed = targetSpeed;
	}
	else
	{
		now_speed += step;
		if( now_speed>=targetSpeed ) now_speed = targetSpeed;
	}

	return now_speed;
}

/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

static int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}


///**************************************************************************
//Function: Smoothing the front wheel steering speed to prevent excessive steering gear current
//Input   : Current servo PWM, Target servo PWM, Smooth value
//Output  : none
//函数功能：对前轮转向速度做平滑处理，防止舵机电流过大
//入口参数：当前舵机控制PWM值 目标舵机控制PWM值 平滑值
//返回  值：无
//**************************************************************************/
//int Smooth_steering(int currentPWM, int targetPWM, float step)
//{
//    int threshold=7;
//    if     (targetPWM>currentPWM+threshold) currentPWM+=step;
//    else if(targetPWM<currentPWM-threshold) currentPWM-=step;
//    else                                    currentPWM =targetPWM;

//    return currentPWM;
//}


/**************************************************************************
Functionality: Radian to Degree Conversion - Converts a given radian value to its corresponding degree value.
Input Parameters: Radian value.
Return Value: Degree value corresponding to the input radian.
Author: WHEELTEC
函数功能：弧度转角度
入口参数：弧度
返回  值：输入弧度对应的角度
作    者：WHEELTEC
**************************************************************************/
float rad_to_angle(const float rad)
{
    return rad/PI*180.0f;
}

/**************************************************************************
Functionality: Degree to Radian Conversion - Converts a given degree value to its corresponding radian value.
Input Parameters: Degree value.
Return Value: Radian value corresponding to the input degree.
Author: WHEELTEC
函数功能：角度转弧度
入口参数：角度
返回  值：输入角度对应的弧度
作    者：WHEELTEC
**************************************************************************/
float angle_to_rad(const float angle)
{
    return angle/180.0f*PI;
}


/*-------------------------------- Software initialization related functions ------------------------------------*/
/*--------------------------------        软件初始化相关函数          ------------------------------------*/
/**************************************************************************
Functionality: Incremental PI Controller Initialization - Initializes an incremental PI 
               (Proportional-Integral) controller with specified parameters.
Input Parameters: PI controller, kp value (proportional gain), ki value (integral gain).
Return Value: None.
Author: WHEELTEC
函数功能：增量式PI控制器初始化
入口参数：PI控制器、kp值、ki值
返回  值：无
作    者：WHEELTEC
**************************************************************************/
void PI_Controller_Init(PI_CONTROLLER* p,int kp,int ki)
{
	p->Bias = 0;
	p->LastBias = 0;
	p->Output = 0;
	p->kp = kp;
	p->ki = ki;
}

//机器人控制相关变量初始化
void ROBOT_CONTROL_t_Init(ROBOT_CONTROL_t* p)
{
	//可调参数
	p->limt_max_speed = 3.5f;    //机器人限制最大的运动速度 m/s
	p->rc_speed = 500;           //机器人遥控速度基准,单位 mm/s
	p->smooth_MotorStep = 0.02f; //机器人电机速度平滑步进值
	p->smooth_ServoStep = 20;    //机器人舵机速度平滑步进值
	p->SoftWare_Stop = 0;        //机器人软件失能位
	
	//只读参数,保持默认
	p->Vx = 0;
	p->Vy = 0;
	p->Vz = 0;
	p->smooth_Vx = 0;
	p->smooth_Vy = 0;
	p->smooth_Vz = 0;
	p->command_lostcount = 0;
	p->ControlMode = 1;
	p->FlagStop = 0;
	
	//纠偏系数
	p->LineDiffParam = 50;
}

//自检变量初始化
static void ROBOT_SELFCHECK_t_Init(ROBOT_SELFCHECK_t* p)
{
	p->check_a = 0;
	p->check_b = 0;
	p->check_c = 0;
	p->check_d = 0;
	p->check_end = 0;
	p->errorflag  =0;
	p->DeepCheck = 0;
}

//检查某个变量是否发生改变
//入口参数:执行该函数的频率、需要检查的数据、变化幅度超过多少表示变化
//返回值：1发生了改变 0:未改变
uint8_t ValChangeCheck(const uint16_t rate,const short checkval,const uint8_t changeEva)
{
	static uint16_t timecore;
	static short lastval;
	
	const uint8_t DivisionFac = 2;     //分频系数
	if( rate < DivisionFac ) return 0;//频率过小，无法检测
	
	uint8_t changeflag = 0;//表示是否有改变

	timecore++;
	if( timecore >= rate/DivisionFac ) // 500ms检测1次
	{
		timecore = 0;
		if( abs( lastval - checkval ) > changeEva )  changeflag = 1;
		else
		{
			changeflag=0;
		}
		lastval = checkval;
	}
	
	return changeflag ;
}

//深度自检程序,需要把小车架起来后运行检查
static uint8_t Deep_SelfCheck( u16 RATE )
{	
	static uint32_t timecore = 0;
	static uint8_t errflag = 0;
	uint8_t check_ready = 0;
	
	//测量方法：给定单个轮子的编码器目标值,观察各个数据情况.
	
	/*
	报错优先级：
	1.出现负数,说明是车型选错,或者编码器AB相接反或者是电机输出端的正负接反
	2.其他编码器有值,说明是电机驱动输出口接错,或者是编码器接口接错.
	3.编码器没有值+PWM过大,则是编码器未接或者是电机接口未接
	*/

	timecore++;

	if( timecore==1 )
	{
		if( data_TaskHandle!=NULL ) vTaskSuspend(data_TaskHandle);//在深度自检程序运行时,挂起数据发送任务(该任务占用串口1),以免资源冲突
		any_printf(USART1,"\r\n小车进入深度自检模式,电机的转动顺序为A、B、C、D.请观察转动情况与报错日志.\r\n");
		Buzzer_AddTask(1,20);//蜂鸣器提示表示已进入深度自检模式
	}
	
	//第0~1秒前给定A轮速度
	if( timecore<RATE*2)
	{
		//直接指定电机的目标值并允许响应
		robot.MOTOR_A.Target = 0.5f;
		robot.MOTOR_B.Target = 0.0f;
		robot.MOTOR_C.Target = 0.0f;
		robot.MOTOR_D.Target = 0.0f;
		ResponseControl();
		PWMB = 0;PWMC = 0;PWMD = 0;
	}
	else if( timecore==RATE*2 ) //检查A轮是否存在异常
	{
		any_printf(USART1,"================= 电机A ================\r\n");
		
		//开始检查数据
		if( robot.MOTOR_A.Encoder > 0.4f && robot.MOTOR_A.Encoder < 0.6f )
		{
			any_printf(USART1,"电机A正常.\r\n");
			//正常情况
			//A轮正常
		}
		else
		{
			errflag = 1; //出现错误
			
			//按报错优先级检查错误情况
			if( robot.MOTOR_A.Encoder < - 0.2f ) 
			{
				any_printf(USART1,"检测到A电机编码器极性错误.\r\n");
				any_printf(USART1,"请检查：\r\n1.车型选择是否正确\r\n2.电机A的正负极是否存在线序错误\r\n3.电机A的编码器AB相线序是否错误\r\n");
				//对应接口出现负值,请检查：1.车型是否正确 2.电机的正负级是否反接 3.编码器AB相的线序是否正确
			}
			else if( fabs(robot.MOTOR_B.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机A时,检测到编码器接口B的信号.\r\n自检时电机A是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A、B是否存在调换\r\n否: 1.检查驱动器的A、B接口是否存在调换\r\n");
				//编码器B有值,请检查：1.编码器接口A与接口B接线是否调换了 2.驱动器的A、B接口是否存在调换
			}
			else if( fabs(robot.MOTOR_C.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机A时,检测到编码器接口C的信号.\r\n自检时电机A是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A、C是否存在调换\r\n否: 1.检查驱动器的A、C接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//编码器C有值,请检查：1.编码器A与C接口接线是否调换了 2.驱动器A、C接口是否存在调换 3.板子的牛角座是否存在调换
			}
			else if( fabs(robot.MOTOR_D.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机A时,检测到编码器接口D的信号.\r\n自检时电机A是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A、D是否存在调换\r\n否: 1.检查驱动器的A、D接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//编码器D有值,请检查：1.编码器A与D接口接线是否调换了 2.驱动器A、D接口是否存在调换 3.板子的牛角座是否存在调换
			}
			else if( fabs(robot.MOTOR_A.Encoder) < 0.05f && PI_MotorA.Output > 5000 ) 
			{
				any_printf(USART1,"驱动电机A时,未检测到编码器信号.\r\n自检时电机A是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A是否存在松动或未接\r\n否: 1.检查电机A的电源线是否存在松动或未接 2.检查板子右下角左边的牛角座是否松动或未接\r\n");
				//编码器没有值,对应接口PWM值较大,请检查：1.编码器接口是否松动或未接 2.电机正负极接头是否松动或者未接
			}
			
		}
		
		any_printf(USART1,"================= 电机A ================\r\n\r\n");
		
		//检查完毕,复位PI控制器,防止下一个电机检查受到干扰
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
	}
	else if( timecore>RATE*2 && timecore<RATE*4 ) //B轮控制
	{
		robot.MOTOR_A.Target = 0.0f;robot.MOTOR_B.Target = 0.5f;
		robot.MOTOR_C.Target = 0.0f;robot.MOTOR_D.Target = 0.0f;
		ResponseControl();
		PWMA = 0;PWMC = 0;PWMD = 0;
	} 
	else if( timecore==RATE*4 )  //B轮检查
	{
		any_printf(USART1,"================= 电机B ================\r\n");
		
		if( robot.MOTOR_B.Encoder > 0.4f && robot.MOTOR_B.Encoder < 0.6f )
		{
			any_printf(USART1,"电机B正常.\r\n");
		}
		else
		{
			errflag = 1; //出现错误
			
			if( robot.MOTOR_B.Encoder < - 0.2f ) 
			{
				any_printf(USART1,"检测到B电机编码器极性错误.\r\n");
				any_printf(USART1,"请检查：\r\n1.车型选择是否正确\r\n2.电机B的正负极是否存在线序错误\r\n3.电机B的编码器AB相线序是否错误\r\n");
				//对应接口出现负值,请检查：1.车型是否正确 2.电机的正负级是否反接 3.编码器AB相的线序是否正确
			}
			else if( fabs(robot.MOTOR_A.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机B时,检测到编码器接口A的信号.\r\n自检时电机B是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A、B是否存在调换\r\n否: 1.检查驱动器的A、B接口是否存在调换\r\n");
				//编码器B有值,请检查：1.编码器接口B与接口A接线是否调换了 2.驱动器的B、A接口是否存在调换
			}
			else if( fabs(robot.MOTOR_C.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机B时,检测到编码器接口C的信号.\r\n自检时电机B是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口B、C是否存在调换\r\n否: 1.检查驱动器的B、C接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//编码器C有值,请检查：1.编码器B与C接口接线是否调换了 2.驱动器B、C接口是否存在调换 3.板子的牛角座是否存在调换
			}
			else if( fabs(robot.MOTOR_D.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机B时,检测到编码器接口D的信号.\r\n自检时电机B是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口B、D是否存在调换\r\n否: 1.检查驱动器的B、D接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//编码器D有值,请检查：1.编码器B与D接口接线是否调换了 2.驱动器B、D接口是否存在调换 3.板子的牛角座是否存在调换
			}
			else if( fabs(robot.MOTOR_B.Encoder) < 0.05f && PI_MotorB.Output > 5000 ) 
			{
				any_printf(USART1,"驱动电机B时,未检测到编码器信号.\r\n自检时电机B是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口B是否存在松动或未接\r\n否: 1.检查电机B的电源线是否存在松动或未接 2.检查板子右下角左边的牛角座是否松动或未接\r\n");
				//编码器没有值,对应接口PWM值较大,请检查：1.编码器接口是否松动或未接 2.电机正负极接头是否松动或者未接
			}
		}
		any_printf(USART1,"================= 电机B ================\r\n\r\n");
		//检查完毕,复位PI控制器,防止下一个电机检查受到干扰
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
	}
	
	#if defined OMNI_CAR || defined _4WD_CAR || defined MEC_CAR
	
	else if( timecore>RATE*4 && timecore<RATE*6 )//C轮控制
	{
		robot.MOTOR_A.Target = 0.0f;robot.MOTOR_B.Target = 0.0f;
		robot.MOTOR_C.Target = 0.5f;robot.MOTOR_D.Target = 0.0f;
		ResponseControl();
		PWMA = 0;PWMB = 0;PWMD = 0;
	} 
	else if( timecore==RATE*6 )//C轮检查
	{
		any_printf(USART1,"================= 电机C ================\r\n");
		
		if( robot.MOTOR_C.Encoder > 0.4f && robot.MOTOR_C.Encoder < 0.6f )
		{
			any_printf(USART1,"电机C正常.\r\n");
		}
		else
		{
			errflag = 1; //出现错误
			
			if( robot.MOTOR_C.Encoder < - 0.2f ) 
			{
				any_printf(USART1,"检测到C电机编码器极性错误.\r\n");
				any_printf(USART1,"请检查：\r\n1.车型选择是否正确\r\n2.电机C的正负极是否存在线序错误\r\n3.电机C的编码器AB相线序是否错误\r\n");
				//对应接口出现负值,请检查：1.车型是否正确 2.电机的正负级是否反接 3.编码器AB相的线序是否正确
			}
			else if( fabs(robot.MOTOR_A.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机C时,检测到编码器接口A的信号.\r\n自检时电机C是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A、C是否存在调换\r\n否: 1.检查驱动器的A、C接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//编码器B有值,请检查：1.编码器接口C与接口A接线是否调换了 2.驱动器的C、A接口是否存在调换 3.板子的牛角座是否存在调换
			}
			else if( fabs(robot.MOTOR_B.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机C时,检测到编码器接口B的信号.\r\n自检时电机C是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口B、C是否存在调换\r\n否: 1.检查驱动器的B、C接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//编码器C有值,请检查：1.编码器C与B接口接线是否调换了 2.驱动器C、B接口是否存在调换 3.板子的牛角座是否存在调换
			}
			else if( fabs(robot.MOTOR_D.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机C时,检测到编码器接口D的信号.\r\n自检时电机C是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口C、D是否存在调换\r\n否: 1.检查驱动器的C、D接口是否存在调换\r\n");
				//编码器D有值,请检查：1.编码器C与D接口接线是否调换了 2.驱动器C、D接口是否存在调换 
			}
			else if( fabs(robot.MOTOR_C.Encoder) < 0.05f && PI_MotorC.Output > 5000 ) 
			{
				any_printf(USART1,"驱动电机C时,未检测到编码器信号.\r\n自检时电机C是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口C是否存在松动或未接\r\n否: 1.检查电机C的电源线是否存在松动或未接 2.检查板子右下角右边的牛角座是否松动或未接\r\n");
				//编码器没有值,对应接口PWM值较大,请检查：1.编码器接口是否松动或未接 2.电机正负极接头是否松动或者未接
			}
		}
		any_printf(USART1,"================= 电机C ================\r\n\r\n");
		//检查完毕,复位PI控制器,防止下一个电机检查受到干扰
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
	}
	
	#endif /* if defined OMNI_CAR _4WD_CAR MEC_CAR */
	
	#if defined _4WD_CAR || defined MEC_CAR
	
	else if( timecore>RATE*6 && timecore<RATE*8 )//D轮控制
	{
		robot.MOTOR_A.Target = 0.0f;robot.MOTOR_B.Target = 0.0f;
		robot.MOTOR_C.Target = 0.0f;robot.MOTOR_D.Target = 0.5f;
		ResponseControl();
		PWMA = 0;PWMB = 0;PWMC = 0;
	}
	else if( timecore==RATE*8  )//D轮检查
	{
		any_printf(USART1,"================= 电机D ================\r\n");
		
		if( robot.MOTOR_D.Encoder > 0.4f && robot.MOTOR_D.Encoder < 0.6f )
		{
			any_printf(USART1,"电机D正常.\r\n");
		}
		else
		{
			errflag = 1; //出现错误
			
			if( robot.MOTOR_D.Encoder < - 0.2f ) 
			{
				any_printf(USART1,"检测到D电机编码器极性错误.\r\n");
				any_printf(USART1,"请检查：\r\n1.车型选择是否正确\r\n2.电机D的正负极是否存在线序错误\r\n3.电机D的编码器AB相线序是否错误\r\n");
				//对应接口出现负值,请检查：1.车型是否正确 2.电机的正负级是否反接 3.编码器AB相的线序是否正确
			}
			else if( fabs(robot.MOTOR_A.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机D时,检测到编码器接口A的信号.\r\n自检时电机D是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A、D是否存在调换\r\n否: 1.检查驱动器的A、D接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//编码器B有值,请检查：1.编码器接口D与接口A接线是否调换了 2.驱动器的D、A接口是否存在调换 3.板子的牛角座是否存在调换
			}
			else if( fabs(robot.MOTOR_B.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机D时,检测到编码器接口B的信号.\r\n自检时电机D是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口B、D是否存在调换\r\n否: 1.检查驱动器的B、D接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//编码器C有值,请检查：1.编码器D与B接口接线是否调换了 2.驱动器D、B接口是否存在调换 3.板子的牛角座是否存在调换
			}
			else if( fabs(robot.MOTOR_C.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机D时,检测到编码器接口C的信号.\r\n自检时电机D是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口C、D是否存在调换\r\n否: 1.检查驱动器的C、D接口是否存在调换\r\n");
				//编码器D有值,请检查：1.编码器C与D接口接线是否调换了 2.驱动器C、D接口是否存在调换 
			}
			else if( fabs(robot.MOTOR_D.Encoder) < 0.05f && PI_MotorD.Output > 5000 ) 
			{
				any_printf(USART1,"驱动电机D时,未检测到编码器信号.\r\n自检时电机D是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口D是否存在松动或未接\r\n否: 1.检查电机D的电源线是否存在松动或未接 2.检查板子右下角右边的牛角座是否松动或未接\r\n");
				//编码器没有值,对应接口PWM值较大,请检查：1.编码器接口是否松动或未接 2.电机正负极接头是否松动或者未接
			}
		}
		
		any_printf(USART1,"================= 电机D ================\r\n");
		
		//检查完毕,复位PI控制器,防止下一个电机检查受到干扰
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
	}
	#endif /* if defined  _4WD_CAR MEC_CAR */
	
	else
	{
		check_ready = 1;
		robot.MOTOR_A.Target = 0.0f;
		robot.MOTOR_B.Target = 0.0f;
		robot.MOTOR_C.Target = 0.0f;
		robot.MOTOR_D.Target = 0.0f;
		Set_Pwm(0,0,0,0,0);
		any_printf(USART1,"\r\n\r\n");
		Buzzer_AddTask(1,20);//蜂鸣器提示表示已完成深度自检
		
		//深度自检结束且无报错,恢复挂起的任务.
		if( 0 == errflag  )
			if( data_TaskHandle!=NULL ) vTaskResume(data_TaskHandle);
		
		timecore = 0;
		errflag = 0;
	}

	return check_ready;
	
}
