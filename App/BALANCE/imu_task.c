#include "imu_task.h"

//imu数据结构体
IMU_DATA_t imu;

void imu_zero_sample(void);

void MPU6050_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {
        //This task runs at 100Hz
        //此任务以100Hz的频率运行
        vTaskDelayUntil(&lastWakeTime, F2T(IMU_TASK_RATE));

        //Read the gyroscope zero before starting
        //开机前，读取陀螺仪零点
        if(SysVal.Time_count<CONTROL_DELAY)
        {	
            imu_zero_sample();
        }
        //Get acceleration sensor data
        MPU6050_Get_Accelscope();

        //Get gyroscope data
        MPU6050_Get_Gyroscope(); //得到陀螺仪数据

    }
}


/**************************************************************************
函数功能：复制imu的数值
入口参数：需要赋值的结构体变量，被复制的结构体变量
返回  值：无
作    者：WHEELTEC
**************************************************************************/
/* [C50C] 陀螺仪/加速度零点累加平均.
   MPU6050 任务在 Time_count<CONTROL_DELAY 窗口内调用.
   原代码每拍用当前读数覆盖零点, 只保留最后一拍, 那一拍不静止零点就错.
   现在按真实采样次数累加求平均, 抽样满 IMU_ZERO_SAMPLES 就停止,
   零点保持已有的平均值. */
#define IMU_ZERO_SAMPLES  200

static int32_t  imu_sum_gyro[3];
static int32_t  imu_sum_accel[3];
static uint16_t imu_zero_cnt;

void imu_zero_sample(void)
{
    if( SysVal.Time_count >= CONTROL_DELAY ) return;
    if( imu_zero_cnt >= IMU_ZERO_SAMPLES )   return;

    imu_sum_gyro[0]  += imu.gyro.x;
    imu_sum_gyro[1]  += imu.gyro.y;
    imu_sum_gyro[2]  += imu.gyro.z;
    imu_sum_accel[0] += imu.accel.x;
    imu_sum_accel[1] += imu.accel.y;
    imu_sum_accel[2] += imu.accel.z;
    imu_zero_cnt++;

    imu.Deviation_gyro.x  = (int16_t)(imu_sum_gyro[0]  / imu_zero_cnt);
    imu.Deviation_gyro.y  = (int16_t)(imu_sum_gyro[1]  / imu_zero_cnt);
    imu.Deviation_gyro.z  = (int16_t)(imu_sum_gyro[2]  / imu_zero_cnt);
    imu.Deviation_accel.x = (int16_t)(imu_sum_accel[0] / imu_zero_cnt);
    imu.Deviation_accel.y = (int16_t)(imu_sum_accel[1] / imu_zero_cnt);
    imu.Deviation_accel.z = (int16_t)(imu_sum_accel[2] / imu_zero_cnt);
}

void ImuData_copy(IMU_BASE_t* req_val,const IMU_BASE_t* copied_val)
{
	req_val->x = copied_val -> x;
	req_val->y = copied_val -> y;
	req_val->z = copied_val -> z;
}


