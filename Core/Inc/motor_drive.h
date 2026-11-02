/* 四路麦轮电机驱动、编码器读取与 ROS 2 上位机串口通信
 * 轮序: 0=RF 右前, 1=LF 左前, 2=RR 右后, 3=LR 左后
 * 引脚定义见 pinmap.md 与 main.h
 * 串口协议见 ros2_ws/src/mecanum_bridge/mecanum_bridge/protocol.py
 */
#ifndef MOTOR_DRIVE_H
#define MOTOR_DRIVE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_WHEEL_COUNT 4U
#define MOTOR_DUTY_MAX    1000 /* 占空比范围 -1000..1000 (单位 ‰) */

/* 电机驱动与编码器 */
void MotorDrive_Init(void);
void MotorDrive_SetEnable(uint8_t enable);
void MotorDrive_SetDuty(uint8_t wheel, int16_t duty);
int16_t MotorDrive_ReadEncoderDelta(uint8_t wheel);

/* ROS 2 串口通信 */
void RosLink_Init(void);
void RosLink_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVE_H */
