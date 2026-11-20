"""C50C UART3 状态帧接收与 ROS 2 IMU 桥接包。

只读数据通路：解析 UART3 的 48 字节状态帧并发布 sensor_msgs/Imu。
本包不向底盘发送任何串口命令。
"""
