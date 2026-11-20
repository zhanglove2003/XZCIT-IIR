#!/usr/bin/env bash
# Start the established baseline bridge on the chassis USB port.
set -eo pipefail

device_link=/dev/serial/by-path/platform-fc800000.usb-usb-0:1.2:1.0
port=$(readlink -e "$device_link") || {
    echo "底盘通信口不存在: $device_link" >&2
    exit 1
}
# device 传 by-path 符号链接而非解析后的 ttyACM 名：USB 重枚举后
# udev 会把链接重新指到（可能变号的）新设备，桥重开串口时打开的
# 永远是"当前那个设备"。ttyACM 名在重枚举后可能失效或换号。
# （2026-11-14 串口看门狗配套改动；重开逻辑在 uart3_node 内部。）
if [[ ! "$port" =~ ^/dev/ttyACM[0-9]+$ ]]; then
    echo "底盘通信口类型不符合预期: $port" >&2
    exit 1
fi
if pgrep -f '[/]c50c_uart3/c50c_uart3_node' >/dev/null; then
    echo '已有底盘桥在运行，拒绝重复启动' >&2
    exit 1
fi
if fuser "$port" >/dev/null 2>&1; then
    echo "底盘通信口已被占用: $port" >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
cd /home/orangepi

# 写开机时刻，给网页的"开机初始化中"倒计时用（内容是墙上时钟秒）。
# 这条命令紧接着 exec 底盘桥，与固件 20 秒静止要求同步。
mkdir -p /home/orangepi/.cache/opi-robot
date +%s > /home/orangepi/.cache/opi-robot/boot_hold

exec /home/orangepi/ros2_ws/install/c50c_uart3/lib/c50c_uart3/c50c_uart3_node \
    --ros-args -p device:="$device_link" -p baudrate:=115200 \
    -p drive_enable:=true -p drive_input_enable:=true \
    -p drive_fd_authorised:=true -p drive_firmware_protocol:=baseline \
    -p odom_enable:=true -p odom_tf_enable:=false -p gyro_bias_enable:=true
