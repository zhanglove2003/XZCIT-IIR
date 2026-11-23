#!/bin/bash
# 启动摄像头、雷达、网页调试、静态TF、融合安全、SLAM 全套。
# 自动按 CP2102 找雷达串口，不写死 ttyUSB 编号。
#
# 底盘车速/里程/电压由底盘桥 c50c_uart3_node 独占 /dev/ttyACM1 读取，
# 再经 /diagnostics 发布；网页从该话题取值显示，底盘卡与电池卡有数据。
#
# web_debug 由 c50c-web.service 托管（Restart=always + 串口饿死自愈 +
# TimeoutStopSec=5 解决 SIGTERM 卡死），本脚本只负责 restart 对齐新代码。
# 2026-11-15 重建：板上原文件第二次被断电清零（11-11 一次、11-15 一次），
# 依 cruise-20261113 备份 + 执行计划记忆重建，配 c50c-bootguard 防复发。

source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
cd /home/orangepi

mkdir -p /home/orangepi/.cache/opi-robot

# 看门狗计时器先停：重启窗口内不许它判 8088 死亡拉起旧实例
systemctl --user stop c50c-web-watchdog.timer 2>/dev/null

# 找雷达串口：优先 by-id 稳定链接
PORT=""
for p in /dev/serial/by-id/*CP2102*-if00-port0; do
    [ -e "$p" ] && PORT="$p" && break
done
if [ -z "$PORT" ]; then
    PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
fi
echo "雷达串口: ${PORT:-无}"

LIB=/home/orangepi/ros2_ws/install/opi_robot/lib/opi_robot

if [ ! -d /opt/ros/humble ]; then
    echo "ROS2 未安装"
    exit 1
fi

pkill -f "$LIB/camera_node" 2>/dev/null
pkill -f "$LIB/lidar_node" 2>/dev/null
pkill -f "$LIB/web_debug" 2>/dev/null
pkill -f "$LIB/chassis_status_node" 2>/dev/null
pkill -f "$LIB/chassis_tee.py" 2>/dev/null
pkill -f "$LIB/fusion_node" 2>/dev/null
pkill -f "$LIB/static_tf" 2>/dev/null
pkill -f "async_slam_toolbox_node" 2>/dev/null
rm -f /home/orangepi/.cache/opi-robot/chassis.traffic
sleep 1

# EKF、ROS daemon 等进程可能仍在使用 Fast DDS 共享内存，不能在此删除。
# 保留原有桥重启流程。
systemctl --user try-restart c50c-chassis.service 2>/dev/null

# [N2 2026-11-17] 幂等化：三节点启动段各加存活检查，已存活则跳过
pgrep -f "$LIB/camera_node" >/dev/null || {
    setsid nohup python3 "$LIB/camera_node" --ros-args \
        -p width:=640 -p height:=480 > /tmp/cn.log 2>&1 < /dev/null &
}
if [ -n "$PORT" ] && ! pgrep -f "$LIB/lidar_node" >/dev/null; then
    setsid nohup python3 "$LIB/lidar_node" --ros-args -p lidar_port:="$PORT" > /tmp/ln.log 2>&1 < /dev/null &
elif [ -z "$PORT" ]; then
    echo "未找到雷达串口，跳过雷达节点"
fi
# base_link→laser 静态外参（x=+0.08 y=0 z=0.22，阶段 A tf2_echo 验证）；
# static_tf 本身是 shell 包装脚本，不能再用 python3 启
setsid nohup "$LIB/static_tf" > /tmp/tf.log 2>&1 < /dev/null &
# [C50C 2026-11-15] base_link→ultrasonic_front 静态外参：前置 HC-SR04，
# 车头正中、离地约 10cm（傍晚移装，用户操作）；x=0.19 实测（前缘 0.20 - 传感器距前缘 1cm，用户量测）。
# 本脚本顶部已 source ROS；pkill 匹配 TF 参数串，勿与节点名混淆。
pkill -f "child-frame-id ultrasonic_front" 2>/dev/null
setsid nohup ros2 run tf2_ros static_transform_publisher \
    --x 0.19 --y 0 --z 0.10 --qx 0 --qy 0 --qz 0 --qw 1 \
    --frame-id base_link --child-frame-id ultrasonic_front \
    > /tmp/tf_us.log 2>&1 < /dev/null &
# base_link→imu_link 静态外参（MPU-6050 平放板载、轴对齐车体；依据：主控原理图 mpu6050 电路段
# + 11-15 实测定案 IMU gz 符合 REP-103；z=0.05m 保守估计，imu0_differential 只吃角速度不敏感平移）
pkill -f "child-frame-id imu_link" 2>/dev/null
setsid nohup ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0.05 --qx 0 --qy 0 --qz 0 --qw 1 \
    --frame-id base_link --child-frame-id imu_link \
    > /tmp/imu_tf.log 2>&1 < /dev/null &
# 融合安全门（stop 0.4 / slow 0.9）；enable_cmd_vel:=true 让巡航实车可驱动（2026-11-15）
pgrep -f "$LIB/fusion_node" >/dev/null || {
    setsid nohup python3 "$LIB/fusion_node" --ros-args -p enable_cmd_vel:=true -p cruise_max_run_s:=1.0 -p body_half_length:=0.2 -p body_half_width:=0.2 -p body_center_x:=-0.08 > /tmp/fusion.log 2>&1 < /dev/null &
}
# SLAM（雷达在才启）：绕开 launch 直接跑节点，参数名坑见执行计划
# [navfix 2026-11-18] SLAM_MODE 门控：nav 模式不启 slam_toolbox（map→odom 归 AMCL）
if [ -n "$PORT" ] && [ "${SLAM_MODE:-mapping}" = mapping ]; then
    LD_LIBRARY_PATH="/home/orangepi/.local/lib/c50c-slam-heading:${LD_LIBRARY_PATH:-}" nohup /opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node --ros-args \
        -r __node:=slam_toolbox \
        --params-file /home/orangepi/slam_online_async.yaml \
        -p use_sim_time:=false > /home/orangepi/slam.log 2>&1 < /dev/null &
else
    echo "SLAM_MODE=${SLAM_MODE:-mapping}：本模式不启动 slam_toolbox"
fi
systemctl --user restart c50c-web.service

sleep 5
echo "--- 端口 ---"
ss -ltn | grep 8088 || echo "8088 未监听"
echo "--- 网页 ---"
tail -2 /tmp/wb.log
echo "--- 摄像头 ---"
tail -1 /tmp/cn.log
echo "--- 雷达 ---"
tail -2 /tmp/ln.log 2>/dev/null
echo "--- 融合 ---"
tail -2 /tmp/fusion.log 2>/dev/null
echo "--- SLAM ---"
tail -2 /home/orangepi/slam.log 2>/dev/null

# [N2 2026-11-17] 修复原文件末尾被断电截断的行（原为 2>/dev，且缺换行）
systemctl --user start c50c-web-watchdog.timer 2>/dev/null
