# C50C ROS 2 导航与控制栈（Orange Pi 5 Pro / ROS 2 Humble）

本目录归档了实车运行验证通过的 Orange Pi 5 Pro 上位机 ROS 2 完整源码、Nav2 参数、EKF/AMCL 配置及 `systemd --user` 开机自启单元。

## 目录结构

- `ros2_ws/src/opi_robot/`：传感器节点（`camera_node`、`lidar_node`）、融合安全门（`fusion_node`，含超声波/雷达近距停车与无效锁存）及 Web 控制台服务端（`web_debug` + `console_page.html` + `patrol.py`，含 `--fake` 模式与安全许可链路）。
- `ros2_ws/src/c50c_uart3/`：STM32F407VE 底盘串口桥（`c50c_uart3_node`），负责解析 48 字节底盘帧发布 `/odom_candidate`、`/imu/data_raw`、`/ultrasonic_front`、`/diagnostics`，并结合 `/drive_safety`、`/motion_license`、`/guard_authorize` 安全仲裁后下发速度指令。
- `ros2_ws/src/c50c_odom/`：纯里程计积分工具包。
- `nav2/`：Nav2 定位与导航配置——
  - `nav2_localization.launch.py`：拉起 `map_server`、`amcl` 与 `lifecycle_manager_localization`。
  - `nav2_navigation.launch.py`：拉起 Nav2 控制/规划/行为树/速度平滑节点并保留 `/cmd_vel_smoothed` 输出至 `nav_relay.py`。
  - `nav2_params_20261120.yaml`：采用 `RegulatedPurePursuitController`（RPP）路径跟踪控制器。
  - `nav_relay.py`：`/cmd_vel_smoothed -> /nav_cmd_vel` 速度换算与转向优先迟滞中继（`--turn-priority-wz 0.38 --turn-release-wz 0.14`，含上游超时归零看门狗）。
  - `bt_navigate_*.xml`：无自旋恢复的导航行为树。
- `config/`：
  - `ekf.yaml`：`robot_localization` EKF 配置（编码器 `vx` + IMU `gz` 融合输出 `odom -> base_link` TF）。
  - `localization_params.yaml` & `maps/c50c-field-20261120/`：场地实测建图与 AMCL 参数（大型 `.posegraph` / `.data` 二进制建图中间文件未入库）。
- `systemd/`：用户级 `systemd` 服务单元（`c50c-chassis`、`c50c-ekf`、`c50c-localization`、`c50c-nav2`、`c50c-web`、`opi-robot`、`dds-watchdog` 等，`SLAM_MODE=nav`）及配套启动/看门狗脚本（`systemd/scripts/`）。
