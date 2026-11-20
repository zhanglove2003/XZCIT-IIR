# c50c_uart3

C50C UART3 状态帧 → ROS 2 只读桥接。**不向底盘发送任何串口命令；默认只发
布 `imu/data_raw`。里程计候选为默认关闭的影子话题（见下），无论开关状态都
不发布正式 `/odom`、不发布 `odom → base_link` TF。**

协议依据：本仓库 `c50c-firmware-ref/` 内存的 11-05 固件基线组帧代码与
参考解码器（SHA-256 见 `task-results/c50c-uart3-ros2.md`），解析语义与
已验证的 `tools/c50c_uart3_monitor.py` 一致。

## 24 字节状态帧字段表

（固件 `data_transition()`，20 Hz；数值为大端 16 位有符号，除注明外）

| 字节 | 字段 | 单位 | 来源 | ROS 侧处理 |
| --- | --- | --- | --- | --- |
| 0 | 帧头 0x7B | - | 常量 | 同步用 |
| 1 | stop_flag（FlagStop） | - | 软件失能标志 | 0=使能；非 0=失能 |
| 2-3 | chassis vx | mm/s | 后轮编码器里程计 | 仅解析保留，不发布 |
| 4-5 | chassis vy | mm/s | 后轮编码器里程计 | 仅解析保留，不发布 |
| 6-7 | chassis wz | mrad/s | 编码器差速解算（非 IMU） | 仅解析保留，不发布 |
| 8-13 | accel x/y/z | LSB | MPU-6050（±2 g，含固件轴映射；z 含 +16384 即 +1 g 基准） | ÷1671.84 → m/s²，发布 |
| 14-19 | gyro x/y/z | LSB | MPU-6050（±500 dps，含固件轴映射）；**FlagStop≠0 时 z 被固件钳零** | ÷3754.93 → rad/s；FlagStop≠0 时整组置零并标记不可用 |
| 20-21 | voltage | mV | 电池电压 | 仅解析保留，不发布 |
| 22 | BCC | - | 前 22 字节 XOR | 校验 |
| 23 | 帧尾 0x7D | - | 常量 | 校验 |

## ROS 话题

- `~/imu/data_raw`（默认 `/imu/data_raw`），`sensor_msgs/Imu`，SensorData QoS：
  - **姿态**：固件不提供四元数 → 四元数全零，`orientation_covariance[0] = -1`（恒不可用）。
  - **角速度**：`FlagStop = 0` 时为实测值（协方差对角 = `gyro_variance` 参数，占位值）；
    `FlagStop ≠ 0` 时陀螺 Z 被固件强制置零，无法区分真实静止与数据压零，
    整组角速度置零且 `angular_velocity_covariance[0] = -1`。
  - **线加速度**：实测零偏扣除值换算（z 含 +1 g 基准；X/Y 的静置重力分量被
    上电零偏扣除吸收，即"相对上电姿态的线性加速度 + Z 重力"，REP 145 的
    raw 约定以此为准，现场需复核）。
- 固件轴映射（芯片 Y→ROS X、芯片 X 取负→ROS Y、Z→Z）已在固件组帧时完成，
  本包**不做二次旋转**。芯片实际安装朝向无归档证据，物理轴向**待现场逐轴
  实测验证**（尤其 D500 倒装后）。

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `device` | （必填） | UART3 串口设备路径 |
| `baudrate` | 115200 | 8N1，无流控 |
| `frame_id` | `imu_link` | Imu 消息 frame_id |
| `accel_variance` | 0.01 | 加速度协方差占位值，待现场标定 |
| `gyro_variance` | 0.001 | 角速度协方差占位值，待现场标定 |
| `frame_gap_warn_s` | 0.25 | 帧间隔超时告警阈值（标称 50 ms） |
| `odom_enable` | false | 里程计影子输出开关（默认关闭） |
| `odom_topic` | `odom_candidate` | 影子话题名（相对命名空间） |
| `odom_frame_id` | `odom` | Odometry header.frame_id |
| `odom_child_frame_id` | `base_link` | Odometry child_frame_id |
| `odom_max_interval_s` | 0.5 | 断流间隙阈值（与固件 dt>500ms 丢弃语义一致） |
| `odom_pose_variance_x/y/yaw` | 0.01 | 位姿协方差占位值，未标定 |
| `odom_twist_variance_linear/angular` | 0.01 | 速度协方差占位值，未标定 |
| `odom_unmeasured_variance` | 1e6 | 未测量维度方差（"不可融合"标记，非标定值） |
| `odom_tf_enable` | false | 候选 TF 输出开关（默认关闭；须与 `odom_enable` 同开；与 `use_identity_odom:=true` 互斥） |

## 串口占用（重要）

**现场启动顺序：先关闭 UART3 监视器和其他读取程序，再启动桥接节点。**

- 节点以 `O_RDONLY | O_NOCTTY | O_NONBLOCK` 只读打开设备并设置 `TIOCEXCL`，
  不发送任何字节。
- 但 `TIOCEXCL` 只能阻止**本节点打开之后**的新打开请求（后开者得到
  EBUSY），**不能关闭或发现所有已有占用者**——先于节点打开的进程可以
  继续持有并读取该设备。
- 节点的 termios 配置（raw 模式、波特率）作用于 tty 线路本身，会被共享
  该设备的所有进程感知；已有进程打开时，**不能**把本节点当作无影响地
  并行读取。
- 节点启动（及每次重连）时扫描 `/proc` 提示疑似占用者（PID 列表告警）。
  该检查是尽力而为：有竞态（检查后打开前仍有窗口）、读不到无权限进程
  的 fd、不检测内核内部占用——**它是提示，不是无竞态的独占保证**。

## 里程计候选影子话题（默认关闭）

节点内置 `OdomCandidate`（`StatusFrame` → `c50c_odom.PlanarOdomIntegrator` →
`nav_msgs/msg/Odometry`）接线，由参数 `odom_enable`（**默认 false**）控制：

- 关闭（默认）：无影子话题发布器、无积分状态，节点行为与只发 IMU 时一致。
- 开启：发布可配置的影子话题（默认 `odom_candidate`），**不发布正式
  `/odom`**；影子话题用默认可靠 QoS。启动时对 `odom_topic`
  参数与外部重映射做解析守卫：最终话题名若为全局 `/odom`，节点拒绝启动
  （RuntimeError）而非静默发布——因此任何配置/重映射组合都不会把影子
  输出落到全局 `/odom`（要么发影子名，要么拒绝启动）。**带命名空间的
  同名话题（如 `/ns/odom`）不在守卫范围**，需启动配置审查。

### 可选候选 TF（`odom_tf_enable`，默认关闭）

- 须与 `odom_enable` 同时开启（否则启动报 RuntimeError——TF 由同一候选
  积分位姿派生，没有候选就没有 TF）。
- 开启后随影子话题同帧发布 `odom→base_link` 变换（`/tf`，20 Hz）：
  **与 Odometry 消息同帧、同位姿、同 ROS 时间戳**——TF 直接取自已构造
  消息的位姿字段，节点内没有第二套积分器。这是候选位姿的 TF，
  **不是真实里程计 TF**（偏航绝对方向、速度标度未验收）。
- **与建图 `use_identity_odom:=true` 互斥**：两者都往 `odom→base_link`
  这条 TF 边上发布时，是两个发布者竞争同一条边（TF 树会出现该边的多个
  父源，slam_toolbox 的运动模型在两者间摇摆）。启动时节点会统计
  `/tf` + `/tf_static` 既有发布者数量并告警——这是**尽力而为的配置审查
  提示**：无法判断对方发布哪条边，也无法消除之后加入的外部发布者，
  不构成互斥保证；互斥必须靠启动配置保证。
- 重连语义与消息一致：重连后首帧只重建积分基准（TF 位姿不变），
  第二帧起积分。
- 积分时间基准 = 主机单调接收时刻；消息戳用节点（ROS）时钟（与 IMU 同帧
  取得）。首帧只建时间基准；重复/倒退时间、`dt > odom_max_interval_s`
  断流间隙均不跨积分，且计入诊断。**串口重连**时额外做两件事：丢弃解析器
  残余半帧（断开前后字节可能误拼成恰好通过 BCC 的假帧），并重置积分基准
  ——重连后**首帧只重建基准、不产生位移**（重连→首帧之间没有速度样本，
  不能按首帧速度补算），从第二帧起按常规间隔策略积分；断流前位姿保留。
- 角速度来源是帧 offset 6-7 的**编码器差速解算 wz**，与 IMU 陀螺 Z
  （offset 18-19，FlagStop≠0 被固件钳零）严格区分、不混用。
- 断流/间隙诊断：`gap`、`nonpositive_dt`、重连重置均打 WARN 日志；
  计数在 DEBUG 日志逐帧输出（accepted/gap/npdt/resets）。
- **速度标度未标定**（轮径/轮距/编码器方向待真车标定），消息数值是固件
  标度原样；手转轮方向复采只证明符号映射，不证明整车偏航绝对方向与标度。

协方差说明：pose 的 x/y/yaw 与 twist 的 vx/vyaw 为**未标定占位值**；
z/roll/pitch/vy/vz/vroll/vpitch 为未测量维度，默认填大方差
（`odom_unmeasured_variance = 1e6`）表达"不可融合"。全部经参数可配置，
任何取值都不是标定结果。下游（如 robot_localization）应只融合
x/y/yaw/vx/vyaw 维度。

## 标准构建与安装（ROS 2 Humble）

```bash
source /opt/ros/humble/setup.bash
cd <本仓库根>          # 即 c50c_odom/ 与 c50c_uart3/ 所在目录
colcon build --packages-up-to c50c_uart3
source install/setup.bash
# 此后无需 PYTHONPATH / 源码目录即可导入并构造消息：
python3 -c "from c50c_uart3.odom_msg import OdomCandidate"
```

注意：`c50c_uart3` 已声明对 `c50c_odom` 的 `exec_depend`，因此
`colcon build --packages-select c50c_uart3` 在 `c50c_odom` 未构建时会报
"Check that the following packages have been built"——用
`--packages-up-to c50c_uart3`（或直接 `colcon build`）即可。

## 已声明限制

1. 帧内无采样时间戳：`header.stamp` 为主机接收时刻（节点时钟），20 Hz 发送
   + 串口/调度延迟不补偿。
2. 上电校准窗口（固件上电前 10 s 采集零偏，期间帧内为未扣零偏原始值）：
   接收端无法判断 C50C 何时上电，节点不猜测窗口是否结束，照实转发。
3. 帧无序列号、BCC 为弱校验（对偶数位翻转不敏感）：丢帧/重复帧不可精确
   检测，仅有帧间隔超时告警。
4. 协方差为占位参数，非标定值。
5. 物理轴向、`FlagStop` 与 PS2 失联的关系、DTR/RTS 对板子的影响均待现场
   验证（见审计报告第 9 节）。
6. 里程计影子输出：vx/wz 速度标度（轮径、轮距、电位器档位）、原地左转时
   wz 的 REP-103 绝对方向均未验收；vy 恒 0 为固件结构零（非测量值，协方差
   标记为不可融合）；帧内无采样时间戳，积分用主机接收时刻、消息戳用节点
   时钟，串口/调度延迟不补偿；全部经默认关闭的开关与影子话题隔离，接入
   正式 /odom 前的清单见 `task-results/c50c-odom-packaging.md`。

## 测试

```bash
# 离线测试（不需要 ROS；在仓库根目录运行）：
python3 -m pytest c50c_odom/test/ \
                  c50c_uart3/test/test_frame_parser.py \
                  c50c_uart3/test/test_conversions.py \
                  c50c_uart3/test/test_decoder_matches_reference.py -v

# ROS 2 Humble 上（额外运行节点集成测试与消息构造测试）：
python3 -m pytest c50c_odom/test/ c50c_uart3/test/ -v \
  -p no:launch_testing -p no:launch_ros -p no:launch_pytest
```
