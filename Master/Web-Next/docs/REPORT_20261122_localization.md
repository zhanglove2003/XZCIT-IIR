# C50C / XUCIT-IIR Web-Next 定位链 — 现场问题报告（供其他智能体诊断）

> 生成时间：2026-11-22 20:20 CST
> 现场：Orange Pi 5 Pro（aarch64, 8 核, 8 GB），ROS 2 Humble，实车 C50C。
> 你们看到的是一台**真实在跑的车**，不是仿真。任何全局重定位/播撒位姿都会影响真车运动许可与安全链。

---

## 0. 一句话问题陈述

Web 控制台上的「重定位」按钮已从桩改成**真·全局扫描匹配 → 发布 `/initialpose` 重播 AMCL 粒子**（移植旧栈纯 NumPy `relocalize.py`）。现在整条链路能跑通，但暴露四个互相纠缠的问题，需要外部判断：
**(A) 匹配器给出的位姿与用户肉眼确认的物理位置不一致，且证据两边都自洽；(B) AMCL 在静止不动时也会像被外部喂位姿一样跳变；(C) 一旦把垫片切到 CycloneDDS，AMCL 的 TF 链在 3 分钟后断（`map` 帧消失）；(D) 即使走 FastDDS，所有 ROS **服务**调用（参数/生命周期）在本车 TF 全空时也全部超时。**

---

## 1. 系统架构与拓扑

```
              ┌──────────────────────────── Orange Pi 5 Pro ────────────────────────────┐
 浏览器 ──►  c50c-web-next.service    :8088  (Rust axum, 单文件 aarch64 musl)
              │   ▲ /bridge HTTP 上行 POST /bridge/frames，下行 GET /bridge/commands 长轮询
              ▼   │
            c50c-web-bridge.service  ros_bridge_shim.py (Python rclpy 垫片)
              │  RMW: 现为 rmw_cyclonedds_cpp（A/B 试验中），其余节点全为默认 rmw_fastrtps_cpp
              ▼
   ┌──────────── ROS 2 Humble 图（全部 FastDDS 默认）────────────┐
   │ c50c_uart3_node (2172)  /c50c_uart3/*  参数服务(RELIABLE)      │
   │ fusion_node     (1629)  /odom(空话题!) /imu /battery...       │
   │ lidar_node      (1617)  /scan  100 Hz, 360 束, -180..179°     │
   │ camera_node     (1615)  /image_raw                            │
   │ static_transform_publisher ×3: base_link→laser (0.08,0,0.22,无旋转) 等 │
   │ c50c-ekf.service  → 发布 odom→base_link TF (?)                │
   │ c50c-localization.service: map_server + amcl(2240) + lifecycle_manager_localization │
   │ c50c-nav2.service: controller/planner/bt_navigator/smoother/behavior/waypoint/velocity_smoother │
   └────────────────────────────────────────────────────────────┘
```

机器：`/etc` 内 `RMW_IMPLEMENTATION` 未设置 → 所有节点 = `rmw_fastrtps_cpp`。
垫片 systemd 单元里被我们插入 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（`.bak-rmw` 有备份）。

### 关键文件
| 作用 | 路径 |
|---|---|
| Web-Next 仓 | `/mnt/data/project-ai/Embedded_Development/Antigr/repo-push/XZCIT-IIR/Master/Web-Next` |
| 垫片源 | 仓内 `deploy/ros_bridge_shim.py`，板端 `/home/orangepi/web_next_deploy/ros_bridge_shim.py` |
| 旧栈移植匹配器 | 仓内 `deploy/relocalize.py`（504 行，纯 NumPy，0 scipy） |
| 定位参数 | `/home/orangepi/maps/c50c-field-20261120/localization_params.yaml` |
| AMCL 日志 | `/tmp/nav2_loc.log` |
| 垫片日志 | `/tmp/wbbridge.log` |
| 融合节点日志 | `/tmp/fusion.log`（17 MB，刷屏级） |
| 旧栈参考实现 | `ROS/ros2_ws/src/opi_robot/opi_robot/web_node/{web_debug.py, relocalize.py}` |

---

## 2. 实测硬件/数据事实（都可复现）

| 项 | 值 |
|---|---|
| `/scan` | 100 Hz, 360 束, `angle_min=0.0 rad`, `angle_increment=0.017453 rad (1°)`, `range_min=0.02`, `range_max=12.0`, QoS BEST_EFFORT |
| `laser` 安装 TF | `base_link→laser: x=0.08, y=0, z=0.22, qx=qy=qz=0, qw=1`（无旋转） |
| `/map` | 394×278, `res=0.05`, `origin=(-8.36,-8.37)`, spans x∈[-8.36, 11.34], y∈[-8.37, 5.53] |
| `/odom` | **无发布者**（`ros2 topic hz /odom` 报 does not appear to be published） |
| `/tf` | 在 FastDDS 默认 CLI 下 `ros2 topic hz /tf` 无输出 |
| AMCL QoS on /scan | BEST_EFFORT, 队列大量 `Message Filter dropping message ... queue is full`，**累计 5578 次**（约每 3 s 一次） |
| 资源 | 8 核, load 7.25, 垫片 104% CPU, uart3 47%, fusion 44%, lidar 33%, camera 28%, amcl 11% |
| AMCL 参数 | `set_initial_pose:true, initial_pose(0,0,0)`, `min_particles 500, max_particles 2000`, `sigma_hit 0.2`, `z_hit 0.7, z_rand 0.2`, `update_min_a 0.1, update_min_d 0.1`, `laser_max_range 10.0`, `transform_tolerance 1.0`, `max_beams 90` |

---

## 3. 问题 A：全局匹配器给出 (0,0)，用户说车在地图中央且「应该在别处」

### 3.1 匹配器实现（`deploy/relocalize.py`）
- 三阶段：72 个 yaw（5°）全图粗搜（候选栅格 stride 4，需 clearance ≥0.18 m）→ 前 10 峰各 ±0.25 m/±6° 精搜 → 前 2 峰 ±0.06 m/±2° 微调。
- 打分：`LUT = exp(-0.5*(d/σ)²)`，`σ = max(0.15, 2.5*res) = 0.15`；score = 所有有效束 LUT 值均值 − 中段撞墙惩罚。
- `hit_ratio` = 束端点落在 `dist<=0.125 m`（`max(σ,2.5*res)`）内的比例；门槛 `MIN_HIT_RATIO = 0.35`。
- **只用「束端点是否落在障碍附近」打分，不用空间分布/可见性/自由空间一致性。**

### 3.2 现场结果（同一次点击，垫片日志原样）
```
[bridge_shim 20:02:33] relocalize 全局匹配：x=-0.035 y=0.050 yaw=0.0175 命中率=89% 匹配=23397ms
```
随后 amcl 日志：
```
[INFO] initialPoseReceived
[WARN] Failed to transform initial pose in time (Lookup would require extrapolation into the future. Requested ... latest data ... looking up transform from frame [base_link] to frame [odom])
[INFO] Setting pose (...): -0.035 0.050 0.017
```

### 3.3 用户判定
> "我的小车在地图中央的……雷达匹配是这个地方？？？"（用户看过地图，认为车不在 (0,0)）
> 之后用户手动把它摆到 (2.44, -3.32) 并说 "位置差不多了，精度还有点问题"。

### 3.4 我做的独立验证（本地离线复算，同帧数据）

把板端 `/map` 栅格和**一帧真实 `/scan`** 导出到本地，用同一份 `relocalize.py` 复算：

| 候选 | score | hit_ratio | 在 ±0.35 m/±12° 邻域内局部最优？ |
|---|---|---|---|
| 匹配器 (x=-0.035, y=0.050, yaw=0.0175) | **0.9063** | **92.4%** | 是，邻域搜到的最优点就是它本身（位移 0） |
| 用户 (x=2.44, y=-3.32, yaw=0.077) | 0.3208 | 30.3% | 局部最优在 (2.09,-3.67)，score 0.4566, hit 43.8% |

雷达点叠加图（红色=匹配器位姿，蓝色=用户位姿，绿圈=车体，绿色短线=朝向）：
- `/tmp/ov_MATCH.png` — 红点几乎每一条都压在墙体像素上，轮廓与墙线严丝合缝。
- `/tmp/ov_USER.png` — 蓝点大部分悬在空白区，只有少数几条贴墙。

### 3.5 矛盾点（需要外部判断）
1. **数据层面匹配器完胜（92% vs 30%，且 92% 在整幅图上都是最优点）**；若匹配器错，说明这份 `/map` 与这份 `/scan` 的「正确」对位就是 (0,0)，即**地图本身把车画在了原点**。
2. 但用户是看着**界面地图**说车在中央的。`/map` 的几何中心 = (1.49, -1.42) —— 差得很远，用户的"中央"是**渲染视口中央（即机器人箭头本身）**，这会形成自证循环：箭头在视口中心 ≠ 车在物理场所中心。
3. 也可能是：激光安装角/车体朝向定义不一致、或激光与车体中心偏移（0.08 m 不足以解释 2.5 m），或**当前这帧 scan 是在车被挪动后采集的**（用户挪车时间线未确认）。
4. **需要外部给出一个与位姿无关的地面真值手段**（例如：把车推到地图上已知的地标如墙角/门口，量取地图坐标；或检查地图 `.posegraph` 里建图时 SLAM 轨迹的起点是否被定义成原点 `initial_pose(0,0)`）。

> 附：`localization_params.yaml` 里 `set_initial_pose: true, initial_pose: (0,0,0)`，`always_reset_initial_pose: false`，`first_map_only: false`。**建图时 SLAM 起点就是 map 原点是非常常见的**，如果建图开始时车就在这间屋子的中央，那 (0,0) 恰好是设计原点而非巧合——请帮我们判断这点。

---

## 4. 问题 B：车静止时 AMCL 会「自己跳位」

AMCL 日志（车没动，无新 `/initialpose` 也应保持不变）：
```
20:12:02  Setting pose:  0.000 0.020 0.017
20:13:09  Setting pose: -0.030 0.020 0.026
20:13:59  Setting pose: -0.015 0.035 0.017
20:16:13  Setting pose: -0.035 0.050 0.017
```
每次都伴随：
```
Failed to transform initial pose in time (Lookup would require extrapolation into the future.
 Requested time T.xxx  but the latest data is at time T.xxx-0.035,
 when looking up transform from frame [base_link] to frame [odom])
```
观察：
- 跳幅 1–3 cm，方向朝 (0,0) 收敛；朝向 1.0–1.5°。
- 消息是 `initialPoseReceived` → 说明**确有 `/initialpose` 到达**。已确认：
  - 我在垫片里发布的 `/initialpose` 用 **零时间戳**（这是旧栈趟过的坑，真实时钟会触发同样的 extrapolation 警告）；
  - 但日志里的 `Requested time` 是**真实墙钟**，说明存在**别的发布者**在用真实时间戳发 `/initialpose`（可能是某个 relay/遥操作/启动脚本，尚未定位；`ros2 topic info /initialpose` 曾显示 Publisher=1，需再次核对 publisher 的 node 名）。
- `Failed to transform initial pose ... looking up base_link→odom` = **倒查链路 `map→odom→base_link` 的 odom 段时序不对**：即 odom 侧 TF 比 scan 时间戳晚 30–40 ms 且没有外推容差。
- 可疑根因（待确认）：
  1. `c50c-ekf` 输出的 `odom→base_link` 频率低于 scan（100 Hz），`transform_tolerance 1.0` 本应兜住 35 ms，但报错说明**查的是 `base_link→odom` 的反向**且 AMCL 的 initialpose 回调用了 `rclcpp::Time::now()` 而不是零时间戳语义；
  2. `fusion_node` 的 `/odom` 无发布者 → EKF 可能吃的是另一路（`/c50c_odom/*`）；若 EKF 掉帧，odom TF 会中断，AMCL 就会在 `Message Filter` 里丢帧（5578 次）；
  3. 第 2 点叠加「静止时 AMCL 仍以 1 Hz 重采样」，粒子在 likelihood_field 上会做小幅漂移 —— 但**漂移不会产生 `initialPoseReceived` 日志**，所以仍指向有外部发布者。

---

## 5. 问题 C：切 CycloneDDS 后 AMCL / TF 链在 ~3 分钟断（`map` 帧消失）

### 5.1 背景：为什么切 CycloneDDS
垫片走 FastDDS 时每 ~2 min 出现一次：
```
RCLError: publisher's context is invalid   （rclpy 上下文坏死）
```
同时 `/map` 收不到（前端"地图等待中"）、许可心跳中断（自动锁停 `license_expired`）。这是**坑 19**，进程内不可修，只能重启。

A/B 试验：给**垫片这一个进程**设 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（其余全部 FastDDS）。

### 5.2 结果（20:01:55 起，12 min 后）
| 指标 | CycloneDDS 下 |
|---|---|
| `收到首帧 /map：394x278 res=0.050` | ✅ 立即恢复 |
| `map_id: ec9c04124c29d63e`, `chassis: fresh hz 20.3` | ✅ |
| `context is invalid` | ✅ 0 次（启动后无） |
| `uart3: offline` | ❌ 持续，每 10 s `参数探测超时 10s 无回调，重置后重试` |
| **20:15 之后** `ros2 topic echo /amcl_pose` | ❌ 无任何输出 |
| `ros2 run tf2_ros tf2_echo map base_link` | ❌ `Invalid frame ID "map" ... frame does not exist`（TF 全空） |
| `ros2 topic hz /tf`、`/odom` | ❌ 空 |
| `/scan` | ✅ 仍 100 Hz, 6 订阅者 |

`amcl` 进程仍活着（`Sl`, 10.9% CPU, 已运行 1 天 10 小时），只是**不再发 TF**。

### 5.3 需要外部判断
- 这是 FastDDS（服务端）与 CycloneDDS（客户端）**跨 RMW 互操作**的典型症状吗？ROS 2 Humble 的 standard-supported pairwise 测试**不含** FastDDS↔CycloneDDS 组合。
- **服务调用全灭**（`list_parameters` / `lifecycle get_state` 在两种 RMW 下都超时）是否也属同源？
  - 补充对照：用 **CycloneDDS 的 CLI** 调 `/c50c_uart3_node/list_parameters`（目标节点是 FastDDS）→ 同样"making request"后静默超时。
- 现象是否与「TF 全空」互为因果：AMCL 在 `map→odom` 发布上卡死后，所有依赖 TF 的节点（含参数服务的 quality-of-service 探测？）都失联？
- 我们的**首选方案**其实是「全线统一 RMW」（全部节点都换成 CycloneDDS 或全部 FastDDS），但 `dds-watchdog.service`、`c50c-chassis.service` 等注入式启动脚本里有各自的 export，需要评估统一的风险与迁移顺序 —— 请给推荐顺序与回滚点。

---

## 6. 问题 D：匹配耗时 23.4 s 远超前端 12 s ACK 窗口

- 实测 `lut_ms + match_ms`：首次点击 **23397 ms**（LUT 构建 + 72 yaw × ~2k 候选 × 90 束粗搜）。
- `RELOCALIZE_ACK_TIMEOUT_MS = 12000`，导致 UI 弹「垫片未在 12000ms 内确认命令」而后台其实成功了。
- 期望目标：**≤ 5 s**，让用户在车上就能等。
- 现成的加速点（未实施，等判断）：
  1. 粗搜用 `np.argpartition` 提前把候选砍到 ~500（stride 6），先粗后排；
  2. 用 `scipy.ndimage.distance_transform_edt` 替自研 EDT（**板端可能没有 scipy，需确认**）；
  3. 打分矩阵改用 `float32` + `np.lib.stride_tricks` 分块，或把最多 72 yaw 的 inner loop 向量化成 3D 张量；
  4. 粗搜阶段束数降到 60，精搜维持 180；
  5. 长期方向：按语言偏好（**C/C++/Rust，Python 仅为 rclpy 垫片不可替代**）把这套匹配器写成 **Rust** 独立二进制/子进程，通过 stdin/stdout 与垫片交换 PGM+scan —— 这是用户明确期待的方向。

---

## 7. 其它观察（可能相关）

1. **`/odom` 是空话题** —— `fusion_node` 实际上不发布 `/odom`（EKF 输入/输出路径需要重新核对，可能只有 `/c50c_odom/*` 与 `odom→base_link` CF）。
2. **CPU 已饱和**：load 7.25/8 核；垫片 104%（rclpy 全速率 scan JSON 序列化），camera_node 28% 一直在跑。在共享核上跑 33 s 的纯 NumPy 匹配会拖慢其它节点 → 也可能是 `Message Filter ... queue is full` 的部分诱因。
3. **AMCL 在 `queue is full` 上累计 5578 次**，说明 `/scan` 端到端延迟已逼近 2 s 队列上限。
4. **`/initialpose` 存在零时间戳 + 真实时间戳混合**（见问题 B）；旧栈经验是「零时间戳 = AMCL 采纳关键」，若存在别的发布者用真实时间戳，会持续制造 extrapolation 警告。
5. 浏览器缓存坑：`index.html` 无 cache-control，刷新拿到旧文档导致 `#toasts` 缺失、按钮点了无反应（已用 `?cb=...` 绕过，非当前问题）。

---

## 8. 请帮我们做什么（按优先级）

1. **A**：给出「判断 (0,0) 与 (2.44,-3.32) 哪个是物理真值」的**可执行实验设计**（不依赖当前 AMCL），并说明匹配器 92% 高命中率是否为"过拟合空白区"的典型假象；若要加固，用什么指标（如端点空间分布熵 / 自由空间可见性检验 / 双向一致性）。
2. **C**：RMW 互操作与「AMCL 停发 TF」的因果关系，以及**统一 RMW 的最小风险迁移顺序**（含 watchdog 注入点）。
3. **B**：定位那个"用真实时间戳发 `/initialpose` 的发布者"，以及 `base_link→odom` 反向查询 35 ms 外推失败的根因与修法（`transform_tolerance` 是否对 `initialPoseReceived` 路径生效？）。
4. **D**：匹配器**≤5 s** 的最短路径（是否值得直接 Rust 重写；若 Rust，给出 I/O 契约）。
5. **D'**：垫片 104% CPU 的优化（scan 序列化降频/批量），以及 AMCL `queue is full` 的 QoS 建议。

---

## 9. 现状约束（请勿违背）

- **不要批量热改 AMCL 参数**：之前一次性设 7 个参数，位姿从 (2.44,-3.32) 崩到 (0.037,-0.062) 再 (-0.183,-1.198)，已全部回滚到 yaml 原值。一次只动一个变量并验证收敛。
- **不要在后台跑无人值守的调参命令**（用户明确禁止）。
- 任何会发布 `/initialpose` 或 `reinitialize_global_localization` 的命令都可能让真车粒子收敛到错误位置、进而影响运动许可；必须先确认车体旁无人、且处于锁停态。
- 命令超时上限 45 s。
