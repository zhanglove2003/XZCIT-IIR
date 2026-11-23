# C50C Web-Next 桥接协议（`/bridge`）

本文是 `deploy/ros_bridge_shim.py`（板端垫片）与 `src/ros.rs`（Web-Next 桥接层）
之间的线上契约，部署报告直接引用。旧实现对照行号均指旧仓库
`Web/opi_robot/web_node/`（web_debug.py 等）。

## 0. 总览

```
ROS 2 话题 ──订阅──▶ 垫片(bridge_shim.py) ──HTTP──▶ Web-Next(ros.rs) ──▶ 浏览器
   ▲                    │        POST /bridge/frames（上行帧批）      │
   └────发布──────◀─────┘        GET  /bridge/commands（下行命令长轮询）◀── 浏览器 /ws
```

- 两 endpoint 挂 Web-Next 既有 Router（main.rs），**仅 ros 模式挂载**；
  fake 模式路由零变化（`/bridge/*` 404）。
- 鉴权：环境变量 `OPI_BRIDGE_TOKEN`（≥32 hex 随机），两方向请求都带
  `X-Bridge-Token` 头，服务端常数时间比较（safety.rs::ct_eq 模式）。
  401 不区分「令牌错/缺」、不回显令牌（防探测）。Web-Next 绑 0.0.0.0，
  无令牌的裸端点会被同网段任意主机伪造（上行注入假 scan/pose/chassis，
  下行偷走 nav_goal 命令驱动实车），故两方向都必须验令牌。
- 垫片单消费者假设：命令队列只有一个消费者（同板单垫片进程）。
- 日志卫生：token 与 manual_login 密码一律不落日志（延续 ws_control.rs
  脱敏惯例）。

## 1. 上行：`POST /bridge/frames`

请求体：`{"mono_ms": <垫片单调毫秒>, "frames": [帧...]}`

- 单批 ≤ **256 帧**（超出 → `400`）；body ≤ **8MB**（超出 → `413`）；
  JSON 解析失败 / 缺 frames → `400`；令牌错 → `401`。
- hub 侧**逐帧独立处理**：坏帧/未知 t 跳过不拒整批。
- 响应 `200 {"ok":true,"accepted":N,"cmds_pending":bool}` ——
  `cmds_pending=true` 供垫片立即转命令轮询（否则长轮询 20s）。
- 活性：hub 判「>10s 无任何帧批（含 probe）」→ `nodes.web_bridge=offline`。
  窗口（10s）必须大于 probe 周期（3s）并留余量，否则底盘+雷达静默但垫片
  健康的情形会在 online/offline 间周期抖动；旧实现 web_bridge 卡片恒
  online（web_debug.py:2071），节点真死由各传感器视图 stale/unknown 表达，
  绝不冒充实时。垫片进程退出 → 许可断流 → hub 与下游 guard 双保险锁停。

### 1.1 帧字段对照表（frameMap）

| 垫片帧 | 来源话题 / 类型 | 字段 | hub 落点（对齐 fake.rs 写法） |
|---|---|---|---|
| `{t:"img"}` | `/image_raw/compressed` sensor_msgs/CompressedImage，QoS sensor | `d`(b64 原样不重编码), `w`,`h`(魔数嗅探，读不出 null), `mime`(魔数嗅探 image/jpeg\|image/png) | media_slot.image_*（Arc 换新 last-wins；image_seq 自增；image_fps 由 hub 按 2s 窗口实测写常量位） |
| `{t:"scan"}` | `/scan` sensor_msgs/LaserScan，QoS sensor | `p`:[[deg,r]...]（deg=round(degrees(angle_min+inc*i),2)；nan/inf/超程点记 0.0；有效点 round(r,3)）, `n`(有效数), `all`(总点数) | media_slot.scan/scan_valid/scan_seq；scan_hz 由 hub 按 2s 窗口实测（ws_video scan 帧读实测值，不再恒 10.0） |
| `{t:"map"}` | `/map` nav_msgs/OccupancyGrid，QoS=status_qos(1)+**TRANSIENT_LOCAL**（硬前提：qos.py 两个默认都是 VOLATILE，漏改永远收不到 map_server 的 latched 静态图） | `seq`(垫片到达自增；hub 侧钳位单调), `w`,`h`,`res`,`ox`,`oy`, `d`(int8 -1→255 后 zlib+base64，浏览器 DecompressionStream('deflate') 直解) | media_slot.grid（GridPub）+ `current_map_id = sha256(解压后栅格)[:16]`（对齐 fake.rs:225；与旧「地图 yaml 文件 hash」不互通，属可接受偏差，见 risks#8） |
| `{t:"pose"}` | map→base_link TF 0.2s 定时 lookup | `pose:{x,y,th}`（x/y round3，th=四元数→yaw round4）；**lookup 失败不发帧**，hub 保旧值由 pose_age 判新鲜 | media_slot.pose/pose_mono_ms |
| `{t:"chassis"}` | `/diagnostics` DiagnosticArray，QoS status；name∈CHASSIS_NAMES、键名候选 _KEYS（chassis.py:46-57 实测沉淀，含 voltage_mv/battery_mv 别名） | `voltage_mv`,`vx_mm_s`,`vy_mm_s`,`yaw_rate_mrad_s`,`stop_flag`,`ultrasonic_front_mm`(0xFFFF=无回波),`level`,`message`（全部可 null） | Raw.chassis → 20Hz 装配 → ChassisView（CHASSIS_TIMEOUT=2.0s；stop_text/drive_ready 按 chassis.py:177-185；wheels=四驱反解 chassis.py:125-140）；**里程积分**（trapezoid、丢帧>0.5s 不计入，chassis_status.py:59-89）；chassis_hz 1s 窗口实测。解析失败垫片不发帧 → hub 保持上一次记录不清空 |
| `{t:"lidar_diag"}` | 同上，name=='lidar'（sensors.py:39-63） | `level`,`message`,`state`,`reason`,`age_s`,`coverage`,`reconnects`,`port`,`frames_ok`,`frames_bad` | Raw.lidar_diag → status.lidar_detail（DIAG_TIMEOUT=3.0s 内附 diag_state） |
| `{t:"imu"}` | `/imu/data_raw` sensor_msgs/Imu，QoS sensor；**只做心跳不读数值** | `hz`(垫片本地计数、1s 窗口实测、1Hz 聚合一帧) | Raw.imu → ImuView{state,age,hz}（IMU_TIMEOUT=2.0s） |
| `{t:"cmd"}` | `/nav_cmd_vel` `/demo_cmd_vel` `/manual_cmd_vel` `/auto_cmd_vel` Twist，QoS sensor×4 | `which`:"nav\|demo\|manual\|auto", `vx`,`vy`,`wz` | Raw.cmds[which] → CmdView（CMD_TIMEOUT=1.0s）；which=='nav' 刷新 nav_seen_t。**注意**：垫片自己发 manual_cmd_vel 会收到回声——这正是旧 cmd.manual 卡片的数据源，无害且必需，hub 落库并存不过滤 |
| `{t:"drive_safety"}` | `/drive_safety` std_msgs/String，QoS status | `raw`(JSON 解析成功为 dict，失败为 null) | Raw.drive_safety → DriveSafetyView：hub 按 `raw.ttl_s`（缺省 1s）做新鲜度，过期 state='stale'、allowed=None——过期许可不等于允许也不等于拒绝（web_debug.py:2027-2052） |
| `{t:"moving"}` | `/robot/moving_bins` std_msgs/UInt8MultiArray，QoS status；**类型不可用时垫片跳过订阅，点云照常** | `bins`:[0..119 每 3° 索引] | status.moving（2s 新鲜度） |
| `{t:"probe"}` | 垫片 3s 合成：图 API（camera/lidar 按 get_publishers_info_by_topic 判节点名；**chassis_status_node 本车栈无此发布者（/diagnostics 实测 7 发布者无它，2026-11-21），活性改挂 c50c_uart3_node 参数服务存活，与底盘数据超时同源**）+ 参数服务（c50c_uart3_node，2s ListParameters/GetParameters，6s 内可达=在线） | `nodes_seen`:{camera_node,lidar_node,chassis_status_node(bool)}, `safety`:{drive_enable,drive_input_enable,drive_firmware_protocol,online,source,checked_t} | status.nodes（camera/lidar/chassis 6s 窗口；c50c_uart3_node=safety.online）+ status.safety（Value 透传）。**probe 周期必须小于 hub 的 10s 桥活性窗口**。另：/scan 源实测 ~100Hz，垫片节流到 20Hz 喂 UI（桶驻留渲染足够，省掉全速率逐点转换），单元 CPUAffinity=0 1 2 3 钉小核给 nav2 让大核 |
| `{t:"nav_result"}` | 垫片 NavigateToPose 回调 | `rid`(原样回传), `status`:"accepted\|succeeded\|cancelled\|failed", `text`(4=到达目标/5=已取消/6=导航中止（规划/控制失败）), `plan_error`(Nav2 error_code 文本) | hub nav/巡逻状态机：rid==在途目标时 accepted→pending、succeeded→index+1（回绕 laps+1）并派发下一航点；failed/cancelled→锁停；迟到/被顶替/锁停中的 accepted 一律立即取消；15s 在途未接受超时锁停 |
| `{t:"nav_feedback"}` | NavigateToPose feedback | `rid`, `dist`(distance_remaining round2), `eta`(estimated_time_remaining round1), `x`,`y`(current_pose round2) | Raw.nav_feedback → status.nav.feedback/feedback_age |
| `{t:"cmd_ack"}` | 垫片对需确认命令的回执 | `id`(命令 id), `c`:"ultrasonic_rearm"\|"cruise_enable", `ok`, `error` | resolve 等待中的 oneshot：ultrasonic_rearm 失败→页面如实显示（不假装已恢复）；cruise_enable 失败→manual_error、状态不落 |
| `{t:"bridge_hello"}` | 垫片（重）连成功后首帧 | `topics`:{subscriptions[],publishers[],moving_bins,tf,nav_action} | web_bridge 活性刷新 + **清空未投递命令队列** + 重置出口意图（由新许可流自然重新武装 guard，避免旧积压错序） |

### 1.2 「话题无数据时如实置 null」

- img 的 `w`/`h` 嗅探失败 → null；
- drive_safety 解析失败 → `raw:null`（页面显示无许可）；
- chassis/lidar_diag 各数值字段解析不出 → null（键名候选全不命中）；
- TF lookup 失败 → 整帧不发（不是发 null 位姿——hub 保旧值，pose_age 表达）；
- probe 的 safety 值读不到 → null + source 注明原因。

## 2. 下行：`GET /bridge/commands?wait_ms=20000&cursor=N`

- 长轮询：有命令即时返回（tokio::sync::Notify 唤醒，不等满 wait_ms），
  无命令等满 wait_ms 返回 `{"cmds":[],"cursor":N}`（wait_ms 钳到 30s）。
- 响应 `{"cmds":[...], "cursor":M}`；命令带单调递增 id（命令体含 `id`
  字段），环形保留最近 **256** 条。
- cursor 语义：垫片回传上次响应的 cursor。**按 cursor 补投仅限值语义
  幂等命令**；边沿触发命令投递一次即出队，**永不补投重放**。
- 垫片（重）连发 bridge_hello 时 hub 清空**未投递**队列——由新许可流自然
  重新武装 guard，避免旧积压错序（license_forward/guard_authorize 错序
  重放会让下游 guard 锁存误判，web_debug.py:910-914）。

### 2.1 命令对照表（commandMap）

| 命令 | 目标话题 / 类型 | 语义 | 补投 |
|---|---|---|---|
| `{c:"manual_cmd",id,vx,wz}` | `/manual_cmd_vel` Twist（status_qos） | 手动最新命令 (vx,wz)；垫片承担 0.05s tick、0.10s 重发周期；左转=负 angular.z 符号约定在 hub（safety.rs:653-660） | 是（最新值生效） |
| `{c:"manual_zero",id}` | 同上 | 停止即发零。**派生点**：manual_stop handler + 20Hz manual_tick 的 350ms 心跳超时/timed 到期（意图对账），否则垫片 10Hz 重发循环会把最后非零速度无限期重发（旧实现 tick→_stop 在停车瞬间立即发零，manual_forward.py:199-210→159-174） | 是（幂等） |
| `{c:"license_forward",id,frame:{session,seq,expires_mono}}` | `/motion_license` std_msgs/String（status_qos） | 每条被接受的许可（100ms 一条）转发 JSON（紧凑分隔符）。**seq 原样转发**（防重放校验完全靠 seq，web_debug.py:649-654,909-924）；`session` 为 **int≥1**——浏览器会话 hex 字符串（蓝图 §4.3(1)）到下游 guard 协议（motion_guard.py License.check 要求 int，字符串一律 session_invalid 拒收）的稳定映射：同一浏览器会话恒映射同一 int，页面接管（新 hex）分配新 int；`expires_mono` 为 **boot 相对单调秒**（/proc/uptime 锚 + ttl——guard 用 time.monotonic() 比较，Web-Next 进程 uptime 与之无关）| 是 |
| `{c:"guard_authorize",id,value}` | `/guard_authorize` std_msgs/Bool | 按值去重（web_debug.py:900-906）。True 方向=每条被接受的许可（接受即伴随，guard 初始 unauthorized，can_write 须 authorized 才放行写串口——启动首许可、巡航续期、手动接管后恢复全靠这条）；False 方向=锁停/会话失效/断连/motion_stop（**派生点主要在 20Hz tick**：license_tick/session_tick/conn_closed） | 是 |
| `{c:"rearm_request",id}` | `/rearm_request` std_msgs/Bool | motion_rearm 成功后发出，**必须与 guard_authorize(True) 同批按序**（先补鉴权再发解停，否则下游 preconditions_ok 永假，web_debug.py:555-560）。一次性边沿触发：下游「请求一次即消费、无论成败清标记」（guard_bridge.py:258-261），**永不补投重放**——重放陈旧请求会在无用户动作时解锁下游 guard | **否** |
| `{c:"ultrasonic_rearm",id}` | `/ultrasonic_rearm` std_msgs/Bool | 单发一帧，边沿触发语义（每次发布清 fusion 超声门无效锁）；垫片发布失败须经 cmd_ack 回 false 如实显示（不假装已恢复，web_debug.py:570-580） | **否** |
| `{c:"cruise_enable",id,on}` | `/cruise_enable` std_msgs/Bool | 开关变化即发一帧，无周期（web_debug.py:150-164）；垫片 ack 发布成败，失败→manual_error、状态不落 | 是 |
| `{c:"nav_goal",id,rid,x,y,yaw}` | NavigateToPose action | 目标构造逐字对齐 nav_goal.py:128-150：frame_id='map'、**stamp=零时间戳**（Nav2 据此取最新 TF 变换）、yaw→quaternion(z=sin(θ/2),w=cos(θ/2))、wait_for_server 2.0s；巡逻派发与手动点选共用同一在途目标槽（在途未结时新目标拒绝，nav_goal.py:106-111）；hub 侧范围校验 X∈[-8,22]、Y∈[-8,8] + 距车 <0.3m 拒绝（间距常量在旧代码是死值，契约 risks#7 落成真校验） | **否** |
| `{c:"nav_cancel",id,rid}` | 同上（cancel_goal_async） | 取消在途目标；取消应答失败垫片回滚 active（nav_goal.py:366-384），不发结束帧——hub 保持锁停等真结果 | **否** |
| `{c:"patrol_status",id,frame:{...}}` | `/patrol_status` std_msgs/String（status_qos） | hub 每 ~1s 捎带最新 patrol_status_frame（safety.rs:898-954 同构帧）；垫片以 10Hz 重发缓存帧（web_debug.py:357-366）；只保留最新一帧（槽位顶替）；页面直推仍由 ws_control 既有推送承担 | 槽位（只发最新） |
| `{c:"relocalize",id,x,y,yaw}` | `/initialpose` geometry_msgs/PoseWithCovarianceStamped（status_qos(1)） | 画面当前位姿整包（x/y/yaw 雷达朝向）→ 垫片发布 /initialpose 让 AMCL 重新播种。构造逐字对齐旧栈 web_debug.py::relocalize_pose 752-765：frame_id='map'、**header.stamp 保持零时间戳**（AMCL 对零时间戳按"最新"处理，写本机时钟触发 Lookup extrapolation，历史坑 nav2_params_20261120.yaml:35-43）、yaw→quaternion(z=sin(θ/2),w=cos(θ/2))、cov[0]=cov[7]=0.09, cov[35]=0.0685。一次性边沿触发：AMCL 采纳即消费，重放同帧无意义；web_next 侧 relocal_busy 防重入 | **否** |

### 2.2 垫片侧节律（两个 10Hz 重发循环）

- manual：0.05s tick / 0.10s 发布周期（manual_forward.py:18 与
  web_debug.py:1242）；manual_zero 停止即发零一次。
- patrol_status：10Hz 重发 hub 缓存帧（web_debug.py:357-366）。

## 3. 出口意图对账（hub 侧派生模型）

hub 维护 last_emitted 出口意图（authorize 值 / manual cmd 值 / cruise 值 /
in-flight nav rid），**所有 Control 变更点**写意图：

- handler 即时 flush 关键序：license 接受即 license_forward +
  guard_authorize(True)（ws_control.rs handle_license Ok 分支）；rearm 同批
  authorize→rearm_request（handle_motion_rearm Ok 分支）；manual_start 即
  manual_cmd；
- 20Hz tick 末尾统一对账差量派发：manual_tick 清零 manual.cmd 后对账即发
  manual_zero；license_tick/session_tick 失效对账即发 authorize(False)
  （对齐旧实现——会话失效的第一动作就是撤下游鉴权，web_debug.py:741-743）；
- conn_closed（最后一个实时页连接走掉）同样经对账派生。

20Hz tick 还承担（契约 risks#2，安全项不是优化项）：license_tick /
manual_tick / session_tick 三个看护（ros 模式 FakeEngine.tick 不跑，必须
独立 tokio 任务）+ 15s 在途未接受超时锁停 + 巡逻航点派发 + patrol_status
捎带 + Raw → StatusPayload 装配。

## 4. 已知偏差（部署说明必须记录）

1. **map_id 语义**：旧实现用地图 yaml 文件 sha256 前 16 位绑定路线
   （web_debug.py:185-193）；Web-Next 用栅格内容 sha256（fake.rs:225），
   ros 模式沿用栅格内容 hash。与旧 patrol 持久化路线不互通。
2. **expires_mono 折算与 session 映射**：浏览器协议发 ttl_ms 与 hex
   字符串会话号，下游 guard JSON 需要 expires_mono（time.monotonic()
   boot 相对秒）与 int≥1 会话号；hub 以「/proc/uptime 锚 + ttl」折算
   expires_mono，session 经稳定映射表（hex→int，接管换号）转换，
   seq 原样转发。/proc/uptime 不可读时退回进程 uptime 锚并告警
   （仅非 Linux 环境；实车 Linux 一定可读）。
3. **goal 距离校验**：nav_goal.py:31 GOAL_MIN_DIST_M=0.3 在旧代码是
   未被引用的死常量；ros 模式按契约 risks#7 落成真校验。
4. **relocalize 无 ROS 出口 → 已接真链路**（2026-11-22 修复）：旧
   web_node/ 全目录无 relocalize 接线（grep 零命中）；relocalize.py 依赖
   NumPy，与 stdlib-only 垫片冲突，全局扫描匹配算法不移入垫片。**现在的
   真实现走「当前位姿重播种」**：前端把画面位姿整包 (x,y,yaw) 随 relocalize
   消息上传，hub 入队边沿命令，垫片发布 /initialpose（格式照旧栈
   web_debug.py::relocalize_pose，见 §2.1 relocalize 行）让 AMCL 重新播种
   收敛。**语义边界**：这修的是"在有雷达确认的位置重拾粒子"；若 AMCL 已
   严重漂移需从零找回位姿，需要搬 relocalize.py 的 numpy 全局扫描匹配
   （另一档改动，未做）。
5. **网络/主机信息**：host 仍为 null（旧 proc 采集不迁移）；network 由 Web-Next 的 netinfo（src/netinfo.rs）Rust 侧 5 秒周期采集，不进垫片，协议帧形状不变。
6. **/guard_status 订阅不迁移**：旧实现从未消费，不在 100% 兼容基线内。
7. **垫片单消费者**：命令队列假定单垫片进程；多垫片并发轮询时边沿命令
   可能被重复投递（部署约束：一台板一个垫片）。
