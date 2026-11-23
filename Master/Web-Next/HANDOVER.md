# C50C Web 控制台（Web-Next）交接文档

> 交接日期：2026-11-22。写给接手这台车 Web 控制台的下一个工程师 / AI。
> 当前板端状态：**Web-Next 在跑（8088 归 c50c-web-next.service + c50c-web-bridge.service），旧 web_debug 已停用停 enable**，
> 想回退旧栈见 §4。本文档覆盖新栈的架构、部署、回退、协议、坑位与验收方法。

---

## 1. 现状速览（2026-11-22 交接时）

| 组件 | 状态 | 说明 |
|---|---|---|
| `c50c-web.service`（旧 web_debug，site-packages） | **masked（2026-11-22）** | 单元文件改名 `.masked-20261122` 保留 + `/dev/null` 链接占位；回退须先 unmask（见 §4）。文件本体未动 |
| `c50c-web-watchdog.service` | **stopped + disabled** | 旧栈随从，随旧栈整体停用；其 timer 已一并 mask（start_web.sh 会重新拉起 timer，不 mask 会复活） |
| `c50c-web-next.service`（Web-Next axum 二进制） | **active + enabled，占 8088** | 2026-11-22 切换，现跑的是它 |
| `c50c-web-bridge.service`（rclpy 垫片） | **active + enabled** | 与 web-next 成对启停 |
| 板端部署目录 `/home/orangepi/web_next_deploy/` | 完整保留 | 二进制/垫片/单元/协议文档/旧单元备份 |

旧栈与新栈**只经 8088 端口互斥**，运动链路（nav2/uart3/底盘）与两者都无耦合，切换不需要停导航。

---

## 2. 架构总览

```
浏览器（零依赖前端，include_str! 编进二进制）
   │  /ws（控制 JSON 帧） + /ws/video（视频） + /api/state /api/wslog
   ▼
web_next（Rust axum + tokio，aarch64 musl 静态单文件，:8088，ros 模式）
   │  /bridge HTTP：垫片 POST 帧批（≤256帧/批）+ 长轮询 GET 命令（wait_ms=20000）
   │  鉴权：X-Bridge-Token 共享令牌（防局域网伪造帧）
   ▼
ros_bridge_shim.py（板端垫片：python3.10 stdlib + rclpy，无第三方依赖）
   │  订阅：/scan /image_raw/compressed /diagnostics /drive_safety /map(TRANSIENT_LOCAL)
   │        nav_cmd_vel demo_cmd_vel manual_cmd_vel auto_cmd_vel /robot/moving_bins /imu/data_raw
   │  发布：manual_cmd_vel /patrol_status /cruise_enable /ultrasonic_rearm /guard_authorize
   │        + NavigateToPose action client + 参数服务探测 c50c_uart3_node
   ▼
ROS 2 Humble 实车栈（nav2/amcl/ekf/c50c_uart3/camera/lidar/fusion 全在线）
```

- 许可仲裁、会话、巡航状态机、wslog 全在 web_next 的 Rust hub（safety.rs）里，垫片只做话题 I/O 与消息转换。
- 伪代码协议源：`deploy/BRIDGE_PROTOCOL.md`（逐帧契约，含 QoS 与超时语义）——**改协议前必读**。
- fake 模式（`--fake` / `OPI_WEB_FAKE=1`）与 ros 模式共用同一套 hub，`--fake` 仅供本地开发，板上**不得**带 --fake 跑。

---

## 3. 关键路径与凭据

### 本机（开发机，x86_64）

| 项 | 路径 |
|---|---|
| 源码根 | `/mnt/data/project-ai/Embedded_Development/Antigr/repo-push/XZCIT-IIR/Master/Web-Next` |
| 后端 | `src/{main,netinfo,audio,sysinfo,fake,safety,state,ros,ws_control,ws_video}.rs`（ros.rs 是桥接注入层；netinfo.rs 是实车 WiFi 采集器，main.rs 启动、ros.rs 注入 /api/state 的 network 对象；audio.rs 是音频设备探测——喇叭/板载麦克风挂 es8388、摄像头 USB 麦，同样 5s spawn_blocking，注入 `audio` 对象；sysinfo.rs 是系统占用采集——CPU /proc/stat 差分、GPU devfreq fb000000.gpu/load、内存 meminfo、磁盘 df -P /，注入 `sys` 对象） |
| 前端 | `web/`（index.html + styles/ + js/，经 include_str! 编入，改完必须重新编译二进制才生效） |
| 垫片源码 | `deploy/ros_bridge_shim.py`（md5 `ca21c078…`，与板端一致） |
| 单元模板 | `deploy/c50c-web-next.service`、`deploy/c50c-web-bridge.service`（**仓库里是占位符模板**） |
| 协议契约 | `deploy/BRIDGE_PROTOCOL.md` |
| 蓝图 | `~/.gemini/antigravity/brain/63492d8e-6ed3-479b-a101-bbab07f0928d/c50c_web_rewrite_blueprint.md`（§4 协议基线） |
| 旧实现参考 | `../Web/opi_robot/web_node/web_debug.py`（只读！安全隔离铁律） |
| aarch64 产物 | `target/aarch64-unknown-linux-musl/release/web_next`（md5 `4a681cc5…`，本次实测，与板端 `/home/orangepi/web_next_deploy/web_next` 一致） |
| 端到端模拟器 | `tools/shim_e2e.py`（本地起 web_next ros 模式喂帧+WS登录+手动命令全回路断言） |

### 板端（`ssh opi` = orangepi@192.168.31.148，免密；异地走 `ssh opi-remote`）

| 项 | 路径 |
|---|---|
| 部署目录 | `/home/orangepi/web_next_deploy/`（web_next、ros_bridge_shim.py、两单元、BRIDGE_PROTOCOL.md、c50c-web.service.bak） |
| 新单元（已装） | `~/.config/systemd/user/c50c-web-next.service`、`c50c-web-bridge.service`（**板上的是占位符已替换的实例**） |
| 新栈日志 | `/tmp/wbnext.log`、`/tmp/wbbridge.log`（unit StandardOutput=append；**journalctl 里没有**） |
| 控制台页面 | http://192.168.31.148:8088/ （现跑的是 Web-Next） |
| 排查工具 | `py-spy`（用户级，`~/.local/bin/py-spy`，2026-11-22 装；`py-spy dump --pid <垫片pid> --nonblocking` 看每个线程 Python 栈，卡死定位用）；`gdb`（系统级，sudo 装） |

### 凭据

| 项 | 值 |
|---|---|
| Web 登录 | 用户名 `admin`，密码 `admin` |
| 密码哈希 | `OPI_WEB_PASSWORD_SHA256=8c6976e5b5410415bde908bd4dee15dfb167a9c873fc4bb8a81f6f2ab448a918`（= sha256('admin')，与旧栈一致） |
| 桥接令牌 | `OPI_BRIDGE_TOKEN=c50c-bridge-20261121-a83f`（两个单元必须同值；换令牌要同时改两处） |
| 板子 sudo | 密码 `xzcit-opi`（一般用不到，全部操作都是 systemd --user 用户级） |

---

## 4. 切换 / 回退操作手册

### 切到 Web-Next（接管 8088）

```bash
ssh opi '
systemctl --user stop c50c-web.service c50c-web-watchdog.service
systemctl --user disable c50c-web.service c50c-web-watchdog.service
systemctl --user enable --now c50c-web-next.service c50c-web-bridge.service
sleep 3
systemctl --user is-active c50c-web-next.service c50c-web-bridge.service
ss -ltn | grep 8088'
```

### 回退到旧 web_debug（当前状态就是这么来的）

**2026-11-22 起 c50c-web.service 与 watchdog.timer 已被 mask**（单元文件改名保留为
`c50c-web.service.masked-20261122`、`c50c-web-watchdog.timer.masked-20261122`，另建
`→ /dev/null` 符号链接占位）：不 mask 的话 `opi-robot.service`（start_web.sh）每次
重启都会 `restart c50c-web.service` 把旧 web_debug 拉起来抢 8088，Web-Next 重启窗口
内可能被它插足，且 web_debug 绑定失败会陷入 Restart=always 每 3s 空转。页面
「传感器设置」的重启相机/雷达按钮就是走 `systemctl --user restart opi-robot.service`，
必须保持 mask。回退时先 unmask：

```bash
ssh opi '
systemctl --user stop c50c-web-next.service c50c-web-bridge.service
systemctl --user disable c50c-web-next.service c50c-web-bridge.service
cd ~/.config/systemd/user
rm c50c-web.service c50c-web-watchdog.timer        # 移除 /dev/null 占位链接
mv c50c-web.service.masked-20261122 c50c-web.service
mv c50c-web-watchdog.timer.masked-20261122 c50c-web-watchdog.timer
systemctl --user daemon-reload
systemctl --user reset-failed c50c-web.service c50c-web-watchdog.service
systemctl --user enable --now c50c-web.service c50c-web-watchdog.service'
```

浏览器里旧标签页要**强刷**才会加载另一套前端。`reset-failed` 不是装饰：web_debug/垫片被 SIGTERM 杀时优雅退出超时会留下 `failed` 状态，不清理会吓到下一个人。

### 切换后验收清单（每条都做过，照抄即可）

1. `systemctl --user is-active c50c-web-next.service c50c-web-bridge.service` → 双 active；
2. `curl -s --noproxy '*' http://192.168.31.148:8088/api/state` → `"mode":"ros"` 且 `chassis` 字段非 null（垫片没连上时全 null）；`network` 对象应有值且 `ssid` 非空（实车 iw 采集器 src/netinfo.rs 已在跑，2026-11-22 实测 `ssid:"103"`、`iface:"wlan0"`）；
3. `tail /tmp/wbbridge.log` → 应见 `垫片启动…已连接 Web-Next…发送 bridge_hello`，无 ImportError/Traceback；
4. 浏览器开概览页 → 判读条/电池/节点有真值（这台车此刻的真实告警见 §7⑦）；
5. 登录 admin/admin → 手动控制页出现 `相机 12.5 fps` 级别的帧率与雷达画中画；
6. （改过代码时）板端文件与本地 md5 配对相等再重启，见 §5。

---

## 5. 重新构建与部署（改了代码之后）

本机无 aarch64 交叉 gcc/zig/docker，**用 musl 静态目标自包含链接**（rustup target `aarch64-unknown-linux-musl` 已装好）：

```bash
cd /mnt/data/project-ai/Embedded_Development/Antigr/repo-push/XZCIT-IIR/Master/Web-Next
cargo build --release --target aarch64-unknown-linux-musl --manifest-path Cargo.toml
file target/aarch64-unknown-linux-musl/release/web_next   # 必须含 "ELF 64-bit … aarch64"
# 本地门禁（ros 模式 + 模拟垫片全回路）：
OPI_WEB_PORT=8091 OPI_WEB_PASSWORD_SHA256=8c69…a918 OPI_BRIDGE_TOKEN=e2etoken \
  setsid nohup target/debug/web_next >/tmp/wxn.log 2>&1 & disown
OPI_BRIDGE_TOKEN=e2etoken python3 tools/shim_e2e.py 127.0.0.1 8091   # 期望 SIM_RESULT=status_ok,command_ok
```

**本机网络两个坑**：shell profile 注入的代理 `127.0.0.1:7897` 是死的（活的是 7890），rustup/curl 都会撞——
curl 一律加 `--noproxy '*'`；rustup 下载用：

```bash
https_proxy= http_proxy= all_proxy= HTTPS_PROXY= HTTP_PROXY= ALL_PROXY= \
RUSTUP_DIST_SERVER=https://rsproxy.cn RUSTUP_UPDATE_ROOT=https://rsproxy.cn/rustup \
rustup target add aarch64-unknown-linux-musl
```

传板与切换（**md5 必须同一文件两端配对比，别跨文件比**；开发机一切命令超时 ≤45s；
**二进制正在运行时 scp 直传同名文件会 "dest open Failure"（ETXTBSY），先传 `.new` 再 mv 原子替换**）：

```bash
scp -o BatchMode=yes target/aarch64-unknown-linux-musl/release/web_next opi:/home/orangepi/web_next_deploy/web_next.new
ssh opi 'mv /home/orangepi/web_next_deploy/web_next.new /home/orangepi/web_next_deploy/web_next'
scp -o BatchMode=yes deploy/ros_bridge_shim.py opi:/home/orangepi/web_next_deploy/ros_bridge_shim.py
md5sum <本地各文件>   # 与 ssh opi 'md5sum /home/orangepi/web_next_deploy/<同名>' 逐对相等
```

单元文件只在**首次安装**或改单元时才重装：仓库模板占位符 `__BIN__` / `__SHIM__` / `__PW__` / `__TOKEN__` **四个都要 sed**
（2026-11-21 部署翻车的根因就是只换了后两个，`ExecStart=__BIN__` 导致 EXEC 失败无限重启）。板上已是实例，日常改代码只需重传二进制/垫片 + `systemctl --user restart`。

垫片改动只涉及 python：`python3 -m py_compile deploy/ros_bridge_shim.py` 过了就能直接 scp 重启，不用重编二进制。

---

## 6. 协议要点（详见 deploy/BRIDGE_PROTOCOL.md）

- 浏览器上行 `{t:"manual_start"|"manual_move"|"manual_stop"|"nav_goal"|"patrol_*"|"cruise_set"|"us_rearm"|"login"…}`；下行 `{t:"status"|"scan"|"map"|"pose"|"img"|"probe"|"cmd"|"nav_result"…}`。
- 手动驾驶：浏览器 150ms 心跳，hub 仲裁后经 /bridge 命令队列下发垫片，垫片以 `manual_cmd_vel`（Twist, status_qos）发出；许可被拒时 hub 弹 `manual_error`。
- `/drive_safety`（fusion_node 的 String JSON）→ hub 按 `ttl_s`（缺省 1s）判新鲜，过期 `allowed=None`；锁停/放行跳变由前端 toast 呈现。
- `/map` 是 RELIABLE+TRANSIENT_LOCAL（map_server 静态栅格），垫片重启后也能自动收到，别改回 volatile。
- 帧形状的唯一权威是 fake.rs 的产物（前端按它写的）；ros.rs 注入字段与之逐一对齐，**改任何一边都要过 `tools/shim_e2e.py`**。
- **喇叭测试音 + 音量条（2026-11-22）**：喇叭卡片从「纯状态」升级为可操作——`{t:"speaker_test"}` 放三声 880 Hz/120 ms 提示音（同一次 `aplay` 播完，不是连起三个进程），`{t:"speaker_volume", volume:0..100}` 调 es8388 的 `PCM` 混音控件；两者都回 `{t:"sensor_status", ok, message, volume?}` 复用现有 toast 分支。实现全部在 `src/speaker.rs`：**raw ALSA**（板上没有 PulseAudio/PipeWire），`aplay -D plughw:<es8388卡>,0` + `amixer -c <卡> sset PCM N%`，卡号由 `audio::es8388_card()` 从 `/proc/asound/cards` 解析（找不到回退 2）。wav 头 + 正弦现场生成到 `/tmp`，用后即删；外部命令一律 8s 超时兜底。**音量显示**：`status.audio.volume` 首次从 amixer 读真值并缓存，之后由 `set_volume` 回读维护（5s 一次的状态探测不反复 fork）；读不到就 `null`，前端显示「—」并把滑块/按钮置灰（没检测到喇叭同样置灰）。滑块 `input` 只更新读数、`change`（松手）才下发命令，避免拖动过程刷屏。同 `sensor_restart` 一样不经垫片/ROS，BRIDGE_PROTOCOL 不涉及。
- **传感器重启（2026-11-22）**：浏览器 `{t:"sensor_restart", sensor:"camera"|"lidar"|"chassis"}` → 服务端**白名单**映射 `systemctl --user` 单元（camera/lidar→`opi-robot.service`，二者同进程组，start_web.sh 会连带重启融合/SLAM/底盘桥 try-restart；chassis→`c50c-chassis.service`，ExecStartPre sleep 20 约 20-30s 恢复）→ `spawn_blocking` + 45s 超时 → 回 `{t:"sensor_status", ok, message}` 弹 toast。**不在白名单的设备（喇叭/矩阵麦克风）如实拒绝**——它们没有守护进程可重启。这条不经垫片/ROS，是 web_next 直接跑 systemctl，BRIDGE_PROTOCOL 不涉及。
- **`status.audio`（2026-11-22）**：`{speaker, mic_board, mic_usb}` 三个布尔，audio.rs 读 `/proc/asound/cards`（按 es8388 找卡号——注意每张卡下面有非数字续行，必须跳过不能中断解析）+ `/dev/snd/pcmC*D0p|c`（尾字符才是类型，`pcmC2D0p` 去前缀后是 `2D0p`）。HDMI/DP 的 pcm*p **不算喇叭**。fake 模式 null。
- **`status.sys`（2026-11-22）**：`{cpu, gpu, mem, disk, temp}`——前四为百分比（整数，缺项 null），sysinfo.rs 每 5s 采样：CPU 取 /proc/stat 首行与上拍差分（**首拍无 cpu 字段**，前端显示 —）；GPU 读 `/sys/class/devfreq/fb000000.gpu/load`（格式 `<util>@<freq>Hz`）；内存 (MemTotal−MemAvailable)/MemTotal；磁盘 `df -P /` 的 Capacity 列。`temp` 为 SoC 温度（2026-11-22 增）：thermal_zone0 毫摄氏度 → 0.1℃ 精度（板上七区 soc/bigcore0/bigcore1/little/center/gpu/npu，只报主温）。概览「系统」卡渲染为 **2×2 SVG 圆形仪表**（`.sys-donuts`，r=30 周长 188.4956、dashoffset=周长×(1-占用)、rotate(-90°) 从 12 点顺时针、环内 14px 读数、≥85% 整环+数字变红），纯 CSS 变量配色、无第三方图表库。四环 = **CPU / GPU / 内存 / 温度**（2026-11-22 用户定版：磁盘环撤除、温度数字行撤除换成环——温度环弧按 0-100℃ 线性、环内 0.1℃ 精度「39.8°」、≥75℃ 环变黄（.hot）、≥85℃ 整环红，判读条同步报「系统高温/过热」；磁盘仍采集不下发界面）。
- **排版三级刻度（2026-11-22，用户定版）**：tokens.css `--fs-s:14px / --fs-m:24px / --fs-l:48px`，**全站 font-size 只准取这三档**（约 2x 步进，用户口径「黄金比例」）——s=正文/标签/chip/h3，m=键值读数（.kv b）/结论行/顶栏 h1/方向键，l=电池大数字。别引入 12/13px 这类杂值；顶栏 RTT 芯片定宽 74px 是按 14px mono 估的，改字号要复测。
- **侧栏收起/展开（2026-11-22）**：`#btn-side`（icon-btn 圆钮）切 `body.side-collapsed`（--sidebar-w 58px、藏文字、图标居中，与 700px 窄屏媒体查询同形态），localStorage `c50c-side-collapsed` 记忆，切路由不重置。**按钮位置在侧栏本体 brand 行右缘**（用户定版，2026-11-22 从顶栏挪入）——收起态藏 brand 圆点、按钮顶替为图标列首钮，58px 内放不下两个图标；700px 窄屏媒体查询同步藏圆点。
- **判读条展开/收起（2026-11-22）**：概览判读条有异常时可点击展开逐条原因（`.verdict.expandable` + chevron），再点收起；每条 issue 带 `detail`（数据全部来自 status，不编造），列表 HTML 按 issues 签名缓存（`_lastSig`），异常清零强制收起。`ui.verdictOpen` 内存态不落 localStorage。
- **卡值排版纪律（2026-11-22，用户定版）**：`.kv b` 一律 `white-space:nowrap; overflow:hidden; text-overflow:ellipsis`（单行省略兜底，实测行高 28-29px，WiFi 值「103 · -53 dBm」不再折行，`netWiFiText()` 已去频段段）；24px 值字重减细：`.kv b` 500 / `.kv b.big` 与 `.verdict-row b`、`.bat-row .big` 600——用户嫌 600/700「太锋利」。
- **电池卡定型（2026-11-22）**：大数字百分比 + 状态文案 + 电量条，**「电压」行、「充电」行均已删**（用户明确不要；充电状态并入状态文案）。状态文案由 `BatteryView::from_voltage` 下发：**充电中（>25.0V）/ 放电（非充电正常态）/ 电量低，请充电 / 低于停用线，立即停车充电 / 无数据**——`charging` 是纯电压阈值推断（无电流/插入检测信号），拔充电器后随电压缓降跌破 25.0V 才翻，有几分钟滞后。`ov-bat-charge` 元素已删，`bat.charging` 仅剩电量条配色（accent/ok/warn/bad）在用。
- **传感器设置两分区（2026-11-22）**：控制页传感器 tab 拆 `.sensor-zone`×2——「可重启」（超声波/相机/雷达/底盘，各带状态行+重启钮）与「状态监测」（喇叭：状态+测试音+音量条；矩阵麦克风：纯状态）。超声波卡只有「状态」（正常/锁存/故障），**距离从此不在 UI 任何位置显示**（用户明确要求，`mp-us-v`/`mp-us-gate` 元素与渲染行均已删，后端字段保留）。
- **地图并入控制页（2026-11-22，用户定版）**：`#/map` 独立路由与侧栏「地图」入口删除，地图成为控制页**第三 tab**（手动控制 / 传感器设置 / 地图），整体需登录后可见（原「地图免登录可看」语义取消）。落地：index.html 删 `page-map` section、`.map-grid` 迁入 `#ctl-pane-map`（**登录卡删除**，地图操作统一走控制页登录）；app.js `PAGE_TITLES` 删 map 键、`setCtlTab`/`bindCtlTabs` 改三态（ArrowLeft/Right 循环模 3）、`setCtlTab('map')` 补 `Store.mark(F.MAPTEX)`（tab 切换不走 applyRoute，纹理重建靠它）、`showLoginErr`/`setAuthed` 去 map 分支、`bindMap` 去内联登录绑定；canvas-map.js 渲染门控 `ui.page === 'map'` → `'control'`；ws.js `setPage` 视频通道只随 control（原 control||map 并集）；layout.css `#ctl-pane-map{display:block; grid-template-*:none}` 不吃 pad/cam 两栏网格。`#/map` 旧链接由 `pageFromHash` 兜底回退 overview 无死链。后端零改动（残留 `page=="map"` 硬编码无害，前端不再发 view:map）。**补（同日）：`btn-motion-stop`（全局停止）/`btn-motion-rearm`（恢复）从航点巡航卡挪到手动控制 tab pad 卡档位行下方**（用户定版：这俩是全局操作，应在手动控制页；id 不变、`bindMap` 绑定照常生效，地图航点巡航卡只剩保存/开始）。**补（同日）：`motion_rearm`/`patrol_start` 成功回执 message 由 `"ok"` 改中文「已恢复」/「巡航已开始」**——服务端透传的英文 message 会直接上 toast/patrol-msg，别漏。
- **主字体 HarmonyOS Sans SC（2026-11-22，用户四字体对比后定版）**：全量 woff2（Regular/Medium/Bold，4.2MB×3≈13MB）`include_bytes!` 编进二进制，`/fonts/*.woff2` 路由 `public, max-age=31536000, immutable` 下发，index.html 三个 `<link rel="preload" as="font" crossorigin>` 预热（**font 必须标 crossorigin，否则二次下载**）。字体源：npmmirror 包 `@fontpkg/harmony-os-sans-sc`（官方 TTF 全字重，fontTools 转 woff2，flavor 切换即可）。**字重区间映射**（tokens.css @font-face）：CSS 400→Regular、500-650→Medium（600/650 渲染为 Medium，比系统 Semibold 柔）、700-900→Bold——改字重直接改 CSS 数值即可落对应文件。**HarmonyOS 自带 tnum**，body 已开 `font-variant-numeric: tabular-nums`，数字等宽成立，`--mono` 已换成同族字体兜底系统等宽。RTT 74px 定宽复测：14px 下 "1234 ms" 实测 54px，余量充足。二进制因此从 ~4.1MB 涨到 16.4MB。**部署坑：运行中的二进制 scp 直传会 "dest open Failure"（ETXTBSY），先传 `web_next.new` 再 mv 替换**（§5 补充）。

---

## 7. 已知坑与纪律（血泪清单，逐条实测）

1. **安全隔离铁律**：绝不删除/破坏 `Master/Web/opi_robot`、`Master/ROS/ros2_ws/src/opi_robot`、板端 site-packages 的 web_debug——回退保命靠它们完好。
2. **垫片日志在 /tmp/wb*.log**，journal 里没有；查 user 单元日志必须 `journalctl --user-unit=c50c-web-bridge.service`（板上 `/var/log/journal` 为空，`journalctl --user -u` 只回 "No journal files were found"）。
3. **OccupancyGrid 在 nav_msgs.msg**，不是 sensor_msgs（曾让垫片 import 崩 26 连）；垫片对可选消息一律 try/except 兜底（学旧实现）。
4. **单元占位符四个全要换**（§5）；两个单元的 OPI_BRIDGE_TOKEN 必须同值。
5. **`/scan` 源实测 ~99.7Hz**：rclpy 反序列化常态吃约 1 核（旧 web_debug 同栈同率，一直是隐形大户）。垫片已节流 20Hz 喂 UI + 单元 `CPUAffinity=0 1 2 3` 钉小核——本机 RK3588 **cpu0-3=1.8G 小核、cpu4-7=2.3G 大核**（与常见 RK3588 相反，实测 /sys/…/cpuinfo_max_freq），大核必须留给 nav2。
6. **本车栈没有 chassis_status_node**：/diagnostics 实测发布者 = c50c_uart3/ekf/fusion/lidar/两个 lifecycle_manager/垫片自己。垫片已把"底盘节点活性"挂到 uart3 参数服务存活（与"底盘数据超时"同源），别改回按节点名探测，否则概览常挂假离线。
7. **这台车此刻的真实状态**（2026-11-22 实测，不是 bug）：超声波正视读数 2.068m（fault=false、stop=false）、`drive_safety.allowed=true`、车辆在充电（底盘 25.79V / 电池 25.79V、charging=true）——但运动许可仍锁停：`patrol.gate=lock_stop`、`lock_reason=license_lost`、`stop_cause=license_expired`、`released=false`；实测 interval=3s 两次快照底盘 state=fresh、底盘 hz 20.4→19.9、里程 0m 未变化。概览判读条的报警是真的；`nodes.web_bridge` 是垫片自己，恒在线。网络卡 WiFi 行现在显示实车采集值（ssid=103、5 GHz、rssi/signal_dbm=-54），不再是"无数据"。
8. **bash 起后台进程必死**（ssh 会话退出连带被杀，实测两次），长期进程一律 systemd --user；`pkill` 与启动放同一个 Bash cell 会连带杀掉新进程。
9. **开发机命令超时 ≤45s**（用户铁律）：cargo 慢就切片增量跑；agent 编排里的 world.run timeoutMs ≤44000。
10. 改前端 CSS 别踩两坑：UA 原生 outset 边框必须全局清零；custom property 里的 var() 在 :root 固化，`--ring-c` 覆盖传不进嵌套 token（elev token 只存无 var() 余量，环线在使用处直写）。
11. 路由切换故意不包 startViewTransition（VT 快照定格空屏=进场"卡一下"主因），VT 只留给主题切换——别按蓝图把路由 VT 加回来。
12. **meta[color-scheme] 有三个值**：显式 "light"/"dark" + 跟随系统的 "light dark"（与系统一致时 JS 故意回退到它、不落存储）。凡按它分支的 CSS（如主题钮日/月显隐，tokens.css 末尾）**必须覆盖第三态**——用 `@media (prefers-color-scheme: …)` × `:has(meta[content="light dark"])` 补齐，否则跟随系统态会日月同显（2026-11-22 实际翻过车）。
13. **透传子对象必须逐字段对齐 fake.rs 权威形状**（2026-11-22 翻车：超声波误报"故障"数日）。ros 模式下发的是 fusion `/drive_safety` 的字段名 `{stop, deny, valid, range_m, fault, fault_reason, age, stop_distance, release_distance, invalid_at}`（fusion_node.py:992，`deny = stop || !valid`；`fault`=无效/断流需重启，`stop`=近距迟滞锁存远离自解），而前端按权威形状读 `{m, ok, latched}` 三态——字段名对不上则 `usd.ok` 恒 undefined、`!usd.ok` 恒真，传感器再健康也误报。**修复在 `drive_safety_view_of` 归一**（`ultrasonic_view_of`：`latched = stop && !fault`、`ok = valid && !stop && !fault`、m=range_m 四舍五入；ds 帧过期时 ultrasonic 置 None——断流由底盘超时/节点离线报警覆盖，不拿陈旧证据报假故障）。语义以 fusion 源码为准只读不改；前端禁止直接读 valid/fault/stop。
14. **许可会话会被隐藏页面永久占据**（2026-11-22 实锤：内嵌浏览器里残留 3~5 个看不见的控制台 webview，其一自 16:03 服务重启起持有许可，可见页被 `session_mismatch` 永久拒绝、判读条常挂「许可异常 会话被占用」）。机制：每页 Blob Worker 独立连接 100ms 心跳、页内随机会话号，持有权只在持有者停跳 350ms 后易主——隐藏页不心跳不算"死"，前台页就永远抢不到；且隐藏页持证会让「网页许可中断」锁停永不触发（巡航失去网页监护）。**修复（2026-11-22 已上板）：许可权随前台页**——`visibilitychange` 门控 Worker（ws.js `gate` 消息）：隐藏=发 `motion_license_release` 交还并断开许可连接，回前台自动重连、≤100ms 心跳即接管；服务端 `Hub::license_release`（safety.rs）只清 `session/last_rx_ms` 并把 `last_seq` 归 **-1**（前端 seq 页面生命周期内不清零，不重置会被 seq_replay 永久拒绝），**不发断流锁停**（license_tick 的 last_rx=None 分支；fusion 侧靠自身 350ms 超时走 report_lost，与页面关闭同路径，web_next 不转发释放）。**release 帧若在 WS 断连中丢失则退化为 350ms 断流锁停（崩溃语义，安全兜底）。** 验收法：本地 `cargo run -- --fake --port 8090` + MCP 双窗口（MCP 每页是独立窗口都算 visible，测隐藏要 `Object.defineProperty(document,'hidden',{get:()=>true})` + `dispatchEvent(new Event('visibilitychange'))` 手动门控）：A 持证→A 门控隐藏→B 1.2s 内 Adopt 正常；A 恢复可见→B 保持、A 被拒（先持有者稳定，无乒乓）；B 关闭→A ≤1.5s 回收。**旧代码残留页不认识 release，清一次僵尸需重启 ZCode 内嵌浏览器面板/重启 ZCode**。
15. **锁停态对 `manual_start` 是"假通过、真不动"的静默漏洞**（2026-11-22 用户实测：按方向键 toast「手动控制已接管」但车不动、无任何报错）。机制：锁停有两种——急停 `motion_stop`（`locked=true,user_locked=true`）和**自动锁停** `license_lost/session_dead/web_disconnected/goal_timeout`（`locked=true,user_locked=false`）；旧 `manual_start` 手控门槛只查 `user_locked`，自动锁停直接放行，速度照发但下游全局锁停不执行，静默失败。**修复（2026-11-22 已上板，md5 d0c50eaec23d）**：(A) `manual_start`（safety.rs）在 user_locked 后加 `if ctl.locked { return Err("已锁停，请先点「恢复」") }`——所有锁停拦手动，中文 manual_error 经现有 toast 通路可见；(B) `manual_heartbeat` 返回 `Option<String>`，owner 被其他页面顶替时补一条「手动控制权已被其他页面接管」，`steal_warned` 边沿去重（owner=None 空转**不报**，防单页误报）；(C) 恢复路径不动（`motion_rearm` 收尾已解锁，自动锁停恢复后不阻塞）。**验收法（板端实车 2026-11-22 过）**：真实锁停态按方向键→唯一 toast「已锁停，请先点「恢复」」无误报；点恢复→「已恢复」+`locked=false/gate=ok`；解锁窗口按 W 保持→「手动控制已接管」+`task.manual.active=true,input=0.4` 车可控；合成 keydown 必须配 keyup 否则 `pressed` 残留、再按无反应（前端正常去重，非 bug）。**顺带实测到垫片断连周期事故**（17:35:17 重连）：每次断连 1s 内 ROS 侧许可失效→停 `license_lost` 再锁停，恢复回调有效，这正是本修复要让用户看见的报错场景。
16. **垫片 command_thread 消费端卡死=「点前进车不动」+「恢复后 1s 又锁停」同一根因**（2026-11-22 实锤）。现象：点方向按钮/按方向键 toast「手动控制已接管」、`task.manual.active=true`、许可 `license_state=ok`，但垫片观测 `cmd.manual=state=unknown`（manual_cmd_vel 无帧）、车不动；且恢复解锁后 ~1s 又 `license_lost` 锁停。根因链：**垫片 `ros_bridge_shim.py` 的 command_thread（GET /bridge/commands 长轮询消费者）卡死**——一次重建连接后永久挂在 `conn.getresponse()`，既不消费也不超时报错 → web_next 出口命令队列**持续积压**：manual_cmd 堆积→垫片 never 发 manual_cmd_vel→车不动；license_forward 堆积→融合侧 350ms 收不到许可→license_expired 锁停。**为什么页面完全看不出来**：上行线程（POST /bridge/frames）独立且活着，状态卡全 live；下行线程是独立的，卡死不打扰上行。**确诊手段**：`curl -H 'X-Bridge-Token: $TOKEN' 'http://127.0.0.1:8088/bridge/commands?wait_ms=3000&cursor=0'` 看到 license_forward 积压 200+ 条且二次取样 cursor 增长（新积压，非死等）；`ss -tnp | grep <垫片pid>` 只有 2 个到 8088 连接（上行+下行各一），下行 `Recv-Q` 永久挂起；垫片日志停在最后一次「命令轮询断开…1.0s 后重连」后再无轮询日志（正常空轮询也不打日志，靠积压量判定）。**修复（2026-11-22）**：`systemctl --user restart c50c-web-bridge.service`——重启后 T1 快照 72 条 license_forward 数秒内被取走、按住前进 `cmd.manual=state=fresh, vx_m_s=0.4`（manual_cmd_vel 真发布）。**防复发（建议）**：给 command_thread 加轮询周期级看门狗（如 25s 无成功 `resp.read()` 即 `conn.close()` 强制重建），或把队列积压量暴露成状态字段做告警；也可并入 dds-watchdog 探测。**遗留**：垫片层已通，但底盘实测 `chassis.vx=0/里程 0`（23.97V、charging=false、drive_enable=true、drive_safety.allowed=true、许可 ok）——manual_cmd_vel → cmd_vel 的最终驱动段（fusion/uart3）未在本轮确认，需人车目视条件实车验收。
17. **「重定位」曾是纯桩→已接真 AMCL 链路**（2026-11-22 用户追问「重定位怎么没实现」，属实：旧 `handle_relocalize` 只 sleep(1.2s) 假回执「已完成全局重定位校准」）。**真实现（2026-11-22 已上板，md5 d6bd1d45584b）**＝「当前位姿重播种」：前端 `btn-reloc`（app.js）把画面位姿整包 (x,y,yaw) 上传 → `handle_relocalize`（ws_control.rs）解析+校验 → `ros.rs::request_relocalize` 入队边沿命令 `{c:"relocalize",x,y,yaw}` → 垫片 `_publish_initialpose` 发布 `/initialpose`（PoseWithCovarianceStamped，格式逐字对齐旧栈 web_debug.py::relocalize_pose 752-765：**frame_id='map'+header.stamp 零时间戳**——AMCL 对零时间戳按"最新"处理，写本机时钟触发 Lookup extrapolation 历史坑 nav2_params_20261120.yaml:35-43；yaw→四元数 z=sin(θ/2) w=cos(θ/2)；cov[0]=cov[7]=0.09, cov[35]=0.0685）；回执带坐标「重定位校准已下发：(x.xx, x.xx) m，朝向 x°」。**语义边界**：修的是"在雷达确认的位置重新播种粒子"；AMCL 已严重漂移需从零找回位姿→需搬 relocalize.py numpy 全局扫描匹配（另一档改动未做）。**验收（板端实车 2026-11-22 过）**：`ros2 topic echo /initialpose --once` 抓到完整帧 stamp=0/x=-0.02/y=-0.006/四元数(0.0262,0.9997)=3.0°/cov 0.09，与画面 `th=0.0526`、toast「朝向 3.0°」三方一致；历史坑 malformed **未复现**（零时间戳格式采纳成功）。**教训**：新栈前端一直有的「重定位」按钮+ws.js toast 通路是蓝图预留，后端是假实现——凡是 UI 存在但行为可疑的按钮，先读 handler 是不是桩，别被 "按钮在" 骗过。
18. **`relocal_busy` 泄漏→重定位二次起全被「已在进行中」拦**（2026-11-22 实测：用户问「重定位了图标怎么没动」）。根因：第一次实现 `handle_relocalize`（ws_control.rs）ros 模式**成功分支 `Ok(())` 漏调 `hub.relocalize_end()`**（只有 `Err` 分支释放）→ 第一次点后 `relocal_busy` 永久卡 `true` → 之后每次点重定位都被 `relocalize_begin()` 防重入拦下回「重定位已在进行中」，`/initialpose` 压根不再发，AMCL 收不到新播种→图标永远不动，且 toast 骗人（像成功但其实是拒绝）。**修复（2026-11-22 上板，md5 6c81c79a991c）**：`Ok(())` 成功分支补 `hub.relocalize_end()`（fake 分支的 spawn 任务早有 end，ros 分支漏了）。**验收**：连点两次重定位→两次都回成功「重定位校准已下发」、无「已在进行中」。**教训**：busy/防重入锁的成功分支最容易漏释放，回了成功 toast 就是大事——凡成功路径也要在「出作用域前」统一归零状态；「像成功其实是拒绝」的 toast 最危险（用户会照着成功去等效果）。
19. **垫片 rclpy context 失效=「地图等待中」+许可锁停同源**（2026-11-22 实锤，与坑 16 是**两个不同失败模式**）。坑 16 是 command_thread 的 socket 长轮询卡死（积压未派发）；这次是**rclpy 上下文坏死**：垫片还在跑、上行帧还在发（状态卡全 live、pose/scan 新鲜），但 `publish()` 全部抛 `RCLError("Failed to publish: publisher's context is invalid, at ./src/rcl/publisher.c:389")`——license_forward 派发即异常→融合侧许可断流锁停；同时 `/map` 订阅端也死→前端 map 恒 null 画不出图→「等待地图...」。**两大症状（地图等待中 + 许可锁停）出自同一个垫片进程的 rclpy 失效**，别各查各的。**确诊**：`tail /tmp/wbbridge.log` 看到 `派发异常：RCLError(...context is invalid)` 连续刷（旧进程 18:37:37-39 刷到重启前最后一刻）；py-spy dump 显示 command_thread 在 `getresponse/readinto` 长轮询正常态（排除坑 16——两坑症状一样但栈不同）。**修复（2026-11-22）**：`systemctl --user restart c50c-web-bridge.service`，重启后（a）地图立即可用——TRANSIENT_LOCAL 缓存 /map 重订阅即回（frontend `map:{seq:1,w:394,h:278,res:0.05}`）；（b）队列积压 license_forward 两分钟内被新进程消化。**验收法**：两次 `curl .../bridge/commands?wait_ms=1000&cursor=N` 间隔 4s，cursor 前进即队列在被消费；垫片日志重启后零派发异常。**看门狗（2026-11-22 已上板，md5 d321651e7fd228013a6306bc6e58f551）双路防复发，与坑 16 一并落**：
- **坑 19 路（publish context 坏死→自杀重启）**：`dispatch_command` 异常里识别 `context is invalid/destroyed/shutdown` 类 RCLError，计入 30s 滑窗（`record_publish_failure`，deque maxlen=64）；滑窗内 ≥20 次（10Hz 心跳下约 2s 即满）→ `os._exit(1)`，systemd `Restart=always` 3s 拉起新进程全新 rclpy context。`os._exit` 而非 `sys.exit`：dispatch 跑在 command_thread 里，sys.exit 只杀线程进程继续跑（许可永远断流但 pad 活着，比崩溃更糟）。其他偶发 publish 异常只记日志不计数，杜绝一条失败就误杀。
- **坑 16 路（command_thread 轮询卡死→强制断连重连）**：command_thread 每次长轮询 `cmd_poll_begin(conn)` 记录连接+发起时刻，`finally` 里 `cmd_poll_done()` 清除；main 线程每 1s `cmd_watchdog()` 巡检——超过 `CMD_POLL_STALL_S=25s`（长轮询最长 20s + 5s 余量）未完成即 `conn.close()`，让 command_thread 在 getresponse/read 抛异常走退避重连（全新连接/全新 HTTP 状态），**不自杀**（上行线程健康，坑 16 和 19 的差异点）。
- **验收**：`record_publish_failure` 滑窗三分支本地回归过（非 context 不计数/第 20 次触发/31s 滑窗老化不误杀）；`cmd_watchdog` 板端单测四分支过（无轮巡不动/未超时不关/超时强制断连/完成后清空不误杀）。部署后跑 3 分钟零「看门狗」触发现（25s 阈值对 20s 轮询余量充足），许可随前台页正常停跳/恢复。

---

## 8. 未覆盖 / 后续建议

- **实车运动未验收**（2026-11-22）：`attempted=false`——运动许可仍锁停（patrol.gate=lock_stop，lock_reason=license_lost，stop_cause=license_expired）、无人站在车旁目视、未确认车已离桩周围无障碍，三条硬条件均不满足，故不冒进发运动命令；两次间隔 3s 快照里程恒 0m，无轨迹。需下次具备人车目视条件后实测。
- **断电重启自启未实测**：单元已 enable + linger 开启，理论上自启，值得拔电验证一次。
- 网络卡 WiFi 行的实车采集器已实现（src/netinfo.rs：SlotOnceLock<Mutex<…>> 槽、5 秒周期 spawn_blocking 采 iw dev link / ip -4 -o，network 对象含 iface/state/ssid/ip/mac/rssi/signal_dbm/freq_mhz/band/tx_mbps/rx_mbps/quality/age_s，rssi 与 signal_dbm 同值双写；fake 模式跳过不采）。
- `password_configured` 可暴露到 /api/state，让登录报错区分"密码错"与"未配置密码"（后台字段已存在，只差暴露）。
- 部署编排的完整工作流脚本在 `~/.zcode/…/workflow-drafts/deploy-web-next.ts`（勘察→评审→实现→门禁→交叉编译→切换→冒烟→复核全程，含回滚洞），重跑前先读 §7 的坑位清单。

## 9. 一页纸速查

```bash
# 谁在跑 8088？
ssh opi 'systemctl --user is-active c50c-web.service c50c-web-next.service; ss -ltn | grep 8088'
# 新栈健康？
ssh opi 'systemctl --user is-active c50c-web-next.service c50c-web-bridge.service; tail -5 /tmp/wbnext.log /tmp/wbbridge.log'
# 真数据到了吗？
curl -s --noproxy '*' http://192.168.31.148:8088/api/state | head -c 400
# 切换/回退 → §4；重构建部署 → §5；怪事先查 §7
```
