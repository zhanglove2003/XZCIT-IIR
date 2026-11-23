# C50C 网页控制台

本目录保留网页前端、HTTP/WebSocket 服务、手动控制、航点巡航、导航目标处理和状态展示代码。`opi_robot/web_node/console_page.html` 是页面入口，`opi_robot/web_node/web_debug.py` 是服务入口。

在 `Master/Web` 中运行以下命令，可使用合成数据试用页面，不连接实车控制话题：

```sh
python3 -m opi_robot.web_node.web_debug --fake
```

默认端口为 `8088`，服务监听所有网络接口。可通过 `OPI_WEB_PORT` 修改端口。

控制账户为 `admin`。在启动服务的环境中设置 `OPI_WEB_PASSWORD_SHA256`，值为自行配置的密码的 SHA-256 小写十六进制摘要。未设置时无法登录控制账户。请勿将密码或摘要提交到仓库。

实车模式依赖已部署的 ROS 2 接口（完整包、Nav2 配置与 systemd 自启单元见 `../ROS/`），包括底盘控制、传感器、安全许可、地图和导航。

## 现场验证更新（2026-11-20）

- **电池电压重标定**（`console_views.py` / `console_page.html`）：6S 锂电以 `24.5 V` 为 100% 满电、`21.0 V` 为停用下限、`>25.0 V` 判定为充电中（`charging: true`，HUD 显示 `充电中（xx.xV）`）。
- **后台标签页防断连**：`console_page.html` 采用独立 `Blob` Web Worker（`motionLicWorker`）维持 100 ms `/motion_license` 心跳，修复切会 `session_id` 时序号归零导致的 `seq_replay`，并为 `applyPatrol` 复位增加 600 ms 冷却（保留 `pagehide` 退出停保）。
- **静态地图 5 Hz 位姿与自动收图**：`web_debug.py` 以 `RELIABLE + TRANSIENT_LOCAL` 订阅 `/map`（重启或开机顺序错开均自动获取静态地图），`/ws/video` 新增 5 Hz `{t:'pose'}` 实时位姿帧；多航点录入草稿保留并绘制编号与闭环连线。
