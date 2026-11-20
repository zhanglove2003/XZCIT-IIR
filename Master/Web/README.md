# C50C 网页控制台

本目录保留网页前端、HTTP/WebSocket 服务、手动控制、航点巡航、导航目标处理和状态展示代码。`opi_robot/web_node/console_page.html` 是页面入口，`opi_robot/web_node/web_debug.py` 是服务入口。

在 `Master/Web` 中运行以下命令，可使用合成数据试用页面，不连接实车控制话题：

```sh
python3 -m opi_robot.web_node.web_debug --fake
```

默认端口为 `8088`，服务监听所有网络接口。可通过 `OPI_WEB_PORT` 修改端口。

控制账户为 `admin`。在启动服务的环境中设置 `OPI_WEB_PASSWORD_SHA256`，值为自行配置的密码的 SHA-256 小写十六进制摘要。未设置时无法登录控制账户。请勿将密码或摘要提交到仓库。

实车模式依赖已部署的 ROS 2 接口，包括底盘控制、传感器、安全许可、地图和导航。`Master/ROS` 暂无实现；本目录不包含底盘驱动、建图或导航栈，也不替换实车上已运行的程序。
