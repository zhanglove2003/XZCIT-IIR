# 设备路径选择

本文说明为什么雷达与摄像头使用稳定路径，以及换 USB 口后如何重新确认。

## 规则

1. 雷达用 `/dev/serial/by-path/`。
2. 摄像头用 `/dev/v4l/by-id/`。
3. 不用 `/dev/ttyUSB0` 与 `/dev/video0`。
4. 每次换 USB 口或换设备后，重新确认路径并写入 `launch/robot.launch.py`。

## 两种稳定路径的区别

| 前缀 | 由什么决定 | 何时变化 |
|---|---|---|
| `/dev/serial/by-path/` | USB 控制器 + 物理端口号 | 换 USB 口就变 |
| `/dev/serial/by-id/` | 设备序列号 | 换设备才变 |
| `/dev/v4l/by-id/` | 设备序列号 | 换设备才变 |
| `/dev/v4l/by-path/` | USB 控制器 + 物理端口号 | 换 USB 口就变 |

by-path 说明"插在哪个口"。by-id 说明"是哪台设备"。
雷达当前用 by-path，因为香橙派现场实测就是这个路径。
摄像头当前用 by-id，因为重新枚举后编号会变。

## 现场实测值（2026-11-06 只读核对，Orange Pi 5 Pro）

```
雷达:   /dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0
摄像头: /dev/v4l/by-id/usb-HD_720P_Webcam_HD_720P_Webcam_20130901-video-index0
```

两个值都写在 `launch/robot.launch.py` 的默认参数里。

## 重新确认路径

列出当前所有候选：

```bash
ls -l /dev/serial/by-path/ /dev/serial/by-id/ /dev/v4l/by-id/ /dev/v4l/by-path/
```

看雷达插在哪个物理口：

```bash
udevadm info -q path -n /dev/ttyUSB0
udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_PATH|ID_SERIAL'
```

得到 `ID_PATH` 后，路径就是 `/dev/serial/by-path/<ID_PATH>-port0`。

看摄像头序列号：

```bash
udevadm info -q property -n /dev/video0 | grep -E 'ID_SERIAL|ID_PATH'
```

得到 `ID_SERIAL` 后，路径就是 `/dev/v4l/by-id/<ID_SERIAL>-video-index0`。

## 节点如何选雷达端口

`lidar_port` 非空时，节点只用这个路径，不扫描。

`lidar_port` 为空时，节点每次重连前重新扫一次 `/dev/serial/by-path/*usb*`：

- 0 个候选：保持空，状态停在 disconnected，原因 device_missing。
- 1 个候选：自动采用。
- 2 个或更多候选：拒绝盲选。打一条错误日志，列出全部候选，保持 disconnected。

所以插了 USB 转串口的其他设备时，必须用 `lidar_port` 明确指定雷达端口。

诊断消息的 `port_status` 字段说明当前状态：

- `explicit`：用了显式参数。
- `missing`：没有候选。
- `ambiguous`：候选太多，已拒绝。
- `scanning`：已自动选定一个候选。

## 覆盖默认值

命令行动态指定：

```bash
ros2 launch opi_robot robot.launch.py \
  lidar_port:=/dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0 \
  video_device:=/dev/v4l/by-id/usb-HD_720P_Webcam_HD_720P_Webcam_20130901-video-index0
```
