"""启动文件：摄像头 + 激光雷达 + 融合 + 网页调试。

第一阶段约束：enable_cmd_vel 默认 false，不向底盘发速度指令。

设备路径默认值来自香橙派现场实测（2026-11-06 只读核对）：

- 雷达：``/dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0``
  by-path 含 USB 控制器与物理端口号，换 USB 口就会变。
- 摄像头：``/dev/v4l/by-id/usb-HD_720P_Webcam_HD_720P_Webcam_20130901-video-index0``
  by-id 含设备序列号，插拔或重新枚举后编号变化也不受影响。

不要再写 /dev/video0 或 /dev/ttyUSB0：枚举顺序变化会指向别的设备。
两者都可用 launch 参数覆盖，详见 docs/device-paths.md。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成四个节点的启动描述。"""
    video_device = LaunchConfiguration('video_device')
    lidar_port = LaunchConfiguration('lidar_port')
    enable_cmd = LaunchConfiguration('enable_cmd_vel')
    angle_origin = LaunchConfiguration('angle_origin')
    patrol_map_yaml = LaunchConfiguration('patrol_map_yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value=('/dev/v4l/by-id/'
                           'usb-HD_720P_Webcam_HD_720P_Webcam_20130901'
                           '-video-index0'),
            description='摄像头设备节点，用 /dev/v4l/by-id 稳定路径'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value=('/dev/serial/by-path/'
                           'platform-fc8c0000.usb-usb-0:1:1.0-port0'),
            description='激光雷达串口，必须是现场实测的 by-path 稳定路径'),
        DeclareLaunchArgument('enable_cmd_vel', default_value='false',
                              description='是否发布 cmd_vel，第一阶段保持 false'),
        DeclareLaunchArgument('angle_origin', default_value='mirror',
                              description='角度约定：mirror 或 raw'),
        # 巡航路线绑定地图内容（yaml 内 image 指向的 pgm 一并哈希）。
        # 不给就保持空串：网页侧按"地图未知"拒绝保存/开始巡航，
        # 不会拿一条来历不明的路线去跑。
        DeclareLaunchArgument(
            'patrol_map_yaml', default_value='',
            description='地图 yaml 路径（SLAM/nav2 保存的地图），路线按它绑定'),

        Node(
            package='opi_robot',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'device': video_device,
                'width': 1280,
                'height': 720,
                'pixel_format': 'MJPG',
                'frame_id': 'camera_link',
            }],
        ),

        Node(
            package='opi_robot',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[{
                'lidar_port': lidar_port,
                'baudrate': 230400,
                'frame_id': 'laser',
                'bins': 360,
                'range_min': 0.02,
                'range_max': 12.0,
                'stale_point_age': 0.5,
                'origin': angle_origin,
                'angle_offset_deg': 0.0,
                'frame_timeout': 0.5,
                'recovery_coverage': 0.9,
                'reconnect_interval': 2.0,
            }],
        ),

        Node(
            package='opi_robot',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[{
                # 停车距离取 0.4 m：2026-11-14 用户定（原 0.25 m）。
                'stop_distance': 0.4,
                'slow_distance': 0.90,
                'front_half_angle_deg': 30.0,
                # 与底盘桥 PLACEHOLDER_MAX_LINEAR_MPS 保持一致，超限会被判 NAV_INVALID
                'max_linear': 0.4,
                'enable_cmd_vel': enable_cmd,
                'cruise_timeout': 0.0,   # ≤0 = 无限运行直到手动停止
                # 巡航档 0.4 m/s；融合节点取它与 max_linear 的较小值。
                'cruise_linear': 0.4,
                'scan_timeout': 1.0,
                'lidar_diag_timeout': 2.0,
                'auto_turn': False,
            }],
        ),

        Node(
            package='opi_robot',
            executable='web_debug',
            name='web_debug',
            output='screen',
            # 控制台显示的安全开关。launch 里如实传 false/false：
            # 这里只负责"显示真实值"，不负责打开驱动。
            parameters=[{
                'drive_enable': False,
                'drive_input_enable': False,
                'drive_firmware_protocol': 'baseline',
                'patrol_map_yaml': patrol_map_yaml,
            }],
        ),
    ])
