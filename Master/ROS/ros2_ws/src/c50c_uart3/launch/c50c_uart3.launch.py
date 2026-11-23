"""启动文件：C50C UART3 桥接节点，显式选择固件方言。

为什么必须显式写 ``drive_firmware_protocol``
-------------------------------------------
``uart3_node`` 的参数默认值是 ``DEFAULT_FIRMWARE_PROTOCOL``（= ``'v3'``）。
``v3`` 是**未交付的会话式方言**：它要发会话帧、要求 ack 位与序号。板上跑
的是 2026-11-05 基线固件（无会话、无 ack、无序号）。若启动时不显式指定，
节点会按 v3 说话，固件一个字节也不认。

因此：
- 本文件的 ``drive_firmware_protocol`` 默认值写死 ``'baseline'``，
  **不依赖**节点参数默认值；
- 参数值仍是 ROS 参数，可用 ``drive_firmware_protocol:=v3`` 覆盖；
- 非法值由节点在启动时直接拒绝（抛 ``RuntimeError``），不会回退。

本文件不打开串口、不设置 DTR/RTS，只声明参数与超时。

安全默认
--------
``drive_enable``、``drive_input_enable``、``drive_fd_authorised`` 三项
全部默认 ``false``。三项目前都开，节点才向 UART3 写命令字节。这是本轮
不改的既存设计，本启动文件保持原样。

证据边界
--------
本文件只保证「启动配置把 baseline 写进参数」。它**不**证明板上固件是
baseline，也**不**证明任何命令被执行。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: 板上 2026-11-05 基线固件对应的方言名（与
#: ``c50c_uart3.chassis_cmd.arbiter.PROTOCOL_BASELINE`` 同值）。
#: 这里写字面量而不 import：launch 文件不随 Python 包路径分发，
#: 导入失败会让启动直接崩。一致性由测试
#: ``test_c50c_uart3_launch.py::test_protocol_default_matches_package_constant``
#: 守住。
BUNDLED_FIRMWARE_PROTOCOL = 'baseline'


def generate_launch_description():
    """生成 C50C UART3 节点的启动描述。"""
    device = LaunchConfiguration('device')
    baudrate = LaunchConfiguration('baudrate')
    protocol = LaunchConfiguration('drive_firmware_protocol')
    drive_enable = LaunchConfiguration('drive_enable')
    drive_input_enable = LaunchConfiguration('drive_input_enable')
    drive_fd_authorised = LaunchConfiguration('drive_fd_authorised')
    tick_period = LaunchConfiguration('drive_tick_period_s')
    odom_enable = LaunchConfiguration('odom_enable')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyACM1',
            description='UART3 串口设备（香橙派实测为 ttyACM1，'
                        'by-path platform-fc800000.usb-usb-0:1.2:1.0）'),
        DeclareLaunchArgument('baudrate', default_value='115200',
                              description='UART3 波特率，固件固定 115200 8N1'),
        DeclareLaunchArgument(
            'drive_firmware_protocol',
            default_value=BUNDLED_FIRMWARE_PROTOCOL,
            description='固件方言，板上是 2026-11-05 基线固件，故默认 '
                        'baseline；显式写死，不继承节点侧 v3 默认值'),
        DeclareLaunchArgument('drive_enable', default_value='false',
                              description='是否挂载直行命令桥（默认关）'),
        DeclareLaunchArgument('drive_input_enable', default_value='false',
                              description='是否订阅 cmd_vel/停止话题（默认关）'),
        DeclareLaunchArgument('drive_fd_authorised', default_value='false',
                              description='是否授权向真实 fd 写入（默认关）'),
        DeclareLaunchArgument('drive_tick_period_s', default_value='0.02',
                              description='命令节拍，0.02 s 即 50 Hz，与固件 '
                                          '100 Hz 主循环和 20 Hz 状态帧匹配'),
        DeclareLaunchArgument('odom_enable', default_value='false',
                              description='里程计候选位姿，默认关'),

        Node(
            package='c50c_uart3',
            executable='c50c_uart3_node',
            name='c50c_uart3_node',
            output='screen',
            parameters=[{
                'device': device,
                'baudrate': baudrate,
                'drive_firmware_protocol': protocol,
                'drive_enable': drive_enable,
                'drive_input_enable': drive_input_enable,
                'drive_fd_authorised': drive_fd_authorised,
                'drive_tick_period_s': tick_period,
                'odom_enable': odom_enable,
            }],
        ),
    ])
