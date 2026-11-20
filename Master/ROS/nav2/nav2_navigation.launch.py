"""C50C 导航模式 launch（候选，未部署）。

目的：让 velocity_smoother 的真实输出落在 /cmd_vel_smoothed，而不是被
Humble 自带 navigation_launch.py 重映射到 /cmd_vel。

做法：不直接 Include nav2_bringup/navigation_launch.py（它把
('cmd_vel_smoothed','cmd_vel') 写死在 Node/ComposableNode 上，launch 层
无法覆盖），而是按同一节点集合逐项声明，唯一差异是 velocity_smoother 的
重映射：
    cmd_vel          -> /cmd_vel_nav        （smoother 输入 = controller_server 输出）
    cmd_vel_smoothed -> /cmd_vel_smoothed   （smoother 输出，relay 订阅它）

同时拉起 nav_relay.py（/cmd_vel_smoothed -> /nav_cmd_vel）。

本 launch 只启动软件节点，不启动建图、不启动硬件驱动、不发速度。
导航模式下不允许 slam_toolbox 同时运行（map->odom 单发布者，见 README 第 4 节）。
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            GroupAction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SMOOTHER_INPUT = '/cmd_vel_nav'        # navigation_launch.py:122 remap cmd_vel->cmd_vel_nav
SMOOTHER_OUTPUT = '/cmd_vel_smoothed'  # navigation_launch.py:183 把它改成了 cmd_vel（本文件修正）
# relay 显式命令行（字符串形式，与 nav_relay.py parse_args 的默认值同值；start_relay 不读它）
RELAY_LIN_EFF = '0.78'
RELAY_ANG_GAIN = '2.41'
RELAY_MAX_OUT = '0.15'
RELAY_MAX_ANG = '0.65'
RELAY_TURN_PRIO = '0.38'
RELAY_TURN_REL = '0.14'
RELAY_STALE = '1.0'

LIFECYCLE_NODES = ['controller_server', 'planner_server', 'behavior_server',
                   'bt_navigator', 'waypoint_follower', 'velocity_smoother']


def _common(name):
    return dict(name=name, output='screen')


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    relay_script = LaunchConfiguration('relay_script')
    start_relay = LaunchConfiguration('start_relay')

    # 注意：ROS2 launch 的 Node 不接受 **kwargs，统一显式写出

    tf_remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    nav_nodes = GroupAction(actions=[
        Node(package='nav2_controller', executable='controller_server',
             respawn=False, respawn_delay=2.0,
             arguments=['--ros-args', '--log-level', log_level],
             remappings=tf_remap + [('cmd_vel', 'cmd_vel_nav')], name='controller_server', output='screen', parameters=[params_file]),
        Node(package='nav2_smoother', executable='smoother_server',
             remappings=tf_remap, name='smoother_server', output='screen', parameters=[params_file]),
        Node(package='nav2_planner', executable='planner_server',
             remappings=tf_remap, name='planner_server', output='screen', parameters=[params_file]),
        Node(package='nav2_behaviors', executable='behavior_server',
             respawn=False, respawn_delay=2.0,
             arguments=['--ros-args', '--log-level', log_level],
             # 与标准 navigation_launch.py:148-152 一致：behavior_server 只做 tf 重映射
             remappings=tf_remap, name='behavior_server', output='screen', parameters=[params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             respawn=False, respawn_delay=2.0,
             arguments=['--ros-args', '--log-level', log_level],
             remappings=tf_remap, name='bt_navigator', output='screen', parameters=[params_file]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             respawn=False, respawn_delay=2.0,
             arguments=['--ros-args', '--log-level', log_level],
             remappings=tf_remap, name='waypoint_follower', output='screen', parameters=[params_file]),
        # 与标准 navigation_launch.py 的唯一差异（其余节点 remap 与之逐项一致）。
        # smoother 显式启用：本 Node 无 condition，由 lifecycle_manager_navigation 统一 autostart；
        # 参数块 nav2_params.yaml:velocity_smoother 给出 max/min_velocity。
        # 订阅默认名 cmd_vel（libvelocity_smoother_core.so 字符串）→ /cmd_vel_nav；
        # 发布默认名 cmd_vel_smoothed（同一 .so 字符串）→ /cmd_vel_smoothed（不 remap 成 cmd_vel）。
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             respawn=False, respawn_delay=2.0,
             arguments=['--ros-args', '--log-level', log_level],
             remappings=tf_remap + [('cmd_vel', SMOOTHER_INPUT),
                                    ('cmd_vel_smoothed', SMOOTHER_OUTPUT)],
             name='velocity_smoother', output='screen', parameters=[params_file]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             arguments=['--ros-args', '--log-level', log_level],
             parameters=[{'use_sim_time': use_sim_time},
                         {'autostart': autostart},
                         {'node_names': LIFECYCLE_NODES}]),
    ])

    # relay 的所有换算常数/上限都显式传参（不依赖脚本默认值），relay 启动日志会逐项回显。
    relay = ExecuteProcess(
        cmd=['python3', relay_script,
             '--in', SMOOTHER_OUTPUT, '--out', '/nav_cmd_vel',
             '--lin-eff', RELAY_LIN_EFF, '--ang-gain', RELAY_ANG_GAIN,
             '--max-out', RELAY_MAX_OUT, '--max-ang', RELAY_MAX_ANG,
             '--turn-priority-wz', RELAY_TURN_PRIO,
             '--turn-release-wz', RELAY_TURN_REL,
             '--stale-timeout', RELAY_STALE],
        output='screen',
        condition=IfCondition(start_relay),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.expanduser('~/nav2_params.yaml'),
                              description='绝对路径；单写 params_file= 会被 ros2 当未知参数'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('relay_script',
                              default_value=os.path.expanduser('~/nav_relay.py')),
        DeclareLaunchArgument('start_relay', default_value='true'),
        nav_nodes,
        relay,
    ])
