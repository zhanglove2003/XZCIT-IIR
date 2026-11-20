"""C50C 定位模式 launch（map_server + AMCL + lifecycle_manager_localization）。

由 systemd 用户单元 c50c-localization.service 拉起，仅加载已保存地图与
AMCL 激光定位，不启动 slam_toolbox，避免双发布 map->odom。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIFECYCLE_NODES = ['map_server', 'amcl']


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    tf_remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    loc_nodes = GroupAction(actions=[
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=tf_remap,
            parameters=[params_file],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=tf_remap,
            parameters=[params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': LIFECYCLE_NODES},
            ],
        ),
    ])

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.expanduser(
                '~/maps/c50c-field-20261120/localization_params.yaml'),
            description='Absolute path to localization_params.yaml',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        loc_nodes,
    ])
