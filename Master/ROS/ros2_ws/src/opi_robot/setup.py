from setuptools import setup
import os
from glob import glob

package_name = 'opi_robot'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name,
              package_name + '.cam_node',
              package_name + '.lidar_node',
              package_name + '.fusion_node',
              package_name + '.web_node'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # 控制台页面由 web_debug.py 从自身目录读取，必须随包安装。
        (os.path.join('lib', package_name, 'web_node'),
            ['opi_robot/web_node/console_page.html']),
        # 诊断与 QoS 验证脚本随包安装，部署后可直接在目标机运行：
        #   python3 ~/ros2_ws/install/lib/opi_robot/real_ros_diag_check.py ...
        (os.path.join('lib', package_name),
            glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='snow',
    maintainer_email='snow@local',
    description='Orange Pi 5 Pro 摄像头与激光雷达调试包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = opi_robot.lidar_node.lidar_node:main',
            'camera_node = opi_robot.cam_node.camera_node:main',
            'fusion_node = opi_robot.fusion_node.fusion_node:main',
            'web_debug = opi_robot.web_node.web_debug:main',
        ],
    },
)
