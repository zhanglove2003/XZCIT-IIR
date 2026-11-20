from setuptools import setup

package_name = 'c50c_uart3'

setup(
    name=package_name,
    version='0.1.0',
    # 子包必须显式列出：节点运行时会 import c50c_uart3.chassis_cmd，
    # 只写顶层包时标准构建产物中没有 chassis_cmd 目录。
    packages=[package_name, package_name + '.chassis_cmd'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 启动文件随包安装：装在 share/c50c_uart3/launch 下，供
        # ros2 launch c50c_uart3 c50c_uart3.launch.py 使用。
        ('share/' + package_name + '/launch',
            ['launch/c50c_uart3.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='snow',
    # catkin_pkg 要求 RFC 合法邮箱(snow@local 会使 package.xml 解析失败,
    # colcon 退化为普通 Python 项目、不生成 ament 索引钩子);example.com
    # 为保留占位域名,接入前可换成真实地址。
    maintainer_email='snow@example.com',
    description='C50C UART3 状态帧只读接收与 ROS 2 IMU 桥接（不发送底盘命令）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'c50c_uart3_node = c50c_uart3.uart3_node:main',
        ],
    },
)
