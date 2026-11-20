"""入口完整性测试：四个可执行入口都能导入，节点类都能构造。

不连硬件、不起 ROS 运行时，只确认 setup.py 的 console_scripts
与包内模块路径一致。这类错误只有部署到设备后才会暴露，先在这里挡住。
"""

import importlib

import pytest

import fake_ros

ENTRY_MODULES = [
    'opi_robot.lidar_node.lidar_node',
    'opi_robot.fusion_node.fusion_node',
    'opi_robot.web_node.web_debug',
]


@pytest.mark.parametrize('name', ENTRY_MODULES)
def test_entry_module_imports_and_has_main(name):
    mod = importlib.import_module(name)
    assert callable(getattr(mod, 'main', None)), '%s 缺少 main()' % name


def test_all_packages_exist():
    for pkg in ('opi_robot', 'opi_robot.lidar_node',
                'opi_robot.fusion_node', 'opi_robot.web_node'):
        assert importlib.import_module(pkg) is not None


def test_lidar_node_constructs():
    from opi_robot.lidar_node.lidar_node import LidarNode
    node = LidarNode()
    assert 'scan' in node.publishers
    assert 'diagnostics' in node.publishers


def test_fusion_node_constructs():
    from opi_robot.fusion_node.fusion_node import FusionNode
    node = FusionNode()
    assert 'auto_cmd_vel' in node.publishers
    assert 'obstacle_report' in node.publishers
    # 第一阶段不得再发布 visual_target 占位话题
    assert 'visual_target' not in node.publishers


def test_web_bridge_constructs():
    from opi_robot.web_node.web_debug import Bridge
    bridge = Bridge()
    assert 'diagnostics' in bridge.subscriptions
