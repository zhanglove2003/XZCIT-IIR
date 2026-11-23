"""启动配置的定向测试：``drive_firmware_protocol`` 必须显式写成 baseline。

背景
----
``uart3_node`` 的参数默认值是 ``DEFAULT_FIRMWARE_PROTOCOL``（``'v3'``）。
v3 是未交付的会话方言。板上跑 2026-11-05 基线固件。启动时不给参数，节点
就按 v3 说话，固件不认。所以启动配置必须**显式**写出方言，不能靠默认值。

本文件验证的是配置层：启动文件存在、参数名与包内常量一致、默认值不是
v3、且被写进节点的 ``parameters``。它**不**证明板上固件是 baseline，
也**不**证明任何命令被执行。

无 ROS 环境：用替身模块模拟 ``launch``/``launch_ros``，只执行
``generate_launch_description()`` 这一个函数，不启动任何进程、不打开串口。
"""
from __future__ import annotations

import ast
import pathlib
import sys
import types

import pytest

from c50c_uart3.chassis_cmd.arbiter import (FIRMWARE_PROTOCOLS,
                                            PROTOCOL_BASELINE)

_ROOT = pathlib.Path(__file__).resolve().parents[2]       # …/c50c_uart3
_LAUNCH = _ROOT / 'launch' / 'c50c_uart3.launch.py'
_SETUP = _ROOT / 'setup.py'


class _LaunchConfiguration:
    """``LaunchConfiguration`` 替身：只记名字。"""

    def __init__(self, name: str) -> None:
        self.name = name

    def __repr__(self) -> str:
        return f'<LC {self.name}>'


class _DeclareLaunchArgument:
    def __init__(self, name, default_value=None, description=None):
        self.name = name
        self.default_value = default_value
        self.description = description


class _Node:
    def __init__(self, package, executable, name, output, parameters):
        self.package = package
        self.executable = executable
        self.name = name
        self.output = output
        self.parameters = parameters


class _LaunchDescription:
    def __init__(self, entities):
        self.entities = list(entities)


def _import_launch_module():
    """无 ROS 环境导入 launch 文件：只替换 ``launch``/``launch_ros``。"""

    def mod(name, **attrs):
        m = sys.modules.get(name) or types.ModuleType(name)
        m.__path__ = getattr(m, '__path__', [])
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    mod('launch', LaunchDescription=_LaunchDescription)
    mod('launch.actions', DeclareLaunchArgument=_DeclareLaunchArgument)
    mod('launch.substitutions', LaunchConfiguration=_LaunchConfiguration)
    mod('launch_ros')
    mod('launch_ros.actions', Node=_Node)

    import importlib.util
    spec = importlib.util.spec_from_file_location('_c50c_launch', _LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _parsed():
    msgs = _import_launch_module()
    ld = msgs.generate_launch_description()
    assert isinstance(ld, _LaunchDescription)
    args = {e.name: e for e in ld.entities
            if isinstance(e, _DeclareLaunchArgument)}
    nodes = [e for e in ld.entities if isinstance(e, _Node)]
    return msgs, args, nodes


# ------------------------------------------------------------------ 文件存在
def test_launch_file_exists_and_is_installed():
    """启动文件存在，且被 ``setup.py`` 装进 share 目录。"""
    assert _LAUNCH.is_file(), f'缺少启动文件 {_LAUNCH}'
    setup_text = _SETUP.read_text(encoding='utf-8')
    assert 'launch/c50c_uart3.launch.py' in setup_text, (
        'setup.py 未安装启动文件，ros2 launch 找不到它')


# --------------------------------------------------- 显式 baseline，不是 v3
def test_protocol_default_matches_package_constant():
    """启动文件的 baseline 字面量与包内常量同值，避免两处漂移。"""
    msgs, _args, _nodes = _parsed()
    assert msgs.BUNDLED_FIRMWARE_PROTOCOL == PROTOCOL_BASELINE
    assert msgs.BUNDLED_FIRMWARE_PROTOCOL in FIRMWARE_PROTOCOLS


def test_protocol_argument_is_declared_with_baseline_default():
    """``drive_firmware_protocol`` 被显式声明，默认 baseline，不是 v3。"""
    _msgs, args, _nodes = _parsed()
    assert 'drive_firmware_protocol' in args, (
        '启动文件必须声明 drive_firmware_protocol')
    default = args['drive_firmware_protocol'].default_value
    assert default == PROTOCOL_BASELINE
    assert default != 'v3', '不得把未交付的 v3 当作车载协议'


def test_protocol_is_passed_into_node_parameters():
    """协议参数真的写进节点 ``parameters``，不是只声明不用。"""
    msgs, _args, nodes = _parsed()
    assert len(nodes) == 1
    node = nodes[0]
    assert node.package == 'c50c_uart3'
    assert node.executable == 'c50c_uart3_node'
    params = node.parameters
    assert isinstance(params, list) and len(params) == 1
    table = params[0]
    assert 'drive_firmware_protocol' in table, '协议参数未传给节点'
    value = table['drive_firmware_protocol']
    assert isinstance(value, _LaunchConfiguration)
    assert value.name == 'drive_firmware_protocol', (
        '必须用 LaunchConfiguration 传参，才能被命令行覆盖')


def test_literal_default_not_inherited_from_node_default():
    """默认值必须是字面量 baseline，不能是空串或交给节点默认值。"""
    _msgs, args, _nodes = _parsed()
    default = args['drive_firmware_protocol'].default_value
    assert isinstance(default, str) and default, (
        '默认值必须是明确字符串；留空会让节点回落到 v3')


# --------------------------------------------------------------- 安全默认
@pytest.mark.parametrize('name', ['drive_enable', 'drive_input_enable',
                                  'drive_fd_authorised'])
def test_write_paths_default_off(name):
    """三项写入开关默认 false：启动不等于授权发命令。"""
    _msgs, args, nodes = _parsed()
    assert name in args, f'启动文件必须声明 {name}'
    assert args[name].default_value == 'false'
    assert name in nodes[0].parameters[0]


def test_odom_default_off_and_device_is_explicit():
    """里程计默认关；设备路径写死实测值，不留空。"""
    _msgs, args, _nodes = _parsed()
    assert args['odom_enable'].default_value == 'false'
    assert args['device'].default_value == '/dev/ttyACM1'
    assert args['baudrate'].default_value == '115200'


# ------------------------------------------------- 与生产节点参数名一致
def test_every_passed_parameter_is_declared_by_the_node():
    """启动文件传的每个参数名，生产节点都必须声明过，否则会被忽略。"""
    _msgs, _args, nodes = _parsed()
    passed = set(nodes[0].parameters[0])
    src = (_ROOT / 'c50c_uart3' / 'uart3_node.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    declared = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter'
                and node.args and isinstance(node.args[0], ast.Constant)):
            declared.add(node.args[0].value)
    missing = passed - declared
    assert not missing, f'启动文件传了节点未声明的参数: {sorted(missing)}'


def test_node_default_stays_v3_and_is_not_changed_by_launch():
    """节点侧默认值仍是 v3（既有行为不变）；baseline 只由启动配置选出。"""
    from c50c_uart3.chassis_cmd.bridge import DEFAULT_FIRMWARE_PROTOCOL
    assert DEFAULT_FIRMWARE_PROTOCOL == 'v3'
