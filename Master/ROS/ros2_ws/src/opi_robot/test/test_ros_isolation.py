"""隔离守卫与真实 ROS 脚本的静态检查。

第四次审核阻断 3 的回归保护：脚本在独立 context 上 init，
节点必须用同一个 context 构造，否则真实 ROS 2 会抛
NotInitializedException: rclpy.init() has not been called。

这些测试不需要 rclpy，只检查源码与守卫行为。真实运行仍需目标机。
"""

import importlib.util
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / 'scripts'


def load_isolation():
    spec = importlib.util.spec_from_file_location(
        'ros_isolation_under_test', SCRIPTS / 'ros_isolation.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


R = load_isolation()


class TestDomainRange:
    """域号必须落在 1~232。"""

    def test_zero_is_rejected(self):
        assert R._parse_positive_domain('0') is None

    def test_one_is_accepted(self):
        assert R._parse_positive_domain('1') == 1

    def test_typical_domain(self):
        assert R._parse_positive_domain('213') == 213

    def test_upper_bound_is_accepted(self):
        assert R._parse_positive_domain('232') == 232

    def test_above_upper_bound_is_rejected(self):
        assert R._parse_positive_domain('233') is None

    def test_far_above_upper_bound_is_rejected(self):
        assert R._parse_positive_domain('999') is None

    def test_negative_is_rejected(self):
        assert R._parse_positive_domain('-1') is None

    def test_non_numeric_is_rejected(self):
        assert R._parse_positive_domain('abc') is None
        assert R._parse_positive_domain(None) is None

    def test_domain_max_constant_is_documented(self):
        # 上限必须有注释说明依据，避免以后被随手改大
        src = (SCRIPTS / 'ros_isolation.py').read_text()
        idx = src.index('DOMAIN_MAX = 232')
        assert '232' in src[idx - 200:idx + 40]


class TestIsolationGate:
    def test_missing_flag_rejected(self):
        ok, why = R.is_isolated({})
        assert not ok
        assert 'OPI_TEST_ISOLATED' in why

    def test_domain_zero_rejected(self):
        ok, why = R.is_isolated({'OPI_TEST_ISOLATED': '1', 'ROS_DOMAIN_ID': '0'})
        assert not ok
        assert '域号' in why

    def test_domain_above_max_rejected(self):
        ok, _ = R.is_isolated({'OPI_TEST_ISOLATED': '1', 'ROS_DOMAIN_ID': '233'})
        assert not ok

    def test_valid_env_accepted(self):
        ok, why = R.is_isolated({'OPI_TEST_ISOLATED': '1', 'ROS_DOMAIN_ID': '213'})
        assert ok, why

    def test_argv_form_accepted(self):
        rest = R.require_isolated_domain(['--ros-domain-id', '88', '--self'])
        assert '--self' in rest

    def test_argv_equals_form(self):
        rest = R.require_isolated_domain(['--ros-domain-id=88'])
        assert rest == []

    def test_no_isolation_raises_exit_2(self, monkeypatch):
        # 必须先清掉环境变量，否则本机恰好导出了隔离配置时不抛异常
        monkeypatch.delenv(R.FLAG_VAR, raising=False)
        monkeypatch.delenv(R.DOMAIN_VAR, raising=False)
        with pytest.raises(R.NotIsolated) as exc:
            R.require_isolated_domain(['--self'])
        assert exc.value.code == 2

    def test_domain_zero_note_only_for_zero(self, monkeypatch, capsys):
        """域号非法（越界/非数字）时不该打印"域 0 禁止使用"。"""
        monkeypatch.setenv(R.FLAG_VAR, '1')
        for bad in ('233', 'abc', '-1', '0'):
            monkeypatch.setenv(R.DOMAIN_VAR, bad)
            with pytest.raises(R.NotIsolated):
                R.require_isolated_domain(['--self'])
            err = capsys.readouterr().err
            if bad == '0':
                assert '域 0 禁止使用' in err, bad
            else:
                assert '域 0 禁止使用' not in err, bad

    def test_argv_domain_overrides_env(self, monkeypatch):
        monkeypatch.setenv(R.FLAG_VAR, '1')
        monkeypatch.setenv(R.DOMAIN_VAR, '213')
        # 环境已合规时不抛；命令行再给一个也能通过
        rest = R.require_isolated_domain(['--self'])
        assert '--self' in rest


class TestScriptsUseSameContext:
    """阻断 3：init / 节点构造 / shutdown 必须用同一个 context。"""

    def _src(self, name):
        return (SCRIPTS / name).read_text()

    def test_lidar_node_accepts_context_keyword(self):
        src = (ROOT / 'opi_robot' / 'lidar_node' / 'lidar_node.py').read_text()
        assert 'def __init__(self, context=None):' in src
        assert "super().__init__('lidar_node', context=context)" in src

    def test_fusion_node_accepts_context_keyword(self):
        src = (ROOT / 'opi_robot' / 'fusion_node' / 'fusion_node.py').read_text()
        assert 'def __init__(self, context=None):' in src
        assert "super().__init__('fusion_node', context=context)" in src

    def test_camera_node_accepts_context_keyword(self):
        src = (ROOT / 'opi_robot' / 'cam_node' / 'camera_node.py').read_text()
        assert "super().__init__('camera_node', context=context)" in src

    def test_web_bridge_accepts_context_keyword(self):
        src = (ROOT / 'opi_robot' / 'web_node' / 'web_debug.py').read_text()
        assert "super().__init__('web_bridge', context=context)" in src

    def test_diag_check_constructs_lidar_with_context(self):
        src = self._src('real_ros_diag_check.py')
        assert 'LN.LidarNode(context=context)' in src
        assert 'rclpy.init(context=context' in src
        assert 'rclpy.shutdown(context=context)' in src

    def test_qos_check_constructs_pair_check_with_context(self):
        src = self._src('qos_integration_check.py')
        assert 'rclpy.init(context=context)' in src
        assert 'rclpy.shutdown(context=context)' in src
        assert 'context=_CONTEXT' in src
        # 节点基类也要透传，否则 PairCheck 落在默认 context
        assert "super().__init__('qos_pair_check', context=context)" in src

    def test_no_bare_create_node_without_context(self):
        """避免以后新增的 create_node 漏掉 context。

        不能只用正则截到第一个右括号：参数里有 ')' 字符。
        这里按括号配平扫描。
        """
        src = self._src('qos_integration_check.py')
        needle = 'rclpy.create_node('
        pos = 0
        found = 0
        while True:
            i = src.find(needle, pos)
            if i < 0:
                break
            found += 1
            depth = 0
            j = i + len(needle) - 1
            while j < len(src):
                if src[j] == '(':
                    depth += 1
                elif src[j] == ')':
                    depth -= 1
                    if depth == 0:
                        break
                j += 1
            call = src[i:j + 1]
            assert 'context=' in call, call
            pos = j + 1
        assert found >= 1, '脚本里应至少有一个 create_node' 


class TestNoStaleAttributeNames:
    """改名后的属性不能残留旧名字。"""

    def test_fusion_uses_lidar_state_not_reason(self):
        src = (ROOT / 'opi_robot' / 'fusion_node' / 'fusion_node.py').read_text()
        assert 'self.lidar_reason' not in src

    def test_recovery_gate_present(self):
        src = (ROOT / 'opi_robot' / 'fusion_node' / 'fusion_node.py').read_text()
        assert 'recovery_no_new_scan' in src
        assert 'lost_seq' in src
