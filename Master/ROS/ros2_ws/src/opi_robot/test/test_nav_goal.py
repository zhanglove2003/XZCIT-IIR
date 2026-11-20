"""[C50C 阶段 D 2026-11-16] 网页点图导航 nav_goal 模块单测。

只测不依赖 rclpy 的纯逻辑：目标校验/状态机/视图。ActionClient 接线
（attach/_send）走板上集成验证，不在离线套件里。
"""

import importlib.util
import os

# 独立加载 nav_goal.py（它不 import 同包其他模块，单文件可加载）。
# 本地测试根与板上包布局不同：先试同目录（离线根），再试包内 web_node/。
_HERE = os.path.dirname(os.path.abspath(__file__))
_CANDIDATES = [
    os.path.join(_HERE, 'nav_goal.py'),
    os.path.join(_HERE, '..', 'opi_robot', 'web_node', 'nav_goal.py'),
]
_nav_path = next((p for p in _CANDIDATES if os.path.exists(p)), None)
if _nav_path is None:
    raise FileNotFoundError('nav_goal.py 不在已知位置: %r' % _CANDIDATES)
_SPEC = importlib.util.spec_from_file_location('nav_goal', _nav_path)
NG = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(NG)


class TestRequestGoalValidation:
    def _ctrl(self):
        return NG.NavGoalControl()

    def test_non_numeric_rejected(self):
        c = self._ctrl()
        ok, v = c.request_goal('abc', 1.0)
        assert not ok
        assert v['state'] == 'idle'
        assert '数字' in v['error']

    def test_nan_inf_rejected(self):
        c = self._ctrl()
        ok, v = c.request_goal(float('nan'), 1.0)
        assert not ok
        ok, v = c.request_goal(1.0, float('inf'))
        assert not ok

    def test_out_of_range_rejected(self):
        c = self._ctrl()
        ok, v = c.request_goal(999.0, 0.0)
        assert not ok
        assert '超出地图范围' in v['error']
        ok, v = c.request_goal(-50.0, 100.0)
        assert not ok

    def test_valid_in_range_accepted_shape(self):
        """范围合法 → 进入 sending（无 client 时 _send 失败转 failed）。"""
        c = self._ctrl()
        # 无 rclpy/ActionClient：attach 没跑，_send 里 import 失败 → failed
        ok, v = c.request_goal(19.1, 2.37)
        assert ok or v['state'] == 'failed'
        assert v['goal']['x'] == 19.1
        assert v['goal']['y'] == 2.37
        assert v['seq'] == 1

    def test_th_none_kept(self):
        c = self._ctrl()
        c.request_goal(19.1, 2.37, None)
        assert c.view()['goal']['th'] is None

    def test_th_rounded(self):
        c = self._ctrl()
        c.request_goal(19.1, 2.37, 0.264012345)
        assert c.view()['goal']['th'] == 0.264

    def test_duplicate_goal_rejected_while_active(self):
        c = self._ctrl()
        c._state = 'active'          # 直置 active 模拟进行中
        ok, v = c.request_goal(19.1, 2.37)
        assert not ok
        assert '进行中' in v['error']


class TestCancel:
    def test_cancel_without_goal(self):
        c = NG.NavGoalControl()
        ok, v = c.cancel()
        assert not ok
        assert '没有进行中' in v['error']

    def test_cancel_active_state(self):
        c = NG.NavGoalControl()
        c._state = 'active'
        c._goal_handle = object()    # 假句柄：cancel_goal_async 调用被吞
        ok, v = c.cancel()
        assert ok
        assert v['state'] == 'canceled'


class TestView:
    def test_view_fields_present(self):
        c = NG.NavGoalControl()
        v = c.view()
        for k in ('state', 'goal', 'age', 'feedback', 'result', 'error',
                  'client', 'seq'):
            assert k in v
        assert v['state'] == 'idle'
        assert v['client'] is False
        assert v['goal'] is None

    def test_snapshot_error_overrides(self):
        c = NG.NavGoalControl()
        v = c._snapshot('自定义错误')
        assert v['error'] == '自定义错误'


class TestConstants:
    def test_goal_bounds_cover_scanned_area(self):
        """地图 origin -8.28,-8.03、584×256@0.05 → 已扫区约 -8..21,-8..5。

        限幅必须覆盖已扫区、卡在明显出图处。
        """
        assert NG.GOAL_MIN_X <= -8.0
        assert NG.GOAL_MAX_X >= 21.0
        assert NG.GOAL_MIN_Y <= -8.0
        assert NG.GOAL_MAX_Y >= 5.0
