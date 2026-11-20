"""B6 复核修补用例：`rearm_required` / `no_status_evidence` 的网页侧口径。

整树复核 R1 阻塞项 B6（reviews/tree-r1-reviewer.md:81-86）：guard 枚举
``ZERO_REASON``/``STOP_CAUSE`` 声明了 ``rearm_required``（故障已恢复、
等待显式 rearm），桥在缺底盘回传证据时使用 ``no_status_evidence``，但
``/patrol_status`` 只有 lock_stop / license_lost 两档，页面无法回答
"要不要人按恢复"——三个"同一套枚举"只做到字符串子集，语义未打通。

网页区修复（guard/桥一侧何时真正产出这两个字面属 c50c_uart3 改造区，
本次任务禁止触碰，见 reviews/b6-rearm-noevidence-fix.md 的 open 项）：

- ``patrol.rearm_state()``：锁停/许可/恢复动作 → ``rearm_required``
  （许可有效、锁停未解除，等用户按恢复）/ ``rearm_pending``（许可失效，
  先恢复许可）/ ``not_required``（锁停已解除）三档展示口径；
- ``patrol.status_frame`` 与 ``web_debug.patrol_status_frame`` 输出
  ``rearm_state`` 字段；
- ``console_page.html`` 锁停卡片转述 ``rearm_state``。

两次 pytest 实跑输出（docker c50c-humble）：

修复前（/tmp/b6web，三处网页改动还原后的副本；把本文件拷进去跑）::

  $ cd /tmp/b6web && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \\
    PYTHONPATH=.:/…/loop/candidate/c50c_uart3 python3 -m pytest -q test/test_b6_rearm_state.py
  FAILED test/test_b6_rearm_state.py::test_rearm_state_not_required_when_unlocked
  FAILED test/test_b6_rearm_state.py::test_rearm_state_required_with_pending_cancel
  FAILED test/test_b6_rearm_state.py::test_rearm_state_never_leaves_declared_set
  FAILED test/test_b6_rearm_state.py::test_status_frame_carries_rearm_state_locked
  FAILED test/test_b6_rearm_state.py::test_status_frame_carries_rearm_state_unlocked
  FAILED test/test_b6_rearm_state.py::test_web_patrol_status_frame_fallback_carries_rearm_state
  FAILED test/test_b6_rearm_state.py::test_console_page_renders_rearm_state
  10 failed in 0.12s

修复后（本文件即候选树内）::

  $ cd <candidate>/opi_robot && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \\
    PYTHONPATH=.:../c50c_uart3 python3 -m pytest -q test/test_b6_rearm_state.py
  ..........                                                               [100%]
  10 passed in 0.03s
"""

import json

import pytest

from opi_robot.web_node import patrol as P
from opi_robot.web_node import web_debug as W

# c50c_uart3 为独立包，只读其枚举做对账（与 test_reason_enum 相同约定）。
from c50c_uart3.motion_guard import ZERO_REASON  # noqa: E402

from test_patrol_session import send_ok  # noqa: E402,F401


# ---------------------------------------------------------------- 1. 口径函数
def test_rearm_state_enum_aligns_with_guard_declaration():
    """rearm_state 的取值必须落在 guard 已声明的枚举字面内（B6 对账）。"""
    for state in P.GUARD_REARM_STATES:
        if state == 'not_required':
            continue                      # 展示哨兵，不是 guard 零速原因
        assert state in ZERO_REASON, \
            'rearm_state %r 不在 guard ZERO_REASON 里，页面与下游无法对账' % state
    assert 'rearm_required' in ZERO_REASON
    assert 'no_status_evidence' in ZERO_REASON


def test_rearm_state_required_when_licensed_locked():
    """许可有效 + 锁停未解除：rearm_required（等用户按恢复）。"""
    assert P.rearm_state(True, True, False, None) == 'rearm_required'


def test_rearm_state_pending_when_license_lost():
    """许可失效：先恢复许可，恢复按钮必然失败 → rearm_pending。"""
    assert P.rearm_state(True, False, False, None) == 'rearm_pending'


def test_rearm_state_not_required_when_unlocked():
    assert P.rearm_state(False, True, True, None) == 'not_required'
    assert P.rearm_state(False, False, False, None) == 'not_required'


def test_rearm_state_required_with_pending_cancel():
    """有待确认取消（许可在、锁在）：仍是 rearm_required，且可对账。"""
    assert P.rearm_state(True, True, False, 7) == 'rearm_required'


def test_rearm_state_never_leaves_declared_set():
    """防御：任何输入都不得产出枚举之外的字面。"""
    for locked in (True, False):
        for licensed in (True, False):
            for released in (True, False):
                state = P.rearm_state(locked, licensed, released, 1)
                assert state in P.GUARD_REARM_STATES


# ---------------------------------------------------------------- 2. 状态帧
def test_status_frame_carries_rearm_state_locked():
    """锁停中（许可有效）的状态帧必须带 rearm_state=rearm_required。"""
    runner = P.PatrolRunner(dispatch=lambda g: 1, cancel=lambda r, k: True,
                            lock_stop=lambda reason: None, now=lambda: 0.0)
    lic = {'fresh': True}
    frame = P.status_frame(runner, lic, 'mapA')
    assert frame['locked'] is True
    assert frame['rearm_state'] == 'rearm_required'    # 修复前 KeyError，红
    assert json.dumps(frame['rearm_state']) in json.dumps(frame)


def test_status_frame_carries_rearm_state_unlocked():
    runner = P.PatrolRunner(dispatch=lambda g: 1, cancel=lambda r, k: True,
                            lock_stop=lambda reason: None, now=lambda: 0.0)
    runner.release_lock()
    frame = P.status_frame(runner, {'fresh': True}, 'mapA')
    assert frame['locked'] is False
    assert frame['rearm_state'] == 'not_required'


def test_web_patrol_status_frame_fallback_carries_rearm_state(send_ok):
    """web_debug.patrol_status_frame 的 runner/许可缺省兜底帧也要带口径。"""
    W.configure_patrol(reset_runner=True)
    W._patrol['runner'] = None
    W._patrol['license'] = None
    frame = W.patrol_status_frame()
    assert frame['rearm_state'] == 'rearm_pending'     # 修复前 KeyError，红


# ---------------------------------------------------------------- 3. 页面转述
def test_console_page_renders_rearm_state():
    """页面锁停卡片必须有 rearm_state 的转述（B6）。"""
    html = open(P.__file__.replace('patrol.py', 'console_page.html'),
                encoding='utf-8').read()
    assert 'patrol_rearm_state' in html                # 修复前红
    assert 'rearm_required' in html and 'rearm_pending' in html
