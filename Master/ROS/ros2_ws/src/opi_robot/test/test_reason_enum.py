"""B5 复核修补用例：patrol 停车原因与 guard 枚举对齐。

复核记录（reviews/tree-r1-reviewer.md §B5）指出：

- patrol ``LOCK_REASON_TEXT`` 的 9 个原因里与 guard ``ZERO_REASON``/``STOP_CAUSE``
  交集只有 ``global_stop``；``patrol_downstream_lock_stop`` 把原因整个丢掉
  （只撤回鉴权），下游只能按 ``global_stop`` 锁停——页面上显示
  ``nav_failed``，下游记录却是另一套字面，无法对账。
- ``map_changed`` 是 stop_reason 却不在 ``LOCK_REASON_TEXT`` 里，
  页面只能显示英文原文。

修复（网页区）：
- ``patrol.PATROL_TO_GUARD_CAUSE``：patrol 锁停原因 → guard stop_cause 枚举
  的唯一权威映射（``guard_stop_cause()``）；
- ``patrol.LOCK_REASON_TEXT`` 补齐 ``map_changed``；
- ``patrol_downstream_lock_stop`` 不再丢原因：记录映射后的
  ``last_stop_cause``，诊断可对账。
"""

import json

import pytest

from opi_robot.web_node import patrol as P
from opi_robot.web_node import web_debug as W

# c50c_uart3 的枚举是"另一套"的那一套：这里只读它做对账，不改它。
from c50c_uart3.motion_guard import ZERO_REASON, STOP_CAUSE  # noqa: E402
from test_patrol_session import send_ok  # noqa: E402,F401


GUARD_CAUSES = (set(ZERO_REASON) | set(STOP_CAUSE)) - {'none', 'no_demand'}


# ---------------------------------------------------------------- 1. 映射完备
def test_patrol_lock_reasons_map_into_guard_enum():
    """patrol 每个锁停原因都必须映射到 guard 枚举里的一个 stop_cause。"""
    for reason in P.LOCK_REASON_TEXT:
        cause = P.guard_stop_cause(reason)
        assert cause in GUARD_CAUSES, \
            'patrol 原因 %r 映射到 %r，不在 guard 枚举里' % (reason, cause)


def test_mapping_is_honest_per_reason():
    """映射必须逐条诚实：许可丢失=license_expired；其余显式停车=global_stop。"""
    assert P.guard_stop_cause('license_lost') == 'license_expired'
    assert P.guard_stop_cause('global_stop') == 'global_stop'
    assert P.guard_stop_cause('manual_takeover') == 'global_stop'
    for reason in ('no_safe_path', 'nav_failed', 'nav_rejected',
                   'dispatch_failed', 'goal_timeout', 'web_disconnected',
                   'map_changed'):
        assert P.guard_stop_cause(reason) == 'global_stop', reason
    assert P.guard_stop_cause('') == 'global_stop'      # 空原因兜底
    assert P.guard_stop_cause('未知新原因') == 'global_stop'


def test_map_changed_has_display_text():
    assert 'map_changed' in P.LOCK_REASON_TEXT


# ---------------------------------------------------------------- 2. 下游通道
class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(getattr(msg, 'data', msg))


class _Node:
    def create_publisher(self, typ, topic, qos):
        return _Pub()


def test_lock_stop_sink_records_guard_cause(send_ok):
    """runner 锁停时下游通道必须记下映射后的 stop_cause，不得丢原因。"""
    W.configure_patrol(reset_runner=True)
    W.patrol_downstream_attach(_Node())
    runner = W._patrol['runner']
    runner.lock_stop('nav_failed')
    assert W._patrol_downstream['last_stop_cause'] == 'global_stop'
    runner.release_lock()
    runner.lock_stop('license_lost')
    assert W._patrol_downstream['last_stop_cause'] == 'license_expired'


def test_status_frame_carries_guard_stop_cause(send_ok):
    """/patrol_status 帧带 guard 口径 stop_cause，页面可与下游记录对账。"""
    W.configure_patrol(reset_runner=True)
    W._patrol['license'] = P.MotionLicense(
        lambda reason: None, now=W._patrol_now)
    lic = W._patrol['license']
    lic.accept(1, 0)
    runner = W._patrol['runner']
    runner.release_lock()                  # runner 出厂即带 global_stop 首因锁
    runner.lock_stop('nav_failed')
    frame = W.patrol_status_frame()
    assert frame['locked'] is True
    assert frame['lock_reason'] == 'nav_failed'
    assert frame['stop_cause'] == 'global_stop'
    assert json.dumps(frame['stop_cause']) in json.dumps(frame)
