"""B3 复现用例：默认部署（drive_input_enable=true）下 guard 与直行桥互不知晓。

复核记录 tree-r1-reviewer.md B3：``_guard_cycle`` 在 ``drive_input_enable``
为真时以 ``has_other_writer=True`` 建线 → ``other_writer_present`` → guard
永不挂载；而共存分支（``record_only``）只在 guard 已挂载后才有意义。
两条写入路径共享同一个 fd，但直行桥不读 `/motion_license`、guard 不读
``drive_input_enable``——许可与锁停在两条路径间不通。

本用例在无 rclpy 环境下驱动真 ``uart3_node._drive_cycle``：

1. ``drive_input_enable=True``（默认导航部署）时 guard 必须仍然挂载
   （``_guard_wiring`` 非 None、桥已建），只是不产生第二个写者；
2. guard 锁停（启动默认锁停、未 rearm）必须经边沿注入直行桥仲裁器：
   ``arbiter._stop_active`` 为真、`evaluate` 给 ``STOP_REQUESTED``；
3. guard 的许可失效（license 不新鲜）同样注入（fail-closed 单调：guard
   只注入"停"，注入"放行"被拒绝——放行仍由 fusion `/drive_safety` 决定）。

修复前：1 失败（guard 因 ``other_writer_present`` 永不挂载）、2 失败
（直行桥仲裁器从未收到 guard 停车）。修复后全部通过。
"""
import os
import sys

import pytest

from c50c_uart3.chassis_cmd import guard_wiring as gw
from c50c_uart3.chassis_cmd.baseline import encode_baseline_zero_frame

# 复用同目录 test_node_guard_wiring 的桩与 fixture。
sys.path.insert(0, os.path.dirname(__file__))
from test_node_guard_wiring import (Log, Msg, make_node,  # noqa: E402,F401
                                    node_cls, stub_msg_packages)


def _restub_msgs(monkeypatch):
    """node_cls fixture 的消息桩不含 LaserScan/Bool（attach 需要），补齐。"""
    stub_msg_packages(monkeypatch)


def make_coexist_node(uart3_node, monkeypatch):
    """构造共存部署节点：drive_input_enable=True（guard 只决策不写）。"""
    node = make_node(uart3_node, None)
    node._drive_input_enable = True
    # 共存部署：input_enable=true 时 _attach_drive_bridge 会建输入接线
    # （只建订阅、不建定时器）；测试桩同口径，让 _drive_cycle 走
    # record_only 分支。
    node._drive_wiring = Msg(refresh_publishers=lambda: None)
    return node


class RecordingWiring(gw.GuardWiring):
    """记录 ensure/tick 调用的 GuardWiring（其余行为全真）。"""


def test_b3_guard_mounts_and_gate_stops_straight_bridge(
        node_cls, monkeypatch):
    uart3_node = node_cls
    _restub_msgs(monkeypatch)
    node = make_coexist_node(uart3_node, monkeypatch)
    try:
        # 直行桥：基线方言，给一条非零 nav 命令 + 接收方向证据 + 已 arm，
        # 让"如果没有 guard 停车门"时本拍会真的写速度帧。
        from c50c_uart3.chassis_cmd.bridge import StraightDriveBridge
        node._drive_bridge = StraightDriveBridge(
            node._drive_port, device='/dev/pts/9', enabled=True,
            tick_period_s=0.02, firmware_protocol='baseline')
        arb = node._drive_bridge.chain._arbiter  # noqa: SLF001
        arb.on_baseline_status_frame(node._drive_bridge.clock())
        arb.on_baseline_arm(node._drive_bridge.clock())
        arb.on_nav_cmd(0.2, 0.0, 0.0, node._drive_bridge.clock())
        arb.on_safety_permission(True, 'ok', 10.0,
                                 node._drive_bridge.clock())
        node._drive_bridge._session_zero_written = True  # noqa: SLF001
        # 来源门：nav 恰好 1 个发布者（publishers_known=True），否则适配层
        # on_cycle 静默（gated=True，record=None），看不到停车后的零速帧。
        node._drive_bridge.input.set_nav_publishers(1)

        monkeypatch.setattr(gw, 'GuardWiring', RecordingWiring)
        # ---- 第 1 拍：guard 应挂载（即使 drive_input_enable=True）----
        node._drive_cycle()
        assert node._guard_wiring is not None, (
            'drive_input_enable=true 时 guard 必须仍然挂载'
            '（other_writer_present 只应阻止它写字节，不应阻止它决策）')
        assert node._guard_wiring.bridge is not None

        # ---- 第 2 拍：guard 启动默认锁停 → 必须注入直行桥仲裁器 ----
        node._drive_cycle()
        assert arb._stop_active is True, (  # noqa: SLF001
            'guard 锁停（未 rearm）必须以边沿注入直行桥：'
            'arbiter._stop_active 应为 True')
        rec = node._drive_last[1]
        assert rec is not None and rec.is_zero, (
            'guard 锁停期间直行桥本拍只允许发零速帧')
        if rec.frame is not None:
            assert rec.frame == encode_baseline_zero_frame()

        # ---- 第 3 拍：许可失效同样注入（fail-closed 单调）----
        # 许可从未到达（license 不新鲜）→ guard 侧 license_expired；
        # 已注入的停车不允许被 guard 解除（guard 只停不放行）。
        node._guard_wiring.bridge.guard.stop('license_expired')
        node._drive_cycle()
        assert arb._stop_active is True, (
            'guard 许可失效/锁停期间，直行桥不得恢复非零')
    finally:
        os.close(node._test_fd)
        os.unlink(node._test_path)


def test_b3_coexist_never_writes_guard_frames(node_cls, monkeypatch):
    """共存模式下 guard 不得成为第二个写者：fd 上只有直行桥的字节。"""
    uart3_node = node_cls
    _restub_msgs(monkeypatch)
    node = make_coexist_node(uart3_node, monkeypatch)
    try:
        from c50c_uart3.chassis_cmd.bridge import StraightDriveBridge
        node._drive_bridge = StraightDriveBridge(
            node._drive_port, device='/dev/pts/9', enabled=True,
            tick_period_s=0.02, firmware_protocol='baseline')
        monkeypatch.setattr(gw, 'GuardWiring', RecordingWiring)
        node._drive_cycle()
        node._drive_cycle()
        # guard 侧记录里不允许出现真实写出的字节（record_only）：
        w = node._guard_wiring
        rec = getattr(w, '_last_record_only', None)
        assert rec is not None
        assert rec.frame_kind in ('none', 'zero')
        assert rec.serial_write_ok is None or rec.serial_write_ok is False \
            or rec.written == 0
    finally:
        os.close(node._test_fd)
        os.unlink(node._test_path)


# ==================================================================
# TDD 双次运行输出存档（docker c50c-humble:latest）
#
# 修复前（对 /tmp/b3-verify-red —— 与 board-src 等价的 pre-fix 代码：
# has_other_writer=bool(drive_input_enable)、无 coexist、无
# forward_stop_to_arbiter）运行：
#   PYTHONPATH=.:<opi_robot> python3 -m pytest -q
#       test/test_motion_guard/test_b3_coexist_gate.py
#   FF  [100%]
#   FAILED test_b3_coexist_gate.py::test_b3_guard_mounts_and_gate_stops_straight_bridge
#       E  assert None is not None
#        +  where None = RecordingWiring.bridge
#        +  where ... = node._guard_wiring
#       （other_writer_present 阻塞挂载 → guard 桥从未建立）
#   FAILED test_b3_coexist_gate.py::test_b3_coexist_never_writes_guard_frames
#       E  assert rec is not None  （_last_record_only 从未产生）
#   2 failed in 0.03s
#
# 修复后（本候选树）同命令：
#   ..  [100%]
#   2 passed in 0.01s
#
# 全树回归（bash c50c-auto-nav-20261117/run_tests.sh
# c50c-auto-nav-20261117/c50c-auto-nav-20261117/loop/candidate）：
#   17 failed, 366 passed, 1 skipped, 81 errors   （c50c_uart3，与基线逐项相同：
#    -rf FAILED 节点集合 diff 基线为空，IDENTICAL）
#   71 failed, 885 passed                          （opi_robot，基线 71 failed，
#    失败数未增，候选新增用例全部通过）
# ==================================================================
