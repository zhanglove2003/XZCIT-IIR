"""任务 B 测试夹具（test_chassis_cmd 子目录专用）。

- ``firmware``：常驻固件解析器（真实固件片段编译），每个测试一份新板。
- ``fake_clock``：可控单调时钟，避免测试依赖真实时间。
- ``chain``：仲裁器 + 内存假串口。
"""
import pytest

from c50c_uart3.chassis_cmd.arbiter import Arbiter
from c50c_uart3.chassis_cmd.chain import StraightCommandChain
from c50c_uart3.chassis_cmd.transport import MemoryTransport
from sim_confirm import mark_endpoint_clean, mark_chain_clean, \
    new_receipt as _new_receipt, grant_safety, TEST_SAFETY_MAX_AGE_S

_SIM = _new_receipt('conftest')


class FakeClock:
    """单调时钟替身：只在显式 ``advance`` 时前进。"""

    def __init__(self, start: float = 100.0) -> None:
        self.t = float(start)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> float:
        self.t += float(dt)
        return self.t


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def firmware():
    from fw_parser_harness import FirmwareModel
    model = FirmwareModel.start()
    try:
        yield model
    finally:
        model.close()


@pytest.fixture
def transport():
    # 第六轮：端口默认"不可判定 = 按脏处理"。内存假串口没有残帧，
    # 测试用显式证据把它声明为干净。生产路径没有这个入口。
    port = MemoryTransport()
    mark_endpoint_clean(port, receipt=_SIM)
    return port


@pytest.fixture
def chain(transport, clock):
    # 2026-11-14 起仲裁器有安全许可门（无许可 = 禁非零）。既有测试验证的
    # 是许可**之后**的仲裁/传输行为，这里模拟"fusion_node 在线且安全"：
    # 生产路径上许可只能来自 /drive_safety 话题，且每 0.1 s 重发。
    # 时限放大到测试尺度；许可门本身的拒绝/过期/钳制行为由
    # test_safety_gate.py 单独验证。
    arbiter = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5,
                      safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
    grant_safety(arbiter, clock())
    return StraightCommandChain(arbiter, transport)
