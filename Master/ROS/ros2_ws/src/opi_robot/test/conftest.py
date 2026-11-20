"""让 `import opi_robot` 解析到本仓库源码目录，无需安装。"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)
if SRC not in sys.path:
    sys.path.insert(0, SRC)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fake_ros  # noqa: E402

fake_ros.install()

# ---------------------------------------------------------------------------
# 雷达测距过期判定用单调时钟。
#
# reader 的 ``stale_point_age`` 判定必须走 ``time.monotonic()``，不走 ROS 时钟。
# 本机与 CI 用真实墙钟会让"推进 1.6 秒"变成真的等 1.6 秒，测试也不可控。
# 因此每个测试前把 reader 模块内的 ``time`` 换成可控替身，
# 并在测试结束复位，避免跨用例串味。
# ---------------------------------------------------------------------------
import pytest  # noqa: E402

import lidar_fixtures  # noqa: E402
from opi_robot.lidar_node import reader as _reader  # noqa: E402


@pytest.fixture(autouse=True)
def _fake_monotonic(monkeypatch):
    import types

    start = 1000.0
    lidar_fixtures.set_monotonic(start)
    monkeypatch.setattr(
        _reader, 'time',
        types.SimpleNamespace(monotonic=lidar_fixtures.monotonic))
    yield lidar_fixtures
    lidar_fixtures.set_monotonic(start)
