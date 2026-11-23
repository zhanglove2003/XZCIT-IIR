#!/usr/bin/env python3
"""colcon/pytest 结果与存量基线比对。

为什么需要这一步：
    仓库现有 179 个存量测试失败，其中 134 个是测试引用了从未纳入 git 的本机文件
    （src/deploy_stage1.sh、src/baseline-src/**、src/reports/** 等）。
    这些用例在干净 CI 环境里必然失败，若直接用 colcon test-result 卡门禁，
    feat 分支将永远无法合并任何 PR。

判定规则（棘轮）：
    基线内的失败  -> 放行（存量债务，不阻断）
    基线外的失败  -> 回归，退出码 1
    pytest 的 skip -> 不视为失败（任务书要求：合理 skip 不判失败）

用法（在仓库根目录执行）：
    python3 .github/scripts/compare_ros2_tests.py
"""

from __future__ import annotations

import glob
import sys
import xml.etree.ElementTree as ET

BASELINE = ".github/ros2-test-baseline.txt"
RESULT_GLOB = "Master/ROS/ros2_ws/build/*/pytest.xml"


def load_baseline(path: str) -> set[str]:
    keys: set[str] = set()
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line and not line.startswith("#"):
                keys.add(line)
    return keys


def main() -> int:
    base = load_baseline(BASELINE)
    files = sorted(glob.glob(RESULT_GLOB))
    if not files:
        print("::error::未找到任何 pytest.xml，colcon test 可能未真正执行")
        return 1

    total = skipped = known = 0
    new_failures: list[str] = []
    for path in files:
        root = ET.parse(path).getroot()
        for tc in root.iter("testcase"):
            total += 1
            if tc.find("skipped") is not None:
                skipped += 1
                continue
            if tc.find("failure") is None and tc.find("error") is None:
                continue
            key = f"{tc.get('classname')}::{tc.get('name')}"
            if key in base:
                known += 1
            else:
                new_failures.append(key)

    print(
        f"用例总数 {total}，skip {skipped}，"
        f"基线内失败 {known}，新增失败 {len(new_failures)}"
    )

    if new_failures:
        print("")
        print(f"::error::检测到 {len(new_failures)} 个基线外的测试失败（回归）")
        for key in sorted(new_failures):
            print(f"  - {key}")
        print("")
        print("处理方式：修复这些用例；若确认为既有债务，")
        print("经确认后把用例名追加到 .github/ros2-test-baseline.txt。")
        return 1

    print("OK：无新增测试失败")
    return 0


if __name__ == "__main__":
    sys.exit(main())
