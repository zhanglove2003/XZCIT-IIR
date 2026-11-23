"""测试路径引导：未安装包的情况下让 c50c_uart3 与测试辅助模块可导入。"""
import sys
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
PKG_SOURCE_ROOT = TEST_DIR.parent

for path in (str(PKG_SOURCE_ROOT), str(TEST_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

# 固件侧观测夹具：test/firmware_harness 用 gcc 编译真实固件片段，
# 不依赖 ROS，也不打开任何真实串口。
FW_HARNESS_DIR = TEST_DIR / 'firmware_harness'
if str(FW_HARNESS_DIR) not in sys.path:
    sys.path.append(str(FW_HARNESS_DIR))
