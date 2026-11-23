#!/usr/bin/env python3
"""串口占用检查辅助（无 ROS 依赖，便于离线测试）。

适用范围说明（重要）：``find_device_holders`` 是**尽力而为的提示**，
不是无竞态的独占保证——

- 只反映扫描瞬间的状态：扫描后、打开设备前仍可能有进程打开（竞态窗口）；
- 读不到无权限进程的 fd（``/proc/<pid>/fd`` 对他人进程受限），覆盖不全；
- 不检测内核内部占用（如某些驱动内部引用）。

真正的独占语义只有 ``TIOCEXCL``：它只限制设置之后的**新**打开请求，
不能排除设置前已打开设备的进程。因此现场仍必须遵守启动顺序：
先关闭监视器/读取程序，再启动桥接节点。
"""
from __future__ import annotations

import os


def find_device_holders(device: str) -> list[str]:
    """返回疑似打开了 device 的其他进程 PID 列表（字符串）。

    同时匹配设备的真实路径与原始路径：``/proc/<pid>/fd`` 的符号链接
    通常显示内核解析后的设备路径（如 /dev/ttyUSB0），而打开时用的可能
    是 /dev/serial/by-path/... 之类的别名。自身进程不列入。
    """
    try:
        dev_real = os.path.realpath(device)
        if not os.path.exists(dev_real):
            return []
    except OSError:
        return []
    candidates = {dev_real, device}
    my_pid = str(os.getpid())
    holders: list[str] = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or entry == my_pid:
            continue
        fd_dir = f'/proc/{entry}/fd'
        try:
            for fd_name in os.listdir(fd_dir):
                try:
                    target = os.readlink(f'{fd_dir}/{fd_name}')
                except OSError:
                    continue
                if target in candidates:
                    holders.append(entry)
                    break
        except OSError:
            continue  # 无权限（他人进程）或进程已退出
    return sorted(holders)
