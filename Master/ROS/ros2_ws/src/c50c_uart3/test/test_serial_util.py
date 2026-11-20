"""串口占用检查（find_device_holders）定向测试（离线，不依赖 ROS、不接触真实串口）。

同时验证审核确认的 TIOCEXCL 语义：它只阻止之后的新打开请求，
不能排除设置之前已打开设备的进程。
"""
import os
import subprocess
import sys
import time

import pytest

from c50c_uart3.serial_util import find_device_holders


def test_no_holder_on_fresh_pty():
    master_fd, slave_fd = os.openpty()
    try:
        assert find_device_holders(os.ttyname(slave_fd)) == []
    finally:
        os.close(master_fd)
        os.close(slave_fd)


def test_detects_external_process_holding_pty():
    master_fd, slave_fd = os.openpty()
    slave_name = os.ttyname(slave_fd)
    try:
        # 子进程持有 slave fd 5 秒，模拟"先于节点打开的进程"
        holder = subprocess.Popen(
            [sys.executable, '-I', '-c',
             'import os,sys,time\n'
             'fd = os.open(sys.argv[1], os.O_RDONLY | os.O_NOCTTY)\n'
             'print("ready", flush=True)\n'
             'time.sleep(5)\n', slave_name],
            stdout=subprocess.PIPE, text=True)
        try:
            assert holder.stdout.readline().strip() == 'ready'
            holders = find_device_holders(slave_name)
            assert str(holder.pid) in holders, \
                '应检测到先打开设备的子进程'
        finally:
            holder.terminate()
            holder.wait(timeout=5)
    finally:
        os.close(master_fd)
        os.close(slave_fd)


def test_tiocexcl_blocks_later_open_but_not_earlier_holders():
    # 审核复现的语义：已有持有者不受影响，新打开者得到 EBUSY
    master_fd, slave_fd = os.openpty()
    slave_name = os.ttyname(slave_fd)
    import fcntl
    import termios
    try:
        holder = subprocess.Popen(
            [sys.executable, '-I', '-c',
             'import os,sys,time\n'
             'fd = os.open(sys.argv[1], os.O_RDONLY | os.O_NOCTTY)\n'
             'print("ready", flush=True)\n'
             'time.sleep(5)\n', slave_name],
            stdout=subprocess.PIPE, text=True)
        try:
            assert holder.stdout.readline().strip() == 'ready'
            # 本进程此时打开（早于 TIOCEXCL 设置），应成功
            early_fd = os.open(slave_name, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
            try:
                fcntl.ioctl(early_fd, termios.TIOCEXCL)
                # TIOCEXCL 之后的新打开应失败
                with pytest.raises(OSError):
                    os.open(slave_name, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
            finally:
                os.close(early_fd)
        finally:
            holder.terminate()
            holder.wait(timeout=5)
    finally:
        os.close(master_fd)
        os.close(slave_fd)


def test_nonexistent_device_returns_empty():
    assert find_device_holders('/dev/does-not-exist-c50c-test') == []


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__]))
