"""写入端守卫：真实设备一律拒绝，隔离目标只允许伪终端。

这些测试证明"本候选不会打开真实串口"是**代码属性**，不是约定。
"""
import os
import pytest

from c50c_uart3.chassis_cmd.transport import (MemoryTransport,
                                              RealDeviceAccessForbidden,
                                              SerialOwnerTransport,
                                              assert_isolated_target,
                                              open_transport)


@pytest.mark.parametrize('path', ['/dev/ttyUSB0', '/dev/ttyAMA0', '/dev/ttyS3',
                                  '/dev/serial/by-id/usb-xyz',
                                  '/dev/ttyAMA0-open', '', 'ttyUSB0',
                                  '/dev/pts/12x', '/dev/pts/'])
def test_real_registry_paths_rejected(path):
    with pytest.raises(RealDeviceAccessForbidden):
        assert_isolated_target(path)


def test_pts_path_accepted():
    assert assert_isolated_target('/dev/pts/0') is None


def test_open_transport_refuses_real_device():
    with pytest.raises(RealDeviceAccessForbidden):
        open_transport('/dev/ttyAMA0')


def test_serial_owner_transport_is_not_usable():
    with pytest.raises(RealDeviceAccessForbidden):
        SerialOwnerTransport('/dev/pts/3')


def test_open_transport_default_is_memory():
    t = open_transport('memory://')
    assert isinstance(t, MemoryTransport)
    assert isinstance(open_transport(''), MemoryTransport)


def test_memory_transport_records_writes():
    t = MemoryTransport()
    assert t.write(b'\x01\x02') is True
    assert t.writes == [b'\x01\x02']


def test_memory_transport_can_simulate_failure():
    t = MemoryTransport()
    t.fail_next = 2
    assert t.write(b'a') is False
    assert t.write(b'b') is False
    assert t.write(b'c') is True
    assert t.writes == [b'c']


def test_pty_is_used_only_when_test_creates_it():
    """伪终端是隔离目标；本模块不自己开设备，由测试注入。"""
    master, slave = os.openpty()
    try:
        path = os.ttyname(slave)
        assert path.startswith('/dev/pts/')
        assert_isolated_target(path)
    finally:
        os.close(master)
        os.close(slave)
