#!/usr/bin/env python3
"""伪终端端到端脚本（任务 B）：桥 → /dev/pts/N → 固件解析器。

这是唯一真正"过一遍写系统调用"的验证：桥的写入端是伪终端主/从对的一侧，
读取端把收到的字节原样送给真实固件解析器。全程不碰任何真实 UART。

用法::

    /tmp/odom-taskb-venv/bin/python test/firmware_harness/pty_bridge_run.py

输出为可复制的文本日志；退出码 0 表示全部断言通过。
"""
from __future__ import annotations

import os
import pathlib
import select
import sys
import tty

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from fw_parser_harness import FirmwareModel                  # noqa: E402
from c50c_uart3.chassis_cmd import encode_speed_frame        # noqa: E402
from c50c_uart3.chassis_cmd.arbiter import Arbiter           # noqa: E402
from c50c_uart3.chassis_cmd.transport import assert_isolated_target  # noqa: E402

ZERO = encode_speed_frame(0.0, 0.0, 0.0)


class PtyTransport:
    """把帧写进伪终端主端；从端由另一个对象读取。"""

    kind = 'pty'

    def __init__(self, master_fd: int, slave_path: str) -> None:
        assert_isolated_target(slave_path)      # 只允许 /dev/pts/N
        self.master_fd = master_fd
        self.slave_path = slave_path
        self.writes: list[bytes] = []

    def write(self, data: bytes) -> bool:
        os.write(self.master_fd, data)
        self.writes.append(bytes(data))
        return True

    def reconnect(self) -> None:
        pass


class ArbiterAdapter:
    """适配 chain 期望的接口，但不经过 StraightCommandChain 的包装。

    直接用仲裁器，避免为了脚本再引入一层；`tick` 的行为与 chain 一致。
    """

    def __init__(self, arbiter: Arbiter, transport: PtyTransport) -> None:
        self.arbiter = arbiter
        self.transport = transport
        self.records = []

    def tick(self, now: float):
        d = self.arbiter.evaluate(now)
        ok = False
        if d.frame is not None:
            ok = self.transport.write(d.frame)
            self.arbiter.on_write_result(ok)
        self.records.append((now, d.frame, d.is_zero, d.zero_reason, ok))
        return d


def drain(reader_fd: int, timeout: float = 0.05) -> bytes:
    """取出接收端已到达的字节（阻塞读 + select 判就绪）。"""
    out = bytearray()
    while True:
        r, _, _ = select.select([reader_fd], [], [], timeout)
        if not r:
            return bytes(out)
        chunk = os.read(reader_fd, 4096)
        if not chunk:
            return bytes(out)
        out += chunk


def main() -> int:
    master, slave = os.openpty()
    slave_path = os.ttyname(slave)
    # 伪终端默认带行规程（ICRNL/ECHO/ONLCR 等），会把 0x0D/0x0A 等字节改写。
    # 真实串口是 8N1 raw，因此两端都必须置 raw，否则字节比对无意义。
    tty.setraw(master)
    # 接收端必须以 O_RDWR 打开从端：只以只读方式打开时，主端写入的字节
    # 不会被挂到读队列上（本机内核实测），读取会一直阻塞。
    reader = os.open(slave_path, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(reader)
    print(f'[pty] slave = {slave_path} (raw), reader fd = {reader}')
    assert_isolated_target(slave_path)

    fw = FirmwareModel.start()
    transport = PtyTransport(master, slave_path)
    arb = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5)
    chain = ArbiterAdapter(arb, transport)

    clock = [100.0]
    failures: list[str] = []

    def check(name: str, cond: bool, detail: str = '') -> None:
        print(f'[{"PASS" if cond else "FAIL"}] {name} {detail}')
        if not cond:
            failures.append(name)

    def step(dt: float = 0.02, ticks: int = 2):
        clock[0] += dt
        fw.tick(ticks)
        d = chain.tick(clock[0])
        received = drain(reader)
        if received:
            fw.feed_bytes(received)
        return d, received

    # 场景 1：首帧零速 → 授权 → 非零
    arb.on_publisher_count(1, 0)
    arb.on_firmware_confirm(True)
    # 第五轮：启动默认静默（接收端状态未知）。这一行是**测试构造的模拟
    # 干净状态**，不是生产确认；真实系统里没有这个入口。
    arb.mark_receiver_clean(simulated=True)
    arb.on_nav_cmd(0.05, 0.0, 0.0, clock[0])
    d1, rx1 = step(0.0, 0)
    check('场景1 首帧为零速', d1.is_zero, d1.frame.hex() if d1.frame else 'None')
    check('场景1 固件收到字节与桥一致', rx1 == d1.frame, rx1.hex())
    check('场景1 固件已授权', fw.state.armed)
    d2, rx2 = step()
    check('场景1 第二帧为非零', not d2.is_zero, d2.frame.hex() if d2.frame else 'None')
    check('场景1 固件解出 0.05 m/s', abs(fw.state.vx - 0.05) < 1e-6,
          f'vx={fw.state.vx:.4f}')

    # 场景 2：命令流中断 → 桥零速 → 固件停车
    clock[0] += 0.35
    d3 = chain.tick(clock[0])
    rx = drain(reader)
    fw.feed_bytes(rx)
    check('场景2 过期后输出零速', d3.is_zero, str(d3.zero_reason))
    check('场景2 固件目标速度归零', fw.state.vx == 0.0, f'vx={fw.state.vx}')

    # 场景 3：桥完全停发 → 固件自行 1 s 超时停车
    fw.tick(100)
    check('场景3 固件超时停车并解除授权', (not fw.state.armed) and fw.state.vx == 0.0,
          f'armed={fw.state.armed} epoch={fw.state.epoch}')

    # 场景 4：重连 → 桥只发零速
    arb.on_firmware_confirm(True)
    arb.on_serial_reconnect()
    nonzero_after = 0
    for _ in range(20):
        arb.on_nav_cmd(0.05, 0.0, 0.0, clock[0])
        d, rx = step()
        if not d.is_zero:
            nonzero_after += 1
    check('场景4 重连后 20 拍无非零', nonzero_after == 0, f'non0={nonzero_after}')
    check('场景4 固件保持零速', fw.state.vx == 0.0)

    # 场景 5：写入端拒绝真实设备
    from c50c_uart3.chassis_cmd.transport import RealDeviceAccessForbidden
    try:
        assert_isolated_target('/dev/ttyAMA0')
        check('场景5 真机路径被拒绝', False)
    except RealDeviceAccessForbidden:
        check('场景5 真机路径被拒绝', True)

    fw.close()
    os.close(reader)
    os.close(master)
    os.close(slave)
    print(f'[summary] failures={len(failures)}')
    return 0 if not failures else 1


if __name__ == '__main__':
    raise SystemExit(main())
