#!/usr/bin/env python3
"""串口日志采集 + 实时解析。

用法：
    sg dialout -c 'python3 tools/serial_log.py'
    sg dialout -c 'python3 tools/serial_log.py --port /dev/ttyACM0 --baud 115200 --out /tmp/log.txt'

为什么需要 sg dialout：/dev/ttyACM0 属于 root:dialout 660，
当前会话没继承 dialout 组，必须借 sg 换组才能读。

为什么这么设 termios：CH9102F 走 CDC-ACM，必须显式清掉流控（CLOCAL|CREAD）、
设 raw 模式，否则 read() 拿不到数据或全是乱码——实测踩过。
"""
import argparse
import os
import select
import sys
import termios
import time
import tty

BAUD = {
    9600: termios.B9600, 19200: termios.B19200, 38400: termios.B38400,
    57600: termios.B57600, 115200: termios.B115200, 230400: termios.B230400,
}


def open_port(port, baud):
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)                       # 必须先 setraw 再 tcgetattr
    a = termios.tcgetattr(fd)
    a[4] = a[5] = BAUD[baud]
    a[2] = (a[2] & ~termios.CSTOPB & ~termios.PARENB & ~termios.CSIZE) \
        | termios.CS8 | termios.CLOCAL | termios.CREAD
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200, choices=sorted(BAUD))
    ap.add_argument("--out", default=None, help="同时写入文件（便于事后 grep）")
    ap.add_argument("--seconds", type=float, default=0, help="0 = 一直跑到 Ctrl-C")
    args = ap.parse_args()

    fd = open_port(args.port, args.baud)
    fh = open(args.out, "a", encoding="utf-8", errors="replace") if args.out else None
    print(f"# {args.port} @ {args.baud} 8N1 raw  (Ctrl-C 退出)", file=sys.stderr)

    buf = b""
    t0 = time.time()
    try:
        while True:
            if args.seconds and (time.time() - t0) > args.seconds:
                break
            r, _, _ = select.select([fd], [], [], 0.3)
            if not r:
                continue
            try:
                data = os.read(fd, 8192)
            except BlockingIOError:
                continue
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", "replace").rstrip("\r")
                print(text)
                if fh:
                    fh.write(text + "\n")
                    fh.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if fh:
            fh.close()
        os.close(fd)


if __name__ == "__main__":
    main()
