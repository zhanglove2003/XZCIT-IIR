#!/usr/bin/env python3
"""Decode C50C 24-byte ROS status frames; use only when ROS is not using UART3."""
import argparse
import datetime
import json
import os
import select
import sys
import termios
import time

FRAME_LEN = 24


class StatusFrameParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        out = []
        while self.buffer:
            try:
                start = self.buffer.index(0x7b)
            except ValueError:
                self.buffer.clear()
                break
            del self.buffer[:start]
            if len(self.buffer) < FRAME_LEN:
                break
            frame = self.buffer[:FRAME_LEN]
            checksum = 0
            for b in frame[:22]:
                checksum ^= b
            if frame[23] != 0x7d or frame[22] != checksum:
                del self.buffer[0]
                continue
            values = [int.from_bytes(frame[i:i+2], 'big', signed=True)
                      for i in range(2, 20, 2)]
            out.append({'stop': frame[1], 'velocity_mm_s': values[:2],
                        'yaw_mrad_s': values[2], 'accel_raw': values[3:6],
                        'gyro_raw': values[6:9],
                        'voltage_mv': int.from_bytes(frame[20:22], 'big')})
            del self.buffer[:FRAME_LEN]
        return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device', required=True)
    p.add_argument('--seconds', type=float, default=0, help='0 runs until Ctrl-C')
    p.add_argument('--log', help='append JSON lines to this file')
    args = p.parse_args()
    if args.seconds < 0:
        p.error('--seconds cannot be negative')
    fd = os.open(args.device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        a = termios.tcgetattr(fd)
        a[0] = a[1] = a[3] = 0
        a[2] = (a[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB |
                          getattr(termios, 'CRTSCTS', 0))) | termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        a[6][termios.VMIN] = a[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, a)
        termios.tcflush(fd, termios.TCIFLUSH)
        parser = StatusFrameParser()
        end = time.monotonic() + args.seconds if args.seconds else None
        f = open(args.log, 'a', encoding='utf-8', buffering=1) if args.log else None
        try:
            while end is None or time.monotonic() < end:
                if select.select([fd], [], [], .25)[0]:
                    chunk = os.read(fd, 1024)
                    if not chunk:
                        raise OSError('serial disconnected')
                    for item in parser.feed(chunk):
                        item['time'] = datetime.datetime.now().astimezone().isoformat(timespec='milliseconds')
                        line = json.dumps(item, ensure_ascii=True, separators=(',', ':'))
                        print(line, flush=True)
                        if f:
                            f.write(line + '\n')
        finally:
            if f:
                f.close()
    except KeyboardInterrupt:
        return 0
    finally:
        os.close(fd)
    return 0


if __name__ == '__main__':
    sys.exit(main())
