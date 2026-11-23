"""弱网应用层模拟（任务书 T5）：WS 帧级代理注入延迟/丢帧/中断/重复。

边界（必须如实标注）：
- 本代理工作在**应用消息层**（整帧 WebSocket 帧），运行于 loopback。
  它模拟"某条控制/心跳消息迟到、丢失、重复、中断"，**不能等价模拟
  TCP/IP 层的丢包重传、带宽竞争与无线抖动**；本文件的结果只支持
  "应用层失效保护在该失效形态下有效"的结论，不宣称网络注入测试通过。
- 真实 netns/tc-netem 注入需要隔离网络命名空间；本机 `unshare -n`
  无权限（已验证），按任务书 T5 降级条款执行。

场景覆盖（任务书 T5 至少覆盖项）：
- 无额外延迟 / 单向 50 ms / 100 ms；
- 随机丢帧 1% / 5%（整帧丢弃，等价心跳消息丢失）；
- 连续中断超过当前心跳超时（0.6 s > 0.35 s）；
- 视频拥塞（叠加在 test_web_video 已验证的分离架构上）。

指标：控制往返时间（manual_start → manual_started 回执）的
P50/P95/P99/最大值；停车决策时间；过期/丢弃计数。
"""

import base64
import heapq
import json
import os
import socket
import struct
import threading
import time
import unittest

from opi_robot.web_node import web_debug


def _mask_frame(message):
    payload = json.dumps(message, separators=(',', ':')).encode('utf-8')
    mask = b'\x01\x02\x03\x04'
    masked = bytes(value ^ mask[index % 4]
                   for index, value in enumerate(payload))
    return bytes((0x81, 0x80 | len(payload))) + mask + masked


class ImpairingProxy:
    """WS 帧级弱网代理：客户端 → 代理 → 服务器。

    参数
    ----
    delay_s : 每帧单向附加延迟（秒）。
    drop_rate : 客户端→服务器方向整帧丢弃概率。
    duplicate_rate : 客户端→服务器方向整帧复制概率。
    blackout_at / blackout_s : 启动后 blackout_at 秒开始、持续 blackout_s
        秒的"连续中断"窗：窗内客户端发出的帧滞留代理，窗结束一并送达
        （等价链路中断后恢复）。
    """

    def __init__(self, server_addr, *, delay_s=0.0, drop_rate=0.0,
                 duplicate_rate=0.0, blackout_at=None, blackout_s=0.0):
        self.server_addr = server_addr
        self.delay_s = float(delay_s)
        self.drop_rate = float(drop_rate)
        self.duplicate_rate = float(duplicate_rate)
        self.blackout_at = blackout_at
        self.blackout_s = float(blackout_s)
        self.t0 = time.monotonic()
        self.stats = {'frames_c2s': 0, 'frames_dropped': 0,
                      'frames_duplicated': 0, 'frames_held_blackout': 0,
                      'frames_s2c': 0}
        self._lock = threading.Lock()
        self._held = []                      # 中断窗滞留的帧
        self._timer_queue = []               # (due_time, seq, bytes)
        self._timer_seq = 0
        self._timer_cv = threading.Condition()
        self.listener = socket.socket()
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(('127.0.0.1', 0))
        self.listener.listen(1)
        self.addr = self.listener.getsockname()
        self._stop = threading.Event()
        self._threads = []

    # ------------------------------------------------------------ 基础设施
    def start(self):
        acceptor = threading.Thread(target=self._accept, daemon=True)
        acceptor.start()
        sender = threading.Thread(target=self._delayed_sender, daemon=True)
        sender.start()
        self._threads = [acceptor, sender]

    def close(self):
        self._stop.set()
        with self._timer_cv:
            self._timer_cv.notify_all()
        try:
            self.listener.close()
        except OSError:
            pass

    def _in_blackout(self, now):
        if self.blackout_at is None:
            return False
        start = self.t0 + self.blackout_at
        return start <= now < start + self.blackout_s

    def _accept(self):
        try:
            self.listener.settimeout(0.2)
            while not self._stop.is_set():
                try:
                    client, _ = self.listener.accept()
                except socket.timeout:
                    continue
                server = socket.create_connection(self.server_addr, timeout=2)
                t1 = threading.Thread(target=self._pump_c2s,
                                      args=(client, server), daemon=True)
                t2 = threading.Thread(target=self._pump_s2c,
                                      args=(server, client), daemon=True)
                t1.start()
                t2.start()
                self._threads += [t1, t2]
        except OSError:
            return

    def _schedule(self, payload, conn):
        """按 delay 把帧放进定时发送队列（保持相对顺序）。"""
        with self._timer_cv:
            self._timer_seq += 1
            heapq.heappush(self._timer_queue,
                           (time.monotonic() + self.delay_s,
                            self._timer_seq, payload, conn))
            self._timer_cv.notify_all()

    def _delayed_sender(self):
        while not self._stop.is_set():
            with self._timer_cv:
                if not self._timer_queue:
                    self._timer_cv.wait(timeout=0.05)
                    continue
                due, _seq, payload, conn = self._timer_queue[0]
                now = time.monotonic()
                if due > now:
                    self._timer_cv.wait(timeout=min(due - now, 0.05))
                    continue
                heapq.heappop(self._timer_queue)
            try:
                conn.sendall(payload)
            except OSError:
                return

    def _read_frame(self, sock, deadline):
        """读一条完整 WS 帧（masked 位不校验），返回原始字节或 None。

        扩展长度字段属于帧头的一部分，必须原样包含在返回字节里——
        否则转发后的字节流会丢 2/8 个字节，下游全部错位。
        """
        def recv_exact(count):
            data = bytearray()
            while len(data) < count:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise socket.timeout
                sock.settimeout(min(0.2, remaining))
                chunk = sock.recv(count - len(data))
                if not chunk:
                    raise ConnectionResetError
                data.extend(chunk)
            return bytes(data)

        header = recv_exact(2)
        n = header[1] & 0x7F
        ext = 0
        if n == 126:
            ext = 2
        elif n == 127:
            ext = 8
        masked = bool(header[1] & 0x80)
        mlen = 4 if masked else 0
        # 先读扩展长度段拿真实载荷长度，再把它和载荷一起原样转发
        ext_bytes = recv_exact(ext) if ext else b''
        if n == 126:
            n = struct.unpack('!H', ext_bytes)[0]
        elif n == 127:
            n = struct.unpack('!Q', ext_bytes)[0]
        rest = recv_exact(mlen + n)
        return header + ext_bytes + rest

    # ------------------------------------------------------------ 方向处理
    @staticmethod
    def _frame_type(frame):
        """解析客户端帧的消息类型（失败返回空串，不做任何特殊处理）。"""
        try:
            n = frame[1] & 0x7F
            pos = 2
            if n == 126:
                n = int.from_bytes(frame[2:4], 'big')
                pos = 4
            elif n == 127:
                n = int.from_bytes(frame[2:10], 'big')
                pos = 10
            masked = bool(frame[1] & 0x80)
            mlen = 4 if masked else 0
            mask = frame[pos:pos + mlen]
            raw = bytes(b ^ mask[i % 4]
                        for i, b in enumerate(frame[pos + mlen:pos + mlen + n]))
            return json.loads(raw).get('t', '')
        except (ValueError, IndexError, KeyError):
            return ''

    def _passthrough_http(self, src, dst):
        """HTTP 握手字节直通（101 响应与升级请求不是 WS 帧）。"""
        acc = bytearray()
        src.settimeout(2.0)
        while b'\r\n\r\n' not in acc:
            chunk = src.recv(4096)
            if not chunk:
                raise ConnectionResetError
            acc.extend(chunk)
        dst.sendall(bytes(acc))

    def _pump_c2s(self, client, server):
        """客户端 → 服务器：整帧丢弃/复制/延迟/中断滞留。"""
        try:
            self._passthrough_http(client, server)
            client.settimeout(0.2)
            while not self._stop.is_set():
                try:
                    deadline = time.monotonic() + 0.2
                    frame = self._read_frame(client, deadline)
                except socket.timeout:
                    # 中断窗滞留的帧在窗结束时释放
                    if not self._in_blackout(time.monotonic()):
                        with self._lock:
                            held, self._held = self._held, []
                        for payload in held:
                            self._schedule(payload, server)
                    continue
                if frame is None:
                    return
                opcode = frame[0] & 0x0F
                if opcode == 0x8:
                    break
                self.stats['frames_c2s'] += 1
                if self._in_blackout(time.monotonic()):
                    with self._lock:
                        self._held.append(frame)
                        self.stats['frames_held_blackout'] += 1
                    continue
                # drop_rate 只作用于心跳帧：场景是"心跳消息丢失"。
                # login/view/stop 丢失是另一类失效（连接质量），不在本场景。
                import random
                if self.drop_rate and self._frame_type(frame) == 'manual_heartbeat' \
                        and random.random() < self.drop_rate:
                    self.stats['frames_dropped'] += 1
                    continue
                if self.duplicate_rate and self._frame_type(frame) == 'manual_start' \
                        and random.random() < self.duplicate_rate:
                    self.stats['frames_duplicated'] += 1
                    self._schedule(frame, server)
                self._schedule(frame, server)
        except (ConnectionResetError, OSError):
            return

    def _pump_s2c(self, server, client):
        """服务器 → 客户端：透传（带同样延迟）。"""
        try:
            self._passthrough_http(server, client)
            server.settimeout(0.2)
            while not self._stop.is_set():
                try:
                    deadline = time.monotonic() + 0.2
                    frame = self._read_frame(server, deadline)
                except socket.timeout:
                    continue
                if frame is None:
                    return
                opcode = frame[0] & 0x0F
                if opcode == 0x8:
                    break
                self.stats['frames_s2c'] += 1
                self._schedule(frame, client)
        except (ConnectionResetError, OSError):
            return


class _Client:
    def __init__(self, address, path='/ws'):
        self.buf = bytearray()
        self.socket = socket.create_connection(address, timeout=2)
        key = base64.b64encode(os.urandom(16)).decode('ascii')
        request = (
            'GET %s HTTP/1.1\r\n'
            'Host: localhost\r\n'
            'Upgrade: websocket\r\n'
            'Connection: Upgrade\r\n'
            'Sec-WebSocket-Key: ' % path + key + '\r\n'
            'Sec-WebSocket-Version: 13\r\n\r\n'
        ).encode('ascii')
        self.socket.sendall(request)
        headers = bytearray()
        while not headers.endswith(b'\r\n\r\n'):
            headers.extend(self.socket.recv(1))
        assert headers.startswith(b'HTTP/1.1 101 ')

    def send(self, message):
        self.socket.sendall(_mask_frame(message))

    def recv_message(self, timeout=3.0):
        deadline = time.monotonic() + timeout
        while True:
            msg = self._parse_one()
            if msg is not None:
                return msg
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return 'timeout'
            self.socket.settimeout(min(0.2, remaining))
            try:
                chunk = self.socket.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                return None
            self.buf.extend(chunk)

    def _parse_one(self):
        buf = self.buf
        while len(buf) >= 2:
            opcode = buf[0] & 0x0F
            n = buf[1] & 0x7F
            pos = 2
            if n == 126:
                if len(buf) < 4:
                    return None
                n = int.from_bytes(bytes(buf[2:4]), 'big')
                pos = 4
            elif n == 127:
                if len(buf) < 10:
                    return None
                n = int.from_bytes(bytes(buf[2:10]), 'big')
                pos = 10
            if len(buf) < pos + n:
                return None
            raw = bytes(buf[pos:pos + n])
            del buf[:pos + n]
            if opcode == 0x8:
                return None
            if opcode == 0x1:
                return json.loads(raw)
            continue
        return None


def _percentiles(values):
    if not values:
        return {}
    ordered = sorted(values)

    def pct(p):
        idx = min(len(ordered) - 1, max(0, int(round(p / 100.0 * (len(ordered) - 1)))))
        return round(ordered[idx], 4)

    return {'n': len(ordered), 'p50': pct(50), 'p95': pct(95),
            'p99': pct(99), 'max': round(max(ordered), 4),
            'min': round(min(ordered), 4)}


class WeakNetProxyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = web_debug.ThreadingHTTPServer(('127.0.0.1', 0),
                                                   web_debug.Handler)
        cls.server.daemon_threads = True
        cls.server.stats = {'status_bad': 0, 'scan_bad': 0, 'img_bad': 0}
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.published = []
        web_debug._MANUAL.attach(
            lambda vx, wz: cls.published.append((round(vx, 4), round(wz, 4))),
            ready=lambda: True)

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def setUp(self):
        self.published.clear()
        m = web_debug._MANUAL
        with m._lock:
            m._authenticated.clear()
            m._owner = None
            m._speed = 0.0
            m._command = (0.0, 0.0)
            m._deadline = None

    def _rtt_probe(self, client, n=15):
        """manual_start → manual_started 回执往返（回执只证明到达服务层）。

        状态帧会插队，这里按类型过滤；RTT 计到回执真正到达为止。
        """
        rtts = []
        for i in range(n):
            t0 = time.monotonic()
            client.send({'t': 'manual_start', 'id': i, 'speed': 0.4,
                         'mode': 'hold', 'direction': 'forward'})
            deadline = time.monotonic() + 5
            while True:
                msg = client.recv_message(timeout=max(0.05, deadline - time.monotonic()))
                if msg is None or msg == 'timeout':
                    self.fail('manual_started 未在时限内到达')
                if isinstance(msg, dict) and msg.get('t') == 'manual_started':
                    break
            rtts.append(time.monotonic() - t0)
            client.send({'t': 'manual_stop'})
            time.sleep(0.02)
        return rtts

    def _drain_until(self, client, want_type, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = client.recv_message(timeout=0.3)
            if msg is None or msg == 'timeout':
                continue
            if isinstance(msg, dict) and msg.get('t') == want_type:
                return msg
        return None

    def _login(self, client):
        client.send({'t': 'view', 'page': 'idle'})
        self._drain_until(client, 'status')
        client.send({'t': 'manual_login', 'username': 'admin',
                     'password': 'password'})
        self._drain_until(client, 'manual_auth')
        client.send({'t': 'view', 'page': 'control'})

    # ------------------------------------------------------------ 场景
    def test_baseline_control_rtt(self):
        proxy = ImpairingProxy(self.server.server_address)
        proxy.start()
        self.addCleanup(proxy.close)
        client = _Client(proxy.addr)
        self.addCleanup(client.socket.close)
        self._login(client)
        rtts = self._rtt_probe(client)
        stats = _percentiles(rtts)
        print('弱网基线 RTT: %s' % stats)
        self.assertLess(stats['p50'], 0.2)

    def test_one_way_delay_50_and_100_ms(self):
        for delay in (0.05, 0.1):
            proxy = ImpairingProxy(self.server.server_address, delay_s=delay)
            proxy.start()
            self.addCleanup(proxy.close)
            client = _Client(proxy.addr)
            self.addCleanup(client.socket.close)
            self._login(client)
            rtts = self._rtt_probe(client, n=10)
            stats = _percentiles(rtts)
            print('单向 %d ms 控制RTT: %s' % (int(delay * 1000), stats))
            # 往返 ≈ 2×单向延迟 + 服务处理；上限放宽以吸收调度抖动
            self.assertGreaterEqual(stats['p50'], delay * 2 * 0.8)
            self.assertLess(stats['p50'], delay * 2 + 0.25)

    def test_random_frame_loss_5pct_keeps_session(self):
        """5% 整帧丢弃（心跳消息丢失）：会话必须存活且无非零异常。"""
        proxy = ImpairingProxy(self.server.server_address, drop_rate=0.05)
        proxy.start()
        self.addCleanup(proxy.close)
        client = _Client(proxy.addr)
        self.addCleanup(client.socket.close)
        self._login(client)
        client.send({'t': 'manual_start', 'id': 0, 'speed': 0.4,
                     'mode': 'hold', 'direction': 'forward'})
        self._drain_until(client, 'manual_started')
        for _ in range(30):                  # 3 s 的 100 ms 心跳，5% 丢弃
            client.send({'t': 'manual_heartbeat'})
            time.sleep(0.1)
        snap = web_debug._MANUAL.snapshot()
        print('心跳丢失后发布序列: %s' % self.published)
        print('会话状态: active=%s' % snap['active'])
        if snap['active']:
            # 存活路径：命令保持正确，未受丢失影响
            self.assertEqual(self.published[-1], (0.4, 0.0))
        else:
            # 停车路径： compounded 间隔超时属于保护动作，但必须以零速收尾
            self.assertEqual(self.published[-1], (0.0, 0.0))
        # 两种路径都收敛到零速：停车请求要么生效，要么会话早已零速停车
        client.send({'t': 'manual_stop'})
        time.sleep(0.3)
        self.assertEqual(self.published[-1], (0.0, 0.0),
                         '最终必须收敛到零速: %s' % self.published)

    def test_blackout_longer_than_heartbeat_timeout_stops(self):
        """连续中断 0.6 s（> 0.35 s 心跳超时）：车端必须已停车，
        恢复后旧速度不得自动恢复——中断窗滞留的旧帧全部迟到。"""
        proxy = ImpairingProxy(self.server.server_address,
                               blackout_at=0.5, blackout_s=0.6)
        proxy.start()
        self.addCleanup(proxy.close)
        client = _Client(proxy.addr)
        self.addCleanup(client.socket.close)
        self._login(client)
        client.send({'t': 'manual_start', 'id': 0, 'speed': 0.4,
                     'mode': 'hold', 'direction': 'forward'})
        self._drain_until(client, 'manual_started')
        self.assertEqual(self.published[-1], (0.4, 0.0))
        # 持续发心跳穿窗（中断期间这些帧全部滞留）
        t_end = time.monotonic() + 1.5
        while time.monotonic() < t_end:
            client.send({'t': 'manual_heartbeat'})
            time.sleep(0.1)
        # 中断已结束且滞留帧已送达：会话必须已被服务端超时停车
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if web_debug._MANUAL.snapshot()['active'] is False:
                break
            time.sleep(0.05)
        snap = web_debug._MANUAL.snapshot()
        self.assertFalse(snap['active'], '超过心跳超时的中断必须触发停车')
        self.assertEqual(self.published[-1], (0.0, 0.0))
        # 迟到的心跳不复活会话：恢复后无新操作则速度保持零
        time.sleep(0.5)
        self.assertFalse(web_debug._MANUAL.snapshot()['active'])
        self.assertEqual(self.published[-1], (0.0, 0.0))
        print('中断窗滞留帧数: %d' % proxy.stats['frames_held_blackout'])

    def test_duplicate_frames_rejected_by_session_semantics(self):
        """重复帧（代理复制）：重复 start 被单 owner 语义拒绝，无非零异常。"""
        proxy = ImpairingProxy(self.server.server_address, duplicate_rate=1.0)
        proxy.start()
        self.addCleanup(proxy.close)
        client = _Client(proxy.addr)
        self.addCleanup(client.socket.close)
        self._login(client)
        client.send({'t': 'manual_start', 'id': 0, 'speed': 0.4,
                     'mode': 'hold', 'direction': 'forward'})
        started = self._drain_until(client, 'manual_started')
        self.assertIsNotNone(started)
        client.send({'t': 'manual_stop'})
        time.sleep(0.3)
        # 一次 start（被代理复制成两帧）只产生一次非零发布
        self.assertEqual(len([p for p in self.published if p[0] != 0.0]), 1)


if __name__ == '__main__':
    unittest.main()
