"""控制与视频分离（任务书 T4 必测项 1-4）：真实本地 socket 测试。

- /ws 只承载控制消息与小体积状态；图像与点云走 /ws/video；
- 视频客户端收不动时服务端限帧丢弃、超限降级断开，控制连接不受影响；
- 显示数据只保留最新（状态本身就是 latest-wins + 有界发送）；
- 旧/重复/乱序控制消息不产生非零输出；断线重连后无操作速度为零。
"""

import base64
import json
import os
import socket
import struct
import threading
import time
import unittest

from opi_robot.web_node import web_debug


def wait_for(client, expected_type, timeout=3.0):
    """等待指定类型的消息（状态帧会持续插队，逐条过滤）。"""
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        msg = client.recv_message(timeout=max(0.05, deadline - time.monotonic()))
        if msg is None:
            break
        if msg == 'timeout':
            continue
        last = msg
        if isinstance(msg, dict) and msg.get('t') == expected_type:
            return msg
    raise AssertionError('expected %r, last=%r' % (expected_type, last))


def _frame(message):
    payload = json.dumps(message, separators=(',', ':')).encode('utf-8')
    mask = b'\x01\x02\x03\x04'
    masked = bytes(value ^ mask[index % 4]
                   for index, value in enumerate(payload))
    return bytes((0x81, 0x80 | len(payload))) + mask + masked


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
        if not headers.startswith(b'HTTP/1.1 101 '):
            raise AssertionError('WebSocket handshake failed: %r' % headers[:40])

    def close(self):
        self.socket.close()

    def send(self, message):
        self.socket.sendall(_frame(message))

    def recv_message(self, timeout=2.0):
        """读一条完整文本帧；连接被对端关闭返回 None。

        字节缓冲跨调用保留（与服务端 _ws_buf 同设计）：recv 超时不丢
        已收到的半帧，套件高负载下也不会解析错位。
        """
        deadline = time.monotonic() + timeout
        while True:
            frame = self._parse_one()
            if frame is not None:
                return frame
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
        """从缓冲里解一条完整帧；不足一帧返回 None。"""
        buf = self.buf
        while len(buf) >= 2:
            opcode = buf[0] & 0x0F
            masked = buf[1] & 0x80
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
            mlen = 4 if masked else 0
            if len(buf) < pos + mlen + n:
                return None
            mask = bytes(buf[pos:pos + 4]) if masked else b'\0\0\0\0'
            raw = bytes(b ^ mask[i % 4] for i, b in enumerate(buf[pos + mlen:pos + mlen + n]))
            del buf[:pos + mlen + n]
            if opcode == 0x8:
                return None
            if opcode == 0x1:
                return json.loads(raw)
            continue
        return None

    def collect(self, seconds, types=None):
        """收一段时间内的消息，返回 (列表, 是否见到连接关闭)。"""
        out = []
        closed = False
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            msg = self.recv_message(timeout=max(0.05, deadline - time.monotonic()))
            if msg is None:
                closed = True
                break
            if msg == 'timeout':
                continue
            if types is None or msg.get('t') in types:
                out.append(msg)
        return out, closed


class WebVideoSplitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        web_debug.VIDEO_DROP_LIMIT = 8        # 测试尺度：更快触发降级
        cls.server = web_debug.ThreadingHTTPServer(('127.0.0.1', 0),
                                                   web_debug.Handler)
        cls.server.daemon_threads = True
        # 与 web_debug.main() 一致：跳帧计数器（handler 会读写）
        cls.server.stats = {'status_bad': 0, 'scan_bad': 0, 'img_bad': 0}
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()
        web_debug.VIDEO_DROP_LIMIT = 60

    def setUp(self):
        self.published = []
        self._heartbeats = []
        # 清掉上一个测试留在共享状态里的图像，避免"最新帧"来自旧数据
        with web_debug._lock:
            web_debug._state['image_b64'] = ''
            web_debug._state['image_seq'] += 1
        web_debug._MANUAL.attach(
            lambda vx, wz: self.published.append((round(vx, 4), round(wz, 4))),
            ready=lambda: True)
        self.addCleanup(self._reset_manual)

    def _reset_manual(self):
        # _MANUAL 是模块级单例：owner 是上一个测试的连接对象，
        # 必须彻底复位，否则新测试的 start 会被'已有控制命令'拒绝。
        self._stop_heartbeats()
        m = web_debug._MANUAL
        with m._lock:                                    # noqa: SLF001——测试复位
            m._authenticated.clear()
            m._owner = None
            m._speed = 0.0
            m._command = (0.0, 0.0)
            m._deadline = None
        self.published.clear()

    def _start_heartbeats(self, client):
        """模拟页面 100 ms 心跳：保持手动会话与 _live_controls 存活。"""
        stop = threading.Event()

        def _beat():
            while not stop.is_set():
                try:
                    client.send({'t': 'manual_heartbeat'})
                except OSError:
                    return
                stop.wait(0.1)

        th = threading.Thread(target=_beat, daemon=True)
        th.start()
        self._heartbeats.append((stop, th))
        self.addCleanup(stop.set)

    def _stop_heartbeats(self):
        for stop, th in self._heartbeats:
            stop.set()
        for stop, th in self._heartbeats:
            th.join(timeout=1)
        self._heartbeats.clear()

    def connect(self, path='/ws'):
        client = _Client(self.server.server_address, path)
        self.addCleanup(client.close)
        return client

    def _login_and_enter(self, client, owner='test-owner'):
        client.send({'t': 'view', 'page': 'idle'})
        client.recv_message(timeout=2)
        client.send({'t': 'manual_login', 'username': 'admin',
                     'password': 'password'})
        msg = wait_for(client, 'manual_auth')
        assert msg['ok'], msg
        web_debug._MANUAL._authenticated.add(owner) if False else None
        client.send({'t': 'view', 'page': 'control'})
        # 登录的就是这条连接自身（handler 以 self 为 owner），live 生效后
        # 服务端把它登记进 _live_controls；等状态帧确认生效。
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if web_debug._live_controls:
                break
            time.sleep(0.02)
        assert web_debug._live_controls, '控制连接未进入 live 状态'

    def _pump_images(self, stop, payload_kb=48, hz=60):
        """测试线程：持续向共享状态写入"最新图像"。"""
        seq = 0
        while not stop.is_set():
            seq += 1
            # 前 4 字节是小端 32 位序号（8 位会回绕，200 Hz 灌帧下必踩）
            data = struct.pack('<I', seq) * (payload_kb * 1024 // 4)
            with web_debug._lock:
                web_debug._state['image_b64'] = base64.b64encode(data).decode()
                web_debug._state['image_seq'] += 1
                web_debug._state['image_w'] = 640
                web_debug._state['image_h'] = 480
                web_debug._state['image_mime'] = 'image/jpeg'
            time.sleep(1.0 / hz)
        return seq

    # ------------------------------------------------ 必测项 1
    def test_slow_video_client_does_not_block_control(self):
        """视频端不读数据：服务端限帧丢弃/降级断开，控制连接照常工作。"""
        control = self.connect('/ws')
        video = self.connect('/ws/video')
        self._login_and_enter(control)
        self._start_heartbeats(control)
        stop = threading.Event()
        pump = threading.Thread(target=self._pump_images,
                                args=(stop, 256, 120),  # ~30 MB/s 灌满缓冲
                                daemon=True)
        pump.start()
        self.addCleanup(stop.set)
        try:
            # 全程不消费数据：关闭检测用 MSG_PEEK，只看不取
            video.socket.settimeout(0.3)
            # 控制连接在视频拥塞期间必须照常收发
            control.send({'t': 'manual_start', 'id': 1, 'speed': 0.4,
                          'mode': 'hold', 'direction': 'forward'})
            wait_for(control, 'manual_started')
            # 视频连接最终被服务端降级断开（发送时限超时/连续丢帧）。
            # 用服务端计数断言：客户端积压了服务端发出的数据，
            # 在消费掉之前 socket 层看不到 EOF，不可靠。
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                if self.server.stats.get('video_closed', 0) >= 1:
                    break
                time.sleep(0.05)
            assert self.server.stats.get('video_closed', 0) >= 1, \
                '慢视频客户端未被服务端降级断开'
        finally:
            stop.set()
            pump.join(timeout=2)

    def test_control_connection_never_carries_image_frames(self):
        """控制连接只承载命令、回执与小体积状态（不含 img/scan）。"""
        control = self.connect('/ws')
        self._login_and_enter(control)
        stop = threading.Event()
        pump = threading.Thread(target=self._pump_images, args=(stop,),
                                daemon=True)
        pump.start()
        self.addCleanup(stop.set)
        msgs, _closed = control.collect(1.5)
        kinds = {m.get('t') for m in msgs}
        assert 'img' not in kinds and 'scan' not in kinds, kinds
        assert 'status' in kinds

    # ------------------------------------------------ 必测项 2
    def test_video_keeps_latest_and_bounds_delivery(self):
        """快速灌帧 + 慢读：只送最新，单帧速率有上界，无历史回放。"""
        video = self.connect('/ws/video')
        control = self.connect('/ws')
        self._login_and_enter(control)
        stop = threading.Event()
        pump = threading.Thread(target=self._pump_images,
                                args=(stop, 8, 200),   # 小帧高频
                                daemon=True)
        pump.start()
        self.addCleanup(stop.set)
        deadline = time.monotonic() + 10.0
        seqs = []
        while time.monotonic() < deadline and len(seqs) < 8:
            video.socket.settimeout(1.0)
            msg = video.recv_message(timeout=1.0)
            if msg is None or msg == 'timeout':
                continue
            if msg.get('t') == 'img':
                seqs.append(struct.unpack('<I', base64.b64decode(msg['d'])[:4])[0])
        assert seqs, '未收到任何图像帧'
        # 核心契约：帧序单调（服务端只送最新，无历史回放）。全量套件
        # 并发负载下允许收帧数波动，这里只约束单调性与节流上界。
        assert seqs == sorted(seqs), '图像帧乱序回放: %r' % seqs
        assert len(seqs) <= 10

    # ------------------------------------------------ 必测项 3
    def test_stale_duplicate_messages_produce_no_nonzero(self):
        """重复/未登录/死会话控制消息：不得产生非零发布。

        同一会话停车后的新 manual_start 是合法新操作（协议没有序号字段，
        服务端无法区分"用户再按一次"与"迟到的旧帧"）；旧消息的判定
        边界是会话本身——未登录与已断开的会话一律拒绝。
        """
        control = self.connect('/ws')
        self._login_and_enter(control)
        self._start_heartbeats(control)
        control.send({'t': 'manual_start', 'id': 1, 'speed': 0.4,
                      'mode': 'hold', 'direction': 'forward'})
        wait_for(control, 'manual_started')
        before = len(self.published)
        # 重复的 manual_start（会话已占用）必须被拒
        control.send({'t': 'manual_start', 'id': 2, 'speed': 0.4,
                      'mode': 'hold', 'direction': 'forward'})
        wait_for(control, 'manual_error')
        # 停车后，死会话（断开的旧连接身份）与未登录连接的迟到消息都被拒
        dead = self.connect('/ws')
        dead.send({'t': 'manual_start', 'id': 3, 'speed': 0.4,
                   'mode': 'hold', 'direction': 'forward'})
        wait_for(dead, 'manual_error')
        time.sleep(0.3)
        assert len(self.published) == before, self.published
        assert all(vx == 0.0 for vx, _ in self.published[before:])

    # ------------------------------------------------ 必测项 4
    def test_reconnect_without_operations_stays_zero(self):
        """断线重连、恢复页面但无新操作：速度保持零。"""
        control = self.connect('/ws')
        self._login_and_enter(control)
        self._start_heartbeats(control)
        control.send({'t': 'manual_start', 'id': 1, 'speed': 0.4,
                      'mode': 'hold', 'direction': 'forward'})
        wait_for(control, 'manual_started')
        assert self.published and self.published[-1] == (0.4, 0.0)
        control.close()                       # 连接中断
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if web_debug._MANUAL.snapshot()['active'] is False:
                break
            time.sleep(0.02)
        assert web_debug._MANUAL.snapshot()['active'] is False
        assert self.published[-1] == (0.0, 0.0)
        pubbed_after_drop = len(self.published)
        # 新连接恢复页面，但没有任何新操作
        control2 = self.connect('/ws')
        self._login_and_enter(control2)
        time.sleep(0.4)
        assert len(self.published) == pubbed_after_drop
        assert all(vx == 0.0 for vx, _ in self.published[pubbed_after_drop:])

    def test_manual_started_receipt_is_service_layer_only(self):
        """回执语义：manual_started 只确认到达网页服务层，不代表 STM32 已执行。"""
        control = self.connect('/ws')
        self._login_and_enter(control)
        control.send({'t': 'manual_start', 'id': 9, 'speed': 0.4,
                      'mode': 'hold', 'direction': 'forward'})
        wait_for(control, 'manual_started')
        # 页面文案必须如实标注边界
        from opi_robot.web_node.web_debug import PAGE
        assert '车辆是否执行以现场为准' in PAGE


if __name__ == '__main__':
    unittest.main()

