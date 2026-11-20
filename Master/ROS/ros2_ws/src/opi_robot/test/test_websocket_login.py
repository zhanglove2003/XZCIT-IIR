"""Regression tests for WebSocket login using real local sockets."""

import base64
import json
import os
import socket
import struct
import threading
import time
import unittest

from opi_robot.web_node import web_debug


def _frame(message):
    payload = json.dumps(message, separators=(',', ':')).encode('utf-8')
    mask = b'\x01\x02\x03\x04'
    masked = bytes(value ^ mask[index % 4]
                   for index, value in enumerate(payload))
    return bytes((0x81, 0x80 | len(payload))) + mask + masked


class _Client:
    def __init__(self, address, first_message=None):
        self.socket = socket.create_connection(address, timeout=2)
        key = base64.b64encode(os.urandom(16)).decode('ascii')
        request = (
            'GET /ws HTTP/1.1\r\n'
            'Host: localhost\r\n'
            'Upgrade: websocket\r\n'
            'Connection: Upgrade\r\n'
            'Sec-WebSocket-Key: ' + key + '\r\n'
            'Sec-WebSocket-Version: 13\r\n\r\n'
        ).encode('ascii')
        if first_message is not None:
            request += _frame(first_message)
        self.socket.sendall(request)
        headers = bytearray()
        while not headers.endswith(b'\r\n\r\n'):
            headers.extend(self.socket.recv(1))
        if not headers.startswith(b'HTTP/1.1 101 '):
            raise AssertionError('WebSocket handshake failed')

    def close(self):
        self.socket.close()

    def send(self, message):
        self.socket.sendall(_frame(message))

    def _read_exactly(self, count, deadline):
        data = bytearray()
        while len(data) < count:
            self.socket.settimeout(max(0.001, deadline - time.monotonic()))
            chunk = self.socket.recv(count - len(data))
            if not chunk:
                raise AssertionError('WebSocket closed before the expected message')
            data.extend(chunk)
        return bytes(data)

    def receive(self, expected_type, timeout=2):
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                header = self._read_exactly(2, deadline)
                length = header[1] & 0x7f
                if length == 126:
                    length = struct.unpack('!H', self._read_exactly(2, deadline))[0]
                elif length == 127:
                    length = struct.unpack('!Q', self._read_exactly(8, deadline))[0]
                payload = self._read_exactly(length, deadline)
                if header[0] & 0x0f == 1:
                    message = json.loads(payload)
                    if message.get('t') == expected_type:
                        return message
        except socket.timeout:
            pass
        raise AssertionError(f'No {expected_type} message arrived within {timeout} s')


class WebSocketLoginTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = web_debug.ThreadingHTTPServer(('127.0.0.1', 0), web_debug.Handler)
        cls.server.daemon_threads = True
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def connect(self, first_message=None):
        client = _Client(self.server.server_address, first_message)
        self.addCleanup(client.close)
        return client

    def test_status_arrives_before_any_client_message(self):
        client = self.connect()
        self.assertEqual(client.receive('status')['t'], 'status')

    def test_login_receives_response_without_extra_client_messages(self):
        client = self.connect()
        client.send({'t': 'view', 'page': 'idle'})
        client.receive('status')
        client.send({'t': 'manual_login', 'username': 'admin', 'password': 'password'})
        self.assertTrue(client.receive('manual_auth')['ok'])
        self.assertEqual(client.receive('status')['t'], 'status')

    def test_frame_sent_with_handshake_is_processed(self):
        client = self.connect({'t': 'manual_login', 'username': 'admin',
                               'password': 'password'})
        self.assertTrue(client.receive('manual_auth')['ok'])

    def test_split_login_frame_does_not_interrupt_status(self):
        client = self.connect()
        client.receive('status')
        frame = _frame({'t': 'manual_login', 'username': 'admin',
                        'password': 'password'})
        client.socket.sendall(frame[:5])
        client.receive('status')
        client.socket.sendall(frame[5:])
        self.assertTrue(client.receive('manual_auth')['ok'])


if __name__ == '__main__':
    unittest.main()
