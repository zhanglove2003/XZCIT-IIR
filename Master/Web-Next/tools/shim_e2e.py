#!/usr/bin/env python3
"""本地端到端模拟器：在 Web-Next（ros 模式）之上模拟垫片全链路。

stdlib-only 单文件。流程（对应交付要求）：
 1. 启动 Web-Next（ros 模式，OPI_WEB_PASSWORD_SHA256=sha256("admin")、
    OPI_BRIDGE_TOKEN 固定 32 hex）；
 2. POST 数帧底盘/电池（电压）/雷达/节点状态到 /bridge/frames；
 3. 按 RFC6455 手写 WS 客户端连 /ws：收 motion_license_session →
    manual_login（admin/admin）→ view=control → manual_start；
 4. 轮询 /bridge/commands 断言收到 Twist 命令（manual_cmd）；
 5. manual_stop 后断言 manual_zero（tick 侧对账派发）；
 6. GET /api/state 断言 chassis/battery/nodes 非空。

全程打印一行 SIM_RESULT=status_ok,command_ok 或 SIM_RESULT=fail:<原因>，
退出码随之（0 / 1）。

用法：python3 tools/shim_e2e.py [--keep]    # --keep 保留服务进程输出
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import signal
import socket
import struct
import re
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

HOST = "127.0.0.1"
# 端口随机（start_server 向内核要空闲端口）：避免与遗留进程/其他会话的
# 固定端口冲突——之前调试中出现过「旧进程仍占固定端口，新进程 bind 失败、
# e2e 与旧进程对话」的假象，改用临时端口后该类污染在结构上不可能发生。
_actual_port = None
TOKEN = "e2e" + "0" * 29  # 32 字符固定令牌（仅本地回环测试）
PASSWORD_SHA256 = "8c6976e5b5410415bde908bd4dee15dfb167a9c873fc4bb8a81f6f2ab448a918"  # admin
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def fail(reason: str) -> int:
    print("SIM_RESULT=fail:%s" % reason)
    return 1


# ---------------------------------------------------------------------------
# Web-Next 进程管理
# ---------------------------------------------------------------------------

def ensure_binary() -> str:
    env_bin = os.environ.get("WEB_NEXT_BIN")
    if env_bin and os.path.exists(env_bin):
        return env_bin
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    local = os.path.join(here, "target", "debug", "web_next")
    if os.path.exists(local):
        return local
    print("SIM-STEP cargo build（无现成二进制）")
    subprocess.run(["cargo", "build"], cwd=here, check=True, timeout=600)
    if os.path.exists(local):
        return local
    raise FileNotFoundError("web_next 二进制未找到且 cargo build 未产出")


def free_port() -> int:
    """向内核要一个空闲 TCP 端口（绑定 0 后关闭）。

    关闭到服务真正 bind 之间存在极小的被抢占窗口，但本机 e2e 环境可控，
    且远比固定端口可靠——固定端口曾被遗留进程占用，导致 e2e 与旧进程
    对话、读到旧 hub 状态的假象。"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sk:
        sk.bind(("127.0.0.1", 0))
        return sk.getsockname()[1]


def actual_port() -> int:
    global _actual_port
    if _actual_port is None:
        raise RuntimeError("服务未启动")
    return _actual_port


def probe_serving(host: str, port: int, timeout: float = 1.5) -> bool:
    """host:port 是否已有 Web-Next 在服 /api/state（门禁可能预启动服务）。"""
    try:
        with urllib.request.urlopen(
                "http://%s:%d/api/state" % (host, port), timeout=timeout) as resp:
            return resp.status == 200
    except OSError:
        return False


def start_server(binary: str, log_path: str):
    global _actual_port
    if _actual_port is None:
        _actual_port = free_port()
    if probe_serving(HOST, _actual_port):
        # 门禁已预先启动服务（本次调用传入了 host/port 且端口已在服）：
        # 直接对接，不另起进程，也不持有进程句柄（退出时不杀它）。
        print("SIM-STEP 对接已运行服务 %s:%d（不另起进程）"
              % (HOST, _actual_port))
        return None
    env = dict(os.environ)
    env["OPI_WEB_PASSWORD_SHA256"] = PASSWORD_SHA256
    env["OPI_BRIDGE_TOKEN"] = TOKEN
    env.pop("OPI_WEB_FAKE", None)  # ros 模式
    proc = subprocess.Popen(
        [binary, "--port", str(_actual_port)],
        stdout=open(log_path, "wb"), stderr=subprocess.STDOUT, env=env)
    deadline = time.time() + 15
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RuntimeError("服务进程提前退出，日志 %s" % log_path)
        try:
            with urllib.request.urlopen(
                    "http://%s:%d/api/state" % (HOST, _actual_port), timeout=2) as resp:
                if resp.status == 200:
                    print("SIM-STEP 服务已就绪 pid=%s port=%d" % (proc.pid, _actual_port))
                    return proc
        except OSError:
            time.sleep(0.2)
    raise RuntimeError("服务 15s 内未就绪，日志 %s" % log_path)


def http_json(path: str, body: dict | None = None, method: str | None = None,
              timeout: float = 30.0):
    url = "http://%s:%d%s" % (HOST, actual_port(), path)
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    if TOKEN:
        headers["X-Bridge-Token"] = TOKEN
    req = urllib.request.Request(url, data=data, headers=headers,
                                 method=method or ("POST" if data else "GET"))
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.status, json.loads(resp.read().decode("utf-8"))


# ---------------------------------------------------------------------------
# RFC6455 手写 WS 客户端（客户端帧必须带掩码）
# ---------------------------------------------------------------------------

class WsClient:
    def __init__(self, host: str, port: int, path: str):
        self.sock = socket.create_connection((host, port), timeout=10)
        key = base64.b64encode(os.urandom(16)).decode()
        request = (
            "GET %s HTTP/1.1\r\n"
            "Host: %s:%d\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Key: %s\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n" % (path, host, port, key))
        self.sock.sendall(request.encode())
        response = b""
        while b"\r\n\r\n" not in response:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise RuntimeError("握手连接关闭")
            response += chunk
        head, _, rest = response.partition(b"\r\n\r\n")
        if b" 101 " not in head.split(b"\r\n")[0]:
            raise RuntimeError("握手失败：%r" % head[:120])
        accept = base64.b64encode(
            hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        headers = head.decode("latin1").split("\r\n")[1:]
        if not any(h.lower().startswith("sec-websocket-accept:")
                   and h.split(":", 1)[1].strip() == accept for h in headers):
            raise RuntimeError("Sec-WebSocket-Accept 校验失败")
        self._buf = bytearray(rest)
        self._send_lock = threading.Lock()
        self.messages: list[dict] = []
        self._cond = threading.Condition()
        self.closed = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def send_text(self, text: str) -> None:
        payload = text.encode("utf-8")
        mask = os.urandom(4)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        header = bytearray([0x81])  # FIN + text
        n = len(payload)
        if n < 126:
            header.append(0x80 | n)
        elif n < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", n)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", n)
        header += mask
        with self._send_lock:
            self.sock.sendall(bytes(header) + masked)

    def _read_exact(self, n: int) -> bytes:
        while len(self._buf) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("对端关闭")
            self._buf += chunk
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def _read_loop(self) -> None:
        try:
            while True:
                head = self._read_exact(2)
                fin_op = head[0]
                opcode = fin_op & 0x0F
                masked = head[1] & 0x80
                length = head[1] & 0x7F
                if length == 126:
                    length = struct.unpack(">H", self._read_exact(2))[0]
                elif length == 127:
                    length = struct.unpack(">Q", self._read_exact(8))[0]
                mask = self._read_exact(4) if masked else None
                payload = self._read_exact(length)
                if mask:
                    payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
                if opcode == 0x8:  # close
                    with self._send_lock:
                        try:
                            self.sock.sendall(b"\x88\x00")  # 关闭帧回执
                        except OSError:
                            pass
                    break
                if opcode == 0x9:  # ping → pong（带掩码）
                    with self._send_lock:
                        mask = os.urandom(4)
                        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
                        self.sock.sendall(
                            bytes([0x8A, 0x80 | len(payload)]) + mask + masked)
                    continue
                if opcode in (0x1, 0x2):
                    msg = json.loads(payload.decode("utf-8"))
                    with self._cond:
                        self.messages.append(msg)
                        self._cond.notify_all()
        except (ConnectionError, OSError, ValueError):
            pass
        finally:
            self.closed.set()
            with self._cond:
                self._cond.notify_all()

    def wait_for(self, t: str, timeout: float = 5.0, from_start: bool = False) -> dict | None:
        """等待指定类型的消息。默认只看调用之后到达的（避免拿到上一条请求
        的陈旧回执）；from_start=True 时从连接后的第一条消息起扫——服务器在
        握手完成时立即推送会话帧，它与握手响应可能同批到达，先于本调用，
        只看「调用之后」会永远错过它。"""
        with self._cond:
            start = 0 if from_start else len(self.messages)
        deadline = time.time() + timeout
        with self._cond:
            while True:
                for msg in self.messages[start:]:
                    if msg.get("t") == t:
                        return msg
                remaining = deadline - time.time()
                if remaining <= 0 or self.closed.is_set():
                    return None
                self._cond.wait(remaining)

    def close(self) -> None:
        try:
            with self._send_lock:
                self.sock.sendall(b"\x88\x80" + os.urandom(4))
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        finally:
            self.sock.close()


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def main() -> int:
    global HOST, TOKEN, _actual_port
    parser = argparse.ArgumentParser(
        description="Web-Next /bridge 端到端模拟器（stdlib-only）。可选位置参数 "
                    "[host] [port]：port 在服则直接对接已运行服务，否则在 port 上"
                    "自行启动服务；两者都缺省时自选空闲端口。")
    parser.add_argument("host", nargs="?", default=None,
                        help="服务地址（默认 127.0.0.1）")
    parser.add_argument("port", nargs="?", type=int, default=None,
                        help="服务端口（缺省向内核要空闲端口）")
    parser.add_argument("--keep", action="store_true", help="保留服务日志路径打印")
    args = parser.parse_args()
    log_path = "/tmp/shim_e2e_server.log"
    if args.host:
        HOST = args.host
    if args.port:
        _actual_port = args.port
    # 令牌解析：门禁环境若已注入 OPI_BRIDGE_TOKEN（预启动服务时必需）则
    # 沿用同一值；自带启动时未注入则用固定测试常量。
    env_token = os.environ.get("OPI_BRIDGE_TOKEN")
    if env_token:
        TOKEN = env_token

    proc = None
    ws = None
    stop_license = threading.Event()
    try:
        if probe_serving(HOST, _actual_port) if _actual_port else False:
            proc = None  # 对接模式：服务已在运行
            print("SIM-STEP 对接已运行服务 %s:%d（不另起进程）"
                  % (HOST, _actual_port))
        else:
            binary = ensure_binary()
            proc = start_server(binary, log_path)

        # ---- 2) 上行帧批：底盘/电池/雷达/节点状态 ----
        status, resp = http_json("/bridge/frames", body={"mono_ms": 0, "frames": [
            {"t": "chassis", "voltage_mv": 24150, "vx_mm_s": 150, "vy_mm_s": 0,
             "yaw_rate_mrad_s": 20, "stop_flag": 0, "ultrasonic_front_mm": 1200,
             "level": 0, "message": "ok"},
            {"t": "imu", "hz": 19.8},
            {"t": "scan", "p": [[0.0, 1.5], [90.0, 0.0], [180.0, 2.25], [270.0, 3.0]],
             "n": 3, "all": 4},
            {"t": "probe",
             "nodes_seen": {"camera_node": True, "lidar_node": True,
                            "chassis_status_node": True},
             "safety": {"drive_enable": True, "drive_input_enable": True,
                        "drive_firmware_protocol": "baseline", "online": True,
                        "source": "在线节点 c50c_uart3_node", "checked_t": time.time()}},
            {"t": "moving", "bins": [0, 1, 2]},
            {"t": "drive_safety", "raw": {"allowed": True, "reason": "ok",
                                          "seq": 1, "ttl_s": 1.0}},
        ]})
        if status != 200 or not resp.get("ok"):
            return fail("frames POST: HTTP %s %r" % (status, resp))
        print("SIM-STEP frames POST accepted=%s" % resp.get("accepted"))
        if resp.get("accepted") != 6:
            return fail("accepted=%s != 6（有帧未落库）" % resp.get("accepted"))

        # ---- 3) WS：握手 → 会话帧 → 登录 → view → manual_start ----
        ws = WsClient(HOST, actual_port(), "/ws")
        # 会话帧在握手完成时立即推送（可能与握手响应同批到达），从消息 0 起扫
        hello = ws.wait_for("motion_license_session", timeout=5, from_start=True)
        if not hello or not hello.get("session_id"):
            return fail("未收到 motion_license_session")
        session = hello["session_id"]
        print("SIM-STEP WS 会话 %s" % session[:8])

        def license_worker():
            seq = 0
            while not stop_license.wait(0.1):
                seq += 1
                ws.send_text(json.dumps({"t": "motion_license", "session_id": session,
                                         "seq": seq, "sent_ms": int(time.time() * 1000),
                                         "ttl_ms": 350}))

        threading.Thread(target=license_worker, daemon=True,
                         name="license_worker").start()

        ws.send_text(json.dumps({"t": "manual_login", "username": "admin",
                                 "password": "admin"}))
        auth = ws.wait_for("manual_auth", timeout=5)
        if not auth or auth.get("ok") is not True:
            return fail("manual_auth 未通过: %r" % auth)
        ws.send_text(json.dumps({"t": "view", "page": "control"}))
        ws.send_text(json.dumps({"t": "manual_start", "speed": 0.2, "mode": "timed",
                                 "duration": 5, "direction": "forward"}))
        started = ws.wait_for("manual_started", timeout=5)
        if not started:
            return fail("manual_started 未回执（manual_start 被拒）")
        print("SIM-STEP manual_start 已接受")

        # ---- 4) 轮询 /bridge/commands：断言收到 Twist 命令 ----
        status, poll = http_json("/bridge/commands?wait_ms=3000&cursor=0")
        if status != 200:
            return fail("commands GET: HTTP %s" % status)
        cmds = poll.get("cmds", [])
        cursor = poll.get("cursor", 0)
        manual_cmd = next((c for c in cmds if c.get("c") == "manual_cmd"), None)
        if not manual_cmd:
            return fail("命令队列未收到 manual_cmd（Twist 命令缺失）: %r" %
                        [c.get("c") for c in cmds])
        if abs(manual_cmd.get("vx", 0.0) - 0.2) > 1e-6 or \
                abs(manual_cmd.get("wz", 0.0)) > 1e-6:
            return fail("manual_cmd 值不对: %r" % manual_cmd)
        extra = sorted({c.get("c") for c in cmds} - {"manual_cmd"})
        print("SIM-STEP 收到命令: manual_cmd + %s" % (extra or "无"))

        # ---- 5) manual_stop → manual_zero（tick 侧对账派发）----
        # 同窗口顺带验证许可转发（license_forward）与授权（guard_authorize
        # True）：100ms 许可流每条被接受的许可都伴随转发 + 按值去重授权。
        ws.send_text(json.dumps({"t": "manual_stop"}))
        deadline = time.time() + 5
        got_zero = False
        got_license_fwd = False
        saw_authorize_true = False
        while time.time() < deadline and not (got_zero and got_license_fwd):
            status, poll = http_json("/bridge/commands?wait_ms=1500&cursor=%s" % cursor)
            if status != 200:
                return fail("commands GET#2: HTTP %s" % status)
            for c in poll.get("cmds", []):
                if c.get("c") == "manual_zero":
                    got_zero = True
                elif c.get("c") == "license_forward":
                    got_license_fwd = True
                    frame = c.get("frame") or {}
                    # 下游 guard 协议（motion_guard.py License.check）：session
                    # 必须 int≥1（浏览器 hex 会话的稳定映射）、expires_mono 为
                    # boot 相对单调秒、seq 原样。
                    sess = frame.get("session")
                    if not isinstance(sess, int) or isinstance(sess, bool) or sess < 1:
                        return fail("license_forward.session 必须 int≥1: %r" % frame)
                    if not isinstance(frame.get("expires_mono"), (int, float)) \
                            or frame.get("expires_mono", 0) <= 0:
                        return fail("license_forward.expires_mono 非法: %r" % frame)
                    if not isinstance(frame.get("seq"), int) or frame.get("seq", -1) < 1:
                        return fail("license_forward.seq 非法: %r" % frame)
                elif c.get("c") == "guard_authorize" and c.get("value") is True:
                    saw_authorize_true = True
            cursor = poll.get("cursor", cursor)
        if not got_zero:
            return fail("manual_stop 后未收到 manual_zero（tick 对账缺失）")
        if not got_license_fwd:
            return fail("许可流未转发 license_forward（web_debug.py:909-924 缺失）")
        if not saw_authorize_true:
            return fail("许可接受未伴随 guard_authorize(True)（web_debug.py:640,655）")
        print("SIM-STEP manual_zero + license_forward + authorize(True) 已验证")

        # ---- 6) GET /api/state：chassis/battery/nodes 非空 ----
        # 等一拍 20Hz 装配把 Raw 折算进状态树
        time.sleep(0.3)
        status, state = http_json("/api/state")
        if status != 200:
            return fail("api/state: HTTP %s" % status)
        chassis = state.get("chassis") or {}
        battery = state.get("battery") or {}
        nodes = state.get("nodes") or {}
        if chassis.get("state") != "fresh":
            return fail("chassis.state=%r != fresh" % chassis.get("state"))
        if chassis.get("voltage_v") != 24.15:
            return fail("chassis.voltage_v=%r != 24.15" % chassis.get("voltage_v"))
        if battery.get("percent") is None or battery.get("text") in (None, "无数据"):
            return fail("battery 视图为空: %r" % battery)
        if not nodes or nodes.get("web_bridge") != "online":
            return fail("nodes 视图异常: %r" % nodes)
        if nodes.get("c50c_uart3_node") != "online":
            return fail("c50c_uart3_node=%r != online（probe 未落库）" %
                        nodes.get("c50c_uart3_node"))
        if not isinstance(state.get("scan_hz"), (int, float)):
            return fail("scan_hz 非数值: %r" % state.get("scan_hz"))
        # network 键必须存在（值允许 null：本机可能没有无线网卡）
        if "network" not in state:
            return fail("api/state 缺失 network 键")
        epoch_before = state.get("srv_epoch")
        print("SIM-STEP /api/state: chassis fresh v=%s battery=%s%% nodes.web_bridge=%s"
              % (chassis.get("voltage_v"), battery.get("percent"),
                 nodes.get("web_bridge")))

        # ---- 7) 巡逻全环：map → patrol_save → patrol_start → nav_goal 派发
        #          → nav_result 推进（index/laps 回绕）→ motion_stop 取消 ----
        raw_grid = bytes([0, 100, 255, 0])  # 2×2 栅格内容（map_id 的输入）
        import zlib as _z
        grid_b64 = base64.b64encode(
            _z.compress(raw_grid, 6)).decode("ascii")
        status, resp = http_json("/bridge/frames", body={"mono_ms": 0, "frames": [
            {"t": "map", "seq": 1, "w": 2, "h": 2, "res": 0.05,
             "ox": -8.28, "oy": -8.03, "d": grid_b64},
        ]})
        if status != 200 or resp.get("accepted") != 1:
            return fail("map 帧未落库: %r" % resp)
        ws.send_text(json.dumps({"t": "patrol_save", "points":
                                 [[0.5, 0.5], [1.5, 0.5], [1.5, 1.5]]}))
        saved = ws.wait_for("patrol_status", timeout=5)
        if not saved or saved.get("ok") is not True:
            return fail("patrol_save 失败: %r" % (saved or {}).get("message"))
        ws.send_text(json.dumps({"t": "patrol_start"}))
        started_p = ws.wait_for("patrol_status", timeout=5)
        if not started_p or started_p.get("ok") is not True:
            return fail("patrol_start 失败: %r" % (started_p or {}).get("message"))
        expect_rids = ["p0-0-", "p0-1-", "p0-2-", "p1-0-"]  # 3 航点一圈后回绕
        for expect in expect_rids:
            # 巡逻派发发生在 20Hz tick 的 nav_housekeeping（patrol_start 落库
            # 后的下一个 50ms 节拍），单次长轮询会与该节拍竞态——本条 poll
            # 可能恰好被此刻挂起的 license_forward 立即唤醒返回、还没有
            # nav_goal。必须循环轮询直到期望 rid 出现（或超时）。
            goal = None
            deadline = time.time() + 5
            while goal is None and time.time() < deadline:
                status, poll = http_json(
                    "/bridge/commands?wait_ms=1000&cursor=%s" % cursor)
                if status != 200:
                    return fail("commands GET(巡逻): HTTP %s" % status)
                cursor = poll.get("cursor", cursor)
                goal = next((c for c in poll.get("cmds", [])
                             if c.get("c") == "nav_goal"
                             and str(c.get("rid", "")).startswith(expect)), None)
            if not goal:
                _, st_now = http_json("/api/state")
                pv = st_now.get("patrol") or {}
                lic = pv.get("license") or {}
                from collections import Counter as _C
                kinds = dict(_C(m.get("t") for m in ws.messages))
                lic_alive = any(t.name == "license_worker" and t.is_alive()
                                for t in threading.enumerate())
                return fail("巡逻派发异常（期望 %s…）: cmds=%r patrol=%r lic=%r kinds=%r "
                            "lic_thread_alive=%s epoch_now=%s epoch_before=%s "
                            "alive=%s start_reply=%r" %
                            (expect, [c.get("c") for c in poll.get("cmds", [])],
                             {k: pv.get(k) for k in ("status", "index", "laps", "locked",
                                                     "stop_reason", "route_valid",
                                                     "licensed", "license_state")},
                             {k: lic.get(k) for k in ("seq", "fresh", "age_s", "rejected",
                                                      "session_dead", "takeovers")},
                             kinds, lic_alive,
                             st_now.get("srv_epoch"), epoch_before,
                             ("adopted" if proc is None
                              else ("alive" if proc.poll() is None else "dead")),
                             (started_p or {}).get("message")))
            rid = goal["rid"]
            for st in ("accepted", "succeeded"):
                http_json("/bridge/frames", body={"mono_ms": 0, "frames": [
                    {"t": "nav_result", "rid": rid, "status": st, "text": ""}]})
        state2 = None
        status, state2 = http_json("/api/state")
        patrol_view = state2.get("patrol") or {}
        if patrol_view.get("laps") != 1 or patrol_view.get("index") != 0:
            return fail("巡逻回绕异常: laps=%r index=%r" %
                        (patrol_view.get("laps"), patrol_view.get("index")))
        # 回绕后第 2 圈第 2 个航点由 tick 继续派发（p1-1-…）——先观察到
        # 它在途，再 motion_stop，nav_cancel 断言才确定（motion_stop 若在
        # 派发节拍前到达，nav.rid 仍为 None，本来就不该发 cancel）。
        deadline = time.time() + 5
        next_goal = None
        while next_goal is None and time.time() < deadline:
            status, poll = http_json("/bridge/commands?wait_ms=1000&cursor=%s" % cursor)
            if status != 200:
                return fail("commands GET(回绕后): HTTP %s" % status)
            cursor = poll.get("cursor", cursor)
            next_goal = next((c for c in poll.get("cmds", [])
                              if c.get("c") == "nav_goal"), None)
        if not next_goal or not str(next_goal.get("rid", "")).startswith("p1-1-"):
            return fail("回绕后未继续派发 p1-1-…: %r" % (next_goal,))
        ws.send_text(json.dumps({"t": "motion_stop"}))
        deadline = time.time() + 5
        got_cancel = False
        while time.time() < deadline and not got_cancel:
            status, poll = http_json("/bridge/commands?wait_ms=1500&cursor=%s" % cursor)
            cursor = poll.get("cursor", cursor)
            got_cancel = any(c.get("c") == "nav_cancel"
                             and c.get("rid") == next_goal.get("rid")
                             for c in poll.get("cmds", []))
        if not got_cancel:
            return fail("motion_stop 后未派发 nav_cancel（在途目标未取消）")
        print("SIM-STEP 巡逻全环：4 航点派发/推进/回绕 laps=1 + motion_stop 取消 ✓")

        if args.keep and proc is not None:
            print("服务日志：%s" % log_path)
        print("SIM_RESULT=status_ok,command_ok")
        return 0
    except Exception as exc:  # noqa: BLE001 —— 任何异常都如实报告
        return fail("%s: %s" % (type(exc).__name__, exc))
    finally:
        stop_license.set()
        if ws is not None:
            ws.close()
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()


if __name__ == "__main__":
    sys.exit(main())
