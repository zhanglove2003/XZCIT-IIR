"""Authenticated manual motion state for the web console."""

import hashlib
import hmac
import math
import os
import threading
import time


MAX_INPUT = 0.8
MAX_TURN_INPUT = 0.8
# 2026-11-14 用户确认：速度策略由网页挡位定义（空档 0 / 巡航挡 0.4 /
# 运动挡 0.8）。服务端输入上限钉在最高挡——挡位之外的速度在此拒绝，
# 不再依赖桥的仲裁上限兜底。转向同样取挡位值，两个上限一致。
MAX_TIMED_SECONDS = 10.0
HEARTBEAT_TIMEOUT = 0.35
PUBLISH_PERIOD = 0.10


_STOP_LOG = []
# 诊断计数：心跳到达次数与周期重发次数（帧 0 项）。
_START_LOG = []
_BEAT_COUNT = [0]
_PUB_COUNT = [0]


class ManualForwardControl:
    """One authenticated browser owns a motion command until it stops."""

    def __init__(self, now=None):
        self._now = now or time.monotonic
        self._lock = threading.RLock()
        self._authenticated = set()
        self._publish = None
        self._ready = None
        self._owner = None
        self._speed = 0.0
        self._command = (0.0, 0.0)
        self._direction = 'forward'
        self._last_heartbeat = 0.0
        self._last_publish = 0.0
        self._deadline = None
        self._takeover_hook = None

    def attach(self, publish, ready=None):
        with self._lock:
            self._publish = publish
            self._ready = ready

    def login(self, owner, username, password):
        if not isinstance(username, str) or not isinstance(password, str):
            return False
        digest = hashlib.sha256(password.encode('utf-8', 'surrogatepass')).hexdigest()
        configured_digest = os.environ.get('OPI_WEB_PASSWORD_SHA256', '')
        valid = (hmac.compare_digest(username.encode('utf-8', 'surrogatepass'), b'admin') and
                 hmac.compare_digest(digest.encode('ascii'), configured_digest.encode('utf-8')))
        with self._lock:
            if valid:
                self._authenticated.add(owner)
            else:
                self.disconnect(owner)
        return valid

    def authenticated(self, owner):
        with self._lock:
            return owner in self._authenticated

    @staticmethod
    def _positive_number(value, maximum, name):
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or not 0 < value <= maximum):
            raise ValueError(f'{name} 必须大于 0 且不超过 {maximum}')
        return float(value)

    def start(self, owner, speed, mode, duration=None, *, direction='forward'):
        with self._lock:
            if owner not in self._authenticated:
                raise PermissionError('请先登录手动控制')
            # 巡航接管：真正从空闲进入手动输入时，先取消旧导航。
            if self._owner is None and self._takeover_hook is not None:
                try:
                    self._takeover_hook(owner)
                except Exception:          # noqa: BLE001——接管钩子不影响手动输入
                    pass
            if self._publish is None or (self._ready is not None and not self._ready()):
                raise RuntimeError('底盘命令通道未就绪，检查底盘节点和发布者')
            if self._owner is not None:
                raise RuntimeError('已有控制命令运行，请先停车')
            # 转向符号以实车为准：positive angular.z 经固件差速后让车右转，
            # 所以"左转"按钮发负值。2026-11-13 巡航档实测左右反了。
            directions = {
                'forward': (1.0, 0.0, MAX_INPUT),
                'reverse': (-1.0, 0.0, MAX_INPUT),
                'left': (0.0, -1.0, MAX_TURN_INPUT),
                'right': (0.0, 1.0, MAX_TURN_INPUT),
            }
            if not isinstance(direction, str) or direction not in directions:
                raise ValueError('未知控制方向')
            x_sign, z_sign, maximum = directions[direction]
            speed = self._positive_number(speed, maximum, '运动输入')
            if mode == 'timed':
                duration = self._positive_number(duration, MAX_TIMED_SECONDS, '运行时间')
            elif mode != 'hold' or duration is not None:
                raise ValueError('未知控制方式')
            now = self._now()
            self._command = (x_sign * speed, z_sign * speed)
            self._speed = speed
            self._direction = direction
            _START_LOG.append('%.3f dir=%s speed=%.3f cmd=%.3f,%.3f'
                              % (now, direction, speed,
                                 self._command[0], self._command[1]))
            if len(_START_LOG) > 50:
                del _START_LOG[:25]
            self._publish(*self._command)
            self._owner = owner
            self._speed = speed
            self._direction = direction
            self._last_heartbeat = now
            self._last_publish = now
            self._deadline = now + duration if mode == 'timed' else None

    def takeover_hook(self, hook):
        """登记"本连接即将开始一次手动输入"的钩子（巡航取消旧导航）。

        钩子在该连接通过鉴权、且此刻并不按着手动键时、**在发布第一条
        非零速度之前**调用；钩子返回值不参与安全判断（服务端只看自己
        的巡航状态机）。放在 start 里而不是调用方，是为了让"先取消旧
        导航、再发手动速度"的顺序由这一个入口保证。
        """
        with self._lock:
            self._takeover_hook = hook

    def binding(self, owner):
        """本连接是否处在"即将可以手动输入"的状态：已登录、没有别的持有者。"""
        with self._lock:
            return owner in self._authenticated and self._owner is None

    def holds(self, owner):
        """本连接此刻是否在按着手动键（有活跃手动命令）。

        巡航的"手动接管"只在真的接管时触发：否则"手动接管"会把别的
        操作（全局停车）留下的锁停顺手放开。
        """
        with self._lock:
            return self._owner is not None and self._owner == owner

    def heartbeat(self, owner):
        _BEAT_COUNT[0] += 1
        with self._lock:
            if self._owner is owner and owner in self._authenticated:
                now = self._now()
                if (now - self._last_heartbeat > HEARTBEAT_TIMEOUT or
                        (self._deadline is not None and now >= self._deadline)):
                    self._stop()
                else:
                    self._last_heartbeat = now

    def _stop(self):
        active = self._owner is not None
        if active:
            # 诊断：记录主动停车的原因，便于区分心跳超时、定时结束与手动停车���
            import traceback
            frame = traceback.extract_stack(limit=3)[0]
            _STOP_LOG.append('%.3f %s:%d' % (self._now(), frame.name, frame.lineno))
            if len(_STOP_LOG) > 200:
                del _STOP_LOG[:100]
        self._owner = None
        self._speed = 0.0
        self._command = (0.0, 0.0)
        self._deadline = None
        if active:
            self._publish(*self._command)

    def stop(self, owner):
        with self._lock:
            if owner not in self._authenticated:
                raise PermissionError('请先登录手动控制')
            self._stop()

    def disconnect(self, owner):
        with self._lock:
            self._authenticated.discard(owner)
            if self._owner is owner:
                self._stop()

    def snapshot_debug(self):
        # 诊断：返回最近 20 条停车原因与心跳/发布计数。
        # owner 输出不透明标识（对象 id 字符串）而不是对象本身：它会进入
        # status_payload 的 JSON——2026-11-14 修复：此前手动驾驶激活时
        # owner 是连接对象，json.dumps 必然 TypeError，状态推送全程失败。
        with self._lock:
            owner = None if self._owner is None else str(id(self._owner))
            return {'stops': list(_STOP_LOG[-20:]), 'owner': owner,
                    'beats': _BEAT_COUNT[0], 'pubbed': _PUB_COUNT[0],
                    'gap': round(self._now() - self._last_heartbeat, 3),
                    'starts': list(_START_LOG[-6:])}

    def tick(self):
        with self._lock:
            if self._owner is None:
                return
            now = self._now()
            if (now - self._last_heartbeat > HEARTBEAT_TIMEOUT or
                    (self._deadline is not None and now >= self._deadline)):
                self._stop()
            elif now - self._last_publish >= PUBLISH_PERIOD:
                _PUB_COUNT[0] += 1
                self._publish(*self._command)
                self._last_publish = now

    def snapshot(self):
        with self._lock:
            remaining = None if self._deadline is None else max(0.0, self._deadline - self._now())
            return {'available': self._publish is not None,
                    'active': self._owner is not None,
                    'input': self._speed, 'direction': self._direction,
                    'remaining': remaining}
