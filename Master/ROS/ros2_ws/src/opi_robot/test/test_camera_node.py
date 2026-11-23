"""camera_node 断流测试：拔线判定、节流重连、不重发旧帧。

用假 V4L2Camera 驱动真实节点代码，不碰 /dev/video*。
"""

import time as _time
import types

import pytest

import fake_ros
from opi_robot.cam_node import camera_node as CN


class FakeCam:
    """可控的摄像头替身：能返回帧，也能模拟拔线。"""

    instances = []

    def __init__(self, device, w, h, fmt, buffers=4):
        self.device = device
        self.width = w
        self.height = h
        self.pixel_format = fmt
        self.sizeimage = 1000
        self.num_buffers = 4
        self.open_calls = 0
        self.close_calls = 0
        self.alive = True
        self.fail_open = False
        self.seq = 0
        self.payload = b'\xff\xd8jpeg\xff\xd9'
        FakeCam.instances.append(self)

    def open(self):
        self.open_calls += 1
        if self.fail_open:
            raise OSError(2, 'No such file or directory')
        self.alive = True
        return CN.fourcc(self.pixel_format)

    def grab(self, timeout=0.5):
        if not self.alive:
            return None
        self.seq += 1
        return self.payload, self.seq

    def close(self):
        self.close_calls += 1
        self.alive = False


@pytest.fixture
def clock():
    """可推进的单调时钟。"""
    state = {'t': 100.0}

    def mono():
        return state['t']

    def sleep(s):
        state['t'] += s
        return None

    state['mono'] = mono
    state['sleep'] = sleep
    state['advance'] = lambda d: state.__setitem__('t', state['t'] + d)
    return state


@pytest.fixture
def node(monkeypatch, clock):
    FakeCam.instances = []
    monkeypatch.setattr(CN, 'V4L2Camera', FakeCam)
    monkeypatch.setattr(CN, 'time', types.SimpleNamespace(
        monotonic=clock['mono'], sleep=lambda s: None))
    return CN.CameraNode()


def published(node):
    return node.pub.messages


class TestStartup:
    def test_publishes_on_first_frames(self, node):
        for _ in range(3):
            node.poll()
        assert len(published(node)) == 3

    def test_open_failure_does_not_crash(self, monkeypatch, clock):
        FakeCam.instances = []
        monkeypatch.setattr(CN, 'V4L2Camera', FakeCam)
        monkeypatch.setattr(CN, 'time', types.SimpleNamespace(
            monotonic=clock['mono'], sleep=lambda s: None))
        orig = FakeCam.open

        def boom(self):
            raise OSError(2, 'No such file or directory')
        monkeypatch.setattr(FakeCam, 'open', boom)
        node = CN.CameraNode()
        assert node.streaming is False
        node.poll()      # 不应抛异常
        assert published(node) == []


class TestStreamLoss:
    def test_detects_stream_loss_after_timeout(self, node, clock):
        node.poll()
        assert node.streaming is True
        cam = FakeCam.instances[-1]
        cam.alive = False        # 模拟拔线
        clock['advance'](1.0)
        node.poll()              # 未超时，仍认为在流
        assert node.streaming is True
        clock['advance'](1.5)    # 累计 2.5s > stream_timeout 2.0
        node.poll()
        assert node.streaming is False
        assert cam.close_calls >= 1

    def test_no_frames_published_while_down(self, node, clock, monkeypatch):
        """设备物理拔掉：重开也失败，全程不得发布任何图像。"""
        node.poll()
        cam = FakeCam.instances[-1]
        cam.alive = False

        def gone(self):
            raise OSError(2, 'No such file or directory')
        monkeypatch.setattr(FakeCam, 'open', gone)

        before = len(published(node))
        for _ in range(50):
            clock['advance'](0.1)
            node.poll()
        assert len(published(node)) == before
        assert node.streaming is False

    def test_reconnect_is_throttled(self, node, clock):
        node.poll()
        cam = FakeCam.instances[-1]
        cam.alive = False
        clock['advance'](3.0)
        node.poll()
        assert node.streaming is False
        base = len(FakeCam.instances)
        # 节流窗口内多次轮询不应反复打开
        for _ in range(20):
            clock['advance'](0.01)
            node.poll()
        assert len(FakeCam.instances) == base

    def test_reconnects_after_interval(self, node, clock):
        node.poll()
        cam = FakeCam.instances[-1]
        cam.alive = False
        clock['advance'](3.0)
        node.poll()
        assert node.streaming is False
        clock['advance'](node.reconnect_interval + 0.1)
        node.poll()
        assert node.streaming is True
        assert node.reconnects == 1

    def test_new_frames_after_recovery(self, node, clock):
        node.poll()
        cam = FakeCam.instances[-1]
        cam.alive = False
        clock['advance'](3.0)
        node.poll()
        before = len(published(node))
        clock['advance'](node.reconnect_interval + 0.1)
        node.poll()               # 重开
        node.poll()               # 取新帧
        assert len(published(node)) > before

    def test_published_stamps_increase(self, node, clock):
        stamps = []
        for _ in range(3):
            node.poll()
            clock['advance'](0.1)
        for m in published(node):
            stamps.append((m.header.stamp.sec, m.header.stamp.nanosec))
        assert stamps == sorted(stamps)


class TestOpenRetry:
    def test_open_failure_retried_with_throttle(self, monkeypatch, clock):
        FakeCam.instances = []
        monkeypatch.setattr(CN, 'V4L2Camera', FakeCam)
        monkeypatch.setattr(CN, 'time', types.SimpleNamespace(
            monotonic=clock['mono'], sleep=lambda s: None))
        orig_open = FakeCam.open

        def fail_once(self):
            if len(FakeCam.instances) == 1:
                raise OSError(16, 'Device or resource busy')
            return orig_open(self)
        monkeypatch.setattr(FakeCam, 'open', fail_once)
        node = CN.CameraNode()
        assert node.streaming is False
        assert 'busy' in node.last_error
        clock['advance'](node.reconnect_interval + 0.1)
        node.poll()
        assert node.streaming is True


# ---------- Codex 审核第 7 项：资源泄漏与首帧超时 ----------


class FakeFdCam:
    """记录 open/close 次数的替身，用来看 fd 是否泄漏。"""

    def __init__(self, device, w, h, fmt, buffers=4):
        self.device = device
        self.width = w
        self.height = h
        self.pixel_format = fmt
        self.sizeimage = 1000
        self.num_buffers = 4
        self.opened = 0
        self.closed = 0
        FakeFdCam.instances.append(self)

    def open(self):
        self.opened += 1
        if FakeFdCam.fail_at_open is not None:
            FakeFdCam.fail_at_open(self)
        return CN.fourcc(self.pixel_format)

    def grab(self, timeout=0.5):
        return None          # 永远不出帧

    def close(self):
        self.closed += 1


FakeFdCam.instances = []
FakeFdCam.fail_at_open = None


class TestFirstFrameTimeout:
    """设备打开成功但一直不出帧时，必须按断流处理并关闭。

    Codex 复现的条件：``/dev/video*`` 存在、``VIDIOC_STREAMON`` 成功，
    但摄像头不产生任何帧。旧代码只在"曾经收到过帧"之后才做超时判断，
    这种情况会一直干等，节点既不报错也不重连。
    """

    def test_first_frame_never_arrives_triggers_close(self, monkeypatch, clock):
        monkeypatch.setattr(CN, 'V4L2Camera', FakeFdCam)
        monkeypatch.setattr(CN.time, 'monotonic', clock['mono'])
        monkeypatch.setattr(CN.time, 'sleep', clock['sleep'])
        FakeFdCam.instances = []
        FakeFdCam.fail_at_open = None

        node = CN.CameraNode()
        assert node.streaming is True
        cam = FakeFdCam.instances[-1]

        clock['advance'](0.5)
        node.poll()
        assert node.streaming is True, '短时间没帧不应立刻判定断流'

        clock['advance'](2.5)         # 超过 stream_timeout=2.0
        node.poll()
        assert node.streaming is False, '首帧超时后必须关闭设备'
        assert cam.closed == 1
        assert node.cam is None

    def test_first_frame_timeout_reconnects_after_interval(self, monkeypatch,
                                                          clock):
        monkeypatch.setattr(CN, 'V4L2Camera', FakeFdCam)
        monkeypatch.setattr(CN.time, 'monotonic', clock['mono'])
        monkeypatch.setattr(CN.time, 'sleep', clock['sleep'])
        FakeFdCam.instances = []
        FakeFdCam.fail_at_open = None

        node = CN.CameraNode()
        clock['advance'](3.0)
        node.poll()                    # 首帧超时，关闭
        assert node.streaming is False
        n_before = len(FakeFdCam.instances)

        clock['advance'](0.5)
        node.poll()                    # 还没到重连间隔
        assert len(FakeFdCam.instances) == n_before

        clock['advance'](2.0)
        node.poll()                    # 到间隔，重新打开
        assert len(FakeFdCam.instances) == n_before + 1


class TestOpenFailureReleases:
    """打开中途失败时必须释放已分配资源。

    Codex 复现的条件：``VIDIOC_S_FMT`` 成功、``mmap`` 失败。
    旧代码直接把异常抛给 ``_open_once`` 的 ``except OSError``，
    fd 与已建的映射留在对象里，每次重试都多泄漏一份。
    """

    def test_open_exception_calls_close(self, monkeypatch, clock):
        """让真实的 V4L2Camera 走到 mmap 阶段失败，确认自己收尾。"""
        monkeypatch.setattr(CN.time, 'monotonic', clock['mono'])
        monkeypatch.setattr(CN.time, 'sleep', clock['sleep'])

        closed = {'n': 0}
        real_close = CN.V4L2Camera.close
        real_mmap = CN.mmap.mmap
        real_ioctl = CN.fcntl.ioctl
        real_os_open = CN.os.open
        real_os_close = CN.os.close
        state = {'fd_open': False, 'closed': 0}

        def fake_open(path, flags):
            state['fd_open'] = True
            return 42

        def fake_close(fd):
            if state['fd_open']:
                state['fd_open'] = False
                state['closed'] += 1

        def boom(*a, **kw):
            raise OSError(12, 'Cannot allocate memory')

        def fake_ioctl(fd, request, arg=None):
            # 让 REQBUFS 报告 2 个缓冲，随后 mmap 失败
            if request == CN.VIDIOC_REQBUFS and arg is not None:
                CN.struct.pack_into('<I', arg, 0, 2)
            return 0

        monkeypatch.setattr(CN.os, 'open', fake_open)
        monkeypatch.setattr(CN.os, 'close', fake_close)
        monkeypatch.setattr(CN.fcntl, 'ioctl', fake_ioctl)
        monkeypatch.setattr(CN.mmap, 'mmap', boom)

        cam = CN.V4L2Camera('/dev/null', 640, 480, 'MJPG')
        with pytest.raises(OSError):
            cam.open()
        assert state['closed'] == 1, '打开中途失败必须关闭已打开的 fd'
        assert cam.fd is None
        assert cam.maps == []

    def test_open_is_retryable_after_failure(self, monkeypatch, clock):
        monkeypatch.setattr(CN.time, 'monotonic', clock['mono'])
        monkeypatch.setattr(CN.time, 'sleep', clock['sleep'])

        state = {'fail': True}

        class FlakyCam(FakeCam):
            def open(self):
                self.open_calls += 1
                if state['fail']:
                    raise OSError(13, 'Permission denied')
                return CN.fourcc(self.pixel_format)

        monkeypatch.setattr(CN, 'V4L2Camera', FlakyCam)
        node = CN.CameraNode()
        assert node.streaming is False
        attempts = node.open_attempts

        clock['advance'](2.5)
        state['fail'] = False
        node.poll()
        assert node.open_attempts == attempts + 1
        assert node.streaming is True


class TestV4L2CameraClose:
    """V4L2Camera.close() 必须可重复调用，且清空缓冲列表。"""

    def test_close_is_idempotent(self):
        cam = CN.V4L2Camera('/dev/null', 640, 480, 'MJPG')
        cam.close()
        cam.close()
        assert cam.maps == []
        assert cam.fd is None
