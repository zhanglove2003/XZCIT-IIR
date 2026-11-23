"""USB 摄像头 ROS2 节点。

用 V4L2 MMAP 直接采集 MJPG 帧，发布 sensor_msgs/CompressedImage。
不依赖 cv_bridge 与 OpenCV，MJPG 帧原样透传。

结构体布局按 Linux 内核 v4l2_buffer / v4l2_format 真实偏移：
  v4l2_format(208B): type@0, pix.width@8, pix.height@12, pix.pixelformat@16,
                     pix.field@20, pix.bytesperline@24, pix.sizeimage@28
  v4l2_buffer(68B):  index@0, type@4, bytesused@8, flags@12, field@16,
                     sequence@44, memory@48, m.offset@52, length@56
"""

import os
import fcntl
import mmap
import struct
import select
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from .. import qos as Q

VIDIOC_S_FMT = 0xC0D05605
VIDIOC_REQBUFS = 0xC0145608
VIDIOC_QUERYBUF = 0xC0585609
VIDIOC_QBUF = 0xC058560F
VIDIOC_DQBUF = 0xC0585611
VIDIOC_STREAMON = 0x40045612
VIDIOC_STREAMOFF = 0x40045613

V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_FIELD_NONE = 1

# v4l2_buffer 字段偏移（结构体 88 字节，对齐与内核一致）
OFF_BUF_INDEX = 0
OFF_BUF_TYPE = 4
OFF_BUF_BYTESUSED = 8
OFF_BUF_SEQUENCE = 56
OFF_BUF_MEMORY = 60
OFF_BUF_OFFSET = 64
OFF_BUF_LENGTH = 72
BUF_SIZE = 88

# v4l2_format 字段偏移
OFF_FMT_WIDTH = 8
OFF_FMT_HEIGHT = 12
OFF_FMT_PIXELFORMAT = 16
OFF_FMT_FIELD = 20
OFF_FMT_SIZEIMAGE = 28
FMT_SIZE = 208


def fourcc(s):
    """四字符码转整数。"""
    return (ord(s[0]) | (ord(s[1]) << 8) | (ord(s[2]) << 16) | (ord(s[3]) << 24))


def fourcc_str(v):
    """整数转四字符码。"""
    return ''.join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


class V4L2Camera:
    """最小 V4L2 采集封装。"""

    def __init__(self, device, width, height, pixel_format, buffers=4):
        self.device = device
        self.width = width
        self.height = height
        self.pixel_format = pixel_format
        self.num_buffers = buffers
        self.fd = None
        self.maps = []
        self.buf_lens = []
        self.streaming = False

    def _buf(self, index):
        """构造一个 v4l2_buffer。"""
        b = bytearray(BUF_SIZE)
        struct.pack_into('<I', b, OFF_BUF_INDEX, index)
        struct.pack_into('<I', b, OFF_BUF_TYPE, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into('<I', b, OFF_BUF_MEMORY, V4L2_MEMORY_MMAP)
        return b

    def open(self):
        """打开设备、协商格式、映射缓冲区并启动采集。

        中途任何一步失败都必须释放已分配的描述符与 mmap，
        否则每次重试都会多泄漏一个 fd 和若干映射，最终让进程耗尽句柄。
        """
        try:
            return self._open_unchecked()
        except Exception:
            self.close()
            raise

    def _open_unchecked(self):
        """真正打开流程。失败时不负责清理，由 open() 统一收尾。"""
        self.fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)

        fmt = bytearray(FMT_SIZE)
        struct.pack_into('<I', fmt, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into('<I', fmt, OFF_FMT_WIDTH, self.width)
        struct.pack_into('<I', fmt, OFF_FMT_HEIGHT, self.height)
        struct.pack_into('<I', fmt, OFF_FMT_PIXELFORMAT, fourcc(self.pixel_format))
        struct.pack_into('<I', fmt, OFF_FMT_FIELD, V4L2_FIELD_NONE)
        fcntl.ioctl(self.fd, VIDIOC_S_FMT, fmt)

        self.width = struct.unpack_from('<I', fmt, OFF_FMT_WIDTH)[0]
        self.height = struct.unpack_from('<I', fmt, OFF_FMT_HEIGHT)[0]
        actual = struct.unpack_from('<I', fmt, OFF_FMT_PIXELFORMAT)[0]
        self.sizeimage = struct.unpack_from('<I', fmt, OFF_FMT_SIZEIMAGE)[0]

        req = bytearray(20)
        struct.pack_into('<I', req, 0, self.num_buffers)
        struct.pack_into('<I', req, 4, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into('<I', req, 8, V4L2_MEMORY_MMAP)
        fcntl.ioctl(self.fd, VIDIOC_REQBUFS, req)
        self.num_buffers = struct.unpack_from('<I', req, 0)[0]

        for i in range(self.num_buffers):
            b = self._buf(i)
            fcntl.ioctl(self.fd, VIDIOC_QUERYBUF, b)
            length = struct.unpack_from('<I', b, OFF_BUF_LENGTH)[0]
            offset = struct.unpack_from('<I', b, OFF_BUF_OFFSET)[0]
            self.buf_lens.append(length)
            self.maps.append(
                mmap.mmap(self.fd, length, mmap.MAP_SHARED,
                          mmap.PROT_READ | mmap.PROT_WRITE, offset=offset))

        for i in range(self.num_buffers):
            fcntl.ioctl(self.fd, VIDIOC_QBUF, self._buf(i))

        fcntl.ioctl(self.fd, VIDIOC_STREAMON,
                    struct.pack('<I', V4L2_BUF_TYPE_VIDEO_CAPTURE))
        self.streaming = True
        return actual

    def grab(self, timeout=0.5):
        """取一帧并立刻归还。返回 (bytes, seq) 或 None。"""
        # 先检查队列里还有没有未取的帧，避免 select 空转
        while True:
            r, _, _ = select.select([self.fd], [], [], timeout)
            if not r:
                return None
            b = self._buf(0)
            try:
                fcntl.ioctl(self.fd, VIDIOC_DQBUF, b)
            except OSError:
                return None
            index = struct.unpack_from('<I', b, OFF_BUF_INDEX)[0]
            bytesused = struct.unpack_from('<I', b, OFF_BUF_BYTESUSED)[0]
            seq = struct.unpack_from('<I', b, OFF_BUF_SEQUENCE)[0]
            if bytesused == 0 or bytesused > self.buf_lens[index]:
                bytesused = self.buf_lens[index]
            data = bytes(self.maps[index][:bytesused])
            fcntl.ioctl(self.fd, VIDIOC_QBUF, self._buf(index))
            return data, seq

    def close(self):
        """停止采集并释放资源。可重复调用，部分初始化后也能安全收尾。"""
        if self.streaming and self.fd is not None:
            try:
                fcntl.ioctl(self.fd, VIDIOC_STREAMOFF,
                            struct.pack('<I', V4L2_BUF_TYPE_VIDEO_CAPTURE))
            except OSError:
                pass
        self.streaming = False
        for m in self.maps:
            try:
                m.close()
            except Exception:
                pass
        self.maps = []
        self.buf_lens = []
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None


class CameraNode(Node):
    """摄像头采集与压缩图像发布。

    第一阶段补丁：断流检测与重连。

    原实现拔线后 ``grab`` 只返回 None，节点继续空转、不再发布。
    上层无法区分"没帧"和"摄像头坏了"，只能靠超时猜。
    这里改为：
    - 连续 ``stream_timeout`` 秒没有取到帧，判定断流并关闭设备；
    - 按 ``reconnect_interval`` 节流重新打开，避免空转刷屏；
    - 断流期间不发布任何图像，绝不重发旧帧；
    - 恢复后重新打开设备并继续发布新帧。
    """

    def __init__(self, context=None):
        # context 用于测试脚本在独立域里建节点（见 scripts/ros_isolation.py）。
        # 生产启动走 launch，不传 context，行为不变。
        super().__init__('camera_node', context=context)

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('pixel_format', 'MJPG')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('fps_limit', 0.0)
        self.declare_parameter('stream_timeout', 2.0)
        self.declare_parameter('reconnect_interval', 2.0)

        dev = self.get_parameter('device').value
        w = self.get_parameter('width').value
        h = self.get_parameter('height').value
        fmt = self.get_parameter('pixel_format').value
        self.frame_id = self.get_parameter('frame_id').value
        fps_limit = self.get_parameter('fps_limit').value
        self.min_interval = 1.0 / fps_limit if fps_limit > 0 else 0.0
        self.stream_timeout = float(
            self.get_parameter('stream_timeout').value)
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval').value)

        self.dev, self.w, self.h, self.fmt = dev, w, h, fmt
        self.cam = None
        self.streaming = False
        self.last_frame_t = None
        self.awaiting_first_frame = False
        self.last_reopen_t = 0.0
        self.open_attempts = 0
        self.reconnects = 0
        self.last_error = ''
        self._open_once(first=True)

        # 图像是传感器流：与 qos.py 的约定一致，用 BEST_EFFORT。
        # 默认 RELIABLE 会让慢订阅者反压采集端。
        self.pub = self.create_publisher(
            CompressedImage, 'image_raw/compressed', Q.sensor_qos())
        self.count = 0
        self.last_pub = 0.0
        self.last_seq = -1
        self.gaps = 0
        self.t_start = time.monotonic()

        self.create_timer(0.005, self.poll)
        self.create_timer(10.0, self.log_stats)

    # ---------- 设备开关 ----------

    def _open_once(self, first=False):
        """打开设备。失败时记录原因并保持关闭，不抛异常。"""
        self.open_attempts += 1
        self.last_reopen_t = time.monotonic()
        cam = V4L2Camera(self.dev, self.w, self.h, self.fmt)
        try:
            actual = cam.open()
        except Exception as exc:
            # V4L2Camera.open() 已自行释放部分初始化的资源
            self.last_error = str(exc)
            self.cam = None
            self.streaming = False
            self.get_logger().error(
                '摄像头打开失败: %s (%s)' % (self.dev, exc))
            return False
        self.cam = cam
        self.streaming = True
        # 首帧超时从这里起算：设备打开成功但一直没有帧，同样算断流
        self.last_frame_t = time.monotonic()
        self.awaiting_first_frame = True
        self.get_logger().info(
            'camera_node %s: %s %dx%d 实际格式=%s sizeimage=%d 缓冲=%d'
            % ('启动' if first else '重新连接', self.dev,
               cam.width, cam.height, fourcc_str(actual),
               cam.sizeimage, cam.num_buffers))
        if actual != fourcc(self.fmt):
            self.get_logger().error(
                '格式协商失败: 请求 %s 实际 %s'
                % (self.fmt, fourcc_str(actual)))
        if not first:
            self.reconnects += 1
        return True

    def _close_once(self):
        """关闭设备并标记断流。

        同时把节流起点设为当前时刻：断流之后必须等满
        reconnect_interval 才能重开，避免每轮都去敲设备。
        """
        if self.cam is not None:
            try:
                self.cam.close()
            except Exception:
                pass
        self.cam = None
        self.streaming = False
        self.last_reopen_t = time.monotonic()

    def _maybe_reopen(self, now):
        """按节流间隔尝试重开；未到时间则什么都不做。"""
        if now - self.last_reopen_t < self.reconnect_interval:
            return False
        return self._open_once()

    def poll(self):
        """取帧并发布。断流时先尝试重连，取到新帧才发布。"""
        now = time.monotonic()

        if not self.streaming:
            self._maybe_reopen(now)
            return

        got = self.cam.grab(timeout=0.05)
        if got is None:
            # 首帧超时与后续断流用同一套判定：
            # 设备打开成功却一直不出帧，也算断流，不能一直干等。
            if (self.last_frame_t is not None
                    and now - self.last_frame_t > self.stream_timeout):
                if self.awaiting_first_frame:
                    self.get_logger().error(
                        '摄像头打开后 %.1fs 未收到首帧，判定断流并关闭设备'
                        % self.stream_timeout)
                else:
                    self.get_logger().error(
                        '摄像头超过 %.1fs 无帧，判定断流并关闭设备'
                        % self.stream_timeout)
                self._close_once()
            return

        if self.awaiting_first_frame:
            self.awaiting_first_frame = False
            self.get_logger().info('摄像头收到首帧')
        self.last_frame_t = now
        data, seq = got
        if self.last_seq >= 0 and seq != self.last_seq + 1:
            self.gaps += 1
        self.last_seq = seq
        self.count += 1

        if self.min_interval and (now - self.last_pub) < self.min_interval:
            return
        self.last_pub = now

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = data
        self.pub.publish(msg)

    def log_stats(self):
        """输出统计。断流期间刷屏无意义，只报一次状态变化。"""
        dt = max(time.monotonic() - self.t_start, 1e-6)
        self.get_logger().info(
            '摄像头统计: 采集=%d 平均%.2ffps 序号跳变=%d 状态=%s '
            '重连=%d 打开尝试=%d'
            % (self.count, self.count / dt, self.gaps,
               'streaming' if self.streaming else 'down',
               self.reconnects, self.open_attempts))

    def destroy_node(self):
        """释放摄像头。"""
        self._close_once()
        super().destroy_node()


def main(args=None):
    """启动摄像头节点。SIGTERM 也按正常收尾处理。"""
    import signal

    def on_term(signum, frame):
        """收到 SIGTERM 时抛出，退出 spin。"""
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, on_term)

    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
