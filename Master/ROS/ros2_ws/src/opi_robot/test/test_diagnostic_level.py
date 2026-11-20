"""诊断等级的类型转换测试。

真实 ROS 2 Humble 的 `DiagnosticStatus.level` 是单字节 byte 字段，
赋值整数会在序列化时抛 AssertionError。伪 ROS 也按这个规则检查。

本文件覆盖三个位置：发布端编码、融合节点解码、网页解析。
"""

import fake_ros
import pytest

from opi_robot import diagnostic_level as DL
from opi_robot.fusion_node import failclosed as F
from opi_robot.fusion_node import fusion_node as FN
from opi_robot.lidar_node import health as H
from opi_robot.web_node import sensors as S


class TestEncode:
    def test_returns_single_byte(self):
        for lv in (0, 1, 2, 3):
            out = DL.encode_level(lv)
            assert isinstance(out, bytes)
            assert len(out) == 1
            assert out[0] == lv

    def test_accepts_bytes(self):
        assert DL.encode_level(b'\x02') == b'\x02'
        assert DL.encode_level(bytearray(b'\x01')) == b'\x01'

    def test_rejects_bad_length(self):
        with pytest.raises(DL.LevelError):
            DL.encode_level(b'\x00\x01')

    def test_rejects_out_of_range(self):
        with pytest.raises(DL.LevelError):
            DL.encode_level(256)
        with pytest.raises(DL.LevelError):
            DL.encode_level(-1)

    def test_rejects_non_number(self):
        with pytest.raises(DL.LevelError):
            DL.encode_level('abc')

    def test_fits_real_message_field(self):
        # 伪 ROS 的 byte 校验与真实 ROS 一致；编码后能直接赋值
        for lv in (0, 1, 2, 3):
            st = fake_ros.DiagnosticStatus()
            st.level = DL.encode_level(lv)
            assert bytes(st.level) == bytes([lv])

    def test_plain_int_assignment_is_rejected(self):
        # 回归保护：真实环境里直接给整数会崩，伪 ROS 必须同样报错
        st = fake_ros.DiagnosticStatus()
        with pytest.raises(AssertionError):
            st.level = 2


class TestDecode:
    def test_int(self):
        for lv in (0, 1, 2, 3):
            assert DL.decode_level(lv) == lv

    def test_bytes(self):
        for lv in (0, 1, 2, 3):
            assert DL.decode_level(bytes([lv])) == lv

    def test_bytearray(self):
        assert DL.decode_level(bytearray(b'\x02')) == 2

    def test_numeric_string(self):
        assert DL.decode_level('2') == 2
        assert DL.decode_level(' 1 ') == 1

    def test_unknown_returns_none(self):
        for bad in (None, '', 'abc', b'', object()):
            assert DL.decode_level(bad) is None

    def test_multi_byte_bytes_is_unknown(self):
        # 真实字段只有 1 字节；多字节不可能是合法等级，按未知处理
        assert DL.decode_level(b'\x00\x02') is None

    def test_negative_int_is_unknown(self):
        assert DL.decode_level(-1) is None
        assert DL.decode_level(-200) is None

    def test_undefined_levels_are_unknown(self):
        for bad in (4, 7, 99, 255):
            assert DL.decode_level(bad) is None, bad

    def test_bool_is_not_a_level(self):
        assert DL.decode_level(True) is None

    def test_float_only_when_integral(self):
        assert DL.decode_level(2.0) == 2
        assert DL.decode_level(2.5) is None

    def test_empty_bytes_is_unknown(self):
        assert DL.decode_level(b'') is None

    def test_known_levels_survive(self):
        for lv in (0, 1, 2, 3):
            assert DL.decode_level(bytes([lv])) == lv
            assert DL.decode_level(lv) == lv


class TestHelpers:
    def test_is_ok(self):
        assert DL.is_ok(b'\x00') is True
        assert DL.is_ok(0) is True
        assert DL.is_ok(b'\x01') is False
        assert DL.is_ok(None) is False

    def test_is_fault_treats_unknown_as_fault(self):
        assert DL.is_fault(b'\x02') is True
        assert DL.is_fault(b'\x00') is False
        assert DL.is_fault(None) is True

    def test_constants_match_health(self):
        assert DL.LEVEL_OK == H.LEVEL_OK
        assert DL.LEVEL_WARN == H.LEVEL_WARN
        assert DL.LEVEL_ERROR == H.LEVEL_ERROR
        assert DL.LEVEL_STALE == H.LEVEL_STALE


def _diag(level, state='healthy'):
    return fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(
            level=level, name='lidar', message=state,
            values=[fake_ros.KeyValue(key='state', value=state)])
    ])


def _raw_diag(raw_level, state='healthy'):
    """构造带任意原始 level 值的诊断消息，用于测试畸形输入。

    绕过 make() 的编码，模拟消息来源给出非预期内容的情况。
    """
    st = fake_ros.DiagnosticStatus(name='lidar', message=state,
                                   values=[fake_ros.KeyValue(key='state',
                                                             value=state)])
    object.__setattr__(st, 'level', raw_level)
    return fake_ros.DiagnosticArray(status=[st])


class TestFusionDecodesRealType:
    """融合节点必须能处理真实字节等级。"""

    def _node(self):
        # 用真实构造而不是 __new__ 手工拼字段。
        # 第五次审核阻断 2 的教训：手造节点会随属性改名/新增而失效，
        # 而且这种失效只有到真实 ROS 上才暴露。
        n = FN.FusionNode()
        n.lidar_diag_timeout = 2.0
        n.fresh.mark(99.5)
        n._now = 100.0
        n.now_s = lambda: n._now
        return n

    def test_ok_bytes_is_accepted(self):
        n = self._node()
        n.on_diagnostics(_diag(DL.encode_level(0)))
        assert n.lidar_status == 0

    def test_error_bytes_blocks(self):
        n = self._node()
        n.on_diagnostics(_diag(DL.encode_level(2), state='disconnected'))
        trusted, why = n.lidar_trusted(n._now)
        assert trusted is False
        assert why.startswith('lidar_')

    def test_unreadable_level_blocks(self):
        n = self._node()
        n.on_diagnostics(_raw_diag(b''))
        trusted, why = n.lidar_trusted(n._now)
        assert trusted is False
        assert why == 'diagnostics_unreadable'


class TestWebDecodesRealType:
    def test_parse_bytes_level(self):
        for lv in (0, 1, 2, 3):
            d = S.parse_diagnostic(_diag(DL.encode_level(lv)))
            assert d['level'] == lv

    def test_level_to_state_handles_none(self):
        assert S.level_to_state(None) == 'unknown'

    def test_parse_does_not_crash_on_malformed(self):
        d = S.parse_diagnostic(_raw_diag(b''))
        # 无法识别时给 None，视图层按未知处理，不抛异常
        assert d is not None
        assert d['level'] is None
        assert S.level_to_state(d['level']) == 'unknown'


class TestLidarPublishesRealType:
    def test_publish_diag_writes_bytes_level(self, monkeypatch):
        import types
        import lidar_fixtures
        from opi_robot.lidar_node import lidar_node as LN

        class FakeSerial:
            def __init__(self, path, baud):
                self.path = path
                self.chunks = bytearray()

            def push(self, data):
                self.chunks.extend(data)

            def read(self, n=4096):
                if not self.chunks:
                    return b''
                data = bytes(self.chunks[:n])
                del self.chunks[:n]
                return data

            def close(self):
                pass

        port = '/dev/serial/by-path/platform-x-usb-0:1.2:1.0-port0'
        monkeypatch.setattr(LN, '_open_serial',
                            lambda path, baud: FakeSerial(path, baud))
        monkeypatch.setattr(LN, 'glob',
                            types.SimpleNamespace(glob=lambda pat: [port]))
        monkeypatch.setattr(LN, 'os', types.SimpleNamespace(
            path=types.SimpleNamespace(exists=lambda p: True)))
        monkeypatch.setattr(LN, 'time', types.SimpleNamespace(
            sleep=lambda s: None, monotonic=lidar_fixtures.monotonic))

        n = LN.LidarNode()
        n.on_poll()
        n.publish_diag()
        st = n.publishers['diagnostics'].messages[-1].status[0]
        # 真实 byte 字段：长度 1 的 bytes，能正常序列化
        assert isinstance(st.level, bytes)
        assert len(st.level) == 1
        # 启动阶段是 connecting，对应 WARN
        assert DL.decode_level(st.level) == H.LEVEL_WARN

class TestRealMessageShape:
    """模拟真实 ROS 2 消息字段，验证脚本真实_ros_diag_check 的核心断言。

    本机没有 rclpy，这里用伪 ROS 复现同样的字段类型约束。
    真实环境仍需在目标机运行 scripts/real_ros_diag_check.py。
    """

    def test_level_field_is_bytes_after_encode(self):
        for lv in (0, 1, 2, 3):
            st = fake_ros.DiagnosticStatus()
            st.name = 'lidar'
            st.level = DL.encode_level(lv)
            assert isinstance(st.level, bytes)
            assert len(st.level) == 1
            assert st.level[0] == lv

    def test_real_int_assignment_raises(self):
        st = fake_ros.DiagnosticStatus()
        st.name = 'lidar'
        with pytest.raises(AssertionError):
            st.level = DL.LEVEL_ERROR

    def test_full_chain_healthy_error_stale(self):
        """发布 -> 融合 -> 网页，三段都用同一份字节等级。"""
        from opi_robot.fusion_node import failclosed as F
        from opi_robot.fusion_node import fusion_node as FN

        def build(level, state):
            st = fake_ros.DiagnosticStatus.make(
                level=level, name='lidar', message=state,
                values=[fake_ros.KeyValue(key='state', value=state)])
            return fake_ros.DiagnosticArray(status=[st])

        n = FN.FusionNode()
        n.lidar_diag_timeout = 2.0
        n.fresh.mark(99.5)
        n._now = 100.0
        n.now_s = lambda: n._now

        n.on_diagnostics(build(DL.LEVEL_OK, 'healthy'))
        assert n.lidar_trusted(n._now)[0] is True

        n.on_diagnostics(build(DL.LEVEL_ERROR, 'disconnected'))
        trusted, why = n.lidar_trusted(n._now)
        assert trusted is False and why.startswith('lidar_')

        n.lidar_status_time = n._now - 10.0
        trusted, why = n.lidar_trusted(n._now)
        assert trusted is False and why == 'diagnostics_stale'

        # 同一份消息给网页解析，结果一致
        d = S.parse_diagnostic(build(DL.LEVEL_ERROR, 'disconnected'))
        assert d['level'] == DL.LEVEL_ERROR
        assert S.level_to_state(d['level']) != 'ok'
