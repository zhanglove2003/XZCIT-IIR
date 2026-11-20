"""QoS 现场检查脚本的规格测试（第五次、第六次审核阻断 4）。

只测纯函数与规格表：真实 ROS 查询需要目标机。
用假的 profile / info 对象模拟库返回的情形：
  1. 完全符合规格 -> 端点全过，链路记为完全验证；
  2. 任一字段不符（含订阅端 depth）-> 对应项 FAIL；
  3. 端点缺失 -> FAIL，且链路结论必须是"未验证"，不许报通过；
  4. 库没给出字段 / SYSTEM_DEFAULT / 未知策略 -> UNKNOWN，不当作通过；
  5. 兼容性：VOLATILE 发布配 TRANSIENT_LOCAL 订阅、可靠性不兼容判 ERROR。
"""

import ast
import importlib.util
import pathlib
import sys
import types

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCRIPT = ROOT / 'scripts' / 'qos_integration_check.py'
QOS_MOD = ROOT / 'opi_robot' / 'qos.py'


class _Enum:
    """扮演 rclpy 的枚举值。"""

    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return self.name


def _profile(reliability='BEST_EFFORT', durability='VOLATILE',
             history='KEEP_LAST', depth=10):
    """造一个 QoSProfile 替身。传 _Enum 实例可模拟未知策略。"""
    return types.SimpleNamespace(
        reliability=_Enum(reliability) if isinstance(reliability, str)
        else reliability,
        durability=_Enum(durability) if isinstance(durability, str)
        else durability,
        history=_Enum(history) if isinstance(history, str)
        else history,
        depth=depth)


class _UnknownPolicy:
    """认不出的枚举值。"""

    def __repr__(self):
        return '<unknown policy>'


def _QoSProfile(depth=10):
    """替身 QoSProfile：默认值与 rclpy.qos.QoSProfile 一致（可靠 + 易失）。"""
    return types.SimpleNamespace(depth=depth,
                                 reliability=_Enum('RELIABLE'),
                                 durability=_Enum('VOLATILE'),
                                 history=_Enum('KEEP_LAST'))


class _Info:
    def __init__(self, profile):
        self.qos_profile = profile


class _FakeNode:
    def __init__(self, pubs, subs):
        self._pubs = pubs
        self._subs = subs

    def get_publishers_info_by_topic(self, _topic):
        return self._pubs

    def get_subscriptions_info_by_topic(self, _topic):
        return self._subs

    def destroy_node(self):
        return None


@pytest.fixture
def Q():
    """加载脚本模块，替掉 rclpy 与 sensor_msgs。"""
    rclpy = types.ModuleType('rclpy')

    class _Node:
        def __init__(self, *a, **k):
            pass

    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = _Node
    exec_mod = types.ModuleType('rclpy.executors')
    exec_mod.SingleThreadedExecutor = object

    qos_mod = types.ModuleType('rclpy.qos')
    qos_mod.QoSProfile = _QoSProfile
    qos_mod.ReliabilityPolicy = types.SimpleNamespace(
        RELIABLE=_Enum('RELIABLE'), BEST_EFFORT=_Enum('BEST_EFFORT'),
        SYSTEM_DEFAULT=_Enum('SYSTEM_DEFAULT'))
    qos_mod.DurabilityPolicy = types.SimpleNamespace(
        VOLATILE=_Enum('VOLATILE'),
        TRANSIENT_LOCAL=_Enum('TRANSIENT_LOCAL'),
        SYSTEM_DEFAULT=_Enum('SYSTEM_DEFAULT'))
    qos_mod.HistoryPolicy = types.SimpleNamespace(
        KEEP_LAST=_Enum('KEEP_LAST'), KEEP_ALL=_Enum('KEEP_ALL'),
        SYSTEM_DEFAULT=_Enum('SYSTEM_DEFAULT'))
    qos_mod.qos_profile_sensor_data = None
    # 默认不提供 qos_check_compatible：走本地兜底规则，便于测未知策略
    rclpy.qos = qos_mod

    sensor_mod = types.ModuleType('sensor_msgs.msg')
    sensor_mod.LaserScan = object
    sensor_mod.CompressedImage = object
    sensor_msgs = types.ModuleType('sensor_msgs')

    stubs = {
        'rclpy': rclpy, 'rclpy.node': node_mod,
        'rclpy.executors': exec_mod, 'rclpy.qos': qos_mod,
        'sensor_msgs': sensor_msgs, 'sensor_msgs.msg': sensor_mod,
    }
    saved = {k: sys.modules.get(k) for k in stubs}
    sys.modules.update(stubs)
    rclpy.create_node = _FakeNode
    try:
        spec = importlib.util.spec_from_file_location(
            'qos_check_under_test', SCRIPT)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v

    mod.RESULTS.clear()
    mod._CONTEXT = object()
    mod._FakeNode = _FakeNode
    mod._Info = _Info
    mod._UnknownPolicy = _UnknownPolicy

    def _profile_for(reliability='BEST_EFFORT', durability='VOLATILE',
                     history='KEEP_LAST', depth=10):
        """用**脚本自己的枚举对象**造 profile。

        脚本通过 ``ReliabilityPolicy.BEST_EFFORT`` 比对，所以替身 profile
        里必须放同一批对象；用本文件的 _Enum 会永远不相等，测试就成了
        假通过。
        """
        def pick(field, name):
            table = mod._ENUM_NAMES[field]
            for value, label in table.items():
                if label == name:
                    return value
            return _UnknownPolicy()
        return types.SimpleNamespace(
            reliability=pick('reliability', reliability),
            durability=pick('durability', durability),
            history=pick('history', history),
            depth=depth)

    mod._profile = _profile_for
    yield mod


def _record_map(mod):
    return {name: ok for name, ok, _ in mod.RESULTS}


def _use(mod, pubs, subs):
    """让 create_node 返回预设端点集的假节点。"""
    mod.rclpy.create_node = lambda *a, **k: mod._FakeNode(pubs, subs)


SENSOR = dict(reliability='BEST_EFFORT', durability='VOLATILE',
              history='KEEP_LAST', depth=10)
STATUS = dict(SENSOR, reliability='RELIABLE')


class TestSpecTable:
    def test_every_topic_has_full_publisher_spec(self, Q):
        for topic, spec in Q.SPECS.items():
            for field in ('reliability', 'durability', 'history', 'depth'):
                assert field in spec['publisher'], (topic, field)

    def test_every_topic_has_full_subscriber_spec(self, Q):
        """第六次审核：订阅端缺规格会让 depth 等字段被静默跳过。"""
        for topic, spec in Q.SPECS.items():
            for field in ('reliability', 'durability', 'history', 'depth'):
                assert field in spec['subscriber'], (topic, field)

    def test_both_ends_are_required(self, Q):
        for topic, spec in Q.SPECS.items():
            assert spec['publishers_endpoints'] == 'require', topic
            assert spec['subscriber_endpoints'] == 'require', topic

    def test_specs_match_qos_module(self, Q):
        """规格必须与 opi_robot/qos.py 一致，防止两处漂移。"""
        spec = importlib.util.spec_from_file_location('qos_module', QOS_MOD)
        q = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(q)
        for topic, entry in Q.SPECS.items():
            want = STATUS if topic == '/diagnostics' else SENSOR
            for end in ('publisher', 'subscriber'):
                assert entry[end]['reliability'] == want['reliability'], \
                    (topic, end)
                assert entry[end]['depth'] == want['depth'], (topic, end)
            assert entry['publisher']['durability'] == \
                q.SENSOR_QOS_SPEC['durability'], topic
            assert entry['publisher']['history'] == \
                q.SENSOR_QOS_SPEC['history'], topic


class TestMissingEndpoint:
    def test_no_publisher_fails(self, Q):
        _use(Q, [], [])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is False
        assert summary['link'] is False
        assert _record_map(Q)['/scan 发布端存在'] is False

    def test_required_subscriber_missing_fails(self, Q):
        """只有发布端时，链路结论必须是未验证，不能报告通过。"""
        _use(Q, [Q._Info(Q._profile())], [])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is True, Q.RESULTS
        assert summary['link'] is False, '没有订阅者时不能报告链路通过'
        assert _record_map(Q)['/scan 订阅端存在'] is False
        assert _record_map(Q)['/scan 链路完全验证'] is False


class TestFieldChecks:
    def test_all_fields_match_passes(self, Q):
        _use(Q, [Q._Info(Q._profile())], [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is True, Q.RESULTS
        assert summary['link'] is True, Q.RESULTS
        for name, good in _record_map(Q).items():
            assert good, name

    def test_wrong_reliability_fails(self, Q):
        _use(Q, [Q._Info(Q._profile(reliability='RELIABLE'))],
             [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is False

    def test_wrong_durability_fails(self, Q):
        _use(Q, [Q._Info(Q._profile(durability='TRANSIENT_LOCAL'))],
             [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is False

    def test_wrong_history_fails(self, Q):
        _use(Q, [Q._Info(Q._profile(history='KEEP_ALL'))],
             [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is False

    def test_wrong_depth_fails(self, Q):
        _use(Q, [Q._Info(Q._profile(depth=999))], [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is False
        assert _record_map(Q)['/scan 发布端 depth'] is False

    def test_wrong_subscriber_depth_fails(self, Q):
        """第六次审核：订阅端 depth 以前根本没查。"""
        _use(Q, [Q._Info(Q._profile())], [Q._Info(Q._profile(depth=42))])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is False
        assert _record_map(Q)['/scan 订阅端 depth'] is False

    def test_image_and_diagnostics_depth_are_checked(self, Q):
        for topic in ('/image_raw/compressed', '/diagnostics'):
            Q.RESULTS.clear()
            want = STATUS if topic == '/diagnostics' else SENSOR
            _use(Q, [Q._Info(Q._profile(**dict(want, depth=999)))],
                 [Q._Info(Q._profile(**want))])
            summary = Q.check_topic_qos(topic, Q.SPECS[topic])
            assert summary['spec'] is False, topic
            assert _record_map(Q)['%s 发布端 depth' % topic] is False, topic

    def test_system_default_is_unverifiable_not_a_pass(self, Q):
        """图发现只给 SYSTEM_DEFAULT：标"无法验证"，不算通过也不算错配。"""
        _use(Q, [Q._Info(Q._profile(reliability='SYSTEM_DEFAULT'))],
             [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is False, '无法验证不能算通过'
        assert 'reliability' in summary['unknown'], summary
        rec = _record_map(Q)
        assert rec['/scan 发布端 reliability'] is False

    def test_missing_field_object_is_unverifiable(self, Q):
        _use(Q, [Q._Info(types.SimpleNamespace())], [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is False
        assert set(summary['unknown']) >= {'reliability', 'durability',
                                           'history', 'depth'}, summary
        assert Q._qos_field(types.SimpleNamespace(), 'reliability') == \
            Q.UNKNOWN
        assert Q._qos_field(None, 'depth') == Q.UNKNOWN

    def test_unknown_policy_is_unverifiable(self, Q):
        profile = Q._profile()
        profile.durability = Q._UnknownPolicy()
        assert Q._qos_field(profile, 'durability') == Q.UNKNOWN
        _use(Q, [Q._Info(profile)], [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is False
        assert summary['unknown'] == ['durability'], summary

    def test_humble_depth_zero_is_unverifiable(self, Q):
        """第七次审核现场现象：图发现 depth 返回 0。"""
        _use(Q, [Q._Info(Q._profile(depth=0))], [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is False, '读到 0 不能算通过'
        assert 'depth' in summary['unknown'], summary
        assert summary['link'] is False, '字段无法验证时链路不能报通过'

    def test_humble_history_unknown_is_unverifiable(self, Q):
        _use(Q, [Q._Info(Q._profile(history='UNKNOWN'))],
             [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is False
        assert 'history' in summary['unknown'], summary


class TestCompatibility:
    def test_volatile_publisher_transient_local_subscriber_is_error(self, Q):
        """第六次审核给出的具体反例。"""
        pub = Q._profile(durability='VOLATILE')
        sub = Q._profile(durability='TRANSIENT_LOCAL')
        state, why = Q.compatibility(pub, sub)
        assert state == 'ERROR', (state, why)
        assert 'TRANSIENT_LOCAL' in why

    def test_reliability_mismatch_is_error(self, Q):
        state, _ = Q.compatibility(Q._profile(reliability='BEST_EFFORT'),
                                   Q._profile(reliability='RELIABLE'))
        assert state == 'ERROR'

    def test_normal_combo_is_ok(self, Q):
        state, _ = Q.compatibility(Q._profile(), Q._profile())
        assert state == 'OK'

    def test_unknown_policy_is_unknown_not_ok(self, Q):
        pub = Q._profile()
        pub.durability = Q._UnknownPolicy()
        state, why = Q.compatibility(pub, Q._profile())
        assert state == 'UNKNOWN', (state, why)

    def test_uses_ros_checker_when_available(self, Q):
        """有 qos_check_compatible 时必须优先用 ROS 自己的判定。"""
        calls = []

        class _Result:
            compatibility = types.SimpleNamespace(name='ERROR')
            reason = 'Volatile publisher and transient local subscription'

        def _checker(pub, sub):
            calls.append((pub, sub))
            return _Result()

        Q.rclpy.qos.qos_check_compatible = _checker
        state, why = Q.compatibility(Q._profile(), Q._profile())
        assert state == 'ERROR'
        assert calls, '没有调用 qos_check_compatible'
        assert 'Volatile publisher' in why

    def test_incompatible_pair_flag_check_fails(self, Q):
        """逐项字段各自合格但组合不兼容时，兼容性项必须 FAIL。"""
        pub = Q._profile(reliability='BEST_EFFORT', durability='VOLATILE',
                         depth=999)
        sub = Q._profile(reliability='BEST_EFFORT',
                         durability='TRANSIENT_LOCAL')
        _use(Q, [Q._Info(pub)], [Q._Info(sub)])
        topic = '/image_raw/compressed'
        summary = Q.check_topic_qos(topic, Q.SPECS[topic])
        assert summary['spec'] is False
        assert summary['link'] is False
        assert _record_map(Q)['%s 端点兼容性' % topic] is False

    def test_compatible_pair_reports_link_ok(self, Q):
        _use(Q, [Q._Info(Q._profile())], [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['link'] is True, Q.RESULTS
        assert _record_map(Q)['/scan 端点兼容性'] is True


class TestScriptStructure:
    def test_executor_is_bound_to_context(self):
        text = SCRIPT.read_text()
        assert 'SingleThreadedExecutor(context=self.context)' in text
        for line in text.splitlines():
            code = line.split('#', 1)[0]
            assert 'rclpy.spin_once' not in code, line

    def test_shutdown_uses_same_context(self):
        assert 'rclpy.shutdown(context=context)' in SCRIPT.read_text()

    def test_reports_unverified_link_separately(self):
        assert '链路完全验证' in SCRIPT.read_text()

    def test_main_reports_unverified_topics(self):
        text = SCRIPT.read_text()
        assert "endswith('链路完全验证')" in text
        assert '不能当作通过' in text


class TestRealDiagScriptStructure:
    """第六次审核阻断 1 的静态回归：诊断脚本不得再用 __new__ 造节点。"""

    def _text(self):
        return (ROOT / 'scripts' / 'real_ros_diag_check.py').read_text()

    def test_no_new_object_for_fusion_node(self):
        """只查代码，不查说明文字：docstring 里提到旧写法是正常的。"""
        tree = ast.parse(self._text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Attribute) and node.attr == '__new__':
                raise AssertionError('第 %d 行仍在用 __new__' % node.lineno)

    def test_uses_real_construction_with_context(self):
        assert 'FN.FusionNode(context=ctx)' in self._text()

    def test_asserts_lost_seq_latched(self):
        assert 'lost_seq' in self._text()

    def test_stops_timers_and_destroys(self):
        text = self._text()
        assert 'halt_timers()' in text
        assert 'destroy_node()' in text
        assert 'rclpy.shutdown(context=ctx)' in text


class TestScriptRunsAgainstFakeRos:
    """第七次审核阻断 1：端到端跑脚本主流程。

    只在 AST 里查"函数存在"不够。这里用假 ROS 让脚本真的执行一遍，
    证明：融合检查确实被调用、六项都在，删掉调用或让融合检查抛错
    时脚本不能报通过。
    """

    @staticmethod
    def _run(monkeypatch, capsys, drop_call=False, break_fusion=False,
             drop_name=None, duplicate=None):
        import os
        # 脚本要求隔离域，先给环境
        monkeypatch.setenv('OPI_TEST_ISOLATED', '1')
        monkeypatch.setenv('ROS_DOMAIN_ID', '231')
        diag_path = ROOT / 'scripts' / 'real_ros_diag_check.py'
        text = diag_path.read_text()
        if drop_call:
            text = text.replace(
                "    check('fusion_decodes_real_diagnostics', fusion_decode)"
                "\n", '')
            assert "check('fusion_decodes_real_diagnostics'" not in text
        if drop_name is not None:
            # 删掉任意一条必需检查的调用行（整行按名字删除）
            kept = [ln for ln in text.splitlines(True)
                    if "check('%s'" % drop_name not in ln]
            assert len(kept) == len(text.splitlines(True)) - 1, drop_name
            text = ''.join(kept)
        if duplicate is not None:
            # 复制一条必需检查的调用行，制造重复项
            lines = text.splitlines(True)
            for i, ln in enumerate(lines):
                if "check('%s'" % duplicate in ln:
                    lines.insert(i + 1, ln)
                    break
            else:
                raise AssertionError('找不到调用：%s' % duplicate)
            text = ''.join(lines)
        if break_fusion:
            text = text.replace(
                "    def fusion_decode():",
                "    def fusion_decode():\n        raise RuntimeError('boom')",
                1)
        mod = types.ModuleType('diag_under_test')
        mod.__file__ = str(diag_path)
        code = compile(text, str(diag_path), 'exec')
        sys.path.insert(0, str(ROOT / 'scripts'))
        try:
            exec(code, mod.__dict__)
        finally:
            sys.path.pop(0)
        rc = mod.main([])
        out = capsys.readouterr().out
        return rc, out, mod

    def test_all_six_checks_run_and_pass(self, monkeypatch, capsys):
        rc, out, mod = self._run(monkeypatch, capsys)
        assert 'PASS fusion_decodes_real_diagnostics' in out, out
        assert '共 6 项，通过 6，失败 0' in out, out
        assert rc == 0, out

    def test_removing_the_call_cannot_pass(self, monkeypatch, capsys):
        rc, out, _mod = self._run(monkeypatch, capsys, drop_call=True)
        # 这一项没有被执行，所以不会有 PASS/FAIL 行
        assert 'PASS fusion_decodes_real_diagnostics' not in out, out
        assert 'FAIL fusion_decodes_real_diagnostics' not in out, out
        assert '共 5 项' in out, out
        # 第八次审核阻断 2：少跑一项必须退出非零，不能只看条数
        assert rc == 1, out
        assert '缺少必需检查' in out, out
        assert 'fusion_decodes_real_diagnostics' in out.split('缺少必需检查')[1]

    def test_missing_other_required_check_fails(self, monkeypatch, capsys):
        rc, out, _mod = self._run(
            monkeypatch, capsys, drop_name='roundtrip_all_levels')
        assert rc == 1, out
        assert '缺少必需检查' in out, out
        assert 'roundtrip_all_levels' in out, out
        assert '共 5 项，通过 5，失败 0' in out, out

    def test_duplicate_check_fails(self, monkeypatch, capsys):
        rc, out, _mod = self._run(
            monkeypatch, capsys, duplicate='roundtrip_all_levels')
        assert rc == 1, out
        assert '重复执行' in out, out
        assert 'roundtrip_all_levels' in out.split('重复执行')[1], out
        assert '共 7 项，通过 7，失败 0' in out, out

    def test_required_list_is_explicit(self):
        text = (ROOT / 'scripts' / 'real_ros_diag_check.py').read_text()
        tree = ast.parse(text)
        names = None
        for node in tree.body:
            if isinstance(node, ast.Assign) and \
                    getattr(node.targets[0], 'id', '') == 'REQUIRED_CHECKS':
                names = [c.value for c in node.value.elts]
        assert names is not None, '缺少 REQUIRED_CHECKS'
        assert len(names) == 6, names
        for name in names:
            assert "check('%s'" % name in text, name

    def test_fusion_failure_makes_script_fail(self, monkeypatch, capsys):
        rc, out, _mod = self._run(monkeypatch, capsys, break_fusion=True)
        assert 'FAIL fusion_decodes_real_diagnostics' in out, out
        assert '共 6 项，通过 5，失败 1' in out, out
        assert rc == 1, out

    def test_laserscan_is_imported(self):
        text = (ROOT / 'scripts' / 'real_ros_diag_check.py').read_text()
        assert 'from sensor_msgs.msg import LaserScan' in text


class TestCompatReturnFormats:
    """第七次审核阻断 2：Humble 的 qos_check_compatible 返回 tuple。"""

    def test_tuple_ok(self, Q):
        Q.rclpy.qos.qos_check_compatible = lambda p, s: (_Enum('OK'), 'fine')
        state, why = Q.compatibility(Q._profile(), Q._profile())
        assert state == 'OK', (state, why)

    def test_tuple_warn(self, Q):
        Q.rclpy.qos.qos_check_compatible = lambda p, s: (
            _Enum('WARN'), 'deprecated policy')
        state, _ = Q.compatibility(Q._profile(), Q._profile())
        assert state == 'WARN'

    def test_tuple_error(self, Q):
        Q.rclpy.qos.qos_check_compatible = lambda p, s: (
            _Enum('ERROR'), 'incompatible')
        state, _ = Q.compatibility(Q._profile(), Q._profile())
        assert state == 'ERROR'

    def test_tuple_with_unknown_enum_falls_back(self, Q):
        """枚举名认不出来时要回落本地规则，不能直接报通过。"""
        Q.rclpy.qos.qos_check_compatible = lambda p, s: (
            _Enum('WHATEVER'), 'x')
        state, why = Q.compatibility(
            Q._profile(durability='VOLATILE'),
            Q._profile(durability='TRANSIENT_LOCAL'))
        assert state == 'ERROR', (state, why)
        assert '本地规则' in why

    def test_exception_falls_back_to_local_rules(self, Q):
        def boom(p, s):
            raise RuntimeError('no such topic')
        Q.rclpy.qos.qos_check_compatible = boom
        state, why = Q.compatibility(
            Q._profile(reliability='BEST_EFFORT'),
            Q._profile(reliability='RELIABLE'))
        assert state == 'ERROR', (state, why)
        assert '调用失败' in why

    def test_object_form_still_supported(self, Q):
        class _R:
            compatibility = _Enum('ERROR')
            reason = 'obj form'
        Q.rclpy.qos.qos_check_compatible = lambda p, s: _R()
        state, _ = Q.compatibility(Q._profile(), Q._profile())
        assert state == 'ERROR'

    def test_bare_enum_supported(self, Q):
        Q.rclpy.qos.qos_check_compatible = lambda p, s: _Enum('OK')
        state, _ = Q.compatibility(Q._profile(), Q._profile())
        assert state == 'OK'

    def test_no_checker_uses_local_rules(self, Q):
        assert not hasattr(Q.rclpy.qos, 'qos_check_compatible')
        state, why = Q.compatibility(
            Q._profile(durability='VOLATILE'),
            Q._profile(durability='TRANSIENT_LOCAL'))
        assert state == 'ERROR', (state, why)


class TestLocalCompatUnresolved:
    """第八次审核阻断 3：SYSTEM_DEFAULT 不能被本地规则判成兼容。"""

    def test_reliability_system_default_is_unknown(self, Q):
        state, why = Q._local_compat(
            Q._profile(reliability='SYSTEM_DEFAULT'), Q._profile())
        assert state == 'UNKNOWN', (state, why)
        assert 'reliability' in why
        assert 'SYSTEM_DEFAULT' in why

    def test_durability_system_default_is_unknown(self, Q):
        state, why = Q._local_compat(
            Q._profile(durability='SYSTEM_DEFAULT'),
            Q._profile(durability='SYSTEM_DEFAULT'))
        assert state == 'UNKNOWN', (state, why)
        assert 'durability' in why

    def test_unknown_policy_is_unknown(self, Q):
        pub = Q._profile()
        pub.reliability = Q._UnknownPolicy()
        state, why = Q._local_compat(pub, Q._profile())
        assert state == 'UNKNOWN', (state, why)

    def test_known_compatible_still_ok_but_scoped(self, Q):
        state, why = Q._local_compat(Q._profile(), Q._profile())
        assert state == 'OK', (state, why)
        # 不能把局部规则说成完整兼容性证明
        assert '不是全部 DDS 策略' in why, why

    def test_known_incompatible_still_error(self, Q):
        state, _ = Q._local_compat(
            Q._profile(durability='VOLATILE'),
            Q._profile(durability='TRANSIENT_LOCAL'))
        assert state == 'ERROR'

    def test_checker_missing_with_system_default(self, Q):
        Q.rclpy.qos.qos_check_compatible = None
        state, _ = Q.compatibility(
            Q._profile(reliability='SYSTEM_DEFAULT'), Q._profile())
        assert state == 'UNKNOWN', state

    def test_checker_exception_with_system_default(self, Q):
        def boom(p, s):
            raise RuntimeError('boom')
        Q.rclpy.qos.qos_check_compatible = boom
        state, why = Q.compatibility(
            Q._profile(durability='SYSTEM_DEFAULT'), Q._profile())
        assert state == 'UNKNOWN', (state, why)
        assert '调用失败' in why

    def test_checker_unrecognized_result_with_system_default(self, Q):
        Q.rclpy.qos.qos_check_compatible = lambda p, s: (_Enum('NOPE'), 'x')
        state, why = Q.compatibility(
            Q._profile(reliability='SYSTEM_DEFAULT'),
            Q._profile(reliability='SYSTEM_DEFAULT'))
        assert state == 'UNKNOWN', (state, why)
        assert '本地规则' in why

    def test_no_path_returns_ok_for_system_default(self, Q):
        """三条 fallback 路径都必须给 UNKNOWN，不能有任何一条报 OK。"""
        variants = [None,
                    (lambda p, s: (_ for _ in ()).throw(RuntimeError('x'))),
                    (lambda p, s: (_Enum('NOPE'), 'x'))]
        for checker in variants:
            Q.rclpy.qos.qos_check_compatible = checker
            state, _ = Q.compatibility(
                Q._profile(reliability='SYSTEM_DEFAULT'),
                Q._profile(reliability='SYSTEM_DEFAULT'))
            assert state != 'OK', checker

    def test_compat_record_never_ok_for_system_default_endpoints(self, Q):
        """兼容性项在 SYSTEM_DEFAULT 端点下不能记录成 PASS。"""
        pub = Q._profile(reliability='SYSTEM_DEFAULT')
        sub = Q._profile(reliability='SYSTEM_DEFAULT')
        _use(Q, [Q._Info(pub)], [Q._Info(sub)])
        Q.rclpy.qos.qos_check_compatible = None
        topic = '/scan'
        Q.check_topic_qos(topic, dict(Q.SPECS[topic]))
        assert _record_map(Q)['%s 端点兼容性' % topic] is False, Q.RESULTS


class TestFourScenarios:
    """第七次审核要求：四种情景的判定互不混淆。

    完整兼容 -> 端点与链路都通过；
    不兼容   -> 兼容性 FAIL，链路 FAIL；
    缺端点   -> 端点项 FAIL，链路 FAIL；
    字段未知 -> 标"无法验证"，既不通过也不算错配。
    """

    def test_complete_compatible(self, Q):
        _use(Q, [Q._Info(Q._profile())], [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary == {'spec': True, 'link': True, 'unknown': [],
                           'pubs': 1, 'subs': 1}

    def test_incompatible(self, Q):
        _use(Q, [Q._Info(Q._profile(durability='VOLATILE', depth=999))],
             [Q._Info(Q._profile(durability='TRANSIENT_LOCAL'))])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['link'] is False
        assert _record_map(Q)['/scan 端点兼容性'] is False

    def test_missing_endpoint(self, Q):
        _use(Q, [], [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['link'] is False
        assert summary['unknown'] == [], '缺端点不是"字段未知"'

    def test_unknown_field(self, Q):
        _use(Q, [Q._Info(Q._profile(depth=0))], [Q._Info(Q._profile())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['unknown'] == ['depth']
        assert summary['spec'] is False and summary['link'] is False


class TestFourScenarioAssertions:
    """第八次审核阻断 1：四种情景必须分别断言。

    这里从交付脚本里按 AST 取出断言函数本体，用 Humble 真实的记录形态
    喂进去：reliability/durability 可核验、history/depth 读不到。
    情景 A 不能因此失败，情景 C 也要能独立判定"缺订阅端"。
    """

    WANTED = ('UNVERIFIABLE_PREFIX', '_rows', 'entries', 'worst_state',
              'entry', 'unverifiable', 'assert_ok', 'field_state',
              'assert_scenario_a', 'assert_scenario_c',
              'wait_for_endpoint_counts', 'assert_scenario_e')

    @staticmethod
    def _load():
        text = (ROOT / 'scripts' / 'qos_four_scenario_check.py').read_text()
        tree = ast.parse(text)
        picked = []
        for node in tree.body:
            if isinstance(node, ast.FunctionDef) and node.name in \
                    TestFourScenarioAssertions.WANTED:
                picked.append(node)
            elif isinstance(node, ast.Assign):
                name = getattr(node.targets[0], 'id', '')
                if name in TestFourScenarioAssertions.WANTED:
                    picked.append(node)
            elif isinstance(node, (ast.Import, ast.ImportFrom)):
                # 被提取的函数用到 time/types，把对应顶层 import 一并带走
                mods = ([a.name for a in node.names]
                        if isinstance(node, ast.Import) else [node.module])
                if any(m in ('time', 'types') for m in mods if m):
                    picked.append(node)
        assert len(picked) == len(TestFourScenarioAssertions.WANTED) + 2, \
            [n.name for n in picked if hasattr(n, 'name')]
        mod = types.ModuleType('scenario_asserts')
        mod.Q = types.SimpleNamespace(RESULTS=[])
        code = compile(ast.Module(body=picked, type_ignores=[]),
                       '<scenario_asserts>', 'exec')
        exec(code, mod.__dict__)
        return mod

    @staticmethod
    def _humble_records(topic, unknown=('history', 'depth'),
                        compat=('True', 'OK（本地规则）')):
        """按 Humble 图发现的真实形态造记录。"""
        rows = [(topic + ' reliability', True, ''),
                (topic + ' durability', True, ''),
                (topic + ' 发布端存在', True, ''),
                (topic + ' 订阅端存在', True, '')]
        for label in ('发布端', '订阅端'):
            for field in ('reliability', 'durability'):
                rows.append((topic + ' ' + label + ' ' + field, True, ''))
            for field in ('history', 'depth'):
                if field in unknown:
                    rows.append((topic + ' ' + label + ' ' + field, False,
                                 '无法验证：图发现没有提供该字段'))
                else:
                    rows.append((topic + ' ' + label + ' ' + field, True, ''))
        rows.append((topic + ' 端点兼容性', compat[0] == 'True', compat[1]))
        return rows

    def test_scenario_a_passes_when_history_depth_unverified(self):
        mod = self._load()
        topic = '/qos_a'
        rows = self._humble_records(topic)
        summary = {'spec': False, 'link': False,
                   'unknown': ['depth', 'history']}
        unv = mod.assert_scenario_a(topic, summary, rows)
        assert set(unv) == {'发布端/history', '发布端/depth',
                            '订阅端/history', '订阅端/depth'}, unv

    def test_scenario_a_passes_when_graph_provides_all_fields(self):
        mod = self._load()
        topic = '/qos_a'
        rows = self._humble_records(topic, unknown=())
        summary = {'spec': True, 'link': True, 'unknown': []}
        assert mod.assert_scenario_a(topic, summary, rows) == []

    def test_scenario_a_fails_on_real_mismatch(self):
        mod = self._load()
        topic = '/qos_a'
        rows = [(n, False, '实际 VOLATILE，期望 TRANSIENT_LOCAL')
                if n == topic + ' 发布端 durability' else (n, ok, d)
                for n, ok, d in self._humble_records(topic)]
        summary = {'spec': False, 'link': False, 'unknown': []}
        with pytest.raises(AssertionError, match='durability'):
            mod.assert_scenario_a(topic, summary, rows)

    def test_scenario_a_fails_when_compat_misreported(self):
        """字段缺失不能导致把模拟端点判成不兼容。"""
        mod = self._load()
        topic = '/qos_a'
        rows = self._humble_records(
            topic, compat=('False', 'ERROR（本地规则）'))
        summary = {'spec': False, 'link': False,
                   'unknown': ['depth', 'history']}
        with pytest.raises(AssertionError):
            mod.assert_scenario_a(topic, summary, rows)

    def test_scenario_a_rejects_unexpected_unknown_field(self):
        mod = self._load()
        topic = '/qos_a'
        rows = self._humble_records(topic)
        summary = {'spec': False, 'link': False,
                   'unknown': ['depth', 'history', 'reliability']}
        with pytest.raises(AssertionError):
            mod.assert_scenario_a(topic, summary, rows)

    def test_scenario_c_passes_with_missing_subscriber(self):
        mod = self._load()
        topic = '/qos_c'
        rows = [r for r in self._humble_records(topic)
                if not r[0].endswith('订阅端存在')
                and ' 订阅端 ' not in r[0]]
        rows.append((topic + ' 订阅端存在', False, '没有订阅者；链路结论记为未验证'))
        summary = {'spec': False, 'link': False,
                   'unknown': ['depth', 'history']}
        unv = mod.assert_scenario_c(topic, summary, rows)
        assert set(unv) == {'history', 'depth'}, unv

    def test_scenario_c_fails_when_subscriber_exists(self):
        mod = self._load()
        topic = '/qos_c'
        rows = self._humble_records(topic)
        summary = {'spec': True, 'link': True, 'unknown': []}
        with pytest.raises(AssertionError, match='缺订阅端'):
            mod.assert_scenario_c(topic, summary, rows)

    def test_scenario_c_fails_when_link_reported_ok(self):
        mod = self._load()
        topic = '/qos_c'
        rows = [r for r in self._humble_records(topic)
                if ' 订阅端 ' not in r[0]]
        rows.append((topic + ' 订阅端存在', False, '没有订阅者'))
        summary = {'spec': False, 'link': True, 'unknown': []}
        with pytest.raises(AssertionError):
            mod.assert_scenario_c(topic, summary, rows)


class TestMultiEndpointAggregation:
    """第十次审核其他项 2：多发布端/订阅端必须逐端聚合。

    旧 entry() 只取重复标签的第一条：第一端字段未知或合格时，
    第二端明确错误会被漏掉。聚合规则：任一端明确错误 -> bad，
    优先于未验证与通过。
    """

    def _load(self):
        return TestFourScenarioAssertions._load()

    def test_bad_after_ok_is_caught(self):
        mod = self._load()
        rows = [('/t 发布端 durability', True, 'VOLATILE（期望 VOLATILE）'),
                ('/t 发布端 durability', False,
                 '实际 TRANSIENT_LOCAL，期望 VOLATILE')]
        assert mod.worst_state(
            [(ok, d) for _n, ok, d in rows]) == 'bad'
        ok, detail = mod.entry('/t', '发布端 durability', rows)
        assert ok is False
        assert 'TRANSIENT_LOCAL' in detail

    def test_bad_after_unverifiable_is_caught(self):
        """第一端字段未知、第二端明确错误：必须报 bad，不许被未验证掩盖。"""
        mod = self._load()
        rows = [('/t 发布端 durability', False,
                 '无法验证：图发现没有提供该字段'),
                ('/t 发布端 durability', False,
                 '实际 TRANSIENT_LOCAL，期望 VOLATILE')]
        assert mod.worst_state(
            [(ok, d) for _n, ok, d in rows]) == 'bad'

    def test_unverifiable_after_ok(self):
        mod = self._load()
        rows = [('/t 发布端 history', True, 'KEEP_LAST（期望 KEEP_LAST）'),
                ('/t 发布端 history', False,
                 '无法验证：图发现只给了 SYSTEM_DEFAULT')]
        assert mod.worst_state(
            [(ok, d) for _n, ok, d in rows]) == 'unverifiable'

    def test_all_ok_aggregates_ok(self):
        mod = self._load()
        rows = [('/t 发布端 reliability', True, ''), ] * 3
        assert mod.worst_state(
            [(ok, d) for _n, ok, d in rows]) == 'ok'

    def test_field_state_uses_aggregation(self):
        mod = self._load()
        rows = [('/t 发布端 depth', True, '10（期望 10）'),
                ('/t 发布端 depth', False, '999（期望 10）')]
        assert mod.field_state('/t', '发布端', 'depth', rows) == 'bad'

    def test_scenario_a_fails_on_second_publisher_error(self):
        """情景 A：两个发布端，第一端全对、第二端 durability 错 -> 必须失败。"""
        mod = self._load()
        topic = '/qos_e'
        rows = TestFourScenarioAssertions._humble_records(topic, unknown=())
        rows += [(topic + ' 发布端 durability', True, ''),
                 (topic + ' 发布端 durability', False,
                  '实际 TRANSIENT_LOCAL，期望 VOLATILE')]
        summary = {'spec': False, 'link': False, 'unknown': []}
        with pytest.raises(AssertionError, match='durability'):
            mod.assert_scenario_a(topic, summary, rows)

    def test_scenario_a_fails_when_unverifiable_then_bad(self):
        """第一端 history 未验证、第二端 reliability 明确错误：
        reliability 的 assert_ok 必须抓住，不许只看到第一端。"""
        mod = self._load()
        topic = '/qos_e'
        rows = TestFourScenarioAssertions._humble_records(topic)
        rows += [(topic + ' 发布端 reliability', True, ''),
                 (topic + ' 发布端 reliability', False,
                  '实际 RELIABLE，期望 BEST_EFFORT')]
        summary = {'spec': False, 'link': False,
                   'unknown': ['depth', 'history']}
        with pytest.raises(AssertionError, match='RELIABLE'):
            mod.assert_scenario_a(topic, summary, rows)

    def test_scenario_c_aggregates_publisher_fields(self):
        """情景 C：多个发布端里有一个字段错，也必须判失败。"""
        mod = self._load()
        topic = '/qos_c'
        rows = [r for r in TestFourScenarioAssertions._humble_records(topic)
                if not r[0].endswith('订阅端存在') and ' 订阅端 ' not in r[0]]
        rows.append((topic + ' 订阅端存在', False,
                     '没有订阅者；链路结论记为未验证'))
        rows += [(topic + ' 发布端 durability', True, ''),
                 (topic + ' 发布端 durability', False,
                  '实际 TRANSIENT_LOCAL，期望 VOLATILE')]
        summary = {'spec': False, 'link': False, 'unknown': []}
        with pytest.raises(AssertionError, match='durability'):
            mod.assert_scenario_c(topic, summary, rows)


class TestMultiEndpointViaRealCheck:
    """用真实 check_topic_qos 产生的多端点记录喂给情景断言（端到端）。"""

    def test_second_publisher_wrong_durability_fails_scenario_a(self, Q):
        _use(Q, [Q._Info(Q._profile(**SENSOR)),
                 Q._Info(Q._profile(**dict(SENSOR,
                                           durability='TRANSIENT_LOCAL')))],
             [Q._Info(Q._profile(**SENSOR))])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        rows = [r for r in Q.RESULTS if r[0] == '/scan 发布端 durability']
        assert len(rows) == 2, rows
        mod = TestFourScenarioAssertions._load()
        mod.Q = Q
        with pytest.raises(AssertionError, match='durability|TRANSIENT'):
            mod.assert_scenario_a('/scan', summary, Q.RESULTS)

    def test_unverifiable_then_bad_still_caught(self, Q):
        """第一发布端 depth 读不到、第二发布端 durability 明确错：
        旧逻辑只看第一条，depth 记为未验证后 durability 被漏掉。"""
        blind = Q._profile(**SENSOR)
        blind.depth = 0
        _use(Q, [Q._Info(blind),
                 Q._Info(Q._profile(**dict(SENSOR,
                                           durability='TRANSIENT_LOCAL')))],
             [Q._Info(Q._profile(**SENSOR))])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['spec'] is False
        mod = TestFourScenarioAssertions._load()
        mod.Q = Q
        with pytest.raises(AssertionError, match='durability|TRANSIENT'):
            mod.assert_scenario_a('/scan', summary, Q.RESULTS)

    def test_incompatible_second_pair_recorded(self, Q):
        """两个发布端都合格、第二订阅端 RELIABLE：兼容性配对必须含 ERROR，
        聚合后的'端点兼容性'不许判通过。"""
        _use(Q, [Q._Info(Q._profile(**SENSOR)),
                 Q._Info(Q._profile(**SENSOR))],
             [Q._Info(Q._profile(**SENSOR)),
              Q._Info(Q._profile(**STATUS))])
        Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        compat = [r for r in Q.RESULTS if r[0] == '/scan 端点兼容性']
        # 2 发布端 × 2 订阅端 = 4 个配对，全部逐对记录
        assert len(compat) == 4, compat
        assert any(not ok for _n, ok, _d in compat), compat
        assert sum(1 for _n, ok, d in compat if not ok and 'ERROR' in d) >= 1


class TestFourScenarioScript:
    """四情景脚本必须走真实图发现，不能抄近道。"""

    PATH = ROOT / 'scripts' / 'qos_four_scenario_check.py'

    def test_exists_and_compiles(self):
        import py_compile
        py_compile.compile(str(self.PATH), doraise=True)

    def test_calls_check_topic_qos(self):
        assert 'Q.check_topic_qos(' in self.PATH.read_text()

    def test_creates_real_endpoints(self):
        text = self.PATH.read_text()
        assert 'create_publisher' in text
        assert 'create_subscription' in text

    def test_requires_isolated_domain(self):
        assert 'require_isolated_domain' in self.PATH.read_text()

    def test_multi_endpoint_scenario_exists(self):
        """第十次审核其他项 2：必须有多端点情景，逐端聚合。"""
        text = self.PATH.read_text()
        assert '情景E_多端点' in text, text
        assert 'worst_state' in text
        assert 'entries' in text

    def test_does_not_publish_cmd_vel(self):
        for line in self.PATH.read_text().splitlines():
            code = line.split('#', 1)[0]
            assert 'cmd_vel' not in code or '不发布' in line, line

    def test_copy_matches(self):
        a = ROOT.parent / 'scripts' / 'qos_four_scenario_check.py'
        assert a.read_bytes() == self.PATH.read_bytes(), '两份副本不一致'

class TestContractFromRealCheck:
    """第九次审核阻断 1：情景断言必须消费真实返回值。

    旧测试手工造了 `发布端存在` 的 True 记录，而 `check_topic_qos()`
    当时只在端点缺失时记录 False，根本不产生 True 记录，于是真实
    Humble 上情景 A/C 直接抛 `没有记录`。这里全程调用真实函数，
    再把它的记录喂给情景断言，覆盖两端都在、缺发布端、缺订阅端、
    字段未知与不兼容五种情况。
    """

    @staticmethod
    def _scen(Q):
        mod = TestFourScenarioAssertions._load()
        mod.Q = Q
        return mod

    def test_both_ends_present_records_both_true(self, Q):
        _use(Q, [Q._Info(Q._profile(**SENSOR))],
             [Q._Info(Q._profile(**SENSOR))])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        names = _record_map(Q)
        assert names['/scan 发布端存在'] is True
        assert names['/scan 订阅端存在'] is True
        assert summary == {'spec': True, 'link': True, 'unknown': [],
                           'pubs': 1, 'subs': 1}
        assert self._scen(Q).assert_scenario_a('/scan', summary,
                                               Q.RESULTS) == []

    def test_presence_record_carries_the_count(self, Q):
        _use(Q, [Q._Info(Q._profile(**SENSOR)),
                 Q._Info(Q._profile(**SENSOR))],
             [Q._Info(Q._profile(**SENSOR))])
        Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        detail = {name: text for name, _ok, text in Q.RESULTS}
        assert '2' in detail['/scan 发布端存在']
        assert '1' in detail['/scan 订阅端存在']

    def test_unknown_fields_still_pass_scenario_a(self, Q):
        prof = Q._profile(**SENSOR)
        def blind():
            return types.SimpleNamespace(
                reliability=prof.reliability, durability=prof.durability,
                history=_UnknownPolicy(), depth=0)
        _use(Q, [Q._Info(blind())], [Q._Info(blind())])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['unknown'] == ['depth', 'history']
        assert summary['spec'] is False
        assert summary['link'] is False
        unseen = set(self._scen(Q).assert_scenario_a('/scan', summary,
                                                     Q.RESULTS))
        assert unseen == {'发布端/history', '发布端/depth',
                          '订阅端/history', '订阅端/depth'}

    def test_missing_subscriber_passes_scenario_c(self, Q):
        _use(Q, [Q._Info(Q._profile(**SENSOR))], [])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert _record_map(Q)['/scan 订阅端存在'] is False
        self._scen(Q).assert_scenario_c('/scan', summary, Q.RESULTS)

    def test_missing_publisher_is_recorded_and_fails_scenario_a(self, Q):
        _use(Q, [], [Q._Info(Q._profile(**SENSOR))])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert _record_map(Q)['/scan 发布端存在'] is False
        with pytest.raises(AssertionError):
            self._scen(Q).assert_scenario_a('/scan', summary, Q.RESULTS)

    def test_incompatible_pair_is_recorded_and_fails_scenario_a(self, Q):
        _use(Q, [Q._Info(Q._profile(**SENSOR))],
             [Q._Info(Q._profile(**STATUS))])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert _record_map(Q)['/scan 端点兼容性'] is False
        assert summary['link'] is False
        with pytest.raises(AssertionError):
            self._scen(Q).assert_scenario_a('/scan', summary, Q.RESULTS)


class TestScenarioEEndpointCounts:
    """第十一次审核阻断 2：情景 E 必须先确认图发现数量，再做聚合断言。

    旧情景 E 固定等待后只查错误字段：图发现只返回错误的那 1 个发布端和
    1 个订阅端时照样 PASS，却输出"2 发布端 + 2 订阅端"。现在：
      - wait_for_endpoint_counts 限时等待数量达标，超时报实际数量；
      - assert_scenario_e 先断言 pubs==2 / subs==2，再查逐端聚合与
        2×2 兼容性配对。
    """

    def _scen(self, Q):
        mod = TestFourScenarioAssertions._load()
        mod.Q = Q
        return mod

    def _pub_sub(self, Q):
        """两个发布端（1 合格 1 durability 错）+ 两个订阅端（1 合格 1 reliability 错）。"""
        return (
            [Q._Info(Q._profile(**SENSOR)),
             Q._Info(Q._profile(**dict(SENSOR,
                                       durability='TRANSIENT_LOCAL')))],
            [Q._Info(Q._profile(**SENSOR)),
             Q._Info(Q._profile(**dict(SENSOR, reliability='RELIABLE')))],
        )

    def test_wait_returns_when_counts_reached(self):
        mod = self._scen(None)
        seq = iter([(0, 0), (1, 1), (2, 2)])
        got = mod.wait_for_endpoint_counts(lambda: next(seq), 2, 2,
                                           timeout=5.0, interval=0)
        assert got == (2, 2)

    def test_wait_reports_actual_counts_on_timeout(self):
        mod = self._scen(None)
        with pytest.raises(AssertionError, match=r'1/2、订阅端 1/2'):
            mod.wait_for_endpoint_counts(lambda: (1, 1), 2, 2,
                                         timeout=0.05, interval=0)

    def test_wait_reports_partial_pub_on_timeout(self):
        mod = self._scen(None)
        with pytest.raises(AssertionError, match=r'发布端 2/2、订阅端 0/2'):
            mod.wait_for_endpoint_counts(lambda: (2, 0), 2, 2,
                                         timeout=0.05, interval=0)

    def test_full_discovery_passes(self, Q):
        pubs, subs = self._pub_sub(Q)
        _use(Q, pubs, subs)
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['pubs'] == 2 and summary['subs'] == 2
        compat = [r for r in Q.RESULTS if r[0] == '/scan 端点兼容性']
        assert len(compat) == 4, compat
        # 断言不依赖等待：数量已达标即通过
        self._scen(Q).assert_scenario_e('/scan', summary, Q.RESULTS)

    def test_partial_discovery_fails_even_though_errors_are_found(self, Q):
        """只发现错误端点（1+1）：字段聚合照样抓到错误，但数量断言必须失败。

        这就是旧情景 E 的假通过形态：不查数量时，下面这些字段断言全部
        能过，结论却冒充了 2+2。
        """
        _use(Q, [Q._Info(Q._profile(**dict(SENSOR,
                                           durability='TRANSIENT_LOCAL')))],
             [Q._Info(Q._profile(**dict(SENSOR, reliability='RELIABLE')))])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['pubs'] == 1 and summary['subs'] == 1
        mod = self._scen(Q)
        # 旧断言（字段 + 兼容性）在 1+1 下确实全过——证明旧逻辑假通过
        assert mod.field_state('/scan', '发布端', 'durability',
                               Q.RESULTS) == 'bad'
        assert mod.field_state('/scan', '订阅端', 'reliability',
                               Q.RESULTS) == 'bad'
        compat = [r for r in Q.RESULTS if r[0] == '/scan 端点兼容性']
        assert len(compat) == 1 and not compat[0][1], compat
        # 新断言先卡数量
        with pytest.raises(AssertionError, match='发布端发现数'):
            mod.assert_scenario_e('/scan', summary, Q.RESULTS)

    def test_one_subscriber_missing_fails(self, Q):
        pubs, subs = self._pub_sub(Q)
        _use(Q, pubs, subs[:1])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        with pytest.raises(AssertionError, match='订阅端发现数'):
            self._scen(Q).assert_scenario_e('/scan', summary, Q.RESULTS)

    def test_only_ok_endpoints_discovered_fails(self, Q):
        """只发现 2 个合格端点（错误端点没被发现）：数量够但聚合必须抓不到错误 -> 失败。"""
        _use(Q, [Q._Info(Q._profile(**SENSOR))],
             [Q._Info(Q._profile(**SENSOR))])
        summary = Q.check_topic_qos('/scan', Q.SPECS['/scan'])
        assert summary['pubs'] == 1
        mod = self._scen(Q)
        with pytest.raises(AssertionError, match='发布端发现数'):
            mod.assert_scenario_e('/scan', summary, Q.RESULTS)

    def test_scenario_e_script_contains_count_gate(self):
        text = (ROOT / 'scripts' / 'qos_four_scenario_check.py').read_text()
        assert 'wait_for_endpoint_counts(poll, 2, 2)' in text
        assert "assert_scenario_e(topic, r)" in text
