"""B4 入口不得在导入或默认静检时发送非零命令。"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/b4_direction_test.py'


@pytest.fixture
def entry():
    spec = importlib.util.spec_from_file_location('b4_direction_entry', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_each_run_uses_a_new_session(entry, monkeypatch):
    ticks = iter([10001, 10002])
    monkeypatch.setattr(entry.time, 'monotonic_ns', lambda: next(ticks))
    assert entry.new_session() != entry.new_session()


def test_missing_subscribers_prevent_authorize(entry, monkeypatch):
    sent = []
    probe = entry.B4Test.__new__(entry.B4Test)
    probe.error = ''
    probe.license_pub = SimpleNamespace(get_subscription_count=lambda: 0)
    probe.auth_pub = SimpleNamespace(get_subscription_count=lambda: 1,
                                     publish=lambda msg: sent.append(msg))
    probe.rearm_pub = SimpleNamespace(get_subscription_count=lambda: 1,
                                      publish=lambda msg: sent.append(msg))
    probe.cmd_pub = SimpleNamespace(get_subscription_count=lambda: 1)
    ticks = iter([0.0, 4.0])
    monkeypatch.setattr(entry.time, 'monotonic', lambda: next(ticks))
    with pytest.raises(RuntimeError, match='DDS 发现超时'):
        probe.prepare()
    assert sent == []


def test_default_check_does_not_run_motion_phases(entry):
    calls = []
    probe = entry.B4Test.__new__(entry.B4Test)
    probe.prepare = lambda: calls.append('prepare')
    probe.drive = lambda *a: calls.append(('drive', a))
    probe.finish = lambda: calls.append('finish')
    probe.run(drive=False)
    assert calls == ['prepare', 'finish']


def test_cleanup_runs_when_prepare_fails(entry):
    calls = []
    probe = entry.B4Test.__new__(entry.B4Test)
    def fail():
        raise RuntimeError('其他许可会话仍活动')
    probe.prepare = fail
    probe.finish = lambda: calls.append('finish')
    with pytest.raises(RuntimeError, match='其他许可会话'):
        probe.run(drive=False)
    assert calls == ['finish']


def test_foreign_license_aborts_without_taking_over(entry):
    import json
    probe = entry.B4Test.__new__(entry.B4Test)
    probe.session = 123
    probe.error = ''
    probe.on_license(SimpleNamespace(data=json.dumps({'session': 456})))
    assert '其他许可会话仍活动' in probe.error


def test_finish_publishes_zero_and_revokes_auth(entry, monkeypatch):
    sent = []
    probe = entry.B4Test.__new__(entry.B4Test)
    probe.licensing = True
    probe.Twist = lambda: SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0),
                                         angular=SimpleNamespace(z=0.0))
    probe.Bool = lambda data: SimpleNamespace(data=data)
    probe.cmd_pub = SimpleNamespace(publish=lambda m: sent.append(('cmd', m)))
    probe.auth_pub = SimpleNamespace(publish=lambda m: sent.append(('auth', m)))
    ticks = iter([0.0, 1.0])
    monkeypatch.setattr(entry.time, 'monotonic', lambda: next(ticks))
    probe.finish()
    assert probe.licensing is False
    assert sent[0][0] == 'cmd'
    assert (sent[0][1].linear.x, sent[0][1].angular.z) == (0.0, 0.0)
    assert sent[1][0] == 'auth' and sent[1][1].data is False


def test_cli_defaults_to_static_check(entry, monkeypatch):
    calls = []
    node = SimpleNamespace(destroy_node=lambda: calls.append('destroy'))
    import rclpy
    monkeypatch.setattr(rclpy, 'create_node', lambda name: node)
    monkeypatch.setattr(entry, 'B4Test', lambda n: SimpleNamespace(
        run=lambda drive: calls.append(('drive', drive))))
    entry.main([])
    assert calls == [('drive', False), 'destroy']
