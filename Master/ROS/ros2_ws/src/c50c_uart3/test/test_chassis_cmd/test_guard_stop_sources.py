"""外部停车与 guard 锁停互不覆盖；只使用内存串口和模拟授权。"""
from types import SimpleNamespace

import pytest

from c50c_uart3.chassis_cmd.arbiter import ZeroReason
from c50c_uart3.chassis_cmd.guard_wiring import GuardWiring


@pytest.fixture
def guard_wiring():
    wiring = GuardWiring(
        SimpleNamespace(), port=None, device='test',
        firmware_protocol='baseline', baseline_protocol='baseline', coexist=True)
    wiring.bridge = SimpleNamespace(
        guard=SimpleNamespace(stopped=False, stop_cause='test'))
    return wiring


@pytest.mark.parametrize('first_stop', ['external', 'guard'])
@pytest.mark.parametrize('first_release', ['external', 'guard'])
def test_stop_sources_remain_independent(
        chain, clock, guard_wiring, first_stop, first_release):
    arbiter = chain._arbiter
    arbiter.on_publisher_count(1, 0)
    arbiter.on_firmware_confirm(True)
    arbiter.mark_receiver_clean(simulated=True)
    assert arbiter.evaluate(clock()).is_zero
    chain.submit_nav(0.05, 0.0, 0.0, clock())
    assert not arbiter.evaluate(clock()).is_zero

    def set_source(source, active):
        if source == 'external':
            chain.set_stop(active)
        else:
            guard_wiring.bridge.guard.stopped = active
            guard_wiring.forward_stop_to_arbiter(chain)

    other_stop = 'guard' if first_stop == 'external' else 'external'
    other_release = 'guard' if first_release == 'external' else 'external'
    set_source(first_stop, True)
    set_source(other_stop, True)
    assert arbiter.evaluate(clock()).zero_reason is ZeroReason.STOP_REQUESTED

    set_source(first_release, False)
    assert arbiter.evaluate(clock()).zero_reason is ZeroReason.STOP_REQUESTED
    chain.submit_nav(0.08, 0.0, 0.0, clock())
    assert arbiter.evaluate(clock()).zero_reason is ZeroReason.STOP_REQUESTED

    set_source(other_release, False)
    assert arbiter.evaluate(clock()).is_zero
    chain.submit_nav(0.06, 0.0, 0.0, clock())
    assert not arbiter.evaluate(clock()).is_zero


@pytest.mark.parametrize('first_release', ['external', 'guard'])
def test_reconnect_preserves_both_stop_sources(chain, guard_wiring, first_release):
    chain.set_stop(True)
    guard_wiring.bridge.guard.stopped = True
    guard_wiring.forward_stop_to_arbiter(chain)
    chain.on_transport_reconnect()

    if first_release == 'external':
        chain.set_stop(False)
    else:
        guard_wiring.bridge.guard.stopped = False
        guard_wiring.forward_stop_to_arbiter(chain)
    assert chain._arbiter._stop_active is True

    if first_release == 'external':
        guard_wiring.bridge.guard.stopped = False
        guard_wiring.forward_stop_to_arbiter(chain)
    else:
        chain.set_stop(False)
    assert chain._arbiter._stop_active is False
