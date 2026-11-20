"""电量型号未确认前，仅显示状态帧电压；不按 3S 锂电猜测百分比。"""

import pytest

from opi_robot.web_node.console_views import battery_view


@pytest.mark.parametrize('voltage', [4.5, 12.6, 24.5])
def test_unverified_battery_voltage_never_implies_charge_or_alarm(voltage):
    result = battery_view(voltage)

    assert result['voltage_v'] == voltage
    assert result['percent'] is None
    assert result['state'] == 'unknown'
    assert result['text'] == '电量未标定'


def test_missing_voltage_is_not_displayed_as_zero():
    result = battery_view(None)

    assert result['voltage_v'] is None
    assert result['percent'] is None
    assert result['text'] == '无数据'


def test_stop_flag_alone_does_not_claim_the_chassis_is_ready():
    from opi_robot.web_node.chassis import chassis_view

    view = chassis_view({'stop_flag': 0}, 10.0, 10.5)
    assert view['stop_flag'] == 0
    assert view['drive_ready'] is None
    assert view['stop_text'] == '软件未失能；就绪未知'


def test_chassis_data_does_not_fabricate_a_separate_status_node():
    from opi_robot.web_node.web_debug import node_view

    nodes = node_view({}, 100.0, chassis_up=True, bridge_up=True,
                      scan_but_no_node=False)
    assert nodes['chassis_status_node'] == 'offline'
    nodes = node_view({'chassis_status_node': 99.0}, 100.0,
                      chassis_up=True, bridge_up=True, scan_but_no_node=False)
    assert nodes['chassis_status_node'] == 'online'
