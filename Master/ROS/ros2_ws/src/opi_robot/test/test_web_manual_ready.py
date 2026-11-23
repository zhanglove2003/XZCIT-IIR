"""手动就绪判断应查询手动话题，并接受命令桥与 guard 的两个端点。"""
from types import SimpleNamespace

import pytest

from opi_robot.web_node.web_debug import Bridge


def endpoint(name='c50c_uart3_node', namespace='/'):
    return SimpleNamespace(node_name=name, node_namespace=namespace)


@pytest.mark.parametrize('count', [1, 2])
def test_manual_ready_accepts_chassis_endpoints(count):
    topics = []
    def subscriptions(topic):
        topics.append(topic)
        return [endpoint() for _ in range(count)] if topic == '/manual_cmd_vel' else []
    node = SimpleNamespace(get_subscriptions_info_by_topic=subscriptions)
    assert Bridge._manual_ready(node) is True
    assert topics == ['/manual_cmd_vel']


@pytest.mark.parametrize('endpoints', [[], [endpoint('web_bridge')], [endpoint(namespace='/other')]])
def test_manual_ready_rejects_missing_chassis(endpoints):
    node = SimpleNamespace(get_subscriptions_info_by_topic=lambda topic: endpoints)
    assert Bridge._manual_ready(node) is False
