#!/usr/bin/env python3
"""在真实 ROS 2 上验证 QoS 检查脚本的四种判定情景。

第八次审核阻断 1 要求：四种情景必须**分别**断言，不能用一个总布尔值
把"配置不符""字段无法验证""端点缺失"混在一起判定。

本脚本在**指定的隔离域**里当场创建模拟端点（不打开任何设备、不碰
真实话题），再用检查脚本自己的 `check_topic_qos()` 走真实图发现判定：

  情景 A 完整兼容：发布端与订阅端都符合规格。
         reliability 与 durability 分别断言；history 与 depth 由图发现
         决定：提供就核，不提供就明确记为未验证，不当作已核验，
         也不因此误报模拟端点不兼容。
  情景 B 不兼容  ：VOLATILE 发布 / TRANSIENT_LOCAL 订阅 -> 兼容性 ERROR。
  情景 C 缺端点  ：只有发布端没有订阅端 -> 必须独立断言"缺订阅端"，
         同时允许已存在的发布端字段未知。
  情景 D 字段未知：用可控对象喂进判定函数，验证 SYSTEM_DEFAULT 与读不到
         的 depth 都落到"无法验证"，不依赖目标机的图发现能力。
  情景 E 多端点：同一话题建多个发布端/订阅端，靠前的端点符合规格、
         靠后的端点明确错误。判定必须逐端聚合（第十次审核其他项 2）：
         不能因为第一条记录是好的就把后面的错误漏掉。

另有一条 `受控消息收发` 检查：在同一对端点上真发一条消息，证明链路
能通。它**不能**证明 history 或 depth 的具体配置。

用法（香橙派，已 source ROS 2 Humble）：

    python3 qos_four_scenario_check.py --ros-domain-id 251

退出码：0 全部符合预期；1 有判定不符；2 环境不具备。
"""

import os
import sys
import time
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ros_isolation import require_isolated_domain, describe_ros_graph_ops  # noqa: E402

if __name__ == '__main__':
    _REST_ARGV = require_isolated_domain(sys.argv[1:])
else:
    _REST_ARGV = list(sys.argv[1:])

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.qos import DurabilityPolicy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import LaserScan
    import qos_integration_check as Q
except ImportError as exc:
    print('SKIP 环境不具备：%s' % exc)
    print('     需要 source /opt/ros/humble/setup.bash')
    sys.exit(2)

SCENARIO_RESULTS = []
EXTRA_RESULTS = []
SPEC = None

UNVERIFIABLE_PREFIX = '无法验证'


def check(bucket, name, fn):
    try:
        detail = fn()
        bucket.append((name, True, detail or ''))
        print('PASS %s %s' % (name, detail or ''))
    except Exception as exc:                       # noqa: BLE001
        bucket.append((name, False, '%s: %s' % (type(exc).__name__, exc)))
        print('FAIL %s %s: %s' % (name, type(exc).__name__, exc))


def _rows(results=None):
    return Q.RESULTS if results is None else results


def entries(topic, label, results=None):
    """取一个标签下的**全部**记录（多发布端/订阅端会出现同名多条）。"""
    name = '%s %s' % (topic, label)
    rows = [(ok, detail) for n, ok, detail in _rows(results) if n == name]
    if not rows:
        raise AssertionError('没有记录：%s' % name)
    return rows


def worst_state(rows):
    """逐端聚合（第十次审核其他项 2）。

    旧实现 entry() 只取重复标签的第一条：多发布端时，若第一端字段
    未知、第二端明确错误，单个情景的断言会漏掉后者。聚合规则：
    任一端明确错误 -> 'bad'；否则任一端无法验证 -> 'unverifiable'；
    全部通过 -> 'ok'。坏结论优先，不允许被前面的好记录掩盖。
    """
    if any(not ok and not unverifiable(detail) for ok, detail in rows):
        return 'bad'
    if any(not ok for ok, _detail in rows):
        return 'unverifiable'
    return 'ok'


def entry(topic, label, results=None):
    """取聚合后的判定：(是否通过, 说明文字)。"""
    rows = entries(topic, label, results)
    if len(rows) == 1:
        return rows[0]
    state = worst_state(rows)
    detail = '；'.join(d for _ok, d in rows) + '（共 %d 端）' % len(rows)
    return state == 'ok', detail


def unverifiable(detail):
    return detail.startswith(UNVERIFIABLE_PREFIX)


def assert_ok(topic, label, results=None):
    """断言该标签下每一条记录都通过（无法验证也算未通过）。"""
    rows = entries(topic, label, results)
    bad = [d for ok, d in rows if not ok and not unverifiable(d)]
    unv = [d for ok, d in rows if not ok and unverifiable(d)]
    if bad:
        raise AssertionError('%s 未通过：%s' % (label, '；'.join(bad)))
    if unv:
        raise AssertionError('%s 无法验证：%s' % (label, '；'.join(unv)))
    return '；'.join(d for _ok, d in rows)


def field_state(topic, label, field, results=None):
    """返回 'ok' / 'bad' / 'unverifiable'，多端点按 worst_state 聚合。"""
    rows = entries(topic, '%s %s' % (label, field), results)
    return worst_state(rows)


def assert_scenario_a(topic, summary, results=None):
    """情景 A：端点都符合规格。

    独立断言 reliability / durability；history、depth 由图发现能力决定，
    读不到就记未验证，既不当作已核验，也不因此判不兼容。
    """
    for label in ('发布端', '订阅端'):
        for field in ('reliability', 'durability'):
            assert_ok(topic, '%s %s' % (label, field), results)

    unverified_fields = []
    for label in ('发布端', '订阅端'):
        for field in ('history', 'depth'):
            state = field_state(topic, label, field, results)
            assert state != 'bad', '%s/%s 与规格不符' % (label, field)
            if state == 'unverifiable':
                unverified_fields.append('%s/%s' % (label, field))

    # 兼容性必须 OK，绝不因字段缺失误报不兼容
    compat_detail = assert_ok(topic, '端点兼容性', results)
    assert 'ERROR' not in compat_detail, compat_detail

    assert_ok(topic, '发布端存在', results)
    assert_ok(topic, '订阅端存在', results)

    assert set(summary['unknown']) <= {'history', 'depth'}, summary
    # 有未知字段时 spec 与 link 都不许报通过；全核到才可报通过
    expect = not summary['unknown']
    assert summary['spec'] is expect, summary
    assert summary['link'] is expect, summary
    return unverified_fields


def assert_scenario_c(topic, summary, results=None):
    """情景 C：缺订阅端必须独立判失败；已存在端点字段未知是允许的。"""
    ok, detail = entry(topic, '订阅端存在', results)
    assert ok is False, '缺订阅端却判通过：%s' % detail
    assert '没有订阅者' in detail, detail
    assert_ok(topic, '发布端存在', results)
    for field in ('reliability', 'durability'):
        assert_ok(topic, '发布端 %s' % field, results)
    assert summary['link'] is False, summary
    bad = [f for f in ('history', 'depth')
           if field_state(topic, '发布端', f, results) == 'bad']
    assert not bad, '发布端字段与规格不符：%s' % bad
    return [f for f in ('history', 'depth')
            if field_state(topic, '发布端', f, results) == 'unverifiable']


def qos(reliability='BEST_EFFORT', durability='VOLATILE',
        history='KEEP_LAST', depth=10):
    q = QoSProfile(depth=depth)
    q.reliability = getattr(ReliabilityPolicy, reliability)
    q.durability = getattr(DurabilityPolicy, durability)
    q.history = getattr(HistoryPolicy, history)
    return q


def wait_for_endpoint_counts(poll, want_pubs, want_subs, timeout=8.0,
                             interval=0.05):
    """限时轮询图发现，直到端点数量达到预期（第十一次审核阻断 2）。

    poll() 返回 (发布端数, 订阅端数)。达到预期立即返回实际数量；
    超时抛 AssertionError 并带出**实际**数量——数量不足不许继续做
    聚合断言，否则个别端点会被冒充成完整的多端点检查。
    """
    deadline = time.monotonic() + timeout
    counts = (0, 0)
    while True:
        counts = tuple(poll())
        if counts[0] >= want_pubs and counts[1] >= want_subs:
            return counts
        if time.monotonic() >= deadline:
            raise AssertionError(
                '等待图发现超时（%.1fs）：发布端 %d/%d、订阅端 %d/%d'
                % (timeout, counts[0], want_pubs, counts[1], want_subs))
        time.sleep(interval)


def assert_scenario_e(topic, summary, results=None):
    """情景 E 断言（第十一次审核阻断 2）：先数量、后聚合。

    必须先确认图发现实际看到了预期的 2 个发布端、2 个订阅端，
    再验证逐端字段聚合与 2×2 全部兼容性配对。只发现错误端点
    （1 发布端 + 1 订阅端）时，数量断言必须失败，不许输出
    "2 发布端 + 2 订阅端"的结论。
    """
    assert summary.get('pubs') == 2, \
        '发布端发现数 %r != 2，多端点检查不成立' % (summary.get('pubs'),)
    assert summary.get('subs') == 2, \
        '订阅端发现数 %r != 2，多端点检查不成立' % (summary.get('subs'),)
    assert summary['link'] is False, summary
    assert field_state(topic, '发布端', 'durability', results) == 'bad', \
        '第二发布端的 durability 错误被漏掉'
    assert field_state(topic, '订阅端', 'reliability', results) == 'bad', \
        '第二订阅端的 reliability 错误被漏掉'
    compat = entries(topic, '端点兼容性', results)
    assert len(compat) == 4, \
        '兼容性配对应为 2×2=4 条，实际 %d 条：%s' % (len(compat), compat)
    assert any(not ok and 'ERROR' in detail for ok, detail in compat), compat
    return len(compat)


def main(argv=None):
    global SPEC
    argv = _REST_ARGV if argv is None else argv
    print(describe_ros_graph_ops())
    SPEC = {
        'publisher': dict(Q.SPECS['/scan']['publisher']),
        'subscriber': dict(Q.SPECS['/scan']['subscriber']),
        'publishers_endpoints': 'require',
        'subscriber_endpoints': 'require',
    }

    stamp = str(int(time.time()))
    context = rclpy.Context()
    Q._CONTEXT = context
    rclpy.init(context=context)
    node = Node('qos_four_scenario_%s' % stamp, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    keep = []

    def spin(times=20):
        for _ in range(times):
            executor.spin_once(timeout_sec=0.02)

    def pub(topic, q):
        p = node.create_publisher(LaserScan, topic, q)
        keep.append(p)
        return p

    def sub(topic, q, sink=None):
        cb = (lambda m: sink.append(m)) if sink is not None else (lambda m: None)
        s = node.create_subscription(LaserScan, topic, cb, q)
        keep.append(s)
        return s

    try:
        def scen_a():
            topic = '/qos_a_%s' % stamp
            pub(topic, qos())
            sub(topic, qos())
            spin()
            r = Q.check_topic_qos(topic, SPEC)
            unverified_fields = assert_scenario_a(topic, r)
            ok, compat_detail = entry(topic, '端点兼容性')
            note = ('history/depth 未验证(%s)，按未验证处理'
                    % ','.join(unverified_fields)) if unverified_fields \
                else 'history/depth 已由图发现核验'
            return '兼容性=%s；%s' % (compat_detail.split('（')[0], note)

        def scen_b():
            topic = '/qos_b_%s' % stamp
            pub(topic, qos(durability='VOLATILE'))
            sub(topic, qos(durability='TRANSIENT_LOCAL'))
            spin()
            r = Q.check_topic_qos(topic, SPEC)
            assert r['link'] is False, r
            ok, detail = entry(topic, '端点兼容性')
            assert ok is False, '不兼容组合被判通过：%s' % detail
            assert 'ERROR' in detail, detail
            return '兼容性=ERROR link=False'

        def scen_c():
            topic = '/qos_c_%s' % stamp
            pub(topic, qos())
            spin()
            r = Q.check_topic_qos(topic, SPEC)
            unv = assert_scenario_c(topic, r)
            note = ('发布端 history/depth 未验证(允许)' if unv
                    else '发布端四字段全部核验')
            return '缺订阅端=是 link=False；%s' % note

        def scen_d():
            """字段未知的判定不依赖目标机图发现能力。"""
            topic = '/qos_d_%s' % stamp
            want = dict(Q.SPECS['/scan']['publisher'])

            # 用可控对象构造三种"读不到"的情形
            sysdef = types.SimpleNamespace(
                reliability=ReliabilityPolicy.SYSTEM_DEFAULT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST, depth=10)
            zero_depth = types.SimpleNamespace(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST, depth=0)

            seen = []
            for label, prof in (('reliability=SYSTEM_DEFAULT', sysdef),
                                ('depth=0', zero_depth)):
                ok, unv = Q._check_endpoint(topic, label, prof, want)
                assert unv, '%s 没有被记成无法验证' % label
                seen.append('%s->%s' % (label, ','.join(unv)))

            state, why = Q._local_compat(sysdef, sysdef)
            assert state != 'OK', 'SYSTEM_DEFAULT 被判兼容：%s' % why
            assert '无法判断' in why, why
            return '；'.join(seen) + '；local_compat=%s' % state

        def scen_e():
            """多端点聚合 + 端点数量确认（第十一次审核阻断 2）。

            两个发布端：第一个符合规格，第二个 durability 明确错误；
            两个订阅端：第一个符合规格，第二个 reliability 明确错误。
            先限时等待图发现真正看到 2+2 个端点——只发现错误端点时
            本情景必须失败，不许用 1+1 冒充多端点检查——然后核对
            逐端字段聚合与 2×2 全部兼容性配对。
            """
            topic = '/qos_e_%s' % stamp
            pub(topic, qos())                               # 发布端1：符合规格
            pub(topic, qos(durability='TRANSIENT_LOCAL'))   # 发布端2：durability 错
            sub(topic, qos())                               # 订阅端1：符合规格
            sub(topic, qos(reliability='RELIABLE'))         # 订阅端2：reliability 错

            def poll():
                spin(2)
                return (len(node.get_publishers_info_by_topic(topic)),
                        len(node.get_subscriptions_info_by_topic(topic)))

            got = wait_for_endpoint_counts(poll, 2, 2)
            r = Q.check_topic_qos(topic, SPEC)
            pairs = assert_scenario_e(topic, r)
            return '图发现确认 %d 发布端 + %d 订阅端；2×2=%d 个兼容性' \
                '配对，靠后端点的 durability/reliability 错误均被逐端抓住' \
                % (got[0], got[1], pairs)

        def transfer():
            """受控消息收发：证明链路能通，不能证明 history/depth。"""
            topic = '/qos_xfer_%s' % stamp
            sink = []
            p = pub(topic, qos())
            sub(topic, qos(), sink)
            spin(30)
            msg = LaserScan()
            msg.ranges = [1.0]
            for _ in range(20):
                p.publish(msg)
                spin(5)
                if sink:
                    break
            assert sink, '端点存在但没有收到消息'
            return '收到 %d 条（不能证明 history/depth）' % len(sink)

        check(SCENARIO_RESULTS, '情景A_完整兼容', scen_a)
        check(SCENARIO_RESULTS, '情景B_不兼容', scen_b)
        check(SCENARIO_RESULTS, '情景C_缺订阅端', scen_c)
        check(SCENARIO_RESULTS, '情景D_字段未知', scen_d)
        check(SCENARIO_RESULTS, '情景E_多端点', scen_e)
        check(EXTRA_RESULTS, '受控消息收发', transfer)
    finally:
        keep.clear()
        executor.shutdown()
        try:
            executor.remove_node(node)
        except Exception:                            # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown(context=context)

    def report(title, rows):
        bad = [r for r in rows if not r[1]]
        print()
        print('%s：共 %d 项，通过 %d，失败 %d'
              % (title, len(rows), len(rows) - len(bad), len(bad)))
        for name, _ok, detail in bad:
            print('  失败 %s: %s' % (name, detail))
        return bad

    bad = report('情景判定', SCENARIO_RESULTS)
    bad += report('附加检查', EXTRA_RESULTS)
    return 0 if not bad else 1


if __name__ == '__main__':
    sys.exit(main())
