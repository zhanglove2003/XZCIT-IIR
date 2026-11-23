#!/usr/bin/env python3
"""真实 ROS 2 QoS 集成验证（必须在香橙派上运行，本机无 rclpy）。

背景：Codex 审核第 2 项发现发布端用 BEST_EFFORT、订阅端用默认 RELIABLE，
真实环境下融合节点订阅 `/scan` 收到 0 条消息。伪 ROS 测试抓不到这类问题，
所以需要一段在真实 ROS 2 上跑的验证。

本脚本不依赖 opi_robot 包，只做三件事：

1. 复现不兼容组合：BEST_EFFORT 发布 + RELIABLE 订阅，确认收不到（对照组）。
2. 断言约定组合：sensor QoS 发布 + sensor QoS 订阅，确认能收到。
3. 读取正在运行的 `/scan` 与 `/image_raw/compressed` 的真实 QoS，
   与 `opi_robot.qos.SENSOR_QOS_SPEC` 比对。

用法（在香橙派，已 source ROS 与环境）：

    python3 qos_integration_check.py --ros-domain-id 137
    python3 qos_integration_check.py --ros-domain-id 137 --self

**必须指定隔离域。** 脚本会创建真实节点并发布测试话题，
默认域（0）上的现场节点会收到这些消息。缺隔离配置时脚本 exit 2。
等价写法：`ROS_DOMAIN_ID=137 OPI_TEST_ISOLATED=1 python3 ...`

退出码：0 全部通过；1 有断言失败；2 环境不具备（缺 rclpy / 话题不存在）。
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ros_isolation import require_isolated_domain, describe_ros_graph_ops  # noqa: E402

# 先校验隔离域再碰 rclpy：缺隔离时直接退出，不建任何节点。
if __name__ == '__main__':
    _REST_ARGV = require_isolated_domain(sys.argv[1:])
else:
    _REST_ARGV = list(sys.argv[1:])

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                           DurabilityPolicy)
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, CompressedImage
except Exception as exc:                                    # noqa: BLE001
    print('环境不具备：%s' % exc)
    sys.exit(2)


RESULTS = []
# 图发现无法提供的字段：[(topic, [field, ...])]
UNVERIFIED = []


def record(name, ok, detail=''):
    RESULTS.append((name, ok, detail))
    print('[%s] %s %s' % ('PASS' if ok else 'FAIL', name, detail))


def sensor_profile():
    """与 opi_robot/qos.py 的 SENSOR_QOS_SPEC 等价。

    这里刻意独立写一份，避免"用被测代码去验证被测代码"。
    若 opi_robot.qos 的取值变了，本脚本的比对会失败并提示同步。
    """
    q = QoSProfile(depth=10)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.history = HistoryPolicy.KEEP_LAST
    q.durability = DurabilityPolicy.VOLATILE
    return q


# 全脚本共用的 context。main() 里赋值，其余函数只读。
_CONTEXT = None


class PairCheck(Node):
    """在同一个节点里建一对发布的订阅，数收到的消息。"""

    def __init__(self, pub_qos, sub_qos, msg_type, topic, context=None):
        # context 必须与 rclpy.init() 用的是同一个，否则节点落在默认
        # 未初始化的 context 上，构造时抛 NotInitializedException。
        super().__init__('qos_pair_check', context=context)
        self.count = 0
        self.pub = self.create_publisher(msg_type, topic, pub_qos)
        self.sub = self.create_subscription(
            msg_type, topic, self._cb, sub_qos)
        self.topic = topic
        self.pub_type_is_laser = (msg_type is LaserScan)

    def _cb(self, _msg):
        self.count += 1

    def run(self, duration=2.0):
        # 执行器必须绑定本节点的 context。rclpy.spin_once(self) 会新建
        # 默认全局 executor，而默认 executor 属于默认 context，
        # 在“节点用私有 context”的场景下会抛 AttributeError: __enter__。
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self)
        try:
            return self._spin_loop(executor, duration)
        finally:
            executor.shutdown()
            executor.remove_node(self)

    def _spin_loop(self, executor, duration):
        t_end = time.monotonic() + duration
        while time.monotonic() < t_end:
            if self.pub_type_is_laser:
                m = LaserScan()
                m.angle_min = 0.0
                m.angle_max = 0.0
                m.angle_increment = 1.0
                m.range_min = 0.02
                m.range_max = 12.0
                m.ranges = [1.0]
            else:
                m = CompressedImage()
                m.format = 'jpeg'
                m.data = b'\xff\xd8\xff\xd9'
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(m)
            executor.spin_once(timeout_sec=0.02)
        return self.count


def check_pair(name, pub_qos, sub_qos, expect_got, msg_type, topic):
    """建收发对，跑 2 秒，核对是否符合预期。"""
    node = PairCheck(pub_qos, sub_qos, msg_type, topic, context=_CONTEXT)
    try:
        got = node.run(2.0)
    finally:
        node.destroy_node()
    if expect_got:
        ok = got > 0
        detail = '收到 %d 条' % got
    else:
        ok = got == 0
        detail = '收到 %d 条（预期 0，说明组合确实不兼容）' % got
    record(name, ok, detail)
    return got


# 端点规格表（回应第五次、第六次审核阻断 4）。
#
# 每个话题给出**发布端和订阅端**的完整规格。字段值必须与
# ``opi_robot/opi_robot/qos.py`` 一致；测试里有对拍断言，防止两边漂移。
#
# *_endpoints: 'require' 表示该端点必须存在，缺失即失败；
#             'verify-if-present' 表示存在才检查，但整体结论会标成
#             "只检查了发布端 / 链路未完全验证"，**不会**报链路通过。
SPECS = {
    '/scan': {
        'publisher': {'reliability': 'BEST_EFFORT', 'durability': 'VOLATILE',
                      'history': 'KEEP_LAST', 'depth': 10},
        'subscriber': {'reliability': 'BEST_EFFORT', 'durability': 'VOLATILE',
                       'history': 'KEEP_LAST', 'depth': 10},
        'publishers_endpoints': 'require',
        'subscriber_endpoints': 'require',
    },
    '/image_raw/compressed': {
        'publisher': {'reliability': 'BEST_EFFORT', 'durability': 'VOLATILE',
                      'history': 'KEEP_LAST', 'depth': 10},
        'subscriber': {'reliability': 'BEST_EFFORT', 'durability': 'VOLATILE',
                       'history': 'KEEP_LAST', 'depth': 10},
        'publishers_endpoints': 'require',
        'subscriber_endpoints': 'require',
    },
    '/diagnostics': {
        'publisher': {'reliability': 'RELIABLE', 'durability': 'VOLATILE',
                      'history': 'KEEP_LAST', 'depth': 10},
        'subscriber': {'reliability': 'RELIABLE', 'durability': 'VOLATILE',
                       'history': 'KEEP_LAST', 'depth': 10},
        'publishers_endpoints': 'require',
        'subscriber_endpoints': 'require',
    },
}

FIELDS = ('reliability', 'durability', 'history', 'depth')

_ENUM_NAMES = {
    'reliability': {ReliabilityPolicy.RELIABLE: 'RELIABLE',
                    ReliabilityPolicy.BEST_EFFORT: 'BEST_EFFORT',
                    ReliabilityPolicy.SYSTEM_DEFAULT: 'SYSTEM_DEFAULT'},
    'durability': {DurabilityPolicy.VOLATILE: 'VOLATILE',
                   DurabilityPolicy.TRANSIENT_LOCAL: 'TRANSIENT_LOCAL',
                   DurabilityPolicy.SYSTEM_DEFAULT: 'SYSTEM_DEFAULT'},
    'history': {HistoryPolicy.KEEP_LAST: 'KEEP_LAST',
                HistoryPolicy.KEEP_ALL: 'KEEP_ALL',
                HistoryPolicy.SYSTEM_DEFAULT: 'SYSTEM_DEFAULT'},
}

# 图发现拿不到值时用它，表示"未知"，永远不等于任何期望值
UNKNOWN = 'UNKNOWN'


def _policy_name(field, raw):
    """把枚举值转成名字。认不出来返回 UNKNOWN，不冒充任何具体策略。"""
    table = _ENUM_NAMES.get(field)
    if table is None or raw is None:
        return UNKNOWN
    for policy, name in table.items():
        if raw == policy:
            return name
    return UNKNOWN


def _qos_field(profile, field):
    """从 QoSProfile 取字段名字。取不到时返回 UNKNOWN。"""
    if profile is None:
        return UNKNOWN
    if field == 'depth':
        raw = getattr(profile, 'depth', None)
        if isinstance(raw, int):
            # 0 不是合法深度：Humble 的图发现对没填的 depth 返回 0。
            # 这是"库没提供"，不是"端点用了 0"。
            return raw if raw > 0 else UNKNOWN
        return UNKNOWN
    return _policy_name(field, getattr(profile, field, None))


def _profile_to_dict(profile):
    return {f: _qos_field(profile, f) for f in FIELDS}


def _compare(name, got, want):
    """比对一个字段，打印并返回是否通过。

    UNKNOWN 与任何期望值都不相等，因此自动判失败：
    既不会把"查不到"当成通过，也不会把未知策略当成兼容。
    """
    good = (got == want)
    record(name, good, '%r（期望 %r）' % (got, want))
    return good


def _check_endpoint(topic, label, profile, want):
    """核对一个端点的四个字段。

    返回 (符合规格?, 无法验证的字段列表)。

    第七次审核阻断 2：Humble 的图发现**不提供** history（返回
    SYSTEM_DEFAULT）和 depth（返回 0）。那是**库的能力限制**，
    不能据此说现场端点配错了，也不能当成通过。这两种情况记成
    "无法验证"，与"不符规格"分开统计。
    """
    ok = True
    unverifiable = []
    for field in FIELDS:
        if field not in want:
            continue
        got = _qos_field(profile, field)
        if got == UNKNOWN:
            record('%s %s %s' % (topic, label, field), False,
                   '无法验证：图发现没有提供该字段（期望 %r）'
                   % (want[field],))
            unverifiable.append(field)
            continue
        if got == 'SYSTEM_DEFAULT':
            record('%s %s %s' % (topic, label, field), False,
                   '无法验证：图发现只给了 SYSTEM_DEFAULT（期望 %r）'
                   % (want[field],))
            unverifiable.append(field)
            continue
        if not _compare('%s %s %s' % (topic, label, field), got,
                        want[field]):
            ok = False
    return ok, unverifiable


def _wanted_profile(want):
    """把规格字典转成 rclpy.QoSProfile，供 qos_check_compatible 使用。"""
    profile = QoSProfile(depth=want.get('depth', 10))
    if 'reliability' in want:
        profile.reliability = getattr(ReliabilityPolicy, want['reliability'])
    if 'durability' in want:
        profile.durability = getattr(DurabilityPolicy, want['durability'])
    if 'history' in want:
        profile.history = getattr(HistoryPolicy, want['history'])
    return profile


def _parse_compat_result(result):
    """把 qos_check_compatible 的返回值归一成 (状态名, 说明)。

    Humble（rclpy 3.x）返回的是 ``(QoSCompatibility, reason)`` 元组，
    不是带 .compatibility 字段的对象。第七次审核就是在这一步抛
    ``'tuple' object has no attribute 'compatibility'``。

    同时接受：对象形式（.compatibility/.reason）、二元组、
    以及枚举本身。认不出来返回 None。
    """
    level = None
    text = ''
    if isinstance(result, tuple):
        if not result:
            return None
        level = result[0]
        text = result[1] if len(result) > 1 else ''
    elif hasattr(result, 'compatibility'):
        level = result.compatibility
        text = getattr(result, 'reason', '')
    else:
        level = result
    name = getattr(level, 'name', None)
    if name is None:
        return None
    name = str(name).upper()
    for known in ('OK', 'WARN', 'ERROR'):
        if known in name:
            return known, str(text or name)
    return None


# 这两个值都不代表端点声明了具体策略，无法据此判断兼容性。
# SYSTEM_DEFAULT 表示交给环境决定，实际取值由图发现给不出。
UNRESOLVED_VALUES = (UNKNOWN, 'SYSTEM_DEFAULT')

_LOCAL_COVERS = ('reliability', 'durability')


def _local_compat(pub_profile, sub_profile):
    """本地兜底：只看 reliability 与 durability 的兼容规则。

    第八次审核阻断 3：旧实现只判 UNKNOWN，SYSTEM_DEFAULT 会被当成
    已知策略，进而可能输出 OK。现在两者都算无法判断。

    本函数不是全部 DDS QoS 策略的完整兼容性证明：deadline、liveliness、
    lease duration 等一律不看。返回值只能是 ERROR 或 UNKNOWN，
    确认兼容时也写明覆盖范围有限。
    """
    for field in _LOCAL_COVERS:
        pv = _qos_field(pub_profile, field)
        sv = _qos_field(sub_profile, field)
        if pv in UNRESOLVED_VALUES or sv in UNRESOLVED_VALUES:
            return 'UNKNOWN', '%s 无法判断（发布 %s / 订阅 %s）' % (
                field, pv, sv)
        if field == 'reliability':
            if pv == 'BEST_EFFORT' and sv == 'RELIABLE':
                return 'ERROR', 'BEST_EFFORT 发布与 RELIABLE 订阅不兼容'
        if field == 'durability':
            if pv == 'VOLATILE' and sv == 'TRANSIENT_LOCAL':
                return 'ERROR', 'VOLATILE 发布与 TRANSIENT_LOCAL 订阅不兼容'
    return 'OK', ('本地规则只覆盖 reliability 与 durability，'
                  '这两项未见冲突；不是全部 DDS 策略的完整兼容性证明')


def compatibility(pub_profile, sub_profile):
    """判断一对发布/订阅 QoS 是否兼容。返回 (状态, 说明)。

    状态取值：
      'OK' / 'WARN' / 'ERROR' / 'UNKNOWN'（无法判定）。

    第六次审核阻断 2：旧实现只比 reliability。
    第七次审核阻断 2：Humble 返回 tuple，没解析对。
    这里优先调用 ROS 自己的 qos_check_compatible；调用失败或格式
    不认识时退回本地完整规则，并在说明里写清回落原因。
    """
    checker = getattr(rclpy.qos, 'qos_check_compatible', None)
    if checker is None:
        return _local_compat(pub_profile, sub_profile)
    try:
        result = checker(pub_profile, sub_profile)
    except Exception as exc:                           # noqa: BLE001
        why = 'qos_check_compatible 调用失败：%s' % exc
    else:
        parsed = _parse_compat_result(result)
        if parsed is not None:
            return parsed
        why = 'qos_check_compatible 返回无法识别的结果：%r' % (result,)
    state, detail = _local_compat(pub_profile, sub_profile)
    return state, '%s；本地规则：%s' % (why, detail)


def check_topic_qos(topic, spec):
    """按规格核对话题上的端点。

    返回字典，结论分开，任何一项都不能冒充另外一项：

      'spec'    端点配置是否符合约定（True/False）
      'link'    消息链路能否完全验证（True/False）
      'unknown' 无法验证的字段列表，如 ['history', 'depth']
      'pubs'    图发现实际看到的发布端数量（第十一次审核阻断 2）
      'subs'    图发现实际看到的订阅端数量

    第七次审核要求：端点配置验证、图发现可验证内容、消息实际收发
    三者分开记录，并保留结论边界。图发现不提供 history/depth 时
    标"无法验证"，既不算配置错误，也不算通过。
    """
    node = rclpy.create_node(
        'qos_inspect_%s' % topic.strip('/').replace('/', '_'),
        context=_CONTEXT)
    try:
        pubs = node.get_publishers_info_by_topic(topic)
        subs = node.get_subscriptions_info_by_topic(topic)
    finally:
        node.destroy_node()

    spec_ok = True
    link_ok = True
    unknown = []

    # 端点存在性必须两种状态都记录。
    # 第九次审核阻断 1：旧实现只在缺失时记录 False，端点存在时没有任何
    # 记录，下游（四情景脚本的情景 A/C、现场核对）查不到 True 记录，
    # 直接抛"没有记录"。这里统一契约：存在记 True 并给出数量，
    # 缺失记 False 并写明链路结论未验证。
    if spec.get('publishers_endpoints') == 'require':
        record('%s 发布端存在' % topic, bool(pubs),
               ('发现 %d 个发布者' % len(pubs)) if pubs
               else '没有发布者；链路结论记为未验证')
        if not pubs:
            spec_ok = False
            link_ok = False
    if spec.get('subscriber_endpoints') == 'require':
        record('%s 订阅端存在' % topic, bool(subs),
               ('发现 %d 个订阅者' % len(subs)) if subs
               else '没有订阅者；链路结论记为未验证')
        if not subs:
            link_ok = False

    for info in pubs:
        ok, unv = _check_endpoint(topic, '发布端', info.qos_profile,
                                  spec['publisher'])
        spec_ok = spec_ok and ok
        unknown.extend(unv)
    for info in subs:
        ok, unv = _check_endpoint(topic, '订阅端', info.qos_profile,
                                  spec['subscriber'])
        spec_ok = spec_ok and ok
        unknown.extend(unv)

    saw_pair = False
    for p_info in pubs:
        for s_info in subs:
            saw_pair = True
            state, why = compatibility(p_info.qos_profile, s_info.qos_profile)
            record('%s 端点兼容性' % topic, state == 'OK',
                   '%s（%s）' % (state, why))
            link_ok = link_ok and (state == 'OK')

    if not saw_pair:
        link_ok = False

    unknown = sorted(set(unknown))
    if unknown:
        # 端点配置本身没被证实：
        #   - spec 不能算"符合规格"；
        #   - link 也不能报完全验证（消息链路成立的前提是端点配置已知）。
        spec_ok = False
        link_ok = False
    detail = '发布端 %d 个 / 订阅端 %d 个' % (len(pubs), len(subs))
    if unknown:
        detail += '；字段无法验证：%s' % ','.join(unknown)
    record('%s 链路完全验证' % topic, link_ok, detail)
    if unknown:
        print('   注意：%s 的 %s 无法由图发现验证，结论按未验证处理'
              % (topic, ','.join(unknown)))
    return {'spec': spec_ok, 'link': link_ok, 'unknown': unknown,
            'pubs': len(pubs), 'subs': len(subs)}


def main(argv=None):
    argv = _REST_ARGV if argv is None else argv
    if __name__ == '__main__':
        print(describe_ros_graph_ops())
    self_only = '--self' in argv
    global _CONTEXT
    context = rclpy.Context()
    _CONTEXT = context
    rclpy.init(context=context)
    try:
        stamp = str(int(time.time()))

        # 1. 对照组：不兼容组合必须收不到，证明检查本身有效
        check_pair(
            '对照组 BEST_EFFORT 发布 + RELIABLE 订阅（预期 0 条）',
            sensor_profile(),
            QoSProfile(depth=10),          # 默认 RELIABLE
            expect_got=False,
            msg_type=LaserScan,
            topic='/qos_review_incompatible_%s' % stamp)

        # 2. 约定组合：必须能收到
        check_pair(
            '约定组合 sensor 发布 + sensor 订阅（预期 >0 条）',
            sensor_profile(),
            sensor_profile(),
            expect_got=True,
            msg_type=LaserScan,
            topic='/qos_review_compatible_%s' % stamp)

        check_pair(
            '图像 sensor 发布 + sensor 订阅（预期 >0 条）',
            sensor_profile(),
            sensor_profile(),
            expect_got=True,
            msg_type=CompressedImage,
            topic='/qos_review_image_%s' % stamp)

        if not self_only:
            # 3. 真实话题的 QoS：按 SPECS 逐项核对
            #    三个结论分开记录：端点配置、链路、无法验证的字段
            for topic, spec in SPECS.items():
                summary = check_topic_qos(topic, spec)
                if summary['unknown']:
                    UNVERIFIED.append((topic, summary['unknown']))
    finally:
        rclpy.shutdown(context=context)

    bad = [r for r in RESULTS if not r[1]]
    link = [r for r in RESULTS if r[0].endswith('链路完全验证')]
    unverified = [r for r in link if not r[1]]
    print()
    print('合计 %d 项，失败 %d 项' % (len(RESULTS), len(bad)))
    if unverified:
        print()
        print('!! 以下话题的链路未完全验证（不能当作通过）：')
        for name, _ok, detail in unverified:
            print('   %s %s' % (name.replace(' 链路完全验证', ''), detail))
    if UNVERIFIED:
        print()
        print('!! 以下字段图发现无法提供，结论为"无法验证"'
              '（既不算配置错误，也不算通过）：')
        for topic, fields in UNVERIFIED:
            print('   %s: %s' % (topic, ', '.join(fields)))
    return 0 if not bad else 1


if __name__ == '__main__':
    sys.exit(main(_REST_ARGV))
