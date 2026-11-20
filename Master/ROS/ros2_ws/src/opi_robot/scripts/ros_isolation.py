#!/usr/bin/env python3
"""真实 ROS 2 测试脚本的隔离域守卫。

背景：在真实 ROS 2 里建节点并发布消息，会进入当前 ROS 域。
现场若已有融合节点在默认域（0）订阅 `/diagnostics` 或 `/scan`，
测试消息会被它收到，可能让现场节点误判传感器状态。

本模块提供统一入口：脚本启动前必须先调用 `require_isolated_domain()`。
不满足隔离条件时直接 `SystemExit(2)`，不建立任何节点。

隔离条件（满足其一即可）：
  A. 设置了 `OPI_TEST_ISOLATED=1`，并且 `ROS_DOMAIN_ID` 是大于 0 的整数；
  B. 传入了 `--ros-domain-id N`（N > 0），此时由脚本自己写环境变量。

禁止使用的域：0 是默认域，现场节点在跑。域 213 是历史审核用的隔离域，
也建议避开——如果同一域里还有别的审核进程，会互相干扰。用前先看提示。
"""

import os
import sys

# 现场默认域，绝不允许测试脚本使用
DEFAULT_DOMAIN = 0
# 审核期间用过的隔离域，可以用但要提示可能与他人共用
HISTORIC_DOMAINS = (213,)

FLAG_VAR = 'OPI_TEST_ISOLATED'
DOMAIN_VAR = 'ROS_DOMAIN_ID'


class NotIsolated(SystemExit):
    """没有满足隔离条件。退出码 2，不建节点。

    `domain_zero_note` 只在"实际解析出的域号正好是 0"时为真。
    域号非法（如 233 越界、非数字）不打印"域 0 禁止使用"，
    那会把读者的注意力引到错误的原因上。
    """

    def __init__(self, message, domain_zero_note=False):
        sys.stderr.write('拒绝运行：%s\n' % message)
        sys.stderr.write('\n隔离方式（二选一）：\n')
        sys.stderr.write('  ROS_DOMAIN_ID=137 %s=1 python3 <script>\n'
                         % FLAG_VAR)
        sys.stderr.write('  或给脚本加参数 --ros-domain-id 137\n')
        if domain_zero_note:
            sys.stderr.write('\n域 0 禁止使用：现场节点在默认域运行，'
                             '测试消息会污染它的状态。\n')
        super().__init__(2)


# ROS 2 文档给出的有效域号范围是 0~232。上限 232 用于把域号编码进
# UDP 组播端口（7400 + 250*d），超过会算出非法端口。
# 本脚本要求隔离域明显不同于现场默认域，所以下限用 DEFAULT_DOMAIN 之上。
DOMAIN_MAX = 232


def _parse_positive_domain(raw):
    """把字符串解析成合法域号。失败返回 None。

    上界取 ROS 2 文档的 232。若目标机的 ROS/RMW 实现不支持这个范围，
    init 阶段会报错退出，不会静默落到默认域。
    """
    try:
        value = int(str(raw).strip())
    except (TypeError, ValueError):
        return None
    if value <= DEFAULT_DOMAIN:
        return None
    if value > DOMAIN_MAX:
        return None
    return value


def detect_from_argv(argv):
    """从命令行取 `--ros-domain-id N`。

    返回 (domain_or_None, argv_without_option)。
    支持 `--ros-domain-id 5` 与 `--ros-domain-id=5`。
    """
    out = []
    found = None
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == '--ros-domain-id':
            if i + 1 < len(argv):
                found = _parse_positive_domain(argv[i + 1])
                i += 2
                continue
            return None, argv
        if arg.startswith('--ros-domain-id='):
            found = _parse_positive_domain(arg.split('=', 1)[1])
            i += 1
            continue
        out.append(arg)
        i += 1
    return found, out


def is_isolated(env=None):
    """判断当前环境是否满足隔离条件。返回 (bool, 说明)。"""
    env = os.environ if env is None else env
    if env.get(FLAG_VAR) != '1':
        return False, '缺少 %s=1' % FLAG_VAR
    domain = _parse_positive_domain(env.get(DOMAIN_VAR))
    if domain is None:
        return False, '%s 不是有效域号（当前 %r）' % (
            DOMAIN_VAR, env.get(DOMAIN_VAR))
    return True, '域 %d' % domain


def require_isolated_domain(argv=None):
    """校验隔离条件。不满足则抛 NotIsolated（退出码 2）。

    返回值：去掉隔离相关参数后的 argv（从下标 1 开始的部分）。
    """
    if argv is None:
        argv = sys.argv[1:]

    domain_from_argv, rest = detect_from_argv(list(argv))
    if domain_from_argv is not None:
        os.environ[DOMAIN_VAR] = str(domain_from_argv)
        os.environ[FLAG_VAR] = '1'

    ok, detail = is_isolated()
    if not ok:
        raw = os.environ.get(DOMAIN_VAR)
        is_zero = (str(raw).strip() == '0') if raw is not None else False
        raise NotIsolated(detail, domain_zero_note=is_zero)

    domain = int(os.environ[DOMAIN_VAR])
    if domain in HISTORIC_DOMAINS:
        sys.stderr.write('提示：当前用域 %d，历史审核用过它，'
                         '确认没有别的进程在同一域。\n' % domain)
    sys.stdout.write('隔离域：%d（本脚本不会向默认域发布消息）\n' % domain)
    return rest


def describe_ros_graph_ops():
    """脚本对 ROS graph 做了什么。打印给用户看。"""
    return (
        'ROS graph 操作说明：\n'
        '  - 在当前进程内创建节点，订阅/发布测试话题；\n'
        '  - 不修改、不重启任何已存在的节点；\n'
        '  - 不连接雷达串口、不打开摄像头、不发布 /cmd_vel；\n'
        '  - 进程退出时销毁节点并 shutdown context。\n'
    )
