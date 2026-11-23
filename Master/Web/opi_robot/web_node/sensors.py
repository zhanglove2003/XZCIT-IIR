"""网页端传感器新鲜度与诊断解析的纯逻辑。

把"数据什么时候算新鲜"和"诊断怎么解读"从 web_debug 里抽出来，
这样无需启动 HTTP 服务即可测试。

关键规则：**断线后不能继续显示旧帧当作实时数据**。
过期数据一律返回状态 ``stale``，绝不返回 ``fresh``。
"""
from ..diagnostic_level import decode_level

CONNECTING = 'connecting'
FRESH = 'fresh'
STALE = 'stale'
UNKNOWN = 'unknown'

# diagnostic_msgs/DiagnosticStatus 等级
LEVEL_OK = 0
LEVEL_WARN = 1
LEVEL_ERROR = 2
LEVEL_STALE = 3


def freshness(last_t, now, timeout, ever_seen=False):
    """由最近一次数据时间给出新鲜度状态。

    ever_seen 用于区分"从未收到"和"收到过但已过期"。
    """
    if last_t is None:
        return UNKNOWN if not ever_seen else STALE
    age = now - last_t
    if age < 0:
        # 时钟回退时不谎报新鲜
        return STALE
    if age < timeout:
        return FRESH
    return STALE


def parse_diagnostic(msg):
    """从 DiagnosticArray 中取 lidar 状态，返回字典或 None。"""
    if msg is None:
        return None
    for status in getattr(msg, 'status', []) or []:
        if status.name != 'lidar':
            continue
        values = {}
        for kv in getattr(status, 'values', []) or []:
            values[kv.key] = kv.value
        return {
            # 真实消息的 level 是单字节 bytes；统一解码成整数。
            # 无法识别时给 None，由视图层按"未知"处理。
            'level': decode_level(status.level),
            'message': status.message,
            'state': values.get('state', 'unknown'),
            'reason': values.get('reason', ''),
            'age_s': _to_float(values.get('age_s')),
            'coverage': _to_float(values.get('coverage')),
            'reconnects': _to_int(values.get('reconnects')),
            'port': values.get('port', ''),
            'frames_ok': _to_int(values.get('frames_ok')),
            'frames_bad': _to_int(values.get('frames_bad')),
        }
    return None


def _to_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def _to_int(text):
    try:
        return int(text)
    except (TypeError, ValueError):
        return None


def level_to_state(level):
    """诊断等级映射到可显示的标签。无法识别的等级不是 OK。"""
    if level is None:
        return 'unknown'
    if level == LEVEL_OK:
        return 'ok'
    if level == LEVEL_WARN:
        return 'warn'
    return 'error'


def sensor_view(lidar_diag, lidar_diag_t, scan_t, image_t, now,
                scan_timeout=1.0, image_timeout=1.0,
                diag_timeout=3.0):
    """汇总三个传感器视图，供网页直接渲染。

    返回字段全部是字符串或简单数值，不含 inf/nan，可直接进 JSON。
    """
    diag_state = UNKNOWN
    if lidar_diag is not None:
        diag_age = None if lidar_diag_t is None else now - lidar_diag_t
        if diag_age is None or diag_age > diag_timeout:
            diag_state = STALE
        else:
            diag_state = level_to_state(lidar_diag['level'])

    scan_state = freshness(scan_t, now, scan_timeout, scan_t is not None)
    image_state = freshness(image_t, now, image_timeout, image_t is not None)

    # 只有诊断健康且扫描新鲜时，雷达数据才算可信
    trusted = diag_state == 'ok' and scan_state == FRESH
    if scan_state == FRESH and diag_state != 'ok':
        scan_state = STALE

    return {
        'lidar_diag': diag_state,
        'lidar_state': ((lidar_diag.get('state') or UNKNOWN)
                        if lidar_diag else UNKNOWN),
        'lidar_reason': ((lidar_diag.get('reason') or '')
                         if lidar_diag else ''),
        'lidar_age': (None if scan_t is None else round(now - scan_t, 2)),
        'lidar_trusted': trusted,
        'scan': scan_state,
        'image': image_state,
        'image_age': (None if image_t is None else round(now - image_t, 2)),
    }


STATE_TEXT = {
    FRESH: '实时',
    STALE: '已过期',
    UNKNOWN: '无数据',
    CONNECTING: '连接中',
}

DIAG_TEXT = {
    'ok': '正常',
    'warn': '警告',
    'error': '故障',
    STALE: '过期',
    UNKNOWN: '未知',
}


def describe_state(state):
    """状态到中文短标签。未知状态原样返回，不猜。"""
    return STATE_TEXT.get(state, str(state))
