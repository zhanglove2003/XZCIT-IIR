"""diagnostic_msgs/DiagnosticStatus 等级的双向转换。

真实 ROS 2 与多种版本的 diagnostic_msgs 对 `level` 字段类型不一致：
有的版本是 `uint8`，有的把等级声明成 `byte`，序列化时要求
长度为 1 的 bytes（`b'\x00'`、`b'\x01'`…）。

这个模块是唯一的转换点。发布端与两个接收端都调用它，
不在别处直接比较或赋值 `status.level`。

用法：

    from ..diagnostic_level import encode_level, decode_level

    status.level = encode_level(LEVEL_ERROR)     # 发布
    level = decode_level(status.level)           # 订阅

`decode_level` 对未知或畸形输入返回 `None`，调用方必须把它当
"未知"处理，不能当成 OK。
"""

LEVEL_OK = 0
LEVEL_WARN = 1
LEVEL_ERROR = 2
LEVEL_STALE = 3

# 未在 messages 里声明等级时的兜底：只认这几个整数
_KNOWN = (LEVEL_OK, LEVEL_WARN, LEVEL_ERROR, LEVEL_STALE)


class LevelError(ValueError):
    """等级值无法识别。"""


def encode_level(level):
    """把整数等级转成消息字段可以接受的类型。

    返回 bytes 长度 1。真实 ROS 2 的 `byte` 字段接受 bytes；
    接受整数的版本也能从 bytes 还原。
    """
    if isinstance(level, (bytes, bytearray)):
        if len(level) != 1:
            raise LevelError('等级 bytes 长度必须是 1，收到 %d' % len(level))
        return bytes(level)
    try:
        value = int(level)
    except (TypeError, ValueError):
        raise LevelError('等级不是整数: %r' % (level,))
    if value < 0 or value > 255:
        raise LevelError('等级超出 0..255: %r' % (level,))
    return bytes([value])


def decode_level(raw):
    """把消息字段里的等级还原成整数。

    接受 int、长度为 1 的 bytes / bytearray、以及数字字符串。
    只返回 `_KNOWN` 里声明的等级。

    以下情况一律返回 None（未知），调用方必须 fail closed：
      - None、空 bytes、多字节 bytes；
      - 负整数；
      - 未在 `_KNOWN` 里声明的整数（例如 7、255）；
      - 非数字字符串、浮点数、其他类型。
    """
    value = _as_int(raw)
    if value is None:
        return None
    if value not in _KNOWN:
        return None
    return value


def _as_int(raw):
    """把原始值尽量还原成整数。无法还原返回 None。"""
    if raw is None or isinstance(raw, bool):
        return None
    if isinstance(raw, (bytes, bytearray)):
        if len(raw) != 1:
            return None
        return raw[0]
    if isinstance(raw, str):
        try:
            return int(raw.strip(), 10)
        except ValueError:
            return None
    if isinstance(raw, float):
        # 浮点只在恰好等于整数时接受
        return int(raw) if raw == int(raw) else None
    if isinstance(raw, int):
        return raw
    try:
        return int(raw)
    except (TypeError, ValueError):
        return None


def is_ok(raw):
    """等级是否为 OK。无法识别的等级不是 OK。"""
    return decode_level(raw) == LEVEL_OK


def is_fault(raw, threshold=LEVEL_ERROR):
    """等级是否达到或超过故障阈值。无法识别时按故障处理（fail closed）。"""
    value = decode_level(raw)
    if value is None:
        return True
    return value >= threshold
