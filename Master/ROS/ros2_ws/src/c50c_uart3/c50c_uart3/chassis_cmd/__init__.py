"""C50C 底盘 UART3 控制命令编码（纯函数，离线）。

输入线速度/角速度，输出固件可解析的 11 字节协议帧。
**不含任何串口 I/O，不发送任何数据**；发送与电机启动不在本模块职责内。

来源与版本（任务 B，2026-11-08）：本文件复制自已审核候选
``/home/snow/robot-task-worktrees/c50c-cmd-enc.RACZMy/worktree/ros_stage/
opi_robot/opi_robot/chassis_cmd/__init__.py``（原 SHA-256
``fb345477850c2d8faf8e46b66de5c3c6d20b9db088105981d400792530e9df11``）。
本任务的改动只有两处：本段来源说明，以及文件末尾新增的
``validate_speed_command``（桥的输入校验）。协议常量、BCC、量化与
``encode_speed_frame`` 保持与原候选逐字节一致。原候选未合并进本仓库，
复制关系以本段说明为准（不能用 git 溯源）。

协议依据（固件基线 2026-11-05，`source.tar.gz` SHA-256 见交接目录）：

- 帧解析状态机：``BALANCE/uartx_callback.c:27-164``（``UartxControll_Callback``）
- 帧头/帧尾：``BALANCE/Inc/data_task.h:12-13``（0x7B / 0x7D）
- BCC 校验：``BALANCE/data_task.c:73-80``（字节 0..8 逐字节异或）
- 速度解码：``BALANCE/uartx_callback.c:437-448``
  （``(High<<8)+Low`` 截断为 int16，毫单位 → 单位）
- 命令字节：``uartx_callback.c:60-118``（0=速度命令；本模块只编码 0）
- 限速：``BALANCE/balance_task.c:949-951``（默认 3.5 m/s，``balance_task.c:2566``）

已知的固件侧重大限制（详见 task-results/c50c-cmd-enc.md）：
在当前烧录的 REAR_WHEEL_DEBUG+FOUR_TURN_DEBUG 构建中，速度命令被解析但
**不会驱动电机**（``balance_task.c:1076-1121`` 的 PS2 调试分支独占输出）。
"""

# ---- 协议常量（来源见模块 docstring 行号）----
FRAME_HEADER = 0x7B          # data_task.h:12
FRAME_TAIL = 0x7D            # data_task.h:13
CMD_SPEED = 0x00             # uartx_callback.c:60（buffer[1]==0 → 三轴速度）
FRAME_LENGTH = 11            # uartx_callback.c:51（if (Count == 11) 校验包长）
PAYLOAD_BYTES = 9            # uartx_callback.c:58（Check_BCC(buffer, 9)；定义在 data_task.c:73）

# v3 帧序号门（chassis_ack.c 的 check_seq）：frame[2] = 0x80 | seq。
# 常量与序号状态对象都在 seq.py，这里只引用。
from .seq import CAP_HOST_ACK, SEQ_MASK  # noqa: E402

# 线协议 int16 上限：±32767 毫单位（量化 0.001）
WIRE_LIMIT_MPS = 32.767
# 固件在 Drive_Motor 入口按轴限幅（balance_task.c:949-951，默认值
# balance_task.c:2566）。固件是静默截断，本模块默认只检查不截断，
# 超限是否放行由上层显式决定。
FIRMWARE_SPEED_LIMIT_MPS = 3.5


def check_bcc(data: bytes) -> int:
    """与固件 ``Check_BCC``（data_task.c:73-80）一致：逐字节异或。"""
    bcc = 0
    for b in data:
        bcc ^= b
    return bcc


def _to_int16_be(value_milli: int) -> tuple[int, int]:
    """毫单位整数 → (高字节, 低字节)，二进制补码 16 位。

    固件解码为 ``((High<<8)+Low)`` 赋给 ``short``（uartx_callback.c:445），
    即大端二进制补码。
    """
    if not -32768 <= value_milli <= 32767:
        raise ValueError(
            '速度毫单位 %d 超出 int16 线协议范围 ±32767' % value_milli)
    v = value_milli & 0xFFFF
    return (v >> 8) & 0xFF, v & 0xFF


def encode_speed_frame(vx: float, vz: float, vy: float = 0.0, *,
                       seq: int = 0) -> bytes:
    """编码三轴速度命令帧。

    参数单位：``vx``/``vy`` 线速度 m/s；``vz`` 角速度 rad/s
    （固件 4WD 运动学 ``InverseKinematics_4wd`` 使用 ``Vx`` 与 ``Vz``，
    单位注释 m/s 与 rad/s，``balance_task.c:879``；差速车不使用 ``vy``）。

    ``seq`` 是本会话的逐帧序号（0..127），写进 ``frame[2]`` 的低 7 位；
    bit7 固定置 :data:`~c50c_uart3.chassis_cmd.seq.CAP_HOST_ACK`（v3 固件
    要求该位，否则以 ``CHASSIS_RX_NO_SEQ`` 拒绝整帧）。序号状态的唯一
    归属对象是 :class:`~c50c_uart3.chassis_cmd.seq.FrameSeq`；本函数只按
    调用方给的序号编码，不自增、不缓存。

    量化：固件以 0.001 为最小步长解码（毫单位），本模块按四舍五入
    （Python ``round``，银行家舍入）取整。若需要与 C 的截断语义逐位
    对齐，调用方应自行保证输入已是毫单位整数值的准确倍数。

    返回 11 字节帧；**只构造字节，不做任何发送**。

    引发 ValueError：速度超出线协议 ±32.767，或超出固件限速
    3.5 m/s/rad/s（见 ``strict_firmware_limit``）。
    """
    for name, v in (('vx', vx), ('vy', vy), ('vz', vz)):
        if abs(v) > WIRE_LIMIT_MPS:
            raise ValueError('%s=%r 超出线协议 ±%s' % (name, v, WIRE_LIMIT_MPS))
        if abs(v) > FIRMWARE_SPEED_LIMIT_MPS:
            raise ValueError(
                '%s=%r 超出固件限速 %s（balance_task.c:949-951/2566）；'
                '固件会静默截断，本模块不静默放行'
                % (name, v, FIRMWARE_SPEED_LIMIT_MPS))

    if not isinstance(seq, int) or isinstance(seq, bool) \
            or not 0 <= seq <= SEQ_MASK:
        raise ValueError(f'序号必须是 0..{SEQ_MASK} 的整数，收到 {seq!r}')

    frame = bytearray(FRAME_LENGTH)
    frame[0] = FRAME_HEADER
    frame[1] = CMD_SPEED
    # v3：bit7 能力位 + bit0..6 逐帧序号（chassis_ack.c 的 check_seq）
    frame[2] = CAP_HOST_ACK | seq
    for idx, v in ((3, vx), (5, vy), (7, vz)):
        hi, lo = _to_int16_be(int(round(v * 1000)))
        frame[idx] = hi
        frame[idx + 1] = lo
    frame[9] = check_bcc(frame[:PAYLOAD_BYTES])
    frame[10] = FRAME_TAIL
    return bytes(frame)


# ---- 任务 B 新增：桥的输入校验（不改编码）-------------------------------
# 上限是**占位值**，不是已标定的速度或停车距离：第一阶段的直行速度与
# 停车距离都待现场确认（任务书 C）。桥把更小上限作为默认值，避免把固件
# 的 3.5 m/s 静默截断上限当作可用速度。
# 线速度上限（m/s）。0.3 为人工设定的操作上限，不是实测标定值。
# 0.4 与网页巡航档一致（2026-11-13 用户要求：三处统一为 0.4，不留歧义）。
PLACEHOLDER_MAX_LINEAR_MPS = 0.4
PLACEHOLDER_MAX_ANGULAR_RAD_S = 0.30


def is_zero_command(vx: float, vy: float, vz: float) -> bool:
    """三轴量化后是否全为零（与固件 ``ROS_Command_Frame`` 的判零口径一致：
    固件比较的是**已解码的 float**，即毫单位取整后的结果）。

    例如 ``vx=0.0004`` 量化后为 0 毫单位，固件视为零速帧，可用于授权；
    这里同样视为零，避免桥误判成"非零非法输入"。
    """
    for v in (vx, vy, vz):
        # NaN/inf 不能参与量化（round(nan) 抛 ValueError、round(inf) 抛
        # OverflowError），且它们本身非法，按"非零"处理交给校验拒绝。
        if not isinstance(v, (int, float)) or v != v or v in (
                float('inf'), float('-inf')):
            return False
        if int(round(v * 1000)) != 0:
            return False
    return True


def validate_speed_command(vx: float, vy: float, vz: float,
                           max_linear: float = PLACEHOLDER_MAX_LINEAR_MPS,
                           max_angular: float = PLACEHOLDER_MAX_ANGULAR_RAD_S,
                           *, max_forward: float | None = None,
                           ) -> str | None:
    """校验一组速度命令。返回 ``None`` 表示通过，否则返回拒绝原因字符串。

    只做**数值与限速**校验，不判断来源、过期或停车状态——那些由桥的
    其它模块负责。``nan``/``inf`` 一律拒绝（会让比较失效并破坏编码）。
    """
    values = (('vx', vx), ('vy', vy), ('vz', vz))
    for name, v in values:
        if not isinstance(v, (int, float)):
            return f'{name} 不是数值: {type(v).__name__}'
        # NaN 与任何数比较均为 False，必须在范围检查之前显式拒绝
        if v != v:
            return f'{name} 为 NaN'
        if v in (float('inf'), float('-inf')):
            return f'{name} 为无穷大'
    forward_limit = max_linear if max_forward is None else max_forward
    if abs(vx) > forward_limit:
        return f'vx={vx} 超出占位限速 ±{forward_limit} m/s'
    if abs(vy) > max_linear:
        return f'vy={vy} 超出占位限速 ±{max_linear} m/s'
    if abs(vz) > max_angular:
        return f'vz={vz} 超出占位限速 ±{max_angular} rad/s'
    # 编码器自身还有线协议 ±32.767 与固件 3.5 的上限；占位限速更小时
    # 上面的检查先挡住。此处保留一次尝试，确保任何配置组合都不会把
    # 超线协议的值送进编码（例如调用方把 max_* 调得很大）。
    try:
        encode_speed_frame(vx, vz, vy)
    except ValueError as exc:
        return f'编码拒绝: {exc}'
    return None


def is_zero_frame(frame: bytes) -> bool:
    """字节串是否为合法的**零速**命令帧（三轴量化后全零）。

    用于第二道防线：任何"禁止非零"状态下都只放行零速帧。非法长度或
    非法帧头/帧尾一律视为**非零**（保守方向：宁可拦下，不可放行）。
    """
    if not isinstance(frame, (bytes, bytearray)) or len(frame) != FRAME_LENGTH:
        return False
    if frame[0] != FRAME_HEADER or frame[10] != FRAME_TAIL:
        return False
    if frame[1] != CMD_SPEED:
        return False
    return all(b == 0 for b in frame[3:9])
