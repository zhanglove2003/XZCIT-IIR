"""原始雷达帧证据追踪与安全许可判定（纯逻辑，无 rclpy）。

为什么需要这个模块（回应任务书 T1 / §2 雷达新鲜度警告）：

``lidar_node`` 在健康窗口内（``frame_timeout=0.5 s``）即使**零个新原始帧**
也会按 poll 周期把环形缓存整圈重发并写新 header 时间（``reader.poll`` →
``_build_scan``）。因此「持续收到 /scan」不等于「持续收到新的原始雷达数据」；
消息计数与新 header 时间不能作为原始帧新鲜度的证据。

本模块提供三件事：

1. :class:`RawEvidenceTracker` —— 消费 lidar_node 发出的原始帧证据
   （``epoch / frames_ok / max_point_age_ms``，见 ``lidar_node._publish_evidence``），
   只认「同一 epoch 内 ``frames_ok`` 严格增长」为原始数据在流动。
   epoch 变更（节点重启）与计数回退都作废增长历史，必须重新建立；
   证据停更或计数停滞超过时限一律判失效。全部判定用调用方注入的
   单调时钟秒值，不比较消息时间戳。
2. :class:`UltrasonicGate` —— 前置 HC-SR04 的补充停车迟滞门
   （2026-11-15 接入）。进入停车 ≤ ``stop_d``，退出 ≥ ``release_d``，
   中间保持原状态防抖动。2026-11-17 起改为 **fail-closed**：无回波
   （``0xFFFF`` → ``max_range``）、断流、从未收到都判无效 → 停车，
   恢复必须显式 ``rearm()``（重新启动）。
3. :func:`permission` —— 把「原始帧证据 + 雷达可信 + 避障动作 +
   超声波补充停车」映射成单一许可布尔量与原因。前两项任何一项无法
   确认都返回 ``False``（fail-closed）。

本模块不做 I/O、不导入 ROS；节点层（``fusion_node``）负责把话题消息
转成这里的函数调用。雷达障碍距离阈值不在这里出现——它们属于
``failclosed.classify``，本模块只消费其结论；超声波阈值在本模块出现，
是因为迟滞/超时语义与许可判定不可分离（见 UltrasonicGate）。
"""

from __future__ import annotations

import json
import math

#: 证据 JSON 的 schema 版本。不匹配一律按「无证据」处理。
RAW_EVIDENCE_SCHEMA = 1

#: 安全许可 JSON（``drive_safety``）的 schema 版本。不匹配按拒绝处理。
PERMISSION_SCHEMA = 1

#: 超声波"无回波"映射值（``Range.max_range``）的缺省上界（m）。与
#: ``c50c_uart3`` 的 ``ultrasonic_max_range_m`` 默认值一致。
DEFAULT_INVALID_AT = 4.5

#: 放行前同一 epoch 内 ``frames_ok`` 必须观察到的最少增长次数。
#: 1 次增长即证明「基线之后确实来了新原始帧」；首条消息只建立基线。
MIN_INCREASES = 1


class RawEvidenceTracker:
    """原始帧证据的状态追踪（每次收到证据消息调一次 :meth:`feed`）。

    参数
    ----
    timeout_s : 失效时限（秒）。两个时钟共用它：
        证据消息停止到达超过时限，或 ``frames_ok`` 最后一次增长超过
        时限，都判失效。默认与雷达 ``frame_timeout``（0.5 s）同级，
        由节点参数 ``raw_frame_timeout`` 注入。
    """

    def __init__(self, timeout_s: float) -> None:
        if timeout_s <= 0:
            raise ValueError('证据时限必须为正')
        self.timeout_s = float(timeout_s)
        self.epoch = None
        self.last_count = None
        self.increases = 0          # 当前 epoch 内观察到的增长次数
        self.last_increase_t = None
        self.last_rx_t = None
        self.regressions = 0        # 同 epoch 内计数回退次数（诊断用）
        self.epoch_changes = 0      # epoch 变更次数（诊断用）
        self.malformed = 0          # 无法解析的证据消息数（诊断用）
        self.max_point_age_ms = None
        self.messages = 0

    # ------------------------------------------------------------ 输入
    def feed(self, epoch, frames_ok, now, max_point_age_ms=None) -> None:
        """记录一条证据。``now`` 为单调时钟秒值。

        - epoch 变更（lidar 节点重启）：作废增长历史，从当前计数重建
          基线；此后需重新观察 :data:`MIN_INCREASES` 次增长才放行。
        - 计数增长：原始数据在流动，刷新增长时刻。
        - 计数回退（同 epoch）：不可能由正常递增产生，按「无法证明
          新数据」处理——作废增长历史并以新值重建基线，计数为证。
        - 计数相等：可能是缓存圈重发，不动增长时刻（由时限判停滞）。
        """
        if isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 0:
            raise ValueError('epoch 必须是非负整数')
        if (isinstance(frames_ok, bool) or not isinstance(frames_ok, int)
                or frames_ok < 0):
            raise ValueError('frames_ok 必须是非负整数')
        if self.epoch is None:
            # 首次观测：只建立基线，不计为 epoch 变更
            self.epoch = epoch
            self.last_count = frames_ok
            self.increases = 0
            self.last_increase_t = None
        elif epoch != self.epoch:
            self.epoch_changes += 1
            self.epoch = epoch
            self.last_count = frames_ok
            self.increases = 0
            self.last_increase_t = None
        elif frames_ok > self.last_count:
            self.increases += 1
            self.last_count = frames_ok
            self.last_increase_t = float(now)
        elif frames_ok < self.last_count:
            self.regressions += 1
            self.increases = 0
            self.last_count = frames_ok
            self.last_increase_t = None
        self.last_rx_t = float(now)
        self.max_point_age_ms = max_point_age_ms
        self.messages += 1

    def note_malformed(self) -> None:
        """收到一条无法解析的证据消息（fail-closed：不影响已有时限）。"""
        self.malformed += 1

    # ------------------------------------------------------------ 判定
    def status(self, now: float) -> dict:
        """当前证据状态。``flowing`` 为 False 时 ``reason`` 说明为什么。"""
        if self.last_rx_t is None:
            return self._verdict(False, 'no_evidence', now, None)
        age = float(now) - self.last_rx_t
        if age > self.timeout_s:
            return self._verdict(False, 'evidence_stale', now, age)
        if self.last_increase_t is None:
            return self._verdict(False, 'evidence_no_increase', now, age)
        idle = float(now) - self.last_increase_t
        if idle > self.timeout_s:
            return self._verdict(False, 'raw_frames_stalled', now, age)
        if self.increases < MIN_INCREASES:
            return self._verdict(False, 'evidence_rebuilding', now, age)
        return self._verdict(True, 'ok', now, age)

    def _verdict(self, flowing, reason, now, age) -> dict:
        return {
            'flowing': flowing,
            'reason': reason,
            'age': None if age is None else round(max(0.0, age), 3),
            'idle_s': (None if self.last_increase_t is None
                       else round(max(0.0, float(now) - self.last_increase_t), 3)),
            'epoch': self.epoch,
            'frames_ok': self.last_count,
            'increases': self.increases,
            'regressions': self.regressions,
            'epoch_changes': self.epoch_changes,
            'malformed': self.malformed,
            'max_point_age_ms': self.max_point_age_ms,
            'timeout_s': self.timeout_s,
        }

    def snapshot(self) -> dict:
        """只读快照（诊断/报告用，不做判定）。"""
        return {
            'epoch': self.epoch, 'frames_ok': self.last_count,
            'increases': self.increases, 'regressions': self.regressions,
            'epoch_changes': self.epoch_changes, 'malformed': self.malformed,
            'messages': self.messages,
            'max_point_age_ms': self.max_point_age_ms,
            'timeout_s': self.timeout_s,
        }


def parse_evidence_json(text):
    """解析 lidar_node 发出的证据 JSON。非法返回 ``None``。

    严格契约：``v`` 必须等于 :data:`RAW_EVIDENCE_SCHEMA`；``epoch`` 与
    ``frames_ok`` 必须是非负整数（Python 的 ``bool`` 是 ``int`` 子类，
    显式拒绝）；``max_point_age_ms`` 为非负整数或 ``null``。
    任何多余或缺失字段不在此拒绝——消费方只读已定义字段。
    """
    if not isinstance(text, (str, bytes, bytearray)):
        return None
    try:
        data = json.loads(text)
    except (ValueError, UnicodeDecodeError):
        return None
    if not isinstance(data, dict):
        return None
    if data.get('v') != RAW_EVIDENCE_SCHEMA:
        return None
    epoch = data.get('epoch')
    count = data.get('frames_ok')
    for value in (epoch, count):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            return None
    max_age = data.get('max_point_age_ms')
    if max_age is not None and (isinstance(max_age, bool)
                                or not isinstance(max_age, int)
                                or max_age < 0):
        return None
    return {'epoch': epoch, 'frames_ok': count, 'max_point_age_ms': max_age}


class UltrasonicGate:
    """前置 HC-SR04 的补充停车迟滞门（纯逻辑，无 ROS）。

    语义（2026-11-15 与用户现场约定；2026-11-17 改为 fail-closed）：

    - **信任上限**：读数可信上界是 ``invalid_at``（缺省 4.5 m；节点注入
      ``us_max_range``）。``>= invalid_at`` 判无效；停车迟滞仍按
      ``stop_d`` / ``release_d`` 两阈值。
    - **迟滞**：进入停车 ≤ ``stop_d``，退出 ≥ ``release_d``，两阈值
      之间保持上一状态，防止读数在边界抖动时许可反复翻转。
    - **无回波 = 无效**：发布端把固件 ``0xFFFF``（无回波/数据陈旧）映射成
      ``range == max_range``（``c50c_uart3/uart3_node.py``
      ``_publish_ultrasonic``）。旧实现把这种读数当"远处无障碍"喂给
      迟滞门，于是无回波天然等于"畅通"（fail-open）。现在：
        - ``range >= invalid_at``（默认取 ``max_range`` 参数）判无效，
          **立即断开（unlatch）、不刷新新鲜度**；
        - 读数停更超过 ``timeout_s`` 同样判无效。
      无效不是"畅通"，而是"不知道"→ ``verdict`` 的 ``stop=True``，
      许可因此拒绝。恢复必须满足 ``rearm()``：**重新启动**（用户 2026-11-17
      指示："无效即停，恢复后需重新启动"），不会因为读数自己变远而
      自动放行。
    - **只增停不解除**：本门输出只会让许可更严；它不消费、也不覆盖
      雷达的任何结论。

    全部判定用调用方注入的单调时钟秒值（与 RawEvidenceTracker 同约定）。
    """

    def __init__(self, stop_d: float, release_d: float,
                 timeout_s: float, invalid_at: float = None) -> None:
        if not (0.0 < stop_d < release_d):
            raise ValueError('需要 0 < stop_d < release_d')
        if timeout_s <= 0:
            raise ValueError('timeout_s 必须为正')
        self.stop_d = float(stop_d)
        self.release_d = float(release_d)
        self.timeout_s = float(timeout_s)
        # 无回波映射值（max_range），节点注入 us_max_range（4.5）。
        # [2026-11-17 复核] 缺省不再取 release_d：那会让"读数恰好等于
        # release_d"（本应退出迟滞停车）被判 no_echo。缺省 = 传感器规格
        # 上限 4.50 m（与 uart3_node 的 ultrasonic_max_range_m 默认一致）。
        if invalid_at is None:
            invalid_at = DEFAULT_INVALID_AT
        if not (self.release_d < float(invalid_at)):
            raise ValueError('invalid_at 必须 > release_distance')
        self.invalid_at = float(invalid_at)
        self.latched = False        # True = 补充停车生效（含无效锁存）
        self.last_range = None      # 最近一次可采信读数（m）
        self.last_rx_t = None       # 最近一次可采信读数时刻（单调秒）
        self.fault = False          # True = 处于"无效/断流"态，需 rearm
        self.fault_reason = ''      # 'no_echo' | 'stale' | 'never'
        self.fault_since = None
        self.rearms = 0
        self._rearm_waiting_sample = False

    # ------------------------------------------------------------ 输入
    def feed(self, range_m, now: float) -> None:
        """记录一条 Range 读数。

        非有限值、0/负数、>= ``invalid_at`` 一律按无效处理（不刷新新鲜度，
        见类 docstring）；有效读数只刷新新鲜度并驱动迟滞锁存。
        """
        try:
            r = float(range_m)
        except (TypeError, ValueError):
            self._fault('no_echo', now)
            return
        if not math.isfinite(r) or r <= 0.0:
            # NaN/inf / 0 / 负数：坏数据，无效（fail-closed，需 rearm）
            self._fault('no_echo', now)
            return
        if r >= self.invalid_at:
            # [2026-11-18 用户定案] 无回波（0xFFFF→max_range）= 波束打空，
            # 近距传感器"没听到"≠"有障碍"：按"远处无约束"处理（unlatch、
            # 刷新新鲜度），不再锁车。近距停车仍由 stop_d/release_d 迟滞管。
            self._rearm_waiting_sample = False
            self.last_range = r
            self.last_rx_t = float(now)
            self.latched = False
            return
        if self.fault:
            # 已无效：读数不能自行解除，必须 rearm。此时不刷新新鲜度，
            # 保留故障前的最后一条可采信读数（snapshot / age 语义，
            # 与 test_nan_invalid 的"时间戳没被 NaN 刷新"一致）。
            return
        self._rearm_waiting_sample = False
        self.last_range = r
        self.last_rx_t = float(now)
        if self.latched:
            if r >= self.release_d:
                self.latched = False
        elif r <= self.stop_d:
            self.latched = True

    def _fault(self, reason, now):
        self._rearm_waiting_sample = False
        self.fault = True
        self.fault_reason = reason
        if self.fault_since is None:
            self.fault_since = float(now)
        self.latched = False

    # ------------------------------------------------------------ 判定
    def verdict(self, now: float):
        """返回 ``(stop, range_m, us_valid, age_s)``。

        ``stop``：True = 拒绝放行（近距锁存 或 无效/断流锁存 或 从未收到）。
        ``us_valid``：False = 输入无效（无回波 / 断流），此时 stop 恒 True。
        超时期间不重置近距锁存（读数恢复且已 rearm 后，若仍 ≤ stop_d 会
        立即再进停车，方向安全）。
        """
        age = None if self.last_rx_t is None else float(now) - self.last_rx_t
        if age is None:
            if not self.fault and not self._rearm_waiting_sample:
                self._fault('never', now)
            return True, None, False, None
        if math.isfinite(age) and age > self.timeout_s and not self.fault:
            self._fault('stale', now)            # NaN 等异常年龄按断流处理
        elif not math.isfinite(age) and not self.fault:
            self._fault('stale', now)
        if self.fault:
            return True, None, False, age
        return self.latched, self.last_range, True, age

    def rearm(self, now: float = None) -> bool:
        """外部明确"重新启动"后调用：清无效锁存。

        清复位后的状态后，下一条有效读数按 ``stop_d``/``release_d`` 正常
        判定（新读数到来之前 ``verdict`` 仍是 ``stop=True``，因为
        等待新读数，不重新锁存 ``never``）。读数本身永远不会自动清除
        无效锁。网页入口 ``web_debug.ultrasonic_rearm()`` 会调用本方法。
        """
        self.fault = False
        self.fault_reason = ''
        self.fault_since = None
        # [2026-11-17 复核] 复位后不再预先锁存停车：旧实现置 latched=True
        # 并把 last_rx_t 清 None，下一条读数（> release_d）走 "已锁存" 分支
        # 才能退出，读数在 (stop_d, release_d] 时会一直停着；读数 ≤ stop_d
        # 时又不会重新进场。现在由下一条有效读数按两阈值正常判定：
        # 0.20 m → 停车、0.90 m → 放行。
        self.latched = False
        self.last_range = None
        self.last_rx_t = None
        self._rearm_waiting_sample = True
        self.rearms += 1
        return True

    def snapshot(self) -> dict:
        """只读快照（诊断/上报用，不做判定）。"""
        return {'stop': self.latched, 'fault': self.fault,
                'fault_reason': self.fault_reason,
                'range_m': self.last_range,
                'stop_distance': self.stop_d,
                'release_distance': self.release_d,
                'invalid_at': self.invalid_at,
                'timeout_s': self.timeout_s,
                'rearms': self.rearms}


def permission(evidence_ok, evidence_reason,
               trusted_ok, trusted_why,
               action, action_reason,
               us_stop=False, us_reason='',
               motion_blocked=False, motion_reason=''):
    """把独立判定合成为单一许可。返回 ``(allowed, reason)``。

    顺序即优先级：原始帧证据失效 > 雷达不可信 > 雷达检测到需要停车的
    障碍 > 运动方向前置门（``motion_check.obstacle_blocked``，自动/导航
    命令与许可共用的同一判据）> 超声波补充停车 > 覆盖/数据未知。
    ``clear`` 与 ``slow`` 放行——慢行区（``slow_distance`` 与
    ``stop_distance`` 之间）不是「判为需要停车」的障碍，但运动方向门
    仍会按 ``stop_distance`` 再核一次。

    ``us_stop`` / ``motion_blocked`` 只会让许可更严：为 True 时一律拒绝，
    为 False 时对许可无任何影响。超声波输入无效（``us_valid=False``，见
    ``UltrasonicGate.verdict``）时必须由调用方折算成 ``us_stop=True``
    （原因 ``ultrasonic_invalid``）——本函数看不到 valid 位，不会替它
    把"无回波"当成"畅通"。
    """
    if not evidence_ok:
        return False, evidence_reason
    if not trusted_ok:
        return False, trusted_why or 'lidar_unknown'
    if action == 'stop':
        return False, 'obstacle_stop'
    if motion_blocked:
        return False, motion_reason or 'motion_blocked'
    if us_stop:
        return False, us_reason or 'ultrasonic_stop'
    if action == 'unknown':
        return False, action_reason or 'front_no_points'
    if action in ('clear', 'slow'):
        return True, 'ok'
    return False, 'unknown_action:%s' % (action,)


__all__ = ['MIN_INCREASES', 'PERMISSION_SCHEMA', 'RAW_EVIDENCE_SCHEMA',
           'RawEvidenceTracker', 'UltrasonicGate', 'parse_evidence_json',
           'permission', 'encoders_still', 'stop_stage_record']

#: 编码器静止证据的有效时限（秒）。桥把四轮瞬时轮速放诊断键
#: encoder_{a..d}_reported，本时限内全 0 才算"回传证实停住"；
#: 超时即证据失效，回落 None（不知道），不拿陈旧诊断冒充静止。
ENCODER_STILL_TIMEOUT_S = 0.5


def encoders_still(enc, now, last_t, timeout_s=ENCODER_STILL_TIMEOUT_S):
    """底盘回传静止证据：True=四轮全 0；False=有回传且在动；None=无证据。

    ``enc`` 为 {encoder_a_reported: ...} 形式的键值表（来自诊断），
    ``last_t`` 是最近一条编码器诊断到达时刻（节点单调钟秒）。证据
    过期与从未收到都返回 None——与 guard_bridge 的 vehicle_stopped
    三态口径一致（写成功/回传/发零速三件事分开，B8）。
    """
    if last_t is None or now - float(last_t) > float(timeout_s):
        return None
    if not enc or len(enc) < 4:
        return None
    try:
        values = [float(v) for v in enc.values()]
    except (TypeError, ValueError):
        return None
    return all(v == 0.0 for v in values)


def stop_stage_record(zero_issued, issued, enc_still):
    """组装三阶段停状态 dict（B8：三件事分开记录，不合并成一个布尔）。

    - ``zero_issued``：本拍命令是否为零速（决策层口径）；
    - ``issued``：零速命令是否实际送出（fusion 每拍发布 cmd_vel；
      发布通道关闭时为 False——"没发命令"不伪装成"已发停车命令"）；
    - ``vehicle_stopped``：底盘回传证实停住（True/False/None）。
    """
    return {'zero_issued': bool(zero_issued), 'issued': bool(issued),
            'vehicle_stopped': enc_still}
