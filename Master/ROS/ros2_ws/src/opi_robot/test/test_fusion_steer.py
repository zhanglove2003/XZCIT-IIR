"""[B3 2026-11-15] 航向保持（直线行走）测试：P 控制器纯函数 + 节点级行为。

符号链（实测定案，c50c 符号链记录）：
- IMU gz 为 REP-103 正向（+ = 逆时针左转）；
- 命令 angular.z 与 REP-103 反向（+ = 顺时针右转），量纲 = 轮差速 m/s。
所以"车头右偏（yaw 掉到基准下）→ 命令为负（左转回正）"。

安全契约不变：
- 巡航关/无基准/IMU 断流/停车判定 → 转向一律 0，绝不凭旧数据打方向。
"""

import json
import math

import pytest

import fake_ros  # 由 conftest 装载到 sys.modules
from opi_robot.fusion_node.fusion_node import (FusionNode, heading_correction,
                                               wrap_angle)


# ---------- 纯函数 ----------

def test_wrap_angle_normalizes():
    assert wrap_angle(0.1) == pytest.approx(0.1)
    assert wrap_angle(3.5) == pytest.approx(3.5 - 2 * math.pi)
    assert wrap_angle(-3.5) == pytest.approx(-3.5 + 2 * math.pi)
    assert wrap_angle(0.0) == 0.0


def test_correction_drift_right_gives_left_command():
    """车头右偏（yaw=−0.1，顺时针掉了）→ 命令为负 = 左转回正。

    这是 B3 最要紧的符号断言：IMU 正向与命令反向两级符号必须
    相互抵消成"误差正 → 命令负"。
    """
    cmd, err = heading_correction(0.0, -0.1, gain=2.0, max_cmd=0.5,
                                  deadband=0.01)
    assert err == pytest.approx(0.1)
    assert cmd == pytest.approx(-0.2)


def test_correction_drift_left_gives_right_command():
    cmd, _ = heading_correction(0.0, 0.1, gain=2.0, max_cmd=0.5,
                                deadband=0.01)
    assert cmd == pytest.approx(0.2)


def test_correction_wraps_across_pi():
    """基准 3.0、当前 -3.2：最短路径是顺时针 0.083（不是逆时针 6.2）。

    err 为负 = 车头已逆时针越过基准 → 命令为正 = 右转回正。
    """
    cmd, err = heading_correction(3.0, -3.2, gain=2.0, max_cmd=0.5,
                                  deadband=0.01)
    assert err == pytest.approx(6.2 - 2 * math.pi)
    assert cmd == pytest.approx(-2.0 * (6.2 - 2 * math.pi))


def test_correction_clamped():
    cmd, _ = heading_correction(0.0, -1.5, gain=2.0, max_cmd=0.5,
                                deadband=0.01)
    assert cmd == pytest.approx(-0.5)


def test_correction_deadband_zero():
    """死区内回 0：IMU 噪声（~0.0005 rad/s 积分）不引发画龙。"""
    cmd, err = heading_correction(0.0, -0.005, gain=2.0, max_cmd=0.5,
                                  deadband=0.01)
    assert cmd == 0.0
    assert err == pytest.approx(0.005)


# ---------- 节点级 ----------

def imu_msg(gz, sec, nanosec=0):
    m = fake_ros.Imu()
    m.header.stamp = fake_ros.Time(sec=sec, nanosec=nanosec)
    m.angular_velocity.z = gz
    return m


def diag_encoders(values=(0, 0, 0, 0)):
    """编码器诊断（桥把四轮瞬时轮速放 encoder_{a..d}_reported，轮停回 0）。"""
    kv = [fake_ros.KeyValue(key='encoder_%s_reported' % n, value=str(v))
          for n, v in zip('abcd', values)]
    return fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(
            level=fake_ros.DiagnosticStatus.OK, name='chassis',
            message='ok', values=kv)])


def park(node):
    """让车进入"被证实静止"：编码器全 0。"""
    node.on_diagnostics(diag_encoders([0, 0, 0, 0]))


def feed_bias(node, gz=0.0):
    """停稳（编码器全 0）后喂满 20 个静止样本（20Hz×1s），零偏收敛。"""
    park(node)
    for i in range(20):
        node.on_imu(imu_msg(gz, i, (i * 50_000_000) % 1_000_000_000))
    assert node.imu_bias is not None


def feed_rate(node, gz, steps, dt_ns=50_000_000):
    """以固定角速度喂 steps 步（每步 0.05s），返回积分出的航向。

    注意时间戳只在纳秒上累加（同一秒内）——秒与纳秒同时累加会让
    dt 变成 1.05s，触发 on_imu 的跨步保护被整体跳过。
    """
    for i in range(steps):
        node.on_imu(imu_msg(gz, 120, i * dt_ns))
    return node.imu_yaw


def make_node():
    return FusionNode()


def last_report(node):
    msgs = node.publishers['obstacle_report'].messages
    assert msgs, '未发布 obstacle_report'
    return json.loads(msgs[-1].data)


class TestBiasCalibration:
    def test_static_bias_captured(self):
        node = make_node()
        feed_bias(node, gz=0.002)
        assert node.imu_bias == pytest.approx(0.002)

    def test_moving_car_never_converges(self):
        """编码器非零（车在动/被手推）→ 零偏绝不刷新（用户定则）。"""
        node = make_node()
        node.on_diagnostics(diag_encoders([120, 0, 0, 0]))   # 一轮在转
        for i in range(40):
            node.on_imu(imu_msg(0.0, i, (i * 50_000_000) % 1_000_000_000))
        assert node.imu_bias is None
        # 停稳后 1 秒内收敛
        park(node)
        for i in range(20, 40):
            node.on_imu(imu_msg(0.0, i, (i * 50_000_000) % 1_000_000_000))
        assert node.imu_bias is not None

    def test_steady_rotation_rejected_by_mean_guard(self):
        """匀速旋转骗得过方差，但骗不过均值上限（|均值|<0.05）。

        编码器全 0（车被抬着转）+ gz 恒 0.3：方差 0、均值 0.3——
        停着的车零偏不可能是 0.3，拒绝刷新。
        """
        node = make_node()
        park(node)
        for i in range(40):
            node.on_imu(imu_msg(0.3, i, (i * 50_000_000) % 1_000_000_000))
        assert node.imu_bias is None

    def test_no_encoder_evidence_never_converges(self):
        """从未收到编码器诊断 → 静止无从证实 → 不刷新（保守）。"""
        node = make_node()
        for i in range(40):
            node.on_imu(imu_msg(0.0, i, (i * 50_000_000) % 1_000_000_000))
        assert node.imu_bias is None

    def test_bias_refreshes_while_parked(self):
        """零偏跟随：停着的时候窗口持续刷新（顺带跟温漂）。"""
        node = make_node()
        feed_bias(node, gz=0.001)
        first = node.imu_bias
        for i in range(40, 60):
            node.on_imu(imu_msg(0.003, i, (i * 50_000_000) % 1_000_000_000))
        assert node.imu_bias == pytest.approx(0.003)
        assert node.imu_bias != first

    def test_bias_frozen_during_cruise(self):
        """巡航中冻结：行驶的 gz 是真旋转，绝不混进零偏。"""
        node = make_node()
        feed_bias(node, gz=0.0)
        node.on_cruise(fake_ros.Bool(data=True))
        for i in range(40, 60):
            node.on_imu(imu_msg(0.5, i, (i * 50_000_000) % 1_000_000_000))
        assert node.imu_bias == pytest.approx(0.0)

    def test_integration_subtracts_bias(self):
        node = make_node()
        feed_bias(node, gz=0.002)
        yaw = feed_rate(node, gz=0.5, steps=10)
        # 第一步只铺时间戳不积分（没有前一步算 dt），10 步积 9 步
        assert yaw == pytest.approx((0.5 - 0.002) * 0.45, abs=1e-9)


class TestReferenceCapture:
    def test_rising_edge_locks_reference(self):
        node = make_node()
        feed_bias(node)
        feed_rate(node, gz=0.3, steps=10)          # 转过一段
        locked = node.imu_yaw
        node.on_cruise(fake_ros.Bool(data=True))
        assert node.cruise_yaw_ref == pytest.approx(locked)

    def test_engage_while_moving_no_reference(self):
        """用户定则：开启巡航时车在动 → 不锁基准，本次没有航向保持。

        巡航的直行照常（linear 正常发），只是不打方向——宁可走不直，
        不可带着脏零偏/未知基准开车。
        """
        node = make_node()
        node.enable_cmd_vel = True
        node.on_diagnostics(diag_ok())
        std_scan = clear_scan()
        feed_bias(node)
        stamp = node.get_clock().now_msg()   # 帧时间戳盖在时间推进之后
        std_scan.header.stamp = fake_ros.Time(sec=stamp.sec,
                                              nanosec=stamp.nanosec)
        node.on_scan(std_scan)
        node.on_ultrasonic(fake_ros.Range(
            range=0.8, min_range=0.02, max_range=4.5))    # 门 fail-closed
        node.body_geometry['half_length'] = 0.25
        node.body_geometry['half_width'] = 0.15
        node.on_diagnostics(diag_encoders([80, 80, 80, 80]))  # 在动
        feed_rate(node, gz=0.3, steps=10)
        node.on_cruise(fake_ros.Bool(data=True))
        assert node.cruise_yaw_ref is None
        # 最后一次喂扫描在 10 步 IMU 之前，补一帧新鲜的（10Hz 实车常态）
        node.on_diagnostics(diag_ok())
        fresh_scan_frame(node)
        node.on_ultrasonic(fake_ros.Range(
            range=0.8, min_range=0.02, max_range=4.5))
        node.body_geometry['half_length'] = 0.25
        node.body_geometry['half_width'] = 0.15
        node.get_clock().advance(0.1)
        node.decide()
        msgs = node.publishers['auto_cmd_vel'].messages
        assert msgs[-1].linear.x == pytest.approx(0.4)
        assert msgs[-1].angular.z == 0.0

    def test_repeat_true_keeps_reference(self):
        """巡航中网页重复发 True（心跳语义）不重锁基准。"""
        node = make_node()
        feed_bias(node)
        node.on_cruise(fake_ros.Bool(data=True))
        ref = node.cruise_yaw_ref
        feed_rate(node, gz=0.3, steps=10)
        node.on_cruise(fake_ros.Bool(data=True))
        assert node.cruise_yaw_ref == pytest.approx(ref)

    def test_reopen_relocks(self):
        node = make_node()
        feed_bias(node)
        node.on_cruise(fake_ros.Bool(data=True))
        feed_rate(node, gz=0.3, steps=10)
        node.on_cruise(fake_ros.Bool(data=False))
        feed_rate(node, gz=-0.3, steps=10)
        relocked = node.imu_yaw
        node.on_cruise(fake_ros.Bool(data=True))
        assert node.cruise_yaw_ref == pytest.approx(relocked)


def diag_ok():
    return fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(
            level=fake_ros.DiagnosticStatus.OK, name='lidar',
            message='healthy',
            values=[fake_ros.KeyValue(key='state', value='healthy')])
    ])


def clear_scan(distance=3.0):
    ranges = [math.inf] * 360
    for d in range(-30, 31):
        ranges[d % 360] = distance
    for center in (90, -90):
        for d in range(center - 30, center + 31):
            ranges[d % 360] = distance
    return fake_ros.LaserScan(
        ranges=ranges, angle_min=0.0,
        angle_increment=2 * math.pi / 360,
        angle_max=2 * math.pi - 2 * math.pi / 360,
        range_min=0.02, range_max=12.0)


class TestDecideSteers:
    def _driving_node(self):
        node = make_node()
        node.enable_cmd_vel = True
        node.on_diagnostics(diag_ok())
        fresh_scan_frame(node)
        node.body_geometry['half_length'] = 0.25
        node.body_geometry['half_width'] = 0.15
        feed_bias(node)          # 含 park()：编码器全 0，零偏收敛
        # 最后喂超声（feed_bias 含时间推进，先喂会被判断流 → fail-closed 停机）
        node.on_ultrasonic(fake_ros.Range(
            range=0.8, min_range=0.02, max_range=4.5))
        node.on_cruise(fake_ros.Bool(data=True))
        return node

    def test_drift_right_steers_left(self):
        """实车场景全链路：右偏 0.18 rad → 命令 -0.36（左转回正）。

        10 步只积 9 步（第一步铺时间戳）：0.4 m/s×0.45s = 0.18 rad。
        """
        node = self._driving_node()
        feed_rate(node, gz=-0.4, steps=10)          # 顺时针掉 0.18 rad
        node.decide()
        msgs = node.publishers['auto_cmd_vel'].messages
        assert msgs[-1].linear.x == pytest.approx(0.4)
        assert msgs[-1].angular.z == pytest.approx(-0.36)

    def test_on_heading_zero_steer(self):
        node = self._driving_node()
        node.decide()                               # 没有漂移
        msgs = node.publishers['auto_cmd_vel'].messages
        assert msgs[-1].angular.z == 0.0
        assert msgs[-1].linear.x == pytest.approx(0.4)

    def test_imu_timeout_stops_cruise_until_restarted(self):
        node = self._driving_node()
        feed_rate(node, gz=-0.4, steps=10)
        node.decide()
        msgs = node.publishers['auto_cmd_vel'].messages
        assert msgs[-1].linear.x > 0.0 and msgs[-1].angular.z != 0.0

        node.imu_last_t = node.now_s() - 2.0        # 超过 imu_timeout=0.5
        node.decide()
        assert msgs[-1].linear.x == 0.0
        assert msgs[-1].angular.z == 0.0

        park(node)
        feed_rate(node, gz=0.0, steps=1)
        node.on_cruise(fake_ros.Bool(data=True))
        node.decide()
        assert msgs[-1].linear.x == 0.0
        assert msgs[-1].angular.z == 0.0

        node.on_cruise(fake_ros.Bool(data=False))
        node.on_cruise(fake_ros.Bool(data=True))
        node.decide()
        assert msgs[-1].linear.x > 0.0

    def test_imu_sample_after_timeout_cannot_hide_gap_between_decisions(self):
        node = self._driving_node()
        node.decide()
        msgs = node.publishers['auto_cmd_vel'].messages
        assert msgs[-1].linear.x > 0.0
        node.get_clock().advance(0.51)
        fresh_scan_frame(node)
        node.on_ultrasonic(fake_ros.Range(range=0.8, min_range=0.02,
                                          max_range=4.5))
        feed_rate(node, gz=0.0, steps=1)
        node.decide()
        assert msgs[-1].linear.x == 0.0
        assert last_report(node)['cruise_stop_reason'] == 'imu_timeout'

    def test_stop_action_zeroes_steer(self):
        """近障停车：转向一并归零，停车时不打方向。"""
        node = self._driving_node()
        feed_rate(node, gz=-0.4, steps=10)
        node.on_scan(clear_scan(distance=0.2))      # 前方 0.2m → stop
        node.decide()
        msgs = node.publishers['auto_cmd_vel'].messages
        assert msgs[-1].linear.x == 0.0
        assert msgs[-1].angular.z == 0.0

    def test_report_carries_steer_observation(self):
        """obstacle_report 带 steer 观测块：网页/排查"走不直"先看这。"""
        node = self._driving_node()
        feed_rate(node, gz=-0.4, steps=10)          # 掉 0.18 rad
        node.decide()
        rep = last_report(node)
        assert rep['steer'] is not None
        assert rep['steer']['err_deg'] == pytest.approx(
            math.degrees(0.18), abs=0.5)
        assert rep['steer']['cmd'] == pytest.approx(-0.36, abs=0.01)

    def test_no_cruise_no_steer(self):
        """巡航关：转向恒 0（原"角速度恒为零"契约在巡航外的延续）。"""
        node = make_node()
        node.enable_cmd_vel = True
        node.on_diagnostics(diag_ok())
        std_scan = clear_scan()
        feed_bias(node)
        stamp = node.get_clock().now_msg()   # 帧时间戳盖在时间推进之后
        std_scan.header.stamp = fake_ros.Time(sec=stamp.sec,
                                              nanosec=stamp.nanosec)
        node.on_scan(std_scan)
        feed_rate(node, gz=-0.4, steps=10)
        stamp_now(node)
        node.decide()
        msgs = node.publishers['auto_cmd_vel'].messages
        assert msgs[-1].angular.z == 0.0
        assert msgs[-1].linear.x == 0.0


# ---------- [B3 PI 2026-11-16] 积分项 ----------
#
# 实车证据：P 控制必有稳态偏差（匀速段 err 恒 +2.0~2.4°，cmd 恒
# −0.08），直行右偏 20cm 的根因。PI = 死区只门控 P 项，积分直通
# （反画龙：误差进死区后偏置仍在，不再极限环）。


class TestHeadingCorrectionPI:
    """纯函数：积分项直通 + 死区只门控 P + 总量钳位。"""

    def test_integral_passes_through_deadband(self):
        """误差进死区（P=0）→ cmd = 积分：偏置不被死区吃掉。"""
        cmd, err = heading_correction(0.0, -0.005, gain=2.0, max_cmd=0.5,
                                      deadband=0.01, integral=-0.08)
        assert err == pytest.approx(0.005)
        assert cmd == pytest.approx(-0.08)

    def test_integral_adds_to_p_outside_deadband(self):
        cmd, _ = heading_correction(0.0, -0.1, gain=2.0, max_cmd=0.5,
                                    deadband=0.01, integral=-0.05)
        assert cmd == pytest.approx(-0.05 - 0.2)

    def test_integral_opposite_side(self):
        """左偏（err 负）+ 正积分 → 同号相加。"""
        cmd, _ = heading_correction(0.0, 0.05, gain=2.0, max_cmd=0.5,
                                    deadband=0.01, integral=0.1)
        assert cmd == pytest.approx(0.1 + 0.1)

    def test_total_clamp_covers_integral(self):
        """积分 + P 超限 → 总量钳位（积分自身钳位在节点侧）。"""
        cmd, _ = heading_correction(0.0, -0.3, gain=2.0, max_cmd=0.5,
                                    deadband=0.01, integral=-0.15)
        assert cmd == pytest.approx(-0.5)
        # 死区内 P=0，积分单独超限同样被钳
        cmd, _ = heading_correction(0.0, -0.001, gain=2.0, max_cmd=0.5,
                                    deadband=0.01, integral=-0.6)
        assert cmd == pytest.approx(-0.5)


def stamp_now(node):
    """把未填（0）的扫描 header 时间戳补成节点当前时刻（等价于 10 Hz
    新帧；实测规律：同一节点时钟时刻的帧不会被判陈旧）。"""
    msg = node.latest_scan
    if msg is not None and getattr(msg, 'header', None) is not None:
        stamp = node.get_clock().now_msg()
        msg.header.stamp = fake_ros.Time(sec=stamp.sec,
                                         nanosec=stamp.nanosec)
    return msg


def fresh_scan_frame(node):
    """喂一帧"带真实时间戳"的扫描。

    fake_ros 的 LaserScan 默认 ``header.stamp = 0``（未填）。方向门按原始
    header 帧龄判定，0 时间戳在 fusion_node.raw_frame_age 里按"未填"返回
    None → stale（fail-closed）。节点时钟已走到 1 s 以上，这里把帧时间戳
    写成当前节点时钟，等价于实车 10 Hz 新帧。
    """
    msg = clear_scan()
    stamp = node.get_clock().now_msg()
    msg.header.stamp = fake_ros.Time(sec=stamp.sec, nanosec=stamp.nanosec)
    node.on_scan(msg)
    return msg


def driving_node():
    """行驶中的巡航节点：雷达清爽、零偏收敛、基准已锁、巡航开。"""
    node = make_node()
    node.enable_cmd_vel = True
    node.on_diagnostics(diag_ok())
    node.on_scan(clear_scan())
    # 2026-11-17：超声门 fail-closed + 方向门默认启用（需标定轮廓）
    node.on_ultrasonic(fake_ros.Range(range=0.8, min_range=0.02,
                                      max_range=4.5))
    node.body_geometry['half_length'] = 0.25
    node.body_geometry['half_width'] = 0.15
    feed_bias(node)
    node.on_cruise(fake_ros.Bool(data=True))
    return node


def decide_fresh(node):
    """IMU 推进之后补时间戳再判（扫描是在 IMU 推进之前喂的）。"""
    stamp_now(node)
    node.decide()
    return stamp_now(node)


def edge_msgs(node):
    return node.publishers['auto_cmd_vel'].messages


def fresh_evidence(node, moving=True):
    """重喂一轮证据（雷达+诊断+编码器）：真车上这些 10Hz 持续来，
    仿真里不重喂会被 ScanFreshness 判过期 → unknown，与实车不符。"""
    node.on_diagnostics(diag_ok())
    node.on_scan(clear_scan())
    node.on_diagnostics(diag_encoders([80, 80, 80, 80] if moving
                                      else [0, 0, 0, 0]))


class TestSteerPI:
    def _accumulated(self, rounds=6, advance=True):
        """跑出非零积分：每轮时钟 +0.1s、恒定右偏 gz=-0.02、证据新鲜。"""
        node = driving_node()
        node.decide()                       # 铺 _steer_integral_t
        for _ in range(rounds):
            if advance:
                node.get_clock().advance(0.1)
                fresh_evidence(node, moving=True)
            feed_rate(node, gz=-0.02, steps=10)
            stamp_now(node)                 # IMU 推进后补新帧时间戳（10Hz）
            node.decide()
            stamp_now(node)
        return node

    def test_accumulates_while_driving(self):
        node = self._accumulated()
        assert node.steer_integral < 0.0    # 右偏 → 负积分（左转储备）

    def test_frozen_when_body_still(self):
        """编码器全 0（停着/手推/滑转）→ 积分冻结，不吞静止期的假误差。"""
        node = driving_node()
        node.decide()
        node.get_clock().advance(0.1)
        feed_rate(node, gz=-0.02, steps=10)  # IMU 在转，编码器说没动
        node.decide()
        assert node.steer_integral == 0.0

    def test_frozen_without_clock_advance(self):
        """时钟未推进（dt=0）→ 不积分。既有用例不推时钟也靠这条保持原行为。"""
        node = driving_node()
        node.decide()
        node.on_diagnostics(diag_encoders([80, 80, 80, 80]))
        feed_rate(node, gz=-0.02, steps=10)
        node.decide()                        # now_s 没变 → dt 无效
        assert node.steer_integral == 0.0

    def test_clamped_at_integral_max(self):
        node = self._accumulated(rounds=30)
        assert node.steer_integral == pytest.approx(-0.15)
        rep = last_report(node)
        assert rep['steer']['integ'] == pytest.approx(-0.15)

    def test_reset_on_stop(self):
        node = self._accumulated()
        assert node.steer_integral < 0.0
        node.on_scan(clear_scan(distance=0.2))   # 前障 → stop
        node.decide()
        assert node.steer_integral == 0.0
        assert node._steer_integral_t is None

    def test_reset_when_cruise_inactive(self):
        node = self._accumulated()
        node.on_cruise(fake_ros.Bool(data=False))
        node.decide()
        assert node.steer_integral == 0.0

    def test_reset_on_relock(self):
        """关→开重锁基准的同时积分清零：旧积分对新基准毫无意义。"""
        node = self._accumulated()
        assert node.steer_integral < 0.0
        park(node)
        node.on_cruise(fake_ros.Bool(data=False))
        node.on_cruise(fake_ros.Bool(data=True))
        assert node.steer_integral == 0.0
        assert node.cruise_yaw_ref == pytest.approx(node.imu_yaw)

    def test_imu_timeout_clears_steering_integral(self):
        node = self._accumulated()
        assert node.steer_integral != 0.0
        node.imu_last_t = node.now_s() - 2.0
        node.decide()
        msgs = edge_msgs(node)
        assert msgs[-1].linear.x == 0.0
        assert msgs[-1].angular.z == 0.0
        assert node.steer_integral == 0.0

    def test_pi_cancels_constant_pull(self):
        """行为闭环：恒定右拉下 20s 内 err→0、cmd→拉力平衡点。

        植物模型（实测 11-15 夜匀速段反推，REP-103 gz 口径）：
        ω = −0.3·cmd − 0.024 —— 命令层反向（+ = 右转），右拉把 gz
        往负推；cmd=−0.08 恰好平衡拉力（对应 P 稳态 err=+2.3°）。
        PI 应在 ~15s 内把 err 压到死区附近、cmd 收敛到 −0.08：
        直行不再靠"稳态歪着头"换直线。
        """
        node = driving_node()
        node.decide()
        msgs = edge_msgs(node)
        t_ns = 200_000_000_000                    # 避开喂零偏的 sec=120 区
        for _ in range(200):                      # 0.1s × 200 = 20s
            cmd = msgs[-1].angular.z
            w = -0.3 * cmd - 0.024
            node.get_clock().advance(0.1)
            fresh_evidence(node, moving=True)
            for _ in range(2):                    # 20Hz 两样本 = 0.1s
                node.on_imu(imu_msg(w, t_ns // 1_000_000_000,
                                    t_ns % 1_000_000_000))
                t_ns += 50_000_000
            stamp_now(node)                       # 扫描时间戳跟随时钟（10Hz 实车）
            node.decide()
            msgs = edge_msgs(node)
        rep = last_report(node)
        assert abs(rep['steer']['err_deg']) < 0.2
        assert rep['steer']['cmd'] == pytest.approx(-0.024 / 0.3, abs=0.01)
        assert rep['steer']['integ'] == pytest.approx(-0.024 / 0.3,
                                                      abs=0.01)
