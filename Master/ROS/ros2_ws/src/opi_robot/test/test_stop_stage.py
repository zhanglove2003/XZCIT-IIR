"""B8：「停」的三处口径统一（雷达/超声消费侧）。

整树复核 R1 阻塞项 B8：页面与日志对"停"有三套互不通气的口径——
1. 命令层是否发了零速（fusion_node 本拍 cmd_vel 是否为零）；
2. 是否真的把停命令送出（本拍是否 publish 了 Twist，即"等效的串口
   写入"一拍：fusion 是唯一判定点，零速 Twist 每拍必发；
   enable_cmd_vel=False 时连发都没有，必须如实标 unissued）；
3. 车辆实际停住（底盘回传编码器四轮全 0 的证据，fresh 与否分开记）。

修复前：/drive_safety 只有 allowed/reason（"该不该动"），obstacle_report
只有 action/linear（"打算怎么动"），页面没有任何一处能同时回答
"发了零速没有 / 停命令有没有送出去 / 车被回传证实停了没有"。
本测试先把缺口钉住：三阶段状态必须出现在 /drive_safety 与
obstacle_report 的 `stop_stage` 字段里。
"""

import json
import math

import fake_ros  # 由 conftest 装载到 sys.modules
from opi_robot.fusion_node.fusion_node import FusionNode


def scan(ranges):
    msg = fake_ros.LaserScan(ranges=list(ranges), angle_min=0.0,
                             angle_increment=2 * math.pi / len(ranges),
                             angle_max=2 * math.pi - 2 * math.pi / len(ranges),
                             range_min=0.02, range_max=12.0)
    return msg


def all_around(distance, half=30):
    data = {str(d): distance for d in range(-half, half + 1)}
    for center in (90, -90):
        for d in range(center - half, center + half + 1):
            data[str(d)] = distance
    ranges = [math.inf] * 360
    for key, val in data.items():
        ranges[int(key) % 360] = val
    return scan(ranges)


def diag(level, state='healthy'):
    return fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(level=level, name='lidar',
                                       message=state,
                                       values=[fake_ros.KeyValue(
                                           key='state', value=state)])
    ])


def diag_encoders(values=(0, 0, 0, 0)):
    kv = [fake_ros.KeyValue(key='encoder_%s_reported' % n, value=str(v))
          for n, v in zip('abcd', values)]
    return fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus.make(
            level=fake_ros.DiagnosticStatus.OK, name='chassis',
            message='ok', values=kv)])


def evidence_json(epoch=7, count=100):
    return json.dumps({'v': 1, 'epoch': epoch, 'frames_ok': count},
                      separators=(',', ':'))


def make_node(front=3.0, encoders=(0, 0, 0, 0), cmd_vel=True):
    node = FusionNode()
    node.enable_cmd_vel = cmd_vel
    node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
    node.on_diagnostics(diag_encoders(encoders))
    node.on_scan(all_around(front))
    node.on_ultrasonic(fake_ros.Range(range=0.8, min_range=0.02,
                                      max_range=4.5))
    node.on_evidence(fake_ros.String(data=evidence_json(count=100)))
    node.on_evidence(fake_ros.String(data=evidence_json(count=105)))
    node.decide()
    return node


def last_safety(node):
    msgs = node.publishers['drive_safety'].messages
    assert msgs, '未发布 drive_safety'
    return json.loads(msgs[-1].data)


def last_report(node):
    msgs = node.publishers['obstacle_report'].messages
    assert msgs, '未发布 obstacle_report'
    return json.loads(msgs[-1].data)


class TestStopStageInSafetyPayload:
    """/drive_safety 必须带 stop_stage：三件事分开记录。"""

    def test_keys_present(self):
        node = make_node()
        payload = last_safety(node)
        assert 'stop_stage' in payload, (
            '/drive_safety 缺 stop_stage：三处"停"口径未落到同一处')

    def test_stopped_vehicle_all_zero_confirmed(self):
        # 命令零速（巡航关）→ 零速已发 + 已送出 + 回传证实停住。
        node = make_node(encoders=(0, 0, 0, 0))
        stage = last_safety(node)['stop_stage']
        assert stage['zero_issued'] is True
        assert stage['issued'] is True
        assert stage['vehicle_stopped'] is True

    def test_vehicle_moving_reported_not_stopped(self):
        # 回传四轮速度非零：即使命令是零，也不能宣称车停住了。
        node = make_node(encoders=(120, 120, 120, 120))
        stage = last_safety(node)['stop_stage']
        assert stage['zero_issued'] is True
        assert stage['issued'] is True
        assert stage['vehicle_stopped'] is False

    def test_no_encoder_evidence_is_unknown(self):
        # 从未收到底盘诊断：停没停是"不知道"，不是 True。
        node = FusionNode()
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        node.on_scan(all_around(3.0))
        node.on_ultrasonic(fake_ros.Range(range=0.8, min_range=0.02,
                                          max_range=4.5))
        node.on_evidence(fake_ros.String(data=evidence_json(count=100)))
        node.on_evidence(fake_ros.String(data=evidence_json(count=105)))
        node.decide()
        stage = last_safety(node)['stop_stage']
        assert stage['vehicle_stopped'] is None

    def test_cmd_vel_disabled_issued_false(self):
        # enable_cmd_vel=False：连零速 Twist 都没发出去，issued 必须 False，
        # 不能把"没发命令"伪装成"已发停车命令"。
        node = FusionNode()
        node.enable_cmd_vel = False
        node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
        node.on_diagnostics(diag_encoders([0, 0, 0, 0]))
        node.on_scan(all_around(3.0))
        node.on_ultrasonic(fake_ros.Range(range=0.8, min_range=0.02,
                                          max_range=4.5))
        node.on_evidence(fake_ros.String(data=evidence_json(count=100)))
        node.on_evidence(fake_ros.String(data=evidence_json(count=105)))
        node.decide()
        stage = last_safety(node)['stop_stage']
        assert stage['zero_issued'] is True
        assert stage['issued'] is False


class TestStopStageInReport:
    """obstacle_report 同步携带同一份 stop_stage（页面从一处读）。"""

    def test_report_carries_stop_stage(self):
        node = make_node()
        report = last_report(node)
        assert 'stop_stage' in report, (
            'obstacle_report 缺 stop_stage：页面仍需拼三处口径')

    def test_stale_encoder_stops_being_evidence(self):
        # 编码器诊断停更 → 回传证据过期，vehicle_stopped 回落 None。
        node = make_node(encoders=(0, 0, 0, 0))
        # 推进节点时钟超过静止证据时限
        node.get_clock().advance(10.0)
        node.decide()
        stage = last_safety(node)['stop_stage']
        assert stage['vehicle_stopped'] is None
