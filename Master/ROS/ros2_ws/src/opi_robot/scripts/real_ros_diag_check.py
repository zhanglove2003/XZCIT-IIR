#!/usr/bin/env python3
"""在真实 ROS 2 环境中验证诊断等级链路。

本机没有 ROS 2，这个脚本要在目标机（Orange Pi，ROS 2 Humble）上跑。
它不连接雷达、不发布 /cmd_vel、不打开摄像头。

验证内容：
  1. 真实 DiagnosticStatus.level 能接受 encode_level() 的结果；
  2. 真实雷达发布端的等级字段类型正确（用假设备参数，不连硬件）；
  3. 真实 FusionNode 能解码真实诊断消息，健康/过期/断线三种状态判断正确；
  4. 真实 web sensors.parse_diagnostic 能解析真实消息。

用法（在香橙派上，脚本随包安装到 install/lib/opi_robot/）：

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 ~/ros2_ws/install/lib/opi_robot/real_ros_diag_check.py --ros-domain-id 137

**必须指定隔离域。** 脚本会在当前 ROS 域里建真实节点并发布诊断，
默认域（0）上现场融合节点会收到这些测试消息。因此：

  - 缺少隔离配置时脚本直接 exit 2，不建任何节点；
  - 推荐用 `--ros-domain-id N`（N > 0），或
    `ROS_DOMAIN_ID=N OPI_TEST_ISOLATED=1`；
  - 域 213 是历史审核用过的，仍可用但会提示可能与他人共用。

结果打印为 PASS/FAIL 行，最后给汇总。退出码：
  0 全部通过；1 有失败；2 环境不具备或缺隔离配置；3 脚本自身错误。
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ros_isolation import require_isolated_domain, describe_ros_graph_ops  # noqa: E402

RESULTS = []

# 第八次审核阻断 2：只比"已记录项有没有失败"挡不住"少跑一项"。
# 这里显式列出必须有结论的检查名称；缺项或重复一律判失败。
REQUIRED_CHECKS = (
    'real_field_accepts_bytes',
    'real_field_rejects_int',
    'roundtrip_all_levels',
    'lidar_publish_diag_bytes',
    'fusion_decodes_real_diagnostics',
    'web_parse_real_diagnostics',
)


def check(name, fn):
    try:
        detail = fn()
        RESULTS.append((name, True, detail or ''))
        print('PASS %s %s' % (name, detail or ''))
    except Exception as exc:                      # noqa: BLE001
        RESULTS.append((name, False, '%s: %s' % (type(exc).__name__, exc)))
        print('FAIL %s %s: %s' % (name, type(exc).__name__, exc))


def main(argv=None):
    # 先确认隔离域，再碰 rclpy。不满足就退出，绝不建节点。
    rest = require_isolated_domain(argv)
    print(describe_ros_graph_ops())
    if '--help' in rest or '-h' in rest:
        print(__doc__)
        return 0

    try:
        import rclpy                                    # noqa: F401
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
        from sensor_msgs.msg import LaserScan
    except ImportError as exc:
        print('SKIP 环境不具备：%s' % exc)
        print('     需要 source /opt/ros/humble/setup.bash 并确认已安装'
              ' diagnostic_msgs')
        return 2

    sys.path.insert(0, os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    from opi_robot import diagnostic_level as DL
    from opi_robot.fusion_node import failclosed as F
    from opi_robot.fusion_node import fusion_node as FN
    from opi_robot.web_node import sensors as S

    # ---- 1. 真实消息字段类型 ----
    def real_field_accepts_bytes():
        for lv in (0, 1, 2, 3):
            st = DiagnosticStatus()
            st.name = 'probe'
            st.level = DL.encode_level(lv)
            raw = st.level
            assert isinstance(raw, (bytes, bytearray)), type(raw)
            assert len(raw) == 1, len(raw)
            assert raw[0] == lv, (raw, lv)
        return 'level 接受单字节 bytes (OK/WARN/ERROR/STALE)'
    check('real_field_accepts_bytes', real_field_accepts_bytes)

    def real_field_rejects_int():
        st = DiagnosticStatus()
        st.name = 'probe'
        try:
            st.level = 2
        except Exception as exc:                     # noqa: BLE001
            return '整数赋值被拒绝：%s' % type(exc).__name__
        raise AssertionError('真实字段接受了整数，与审核结论不符')
    check('real_field_rejects_int', real_field_rejects_int)

    def roundtrip_all_levels():
        out = []
        for lv in (0, 1, 2, 3):
            st = DiagnosticStatus()
            st.level = DL.encode_level(lv)
            got = DL.decode_level(st.level)
            assert got == lv, (got, lv)
            out.append('%.0f' % lv)
        return '往返一致 ' + '/'.join(out)
    check('roundtrip_all_levels', roundtrip_all_levels)

    # ---- 2. 真实雷达发布端（假设备参数，不连硬件） ----
    def lidar_publish_type():
        """在隔离域里实例化真实雷达节点，确认 publish_diag() 不抛异常。

        设备参数指向不存在的路径，节点不会连上串口，也不会发布 /scan。
        定时器用 halt_timers() 取消（节点已保存句柄），节点会被销毁，
        context 在 finally 里 shutdown。
        """
        import rclpy
        from opi_robot.lidar_node import lidar_node as LN
        from opi_robot.lidar_node import health as H

        context = rclpy.Context()
        rclpy.init(context=context,
                   args=['--ros-args', '-p', 'lidar_port:=/tmp/no-such-dev'])
        node = None
        try:
            # context 必须与 rclpy.init() 相同，否则节点落在默认
            # 未初始化的 context 上，抛 NotInitializedException。
            node = LN.LidarNode(context=context)
            stopped = node.halt_timers()
            assert stopped == 2, '应取消 poll_timer 与 diag_timer，实际 %d' % stopped

            probe = DiagnosticStatus()
            probe.name = 'lidar'
            probe.level = DL.encode_level(H.diagnostic_level(H.STALE))
            assert isinstance(probe.level, bytes), type(probe.level)
            assert len(probe.level) == 1, len(probe.level)

            node.publish_diag()   # 走真实发布路径，确认不抛异常
            return 'level=%r 已取消定时器=%d' % (probe.level, stopped)
        finally:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:                    # noqa: BLE001
                    pass
            try:
                rclpy.shutdown(context=context)
            except Exception:                        # noqa: BLE001
                pass
    check('lidar_publish_diag_bytes', lidar_publish_type)

    # ---- 3. 融合节点解码真实消息 ----
    class _Probe:
        pass

    def fusion_decode():
        """用真实 ROS 消息驱动一个**真实构造**的 FusionNode。

        第六次审核阻断 1：旧版用 `FusionNode.__new__` 手工拼对象，
        新增字段（lost_seq 等）没有初始化，真实 ROS 上直接 AttributeError，
        而且那种写法永远测不到构造函数、定时器和 context 清理。

        这里用独立 context 建节点：init / 构造 / 销毁 / shutdown 同一个。
        """
        import rclpy

        ctx = rclpy.Context()
        rclpy.init(context=ctx, args=['--ros-args',
                                      '-p', 'lidar_port:=/tmp/no-such-lidar'])
        node = None
        try:
            node = FN.FusionNode(context=ctx)
            node._script_now = node.now_s()
            node.now_s = lambda: node._script_now

            def diag(level, state):
                st = DiagnosticStatus()
                st.name = 'lidar'
                st.message = state
                st.level = DL.encode_level(level)
                from diagnostic_msgs.msg import KeyValue
                kv = KeyValue()
                kv.key = 'state'
                kv.value = state
                st.values = [kv]
                arr = DiagnosticArray()
                arr.status = [st]
                return arr

            def scan():
                m = LaserScan()
                m.angle_min = 0.0
                m.angle_increment = math.pi / 180.0
                m.angle_max = m.angle_increment * 359
                m.range_min = 0.02
                m.range_max = 12.0
                m.ranges = [2.0] * 360
                return m

            t0 = node._script_now

            # 1) 有新扫描 + OK/healthy -> 可信
            node.on_scan(scan())
            node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
            ok_trusted, ok_why = node.lidar_trusted(node._script_now)
            assert ok_trusted is True, (ok_trusted, ok_why)

            # 2) 不可信诊断到达时立刻锁存边界（不等下一次判定）
            node.on_diagnostics(diag(DL.LEVEL_ERROR, 'disconnected'))
            assert node.lost_seq is not None, '不可信诊断未锁存 lost_seq'
            bad_trusted, bad_why = node.lidar_trusted(node._script_now)
            assert bad_trusted is False and bad_why.startswith('lidar_'), bad_why

            # 3) 回到 healthy 但没有新扫描 -> 不能恢复
            node.on_diagnostics(diag(DL.LEVEL_OK, 'healthy'))
            gap_trusted, gap_why = node.lidar_trusted(node._script_now)
            assert gap_trusted is False, (gap_trusted, gap_why)
            assert gap_why == 'recovery_no_new_scan', gap_why

            # 4) 失信边界之后来了新扫描 -> 恢复
            node._script_now = t0 + 0.1
            node.on_scan(scan())
            rec_trusted, rec_why = node.lidar_trusted(node._script_now)
            assert rec_trusted is True, (rec_trusted, rec_why)

            # 5) 诊断超时 -> 不可信
            node._script_now = t0 + 10.0
            stale_trusted, stale_why = node.lidar_trusted(node._script_now)
            assert stale_trusted is False, (stale_trusted, stale_why)
            assert stale_why == 'diagnostics_stale', stale_why
            return 'healthy=%s error=%s gap=%s recovered=%s stale=%s' % (
                ok_why, bad_why, gap_why, rec_why, stale_why)
        finally:
            if node is not None:
                try:
                    node.halt_timers()
                except Exception:                    # noqa: BLE001
                    pass
                try:
                    node.destroy_node()
                except Exception:                    # noqa: BLE001
                    pass
            try:
                rclpy.shutdown(context=ctx)
            except Exception:                        # noqa: BLE001
                pass

    # 第七次审核阻断 1：函数定义完必须调用，否则"5 项全过"里根本
    # 没有融合检查。删除这一行，本脚本就少一项，测试会立刻失败。
    check('fusion_decodes_real_diagnostics', fusion_decode)

    # ---- 4. 网页解析真实消息 ----
    def web_parse():
        st = DiagnosticStatus()
        st.name = 'lidar'
        st.message = 'healthy'
        st.level = DL.encode_level(DL.LEVEL_OK)
        from diagnostic_msgs.msg import KeyValue
        for k, v in (('state', 'healthy'), ('reason', 'ok'),
                     ('age_s', '0.10'), ('coverage', '1.0')):
            kv = KeyValue()
            kv.key = k
            kv.value = v
            st.values.append(kv)
        arr = DiagnosticArray()
        arr.status = [st]
        d = S.parse_diagnostic(arr)
        assert d is not None, 'parse_diagnostic 返回 None'
        assert d['level'] == 0, d['level']
        assert d['state'] == 'healthy', d['state']
        assert S.level_to_state(d['level']) == 'ok'
        return 'level=%r state=%s' % (d['level'], d['state'])
    check('web_parse_real_diagnostics', web_parse)

    # ---- 汇总 ----
    print()
    failed = [r for r in RESULTS if not r[1]]
    ran = [r[0] for r in RESULTS]
    missing = [n for n in REQUIRED_CHECKS if n not in ran]
    duplicated = sorted(n for n in set(ran) if ran.count(n) > 1)

    print('共 %d 项，通过 %d，失败 %d'
          % (len(RESULTS), len(RESULTS) - len(failed), len(failed)))
    print('必需检查 %d 项' % len(REQUIRED_CHECKS))
    for name, ok, detail in RESULTS:
        if not ok:
            print('  失败 %s: %s' % (name, detail))
    if missing:
        print('!! 缺少必需检查（不能当作通过）：%s' % ', '.join(missing))
    if duplicated:
        print('!! 必需检查重复执行（结论不可信）：%s' % ', '.join(duplicated))
    if missing or duplicated:
        return 1
    return 0 if not failed else 1


if __name__ == '__main__':
    sys.exit(main())
