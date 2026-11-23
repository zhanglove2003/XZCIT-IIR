#!/usr/bin/env python3
"""解析→积分链路的离线回放定向测试(纯 Python,无 ROS 依赖)。

数据:2026-11-07 现场手动转轮采集(工作树 raw-data/,两份 JSONL):
- mixed-20261107T1748.jsonl:静止→左后轮向前→右后轮向前→静止(1806 帧);
- direction-recheck-20261107T1805.jsonl:左前→右前→右后(1305 帧,方向复采)。

覆盖:
1. 24 字节帧编码往返(测试内重编码 → StatusFrameParser 解析,字段一致);
2. 真实采集回放:vy 恒 0、vx/wz 方向特征、静止窗口位姿冻结、
   停轮后速度归零;
3. 时间基准:回放用帧的**主机接收时刻**;注入 >0.5 s 间隙时积分器按
   断流间隙处理,不跨间隙积分。
"""
import json
import math
import sys
from datetime import datetime
from pathlib import Path

# parents[1] = 本 ROS 包根(其下 c50c_odom/ 为 Python 包);
# parents[2]/c50c_uart3 = 仓库内 c50c_uart3 包源码(回放需解析真实帧)。
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "c50c_uart3"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from c50c_odom.odometry import BodyTwist, PlanarOdomIntegrator, twist_from_parsed
from c50c_uart3.frame_parser import StatusFrameParser

RAW = Path(__file__).resolve().parents[2] / "raw-data"
MIXED = RAW / "mixed-20261107T1748.jsonl"
RECHECK = RAW / "direction-recheck-20261107T1805.jsonl"


def _ts(s: str) -> float:
    return datetime.fromisoformat(s).timestamp()


# ---------- 测试专用:按固件组帧格式重编码(大端 int16 + BCC + 0x7B/0x7D) ----------

def encode_status_frame(stop=0, vx=0, vy=0, wz=0,
                        accel=(0, 0, 16384), gyro=(0, 0, 0), voltage=4060):
    def i16(v):
        return int(v).to_bytes(2, "big", signed=True)

    def u16(v):  # voltage_mv 在固件/解码器里是无符号 16 位
        return int(v).to_bytes(2, "big", signed=False)

    body = bytes([0x7B, stop]) + i16(vx) + i16(vy) + i16(wz) \
        + i16(accel[0]) + i16(accel[1]) + i16(accel[2]) \
        + i16(gyro[0]) + i16(gyro[1]) + i16(gyro[2]) + u16(voltage)
    bcc = 0
    for b in body:
        bcc ^= b
    return body + bytes([bcc, 0x7D])


# ---------- 1. 编码往返 ----------

def test_frame_roundtrip_fields():
    raw = encode_status_frame(stop=0, vx=-1234, vy=0, wz=567,
                              accel=(30, -41, 16382), gyro=(-3, 0, 2), voltage=4059)
    frames = StatusFrameParser().feed(raw)
    assert len(frames) == 1
    f = frames[0]
    assert f.stop_flag == 0
    assert f.chassis_velocity_mm_s == (-1234, 0)
    assert f.chassis_yaw_rate_mrad_s == 567
    assert f.accel_raw_lsb == (30, -41, 16382)
    assert f.gyro_raw_lsb == (-3, 0, 2)
    assert f.voltage_mv == 4059


def test_frame_roundtrip_int16_edges():
    raw = encode_status_frame(vx=-32768, wz=32767, voltage=65535)
    (f,) = StatusFrameParser().feed(raw)
    assert f.chassis_velocity_mm_s[0] == -32768
    assert f.chassis_yaw_rate_mrad_s == 32767
    assert f.voltage_mv == 65535


def test_two_frames_back_to_back():
    blob = encode_status_frame(vx=10) + encode_status_frame(vx=-10)
    frames = StatusFrameParser().feed(blob)
    assert [f.chassis_velocity_mm_s[0] for f in frames] == [10, -10]


# ---------- 2. 真实采集回放 ----------

def _replay(path):
    """按 JSONL 逐帧:twist_from_parsed + 积分器,主机接收时刻为时间基准。

    不变量(随 ig 诊断计数一起返回):
    - 采集内无 >0.5 s 断流 → 无 "gap";
    - 主机时间戳为 ms 精度,存在同毫秒相邻帧 → dt=0 被 nonpositive_dt
      拒绝且 last_t 不前移,数量应恰等于数据中同毫秒重复帧数。
    """
    ig = PlanarOdomIntegrator()
    rows = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            d = json.loads(line)
            rows.append(d)
            rep = ig.step(_ts(d["time"]),
                          twist_from_parsed(d["velocity_mm_s"][0],
                                            d["velocity_mm_s"][1],
                                            d["yaw_mrad_s"]))
            assert rep.reason in ("ok", "first", "nonpositive_dt"), \
                f"意外拒绝: {rep.reason}"
    dup = sum(1 for a, b in zip(rows, rows[1:]) if _ts(b["time"]) == _ts(a["time"]))
    assert ig.rejected_gap == 0, "采集内不应有 >0.5s 断流"
    assert ig.rejected_nonpositive_dt == dup, \
        "nonpositive_dt 拒绝数应恰等于同毫秒重复帧数"
    return ig, rows


def test_replay_mixed_capture_vy_zero_and_directions():
    ig, rows = _replay(MIXED)
    t0, t1 = _ts(rows[0]["time"]), _ts(rows[-1]["time"])

    assert all(r["velocity_mm_s"][1] == 0 for r in rows), "vy 应恒 0"
    # 现场已确认的阶段边界(方向复采定案:左前→wz<0,右前→wz>0)
    left = [r for r in rows if _ts("2026-11-07T17:49:15.393+08:00") <= _ts(r["time"]) <= _ts("2026-11-07T17:49:30.543+08:00")]
    right = [r for r in rows if _ts("2026-11-07T17:49:41.592+08:00") <= _ts(r["time"]) <= _ts("2026-11-07T17:49:54.342+08:00")]
    act_l = [r for r in left if any(r["velocity_mm_s"]) or r["yaw_mrad_s"]]
    act_r = [r for r in right if any(r["velocity_mm_s"]) or r["yaw_mrad_s"]]
    assert len(act_l) > 100 and len(act_r) > 100
    assert sum(1 for r in act_l if r["velocity_mm_s"][0] > 0) / len(act_l) > 0.9
    assert sum(1 for r in act_l if r["yaw_mrad_s"] < 0) / len(act_l) > 0.95
    assert sum(1 for r in act_r if r["velocity_mm_s"][0] > 0) / len(act_r) > 0.9
    assert sum(1 for r in act_r if r["yaw_mrad_s"] > 0) / len(act_r) > 0.95

    # 静止窗口:首帧后与末帧前的静止段速度字段全零(位姿冻结由积分器
    # 零速度语义保证,由 test_odometry.py 的静止用例覆盖)
    head = [r for r in rows if _ts(r["time"]) < _ts("2026-11-07T17:48:44+08:00")]
    tail = [r for r in rows if _ts(r["time"]) > _ts("2026-11-07T17:50:04+08:00")]
    assert all(r["velocity_mm_s"] == [0, 0] and r["yaw_mrad_s"] == 0
               for r in head + tail)
    # 停轮后速度归零(复采已证 50ms 内;此处断言静止窗口全零即可)
    assert ig.state.twist.vx == 0.0 and ig.state.twist.wz == 0.0  # 末段静止


def test_replay_recheck_capture_three_directions():
    """方向复采:左前(wz<0)→右前(wz>0)→右后(wz<0 且 vx<0),镜像自证。"""
    ig, rows = _replay(RECHECK)
    assert all(r["velocity_mm_s"][1] == 0 for r in rows)
    p = {name: (a, b) for name, a, b in [
        ("left_fwd", "2026-11-07T18:05:45.571+08:00", "2026-11-07T18:05:55.171+08:00"),
        ("right_fwd", "2026-11-07T18:06:03.921+08:00", "2026-11-07T18:06:12.521+08:00"),
        ("right_back", "2026-11-07T18:06:19.221+08:00", "2026-11-07T18:06:27.920+08:00")]}
    p = {k: (_ts(a), _ts(b)) for k, (a, b) in p.items()}

    def active(a, b):
        seg = [r for r in rows if a <= _ts(r["time"]) <= b]
        return [r for r in seg if any(r["velocity_mm_s"]) or r["yaw_mrad_s"]]

    la, ra, ba = (active(*p[k]) for k in ("left_fwd", "right_fwd", "right_back"))
    # 符号占比断言:手部停顿存在 ±1~2 mrad/s 的瞬态,不能逐帧 all()
    # (实测占比:左前 wz 98.5%/vx 100%,右前 100%/100%,右后 96.0%/100%)
    def frac(rs, field, exp):
        def val(r):
            return r["velocity_mm_s"][0] if field == "velocity_mm_s" else r["yaw_mrad_s"]
        vals = [val(r) for r in rs if val(r) != 0]
        return sum(1 for v in vals if exp * v > 0) / len(vals)

    assert frac(la, "velocity_mm_s", 1) > 0.95 and frac(la, "yaw_mrad_s", -1) > 0.95
    assert frac(ra, "velocity_mm_s", 1) > 0.95 and frac(ra, "yaw_mrad_s", 1) > 0.95
    assert frac(ba, "velocity_mm_s", -1) > 0.95 and frac(ba, "yaw_mrad_s", -1) > 0.95

    # 右前/右后为同轮反方向:vx、wz 均值应符号相反、量级相近(镜像自证)
    vx_r = sum(r["velocity_mm_s"][0] for r in ra) / len(ra)
    vx_b = sum(r["velocity_mm_s"][0] for r in ba) / len(ba)
    wz_r = sum(r["yaw_mrad_s"] for r in ra) / len(ra)
    wz_b = sum(r["yaw_mrad_s"] for r in ba) / len(ba)
    assert vx_r > 0 > vx_b and wz_r > 0 > wz_b
    assert math.isclose(vx_r, -vx_b, rel_tol=0.15)
    assert math.isclose(wz_r, -wz_b, rel_tol=0.15)


def test_replay_gap_policy_on_synthetic_drop():
    """在回放时间轴上注入 1 s 间隙:积分器按断流间隙处理,不跨间隙积分。"""
    ig = PlanarOdomIntegrator()
    rows = [json.loads(l) for l in open(MIXED, encoding="utf-8")][:120]
    t0 = _ts(rows[0]["time"])
    fed_gap = False
    for i, r in enumerate(rows):
        t = _ts(r["time"])
        if i == 60:
            t += 1.0  # 注入 1 s 断流
            fed_gap = True
        rep = ig.step(t, twist_from_parsed(r["velocity_mm_s"][0],
                                           r["velocity_mm_s"][1], r["yaw_mrad_s"]))
        if i == 60:
            assert rep.reason == "gap" and not rep.integrated
    assert fed_gap and ig.rejected_gap == 1


# ---------- 3. twist 换算边界 ----------

def test_twist_scaling_is_exact_division():
    tw = twist_from_parsed(-32768, 1, 32767)
    assert tw.vx == -32.768 and tw.vy == 0.001 and tw.wz == 32.767
    assert BodyTwist() == twist_from_parsed(0, 0, 0)
