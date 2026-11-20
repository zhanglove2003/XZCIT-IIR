#!/usr/bin/env python3
"""C50C 平面里程计积分器(候选代码,离线验证版)。

**状态:候选,不发布 /odom,不发 TF,不接建图节点。**

输入是"已解析的速度 + 主机接收时刻",不读串口(全系统仍只有 UART3 节点
一个串口读取者,本模块与串口完全解耦)。

字段依据(均绑定固件源码,详见 task-results/c50c-odom-prep.md 字段来源表):

- 速度字段来自固件 ``rear_odom_step()``(REAR_WHEEL_DEBUG + ENCODER_DIAG +
  _4WD_CAR 编译配置,`data_task.c:94-116`):后轮编码器原始累计计数
  (left=软件 B 通道, right=软件 C 通道,C 通道固件内取负)差分得到
  vx(mm/s)与 wz(mrad/s);**vy 恒为 0**(``rear_odom_result_t.vy_mm_s``
  从未被赋值,`rear_odom.h:47` 整体清零后无赋值路径)。
- 单位换算只有 ÷1000( mm/s→m/s、mrad/s→rad/s),是精确换算,非标定参数。
- 角速度说明:帧内有两个"角速度"——offset 6 的 ``chassis_yaw_rate`` 来自
  **编码器差速解算**(本模块用这个);offset 18 的 ``gyro_raw_lsb[2]`` 来自
  **IMU 陀螺**(且 FlagStop≠0 时被固件钳零)。二者不可混淆;本模块默认
  使用编码器解算值,若未来想改用陀螺值,必须显式传入而不是改符号表。
- 时间:帧内没有采样时间戳,全部时间来自**主机接收时刻**(调用方提供)。
  固件侧 20 Hz 定时发送(data_task.c:53),主机间隔仅是接收间隔的近似。

数值方法:对常值机体twist (vx, vy, wz) 用 SE(2) 指数映射精确积分
(不是欧拉折线),wz≈0 时退化为直线式。测试用细步长数值积分交叉验证。

时间间隔策略(与固件 ``rear_odom_step`` 的 dt>500ms 丢弃语义对齐):

- dt <= 0:拒绝本样本,位姿不变,``last_t`` 不前移(防负间隔连锁);
- dt > ``max_interval_s``:视为断流间隙,**不跨间隙积分**,位姿不变,
  ``last_t`` 重置为当前时刻(下一样本重新起算);
- 其余:正常积分。

固件侧未验证项(轮径、轮距、编码器方向、102000 counts/圈为暂定值、
yaw 符号约定)不在本模块内——本模块只做 SI 单位积分,正确性边界见
交付报告"接入 /odom 前的验证清单"。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

# 与固件 rear_odom_step 的 dt>500u 丢弃语义一致(rear_odom.h:52)
DEFAULT_MAX_INTERVAL_S = 0.5
# 低于该阈值视为零角速度,走直线精确式(避免除零);rad/s
_ZERO_WZ_EPS = 1e-9


@dataclass(frozen=True)
class BodyTwist:
    """机体坐标系速度,SI 单位(REP-103:x 前、y 左、z/偏航逆时针为正)。

    注意:符号约定是 ROS 侧假设;固件输出的物理正方向(编码器安装方向、
    C 通道取负的实际含义)未经真机验证,见交付报告缺口表。
    """

    vx: float = 0.0  # m/s
    vy: float = 0.0  # m/s(当前固件恒 0,保留以支持未来麦轮/侧移)
    wz: float = 0.0  # rad/s


@dataclass(frozen=True)
class OdomState:
    """平面里程计状态:x, y (m),yaw (rad,规整到 (-pi, pi]),最新速度 SI。"""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    twist: BodyTwist = BodyTwist()


@dataclass(frozen=True)
class StepReport:
    """单步处理结果,供测试与将来的诊断话题使用。"""

    state: OdomState
    integrated: bool        # 本样本是否发生了位姿积分
    reason: str             # "ok" | "first" | "nonpositive_dt" | "gap"


def twist_from_parsed(vx_mm_s: int, vy_mm_s: int, wz_mrad_s: int) -> BodyTwist:
    """已解析帧字段 → SI 机体twist。

    固件发送即 mm/s 与 mrad/s(``rear_odom.h`` 的 ``vx_mm_s``/``wz_mrad_s``
    经 ``data_task.c`` buffer[2..7] 大端直发),换算只有 ÷1000。
    """
    return BodyTwist(vx=vx_mm_s / 1000.0, vy=vy_mm_s / 1000.0, wz=wz_mrad_s / 1000.0)


def wrap_angle(a: float) -> float:
    """把角度规整到 (-pi, pi]。"""
    return _wrap(a)


def _wrap(a: float) -> float:
    # math.remainder 已把结果规整到 [-pi, pi];只需把 -pi 归一到 +pi 保持
    # 区间 (-pi, pi] 一致性(测试有定向用例)。
    r = math.remainder(a, 2.0 * math.pi)
    if r <= -math.pi:
        r += 2.0 * math.pi
    return r


def integrate_twist(state: OdomState, twist: BodyTwist, dt: float) -> OdomState:
    """常值机体twist 在 dt 内的精确平面积分(SE(2) 指数映射)。

    运动学:xdot = vx·cos(yaw) − vy·sin(yaw),ydot = vx·sin(yaw) + vy·cos(yaw),
    yawdot = wz。对常值 twist 有闭式解;wz≈0 时为直线。
    """
    if abs(twist.wz) < _ZERO_WZ_EPS:
        c, s = math.cos(state.yaw), math.sin(state.yaw)
        return replace(
            state,
            x=state.x + (twist.vx * c - twist.vy * s) * dt,
            y=state.y + (twist.vx * s + twist.vy * c) * dt,
            yaw=state.yaw,
            twist=twist,
        )
    th0 = state.yaw
    th1 = th0 + twist.wz * dt
    k_vx = twist.vx / twist.wz
    k_vy = twist.vy / twist.wz
    s0, c0 = math.sin(th0), math.cos(th0)
    s1, c1 = math.sin(th1), math.cos(th1)
    new_x = state.x + k_vx * (s1 - s0) + k_vy * (c1 - c0)
    new_y = state.y - k_vx * (c1 - c0) + k_vy * (s1 - s0)
    return replace(state, x=new_x, y=new_y, yaw=_wrap(th1), twist=twist)


class PlanarOdomIntegrator:
    """增量式平面里程计积分器。

    用法:每收到一帧已解析状态,把主机接收时刻与解析出的 SI 速度喂给
    :meth:`step`。首个样本只建立时间基准与速度,不产生位移。
    """

    def __init__(self, max_interval_s: float = DEFAULT_MAX_INTERVAL_S) -> None:
        if not max_interval_s > 0.0:
            raise ValueError("max_interval_s 必须为正")
        self._max_interval = max_interval_s
        self._state = OdomState()
        self._last_t: float | None = None
        # 诊断计数:gap/非正间隔/显式重置次数,供接入方暴露为诊断字段
        self.rejected_nonpositive_dt = 0
        self.rejected_gap = 0
        self.accepted_steps = 0
        self.explicit_resets = 0

    @property
    def state(self) -> OdomState:
        return self._state

    def step(self, t_receive: float, twist: BodyTwist) -> StepReport:
        """喂入一个样本:t_receive 为主机接收时刻(秒,单调时钟)。

        返回新状态与该步的处理类别。任何被拒样本都不改变位姿。
        """
        if self._last_t is None:
            self._last_t = t_receive
            self._state = replace(self._state, twist=twist)
            return StepReport(self._state, integrated=False, reason="first")

        dt = t_receive - self._last_t
        if dt <= 0.0:
            # 时间倒退/重复:拒绝且不前移 last_t,防止负 dt 连锁破坏积分
            self.rejected_nonpositive_dt += 1
            return StepReport(self._state, integrated=False, reason="nonpositive_dt")
        if dt > self._max_interval:
            # 断流间隙:不跨间隙积分(间隙内速度未知),重置时间基准
            self.rejected_gap += 1
            self._last_t = t_receive
            self._state = replace(self._state, twist=twist)
            return StepReport(self._state, integrated=False, reason="gap")

        self._last_t = t_receive
        self._state = integrate_twist(self._state, twist, dt)
        self.accepted_steps += 1
        return StepReport(self._state, integrated=True, reason="ok")

    def reset_time_base(self) -> None:
        """显式重置时间基准(断流/重连):下一样本只建立新基准,不产生位移。

        与 :meth:`step` 的 gap 分支不同——gap 分支在重置基准的**同一样本**
        里记录速度、之后照常积分,适用于"流未断、只是间隔大"的场合。重连
        场景调用方明确知道间隙存在,且间隙内没有速度样本:若只把基准设为
        重连时刻,首帧会用首帧速度对"重连→首帧到达"这段无样本时间补算
        位移。因此本方法把状态退回"待首帧":下一样本走 :meth:`step` 的
        first 分支(只建基准、记录速度、不积分),再下一帧才按常规间隔
        策略积分。位姿保留断流前的值;诊断计数 explicit_resets 累加。
        """
        self._last_t = None
        self.explicit_resets += 1
