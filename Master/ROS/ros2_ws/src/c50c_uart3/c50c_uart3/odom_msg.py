#!/usr/bin/env python3
"""C50C 编码器里程计候选 → nav_msgs/msg/Odometry 候选消息构造。

**状态:候选。本模块只做"已解析帧 → 积分位姿 + 消息字段"的纯库转换,
不打开串口、不发布任何话题、不发 TF。**发布与否由调用方决定;uart3_node
已接入本模块但**默认关闭**(``odom_enable=False``),开启后也只发布可配置
的影子话题(默认 ``odom_candidate``;最终话题名解析为全局 /odom 时节点
启动即拒绝),不发布正式 /odom、不发 TF。

数据流(全系统串口读取者仍只有 uart3_node 一个,本模块不新增):

    串口字节 ──StatusFrameParser──▶ StatusFrame ──本模块──▶ nav_msgs/Odometry
                                                  └──▶ c50c_odom.PlanarOdomIntegrator

关键约定(与 c50c-odom-prep / c50c-rear-wheel-verify 报告一致):

- 角速度来源:帧 offset 6-7 的 ``chassis_yaw_rate_mrad_s``(**编码器差速
  解算**)。**不使用** offset 18-19 的 ``gyro_raw_lsb[2]``——那是 IMU 陀螺,
  且 FlagStop≠0 时被固件钳零,与编码器解算混用会引入双重来源错误。
- vy:REAR_WHEEL_DEBUG 分支恒 0(``rear_odom.h:47`` 无赋值路径),照实转发。
- 单位:mm/s→m/s、mrad/s→rad/s 只做 ÷1000 精确换算(非标定参数)。
- **速度标度未验证**:vx/wz 数值标度依赖固件运行时电位器选择的轮径/轮距,
  未经真机标定。本模块不做任何标度修正,消息里的数值是固件标度原样。
- 时间:帧内无采样时间戳。积分时间基准 = 调用方提供的**主机接收时刻**
  (单调时钟,``update_from_frame`` 的 ``t_receive_monotonic`` 参数);
  消息 ``header.stamp`` 由调用方用节点时钟另行提供(通常与接收时刻同源)。
  主机接收时刻 ≠ 固件采样时刻(20 Hz 定时 + 串口/调度延迟),偏差不补偿
  ——已声明的限制。积分的时间间隔异常策略由 ``PlanarOdomIntegrator``
  原样保留:dt≤0 拒绝、dt>0.5 s 视为断流间隙不跨积分。
- 协方差(nav_msgs/Odometry 的 pose/twist 各带 36 项 6×6 行主序协方差,
  顺序 x,y,z,roll,pitch,yaw / vx,vy,vz,vroll,vpitch,vyaw;填的是
  **方差**(σ²),单位 m²、rad²、(m/s)²、(rad/s)²,对角线为各轴方差,
  非对角线为轴间协方差):

  - **占位对角项**(pose:x、y、yaw;twist:vx、vyaw):固件不提供任何
    不确定度,默认 0.01 是**未经现场标定的占位数**,不代表实测精度;
    接入真实 /odom 前必须标定或由上游(如 robot_localization)给出。
    经 ``OdomCandidateConfig`` 各字段可配置。
  - **未测量维度**(pose:z、roll、pitch;twist:vy、vz、vroll、vpitch):
    消息数值恒 0 是平面假设 / 固件结构零(vy 在 ``rear_odom.h`` 无赋值
    路径),不是测量值。按 ``unmeasured_variance``(默认 1e6)填大方差,
    明确表达"该维度无有效数据、数值与协方差均不可信,下游不应融合";
    **不填 0**(会被读成"精确已知零"),**不填 -1**(-1"无数据"标记
    只在 sensor_msgs/Imu 姿态上有约定语义,nav_msgs/Odometry 没有)。
  - 非对角线保持 0:等价于假设各轴独立;差速车 vx 与 wz 的相关性
    未测量、未建模。
  - **下游使用限制**:消费方(如 robot_localization)应通过融合维度
    配置只选用 x、y、yaw、vx、vyaw,不能依赖协方差或数值为 0 来判断
    维度有效性;若按消息值融合 vy=0,会把侧向速度钉死在 0。本模块的
    协方差字段全部可配置,任何取值(含默认值)都不是标定结果。
- 方向:现场已确认(2026-11-07 方向复采)——左后轮向前 → vx>0/wz<0,
  右后轮向前 → vx>0/wz>0,右后轮向后 → vx<0/wz<0,与归档固件公式
  ``vx=(left+right)×500``、``wz=(right−left)×1000/(2×半轮距)`` 一致;
  wz 的 REP-103 绝对符号约定(原地左转是否为正)仍未验证。
"""
from __future__ import annotations

from dataclasses import dataclass

from c50c_odom.odometry import (
    PlanarOdomIntegrator,
    StepReport,
    twist_from_parsed,
)

from .frame_parser import StatusFrame

# 占位协方差:未经现场标定,接入真实 /odom 前必须替换为实测或上游提供
DEFAULT_PLACEHOLDER_COVARIANCE = 0.01
# 未测量维度的默认方差:不是任何标定结果,只表达"此维度没有有效数据,
# 数值与协方差都不可信,下游不应融合"。取值足够大,常见 EKF(如
# robot_localization)会自然忽略该维度;详见模块文档协方差一节。
DEFAULT_UNMEASURED_VARIANCE = 1e6


@dataclass(frozen=True)
class OdomCandidateConfig:
    """候选 /odom 消息的配置:frame_id、child_frame_id 与协方差。

    所有协方差字段均为**方差**(σ²)。占位字段(x/y/yaw/vx/vyaw)默认
    0.01 未经标定;未测量维度(z/roll/pitch/vy/vz/vroll/vpitch)用
    ``unmeasured_variance`` 标记"无有效数据"。字段全部可配置,任何取值
    都不是标定结果。
    """

    frame_id: str = "odom"
    child_frame_id: str = "base_link"
    # 协方差均为占位值,未标定——见模块文档
    pose_covariance_x: float = DEFAULT_PLACEHOLDER_COVARIANCE
    pose_covariance_y: float = DEFAULT_PLACEHOLDER_COVARIANCE
    pose_covariance_yaw: float = DEFAULT_PLACEHOLDER_COVARIANCE
    twist_covariance_linear: float = DEFAULT_PLACEHOLDER_COVARIANCE
    twist_covariance_angular: float = DEFAULT_PLACEHOLDER_COVARIANCE
    # 未知/未测量维度的方差:大方差表达"不融合",不用 0(读作精确已知)
    # 也不用 -1(nav_msgs/Odometry 无此标准语义)
    unmeasured_variance: float = DEFAULT_UNMEASURED_VARIANCE

    def __post_init__(self) -> None:
        import math

        fields = {
            "pose_covariance_x": self.pose_covariance_x,
            "pose_covariance_y": self.pose_covariance_y,
            "pose_covariance_yaw": self.pose_covariance_yaw,
            "twist_covariance_linear": self.twist_covariance_linear,
            "twist_covariance_angular": self.twist_covariance_angular,
            "unmeasured_variance": self.unmeasured_variance,
        }
        for name, value in fields.items():
            # 方差必须为有限正数:0 会向下游声称"精确已知",负值/-1 在
            # nav_msgs/Odometry 上没有标准语义,nan/inf 无法被消费者正确处理
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(
                    f"{name} 必须为有限正数(方差),当前 {value!r}"
                    "(0 会声称无不确定度,-1 在 nav_msgs/Odometry 上无标准语义)")


class OdomCandidate:
    """把已解析的 C50C 状态帧接入里程计积分,并可构造候选 Odometry 消息。

    用法(由持有串口读取者的调用方逐帧调用):

        candidate = OdomCandidate()
        report = candidate.update_from_frame(frame, time.monotonic())
        msg = candidate.build_msg(stamp=node_now)   # 可选
    """

    def __init__(self,
                 config: OdomCandidateConfig | None = None,
                 integrator: PlanarOdomIntegrator | None = None) -> None:
        self._config = config or OdomCandidateConfig()
        self._integrator = integrator or PlanarOdomIntegrator()

    @property
    def config(self) -> OdomCandidateConfig:
        return self._config

    @property
    def state(self):
        return self._integrator.state

    def update_from_frame(self, frame: StatusFrame, t_receive_monotonic: float) -> StepReport:
        """把一帧的**编码器**速度送入积分器;t_receive_monotonic 为主机接收时刻。

        使用 ``chassis_velocity_mm_s`` 与 ``chassis_yaw_rate_mrad_s``
        (编码器差速解算);帧内 IMU 字段不参与本计算。
        """
        vx, vy = frame.chassis_velocity_mm_s
        twist = twist_from_parsed(vx, vy, frame.chassis_yaw_rate_mrad_s)
        return self._integrator.step(t_receive_monotonic, twist)

    def build_msg(self, stamp) -> "object":
        """按当前积分状态构造 nav_msgs/msg/Odometry。

        ``stamp``:调用方提供的 ``builtin_interfaces/msg/Time``(节点时钟),
        本模块不从任何时钟取时间。积分器诊断计数(accepted/gap/非正间隔)
        不进消息,由调用方经 :attr:`integrator_diagnostics` 自行暴露。
        """
        from nav_msgs.msg import Odometry  # 延迟导入:无 ROS 环境时仅影响本方法

        state = self._integrator.state
        cfg = self._config
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = cfg.frame_id
        msg.child_frame_id = cfg.child_frame_id

        msg.pose.pose.position.x = state.x
        msg.pose.pose.position.y = state.y
        msg.pose.pose.position.z = 0.0
        # 姿态用平面 yaw 构造四元数(纯平面解算,无 roll/pitch 信息)
        msg.pose.pose.orientation.z = _yaw_to_quat_z(state.yaw)
        msg.pose.pose.orientation.w = _yaw_to_quat_w(state.yaw)

        msg.twist.twist.linear.x = state.twist.vx
        msg.twist.twist.linear.y = state.twist.vy
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = state.twist.wz

        # pose 6×6 对角线:x, y, z, roll, pitch, yaw。
        # z/roll/pitch 未测量(纯平面解算,数值恒 0 只是平面假设),
        # 大方差标记"不可融合",不假装精确为零。
        _set_diag(msg.pose.covariance, 0, cfg.pose_covariance_x)
        _set_diag(msg.pose.covariance, 1, cfg.pose_covariance_y)
        _set_diag(msg.pose.covariance, 2, cfg.unmeasured_variance)
        _set_diag(msg.pose.covariance, 3, cfg.unmeasured_variance)
        _set_diag(msg.pose.covariance, 4, cfg.unmeasured_variance)
        _set_diag(msg.pose.covariance, 5, cfg.pose_covariance_yaw)

        # twist 6×6 对角线:vx, vy, vz, vroll, vpitch, vyaw。
        # vy 恒 0 是固件结构零(rear_odom.h 无赋值路径),vz/vroll/vpitch
        # 无数据——四者均为未测量维度,标记为不可融合。
        _set_diag(msg.twist.covariance, 0, cfg.twist_covariance_linear)
        _set_diag(msg.twist.covariance, 1, cfg.unmeasured_variance)
        _set_diag(msg.twist.covariance, 2, cfg.unmeasured_variance)
        _set_diag(msg.twist.covariance, 3, cfg.unmeasured_variance)
        _set_diag(msg.twist.covariance, 4, cfg.unmeasured_variance)
        _set_diag(msg.twist.covariance, 5, cfg.twist_covariance_angular)
        return msg

    def reset_time_base(self) -> None:
        """显式重置积分时间基准(断流/重连):首样本只建立新基准,不产生位移。

        供持有串口读取者的调用方在**明确知道发生断流**时主动调用
        (如串口重新打开)。与被动 dt>max_interval 判定不同:重连后的
        首帧不会用首帧速度对"重连→首帧到达"这段无速度样本的时间补算
        位移——首帧只建基准,从第二帧开始积分;位姿保留断流前的值。
        """
        self._integrator.reset_time_base()

    def integrator_diagnostics(self) -> dict:
        """积分器诊断计数(供调用方日志/诊断字段,不进 Odometry 消息)。"""
        return {
            "accepted_steps": self._integrator.accepted_steps,
            "rejected_gap": self._integrator.rejected_gap,
            "rejected_nonpositive_dt": self._integrator.rejected_nonpositive_dt,
            "explicit_resets": self._integrator.explicit_resets,
        }


def _yaw_to_quat_z(yaw: float) -> float:
    import math
    return math.sin(yaw / 2.0)


def _yaw_to_quat_w(yaw: float) -> float:
    import math
    return math.cos(yaw / 2.0)


def _set_diag(cov_seq, index6x6: int, value: float) -> None:
    """6×6 行主序协方差对角线赋值;其余(非对角线)保持 0——等价于假设
    各轴独立,vx/wz 相关性未测量、未建模。index6x6 为行/列序号。"""
    cov_seq[index6x6 * 6 + index6x6] = value
