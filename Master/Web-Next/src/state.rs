//! 机器人核心状态快照：`status_payload` 状态模型与全局 `StateStore`（RwLock 快照）。
//!
//! 字段名与旧实现逐字对齐，依据：
//! - 蓝图 §4.3(2)（`{t:"status"}` 下行核心字段 / srv_epoch / 看开分离）；
//! - 旧 `web_debug.py::status_payload`（web_debug.py:1944-2024）；
//! - 旧 `console_views.py`：6S 电池标定（console_views.py:121-163）、
//!   任务状态机（console_views.py:23-118）、开机倒计时（console_views.py:62-88）。
//!
//! 序列化即 JSON 下行载荷：Rust 字段名 == 协议字段名（snake_case 原样保留）。

use std::sync::{OnceLock, RwLock};

use serde::Serialize;

// ---------------------------------------------------------------------------
// 6S 锂电池现场标定（2026-11-20 规范，console_views.py:121-129）
// ---------------------------------------------------------------------------

/// 电池串数：6S 锂离子。
#[allow(dead_code)] // 6S 标定记录，文案由 BatteryView 承载
pub const BAT_CELLS: i32 = 6;
/// 满电电压：24.5V == 100%。
pub const BAT_FULL_V: f64 = 24.5;
/// 充电判定线：> 25.0V 视为充电中（`charging: true`，文案 `充电中`）。
pub const BAT_CHARGING_V: f64 = 25.0;
/// 停用线：21.0V == 0%（3.5V/串），低于停用线继续放电会永久损伤电芯。
pub const BAT_EMPTY_V: f64 = 21.0;
/// 低电警告线：22.2V（低于标称电压即提示充电）。
pub const BAT_WARN_V: f64 = 22.2;

/// 开机静止倒计时时长（对应 c50c-chassis.service 的 ExecStartPre sleep 20）。
pub const BOOT_HOLD_S: f64 = 20.0;

/// 6S 电池标定视图（console_views.py::battery_view 的逐字移植）。
#[derive(Debug, Clone, Serialize)]
pub struct BatteryView {
    /// 上报电压（V，保留 2 位）。
    pub voltage_v: Option<f64>,
    /// 电量百分比：电压在 [21.0V, 24.5V] 的线性映射，钳位 0..=100。
    pub percent: Option<i64>,
    /// > 25.0V 为充电中。
    pub charging: bool,
    /// `unknown` | `ok` | `warn` | `bad`。
    pub state: &'static str,
    /// 展示文案：`无数据` | `充电中` | `低于停用线，立即停车充电` | `电量低，请充电` | `放电`
    /// （非充电即放电，用户定版：状态行直接表达「充/放」，不再用「正常」这种自证词）。
    pub text: &'static str,
}

impl BatteryView {
    pub fn no_data() -> Self {
        Self { voltage_v: None, percent: None, charging: false, state: "unknown", text: "无数据" }
    }

    /// 按现场标定（24.5V 满电 / >25.0V 充电中）推算电量与状态。
    /// 百分比是仪表估计，不是库仑计量（锂电开路曲线非线性、带载有瞬时压降）。
    pub fn from_voltage(voltage_v: Option<f64>) -> Self {
        let Some(voltage) = voltage_v else { return Self::no_data() };
        let raw = (voltage - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V) * 100.0;
        let (state, text, charging) = if voltage > BAT_CHARGING_V {
            ("ok", "充电中", true)
        } else if voltage <= BAT_EMPTY_V {
            ("bad", "低于停用线，立即停车充电", false)
        } else if voltage < BAT_WARN_V {
            ("warn", "电量低，请充电", false)
        } else {
            ("ok", "放电", false)
        };
        Self {
            voltage_v: Some((voltage * 100.0).round() / 100.0),
            percent: Some(raw.round().clamp(0.0, 100.0) as i64),
            charging,
            state,
            text,
        }
    }
}

// ---------------------------------------------------------------------------
// 任务状态机（console_views.py:23-118）
// ---------------------------------------------------------------------------

/// 任务状态机五态（STATE_* 常量逐字对齐）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum TaskState {
    Idle,
    Nav,
    Patrol,
    Return,
    Manual,
}

impl TaskState {
    /// 协议取值：`idle` / `nav` / `patrol` / `return` / `manual`。
    pub fn as_str(&self) -> &'static str {
        match self {
            TaskState::Idle => "idle",
            TaskState::Nav => "nav",
            TaskState::Patrol => "patrol",
            TaskState::Return => "return",
            TaskState::Manual => "manual",
        }
    }

    /// 展示文案（_TEXT 映射逐字对齐）。
    pub fn text(&self) -> &'static str {
        match self {
            TaskState::Idle => "无任务",
            TaskState::Nav => "导航前往任务目标中",
            TaskState::Patrol => "巡逻中",
            TaskState::Return => "低电量回充中",
            TaskState::Manual => "手动控制中",
        }
    }

    /// 任务状态机判定（console_views.py::task_state 的移植）：
    /// 低电量回充 > 巡逻 > 导航/手动（命令须新鲜且非零速）> 无任务。
    pub fn decide(
        nav_fresh: bool,
        nav_vx: f64,
        nav_wz: f64,
        nav_age: Option<f64>,
        manual_active: bool,
        battery_low: bool,
        patrol_fresh: bool,
    ) -> Self {
        if battery_low {
            return TaskState::Return;
        }
        if patrol_fresh {
            return TaskState::Patrol;
        }
        let moving = nav_vx.abs() > 1e-6 || nav_wz.abs() > 1e-6;
        if nav_fresh && nav_age.is_some_and(|age| age <= 1.0) {
            if moving && manual_active {
                return TaskState::Manual;
            }
            if moving {
                return TaskState::Nav;
            }
        }
        TaskState::Idle
    }
}

/// 状态帧里的任务卡片（`view['task'] = _task_view(...)`，web_debug.py:2076-2104）。
#[derive(Debug, Clone, Serialize)]
pub struct TaskView {
    /// `idle` / `nav` / `patrol` / `return` / `manual`。
    pub state: &'static str,
    /// 展示文案。
    pub text: &'static str,
    /// 当前导航命令 vx（m/s）。
    pub cmd_vx: Option<f64>,
    /// 当前导航命令 wz（rad/s）。
    pub cmd_wz: Option<f64>,
    /// 命令通道是否就绪：底盘状态帧新鲜，或安全心跳在线。
    pub channel: bool,
    /// 手动控制诊断（manual_forward.snapshot* 字段）。
    pub manual: serde_json::Value,
}

// ---------------------------------------------------------------------------
// status_payload 子视图
// ---------------------------------------------------------------------------

/// 底盘卡片（chassis.py::chassis_view + `hz`）。超时后 state=stale，绝不显示旧值当实时。
#[derive(Debug, Clone, Serialize)]
pub struct ChassisView {
    /// `fresh` | `stale` | `unknown`。
    pub state: &'static str,
    pub age: Option<f64>,
    pub voltage_v: Option<f64>,
    pub vx_m_s: Option<f64>,
    pub vy_m_s: Option<f64>,
    pub wz_rad_s: Option<f64>,
    /// 固件 stop_flag：0=软件未失能；非 0=软件失能；None=未知。
    pub stop_flag: Option<i32>,
    /// `未知` | `软件未失能；就绪未知` | `软件失能`。
    pub stop_text: &'static str,
    pub drive_ready: Option<bool>,
    pub mileage_m: Option<f64>,
    /// 前置 HC-SR04 距离（m）；0xFFFF=无回波不显示旧值。
    pub ultrasonic_m: Option<f64>,
    /// 轮速分解（wheel_view），暂透传 TODO(下一阶段类型化)。
    pub wheels: Option<serde_json::Value>,
    /// 底盘状态实测频率。
    pub hz: Option<f64>,
}

/// IMU 心跳卡片（chassis.py::imu_view）。
#[derive(Debug, Clone, Serialize)]
pub struct ImuView {
    pub state: &'static str,
    pub age: Option<f64>,
    pub hz: Option<f64>,
}

/// 速度命令卡片（chassis.py::cmd_view）。
#[derive(Debug, Clone, Serialize)]
pub struct CmdView {
    pub state: &'static str,
    pub age: Option<f64>,
    pub vx_m_s: Option<f64>,
    pub wz_rad_s: Option<f64>,
}

/// 四路命令视图（`view['cmd']`：nav / demo / manual / auto）。
#[derive(Debug, Clone, Serialize)]
pub struct CmdViews {
    pub nav: CmdView,
    pub demo: CmdView,
    pub manual: CmdView,
    pub auto: CmdView,
}

/// `/drive_safety` 展示视图（web_debug.py::drive_safety_view）。
/// 过期（超 ttl 或无 ttl 按 1s）时 state='stale'、allowed=None：
/// 过期许可不等于允许，也不等于拒绝，页面显示"无许可"。
#[derive(Debug, Clone, Serialize)]
pub struct DriveSafetyView {
    pub state: &'static str,
    pub allowed: Option<bool>,
    pub reason: Option<String>,
    pub age: Option<f64>,
    pub seq: Option<i64>,
    /// fusion_node 唯一判定点给出的运动方向结论（与自动命令同源）。
    pub motion: Option<serde_json::Value>,
    pub ultrasonic: Option<serde_json::Value>,
}

/// 节点在线视图（web_debug.py::node_view）。`online` | `offline`。
#[derive(Debug, Clone, Serialize)]
pub struct NodesView {
    pub camera_node: &'static str,
    pub lidar_node: &'static str,
    pub c50c_uart3_node: &'static str,
    pub chassis_status_node: &'static str,
    /// web 节点自身恒在线。
    pub web_bridge: &'static str,
    pub scan_without_node: bool,
}

/// 开机初始化视图（console_views.py::boot_view）。
/// 标记文件存在且未超时 → initializing=true，前端显示 20s 静止倒计时。
#[derive(Debug, Clone, Serialize)]
pub struct BootView {
    pub initializing: bool,
    pub remain_s: f64,
    pub hold_s: f64,
}

/// 巡航开关视图（web_debug.py::cruise_view）。
/// `on` 是脚本侧已下发的值；`available` 表示发布器就绪。
#[derive(Debug, Clone, Serialize)]
pub struct CruiseView {
    pub on: bool,
    pub available: bool,
}

/// `/ws` 服务端下行 `{t:"status"}` 完整载荷（web_debug.py::status_payload 的顶层结构）。
#[derive(Debug, Clone, Serialize)]
pub struct StatusPayload {
    /// 恒为 `"status"`。
    pub t: &'static str,
    /// 运行模式：`ros`（实车）| `fake`（`--fake` 仿真源）。
    pub mode: &'static str,
    pub chassis: ChassisView,
    pub imu: ImuView,
    pub cmd: CmdViews,
    /// 桥安全心跳 TODO(下一阶段类型化)：现值透传（含 `online` 键）。
    pub safety: serde_json::Value,
    pub drive_safety: DriveSafetyView,
    /// 导航状态视图 TODO(下一阶段类型化)。
    pub nav: serde_json::Value,
    /// 雷达实测频率（Hz）。
    pub scan_hz: f64,
    /// 相机实测帧率（fps）。
    pub image_fps: f64,
    /// 雷达累计帧数（旧 `_state['scan_frames']` 计数器语义）。
    pub scan_frames: serde_json::Value,
    /// 当前帧有效点数（旧 `_state['scan_valid']`：r>0 的点数）。
    pub scan_valid: serde_json::Value,
    /// 网络信息（Fake 模式无 iw 数据源，保持 null，不冒充有数据）。
    pub network: serde_json::Value,
    /// 音频设备探测（喇叭/板载麦克风挂 es8388、USB 摄像头麦克风；Fake
    /// 模式保持 null，不冒充有数据）。
    pub audio: serde_json::Value,
    /// 系统占用（CPU/GPU/内存/磁盘百分比；Fake 模式保持 null，不冒充）。
    pub sys: serde_json::Value,
    /// 主机信息（同上）。
    pub host: serde_json::Value,
    /// 雷达诊断明细（web_debug.py::_status_msg 附加的 lidar_detail）。
    pub lidar_detail: serde_json::Value,
    /// 开机 20s 静止倒计时。
    pub boot: BootView,
    /// 进程纪元：服务端启动的 Unix 时间戳（秒）。
    /// 前端检测到变化时整页刷新以重建全部连接。
    pub srv_epoch: i64,
    pub nodes: NodesView,
    pub task: TaskView,
    pub battery: BatteryView,
    pub cruise: CruiseView,
    /// 航点巡航 + 运动许可帧（patrol_status_frame）。
    pub patrol: serde_json::Value,
    /// 移动标记（仅保留 2 秒新鲜度）。
    pub moving: Vec<serde_json::Value>,
}

impl StatusPayload {
    /// 构造服务端启动时的初始快照（等价旧代码各 view 的空态分支）。
    pub fn initial(mode: &'static str, srv_epoch: i64) -> Self {
        Self {
            t: "status",
            mode,
            chassis: ChassisView {
                state: "unknown",
                age: None,
                voltage_v: None,
                vx_m_s: None,
                vy_m_s: None,
                wz_rad_s: None,
                stop_flag: None,
                stop_text: "未知",
                drive_ready: None,
                mileage_m: None,
                ultrasonic_m: None,
                wheels: None,
                hz: None,
            },
            imu: ImuView { state: "unknown", age: None, hz: None },
            cmd: CmdViews {
                nav: CmdView { state: "unknown", age: None, vx_m_s: None, wz_rad_s: None },
                demo: CmdView { state: "unknown", age: None, vx_m_s: None, wz_rad_s: None },
                manual: CmdView { state: "unknown", age: None, vx_m_s: None, wz_rad_s: None },
                auto: CmdView { state: "unknown", age: None, vx_m_s: None, wz_rad_s: None },
            },
            safety: serde_json::Value::Null,
            drive_safety: DriveSafetyView {
                state: "unknown",
                allowed: None,
                reason: None,
                age: None,
                seq: None,
                motion: None,
                ultrasonic: None,
            },
            nav: serde_json::Value::Null,
            scan_hz: 0.0,
            image_fps: 0.0,
            scan_frames: serde_json::json!(0),
            scan_valid: serde_json::json!(0),
            network: serde_json::Value::Null,
            audio: serde_json::Value::Null,
            sys: serde_json::Value::Null,
            host: serde_json::Value::Null,
            lidar_detail: serde_json::Value::Null,
            boot: BootView { initializing: false, remain_s: 0.0, hold_s: BOOT_HOLD_S },
            srv_epoch,
            nodes: NodesView {
                camera_node: "offline",
                lidar_node: "offline",
                c50c_uart3_node: "offline",
                chassis_status_node: "offline",
                web_bridge: "online",
                scan_without_node: false,
            },
            task: TaskView {
                state: TaskState::Idle.as_str(),
                text: TaskState::Idle.text(),
                cmd_vx: None,
                cmd_wz: None,
                channel: false,
                manual: serde_json::Value::Null,
            },
            battery: BatteryView::no_data(),
            cruise: CruiseView { on: false, available: false },
            patrol: serde_json::Value::Null,
            moving: Vec::new(),
        }
    }
}

// ---------------------------------------------------------------------------
// 全局 StateStore（RwLock 快照中心）
// ---------------------------------------------------------------------------

/// 全局状态快照中心：单写多读，读者拿整份克隆（20Hz 推送 / `/api/state`）。
pub struct StateStore {
    inner: RwLock<StatusPayload>,
}

impl StateStore {
    /// 以当前 Unix 时间戳为 `srv_epoch` 创建快照中心。
    pub fn new(mode: &'static str) -> Self {
        let srv_epoch = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs() as i64)
            .unwrap_or(0);
        Self { inner: RwLock::new(StatusPayload::initial(mode, srv_epoch)) }
    }

    /// 读取整份快照（与 `/ws` 推送的 `{t:"status"}` 同构，供 `/api/state`）。
    pub fn snapshot(&self) -> StatusPayload {
        self.inner.read().expect("state poisoned").clone()
    }

    /// 就地更新快照。
    pub fn update(&self, f: impl FnOnce(&mut StatusPayload)) {
        f(&mut self.inner.write().expect("state poisoned"));
    }
}

static STATE: OnceLock<StateStore> = OnceLock::new();

/// 进程启动时初始化全局快照中心（幂等，返回既有实例）。
pub fn init_global(mode: &'static str) -> &'static StateStore {
    STATE.get_or_init(|| StateStore::new(mode))
}

/// 取全局快照中心；未初始化视为程序装配错误。
pub fn global() -> &'static StateStore {
    STATE.get().expect("StateStore not initialized: call init_global() in main")
}
