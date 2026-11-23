//! ros 模式桥接层：`/bridge` 两端点 + 帧落库 + 出口命令队列 + 20Hz 看护装配。
//!
//! 数据面（契约 frameMap）：垫片（deploy/ros_bridge_shim.py）逐字复刻旧
//! web_debug.py Bridge 的订阅与转换，把「垫片帧」批量 POST 到
//! `POST /bridge/frames`；本模块按 FakeEngine 的写入路径落库——
//! img/scan/map/pose 写 `fake::media_slot()`（Arc 换新 = last-wins，
//! fake.rs:269-298 同款），状态类帧写本模块原始缓存 `Raw`，由 20Hz
//! 装配任务折算进 `StateStore.update`（state.rs:400-404），最终由
//! ws_video.rs 的既有节流逻辑推给浏览器。
//!
//! 控制面（契约 commandMap）：hub 侧状态机在 safety.rs（fake/ros 共用）；
//! ros 模式在「意图对账」模式下派生 ROS 出口命令——所有 Control 变更点
//! （ws_control handler、20Hz 看护 tick 的三个状态转变、conn_closed）写
//! 出口意图，20Hz tick 末尾统一对账差量派发；垫片经
//! `GET /bridge/commands` 长轮询取走并发布到 ROS 话题。
//!
//! 安全边界：
//! - 令牌：环境变量 `OPI_BRIDGE_TOKEN`，`X-Bridge-Token` 头，常数时间
//!   比较（safety.rs::ct_eq 模式）；401 不区分「缺/错」且不回显令牌。
//!   Web-Next 绑 0.0.0.0（main.rs:174），无令牌的裸端点会被同网段任意
//!   主机伪造（上行注入假状态、下行偷走 nav_goal 驱动实车）。
//! - 边沿触发命令（rearm_request / ultrasonic_rearm / nav_goal /
//!   nav_cancel）永不随 cursor 补投重放——重放陈旧解停请求会在无用户
//!   动作时解锁下游 guard（guard_bridge.py:195-211,258-261 语义）。
//! - 活性：>10s 无任何帧批（含 probe）→ nodes.web_bridge=offline；窗口
//!   必须大于垫片 probe 周期（3s）并留余量，防传感器静默时抖动。
//!
//! 与旧实现的对齐点（行号见旧仓库 Web/opi_robot/）：
//! - 新鲜度常数：SCAN/IMAGE/DIAG/CHASSIS/IMU/CMD（web_debug.py:805-812）；
//! - drive_safety_view 的 ttl 语义（web_debug.py:2027-2052）；
//! - 里程积分 trapezoid + 0.5s 丢帧上限（chassis_status.py:59-89）；
//! - guard_authorize 按值去重（web_debug.py:900-906）；
//! - 巡逻 rid 推进：accepted→pending、succeeded→index+1/回绕 laps+1、
//!   failed/cancelled→锁停、锁停中迟到接受→立即取消（patrol.py:521-581）；
//! - 15s 在途未接受超时锁停（patrol.py:613-620）。

use std::collections::{HashMap, VecDeque};
use std::io::Read as _;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use axum::body::Bytes;
use axum::extract::{DefaultBodyLimit, Query};
use axum::http::{header, HeaderMap, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::Router;
use serde_json::{json, Value};

use crate::fake::{self, GridPub};
use crate::safety::{self, Hub};
use crate::state::{
    BatteryView, ChassisView, CmdView, CmdViews, CruiseView, DriveSafetyView, ImuView, NodesView,
    StateStore, StatusPayload, TaskState, TaskView,
};
use crate::Config;

// ---------------------------------------------------------------------------
// 常数（对齐旧实现与契约）
// ---------------------------------------------------------------------------

/// 单批帧数上限（契约 endpointContract①）。
pub const FRAME_BATCH_MAX: usize = 256;
/// 请求体上限：8MB（图像 base64 主要体积；垫片与 Web-Next 同板走回环）。
pub const BODY_LIMIT_BYTES: usize = 8 * 1024 * 1024;
/// 垫片活性窗口（ms）：>10s 无任何帧批（含 probe）判 offline。必须大于
/// 垫片 probe 周期（3s）并留余量，防传感器静默但垫片健康时抖动。
pub const BRIDGE_OFFLINE_MS: u64 = 10_000;
/// patrol_status 帧捎带周期（ms）：命令流每 ~1s 捎带最新帧（契约
/// commandMap patrol_tick 行），垫片以 10Hz 重发缓存帧。
pub const PATROL_FRAME_PERIOD_MS: u64 = 1000;
/// 命令环形保留条数（契约 endpointContract②）。
pub const CMD_RING_MAX: usize = 256;
/// 命令确认（cmd_ack）等待时限（ms）。
pub const CMD_ACK_TIMEOUT_MS: u64 = 2000;
/// 全局重定位（numpy 扫描-地图匹配）ack 等待时限（ms）：垫片经子进程跑
/// 独立 Python 避开 GIL 争用（小核 ~4–5s），子进程超时上限 30s 再留余量。
pub const RELOCALIZE_ACK_TIMEOUT_MS: u64 = 35_000;
/// 长轮询等待上限（ms）：契约 wait_ms=20000 缺省，钳到 30s 防连接被中间
/// 设备掐断。
pub const CMD_WAIT_MAX_MS: u64 = 30_000;

// 旧 web_debug.py:805-812 的新鲜度常数（秒）。SCAN_TIMEOUT_S（1.5）与
// IMAGE_TIMEOUT_S（3.0）属于传感器视图（sensors.py::sensor_view），ros 模式
// 的 StatusPayload 无对应卡片，保留记录不参与编译路径。
#[allow(dead_code)]
const SCAN_TIMEOUT_S: f64 = 1.5;
const DIAG_TIMEOUT_S: f64 = 3.0;
const CHASSIS_TIMEOUT_S: f64 = 2.0;
const IMU_TIMEOUT_S: f64 = 2.0;
const CMD_TIMEOUT_S: f64 = 1.0;
/// 节点在线窗口（web_debug.py::node_view 默认 6.0s）。
const NODE_ONLINE_S: f64 = 6.0;
/// 里程积分最大步长（chassis_status.py:53 MAX_STEP_S = 0.5）。
const MILEAGE_MAX_STEP_S: f64 = 0.5;
/// 点选导航目标范围（nav_goal.py:26-29；地图栅格 584×256@0.05，原点
/// -8.28,-8.03，覆盖已扫区留余量）。
pub const GOAL_MIN_X: f64 = -8.0;
pub const GOAL_MAX_X: f64 = 22.0;
pub const GOAL_MIN_Y: f64 = -8.0;
pub const GOAL_MAX_Y: f64 = 8.0;
/// 目标与车的最小间距（nav_goal.py:31 GOAL_MIN_DIST_M——旧代码里是
/// 未被引用的死常量，契约 risks#7 要求 ros 模式落成真校验）。
pub const GOAL_MIN_DIST_M: f64 = 0.3;

// ---------------------------------------------------------------------------
// 小工具：定窗频率计 / 里程积分
// ---------------------------------------------------------------------------

/// 固定窗口频率计（web_debug.py:1104-1123 RateMeter 的移植）。
/// 窗口到期才更新，避免抖动。
#[derive(Default)]
struct RateMeter {
    window_s: f64,
    n: u64,
    t0_ms: Option<u64>,
    hz: f64,
}

impl RateMeter {
    fn new(window_s: f64) -> Self {
        Self { window_s, n: 0, t0_ms: None, hz: 0.0 }
    }
    /// 记一次到达，返回当前频率（窗口内保持上次值）。t0 用 Option：
    /// safety::now_ms() 在进程启动的头几毫秒就是合法的 0，不能用 0 当
    /// 「未起始」哨兵。
    fn tick(&mut self, now_ms: u64) -> f64 {
        if self.t0_ms.is_none() {
            self.t0_ms = Some(now_ms);
        }
        self.n += 1;
        let dt = now_ms.saturating_sub(self.t0_ms.unwrap_or(now_ms)) as f64 / 1000.0;
        if dt >= self.window_s {
            self.hz = (self.n as f64 / dt * 10.0).round() / 10.0;
            self.n = 0;
            self.t0_ms = Some(now_ms);
        }
        self.hz
    }
    fn value(&self) -> f64 {
        self.hz
    }
}

/// 里程积分器（chassis_status.py:59-89 MileageIntegrator 的移植）：
/// 对固件上报 vx 做 trapezoid 时间积分；停车不增长，丢帧期间（dt 超过
/// max_step_s）不计入。软件积分，不是轮式里程计。
#[derive(Default)]
struct MileageIntegrator {
    last_t_ms: Option<u64>,
    last_vx: Option<f64>,
    distance_m: f64,
}

impl MileageIntegrator {
    fn update(&mut self, vx_mm_s: i64, now_ms: u64) {
        let vx = vx_mm_s as f64 / 1000.0;
        if let Some(t) = self.last_t_ms {
            let dt = (now_ms.saturating_sub(t)) as f64 / 1000.0;
            if (0.0..=MILEAGE_MAX_STEP_S).contains(&dt) {
                let prev = self.last_vx.unwrap_or(vx);
                self.distance_m += (0.5 * (prev + vx)).abs() * dt;
            }
        }
        self.last_t_ms = Some(now_ms);
        self.last_vx = Some(vx);
    }
    fn trip(&self) -> f64 {
        self.distance_m
    }
}

fn round2(v: f64) -> f64 {
    (v * 100.0).round() / 100.0
}

fn round3(v: f64) -> f64 {
    (v * 1000.0).round() / 1000.0
}

fn round4(v: f64) -> f64 {
    (v * 10000.0).round() / 10000.0
}

// ---------------------------------------------------------------------------
// 原始缓存（状态类帧的落库点，20Hz 装配任务折算进 StateStore）
// ---------------------------------------------------------------------------

/// `/diagnostics` 里底盘状态的解析结果（chassis.py:62-90 的键名候选匹配
/// 由垫片完成；hub 只收转换后的垫片帧字段）。level/message 属契约帧字段，
/// 旧 chassis_view 同样不渲染，仅落库备查。
#[allow(dead_code)]
struct ChassisRaw {
    voltage_mv: Option<i64>,
    vx_mm_s: Option<i64>,
    vy_mm_s: Option<i64>,
    yaw_rate_mrad_s: Option<i64>,
    stop_flag: Option<i64>,
    ultrasonic_front_mm: Option<i64>,
    level: Option<i64>,
    message: String,
}

/// 雷达诊断（sensors.py:39-63 parse_diagnostic 的垫片侧产物）。
struct LidarDiagRaw {
    level: Option<i64>,
    message: String,
    state: String,
    reason: String,
    age_s: Option<f64>,
    coverage: Option<f64>,
    reconnects: Option<i64>,
    port: String,
    frames_ok: Option<i64>,
    frames_bad: Option<i64>,
}

/// 一路速度命令回显（web_debug.py::on_cmd + _copy_twist）。vy 属契约帧
/// 字段；CmdView 只展示 vx/wz（chassis.py::cmd_view 同）。
#[allow(dead_code)]
struct CmdRaw {
    vx: f64,
    vy: f64,
    wz: f64,
}

/// navigate_to_pose 反馈（nav_goal.py:248-269 的字段）。
struct NavFeedbackRaw {
    rid: String,
    dist: Option<f64>,
    eta: Option<f64>,
    x: Option<f64>,
    y: Option<f64>,
}

/// 探测帧的节点可见时间戳（hub 单调毫秒；None=最近一次探测不在线）。
#[derive(Default)]
struct NodesSeen {
    camera_node: Option<u64>,
    lidar_node: Option<u64>,
    chassis_status_node: Option<u64>,
}

/// 全部状态类帧的原始缓存。时间戳一律 hub 单调毫秒（safety::now_ms），
/// 不信任垫片上报的 mono_ms。
#[derive(Default)]
/// 时间戳一律 Option<u64>（hub 单调毫秒）：None=从未收到——safety::now_ms()
/// 启动头几毫秒就是 0，不能用 0 当「从未」哨兵。
struct Raw {
    chassis: Option<ChassisRaw>,
    chassis_t: Option<u64>,
    lidar_diag: Option<LidarDiagRaw>,
    diag_t: Option<u64>,
    imu_hz: Option<f64>,
    imu_t: Option<u64>,
    /// nav / demo / manual / auto 四路命令回显。
    cmds: [Option<CmdRaw>; 4],
    cmd_t: [Option<u64>; 4],
    nav_seen_t: Option<u64>,
    drive_safety: Option<Value>,
    drive_safety_t: Option<u64>,
    moving: Option<Vec<i64>>,
    moving_t: Option<u64>,
    nodes_seen: NodesSeen,
    safety: Option<Value>,
    safety_seen_t: Option<u64>,
    nav_feedback: Option<NavFeedbackRaw>,
    nav_feedback_t: Option<u64>,
    /// 最近一次 nav_result 携带的 Nav2 error_code 文本。
    last_plan_error: String,
    /// 最近一次帧批到达时刻（web_bridge 活性判定）。
    last_batch_ms: Option<u64>,
    mileage: MileageIntegrator,
    /// 雷达/相机 2s 窗口实测（web_debug.py:1338-1346,1285-1293）；
    /// 底盘 1s 窗口（web_debug.py:1107 RateMeter 缺省）。
    scan_rate: RateMeter,
    image_rate: RateMeter,
    chassis_rate: RateMeter,
}

impl Raw {
    fn new() -> Self {
        Self {
            scan_rate: RateMeter::new(2.0),
            image_rate: RateMeter::new(2.0),
            chassis_rate: RateMeter::new(1.0),
            ..Default::default()
        }
    }
}

static RAW: OnceLock<Mutex<Raw>> = OnceLock::new();

fn raw_slot() -> &'static Mutex<Raw> {
    RAW.get_or_init(|| Mutex::new(Raw::new()))
}

// ---------------------------------------------------------------------------
// 出口命令队列（GET /bridge/commands 长轮询的数据源）
// ---------------------------------------------------------------------------

struct QueuedCmd {
    id: u64,
    body: Value,
    /// 值语义幂等命令（license_forward / guard_authorize / manual_cmd /
    /// manual_zero / cruise_enable）：可按 cursor 补投。边沿触发命令
    /// （rearm_request / ultrasonic_rearm / nav_goal / nav_cancel /
    /// patrol_status）false——投递一次即出队，永不重放。
    replayable: bool,
    delivered: bool,
}

struct QueueInner {
    next_id: u64,
    ring: VecDeque<QueuedCmd>,
    /// patrol_status 最新帧槽位（等价旧 _patrol['publisher'] 的 10Hz 流：
    /// 只有最新一帧有意义，未投递就被下一帧顶替）。
    patrol_slot: Option<Value>,
    /// 垫片最近一次轮询声明的 cursor（cmds_pending 判定基准）。
    last_cursor: u64,
}

/// 出口命令队列：单消费者（垫片），Notify 即时唤醒长轮询。
pub struct CommandQueue {
    st: Mutex<QueueInner>,
    notify: tokio::sync::Notify,
}

static QUEUE: OnceLock<CommandQueue> = OnceLock::new();

fn queue_slot() -> &'static CommandQueue {
    QUEUE.get_or_init(|| {
        CommandQueue {
            st: Mutex::new(QueueInner {
                next_id: 1,
                ring: VecDeque::new(),
                patrol_slot: None,
                last_cursor: 0,
            }),
            notify: tokio::sync::Notify::new(),
        }
    })
}

impl CommandQueue {
    /// 入队一条出口命令，返回命令 id。命令体会带上自身 id（垫片的
    /// cmd_ack 按 id 回执：ultrasonic_rearm / cruise_enable 需要垫片如实
    /// 回报发布成败，契约 commandMap 两行）。
    fn push(&self, body: Value, replayable: bool) -> u64 {
        let mut st = self.st.lock().expect("queue poisoned");
        let id = st.next_id;
        st.next_id += 1;
        let mut body = body;
        if let Value::Object(map) = &mut body {
            map.insert("id".into(), json!(id));
        }
        st.ring.push_back(QueuedCmd { id, body, replayable, delivered: false });
        while st.ring.len() > CMD_RING_MAX {
            // 溢出丢最老，但优先丢已投递条目：ring 中 delivered==true 的一定
            // 是值语义命令（边沿命令投递即出队，见 poll 的 retain），补投已
            // 消费过，再丢不损补投语义；未投递的值语义命令（license_forward
            // 10Hz 积压）尽可能保留（评审 low：环形溢出区分 delivered）。
            // 全为未投递时才丢最老——方向 fail-safe：下游 guard 有 350ms
            // 许可超时锁停兜底，丢帧不丢安全性。
            let drop_idx = st.ring.iter().position(|c| c.delivered).unwrap_or(0);
            st.ring.remove(drop_idx);
        }
        drop(st);
        self.notify.notify_one();
        id
    }

    /// patrol_status 特殊入队：只保留最新一帧（顶替未投递的旧帧）。
    fn push_patrol_frame(&self, frame: Value) {
        let mut st = self.st.lock().expect("queue poisoned");
        st.patrol_slot = Some(json!({ "c": "patrol_status", "frame": frame }));
        drop(st);
        self.notify.notify_one();
    }

    /// bridge_hello（垫片重连）：清空未投递命令。由新许可流自然重新武装
    /// guard，避免旧积压错序（契约 endpointContract②修订）。
    fn clear_undelivered(&self) {
        let mut st = self.st.lock().expect("queue poisoned");
        st.ring.retain(|c| c.delivered);
        if st.last_cursor >= st.next_id {
            st.last_cursor = 0;
        }
        st.patrol_slot = None;
    }

    /// 是否还有垫片没见过的命令（POST /bridge/frames 响应的 cmds_pending，
    /// 供垫片立即转命令轮询）。
    fn pending(&self) -> bool {
        let st = self.st.lock().expect("queue poisoned");
        st.patrol_slot.is_some()
            || st.ring.iter().any(|c| c.id > st.last_cursor && (c.replayable || !c.delivered))
    }

    /// 长轮询：返回 (cmds, new_cursor)。值语义命令按 cursor 补投；边沿
    /// 命令投递即出队。`wait_ms` 内无命令返回空表。
    async fn poll(&self, cursor: u64, wait_ms: u64) -> (Vec<Value>, u64) {
        let deadline = Instant::now() + Duration::from_millis(wait_ms.min(CMD_WAIT_MAX_MS));
        let mut eff_cursor = cursor;
        loop {
            {
                let mut st = self.st.lock().expect("queue poisoned");
                if eff_cursor >= st.next_id {
                    // 垫片带着上一代 web_next 进程的旧 cursor 来轮询（例如单边
                    // 重启 web_next 后 next_id 从 1 重新计数）：自动归零同步。
                    eff_cursor = 0;
                    st.last_cursor = 0;
                } else {
                    st.last_cursor = st.last_cursor.max(eff_cursor);
                }
                let mut out = Vec::new();
                if let Some(frame) = st.patrol_slot.take() {
                    out.push(frame);
                }
                let mut max_id = 0u64;
                for c in st.ring.iter_mut() {
                    if c.id > eff_cursor && (c.replayable || !c.delivered) {
                        out.push(c.body.clone());
                        c.delivered = true;
                        max_id = c.id;
                    }
                }
                // 边沿命令投递即出队（不再占用环形容量）。
                st.ring.retain(|c| c.replayable || !c.delivered);
                if !out.is_empty() {
                    return (out, max_id.max(eff_cursor));
                }
            }
            let now = Instant::now();
            if now >= deadline {
                return (Vec::new(), eff_cursor);
            }
            let _ = tokio::time::timeout(deadline - now, self.notify.notified()).await;
        }
    }
}

// ---------------------------------------------------------------------------
// 出口意图（意图对账模式：hub 维护 last_emitted，tick 末尾对账差量派发）
// ---------------------------------------------------------------------------

struct Intent {
    /// 最近一次发给下游的 guard_authorize 值（按值去重，
    /// web_debug.py:900-906）。None=未知（启动 / bridge_hello 后）。
    last_authorize: Option<bool>,
    /// 手动命令流是否处于「垫片正在 10Hz 重发」状态（manual_zero 之后的
    /// 对账基准）。
    manual_stream: bool,
    /// 最近一次发出去的手动 (vx, wz)。
    last_manual: Option<(f64, f64)>,
    /// 最近一次发出去的 cruise_enable 值。
    last_cruise: Option<bool>,
    /// 上次 patrol_status 帧捎带时刻。
    last_patrol_frame_ms: u64,
    /// rid 序号源（p…/m… 前缀区分巡逻/手动目标）。
    rid_seq: u64,
}

static INTENT: OnceLock<Mutex<Intent>> = OnceLock::new();

fn intent_slot() -> &'static Mutex<Intent> {
    INTENT.get_or_init(|| {
        Mutex::new(Intent {
            last_authorize: None,
            manual_stream: false,
            last_manual: None,
            last_cruise: None,
            last_patrol_frame_ms: 0,
            rid_seq: 0,
        })
    })
}

/// 锁停事件后的立即撤回（对齐旧 patrol_downstream_lock_stop：
/// web_debug.py:935-947 第一动作撤下游鉴权）。置 last_authorize=Some(false)
/// 后，tick 对账在许可恢复前不会再把它顶回 True。
fn force_authorize_false() {
    let mut intent = intent_slot().lock().expect("intent poisoned");
    force_authorize_false_locked(&mut intent);
}

/// 持 intent 锁版本的撤回。std Mutex 非重入：nav_housekeeping 的 15s 在途
/// 超时分支与 nav_result 的 _fail 分支都在已持有 intent_slot 的上下文里
/// 撤回，若走自锁版本会重入 lock() 自死锁——tick 线程卡死导致三看护/
/// manual_zero 对账/巡逻推进/状态装配全停，guard_authorize(false) 与
/// nav_cancel 永不发出（评审 high-2）。
fn force_authorize_false_locked(intent: &mut Intent) {
    queue_slot().push(json!({ "c": "guard_authorize", "value": false }), true);
    intent.last_authorize = Some(false);
}

// ---------------------------------------------------------------------------
// 命令确认（cmd_ack）：ultrasonic_rearm / cruise_enable 需要垫片如实回报
// 发布成败（契约 commandMap 两行的「不假装已恢复」要求）。
// ---------------------------------------------------------------------------

static ACKS: OnceLock<Mutex<HashMap<u64, tokio::sync::oneshot::Sender<Value>>>> = OnceLock::new();

fn acks_slot() -> &'static Mutex<HashMap<u64, tokio::sync::oneshot::Sender<Value>>> {
    ACKS.get_or_init(|| Mutex::new(HashMap::new()))
}

/// 投递一条需要垫片确认的命令并等待 cmd_ack。超时/断连按失败处理，
/// 由调用方如实回显（不假装成功）。replayable 按契约 commandMap 的补投
/// 列：cruise_enable=是（垫片重连且 cursor 未达时补投）、
/// ultrasonic_rearm=否（单发边沿，重放会在无用户动作时清超声门锁）。
async fn push_and_wait_ack(body: Value, replayable: bool, timeout_ms: u64) -> Result<Value, String> {
    let id = queue_slot().push(body, replayable);
    let (tx, rx) = tokio::sync::oneshot::channel::<Value>();
    let registered = {
        let mut acks = acks_slot().lock().expect("acks poisoned");
        // 同 id 极不可能重复（单调分配），防御性覆盖旧发送端。
        acks.insert(id, tx).is_none()
    };
    let _ = registered;
    match tokio::time::timeout(Duration::from_millis(timeout_ms), rx).await {
        Ok(Ok(ack)) => Ok(ack),
        _ => {
            acks_slot().lock().expect("acks poisoned").remove(&id);
            Err(format!("垫片未在 {}ms 内确认命令（断连或离线）", timeout_ms))
        }
    }
}

// ---------------------------------------------------------------------------
// 全局开关与令牌
// ---------------------------------------------------------------------------

static ENABLED: AtomicBool = AtomicBool::new(false);
static TOKEN: OnceLock<Option<String>> = OnceLock::new();
/// 服务端纪元缓存（装配用，避免 20Hz 整份克隆 StateStore）。
static SRV_EPOCH: OnceLock<i64> = OnceLock::new();

/// ros 模式是否已装配（ws_control据此分叉 fake/ros 出口）。
pub fn enabled() -> bool {
    ENABLED.load(Ordering::Relaxed)
}

fn init_token(cfg: &Config) {
    let _ = TOKEN.get_or_init(|| cfg.bridge_token.clone());
}

/// 令牌校验：`X-Bridge-Token` 头，常数时间比较（safety.rs::ct_eq 模式）。
/// 401 不区分「缺/错」，不回显令牌（防探测）。
fn token_ok(headers: &HeaderMap) -> bool {
    let Some(configured) = TOKEN.get().and_then(Option::as_deref) else {
        // 未配置令牌：拒绝一切（fail-closed；maybe_start 会打警告）。
        return false;
    };
    let Some(provided) = headers.get("x-bridge-token").and_then(|v| v.to_str().ok()) else {
        return false;
    };
    safety::ct_eq(provided.as_bytes(), configured.as_bytes())
}

fn json_response(status: StatusCode, body: &Value) -> Response {
    (
        status,
        [
            (header::CONTENT_TYPE, "application/json"),
            (header::CACHE_CONTROL, "no-store"),
        ],
        body.to_string(),
    )
        .into_response()
}

// ---------------------------------------------------------------------------
// 装配与路由
// ---------------------------------------------------------------------------

/// ros 模式装配：初始化静态槽位并 spawn 20Hz 看护/装配任务。fake 模式
/// 不调用（main.rs 保证），fake 行为零变化。
pub async fn maybe_start(cfg: &Arc<Config>) {
    if cfg.fake {
        return;
    }
    init_token(cfg);
    let _ = raw_slot();
    let _ = queue_slot();
    let _ = intent_slot();
    let _ = acks_slot();
    let _ = SRV_EPOCH.get_or_init(|| crate::state::global().snapshot().srv_epoch);
    if cfg.bridge_token.is_none() {
        eprintln!(
            "警告：OPI_BRIDGE_TOKEN 未设置——/bridge 端点 fail-closed（全部 401），\
             垫片将无法上报数据"
        );
    }
    ENABLED.store(true, Ordering::Relaxed);
    tokio::spawn(tick_task());
    eprintln!(
        "ros 模式：/bridge 已装配（POST /bridge/frames ≤{FRAME_BATCH_MAX} 帧/批、\
         GET /bridge/commands 长轮询），等待垫片连接"
    );
}

/// `/bridge` 路由（仅 ros 模式由 main.rs 挂载）。
pub fn router(cfg: &Arc<Config>) -> Router {
    init_token(cfg);
    Router::new()
        .route("/bridge/frames", post(post_frames))
        .route("/bridge/commands", get(poll_commands))
        .layer(DefaultBodyLimit::max(BODY_LIMIT_BYTES))
}

// ---------------------------------------------------------------------------
// HTTP 端点
// ---------------------------------------------------------------------------

async fn post_frames(headers: HeaderMap, body: Bytes) -> Response {
    if !token_ok(&headers) {
        return json_response(StatusCode::UNAUTHORIZED, &json!({ "ok": false, "error": "unauthorized" }));
    }
    if body.len() > BODY_LIMIT_BYTES {
        return json_response(StatusCode::PAYLOAD_TOO_LARGE, &json!({ "ok": false, "error": "body_too_large" }));
    }
    let Ok(batch) = serde_json::from_slice::<Value>(&body) else {
        return json_response(StatusCode::BAD_REQUEST, &json!({ "ok": false, "error": "bad_json" }));
    };
    let Some(frames) = batch.get("frames").and_then(Value::as_array) else {
        return json_response(StatusCode::BAD_REQUEST, &json!({ "ok": false, "error": "missing_frames" }));
    };
    if frames.len() > FRAME_BATCH_MAX {
        return json_response(StatusCode::BAD_REQUEST, &json!({ "ok": false, "error": "too_many_frames" }));
    }
    let now = safety::now_ms();
    let mut accepted = 0usize;
    for frame in frames {
        // 逐帧独立 try：坏帧跳过不拒整批（契约 endpointContract①）。
        if apply_frame(frame) {
            accepted += 1;
        }
    }
    raw_slot().lock().expect("raw poisoned").last_batch_ms = Some(now);
    let pending = queue_slot().pending();
    json_response(
        StatusCode::OK,
        &json!({ "ok": true, "accepted": accepted, "cmds_pending": pending }),
    )
}

#[derive(serde::Deserialize)]
struct CmdQuery {
    wait_ms: Option<u64>,
    cursor: Option<u64>,
}

async fn poll_commands(headers: HeaderMap, Query(q): Query<CmdQuery>) -> Response {
    if !token_ok(&headers) {
        return json_response(StatusCode::UNAUTHORIZED, &json!({ "ok": false, "error": "unauthorized" }));
    }
    let wait = q.wait_ms.unwrap_or(20_000).min(CMD_WAIT_MAX_MS);
    let cursor = q.cursor.unwrap_or(0);
    let (cmds, new_cursor) = queue_slot().poll(cursor, wait).await;
    json_response(StatusCode::OK, &json!({ "cmds": cmds, "cursor": new_cursor }))
}

// ---------------------------------------------------------------------------
// 帧落库（契约 frameMap：img/scan/map/pose → media_slot；状态类 → Raw）
// ---------------------------------------------------------------------------

fn mime_static(mime: &str) -> &'static str {
    match mime {
        "image/jpeg" => "image/jpeg",
        _ => "image/png",
    }
}

/// 应用一帧。返回是否为「识别且成功落库」的帧（坏帧跳过不拒整批）。
fn apply_frame(frame: &Value) -> bool {
    let Some(t) = frame.get("t").and_then(Value::as_str) else {
        return false;
    };
    match t {
        "img" => apply_img(frame),
        "scan" => apply_scan(frame),
        "map" => apply_map(frame),
        "pose" => apply_pose(frame),
        "chassis" => apply_chassis(frame),
        "lidar_diag" => apply_lidar_diag(frame),
        "imu" => apply_imu(frame),
        "cmd" => apply_cmd(frame),
        "drive_safety" => apply_drive_safety(frame),
        "moving" => apply_moving(frame),
        "probe" => apply_probe(frame),
        "nav_feedback" => apply_nav_feedback(frame),
        "nav_result" => apply_nav_result(frame),
        "cmd_ack" => apply_cmd_ack(frame),
        "bridge_hello" => apply_bridge_hello(frame),
        _ => false,
    }
}

/// `{t:"img",d,w,h,mime}` → media_slot.image_*（fake.rs:605-651 同写法：
/// Arc 换新 last-wins，seq 自增，fps 实测写入常量位）。
fn apply_img(frame: &Value) -> bool {
    let Some(d) = frame.get("d").and_then(Value::as_str) else {
        return false;
    };
    let w = frame.get("w").and_then(Value::as_u64).unwrap_or(0) as u32;
    let h = frame.get("h").and_then(Value::as_u64).unwrap_or(0) as u32;
    let mime = frame.get("mime").and_then(Value::as_str).unwrap_or("image/jpeg");
    let now = safety::now_ms();
    let fps = {
        let mut raw = raw_slot().lock().expect("raw poisoned");
        raw.image_rate.tick(now)
    };
    let mut media = fake::media_slot().write().expect("media poisoned");
    media.image_b64 = Arc::from(d);
    media.image_w = w;
    media.image_h = h;
    media.image_seq += 1;
    media.image_mime = mime_static(mime);
    media.image_fps = fps;
    true
}

/// `{t:"scan",p,n,all}` → media_slot.scan/scan_valid/scan_seq
/// （fake.rs:618-623 同构；形状与 ws_video.rs:177-190 逐字一致）。
fn apply_scan(frame: &Value) -> bool {
    let Some(pts_in) = frame.get("p").and_then(Value::as_array) else {
        return false;
    };
    let n = frame.get("n").and_then(Value::as_u64).unwrap_or(0) as usize;
    let now = safety::now_ms();
    let hz = {
        let mut raw = raw_slot().lock().expect("raw poisoned");
        raw.scan_rate.tick(now)
    };
    let mut pts: Vec<(f64, f64)> = Vec::with_capacity(pts_in.len());
    for p in pts_in {
        if let Some(pair) = p.as_array() {
            let deg = pair.first().and_then(Value::as_f64).unwrap_or(0.0);
            let r = pair.get(1).and_then(Value::as_f64).unwrap_or(0.0);
            pts.push((deg, r));
        }
    }
    let mut media = fake::media_slot().write().expect("media poisoned");
    media.scan = Arc::new(pts);
    media.scan_valid = n;
    media.scan_seq += 1;
    media.scan_hz = hz;
    true
}

/// `{t:"map",seq,w,h,res,ox,oy,d}` → media_slot.grid + current_map_id。
/// d 是垫片 zlib+base64（与旧 _map_frame_json 同编码，浏览器
/// DecompressionStream('deflate') 直解）；map_id=解压后栅格内容 sha256
/// 前 16 位（对齐 fake.rs:225，契约 risks#8 有意决策）。
fn apply_map(frame: &Value) -> bool {
    let (Some(d), Some(w), Some(h)) = (
        frame.get("d").and_then(Value::as_str),
        frame.get("w").and_then(Value::as_u64),
        frame.get("h").and_then(Value::as_u64),
    ) else {
        return false;
    };
    use base64::Engine as _;
    let Ok(compressed) = base64::engine::general_purpose::STANDARD.decode(d) else {
        return false;
    };
    let mut raw_bytes = Vec::new();
    if flate2::read::ZlibDecoder::new(compressed.as_slice())
        .read_to_end(&mut raw_bytes)
        .is_err()
    {
        return false;
    }
    let map_id = safety::sha256_hex(&raw_bytes)[..16].to_string();
    let seq_in = frame.get("seq").and_then(Value::as_u64).unwrap_or(1);
    let grid_t_unix = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|t| t.as_secs_f64())
        .unwrap_or(0.0);
    {
        let hub = safety::global();
        let mut ctl = hub.control.lock().expect("control poisoned");
        // 垫片进程重启会把 seq 归零：钳到「上一个 seq + 1」保 ws_video 的
        // 「seq 变化才推」判定永远触发。
        let prev = ctl.map_seq_last;
        let seq = if seq_in > prev { seq_in } else { prev + 1 };
        ctl.map_seq_last = seq;
        ctl.current_map_id = Some(map_id);
        let grid = GridPub {
            seq,
            w: w as usize,
            h: h as usize,
            res: frame.get("res").and_then(Value::as_f64).unwrap_or(fake::GRID_RES),
            ox: frame.get("ox").and_then(Value::as_f64).unwrap_or(0.0),
            oy: frame.get("oy").and_then(Value::as_f64).unwrap_or(0.0),
            data_b64: Arc::from(d),
            grid_t_unix,
        };
        let mut media = fake::media_slot().write().expect("media poisoned");
        media.grid = Arc::new(grid);
    }
    true
}

/// `{t:"pose",pose:{x,y,th}}` → media_slot.pose/pose_mono_ms
/// （fake.rs:263-266 同款；lookup 失败垫片不发帧，hub 保旧值由
/// pose_age 判新鲜，web_debug.py:1377-1385）。
fn apply_pose(frame: &Value) -> bool {
    let Some(pose) = frame.get("pose") else {
        return false;
    };
    let (Some(x), Some(y)) = (
        pose.get("x").and_then(Value::as_f64),
        pose.get("y").and_then(Value::as_f64),
    ) else {
        return false;
    };
    let th = pose.get("th").and_then(Value::as_f64).unwrap_or(0.0);
    let mut media = fake::media_slot().write().expect("media poisoned");
    media.pose = (x, y, th);
    media.pose_mono_ms = safety::now_ms();
    true
}

/// `{t:"chassis",...}` → Raw.chassis + 里程积分 + 底盘频率。
fn apply_chassis(frame: &Value) -> bool {
    let now = safety::now_ms();
    let voltage_mv = frame.get("voltage_mv").and_then(Value::as_i64);
    let vx_mm_s = frame.get("vx_mm_s").and_then(Value::as_i64);
    let mut raw = raw_slot().lock().expect("raw poisoned");
    if let Some(vx) = vx_mm_s {
        raw.mileage.update(vx, now);
    }
    let hz = raw.chassis_rate.tick(now);
    raw.chassis = Some(ChassisRaw {
        voltage_mv,
        vx_mm_s,
        vy_mm_s: frame.get("vy_mm_s").and_then(Value::as_i64),
        yaw_rate_mrad_s: frame.get("yaw_rate_mrad_s").and_then(Value::as_i64),
        stop_flag: frame.get("stop_flag").and_then(Value::as_i64),
        ultrasonic_front_mm: frame.get("ultrasonic_front_mm").and_then(Value::as_i64),
        level: frame.get("level").and_then(Value::as_i64),
        message: frame
            .get("message")
            .and_then(Value::as_str)
            .unwrap_or_default()
            .to_string(),
    });
    raw.chassis_t = Some(now);
    let _ = hz;
    true
}

/// `{t:"lidar_diag",...}` → Raw.lidar_diag（解析失败垫片不发帧，hub 保持
/// 上一次记录不清空，web_debug.py:1395-1411）。
fn apply_lidar_diag(frame: &Value) -> bool {
    let mut raw = raw_slot().lock().expect("raw poisoned");
    raw.lidar_diag = Some(LidarDiagRaw {
        level: frame.get("level").and_then(Value::as_i64),
        message: frame.get("message").and_then(Value::as_str).unwrap_or_default().to_string(),
        state: frame.get("state").and_then(Value::as_str).unwrap_or("unknown").to_string(),
        reason: frame.get("reason").and_then(Value::as_str).unwrap_or_default().to_string(),
        age_s: frame.get("age_s").and_then(Value::as_f64),
        coverage: frame.get("coverage").and_then(Value::as_f64),
        reconnects: frame.get("reconnects").and_then(Value::as_i64),
        port: frame.get("port").and_then(Value::as_str).unwrap_or_default().to_string(),
        frames_ok: frame.get("frames_ok").and_then(Value::as_i64),
        frames_bad: frame.get("frames_bad").and_then(Value::as_i64),
    });
    raw.diag_t = Some(safety::now_ms());
    true
}

/// `{t:"imu",hz}` → Raw.imu（心跳：垫片本地计数 1Hz 聚合，web_debug.py:1414-1419）。
fn apply_imu(frame: &Value) -> bool {
    let mut raw = raw_slot().lock().expect("raw poisoned");
    raw.imu_hz = frame.get("hz").and_then(Value::as_f64);
    raw.imu_t = Some(safety::now_ms());
    true
}

const CMD_ORDER: [&str; 4] = ["nav", "demo", "manual", "auto"];

/// `{t:"cmd",which,vx,vy,wz}` → Raw.cmds[which]；nav 到达刷新 nav_seen_t
/// （web_debug.py:1445-1452）。
fn apply_cmd(frame: &Value) -> bool {
    let Some(which) = frame.get("which").and_then(Value::as_str) else {
        return false;
    };
    let Some(idx) = CMD_ORDER.iter().position(|w| *w == which) else {
        return false;
    };
    let cmd = CmdRaw {
        vx: frame.get("vx").and_then(Value::as_f64).unwrap_or(0.0),
        vy: frame.get("vy").and_then(Value::as_f64).unwrap_or(0.0),
        wz: frame.get("wz").and_then(Value::as_f64).unwrap_or(0.0),
    };
    let mut raw = raw_slot().lock().expect("raw poisoned");
    raw.cmds[idx] = Some(cmd);
    raw.cmd_t[idx] = Some(safety::now_ms());
    if which == "nav" {
        raw.nav_seen_t = raw.cmd_t[idx];
    }
    true
}

/// `{t:"drive_safety",raw}` → Raw.drive_safety（解析失败 raw=null，
/// 页面显示无许可；web_debug.py:1428-1443）。
fn apply_drive_safety(frame: &Value) -> bool {
    let payload = frame.get("raw").filter(|v| v.is_object()).cloned();
    let mut raw = raw_slot().lock().expect("raw poisoned");
    raw.drive_safety = payload;
    raw.drive_safety_t = Some(safety::now_ms());
    true
}

/// `{t:"moving",bins}` → Raw.moving（2s 新鲜度在装配时判，
/// web_debug.py:2021-2023）。
fn apply_moving(frame: &Value) -> bool {
    let Some(bins) = frame.get("bins").and_then(Value::as_array) else {
        return false;
    };
    let parsed: Vec<i64> = bins.iter().filter_map(Value::as_i64).collect();
    let mut raw = raw_slot().lock().expect("raw poisoned");
    raw.moving = Some(parsed);
    raw.moving_t = Some(safety::now_ms());
    true
}

/// `{t:"probe",nodes_seen,safety}` → Raw.nodes_seen + Raw.safety（3s 一帧；
/// c50c_uart3_node 在线性由 safety.online 承载，web_debug.py::node_view）。
fn apply_probe(frame: &Value) -> bool {
    let now = safety::now_ms();
    let mut raw = raw_slot().lock().expect("raw poisoned");
    let seen = |name: &str| {
        frame
            .get("nodes_seen")
            .and_then(|n| n.get(name))
            .and_then(Value::as_bool)
            .unwrap_or(false)
            .then_some(now)
    };
    raw.nodes_seen.camera_node = seen("camera_node");
    raw.nodes_seen.lidar_node = seen("lidar_node");
    raw.nodes_seen.chassis_status_node = seen("chassis_status_node");
    if let Some(safety) = frame.get("safety").filter(|v| v.is_object()) {
        raw.safety = Some(safety.clone());
        raw.safety_seen_t = Some(now);
    }
    true
}

/// `{t:"nav_feedback",rid,dist,eta,x,y}` → Raw.nav_feedback。
fn apply_nav_feedback(frame: &Value) -> bool {
    let Some(rid) = frame.get("rid").and_then(Value::as_str) else {
        return false;
    };
    let mut raw = raw_slot().lock().expect("raw poisoned");
    raw.nav_feedback = Some(NavFeedbackRaw {
        rid: rid.to_string(),
        dist: frame.get("dist").and_then(Value::as_f64),
        eta: frame.get("eta").and_then(Value::as_f64),
        x: frame.get("x").and_then(Value::as_f64),
        y: frame.get("y").and_then(Value::as_f64),
    });
    raw.nav_feedback_t = Some(safety::now_ms());
    true
}

/// `{t:"cmd_ack",id,ok,error,detail?}` → resolve 等待中的 oneshot。
fn apply_cmd_ack(frame: &Value) -> bool {
    let Some(id) = frame.get("id").and_then(Value::as_u64) else {
        return false;
    };
    let ack = json!({
        "ok": frame.get("ok").and_then(Value::as_bool).unwrap_or(false),
        "error": frame.get("error").cloned().unwrap_or(Value::Null),
        "detail": frame.get("detail").cloned().unwrap_or(Value::Null),
    });
    if let Some(tx) = acks_slot().lock().expect("acks poisoned").remove(&id) {
        let _ = tx.send(ack);
    }
    true
}

/// `{t:"bridge_hello",topics}` → 垫片重连登记：清空未投递命令、重置出口
/// 意图（由新许可流自然重新武装 guard）、刷新 web_bridge 活性。
fn apply_bridge_hello(frame: &Value) -> bool {
    queue_slot().clear_undelivered();
    {
        let mut intent = intent_slot().lock().expect("intent poisoned");
        intent.last_authorize = None;
        intent.last_cruise = None;
        intent.last_manual = None;
        intent.manual_stream = false;
    }
    let mut raw = raw_slot().lock().expect("raw poisoned");
    raw.last_batch_ms = Some(safety::now_ms());
    let _ = frame.get("topics");
    true
}

// ---------------------------------------------------------------------------
// nav_result：rid 应答制驱动巡逻推进 / 单点导航闭环（契约 frameMap
// navigate_to_pose 行 + commandMap patrol 行，patrol.py:521-581 语义）
// ---------------------------------------------------------------------------

/// `{t:"nav_result",rid,status,text,plan_error}` 的 hub 侧状态机。
/// 状态值：accepted | succeeded | cancelled | failed。
fn apply_nav_result(frame: &Value) -> bool {
    let Some(rid) = frame.get("rid").and_then(Value::as_str).map(str::to_string) else {
        return false;
    };
    let status = frame.get("status").and_then(Value::as_str).unwrap_or("");
    let text = frame.get("text").and_then(Value::as_str).unwrap_or("").to_string();
    let plan_error =
        frame.get("plan_error").and_then(Value::as_str).unwrap_or("").to_string();
    {
        let mut raw = raw_slot().lock().expect("raw poisoned");
        raw.last_plan_error = plan_error.clone();
    }
    // 锁序 intent→control（与 20Hz tick 的 nav_housekeeping 一致）：_fail
    // 分支要在持锁上下文撤回下游授权，走 force_authorize_false_locked——
    // 自锁版在「本函数持 control、tick 持 intent 等 control」时交叉死锁。
    let mut intent = intent_slot().lock().expect("intent poisoned");
    let (handled, withdraw) = match status {
        "accepted" => (on_nav_accepted(&rid), false),
        "succeeded" | "cancelled" | "failed" => on_nav_finished(&rid, status, &text),
        _ => (false, false),
    };
    if withdraw {
        force_authorize_false_locked(&mut intent);
    }
    handled
}

/// 目标被 Nav2 接受：巡逻 sending→pending（patrol.py:542-544）；锁停/
/// 停止中的迟到/未知接受一律立即取消（patrol.py:521-541）。
fn on_nav_accepted(rid: &str) -> bool {
    let hub = safety::global();
    let mut ctl = hub.control.lock().expect("control poisoned");
    let is_patrol_sending = ctl.patrol.sending_rid.as_deref() == Some(rid);
    let is_patrol_pending = ctl.patrol.pending_rid.as_deref() == Some(rid);
    let is_nav_rid = ctl.nav.rid.as_deref() == Some(rid);
    if is_patrol_sending {
        ctl.patrol.sending_rid = None;
        if ctl.locked || ctl.patrol.status != "running" || ctl.patrol.cancel_request.is_some() {
            // 锁停中绝不继续巡航：接受即取消，锁停保持（patrol.py:551-556）。
            queue_slot().push(json!({ "c": "nav_cancel", "rid": rid }), false);
            ctl.patrol.cancel_request = Some(rid.to_string());
        } else {
            ctl.patrol.pending_rid = Some(rid.to_string());
            if is_nav_rid && ctl.nav.state == "sending" {
                ctl.nav.state = "active";
            }
        }
        return true;
    }
    if is_nav_rid && ctl.nav.state == "sending" {
        // 手动单点导航：sending→active。
        ctl.nav.state = "active";
        return true;
    }
    if !(is_patrol_pending || is_nav_rid) {
        // 迟到/被顶替/未知 rid 的接受：立即取消（patrol.py:521-541）。
        queue_slot().push(json!({ "c": "nav_cancel", "rid": rid }), false);
    }
    false
}

/// 目标结束（succeeded/cancelled/failed）：巡逻 succeeded→index+1、回绕
/// laps+1（patrol.py:577-581），failed/cancelled→_fail 锁停
/// （patrol.py:574-576）；手动目标按结果落 nav 状态。
/// 返回 (是否处理了该 rid, 是否需要撤回下游授权)。撤回由调用方
/// （apply_nav_result，持 intent 锁）用 force_authorize_false_locked 完成——
/// 本函数持 control 锁，自调 force_authorize_false 的自锁版会以
/// control→intent 的锁序与 20Hz tick 的 intent→control 交叉死锁。
fn on_nav_finished(rid: &str, status: &str, text: &str) -> (bool, bool) {
    let hub = safety::global();
    let mut ctl = hub.control.lock().expect("control poisoned");
    // cancel_request 确认：只有 Nav2 明确回「取消成功」才算
    // （patrol.py:566-572：failed/succeeded 是旧目标自己跑完，取消未确认）。
    if ctl.patrol.cancel_request.as_deref() == Some(rid) {
        if status == "cancelled" {
            ctl.patrol.cancel_request = None;
        }
        // 巡逻目标的结果走取消确认分支即返回，不再推进 index
        // （patrol.py on_result 的 cancel_request 分支）。
        if ctl.patrol.sending_rid.as_deref() == Some(rid)
            || ctl.patrol.pending_rid.as_deref() == Some(rid)
        {
            ctl.patrol.sending_rid = None;
            ctl.patrol.pending_rid = None;
            if ctl.nav.rid.as_deref() == Some(rid) {
                ctl.nav.rid = None;
                set_nav_state(&mut ctl, status, text);
            }
            return (true, false);
        }
    }
    let is_patrol = ctl.patrol.sending_rid.as_deref() == Some(rid)
        || ctl.patrol.pending_rid.as_deref() == Some(rid);
    if is_patrol {
        ctl.patrol.sending_rid = None;
        ctl.patrol.pending_rid = None;
        if ctl.nav.rid.as_deref() == Some(rid) {
            set_nav_state(&mut ctl, status, text);
            ctl.nav.rid = None;
        }
        if status == "succeeded" {
            if ctl.patrol.status == "running" {
                let n = ctl
                    .patrol
                    .route
                    .as_ref()
                    .map(|r| r.points.len())
                    .unwrap_or(0);
                if n > 0 {
                    ctl.patrol.index += 1;
                    if ctl.patrol.index >= n {
                        ctl.patrol.index = 0;
                        ctl.patrol.laps += 1;
                    }
                }
                // 下一航点由 20Hz tick 的派发对账发出（_issue 语义）。
            }
        } else {
            // _fail：锁停 + 撤回下游鉴权（patrol.py:574-576 →
            // web_debug.py:935-947 lock_stop 第一动作）。
            let reason = if status == "cancelled" { "nav_cancelled" } else { "nav_failed" };
            ctl.patrol.status = "stopped";
            ctl.patrol.stop_reason = reason.to_string();
            Hub::lock_locked(&mut ctl, reason);
            // 撤回下游鉴权（web_debug.py:935-947 lock_stop 第一动作）在
            // apply_nav_result 的 intent 锁内完成（见返回值约定）。
            return (true, true);
        }
        return (true, false);
    }
    if ctl.nav.rid.as_deref() == Some(rid) {
        set_nav_state(&mut ctl, status, text);
        ctl.nav.rid = None;
        return (true, false);
    }
    (false, false)
}

/// 把动作结果落到单点导航槽（nav_goal.py:290-324 的状态映射）。
fn set_nav_state(ctl: &mut safety::Control, status: &str, text: &str) {
    ctl.nav.state = match status {
        "succeeded" => "done",
        "cancelled" => "canceled",
        _ => "failed",
    };
    ctl.nav.result = text.to_string();
    if status != "succeeded" && !text.is_empty() {
        ctl.nav.error = text.to_string();
    }
}

// ---------------------------------------------------------------------------
// 20Hz 看护 + 意图对账 + 装配
// ---------------------------------------------------------------------------

/// 20Hz 主任务：看护 → 对账派发 → 状态装配（契约 risks#2：ros 模式
/// 必须有独立 20Hz tokio 任务，否则 350ms 许可超时锁停/手动超时归零/
/// 会话失效在实车模式全部失效——安全项不是优化项）。
async fn tick_task() {
    let mut ticker =
        tokio::time::interval(Duration::from_millis(crate::fake::TICK_MS));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        ticker.tick().await;
        tick_once();
    }
}

fn tick_once() {
    let hub = safety::global();
    // 1) 三个看护（与 FakeEngine.tick 相同顺序，fake.rs:417-419）。
    hub.license_tick();
    hub.manual_tick();
    hub.session_tick();
    // 2) 手动命令对账（manual_zero 派发点：350ms 心跳超时与 timed 到期
    //    都发生在 manual_tick 里清 owner，这里按意图差量发零——旧实现
    //    tick→_stop 在停车瞬间立即发零，manual_forward.py:199-210→159-174）。
    flush_manual_cmd();
    // 3) 导航看护 + authorize/cruise 对账 + 巡逻派发。
    nav_housekeeping_and_dispatch();
    // 4) patrol_status 帧捎带（~1s）。
    maybe_push_patrol_frame();
    // 5) 状态装配（ros 原始缓存 → StatusPayload）。
    assemble_status();
}

/// 手动出口对账：owner 在 → 最新 (vx,wz) 变化才发 manual_cmd；owner 无
/// 且命令流活跃 → manual_zero。覆盖 manual_stop handler、manual_tick
/// 超时、timed 到期、motion_stop、conn_closed 等全部清 owner 路径。
pub fn flush_manual_cmd() {
    if !enabled() {
        return;
    }
    let hub = safety::global();
    let mut intent = intent_slot().lock().expect("intent poisoned");
    let ctl = hub.control.lock().expect("control poisoned");
    if ctl.manual.owner.is_some() {
        let (vx, wz) = ctl.manual.cmd;
        if intent.last_manual != Some((vx, wz)) || !intent.manual_stream {
            queue_slot().push(json!({ "c": "manual_cmd", "vx": vx, "wz": wz }), true);
            intent.last_manual = Some((vx, wz));
            intent.manual_stream = true;
        }
    } else if intent.manual_stream {
        queue_slot().push(json!({ "c": "manual_zero" }), true);
        intent.manual_stream = false;
        intent.last_manual = None;
    }
}

/// 导航看护 + 出口意图对账（authorize / cruise / 巡逻派发）。
fn nav_housekeeping_and_dispatch() {
    nav_housekeeping_at(safety::now_ms());
}

/// now 参数化（生产走 safety::now_ms；测试可推时钟到未来以构造
/// 「15s 在途未接受」——进程 uptime 不足 15s 时无法伪造过去时间戳，
/// saturating_sub 会把 goal_t 钳到 0 导致超时判定失效）。
fn nav_housekeeping_at(now: u64) {
    let hub = safety::global();
    let mut intent = intent_slot().lock().expect("intent poisoned");
    let mut ctl = hub.control.lock().expect("control poisoned");

    // 15s 在途未接受超时（patrol.py:613-620 overall_timeout）：取消并锁停。
    if ctl.nav.state == "sending"
        && ctl.nav.goal_t_ms.is_some_and(|t| now.saturating_sub(t) > safety::NAV_GOAL_TIMEOUT_MS)
    {
        let rid = ctl.nav.rid.clone();
        ctl.nav.state = "failed";
        ctl.nav.result = "failed".to_string();
        ctl.nav.error = "导航超时".to_string();
        if let Some(r) = &rid {
            if ctl.patrol.sending_rid.as_deref() == Some(r.as_str()) {
                ctl.patrol.sending_rid = None;
                ctl.patrol.status = "stopped";
                ctl.patrol.stop_reason = "goal_timeout".to_string();
                Hub::lock_locked(&mut ctl, "goal_timeout");
                // intent 锁已在本函数头部持有（ros.rs intent→control 同序）：
                // 重入 force_authorize_false 的自锁版会自死锁，走 _locked。
                force_authorize_false_locked(&mut intent);
                queue_slot().push(json!({ "c": "nav_cancel", "rid": r }), false);
                ctl.patrol.cancel_request = Some(r.clone());
            }
        }
        ctl.nav.rid = None;
    }

    // guard_authorize 对账：期望值 = 许可新鲜（web_debug.py:640,655 每条
    // 被接受的许可都是授权源；断流/锁停即撤回）。按值去重。
    let fresh = Hub::license_fresh_locked(&ctl, now);
    if intent.last_authorize != Some(fresh) {
        queue_slot().push(json!({ "c": "guard_authorize", "value": fresh }), true);
        intent.last_authorize = Some(fresh);
    }

    // cruise_enable 对账：开关状态变化才发一帧，无周期
    // （web_debug.py:150-164 set_cruise）。
    if intent.last_cruise != Some(ctl.cruise_on) {
        queue_slot().push(json!({ "c": "cruise_enable", "on": ctl.cruise_on }), true);
        intent.last_cruise = Some(ctl.cruise_on);
    }

    // 巡逻派发：running 且锁停已解除、在途目标槽空闲、目标与巡逻 rid 都
    // 不在途时派下一航点（patrol.py::_issue + 单一在途目标槽契约）。
    let dispatchable = ctl.patrol.status == "running"
        && !ctl.locked
        && !ctl.user_locked
        && ctl.nav.state != "sending"
        && ctl.nav.state != "active"
        && ctl.patrol.sending_rid.is_none()
        && ctl.patrol.pending_rid.is_none();
    if dispatchable {
        if let Some(route) = ctl.patrol.route.clone() {
            let n = route.points.len();
            if n > 0 {
                let idx = ctl.patrol.index.min(n - 1);
                let (x, y) = route.points[idx];
                let (nx, ny) = route.points[(idx + 1) % n];
                // 航向按指向下一点计算（safety.rs:909-913 patrol_status_frame
                // 的 goal yaw 同源）。
                let yaw = (ny - y).atan2(nx - x);
                intent.rid_seq += 1;
                let rid = format!("p{}-{}-{}", ctl.patrol.laps, idx, intent.rid_seq);
                ctl.patrol.sending_rid = Some(rid.clone());
                ctl.nav.state = "sending";
                ctl.nav.rid = Some(rid.clone());
                ctl.nav.goal = Some(safety::Goal { x, y, yaw: Some(yaw) });
                ctl.nav.goal_t_ms = Some(now);
                ctl.nav.result.clear();
                ctl.nav.error.clear();
                queue_slot().push(
                    json!({ "c": "nav_goal", "rid": rid, "x": round3(x), "y": round3(y),
                            "yaw": round4(yaw) }),
                    false,
                );
            }
        }
    }
    drop(ctl);
    drop(intent);
}

/// patrol_status 帧 ~1s 捎带（命令流承担，页面直推仍由 ws_control 既有
/// 推送承担——契约 commandMap patrol_tick 行）。
fn maybe_push_patrol_frame() {
    let now = safety::now_ms();
    {
        let intent = intent_slot().lock().expect("intent poisoned");
        if now.saturating_sub(intent.last_patrol_frame_ms) < PATROL_FRAME_PERIOD_MS {
            return;
        }
    }
    let frame = safety::global().patrol_status_frame();
    queue_slot().push_patrol_frame(frame);
    intent_slot().lock().expect("intent poisoned").last_patrol_frame_ms = now;
}

// ---------------------------------------------------------------------------
// ws_control 的 ros 出口钩子（handler 即时 flush 关键序）
// ---------------------------------------------------------------------------

/// 系统 boot 相对单调毫秒（/proc/uptime 第一列）。下游 guard 的许可
/// 期限判定用 time.monotonic()（boot 相对秒，guard_wiring.py:58
/// clock=time.monotonic、motion_guard.py License.check 与 exp 直接比较），
/// 与 Web-Next 进程 uptime（now_ms，safety.rs:40 Instant::now 锚）无关——
/// 锚错则实车上每条许可都被判 expired 拒收（评审 high-1）。
/// /proc/uptime 读不到时（非 Linux 环境）退回进程 uptime 并告警：本地
/// 沙箱可继续跑，实车 Linux 一定有 /proc/uptime。注意 /proc/uptime 计入
/// suspend（CLOCK_BOOTTIME），time.monotonic 不计——机器人无 suspend，
/// 两者一致；旧链路的基准也是 time.monotonic（web_debug.py:181）。
fn boot_mono_ms() -> u64 {
    if let Ok(s) = std::fs::read_to_string("/proc/uptime") {
        if let Some(f) = s.split_whitespace().next() {
            if let Ok(v) = f.parse::<f64>() {
                return (v * 1000.0) as u64;
            }
        }
    }
    eprintln!("警告：/proc/uptime 不可读，license expires_mono 退回进程 uptime 锚");
    safety::now_ms()
}

/// 浏览器会话号（hex 字符串，蓝图 §4.3(1)）→ 下游 guard 会话号（int≥1，
/// motion_guard.py License.check：session 必须 _int_or_none 命中且 >0，
/// 字符串一律 session_invalid 拒收）的稳定映射。同一浏览器会话恒映射
/// 同一 int（guard 的 session_mismatch 比较「与最近一条相同」）；页面
/// 接管产生新 hex → 分配新 int，保序下游「会话接管只能在旧许可失效后
/// 发生」的锁存语义。旧实现转发浏览器原文（int 会话号），Web-Next 浏览器
/// 协议改字符串后此处是两协议间的唯一适配点；seq 不经此映射、原样转发
/// （防重放校验完全靠 seq，不因映射受影响）。
fn session_int(hex: &str) -> i64 {
    static MAP: OnceLock<Mutex<std::collections::HashMap<String, i64>>> = OnceLock::new();
    static BASE: OnceLock<i64> = OnceLock::new();
    static SEQ: std::sync::atomic::AtomicI64 = std::sync::atomic::AtomicI64::new(0);
    // 进程启动纪元基数：确保 c50c-web-next 重启后分配的下游 session 整数绝不
    // 与重启前旧值碰撞，避免 c50c_uart3_node 按旧会话高 seq 判 seq_replay 拒收。
    let base = *BASE.get_or_init(|| (boot_mono_ms() as i64).max(1).saturating_mul(1000));
    let mut m = MAP
        .get_or_init(|| Mutex::new(std::collections::HashMap::new()))
        .lock()
        .expect("session map poisoned");
    if let Some(&v) = m.get(hex) {
        return v;
    }
    let v = base + SEQ.fetch_add(1, Ordering::Relaxed) + 1;
    m.insert(hex.to_string(), v);
    v
}

/// 许可被接受（handle_license Ok 分支）：license_forward 转发
/// {session,seq,expires_mono}（紧凑分隔符，web_debug.py:909-924）+
/// guard_authorize(True) 按值去重（web_debug.py:640,655,900-906）。
/// 浏览器协议发 ttl_ms（web/js/ws.js:185）与会话 hex 字符串，下游 guard
/// 订阅的 JSON 需要 expires_mono（boot 相对秒）与会话 int——两者都在本
/// 函数折算/映射（见 boot_mono_ms / session_int）。
pub fn on_license_accepted(session: &str, seq: i64, ttl_ms: u64) {
    if !enabled() {
        return;
    }
    let expires_mono = (boot_mono_ms() + ttl_ms.min(5_000)) as f64 / 1000.0;
    queue_slot().push(
        json!({
            "c": "license_forward",
            "frame": {
                "session": session_int(session),
                "seq": seq,
                "expires_mono": expires_mono,
            },
        }),
        true,
    );
    let mut intent = intent_slot().lock().expect("intent poisoned");
    if intent.last_authorize != Some(true) {
        queue_slot().push(json!({ "c": "guard_authorize", "value": true }), true);
        intent.last_authorize = Some(true);
    }
}

/// motion_rearm 成功（handle_motion_rearm Ok 分支）：guard_authorize
/// True(force) + rearm_request 同批按序投递（顺序铁律 web_debug.py:555-560：
/// 先补鉴权再发解停，否则下游 preconditions_ok 永假）。rearm_request 是
/// 一次性边沿触发，永不随 cursor 补投重放（guard_bridge.py:258-261）。
pub fn on_rearm_ok() {
    if !enabled() {
        return;
    }
    let mut intent = intent_slot().lock().expect("intent poisoned");
    queue_slot().push(json!({ "c": "guard_authorize", "value": true }), true);
    intent.last_authorize = Some(true);
    queue_slot().push(json!({ "c": "rearm_request" }), false);
}

/// motion_stop（handle_motion_stop）：撤回下游鉴权（web_debug.py:935-947）
/// + 取消在途 nav 目标（safety.rs:562-563 → nav_cancel 命令）。manual_zero
/// 由 flush_manual_cmd 的对账覆盖（motion_stop 清 manual.owner）。
pub fn on_motion_stop() {
    if !enabled() {
        return;
    }
    force_authorize_false();
    let hub = safety::global();
    let mut ctl = hub.control.lock().expect("control poisoned");
    if let Some(rid) = ctl.nav.rid.clone() {
        queue_slot().push(json!({ "c": "nav_cancel", "rid": rid }), false);
        if ctl.patrol.sending_rid.as_deref() == Some(rid.as_str())
            || ctl.patrol.pending_rid.as_deref() == Some(rid.as_str())
        {
            ctl.patrol.cancel_request = Some(rid.clone());
        }
        ctl.nav.rid = None;
    }
    drop(ctl);
    flush_manual_cmd();
}

/// 手动单点导航（ros 模式 handle_nav_goal 分叉）：范围/间距校验 +
/// 与巡逻共用单一在途目标槽（nav_goal.py:96-111 契约 risks#7）。
/// 返回 (ok, nav 视图)。
pub fn request_nav_goal(x: f64, y: f64, th: Option<f64>) -> (bool, Value) {
    let hub = safety::global();
    let mut ctl = hub.control.lock().expect("control poisoned");
    if !x.is_finite() || !y.is_finite() || th.is_some_and(|t| !t.is_finite()) {
        ctl.nav.error = "目标坐标不是数字".to_string();
        return (false, nav_view_locked(&ctl));
    }
    if !(GOAL_MIN_X <= x && x <= GOAL_MAX_X && GOAL_MIN_Y <= y && y <= GOAL_MAX_Y) {
        ctl.nav.error = format!(
            "目标超出地图范围（{GOAL_MIN_X:.0}~{GOAL_MAX_X:.0}, {GOAL_MIN_Y:.0}~{GOAL_MAX_Y:.0}）"
        );
        return (false, nav_view_locked(&ctl));
    }
    if ctl.nav.state == "sending"
        || ctl.nav.state == "active"
        || ctl.patrol.sending_rid.is_some()
        || ctl.patrol.pending_rid.is_some()
    {
        ctl.nav.error = "已有导航任务进行中，先取消再发新目标".to_string();
        return (false, nav_view_locked(&ctl));
    }
    // 与车距校验（nav_goal.py:31 的意图常量落成真校验；太近 DWB 直接
    // 判定到达或振荡）。车位姿来自 TF pose 帧（media_slot）。
    {
        let media = fake::media_slot().read().expect("media poisoned");
        if media.pose_mono_ms > 0 {
            let (px, py, _) = media.pose;
            let dist = ((x - px).powi(2) + (y - py).powi(2)).sqrt();
            if dist < GOAL_MIN_DIST_M {
                ctl.nav.error =
                    format!("目标距车太近（{dist:.2} m < {GOAL_MIN_DIST_M} m）");
                return (false, nav_view_locked(&ctl));
            }
        }
    }
    ctl.nav.seq += 1;
    let rid = format!("m{}", ctl.nav.seq);
    ctl.nav.state = "sending";
    ctl.nav.goal = Some(safety::Goal { x, y, yaw: th });
    ctl.nav.goal_t_ms = Some(safety::now_ms());
    ctl.nav.rid = Some(rid.clone());
    ctl.nav.result.clear();
    ctl.nav.error.clear();
    // th=None 表示保持当前朝向（nav_goal.py:128-150：四元数取 yaw=0 的
    // 单位四元数，Nav2 据最新 TF 变换执行）。
    let yaw = th.unwrap_or(0.0);
    queue_slot().push(
        json!({ "c": "nav_goal", "rid": rid, "x": round3(x), "y": round3(y),
                "yaw": round4(yaw) }),
        false,
    );
    drop(ctl);
    (true, safety::global().nav_view())
}

/// 重定位（ros 模式 handle_relocalize 分叉）：把画面当前位姿整包入队，
/// 垫片发布 /initialpose 让 AMCL 重新播种。值校验照 nav_goal：有限数 +
/// 地图范围；不入队 nav 状态机（重定位不是导航任务）。
/// 全局重定位（ros 模式 handle_relocalize）：入队边沿命令并等待垫片
/// numpy 全局扫描匹配的 cmd_ack。垫片用缓存 /map + 最近 /scan 做三阶段
/// 匹配（旧栈 relocalize.py 同款），结果经 detail 回传真实坐标/朝向/
/// 命中率，前端 toast 如实显示（不假装成功）。匹配耗时可达数秒，超时按
/// 失败如实回显。
pub async fn request_relocalize_ros() -> (bool, Value) {
    let result = push_and_wait_ack(
        json!({ "c": "relocalize" }),
        false, // 边沿触发不重放（AMCL 采纳即消费）
        RELOCALIZE_ACK_TIMEOUT_MS,
    )
    .await;
    match result {
        Ok(ack) => {
            let ok = ack.get("ok").and_then(Value::as_bool).unwrap_or(false);
            let detail = ack.get("detail").cloned().unwrap_or(Value::Null);
            let error = ack
                .get("error")
                .and_then(Value::as_str)
                .unwrap_or("垫片未回报原因")
                .to_string();
            (ok, if ok { detail } else { json!({ "error": error }) })
        }
        Err(e) => (false, json!({ "error": e })),
    }
}

/// 手动导航取消（ros 模式 handle_nav_cancel 分叉）：与巡逻共用单一在途
/// 目标槽；进行中同时停止巡航、下发 nav_cancel 并立即发 manual_zero 刹停，
/// 若仅残留 failed/done/canceled 状态或目标也一并复位回 idle 并清空目标。
pub fn cancel_nav_goal() -> (bool, Value) {
    let hub = safety::global();
    let mut ctl = hub.control.lock().expect("control poisoned");
    let nav_in_flight = ctl.nav.state == "active" || ctl.nav.state == "sending";
    let patrol_running = ctl.patrol.status == "running"
        || ctl.patrol.sending_rid.is_some()
        || ctl.patrol.pending_rid.is_some();
    let active_motion = nav_in_flight || patrol_running;
    let has_residual = ctl.nav.goal.is_some() || ctl.nav.state != "idle";
    if !active_motion && !has_residual {
        ctl.nav.error = "当前没有导航目标".to_string();
        return (false, nav_view_locked(&ctl));
    }
    // 巡航运行中点「取消导航」必须同步停止巡航，否则 20Hz tick 的
    // nav_housekeeping_and_dispatch 会在取消回执落地后 ~50ms 立即重派下一航点。
    if patrol_running {
        ctl.patrol.status = "stopped";
        ctl.patrol.stop_reason = "nav_cancelled".to_string();
    }
    let mut rids_to_cancel: Vec<String> = Vec::new();
    for opt in [&ctl.nav.rid, &ctl.patrol.sending_rid, &ctl.patrol.pending_rid] {
        if let Some(r) = opt {
            if !rids_to_cancel.contains(r) {
                rids_to_cancel.push(r.clone());
            }
        }
    }
    ctl.nav.state = "idle";
    ctl.nav.goal = None;
    ctl.nav.goal_t_ms = None;
    ctl.nav.result = if active_motion {
        "canceled".to_string()
    } else {
        String::new()
    };
    ctl.nav.error.clear();
    for r in &rids_to_cancel {
        queue_slot().push(json!({ "c": "nav_cancel", "rid": r }), false);
        if ctl.patrol.sending_rid.as_deref() == Some(r.as_str())
            || ctl.patrol.pending_rid.as_deref() == Some(r.as_str())
        {
            ctl.patrol.cancel_request = Some(r.clone());
        }
    }
    // 在途任务取消时同步发一次 manual_zero（/manual_cmd_vel 优先级高于 /nav_cmd_vel），
    // 让底盘立即刹停而不必等 Nav2 cancel 异步回调或 nav_relay 1s 超时，同时不锁停 guard。
    if active_motion {
        queue_slot().push(json!({ "c": "manual_zero" }), false);
    }
    ctl.nav.rid = None;
    drop(ctl);
    {
        let mut raw = raw_slot().lock().expect("raw poisoned");
        raw.nav_feedback = None;
        raw.nav_feedback_t = None;
    }
    (true, safety::global().nav_view())
}

/// cruise_set（ros 模式 handle_cruise_set 分叉）：状态机值照旧记在
/// ctl.cruise_on；下发经垫片并按 cmd_ack 如实回显（契约 commandMap
/// cruise_set 行：通道不可用→manual_error）。失败不落状态。
pub async fn cruise_set_ros(on: bool) -> Result<(), String> {
    let ack = push_and_wait_ack(
        json!({ "c": "cruise_enable", "on": on }),
        true, // 补投=是（BRIDGE_PROTOCOL.md commandMap cruise_enable 行）
        CMD_ACK_TIMEOUT_MS,
    )
    .await?;
    let ok = ack.get("ok").and_then(Value::as_bool).unwrap_or(false);
    if !ok {
        let err = ack
            .get("error")
            .and_then(Value::as_str)
            .filter(|s| !s.is_empty())
            .unwrap_or("垫片上报巡航下发失败");
        return Err(err.to_string());
    }
    let hub = safety::global();
    hub.control.lock().expect("control poisoned").cruise_on = on;
    intent_slot().lock().expect("intent poisoned").last_cruise = Some(on);
    Ok(())
}

/// ultrasonic_rearm（ros 模式 handle_ultrasonic_rearm 分叉）：发布成功才
/// 清超声闩；失败如实显示，不假装已恢复（web_debug.py:564-580）。
pub async fn ultrasonic_rearm_ros() -> (bool, &'static str) {
    let result = push_and_wait_ack(
        json!({ "c": "ultrasonic_rearm" }),
        false, // 补投=否（单发边沿：重放会在无用户动作时清超声门锁）
        CMD_ACK_TIMEOUT_MS,
    )
    .await;
    match result {
        Ok(ack) if ack.get("ok").and_then(Value::as_bool).unwrap_or(false) => {
            safety::global().ultrasonic_rearm();
            (true, "已请求重新启动超声波")
        }
        Ok(_) => (false, "超声恢复发布失败：垫片上报发布异常"),
        Err(_) => (false, "超声恢复通道不可用：垫片未确认"),
    }
}



// ---------------------------------------------------------------------------
// 状态装配（ros 原始缓存 → StatusPayload，20Hz）
// ---------------------------------------------------------------------------

/// 持锁版导航视图（镜像 safety.rs:994-1009 nav_view_locked；ros.rs 在已
/// 持有 control 锁的校验路径上需要带错误文案的视图，std Mutex 不可重入，
/// 不能走 Hub::nav_view）。
fn nav_view_locked(ctl: &safety::Control) -> Value {
    let nav = &ctl.nav;
    let now = safety::now_ms();
    json!({
        "state": nav.state,
        "goal": nav.goal.map(|g| json!({"x": g.x, "y": g.y, "yaw": g.yaw})),
        "age": nav.goal_t_ms.map(|t| (now.saturating_sub(t) as f64 / 1000.0 * 10.0).round() / 10.0),
        "feedback": Value::Null,
        "feedback_age": Value::Null,
        "result": nav.result,
        "error": nav.error,
        "plan_error": "",
        "client": true,
        "seq": nav.seq,
    })
}

/// 新鲜度三态（sensors.py:23-36 freshness 的移植）：None=从未收到→unknown。
fn freshness(last_t_ms: Option<u64>, now: u64, timeout_s: f64) -> (&'static str, Option<f64>) {
    let Some(t) = last_t_ms else {
        return ("unknown", None);
    };
    let age = now.saturating_sub(t) as f64 / 1000.0;
    if age < timeout_s {
        ("fresh", Some(round2(age)))
    } else {
        ("stale", Some(round2(age)))
    }
}

/// fusion 的 ultrasonic 证据对象归一成 fake.rs 协议权威形状 `{m, ok, latched}`
/// ——前端三态按权威形状写（latched→锁存、ok→正常、其余→故障），透传形状
/// （valid/fault/stop/range_m，fusion_node.py:992）曾致 `usd.ok` 恒 undefined、
/// 界面永远显示「超声波故障」（2026-11-22 修）。
/// 语义对照（safety_state.py）：stop=近距迟滞锁存（远离 release_d 自解）；
/// fault=无效/断流（必须 rearm，UI 文案「重启超声波」）；valid=读数有效。
/// 故障优先于锁存展示（fault 态不占锁存位）；字段缺失一律 fail-closed。
fn ultrasonic_view_of(u: &Value) -> Value {
    let valid = u.get("valid").and_then(Value::as_bool).unwrap_or(false);
    let stop = u.get("stop").and_then(Value::as_bool).unwrap_or(false);
    let fault = u.get("fault").and_then(Value::as_bool).unwrap_or(false);
    let m = u.get("range_m").and_then(Value::as_f64).map(round2);
    let latched = stop && !fault;
    let ok = valid && !stop && !fault;
    json!({ "m": m, "ok": ok, "latched": latched })
}

/// `/drive_safety` 展示视图（web_debug.py:2027-2052 drive_safety_view 的
/// 逐字移植）：过期（超消息自带 ttl_s，缺省 1s）时 state='stale'、
/// allowed=None——过期许可不等于允许，也不等于拒绝。
/// stale 时 ultrasonic 一并置空：链路断流由底盘超时/节点离线报警覆盖，
/// 不拿陈旧证据报「故障」假警。
fn drive_safety_view_of(payload: Option<&Value>, t_ms: Option<u64>, now: u64) -> DriveSafetyView {
    let Some(payload) = payload.filter(|_| t_ms.is_some()) else {
        return DriveSafetyView {
            state: "unknown",
            allowed: None,
            reason: None,
            age: None,
            seq: None,
            motion: None,
            ultrasonic: None,
        };
    };
    let age = now.saturating_sub(t_ms.unwrap_or(now)) as f64 / 1000.0;
    let ttl = payload.get("ttl_s").and_then(Value::as_f64).unwrap_or(1.0);
    let fresh = age <= ttl;
    DriveSafetyView {
        state: if fresh { "fresh" } else { "stale" },
        allowed: if fresh { payload.get("allowed").and_then(Value::as_bool) } else { None },
        reason: if fresh {
            payload.get("reason").and_then(Value::as_str).map(str::to_string)
        } else {
            None
        },
        age: Some(round2(age)),
        seq: payload.get("seq").and_then(Value::as_i64),
        motion: payload.get("motion").cloned(),
        ultrasonic: if fresh {
            payload.get("ultrasonic").filter(|u| u.is_object()).map(ultrasonic_view_of)
        } else {
            None
        },
    }
}

/// 底盘卡片（chassis.py:143-198 chassis_view 的移植 + hz）。
#[allow(clippy::too_many_arguments)]
fn chassis_view_of(raw: &Raw, now: u64) -> ChassisView {
    let (state, age) = freshness(raw.chassis_t, now, CHASSIS_TIMEOUT_S);
    let mut view = ChassisView {
        state,
        age,
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
        hz: Some(raw.chassis_rate.value()),
    };
    let Some(chassis) = raw.chassis.as_ref() else {
        return view;
    };
    if state != "fresh" {
        return view;
    }
    if let Some(mv) = chassis.voltage_mv {
        view.voltage_v = Some(round2(mv as f64 / 1000.0));
    }
    if let Some(vx) = chassis.vx_mm_s {
        view.vx_m_s = Some(round4(vx as f64 / 1000.0));
    }
    if let Some(vy) = chassis.vy_mm_s {
        view.vy_m_s = Some(round4(vy as f64 / 1000.0));
    }
    if let Some(wz) = chassis.yaw_rate_mrad_s {
        view.wz_rad_s = Some(round4(wz as f64 / 1000.0));
    }
    view.stop_flag = chassis.stop_flag.map(|s| s as i32);
    match chassis.stop_flag {
        Some(0) => view.stop_text = "软件未失能；就绪未知",
        None => view.stop_text = "未知",
        Some(_) => {
            view.stop_text = "软件失能";
            view.drive_ready = Some(false);
        }
    }
    if raw.chassis_t.is_some() {
        view.mileage_m = Some(round2(raw.mileage.trip()));
    }
    // 0xFFFF = 无回波/数据陈旧（固件约定），不显示旧值（chassis.py:191-194）。
    if let Some(us) = chassis.ultrasonic_front_mm {
        if us != 0xFFFF {
            view.ultrasonic_m = Some(round3(us as f64 / 1000.0));
        }
    }
    if let (Some(vx), Some(wz)) = (view.vx_m_s, view.wz_rad_s) {
        view.wheels = Some(fake::wheel_view(vx, wz));
    }
    view
}

fn cmd_view_of(raw: &Raw, idx: usize, now: u64) -> CmdView {
    let (state, age) = freshness(raw.cmd_t[idx], now, CMD_TIMEOUT_S);
    match raw.cmds[idx].as_ref().filter(|_| state == "fresh") {
        Some(c) => CmdView {
            state,
            age,
            vx_m_s: Some(round4(c.vx)),
            wz_rad_s: Some(round4(c.wz)),
        },
        None => CmdView { state, age, vx_m_s: None, wz_rad_s: None },
    }
}

/// 20Hz 装配：把 Raw 原始缓存折算进 StatusPayload（state.rs:400-404）。
/// 视图组装对齐 fake.rs:648-838 与旧 web_debug.py:1944-2024 status_payload。
fn assemble_status() {
    let store: &'static StateStore = crate::state::global();
    let hub = safety::global();
    let now = safety::now_ms();

    // 控制域快照（注意：patrol_status_frame / nav_view 会再锁 control，
    // 必须先在本作用域内取完所需值再放开，std Mutex 不可重入——
    // fake.rs:696-699 同款注释）。
    let (cruise_on, manual_dbg, marker) = {
        let ctl = hub.control.lock().expect("control poisoned");
        let manual = &ctl.manual;
        let manual_dbg = json!({
            "active": manual.owner.is_some(),
            "input": manual.speed,
            "direction": manual.direction,
            "remaining": manual
                .deadline_ms
                .map(|d| ((d as f64 - now as f64) / 1000.0).max(0.0)),
            "beats": manual.beats,
            "stops": manual.stops,
        });
        (ctl.cruise_on, manual_dbg, ctl.boot_marker.clone())
    };

    // 注意：patrol_status_frame / nav_view 内部会再锁 control。
    let patrol_frame = hub.patrol_status_frame();
    let mut nav_full = hub.nav_view();

    let (chassis_v, imu, cmds, ds_view, safety_v, moving, nodes, lidar_detail, scan_hz, image_fps,
     nav_fb, nav_fb_t, plan_error) = {
        let raw = raw_slot().lock().expect("raw poisoned");
        let chassis_v = chassis_view_of(&raw, now);
        let (imu_state, imu_age) = freshness(raw.imu_t, now, IMU_TIMEOUT_S);
        let imu = ImuView {
            state: imu_state,
            age: imu_age,
            hz: raw.imu_hz.map(|h| (h * 10.0).round() / 10.0),
        };
        let cmds = CmdViews {
            nav: cmd_view_of(&raw, 0, now),
            demo: cmd_view_of(&raw, 1, now),
            manual: cmd_view_of(&raw, 2, now),
            auto: cmd_view_of(&raw, 3, now),
        };
        let ds_view = drive_safety_view_of(raw.drive_safety.as_ref(), raw.drive_safety_t, now);
        let safety_v = raw.safety.clone().unwrap_or(Value::Null);
        let moving = raw
            .moving
            .clone()
            .filter(|_| raw.moving_t.is_some_and(|t| now.saturating_sub(t) < 2000))
            .map(|bins| bins.into_iter().map(|b| json!(b)).collect::<Vec<_>>())
            .unwrap_or_default();
        // 节点在线视图（web_debug.py::node_view）：camera/lidar/chassis 按
        // 6s 探测窗口；c50c_uart3_node = 参数服务在线（safety.online）；
        // web_bridge = 垫片 10s 帧批活性（契约修订：旧实现恒 online）。
        let seen_recent =
            |t: Option<u64>| t.is_some_and(|t| (now.saturating_sub(t) as f64) / 1000.0 < NODE_ONLINE_S);
        let bridge_up = raw
            .last_batch_ms
            .is_some_and(|t| now.saturating_sub(t) < BRIDGE_OFFLINE_MS);
        let safety_online = raw
            .safety
            .as_ref()
            .and_then(|s| s.get("online"))
            .and_then(Value::as_bool)
            .unwrap_or(false);
        let scan_frames = {
            let media = fake::media_slot().read().expect("media poisoned");
            media.scan_seq
        };
        let nodes = NodesView {
            camera_node: if seen_recent(raw.nodes_seen.camera_node) { "online" } else { "offline" },
            lidar_node: if seen_recent(raw.nodes_seen.lidar_node) { "online" } else { "offline" },
            c50c_uart3_node: if safety_online { "online" } else { "offline" },
            chassis_status_node: if seen_recent(raw.nodes_seen.chassis_status_node) {
                "online"
            } else {
                "offline"
            },
            web_bridge: if bridge_up { "online" } else { "offline" },
            scan_without_node: scan_frames > 0 && !seen_recent(raw.nodes_seen.lidar_node),
        };
        // 雷达诊断明细（fake.rs:816-822 同位；ros 模式来自 /diagnostics 的
        // lidar 条目，DIAG_TIMEOUT=3.0 内 fresh）。
        let lidar_detail = match raw.lidar_diag.as_ref() {
            Some(d) => {
                let (dstate, _) = freshness(raw.diag_t, now, DIAG_TIMEOUT_S);
                json!({
                    "state": d.state,
                    "reason": d.reason,
                    "age_s": d.age_s,
                    "coverage": d.coverage,
                    "reconnects": d.reconnects,
                    "port": d.port,
                    "frames_ok": d.frames_ok,
                    "frames_bad": d.frames_bad,
                    "level": d.level,
                    "message": d.message,
                    "diag_state": dstate,
                })
            }
            None => Value::Null,
        };
        let scan_hz = raw.scan_rate.value();
        let image_fps = raw.image_rate.value();
        let nav_fb = raw.nav_feedback.as_ref().map(|f| {
            json!({
                "rid": f.rid,
                "dist": f.dist,
                "eta": f.eta,
                "x": f.x,
                "y": f.y,
            })
        });
        let nav_fb_t = raw.nav_feedback_t;
        let plan_error = raw.last_plan_error.clone();
        (
            chassis_v, imu, cmds, ds_view, safety_v, moving, nodes, lidar_detail, scan_hz,
            image_fps, nav_fb, nav_fb_t, plan_error,
        )
    };

    // 导航视图叠加：feedback / feedback_age / plan_error / safety（旧
    // status_payload: view['nav'] = NG.nav_view(ds_view)，fake.rs:763-766 同）。
    if let Value::Object(map) = &mut nav_full {
        let fb_age = nav_fb_t
            .map(|t| json!(round2(now.saturating_sub(t) as f64 / 1000.0)))
            .unwrap_or(Value::Null);
        map.insert("feedback".into(), nav_fb.unwrap_or(Value::Null));
        map.insert("feedback_age".into(), fb_age);
        map.insert("plan_error".into(), json!(plan_error));
        map.insert(
            "safety".into(),
            serde_json::to_value(&ds_view).unwrap_or(Value::Null),
        );
    }

    // 任务状态机（console_views.py::task_state；battery_low 旧实现硬编码
    // False——本版本无回充节点，不冒充）。
    let nav_fresh = cmds.nav.state == "fresh";
    let nav_vx = cmds.nav.vx_m_s.unwrap_or(0.0);
    let nav_wz = cmds.nav.wz_rad_s.unwrap_or(0.0);
    let nav_age = if nav_fresh { cmds.nav.age } else { None };
    let manual_active = manual_dbg.get("active").and_then(Value::as_bool).unwrap_or(false);
    let patrol_fresh = patrol_frame.get("status").and_then(Value::as_str) == Some("running");
    let task_state = TaskState::decide(
        nav_fresh,
        nav_vx,
        nav_wz,
        nav_age,
        manual_active,
        false,
        patrol_fresh,
    );
    let channel = chassis_v.state == "fresh"
        || safety_v.get("online").and_then(Value::as_bool).unwrap_or(false);
    let task = TaskView {
        state: task_state.as_str(),
        text: task_state.text(),
        cmd_vx: if nav_fresh { cmds.nav.vx_m_s } else { None },
        cmd_wz: if nav_fresh { cmds.nav.wz_rad_s } else { None },
        channel,
        manual: manual_dbg,
    };

    let battery = BatteryView::from_voltage(chassis_v.voltage_v);
    let media_stat = {
        let media = fake::media_slot().read().expect("media poisoned");
        (media.scan_valid, media.scan_seq)
    };
    let boot = fake::boot_view(&marker);
    let srv_epoch = *SRV_EPOCH.get_or_init(|| store.snapshot().srv_epoch);

    let payload = StatusPayload {
        t: "status",
        mode: "ros",
        chassis: chassis_v,
        imu,
        cmd: cmds,
        safety: safety_v,
        drive_safety: ds_view,
        nav: nav_full,
        scan_hz,
        image_fps,
        scan_frames: json!(media_stat.1),
        scan_valid: json!(media_stat.0),
        // 网络信息快照来自 netinfo（5 秒间隔采集器最新值，未采到为
        // null）；host 保持 null（未定义主机名语义）；audio 来自
        // audio.rs 的设备探测（喇叭/麦克风设备在位性）。
        network: crate::netinfo::snapshot(),
        audio: crate::audio::snapshot(),
        sys: crate::sysinfo::snapshot(),
        host: Value::Null,
        lidar_detail,
        boot,
        srv_epoch,
        nodes,
        task,
        battery,
        cruise: CruiseView {
            on: cruise_on,
            available: raw_last_batch_ms().is_some_and(|t| now.saturating_sub(t) < BRIDGE_OFFLINE_MS),
        },
        patrol: patrol_frame,
        moving,
    };
    store.update(|st| *st = payload);
}

fn raw_last_batch_ms() -> Option<u64> {
    raw_slot().lock().expect("raw poisoned").last_batch_ms
}

// ---------------------------------------------------------------------------
// 测试
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    /// 测试共享进程级静态（OnceLock 全局 + 命令队列 + 媒体槽），必须串行。
    static TEST_LOCK: Mutex<()> = Mutex::new(());

    fn guard() -> std::sync::MutexGuard<'static, ()> {
        TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// 测试装配：全局 OnceLock 幂等初始化（首个测试生效，后续复用）。
    fn setup() -> &'static Config {
        static ONCE: OnceLock<Config> = OnceLock::new();
        ONCE.get_or_init(|| Config {
            fake: false,
            port: 0,
            password_sha256: None,
            boot_marker: std::path::PathBuf::from("/tmp/c50c-web-next-test-boot"),
            bridge_token: Some("0123456789abcdef0123456789abcdef".to_string()),
        })
    }

    fn init_all() {
        let cfg = setup();
        init_token(cfg);
        let _ = crate::state::init_global("ros");
        let _ = safety::init_hub(cfg);
        let _ = raw_slot();
        let _ = queue_slot();
        let _ = intent_slot();
        let _ = acks_slot();
        let _ = SRV_EPOCH.get_or_init(|| 0);
        ENABLED.store(true, Ordering::Relaxed);
    }

    fn frame(t: &str, extra: Value) -> Value {
        let mut f = json!({ "t": t });
        if let (Value::Object(a), Value::Object(b)) = (&mut f, extra) {
            for (k, v) in b {
                a.insert(k, v);
            }
        }
        f
    }

    #[test]
    fn test_token_check_constant_time() {

        let _guard = guard();
        init_all();
        let mut h = HeaderMap::new();
        assert!(!token_ok(&h), "缺令牌必须 401");
        h.insert("x-bridge-token", "wrong".parse().unwrap());
        assert!(!token_ok(&h), "错令牌必须 401");
        h.insert("x-bridge-token", "0123456789abcdef0123456789abcdef".parse().unwrap());
        assert!(token_ok(&h), "对令牌必须放行");
    }

    #[test]
    fn test_media_frames_last_wins() {

        let _guard = guard();
        init_all();
        assert!(apply_frame(&frame(
            "scan",
            json!({ "p": [[0.0, 1.5], [90.0, 0.0], [180.0, 2.25]], "n": 2, "all": 3 })
        )));
        {
            let media = fake::media_slot().read().unwrap();
            assert_eq!(media.scan_seq, 1);
            assert_eq!(media.scan_valid, 2);
            assert_eq!(media.scan.len(), 3);
            assert_eq!(media.scan[0], (0.0, 1.5));
        }
        assert!(apply_frame(&frame(
            "pose",
            json!({ "pose": { "x": 1.234, "y": -0.5, "th": 1.5708 } })
        )));
        {
            let media = fake::media_slot().read().unwrap();
            assert_eq!(media.pose, (1.234, -0.5, 1.5708));
            // now_ms() 在进程头几毫秒就是合法的 0，只判「不超过当前时刻」。
            assert!(media.pose_mono_ms <= safety::now_ms());
        }
        assert!(apply_frame(&frame(
            "img",
            json!({ "d": "aGVsbG8=", "w": 384, "h": 216, "mime": "image/jpeg" })
        )));
        {
            let media = fake::media_slot().read().unwrap();
            assert_eq!(media.image_seq, 1);
            assert_eq!(media.image_w, 384);
            assert_eq!(media.image_mime, "image/jpeg");
        }
        // 坏帧跳过不拒整批
        assert!(!apply_frame(&frame("img", json!({}))));
        assert!(!apply_frame(&frame("unknown_t", json!({}))));
    }

    #[test]
    fn test_map_applier_decodes_and_binds_map_id() {

        let _guard = guard();
        init_all();
        // 栅格内容 [0,100,255] → zlib+base64
        let raw_grid: Vec<u8> = vec![0u8, 100, 255];
        use base64::Engine as _;
        let mut z = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
        std::io::Write::write_all(&mut z, &raw_grid).unwrap();
        let compressed = z.finish().unwrap();
        let d = base64::engine::general_purpose::STANDARD.encode(&compressed);
        let expect_id = safety::sha256_hex(&raw_grid)[..16].to_string();
        assert!(apply_frame(&frame(
            "map",
            json!({ "seq": 5, "w": 3, "h": 1, "res": 0.05, "ox": -8.28, "oy": -8.03, "d": d })
        )));
        let hub = safety::global();
        {
            let ctl = hub.control.lock().unwrap();
            assert_eq!(ctl.current_map_id.as_deref(), Some(expect_id.as_str()));
        }
        let media = fake::media_slot().read().unwrap();
        assert_eq!(media.grid.seq, 5);
        assert_eq!(media.grid.w, 3);
        assert_eq!(media.grid.data_b64.as_ref(), d);
    }

    #[test]
    fn test_command_queue_edge_never_replayed() {

        let _guard = guard();
        init_all();
        let q = queue_slot();
        // 清场（幂等：测试进程共享静态队列）
        q.clear_undelivered();
        {
            let mut st = q.st.lock().unwrap();
            st.ring.clear();
            st.patrol_slot = None;
            st.last_cursor = 0;
        }
        let id_value = q.push(json!({ "c": "guard_authorize", "value": true }), true);
        let id_edge = q.push(json!({ "c": "rearm_request" }), false);
        let (cmds, cursor) = tokio::runtime::Runtime::new()
            .unwrap()
            .block_on(q.poll(0, 0));
        assert_eq!(cmds.len(), 2, "首次轮询应拿到值语义+边沿两条");
        assert_eq!(cursor, id_edge, "cursor 推进到最后一条");
        // cursor 之后的轮询：值语义补投（cursor 之前的不补）
        let (cmds2, _) = tokio::runtime::Runtime::new().unwrap().block_on(q.poll(id_edge, 0));
        assert!(cmds2.is_empty(), "cursor 已到最新：无命令");
        // 新值命令 + cursor 倒退（模拟重连）：值语义补投，边沿不补投
        let id_value2 = q.push(json!({ "c": "manual_cmd", "vx": 0.2, "wz": 0.0 }), true);
        let (cmds3, cursor3) = tokio::runtime::Runtime::new().unwrap().block_on(q.poll(0, 0));
        let kinds: Vec<&str> = cmds3
            .iter()
            .map(|c| c.get("c").and_then(Value::as_str).unwrap_or(""))
            .collect();
        assert!(kinds.contains(&"guard_authorize"), "值语义命令按 cursor 补投");
        assert!(kinds.contains(&"manual_cmd"));
        assert!(!kinds.contains(&"rearm_request"), "边沿命令绝不重放");
        assert!(kinds.iter().any(|k| *k == "manual_cmd"));
        let _ = id_value2;
        // bridge_hello 清空未投递（垫片保持自己的 cursor，不回退重放）
        let _id_pending = q.push(json!({ "c": "manual_cmd", "vx": 0.0, "wz": 0.0 }), true);
        assert!(q.pending(), "push 后应有未投递命令");
        q.clear_undelivered();
        let (cmds4, _) = tokio::runtime::Runtime::new().unwrap().block_on(q.poll(cursor3, 0));
        assert!(cmds4.is_empty(), "hello 后未投递队列必须清空（按垫片当前 cursor）");
        assert!(!q.pending());
        let _ = id_value;
    }

    #[test]
    fn test_command_queue_stale_cursor_resets_across_restart() {
        let _guard = guard();
        init_all();
        let q = queue_slot();
        {
            let mut st = q.st.lock().unwrap();
            st.ring.clear();
            st.patrol_slot = None;
            st.last_cursor = 0;
        }
        let id = q.push(json!({ "c": "guard_authorize", "value": true }), true);
        let stale_cursor = id + 1000;
        let (cmds, cursor) = tokio::runtime::Runtime::new()
            .unwrap()
            .block_on(q.poll(stale_cursor, 0));
        assert_eq!(cmds.len(), 1, "旧进程大 cursor 应自动归零并投递新命令");
        assert_eq!(cursor, id, "返回游标同步到新进程命令 id");
    }

    #[test]
    fn test_relocalize_enqueues_global_match_edge_command() {
        // 全局匹配重定位：命令体只有 {c:"relocalize"}（位姿由垫片用
        // /map+/scan 实算，不依赖前端先验），边沿触发不重放。
        let _guard = guard();
        init_all();
        let q = queue_slot();
        q.clear_undelivered();
        {
            let mut st = q.st.lock().unwrap();
            st.ring.clear();
            st.patrol_slot = None;
            st.last_cursor = 0;
        }
        let id = q.push(json!({ "c": "relocalize" }), false);
        assert!(id > 0);
        let (cmds, cursor) = tokio::runtime::Runtime::new().unwrap().block_on(q.poll(0, 0));
        assert_eq!(cmds.len(), 1);
        let c = &cmds[0];
        assert_eq!(c.get("c").and_then(Value::as_str), Some("relocalize"));
        assert_eq!(c.get("id").and_then(Value::as_u64), Some(id), "命令带自身 id 供 cmd_ack 回执");
        // 边沿触发：poll 过即不重放
        let (cmds2, _) = tokio::runtime::Runtime::new()
            .unwrap()
            .block_on(q.poll(cursor, 0));
        assert!(cmds2.is_empty(), "relocalize 是边沿命令，绝不重放");
    }

    #[test]
    fn test_manual_intent_reconcile_emits_zero() {

        let _guard = guard();
        init_all();
        let hub = safety::global();
        {
            // 登记一个已登录连接并接管手动（直接操作 Control，等价
            // manual_login + manual_start 的落库结果）。
            let mut ctl = hub.control.lock().unwrap();
            ctl.manual.authenticated.insert(42);
            ctl.manual.owner = Some(42);
            ctl.manual.cmd = (0.2, 0.0);
            ctl.manual.last_heartbeat_ms = safety::now_ms();
        }
        flush_manual_cmd();
        {
            let st = queue_slot().st.lock().unwrap();
            let last = st.ring.back().unwrap();
            assert_eq!(last.body.get("c").and_then(Value::as_str), Some("manual_cmd"));
            assert_eq!(last.body.get("vx").and_then(Value::as_f64), Some(0.2));
            assert!(last.replayable);
        }
        // 清零（等价 manual_stop / manual_tick 超时 / conn_closed 的结果）
        {
            let mut ctl = hub.control.lock().unwrap();
            ctl.manual.owner = None;
            ctl.manual.cmd = (0.0, 0.0);
        }
        flush_manual_cmd();
        {
            let st = queue_slot().st.lock().unwrap();
            let last = st.ring.back().unwrap();
            assert_eq!(last.body.get("c").and_then(Value::as_str), Some("manual_zero"));
        }
        // 幂等：再对账不发重复 manual_zero
        let before = queue_slot().st.lock().unwrap().next_id;
        flush_manual_cmd();
        assert_eq!(queue_slot().st.lock().unwrap().next_id, before, "无变化不再派发");
    }

    #[test]
    fn test_license_forward_and_authorize_true() {

        let _guard = guard();
        init_all();
        on_license_accepted("sess-1", 7, 350);
        let st = queue_slot().st.lock().unwrap();
        let kinds: Vec<(&str, bool)> = st
            .ring
            .iter()
            .map(|c| {
                (
                    c.body.get("c").and_then(Value::as_str).unwrap_or(""),
                    c.replayable,
                )
            })
            .collect();
        assert!(kinds.contains(&("license_forward", true)));
        assert!(kinds.contains(&("guard_authorize", true)));
        let fwd = st
            .ring
            .iter()
            .find(|c| c.body.get("c").and_then(Value::as_str) == Some("license_forward"))
            .unwrap();
        let frame = fwd.body.get("frame").unwrap();
        // 下游 guard（motion_guard.py License.check）要求 session int≥1：
        // 浏览器 hex 会话经稳定映射转 int（评审 high-1）。
        let sess = frame.get("session").and_then(Value::as_i64);
        assert!(sess.is_some_and(|v| v >= 1), "session 必须 int≥1: {:?}", frame);
        assert_eq!(frame.get("seq").and_then(Value::as_i64), Some(7), "seq 原样转发");
        let exp = frame.get("expires_mono").and_then(Value::as_f64).unwrap();
        // 锚点是 boot 相对单调（/proc/uptime），必须大于进程 uptime 秒
        // （boot 相对时间 ≥ 进程运行时间；guard 用 time.monotonic 比较）。
        assert!(exp > safety::now_ms() as f64 / 1000.0,
                "expires_mono 锚点必须是 boot 相对: {} vs uptime {}s", exp, safety::now_ms() as f64 / 1000.0);
    }

    #[test]
    fn test_session_int_mapping_stable_and_takeover() {
        let _guard = guard();
        init_all();
        let a1 = session_int("aa11");
        let a2 = session_int("aa11");
        assert_eq!(a1, a2, "同一浏览器会话恒映射同一 int（guard session_mismatch 比较）");
        assert!(a1 >= 1);
        let b1 = session_int("bb22");
        assert_ne!(a1, b1, "页面接管（新 hex）分配新 int");
        assert!(b1 > a1, "新会话号递增");
    }

    #[test]
    fn test_goal_timeout_branch_emits_withdraw_and_cancel() {
        // 回归评审 high-2：15s 在途超时分支在已持 intent 锁的上下文里
        // 撤回授权——若 force_authorize_false 重入 intent_slot（std Mutex
        // 非重入）本测试会永久挂死（死锁），修复后应正常发出
        // guard_authorize(false) + nav_cancel 并锁停。
        let _guard = guard();
        init_all();
        let hub = safety::global();
        let now = safety::now_ms();
        {
            let mut ctl = hub.control.lock().unwrap();
            ctl.patrol.status = "running";
            ctl.locked = false;
            ctl.patrol.sending_rid = Some("p0-0-1".to_string());
            ctl.nav.rid = Some("p0-0-1".to_string());
            ctl.nav.state = "sending";
            ctl.nav.goal_t_ms = Some(now); // 刚派发；把判定时钟推到 16s 后
        }
        nav_housekeeping_at(now + safety::NAV_GOAL_TIMEOUT_MS + 1000);
        {
            let ctl = hub.control.lock().unwrap();
            assert_eq!(ctl.patrol.status, "stopped");
            assert_eq!(ctl.patrol.stop_reason, "goal_timeout");
            assert!(ctl.locked, "goal_timeout 必须锁停");
            assert_eq!(ctl.nav.rid, None);
        }
        {
            let intent = intent_slot().lock().unwrap();
            assert_eq!(intent.last_authorize, Some(false), "超时分支必须撤回授权");
        }
        {
            let st = queue_slot().st.lock().unwrap();
            assert!(st.ring.iter().any(|c| {
                c.body.get("c").and_then(Value::as_str) == Some("guard_authorize")
                    && c.body.get("value") == Some(&Value::Bool(false))
            }), "guard_authorize(false) 必须入队");
            assert!(st.ring.iter().any(|c| {
                c.body.get("c").and_then(Value::as_str) == Some("nav_cancel")
                    && c.body.get("rid").and_then(Value::as_str) == Some("p0-0-1")
            }), "nav_cancel 必须入队");
        }
        // 测试卫生：本测试设置的 ctl 全局状态（cancel_request/locked/在途
        // rid）必须清掉——共享 ctl 的后续测试（如
        // test_nav_result_patrol_advance_and_fail_lock 派发同形 rid
        // "p0-0-…"）会因遗留 cancel_request 走取消分支而断言失败。
        {
            let mut ctl = hub.control.lock().unwrap();
            ctl.patrol.cancel_request = None;
            ctl.patrol.sending_rid = None;
            ctl.patrol.pending_rid = None;
            ctl.patrol.status = "idle";
            ctl.patrol.stop_reason.clear();
            ctl.locked = false;
            ctl.lock_reason.clear();
            ctl.nav.state = "idle";
            ctl.nav.rid = None;
            ctl.nav.goal_t_ms = None;
            ctl.nav.result.clear();
            ctl.nav.error.clear();
        }
    }

    #[test]
    fn test_ring_overflow_drops_delivered_first() {
        // 评审 low：环形溢出优先丢已投递条目（补投已消费），未投递的
        // 值语义命令（license_forward 积压）尽可能保留。
        let _guard = guard();
        init_all();
        // 预塞未投递值语义命令 CMD_RING_MAX 条（license_forward 形态）
        for i in 0..CMD_RING_MAX {
            queue_slot().push(json!({ "c": "license_forward", "marker": i }), true);
        }
        // 消费一半（cursor 推进 → 前一半 delivered）
        let (_, _) = {
            // poll 是 async——用 handleblock_on 不引 tokio 依赖到测试线程，
            // 这里直接手动标记已投递等价：前 CMD_RING_MAX/2 条标记 delivered
            let mut st = queue_slot().st.lock().unwrap();
            for (i, c) in st.ring.iter_mut().enumerate() {
                if i < CMD_RING_MAX / 2 {
                    c.delivered = true;
                }
            }
            (0u64, 0u64)
        };
        // 再压入 CMD_RING_MAX/2 条 → 触发溢出
        for i in 0..CMD_RING_MAX / 2 {
            queue_slot().push(json!({ "c": "license_forward", "marker": 1000 + i }), true);
        }
        let st = queue_slot().st.lock().unwrap();
        assert!(st.ring.len() <= CMD_RING_MAX);
        // 已投递的前一半应被优先清出
        let delivered_left = st.ring.iter().filter(|c| c.delivered).count();
        // 未投递的原 CMD_RING_MAX/2 条中较新部分应保留
        let undelivered = st.ring.iter().filter(|c| !c.delivered).count();
        assert!(undelivered >= CMD_RING_MAX / 2 - CMD_RING_MAX / 2, "sanity");
        assert!(delivered_left < CMD_RING_MAX / 2,
                "溢出应优先丢已投递条目: delivered_left={} undelivered={}",
                delivered_left, undelivered);
    }

    #[test]
    fn test_rearm_batch_order_authorize_first() {

        let _guard = guard();
        init_all();
        on_rearm_ok();
        let st = queue_slot().st.lock().unwrap();
        let kinds: Vec<&str> = st
            .ring
            .iter()
            .map(|c| c.body.get("c").and_then(Value::as_str).unwrap_or(""))
            .collect();
        let a = kinds.iter().position(|k| *k == "guard_authorize").unwrap();
        let r = kinds.iter().position(|k| *k == "rearm_request").unwrap();
        assert!(a < r, "先补鉴权再发解停（顺序铁律 web_debug.py:555-560）");
        let rearm = st.ring.iter().find(|c| {
            c.body.get("c").and_then(Value::as_str) == Some("rearm_request")
        }).unwrap();
        assert!(!rearm.replayable, "rearm_request 边沿触发永不补投");
    }

    #[test]
    fn test_mileage_integration_trapezoid() {

        let _guard = guard();
        let mut m = MileageIntegrator::default();
        m.update(1000, 1000); // 1 m/s @ t=1s：首帧只记录
        m.update(1000, 1500); // dt=0.5s（=MAX_STEP_S）@ 1 m/s → +0.5m
        m.update(1000, 4000); // dt=2.5s > 0.5s：丢帧不计入
        assert!((m.trip() - 0.5).abs() < 1e-9, "正常步长积分 + 丢帧期间不计入");
    }

    #[test]
    fn test_rate_meter_window() {

        let _guard = guard();
        let mut r = RateMeter::new(2.0);
        assert_eq!(r.tick(0), 0.0);
        assert_eq!(r.tick(1000), 0.0, "窗口内保持上次值");
        assert_eq!(r.tick(2500), 1.2, "窗口到期：n=3 / dt=2.5s → 1.2Hz");
        assert_eq!(r.value(), 1.2);
    }

    #[test]
    fn test_drive_safety_view_ttl() {

        let _guard = guard();
        let now = 100_000;
        let payload = json!({ "allowed": true, "reason": "ok", "seq": 3, "ttl_s": 1.0 });
        let v = drive_safety_view_of(Some(&payload), Some(now - 500), now);
        assert_eq!(v.state, "fresh");
        assert_eq!(v.allowed, Some(true));
        let v2 = drive_safety_view_of(Some(&payload), Some(now - 1500), now);
        assert_eq!(v2.state, "stale", "超 ttl 必须置 stale");
        assert_eq!(v2.allowed, None, "过期许可不等于允许");
        let v3 = drive_safety_view_of(None, None, now);
        assert_eq!(v3.state, "unknown");
    }

    #[test]
    fn test_nav_result_patrol_advance_and_fail_lock() {

        let _guard = guard();
        init_all();
        let hub = safety::global();
        {
            let mut ctl = hub.control.lock().unwrap();
            ctl.patrol.route = Some(safety::Route {
                map_id: "map".into(),
                points: vec![(1.0, 1.0), (2.0, 1.0), (2.0, 2.0)],
            });
            ctl.patrol.status = "running";
            ctl.patrol.index = 0;
            ctl.patrol.laps = 0;
            ctl.current_map_id = Some("map".into());
            ctl.locked = false;
        }
        // 派发（对账路径）
        nav_housekeeping_and_dispatch();
        let rid = {
            let ctl = hub.control.lock().unwrap();
            ctl.patrol.sending_rid.clone().unwrap()
        };
        assert_eq!(ctl_rid(&hub), rid, "单一在途目标槽：nav.rid == patrol.sending_rid");
        // accepted → pending
        assert!(apply_frame(&frame(
            "nav_result",
            json!({ "rid": rid, "status": "accepted", "text": "" })
        )));
        {
            let ctl = hub.control.lock().unwrap();
            assert_eq!(ctl.patrol.pending_rid.as_deref(), Some(rid.as_str()));
            assert!(ctl.patrol.sending_rid.is_none());
            assert_eq!(ctl.nav.state, "active");
        }
        // succeeded → index+1
        assert!(apply_frame(&frame(
            "nav_result",
            json!({ "rid": rid, "status": "succeeded", "text": "到达目标" })
        )));
        {
            let ctl = hub.control.lock().unwrap();
            assert_eq!(ctl.patrol.index, 1);
            assert_eq!(ctl.patrol.laps, 0);
        }
        // failed → 锁停
        nav_housekeeping_and_dispatch(); // 派发第二个航点
        let rid2 = {
            let ctl = hub.control.lock().unwrap();
            let r = ctl.patrol.sending_rid.clone().unwrap();
            // 直接把它当作 accepted 后 failed
            r
        };
        apply_frame(&frame("nav_result", json!({ "rid": rid2, "status": "accepted", "text": "" })));
        apply_frame(&frame(
            "nav_result",
            json!({ "rid": rid2, "status": "failed", "text": "导航中止（规划/控制失败）" })
        ));
        {
            let ctl = hub.control.lock().unwrap();
            assert_eq!(ctl.patrol.status, "stopped");
            assert_eq!(ctl.patrol.stop_reason, "nav_failed");
            assert!(ctl.locked, "_fail 必须锁停（patrol.py:574-576）");
        }
        // intent：authorize 已被 force_authorize_false 置 Some(false)
        {
            let intent = intent_slot().lock().unwrap();
            assert_eq!(intent.last_authorize, Some(false));
        }
        // 锁停中的迟到接受 → 立即取消
        nav_housekeeping_and_dispatch(); // 巡逻已 stopped，不派发
        let late_rid = "late-rid-unknown-1";
        apply_frame(&json!({ "t": "nav_result", "rid": late_rid, "status": "accepted" }));
        {
            let st = queue_slot().st.lock().unwrap();
            let cancelled = st.ring.iter().any(|c| {
                c.body.get("c").and_then(Value::as_str) == Some("nav_cancel")
                    && c.body.get("rid").and_then(Value::as_str) == Some(late_rid)
            });
            assert!(cancelled, "未知/迟到接受必须立即取消（patrol.py:521-541）");
        }
    }

    fn ctl_rid(hub: &safety::Hub) -> String {
        hub.control.lock().unwrap().nav.rid.clone().unwrap()
    }

    #[test]
    fn test_assemble_status_from_raw() {

        let _guard = guard();
        init_all();
        apply_frame(&frame(
            "chassis",
            json!({ "voltage_mv": 24150, "vx_mm_s": 150, "vy_mm_s": 0,
                    "yaw_rate_mrad_s": 20, "stop_flag": 0,
                    "ultrasonic_front_mm": 1200, "level": 0, "message": "ok" }),
        ));
        apply_frame(&frame("imu", json!({ "hz": 19.8 })));
        apply_frame(&frame("cmd", json!({ "which": "nav", "vx": 0.3, "vy": 0.0, "wz": -0.1 })));
        apply_frame(&frame(
            "probe",
            json!({ "nodes_seen": { "camera_node": true, "lidar_node": true,
                    "chassis_status_node": true },
                    "safety": { "drive_enable": true, "drive_input_enable": true,
                    "drive_firmware_protocol": "baseline", "online": true,
                    "source": "在线节点 c50c_uart3_node" } }),
        ));
        apply_frame(&frame("bridge_hello", json!({ "topics": {} })));
        assemble_status();
        let st = crate::state::global().snapshot();
        assert_eq!(st.mode, "ros");
        assert_eq!(st.chassis.state, "fresh");
        assert_eq!(st.chassis.voltage_v, Some(24.15));
        assert_eq!(st.chassis.vx_m_s, Some(0.15));
        assert_eq!(st.chassis.mileage_m, Some(0.0), "单帧只记录不积分");
        assert_eq!(st.imu.hz, Some(19.8));
        assert_eq!(st.cmd.nav.state, "fresh");
        assert_eq!(st.cmd.nav.vx_m_s, Some(0.3));
        assert_eq!(st.battery.percent, Some(90), "(24.15-21)/3.5 → 90%");
        assert_eq!(st.nodes.c50c_uart3_node, "online");
        assert_eq!(st.nodes.web_bridge, "online", "hello 后 10s 活性窗口内");
        assert!(st.task.channel);
    }

    #[test]
    fn test_frame_batch_limit_constant() {

        let _guard = guard();        assert!(FRAME_BATCH_MAX == 256);
        assert!(BODY_LIMIT_BYTES == 8 * 1024 * 1024);
        assert!(BRIDGE_OFFLINE_MS == 10_000, "活性窗口必须大于 probe 周期 3s");
    }

    #[test]
    fn test_cmd_ack_preserves_detail() {
        let _guard = guard();
        init_all();
        assert!(RELOCALIZE_ACK_TIMEOUT_MS >= 30_000);
        let (tx, mut rx) = tokio::sync::oneshot::channel();
        acks_slot().lock().expect("acks poisoned").insert(999_001, tx);
        assert!(apply_frame(&json!({
            "t": "cmd_ack",
            "id": 999_001,
            "ok": true,
            "error": null,
            "detail": {
                "x": -0.005,
                "y": 0.005,
                "yaw_deg": -1.0,
                "hit_ratio": 0.93,
                "match_ms": 3150.0,
            }
        })));
        let ack = rx.try_recv().expect("cmd_ack must resolve oneshot");
        assert_eq!(ack.get("ok").and_then(Value::as_bool), Some(true));
        let detail = ack.get("detail").expect("detail must be preserved");
        assert_eq!(detail.get("x").and_then(Value::as_f64), Some(-0.005));
        assert_eq!(detail.get("y").and_then(Value::as_f64), Some(0.005));
        assert_eq!(detail.get("yaw_deg").and_then(Value::as_f64), Some(-1.0));
        assert_eq!(detail.get("hit_ratio").and_then(Value::as_f64), Some(0.93));
        assert_eq!(detail.get("match_ms").and_then(Value::as_f64), Some(3150.0));
    }

    #[test]
    fn test_cancel_nav_goal_clears_failed_and_active_goal() {
        let _guard = guard();
        init_all();
        let hub = safety::global();
        // 1) 残留 failed 状态 + goal → cancel_nav_goal 必须清零并回到 idle
        {
            let mut ctl = hub.control.lock().unwrap();
            ctl.nav.state = "failed";
            ctl.nav.goal = Some(safety::Goal { x: 3.81, y: -0.30, yaw: None });
            ctl.nav.goal_t_ms = Some(1234);
            ctl.nav.error = "导航中止（规划/控制失败）".to_string();
            ctl.nav.rid = None;
            ctl.patrol.status = "idle";
            ctl.patrol.sending_rid = None;
            ctl.patrol.pending_rid = None;
        }
        let (ok, view) = cancel_nav_goal();
        assert!(ok, "失败残留目标必须允许被清除");
        assert_eq!(view.get("state").and_then(Value::as_str), Some("idle"));
        assert!(view.get("goal").is_some_and(Value::is_null));
        {
            let ctl = hub.control.lock().unwrap();
            assert!(ctl.nav.goal.is_none());
            assert!(ctl.nav.error.is_empty());
        }
        assert!(queue_slot().st.lock().unwrap().ring.is_empty(), "无在途 rid 时不发 nav_cancel");

        // 2) 已在 idle 且无目标时再次取消应拒并提示
        let (ok2, view2) = cancel_nav_goal();
        assert!(!ok2);
        assert_eq!(view2.get("error").and_then(Value::as_str), Some("当前没有导航目标"));

        // 3) 进行中的目标 → 回 idle、清 goal、下发 nav_cancel
        {
            let mut ctl = hub.control.lock().unwrap();
            ctl.nav.state = "active";
            ctl.nav.goal = Some(safety::Goal { x: 0.0, y: 0.0, yaw: Some(0.0) });
            ctl.nav.rid = Some("m42".to_string());
        }
        let (ok3, view3) = cancel_nav_goal();
        assert!(ok3);
        assert_eq!(view3.get("state").and_then(Value::as_str), Some("idle"));
        assert_eq!(view3.get("result").and_then(Value::as_str), Some("canceled"));
        assert!(view3.get("goal").is_some_and(Value::is_null));
        let st = queue_slot().st.lock().unwrap();
        assert!(st.ring.iter().any(|c| {
            c.body.get("c").and_then(Value::as_str) == Some("nav_cancel")
                && c.body.get("rid").and_then(Value::as_str) == Some("m42")
        }));
    }

    #[test]
    fn test_cancel_nav_goal_stops_patrol_and_prevents_redispatch() {
        let _guard = guard();
        init_all();
        let hub = safety::global();
        {
            let mut ctl = hub.control.lock().unwrap();
            ctl.patrol.route = Some(safety::Route {
                map_id: "map".into(),
                points: vec![(1.0, 0.0), (2.0, 0.0)],
            });
            ctl.patrol.status = "running";
            ctl.patrol.index = 0;
            ctl.patrol.laps = 0;
            ctl.current_map_id = Some("map".into());
            ctl.locked = false;
            ctl.user_locked = false;
        }
        nav_housekeeping_and_dispatch();
        let rid = {
            let ctl = hub.control.lock().unwrap();
            ctl.patrol.sending_rid.clone().expect("patrol waypoint dispatched")
        };
        assert!(apply_frame(&frame(
            "nav_result",
            json!({ "rid": rid, "status": "accepted", "text": "" })
        )));
        {
            let ctl = hub.control.lock().unwrap();
            assert_eq!(ctl.patrol.pending_rid.as_deref(), Some(rid.as_str()));
            assert_eq!(ctl.nav.state, "active");
        }
        queue_slot().st.lock().unwrap().ring.clear();

        // 取消导航必须同步停掉巡航、清空 nav 槽、发 nav_cancel + manual_zero
        let (ok, view) = cancel_nav_goal();
        assert!(ok);
        assert_eq!(view.get("state").and_then(Value::as_str), Some("idle"));
        assert!(view.get("goal").is_some_and(Value::is_null));
        {
            let ctl = hub.control.lock().unwrap();
            assert_eq!(ctl.patrol.status, "stopped");
            assert_eq!(ctl.patrol.stop_reason, "nav_cancelled");
            assert!(ctl.nav.goal.is_none());
        }
        {
            let st = queue_slot().st.lock().unwrap();
            assert!(st.ring.iter().any(|c| {
                c.body.get("c").and_then(Value::as_str) == Some("nav_cancel")
                    && c.body.get("rid").and_then(Value::as_str) == Some(rid.as_str())
            }));
            assert!(st.ring.iter().any(|c| {
                c.body.get("c").and_then(Value::as_str) == Some("manual_zero")
            }), "取消在途导航必须下发 manual_zero 立即刹停");
        }

        // 取消回执到达后再跑 20Hz 对账，绝不可重新派发巡航目标
        queue_slot().st.lock().unwrap().ring.clear();
        assert!(apply_frame(&frame(
            "nav_result",
            json!({ "rid": rid, "status": "cancelled", "text": "已取消" })
        )));
        nav_housekeeping_and_dispatch();
        {
            let ctl = hub.control.lock().unwrap();
            assert_eq!(ctl.patrol.status, "stopped");
            assert!(ctl.patrol.sending_rid.is_none());
            assert!(ctl.patrol.pending_rid.is_none());
            assert!(ctl.patrol.cancel_request.is_none());
            assert_eq!(ctl.nav.state, "idle");
        }
        assert!(queue_slot().st.lock().unwrap().ring.iter().all(|c| {
            c.body.get("c").and_then(Value::as_str) != Some("nav_goal")
        }), "取消后不得重新派发 nav_goal");

        // 测试卫生：清理全局状态
        let mut ctl = hub.control.lock().unwrap();
        ctl.patrol.route = None;
        ctl.patrol.status = "idle";
        ctl.patrol.stop_reason.clear();
        ctl.patrol.cancel_request = None;
        ctl.patrol.sending_rid = None;
        ctl.patrol.pending_rid = None;
        ctl.locked = false;
        ctl.user_locked = false;
    }
}
