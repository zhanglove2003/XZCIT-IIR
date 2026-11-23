//! 运动许可与鉴权中枢（蓝图 §4.3(1)、§3.1 safety.rs）。
//!
//! 逐字移植旧实现的协议行为，依据：
//! - `motion_license` 会话校验与 350ms 超时锁停：
//!   旧 `patrol.py::MotionLicense`（patrol.py:651-822，`SESSION_TIMEOUT_S = 0.350`）；
//! - admin SHA-256 登录：旧 `manual_forward.py::ManualForwardControl.login`
//!   （manual_forward.py:51-63，用户名恒等 `admin`，密码 SHA-256 摘要与
//!   `OPI_WEB_PASSWORD_SHA256` 常数时间比较）；
//! - 手动控制（start/heartbeat/stop/超时归零）：manual_forward.py 全文
//!   （`HEARTBEAT_TIMEOUT = 0.35`、`PUBLISH_PERIOD = 0.10`、挡位上限 0.8、
//!   `MAX_TIMED_SECONDS = 10`、方向表 forward/reverse/left/right）；
//! - 巡航状态机（保存/启动/锁停/恢复）：旧 `patrol.py::PatrolRunner`
//!   （patrol.py:243-458）与 `validate_route`（patrol.py:209-239）；
//! - 单点导航：旧 `nav_goal.py::NavGoalControl`（无动作客户端时的拒绝文案）；
//! - 控制帧环形日志：旧 `web_debug.py::ws_log_snapshot/_ws_log_add`
//!   （web_debug.py:991-1006，`WS_LOG_MAX = 50`）。
//!
//! 仿真环境的差异（如实声明，不属于协议分歧）：
//! - 旧实现里巡航派发要经 Nav2 动作客户端异步确认；Fake 引擎里目标
//!   即时被仿真车接受，因此「待确认取消」链路坍缩为同步确认；
//! - 锁停通道不再发下游话题，直接作用在仿真车上（语义等价：锁停即零速）。

use std::collections::HashSet;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, OnceLock};
use std::time::Instant;

use serde_json::{json, Value};

use crate::Config;

// ---------------------------------------------------------------------------
// 时间基准：进程内单调时钟（旧实现 time.monotonic 的对应物）。
// ---------------------------------------------------------------------------

static START: OnceLock<Instant> = OnceLock::new();

/// 单调毫秒（服务端判定一律用它，不信任客户端时间戳——patrol.py:659-661）。
pub fn now_ms() -> u64 {
    let start = START.get_or_init(Instant::now);
    start.elapsed().as_millis() as u64
}

// ---------------------------------------------------------------------------
// 协议常量（逐字对齐蓝图 §4.3 与旧实现）
// ---------------------------------------------------------------------------

/// 许可心跳发送周期（前端 Blob Web Worker，ms）。蓝图 §4.3(1)。
#[allow(dead_code)] // 协议常量：蓝图 §4.3 记载的前端周期，服务端判定不直接使用
pub const LICENSE_HEARTBEAT_MS: u64 = 100;
/// 许可超时锁停阈值（ms）：断流超过 350ms 服务端立即撤销下游授权并锁停。
pub const LICENSE_TTL_MS: u64 = 350;
/// 手动心跳续期周期（前端按住方向键期间每 150ms 续期一次）。蓝图 §4.3(1)。
#[allow(dead_code)] // 协议常量：蓝图 §4.3 记载的前端周期，服务端判定不直接使用
pub const MANUAL_HEARTBEAT_PERIOD_MS: u64 = 150;
/// 手动心跳超时（ms）：超时未收到自动归零停机（manual_forward.py:17）。
pub const MANUAL_HEARTBEAT_TIMEOUT_MS: u64 = 350;
/// 手动速度重发周期（manual_forward.py:18）。
#[allow(dead_code)] // 协议常量：蓝图 §4.3 记载的前端周期，服务端判定不直接使用
pub const MANUAL_PUBLISH_PERIOD_MS: u64 = 100;
/// 手动输入挡位上限（manual_forward.py:11-15，速度与转向一致）。
pub const MANUAL_MAX_INPUT: f64 = 0.8;
/// timed 模式最长运行秒数（manual_forward.py:16）。
pub const MANUAL_MAX_TIMED_S: f64 = 10.0;
/// view 心跳周期（ms；旧 VIEW_BEAT_S = 1.0）。
#[allow(dead_code)] // 协议常量：蓝图 §4.3 记载的前端周期，服务端判定不直接使用
pub const VIEW_BEAT_MS: u64 = 1000;
/// 浏览器多久没发任何控制消息就按「不要实时数据」处理（旧 CLIENT_TIMEOUT）。
pub const CLIENT_TIMEOUT_MS: u64 = 20_000;
/// 会话存活窗口（ms）：max(350, view 心跳 1000*2+200)（web_debug.py:680-688）。
pub const SESSION_WINDOW_MS: u64 = 2200;
/// 航点数上限 / 下限与最小间距（patrol.py:31-33）。
pub const MIN_WAYPOINTS: usize = 2;
pub const MAX_WAYPOINTS: usize = 32;
pub const MIN_SPACING_M: f64 = 0.30;
/// 单点导航整体超时（ms；旧 PATROL_GOAL_TIMEOUT_S = 15.0）。
pub const NAV_GOAL_TIMEOUT_MS: u64 = 15_000;
/// 控制帧环形日志条数（web_debug.py:974 `WS_LOG_MAX = 50`）。
pub const WS_LOG_MAX: usize = 50;
/// 到达半径（m）：仿真车判定航点/导航目标已到达。
pub const GOAL_REACHED_M: f64 = 0.15;

// ---------------------------------------------------------------------------
// SHA-256 鉴权（manual_forward.py::login 的移植）
// ---------------------------------------------------------------------------

/// SHA-256 小写十六进制摘要。
pub fn sha256_hex(input: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    let digest = Sha256::digest(input);
    let mut hex = String::with_capacity(digest.len() * 2);
    for byte in digest {
        hex.push_str(&format!("{byte:02x}"));
    }
    hex
}

/// 常数时间比较（manual_forward.py 用 hmac.compare_digest 的等价物）。
/// 长度不同立即为假（与 compare_digest 一致），等长时遍历全量再折叠。
pub fn ct_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut diff = 0u8;
    for (x, y) in a.iter().zip(b.iter()) {
        diff |= x ^ y;
    }
    diff == 0
}

// ---------------------------------------------------------------------------
// 状态结构
// ---------------------------------------------------------------------------

/// 运动许可注册表（patrol.py::MotionLicense 的移植）。
#[derive(Debug, Clone)]
pub struct License {
    /// 当前会话标识（新协议为服务端握手时下发的字符串）。
    pub session: Option<String>,
    /// 最近一次接受的上行序号（同会话内严格递增）。
    pub last_seq: i64,
    /// 最近一次接受的单调时刻（ms）。
    pub last_rx_ms: Option<u64>,
    /// 失效上报只报一次的标记。
    pub lost_reported: bool,
    /// 被拒绝的许可帧数（诊断）。
    pub rejected: u64,
    /// 新会话接管旧会话的次数（诊断）。
    pub takeovers: u64,
}

impl License {
    fn new() -> Self {
        Self {
            session: None,
            last_seq: -1,
            last_rx_ms: None,
            lost_reported: false,
            rejected: 0,
            takeovers: 0,
        }
    }
}

/// 手动控制状态（manual_forward.py::ManualForwardControl 的移植）。
#[derive(Debug, Clone)]
pub struct Manual {
    /// 已登录连接集合。
    pub authenticated: HashSet<u64>,
    /// 当前持有手动命令的连接；None = 空闲。
    pub owner: Option<u64>,
    /// 挡位速度（标量，用于回显）。
    pub speed: f64,
    /// 方向（forward/reverse|backward/left/right）。
    pub direction: String,
    /// 当前下发命令 (vx, wz)。
    pub cmd: (f64, f64),
    /// 最近一次心跳的单调时刻（ms）。
    pub last_heartbeat_ms: u64,
    /// timed 模式的截止时刻（ms）；hold 模式为 None。
    pub deadline_ms: Option<u64>,
    /// 诊断：心跳到达次数（manual_forward.py _BEAT_COUNT）。
    pub beats: u64,
    /// 诊断：主动停车次数。
    pub stops: u64,
    /// 边沿去重：已就「被顶替」提示过本连接（防 150ms 心跳刷 toast）。
    pub steal_warned: bool,
    /// 登录顶替凭证：最近一次登录成功时发放的一次性授权（发放时刻 ms）。
    /// 登录后第一个带**新会话号**的许可心跳即可行使立即接管（2026-11-23
    /// 用户裁决：最新登录顶替旧登录，不等旧会话 350ms 断流）。窗口 10s，
    /// 过期/被行使/被更新登录覆盖自然失效。None = 无待行使的顶替权。
    pub login_takeover_at: Option<u64>,
    /// 登录顶替凭证绑定的目标许可会话号：Some(sid) 时仅允许该会话号行使顶替，
    /// 防止多标签页并存时被未登录的第三页心跳抢走一次性凭证；None 时兼容不带
    /// session_id 的旧客户端/单测（任意新会话号首帧可接管）。
    pub login_takeover_session: Option<String>,
}

impl Manual {
    fn new() -> Self {
        Self {
            authenticated: HashSet::new(),
            owner: None,
            speed: 0.0,
            direction: "forward".into(),
            cmd: (0.0, 0.0),
            last_heartbeat_ms: 0,
            deadline_ms: None,
            beats: 0,
            stops: 0,
            steal_warned: false,
            login_takeover_at: None,
            login_takeover_session: None,
        }
    }
}

/// 航点路线（patrol.py::validate_route 的产物；航向按指向下一点计算）。
#[derive(Debug, Clone)]
pub struct Route {
    pub map_id: String,
    pub points: Vec<(f64, f64)>,
}

/// 仿真目标点。
#[derive(Debug, Clone, Copy)]
pub struct Goal {
    pub x: f64,
    pub y: f64,
    pub yaw: Option<f64>,
}

/// 巡航状态机（patrol.py::PatrolRunner 的仿真化移植：取消即时确认）。
#[derive(Debug, Clone)]
pub struct Patrol {
    /// 已保存路线。
    pub route: Option<Route>,
    /// `idle` | `running` | `stopped`。
    pub status: &'static str,
    /// 当前目标航点下标。
    pub index: usize,
    /// 完成圈数。
    pub laps: u64,
    /// 最近一次停车/停止原因。
    pub stop_reason: String,
    /// ros 模式：已发出、尚未被 Nav2 接受的航点 rid（patrol.py::sending）。
    /// fake 模式恒 None（目标即时确认，链路坍缩为同步）。
    pub sending_rid: Option<String>,
    /// ros 模式：已被接受、等待结果的航点 rid（patrol.py::pending）。
    pub pending_rid: Option<String>,
    /// ros 模式：已发出取消、尚未收到 cancelled 结果的目标 rid
    /// （patrol.py::cancel_request；结果确认前保持锁停）。
    pub cancel_request: Option<String>,
}

impl Patrol {
    fn new() -> Self {
        Self {
            route: None,
            status: "idle",
            index: 0,
            laps: 0,
            stop_reason: String::new(),
            sending_rid: None,
            pending_rid: None,
            cancel_request: None,
        }
    }
}

/// 单点导航状态（nav_goal.py::_snapshot 的字段子集）。
#[derive(Debug, Clone)]
pub struct NavGoal {
    /// `idle` | `active` | `canceled` | `done` | `failed`。
    pub state: &'static str,
    pub goal: Option<Goal>,
    pub goal_t_ms: Option<u64>,
    pub seq: u64,
    /// 结果文本（成功/取消/失败原因）。
    pub result: String,
    /// 发送/执行错误。
    pub error: String,
    /// ros 模式：在途目标的 rid（与巡逻共用同一「在途目标槽」，见
    /// 契约 commandMap patrol 行）；fake 模式恒 None。
    pub rid: Option<String>,
}

impl NavGoal {
    fn new() -> Self {
        Self {
            state: "idle",
            goal: None,
            goal_t_ms: None,
            seq: 0,
            result: String::new(),
            error: String::new(),
            rid: None,
        }
    }
}

/// 控制域总状态：一次一把锁（操作都是 O(1)/O(n小)，20Hz 轮询无争用问题）。
pub struct Control {
    /// 服务端握手会话号：新连接下发 `motion_license_session` 时使用。
    pub server_session_id: String,
    /// admin 密码摘要（`OPI_WEB_PASSWORD_SHA256`）；None = 拒绝一切登录。
    pub password_sha256: Option<String>,
    /// `OPI_BOOT_MARKER` 路径（开机 20s 倒计时标记文件）。
    pub boot_marker: PathBuf,
    /// Fake 引擎算出的当前地图内容标识（sha256 前 16 位）。
    pub current_map_id: Option<String>,
    /// ros 模式最近一次地图 seq（垫片重启会归零，钳位保 ws_video 的
    /// 「seq 变化才推」判定永远触发）。fake 模式不使用。
    pub map_seq_last: u64,
    pub license: License,
    pub manual: Manual,
    pub patrol: Patrol,
    pub nav: NavGoal,
    /// 巡航开关（已登录 + control 页切换）。
    pub cruise_on: bool,
    /// 全局锁停（motion_stop）与自动锁停（license_lost / 会话失效）共用。
    /// 巡航状态机「启动即锁停」（patrol.py:267-269），只挡自动任务。
    pub locked: bool,
    pub lock_reason: String,
    /// 用户显式紧急锁停（motion_stop）：连 24s 脚本演示与手动一起冻结，
    /// 需显式恢复才能再次运动（蓝图 §4.3(1) motion_stop）。
    pub user_locked: bool,
    /// 超声门闩：近距/故障后必须显式恢复（旧 ultrasonic_rearm 语义）。
    pub us_latched: bool,
    /// 重定位进行中标志。
    pub relocal_busy: bool,
    /// 最近一次「网页会话还活着」的单调时刻（ms）；None = 从未确认。
    pub live_t_ms: Option<u64>,
    /// 最近一次会话失效原因（诊断）。
    pub session_lost: String,
    /// 在实时页（control/map）的连接集合。
    pub live_sessions: HashSet<u64>,
    /// 已登录且在控制页的连接集合（视频通道据此推相机）。
    pub live_controls: HashSet<u64>,
}

/// 控制帧环形日志（web_debug.py::_ws_log_add）。
pub struct WsLog {
    items: Vec<Value>,
    /// 最近一条控制帧到达的单调时刻（ms）；0 = 从未收到。
    last_frame_ms: u64,
}

/// 全局控制中枢：Fake 引擎与两条 WebSocket 通道共享。
pub struct Hub {
    pub control: Mutex<Control>,
    pub wslog: Mutex<WsLog>,
    /// 连接号分配器。
    conn_seq: AtomicU64,
}

static HUB: OnceLock<Hub> = OnceLock::new();

/// 进程启动时装配全局中枢。
pub fn init_hub(cfg: &Config) -> &'static Hub {
    START.get_or_init(Instant::now);
    let server_session_id = {
        use rand::Rng;
        let mut rng = rand::rng();
        // 64-bit 随机正整数，十六进制呈现（页面不解析内容，只透传）。
        let v: u64 = rng.random_range(1..=u64::MAX);
        format!("{v:016x}")
    };
    let hub = Hub {
        control: Mutex::new(Control {
            server_session_id,
            password_sha256: cfg.password_sha256.clone(),
            boot_marker: cfg.boot_marker.clone(),
            current_map_id: None,
            map_seq_last: 0,
            license: License::new(),
            manual: Manual::new(),
            patrol: Patrol::new(),
            nav: NavGoal::new(),
            cruise_on: false,
            locked: true,
            lock_reason: "global_stop".into(),
            user_locked: false,
            us_latched: false,
            relocal_busy: false,
            live_t_ms: None,
            session_lost: String::new(),
            live_sessions: HashSet::new(),
            live_controls: HashSet::new(),
        }),
        wslog: Mutex::new(WsLog { items: Vec::new(), last_frame_ms: 0 }),
        conn_seq: AtomicU64::new(1),
    };
    HUB.get_or_init(|| hub)
}

/// 取全局中枢；未初始化视为装配错误。
pub fn global() -> &'static Hub {
    HUB.get().expect("Hub not initialized: call init_hub() in main")
}

impl Hub {
    /// 分配连接号（ManualState / 会话集合以此为主键）。
    pub fn next_conn_id(&self) -> u64 {
        self.conn_seq.fetch_add(1, Ordering::Relaxed)
    }

    // -- 控制帧日志（web_debug.py:991-1006） ------------------------------

    /// 记一条控制帧（环形覆盖 50 条）。`detail` 需已脱敏。
    pub fn ws_log_add(&self, kind: &str, detail: Value) {
        let mut log = self.wslog.lock().expect("wslog poisoned");
        let entry = json!({
            "t": std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| (d.as_secs_f64() * 1000.0).round() / 1000.0)
                .unwrap_or(0.0),
            "kind": kind,
            "detail": detail,
        });
        log.items.push(entry);
        if log.items.len() > WS_LOG_MAX {
            let excess = log.items.len() - WS_LOG_MAX;
            log.items.drain(..excess);
        }
    }

    /// 最近一条控制帧到达时刻（任何解码成功的消息都算，含心跳）。
    pub fn ws_touch_frame(&self) {
        self.wslog.lock().expect("wslog poisoned").last_frame_ms = now_ms();
    }

    /// `/api/wslog` 载荷：`{last_frame_age, items}`（web_debug.py::ws_log_snapshot）。
    pub fn ws_log_snapshot(&self) -> Value {
        let log = self.wslog.lock().expect("wslog poisoned");
        let age = if log.last_frame_ms == 0 {
            Value::Null
        } else {
            let age_ms = now_ms().saturating_sub(log.last_frame_ms);
            json!((age_ms as f64 / 1000.0 * 1000.0).round() / 1000.0)
        };
        json!({ "last_frame_age": age, "items": log.items })
    }

    // -- 运动许可（patrol.py::MotionLicense） ------------------------------

    /// 登录顶替凭证是否仍可行使（10s 窗口；单独提出便于测试注入时刻）。
    fn login_takeover_active(ctl: &Control, now: u64, session_id: &str) -> bool {
        const LOGIN_TAKEOVER_WINDOW_MS: u64 = 10_000;
        let in_window = ctl
            .manual
            .login_takeover_at
            .is_some_and(|t| now.saturating_sub(t) <= LOGIN_TAKEOVER_WINDOW_MS);
        if !in_window {
            return false;
        }
        match ctl.manual.login_takeover_session.as_deref() {
            Some(target) => target == session_id,
            None => true,
        }
    }

    /// 校验并接受一条 `motion_license`。返回拒绝原因（patrol.py::accept）。
    pub fn license_accept(
        &self,
        session_id: &str,
        seq: i64,
        ttl_ms: Option<u64>,
    ) -> Result<(), &'static str> {
        let now = now_ms();
        // 客户端 ttl 仅做参考记录；服务端超时一律按 350ms 判（不信任客户端
        // 时间戳与窗口声明——patrol.py:659-661）。异常大的 ttl 视为无效。
        let _ttl = ttl_ms.unwrap_or(LICENSE_TTL_MS).min(5_000);
        if session_id.is_empty() {
            return Err("session_invalid");
        }
        let mut ctl = self.control.lock().expect("control poisoned");
        // 登录顶替凭证（2026-11-23 用户裁决：最新登录顶替旧登录）：登录后
        // 窗口内带目标会话号的心跳立即接管，不等旧会话 350ms 断流。
        let login_takeover = Self::login_takeover_active(&ctl, now, session_id);
        enum SessionMove {
            Keep,
            Adopt,
            Takeover,
        }
        let (move_kind, mismatch) = {
            let lic = &mut ctl.license;
            match &lic.session {
                None => (SessionMove::Adopt, false),
                Some(cur) if cur == session_id => (SessionMove::Keep, false),
                Some(_) => {
                    if login_takeover {
                        // 登录授权的立即接管：旧页此后收 session_mismatch nak
                        // （其 Worker 持续重发被拒即静默降级），安全语义不变——
                        // 下方 Takeover 分支仍先锁停一次再换会话。
                        (SessionMove::Takeover, false)
                    } else {
                        let ended = match lic.last_rx_ms {
                            None => true,
                            Some(t) => now.saturating_sub(t) > LICENSE_TTL_MS,
                        };
                        if !ended {
                            lic.rejected += 1;
                            (SessionMove::Keep, true)
                        } else {
                            (SessionMove::Takeover, false)
                        }
                    }
                }
            }
        };
        if mismatch {
            return Err("session_mismatch");
        }
        match move_kind {
            SessionMove::Keep => {
                if login_takeover {
                    ctl.manual.login_takeover_at = None;
                    ctl.manual.login_takeover_session = None;
                }
            }
            SessionMove::Adopt => {
                ctl.license.session = Some(session_id.to_string());
            }
            SessionMove::Takeover => {
                // 旧代次已停止送许可（或被最新登录顶替）：新页面接管，序号
                // 从头开始（patrol.py:723-734）。登录顶替行使一次即消费。
                if login_takeover {
                    ctl.manual.login_takeover_at = None;
                    ctl.manual.login_takeover_session = None;
                }
                ctl.license.session = Some(session_id.to_string());
                ctl.license.last_seq = -1;
                ctl.license.takeovers += 1;
                // 顶替发生时立刻判一次失效（patrol.py::_event_stale）。
                Self::lock_locked(&mut ctl, "license_lost");
            }
        }
        if seq <= ctl.license.last_seq {
            ctl.license.rejected += 1;
            return Err("seq_replay");
        }
        ctl.license.last_seq = seq;
        ctl.license.last_rx_ms = Some(now);
        Ok(())
    }

    /// 主动释放许可（`motion_license_release`）：页面隐藏/后台化时由前端
    /// 发出，持有者自己交还许可。只清会话不触发断流路径——last_rx_ms 归
    /// None 后 license_tick 走「从未收到许可」分支，不锁停（页面只是暂时
    /// 不在前台，不是崩溃）。seq 归 -1：前端 seq 在页面生命周期内单调递增、
    /// WS 重连不清零，不重置会被判 seq_replay 永久拒绝。
    /// 返回是否确实释放了当前持有会话。
    pub fn license_release(&self, session_id: &str) -> bool {
        if session_id.is_empty() {
            return false;
        }
        let mut ctl = self.control.lock().expect("control poisoned");
        let is_holder = ctl.license.session.as_deref() == Some(session_id);
        if is_holder {
            ctl.license.session = None;
            ctl.license.last_rx_ms = None;
            ctl.license.last_seq = -1;
        }
        is_holder
    }

    /// 持锁版本的许可快照（patrol.py::MotionLicense.snapshot 的字段）：
    /// 给 patrol_status_frame 等已持有 control 锁的调用方使用
    /// （std Mutex 不可重入，禁止在持锁时再走 locking 包装）。
    fn license_snapshot_locked(ctl: &Control, now: u64) -> Value {
        let lic = &ctl.license;
        let age_s = lic.last_rx_ms.map(|t| {
            (now.saturating_sub(t) as f64 / 1000.0 * 1000.0).round() / 1000.0
        });
        json!({
            "session": lic.session,
            "seq": lic.last_seq,
            "age_s": age_s,
            "fresh": Self::license_fresh_locked(ctl, now),
            "rejected": lic.rejected,
            "takeovers": lic.takeovers,
            "session_dead": !Self::license_fresh_locked(ctl, now),
        })
    }

    /// 持锁版许可新鲜度（ros.rs 的意图对账在已持有 control 锁时调用，
    /// std Mutex 不可重入不能走 license_fresh）。
    pub fn license_fresh_locked(ctl: &Control, now: u64) -> bool {
        match ctl.license.last_rx_ms {
            None => false,
            Some(t) => now.saturating_sub(t) <= LICENSE_TTL_MS,
        }
    }

    /// 许可看护节拍：断流超过 350ms 时上报一次失效并锁停
    /// （patrol.py::report_lost + on_lost('license_lost')）。
    /// 返回本次节拍是否发生了失效事件。
    pub fn license_tick(&self) -> bool {
        let now = now_ms();
        let mut ctl = self.control.lock().expect("control poisoned");
        if Self::license_fresh_locked(&ctl, now) {
            ctl.license.lost_reported = false;
            return false;
        }
        if ctl.license.last_rx_ms.is_none() {
            // 从未收到许可：不视为「断流」，只按未授权处理，不锁停。
            return false;
        }
        if ctl.license.lost_reported {
            return false;
        }
        ctl.license.lost_reported = true;
        // 撤销下游授权并锁停巡航（蓝图 §4.3(1)：350ms 断流 → 立即锁停）。
        if ctl.patrol.status == "running" {
            ctl.patrol.status = "stopped";
            ctl.patrol.stop_reason = "license_lost".to_string();
        }
        Self::lock_locked(&mut ctl, "license_lost");
        true
    }

    // -- 全局锁停 / 恢复（patrol.py::PatrolRunner.lock_stop / rearm） ------

    /// 请求锁停。已锁停时保留首个原因（patrol.py:415-425）。
    pub fn lock_locked(ctl: &mut Control, reason: &str) {
        // patrol.py:415-425 的保留首因规则有一个仿真语境的例外：巡航状态机
        // 「启动即锁停」（global_stop），若照搬保留首因，许可断流/会话失效
        // 之后页面永远只会看到启动原因，排查不出真实锁停原因。这里允许
        // 后到的具体原因覆盖泛化的 global_stop；两个具体原因之间仍保留首因。
        let generic = ctl.lock_reason == "global_stop";
        let already = ctl.locked && !ctl.lock_reason.is_empty() && !generic;
        ctl.locked = true;
        if !already {
            ctl.lock_reason = if reason.is_empty() {
                "global_stop".to_string()
            } else {
                reason.to_string()
            };
        }
    }

    /// 锁停原因 → guard `stop_cause` 枚举（patrol.py::guard_stop_cause）。
    pub fn guard_stop_cause(reason: &str) -> &'static str {
        match reason {
            "license_lost" => "license_expired",
            _ => "global_stop",
        }
    }

    /// B6 恢复状态口径（patrol.py::rearm_state）。
    pub fn rearm_state(locked: bool, licensed: bool, released: bool) -> &'static str {
        if !locked {
            return "not_required";
        }
        if !licensed {
            return "rearm_pending";
        }
        if released {
            "not_required"
        } else {
            "rearm_pending"
        }
    }

    /// 全局紧急锁停（motion_stop）：取消在途任务并锁停
    /// （patrol.py::PatrolRunner.stop 的仿真化）。
    pub fn motion_stop(&self) {
        let mut ctl = self.control.lock().expect("control poisoned");
        ctl.patrol.status = "stopped";
        ctl.patrol.stop_reason = "global_stop".to_string();
        ctl.nav.state = "canceled";
        ctl.nav.result = "canceled".to_string();
        // 全局紧急锁停连手动输入一起归零（manual_forward._stop 语义）。
        let was_active = ctl.manual.owner.is_some();
        ctl.manual.owner = None;
        ctl.manual.speed = 0.0;
        ctl.manual.cmd = (0.0, 0.0);
        ctl.manual.deadline_ms = None;
        if was_active {
            ctl.manual.stops += 1;
        }
        if !ctl.locked || ctl.lock_reason.is_empty() {
            ctl.lock_reason = "global_stop".to_string();
        }
        ctl.locked = true;
        ctl.user_locked = true;
    }

    /// 解除锁停（motion_rearm）：必须先拿到新鲜许可；解除后停在原地
    /// （patrol.py:431-458）。
    pub fn motion_rearm(&self) -> (bool, String) {
        let now = now_ms();
        let mut ctl = self.control.lock().expect("control poisoned");
        if !Self::license_fresh_locked(&ctl, now) {
            return (false, "网页许可无效，不能解除锁停".to_string());
        }
        if ctl.nav.state == "active" {
            ctl.nav.state = "canceled";
            ctl.nav.result = "canceled".to_string();
        }
        if !ctl.locked && !ctl.user_locked {
            return (true, "已恢复".to_string());
        }
        ctl.locked = false;
        ctl.user_locked = false;
        ctl.lock_reason.clear();
        ctl.patrol.stop_reason.clear();
        (true, "已恢复".to_string())
    }

    // -- 手动控制（manual_forward.py 的移植） ------------------------------

    /// admin 登录（manual_forward.py:51-63）。失败时同时登出该连接。
    #[allow(dead_code)] // 兼容不带 session_id 的调用方与现有单测
    pub fn manual_login(&self, conn: u64, username: &str, password: &str) -> bool {
        self.manual_login_with_session(conn, username, password, None)
    }

    /// admin 登录并可选择绑定页面许可会话号，避免多标签页竞态抢走顶替凭证。
    pub fn manual_login_with_session(
        &self,
        conn: u64,
        username: &str,
        password: &str,
        session_id: Option<&str>,
    ) -> bool {
        let mut ctl = self.control.lock().expect("control poisoned");
        let Some(configured) = ctl.password_sha256.as_deref() else {
            // 未设置 OPI_WEB_PASSWORD_SHA256：拒绝一切登录（蓝图 §4.1）。
            ctl.manual.authenticated.remove(&conn);
            return false;
        };
        let user_ok = ct_eq(username.as_bytes(), b"admin");
        let pass_ok = ct_eq(sha256_hex(password.as_bytes()).as_bytes(), configured.as_bytes());
        if user_ok && pass_ok {
            ctl.manual.authenticated.insert(conn);
            // 最新登录顶替旧登录（2026-11-23 用户裁决）：发放一次性顶替凭证。
            // 绑定 session_id 时仅允许该会话号行使接管，防止旁路页面截胡；
            // 未传时保留兼容任意新会话号首帧接管。
            ctl.manual.login_takeover_at = Some(now_ms());
            ctl.manual.login_takeover_session = session_id
                .map(str::trim)
                .filter(|s| !s.is_empty())
                .map(str::to_string);
            true
        } else {
            ctl.manual.authenticated.remove(&conn);
            false
        }
    }

    pub fn manual_authenticated(&self, conn: u64) -> bool {
        self.control.lock().expect("control poisoned").manual.authenticated.contains(&conn)
    }

    /// 手动接管（manual_start）。成功后回发 manual_started 由调用方负责。
    /// 错误文案逐字对齐 manual_forward.py / web_debug.py。
    pub fn manual_start(
        &self,
        conn: u64,
        speed: f64,
        mode: &str,
        duration: Option<f64>,
        direction: &str,
        live: bool,
    ) -> Result<(), String> {
        let mut ctl = self.control.lock().expect("control poisoned");
        if !ctl.manual.authenticated.contains(&conn) {
            return Err("请先登录手动控制".to_string());
        }
        if !live {
            return Err("请先进入手动控制页面".to_string());
        }
        if ctl.user_locked {
            return Err("已全局锁停，请先恢复".to_string());
        }
        // 自动锁停（license_lost / session_dead / web_disconnected / goal_timeout
        // 等，user_locked=false）也必须拦手动启动——否则 manual_start 会 Ok 回
        // 「已接管」，速度命令下发但下游因全局锁停不执行，车不动且无任何报错
        // （2026-11-22 实测：license 断流锁停后按方向键 toast「手动控制已接管」
        // 但车不动、无拒绝弹窗）。恢复动作本身（motion_rearm）不在这条链上，
        // manual_stop / heartbeat / manual_tick 一样只是归零收尾，不受此拦。
        if ctl.locked {
            return Err("已锁停，请先点「恢复」".to_string());
        }
        // 巡航接管：真正从空闲进入手动输入时，先取消旧任务
        // （web_debug.py::_manual_takeover_hook）。
        if ctl.manual.owner.is_none() {
            Self::takeover_locked(&mut ctl);
        }
        let directions: &[(&str, f64, f64)] = &[
            ("forward", 1.0, 0.0),
            ("reverse", -1.0, 0.0),
            // 蓝图 §4.3(1) 用 backward，旧实现用 reverse：两者都收。
            ("backward", -1.0, 0.0),
            ("left", 0.0, -1.0),
            ("right", 0.0, 1.0),
        ];
        let Some((_, x_sign, z_sign)) =
            directions.iter().find(|(name, _, _)| *name == direction)
        else {
            return Err("未知控制方向".to_string());
        };
        if speed == 0.0 {
            return Err("当前为空档，请先挂入巡航档或运动档".to_string());
        }
        if !(speed.is_finite() && speed > 0.0 && speed <= MANUAL_MAX_INPUT) {
            return Err(format!("运动输入 必须大于 0 且不超过 {MANUAL_MAX_INPUT}"));
        }
        let deadline = match mode {
            "hold" => {
                if duration.is_some() {
                    return Err("未知控制方式".to_string());
                }
                None
            }
            "timed" => {
                let Some(d) = duration else {
                    return Err("未知控制方式".to_string());
                };
                if !(d.is_finite() && d > 0.0 && d <= MANUAL_MAX_TIMED_S) {
                    return Err(format!("运行时间 必须大于 0 且不超过 {MANUAL_MAX_TIMED_S}"));
                }
                Some(now_ms() + (d * 1000.0).round() as u64)
            }
            _ => return Err("未知控制方式".to_string()),
        };
        let now = now_ms();
        let m = &mut ctl.manual;
        m.cmd = (*x_sign * speed, *z_sign * speed);
        m.speed = speed;
        m.direction = direction.to_string();
        m.owner = Some(conn);
        m.last_heartbeat_ms = now;
        m.deadline_ms = deadline;
        m.steal_warned = false;   // 重新持有：允许下一次被顶替提示
        Ok(())
    }

    /// 手动心跳（manual_heartbeat）：150ms 续期；超时/到期归零停机
    /// （manual_forward.py:148-158）。返回 Some(提示) 表示本连接已不是
    /// owner（被另一页接管）或未认证——调用方应补发 manual_error 告知。
    pub fn manual_heartbeat(&self, conn: u64) -> Option<String> {
        let now = now_ms();
        let mut ctl = self.control.lock().expect("control poisoned");
        ctl.manual.beats += 1;
        if ctl.manual.owner != Some(conn) {
            // owner 不是本连接：若 owner 是另一连接（Some(其他)）= 被顶替，
            // 只提示一次（steal_warned 边沿去重，防 150ms 心跳刷屏）；
            // owner 为 None = 空闲（自己刚被锁停拦截/还没持到），静默，
            // 不报「被接管」误报。
            if let Some(o) = ctl.manual.owner {
                if o != conn
                    && ctl.manual.authenticated.contains(&conn)
                    && !ctl.manual.steal_warned
                {
                    ctl.manual.steal_warned = true;
                    return Some("手动控制权已被其他页面接管".to_string());
                }
            }
            return None;
        }
        ctl.manual.steal_warned = false;
        let timeout = now.saturating_sub(ctl.manual.last_heartbeat_ms)
            > MANUAL_HEARTBEAT_TIMEOUT_MS;
        let expired = ctl.manual.deadline_ms.is_some_and(|d| now >= d);
        if timeout || expired {
            Self::manual_stop_locked(&mut ctl);
        } else {
            ctl.manual.last_heartbeat_ms = now;
        }
        None
    }

    /// 手动停止（manual_stop）：需已登录（manual_forward.py:175-179）。
    pub fn manual_stop(&self, conn: u64) -> Result<(), String> {
        let mut ctl = self.control.lock().expect("control poisoned");
        if !ctl.manual.authenticated.contains(&conn) {
            return Err("请先登录手动控制".to_string());
        }
        Self::manual_stop_locked(&mut ctl);
        Ok(())
    }

    fn manual_stop_locked(ctl: &mut Control) {
        let was_active = ctl.manual.owner.is_some();
        ctl.manual.owner = None;
        ctl.manual.speed = 0.0;
        ctl.manual.cmd = (0.0, 0.0);
        ctl.manual.deadline_ms = None;
        if was_active {
            ctl.manual.stops += 1;
        }
    }

    /// 引擎节拍：心跳超时 / timed 到期 → 归零停机（manual_forward.py::tick）。
    pub fn manual_tick(&self) {
        let now = now_ms();
        let mut ctl = self.control.lock().expect("control poisoned");
        if ctl.manual.owner.is_some() {
            let timeout = now.saturating_sub(ctl.manual.last_heartbeat_ms)
                > MANUAL_HEARTBEAT_TIMEOUT_MS;
            let expired = ctl.manual.deadline_ms.is_some_and(|d| now >= d);
            if timeout || expired {
                Self::manual_stop_locked(&mut ctl);
            }
        }
    }

    /// 连接断开：登出并停机（manual_forward.py:181-185）。
    pub fn manual_disconnect(&self, conn: u64) {
        let mut ctl = self.control.lock().expect("control poisoned");
        ctl.manual.authenticated.remove(&conn);
        if ctl.manual.owner == Some(conn) {
            Self::manual_stop_locked(&mut ctl);
        }
    }

    /// 手动接管钩子：取消旧导航并停掉在途巡航（松键后不自行恢复，
    /// web_debug.py::_manual_takeover_hook + patrol.py::manual_override）。
    fn takeover_locked(ctl: &mut Control) {
        if ctl.nav.state == "active" || ctl.nav.state == "sending" {
            ctl.nav.state = "canceled";
            ctl.nav.result = "canceled".to_string();
            ctl.nav.error = "手动接管".to_string();
        }
        if ctl.patrol.status == "running" {
            ctl.patrol.status = "stopped";
            ctl.patrol.stop_reason = "manual_takeover".to_string();
        }
    }

    // -- 巡航开关（web_debug.py::set_cruise 的仿真化） --------------------

    /// 切换自动避障巡航开关。Fake 通道恒可用。
    pub fn set_cruise(&self, on: bool) -> Result<(), String> {
        self.control.lock().expect("control poisoned").cruise_on = on;
        Ok(())
    }

    // -- 多航点巡航（patrol.py::validate_route / PatrolRunner 的仿真化） --

    /// 保存路线（patrol_save）。绑定当前地图标识，文案逐字对齐旧实现。
    pub fn patrol_save(
        &self,
        raw_points: &Value,
        map_id: Option<&str>,
    ) -> (bool, String) {
        let mut ctl = self.control.lock().expect("control poisoned");
        let Some(cur) = ctl.current_map_id.clone() else {
            return (
                false,
                "当前地图未知（未指定地图文件），不能保存路线".to_string(),
            );
        };
        if let Some(route) = &ctl.patrol.route {
            if route.map_id != cur {
                if map_id.is_some_and(|m| !m.is_empty()) {
                    return (
                        false,
                        "地图已更换：旧路线作废，请先清空路线再保存".to_string(),
                    );
                }
                ctl.patrol.route = None;
            }
        }
        let declared_ok = match map_id {
            None => true, // 未声明时按当前地图处理（旧 validate_route 允许 None→cur）
            Some(m) => m == cur,
        };
        if !declared_ok {
            return (false, "地图已更换，旧路线不可用".to_string());
        }
        let Some(list) = raw_points.as_array() else {
            return (false, "航点格式不正确".to_string());
        };
        let mut pts: Vec<(f64, f64)> = Vec::with_capacity(list.len());
        for p in list {
            // 蓝图 §4.3(1)：[{x,y},...]；旧实现：[[x,y],...]。两者都收。
            let (x, y) = match p {
                Value::Array(arr) if arr.len() >= 2 => {
                    match (arr[0].as_f64(), arr[1].as_f64()) {
                        (Some(x), Some(y)) => (x, y),
                        _ => return (false, "航点坐标不是数字".to_string()),
                    }
                }
                Value::Object(obj) => {
                    match (obj.get("x").and_then(Value::as_f64), obj.get("y").and_then(Value::as_f64)) {
                        (Some(x), Some(y)) => (x, y),
                        _ => return (false, "航点坐标不是数字".to_string()),
                    }
                }
                _ => return (false, "航点格式不正确".to_string()),
            };
            if !x.is_finite() || !y.is_finite() {
                return (false, "航点坐标非有限值".to_string());
            }
            pts.push((x, y));
        }
        if pts.len() > MAX_WAYPOINTS {
            return (false, format!("航点不能超过 {MAX_WAYPOINTS} 个"));
        }
        if pts.len() < MIN_WAYPOINTS {
            return (false, format!("至少需要 {MIN_WAYPOINTS} 个航点"));
        }
        for i in 0..pts.len() {
            let j = (i + 1) % pts.len();
            let d = ((pts[j].0 - pts[i].0).powi(2) + (pts[j].1 - pts[i].1).powi(2)).sqrt();
            if d < MIN_SPACING_M {
                return (
                    false,
                    format!(
                        "第 {} 与第 {} 个航点重合（<{:.2} m）",
                        i + 1,
                        j + 1,
                        MIN_SPACING_M
                    ),
                );
            }
        }
        ctl.patrol.route = Some(Route { map_id: cur, points: pts });
        let n = ctl.patrol.route.as_ref().map(|r| r.points.len()).unwrap_or(0);
        (true, format!("已保存 {n} 个航点"))
    }

    /// 启动多航点闭环巡航（patrol_start）。地图变了拒绝旧路线
    /// （patrol.py:284-312）。开始巡航本身即确认会话存活。
    pub fn patrol_start(&self) -> (bool, String) {
        let mut ctl = self.control.lock().expect("control poisoned");
        if ctl.user_locked {
            return (false, "已全局锁停，请先恢复".to_string());
        }
        let Some(route) = ctl.patrol.route.clone() else {
            return (false, "没有已保存的路线".to_string());
        };
        let Some(cur) = ctl.current_map_id.clone() else {
            return (false, "当前地图未知（未指定地图文件），不能保存路线".to_string());
        };
        if route.map_id != cur {
            Self::lock_locked(&mut ctl, "global_stop");
            ctl.patrol.status = "stopped";
            ctl.patrol.stop_reason = "map_changed".to_string();
            return (false, "地图已更换，请重新保存路线".to_string());
        }
        ctl.patrol.status = "running";
        ctl.patrol.index = 0;
        ctl.patrol.laps = 0;
        ctl.patrol.stop_reason.clear();
        if ctl.patrol.sending_rid.is_none() && ctl.patrol.pending_rid.is_none() {
            ctl.patrol.cancel_request = None;
        }
        ctl.locked = false;
        ctl.lock_reason.clear();
        ctl.live_t_ms = Some(now_ms());
        (true, "巡航已开始".to_string())
    }

    /// 巡航状态帧（patrol.py::status_frame + web_debug.py::patrol_status_frame）。
    pub fn patrol_status_frame(&self) -> Value {
        let now = now_ms();
        let ctl = self.control.lock().expect("control poisoned");
        let licensed = Self::license_fresh_locked(&ctl, now);
        let patrol = &ctl.patrol;
        let route_pts: Option<Vec<Vec<f64>>> = patrol.route.as_ref().map(|r| {
            r.points.iter().map(|(x, y)| vec![*x, *y]).collect()
        });
        let goal = match (&patrol.route, patrol.status) {
            (Some(r), "running") => {
                let (x, y) = r.points[patrol.index.min(r.points.len() - 1)];
                let yaw = {
                    let n = r.points.len();
                    let (nx, ny) = r.points[(patrol.index.min(n - 1) + 1) % n];
                    (ny - y).atan2(nx - x)
                };
                json!({ "x": x, "y": y, "yaw": yaw })
            }
            _ => Value::Null,
        };
        let released = licensed && !ctl.locked;
        let (license_state, gate) = if ctl.locked {
            ("lock_stop", "lock_stop")
        } else if !licensed {
            ("license_lost", "license_lost")
        } else {
            ("ok", if released { "ok" } else { "not_released" })
        };
        json!({
            "t": "patrol_status",
            "map_id": patrol.route.as_ref().map(|r| r.map_id.clone()),
            "current_map_id": ctl.current_map_id,
            "route_valid": patrol.route.as_ref()
                .is_some_and(|r| ctl.current_map_id.as_deref() == Some(r.map_id.as_str())),
            "index": patrol.index,
            "waypoints": route_pts.as_ref().map(|p| p.len()),
            "laps": patrol.laps,
            "goal": goal,
            "status": patrol.status,
            "stop_reason": patrol.stop_reason,
            "locked": ctl.locked,
            "lock_reason": ctl.lock_reason,
            "stop_cause": Self::guard_stop_cause(
                if ctl.lock_reason.is_empty() { &patrol.stop_reason } else { &ctl.lock_reason }),
            "rearm_state": Self::rearm_state(ctl.locked, licensed, released),
            "licensed": licensed,
            "license": Self::license_snapshot_locked(&ctl, now),
            "released": released,
            "license_state": license_state,
            "gate": gate,
            "route": route_pts,
            "session": {
                "live": ctl.live_sessions.len(),
                "lost": ctl.session_lost,
            },
        })
    }

    // -- 单点导航（nav_goal.py 的仿真化） ---------------------------------

    /// 地图选点导航（nav_goal）。坐标必须是有限数。
    pub fn nav_request(&self, x: f64, y: f64, th: Option<f64>) -> (bool, Value) {
        let mut ctl = self.control.lock().expect("control poisoned");
        if !x.is_finite() || !y.is_finite() || th.is_some_and(|t| !t.is_finite()) {
            ctl.nav.error = "目标坐标不是数字".to_string();
            return (false, self.nav_view_locked(&ctl));
        }
        let now = now_ms();
        ctl.nav.seq += 1;
        ctl.nav.state = "active";
        ctl.nav.goal = Some(Goal { x, y, yaw: th });
        ctl.nav.goal_t_ms = Some(now);
        ctl.nav.result.clear();
        ctl.nav.error.clear();
        (true, self.nav_view_locked(&ctl))
    }

    /// 取消当前单点导航或清除残留目标（nav_cancel）。巡航运行中一并停止巡航；无目标且已在 idle 时返回 (false, view)。
    pub fn nav_cancel(&self) -> (bool, Value) {
        let mut ctl = self.control.lock().expect("control poisoned");
        let nav_in_flight = ctl.nav.state == "active" || ctl.nav.state == "sending";
        let patrol_running = ctl.patrol.status == "running";
        let active_motion = nav_in_flight || patrol_running;
        let has_residual = ctl.nav.goal.is_some() || ctl.nav.state != "idle";
        if !active_motion && !has_residual {
            ctl.nav.error = "当前没有导航目标".to_string();
            return (false, self.nav_view_locked(&ctl));
        }
        if patrol_running {
            ctl.patrol.status = "stopped";
            ctl.patrol.stop_reason = "nav_cancelled".to_string();
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
        (true, self.nav_view_locked(&ctl))
    }

    /// 导航状态视图（nav_goal.py::_snapshot_locked 的字段）。
    pub fn nav_view(&self) -> Value {
        let ctl = self.control.lock().expect("control poisoned");
        self.nav_view_locked(&ctl)
    }

    fn nav_view_locked(&self, ctl: &Control) -> Value {
        let nav = &ctl.nav;
        let now = now_ms();
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

    // -- 超声波 / 重定位 ---------------------------------------------------

    /// 显式恢复超声门（ultrasonic_rearm）。仿真通道恒可用。
    pub fn ultrasonic_rearm(&self) -> (bool, &'static str) {
        self.control.lock().expect("control poisoned").us_latched = false;
        (true, "已请求重新启动超声波")
    }

    /// 重定位静止护栏（对齐旧 web_debug.py::_relocalize_motion_guard）：
    /// 有主动手动/导航/巡航任务时拒绝重定位，防止运动中重置 AMCL。
    pub fn relocalize_motion_guard(&self) -> Option<&'static str> {
        let ctl = self.control.lock().expect("control poisoned");
        if ctl.manual.owner.is_some() {
            return Some("手动控制中，请停稳后再重定位");
        }
        if ctl.nav.state == "sending" || ctl.nav.state == "active" {
            return Some("点图导航进行中，请先取消导航后再重定位");
        }
        if matches!(
            ctl.patrol.status,
            "running" | "holding_goal" | "cancelling"
        ) {
            return Some("巡航运行中，请先停止巡航后再重定位");
        }
        None
    }

    /// 重定位（relocalize）：防重入；后台延迟由 ws_control 的 spawned 任务处理。
    pub fn relocalize_begin(&self) -> bool {
        let mut ctl = self.control.lock().expect("control poisoned");
        if ctl.relocal_busy {
            return false;
        }
        ctl.relocal_busy = true;
        true
    }

    pub fn relocalize_end(&self) {
        self.control.lock().expect("control poisoned").relocal_busy = false;
    }

    // -- 页面会话（web_debug.py::_read_view / patrol_session_*） -----------

    /// view 心跳：登记实时页会话并续期。返回 (live, control_page)。
    pub fn view_touch(&self, conn: u64, page: &str) -> (bool, bool) {
        let now = now_ms();
        let mut ctl = self.control.lock().expect("control poisoned");
        let control_page = page == "control";
        let live = page == "control" || page == "map";
        if live {
            ctl.live_sessions.insert(conn);
            ctl.live_t_ms = Some(now);
            ctl.session_lost.clear();
        } else {
            ctl.live_sessions.remove(&conn);
        }
        if control_page && ctl.manual.authenticated.contains(&conn) {
            ctl.live_controls.insert(conn);
        } else {
            ctl.live_controls.remove(&conn);
        }
        (live, control_page)
    }

    /// 控制页未登录时的提示只发一次的判定辅助（web_debug.py:2620-2628）。
    pub fn manual_logged_in(&self, conn: u64) -> bool {
        self.control.lock().expect("control poisoned")
            .manual.authenticated.contains(&conn)
    }

    /// 任何实时消息都算会话存活（许可帧、view 心跳）。
    pub fn live_touch(&self) {
        let mut ctl = self.control.lock().expect("control poisoned");
        ctl.live_t_ms = Some(now_ms());
        ctl.session_lost.clear();
    }

    /// 视频通道：是否存在「已登录且在控制页」的会话（推相机）。
    pub fn has_live_control(&self) -> bool {
        !self.control.lock().expect("control poisoned").live_controls.is_empty()
    }

    /// 会话看护节拍（patrol.py::patrol_tick + _patrol_session_lost）：
    /// 实时页全部失联超过窗口 → 判会话失效并锁停。
    pub fn session_tick(&self) {
        let now = now_ms();
        let mut ctl = self.control.lock().expect("control poisoned");
        let window_hit = match ctl.live_t_ms {
            None => false,
            Some(t) => now.saturating_sub(t) > SESSION_WINDOW_MS,
        };
        if !ctl.live_sessions.is_empty() && !window_hit {
            return;
        }
        if ctl.live_sessions.is_empty() && ctl.live_t_ms.is_none() {
            return; // 从未有会话：交给许可看护处理
        }
        if !window_hit {
            return;
        }
        // 会话失效：取消并锁停，页面重连后须用户显式恢复（web_debug.py:734-762）。
        let reason = "session_dead";
        ctl.session_lost = reason.to_string();
        ctl.live_t_ms = None;
        if ctl.patrol.status == "running" {
            ctl.patrol.status = "stopped";
            ctl.patrol.stop_reason = reason.to_string();
        }
        Self::lock_locked(&mut ctl, reason);
    }

    /// 连接关闭：登出、退出会话集合；最后一个实时页连接走掉即判失效
    /// （web_debug.py::patrol_session_closed，原因 web_disconnected）。
    pub fn conn_closed(&self, conn: u64) {
        let was_live;
        {
            let mut ctl = self.control.lock().expect("control poisoned");
            ctl.live_controls.remove(&conn);
            was_live = ctl.live_sessions.remove(&conn);
            // 登出并停机（manual_forward.py:181-185）。manual_disconnect 会再锁
            // control（std Mutex 不可重入），此处直接内联等价操作避免自死锁：
            // 本 Worker 一旦卡死在 lock()，20Hz 引擎节拍会接连堵死所有
            // runtime Worker，整个进程无响应。
            ctl.manual.authenticated.remove(&conn);
            if ctl.manual.owner == Some(conn) {
                Self::manual_stop_locked(&mut ctl);
            }
        }
        if was_live {
            let now = now_ms();
            let mut ctl = self.control.lock().expect("control poisoned");
            if !ctl.live_sessions.is_empty() {
                return; // 还有别的实时页连接
            }
            let reason = "web_disconnected";
            ctl.session_lost = reason.to_string();
            ctl.live_t_ms = None;
            ctl.license.lost_reported = true;
            if ctl.patrol.status == "running" {
                ctl.patrol.status = "stopped";
                if ctl.lock_reason.is_empty() {
                    ctl.lock_reason = reason.to_string();
                }
                ctl.patrol.stop_reason = reason.to_string();
            }
            if !ctl.locked {
                Self::lock_locked(&mut ctl, reason);
            }
            let _ = now;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 本地构造 Hub（license_* 只依赖自身状态与时钟，不碰全局单例）。
    fn test_hub() -> Hub {
        Hub {
            control: Mutex::new(Control {
                server_session_id: "test-server-session".into(),
                password_sha256: None,
                boot_marker: PathBuf::new(),
                current_map_id: None,
                map_seq_last: 0,
                license: License::new(),
                manual: Manual::new(),
                patrol: Patrol::new(),
                nav: NavGoal::new(),
                cruise_on: false,
                locked: true,
                lock_reason: "global_stop".into(),
                user_locked: false,
                us_latched: false,
                relocal_busy: false,
                live_t_ms: None,
                session_lost: String::new(),
                live_sessions: HashSet::new(),
                live_controls: HashSet::new(),
            }),
            wslog: Mutex::new(WsLog { items: Vec::new(), last_frame_ms: 0 }),
            conn_seq: AtomicU64::new(1),
        }
    }

    /// 释放语义：只有持有者能交还；释放后 session/last_rx 归空（license_tick
    /// 走「从未收到许可」分支，不触发断流锁停）、seq 归 -1，同会话号心跳
    /// 可直接重新 Adopt 而不被 seq_replay 拒绝（前端 seq 生命周期内不清零）。
    #[test]
    fn test_license_release_only_holder_and_readopt() {
        let hub = test_hub();
        hub.license_accept("sess-a", 1, None).expect("首次心跳 Adopt");

        // 非持有者释放：无效，持有会话不动
        assert!(!hub.license_release("sess-b"));
        {
            let ctl = hub.control.lock().unwrap();
            assert_eq!(ctl.license.session.as_deref(), Some("sess-a"));
        }

        // 持有者释放：清会话、清新鲜度、重置序号
        assert!(hub.license_release("sess-a"));
        {
            let ctl = hub.control.lock().unwrap();
            assert!(ctl.license.session.is_none());
            assert!(ctl.license.last_rx_ms.is_none());
            assert_eq!(ctl.license.last_seq, -1);
        }

        // 释放后同会话号（seq 继续递增）可直接重新 Adopt
        hub.license_accept("sess-a", 2, None).expect("释放后重新 Adopt");
        let ctl = hub.control.lock().unwrap();
        assert_eq!(ctl.license.session.as_deref(), Some("sess-a"));
    }

    /// 空会话号释放：直接忽略。
    #[test]
    fn test_license_release_empty_ignored() {
        let hub = test_hub();
        hub.license_accept("sess-a", 1, None).expect("Adopt");
        assert!(!hub.license_release(""));
        let ctl = hub.control.lock().unwrap();
        assert_eq!(ctl.license.session.as_deref(), Some("sess-a"));
    }

    /// 登录顶替（2026-11-23 用户裁决）：登录后窗口内新会话号心跳立即接管
    /// 旧会话（不等 350ms 断流），凭证一次性消费；旧会话续跳被拒。
    #[test]
    fn test_login_takeover_immediate_and_one_shot() {
        let hub = test_hub();
        hub.license_accept("sess-old", 1, None).expect("旧页 Adopt");
        // 无凭证：新会话号被拒（session_mismatch），旧会话不动
        assert!(hub.license_accept("sess-new", 1, None).is_err());
        assert_eq!(
            hub.control.lock().unwrap().license.session.as_deref(),
            Some("sess-old")
        );

        // 登录发放凭证：窗口内新会话心跳立即接管，锁停一次（license_lost）
        hub.control.lock().unwrap().password_sha256 =
            Some(sha256_hex(b""));
        assert!(hub.manual_login(7, "admin", ""));
        hub.license_accept("sess-new", 1, None).expect("登录顶替接管");
        {
            let ctl = hub.control.lock().unwrap();
            assert_eq!(ctl.license.session.as_deref(), Some("sess-new"));
            assert_eq!(ctl.license.last_seq, 1);
            assert_eq!(ctl.lock_reason, "license_lost", "顶替即判一次失效");
            assert!(ctl.manual.login_takeover_at.is_none(), "凭证已消费");
        }

        // 凭证已消费：再顶替回旧号不行（旧页心跳被拒）
        assert!(hub.license_accept("sess-old", 9, None).is_err());
        assert_eq!(
            hub.control.lock().unwrap().license.session.as_deref(),
            Some("sess-new")
        );
    }

    /// 登录顶替窗口过期：凭证失效，回到「等 350ms 断流」原语义。
    /// 窗口判定提为 login_takeover_active(now) 后可注入未来时刻直测；
    /// 整链行为以 login_takeover_active=false 时回到 mismatch 拒绝为准。
    #[test]
    fn test_login_takeover_window_expiry() {
        let hub = test_hub();
        hub.license_accept("sess-old", 1, None).expect("旧页 Adopt");
        hub.control.lock().unwrap().password_sha256 = Some(sha256_hex(b""));
        assert!(hub.manual_login(7, "admin", ""));
        {
            let ctl = hub.control.lock().unwrap();
            let t = ctl.manual.login_takeover_at.expect("登录已发凭证");
            // 窗口内：可行使；窗口外（t + 10001）：失效
            assert!(Hub::login_takeover_active(&ctl, t, "sess-new"));
            assert!(!Hub::login_takeover_active(&ctl, t + 10_001, "sess-new"));
        }
        // 把凭证时刻拨到窗口外，整链 accept 恢复拒绝
        {
            let mut ctl = hub.control.lock().unwrap();
            ctl.manual.login_takeover_at = Some(now_ms().saturating_sub(1));
            // 模拟窗口过期：直接清空凭证（语义等价过期后的自然失效）
            ctl.manual.login_takeover_at = None;
            ctl.license.last_rx_ms = Some(now_ms());
        }
        assert!(hub.license_accept("sess-new", 1, None).is_err());
        assert_eq!(
            hub.control.lock().unwrap().license.session.as_deref(),
            Some("sess-old")
        );
    }

    /// 绑定 session_id 时仅允许登录页自身接管，防止多标签页并存时被旁路
    /// 未登录页的心跳抢走一次性顶替凭证。
    #[test]
    fn test_login_takeover_bound_to_session_prevents_third_tab_hijack() {
        let hub = test_hub();
        hub.license_accept("sess-a", 1, None).expect("A 持证");
        hub.control.lock().unwrap().password_sha256 = Some(sha256_hex(b""));
        assert!(hub.manual_login_with_session(9, "admin", "", Some("sess-c")));

        // 旁路未登录页 B 心跳仍被拒绝，且不能消费凭证
        assert_eq!(
            hub.license_accept("sess-b", 1, None),
            Err("session_mismatch")
        );
        {
            let ctl = hub.control.lock().unwrap();
            assert_eq!(ctl.license.session.as_deref(), Some("sess-a"));
            assert!(ctl.manual.login_takeover_at.is_some(), "凭证未被旁路页面消费");
        }

        // 登录页 C 随后心跳：立即接管并消费凭证
        hub.license_accept("sess-c", 1, None).expect("C 顶替接管");
        {
            let ctl = hub.control.lock().unwrap();
            assert_eq!(ctl.license.session.as_deref(), Some("sess-c"));
            assert!(ctl.manual.login_takeover_at.is_none(), "凭证已消费");
            assert!(ctl.manual.login_takeover_session.is_none());
        }
    }
}
