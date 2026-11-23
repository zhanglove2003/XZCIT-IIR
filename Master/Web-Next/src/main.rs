//! C50C Web 控制台重写（方案 B）——Rust axum 后端装配。
//!
//! 蓝图：§4.1 启动参数与环境变量、§4.2 HTTP 端点、§4.3 双通道 WebSocket。
//! 前端静态资源构建期经 include_str! 编进单一可执行文件，运行时零外部
//! 网络依赖、离线局域网直开。

mod audio;
mod fake;
mod mic;
mod netinfo;
mod ros;
mod safety;
mod speaker;
mod state;
mod sysinfo;
mod ws_control;
mod ws_video;

use std::path::PathBuf;
use std::sync::Arc;

use axum::http::{header, StatusCode};
use axum::response::IntoResponse;
use axum::routing::get;
use axum::Router;


/// `OPI_WEB_PORT` 默认端口（蓝图 §4.1）。
const DEFAULT_PORT: u16 = 8088;
/// `OPI_BOOT_MARKER` 默认路径（蓝图 §4.1：开机 20 秒静止倒计时标记文件）。
const DEFAULT_BOOT_MARKER: &str = ".cache/opi-robot/boot_hold";

/// 装配配置：CLI 参数与环境变量合并后的最终值。
#[derive(Debug, Clone)]
pub struct Config {
    /// `--fake` 或 `OPI_WEB_FAKE=1`：启用内置合成数据源，不连接 ROS 2。
    pub fake: bool,
    /// `--port` 或 `OPI_WEB_PORT`，默认 8088。
    pub port: u16,
    /// `OPI_WEB_PASSWORD_SHA256`：admin 密码的 SHA-256 小写十六进制摘要。
    /// 未设置时拒绝一切登录（蓝图 §4.1）。
    pub password_sha256: Option<String>,
    /// `OPI_BRIDGE_TOKEN`：/bridge 两端点的共享令牌（≥32 hex 随机）。
    /// 未设置时 ros 模式端点 fail-closed（全部 401）——Web-Next 绑
    /// 0.0.0.0，无令牌的裸端点会被同网段任意主机伪造（上行注入假
    /// scan/pose/chassis，下行偷走 nav_goal 命令驱动实车）。
    pub bridge_token: Option<String>,
    /// `OPI_BOOT_MARKER`：开机静止倒计时标记文件路径。
    pub boot_marker: PathBuf,
}

impl Config {
    fn from_env_and_args() -> Config {
        let mut fake = std::env::var("OPI_WEB_FAKE").is_ok_and(|v| v == "1");
        let mut port = std::env::var("OPI_WEB_PORT")
            .ok()
            .and_then(|v| v.parse::<u16>().ok())
            .unwrap_or(DEFAULT_PORT);
        // CLI 参数覆盖环境变量：--fake 只置位，--port 只在合法时生效。
        let args: Vec<String> = std::env::args().collect();
        let mut i = 1;
        while i < args.len() {
            match args[i].as_str() {
                "--fake" => fake = true,
                "--port" => {
                    i += 1;
                    if let Some(v) = args.get(i) {
                        match v.parse::<u16>() {
                            Ok(p) => port = p,
                            Err(_) => eprintln!("--port: 非法端口 {v:?}，忽略"),
                        }
                    }
                }
                other => eprintln!("未知参数 {other:?}，忽略（支持：--fake --port <n>）"),
            }
            i += 1;
        }
        let password_sha256 =
            std::env::var("OPI_WEB_PASSWORD_SHA256").ok().filter(|v| !v.is_empty());
        let bridge_token = std::env::var("OPI_BRIDGE_TOKEN").ok().filter(|v| !v.is_empty());
        let boot_marker = std::env::var("OPI_BOOT_MARKER")
            .map(PathBuf::from)
            .unwrap_or_else(|_| {
                let home = std::env::var("HOME").unwrap_or_default();
                PathBuf::from(home).join(DEFAULT_BOOT_MARKER)
            });
        Config { fake, port, password_sha256, bridge_token, boot_marker }
    }
}

// ---------------------------------------------------------------------------
// 内置静态资源：构建期经 include_str! 编进单一可执行文件，离线局域网直开。
// ---------------------------------------------------------------------------

const INDEX_HTML: &str = include_str!("../web/index.html");
const TOKENS_CSS: &str = include_str!("../web/styles/tokens.css");
const LAYOUT_CSS: &str = include_str!("../web/styles/layout.css");
const MICRO_CSS: &str = include_str!("../web/styles/micro.css");
const APP_JS: &str = include_str!("../web/js/app.js");
const STORE_JS: &str = include_str!("../web/js/store.js");
const SCHEDULER_JS: &str = include_str!("../web/js/scheduler.js");
const WS_JS: &str = include_str!("../web/js/ws.js");
const CANVAS_MAP_JS: &str = include_str!("../web/js/canvas-map.js");
const CANVAS_RADAR_JS: &str = include_str!("../web/js/canvas-radar.js");
const SPOTLIGHT_JS: &str = include_str!("../web/js/spotlight.js");

const NO_STORE: &str = "no-store";

const CSS_TYPE: &str = "text/css; charset=utf-8";
const JS_TYPE: &str = "text/javascript; charset=utf-8";
const FONT_TYPE: &str = "font/woff2";

// 主字体 HarmonyOS Sans SC（免费商用）：全量 woff2 构建期编进二进制，
// 经 /fonts/*.woff2 下发（内容随版本走，URL 不变 → immutable 长缓存）。
const FONT_REGULAR: &[u8] = include_bytes!("../web/fonts/HarmonyOSSansSC-Regular.woff2");
const FONT_MEDIUM: &[u8] = include_bytes!("../web/fonts/HarmonyOSSansSC-Medium.woff2");
const FONT_BOLD: &[u8] = include_bytes!("../web/fonts/HarmonyOSSansSC-Bold.woff2");

async fn index() -> impl IntoResponse {
    (
        [(header::CONTENT_TYPE, "text/html; charset=utf-8"), (header::CACHE_CONTROL, NO_STORE)],
        INDEX_HTML,
    )
}

async fn favicon() -> impl IntoResponse {
    ([(header::CACHE_CONTROL, NO_STORE)], StatusCode::NO_CONTENT)
}

fn static_asset(body: &'static str, content_type: &'static str) -> impl IntoResponse {
    (
        [(header::CONTENT_TYPE, content_type), (header::CACHE_CONTROL, NO_STORE)],
        body,
    )
}

/// 字体等版本化产物：immutable 长缓存（二进制更新 = 内容更新，浏览器按
/// 强刷兜底即可）。
fn static_bytes(body: &'static [u8], content_type: &'static str) -> impl IntoResponse {
    (
        [
            (header::CONTENT_TYPE, content_type),
            (header::CACHE_CONTROL, "public, max-age=31536000, immutable"),
        ],
        body,
    )
}

/// `/api/state`：返回当前 `status_payload` 快照（与 `/ws` 推送的 `{t:"status"}` 同构）。
async fn api_state() -> impl IntoResponse {
    let payload = state::global().snapshot();
    let body = serde_json::to_string(&payload).unwrap_or_else(|_| "{}".into());
    (
        [(header::CONTENT_TYPE, "application/json"), (header::CACHE_CONTROL, NO_STORE)],
        body,
    )
}

/// `/api/wslog`：最近 50 条控制帧收发环形日志（`{last_frame_age, items}`，
/// 脱敏密码字段；web_debug.py::ws_log_snapshot）。
async fn api_wslog() -> impl IntoResponse {
    let body = safety::global().ws_log_snapshot().to_string();
    (
        [(header::CONTENT_TYPE, "application/json"), (header::CACHE_CONTROL, NO_STORE)],
        body,
    )
}

#[tokio::main]
async fn main() {
    let cfg = Arc::new(Config::from_env_and_args());
    // 鉴权要求：未设置 OPI_WEB_PASSWORD_SHA256 时拒绝一切登录（蓝图 §4.1）。
    if cfg.password_sha256.is_none() {
        eprintln!("警告：OPI_WEB_PASSWORD_SHA256 未设置，登录通道将拒绝一切请求");
    }
    let mode: &'static str = if cfg.fake { "fake" } else { "ros" };
    let store = state::init_global(mode);
    // 控制中枢（许可/手动/锁停/巡逻/导航/环形日志）先于数据源装配。
    safety::init_hub(&cfg);
    eprintln!("srv_epoch={} mode={mode} port={}", store.snapshot().srv_epoch, cfg.port);

    // --fake 时启动 FakeEngine 仿真源（20Hz 底盘 / 10Hz 雷达 / 5Hz 相机 /
    // 1Hz 地图 / 5Hz 位姿）；ros 模式启动 /bridge 桥接层与 20Hz 看护装配
    // 任务（蓝图 §3 实线：ROS 2 Bridge 经垫片上报帧）。
    fake::maybe_start(&cfg).await;
    ros::maybe_start(&cfg).await;
    // WiFi/网络信息采集器（ros 模式启动后台任务，fake 模式内部跳过）。
    netinfo::maybe_start(&cfg).await;
    // 音频设备探测（喇叭/板载麦克风挂 es8388 编解码器；同 netinfo，fake 跳过）。
    audio::maybe_start(&cfg).await;
    // 系统占用采集（CPU/GPU/内存/磁盘；同 netinfo，fake 跳过）。
    sysinfo::maybe_start(&cfg).await;

    let mut app = Router::new()
        .route("/", get(index))
        .route("/index.html", get(index))
        .route("/api/state", get(api_state))
        .route("/api/wslog", get(api_wslog))
        .route("/favicon.ico", get(favicon))
        .route("/styles/tokens.css", get(|| async { static_asset(TOKENS_CSS, CSS_TYPE) }))
        .route("/styles/layout.css", get(|| async { static_asset(LAYOUT_CSS, CSS_TYPE) }))
        .route("/styles/micro.css", get(|| async { static_asset(MICRO_CSS, CSS_TYPE) }))
        .route("/js/app.js", get(|| async { static_asset(APP_JS, JS_TYPE) }))
        .route("/js/store.js", get(|| async { static_asset(STORE_JS, JS_TYPE) }))
        .route("/js/scheduler.js", get(|| async { static_asset(SCHEDULER_JS, JS_TYPE) }))
        .route("/js/ws.js", get(|| async { static_asset(WS_JS, JS_TYPE) }))
        .route("/js/canvas-map.js", get(|| async { static_asset(CANVAS_MAP_JS, JS_TYPE) }))
        .route("/js/canvas-radar.js", get(|| async { static_asset(CANVAS_RADAR_JS, JS_TYPE) }))
        .route("/js/spotlight.js", get(|| async { static_asset(SPOTLIGHT_JS, JS_TYPE) }))
        .route("/fonts/HarmonyOSSansSC-Regular.woff2", get(|| async { static_bytes(FONT_REGULAR, FONT_TYPE) }))
        .route("/fonts/HarmonyOSSansSC-Medium.woff2", get(|| async { static_bytes(FONT_MEDIUM, FONT_TYPE) }))
        .route("/fonts/HarmonyOSSansSC-Bold.woff2", get(|| async { static_bytes(FONT_BOLD, FONT_TYPE) }))
        .route("/ws", get(ws_control::endpoint))
        .route("/ws/video", get(ws_video::endpoint));
    // /bridge 两端点仅 ros 模式挂载：fake 模式路由零变化（契约
    // endpointContract：两 endpoint 挂既有 Router，仅 ros 模式暴露）。
    if !cfg.fake {
        app = app.merge(ros::router(&cfg));
    }

    let listener = tokio::net::TcpListener::bind(("0.0.0.0", cfg.port))
        .await
        .unwrap_or_else(|e| {
            panic!("监听 0.0.0.0:{} 失败：{e}（8088 被旧服务占用时用 --port 换端口）", cfg.port)
        });
    eprintln!("C50C Web-Next 监听 http://0.0.0.0:{}（Ctrl-C 退出）", cfg.port);
    axum::serve(listener, app).await.expect("HTTP 服务异常退出");
}
