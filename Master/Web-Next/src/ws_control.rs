//! `/ws` 控制与状态通道（蓝图 §4.3）。
//!
//! 握手后立即下发 `motion_license_session`（会话号 + 服务端时钟锚点），
//! 随后按 20Hz（50ms）推送 `{t:"status"}` 完整快照；上行消息处理与回执
//! 逐字对齐蓝图 §4.3(1) 与旧 web_debug.py::_read_view/_manual_message。
//!
//! 发送语义（web_debug.py::_send_text/_send_display 的对应物）：
//! - 控制回执（登录/手动确认/patrol_status/nav_status）：必达，走有界队列
//!   的 await 发送；
//! - 状态帧：可丢，队列满即丢弃本帧，绝不阻塞控制消息的读取。
//!
//! 环形日志脱敏与噪音抑制：
//! - `manual_login` 只记字段名清单，不落密码（web_debug.py:2602-2605）；
//! - `ping` 不进日志（web_debug.py:2668-2671「进日志全是噪音」）；
//! - `motion_license`（100ms Worker 心跳）不进环形日志——10Hz 心跳会把
//!   50 条环形日志在 5 秒内刷穿，让「排查点了没反应」失去意义；但
//!   `last_frame_age` 仍然统计它（它是连接活性的真实信号）。

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::response::Response;
use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::sync::mpsc::{self, Sender};

use crate::safety::{self, Hub};
use crate::state;

/// 状态推送周期（ms）。蓝图 §4.3 序列图：每 50ms（20Hz）。
const STATUS_PERIOD_MS: u64 = 50;
/// 出站队列容量：控制回执 + 少量状态帧的缓冲上限。
const OUTBOUND_CAP: usize = 64;
/// 控制回执单次发送可写等待时限（ms）。健康客户端始终在读取，正常发送
/// 是微秒级；客户端长期不读（标签页挂起）时 TCP 缓冲堆满、队列满，
/// 若 await 无限等待会卡死本连接的整条控制处理循环（含 motion_license
/// 处理），许可被误判断流。超时即视为链路已死，断开由页面自动重连
/// （与 /ws/video 的慢客户端降级同一哲学）。
const CONTROL_SEND_TIMEOUT_MS: u64 = 2000;

/// `/ws` 端点。
pub async fn endpoint(upgrade: WebSocketUpgrade) -> Response {
    upgrade.on_upgrade(|socket| handle_socket(socket))
}

/// 每连接的会话状态。
struct ConnState {
    conn: u64,
    live: bool,
    page: Option<String>,
    /// 控制页未登录提示只发一次（web_debug.py:2620-2628）。
    refused_once: bool,
    /// 最近一次收到任何消息的单调时刻（CLIENT_TIMEOUT 判定）。
    last_msg_ms: u64,
}

pub async fn handle_socket(socket: WebSocket) {
    let hub: &'static Hub = safety::global();
    let conn = hub.next_conn_id();
    let (tx, mut rx) = mpsc::channel::<Message>(OUTBOUND_CAP);
    let (mut sink, mut stream) = socket.split();

    // 握手后立即下发会话帧（蓝图 §4.3 序列图第一步）。
    let (server_session, clock_ms) = {
        let ctl = hub.control.lock().expect("control poisoned");
        (ctl.server_session_id.clone(), safety::now_ms())
    };
    let session_hello = json!({
        "t": "motion_license_session",
        "session_id": server_session,
        "clock_ms": clock_ms,
    })
    .to_string();
    if sink.send(Message::Text(session_hello.into())).await.is_err() {
        return;
    }

    // 写任务：出站队列 → 套接字。对端读不动时带时限发送，超时断开，
    // 绝不让一个挂起的客户端卡死控制循环（ws_control「必达」只对健康
    // 链路承诺——死链上的必达没有意义，页面会自动重连）。
    let writer = tokio::spawn(async move {
        while let Some(msg) = rx.recv().await {
            let sent = tokio::time::timeout(
                std::time::Duration::from_millis(CONTROL_SEND_TIMEOUT_MS),
                sink.send(msg),
            )
            .await;
            if !matches!(sent, Ok(Ok(()))) {
                eprintln!("控制客户端 {conn} 发送超时（对端长期不读），主动断开");
                break;
            }
        }
    });

    let mut state = ConnState {
        conn,
        live: false,
        page: None,
        refused_once: false,
        last_msg_ms: safety::now_ms(),
    };
    let mut status_ticker =
        tokio::time::interval(std::time::Duration::from_millis(STATUS_PERIOD_MS));
    status_ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

    loop {
        tokio::select! {
            // 上行控制消息。
            incoming = stream.next() => {
                let Some(msg) = incoming else { break };
                let Ok(msg) = msg else { break };
                match msg {
                    Message::Text(text) => {
                        state.last_msg_ms = safety::now_ms();
                        hub.ws_touch_frame();
                        process_text(hub, &mut state, &text, &tx).await;
                    }
                    Message::Close(_) => break,
                    _ => {}
                }
            }
            // 20Hz 状态推送：可丢帧，不阻塞上行处理。
            _ = status_ticker.tick() => {
            // 控制消息停了（页面崩溃/挂起）：退出实时会话（web_debug.py:2654-2663）。
            if state.live
                && safety::now_ms().saturating_sub(state.last_msg_ms) > safety::CLIENT_TIMEOUT_MS
            {
                state.live = false;
                hub.conn_closed(state.conn);
                // 修复：CLIENT_TIMEOUT 强制登出必须通知前端——conn_closed 只删
                // 服务端会话，socket 仍开，前端 ui.authed 仍为 true、登录表单
                // 保持隐藏，所有控制操作都被服务端拒绝。后台节流页签的 view
                // 心跳可能 60s 才来一条，被动恢复太慢，这里立即下发登出通知
                // （复用 manual_error：前端按「请先登录」正则 setAuthed(false)）。
                send_manual_error(&tx, "心跳超时已登出，请先登录手动控制").await;
            }
                let payload = state::global().snapshot();
                if let Ok(text) = serde_json::to_string(&payload) {
                    // 状态是小体积显示数据：客户端收不动时丢弃本帧。
                    let _ = tx.try_send(Message::Text(text.into()));
                }
            }
        }
    }

    drop(tx);
    let _ = writer.await;
    hub.conn_closed(state.conn);
}

/// 解析并处理一条文本帧；回执在处理函数内直接入队。
async fn process_text(hub: &'static Hub, state: &mut ConnState, text: &str, tx: &Sender<Message>) {
    let Ok(msg) = serde_json::from_str::<Value>(text) else {
        return;
    };
    if !msg.is_object() {
        return;
    }
    let Some(kind) = msg.get("t").and_then(Value::as_str) else {
        return;
    };
    process_msg(hub, state, kind, &msg, tx).await;
}

async fn process_msg(
    hub: &'static Hub,
    state: &mut ConnState,
    kind: &str,
    msg: &Value,
    tx: &Sender<Message>,
) {
    match kind {
        "ping" => {
            // RTT 探测：原样回显时间戳。不鉴权、不进 ws 日志。
            let pong = json!({ "t": "pong", "ts": msg.get("ts").cloned().unwrap_or(Value::Null) });
            send_reply(tx, pong, false).await;
            return;
        }
        "motion_license" => {
            handle_license(hub, state, msg, tx).await;
            return;
        }
        "motion_license_release" => {
            // 页面隐藏/后台化：持有者主动交还许可。与心跳同权：不鉴权、
            // 不进环形日志（10Hz 心跳同款理由，release 至多每可见性翻转
            // 一次，但不给它特例）。
            handle_license_release(hub, msg).await;
            return;
        }
        _ => {}
    }

    // 其余控制帧进环形日志（manual_login 脱敏：只记字段名清单）。
    if kind == "manual_login" {
        let mut fields: Vec<String> =
            msg.as_object().map(|o| o.keys().cloned().collect()).unwrap_or_default();
        fields.sort();
        hub.ws_log_add("rx", json!({ "t": kind, "fields": fields }));
    } else {
        hub.ws_log_add("rx", msg.clone());
    }

    match kind {
        "view" => handle_view(hub, state, msg, tx).await,
        "manual_login" => handle_login(hub, state, msg, tx).await,
        "manual_start" => handle_manual_start(hub, state, msg, tx).await,
        "manual_heartbeat" => {
            // 被顶替（本连接不再是 owner）时返回提示文案，由调用方补发
            // manual_error——否则另一页接管后本页还傻等，全程静默。
            if let Some(warn) = hub.manual_heartbeat(state.conn) {
                send_manual_error(tx, &warn).await;
            }
        }
        "manual_stop" => {
            if let Err(e) = hub.manual_stop(state.conn) {
                send_manual_error(tx, &e).await;
            }
        }
        "cruise_set" => handle_cruise_set(hub, state, msg, tx).await,
        "ultrasonic_rearm" => handle_ultrasonic_rearm(hub, state, tx).await,
        "sensor_restart" => handle_sensor_restart(hub, state, msg, tx).await,
        "speaker_test" => handle_speaker_test(hub, state, tx).await,
        "speaker_volume" => handle_speaker_volume(hub, state, msg, tx).await,
        "mic_record_start" => handle_mic_record_start(hub, state, tx).await,
        "mic_record_stop" => handle_mic_record_stop(hub, state, tx).await,
        "mic_play" => handle_mic_play(hub, state, tx).await,
        "relocalize" => handle_relocalize(hub, state, msg, tx).await,
        "patrol_save" => handle_patrol_save(hub, state, msg, tx).await,
        "patrol_start" => handle_patrol_start(hub, state, tx).await,
        "motion_stop" => handle_motion_stop(hub, state, tx).await,
        "motion_rearm" => handle_motion_rearm(hub, state, tx).await,
        "nav_goal" => handle_nav_goal(hub, state, msg, tx).await,
        "nav_cancel" => handle_nav_cancel(hub, state, tx).await,
        _ => {}
    }
}

async fn send_reply(tx: &Sender<Message>, value: Value, log: bool) {
    let text = value.to_string();
    if log {
        let detail = if value.get("t").and_then(Value::as_str) == Some("patrol_status") {
            json!({ "t": "patrol_status", "ok": value.get("ok") })
        } else {
            value.clone()
        };
        safety::global().ws_log_add("tx", detail);
    }
    let _ = tx.send(Message::Text(text.into())).await;
}

async fn send_manual_error(tx: &Sender<Message>, message: &str) {
    send_reply(tx, json!({ "t": "manual_error", "message": message }), true).await;
}

/// `motion_license`：100ms Worker 心跳。不要求登录——许可只决定
/// 「下游是否认为网页还活着」（web_debug.py:2673-2680）。
async fn handle_license(hub: &'static Hub, state: &ConnState, msg: &Value, tx: &Sender<Message>) {
    // 蓝图 §4.3(1)：session_id 为字符串；兼容旧协议的整数 session。
    let session_id = msg
        .get("session_id")
        .and_then(Value::as_str)
        .map(str::to_string)
        .or_else(|| {
            msg.get("session")
                .and_then(Value::as_i64)
                .filter(|s| *s > 0)
                .map(|s| s.to_string())
        })
        .unwrap_or_default();
    let seq = match msg.get("seq") {
        Some(Value::Number(n)) => n.as_i64().unwrap_or(-1),
        _ => -1,
    };
    let ttl_ms = msg.get("ttl_ms").and_then(Value::as_u64);
    hub.live_touch();
    let _ = state; // 许可帧不改变 live 状态
    match hub.license_accept(&session_id, seq, ttl_ms) {
        Ok(()) => {
            // ros 模式：被接受的许可 = license_forward（原文转发，不许自造
            // 号码）+ guard_authorize(True)（按值去重）——web_debug.py:640,
            // 655,900-924。fake 模式无下游通道，跳过。
            if crate::ros::enabled() {
                crate::ros::on_license_accepted(
                    &session_id,
                    seq,
                    ttl_ms.unwrap_or(safety::LICENSE_TTL_MS),
                );
            }
            let clock_ms = safety::now_ms();
            send_reply(
                tx,
                json!({
                    "t": "motion_license_session",
                    "session_id": session_id,
                    "clock_ms": clock_ms,
                }),
                false,
            )
            .await;
        }
        Err(reason) => {
            let mut nak = json!({
                "t": "motion_license_nak",
                "reason": reason,
                "session_id": session_id,
            });
            if reason == "session_mismatch" {
                // 告诉被拒页面「服务器现在认哪个号」：本页是被顶替的一方
                // （web_debug.py::_session_msg_text）。
                let owner = {
                    let ctl = hub.control.lock().expect("control poisoned");
                    ctl.license.session.clone()
                };
                nak["owner"] = owner.map(Value::String).unwrap_or(Value::Null);
            }
            send_reply(tx, nak, false).await;
        }
    }
}

/// `motion_license_release`：页面隐藏时主动交还许可（许可权随前台页）。
/// 只清当前持有会话；不是持有者或会话号为空则静默忽略——不回 nak，
/// 隐藏页此刻正在断开连接，回复无人收。下游不转发「释放」：fusion
/// 侧有自己的 350ms 断流超时，心跳流停了它自己走 report_lost 锁停
/// （与页面关闭同路径）。
async fn handle_license_release(hub: &'static Hub, msg: &Value) {
    let session_id = msg
        .get("session_id")
        .and_then(Value::as_str)
        .unwrap_or_default();
    hub.live_touch();
    hub.license_release(session_id);
}

/// `view`：页面心跳（1Hz）。先处理 view 再处理手动消息的顺序由
/// 上层调用顺序保证（web_debug.py:2608-2636）。
async fn handle_view(hub: &'static Hub, state: &mut ConnState, msg: &Value, tx: &Sender<Message>) {
    let page = msg.get("page").and_then(Value::as_str).unwrap_or("idle").to_string();
    if page == "control" && !hub.manual_logged_in(state.conn) {
        // 控制页未登录：提示一次，live 保持 false。
        if state.live || !state.refused_once {
            state.refused_once = true;
            send_manual_error(tx, "请先登录手动控制").await;
        }
        state.live = false;
        state.page = Some(page);
        return;
    }
    let (live, _control_page) = hub.view_touch(state.conn, &page);
    state.live = live;
    state.page = Some(page.clone());
    state.refused_once = false;
    if !live {
        // 离开实时页：手动会话按原语义断开（web_debug.py:2631-2632）。
        hub.manual_disconnect(state.conn);
    }
}

/// `manual_login`：校验 admin + SHA-256，回发 manual_auth。
async fn handle_login(hub: &'static Hub, state: &mut ConnState, msg: &Value, tx: &Sender<Message>) {
    let username = msg.get("username").and_then(Value::as_str).unwrap_or("");
    let password = msg.get("password").and_then(Value::as_str).unwrap_or("");
    let session_id = msg.get("session_id").and_then(Value::as_str);
    let ok = hub.manual_login_with_session(state.conn, username, password, session_id);
    send_reply(tx, json!({ "t": "manual_auth", "ok": ok }), true).await;
    if ok && state.page.as_deref() == Some("control") {
        // 蓝图语义「已登录 + 在控制页」即刻成立：登录成功时若本连接已经
        // 在控制页心跳，直接登记实时会话（旧实现要等下一个 1s view 心跳）。
        hub.view_touch(state.conn, "control");
        state.live = true;
    }
}

/// `manual_start`：接管底盘控制（自动取消正在进行的巡航/导航）。
async fn handle_manual_start(hub: &'static Hub, state: &ConnState, msg: &Value, tx: &Sender<Message>) {
    let speed = msg.get("speed").and_then(Value::as_f64).unwrap_or(f64::NAN);
    let mode = msg.get("mode").and_then(Value::as_str).unwrap_or("");
    let duration = msg.get("duration").and_then(Value::as_f64);
    let direction = msg.get("direction").and_then(Value::as_str).unwrap_or("forward");
    match hub.manual_start(state.conn, speed, mode, duration, direction, state.live) {
        Ok(()) => {
            // ros 模式：手动接管成功即 flush 最新 (vx,wz)（旧 manual_forward
            // start 里 publish(*command) 立即发第一条，manual_forward.py:141）。
            if crate::ros::enabled() {
                crate::ros::flush_manual_cmd();
            }
            send_reply(
                tx,
                json!({ "t": "manual_started", "id": msg.get("id").cloned().unwrap_or(Value::Null) }),
                true,
            )
            .await;
        }
        Err(e) => send_manual_error(tx, &e).await,
    }
}

/// `cruise_set`：切换自动避障巡航开关。蓝图 §4.3(1)：已登录 + control 页。
async fn handle_cruise_set(hub: &'static Hub, state: &ConnState, msg: &Value, tx: &Sender<Message>) {
    if !hub.manual_authenticated(state.conn) {
        send_manual_error(tx, "请先登录手动控制").await;
        return;
    }
    if !state.live || state.page.as_deref() != Some("control") {
        send_manual_error(tx, "请先进入手动控制页面").await;
        return;
    }
    match msg.get("on") {
        Some(Value::Bool(on)) => {
            if crate::ros::enabled() {
                // ros 模式：下发经垫片并按 cmd_ack 如实回显；失败不落状态
                // （契约 commandMap cruise_set 行：通道不可用→manual_error）。
                if let Err(e) = crate::ros::cruise_set_ros(*on).await {
                    send_manual_error(tx, &e).await;
                }
            } else if let Err(e) = hub.set_cruise(*on) {
                send_manual_error(tx, &e).await;
            }
        }
        _ => send_manual_error(tx, "巡航开关缺少 on 布尔字段").await,
    }
}

/// 驾驶类消息统一鉴权（web_debug.py::_require_drive_auth）：
/// 已登录 + 在实时页（控制页或地图页）。
fn require_drive_auth(hub: &'static Hub, state: &ConnState) -> Result<(), &'static str> {
    if !hub.manual_authenticated(state.conn) {
        return Err("请先登录手动控制");
    }
    if !state.live {
        return Err("请先进入实时页面（控制页或地图页）");
    }
    Ok(())
}

async fn handle_ultrasonic_rearm(hub: &'static Hub, state: &ConnState, tx: &Sender<Message>) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_reply(
            tx,
            json!({ "t": "manual_error", "ok": false, "message": e }),
            true,
        )
        .await;
        return;
    }
    let (ok, info) = if crate::ros::enabled() {
        // ros 模式：发布经垫片并按 cmd_ack 回报；失败如实显示，不假装已
        // 恢复（web_debug.py:564-580）。
        crate::ros::ultrasonic_rearm_ros().await
    } else {
        hub.ultrasonic_rearm()
    };
    let kind = if ok { "ultrasonic_status" } else { "manual_error" };
    send_reply(tx, json!({ "t": kind, "ok": ok, "message": info }), true).await;
}

/// `sensor_restart`：重启传感器对应的后台服务（白名单制，目标必须映射到
/// 确切的 systemd --user 单元）。相机/雷达同属 opi-robot.service 进程组
/// （start_web.sh 幂等拉起相机/雷达/融合/SLAM），底盘是独立单元
/// c50c-chassis.service（ExecStartPre sleep 20，重启约 20-30s 恢复）。
/// 喇叭/矩阵麦克风没有守护进程——如实拒绝，不假装重启。fake 模式下本机
/// 没有这些单元，systemctl 报错原样回显（不冒充成功）。
async fn handle_sensor_restart(
    hub: &'static Hub,
    state: &ConnState,
    msg: &Value,
    tx: &Sender<Message>,
) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_manual_error(tx, e).await;
        return;
    }
    let sensor = msg.get("sensor").and_then(Value::as_str).unwrap_or("");
    let (unit, label) = match sensor {
        "camera" => ("opi-robot.service", "相机（视觉/雷达进程组）"),
        "lidar" => ("opi-robot.service", "雷达（视觉/雷达进程组）"),
        "chassis" => ("c50c-chassis.service", "底盘桥"),
        _ => {
            send_reply(
                tx,
                json!({ "t": "sensor_status", "ok": false,
                        "message": "该设备没有后台服务，无法重启" }),
                true,
            )
            .await;
            return;
        }
    };
    let tx = tx.clone();
    let unit = unit.to_string();
    let label = label.to_string();
    tokio::spawn(async move {
        // spawn_blocking 跑 systemctl（阻塞最长 40s：底盘 ExecStartPre 20s
        // + 启动），外层 45s 超时兜底，绝不悬挂 ws 任务。
        let job = tokio::task::spawn_blocking(move || {
            std::process::Command::new("systemctl")
                .args(["--user", "restart", &unit])
                .output()
        });
        let (ok, message) = match tokio::time::timeout(
            std::time::Duration::from_secs(45),
            job,
        )
        .await
        {
            // 三层 Result：外层超时 → spawn_blocking JoinError → 命令执行结果
            Err(_) => (false, format!("{label}重启超时（45s），请到板上查看服务状态")),
            Ok(Err(e)) => (false, format!("{label}重启失败：{e}")),
            Ok(Ok(Err(e))) => (false, format!("{label}重启失败：{e}")),
            Ok(Ok(Ok(out))) if out.status.success() => (true, format!("{label}服务已重启")),
            Ok(Ok(Ok(out))) => (
                false,
                format!(
                    "{label}重启失败：{}",
                    String::from_utf8_lossy(&out.stderr).trim()
                ),
            ),
        };
        send_reply(&tx, json!({ "t": "sensor_status", "ok": ok, "message": message }), true)
            .await;
    });
}

/// `speaker_test`：车体喇叭放三声 880 Hz 提示音。走 raw ALSA（aplay：
/// 板上没有 PulseAudio/PipeWire），阻塞 ~0.6 s，放 spawn_blocking 里不占
/// ws 任务。fake 模式没有声卡硬件也要真响（本机若有 aplay 就照放，失败
/// 原样回错，不冒充成功）。
async fn handle_speaker_test(hub: &'static Hub, state: &ConnState, tx: &Sender<Message>) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_manual_error(tx, e).await;
        return;
    }
    let tx = tx.clone();
    tokio::spawn(async move {
        let job = tokio::task::spawn_blocking(crate::speaker::play_test_tone);
        let (ok, message) = match job.await {
            Err(e) => (false, format!("喇叭测试失败：{e}")),
            Ok(Err(e)) => (false, format!("喇叭测试失败：{e}")),
            Ok(Ok(v)) => {
                let dev = v.get("device").and_then(Value::as_str).unwrap_or("?");
                (true, format!("已播放三声测试音（{dev}）"))
            }
        };
        send_reply(&tx, json!({ "t": "sensor_status", "ok": ok, "message": message }), true)
            .await;
    });
}

/// `speaker_volume`：调喇叭音量（0–100 %，映射到 es8388 的 `PCM` 混音
/// 控件）。每次都真写 amixer 并回读，回包带 `volume` 供滑块校正。
async fn handle_speaker_volume(hub: &'static Hub, state: &ConnState, msg: &Value, tx: &Sender<Message>) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_manual_error(tx, e).await;
        return;
    }
    let Some(pct) = msg.get("volume").and_then(Value::as_u64) else {
        send_manual_error(tx, "音量参数缺失或非法").await;
        return;
    };
    let pct = pct.min(100) as u32;
    let tx = tx.clone();
    tokio::spawn(async move {
        let job = tokio::task::spawn_blocking(move || crate::speaker::set_volume(pct));
        let (ok, message, volume) = match job.await {
            Err(e) => (false, format!("音量设置失败：{e}"), Value::Null),
            Ok(Err(e)) => (false, format!("音量设置失败：{e}"), Value::Null),
            Ok(Ok(v)) => (true, format!("音量已调到 {v}%"), json!(v)),
        };
        send_reply(
            &tx,
            json!({ "t": "sensor_status", "ok": ok, "message": message, "volume": volume }),
            true,
        )
        .await;
    });
}

fn mic_stop_message(info: &crate::mic::StopInfo, auto_stopped: bool) -> String {
    let low_hint =
        if info.peak < crate::mic::NORM_MIN_PEAK { "，音量极低请靠近说话" } else { "" };
    if auto_stopped {
        format!("录音已达上限并自动结束（{:.1}s{low_hint}）", info.duration_s)
    } else {
        format!("录音完成，时长 {:.1}s（{}{low_hint}）", info.duration_s, info.label)
    }
}

/// `mic_record_start`：启动板上麦克风录音（raw ALSA `arecord`，最长 60s），
/// 并挂 61s 自动停止看护任务以免连接断开后写满 `/tmp`。
async fn handle_mic_record_start(hub: &'static Hub, state: &ConnState, tx: &Sender<Message>) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_manual_error(tx, e).await;
        return;
    }
    let tx = tx.clone();
    tokio::spawn(async move {
        let job = tokio::task::spawn_blocking(crate::mic::start_recording);
        let reply = match job.await {
            Err(e) => json!({
                "t": "mic_status",
                "ok": false,
                "action": "start",
                "message": format!("开始录音失败：{e}"),
                "mic": crate::mic::status_snapshot(),
            }),
            Ok(Err(e)) => json!({
                "t": "mic_status",
                "ok": false,
                "action": "start",
                "message": format!("开始录音失败：{e}"),
                "mic": crate::mic::status_snapshot(),
            }),
            Ok(Ok(info)) => {
                let rec_gen = info.generation;
                let tx_auto = tx.clone();
                tokio::spawn(async move {
                    tokio::time::sleep(std::time::Duration::from_secs(
                        crate::mic::MAX_RECORD_SECS + 1,
                    ))
                    .await;
                    if let Ok(Ok(Some(stop))) = tokio::task::spawn_blocking(move || {
                        crate::mic::stop_if_generation(rec_gen)
                    })
                    .await
                    {
                        send_reply(
                            &tx_auto,
                            json!({
                                "t": "mic_status",
                                "ok": true,
                                "action": "auto_stop",
                                "message": mic_stop_message(&stop, true),
                                "mic": stop.snapshot,
                            }),
                            true,
                        )
                        .await;
                    }
                });
                json!({
                    "t": "mic_status",
                    "ok": true,
                    "action": "start",
                    "message": format!(
                        "开始录音（{}，最长 {}s）",
                        info.label,
                        crate::mic::MAX_RECORD_SECS
                    ),
                    "mic": info.snapshot,
                })
            }
        };
        send_reply(&tx, reply, true).await;
    });
}

/// `mic_record_stop`：手动停止当前录音并生成可回放的标准 WAV。
async fn handle_mic_record_stop(hub: &'static Hub, state: &ConnState, tx: &Sender<Message>) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_manual_error(tx, e).await;
        return;
    }
    let tx = tx.clone();
    tokio::spawn(async move {
        let job = tokio::task::spawn_blocking(crate::mic::stop_recording);
        let reply = match job.await {
            Err(e) => json!({
                "t": "mic_status",
                "ok": false,
                "action": "stop",
                "message": format!("结束录音失败：{e}"),
                "mic": crate::mic::status_snapshot(),
            }),
            Ok(Err(e)) => json!({
                "t": "mic_status",
                "ok": false,
                "action": "stop",
                "message": format!("结束录音失败：{e}"),
                "mic": crate::mic::status_snapshot(),
            }),
            Ok(Ok(info)) => json!({
                "t": "mic_status",
                "ok": true,
                "action": "stop",
                "message": mic_stop_message(&info, false),
                "mic": info.snapshot,
            }),
        };
        send_reply(&tx, reply, true).await;
    });
}

/// `mic_play`：经车体喇叭（es8388 `aplay`）播放最近录制的音频。
async fn handle_mic_play(hub: &'static Hub, state: &ConnState, tx: &Sender<Message>) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_manual_error(tx, e).await;
        return;
    }
    let (job, playing_snap) = match crate::mic::begin_play() {
        Ok(pair) => pair,
        Err(e) => {
            send_reply(
                tx,
                json!({
                    "t": "mic_status",
                    "ok": false,
                    "action": "play_done",
                    "message": format!("播放录音失败：{e}"),
                    "mic": crate::mic::status_snapshot(),
                }),
                true,
            )
            .await;
            return;
        }
    };
    send_reply(
        tx,
        json!({
            "t": "mic_status",
            "ok": true,
            "action": "playing",
            "message": "",
            "mic": playing_snap,
        }),
        true,
    )
    .await;

    let tx = tx.clone();
    tokio::spawn(async move {
        let task = tokio::task::spawn_blocking(move || crate::mic::finish_play(job));
        let (ok, message, mic_snap) = match task.await {
            Err(e) => (false, format!("播放录音失败：{e}"), crate::mic::status_snapshot()),
            Ok(Err(e)) => (false, format!("播放录音失败：{e}"), crate::mic::status_snapshot()),
            Ok(Ok((dur, _dev, snap))) => {
                (true, format!("录音播放完成（{:.1}s）", dur), snap)
            }
        };
        send_reply(
            &tx,
            json!({
                "t": "mic_status",
                "ok": ok,
                "action": "play_done",
                "message": message,
                "mic": mic_snap,
            }),
            true,
        )
        .await;
    });
}

/// ros 模式走真实链路并等待真实结果；fake 模式保留假回执（无 AMCL）。
async fn handle_relocalize(hub: &'static Hub, state: &ConnState, msg: &Value, tx: &Sender<Message>) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_manual_error(tx, e).await;
        return;
    }
    // 前端若带了位姿（旧协议），仅做合法性校验并记录；全局匹配以垫片实算为准。
    if let (Some(x), Some(y), Some(yaw)) = (
        msg.get("x").and_then(Value::as_f64),
        msg.get("y").and_then(Value::as_f64),
        msg.get("yaw").and_then(Value::as_f64),
    ) {
        if !x.is_finite() || !y.is_finite() || !yaw.is_finite() {
            send_manual_error(tx, "重定位位姿参数非法").await;
            return;
        }
    }
    if let Some(reason) = hub.relocalize_motion_guard() {
        send_reply(
            tx,
            json!({ "t": "relocalize_status", "ok": false, "message": reason }),
            true,
        )
        .await;
        return;
    }
    if !hub.relocalize_begin() {
        send_reply(
            tx,
            json!({ "t": "relocalize_status", "ok": false, "message": "重定位已在进行中" }),
            true,
        )
        .await;
        return;
    }
    if crate::ros::enabled() {
        let tx = tx.clone();
        tokio::spawn(async move {
            let (ok, detail) = crate::ros::request_relocalize_ros().await;
            hub.relocalize_end();
            if ok {
                let x = detail.get("x").and_then(Value::as_f64).unwrap_or(0.0);
                let y = detail.get("y").and_then(Value::as_f64).unwrap_or(0.0);
                let yaw_deg = detail.get("yaw_deg").and_then(Value::as_f64).unwrap_or(0.0);
                let hit = detail.get("hit_ratio").and_then(Value::as_f64).unwrap_or(0.0);
                let ms = detail.get("match_ms").and_then(Value::as_f64).unwrap_or(0.0);
                send_reply(
                    &tx,
                    json!({ "t": "relocalize_status", "ok": true, "x": x, "y": y,
                            "yaw": yaw_deg, "hit_ratio": hit, "match_ms": ms,
                            "message": format!(
                                "重定位完成：({x:+.2}, {y:+.2}) m，朝向 {yaw_deg:.1}°，匹配率 {:.0}%（{} ms）",
                                hit * 100.0, ms as i64) }),
                    true,
                )
                .await;
            } else {
                let e = detail
                    .get("error")
                    .and_then(Value::as_str)
                    .unwrap_or("重定位失败");
                send_reply(
                    &tx,
                    json!({ "t": "relocalize_status", "ok": false, "message": e }),
                    true,
                )
                .await;
            }
        });
        return;
    }
    // fake 模式：无 AMCL，保留假回执（本地 UI 演示）。
    let tx = tx.clone();
    tokio::spawn(async move {
        tokio::time::sleep(std::time::Duration::from_millis(1200)).await;
        hub.relocalize_end();
        send_reply(
            &tx,
            json!({ "t": "relocalize_status", "ok": true, "message": "已完成重定位校准（演示）" }),
            true,
        )
        .await;
    });
}

async fn handle_patrol_save(hub: &'static Hub, state: &ConnState, msg: &Value, tx: &Sender<Message>) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_manual_error(tx, e).await;
        return;
    }
    let points = msg.get("points").cloned().unwrap_or(Value::Null);
    let map_id = msg.get("map_id").and_then(Value::as_str).map(str::to_string);
    let (ok, message) = hub.patrol_save(&points, map_id.as_deref());
    let mut frame = hub.patrol_status_frame();
    frame["t"] = json!("patrol_status");
    frame["ok"] = json!(ok);
    frame["message"] = json!(message);
    send_reply(tx, frame, true).await;
}

async fn handle_patrol_start(hub: &'static Hub, state: &ConnState, tx: &Sender<Message>) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_manual_error(tx, e).await;
        return;
    }
    let (ok, message) = hub.patrol_start();
    let mut frame = hub.patrol_status_frame();
    frame["t"] = json!("patrol_status");
    frame["ok"] = json!(ok);
    frame["message"] = json!(message);
    send_reply(tx, frame, true).await;
}

async fn handle_motion_stop(hub: &'static Hub, state: &ConnState, tx: &Sender<Message>) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_manual_error(tx, e).await;
        return;
    }
    hub.motion_stop();
    // ros 模式：撤回下游鉴权 + 取消在途 nav 目标（web_debug.py:935-947
    // lock_stop 第一动作；safety.rs:562-563 nav_cancel）。
    if crate::ros::enabled() {
        crate::ros::on_motion_stop();
    }
    let mut frame = hub.patrol_status_frame();
    frame["t"] = json!("patrol_status");
    frame["ok"] = json!(true);
    frame["message"] = json!("已全局停止");
    send_reply(tx, frame, true).await;
}

async fn handle_motion_rearm(hub: &'static Hub, state: &ConnState, tx: &Sender<Message>) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_manual_error(tx, e).await;
        return;
    }
    let (ok, message) = hub.motion_rearm();
    // ros 模式：先补鉴权再发解停，两条命令同批按序（顺序铁律
    // web_debug.py:555-560；rearm_request 边沿触发永不补投）。
    if ok && crate::ros::enabled() {
        crate::ros::on_rearm_ok();
    }
    let mut frame = hub.patrol_status_frame();
    frame["t"] = json!("patrol_status");
    frame["ok"] = json!(ok);
    frame["message"] = json!(message);
    send_reply(tx, frame, true).await;
}

async fn handle_nav_goal(hub: &'static Hub, state: &ConnState, msg: &Value, tx: &Sender<Message>) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_manual_error(tx, e).await;
        return;
    }
    let x = msg.get("x").and_then(Value::as_f64);
    let y = msg.get("y").and_then(Value::as_f64);
    let th = msg.get("th").and_then(Value::as_f64);
    let (Some(x), Some(y)) = (x, y) else {
        send_manual_error(tx, "目标坐标不是数字").await;
        return;
    };
    // ros 模式走 ros::request_nav_goal：范围/间距校验 + 与巡逻共用单一
    // 在途目标槽（nav_goal.py:96-111；契约 risks#7）；fake 模式保持原样。
    let (ok, view) = if crate::ros::enabled() {
        crate::ros::request_nav_goal(x, y, th)
    } else {
        hub.nav_request(x, y, th)
    };
    let mut out = view;
    out["t"] = json!("nav_status");
    send_reply(tx, out, true).await;
    if !ok {
        let err = {
            let ctl = hub.control.lock().expect("control poisoned");
            ctl.nav.error.clone()
        };
        send_manual_error(tx, &err).await;
    }
}

async fn handle_nav_cancel(hub: &'static Hub, state: &ConnState, tx: &Sender<Message>) {
    if let Err(e) = require_drive_auth(hub, state) {
        send_manual_error(tx, e).await;
        return;
    }
    let (ok, view) = if crate::ros::enabled() {
        crate::ros::cancel_nav_goal()
    } else {
        hub.nav_cancel()
    };
    let mut out = view;
    out["t"] = json!("nav_status");
    send_reply(tx, out, true).await;
    if !ok {
        let err = {
            let ctl = hub.control.lock().expect("control poisoned");
            ctl.nav.error.clone()
        };
        send_manual_error(tx, &err).await;
    }
}
