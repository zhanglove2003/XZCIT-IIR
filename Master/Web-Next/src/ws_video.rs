//! `/ws/video` 媒体流通道（蓝图 §4.3 序列图）。
//!
//! 只推显示数据，不处理任何控制消息——控制与停车不经此路
//! （web_debug.py::_video_loop，2026-11-14 控制与视频分离）：
//! - 10Hz `{"t":"scan","p":[[deg,r],...],"hz":10.0,"n":有效点数,"all":总点数}`；
//! - 1Hz 且仅 seq 变化时 `{"t":"map", ... ,"d":"<zlib+base64>","seq":...,
//!   "pose":{x,y,th}}` 占据栅格（`d` 键名按蓝图 §4.3 序列图逐字）；
//! - 5Hz `{"t":"pose","pose":{...},"pose_age":...,"map_age":...}` 位姿；
//! - 仅存在「已登录且在控制页」会话时 `{"t":"img","d":...,"w":384,"h":216,
//!   "fps":5.0,"mime":"image/png"}` 相机帧（看/开分离：点云与栅格免登录
//!   直推，相机只在控制页显示）。
//!
//! 慢客户端丢帧降级（web_debug.py::_video_loop + VIDEO_DROP_LIMIT）：
//! - 出站队列满 → 丢弃本帧（显示数据可丢）并计数，连续丢满
//!   `VIDEO_DROP_LIMIT` 帧主动断开，前端自动重连；
//! - 发送超时 `DISPLAY_SEND_TIMEOUT_S`（500ms）说明可能发生部分写入，
//!   字节流不再可信，同样断开本连接。

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::response::Response;
use futures_util::{SinkExt, StreamExt};
use serde_json::json;
use tokio::sync::{mpsc, watch};

use crate::fake;
use crate::safety;

/// 状态轮询周期（ms）：与旧实现一致的主循环节拍。
const LOOP_PERIOD_MS: u64 = 20;
/// 扫描点推送周期（ms）：整圈约 8KB，画面上限 10Hz（web_debug.py:2408-2410）。
const SCAN_PERIOD_MS: u64 = 100;
/// 地图推送周期（ms）：SLAM 栅格变化不快，1Hz 且仅在 seq 变化时推。
const MAP_PERIOD_MS: u64 = 1000;
/// 位姿推送周期（ms）：静态地图下独立轻量 pose 帧按 5Hz 推。
const POSE_PERIOD_MS: u64 = 200;
/// 图像推送周期（ms）：最多 IMG_HZ=5 帧/秒。
const IMG_PERIOD_MS: u64 = 200;
/// 显示帧发送可写等待时限（ms）。
const DISPLAY_SEND_TIMEOUT_MS: u64 = 500;
/// 连续丢弃多少帧后主动断开（慢客户端降级）。
const VIDEO_DROP_LIMIT: u32 = 60;
/// 出站队列容量。
const OUTBOUND_CAP: usize = 8;

/// `/ws/video` 端点。
pub async fn endpoint(upgrade: WebSocketUpgrade) -> Response {
    upgrade.on_upgrade(|socket| handle_socket(socket))
}

pub async fn handle_socket(socket: WebSocket) {
    let (mut sink, mut stream) = socket.split();
    let (tx, mut rx) = mpsc::channel::<Message>(OUTBOUND_CAP);
    let (close_tx, close_rx) = watch::channel(false);

    // 读侧只负责检测关闭：慢客户端断开时尽快终结推送循环。
    let reader = tokio::spawn(async move {
        while let Some(Ok(msg)) = stream.next().await {
            if matches!(msg, Message::Close(_)) {
                break;
            }
        }
        let _ = close_tx.send(true);
    });
    let _ = reader; // 主循环退出、sink 落地后自然结束

    let mut last_img_seq: u64 = 0;
    let mut last_scan_seq: u64 = 0;
    let mut last_map_seq: u64 = 0;
    let mut last_pose_ms: u64 = 0;
    let mut next_scan = 0u64;
    let mut next_map = 0u64;
    let mut next_pose = 0u64;
    let mut next_img = 0u64;
    let mut drops: u32 = 0;

    let mut close_rx = close_rx;
    let mut ticker = tokio::time::interval(std::time::Duration::from_millis(LOOP_PERIOD_MS));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

    loop {
        tokio::select! {
            changed = close_rx.changed() => {
                if changed.is_err() || *close_rx.borrow() {
                    break;
                }
            }
            // 出站队列 → 套接字：带时限发送，超时即断开（部分写入不可信）。
            out = rx.recv() => {
                let Some(msg) = out else { break };
                let send = tokio::time::timeout(
                    std::time::Duration::from_millis(DISPLAY_SEND_TIMEOUT_MS),
                    sink.send(msg),
                ).await;
                match send {
                    Ok(Ok(())) => drops = 0,
                    Ok(Err(_)) | Err(_) => break,
                }
            }
            _ = ticker.tick() => {
                if *close_rx.borrow() {
                    break;
                }
                let now = safety::now_ms();
                for frame in build_frames(
                    now,
                    &mut last_img_seq,
                    &mut last_scan_seq,
                    &mut last_map_seq,
                    &mut last_pose_ms,
                    &mut next_scan,
                    &mut next_map,
                    &mut next_pose,
                    &mut next_img,
                ) {
                    match tx.try_send(Message::Text(frame.into())) {
                        Ok(()) => drops = 0,
                        Err(mpsc::error::TrySendError::Full(_)) => drops += 1,
                        Err(mpsc::error::TrySendError::Closed(_)) => {
                            drops = VIDEO_DROP_LIMIT;
                            break;
                        }
                    }
                }
                if drops >= VIDEO_DROP_LIMIT {
                    // 慢客户端降级：主动断开，页面会自动重连（web_debug.py:2500-2505）。
                    eprintln!("视频客户端连续 {VIDEO_DROP_LIMIT} 帧不可写，主动断开（慢客户端降级）");
                    break;
                }
            }
        }
    }

    // 关闭 WebSocket：向对端发送 Close 帧。
    let _ = sink.send(Message::Close(None)).await;
    let _ = rx.close();
}

/// 按节流周期构建本拍要推的帧（顺序：img → scan → map → pose，与旧实现一致）。
#[allow(clippy::too_many_arguments)]
fn build_frames(
    now: u64,
    last_img_seq: &mut u64,
    last_scan_seq: &mut u64,
    last_map_seq: &mut u64,
    last_pose_ms: &mut u64,
    next_scan: &mut u64,
    next_map: &mut u64,
    next_pose: &mut u64,
    next_img: &mut u64,
) -> Vec<String> {
    let hub = safety::global();
    let media = fake::media_slot().read().expect("media poisoned");
    let mut out = Vec::with_capacity(4);

    // 相机帧：仅存在已登录且在控制页的会话时推送（看/开分离）。
    if media.image_seq != *last_img_seq
        && !media.image_b64.is_empty()
        && now >= *next_img
        && hub.has_live_control()
    {
        *last_img_seq = media.image_seq;
        *next_img = now + IMG_PERIOD_MS;
        out.push(
            json!({
                "t": "img",
                "d": media.image_b64.as_ref(),
                "w": media.image_w,
                "h": media.image_h,
                "fps": media.image_fps,
                "mime": media.image_mime,
            })
            .to_string(),
        );
    }

    // 雷达点云：10Hz 上限，新帧才推。
    if media.scan_seq != *last_scan_seq && !media.scan.is_empty() && now >= *next_scan {
        *last_scan_seq = media.scan_seq;
        *next_scan = now + SCAN_PERIOD_MS;
        let pts: Vec<[f64; 2]> = media.scan.iter().map(|(d, r)| [*d, *r]).collect();
        out.push(
            json!({
                "t": "scan",
                "p": pts,
                "hz": media_scan_hz(&media),
                "n": media.scan_valid,
                "all": media.scan.len(),
            })
            .to_string(),
        );
    }

    // 占据栅格：seq 变化才推，1Hz 上限；位姿与栅格同帧携带。
    let pose = pose_value(&media);
    let pose_age = pose_age_value(&media, now);
    if media.grid.seq != 0 && media.grid.seq != *last_map_seq && now >= *next_map {
        *last_map_seq = media.grid.seq;
        *next_map = now + MAP_PERIOD_MS;
        *last_pose_ms = media.pose_mono_ms;
        *next_pose = now + POSE_PERIOD_MS;
        let grid_age = unix_now() - media.grid.grid_t_unix;
        out.push(
            json!({
                "t": "map",
                "seq": media.grid.seq,
                "w": media.grid.w,
                "h": media.grid.h,
                "res": media.grid.res,
                "ox": media.grid.ox,
                "oy": media.grid.oy,
                "age": (grid_age * 100.0).round() / 100.0,
                "d": media.grid.data_b64.as_ref(),
                "pose": pose,
                "pose_age": pose_age,
            })
            .to_string(),
        );
    } else if media.pose_mono_ms != *last_pose_ms && media.pose_mono_ms > 0 && now >= *next_pose {
        // 静态地图模式下单独推实时位姿（5Hz，避免重复发送整幅栅格）。
        *last_pose_ms = media.pose_mono_ms;
        *next_pose = now + POSE_PERIOD_MS;
        let map_age = unix_now() - media.grid.grid_t_unix;
        out.push(
            json!({
                "t": "pose",
                "pose": pose,
                "pose_age": pose_age,
                "map_age": (map_age * 100.0).round() / 100.0,
            })
            .to_string(),
        );
    }

    out
}

fn media_scan_hz(media: &fake::Media) -> f64 {
    // 契约 frameMap /scan 行修订点：读 media.scan_hz 实测值——Fake 引擎写
    // 常量 SCAN_HZ（声明 fake.rs:49、写入位 fake.rs:629，行为不变）；ros
    // 模式由桥接层按 2s 窗口实测写入，不再恒报 10.0 假值。
    media.scan_hz
}

fn pose_value(media: &fake::Media) -> serde_json::Value {
    let (x, y, th) = media.pose;
    json!({
        "x": (x * 1000.0).round() / 1000.0,
        "y": (y * 1000.0).round() / 1000.0,
        "th": (th * 10000.0).round() / 10000.0,
    })
}

fn pose_age_value(media: &fake::Media, now: u64) -> serde_json::Value {
    if media.pose_mono_ms == 0 {
        return serde_json::Value::Null;
    }
    let age = now.saturating_sub(media.pose_mono_ms) as f64 / 1000.0;
    json!((age * 100.0).round() / 100.0)
}

fn unix_now() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}
