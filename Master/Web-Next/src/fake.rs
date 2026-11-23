//! FakeEngine 合成仿真源（`--fake` / `OPI_WEB_FAKE=1`）。
//!
//! 移植并优化旧 `fake_source.py` 的合成数据语义（fake_source.py 全文）：
//! - 24s 周期脚本动作（0-8s 加速直行 / 8-14s 匀速 / 14-18s 减速转向 /
//!   18-24s 停车保持），叠加透视相机、360° 雷达、占据栅格与位姿轨迹；
//! - 6S 电池动态电压：电压随负载缓慢下坠、停车回升
//!   （fake_source.py::chassis_status 的负载模型，按 2026-11-20 现场标定
//!   24.5V 满电 / 21.0V 停用线 / 22.2V 低电警告 / >25.0V 充电中折算）；
//! - 矩形房间 + 圆柱的射线求交点云，带周期性遮挡与噪声
//!   （fake_source.py::scan_points）。
//!
//! 相对旧实现的增量（蓝图 §6 阶段 2 要求的完整交互闭环）：
//! - 旧 FakeBridge 不生成地图/位姿；这里补上静态占据栅格（房间+柱子）与
//!   由命令速度积分的位姿轨迹，雷达点云改在世界系下随位姿旋转平移；
//! - 旧 FakeBridge 的手动命令没有下发通道（manual_forward._publish 为
//!   None）；这里手动/巡航/航点/点选导航直接驱动力仿真车，任务结束后
//!   回落 24s 脚本动作；
//! - 雷达/相机/地图快照放在独立的 `MediaStore`，不进每秒 20 次克隆的
//!   状态快照。

use std::f64::consts::PI;
use std::io::Write;
use std::sync::{Arc, OnceLock, RwLock};

use serde_json::{json, Value};

use crate::safety::{self, Hub};
use crate::state::{
    BatteryView, BootView, ChassisView, CmdView, CmdViews, CruiseView, DriveSafetyView, ImuView,
    NodesView, StateStore, StatusPayload, TaskState, TaskView,
};
use crate::Config;

/// 仿真循环周期（fake_source.py:18 `CYCLE_S = 24.0`）。
pub const CYCLE_S: f64 = 24.0;
/// 假房间尺寸（m）；雷达原点在房间中心（fake_source.py:21-22）。
pub const ROOM_W_M: f64 = 6.4;
pub const ROOM_H_M: f64 = 4.6;
/// 房间内柱子（fake_source.py:23-25）。
pub const POLE_X_M: f64 = 1.9;
pub const POLE_Y_M: f64 = 0.7;
pub const POLE_R_M: f64 = 0.22;
/// 合成画面尺寸（fake_source.py:27-29）。
pub const CAM_W: u32 = 384;
pub const CAM_H: u32 = 216;
/// 引擎节拍：20Hz（蓝图 §6 阶段 2「20Hz 底盘状态」）。
pub const TICK_MS: u64 = 50;
/// 雷达 10Hz（fake_source.py 注释：与常见 RPLIDAR 一致）。
pub const SCAN_HZ: f64 = 10.0;
/// 相机 5Hz（旧 IMG_HZ = 5.0）。
pub const IMAGE_HZ: f64 = 5.0;
/// 占据栅格分辨率（蓝图 §4.3 序列图示例值 0.05）。
pub const GRID_RES: f64 = 0.05;
/// 行驶墙壁安全边距（m）：车体半径 + 余量。
pub const WALL_MARGIN_M: f64 = 0.35;
/// 超声波近距闩值（m）：低于此值置位近距闩，需显式恢复。
pub const US_NEAR_M: f64 = 0.35;
/// 自动任务（巡航/航点/导航）行驶速度（m/s）。
pub const AUTO_SPEED_M_S: f64 = 0.30;

// ---------------------------------------------------------------------------
// PNG 编码（fake_source.py::png_bytes 的移植；zlib 用 flate2）
// ---------------------------------------------------------------------------

fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

fn png_chunk(out: &mut Vec<u8>, tag: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    let mut body = Vec::with_capacity(4 + data.len());
    body.extend_from_slice(tag);
    body.extend_from_slice(data);
    out.extend_from_slice(&body);
    out.extend_from_slice(&crc32(&body).to_be_bytes());
}

/// RGB 行数据 → PNG（8-bit 真彩色，无隔行）。
fn png_bytes(width: u32, height: u32, rows: &[Vec<u8>]) -> Vec<u8> {
    let mut raw = Vec::with_capacity(rows.len() * (width as usize * 3 + 1));
    for row in rows {
        raw.push(0u8); // filter type 0：None
        raw.extend_from_slice(row);
    }
    let mut enc = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
    enc.write_all(&raw).expect("png zlib write");
    let idat = enc.finish().expect("png zlib finish");
    let mut out = Vec::with_capacity(idat.len() + 128);
    out.extend_from_slice(b"\x89PNG\r\n\x1a\n");
    let mut ihdr = Vec::with_capacity(13);
    ihdr.extend_from_slice(&width.to_be_bytes());
    ihdr.extend_from_slice(&height.to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]);
    png_chunk(&mut out, b"IHDR", &ihdr);
    png_chunk(&mut out, b"IDAT", &idat);
    png_chunk(&mut out, b"IEND", b"");
    out
}

// ---------------------------------------------------------------------------
// 合成相机画面（fake_source.py::camera_frame / _sky_rows 的逐字移植）
// ---------------------------------------------------------------------------

fn camera_rows(width: u32, height: u32, phase: f64) -> Vec<Vec<u8>> {
    let width = width as usize;
    let height = height as usize;
    let horizon = (height as f64 * 0.42) as usize;
    let mut rows: Vec<Vec<u8>> = Vec::with_capacity(height);
    for y in 0..height {
        let mut row = vec![0u8; width * 3];
        if y < horizon {
            let t = y as f64 / horizon.max(1) as f64;
            let (r, g, b) = ((18.0 + 30.0 * t) as u8, (24.0 + 40.0 * t) as u8, (38.0 + 62.0 * t) as u8);
            for px in row.chunks_exact_mut(3) {
                px[0] = r;
                px[1] = g;
                px[2] = b;
            }
        } else {
            let t = (y - horizon) as f64 / (height - horizon).max(1) as f64;
            let base = (38.0 + 46.0 * t) as i64;
            // 透视地面网格：横线按 t 压缩，竖线向灭点收拢
            let hstep = 26.0 * (1.0 - t * 0.8);
            let hstep = hstep.max(3.0) as i64;
            let hline = ((y - horizon) as i64) % hstep == 0;
            let depth = ((y - horizon + 1) as f64 * 0.5).max(1.0);
            let cx = width as f64 / 2.0;
            for x in 0..width {
                let dx = (x as f64 - cx) / depth;
                let vline = (dx - dx.round()).abs() < 0.055;
                let v = base + if hline || vline { 34 } else { 0 };
                row[x * 3] = (v - 6).clamp(0, 255) as u8;
                row[x * 3 + 1] = v.clamp(0, 255) as u8;
                row[x * 3 + 2] = (v + 8).clamp(0, 255) as u8;
            }
        }
        rows.push(row);
    }
    // 目标块随相位左右移动，模拟场景里有人或箱子
    let block_w = ((width / 12) as i64).max(10);
    let block_h = ((height / 4) as i64).max(16);
    let target_cx =
        (width as f64 * (0.5 + 0.32 * phase.sin())) as i64;
    let top = horizon as i64 - block_h + (6.0 * (phase * 2.0).sin()) as i64;
    for yy in top.max(0)..(top + block_h).min(height as i64) {
        let y = yy as usize;
        let row = &mut rows[y];
        let shade_span = block_h.max(1) as f64;
        for xx in (target_cx - block_w / 2).max(0)..(target_cx + block_w / 2).min(width as i64) {
            let x = xx as usize;
            let shade =
                200.0 - 40.0 * ((yy - top) as f64 / shade_span);
            let shade = shade.clamp(0.0, 255.0) as u8;
            row[x * 3] = shade;
            row[x * 3 + 1] = 60;
            row[x * 3 + 2] = 52;
        }
    }
    // 取景十字与安全框
    let cx = width / 2;
    let cy = (height as f64 * 0.42) as usize;
    for x in 0..width {
        rows[cy][x * 3] = 77;
        rows[cy][x * 3 + 1] = 163;
        rows[cy][x * 3 + 2] = 255;
    }
    for row in rows.iter_mut() {
        row[cx * 3] = 77;
        row[cx * 3 + 1] = 163;
        row[cx * 3 + 2] = 255;
    }
    rows
}

// ---------------------------------------------------------------------------
// 占据栅格（旧实现从 /map 取；Fake 模式按假房间合成一张静态栅格）
// ---------------------------------------------------------------------------

/// 静态栅格快照（含一次性 zlib+base64 编码结果，地图页 1Hz 推送直接引用）。
pub struct GridPub {
    pub seq: u64,
    pub w: usize,
    pub h: usize,
    pub res: f64,
    pub ox: f64,
    pub oy: f64,
    /// zlib 压缩 + base64（浏览器 DecompressionStream('deflate') 原生解压，
    /// 与旧 _map_frame_json 同一套编码，web_debug.py:2903-2924）。
    pub data_b64: Arc<str>,
    pub grid_t_unix: f64,
}

fn build_grid() -> (String, GridPub) {
    let w = (ROOM_W_M / GRID_RES).round() as usize;
    let h = (ROOM_H_M / GRID_RES).round() as usize;
    let half_w = ROOM_W_M / 2.0;
    let half_h = ROOM_H_M / 2.0;
    let ox = -half_w;
    let oy = -half_h;
    let mut data = vec![0u8; w * h];
    for cy in 0..h {
        for cx in 0..w {
            // OccupancyGrid 第 0 行是 y 最大的一行（ROS 约定）。
            let wx = ox + (cx as f64 + 0.5) * GRID_RES;
            let wy = oy + ((h - 1 - cy) as f64 + 0.5) * GRID_RES;
            let wall = wx >= half_w - GRID_RES / 2.0
                || wx <= -half_w + GRID_RES / 2.0
                || wy >= half_h - GRID_RES / 2.0
                || wy <= -half_h + GRID_RES / 2.0;
            let pole_d2 = (wx - POLE_X_M).powi(2) + (wy - POLE_Y_M).powi(2);
            let pole = pole_d2 <= (POLE_R_M + GRID_RES).powi(2);
            if wall || pole {
                data[cy * w + cx] = 100;
            }
        }
    }
    let map_id = safety::sha256_hex(&data)[..16].to_string();
    let mut zenc = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
    zenc.write_all(&data).expect("grid zlib write");
    let compressed = zenc.finish().expect("grid zlib finish");
    use base64::Engine as _;
    let data_b64 = base64::engine::general_purpose::STANDARD.encode(&compressed);
    let grid_t_unix = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0);
    let pub_grid = GridPub {
        seq: 1, // 静态地图（AMCL 场景下栅格 seq 不再变，位姿走独立 5Hz 帧）
        w,
        h,
        res: GRID_RES,
        ox,
        oy,
        data_b64: Arc::from(data_b64.as_str()),
        grid_t_unix,
    };
    (map_id, pub_grid)
}

// ---------------------------------------------------------------------------
// 媒体快照中心（/ws/video 的数据源，与状态快照分离）
// ---------------------------------------------------------------------------

pub struct Media {
    pub image_b64: Arc<str>,
    pub image_w: u32,
    pub image_h: u32,
    pub image_seq: u64,
    pub image_mime: &'static str,
    pub image_fps: f64,
    /// 雷达实测频率（Hz）。Fake 引擎写常量 SCAN_HZ（fake.rs:637 的常量位）；
    /// ros 模式由桥接层按 2s 窗口实测写入（契约 frameMap /scan 行修订点），
    /// ws_video.rs 的 scan 帧读这里，不再读常量。
    pub scan_hz: f64,
    pub scan: Arc<Vec<(f64, f64)>>,
    pub scan_valid: usize,
    pub scan_seq: u64,
    pub grid: Arc<GridPub>,
    /// 仿真车位姿 (x, y, th)，房间坐标系（与栅格/点云同系）。
    pub pose: (f64, f64, f64),
    /// 最近一次位姿更新的单调毫秒。
    pub pose_mono_ms: u64,
}

static MEDIA: OnceLock<RwLock<Media>> = OnceLock::new();

/// 媒体快照中心（/ws/video 数据源）。
pub fn media_slot() -> &'static RwLock<Media> {
    MEDIA.get_or_init(|| {
        RwLock::new(Media {
            image_b64: Arc::from(""),
            image_w: 0,
            image_h: 0,
            image_seq: 0,
            image_mime: "image/png",
            image_fps: 0.0,
            scan_hz: 0.0,
            scan: Arc::new(Vec::new()),
            scan_valid: 0,
            scan_seq: 0,
            grid: Arc::new(GridPub {
                seq: 0,
                w: 0,
                h: 0,
                res: GRID_RES,
                ox: 0.0,
                oy: 0.0,
                data_b64: Arc::from(""),
                grid_t_unix: 0.0,
            }),
            pose: (0.0, 0.0, 0.0),
            pose_mono_ms: 0,
        })
    })
}

/// 世界系射线求交：矩形房间四面墙 + 圆柱，取最近正交点
/// （fake_source.py::scan_points 的求交逻辑，原点从房间中心推广到任意位姿）。
fn ray_hit(px: f64, py: f64, angle: f64) -> f64 {
    let dx = angle.cos();
    let dy = angle.sin();
    let half_w = ROOM_W_M / 2.0;
    let half_h = ROOM_H_M / 2.0;
    let mut best = f64::INFINITY;
    for (num, den) in [
        (half_w - px, dx),
        (-half_w - px, dx),
        (half_h - py, dy),
        (-half_h - py, dy),
    ] {
        if den.abs() < 1e-9 {
            continue;
        }
        let t = num / den;
        if t > 0.0 {
            best = best.min(t);
        }
    }
    // 与圆柱求交
    let ox = POLE_X_M - px;
    let oy = POLE_Y_M - py;
    let b = 2.0 * (dx * ox + dy * oy);
    let c = ox * ox + oy * oy - POLE_R_M * POLE_R_M;
    let disc = b * b - 4.0 * c;
    if disc > 0.0 {
        let t = (-b - disc.sqrt()) / 2.0;
        if t > 0.0 {
            best = best.min(t);
        }
    }
    if best.is_finite() {
        best
    } else {
        0.0
    }
}

fn round2(v: f64) -> f64 {
    (v * 100.0).round() / 100.0
}

fn round3(v: f64) -> f64 {
    (v * 1000.0).round() / 1000.0
}

// ---------------------------------------------------------------------------
// FakeEngine
// ---------------------------------------------------------------------------

pub struct FakeEngine {
    hub: &'static Hub,
    store: &'static StateStore,
    mode: &'static str,
    srv_epoch: i64,
    /// 仿真车位姿。
    pose: (f64, f64, f64),
    /// 里程积分（m，|vx| 积分；旧 MileageIntegrator 语义）。
    mileage: f64,
    /// 上一节拍时刻（ms）。
    last_tick_ms: u64,
    /// 节拍计数（10Hz 雷达 / 5Hz 相机按 2/4 分频）。
    cycle: u64,
    /// drive_safety 序号。
    ds_seq: u64,
    /// 24s 相位起点（引擎启动时刻）。
    t0_ms: u64,
    /// 当前命令（诊断/状态帧用）。
    cmd: (f64, f64),
    /// 命令来源（nav/auto/manual/demo）。
    cmd_source: &'static str,
}

impl FakeEngine {
    pub fn new(store: &'static StateStore, hub: &'static Hub, mode: &'static str) -> Self {
        let srv_epoch = store.snapshot().srv_epoch;
        let t0_ms = safety::now_ms();
        Self {
            hub,
            store,
            mode,
            srv_epoch,
            pose: (-2.55, -1.35, 0.0),
            mileage: 0.0,
            last_tick_ms: t0_ms,
            cycle: 0,
            ds_seq: 0,
            t0_ms,
            cmd: (0.0, 0.0),
            cmd_source: "demo",
        }
    }

    /// 24s 相位（fake_source.py::FakeSource.phase）。
    fn phase(&self) -> f64 {
        ((safety::now_ms() - self.t0_ms) as f64 / 1000.0).max(0.0)
    }

    /// 主循环：20Hz。永不返回（tokio 任务）。
    pub async fn run(mut self) {
        let mut tick_interval = tokio::time::interval(std::time::Duration::from_millis(TICK_MS));
        tick_interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            tick_interval.tick().await;
            self.tick();
        }
    }

    /// 单节拍：看护 → 决策 → 积分 → 合成 → 写快照。
    fn tick(&mut self) {
        let now = safety::now_ms();
        let dt = (now - self.last_tick_ms).max(1) as f64 / 1000.0;
        self.last_tick_ms = now;
        self.cycle += 1;
        self.hub.license_tick();
        self.hub.manual_tick();
        self.hub.session_tick();
        self.nav_housekeeping(now);

        let command = self.decide_command();
        self.integrate(command, dt);
        self.synthesize(now);
        self.publish_status(now);
    }

    /// 导航超时与到达判定之外的看护（到达判定在跟随器里做）。
    fn nav_housekeeping(&mut self, now: u64) {
        let mut ctl = self.hub.control.lock().expect("control poisoned");
        if ctl.nav.state == "active" {
            if let Some(t) = ctl.nav.goal_t_ms {
                if now.saturating_sub(t) > safety::NAV_GOAL_TIMEOUT_MS {
                    ctl.nav.state = "failed";
                    ctl.nav.result = "failed".to_string();
                    ctl.nav.error = "导航超时".to_string();
                }
            }
        }
    }

    /// 运动决策：锁停 > 手动 > 自动任务（许可新鲜才动）> 24s 脚本演示。
    fn decide_command(&mut self) -> (f64, f64) {
        let mut ctl = self.hub.control.lock().expect("control poisoned");
        let fresh = {
            let now = safety::now_ms();
            match ctl.license.last_rx_ms {
                Some(t) => now.saturating_sub(t) <= safety::LICENSE_TTL_MS,
                None => false,
            }
        };
        if ctl.user_locked {
            // 全局紧急锁停：连脚本演示一起冻结，需显式恢复。
            self.cmd_source = "locked";
            return (0.0, 0.0);
        }
        // 手动优先：持有者仍在心跳窗口内（超时已由 manual_tick 归零）。
        if ctl.manual.owner.is_some() {
            self.cmd_source = "manual";
            return ctl.manual.cmd;
        }
        let auto_active = ctl.patrol.status == "running"
            || ctl.nav.state == "active"
            || ctl.cruise_on;
        if auto_active && ctl.locked {
            // 巡航门锁停（启动即锁停 / license_lost / 会话失效）：
            // 自动任务零速等待恢复；无任务在跑时脚本演示照常。
            self.cmd_source = "gated";
            return (0.0, 0.0);
        }
        if auto_active {
            // 自动运动需要有效许可（断流 350ms 即锁停，蓝图 §4.3(1)）。
            if !fresh {
                self.cmd_source = "unlicensed";
                return (0.0, 0.0);
            }
            if ctl.us_latched {
                self.cmd_source = "us_blocked";
                return (0.0, 0.0);
            }
            if ctl.nav.state == "active" {
                // 点选导航优先于巡逻：到点后 nav→done，巡逻自动续跑
                // （patrol.status 保持 running，index 冻结在当前航点）。
                return self.follow_nav(&mut ctl);
            }
            if ctl.patrol.status == "running" {
                return self.follow_patrol(&mut ctl);
            }
            self.cmd_source = "auto";
            return self.cruise_wander();
        }
        self.cmd_source = "demo";
        self.scripted_demo()
    }

    /// 航点跟随：到点推进下标，闭环回绕计圈（patrol.py 巡逻语义）。
    fn follow_patrol(&mut self, ctl: &mut std::sync::MutexGuard<'_, safety::Control>) -> (f64, f64) {
        let Some(route) = ctl.patrol.route.clone() else {
            return (0.0, 0.0);
        };
        if route.points.is_empty() {
            return (0.0, 0.0);
        }
        let n = route.points.len();
        let idx = ctl.patrol.index.min(n - 1);
        let (tx, ty) = route.points[idx];
        let dist = ((tx - self.pose.0).powi(2) + (ty - self.pose.1).powi(2)).sqrt();
        if dist < safety::GOAL_REACHED_M {
            let next = idx + 1;
            if next >= n {
                ctl.patrol.index = 0;
                ctl.patrol.laps += 1;
            } else {
                ctl.patrol.index = next;
            }
        }
        self.cmd_source = "nav";
        self.follow_point(tx, ty, AUTO_SPEED_M_S)
    }

    /// 单点导航跟随：到达即 done/succeeded（nav_goal.py 结果语义）。
    fn follow_nav(&mut self, ctl: &mut std::sync::MutexGuard<'_, safety::Control>) -> (f64, f64) {
        let Some(goal) = ctl.nav.goal else {
            return (0.0, 0.0);
        };
        let dist = ((goal.x - self.pose.0).powi(2) + (goal.y - self.pose.1).powi(2)).sqrt();
        if dist < safety::GOAL_REACHED_M {
            ctl.nav.state = "done";
            ctl.nav.result = "succeeded".to_string();
            self.cmd_source = "nav";
            return (0.0, 0.0);
        }
        self.cmd_source = "nav";
        self.follow_point(goal.x, goal.y, AUTO_SPEED_M_S)
    }

    /// 比例导航跟随器：先转向，对准后再走。
    fn follow_point(&self, tx: f64, ty: f64, speed: f64) -> (f64, f64) {
        let (px, py, th) = self.pose;
        let err = wrap_pi((ty - py).atan2(tx - px) - th);
        let wz = (2.2 * err).clamp(-1.2, 1.2);
        let vx = if err.abs() > 0.7 { 0.0 } else { speed * (1.0 - err.abs() / 0.7).max(0.3) };
        (vx, wz)
    }

    /// 自动避障巡航：前向射线近障即转向，取空间更大的一侧。
    fn cruise_wander(&self) -> (f64, f64) {
        let (px, py, th) = self.pose;
        let front = ray_hit(px, py, th);
        let left = ray_hit(px, py, th + 0.7);
        let right = ray_hit(px, py, th - 0.7);
        if front < 0.55 {
            let side = if left > right { 0.9 } else { -0.9 };
            (0.0, side)
        } else {
            let wz = (0.8 * (left - right)).clamp(-0.7, 0.7);
            (AUTO_SPEED_M_S, wz)
        }
    }

    /// 24s 脚本演示（fake_source.py::chassis_status 的速度段）：
    /// 0-8s 加速直行 / 8-14s 匀速 / 14-18s 减速转向 / 18-24s 停车保持。
    fn scripted_demo(&self) -> (f64, f64) {
        let cycle = self.phase() % CYCLE_S;
        if cycle < 8.0 {
            (0.42 * (cycle / 8.0), 0.0)
        } else if cycle < 14.0 {
            (0.42, 0.0)
        } else if cycle < 18.0 {
            let k = (cycle - 14.0) / 4.0;
            // 转向积分恰为一个半圈（π），转完后朝返程方向
            ((0.42 * (1.0 - k)).max(0.0), (PI / 4.0) * (1.0 - (2.0 * PI * k).cos()))
        } else {
            (0.0, 0.0)
        }
    }

    /// 位姿积分 + 墙壁钳制 + 超声波测距。
    fn integrate(&mut self, command: (f64, f64), dt: f64) {
        let (vx, wz) = command;
        self.cmd = command;
        let (px, py, th) = self.pose;
        let th2 = wrap_pi(th + wz * dt);
        let mut nx = px + vx * th2.cos() * dt;
        let mut ny = py + vx * th2.sin() * dt;
        let half_w = ROOM_W_M / 2.0 - WALL_MARGIN_M;
        let half_h = ROOM_H_M / 2.0 - WALL_MARGIN_M;
        nx = nx.clamp(-half_w, half_w);
        ny = ny.clamp(-half_h, half_h);
        self.pose = (nx, ny, th2);
        self.mileage += vx.abs() * dt;
        // 超声波近距闩：低于阈值置位；只能由 ultrasonic_rearm 显式恢复。
        let us = ray_hit(nx, ny, th2).min(3.99);
        if us < US_NEAR_M {
            self.hub.control.lock().expect("control poisoned").us_latched = true;
        }
    }

    /// 合成雷达（10Hz）与相机（5Hz）并写媒体快照。
    fn synthesize(&mut self, now: u64) {
        let phase = self.phase();
        // 雷达 10Hz：每 2 拍一帧
        if self.cycle % 2 == 0 {
            let (px, py, th) = self.pose;
            let mut pts: Vec<(f64, f64)> = Vec::with_capacity(360);
            let mut valid = 0usize;
            for i in 0..360usize {
                let deg = i as f64 * 360.0 / 360.0;
                let mut rng = ray_hit(px, py, th + deg.to_radians());
                // 周期性遮挡，模拟真实雷达的点丢失（fake_source.py:131-133）
                if (i + (phase * 12.0) as usize) % 97 < 3 {
                    rng = 0.0;
                }
                if rng > 0.0 {
                    rng = (rng + 0.02 * (i as f64 * 1.7 + phase * 3.0).sin()).max(0.12);
                    valid += 1;
                }
                pts.push((round2(deg), round3(rng)));
            }
            let mut media = media_slot().write().expect("media poisoned");
            media.scan = Arc::new(pts);
            media.scan_valid = valid;
            media.scan_seq += 1;
            media.scan_hz = SCAN_HZ;
        }
        // 相机 5Hz：每 4 拍一帧
        if self.cycle % 4 == 0 {
            let rows = camera_rows(CAM_W, CAM_H, phase);
            let png = png_bytes(CAM_W, CAM_H, &rows);
            use base64::Engine as _;
            let b64 = base64::engine::general_purpose::STANDARD.encode(&png);
            let mut media = media_slot().write().expect("media poisoned");
            media.image_b64 = Arc::from(b64.as_str());
            media.image_w = CAM_W;
            media.image_h = CAM_H;
            media.image_seq += 1;
            media.image_mime = "image/png";
            media.image_fps = IMAGE_HZ;
        }
        // 位姿 20Hz 内部更新（ws_video 按 5Hz 推帧）
        {
            let mut media = media_slot().write().expect("media poisoned");
            media.pose = self.pose;
            media.pose_mono_ms = now;
        }
    }

    /// 构建并写入状态快照（20Hz）。
    fn publish_status(&mut self, now: u64) {
        let phase = self.phase();
        let unix = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0);
        let (vx, wz) = self.cmd;
        let stop_flag = {
            // 24s 脚本最后 6 秒固件停车保持（fake_source.py:159-160）；
            // 任务驱动或锁停时只是零速，不是固件失能。
            self.cmd_source == "demo" && (phase % CYCLE_S) >= 18.0
        };
        let load = (vx.abs() / 0.42).clamp(0.0, 1.5);
        // 6S 动态电压：满电 24.5V 附近，随负载下坠，停车回升
        // （fake_source.py:161-165 的负载模型按 6S 标定折算）。
        let voltage = if stop_flag {
            24.47 - 0.04 * (phase * 0.7).sin()
        } else {
            24.45 - 0.62 * load - 0.05 * (phase * 0.7).sin()
        };
        let voltage_v = round2(voltage);
        let us_raw = ray_hit(self.pose.0, self.pose.1, self.pose.2).min(3.99);
        let us_m = round3(us_raw);

        let (locked, lock_reason, cruise_on, us_latched, licensed, manual_dbg) = {
            let ctl = self.hub.control.lock().expect("control poisoned");
            let licensed = match ctl.license.last_rx_ms {
                Some(t) => now.saturating_sub(t) <= safety::LICENSE_TTL_MS,
                None => false,
            };
            let manual = &ctl.manual;
            let manual_dbg = json!({
                "active": manual.owner.is_some(),
                "input": manual.speed,
                "direction": manual.direction,
                "remaining": manual.deadline_ms.map(|d| ((d as f64 - now as f64) / 1000.0).max(0.0)),
                "beats": manual.beats,
                "stops": manual.stops,
            });
            (
                ctl.locked,
                ctl.lock_reason.clone(),
                ctl.cruise_on,
                ctl.us_latched,
                licensed,
                manual_dbg,
            )
        };
        // 注意：patrol_status_frame / nav_view 内部会再锁 control，
        // 必须在上面的锁作用域结束之后调用（std Mutex 不可重入）。
        let patrol_frame = self.hub.patrol_status_frame();
        let nav_view = self.hub.nav_view();

        // 速度命令视图：nav=自动跟随命令，auto=巡航命令，manual=手动命令。
        let (nav_cmd, auto_cmd, manual_cmd) = match self.cmd_source {
            "manual" => (None, None, Some(self.cmd)),
            "nav" => (Some(self.cmd), None, None),
            "auto" => (None, Some(self.cmd), None),
            _ => (None, None, None),
        };
        let cmd_view = |cmd: Option<(f64, f64)>| CmdView {
            state: if cmd.is_some() { "fresh" } else { "unknown" },
            age: cmd.map(|_| 0.0),
            vx_m_s: cmd.map(|(vx, _)| round4(vx)),
            wz_rad_s: cmd.map(|(_, wz)| round4(wz)),
        };
        let drive_ready = if stop_flag { Some(false) } else { None };

        let wheels = wheel_view(round4(vx), round4(wz));
        let battery = BatteryView::from_voltage(Some(voltage_v));
        // 任务状态机：手动 > 巡逻 > 导航/巡航 > 无任务。
        let nav_fresh = nav_cmd.is_some();
        let task_state = TaskState::decide(
            nav_fresh || manual_cmd.is_some(),
            self.cmd.0,
            self.cmd.1,
            if nav_fresh || manual_cmd.is_some() { Some(0.0) } else { None },
            manual_cmd.is_some(),
            false, // 电池低电走真实电压，Fake 不伪造低电量回充
            patrol_frame.get("status").and_then(Value::as_str) == Some("running"),
        );
        let task = TaskView {
            state: task_state.as_str(),
            text: task_state.text(),
            cmd_vx: if nav_fresh { Some(round4(vx)) } else { None },
            cmd_wz: if nav_fresh { Some(round4(wz)) } else { None },
            channel: true,
            manual: manual_dbg,
        };

        // /drive_safety 展示视图：Fake 模式下由许可+锁停+超声门直接合成。
        self.ds_seq += 1;
        let ds_allowed = !locked && licensed && !us_latched;
        let ds_reason = if locked {
            format!("锁停：{lock_reason}")
        } else if !licensed {
            "运动许可不新鲜（断流超 350ms）".to_string()
        } else if us_latched {
            "超声波近距/故障锁存，需显式恢复".to_string()
        } else {
            "许可有效，运动方向门放行".to_string()
        };
        let ds_view = DriveSafetyView {
            state: "fresh",
            allowed: Some(ds_allowed),
            reason: Some(ds_reason.clone()),
            age: Some(0.0),
            seq: Some(self.ds_seq as i64),
            motion: None,
            ultrasonic: Some(json!({
                "m": us_m,
                "ok": !us_latched,
                "latched": us_latched,
            })),
        };
        let mut nav_full = nav_view;
        if let Value::Object(map) = &mut nav_full {
            map.insert("safety".into(), serde_json::to_value(&ds_view).unwrap_or(Value::Null));
        }

        let (scan_valid, scan_frames, image_fps) = {
            let media = media_slot().read().expect("media poisoned");
            (media.scan_valid, media.scan_seq, media.image_fps)
        };

        let boot = boot_view(&self.hub.control.lock().expect("control poisoned").boot_marker);

        let payload = StatusPayload {
            t: "status",
            mode: self.mode,
            chassis: ChassisView {
                state: "fresh",
                age: Some(0.0),
                voltage_v: Some(voltage_v),
                vx_m_s: Some(round4(vx)),
                vy_m_s: Some(0.0),
                wz_rad_s: Some(round4(wz)),
                stop_flag: Some(if stop_flag { 1 } else { 0 }),
                stop_text: if stop_flag { "软件失能" } else { "软件未失能；就绪未知" },
                drive_ready,
                mileage_m: Some(round2(self.mileage)),
                ultrasonic_m: Some(us_m),
                wheels: Some(wheels),
                hz: Some(20.0),
            },
            imu: ImuView { state: "fresh", age: Some(0.0), hz: Some(20.0) },
            cmd: CmdViews {
                nav: cmd_view(nav_cmd),
                demo: CmdView { state: "unknown", age: None, vx_m_s: None, wz_rad_s: None },
                manual: cmd_view(manual_cmd),
                auto: cmd_view(auto_cmd),
            },
            safety: json!({
                "drive_enable": true,
                "drive_input_enable": true,
                "drive_firmware_protocol": "baseline",
                "source": "假数据模式",
                "online": true,
                "checked_t": unix,
            }),
            drive_safety: ds_view,
            nav: nav_full,
            scan_hz: SCAN_HZ,
            image_fps,
            scan_frames: json!(scan_frames),
            scan_valid: json!(scan_valid),
            network: Value::Null,
            audio: Value::Null,
            sys: Value::Null,
            host: Value::Null,
            lidar_detail: json!({
                "coverage": 0.97,
                "frames_ok": scan_frames,
                "frames_bad": 0,
                "reconnects": 0,
                "port": "/dev/fake",
            }),
            boot,
            srv_epoch: self.srv_epoch,
            nodes: NodesView {
                camera_node: "online",
                lidar_node: "online",
                c50c_uart3_node: "online",
                chassis_status_node: "online",
                web_bridge: "online",
                scan_without_node: false,
            },
            task,
            battery,
            cruise: CruiseView { on: cruise_on, available: true },
            patrol: patrol_frame,
            moving: Vec::new(),
        };
        self.store.update(|st| *st = payload);
    }
}

fn round4(v: f64) -> f64 {
    (v * 10000.0).round() / 10000.0
}

fn wrap_pi(a: f64) -> f64 {
    let mut a = a.rem_euclid(2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    }
    a
}

/// 四驱反解轮组速度（chassis.py::true_wheel_speed / wheel_view 的移植；
/// 车轮速度是推算值，不是实测值）。ros 模式的底盘卡片复用同一实现
/// （chassis.py:125-140 同源），故 pub(crate)。
pub(crate) fn wheel_view(vx_m_s: f64, wz_rad_s: f64) -> Value {
    const HALF_WHEEL_SPACING_M: f64 = 0.230;
    const HALF_AXLE_SPACING_M: f64 = 0.200;
    const WHEEL_DIAMETER_M: f64 = 0.152;
    const TURN_LEVER_M: f64 = HALF_WHEEL_SPACING_M + HALF_AXLE_SPACING_M;
    const WHEEL_CIRCUMFERENCE_M: f64 = PI * WHEEL_DIAMETER_M;
    let lever = wz_rad_s * TURN_LEVER_M;
    let left = vx_m_s - lever;
    let right = vx_m_s + lever;
    let rpm = |v: f64| {
        if WHEEL_CIRCUMFERENCE_M <= 0.0 {
            0.0
        } else {
            v / WHEEL_CIRCUMFERENCE_M * 60.0
        }
    };
    json!({
        "left_m_s": round4(left),
        "right_m_s": round4(right),
        "left_rpm": (rpm(left) * 10.0).round() / 10.0,
        "right_rpm": (rpm(right) * 10.0).round() / 10.0,
        "per_wheel": false,
        "estimated": true,
        "turn_lever_m": TURN_LEVER_M,
        "wheel_diameter_m": WHEEL_DIAMETER_M,
        "half_wheel_spacing_m": HALF_WHEEL_SPACING_M,
        "half_axle_spacing_m": HALF_AXLE_SPACING_M,
        "note": "车轮速度由车体速度反解，参数未现场标定",
    })
}

/// 开机静止倒计时（console_views.py::boot_view 的移植）：
/// 标记文件是墙上时钟秒（浮点）；文件不存在/读不出/已超时 → 已结束。
/// ros 模式的状态装配复用同一实现，故 pub(crate)。
pub(crate) fn boot_view(marker: &std::path::Path) -> BootView {
    let mut view = BootView { initializing: false, remain_s: 0.0, hold_s: crate::state::BOOT_HOLD_S };
    let Ok(text) = std::fs::read_to_string(marker) else {
        return view;
    };
    let Ok(boot_epoch) = text.trim().parse::<f64>() else {
        return view;
    };
    let wall_now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0);
    let elapsed = wall_now - boot_epoch;
    if elapsed < 0.0 {
        // 时钟对不上（上次开机留下的）：当成已结束，不显示负数
        return view;
    }
    let remain = crate::state::BOOT_HOLD_S - elapsed;
    if remain <= 0.0 {
        return view;
    }
    view.initializing = true;
    view.remain_s = (remain * 10.0).round() / 10.0;
    view
}

/// `--fake` 时启动 FakeEngine：建栅格、登记地图标识、spawn 20Hz 任务。
pub async fn maybe_start(cfg: &Arc<Config>) {
    if !cfg.fake {
        eprintln!("ros 模式：ROS 2 Bridge 为预留接口层，暂无数据源（蓝图 §3）");
        return;
    }
    let store = crate::state::global();
    let hub = safety::global();
    let (map_id, grid) = build_grid();
    {
        let mut media = media_slot().write().expect("media poisoned");
        media.grid = Arc::new(grid);
    }
    hub.control.lock().expect("control poisoned").current_map_id = Some(map_id);
    let engine = FakeEngine::new(store, hub, "fake");
    tokio::spawn(engine.run());
    eprintln!("--fake：FakeEngine 已启动（20Hz 底盘 / 10Hz 雷达 / 5Hz 相机 / 1Hz 地图 / 24s 周期脚本）");
}
