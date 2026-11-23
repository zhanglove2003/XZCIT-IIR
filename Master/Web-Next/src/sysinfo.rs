//! 系统占用采集器（CPU / GPU / 内存 / 磁盘）：ros 模式（非 fake）每 5 秒
//! 用 `spawn_blocking` 采样一次，结果写进进程级槽；`/api/state` 与
//! `{t:"status"}` 的 `status.sys` 从这里取快照（fake 模式保持 null，
//! fake.rs 不参与）。
//!
//! 板上事实（2026-11-22，RK3588 OrangePi 5 Pro）：
//! - CPU 聚合占用取 /proc/stat 首行，与上次采样做差分（首拍无前值，
//!   cpu 字段缺失，前端显示 —）；
//! - GPU 是 Mali，利用率在 `/sys/class/devfreq/fb000000.gpu/load`，格式
//!   `<util>@<freq>Hz`（实测 `0@300000000Hz`）；
//! - 内存取 /proc/meminfo 的 (MemTotal − MemAvailable) / MemTotal
//!   （实测 8G，含页缓存回收的可用量，比 used/MemTotal 更贴近体感）；
//! - 磁盘取 `df -P /` 的 Capacity 列（root 在 /dev/mmcblk0p1，实测 4%）；
//! - 温度取 thermal_zone0 的 SoC 主温度（毫摄氏度，实测 38.8°C；板上另有
//!   bigcore0/1、little、center、gpu、npu 七个区，界面只报代表性主温）。
//!
//! 安全边界：所有读取/命令都在 spawn_blocking 阻塞线程里跑，任何失败
//! 都不 panic、不拖累 20Hz 的 tick_task；采不到的字段缺省（null）。

use std::process::Command;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

use serde_json::{Value, json};

use crate::Config;

/// 采集周期（秒）。
const POLL_SECS: u64 = 5;
/// df 命令超时（coreutils timeout，秒）。
const DF_TIMEOUT_SECS: u64 = 2;
/// GPU devfreq 路径（RK3588 Mali 固定挂这里；路径缺失 → gpu 字段缺省）。
const GPU_LOAD_PATH: &str = "/sys/class/devfreq/fb000000.gpu/load";
/// SoC 主温度区（thermal_zone0 = soc-thermal，毫摄氏度整数）。
const SOC_THERMAL_ZONE: &str = "/sys/class/thermal/thermal_zone0/temp";

/// 进程级系统占用槽：ros 模式的采集任务写，其余线程只读（snapshot）。
fn slot() -> &'static Mutex<Value> {
    static SLOT: OnceLock<Mutex<Value>> = OnceLock::new();
    SLOT.get_or_init(|| Mutex::new(Value::Null))
}

/// 最新系统占用快照；未采到过返回 `Value::Null`。任何异常都回退
/// `Value::Null`，不 panic。
pub fn snapshot() -> Value {
    match slot().lock() {
        Ok(g) => g.clone(),
        Err(_) => Value::Null, // 槽中毒：宁可空也不炸
    }
}

/// ros 模式装配：spawn 一个采集后台任务，立刻采一次，之后每 5 秒一次；
/// fake 模式内部跳过。调用方与 20Hz 的 tick_task 不被阻塞。
pub async fn maybe_start(cfg: &Arc<Config>) {
    if cfg.fake {
        return;
    }
    tokio::spawn(async move {
        let mut prev: Option<(u64, u64)> = None; // (total, idle) 差分基线
        loop {
            // spawn_blocking：/proc 读取与 df 都在阻塞线程里跑，绝不拖累
            // axum worker 与 20Hz 的 tick_task。基线随闭包逐轮传递。
            let p = prev;
            let (v, next) = match tokio::task::spawn_blocking(move || collect(p)).await {
                Ok(pair) => pair,
                Err(_) => (Value::Null, prev), // 采集线程异常 → 保持上次数据
            };
            if !v.is_null() {
                if let Ok(mut g) = slot().lock() {
                    *g = v;
                }
            }
            prev = next;
            tokio::time::sleep(Duration::from_secs(POLL_SECS)).await;
        }
    });
    eprintln!("ros 模式：系统占用采集器已启动（每 {POLL_SECS}s，/proc/stat + devfreq + df）");
}

/// 读 /proc/stat 首行 → (总 jiffies, idle+iowait)。
fn cpu_jiffies() -> Option<(u64, u64)> {
    let line = std::fs::read_to_string("/proc/stat").ok()?;
    let line = line.lines().next()?;
    let mut it = line.split_whitespace().skip(1); // 跳过 "cpu" 标签
    let mut total = 0u64;
    let mut idle = 0u64;
    for (i, f) in it.by_ref().enumerate() {
        let v = f.parse::<u64>().ok()?;
        total += v;
        if i == 3 || i == 4 {
            idle += v; // idle(第4列) + iowait(第5列)
        }
        if i > 7 {
            break; // user..steal 共 8 列参与 total；guest 列已含在 user 里不计
        }
    }
    Some((total, idle))
}

/// Mali devfreq load：`<util>@<freq>Hz` → util（0-100，取整）。
fn gpu_load() -> Option<u32> {
    let raw = std::fs::read_to_string(GPU_LOAD_PATH).ok()?;
    let util = raw.split('@').next()?.trim().parse::<u32>().ok()?;
    Some(util.min(100))
}

/// 内存占用百分比：(MemTotal − MemAvailable) / MemTotal × 100。
fn mem_pct() -> Option<u32> {
    let text = std::fs::read_to_string("/proc/meminfo").ok()?;
    let field = |key: &str| -> Option<u64> {
        text.lines().find_map(|l| {
            l.strip_prefix(key)
                .map(|rest| rest.trim_start_matches(':').trim())
                .and_then(|v| v.split_whitespace().next()?.parse::<u64>().ok())
        })
    };
    let total = field("MemTotal")?;
    let avail = field("MemAvailable")?;
    if total == 0 {
        return None;
    }
    Some((((total - avail) as f64 / total as f64) * 100.0).round() as u32)
}

/// 根分区占用百分比：`df -P /` 的 Capacity 列（timeout 兜底）。
fn disk_pct() -> Option<u32> {
    let out = Command::new("timeout")
        .args([
            format!("{DF_TIMEOUT_SECS}s"),
            "df".to_string(),
            "-P".to_string(),
            "/".to_string(),
        ])
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let text = String::from_utf8_lossy(&out.stdout);
    let last = text.lines().last()?;
    let cap = last.split_whitespace().nth(4)?; // Filesystem 1024-blocks Used Avail Capacity
    cap.strip_suffix('%')?.parse::<u32>().ok()
}

/// SoC 温度（℃）：thermal_zone0/temp 毫摄氏度 → 0.1℃ 精度一位小数。
fn soc_temp() -> Option<f64> {
    let raw = std::fs::read_to_string(SOC_THERMAL_ZONE).ok()?;
    let milli = raw.trim().parse::<i64>().ok()?;
    if !(-40_000..=150_000).contains(&milli) {
        return None; // 明显不合理（读歪/裸寄存器值）宁可缺省
    }
    Some((milli as f64 / 1000.0 * 10.0).round() / 10.0)
}

/// 采样一次并产出 status.sys JSON；cpu 依赖差分基线（首拍缺 cpu 字段）。
fn collect(prev: Option<(u64, u64)>) -> (Value, Option<(u64, u64)>) {
    let cur = cpu_jiffies();
    let cpu = match (prev, cur) {
        (Some((pt, pi)), Some((ct, ci))) if ct > pt => {
            let busy = (ct - pt) - (ci.saturating_sub(pi));
            Some(
                (((busy as f64) / (ct - pt) as f64) * 100.0)
                    .round()
                    .clamp(0.0, 100.0) as u32,
            )
        }
        _ => None,
    };
    let v = json!({
        "cpu": cpu,
        "gpu": gpu_load(),
        "mem": mem_pct(),
        "disk": disk_pct(),
        "temp": soc_temp(),
    });
    (v, cur)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpu_jiffies_parses_proc_stat() {
        // 真机 /proc/stat 存在（CI 容器一般也有）；只验证"能读出且不炸"，
        // 数值合理性无法固定断言（其他测试线程在跑）。
        if let Some((total, idle)) = cpu_jiffies() {
            assert!(total > 0);
            assert!(idle <= total);
        }
    }

    #[test]
    fn gpu_load_format() {
        // load 文件格式 <util>@<freq>Hz；真机有 /sys/class/devfreq，
        // 容器里多半没有 → None 也是合法结果（不 panic 即契约）。
        if let Some(v) = gpu_load() {
            assert!(v <= 100);
        }
    }

    #[test]
    fn mem_pct_between_0_and_100() {
        if let Some(v) = mem_pct() {
            assert!(v <= 100);
        }
    }

    #[test]
    fn soc_temp_plausible_when_present() {
        // 真机 thermal_zone0 存在（容器多半没有 → None 合法，契约是不炸）；
        // 有值时必须在常识范围内，且只有一位小数精度。
        if let Some(t) = soc_temp() {
            assert!((-20.0..=110.0).contains(&t));
            assert_eq!(t, (t * 10.0).round() / 10.0);
        }
    }
}
