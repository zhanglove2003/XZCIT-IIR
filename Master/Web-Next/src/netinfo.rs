//! WiFi/网络信息采集器：ros 模式（非 fake）每 5 秒用 `spawn_blocking`
//! 跑一次 `iw` / `ip` 采集，结果写进进程级槽；`/api/state` 与
//! `{t:"status"}` 的 `status.network` 从这里取快照（fake 模式保持 null，
//! fake.rs 不参与）。
//!
//! 板上事实（2026-11-22）：RK3588 OrangePi，非特权用户可直接跑
//! /usr/sbin/iw 并读 /sys/class/net/<if>/address；`iw dev` 输出含
//! "Interface wlan0"，SSID "103"、freq 5180、signal -55 dBm、
//! RX 433.3 / TX 292.5 Mbit/s。
//!
//! 安全边界：所有外部命令都在 spawn_blocking 阻塞线程里跑，任何失败/
//! 超时都不 panic、都不拖累 20Hz 的 tick_task；采集不到就回退
//! `{"state":"absent",...}` 或保持 null。

use std::process::Command;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use serde_json::{json, Value};

use crate::Config;

/// 采集周期（秒）。
const POLL_SECS: u64 = 5;
/// 单条外部命令超时（coreutils `timeout 2`，秒）。
const CMD_TIMEOUT_SECS: u64 = 2;

/// 进程级网络信息槽：ros 模式的采集任务写，其余线程只读（snapshot）。
/// 元组为（采样时刻, 最近一次网络信息 JSON）；未采到前是 (None, Null)。
fn slot() -> &'static Mutex<(Option<Instant>, Value)> {
    static SLOT: OnceLock<Mutex<(Option<Instant>, Value)>> = OnceLock::new();
    SLOT.get_or_init(|| Mutex::new((None, Value::Null)))
}

/// 最新网络信息快照；未采到过返回 `Value::Null`。`age_s` 读取时按采样
/// 时刻补算（保持新鲜度语义），任何异常都回退 `Value::Null`，不 panic。
pub fn snapshot() -> Value {
    let (taken_at, mut v) = match slot().lock() {
        Ok(g) => (g.0, g.1.clone()),
        Err(_) => return Value::Null, // 槽中毒：宁可空也不炸
    };
    if let Value::Object(map) = &mut v {
        map.insert(
            "age_s".into(),
            taken_at
                .map(|t| json!(t.elapsed().as_secs_f64()))
                .unwrap_or(json!(0.0)),
        );
    }
    v
}

/// ros 模式装配：spawn 一个网络采集后台任务，立刻采一次，之后每 5 秒
/// 一次；fake 模式内部跳过。调用方与 20Hz 的 tick_task 不被阻塞。
pub async fn maybe_start(cfg: &Arc<Config>) {
    if cfg.fake {
        return;
    }
    tokio::spawn(async move {
        loop {
            // spawn_blocking：iw/ip/timeout 都在阻塞线程里跑，绝不拖累
            // axum worker 与 20Hz 的 tick_task。
            let v = match tokio::task::spawn_blocking(collect).await {
                Ok(v) => v,
                Err(_) => Value::Null, // 采集线程异常 → 保持上次数据
            };
            if !v.is_null() {
                if let Ok(mut g) = slot().lock() {
                    g.0 = Some(Instant::now());
                    g.1 = v;
                }
            }
            tokio::time::sleep(Duration::from_secs(POLL_SECS)).await;
        }
    });
    eprintln!("ros 模式：WiFi 网络采集器已启动（每 {POLL_SECS}s，iw/ip）");
}

/// 命令定位：PATH 里找不到时回退到常见绝对路径（systemd 等精简环境
/// 的 PATH 常缺 /usr/sbin）；都没有返回 None。
fn which(name: &str, abs: &[&str]) -> Option<String> {
    let in_path = std::env::var("PATH").is_ok_and(|path| {
        path.split(':')
            .any(|dir| std::path::Path::new(dir).join(name).is_file())
    });
    if in_path {
        return Some(name.to_string());
    }
    abs.iter()
        .find(|p| std::path::Path::new(p).is_file())
        .map(|p| (*p).to_string())
}

/// 在 spawn_blocking 线程里跑外部命令：优先 coreutils `timeout 2 <cmd>`
/// 包裹，timeout 找不到/起不来就退回直接跑（损失超时保护但保功能）；
/// 任何失败返回 None，不 panic。
fn run_cmd(name: &str, abs: &[&str], args: &[&str]) -> Option<String> {
    let bin = which(name, abs).unwrap_or_else(|| name.to_string());
    if let Some(t) = which("timeout", &["/usr/bin/timeout", "/bin/timeout"]) {
        let out = Command::new(t)
            .arg(CMD_TIMEOUT_SECS.to_string())
            .arg(&bin)
            .args(args)
            .output();
        if let Ok(o) = out {
            // 非零退出码不一定代表失败（iw 未连接时退出码 1 但 stdout
            // 有 "Not connected."），stdout 内容才是解析的输入。
            return Some(String::from_utf8_lossy(&o.stdout).into_owned());
        }
    }
    // timeout 不可用（spawn 失败）→ 直接跑。
    Command::new(bin)
        .args(args)
        .output()
        .ok()
        .map(|o| String::from_utf8_lossy(&o.stdout).into_owned())
}

// ---------------------------------------------------------------------------
// 纯解析函数（&str 入、结构出）：单测覆盖，任何输入不 panic
// ---------------------------------------------------------------------------

/// 取「key 开头」的行，返回冒号后的内容（去首尾空白）。key 可带冒号
/// （"RX:"）也可不带（"SSID"）；SSID 串保留内部空格。
fn line_value<'a>(out: &'a str, key: &str) -> Option<&'a str> {
    out.lines().map(str::trim).find_map(|l| {
        let rest = l.strip_prefix(key)?;
        let v = rest
            .strip_prefix(':')
            .map(str::trim)
            .unwrap_or(rest)
            .trim();
        (!v.is_empty()).then_some(v)
    })
}

fn i64_of(s: &str) -> Option<i64> {
    s.split_whitespace().next()?.parse::<i64>().ok()
}
fn f64_of(s: &str) -> Option<f64> {
    s.split_whitespace().next()?.parse::<f64>().ok()
}

/// `iw dev <iface> link` 解析中间结果。
#[derive(Debug, Clone, PartialEq, Default)]
pub struct IWLink {
    /// 已连接（有 SSID 行且无 "Not connected"）。
    pub connected: bool,
    pub ssid: Option<String>,
    pub freq_mhz: Option<i64>,
    pub signal_dbm: Option<i64>,
    pub tx_mbps: Option<f64>,
    pub rx_mbps: Option<f64>,
}

/// 解析 `iw dev <iface> link` 的 stdout。未连接（"Not connected" / 无
/// SSID 行 / 空输出）→ connected=false。
pub fn parse_iw_link(out: &str) -> IWLink {
    let o = out.trim();
    let ssid = line_value(o, "SSID").map(str::to_string);
    let connected = !o.is_empty() && !o.contains("Not connected") && ssid.is_some();
    IWLink {
        connected,
        ssid,
        freq_mhz: line_value(o, "freq:").and_then(i64_of),
        signal_dbm: line_value(o, "signal:").and_then(i64_of),
        // 兼容新版 "RX: x Mbit/s" 与 station 风格 "RX bitrate: x MBit/s"。
        tx_mbps: ["TX bitrate:", "TX:"]
            .into_iter()
            .filter_map(|k| line_value(o, k))
            .filter_map(f64_of)
            .next(),
        rx_mbps: ["RX bitrate:", "RX:"]
            .into_iter()
            .filter_map(|k| line_value(o, k))
            .filter_map(f64_of)
            .next(),
    }
}

/// `quality = max(0, min(100, 2*(dbm+100)))`；越界自动收敛。
pub fn quality_signal(dbm: i64) -> i64 {
    (2 * (dbm + 100)).clamp(0, 100)
}

/// 频段：freq >= 5000 → "5 GHz"，否则 "2.4 GHz"（前端约定）。
pub fn band_of(freq_mhz: i64) -> &'static str {
    if freq_mhz >= 5000 {
        "5 GHz"
    } else {
        "2.4 GHz"
    }
}

/// 解析 `ip -4 -o addr show dev <iface>` 的 stdout，取首个 inet v4
/// 地址（去掉前缀长度 /24）；无 v4 地址返回 None。
pub fn parse_ipv4(out: &str) -> Option<String> {
    out.lines().find_map(|line| {
        let mut t = line.split_whitespace();
        t.find(|w| *w == "inet")?;
        let addr = t.next()?;
        let ip = addr.split('/').next().unwrap_or("");
        (!ip.is_empty()).then(|| ip.to_string())
    })
}

// ---------------------------------------------------------------------------
// 采集器
// ---------------------------------------------------------------------------

/// 无线网卡发现：/sys/class/net/*/wireless 目录存在即视为无线接口。
fn find_wlan_iface() -> Option<String> {
    let entries = std::fs::read_dir("/sys/class/net").ok()?;
    entries.flatten().find_map(|e| {
        let path = e.path();
        if path.join("wireless").is_dir() {
            path.file_name().and_then(|n| n.to_str()).map(str::to_string)
        } else {
            None
        }
    })
}

/// 采集一次（spawn_blocking 线程内调用）。无无线网卡 → `{"state":"absent",...}`；
/// 命令失败/超时 → 相应字段缺省（null/未连接）。不 panic。
fn collect() -> Value {
    // 无线网卡发现：没有也不崩，输出 absent。
    let Some(iface) = find_wlan_iface() else {
        return json!({
            "iface": "", "state": "absent",
            "ssid": null, "ip": null, "mac": null,
            "rssi": null, "signal_dbm": null,
            "freq_mhz": null, "band": null,
            "tx_mbps": null, "rx_mbps": null,
            "quality": null, "age_s": 0.0,
        });
    };

    // MAC：sysfs 直接读（板上事实：非特权用户可读）。
    let mac = std::fs::read_to_string(format!("/sys/class/net/{iface}/address"))
        .map(|s| s.trim().to_string())
        .ok()
        .filter(|s| !s.is_empty());

    // iw dev <iface> link：SSID / freq / signal / TX-RX bitrate。
    let iw_link = run_cmd(
        "iw",
        &["/usr/sbin/iw", "/sbin/iw"],
        &["dev", &iface, "link"],
    )
    .map(|s| parse_iw_link(&s))
    .unwrap_or_default();
    // ip -4 -o addr show dev <iface>：IPv4（可能没地址，解析得 None）。
    let ip = run_cmd(
        "ip",
        &["/usr/sbin/ip", "/bin/ip"],
        &["-4", "-o", "addr", "show", "dev", &iface],
    )
    .and_then(|s| parse_ipv4(&s));

    // rssi 与 signal_dbm 同值双写；quality 边界收敛；band 按 5000 分界。
    let rssi = iw_link.signal_dbm;
    let quality = iw_link.signal_dbm.map(quality_signal);
    let band = iw_link.freq_mhz.map(band_of);
    let state = if iw_link.connected { "connected" } else { "no-conn" };

    json!({
        "iface": iface,
        "state": state,
        "ssid": iw_link.ssid,
        "ip": ip,
        "mac": mac,
        "rssi": rssi,
        "signal_dbm": rssi,
        "freq_mhz": iw_link.freq_mhz,
        "band": band,
        "tx_mbps": iw_link.tx_mbps,
        "rx_mbps": iw_link.rx_mbps,
        "quality": quality,
        "age_s": 0.0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// iw：已连接完整输出（板上事实 5180 / -55 / 433.3 / 292.5）。
    #[test]
    fn iw_link_connected_full() {
        let out = "Connected to 3c:06:30:a8:b2:c4 (on wlan0)
\tSSID: 103
\tfreq: 5180
\tRX: 433.3 Mbit/s
\tTX: 292.5 Mbit/s
\tsignal: -55 dBm
";
        let lk = parse_iw_link(out);
        assert!(lk.connected);
        assert_eq!(lk.ssid.as_deref(), Some("103"));
        assert_eq!(lk.freq_mhz, Some(5180));
        assert_eq!(lk.signal_dbm, Some(-55));
        assert_eq!(lk.rx_mbps, Some(433.3));
        assert_eq!(lk.tx_mbps, Some(292.5));
    }

    /// iw：station 风格 "RX bitrate: x MBit/s" 也要能解析，SSID 含空格。
    #[test]
    fn iw_link_bitrate_style() {
        let out = "Connected to 00:11:22:33:44:55 (on wlan0)
\tSSID: My WiFi
\tRX bitrate: 144.4 MBit/s
\tTX bitrate: 130.0 MBit/s
\tsignal: -50 dBm
";
        let lk = parse_iw_link(out);
        assert!(lk.connected);
        assert_eq!(lk.ssid.as_deref(), Some("My WiFi"));
        assert_eq!(lk.rx_mbps, Some(144.4));
        assert_eq!(lk.tx_mbps, Some(130.0));
        assert_eq!(lk.signal_dbm, Some(-50));
    }

    /// iw：Not connected —— 未连接，字段全空。
    #[test]
    fn iw_link_not_connected() {
        let lk = parse_iw_link("Not connected.\n");
        assert!(!lk.connected);
        assert!(lk.ssid.is_none());
        assert!(lk.signal_dbm.is_none());
        assert!(lk.rx_mbps.is_none());
        assert!(lk.tx_mbps.is_none());
    }

    /// iw：有 freq 但无 SSID 行 → 判未连接（无 SSID 即未连），freq 仍解析。
    #[test]
    fn iw_link_no_ssid_line() {
        let out = "Connected to 00:11:22:33:44:55 (on wlan0)\n\tfreq: 2412\n";
        let lk = parse_iw_link(out);
        assert!(!lk.connected, "无 SSID 行必须判未连接");
        assert_eq!(lk.freq_mhz, Some(2412), "freq 仍应解析出来");
    }

    /// ip -4 -o：真实命令限定单接口（`show dev wlan0`），输出只有该
    /// 接口的行；取首个 inet v4 地址并丢前缀长度；无 v4 地址 None。
    #[test]
    fn ipv4_takes_first_inet() {
        let out = "2: wlan0    inet 192.168.31.103/24 brd 192.168.31.255 scope global wlan0\\       valid_lft forever preferred_lft forever\n";
        assert_eq!(parse_ipv4(out).as_deref(), Some("192.168.31.103"));
        assert!(parse_ipv4("").is_none());
        assert_eq!(
            parse_ipv4("2: wlan0    inet6 fe80::1/64 scope link\n").is_none(),
            true,
            "无 inet4 时不取 inet6"
        );
    }

    /// quality 边界：[-100,-50] 线性段内取值，越界 clamp 到 0/100。
    #[test]
    fn quality_bounds() {
        assert_eq!(quality_signal(-100), 0);
        assert_eq!(quality_signal(-55), 90, "板上 -55 dBm → 90");
        assert_eq!(quality_signal(-50), 100);
        assert_eq!(quality_signal(-120), 0, "低于 -100 clamp 到 0");
        assert_eq!(quality_signal(-40), 100, "高于 -50 clamp 到 100");
        assert!((0..=100).contains(&quality_signal(-55)));
    }

    /// band 分界：5000 归 "5 GHz"（前端约定）。
    #[test]
    fn band_boundary() {
        assert_eq!(band_of(2412), "2.4 GHz");
        assert_eq!(band_of(4999), "2.4 GHz");
        assert_eq!(band_of(5000), "5 GHz");
        assert_eq!(band_of(5180), "5 GHz");
    }
}