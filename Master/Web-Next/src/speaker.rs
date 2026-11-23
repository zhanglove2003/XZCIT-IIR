//! 车体喇叭：测试提示音 + 音量调节。
//!
//! 板上事实（2026-11-22）：喇叭挂 es8388 编解码器（card2，`pcmC2D0p`），
//! 系统里没装 PulseAudio/PipeWire，所以走 **raw ALSA**：`aplay` 播放 +
//! `amixer` 调 `PCM` 混音控件的音量（0–192，线性）。
//!
//! 不依赖 wav 文件：测试音是现场生成的 48 kHz/16 bit 单声道正弦，
//! 写进临时文件再交给 aplay（`aplay` 不接受 stdin 的 WAV 头）。三声
//! 用同一次播放（音频里自带静音间隔），避免连起三个进程把 CPU 打满。
//!
//! 安全边界：所有外部命令带超时，失败原样回传错误字符串，不 panic；
//! 阻塞调用只在 spawn_blocking 里跑，不占 tokio worker。

use std::io::Write as _;
use std::path::PathBuf;
use std::sync::{Mutex, OnceLock};
use std::time::Duration;

use serde_json::{Value, json};

/// 提示音参数：3 声 880 Hz / 120 ms，间隔 90 ms（总时长 ~0.54 s）。
const TONE_HZ: f64 = 880.0;
const TONE_MS: u32 = 120;
const TONE_COUNT: u32 = 3;
const GAP_MS: u32 = 90;
const RATE: u32 = 48_000;

/// 外部命令超时：播放 ~0.6 s，留足余量；amixer 远快于此。
const CMD_TIMEOUT: Duration = Duration::from_secs(8);

/// es8388 声卡号（`/proc/asound/cards` 里带 es8388 的那张）；找不到退回
/// card 2（板上实测编号，见 audio.rs 顶部注释）。
fn card() -> u32 {
    super::audio::es8388_card().unwrap_or(2)
}

/// 生成一次测试音 wav（三声，同一次播放），返回临时文件路径。
fn write_test_tone() -> std::io::Result<PathBuf> {
    let rate = RATE as usize;
    let tone_n = rate * TONE_MS as usize / 1000;
    let gap_n = rate * GAP_MS as usize / 1000;
    let fade = rate / 100; // 10 ms 淡入淡出，避免爆音
    let mut pcm: Vec<i16> = Vec::with_capacity(rate);
    for k in 0..TONE_COUNT {
        for i in 0..tone_n {
            let mut a = 1.0f64;
            if i < fade {
                a = i as f64 / fade as f64;
            } else if i + fade > tone_n {
                a = (tone_n - i) as f64 / fade as f64;
            }
            let s = 12_000.0 * a * (std::f64::consts::TAU * TONE_HZ * i as f64 / rate as f64).sin();
            pcm.push(s as i16);
        }
        if k + 1 < TONE_COUNT {
            pcm.extend(std::iter::repeat(0i16).take(gap_n));
        }
    }

    let mut body: Vec<u8> = Vec::with_capacity(pcm.len() * 2 + 44);
    let data_len = (pcm.len() * 2) as u32;
    body.extend_from_slice(b"RIFF");
    body.extend_from_slice(&(36 + data_len).to_le_bytes());
    body.extend_from_slice(b"WAVEfmt ");
    body.extend_from_slice(&16u32.to_le_bytes());
    body.extend_from_slice(&1u16.to_le_bytes()); // PCM
    body.extend_from_slice(&1u16.to_le_bytes()); // 单声道
    body.extend_from_slice(&RATE.to_le_bytes());
    body.extend_from_slice(&(RATE * 2).to_le_bytes()); // byte rate
    body.extend_from_slice(&2u16.to_le_bytes()); // block align
    body.extend_from_slice(&16u16.to_le_bytes()); // bits
    body.extend_from_slice(b"data");
    body.extend_from_slice(&data_len.to_le_bytes());
    for s in &pcm {
        body.extend_from_slice(&s.to_le_bytes());
    }

    let path = std::env::temp_dir().join(format!("wbnext-speaker-{}.wav", std::process::id()));
    let mut f = std::fs::File::create(&path)?;
    f.write_all(&body)?;
    Ok(path)
}

/// 跑一条外部命令；超时/启动失败都折算成 Err(消息)。
fn run(cmd: &str, args: &[String]) -> Result<String, String> {
    let mut child = std::process::Command::new(cmd)
        .args(args)
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .map_err(|e| format!("无法启动 {cmd}：{e}"))?;
    let deadline = std::time::Instant::now() + CMD_TIMEOUT;
    loop {
        match child.try_wait() {
            Ok(Some(st)) => {
                let out = child.wait_with_output().map_err(|e| e.to_string())?;
                let text = String::from_utf8_lossy(&out.stderr).trim().to_string();
                return if st.success() {
                    Ok(text)
                } else {
                    Err(if text.is_empty() {
                        format!("{cmd} 退出码 {:?}", st.code())
                    } else {
                        text
                    })
                };
            }
            Ok(None) if std::time::Instant::now() < deadline => {
                std::thread::sleep(Duration::from_millis(20));
            }
            Ok(None) => {
                let _ = child.kill();
                let _ = child.wait();
                return Err(format!("{cmd} 超时（{}s）", CMD_TIMEOUT.as_secs()));
            }
            Err(e) => return Err(format!("{cmd} 运行失败：{e}")),
        }
    }
}

/// 播放三声测试音。阻塞 ~0.6 s，调用方放进 spawn_blocking。
pub fn play_test_tone() -> Result<Value, String> {
    let path = write_test_tone().map_err(|e| format!("生成测试音失败：{e}"))?;
    let dev = format!("plughw:{},0", card());
    let res = run(
        "aplay",
        &[
            "-q".into(),
            "-D".into(),
            dev.clone(),
            path.display().to_string(),
        ],
    );
    let _ = std::fs::remove_file(&path);
    res.map(|_| json!({ "device": dev, "tones": TONE_COUNT }))
}

/// 跑 amixer 取 stdout（`sget` 的正文在 stdout，run() 只回 stderr）。
fn amixer_stdout(args: &[&str]) -> Result<String, String> {
    let mut full_args = vec!["-c".to_string(), card().to_string()];
    full_args.extend(args.iter().map(|s| s.to_string()));
    let out = std::process::Command::new("amixer")
        .args(&full_args)
        .output()
        .map_err(|e| format!("无法启动 amixer：{e}"))?;
    let err = String::from_utf8_lossy(&out.stderr).trim().to_string();
    if !out.status.success() {
        return Err(if err.is_empty() {
            "amixer 执行失败".into()
        } else {
            err
        });
    }
    Ok(String::from_utf8_lossy(&out.stdout).to_string())
}

/// 当前音量百分比（0–100）：读 `amixer sget PCM` 的 `[NN%]`。
pub fn volume_percent() -> Result<u32, String> {
    let text = amixer_stdout(&["sget", "PCM"])?;
    parse_percent(&text).ok_or_else(|| "无法解析音量".to_string())
}

/// 从 `amixer` 输出里抽第一个 `[NN%]`。
fn parse_percent(text: &str) -> Option<u32> {
    let start = text.find('[')?;
    let rest = &text[start + 1..];
    let end = rest.find('%')?;
    rest[..end].trim().parse().ok()
}

/// 音量缓存的进程级单例槽（读/写共用，别各开一个 static）。
fn volume_slot() -> &'static Mutex<Option<u32>> {
    static V: OnceLock<Mutex<Option<u32>>> = OnceLock::new();
    V.get_or_init(|| Mutex::new(None))
}

/// 当前音量（0–100）。首个请求时从 amixer 读真值并缓存，之后由
/// `set_volume` 维护——5 s 一次的状态探测因此只 fork 一次进程，不持续
/// 吃 CPU。读不到（本机没有 amixer）就回 None，让 UI 显示「—」。
pub fn cached_volume() -> Option<u32> {
    let m = volume_slot();
    if let Ok(g) = m.lock() {
        if let Some(v) = *g {
            return Some(v);
        }
    }
    let v = volume_percent().ok()?;
    remember_volume(v);
    Some(v)
}

/// 记下刚设置成功的音量。
fn remember_volume(v: u32) {
    let m = volume_slot();
    if let Ok(mut g) = m.lock() {
        *g = Some(v);
    }
}

/// 设置音量：把百分比换成 amixer 的原始刻度（0–192）。
pub fn set_volume(pct: u32) -> Result<u32, String> {
    let pct = pct.min(100);
    run(
        "amixer",
        &[
            "-c".into(),
            card().to_string(),
            "sset".into(),
            "PCM".into(),
            format!("{pct}%"),
        ],
    )?;
    // 回读实际值：amixer 会把百分比取整到原始刻度，回读保证滑块显示的是真值
    let applied = volume_percent().unwrap_or(pct);
    remember_volume(applied);
    Ok(applied)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_percent_reads_alsa_brackets() {
        let s = "Simple mixer control 'PCM',0\n  Front Left: 134 [70%] [-29.00dB]\n  Front Right: 134 [70%] [-29.00dB]";
        assert_eq!(parse_percent(s), Some(70));
        assert_eq!(parse_percent("no brackets"), None);
        assert_eq!(parse_percent("[100%] [0.00dB]"), Some(100));
    }

    #[test]
    fn test_tone_is_valid_wav() {
        let p = write_test_tone().unwrap();
        let bytes = std::fs::read(&p).unwrap();
        std::fs::remove_file(&p).ok();
        assert!(bytes.len() > 44);
        assert_eq!(&bytes[0..4], b"RIFF");
        assert_eq!(&bytes[8..12], b"WAVE");
        // 三声 + 两段间隔 ≈ 0.54 s @48k mono 16bit ≈ 51.8 KB
        let samples = (bytes.len() - 44) / 2;
        assert!(samples > RATE as usize / 2, "samples={samples}");
    }
}
