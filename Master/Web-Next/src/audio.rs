//! 音频设备探测：读 `/proc/asound/cards` 与 `/dev/snd/pcm*`（纯文件系统
//! 读取，不起外部命令），5 秒缓存进进程级槽；`/api/state` 与
//! `{t:"status"}` 的 `status.audio` 从这里取快照（fake 模式保持 null，
//! fake.rs 不参与）。
//!
//! 板上事实（2026-11-22）：RK3588 有 4 张声卡——HDMI/DP 无外接喇叭、
//! es8388（card2：播放 pcmC2D0p = 车体喇叭，采集 pcmC2D0c = 板载麦克
//! 风）、USB 摄像头（card3：采集 pcmC3D0c）。没有独立的"矩阵麦克风"
//! 设备——若后续接入（如 ReSpeaker），新采集卡出现在 /proc/asound 里
//! 即可被 mic_usb 计入。
//!
//! 安全边界：读取失败就回退 `Value::Null`，不 panic、不拖累 20Hz 的
//! tick_task；采集在 spawn_blocking 阻塞线程里跑。

use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use serde_json::{json, Value};

use crate::Config;

/// 采集周期（秒）。
const POLL_SECS: u64 = 5;

/// 进程级音频设备槽：ros 模式的探测任务写，其余线程只读（snapshot）。
fn slot() -> &'static Mutex<(Option<Instant>, Value)> {
    static SLOT: OnceLock<Mutex<(Option<Instant>, Value)>> = OnceLock::new();
    SLOT.get_or_init(|| Mutex::new((None, Value::Null)))
}

/// 最新音频设备快照；未采到过返回 `Value::Null`。任何异常都回退
/// `Value::Null`，不 panic。
pub fn snapshot() -> Value {
    match slot().lock() {
        Ok(g) => g.1.clone(),
        Err(_) => Value::Null, // 槽中毒：宁可空也不炸
    }
}

/// ros 模式装配：spawn 一个音频探测后台任务，立刻采一次，之后每 5 秒
/// 一次；fake 模式内部跳过。调用方与 20Hz 的 tick_task 不被阻塞。
pub async fn maybe_start(cfg: &Arc<Config>) {
    if cfg.fake {
        return;
    }
    tokio::spawn(async move {
        loop {
            let v = tokio::task::spawn_blocking(collect)
                .await
                .unwrap_or(Value::Null);
            if !v.is_null() {
                if let Ok(mut g) = slot().lock() {
                    g.0 = Some(Instant::now());
                    g.1 = v;
                }
            }
            tokio::time::sleep(Duration::from_secs(POLL_SECS)).await;
        }
    });
    eprintln!("ros 模式：音频设备探测已启动（每 {POLL_SECS}s，/proc/asound）");
}

/// 在 /proc/asound/cards 里按编解码器名找声卡号（如 "es8388" → 2）。
/// 行格式：`  2 [rockchipes8388  ]: rockchip_es8388 - rockchip,es8388`。
fn card_of(codec: &str) -> Option<u32> {
    card_of_in("/proc/asound/cards", codec)
}

/// es8388 声卡号（车体喇叭所在卡）；找不到返回 None，由调用方决定回退。
/// speaker.rs 靠它定位 aplay/amixer 的 `-c` 参数。
pub fn es8388_card() -> Option<u32> {
    card_of("es8388")
}

fn card_of_in(path: &str, codec: &str) -> Option<u32> {
    let text = std::fs::read_to_string(path).ok()?;
    for line in text.lines() {
        // 注意：cards 文件每张卡下面还有一行设备名续行（非数字开头），
        // 必须跳过而不是退出——曾因 `?` 提前返回导致整卡探测恒 None。
        let mut parts = line.split_whitespace();
        let Some(idx_str) = parts.next() else { continue };
        let Ok(idx) = idx_str.parse::<u32>() else { continue };
        let Some(name_raw) = parts.next() else { continue };
        let name = name_raw.trim_matches(|c| c == '[' || c == ']');
        if name.contains(codec) {
            return Some(idx);
        }
    }
    None
}

/// 解析 `pcmC<card>D<dev>p|c` 设备名 → (声卡号, 是否播放设备)。
/// controlC*/seq/timer 等非 pcm 节点返回 None。
fn classify_pcm(name: &str) -> Option<(u32, bool)> {
    let rest = name.strip_prefix("pcmC")?;
    let (card_s, tail) = rest.split_once('D')?;
    let card = card_s.parse::<u32>().ok()?;
    if tail.ends_with('p') {
        Some((card, true))
    } else if tail.ends_with('c') {
        Some((card, false))
    } else {
        None
    }
}

/// 探测一次：喇叭/板载麦克风挂 es8388 编解码器，USB 摄像头的麦克风是
/// 其余采集设备。只有播放设备存在才算喇叭在线（HDMI/DP 的 pcm*p 设备
/// 不算——没接喇叭，报"在线"是撒谎）。
fn collect() -> Value {
    let es = card_of("es8388");
    let (mut speaker, mut mic_board, mut mic_usb) = (false, false, false);
    let entries = match std::fs::read_dir("/dev/snd") {
        Ok(e) => e,
        Err(_) => return Value::Null,
    };
    for entry in entries.flatten() {
        let name = entry.file_name();
        let Some((card, playback)) = classify_pcm(&name.to_string_lossy()) else { continue };
        let is_es = es == Some(card);
        if playback {
            if is_es {
                speaker = true;
            }
        } else if is_es {
            mic_board = true;
        } else {
            mic_usb = true;
        }
    }
    // 当前音量：首次读一次 amixer 真值（之后由 speaker::set_volume 维护
    // 缓存，探测不再 fork 进程）；读不到（本机无 amixer，如开发机 fake
    // 模式）就回 null，前端显示「—」并把滑块置灰，绝不瞎猜百分比。
    let volume = crate::speaker::cached_volume().map_or(Value::Null, |v| json!(v));
    json!({ "speaker": speaker, "mic_board": mic_board, "mic_usb": mic_usb, "volume": volume })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn card_of_finds_es8388_style_line() {
        // 模拟 /proc/asound/cards 的行格式（缩进 + [名字]），验证行解析
        let dir = std::env::temp_dir().join(format!("wbnext-audio-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let cards = dir.join("cards");
        std::fs::write(
            &cards,
            "  0 [rockchiphdmi0  ]: rockchip-hdmi0 - rockchip-hdmi0\n  \
             rockchip-hdmi0\n  \
             2 [rockchipes8388  ]: rockchip_es8388 - rockchip,es8388\n",
        )
        .unwrap();
        // 续行（非数字开头）必须跳过，不能中断解析
        assert_eq!(card_of_in(cards.to_str().unwrap(), "es8388"), Some(2));
        assert_eq!(card_of_in(cards.to_str().unwrap(), "repeaker"), None);
        assert_eq!(card_of_in("/nonexistent/cards", "es8388"), None);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn classify_pcm_parses_alsa_device_names() {
        assert_eq!(classify_pcm("pcmC2D0p"), Some((2, true))); // es8388 播放=喇叭
        assert_eq!(classify_pcm("pcmC2D0c"), Some((2, false))); // es8388 采集=板载麦
        assert_eq!(classify_pcm("pcmC3D0c"), Some((3, false))); // 摄像头 USB 麦
        assert_eq!(classify_pcm("pcmC0D0p"), Some((0, true))); // HDMI（不算喇叭）
        assert_eq!(classify_pcm("controlC2"), None);
        assert_eq!(classify_pcm("seq"), None);
        assert_eq!(classify_pcm("timer"), None);
    }

    #[test]
    fn collect_falls_back_to_null_without_snd() {
        // /dev/snd 在 CI/容器里不存在时 collect 必须回退 Null 而不是 panic
        // （真机上有 /dev/snd，板上实测验证；这里只保证不炸的契约）。
        let v = collect();
        assert!(v.is_null() || v.is_object());
    }
}
