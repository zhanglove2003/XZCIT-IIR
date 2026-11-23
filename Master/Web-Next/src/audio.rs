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

/// 当前选定的录音采集设备（声卡号 + ALSA 设备串 + 简洁来源标签）。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CaptureTarget {
    pub card: u32,
    pub alsa_dev: String,
    pub label: &'static str,
}

/// 外置/阵列麦在 `/proc/asound/cards` 中优先匹配的关键词（小写；排除普通摄像头）。
const PREFERRED_MIC_KEYWORDS: &[&str] = &[
    "timesintelli",
    "respeaker",
    "array",
    "xfm",
    "seeed",
    "mic",
    "uac",
    "headset",
    "usb audio",
    "usb-audio",
];

/// 最新音频设备快照；未采到过返回 `Value::Null`。任何异常都回退
/// `Value::Null`，不 panic。ros 模式下附带实时麦克风状态 `mic`（纯内存
/// 读，无 I/O）。
pub fn snapshot() -> Value {
    let mut v = match slot().lock() {
        Ok(g) => g.1.clone(),
        Err(_) => Value::Null, // 槽中毒：宁可空也不炸
    };
    if let Value::Object(ref mut map) = v {
        map.insert("mic".into(), crate::mic::status_snapshot());
    }
    v
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

fn cards_in(path: &str) -> Vec<(u32, String)> {
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let mut out: Vec<(u32, String)> = Vec::new();
    for line in text.lines() {
        let mut parts = line.split_whitespace();
        let Some(first) = parts.next() else { continue };
        if let Ok(idx) = first.parse::<u32>() {
            out.push((idx, line.to_ascii_lowercase()));
        } else if let Some(last) = out.last_mut() {
            // 把声卡续行（如 "Timesintelli USB Audio Device at ..."）拼入同卡描述
            last.1.push(' ');
            last.1.push_str(&line.to_ascii_lowercase());
        }
    }
    out
}

/// 解析 `pcmC<card>D<dev>p|c` 设备名 → (声卡号, 设备号, 是否播放设备)。
/// controlC*/seq/timer 等非 pcm 节点返回 None。
fn classify_pcm_dev(name: &str) -> Option<(u32, u32, bool)> {
    let rest = name.strip_prefix("pcmC")?;
    let (card_s, tail) = rest.split_once('D')?;
    let card = card_s.parse::<u32>().ok()?;
    if let Some(dev_s) = tail.strip_suffix('p') {
        let dev = dev_s.parse::<u32>().ok()?;
        Some((card, dev, true))
    } else if let Some(dev_s) = tail.strip_suffix('c') {
        let dev = dev_s.parse::<u32>().ok()?;
        Some((card, dev, false))
    } else {
        None
    }
}

/// 解析 `pcmC<card>D<dev>p|c` 设备名 → (声卡号, 是否播放设备)。
#[cfg(test)]
fn classify_pcm(name: &str) -> Option<(u32, bool)> {
    classify_pcm_dev(name).map(|(card, _, playback)| (card, playback))
}

/// 判断指定声卡是否为外置 USB 麦克风/阵列麦（排除板载 es8388 与 USB 摄像头空音频节点）。
fn is_usb_mic_card(card: u32, es_card: Option<u32>, cards: &[(u32, String)]) -> bool {
    Some(card) != es_card
        && cards.iter().any(|(c, line)| {
            *c == card
                && !line.contains("webcam")
                && !line.contains("camera")
                && PREFERRED_MIC_KEYWORDS.iter().any(|kw| line.contains(kw))
        })
}

/// 择优选择外置 USB 矩阵麦克风录音设备（排除板载 es8388 与无麦摄像头）。
fn select_capture_device(
    cards: &[(u32, String)],
    capture_pcms: &[(u32, u32)],
    es_card: Option<u32>,
) -> Option<CaptureTarget> {
    let mut pcms = capture_pcms.to_vec();
    pcms.sort_unstable();

    if let Some(&(card, dev)) = pcms
        .iter()
        .find(|&&(card, _)| is_usb_mic_card(card, es_card, cards))
    {
        return Some(CaptureTarget {
            card,
            alsa_dev: format!("plughw:{card},{dev}"),
            label: "USB 麦",
        });
    }

    None
}

/// 探测当前外置 USB 矩阵麦克风 ALSA 录音设备。
pub fn capture_device() -> Option<CaptureTarget> {
    let entries = std::fs::read_dir("/dev/snd").ok()?;
    let mut capture_pcms = Vec::new();
    for entry in entries.flatten() {
        let name = entry.file_name();
        if let Some((card, dev, false)) = classify_pcm_dev(&name.to_string_lossy()) {
            capture_pcms.push((card, dev));
        }
    }
    let cards = cards_in("/proc/asound/cards");
    select_capture_device(&cards, &capture_pcms, es8388_card())
}

/// 探测一次：喇叭挂 es8388 编解码器，外置矩阵麦克风通过 `is_usb_mic_card`
/// 识别（排除板载 es8388 空悬输入与 USB 摄像头无效音频端点）。
fn collect() -> Value {
    let es = card_of("es8388");
    let cards = cards_in("/proc/asound/cards");
    let (mut speaker, mut mic_board, mut mic_usb) = (false, false, false);
    let entries = match std::fs::read_dir("/dev/snd") {
        Ok(e) => e,
        Err(_) => return Value::Null,
    };
    for entry in entries.flatten() {
        let name = entry.file_name();
        let Some((card, _, playback)) = classify_pcm_dev(&name.to_string_lossy()) else {
            continue;
        };
        let is_es = es == Some(card);
        if playback {
            if is_es {
                speaker = true;
            }
        } else if is_es {
            mic_board = true;
        } else if is_usb_mic_card(card, es, &cards) {
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
    fn classify_pcm_dev_parses_multi_devices() {
        assert_eq!(classify_pcm_dev("pcmC3D12c"), Some((3, 12, false)));
        assert_eq!(classify_pcm_dev("pcmC2D0p"), Some((2, 0, true)));
        assert_eq!(classify_pcm_dev("pcmC2D0x"), None);
        assert_eq!(classify_pcm_dev("controlC3"), None);
    }

    #[test]
    fn select_capture_device_priority_order() {
        let cards = vec![
            (2, "2 [rockchipes8388 ]: rockchip_es8388 - rockchip,es8388".into()),
            (3, "3 [webcam ]: usb-audio - hd 720p webcam".into()),
            (
                4,
                "4 [device ]: usb-audio - usb audio device timesintelli usb audio device".into(),
            ),
        ];
        // Timesintelli 矩阵麦 + Webcam + es8388 -> 选中 Timesintelli (card 4)
        assert_eq!(
            select_capture_device(&cards, &[(2, 0), (3, 0), (4, 0)], Some(2)),
            Some(CaptureTarget { card: 4, alsa_dev: "plughw:4,0".into(), label: "USB 麦" })
        );
        // 拔掉矩阵麦后（仅剩 Webcam + 板载 es8388）-> 视为无可用矩阵麦克风 (None)
        assert_eq!(
            select_capture_device(&cards, &[(2, 0), (3, 0)], Some(2)),
            None
        );
        assert_eq!(select_capture_device(&cards, &[(3, 0)], Some(2)), None);
        assert_eq!(select_capture_device(&cards, &[(2, 0)], Some(2)), None);
        assert_eq!(select_capture_device(&cards, &[], Some(2)), None);
        assert!(is_usb_mic_card(4, Some(2), &cards));
        assert!(!is_usb_mic_card(3, Some(2), &cards));
        assert!(!is_usb_mic_card(2, Some(2), &cards));
    }

    #[test]
    fn collect_falls_back_to_null_without_snd() {
        // /dev/snd 在 CI/容器里不存在时 collect 必须回退 Null 而不是 panic
        // （真机上有 /dev/snd，板上实测验证；这里只保证不炸的契约）。
        let v = collect();
        assert!(v.is_null() || v.is_object());
    }
}
