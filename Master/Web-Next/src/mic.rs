//! 车体麦克风录音与本机回放：控制页「矩阵麦克风」卡片的「开始录音 / 结束录音」
//! 与「播放录音」按钮由这里驱动。
//!
//! 板上事实（2026-11-23）：系统无 PulseAudio/PipeWire，走 raw ALSA
//! （`arecord` 采集 + `aplay` 回放）。`arecord` 先录原始 S16_LE 16 kHz
//! 单声道 PCM 到临时文件，再由本模块在停止后补 44 字节 RIFF/WAVE 头并
//! 原子切换为 `.wav`——避免 `SIGKILL` 留下损坏的 WAV 长度头，也无需引入
//! `libc`/`unsafe` 发 `SIGINT`。
//!
//! 安全边界：所有外部命令带硬上限或超时，阻塞调用只跑在 `spawn_blocking`
//! 里；进程级 `Mutex<MicState>` 只在微秒级状态切换时短暂持锁，绝不跨 I/O
//! 或 `sleep` 持锁。

use std::io::Read as _;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::{Mutex, OnceLock};
use std::time::{Duration, Instant};

use serde_json::{json, Value};

/// 单次录音硬上限（秒），传递给 `arecord -d` 并配看护任务兜底。
pub const MAX_RECORD_SECS: u64 = 60;

const SAMPLE_RATE: u32 = 16_000;
const CHANNELS: u16 = 1;
const BITS_PER_SAMPLE: u16 = 16;
const BYTES_PER_SEC: usize =
    (SAMPLE_RATE as usize) * (CHANNELS as usize) * (BITS_PER_SAMPLE as usize / 8);
const STARTUP_CHECK_MS: u64 = 120;

pub const NORM_MIN_PEAK: u16 = 800;
const NORM_TARGET_PEAK: f64 = 18_000.0;
const NORM_MAX_GAIN: f64 = 4.0;
const HPF_ALPHA: f64 = 0.905;

#[derive(Debug)]
struct ActiveRec {
    generation: u64,
    child: Child,
    raw_path: PathBuf,
    device: String,
    label: &'static str,
    started_at: Instant,
}

struct SavedRec {
    wav_path: PathBuf,
    duration_s: f64,
    device: String,
}

enum MicPhase {
    Idle,
    Starting,
    Recording(ActiveRec),
    Stopping,
    Playing,
}

struct MicState {
    phase: MicPhase,
    generation: u64,
    last: Option<SavedRec>,
}

impl MicState {
    fn new() -> Self {
        Self { phase: MicPhase::Idle, generation: 0, last: None }
    }

    fn phase_name(&self) -> &'static str {
        match self.phase {
            MicPhase::Idle => "idle",
            MicPhase::Starting => "starting",
            MicPhase::Recording(_) => "recording",
            MicPhase::Stopping => "stopping",
            MicPhase::Playing => "playing",
        }
    }

    fn snapshot(&self) -> Value {
        let recording = matches!(self.phase, MicPhase::Recording(_));
        let playing = matches!(self.phase, MicPhase::Playing);
        let has_recording = self.last.is_some();
        let duration_s = self.last.as_ref().map(|r| r.duration_s);
        let elapsed_s = match &self.phase {
            MicPhase::Recording(rec) => round_tenths(rec.started_at.elapsed().as_secs_f64()),
            _ => 0.0,
        };
        let device: Option<&str> = match &self.phase {
            MicPhase::Recording(rec) => Some(rec.device.as_str()),
            _ => self.last.as_ref().map(|r| r.device.as_str()),
        };
        json!({
            "phase": self.phase_name(),
            "recording": recording,
            "playing": playing,
            "has_recording": has_recording,
            "duration_s": duration_s,
            "elapsed_s": elapsed_s,
            "device": device,
        })
    }

    /// 开始录音前抢占 `Starting` 槽，返回本次代次号。
    fn try_begin_start(&mut self) -> Result<u64, String> {
        match self.phase {
            MicPhase::Idle => {
                self.phase = MicPhase::Starting;
                self.generation += 1;
                Ok(self.generation)
            }
            MicPhase::Starting | MicPhase::Recording(_) => Err("正在录音中".into()),
            MicPhase::Stopping => Err("录音正在保存，请稍候".into()),
            MicPhase::Playing => Err("正在播放录音，请稍候".into()),
        }
    }

    /// 结束录音前抢占 `Stopping` 槽；`expected_gen` 仅用于自动停止定时器校验代次。
    fn try_take_recording(&mut self, expected_gen: Option<u64>) -> Result<Option<ActiveRec>, String> {
        match &self.phase {
            MicPhase::Recording(rec) if expected_gen.map_or(true, |g| rec.generation == g) => {
                let prev = std::mem::replace(&mut self.phase, MicPhase::Stopping);
                match prev {
                    MicPhase::Recording(rec) => Ok(Some(rec)),
                    _ => unreachable!(),
                }
            }
            _ if expected_gen.is_some() => Ok(None),
            MicPhase::Starting => Err("录音正在启动，请稍后再结束".into()),
            MicPhase::Stopping => Err("录音正在保存，请稍候".into()),
            MicPhase::Idle | MicPhase::Recording(_) | MicPhase::Playing => {
                Err("当前未在录音".into())
            }
        }
    }

    /// 播放录音前抢占 `Playing` 槽，返回 WAV 路径与记录时长。
    fn try_begin_play(&mut self) -> Result<(PathBuf, f64), String> {
        match self.phase {
            MicPhase::Idle => {}
            MicPhase::Starting | MicPhase::Recording(_) => {
                return Err("正在录音中，请先结束录音".into());
            }
            MicPhase::Stopping => return Err("录音正在保存，请稍候".into()),
            MicPhase::Playing => return Err("正在播放录音".into()),
        }
        let missing = self.last.as_ref().map_or(true, |r| !r.wav_path.exists());
        if missing {
            let had_record = self.last.take().is_some();
            return Err(if had_record {
                "录音文件不存在，请重新录音".into()
            } else {
                "尚未录制声音，请先录音".into()
            });
        }
        let saved = self.last.as_ref().expect("checked exists");
        let wav_path = saved.wav_path.clone();
        let duration_s = saved.duration_s;
        self.phase = MicPhase::Playing;
        Ok((wav_path, duration_s))
    }
}

fn slot() -> &'static Mutex<MicState> {
    static MIC: OnceLock<Mutex<MicState>> = OnceLock::new();
    MIC.get_or_init(|| Mutex::new(MicState::new()))
}

fn round_tenths(secs: f64) -> f64 {
    (secs * 10.0).round() / 10.0
}

/// 若外部线程带了 `Starting` 阶段却提前出错/panic，确保退回 `Idle`。
struct PhaseResetGuard {
    expected_starting: bool,
    armed: bool,
}

impl PhaseResetGuard {
    fn for_starting() -> Self {
        Self { expected_starting: true, armed: true }
    }

    fn for_stopping() -> Self {
        Self { expected_starting: false, armed: true }
    }

    fn disarm(&mut self) {
        self.armed = false;
    }
}

impl Drop for PhaseResetGuard {
    fn drop(&mut self) {
        if !self.armed {
            return;
        }
        if let Ok(mut st) = slot().lock() {
            let should_reset = if self.expected_starting {
                matches!(st.phase, MicPhase::Starting)
            } else {
                matches!(st.phase, MicPhase::Stopping)
            };
            if should_reset {
                st.phase = MicPhase::Idle;
            }
        }
    }
}

/// 二阶一阶串联高通滤波（fc ≈ 260 Hz @ 16 kHz），滤除车体风扇与供电低频轰鸣（< 250 Hz），
/// 保留 300–4000 Hz 人声频段并消除直流偏置。
pub fn highpass_filter_pcm16(raw_pcm: &[u8]) -> Vec<u8> {
    let pcm_len = raw_pcm.len() & !1;
    let n = pcm_len / 2;
    if n == 0 {
        return Vec::new();
    }
    let mut buf: Vec<f64> = raw_pcm[..pcm_len]
        .chunks_exact(2)
        .map(|c| f64::from(i16::from_le_bytes([c[0], c[1]])))
        .collect();
    for _ in 0..2 {
        let mut prev_x = buf[0];
        let mut prev_y = 0.0;
        for sample in &mut buf {
            let x = *sample;
            let y = HPF_ALPHA * (prev_y + x - prev_x);
            *sample = y;
            prev_x = x;
            prev_y = y;
        }
    }
    let mut out = Vec::with_capacity(pcm_len);
    for y in buf {
        let s = y.round().clamp(-32768.0, 32767.0) as i16;
        out.extend_from_slice(&s.to_le_bytes());
    }
    out
}

/// 扫描 S16_LE 单声道样本求绝对峰值；对 `NORM_MIN_PEAK <= peak < NORM_TARGET_PEAK`
/// 的有效语音施加上限 `4x` 的线性增益，静音/底噪与已响亮信号保持原样不放大。
pub fn analyze_and_normalize_pcm16(raw_pcm: &[u8]) -> (Vec<u8>, u16) {
    let pcm_len = raw_pcm.len() & !1;
    let slice = &raw_pcm[..pcm_len];
    let mut peak: u16 = 0;
    for chunk in slice.chunks_exact(2) {
        let abs = i16::from_le_bytes([chunk[0], chunk[1]]).unsigned_abs();
        if abs > peak {
            peak = abs;
        }
    }
    if peak < NORM_MIN_PEAK || f64::from(peak) >= NORM_TARGET_PEAK {
        return (slice.to_vec(), peak);
    }
    let gain = (NORM_TARGET_PEAK / f64::from(peak)).min(NORM_MAX_GAIN);
    let mut out = Vec::with_capacity(pcm_len);
    for chunk in slice.chunks_exact(2) {
        let s = i16::from_le_bytes([chunk[0], chunk[1]]);
        let scaled = (f64::from(s) * gain).round().clamp(-32768.0, 32767.0) as i16;
        out.extend_from_slice(&scaled.to_le_bytes());
    }
    (out, peak)
}

/// 录音前自动设定板载 ES8388 采集路由与增益：
/// 必须同时将 `Left PGA Mux` 与 `Right PGA Mux` 设为差分（`ADCCONTROL2 = 0xF8`）
/// 才能激活 `Line 2` 差分麦放，避免单端悬空只录进底噪。
fn configure_es8388_capture(card: u32) {
    let card_s = card.to_string();
    let steps: &[&[&str]] = &[
        &["sset", "Capture Mute", "off"],
        &["sset", "Capture Digital", "192"],
        &["sset", "Main Mic", "on"],
        &["sset", "Headset Mic", "off"],
        &["sset", "ALC Capture NG", "off"],
        &["sset", "ALC Capture Function", "Off"],
        &["sset", "ADC Data Select", "Left Left"],
        &["sset", "Differential Mux", "Line 2"],
        &["sset", "Left PGA Mux", "DifferentialL"],
        &["sset", "Right PGA Mux", "DifferentialR"],
        &["sset", "Left Channel", "8"],
        &["sset", "Right Channel", "8"],
    ];
    for args in steps {
        match Command::new("amixer")
            .args(["-q", "-c", &card_s])
            .args(*args)
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .output()
        {
            Ok(out) if !out.status.success() => {
                let err = String::from_utf8_lossy(&out.stderr).trim().to_string();
                eprintln!("mic: amixer -c {card_s} {} 失败：{err}", args.join(" "));
            }
            Err(e) => eprintln!("mic: 无法执行 amixer：{e}"),
            Ok(_) => {}
        }
    }
}

/// 录音前开启外置 USB 矩阵麦克风（如 Timesintelli `cafe:4110` 的 `'Mic',0` 与 `'Mic',1`）采集增益并解除静音。
fn configure_usb_mic_capture(card: u32) {
    let card_s = card.to_string();
    for ctl in ["Mic,0", "Mic,1", "Mic"] {
        let _ = Command::new("amixer")
            .args(["-q", "-c", &card_s, "sset", ctl, "85%", "unmute", "cap"])
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .output();
    }
}

/// 将原始 S16_LE 单声道 PCM 字节封装为完整 RIFF/WAVE（奇数尾字节截断）。
pub fn build_wav_pcm16_mono(raw_pcm: &[u8], sample_rate: u32) -> Vec<u8> {
    let data_len = raw_pcm.len() & !1;
    let data_u32 = data_len as u32;
    let byte_rate = sample_rate * (CHANNELS as u32) * (BITS_PER_SAMPLE as u32 / 8);
    let block_align = CHANNELS * (BITS_PER_SAMPLE / 8);

    let mut wav = Vec::with_capacity(44 + data_len);
    wav.extend_from_slice(b"RIFF");
    wav.extend_from_slice(&(36u32.saturating_add(data_u32)).to_le_bytes());
    wav.extend_from_slice(b"WAVEfmt ");
    wav.extend_from_slice(&16u32.to_le_bytes());
    wav.extend_from_slice(&1u16.to_le_bytes()); // PCM
    wav.extend_from_slice(&CHANNELS.to_le_bytes());
    wav.extend_from_slice(&sample_rate.to_le_bytes());
    wav.extend_from_slice(&byte_rate.to_le_bytes());
    wav.extend_from_slice(&block_align.to_le_bytes());
    wav.extend_from_slice(&BITS_PER_SAMPLE.to_le_bytes());
    wav.extend_from_slice(b"data");
    wav.extend_from_slice(&data_u32.to_le_bytes());
    wav.extend_from_slice(&raw_pcm[..data_len]);
    wav
}

/// 返回麦克风实时状态快照（微秒级内存锁读，无 I/O）。
pub fn status_snapshot() -> Value {
    match slot().lock() {
        Ok(st) => st.snapshot(),
        Err(_) => json!({
            "phase": "idle",
            "recording": false,
            "playing": false,
            "has_recording": false,
            "duration_s": null,
            "elapsed_s": 0.0,
            "device": null,
        }),
    }
}

/// 开始录音成功后的回传摘要。
pub struct StartInfo {
    pub generation: u64,
    pub label: &'static str,
    pub snapshot: Value,
}

/// 启动 `arecord` 录制原始 PCM；阻塞 ~120 ms 做启动探活，放 `spawn_blocking` 中运行。
pub fn start_recording() -> Result<StartInfo, String> {
    let my_gen = {
        let mut st = slot().lock().map_err(|_| "麦克风状态锁异常".to_string())?;
        st.try_begin_start()?
    };
    let mut guard = PhaseResetGuard::for_starting();

    let target = crate::audio::capture_device()
        .ok_or_else(|| "未检测到可用麦克风设备".to_string())?;
    if target.label == "板载麦" {
        configure_es8388_capture(target.card);
    } else {
        configure_usb_mic_capture(target.card);
    }
    let raw_path =
        std::env::temp_dir().join(format!("wbnext-mic-raw-{}.pcm", std::process::id()));
    let _ = std::fs::remove_file(&raw_path);

    let mut child = Command::new("arecord")
        .args([
            "-q",
            "-D",
            target.alsa_dev.as_str(),
            "-t",
            "raw",
            "-f",
            "S16_LE",
            "-r",
            &SAMPLE_RATE.to_string(),
            "-c",
            &CHANNELS.to_string(),
            "-d",
            &MAX_RECORD_SECS.to_string(),
            raw_path.to_str().unwrap_or("/tmp/wbnext-mic-raw.pcm"),
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| format!("无法启动 arecord：{e}"))?;

    // 短等确认设备未被占用或参数拒绝
    std::thread::sleep(Duration::from_millis(STARTUP_CHECK_MS));
    match child.try_wait() {
        Ok(Some(status)) => {
            let mut err_buf = String::new();
            if let Some(mut se) = child.stderr.take() {
                let _ = se.read_to_string(&mut err_buf);
            }
            let _ = std::fs::remove_file(&raw_path);
            let msg = err_buf.trim();
            return Err(if msg.is_empty() {
                format!("录音启动后立即退出（退出码 {:?}）", status.code())
            } else {
                format!("录音设备打开失败：{msg}")
            });
        }
        Ok(None) => {}
        Err(e) => {
            let _ = child.kill();
            let _ = child.wait();
            let _ = std::fs::remove_file(&raw_path);
            return Err(format!("检查录音进程失败：{e}"));
        }
    }

    let snapshot = {
        let mut st = slot().lock().map_err(|_| "麦克风状态锁异常".to_string())?;
        st.phase = MicPhase::Recording(ActiveRec {
            generation: my_gen,
            child,
            raw_path,
            device: target.alsa_dev,
            label: target.label,
            started_at: Instant::now(),
        });
        guard.disarm();
        st.snapshot()
    };

    Ok(StartInfo { generation: my_gen, label: target.label, snapshot })
}

/// 结束录音成功后的回传摘要。
pub struct StopInfo {
    pub duration_s: f64,
    pub peak: u16,
    pub label: &'static str,
    pub snapshot: Value,
}

/// 手动结束当前录音并保存为标准 WAV；阻塞操作，放 `spawn_blocking` 中运行。
pub fn stop_recording() -> Result<StopInfo, String> {
    stop_inner(None)?
        .ok_or_else(|| "当前未在录音".to_string())
}

/// 仅当仍在运行对应代次的录音时自动结束；若已被用户手动停止则返回 `Ok(None)`。
pub fn stop_if_generation(expected_gen: u64) -> Result<Option<StopInfo>, String> {
    stop_inner(Some(expected_gen))
}

fn stop_inner(expected_gen: Option<u64>) -> Result<Option<StopInfo>, String> {
    let mut rec = {
        let mut st = slot().lock().map_err(|_| "麦克风状态锁异常".to_string())?;
        match st.try_take_recording(expected_gen)? {
            Some(r) => r,
            None => return Ok(None),
        }
    };
    let mut guard = PhaseResetGuard::for_stopping();

    if let Ok(None) = rec.child.try_wait() {
        let _ = rec.child.kill();
    }
    let stderr = rec
        .child
        .wait_with_output()
        .ok()
        .map(|o| String::from_utf8_lossy(&o.stderr).trim().to_string())
        .unwrap_or_default();

    let raw = std::fs::read(&rec.raw_path).unwrap_or_default();
    let _ = std::fs::remove_file(&rec.raw_path);

    let pcm_len = raw.len() & !1;
    if pcm_len == 0 {
        return Err(if stderr.is_empty() {
            "未采集到音频数据，请检查麦克风".into()
        } else {
            format!("未采集到音频数据：{stderr}")
        });
    }

    let filtered = highpass_filter_pcm16(&raw[..pcm_len]);
    let (norm_pcm, peak) = analyze_and_normalize_pcm16(&filtered);
    let wav_bytes = build_wav_pcm16_mono(&norm_pcm, SAMPLE_RATE);
    let wav_path =
        std::env::temp_dir().join(format!("wbnext-mic-rec-{}.wav", std::process::id()));
    let tmp_path = wav_path.with_extension("wav.tmp");
    std::fs::write(&tmp_path, &wav_bytes).map_err(|e| format!("写入录音文件失败：{e}"))?;
    std::fs::rename(&tmp_path, &wav_path).map_err(|e| {
        let _ = std::fs::remove_file(&tmp_path);
        format!("保存录音文件失败：{e}")
    })?;

    let duration_s = round_tenths(pcm_len as f64 / BYTES_PER_SEC as f64).max(0.1);
    let label = rec.label;

    let snapshot = {
        let mut st = slot().lock().map_err(|_| "麦克风状态锁异常".to_string())?;
        st.last = Some(SavedRec { wav_path, duration_s, device: rec.device });
        st.phase = MicPhase::Idle;
        guard.disarm();
        st.snapshot()
    };

    Ok(Some(StopInfo { duration_s, peak, label, snapshot }))
}

/// 已抢占播放槽的作业句柄；若未调用 `finish_play` 即丢弃，自动还原 `Idle`。
pub struct PlayJob {
    wav_path: PathBuf,
    duration_s: f64,
    device: String,
    finished: bool,
}

impl Drop for PlayJob {
    fn drop(&mut self) {
        if !self.finished {
            if let Ok(mut st) = slot().lock() {
                if matches!(st.phase, MicPhase::Playing) {
                    st.phase = MicPhase::Idle;
                }
            }
        }
    }
}

/// 非阻塞抢占播放槽，返回作业句柄与即时状态快照（`playing: true`）。
pub fn begin_play() -> Result<(PlayJob, Value), String> {
    let mut st = slot().lock().map_err(|_| "麦克风状态锁异常".to_string())?;
    let (wav_path, duration_s) = st.try_begin_play()?;
    let device = format!("plughw:{},0", crate::audio::es8388_card().unwrap_or(2));
    let snapshot = st.snapshot();
    Ok((PlayJob { wav_path, duration_s, device, finished: false }, snapshot))
}

/// 在车体喇叭上播放已保存录音，阻塞至结束或超时；放 `spawn_blocking` 中运行。
pub fn finish_play(mut job: PlayJob) -> Result<(f64, String, Value), String> {
    let timeout = Duration::from_secs((job.duration_s.ceil() as u64 + 8).max(12));
    let play_res = run_aplay(&job.device, &job.wav_path, timeout);

    let snapshot = {
        let mut st = slot().lock().map_err(|_| "麦克风状态锁异常".to_string())?;
        if matches!(st.phase, MicPhase::Playing) {
            st.phase = MicPhase::Idle;
        }
        job.finished = true;
        st.snapshot()
    };

    play_res.map(|_| (job.duration_s, job.device.clone(), snapshot))
}

fn run_aplay(device: &str, wav_path: &std::path::Path, timeout: Duration) -> Result<(), String> {
    let mut child = Command::new("aplay")
        .args(["-q", "-D", device])
        .arg(wav_path)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| format!("无法启动 aplay：{e}"))?;

    let deadline = Instant::now() + timeout;
    loop {
        match child.try_wait() {
            Ok(Some(st)) => {
                let out = child.wait_with_output().map_err(|e| e.to_string())?;
                let err = String::from_utf8_lossy(&out.stderr).trim().to_string();
                return if st.success() {
                    Ok(())
                } else {
                    Err(if err.is_empty() {
                        format!("aplay 退出码 {:?}", st.code())
                    } else {
                        err
                    })
                };
            }
            Ok(None) if Instant::now() < deadline => {
                std::thread::sleep(Duration::from_millis(20));
            }
            Ok(None) => {
                let _ = child.kill();
                let _ = child.wait();
                return Err(format!("aplay 超时（{}s）", timeout.as_secs()));
            }
            Err(e) => return Err(format!("aplay 运行失败：{e}")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wav_header_valid_and_truncates_odd_byte() {
        let raw = [1u8, 2, 3, 4, 5]; // 奇数字节 -> 截断成 4 字节 PCM
        let wav = build_wav_pcm16_mono(&raw, 16_000);
        assert_eq!(wav.len(), 44 + 4);
        assert_eq!(&wav[0..4], b"RIFF");
        assert_eq!(u32::from_le_bytes(wav[4..8].try_into().unwrap()), 36 + 4);
        assert_eq!(&wav[8..16], b"WAVEfmt ");
        assert_eq!(u16::from_le_bytes(wav[20..22].try_into().unwrap()), 1); // PCM
        assert_eq!(u16::from_le_bytes(wav[22..24].try_into().unwrap()), 1); // mono
        assert_eq!(u32::from_le_bytes(wav[24..28].try_into().unwrap()), 16_000);
        assert_eq!(u32::from_le_bytes(wav[28..32].try_into().unwrap()), 32_000);
        assert_eq!(u16::from_le_bytes(wav[32..34].try_into().unwrap()), 2);
        assert_eq!(u16::from_le_bytes(wav[34..36].try_into().unwrap()), 16);
        assert_eq!(&wav[36..40], b"data");
        assert_eq!(u32::from_le_bytes(wav[40..44].try_into().unwrap()), 4);
        assert_eq!(&wav[44..48], &[1, 2, 3, 4]);
    }

    #[test]
    fn mic_state_transitions_and_guards() {
        let mut st = MicState::new();
        assert_eq!(st.try_begin_play().unwrap_err(), "尚未录制声音，请先录音");
        assert_eq!(st.try_take_recording(None).unwrap_err(), "当前未在录音");
        // 带 expected_gen 时非录音态静默返回 Ok(None)
        assert!(st.try_take_recording(Some(1)).unwrap().is_none());

        assert_eq!(st.try_begin_start().unwrap(), 1);
        assert_eq!(st.try_begin_start().unwrap_err(), "正在录音中");
        assert_eq!(st.try_begin_play().unwrap_err(), "正在录音中，请先结束录音");
        assert_eq!(st.try_take_recording(None).unwrap_err(), "录音正在启动，请稍后再结束");

        // 模拟存在有效 WAV 文件后抢占播放槽
        let wav_path =
            std::env::temp_dir().join(format!("wbnext-mic-unit-{}.wav", std::process::id()));
        std::fs::write(&wav_path, b"RIFF").unwrap();
        st.phase = MicPhase::Idle;
        st.last = Some(SavedRec {
            wav_path: wav_path.clone(),
            duration_s: 1.5,
            device: "plughw:3,0".into(),
        });
        let (p, dur) = st.try_begin_play().unwrap();
        assert_eq!(p, wav_path);
        assert_eq!(dur, 1.5);
        assert_eq!(st.try_begin_start().unwrap_err(), "正在播放录音，请稍候");
        assert_eq!(st.try_begin_play().unwrap_err(), "正在播放录音");
        std::fs::remove_file(&wav_path).ok();
    }

    #[test]
    fn play_job_drop_restores_idle_phase() {
        let wav_path =
            std::env::temp_dir().join(format!("wbnext-mic-drop-{}.wav", std::process::id()));
        std::fs::write(&wav_path, b"RIFF").unwrap();
        {
            let mut st = slot().lock().unwrap();
            st.phase = MicPhase::Playing;
        }
        let job = PlayJob {
            wav_path: wav_path.clone(),
            duration_s: 1.0,
            device: "plughw:2,0".into(),
            finished: false,
        };
        drop(job);
        {
            let st = slot().lock().unwrap();
            assert!(matches!(st.phase, MicPhase::Idle));
        }
        std::fs::remove_file(&wav_path).ok();
    }

    fn encode_samples(samples: &[i16]) -> Vec<u8> {
        let mut out = Vec::with_capacity(samples.len() * 2);
        for s in samples {
            out.extend_from_slice(&s.to_le_bytes());
        }
        out
    }

    fn decode_samples(bytes: &[u8]) -> Vec<i16> {
        bytes.chunks_exact(2).map(|c| i16::from_le_bytes([c[0], c[1]])).collect()
    }

    #[test]
    fn analyze_and_normalize_pcm16_bounds() {
        // 静音/底噪（peak < 800）：原样保留，不放大底噪
        let silent = encode_samples(&[0, 120, -799]);
        let (out, peak) = analyze_and_normalize_pcm16(&silent);
        assert_eq!(peak, 799);
        assert_eq!(decode_samples(&out), vec![0, 120, -799]);

        // 轻声（peak = 1000）：上限 4x 放大 -> 4000
        let quiet = encode_samples(&[1000, -500]);
        let (out, peak) = analyze_and_normalize_pcm16(&quiet);
        assert_eq!(peak, 1000);
        assert_eq!(decode_samples(&out), vec![4000, -2000]);

        // 中等音量（peak = 9000）：放大到 18000（2x）
        let med = encode_samples(&[-9_000, 4000]);
        let (out, peak) = analyze_and_normalize_pcm16(&med);
        assert_eq!(peak, 9_000);
        assert_eq!(decode_samples(&out), vec![-18_000, 8000]);

        // 响亮信号（peak = 25000 >= 18000）：不放大且不削波
        let loud = encode_samples(&[25_000, -18_000]);
        let (out, peak) = analyze_and_normalize_pcm16(&loud);
        assert_eq!(peak, 25_000);
        assert_eq!(decode_samples(&out), vec![25_000, -18_000]);

        // i16::MIN 与奇数字节尾部边界无溢出/panic
        let mut edge = encode_samples(&[i16::MIN, 1234]);
        edge.push(0xAA);
        let (out, peak) = analyze_and_normalize_pcm16(&edge);
        assert_eq!(peak, 32_768);
        assert_eq!(decode_samples(&out), vec![i16::MIN, 1234]);
    }

    #[test]
    fn highpass_filter_removes_dc_and_preserves_1khz() {
        // 叠加恒定直流 5000 + 1 kHz 正弦（幅值 4000），滤波后稳态直流应被消除且 1 kHz 保留
        let sr = 16_000usize;
        let samples: Vec<i16> = (0..sr)
            .map(|i| {
                let s = 5000.0 + 4000.0 * (std::f64::consts::TAU * 1000.0 * i as f64 / sr as f64).sin();
                s as i16
            })
            .collect();
        let filtered = decode_samples(&highpass_filter_pcm16(&encode_samples(&samples)));
        let tail = &filtered[sr / 2..];
        let mean: f64 = tail.iter().map(|&x| f64::from(x)).sum::<f64>() / tail.len() as f64;
        let peak = tail.iter().map(|&x| x.unsigned_abs()).max().unwrap_or(0);
        assert!(mean.abs() < 50.0, "DC mean={mean}");
        assert!(peak > 2800 && peak < 4200, "1kHz peak={peak}");
    }
}
