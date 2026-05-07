// SPDX-License-Identifier: MulanPSL-2.0
// cmd/chat.rs — TUI chat client (connects to robonix-liaison via SrvLiaison).
//
// Modalities (one TUI, one stream type per turn):
//
//   ┌────────────────┐ Enter ─────────► SrvLiaison.Stream(Task)
//   │ rbnx chat TUI  │                 (text → PilotEvent stream)
//   └────────────────┘ Ctrl+V ────────► SrvLiaison.StartVoiceSession(req)
//                                       (voice → VoiceEvent stream which
//                                        wraps PilotEvent in `pilot` field)
//
// Liaison handles user-id assignment, voice mic/ASR/voiceprint/TTS/speaker
// orchestration, and forwarding to Pilot. The TUI just renders events.

use anyhow::{Context, Result};
use crossterm::{
    ExecutableCommand,
    event::{self, Event, KeyCode, KeyModifiers},
    terminal::{self, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    Terminal,
    backend::CrosstermBackend,
    layout::{Constraint, Layout},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph, Wrap},
};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use std::cell::RefCell;
use std::io;
use std::rc::Rc;
use tokio_stream::StreamExt;
use uuid::Uuid;

const CONSUMER_ID: &str = "rbnx-cli/chat";
const LIAISON_CONTRACT_ID: &str = "robonix/system/liaison";

/// Atlas contract ids for the audio primitives that have user-visible
/// device choices on a multi-provider host (e.g. local ALSA driver vs
/// the macOS bridge). asr/tts are software backends with one provider
/// per box, so we don't prompt for them.
const MIC_CONTRACT: &str = "robonix/primitive/audio/mic";
const SPEAKER_CONTRACT: &str = "robonix/primitive/audio/speaker";

struct ChatMessage {
    role: Role,
    text: String,
}

enum Role {
    User,
    Agent,
    ToolCall,
    Status,
    Voice,
}

const DEFAULT_LIAISON_FALLBACK: &str = "http://127.0.0.1:50081";

/// Persisted user choices for `rbnx chat`, written to ~/.robonix/chat.yaml.
///
/// Two layers per audio side:
///   *_cap_id      → which audio impl provider in atlas (e.g. .alsa vs .macos)
///   *_device_id   → which OS-level device that impl should drive (impl-
///                   specific id from ListAudioDevices; "" = OS default)
///
/// Hot-swap-able: Ctrl+A in chat re-runs the picker. Initial run reads
/// this file; missing fields trigger a picker prompt.
#[derive(Clone, Default, serde::Serialize, serde::Deserialize)]
struct ChatConfig {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    mic_cap_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    mic_device_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    speaker_cap_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    speaker_device_id: Option<String>,
}

fn chat_config_path() -> Option<std::path::PathBuf> {
    dirs::home_dir().map(|h| h.join(".robonix").join("chat.yaml"))
}

fn load_chat_config() -> ChatConfig {
    let Some(p) = chat_config_path() else {
        return ChatConfig::default();
    };
    let Ok(text) = std::fs::read_to_string(&p) else {
        return ChatConfig::default();
    };
    serde_yaml::from_str(&text).unwrap_or_default()
}

fn save_chat_config(cfg: &ChatConfig) -> Result<()> {
    let p = chat_config_path().context("no home dir")?;
    if let Some(parent) = p.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&p, serde_yaml::to_string(cfg)?)?;
    Ok(())
}

pub async fn execute(server: &str) -> Result<()> {
    let atlas_endpoint = if server.starts_with("http") {
        server.to_string()
    } else {
        format!("http://{server}")
    };

    let liaison_endpoint = if let Ok(ep) = std::env::var("ROBONIX_LIAISON_ENDPOINT") {
        log::info!("using ROBONIX_LIAISON_ENDPOINT={ep}");
        if ep.starts_with("http") {
            ep
        } else {
            format!("http://{ep}")
        }
    } else {
        discover_liaison(&atlas_endpoint).await.unwrap_or_else(|e| {
            log::warn!(
                "liaison discovery timed out ({e:#}), falling back to {DEFAULT_LIAISON_FALLBACK}"
            );
            DEFAULT_LIAISON_FALLBACK.to_string()
        })
    };
    let liaison_endpoint = localhost_to_ipv4_loopback(&liaison_endpoint);

    let mut stdout = io::stdout();
    stdout.execute(EnterAlternateScreen)?;
    terminal::enable_raw_mode()?;

    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    // ensure_audio_devices is best-effort: if atlas is unreachable, no
    // audio providers are registered, or the user skips with Esc, we
    // surface the reason as a warning in the chat history and continue
    // — text chat works without any audio path. Hard errors here would
    // mean a single-user typo at audio-pick time kills the whole TUI.
    let (chat_cfg, audio_warnings) =
        match ensure_audio_devices(&atlas_endpoint, &mut terminal).await {
            Ok(v) => v,
            Err(e) => {
                terminal::disable_raw_mode()?;
                terminal.backend_mut().execute(LeaveAlternateScreen)?;
                return Err(e);
            }
        };

    let result = run_tui(
        &mut terminal,
        &atlas_endpoint,
        &liaison_endpoint,
        chat_cfg,
        &audio_warnings,
    )
    .await;

    terminal::disable_raw_mode()?;
    terminal.backend_mut().execute(LeaveAlternateScreen)?;

    result
}

/// Try to discover Liaison via Atlas, retrying for up to `timeout_secs`.
async fn discover_liaison(atlas_endpoint: &str) -> Result<String> {
    const RETRY_INTERVAL_MS: u64 = 2_000;
    const TIMEOUT_SECS: u64 = 60;

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(TIMEOUT_SECS);
    let mut attempt = 0u32;

    loop {
        attempt += 1;
        match try_discover_once(atlas_endpoint, LIAISON_CONTRACT_ID).await {
            Ok(ep) => return Ok(ep),
            Err(e) => {
                if std::time::Instant::now() >= deadline {
                    anyhow::bail!("liaison not found in Atlas after {TIMEOUT_SECS}s: {e:#}");
                }
                if attempt == 1 {
                    eprintln!("[chat] waiting for Liaison to register in Atlas ({e:#})…");
                }
                tokio::time::sleep(std::time::Duration::from_millis(RETRY_INTERVAL_MS)).await;
            }
        }
    }
}

async fn try_discover_once(atlas_endpoint: &str, contract_id: &str) -> Result<String> {
    let mut atlas = AtlasClient::connect(atlas_endpoint).await?;
    let transport = atlas_pb::Transport::Grpc;
    let records = atlas.query_capabilities("", contract_id, transport).await?;
    for rec in &records {
        let has = rec
            .interfaces
            .iter()
            .any(|i| i.contract_id == contract_id && i.transport == transport as i32);
        if !has {
            continue;
        }
        let (_, endpoint, _) = atlas
            .connect_capability(CONSUMER_ID, &rec.capability_id, contract_id, transport)
            .await?;
        if endpoint.is_empty() {
            continue;
        }
        let uri = if endpoint.starts_with("http") {
            endpoint
        } else {
            format!("http://{endpoint}")
        };
        return Ok(localhost_to_ipv4_loopback(&uri));
    }
    anyhow::bail!("no {contract_id} interface found in Atlas registry")
}

// ── Audio-device first-run picker ───────────────────────────────────────────
//
// Resolution order for mic/speaker capability_ids when starting a voice
// session is: ROBONIX_CHAT_*_NODE env (highest, overrides everything) →
// chat.yaml on disk → first-run TUI picker. The picker only fires when
// neither env nor config supplies the cap_id; once chosen it's saved
// to ~/.robonix/chat.yaml so future sessions don't ask again. To
// re-pick: delete that file (or just edit it), re-run rbnx chat.
//
// Why this lives here and not in liaison: the user choice is per-client
// preference (which physical box's audio do I want when sitting at this
// terminal), not a per-deployment server setting.

/// Pick mode for the legacy modal picker chain (FirstRun only — the
/// Ctrl+A path now uses the dashboard via `run_audio_settings_page`).
/// `Reconfigure` is kept so the older code paths that still take a
/// PickMode parameter compile cleanly even though nothing constructs it.
#[derive(Clone, Copy)]
#[allow(dead_code)]
enum PickMode {
    FirstRun,
    Reconfigure,
}

/// Best-effort audio device discovery. Anything that goes wrong here
/// (atlas unreachable, no providers registered, user pressed Esc) is
/// downgraded to a chat-history warning — text mode keeps working.
async fn ensure_audio_devices(
    atlas_endpoint: &str,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
) -> Result<(ChatConfig, Vec<String>)> {
    pick_audio_settings(atlas_endpoint, terminal, PickMode::FirstRun).await
}

async fn pick_audio_settings(
    atlas_endpoint: &str,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    mode: PickMode,
) -> Result<(ChatConfig, Vec<String>)> {
    let mut cfg = load_chat_config();
    let mut warnings: Vec<String> = Vec::new();

    let env_set = |key: &str| std::env::var(key).ok().filter(|s| !s.is_empty()).is_some();
    let always = matches!(mode, PickMode::Reconfigure);
    let need_mic = always || (!env_set("ROBONIX_CHAT_MIC_NODE") && cfg.mic_cap_id.is_none());
    let need_speaker =
        always || (!env_set("ROBONIX_CHAT_SPEAKER_NODE") && cfg.speaker_cap_id.is_none());
    if !need_mic && !need_speaker {
        return Ok((cfg, warnings));
    }

    let mut atlas = match AtlasClient::connect(atlas_endpoint).await {
        Ok(c) => c,
        Err(e) => {
            warnings.push(format!(
                "audio device pick skipped — atlas unreachable at {atlas_endpoint}: {e:#}. \
                 Text mode still works; voice (Ctrl+V) will fail until atlas is up."
            ));
            return Ok((cfg, warnings));
        }
    };

    if need_mic {
        let saved_cap = cfg.mic_cap_id.clone();
        let saved_dev = cfg.mic_device_id.clone();
        match try_pick(
            &mut atlas,
            terminal,
            "mic",
            MIC_CONTRACT,
            "input",
            saved_cap.as_deref(),
            saved_dev.as_deref(),
            mode,
        )
        .await
        {
            Ok(Some((cap_id, device_id))) => {
                cfg.mic_cap_id = Some(cap_id);
                cfg.mic_device_id = Some(device_id);
            }
            Ok(None) => warnings.push(
                "no mic provider in atlas — voice input disabled (text mode unaffected)".into(),
            ),
            Err(e) => warnings.push(format!("mic pick: {e:#}")),
        }
    }
    if need_speaker {
        let saved_cap = cfg.speaker_cap_id.clone();
        let saved_dev = cfg.speaker_device_id.clone();
        match try_pick(
            &mut atlas,
            terminal,
            "speaker",
            SPEAKER_CONTRACT,
            "output",
            saved_cap.as_deref(),
            saved_dev.as_deref(),
            mode,
        )
        .await
        {
            Ok(Some((cap_id, device_id))) => {
                cfg.speaker_cap_id = Some(cap_id);
                cfg.speaker_device_id = Some(device_id);
            }
            Ok(None) => warnings.push(
                "no speaker provider in atlas — voice playback disabled (text mode unaffected)"
                    .into(),
            ),
            Err(e) => warnings.push(format!("speaker pick: {e:#}")),
        }
    }
    if let Err(e) = save_chat_config(&cfg) {
        log::warn!("could not save chat config: {e:#}");
    }
    Ok((cfg, warnings))
}

/// `Ok(Some((cap_id, device_id)))` = picked both layers; device_id may be ""
/// when the impl returned UNIMPLEMENTED on list_devices.
/// `Ok(None)` = no providers in atlas.
#[allow(clippy::too_many_arguments)]
async fn try_pick(
    atlas: &mut AtlasClient,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    label: &str,
    contract: &str,
    kind: &str,
    saved_cap_id: Option<&str>,
    saved_device_id: Option<&str>,
    mode: PickMode,
) -> Result<Option<(String, String)>> {
    let providers = atlas
        .query_capabilities("", contract, atlas_pb::Transport::Grpc)
        .await?;
    if providers.is_empty() {
        return Ok(None);
    }

    // Layer A — provider (cap_id). FirstRun honours the saved choice
    // when it's still in atlas, or auto-picks the lone provider with a
    // 400 ms flash. Reconfigure (Ctrl+A) ALWAYS shows the TUI even
    // for a single entry — auto-pick on Ctrl+A looks identical to
    // "Ctrl+A did nothing", which is what the user just hit.
    let cap_id = match (mode, saved_cap_id) {
        (PickMode::FirstRun, Some(s)) if providers.iter().any(|p| p.capability_id == s) => {
            s.to_string()
        }
        (PickMode::FirstRun, _) if providers.len() == 1 => {
            let id = providers[0].capability_id.clone();
            flash_picker_message(terminal, &format!("auto-selected {label}: {id}"))?;
            id
        }
        _ => match pick_tui(terminal, label, contract, &providers)? {
            Some(s) => s,
            None => return Ok(None),
        },
    };

    // Layer B — device id within the chosen impl. Connect to its
    // list_devices contract (UNIMPLEMENTED is OK — fall through with "").
    let device_id =
        pick_device_for_cap(atlas, terminal, &cap_id, label, kind, saved_device_id, mode).await?;

    // Tell the impl which device to use. Best-effort; ignore failures.
    if !device_id.is_empty()
        && let Err(e) = call_select_device(atlas, &cap_id, kind, &device_id).await
    {
        log::warn!("SelectAudioDevice on {cap_id} ({kind}={device_id}) failed: {e:#}");
    }

    Ok(Some((cap_id, device_id)))
}

/// Connect to `cap_id`'s list_devices interface, ask for the device
/// list, run a picker on the entries that match `kind` (input/output)
/// + duplex. Returns "" when the impl doesn't expose the contract.
async fn pick_device_for_cap(
    atlas: &mut AtlasClient,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    cap_id: &str,
    label: &str,
    kind: &str,
    saved_device_id: Option<&str>,
    mode: PickMode,
) -> Result<String> {
    use crate::pb::contracts::primitive_audio_list_devices_client::PrimitiveAudioListDevicesClient;

    const LIST_CONTRACT: &str = "robonix/primitive/audio/list_devices";
    // Reconfigure mode shows visible feedback for every silent path
    // below; FirstRun stays quiet so a missing list_devices contract
    // doesn't litter chat history with a non-actionable message.
    let reconf = matches!(mode, PickMode::Reconfigure);

    let endpoint = match atlas
        .connect_capability(
            CONSUMER_ID,
            cap_id,
            LIST_CONTRACT,
            atlas_pb::Transport::Grpc,
        )
        .await
    {
        Ok((_, ep, _)) => {
            if ep.starts_with("http") {
                ep
            } else {
                format!("http://{ep}")
            }
        }
        Err(_) => {
            if reconf {
                flash_picker_message(
                    terminal,
                    &format!(
                        "{cap_id} doesn't expose list_devices — \
                         using OS default device. Rebuild the package \
                         (bash scripts/build.sh) to pick up the new contract."
                    ),
                )?;
            }
            return Ok(String::new());
        }
    };

    let mut client = match PrimitiveAudioListDevicesClient::connect(endpoint.clone()).await {
        Ok(c) => c,
        Err(e) => {
            if reconf {
                flash_picker_message(terminal, &format!("connect list_devices on {cap_id}: {e}"))?;
            }
            return Ok(String::new());
        }
    };
    let resp = match client
        .list_audio_devices(crate::pb::audio::ListAudioDevicesRequest {})
        .await
    {
        Ok(r) => r.into_inner(),
        Err(e) => {
            if reconf {
                flash_picker_message(terminal, &format!("ListAudioDevices on {cap_id}: {e}"))?;
            }
            return Ok(String::new());
        }
    };

    let usable: Vec<crate::pb::audio::AudioDevice> = resp
        .devices
        .into_iter()
        .filter(|d| d.kind == kind || d.kind == "duplex")
        .collect();
    if usable.is_empty() {
        if reconf {
            flash_picker_message(
                terminal,
                &format!("{cap_id} reports no {kind} devices — using OS default"),
            )?;
        }
        return Ok(String::new());
    }

    // FirstRun: honour saved device when still listed; auto-pick a lone
    // device with a flash. Reconfigure: always show the TUI so Ctrl+A
    // gives the user a real page they can interact with.
    if matches!(mode, PickMode::FirstRun) {
        if let Some(saved) = saved_device_id
            && usable.iter().any(|d| d.id == saved)
        {
            return Ok(saved.to_string());
        }
        if usable.len() == 1 {
            let id = usable[0].id.clone();
            flash_picker_message(
                terminal,
                &format!("auto-selected {label} device: {}", usable[0].name),
            )?;
            return Ok(id);
        }
    }

    let chosen = pick_device_tui(terminal, label, &usable)?;
    // pick_device_tui returns "" on Esc/q — preserve any previously-saved
    // device id so a Reconfigure-then-skip doesn't accidentally clear it.
    if chosen.is_empty()
        && let Some(saved) = saved_device_id
    {
        return Ok(saved.to_string());
    }
    Ok(chosen)
}

async fn call_select_device(
    atlas: &mut AtlasClient,
    cap_id: &str,
    kind: &str,
    device_id: &str,
) -> Result<()> {
    use crate::pb::contracts::primitive_audio_select_device_client::PrimitiveAudioSelectDeviceClient;
    const SELECT_CONTRACT: &str = "robonix/primitive/audio/select_device";

    let (_, ep, _) = atlas
        .connect_capability(
            CONSUMER_ID,
            cap_id,
            SELECT_CONTRACT,
            atlas_pb::Transport::Grpc,
        )
        .await?;
    let endpoint = if ep.starts_with("http") {
        ep
    } else {
        format!("http://{ep}")
    };
    let mut client = PrimitiveAudioSelectDeviceClient::connect(endpoint).await?;
    let resp = client
        .select_audio_device(crate::pb::audio::SelectAudioDeviceRequest {
            kind: kind.to_string(),
            id: device_id.to_string(),
        })
        .await?
        .into_inner();
    if !resp.ok {
        anyhow::bail!("impl rejected: {}", resp.error);
    }
    Ok(())
}

fn pick_device_tui(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    label: &str,
    devices: &[crate::pb::audio::AudioDevice],
) -> Result<String> {
    let mut idx = 0usize;
    loop {
        terminal.draw(|f| {
            let area = f.area();
            let lines: Vec<Line> = devices
                .iter()
                .enumerate()
                .map(|(i, d)| {
                    let mark = if i == idx { "▶ " } else { "  " };
                    let style = if i == idx {
                        Style::default()
                            .fg(Color::Black)
                            .bg(Color::Cyan)
                            .add_modifier(Modifier::BOLD)
                    } else {
                        Style::default()
                    };
                    let mut tags = Vec::new();
                    if d.is_default {
                        tags.push("default".to_string());
                    }
                    if !d.note.is_empty() {
                        tags.push(format!("⚠ {}", d.note));
                    }
                    let suffix = if tags.is_empty() {
                        String::new()
                    } else {
                        format!("  ({})", tags.join(", "))
                    };
                    Line::from(vec![Span::styled(
                        format!("{mark}#{} {}{}", d.id, d.name, suffix),
                        style,
                    )])
                })
                .collect();
            let body = Paragraph::new(lines)
                .block(Block::default().borders(Borders::ALL).title(format!(
                    " choose {label} device — ↑↓ select · Enter confirm · Esc skip "
                )))
                .wrap(Wrap { trim: false });
            f.render_widget(body, area);
        })?;
        if event::poll(std::time::Duration::from_millis(200))?
            && let Event::Key(key) = event::read()?
        {
            match key.code {
                KeyCode::Up | KeyCode::Char('k') => idx = idx.saturating_sub(1),
                KeyCode::Down | KeyCode::Char('j') if idx + 1 < devices.len() => idx += 1,
                KeyCode::Enter => return Ok(devices[idx].id.clone()),
                KeyCode::Char('q') | KeyCode::Esc => return Ok(String::new()),
                _ => {}
            }
        }
    }
}

fn pick_tui(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    label: &str,
    contract: &str,
    providers: &[atlas_pb::CapabilityRecord],
) -> Result<Option<String>> {
    let mut idx = 0usize;
    loop {
        terminal.draw(|f| {
            let area = f.area();
            let lines: Vec<Line> = providers
                .iter()
                .enumerate()
                .map(|(i, r)| {
                    let mark = if i == idx { "▶ " } else { "  " };
                    let style = if i == idx {
                        Style::default()
                            .fg(Color::Black)
                            .bg(Color::Cyan)
                            .add_modifier(Modifier::BOLD)
                    } else {
                        Style::default()
                    };
                    let detail = if r.state_detail.is_empty() {
                        String::new()
                    } else {
                        format!("  ({})", r.state_detail)
                    };
                    Line::from(vec![Span::styled(
                        format!("{mark}{}{detail}", r.capability_id),
                        style,
                    )])
                })
                .collect();
            let body = Paragraph::new(lines)
                .block(Block::default().borders(Borders::ALL).title(format!(
                    " choose {label} provider ({contract}) — ↑↓ select · Enter confirm · Esc skip "
                )))
                .wrap(Wrap { trim: false });
            f.render_widget(body, area);
        })?;
        if event::poll(std::time::Duration::from_millis(200))?
            && let Event::Key(key) = event::read()?
        {
            match key.code {
                KeyCode::Up | KeyCode::Char('k') => idx = idx.saturating_sub(1),
                KeyCode::Down | KeyCode::Char('j') if idx + 1 < providers.len() => {
                    idx += 1;
                }
                KeyCode::Enter => return Ok(Some(providers[idx].capability_id.clone())),
                KeyCode::Char('q') | KeyCode::Esc => return Ok(None),
                _ => {}
            }
        }
    }
}

// ── Audio settings dashboard (btop-style, single page) ─────────────────────
//
// Replacement for the old sequential modal pickers. One screen, four
// sections (mic provider / mic device / speaker provider / speaker
// device), Tab cycles section, ↑↓ moves cursor within section, Enter
// picks the highlighted item (and reloads device list when picking a
// new provider, or calls SelectAudioDevice when picking a device),
// Esc / Ctrl+A / q saves and closes.

#[derive(Clone, Copy, PartialEq, Eq)]
enum AudioSection {
    MicProvider,
    MicDevice,
    SpeakerProvider,
    SpeakerDevice,
}

struct AudioSettingsPage {
    mic_providers: Vec<atlas_pb::CapabilityRecord>,
    speaker_providers: Vec<atlas_pb::CapabilityRecord>,
    mic_devices: Vec<crate::pb::audio::AudioDevice>,
    speaker_devices: Vec<crate::pb::audio::AudioDevice>,

    mic_cap_id: String,
    mic_device_id: String,
    speaker_cap_id: String,
    speaker_device_id: String,

    section: AudioSection,
    cursor_mp: usize,
    cursor_md: usize,
    cursor_sp: usize,
    cursor_sd: usize,

    status: String,
}

async fn fetch_devices_filtered(
    atlas: &mut AtlasClient,
    cap_id: &str,
    kind: &str,
) -> Vec<crate::pb::audio::AudioDevice> {
    use crate::pb::contracts::primitive_audio_list_devices_client::PrimitiveAudioListDevicesClient;
    const LIST_CONTRACT: &str = "robonix/primitive/audio/list_devices";
    if cap_id.is_empty() {
        return Vec::new();
    }
    let endpoint = match atlas
        .connect_capability(
            CONSUMER_ID,
            cap_id,
            LIST_CONTRACT,
            atlas_pb::Transport::Grpc,
        )
        .await
    {
        Ok((_, ep, _)) if ep.starts_with("http") => ep,
        Ok((_, ep, _)) => format!("http://{ep}"),
        Err(_) => return Vec::new(),
    };
    let mut client = match PrimitiveAudioListDevicesClient::connect(endpoint).await {
        Ok(c) => c,
        Err(_) => return Vec::new(),
    };
    let resp = match client
        .list_audio_devices(crate::pb::audio::ListAudioDevicesRequest {})
        .await
    {
        Ok(r) => r.into_inner(),
        Err(_) => return Vec::new(),
    };
    resp.devices
        .into_iter()
        .filter(|d| d.kind == kind || d.kind == "duplex")
        .collect()
}

impl AudioSettingsPage {
    async fn load(atlas: &mut AtlasClient, cfg: &ChatConfig) -> Self {
        let mic_providers = atlas
            .query_capabilities("", MIC_CONTRACT, atlas_pb::Transport::Grpc)
            .await
            .unwrap_or_default();
        let speaker_providers = atlas
            .query_capabilities("", SPEAKER_CONTRACT, atlas_pb::Transport::Grpc)
            .await
            .unwrap_or_default();

        let mic_cap_id = cfg.mic_cap_id.clone().unwrap_or_else(|| {
            mic_providers
                .first()
                .map(|p| p.capability_id.clone())
                .unwrap_or_default()
        });
        let speaker_cap_id = cfg.speaker_cap_id.clone().unwrap_or_else(|| {
            speaker_providers
                .first()
                .map(|p| p.capability_id.clone())
                .unwrap_or_default()
        });

        let mic_devices = fetch_devices_filtered(atlas, &mic_cap_id, "input").await;
        let speaker_devices = fetch_devices_filtered(atlas, &speaker_cap_id, "output").await;

        let mic_device_id = cfg.mic_device_id.clone().unwrap_or_default();
        let speaker_device_id = cfg.speaker_device_id.clone().unwrap_or_default();

        // Position cursors at the currently-selected entry so the user
        // sees what's active rather than always landing at row 0.
        let cursor_mp = mic_providers
            .iter()
            .position(|p| p.capability_id == mic_cap_id)
            .unwrap_or(0);
        let cursor_sp = speaker_providers
            .iter()
            .position(|p| p.capability_id == speaker_cap_id)
            .unwrap_or(0);
        let cursor_md = mic_devices
            .iter()
            .position(|d| d.id == mic_device_id)
            .unwrap_or(0);
        let cursor_sd = speaker_devices
            .iter()
            .position(|d| d.id == speaker_device_id)
            .unwrap_or(0);

        let mut status = String::new();
        if mic_providers.is_empty() && speaker_providers.is_empty() {
            status.push_str("no audio providers in atlas — boot the audio package first");
        }

        Self {
            mic_providers,
            speaker_providers,
            mic_devices,
            speaker_devices,
            mic_cap_id,
            mic_device_id,
            speaker_cap_id,
            speaker_device_id,
            section: AudioSection::MicProvider,
            cursor_mp,
            cursor_md,
            cursor_sp,
            cursor_sd,
            status,
        }
    }

    fn current_len(&self) -> usize {
        match self.section {
            AudioSection::MicProvider => self.mic_providers.len(),
            AudioSection::MicDevice => self.mic_devices.len(),
            AudioSection::SpeakerProvider => self.speaker_providers.len(),
            AudioSection::SpeakerDevice => self.speaker_devices.len(),
        }
    }
    fn current_cursor(&self) -> usize {
        match self.section {
            AudioSection::MicProvider => self.cursor_mp,
            AudioSection::MicDevice => self.cursor_md,
            AudioSection::SpeakerProvider => self.cursor_sp,
            AudioSection::SpeakerDevice => self.cursor_sd,
        }
    }
    fn current_cursor_mut(&mut self) -> &mut usize {
        match self.section {
            AudioSection::MicProvider => &mut self.cursor_mp,
            AudioSection::MicDevice => &mut self.cursor_md,
            AudioSection::SpeakerProvider => &mut self.cursor_sp,
            AudioSection::SpeakerDevice => &mut self.cursor_sd,
        }
    }
    fn cursor_up(&mut self) {
        let c = self.current_cursor_mut();
        if *c > 0 {
            *c -= 1;
        }
    }
    fn cursor_down(&mut self) {
        let n = self.current_len();
        let c = self.current_cursor_mut();
        if *c + 1 < n {
            *c += 1;
        }
    }
    fn next_section(&mut self) {
        self.section = match self.section {
            AudioSection::MicProvider => AudioSection::MicDevice,
            AudioSection::MicDevice => AudioSection::SpeakerProvider,
            AudioSection::SpeakerProvider => AudioSection::SpeakerDevice,
            AudioSection::SpeakerDevice => AudioSection::MicProvider,
        };
    }
    fn prev_section(&mut self) {
        self.section = match self.section {
            AudioSection::MicProvider => AudioSection::SpeakerDevice,
            AudioSection::MicDevice => AudioSection::MicProvider,
            AudioSection::SpeakerProvider => AudioSection::MicDevice,
            AudioSection::SpeakerDevice => AudioSection::SpeakerProvider,
        };
    }

    async fn enter(&mut self, atlas: &mut AtlasClient) {
        let i = self.current_cursor();
        match self.section {
            AudioSection::MicProvider => {
                if let Some(p) = self.mic_providers.get(i) {
                    let new_cap = p.capability_id.clone();
                    if new_cap != self.mic_cap_id {
                        self.mic_cap_id = new_cap.clone();
                        self.mic_devices = fetch_devices_filtered(atlas, &new_cap, "input").await;
                        self.mic_device_id.clear();
                        self.cursor_md = 0;
                    }
                    self.status = format!("mic provider → {new_cap}");
                }
            }
            AudioSection::MicDevice => {
                if let Some(d) = self.mic_devices.get(i) {
                    let id = d.id.clone();
                    let name = d.name.clone();
                    self.mic_device_id = id.clone();
                    match call_select_device(atlas, &self.mic_cap_id, "input", &id).await {
                        Ok(()) => self.status = format!("mic device → {name} ({id})"),
                        Err(e) => self.status = format!("mic device → {id} (warn: {e:#})"),
                    }
                }
            }
            AudioSection::SpeakerProvider => {
                if let Some(p) = self.speaker_providers.get(i) {
                    let new_cap = p.capability_id.clone();
                    if new_cap != self.speaker_cap_id {
                        self.speaker_cap_id = new_cap.clone();
                        self.speaker_devices =
                            fetch_devices_filtered(atlas, &new_cap, "output").await;
                        self.speaker_device_id.clear();
                        self.cursor_sd = 0;
                    }
                    self.status = format!("speaker provider → {new_cap}");
                }
            }
            AudioSection::SpeakerDevice => {
                if let Some(d) = self.speaker_devices.get(i) {
                    let id = d.id.clone();
                    let name = d.name.clone();
                    self.speaker_device_id = id.clone();
                    match call_select_device(atlas, &self.speaker_cap_id, "output", &id).await {
                        Ok(()) => self.status = format!("speaker device → {name} ({id})"),
                        Err(e) => self.status = format!("speaker device → {id} (warn: {e:#})"),
                    }
                }
            }
        }
    }

    async fn refresh(&mut self, atlas: &mut AtlasClient) {
        *self = Self::load(
            atlas,
            &ChatConfig {
                mic_cap_id: (!self.mic_cap_id.is_empty()).then(|| self.mic_cap_id.clone()),
                mic_device_id: (!self.mic_device_id.is_empty()).then(|| self.mic_device_id.clone()),
                speaker_cap_id: (!self.speaker_cap_id.is_empty())
                    .then(|| self.speaker_cap_id.clone()),
                speaker_device_id: (!self.speaker_device_id.is_empty())
                    .then(|| self.speaker_device_id.clone()),
            },
        )
        .await;
        self.status = "refreshed".into();
    }

    fn draw(&self, frame: &mut ratatui::Frame) {
        let mut lines: Vec<Line> = Vec::with_capacity(64);
        let dim = Style::default().fg(Color::DarkGray);
        let normal = Style::default();
        let selected_style = Style::default()
            .fg(Color::Green)
            .add_modifier(Modifier::BOLD);
        let cursor_style = Style::default()
            .fg(Color::Black)
            .bg(Color::Cyan)
            .add_modifier(Modifier::BOLD);

        let render_provider_row = |i: usize,
                                   p: &atlas_pb::CapabilityRecord,
                                   sel: &str,
                                   in_section: bool,
                                   cursor: usize|
         -> Line {
            let is_cursor = in_section && i == cursor;
            let is_selected = p.capability_id == sel;
            let mark_left = if is_cursor { "▶" } else { " " };
            let bullet = if is_selected { "●" } else { "○" };
            let prefix = format!(" {mark_left} {bullet} ");
            let style = if is_cursor {
                cursor_style
            } else if is_selected {
                selected_style
            } else {
                normal
            };
            Line::from(vec![Span::styled(
                format!("{prefix}{}", p.capability_id),
                style,
            )])
        };

        let render_device_row = |i: usize,
                                 d: &crate::pb::audio::AudioDevice,
                                 sel: &str,
                                 in_section: bool,
                                 cursor: usize|
         -> Line {
            let is_cursor = in_section && i == cursor;
            let is_selected = d.id == sel;
            let mark_left = if is_cursor { "▶" } else { " " };
            let bullet = if is_selected { "●" } else { "○" };
            let mut tags: Vec<String> = Vec::new();
            if d.is_default {
                tags.push("default".into());
            }
            if !d.note.is_empty() {
                tags.push(format!("⚠ {}", d.note));
            }
            let suffix = if tags.is_empty() {
                String::new()
            } else {
                format!("   [{}]", tags.join(", "))
            };
            let body = format!(" {mark_left} {bullet} {:<10}  {}{}", d.id, d.name, suffix);
            let style = if is_cursor {
                cursor_style
            } else if is_selected {
                selected_style
            } else {
                normal
            };
            Line::from(vec![Span::styled(body, style)])
        };

        let section_title = |title: &str, active: bool| -> Line {
            let bar = "━".repeat(8);
            let prefix = if active { "▼" } else { " " };
            let style = if active {
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD)
            } else {
                Style::default()
                    .fg(Color::Gray)
                    .add_modifier(Modifier::BOLD)
            };
            Line::from(vec![Span::styled(
                format!("{prefix} {bar} {title} {bar}"),
                style,
            )])
        };

        // ─── MICROPHONE ───
        lines.push(Line::from(""));
        lines.push(Line::from(vec![Span::styled(
            "  MICROPHONE",
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD),
        )]));
        lines.push(section_title(
            "Provider",
            self.section == AudioSection::MicProvider,
        ));
        if self.mic_providers.is_empty() {
            lines.push(Line::from(vec![Span::styled(
                "    (no providers — rbnx boot first)",
                dim,
            )]));
        } else {
            for (i, p) in self.mic_providers.iter().enumerate() {
                lines.push(render_provider_row(
                    i,
                    p,
                    &self.mic_cap_id,
                    self.section == AudioSection::MicProvider,
                    self.cursor_mp,
                ));
            }
        }
        lines.push(section_title(
            "Device",
            self.section == AudioSection::MicDevice,
        ));
        if self.mic_devices.is_empty() {
            lines.push(Line::from(vec![Span::styled(
                "    (no devices — impl missing list_devices, or rebuild package)",
                dim,
            )]));
        } else {
            for (i, d) in self.mic_devices.iter().enumerate() {
                lines.push(render_device_row(
                    i,
                    d,
                    &self.mic_device_id,
                    self.section == AudioSection::MicDevice,
                    self.cursor_md,
                ));
            }
        }

        // ─── SPEAKER ───
        lines.push(Line::from(""));
        lines.push(Line::from(vec![Span::styled(
            "  SPEAKER",
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD),
        )]));
        lines.push(section_title(
            "Provider",
            self.section == AudioSection::SpeakerProvider,
        ));
        if self.speaker_providers.is_empty() {
            lines.push(Line::from(vec![Span::styled(
                "    (no providers — rbnx boot first)",
                dim,
            )]));
        } else {
            for (i, p) in self.speaker_providers.iter().enumerate() {
                lines.push(render_provider_row(
                    i,
                    p,
                    &self.speaker_cap_id,
                    self.section == AudioSection::SpeakerProvider,
                    self.cursor_sp,
                ));
            }
        }
        lines.push(section_title(
            "Device",
            self.section == AudioSection::SpeakerDevice,
        ));
        if self.speaker_devices.is_empty() {
            lines.push(Line::from(vec![Span::styled(
                "    (no devices — impl missing list_devices, or rebuild package)",
                dim,
            )]));
        } else {
            for (i, d) in self.speaker_devices.iter().enumerate() {
                lines.push(render_device_row(
                    i,
                    d,
                    &self.speaker_device_id,
                    self.section == AudioSection::SpeakerDevice,
                    self.cursor_sd,
                ));
            }
        }

        lines.push(Line::from(""));

        let area = frame.area();
        let chunks = Layout::vertical([
            Constraint::Min(0),
            Constraint::Length(1),
            Constraint::Length(1),
        ])
        .split(area);

        let body = Paragraph::new(lines)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(" Audio Settings — ● selected · ▶ cursor "),
            )
            .wrap(Wrap { trim: false });
        frame.render_widget(body, chunks[0]);

        let status_line = if self.status.is_empty() {
            String::from(" ")
        } else {
            format!(" {}", self.status)
        };
        let status = Paragraph::new(status_line).style(Style::default().fg(Color::Cyan));
        frame.render_widget(status, chunks[1]);

        let help = Paragraph::new(
            " Tab/Shift+Tab: section · ↑↓ jk: item · Enter/Space: pick · r: refresh · Esc/Ctrl+A: close",
        )
        .style(dim);
        frame.render_widget(help, chunks[2]);
    }
}

async fn run_audio_settings_page(
    atlas_endpoint: &str,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    cfg: ChatConfig,
) -> Result<ChatConfig> {
    let mut atlas = AtlasClient::connect(atlas_endpoint)
        .await
        .with_context(|| format!("connect to atlas at '{atlas_endpoint}' for audio settings"))?;
    let mut page = AudioSettingsPage::load(&mut atlas, &cfg).await;

    loop {
        terminal.draw(|f| page.draw(f))?;
        if event::poll(std::time::Duration::from_millis(150))?
            && let Event::Key(key) = event::read()?
        {
            match (key.modifiers, key.code) {
                (_, KeyCode::Tab) => page.next_section(),
                (_, KeyCode::BackTab) => page.prev_section(),
                (_, KeyCode::Up) | (_, KeyCode::Char('k')) => page.cursor_up(),
                (_, KeyCode::Down) | (_, KeyCode::Char('j')) => page.cursor_down(),
                (_, KeyCode::Enter) | (_, KeyCode::Char(' ')) => page.enter(&mut atlas).await,
                (_, KeyCode::Char('r')) => page.refresh(&mut atlas).await,
                (KeyModifiers::CONTROL, KeyCode::Char('a'))
                | (_, KeyCode::Esc)
                | (_, KeyCode::Char('q')) => break,
                _ => {}
            }
        }
    }

    let new_cfg = ChatConfig {
        mic_cap_id: (!page.mic_cap_id.is_empty()).then_some(page.mic_cap_id),
        mic_device_id: (!page.mic_device_id.is_empty()).then_some(page.mic_device_id),
        speaker_cap_id: (!page.speaker_cap_id.is_empty()).then_some(page.speaker_cap_id),
        speaker_device_id: (!page.speaker_device_id.is_empty()).then_some(page.speaker_device_id),
    };
    if let Err(e) = save_chat_config(&new_cfg) {
        log::warn!("could not save chat config: {e:#}");
    }
    Ok(new_cfg)
}

/// Render a single-line status page and pause briefly so the user can
/// read it before the next picker step (or the chat) takes over the
/// screen. Long messages now sit for 1.4 s — the previous 400 ms felt
/// like "the page just flashed and disappeared."
fn flash_picker_message(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    msg: &str,
) -> Result<()> {
    terminal.draw(|f| {
        let body = Paragraph::new(msg)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(" rbnx chat — audio "),
            )
            .wrap(Wrap { trim: false });
        f.render_widget(body, f.area());
    })?;
    std::thread::sleep(std::time::Duration::from_millis(1400));
    Ok(())
}

async fn run_tui(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    atlas_endpoint: &str,
    liaison_endpoint: &str,
    mut chat_cfg: ChatConfig,
    audio_warnings: &[String],
) -> Result<()> {
    let local_user = format!("local:{}", whoami_username());
    let mut initial: Vec<ChatMessage> = Vec::with_capacity(1 + audio_warnings.len());
    initial.push(ChatMessage {
        role: Role::Status,
        text: format!(
            "Connected to Liaison at {liaison_endpoint} as {local_user}. \
             Enter = send · Ctrl+V = voice (auto end on silence) · Ctrl+A = audio settings · Esc = abort turn · Ctrl+C = quit."
        ),
    });
    for w in audio_warnings {
        initial.push(ChatMessage {
            role: Role::Status,
            text: w.clone(),
        });
    }
    let messages: Rc<RefCell<Vec<ChatMessage>>> = Rc::new(RefCell::new(initial));
    let mut input = String::new();
    let mut scroll: u16 = 0;
    let mut busy = false;
    let session_id = Uuid::new_v4().to_string();

    loop {
        draw(terminal, &messages.borrow(), &input, scroll, busy)?;

        if event::poll(std::time::Duration::from_millis(50))?
            && let Event::Key(key) = event::read()?
        {
            if key.modifiers.contains(KeyModifiers::CONTROL) && key.code == KeyCode::Char('c') {
                let _ = notify_session_end(liaison_endpoint, &session_id, &local_user).await;
                break;
            }

            // Ctrl+A → btop-style single-page audio settings dashboard.
            // All four sections (mic provider, mic device, speaker
            // provider, speaker device) visible at once, Tab to cycle,
            // ↑↓ to move within a section, Enter to pick. Esc to close.
            if !busy
                && key.modifiers.contains(KeyModifiers::CONTROL)
                && key.code == KeyCode::Char('a')
            {
                match run_audio_settings_page(atlas_endpoint, terminal, chat_cfg.clone()).await {
                    Ok(new_cfg) => {
                        chat_cfg = new_cfg;
                        messages.borrow_mut().push(ChatMessage {
                            role: Role::Status,
                            text: format!(
                                "audio settings updated: mic={}/{} · speaker={}/{}",
                                chat_cfg.mic_cap_id.as_deref().unwrap_or("(unset)"),
                                chat_cfg.mic_device_id.as_deref().unwrap_or("default"),
                                chat_cfg.speaker_cap_id.as_deref().unwrap_or("(unset)"),
                                chat_cfg.speaker_device_id.as_deref().unwrap_or("default"),
                            ),
                        });
                    }
                    Err(e) => messages.borrow_mut().push(ChatMessage {
                        role: Role::Status,
                        text: format!("audio settings: {e:#}"),
                    }),
                }
                continue;
            }

            // Ctrl+V → push-to-talk voice session (auto-ends on silence).
            if !busy
                && key.modifiers.contains(KeyModifiers::CONTROL)
                && key.code == KeyCode::Char('v')
            {
                busy = true;
                messages.borrow_mut().push(ChatMessage {
                    role: Role::Status,
                    text: "Ctrl+V — starting voice session…".to_string(),
                });
                draw(terminal, &messages.borrow(), &input, scroll, busy)?;
                if let Err(e) = run_voice_session_with_esc_abort(
                    liaison_endpoint,
                    &session_id,
                    &local_user,
                    Rc::clone(&messages),
                    terminal,
                    &input,
                    &mut scroll,
                    &chat_cfg,
                )
                .await
                {
                    messages.borrow_mut().push(ChatMessage {
                        role: Role::Status,
                        text: format!("Voice error: {e:#}"),
                    });
                }
                busy = false;
                continue;
            }

            if busy {
                match key.code {
                    KeyCode::PageUp => scroll = scroll.saturating_add(5),
                    KeyCode::PageDown => scroll = scroll.saturating_sub(5),
                    _ => {}
                }
                continue;
            }
            match key.code {
                KeyCode::Enter => {
                    let msg = input.trim().to_string();
                    input.clear();
                    scroll = 0;
                    if msg.is_empty() {
                        continue;
                    }
                    if msg == "quit" || msg == "exit" {
                        let _ =
                            notify_session_end(liaison_endpoint, &session_id, &local_user).await;
                        break;
                    }
                    messages.borrow_mut().push(ChatMessage {
                        role: Role::User,
                        text: msg.clone(),
                    });
                    busy = true;
                    draw(terminal, &messages.borrow(), &input, scroll, busy)?;

                    match run_text_intent_with_esc_abort(
                        liaison_endpoint,
                        &session_id,
                        &local_user,
                        &msg,
                        Rc::clone(&messages),
                        terminal,
                        &input,
                        &mut scroll,
                    )
                    .await
                    {
                        Ok(()) => {}
                        Err(e) => {
                            messages.borrow_mut().push(ChatMessage {
                                role: Role::Status,
                                text: format!("Error: {e:#}"),
                            });
                        }
                    }
                    busy = false;
                }
                KeyCode::Char(c) => input.push(c),
                KeyCode::Backspace => {
                    input.pop();
                }
                KeyCode::PageUp => scroll = scroll.saturating_add(5),
                KeyCode::PageDown => scroll = scroll.saturating_sub(5),
                _ => {}
            }
        }
    }
    Ok(())
}

// ── Liaison gRPC helpers ─────────────────────────────────────────────────────

fn build_text_task(session_id: &str, user_id: &str, text: &str) -> crate::pb::pilot::Task {
    use crate::pb::pilot::Task;
    const INTENT_SOURCE_TEXT: u32 = 0;
    Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: INTENT_SOURCE_TEXT,
        text: text.to_string(),
        audio_data: vec![],
        context_json: serde_json::json!({"user_id": user_id, "modality": "text"}).to_string(),
        timestamp_ms: now_ms(),
    }
}

fn build_control_task(
    session_id: &str,
    user_id: &str,
    extra_context_json: &str,
) -> crate::pb::pilot::Task {
    use crate::pb::pilot::Task;
    const INTENT_SOURCE_TEXT: u32 = 0;
    let mut ctx: serde_json::Value =
        serde_json::from_str(extra_context_json.trim()).unwrap_or_else(|_| serde_json::json!({}));
    if let Some(obj) = ctx.as_object_mut() {
        obj.entry("user_id").or_insert(serde_json::json!(user_id));
    }
    Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: INTENT_SOURCE_TEXT,
        text: String::new(),
        audio_data: vec![],
        context_json: ctx.to_string(),
        timestamp_ms: now_ms(),
    }
}

async fn notify_session_end(liaison_endpoint: &str, session_id: &str, user_id: &str) -> Result<()> {
    use crate::pb::contracts::system_liaison_client::SystemLiaisonClient;

    let mut client = SystemLiaisonClient::connect(liaison_endpoint.to_string())
        .await
        .context("failed to connect to Liaison for session_end")?;

    let task = build_control_task(session_id, user_id, r#"{"session_end":true}"#);
    let mut stream = client
        .submit_task(tonic::Request::new(task))
        .await
        .context("Liaison Stream session_end failed")?
        .into_inner();
    while stream.next().await.is_some() {}
    Ok(())
}

async fn abort_session(liaison_endpoint: &str, session_id: &str, user_id: &str) -> Result<()> {
    use crate::pb::contracts::system_liaison_client::SystemLiaisonClient;

    let mut client = SystemLiaisonClient::connect(liaison_endpoint.to_string())
        .await
        .context("failed to connect to Liaison for abort_turn")?;

    let task = build_control_task(session_id, user_id, r#"{"abort_turn":true}"#);
    let _ = client
        .submit_task(tonic::Request::new(task))
        .await
        .context("Liaison abort_turn Stream failed")?;
    Ok(())
}

// ── Text turn ────────────────────────────────────────────────────────────────

#[allow(clippy::too_many_arguments)]
async fn run_text_intent_with_esc_abort(
    liaison_endpoint: &str,
    session_id: &str,
    user_id: &str,
    user_msg: &str,
    messages: Rc<RefCell<Vec<ChatMessage>>>,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    input: &str,
    scroll: &mut u16,
) -> Result<()> {
    use crate::pb::contracts::system_liaison_client::SystemLiaisonClient;
    use crate::pb::pilot::PilotEvent;
    use tonic::Status;

    let (tx, mut rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(64);
    let liaison_ep = liaison_endpoint.to_string();
    let task = build_text_task(session_id, user_id, user_msg);

    let _stream_task = tokio::spawn(async move {
        let mut client = match SystemLiaisonClient::connect(liaison_ep.clone()).await {
            Ok(c) => c,
            Err(e) => {
                let _ = tx.send(Err(Status::unavailable(e.to_string()))).await;
                return;
            }
        };
        let stream = match client.submit_task(tonic::Request::new(task)).await {
            Ok(r) => r.into_inner(),
            Err(e) => {
                let _ = tx.send(Err(e)).await;
                return;
            }
        };
        let mut stream = stream;
        while let Some(item) = stream.next().await {
            if tx.send(item).await.is_err() {
                break;
            }
        }
    });

    loop {
        tokio::select! {
            biased;
            item = rx.recv() => {
                match item {
                    None => break,
                    Some(Ok(event)) => {
                        apply_pilot_event(&messages, &event)?;
                        draw(terminal, &messages.borrow(), input, 0, true)?;
                    }
                    Some(Err(e)) => {
                        messages.borrow_mut().push(ChatMessage {
                            role: Role::Status,
                            text: format!("Liaison stream error: {e}"),
                        });
                        draw(terminal, &messages.borrow(), input, 0, true)?;
                        break;
                    }
                }
            }
            _ = tokio::time::sleep(std::time::Duration::from_millis(25)) => {
                if event::poll(std::time::Duration::ZERO)?
                    && let Event::Key(key) = event::read()? {
                        match key.code {
                            KeyCode::Esc => {
                                let _ = abort_session(liaison_endpoint, session_id, user_id).await;
                                messages.borrow_mut().push(ChatMessage {
                                    role: Role::Status,
                                    text: "Esc — abort_turn sent (in-flight turn should stop)."
                                        .to_string(),
                                });
                                draw(terminal, &messages.borrow(), input, *scroll, true)?;
                            }
                            KeyCode::PageUp => {
                                *scroll = scroll.saturating_add(5);
                                draw(terminal, &messages.borrow(), input, *scroll, true)?;
                            }
                            KeyCode::PageDown => {
                                *scroll = scroll.saturating_sub(5);
                                draw(terminal, &messages.borrow(), input, *scroll, true)?;
                            }
                            _ => {}
                        }
                    }
            }
        }
    }
    Ok(())
}

// ── Voice turn ───────────────────────────────────────────────────────────────

#[allow(clippy::too_many_arguments)]
async fn run_voice_session_with_esc_abort(
    liaison_endpoint: &str,
    session_id: &str,
    user_id: &str,
    messages: Rc<RefCell<Vec<ChatMessage>>>,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    input: &str,
    scroll: &mut u16,
    chat_cfg: &ChatConfig,
) -> Result<()> {
    use crate::pb::contracts::system_liaison_client::SystemLiaisonClient;
    use crate::pb::liaison::{StartVoiceSessionRequest, VoiceEvent};
    use tonic::Status;

    let req = StartVoiceSessionRequest {
        session_id: session_id.to_string(),
        client_user_id: user_id.to_string(),
        record_seconds: voice_record_seconds(),
        language: voice_language(),
        tts_enabled: voice_tts_enabled(),
        mic_node_id: voice_node_with_cfg("ROBONIX_CHAT_MIC_NODE", chat_cfg.mic_cap_id.as_deref()),
        asr_node_id: voice_node("ROBONIX_CHAT_ASR_NODE"),
        voiceprint_node_id: String::new(),
        tts_node_id: voice_node("ROBONIX_CHAT_TTS_NODE"),
        speaker_node_id: voice_node_with_cfg(
            "ROBONIX_CHAT_SPEAKER_NODE",
            chat_cfg.speaker_cap_id.as_deref(),
        ),
        context_json: String::new(),
    };

    let (tx, mut rx) = tokio::sync::mpsc::channel::<Result<VoiceEvent, Status>>(64);
    let liaison_ep = liaison_endpoint.to_string();

    let _stream_task = tokio::spawn(async move {
        let mut client = match SystemLiaisonClient::connect(liaison_ep.clone()).await {
            Ok(c) => c,
            Err(e) => {
                let _ = tx.send(Err(Status::unavailable(e.to_string()))).await;
                return;
            }
        };
        let stream = match client.start_voice_session(tonic::Request::new(req)).await {
            Ok(r) => r.into_inner(),
            Err(e) => {
                let _ = tx.send(Err(e)).await;
                return;
            }
        };
        let mut stream = stream;
        while let Some(item) = stream.next().await {
            if tx.send(item).await.is_err() {
                break;
            }
        }
    });

    loop {
        tokio::select! {
            biased;
            item = rx.recv() => {
                match item {
                    None => break,
                    Some(Ok(event)) => {
                        apply_voice_event(&messages, &event)?;
                        draw(terminal, &messages.borrow(), input, 0, true)?;
                    }
                    Some(Err(e)) => {
                        messages.borrow_mut().push(ChatMessage {
                            role: Role::Status,
                            text: format!("Voice stream error: {e}"),
                        });
                        draw(terminal, &messages.borrow(), input, 0, true)?;
                        break;
                    }
                }
            }
            _ = tokio::time::sleep(std::time::Duration::from_millis(25)) => {
                if event::poll(std::time::Duration::ZERO)?
                    && let Event::Key(key) = event::read()? {
                        match key.code {
                            KeyCode::Esc => {
                                let _ = abort_session(liaison_endpoint, session_id, user_id).await;
                                messages.borrow_mut().push(ChatMessage {
                                    role: Role::Status,
                                    text: "Esc — abort_turn sent (Pilot stops; voice playback may still finish).".to_string(),
                                });
                                draw(terminal, &messages.borrow(), input, *scroll, true)?;
                            }
                            KeyCode::PageUp => {
                                *scroll = scroll.saturating_add(5);
                                draw(terminal, &messages.borrow(), input, *scroll, true)?;
                            }
                            KeyCode::PageDown => {
                                *scroll = scroll.saturating_sub(5);
                                draw(terminal, &messages.borrow(), input, *scroll, true)?;
                            }
                            _ => {}
                        }
                    }
            }
        }
    }
    Ok(())
}

fn voice_record_seconds() -> u32 {
    std::env::var("ROBONIX_CHAT_VOICE_SECONDS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0) // 0 → server default (5s)
}

fn voice_language() -> String {
    std::env::var("ROBONIX_CHAT_VOICE_LANG").unwrap_or_default()
}

fn voice_tts_enabled() -> bool {
    !matches!(
        std::env::var("ROBONIX_CHAT_VOICE_TTS").as_deref(),
        Ok("0") | Ok("false") | Ok("no") | Ok("off")
    )
}

fn voice_node(env_key: &str) -> String {
    std::env::var(env_key).unwrap_or_default()
}

/// Same as `voice_node` but falls back to `cfg_value` (from chat.yaml)
/// when the env var is unset / empty. Empty result still means "let
/// liaison auto-pick from atlas".
fn voice_node_with_cfg(env_key: &str, cfg_value: Option<&str>) -> String {
    if let Ok(v) = std::env::var(env_key)
        && !v.is_empty()
    {
        return v;
    }
    cfg_value.unwrap_or("").to_string()
}

// ── Event → ChatMessage rendering ────────────────────────────────────────────

fn apply_pilot_event(
    messages: &Rc<RefCell<Vec<ChatMessage>>>,
    event: &crate::pb::pilot::PilotEvent,
) -> Result<()> {
    // event_kind discriminants — see PilotEvent.msg.
    const EVT_TEXT_CHUNK: u32 = 0;
    const EVT_PLAN: u32 = 1;
    const EVT_FINAL_TEXT: u32 = 4;

    let mut m = messages.borrow_mut();
    match event.event_kind {
        EVT_TEXT_CHUNK => {
            let t = event.text_chunk.clone();
            if let Some(last) = m.last_mut() {
                if matches!(last.role, Role::Agent) {
                    last.text.push_str(&t);
                } else {
                    m.push(ChatMessage {
                        role: Role::Agent,
                        text: t,
                    });
                }
            } else {
                m.push(ChatMessage {
                    role: Role::Agent,
                    text: t,
                });
            }
        }
        EVT_FINAL_TEXT => {
            let t = event.final_text.clone();
            let has_agent = m.last().is_some_and(|x| matches!(x.role, Role::Agent));
            if !has_agent && !t.is_empty() {
                m.push(ChatMessage {
                    role: Role::Agent,
                    text: t,
                });
            }
        }
        EVT_PLAN => {
            // dev called this EVT_TASK_GRAPH with `event.task_graph` carrying
            // tool_name + args_json. dev-packaging renamed the message to
            // Plan/CapabilityCall; only contract_id + args_json are exposed,
            // so we leaf-strip contract_id back into a tool-name lookalike to
            // preserve the same `[r{round}] {name}({args})` line shape.
            if let Some(ref p) = event.plan {
                for call in &p.calls {
                    let leaf = call
                        .contract_id
                        .rsplit_once('/')
                        .map(|(_, l)| l.to_string())
                        .unwrap_or_else(|| call.contract_id.clone());
                    m.push(ChatMessage {
                        role: Role::ToolCall,
                        text: format!("[r{}] {}({})", p.round, leaf, call.args_json),
                    });
                }
            }
        }
        _ => {}
    }
    Ok(())
}

fn apply_voice_event(
    messages: &Rc<RefCell<Vec<ChatMessage>>>,
    event: &crate::pb::liaison::VoiceEvent,
) -> Result<()> {
    // Mirror the kinds in voice.rs / VoiceEvent.msg
    const KIND_SESSION_STARTED: u32 = 0;
    const KIND_RECORDING_STARTED: u32 = 1;
    const KIND_RECORDING_DONE: u32 = 2;
    const KIND_ASR_PARTIAL: u32 = 3;
    const KIND_ASR_FINAL: u32 = 4;
    const KIND_USER_IDENTIFIED: u32 = 5;
    const KIND_PILOT: u32 = 6;
    const KIND_TTS_STARTED: u32 = 7;
    const KIND_TTS_DONE: u32 = 8;
    const KIND_SESSION_DONE: u32 = 9;
    const KIND_ERROR: u32 = 10;

    match event.event_kind {
        KIND_SESSION_STARTED | KIND_RECORDING_STARTED | KIND_RECORDING_DONE => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("voice · {}", event.status_message),
            });
        }
        KIND_ASR_PARTIAL => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("asr (partial, {:.2}): {}", event.confidence, event.text),
            });
        }
        KIND_ASR_FINAL => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::User,
                text: format!("(voice) {}", event.text),
            });
        }
        KIND_USER_IDENTIFIED => {
            let label = if event.status_message.is_empty() {
                format!("identified user → {}", event.user_id)
            } else {
                format!(
                    "identified user → {} · {}",
                    event.user_id, event.status_message
                )
            };
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: label,
            });
        }
        KIND_PILOT => {
            if let Some(ref pe) = event.pilot {
                apply_pilot_event(messages, pe)?;
            }
        }
        KIND_TTS_STARTED => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("tts · {}", event.status_message),
            });
        }
        KIND_TTS_DONE => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("tts done · {}", event.status_message),
            });
        }
        KIND_SESSION_DONE => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Status,
                text: "voice session done".to_string(),
            });
        }
        KIND_ERROR => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Status,
                text: format!("voice error: {}", event.error),
            });
        }
        _ => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("voice (kind={}) {}", event.event_kind, event.status_message),
            });
        }
    }
    Ok(())
}

fn draw(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    messages: &[ChatMessage],
    input: &str,
    scroll: u16,
    busy: bool,
) -> Result<()> {
    terminal.draw(|f| {
        let area = f.area();
        let chunks = Layout::vertical([Constraint::Min(3), Constraint::Length(3)]).split(area);

        let mut lines: Vec<Line> = Vec::new();
        for msg in messages {
            let (prefix, indent, style) = match msg.role {
                Role::User => (
                    "You:   ",
                    "       ",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ),
                // "Robonix:" — the user-facing system name. The reply
                // went through Liaison → Pilot → tools → Liaison → here,
                // so naming it after one internal crate ("Pilot") leaks
                // an architectural detail that the user has no reason
                // to know or care about.
                Role::Agent => ("Robonix: ", "         ", Style::default().fg(Color::Green)),
                Role::ToolCall => (">      ", "       ", Style::default().fg(Color::Yellow)),
                Role::Status => (
                    "",
                    "",
                    Style::default()
                        .fg(Color::Magenta)
                        .add_modifier(Modifier::ITALIC),
                ),
                Role::Voice => (
                    "[v]    ",
                    "       ",
                    Style::default()
                        .fg(Color::Blue)
                        .add_modifier(Modifier::ITALIC),
                ),
            };
            for (i, text_line) in msg.text.lines().enumerate() {
                let lead = if i == 0 { prefix } else { indent };
                lines.push(Line::from(vec![
                    Span::styled(lead, style),
                    Span::styled(text_line.to_string(), style),
                ]));
            }
        }

        let status = if busy { " [thinking...]" } else { "" };
        let block = Block::default()
            .borders(Borders::ALL)
            .title(format!(" Robonix{status} "));
        let inner = block.inner(chunks[0]);

        let text_only = Paragraph::new(lines.clone()).wrap(Wrap { trim: false });
        let total_lines = text_only.line_count(inner.width) as u16;
        let visible = inner.height;
        let auto_scroll = if scroll == 0 {
            total_lines.saturating_sub(visible)
        } else {
            total_lines.saturating_sub(visible).saturating_sub(scroll)
        };

        let history = Paragraph::new(lines)
            .block(block)
            .wrap(Wrap { trim: false })
            .scroll((auto_scroll, 0));
        f.render_widget(history, chunks[0]);

        let input_widget =
            Paragraph::new(input.to_string()).block(Block::default().borders(Borders::ALL).title(
                " > Enter = send · Ctrl+V = voice (auto end) · Esc = abort · Ctrl+C = quit ",
            ));
        f.render_widget(input_widget, chunks[1]);
    })?;
    Ok(())
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

/// Pilot/Liaison bind IPv4 (`0.0.0.0`). Resolving `localhost` often prefers
/// `::1`, so the gRPC client hits IPv6 and gets connection refused — force IPv4 loopback.
fn localhost_to_ipv4_loopback(url: &str) -> String {
    url.replace("localhost", "127.0.0.1")
}

/// Tiny username probe — avoids pulling in `whoami` for the CLI by reading
/// $USER/$USERNAME with a "user" fallback. Liaison uses the real `whoami`
/// crate when it stamps the canonical `local:<user>` identity.
fn whoami_username() -> String {
    std::env::var("USER")
        .or_else(|_| std::env::var("USERNAME"))
        .unwrap_or_else(|_| "user".to_string())
}
