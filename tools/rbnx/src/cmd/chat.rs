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
    layout::{Alignment, Constraint, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Clear, Paragraph, Wrap},
};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use std::cell::RefCell;
use std::io;
use std::rc::Rc;
use std::sync::atomic::{AtomicU64, Ordering};
use tokio_stream::StreamExt;
use uuid::Uuid;

const CONSUMER_ID: &str = "rbnx-cli/chat";
// Liaison was split into two contracts (submit + voice). chat only needs
// to find one of them in atlas to know liaison is up; submit is the
// always-present one (voice is optional / mode-gated), so wait on it.
const LIAISON_CONTRACT_ID: &str = "robonix/system/liaison/submit";

/// Atlas contract ids for the audio primitives that have user-visible
/// device choices on a multi-provider host (e.g. local ALSA driver vs
/// the macOS bridge). asr/tts are software backends with one provider
/// per box, so we don't prompt for them.
const MIC_CONTRACT: &str = "robonix/primitive/audio/mic";
const SPEAKER_CONTRACT: &str = "robonix/primitive/audio/speaker";

struct ChatMessage {
    role: Role,
    text: String,
    /// Per-message author label override. When `role == Role::User` and
    /// `label` is Some(name), the chat renders that name (padded) instead
    /// of the default "You" prefix — so a voiceprint-identified turn shows
    /// "Alice    " instead of generic "You".
    label: Option<String>,
}

enum Role {
    User,
    Agent,
    ToolCall,
    Status,
    Voice,
    /// Hard-deny rendered in bold red. Used for:
    ///   - voiceprint access-control rejection (heard speaker ≠ pinned controller)
    ///   - sentinel rule intercept (executor refused the capability call)
    ///
    /// Operator must see these immediately; they survive auto-scroll.
    Denied,
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
    /// Currently active controller's user_id (e.g. "voice:alice"). When
    /// set, voice sessions whose identified speaker doesn't match are
    /// blocked at the chat client — no Pilot call, no car action. Empty
    /// / unset = allow anyone.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    current_controller: Option<String>,
    /// When true, replies to TYPED input (Enter) are also synthesised
    /// and played through the chosen speaker, and pilot is told the
    /// reply will be spoken (context_json.modality=voice) so it follows
    /// its brevity / no-markdown rule. Voice input (Ctrl+V) always
    /// produces voice output regardless of this flag.
    #[serde(default)]
    response_voice: bool,
}

// ── Voiceprint client-side cache ────────────────────────────────────────────
//
// Local mirror of the voiceprint catalog. Backed by `~/.robonix/voiceprint.json`
// and refreshed against the voiceprint service on every Ctrl+U open. Two
// reasons it exists:
//   1. Render the users modal instantly on open without an RPC stall.
//   2. Resolve `voice:<id>` → "Alice" on User-message render so the chat
//      shows "You (Alice)" without a per-draw network call.

#[derive(Clone, Default, serde::Serialize, serde::Deserialize)]
struct VoiceprintDb {
    #[serde(default)]
    users: std::collections::BTreeMap<String, VoiceprintUser>,
}

#[derive(Clone, serde::Serialize, serde::Deserialize)]
struct VoiceprintUser {
    name: String,
    /// Unix epoch seconds, just for display. Not used for matching.
    enrolled_at: u64,
}

fn voiceprint_db_path() -> Option<std::path::PathBuf> {
    dirs::home_dir().map(|h| h.join(".robonix").join("voiceprint.json"))
}

fn load_voiceprint_db() -> VoiceprintDb {
    voiceprint_db_path()
        .and_then(|p| std::fs::read_to_string(&p).ok())
        .and_then(|t| serde_json::from_str(&t).ok())
        .unwrap_or_default()
}

fn save_voiceprint_db(db: &VoiceprintDb) -> Result<()> {
    let p = voiceprint_db_path().context("no home dir")?;
    if let Some(parent) = p.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&p, serde_json::to_string_pretty(db)?)?;
    Ok(())
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
    let providers = atlas.query_capabilities("", contract_id, transport).await?;
    for provider in &providers {
        let has = provider
            .capabilities
            .iter()
            .any(|i| i.contract_id == contract_id && i.transport == transport as i32);
        if !has {
            continue;
        }
        let (_, endpoint, _) = atlas
            .connect_capability(CONSUMER_ID, &provider.id, contract_id, transport)
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
    anyhow::bail!("no {contract_id} capability found in Atlas registry")
}

// ── Audio-device first-run picker ───────────────────────────────────────────
//
// Resolution order for mic/speaker capability_ids when starting a voice
// session is: ROBONIX_CHAT_*_NODE env (highest, overrides everything) →
// chat.yaml on disk → first-run TUI picker. The picker only fires when
// neither env nor config supplies the provider_id; once chosen it's saved
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
    let mut need_mic = always || (!env_set("ROBONIX_CHAT_MIC_NODE") && cfg.mic_cap_id.is_none());
    let mut need_speaker =
        always || (!env_set("ROBONIX_CHAT_SPEAKER_NODE") && cfg.speaker_cap_id.is_none());

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

    // Validate stored pins against current atlas state. A pin pointing at a
    // provider that's not in this deploy (e.g. config saved from an earlier
    // deploy where the audio provider was named differently) would silently break
    // voice — re-prompt instead.
    if !need_mic
        && let Some(pin) = cfg.mic_cap_id.as_deref()
        && !pin_exists_in_atlas(&mut atlas, pin, MIC_CONTRACT).await
    {
        warnings.push(format!(
            "mic pin '{pin}' not in atlas (stale config) — re-prompting"
        ));
        cfg.mic_cap_id = None;
        cfg.mic_device_id = None;
        need_mic = true;
    }
    if !need_speaker
        && let Some(pin) = cfg.speaker_cap_id.as_deref()
        && !pin_exists_in_atlas(&mut atlas, pin, SPEAKER_CONTRACT).await
    {
        warnings.push(format!(
            "speaker pin '{pin}' not in atlas (stale config) — re-prompting"
        ));
        cfg.speaker_cap_id = None;
        cfg.speaker_device_id = None;
        need_speaker = true;
    }
    if !need_mic && !need_speaker {
        return Ok((cfg, warnings));
    }

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
            Ok(Some((provider_id, device_id))) => {
                cfg.mic_cap_id = Some(provider_id);
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
            Ok(Some((provider_id, device_id))) => {
                cfg.speaker_cap_id = Some(provider_id);
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

/// True iff atlas currently has a provider whose id (or namespace) matches the
/// pin AND it provides `contract` over GRPC. Used at chat startup to detect
/// stale pins from a prior deploy whose audio provider has since been renamed
/// or removed — caller drops the pin and re-prompts the picker instead of
/// silently letting voice fail with "no provider".
async fn pin_exists_in_atlas(atlas: &mut AtlasClient, pin: &str, contract: &str) -> bool {
    let Ok(providers) = atlas
        .query_capabilities("", contract, atlas_pb::Transport::Grpc)
        .await
    else {
        // Atlas unreachable — don't drop the pin on a transient failure;
        // the outer caller already warned and degraded.
        return true;
    };
    providers.iter().any(|r| r.id == pin || r.namespace == pin)
}

/// `Ok(Some((provider_id, device_id)))` = picked both layers; device_id may be ""
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

    // Layer A — provider (provider_id). FirstRun honours the saved choice
    // when it's still in atlas, or auto-picks the lone provider with a
    // 400 ms flash. Reconfigure (Ctrl+A) ALWAYS shows the TUI even
    // for a single entry — auto-pick on Ctrl+A looks identical to
    // "Ctrl+A did nothing", which is what the user just hit.
    let provider_id = match (mode, saved_cap_id) {
        (PickMode::FirstRun, Some(s)) if providers.iter().any(|p| p.id == s) => s.to_string(),
        (PickMode::FirstRun, _) if providers.len() == 1 => {
            let id = providers[0].id.clone();
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
    let device_id = pick_device_for_cap(
        atlas,
        terminal,
        &provider_id,
        label,
        kind,
        saved_device_id,
        mode,
    )
    .await?;

    // Tell the impl which device to use. Best-effort; ignore failures.
    if !device_id.is_empty()
        && let Err(e) = call_select_device(atlas, &provider_id, kind, &device_id).await
    {
        log::warn!("SelectAudioDevice on {provider_id} ({kind}={device_id}) failed: {e:#}");
    }

    Ok(Some((provider_id, device_id)))
}

/// Connect to `provider_id`'s list_devices capability, ask for the device
/// list, run a picker on the entries that match `kind` (input/output)
/// + duplex. Returns "" when the impl doesn't expose the contract.
async fn pick_device_for_cap(
    atlas: &mut AtlasClient,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    provider_id: &str,
    label: &str,
    kind: &str,
    saved_device_id: Option<&str>,
    mode: PickMode,
) -> Result<String> {
    use crate::pb::contracts::robonix_primitive_audio_list_devices_client::RobonixPrimitiveAudioListDevicesClient;

    const LIST_CONTRACT: &str = "robonix/primitive/audio/list_devices";
    // Reconfigure mode shows visible feedback for every silent path
    // below; FirstRun stays quiet so a missing list_devices contract
    // doesn't litter chat history with a non-actionable message.
    let reconf = matches!(mode, PickMode::Reconfigure);

    let endpoint = match atlas
        .connect_capability(
            CONSUMER_ID,
            provider_id,
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
                        "{provider_id} doesn't expose list_devices — \
                         using OS default device. Rebuild the package \
                         (bash scripts/build.sh) to pick up the new contract."
                    ),
                )?;
            }
            return Ok(String::new());
        }
    };

    let mut client = match RobonixPrimitiveAudioListDevicesClient::connect(endpoint.clone()).await {
        Ok(c) => c,
        Err(e) => {
            if reconf {
                flash_picker_message(
                    terminal,
                    &format!("connect list_devices on {provider_id}: {e}"),
                )?;
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
                flash_picker_message(terminal, &format!("ListAudioDevices on {provider_id}: {e}"))?;
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
                &format!("{provider_id} reports no {kind} devices — using OS default"),
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
    provider_id: &str,
    kind: &str,
    device_id: &str,
) -> Result<()> {
    use crate::pb::contracts::robonix_primitive_audio_select_device_client::RobonixPrimitiveAudioSelectDeviceClient;
    const SELECT_CONTRACT: &str = "robonix/primitive/audio/select_device";

    let (_, ep, _) = atlas
        .connect_capability(
            CONSUMER_ID,
            provider_id,
            SELECT_CONTRACT,
            atlas_pb::Transport::Grpc,
        )
        .await?;
    let endpoint = if ep.starts_with("http") {
        ep
    } else {
        format!("http://{ep}")
    };
    let mut client = RobonixPrimitiveAudioSelectDeviceClient::connect(endpoint).await?;
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
    providers: &[atlas_pb::CapabilityProvider],
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
                    Line::from(vec![Span::styled(format!("{mark}{}{detail}", r.id), style)])
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
                KeyCode::Enter => return Ok(Some(providers[idx].id.clone())),
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
    mic_providers: Vec<atlas_pb::CapabilityProvider>,
    speaker_providers: Vec<atlas_pb::CapabilityProvider>,
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
    provider_id: &str,
    kind: &str,
) -> Vec<crate::pb::audio::AudioDevice> {
    use crate::pb::contracts::robonix_primitive_audio_list_devices_client::RobonixPrimitiveAudioListDevicesClient;
    const LIST_CONTRACT: &str = "robonix/primitive/audio/list_devices";
    if provider_id.is_empty() {
        return Vec::new();
    }
    let endpoint = match atlas
        .connect_capability(
            CONSUMER_ID,
            provider_id,
            LIST_CONTRACT,
            atlas_pb::Transport::Grpc,
        )
        .await
    {
        Ok((_, ep, _)) if ep.starts_with("http") => ep,
        Ok((_, ep, _)) => format!("http://{ep}"),
        Err(_) => return Vec::new(),
    };
    let mut client = match RobonixPrimitiveAudioListDevicesClient::connect(endpoint).await {
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
                .map(|p| p.id.clone())
                .unwrap_or_default()
        });
        let speaker_cap_id = cfg.speaker_cap_id.clone().unwrap_or_else(|| {
            speaker_providers
                .first()
                .map(|p| p.id.clone())
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
            .position(|p| p.id == mic_cap_id)
            .unwrap_or(0);
        let cursor_sp = speaker_providers
            .iter()
            .position(|p| p.id == speaker_cap_id)
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
                    let new_cap = p.id.clone();
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
                    let new_cap = p.id.clone();
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
                current_controller: None, // not consumed by Audio refresh
                response_voice: false,    // not consumed by Audio refresh
            },
        )
        .await;
        self.status = "refreshed".into();
    }

    /// Centered modal popup with two side-by-side columns (Microphone /
    /// Speaker). Each column shows the provider list on top and the
    /// device list below it. The currently focused section (Tab cycles
    /// through MicProvider → MicDevice → SpeakerProvider → SpeakerDevice)
    /// gets a bright cyan border; the others render dim. Footer hints
    /// live in the outer block's title_bottom so they're always visible.
    fn draw(&self, frame: &mut ratatui::Frame) {
        let area = centered_rect(85, 80, frame.area());
        frame.render_widget(Clear, area);

        // Outer popup block — title at top, hints at bottom. Status (if
        // any) sits in the title bar on the right so it's seen but
        // doesn't eat list space.
        let outer_title = Line::from(vec![
            Span::raw(" "),
            Span::styled(
                "Audio Settings",
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw(" "),
        ]);
        let status_span = if self.status.is_empty() {
            Span::raw(" ")
        } else {
            Span::styled(
                format!(" {} ", self.status),
                Style::default().fg(Color::Yellow),
            )
        };
        let hints_line = Line::from(vec![
            Span::raw(" "),
            Span::styled("Tab", Style::default().fg(Color::White)),
            Span::styled(" switch column · ", Style::default().fg(Color::DarkGray)),
            Span::styled("↑↓ / jk", Style::default().fg(Color::White)),
            Span::styled(" move · ", Style::default().fg(Color::DarkGray)),
            Span::styled("Enter", Style::default().fg(Color::White)),
            Span::styled(" select · ", Style::default().fg(Color::DarkGray)),
            Span::styled("r", Style::default().fg(Color::White)),
            Span::styled(" refresh · ", Style::default().fg(Color::DarkGray)),
            Span::styled("Esc", Style::default().fg(Color::White)),
            Span::styled(" close ", Style::default().fg(Color::DarkGray)),
        ]);
        let outer = Block::default()
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Cyan))
            .title(outer_title)
            .title_top(Line::from(status_span).alignment(Alignment::Right))
            .title_bottom(hints_line);
        let inner = outer.inner(area);
        frame.render_widget(outer, area);

        // Two columns: microphone on the left, speaker on the right.
        let cols = Layout::horizontal([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(inner);
        self.draw_side(
            frame,
            cols[0],
            "Microphone",
            &self.mic_providers,
            &self.mic_cap_id,
            self.cursor_mp,
            &self.mic_devices,
            &self.mic_device_id,
            self.cursor_md,
            self.section == AudioSection::MicProvider,
            self.section == AudioSection::MicDevice,
        );
        self.draw_side(
            frame,
            cols[1],
            "Speaker",
            &self.speaker_providers,
            &self.speaker_cap_id,
            self.cursor_sp,
            &self.speaker_devices,
            &self.speaker_device_id,
            self.cursor_sd,
            self.section == AudioSection::SpeakerProvider,
            self.section == AudioSection::SpeakerDevice,
        );
    }

    /// Render one column (microphone or speaker). The Provider sub-pane
    /// sits on top, the Device sub-pane below. Each pane has its own
    /// Block, and the currently-focused one gets a bright cyan border
    /// so the user can see at a glance which list ↑↓ will move within.
    #[allow(clippy::too_many_arguments)]
    fn draw_side(
        &self,
        frame: &mut ratatui::Frame,
        area: Rect,
        column_title: &str,
        providers: &[atlas_pb::CapabilityProvider],
        sel_cap: &str,
        cursor_p: usize,
        devices: &[crate::pb::audio::AudioDevice],
        sel_dev: &str,
        cursor_d: usize,
        provider_focused: bool,
        device_focused: bool,
    ) {
        // Allocate the top half to providers (small list, usually 1-2
        // entries) and the rest to devices (long list, can be 20+).
        // Devices get the meat of the space.
        let panes = Layout::vertical([Constraint::Length(6), Constraint::Min(3)])
            .margin(1)
            .split(area);

        // Column title bar (above both panes). Subtle, just labels which
        // device side we're looking at.
        let title_area = Rect {
            x: area.x + 1,
            y: area.y,
            width: area.width.saturating_sub(2),
            height: 1,
        };
        frame.render_widget(
            Paragraph::new(Line::from(Span::styled(
                column_title,
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            ))),
            title_area,
        );

        // ── Provider sub-pane ────────────────────────────────────────
        let provider_block = self.focusable_block(" Source ", provider_focused);
        let provider_inner = provider_block.inner(panes[0]);
        frame.render_widget(provider_block, panes[0]);
        let provider_lines: Vec<Line> = if providers.is_empty() {
            vec![Line::from(Span::styled(
                "  (no providers)",
                Style::default().fg(Color::DarkGray),
            ))]
        } else {
            providers
                .iter()
                .enumerate()
                .map(|(i, p)| self.row_provider(i, p, sel_cap, cursor_p, provider_focused))
                .collect()
        };
        frame.render_widget(
            Paragraph::new(provider_lines).wrap(Wrap { trim: false }),
            provider_inner,
        );

        // ── Device sub-pane ──────────────────────────────────────────
        let device_block = self.focusable_block(" Device ", device_focused);
        let device_inner = device_block.inner(panes[1]);
        frame.render_widget(device_block, panes[1]);
        let device_lines: Vec<Line> = if devices.is_empty() {
            vec![Line::from(Span::styled(
                "  (no devices — driver doesn't expose list_devices)",
                Style::default().fg(Color::DarkGray),
            ))]
        } else {
            devices
                .iter()
                .enumerate()
                .map(|(i, d)| self.row_device(i, d, sel_dev, cursor_d, device_focused))
                .collect()
        };
        // Scroll so the cursor stays visible when the list is taller
        // than the pane (the device list on most cards is 20+ entries).
        let scroll = if device_focused {
            let visible = device_inner.height as usize;
            if visible > 0 && cursor_d + 1 > visible {
                (cursor_d + 1 - visible) as u16
            } else {
                0
            }
        } else {
            0
        };
        frame.render_widget(
            Paragraph::new(device_lines)
                .wrap(Wrap { trim: false })
                .scroll((scroll, 0)),
            device_inner,
        );
    }

    /// Common block style — bright cyan when focused, dim gray when not.
    /// Lets the user immediately see which sub-pane keystrokes will hit.
    fn focusable_block(&self, title: &str, focused: bool) -> Block<'_> {
        let style = if focused {
            Style::default()
                .fg(Color::Cyan)
                .add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::DarkGray)
        };
        Block::default()
            .borders(Borders::ALL)
            .border_style(style)
            .title(Line::from(Span::styled(title.to_string(), style)))
    }

    /// One provider row. `●` = currently selected provider, `○` = other.
    /// `▶` marks the cursor row (focused pane only).
    fn row_provider(
        &self,
        i: usize,
        p: &atlas_pb::CapabilityProvider,
        sel: &str,
        cursor: usize,
        focused: bool,
    ) -> Line<'_> {
        let is_cursor = focused && i == cursor;
        let is_selected = p.id == sel;
        let mark = if is_cursor { "▶" } else { " " };
        let bullet = if is_selected { "●" } else { "○" };
        let style = if is_cursor {
            Style::default()
                .fg(Color::Black)
                .bg(Color::Cyan)
                .add_modifier(Modifier::BOLD)
        } else if is_selected {
            Style::default()
                .fg(Color::Green)
                .add_modifier(Modifier::BOLD)
        } else {
            Style::default()
        };
        Line::from(vec![Span::styled(
            format!(" {mark} {bullet} {} ", p.id),
            style,
        )])
    }

    /// One device row. Same conventions as `row_provider`; appends
    /// `[default]` / `[⚠ note]` tags.
    fn row_device(
        &self,
        i: usize,
        d: &crate::pb::audio::AudioDevice,
        sel: &str,
        cursor: usize,
        focused: bool,
    ) -> Line<'_> {
        let is_cursor = focused && i == cursor;
        let is_selected = d.id == sel;
        let mark = if is_cursor { "▶" } else { " " };
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
        let style = if is_cursor {
            Style::default()
                .fg(Color::Black)
                .bg(Color::Cyan)
                .add_modifier(Modifier::BOLD)
        } else if is_selected {
            Style::default()
                .fg(Color::Green)
                .add_modifier(Modifier::BOLD)
        } else {
            Style::default()
        };
        Line::from(vec![Span::styled(
            format!(" {mark} {bullet} {:<14}  {}{}", d.id, d.name, suffix),
            style,
        )])
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

    // Preserve current_controller across the audio settings round-trip;
    // it's owned by the user-modal flow, not by audio settings.
    let new_cfg = ChatConfig {
        mic_cap_id: (!page.mic_cap_id.is_empty()).then_some(page.mic_cap_id),
        mic_device_id: (!page.mic_device_id.is_empty()).then_some(page.mic_device_id),
        speaker_cap_id: (!page.speaker_cap_id.is_empty()).then_some(page.speaker_cap_id),
        speaker_device_id: (!page.speaker_device_id.is_empty()).then_some(page.speaker_device_id),
        current_controller: cfg.current_controller.clone(),
        response_voice: cfg.response_voice,
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
             Enter = send · Ctrl+V = voice · Ctrl+T = settings · Esc = abort · Ctrl+C = quit · ? = help."
        ),
        label: None,
    });
    if let Some(ctl) = chat_cfg.current_controller.as_deref() {
        initial.push(ChatMessage {
            role: Role::Status,
            text: format!("Access control active — only voice from {ctl} will be accepted."),
            label: None,
        });
    }
    for w in audio_warnings {
        initial.push(ChatMessage {
            role: Role::Status,
            text: w.clone(),
            label: None,
        });
    }
    let messages: Rc<RefCell<Vec<ChatMessage>>> = Rc::new(RefCell::new(initial));
    // Local voiceprint cache; refreshed on Ctrl+U open. Used by the voice
    // event renderer to map `voice:<id>` → display name and by the
    // access-control gate when generating denial labels.
    let voiceprint_db: Rc<RefCell<VoiceprintDb>> = Rc::new(RefCell::new(load_voiceprint_db()));
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

            // `?` (unmodified) toggles the help overlay. Render it, then
            // block until any subsequent key dismisses; the next iteration
            // of the loop will overpaint with the normal draw().
            if !busy
                && key.code == KeyCode::Char('?')
                && !key.modifiers.contains(KeyModifiers::CONTROL)
            {
                render_help_overlay(terminal)?;
                // Wait for a single key event then drop it (don't let it
                // act as an input character).
                loop {
                    if event::poll(std::time::Duration::from_millis(120))?
                        && let Event::Key(_) = event::read()?
                    {
                        break;
                    }
                }
                continue;
            }

            // Ctrl+T → Settings (unified). Replaces the old Ctrl+A/U/S
            // sprawl. Ctrl+A/U/S still work as power-user deep-links
            // that jump straight into a specific category.
            //
            // (Originally bound to Ctrl+, but POSIX C0 only encodes
            // Ctrl+A..Ctrl+_ — `,` has no Ctrl variant unless the
            // terminal opts into Kitty keyboard protocol, so most
            // emulators silently swallowed the keystroke. Ctrl+T = 0x14,
            // a real C0 code, works everywhere.)
            let settings_entry: Option<SettingsCategory> =
                if !busy && key.modifiers.contains(KeyModifiers::CONTROL) {
                    match key.code {
                        KeyCode::Char('t') => Some(SettingsCategory::Modes),
                        KeyCode::Char('a') => Some(SettingsCategory::Audio),
                        KeyCode::Char('u') => Some(SettingsCategory::Users),
                        KeyCode::Char('s') => Some(SettingsCategory::System),
                        _ => None,
                    }
                } else {
                    None
                };
            if let Some(entry) = settings_entry {
                let prev_controller = chat_cfg.current_controller.clone();
                let direct_open = !matches!(entry, SettingsCategory::Modes);
                let db_snapshot = voiceprint_db.borrow().clone();
                let initial = if direct_open { Some(entry) } else { None };
                match run_settings_menu(
                    terminal,
                    atlas_endpoint,
                    chat_cfg.clone(),
                    db_snapshot,
                    initial,
                )
                .await
                {
                    Ok(out) => {
                        chat_cfg = out.chat_cfg;
                        *voiceprint_db.borrow_mut() = out.voiceprint_db;
                        if let Err(e) = save_chat_config(&chat_cfg) {
                            messages.borrow_mut().push(ChatMessage {
                                role: Role::Status,
                                text: format!("warning: chat.yaml save failed: {e:#}"),
                                label: None,
                            });
                        }
                        for line in &out.log_lines {
                            messages.borrow_mut().push(ChatMessage {
                                role: Role::Status,
                                text: line.clone(),
                                label: None,
                            });
                        }
                        if prev_controller != chat_cfg.current_controller {
                            match &chat_cfg.current_controller {
                                Some(c) => {
                                    let name = lookup_user_name(&voiceprint_db.borrow(), c)
                                        .unwrap_or_else(|| c.clone());
                                    messages.borrow_mut().push(ChatMessage {
                                        role: Role::Status,
                                        text: format!(
                                            "active user → {} ({}) — non-matching voices will be denied",
                                            name, c
                                        ),
                                        label: None,
                                    });
                                }
                                None => messages.borrow_mut().push(ChatMessage {
                                    role: Role::Status,
                                    text: "active user cleared — any speaker allowed".to_string(),
                                    label: None,
                                }),
                            }
                        }
                    }
                    Err(e) => messages.borrow_mut().push(ChatMessage {
                        role: Role::Status,
                        text: format!("settings: {e:#}"),
                        label: None,
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
                    label: None,
                });
                draw(terminal, &messages.borrow(), &input, scroll, busy)?;
                if let Err(e) = run_voice_session_with_esc_abort(
                    atlas_endpoint,
                    liaison_endpoint,
                    &session_id,
                    &local_user,
                    Rc::clone(&messages),
                    terminal,
                    &input,
                    &mut scroll,
                    &chat_cfg,
                    Rc::clone(&voiceprint_db),
                )
                .await
                {
                    messages.borrow_mut().push(ChatMessage {
                        role: Role::Status,
                        text: format!("Voice error: {e:#}"),
                        label: None,
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
                        label: None,
                    });
                    busy = true;
                    draw(terminal, &messages.borrow(), &input, scroll, busy)?;

                    let modality = if chat_cfg.response_voice {
                        "voice"
                    } else {
                        "text"
                    };
                    match run_text_intent_with_esc_abort(
                        liaison_endpoint,
                        &session_id,
                        &local_user,
                        &msg,
                        modality,
                        Rc::clone(&messages),
                        terminal,
                        &input,
                        &mut scroll,
                    )
                    .await
                    {
                        Ok(reply) => {
                            if chat_cfg.response_voice && !reply.trim().is_empty() {
                                // Fire-and-forget: a synthesis hiccup
                                // should never block the next turn.
                                let ep = atlas_endpoint.to_string();
                                let sp = chat_cfg.speaker_cap_id.clone().unwrap_or_default();
                                tokio::spawn(async move {
                                    let _ = speak_text(&ep, &sp, &reply).await;
                                });
                            }
                        }
                        Err(e) => {
                            messages.borrow_mut().push(ChatMessage {
                                role: Role::Status,
                                text: format!("Error: {e:#}"),
                                label: None,
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
                // Idle Esc: clear the draft input. During a turn,
                // run_text_intent_with_esc_abort handles Esc differently
                // (sends abort_turn). The header line says "Esc = abort
                // turn" which is true mid-turn; idle Esc is a vim-style
                // "clear what I typed" affordance.
                KeyCode::Esc if !input.is_empty() => {
                    input.clear();
                }
                _ => {}
            }
        }
    }
    Ok(())
}

// ── Liaison gRPC helpers ─────────────────────────────────────────────────────

/// `modality` controls pilot's brevity rule (see planner.rs:181 + the
/// liaison voice flow comment at voice.rs:803). Pass "voice" when the
/// final reply will be spoken — pilot then drops markdown and trims to
/// short sentences. Pass "text" for the default chat-window path.
fn build_text_task(
    session_id: &str,
    user_id: &str,
    text: &str,
    modality: &str,
) -> crate::pb::pilot::Task {
    use crate::pb::pilot::Task;
    const INTENT_SOURCE_TEXT: u32 = 0;
    Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: INTENT_SOURCE_TEXT,
        text: text.to_string(),
        audio_data: vec![],
        context_json: serde_json::json!({"user_id": user_id, "modality": modality}).to_string(),
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
    use crate::pb::contracts::robonix_system_liaison_submit_client::RobonixSystemLiaisonSubmitClient;

    let mut client = RobonixSystemLiaisonSubmitClient::connect(liaison_endpoint.to_string())
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
    use crate::pb::contracts::robonix_system_liaison_submit_client::RobonixSystemLiaisonSubmitClient;

    let mut client = RobonixSystemLiaisonSubmitClient::connect(liaison_endpoint.to_string())
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

/// Runs one text-input turn. Returns the agent's concatenated reply
/// text so the caller can decide whether to TTS it (text-in / voice-out
/// mode in chat.yaml). `modality` is forwarded to pilot via context_json
/// so its brevity rule kicks in when the reply will be spoken.
#[allow(clippy::too_many_arguments)]
async fn run_text_intent_with_esc_abort(
    liaison_endpoint: &str,
    session_id: &str,
    user_id: &str,
    user_msg: &str,
    modality: &str,
    messages: Rc<RefCell<Vec<ChatMessage>>>,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    input: &str,
    scroll: &mut u16,
) -> Result<String> {
    use crate::pb::contracts::robonix_system_liaison_submit_client::RobonixSystemLiaisonSubmitClient;
    use crate::pb::pilot::PilotEvent;
    use tonic::Status;

    let (tx, mut rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(64);
    let liaison_ep = liaison_endpoint.to_string();
    let task = build_text_task(session_id, user_id, user_msg, modality);

    let _stream_task = tokio::spawn(async move {
        let mut client = match RobonixSystemLiaisonSubmitClient::connect(liaison_ep.clone()).await {
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

    // Accumulate every TEXT_CHUNK / FINAL_TEXT into a single reply
    // string. Same logic as liaison's voice path so the eventual TTS
    // payload covers BOTH the round-0 chitchat AND the post-tool round-N
    // final reply (the bug we just fixed in voice.rs).
    let mut accumulated_reply = String::new();

    loop {
        tokio::select! {
            biased;
            item = rx.recv() => {
                match item {
                    None => break,
                    Some(Ok(event)) => {
                        match event.event_kind {
                            0 if !event.text_chunk.is_empty() => {
                                append_reply_fragment(&mut accumulated_reply, &event.text_chunk);
                            }
                            4 if !event.final_text.is_empty() => {
                                append_reply_fragment(&mut accumulated_reply, &event.final_text);
                            }
                            _ => {}
                        }
                        apply_pilot_event(&messages, &event)?;
                        draw(terminal, &messages.borrow(), input, 0, true)?;
                    }
                    Some(Err(e)) => {
                        messages.borrow_mut().push(ChatMessage {
                            role: Role::Status,
                            text: format!("Liaison stream error: {e}"),
                        label: None,
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
                                label: None,
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
    Ok(accumulated_reply)
}

/// Same separator-aware appender liaison's voice path uses. Re-declared
/// here so the text-turn accumulator stays self-contained (we don't want
/// to depend on liaison's internals from the cli crate).
fn append_reply_fragment(into: &mut String, fragment: &str) {
    if !into.is_empty() {
        let last = into.chars().last().unwrap_or(' ');
        let needs_sep = !matches!(
            last,
            '。' | '！' | '？' | '.' | '!' | '?' | '\n' | ' ' | ',' | '，'
        );
        if needs_sep {
            into.push(' ');
        }
    }
    into.push_str(fragment);
}

// ── Voice turn ───────────────────────────────────────────────────────────────

#[allow(clippy::too_many_arguments)]
async fn run_voice_session_with_esc_abort(
    atlas_endpoint: &str,
    liaison_endpoint: &str,
    session_id: &str,
    user_id: &str,
    messages: Rc<RefCell<Vec<ChatMessage>>>,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    input: &str,
    scroll: &mut u16,
    chat_cfg: &ChatConfig,
    voiceprint_db: Rc<RefCell<VoiceprintDb>>,
) -> Result<()> {
    use crate::pb::contracts::robonix_system_liaison_voice_client::RobonixSystemLiaisonVoiceClient;
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
        // Pass the access-control pin to liaison so it can hard-gate
        // BEFORE submitting to pilot. The chat side still renders a
        // red DENIED row on mismatch, but the actual blocking has to
        // happen at the liaison so pilot/TTS never fire — earlier
        // chat-side `abort_session()` was too late, by then liaison
        // had already kicked off the pilot stream.
        context_json: if let Some(ctl) = chat_cfg
            .current_controller
            .as_deref()
            .filter(|s| !s.is_empty())
        {
            format!(r#"{{"active_user_id":"{}"}}"#, ctl.replace('"', "\\\""))
        } else {
            String::new()
        },
    };

    let (tx, mut rx) = tokio::sync::mpsc::channel::<Result<VoiceEvent, Status>>(64);
    let liaison_ep = liaison_endpoint.to_string();

    let _stream_task = tokio::spawn(async move {
        let mut client = match RobonixSystemLiaisonVoiceClient::connect(liaison_ep.clone()).await {
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

    let mut last_identified: Option<String> = None;
    // Whether this session has been blocked by the access-control gate
    // (the heard speaker doesn't match the pinned controller). Once set,
    // we swallow downstream PILOT events so the chat doesn't render a
    // partial plan from an aborted-but-still-streaming pilot turn.
    let mut blocked = false;

    loop {
        tokio::select! {
            biased;
            item = rx.recv() => {
                match item {
                    None => break,
                    Some(Ok(event)) => {
                        // Access-control gate: when a controller is pinned in
                        // chat.yaml, every voice turn must come from that
                        // user_id. We catch USER_IDENTIFIED first, abort
                        // upstream, render a denial line, and flag the
                        // session so PILOT events get dropped.
                        if event.event_kind == KIND_USER_IDENTIFIED
                            && let Some(want) = chat_cfg.current_controller.as_deref()
                            && !want.is_empty()
                            && event.user_id != want
                        {
                            blocked = true;
                            let want_name = lookup_user_name(
                                &voiceprint_db.borrow(),
                                want,
                            ).unwrap_or_else(|| want.to_string());
                            // event.user_id starts with "voice:" only when
                            // voiceprint actually matched a known speaker
                            // above the threshold. Otherwise liaison filled
                            // in the chat-cfg fallback (e.g. "local:wheatfox"),
                            // which means "couldn't identify". Use distinct
                            // wording so the operator knows whether the
                            // wrong person spoke vs. nobody got recognised.
                            let identified = event.user_id.starts_with("voice:");
                            let denial_text = if identified {
                                let got_name = lookup_user_name(
                                    &voiceprint_db.borrow(),
                                    &event.user_id,
                                ).unwrap_or_else(|| event.user_id.clone());
                                format!(
                                    "Unauthorized speaker — active user is {} ({}), but heard {} ({})",
                                    want_name, want, got_name, event.user_id
                                )
                            } else {
                                format!(
                                    "Speaker not recognised — active user is {} ({}), voiceprint didn't match anyone",
                                    want_name, want
                                )
                            };
                            messages.borrow_mut().push(ChatMessage {
                                role: Role::Denied,
                                text: denial_text,
                                label: None,
                            });
                            let _ = abort_session(liaison_endpoint, session_id, user_id).await;
                            // Fire a TTS denial so the operator hears why
                            // the car didn't move. Fire-and-forget.
                            let atlas_url = atlas_endpoint.to_string();
                            let speaker_pin = chat_cfg.speaker_cap_id.clone().unwrap_or_default();
                            tokio::spawn(async move {
                                let _ = speak_text(
                                    &atlas_url,
                                    &speaker_pin,
                                    "未授权用户,无法执行任务",
                                ).await;
                            });
                            draw(terminal, &messages.borrow(), input, 0, true)?;
                            continue;
                        }

                        if blocked && event.event_kind == KIND_PILOT {
                            // swallow — already showed denial above.
                            continue;
                        }

                        apply_voice_event(
                            &messages,
                            &event,
                            &voiceprint_db,
                            &mut last_identified,
                        )?;
                        draw(terminal, &messages.borrow(), input, 0, true)?;
                    }
                    Some(Err(e)) => {
                        messages.borrow_mut().push(ChatMessage {
                            role: Role::Status,
                            text: format!("Voice stream error: {e}"),
                        label: None,
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
                                label: None,
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
    const EVT_BATCH_RESULT: u32 = 2;
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
                        label: None,
                    });
                }
            } else {
                m.push(ChatMessage {
                    role: Role::Agent,
                    text: t,
                    label: None,
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
                    label: None,
                });
            }
        }
        EVT_BATCH_RESULT => {
            // Surface sentinel intercepts (and only sentinel — generic
            // capability failures are usually explained by the LLM in the
            // next text chunk, so we don't want to double-render them).
            //
            // The executor encodes the rejection as "sentinel:RULE_ID: REASON"
            // (see system/executor/src/dispatch/mod.rs). Split
            // it back into structured fields so the chat can present a
            // legible block instead of one mashed-together line — the
            // operator needs to see (in order): WHAT was attempted, WHICH
            // rule fired, WHY, and HOW TO REMEDY.
            if let Some(ref br) = event.batch_result {
                for r in &br.results {
                    if !r.success && r.error.starts_with("sentinel:") {
                        let leaf = r
                            .contract_id
                            .rsplit_once('/')
                            .map(|(_, l)| l.to_string())
                            .unwrap_or_else(|| r.contract_id.clone());
                        let target = if r.provider_id.is_empty() {
                            leaf
                        } else {
                            format!("{}.{}", r.provider_id, leaf)
                        };
                        // Error wire format from executor:
                        //   "sentinel:RULE_ID: REASON [deny_window: HH:MM–HH:MM]"
                        // Peel off the bracketed window first (lives at
                        // the tail), then split RULE_ID off REASON at
                        // the first ": ".
                        let body = r.error.trim_start_matches("sentinel:").trim_start();
                        let (head, window) = match body.rfind(" [deny_window:") {
                            Some(idx) => {
                                let win = body[idx + " [deny_window:".len()..]
                                    .trim_end_matches(']')
                                    .trim()
                                    .to_string();
                                (body[..idx].to_string(), Some(win))
                            }
                            None => (body.to_string(), None),
                        };
                        let (rule_id, reason) = head
                            .split_once(':')
                            .map(|(rid, rest)| (rid.trim().to_string(), rest.trim().to_string()))
                            .unwrap_or_else(|| (head.clone(), String::new()));
                        let window_line = match window {
                            Some(w) => format!("\n   window  : {w} (deny_between)"),
                            None => String::new(),
                        };
                        let text = format!(
                            "capability call rejected by sentinel\n   call    : {} (r{})\n   contract: {}\n   rule    : {}\n   reason  : {}{}",
                            target,
                            br.round,
                            r.contract_id,
                            rule_id,
                            if reason.is_empty() {
                                "(no reason provided)"
                            } else {
                                reason.as_str()
                            },
                            window_line,
                        );
                        m.push(ChatMessage {
                            role: Role::Denied,
                            text,
                            label: None,
                        });
                    }
                }
            }
        }
        EVT_PLAN => {
            // dev called this EVT_TASK_GRAPH with `event.task_graph` carrying
            // tool_name + args_json. dev-packaging renamed the message to
            // Plan/CapabilityCall; only contract_id + args_json are exposed,
            // so we leaf-strip contract_id back into a tool-name lookalike to
            // preserve the same `[r{round}] {name}({args})` line shape.
            if let Some(ref p) = event.plan {
                for call in plan_calls(p) {
                    // Display as `provider_id.<leaf>(args)` so the user
                    // can see WHICH provider's capability is being
                    // invoked — `snapshot({})` alone is ambiguous when
                    // multiple providers offer the same contract leaf.
                    let leaf = call
                        .contract_id
                        .rsplit_once('/')
                        .map(|(_, l)| l.to_string())
                        .unwrap_or_else(|| call.contract_id.clone());
                    let target = if call.provider_id.is_empty() {
                        leaf
                    } else {
                        format!("{}.{}", call.provider_id, leaf)
                    };
                    m.push(ChatMessage {
                        role: Role::ToolCall,
                        text: format!("[r{}] {}({})", p.round, target, call.args_json),
                        label: None,
                    });
                }
            }
        }
        _ => {}
    }
    Ok(())
}

fn plan_calls(plan: &crate::pb::pilot::Plan) -> Vec<&crate::pb::pilot::CapabilityCall> {
    plan.nodes
        .iter()
        .filter_map(|node| node.call.as_ref())
        .collect()
}

// VoiceEvent kinds — exposed here so the access-control gate in
// run_voice_session_with_esc_abort can match on the same constants
// without re-declaring them. Mirrors voice.rs / VoiceEvent.msg.
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

fn apply_voice_event(
    messages: &Rc<RefCell<Vec<ChatMessage>>>,
    event: &crate::pb::liaison::VoiceEvent,
    voiceprint_db: &Rc<RefCell<VoiceprintDb>>,
    last_identified: &mut Option<String>,
) -> Result<()> {
    match event.event_kind {
        KIND_SESSION_STARTED | KIND_RECORDING_STARTED | KIND_RECORDING_DONE => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("voice · {}", event.status_message),
                label: None,
            });
        }
        KIND_ASR_PARTIAL => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("asr (partial, {:.2}): {}", event.confidence, event.text),
                label: None,
            });
        }
        KIND_ASR_FINAL => {
            // Liaison guarantees USER_IDENTIFIED arrives before ASR_FINAL,
            // so `last_identified` is the right uid for this utterance.
            // Promote a real voiceprint match (uid starts with "voice:")
            // to the ROW label so the chat shows "Alice  (voice) …"
            // instead of the generic "You". Fallback ids (local:*)
            // leave label=None so draw uses "You".
            let author_label = match last_identified.as_deref() {
                Some(uid) if uid.starts_with("voice:") => Some(
                    lookup_user_name(&voiceprint_db.borrow(), uid)
                        .unwrap_or_else(|| uid.to_string()),
                ),
                _ => None,
            };
            messages.borrow_mut().push(ChatMessage {
                role: Role::User,
                text: format!("(voice) {}", event.text),
                label: author_label,
            });
        }
        KIND_USER_IDENTIFIED => {
            *last_identified = Some(event.user_id.clone());
            // Liaison emits ASR_FINAL BEFORE this event (the comment we
            // had above was wrong), so when this arrives the user's row
            // has already been pushed with label=None and is rendering
            // as "You". Back-patch it: scan back from the tail for the
            // most-recent User row that still has label=None and stamp
            // the voiceprint-matched display name on it. Skip if no
            // voiceprint match (event.user_id is fallback like local:*).
            if event.user_id.starts_with("voice:") {
                let name_opt = lookup_user_name(&voiceprint_db.borrow(), &event.user_id);
                let display = name_opt.unwrap_or_else(|| event.user_id.clone());
                let mut m = messages.borrow_mut();
                if let Some(target) = m
                    .iter_mut()
                    .rev()
                    .find(|x| matches!(x.role, Role::User) && x.label.is_none())
                {
                    target.label = Some(display);
                }
            }
            let name = lookup_user_name(&voiceprint_db.borrow(), &event.user_id);
            let label = match (&name, event.status_message.is_empty()) {
                (Some(n), true) => format!("identified user → {} ({})", n, event.user_id),
                (Some(n), false) => format!(
                    "identified user → {} ({}) · {}",
                    n, event.user_id, event.status_message
                ),
                (None, true) => format!("identified user → {}", event.user_id),
                (None, false) => format!(
                    "identified user → {} · {}",
                    event.user_id, event.status_message
                ),
            };
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: label,
                label: None,
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
                label: None,
            });
        }
        KIND_TTS_DONE => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("tts done · {}", event.status_message),
                label: None,
            });
        }
        KIND_SESSION_DONE => {
            *last_identified = None;
            messages.borrow_mut().push(ChatMessage {
                role: Role::Status,
                text: "voice session done".to_string(),
                label: None,
            });
        }
        KIND_ERROR => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Status,
                text: format!("voice error: {}", event.error),
                label: None,
            });
        }
        _ => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("voice (kind={}) {}", event.event_kind, event.status_message),
                label: None,
            });
        }
    }
    Ok(())
}

/// Layout invariants:
///   - `Min(3)` chat history pane (block with header line in title bar,
///     spinner / scroll indicator in title_bottom)
///   - `Length(3)` input pane (block, title carries the current mode marker)
///   - `Length(1)` footer hint line (no border, dim, single line of keymap)
///
/// Visual conventions ([gfargo/tui-design-skill] + lazygit/k9s patterns):
///   - Per-turn separator drawn whenever the next message changes role
///     to `User` after an Agent/Tool reply — turns into visual blocks.
///   - `Role::Voice` partial events (asr partials, tts progress) are
///     elided from the scrolling history and instead summarised in the
///     title_bottom of the chat block as `transient status`. Without
///     this, a single voice turn produces ~10 lines of `[v] ...` noise.
///   - Spinner is the 10-frame braille `⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏` set, advanced once
///     per draw via a `static AtomicU64`. Only rendered while `busy`.
///   - Footer hint line is plain text, `Color::DarkGray`, no border.
///     Always-visible discoverability beats stashing keys in titles.
///
/// We intentionally keep the draw() signature unchanged so the 13
/// existing call sites (mostly inside voice/text turn helpers) don't
/// need to be updated. All new state (spinner counter, time-of-day,
/// last voice transient) is derived inside draw() itself.
fn draw(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    messages: &[ChatMessage],
    input: &str,
    scroll: u16,
    busy: bool,
) -> Result<()> {
    static SPINNER_COUNTER: AtomicU64 = AtomicU64::new(0);
    const SPINNER_FRAMES: &[char] = &['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧', '⠇', '⠏'];
    let spin = SPINNER_FRAMES
        [(SPINNER_COUNTER.fetch_add(1, Ordering::Relaxed) as usize) % SPINNER_FRAMES.len()];

    // The most recent Voice event is the one worth surfacing as the
    // live status footer; older voice traffic isn't useful once a new
    // chunk has arrived. Walk back-to-front so the scan stops early.
    let latest_voice: Option<&str> = messages
        .iter()
        .rev()
        .find(|m| matches!(m.role, Role::Voice))
        .map(|m| m.text.as_str());

    terminal.draw(|f| {
        let area = f.area();
        let chunks = Layout::vertical([
            Constraint::Min(3),
            Constraint::Length(3),
            Constraint::Length(1),
        ])
        .split(area);

        // ── History pane ──────────────────────────────────────────────────
        let mut lines: Vec<Line> = Vec::new();
        let mut prev_role: Option<&Role> = None;
        for msg in messages {
            // Elide Voice partials/progress from the scrolling history —
            // they live in the title_bottom transient instead.
            if matches!(msg.role, Role::Voice) {
                continue;
            }
            // Visual separator between turns: draw a thin rule when the
            // next message kicks off a new User turn (i.e. previous was
            // Agent/Tool reply). Don't insert between Status lines or at
            // top of buffer.
            if matches!(msg.role, Role::User)
                && matches!(prev_role, Some(Role::Agent) | Some(Role::ToolCall))
            {
                lines.push(Line::from(Span::styled(
                    "─".repeat(8),
                    Style::default().fg(Color::DarkGray),
                )));
            }
            // Voice-identified turns carry a per-message label override
            // (the speaker's display name). Pad to the 9-char column so
            // bodies still align with rows that fall back to "You".
            let user_label: String = if let Some(ref n) = msg.label {
                if n.chars().count() >= 9 {
                    let mut t: String = n.chars().take(8).collect();
                    t.push(' ');
                    t
                } else {
                    let pad = 9 - n.chars().count();
                    format!("{}{}", n, " ".repeat(pad))
                }
            } else {
                "You      ".to_string()
            };
            let (label, indent, style): (&str, &str, Style) = match msg.role {
                Role::User => (
                    &user_label,
                    "         ",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ),
                Role::Agent => ("Robonix  ", "         ", Style::default().fg(Color::Green)),
                // Capability invocation from pilot's plan. The body already
                // shows `<leaf>(<args>)`, so no noun prefix — MCP's "tool" /
                // "tool_call" wording is the LLM-side framing and would
                // conflict with CLAUDE.md's concept-stability rule
                // (capability / contract / primitive / service / skill).
                Role::ToolCall => ("⚙        ", "         ", Style::default().fg(Color::Yellow)),
                Role::Status => (
                    "·        ",
                    "         ",
                    Style::default()
                        .fg(Color::Magenta)
                        .add_modifier(Modifier::ITALIC),
                ),
                Role::Denied => (
                    "⛔ DENIED ",
                    "         ",
                    Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
                ),
                // Unreachable — Voice was filtered above. Keep the arm
                // exhaustive so adding a new Role doesn't silently miss.
                Role::Voice => continue,
            };
            for (i, text_line) in msg.text.lines().enumerate() {
                let lead = if i == 0 { label } else { indent };
                lines.push(Line::from(vec![
                    Span::styled(lead.to_string(), style),
                    Span::styled(text_line.to_string(), style),
                ]));
            }
            prev_role = Some(&msg.role);
        }

        // Header in the block title — always-visible session context.
        let mode_marker = "text";
        let hhmm = current_time_hhmm();
        let title = Line::from(vec![
            Span::raw(" "),
            Span::styled(
                "Robonix",
                Style::default()
                    .fg(Color::Green)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw(" · "),
            Span::styled(mode_marker, Style::default().fg(Color::Cyan)),
            Span::raw(" · "),
            Span::styled(hhmm, Style::default().fg(Color::DarkGray)),
            Span::raw(" "),
        ]);

        // Bottom of block: spinner + scroll indicator + last-voice
        // transient. Right-aligned so it sits near the bottom-right corner.
        let scroll_label = if scroll == 0 {
            "scroll: bot"
        } else {
            "scroll: ↑"
        };
        let mut bottom_spans: Vec<Span> = Vec::new();
        if busy {
            bottom_spans.push(Span::styled(
                format!(" {spin} thinking "),
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            ));
            bottom_spans.push(Span::raw("· "));
        }
        if let Some(v) = latest_voice {
            // truncate to keep the bottom row tidy
            let one_line: String = v
                .lines()
                .next()
                .unwrap_or("")
                .chars()
                .take(48)
                .collect::<String>();
            bottom_spans.push(Span::styled(
                format!("🎤 {one_line} "),
                Style::default().fg(Color::Blue),
            ));
            bottom_spans.push(Span::raw("· "));
        }
        bottom_spans.push(Span::styled(
            format!(" {scroll_label} "),
            Style::default().fg(Color::DarkGray),
        ));
        let bottom = Line::from(bottom_spans).alignment(Alignment::Right);

        let block = Block::default()
            .borders(Borders::ALL)
            .title(title)
            .title_bottom(bottom);
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

        // ── Input pane ────────────────────────────────────────────────────
        let input_title = if busy {
            Line::from(vec![
                Span::raw(" "),
                Span::styled(spin.to_string(), Style::default().fg(Color::Yellow)),
                Span::raw(" "),
            ])
        } else {
            Line::from(Span::styled(
                " > ",
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            ))
        };
        let input_widget = Paragraph::new(input.to_string())
            .block(Block::default().borders(Borders::ALL).title(input_title));
        f.render_widget(input_widget, chunks[1]);

        // ── Footer hint bar ──────────────────────────────────────────────
        // Persistent, single-line, no border. Mirrors lazygit/k9s
        // discoverability convention — all top-level keys here at all
        // times, no need to hunt through menus.
        let footer = Paragraph::new(Line::from(vec![
            Span::styled(" Enter", Style::default().fg(Color::White)),
            Span::styled(" send  · ", Style::default().fg(Color::DarkGray)),
            Span::styled("Ctrl+V", Style::default().fg(Color::White)),
            Span::styled(" voice · ", Style::default().fg(Color::DarkGray)),
            Span::styled("Ctrl+U", Style::default().fg(Color::White)),
            Span::styled(" users · ", Style::default().fg(Color::DarkGray)),
            Span::styled("Ctrl+A", Style::default().fg(Color::White)),
            Span::styled(" audio · ", Style::default().fg(Color::DarkGray)),
            Span::styled("?", Style::default().fg(Color::White)),
            Span::styled(" help  · ", Style::default().fg(Color::DarkGray)),
            Span::styled("Ctrl+C", Style::default().fg(Color::White)),
            Span::styled(" quit", Style::default().fg(Color::DarkGray)),
        ]));
        f.render_widget(footer, chunks[2]);
    })?;
    Ok(())
}

/// `HH:MM` of system local time. Used in the chat header.
fn current_time_hhmm() -> String {
    chrono::Local::now().format("%H:%M").to_string()
}

/// Centered Rect for popup overlays — used by `?` help and the Ctrl+U
/// users modal. Standard ratatui recipe.
fn centered_rect(percent_x: u16, percent_y: u16, area: Rect) -> Rect {
    let popup_layout = Layout::vertical([
        Constraint::Percentage((100 - percent_y) / 2),
        Constraint::Percentage(percent_y),
        Constraint::Percentage((100 - percent_y) / 2),
    ])
    .split(area);
    Layout::horizontal([
        Constraint::Percentage((100 - percent_x) / 2),
        Constraint::Percentage(percent_x),
        Constraint::Percentage((100 - percent_x) / 2),
    ])
    .split(popup_layout[1])[1]
}

/// Draws the `?` help overlay on top of the existing terminal contents
/// (history, input, footer all stay visible underneath). Caller is
/// responsible for blocking the main key loop until any key is received,
/// at which point the next `draw()` will overpaint without this overlay.
fn render_help_overlay(terminal: &mut Terminal<CrosstermBackend<io::Stdout>>) -> Result<()> {
    terminal.draw(|f| {
        let area = centered_rect(60, 60, f.area());
        f.render_widget(Clear, area);
        let block = Block::default()
            .borders(Borders::ALL)
            .title(Line::from(vec![Span::styled(
                " Help · Key bindings ",
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            )]))
            .title_bottom(
                Line::from(Span::styled(
                    " any key to close ",
                    Style::default().fg(Color::DarkGray),
                ))
                .alignment(Alignment::Right),
            );
        let rows = vec![
            ("Global", ""),
            ("  Enter", "Send the current input"),
            ("  Ctrl+V", "Hold to talk (auto-ends on silence)"),
            ("  Ctrl+T", "Open Settings (Modes / Audio / Users / System)"),
            ("", ""),
            ("Settings deep-links", ""),
            ("  Ctrl+A", "Settings → Audio"),
            ("  Ctrl+U", "Settings → Users"),
            ("  Ctrl+S", "Settings → System (status / logs / perf)"),
            ("", ""),
            ("Other", ""),
            ("  ?", "Show this help"),
            ("  Ctrl+C", "Quit"),
            ("", ""),
            ("During a turn", ""),
            ("  Esc", "Abort the current turn"),
            ("  PageUp / PageDown", "Scroll history"),
            ("", ""),
            ("Idle", ""),
            ("  Esc", "Clear the draft input"),
        ];
        let lines: Vec<Line> = rows
            .into_iter()
            .map(|(k, v)| {
                if k.is_empty() && v.is_empty() {
                    Line::from("")
                } else if v.is_empty() {
                    Line::from(Span::styled(
                        k.to_string(),
                        Style::default()
                            .fg(Color::Yellow)
                            .add_modifier(Modifier::BOLD),
                    ))
                } else {
                    Line::from(vec![
                        Span::styled(
                            format!("{:<22}", k),
                            Style::default()
                                .fg(Color::White)
                                .add_modifier(Modifier::BOLD),
                        ),
                        Span::styled(v.to_string(), Style::default().fg(Color::Gray)),
                    ])
                }
            })
            .collect();
        let body = Paragraph::new(lines)
            .block(block)
            .wrap(Wrap { trim: false });
        f.render_widget(body, area);
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

// ─────────────────────────────────────────────────────────────────────────────
// Phase 1B — voiceprint Ctrl+U modal + access control
// ─────────────────────────────────────────────────────────────────────────────
//
// The chat client lets the operator enroll a voice and pin a "controller"
// (the user_id whose voice may issue commands). When a controller is set,
// any voice turn whose identified speaker doesn't match is rejected at the
// client — the in-flight pilot stream is aborted and a denial line shows
// up in chat. Without a controller, any speaker is allowed (current
// behaviour pre Phase 1B).
//
// Modal states (Ctrl+U opens at List):
//   List       — table of enrolled users + current controller, key cmds
//                ↑↓ move · Enter set as controller · c clear controller
//                · n enroll new · r refresh · Esc close
//   Naming     — input field for the new user's display name (user_id
//                derived as `voice:<slug(name)>`). Enter starts Recording.
//   Recording  — 5s mic capture from chat_cfg.mic_cap_id. Progress bar.
//   Uploading  — Enroll RPC running.
//   Notice     — info screen ("✓ enrolled" / "✗ error: ..."). Any key
//                returns to List.
//
// Voice prompts via TTS+speaker are fired around the Recording transition
// so the operator hears "please speak now" before recording and "registered
// <name>" on success. They go through the same atlas-resolved TTS + speaker
// providers liaison uses for voice replies.

use tokio_stream::iter as stream_iter;
use tonic::Request;

use crate::pb::audio::AudioChunk;
use crate::pb::contracts::{
    robonix_primitive_audio_mic_client::RobonixPrimitiveAudioMicClient,
    robonix_primitive_audio_speaker_client::RobonixPrimitiveAudioSpeakerClient,
    robonix_system_speech_tts_client::RobonixSystemSpeechTtsClient,
    robonix_system_speech_voiceprint_delete_client::RobonixSystemSpeechVoiceprintDeleteClient,
    robonix_system_speech_voiceprint_enroll_client::RobonixSystemSpeechVoiceprintEnrollClient,
    robonix_system_speech_voiceprint_list_client::RobonixSystemSpeechVoiceprintListClient,
};
use crate::pb::tts as pb_tts;
use crate::pb::voiceprint as pb_voiceprint;

const MIC_RECORD_SECS: u64 = 8;
const MIC_SAMPLE_RATE_HZ: u32 = 16_000;

const VOICEPRINT_ENROLL_CONTRACT: &str = "robonix/system/speech/voiceprint_enroll";
const VOICEPRINT_LIST_CONTRACT: &str = "robonix/system/speech/voiceprint_list";
const VOICEPRINT_DELETE_CONTRACT: &str = "robonix/system/speech/voiceprint_delete";
const TTS_CONTRACT: &str = "robonix/system/speech/tts";

#[derive(Clone, Default)]
struct UserEntry {
    user_id: String,
    user_name: String,
}

enum UsersModalState {
    List,
    Naming {
        input: String,
    },
    Recording {
        started: std::time::Instant,
        audio: Vec<u8>,
        display_name: String,
        user_id: String,
    },
    Uploading {
        display_name: String,
    },
    Notice {
        text: String,
        ok: bool,
    },
}

struct UsersModal {
    state: UsersModalState,
    users: Vec<UserEntry>,
    cursor: usize,
    controller: Option<String>,
    status: String,
}

/// Same two-step Atlas resolution liaison uses: query providers for the
/// contract, optionally honour a pinned provider_id, then ConnectCapability
/// to actually receive the endpoint string. Returns `None` if no live
/// provider is available.
async fn resolve_grpc_endpoint(
    atlas: &mut AtlasClient,
    contract_id: &str,
    pin_provider_id: &str,
) -> Option<String> {
    let providers = atlas
        .query_capabilities("", contract_id, atlas_pb::Transport::Grpc)
        .await
        .ok()?;
    let pick = if pin_provider_id.is_empty() {
        providers.first()
    } else {
        providers
            .iter()
            .find(|r| r.id == pin_provider_id || r.namespace == pin_provider_id)
            .or_else(|| providers.first())
    };
    let provider = pick?;
    let (_channel_id, endpoint, _params) = atlas
        .connect_capability(
            CONSUMER_ID,
            &provider.id,
            contract_id,
            atlas_pb::Transport::Grpc,
        )
        .await
        .ok()?;
    if endpoint.is_empty() {
        return None;
    }
    let with_scheme = if endpoint.starts_with("http") {
        endpoint
    } else {
        format!("http://{endpoint}")
    };
    Some(localhost_to_ipv4_loopback(&with_scheme))
}

/// List enrolled voiceprint users via the service's ListEnrolled RPC.
/// Returns an empty vec if the service isn't reachable; callers display
/// the error inline rather than crashing the modal.
async fn voiceprint_list_users(atlas_endpoint: &str) -> Result<Vec<UserEntry>> {
    let mut atlas = AtlasClient::connect(atlas_endpoint.to_string())
        .await
        .context("connect atlas for voiceprint list")?;
    let endpoint = resolve_grpc_endpoint(&mut atlas, VOICEPRINT_LIST_CONTRACT, "")
        .await
        .ok_or_else(|| anyhow::anyhow!("no voiceprint_list provider in atlas"))?;
    let mut client = RobonixSystemSpeechVoiceprintListClient::connect(endpoint.clone())
        .await
        .with_context(|| format!("dial voiceprint_list at {endpoint}"))?;
    let resp = client
        .list_enrolled(Request::new(pb_voiceprint::ListEnrolledRequest {}))
        .await
        .context("ListEnrolled rpc")?
        .into_inner();
    if !resp.error.is_empty() {
        anyhow::bail!("voiceprint service error: {}", resp.error);
    }
    // users_json: [{"user_id":"...","user_name":"..."}, ...]
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&resp.users_json)
        .with_context(|| format!("parse users_json: {}", resp.users_json))?;
    let users = parsed
        .into_iter()
        .map(|v| UserEntry {
            user_id: v
                .get("user_id")
                .and_then(|x| x.as_str())
                .unwrap_or("")
                .to_string(),
            user_name: v
                .get("user_name")
                .and_then(|x| x.as_str())
                .unwrap_or("")
                .to_string(),
        })
        .filter(|u| !u.user_id.is_empty())
        .collect();
    Ok(users)
}

/// Enroll one user — sends raw PCM + (user_id, user_name) to the voiceprint
/// service. Returns Ok(()) on success and propagates the service's error
/// string otherwise.
async fn voiceprint_enroll(
    atlas_endpoint: &str,
    user_id: &str,
    user_name: &str,
    audio_pcm: Vec<u8>,
) -> Result<()> {
    let mut atlas = AtlasClient::connect(atlas_endpoint.to_string())
        .await
        .context("connect atlas for voiceprint enroll")?;
    let endpoint = resolve_grpc_endpoint(&mut atlas, VOICEPRINT_ENROLL_CONTRACT, "")
        .await
        .ok_or_else(|| anyhow::anyhow!("no voiceprint_enroll provider in atlas"))?;
    let mut client = RobonixSystemSpeechVoiceprintEnrollClient::connect(endpoint.clone())
        .await
        .with_context(|| format!("dial voiceprint_enroll at {endpoint}"))?;
    let resp = client
        .enroll(Request::new(pb_voiceprint::EnrollRequest {
            user_id: user_id.to_string(),
            user_name: user_name.to_string(),
            audio_data: audio_pcm,
            encoding: "pcm_s16le".to_string(),
            sample_rate_hz: MIC_SAMPLE_RATE_HZ,
        }))
        .await
        .context("Enroll rpc")?
        .into_inner();
    if !resp.success {
        anyhow::bail!(
            "voiceprint service refused enrollment: {}",
            if resp.error.is_empty() {
                "unknown error".to_string()
            } else {
                resp.error
            }
        );
    }
    Ok(())
}

/// Drop one enrolled voiceprint from the service. Idempotent on the
/// service side (deleting an absent user_id returns success). Used by
/// the Users modal's `d` key.
async fn voiceprint_delete(atlas_endpoint: &str, user_id: &str) -> Result<()> {
    let mut atlas = AtlasClient::connect(atlas_endpoint.to_string())
        .await
        .context("connect atlas for voiceprint delete")?;
    let endpoint = resolve_grpc_endpoint(&mut atlas, VOICEPRINT_DELETE_CONTRACT, "")
        .await
        .ok_or_else(|| anyhow::anyhow!("no voiceprint_delete provider in atlas"))?;
    let mut client = RobonixSystemSpeechVoiceprintDeleteClient::connect(endpoint.clone())
        .await
        .with_context(|| format!("dial voiceprint_delete at {endpoint}"))?;
    let resp = client
        .delete_enrolled(Request::new(pb_voiceprint::DeleteEnrolledRequest {
            user_id: user_id.to_string(),
        }))
        .await
        .context("DeleteEnrolled rpc")?
        .into_inner();
    if !resp.success {
        anyhow::bail!(
            "voiceprint service refused delete: {}",
            if resp.error.is_empty() {
                "unknown error".to_string()
            } else {
                resp.error
            }
        );
    }
    Ok(())
}

/// Pull `MIC_RECORD_SECS` of raw PCM off the configured mic primitive. Mic
/// chunks arrive as 100ms s16le frames; we accumulate `.data` until the
/// deadline, then drop the stream. Returns the raw byte buffer suitable
/// for handing to voiceprint enrollment.
async fn capture_mic_pcm(atlas_endpoint: &str, mic_pin: &str) -> Result<Vec<u8>> {
    let mut atlas = AtlasClient::connect(atlas_endpoint.to_string())
        .await
        .context("connect atlas for mic capture")?;
    let endpoint = resolve_grpc_endpoint(&mut atlas, MIC_CONTRACT, mic_pin)
        .await
        .ok_or_else(|| anyhow::anyhow!("no audio mic provider in atlas"))?;
    let mut client = RobonixPrimitiveAudioMicClient::connect(endpoint.clone())
        .await
        .with_context(|| format!("dial mic at {endpoint}"))?;
    let mut stream = client
        .mic(Request::new(()))
        .await
        .context("mic rpc")?
        .into_inner();
    let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(MIC_RECORD_SECS);
    let mut buf: Vec<u8> =
        Vec::with_capacity((MIC_SAMPLE_RATE_HZ as usize) * 2 * (MIC_RECORD_SECS as usize));
    loop {
        let now = tokio::time::Instant::now();
        if now >= deadline {
            break;
        }
        let remaining = deadline - now;
        match tokio::time::timeout(remaining, stream.message()).await {
            Ok(Ok(Some(chunk))) => buf.extend_from_slice(&chunk.data),
            Ok(Ok(None)) => break,
            Ok(Err(e)) => anyhow::bail!("mic stream error: {e}"),
            Err(_) => break,
        }
    }
    if buf.is_empty() {
        anyhow::bail!("mic returned 0 bytes of audio");
    }
    Ok(buf)
}

/// Synthesize `text` and play it on the chat-configured speaker. Mirrors
/// `liaison::voice::synthesize_and_play` but with `eprintln!` swallowed and
/// no VoiceEvent emissions — this is fire-and-forget feedback for the
/// modal, the user already sees the visual prompt.
async fn speak_text(atlas_endpoint: &str, speaker_pin: &str, text: &str) -> Result<()> {
    let mut atlas = AtlasClient::connect(atlas_endpoint.to_string())
        .await
        .context("connect atlas for speak")?;
    let tts_endpoint = resolve_grpc_endpoint(&mut atlas, TTS_CONTRACT, "")
        .await
        .ok_or_else(|| anyhow::anyhow!("no tts provider in atlas"))?;
    let speaker_endpoint = resolve_grpc_endpoint(&mut atlas, SPEAKER_CONTRACT, speaker_pin)
        .await
        .ok_or_else(|| anyhow::anyhow!("no speaker provider in atlas"))?;
    let mut tts = RobonixSystemSpeechTtsClient::connect(tts_endpoint.clone())
        .await
        .with_context(|| format!("dial tts at {tts_endpoint}"))?;
    let resp = tts
        .synthesize(Request::new(pb_tts::SynthesizeRequest {
            text: text.to_string(),
            language: String::new(),
            voice: String::new(),
            speed: 1.0,
        }))
        .await
        .context("tts rpc")?
        .into_inner();
    if !resp.error.is_empty() {
        anyhow::bail!("tts error: {}", resp.error);
    }
    if resp.audio_data.is_empty() {
        anyhow::bail!("tts returned empty audio");
    }
    let mut speaker = RobonixPrimitiveAudioSpeakerClient::connect(speaker_endpoint.clone())
        .await
        .with_context(|| format!("dial speaker at {speaker_endpoint}"))?;
    const SLICE: usize = 8 * 1024;
    let chunks: Vec<AudioChunk> = resp
        .audio_data
        .chunks(SLICE)
        .enumerate()
        .map(|(i, slice)| AudioChunk {
            timestamp_ns: 0,
            data: slice.to_vec(),
            sequence: i as u32,
            duration_s: 0.0,
        })
        .collect();
    speaker
        .speaker(Request::new(stream_iter(chunks)))
        .await
        .context("speaker rpc")?;
    Ok(())
}

/// Sanitise a free-form display name into a stable user_id suffix.
/// Strips/replaces anything that isn't ASCII alphanumeric. Used to build
/// `voice:<slug>` identifiers that match what the voiceprint service +
/// liaison's `event_user(user_id=...)` produces.
fn slug_for_user_id(name: &str) -> String {
    let mut s = String::with_capacity(name.len());
    for ch in name.chars() {
        if ch.is_ascii_alphanumeric() {
            s.push(ch.to_ascii_lowercase());
        } else if ch.is_whitespace() || ch == '-' || ch == '_' {
            s.push('_');
        }
    }
    let trimmed = s.trim_matches('_').to_string();
    if trimmed.is_empty() {
        format!("user_{}", now_ms())
    } else {
        trimmed
    }
}

/// Look up a user_id (e.g. "voice:alice") in the local cache and return
/// its display name. `None` if unknown — the caller decides whether to
/// render "(unknown)" or omit the suffix.
fn lookup_user_name(db: &VoiceprintDb, user_id: &str) -> Option<String> {
    db.users.get(user_id).map(|u| u.name.clone())
}

/// Refresh the local voiceprint cache from the service. Returns the new
/// db; callers replace their cell. Errors propagate so the modal can show
/// "(offline)" instead of stale data.
async fn refresh_voiceprint_db(atlas_endpoint: &str) -> Result<VoiceprintDb> {
    let users = voiceprint_list_users(atlas_endpoint).await?;
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let mut db = VoiceprintDb::default();
    for u in &users {
        db.users.insert(
            u.user_id.clone(),
            VoiceprintUser {
                name: u.user_name.clone(),
                enrolled_at: now,
            },
        );
    }
    let _ = save_voiceprint_db(&db);
    Ok(db)
}

/// Draws the Ctrl+U users modal as a centered popup on top of the chat
/// view. The chat under it stays painted (the main `draw()` already ran
/// this frame). Clear+Block follows ratatui's standard overlay recipe.
fn draw_users_modal(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    modal: &UsersModal,
) -> Result<()> {
    terminal.draw(|f| {
        let area = centered_rect(70, 70, f.area());
        f.render_widget(Clear, area);
        let title = match &modal.state {
            UsersModalState::List => " Users · voiceprint ",
            UsersModalState::Naming { .. } => " Users · new enrollment ",
            UsersModalState::Recording { .. } => " Users · recording ",
            UsersModalState::Uploading { .. } => " Users · uploading ",
            UsersModalState::Notice { .. } => " Users · notice ",
        };
        let footer = match &modal.state {
            UsersModalState::List => {
                " ↑↓ move · Enter set active · c clear · n new · d delete · r refresh · Esc close "
            }
            UsersModalState::Naming { .. } => " type name · Enter record · Esc back ",
            UsersModalState::Recording { .. } => " recording — please speak ",
            UsersModalState::Uploading { .. } => " uploading… ",
            UsersModalState::Notice { .. } => " any key to continue ",
        };
        let block = Block::default()
            .borders(Borders::ALL)
            .title(Line::from(Span::styled(
                title,
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            )))
            .title_bottom(
                Line::from(Span::styled(footer, Style::default().fg(Color::DarkGray)))
                    .alignment(Alignment::Right),
            );

        let inner = block.inner(area);
        f.render_widget(block, area);

        let header_line = {
            let ctl = modal
                .controller
                .as_deref()
                .unwrap_or("(any speaker allowed)");
            Line::from(vec![
                Span::styled("active user: ", Style::default().fg(Color::Gray)),
                Span::styled(
                    ctl.to_string(),
                    Style::default()
                        .fg(Color::Yellow)
                        .add_modifier(Modifier::BOLD),
                ),
            ])
        };

        let status_line = if modal.status.is_empty() {
            Line::from("")
        } else {
            Line::from(Span::styled(
                modal.status.clone(),
                Style::default().fg(Color::DarkGray),
            ))
        };

        let mut body_lines: Vec<Line> = vec![header_line, status_line, Line::from("")];

        match &modal.state {
            UsersModalState::List => {
                if modal.users.is_empty() {
                    body_lines.push(Line::from(Span::styled(
                        "no users enrolled yet — press `n` to enroll one",
                        Style::default().fg(Color::DarkGray),
                    )));
                } else {
                    body_lines.push(Line::from(vec![Span::styled(
                        format!("{:<3}{:<28}{}", "", "user_id", "name"),
                        Style::default()
                            .fg(Color::White)
                            .add_modifier(Modifier::BOLD),
                    )]));
                    for (i, u) in modal.users.iter().enumerate() {
                        let marker = if i == modal.cursor { "▶" } else { " " };
                        let is_ctl = modal.controller.as_deref() == Some(u.user_id.as_str());
                        let style = if i == modal.cursor {
                            Style::default()
                                .fg(Color::Black)
                                .bg(Color::Cyan)
                                .add_modifier(Modifier::BOLD)
                        } else if is_ctl {
                            Style::default().fg(Color::Yellow)
                        } else {
                            Style::default().fg(Color::White)
                        };
                        let suffix = if is_ctl { "  ← controller" } else { "" };
                        body_lines.push(Line::from(Span::styled(
                            format!(
                                "{} {} {:<28}{}{}",
                                marker, " ", u.user_id, u.user_name, suffix
                            ),
                            style,
                        )));
                    }
                }
            }
            UsersModalState::Naming { input } => {
                body_lines.push(Line::from(Span::styled(
                    "Enter the new user's display name. We'll capture 5 seconds of voice.",
                    Style::default().fg(Color::Gray),
                )));
                body_lines.push(Line::from(""));
                body_lines.push(Line::from(vec![
                    Span::styled(
                        "name: ",
                        Style::default()
                            .fg(Color::White)
                            .add_modifier(Modifier::BOLD),
                    ),
                    Span::styled(input.clone(), Style::default().fg(Color::Cyan)),
                    Span::styled("█", Style::default().fg(Color::Cyan)),
                ]));
                let slug = slug_for_user_id(input);
                body_lines.push(Line::from(Span::styled(
                    format!("user_id: voice:{}", slug),
                    Style::default().fg(Color::DarkGray),
                )));
            }
            UsersModalState::Recording {
                started,
                display_name,
                ..
            } => {
                let elapsed = started.elapsed().as_secs_f32().min(MIC_RECORD_SECS as f32);
                let total = MIC_RECORD_SECS as f32;
                let pct = (elapsed / total).clamp(0.0, 1.0);
                let bar_w = 40usize;
                let filled = (pct * bar_w as f32) as usize;
                let bar: String = std::iter::repeat_n('█', filled)
                    .chain(std::iter::repeat_n('░', bar_w - filled))
                    .collect();
                body_lines.push(Line::from(Span::styled(
                    format!("Recording for {}…", display_name),
                    Style::default()
                        .fg(Color::Yellow)
                        .add_modifier(Modifier::BOLD),
                )));
                body_lines.push(Line::from(""));
                body_lines.push(Line::from(vec![
                    Span::styled(bar, Style::default().fg(Color::Cyan)),
                    Span::raw("  "),
                    Span::styled(
                        format!("{:.1}s / {:.0}s", elapsed, total),
                        Style::default().fg(Color::Gray),
                    ),
                ]));
                body_lines.push(Line::from(""));
                body_lines.push(Line::from(Span::styled(
                    "please speak normally — short sentences work best",
                    Style::default().fg(Color::DarkGray),
                )));
            }
            UsersModalState::Uploading { display_name } => {
                body_lines.push(Line::from(Span::styled(
                    format!("uploading enrollment for {}…", display_name),
                    Style::default().fg(Color::Yellow),
                )));
            }
            UsersModalState::Notice { text, ok } => {
                let style = if *ok {
                    Style::default()
                        .fg(Color::Green)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)
                };
                for ln in text.split('\n') {
                    body_lines.push(Line::from(Span::styled(ln.to_string(), style)));
                }
            }
        }

        let body = Paragraph::new(body_lines).wrap(Wrap { trim: false });
        f.render_widget(body, inner);
    })?;
    Ok(())
}

/// Outcome of one modal session — returned to `run_tui` so it can update
/// chat_cfg + the local voiceprint cache without the modal holding either
/// across the await boundary.
struct UsersModalOutcome {
    controller: Option<String>,
    db: VoiceprintDb,
    log_lines: Vec<String>,
}

/// Run the Ctrl+U modal until the user presses Esc. Drives its own
/// key/draw loop; the parent's `draw()` is paused for the duration.
#[allow(clippy::too_many_arguments)]
async fn run_users_modal(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    atlas_endpoint: &str,
    chat_cfg: &ChatConfig,
    initial_db: VoiceprintDb,
) -> Result<UsersModalOutcome> {
    let mut modal = UsersModal {
        state: UsersModalState::List,
        users: initial_db
            .users
            .iter()
            .map(|(id, u)| UserEntry {
                user_id: id.clone(),
                user_name: u.name.clone(),
            })
            .collect(),
        cursor: 0,
        controller: chat_cfg.current_controller.clone(),
        status: "refreshing from service…".to_string(),
    };
    let mut db = initial_db;
    let mut log_lines: Vec<String> = Vec::new();

    // Kick off a refresh in the background so the modal renders fast.
    match refresh_voiceprint_db(atlas_endpoint).await {
        Ok(new_db) => {
            modal.users = new_db
                .users
                .iter()
                .map(|(id, u)| UserEntry {
                    user_id: id.clone(),
                    user_name: u.name.clone(),
                })
                .collect();
            modal.status = format!("{} user(s) enrolled", modal.users.len());
            db = new_db;
        }
        Err(e) => {
            modal.status = format!("offline (showing cached list): {e:#}");
        }
    }

    let speaker_pin = chat_cfg.speaker_cap_id.clone().unwrap_or_default();
    let mic_pin = chat_cfg.mic_cap_id.clone().unwrap_or_default();

    loop {
        draw_users_modal(terminal, &modal)?;

        if let UsersModalState::Recording {
            started,
            audio,
            display_name,
            user_id,
        } = &modal.state
            && started.elapsed().as_secs() >= MIC_RECORD_SECS
            && !audio.is_empty()
        {
            {
                let dn = display_name.clone();
                let uid = user_id.clone();
                let audio_buf = audio.clone();
                modal.state = UsersModalState::Uploading {
                    display_name: dn.clone(),
                };
                draw_users_modal(terminal, &modal)?;
                match voiceprint_enroll(atlas_endpoint, &uid, &dn, audio_buf).await {
                    Ok(()) => {
                        log_lines.push(format!("enrolled voice: {} ({})", dn, uid));
                        // Fire-and-forget post-enroll voice prompt.
                        let ep = atlas_endpoint.to_string();
                        let sp = speaker_pin.clone();
                        let prompt = format!("已注册 {} 用户", dn);
                        tokio::spawn(async move {
                            let _ = speak_text(&ep, &sp, &prompt).await;
                        });
                        modal.state = UsersModalState::Notice {
                            text: format!("✓ enrolled {} as {}\n\nrefreshing user list…", dn, uid),
                            ok: true,
                        };
                        draw_users_modal(terminal, &modal)?;
                        if let Ok(new_db) = refresh_voiceprint_db(atlas_endpoint).await {
                            modal.users = new_db
                                .users
                                .iter()
                                .map(|(id, u)| UserEntry {
                                    user_id: id.clone(),
                                    user_name: u.name.clone(),
                                })
                                .collect();
                            db = new_db;
                        }
                    }
                    Err(e) => {
                        let err_str = format!("{e:#}");
                        log_lines.push(format!("enroll failed: {err_str}"));
                        // Dup-voice path: service error string contains
                        // "voice already enrolled as 'NAME'". Extract NAME
                        // and speak a Chinese audio prompt so the operator
                        // hears the rejection reason without staring at
                        // the modal.
                        if let Some(name) = err_str
                            .split_once("voice already enrolled as '")
                            .and_then(|(_, rest)| rest.split_once('\''))
                            .map(|(n, _)| n.to_string())
                        {
                            let ep = atlas_endpoint.to_string();
                            let sp = speaker_pin.clone();
                            let prompt = format!("该声音属于用户 {name}, 您已经注册过");
                            tokio::spawn(async move {
                                let _ = speak_text(&ep, &sp, &prompt).await;
                            });
                        }
                        modal.state = UsersModalState::Notice {
                            text: format!("✗ enroll failed: {err_str}"),
                            ok: false,
                        };
                    }
                }
                continue;
            }
        }

        // Recording state needs frequent redraws to animate the progress
        // bar; everything else can sleep on keys for a longer beat.
        let poll_ms = if matches!(modal.state, UsersModalState::Recording { .. }) {
            80
        } else {
            120
        };

        if event::poll(std::time::Duration::from_millis(poll_ms))?
            && let Event::Key(key) = event::read()?
        {
            match &mut modal.state {
                UsersModalState::List => match key.code {
                    KeyCode::Esc | KeyCode::Char('q') => break,
                    KeyCode::Up if modal.cursor > 0 => modal.cursor -= 1,
                    KeyCode::Down if modal.cursor + 1 < modal.users.len() => modal.cursor += 1,
                    KeyCode::Enter => {
                        if let Some(u) = modal.users.get(modal.cursor) {
                            modal.controller = Some(u.user_id.clone());
                            modal.status =
                                format!("active user set to {} ({})", u.user_name, u.user_id);
                            log_lines
                                .push(format!("active user → {} ({})", u.user_name, u.user_id));
                        }
                    }
                    KeyCode::Char('c') => {
                        modal.controller = None;
                        modal.status = "active user cleared — any speaker allowed".to_string();
                        log_lines.push("active user cleared".to_string());
                    }
                    KeyCode::Char('n') => {
                        modal.state = UsersModalState::Naming {
                            input: String::new(),
                        };
                        modal.status = "enter a display name, then Enter".to_string();
                    }
                    KeyCode::Char('r') => {
                        modal.status = "refreshing…".to_string();
                        draw_users_modal(terminal, &modal)?;
                        match refresh_voiceprint_db(atlas_endpoint).await {
                            Ok(new_db) => {
                                modal.users = new_db
                                    .users
                                    .iter()
                                    .map(|(id, u)| UserEntry {
                                        user_id: id.clone(),
                                        user_name: u.name.clone(),
                                    })
                                    .collect();
                                modal.status = format!("{} user(s) enrolled", modal.users.len());
                                db = new_db;
                            }
                            Err(e) => {
                                modal.status = format!("refresh failed: {e:#}");
                            }
                        }
                    }
                    KeyCode::Char('d') => {
                        // Delete the user under the cursor. Service is
                        // idempotent so we don't bother with a confirm
                        // dialog — the operator can just re-enrol with
                        // `n` if they hit `d` by mistake.
                        let Some(target) = modal.users.get(modal.cursor).cloned() else {
                            continue;
                        };
                        let uid = target.user_id.clone();
                        let dn = target.user_name.clone();
                        modal.status = format!("deleting {dn} ({uid})…");
                        draw_users_modal(terminal, &modal)?;
                        match voiceprint_delete(atlas_endpoint, &uid).await {
                            Ok(()) => {
                                log_lines.push(format!("deleted voice: {dn} ({uid})"));
                                // If the deleted user was the active one,
                                // drop the pin — otherwise next ASR turn
                                // would deny everyone forever.
                                if modal.controller.as_deref() == Some(uid.as_str()) {
                                    modal.controller = None;
                                    log_lines.push(
                                        "active user cleared (it was the deleted user)".to_string(),
                                    );
                                }
                                match refresh_voiceprint_db(atlas_endpoint).await {
                                    Ok(new_db) => {
                                        modal.users = new_db
                                            .users
                                            .iter()
                                            .map(|(id, u)| UserEntry {
                                                user_id: id.clone(),
                                                user_name: u.name.clone(),
                                            })
                                            .collect();
                                        if modal.cursor >= modal.users.len()
                                            && !modal.users.is_empty()
                                        {
                                            modal.cursor = modal.users.len() - 1;
                                        }
                                        modal.status = format!(
                                            "deleted {dn} · {} user(s) left",
                                            modal.users.len()
                                        );
                                        db = new_db;
                                    }
                                    Err(e) => {
                                        modal.status =
                                            format!("deleted {dn} but refresh failed: {e:#}");
                                    }
                                }
                            }
                            Err(e) => {
                                log_lines.push(format!("delete failed: {e:#}"));
                                modal.state = UsersModalState::Notice {
                                    text: format!("✗ delete failed: {e:#}"),
                                    ok: false,
                                };
                            }
                        }
                    }
                    _ => {}
                },
                UsersModalState::Naming { input } => match key.code {
                    KeyCode::Esc => {
                        modal.state = UsersModalState::List;
                        modal.status.clear();
                    }
                    KeyCode::Backspace => {
                        input.pop();
                    }
                    KeyCode::Char(c) if !key.modifiers.contains(KeyModifiers::CONTROL) => {
                        input.push(c);
                    }
                    KeyCode::Enter => {
                        let dn = input.trim().to_string();
                        if dn.is_empty() {
                            modal.status = "name can't be empty".to_string();
                            continue;
                        }
                        let user_id = format!("voice:{}", slug_for_user_id(&dn));
                        // Play the "请说话" prompt to completion BEFORE
                        // starting the mic capture — otherwise the prompt
                        // plays into the operator's own microphone and
                        // pollutes the enrolled embedding. Keep the modal
                        // on a Notice screen during playback so the operator
                        // sees a clear "wait" state instead of a frozen
                        // recording bar at 0.0s.
                        modal.state = UsersModalState::Notice {
                            text: format!(
                                "🔊 about to enrol voiceprint: {}\n\nwait for the spoken prompt to finish playing,\nthen start speaking.",
                                dn
                            ),
                            ok: true,
                        };
                        modal.status = "playing instruction…".to_string();
                        draw_users_modal(terminal, &modal)?;
                        {
                            let prompt = "开始声纹录入,请说话".to_string();
                            if let Err(e) = speak_text(atlas_endpoint, &speaker_pin, &prompt).await
                            {
                                log_lines.push(format!("speak prompt warn: {e:#}"));
                            }
                        }
                        // Short additional gap so the speaker driver fully
                        // drains its ALSA buffer before we open the mic
                        // stream — otherwise the very first ~200ms of
                        // recording still picks up TTS tail.
                        tokio::time::sleep(std::time::Duration::from_millis(300)).await;
                        let started = std::time::Instant::now();
                        modal.state = UsersModalState::Recording {
                            started,
                            audio: Vec::new(),
                            display_name: dn.clone(),
                            user_id: user_id.clone(),
                        };
                        modal.status = format!("recording {} for {}s…", dn, MIC_RECORD_SECS);
                        // Run capture in a background task so the modal
                        // can redraw the progress bar on a 100ms tick.
                        // Synchronous-await would freeze the UI for the
                        // entire MIC_RECORD_SECS window.
                        let ep = atlas_endpoint.to_string();
                        let mp = mic_pin.clone();
                        let capture_handle: tokio::task::JoinHandle<Result<Vec<u8>>> =
                            tokio::spawn(async move { capture_mic_pcm(&ep, &mp).await });
                        let mut handle = Some(capture_handle);
                        // Hard cap: stream.message() can block past the
                        // deadline if the mic stops yielding chunks.
                        // Force-abort 2s past deadline so the UI never
                        // hangs.
                        let hard_cap = tokio::time::Instant::now()
                            + std::time::Duration::from_secs(MIC_RECORD_SECS + 2);
                        let capture_result: Result<Vec<u8>> = loop {
                            draw_users_modal(terminal, &modal)?;
                            // Drain key events — only Esc cancels.
                            if event::poll(std::time::Duration::from_millis(0))?
                                && let Event::Key(k) = event::read()?
                                && matches!(k.code, KeyCode::Esc)
                            {
                                if let Some(h) = handle.take() {
                                    h.abort();
                                }
                                break Err(anyhow::anyhow!("cancelled"));
                            }
                            if let Some(h) = handle.as_ref()
                                && h.is_finished()
                            {
                                let h = handle.take().unwrap();
                                break h
                                    .await
                                    .unwrap_or_else(|e| Err(anyhow::anyhow!("join: {e}")));
                            }
                            if tokio::time::Instant::now() > hard_cap {
                                if let Some(h) = handle.take() {
                                    h.abort();
                                }
                                break Err(anyhow::anyhow!(
                                    "mic capture timeout (>{}s)",
                                    MIC_RECORD_SECS + 2
                                ));
                            }
                            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                        };
                        match capture_result {
                            Ok(buf) => {
                                if let UsersModalState::Recording { audio, .. } = &mut modal.state {
                                    *audio = buf;
                                }
                            }
                            Err(e) if e.to_string() == "cancelled" => {
                                modal.state = UsersModalState::List;
                                modal.status = "recording cancelled".to_string();
                            }
                            Err(e) => {
                                modal.state = UsersModalState::Notice {
                                    text: format!("✗ mic capture failed: {e:#}"),
                                    ok: false,
                                };
                                log_lines.push(format!("mic capture failed: {e:#}"));
                            }
                        }
                    }
                    _ => {}
                },
                UsersModalState::Recording { .. } if matches!(key.code, KeyCode::Esc) => {
                    modal.state = UsersModalState::List;
                    modal.status = "recording cancelled".to_string();
                }
                UsersModalState::Recording { .. } => {}
                UsersModalState::Uploading { .. } => { /* no input */ }
                UsersModalState::Notice { .. } => {
                    modal.state = UsersModalState::List;
                    modal.status.clear();
                }
            }
        }
    }

    Ok(UsersModalOutcome {
        controller: modal.controller,
        db,
        log_lines,
    })
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 1C — Ctrl+S System modal (Status / Logs / Perf)
// ─────────────────────────────────────────────────────────────────────────────
//
// One-key view of the running deploy:
//
//   Status — atlas provider table (provider_id, kind hint, state, ns).
//            Always available so long as atlas is reachable.
//   Logs   — tail the per-component log files boot wrote under
//            `<deploy>/rbnx-boot/logs/`. Discovers `<deploy>` by walking up
//            from CWD looking for `rbnx-boot/state.json`. If no deploy is
//            found nearby, the tab explains that and stays empty.
//   Perf   — `ps -o pid,pcpu,pmem,rss,etime,comm` over the PIDs recorded
//            in `<deploy>/rbnx-boot/state.json`. Same discovery as Logs.
//
// Tabs cycle with Tab / Shift+Tab; jump directly with 1 / 2 / 3.
// `r` refreshes the current tab; ↑↓ pick the focused log file (Logs tab).
// Esc closes the modal.

use super::teardown;

#[derive(Clone, Copy, PartialEq)]
enum SystemTab {
    Status,
    Logs,
    Perf,
}

struct SystemModal {
    tab: SystemTab,
    // Status tab
    providers: Vec<atlas_pb::CapabilityProvider>,
    status_error: Option<String>,
    // Logs / Perf tabs share a discovered deploy dir.
    deploy_dir: Option<std::path::PathBuf>,
    // Logs tab
    log_files: Vec<std::path::PathBuf>,
    log_cursor: usize,
    log_tail: Vec<String>,
    log_error: Option<String>,
    // Perf tab
    boot_pids: Vec<(String, u32)>,
    perf_rows: Vec<String>,
    perf_error: Option<String>,
    // Footer / global status
    note: String,
}

/// Walk up from CWD looking for `rbnx-boot/state.json`. Returns the
/// containing deploy dir (the parent that holds `rbnx-boot/`). Stops at
/// filesystem root. Used by Logs/Perf to scope themselves to an active
/// deploy without forcing the user to launch chat from inside the
/// deploy tree.
fn find_deploy_dir() -> Option<std::path::PathBuf> {
    let cwd = std::env::current_dir().ok()?;
    let mut cur = cwd.as_path();
    loop {
        let candidate = cur.join("rbnx-boot").join("state.json");
        if candidate.exists() {
            return Some(cur.to_path_buf());
        }
        match cur.parent() {
            Some(p) => cur = p,
            None => return None,
        }
    }
}

/// Read the last `n` lines of `path` into a Vec<String>. Uses a single
/// read + split rather than a true reverse-scan — adequate for files
/// boot's spawned packages produce (rotated daily at most, usually
/// <50MB). Returns the lines in original order (oldest first).
fn tail_log_lines(path: &std::path::Path, n: usize) -> std::io::Result<Vec<String>> {
    let s = std::fs::read_to_string(path)?;
    let lines: Vec<String> = s.lines().map(|l| l.to_string()).collect();
    let start = lines.len().saturating_sub(n);
    Ok(lines[start..].to_vec())
}

fn human_state(s: i32) -> &'static str {
    match atlas_pb::LifecycleState::try_from(s)
        .unwrap_or(atlas_pb::LifecycleState::StateUnspecified)
    {
        atlas_pb::LifecycleState::StateRegistered => "REGISTERED",
        atlas_pb::LifecycleState::StateInactive => "INACTIVE",
        atlas_pb::LifecycleState::StateActive => "ACTIVE",
        atlas_pb::LifecycleState::StateError => "ERROR",
        atlas_pb::LifecycleState::StateTerminated => "TERMINATED",
        atlas_pb::LifecycleState::StateUnspecified => "?",
    }
}

fn state_color(s: i32) -> Color {
    match atlas_pb::LifecycleState::try_from(s)
        .unwrap_or(atlas_pb::LifecycleState::StateUnspecified)
    {
        atlas_pb::LifecycleState::StateActive => Color::Green,
        atlas_pb::LifecycleState::StateInactive => Color::Yellow,
        atlas_pb::LifecycleState::StateRegistered => Color::Blue,
        atlas_pb::LifecycleState::StateError => Color::Red,
        atlas_pb::LifecycleState::StateTerminated => Color::DarkGray,
        atlas_pb::LifecycleState::StateUnspecified => Color::DarkGray,
    }
}

async fn refresh_status_tab(modal: &mut SystemModal, atlas_endpoint: &str) {
    match AtlasClient::connect(atlas_endpoint.to_string()).await {
        Ok(mut atlas) => match atlas
            .query_capabilities("", "", atlas_pb::Transport::Unspecified)
            .await
        {
            Ok(providers) => {
                modal.providers = providers;
                modal.status_error = None;
            }
            Err(e) => modal.status_error = Some(format!("query atlas: {e:#}")),
        },
        Err(e) => modal.status_error = Some(format!("connect atlas: {e:#}")),
    }
}

fn refresh_logs_tab(modal: &mut SystemModal) {
    modal.log_files.clear();
    modal.log_tail.clear();
    modal.log_error = None;
    let Some(deploy) = modal.deploy_dir.clone() else {
        modal.log_error =
            Some("no active deploy found near CWD (looked for ./rbnx-boot/state.json)".to_string());
        return;
    };
    let log_dir = deploy.join("rbnx-boot").join("logs");
    let entries = match std::fs::read_dir(&log_dir) {
        Ok(it) => it,
        Err(e) => {
            modal.log_error = Some(format!("read {}: {e:#}", log_dir.display()));
            return;
        }
    };
    let mut files: Vec<std::path::PathBuf> = entries
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "log"))
        .collect();
    files.sort();
    modal.log_files = files;
    if modal.log_cursor >= modal.log_files.len() {
        modal.log_cursor = 0;
    }
    if let Some(path) = modal.log_files.get(modal.log_cursor) {
        match tail_log_lines(path, 80) {
            Ok(lines) => modal.log_tail = lines,
            Err(e) => modal.log_error = Some(format!("tail {}: {e:#}", path.display())),
        }
    }
}

fn refresh_perf_tab(modal: &mut SystemModal) {
    modal.boot_pids.clear();
    modal.perf_rows.clear();
    modal.perf_error = None;
    let Some(deploy) = modal.deploy_dir.clone() else {
        modal.perf_error =
            Some("no active deploy found near CWD (looked for ./rbnx-boot/state.json)".to_string());
        return;
    };
    let state_path = teardown::state_path(&deploy);
    let state = match teardown::read_state(&state_path) {
        Ok(s) => s,
        Err(e) => {
            modal.perf_error = Some(format!("read {}: {e:#}", state_path.display()));
            return;
        }
    };
    modal.boot_pids = state
        .components
        .iter()
        .map(|c| (c.name.clone(), c.pid))
        .collect();
    if modal.boot_pids.is_empty() {
        modal.perf_error = Some("boot state has no components".to_string());
        return;
    }
    // One ps call for the whole set — cheaper than per-PID, and the
    // output's `comm` column lets us cross-check name vs PID for free.
    let pid_csv = modal
        .boot_pids
        .iter()
        .map(|(_, pid)| pid.to_string())
        .collect::<Vec<_>>()
        .join(",");
    let out = std::process::Command::new("ps")
        .args([
            "-o",
            "pid,pcpu,pmem,rss,etime,comm",
            "--no-headers",
            "-p",
            &pid_csv,
        ])
        .output();
    let out = match out {
        Ok(o) => o,
        Err(e) => {
            modal.perf_error = Some(format!("spawn ps: {e:#}"));
            return;
        }
    };
    if !out.status.success() {
        modal.perf_error = Some(format!(
            "ps failed ({}): {}",
            out.status,
            String::from_utf8_lossy(&out.stderr).trim()
        ));
        return;
    }
    modal.perf_rows = String::from_utf8_lossy(&out.stdout)
        .lines()
        .map(|s| s.to_string())
        .collect();
}

fn draw_system_modal(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    modal: &SystemModal,
) -> Result<()> {
    terminal.draw(|f| {
        let area = centered_rect(80, 80, f.area());
        f.render_widget(Clear, area);

        let title_spans = vec![
            Span::styled(
                " System · ",
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            ),
            tab_span("Status", modal.tab == SystemTab::Status),
            Span::raw(" "),
            tab_span("Logs", modal.tab == SystemTab::Logs),
            Span::raw(" "),
            tab_span("Perf", modal.tab == SystemTab::Perf),
            Span::raw(" "),
        ];
        let footer = match modal.tab {
            SystemTab::Status => " Tab next · 1/2/3 jump · r refresh · Esc close ",
            SystemTab::Logs => " Tab next · ↑↓ pick file · r refresh · Esc close ",
            SystemTab::Perf => " Tab next · r refresh · Esc close ",
        };
        let block = Block::default()
            .borders(Borders::ALL)
            .title(Line::from(title_spans))
            .title_bottom(
                Line::from(Span::styled(footer, Style::default().fg(Color::DarkGray)))
                    .alignment(Alignment::Right),
            );
        let inner = block.inner(area);
        f.render_widget(block, area);

        match modal.tab {
            SystemTab::Status => draw_status_tab(f, inner, modal),
            SystemTab::Logs => draw_logs_tab(f, inner, modal),
            SystemTab::Perf => draw_perf_tab(f, inner, modal),
        }
    })?;
    Ok(())
}

fn tab_span(label: &'static str, active: bool) -> Span<'static> {
    if active {
        Span::styled(
            format!(" {label} "),
            Style::default()
                .fg(Color::Black)
                .bg(Color::Cyan)
                .add_modifier(Modifier::BOLD),
        )
    } else {
        Span::styled(format!(" {label} "), Style::default().fg(Color::Gray))
    }
}

fn draw_status_tab(f: &mut ratatui::Frame, area: Rect, modal: &SystemModal) {
    let mut lines: Vec<Line> = Vec::new();
    if let Some(err) = &modal.status_error {
        lines.push(Line::from(Span::styled(
            format!("error: {err}"),
            Style::default().fg(Color::Red),
        )));
        let p = Paragraph::new(lines).wrap(Wrap { trim: false });
        f.render_widget(p, area);
        return;
    }
    if modal.providers.is_empty() {
        lines.push(Line::from(Span::styled(
            "no providers registered",
            Style::default().fg(Color::DarkGray),
        )));
    } else {
        lines.push(Line::from(Span::styled(
            format!(
                "{:<12}{:<28}{:<48}{}",
                "state", "provider_id", "namespace", "caps"
            ),
            Style::default()
                .fg(Color::White)
                .add_modifier(Modifier::BOLD),
        )));
        for p in &modal.providers {
            lines.push(Line::from(vec![
                Span::styled(
                    format!("{:<12}", format!("[{}]", human_state(p.state))),
                    Style::default()
                        .fg(state_color(p.state))
                        .add_modifier(Modifier::BOLD),
                ),
                Span::styled(
                    format!("{:<28}", truncate(&p.id, 27)),
                    Style::default().fg(Color::White),
                ),
                Span::styled(
                    format!("{:<48}", truncate(&p.namespace, 47)),
                    Style::default().fg(Color::Gray),
                ),
                Span::styled(
                    format!("{}", p.capabilities.len()),
                    Style::default().fg(Color::DarkGray),
                ),
            ]));
        }
    }
    let para = Paragraph::new(lines).wrap(Wrap { trim: false });
    f.render_widget(para, area);
}

fn draw_logs_tab(f: &mut ratatui::Frame, area: Rect, modal: &SystemModal) {
    if let Some(err) = &modal.log_error
        && modal.log_files.is_empty()
    {
        let para = Paragraph::new(Line::from(Span::styled(
            err.clone(),
            Style::default().fg(Color::Red),
        )))
        .wrap(Wrap { trim: false });
        f.render_widget(para, area);
        return;
    }
    // Left: file list (30%). Right: tail (rest).
    let cols =
        Layout::horizontal([Constraint::Percentage(30), Constraint::Percentage(70)]).split(area);

    let file_lines: Vec<Line> = if modal.log_files.is_empty() {
        vec![Line::from(Span::styled(
            "(no log files)",
            Style::default().fg(Color::DarkGray),
        ))]
    } else {
        modal
            .log_files
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let name = p
                    .file_stem()
                    .and_then(|n| n.to_str())
                    .unwrap_or("?")
                    .to_string();
                if i == modal.log_cursor {
                    Line::from(Span::styled(
                        format!("▶ {}", name),
                        Style::default()
                            .fg(Color::Black)
                            .bg(Color::Cyan)
                            .add_modifier(Modifier::BOLD),
                    ))
                } else {
                    Line::from(Span::styled(
                        format!("  {}", name),
                        Style::default().fg(Color::White),
                    ))
                }
            })
            .collect()
    };
    let files = Paragraph::new(file_lines)
        .block(
            Block::default()
                .borders(Borders::RIGHT)
                .border_style(Style::default().fg(Color::DarkGray)),
        )
        .wrap(Wrap { trim: false });
    f.render_widget(files, cols[0]);

    let tail_lines: Vec<Line> = if let Some(err) = &modal.log_error {
        vec![Line::from(Span::styled(
            err.clone(),
            Style::default().fg(Color::Red),
        ))]
    } else {
        modal
            .log_tail
            .iter()
            .map(|l| Line::from(Span::raw(l.clone())))
            .collect()
    };
    let tail = Paragraph::new(tail_lines).wrap(Wrap { trim: false });
    f.render_widget(tail, cols[1]);
}

fn draw_perf_tab(f: &mut ratatui::Frame, area: Rect, modal: &SystemModal) {
    let mut lines: Vec<Line> = Vec::new();
    if let Some(err) = &modal.perf_error {
        lines.push(Line::from(Span::styled(
            format!("error: {err}"),
            Style::default().fg(Color::Red),
        )));
    } else {
        lines.push(Line::from(Span::styled(
            format!(
                "{:<8}{:<8}{:<8}{:<12}{:<14}{}",
                "pid", "%cpu", "%mem", "rss(kb)", "uptime", "comm"
            ),
            Style::default()
                .fg(Color::White)
                .add_modifier(Modifier::BOLD),
        )));
        // map pid → name from the boot state for the "from boot" hint
        let pid_to_name: std::collections::HashMap<u32, String> = modal
            .boot_pids
            .iter()
            .map(|(n, p)| (*p, n.clone()))
            .collect();
        for row in &modal.perf_rows {
            // ps output: pid pcpu pmem rss etime comm (whitespace-separated)
            let cols: Vec<&str> = row.split_whitespace().collect();
            if cols.len() < 6 {
                lines.push(Line::from(Span::raw(row.clone())));
                continue;
            }
            let pid: u32 = cols[0].parse().unwrap_or(0);
            let name = pid_to_name
                .get(&pid)
                .cloned()
                .unwrap_or_else(|| cols[5].to_string());
            lines.push(Line::from(vec![
                Span::styled(format!("{:<8}", cols[0]), Style::default().fg(Color::Cyan)),
                Span::styled(format!("{:<8}", cols[1]), Style::default().fg(Color::Green)),
                Span::styled(
                    format!("{:<8}", cols[2]),
                    Style::default().fg(Color::Yellow),
                ),
                Span::styled(
                    format!("{:<12}", cols[3]),
                    Style::default().fg(Color::White),
                ),
                Span::styled(format!("{:<14}", cols[4]), Style::default().fg(Color::Gray)),
                Span::styled(name, Style::default().fg(Color::White)),
            ]));
        }
        if !modal.note.is_empty() {
            lines.push(Line::from(""));
            lines.push(Line::from(Span::styled(
                modal.note.clone(),
                Style::default().fg(Color::DarkGray),
            )));
        }
    }
    let para = Paragraph::new(lines).wrap(Wrap { trim: false });
    f.render_widget(para, area);
}

/// Truncate `s` to at most `max` chars, suffixing `…` when cut. `max`
/// must be ≥ 1; if it's smaller than the ellipsis we just return the
/// raw prefix.
fn truncate(s: &str, max: usize) -> String {
    if s.chars().count() <= max {
        s.to_string()
    } else if max <= 1 {
        s.chars().take(max).collect()
    } else {
        let mut out: String = s.chars().take(max - 1).collect();
        out.push('…');
        out
    }
}

async fn run_system_modal(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    atlas_endpoint: &str,
) -> Result<()> {
    let mut modal = SystemModal {
        tab: SystemTab::Status,
        providers: Vec::new(),
        status_error: None,
        deploy_dir: find_deploy_dir(),
        log_files: Vec::new(),
        log_cursor: 0,
        log_tail: Vec::new(),
        log_error: None,
        boot_pids: Vec::new(),
        perf_rows: Vec::new(),
        perf_error: None,
        note: String::new(),
    };
    refresh_status_tab(&mut modal, atlas_endpoint).await;
    refresh_logs_tab(&mut modal);
    refresh_perf_tab(&mut modal);

    loop {
        draw_system_modal(terminal, &modal)?;
        if event::poll(std::time::Duration::from_millis(120))?
            && let Event::Key(key) = event::read()?
        {
            match key.code {
                KeyCode::Esc => break,
                KeyCode::Char('q') => break,
                KeyCode::Char('s') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                    break;
                }
                KeyCode::Tab => {
                    modal.tab = match modal.tab {
                        SystemTab::Status => SystemTab::Logs,
                        SystemTab::Logs => SystemTab::Perf,
                        SystemTab::Perf => SystemTab::Status,
                    };
                }
                KeyCode::BackTab => {
                    modal.tab = match modal.tab {
                        SystemTab::Status => SystemTab::Perf,
                        SystemTab::Logs => SystemTab::Status,
                        SystemTab::Perf => SystemTab::Logs,
                    };
                }
                KeyCode::Char('1') => modal.tab = SystemTab::Status,
                KeyCode::Char('2') => modal.tab = SystemTab::Logs,
                KeyCode::Char('3') => modal.tab = SystemTab::Perf,
                KeyCode::Char('r') => match modal.tab {
                    SystemTab::Status => refresh_status_tab(&mut modal, atlas_endpoint).await,
                    SystemTab::Logs => refresh_logs_tab(&mut modal),
                    SystemTab::Perf => refresh_perf_tab(&mut modal),
                },
                KeyCode::Up if matches!(modal.tab, SystemTab::Logs) && modal.log_cursor > 0 => {
                    modal.log_cursor -= 1;
                    refresh_logs_tab(&mut modal);
                }
                KeyCode::Down
                    if matches!(modal.tab, SystemTab::Logs)
                        && modal.log_cursor + 1 < modal.log_files.len() =>
                {
                    modal.log_cursor += 1;
                    refresh_logs_tab(&mut modal);
                }
                _ => {}
            }
        }
    }
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 1D — Ctrl+, Settings page (consolidates Audio / Users / System / Modes)
// ─────────────────────────────────────────────────────────────────────────────
//
// Replaces the previous "one Ctrl-key per subsystem" sprawl (Ctrl+A audio,
// Ctrl+U users, Ctrl+S system) with a single discoverable entry point:
//
//   Ctrl+,  → Settings menu (vertical sidebar of categories)
//             ↑↓ select · Enter open · 1-4 jump · Esc close
//
// Each menu entry either renders inline (Modes — small enough to fit on
// the sidebar's right half) or hands off to the existing full-screen
// modal (Audio / Users / System). The existing Ctrl+A/U/S shortcuts
// are kept as power-user deep-links — they no longer appear in the
// footer hint, but pressing them jumps straight to that category.

#[derive(Clone, Copy, PartialEq, Eq)]
enum SettingsCategory {
    Modes,
    Audio,
    Users,
    System,
}

impl SettingsCategory {
    fn label(self) -> &'static str {
        match self {
            Self::Modes => "Modes",
            Self::Audio => "Audio",
            Self::Users => "Users",
            Self::System => "System",
        }
    }

    fn hint(self) -> &'static str {
        match self {
            Self::Modes => "input / output modality",
            Self::Audio => "microphone + speaker",
            Self::Users => "voiceprint enrol + active speaker",
            Self::System => "provider status / logs / process metrics",
        }
    }

    fn all() -> [Self; 4] {
        [Self::Modes, Self::Audio, Self::Users, Self::System]
    }
}

/// What `run_settings_menu` returned to `run_tui`. Carries the updated
/// chat_cfg + voiceprint cache so the parent can persist them once,
/// instead of every category modal touching disk independently.
struct SettingsOutcome {
    chat_cfg: ChatConfig,
    voiceprint_db: VoiceprintDb,
    log_lines: Vec<String>,
}

/// Sidebar Settings menu — Ctrl+, entry point. Dispatches to the
/// existing per-category modals (audio / users / system) or renders
/// Modes inline. Returns once the user presses Esc.
async fn run_settings_menu(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    atlas_endpoint: &str,
    mut chat_cfg: ChatConfig,
    voiceprint_db_initial: VoiceprintDb,
    initial_category: Option<SettingsCategory>,
) -> Result<SettingsOutcome> {
    let cats = SettingsCategory::all();
    let mut cursor: usize = initial_category
        .and_then(|c| cats.iter().position(|x| *x == c))
        .unwrap_or(0);
    let mut voiceprint_db = voiceprint_db_initial;
    let mut log_lines: Vec<String> = Vec::new();
    let mut status_msg = String::new();

    // If a specific category was requested (deep-link from a hotkey),
    // open it immediately instead of forcing the user through the menu.
    if initial_category.is_some() {
        dispatch_settings_category(
            terminal,
            atlas_endpoint,
            cats[cursor],
            &mut chat_cfg,
            &mut voiceprint_db,
            &mut log_lines,
            &mut status_msg,
        )
        .await;
    }

    loop {
        draw_settings_menu(terminal, &cats, cursor, &chat_cfg, &status_msg)?;

        if event::poll(std::time::Duration::from_millis(120))?
            && let Event::Key(key) = event::read()?
        {
            let close = matches!(key.code, KeyCode::Esc | KeyCode::Char('q'))
                || (key.modifiers.contains(KeyModifiers::CONTROL)
                    && key.code == KeyCode::Char(','));
            if close {
                break;
            }
            match key.code {
                KeyCode::Up if cursor > 0 => cursor -= 1,
                KeyCode::Down if cursor + 1 < cats.len() => cursor += 1,
                KeyCode::Char('1') => cursor = 0,
                KeyCode::Char('2') => cursor = 1,
                KeyCode::Char('3') => cursor = 2,
                KeyCode::Char('4') => cursor = 3,
                KeyCode::Enter | KeyCode::Right => {
                    dispatch_settings_category(
                        terminal,
                        atlas_endpoint,
                        cats[cursor],
                        &mut chat_cfg,
                        &mut voiceprint_db,
                        &mut log_lines,
                        &mut status_msg,
                    )
                    .await;
                }
                _ => {}
            }
        }
    }

    Ok(SettingsOutcome {
        chat_cfg,
        voiceprint_db,
        log_lines,
    })
}

#[allow(clippy::too_many_arguments)]
async fn dispatch_settings_category(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    atlas_endpoint: &str,
    cat: SettingsCategory,
    chat_cfg: &mut ChatConfig,
    voiceprint_db: &mut VoiceprintDb,
    log_lines: &mut Vec<String>,
    status_msg: &mut String,
) {
    match cat {
        SettingsCategory::Modes => match run_modes_page(terminal, chat_cfg.clone()).await {
            Ok(new_cfg) => {
                *chat_cfg = new_cfg;
                *status_msg = format!("modes saved · response_voice = {}", chat_cfg.response_voice);
            }
            Err(e) => *status_msg = format!("modes: {e:#}"),
        },
        SettingsCategory::Audio => {
            match run_audio_settings_page(atlas_endpoint, terminal, chat_cfg.clone()).await {
                Ok(new_cfg) => {
                    *chat_cfg = new_cfg;
                    *status_msg = format!(
                        "audio saved · mic={}/{} · speaker={}/{}",
                        chat_cfg.mic_cap_id.as_deref().unwrap_or("(unset)"),
                        chat_cfg.mic_device_id.as_deref().unwrap_or("default"),
                        chat_cfg.speaker_cap_id.as_deref().unwrap_or("(unset)"),
                        chat_cfg.speaker_device_id.as_deref().unwrap_or("default"),
                    );
                }
                Err(e) => *status_msg = format!("audio: {e:#}"),
            }
        }
        SettingsCategory::Users => {
            let db_snapshot = voiceprint_db.clone();
            match run_users_modal(terminal, atlas_endpoint, chat_cfg, db_snapshot).await {
                Ok(out) => {
                    chat_cfg.current_controller = out.controller.clone();
                    *voiceprint_db = out.db;
                    log_lines.extend(out.log_lines);
                    *status_msg = match &chat_cfg.current_controller {
                        Some(c) => format!("users saved · active = {c}"),
                        None => "users saved · active user cleared".to_string(),
                    };
                }
                Err(e) => *status_msg = format!("users: {e:#}"),
            }
        }
        SettingsCategory::System => {
            if let Err(e) = run_system_modal(terminal, atlas_endpoint).await {
                *status_msg = format!("system: {e:#}");
            } else {
                *status_msg = "system view closed".to_string();
            }
        }
    }
}

fn draw_settings_menu(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    cats: &[SettingsCategory],
    cursor: usize,
    chat_cfg: &ChatConfig,
    status_msg: &str,
) -> Result<()> {
    terminal.draw(|f| {
        let area = centered_rect(60, 60, f.area());
        f.render_widget(Clear, area);
        let block = Block::default()
            .borders(Borders::ALL)
            .title(Line::from(Span::styled(
                " Settings ",
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            )))
            .title_bottom(
                Line::from(Span::styled(
                    " ↑↓ select · Enter open · 1-4 jump · Esc / Ctrl+, close ",
                    Style::default().fg(Color::DarkGray),
                ))
                .alignment(Alignment::Right),
            );
        let inner = block.inner(area);
        f.render_widget(block, area);

        let cols = Layout::horizontal([Constraint::Length(20), Constraint::Min(20)]).split(inner);

        // Sidebar — category list
        let sidebar: Vec<Line> = cats
            .iter()
            .enumerate()
            .map(|(i, c)| {
                let mark = if i == cursor { "▶ " } else { "  " };
                let style = if i == cursor {
                    Style::default()
                        .fg(Color::Black)
                        .bg(Color::Cyan)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(Color::White)
                };
                Line::from(Span::styled(format!("{}{}", mark, c.label()), style))
            })
            .collect();
        let side = Paragraph::new(sidebar).wrap(Wrap { trim: false });
        f.render_widget(side, cols[0]);

        // Right pane — short summary for the highlighted category + current
        // chat-cfg snapshot, so the user can glance at "where am I" without
        // entering each tab.
        let cur = cats[cursor];
        let mut body: Vec<Line> = vec![
            Line::from(Span::styled(
                cur.label().to_string(),
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            )),
            Line::from(Span::styled(
                cur.hint().to_string(),
                Style::default().fg(Color::Gray),
            )),
            Line::from(""),
            Line::from(Span::styled(
                "Current configuration:",
                Style::default()
                    .fg(Color::White)
                    .add_modifier(Modifier::BOLD),
            )),
            Line::from(format!(
                "  response voice  : {}",
                if chat_cfg.response_voice { "on" } else { "off" }
            )),
            Line::from(format!(
                "  mic provider    : {}",
                chat_cfg.mic_cap_id.as_deref().unwrap_or("(unset)")
            )),
            Line::from(format!(
                "  speaker provider: {}",
                chat_cfg.speaker_cap_id.as_deref().unwrap_or("(unset)")
            )),
            Line::from(format!(
                "  active user     : {}",
                chat_cfg.current_controller.as_deref().unwrap_or("(any)")
            )),
        ];
        if !status_msg.is_empty() {
            body.push(Line::from(""));
            body.push(Line::from(Span::styled(
                status_msg.to_string(),
                Style::default().fg(Color::Green),
            )));
        }
        let pane = Paragraph::new(body).wrap(Wrap { trim: false });
        f.render_widget(pane, cols[1]);
    })?;
    Ok(())
}

/// Modes page (Settings → Modes). Currently a single binary toggle —
/// `response_voice` — but laid out as a list so future modes
/// (e.g. continuous vs push-to-talk, language preset, …) drop in
/// without restructuring the page.
async fn run_modes_page(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    initial: ChatConfig,
) -> Result<ChatConfig> {
    let mut cfg = initial;
    let mut cursor: usize = 0;
    const ROW_COUNT: usize = 1;
    loop {
        terminal.draw(|f| {
            let area = centered_rect(70, 60, f.area());
            f.render_widget(Clear, area);
            let block = Block::default()
                .borders(Borders::ALL)
                .title(Line::from(Span::styled(
                    " Settings · Modes ",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                )))
                .title_bottom(
                    Line::from(Span::styled(
                        " ↑↓ select · Enter / Space toggle · Esc back ",
                        Style::default().fg(Color::DarkGray),
                    ))
                    .alignment(Alignment::Right),
                );
            let inner = block.inner(area);
            f.render_widget(block, area);

            let mut lines: Vec<Line> = Vec::new();
            lines.push(Line::from(Span::styled(
                "Reply modality",
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            )));
            lines.push(Line::from(Span::styled(
                "How the agent responds when you type via Enter. Voice input \
                 (Ctrl+V) always uses voice output regardless of this toggle.",
                Style::default().fg(Color::Gray),
            )));
            lines.push(Line::from(""));
            let mark = if cursor == 0 { "▶ " } else { "  " };
            let state = if cfg.response_voice {
                "[x] voice + text  (TTS plays the reply; pilot uses voice brevity rule)"
            } else {
                "[ ] voice + text  (replies stay in the chat window only)"
            };
            let row_style = if cursor == 0 {
                Style::default()
                    .fg(Color::Black)
                    .bg(Color::Cyan)
                    .add_modifier(Modifier::BOLD)
            } else {
                Style::default().fg(Color::White)
            };
            lines.push(Line::from(Span::styled(
                format!("{mark}{state}"),
                row_style,
            )));
            lines.push(Line::from(""));
            lines.push(Line::from(Span::styled(
                "Notes",
                Style::default()
                    .fg(Color::White)
                    .add_modifier(Modifier::BOLD),
            )));
            lines.push(Line::from(Span::styled(
                "  · Voice output requires a working speaker provider \
                 (configure under Audio).",
                Style::default().fg(Color::Gray),
            )));
            lines.push(Line::from(Span::styled(
                "  · Pilot trims markdown and shortens sentences when modality=voice.",
                Style::default().fg(Color::Gray),
            )));
            let p = Paragraph::new(lines).wrap(Wrap { trim: false });
            f.render_widget(p, inner);
        })?;

        if event::poll(std::time::Duration::from_millis(120))?
            && let Event::Key(key) = event::read()?
        {
            match key.code {
                KeyCode::Esc | KeyCode::Char('q') | KeyCode::Left => break,
                KeyCode::Up if cursor > 0 => cursor -= 1,
                KeyCode::Down if cursor + 1 < ROW_COUNT => cursor += 1,
                KeyCode::Enter | KeyCode::Char(' ') if cursor == 0 => {
                    cfg.response_voice = !cfg.response_voice;
                }
                _ => {}
            }
        }
    }
    Ok(cfg)
}
