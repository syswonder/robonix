# robonix-liaison

A unified user input gateway supporting both text and voice modalities.

## Architecture

```
User
  │
  ├─ Text input (Enter)
  │     │
  │     ▼
  │   SrvLiaison.Stream(Task) ──► Pilot
  │     │
  │     ▼
  │   PilotEvent stream ◄────────────┘
  │
  └─ Voice input (Ctrl+V in TUI)
        │
        ▼
      SrvLiaison.StartVoiceSession(req)
        │
        ├─ PrmAudioMic.Stream (record N seconds)
        ├─ SrvSpeechAsr.Call (speech recognition)
        ├─ SrvSpeechVoiceprint.Call (voiceprint recognition → user_id)
        ├─ Assemble pilot::Task { user_id, text=transcript, … }
        ├─ SrvPilot.Stream
        ├─ (optional) SrvSpeechTts.Call + PrmAudioSpeaker.Stream
        │
        ▼
      VoiceEvent stream ◄───────────────────────────────────┘
```

## Key Changes

1. **`pilot::Task.user_id`** — New field, populated automatically by Liaison:
   - Text path: `local:<os_user>`
   - Voice path: `voice:<id>` (from voiceprint) or fallback `voice:unknown`

2. **`SrvLiaison.StartVoiceSession`** — New RPC that orchestrates the entire voice conversation.

3. **`rbnx chat` TUI** — Now connects to Liaison instead of Pilot; `Ctrl+V` starts a voice conversation.

## Running the Demo (Mock mode)

No real mic / ASR / TTS / VLM required; uses preset text to verify the end-to-end pipeline:

```bash
cd rust
./examples/voice_demo.sh
```

The output should show `text path → OK` and `voice path → OK`.

## Environment Variables

| Variable | Default | Description |
|------|--------|------|
| `ROBONIX_ATLAS` | `127.0.0.1:50051` | Atlas address |
| `ROBONIX_PILOT_ENDPOINT` | `127.0.0.1:50071` | Pilot address |
| `ROBONIX_LIAISON_PORT` | `50081` | Liaison listen port |
| `ROBONIX_LIAISON_VOICE_MOCK` | (unset) | Set to `1` to skip mic+ASR and use preset text |
| `ROBONIX_LIAISON_VOICE_MOCK_TEXT` | `Hello, please introduce yourself.` | Text used in mock mode |
| `ROBONIX_LIAISON_SOURCE` | (unset) | Set to `text` to enable the stdin text loop (headless) |
| `ROBONIX_LIAISON_ACCESS_ENABLED` | `0` | Set to `1` to require user/voice access before Pilot, ASR post-gate, and TTS |
| `ROBONIX_LIAISON_ALLOWED_USERS` | (empty) | Comma/space separated ids such as `local:alice,voice:alice` |
| `ROBONIX_LIAISON_VOICE_THRESHOLD` | `0.25` | Minimum voiceprint confidence for a voice user to pass |

When access is enabled, text/API tasks must carry an allowed
`context_json.user_id` after Liaison normalisation. Voice turns capture mic
audio first, call voiceprint, and only then enter ASR/Pilot/TTS if either the
client user hint or the matched `voice:<user_id>` is in the allowed set.

## TUI Shortcuts

| Key | Function |
|------|------|
| `Enter` | Send text message |
| `Ctrl+V` | Start voice conversation (default 5-second recording) |
| `Esc` | Interrupt the current turn (abort_turn) |
| `Ctrl+C` | Quit |
| `PageUp/PageDown` | Scroll history |

## Voice Path VoiceEvent Types

| kind | Name | Description |
|------|------|------|
| 0 | SESSION_STARTED | Session started |
| 1 | RECORDING_STARTED | Recording started |
| 2 | RECORDING_DONE | Recording finished |
| 3 | ASR_PARTIAL | ASR intermediate result |
| 4 | ASR_FINAL | ASR final result |
| 5 | USER_IDENTIFIED | Voiceprint recognition result |
| 6 | PILOT | Wrapped PilotEvent |
| 7 | TTS_STARTED | TTS started |
| 8 | TTS_DONE | TTS finished |
| 9 | SESSION_DONE | Session ended normally |
| 10 | ERROR | Error |
