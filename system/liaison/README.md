# robonix-liaison

Liaison is the authenticated user-input gateway in front of Pilot. It exposes
text, voice, hands-free, and Keystone account APIs, resolves the login session
to a canonical account, applies per-user access policy, then forwards accepted
tasks to Pilot.

Liaison does not implement ASR, TTS, microphone capture, speaker output, or
voiceprint itself. Those are separate providers discovered through Atlas.

## Architecture

```
User
  │
  ├─ Text / API task
  │     │
  │     ▼
  │   robonix/system/liaison/submit
  │     │ authenticate session with Keystone
  │     │ inject canonical account identity
  │     ▼
  │   Pilot SubmitTask
  │     │
  │     ▼
  │   PilotEvent stream
  │
  └─ Voice session (Ctrl+V in rbnx chat today)
        │
        ▼
      robonix/system/liaison/voice
        │
        ├─ robonix/primitive/audio/mic_stream
        ├─ robonix/service/speech/asr_stream
        ├─ optional robonix/service/voiceprint/identify
        ├─ Keystone voice guard before Pilot/TTS/action
        ├─ build pilot::Task with canonical account context
        ├─ Pilot SubmitTask
        ├─ optional robonix/service/speech/tts
        ├─ optional speaker playback
        │
        ▼
      VoiceEvent stream
```

## Contracts

- `robonix/system/liaison/submit`: accepts a `pilot::Task` and streams
  `PilotEvent` responses back to the caller.
- `robonix/system/liaison/voice`: starts a voice session and streams
  `VoiceEvent` state, ASR, voiceprint, Pilot, and TTS progress.
- `robonix/system/liaison/handsfree/*`: enables, observes, and disables the
  robot-local wake-word loop. Enabling requires a live Keystone session.
- `robonix.keystone.v1.Keystone/*`: account API proxied by Liaison so clients
  do not need network access to Keystone's private listener.

The Liaison capability contracts are registered by the `liaison` provider id.

## Identity and access

In a Keystone-enabled deployment, the client carries an opaque
`context_json.session_token` for text tasks and a typed `session_token` for
voice and hands-free RPCs. Liaison resolves it before opening audio devices or
forwarding work. Client-supplied `user_id`, display name, and roles are never
authorization inputs.

Accepted Pilot tasks contain canonical `user_id`, `username`, `display_name`,
and `roles`. The session token itself is removed before forwarding to Pilot.

Text turns require a valid account session and never require Voiceprint. For
voice turns:

- voice guard off: use the canonical login identity and skip Voiceprint;
- voice guard on: Voiceprint must identify the same account above the
  configured threshold, otherwise the turn ends before Pilot, TTS, or action.

When Liaison is intentionally deployed without `keystone_endpoint`, the
existing `ROBONIX_LIAISON_ACCESS_*` allow-list policy remains available for
backward compatibility.

## Mock mode

Mock mode skips real microphone and ASR and uses preset text. It is useful for
checking Liaison/Pilot plumbing, but it is not a voice identity test.

```bash
ROBONIX_LIAISON_VOICE_MOCK=1 \
ROBONIX_LIAISON_VOICE_MOCK_TEXT="hello" \
robonix-liaison
```

## Environment Variables

| Variable | Default | Description |
|------|--------|------|
| `ROBONIX_ATLAS` | `127.0.0.1:50051` | Atlas address |
| `ROBONIX_PILOT_ENDPOINT` | `127.0.0.1:50071` | Pilot address |
| `ROBONIX_KEYSTONE_ENDPOINT` | (unset) | Keystone address; when set, account authentication is mandatory |
| `ROBONIX_LIAISON_PORT` | `50081` | Liaison listen port |
| `ROBONIX_LIAISON_VOICE_MOCK` | (unset) | Set to `1` to skip real mic+ASR and use preset text |
| `ROBONIX_LIAISON_VOICE_MOCK_TEXT` | `Hello, please introduce yourself.` | Text used in mock mode |
| `ROBONIX_LIAISON_SOURCE` | (unset) | Set to `text` to enable the stdin text loop (headless) |
| `ROBONIX_LIAISON_ACCESS_ENABLED` | `0` | Set to `1` to require access before Pilot/TTS/action |
| `ROBONIX_LIAISON_ALLOWED_USERS` | (empty) | Comma/space separated ids such as `local:alice,voice:alice` |
| `ROBONIX_LIAISON_VOICE_THRESHOLD` | `0.25` | Minimum voiceprint confidence for a voice user to pass |
| `ROBONIX_LIAISON_VOICE_SAVE_DIR` | (unset) | Directory for captured voice-session audio dumps |

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
| 1 | RECORDING_STARTED | Recording / streaming capture started |
| 2 | RECORDING_DONE | Recording finished |
| 3 | ASR_PARTIAL | ASR intermediate result |
| 4 | ASR_FINAL | ASR final result |
| 5 | USER_IDENTIFIED | Voiceprint recognition result |
| 6 | PILOT | Wrapped PilotEvent |
| 7 | TTS_STARTED | TTS started |
| 8 | TTS_DONE | TTS finished |
| 9 | SESSION_DONE | Session ended normally |
| 10 | ERROR | Error |
