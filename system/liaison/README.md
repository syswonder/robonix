# robonix-liaison

Liaison is the user-input gateway in front of Pilot. It exposes one text/API
submission path and one push-to-talk voice path, normalises identity metadata,
applies optional access policy, then forwards accepted tasks to Pilot.

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
  │     │ normalize context_json.user_id
  │     │ optional access check
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
        ├─ robonix/service/voiceprint/identify
        ├─ voice access gate before Pilot/TTS/action
        ├─ build pilot::Task with text + context_json.user_id/access
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

Both contracts are registered by the `liaison` provider id.

## Identity and access

Pilot `Task` does not have a top-level `user_id` field. Liaison stores the
normalised identity in `Task.context_json.user_id`:

- text/API path: defaults to `local:<os_user>` when absent,
- voice path: uses `voice:<speaker_id>` from voiceprint when available.

When `ROBONIX_LIAISON_ACCESS_ENABLED=1`:

- text/API tasks must carry, or be normalised to, a user id listed in
  `ROBONIX_LIAISON_ALLOWED_USERS`;
- voice turns may capture audio and run voiceprint first, but cannot enter
  Pilot, TTS, or any robot action unless voiceprint identifies an enrolled
  speaker above `ROBONIX_LIAISON_VOICE_THRESHOLD` and `voice:<speaker_id>` is
  allowed;
- a client-provided user hint is audit metadata only for the voice path and
  cannot bypass voiceprint.

Accepted tasks also get `context_json.access` metadata so downstream logs can
distinguish allow-list and voiceprint grants.

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
