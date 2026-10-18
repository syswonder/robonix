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
        │    ▲ robonix/system/liaison/voice/finish ends capture early
        ├─ robonix/service/speech/asr_stream
        ├─ optional robonix/service/voiceprint/identify
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
- `robonix/system/liaison/voice/finish`: ends microphone capture for an
  in-flight voice session and submits the transcript recognized so far.

All three contracts are registered by the `liaison` provider id.

A voice turn normally ends on the ASR backend's own end-of-utterance
detection. Continuous background noise can keep that from ever firing, which
strands the speaker until `record_seconds` expires, so `voice/finish` offers
the manual equivalent: capture stops, the ASR request stream is flushed, and
the turn continues down the normal submission path. It is not an abort —
dropping the `StartVoiceSession` stream is what discards a turn. Calling it
for a session that already ended returns `ok=false`, which callers should
treat as a benign race rather than an error.

When Keystone is configured, `voice/finish` requires a live Keystone session.
As with other controls, the authenticated account does not have to match the
account that started the Pilot session.

Clients should discover `voice/finish` through Atlas before offering a
finish-capture control: a Liaison older than this contract simply never
registers it, and the control belongs hidden rather than failing with
`UNIMPLEMENTED`.

## Identity and access

Pilot `Task` does not have a top-level `user_id` field. Liaison stores the
normalised identity in `Task.context_json.user_id`:

- text/API path: defaults to `local:<os_user>` when absent,
- voice path: uses `voice:<speaker_id>` from voiceprint when available.

When `ROBONIX_KEYSTONE_ENDPOINT` or `system.liaison.keystone_endpoint` is set,
external text, control, push-to-talk voice, and hands-free enable requests must
include a Keystone login token. Liaison retains the validated token while
hands-free mode is enabled and resolves the session before every voice capture.
`rbnx boot` turns this same setting into Executor's `require_keystone` mode so
both boundaries reject a missing session rather than allowing direct Executor
callers to bypass Liaison.
If the account's voice guard is disabled, the voice turn skips Voiceprint; if it
is enabled, Voiceprint must identify the logged-in account above Keystone's
configured threshold.
Liaison removes the token from `context_json`, replaces caller-supplied identity
fields with the Keystone account, and forwards the validated credential in the
internal `Task.auth_session_token` field. The local Liaison access policy is
used only when Keystone is not configured.

Manifest configuration cannot provide a login token. When Keystone is
configured, hands-free mode therefore starts disabled until an authenticated
client enables it.

Keystone does not bind a Pilot `session_id` to one account. An authenticated
user with an existing id may continue or control that session. Liaison's own
headless-session cleanup is forwarded internally without a login session.

When Keystone is not configured and `ROBONIX_LIAISON_ACCESS_ENABLED=1`:

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
| `ROBONIX_KEYSTONE_ENDPOINT` | (unset) | Keystone address; when set, external text, control, push-to-talk voice, and hands-free requests require a login session |
| `ROBONIX_SESSION_TOKEN` | (unset) | Keystone login token attached to text, control, and push-to-talk voice requests by `rbnx chat` |
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
