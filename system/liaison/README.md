# robonix-liaison

`robonix-liaison` is the unified user-input gateway for Robonix — one of the 12 system components. It owns text and voice as two parallel input modalities and forwards the resulting tasks to `pilot`. Clients always talk to liaison, never to pilot directly.

## Architecture

```
user
  │
  ├─ text input (Enter)
  │     │
  │     ▼
  │   SrvLiaison.Stream(Task) ──► pilot
  │     │
  │     ▼
  │   PilotEvent stream ◄────────────┘
  │
  └─ voice input (Ctrl+V in TUI)
        │
        ▼
      SrvLiaison.StartVoiceSession(req)
        │
        ├─ PrmAudioMic.Stream            (record N seconds)
        ├─ SrvSpeechAsr.Call             (speech-to-text)
        ├─ SrvSpeechVoiceprint.Call      (identify speaker → user_id)
        ├─ build pilot::Task { user_id, text=transcript, … }
        ├─ SrvPilot.Stream
        ├─ (optional) SrvSpeechTts.Call + PrmAudioSpeaker.Stream
        │
        ▼
      VoiceEvent stream ◄───────────────────────────────────┘
```

Both paths populate `pilot::Task.user_id` so downstream policy (sentinel) and personalised pilot behaviour can act on who is talking:

- text path  → `local:<os_user>`
- voice path → `voice:<id>` from voiceprint, falling back to `voice:unknown`

## Demo (mock mode)

End-to-end run without real mic / ASR / TTS / VLM, using a pre-canned transcript:

```bash
bash system/liaison/examples/voice_demo.sh
```

Expected output: `text path → OK` followed by `voice path → OK`.

## Environment variables

| Variable                          | Default                              | Purpose |
| ---                               | ---                                  | --- |
| `ROBONIX_ATLAS`                   | `127.0.0.1:50051`                    | Atlas endpoint |
| `ROBONIX_PILOT_ENDPOINT`          | `127.0.0.1:50071`                    | Pilot endpoint |
| `ROBONIX_LIAISON_PORT`            | `50081`                              | Liaison gRPC listen port |
| `ROBONIX_LIAISON_VOICE_MOCK`      | (unset)                              | Set to `1` to skip mic+ASR and use the canned transcript |
| `ROBONIX_LIAISON_VOICE_MOCK_TEXT` | `Hello, please introduce yourself.`  | The canned transcript used when `_VOICE_MOCK=1` |
| `ROBONIX_LIAISON_SOURCE`          | (unset)                              | Set to `text` for a headless stdin text loop |

## TUI shortcuts (`rbnx chat`)

| Key                | Action |
| ---                | --- |
| `Enter`            | Send a text message |
| `Ctrl+V`           | Start a voice turn (default: 5 s of mic capture) |
| `Esc`              | Abort the current turn (`abort_turn`) |
| `Ctrl+C`           | Quit |
| `PageUp`/`PageDown`| Scroll the history view |

## `VoiceEvent` kinds (voice path)

| kind | Name               | Meaning |
| ---  | ---                | --- |
| 0    | `SESSION_STARTED`  | Session opened |
| 1    | `RECORDING_STARTED`| Mic capture started |
| 2    | `RECORDING_DONE`   | Mic capture finished |
| 3    | `ASR_PARTIAL`      | ASR partial result |
| 4    | `ASR_FINAL`        | ASR final transcript |
| 5    | `USER_IDENTIFIED`  | Voiceprint identity decision |
| 6    | `PILOT`            | Wrapped `PilotEvent` |
| 7    | `TTS_STARTED`      | TTS playback started |
| 8    | `TTS_DONE`         | TTS playback finished |
| 9    | `SESSION_DONE`     | Session ended normally |
| 10   | `ERROR`            | Session ended with an error |
