# Voice Session

## 1. Overview of the New Feature

This change adds **voice interaction capabilities** to the robonix system. Previously, users could only interact with the system through
text input; now users can press `Ctrl+V` in the TUI, and the system automatically completes
the full pipeline of "record audio -> speech recognition -> intelligent reasoning -> speech synthesis".

## 2. Overall System Architecture and Flow

robonix uses a microservice architecture in which services communicate over gRPC, with Atlas handling service registration and discovery.

### Services Involved

| Service | Port | Responsibility |
|------|------|------|
| **Atlas** | :50051 | Control plane: service registration, discovery, endpoint allocation |
| **Liaison** | :50082 | User entry gateway: receives text/voice requests, orchestrates downstream calls |
| **Pilot** | :50071 | Reasoning engine: receives user intent, calls tools, generates replies (based on DeepSeek VLM) |
| **speech_service** | :dynamic | Speech service: Whisper ASR speech recognition + Edge TTS speech synthesis |
| **mock_audio** | :50091 | Hardware emulation: uses WAV files in place of a real microphone/speaker (for testing only) |

### Text Path (Enter)

The user types text in the TUI and presses Enter to send:

```
User (TUI)
  │
  │  Input "I want to query my current location"
  │
  ▼
SrvLiaison.Stream(Task)                    ← gRPC ① User→Liaison
  │
  │  Liaison assembles Task { text, user_id, session_id, ... }
  │
  ▼
SrvPilot.Stream(Task)                      ← gRPC ② Liaison→Pilot
  │
  │  Pilot parses intent → calls robot_state tool → generates reply
  │
  ▼
PilotEvent stream                          → gRPC ② returns
  │
  │  Liaison passes PilotEvent through to the TUI
  │
  ▼
TUI renders Pilot reply
```

Involves **2 gRPC calls**.

### Voice Path (Ctrl+V)

When the user presses Ctrl+V, Liaison internally orchestrates a 5-step gRPC call sequence:

```
User (TUI)
  │
  │  Press Ctrl+V
  │
  ▼
SrvLiaison.StartVoiceSession(req)          ← gRPC ① User→Liaison
  │
  │  ┌─────────── Liaison internal orchestration ──────────────────────┐
  │  │                                                                │
  │  │  Step 1: Recording                                            │
  │  │  PrmAudioMic.Stream()               ← gRPC ② Liaison→mock_audio
  │  │    mock_audio reads the WAV file, streams back PCM audio chunks│
  │  │    Returns: 86016 bytes PCM (16kHz mono s16le, ~2.69s)         │
  │  │                                                                │
  │  │  Step 2: Speech recognition                                   │
  │  │  SrvSpeechAsr.Call(audio_data)       ← gRPC ③ Liaison→speech_service
  │  │    speech_service uses Whisper to recognize the audio          │
  │  │    Returns: "Where is my current location" (confidence=0.9)    │
  │  │                                                                │
  │  │  Step 3: Reasoning                                            │
  │  │  SrvPilot.Stream(Task)              ← gRPC ④ Liaison→Pilot
  │  │    Task { text="Where is my current location", source=AUDIO,   │
  │  │           user_id="voice:liukaile", ... }                      │
  │  │    Pilot calls tools such as robot_state, streams back PilotEvent│
  │  │    Returns: location info + suggested actions (~300 chars)     │
  │  │                                                                │
  │  │  Step 4: Speech synthesis                                     │
  │  │  SrvSpeechTts.Call(text)            ← gRPC ⑤ Liaison→speech_service
  │  │    speech_service uses Edge TTS to synthesize the Pilot reply into audio│
  │  │    Returns: MP3 audio (292 chars → ~337KB)                     │
  │  │                                                                │
  │  │  Step 5: Playback/Save                                        │
  │  │  PrmAudioSpeaker.Stream(chunks)     ← gRPC ⑥ Liaison→mock_audio
  │  │    mock_audio saves the MP3 to /tmp/robonix_tts_output.mp3      │
  │  │                                                                │
  │  └────────────────────────────────────────────────────────────────┘
  │
  ▼
VoiceEvent stream                          → gRPC ① returns
  │
  │  After each step completes, Liaison sends a VoiceEvent to the TUI:
  │  SESSION_STARTED → RECORDING → ASR_FINAL → PILOT → TTS → DONE
  │
  ▼
TUI renders the progress of each step and the Pilot reply in real time
```

Involves **6 gRPC calls** (1 User→Liaison + 5 internal Liaison orchestration calls).

## 3. Services Actually Used in the TUI Test

The design principle of the `run_tui_test.sh` test script: **reuse the real services already
in the dev stack, and mock only the hardware**.

| Component | What it uses | Real? | Notes |
|------|----------|----------|------|
| Atlas | dev stack's robonix-atlas | ✓ Real | Service registration and discovery |
| Pilot | dev stack's robonix-pilot | ✓ Real | DeepSeek VLM reasoning, calls tools such as robot_state |
| ASR | dev stack's speech_service | ✓ Real | Whisper large-v3 model, real speech recognition |
| TTS | dev stack's speech_service | ✓ Real | Edge TTS (Microsoft), real speech synthesis |
| Microphone | mock_audio (new) | mock | WAV file emulates recording, replaces ALSA hardware |
| Speaker | mock_audio (new) | mock | Receives audio and writes to file, replaces ALSA hardware |
| Liaison | standalone instance (new) | ✓ Real | Added the StartVoiceSession RPC |

**Why mock the microphone/speaker?**
The development machine has no audio hardware (or it is a shared multi-user server), but the gRPC call chain still needs full testing.
mock_audio provides exactly the same gRPC interface as the real audio_driver
(`PrmAudioMic.Stream` / `PrmAudioSpeaker.Stream`), so downstream services cannot tell the difference.

**Why does Liaison use a standalone port?**
The dev stack already has a Liaison at :50081, but it does not support voice. The newly added Liaison runs at
:50082, and `ROBONIX_LIAISON_ENDPOINT` lets the TUI connect to it directly.

## 4. Verified Test Results

### Text Path

```
Input: "I want to query my current location"
Pilot reply: calls robot_state → returns coordinates (x≈0.00012, y≈0, heading≈0°)
Result: correctly identified as the start position
```

### Voice Path

```
WAV input: "Where is my current location" (synthesized by edge_tts, 16kHz PCM, 2.69s)
ASR recognition: gRPC → speech_service (Whisper) → recognizes the text
Pilot reasoning: calls robot_state/list_named_locations → returns location + known places
TTS synthesis: gRPC → speech_service (Edge TTS) → 292 chars → 337KB MP3
Speaker: gRPC → mock_audio → saves to /tmp/robonix_tts_output.mp3
```

### Pilot Failure Fallback

```
When Pilot is unreachable:
  Liaison does not interrupt the session
  Automatically returns "Information received successfully" as a mock reply
  The TUI displays normally, and the user can continue operating
```

## 5. How to Run the Test

```bash
# 1. Make sure the dev stack is started (Atlas + Pilot + speech_service + ...)
cd rust/examples
./run.sh

# 2. In another terminal, start the voice TUI test
cd rust/examples
./run_tui_test.sh

# 3. Operate in the TUI
#    Enter   → text input test
#    Ctrl+V  → voice input test (WAV → ASR → Pilot → TTS → file)
#    Ctrl+C  → exit
```

Customizing the test:
```bash
MOCK_WAV_TEXT="Navigate me to the kitchen" ./run_tui_test.sh   # different voice content
MOCK_WAV_INPUT=/path/to/recording.wav ./run_tui_test.sh        # custom WAV
MOCK_WAV_OUTPUT=~/tts_result.wav ./run_tui_test.sh             # specify output location
```

## 6. Switching from Mock Hardware to Real Hardware

Currently only the microphone and speaker are mocked. To switch:

**Real microphone**: use `audio_driver` (the ALSA driver) instead of mock_audio,
and remove the `ROBONIX_CHAT_MIC_NODE` pin.

**Real speaker**: same as above, remove the `ROBONIX_CHAT_SPEAKER_NODE` pin.
Note that TTS output is MP3, so the speaker must support MP3 playback.

**Fully real**: just use `./run.sh` to start the full stack (including audio_driver),
and Ctrl+V in the TUI will use the real microphone and speaker.

## 7. List of New/Modified Code

| File | Type | Notes |
|------|------|------|
| `robonix-liaison/src/voice.rs` | New | Voice session orchestrator (893 lines) |
| `robonix-liaison/src/main.rs` | Modified | Integrate StartVoiceSession RPC + Pilot fallback |
| `robonix-cli/src/cmd/chat.rs` | Modified | TUI Ctrl+V + direct Liaison connection + node pinning |
| `liaison.proto` | Modified | VoiceEvent / StartVoiceSession_Request messages |
| `pilot.proto` | Modified | Task.user_id field |
| `robonix_contracts.proto` | Modified | StartVoiceSession RPC / SrvSpeechVoiceprint |
| `voiceprint.proto` | New | Voiceprint recognition message definitions |
| `mock_audio.rs` | New | WAV-based mock microphone/speaker |
| `run_tui_test.sh` | New | One-click test script |
