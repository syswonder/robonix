# Audio Driver

Audio driver -- Robonix primitive layer, automatically scans ALSA devices, provides gRPC streaming interfaces for microphone capture and speaker playback.

**This package also serves as a reference template for all future primitive driver packages.**

## Architecture Position

```
┌─────────────────────────────────────────────┐
│  speech_service  (robonix/service/speech)       │
│    ↕ gRPC: AudioChunk                       │
├─────────────────────────────────────────────┤
│  audio_driver    ← you are here (robonix/primitive/audio)  │
│    ↕ ALSA: arecord / aplay                  │
├─────────────────────────────────────────────┤
│         Hardware (USB Mic / Speaker)        │
└─────────────────────────────────────────────┘
```

- **Upper layer**: Provides audio streams to services like speech_service via gRPC
- **Lower layer**: Directly controls sound card hardware through the ALSA subsystem
- **Automation**: Automatically scans USB microphones/speakers at startup, no manual configuration needed

## gRPC Interfaces

| Service | RPC | Mode | Contract ID | Data Flow |
|---------|-----|------|-------------|-----------|
| PrmAudioMic | Stream | Server-stream | `robonix/primitive/audio/mic` | Microphone -> Caller |
| PrmAudioSpeaker | Stream | Client-stream | `robonix/primitive/audio/speaker` | Caller -> Speaker |

## Directory Structure

```
audio_driver/
├── rbnx-build/                 # Managed venv + generated contract modules
├── scripts/
│   ├── build.sh                # Venv sync + codegen + import smoke test
│   └── start.sh                # Managed-venv runtime entry
├── audio_driver/
│   ├── __init__.py             # Package entry point
│   ├── node.py                 # Main entry: Atlas registration + daemon threads + main()
│   ├── alsa_utils.py           # ALSA device scanning + driver registry
│   ├── mic_driver.py           # Microphone capture (arecord subprocess)
│   └── speaker_driver.py       # Speaker playback (aplay subprocess)
├── robonix_manifest.yaml       # Robonix package descriptor
├── requirements.txt
└── .gitignore
```

## Quick Start

### 1. Build

```bash
bash scripts/build.sh
```

This creates `rbnx-build/venv`, installs dependencies, runs `rbnx codegen`
with that interpreter, and verifies the generated imports. The host still
needs `arecord` and `aplay` from `alsa-utils`.

### 2. Start the Driver

```bash
bash scripts/start.sh
```

### 3. Launch via Robonix

```bash
rbnx run com.robonix.example.audio_driver
```

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `AUDIO_MIC_DEVICE` | Auto-detect | Microphone ALSA device (e.g. `hw:1,0`) |
| `AUDIO_MIC_SAMPLE_RATE` | `16000` | Capture sample rate (Hz) |
| `AUDIO_MIC_CHANNELS` | `1` | Capture channel count |
| `AUDIO_MIC_BITS` | `16` | Capture bit depth |
| `AUDIO_MIC_CHUNK_MS` | `100` | Chunk duration per read (milliseconds) |
| `AUDIO_MIC_PORT` | `0` (auto-assign) | Mic gRPC port |
| `AUDIO_SPEAKER_DEVICE` | Auto-detect | Speaker ALSA device |
| `AUDIO_SPEAKER_SAMPLE_RATE` | `24000` | Playback sample rate (Hz) |
| `AUDIO_SPEAKER_CHANNELS` | `1` | Playback channel count |
| `AUDIO_SPEAKER_BITS` | `16` | Playback bit depth |
| `AUDIO_SPEAKER_PORT` | `0` (auto-assign) | Speaker gRPC port |
| `AUDIO_DRIVER_STANDALONE` | — | Set to `1` to skip Atlas registration |
| `ROBONIX_ATLAS` | `localhost:50051` | Atlas control plane address |
| `ROBONIX_NODE_ID` | `com.robonix.primitive.audio` | Atlas node ID |

## Automatic Device Discovery

At startup, the following sequence is executed:

```
1. arecord -l  →  Parse input devices (microphones)
2. aplay -l    →  Parse output devices (speakers)
3. Merge & deduplicate  →  Same card/device marked as both is_input + is_output
4. Select defaults  →  Prefer USB devices (card >= 1), then any available device
5. Environment variable override  →  If AUDIO_MIC_DEVICE / AUDIO_SPEAKER_DEVICE is set, use specified value
```

## Driver Registry

Hardware-specific drivers can be added via a plugin mechanism:

```python
from audio_driver.alsa_utils import AudioDeviceDriver, register_driver

class RespeakerDriver(AudioDeviceDriver):
    """ReSpeaker multi-channel microphone array driver"""

    def detect(self, devices):
        return [d for d in devices if "ReSpeaker" in d.name]

    def name(self):
        return "ReSpeaker Driver"

register_driver(RespeakerDriver())
```

Built-in drivers:
- **DefaultAlsaDriver** -- Accepts all standard ALSA devices (registered by default)

## Module Descriptions

### mic_driver.py -- Microphone Capture

```
ALSA device → arecord subprocess → stdout → MicDriver.read_chunk() → AudioChunk dict
```

- Calls `arecord -D hw:X,Y -f S16_LE -r 16000 -c 1 -t raw`
- `read_chunk()` blocks reading a fixed-size byte count (default 100ms = 3200 bytes @ 16kHz mono s16le)
- Returns dict: `{timestamp_ns, data, sequence, duration_s}`

### speaker_driver.py -- Speaker Playback

```
gRPC AudioChunk → SpeakerDriver.play_chunk() → aplay stdin → ALSA device
```

- Calls `aplay -D hw:X,Y -f S16_LE -r 24000 -c 1 -t raw`
- Lazy start: aplay is not started until the first `play_chunk()` call
- Auto-restart: On BrokenPipeError, aplay is automatically restarted and the chunk is retried
- Thread-safe: Uses threading.Lock internally

### node.py -- Main Entry Point

Startup sequence (following the tiago_bridge pattern):

```
1. scan_alsa_devices()           → Discover hardware
2. find_default_mic/speaker()    → Select devices
3. MicDriver / SpeakerDriver()   → Create driver instances
4. auto-pick ports               → Auto-assign gRPC ports
5. RegisterPrimitive                  → Atlas registration
6. DeclareCapability × 2          → Declare mic + speaker capabilities
7. daemon threads                → Start heartbeat + 2 gRPC servers
8. main thread sleep             → Block and wait
```

## As a Reference Template

When creating a new primitive driver, copy this package and modify:

| Replace | This Package | Your Package |
|---------|-------------|-------------|
| Proto message types | AudioConfig, AudioChunk | Your hardware data types |
| gRPC services | PrmAudioMic, PrmAudioSpeaker | Your device interfaces |
| Contract IDs | robonix/primitive/audio/* | robonix/primitive/your_device/* |
| Scanning tools | arecord -l / aplay -l | Your device discovery method |
| Driver classes | MicDriver, SpeakerDriver | Your device drivers |
| Environment variable prefix | AUDIO_ | Your device prefix |
| Manifest node ID | com.robonix.primitive.audio | com.robonix.primitive.your_device |

Keep the managed build contract unchanged: prepare the package venv before
`rbnx codegen`, select its interpreter through both `PATH` and
`RBNX_CODEGEN_PYTHON`, and import-test generated modules before runtime.

## Atlas Integration

- **RegisterPrimitive**: `audio_driver`, namespace `robonix/primitive/audio`
- **DeclareCapability**: mic (server-stream) + speaker (client-stream)
- **Heartbeat**: Sends NodeHeartbeat every 15 seconds
- **Degradation**: Automatically runs standalone when Atlas is unavailable
