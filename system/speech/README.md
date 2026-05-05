# Speech Service

Voice interaction service -- Robonix srv layer, providing three types of gRPC interfaces: ASR (Automatic Speech Recognition), TTS (Text-to-Speech), and Dialog (Voice Dialog).

## Architecture Position

```
┌─────────────────────────────────────────────┐
│              Application Layer              │
├─────────────────────────────────────────────┤
│  speech_service  ← you are here (robonix/service/speech)  │
├─────────────────────────────────────────────┤
│  audio_driver    (robonix/primitive/audio)        │
├─────────────────────────────────────────────┤
│         Hardware (ALSA / USB Mic/Speaker)   │
└─────────────────────────────────────────────┘
```

- **Upper layer**: Receives text requests from applications (TTS) or returns recognition results (ASR)
- **Lower layer**: Receives raw PCM audio streams from audio_driver
- **Adaptability**: Automatically converts any sample rate/channel/encoding to 16kHz mono pcm_s16le -- callers do not need to pre-process audio

## gRPC Interfaces

| Service | RPC | Mode | Backend | Description |
|---------|-----|------|---------|-------------|
| SpeechAsr | Recognize | Unary | Whisper (GPU FP16) | One-shot recognition, suitable for complete utterances |
| SpeechAsr | RecognizeStream | Bidi-stream | FunASR Paraformer | Streaming recognition, 600ms granularity |
| SpeechTts | Synthesize | Unary | Edge TTS | One-shot synthesis, returns complete MP3 |
| SpeechTts | SynthesizeStream | Server-stream | Edge TTS | Streaming synthesis, yields chunks as generated |
| SpeechDialog | StartDialog | Server-stream | — | Voice dialog session management |

## Directory Structure

```
speech_service/
├── proto/
│   └── speech_service.proto    # Self-contained gRPC definitions (inline AudioConfig/AudioChunk)
├── proto_gen/                  # Generated *_pb2.py at build time (git-ignored)
├── scripts/
│   └── build.sh                # Proto code generation
├── speech_service/
│   ├── __init__.py             # Package entry point
│   ├── service.py              # gRPC servicer + backend engines + main()
│   └── audio_utils.py          # Audio format adaptation (any format → 16kHz mono s16le)
├── robonix_manifest.yaml       # Robonix package descriptor
├── requirements.txt
└── .gitignore
```

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
# Core dependencies: grpcio, grpcio-tools, transformers, torch, funasr, edge-tts, scipy, numpy
```

### 2. Generate Proto Stubs

```bash
bash scripts/build.sh
```

### 3. Start the Service

```bash
# Normal mode (requires GPU + model weights)
export PYTHONPATH=$(pwd)/proto_gen:${PYTHONPATH:-}
python -m speech_service.service

# CI mode (no GPU/model needed, returns mock results)
SPEECH_CI_MODE=1 python -m speech_service.service

# Specify port
SPEECH_PORT=50060 python -m speech_service.service

# Skip Atlas registration
SPEECH_STANDALONE=1 python -m speech_service.service
```

### 4. Launch via Robonix

```bash
rbnx run com.robonix.example.speech_service
```

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ASR_MODEL` | `whisper-large-merged` | Whisper model path |
| `ASR_DEVICE` | `cuda` | Torch device (cuda/cpu) |
| `ASR_CHUNK_LENGTH` | `30.0` | Whisper long-audio chunk length (seconds) |
| `ASR_BATCH_SIZE` | `4` | Whisper batch size |
| `FUNASR_MODEL` | `paraformer-zh-streaming` | FunASR streaming model |
| `FUNASR_CHUNK_SIZE` | `[0,10,5]` | Paraformer chunk_size parameter |
| `TTS_VOICE` | `zh-CN-XiaoxiaoNeural` | Edge TTS voice name |
| `ROBONIX_ATLAS` | `localhost:50051` | Atlas control plane address |
| `SPEECH_PORT` | `0` (auto-assign) | gRPC listen port |
| `SPEECH_BIND_ADDR` | `0.0.0.0` | gRPC bind address |
| `SPEECH_CI_MODE` | — | Set to `1` to enable mock mode |

## Audio Adaptation

`audio_utils.py` processing pipeline:

```
Input audio (any format)
  │
  ├─ 1. Decode → float32 numpy [-1, 1]
  │     Supported: pcm_s16le, pcm_f32le, pcm_s32le, pcm_u8, pcm_s24le, wav
  │
  ├─ 2. Multi-channel downmix → mono
  │     reshape(-1, channels).mean(axis=1)
  │
  ├─ 3. Resample → 16kHz
  │     scipy.signal.resample_poly (polyphase FIR filter)
  │
  └─ 4. Encode → pcm_s16le
```

Callers do not need to worry about audio format -- the server automatically handles inputs such as 8kHz/16kHz/48kHz/stereo.

## Atlas Integration

Optionally registers with the Atlas control plane at startup:

- **RegisterCapability** (legacy `RegisterNode` shim): `com.robonix.system.speech`, namespace `robonix/system/speech`
- **DeclareInterface × 5**: asr, asr_stream, tts, tts_stream, dialog

Automatically degrades to standalone mode when Atlas is unavailable.

## Proto Design

`speech_service.proto` is **self-contained** -- AudioConfig and AudioChunk types are defined inline, with no dependency on external proto files. They are decoupled from the identically-named types in `audio_driver.proto`, allowing each to evolve independently.
