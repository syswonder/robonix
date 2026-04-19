# Speech Service

语音交互服务 — Robonix srv 层，提供 ASR（语音识别）、TTS（语音合成）、Dialog（语音对话）三类 gRPC 接口。

## 架构定位

```
┌─────────────────────────────────────────────┐
│              Application Layer              │
├─────────────────────────────────────────────┤
│  speech_service  ← 你在这里 (robonix/srv/speech)  │
├─────────────────────────────────────────────┤
│  audio_driver    (robonix/prm/audio)        │
├─────────────────────────────────────────────┤
│         Hardware (ALSA / USB Mic/Speaker)   │
└─────────────────────────────────────────────┘
```

- **上层**：接收应用层文本请求（TTS）或返回识别结果（ASR）
- **下层**：从 audio_driver 接收原始 PCM 音频流
- **自适应性**：自动将任意采样率/声道/编码转为 16kHz mono pcm_s16le，调用方无需预处理

## gRPC 接口

| 服务 | RPC | 模式 | 后端 | 说明 |
|------|-----|------|------|------|
| SpeechAsr | Recognize | Unary | Whisper (GPU FP16) | 一次性识别，适合完整语句 |
| SpeechAsr | RecognizeStream | Bidi-stream | FunASR Paraformer | 流式识别，600ms 粒度 |
| SpeechTts | Synthesize | Unary | Edge TTS | 一次性合成，返回完整 MP3 |
| SpeechTts | SynthesizeStream | Server-stream | Edge TTS | 流式合成，边生成边返回 |
| SpeechDialog | StartDialog | Server-stream | — | 语音对话会话管理 |

## 目录结构

```
speech_service/
├── proto/
│   └── speech_service.proto    # 自包含 gRPC 定义（内联 AudioConfig/AudioChunk）
├── proto_gen/                  # build 时生成的 *_pb2.py（git 忽略）
├── scripts/
│   └── build.sh                # proto 代码生成
├── speech_service/
│   ├── __init__.py             # 包入口
│   ├── service.py              # gRPC servicer + 后端引擎 + main()
│   └── audio_utils.py          # 音频格式自适应（任意格式 → 16kHz mono s16le）
├── robonix_manifest.yaml       # Robonix 包描述
├── requirements.txt
└── .gitignore
```

## 快速启动

### 1. 安装依赖

```bash
pip install -r requirements.txt
# 核心依赖：grpcio, grpcio-tools, transformers, torch, funasr, edge-tts, scipy, numpy
```

### 2. 生成 proto stubs

```bash
bash scripts/build.sh
```

### 3. 启动服务

```bash
# 正常模式（需要 GPU + 模型权重）
export PYTHONPATH=$(pwd)/proto_gen:${PYTHONPATH:-}
python -m speech_service.service

# CI 模式（不需要 GPU/模型，返回 mock 结果）
SPEECH_CI_MODE=1 python -m speech_service.service

# 指定端口
SPEECH_PORT=50060 python -m speech_service.service

# 跳过 Atlas 注册
SPEECH_STANDALONE=1 python -m speech_service.service
```

### 4. 通过 Robonix 启动

```bash
rbnx run com.robonix.example.speech_service
```

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ASR_MODEL` | `whisper-large-merged` | Whisper 模型路径 |
| `ASR_DEVICE` | `cuda` | Torch 设备（cuda/cpu） |
| `ASR_CHUNK_LENGTH` | `30.0` | Whisper 长音频分片长度（秒） |
| `ASR_BATCH_SIZE` | `4` | Whisper 批处理大小 |
| `FUNASR_MODEL` | `paraformer-zh-streaming` | FunASR 流式模型 |
| `FUNASR_CHUNK_SIZE` | `[0,10,5]` | Paraformer chunk_size 参数 |
| `TTS_VOICE` | `zh-CN-XiaoxiaoNeural` | Edge TTS 声音名称 |
| `ROBONIX_ATLAS` | `localhost:50051` | Atlas 控制面地址 |
| `SPEECH_PORT` | `0`（自动分配） | gRPC 监听端口 |
| `SPEECH_BIND_ADDR` | `0.0.0.0` | gRPC 绑定地址 |
| `SPEECH_CI_MODE` | — | 设为 `1` 启用 mock 模式 |

## 音频自适应

`audio_utils.py` 处理管线：

```
输入音频（任意格式）
  │
  ├─ 1. 解码 → float32 numpy [-1, 1]
  │     支持: pcm_s16le, pcm_f32le, pcm_s32le, pcm_u8, pcm_s24le, wav
  │
  ├─ 2. 多声道混合 → 单声道
  │     reshape(-1, channels).mean(axis=1)
  │
  ├─ 3. 重采样 → 16kHz
  │     scipy.signal.resample_poly（多相 FIR 滤波器）
  │
  └─ 4. 编码 → pcm_s16le
```

调用方无需关心音频格式，服务端自动处理 8kHz/16kHz/48kHz/stereo 等输入。

## Atlas 集成

启动时可选注册到 Atlas 控制面：

- **RegisterNode**: `com.robonix.services.speech`，namespace `robonix/srv/speech`，kind `service`
- **DeclareInterface × 5**: asr, asr_stream, tts, tts_stream, dialog

Atlas 不可用时自动降级为独立运行模式。

## Proto 设计

`speech_service.proto` 是**自包含**的 — AudioConfig 和 AudioChunk 类型内联定义，不依赖外部 proto 文件。与 `audio_driver.proto` 中的同名类型解耦，各自独立演进。
