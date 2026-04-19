# Audio Driver

音频驱动 — Robonix prm 层，自动扫描 ALSA 设备，提供麦克风采集和扬声器播放的 gRPC 流式接口。

**本包同时作为所有未来 prm driver 包的参考模板。**

## 架构定位

```
┌─────────────────────────────────────────────┐
│  speech_service  (robonix/srv/speech)       │
│    ↕ gRPC: AudioChunk                       │
├─────────────────────────────────────────────┤
│  audio_driver    ← 你在这里 (robonix/prm/audio)  │
│    ↕ ALSA: arecord / aplay                  │
├─────────────────────────────────────────────┤
│         Hardware (USB Mic / Speaker)        │
└─────────────────────────────────────────────┘
```

- **上层**：通过 gRPC 向 speech_service 等服务提供音频流
- **下层**：通过 ALSA 子系统直接操控声卡硬件
- **自动化**：启动时自动扫描 USB 麦克风/扬声器，无需手动配置

## gRPC 接口

| 服务 | RPC | 模式 | Contract ID | 数据流 |
|------|-----|------|-------------|--------|
| PrmAudioMic | Stream | Server-stream | `robonix/prm/audio/mic` | 麦克风 → 调用方 |
| PrmAudioSpeaker | Stream | Client-stream | `robonix/prm/audio/speaker` | 调用方 → 扬声器 |

## 目录结构

```
audio_driver/
├── proto/
│   └── audio_driver.proto      # 自包含 gRPC 定义
├── proto_gen/                  # build 时生成的 *_pb2.py（git 忽略）
├── scripts/
│   └── build.sh                # proto 代码生成
├── audio_driver/
│   ├── __init__.py             # 包入口
│   ├── node.py                 # 主入口：Atlas 注册 + daemon 线程 + main()
│   ├── alsa_utils.py           # ALSA 设备扫描 + 驱动注册表
│   ├── mic_driver.py           # 麦克风采集（arecord subprocess）
│   └── speaker_driver.py       # 扬声器播放（aplay subprocess）
├── robonix_manifest.yaml       # Robonix 包描述
├── requirements.txt
└── .gitignore
```

## 快速启动

### 1. 安装依赖

```bash
pip install -r requirements.txt
# 核心依赖：grpcio, grpcio-tools, protobuf
# 系统依赖：arecord, aplay（alsa-utils 包）
```

### 2. 生成 proto stubs

```bash
bash scripts/build.sh
```

### 3. 启动驱动

```bash
# 正常模式（自动扫描设备 + Atlas 注册）
export PYTHONPATH=$(pwd)/proto_gen:${PYTHONPATH:-}
python -m audio_driver.node

# 独立模式（跳过 Atlas 注册）
AUDIO_DRIVER_STANDALONE=1 python -m audio_driver.node

# 指定设备（覆盖自动检测）
AUDIO_MIC_DEVICE=hw:1,0 AUDIO_SPEAKER_DEVICE=hw:0,0 python -m audio_driver.node
```

### 4. 通过 Robonix 启动

```bash
rbnx run com.robonix.example.audio_driver
```

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `AUDIO_MIC_DEVICE` | 自动检测 | 麦克风 ALSA 设备（如 `hw:1,0`） |
| `AUDIO_MIC_SAMPLE_RATE` | `16000` | 采集采样率（Hz） |
| `AUDIO_MIC_CHANNELS` | `1` | 采集声道数 |
| `AUDIO_MIC_BITS` | `16` | 采集位深 |
| `AUDIO_MIC_CHUNK_MS` | `100` | 每次 chunk 时长（毫秒） |
| `AUDIO_MIC_PORT` | `0`（自动分配） | Mic gRPC 端口 |
| `AUDIO_SPEAKER_DEVICE` | 自动检测 | 扬声器 ALSA 设备 |
| `AUDIO_SPEAKER_SAMPLE_RATE` | `24000` | 播放采样率（Hz） |
| `AUDIO_SPEAKER_CHANNELS` | `1` | 播放声道数 |
| `AUDIO_SPEAKER_BITS` | `16` | 播放位深 |
| `AUDIO_SPEAKER_PORT` | `0`（自动分配） | Speaker gRPC 端口 |
| `AUDIO_DRIVER_STANDALONE` | — | 设为 `1` 跳过 Atlas 注册 |
| `ROBONIX_ATLAS` | `localhost:50051` | Atlas 控制面地址 |
| `ROBONIX_NODE_ID` | `com.robonix.prm.audio` | Atlas 节点 ID |

## 设备自动发现

启动时执行以下流程：

```
1. arecord -l  →  解析输入设备（麦克风）
2. aplay -l    →  解析输出设备（扬声器）
3. 合并去重    →  同一 card/device 的设备标记 is_input + is_output
4. 选择默认    →  优先 USB 设备（card >= 1），其次任意可用设备
5. 环境变量覆盖 →  如果设置了 AUDIO_MIC_DEVICE / AUDIO_SPEAKER_DEVICE，使用指定值
```

## 驱动注册表

支持通过插件机制添加硬件专用驱动：

```python
from audio_driver.alsa_utils import AudioDeviceDriver, register_driver

class RespeakerDriver(AudioDeviceDriver):
    """ReSpeaker 多通道麦克风阵列驱动"""

    def detect(self, devices):
        return [d for d in devices if "ReSpeaker" in d.name]

    def name(self):
        return "ReSpeaker Driver"

register_driver(RespeakerDriver())
```

内置驱动：
- **DefaultAlsaDriver** — 接受所有标准 ALSA 设备（默认注册）

## 模块说明

### mic_driver.py — 麦克风采集

```
ALSA 设备 → arecord subprocess → stdout → MicDriver.read_chunk() → AudioChunk dict
```

- 调用 `arecord -D hw:X,Y -f S16_LE -r 16000 -c 1 -t raw`
- `read_chunk()` 阻塞读取固定大小字节（默认 100ms = 3200 bytes @ 16kHz mono s16le）
- 返回 dict: `{timestamp_ns, data, sequence, duration_s}`

### speaker_driver.py — 扬声器播放

```
gRPC AudioChunk → SpeakerDriver.play_chunk() → aplay stdin → ALSA 设备
```

- 调用 `aplay -D hw:X,Y -f S16_LE -r 24000 -c 1 -t raw`
- 惰性启动：首次 `play_chunk()` 时才启动 aplay
- 自动重启：BrokenPipeError 时自动重启 aplay 并重试
- 线程安全：内部使用 threading.Lock

### node.py — 主入口

启动流程（参照 tiago_bridge 模式）：

```
1. scan_alsa_devices()           → 发现硬件
2. find_default_mic/speaker()    → 选择设备
3. MicDriver / SpeakerDriver()   → 创建驱动实例
4. auto-pick ports               → 自动分配 gRPC 端口
5. RegisterNode                  → Atlas 注册
6. DeclareInterface × 2          → 声明 mic + speaker 接口
7. daemon threads                → 启动心跳 + 2 个 gRPC server
8. main thread sleep             → 阻塞等待
```

## 作为参考模板

新建 prm driver 时复制此包并修改：

| 替换项 | 本包值 | 你的包 |
|--------|--------|--------|
| proto 消息类型 | AudioConfig, AudioChunk | 你的硬件数据类型 |
| gRPC 服务 | PrmAudioMic, PrmAudioSpeaker | 你的设备接口 |
| Contract ID | robonix/prm/audio/* | robonix/prm/你的设备/* |
| 扫描工具 | arecord -l / aplay -l | 你的设备发现方式 |
| 驱动类 | MicDriver, SpeakerDriver | 你的设备驱动 |
| 环境变量前缀 | AUDIO_ | 你的设备前缀 |
| manifest node ID | com.robonix.prm.audio | com.robonix.prm.你的设备 |

保留不变的部分：
- `_ensure_proto_gen()` — proto stub 查找逻辑
- `_register_with_atlas()` — Atlas 注册 + 心跳
- daemon 线程模式 — 每个 interface 一个 gRPC server 线程
- `robonix_manifest.yaml` 结构
- `scripts/build.sh` proto 生成

## Atlas 集成

- **RegisterNode**: `com.robonix.prm.audio`，namespace `robonix/prm/audio`，kind `primitive`
- **DeclareInterface**: mic（server-stream）+ speaker（client-stream）
- **心跳**: 每 15 秒发送 NodeHeartbeat
- **降级**: Atlas 不可用时自动独立运行
