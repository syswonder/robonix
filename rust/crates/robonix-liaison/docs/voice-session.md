# 语音交互功能 (Voice Session)

## 一、新增功能概述

本次在 robonix 系统中新增了 **语音交互能力**。在此之前，用户只能通过
文本输入与系统交互；现在用户可以在 TUI 界面按 `Ctrl+V`，系统自动完成
"录音 → 语音识别 → 智能推理 → 语音合成" 的全流程。

## 二、系统整体架构与链路

robonix 采用微服务架构，各服务之间通过 gRPC 通信，由 Atlas 做服务注册和发现。

### 涉及的服务

| 服务 | 端口 | 职责 |
|------|------|------|
| **Atlas** | :50051 | 控制面板：服务注册、发现、端点分配 |
| **Liaison** | :50082 | 用户入口网关：接收文本/语音请求，编排下游调用 |
| **Pilot** | :50071 | 推理引擎：接收用户意图，调用工具，生成回复（基于 DeepSeek VLM） |
| **speech_service** | :动态 | 语音服务：Whisper ASR 语音识别 + Edge TTS 语音合成 |
| **mock_audio** | :50091 | 硬件模拟：用 WAV 文件替代真实麦克风/扬声器（仅测试用） |

### 文本路径（Enter）

用户在 TUI 输入文本，按 Enter 发送：

```
用户 (TUI)
  │
  │  输入 "我想要查询我当前所在的位置"
  │
  ▼
SrvLiaison.Stream(Task)                    ← gRPC ① 用户→Liaison
  │
  │  Liaison 组装 Task { text, user_id, session_id, ... }
  │
  ▼
SrvPilot.Stream(Task)                      ← gRPC ② Liaison→Pilot
  │
  │  Pilot 解析意图 → 调用 robot_state 工具 → 生成回复
  │
  ▼
PilotEvent 流                              → gRPC ② 返回
  │
  │  Liaison 透传 PilotEvent 给 TUI
  │
  ▼
TUI 渲染 Pilot 回复
```

涉及 **2 次 gRPC 调用**。

### 语音路径（Ctrl+V）

用户按 Ctrl+V，Liaison 内部自动编排 5 步 gRPC 调用：

```
用户 (TUI)
  │
  │  按 Ctrl+V
  │
  ▼
SrvLiaison.StartVoiceSession(req)          ← gRPC ① 用户→Liaison
  │
  │  ┌─────────── Liaison 内部编排 ──────────────────────────────────┐
  │  │                                                                │
  │  │  Step 1: 录音                                                  │
  │  │  PrmAudioMic.Stream()               ← gRPC ② Liaison→mock_audio
  │  │    mock_audio 读取 WAV 文件, 流式返回 PCM 音频块               │
  │  │    返回: 86016 bytes PCM (16kHz mono s16le, ~2.69s)            │
  │  │                                                                │
  │  │  Step 2: 语音识别                                              │
  │  │  SrvSpeechAsr.Call(audio_data)       ← gRPC ③ Liaison→speech_service
  │  │    speech_service 用 Whisper 识别音频                          │
  │  │    返回: "我目前所在的位置是哪里" (confidence=0.9)              │
  │  │                                                                │
  │  │  Step 3: 推理                                                  │
  │  │  SrvPilot.Stream(Task)              ← gRPC ④ Liaison→Pilot
  │  │    Task { text="我目前所在的位置是哪里", source=AUDIO,          │
  │  │           user_id="voice:liukaile", ... }                      │
  │  │    Pilot 调用 robot_state 等工具，流式返回 PilotEvent          │
  │  │    返回: 位置信息 + 操作建议（~300 字）                        │
  │  │                                                                │
  │  │  Step 4: 语音合成                                              │
  │  │  SrvSpeechTts.Call(text)            ← gRPC ⑤ Liaison→speech_service
  │  │    speech_service 用 Edge TTS 将 Pilot 回复合成为音频          │
  │  │    返回: MP3 音频 (292 chars → ~337KB)                         │
  │  │                                                                │
  │  │  Step 5: 播放/保存                                             │
  │  │  PrmAudioSpeaker.Stream(chunks)     ← gRPC ⑥ Liaison→mock_audio
  │  │    mock_audio 将 MP3 保存到 /tmp/robonix_tts_output.mp3        │
  │  │                                                                │
  │  └────────────────────────────────────────────────────────────────┘
  │
  ▼
VoiceEvent 流                              → gRPC ① 返回
  │
  │  每完成一步，Liaison 发送一个 VoiceEvent 给 TUI：
  │  SESSION_STARTED → RECORDING → ASR_FINAL → PILOT → TTS → DONE
  │
  ▼
TUI 实时渲染每一步进度和 Pilot 回复
```

涉及 **6 次 gRPC 调用**（1 次用户→Liaison + 5 次 Liaison 内部编排）。

## 三、TUI 测试实际使用的服务

`run_tui_test.sh` 测试脚本的设计原则：**复用 dev 栈已有的真实服务，只
mock 硬件**。

| 组件 | 用的什么 | 是否真实 | 说明 |
|------|----------|----------|------|
| Atlas | dev 栈的 robonix-atlas | ✓ 真实 | 服务注册发现 |
| Pilot | dev 栈的 robonix-pilot | ✓ 真实 | DeepSeek VLM 推理，调用 robot_state 等工具 |
| ASR | dev 栈的 speech_service | ✓ 真实 | Whisper large-v3 模型，真实语音识别 |
| TTS | dev 栈的 speech_service | ✓ 真实 | Edge TTS (微软)，真实语音合成 |
| 麦克风 | mock_audio (本次新增) | mock | WAV 文件模拟录音，替代 ALSA 硬件 |
| 扬声器 | mock_audio (本次新增) | mock | 接收音频写入文件，替代 ALSA 硬件 |
| Liaison | 独立实例 (本次新增) | ✓ 真实 | 新增了 StartVoiceSession RPC |

**为什么要 mock 麦克风/扬声器？**
开发机没有音频硬件（或多用户共享服务器），但 gRPC 调用链路需要完整测试。
mock_audio 提供与真实 audio_driver 完全相同的 gRPC 接口
（`PrmAudioMic.Stream` / `PrmAudioSpeaker.Stream`），下游服务无法区分。

**为什么 Liaison 用独立端口？**
dev 栈已有一个 Liaison 在 :50081，但不支持语音。本次新增的 Liaison 在
:50082 运行，通过 `ROBONIX_LIAISON_ENDPOINT` 让 TUI 直连。

## 四、已测试验证的结果

### 文本路径

```
输入: "我想要查询我当前所在的位置"
Pilot 回复: 调用 robot_state → 返回坐标 (x≈0.00012, y≈0, heading≈0°)
结果: 正确识别为 start 位置
```

### 语音路径

```
WAV 输入: "我目前所在的位置是哪里" (edge_tts 合成, 16kHz PCM, 2.69s)
ASR 识别: gRPC → speech_service (Whisper) → 识别出文本
Pilot 推理: 调用 robot_state/list_named_locations → 返回位置 + 已知地点
TTS 合成: gRPC → speech_service (Edge TTS) → 292 chars → 337KB MP3
Speaker: gRPC → mock_audio → 保存到 /tmp/robonix_tts_output.mp3
```

### Pilot 故障回退

```
当 Pilot 不可达时:
  Liaison 不中断会话
  自动返回 "成功接收信息" 作为 mock 回复
  TUI 正常显示，用户可继续操作
```

## 五、如何运行测试

```bash
# 1. 确保 dev 栈已启动 (Atlas + Pilot + speech_service + ...)
cd rust/examples
./run.sh

# 2. 在另一个终端启动语音 TUI 测试
cd rust/examples
./run_tui_test.sh

# 3. 在 TUI 中操作
#    Enter   → 文本输入测试
#    Ctrl+V  → 语音输入测试 (WAV → ASR → Pilot → TTS → 文件)
#    Ctrl+C  → 退出
```

自定义测试：
```bash
MOCK_WAV_TEXT="帮我导航到厨房" ./run_tui_test.sh          # 不同语音内容
MOCK_WAV_INPUT=/path/to/recording.wav ./run_tui_test.sh   # 自定义 WAV
MOCK_WAV_OUTPUT=~/tts_result.wav ./run_tui_test.sh        # 指定输出位置
```

## 六、从 mock 硬件切换到真实硬件

当前 mock 的只有麦克风和扬声器。切换方式：

**真实麦克风**：用 `audio_driver`（ALSA 驱动）替代 mock_audio，
取消 `ROBONIX_CHAT_MIC_NODE` 固定。

**真实扬声器**：同上，取消 `ROBONIX_CHAT_SPEAKER_NODE` 固定。
注意 TTS 输出为 MP3，扬声器需支持 MP3 播放。

**完全真实**：直接用 `./run.sh` 启动全栈（含 audio_driver），
TUI 中 Ctrl+V 使用真实麦克风和扬声器。

## 七、新增/修改代码清单

| 文件 | 类型 | 说明 |
|------|------|------|
| `robonix-liaison/src/voice.rs` | 新增 | 语音会话编排器 (893行) |
| `robonix-liaison/src/main.rs` | 修改 | 集成 StartVoiceSession RPC + Pilot 回退 |
| `robonix-cli/src/cmd/chat.rs` | 修改 | TUI Ctrl+V + 直连 Liaison + 节点固定 |
| `liaison.proto` | 修改 | VoiceEvent / StartVoiceSession_Request 消息 |
| `pilot.proto` | 修改 | Task.user_id 字段 |
| `robonix_contracts.proto` | 修改 | StartVoiceSession RPC / SrvSpeechVoiceprint |
| `voiceprint.proto` | 新增 | 声纹识别消息定义 |
| `mock_audio.rs` | 新增 | WAV 模拟麦克风/扬声器 |
| `run_tui_test.sh` | 新增 | 一键测试脚本 |
