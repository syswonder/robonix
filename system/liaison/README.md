# robonix-liaison

统一的用户输入网关，支持文本和语音两种模态。

## 架构

```
用户
  │
  ├─ 文字输入 (Enter)
  │     │
  │     ▼
  │   SrvLiaison.Stream(Task) ──► Pilot
  │     │
  │     ▼
  │   PilotEvent stream ◄────────────┘
  │
  └─ 语音输入 (Ctrl+V in TUI)
        │
        ▼
      SrvLiaison.StartVoiceSession(req)
        │
        ├─ PrmAudioMic.Stream (录音 N 秒)
        ├─ SrvSpeechAsr.Call (语音识别)
        ├─ SrvSpeechVoiceprint.Call (声纹识别 → user_id)
        ├─ 组装 pilot::Task { user_id, text=transcript, … }
        ├─ SrvPilot.Stream
        ├─ (可选) SrvSpeechTts.Call + PrmAudioSpeaker.Stream
        │
        ▼
      VoiceEvent stream ◄───────────────────────────────────┘
```

## 主要改动

1. **`pilot::Task.user_id`** — 新增字段，Liaison 自动填充：
   - 文本路径：`local:<os_user>`
   - 语音路径：`voice:<id>` (来自 voiceprint) 或 fallback `voice:unknown`

2. **`SrvLiaison.StartVoiceSession`** — 新 RPC，全程编排语音对话。

3. **`rbnx chat` TUI** — 现在连接 Liaison 而非 Pilot；`Ctrl+V` 启动语音对话。

## 运行 Demo (Mock 模式)

无需真实 mic / ASR / TTS / VLM，使用预置文本验证端到端链路：

```bash
cd rust
./examples/voice_demo.sh
```

输出应显示 `text path → OK` 和 `voice path → OK`。

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ROBONIX_ATLAS` | `127.0.0.1:50051` | Atlas 地址 |
| `ROBONIX_PILOT_ENDPOINT` | `127.0.0.1:50071` | Pilot 地址 |
| `ROBONIX_LIAISON_PORT` | `50081` | Liaison 监听端口 |
| `ROBONIX_LIAISON_VOICE_MOCK` | (unset) | 设为 `1` 跳过 mic+ASR，使用预置文本 |
| `ROBONIX_LIAISON_VOICE_MOCK_TEXT` | `你好，请介绍一下你自己。` | mock 模式下使用的文本 |
| `ROBONIX_LIAISON_SOURCE` | (unset) | 设为 `text` 启用 stdin 文本循环 (headless) |

## TUI 快捷键

| 按键 | 功能 |
|------|------|
| `Enter` | 发送文本消息 |
| `Ctrl+V` | 开始语音对话 (默认 5 秒录音) |
| `Esc` | 中断当前回合 (abort_turn) |
| `Ctrl+C` | 退出 |
| `PageUp/PageDown` | 滚动历史 |

## 语音路径 VoiceEvent 类型

| kind | 名称 | 说明 |
|------|------|------|
| 0 | SESSION_STARTED | 会话开始 |
| 1 | RECORDING_STARTED | 开始录音 |
| 2 | RECORDING_DONE | 录音结束 |
| 3 | ASR_PARTIAL | ASR 中间结果 |
| 4 | ASR_FINAL | ASR 最终结果 |
| 5 | USER_IDENTIFIED | 声纹识别结果 |
| 6 | PILOT | 包装的 PilotEvent |
| 7 | TTS_STARTED | TTS 开始 |
| 8 | TTS_DONE | TTS 完成 |
| 9 | SESSION_DONE | 会话正常结束 |
| 10 | ERROR | 错误 |
