
# 🧠 Whisper + Xunfei ASR API 语音识别节点（目前基于ROS 2）

这是一个基于 OpenAI Whisper 和讯飞的asr API的语音识别节点，支持关键词触发、语音指令发布、TTS 联动，适用于 Jetson Orin 等边缘设备。

---

## 📦 功能简介

- 🎧 **监听语音**，默认每 3 秒录音一次
- 🔑 **关键词检测**：检测到如 `你好`、`开始` 等关键词后，触发长时间录音
- 🎙️ **录音并识别后续指令**（如“去抓取水杯”）
- 📤 **通过 ROS2 发布指令** 到 Topic：`voice_command`
- 🗣️ **调用 TTS 服务**（PlayText）播报提示

---

## 🚀 快速启动

### ✅ 环境准备（Jetson 或 x86 Linux）

#### 1. 安装依赖

```bash
# Whisper 依赖
pip install openai-whisper torch numpy

# ALSA 录音依赖
sudo apt install alsa-utils ffmpeg

# ROS 2 依赖（假设你已经配置好工作区）
source /opt/ros/humble/setup.bash
```


---

## 📂 节点说明

| 名称 | 类型 | 说明 |
|------|------|------|
| `/voice_command` | Topic `std_msgs/String` | 发布触发后的完整语音指令 |
| `/play_tts` | Service `piper_msgs/srv/PlayText` | 同步/异步播报提示文本 |
| `arecord` | 系统工具 | 用于从 USB 麦克风录音 |

---

## ⚠️ Jetson Orin 常见问题排查

### ❗ `torch.cuda.is_available() == False`
**问题**：Whisper 模型未能使用 GPU  
**解决方案**：
```bash
# 使用 JetPack 6.2 推荐的 PyTorch 安装方式
pip install --no-cache-dir torch torchvision torchaudio --index-url https://pypi.ngc.nvidia.com

# 设置 CUDA 环境变量
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

---

### ❗ `arecord: audio open error: No such file or directory`
**问题**：设备号错误或麦克风未连接  
**解决方案**：

1. 查看设备：
   ```bash
   arecord -l
   ```
2. 修改 `record_audio()` 中的设备名，例如：
   ```python
   cmd = ["arecord", "-D", "plughw:1,0", ...]
   ```

---

### ❗ `OSError: FLAC conversion utility not available`
**问题**：缺少 `flac` 工具（用于转录）  
**解决方案**：
```bash
sudo apt install flac
```

---

### ❗ 语音识别速度慢
**建议**：
- 使用 `small` 模型代替 `medium` 或 `large`
- 使用 GPU 加速（`torch.cuda.is_available()` 应为 `True`）
- 若非必须，限制录音时长（默认 3 秒）

---

## 🧪 测试方法

- 运行节点后，对着麦克风说出关键词，如“你好”
- 听到 TTS 回复后，说一句完整命令
- 该命令将被识别并通过 `voice_command` 发布

---

## 🧩 TODO / 可扩展方向

- 支持更多语言模型（如 `tiny`, `medium`, `large`）
- 增加 VAD（语音激活检测）避免固定录音时长
- 支持 Whisper Streaming（实时语音流转录）
- 自动检测麦克风设备编号（`arecord -l` 解析）

---
