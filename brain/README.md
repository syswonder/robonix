# Brain

本目录负责系统的核心智能决策，包括任务解析、调度控制、LLM/VLM 推理及对 Memory 的读写操作。

## 功能模块：

- 指令解析与任务分解（调用大模型）
- Skill 编排与调度控制
- 与 Memory 和 Cost 模块交互
- 适配不同推理后端（Ollama, DeepSeek, ChatGLM…）

Brain 是系统的“中控”，体现具身智能的主动性。

