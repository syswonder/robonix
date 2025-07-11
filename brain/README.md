# Brain

Central runtime of the embodied system. Responsible for:

- Interpreting commands
- Task decomposition into Skills
- Planning under Cost constraints
- Reading/writing from Memory
- Calling LLM/VLM for reasoning

## 🧠 Includes:

- World model abstraction
- Model interfaces (e.g., via Ollama or DeepSeek)
- Prompt construction (JIT / dynamic)
- Skill orchestration logic

This is the decision-making and planning center of the system.













Test CMD as follow :


配置好环境后（可运行requirements.txt见文件夹中）
先运行server端 + ros2节点端

```SHELL
python  path/to/brain-deepseek.py
```
