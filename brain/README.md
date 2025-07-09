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

```shell

# 创建项目目录
uv init mcp-client
cd mcp-client

# 创建虚拟环境
uv venv

# 激活虚拟环境
# 在 Windows 上：
.venv\Scripts\activate
# 在 Unix 或 MacOS 上：
source .venv/bin/activate

# 安装所需的包
uv add mcp anthropic python-dotenv "mcp[cli]" httpx


uv run client.py path/to/server.py



client.py path/to/server.py

```





