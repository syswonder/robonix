# Robonix 记忆系统环境配置与启动指南 (Ubuntu 22.04)

本指南包含了在您的 Ubuntu 22.04 服务器上从零开始编译和运行 Robonix 以及我们刚刚集成的 `memsearch` 记忆系统微服务的完整步骤。

## 1. 基础环境准备 (Ubuntu 22.04)

Robonix 的核心是用 Rust 编写的，而外围服务（如 VLM、MCP 和 memsearch）使用 Python。仿真栈需要 Docker。

### 1.1 安装系统依赖

```bash
sudo apt update
sudo apt install -y build-essential curl wget git libssl-dev pkg-config python3 python3-pip python3-venv
```

### 1.2 安装 Rust 工具链 (建议 stable >= 1.85)

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"
rustc --version
```

### 1.3 (可选但推荐) 安装 Docker 和 Docker Compose

如果您需要运行 Tiago 的仿真器（Webots），则需要安装 Docker。如果只跑 Agent 和记忆服务，可以跳过。

```bash
sudo apt install docker.io docker-compose-v2
sudo usermod -aG docker $USER
# 退出终端重新登录以使 docker 权限生效
```

***

## 2. 克隆项目与编译核心组件

进入项目目录（如果您已经 clone 好了，跳过 git clone 步骤）：

```bash
# git clone https://github.com/syswonder/robonix
cd robonix

# 确保拉取了子模块 (robonix-interfaces 等)
git submodule update --init --recursive

# 编译 Rust 工作空间
cd rust
cargo build --workspace

# (可选) 将 CLI 工具安装到 ~/.cargo/bin，方便全局使用 rbnx 命令
make install
```

***

## 3. 配置 Python 环境与依赖

我们将使用 Python 虚拟环境来避免污染系统 Python。

```bash
# 仍在 robonix/rust 目录下
python3 -m venv venv
source venv/bin/activate

# 升级 pip 以避免旧版 pip 找不到包的问题
pip install --upgrade pip

# 安装我们在 requirements.txt 中配置的依赖
# (包含了 memsearch[onnx], mcp, fastmcp, openai, grpcio 等)
pip install -r examples/requirements.txt
```

***

## 4. 环境变量配置

我们需要配置 VLM (大语言模型) 的 API Key。

```bash
# 仍在 robonix/rust 目录下
cp examples/.env.example examples/.env
```

编辑 `examples/.env` 文件，填入您的模型配置（例如 Qwen 或 DeepSeek）：

```bash
# 使用 vim 或 nano 编辑
nano examples/.env

# 填入类似如下的内容：
export VLM_API_KEY="sk-您的API_KEY"
export VLM_BASE_URL="https://dashscope.aliyuncs.com/compatible-mode/v1"
export VLM_MODEL="qwen3-vl-plus"

# 配置 HuggingFace 镜像源（解决国内下载 ONNX 向量模型慢的问题）
export HF_ENDPOINT="https://hf-mirror.com"
```

***

## 5. 一键启动测试

由于您的服务器是通过 SSH 连接，没有物理显示器，而且显卡是 MX250（属于入门级，跑 3D 渲染可能吃力），我们建议采用 **Headless（无界面）模式**，或者直接**关闭仿真栈**。

### 5.1 轻量化启动 (推荐：仅 Agent + 记忆系统，不启动仿真)

这是最稳定、最快的测试方式。它会完全跳过 Docker 和 Webots，只启动 Agent 和记忆服务，让您专注测试刚才接入的 `memsearch` 逻辑：

```bash
# 仍在 robonix/rust 目录下，确保激活了虚拟环境
source venv/bin/activate

# 禁用 Tiago 仿真栈，只启动 Agent、VLM 和 Memsearch
START_SIM_STACK=0 ./examples/run.sh
```

### 5.2 (可选) 完整启动并配置 X11 转发

如果您非要跑包含 Webots 和机器人的仿真栈，因为您没有接显示器，Docker 容器会因为找不到 X11 显示服务而报错崩溃（`Could not connect to display`）。

解决方法有两个：

**方法 A：使用 xvfb (虚拟显示帧缓冲) 跑无头模式 (Headless)**
我们需要修改 Docker 的启动入口，在容器内使用 `xvfb-run` 拦截所有的 GUI 渲染请求。
由于项目默认没有针对纯 Headless 模式进行打包配置，这需要您修改代码中的 `compose.yaml` 或入口脚本。**（不太推荐，对新手较麻烦）**

**方法 B：通过 SSH 开启 X11 Forwarding (将画面回传到您的电脑)**

1. 在您的**本地电脑 (macOS)** 上：
   安装 XQuartz：`brew install --cask xquartz`，然后打开它。
2. 在您的**本地电脑 (macOS)** 上，使用 `-X` 参数 SSH 连接您的 Ubuntu 服务器：

   ```bash
   ssh -X user@your_ubuntu_server_ip
   ```
3. 连上服务器后，检查 `DISPLAY` 环境变量是否自动配置好了：

   ```bash
   echo $DISPLAY
   # 应该输出类似 "localhost:10.0" 的内容
   ```
4. 运行 `docker` 之前，允许本地 X server 接收连接：

   ```bash
   xhost +local:docker
   ```
5. 然后启动全栈：

   ```bash
   ./examples/run.sh
   ```

*(注：由于 MX250 性能较弱，加上网络传画面的延迟，Webots 可能会非常卡顿，甚至只有 1 FPS。)*

***

## 6. 验证记忆系统是否生效

当看到屏幕上输出：

```text
robonix-agent ready. Type 'quit' to exit.
You: 
```

说明系统启动成功。

**测试步骤：**

1. 输入一条包含偏好的信息：
   `> 记住，我叫张三，我最喜欢的咖啡是冰美式。`
2. Agent 会调用大模型，大模型应该会自动触发 `save_memory` 工具将其存入本地 `.db`。此时终端会打印出调用 Tool 的相关日志。
3. 清除上下文（或者直接输入 `quit` 退出，重新启动 `./examples/run.sh`）。
4. 询问 Agent：
   `> 你还记得我叫什么名字，以及我喜欢喝什么吗？`
5. **验证记忆是否生效**：此时 Agent 在提问 VLM 之前，会触发底层的静默 `search_memory`。如果您在终端中看到了类似 `[mcp] tool search_memory returned: Relevant memories: ...` 的日志，且大模型准确回答出了“张三”和“冰美式”，说明您的记忆模块已经完美发挥作用！

> **注：首次运行时的模型下载**
> 第一次执行记忆保存或检索时，`memsearch` 会在后台从 HuggingFace 下载约 500MB 的 ONNX 模型文件 (`bge-m3-onnx-int8`)。因此第一次对话可能会感觉卡顿（等待下载），下载完成后后续对话将完全本地毫秒级响应。

