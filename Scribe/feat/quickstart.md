# 快速上手

[toc]

5 分钟跑起一个完整的 Tiago 仿真 + VLM 对话 demo。

## 1. 前置

需要：

- Linux x86_64 + Rust stable + Python ≥ 3.10
- Docker + Compose v2（仿真容器）
- 一个 OpenAI 兼容的 VLM API key（Qwen / GPT-4o / Gemini / Claude via 兼容网关 都行）

推荐：NVIDIA GPU + `nvidia-container-toolkit`（Webots 3D 渲染）；只跑对话不跑仿真可跳过 Docker。

## 2. 构建

```bash
git clone --recursive https://github.com/syswonder/robonix
cd robonix
make install
```

`make install` 会：
- 编译并把 `rbnx`、`robonix-atlas`、`robonix-pilot`、`robonix-executor`、`robonix-liaison`、`robonix-codegen` 装到 `~/.cargo/bin/`
- 自动登记当前 clone 为 robonix 源码根目录，让其他位置的包做 codegen 时能找到 contracts/IDL（见 [Build 与 Codegen](../integration-guide/build-and-codegen.md)）

> Python 依赖按包内的 `package_manifest.yaml` 自行管理；`rbnx start` 在 spawn driver 子进程前会把包的 `rbnx-build/codegen/proto_gen` 加进 `PYTHONPATH`。

## 3. 配 VLM

`rbnx boot` 通过环境变量读取 VLM endpoint（manifest 里以 `${VLM_*}` 形式引用）：

```bash
# OpenAI（或任意 OpenAI 兼容网关）
export VLM_API_KEY=sk-xxx
export VLM_BASE_URL=https://api.openai.com/v1
export VLM_MODEL=gpt-5.5

# Qwen（阿里 DashScope 提供 OpenAI 兼容网关）
export VLM_API_KEY=sk-xxx
export VLM_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
export VLM_MODEL=qwen3-vl-plus
```

把这几行写进 `~/.bashrc` / `~/.zshrc` 或部署目录的 `.env` 都可以。

## 4. 跑起来

整个栈分两个终端，仿真和 Robonix 系统服务/驱动各占一个：

```bash
# T1：仿真容器（Webots + ROS 2 + 3 个 driver，docker compose 栈，Ctrl-C 停）
# sim/start.sh 末尾会自动启动 rviz2，所以不需要手动开。
export DISPLAY=:0
bash examples/webots/sim/start.sh

# T2：Robonix 系统服务 + 系统/服务/技能包
export VLM_BASE_URL=https://api.openai.com/v1
export VLM_API_KEY=sk-...
export VLM_MODEL=gpt-5.5
cd examples/webots
rbnx boot
```

T1 不是 Robonix 包——它就是个 docker compose 栈。Robonix 不管它的生命周期。

T2 的 `rbnx boot` 读 `examples/webots/robonix_manifest.yaml`，按声明顺序拉起所有组件。当前 webots 部署一共 13 个：

- `system` 块（5）：`atlas` + `scene` + `executor` + `pilot` + `liaison`
- `primitive` 块（4）：`tiago_chassis` + `tiago_camera` + `tiago_lidar` + `audio_driver`
- `service` 块（3）：`mapping`（rtabmap 2D + RGBD fusion）+ `simple_nav`（Robonix 自家 A* + Pure-Pursuit，已替代 Nav2）+ `memory`（向量记忆检索）
- `skill` 块（1）：`explore`（frontier 自主探索）

driver 进程跑在仿真容器里（`docker exec`），系统服务（scene、mapping）跑在它们各自的 docker 容器里加入主机 DDS 总线。host 上不需要 ROS 2 环境。

`rbnx boot` 报告全部组件 up 后即可进入下一步。具体启动时序见 [系统部署与启动流程](../architecture/deployment-and-startup.md)。

> **scene 第一次跑要预热**：`scene` 容器构建时会拉 ~3 GB 的 torch+cu124 wheel、concept-graphs 源码，并预下 YOLO-World + MobileSAM 权重。第一次启 sim 之前先 `cd system/scene && bash scripts/build.sh` 把镜像建好（首次约 8–12 分钟）。
>
> 之后打开 <http://localhost:50107/> 看 scene 的 3 栏实时面板：左 = 2D occupancy + 物体定位，中 = 3D 点云 + bbox + Tiago 本体，右 = 相机 RGB + 深度直播流。

> **cache 注意**：`rbnx boot` 第一次会把 `mapping`、`explore` 这些 URL 远端包克隆到 `examples/webots/rbnx-boot/cache/`，以后默认走 cache。如果这两个仓库上游有更新，你需要手动 `cd examples/webots/rbnx-boot/cache/<pkg> && git pull`，或者直接 `rm -rf rbnx-boot/cache` 让下一次 boot 重新克隆——目前 boot 不会主动 fetch（`--build` / `--no-fetch` flag 在 backlog 上）。

## 5. 跟机器人对话

第三个终端：

```bash
rbnx caps          # 列出所有注册的能力提供者 + 其每条 capability
rbnx tools         # LLM 看到的工具列表（MCP transport 子集）
rbnx chat          # 直连 pilot 的 ratatui TUI
```

`rbnx chat` 里输入问题即可。典型一轮：

```
You:   what can you see?
Pilot: I'll capture a current RGB camera snapshot to see what's in view.
       > [r0] camera_snapshot({})
Pilot: The camera shows a potted plant near a beige wall …
```

按 **Esc** 中断当前推理（`AbortSession`）。退出 chat：`Ctrl+C`。

### 自主探索并构建语义地图

`explore` 是自带的 frontier 探索 skill，pilot 会把它当 MCP 工具调用：

```bash
# 一次性 prompt，事件流打到 stdout，state 终止时退出
rbnx ask "请彻底探索整个房间，调用 explore 后等待其完成，每 5 秒查 status，期间不要 cancel"
```

跑起来之后：
- `rbnx-boot/logs/service_explore.log` 每跳 frontier 打一行 `driving to frontier (x,y) size=...`，每个 sweep 步打 `sweep at (x,y) yaw=...°`
- `mapping_rbnx` 的 occupancy grid 会随机器人走过填空白
- `scene` 的 3 栏 web UI 实时反映：物体进 registry、点云累计、地图扩张

默认 Tiago/Webots 场景完成一次完整探索（约 6 个 frontier hops）约需 3–4 分钟。`rbnx ask` 默认 timeout 30 s，长任务用 `timeout 240 rbnx ask "..."` 延长超时，或改用 `rbnx chat` 交互式运行。

清栈：

```bash
bash examples/webots/sim/stop.sh   # 一键 kill 容器内 driver + rbnx boot + docker compose down
```

## 只起子集 / 调试

```bash
# 跳过 system 块（atlas/pilot 等已外部运行时）
rbnx boot --skip-system

# 单独起一个包（调试）
rbnx start -p ./primitives/tiago_chassis
```

## 运行时自省

栈跑起来之后：

```bash
rbnx caps                    # 所有注册的能力提供者及其 capability
rbnx tools                   # agent 可见的 MCP 工具
rbnx describe --provider <provider_id>   # 某个能力提供者的 CAPABILITY.md 全文
rbnx channels                # 当前活跃的 consumer→provider 通道
rbnx inspect                 # 完整 runtime 快照（JSON）
```

## 下一步

- [系统组件](../architecture/components.md)——12 个系统组件的职责与实现状态
- [接入指南](../integration-guide/index.md)——把自己的硬件或算法接入 Robonix
- [Build 与 Codegen](../integration-guide/build-and-codegen.md)——包作者必读（`rbnx setup`、`rbnx codegen`、自定义 contract）
- [接口目录](../interface-catalog/index.md)——`primitive/*` 原语与 `service/*` 服务的能力约定定义

## 常见问题

**Webots 没显示 GUI**：确认 `echo $DISPLAY` 非空，运行 `xhost +local:docker`。

**Webots 卡顿**：确认 `nvidia-smi` 可用且装了 `nvidia-container-toolkit`；否则跑在纯 CPU 软光栅上性能会显著下降。

**MCP 工具暂时不可见（`rbnx tools` 空）**：T1 仿真 + T2 `rbnx boot` 全部就绪需 ~10 s，等一会儿；如果一直空，看 `rbnx-boot/logs/<name>.log`。

**LLM 调工具被 422 拒绝**：driver 端 schema 与函数签名不一致。driver 应该用 `@<provider>.mcp(...)` 装饰器 + codegen 出来的 IO dataclass，schema 由能力约定自动决定，不要手写。详见[开发者指南 §14.9](../developer-guide.md#149-servicemcp--servicegrpc)。

**LLM 跑几轮就停了，但任务没完成**：Pilot 的 system prompt 已经包含 "persistence" 段落要求 LLM 持续迭代直到任务可验证完成；如果还停，多半是 LLM 模型本身倾向短回合（换更强的 reasoner，或者 prompt 里追加任务可验证条件）。

**VLM 报错但 pilot 未中断**：符合预期——错误以普通消息出现在 chat 中，session 不会中断，继续发送下一条即可。
