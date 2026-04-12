<p align="center">
  <img src="images/robonix-logo.svg" alt="Robonix" width="420" />
</p>

<h3 align="center">Robonix — 具身智能操作系统</h3>

<p align="center">
  <em>Rust 原生 EAIOS。以 skill 为中心的运行时：Liaison / Pilot / Atlas / Nexus / Executor —— 从人的意图到具身动作一条链路打通。</em>
</p>

<p align="center">
  <a href="https://github.com/syswonder/robonix/blob/main/LICENSE"><img src="https://img.shields.io/badge/license-MulanPSL--2.0-red?style=flat-square" alt="License" /></a>
  <a href="https://github.com/syswonder/robonix/graphs/contributors"><img src="https://img.shields.io/github/contributors/syswonder/robonix?color=blue&style=flat-square" alt="Contributors" /></a>
  <img src="https://img.shields.io/github/languages/code-size/syswonder/robonix?color=green&style=flat-square" alt="Code size" />
  <img src="https://img.shields.io/github/repo-size/syswonder/robonix?color=lightgray&style=flat-square" alt="Repo size" />
  <img src="https://img.shields.io/github/languages/top/syswonder/robonix?color=orange&style=flat-square" alt="Top language" />
</p>

<p align="center">
  <a href="README.md">English</a> ·
  <a href="docs/">手册 (mdBook)</a> ·
  <a href="rust/README.md">Rust workspace</a> ·
  <a href="rust/examples/README.md">示例与端到端</a>
</p>

<br />

<p align="center">
  <img src="images/demo_01_readme.gif" alt="Robonix demo" width="920" />
</p>

## 快速上手

```bash
git clone --recursive https://github.com/syswonder/robonix
cd robonix/rust
make install                              # → ~/.cargo/bin (rbnx, robonix-atlas, …)
pip install -r examples/requirements.txt
cp examples/.env.example examples/.env    # 填 VLM_API_KEY + VLM_BASE_URL + VLM_MODEL
./examples/run.sh                         # atlas + vlm + tiago sim + pilot + …
```

在另一个终端跟 agent 对话：

```bash
rbnx chat
```

完整首次运行指引见 [**docs/src/getting-started/quickstart.md**](docs/src/getting-started/quickstart.md)。

## 仓库概览

| | |
|---|---|
| [`rust/`](rust/) | Cargo 工作区 —— atlas / pilot / executor / liaison / cli / codegen / sdk / interfaces / buffer |
| [`rust/contracts/`](rust/contracts/) | 稳定 `contract_id` 定义（TOML），对应 `rust/crates/robonix-interfaces/lib/` 下的 ROS IDL |
| [`rust/examples/packages/`](rust/examples/packages/) | 示例包：`vlm_service`、`memsearch_service`、`tiago_sim_stack`、`maniskill_vla_demo`、`zero_copy_demo`、`clawhub_skills` |
| [`docs/`](docs/) | mdBook —— 架构、接口目录、接入指南 |
| [`images/`](images/) | Logo 与 demo 资产 |

## 30 秒架构

<p align="center">
  <img src="images/robonix-layers.png" alt="Robonix 分层架构" width="580" />
</p>

EAIOS 四层抽象：**原语 (primitive) → 服务 (service) → 技能 (skill) → 任务 (task)**。统一控制面（Atlas）负责注册、发现、通道协商，跨多种 transport 工作（gRPC / MCP / ROS 2 / 共享内存）。Agent 推理由 Pilot 负责（VLM + ReAct + TaskGraph），工具分发由 Executor 负责，用户交互由 Liaison 负责。

深入阅读：
- [**系统全景**](docs/src/architecture/overview.md) —— 控制面/数据面、一次请求的完整链路
- [**Crate 索引**](docs/src/architecture/crates.md) —— 各可执行文件职责与端口
- [**命名空间与接口模型**](docs/src/architecture/namespace-and-interfaces.md) —— `robonix/prm/*` 与 `robonix/srv/*` 怎么组织
- [**接口目录**](docs/src/interface-catalog/index.md) —— 每个 primitive + service 契约

## Agent Skills（agentskills.io）

Robonix 原生支持 [**Agent Skills**](https://agentskills.io/) 开放标准 —— 与 Claude Code、Cursor、GitHub Copilot、OpenCode、Gemini CLI、OpenClaw 等 30+ 个 AI 工具共用的 `SKILL.md` + frontmatter 格式。把 skill 放进包的 `skills/` 目录，`rbnx start` 会自动注册到 Atlas。详见 [**docs/src/skill-library.md**](docs/src/skill-library.md)。

## `rbnx` CLI

完整命令 `rbnx --help`。常用的：

```bash
rbnx setup                # 登记当前 clone 为源码根，让任意位置的包都能定位 contracts/IDL
rbnx build -p <pkg>       # 跑包的 build.sh（codegen + 其他）
rbnx codegen -p <pkg>     # 只做 codegen（proto + Python stubs）
rbnx start -p <pkg> -n <node>
rbnx chat                 # TUI → Pilot → Executor
rbnx nodes / tools / channels / inspect / graph
```

包作者请看 [**Build 与 Codegen**](docs/src/integration-guide/build-and-codegen.md)。

## 项目状态

> [!WARNING]
> 处于快速迭代的早期阶段。接口、IDL 布局、内部设计可能在没有预警的情况下变更。在发布正式版本之前不保证 API 稳定性。

已可用：Atlas 控制面、Pilot/Executor ReAct 循环、SKILL.md 发现、`rbnx` 包流程（`validate` / `build` / `codegen` / `start` / `install`）、Tiago Webots E2E、ManiSkill3 VLA demo。

进行中：系统服务的正式部署机制（当前仍以 example package 承载）、TaskGraph 超越线性编码、服务端的 catalog 强校验、产品级 Liaison。

## 参与贡献

从 `dev` 分支拉取，`make fmt`、`make check`、提 PR。若有 `CONTRIBUTING.md` 请参考。

## 许可证

Mulan Permissive Software License, Version 2 (MulanPSL-2.0)。详见 [LICENSE](LICENSE)。
