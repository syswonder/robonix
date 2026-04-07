<p align="center">
  <img src="images/robonix-logo.svg" alt="Robonix" width="420" />
</p>

<h3 align="center">Robonix — The Embodied AI Operating System</h3>

<p align="center">
  <em>Rust-native EAIOS: Robonix is a skill-centric embodied AI operating system composed of five core roles: Liaison, Pilot, Atlas, Nexus, and Executor, forming a complete pipeline from human intent to embodied action.</em>
</p>

<p align="center">
  <a href="https://github.com/syswonder/robonix/blob/main/LICENSE"><img src="https://img.shields.io/badge/license-MulanPSL--2.0-red?style=flat-square" alt="License" /></a>
  <a href="https://github.com/syswonder/robonix/graphs/contributors"><img src="https://img.shields.io/github/contributors/syswonder/robonix?color=blue&style=flat-square" alt="Contributors" /></a>
  <img src="https://img.shields.io/github/languages/code-size/syswonder/robonix?color=green&style=flat-square" alt="Code size" />
  <img src="https://img.shields.io/github/repo-size/syswonder/robonix?color=lightgray&style=flat-square" alt="Repo size" />
  <img src="https://img.shields.io/github/languages/top/syswonder/robonix?color=orange&style=flat-square" alt="Top language" />
</p>

<p align="center">
  <a href="docs/">Docs (mdBook)</a> ·
  <a href="rust/README.md">Rust workspace</a> ·
  <a href="rust/examples/README.md">Examples &amp; E2E</a>
</p>

<br />

<p align="center">
  <b>See it run</b> — Atlas, Pilot, Executor, and simulation wired together.
</p>

<br />

## Installation

### Prerequisites

- Rust toolchain (1.85+): `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`
- protoc: installed automatically via `protoc-bin-vendored` crate
- Python 3.10+ with `grpcio-tools` (for Python proto stubs and example packages)
- Docker (optional, for simulation stacks and containerized packages)

### Build & Install

```bash
git clone https://github.com/syswonder/robonix
cd robonix
git submodule update --init --recursive

cd rust
cargo build --workspace      # compile all crates
make install                 # install binaries to ~/.cargo/bin
```

`make install` runs `robonix-codegen` to regenerate protos, then installs all binaries:

| Binary | Description | Default port |
|--------|-------------|-------------|
| `robonix-atlas` | Control plane (registration, discovery, channel negotiation) | `:50051` |
| `robonix-executor` | Tool dispatch runtime (builtin / MCP / gRPC) | `:50061` |
| `robonix-pilot` | VLM reasoning service (ReAct loop, TaskGraph) | `:50071` |
| `robonix-liaison` | User-facing interaction layer (text → Intent → PilotEvent) | `:50081` |
| `rbnx` | CLI: package management, runtime inspection, chat TUI, topology graph | — |
| `robonix-codegen` | ROS IDL + contracts TOML → generated `.proto` files | — |

### Verify installation

```bash
rbnx --help
robonix-atlas --help
```

### Build targets

From `rust/`:

| Command | Description |
|---------|-------------|
| `make build` | Debug build (all crates) |
| `make release` | Release build |
| `make build-atlas` | Build Atlas only |
| `make install` | Regenerate protos + install all binaries |
| `make proto-interfaces` | Regenerate `robonix_proto/` from ROS IDL + contracts |
| `make fmt` | Format all Rust code |
| `make check` | Check formatting + clippy |
| `make clean` | Remove build artifacts |

## Running Robonix

### E2E demo (full stack with simulation)

```bash
cd rust
pip install -r examples/requirements.txt
cp examples/.env.example examples/.env   # set VLM_API_BASE, VLM_API_KEY, VLM_MODEL
./examples/run.sh
```

### What `run.sh` does

`run.sh` is the E2E demo orchestrator. It starts the full Robonix stack in sequence:

```
1. Validate Python deps (grpc, openai, mcp, memsearch, numpy, PIL, uvicorn)
2. Check X11 DISPLAY for GUI (Webots/rviz2)
3. Kill stale processes
4. rbnx validate + build for each example package
5. Start robonix-atlas (control plane, :50051)
6. rbnx start vlm_service        → VLM chat backend (:50105)
7. rbnx start memsearch_service  → semantic memory MCP server (:50105)
8. rbnx start tiago_sim_stack    → Docker Compose (Webots + Nav2 + rviz2 + MCP bridge)
9. Start robonix-executor        → tool dispatch (:50061)
10. Start robonix-pilot           → VLM reasoning (:50071)
11. Start robonix-liaison         → user interaction (:50081)
12. "stack ready — use 'rbnx chat' to connect"
```

Each component can be toggled via environment variables:

```bash
START_VLM_SERVICE=0 START_SIM_STACK=0 ./examples/run.sh   # runtime only, no sim
START_MEMSEARCH=0 ./examples/run.sh                        # skip memory service
```

### Manual startup (step by step)

```bash
cd rust

# 1. Start control plane
robonix-atlas &

# 2. Build and start packages
rbnx build -p examples/packages/vlm_service
rbnx start -p examples/packages/vlm_service -n com.robonix.services.vlm &

# 3. Start system services
robonix-executor &
robonix-pilot &
robonix-liaison &

# 4. Interact
rbnx chat
```

## `rbnx` CLI

`rbnx` is the primary CLI for package management and runtime inspection.

### Package commands

| Command | Description | Example |
|---------|-------------|---------|
| `rbnx validate <path>` | Validate `robonix_manifest.yaml` | `rbnx validate examples/packages/vlm_service` |
| `rbnx build -p <path>` | Run package build script | `rbnx build -p examples/packages/vlm_service` |
| `rbnx start -p <path> -n <node_id>` | Start a node from a package | `rbnx start -p ... -n com.robonix.services.vlm` |
| `rbnx install --github <repo>` | Install package from GitHub | `rbnx install --github user/repo` |
| `rbnx install --path <dir>` | Install package from local directory | `rbnx install --path ./my_package` |
| `rbnx list` | List installed packages | |
| `rbnx info <name>` | Show package details | `rbnx info vlm_service` |
| `rbnx config --show` | Show CLI configuration | |

### Runtime inspection

These commands query the running Atlas control plane. Use `--server <addr>` to override the default `localhost:50051`.

| Command | Description |
|---------|-------------|
| `rbnx nodes` | List all registered nodes and their interfaces |
| `rbnx describe` | Show SKILL.md content for registered nodes |
| `rbnx tools` | Print all tools visible to the agent (builtin + discovered) |
| `rbnx channels` | Show active negotiated channels |
| `rbnx inspect` | Dump full runtime state as JSON |
| `rbnx graph -o topology.png` | Generate topology graph (PNG or SVG with `--format svg`) |

### Interactive chat

```bash
rbnx chat                             # connect to localhost:50051
rbnx chat --server 192.168.1.10:50051 # remote Atlas
```

The chat TUI connects to Liaison → Pilot → Executor, allowing you to interact with the agent in natural language. The agent discovers available skills and tools from Atlas, reasons via VLM, and dispatches actions.

## Architecture

<p align="center">
  <img src="images/robonix-layers.png" alt="Robonix software architecture layers" width="580" />
</p>

Robonix follows the EAIOS four-layer abstraction — Primitive, Service, Skill, Task — with a unified control plane that handles registration, discovery, and channel negotiation across pluggable transports (gRPC, MCP, ROS 2, shared memory).

### Workspace crates

| Crate | Role |
|-------|------|
| `robonix-atlas` | Control plane: registration, discovery, channel negotiation, skill catalog, heartbeat, runtime inspection |
| `robonix-sdk` | Thin async Rust client for the control-plane API |
| `robonix-pilot` | VLM-driven reasoning: ReAct-style loop; streams `PilotEvent`; dispatches `TaskGraph` to Executor |
| `robonix-executor` | Tool dispatch: builtin / MCP / gRPC; `ExecutorService.Execute` |
| `robonix-liaison` | User-facing layer: text → `Intent` → `PilotEvent` stream |
| `robonix-cli` (`rbnx`) | Package validate / build / start; runtime inspection; `rbnx chat` (TUI); `rbnx graph` (topology PNG/SVG) |
| `robonix-codegen` | ROS IDL + `rust/contracts` TOML → `robonix_proto/` (generated `.proto`, incl. `robonix_contracts.proto`) |
| `robonix-buffer` | Shared-memory / buffer helpers for high-bandwidth data |

### Contracts & IDL

ROS payloads are canonical in `rust/crates/robonix-interfaces/lib/`. Stable `contract_id` paths live in `rust/contracts/**/*.toml` and match control-plane fields (`DeclareInterface` / `QueryNodes`). Regenerate protos:

```bash
cd rust
make proto-interfaces
# or directly:
cargo run -p robonix-codegen -- --lang proto \
  -I crates/robonix-interfaces/lib \
  --contracts contracts \
  -o crates/robonix-interfaces/robonix_proto
```

## Services & Packages

### System services

> [!NOTE]
> Robonix system services are still in early design phase regarding deployment and lifecycle management. Currently, system services (VLM, memory search, SLAM mapping, etc.) are deployed as **example packages** under `rust/examples/packages/` or as standalone repos. A formal service deployment framework is planned but not yet implemented.

The current system services:

| Service | Package location | Contract IDs | Transport |
|---------|-----------------|--------------|-----------|
| Pilot | built-in binary | `robonix/sys/runtime/pilot` | gRPC |
| Executor | built-in binary | `robonix/sys/runtime/executor` | gRPC |
| Liaison | built-in binary | `robonix/sys/runtime/liaison` | gRPC |
| VLM chat | `examples/packages/vlm_service` | `robonix/sys/model/vlm/chat` | gRPC |
| Memory search | `examples/packages/memsearch_service` | `robonix/sys/memory/{search,save,compact}` | MCP |
| SLAM mapping | [`enkerewpo/mapping_rbnx`](https://github.com/enkerewpo/mapping_rbnx) (standalone repo) | `robonix/prm/base/odom`, `robonix/sys/slam/*` | gRPC + ROS 2 |

### Example packages

| Package | Type | Purpose |
|---------|------|---------|
| `vlm_service` | Python gRPC | OpenAI-compatible VLM chat backend |
| `memsearch_service` | Python MCP | Semantic memory search/save/compact over Milvus |
| `tiago_sim_stack` | Docker Compose | Tiago robot sim: Webots + Nav2 + rviz2 + ROS 2 ↔ MCP bridge |
| `maniskill_vla_demo` | Python multi-node | ManiSkill3 VLA demo: Fetch in ReplicaCAD, Octo VLA |
| `zero_copy_demo` | Python + CUDA | Zero-copy shared memory / CUDA IPC demo |
| `clawhub_skills` | Python bridge | Import Agent Skills from OpenClaw ClawHub into Robonix |

## Agent Skills (agentskills.io)

Robonix natively supports the **[Agent Skills](https://agentskills.io/)** open standard for AI agent capabilities. This is the same format used by **30+ AI tools** including Claude Code, Cursor, VS Code, GitHub Copilot, OpenCode, Gemini CLI, and OpenClaw/ClawHub.

### Skill format

Each skill is a directory with a `SKILL.md` file containing YAML frontmatter + markdown body:

```
skills/
└── my-skill/
    ├── SKILL.md          # required: metadata + instructions
    ├── scripts/          # optional: executable code
    ├── references/       # optional: documentation
    └── assets/           # optional: templates, configs
```

```yaml
---
name: my-skill                           # required (1-64 chars, lowercase)
description: What it does and when       # required (1-1024 chars)
license: Apache-2.0                      # optional
compatibility: Requires Python 3.10+    # optional
---

# Instructions for the agent (markdown body)
Step-by-step procedures, examples, edge cases...
```

### How Robonix uses skills

1. **Discovery**: `rbnx start` scans the package's `skills/` directory, parses each `SKILL.md` frontmatter, and registers `name` + `description` with Atlas
2. **Activation**: Pilot examines skill descriptions at conversation time; when a user request matches semantically, the skill is activated
3. **Execution**: The full `SKILL.md` content is loaded and provided to the VLM as context for reasoning

### Importing skills from ClawHub

The `clawhub_skills` example package demonstrates importing community skills from [OpenClaw ClawHub](https://clawhub.ai/) (3000+ skills):

```bash
cd rust
rbnx build -p examples/packages/clawhub_skills    # pulls skills from openclaw/skills
rbnx start -p examples/packages/clawhub_skills -n com.robonix.skills.clawhub &
rbnx describe   # see imported skills
```

Skills from ClawHub, Claude Code, Cursor, or any Agent Skills-compatible source can be placed in a package's `skills/` directory and will be automatically discovered by Robonix.

### Abstract primitives

Interface IDs under `robonix/prm/*` cover camera, base, arm, gripper, manipulation, sensor, and force-torque. Implementations attach to real hardware or simulators by registering with Atlas. Full tables: **[docs/src/interface-catalog/](docs/src/interface-catalog/)**.

## Platforms & PRM

**Integrated demos** (see **[rust/examples/README.md](rust/examples/README.md)**):

| Platform | What you run | Notes |
|----------|----------------|-------|
| **Tiago** (PAL Robotics) | Docker Compose stack: Webots + Nav2 + rviz2 + `tiago_bridge` (ROS 2 ↔ MCP) | Requires Docker, X11; GPU recommended. |
| **ManiSkill3** (Fetch, ReplicaCAD) | Native Python: env / VLA / perception nodes + Atlas + Pilot | No host ROS 2; GPU recommended. |

## Project Status

> [!WARNING]
> Robonix is in an early, fast-moving development phase. Interfaces, IDL layouts, and internal designs may change without notice. No API stability is guaranteed until a versioned release is published.

### Available

- Control plane with multi-transport channels (gRPC, MCP, ROS 2, shared memory)
- **Pilot / Executor** path: VLM + tools + `rbnx chat` (TUI)
- SKILL.md for skill discovery and LLM-oriented behavior text
- Package system (`rbnx validate` / `build` / `start` / `install`)
- **`robonix-codegen`**: ROS `.msg` / `.srv` + contracts → generated `robonix_proto/`
- **Tiago** Webots E2E (`examples/run.sh` + `tiago_sim_stack`: Docker, Webots, Nav2, rviz2, ROS 2–MCP bridge)
- **ManiSkill3** VLA demo (`examples/packages/maniskill_vla_demo`: Fetch in ReplicaCAD, Octo VLA — see **[docs/src/getting-started/maniskill-demo.md](docs/src/getting-started/maniskill-demo.md)**)

### In Progress

- **System service deployment**: formal lifecycle management, systemd integration, declarative orchestration (services are currently example packages)
- **Liaison**: product-grade user-facing interaction
- Contract / namespace catalog enforcement on the server
- **TaskGraph**: behavior-tree / RTDL beyond today's linear wire encoding

### Documentation

**[docs/](docs/)** — mdBook: architecture, interface catalog (primitive / service), integration guides. Build: `cd docs && mdbook build`.

## Contributing

Contributions are welcome. Please:

1. Fork the repository
2. Create a feature branch from `dev`
3. Format code with `cd rust && make fmt`
4. Ensure `make check` passes (formatting + clippy)
5. Submit a pull request

## License

Mulan Permissive Software License, Version 2 (MulanPSL-2.0). See [LICENSE](LICENSE) for details.
