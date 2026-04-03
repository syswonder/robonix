<p align="center">
  <img src="images/robonix-logo.svg" alt="Robonix" width="420" />
</p>

<h3 align="center">Robonix - The Embodied Intelligence Operating System</h3>

<p align="center">
  <em>A unified OS platform for robots, agents, and heterogeneous hardware — communication and scheduling in one control plane.</em>
</p>

<p align="center">
  <strong>Rust-native EAIOS</strong> (Embodied AI Operating System) — register primitives and services once, negotiate transports at runtime, discover them everywhere.
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
  <b>See it run</b> — Atlas, Pilot, Executor, and simulation wired together; <b>Liaison</b> interaction is still a work in progress.
</p>
<p align="center">
  <img src="images/demo_01_readme.gif" alt="Robonix demo: runtime stack in action" width="920" />
</p>

<br />

## Quick start

```bash
git clone https://github.com/syswonder/robonix
cd robonix
git submodule update --init --recursive
cd rust
cargo build --workspace
make install          # rbnx, robonix-codegen, pilot, executor, liaison, atlas wrapper → ~/.cargo/bin
```

**E2E demo** (Docker, X11, VLM API key):

```bash
cd rust
cp examples/.env.example examples/.env   # VLM_API_BASE, VLM_API_KEY, VLM_MODEL
./examples/run.sh
```

**Without simulation** (Atlas + Pilot + Executor + VLM + Liaison process; Liaison interaction still evolving):

```bash
cd rust
START_SIM_STACK=0 ./examples/run.sh
```

More options: **[rust/README.md](rust/README.md)** · walkthrough **[rust/examples/README.md](rust/examples/README.md)**

---

> [!WARNING]
> Robonix is in an early, fast-moving development phase. Interfaces, IDL layouts, and internal designs may change without notice. No API stability is guaranteed until a versioned release is published.

## Why Robonix

| | |
| :--- | :--- |
| **Control plane / data plane** | Register providers once; agents negotiate channels at runtime (gRPC, MCP, ROS 2, shared memory). |
| **EAIOS layers** | Primitive → Service → Skill → Task, with discovery and SKILL.md-oriented agent UX. |
| **Ship with Rust** | Atlas, Pilot, Executor, Liaison (interaction TODO), and `rbnx` CLI in one workspace. |

## Architecture

<p align="center">
  <img src="images/robonix-layers.png" alt="Robonix software architecture layers" width="580" />
</p>

Robonix follows the EAIOS four-layer abstraction — Primitive, Service, Skill, Task — with a unified control plane that handles registration, discovery, and channel negotiation across pluggable transports (gRPC, MCP, ROS 2, shared memory).

## Workspace

| Crate | Role |
|-------|------|
| `robonix-atlas` | Control plane: registration, discovery, channel negotiation, skill catalog, heartbeat, runtime inspection |
| `robonix-sdk` | Thin async Rust client for the control-plane API |
| `robonix-pilot` | VLM-driven reasoning: ReAct-style loop and sessions; streams `PilotEvent`; dispatches **`TaskGraph`** to Executor (v1: linear `TaskCall[]`; BT/RTDL TODO) |
| `robonix-executor` | Tool dispatch: builtin / MCP / gRPC; **`ExecutorService.Execute`** |
| `robonix-liaison` | User-facing layer: text → **`Intent`** → **`PilotEvent`** stream (interaction / UX **TODO**) |
| `robonix-cli` (`rbnx`) | Package validate / build / start; runtime inspection; **`rbnx chat`** (TUI → Pilot); **`rbnx graph`** (topology PNG/SVG) |
| `robonix-codegen` | ROS IDL + **`rust/contracts`** TOML → **`rust/crates/robonix-interfaces/robonix_proto/`** (generated `.proto`, incl. `robonix_contracts.proto`) |
| `robonix-buffer` | Shared-memory / buffer helpers for high-bandwidth data |

ROS payloads are canonical in **`rust/crates/robonix-interfaces/lib/`**. Stable **`contract_id`** paths live in **`rust/contracts/**/*.toml`** and match control-plane fields (`DeclareInterface` / `QueryNodes`). Regenerate protos with `robonix-codegen` from **`rust/`** (see **[rust/README.md](rust/README.md)**).

## Project Status

### Available

- Control plane with multi-transport channels (gRPC, MCP, ROS 2, shared memory)
- **Pilot / Executor** path: VLM + tools + **`rbnx chat`** (TUI → Pilot)
- SKILL.md for skill discovery and LLM-oriented behavior text
- Package system (`rbnx validate` / `build` / `start`)
- **`robonix-codegen`**: ROS `.msg` / `.srv` + contracts → generated **`robonix_proto/`** (do not hand-edit)
- **Tiago** Webots E2E (`examples/run.sh` + `tiago_sim_stack`: Docker, Webots, Nav2, rviz2, ROS 2–MCP bridge)
- **ManiSkill3** VLA demo (`examples/packages/maniskill_vla_demo`: Fetch in ReplicaCAD, Octo VLA, no Docker/ROS on host — see **[docs/src/getting-started/maniskill-demo.md](docs/src/getting-started/maniskill-demo.md)**)

### In Progress

- **Liaison**: user-facing interaction beyond the current wiring (product-grade UX **TODO**)
- Contract / namespace catalog enforcement on the server
- **TaskGraph**: behavior-tree / RTDL beyond today’s linear wire encoding

### Documentation

- **[docs/](docs/)** — mdBook: architecture, interface catalog (**primitive/** vs **service/**), integration guides. Build: `cd docs && mdbook build`.

## Platforms & PRM

**Integrated demos** (see **[rust/examples/README.md](rust/examples/README.md)**):

| Platform | What you run | Notes |
|----------|----------------|-------|
| **Tiago** (PAL Robotics) | Docker Compose stack: Webots + Nav2 + rviz2 + `tiago_bridge` (ROS 2 ↔ MCP) | Needs Docker, X11, GPU recommended; driven by `./examples/run.sh` or `rbnx start` on `tiago_sim_stack`. |
| **ManiSkill3** (Fetch, ReplicaCAD) | Native Python: env / VLA / perception nodes + Atlas + Pilot | No host ROS 2; GPU recommended; package `maniskill_vla_demo`. |

**Abstract primitives** — interface IDs under `robonix/prm/*` cover camera, base, arm, gripper, manipulation, sensor, force-torque, and related payloads. Implementations attach to real hardware or simulators by registering with Atlas. Full tables: **[rust/crates/robonix-interfaces/README.md](rust/crates/robonix-interfaces/README.md)**.

## Services (examples)

| Service | `contract_id` (stable path) | Transport |
|---------|----------------------------|-----------|
| Pilot | `robonix/sys/runtime/pilot` | gRPC |
| Executor | `robonix/sys/runtime/executor` | gRPC |
| Liaison | `robonix/sys/runtime/liaison` | gRPC |
| VLM (OpenAI-compatible backend) | `robonix/sys/model/vlm/chat` | gRPC |
| Memory search (gRPC wire) | `robonix/sys/memory/search` | gRPC |

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch from `dev`
3. Format code with `cd rust && make fmt`
4. Ensure `make check` passes (formatting + clippy)
5. Submit a pull request

## License

Mulan Permissive Software License, Version 2 (MulanPSL-2.0). See [LICENSE](LICENSE) file for details.
