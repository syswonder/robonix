# Robonix (Rust workspace)

Robonix is a robotics EAIOS (Embodied AI Operating System) framework: capability instances register with Atlas, declare interfaces over pluggable transports, and ship one CAPABILITY.md per package so agents and tools can discover how to use them.

Platform: primary target is Linux; ROS 2 workloads are not assumed on the host — use containers or a sourced distro when needed.

## Architecture

| Crate | Role |
|--------|------|
| robonix-atlas | gRPC control plane: capability registration, interface discovery, CAPABILITY.md catalog |
| robonix-sdk | Thin Rust client for the Atlas API (being retired in favour of robonix-proto, task #31) |
| robonix-pilot | VLM-driven reasoning service: ReAct loop, session management, `TaskGraph` slices (v1 linear; BT/RTDL TODO) |
| robonix-executor | Tool dispatch runtime: builtin / MCP / gRPC routing |
| robonix-liaison | User-facing interaction layer: text stdin→Intent→PilotEvent |
| robonix-cli | `rbnx` CLI for package validate / build / start and runtime inspection |
| robonix-buffer | Shared-memory / buffer utilities (optional data-plane paths) |
| robonix-codegen | ROS IDL + `contracts/` TOML → generates `robonix_proto/*.proto` |

## Workspace layout

```
rust/
├── crates/                  # Rust packages (atlas, sdk, pilot, executor, liaison, CLI, robonix-codegen, buffer)
├── contracts/               # Contract TOML → stable contract_id + shape; input to robonix-codegen --contracts
├── examples/                # E2E demo: packages/, scripts/, run.sh
├── proto/                   # Control plane: atlas.proto, …
├── crates/robonix-interfaces/
│   ├── lib/                 # ROS IDL (.msg/.srv) — canonical payload definitions
│   └── robonix_proto/       # robonix-codegen-generated `.proto` only (from `lib/` + `contracts/`)
└── _deprecated/             # Legacy code kept for reference
```

User-facing documentation (mdBook) lives in **../docs/** (`mdbook build`).

## Quick start

Build everything:

```bash
cd rust
cargo build --workspace
```

Run the control plane:

```bash
./start_server   # starts robonix-atlas on 0.0.0.0:50051
```

Run the full E2E stack (atlas + executor + pilot + VLM + Tiago sim + liaison):

```bash
cd rust
cp examples/.env.example examples/.env   # set VLM_* keys
./examples/run.sh                         # all 4 services + VLM + tiago_sim_stack
START_SIM_STACK=0 ./examples/run.sh       # VLM only (no sim container)
```

Manual Tiago sim (with `robonix-atlas` already up):

```bash
cd rust
cargo run -p robonix-cli -- build -p examples/packages/tiago_sim_stack
cargo run -p robonix-cli -- start -p examples/packages/tiago_sim_stack -n com.robonix.prm.tiago
```

See [`examples/README.md`](examples/README.md).

## Key concepts

- **Capability instance** — Registered execution unit (primitive, service, skill) with a reverse-DNS id under one namespace, optionally with a CAPABILITY.md.
- **Capability interface** — A single `contract_id` offered by an instance. The instance binds a `(transport, endpoint)` per interface via `DeclareInterface`.
- **Transport** — Concrete wiring (gRPC, MCP, ROS 2, shared memory, …); closed enum on the wire (`Transport`).
- **CAPABILITY.md** — One markdown per package describing all of its interfaces; served on demand via `QueryCapabilityMd`. Replaces the legacy per-skill `SKILL.md` convention.
- **Intent** — Single user turn; Liaison sends it to Pilot and streams back `PilotEvent`.
- **TaskGraph** — One incremental slice of the task (behavior-tree-shaped contract; v1 = linear `TaskCall[]`); dispatched by Executor (TODO: full BT + RTDL).
