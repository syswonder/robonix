# Robonix (Rust workspace)

Robonix is a robotics control-plane framework: nodes register capabilities, declare interfaces, negotiate channels over pluggable transports, and expose SKILL.md text so agents and tools can discover how to use them.

Platform: primary target is Linux; ROS 2 workloads are not assumed on the host — use containers or a sourced distro when needed.

## Architecture

| Crate | Role |
|--------|------|
| robonix-server | gRPC control plane: registration, discovery, channel negotiation, skill catalog |
| robonix-sdk | Thin Rust client for the runtime API |
| robonix-agent | System agent: discovers VLM via control plane, runs ReAct loop with MCP tool calling |
| robonix-cli | `rbnx` CLI for package validate / build / start and runtime inspection |
| ridlc | ROS IDL codegen — generates `.proto` from ROS `.msg`/`.srv` definitions |

## Workspace layout

```
rust/
├── crates/                  # Rust packages (server, sdk, agent, CLI, ridlc)
├── docs/                    # Namespace sketch (docs/NAMESPACE.md), PoC walkthrough (docs/POC.md)
├── examples/                # Minimal E2E: packages/, scripts/, run.sh
├── proto/                   # robonix_runtime.proto (authoritative control-plane gRPC API)
├── robonix-interfaces/
│   ├── lib/                 # ROS IDL (.msg/.srv) — canonical type definitions
│   └── robonix_proto/       # Generated .proto (via ridlc --lang proto)
└── _deprecated/             # Legacy code kept for reference
```

## Quick start

Build everything:

```bash
cd rust
cargo build --workspace
```

Run the control plane:

```bash
./start_server
```

Run the E2E example (server + VLM + Tiago sim + agent):

```bash
cd rust
cp examples/.env.example examples/.env   # set VLM_* keys
./examples/run.sh                         # VLM + tiago_sim_stack (Docker Webots + Nav2 + bridge; needs Docker + X11)
START_SIM_STACK=0 ./examples/run.sh       # VLM only (no sim container)
```

Manual Tiago sim (with `robonix-server` already up):

```bash
cd rust
cargo run -p robonix-cli -- build -p examples/packages/tiago_sim_stack
cargo run -p robonix-cli -- start -p examples/packages/tiago_sim_stack -n com.robonix.prm.tiago
```

See [`examples/README.md`](examples/README.md) and [`docs/NAMESPACE.md`](docs/NAMESPACE.md).

## Key concepts

- **Node** — Registered participant (primitive, service, skill) with a namespace and optional SKILL.md.
- **Interface** — Named capability on a node; lists supported transports and opaque metadata.
- **Channel** — Allocated connection from `NegotiateChannel` (id, transport, endpoint).
- **Transport** — Concrete wiring (gRPC, MCP, ROS 2, shared memory); chosen at negotiation time.
- **SKILL.md** — Human/agent-oriented description of how to invoke the node; served via `QueryAllSkills`.
