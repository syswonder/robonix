# Robonix (Rust workspace)

Robonix is a robotics control-plane framework: nodes register capabilities, declare interfaces, negotiate channels over pluggable transports, and expose SKILL.md text so agents and tools can discover how to use them.

Platform: primary target is Linux (glibc-based and musl-friendly builds are acceptable); ROS 2 workloads are not assumed on the host—use containers or a sourced distro when needed.

## Architecture

| Crate | Role |
|--------|------|
| robonix-server | gRPC control plane: registration, discovery, channel negotiation, skill catalog, heartbeat/unregister |
| robonix-sdk | Thin Rust client for the runtime API |
| robonix-agent | System agent: discovers VLM via control plane, runs ReAct loop |
| ridlc | ROS IDL codegen (Python / Rust / proto, mapping hints) |

Other workspace crates (e.g. robonix-cli, robonix-buffer) support tooling and data paths.

## Workspace layout

```
rust/
├── crates/                  # Rust packages (server, sdk, agent, CLI, ridlc, ...)
├── docs/                    # Namespace sketch (see docs/NAMESPACE.md)
├── examples/                # Minimal E2E: packages/, scripts/, run.sh
├── proto/                   # robonix_runtime.proto (authoritative for gRPC API)
├── robonix-interfaces/
│   ├── lib/                 # ROS IDL (.msg) -- canonical type definitions
│   ├── robonix_proto/       # Hand-authored / generated proto (see ridlc --lang proto)
│   └── robonix_mapping/     # Cross-distro hints (via ridlc --lang mapping)
└── _deprecated/             # Legacy sim packages (e.g. tiago_demo_package for Webots)
```

## Quick start

Minimal platform (control-plane smoke, proto regeneration, Tiago Docker bridge): see [`examples/MINIMAL_PLATFORM.md`](examples/MINIMAL_PLATFORM.md).

Build everything:

```bash
cd rust
cargo build --workspace
```

Run the control plane:

```bash
./start_server
```

Run the E2E example (**`rbnx` validate/build/start** per RFC002 package; server + agent are `cargo run`):

```bash
cd rust
cp examples/.env.example examples/.env   # VLM_* keys
./examples/run.sh                 # VLM + tiago_sim_stack (Docker Webots + Nav2 + bridge; needs Docker + X11)
START_SIM_STACK=0 ./examples/run.sh  # VLM only (no sim container)
```

Dockerized Tiago sim (same stack `run_e2e` starts by default); with `robonix-server` already up:

```bash
cd rust
cargo run -p robonix-cli -- build -p examples/packages/tiago_sim_stack
cargo run -p robonix-cli -- start -p examples/packages/tiago_sim_stack -n com.robonix.prm.tiago
```

Cross-distro mapping hint file (optional, for IDL tooling):

```bash
cargo run -p ridlc -- --lang mapping \
  --idl-diff tools/ros_distro_idl_difftest/reports/idl_diff_report.json \
  --from-distro humble --to-distro jazzy \
  -o robonix-interfaces/robonix_mapping
```

See [`examples/README.md`](examples/README.md) and [`docs/NAMESPACE.md`](docs/NAMESPACE.md).

## Key concepts

- Node -- Registered participant (primitive, service, skill, ...) with a namespace and optional SKILL.md.
- Interface -- Named capability on a node; lists supported transports and opaque metadata.
- Channel -- Allocated connection from NegotiateChannel (id, transport, endpoint string).
- Transport -- Concrete wiring (e.g. ROS 2, shared memory, gRPC, MCP); chosen at negotiation time.
- SKILL.md -- Human/agent-oriented description of how to invoke the node; served via `QueryAllSkills` / `QuerySkillMd`.

## Design docs (RFCs)

Specifications and rationale live in the main repo docs:

- [RFC index](../docs/src/rfc/) -- e.g. control plane (RFC003), RIDL, transports, SKILL format.
