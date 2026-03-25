<p align="center">
  <img src="images/robonix-logo.svg" alt="Robonix logo" width="400" />
  <br><br>
  <a href="https://github.com/syswonder/robonix/blob/main/LICENSE"><img src="https://img.shields.io/badge/license-MulanPSL--2.0-red" alt="License: MulanPSL-2.0" /></a>    <img src="https://img.shields.io/github/contributors/syswonder/robonix?color=blue" alt="Contributors" />
  <img src="https://img.shields.io/github/languages/code-size/syswonder/robonix?color=green" alt="Code size" />
  <img src="https://img.shields.io/github/repo-size/syswonder/robonix?color=white" alt="Repo size" />
  <img src="https://img.shields.io/github/languages/top/syswonder/robonix?color=orange" alt="Languages" />
  <br><br>
</p>

**Robonix** is an open-source embodied intelligence framework built with Rust, implementing the EAIOS (Embodied AI Operating System) architecture. It decouples AI models from hardware through a control-plane / data-plane design, so providers (sensors, actuators, algorithm services) register once and any agent or consumer can discover and use them at runtime.

> [!WARNING]
> Robonix is in an early, fast-moving development phase. Interfaces, IDL layouts, and internal designs may change without notice. No API stability is guaranteed until a versioned release is published.

## Architecture

<p align="center">
  <img src="images/robonix-layers.png" alt="Robonix software architecture layers" width="580" />
</p>

Robonix follows the EAIOS four-layer abstraction — Primitive, Service, Skill, Task — with a unified control plane that handles registration, discovery, and channel negotiation across pluggable transports (gRPC, MCP, ROS 2, shared memory).

## Workspace

| Crate | Role |
|-------|------|
| `robonix-server` | gRPC control plane: node registration, interface declaration, channel negotiation, skill catalog, heartbeat, runtime inspection |
| `robonix-sdk` | Thin async Rust client for the control-plane API |
| `robonix-agent` | System agent: VLM-driven ReAct loop with dynamic MCP tool discovery and SKILL.md-based behavior |
| `robonix-cli` (`rbnx`) | Package validate / build / start, runtime inspection (`nodes`, `describe`, `tools`, `channels`, `inspect`) |
| `ridlc` | ROS IDL → Proto codegen: reads `.msg` / `.srv` definitions and generates `.proto` files |

## Project Status

### Available

- Control plane with multi-transport channels (gRPC, MCP, ROS 2, shared memory)
- VLM-driven agent with ReAct loop and MCP tool calling
- SKILL.md format for LLM-driven skill discovery and behavior guidance
- Package system (`rbnx validate` / `build` / `start`)
- `ridlc` codegen: ROS `.msg` / `.srv` → `.proto`
- Tiago Webots E2E demo (Docker: Webots + Nav2 + rviz2 + MCP bridge)

### In Progress

- Namespace catalog validation on server side
- Standard primitive interface contract enforcement

## Hardware / PRM Support

| Platform | Type | Status |
|----------|------|--------|
| Tiago (PAL Robotics) | Webots simulation + Nav2 + MCP bridge | Available |

Abstract primitive interfaces are defined for camera, base, arm, gripper, sensor, and force-torque under the `robonix/prm/*` namespace. See `rust/robonix-interfaces/README.md` for the full capability table.

## Services

| Service | Interface | Transport |
|---------|-----------|-----------|
| VLM (OpenAI-compatible backend) | `robonix/sys/model/vlm/chat` | gRPC |

## Quick Start

```bash
git clone https://github.com/syswonder/robonix
cd robonix
git submodule update --init --recursive
cd rust
cargo build --workspace
make install          # installs rbnx, ridlc, robonix-agent, robonix-server wrapper
```

Run the full E2E demo (requires Docker, X11, and a VLM API key):

```bash
cd rust
cp examples/.env.example examples/.env   # fill in VLM_API_BASE, VLM_API_KEY, VLM_MODEL
./examples/run.sh
```

Run without simulation (VLM + agent only):

```bash
cd rust
START_SIM_STACK=0 ./examples/run.sh
```

See [rust/README.md](rust/README.md) for more options and [rust/examples/README.md](rust/examples/README.md) for the demo walkthrough.

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch from `dev`
3. Format code with `cd rust && make fmt`
4. Ensure `make check` passes (formatting + clippy)
5. Submit a pull request

## License

Mulan Permissive Software License, Version 2 (MulanPSL-2.0). See [LICENSE](LICENSE) file for details.
