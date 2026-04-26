<p align="center">
  <img src="images/robonix-logo.svg" alt="Robonix" width="420" />
</p>

<h3 align="center">Robonix — The Embodied AI Operating System</h3>

<p align="center">
  <em>Rust-native EAIOS. Skill-centric runtime with Liaison, Pilot, Atlas, Nexus, Executor — human intent to embodied action.</em>
</p>

<p align="center">
  <a href="https://github.com/syswonder/robonix/blob/main/LICENSE"><img src="https://img.shields.io/badge/license-MulanPSL--2.0-red?style=flat-square" alt="License" /></a>
  <a href="https://github.com/syswonder/robonix/graphs/contributors"><img src="https://img.shields.io/github/contributors/syswonder/robonix?color=blue&style=flat-square" alt="Contributors" /></a>
  <img src="https://img.shields.io/github/languages/code-size/syswonder/robonix?color=green&style=flat-square" alt="Code size" />
  <img src="https://img.shields.io/github/repo-size/syswonder/robonix?color=lightgray&style=flat-square" alt="Repo size" />
  <img src="https://img.shields.io/github/languages/top/syswonder/robonix?color=orange&style=flat-square" alt="Top language" />
</p>

<br />


## Quickstart

```bash
git clone --recursive https://github.com/syswonder/robonix
cd robonix/rust
make install                              # → ~/.cargo/bin (rbnx, robonix-atlas, …)
cd examples
# edit robonix_manifest.yaml to set the VLM endpoint, api key, and model to your own
rbnx deploy
```

Open another terminal and talk to the agent:

```bash
rbnx chat
```

Full first-run walkthrough: [**docs/src/getting-started/quickstart.md**](https://github.com/syswonder/robonix-book/blob/main/src/getting-started/quickstart.md).

## Architecture

A unified control plane (Atlas) handles registration, discovery, and channel negotiation across transports (gRPC, MCP, ROS 2, shared memory). Agent reasoning lives in Pilot (VLM + ReAct + TaskGraph), tool dispatch in Executor, user-facing interaction in Liaison.

Dive deeper:
- [**Overview**](https://github.com/syswonder/robonix-book/blob/main/src/architecture/overview.md) — control/data plane, one full request end-to-end
- [**Crates**](https://github.com/syswonder/robonix-book/blob/main/src/architecture/crates.md) — each binary's role and port
- [**Namespaces & contracts**](https://github.com/syswonder/robonix-book/blob/main/src/architecture/namespace-and-interfaces.md) — how `robonix/primitive/*` and `robonix/service/*` work
- [**Interface catalog**](https://github.com/syswonder/robonix-book/blob/main/src/interface-catalog/index.md) — every primitive + service contract

## Status

> [!WARNING]
> Robonix is in early development. API and interfaces, IDL layouts, internal designs may change without notice. No API stability until a versioned release.

## License

Mulan Permissive Software License, Version 2 (MulanPSL-2.0). See [LICENSE](LICENSE).
