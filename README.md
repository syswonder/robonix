<p align="center">
  <img src="images/robonix-logo.svg" alt="Robonix" width="420" />
</p>

<h3 align="center">Robonix — The Embodied AI Operating System</h3>

<p align="center">
  <em>Rust-native EAIOS. Capability-centric runtime with Atlas (control plane), Pilot (LLM + plan), Executor (dispatch), Liaison (dialogue) — human intent to embodied action.</em>
</p>

<p align="center">
  <a href="https://github.com/syswonder/robonix/blob/main/LICENSE"><img src="https://img.shields.io/badge/license-MulanPSL--2.0-red?style=flat-square" alt="License" /></a>
  <a href="https://github.com/syswonder/robonix/graphs/contributors"><img src="https://img.shields.io/github/contributors/syswonder/robonix?color=blue&style=flat-square" alt="Contributors" /></a>
  <img src="https://img.shields.io/github/languages/code-size/syswonder/robonix?color=green&style=flat-square" alt="Code size" />
  <img src="https://img.shields.io/github/repo-size/syswonder/robonix?color=lightgray&style=flat-square" alt="Repo size" />
  <img src="https://img.shields.io/github/languages/top/syswonder/robonix?color=orange&style=flat-square" alt="Top language" />
</p>

<br />

<p align="center">
  <img src="images/demo_01_readme.gif" alt="Robonix demo: runtime stack in action" width="920" />
</p>

## Quickstart

```bash
git clone --recursive https://github.com/syswonder/robonix
cd robonix/rust
make install                              # → ~/.cargo/bin (rbnx, robonix-atlas, robonix-pilot, robonix-executor)
                                          # also runs `rbnx setup` to register this clone as the source tree
```

The Webots Tiago example (`examples/webots/`) is the standard end-to-end demo.
It runs in two terminals: the simulator and Robonix itself.

```bash
# Terminal (1) — simulation environment (Webots GUI; not a Robonix package — just docker compose)
bash examples/webots/sim/start.sh

# Then in another terminal (2) — Robonix stack: system services + Tiago primitives + Nav2 service
export VLM_BASE_URL=https://api.openai.com/v1   # any OpenAI-compatible endpoint
export VLM_API_KEY=sk-...
export VLM_MODEL=gpt-5.4-mini
cd examples/webots
rbnx build # might take a while since scene and speech needs some model weights and docker containers downloaded while building
rbnx boot
```

Once `rbnx boot` reports the stack is up, in a third terminal:

```bash
# Terminal (3)
rbnx caps          # list registered capabilities + interfaces
rbnx chat          # interactive ratatui chat with the pilot
```

To tear everything down — Ctrl-C the `rbnx boot` terminal, or from
any other shell:

```bash
cd examples/webots && rbnx shutdown      # reads rbnx-boot/state.json,
                                         # SIGTERMs each component's PGID
bash sim/stop.sh                         # then stop the Webots container
```

Full first-run walkthrough: [**docs/src/getting-started/quickstart.md**](https://github.com/syswonder/robonix-book/blob/main/src/getting-started/quickstart.md).

## Architecture

Atlas is the single control plane: every capability provider (`primitive` / `service` / `skill`) calls `RegisterPrimitive` / `RegisterService` / `RegisterSkill` + `DeclareCapability(transport, endpoint, params)` on startup; pilot/executor discover via `Query` and open data-plane connections via `ConnectCapability`. Transports are pluggable — gRPC, MCP, ROS 2 — and the contract TOMLs under `capabilities/` describe the schemas all of them are allowed to carry.

Reasoning lives in **Pilot** (LLM agent loop with persistent execution semantics; CAPABILITY.md per provider is lazy-loaded by the LLM via `read_file`). Capability dispatch lives in **Executor** (capability-call routing + a few in-process builtins for filesystem / shell). User-facing dialogue lives in **Liaison** (audio / NLP front-end + session orchestration; clients talk to Liaison, never to Pilot directly).

Dive deeper:
- [**Overview**](https://github.com/syswonder/robonix-book/blob/main/src/architecture/overview.md) — control plane, one full request end-to-end
- [**Crates**](https://github.com/syswonder/robonix-book/blob/main/src/architecture/crates.md) — each binary's role and listening port
- [**Namespaces & contracts**](https://github.com/syswonder/robonix-book/blob/main/src/architecture/namespace-and-interfaces.md) — how `robonix/primitive/*` and `robonix/service/*` work
- [**Interface catalog**](https://github.com/syswonder/robonix-book/blob/main/src/interface-catalog/index.md) — every primitive + service contract

## Status

> [!WARNING]
> Robonix is in early development. API and interfaces, IDL layouts, internal designs may change without notice. No API stability until a versioned release.

## License

Mulan Permissive Software License, Version 2 (MulanPSL-2.0). See [LICENSE](LICENSE).
