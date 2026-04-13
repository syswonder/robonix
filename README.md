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

<p align="center">
  <a href="README-zh.md">简体中文</a> ·
  <a href="docs/">Docs (mdBook)</a> ·
  <a href="rust/README.md">Rust workspace</a> ·
  <a href="rust/examples/README.md">Examples &amp; E2E</a>
</p>

<br />

<p align="center">
  <img src="images/demo_01_readme.gif" alt="Robonix demo: runtime stack in action" width="920" />
</p>

## Quickstart

```bash
git clone --recursive https://github.com/syswonder/robonix
cd robonix/rust
make install                              # → ~/.cargo/bin (rbnx, robonix-atlas, …)
pip install -r examples/requirements.txt
cp examples/.env.example examples/.env    # fill VLM_API_KEY + VLM_BASE_URL + VLM_MODEL
./examples/run.sh                         # atlas + vlm + tiago sim + pilot + …
```

Open another terminal and talk to the agent:

```bash
rbnx chat
```

Full first-run walkthrough: [**docs/src/getting-started/quickstart.md**](https://github.com/syswonder/robonix-book/blob/main/src/getting-started/quickstart.md).

## What's here

| | |
|---|---|
| [`rust/`](rust/) | Cargo workspace — atlas / pilot / executor / liaison / cli / codegen / sdk / interfaces / buffer |
| [`rust/contracts/`](rust/contracts/) | Stable `contract_id` definitions (TOML). Map to ROS IDL under `rust/crates/robonix-interfaces/lib/`. |
| [`rust/examples/packages/`](rust/examples/packages/) | Example packages: `vlm_service`, `memsearch_service`, `tiago_sim_stack`, `maniskill_vla_demo`, `zero_copy_demo`, `clawhub_skills` |
| [`docs/`](https://github.com/syswonder/robonix-book) | mdBook — architecture, interface catalog, integration guides |
| [`images/`](images/) | Logo + demo assets |

## Architecture — 30 seconds

<p align="center">
  <img src="images/robonix-layers.png" alt="Robonix software architecture layers" width="580" />
</p>

Four-layer EAIOS abstraction: **primitive → service → skill → task**. A unified control plane (Atlas) handles registration, discovery, and channel negotiation across transports (gRPC, MCP, ROS 2, shared memory). Agent reasoning lives in Pilot (VLM + ReAct + TaskGraph), tool dispatch in Executor, user-facing interaction in Liaison.

Dive deeper:
- [**Overview**](https://github.com/syswonder/robonix-book/blob/main/src/architecture/overview.md) — control/data plane, one full request end-to-end
- [**Crates**](https://github.com/syswonder/robonix-book/blob/main/src/architecture/crates.md) — each binary's role and port
- [**Namespaces & contracts**](https://github.com/syswonder/robonix-book/blob/main/src/architecture/namespace-and-interfaces.md) — how `robonix/prm/*` and `robonix/srv/*` work
- [**Interface catalog**](https://github.com/syswonder/robonix-book/blob/main/src/interface-catalog/index.md) — every primitive + service contract

## Agent Skills (agentskills.io)

Robonix can ingest [**Agent Skills**](https://agentskills.io/) — the same `SKILL.md`+frontmatter format used by Claude Code, Cursor, GitHub Copilot, OpenCode, Gemini CLI, OpenClaw, etc. Place them under `~/.robonix/skills/<name>/` (or any directory in `ROBONIX_SKILLS_EXTRA_DIRS`); Pilot picks them up at startup.

> **Disambiguation**: an "Agent Skill" here is an **agent-context playbook** — a Markdown file telling an LLM how to use a set of tools. It is **not** the same thing as the **Skill layer** in the Robonix whitepaper, which refers to deployable agent behaviors (basic skills = pretrained VLA/RL processes; RTDL skills = runtime-generated structured plans). The two share a name but live at different abstraction levels. See [**docs/src/skill-library.md**](https://github.com/syswonder/robonix-book/blob/main/src/skill-library.md) for the Robonix Skill model — Agent Skills are just one possible source of `SkillInfo` injected into the VLM prompt.

> Earlier versions auto-scanned `<package>/skills/` and registered entries to Atlas. That mechanism has been **removed** — packages and skills are now decoupled. See the migration note in `skill-library.md`.

## `rbnx` CLI

`rbnx --help` for the full list. Most-used:

```bash
rbnx setup                # register the clone so other packages find contracts/IDL
rbnx build -p <pkg>       # run the package's build.sh (codegen + whatever)
rbnx codegen -p <pkg>     # just the codegen part (proto + Python stubs)
rbnx start -p <pkg> -n <node>
rbnx chat                 # TUI → Pilot → Executor
rbnx nodes / tools / channels / inspect / graph
```

Package authors: see [**Build & Codegen**](https://github.com/syswonder/robonix-book/blob/main/src/integration-guide/build-and-codegen.md).

## Status

> [!WARNING]
> Early, fast-moving development. Interfaces, IDL layouts, internal designs may change without notice. No API stability until a versioned release.

Working today: Atlas control plane, Pilot/Executor ReAct loop, SKILL.md discovery, `rbnx` package flow (`validate` / `build` / `codegen` / `start` / `install`), Tiago Webots E2E, ManiSkill3 VLA demo.

In progress: formal system-service deployment (services are example packages today), TaskGraph beyond linear, catalog enforcement on the server, product-grade Liaison.

## Contributing

Fork from `dev`, `make fmt`, `make check`, PR. See `CONTRIBUTING.md` if present.

## License

Mulan Permissive Software License, Version 2 (MulanPSL-2.0). See [LICENSE](LICENSE).
