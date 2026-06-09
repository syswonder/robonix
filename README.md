<p align="center">
  <img src="images/robonix-logo.svg" alt="Robonix" width="420" />
</p>

<h3 align="center">Robonix — The Embodied AI Operating System</h3>

<p align="center">
  <em>An EAIOS that turns the robot body into a uniform, capability-first runtime — so models, sensors, and actuators plug in once and reuse everywhere.</em>
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

## What it is

Robonix is the **operating system** between a robot's hardware and an embodied
LLM/VLM/VLA/WAM brain. It standardises how device drivers, runtime services, user
skills, and the planner discover and talk to each other; it owns identity,
configuration, time, transport, logging, health, body model, scene model,
execution, and safety as named, replaceable components.

| Component                           | What it owns                                                           |
| ----------------------------------- | ---------------------------------------------------------------------- |
| **[atlas](system/atlas/)**       | capability discovery / catalog                                         |
| **[chronos](system/chronos/)**   | unified time / PTP alignment*(stub)*                                 |
| **[executor](system/executor/)** | capability orchestration & dispatch                                    |
| **[keystone](system/keystone/)** | identity / configuration / policy*(stub)*                            |
| **[liaison](system/liaison/)**   | human–machine interaction (chat / audio / TUI)                        |
| **[nexus](system/nexus/)**       | gRPC / MCP / ROS 2 communication libraries*(library, not a process)* |
| **[pilot](system/pilot/)**       | planning / decision / memory / world model                             |
| **[scene](system/scene/)**       | scene state / semantic map / object registry                           |
| **[scribe](system/scribe/)**     | structured logs / replay / audit*(stub)*                             |
| **[sentinel](system/sentinel/)** | safety supervision*(in executor for v0.1)*                           |
| **[soma](system/soma/)**         | body state / device & primitive abstraction*(stub)*                  |
| **[vitals](system/vitals/)**     | health monitoring / heartbeat*(partial via atlas)*                   |

On top of system, three open categories — provided as
contracts (61 standard interfaces in `capabilities/`) and reference
implementations alongside the system:

* **primitive** — one device per package (camera, lidar, chassis, arm). Lives
  in deployment repos and per-example folders (e.g. `examples/webots/primitives/`).
* **service** — runtime functionality (mapping, navigation, semantic map,
  memory, speech, voiceprint). Default reference implementations ship in
  [`services/`](services/); each can be swapped out by a deployment.
* **skill** — user-defined reusable execution flows (grasp, place, explore,
  fold-clothes …). Lives wherever the deploy/integrator wants.

## Quickstart

```bash
git clone --recursive https://github.com/syswonder/robonix
cd robonix
make install   # builds the Cargo workspace and installs
               # rbnx + robonix-{atlas,pilot,executor,liaison,codegen}
               # to ~/.cargo/bin, then registers this clone via `rbnx setup`
```

The Webots Tiago example (`examples/webots/`) is the standard end-to-end demo.
Two terminals — the simulator and Robonix itself.

```bash
# (1) — simulation environment (Webots GUI; not a Robonix package — just docker compose)
bash examples/webots/sim/start.sh

# (2) — Robonix: system services + Tiago primitives + Nav2 + scene
export VLM_BASE_URL=https://api.openai.com/v1   # any OpenAI-compatible endpoint
export VLM_API_KEY=sk-...
export VLM_MODEL=gpt-5.5
cd examples/webots
rbnx build       # first run pulls model weights + docker images, may take a while
rbnx boot
```

Once `rbnx boot` reports the stack is up:

```bash
# (3)
rbnx caps          # list registered capabilities + interfaces
rbnx chat          # interactive TUI chat with the pilot
```

Tear-down:

```bash
cd examples/webots && rbnx shutdown    # reads rbnx-boot/state.json,
                                       # SIGTERMs each component's PGID
bash examples/webots/sim/stop.sh       # then stop the Webots container
```

Full first-run walkthrough:
[**docs/src/getting-started/quickstart.md**](https://github.com/syswonder/robonix-book/blob/main/src/getting-started/quickstart.md).

## Repository layout

```
system/         system components, one directory each
services/       default reference service implementations (memsearch, voiceprint, speech)
pylib/          Python SDK (robonix-api on PyPI)
capabilities/   contract TOMLs + ROS-style IDL tree (capabilities/lib/)
tools/          dev tooling — rbnx CLI + codegen
examples/       end-to-end deployments (webots, test_ci)
docs/           mdBook developer guide (submodule)
Cargo.toml      workspace for the Rust components (4 in system/, 2 in tools/)
Makefile        top-level orchestrate (build / install / fmt / check)
```

`system/<name>/` and `services/<name>/` and `tools/<name>/` are each
self-contained packages — Rust ones carry their own `Cargo.toml`, Python
ones their own `pyproject.toml`. There is no top-level `rust/` /
`python/` split; the runtime role is what determines where a component
lives, not the implementation language.

## Architecture

Dive deeper:

* [**Overview**](https://github.com/syswonder/robonix-book/blob/main/src/architecture/overview.md) — control plane, one full request end-to-end
* [**Namespaces & contracts**](https://github.com/syswonder/robonix-book/blob/main/src/architecture/namespace-and-interfaces.md) — how `robonix/primitive/*` / `robonix/service/*` / `robonix/skill/*` / `robonix/system/*` work
* [**Interface catalog**](https://github.com/syswonder/robonix-book/blob/main/src/interface-catalog/index.md) — every primitive + service contract

## Skills

We develop Skill workflows adapted to Robonix, including fine-tuning, post-training and model deployment optimization.

## Status

> \[!WARNING]
> Robonix is in early development. APIs, IDL layouts, and internal designs
> may change without notice. No API stability until a versioned release.

## Contributors

[![All Contributors](https://img.shields.io/github/all-contributors/syswonder/robonix?color=ee8449&style=flat-square)](#contributors)

Thanks goes to these wonderful people:

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="20%"><a href="https://github.com/enkerewpo"><img src="https://avatars.githubusercontent.com/u/17263645?v=4?s=80" width="80px;" alt="wheatfox"/><br /><sub><b>wheatfox</b></sub></a><br /><a href="https://github.com/syswonder/robonix/commits?author=enkerewpo" title="Code">💻</a> <a href="#maintenance-enkerewpo" title="Maintenance">🚧</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/HustWolfzzb"><img src="https://avatars.githubusercontent.com/u/19464597?v=4?s=80" width="80px;" alt="Zhaobo Zhang"/><br /><sub><b>Zhaobo Zhang</b></sub></a><br /><a href="https://github.com/syswonder/robonix/commits?author=HustWolfzzb" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/KouweiLee"><img src="https://avatars.githubusercontent.com/u/98637586?v=4?s=80" width="80px;" alt="Guowei Li"/><br /><sub><b>Guowei Li</b></sub></a><br /><a href="https://github.com/syswonder/robonix/commits?author=KouweiLee" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/ken4647"><img src="https://avatars.githubusercontent.com/u/87317372?v=4?s=80" width="80px;" alt="wuzheng"/><br /><sub><b>wuzheng</b></sub></a><br /><a href="https://github.com/syswonder/robonix/commits?author=ken4647" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/kaileliu"><img src="https://avatars.githubusercontent.com/u/157936297?v=4?s=80" width="80px;" alt="Kaile Liu"/><br /><sub><b>Kaile Liu</b></sub></a><br /><a href="https://github.com/syswonder/robonix/commits?author=kaileliu" title="Code">💻</a></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="20%"><a href="https://github.com/HeartLinked"><img src="https://avatars.githubusercontent.com/u/78212101?v=4?s=80" width="80px;" alt="Feiyang Li"/><br /><sub><b>Feiyang Li</b></sub></a><br /><a href="https://github.com/syswonder/robonix/commits?author=HeartLinked" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/1mujue"><img src="https://avatars.githubusercontent.com/u/115391890?v=4?s=80" width="80px;" alt="MuJue"/><br /><sub><b>MuJue</b></sub></a><br /><a href="https://github.com/syswonder/robonix/commits?author=1mujue" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/LittleRookie1115"><img src="https://avatars.githubusercontent.com/u/157590849?v=4?s=80" width="80px;" alt="Zhenyu Zhang"/><br /><sub><b>Zhenyu Zhang</b></sub></a><br /><a href="https://github.com/syswonder/robonix/commits?author=LittleRookie1115" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/lhw2002426"><img src="https://avatars.githubusercontent.com/u/75192950?v=4?s=80" width="80px;" alt="lhw2002426"/><br /><sub><b>lhw2002426</b></sub></a><br /><a href="https://github.com/syswonder/robonix/commits?author=lhw2002426" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/zhengzihaoPKU"><img src="https://avatars.githubusercontent.com/u/141690701?v=4?s=80" width="80px;" alt="Zihao Zheng"/><br /><sub><b>Zihao Zheng</b></sub></a><br /><a href="https://github.com/syswonder/robonix/commits?author=zhengzihaoPKU" title="Code">💻</a></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="20%"><a href="https://github.com/QingFeng34048"><img src="https://avatars.githubusercontent.com/u/202889188?v=4?s=80" width="80px;" alt="qingfeng123"/><br /><sub><b>qingfeng123</b></sub></a><br /><a href="#tool-QingFeng34048" title="Tools">🔧</a> <a href="#data-QingFeng34048" title="Data">🔣</a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!

## License

Mulan Permissive Software License, Version 2 (MulanPSL-2.0). See [LICENSE](LICENSE).
