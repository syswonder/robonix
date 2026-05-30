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

## Quickstart

```bash
git clone --recursive https://github.com/syswonder/robonix
cd robonix
make install   # builds the Cargo workspace and installs
               # rbnx + robonix-{atlas,pilot,executor,liaison,codegen}
               # to ~/.cargo/bin, then registers this clone via `rbnx setup`
```

The Webots Tiago example (`examples/webots/`) is the standard end-to-end demo. Two terminals — the simulator and Robonix itself.

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
cd examples/webots && rbnx shutdown    # reads rbnx-boot/state.json, SIGTERMs each component's PGID
bash examples/webots/sim/stop.sh       # then stop the Webots container
```

Full first-run walkthrough: [**docs/src/getting-started/quickstart.md**](https://github.com/syswonder/robonix-book/blob/main/src/getting-started/quickstart.md).

## What it is

Robonix is the **system substrate** between a robot's hardware and an embodied LLM/VLA brain. It standardises how device drivers, runtime services, user skills, and the planner discover and talk to each other; it owns identity, configuration, time, transport, logging, health, body model, scene model, execution, and safety as named, replaceable components.

The whitepaper ships **12 system components** organised by role (not by implementation language):

| Component | What it owns |
| --- | --- |
| **[atlas](system/atlas/)**       | capability discovery / catalog                    |
| **[chronos](system/chronos/)**   | unified time / PTP alignment              *(stub)*  |
| **[executor](system/executor/)** | capability orchestration & dispatch              |
| **[keystone](system/keystone/)** | identity / configuration / policy         *(stub)*  |
| **[liaison](system/liaison/)**   | human–machine interaction (chat / audio / TUI)   |
| **[nexus](system/nexus/)**       | high-performance transport abstraction    *(stub)*  |
| **[pilot](system/pilot/)**       | planning / decision / memory / world model       |
| **[scene](system/scene/)**       | scene state / semantic map / object registry     |
| **[scribe](system/scribe/)**     | structured logs / replay / audit          *(stub)*  |
| **[sentinel](system/sentinel/)** | safety supervision           *(in executor for v0.1)*  |
| **[soma](system/soma/)**         | body state / device & primitive abstraction *(stub)* |
| **[vitals](system/vitals/)**     | health monitoring / heartbeat *(partial via atlas)*  |

On top of system, three open application categories — provided as contracts (61 standard interfaces in `capabilities/`) and reference implementations alongside the system:

- **primitive** — one device per package (camera, lidar, chassis, arm). Lives in deployment repos and per-example folders (e.g. `examples/webots/primitives/`).
- **service** — runtime functionality (mapping, navigation, semantic map, memory, speech, voiceprint). Default reference implementations ship in [`services/`](services/); each can be swapped out by a deployment.
- **skill** — user-defined reusable execution flows (grasp, place, explore, fold-clothes …). Lives wherever the deploy/integrator wants.

## Repository layout

```
system/         the 12 system components, one directory each
services/       default reference service implementations (memsearch, voiceprint, speech)
pylib/          Python SDK (robonix-api on PyPI)
capabilities/   contract TOMLs + ROS-style IDL tree (capabilities/lib/)
tools/          dev tooling — rbnx CLI + codegen
examples/       end-to-end deployments (webots, test_ci)
docs/           mdBook developer guide (submodule)
Cargo.toml      workspace for the Rust components (4 in system/, 2 in tools/)
Makefile        top-level orchestrate (build / install / fmt / check)
```

`system/<name>/` and `services/<name>/` and `tools/<name>/` are each self-contained packages — Rust ones carry their own `Cargo.toml`, Python ones their own `pyproject.toml`. There is no top-level `rust/` / `python/` split; the runtime role is what determines where a component lives, not the implementation language.

## Architecture

Robonix is a **capability-centric runtime** built around one idea: every operation a robot can perform — read a camera frame, drive the chassis, plan a multi-step task, persist a memory — is registered with the system as a typed *capability*. Discovery, connection, lifecycle, and safety are all expressed in terms of those capabilities, not in terms of which process or transport happens to provide them today.

The control plane is `atlas`. On startup every provider — a hardware driver (primitive), a runtime service (mapping, voiceprint, …), a high-level skill — calls `RegisterPrimitive` / `RegisterService` / `RegisterSkill` and then issues one `DeclareCapability(contract_id, transport, endpoint, params)` per interface it serves. Consumers (`pilot`, `executor`, downstream services) discover providers via `QueryCapabilities` and open data-plane channels via `ConnectCapability`. Transports are pluggable — gRPC, MCP, ROS 2 — and the TOML contracts under `capabilities/` are the binding agreement that lets the same contract id be served over any of them.

A single user task — "tell me what you see and put the cup on the table" — flows through the stack as follows:

1. **liaison** turns mic audio (or text) into a structured task, gates it through voiceprint identity + the access-control policy, and forwards it to pilot.
2. **pilot** asks atlas which capabilities are currently `ACTIVE`, builds a prompt from their `CAPABILITY.md` blurbs, asks the VLM for a step-by-step plan, then sends the plan to executor.
3. **executor** dispatches each plan step as a capability call. Before every call it consults **sentinel**, which can deny based on rate limit, deny-window, or identity policy.
4. The provider (a primitive, a service, or a skill) runs the call and returns a result. Results that came from perception are reconciled by **scene**'s semantic-map layer, so the next round of planning sees an updated world.
5. liaison streams the agent's narration back to the user as TTS audio while executor keeps working.

Startup of the stack itself follows the same dependency direction. First the L0 services (`chronos`, `atlas`, `nexus`, `scribe`) come up — they own time, the capability catalog, transport, and logs. Then `soma` enumerates hardware and registers primitives, `scene` begins consuming perception and builds an object registry, `sentinel` loads safety rules, `executor` subscribes to the capability catalog, and finally `pilot + liaison` load LLM credentials and open the user channel. At that point `rbnx caps` shows every component `ACTIVE` and the system is ready to accept tasks.

For the long form — full RPC tables, lifecycle state machine, contract grammar, codegen pipeline — see the developer guide:

- [**Overview**](https://github.com/syswonder/robonix-book/blob/main/src/architecture/overview.md) — one full request, end-to-end
- [**Namespaces & contracts**](https://github.com/syswonder/robonix-book/blob/main/src/architecture/namespace-and-interfaces.md) — how `robonix/primitive/*` / `robonix/service/*` / `robonix/skill/*` / `robonix/system/*` are wired
- [**Interface catalog**](https://github.com/syswonder/robonix-book/blob/main/src/interface-catalog/index.md) — every primitive + service contract Robonix ships

## Status

> [!WARNING]
> Robonix is in early development. APIs, IDL layouts, and internal designs may change without notice. No API stability until a versioned release.

5 of the 12 system components are implemented today (atlas, executor, liaison, pilot, scene); sentinel runs as an executor sub-module; vitals' heartbeat half lives in atlas. The remaining stubs are tracked under `system/<name>/README.md` with v0.2 plans.

## Reference packages

A set of packages you can adopt directly, fork, or read as templates when writing your own primitives / services / skills. The whitepaper interfaces (61 standard contracts in `capabilities/`) are the binding agreement — the implementations below are *one* way to satisfy them. Swap any of them out by pointing your deploy manifest at a different repo.

### Default reference services (this repo)

Live in [`services/`](services/) — installed by default on a fresh boot, easy to replace per deploy.

| Package | Contract namespace | What it implements |
| --- | --- | --- |
| [`services/memsearch`](services/memsearch/)   | `robonix/service/memory/*`    | SQLite + vector-index memory store (save / search / compact) |
| [`services/voiceprint`](services/voiceprint/) | `robonix/service/voiceprint/*` | ECAPA-TDNN voiceprint enrolment + identification |
| [`services/speech`](services/speech/)         | `robonix/service/speech/*`     | FunASR-based STT + TTS dialog service |

### Example primitives — Webots Tiago demo

Live in [`examples/webots/primitives/`](examples/webots/primitives/). Each one is a 100-200 LoC Python package — a clean read-along for writing a driver for your own hardware.

| Package | Contract namespace | Source |
| --- | --- | --- |
| `tiago_chassis`     | `robonix/primitive/chassis/*` | `/cmd_vel` + `/amcl_pose` bridge |
| `tiago_camera`      | `robonix/primitive/camera/*`  | RGB + depth + snapshot/depth_snapshot MCP |
| `tiago_lidar`       | `robonix/primitive/lidar/*`   | 2D laser scan bridge |
| `audio_driver`      | `robonix/primitive/audio/*`   | Host-side ALSA mic + speaker |
| `audio_macos_bridge`| `robonix/primitive/audio/*`   | macOS CoreAudio mic + speaker over the same contract |

### Upstream packages

Pulled by `rbnx build` from a deploy manifest's `url:`/`branch:` entries. Each repo is small, self-contained, and a reference for a specific category.

| Repo | Category | What it does |
| --- | --- | --- |
| [`mapping_rbnx`](https://github.com/enkerewpo/mapping_rbnx)                  | service (mapping)        | RTAB-Map / FAST-LIO2 SLAM behind `service/map/occupancy_grid` |
| [`nav2_wrapper_rbnx`](https://github.com/enkerewpo/nav2_wrapper_rbnx)        | service (navigation)     | Nav2 stack behind `service/navigation/{navigate,status,cancel}` |
| [`explore_rbnx`](https://github.com/enkerewpo/explore_rbnx)                  | skill (exploration)      | LLM-driven frontier exploration — example skill package |
| [`mid360_lidar_rbnx`](https://github.com/enkerewpo/mid360_lidar_rbnx)        | primitive (lidar)        | Livox MID-360 3D point cloud |
| [`mid360_imu_rbnx`](https://github.com/enkerewpo/mid360_imu_rbnx)            | primitive (imu)          | Livox MID-360 IMU |
| [`realsense_camera_rbnx`](https://github.com/enkerewpo/realsense_camera_rbnx)| primitive (camera)       | Intel RealSense D435i RGB + depth + snapshot MCP |
| [`ranger_chassis_rbnx`](https://github.com/enkerewpo/ranger_chassis_rbnx)    | primitive (chassis)      | AgileX Ranger Mini CAN-bus chassis |
| [`ranger_description_rbnx`](https://github.com/enkerewpo/ranger_description_rbnx) | primitive (description) | Ranger Mini URDF + meshes |
| [`ranger_mini_deploy`](https://github.com/enkerewpo/ranger_mini_deploy)      | deploy manifest          | Reference real-robot deploy combining the above |
| [`template_rbnx`](https://github.com/enkerewpo/template_rbnx)                | starter                  | Skeleton repo to fork when starting a new primitive / service / skill |

### Writing your own

Easiest path: clone [`template_rbnx`](https://github.com/enkerewpo/template_rbnx), pick a namespace under `robonix/primitive/...` / `robonix/service/...` / `robonix/skill/...`, declare the contract TOMLs you implement under your package's `capabilities/`, and write your handlers against the Python SDK (`pip install robonix-api`). The [developer guide](https://github.com/syswonder/robonix-book/blob/main/src/integration-guide/packaging-spec.md) walks through the full package layout; the packages above are concrete examples to read alongside.

## License

Mulan Permissive Software License, Version 2 (MulanPSL-2.0). See [LICENSE](LICENSE).
