<p align="center">
  <img src="images/robonix-logo.svg" alt="Robonix" width="420" />
</p>

<h3 align="center">Robonix — The Embodied AI Operating System</h3>

<p align="center">
  <em>A system substrate for building embodied intelligence across heterogeneous robots.</em>
</p>

<p align="center">
  <a href="https://robonix.ai"><b>robonix.ai</b></a>
  &nbsp;·&nbsp;
  <a href="https://book.robonix.ai/">Documentation</a>
  &nbsp;·&nbsp;
  <a href="https://packages.robonix.ai/">Package catalog</a>
  &nbsp;·&nbsp;
  <a href="#quick-start">Quick start</a>
</p>

<p align="center">
  <a href="https://github.com/syswonder/robonix/blob/main/LICENSE"><img src="https://img.shields.io/badge/license-MulanPSL--2.0-red" alt="License" /></a>
  <img src="https://img.shields.io/github/languages/code-size/syswonder/robonix?color=green" alt="Code size" />
  <img src="https://img.shields.io/github/repo-size/syswonder/robonix?color=lightgray" alt="Repo size" />
  <img src="https://img.shields.io/github/languages/top/syswonder/robonix?color=orange" alt="Top language" />
  <a href="https://packages.robonix.ai/packages/"><img src="https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fpackages.robonix.ai%2Fapi%2Fv1%2Fpackages&query=%24.packages.length&label=Robonix%20packages&color=0f766e" alt="Robonix packages" /></a>
  <a href="#supported-robots"><img src="https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fpackages.robonix.ai%2Fapi%2Fv1%2Frobots&query=%24.robots.length&label=Robot%20deployments&color=2563eb" alt="Published robot deployments" /></a>
  <a href="https://deepwiki.com/syswonder/robonix"><img src="https://deepwiki.com/badge.svg" alt="Ask DeepWiki"></a>
</p>
<br />

**Robonix** is an *agentic* operating system for embodied artificial intelligence (AI). It exposes a robot's hardware, services, and skills as typed capabilities. A large language model (LLM) translates a natural-language task into a program that orchestrates those capabilities; Robonix validates that program and then executes it under its supervision. The planning model is configurable — GPT, Qwen, etc., or one you host yourself.

Robonix ships with these benefits:

* **Simple Integration.** A body is integrated by implementing contracts for the hardware it exposes; everything above programs against capabilities rather than vendor APIs. A skill or task can transfer to another body that offers compatible contracts and meets the physical preconditions.
* **Skill Composition.** Long-horizon behavior is assembled from independently tested skills instead of learned end to end, so a failed step can be retried or replaced without restarting the run.
* **Observable Plans.** Plans are explicit and typed: what the model proposed, what the runtime admitted, which provider executed each call, and where it failed. Every system component writes structured logs, so a run leaves a machine-readable trail on disk.

## Design of Robonix

<p align="center">
  <img src="images/robonix-architecture.svg" alt="Robonix architecture" width="95%" />
</p>


​	The figure shows the overall architecture of Robonix. The user talks to **Liaison**, which hands the **task** to **Pilot**; Pilot plans it with the model and passes the plan to **Executor**, which **Sentinel** monitors, while **Scene** keeps the state of the world. Below these services sit six core components, and below them **Soma**, the body abstraction: its **skill library** of reusable behaviors calls the **primitives**, each of which exposes a single hardware function, and the primitives drive the robot hardware.

| Abstraction | Definition |
| --- | --- |
| **primitive** | A single hardware function — sensing or actuation — behind a software interface. |
| **service** | Shared functionality or state that tasks and skills draw on: scene, map, memory, navigation. |
| **skill** | A learned model or an algorithmic procedure, packaged as a reusable executable unit. |
| **task** | A goal, its constraints, and the conditions that count as completion. |

Each primitive, service, and skill declares what it offers in a versioned **contract**, and announces itself to [Atlas](system/atlas/) at startup. Callers bind to the contract rather than the implementation behind it, which lets a task transfer to another body and an implementation be replaced without changing its callers.

***The agent loop:*** [Pilot](system/pilot/) sends the model the goal, the capabilities currently available, and the state of the world; the model responds with an **RTDL** program—a graph whose nodes are capability calls, run in sequence or concurrently. Pilot validates that program and hands a plan to [Executor](system/executor/), which runs it and returns typed results step by step until the task is done. Branching and loops are in development.

The [System Architecture](#system-architecture) section below introduces the other components, such as Atlas, Liaison, Vitals, and Scribe.

## Quick Start

### Prerequisites

Install these from their own documentation first — Robonix does not provide or install them for you.

| Tool | Why it is needed | Install |
| --- | --- | --- |
| **Rust** (stable) | The system components are Rust; `make install` builds them with cargo | [rustup.rs](https://rustup.rs/) |
| **uv** | Resolves and runs the Python workspace — services and primitives | [docs.astral.sh/uv](https://docs.astral.sh/uv/getting-started/installation/) |
| **Docker** | Runs the Webots simulator stack, and any capability provider you choose to containerize | [docs.docker.com](https://docs.docker.com/engine/install/) |

Rust and uv install into your home directory, so put them on `PATH` before continuing:

```bash
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
```

### Install Robonix

```bash
git clone --recursive https://github.com/syswonder/robonix.git
cd robonix
make install
```

This builds the system components and the `rbnx` command-line interface (CLI) into `~/.cargo/bin`. See [Host Platforms](#host-platforms) for what is regularly tested.

Start the Webots simulator in one terminal:

```bash
export DISPLAY=:0
bash examples/webots/sim/start.sh
```

Boot Robonix in a second terminal with any OpenAI-compatible VLM endpoint:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export VLM_BASE_URL=https://api.openai.com/v1
export VLM_API_KEY=sk-...
export VLM_MODEL=your-model-name

cd examples/webots
rbnx build
rbnx boot
```

Then run `rbnx chat` in a third terminal. Try `go to room 101`, `what can you see?`, or `explore the office`. See the [Getting Started guide](https://book.robonix.ai/getting-started/quickstart) for the complete walkthrough.

## Quick Development

The [Robonix package template](https://github.com/syswonder/template-rbnx) contains a mock primitive, a service, and a skill that boot without robot hardware:

```bash
git clone https://github.com/syswonder/template-rbnx.git
cd template-rbnx
cp .env.example .env
# Fill in the three VLM values in .env.
set -a; source .env; set +a
rbnx build
rbnx boot
```

Run `rbnx caps` to inspect the live providers, then try `rbnx chat` and ask the robot to say hello. Each example package keeps its manifest, `config.spec`, build/start scripts, implementation, and optional capability definitions in one directory. Start there, then follow the [package integration guide](https://book.robonix.ai/integration-guide/package-catalog) to publish a reusable package.

## Supported Robots

[![robots](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fpackages.robonix.ai%2Fapi%2Fv1%2Frobots&query=%24.robots.length&label=robots&color=2563eb)](https://packages.robonix.ai/robots/)

Robot bodies published to the catalog, with more on the way: wheeled, tracked, and quadruped bases, fixed and dual arms, standalone dexterous hands, a humanoid, and four simulated bodies. They span several vendors' chassis software development kits (SDKs), both versions of the Robot Operating System (ROS 1 and ROS 2), and both grippers and five-finger hands, while running the same system services, capability contracts, and skills.

<table>
  <tr>
    <td align="center" width="16.6%"><a href="https://packages.robonix.ai/robots/robonix.robot.agilex.ranger_mini_v3/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.agilex.ranger_mini_v3-380.webp" width="160" alt="AgileX Ranger Mini v3" /><br /><sub><b>AgileX Ranger Mini v3</b></sub></a></td>
    <td align="center" width="16.6%"><a href="https://packages.robonix.ai/robots/robonix.robot.deep_robotics.lite3/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.deep_robotics.lite3-380.webp" width="160" alt="DEEP Robotics Lite3" /><br /><sub><b>DEEP Robotics Lite3</b></sub></a></td>
    <td align="center" width="16.6%"><a href="https://packages.robonix.ai/robots/robonix.robot.deep_robotics.lynx_s10/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.deep_robotics.lynx_s10-380.webp" width="160" alt="DEEP Robotics Lynx S10" /><br /><sub><b>DEEP Robotics Lynx S10</b></sub></a></td>
    <td align="center" width="16.6%"><a href="https://packages.robonix.ai/robots/robonix.robot.unitree.go2/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.unitree.go2-380.webp" width="160" alt="Unitree Go2" /><br /><sub><b>Unitree Go2</b></sub></a></td>
    <td align="center" width="16.6%"><a href="https://packages.robonix.ai/robots/robonix.robot.yobotics.y20w/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.yobotics.y20w-380.webp" width="160" alt="Yobotics Y20W" /><br /><sub><b>Yobotics Y20W</b></sub></a></td>
    <td align="center" width="16.6%"><a href="https://packages.robonix.ai/robots/robonix.robot.mirrorme.bpx/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.mirrorme.bpx-380.webp" width="160" alt="MirrorMe BPX" /><br /><sub><b>MirrorMe BPX</b></sub></a></td>
  </tr>
  <tr>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.wheeltec.r550/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.wheeltec.r550-380.webp" width="160" alt="WHEELTEC R550" /><br /><sub><b>WHEELTEC R550</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.yahboom.rosmaster_x3/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.yahboom.rosmaster_x3-380.webp" width="160" alt="Yahboom ROSMASTER X3" /><br /><sub><b>Yahboom ROSMASTER X3</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.hantewin.benben/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.hantewin.benben-380.webp" width="160" alt="Hantewin Benben" /><br /><sub><b>Hantewin Benben</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.beingbeyond.d1/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.beingbeyond.d1-380.webp" width="160" alt="BeingBeyond D1" /><br /><sub><b>BeingBeyond D1</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.agilex.dual_piper/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.agilex.dual_piper-380.webp" width="160" alt="AgileX Dual Piper" /><br /><sub><b>AgileX Dual Piper</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.wowrobo.roboarm/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.wowrobo.roboarm-380.webp" width="160" alt="WowRobo Roboarm" /><br /><sub><b>WowRobo Roboarm</b></sub></a></td>
  </tr>
  <tr>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.linkerbot.linker_hand_o6/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.linkerbot.linker_hand_o6-380.webp" width="160" alt="LinkerHand O6" /><br /><sub><b>LinkerHand O6</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.pal_robotics.tiago_webots/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.pal_robotics.tiago_webots-380.webp" width="160" alt="Webots TIAGo Lite (sim)" /><br /><sub><b>Webots TIAGo Lite (sim)</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.syswonder.minecraft_bot/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.syswonder.minecraft_bot-380.webp" width="160" alt="Minecraft Bot (sim)" /><br /><sub><b>Minecraft Bot (sim)</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.agilex.ranger_with_piper_mujoco/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.agilex.ranger_with_piper_mujoco-380.webp" width="160" alt="Ranger + Piper (sim)" /><br /><sub><b>Ranger + Piper (sim)</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.unitree.go2_mujoco/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.unitree.go2_mujoco-380.webp" width="160" alt="Unitree Go2 (sim)" /><br /><sub><b>Unitree Go2 (sim)</b></sub></a></td>
  </tr>
</table>

<details>
<summary><b>Per-robot hardware and links</b></summary>

| Robot | Hardware | Maintainer | Links |
| --- | --- | --- | --- |
| AgileX Ranger Mini v3 | Ranger Mini v3 chassis; Livox MID-360 lidar and IMU; RealSense D435i RGB-D; optional Piper arm; audio | syswonder | [repo](https://github.com/syswonder/robot-agilex-ranger_mini_v3) · [catalog](https://packages.robonix.ai/robots/robonix.robot.agilex.ranger_mini_v3/) |
| DEEP Robotics Lite3 | Lite3 quadruped; Livox MID-360 lidar and IMU; Orbbec Gemini 330 RGB-D | [Bunnycxk](https://github.com/Bunnycxk) | [repo](https://github.com/syswonder/robot-deep_robotics-lite3) · [catalog](https://packages.robonix.ai/robots/robonix.robot.deep_robotics.lite3/) |
| DEEP Robotics Lynx S10 | Lynx S10 wheeled-quadruped over UDP; Orbbec Gemini 336L RGB-D; InternVLA navigation | [1mujue](https://github.com/1mujue) | [repo](https://github.com/syswonder/robot-deep_robotics-lynx_s10) · [catalog](https://packages.robonix.ai/robots/robonix.robot.deep_robotics.lynx_s10/) |
| Unitree Go2 | Go2 quadruped; onboard lidar, camera, IMU; audio bridge | [Origamii520](https://github.com/Origamii520) | [repo](https://github.com/syswonder/robot-unitree-go2) · [catalog](https://packages.robonix.ai/robots/robonix.robot.unitree.go2/) |
| Yobotics Y20W | Y20W chassis and posture; Livox MID-360 lidar; RealSense D435i RGB-D; speech | [chenx1118](https://github.com/chenx1118) | [repo](https://github.com/syswonder/robot-yobotics-y20w) · [catalog](https://packages.robonix.ai/robots/robonix.robot.yobotics.y20w/) |
| MirrorMe BPX | BPX quadruped; odometry; guarded stand and sit posture control | [nonkr](https://github.com/nonkr) | [repo](https://github.com/mirrormerobotics/robot-mirrorme-bpx) · [catalog](https://packages.robonix.ai/robots/robonix.robot.mirrorme.bpx/) |
| WHEELTEC R550 | R550 tracked chassis and IMU; LSLIDAR N10P; Orbbec Astra S RGB-D | [sherry-part](https://github.com/sherry-part) | [repo](https://github.com/syswonder/robot-wheeltec-r550) · [catalog](https://packages.robonix.ai/robots/robonix.robot.wheeltec.r550/) |
| Yahboom ROSMASTER X3 | Mecanum chassis on Jetson TX2 NX; RPLidar; guarded ROS 1 navigation | [luoyg0831-a11y](https://github.com/luoyg0831-a11y) | [repo](https://github.com/syswonder/robot-yahboom-rosmaster_x3) · [catalog](https://packages.robonix.ai/robots/robonix.robot.yahboom.rosmaster_x3/) |
| Hantewin Benben | Benben chassis; Livox MID-360 and LSLIDAR LakiBeam1; RealSense camera; audio | [Futaba19-c](https://github.com/Futaba19-c) | [repo](https://github.com/syswonder/robot-hantewin-benben) · [catalog](https://packages.robonix.ai/robots/robonix.robot.hantewin.benben/) |
| BeingBeyond D1 | Fixed-base 6-DOF arm; 2-DOF head; five-finger hand; head RGB-D; pick, place, stack, sort | [Ciliphen](https://github.com/Ciliphen) | [repo](https://github.com/syswonder/robot-beingbeyond-d1) · [catalog](https://packages.robonix.ai/robots/robonix.robot.beingbeyond.d1/) |
| AgileX Dual Piper | Two Piper arms and CAN grippers on separate buses; per-arm joint telemetry | syswonder | [repo](https://github.com/syswonder/robot-agilex-dual_piper) · [catalog](https://packages.robonix.ai/robots/robonix.robot.agilex.dual_piper/) |
| WowRobo Roboarm | Five-axis LeRobot Koch arm; Orbbec Gemini 215 RGB-D; audio | [gaoyz1235](https://github.com/gaoyz1235) | [repo](https://github.com/syswonder/robot-wowrobo-roboarm) · [catalog](https://packages.robonix.ai/robots/robonix.robot.wowrobo.roboarm/) |
| LinkerHand O6 | Standalone six-axis five-finger hand over CAN; gesture and finger-motion skills | [Ciliphen](https://github.com/Ciliphen) | [repo](https://github.com/syswonder/robot-linkerbot-linker_hand_o6) · [catalog](https://packages.robonix.ai/robots/robonix.robot.linkerbot.linker_hand_o6/) |
| Webots TIAGo Lite (sim) | Simulated differential-drive base; head RGB-D; Hokuyo lidar; audio | syswonder | [repo](https://github.com/syswonder/robot-pal_robotics-tiago_webots) · [catalog](https://packages.robonix.ai/robots/robonix.robot.pal_robotics.tiago_webots/) |
| Minecraft Bot (sim) | Minecraft player body; camera, chassis, world state, inventory, navigation | [ZZJJWarth](https://github.com/ZZJJWarth) | [repo](https://github.com/syswonder/robot-syswonder-minecraft_bot) · [catalog](https://packages.robonix.ai/robots/robonix.robot.syswonder.minecraft_bot/) |
| Ranger + Piper (sim) | MuJoCo Ranger Mini v3 and Piper; web or native viewer; mapping, navigation, manipulation | syswonder | [repo](https://github.com/syswonder/robot-agilex-ranger_with_piper_mujoco) · [catalog](https://packages.robonix.ai/robots/robonix.robot.agilex.ranger_with_piper_mujoco/) |
| Unitree Go2 (sim) | MuJoCo Go2 quadruped; web or native viewer; exploration and semantic navigation | syswonder | [repo](https://github.com/syswonder/robot-unitree-go2-mujoco) · [catalog](https://packages.robonix.ai/robots/robonix.robot.unitree.go2_mujoco/) |

</details>

Each deployment links the complete robot manifest and its primitive, service, and skill dependencies. Published deployment metadata does not replace the hardware-specific safety, commissioning, and acceptance gates documented by each repository. See the [robot catalog](https://packages.robonix.ai/robots/) for published integrations.

## Packages

Every robot above is assembled from packages, not written as one program. A package declares the capabilities it provides against the shared contracts, so what it offers does not depend on which body it was written for, and a deployment can swap one implementation for another without the layers above noticing.

[![packages](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fpackages.robonix.ai%2Fapi%2Fv1%2Fpackages&query=%24.packages.length&label=packages&color=0f766e)](https://packages.robonix.ai/packages/)

Browse them in the [package catalog](https://packages.robonix.ai/packages/), or publish your own with the [package integration guide](https://book.robonix.ai/integration-guide/package-catalog).

## Host Platforms

The Rust system components and the Python packages are written to stay portable across architectures; x86-64, arm64 (NVIDIA Jetson), and LoongArch64 are tested.

| Arch    | OS / Distribution                                  | Status     |
| ------- | -------------------------------------------------- | ---------- |
| x86\_64 | Ubuntu 22.04                                       | ✅ Tested  |
| x86\_64 | Debian 13                                          | ✅ Tested  |
| arm64   | NVIDIA Jetson — JetPack 6.2 (L4T 36.4.3, Ubuntu 22.04) | ✅ Tested  |
| LoongArch64 | Loongson 3A6000 + AMD Radeon 7900 XTX — Loong ArchLinux 2026.08.07 | ✅ Tested |
| x86\_64 / arm64 | Ubuntu 24.04 and newer                     | 🚧 Planned |

> **Note (LoongArch64):** Robonix itself runs on the Loongson 3A6000 host, while the simulation platform (Webots) runs on a separate x86\_64 machine with Ubuntu 22.04. The two machines are connected over Ethernet on the same local area network (LAN).

"Tested" means the full Robonix pipeline runs end-to-end on that platform — in simulation or on a real robot: voice & interaction, task execution, body movement, scene & mapping (semantic map + spatial map), navigation, and skill execution. Other Linux distributions will likely work but are not regularly verified.

Capability providers that use ROS 2 are built and tested against [ROS 2 Humble](https://docs.ros.org/en/humble/).

## System Architecture

Six components form the core. A deployment cannot replace them; everything else is built on top.

| Core | Responsibility |
| --- | --- |
| **[atlas](system/atlas/)** | Capability catalog — every running provider and its contract |
| **[nexus](system/nexus/)** | Communication over gRPC, the Model Context Protocol (MCP), and ROS 2 |
| **[keystone](system/keystone/)** | Identity, configuration, and access policy |
| **[chronos](system/chronos/)** | One time source across sensors, actuators, and components |
| **[scribe](system/scribe/)** | Structured logging |
| **[vitals](system/vitals/)** | Onboard health: temperatures, voltage, joint motors |

Everything above the core is a service, and a deployment can replace any of it. Robonix ships a default implementation for the ones below, and for others such as navigation and mapping.

| Service | Responsibility |
| --- | --- |
| **[pilot](system/pilot/)** | Turns a task into an RTDL program with the model, and validates it |
| **[executor](system/executor/)** | Runs the program, dispatching each call to its provider |
| **[soma](system/soma/)** | The robot's body description and state |
| **[scene](system/scene/)** | The environment: objects, their relations, and an occupancy grid |
| **[sentinel](system/sentinel/)** | Decides whether a capability call is allowed |
| **[liaison](system/liaison/)** | User input — text and voice |

Every contract lives in [`capabilities/`](capabilities/), whether a core component, a service, or a package implements it. Primitives and skills live in their own repositories. Some services still sit under `system/` in the source tree, and some capability interface names still carry their old spelling; both will migrate soon.

## Documentation

Full documentation lives at **[book.robonix.ai](https://book.robonix.ai/)**.

**Getting started**

* [Quickstart](https://book.robonix.ai/getting-started/quickstart) — the full version of the Webots walkthrough in this README
* [Host Platforms](#host-platforms) — what is tested, and what is not

**Understanding the system**

* [Architecture overview](https://book.robonix.ai/architecture/components) — the control plane, and one full request end to end
* [Namespaces & contracts](https://book.robonix.ai/architecture/namespace-and-contracts) — how `robonix/primitive/*`, `robonix/service/*`, `robonix/skill/*`, and `robonix/system/*` relate
* [Interface catalog](https://book.robonix.ai/interface-catalog/) — every primitive and service contract, generated from `capabilities/`

**Building on it**

* [Package integration guide](https://book.robonix.ai/integration-guide/package-catalog) — write a package and publish it to the catalog
* [Package catalog](https://packages.robonix.ai/packages/) — every published package, browsable by kind
* [Robot catalog](https://packages.robonix.ai/robots/) — every published deployment, with its full dependency tree

## Changelog

Release history lives in [CHANGELOG.md](CHANGELOG.md), formatted per [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Contributors add entries under `## [Unreleased]`; they are moved into a versioned section at release time.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for the repository's license headers, code style, validation commands, commit format, human-authorship policy, and AI assistance disclosure rules.

## License

Mulan Permissive Software License, Version 2 (MulanPSL-2.0). See [LICENSE](LICENSE).
