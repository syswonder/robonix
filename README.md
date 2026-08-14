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
  <a href="https://github.com/syswonder/robonix/blob/main/LICENSE"><img src="https://img.shields.io/badge/license-MulanPSL--2.0-red?style=flat-square" alt="License" /></a>
  <a href="https://github.com/syswonder/robonix/graphs/contributors"><img src="https://img.shields.io/github/contributors/syswonder/robonix?color=blue&style=flat-square" alt="Contributors" /></a>
  <img src="https://img.shields.io/github/languages/code-size/syswonder/robonix?color=green&style=flat-square" alt="Code size" />
  <img src="https://img.shields.io/github/repo-size/syswonder/robonix?color=lightgray&style=flat-square" alt="Repo size" />
  <img src="https://img.shields.io/github/languages/top/syswonder/robonix?color=orange&style=flat-square" alt="Top language" />
  <a href="https://packages.robonix.ai/packages/"><img src="https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fpackages.robonix.ai%2Fapi%2Fv1%2Fpackages&query=%24.packages.length&label=Robonix%20packages&color=0f766e&style=flat-square" alt="Robonix packages" /></a>
  <a href="#supported-robots"><img src="https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fpackages.robonix.ai%2Fapi%2Fv1%2Frobots&query=%24.robots.length&label=Robot%20deployments&color=2563eb&style=flat-square" alt="Published robot deployments" /></a>
</p>

<br />

## Robonix

Robonix is an operating system for embodied intelligence.

Embodied cognition holds that intelligence does not come from representation alone. It emerges from a body interacting with an environment: brain and body act together, and general capability is learned through a closed perceive–plan–act loop with the world. That leaves a question of system design rather than of learning — **how should an embodied "brain" be built and run** so a machine can work this way at all?

Recent answers come almost entirely from the model side: vision-language-action models, world models that aim to understand and predict an environment, and proposed architectures for an embodied brain. They ask what to learn and how to learn it. We think the brain also has to be *supported* — developed, deployed, executed, and managed on real hardware — and that this is what an operating system is for. Robonix is our attempt to build one.

The approach is to decouple models from bodies. Robonix treats AI models and skills as programs and exposes robot hardware as discoverable capabilities, so a body is integrated once while models and skills are written against shared interfaces for cameras, lidar, chassis, arms, mapping, navigation, speech, and more. The goal is simple to state: **train once, deploy on any robot**.

Around that, the concerns common to every perceive–understand–plan–act loop are factored into system services spanning perception, interconnection, cognition, and control, so that each robot does not re-implement them. What we want from this is an ecosystem in which embodied software and hardware can advance independently of one another.

## Supported Robots

[![robots](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fpackages.robonix.ai%2Fapi%2Fv1%2Frobots&query=%24.robots.length&label=robots&color=2563eb&style=flat-square)](https://packages.robonix.ai/robots/)

Robot bodies published to the catalog, with more on the way: wheeled, tracked, and quadruped bases, fixed and dual arms, standalone dexterous hands, and two simulated bodies. They span several vendors' chassis SDKs, both ROS 1 and ROS 2, and both grippers and five-finger hands, while running the same system services, capability contracts, and skills.

<table>
  <tr>
    <td align="center" width="25%"><a href="https://packages.robonix.ai/robots/robonix.robot.agilex.ranger_mini_v3/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.agilex.ranger_mini_v3-380.webp" width="240" alt="AgileX Ranger Mini v3" /><br /><sub><b>AgileX Ranger Mini v3</b></sub></a></td>
    <td align="center" width="25%"><a href="https://packages.robonix.ai/robots/robonix.robot.deep_robotics.lite3/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.deep_robotics.lite3-380.webp" width="240" alt="DEEP Robotics Lite3" /><br /><sub><b>DEEP Robotics Lite3</b></sub></a></td>
    <td align="center" width="25%"><a href="https://packages.robonix.ai/robots/robonix.robot.deep_robotics.lynx_s10/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.deep_robotics.lynx_s10-380.webp" width="240" alt="DEEP Robotics Lynx S10" /><br /><sub><b>DEEP Robotics Lynx S10</b></sub></a></td>
    <td align="center" width="25%"><a href="https://packages.robonix.ai/robots/robonix.robot.unitree.go2/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.unitree.go2-380.webp" width="240" alt="Unitree Go2" /><br /><sub><b>Unitree Go2</b></sub></a></td>
  </tr>
  <tr>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.yobotics.y20w/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.yobotics.y20w-380.webp" width="240" alt="Yobotics Y20W" /><br /><sub><b>Yobotics Y20W</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.wheeltec.r550/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.wheeltec.r550-380.webp" width="240" alt="WHEELTEC R550 mini_tank" /><br /><sub><b>WHEELTEC R550</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.yahboom.rosmaster_x3/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.yahboom.rosmaster_x3-380.webp" width="240" alt="Yahboom ROSMASTER X3" /><br /><sub><b>Yahboom ROSMASTER X3</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.beingbeyond.d1/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.beingbeyond.d1-380.webp" width="240" alt="BeingBeyond D1" /><br /><sub><b>BeingBeyond D1</b></sub></a></td>
  </tr>
  <tr>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.agilex.dual_piper/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.agilex.dual_piper-380.webp" width="240" alt="AgileX dual Piper" /><br /><sub><b>AgileX Dual Piper</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.wowrobo.roboarm/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.wowrobo.roboarm-380.webp" width="240" alt="WowRobo Roboarm" /><br /><sub><b>WowRobo Roboarm</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.linkerbot.linker_hand_o6/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.linkerbot.linker_hand_o6-380.webp" width="240" alt="LinkerBot LinkerHand O6" /><br /><sub><b>LinkerHand O6</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.syswonder.minecraft_bot/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.syswonder.minecraft_bot-380.webp" width="240" alt="Minecraft Bot" /><br /><sub><b>Minecraft Bot (sim)</b></sub></a></td>
  </tr>
  <tr>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.pal_robotics.tiago_webots/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.pal_robotics.tiago_webots-380.webp" width="240" alt="Webots TIAGo Lite" /><br /><sub><b>Webots TIAGo Lite (sim)</b></sub></a></td>
    <td align="center"><a href="https://packages.robonix.ai/robots/robonix.robot.hantewin.benben/"><img src="https://packages.robonix.ai/assets/previews/robonix.robot.hantewin.benben-380.webp" width="240" alt="Hantewin Benben" /><br /><sub><b>Hantewin Benben</b></sub></a></td>
  </tr>
</table>

| Robot | Integrated hardware | Maintained by | Deployment | Catalog |
| --- | --- | --- | --- | --- |
| AgileX Ranger Mini v3 | Ranger Mini v3 chassis; Livox MID-360 lidar and IMU; Intel RealSense D435i RGB-D camera; optional AgileX Piper arm; audio | syswonder | [link](https://github.com/syswonder/robot-agilex-ranger_mini_v3) | [link](https://packages.robonix.ai/robots/robonix.robot.agilex.ranger_mini_v3/) |
| DEEP Robotics Lite3 | Lite3 quadruped chassis; Livox MID-360 lidar and IMU; Orbbec Gemini 330-series RGB-D camera | [Bunnycxk](https://github.com/Bunnycxk) | [link](https://github.com/syswonder/robot-deep_robotics-lite3) | [link](https://packages.robonix.ai/robots/robonix.robot.deep_robotics.lite3/) |
| DEEP Robotics Lynx S10 | Lynx S10 wheeled-quadruped chassis over UDP; Orbbec Gemini 336L RGB-D camera; InternVLA vision-language navigation | [1mujue](https://github.com/1mujue) | [link](https://github.com/syswonder/robot-deep_robotics-lynx_s10) | [link](https://packages.robonix.ai/robots/robonix.robot.deep_robotics.lynx_s10/) |
| Unitree Go2 | Go2 quadruped chassis; onboard lidar, camera, and IMU; audio bridge | [Origamii520](https://github.com/Origamii520) | [link](https://github.com/syswonder/robot-unitree-go2) | [link](https://packages.robonix.ai/robots/robonix.robot.unitree.go2/) |
| Yobotics Y20W | Y20W chassis motion and posture; Livox MID-360 lidar; Intel RealSense D435i RGB-D camera; speech interaction and scene understanding | [chenx1118](https://github.com/chenx1118) | [link](https://github.com/chenx1118/robot-yobotics-y20w) | [link](https://packages.robonix.ai/robots/robonix.robot.yobotics.y20w/) |
| WHEELTEC R550 mini_tank | R550 tracked chassis and IMU; LSLIDAR N10P; Orbbec Astra S RGB-D camera | [sherry-part](https://github.com/sherry-part) | [link](https://github.com/syswonder/robot-wheeltec-r550) | [link](https://packages.robonix.ai/robots/robonix.robot.wheeltec.r550/) |
| Yahboom ROSMASTER X3 | ROSMASTER X3 mecanum chassis on Jetson TX2 NX; RPLidar; guarded short-distance ROS 1 `move_base` navigation over rosbridge | [luoyg0831-a11y](https://github.com/luoyg0831-a11y) | [link](https://github.com/luoyg0831-a11y/robot-yahboom-rosmaster-x3) | [link](https://packages.robonix.ai/robots/robonix.robot.yahboom.rosmaster_x3/) |
| BeingBeyond D1 | Fixed-base 6-DOF arm; 2-DOF head pan/tilt; five-finger dexterous hand; head RGB-D camera; VLM and YOLO-OBB detection with pick, place, stack, and sort skills | [Ciliphen](https://github.com/Ciliphen) | [link](https://github.com/syswonder/robot-beingbeyond-d1) | [link](https://packages.robonix.ai/robots/robonix.robot.beingbeyond.d1/) |
| AgileX Dual Piper | Two AgileX Piper arms and factory CAN grippers on independent buses; per-arm joint health telemetry; guarded dual-arm initialization; audio | [LittleRookie1115](https://github.com/LittleRookie1115) | [link](https://github.com/syswonder/robot-agilex-dual-piper) | [link](https://packages.robonix.ai/robots/robonix.robot.agilex.dual_piper/) |
| WowRobo Roboarm | Five-axis LeRobot Koch arm; Orbbec Gemini 215 RGB-D camera; audio | [gaoyz1235](https://github.com/gaoyz1235) | [link](https://github.com/syswonder/robot-wowrobo-roboarm) | [link](https://packages.robonix.ai/robots/robonix.robot.wowrobo.roboarm/) |
| LinkerBot LinkerHand O6 | Standalone six-axis five-finger dexterous hand over CAN; no arm, base, or camera; gesture and finger-motion skills | [Ciliphen](https://github.com/Ciliphen) | [link](https://github.com/syswonder/robot-linkerbot-linker_hand_o6) | [link](https://packages.robonix.ai/robots/robonix.robot.linkerbot.linker_hand_o6/) |
| Webots TIAGo Lite (simulation) | Simulated differential-drive chassis; head RGB-D camera; Hokuyo planar lidar; audio; runs the full stack with no robot hardware | syswonder | [link](https://github.com/syswonder/robot-pal_robotics-tiago_webots) | [link](https://packages.robonix.ai/robots/robonix.robot.pal_robotics.tiago_webots/) |
| Minecraft Bot (simulation) | Minecraft player body; camera, chassis, world-state, inventory, navigation, and exploration providers | [ZZJJWarth](https://github.com/ZZJJWarth) | [link](https://github.com/syswonder/robot-syswonder-minecraft_bot) | [link](https://packages.robonix.ai/robots/robonix.robot.syswonder.minecraft_bot/) |
| Hantewin Benben | Benben chassis; Livox MID-360 lidar and IMU; LSLIDAR LakiBeam1; Intel RealSense camera; audio; mapping, navigation, and speech | [Futaba19-c](https://github.com/Futaba19-c) | [link](https://github.com/syswonder/robot-hantewin-benben) | [link](https://packages.robonix.ai/robots/robonix.robot.hantewin.benben/) |

Each deployment links the complete robot manifest and its primitive, service, and skill dependencies. Published deployment metadata does not replace the hardware-specific safety, commissioning, and acceptance gates documented by each repository. See the [robot catalog](https://packages.robonix.ai/robots/) for published integrations.

## Packages

Each robot above is assembled from packages rather than written as one program. A package declares the capabilities it provides against contracts shared across every robot, so what a package offers does not depend on which body it was written for, and a deployment can swap one implementation for another without the layers above noticing.

There are three kinds of provider:

| Kind | Provides |
| --- | --- |
| **primitive** | One device — cameras, lidar, chassis, arms, grippers, audio |
| **service** | Runtime functionality built on those devices — mapping, navigation, grasp pose, memory, perception |
| **skill** | Reusable execution flows — grasp, place, transfer, explore, data collection |

[![packages](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fpackages.robonix.ai%2Fapi%2Fv1%2Fpackages&query=%24.packages.length&label=packages&color=0f766e&style=flat-square)](https://packages.robonix.ai/packages/)

Browse them in the [package catalog](https://packages.robonix.ai/packages/), or publish your own with the [package integration guide](https://book.robonix.ai/integration-guide/package-catalog).

## Host Platforms

| Arch    | OS / Distribution                                  | Status     |
| ------- | -------------------------------------------------- | ---------- |
| x86\_64 | Ubuntu 22.04                                       | ✅ Tested  |
| x86\_64 | Debian 13                                          | ✅ Tested  |
| arm64   | NVIDIA Jetson — JetPack 6.2 (L4T 36.4.3, Ubuntu 22.04) | ✅ Tested  |
| x86\_64 / arm64 | Ubuntu 24.04 and newer                     | 🚧 Planned |

"Tested" means the full Robonix pipeline runs end-to-end on that platform — in simulation or on a real robot: voice & interaction, task execution, body movement, scene & mapping (semantic map + spatial map), navigation, and skill execution. Other Linux distributions will likely work but are not regularly verified.

Capability providers that use ROS 2 are built and tested against [ROS 2 Humble](https://docs.ros.org/en/humble/).

## Quick Start

### Prerequisites

Install these from their own documentation first — Robonix does not provide or install them for you.

| Tool | Why it is needed | Install |
| --- | --- | --- |
| **Rust** (stable) | The system components are Rust; `make install` builds them with cargo | [rustup.rs](https://rustup.rs/) |
| **uv** | Resolves and runs the Python workspace — services and primitives | [docs.astral.sh/uv](https://docs.astral.sh/uv/getting-started/installation/) |
| **Docker** | Runs the Webots simulator stack, and any capability provider you choose to containerise | [docs.docker.com](https://docs.docker.com/engine/install/) |

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

This builds the system components and the `rbnx` CLI into `~/.cargo/bin`. See [Host Platforms](#host-platforms) for what is regularly tested.

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

## System Architecture

Robonix divides the work of an embodied brain across twelve system components, keeping planning, execution, state, and health separate from individual hardware drivers and skills. In the operating-system analogy, models and skills are programs and robot capabilities are resources: running plans have explicit identity and state, so they can be observed, steered, and cancelled without every skill re-implementing those mechanisms.

| Component                        | What it is                                                                               |
| -------------------------------- | ---------------------------------------------------------------------------------------- |
| **[atlas](system/atlas/)**       | The catalog of every running primitive / service / skill and its contract; components find and connect to capabilities through it |
| **[soma](system/soma/)**         | Serves the robot's body description — `soma.yaml` and its URDF — to every other component |
| **[scene](system/scene/)**       | Current best estimate of the environment: tracked objects with pose and class, their relations, and a 2D occupancy grid |
| **[pilot](system/pilot/)**       | Builds the prompt from the capability catalog, asks the VLM, and turns the answer into an executable plan |
| **[executor](system/executor/)** | Executes those plans — dispatches each step to a primitive, service, or skill through atlas, with observable task state |
| **[liaison](system/liaison/)**   | The user-input gateway in front of pilot: text and push-to-talk voice, identity, and access policy |
| **[vitals](system/vitals/)**     | Monitors onboard health — temperatures, voltage, joint motors — and evaluates thresholds |
| **[scribe](system/scribe/)**     | The structured logging library every component writes its journal through |
| **[sentinel](system/sentinel/)** | Decides whether a capability call is allowed under the current robot state, operator, and policy |
| **[keystone](system/keystone/)** | Stores the body's identity, persistent configuration, and operator access policy |
| **[chronos](system/chronos/)**   | A single time source that aligns timestamps across sensors, actuators, and components |
| **[nexus](system/nexus/)**       | Inter-component communication over gRPC, MCP, and ROS 2 |

The contracts these components and every package implement live in [`capabilities/`](capabilities/). Reference implementations of the built-in services ship in [`services/`](services/); a deployment may replace any of them, and primitives and skills live in their own repositories.

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
      <td align="center" valign="top" width="20%"><a href="https://github.com/zhengzihaoPKU"><img src="https://avatars.githubusercontent.com/u/141690701?v=4?s=80" width="80px;" alt="Zihao Zheng"/><br /><sub><b>Zihao Zheng</b></sub></a><br /><a href="#tool-zhengzihaoPKU" title="Tools">🔧</a> <a href="#data-zhengzihaoPKU" title="Data">🔣</a></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="20%"><a href="https://github.com/QingFeng34048"><img src="https://avatars.githubusercontent.com/u/202889188?v=4?s=80" width="80px;" alt="qingfeng123"/><br /><sub><b>qingfeng123</b></sub></a><br /><a href="#tool-QingFeng34048" title="Tools">🔧</a> <a href="#data-QingFeng34048" title="Data">🔣</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/ohhhHwH"><img src="https://avatars.githubusercontent.com/u/76088492?v=4?s=80" width="80px;" alt="longyunhou"/><br /><sub><b>longyunhou</b></sub></a><br /><a href="https://github.com/syswonder/robonix/commits?author=ohhhHwH" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/Origamii520"><img src="https://avatars.githubusercontent.com/u/214653863?v=4?s=80" width="80px;" alt="Origamii520"/><br /><sub><b>Origamii520</b></sub></a><br /><a href="https://github.com/syswonder/robonix/commits?author=Origamii520" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/Bunnycxk"><img src="https://avatars.githubusercontent.com/u/41613268?v=4?s=80" width="80px;" alt="Xiankun Chen"/><br /><sub><b>Xiankun Chen</b></sub></a><br /><a href="https://github.com/syswonder/robonix/commits?author=Bunnycxk" title="Code">💻</a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!

## License

Mulan Permissive Software License, Version 2 (MulanPSL-2.0). See [LICENSE](LICENSE).
