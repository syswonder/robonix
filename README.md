# DeepEmbody

**DeepEmbody** is an open-source embodied intelligence framework designed to provide the following features:

* [x] Support for multi-process/task management, including various task types such as ROS2 nodes, Python scripts, and C++ programs.
* [ ] Native support for LLM/VLM-based MCP service invocation.
* [ ] Lifecycle management for tasks/nodes/services, including auto-restart and auto-recovery.
* [ ] Task/node/service scheduling services, including priority-based and resource-based scheduling.
* [ ] Logging management for tasks/nodes/services, including real-time logs, historical logs, and alert logs.

---

## Architecture

```
.
├── brain
├── capability
│   ├── example_hello
│   │   ├── api
│   │   ├── config
│   │   ├── description.yml
│   │   ├── README.md
│   │   └── src
│   │       └── hello.py
│   └── example_ping
│       ├── api
│       ├── config
│       ├── description.yml
│       └── README.md
├── config
├── cost
├── driver
├── infra
├── manager
├── memory
├── README.md
└── skill
```

* **driver:** The hardware abstraction layer, responsible for abstracting hardware into software-accessible data streams, such as ROS2 topics.

* **capability:** Standard applications callable by the intelligence layer (e.g., via MCP), such as: `audio_speak`, `mic_listen`, `nav2_walk`, `yolo_look`, `grasper`, or even `internet`. Each capability should include:

  > * `api`: Interface encapsulation. Should provide APIs for: `init/start/soft-configure/act/sense/emergency/standby/shutdown`.
  >   “Soft configuration” refers to global/local configurations modifiable via API.
  >
  > * `description`: Framework configuration, e.g., whether to initialize on power-on, and other parameters.
  >
  > * `src`: Directory for implementation logic and source code.
  >
  > * `README.md`: Developer manual.

* **skill:** Skill packages that complete complex actions (e.g., making a bed) by combining capabilities. Can also be implemented via agent logic.

* **memory:** Stores temporary or long-term data, such as just-in-time prompts, etc.

* **brain:** The intelligence core. It includes the world model, the LLM/VLM runtime, and a set of memory + learn interfaces for updating/accessing memory, as well as model management interfaces.

* **cost:** Represents intrinsic value evaluation—controls the reasoning preference and behavioral principles of the brain, such as worldview prompts.

* **config:** Global hardware configuration and basic functionalities such as OTA (over-the-air) updates.

* **manager:** The bootstrapper, runs at system power-on. It scans the `capability` directory and initializes the necessary modules. Meant to be stable and rarely updated—like a BIOS.

* **infra:** Middleware communication layer for DDS/rclrpc, providing multiple inter-node communication mechanisms such as topics, services, actions, and streams.

---

## Setup

```bash
git clone https://github.com/syswonder/deepembody.git
cd deepembody
```

## Startup

```bash
python3 ./manager/boot.py
```

## Python

for userspace programming and simulation, you need to install the following packages:

```bash
pip install rich loguru mcp grpcio grpcio-tools pyyaml
```

---
