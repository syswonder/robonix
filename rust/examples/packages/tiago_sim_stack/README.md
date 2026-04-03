# `tiago_sim_stack` (RFC002 + Docker Compose)

Single Compose service **`ros2-bridge`**: **Webots (GUI)** + `eaios_webots` launch + **Nav2** bringup + **`tiago_bridge`** registering MCP with `robonix-atlas`.

## Requirements

- Docker, host **X11** (this stack is **not** headless).
- Before `docker compose up`, allow local clients to use your display, e.g. `xhost +local:docker` (revert when done).
- Set **`DISPLAY`** (e.g. `export DISPLAY=:0`) in the shell that runs Compose so the container inherits it.
- **`robonix-atlas`** listening on the address in **`ROBONIX_ATLAS`** (default `127.0.0.1:50051`). With `network_mode: host`, the container shares the host network namespace.

## `rbnx` (from repository `rust/`)

```bash
cargo run -p robonix-cli -- validate examples/packages/tiago_sim_stack
cargo run -p robonix-cli -- build -p examples/packages/tiago_sim_stack
# with robonix-atlas already running:
cargo run -p robonix-cli -- start -p examples/packages/tiago_sim_stack -n com.robonix.prm.tiago
```

`start` runs `docker compose up --build ros2-bridge` in the foreground (Ctrl+C stops the stack).

## End-to-end script

From `rust/`:

```bash
./examples/run.sh
```

This uses **`rbnx` validate/build/start** for `vlm_service` and `tiago_sim_stack` by default (and starts `robonix-atlas` + `robonix-pilot` unless disabled — see `run.sh` header). Use **`START_SIM_STACK=0`** to skip the sim container.

## Agent skills (`skills/`)

`rbnx start` registers **`skills/* /SKILL.md`** with `robonix-atlas` for `robonix-pilot`. Notable:

- **`object_search_wander`** — find a target (e.g. door) with **`robot_state` + `camera_snapshot` + short `base_cmd`**; explicitly **no** `base_navigate` / nav goals.
- **`navigation`** — Nav2 map goals when allowed.
- **`visual_inspection`** — camera-first perception.

## Layout

| Path | Role |
|------|------|
| `tiago_bridge/` | Python module: ROS2 ↔ MCP bridge (camera, pose, scan, navigation tools). |
| `ros_ws/src/eaios_webots` | ROS 2 Python package: Webots world + `robot_launch.py`. |
| `nav2_bringup/config` | Nav2 + AMCL YAML, map, and rviz config. |
| `bridge/Dockerfile` | Humble + Webots `.deb` + colcon build of `eaios_webots` + `tiago_bridge`. |
| `bridge/entrypoint.sh` | Start Webots stack → Nav2 → rviz2 → `tiago_bridge.node`. |

## Webots GUI feels slow?

Compared to the older **`docker/run.sh`** dev image, this Compose file used to start Webots with **only** the X11 socket mounted. Two things hurt a lot:

1. **Default `/dev/shm` (64MB)** — Qt and Webots use shared memory; it is easy to end up swapping or stalling. `compose.yaml` now sets **`shm_size: '2gb'`** (same idea as giving GL apps enough IPC).
2. **No GPU in the container** — `docker/run.sh` adds **`--gpus all`** when `nvidia-smi` exists and mounts **`/dev/dri`**. Without that, Webots often falls back to **software OpenGL** over X11, which feels “stuck” or single-digit FPS.

**What to do**

- **NVIDIA:** install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html), then run:
  ```bash
  docker compose -f compose.yaml -f compose.gpu.yaml up --build
  ```
  (or the same merge from repo root with `-f` paths adjusted).
- **Intel / AMD:** keep the default `compose.yaml`; **`/dev/dri`** is already mounted for Mesa.
- Still bad? Try **`ipc: host`** on the service (stronger than large shm; dev-only).

## Topic notes

`tiago_bridge` defaults (camera, scan, `/amcl_pose`) may not match this Webots world without extra sensors or remaps. Override with **`TIAGO_*_TOPIC`** env vars in `compose.yaml` if needed.
