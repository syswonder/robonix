# `tiago_sim_stack` (RFC002 + Docker Compose)

Single Compose service **`ros2-bridge`**: **Webots (GUI)** + `eaios_webots` launch + **Nav2** bringup + **`tiago_bridge`** registering MCP with `robonix-server`.

## Requirements

- Docker, host **X11** (this stack is **not** headless).
- Before `docker compose up`, allow local clients to use your display, e.g. `xhost +local:docker` (revert when done).
- Set **`DISPLAY`** (e.g. `export DISPLAY=:0`) in the shell that runs Compose so the container inherits it.
- **`robonix-server`** listening on the address in **`ROBONIX_SERVER`** (default `127.0.0.1:50051`). With `network_mode: host`, the container shares the host network namespace.

## `rbnx` (from repository `rust/`)

```bash
cargo run -p robonix-cli -- validate examples/packages/tiago_sim_stack
cargo run -p robonix-cli -- build -p examples/packages/tiago_sim_stack
# with robonix-server already running:
cargo run -p robonix-cli -- start -p examples/packages/tiago_sim_stack -n com.robonix.prm.tiago
```

`start` runs `docker compose up --build ros2-bridge` in the foreground (Ctrl+C stops the stack).

## End-to-end script

From `rust/`:

```bash
./examples/run.sh
```

This uses **`rbnx` validate/build/start** for `vlm_service` and `tiago_sim_stack` by default (and starts `robonix-server` + `robonix-agent` unless disabled — see `run.sh` header). Use **`START_SIM_STACK=0`** to skip the sim container.

## Layout

| Path | Role |
|------|------|
| `ros_ws/src/eaios_webots` | ROS 2 Python package: Webots world + `robot_launch.py` (from deprecated `tiago_demo_package`). |
| `nav2_bringup/config` | Nav2 + AMCL YAML and map for `nav2_bringup`. |
| `bridge/Dockerfile` | Humble + Webots `.deb` + colcon build of `eaios_webots` + `tiago_bridge`. |
| `bridge/entrypoint.sh` | Start Webots stack → Nav2 → `tiago_bridge.node`. |

## Topic notes

`tiago_bridge` defaults (camera, scan, `/amcl_pose`) may not match this Webots world without extra sensors or remaps. Override with **`TIAGO_*_TOPIC`** env vars in `compose.yaml` if needed.
