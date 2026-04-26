# Tiago Webots sim container

The simulation **environment** for the webots example. Brings up
**Webots (GUI) + the `eaios_webots` Tiago controller** in one container.
That's all — Nav2 lives in the [`tiago_nav2`](../services/tiago_nav2/)
service package, and per-device drivers live in
[`../primitives/`](../primitives/). Robonix `docker exec`s those into
this container at deploy time.

## Run

Start the sim **first**, then `rbnx deploy` from `examples/webots/`:

```bash
# Terminal 1 — sim (GUI; Ctrl-C to stop):
bash examples/webots/sim/start.sh

# Terminal 2 — robonix:
cd examples/webots
rbnx deploy
```

`start.sh` auto-detects `nvidia-smi` and merges `compose.gpu.yaml` when
present. Force CPU-only with `ROBONIX_FORCE_CPU=1`. The container's name
is `robonix_tiago_sim` (referenced by every driver package's
`docker exec`).

## Requirements

- Docker + Docker Compose v2.
- Host X11 — `DISPLAY` set in the launching shell, plus
  `xhost +local:docker` once per session (`start.sh` does this for you
  when xhost is available).
- For NVIDIA GPU: `nvidia-container-toolkit` installed on the host.

## Layout

| Path | Role |
|------|------|
| `start.sh` | User-facing launcher. `bash start.sh`. |
| `compose.yaml` | Single `sim` service: Webots + eaios_webots + bind-mounts of `../primitives` and `../services` into the container at `/robonix_pkgs`. |
| `compose.gpu.yaml` | Optional NVIDIA GPU passthrough (auto-merged by `start.sh`). |
| `bridge/Dockerfile` | Humble + Webots `.deb` + Python deps used by docker-exec'd robonix drivers. |
| `bridge/entrypoint.sh` | Launch Webots, then `wait` so the container stays alive. |
| `bridge/webots_assets_seed.tar.gz` | Pre-baked Webots proto/texture cache (offline-fast first run). |
| `ros_ws/src/eaios_webots` | ROS 2 launch + Webots world for the simulated Tiago. |

## Why is the container kept alive after Webots launches?

Robonix drivers (e.g. `tiago_chassis`, `tiago_camera`, `tiago_lidar`,
`tiago_nav2`) run in **this** container via `docker exec` so they share
the same DDS graph as Webots. The entrypoint ends with `wait` (instead
of the previous `exec python3 -m tiago_bridge.node`) so Compose treats
the container as live for as long as the Webots launch is alive.
