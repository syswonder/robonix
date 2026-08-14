# Tiago Webots sim container

The simulation **environment** for the webots example. Brings up
**Webots (GUI) + the `eaios_webots` Tiago controller** in one container.
That's all — Nav2 lives in the [`tiago_nav2`](../services/tiago_nav2/)
service package, and per-device drivers live in
[`../primitives/`](../primitives/). Robonix `docker exec`s those into
this container at deploy time.

## Run

Start the sim **first**, then `rbnx boot` from `examples/webots/`:

```bash
# Terminal 1 — sim (GUI; Ctrl-C to stop):
bash examples/webots/sim/start.sh

# Terminal 2 — robonix:
bash examples/webots/boot.sh
```

## Robot selection

TIAGo Lite remains the default:

```bash
bash examples/webots/sim/start.sh --tiago-variant lite
```

Select the complete single-arm Webots TIAGo with its default parallel gripper:

```bash
bash examples/webots/sim/start.sh --tiago-variant full
```

The launcher enables the official Webots R2025a asset download automatically
for `full`. The first download is about 661 MB; the persistent cache makes
subsequent starts skip it. Set `ROBONIX_WEBOTS_DOWNLOAD_ALL_ASSETS=0` only to
opt out explicitly. In the Robonix terminal use the same selection so Soma and
Vitals expose the matching body:

```bash
bash examples/webots/boot.sh --tiago-variant full
```

You can also export `ROBONIX_TIAGO_VARIANT=lite|full`. An explicit `--robot`
still overrides the variant's adapter URDF for advanced testing.

World selection:

```bash
bash examples/webots/sim/start.sh --world office.wbt
bash examples/webots/sim/start.sh --world apartment.wbt
bash examples/webots/sim/start.sh --world complete_apartment.wbt
bash examples/webots/sim/start.sh --world break_room.wbt
bash examples/webots/sim/start.sh --world kitchen.wbt
```

Or:

```bash
export ROBONIX_WEBOTS_WORLD=kitchen.wbt
bash examples/webots/sim/start.sh
```

`office.wbt` is the default. On its first start, the simulator downloads the
small, versioned office seed from the
[Robonix Assets](https://github.com/syswonder/robonix-assets/releases/tag/webots-office-seed-v3)
Release through `https://ghfast.top/`, verifies its SHA-256 checksum, and stores
it in the persistent `webots_cache` Docker volume. Later starts reuse that
cache. Set `ROBONIX_WEBOTS_SEED_MIRROR` to another prefix, or to an empty value
for direct GitHub access; `ROBONIX_WEBOTS_SEED_URL` overrides the complete
source URL.

For the other built-in worlds, download Cyberbotics' official offline asset
bundle once before launching:

```bash
ROBONIX_WEBOTS_DOWNLOAD_ALL_ASSETS=1 \
  bash examples/webots/sim/start.sh --world apartment.wbt
```

This downloads `assets-R2025a.zip` from the Webots GitHub release through
`https://ghfast.top/` by default and extracts it into the persistent
`webots_cache` Docker volume. Later starts reuse the cache and skip the
download. Override `ROBONIX_WEBOTS_ASSETS_MIRROR` or `ROBONIX_WEBOTS_ASSETS_URL`
only if your network needs a different mirror/source.

|  |  |
|---|---|
| `office.wbt`<br>![office](thumbnails/office.jpg) | `apartment.wbt`<br>![apartment](thumbnails/apartment.jpg) |
| `complete_apartment.wbt`<br>![complete apartment](thumbnails/complete_apartment.jpg) | `break_room.wbt`<br>![break room](thumbnails/break_room.jpg) |
| `kitchen.wbt`<br>![kitchen](thumbnails/kitchen.jpg) |  |

`start.sh` auto-detects `nvidia-smi` and merges `compose.gpu.yaml` when
present. Force CPU-only with `ROBONIX_FORCE_CPU=1`. The container's name
is `robonix_tiago_sim` (referenced by every driver package's
`docker exec`).

## Requirements

- Docker + Docker Compose v2.
- Host X11 — `DISPLAY` set in the launching shell, plus
  `xhost +local:docker` once per session (`start.sh` does this for you
  when xhost is available). **SSH / MobaXterm (MoTTY) forwarding**
  (`DISPLAY=localhost:10.0` etc.) also needs a valid X11 cookie: use
  `ssh -Y` (trusted forwarding) so the server creates/updates
  `~/.Xauthority`, or set `XAUTHORITY` to your host cookie path before
  `start.sh`. The sim compose file bind-mounts that file into the
  container as `/root/.Xauthority`.
- For NVIDIA GPU: `nvidia-container-toolkit` installed on the host. An
  NVIDIA-backed headless Xorg server additionally requires `nvidia_drv.so` and
  the driver-version-matched `libglxserver_nvidia.so`; `start.sh` discovers
  them only for `WEBOTS_HEADLESS_MODE=nvidia` or GPU-backed `auto`. For a
  non-standard driver layout, set
  `ROBONIX_NVIDIA_XORG_DRIVER` and `ROBONIX_NVIDIA_GLX_SERVER` to the two
  regular files before starting the simulator.

## Layout

| Path | Role |
|------|------|
| `start.sh` | User-facing launcher. `bash start.sh`. |
| `compose.yaml` | Single `sim` service: Webots + eaios_webots + bind-mounts of `../primitives` and `../services` into the container at `/robonix_pkgs`. |
| `compose.gpu.yaml` | Optional NVIDIA GPU passthrough (auto-merged by `start.sh`). |
| `compose.nvidia-xorg.yaml` | NVIDIA Xorg server modules, merged only for the `nvidia` or GPU-backed `auto` headless backend. |
| `compose.stream.yaml` | Optional browser-streaming mode — headless Xorg, Webots `--stream`, and the bandwidth-optimized browser endpoint. Merged when `ROBONIX_SIM_STREAM=1`. |
| `bridge/Dockerfile` | Humble + Webots `.deb` + Python deps used by docker-exec'd robonix drivers. |
| `bridge/entrypoint.sh` | Launch Webots and its browser-stream helpers, then `wait` so the container stays alive. Picks display backend per `WEBOTS_HEADLESS_MODE`. |
| `bridge/viewer_server.py` | Serve WebotsView locally and proxy/cache remote viewer and world assets. |
| `bridge/webots_stream_proxy.py` | Forward the live W3D stream while dropping unused robot-window camera payloads. |
| `bridge/streaming_healthcheck.py` | Verify that both browser-stream helpers are alive and reachable. |
| `bridge/update-webots-seed.sh` | Maintainer tool that exports an updated office cache for publication in `syswonder/robonix-assets`. |
| `ros_ws/src/eaios_webots` | ROS 2 launch, Lite/full adapter URDFs, deterministic world conversion, and Webots worlds. |

## Headless / browser-streaming mode

Some deployments don't have a usable host X server: a headless
server, a multi-user box where everyone reaches it through xrdp or
NoMachine (both give you a `Mesa llvmpipe` software-rendered X session
that drops Webots to ~0.01x real-time), or a shared GPU node whose
physical display is on the BMC instead of an NVIDIA card.

The fix is to (a) start an NVIDIA-backed Xorg **inside** the container
— isolated from any host user's display — and (b) use Webots' built-in
WebSocket streaming so the 3D view shows up in a remote browser.

```bash
ROBONIX_SIM_STREAM=1 bash examples/webots/sim/start.sh
```

`start.sh` then merges `compose.stream.yaml` and prints the access
URLs (tailscale + LAN). rviz2 still launches as usual — it forwards
the **host** `$DISPLAY`, so users running the script from inside an
xrdp / NoMachine session keep seeing rviz in their session; only the
GPU-heavy webots 3D view goes to the browser stream.

Open `http://<server>:8080/` in a browser and hit Connect — the WS URL
is pre-filled with the page's hostname so a third machine doesn't end
up dialling its own `localhost`. The viewer uses the optimized W3D endpoint on
port `1235`: it keeps the interactive WebGL scene and live robot transforms,
but removes high-rate robot-window camera messages that the standard viewer
does not consume. Viewer JavaScript, textures, meshes, and world assets are
proxied and cached by the server, avoiding repeated cross-network downloads.

For a remote machine, forward both endpoints over SSH:

```bash
ssh -N \
  -L 18080:127.0.0.1:8080 \
  -L 11235:127.0.0.1:1235 \
  user@server
```

Then open `http://127.0.0.1:18080/?wsPort=11235`. The first load populates the
viewer cache; later loads reuse it.

Override the ports for parallel deployments with
`ROBONIX_SIM_VIEWER_PORT` and `ROBONIX_SIM_STREAM_PORT`. The latter is the
optimized public endpoint; browsers should not connect to Webots' raw port
`1234` directly. These variables configure the actual helper listeners, so
they work with the default host network as well as bridge port publishing.
The entrypoint supervises both helpers and exits if either one fails; `start.sh`
only prints the viewer URL after both endpoints pass their readiness checks.

Run the lightweight custom-port and helper-readiness smoke test with:

```bash
python3 -m pip install "websockets>=14,<17"
python3 examples/webots/sim/tests/test_streaming.py
```

Backend selection (env on the sim container):

| `WEBOTS_HEADLESS_MODE` | Effect |
|---|---|
| `host` (default w/o stream) | inherit `$DISPLAY` from compose — legacy local-X path |
| `auto` (default in stream) | NVIDIA Xorg `:48` on the GPU with most free memory; falls back to Xvfb if `/dev/nvidia0` is absent |
| `nvidia` | force NVIDIA Xorg `:48` (fails fast if no GPU) |
| `xvfb` | software llvmpipe on `:99` — slow but needs no GPU |

Confirm that stream mode is actually using NVIDIA rather than the Xvfb
fallback:

```bash
docker exec robonix_tiago_sim bash -lc \
  'DISPLAY=${ROBONIX_SIM_XDISPLAY:-:48} glxinfo -B | grep -E "OpenGL vendor|OpenGL renderer"'
nvidia-smi
```

The renderer must contain `NVIDIA`, and `nvidia-smi` should list Xorg and
Webots. If Compose reports that an Xorg module source is not a regular file,
remove any stale directory at that source path and run `start.sh` again; the
launcher now refuses to create such directories.

`:48` sits well outside the host's normal X allocator range
(`:0..:12` for physical sessions, `:1001..:1099` for xrdp), so the X
socket that leaks into the bind-mounted `/tmp/.X11-unix` will not
collide with any host user's session; the socket file appearing there
also makes the host's X allocator skip the number for any future
session it spins up.

## Why is the container kept alive after Webots launches?

Robonix drivers (e.g. `tiago_chassis`, `tiago_camera`, `tiago_lidar`,
`tiago_nav2`) run in **this** container via `docker exec` so they share
the same DDS graph as Webots. The entrypoint ends with `wait` (instead
of the previous `exec python3 -m tiago_bridge.node`) so Compose treats
the container as live for as long as the Webots launch is alive.
