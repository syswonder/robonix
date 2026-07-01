<!-- SPDX-License-Identifier: MulanPSL-2.0 -->
# Webots deploy — troubleshooting

Issues hit when building / booting this deploy on a fresh machine (esp. CN
networks and shared / multi-tenant GPU servers), with the fix for each. Symptom
lines are the literal error you'll see.

## Build environment

### `Python module 'grpc_tools' not importable from /usr/bin/python3`
`rbnx codegen` (run during every package build) needs **grpcio-tools** in the
system `python3` to emit `_pb2.py` / `_pb2_grpc.py`. A fresh box doesn't have
it, so every package fails at the codegen step.

```bash
python3 -m pip install --user grpcio-tools
# CN: add  -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### `Error: resolved path does not exist: …/system/atlas/proto (robonix_source_path=…)`
`rbnx` reads the robonix source root from `~/.robonix/config.yaml`
(`robonix_source_path`), **not** from the `ROBONIX_SOURCE_PATH` env var. If it
points at a stale / other checkout, codegen can't find the IDL and every package
fails. Point it at your checkout:

```bash
cd /path/to/your/robonix && rbnx setup .
# verify:  grep robonix_source_path ~/.robonix/config.yaml
```
On a **shared box** where you must not clobber another user's config, back it up
first and restore after (`cp ~/.robonix/config.yaml /tmp/cfg.bak; … ; cp back`).

### `[robonix-codegen] unsupported --lang 'ros2'. Supported: 'proto', 'mcp'.`
A **stale `robonix-codegen`** is installed in `~/.cargo/bin/` (from an older
checkout). `rbnx codegen` prefers `$ROBONIX_CODEGEN_BIN` → `~/.cargo/bin/robonix-codegen`
→ workspace `target/`, so the old binary wins over your fresh build and rejects
newer `--lang` values the package build.sh uses (e.g. `--ros2`). Fix either:

```bash
# point at your fresh build (non-invasive — doesn't touch ~/.cargo/bin):
export ROBONIX_CODEGEN_BIN=/path/to/your/robonix/target/debug/robonix-codegen
# OR reinstall the fresh one over the stale:
cd /path/to/your/robonix && make install
```

### `uv: command not found`
Python package builds use uv. Install (user-space):
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh   # adds ~/.local/bin
```

### `git clone https://github.com/… : Failed to connect to github.com port 443: Connection timed out`
`rbnx build` clones the remote providers (`mapping_rbnx`, `nav2_wrapper_rbnx`,
`explore_rbnx`) from GitHub. Some runners can time out on direct GitHub clone
traffic even when release downloads work. Route git through the configured
mirror:

```bash
git config --global url."https://ghfast.top/https://github.com/".insteadOf "https://github.com/"
```

### Model weights won't download (HuggingFace unreachable)
`voiceprint` (ECAPA-TDNN) and `speech` (FunASR) fetch model weights at build
time. HuggingFace — and even `hf-mirror.com` via `huggingface_hub` — fail on CN
networks (hf_hub only follows same-host redirects; hf-mirror returns a
cross-host 308). Use **ModelScope** instead (voiceprint already does; speech's
FunASR pulls from ModelScope by default). If a package still hits HF, the fix is
to fetch from ModelScope's SDK (`modelscope.snapshot_download`), not to set
`HF_ENDPOINT`.

### CUDA disabled in scene/voiceprint despite a working GPU
An **empty** `CUDA_VISIBLE_DEVICES=""` tells CUDA "no GPUs" while `nvidia-smi`
still works, so `torch.cuda.is_available()` is False. Only forward
`CUDA_VISIBLE_DEVICES` when it's non-empty; with docker GPU passthrough also set
`NVIDIA_DRIVER_CAPABILITIES=all` (or include `compute`) — `--gpus all` alone
injects only `utility` and CUDA libs aren't mounted.

## Running a 2nd sim on a shared host (CI alongside an interactive user)

The Webots sim assumes one instance per host. To run an isolated sim (e.g. CI)
beside someone else's, override these — all default to the single-tenant value,
so a normal `bash sim/start.sh` is unchanged:

| Collision | Symptom | Env override |
|-----------|---------|--------------|
| container name | `docker exec robonix_tiago_sim …` kills the other run's drivers | `ROBONIX_SIM_CONTAINER=mine` (+ `ROBONIX_SIM_PROJECT=mine`) |
| Xorg display `:48` | second nvidia Xorg can't claim `:48` | `ROBONIX_SIM_XDISPLAY=:49` |
| GPU | two webots on one GPU stall / contend | `ROBONIX_GPU_ID=2 NVIDIA_VISIBLE_DEVICES=2` (use a free GPU) |
| webots stream port `:1234` | `webots: invalid value "port=…" to '--stream'` (can't offset in CLI) AND host-net bind clash | **run on a bridge network**: `ROBONIX_SIM_NETWORK=bridge` (own netns → own 1234; default `host` unchanged) |
| FastRTPS IPC | webots extern-controller IPC keyed by port 1234 collides under `ipc:host` | `ROBONIX_SIM_IPC=private` |
| ROS2 / DDS bus | topics cross-talk between runs | `ROS_DOMAIN_ID=91` (any value ≠ the other run's) |

> The webots 3D/sensor render only works on the **stream** path
> (`WEBOTS_STREAM=1` + an NVIDIA Xorg). The non-stream GUI path renders a black
> viewport headless, so cameras/lidar never initialise and the robot's extern
> controller never connects ("world is not yet ready"). Don't try to run sensors
> headless without stream mode.

A fully-isolated CI sim launch therefore looks like:
```bash
ROBONIX_SIM_NETWORK=bridge ROBONIX_SIM_CONTAINER=ci-sim ROBONIX_SIM_PROJECT=ci-sim \
ROBONIX_SIM_IPC=private ROBONIX_GPU_ID=2 NVIDIA_VISIBLE_DEVICES=2 \
ROBONIX_SIM_XDISPLAY=:49 ROS_DOMAIN_ID=91 WEBOTS_HEADLESS_MODE=nvidia WEBOTS_STREAM=1 \
docker compose -f compose.yaml -f compose.gpu.yaml -f compose.stream.yaml up --build -d
```
Robot topics (`/scanner`, `/odom`, `/head_front_camera/*`) appear within ~15 s.
