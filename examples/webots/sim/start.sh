#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Bring up the Tiago Webots sim container. Run this BEFORE `rbnx boot`
# from examples/webots/ — robonix drivers are docker-exec'd into the
# container started here, so the container has to exist first.
#
# Auto-detects nvidia-smi to merge compose.gpu.yaml. To force CPU-only,
# unset CUDA_VISIBLE_DEVICES or set ROBONIX_FORCE_CPU=1.
#
# Re-running is safe: docker compose up reuses the running container.
# Stop with Ctrl-C, or from another terminal: `docker compose -f compose.yaml down`.
set -euo pipefail

# Resolve the script's own directory ONCE in absolute form. We cd into
# it below; afterwards `$(dirname "$0")` no longer points anywhere
# usable (it's relative to the *original* CWD, which is gone). The
# previous code used `$(dirname "$0")` later in the file to find
# start_rviz.sh, which silently failed on every invocation that
# wasn't run from the sim/ directory itself — rviz never launched
# even though the script printed "[sim/start] launching rviz2 ..."
# because the bash sub-process couldn't find start_rviz.sh.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# Auto-detect DISPLAY if the launching shell didn't export one. Probes
# the standard local X server slots via `xset q`; if any responds, use
# it. Falls back to :0 so headless / non-X bash still gets a sensible
# default the docker compose env interpolation can use. Without this,
# users repeatedly forgot to `export DISPLAY=:0` and Webots / rviz2
# came up invisible.
if [[ -z "${DISPLAY:-}" ]]; then
    if command -v xset &>/dev/null; then
        for d in :0 :1 :10; do
            if DISPLAY="$d" xset q &>/dev/null; then
                export DISPLAY="$d"
                echo "[sim/start] auto-detected DISPLAY=$DISPLAY"
                break
            fi
        done
    fi
    : "${DISPLAY:=:0}"; export DISPLAY
fi

CF=(-f compose.yaml)
if [[ "${ROBONIX_FORCE_CPU:-0}" != "1" ]] && command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
  CF+=(-f compose.gpu.yaml)
  echo "[sim/start] NVIDIA GPU detected — merging compose.gpu.yaml"
else
  echo "[sim/start] no GPU (or ROBONIX_FORCE_CPU=1) — CPU-only Webots"
fi

# X11 GUI: ensure docker can reach the local DISPLAY. xhost is harmless
# on systems without an X server (it just fails silently).
if command -v xhost &>/dev/null; then
  xhost +local:docker >/dev/null 2>&1 || true
fi

# Bring sim up detached so we can layer rviz on top before tailing logs.
docker compose "${CF[@]}" up --build -d

# Wait until ros2 inside the container has more than a handful of
# topics — proxy for "webots controller has spawned the robot and is
# publishing /scanner /odom /head_front_camera/* etc."
echo "[sim/start] waiting for sim ros topics..."
for _ in $(seq 1 60); do
    n=$(docker exec robonix_tiago_sim bash -c \
        'source /opt/ros/humble/setup.bash 2>/dev/null && ros2 topic list 2>/dev/null | wc -l' 2>/dev/null || echo 0)
    if [[ "${n:-0}" -gt 20 ]]; then
        echo "[sim/start] ros up ($n topics)"
        break
    fi
    sleep 2
done

# Auto-launch rviz2 inside the sim container (it has ros-humble-rviz2,
# host doesn't have to). Same DDS bus as the rest of the stack so
# /map, /scanner, /tf, /goal_pose all work. User can click "2D Nav
# Goal" → simple_nav drives the robot.
if command -v xhost &>/dev/null; then
    xhost +local:docker >/dev/null 2>&1 || true
fi
echo "[sim/start] launching rviz2 (config: rviz2_default.rviz)"
bash "$SCRIPT_DIR/start_rviz.sh" >/tmp/rviz2.log 2>&1 &

# Stay foreground tailing logs so Ctrl-C is the natural stop pattern.
exec docker compose "${CF[@]}" logs -f
