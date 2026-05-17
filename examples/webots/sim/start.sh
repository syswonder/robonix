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

# X11 cookie file for Docker bind-mount (SSH/Moba MoTTY forwarding uses
# localhost:N and requires MIT-MAGIC-COOKIE in the container). compose.yaml
# mounts this path to /root/.Xauthority.
export ROBONIX_HOST_XAUTH="${ROBONIX_HOST_XAUTH:-${XAUTHORITY:-$HOME/.Xauthority}}"
if [[ -e "$ROBONIX_HOST_XAUTH" && ! -f "$ROBONIX_HOST_XAUTH" ]]; then
    echo "[sim/start] error: X11 auth path exists but is not a regular file: $ROBONIX_HOST_XAUTH" >&2
    echo "[sim/start] Refusing to continue because Docker would bind-mount an invalid path at /root/.Xauthority." >&2
    exit 1
fi

if [[ ! -f "$ROBONIX_HOST_XAUTH" ]]; then
    echo "[sim/start] warning: X11 auth file missing: $ROBONIX_HOST_XAUTH"
    echo "[sim/start] Creating an empty X11 auth file so Docker bind-mounts a file at /root/.Xauthority."
    mkdir -p "$(dirname "$ROBONIX_HOST_XAUTH")"
    if ! touch "$ROBONIX_HOST_XAUTH"; then
        echo "[sim/start] error: failed to create X11 auth file: $ROBONIX_HOST_XAUTH" >&2
        echo "[sim/start] For SSH forwarding use: ssh -Y user@host (or trusted -X). For local :0, log in to a desktop session once so ~/.Xauthority exists." >&2
        exit 1
    fi
fi

CF=(-f compose.yaml)
if [[ "${ROBONIX_FORCE_CPU:-0}" != "1" ]] && command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
  CF+=(-f compose.gpu.yaml)
  echo "[sim/start] NVIDIA GPU detected — merging compose.gpu.yaml"
else
  echo "[sim/start] no GPU (or ROBONIX_FORCE_CPU=1) — CPU-only Webots"
fi

# Optional browser-streaming mode (ROBONIX_SIM_STREAM=1). Useful on
# headless servers or xrdp boxes where the host's X session is software-
# rendered (llvmpipe → 0.01x simulation). See compose.stream.yaml and
# bridge/entrypoint.sh.
if [[ "${ROBONIX_SIM_STREAM:-0}" = "1" ]]; then
  CF+=(-f compose.stream.yaml)
  echo "[sim/start] stream mode — merging compose.stream.yaml (WS :1234, viewer :8080)"
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

if [[ "${ROBONIX_SIM_STREAM:-0}" = "1" ]]; then
  echo
  echo "[sim/start] ========== webots stream ready =========="
  echo "[sim/start] open ONE of these in a browser:"
  # tailscale first (works from anywhere on the tailnet)
  if command -v tailscale &>/dev/null; then
    tailscale ip -4 2>/dev/null | sed 's|^|  http://|;s|$|:8080/|'
  fi
  # then LAN / global v4 addresses
  ip -4 -o addr show scope global 2>/dev/null \
    | awk '{print $4}' | cut -d/ -f1 \
    | sed 's|^|  http://|;s|$|:8080/|'
  echo "[sim/start] the viewer auto-fills ws://<that-host>:1234 — just hit Connect"
  echo
fi

# Auto-launch rviz2 inside the sim container (it has ros-humble-rviz2,
# host doesn't have to). Same DDS bus as the rest of the stack so
# /map, /scanner, /tf, /goal_pose all work. User can click "2D Nav
# Goal" → simple_nav drives the robot.
#
# Works in stream mode too — start_rviz.sh forwards the *host* DISPLAY
# into `docker exec`, independent of the container's internal Xorg :48.
# So an xrdp / NoMachine user still gets rviz inside their session;
# webots' 3D view streams to the browser via :8080 separately.
if command -v xhost &>/dev/null; then
    xhost +local:docker >/dev/null 2>&1 || true
fi
echo "[sim/start] launching rviz2 (config: rviz2_default.rviz)"
# Per-user log path: /tmp is shared on multi-tenant boxes and a fixed
# /tmp/rviz2.log file owned by another user blocks rewrite. ${USER:-rviz}
# falls back when USER is unset (e.g. cron/system contexts).
RVIZ_LOG="${TMPDIR:-/tmp}/rviz2-${USER:-rviz}.log"
bash "$SCRIPT_DIR/start_rviz.sh" >"$RVIZ_LOG" 2>&1 &
echo "[sim/start] rviz2 log: $RVIZ_LOG"

# Stay foreground tailing logs so Ctrl-C is the natural stop pattern.
exec docker compose "${CF[@]}" logs -f
