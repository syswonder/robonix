#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Bring up the Tiago Webots sim container. Run this BEFORE `rbnx boot`
# from examples/webots/ — robonix drivers are docker-exec'd into the
# container started here, so the container has to exist first.
#
# Auto-detects nvidia-smi to merge compose.gpu.yaml. To force CPU-only,
# set ROBONIX_FORCE_CPU=1.
#
# Re-running is safe: docker compose up reuses the running container.
# Stop with Ctrl-C, or from another terminal: `docker compose -f compose.yaml down`.
set -euo pipefail

# Sim container / compose-project names — overridable via ROBONIX_SIM_CONTAINER
# / ROBONIX_SIM_PROJECT so a CI / parallel deploy brings up its OWN isolated
# Webots container instead of the shared default. Exported so the compose files
# (`name:`, `container_name:`) interpolate the same values. Stream ports are
# similarly offset via ROBONIX_SIM_STREAM_PORT / ROBONIX_SIM_VIEWER_PORT, and
# GPU via ROBONIX_GPU_ID. Defaults preserve existing single-deploy behaviour.
export ROBONIX_SIM_CONTAINER="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"
export ROBONIX_SIM_PROJECT="${ROBONIX_SIM_PROJECT:-robonix_tiago_sim}"
SIM_CT="$ROBONIX_SIM_CONTAINER"

# Resolve the script's own directory ONCE in absolute form. We cd into
# it below; afterwards `$(dirname "$0")` no longer points anywhere
# usable (it's relative to the *original* CWD, which is gone). The
# previous code used `$(dirname "$0")` later in the file to find
# start_rviz.sh, which silently failed on every invocation that
# wasn't run from the sim/ directory itself — rviz never launched
# even though the script printed "[sim/start] launching rviz2 ..."
# because the bash sub-process couldn't find start_rviz.sh.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
# Avoid Docker Hub metadata checks on every compose rebuild. The helper creates
# a local alias and pulls through configured mirrors on first use.
# Override ROBONIX_SIM_ROS_BASE_IMAGE for a custom/mirror/digest base.
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/docker_base_image.sh"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/nvidia_host.sh"
export ROBONIX_SIM_ROS_BASE_IMAGE="${ROBONIX_SIM_ROS_BASE_IMAGE:-robonix-osrf-ros:humble-desktop-full}"
robonix_ensure_local_base_image "$ROBONIX_SIM_ROS_BASE_IMAGE" "osrf/ros:humble-desktop-full"
cd "$SCRIPT_DIR"

# Webots world / robot launch args.
# Usage:
#   ./start.sh --world your_new_world.wbt
#   ./start.sh --tiago-variant full
#   ROBONIX_WEBOTS_WORLD=your_new_world.wbt ./start.sh
export ROBONIX_WEBOTS_WORLD="${ROBONIX_WEBOTS_WORLD:-office.wbt}"
export ROBONIX_TIAGO_VARIANT="${ROBONIX_TIAGO_VARIANT:-lite}"
export ROBONIX_WEBOTS_ROBOT="${ROBONIX_WEBOTS_ROBOT:-}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --world|-w)
      [[ $# -ge 2 ]] || { echo "[sim/start] --world requires a value" >&2; exit 2; }
      export ROBONIX_WEBOTS_WORLD="$2"
      shift 2
      ;;
    --robot|-r)
      [[ $# -ge 2 ]] || { echo "[sim/start] --robot requires a value" >&2; exit 2; }
      export ROBONIX_WEBOTS_ROBOT="$2"
      shift 2
      ;;
    --tiago-variant)
      [[ $# -ge 2 ]] || { echo "[sim/start] --tiago-variant requires a value" >&2; exit 2; }
      export ROBONIX_TIAGO_VARIANT="$2"
      shift 2
      ;;
    --help|-h)
      echo "Usage: $0 [--world WORLD.wbt] [--tiago-variant lite|full] [--robot ROBOT.urdf]"
      exit 0
      ;;
    *)
      echo "[sim/start] unknown argument: $1" >&2
      echo "Usage: $0 [--world WORLD.wbt] [--tiago-variant lite|full] [--robot ROBOT.urdf]" >&2
      exit 1
      ;;
  esac
done

case "$ROBONIX_TIAGO_VARIANT" in
  lite)
    export ROBONIX_WEBOTS_ROBOT="${ROBONIX_WEBOTS_ROBOT:-tiago_webots.urdf}"
    ;;
  full)
    export ROBONIX_WEBOTS_ROBOT="${ROBONIX_WEBOTS_ROBOT:-tiago_full_webots.urdf}"
    ;;
  *)
    echo "[sim/start] unsupported TIAGo variant: $ROBONIX_TIAGO_VARIANT (choose lite or full)" >&2
    exit 2
    ;;
esac

if [[ "$ROBONIX_TIAGO_VARIANT" == "full" && -z "${ROBONIX_WEBOTS_DOWNLOAD_ALL_ASSETS+x}" ]]; then
  export ROBONIX_WEBOTS_DOWNLOAD_ALL_ASSETS=1
  echo "[sim/start] full TIAGo: enabling the one-time Webots R2025a asset download (~661 MB)"
fi

echo "[sim/start] using Webots world: $ROBONIX_WEBOTS_WORLD"
echo "[sim/start] using TIAGo variant: $ROBONIX_TIAGO_VARIANT"
echo "[sim/start] using robot URDF: $ROBONIX_WEBOTS_ROBOT"

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
EFFECTIVE_HEADLESS_MODE=$(robonix_effective_webots_headless_mode \
  "${WEBOTS_HEADLESS_MODE:-}" "${ROBONIX_SIM_STREAM:-0}")
if [[ "${ROBONIX_FORCE_CPU:-0}" != "1" ]] && command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
  CF+=(-f compose.gpu.yaml)
  robonix_select_nvidia_gpu "${ROBONIX_GPU_ID:-}"
  if [[ "$EFFECTIVE_HEADLESS_MODE" == "nvidia" || "$EFFECTIVE_HEADLESS_MODE" == "auto" ]]; then
    robonix_resolve_nvidia_xorg_modules "$ROBONIX_GPU_ID"
    CF+=(-f compose.nvidia-xorg.yaml)
  fi
else
  if [[ "$EFFECTIVE_HEADLESS_MODE" == "nvidia" ]]; then
    echo "[sim/start] WEBOTS_HEADLESS_MODE=nvidia requires an available NVIDIA GPU" >&2
    exit 1
  fi
  echo "[sim/start] no GPU (or ROBONIX_FORCE_CPU=1) — CPU-only Webots"
fi

# Optional browser-streaming mode (ROBONIX_SIM_STREAM=1). Useful on
# headless servers or xrdp boxes where the host's X session is software-
# rendered (llvmpipe → 0.01x simulation). See compose.stream.yaml and
# bridge/entrypoint.sh.
if [[ "${ROBONIX_SIM_STREAM:-0}" = "1" ]]; then
  export ROBONIX_SIM_STREAM_PORT="${ROBONIX_SIM_STREAM_PORT:-1235}"
  export ROBONIX_SIM_VIEWER_PORT="${ROBONIX_SIM_VIEWER_PORT:-8080}"
  if [[ "$ROBONIX_SIM_STREAM_PORT" = "1234" ]]; then
    echo "[sim/start] error: stream port 1234 is reserved for Webots' raw endpoint" >&2
    exit 1
  fi
  if [[ "$ROBONIX_SIM_STREAM_PORT" = "$ROBONIX_SIM_VIEWER_PORT" ]]; then
    echo "[sim/start] error: stream and viewer ports must be different" >&2
    exit 1
  fi
  CF+=(-f compose.stream.yaml)
  echo "[sim/start] stream mode — merging compose.stream.yaml (optimized WS :${ROBONIX_SIM_STREAM_PORT}, viewer :${ROBONIX_SIM_VIEWER_PORT})"
fi

allow_x11_for_docker() {
  if ! command -v xhost &>/dev/null; then
    echo "[sim/start] warning: xhost not found; Docker may not be allowed to open DISPLAY=$DISPLAY"
    return 0
  fi

  local ok=0
  xhost +SI:localuser:root >/dev/null 2>&1 && ok=1 || true
  xhost +local:root >/dev/null 2>&1 && ok=1 || true
  xhost +local:docker >/dev/null 2>&1 && ok=1 || true
  if [[ "$ok" != "1" ]]; then
    echo "[sim/start] warning: failed to authorize Docker for DISPLAY=$DISPLAY"
    echo "[sim/start] If Webots exits with Qt xcb / 'No protocol specified', run:"
    echo "[sim/start]   xhost +SI:localuser:root +local:root"
  fi
}

allow_x11_for_docker

# Bring sim up detached so we can layer rviz on top before tailing logs.
docker compose "${CF[@]}" up --build -d

# Wait until ros2 inside the container has more than a handful of
# topics — proxy for "webots controller has spawned the robot and is
# publishing /scanner /odom /head_front_camera/* etc."
echo "[sim/start] waiting for sim ros topics..."
for _ in $(seq 1 60); do
    if [[ "$(docker inspect -f '{{.State.Running}}' "$SIM_CT" 2>/dev/null || true)" != "true" ]]; then
        echo "[sim/start] error: simulation container exited during startup" >&2
        docker logs --tail 120 "$SIM_CT" 2>&1 || true
        exit 1
    fi
    n=$(docker exec "$SIM_CT" bash -c \
        'source /opt/ros/humble/setup.bash 2>/dev/null && ros2 topic list 2>/dev/null | wc -l' 2>/dev/null || echo 0)
    if [[ "${n:-0}" -gt 20 ]]; then
        echo "[sim/start] ros up ($n topics)"
        break
    fi
    sleep 2
done

if [[ "${ROBONIX_SIM_STREAM:-0}" = "1" ]]; then
  echo "[sim/start] waiting for browser-stream helpers..."
  stream_ready=0
  for _ in $(seq 1 60); do
    if [[ "$(docker inspect -f '{{.State.Running}}' "$SIM_CT" 2>/dev/null || true)" != "true" ]]; then
      break
    fi
    if docker exec "$SIM_CT" test -f /tmp/webots-stream-ready 2>/dev/null \
      && docker exec "$SIM_CT" python3 /streaming_healthcheck.py \
        --viewer-port "$ROBONIX_SIM_VIEWER_PORT" \
        --stream-port "$ROBONIX_SIM_STREAM_PORT" 2>/dev/null; then
      stream_ready=1
      break
    fi
    sleep 1
  done
  if [[ "$stream_ready" != "1" ]]; then
    echo "[sim/start] error: browser-stream helpers are not ready" >&2
    docker logs --tail 120 "$SIM_CT" 2>&1 || true
    exit 1
  fi

  echo
  echo "[sim/start] ========== webots stream ready =========="
  echo "[sim/start] open ONE of these in a browser:"
  # tailscale first (works from anywhere on the tailnet)
  if command -v tailscale &>/dev/null; then
    tailscale ip -4 2>/dev/null | awk -v port="$ROBONIX_SIM_VIEWER_PORT" '{print "  http://" $1 ":" port "/"}'
  fi
  # then LAN / global v4 addresses
  ip -4 -o addr show scope global 2>/dev/null \
    | awk '{print $4}' | cut -d/ -f1 \
    | awk -v port="$ROBONIX_SIM_VIEWER_PORT" '{print "  http://" $1 ":" port "/"}'
  echo "[sim/start] the viewer auto-fills ws://<that-host>:${ROBONIX_SIM_STREAM_PORT} — just hit Connect"
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
allow_x11_for_docker
echo "[sim/start] launching rviz2 (config: rviz2_default.rviz)"
# Per-user log path: /tmp is shared on multi-tenant boxes and a fixed
# /tmp/rviz2.log file owned by another user blocks rewrite. ${USER:-rviz}
# falls back when USER is unset (e.g. cron/system contexts).
RVIZ_LOG="${TMPDIR:-/tmp}/rviz2-${USER:-rviz}.log"
bash "$SCRIPT_DIR/start_rviz.sh" >"$RVIZ_LOG" 2>&1 &
echo "[sim/start] rviz2 log: $RVIZ_LOG"

# Stay foreground tailing logs so Ctrl-C is the natural stop pattern.
exec docker compose "${CF[@]}" logs -f
