#!/usr/bin/env bash
# ManiSkill3 VLA demo: env + perception + VLA nodes → robonix-pilot.
#
# Usage:
#   cd rust/examples/packages/maniskill_vla_demo
#   ./run.sh setup   # create uv venv + install all deps + download assets
#   ./run.sh start   # activate venv + launch all nodes + agent
#
# Env:
#   VLA_POLICY=scripted|octo  (default: octo)
#   MANISKILL_ENV_ID          (default: ReplicaCADTidyHouseTrain_SceneManipulation-v1)
#                              Others: ReplicaCADSetTableTrain_SceneManipulation-v1
#                                      ReplicaCADPrepareGroceriesTrain_SceneManipulation-v1
#                              Simple: PickCube-v1  (no asset download needed)
#   MANISKILL_CONTROL_MODE    (default: pd_ee_delta_pose)
#   MANISKILL_CAM_W / _CAM_H  (default: 640x480)
#   MANISKILL_SHADER_PACK     (default:  rt-fast) sensor camera shader:
#                               minimal | default | rt-fast | rt-med | rt
#   PERCEPTION_BACKEND        (default: yolo_world) yolo_world | grounding_dino
#   PERCEPTION_YOLO_WEIGHTS   (default: yolov8s-worldv2.pt) ultralytics YOLO-World weights (auto-cached)
#   VIZ_DETECT_BACKEND        (default: yolo_world) same as PERCEPTION_BACKEND for viz overlay
#   HF_ENDPOINT               (default: https://hf-mirror.com)
#   ROBONIX_ATLAS            (default: 127.0.0.1:50051)
#   START_VLM_SERVICE=1|0     (default: 1)
#   START_PILOT=1|0           (default: 1)
#   START_VIZ=1|0             (default: 1) — Rerun visualizer
#   START_MAPPING=1|0         (default: 0) — ROS2/RTAB-Map Docker container (optional ROS test)
#   BRIDGE_FPS                (default: 10) — env_node poll rate inside the RTAB-Map container
#   VIZ_DETECT_QUERY          (default: "object . cup . box") detection query
#   VIZ_DETECT_DEVICE         (default: auto) detector device: auto|cuda|cpu
#   VIZ_DETECT_FP16           (default: 1) use fp16 on CUDA for lower VRAM
#   VIZ_NO_DETECT=1           disable detection overlay in viz_node
#   VIZ_FPS                   (default: 30, lower memory/CPU pressure)
#   RERUN_GRPC_PORT           (default: 9877)
#   DEMO_MEMORY_PROFILE       (default: balanced) low|balanced
#   PERCEPTION_DEVICE         (default: auto) auto|cuda|cpu
#   PERCEPTION_FP16           (default: 1) use fp16 on CUDA for lower VRAM

set -euo pipefail

PKG_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXAMPLES_ROOT="$(cd "$PKG_ROOT/../.." && pwd)"
PACKAGES="$EXAMPLES_ROOT/packages"
RUST_ROOT="$(cd "$EXAMPLES_ROOT/.." && pwd)"
VENV="$PKG_ROOT/.venv"

export ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"
export ROBONIX_META_GRPC_ENDPOINT="${ROBONIX_META_GRPC_ENDPOINT:-$ROBONIX_ATLAS}"
export RUST_LOG="${RUST_LOG:-robonix_atlas=info,robonix_pilot=info,robonix_executor=info}"
export VLA_POLICY="${VLA_POLICY:-octo}"
export MANISKILL_ENV_ID="${MANISKILL_ENV_ID:-ReplicaCADTidyHouseTrain_SceneManipulation-v1}"
export MANISKILL_CONTROL_MODE="${MANISKILL_CONTROL_MODE:-pd_ee_delta_pose}"
export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"
export START_VLM_SERVICE="${START_VLM_SERVICE:-1}"
export START_PILOT="${START_PILOT:-1}"
export START_EXECUTOR="${START_EXECUTOR:-1}"
export START_VIZ="${START_VIZ:-1}"
export START_MAPPING="${START_MAPPING:-0}"
export VIZ_DETECT_QUERY="${VIZ_DETECT_QUERY:-object . cup . box}"
export VIZ_FPS="${VIZ_FPS:-30}"
export RERUN_GRPC_PORT="${RERUN_GRPC_PORT:-9877}"
export DEMO_MEMORY_PROFILE="${DEMO_MEMORY_PROFILE:-balanced}"
export PERCEPTION_DEVICE="${PERCEPTION_DEVICE:-auto}"
export PERCEPTION_FP16="${PERCEPTION_FP16:-1}"
export PERCEPTION_BACKEND="${PERCEPTION_BACKEND:-yolo_world}"
export PERCEPTION_YOLO_WEIGHTS="${PERCEPTION_YOLO_WEIGHTS:-yolov8s-worldv2.pt}"
export VIZ_DETECT_BACKEND="${VIZ_DETECT_BACKEND:-yolo_world}"
export VIZ_YOLO_WEIGHTS="${VIZ_YOLO_WEIGHTS:-yolov8s-worldv2.pt}"
export MANISKILL_SHADER_PACK="${MANISKILL_SHADER_PACK:-rt-fast}"

# Memory-focused defaults (safe for long runs on smaller GPUs)
export XLA_PYTHON_CLIENT_PREALLOCATE="${XLA_PYTHON_CLIENT_PREALLOCATE:-false}"
export TOKENIZERS_PARALLELISM="${TOKENIZERS_PARALLELISM:-false}"
export TF_CPP_MIN_LOG_LEVEL="${TF_CPP_MIN_LOG_LEVEL:-2}"
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True,max_split_size_mb:64}"

if [ "$DEMO_MEMORY_PROFILE" = "low" ]; then
  # Keep camera reasonably clear but reduce memory/throughput pressure.
  export MANISKILL_CAM_W="${MANISKILL_CAM_W:-512}"
  export MANISKILL_CAM_H="${MANISKILL_CAM_H:-384}"
else
  export MANISKILL_CAM_W="${MANISKILL_CAM_W:-640}"
  export MANISKILL_CAM_H="${MANISKILL_CAM_H:-480}"
fi

if [ -f "$EXAMPLES_ROOT/.env" ]; then
  set -a; source "$EXAMPLES_ROOT/.env"; set +a
fi

# ── Helpers ───────────────────────────────────────────────────────────────────

rbnx() {
  (cd "$RUST_ROOT" && cargo run -p robonix-cli -- "$@")
}

rbnx_validate_build() {
  local abs_pkg="$1"
  echo "[demo] rbnx validate $abs_pkg"
  rbnx validate "$abs_pkg"
  echo "[demo] rbnx build -p $abs_pkg"
  rbnx build -p "$abs_pkg"
}

check_python_dep() {
  local module="$1"
  python3 - <<PY >/dev/null 2>&1
import importlib.util, sys
sys.exit(0 if importlib.util.find_spec("$module") else 1)
PY
}

# Detect X11/Wayland display for launching GUI apps
_detect_gui_env() {
  [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ] && return 0
  local uid; uid=$(id -u)
  local xdg="${XDG_RUNTIME_DIR:-/run/user/$uid}"
  for sock in "$xdg"/wayland-{0,1,2,3}; do
    [ -S "$sock" ] && export WAYLAND_DISPLAY="${sock##*/}" && \
      echo "[demo] detected Wayland: $WAYLAND_DISPLAY" && return 0
  done
  for n in 0 1 2 3; do
    if [ -S "/tmp/.X11-unix/X$n" ]; then
      export DISPLAY=":$n"
      echo "[demo] detected X11: $DISPLAY"
      [ -f "${XAUTHORITY:-$HOME/.Xauthority}" ] && \
        export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
      return 0
    fi
  done
  echo "[demo] WARNING: no display found — Rerun window may not open"
  return 1
}

# PIDs of background jobs started by this script
_DEMO_PIDS=()
bg() { "$@" & _DEMO_PIDS+=($!); }

cleanup() {
  echo "[demo] shutting down..."
  for pid in "${_DEMO_PIDS[@]}"; do
    local pgid
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    if [ -n "$pgid" ] && [ "$pgid" != "$$" ]; then
      kill -- -"$pgid" 2>/dev/null || true
    else
      kill "$pid" 2>/dev/null || true
    fi
  done
  pkill -TERM -f 'python3 -m maniskill_vla_demo' 2>/dev/null || true
  pkill -TERM -f 'target/debug/robonix-atlas'    2>/dev/null || true
  pkill -TERM -f 'target/debug/robonix-executor' 2>/dev/null || true
  pkill -TERM -f 'target/debug/robonix-pilot'    2>/dev/null || true
  pkill -TERM -f 'target/debug/rbnx start'       2>/dev/null || true
  pkill -TERM -f "rerun rerun+http"               2>/dev/null || true
  sleep 0.5
  pkill -KILL -f 'python3 -m maniskill_vla_demo' 2>/dev/null || true
  pkill -KILL -f 'target/debug/robonix-atlas'    2>/dev/null || true
  pkill -KILL -f 'target/debug/robonix-executor' 2>/dev/null || true
  pkill -KILL -f 'target/debug/robonix-pilot'    2>/dev/null || true
  pkill -KILL -f "rerun rerun+http"               2>/dev/null || true
  wait 2>/dev/null || true
  "$EXAMPLES_ROOT/stop.sh"
  echo "[demo] all processes stopped."
}

pre_cleanup() {
  echo "[demo] cleaning up any leftover processes..."
  pkill -TERM -f 'python3 -m maniskill_vla_demo' 2>/dev/null || true
  pkill -TERM -f 'target/debug/robonix-atlas'    2>/dev/null || true
  pkill -TERM -f 'target/debug/robonix-executor' 2>/dev/null || true
  pkill -TERM -f 'target/debug/robonix-pilot'    2>/dev/null || true
  pkill -TERM -f 'target/debug/rbnx start'       2>/dev/null || true
  sleep 0.5
  pkill -KILL -f 'python3 -m maniskill_vla_demo' 2>/dev/null || true
  pkill -KILL -f 'target/debug/robonix-atlas'    2>/dev/null || true
  pkill -KILL -f 'target/debug/robonix-executor' 2>/dev/null || true
  pkill -KILL -f 'target/debug/robonix-pilot'    2>/dev/null || true
}

# ── setup ────────────────────────────────────────────────────────────────────

cmd_setup() {
  cd "$PKG_ROOT"

  echo "[demo] ensuring Python 3.11 is available..."
  uv python install 3.11

  echo "[demo] creating venv + installing all dependencies (uv sync)..."
  echo "  This installs Octo's full dependency stack (JAX 0.4.20, TF 2.15, etc.)"
  echo "  First run may take 10-20 minutes due to large packages."
  # UV_INDEX_URL can be overridden for PyPI mirrors
  uv sync

  echo "[demo] downloading ManiSkill3 scene assets..."
  HF_ENDPOINT="$HF_ENDPOINT" uv run python3 -m mani_skill.utils.download_asset ReplicaCAD \
    || echo "[demo] WARNING: ReplicaCAD download failed — indoor scene envs will not work"
  HF_ENDPOINT="$HF_ENDPOINT" uv run python3 -m mani_skill.utils.download_asset ReplicaCADRearrange \
    || echo "[demo] WARNING: ReplicaCADRearrange download failed — rearrange tasks will not work"

  echo "[demo] building gRPC proto stubs..."
  rbnx_validate_build "$PKG_ROOT"

  echo ""
  echo "[demo] setup complete."
  echo "  Python: $(uv run python3 --version)"
  echo "  Venv:   $VENV"
  echo "  Run:    ./run.sh start"
}

# ── start ────────────────────────────────────────────────────────────────────

cmd_start() {
  # ── Activate the project venv ─────────────────────────────────────────────
  if [ ! -f "$VENV/bin/activate" ]; then
    echo "[demo] ERROR: venv not found at $VENV"
    echo "[demo]   Run first:  ./run.sh setup"
    exit 1
  fi
  # shellcheck source=/dev/null
  source "$VENV/bin/activate"
  echo "[demo] venv activated  ($(python3 --version), $(which python3))"
  echo "[demo] memory profile=${DEMO_MEMORY_PROFILE}  cam=${MANISKILL_CAM_W}x${MANISKILL_CAM_H}  viz_fps=${VIZ_FPS}"

  echo "[demo] checking Python deps..."
  missing=()
  for m in grpc mcp numpy PIL uvicorn gymnasium mani_skill rerun; do
    check_python_dep "$m" || missing+=("$m")
  done
  if [ "$START_VLM_SERVICE" = "1" ]; then
    check_python_dep "openai" || missing+=("openai")
  fi
  if [ "${#missing[@]}" -ne 0 ]; then
    echo "[demo] missing: ${missing[*]}"
    echo "[demo] run: ./run.sh setup"
    if [[ " ${missing[*]} " == *" openai "* ]]; then
      echo "[demo] note: vlm_service requires Python package 'openai'"
    fi
    exit 1
  fi

  pre_cleanup
  trap cleanup INT TERM EXIT

  rbnx_validate_build "$PKG_ROOT"
  if [ "$START_VLM_SERVICE" = "1" ]; then
    rbnx_validate_build "$PACKAGES/vlm_service"
  fi

  RBNX_START_OPTS=(start --endpoint "$ROBONIX_ATLAS")

  # robonix-atlas (control plane)
  echo "[demo] starting robonix-atlas..."
  bg bash -c "cd '$RUST_ROOT' && exec cargo run -p robonix-atlas"
  sleep 2

  # vlm_service
  if [ "$START_VLM_SERVICE" = "1" ]; then
    echo "[demo] starting vlm_service..."
    bg bash -c "cd '$RUST_ROOT' && exec cargo run -p robonix-cli -- \
      ${RBNX_START_OPTS[*]} -p '$PACKAGES/vlm_service' -n com.robonix.services.vlm"
    sleep 1
  fi

  # env_node (ManiSkill3 init is slow)
  echo "[demo] starting env_node..."
  bg bash -c "cd '$RUST_ROOT' && exec cargo run -p robonix-cli -- \
    ${RBNX_START_OPTS[*]} -p '$PKG_ROOT' -n com.robonix.demo.maniskill"
  sleep 5

  # perception_node
  echo "[demo] starting perception_node..."
  bg bash -c "cd '$RUST_ROOT' && exec cargo run -p robonix-cli -- \
    ${RBNX_START_OPTS[*]} -p '$PKG_ROOT' -n com.robonix.demo.perception"
  sleep 3

  # vla_node
  echo "[demo] starting vla_node (policy=$VLA_POLICY)..."
  bg bash -c "cd '$RUST_ROOT' && VLA_POLICY='$VLA_POLICY' exec cargo run -p robonix-cli -- \
    ${RBNX_START_OPTS[*]} -p '$PKG_ROOT' -n com.robonix.demo.vla"
  sleep 2

  # viz_node + Rerun desktop viewer
  if [ "$START_VIZ" = "1" ]; then
    echo "[demo] starting viz_node (Rerun data server)..."
    VIZ_CMD=(python3 -m maniskill_vla_demo.viz_node --fps "${VIZ_FPS}")
    VIZ_CMD+=(--detect-query "${VIZ_DETECT_QUERY}")
    [ "${VIZ_NO_DETECT:-0}" = "1" ] && VIZ_CMD+=(--no-detect)
    bg "${VIZ_CMD[@]}"
    sleep 2

    _RERUN_URL="rerun+http://localhost:${RERUN_GRPC_PORT}/proxy"
    if command -v rerun >/dev/null 2>&1; then
      _detect_gui_env
      export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
      export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}"
      echo "[demo] opening Rerun desktop viewer → ${_RERUN_URL}"
      echo "[demo]   DISPLAY='${DISPLAY:-}'  XAUTHORITY='${XAUTHORITY:-}'"
      bg rerun "$_RERUN_URL"
      sleep 1
    else
      echo "[demo] WARNING: 'rerun' CLI not found — install with: uv run pip install rerun-cli"
      echo "[demo]   Then run:  rerun ${_RERUN_URL}"
    fi
  fi

  # RTAB-Map mapping node (Docker container)
  # Builds the image on first run (~1 min), then starts the container.
  # Requires: docker, env_node running, proto_stubs/ generated by build step.
  if [ "$START_MAPPING" = "1" ]; then
    if ! command -v docker >/dev/null 2>&1; then
      echo "[demo] WARNING: docker not found — skipping RTAB-Map mapping node"
      echo "[demo]   Install Docker and re-run, or set START_MAPPING=0 to suppress"
    else
      echo "[demo] starting RTAB-Map mapping node (Docker)..."
      echo "[demo]   (first run builds the image — may take ~1 min)"
      bg bash -c "cd '$PKG_ROOT' && \
        ROBONIX_ATLAS='${ROBONIX_ATLAS}' \
        RERUN_GRPC_URL='rerun+http://localhost:${RERUN_GRPC_PORT}' \
        BRIDGE_FPS='${BRIDGE_FPS:-10}' \
        MAP_LOG_INTERVAL='${MAP_LOG_INTERVAL:-30}' \
        bash docker/run_rtabmap.sh"
      sleep 2
    fi
  fi

  # robonix-executor + robonix-pilot (background gRPC services)
  if [ "$START_EXECUTOR" = "1" ]; then
    echo "[demo] starting robonix-executor (background)..."
    bg bash -c "cd '$RUST_ROOT' && exec cargo run -p robonix-executor"
    sleep 1
  fi
  if [ "$START_PILOT" = "1" ]; then
    echo "[demo] starting robonix-pilot (background, headless gRPC)..."
    bg bash -c "cd '$RUST_ROOT' && exec cargo run -p robonix-pilot"
    sleep 3
  fi

  echo ""
  echo "============================================================"
  echo "  All nodes running.  Use 'rbnx chat' to interact."
  echo ""
  echo "  rbnx chat --server ${ROBONIX_ATLAS}"
  echo ""
  echo "  Or generate a topology graph:"
  echo "  rbnx graph --server ${ROBONIX_ATLAS} -o topology.png"
  echo ""
  echo "  Press Ctrl+C to stop all nodes."
  echo "============================================================"
  echo ""
  wait
}

# ── dispatch ─────────────────────────────────────────────────────────────────

case "${1:-help}" in
  setup)  cmd_setup ;;
  start)  cmd_start ;;
  *)
    echo "Usage: $0 {setup|start}"
    echo ""
    echo "  setup   Create .venv (Python 3.11), install all deps, download assets"
    echo "  start   Activate .venv and launch all nodes + robonix-pilot"
    echo ""
  echo "Environment variables (see top of this file for full list):"
  echo "  VLA_POLICY=octo|scripted   MANISKILL_ENV_ID=...   HF_ENDPOINT=..."
  echo "  START_MAPPING=0            — skip RTAB-Map Docker (e.g. if Docker unavailable)"
    ;;
esac
