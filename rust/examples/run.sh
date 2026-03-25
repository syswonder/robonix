#!/usr/bin/env bash
# End-to-end: robonix-server → RFC002 packages via **rbnx** (validate / build / start) → robonix-agent.
#
# From repository `rust/` (or invoke with absolute path to this script):
#   pip install -r examples/requirements.txt
#   cp examples/.env.example examples/.env
#   ./examples/run.sh
#
# Env:
#   START_VLM_SERVICE=1|0     (default 1) — rbnx validate/build/start packages/vlm_service
#   START_SIM_STACK=1|0     (default 1) — rbnx validate/build/start packages/tiago_sim_stack (Docker + Webots + Nav2 + MCP bridge)
#   START_AGENT=1|0         (default 1)
#   SMOKE_USE_EXISTING_SERVER=1 — do not start robonix-server

set -euo pipefail

RUST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXAMPLES_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGES="${EXAMPLES_ROOT}/packages"

cd "$EXAMPLES_ROOT"

if [ -f .env ]; then
  set -a
  # shellcheck source=/dev/null
  source .env
  set +a
fi

export ROBONIX_SERVER="${ROBONIX_SERVER:-127.0.0.1:50051}"
export ROBONIX_META_GRPC_ENDPOINT="${ROBONIX_META_GRPC_ENDPOINT:-$ROBONIX_SERVER}"
export RUST_LOG="${RUST_LOG:-robonix_server=info,robonix_agent=info}"
export START_VLM_SERVICE="${START_VLM_SERVICE:-1}"
export START_SIM_STACK="${START_SIM_STACK:-1}"
export START_AGENT="${START_AGENT:-1}"

export PYTHONPATH="${PACKAGES}/vlm_service${PYTHONPATH:+:$PYTHONPATH}"

rbnx() {
  (cd "$RUST_ROOT" && cargo run -p robonix-cli -- "$@")
}

rbnx_validate_build() {
  local abs_pkg="$1"
  echo "[e2e] rbnx validate $abs_pkg"
  rbnx validate "$abs_pkg"
  echo "[e2e] rbnx build -p $abs_pkg"
  rbnx build -p "$abs_pkg"
}

check_python_dep() {
  local module="$1"
  python3 - <<PY >/dev/null 2>&1
import importlib.util
import sys
sys.exit(0 if importlib.util.find_spec("$module") else 1)
PY
}

echo "[e2e] checking required Python modules..."
missing=()
for m in grpc openai mcp numpy PIL uvicorn; do
  check_python_dep "$m" || missing+=("$m")
done
if [ "${#missing[@]}" -ne 0 ]; then
  echo "[e2e] missing modules: ${missing[*]}"
  echo "[e2e] install: pip install -r \"${EXAMPLES_ROOT}/requirements.txt\""
  exit 1
fi

if ! python3 -c 'import grpc; p=[int(x) for x in grpc.__version__.split(".")[:3]]; raise SystemExit(0 if tuple(p + [0]*(3-len(p))) >= (1,78,0) else 1)' 2>/dev/null; then
  echo "[e2e] grpcio>=1.78 required — pip install -r \"${EXAMPLES_ROOT}/requirements.txt\""
  exit 1
fi

if [ "$START_SIM_STACK" = "1" ]; then
  if [ -z "${DISPLAY:-}" ]; then
    echo "[e2e] warning: DISPLAY is unset — Webots/rviz2 GUI will not show."
    echo "[e2e]   Set DISPLAY and X11 auth (see packages/tiago_sim_stack/README.md)."
  else
    if command -v xhost &>/dev/null; then
      xhost +local:docker 2>/dev/null || true
    else
      echo "[e2e] warning: xhost not found — run 'xhost +local:docker' manually for GUI."
    fi
  fi
fi

pkill -9 -f 'robonix-server' 2>/dev/null || true
pkill -9 -f 'robonix-agent' 2>/dev/null || true
pkill -9 -f 'vlm_service.service' 2>/dev/null || true
pkill -9 -f 'tiago_bridge.node' 2>/dev/null || true
sleep 0.3

SIM_STACK_DIR="${PACKAGES}/tiago_sim_stack"

# Ensure local workspace skills override registry-provided paths.
# The agent merges skills as: registry (lowest) → ~/.robonix/skills → ROBONIX_SKILLS_EXTRA_DIRS (last wins).
# Without this, registry Skill.path may point at a different checkout.
export ROBONIX_SKILLS_EXTRA_DIRS="${SIM_STACK_DIR}/skills${ROBONIX_SKILLS_EXTRA_DIRS:+:${ROBONIX_SKILLS_EXTRA_DIRS}}"

cleanup() {
  echo "[e2e] shutting down..."
  if [ "${START_SIM_STACK:-0}" = "1" ] && [ -d "${SIM_STACK_DIR}" ]; then
    (cd "$SIM_STACK_DIR" && docker compose -f compose.yaml down) 2>/dev/null || true
  fi
  kill $(jobs -p) 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

if [ "$START_VLM_SERVICE" = "1" ]; then
  rbnx_validate_build "$PACKAGES/vlm_service"
fi

if [ "$START_SIM_STACK" = "1" ]; then
  if ! command -v docker >/dev/null 2>&1; then
    echo "[e2e] START_SIM_STACK=1 requires Docker."
    exit 1
  fi
  rbnx_validate_build "$PACKAGES/tiago_sim_stack"
fi

if [[ "${SMOKE_USE_EXISTING_SERVER:-0}" != "1" ]]; then
  echo "[e2e] starting robonix-server..."
  (cd "$RUST_ROOT" && cargo run -p robonix-server) &
  sleep 2
fi

RBNX_START_OPTS=(start --endpoint "$ROBONIX_SERVER")

if [ "$START_VLM_SERVICE" = "1" ]; then
  echo "[e2e] rbnx start vlm_service (background)..."
  (cd "$RUST_ROOT" && cargo run -p robonix-cli -- "${RBNX_START_OPTS[@]}" -p "$PACKAGES/vlm_service" -n com.robonix.services.vlm) &
  sleep 1
fi

if [ "$START_SIM_STACK" = "1" ]; then
  echo "[e2e] rbnx start tiago_sim_stack (background, docker compose)..."
  (cd "$RUST_ROOT" && cargo run -p robonix-cli -- "${RBNX_START_OPTS[@]}" -p "$PACKAGES/tiago_sim_stack" -n com.robonix.prm.tiago) &
  sleep 4
fi

if [ "$START_AGENT" = "1" ]; then
  echo "[e2e] starting robonix-agent (foreground, stdin)..."
  (cd "$RUST_ROOT" && cargo run -p robonix-agent)
else
  echo "[e2e] START_AGENT=0 — waiting (Ctrl+C to exit)."
  wait
fi
