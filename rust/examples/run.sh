#!/usr/bin/env bash
# End-to-end: robonix-atlas → robonix-executor → robonix-pilot → RFC002 packages via **rbnx** → robonix-liaison.
#
# From repository `rust/` (or invoke with absolute path to this script):
#   pip install -r examples/requirements.txt
#   cp examples/.env.example examples/.env
#   ./examples/run.sh
#
# Env:
#   START_VLM_SERVICE=1|0   (default 1) — rbnx validate/build/start packages/vlm_service
#   START_SIM_STACK=1|0     (default 1) — rbnx validate/build/start packages/tiago_sim_stack (Docker + Webots + Nav2 + MCP bridge)
#   START_PILOT=1|0         (default 1) — start robonix-pilot (reasoning/planning service)
#   START_EXECUTOR=1|0      (default 1) — start robonix-executor (tool dispatch service)
#   START_LIAISON=1|0       (default 1) — start robonix-liaison (text interaction loop)
#   SMOKE_USE_EXISTING_ATLAS=1 — do not start robonix-atlas
#   START_MEMSEARCH=1|0     (default 1) — rbnx validate/build/start packages/memsearch_service

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

export ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"
export ROBONIX_META_GRPC_ENDPOINT="${ROBONIX_META_GRPC_ENDPOINT:-$ROBONIX_ATLAS}"
export ROBONIX_ATLAS_ENDPOINT="${ROBONIX_ATLAS_ENDPOINT:-http://$ROBONIX_ATLAS}"
export ROBONIX_EXECUTOR_ENDPOINT="${ROBONIX_EXECUTOR_ENDPOINT:-http://127.0.0.1:50061}"
export ROBONIX_PILOT_ENDPOINT="${ROBONIX_PILOT_ENDPOINT:-http://127.0.0.1:50071}"
export RUST_LOG="${RUST_LOG:-robonix_atlas=info,robonix_pilot=info,robonix_executor=info,robonix_liaison=warn}"
export START_VLM_SERVICE="${START_VLM_SERVICE:-1}"
export START_SIM_STACK="${START_SIM_STACK:-1}"
export START_PILOT="${START_PILOT:-1}"
export START_EXECUTOR="${START_EXECUTOR:-1}"
export START_LIAISON="${START_LIAISON:-1}"
export START_MEMSEARCH="${START_MEMSEARCH:-1}"
export START_SPEECH_SERVICE="${START_SPEECH_SERVICE:-1}"
export START_AUDIO_DRIVER="${START_AUDIO_DRIVER:-1}"

export PYTHONPATH="${PACKAGES}/vlm_service:${PACKAGES}/memsearch_service:${PACKAGES}/speech_service:${PACKAGES}/audio_driver${PYTHONPATH:+:$PYTHONPATH}"

rbnx() {
  (cd "$RUST_ROOT" && cargo run -p robonix-cli -- "$@")
}

rbnx_validate_build() {
  local abs_pkg="$1"
  echo "[example] rbnx validate $abs_pkg"
  rbnx validate "$abs_pkg"
  echo "[example] rbnx build -p $abs_pkg"
  rbnx build -p "$abs_pkg"
}

check_python_dep() {
  local module="$1"
  python - <<PY >/dev/null 2>&1
import importlib.util
import sys
sys.exit(0 if importlib.util.find_spec("$module") else 1)
PY
}

echo "[example] checking required Python modules..."
missing=()
for m in grpc openai mcp memsearch numpy PIL uvicorn; do
  check_python_dep "$m" || missing+=("$m")
done
if [ "${#missing[@]}" -ne 0 ]; then
  echo "[example] missing modules: ${missing[*]}"
  echo "[example] install: pip install -r \"${EXAMPLES_ROOT}/requirements.txt\""
  echo "[example] or make sure your virtual environment is active."
  exit 1
fi

if ! python -c 'import grpc; p=[int(x) for x in grpc.__version__.split(".")[:3]]; raise SystemExit(0 if tuple(p + [0]*(3-len(p))) >= (1,60,0) else 1)' 2>/dev/null; then
  echo "[example] grpcio>=1.60 required — pip install -r \"${EXAMPLES_ROOT}/requirements.txt\""
  exit 1
fi

if [ "$START_SIM_STACK" = "1" ]; then
  if [ -z "${DISPLAY:-}" ]; then
    echo "[example] warning: DISPLAY is unset — Webots/rviz2 GUI will not show."
    echo "[example]   Set DISPLAY and X11 auth (see packages/tiago_sim_stack/README.md)."
  else
    if command -v xhost &>/dev/null; then
      xhost +local:docker 2>/dev/null || true
    else
      echo "[example] warning: xhost not found — run 'xhost +local:docker' manually for GUI."
    fi
  fi
fi

pkill -9 -f 'robonix-atlas' 2>/dev/null || true
pkill -9 -f 'robonix-executor' 2>/dev/null || true
pkill -9 -f 'robonix-pilot' 2>/dev/null || true
pkill -9 -f 'robonix-liaison' 2>/dev/null || true
pkill -9 -f 'vlm_service.service' 2>/dev/null || true
pkill -9 -f 'memsearch_service.service' 2>/dev/null || true
pkill -9 -f 'tiago_bridge.node' 2>/dev/null || true
pkill -9 -f 'speech_service.service' 2>/dev/null || true
pkill -9 -f 'audio_driver.node' 2>/dev/null || true
sleep 0.3

SIM_STACK_DIR="${PACKAGES}/tiago_sim_stack"

# Ensure local workspace skills override registry-provided paths.
# Pilot merges skills as: registry (lowest) → ~/.robonix/skills → ROBONIX_SKILLS_EXTRA_DIRS (last wins).
# Without this, registry Skill.path may point at a different checkout.
export ROBONIX_SKILLS_EXTRA_DIRS="${SIM_STACK_DIR}/skills${ROBONIX_SKILLS_EXTRA_DIRS:+:${ROBONIX_SKILLS_EXTRA_DIRS}}"

cleanup() {
  echo "[example] shutting down..."
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
    echo "[example] START_SIM_STACK=1 requires Docker."
    exit 1
  fi
  rbnx_validate_build "$PACKAGES/tiago_sim_stack"
fi

# ── 1. Atlas (control plane) ──────────────────────────────────────────────────
if [[ "${SMOKE_USE_EXISTING_ATLAS:-0}" != "1" ]]; then
  echo "[example] starting robonix-atlas (control plane)..."
  (cd "$RUST_ROOT" && cargo run -p robonix-atlas) &
  sleep 2
fi

RBNX_START_OPTS=(start --endpoint "$ROBONIX_ATLAS")

if [ "$START_VLM_SERVICE" = "1" ]; then
  echo "[example] rbnx start vlm_service (background)..."
  (cd "$RUST_ROOT" && cargo run -p robonix-cli -- "${RBNX_START_OPTS[@]}" -p "$PACKAGES/vlm_service" -n com.robonix.services.vlm) &
  sleep 1
fi

if [ "$START_MEMSEARCH" = "1" ]; then
  echo "[example] rbnx start memsearch_service (background)..."
  rbnx_validate_build "$PACKAGES/memsearch_service"
  (cd "$RUST_ROOT" && cargo run -p robonix-cli -- "${RBNX_START_OPTS[@]}" -p "$PACKAGES/memsearch_service" -n com.robonix.services.memsearch) &
  sleep 1
fi

if [ "$START_AUDIO_DRIVER" = "1" ]; then
  echo "[example] rbnx start audio_driver (background)..."
  rbnx_validate_build "$PACKAGES/audio_driver"
  (cd "$RUST_ROOT" && cargo run -p robonix-cli -- "${RBNX_START_OPTS[@]}" -p "$PACKAGES/audio_driver" -n com.robonix.prm.audio) &
  sleep 1
fi

if [ "$START_SPEECH_SERVICE" = "1" ]; then
  echo "[example] rbnx start speech_service (background)..."
  rbnx_validate_build "$PACKAGES/speech_service"
  (cd "$RUST_ROOT" && cargo run -p robonix-cli -- "${RBNX_START_OPTS[@]}" -p "$PACKAGES/speech_service" -n com.robonix.services.speech) &
  sleep 1
fi

if [ "$START_SIM_STACK" = "1" ]; then
  echo "[example] rbnx start tiago_sim_stack (background, docker compose)..."
  (cd "$RUST_ROOT" && cargo run -p robonix-cli -- "${RBNX_START_OPTS[@]}" -p "$PACKAGES/tiago_sim_stack" -n com.robonix.prm.tiago) &
  sleep 4
fi

# ── 2. Executor (tool dispatch runtime) ──────────────────────────────────────
if [ "$START_EXECUTOR" = "1" ]; then
  echo "[example] starting robonix-executor (background)..."
  (cd "$RUST_ROOT" && cargo run -p robonix-executor) &
  sleep 1
fi

# ── 3. Pilot (VLM reasoning service) ─────────────────────────────────────────
if [ "$START_PILOT" = "1" ]; then
  echo "[example] starting robonix-pilot (background)..."
  (cd "$RUST_ROOT" && cargo run -p robonix-pilot) &
  sleep 2
fi

# ── 4. Liaison (user-facing gRPC server) ─────────────────────────────────────
if [ "$START_LIAISON" = "1" ]; then
  echo "[example] starting robonix-liaison (background, gRPC on :50081)..."
  (cd "$RUST_ROOT" && cargo run -p robonix-liaison) &
  sleep 2
fi

echo "[example] stack ready — use 'rbnx chat' to connect (Ctrl+C to stop)."
wait
