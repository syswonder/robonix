#!/usr/bin/env bash
# clawhub_skills — full-stack launcher.
#
# Starts the complete Robonix runtime stack:
#   1. robonix-atlas       control plane               (:50051)
#   2. vlm_service         VLM chat backend (OpenAI-compatible, required by Pilot)
#   3. clawhub_skills      Agent Skills bridge (imports from ClawHub)
#   4. robonix-executor    tool dispatch runtime        (:50061)
#   5. robonix-pilot       VLM reasoning service        (:50071)
#   6. robonix-liaison     user interaction layer       (:50081)
#
# Prerequisites:
#   1. Build the workspace:  cd rust && cargo build --workspace
#   2. Configure VLM:        cp examples/.env.example examples/.env
#                            then edit examples/.env to set:
#                              VLM_API_BASE=https://api.openai.com/v1  (or compatible endpoint)
#                              VLM_API_KEY=sk-...
#                              VLM_MODEL=gpt-4o  (or any chat-completions model)
#   3. Install Python deps:  pip install -r examples/requirements.txt
#
# The .env file is auto-loaded from examples/.env.
# Without a valid VLM config, Pilot will fail to discover the VLM provider.
#
# Usage:
#   ./run.sh                                     # full stack
#   START_VLM_SERVICE=0 START_PILOT=0 ./run.sh   # no VLM, skills-only
#   SMOKE_USE_EXISTING_ATLAS=1 ./run.sh          # reuse running atlas
#
# Env toggles:
#   START_VLM_SERVICE=1|0   (default 1)  VLM chat service
#   START_EXECUTOR=1|0      (default 1)  tool dispatch
#   START_PILOT=1|0         (default 1)  VLM reasoning
#   START_LIAISON=1|0       (default 1)  user interaction
#   SMOKE_USE_EXISTING_ATLAS=1           reuse already-running atlas
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUST_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
EXAMPLES_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PACKAGES="$EXAMPLES_ROOT/packages"

# ── Load .env ────────────────────────────────────────────────────────────────
if [ -f "$EXAMPLES_ROOT/.env" ]; then
  echo "[clawhub] loading config from examples/.env"
  set -a; source "$EXAMPLES_ROOT/.env"; set +a
else
  echo "[clawhub] WARNING: examples/.env not found"
  echo "[clawhub]   cp examples/.env.example examples/.env"
  echo "[clawhub]   then set VLM_API_BASE, VLM_API_KEY, VLM_MODEL"
  echo ""
fi

export ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"
export ROBONIX_META_GRPC_ENDPOINT="${ROBONIX_META_GRPC_ENDPOINT:-$ROBONIX_ATLAS}"
export ROBONIX_ATLAS_ENDPOINT="${ROBONIX_ATLAS_ENDPOINT:-http://$ROBONIX_ATLAS}"
export ROBONIX_EXECUTOR_ENDPOINT="${ROBONIX_EXECUTOR_ENDPOINT:-http://127.0.0.1:50061}"
export ROBONIX_PILOT_ENDPOINT="${ROBONIX_PILOT_ENDPOINT:-http://127.0.0.1:50071}"
export RUST_LOG="${RUST_LOG:-robonix_atlas=info,robonix_pilot=info,robonix_executor=info,robonix_liaison=warn}"

export START_VLM_SERVICE="${START_VLM_SERVICE:-1}"
export START_EXECUTOR="${START_EXECUTOR:-1}"
export START_PILOT="${START_PILOT:-1}"
export START_LIAISON="${START_LIAISON:-1}"

export PYTHONPATH="${PACKAGES}/vlm_service:${PACKAGES}/memsearch_service${PYTHONPATH:+:$PYTHONPATH}"

rbnx() {
  cargo run --manifest-path "$RUST_ROOT/Cargo.toml" -p robonix-cli -- "$@"
}

rbnx_validate_build() {
  local pkg="$1"
  echo "[clawhub] rbnx validate + build: $(basename "$pkg")"
  rbnx validate "$pkg"
  rbnx build -p "$pkg"
}

# ── Cleanup on exit ──────────────────────────────────────────────────────────
cleanup() {
  echo ""
  echo "[clawhub] shutting down..."
  kill $(jobs -p) 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# Kill stale processes
pkill -9 -f 'robonix-atlas' 2>/dev/null || true
pkill -9 -f 'robonix-executor' 2>/dev/null || true
pkill -9 -f 'robonix-pilot' 2>/dev/null || true
pkill -9 -f 'robonix-liaison' 2>/dev/null || true
pkill -9 -f 'vlm_service.service' 2>/dev/null || true
pkill -9 -f 'clawhub_bridge' 2>/dev/null || true
sleep 0.3

# ── 1. Atlas (control plane) ─────────────────────────────────────────────────
if [[ "${SMOKE_USE_EXISTING_ATLAS:-0}" != "1" ]]; then
  echo "[clawhub] starting robonix-atlas (control plane, :50051)..."
  cargo run --manifest-path "$RUST_ROOT/Cargo.toml" -p robonix-atlas &
  sleep 2
fi

RBNX_START_OPTS=(start --endpoint "$ROBONIX_ATLAS")

# ── 2. VLM service (required by Pilot for reasoning) ────────────────────────
if [ "$START_VLM_SERVICE" = "1" ]; then
  rbnx_validate_build "$PACKAGES/vlm_service"
  echo "[clawhub] starting vlm_service..."
  cargo run --manifest-path "$RUST_ROOT/Cargo.toml" -p robonix-cli -- "${RBNX_START_OPTS[@]}" \
    -p "$PACKAGES/vlm_service" -n com.robonix.services.vlm &
  sleep 1
fi

# ── 3. ClawHub skills package (build + start) ────────────────────────────────
rbnx_validate_build "$SCRIPT_DIR"

echo "[clawhub] starting clawhub_skills bridge node..."
cargo run --manifest-path "$RUST_ROOT/Cargo.toml" -p robonix-cli -- "${RBNX_START_OPTS[@]}" \
  -p "$SCRIPT_DIR" -n com.robonix.skills.clawhub &
sleep 2

# ── 4. Executor (tool dispatch runtime) ──────────────────────────────────────
if [ "$START_EXECUTOR" = "1" ]; then
  echo "[clawhub] starting robonix-executor (:50061)..."
  cargo run --manifest-path "$RUST_ROOT/Cargo.toml" -p robonix-executor &
  sleep 1
fi

# ── 5. Pilot (VLM reasoning service) ─────────────────────────────────────────
if [ "$START_PILOT" = "1" ]; then
  echo "[clawhub] starting robonix-pilot (:50071)..."
  cargo run --manifest-path "$RUST_ROOT/Cargo.toml" -p robonix-pilot &
  sleep 2
fi

# ── 6. Liaison (user-facing gRPC server) ─────────────────────────────────────
if [ "$START_LIAISON" = "1" ]; then
  echo "[clawhub] starting robonix-liaison (:50081)..."
  cargo run --manifest-path "$RUST_ROOT/Cargo.toml" -p robonix-liaison &
  sleep 2
fi

# ── Ready ────────────────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════════════════"
echo "  Robonix stack ready with ClawHub Agent Skills"
echo ""
echo "  Atlas:    $ROBONIX_ATLAS"
echo "  Executor: $ROBONIX_EXECUTOR_ENDPOINT"
echo "  Pilot:    $ROBONIX_PILOT_ENDPOINT"
echo ""
echo "  Useful commands:"
echo "    rbnx nodes      — list registered nodes"
echo "    rbnx describe   — show imported skills"
echo "    rbnx tools      — list all agent tools"
echo "    rbnx graph -o topology.png"
echo "    rbnx chat       — interactive agent chat"
echo "════════════════════════════════════════════════════════════════"
echo ""

echo "[clawhub] stack ready — use 'rbnx chat' to connect (Ctrl+C to stop)."
wait