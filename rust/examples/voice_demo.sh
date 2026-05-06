#!/usr/bin/env bash
# voice_demo.sh — end-to-end smoke test for the voice pipeline (mock mode).
#
# Boots:
#   1. robonix-atlas        (service registry / discovery)
#   2. mock_pilot           (SrvPilot stub that echoes Task.user_id + text)
#   3. robonix-liaison      (voice orchestrator, ROBONIX_LIAISON_VOICE_MOCK=1)
#
# Then runs:
#   4. voice_demo_client    (text path + voice path assertions)
#
# Requires: rust toolchain with `cargo run --release -p robonix-xxx`.
#
# Exit 0 = all checks passed; non-zero = failure.

set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

cleanup() {
    echo "[voice_demo] cleaning up…"
    [[ -n "$ATLAS_PID"   ]] && kill "$ATLAS_PID"   2>/dev/null || true
    [[ -n "$PILOT_PID"   ]] && kill "$PILOT_PID"   2>/dev/null || true
    [[ -n "$LIAISON_PID" ]] && kill "$LIAISON_PID" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT

echo "========================================="
echo " Robonix Voice Demo (mock mode)"
echo "========================================="
echo ""
echo "[voice_demo] building all required binaries (release)…"
cargo build --release -p robonix-atlas -p robonix-liaison --examples 2>&1 | tail -5

ATLAS_PORT=${ATLAS_PORT:-50051}
PILOT_PORT=${MOCK_PILOT_PORT:-50071}
LIAISON_PORT=${ROBONIX_LIAISON_PORT:-50081}

export ROBONIX_ATLAS="127.0.0.1:$ATLAS_PORT"
export ROBONIX_PILOT_ENDPOINT="http://127.0.0.1:$PILOT_PORT"
export ROBONIX_LIAISON_PORT="$LIAISON_PORT"
# Enable mock mode: Liaison skips real mic+ASR and uses a canned transcript.
export ROBONIX_LIAISON_VOICE_MOCK=1
export ROBONIX_LIAISON_VOICE_MOCK_TEXT="你好，请介绍一下你自己。"

# ── 1. Atlas ─────────────────────────────────────────────────────────────────
echo ""
echo "[voice_demo] starting Atlas on :$ATLAS_PORT …"
cargo run --release -p robonix-atlas -- --port "$ATLAS_PORT" &>/dev/null &
ATLAS_PID=$!
sleep 1

# ── 2. Mock Pilot ────────────────────────────────────────────────────────────
echo "[voice_demo] starting mock_pilot on :$PILOT_PORT …"
cargo run --release -p robonix-liaison --example mock_pilot &>/dev/null &
PILOT_PID=$!
sleep 1

# ── 3. Liaison ───────────────────────────────────────────────────────────────
echo "[voice_demo] starting Liaison on :$LIAISON_PORT (voice mock enabled)…"
cargo run --release -p robonix-liaison &>/dev/null &
LIAISON_PID=$!
sleep 2

# ── 4. Demo client ───────────────────────────────────────────────────────────
echo ""
echo "[voice_demo] running voice_demo_client…"
echo "-----------------------------------------"
if cargo run --release -p robonix-liaison --example voice_demo_client; then
    echo "-----------------------------------------"
    echo "[voice_demo] SUCCESS"
    exit 0
else
    echo "-----------------------------------------"
    echo "[voice_demo] FAILED (see output above)"
    exit 1
fi
