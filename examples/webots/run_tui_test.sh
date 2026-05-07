#!/usr/bin/env bash
# run_tui_test.sh — one-shot launcher for the voice + text TUI test stack.
#
# Reuses the dev stack (Atlas, Pilot, speech_service) and only starts the
# new test-only components:
#   1. mock_audio   — WAV-file-backed mic + speaker
#   2. Liaison      — bound to its own port to avoid colliding with any
#                     liaison the dev stack may already have started
#   3. rbnx chat    — the TUI client
#
# Prerequisites: Atlas + Pilot + speech_service already running (e.g. via
# examples/webots/run.sh).
#
# Usage:
#   examples/webots/run_tui_test.sh
#
# Optional env:
#   MOCK_WAV_INPUT=<path>  — custom voice-input WAV (default: synthesised
#                            from MOCK_WAV_TEXT below)
#   MOCK_WAV_TEXT=<utf8>   — text to feed an edge-tts pre-render when
#                            MOCK_WAV_INPUT is not set
#                            (default: "where am I right now")
#   MOCK_WAV_OUTPUT=<path> — TTS output file (default: /tmp/robonix_tts_output.wav)

set -euo pipefail

EXAMPLES_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUST_ROOT="$(cd "${EXAMPLES_ROOT}/../../rust" && pwd)"

cd "$EXAMPLES_ROOT"

_SAVED_START_PILOT="${START_PILOT:-}"
_SAVED_START_LIAISON="${START_LIAISON:-}"
if [ -f .env ]; then
  set -a; source .env; set +a
fi
export START_PILOT="${_SAVED_START_PILOT:-0}"
export START_LIAISON=""

export ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"
export ROBONIX_META_GRPC_ENDPOINT="${ROBONIX_META_GRPC_ENDPOINT:-$ROBONIX_ATLAS}"
export ROBONIX_ATLAS_ENDPOINT="${ROBONIX_ATLAS_ENDPOINT:-http://$ROBONIX_ATLAS}"
export ROBONIX_PILOT_ENDPOINT="${ROBONIX_PILOT_ENDPOINT:-http://127.0.0.1:50071}"
export ROBONIX_LIAISON_PORT="${ROBONIX_LIAISON_PORT:-50082}"
export RUST_LOG="${RUST_LOG:-robonix_atlas=info,robonix_liaison=info,mock_audio=info}"
export MOCK_AUDIO_PORT="${MOCK_AUDIO_PORT:-50091}"
export MOCK_WAV_TEXT="${MOCK_WAV_TEXT:-where am I right now}"
export MOCK_WAV_OUTPUT="${MOCK_WAV_OUTPUT:-/tmp/robonix_tts_output.wav}"
export ROBONIX_CHAT_VOICE_TTS="${ROBONIX_CHAT_VOICE_TTS:-1}"
export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"

# Pin the TUI to the test Liaison so it doesn't pick up an unrelated
# liaison registered with Atlas by some other test run.
export ROBONIX_LIAISON_ENDPOINT="http://127.0.0.1:${ROBONIX_LIAISON_PORT}"

# Pin mic / speaker discovery to mock_audio; everything else is auto-discovered via Atlas.
export ROBONIX_CHAT_MIC_NODE="com.robonix.demo.mock_audio"
export ROBONIX_CHAT_SPEAKER_NODE="com.robonix.demo.mock_audio"

rbnx() {
  (cd "$RUST_ROOT" && cargo run -p robonix-cli -- "$@")
}

# Drop any leftover processes from a previous run.
pkill -9 -u "$(id -u)" -f 'robonix-liaison' 2>/dev/null || true
pkill -9 -u "$(id -u)" -f 'mock_audio' 2>/dev/null || true
pkill -9 -u "$(id -u)" -f 'mock_pilot' 2>/dev/null || true
sleep 0.5

cleanup() {
  echo "[tui-test] shutting down..."
  kill $(jobs -p) 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

check_local_port() {
  local port=$1
  local name=$2
  if ss -tlnp 2>/dev/null | grep -q ":${port} "; then
    echo "[tui-test] ERROR: port $port ($name) is already in use."
    echo "[tui-test] Override: ROBONIX_LIAISON_PORT=50182 MOCK_AUDIO_PORT=50191 $0"
    exit 1
  fi
}
check_local_port "$ROBONIX_LIAISON_PORT" "Liaison"
check_local_port "$MOCK_AUDIO_PORT" "mock_audio"

ATLAS_PORT="${ROBONIX_ATLAS##*:}"
PILOT_PORT="${ROBONIX_PILOT_ENDPOINT##*:}"

echo "[tui-test] checking dev stack..."
if ! ss -tlnp 2>/dev/null | grep -q ":${ATLAS_PORT} "; then
  echo "[tui-test] ERROR: Atlas not running on :${ATLAS_PORT}"
  echo "[tui-test] Start the dev stack first: examples/webots/run.sh"
  exit 1
fi
echo "[tui-test]   Atlas:          :${ATLAS_PORT} ok"

if ss -tlnp 2>/dev/null | grep -q ":${PILOT_PORT} "; then
  echo "[tui-test]   Pilot:          :${PILOT_PORT} ok"
else
  echo "[tui-test]   Pilot:          :${PILOT_PORT} missing (Liaison will fail at SubmitTask)"
fi

if pgrep -f 'speech_service.service' >/dev/null 2>&1; then
  echo "[tui-test]   speech_service: running (real ASR + TTS)"
else
  echo "[tui-test]   speech_service: not running"
  echo "[tui-test]   WARNING: ASR/TTS will not work. Start speech_service via run.sh"
fi

if [ -z "${MOCK_WAV_INPUT:-}" ]; then
  MOCK_WAV_INPUT="/tmp/robonix_test_input.wav"
  echo "[tui-test] generating WAV: \"${MOCK_WAV_TEXT}\""
  python3 -c "
import asyncio, edge_tts, sys
async def main():
    c = edge_tts.Communicate(sys.argv[1], 'en-US-AriaNeural')
    await c.save('/tmp/_robonix_tts_tmp.mp3')
asyncio.run(main())
" "${MOCK_WAV_TEXT}" 2>&1
  ffmpeg -y -loglevel error -i /tmp/_robonix_tts_tmp.mp3 \
    -ar 16000 -ac 1 -sample_fmt s16 "${MOCK_WAV_INPUT}" 2>&1
  rm -f /tmp/_robonix_tts_tmp.mp3
  echo "[tui-test]   WAV: ${MOCK_WAV_INPUT}"
fi
export MOCK_WAV_INPUT

echo "[tui-test] starting mock_audio (:${MOCK_AUDIO_PORT})..."
(cd "$RUST_ROOT" && cargo run -p robonix-liaison --example mock_audio) &
sleep 2

echo "[tui-test] starting Liaison (:${ROBONIX_LIAISON_PORT})..."
(cd "$RUST_ROOT" && cargo run -p robonix-liaison) &
sleep 2

cat <<EOF

  TUI ready
    Enter   -> text -> Liaison -> Pilot
    Ctrl+V  -> WAV -> ASR(gRPC) -> Pilot -> TTS(gRPC) -> audio file
    Esc     -> abort turn   Ctrl+C -> exit

    input  WAV:  ${MOCK_WAV_INPUT}
    output WAV:  ${MOCK_WAV_OUTPUT}

EOF

rbnx chat --server "$ROBONIX_ATLAS"
