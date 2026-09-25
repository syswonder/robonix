#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Black-box Pilot test: a session survives a Pilot restart. Pilot is told a
# code word, stopped, and started again; asked in the same session, it must
# answer with the code word, restored from the session transcript on disk.
set -euo pipefail

ROOT="${ROBONIX_SOURCE_PATH:-$(cd "$(dirname "$0")/.." && pwd)}"
DEPLOY_DIR="${ROBONIX_TEST_DEPLOY_DIR:-$HOME/robot-agilex-ranger_mini_v3}"
ATLAS_ADDR="${RBNX_TEST_ATLAS_ADDR:-127.0.0.1:52151}"
EXECUTOR_ADDR="${RBNX_TEST_EXECUTOR_ADDR:-127.0.0.1:52161}"
PILOT_ADDR="${RBNX_TEST_PILOT_ADDR:-127.0.0.1:52171}"
ATLAS_BIN="${ROBONIX_ATLAS_BIN:-$(command -v robonix-atlas || true)}"
EXECUTOR_BIN="${ROBONIX_EXECUTOR_BIN:-$(command -v robonix-executor || true)}"
PILOT_BIN="${ROBONIX_PILOT_BIN:-$(command -v robonix-pilot || true)}"
PYTHON_BIN="${ROBONIX_TEST_PYTHON:-python3}"
WORK="$(mktemp -d /tmp/robonix-pilot-restore.XXXXXX)"
ATLAS_LOG="$WORK/atlas.out"
EXECUTOR_LOG="$WORK/executor.out"
PILOT_LOG="$WORK/pilot.out"
SESSION_ID="pilot-restore-$(date +%s)-$$"
CODE_WORD="7429"

cleanup() {
  local rc=$?
  for pid in "${PILOT_PID:-}" "${EXECUTOR_PID:-}" "${ATLAS_PID:-}"; do
    [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
  done
  for pid in "${PILOT_PID:-}" "${EXECUTOR_PID:-}" "${ATLAS_PID:-}"; do
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
  done
  if [[ $rc -ne 0 ]]; then
    echo "FAIL: artifacts retained at $WORK" >&2
    echo "--- pilot log tail ---" >&2
    tail -80 "$PILOT_LOG" >&2 || true
  elif [[ "${ROBONIX_TEST_KEEP_ARTIFACTS:-0}" == "1" ]]; then
    echo "artifacts retained at $WORK" >&2
  else
    rm -rf "$WORK"
  fi
  exit "$rc"
}
trap cleanup EXIT INT TERM

: "${ATLAS_BIN:?robonix-atlas is not on PATH; set ROBONIX_ATLAS_BIN}"
: "${EXECUTOR_BIN:?robonix-executor is not on PATH; set ROBONIX_EXECUTOR_BIN}"
: "${PILOT_BIN:?robonix-pilot is not on PATH; set ROBONIX_PILOT_BIN}"
if [[ ! -f "$DEPLOY_DIR/.env" ]]; then
  echo "missing $DEPLOY_DIR/.env" >&2
  exit 2
fi
set -a
# shellcheck disable=SC1090
source "$DEPLOY_DIR/.env"
set +a
: "${VLM_BASE_URL:?VLM_BASE_URL missing}"
: "${VLM_API_KEY:?VLM_API_KEY missing}"
: "${VLM_MODEL:?VLM_MODEL missing}"
export ROBONIX_VLM_UPSTREAM="$VLM_BASE_URL"
export ROBONIX_VLM_API_KEY="$VLM_API_KEY"
export ROBONIX_VLM_MODEL="$VLM_MODEL"
export ROBONIX_VLM_FORMAT=openai
export ROBONIX_SOURCE_PATH="$ROOT"
# Keep this run's logs and session state in its own directory.
export SCRIBE_LOG_DIR="$WORK/logs"
export ROBONIX_SESSION_DIR="$WORK/sessions"
export SCRIBE_STDOUT_LEVEL=warn
export SCRIBE_FILE_LEVEL=info

wait_port() {
  "$PYTHON_BIN" - "$1" <<'PY'
import socket, sys, time
host, port = sys.argv[1].rsplit(":", 1)
deadline = time.time() + 15
while time.time() < deadline:
    try:
        with socket.create_connection((host, int(port)), timeout=0.25):
            raise SystemExit(0)
    except OSError:
        time.sleep(0.1)
raise SystemExit(f"{sys.argv[1]} did not listen within 15s")
PY
}

start_pilot() {
  "$PILOT_BIN" \
    --atlas "$ATLAS_ADDR" --listen "$PILOT_ADDR" --id pilot \
    >>"$PILOT_LOG" 2>&1 &
  PILOT_PID=$!
  wait_port "$PILOT_ADDR"
}

"$ATLAS_BIN" \
  --listen "$ATLAS_ADDR" --capabilities "$ROOT/capabilities" \
  >"$ATLAS_LOG" 2>&1 &
ATLAS_PID=$!
wait_port "$ATLAS_ADDR"

"$EXECUTOR_BIN" \
  --atlas "$ATLAS_ADDR" --listen "$EXECUTOR_ADDR" --id executor \
  >"$EXECUTOR_LOG" 2>&1 &
EXECUTOR_PID=$!
wait_port "$EXECUTOR_ADDR"

start_pilot

PROTO_SOURCE="$(find "$ROOT/target" -path '*/out/robonix_contracts.proto' -print 2>/dev/null | head -1 | xargs -n1 dirname 2>/dev/null)"
if [[ -z "$PROTO_SOURCE" || ! -d "$PROTO_SOURCE" ]]; then
  echo "generated proto source not found; build Pilot first" >&2
  exit 2
fi
PROTO_GEN="$WORK/proto_gen"
mkdir -p "$PROTO_GEN"
cp "$PROTO_SOURCE"/*.proto "$PROTO_GEN/"
"$PYTHON_BIN" -m grpc_tools.protoc -I "$PROTO_GEN" \
  --python_out="$PROTO_GEN" --grpc_python_out="$PROTO_GEN" "$PROTO_GEN"/*.proto

# Send one task and print Pilot's final reply.
ask() {
  PYTHONPATH="$PROTO_GEN${PYTHONPATH:+:$PYTHONPATH}" "$PYTHON_BIN" - \
    "$PILOT_ADDR" "$SESSION_ID" "$1" <<'PY'
import json, sys, uuid
import grpc
import pilot_pb2
import robonix_contracts_pb2_grpc as contracts

endpoint, session_id, text = sys.argv[1:4]
stub = contracts.RobonixSystemPilotStub(grpc.insecure_channel(endpoint))
task = pilot_pb2.Task(
    task_id=str(uuid.uuid4()),
    session_id=session_id,
    text=text,
    context_json=json.dumps({"client": "pilot-restore-test", "interaction_mode": "task"}),
)
final = ""
for event in stub.SubmitTask(task, timeout=120):
    if event.final_text:
        final = event.final_text
print(final)
PY
}

FAILED=0
check() {
  if eval "$2"; then
    echo "PASS  $1"
  else
    echo "FAIL  $1"
    FAILED=1
  fi
}

FIRST="$(ask "Remember this code word: blue giraffe $CODE_WORD. Do not call any capability; reply only 'noted'.")"
echo "before restart: $FIRST"

kill -TERM "$PILOT_PID"
wait "$PILOT_PID" 2>/dev/null || true
start_pilot

SECOND="$(ask "What code word did I give you earlier? Do not call any capability; reply with the code word only.")"
echo "after restart: $SECOND"

TRANSCRIPT="$WORK/sessions/$SESSION_ID/transcript.jsonl"
echo
echo "=== PILOT SESSION RESTORE ASSERTIONS ==="
check "the session transcript was written" '[[ -s "$TRANSCRIPT" ]]'
check "the transcript starts with its session record" \
  'head -1 "$TRANSCRIPT" | grep -q "\"kind\":\"session\""'
check "the restarted Pilot restored the session from its transcript" \
  'grep -q "restored session '"'"'$SESSION_ID'"'"' from its transcript" "$WORK/logs/pilot.log"'
check "the restored session still knows the code word" '[[ "$SECOND" == *"$CODE_WORD"* ]]'
check "the second task was appended to the same transcript" \
  '[[ $(grep -c "User task (authoritative)" "$TRANSCRIPT") -eq 2 ]]'

echo
if [[ $FAILED -ne 0 ]]; then
  exit 1
fi
echo "PASS: Pilot session restored across a restart"
