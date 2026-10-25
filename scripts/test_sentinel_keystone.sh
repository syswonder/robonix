#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Exercise Keystone, Liaison, Pilot, Sentinel, and Executor on localhost.
set -euo pipefail

ROOT="${ROBONIX_SOURCE_PATH:-$(cd "$(dirname "$0")/.." && pwd)}"
ATLAS_ADDR="${RBNX_TEST_ATLAS_ADDR:-127.0.0.1:52051}"
EXECUTOR_ADDR="${RBNX_TEST_EXECUTOR_ADDR:-127.0.0.1:52061}"
SENTINEL_ADDR="${RBNX_TEST_SENTINEL_ADDR:-127.0.0.1:52062}"
KEYSTONE_ADDR="${RBNX_TEST_KEYSTONE_ADDR:-127.0.0.1:52095}"
PILOT_ADDR="${RBNX_TEST_PILOT_ADDR:-127.0.0.1:52071}"
LIAISON_ADDR="${RBNX_TEST_LIAISON_ADDR:-127.0.0.1:52081}"
VLM_ADDR="${RBNX_TEST_VLM_ADDR:-127.0.0.1:52091}"
TEST_PYTHON="${RBNX_TEST_PYTHON:-python3}"
ATLAS_BIN="${ROBONIX_ATLAS_BIN:-$ROOT/target/debug/robonix-atlas}"
KEYSTONE_BIN="${ROBONIX_KEYSTONE_BIN:-$ROOT/target/debug/robonix-keystone}"
EXECUTOR_BIN="${ROBONIX_EXECUTOR_BIN:-$ROOT/target/debug/robonix-executor}"
PILOT_BIN="${ROBONIX_PILOT_BIN:-$ROOT/target/debug/robonix-pilot}"
LIAISON_BIN="${ROBONIX_LIAISON_BIN:-$ROOT/target/debug/robonix-liaison}"
RBNX_BIN="${RBNX_BIN:-$ROOT/target/debug/rbnx}"
WORK="$(mktemp -d /tmp/robonix-sentinel-keystone.XXXXXX)"
ATLAS_LOG="$WORK/atlas.log"
KEYSTONE_LOG="$WORK/keystone.log"
EXECUTOR_LOG="$WORK/executor.log"
PILOT_LOG="$WORK/pilot.log"
LIAISON_LOG="$WORK/liaison.log"
VLM_LOG="$WORK/vlm.log"
BOOTSTRAP_FILE="$WORK/bootstrap-admin.txt"
OPERATOR_PASSWORD_FILE="$WORK/operator-password.txt"
ADMIN_TOKEN_FILE="$WORK/admin-token.txt"

cleanup() {
  local rc=$1
  trap - EXIT INT TERM
  for pid in "${LIAISON_PID:-}" "${PILOT_PID:-}" "${VLM_PID:-}" "${EXECUTOR_PID:-}" "${KEYSTONE_PID:-}" "${ATLAS_PID:-}"; do
    if [[ -n "$pid" ]]; then kill "$pid" 2>/dev/null || true; fi
  done
  for pid in "${LIAISON_PID:-}" "${PILOT_PID:-}" "${VLM_PID:-}" "${EXECUTOR_PID:-}" "${KEYSTONE_PID:-}" "${ATLAS_PID:-}"; do
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
  rm -f "$BOOTSTRAP_FILE"
  if [[ $rc -ne 0 ]]; then
    echo "FAIL: component logs follow" >&2
    for log in "$ATLAS_LOG" "$KEYSTONE_LOG" "$EXECUTOR_LOG" "$PILOT_LOG" "$LIAISON_LOG" "$VLM_LOG"; do
      echo "--- $log ---" >&2
      tail -80 "$log" >&2 || true
    done
  fi
  rm -rf "$WORK"
  exit "$rc"
}
trap 'cleanup $?' EXIT
trap 'cleanup 130' INT
trap 'cleanup 143' TERM

if ! command -v "$TEST_PYTHON" >/dev/null 2>&1; then
  echo "Python executable not found: $TEST_PYTHON" >&2
  exit 2
fi
for binary in "$ATLAS_BIN" "$KEYSTONE_BIN" "$EXECUTOR_BIN" "$PILOT_BIN" "$LIAISON_BIN" "$RBNX_BIN"; do
  if [[ ! -x "$binary" ]]; then
    echo "missing executable: $binary" >&2
    echo "build with: cargo build -p robonix-atlas -p robonix-keystone -p robonix-executor -p robonix-pilot -p robonix-liaison -p robonix-cli" >&2
    exit 2
  fi
done

if ! "$TEST_PYTHON" -c 'import grpc, grpc_tools' >/dev/null 2>&1; then
  echo "$TEST_PYTHON needs grpcio and grpcio-tools" >&2
  exit 2
fi

wait_for_port() {
  local address=$1
  local pid=$2
  "$TEST_PYTHON" - "$address" <<'PY'
import socket
import sys
import time

host, port = sys.argv[1].rsplit(":", 1)
deadline = time.monotonic() + 10
while time.monotonic() < deadline:
    try:
        with socket.create_connection((host, int(port)), timeout=0.25):
            raise SystemExit(0)
    except OSError:
        time.sleep(0.1)
raise SystemExit(f"{sys.argv[1]} did not listen within 10 seconds")
PY
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "process $pid exited while waiting for $address" >&2
    return 1
  fi
}

cd "$ROOT"
export ROBONIX_SOURCE_PATH="$ROOT"
export ROBONIX_DATA_DIR="$WORK/data"
export SCRIBE_STDOUT_LEVEL=warn
export SCRIBE_FILE_LEVEL=debug
export ROBONIX_SESSION_DIR="$WORK/sessions"

"$ATLAS_BIN" \
  --listen "$ATLAS_ADDR" \
  --capabilities "$ROOT/capabilities" \
  >"$ATLAS_LOG" 2>&1 &
ATLAS_PID=$!
wait_for_port "$ATLAS_ADDR" "$ATLAS_PID"

"$KEYSTONE_BIN" \
  --atlas "$ATLAS_ADDR" \
  --listen "$KEYSTONE_ADDR" \
  --database "$WORK/keystone.db" \
  --bootstrap-credentials-file "$BOOTSTRAP_FILE" \
  >"$KEYSTONE_LOG" 2>&1 &
KEYSTONE_PID=$!
wait_for_port "$KEYSTONE_ADDR" "$KEYSTONE_PID"

"$EXECUTOR_BIN" \
  --atlas "$ATLAS_ADDR" \
  --listen "$EXECUTOR_ADDR" \
  --sentinel-listen "$SENTINEL_ADDR" \
  --sentinel-rules "$WORK/sentinel-rules.json" \
  --require-keystone true \
  --id executor \
  >"$EXECUTOR_LOG" 2>&1 &
EXECUTOR_PID=$!
wait_for_port "$EXECUTOR_ADDR" "$EXECUTOR_PID"
wait_for_port "$SENTINEL_ADDR" "$EXECUTOR_PID"

cat >"$WORK/vlm.py" <<'PY'
import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class Handler(BaseHTTPRequestHandler):
    planning_round = {"admin": 0, "operator": 0}

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        if self.path != "/v1/chat/completions":
            self.send_error(404)
            return
        messages = json.dumps(body.get("messages", []))
        scenario = "admin" if "admin test" in messages else "operator"
        if body.get("response_format"):
            Handler.planning_round[scenario] += 1
            if Handler.planning_round[scenario] == 1:
                reply = {
                    "content": "",
                    "rtdl_description": "list the test workspace",
                    "rtdl": {
                        "op": "do",
                        "op_id": 0,
                        "description": "list the test workspace",
                        "cap": "executor.builtin_list_dir",
                        "args": {"path": "."},
                    },
                    "task_update": None,
                }
            else:
                reply = {
                    "content": (
                        "The test workspace was listed."
                        if scenario == "admin" else "The request was denied."
                    ),
                    "rtdl_description": "",
                    "rtdl": {"op": "sequence", "op_id": 0, "description": "", "children": []},
                    "task_update": None,
                }
            content = json.dumps(reply)
        else:
            content = "The test workspace was listed."
        payload = json.dumps({"choices": [{"delta": {"content": content}, "finish_reason": "stop"}]})
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.end_headers()
        self.wfile.write(f"data: {payload}\n\ndata: [DONE]\n\n".encode())
        self.wfile.flush()

    def log_message(self, format, *args):
        print(format % args, file=sys.stderr, flush=True)


host, port = sys.argv[1].rsplit(":", 1)
ThreadingHTTPServer((host, int(port)), Handler).serve_forever()
PY
"$TEST_PYTHON" -u "$WORK/vlm.py" "$VLM_ADDR" >"$VLM_LOG" 2>&1 &
VLM_PID=$!
wait_for_port "$VLM_ADDR" "$VLM_PID"

"$PILOT_BIN" \
  --atlas "$ATLAS_ADDR" \
  --listen "$PILOT_ADDR" \
  --vlm-upstream "http://$VLM_ADDR/v1" \
  --vlm-api-key test-only \
  --vlm-model test-model \
  --vlm-context-window-tokens 8192 \
  >"$PILOT_LOG" 2>&1 &
PILOT_PID=$!
wait_for_port "$PILOT_ADDR" "$PILOT_PID"

"$LIAISON_BIN" \
  --atlas "$ATLAS_ADDR" \
  --listen "$LIAISON_ADDR" \
  --pilot-endpoint "$PILOT_ADDR" \
  --keystone-endpoint "$KEYSTONE_ADDR" \
  >"$LIAISON_LOG" 2>&1 &
LIAISON_PID=$!
wait_for_port "$LIAISON_ADDR" "$LIAISON_PID"

PROTO_SOURCE="$(find "$ROOT/target/debug/build" \
  -path '*/robonix-executor-*/out/robonix_contracts.proto' \
  -printf '%T@ %h\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"
if [[ -z "$PROTO_SOURCE" || ! -d "$PROTO_SOURCE" ]]; then
  echo "Executor-generated protobuf sources were not found; build Executor first" >&2
  exit 2
fi
PROTO_GEN="$WORK/proto_gen"
mkdir -p "$PROTO_GEN"
cp "$PROTO_SOURCE"/*.proto "$PROTO_GEN/"
"$TEST_PYTHON" -m grpc_tools.protoc \
  -I "$PROTO_GEN" \
  --python_out="$PROTO_GEN" \
  --grpc_python_out="$PROTO_GEN" \
  "$PROTO_GEN"/*.proto

PYTHONPATH="$PROTO_GEN${PYTHONPATH:+:$PYTHONPATH}" "$TEST_PYTHON" - \
  "$KEYSTONE_ADDR" "$SENTINEL_ADDR" "$EXECUTOR_ADDR" "$BOOTSTRAP_FILE" \
  "$OPERATOR_PASSWORD_FILE" "$ADMIN_TOKEN_FILE" "$LIAISON_ADDR" <<'PY'
import json
import os
import pathlib
import secrets
import sys

import grpc
import keystone_pb2
import liaison_pb2
import pilot_pb2
import robonix_contracts_pb2_grpc as contracts
import sentinel_pb2

if not __debug__:
    raise SystemExit("the test requires Python assertions")

keystone_endpoint, sentinel_endpoint, executor_endpoint, bootstrap_file, operator_password_file, admin_token_file, liaison_endpoint = sys.argv[1:]
bootstrap_path = pathlib.Path(bootstrap_file)
credentials = dict(
    line.split("=", 1)
    for line in bootstrap_path.read_text().splitlines()
    if "=" in line
)
password = credentials["password"]
operator_password = secrets.token_urlsafe(24)
operator_password_path = pathlib.Path(operator_password_file)
operator_password_path.write_text(operator_password + "\n")
os.chmod(operator_password_path, 0o600)


def expect_rpc_error(expected, operation, failure):
    try:
        operation()
    except grpc.RpcError as error:
        assert error.code() == expected, error
        return
    raise AssertionError(failure)

with grpc.insecure_channel(keystone_endpoint) as channel:
    login = contracts.RobonixSystemKeystoneLoginStub(channel)
    register = contracts.RobonixSystemKeystoneRegisterStub(channel)
    get_profile = contracts.RobonixSystemKeystoneGetProfileStub(channel)
    change_password = contracts.RobonixSystemKeystoneChangePasswordStub(channel)
    response = login.Login(
        keystone_pb2.Login_Request(username="admin", password=password), timeout=5
    )
    assert response.user.password_change_required, response.user
    token = response.session_token
    expect_rpc_error(
        grpc.StatusCode.UNAUTHENTICATED,
        lambda: login.Login(
            keystone_pb2.Login_Request(
                username="admin' OR 1=1 --", password=password
            ),
            timeout=5,
        ),
        "Keystone accepted SQL injection in the username",
    )
    expect_rpc_error(
        grpc.StatusCode.FAILED_PRECONDITION,
        lambda: get_profile.GetProfile(
            keystone_pb2.GetProfile_Request(session_token=token), timeout=5
        ),
        "Keystone allowed a session that requires a password change",
    )
    new_admin_password = secrets.token_urlsafe(24)
    change_password.ChangePassword(
        keystone_pb2.ChangePassword_Request(
            session_token=token,
            current_password=password,
            new_password=new_admin_password,
        ),
        timeout=5,
    )
    assert not bootstrap_path.exists(), "bootstrap credentials file was not removed"
    profile = get_profile.GetProfile(
        keystone_pb2.GetProfile_Request(session_token=token), timeout=5
    )
    assert profile.user.username == "admin", profile.user
    user_response = register.Register(
        keystone_pb2.Register_Request(
            username="operator",
            display_name="Test Operator",
            email="operator@example.invalid",
            password=operator_password,
        ),
        timeout=5,
    )

assert token, "Keystone returned an empty session token"
assert response.user.username == "admin", response.user
assert "admin" in response.user.roles, response.user
user_token = user_response.session_token
assert user_token, "Keystone returned an empty user session token"
assert "user" in user_response.user.roles, user_response.user
print("PASS Keystone authenticated the bootstrap administrator")
print("PASS Keystone registered and authenticated a normal user")
admin_token_path = pathlib.Path(admin_token_file)
admin_token_path.write_text(token + "\n")
os.chmod(admin_token_path, 0o600)

with grpc.insecure_channel(liaison_endpoint) as channel:
    submit = contracts.RobonixSystemLiaisonSubmitStub(channel)
    set_handsfree = contracts.RobonixSystemLiaisonHandsfreeSetEnabledStub(channel)
    for enabled in (True, False):
        expect_rpc_error(
            grpc.StatusCode.UNAUTHENTICATED,
            lambda enabled=enabled: set_handsfree.SetHandsfree(
                liaison_pb2.SetHandsfree_Request(enabled=enabled), timeout=5
            ),
            f"Liaison accepted unauthenticated hands-free enabled={enabled}",
        )
    disabled = set_handsfree.SetHandsfree(
        liaison_pb2.SetHandsfree_Request(enabled=False, session_token=token), timeout=5
    )
    assert disabled.ok and not disabled.enabled, disabled
    for bad_token in ("", "invalid-session"):
        expect_rpc_error(
            grpc.StatusCode.UNAUTHENTICATED,
            lambda bad_token=bad_token: list(submit.SubmitTask(
                pilot_pb2.Task(
                    task_id="unauthenticated-test",
                    session_id="unauthenticated-test",
                    text="list the test workspace",
                    context_json=json.dumps({"session_token": bad_token}),
                ),
                timeout=5,
            )),
            "Liaison forwarded an unauthenticated task to Pilot",
        )
print("PASS Liaison protected text submission and both hands-free transitions")

with grpc.insecure_channel(sentinel_endpoint) as channel:
    list_rules = contracts.RobonixSystemSentinelListRulesStub(channel)
    response = list_rules.ListRules(
        sentinel_pb2.ListRules_Request(session_token=token), timeout=5
    )
    assert json.loads(response.rules_json) == [], response.rules_json
    expect_rpc_error(
        grpc.StatusCode.UNAUTHENTICATED,
        lambda: list_rules.ListRules(
            sentinel_pb2.ListRules_Request(session_token="invalid-session"), timeout=5
        ),
        "Sentinel accepted an invalid Keystone session",
    )
print("PASS Sentinel resolved the administrator through Keystone")

def plan(plan_id, session_token):
    return pilot_pb2.Plan(
        plan_id=plan_id,
        session_id="sentinel-keystone-smoke",
        nodes=[
            pilot_pb2.RtdlNode(
                node_kind=2,
                op_id="list-workspace",
                description="list the test workspace",
                call=pilot_pb2.CapabilityCall(
                    call_id=f"{plan_id}:list-workspace",
                    provider_id="executor",
                    contract_id="robonix/system/executor/builtin/list_dir",
                    args_json=json.dumps({"path": "."}),
                ),
            )
        ],
        root_index=0,
        auth_session_token=session_token,
    )


def run_plan(execute, plan_id, session_token):
    return list(execute.Execute(plan(plan_id, session_token), timeout=10))


def terminal_state(events, state):
    matches = [
        event.node_state
        for event in events
        if event.HasField("node_state") and event.node_state.state == state
    ]
    assert len(matches) == 1, events
    return matches[0]

with grpc.insecure_channel(executor_endpoint) as channel:
    execute = contracts.RobonixSystemExecutorExecuteStub(channel)
    events = run_plan(execute, "empty-policy-admin", token)
    terminal = terminal_state(events, 2)
    assert terminal.leaf_result.success, terminal.leaf_result
    assert any(
        event.HasField("plan_complete") and not event.plan_complete.any_failed
        for event in events
    ), events

    expect_rpc_error(
        grpc.StatusCode.UNAUTHENTICATED,
        lambda: run_plan(execute, "empty-policy-unauthenticated", ""),
        "Executor accepted a plan without a Keystone session",
    )
    expect_rpc_error(
        grpc.StatusCode.UNAUTHENTICATED,
        lambda: run_plan(execute, "empty-policy-invalid-session", "invalid-session"),
        "Executor accepted an invalid Keystone session",
    )

print("PASS Executor authenticated the plan and Sentinel allowed the empty-policy call")

policy = [
    {
        "id": "allow-admin-list-dir",
        "effect": "allow",
        "priority": 100,
        "conditions": {
            "contract": "robonix/system/executor/builtin/list_dir",
            "roles": ["admin"],
        },
        "reason": "Administrators may list the test workspace",
    },
    {
        "id": "deny-non-admin-list-dir",
        "effect": "deny",
        "priority": 10,
        "conditions": {
            "contract": "robonix/system/executor/builtin/list_dir",
        },
        "reason": "Listing the test workspace requires the administrator role",
    },
]
policy_json = json.dumps(policy)
policy_ids = [rule["id"] for rule in policy]

with grpc.insecure_channel(sentinel_endpoint) as channel:
    list_rules = contracts.RobonixSystemSentinelListRulesStub(channel)
    replace_rules = contracts.RobonixSystemSentinelReplaceRulesStub(channel)

    for operation in (
        lambda: list_rules.ListRules(
            sentinel_pb2.ListRules_Request(session_token=user_token), timeout=5
        ),
        lambda: replace_rules.ReplaceRules(
            sentinel_pb2.ReplaceRules_Request(
                session_token=user_token, rules_json=policy_json
            ),
            timeout=5,
        ),
    ):
        expect_rpc_error(
            grpc.StatusCode.PERMISSION_DENIED,
            operation,
            "Sentinel allowed a normal user to manage rules",
        )

    response = replace_rules.ReplaceRules(
        sentinel_pb2.ReplaceRules_Request(
            session_token=token, rules_json=policy_json
        ),
        timeout=5,
    )
    active = json.loads(response.rules_json)
    assert [rule["id"] for rule in active] == policy_ids, active

    invalid_policy = json.dumps(
        [
            {"id": "duplicate", "effect": "deny"},
            {"id": "duplicate", "effect": "allow"},
        ]
    )
    expect_rpc_error(
        grpc.StatusCode.INVALID_ARGUMENT,
        lambda: replace_rules.ReplaceRules(
            sentinel_pb2.ReplaceRules_Request(
                session_token=token, rules_json=invalid_policy
            ),
            timeout=5,
        ),
        "Sentinel accepted duplicate rule IDs",
    )

    response = list_rules.ListRules(
        sentinel_pb2.ListRules_Request(session_token=token), timeout=5
    )
    active = json.loads(response.rules_json)
    assert [rule["id"] for rule in active] == policy_ids, active

print("PASS Sentinel restricted rule management to Keystone administrators")
print("PASS Sentinel kept the active rules after rejecting an invalid replacement")

with grpc.insecure_channel(executor_endpoint) as channel:
    execute = contracts.RobonixSystemExecutorExecuteStub(channel)

    admin_events = run_plan(execute, "role-policy-admin", token)
    admin_terminal = terminal_state(admin_events, 2)
    assert admin_terminal.leaf_result.success, admin_terminal.leaf_result

    user_events = run_plan(execute, "role-policy-user", user_token)
    user_terminal = terminal_state(user_events, 3)
    assert not user_terminal.leaf_result.success, user_terminal.leaf_result
    assert "deny-non-admin-list-dir" in user_terminal.leaf_result.error, (
        user_terminal.leaf_result
    )
    assert any(
        event.HasField("plan_complete") and event.plan_complete.any_failed
        for event in user_events
    ), user_events

print("PASS Sentinel allowed the administrator and denied the normal user")
PY

for database_file in "$WORK/keystone.db" "$WORK/keystone.db-wal" "$WORK/keystone.db-shm"; do
  if [[ -e "$database_file" && "$(stat -c '%a' "$database_file")" != "600" ]]; then
    echo "Keystone database file is not private: $database_file" >&2
    exit 1
  fi
done

IFS= read -r OPERATOR_PASSWORD < "$OPERATOR_PASSWORD_FILE"
rm -f "$OPERATOR_PASSWORD_FILE"
if ROBONIX_HOME="$WORK/rbnx-home" "$RBNX_BIN" ask \
  "list the test workspace" --server "$ATLAS_ADDR" --json \
  >"$WORK/unauthenticated-ask.out" 2>&1; then
  echo "rbnx ask accepted a request without a Keystone login" >&2
  exit 1
fi
printf '%s\n' "$OPERATOR_PASSWORD" | \
  ROBONIX_HOME="$WORK/rbnx-home" "$RBNX_BIN" login operator --server "$ATLAS_ADDR"
SESSION_FILE="$WORK/rbnx-home/session.yaml"
if [[ ! -f "$SESSION_FILE" || "$(stat -c '%a' "$SESSION_FILE")" != "600" ]]; then
  echo "rbnx login did not create a user-only session file" >&2
  exit 1
fi
IFS= read -r ADMIN_TOKEN < "$ADMIN_TOKEN_FILE"
rm -f "$ADMIN_TOKEN_FILE"
ROBONIX_HOME="$WORK/rbnx-home" ROBONIX_SESSION_TOKEN="$ADMIN_TOKEN" \
  timeout 45 "$RBNX_BIN" ask \
  "admin test: list the test workspace" --server "$ATLAS_ADDR" --json \
  >"$WORK/admin-ask.jsonl"
unset ADMIN_TOKEN
ROBONIX_HOME="$WORK/rbnx-home" timeout 45 "$RBNX_BIN" ask \
  "operator test: list the test workspace" --server "$ATLAS_ADDR" --json \
  >"$WORK/operator-ask.jsonl"
"$TEST_PYTHON" - "$WORK/admin-ask.jsonl" "$WORK/operator-ask.jsonl" <<'PY'
import json
import pathlib
import sys

for path, allowed in ((sys.argv[1], True), (sys.argv[2], False)):
    events = [json.loads(line) for line in pathlib.Path(path).read_text().splitlines()]
    results = [
        result["leaf_result"]
        for event in events
        for result in (event.get("batch_result") or {}).get("results", [])
        if result.get("leaf_result")
    ]
    assert any(
        result["contract_id"] == "robonix/system/executor/builtin/list_dir"
        and result["success"] == allowed
        and (allowed or "deny-non-admin-list-dir" in result["error"])
        for result in results
    ), f"unexpected Executor result for {path}: {results}"
    assert any(event.get("final_text") for event in events), path
PY
echo "PASS rbnx ask carried administrator and user identities through Liaison, Pilot, Executor, and Sentinel"
ROBONIX_HOME="$WORK/rbnx-home" "$RBNX_BIN" logout --server "$ATLAS_ADDR"
if [[ -e "$SESSION_FILE" ]]; then
  echo "rbnx logout did not remove the saved session" >&2
  exit 1
fi
echo "PASS rbnx logged in through Keystone and revoked the saved session"

echo "PASS Keystone, Liaison, Pilot, Sentinel, and Executor are connected"
