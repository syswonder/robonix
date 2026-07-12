#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Real Executor integration test: concurrent long-running RTDL plans,
# boundary stop, targeted cancel, and isolation of unrelated work.
set -euo pipefail

ROOT="${ROBONIX_SOURCE_PATH:-$(cd "$(dirname "$0")/.." && pwd)}"
ATLAS_ADDR="${RBNX_TEST_ATLAS_ADDR:-127.0.0.1:51051}"
EXECUTOR_ADDR="${RBNX_TEST_EXECUTOR_ADDR:-127.0.0.1:51061}"
WORK="$(mktemp -d /tmp/robonix-rtdl-isolation.XXXXXX)"
TRACE="$WORK/timeline.log"
ATLAS_LOG="$WORK/atlas.log"
EXECUTOR_LOG="$WORK/executor.log"

cleanup() {
  local rc=$?
  if [[ -n "${EXECUTOR_PID:-}" ]]; then kill "$EXECUTOR_PID" 2>/dev/null || true; fi
  if [[ -n "${ATLAS_PID:-}" ]]; then kill "$ATLAS_PID" 2>/dev/null || true; fi
  wait "${EXECUTOR_PID:-}" 2>/dev/null || true
  wait "${ATLAS_PID:-}" 2>/dev/null || true
  if [[ $rc -ne 0 ]]; then
    echo "FAIL: artifacts retained at $WORK" >&2
    echo "--- executor log tail ---" >&2
    tail -80 "$EXECUTOR_LOG" >&2 || true
  else
    rm -rf "$WORK"
  fi
  exit "$rc"
}
trap cleanup EXIT INT TERM

export ROBONIX_SOURCE_PATH="$ROOT"
export SCRIBE_STDOUT_LEVEL=warn
export SCRIBE_FILE_LEVEL=debug

"$HOME/.cargo/bin/robonix-atlas" \
  --listen "$ATLAS_ADDR" \
  --capabilities "$ROOT/capabilities" \
  >"$ATLAS_LOG" 2>&1 &
ATLAS_PID=$!

python3 - "$ATLAS_ADDR" <<'PY'
import socket, sys, time
host, port = sys.argv[1].rsplit(":", 1)
deadline = time.time() + 10
while time.time() < deadline:
    try:
        with socket.create_connection((host, int(port)), timeout=0.25):
            raise SystemExit(0)
    except OSError:
        time.sleep(0.1)
raise SystemExit("atlas did not listen within 10s")
PY

"$HOME/.cargo/bin/robonix-executor" \
  --atlas "$ATLAS_ADDR" \
  --listen "$EXECUTOR_ADDR" \
  --id executor \
  >"$EXECUTOR_LOG" 2>&1 &
EXECUTOR_PID=$!

python3 - "$EXECUTOR_ADDR" <<'PY'
import socket, sys, time
host, port = sys.argv[1].rsplit(":", 1)
deadline = time.time() + 10
while time.time() < deadline:
    try:
        with socket.create_connection((host, int(port)), timeout=0.25):
            raise SystemExit(0)
    except OSError:
        time.sleep(0.1)
raise SystemExit("executor did not listen within 10s")
PY

PROTO_GEN="$(find "$ROOT" -path '*/rbnx-build/codegen/proto_gen/robonix_contracts_pb2_grpc.py' -print -quit | xargs dirname)"
if [[ -z "$PROTO_GEN" || ! -d "$PROTO_GEN" ]]; then
  echo "generated Python gRPC stubs not found; build one Robonix package first" >&2
  exit 2
fi

PYTHONPATH="$PROTO_GEN${PYTHONPATH:+:$PYTHONPATH}" python3 - "$EXECUTOR_ADDR" "$TRACE" <<'PY'
import asyncio
import json
import pathlib
import shlex
import sys
import time

import grpc
import pilot_pb2
import robonix_contracts_pb2_grpc as contracts

endpoint, trace_arg = sys.argv[1:3]
trace = pathlib.Path(trace_arg)
started = time.monotonic()
session_id = "bash-cancel-isolation"


def rel() -> float:
    return time.monotonic() - started


def shell_mark(label: str, sleep_s: int, end: str | None = None) -> str:
    target = shlex.quote(str(trace))
    command = f'printf "%.3f {label}\\n" "$(date +%s.%N)" >> {target}; sleep {sleep_s}'
    if end:
        command += f'; printf "%.3f {end}\\n" "$(date +%s.%N)" >> {target}'
    return command


def bash_call(plan_id: str, call_id: str, op_id: str, description: str, command: str):
    return pilot_pb2.RtdlNode(
        node_kind=2,
        op_id=op_id,
        description=description,
        call=pilot_pb2.CapabilityCall(
            call_id=f"{plan_id}:{call_id}",
            provider_id="executor",
            contract_id="robonix/system/executor/builtin/run_command",
            args_json=json.dumps({"command": command}),
        ),
    )


def control_plan(plan_id: str, op: str, args: dict):
    return pilot_pb2.Plan(
        plan_id=plan_id,
        session_id=session_id,
        nodes=[pilot_pb2.RtdlNode(
            node_kind=2,
            op_id=f"{plan_id}-op",
            description=f"{op} target plan",
            call=pilot_pb2.CapabilityCall(
                call_id=f"{plan_id}:0",
                provider_id="executor",
                contract_id=f"robonix/system/executor/builtin/{op}",
                args_json=json.dumps(args),
            ),
        )],
        root_index=0,
    )


plan_a = pilot_pb2.Plan(
    plan_id="A-route",
    session_id=session_id,
    nodes=[
        pilot_pb2.RtdlNode(
            node_kind=0,
            children=[1, 2],
            op_id="A-sequence",
            description="restaurant then meeting room",
        ),
        bash_call("A-route", "1", "A-restaurant", "move to restaurant",
                  shell_mark("A_RESTAURANT_START", 8, "A_RESTAURANT_END")),
        bash_call("A-route", "2", "A-meeting", "move to meeting room",
                  shell_mark("A_MEETING_START", 20, "A_MEETING_END")),
    ],
    root_index=0,
)

greet_target = shlex.quote(str(trace))
greet_cmd = (
    f'printf "%.3f B_GREET_START\\n" "$(date +%s.%N)" >> {greet_target}; '
    f'for i in $(seq 1 60); do printf "%.3f B_GREET_TICK_%02d\\n" '
    f'"$(date +%s.%N)" "$i" >> {greet_target}; sleep 1; done; '
    f'printf "%.3f B_GREET_END\\n" "$(date +%s.%N)" >> {greet_target}'
)
plan_b = pilot_pb2.Plan(
    plan_id="B-greet",
    session_id=session_id,
    nodes=[bash_call("B-greet", "0", "B-watch", "continuous greet watch", greet_cmd)],
    root_index=0,
)
plan_c = pilot_pb2.Plan(
    plan_id="C-open-area",
    session_id=session_id,
    nodes=[bash_call("C-open-area", "0", "C-drive", "move to open area",
                     shell_mark("C_OPEN_START", 30, "C_OPEN_END"))],
    root_index=0,
)
plan_d = pilot_pb2.Plan(
    plan_id="D-return-315",
    session_id=session_id,
    nodes=[bash_call("D-return-315", "0", "D-return", "return to office 315",
                     shell_mark("D_315_START", 10, "D_315_END"))],
    root_index=0,
)

events: dict[str, list] = {}


async def drive(stub, plan):
    rows = []
    print(f"[{rel():6.2f}s] SUBMIT {plan.plan_id}", flush=True)
    async for event in stub.Execute(plan):
        if event.HasField("node_state"):
            ns = event.node_state
            leaf = ns.leaf_result
            rows.append((ns.op_id, ns.state, bool(leaf.success), leaf.error))
            print(
                f"[{rel():6.2f}s] {plan.plan_id:<15} op={ns.op_id:<18} "
                f"state={ns.state} success={bool(leaf.success) if ns.HasField('leaf_result') else '-'} "
                f"error={leaf.error!r}",
                flush=True,
            )
        elif event.HasField("plan_complete"):
            print(
                f"[{rel():6.2f}s] COMPLETE {plan.plan_id} any_failed={event.plan_complete.any_failed}",
                flush=True,
            )
    events[plan.plan_id] = rows
    return rows


async def main():
    async with grpc.aio.insecure_channel(endpoint) as channel:
        stub = contracts.RobonixSystemExecutorExecuteStub(channel)
        a = asyncio.create_task(drive(stub, plan_a))
        b = asyncio.create_task(drive(stub, plan_b))

        await asyncio.sleep(5)
        stop_a = control_plan(
            "ctl-stop-A",
            "stop_plan_at",
            {"plan_id": "A-route", "op_id": "A-meeting", "when": "on_enter"},
        )
        await drive(stub, stop_a)

        await asyncio.sleep(max(0, 12 - rel()))
        c = asyncio.create_task(drive(stub, plan_c))

        await asyncio.sleep(5)
        cancel_c = control_plan(
            "ctl-cancel-C", "cancel_plan", {"plan_id": "C-open-area", "wait_ms": 5000}
        )
        await drive(stub, cancel_c)
        d = asyncio.create_task(drive(stub, plan_d))

        await asyncio.gather(a, c, d)
        await b


asyncio.run(main())

lines = trace.read_text().splitlines() if trace.exists() else []
labels = [line.split(maxsplit=1)[1] for line in lines]
ticks = [label for label in labels if label.startswith("B_GREET_TICK_")]

checks = {
    "A restaurant completed": "A_RESTAURANT_END" in labels,
    "A meeting never started": "A_MEETING_START" not in labels,
    "B greet completed 60 heartbeats": len(ticks) == 60 and "B_GREET_END" in labels,
    "C open-area started": "C_OPEN_START" in labels,
    "C open-area was canceled before end": "C_OPEN_END" not in labels,
    "D return 315 completed": "D_315_END" in labels,
}

def terminal_state(plan_id: str, op_id: str) -> int | None:
    matches = [state for op, state, _ok, _err in events.get(plan_id, []) if op == op_id]
    return matches[-1] if matches else None

checks.update({
    "A boundary node canceled": terminal_state("A-route", "A-meeting") == 4,
    "C targeted node canceled": terminal_state("C-open-area", "C-drive") == 4,
    "D succeeded": terminal_state("D-return-315", "D-return") == 2,
    "B unaffected and succeeded": terminal_state("B-greet", "B-watch") == 2,
})

print("\n=== ASSERTIONS ===")
for name, passed in checks.items():
    print(f"{'PASS' if passed else 'FAIL'}  {name}")
if not all(checks.values()):
    print("\n=== TRACE ===")
    print("\n".join(lines))
    raise SystemExit(1)
print(f"\nPASS: targeted cancellation isolation verified in {rel():.2f}s")
PY
