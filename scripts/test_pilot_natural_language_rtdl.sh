#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Black-box Pilot test: natural-language steers must produce correctly scoped
# RTDL plans. The driver never constructs an RTDL Plan itself.
set -euo pipefail

ROOT="${ROBONIX_SOURCE_PATH:-$(cd "$(dirname "$0")/.." && pwd)}"
DEPLOY_DIR="${ROBONIX_TEST_DEPLOY_DIR:-$HOME/robot-agilex-ranger_mini_v3}"
ATLAS_ADDR="${RBNX_TEST_ATLAS_ADDR:-127.0.0.1:52051}"
EXECUTOR_ADDR="${RBNX_TEST_EXECUTOR_ADDR:-127.0.0.1:52061}"
PILOT_ADDR="${RBNX_TEST_PILOT_ADDR:-127.0.0.1:52071}"
WORK="$(mktemp -d /tmp/robonix-pilot-language.XXXXXX)"
TRACE="$WORK/timeline.log"
ATLAS_LOG="$WORK/atlas.log"
EXECUTOR_LOG="$WORK/executor.log"
PILOT_LOG="$WORK/pilot.log"

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
    tail -120 "$PILOT_LOG" >&2 || true
    echo "--- executor log tail ---" >&2
    tail -80 "$EXECUTOR_LOG" >&2 || true
  else
    rm -rf "$WORK"
  fi
  exit "$rc"
}
trap cleanup EXIT INT TERM

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
export ROBONIX_PILOT_MAX_TOOL_ROUNDS=64
export SCRIBE_STDOUT_LEVEL=warn
export SCRIBE_FILE_LEVEL=debug

wait_port() {
  python3 - "$1" <<'PY'
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

"$HOME/.cargo/bin/robonix-atlas" \
  --listen "$ATLAS_ADDR" --capabilities "$ROOT/capabilities" \
  >"$ATLAS_LOG" 2>&1 &
ATLAS_PID=$!
wait_port "$ATLAS_ADDR"

"$HOME/.cargo/bin/robonix-executor" \
  --atlas "$ATLAS_ADDR" --listen "$EXECUTOR_ADDR" --id executor \
  >"$EXECUTOR_LOG" 2>&1 &
EXECUTOR_PID=$!
wait_port "$EXECUTOR_ADDR"

"$HOME/.cargo/bin/robonix-pilot" \
  --atlas "$ATLAS_ADDR" --listen "$PILOT_ADDR" --id pilot \
  >"$PILOT_LOG" 2>&1 &
PILOT_PID=$!
wait_port "$PILOT_ADDR"

PROTO_SOURCE="$(find "$ROOT/target" -path '*/out/robonix_contracts.proto' -printf '%T@ %h\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"
if [[ -z "$PROTO_SOURCE" || ! -d "$PROTO_SOURCE" ]]; then
  echo "generated proto source not found; build Pilot first" >&2
  exit 2
fi
PROTO_GEN="$WORK/proto_gen"
mkdir -p "$PROTO_GEN"
cp "$PROTO_SOURCE"/*.proto "$PROTO_GEN/"
python3 -m grpc_tools.protoc -I "$PROTO_GEN" \
  --python_out="$PROTO_GEN" --grpc_python_out="$PROTO_GEN" "$PROTO_GEN"/*.proto

PYTHONPATH="$PROTO_GEN${PYTHONPATH:+:$PYTHONPATH}" python3 - "$PILOT_ADDR" "$TRACE" <<'PY'
import asyncio
import json
import pathlib
import shlex
import sys
import time
import uuid

import grpc
import pilot_pb2
import robonix_contracts_pb2_grpc as contracts

endpoint, trace_arg = sys.argv[1:3]
trace = pathlib.Path(trace_arg)
trace_q = shlex.quote(str(trace))
session_id = f"pilot-language-{uuid.uuid4()}"
turn_id = f"turn-{uuid.uuid4()}"
started = time.monotonic()
plans = []
node_events = []
task_states = []
statuses = []
final_texts = []
text_chunks = []
milestones = {}


def rel() -> float:
    return time.monotonic() - started


def mark_command(start_label: str, duration: int, end_label: str) -> str:
    return (
        f'printf "%.3f {start_label}\\n" "$(date +%s.%N)" >> {trace_q}; '
        f'sleep {duration}; '
        f'printf "%.3f {end_label}\\n" "$(date +%s.%N)" >> {trace_q}'
    )


a1 = mark_command("A_RESTAURANT_START", 60, "A_RESTAURANT_END")
a2 = mark_command("A_MEETING_START", 20, "A_MEETING_END")
c_cmd = mark_command("C_OPEN_START", 60, "C_OPEN_END")
d_cmd = mark_command("D_315_START", 10, "D_315_END")
greet_cmd = (
    f'printf "%.3f B_GREET_START\\n" "$(date +%s.%N)" >> {trace_q}; '
    f'for i in $(seq 1 60); do printf "%.3f B_GREET_TICK_%02d\\n" '
    f'"$(date +%s.%N)" "$i" >> {trace_q}; sleep 1; done; '
    f'printf "%.3f B_GREET_END\\n" "$(date +%s.%N)" >> {trace_q}'
)

messages = {
    "initial": (
        "这是纯 Bash 编码代理任务，不涉及机器人硬件。请先执行任务 A1，再执行任务 A2。"
        "A1 和 A2 是两个有先后关系、可分别停止的步骤，不要合并成一个 shell command。"
        f"任务 A1 的命令是：{a1}。任务 A2 的命令是：{a2}。"
    ),
    "greet": (
        "请同时运行一个独立的长期 greet 问候任务，持续运行，不要停止或等待前面的 A 任务。"
        f"greet 命令是：{greet_cmd}。"
    ),
    "stop_after_a1": (
        "我改变主意了：请在 A1 任务完成之后停止整个 A 计划。"
        "持续 greet 任务必须继续运行，不要取消它。"
    ),
    "open_area": (
        "请前往开阔区域；在这个 Bash 测试中对应启动一个新的独立任务 C。"
        f"任务 C 的命令是：{c_cmd}。持续 greet 仍然保持运行。"
    ),
    "cancel_and_return": (
        "取消前往开阔区域对应的任务 C，但不要取消持续 greet。"
        "然后返回 315 办公室；在这个 Bash 测试中对应执行任务 D。"
        f"任务 D 的命令是：{d_cmd}。"
    ),
}


def trace_labels():
    if not trace.exists():
        return []
    return [line.split(maxsplit=1)[1] for line in trace.read_text().splitlines()]


async def wait_label(label: str, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if label in trace_labels():
            return
        await asyncio.sleep(0.2)
    raise RuntimeError(f"timeout waiting for {label}; labels={trace_labels()}")


async def wait_final_after(after: float, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if any(event_time >= after for event_time, _ in final_texts):
            return
        await asyncio.sleep(0.2)
    raise RuntimeError(f"timeout waiting for final reply after t={after:.2f}")


async def collect_stream(stream):
    async for event in stream:
        if event.text_chunk:
            text_chunks.append((rel(), event.text_chunk))
            print(f"[{rel():6.2f}s] TEXT {event.text_chunk[:180]!r}", flush=True)
        elif event.HasField("plan"):
            plan = event.plan
            calls = []
            for node in plan.nodes:
                if node.HasField("call"):
                    calls.append({
                        "op_id": node.op_id,
                        "description": node.description,
                        "contract_id": node.call.contract_id,
                        "args_json": node.call.args_json,
                    })
            plans.append({
                "t": rel(),
                "plan_id": plan.plan_id,
                "round": plan.round,
                "calls": calls,
            })
            print(f"[{rel():6.2f}s] PLAN {plan.plan_id} calls={len(calls)}", flush=True)
            for call in calls:
                print(
                    f"           {call['contract_id'].rsplit('/', 1)[-1]} "
                    f"op={call['op_id']} args={call['args_json'][:220]}",
                    flush=True,
                )
        elif event.HasField("node_state"):
            ns = event.node_state
            node_events.append((rel(), ns.plan_id, ns.op_id, ns.state))
            print(
                f"[{rel():6.2f}s] NODE plan={ns.plan_id} op={ns.op_id} state={ns.state}",
                flush=True,
            )
        elif event.HasField("task_state"):
            task_states.append(event.task_state.status)
            print(f"[{rel():6.2f}s] TASK_STATE {event.task_state.status}", flush=True)
        elif event.HasField("status"):
            statuses.append(event.status.message)
            print(f"[{rel():6.2f}s] STATUS {event.status.message}", flush=True)
        elif event.final_text:
            final_texts.append((rel(), event.final_text))
            print(f"[{rel():6.2f}s] FINAL {event.final_text[:180]!r}", flush=True)


async def submit_steer(stub, name: str):
    print(f"[{rel():6.2f}s] STEER {name}: {messages[name]}", flush=True)
    task = pilot_pb2.Task(
        task_id=f"steer-{name}-{uuid.uuid4()}",
        session_id=session_id,
        text=messages[name],
        context_json=json.dumps({
            "client": "pilot-natural-language-test",
            "interaction_mode": "steer",
            "steer": True,
            "expected_turn_id": turn_id,
        }),
    )
    collector = asyncio.create_task(collect_stream(stub.SubmitTask(task)))
    await asyncio.sleep(0)
    return collector


async def main():
    async with grpc.aio.insecure_channel(endpoint) as channel:
        stub = contracts.RobonixSystemPilotStub(channel)
        initial = pilot_pb2.Task(
            task_id=turn_id,
            session_id=session_id,
            text=messages["initial"],
            context_json=json.dumps({
                "client": "pilot-natural-language-test",
                "interaction_mode": "task",
            }),
        )
        original_stream = stub.SubmitTask(initial)
        collectors = [asyncio.create_task(collect_stream(original_stream))]

        await wait_label("A_RESTAURANT_START", 45)
        collectors.append(await submit_steer(stub, "greet"))
        await wait_label("B_GREET_START", 45)

        await asyncio.sleep(2)
        collectors.append(await submit_steer(stub, "stop_after_a1"))
        await wait_label("A_RESTAURANT_END", 65)
        milestones["a_stopped"] = rel()
        await wait_final_after(milestones["a_stopped"], 45)
        await asyncio.sleep(1)

        collectors.append(await submit_steer(stub, "open_area"))
        await wait_label("C_OPEN_START", 45)
        await asyncio.sleep(5)
        collectors.append(await submit_steer(stub, "cancel_and_return"))
        await wait_label("D_315_END", 60)
        milestones["d_completed"] = rel()
        await wait_final_after(milestones["d_completed"], 45)
        await wait_label("B_GREET_END", 90)
        await asyncio.sleep(3)

        for collector in collectors:
            collector.cancel()
        for collector in collectors:
            try:
                await collector
            except asyncio.CancelledError:
                pass


asyncio.run(asyncio.wait_for(main(), timeout=210))

labels = trace_labels()
ticks = [label for label in labels if label.startswith("B_GREET_TICK_")]


def plan_with_marker(marker: str):
    return next(
        (plan for plan in plans if any(marker in call["args_json"] for call in plan["calls"])),
        None,
    )


plan_a1 = plan_with_marker("A_RESTAURANT_START")
plan_a2 = plan_with_marker("A_MEETING_START")
plan_b = plan_with_marker("B_GREET_START")
plan_c = plan_with_marker("C_OPEN_START")
plan_d = plan_with_marker("D_315_START")
control_calls = [
    (plan, call)
    for plan in plans
    for call in plan["calls"]
    if call["contract_id"].rsplit("/", 1)[-1]
    in {"cancel_plan", "cancel_all_plans", "stop_plan_at"}
]
checks = {
    "Pilot planned A1": plan_a1 is not None,
    "A1 completed": "A_RESTAURANT_END" in labels,
    "A2 never started": "A_MEETING_START" not in labels,
    "Pilot planned independent greet": plan_b is not None,
    "greet produced all 60 heartbeats": len(ticks) == 60 and "B_GREET_END" in labels,
    "Pilot planned open-area task C": plan_c is not None,
    "C started but did not finish": "C_OPEN_START" in labels and "C_OPEN_END" not in labels,
    "Pilot planned return task D": plan_d is not None,
    "D completed": "D_315_END" in labels,
    "plan control never appeared as an RTDL node": not control_calls,
    "Pilot reported out-of-band plan control": "Plan control accepted" in statuses,
    "targeted cancel left greet unaffected": "C_OPEN_END" not in labels and "B_GREET_END" in labels,
    "each RTDL plan had live natural-language feedback": len(text_chunks) >= len(plans),
    "no final reply was broadcast more than once": len({text for _, text in final_texts}) == len(final_texts),
    "return completion reply followed D completion": any(
        event_time >= milestones.get("d_completed", float("inf"))
        for event_time, _ in final_texts
    ),
    "boundary-stop completion reply followed A1 completion": any(
        event_time >= milestones.get("a_stopped", float("inf"))
        for event_time, _ in final_texts
    ),
}

# If A2 was already placed in the same in-flight tree as A1, the out-of-band
# boundary stop must prevent it from starting. If A2 was not yet dispatched,
# latest-steer suppression is the correct behavior.
if plan_a2 is not None and plan_a1 is not None and plan_a2["plan_id"] == plan_a1["plan_id"]:
    checks["A sequence stopped at the explicit A1 boundary"] = (
        "A_MEETING_START" not in labels and "Plan control accepted" in statuses
    )
else:
    checks["latest steer suppressed undispatched A2"] = "A_MEETING_START" not in labels

print("\n=== PILOT BLACK-BOX ASSERTIONS ===")
for name, passed in checks.items():
    print(f"{'PASS' if passed else 'FAIL'}  {name}")
if not all(checks.values()):
    print("\n=== PLAN RECORDS ===")
    print(json.dumps(plans, ensure_ascii=False, indent=2))
    print("\n=== TRACE ===")
    print(trace.read_text() if trace.exists() else "<empty>")
    raise SystemExit(1)
print(f"\nPASS: natural-language Pilot RTDL management verified in {rel():.2f}s")
PY
