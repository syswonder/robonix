#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Exercise streamed task ownership with real Atlas, Pilot and Executor processes.

Build their debug binaries first. Requires grpcio and grpcio-tools; all model
replies are scripted locally. Logs and session transcripts are retained in /tmp.
"""

import asyncio
import json
import os
from pathlib import Path
import shlex
import socket
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = Path(__file__).resolve().parents[1]
WORK = Path(tempfile.mkdtemp(prefix="robonix-task-stream-"))
TRACE = WORK / "actions"
requests = []
streamed_before_tail = []
mode = "concurrent"
sent = set()
response_at = 0.0


def port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def actions():
    return TRACE.read_text() if TRACE.exists() else ""


def envelope(task, command=None, message=None, status=None):
    data = {
        "content": f"Handling {task}",
        "message_updates": [],
        "rtdl_parent_task_id": task,
        "rtdl_description": task,
        "rtdl": {"op": "sequence", "children": []},
        "task_updates": [],
    }
    if message:
        data["message_updates"] = [{"message_id": message, "parent_task_id": "new"}]
        data["rtdl_parent_task_id"] = "new"
    if command:
        data["rtdl"] = {
            "op": "sequence",
            "description": task,
            "children": [
                {
                    "op": "do",
                    "description": task,
                    "cap": "executor.builtin_run_command",
                    "args": {"command": command},
                }
            ],
        }
    if status:
        data["task_updates"] = [
            {
                "task_id": "new" if message else task,
                "goal": task,
                "success_criterion": "action completed",
                "status": status,
            }
        ]
    return data


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        payload = json.dumps(
            {"data": [{"id": "scripted", "context_window": 32768}]}
        ).encode()
        self.send_response(200)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_POST(self):
        global response_at
        request = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        text = json.dumps(request["messages"])
        requests.append(request)
        trace = shlex.quote(str(TRACE))
        request_mode = mode
        if request_mode in {"delayed", "silent"}:
            if request_mode == "delayed" and not response_at:
                response_at = time.monotonic() + 6
            delay = (
                max(0, response_at - time.monotonic())
                if request_mode == "delayed" else 6
            )
            time.sleep(delay)
            data = envelope("T1", status="done")
            data["rtdl_parent_task_id"] = None
            marker = None
        elif request_mode == "stalled":
            data = envelope("T1", f"printf D >> {trace}")
            marker = "D"
        elif request_mode == "concurrent":
            if "A" not in sent:
                sent.add("A")
                data = envelope(
                    "T1",
                    f"printf A >> {trace}; sleep 4; printf a >> {trace}",
                    status="done",
                )
                marker = "A"
            elif "B" not in sent:
                assert "second-task" in text, (
                    "Pilot replanned without a new task or completion"
                )
                sent.add("B")
                data = envelope(
                    "T2",
                    f"printf B >> {trace}; sleep 1; printf b >> {trace}",
                    message="m2",
                    status="done",
                )
                marker = "B"
            else:
                done = actions()
                assert "a" in done or "b" in done, "unexpected extra planning"
                data = envelope("T2" if "b" in done else "T1")
                data["rtdl_parent_task_id"] = None
                data["task_updates"] = [
                    {"task_id": task, "status": "done"}
                    for (task, marker) in [("T1", "a"), ("T2", "b")]
                    if marker in done
                ]
                marker = None
        elif "C" not in sent:
            sent.add("C")
            data = envelope(
                "T1", f"printf C >> {trace}; printf streamed-C-result", status="done"
            )
            marker = "C"
        else:
            assert actions().count("C") == 1
            assert "Pilot rejected your RTDL output" in text
            assert any(
                isinstance(message.get("content"), str)
                and message["content"].startswith(
                    "Executor feedback for the current RTDL leaf"
                )
                and "streamed-C-result" in message["content"]
                for message in request["messages"]
            ), "completed action result missing from retry history"
            data = envelope("T1", status="done")
            data["rtdl_parent_task_id"] = None
            marker = None
        raw = json.dumps(data, separators=(",", ":"))
        (prefix, tail) = raw.split(',"task_updates":', 1)
        prefix += ","
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.end_headers()

        def emit(part):
            self.wfile.write(
                (
                    "data: "
                    + json.dumps({"choices": [{"delta": {"content": part}}]})
                    + "\n\n"
                ).encode()
            )
            self.wfile.flush()

        emit(prefix)
        if marker:
            end = time.monotonic() + 5
            while marker not in actions() and time.monotonic() < end:
                time.sleep(0.02)
            assert marker in actions(), (
                f"action {marker} did not start before task_updates"
            )
            streamed_before_tail.append(marker)
        if request_mode == "stalled":
            time.sleep(6)
        if request_mode == "malformed" and marker == "C":
            time.sleep(0.15)
            emit('"task_updates":BROKEN}')
        else:
            emit('"task_updates":' + tail)
        self.wfile.write(b"data: [DONE]\n\n")
        self.wfile.flush()


async def check(pilot_address):
    import grpc
    import pilot_pb2
    import robonix_contracts_pb2_grpc as contracts
    from google.protobuf.json_format import MessageToDict

    channel = grpc.aio.insecure_channel(
        pilot_address, options=[("grpc.enable_http_proxy", 0)]
    )
    await asyncio.wait_for(channel.channel_ready(), 10)
    stub = contracts.RobonixSystemPilotStub(channel)
    observed = []

    async def consume(session, task_id, text):
        async for event in stub.SubmitTask(
            pilot_pb2.Task(session_id=session, task_id=task_id, text=text), timeout=25
        ):
            if session == "multi-task" and event.HasField("task_state"):
                for task in event.task_state.tasks:
                    if task.status == "done":
                        marker = {"T1": "a", "T2": "b"}[task.task_id]
                        assert marker in actions(), (
                            f"{task.task_id} closed before its action finished"
                        )
            observed.append(MessageToDict(event, preserving_proto_field_name=True))

    first = asyncio.create_task(consume("multi-task", "first", "first-task"))
    deadline = time.monotonic() + 15
    while not any((event.get("plan") for event in observed)):
        assert time.monotonic() < deadline, observed
        await asyncio.sleep(0.05)
    second = asyncio.create_task(consume("multi-task", "second", "second-task"))
    await asyncio.gather(first, second)
    states = [event["task_state"] for event in observed if "task_state" in event]
    deadline = time.monotonic() + 10
    while True:
        records = [
            json.loads(line)
            for line in (WORK / "sessions" / "multi-task" / "transcript.jsonl")
            .read_text()
            .splitlines()
        ]
        tables = [
            record["task"] for record in records if record.get("kind") == "task_state"
        ]
        if tables and all((task["status"] == "done" for task in tables[-1]["tasks"])):
            break
        assert time.monotonic() < deadline, tables[-1]
        await asyncio.sleep(0.05)
    table = tables[-1]
    summary = {
        task["id"]: task
        | {
            "plan_ids": [
                tree["plan_id"] for tree in table["trees"] if tree["task"] == task["id"]
            ]
        }
        for task in table["tasks"]
    }
    assert set(summary) == {"T1", "T2"}, summary
    assert all(
        (
            task["status"] == "done" and len(task["plan_ids"]) == 1
            for task in summary.values()
        )
    ), summary
    assert summary["T1"]["plan_ids"] != summary["T2"]["plan_ids"]
    assert actions().index("B") < actions().index("a"), actions()
    assert actions().count("A") == actions().count("B") == 1, actions()
    assert any(
        (
            len(state.get("tasks", [])) == 2
            and all((task["status"] == "in_progress" for task in state["tasks"]))
            for state in states
        )
    )
    print(
        "PASS: two tasks keep separate streamed trees; execution overlaps; premature done rejected"
    )
    global mode
    mode = "malformed"
    await consume("broken-tail", "broken", "retry-task")
    assert actions().count("C") == 1, actions()
    last = [
        event["task_state"]
        for event in observed
        if event.get("session_id") == "broken-tail" and "task_state" in event
    ][-1]
    assert last["tasks"][0]["status"] == "done", last
    assert len(last["tasks"][0]["plan_ids"]) == 1, last
    print(
        "PASS: malformed tail retains one task-owned tree and its completed result without replay"
    )
    print("streamed before trailing updates:", streamed_before_tail)

    global response_at
    mode = "delayed"
    response_at = 0.0
    start = len(requests)
    await consume("delayed-open", "delayed", "wait-for-vlm")
    attempts = requests[start:]
    assert len(attempts) == 2, len(attempts)
    assert attempts[0] == attempts[1], "opening retry changed the prompt"
    assert any(
        event.get("session_id") == "delayed-open"
        and event.get("event_kind") == 4
        for event in observed
    ), observed
    print("PASS: delayed VLM opening retries the same request once and finishes")

    for failed_mode, session, expected_attempts in [
        ("silent", "silent-open", 2),
        ("stalled", "stalled-stream", 1),
    ]:
        mode = failed_mode
        start = len(requests)
        try:
            await consume(session, session, "timeout-test")
        except grpc.aio.AioRpcError as error:
            assert error.code() == grpc.StatusCode.INTERNAL, error
            assert "timeout" in error.details() or "deadline" in error.details(), error
        else:
            raise AssertionError(f"{failed_mode} unexpectedly finished")
        assert len(requests) - start == expected_attempts, requests[start:]
    assert actions().count("D") == 1, actions()
    print("PASS: opening retries are bounded; a stalled executing stream is not replayed")
    await channel.close()


server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
threading.Thread(target=server.serve_forever, daemon=True).start()
(atlas_port, executor_port, pilot_port) = (port(), port(), port())
env = {
    key: value for (key, value) in os.environ.items() if "proxy" not in key.lower()
} | {
    "ROBONIX_SOURCE_PATH": str(ROOT),
    "ROBONIX_SESSION_DIR": str(WORK / "sessions"),
    "SCRIBE_CONSOLE_LEVEL": "warn",
    "SCRIBE_FILE_LEVEL": "debug",
    "ROBONIX_PILOT_MAX_TOOL_ROUNDS": "12",
    "ROBONIX_PILOT_VLM_IDLE_TIMEOUT_SECS": "5",
}
processes = []


def start(name, args):
    log = (WORK / f"{name}.log").open("w")
    process = subprocess.Popen(
        [str(ROOT / "target/debug" / f"robonix-{name}"), *args],
        cwd=WORK,
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    processes.append(process)
    address = args[args.index("--listen") + 1]
    end = time.monotonic() + 15
    while time.monotonic() < end:
        assert process.poll() is None, (name, (WORK / f"{name}.log").read_text())
        try:
            with socket.create_connection(
                ("127.0.0.1", int(address.split(":")[1])), timeout=0.1
            ):
                return
        except OSError:
            time.sleep(0.05)
    raise RuntimeError(f"{name} did not listen")


try:
    proto = max(
        (ROOT / "target/debug/build").glob(
            "robonix-pilot-*/out/robonix_contracts.proto"
        ),
        key=lambda path: path.stat().st_mtime,
    ).parent
    generated = WORK / "proto"
    generated.mkdir()
    subprocess.run(
        [
            sys.executable,
            "-m",
            "grpc_tools.protoc",
            f"-I{proto}",
            f"--python_out={generated}",
            f"--grpc_python_out={generated}",
            *map(str, proto.glob("*.proto")),
        ],
        check=True,
    )
    sys.path.insert(0, str(generated))
    atlas = f"127.0.0.1:{atlas_port}"
    start("atlas", ["--listen", atlas, "--capabilities", str(ROOT / "capabilities")])
    start(
        "executor",
        [
            "--atlas",
            atlas,
            "--listen",
            f"127.0.0.1:{executor_port}",
            "--id",
            "executor",
        ],
    )
    start(
        "pilot",
        [
            "--atlas",
            atlas,
            "--listen",
            f"127.0.0.1:{pilot_port}",
            "--vlm-upstream",
            f"http://127.0.0.1:{server.server_port}/v1",
            "--vlm-api-key",
            "local-test",
            "--vlm-model",
            "scripted",
            "--vlm-context-window-tokens",
            "32768",
        ],
    )
    asyncio.run(check(f"127.0.0.1:{pilot_port}"))
finally:
    (WORK / "requests.json").write_text(json.dumps(requests, indent=2))
    for process in reversed(processes):
        process.terminate()
    for process in processes:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    server.shutdown()
    print("artifacts:", WORK)
