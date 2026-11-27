# SPDX-License-Identifier: MulanPSL-2.0
"""End-to-end check of Pilot's correction rules, with a scripted stand-in model.

Real Atlas, Executor, Pilot and the bench's mock world run as deployed
(`rbnx boot`); only the model is a stub that records every request it gets
and answers from a script. So what is checked is what the real Pilot sends,
round by round.

  python3 testing/model_bench/corrections_e2e.py \\
      --deploy testing/model_bench/.deploy-N --server 127.0.0.1:<its Atlas> \\
      --world testing/model_bench/.deploy-N/world

The deployment must have been made with
`make_deploy.py --corrections-dir <dir holding goal_near_unreachable.yaml>`, and
its proxy must forward to the stub started here: this script writes the world's
`target.json` to point at it (and restores nothing: use a deployment of its own).

Checked:
  1. goal_near fails with "no free cell"  -> the next request carries the hint
     (names the object) and the one before it does not.
  2. goal_near succeeds                   -> no request carries the hint.
  3. goal_near fails, then the user sends a new message in the same session
     -> the request after the new message no longer carries it.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HINT = "Pilot corrections for this step"
GOAL_NEAR = "robonix/system/scene/goal_near"
# The name the model uses for the contract in an RTDL "cap": the last two path
# segments joined by "_" (`discovery::llm_name`).
CAP = "scene.scene_goal_near"


def rtdl_reply(children: list[dict], description: str, status: str = "in_progress") -> str:
    return json.dumps({
        "content": "",
        "rtdl_description": description,
        "message_updates": [],
        "rtdl_parent_task_id": "T1",
        "task_updates": [{"task_id": "T1", "status": status}] if status != "in_progress" else [],
        "rtdl": {"op": "sequence", "op_id": 1, "description": description, "children": children},
    })


def final_reply(text: str) -> str:
    return json.dumps({
        "content": text,
        "rtdl_description": "answer",
        "message_updates": [],
        "rtdl_parent_task_id": "T1",
        "task_updates": [{"task_id": "T1", "status": "done"}],
        "rtdl": {"op": "sequence", "op_id": 1, "description": "answer", "children": []},
    })


class StubModel:
    """An OpenAI-compatible endpoint that replays `script`, one reply per request."""

    def __init__(self, port: int) -> None:
        self.requests: list[dict] = []
        self.script: list[str] = []
        self._lock = threading.Lock()
        stub = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_: object) -> None:
                pass

            def do_GET(self) -> None:  # model list, probed at startup
                body = json.dumps({"data": [{"id": "stub", "owned_by": "stub"}]}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self) -> None:
                request = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                with stub._lock:
                    stub.requests.append(request)
                    reply = stub.script.pop(0) if stub.script else final_reply("(script exhausted)")
                chunk = "data: " + json.dumps({"choices": [{"delta": {"content": reply}}]}) + "\n\n"
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.end_headers()
                self.wfile.write((chunk + "data: [DONE]\n\n").encode())

        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def reset(self, script: list[str]) -> None:
        with self._lock:
            self.requests, self.script = [], list(script)

    def texts(self) -> list[str]:
        """Every request's messages flattened to one string each."""
        out = []
        for request in self.requests:
            parts = []
            for message in request.get("messages", []):
                content = message.get("content")
                if isinstance(content, list):
                    content = "".join(part.get("text", "") for part in content if isinstance(part, dict))
                parts.append(str(content or ""))
            out.append("\n".join(parts))
        return out


def set_world(world: Path, case: str, fail: dict[str, str]) -> None:
    (world / "state.json").write_text(json.dumps({
        "case": case, "image": None, "fail": fail, "nav_s": 1, "call_s": 0.2,
        "images_dir": str(Path(__file__).resolve().parent / ".images"),
    }))


def ask(rbnx: str, server: str, task: str, session: str, timeout: float) -> None:
    subprocess.run([rbnx, "ask", task, "--json", "--server", server, "--session", session],
                   capture_output=True, text=True, timeout=timeout)


def goal_near_call(object_id: str) -> dict:
    return {"op": "do", "op_id": 2, "description": f"find a spot next to {object_id}",
            "cap": CAP, "args": {"object_id": object_id}}


class Failures:
    def __init__(self) -> None:
        self.items: list[str] = []

    def check(self, ok: bool, what: str) -> None:
        print(f"  {'ok  ' if ok else 'FAIL'} {what}")
        if not ok:
            self.items.append(what)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--deploy", required=True, help="the booted bench deployment")
    ap.add_argument("--server", required=True, help="its Atlas, host:port")
    ap.add_argument("--world", help="its world directory (default <deploy>/world)")
    ap.add_argument("--rbnx", default="rbnx")
    ap.add_argument("--stub-port", type=int, default=52999)
    ap.add_argument("--timeout", type=float, default=90)
    a = ap.parse_args()
    world = Path(a.world or Path(a.deploy) / "world")

    stub = StubModel(a.stub_port)
    # The deployment's proxy forwards to whatever target.json names.
    env_file = Path(tempfile.mkdtemp()) / "stub.env"
    env_file.write_text(f"VLM_BASE_URL=http://127.0.0.1:{a.stub_port}/v1\nVLM_API_KEY=stub\n")
    (world / "target.json").write_text(json.dumps({"env_file": str(env_file), "model": "stub"}))

    failures = Failures()

    print("1. goal_near reports no free cell")
    set_world(world, "e2e-fail", {GOAL_NEAR: "no free cell within 1.5 m of scene.object.0002"})
    stub.reset([
        rtdl_reply([goal_near_call("scene.object.0002")], "find a spot by the plant"),
        final_reply("I could not get next to the plant."),
    ])
    ask(a.rbnx, a.server, "go next to the plant", str(uuid.uuid4()), a.timeout)
    texts = stub.texts()
    failures.check(len(texts) >= 2, f"the model was asked at least twice (asked {len(texts)} times)")
    if len(texts) >= 2:
        failures.check(HINT not in texts[0], "the request before the failure carries no hint")
        failures.check(HINT in texts[1], "the request after the failure carries the hint")
        failures.check("scene.object.0002" in texts[1].split(HINT)[-1],
                       "the hint names the object that cannot be reached")

    print("2. goal_near succeeds")
    set_world(world, "e2e-ok", {})
    stub.reset([
        rtdl_reply([goal_near_call("scene.object.0002")], "find a spot by the plant"),
        final_reply("Done."),
    ])
    ask(a.rbnx, a.server, "go next to the plant", str(uuid.uuid4()), a.timeout)
    failures.check(len(stub.texts()) >= 2 and all(HINT not in t for t in stub.texts()),
                   "no request carries the hint")

    print("3. a new user message ends the rule")
    set_world(world, "e2e-new-message", {GOAL_NEAR: "no free cell within 1.5 m of scene.object.0002"})
    session = str(uuid.uuid4())
    stub.reset([
        rtdl_reply([goal_near_call("scene.object.0002")], "find a spot by the plant"),
        final_reply("I could not get next to the plant."),
        final_reply("Sure, nothing else to do."),
    ])
    ask(a.rbnx, a.server, "go next to the plant", session, a.timeout)
    time.sleep(1)
    ask(a.rbnx, a.server, "never mind, just say hello", session, a.timeout)
    texts = stub.texts()
    failures.check(len(texts) >= 3, f"the model was asked three times (asked {len(texts)} times)")
    if len(texts) >= 3:
        failures.check(HINT in texts[1], "the request after the failure carried the hint")
        failures.check(HINT not in texts[2], "the request after the new message does not")

    print()
    if failures.items:
        print(f"{len(failures.items)} check(s) failed")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
