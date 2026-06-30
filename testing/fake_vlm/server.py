# SPDX-License-Identifier: MulanPSL-2.0
"""Deterministic fake VLM for robonix CI.

Pilot talks to an OpenAI-compatible ``/v1/chat/completions`` endpoint in
**streaming** mode and, critically, drives planning through the *RTDL envelope*
protocol: the assistant must return ONE JSON object in ``content`` (NOT OpenAI
``tool_calls`` — pilot hard-errors on tool_calls in RTDL mode). This server
replaces a real VLM so planning is fully deterministic: every task maps to a
scripted sequence of RTDL envelopes.

How a request is served (the server is effectively stateless — it derives
everything from the request body):

1. Parse the advertised capability catalog out of the **system** message.
   Pilot renders it as ``- capability_name: <provider>.<area>_<leaf>`` lines
   (planner.rs build_rtdl_prompt). Scenarios reference caps by a short
   substring (e.g. ``camera_snapshot``); we resolve it to the exact advertised
   name at request time, so scenarios stay decoupled from provider_id / exact
   contract wording.
2. Read the FIRST ``user`` message text — that is the task prompt. Look up the
   scenario whose ``task`` equals it.
3. Count ``assistant`` messages already in history = the current round index.
   Serve ``scenario.steps[round]``; past the end, serve a terminal "done"
   envelope so a turn can never hang.
4. Stream the envelope JSON back as Server-Sent Events, then ``finish_reason:
   stop`` and ``[DONE]``.

No third-party deps (stdlib only) so CI needs no extra pip install for the VLM.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

# ── catalog parsing ──────────────────────────────────────────────────────────

# Matches the catalog lines pilot renders into the system prompt, e.g.
#   - capability_name: tiago_camera.camera_snapshot
_CAP_LINE = re.compile(r"^\s*-\s*capability_name:\s*(\S+)\s*$", re.MULTILINE)


def advertised_caps(messages: list[dict]) -> list[str]:
    """Return the provider-qualified capability names pilot advertised.

    Scans every system message (the catalog lives in the round-0 full prompt
    and is re-appended each round). De-duplicates while preserving order.
    """
    seen: dict[str, None] = {}
    for m in messages:
        if m.get("role") != "system":
            continue
        content = m.get("content")
        if not isinstance(content, str):
            continue
        for name in _CAP_LINE.findall(content):
            seen.setdefault(name, None)
    return list(seen)


def resolve_cap(match: str, caps: list[str]) -> str | None:
    """Resolve a scenario's short cap matcher to an exact advertised name.

    Prefers an exact ``provider.area_leaf`` match, then a suffix match on the
    part after the dot, then a plain substring. Returns None if nothing
    advertised matches — the runner then sees the cap was never invoked and
    fails the scenario, which is the correct signal ("expected tool absent").
    """
    if match in caps:
        return match
    for c in caps:
        leaf = c.split(".", 1)[-1]
        if leaf == match:
            return c
    for c in caps:
        if match in c:
            return c
    return None


# ── RTDL envelope construction ───────────────────────────────────────────────


def do_node(cap: str, args: dict, description: str) -> dict:
    """One capability call. op_id is always 0 — pilot assigns the real id."""
    return {"op": "do", "op_id": 0, "description": description, "cap": cap, "args": args}


def build_envelope(step: dict, caps: list[str], unresolved: list[str]) -> dict:
    """Turn a scenario step into a 4-key RTDL envelope pilot will accept.

    A step lists ``caps`` (each {match, args, description?}); we wrap the
    resolved do-nodes in a sequence. An empty cap list yields the canonical
    "wait" tree. ``status`` drives task completion: "done" ends the turn.
    Unresolved matchers are recorded (caller logs them) and dropped, so the
    plan still parses but the scenario will fail its coverage assertion.
    """
    children = []
    for spec in step.get("caps", []):
        cap = resolve_cap(spec["match"], caps)
        if cap is None:
            unresolved.append(spec["match"])
            continue
        children.append(do_node(cap, spec.get("args", {}), spec.get("description", spec["match"])))

    rtdl = {
        "op": "sequence",
        "op_id": 0,
        "description": step.get("description", "wait" if not children else "plan"),
        "children": children,
    }

    status = step.get("status", "in_progress")
    task_update = None
    if status:
        task_update = {
            "goal": step.get("goal", "ci scenario"),
            "success_criterion": step.get("success_criterion", "scripted steps complete"),
            "status": status,
        }
    return {
        "content": step.get("content", ""),
        "rtdl_description": step.get("rtdl_description", "ci"),
        "rtdl": rtdl,
        "task_update": task_update,
    }


def terminal_envelope(note: str) -> dict:
    """Safety envelope served past the scripted steps: empty tree + done."""
    return {
        "content": note,
        "rtdl_description": "ci-done",
        "rtdl": {"op": "sequence", "op_id": 0, "description": "wait", "children": []},
        "task_update": {
            "goal": "ci scenario",
            "success_criterion": "scripted steps complete",
            "status": "done",
        },
    }


# ── HTTP / SSE ───────────────────────────────────────────────────────────────


class Handler(BaseHTTPRequestHandler):
    # set by main(): {task_text: scenario_dict}
    scenarios: dict[str, dict] = {}

    def log_message(self, *_args):  # noqa: D401 - silence default stderr spam
        """Suppress the default per-request access log."""

    def _emit(self, line: str):
        print(f"[fake-vlm] {line}", file=sys.stderr, flush=True)

    def do_GET(self):  # noqa: N802 - http.server API
        """Answer /v1/models so clients that probe it don't error."""
        if self.path.rstrip("/").endswith("/models"):
            body = json.dumps(
                {"object": "list", "data": [{"id": "fake-vlm", "object": "model"}]}
            ).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def do_POST(self):  # noqa: N802 - http.server API
        """Serve one scripted RTDL envelope for a chat-completions request."""
        if not self.path.rstrip("/").endswith("/chat/completions"):
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", 0))
        try:
            req = json.loads(self.rfile.read(length) or b"{}")
        except json.JSONDecodeError:
            self.send_error(400, "bad json")
            return

        messages = req.get("messages", [])
        caps = advertised_caps(messages)
        user_text = self._all_user_text(messages)
        # Round index = which planning round this is. Pilot does NOT echo its own
        # RTDL envelope back as an `assistant` message; instead it feeds each
        # round's capability results back as additional `user` messages. So the
        # first call has 1 user message (the task) = round 0, and every later
        # round adds one user message. round = (#user messages) - 1.
        rounds = max(0, sum(1 for m in messages if m.get("role") == "user") - 1)

        scenario = self._match_scenario(user_text)
        if scenario is None:
            self._emit(f"no scenario matched user text {user_text[:80]!r}; ending turn")
            envelope = terminal_envelope("no scripted scenario; nothing to do")
        else:
            steps = scenario.get("steps", [])
            if rounds < len(steps):
                unresolved: list[str] = []
                envelope = build_envelope(steps[rounds], caps, unresolved)
                if unresolved:
                    self._emit(
                        f"scenario {scenario['name']!r} round {rounds}: "
                        f"UNRESOLVED caps {unresolved} (advertised: {caps})"
                    )
                else:
                    called = [c.get("match") for c in steps[rounds].get("caps", [])]
                    self._emit(f"scenario {scenario['name']!r} round {rounds}: caps {called}")
            else:
                envelope = terminal_envelope("scripted steps complete")

        self._stream_envelope(req.get("model", "fake-vlm"), envelope)

    def _all_user_text(self, messages: list[dict]) -> str:
        """Concatenate every user message's text (the task may be wrapped)."""
        parts: list[str] = []
        for m in messages:
            if m.get("role") != "user":
                continue
            c = m.get("content")
            if isinstance(c, str):
                parts.append(c)
            elif isinstance(c, list):  # multimodal: concat text parts
                parts.extend(p.get("text", "") for p in c if p.get("type") == "text")
        return "\n".join(parts)

    def _match_scenario(self, user_text: str) -> dict | None:
        """Find the scenario whose task string appears in the user text.

        Pilot may frame the submitted task inside a larger user block, so we
        match by containment rather than equality. Prefer the longest task
        string when several match, to avoid a short task shadowing a longer one.
        """
        hits = [s for t, s in self.scenarios.items() if t and t in user_text]
        if not hits:
            return None
        return max(hits, key=lambda s: len(s["task"]))

    def _stream_envelope(self, model: str, envelope: dict):
        """Stream the envelope JSON as one content chunk, then stop + [DONE]."""
        payload = json.dumps(envelope, ensure_ascii=False)
        created = int(time.time())
        base = {"id": "fake-cmpl", "object": "chat.completion.chunk", "created": created, "model": model}

        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()

        def chunk(delta: dict, finish):
            obj = dict(base, choices=[{"index": 0, "delta": delta, "finish_reason": finish}])
            self.wfile.write(f"data: {json.dumps(obj)}\n\n".encode())
            self.wfile.flush()

        chunk({"role": "assistant", "content": payload}, None)
        chunk({}, "stop")
        self.wfile.write(b"data: [DONE]\n\n")
        self.wfile.flush()


def load_scenarios(scenario_dir: Path) -> dict[str, dict]:
    """Load every scenario YAML, keyed by its ``task`` prompt for lookup."""
    out: dict[str, dict] = {}
    for path in sorted([*scenario_dir.rglob("*.yaml"), *scenario_dir.rglob("*.yml")]):
        data = yaml.safe_load(path.read_text())
        data.setdefault("name", path.stem)
        out[data["task"].strip()] = data
    return out


def main():
    ap = argparse.ArgumentParser(description="Deterministic fake VLM (RTDL) for robonix CI")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=18080)
    ap.add_argument(
        "--scenarios",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "scenarios",
    )
    args = ap.parse_args()

    Handler.scenarios = load_scenarios(args.scenarios)
    print(
        f"[fake-vlm] {len(Handler.scenarios)} scenario(s) on http://{args.host}:{args.port}/v1",
        file=sys.stderr,
        flush=True,
    )
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
