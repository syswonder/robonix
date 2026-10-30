# SPDX-License-Identifier: MulanPSL-2.0
"""Pilot's VLM endpoint for the bench: forwards each request, streamed, to
the model under test and records what was asked and answered.

The target (endpoint, key file, model id) is read from `target.json` in the
world directory on every request, so one boot of Robonix serves every model.
Each request appends one line to `vlm.jsonl` there: the case, the
response_format Pilot sent, the HTTP status, time to first byte and in
total, whether the reply was empty, the token usage the endpoint reported,
and the error body of a failed request.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from bench import read_env

WORLD: Path
_lock = threading.Lock()


def target() -> dict:
    t = json.loads((WORLD / "target.json").read_text())
    env = read_env(t["env_file"])
    return {"base": env["VLM_BASE_URL"].rstrip("/"), "key": env["VLM_API_KEY"], "model": t["model"]}


def case() -> str | None:
    try:
        return json.loads((WORLD / "state.json").read_text()).get("case")
    except (OSError, json.JSONDecodeError):
        return None


def usage_of(raw: bytes, streamed: bool) -> dict | None:
    """Token usage the endpoint reported, with prompt tokens served from its cache."""
    found = None
    chunks = [line[5:] for line in raw.decode(errors="replace").splitlines() if line.startswith("data:")] if streamed else [raw]
    for chunk in chunks:
        try:
            found = json.loads(chunk).get("usage") or found
        except (ValueError, AttributeError):
            pass
    if not found:
        return None
    details = found.get("prompt_tokens_details") or {}
    return {"prompt_tokens": found.get("prompt_tokens"), "completion_tokens": found.get("completion_tokens"),
            "cached_tokens": details.get("cached_tokens") or found.get("prompt_cache_hit_tokens")}


def reply_text(raw: bytes, streamed: bool) -> str:
    """The assistant content of a reply, streamed (SSE) or not."""
    if not streamed:
        try:
            return json.loads(raw)["choices"][0]["message"].get("content") or ""
        except (ValueError, KeyError, IndexError):
            return ""
    parts = []
    for line in raw.decode(errors="replace").splitlines():
        if line.startswith("data:") and line[5:].strip() not in ("", "[DONE]"):
            try:
                delta = json.loads(line[5:])["choices"][0].get("delta") or {}
                parts.append(delta.get("content") or "")
            except (ValueError, KeyError, IndexError):
                pass
    return "".join(parts)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):  # noqa: N802
        # Pilot asks for the model list to learn the context window; a
        # relay's list is long and says nothing, so answer with the target.
        body = json.dumps({"object": "list", "data": [{"id": "bench", "object": "model", "owned_by": "robonix-model-bench"}]}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):  # noqa: N802
        t = target()
        req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
        req["model"] = t["model"]
        streamed = bool(req.get("stream"))
        fmt = (req.get("response_format") or {}).get("type", "none")
        started = time.time()
        first = None
        raw = b""
        status = 0
        upstream = urllib.request.Request(
            f"{t['base']}/chat/completions",
            data=json.dumps(req).encode(),
            headers={"Content-Type": "application/json", "Authorization": f"Bearer {t['key']}",
                     "Accept": "text/event-stream" if streamed else "application/json"},
        )
        try:
            with urllib.request.urlopen(upstream, timeout=180) as r:
                status = r.status
                self.send_response(status)
                self.send_header("Content-Type", r.headers.get("Content-Type", "application/json"))
                self.end_headers()
                while chunk := r.read1(65536) if hasattr(r, "read1") else r.read(65536):
                    first = first or time.time()
                    raw += chunk
                    self.wfile.write(chunk)
                    self.wfile.flush()
        except urllib.error.HTTPError as e:
            status, raw = e.code, e.read()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
        except OSError as e:
            status, raw = 502, str(e).encode()
            self.send_error(502, str(e)[:200])
        ok = status == 200
        record = {
            "case": case(), "format": fmt, "status": status,
            "first_byte_s": round(first - started, 2) if first else None, "total_s": round(time.time() - started, 2),
            "empty": ok and not reply_text(raw, streamed).strip(),
            "usage": usage_of(raw, streamed) if ok else None,
            **({} if ok else {"error": raw.decode(errors="replace")[:500]}),
            "images": sum(1 for m in req.get("messages", []) if isinstance(m.get("content"), list)),
            # Pilot asks again with this text after rejecting a reply.
            "correction": any(isinstance(m.get("content"), str) and m["content"].startswith("Your previous RTDL response was rejected by Pilot")
                              for m in req.get("messages", [])[-2:]),
        }
        with _lock, (WORLD / "vlm.jsonl").open("a") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")


def main() -> None:
    global WORLD
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--world", required=True, help="the deployment's world directory")
    ap.add_argument("--port", type=int, default=52442)
    a = ap.parse_args()
    WORLD = Path(a.world)
    # Providers booting nearby take ephemeral ports; one may hold ours a moment.
    for attempt in range(30):
        try:
            server = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
            break
        except OSError:
            if attempt == 29:
                raise
            time.sleep(2)
    server.serve_forever()


if __name__ == "__main__":
    main()
