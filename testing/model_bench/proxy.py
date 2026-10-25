# SPDX-License-Identifier: MulanPSL-2.0
"""Pilot's VLM endpoint for the bench: forwards each request, streamed, to
the model under test and records what was asked and answered.

The target (endpoint, key file, model id) is read from `target.json` in the
world directory on every request, so one boot of Robonix serves every model.
Each request appends one line to `vlm.jsonl` there: the case, the
response_format Pilot sent, the HTTP status, time to first byte and in
total, whether the reply was empty, the token usage the endpoint reported,
and the error body of a failed request.
When state.json names a request_log, it also records message text and roles,
image byte counts, and sampling fields there, without image data or headers.
"""

from __future__ import annotations

import argparse
import base64
import binascii
import http.client
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


def state() -> dict:
    try:
        return json.loads((WORLD / "state.json").read_text())
    except (OSError, json.JSONDecodeError):
        return {}


def image_bytes(url: str) -> int | None:
    """Inline images have a measurable size; remote images are never fetched."""
    metadata, comma, payload = url.partition(",")
    if not comma or not metadata.startswith("data:image/") or not metadata.endswith(";base64"):
        return None
    try:
        return len(base64.b64decode(payload, validate=True))
    except (ValueError, binascii.Error):
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
            "cached_tokens": details.get("cached_tokens", found.get("prompt_cache_hit_tokens"))}


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


def stream_error(raw: bytes) -> str | None:
    for line in raw.decode(errors="replace").splitlines():
        if not line.startswith("data:"):
            continue
        try:
            error = json.loads(line[5:]).get("error")
        except (ValueError, AttributeError):
            continue
        if error:
            return json.dumps(error, ensure_ascii=False)
    return None


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
        started, active = time.time(), state()
        tag = active.get("case")
        t = target()
        req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
        req["model"] = t["model"]
        streamed = bool(req.get("stream"))
        fmt = (req.get("response_format") or {}).get("type", "none")
        first = None
        raw = b""
        status = 0
        error = None
        sent_headers = False
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
                sent_headers = True
                while chunk := r.read1(65536) if hasattr(r, "read1") else r.read(65536):
                    first = first or time.time()
                    raw += chunk
                    self.wfile.write(chunk)
                    self.wfile.flush()
        except urllib.error.HTTPError as e:
            status, raw = e.code, e.read()
            e.close()
            error = raw.decode(errors="replace") or f"HTTP {status}: {e.reason}"
            try:
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
            except OSError:
                pass  # The upstream HTTP error is already recorded.
        except (OSError, http.client.HTTPException) as e:
            status = 502
            error = str(e)
            if isinstance(e, http.client.IncompleteRead):
                raw += e.partial
            if not sent_headers:
                try:
                    self.send_error(502, "Upstream connection failed")
                except OSError:
                    pass  # Pilot may have abandoned the request.
        ended = time.time()
        error = error or (stream_error(raw) if streamed else None)
        if streamed and status == 200 and error is None and not any(
                line.startswith(b"data:") and line[5:].strip() == b"[DONE]" for line in raw.splitlines()):
            error = "Upstream stream ended without [DONE]"
        if error:
            try:
                error = json.dumps(json.loads(error), ensure_ascii=False)
            except ValueError:
                pass
        images = [part["image_url"]["url"] for m in req.get("messages", [])
                  if isinstance(m.get("content"), list) for part in m["content"]
                  if part.get("type") == "image_url"]
        # Error bodies can echo credentials or image input sent to the endpoint.
        def safe(text):
            for value in [t["key"], *images]:
                if value:
                    values = [value, json.dumps(value)[1:-1]]
                    if value.startswith("data:"):
                        values.append(value.partition(",")[2])
                    for literal in values:
                        if literal:
                            text = text.replace(literal, "[redacted]").replace(literal.replace("/", "\\/"), "[redacted]")
            return text

        ok = status == 200
        record = {
            "case": tag, "format": fmt, "status": status,
            "start": started, "end": ended, "reply": safe(reply_text(raw, streamed)) if sent_headers else "",
            "first_byte_s": round(first - started, 2) if first else None, "total_s": round(ended - started, 2),
            "empty": ok and not reply_text(raw, streamed).strip(),
            "usage": usage_of(raw, streamed) if sent_headers else None,
            "error": safe(error)[:500] if error else None,
            "images": len(images),
            # Pilot asks again with this text after rejecting a reply.
            "correction": any(isinstance(m.get("content"), str) and m["content"].startswith("Your previous RTDL response was rejected by Pilot")
                              for m in req.get("messages", [])[-2:]),
        }
        with _lock, (WORLD / "vlm.jsonl").open("a") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
        if active.get("request_log"):
            messages = []
            for message in req.get("messages", []):
                content = message.get("content")
                parts = [{"type": "text", "text": content}] if isinstance(content, str) else content or []
                summarized = []
                for part in parts:
                    if part.get("type") == "text":
                        summarized.append({"type": "text", "text": safe(part.get("text", ""))})
                    elif part.get("type") == "image_url":
                        summarized.append({"type": "image", "bytes": image_bytes(part["image_url"]["url"])})
                messages.append({"role": safe(message.get("role", "")), "content": summarized})
            sampling = {key: req[key] for key in (
                "stream", "temperature", "top_p", "max_tokens", "max_completion_tokens", "seed", "stop",
                "frequency_penalty", "presence_penalty", "n",
            ) if key in req}
            detailed = {**record, "messages": messages, "request": sampling}
            with _lock, Path(active["request_log"]).open("a") as f:
                f.write(safe(json.dumps(detailed, ensure_ascii=False)) + "\n")


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
