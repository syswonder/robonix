# SPDX-License-Identifier: MulanPSL-2.0
"""Model transcripts captured through the bench's HTTP proxy."""

from __future__ import annotations

import base64
import importlib
import json
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

from testing.model_bench import bench

with patch.dict("sys.modules", {"bench": bench}):
    proxy = importlib.import_module("testing.model_bench.proxy")


class ModelBenchProxyTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.world = Path(temp.name)
        self.patch = patch.object(proxy, "WORLD", self.world, create=True)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        (self.world / "state.json").write_text('{"case": "case"}')
        self.responses = []
        responses = self.responses
        self.on_model_request = None
        fixture = self

        class Model(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_POST(self):
                self.rfile.read(int(self.headers["Content-Length"]))
                if fixture.on_model_request:
                    fixture.on_model_request()
                status, body, stream = responses.pop(0)
                self.send_response(status)
                self.send_header("Content-Type", "text/event-stream" if stream else "application/json")
                if stream == "broken":
                    self.send_header("Transfer-Encoding", "chunked")
                self.end_headers()
                if stream == "broken":
                    self.wfile.write(f"{len(body):x}\r\n".encode() + body + b"\r\ninvalid chunk\r\n")
                    return
                # Split UTF-8 and SSE events across writes.
                for i in range(0, len(body), 7):
                    self.wfile.write(body[i:i + 7])
                    self.wfile.flush()

        model = self.serve(Model)
        env = self.world / "endpoint.env"
        env.write_text(f"VLM_BASE_URL=http://127.0.0.1:{model.server_port}\nVLM_API_KEY=secret-test-key\n")
        (self.world / "target.json").write_text(json.dumps({"env_file": str(env), "model": "fake"}))
        server = self.serve(proxy.Handler)
        self.url = f"http://127.0.0.1:{server.server_port}/v1/chat/completions"

    def serve(self, handler):
        # Reverse DNS can stall local server setup on macOS.
        with patch("socket.getfqdn", return_value="localhost"):
            server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(thread.join)
        self.addCleanup(server.shutdown)
        return server

    def request(self, messages, stream=False, **fields):
        req = urllib.request.Request(self.url, json.dumps({"messages": messages, "stream": stream, **fields}).encode(),
                                     headers={"Authorization": "Bearer incoming-secret"})
        try:
            with urllib.request.urlopen(req, timeout=5) as response:
                return response.read()
        except urllib.error.HTTPError as error:
            with error:
                return error.read()

    def test_rounds_in_order_with_verbatim_replies_usage_and_image_counts(self):
        replies = ['  {"rtdl": "first"}\n', 'second\n\t\"raw\" 🐈']
        usage = {"prompt_tokens": 12, "completion_tokens": 8, "prompt_tokens_details": {"cached_tokens": 0}}
        plain = json.dumps({"choices": [{"message": {"content": replies[0]}}], "usage": usage}).encode()
        streamed = b"".join(("data: " + json.dumps({"choices": [{"delta": {"content": part}}]},
                                                  ensure_ascii=False) + "\r\n\r\n").encode()
                            for part in [replies[1][:3], replies[1][3:]])
        streamed += ("data: " + json.dumps({"choices": [], "usage": usage}) + "\n\ndata: [DONE]\n\n").encode()
        self.responses.extend([(200, plain, False), (200, streamed, True)])
        messages = [{"role": "user", "content": [{"type": "text", "text": "instruction"},
                    {"type": "image_url", "image_url": {"url": "data:image/png;base64,PRIVATE_IMAGE_ONE"}},
                    {"type": "image_url", "image_url": {"url": "https://example.test/PRIVATE_IMAGE_TWO"}}]}]
        self.assertEqual(self.request(messages), plain)
        self.assertEqual(self.request(messages, stream=True), streamed)
        records = bench.lines_of(self.world / "vlm.jsonl", "case")
        self.assertEqual([r["reply"] for r in records], replies)
        self.assertLessEqual(records[0]["end"], records[1]["start"])
        for r in records:
            self.assertLessEqual(r["start"], r["end"])
            self.assertEqual(r["images"], 2)
            self.assertIsNone(r["error"])
            self.assertEqual(r["usage"], {"prompt_tokens": 12, "completion_tokens": 8, "cached_tokens": 0})
        stored = (self.world / "vlm.jsonl").read_text()
        for forbidden in ("PRIVATE_IMAGE", "secret-test-key", "incoming-secret", "Authorization", "instruction"):
            self.assertNotIn(forbidden, stored)

    def test_request_log_summarizes_inputs_and_pins_state_before_upstream_request(self):
        request_log = self.world / "requests.jsonl"
        (self.world / "state.json").write_text(json.dumps({"case": "case", "request_log": str(request_log)}))
        self.on_model_request = lambda: (self.world / "state.json").write_text(json.dumps({
            "case": "next-case", "request_log": str(self.world / "next-requests.jsonl"),
        }))
        reply = '  {"rtdl":"verbatim"}\n 🐈'
        body = json.dumps({"choices": [{"message": {"content": reply}}]}).encode()
        self.responses.append((200, body, False))
        image = b"\x00\x01\x02\x03\xff"
        inline = "data:image/png;base64," + base64.b64encode(image).decode()
        remote = "https://example.test/private-image?token=hidden"
        messages = [
            {"role": "system", "content": "system instruction"},
            {"role": "user", "content": [{"type": "text", "text": "find the cat"},
                {"type": "image_url", "image_url": {"url": inline}},
                {"type": "image_url", "image_url": {"url": remote}},
                {"type": "image_url", "image_url": {"url": "data:image/png;base64,INVALID!"}}]},
            {"role": "assistant", "content": "earlier reply"},
        ]
        self.assertEqual(self.request(messages, temperature=0.25, max_tokens=100,
                                      api_key="body-secret", base_url="https://private.test", model="private-model"), body)
        diagnostic = bench.lines_of(self.world / "vlm.jsonl", "case")[0]
        detailed = bench.lines_of(request_log, "case")[0]
        self.assertEqual(detailed["reply"], reply)
        self.assertEqual(detailed["images"], 3)
        self.assertEqual(detailed["messages"], [
            {"role": "system", "content": [{"type": "text", "text": "system instruction"}]},
            {"role": "user", "content": [{"type": "text", "text": "find the cat"},
                {"type": "image", "bytes": len(image)}, {"type": "image", "bytes": None},
                {"type": "image", "bytes": None}]},
            {"role": "assistant", "content": [{"type": "text", "text": "earlier reply"}]},
        ])
        self.assertEqual(detailed["request"], {"stream": False, "temperature": 0.25, "max_tokens": 100})
        self.assertEqual({key: detailed[key] for key in diagnostic}, diagnostic)
        self.assertFalse((self.world / "next-requests.jsonl").exists())
        stored = request_log.read_text()
        for forbidden in (inline, remote, "INVALID!", "secret-test-key", "incoming-secret", "Authorization",
                          "body-secret", "private.test", "private-model", "api_key", "base_url"):
            self.assertNotIn(forbidden, stored)
        self.assertNotIn("system instruction", (self.world / "vlm.jsonl").read_text())

    def test_http_and_stream_errors_keep_partial_reply_and_redact_input(self):
        image = "data:image/png;base64,PRIVATE_IMAGE"
        self.responses.extend([(400, f"invalid image {image} secret-test-key".encode(), False),
                               (400, json.dumps({"error": image + " secret-test-key"}).replace("/", "\\/")
                                .replace("secret", "\\u0073ecret").encode(), False)])
        self.request([{"role": "user", "content": [{"type": "image_url", "image_url": {"url": image}}]}])
        self.request([{"role": "user", "content": [{"type": "image_url", "image_url": {"url": image}}]}])
        partial = b'data: {"choices":[{"delta":{"content":"partial raw"}}]}\n\n'
        self.responses.extend([(200, partial + b'data: {"error":{"message":"stream failed"}}\n\n', True),
                               (200, partial, True), (200, partial, "broken")])
        self.request([], stream=True)
        self.request([], stream=True)
        self.request([], stream=True)
        records = bench.lines_of(self.world / "vlm.jsonl", "case")
        self.assertEqual(records[0]["error"], "invalid image [redacted] [redacted]")
        self.assertEqual(records[0]["reply"], "")
        self.assertNotIn("PRIVATE_IMAGE", records[1]["error"])
        self.assertNotIn("secret-test-key", records[1]["error"])
        self.assertEqual([r["reply"] for r in records[2:]], ["partial raw"] * 3)
        self.assertIn("stream failed", records[2]["error"])
        self.assertIn("without [DONE]", records[3]["error"])
        self.assertEqual(records[4]["status"], 502)
        self.assertIsNotNone(records[4]["error"])


if __name__ == "__main__":
    unittest.main()
