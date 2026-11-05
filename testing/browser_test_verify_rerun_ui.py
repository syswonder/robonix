# SPDX-License-Identifier: MulanPSL-2.0
"""Run explicitly with testing/requirements-browser.txt installed."""
import tempfile
import unittest
import argparse
import asyncio
import json
import sys
import threading
import socket
import struct
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rerun as rr
from PIL import Image, ImageDraw

from verify_rerun_ui import capture, run, validate_pixels, validate_recording


class RerunUIEvidenceTests(unittest.TestCase):
    def test_focus_retries_only_connection_resets(self):
        from playwright.async_api import Error, async_playwright

        for resets, status, calls, succeeds in ((1, 200, 2, True),
                                                (3, 200, 3, False),
                                                (0, 503, 1, False)):
            with self.subTest(resets=resets, status=status):
                requests = []

                class Handler(BaseHTTPRequestHandler):
                    def do_POST(self):
                        size = int(self.headers["Content-Length"])
                        requests.append(json.loads(self.rfile.read(size)))
                        if len(requests) <= resets:
                            self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                                                       struct.pack("ii", 1, 0))
                            self.close_connection = True
                            return
                        body = b'{"ok":true}'
                        self.send_response(status)
                        self.send_header("Content-Length", str(len(body)))
                        self.end_headers()
                        self.wfile.write(body)

                    def log_message(self, *args):
                        pass

                server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
                thread = threading.Thread(target=server.serve_forever, daemon=True)
                thread.start()

                async def check():
                    async with async_playwright() as api:
                        origin = f"http://127.0.0.1:{server.server_port}"
                        request = await api.request.new_context(base_url=origin)
                        canvas = SimpleNamespace(wait_for=AsyncMock(), screenshot=AsyncMock())
                        frame = SimpleNamespace(locator=lambda _: canvas,
                            evaluate=AsyncMock(side_effect=[None, {"id": "recording"}]))
                        download = SimpleNamespace(save_as=AsyncMock())
                        downloaded = asyncio.get_running_loop().create_future()
                        downloaded.set_result(download)
                        pending = AsyncMock()
                        pending.__aenter__.return_value = SimpleNamespace(value=downloaded)
                        page = SimpleNamespace(url=origin, request=request,
                            expect_download=lambda **_: pending, screenshot=AsyncMock())
                        proof = {"object": {"id": "chair"}, "object_color": [50, 100, 150]}
                        try:
                            with (tempfile.TemporaryDirectory() as directory,
                                  patch("verify_rerun_ui.api_body", AsyncMock(return_value=b'{}')),
                                  patch("verify_rerun_ui.recording_proof", AsyncMock(return_value=proof)),
                                  patch("verify_rerun_ui.validate_pixels", return_value={})):
                                if succeeds:
                                    self.assertEqual(await capture(page, frame, Path(directory),
                                                                   "before", "3d"), proof)
                                else:
                                    with self.assertRaises((Error, RuntimeError)):
                                        await capture(page, frame, Path(directory), "before", "3d")
                        finally:
                            await request.dispose()

                try:
                    asyncio.run(check())
                    self.assertEqual(requests, [{"object_id": "chair"}] * calls)
                finally:
                    server.shutdown()
                    server.server_close()
                    thread.join()

    def test_real_browser_pageerror_is_recorded_and_prevents_child(self):
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                body = (b'<iframe id="v" src="/broken"></iframe>' if self.path == "/" else
                        b'<script>throw new Error("diagnostic-pageerror")</script>')
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                output = Path(directory)
                marker = output / "child-ran"
                args = argparse.Namespace(base_url=f"http://127.0.0.1:{server.server_port}",
                                          output_dir=output, timeout=1,
                                          command=[sys.executable, "-c",
                                                   f"from pathlib import Path; Path({str(marker)!r}).touch()"])
                self.assertEqual(asyncio.run(run(args)), 1)
                summary = json.loads((output / "summary.json").read_text())
                self.assertTrue(any("diagnostic-pageerror" in e for e in summary["browser_errors"]))
                self.assertFalse(marker.exists())
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_blank_canvas_fails_even_with_canvas_present(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "blank.png"
            Image.new("RGB", (1440, 1000), (16, 18, 24)).save(path)
            with self.assertRaisesRegex(RuntimeError, "not visible"):
                validate_pixels(path)
            image = Image.open(path)
            draw = ImageDraw.Draw(image)
            draw.rectangle((300, 300, 900, 700), fill=(104, 113, 133))
            draw.rectangle((600, 100, 620, 120), fill=(255, 166, 54))
            image.save(path)
            self.assertGreater(validate_pixels(path)["robot_pixels"], 8)

    def test_recording_requires_real_geometry_values_and_matching_id(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene.rrd"
            recording = rr.RecordingStream("robonix-scene")
            recording.save(str(path))
            objects = [{"id": "chair", "settled": True}]
            recording.log("/map/floor", rr.Mesh3D(vertex_positions=[], triangle_indices=[]), static=True)
            recording.flush()
            with self.assertRaisesRegex(RuntimeError, "missing geometry"):
                validate_recording(path, "3d", recording.get_recording_id(), objects)
            recording.log("/map/floor", rr.Mesh3D(
                vertex_positions=[[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                triangle_indices=[[0, 1, 2]]), static=True)
            recording.log("/map/objects/sem_pcd/chair", rr.Points3D([[1, 1, 1]], colors=[[50, 200, 100]]), static=True)
            recording.log("/map/robot/heading", rr.Arrows3D(vectors=[[0.45, 0, 0]]), static=True)
            recording.flush()
            self.assertEqual(len(validate_recording(path, "3d", recording.get_recording_id(), objects)["geometry_columns"]), 4)
            with self.assertRaisesRegex(RuntimeError, "wrong Scene recording"):
                validate_recording(path, "3d", "not-the-active-recording", objects)
            with self.assertRaisesRegex(RuntimeError, "does not match"):
                validate_recording(path, "3d", recording.get_recording_id(), [])
            for robot in ({"label": "robot"}, {"is_robot": True}, {"attributes": {"is_robot": True}}):
                with self.assertRaisesRegex(RuntimeError, "does not match"):
                    validate_recording(path, "3d", recording.get_recording_id(), [{**objects[0], **robot}])
            recording.disconnect()


if __name__ == "__main__":
    unittest.main()
