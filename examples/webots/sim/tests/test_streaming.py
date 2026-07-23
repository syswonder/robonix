#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Smoke-test custom Webots streaming ports and helper readiness."""

import asyncio
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import unittest
import urllib.request

import websockets

SIM_ROOT = Path(__file__).resolve().parents[1]
BRIDGE_ROOT = SIM_ROOT / "bridge"


def unused_ports(count: int) -> list[int]:
    """Allocate distinct ephemeral port numbers for short-lived test processes."""
    listeners = []
    try:
        for _ in range(count):
            listener = socket.socket()
            listener.bind(("127.0.0.1", 0))
            listeners.append(listener)
        return [listener.getsockname()[1] for listener in listeners]
    finally:
        for listener in listeners:
            listener.close()


class StreamingSmokeTest(unittest.IsolatedAsyncioTestCase):
    """Exercise the real viewer, proxy, and shared readiness check."""

    async def asyncSetUp(self) -> None:
        """Start a fake Webots endpoint and both helpers on custom ports."""
        self.temp_dir = tempfile.TemporaryDirectory()
        viewer_root = Path(self.temp_dir.name)
        (viewer_root / "index.html").write_text(
            '<html><body data-stream-port="__WEBOTS_STREAM_PORT__"></body></html>',
            encoding="utf-8",
        )
        self.upstream_port, self.stream_port, self.viewer_port = unused_ports(3)
        self.upstream = await websockets.serve(
            self.echo_upstream, "127.0.0.1", self.upstream_port
        )

        viewer_env = os.environ | {
            "WEBOTS_VIEWER_ROOT": str(viewer_root),
            "WEBOTS_VIEWER_HOST": "127.0.0.1",
            "WEBOTS_VIEWER_PORT": str(self.viewer_port),
            "WEBOTS_PUBLIC_STREAM_PORT": str(self.stream_port),
        }
        proxy_env = os.environ | {
            "WEBOTS_FILTER_HOST": "127.0.0.1",
            "WEBOTS_FILTER_PORT": str(self.stream_port),
            "WEBOTS_FILTER_UPSTREAM": f"ws://127.0.0.1:{self.upstream_port}",
        }
        self.viewer = subprocess.Popen(
            [sys.executable, BRIDGE_ROOT / "viewer_server.py"],
            env=viewer_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.proxy = subprocess.Popen(
            [sys.executable, BRIDGE_ROOT / "webots_stream_proxy.py"],
            env=proxy_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    async def asyncTearDown(self) -> None:
        """Stop every helper and release temporary test resources."""
        for process in (self.proxy, self.viewer):
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
        self.upstream.close()
        await self.upstream.wait_closed()
        self.temp_dir.cleanup()

    @staticmethod
    async def echo_upstream(websocket) -> None:
        """Act as a minimal Webots WebSocket endpoint for proxy forwarding."""
        async for message in websocket:
            await websocket.send(f"echo:{message}")

    def run_healthcheck(self, expected_success: bool, timeout: float = 0.0) -> None:
        """Run the production readiness probe against the custom endpoints."""
        result = subprocess.run(
            [
                sys.executable,
                BRIDGE_ROOT / "streaming_healthcheck.py",
                "--viewer-port",
                str(self.viewer_port),
                "--stream-port",
                str(self.stream_port),
                "--viewer-pid",
                str(self.viewer.pid),
                "--proxy-pid",
                str(self.proxy.pid),
                "--timeout",
                str(timeout),
            ],
            check=False,
        )
        self.assertEqual(result.returncode == 0, expected_success)

    async def test_custom_ports_and_helper_readiness(self) -> None:
        """Verify port injection, proxy forwarding, and failed-child detection."""
        self.run_healthcheck(expected_success=True, timeout=5.0)
        self.run_healthcheck(expected_success=True)
        with urllib.request.urlopen(
            f"http://127.0.0.1:{self.viewer_port}/", timeout=2
        ) as response:
            page = response.read().decode()
        self.assertIn(f'data-stream-port="{self.stream_port}"', page)

        async with websockets.connect(
            f"ws://127.0.0.1:{self.stream_port}"
        ) as websocket:
            await websocket.send("w3d")
            self.assertEqual(await websocket.recv(), "echo:w3d")

        self.proxy.terminate()
        self.proxy.wait(timeout=5)
        self.run_healthcheck(expected_success=False)


if __name__ == "__main__":
    unittest.main()
