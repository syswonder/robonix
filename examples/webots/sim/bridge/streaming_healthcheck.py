#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Check that the Webots streaming viewer and proxy are ready."""

import argparse
import os
import socket
import time
import urllib.error
import urllib.request


def process_alive(pid: int | None) -> bool:
    """Return whether an optional helper PID is still alive."""
    if pid is None:
        return True
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def endpoints_ready(host: str, viewer_port: int, stream_port: int) -> bool:
    """Check the viewer health endpoint and the proxy listening socket."""
    try:
        with urllib.request.urlopen(
            f"http://{host}:{viewer_port}/healthz", timeout=0.5
        ) as response:
            if response.status != 204:
                return False
        with socket.create_connection((host, stream_port), timeout=0.5):
            pass
    except (OSError, urllib.error.URLError):
        return False
    return True


def wait_until_ready(args: argparse.Namespace) -> bool:
    """Wait for both helpers while failing immediately if either process exits."""
    deadline = time.monotonic() + args.timeout
    while True:
        if not process_alive(args.viewer_pid) or not process_alive(args.proxy_pid):
            return False
        if endpoints_ready(args.host, args.viewer_port, args.stream_port):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.1)


def parse_args() -> argparse.Namespace:
    """Parse helper endpoints, optional PIDs, and the readiness timeout."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--viewer-port", type=int, required=True)
    parser.add_argument("--stream-port", type=int, required=True)
    parser.add_argument("--viewer-pid", type=int)
    parser.add_argument("--proxy-pid", type=int)
    parser.add_argument("--timeout", type=float, default=0.0)
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(0 if wait_until_ready(parse_args()) else 1)
