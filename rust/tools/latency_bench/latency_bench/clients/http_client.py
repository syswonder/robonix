# SPDX-License-Identifier: MulanPSL-2.0
"""HTTP echo client for latency benchmark."""

import urllib.request


def echo(url: str, data: bytes) -> bytes:
    """POST data, receive echo in response body. Returns response bytes."""
    req = urllib.request.Request(url, data=data, method="POST")
    req.add_header("Content-Type", "application/octet-stream")
    with urllib.request.urlopen(req) as resp:
        return resp.read()


def get_url(host: str = "127.0.0.1", port: int = 18080) -> str:
    return f"http://{host}:{port}/echo"
