#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Resolve DeepSeek from the catalog and call it directly.

Run with the package importable:

    PYTHONPATH=pylib/robonix-api python3 pylib/robonix-api/examples/llm_deepseek_smoke.py

Set DEEPSEEK_API_KEY first.  This is a manual smoke test, not a unit test: it
proves the query-then-direct-connect path against the real provider.
"""
from __future__ import annotations

import base64
import json
import os
import struct
import sys
import urllib.error
import urllib.request
import zlib
from pathlib import Path

from robonix_api.llm import Catalog, Requirements

ROOT = Path(__file__).resolve().parent.parent


def _png(width: int, height: int, rgb: tuple[int, int, int]) -> bytes:
    """A solid-colour PNG with no third-party image dependency."""
    raw = b"".join(b"\x00" + bytes(rgb) * width for _ in range(height))

    def chunk(kind: bytes, payload: bytes) -> bytes:
        body = struct.pack(">I", len(payload)) + kind + payload
        return body + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )


def main() -> int:
    catalog = Catalog.load(ROOT / "catalog.example.yaml")
    targets = catalog.resolve(
        Requirements(
            capabilities=("vision", "tools"),
            input_tokens=1000,
            max_output_tokens=256,
        ),
    )
    if not targets:
        print("no catalog model satisfies vision+tools", file=sys.stderr)
        return 1
    target = targets[0]
    print(f"resolved: {target.id} ({target.base_url}, model={target.model})")

    key = os.environ.get(target.api_key_env)
    if not key:
        print(
            f"{target.api_key_env} is not set; the resolver worked but the "
            "direct call needs a provider key",
            file=sys.stderr,
        )
        return 0

    image = base64.b64encode(_png(64, 64, (220, 20, 20))).decode("ascii")
    payload = {
        "model": target.model,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": "What color is this image? One word."},
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/png;base64,{image}"},
                    },
                ],
            }
        ],
        "max_tokens": 256,
    }
    request = urllib.request.Request(
        f"{target.base_url.rstrip('/')}/chat/completions",
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {key}",
            "Content-Type": "application/json",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            body = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        print(f"provider HTTP {error.code}: {error.read().decode()[:500]}", file=sys.stderr)
        return 1
    except urllib.error.URLError as error:
        print(f"provider connection failed: {error.reason}", file=sys.stderr)
        return 1

    message = body["choices"][0]["message"]
    print("answer:", (message.get("content") or "").strip())
    print("usage:", body.get("usage"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
