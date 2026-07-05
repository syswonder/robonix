#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
import argparse
import asyncio
import json
import sys
import time

import websockets


async def _check(host: str, port: int, timeout: float) -> int:
    url = f"ws://{host}:{port}/health"
    started = time.time()
    try:
        async with websockets.connect(url, open_timeout=timeout) as ws:
            msg = await asyncio.wait_for(ws.recv(), timeout=timeout)
    except Exception as exc:
        print(f"[mac-audio] FAIL {url}: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1

    elapsed = time.time() - started
    try:
        payload = json.loads(msg)
    except Exception:
        payload = msg
    print(f"[mac-audio] OK {url} in {elapsed:.2f}s: {payload}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Check macOS audio bridge WebSocket health.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=60101)
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    return asyncio.run(_check(args.host, args.port, args.timeout))


if __name__ == "__main__":
    raise SystemExit(main())
