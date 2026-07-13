#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Keep an externally controlled Webots simulation broadcasting in real time."""

import asyncio
import contextlib
import logging
import os
import time

import websockets


UPSTREAM = os.environ.get("WEBOTS_FILTER_UPSTREAM", "ws://127.0.0.1:1234")
INTERVAL = float(os.environ.get("WEBOTS_REALTIME_INTERVAL", "0.2"))

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
LOG = logging.getLogger("webots-stream-keepalive")


async def main() -> None:
    while True:
        try:
            async with websockets.connect(UPSTREAM, max_size=None) as websocket:
                await websocket.send("w3d")
                LOG.info("connected to %s", UPSTREAM)
                last_send = 0.0
                while True:
                    now = time.monotonic()
                    if now - last_send >= INTERVAL:
                        await websocket.send("timeout:1000")
                        await websocket.send("real-time:1000")
                        last_send = now
                    with contextlib.suppress(asyncio.TimeoutError):
                        await asyncio.wait_for(websocket.recv(), timeout=INTERVAL)
        except Exception as error:
            LOG.warning("reconnecting after stream error: %r", error)
            await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(main())
