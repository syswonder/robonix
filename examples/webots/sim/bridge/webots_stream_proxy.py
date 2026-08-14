#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Proxy the Webots W3D stream without unused robot-window telemetry."""

import asyncio
import contextlib
import logging
import os

import websockets


LISTEN_HOST = os.environ.get("WEBOTS_FILTER_HOST", "0.0.0.0")
LISTEN_PORT = int(os.environ.get("WEBOTS_FILTER_PORT", "1235"))
UPSTREAM = os.environ.get("WEBOTS_FILTER_UPSTREAM", "ws://127.0.0.1:1234")

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
LOG = logging.getLogger("webots-stream-proxy")


def should_drop(message: str | bytes) -> bool:
    """WebotsView ignores robot-window camera payloads in W3D mode."""
    return isinstance(message, str) and message.startswith("robot:")


async def proxy(client) -> None:
    peer = client.remote_address
    dropped_messages = 0
    dropped_bytes = 0
    LOG.info("viewer connected: %s", peer)
    try:
        async with websockets.connect(UPSTREAM, max_size=None) as upstream:

            async def client_to_upstream() -> None:
                async for message in client:
                    await upstream.send(message)

            async def upstream_to_client() -> None:
                nonlocal dropped_messages, dropped_bytes
                async for message in upstream:
                    if should_drop(message):
                        dropped_messages += 1
                        dropped_bytes += len(message)
                        continue
                    await client.send(message)

            tasks = {
                asyncio.create_task(client_to_upstream()),
                asyncio.create_task(upstream_to_client()),
            }
            done, pending = await asyncio.wait(
                tasks, return_when=asyncio.FIRST_COMPLETED
            )
            for task in pending:
                task.cancel()
            for task in pending:
                with contextlib.suppress(asyncio.CancelledError):
                    await task
            for task in done:
                task.result()
    except websockets.ConnectionClosed:
        pass
    except Exception:
        LOG.exception("proxy connection failed for %s", peer)
    finally:
        LOG.info(
            "viewer disconnected: %s; dropped %d messages (%d bytes)",
            peer,
            dropped_messages,
            dropped_bytes,
        )


async def main() -> None:
    async with websockets.serve(proxy, LISTEN_HOST, LISTEN_PORT, max_size=None):
        LOG.info("listening on ws://%s:%d -> %s", LISTEN_HOST, LISTEN_PORT, UPSTREAM)
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
