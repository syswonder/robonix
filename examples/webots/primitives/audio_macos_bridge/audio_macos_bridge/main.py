#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Audio primitive — macOS-bridge variant.

Same Capability surface as `audio_driver` (`robonix/primitive/audio/*`):
the difference is that mic / speaker PCM doesn't go through ALSA on
this host — it's relayed over a WebSocket to a small daemon (see
sibling `mac_server/server.py`) running on a macOS box across the LAN.

Wire format both directions: 16 kHz, mono, s16le PCM, ~100 ms chunks
(3200 bytes each). Identical to `audio_driver` so liaison's voice
pipeline never needs to know which backend is loaded.

Config (RBNX_CAP_CONFIG_JSON or env fallbacks):
  host:  IP / hostname of the macOS box running mac_server (env:
         AUDIO_BRIDGE_HOST, default 127.0.0.1).
  port:  TCP port the macOS daemon listens on (env: AUDIO_BRIDGE_PORT,
         default 60000).
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import threading
import time
from queue import Empty as QueueEmpty
from queue import Queue

from robonix_api import Primitive, Ok, Err, Deferred
from google.protobuf.empty_pb2 import Empty

audio_macos_bridge = Primitive(
    id="audio_macos_bridge",
    namespace="robonix/primitive/audio",
)
log = logging.getLogger("audio-macos-bridge")

import audio_pb2          # type: ignore  # noqa: E402  (codegen)

import websockets         # type: ignore  # noqa: E402

# Module-state populated by Driver(CMD_INIT). Both endpoints are
# probed once at init so we fail loudly when the macOS daemon is
# unreachable rather than letting the first mic/speaker call be the
# discovery moment.
bridge_host: str | None = None
bridge_port: int | None = None
speaker_lock = threading.Lock()

# Per-100-ms PCM frame: 16000 Hz * 0.1 s * 2 bytes/sample * 1 ch.
SAMPLE_RATE = 16_000
CHUNK_BYTES = SAMPLE_RATE * 2 // 10


def _ws_url(path: str) -> str:
    return f"ws://{bridge_host}:{bridge_port}{path}"


def _ws_connect(path: str, **kwargs):
    """Open a WebSocket to the macOS daemon, always DIRECTLY (never via a proxy).

    websockets >= 14 auto-reads all_proxy / http_proxy / https_proxy from the
    environment and tunnels ws:// through it. On a host with all_proxy set (this
    deploy box has all_proxy=http://127.0.0.1:7892) that silently routes the
    LAN/tailscale bridge URL into the local proxy, which can't reach the Mac →
    "timed out during opening handshake" on every connect. Passing proxy=None
    forces a direct connection regardless of the ambient proxy env."""
    return websockets.connect(_ws_url(path), proxy=None, **kwargs)


# ── streaming handlers ─────────────────────────────────────────────────────
@audio_macos_bridge.grpc("robonix/primitive/audio/mic")
def mic_stream(request, context):
    """Server-streaming mic capture — proxies frames coming off the
    macOS daemon's `/mic` WebSocket as AudioChunk messages.

    A dedicated asyncio loop runs in a worker thread and shoves binary
    frames into a thread-safe queue; the gRPC handler (sync, called by
    tonic-derived Python servicer) drains the queue. This is necessary
    because robonix-api's gRPC server doesn't expose an async dispatch
    path yet, so the websockets library — which is asyncio-only — has
    to live behind a thread boundary."""
    if bridge_host is None:
        context.abort(__import__("grpc").StatusCode.UNAVAILABLE,
                      "macos bridge not initialized — Driver(CMD_INIT) failed or never ran")
        return

    log.info("mic stream client connected → relaying %s", _ws_url("/mic"))
    q: Queue = Queue(maxsize=64)
    stop = threading.Event()

    async def pump() -> None:
        try:
            async with _ws_connect("/mic", max_size=None) as ws:
                while not stop.is_set():
                    msg = await ws.recv()
                    if isinstance(msg, str):
                        # control plane is unused on /mic; ignore stray text frames
                        continue
                    try:
                        q.put_nowait(msg)
                    except Exception:  # queue full → drop
                        pass
        except Exception as e:  # noqa: BLE001
            log.warning("mic ws closed: %s", e)
        finally:
            q.put_nowait(None)

    def loop_runner() -> None:
        asyncio.run(pump())

    t = threading.Thread(target=loop_runner, name="audio-bridge-mic", daemon=True)
    t.start()

    seq = 0
    try:
        while context.is_active():
            try:
                frame = q.get(timeout=0.5)
            except QueueEmpty:
                continue
            if frame is None:
                break
            yield audio_pb2.AudioChunk(
                timestamp_ns=time.time_ns(),
                data=bytes(frame),
                sequence=seq,
                duration_s=len(frame) / (SAMPLE_RATE * 2.0),
            )
            seq += 1
    finally:
        stop.set()
        log.info("mic stream client disconnected")


@audio_macos_bridge.grpc("robonix/primitive/audio/speaker")
def speaker_stream(request_iterator, context):
    """Client-streaming playback — pipes incoming AudioChunk.data to
    the macOS daemon's `/speaker` WebSocket. Same threading dance as
    `mic_stream` but in reverse: the gRPC iterator hands frames to a
    queue, the asyncio coroutine drains the queue and writes them to
    the WebSocket.

    The gRPC call must not return immediately after the last frame is sent:
    macOS playback continues asynchronously inside sounddevice. Holding the
    speaker lock for the estimated PCM duration keeps consecutive TTS
    utterances serialized even when Liaison speaks sentence by sentence."""
    if bridge_host is None:
        context.abort(__import__("grpc").StatusCode.UNAVAILABLE,
                      "macos bridge not initialized")
        return Empty()

    log.info("speaker stream client waiting for playback lock")
    with speaker_lock:
        log.info("speaker stream client connected → relaying %s", _ws_url("/speaker"))
        q: Queue = Queue(maxsize=64)
        done = threading.Event()

        async def pump() -> None:
            try:
                async with _ws_connect("/speaker", max_size=None) as ws:
                    while True:
                        frame = await asyncio.get_event_loop().run_in_executor(None, q.get)
                        if frame is None:
                            break
                        await ws.send(frame)
            except Exception as e:  # noqa: BLE001
                log.warning("speaker ws closed: %s", e)
            finally:
                done.set()

        def loop_runner() -> None:
            asyncio.run(pump())

        t = threading.Thread(target=loop_runner, name="audio-bridge-speaker", daemon=True)
        t.start()

        total_bytes = 0
        try:
            for chunk in request_iterator:
                if chunk.data:
                    frame = bytes(chunk.data)
                    total_bytes += len(frame)
                    q.put(frame)
        finally:
            q.put(None)
            # Give the websocket a moment to drain; don't block forever in
            # case the server side already went away.
            done.wait(timeout=10.0)
            playback_s = total_bytes / float(SAMPLE_RATE * 2)
            if playback_s > 0:
                time.sleep(min(playback_s + 0.15, 30.0))
            log.info("speaker stream client disconnected")
    return Empty()


# ── device list / select ───────────────────────────────────────────────────
#
# Both forward to the macOS daemon's existing /devices and /set_device
# WebSocket endpoints (one-shot JSON request/response). The daemon already
# tracks current_input/output_device; the bridge just shape-shifts JSON
# into the AudioDevice / Select…Response protobuf types so consumers
# (rbnx chat audio settings page) see the same surface as audio_driver.

def _ws_request(path: str, body: object | None = None, timeout_s: float = 3.0):
    """Open a one-shot WS to the mac_server, optionally send a JSON body,
    receive one JSON response, close. Sync wrapper around the asyncio API
    so the gRPC servicer thread doesn't have to learn asyncio."""
    async def go():
        async with _ws_connect(path, open_timeout=timeout_s) as ws:
            if body is not None:
                await ws.send(json.dumps(body))
            msg = await asyncio.wait_for(ws.recv(), timeout=timeout_s)
            return json.loads(msg) if isinstance(msg, str) else None
    return asyncio.run(go())


@audio_macos_bridge.grpc("robonix/primitive/audio/list_devices")
def list_devices(request, context):
    if bridge_host is None:
        context.abort(__import__("grpc").StatusCode.UNAVAILABLE,
                      "macos bridge not initialized")
        return audio_pb2.ListAudioDevices_Response()
    try:
        payload = _ws_request("/devices")
    except Exception as e:  # noqa: BLE001
        context.abort(__import__("grpc").StatusCode.UNAVAILABLE,
                      f"mac_server /devices unreachable: {e}")
        return audio_pb2.ListAudioDevices_Response()

    devs = []
    in_default = payload.get("input_default")
    out_default = payload.get("output_default")
    in_cur = payload.get("input_current")
    out_cur = payload.get("output_current")
    for d in payload.get("devices", []):
        in_ch = int(d.get("max_input_channels", 0))
        out_ch = int(d.get("max_output_channels", 0))
        if in_ch == 0 and out_ch == 0:
            continue
        kind = "duplex" if (in_ch > 0 and out_ch > 0) else "input" if in_ch else "output"
        is_default = (kind in ("input", "duplex") and d["id"] == in_default) or \
                     (kind in ("output", "duplex") and d["id"] == out_default)
        name = d.get("name", "")
        # macOS BT-HFP devices crash sd.RawInputStream at 16 kHz; flag them
        # so the picker can warn or auto-skip without re-implementing the
        # heuristic in every consumer.
        note = ""
        low = name.lower()
        if any(k in low for k in ("airpods", "bluetooth", "iphone", "ipad")):
            note = "bluetooth"
        devs.append(audio_pb2.AudioDevice(
            id=str(d["id"]),
            name=name,
            kind=kind,
            is_default=is_default,
            channels=max(in_ch, out_ch),
            note=note,
        ))
    return audio_pb2.ListAudioDevices_Response(
        devices=devs,
        current_input_id="" if in_cur is None else str(in_cur),
        current_output_id="" if out_cur is None else str(out_cur),
    )


@audio_macos_bridge.grpc("robonix/primitive/audio/select_device")
def select_device(request, context):
    if bridge_host is None:
        context.abort(__import__("grpc").StatusCode.UNAVAILABLE,
                      "macos bridge not initialized")
        return audio_pb2.SelectAudioDevice_Response()
    kind = (request.kind or "").lower()
    if kind not in ("input", "output"):
        return audio_pb2.SelectAudioDevice_Response(
            ok=False, error=f"kind must be 'input' or 'output', got '{kind}'")
    # mac_server expects integer ids; normalise back from the wire string.
    raw_id = request.id
    sent_id: int | str | None
    if raw_id == "":
        sent_id = None  # mac_server falls back to OS default
    else:
        try:
            sent_id = int(raw_id)
        except ValueError:
            sent_id = raw_id  # forward as string; mac_server will reject if bad
    body = {kind: sent_id}
    try:
        payload = _ws_request("/set_device", body)
    except Exception as e:  # noqa: BLE001
        return audio_pb2.SelectAudioDevice_Response(ok=False, error=f"ws: {e}")
    return audio_pb2.SelectAudioDevice_Response(
        ok=bool(payload.get("ok", False)),
        error=payload.get("error", ""),
    )


# ── driver-init lifecycle ──────────────────────────────────────────────────
@audio_macos_bridge.on_init
def init(cfg):
    """Read host/port from cfg + env, probe the macOS daemon's `/health`
    endpoint once. Refuse to come up if it's unreachable — atlas defers
    instead of advertising dead interfaces, and consumers see a clear
    error rather than mysteriously empty mic streams."""
    global bridge_host, bridge_port

    bridge_host = (cfg.get("host") or os.environ.get("AUDIO_BRIDGE_HOST", "127.0.0.1")).strip()
    bridge_port = int(cfg.get("port") or os.environ.get("AUDIO_BRIDGE_PORT", "60000"))

    log.info("connecting probe → ws://%s:%d/health", bridge_host, bridge_port)

    async def probe() -> str | None:
        try:
            async with _ws_connect("/health", open_timeout=3.0) as ws:
                msg = await asyncio.wait_for(ws.recv(), timeout=3.0)
                return msg if isinstance(msg, str) else "<binary>"
        except Exception as e:  # noqa: BLE001
            return f"error: {e}"

    result = asyncio.run(probe())
    if result is None or (isinstance(result, str) and result.startswith("error:")):
        return Err(
            f"macos bridge unreachable at ws://{bridge_host}:{bridge_port}/health "
            f"({result}). Start `mac_server/server.py` on the macOS host first."
        )
    log.info("bridge healthy: %s", result)
    return Ok()


def main() -> int:
    audio_macos_bridge.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
