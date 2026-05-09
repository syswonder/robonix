#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Audio bridge daemon — runs on the macOS host that physically owns
the mic + speakers, exposes both as WebSocket endpoints over LAN to
the Linux side of the robonix stack (see
`../audio_macos_bridge/main.py`).

Three endpoints, all `ws://0.0.0.0:60000/...`:

  /health    server pings back a one-line JSON status when a client
             connects. Used by the Linux primitive's Driver(CMD_INIT)
             probe.
  /mic       server streams 16 kHz / mono / s16le PCM frames in
             ~100 ms chunks (3200 bytes). One concurrent client is
             enough — CoreAudio supports multiple capture handles
             but the typical robonix flow is "liaison opens one
             stream during a voice turn".
  /speaker   server consumes 16 kHz / mono / s16le PCM chunks pushed
             by the client and feeds them straight into the default
             audio output device.

Dependencies (pip):
  pip install sounddevice websockets

Run:
  python3 server.py [--host 0.0.0.0] [--port 60000] [--input-device N] [--output-device N]

Use `python3 server.py --list-devices` to see CoreAudio device IDs
when the defaults aren't what you want.

Local-only by convention; this script and the sibling
audio_macos_bridge package are git-ignored at the repo root.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import platform
import queue as stdlib_queue
import sys
import threading

import sounddevice as sd       # type: ignore
import websockets              # type: ignore

SAMPLE_RATE = 16_000
CHANNELS = 1
DTYPE = "int16"
# 100 ms frames — same chunking the Linux side advertises in the
# AudioChunk wire format. Lower latency than 200 ms+ at no cost; the
# WebSocket per-frame overhead is negligible at <50 fps.
FRAME_SAMPLES = SAMPLE_RATE // 10
FRAME_BYTES = FRAME_SAMPLES * 2  # int16 mono

log = logging.getLogger("mac-bridge")


def list_devices() -> None:
    print("=" * 72)
    print(f"sounddevice on {platform.system()} {platform.release()}")
    print("=" * 72)
    print(sd.query_devices())
    defaults = sd.default.device
    print()
    print(f"defaults: input={defaults[0]}  output={defaults[1]}")


# Bluetooth audio devices on macOS expose two profiles: A2DP (high-quality
# stereo, no mic) and HFP/HSC (low-quality bidirectional, 8/16 kHz). When
# you open the device with `sd.RawInputStream(samplerate=16000, channels=1)`
# CoreAudio tries the A2DP profile first and gets back paInternalError /
# AUHAL "Invalid Property Value" because A2DP doesn't support input. Going
# through HFP requires extra steps (forcing the SCO profile). Cheapest fix:
# refuse to open Bluetooth devices in auto-pick mode and force the user to
# `--input-device <id>` if they really want one.
_BLUETOOTH_NAME_HINTS: tuple[str, ...] = (
    "airpods", "bluetooth", "iphone", "ipad",
)


def _is_likely_bluetooth(dev: dict) -> bool:
    name = str(dev.get("name", "")).lower()
    return any(h in name for h in _BLUETOOTH_NAME_HINTS)


def pick_input_device(explicit: int | None) -> int | None:
    """If `--input-device` was given, honour it. Otherwise return
    sd.default.device[0] unless that's a Bluetooth device, in which case
    walk the table and pick the first non-Bluetooth input. Returning
    None means "let CoreAudio's default through unchanged"."""
    if explicit is not None:
        return explicit
    default = sd.default.device[0]
    if default is None or default < 0:
        return None
    try:
        info = sd.query_devices(default)
    except Exception:
        return None
    if not _is_likely_bluetooth(info):
        return None  # default is fine; let it through
    log.warning(
        "default input device #%d (%s) looks like Bluetooth — "
        "16 kHz mono RawInputStream tends to fail with paInternalError "
        "on the BT-HFP profile. Searching for a wired alternative…",
        default, info["name"],
    )
    for i, d in enumerate(sd.query_devices()):
        if d["max_input_channels"] > 0 and not _is_likely_bluetooth(d):
            log.info("auto-picked input device #%d (%s)", i, d["name"])
            return i
    return None  # nothing better — fall back to whatever default is


# ── /mic ────────────────────────────────────────────────────────────────────
async def serve_mic(ws, input_device: int | None) -> None:
    """Open a CoreAudio capture stream, push raw int16 frames over
    the WebSocket as binary messages until the client disconnects.

    `sd.RawInputStream` writes into a callback on a CoreAudio thread;
    we hand frames over to the asyncio loop via run_coroutine_threadsafe.
    Backpressure: drop frames if the websocket queue is full — better
    than blocking the audio thread (which would underrun the device)."""
    log.info("mic client connected from %s", ws.remote_address)
    loop = asyncio.get_event_loop()
    # Threading layout: CoreAudio fires `callback` on its own thread to
    # deliver capture frames; the websocket send happens on the asyncio
    # loop. A stdlib `queue.Queue` is the safe bridge across the two —
    # asyncio.Queue is single-thread-only and produced the
    # `coroutine 'Queue.put' was never awaited` runtime warning when
    # callers tried to drive it from CoreAudio's thread.
    q: stdlib_queue.Queue = stdlib_queue.Queue(maxsize=64)

    def callback(indata, frames, time_info, status):
        if status:
            log.debug("input status: %s", status)
        try:
            q.put_nowait(bytes(indata))
        except stdlib_queue.Full:
            pass  # drop on overflow rather than block CoreAudio

    stream = sd.RawInputStream(
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        dtype=DTYPE,
        blocksize=FRAME_SAMPLES,
        device=input_device,
        callback=callback,
    )
    stream.start()
    try:
        while True:
            frame = await loop.run_in_executor(None, q.get)
            await ws.send(frame)
    except websockets.ConnectionClosed:
        log.info("mic client disconnected")
    finally:
        stream.stop()
        stream.close()


# ── /speaker ───────────────────────────────────────────────────────────────
async def serve_speaker(ws, output_device: int | None) -> None:
    """Receive int16 PCM frames over WebSocket, push them into a
    sounddevice OutputStream. Frames are queued so brief network
    jitter doesn't underrun the audio output."""
    log.info("speaker client connected from %s", ws.remote_address)
    loop = asyncio.get_event_loop()

    # Bytearray buffer + lock — the previous queue-of-opaque-chunks
    # design dropped any bytes beyond `len(outdata)` per callback,
    # so an 8 KiB liaison TTS chunk played only the first 3.2 KiB
    # and silently discarded the rest of the utterance.
    buf = bytearray()
    buf_lock = threading.Lock()

    counters = {"cb": 0, "data": 0, "underrun": 0}

    def callback(outdata, frames, time_info, status):
        counters["cb"] += 1
        if status:
            log.debug("output status: %s", status)
        n = len(outdata)
        with buf_lock:
            avail = min(n, len(buf))
            if avail:
                outdata[:avail] = bytes(buf[:avail])
                del buf[:avail]
                counters["data"] += 1
            else:
                counters["underrun"] += 1
            if avail < n:
                outdata[avail:] = b"\x00" * (n - avail)

    # Resolve the actual device id sounddevice will use, so the log
    # tells us *which* speaker is being driven (NoMachine / virtual
    # devices show up alongside the built-in MacBook one and which one
    # CoreAudio picks as default isn't always obvious).
    dev_id = output_device
    if dev_id is None:
        dev_id = sd.default.device[1]
    dev_info = sd.query_devices(dev_id) if dev_id is not None else None
    log.info("speaker stream opening on device id=%s name=%s",
             dev_id,
             dev_info["name"] if dev_info else "(unknown)")

    stream = sd.RawOutputStream(
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        dtype=DTYPE,
        blocksize=FRAME_SAMPLES,
        device=output_device,
        callback=callback,
    )
    stream.start()
    log.info("speaker stream active=%s sr=%s ch=%s dtype=%s",
             stream.active, stream.samplerate, stream.channels, stream.dtype)
    try:
        async for msg in ws:
            if isinstance(msg, bytes):
                with buf_lock:
                    buf.extend(msg)
        # Hold the device open until the buffer drains, otherwise the
        # tail of the utterance gets cut off when the WS closes.
        while True:
            with buf_lock:
                if not buf:
                    break
            await asyncio.sleep(0.05)
    except websockets.ConnectionClosed:
        log.info("speaker client disconnected")
    finally:
        stream.stop()
        stream.close()
        log.info(
            "speaker stream closed: cb=%d data=%d underrun=%d",
            counters["cb"], counters["data"], counters["underrun"],
        )


# ── /health ────────────────────────────────────────────────────────────────
async def serve_health(ws) -> None:
    payload = {
        "ok": True,
        "platform": platform.system(),
        "sample_rate": SAMPLE_RATE,
        "frame_bytes": FRAME_BYTES,
        "default_input": sd.default.device[0],
        "default_output": sd.default.device[1],
    }
    await ws.send(json.dumps(payload))


# ── /devices ──────────────────────────────────────────────────────────────
# Same JSON shape as server_web.py's /devices so the bridge's gRPC
# ListAudioDevices handler doesn't have to branch on which mac_server
# variant is running. Headless server can't change the active device
# at runtime (it's pinned to --input-device/--output-device CLI flags),
# so /set_device returns ok=false with a hint.
async def serve_devices(ws, input_dev, output_dev) -> None:
    devs = sd.query_devices()
    payload = {
        "input_default": sd.default.device[0],
        "output_default": sd.default.device[1],
        "input_current": input_dev,
        "output_current": output_dev,
        "devices": [
            {
                "id": i,
                "name": d["name"],
                "max_input_channels": d["max_input_channels"],
                "max_output_channels": d["max_output_channels"],
            }
            for i, d in enumerate(devs)
        ],
    }
    await ws.send(json.dumps(payload))


async def serve_set_device(ws) -> None:
    try:
        await ws.recv()  # consume the request body but ignore it
    except Exception:  # noqa: BLE001
        pass
    await ws.send(json.dumps({
        "ok": False,
        "error": "headless server.py does not support runtime device changes; "
                 "restart with --input-device / --output-device or use "
                 "server_web.py for live switching",
    }))


# ── dispatch ───────────────────────────────────────────────────────────────
async def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=60_000)
    parser.add_argument("--input-device", type=int, default=None,
                        help="CoreAudio input device id (see --list-devices)")
    parser.add_argument("--output-device", type=int, default=None,
                        help="CoreAudio output device id")
    parser.add_argument("--list-devices", action="store_true")
    parser.add_argument("--log", default="info")
    args = parser.parse_args()

    logging.basicConfig(
        level=args.log.upper(),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    if args.list_devices:
        list_devices()
        return 0

    # Resolve device ids once at boot so subsequent /mic, /speaker calls
    # all hit the same hardware regardless of OS-level default flips
    # (most commonly: AirPods (dis)connecting).
    input_dev = pick_input_device(args.input_device)
    output_dev = args.output_device  # speaker side has no BT-specific quirks
    log.info("resolved devices: input=%s output=%s", input_dev, output_dev)

    async def handler(ws):
        path = ws.request.path if hasattr(ws, "request") else getattr(ws, "path", "")
        if path == "/mic":
            await serve_mic(ws, input_dev)
        elif path == "/speaker":
            await serve_speaker(ws, output_dev)
        elif path == "/health":
            await serve_health(ws)
        elif path == "/devices":
            await serve_devices(ws, input_dev, output_dev)
        elif path == "/set_device":
            await serve_set_device(ws)
        else:
            await ws.close(code=1008, reason=f"unknown path {path!r}")

    log.info("listening on ws://%s:%d  in_dev=%s  out_dev=%s",
             args.host, args.port, args.input_device, args.output_device)
    async with websockets.serve(handler, args.host, args.port, max_size=None):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        sys.exit(asyncio.run(main()))
    except KeyboardInterrupt:
        sys.exit(0)
