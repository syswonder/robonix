#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Web-UI variant of `server.py`.

Same WebSocket protocol (`/mic`, `/speaker`, `/health`) as the headless
server, plus a small in-process HTTP+WS UI:

  http://<host>:<port+1>/     single-page debug UI
                              (device pickers, live VU meter, log panel)
  ws://<host>:<port>/devices  JSON list of CoreAudio devices
  ws://<host>:<port>/vu       server-stream peak RMS values (50 ms tick)
  ws://<host>:<port>/log      newline-delimited live log feed
  ws://<host>:<port>/set_device  one-shot setter; client sends a JSON
                              `{"input": <id|null>, "output": <id|null>}`
                              message and the server updates the
                              shared state used by /mic and /speaker.

Default port: WS 60000, HTTP 60001. The `websockets` library is the
only runtime dep beyond `sounddevice`; HTTP is served via stdlib
`http.server` on a sibling port so the front-end is a single static
HTML page baked into this file.

Run on the macOS box (in a real desktop session — open the UI URL in
Safari / Chrome on the same Mac):

    cd ~/robonix-scripts/mac_server
    . .venv/bin/activate
    python3 server_web.py --port 60000
    # then open http://localhost:60001/

Run with --headless to skip the HTTP UI; equivalent to `server.py`.
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
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import sounddevice as sd       # type: ignore
import websockets              # type: ignore

SAMPLE_RATE = 16_000
CHANNELS = 1
DTYPE = "int16"
FRAME_SAMPLES = SAMPLE_RATE // 10
FRAME_BYTES = FRAME_SAMPLES * 2

log = logging.getLogger("mac-bridge-web")

state_lock = threading.Lock()
state = {
    "input_device": None,
    "output_device": None,
    "mic_clients": 0,
    "speaker_clients": 0,
}


def _state(key):
    with state_lock:
        return state[key]


def _set_state(key, value):
    with state_lock:
        state[key] = value


# ── /health ────────────────────────────────────────────────────────────────
async def serve_health(ws) -> None:
    in_dev = _state("input_device")
    out_dev = _state("output_device")
    payload = {
        "ok": True,
        "platform": platform.system(),
        "sample_rate": SAMPLE_RATE,
        "frame_bytes": FRAME_BYTES,
        "input_device": in_dev if in_dev is not None else sd.default.device[0],
        "output_device": out_dev if out_dev is not None else sd.default.device[1],
    }
    await ws.send(json.dumps(payload))


# ── /mic ───────────────────────────────────────────────────────────────────
async def serve_mic(ws) -> None:
    log.info("mic client connected from %s", ws.remote_address)
    _set_state("mic_clients", _state("mic_clients") + 1)
    loop = asyncio.get_event_loop()
    q: stdlib_queue.Queue = stdlib_queue.Queue(maxsize=64)
    in_dev = _state("input_device")

    def callback(indata, frames, time_info, status):
        if status:
            log.debug("input status: %s", status)
        try:
            q.put_nowait(bytes(indata))
        except stdlib_queue.Full:
            pass

    try:
        stream = sd.RawInputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype=DTYPE,
            blocksize=FRAME_SAMPLES,
            device=in_dev,
            callback=callback,
        )
        stream.start()
    except Exception as e:  # noqa: BLE001
        log.error("mic stream open failed: %s", e)
        try:
            await ws.send(json.dumps({"error": str(e)}))
        except Exception:  # noqa: BLE001
            pass
        await ws.close(code=1011, reason=str(e)[:120])
        _set_state("mic_clients", _state("mic_clients") - 1)
        return

    try:
        while True:
            frame = await loop.run_in_executor(None, q.get)
            await ws.send(frame)
    except websockets.ConnectionClosed:
        log.info("mic client disconnected")
    finally:
        stream.stop()
        stream.close()
        _set_state("mic_clients", _state("mic_clients") - 1)


# ── /speaker ───────────────────────────────────────────────────────────────
async def serve_speaker(ws) -> None:
    log.info("speaker client connected from %s", ws.remote_address)
    _set_state("speaker_clients", _state("speaker_clients") + 1)
    loop = asyncio.get_event_loop()
    q: stdlib_queue.Queue = stdlib_queue.Queue(maxsize=64)
    out_dev = _state("output_device")

    # Bytearray buffer + lock — sounddevice's callback wants exactly
    # `len(outdata)` bytes per call (= FRAME_BYTES at our blocksize).
    # The previous queue-of-opaque-chunks design dropped any bytes
    # beyond that boundary, so an 8 KiB liaison TTS chunk played
    # only the first 3.2 KiB and silently discarded the rest — most
    # of the speech was missing on the wire.
    buf = bytearray()
    buf_lock = threading.Lock()

    def callback(outdata, frames, time_info, status):
        if status:
            log.debug("output status: %s", status)
        n = len(outdata)
        with buf_lock:
            avail = min(n, len(buf))
            if avail:
                outdata[:avail] = bytes(buf[:avail])
                del buf[:avail]
            if avail < n:
                outdata[avail:] = b"\x00" * (n - avail)

    try:
        stream = sd.RawOutputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype=DTYPE,
            blocksize=FRAME_SAMPLES,
            device=out_dev,
            callback=callback,
        )
        stream.start()
    except Exception as e:  # noqa: BLE001
        log.error("speaker stream open failed: %s", e)
        await ws.close(code=1011, reason=str(e)[:120])
        _set_state("speaker_clients", _state("speaker_clients") - 1)
        return

    try:
        async for msg in ws:
            if isinstance(msg, bytes):
                with buf_lock:
                    buf.extend(msg)
        # Hold the stream open after EOF until the buffer drains —
        # otherwise we close the device while the tail of the utterance
        # is still queued and the user hears the last word truncated.
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
        _set_state("speaker_clients", _state("speaker_clients") - 1)


# ── /devices ───────────────────────────────────────────────────────────────
async def serve_devices(ws) -> None:
    devs = sd.query_devices()
    payload = {
        "input_default": sd.default.device[0],
        "output_default": sd.default.device[1],
        "input_current": _state("input_device"),
        "output_current": _state("output_device"),
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


# ── /set_device ────────────────────────────────────────────────────────────
async def serve_set_device(ws) -> None:
    try:
        msg = await ws.recv()
        body = json.loads(msg)
        if "input" in body:
            _set_state("input_device", body["input"])
            log.info("input device set to %s", body["input"])
        if "output" in body:
            _set_state("output_device", body["output"])
            log.info("output device set to %s", body["output"])
        await ws.send(json.dumps({"ok": True}))
    except Exception as e:  # noqa: BLE001
        log.warning("/set_device error: %s", e)
        try:
            await ws.send(json.dumps({"ok": False, "error": str(e)}))
        except Exception:  # noqa: BLE001
            pass


# ── /vu  (independent monitor capture) ─────────────────────────────────────
class VuMonitor:
    def __init__(self) -> None:
        self.level = 0.0
        self._stream: sd.RawInputStream | None = None
        self._device: int | None = None
        self._lock = threading.Lock()

    def restart(self, device: int | None) -> None:
        with self._lock:
            if self._stream is not None:
                try:
                    self._stream.stop()
                    self._stream.close()
                except Exception:  # noqa: BLE001
                    pass
                self._stream = None
            try:
                s = sd.RawInputStream(
                    samplerate=SAMPLE_RATE,
                    channels=CHANNELS,
                    dtype=DTYPE,
                    blocksize=FRAME_SAMPLES,
                    device=device,
                    callback=self._cb,
                )
                s.start()
                self._stream = s
                self._device = device
                log.info("vu monitor restarted on device=%s", device)
            except Exception as e:  # noqa: BLE001
                log.warning("vu monitor open failed (device=%s): %s", device, e)
                self.level = 0.0

    def _cb(self, indata, frames, time_info, status):
        # `indata` is a cffi buffer; indexing it returns single-byte
        # `bytes` objects on Python 3.9, which can't be bit-shifted —
        # earlier hand-rolled int16 decode crashed with `unsupported
        # operand type(s) for <<: 'bytes' and 'int'`. Copy to a real
        # bytes blob and let struct unpack the int16s in one go.
        import struct
        raw = bytes(indata)
        n = len(raw) // 2
        if n == 0:
            return
        samples = struct.unpack(f"<{n}h", raw[: n * 2])
        peak = 0
        for s in samples:
            if s < 0:
                s = -s
            if s > peak:
                peak = s
        self.level = peak / 32768.0


vu_monitor: VuMonitor | None = None


async def serve_vu(ws) -> None:
    log.debug("vu client connected from %s", ws.remote_address)
    last_dev = _state("input_device")
    if vu_monitor is not None:
        vu_monitor.restart(last_dev)
    try:
        while True:
            cur = _state("input_device")
            if vu_monitor is not None and cur != last_dev:
                vu_monitor.restart(cur)
                last_dev = cur
            level = vu_monitor.level if vu_monitor is not None else 0.0
            await ws.send(json.dumps({"level": level}))
            await asyncio.sleep(0.05)
    except websockets.ConnectionClosed:
        pass


# ── /log ───────────────────────────────────────────────────────────────────
_log_buffer: stdlib_queue.Queue = stdlib_queue.Queue(maxsize=2048)


class _LogFanoutHandler(logging.Handler):
    def emit(self, record):
        try:
            _log_buffer.put_nowait(self.format(record))
        except Exception:  # noqa: BLE001
            pass


async def serve_log(ws) -> None:
    loop = asyncio.get_event_loop()
    try:
        while True:
            line = await loop.run_in_executor(None, _log_buffer.get)
            await ws.send(line)
    except websockets.ConnectionClosed:
        pass


# ── handler dispatch ───────────────────────────────────────────────────────
async def handler(ws):
    path = ws.request.path if hasattr(ws, "request") else getattr(ws, "path", "")
    if path == "/mic":
        await serve_mic(ws)
    elif path == "/speaker":
        await serve_speaker(ws)
    elif path == "/health":
        await serve_health(ws)
    elif path == "/devices":
        await serve_devices(ws)
    elif path == "/set_device":
        await serve_set_device(ws)
    elif path == "/vu":
        await serve_vu(ws)
    elif path == "/log":
        await serve_log(ws)
    else:
        await ws.close(code=1008, reason=f"unknown path {path!r}")


# ── HTTP UI ────────────────────────────────────────────────────────────────
# All interactive content is built with createElement / textContent —
# never innerHTML — so static analysis (and humans) can tell at a
# glance that no untrusted string ever flows through HTML parsing.
INDEX_HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8" />
<title>robonix audio bridge</title>
<style>
  body { font-family: -apple-system, sans-serif; margin: 2rem; max-width: 900px;
         background: #0e0e10; color: #eaeaea; }
  h1 { font-size: 18px; margin-bottom: 1rem; color: #4caf50; }
  .row { display: flex; gap: 1rem; margin-bottom: 1rem; align-items: center; }
  label { width: 80px; color: #999; }
  select { flex: 1; padding: 6px; background: #1c1c20; color: #eaeaea;
           border: 1px solid #333; }
  .vu { height: 28px; background: #1c1c20; border: 1px solid #333; position: relative; }
  .vu .bar { height: 100%; width: 0%; background: #4caf50; transition: width 50ms; }
  .status { font-size: 12px; color: #777; margin-bottom: 0.6rem; font-family: Menlo, monospace; }
  .log { background: #0a0a0c; border: 1px solid #222; padding: 8px; height: 240px;
         overflow-y: scroll; font-family: Menlo, monospace; font-size: 11px;
         color: #b8b8c0; white-space: pre; }
  button { background: #2c2c30; color: #eaeaea; border: 1px solid #333;
           padding: 6px 14px; cursor: pointer; }
  button:hover { background: #3a3a40; }
</style>
</head><body>
<h1>robonix audio bridge — debug UI</h1>

<div class="row"><label>input</label>
  <select id="in"></select><button id="refresh">refresh</button></div>
<div class="row"><label>output</label><select id="out"></select></div>

<div class="row"><label>vu</label>
  <div class="vu" style="flex:1"><div class="bar" id="vu"></div></div></div>

<div class="status" id="status">connecting...</div>

<div class="log" id="log"></div>

<script>
const HOST = location.hostname;
const PORT = parseInt(location.port) - 1;   // ws port = http port - 1

function clearChildren(el) {
  while (el.firstChild) el.removeChild(el.firstChild);
}

function makeOption(value, text, selected) {
  const o = document.createElement("option");
  o.value = String(value);
  o.text = text;
  if (selected) o.selected = true;
  return o;
}

function loadDevices() {
  const ws = new WebSocket(`ws://${HOST}:${PORT}/devices`);
  ws.onmessage = (e) => {
    const r = JSON.parse(e.data);
    const inSel = document.getElementById("in");
    const outSel = document.getElementById("out");
    clearChildren(inSel);
    clearChildren(outSel);
    const curIn = r.input_current !== null ? r.input_current : r.input_default;
    const curOut = r.output_current !== null ? r.output_current : r.output_default;
    r.devices.forEach((d) => {
      if (d.max_input_channels > 0) {
        inSel.appendChild(makeOption(
          d.id,
          "#" + d.id + " " + d.name + "  (" + d.max_input_channels + " in)",
          d.id === curIn,
        ));
      }
      if (d.max_output_channels > 0) {
        outSel.appendChild(makeOption(
          d.id,
          "#" + d.id + " " + d.name + "  (" + d.max_output_channels + " out)",
          d.id === curOut,
        ));
      }
    });
    ws.close();
  };
}

document.addEventListener("change", (e) => {
  if (e.target.id === "in" || e.target.id === "out") {
    const ws = new WebSocket(`ws://${HOST}:${PORT}/set_device`);
    ws.onopen = () => {
      const body = e.target.id === "in"
        ? { input: parseInt(document.getElementById("in").value) }
        : { output: parseInt(document.getElementById("out").value) };
      ws.send(JSON.stringify(body));
    };
    ws.onmessage = () => ws.close();
  }
});

document.getElementById("refresh").addEventListener("click", loadDevices);

function startVu() {
  const ws = new WebSocket(`ws://${HOST}:${PORT}/vu`);
  const bar = document.getElementById("vu");
  ws.onmessage = (e) => {
    const lv = JSON.parse(e.data).level;
    const pct = Math.min(100, Math.max(0, lv * 400));
    bar.style.width = pct + "%";
    bar.style.background = lv < 0.18 ? "#4caf50" : lv < 0.24 ? "#ff9800" : "#f44336";
  };
  ws.onclose = () => setTimeout(startVu, 1000);
}

function startLog() {
  const ws = new WebSocket(`ws://${HOST}:${PORT}/log`);
  const el = document.getElementById("log");
  ws.onmessage = (e) => {
    el.appendChild(document.createTextNode(e.data + "\\n"));
    el.scrollTop = el.scrollHeight;
  };
  ws.onclose = () => setTimeout(startLog, 1000);
}

function startStatus() {
  setInterval(() => {
    try {
      const ws = new WebSocket(`ws://${HOST}:${PORT}/health`);
      ws.onmessage = (e) => {
        const h = JSON.parse(e.data);
        document.getElementById("status").textContent =
          "ws://" + HOST + ":" + PORT + "   sr=" + h.sample_rate
          + "   input=#" + h.input_device + "   output=#" + h.output_device;
        ws.close();
      };
    } catch (e) {}
  }, 1500);
}

loadDevices();
startVu();
startLog();
startStatus();
</script>
</body></html>"""


class _UiHandler(BaseHTTPRequestHandler):
    def do_GET(self):  # noqa: N802
        if self.path != "/":
            self.send_error(404)
            return
        body = INDEX_HTML.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass


def run_http_thread(host: str, port: int) -> threading.Thread:
    server = HTTPServer((host, port), _UiHandler)

    def runner():
        log.info("HTTP UI on http://%s:%d/", host, port)
        server.serve_forever()
    t = threading.Thread(target=runner, name="http-ui", daemon=True)
    t.start()
    return t


# ── server entry ───────────────────────────────────────────────────────────
async def _serve_ws(host: str, port: int) -> None:
    async with websockets.serve(handler, host, port, max_size=None):
        log.info("websocket on ws://%s:%d", host, port)
        await asyncio.Future()


def run_ws_thread(host: str, port: int) -> threading.Thread:
    def runner():
        asyncio.run(_serve_ws(host, port))
    t = threading.Thread(target=runner, name="ws-server", daemon=True)
    t.start()
    return t


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=60_000,
                        help="WebSocket port; HTTP UI is served on port+1")
    parser.add_argument("--ui-host", default="127.0.0.1",
                        help="bind address for HTTP UI (loopback by default)")
    parser.add_argument("--headless", action="store_true",
                        help="skip HTTP UI; behave like server.py")
    parser.add_argument("--input-device", type=int, default=None)
    parser.add_argument("--output-device", type=int, default=None)
    parser.add_argument("--log", default="info")
    args = parser.parse_args()

    logging.basicConfig(
        level=args.log.upper(),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    logging.getLogger().addHandler(_LogFanoutHandler())

    if args.input_device is not None:
        _set_state("input_device", args.input_device)
    if args.output_device is not None:
        _set_state("output_device", args.output_device)

    global vu_monitor
    vu_monitor = VuMonitor()
    vu_monitor.restart(_state("input_device"))

    run_ws_thread(args.host, args.port)
    if not args.headless:
        run_http_thread(args.ui_host, args.port + 1)
        log.info("open http://%s:%d/ in your browser", args.ui_host, args.port + 1)

    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
