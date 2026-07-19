"""Reverse client-audio transport for the robot-side audio primitive.

The client opens one outbound WebSocket to this server.  That keeps the
network ownership simple: the user only enters the Robonix host in the client;
the robot never needs a client IP address or an exposed client-side port.
"""
from __future__ import annotations

import asyncio
import json
import logging
import queue
import threading
import time
import uuid
from dataclasses import dataclass

import websockets

log = logging.getLogger("audio-client-bridge.reverse")


@dataclass
class _ClientSession:
    ws: object
    loop: asyncio.AbstractEventLoop


class ReverseAudioBridge:
    """Expose one connected client as PCM mic/speaker streams."""

    def __init__(self, host: str, port: int, frame_bytes: int) -> None:
        self.host = host
        self.port = port
        self.frame_bytes = frame_bytes
        self._session: _ClientSession | None = None
        self._lock = threading.Lock()
        self._connected = threading.Event()
        self._mic_streams: dict[str, queue.Queue[bytes | None]] = {}
        self._active_mic_id: str | None = None
        self._pending_controls: dict[str, queue.Queue[dict]] = {}
        self._pending_lock = threading.Lock()
        self._speaker_lock = threading.Lock()
        self._speaker_epoch = 0
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop_async: asyncio.Event | None = None
        self._started = threading.Event()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, name="audio-client-reverse", daemon=True)
        self._thread.start()
        if not self._started.wait(timeout=5.0):
            raise RuntimeError(f"reverse audio listener did not start on {self.host}:{self.port}")

    def stop(self) -> None:
        loop = self._loop
        stop_async = self._stop_async
        if loop is not None and stop_async is not None and loop.is_running():
            loop.call_soon_threadsafe(stop_async.set)
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)
        self._thread = None
        self._connected.clear()
        self._end_all_mics()

    def is_connected(self) -> bool:
        """Whether a robonix-client currently owns this reverse session."""
        return self._connected.is_set()

    def _run(self) -> None:
        asyncio.run(self._serve())

    async def _serve(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._stop_async = asyncio.Event()
        async with websockets.serve(self._handler, self.host, self.port, max_size=None):
            log.info("reverse client bridge listening on ws://%s:%d/client", self.host, self.port)
            self._started.set()
            await self._stop_async.wait()
            with self._lock:
                session = self._session
            if session is not None:
                await session.ws.close(code=1001, reason="audio provider shutting down")
        self._loop = None
        self._stop_async = None
        self._started.clear()

    async def _handler(self, ws) -> None:
        path = ws.request.path if hasattr(ws, "request") else getattr(ws, "path", "")
        if path != "/client":
            await ws.close(code=1008, reason="expected /client")
            return

        with self._lock:
            previous = self._session
            self._session = _ClientSession(ws=ws, loop=asyncio.get_running_loop())
            self._connected.set()
        if previous is not None:
            try:
                await previous.ws.close(code=1012, reason="replaced by newer client")
            except Exception:  # noqa: BLE001
                pass
        log.info("client reverse-audio session connected: %s", getattr(ws, "remote_address", "unknown"))
        try:
            async for message in ws:
                if isinstance(message, bytes):
                    self._put_active_mic(message)
                    continue
                self._handle_control(message)
        except Exception as exc:  # noqa: BLE001
            log.warning("client reverse-audio session closed: %s", exc)
        finally:
            owns_session = False
            with self._lock:
                if self._session and self._session.ws is ws:
                    self._session = None
                    self._connected.clear()
                    owns_session = True
            if owns_session:
                self._end_all_mics()
            self._fail_pending_controls("client audio session disconnected")
            log.info("client reverse-audio session disconnected")

    @staticmethod
    def _put_mic(target: queue.Queue[bytes | None], frame: bytes | None) -> None:
        try:
            target.put_nowait(frame)
        except queue.Full:
            try:
                target.get_nowait()
            except queue.Empty:
                pass
            try:
                target.put_nowait(frame)
            except queue.Full:
                pass

    def _put_active_mic(self, frame: bytes) -> None:
        with self._lock:
            target = self._mic_streams.get(self._active_mic_id or "")
        if target is not None:
            self._put_mic(target, frame)

    def _end_all_mics(self) -> None:
        with self._lock:
            targets = list(self._mic_streams.values())
            self._active_mic_id = None
        for target in targets:
            self._put_mic(target, None)

    def _handle_control(self, raw: str) -> None:
        try:
            body = json.loads(raw)
        except json.JSONDecodeError:
            return
        if body.get("type") == "mic_end":
            stream_id = str(body.get("stream_id") or "")
            with self._lock:
                target = self._mic_streams.get(stream_id or self._active_mic_id or "")
            if target is not None:
                self._put_mic(target, None)
            return
        if body.get("type") != "control_response":
            return
        request_id = str(body.get("id") or "")
        if not request_id:
            return
        with self._pending_lock:
            response = self._pending_controls.get(request_id)
        if response is not None:
            response.put_nowait(body)

    def _fail_pending_controls(self, message: str) -> None:
        with self._pending_lock:
            pending = list(self._pending_controls.values())
            self._pending_controls.clear()
        for response in pending:
            try:
                response.put_nowait({"ok": False, "error": message})
            except queue.Full:
                pass

    def _send(self, payload: str | bytes) -> bool:
        with self._lock:
            session = self._session
        if session is None:
            return False
        try:
            future = asyncio.run_coroutine_threadsafe(session.ws.send(payload), session.loop)
            future.result(timeout=3.0)
            return True
        except Exception as exc:  # noqa: BLE001
            log.warning("reverse client send failed: %s", exc)
            return False

    def _interrupt_speaker(self) -> None:
        with self._speaker_lock:
            self._speaker_epoch += 1
        self._send(json.dumps({"type": "speaker_stop"}))

    def _current_speaker_epoch(self) -> int:
        with self._speaker_lock:
            return self._speaker_epoch

    def control_request(self, op: str, payload: dict | None = None, timeout_s: float = 3.0) -> dict:
        """Ask the connected client to perform a small local audio operation."""
        request_id = uuid.uuid4().hex
        response: queue.Queue[dict] = queue.Queue(maxsize=1)
        with self._pending_lock:
            self._pending_controls[request_id] = response
        try:
            sent = self._send(
                json.dumps(
                    {
                        "type": "control_request",
                        "id": request_id,
                        "op": op,
                        "payload": payload or {},
                    }
                )
            )
            if not sent:
                raise RuntimeError("client audio session disconnected")
            result = response.get(timeout=timeout_s)
            if not isinstance(result, dict):
                raise RuntimeError("invalid client audio control response")
            return result
        except queue.Empty as exc:
            raise RuntimeError(f"client audio control {op} timed out") from exc
        finally:
            with self._pending_lock:
                self._pending_controls.pop(request_id, None)

    def _require_client(self, context) -> bool:
        if self._connected.wait(timeout=3.0):
            return True
        context.abort(
            __import__("grpc").StatusCode.UNAVAILABLE,
            "no client audio session connected; start robonix-client and connect it to this robot",
        )
        return False

    def mic_stream(self, context, audio_pb2):
        if not self._require_client(context):
            return
        log.info("client microphone stream requested")
        stream_id = uuid.uuid4().hex
        frames: queue.Queue[bytes | None] = queue.Queue(maxsize=128)
        with self._lock:
            previous = self._mic_streams.get(self._active_mic_id or "")
            self._mic_streams[stream_id] = frames
            self._active_mic_id = stream_id
        if previous is not None:
            self._put_mic(previous, None)
        metadata = {item.key: item.value for item in context.invocation_metadata()}
        if metadata.get("x-robonix-barge-in", "").lower() in {"1", "true", "yes"}:
            # Explicit F2 / steer capture clears client-local TTS before PCM
            # starts.  The persistent wake-word listener does not carry this
            # metadata and therefore never mutes normal replies.
            self._interrupt_speaker()
        sequence = 0
        try:
            start = {
                "type": "mic_start",
                "stream_id": stream_id,
                "sample_rate": 16000,
                "channels": 1,
            }
            if not self._send(json.dumps(start)):
                context.abort(
                    __import__("grpc").StatusCode.UNAVAILABLE,
                    "client audio session disconnected",
                )
                return
            while context.is_active():
                try:
                    frame = frames.get(timeout=0.5)
                except queue.Empty:
                    continue
                if frame is None:
                    log.info("client microphone stream ended before frame %d", sequence)
                    break
                if sequence == 0:
                    log.info("client microphone first PCM frame: %d bytes", len(frame))
                yield audio_pb2.AudioChunk(
                    timestamp_ns=time.time_ns(),
                    data=frame,
                    sequence=sequence,
                    duration_s=len(frame) / 32000.0,
                )
                sequence += 1
        finally:
            self._send(json.dumps({"type": "mic_stop", "stream_id": stream_id}))
            with self._lock:
                self._mic_streams.pop(stream_id, None)
                if self._active_mic_id == stream_id:
                    self._active_mic_id = None
            log.info("client microphone stream stopped after %d frame(s)", sequence)

    def speaker_stream(self, request_iterator, context, empty_type):
        if not self._require_client(context):
            return empty_type()
        total = 0
        epoch = self._current_speaker_epoch()
        completed = False
        try:
            for chunk in request_iterator:
                if epoch != self._current_speaker_epoch():
                    log.info("speaker stream superseded by voice barge-in")
                    break
                if chunk.data:
                    data = bytes(chunk.data)
                    total += len(data)
                    if not self._send(data):
                        context.abort(__import__("grpc").StatusCode.UNAVAILABLE, "client audio session disconnected")
                        return empty_type()
            completed = context.is_active() and epoch == self._current_speaker_epoch()
        finally:
            self._send(json.dumps({"type": "speaker_end" if completed else "speaker_stop"}))
        # Preserve serialised speaker semantics without sleeping for a full long
        # utterance in the gRPC worker.
        if completed and total:
            time.sleep(min(total / 32000.0 + 0.1, 5.0))
        return empty_type()
