from __future__ import annotations

import asyncio
import json
import queue
from types import SimpleNamespace

from audio_client_bridge.reverse import ReverseAudioBridge, _ClientSession


class ActiveContext:
    @staticmethod
    def is_active() -> bool:
        return True


def test_barge_in_supersedes_inflight_speaker_generation() -> None:
    bridge = ReverseAudioBridge("127.0.0.1", 0, 3200)
    sent: list[str | bytes] = []
    bridge._require_client = lambda _context: True  # type: ignore[method-assign]
    bridge._send = lambda payload: (sent.append(payload), True)[1]  # type: ignore[method-assign]

    def chunks():
        yield SimpleNamespace(data=b"old-audio-1")
        bridge._interrupt_speaker()
        yield SimpleNamespace(data=b"old-audio-2")

    bridge.speaker_stream(chunks(), ActiveContext(), lambda: object())

    assert b"old-audio-1" in sent
    assert b"old-audio-2" not in sent
    controls = [json.loads(item) for item in sent if isinstance(item, str)]
    assert {control["type"] for control in controls} == {"speaker_stop"}


def test_send_forwards_payload_to_connected_client(monkeypatch) -> None:
    bridge = ReverseAudioBridge("127.0.0.1", 0, 3200)
    delivered: list[str | bytes] = []
    marker_loop = object()

    class FakeWebSocket:
        async def send(self, payload: str | bytes) -> None:
            delivered.append(payload)

    class FakeFuture:
        @staticmethod
        def result(timeout: float) -> None:
            assert timeout == 3.0

    def run_coroutine(coro, loop):
        assert loop is marker_loop
        asyncio.run(coro)
        return FakeFuture()

    bridge._session = _ClientSession(ws=FakeWebSocket(), loop=marker_loop)
    monkeypatch.setattr(
        "audio_client_bridge.reverse.asyncio.run_coroutine_threadsafe",
        run_coroutine,
    )

    assert bridge._send('{"type":"mic_start"}') is True
    assert delivered == ['{"type":"mic_start"}']


def test_send_reports_disconnected_client() -> None:
    bridge = ReverseAudioBridge("127.0.0.1", 0, 3200)

    assert bridge._send('{"type":"mic_start"}') is False


def test_stale_mic_end_does_not_terminate_new_stream() -> None:
    bridge = ReverseAudioBridge("127.0.0.1", 0, 3200)
    old_frames: queue.Queue[bytes | None] = queue.Queue()
    new_frames: queue.Queue[bytes | None] = queue.Queue()
    bridge._mic_streams = {"old": old_frames, "new": new_frames}
    bridge._active_mic_id = "new"

    bridge._handle_control('{"type":"mic_end","stream_id":"old"}')
    bridge._put_active_mic(b"new-pcm")

    assert old_frames.get_nowait() is None
    assert new_frames.get_nowait() == b"new-pcm"
    assert new_frames.empty()
