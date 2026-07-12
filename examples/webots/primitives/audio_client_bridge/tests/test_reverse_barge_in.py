from __future__ import annotations

import json
from types import SimpleNamespace

from audio_client_bridge.reverse import ReverseAudioBridge


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
