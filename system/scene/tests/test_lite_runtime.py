# SPDX-License-Identifier: MulanPSL-2.0
"""Runtime proof that the lite profile never constructs perception models."""

import asyncio
from types import SimpleNamespace

from scene_service import service
from scene_service.state import ObjectRegistry


class _DummyHub:
    def __init__(self, specs):
        self.specs = list(specs)
        self.started = False

    async def start(self) -> None:
        self.started = True


async def _idle_pose_loop(*_args, **_kwargs) -> None:
    await asyncio.Event().wait()


def test_lite_returns_before_camera_wait_or_detector_construction(monkeypatch):
    class _ForbiddenDetector:
        def __init__(self, *_args, **_kwargs):
            raise AssertionError("lite profile constructed a perception detector")

    monkeypatch.setattr(
        service,
        "_build_topic_specs",
        lambda *_args, **_kwargs: [SimpleNamespace(kind="pose")],
    )
    monkeypatch.setattr(service, "SubscribersHub", _DummyHub)
    monkeypatch.setattr(service, "_self_pose_loop", _idle_pose_loop)
    monkeypatch.setattr(service, "ConceptGraphsDetector", _ForbiddenDetector)
    monkeypatch.setattr(service, "VLMObjectDetector", _ForbiddenDetector)

    async def scenario() -> None:
        hub, detector, tasks = await service._start_ros_ingest(
            atlas_stub=SimpleNamespace(),
            registry=ObjectRegistry(),
            self_tracker=SimpleNamespace(),
            config={
                "observations": [{"kind": "pose"}],
                "perception": {"profile": "lite"},
            },
        )
        assert hub.started is True
        assert detector is None
        assert len(tasks) == 1
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    asyncio.run(scenario())
