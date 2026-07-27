# SPDX-License-Identifier: MulanPSL-2.0
"""State-machine tests for the lifecycle watcher's epoch response
(service._lifecycle_watch): confirm → generation bump → cross-map drift.

Drives the watcher with a fake hub and a tiny poll interval on a private
event loop. Importing scene_service.service needs the full runtime import
chain (robonix_api, uvicorn, ...) — skips loudly on a bare checkout.
"""
import asyncio
import os
import sys
import tempfile
import time
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scene_service.annotations import AnnotationStore  # noqa: E402
from scene_service.map_binding import MapBinding  # noqa: E402

try:  # the state package needs scipy — absent on a bare mac checkout
    from scene_service.state.object_registry import (
        BBox3D,
        Pose3D,
        SceneObject,
        ObjectRegistry,
    )
    _IMPORT_ERR = None
except ImportError as e:  # pragma: no cover - environment-dependent
    _IMPORT_ERR = e
    BBox3D = Pose3D = SceneObject = ObjectRegistry = None

ROOM_PTS = [[0.0, 0.0], [2.0, 0.0], [2.0, 1.5], [0.0, 1.5]]


def _service_mod():
    err = _IMPORT_ERR
    if err is None:
        try:
            from scene_service import service as service_mod
            return service_mod
        except ImportError as e:
            err = e
    if os.environ.get("PYTEST_CURRENT_TEST"):
        import pytest
        pytest.skip(f"service import chain unavailable: {err}")
    print(f"  [SKIP] lifecycle flush tests unavailable: {err}")
    return None


def _obj(oid: str) -> SceneObject:
    return SceneObject(
        object_id=oid, cls="cup",
        pose=Pose3D(x=1.0, y=2.0, z=0.0, yaw=0.0, frame_id="map"),
        bbox=BBox3D(size_x=0.1, size_y=0.1, size_z=0.1, yaw=0.0, frame_id="map"),
        confidence=0.9, first_seen=1.0, last_seen=2.0,
        observation_count=3, missing=False, attributes={},
    )


class FakeHub:
    """Mutable latched lifecycle sample; .set() swaps what the watcher sees."""

    def __init__(self):
        self._sample = None
        self._count = 0

    def set(self, map_id, generation, mode="mapping"):
        self._sample = SimpleNamespace(
            map_id=map_id, generation=generation, mode=mode
        )
        self._count += 1

    def has(self, key):
        return key == "map_lifecycle" and self._sample is not None

    def latest(self, _key):
        return self._sample, time.time(), self._count


async def _settle(ticks: float = 8.0, interval: float = 0.01):
    """Give the watcher a few poll cycles."""
    await asyncio.sleep(ticks * interval)


def test_confirm_bump_and_cross_map_drift():
    service_mod = _service_mod()
    if service_mod is None:
        return

    async def scenario(tmp):
        registry = ObjectRegistry()
        async with registry.lock():
            registry._objects.update({"o1": _obj("o1"), "o2": _obj("o2")})
        anno = AnnotationStore(os.path.join(tmp, "anno"), map_id="m1")
        anno.create(kind="room", name="kitchen", points=ROOM_PTS)
        hub = FakeHub()
        binding = MapBinding(map_id="m1", source="env")
        live = {"map_id": "m1", "mode": "", "generation": None, "source": "env"}
        derived_resets = []

        async def reset_derived():
            derived_resets.append(time.time())

        task = asyncio.create_task(service_mod._lifecycle_watch(
            hub, binding, anno,
            registry=registry, live_binding=live,
            derived_state_reset=reset_derived, interval_s=0.01,
        ))
        try:
            # 1) Confirm: matching broadcast → epoch learned, nothing flushed.
            hub.set("m1", 1)
            await _settle()
            assert live["generation"] == 1, live
            assert set(registry._objects) == {"o1", "o2"}
            assert not anno.list()[0].stale
            assert derived_resets == []

            # 2) Generation bump (reset under scene): objects flushed,
            #    room flagged stale (kept), epoch advanced — exactly once.
            hub.set("m1", 2)
            await _settle()
            assert live["generation"] == 2, live
            assert registry._objects == {}, registry._objects
            assert len(derived_resets) == 1
            room = anno.list()[0]
            assert room.stale and "1" in room.stale_reason and "2" in room.stale_reason

            # 3) New observations arrive in the new frame and STAY (the
            #    response fired once; an unchanged broadcast never re-flushes).
            async with registry.lock():
                registry._objects["o3"] = _obj("o3")
            await _settle()
            assert set(registry._objects) == {"o3"}

            # 4) Cross-map drift (mapping loaded another map outside the
            #    facade): objects flushed, but scene does NOT adopt the id.
            hub.set("m9", 1)
            await _settle()
            assert registry._objects == {}, registry._objects
            assert live["map_id"] == "m1", live
            assert len(derived_resets) == 2
        finally:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    with tempfile.TemporaryDirectory() as tmp:
        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(scenario(tmp))
        finally:
            loop.close()
    print("  [PASS] test_confirm_bump_and_cross_map_drift")


def test_facade_load_is_not_drift():
    """A Load through the web facade updates the SAME live-binding dict the
    watcher polls — the watcher must treat it as the new expectation, not
    as drift (no flush of the freshly restored objects)."""
    service_mod = _service_mod()
    if service_mod is None:
        return

    async def scenario(tmp):
        registry = ObjectRegistry()
        hub = FakeHub()
        binding = MapBinding(map_id="default", source="default")
        live = {"map_id": "default", "mode": "", "generation": None,
                "source": "default"}
        task = asyncio.create_task(service_mod._lifecycle_watch(
            hub, binding, None,
            registry=registry, live_binding=live, interval_s=0.01,
        ))
        try:
            # Facade Load: binding dict updated first, then mapping's latched
            # broadcast reflects the loaded map.
            live.update({"map_id": "labx", "mode": "localization",
                         "generation": 7, "source": "ui_load"})
            async with registry.lock():
                registry._objects["restored"] = _obj("restored")
            hub.set("labx", 7, mode="localization")
            await _settle()
            assert set(registry._objects) == {"restored"}, registry._objects
            assert live["generation"] == 7
        finally:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    with tempfile.TemporaryDirectory() as tmp:
        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(scenario(tmp))
        finally:
            loop.close()
    print("  [PASS] test_facade_load_is_not_drift")


def test_watcher_survives_none_stores_and_malformed_samples():
    """The watcher is the session's only runtime epoch guard: it must keep
    running with anno_store/registry absent (their init can fail) and past
    malformed broadcast samples (int() raising must not end the task)."""
    service_mod = _service_mod()
    if service_mod is None:
        return

    async def scenario():
        hub = FakeHub()
        binding = MapBinding(map_id="m1", source="env")
        live = {"map_id": "m1", "mode": "", "generation": None, "source": "env"}
        task = asyncio.create_task(service_mod._lifecycle_watch(
            hub, binding, None,
            registry=None, live_binding=live, interval_s=0.01,
        ))
        try:
            hub.set("m1", 1)
            await _settle()
            assert live["generation"] == 1

            # Malformed sample: generation isn't int()-able.
            hub._sample = SimpleNamespace(map_id="m1", generation="junk",
                                          mode="mapping")
            hub._count += 1
            await _settle()
            assert not task.done()          # tick failed, watch continues

            # Bump with no registry and no anno_store: reaction still
            # advances the epoch instead of dying on the missing pieces.
            hub.set("m1", 2)
            await _settle()
            assert not task.done()
            assert live["generation"] == 2
        finally:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    loop = asyncio.new_event_loop()
    try:
        loop.run_until_complete(scenario())
    finally:
        loop.close()
    print("  [PASS] test_watcher_survives_none_stores_and_malformed_samples")


def test_flush_failure_defers_epoch_advance():
    """A failed flush must NOT acknowledge the new generation: advancing
    would make the next tick see no mismatch and never retry, leaving the
    mis-anchored objects in the registry indefinitely. The whole reaction
    (flush, stale-marking, epoch advance) re-fires until the flush lands."""
    service_mod = _service_mod()
    if service_mod is None:
        return

    async def scenario(tmp):
        registry = ObjectRegistry()
        async with registry.lock():
            registry._objects.update({"o1": _obj("o1")})
        anno = AnnotationStore(os.path.join(tmp, "anno"), map_id="m1")
        anno.create(kind="room", name="kitchen", points=ROOM_PTS)
        hub = FakeHub()
        binding = MapBinding(map_id="m1", source="env")
        live = {"map_id": "m1", "mode": "", "generation": None, "source": "env"}
        hold = {"reason": None}
        task = asyncio.create_task(service_mod._lifecycle_watch(
            hub, binding, anno,
            registry=registry, live_binding=live, interval_s=0.01,
            semantic_hold=hold,
        ))
        try:
            hub.set("m1", 1)
            await _settle()
            assert live["generation"] == 1, live

            def broken_clear():
                raise RuntimeError("registry wedged")

            registry.clear_objects = broken_clear
            hub.set("m1", 2)
            await _settle()
            # Bump seen but the flush failed: epoch NOT advanced, objects
            # kept, room not yet stale — nothing acknowledged the bump.
            assert live["generation"] == 1, live
            assert set(registry._objects) == {"o1"}
            assert not anno.list()[0].stale
            assert hold["reason"] is None      # same-map bump: no hold

            del registry.clear_objects          # the flush heals
            await _settle()
            assert live["generation"] == 2, live
            assert registry._objects == {}, registry._objects
            assert anno.list()[0].stale

            # Cross-map switch with the flush wedged again: the save hold
            # is raised immediately (scene and mapping now diverge), and
            # the warn-once latch is NOT set, so the flush retries.
            async with registry.lock():
                registry._objects["o4"] = _obj("o4")
            registry.clear_objects = broken_clear
            hub.set("m9", 1)
            await _settle()
            assert hold["reason"] and "m9" in hold["reason"], hold
            assert set(registry._objects) == {"o4"}   # flush still failing
            del registry.clear_objects
            await _settle()
            assert registry._objects == {}, registry._objects  # retried
            assert hold["reason"], hold  # only a facade Load clears it
        finally:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    with tempfile.TemporaryDirectory() as tmp:
        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(scenario(tmp))
        finally:
            loop.close()
    print("  [PASS] test_flush_failure_defers_epoch_advance")


if __name__ == "__main__":
    print("Running lifecycle flush tests...\n")
    test_confirm_bump_and_cross_map_drift()
    test_facade_load_is_not_drift()
    test_watcher_survives_none_stores_and_malformed_samples()
    test_flush_failure_defers_epoch_advance()
    print("\nAll tests passed!")
