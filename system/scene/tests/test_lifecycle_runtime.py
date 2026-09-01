# SPDX-License-Identifier: MulanPSL-2.0
"""Scene Driver config and shutdown coordination tests."""

import asyncio
import logging
import threading
import time

from scene_service.lifecycle_runtime import (
    SceneLifecycleRuntime,
    close_scene_runtime_resources,
)
from robonix_api.result import Deferred, Err, Ok


def test_cmd_init_keeps_nested_manifest_config_as_the_runtime_source(
    monkeypatch,
) -> None:
    monkeypatch.delenv("RBNX_CONFIG_FILE", raising=False)
    lifecycle = SceneLifecycleRuntime(logging.getLogger("test"))
    expected = {"camera_provider_id": "front_camera", "web_port": 50107}

    result = lifecycle.on_init(expected)

    assert isinstance(result, Ok)
    assert lifecycle.initialized.is_set()
    assert lifecycle.config == expected


def test_legacy_config_file_remains_a_deprecated_fallback(
    monkeypatch, tmp_path
) -> None:
    config_file = tmp_path / "scene.yaml"
    config_file.write_text("camera_provider_id: legacy_camera\nweb_port: 50108\n")
    monkeypatch.setenv("RBNX_CONFIG_FILE", str(config_file))
    lifecycle = SceneLifecycleRuntime(logging.getLogger("test"))

    result = lifecycle.on_init({})

    assert isinstance(result, Ok)
    assert lifecycle.config == {
        "camera_provider_id": "legacy_camera",
        "web_port": 50108,
    }


def test_activate_waits_for_real_runtime_readiness() -> None:
    lifecycle = SceneLifecycleRuntime(logging.getLogger("test"), activate_timeout_s=0.5)
    result: list[object] = []
    caller = threading.Thread(target=lambda: result.append(lifecycle.on_activate()))

    caller.start()
    assert lifecycle.activation_requested.wait(0.2)
    assert caller.is_alive()
    lifecycle.mark_runtime_ready()
    caller.join(0.2)

    assert isinstance(result[0], Ok)


def test_repeated_activate_is_idempotent_after_runtime_is_ready() -> None:
    lifecycle = SceneLifecycleRuntime(logging.getLogger("test"))
    lifecycle.mark_runtime_ready()

    assert isinstance(lifecycle.on_activate(), Ok)
    assert isinstance(lifecycle.on_activate(), Ok)


def test_shutdown_acknowledges_only_after_async_cleanup() -> None:
    lifecycle = SceneLifecycleRuntime(logging.getLogger("test"), shutdown_timeout_s=0.5)
    result: list[object] = []
    caller = threading.Thread(target=lambda: result.append(lifecycle.on_shutdown()))

    caller.start()
    assert lifecycle.shutdown_requested.wait(0.2)
    assert lifecycle.driver_shutdown_requested.is_set()
    time.sleep(0.02)
    assert caller.is_alive()
    lifecycle.mark_shutdown_complete()
    caller.join(0.2)

    assert isinstance(result[0], Ok)


def test_repeated_shutdown_is_idempotent_after_cleanup() -> None:
    lifecycle = SceneLifecycleRuntime(logging.getLogger("test"))
    lifecycle.mark_shutdown_complete()

    assert isinstance(lifecycle.on_shutdown(), Ok)
    assert isinstance(lifecycle.on_shutdown(), Ok)


def test_shutdown_surfaces_cleanup_failure_instead_of_returning_ok() -> None:
    lifecycle = SceneLifecycleRuntime(logging.getLogger("test"))
    lifecycle.mark_shutdown_complete("web server failed to release port")

    result = lifecycle.on_shutdown()

    assert isinstance(result, Err)
    assert result.message == "web server failed to release port"


def test_runtime_cleanup_awaits_every_owned_resource_and_task() -> None:
    events: list[str] = []

    class _AsyncResource:
        def __init__(self, name: str) -> None:
            self.name = name

        async def stop(self) -> None:
            """Record only after an asynchronous stop has completed."""
            await asyncio.sleep(0.01)
            events.append(self.name)

    class _WebServer:
        should_exit = False

    class _ObjectStore:
        def close(self) -> None:
            """Model the synchronous milvus-lite close operation."""
            time.sleep(0.01)
            events.append("object_store")

    async def scenario() -> None:
        """Start representative tasks, then prove cleanup joins all of them."""
        web_server = _WebServer()

        async def background() -> None:
            try:
                await asyncio.Event().wait()
            finally:
                await asyncio.sleep(0.01)
                events.append("background")

        async def serve() -> None:
            while not web_server.should_exit:
                await asyncio.sleep(0.001)
            await asyncio.sleep(0.01)
            events.append("web")

        background_task = asyncio.create_task(background(), name="test-background")
        web_task = asyncio.create_task(serve(), name="test-web")
        await asyncio.sleep(0)
        await close_scene_runtime_resources(
            background_tasks=[background_task],
            perception=_AsyncResource("perception"),
            hub=_AsyncResource("hub"),
            geometric_loop=_AsyncResource("geometric_loop"),
            web_server=web_server,
            web_task=web_task,
            object_store=_ObjectStore(),
        )

        assert background_task.done()
        assert web_task.done()
        assert set(events) == {
            "perception",
            "hub",
            "geometric_loop",
            "background",
            "web",
            "object_store",
        }

    asyncio.run(scenario())


def test_runtime_cleanup_releases_web_port_before_returning() -> None:
    class _WebServer:
        should_exit = False

    async def scenario() -> None:
        """Bind a real socket and verify immediate rebinding after cleanup."""
        controller = _WebServer()
        ready = asyncio.Event()
        port_holder: list[int] = []

        async def serve() -> None:
            server = await asyncio.start_server(
                lambda _reader, _writer: None,
                "127.0.0.1",
                0,
            )
            port_holder.append(server.sockets[0].getsockname()[1])
            ready.set()
            while not controller.should_exit:
                await asyncio.sleep(0.001)
            await asyncio.sleep(0.02)
            server.close()
            await server.wait_closed()

        web_task = asyncio.create_task(serve(), name="test-port-owner")
        await ready.wait()
        await close_scene_runtime_resources(
            background_tasks=[],
            web_server=controller,
            web_task=web_task,
        )

        rebound = await asyncio.start_server(
            lambda _reader, _writer: None,
            "127.0.0.1",
            port_holder[0],
        )
        rebound.close()
        await rebound.wait_closed()
        assert web_task.done()

    asyncio.run(scenario())


def test_deactivate_is_deferred_while_scene_resources_remain_active() -> None:
    lifecycle = SceneLifecycleRuntime(logging.getLogger("test"))

    result = lifecycle.on_deactivate()

    assert isinstance(result, Deferred)
    assert "remain ACTIVE" in result.reason


def test_activation_surfaces_async_startup_failure() -> None:
    lifecycle = SceneLifecycleRuntime(logging.getLogger("test"), activate_timeout_s=0.5)
    lifecycle.mark_runtime_failed("camera initialization failed")

    result = lifecycle.on_activate()

    assert isinstance(result, Err)
    assert result.message == "camera initialization failed"
