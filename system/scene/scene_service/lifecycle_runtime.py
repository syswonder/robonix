# SPDX-License-Identifier: MulanPSL-2.0
"""Thread-safe lifecycle coordination between Driver RPCs and Scene asyncio."""

from __future__ import annotations

import asyncio
import json
import logging
import os
import threading
from pathlib import Path
from typing import Any

from robonix_api import Deferred, Err, Ok


async def _stop_async_resource(
    label: str,
    resource: Any,
    errors: list[str],
) -> None:
    """Await one resource's ``stop`` method and retain cleanup failures."""
    if resource is None:
        return
    try:
        await resource.stop()
    except Exception as exc:  # noqa: BLE001
        errors.append(f"{label}.stop: {type(exc).__name__}: {exc}")


async def _cancel_and_join_tasks(
    tasks: list[asyncio.Task],
    errors: list[str],
) -> None:
    """Cancel every background task and await its finalizer before return."""
    for task in tasks:
        if not task.done():
            task.cancel()
    if not tasks:
        return
    results = await asyncio.gather(*tasks, return_exceptions=True)
    for task, result in zip(tasks, results):
        if isinstance(result, BaseException) and not isinstance(
            result,
            asyncio.CancelledError,
        ):
            errors.append(
                f"background task {task.get_name()}: "
                f"{type(result).__name__}: {result}"
            )


async def _stop_web_server(
    server: Any,
    task: asyncio.Task | None,
    timeout_s: float,
    errors: list[str],
) -> None:
    """Ask uvicorn to exit and await the serve task that owns its socket."""
    if server is not None:
        server.should_exit = True
    if task is None:
        return
    try:
        await asyncio.wait_for(asyncio.shield(task), timeout=timeout_s)
    except asyncio.TimeoutError:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        errors.append(
            f"web server did not exit and release its socket within {timeout_s:.1f}s"
        )
    except asyncio.CancelledError:
        if not task.cancelled():
            raise
    except Exception as exc:  # noqa: BLE001
        errors.append(f"web server: {type(exc).__name__}: {exc}")


async def close_scene_runtime_resources(
    *,
    background_tasks: list[asyncio.Task],
    scene_graph_stop: asyncio.Event | None = None,
    perception: Any = None,
    hub: Any = None,
    geometric_loop: Any = None,
    web_server: Any = None,
    web_task: asyncio.Task | None = None,
    object_store: Any = None,
    web_timeout_s: float = 8.0,
) -> None:
    """Fully close one active Scene runtime or raise after best-effort cleanup.

    The web server is signaled first so uvicorn can begin graceful shutdown
    while producers stop. Every owned task is joined, and synchronous store
    closure runs off-loop but is awaited. Errors are accumulated so one broken
    resource never prevents the remaining resources from releasing.
    """
    errors: list[str] = []
    if scene_graph_stop is not None:
        scene_graph_stop.set()
    if web_server is not None:
        web_server.should_exit = True

    await _stop_async_resource("perception", perception, errors)
    await _stop_async_resource("hub", hub, errors)
    await _stop_async_resource("geometric_loop", geometric_loop, errors)
    await _cancel_and_join_tasks(background_tasks, errors)
    await _stop_web_server(web_server, web_task, web_timeout_s, errors)

    if object_store is not None:
        try:
            await asyncio.to_thread(object_store.close)
        except Exception as exc:  # noqa: BLE001
            errors.append(f"object_store.close: {type(exc).__name__}: {exc}")
    if errors:
        raise RuntimeError("Scene cleanup failed: " + "; ".join(errors))


class SceneLifecycleRuntime:
    """Own Scene config and synchronize lifecycle RPCs with async startup."""

    def __init__(
        self,
        logger: logging.Logger,
        *,
        activate_timeout_s: float = 85.0,
        shutdown_timeout_s: float = 15.0,
    ) -> None:
        self.log = logger
        self.activate_timeout_s = activate_timeout_s
        self.shutdown_timeout_s = shutdown_timeout_s
        self.initialized = threading.Event()
        self.activation_requested = threading.Event()
        self.runtime_ready = threading.Event()
        self.shutdown_requested = threading.Event()
        self.driver_shutdown_requested = threading.Event()
        self.shutdown_complete = threading.Event()
        self._lock = threading.Lock()
        self._config: dict = {}
        self._runtime_error = ""
        self._shutdown_error = ""

    @property
    def config(self) -> dict:
        """Return a copy of the config accepted by Driver(CMD_INIT)."""
        with self._lock:
            return dict(self._config)

    def on_init(self, cfg: dict):
        """Accept canonical config or the deprecated RBNX_CONFIG_FILE fallback."""
        effective = dict(cfg)
        legacy_path = os.environ.get("RBNX_CONFIG_FILE", "").strip()
        if legacy_path:
            self.log.warning(
                "RBNX_CONFIG_FILE is deprecated for Scene; move these values to "
                "system.scene.config in robonix_manifest.yaml"
            )
            if not effective:
                loaded, error = self._read_legacy_file(Path(legacy_path))
                if error:
                    return Err(error)
                effective = loaded
            else:
                self.log.warning(
                    "ignoring RBNX_CONFIG_FILE because Driver(CMD_INIT) supplied config"
                )

        with self._lock:
            if self.initialized.is_set() and effective != self._config:
                return Err("Scene is already initialized with different config")
            self._config = effective
            self.initialized.set()
        self.log.info("Driver(INIT) accepted Scene config keys: %s", sorted(effective))
        return Ok()

    def on_activate(self):
        """Request async startup and wait until Scene is actually ready."""
        self.activation_requested.set()
        if not self.runtime_ready.wait(self.activate_timeout_s):
            return Err(
                f"Scene activation timed out after {self.activate_timeout_s:.1f}s"
            )
        with self._lock:
            error = self._runtime_error
        return Err(error) if error else Ok()

    def on_shutdown(self):
        """Request async cleanup and wait before Driver(SHUTDOWN) acknowledges."""
        self.driver_shutdown_requested.set()
        self.shutdown_requested.set()
        if not self.shutdown_complete.wait(self.shutdown_timeout_s):
            return Err(f"Scene shutdown timed out after {self.shutdown_timeout_s:.1f}s")
        with self._lock:
            error = self._shutdown_error
        return Err(error) if error else Ok()

    def on_deactivate(self):
        """Reject unsupported deactivation without lying about resource state."""
        return Deferred(
            "Scene does not support CMD_DEACTIVATE yet; resources remain ACTIVE. "
            "Use CMD_SHUTDOWN to stop Scene."
        )

    def request_process_shutdown(self) -> None:
        """Signal shutdown from SIGINT/SIGTERM without blocking the event loop."""
        self.shutdown_requested.set()

    def mark_runtime_ready(self) -> None:
        """Release Driver(ACTIVATE) after all Scene runtime resources are live."""
        self.runtime_ready.set()

    def mark_runtime_failed(self, error: BaseException | str) -> None:
        """Release Driver(ACTIVATE) with a stable failure message."""
        with self._lock:
            self._runtime_error = str(error)
        self.runtime_ready.set()

    def mark_shutdown_complete(self, error: BaseException | str | None = None) -> None:
        """Release Driver(SHUTDOWN), returning Err if cleanup did not finish."""
        if error is not None:
            with self._lock:
                self._shutdown_error = str(error)
        self.shutdown_complete.set()

    @staticmethod
    def _read_legacy_file(path: Path) -> tuple[dict, str]:
        """Load the deprecated JSON/YAML config file without hiding errors."""
        try:
            text = path.read_text()
            try:
                loaded = json.loads(text)
            except json.JSONDecodeError:
                import yaml

                loaded = yaml.safe_load(text) or {}
        except Exception as exc:  # noqa: BLE001
            return {}, f"failed to read deprecated Scene config {path}: {exc}"
        if not isinstance(loaded, dict):
            return {}, f"deprecated Scene config {path} must contain a mapping"
        return loaded, ""
