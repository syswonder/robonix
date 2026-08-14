# SPDX-License-Identifier: MulanPSL-2.0
"""Object-level watchdog: save a memory snapshot for every newly-seen object.

Runs as a background asyncio task inside the Scene service.  At each tick
it polls ``ObjectRegistry.snapshot()``, diffs against previously-seen
object IDs, and for every genuinely new object:

  1. captures the latest RGB camera frame (JPEG-encoded)
  2. builds a ``remember`` request payload (one object, one image)
  3. HTTP POSTs to memgraph's Scene Hook endpoint on port 37798

This is separate from the existing Scene Hook in ``mcp_tools.py``
(which triggers on ``scene.list_objects`` MCP calls and saves the
*whole* scene).  The watchdog runs autonomously and does not require
Pilot to call ``list_objects`` — it is designed for the patrol /
exploration use-case where the robot moves and new objects appear in
view continuously.

Env vars:
  ``SCENE_OBJECT_WATCHDOG`` — set to ``"0"`` to disable (default ``"1"``).
  ``OBJECT_WATCHDOG_INTERVAL_S`` — poll interval in seconds (default 2.0).
"""

from __future__ import annotations

import asyncio
import base64
import logging
import os
import time
from typing import Any, Dict, Set

import numpy as np

try:
    import cv2  # type: ignore
except ImportError:
    cv2 = None

log = logging.getLogger(__name__)

# Set MEMGRAPH_HOOK_URL to the host-visible memgraph Scene Hook
# endpoint.  Default works with --network host; for plain Docker
# use "http://172.17.0.1:37798" (bridge gateway).
_MEMGRAPH_HOOK_URL = os.environ.get(
    "MEMGRAPH_HOOK_URL",
    "http://127.0.0.1:37798",
)
_DEFAULT_INTERVAL_S = 2.0


class ObjectWatchdog:
    """Background observer that detects new objects and saves images.

    One object → one image (the full camera frame at the moment the
    object was first seen).  If multiple new objects appear in the
    same tick a single frame is captured and reused for efficiency;
    each object still gets its own MemoryNode.
    """

    def __init__(
        self,
        *,
        registry,          # ObjectRegistry
        hub,               # SubscribersHub (for .latest("rgb"))
        memgraph_url: str = _MEMGRAPH_HOOK_URL,
        interval_s: float = 0.0,
    ) -> None:
        self._registry = registry
        self._hub = hub
        self._memgraph_url = memgraph_url
        self._interval = (
            interval_s
            if interval_s > 0
            else float(os.environ.get("OBJECT_WATCHDOG_INTERVAL_S", _DEFAULT_INTERVAL_S))
        )
        self._seen_ids: Set[str] = set()
        self._task: asyncio.Task | None = None
        self._running = False

    # ── lifecycle ──────────────────────────────────────────────────────

    async def run(self) -> None:
        """Blocking async entrypoint — use ``asyncio.create_task(wd.run())``."""
        if self._running:
            return
        self._running = True

        # Seed the seen-set from whatever the registry already holds so
        # the first tick only reacts to genuinely new arrivals.
        try:
            objs, _surfs = await self._registry.snapshot()
            self._seen_ids = {o.object_id for o in objs.values() if not o.missing}
        except Exception:
            log.debug("object_watchdog: initial snapshot failed", exc_info=True)
            self._seen_ids = set()

        log.info(
            "object_watchdog: started (interval=%.1fs, %d known objects, url=%s)",
            self._interval, len(self._seen_ids), self._memgraph_url,
        )

        while self._running:
            try:
                await self._tick()
            except Exception:
                log.debug("object_watchdog: tick error", exc_info=True)
            await asyncio.sleep(self._interval)

    def stop(self) -> None:
        """Signal the loop to exit at the next sleep boundary."""
        self._running = False
        if self._task is not None:
            self._task.cancel()

    # ── tick logic ─────────────────────────────────────────────────────

    async def _tick(self) -> None:
        objs, _surfs = await self._registry.snapshot()
        visible: Dict[str, Any] = {
            o.object_id: o for o in objs.values() if not o.missing
        }
        current_ids = set(visible.keys())
        new_ids = current_ids - self._seen_ids

        if not new_ids:
            return

        new_objects = [visible[oid] for oid in new_ids]
        log.info(
            "object_watchdog: %d new object(s): %s",
            len(new_objects),
            ", ".join(f"{o.object_id}({o.cls})" for o in new_objects),
        )

        # Capture one frame for the whole batch.
        loop = asyncio.get_running_loop()
        img_b64 = await loop.run_in_executor(None, self._capture_frame)
        if not img_b64:
            log.warning("object_watchdog: frame capture failed — "
                        "skipping %d new object(s)", len(new_objects))
            return

        # POST one memory node per new object.  Serialised to keep load
        # on memgraph predictable; the watchdog interval provides plenty
        # of headroom.
        saved = 0
        for obj in new_objects:
            ok = await self._save_object(obj, img_b64)
            if ok:
                saved += 1

        # Mark all as seen regardless of save outcome so we don't retry
        # the same objects forever on transient errors.
        self._seen_ids = current_ids

        if saved:
            log.info("object_watchdog: saved %d/%d new object(s)",
                     saved, len(new_objects))

    # ── frame capture ──────────────────────────────────────────────────

    def _capture_frame(self) -> str:
        """Grab the latest RGB frame from the ROS hub and return a
        base64-encoded JPEG string.  Returns ``""`` on any failure.

        This is CPU-bound (numpy reshape + cv2 encode) and called via
        ``run_in_executor`` to avoid blocking the asyncio event loop.
        """
        if self._hub is None or not self._hub.has("rgb"):
            return ""

        rgb_msg, _stamp, _count = self._hub.latest("rgb")
        if rgb_msg is None:
            return ""

        if cv2 is None:
            return ""

        try:
            raw = bytes(rgb_msg.data)
            h, w = rgb_msg.height, rgb_msg.width
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, -1)
            if rgb_msg.encoding == "rgb8":
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            ok, jpg = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                log.warning("object_watchdog: cv2.imencode failed")
                return ""
            return base64.b64encode(jpg.tobytes()).decode("ascii")
        except Exception:
            log.debug("object_watchdog: frame encode failed", exc_info=True)
            return ""

    # ── POST to memgraph ───────────────────────────────────────────────

    async def _save_object(self, obj, img_b64: str) -> bool:
        """POST a single-object remember request to the memgraph Scene Hook."""
        import httpx

        frame_id = str(obj.pose.frame_id or "").strip()
        if not frame_id:
            log.warning(
                "object_watchdog: skip %s — spatial frame is unknown",
                obj.object_id,
            )
            return False
        now_ns = time.time_ns()
        payload: Dict[str, Any] = {
            "session_id": "scene-watchdog",
            "plan_id": "scene-watchdog",
            "log_record": {
                "ts": now_ns,
                "level": "Info",
                "tag": "scene",
                "msg": f"observed new object: {obj.cls}",
            },
            "spatial": {
                "origin": frame_id,
                "objects": [
                    {
                        "obj_id": obj.object_id,
                        "label": obj.cls,
                        "x": float(obj.pose.x),
                        "y": float(obj.pose.y),
                        "z": float(obj.pose.z),
                    }
                ],
            },
            "image_base64": img_b64,
        }

        try:
            async with httpx.AsyncClient(timeout=5.0) as client:
                r = await client.post(self._memgraph_url, json=payload)
            if r.status_code >= 400:
                log.warning(
                    "object_watchdog: memgraph returned %d for %s: %s",
                    r.status_code, obj.object_id, r.text[:200],
                )
                return False
            result = r.json()
            node_id = result.get("node_id", "?")
            log.info("object_watchdog: %s (%s) → node %s",
                     obj.object_id, obj.cls, node_id)
            return True
        except Exception:
            log.debug(
                "object_watchdog: POST failed for %s", obj.object_id,
                exc_info=True,
            )
            return False
