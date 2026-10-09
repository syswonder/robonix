# SPDX-License-Identifier: MulanPSL-2.0
"""Send each newly seen object, with the current camera frame, to memgraph.

Polls the registry and POSTs one `remember` request per new object to the
memgraph Scene Hook, so exploration fills memory without Pilot calling
`list_objects`. SCENE_OBJECT_WATCHDOG=0 disables it;
OBJECT_WATCHDOG_INTERVAL_S sets the poll interval (default 2 s).
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

# With plain Docker networking use the bridge gateway (http://172.17.0.1:37798).
_MEMGRAPH_HOOK_URL = os.environ.get(
    "MEMGRAPH_HOOK_URL",
    "http://127.0.0.1:37798",
)
_DEFAULT_INTERVAL_S = 2.0


class ObjectWatchdog:
    """One memory node per new object; new objects of one tick share a frame."""

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

    async def run(self) -> None:
        """Poll until the task is cancelled."""
        # Objects already present at start are not "new".
        try:
            objs = await self._registry.snapshot()
            self._seen_ids = {o.object_id for o in objs.values() if not o.missing}
        except Exception:
            log.debug("object_watchdog: initial snapshot failed", exc_info=True)
            self._seen_ids = set()

        log.info(
            "object_watchdog: started (interval=%.1fs, %d known objects, url=%s)",
            self._interval, len(self._seen_ids), self._memgraph_url,
        )
        while True:
            try:
                await self._tick()
            except Exception:
                log.debug("object_watchdog: tick error", exc_info=True)
            await asyncio.sleep(self._interval)

    async def _tick(self) -> None:
        objs = await self._registry.snapshot()
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
            ", ".join(f"{o.object_id}({o.label})" for o in new_objects),
        )
        img_b64 = await asyncio.get_running_loop().run_in_executor(
            None, self._capture_frame)
        if not img_b64:
            log.warning("object_watchdog: frame capture failed — "
                        "skipping %d new object(s)", len(new_objects))
            return

        saved = 0
        for obj in new_objects:
            if await self._save_object(obj, img_b64):
                saved += 1
        # Seen even when saving failed, so a flaky memgraph is not retried forever.
        self._seen_ids = current_ids
        if saved:
            log.info("object_watchdog: saved %d/%d new object(s)",
                     saved, len(new_objects))

    def _capture_frame(self) -> str:
        """The latest RGB frame as base64 JPEG, or "" (runs in an executor)."""
        if self._hub is None or not self._hub.has("rgb") or cv2 is None:
            return ""
        rgb_msg, _stamp, _count = self._hub.latest("rgb")
        if rgb_msg is None:
            return ""
        try:
            raw = bytes(rgb_msg.data)
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(rgb_msg.height, rgb_msg.width, -1)
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

    async def _save_object(self, obj, img_b64: str) -> bool:
        """POST one single-object remember request to memgraph."""
        import httpx

        frame_id = str(obj.pose.frame_id or "").strip()
        if not frame_id:
            log.warning(
                "object_watchdog: skip %s — spatial frame is unknown",
                obj.object_id,
            )
            return False
        payload: Dict[str, Any] = {
            "session_id": "scene-watchdog",
            "plan_id": "scene-watchdog",
            "log_record": {
                "ts": time.time_ns(),
                "level": "Info",
                "tag": "scene",
                "msg": f"observed new object: {obj.label}",
            },
            "spatial": {
                "origin": frame_id,
                "objects": [
                    {
                        "obj_id": obj.object_id,
                        "label": obj.label,
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
            log.info("object_watchdog: %s (%s) → node %s",
                     obj.object_id, obj.label, r.json().get("node_id", "?"))
            return True
        except Exception:
            log.debug(
                "object_watchdog: POST failed for %s", obj.object_id,
                exc_info=True,
            )
            return False
