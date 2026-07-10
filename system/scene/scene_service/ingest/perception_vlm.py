# SPDX-License-Identifier: MulanPSL-2.0
"""VLM-based object detection: take an RGB frame, ask an
OpenAI-compatible vision model to enumerate visible objects with
approximate image coordinates, parse the JSON response, and (when
depth is available) reproject to world coordinates.

This lives inside scene/ deliberately — system/ services should not
reverse-depend on a service-layer perception package. The detector
calls the same OpenAI-compatible endpoint that pilot already uses
(VLM_BASE_URL / VLM_API_KEY / VLM_MODEL), so no new credentials.

v1 keeps the implementation simple: small prompt, JSON-only response,
no streaming, no caching. Failures are logged and the perception loop
keeps running on the next tick.
"""
from __future__ import annotations

import asyncio
import base64
import json
import logging
import math
import os
import re
import time
from dataclasses import dataclass
from typing import Awaitable, Callable, Optional

import httpx

from ..state.data_assoc import Detection
from ..state.object_registry import BBox3D, Pose3D

log = logging.getLogger(__name__)


_DETECTION_PROMPT = """You are a visual perception module for a robot operating
in an indoor office. Look at the image and enumerate every distinct physical
object you see that the robot might interact with or need to avoid. For each
object, report:

  - cls: a single lowercase class name from this preferred set when applicable
    (table, chair, door, cup, bottle, tray, tool, person, robot,
    monitor, keyboard, book, plant, box, trash_bin); otherwise pick the
    most specific common noun.
  - confidence: 0.0 to 1.0 (how sure you are it's that class).
  - bbox_2d: image-pixel [x_min, y_min, x_max, y_max] integers.
  - approximate_depth_m: rough metres from the camera, your best guess.

Respond ONLY with a JSON object of the form:
  {"detections": [{"cls": "...", "confidence": 0.83,
                   "bbox_2d": [x0, y0, x1, y1],
                   "approximate_depth_m": 1.7}, ...]}
No prose, no markdown fences, no explanation.

Limit to at most 12 detections, prioritising larger / closer items.
"""


@dataclass
class _CamIntrinsics:
    """Pinhole intrinsics supplied by the active camera deployment.

    Scene must not carry simulator-specific calibration constants. Metric
    perception gets K from `primitive/camera/intrinsics`; deployments without a
    reliable CameraInfo stream may provide a reviewed `intrinsics_fallback` in
    their manifest.
    """
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


class VLMObjectDetector:
    """Runs the RGB-poll → VLM-call → Detection-list pipeline as one
    asyncio task. Calls back into `on_detections` with a batch of
    `Detection` objects at each successful tick.

    The detector reads camera/snapshot via the existing PrimitivePoller
    machinery (passed in as `rgb_fetcher`) so we don't duplicate the
    atlas connect logic."""

    def __init__(
        self,
        *,
        rgb_fetcher: Callable[[], Optional[bytes]],
        chassis_pose_fn: Callable[[], Optional[tuple[float, float, float, float]]],
        on_detections: Callable[[list[Detection]], Awaitable[None]],
        period_s: float = 3.0,
        camera_frame_id: str = "camera_optical_frame",
        intrinsics: Optional[_CamIntrinsics] = None,
    ) -> None:
        # `rgb_fetcher` returns the latest JPEG bytes (or None when no
        # frame has arrived yet). When ROS subscribers are the source,
        # the hub hands us a sensor_msgs/Image whose `data` is raw RGB;
        # service.py's adapter turns that into JPEG before calling us.
        self.rgb_fetcher = rgb_fetcher
        self.chassis_pose_fn = chassis_pose_fn
        self.on_detections = on_detections
        self.period_s = period_s
        self.camera_frame_id = camera_frame_id
        self.intrinsics = intrinsics
        self._missing_intrinsics_logged = False
        self._task: Optional[asyncio.Task[None]] = None
        self._stop = asyncio.Event()

        # Pull VLM creds from env at construction so failures are
        # visible at startup rather than first tick.
        self.base_url = (os.environ.get("VLM_BASE_URL") or os.environ.get("OPENAI_BASE_URL") or "").rstrip("/")
        self.api_key = os.environ.get("VLM_API_KEY") or os.environ.get("OPENAI_API_KEY") or ""
        self.model = os.environ.get("VLM_MODEL") or os.environ.get("OPENAI_MODEL") or "gpt-5.5"
        # Shared VLM-wide reasoning knob. Opt-in: unset/empty → the field is
        # omitted entirely, so non-reasoning models and strict OpenAI-compatible
        # endpoints (which 400 on an unsupported param) are unaffected. Set
        # VLM_REASONING_EFFORT=minimal to keep a reasoning VLM_MODEL (e.g.
        # doubao-seed-2-1-pro) fast (~2 s, no thinking); minimal|low|medium|high.
        self.reasoning_effort = os.environ.get("VLM_REASONING_EFFORT", "").strip()
        if not self.api_key:
            log.warning("[scene-vlm] VLM_API_KEY not set; perception will be inert")

    async def start(self) -> None:
        if self._task is not None:
            return
        self._stop.clear()
        self._task = asyncio.create_task(self._run(), name="scene-vlm-perception")

    async def stop(self) -> None:
        self._stop.set()
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
            self._task = None

    async def _run(self) -> None:
        while not self._stop.is_set():
            try:
                if self.api_key:
                    await self._tick()
            except Exception as e:  # noqa: BLE001
                log.warning("[scene-vlm] tick failed: %s", e)
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=self.period_s)
            except asyncio.TimeoutError:
                pass

    async def _tick(self) -> None:
        # `rgb_fetcher` is a sync callable returning JPEG bytes (or
        # None). Synchronous because the ROS subscriber side caches
        # the latest frame in a thread-safe slot — no awaiting needed.
        jpeg_bytes = self.rgb_fetcher()
        if not jpeg_bytes:
            return
        jpeg_b64 = base64.b64encode(jpeg_bytes).decode("ascii")
        detections_json = await self._call_vlm(jpeg_b64)
        if not detections_json:
            return
        detections = self._project_to_world(detections_json)
        if detections:
            await self.on_detections(detections)

    async def _call_vlm(self, jpeg_b64: str) -> list[dict]:
        """One OpenAI-compatible chat-completions call with image input.
        Returns the parsed `detections` list (possibly empty)."""
        if not self.base_url or not self.api_key:
            return []
        url = f"{self.base_url}/chat/completions"
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }
        body = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": "You produce only valid JSON."},
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": _DETECTION_PROMPT},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{jpeg_b64}"}},
                    ],
                },
            ],
            "temperature": 0.0,
        }
        if self.reasoning_effort:
            body["reasoning_effort"] = self.reasoning_effort
        async with httpx.AsyncClient(timeout=30.0) as client:
            r = await client.post(url, json=body, headers=headers)
            if r.status_code >= 400:
                log.warning("[scene-vlm] VLM HTTP %d: %s", r.status_code, r.text[:200])
                return []
            data = r.json()
            try:
                text = data["choices"][0]["message"]["content"]
            except (KeyError, IndexError):
                return []
        # Strip markdown fences if the model added them despite the prompt.
        text = re.sub(r"^```(?:json)?\s*|\s*```$", "", text.strip(), flags=re.MULTILINE)
        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            log.debug("[scene-vlm] non-JSON response: %s", text[:200])
            return []
        dets = obj.get("detections", []) if isinstance(obj, dict) else []
        return dets if isinstance(dets, list) else []

    def _project_to_world(self, raw: list[dict]) -> list[Detection]:
        """Convert each detection's image bbox + approximate depth into a
        world-frame Pose3D + BBox3D. Algorithm:

          1. bbox center in pixels (cx, cy)
          2. unproject to camera frame using pinhole + depth_m
          3. compose with `chassis_pose` (robot in `map`) plus an
             assumed camera-relative pose (head-mounted, roughly 0,0,1m)

        Step 3 is intentionally crude in v1 (no real TF lookup); it's
        good enough to put cups in approximately the right map cell so
        relations work. TODO: consume tf2 once scene runs alongside
        rclpy on a real robot."""
        out: list[Detection] = []
        chassis = self.chassis_pose_fn()
        if chassis is None:
            rx, ry, rz, ryaw = 0.0, 0.0, 0.0, 0.0
        else:
            rx, ry, rz, ryaw = chassis
        cam_offset_z = 1.1  # head height-ish; configurable later

        K = self.intrinsics
        if K is None:
            if not self._missing_intrinsics_logged:
                log.warning("[scene-vlm] camera intrinsics unavailable; skipping projection")
                self._missing_intrinsics_logged = True
            return []

        for d in raw:
            try:
                cls = str(d.get("cls", "")).strip().lower()
                if not cls:
                    continue
                conf = float(d.get("confidence", 0.5))
                bbox = d.get("bbox_2d", [0, 0, K.width, K.height])
                if not (isinstance(bbox, list) and len(bbox) == 4):
                    continue
                x0, y0, x1, y1 = (float(v) for v in bbox)
                cx_px = 0.5 * (x0 + x1)
                cy_px = 0.5 * (y0 + y1)
                depth = float(d.get("approximate_depth_m", 1.5))
                if depth <= 0.05 or depth > 25.0:
                    depth = 1.5

                # Pinhole back-projection: camera frame X right, Y down, Z forward.
                X_c = (cx_px - K.cx) * depth / K.fx
                Y_c = (cy_px - K.cy) * depth / K.fy
                Z_c = depth

                # Camera→robot: assume camera looks +x of robot, mounted at +z=cam_offset_z.
                # Robot frame: x forward, y left, z up. Rotate camera (z forward, x right, y down)
                # into robot axes: robot.x = cam.z, robot.y = -cam.x, robot.z = -cam.y
                X_r = Z_c
                Y_r = -X_c
                Z_r = -Y_c + cam_offset_z

                # Robot→map: rotate by yaw, translate by chassis pose.
                cos_y = math.cos(ryaw)
                sin_y = math.sin(ryaw)
                X_m = rx + cos_y * X_r - sin_y * Y_r
                Y_m = ry + sin_y * X_r + cos_y * Y_r
                Z_m = rz + Z_r

                # bbox dimensions: rough scale from pixel size + depth, capped to sane values.
                px_w = max(1.0, x1 - x0)
                px_h = max(1.0, y1 - y0)
                size_x = min(2.0, max(0.05, px_w * depth / K.fx))
                size_z = min(2.0, max(0.05, px_h * depth / K.fy))
                size_y = 0.5 * (size_x + size_z)  # depth dimension is unobserved in v1

                out.append(Detection(
                    cls=_canon_class(cls),
                    pose=Pose3D(x=X_m, y=Y_m, z=Z_m, yaw=0.0, frame_id="map"),
                    bbox=BBox3D(size_x=size_x, size_y=size_y, size_z=size_z, yaw=0.0, frame_id="map"),
                    confidence=max(0.0, min(1.0, conf)),
                    source="vlm",
                ))
            except Exception:  # noqa: BLE001
                continue
        return out


_CLASS_ALIASES: dict[str, str] = {
    "desk": "table",
    "computer_desk": "table",
    "office_chair": "chair",
    "monitor_screen": "monitor",
    "screen": "monitor",
    "human": "person",
    "people": "person",
    "doorway": "door",
}


def _canon_class(cls: str) -> str:
    s = cls.strip().lower().replace(" ", "_").replace("-", "_")
    return _CLASS_ALIASES.get(s, s)
