# SPDX-License-Identifier: MulanPSL-2.0
"""VLM object detection: ask an OpenAI-compatible vision model (VLM_BASE_URL,
VLM_API_KEY, VLM_MODEL) for the objects in an RGB frame and project them into
the world frame. An unchanged frame reuses the last answer; failures back off
exponentially.
"""
from __future__ import annotations

import asyncio
import base64
import copy
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
from ..vision_cache import (
    FrameFingerprint,
    InferenceCounters,
    fingerprint_jpeg,
    frames_equivalent,
)

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
    """Pinhole intrinsics from the camera deployment (or its manifest fallback)."""
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


class VLMObjectDetector:
    """Polls ``rgb_fetcher``, asks the VLM, and hands ``Detection`` batches to
    ``on_detections``."""

    def __init__(
        self,
        *,
        rgb_fetcher: Callable[[], Optional[bytes | tuple[bytes, int | float]]],
        camera_to_world_fn: Callable[[], Optional[tuple[object, str]]],
        on_detections: Callable[[list[Detection]], Awaitable[None]],
        period_s: float = 3.0,
        intrinsics: Optional[_CamIntrinsics] = None,
        intrinsics_fn: Optional[Callable[[], Optional[_CamIntrinsics]]] = None,
        frame_change_threshold: Optional[float] = None,
        cache_max_age_s: Optional[float] = None,
        failure_backoff_base_s: Optional[float] = None,
        failure_backoff_max_s: Optional[float] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        # `rgb_fetcher` returns JPEG bytes, optionally with a delivery count so a
        # frozen stream cannot refresh objects.
        self.rgb_fetcher = rgb_fetcher
        self.camera_to_world_fn = camera_to_world_fn
        self.on_detections = on_detections
        self.period_s = period_s
        self.intrinsics = intrinsics
        self.intrinsics_fn = intrinsics_fn
        self._missing_intrinsics_logged = False
        self._missing_transform_logged = False
        self._task: Optional[asyncio.Task[None]] = None
        self._stop = asyncio.Event()
        self._clock = clock
        self.frame_change_threshold = min(1.0, max(0.0, self._setting(
            frame_change_threshold, "SCENE_VLM_FRAME_CHANGE_THRESHOLD", 0.01)))
        self.cache_max_age_s = max(0.0, self._setting(
            cache_max_age_s, "SCENE_VLM_CACHE_MAX_AGE_SEC", 120.0))
        default_backoff_base = max(period_s, 5.0) if math.isfinite(period_s) else 5.0
        self.failure_backoff_base_s = max(0.0, self._setting(
            failure_backoff_base_s, "SCENE_VLM_FAILURE_BACKOFF_BASE_SEC",
            default_backoff_base))
        self.failure_backoff_max_s = max(self.failure_backoff_base_s, self._setting(
            failure_backoff_max_s, "SCENE_VLM_FAILURE_BACKOFF_MAX_SEC", 60.0))
        self._last_success_frame: Optional[FrameFingerprint] = None
        self._last_success_detections: Optional[list[dict]] = None
        self._last_seen_delivery_count: Optional[int | float] = None
        self._last_published_delivery_count: Optional[int | float] = None
        self._last_success_at = 0.0
        self._failure_streak = 0
        self._retry_at = 0.0
        self._stats = InferenceCounters()

        self.base_url = (os.environ.get("VLM_BASE_URL") or os.environ.get("OPENAI_BASE_URL") or "").rstrip("/")
        self.api_key = os.environ.get("VLM_API_KEY") or os.environ.get("OPENAI_API_KEY") or ""
        self.model = os.environ.get("VLM_MODEL") or os.environ.get("OPENAI_MODEL") or "gpt-5.5"
        # Sent only when set; strict endpoints reject the unknown field.
        self.reasoning_effort = os.environ.get("VLM_REASONING_EFFORT", "").strip()
        if not self.api_key:
            log.warning("[scene-vlm] VLM_API_KEY not set; perception will be inert")

    @staticmethod
    def _env_float(key: str, default: float) -> float:
        try:
            return float(os.environ.get(key, str(default)))
        except ValueError:
            return default

    @classmethod
    def _setting(cls, value: Optional[float], env: str, default: float) -> float:
        """``value``, else the environment, else ``default`` when not finite."""
        v = value if value is not None else cls._env_float(env, default)
        return v if math.isfinite(v) else default

    @property
    def inference_counts(self) -> dict[str, int]:
        return self._stats.as_dict()

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
        """Process one frame. A cache hit on a newly delivered frame republishes
        the previous detections so the registry stays fresh."""
        sample = self.rgb_fetcher()
        if sample is None:
            return
        if isinstance(sample, tuple):
            jpeg_bytes, delivery_count = sample
        else:
            jpeg_bytes, delivery_count = sample, None
        if not jpeg_bytes:
            return
        now = self._clock()
        frame = fingerprint_jpeg(jpeg_bytes)
        unchanged = frames_equivalent(
            frame,
            self._last_success_frame,
            threshold=self.frame_change_threshold,
        )
        same_delivery = (
            delivery_count is not None
            and delivery_count == self._last_seen_delivery_count
        )
        cache_fresh = (
            self.cache_max_age_s > 0.0
            and now - self._last_success_at < self.cache_max_age_s
        )
        if unchanged and (same_delivery or cache_fresh):
            self._record_skip("unchanged-frame")
            self._last_seen_delivery_count = delivery_count
            if self._last_success_detections is not None and (
                delivery_count is None
                or delivery_count != self._last_published_delivery_count
            ):
                published = await self._publish_detections(
                    self._last_success_detections
                )
                if published:
                    self._last_published_delivery_count = delivery_count
            return

        retrying = self._failure_streak > 0
        if retrying and now < self._retry_at:
            self._record_skip("failure-backoff")
            return
        if retrying:
            self._stats.retried += 1

        jpeg_b64 = base64.b64encode(jpeg_bytes).decode("ascii")
        try:
            detections_json = await self._call_vlm(jpeg_b64)
        except Exception as e:  # noqa: BLE001
            log.warning("[scene-vlm] VLM call failed: %s: %s", type(e).__name__, e)
            self._record_failure(self._clock())
            return
        if detections_json is None:
            self._record_failure(self._clock())
            return

        self._last_success_frame = frame
        self._last_success_detections = copy.deepcopy(detections_json)
        self._last_seen_delivery_count = delivery_count
        self._last_success_at = self._clock()
        self._clear_failure()
        self._stats.processed += 1
        self._log_stats(logging.INFO, "processed")
        published = await self._publish_detections(detections_json)
        if published:
            self._last_published_delivery_count = delivery_count

    async def _publish_detections(self, raw: list[dict]) -> bool:
        """Project and publish; False leaves unprojectable output pending for a retry."""
        detections = self._project_to_world(raw)
        if raw and not detections:
            return False
        if detections:
            await self.on_detections(detections)
        return True

    def _record_skip(self, reason: str) -> None:
        self._stats.skipped += 1
        level = logging.INFO if self._stats.skipped % 25 == 0 else logging.DEBUG
        self._log_stats(level, reason)

    def _record_failure(self, now: float) -> None:
        self._failure_streak += 1
        exponent = min(self._failure_streak - 1, 10)
        delay = min(
            self.failure_backoff_max_s,
            self.failure_backoff_base_s * (2 ** exponent),
        )
        self._retry_at = now + delay
        self._stats.failed += 1
        self._log_stats(logging.WARNING, f"failed; retry_in={delay:.1f}s")

    def _clear_failure(self) -> None:
        self._failure_streak = 0
        self._retry_at = 0.0

    def _log_stats(self, level: int, reason: str) -> None:
        log.log(
            level,
            "[scene-vlm] inference stats: processed=%d skipped=%d "
            "retried=%d failed=%d reason=%s",
            self._stats.processed,
            self._stats.skipped,
            self._stats.retried,
            self._stats.failed,
            reason,
        )

    async def _call_vlm(self, jpeg_b64: str) -> Optional[list[dict]]:
        """One chat-completions call; the parsed ``detections`` list, or None."""
        if not self.base_url or not self.api_key:
            return None
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
                return None
            data = r.json()
            try:
                text = data["choices"][0]["message"]["content"]
            except (KeyError, IndexError):
                return None
        # Strip markdown fences if the model added them despite the prompt.
        text = re.sub(r"^```(?:json)?\s*|\s*```$", "", text.strip(), flags=re.MULTILINE)
        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            log.debug("[scene-vlm] non-JSON response: %s", text[:200])
            return None
        if not isinstance(obj, dict) or "detections" not in obj:
            return None
        dets = obj["detections"]
        if not isinstance(dets, list):
            return None
        valid = [item for item in dets if isinstance(item, dict)]
        if len(valid) != len(dets):
            log.debug(
                "[scene-vlm] dropped %d malformed detection item(s)",
                len(dets) - len(valid),
            )
        return valid

    def _project_to_world(self, raw: list[dict]) -> list[Detection]:
        """Back-project detections; nothing is admitted without intrinsics and a
        camera-to-world transform with a named frame."""
        import numpy as np

        out: list[Detection] = []
        K = self.intrinsics_fn() if self.intrinsics_fn is not None else self.intrinsics
        if K is None:
            if not self._missing_intrinsics_logged:
                log.warning("[scene-vlm] camera intrinsics unavailable; skipping projection")
                self._missing_intrinsics_logged = True
            return []
        self._missing_intrinsics_logged = False
        transform_state = self.camera_to_world_fn()
        if transform_state is None:
            if not self._missing_transform_logged:
                log.warning(
                    "[scene-vlm] camera pose/extrinsics unavailable; "
                    "withholding spatial detections"
                )
                self._missing_transform_logged = True
            return []
        self._missing_transform_logged = False
        transform, world_frame = transform_state
        world_frame = str(world_frame or "").strip()
        if not world_frame:
            return []
        transform = np.asarray(transform, dtype=np.float64)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            log.warning("[scene-vlm] invalid camera-to-world transform; skipping")
            return []

        for d in raw:
            try:
                label = str(d.get("cls", "")).strip().lower()
                if not label:
                    continue
                conf = float(d.get("confidence", 0.5))
                bbox = d.get("bbox_2d", [0, 0, K.width, K.height])
                if not (isinstance(bbox, list) and len(bbox) == 4):
                    continue
                x0, y0, x1, y1 = (float(v) for v in bbox)
                cx_px = 0.5 * (x0 + x1)
                cy_px = 0.5 * (y0 + y1)
                if d.get("approximate_depth_m") is None:
                    continue
                depth = float(d["approximate_depth_m"])
                if depth <= 0.05 or depth > 25.0:
                    continue

                # Pinhole back-projection: camera frame X right, Y down, Z forward.
                X_c = (cx_px - K.cx) * depth / K.fx
                Y_c = (cy_px - K.cy) * depth / K.fy
                Z_c = depth

                X_m, Y_m, Z_m, _ = transform @ np.array(
                    [X_c, Y_c, Z_c, 1.0],
                    dtype=np.float64,
                )

                # bbox dimensions: rough scale from pixel size + depth, capped to sane values.
                px_w = max(1.0, x1 - x0)
                px_h = max(1.0, y1 - y0)
                size_x = min(2.0, max(0.05, px_w * depth / K.fx))
                size_z = min(2.0, max(0.05, px_h * depth / K.fy))
                size_y = 0.5 * (size_x + size_z)  # depth dimension is unobserved in v1

                out.append(Detection(
                    label=_canon_class(label),
                    pose=Pose3D(
                        x=float(X_m),
                        y=float(Y_m),
                        z=float(Z_m),
                        yaw=0.0,
                        frame_id=world_frame,
                    ),
                    bbox=BBox3D(
                        size_x=size_x,
                        size_y=size_y,
                        size_z=size_z,
                        yaw=0.0,
                        frame_id=world_frame,
                    ),
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


def _canon_class(label: str) -> str:
    s = label.strip().lower().replace(" ", "_").replace("-", "_")
    return _CLASS_ALIASES.get(s, s)
