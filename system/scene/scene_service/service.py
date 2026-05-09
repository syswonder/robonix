# SPDX-License-Identifier: MulanPSL-2.0
"""scene_service entrypoint — wires the registry, ingest pollers,
relation engine, FastMCP server, and atlas registration together.

Capability owns atlas register / driver lifecycle / MCP HTTP / heartbeat
(`cap.bootstrap()` + `cap.use_mcp_app(mcp_tools.mcp)`); everything below
is scene-specific: registry + relations engine, ROS2 ingest hub,
VLM perception, web debug UI.
"""
from __future__ import annotations

import asyncio
import contextlib
import faulthandler
import json
import logging
import os
import signal
import time
from pathlib import Path
from typing import Any, Optional

import uvicorn  # used for the web debug UI; cap owns the MCP HTTP server

# torch + open3d C-extension calls can segfault on driver / kernel
# mismatches. Without faulthandler, exit 139 lands without a Python
# trace and we have to guess. Enable it as early as possible so the C
# stack lands in scene's stderr the moment a SIGSEGV/SIGFPE hits.
faulthandler.enable(all_threads=True)


from robonix_api import Capability, atlas  # noqa: E402
from robonix_api.atlas_types import Ros2Params, Transport  # noqa: E402

cap = Capability(id="scene", namespace="robonix/system/scene")

from . import mcp_tools
from . import web as web_ui
from .ingest.perception_concept_graphs import ConceptGraphsDetector
from .ingest.perception_vlm import VLMObjectDetector, _CamIntrinsics
from .ingest.ros_subscribers import (
    DEFAULT_WEBOTS_TIAGO_TOPICS,
    SubscribersHub,
    TopicSpec,
)
from .state import (
    BBox3D,
    ObjectRegistry,
    Pose3D,
    RelationEngine,
)
from .state.object_registry import now_unix


logging.basicConfig(
    level=os.environ.get("SCENE_LOG_LEVEL", "INFO").upper(),
    format="[scene-service] %(levelname)s %(message)s",
)
log = logging.getLogger("scene-service")


def _load_config() -> dict:
    """Read RBNX_CONFIG_FILE if set; otherwise return an empty config.
    Empty `observations` triggers auto-discovery against atlas — scene
    asks for every cap with a ROS2 topic_out interface and subscribes
    by contract leaf. The manifest only needs to populate
    `observations[]` to override / disable a kind for a deployment.
    """
    path = os.environ.get("RBNX_CONFIG_FILE")
    if not path:
        return {}
    try:
        text = Path(path).read_text()
        try:
            cfg = json.loads(text)
        except json.JSONDecodeError:
            import yaml  # PyYAML is in deps

            cfg = yaml.safe_load(text) or {}
    except Exception as e:  # noqa: BLE001
        log.warning("failed to read %s: %s; using empty config", path, e)
        return {}
    if not isinstance(cfg, dict):
        cfg = {}
    return cfg


# scene's input set is fixed: it knows exactly which contracts it consumes.
# Each tuple is (kind, contract_id, msg_type). At startup we ask atlas which
# of these any registered cap is currently providing over ROS2 and subscribe
# only to those. No leaf-based inference, no QueryContract round-trip — the
# contract id is the API and msg_type is the contract's wire shape promise.
#
# Adding a new sensor type to scene = appending one row here and writing the
# matching consumer. Third-party primitives don't get auto-picked-up under
# foreign namespaces; if you want scene to consume your data, declare it
# under one of these contract ids (which is the whole point of contracts).
_SCENE_CONTRACTS: list[tuple[str, str, str]] = [
    ("rgb",               "robonix/primitive/camera/rgb",        "Image"),
    ("depth",             "robonix/primitive/camera/depth",      "Image"),
    ("lidar2d",           "robonix/primitive/lidar/lidar",       "LaserScan"),
    ("lidar3d",           "robonix/primitive/lidar/pointcloud",  "PointCloud2"),
    ("camera_extrinsics", "robonix/primitive/camera/extrinsics", "TransformStamped"),
    ("pose",              "robonix/service/map/pose",            "PoseWithCovarianceStamped"),
    ("odom",              "robonix/service/map/odom",            "Odometry"),
    ("occupancy_grid",    "robonix/service/map/occupancy_grid",  "OccupancyGrid"),
]

# Optional manifest opt-out: kinds listed here are dropped even if atlas
# advertises them. Useful when a deployment doesn't want scene burning
# CPU on, say, a high-rate depth stream.
_DEFAULT_DISABLED_KINDS: frozenset[str] = frozenset()


# Transport pref: "ros2" (default) drives the rclpy ingest path that
# scene actually has wired today; "grpc" is a placeholder for the
# future streaming-RPC ingest path. Contracts themselves are
# transport-agnostic — `mode = "topic_out"` says "this is a
# unidirectional output stream" and a primitive may serve it over
# ROS2 OR gRPC. Scene picks the transport its ingest understands.
_TRANSPORTS: dict[str, Transport] = {
    "ros2": Transport.ROS2,
    "grpc": Transport.GRPC,
}


def _resolve_pb_transport(name: str) -> int:
    t = _TRANSPORTS.get(name.lower())
    if t is None:
        log.warning("[scene] unknown transport %r in config — defaulting to ros2", name)
        return int(Transport.ROS2)
    return int(t)


# Memo: dedup the "[scene] %r ← atlas: topic=..." log line so the
# 5-second auto-rediscover loop only logs when an endpoint actually
# changes. Keyed on (transport, contract_id) → resolution-signature.
_LAST_RESOLVED: dict[tuple, tuple] = {}


def _build_topic_specs(
    observations: list[dict], atlas_stub, transport: str
) -> list[TopicSpec]:
    """Two paths:

      A. Auto-discovery (default — when manifest's `observations[]` is
         empty / absent). Scene walks `_SCENE_CONTRACTS` and for each
         entry asks atlas which cap (if any) is declaring it over
         `transport`. msg_type comes from the same table — no
         QueryContract round-trip.

      B. Explicit overrides (when manifest provides `observations[]`).
         Each entry is `{kind, contract[, msg_type]}`. msg_type is
         taken from the entry, or from `_SCENE_CONTRACTS` if the
         contract is one scene already knows.

    `transport` is `"ros2"` (today's wired ingest) or `"grpc"` (future).
    Errors at any step skip that one entry but never fail bring-up.
    """
    pb_t = _resolve_pb_transport(transport)
    if observations:
        return _resolve_explicit(observations, atlas_stub, pb_t)
    return _resolve_auto(atlas_stub, pb_t)


def _resolve_auto(_unused, pb_transport: int) -> list[TopicSpec]:
    """Walk `_SCENE_CONTRACTS` and use `atlas.find` + `cap.connect`
    to resolve each contract. atlas hands out the endpoint via
    ConnectCapability; the cap framework also tracks the channel for
    teardown so we don't leak edges in atlas.

    Skipped contracts (no provider yet) come back via the reconciler
    every `period_s` — scene tolerates services that come up after it.

    `_unused` keeps the call signature stable for callers built around
    the old raw atlas_pb stub; the actual lookup goes through `atlas`.
    """
    transport = Transport(pb_transport)
    out: list[TopicSpec] = []
    for kind, contract_id, msg_type in _SCENE_CONTRACTS:
        if kind in _DEFAULT_DISABLED_KINDS:
            continue
        spec = _resolve_one_contract(transport, kind, contract_id, msg_type)
        if spec is not None:
            out.append(spec)
    return out


def _resolve_one_contract(
    transport: Transport,
    kind: str,
    contract_id: str,
    msg_type: str,
) -> Optional[TopicSpec]:
    """atlas.find(contract) → cap.connect → endpoint. Returns None when no
    cap currently advertises the contract over this transport."""
    recs = atlas.find(contract_id=contract_id, transport=transport)
    if not recs:
        return None
    rec = recs[0]
    try:
        ch = cap.connect(provider=rec, contract_id=contract_id, transport=transport)
    except Exception as e:  # noqa: BLE001
        log.warning(
            "[scene] cap.connect(%s/%s) failed: %s",
            rec.capability_id, contract_id, e,
        )
        return None
    endpoint = (ch.endpoint or "").strip()
    if not endpoint:
        ch.close()
        return None
    qos_profile = ""
    if isinstance(ch.params, Ros2Params):
        qos_profile = ch.params.qos_profile or ""
    # Only log on first resolution / change. The auto-discover loop
    # re-resolves every ~5s; spamming the same line every cycle is noise.
    sig = (endpoint, msg_type, qos_profile or "default", rec.capability_id)
    prev = _LAST_RESOLVED.get((transport, contract_id))
    if prev != sig:
        log.info(
            "[scene] %r ← atlas: topic=%s msg=%s qos=%s contract=%s cap=%s",
            kind, endpoint, msg_type, qos_profile or "default",
            contract_id, rec.capability_id,
        )
        _LAST_RESOLVED[(transport, contract_id)] = sig
    return TopicSpec(
        kind=kind,
        topic=endpoint,
        msg_type=msg_type,
        qos_profile=qos_profile or "default",
    )


def _resolve_explicit(
    observations: list[dict], atlas_stub, pb_transport: int
) -> list[TopicSpec]:
    """Manifest-driven override path. Each entry pairs a logical kind
    with a contract id; we go through the same single-contract resolver
    used by the auto path so behaviour is identical. msg_type comes
    from `_SCENE_CONTRACTS` (or the entry's own `msg_type` field if
    it's a brand-new kind not in the default list).
    """
    by_contract: dict[str, tuple[str, str]] = {
        cid: (k, mt) for (k, cid, mt) in _SCENE_CONTRACTS
    }
    out: list[TopicSpec] = []
    for entry in observations:
        kind = str(entry.get("kind", "")).lower()
        contract = str(entry.get("contract", ""))
        if not kind or not contract:
            log.warning(
                "[scene] observation %r: missing kind/contract; skipping", entry
            )
            continue
        msg_type = str(entry.get("msg_type", "")) or by_contract.get(contract, (kind, ""))[1]
        if not msg_type:
            log.warning(
                "[scene] observation %r: no msg_type — add it to the entry "
                "or to _SCENE_CONTRACTS in service.py", entry
            )
            continue
        spec = _resolve_one_contract(Transport(pb_transport), kind, contract, msg_type)
        if spec is not None:
            out.append(spec)
    return out


# ── Self-pose tracker ──────────────────────────────────────────────────────
class _SelfTracker:
    """Owns the `robot` SceneObject — created on first pose update from
    the atlas-resolved `service/map/pose` stream (the canonical map-
    frame pose contract), then EMA-updated. Never goes `missing` (we
    just stop refreshing if the upstream stops responding). Also
    exposes a sync `latest_xy_yaw` callback that the VLM detector
    consumes for camera-to-map projection.
    """

    def __init__(self, registry: ObjectRegistry) -> None:
        self.registry = registry
        self._latest: Optional[tuple[float, float, float, float]] = None
        self._object_id: Optional[str] = None
        # World frame name used for stamped outputs. Updated by the
        # pose loop from the localizer's `header.frame_id` so we never
        # hardcode a specific provider's frame name (`map` for rtabmap,
        # `world` for some mocap setups, etc.).
        self.world_frame_id: str = "map"

    def latest_xy_yaw(self) -> Optional[tuple[float, float, float, float]]:
        return self._latest

    async def on_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        self._latest = (x, y, z, yaw)
        wf = self.world_frame_id
        async with self.registry.lock():
            if (
                self._object_id is None or self._object_id not in self.registry._objects
            ):  # noqa: SLF001
                obj = self.registry.insert_object(
                    cls="robot",
                    pose=Pose3D(x=x, y=y, z=z, yaw=yaw, frame_id=wf),
                    bbox=BBox3D(size_x=0.6, size_y=0.6, size_z=1.5, frame_id=wf),
                    confidence=1.0,
                    now=now_unix(),
                    is_robot=True,
                    source="self",
                )
                self._object_id = obj.object_id
                log.info(
                    "[self] registered self-object %s (frame=%s)", self._object_id, wf
                )
            else:
                obj = self.registry.get_object(self._object_id)
                if obj is not None:
                    self.registry.update_object_pose(
                        obj,
                        Pose3D(x=x, y=y, z=z, yaw=yaw, frame_id=wf),
                        new_confidence=1.0,
                        now=now_unix(),
                        ema_pose=1.0,  # robot's own pose: hard-overwrite
                        ema_conf=1.0,
                    )


# ── Stale-tick: flip missing flag after grace period ───────────────────────
async def _stale_tick(registry: ObjectRegistry, *, period_s: float = 1.0) -> None:
    while True:
        async with registry.lock():
            flipped = registry.mark_stale(now_unix())
        if flipped:
            log.debug("marked %d object(s) missing (grace expired)", flipped)
        await asyncio.sleep(period_s)


async def _auto_discover_loop(
    *, atlas_stub, hub, transport: str, explicit: list[dict], period_s: float = 5.0
) -> None:
    """Background reconciler. Re-runs discovery every `period_s` and
    dynamically adds new (kind, topic) subscriptions as they appear.
    Keeps scene picking up mapping/nav outputs that come online minutes
    after scene started. Explicit observations skip this loop —
    they're static."""
    if explicit:
        return
    while True:
        try:
            await asyncio.sleep(period_s)
            current = hub.has_kinds()
            specs = _build_topic_specs(explicit, atlas_stub, transport)
            for spec in specs:
                if spec.kind not in current:
                    hub.add_spec(spec)
        except Exception as e:  # noqa: BLE001
            log.debug("[scene] auto-discover loop tick: %s", e)


# ── Wire ROS subscribers + downstream consumers ────────────────────────────
async def _start_ros_ingest(
    *,
    atlas_stub,
    registry: ObjectRegistry,
    self_tracker: "_SelfTracker",
    config: dict,
) -> tuple[SubscribersHub, Optional[Any], list[asyncio.Task]]:
    """Bring up the rclpy hub + the per-kind consumers (self-pose
    bridge, ConceptGraphs perception, VLM fallback). Returns
    (hub, detector_or_None, bg_tasks_for_shutdown).

    Each consumer is its own asyncio task so a hung perception call
    doesn't starve the pose updater, and vice versa.

    Detector preference:
      1. ConceptGraphsDetector — RGB + depth + camera_info present and
         YOLO-World/MobileSAM weights baked into the image. This is the
         only path that gives metric-accurate object positions.
      2. VLMObjectDetector — fallback when there is no depth stream.
         Approximate only; positions wobble.
      3. None — neither RGB nor depth available."""
    # Auto-discovery is a never-ending background concern: scene is a
    # system service that runs alongside primitives + other services
    # which may declare their ROS2 outputs at any time (mapping comes
    # up after primitives, a soma can hot-plug a new sensor, etc.).
    # Strategy:
    #   1. Wait until at least one matching contract appears (keeps
    #      retrying — no timeout, scene's whole job is to track these).
    #   2. Background reconciler keeps re-polling forever and adds new
    #      kinds to the hub as they show up.
    # Explicit `observations:` in the manifest skips both phases —
    # those references are static and authoritative.
    explicit = config.get("observations") or []
    transport = str(config.get("transport") or "ros2")
    specs = _build_topic_specs(explicit, atlas_stub, transport)
    if not specs and not explicit:
        attempt = 0
        while not specs:
            attempt += 1
            await asyncio.sleep(2.0)
            specs = _build_topic_specs(explicit, atlas_stub, transport)
            if specs:
                log.info(
                    "[scene] auto-discover: found %d topic(s) on attempt %d",
                    len(specs),
                    attempt,
                )
                break
            if attempt % 5 == 1:
                log.info(
                    "[scene] auto-discover attempt %d: 0 %s topics yet, retrying",
                    attempt,
                    transport,
                )

    if not specs:
        log.warning("no observation topics configured — registry will stay empty")
        specs = []
    hub = SubscribersHub(specs=specs)
    await hub.start()

    bg_tasks: list[asyncio.Task] = []

    # ── self-pose bridge ───────────────────────────────────────────────────
    # Polls hub.latest("pose") at 5 Hz and feeds the SelfTracker. Pose
    # callbacks fire ~10 Hz on /amcl_pose, so 5 Hz consumer is enough
    # to keep the registry's robot record fresh without flooding the
    # asyncio loop. Falls back to /odom when /amcl_pose isn't there.
    #
    # ALWAYS start this task — the loop itself probes hub.has(...) on
    # every tick, so it tolerates pose/odom showing up later (mapping
    # boots after scene; without this we'd skip task creation forever).
    bg_tasks.append(
        asyncio.create_task(
            _self_pose_loop(hub, self_tracker), name="scene-self-pose"
        )
    )

    # Perception startup races with camera primitive cap registration:
    # the chassis cap usually shows up first (its `pose`/`odom` topics
    # populate hub.specs at the initial discovery), but the camera
    # primitive's RGB/depth contracts may take a few extra seconds to
    # land in atlas. Wait up to a bounded window for them to appear via
    # the auto-discover reconciler loop, so we don't permanently lose
    # the ConceptGraphs path because of a startup race.
    perception_wait_s = float(os.environ.get("SCENE_PERCEPTION_WAIT_S", "30"))
    deadline = time.time() + perception_wait_s
    while time.time() < deadline and not (hub.has("rgb") and hub.has("depth")):
        # Pull fresh specs from atlas and add anything new.
        new_specs = _build_topic_specs(explicit, atlas_stub, transport)
        for spec in new_specs:
            if spec.kind not in hub.has_kinds():
                hub.add_spec(spec)
        if hub.has("rgb") and hub.has("depth"):
            log.info("[scene] perception-wait: rgb+depth now available")
            break
        await asyncio.sleep(2.0)

    # ── perception ─────────────────────────────────────────────────────────
    # ConceptGraphs path is *strongly* preferred: it owns metric-accurate
    # depth-backprojected poses. The VLM path stays as a no-depth fallback
    # but we log loudly when we fall into it because pose accuracy will
    # suffer.
    detector: Optional[Any] = None
    if hub.has("rgb") and hub.has("depth"):

        def _rgb_msg() -> Optional[Any]:
            msg, stamp, _ = hub.latest("rgb")
            if msg is None or stamp == 0.0:
                return None
            return msg

        def _depth_msg() -> Optional[Any]:
            msg, stamp, _ = hub.latest("depth")
            if msg is None or stamp == 0.0:
                return None
            return msg

        # Camera intrinsics: prefer a dedicated camera_info topic if a
        # primitive ever publishes one (TODO: subscribe to camera_info as
        # a separate kind). For now we fall back to the static webots
        # tiago intrinsics — same as the VLM path used. If a deployment
        # has different intrinsics it can override via
        # SCENE_CAMERA_INTRINSICS=fx,fy,cx,cy,w,h.
        def _cam_info() -> Optional[_CamIntrinsics]:
            return _resolved_cam_intrinsics()

        detector = ConceptGraphsDetector(
            rgb_fetcher_msg=_rgb_msg,
            depth_fetcher_msg=_depth_msg,
            camera_info_fetcher=_cam_info,
            chassis_pose_fn=self_tracker.latest_xy_yaw,
            world_frame_fn=lambda: getattr(self_tracker, "world_frame_id", "map"),
            on_detections=lambda dets: _ingest_detections(registry, dets),
            registry=registry,
            # Pass the hub so the detector can compose camera→world
            # from atlas-resolved contracts (`service/map/pose` ⊕
            # `primitive/camera/extrinsics`). tf2 is reserved for the
            # legacy fallback path.
            hub=hub,
        )
        await detector.start()
        log.info("[scene] perception: ConceptGraphsDetector (rgb+depth)")
    elif hub.has("rgb"):
        log.warning(
            "[scene] perception: no depth stream — falling back to "
            "VLMObjectDetector. Object positions will be approximate. "
            "Configure a depth topic to get metric-accurate poses."
        )

        def _rgb_jpeg() -> Optional[bytes]:
            msg, stamp, _ = hub.latest("rgb")
            if msg is None or stamp == 0.0:
                return None
            return _image_msg_to_jpeg(msg)

        detector = VLMObjectDetector(
            rgb_fetcher=_rgb_jpeg,
            chassis_pose_fn=self_tracker.latest_xy_yaw,
            on_detections=lambda dets: _ingest_detections(registry, dets),
            period_s=4.0,
        )
        await detector.start()
    else:
        log.warning("[scene] perception: no RGB stream — detector disabled")

    return hub, detector, bg_tasks


def _resolved_cam_intrinsics() -> _CamIntrinsics:
    """Camera intrinsics for the ConceptGraphs detector. Default is
    webots tiago head_front_camera at 640x480 (60° HFOV). Override
    via SCENE_CAMERA_INTRINSICS=fx,fy,cx,cy,w,h."""
    raw = os.environ.get("SCENE_CAMERA_INTRINSICS", "").strip()
    if raw:
        try:
            parts = [float(s) for s in raw.split(",")]
            if len(parts) >= 4:
                fx, fy, cx, cy = parts[:4]
                w = int(parts[4]) if len(parts) > 4 else 640
                h = int(parts[5]) if len(parts) > 5 else 480
                return _CamIntrinsics(width=w, height=h, fx=fx, fy=fy, cx=cx, cy=cy)
        except Exception:  # noqa: BLE001
            pass
    return _CamIntrinsics()


async def _self_pose_loop(hub: SubscribersHub, self_tracker: "_SelfTracker") -> None:
    """Feed SelfTracker the robot's world-frame pose, sourced through
    the `service/map/pose` (or fallback `service/map/odom`) atlas
    contract — i.e. whatever provider mapping/AMCL/mocap registered.

    Frame name comes from the message's `header.frame_id` (so the
    rest of scene's outputs stamp the same world frame the localizer
    is using), not a hardcoded `"map"` constant: a Ranger Mini deploy
    using a stack that publishes pose in `world` or `odom_combined`
    Just Works without scene caring.

    Falls back to tf2 only when neither contract is wired (legacy
    transition path; logged once).
    """
    fallback_warned = False
    while True:
        x = y = z = yaw = None
        frame_id: Optional[str] = None

        # Path A: SLAM-corrected pose contract (preferred — bounded drift).
        if hub.has("pose"):
            msg, stamp_unix, _count = hub.latest("pose")
            if msg is not None and stamp_unix > 0:
                p = (
                    msg.pose.pose
                    if hasattr(msg, "pose") and hasattr(msg.pose, "pose")
                    else msg.pose
                )
                q = p.orientation
                x = float(p.position.x)
                y = float(p.position.y)
                z = float(p.position.z)
                yaw = _quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
                frame_id = (
                    getattr(getattr(msg, "header", None), "frame_id", None) or None
                )

        # Path B: SLAM odom (smoothly varying — for high-rate trackers).
        if x is None and hub.has("odom"):
            msg, stamp_unix, _count = hub.latest("odom")
            if msg is not None and stamp_unix > 0:
                p = msg.pose.pose
                q = p.orientation
                x = float(p.position.x)
                y = float(p.position.y)
                z = float(p.position.z)
                yaw = _quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
                frame_id = (
                    getattr(getattr(msg, "header", None), "frame_id", None) or None
                )

        # Path C: tf2 fallback. Only when no pose contract is in atlas.
        if x is None:
            if not fallback_warned:
                log.warning(
                    "[scene] no pose contract resolved (service/map/pose, /odom). "
                    "Falling back to tf2 lookup; declare a pose provider in atlas to "
                    "remove this side-channel."
                )
                fallback_warned = True
            res = hub.lookup_xy_yaw("base_link", "map")
            if res is not None:
                x, y, z, yaw = res
                frame_id = "map"

        if x is not None:
            if frame_id:
                self_tracker.world_frame_id = frame_id  # type: ignore[attr-defined]
            await self_tracker.on_pose(
                x, y, z, yaw  # pyright: ignore[reportArgumentType]
            )
        await asyncio.sleep(0.2)


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    import math

    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _image_msg_to_jpeg(msg) -> Optional[bytes]:
    """sensor_msgs/Image → JPEG bytes. Webots Tiago publishes
    `bgr8` / `rgb8` for the head camera; we accept both. Falls back
    to None on unknown encodings (rather than throwing — VLM tick
    just skips that frame)."""
    try:
        import numpy as np  # noqa: F401
        from PIL import Image as PILImage

        h, w = msg.height, msg.width
        if h == 0 or w == 0:
            return None
        enc = (msg.encoding or "").lower()
        if enc == "rgb8":
            arr = _bytes_to_array(msg.data, h, w, 3)
            img = PILImage.fromarray(arr, "RGB")
        elif enc == "bgr8":
            arr = _bytes_to_array(msg.data, h, w, 3)
            arr = arr[..., ::-1]  # BGR → RGB
            img = PILImage.fromarray(arr, "RGB")
        elif enc in ("rgba8", "bgra8"):
            arr = _bytes_to_array(msg.data, h, w, 4)
            if enc == "bgra8":
                arr = arr[..., [2, 1, 0, 3]]
            img = PILImage.fromarray(arr, "RGBA").convert("RGB")
        elif enc == "mono8":
            arr = _bytes_to_array(msg.data, h, w, 1).reshape(h, w)
            img = PILImage.fromarray(arr, "L").convert("RGB")
        else:
            log.debug("[scene-vlm] unsupported encoding %r", enc)
            return None
        import io

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=80)
        return buf.getvalue()
    except Exception as e:  # noqa: BLE001
        log.debug("[scene-vlm] image→jpeg failed: %s", e)
        return None


def _bytes_to_array(data, h: int, w: int, channels: int):
    import numpy as np

    arr = np.frombuffer(bytes(data), dtype=np.uint8)
    return arr.reshape(h, w, channels)


async def _ingest_detections(registry: ObjectRegistry, detections):
    """Apply data association on the registry. Imported here to keep the
    top-level imports tidy."""
    from .state.data_assoc import associate

    if not detections:
        return
    async with registry.lock():
        matched, new = associate(registry, list(detections))
    if matched or new:
        log.info(
            "[detect] %d matched, %d new — registry: %s",
            len(matched),
            len(new),
            registry.stats(),
        )


# ── main ───────────────────────────────────────────────────────────────────
async def _run() -> None:
    config = _load_config()

    # Hand the FastMCP app from mcp_tools to Capability so it owns the
    # HTTP server. Capability auto-allocates the port (was hand-set to
    # 50106) and atlas-routes consumers via QueryCapabilities. The 6
    # tools were already decorated with @mcp_contract on mcp_tools.mcp
    # at import time; we still need to declare each on atlas — Capability
    # only auto-declares tools registered via @cap.mcp(), and we kept
    # the @mcp_contract pattern in mcp_tools to avoid a cyclic-import
    # rewrite. Manual declare per tool:
    cap.use_mcp_app(mcp_tools.mcp)

    # Wire state.
    registry = ObjectRegistry(grace_period_s=5.0)
    relations = RelationEngine(registry, period_s=1.0)
    await relations.start()
    self_tracker = _SelfTracker(registry)
    # mcp_tools v0 only needs the registry + the ROS hub (the latter is
    # supplied later in _start_ros_ingest); relations engine still runs
    # internally for ingest/web-UI but isn't exposed in the LLM tool
    # surface anymore.
    mcp_tools.attach_state(registry=registry)

    # Bring up atlas + lifecycle gRPC + MCP HTTP. Non-blocking; scene
    # keeps running its own asyncio event loop after this returns.
    cap.bootstrap()

    # Declare each scene MCP tool on atlas. Each handler has
    # `_robonix_*` attrs stashed by @mcp_contract — re-use them so the
    # description / JSON schema stay in sync with the codegen types.
    for fn in (
        mcp_tools.list_objects,
        mcp_tools.goal_near,
    ):
        cid = getattr(fn, "_robonix_contract_id", None)
        if cid is None:
            log.warning(
                "scene tool %s missing _robonix_contract_id; skipping", fn.__name__
            )
            continue
        in_cls = getattr(fn, "_robonix_input_cls", None)
        schema = json.dumps(in_cls.json_schema()) if in_cls else "{}"
        cap.declare_mcp(
            cid,
            cap.mcp_endpoint,
            description=(fn.__doc__ or "").strip(),
            input_schema_json=schema,
        )
    log.info("scene declared 2 MCP tools at %s", cap.mcp_endpoint)

    # ROS2 ingest hub + downstream consumers (self-pose, perception).
    # _start_ros_ingest still wants a raw atlas stub for QueryCapabilities;
    # cap exposes its underlying atlas client via _atlas.stub for these
    # cases that pre-date the Capability API.
    stub = cap._atlas.stub
    hub, perception, ingest_bg = await _start_ros_ingest(
        atlas_stub=stub,
        registry=registry,
        self_tracker=self_tracker,
        config=config,
    )
    # Now that the hub exists, hand it to mcp_tools so goal_near BFS can
    # read the occupancy grid. (Earlier attach_state call set registry
    # only; this one re-binds with the hub — attach_state is intentionally
    # cheap and idempotent.)
    mcp_tools.attach_state(registry=registry, hub=hub)
    bg_tasks = [
        asyncio.create_task(_stale_tick(registry), name="scene-stale-tick"),
        # Background reconciler: keeps scene's hub adding subscriptions
        # for new ROS2 topic_outs that appear on atlas after start
        # (mapping comes up after scene; same pattern for any future
        # service that publishes a contract scene knows about).
        asyncio.create_task(
            _auto_discover_loop(
                atlas_stub=stub,
                hub=hub,
                transport=str(config.get("transport") or "ros2"),
                explicit=(config.get("observations") or []),
            ),
            name="scene-auto-discover",
        ),
        *ingest_bg,
    ]

    # Web debug UI on a separate port — top-down 2D canvas + objects
    # table + robot pose. Lives in the same asyncio loop as the rest
    # of scene so registry reads are local. Set `web_port: 0` in the
    # deploy-manifest scene block to disable. SCENE_WEB_PORT env is
    # the override of last resort.
    web_port = int(
        int(config.get("web_port") or "0")
        if config.get("web_port") is not None and config.get("web_port") != ""
        else int(os.environ.get("SCENE_WEB_PORT", "50107") or "")
    )
    web_task = None
    web_server: uvicorn.Server | None = None
    if web_port > 0:
        web_app = web_ui.make_app(
            registry=registry,
            relations=relations,
            hub=hub,
            detector=perception,
        )
        web_uv = uvicorn.Config(
            app=web_app,
            host="0.0.0.0",
            port=web_port,
            log_level="warning",
        )
        web_server = uvicorn.Server(web_uv)
        web_task = asyncio.create_task(web_server.serve(), name="scene-web-http")
        log.info("web UI on http://0.0.0.0:%d", web_port)

    log.info(
        "scene up; cap=%s mcp=%s observations=%d",
        cap.id,
        cap.mcp_endpoint,
        len(config.get("observations", [])),
    )

    # Wait for SIGTERM/SIGINT.
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        with contextlib.suppress(NotImplementedError):
            loop.add_signal_handler(sig, stop.set)
    await stop.wait()
    log.info("shutdown signal received; tearing down")

    # Tear down ingest first so we stop mutating the registry…
    if perception is not None:
        with contextlib.suppress(Exception):
            await perception.stop()
    with contextlib.suppress(Exception):
        await hub.stop()
    await relations.stop()
    for t in bg_tasks:
        t.cancel()
    if web_server is not None:
        web_server.should_exit = True

    # Capability owns the gRPC server, MCP HTTP, heartbeat — stop them.
    cap._teardown()


def main() -> None:
    try:
        asyncio.run(_run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
