# SPDX-License-Identifier: MulanPSL-2.0
"""scene_service entrypoint — wires the registry, ingest pollers,
relation engine, FastMCP server, and atlas registration together.

Capability owns atlas register / driver lifecycle / MCP HTTP / heartbeat
(`scene.bootstrap()` + `scene.use_mcp_app(mcp_tools.mcp)`); everything below
is scene-specific: registry + geometric relation loop, ROS2 ingest hub,
VLM perception + scene-graph enrichment, web debug UI.
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


from robonix_api import ATLAS, Service  # noqa: E402
from robonix_api.atlas_types import Ros2Params, Transport  # noqa: E402

scene = Service(id="scene", namespace="robonix/system/scene")

from . import mcp_tools
from . import web as web_ui
from .annotations import AnnotationStore
from .ingest.capabilities import plan_perception
from .map_binding import MapBinding, choose_map_binding, read_latched_lifecycle
from .object_watchdog import ObjectWatchdog
from .map_meta import MapMetaStore
from .robot_geometry import RobotGeometryState, reconcile_robot_geometry
from .ingest.perception_concept_graphs import ConceptGraphsDetector
from .ingest.perception_vlm import VLMObjectDetector, _CamIntrinsics
from .ingest.ros_subscribers import (
    SubscribersHub,
    TopicSpec,
)
from .state import (
    BBox3D,
    ObjectRegistry,
    Pose3D,
)
from .state.object_registry import now_unix
from .web_binding import resolve_web_host


logging.basicConfig(
    level=os.environ.get("SCENE_LOG_LEVEL", "INFO").upper(),
    format="[scene-service] %(levelname)s %(message)s",
)
log = logging.getLogger("scene-service")


def _load_config() -> dict:
    """Read RBNX_CONFIG_FILE if set; otherwise return an empty config.

    An explicitly requested config is safety-relevant input. Unreadable,
    malformed, empty, or non-mapping content must stop startup instead of
    silently widening bind addresses and other settings to legacy defaults.

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
    except OSError as exc:
        raise RuntimeError(f"failed to read explicit Scene config {path}") from exc
    try:
        cfg = json.loads(text)
    except json.JSONDecodeError:
        import yaml  # PyYAML is in deps

        try:
            cfg = yaml.safe_load(text)
        except yaml.YAMLError as exc:
            raise RuntimeError(
                f"failed to parse explicit Scene config {path}"
            ) from exc
    if not isinstance(cfg, dict):
        raise ValueError(f"explicit Scene config root must be a mapping: {path}")
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
    ("lidar3d",           "robonix/primitive/lidar/lidar3d",     "PointCloud2"),
    ("camera_extrinsics", "robonix/primitive/camera/extrinsics", "TransformStamped"),
    ("intrinsics",        "robonix/primitive/camera/intrinsics", "CameraInfo"),
    ("pose",              "robonix/service/map/pose",            "PoseWithCovarianceStamped"),
    # Mapping may reuse external chassis odometry and then intentionally does
    # not redeclare map/odom. Consume the primitive's canonical continuous
    # odometry contract directly; map-corrected pose still comes from TF/pose.
    ("odom",              "robonix/primitive/chassis/odom",      "Odometry"),
    ("occupancy_grid",    "robonix/service/map/occupancy_grid",  "OccupancyGrid"),
    # Latched {map_id, mode, generation} broadcast from mapping. Startup
    # binding is probed separately (_discover_map_binding, before the hub
    # exists); this hub subscription feeds the runtime mismatch watcher
    # (_lifecycle_watch). Needs the generated `map` interface package
    # (ros2_idl overlay) — the hub skips the row gracefully when missing.
    ("map_lifecycle",     "robonix/service/map/lifecycle",       "MapLifecycle"),
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
    """Walk `_SCENE_CONTRACTS` and use `ATLAS.find_capability` + `connect_capability`
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
    """ATLAS.find_capability(contract) → connect_capability → endpoint. Returns None when no
    cap currently advertises the contract over this transport."""
    caps = ATLAS.find_capability(contract_id=contract_id, transport=transport)
    if not caps:
        return None
    cap_view = caps[0]
    try:
        ch = scene.connect_capability(cap_view, contract_id, transport)
    except Exception as e:  # noqa: BLE001
        log.warning(
            "[scene] connect_capability(%s/%s) failed: %s",
            cap_view.provider_id, contract_id, e,
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
    sig = (endpoint, msg_type, qos_profile or "default", cap_view.provider_id)
    prev = _LAST_RESOLVED.get((transport, contract_id))
    if prev != sig:
        log.info(
            "[scene] %r ← atlas: topic=%s msg=%s qos=%s contract=%s cap=%s",
            kind, endpoint, msg_type, qos_profile or "default",
            contract_id, cap_view.provider_id,
        )
        _LAST_RESOLVED[(transport, contract_id)] = sig
    return TopicSpec(
        kind=kind,
        topic=endpoint,
        msg_type=msg_type,
        qos_profile=qos_profile or "default",
    )


def _discover_map_binding(wait_s: float) -> Optional[dict]:
    """Probe mapping's latched lifecycle broadcast for the startup binding.

    Adaptive cost: polls atlas for the `robonix/service/map/lifecycle`
    contract for at most `wait_s` (the normal full-boot order starts scene
    BEFORE mapping, so the contract is usually absent and this returns in
    ~`wait_s`); once the contract resolves — the scene-restart-while-
    mapping-runs case — the latched sample arrives immediately, with a 5 s
    ceiling as safety. Returns {map_id, mode, generation} or None; blocking
    is fine here, it runs before anything else is wired. `wait_s <= 0`
    disables the probe (unit tests / ROS-less runs)."""
    if wait_s <= 0:
        return None
    deadline = time.monotonic() + wait_s
    while True:
        try:
            spec = _resolve_one_contract(
                Transport.ROS2, "map_lifecycle",
                "robonix/service/map/lifecycle", "MapLifecycle",
            )
        except Exception as e:  # noqa: BLE001
            # Probe runs before bootstrap; if atlas isn't reachable yet a
            # wire error must degrade to static binding, not kill startup.
            log.warning("[scene] lifecycle probe: atlas query failed (%s) — "
                        "falling back to static map binding", e)
            return None
        if spec is not None:
            sample = read_latched_lifecycle(spec.topic, timeout_s=5.0)
            if sample is None:
                log.warning(
                    "[scene] lifecycle contract resolved (%s) but no latched "
                    "sample within 5s — falling back to static map binding",
                    spec.topic,
                )
            return sample
        if time.monotonic() >= deadline:
            return None
        time.sleep(0.5)


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

    def __init__(
        self,
        registry: ObjectRegistry,
        robot_geometry: RobotGeometryState,
    ) -> None:
        """Track the robot pose and represent its Soma-owned footprint."""
        self.registry = registry
        self.robot_geometry = robot_geometry
        self._latest: Optional[tuple[float, float, float, float]] = None
        self._object_id: Optional[str] = None
        # World frame name used for stamped outputs. Updated by the
        # pose loop from the localizer's `header.frame_id` so we never
        # hardcode a specific provider's frame name (`map` for rtabmap,
        # `world` for some mocap setups, etc.).
        self.world_frame_id: str = ""

    def latest_xy_yaw(self) -> Optional[tuple[float, float, float, float]]:
        return self._latest

    async def on_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        self._latest = (x, y, z, yaw)
        wf = self.world_frame_id
        footprint = self.robot_geometry.current()
        if not wf or footprint is None:
            return
        async with self.registry.lock():
            if (
                self._object_id is None or self._object_id not in self.registry._objects
            ):  # noqa: SLF001
                obj = self.registry.insert_object(
                    cls="robot",
                    pose=Pose3D(x=x, y=y, z=z, yaw=yaw, frame_id=wf),
                    bbox=BBox3D(
                        size_x=footprint.size_x_m,
                        size_y=footprint.size_y_m,
                        size_z=0.0,
                        frame_id=wf,
                    ),
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
                    obj.bbox = BBox3D(
                        size_x=footprint.size_x_m,
                        size_y=footprint.size_y_m,
                        size_z=0.0,
                        yaw=yaw,
                        frame_id=wf,
                    )
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
    pose_max_age_s = float(
        config.get("pose_max_age_s")
        or os.environ.get("SCENE_POSE_MAX_AGE_S")
        or 2.0
    )
    if pose_max_age_s <= 0.0:
        raise ValueError("pose_max_age_s must be greater than zero")

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
            _self_pose_loop(
                hub,
                self_tracker,
                pose_max_age_s=pose_max_age_s,
            ),
            name="scene-self-pose",
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
    # Which perception tier the current hardware supports is decided by the
    # capability probe, not inline here: `plan.detector` routes to the
    # ConceptGraphs (metric), VLM (visual), or no (geometric) path. The
    # metric path is strongly preferred — it owns depth-backprojected
    # poses; the others are named, logged degradations, not silent ones.
    plan = plan_perception(hub)
    log.info("[scene] perception plan: %s", plan.summary())
    intrinsics_fallback = _scene_intrinsics_fallback(config.get("intrinsics_fallback"))
    detector: Optional[Any] = None
    camera_frame = str(
        config.get("camera_frame")
        or os.environ.get("SCENE_CAMERA_FRAME")
        or ""
    ).strip()
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

    def _active_camera_frame() -> str:
        if camera_frame:
            return camera_frame
        msg = _rgb_msg()
        return str(
            getattr(getattr(msg, "header", None), "frame_id", "") or ""
        ).strip()

    def _active_base_frame() -> str:
        footprint = self_tracker.robot_geometry.current()
        return footprint.base_frame if footprint is not None else ""

    if plan.detector == "concept_graphs":

        # Prefer the live `primitive/camera/intrinsics` contract. Deployments
        # without a reliable CameraInfo stream may opt in via an explicit
        # `intrinsics_fallback` in the scene config. This still runs the real
        # RGB-D ConceptGraphs path; it only supplies K for back-projection when
        # the contract has not delivered a usable sample yet.
        intrinsics_logged = {"ok": False, "bad": False, "fallback": False}

        def _cam_info() -> Optional[_CamIntrinsics]:
            if hub.has("intrinsics"):
                msg, stamp, _ = hub.latest("intrinsics")
                if msg is not None and stamp > 0.0:
                    info_frame = str(
                        getattr(getattr(msg, "header", None), "frame_id", "") or ""
                    ).strip()
                    expected_frame = _active_camera_frame()
                    if not expected_frame or info_frame != expected_frame:
                        if not intrinsics_logged["bad"]:
                            log.warning(
                                "[scene] intrinsics frame mismatch: "
                                "CameraInfo=%s active_camera=%s; withholding "
                                "contract calibration",
                                info_frame or "unknown",
                                expected_frame or "unknown",
                            )
                            intrinsics_logged["bad"] = True
                        msg = None
                if msg is not None and stamp > 0.0:
                    k = _cam_info_to_intrinsics(msg)
                    if k is not None:
                        if not intrinsics_logged["ok"]:
                            log.info(
                                "[scene] camera intrinsics from contract: "
                                "fx=%.1f fy=%.1f cx=%.1f cy=%.1f %dx%d",
                                k.fx, k.fy, k.cx, k.cy, k.width, k.height,
                            )
                            intrinsics_logged["ok"] = True
                        return k
                    # Contract is published but the CameraInfo K is zero/garbage —
                    # distinct from "no contract"; warn once so a miscalibrated
                    # publisher is visible rather than silently stalling perception.
                    if not intrinsics_logged["bad"]:
                        log.warning(
                            "[scene] intrinsics contract published but CameraInfo K "
                            "is unusable — detector will use configured fallback if available"
                        )
                        intrinsics_logged["bad"] = True
            if intrinsics_fallback is not None:
                source, k = intrinsics_fallback
                if not intrinsics_logged["fallback"]:
                    log.warning(
                        "[scene] camera intrinsics fallback: %s "
                        "fx=%.1f fy=%.1f cx=%.1f cy=%.1f %dx%d",
                        source, k.fx, k.fy, k.cx, k.cy, k.width, k.height,
                    )
                    intrinsics_logged["fallback"] = True
                return k
            return None

        detector = ConceptGraphsDetector(
            rgb_fetcher_msg=_rgb_msg,
            depth_fetcher_msg=_depth_msg,
            camera_info_fetcher=_cam_info,
            world_frame_fn=lambda: self_tracker.world_frame_id,
            on_detections=lambda dets: _ingest_detections(registry, dets),
            registry=registry,
            robot_base_frame_fn=_active_base_frame,
            # Pass the hub so the detector can compose camera→world from
            # Atlas-resolved pose and camera-extrinsics contracts.
            hub=hub,
            # Detection cadence. The default 0.6 s keeps objects fresh but runs
            # YOLO+CLIP on the GPU continuously; on a shared Jetson GPU that
            # starves co-located GPU work (e.g. FunASR ASR), making voice slow.
            # Raise SCENE_DETECT_PERIOD_S (e.g. 2.0) to free the GPU when running
            # speech + perception together.
            period_s=float(os.environ.get("SCENE_DETECT_PERIOD_S", "") or 0.6),
            pose_max_age_s=pose_max_age_s,
            camera_frame=camera_frame,
        )
        await detector.start()
        log.info("[scene] perception: ConceptGraphsDetector (rgb+depth)")
    elif plan.detector == "vlm":
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

        # Resolve the contract K at projection time because CameraInfo often
        # arrives after the service starts. Fall back only to the same explicit
        # deployment calibration accepted by the metric path; never invent K.
        def _vlm_intrinsics() -> Optional[_CamIntrinsics]:
            if hub.has("intrinsics"):
                msg, stamp, _ = hub.latest("intrinsics")
                if msg is not None and stamp > 0.0:
                    info_frame = str(
                        getattr(getattr(msg, "header", None), "frame_id", "") or ""
                    ).strip()
                    if info_frame == _active_camera_frame():
                        intrinsics = _cam_info_to_intrinsics(msg)
                        if intrinsics is not None:
                            return intrinsics
                    else:
                        log.warning(
                            "[scene] VLM intrinsics frame mismatch: "
                            "CameraInfo=%s active_camera=%s",
                            info_frame or "unknown",
                            _active_camera_frame() or "unknown",
                        )
            if intrinsics_fallback is not None:
                return intrinsics_fallback[1]
            return None

        detector = VLMObjectDetector(
            rgb_fetcher=_rgb_jpeg,
            camera_to_world_fn=lambda: _camera_to_world_from_contracts(
                hub,
                base_frame=_active_base_frame(),
                camera_frame=_active_camera_frame(),
                expected_world_frame=self_tracker.world_frame_id,
                pose_max_age_s=pose_max_age_s,
            ),
            on_detections=lambda dets: _ingest_detections(registry, dets),
            period_s=4.0,
            intrinsics_fn=_vlm_intrinsics,
        )
        await detector.start()
    else:
        # geometric tier: no camera. Object detection is off, but the
        # occupancy grid + goal_near BFS stay available, so navigation-
        # style queries still work — this is a degraded mode, not a fault.
        log.warning(
            "[scene] perception: geometric tier — no camera wired; object "
            "detection disabled (occupancy_grid + goal_near remain available)"
        )

    return hub, detector, bg_tasks


def _scene_intrinsics_fallback(raw: Any) -> Optional[tuple[str, _CamIntrinsics]]:
    """Return an explicitly configured camera intrinsics fallback.

    Real hardware should publish `primitive/camera/intrinsics`. The fallback is
    opt-in and must include the full K because an unreviewed calibration silently
    moves 3D objects.
    """
    if raw in (None, "", False):
        return None
    if isinstance(raw, str):
        if raw.lower() in {"none", "disabled", "false"}:
            return None
        log.warning("[scene] ignoring incomplete intrinsics_fallback=%r", raw)
        return None
    if not isinstance(raw, dict):
        log.warning("[scene] ignoring invalid intrinsics_fallback=%r", raw)
        return None

    cfg = raw
    source = str(cfg.get("source") or cfg.get("name") or "configured")
    if source.lower() in {"none", "disabled", "false"}:
        return None
    required = ("width", "height", "fx", "fy", "cx", "cy")
    if any(cfg.get(name) is None for name in required):
        log.warning("[scene] ignoring incomplete intrinsics_fallback=%r", raw)
        return None

    def _num(name: str) -> float:
        try:
            return float(cfg[name])
        except (TypeError, ValueError):
            return 0.0

    def _integer(name: str) -> int:
        try:
            return int(cfg[name])
        except (TypeError, ValueError):
            return 0

    k = _CamIntrinsics(
        width=_integer("width"),
        height=_integer("height"),
        fx=_num("fx"),
        fy=_num("fy"),
        cx=_num("cx"),
        cy=_num("cy"),
    )
    if min(k.width, k.height, k.fx, k.fy, k.cx, k.cy) <= 0:
        log.warning("[scene] ignoring incomplete intrinsics_fallback=%r", raw)
        return None
    return source, k


def _cam_info_to_intrinsics(msg: Any) -> Optional[_CamIntrinsics]:
    """Convert a sensor_msgs/CameraInfo into _CamIntrinsics.

    Reads the 3x3 row-major K matrix (`k[0]=fx`, `k[2]=cx`, `k[4]=fy`,
    `k[5]=cy`) plus width/height. ROS2 exposes the field as lowercase
    `k`; we also accept `K` for safety. Returns None when any of
    fx/fy/cx/cy/width/height is non-positive — an all-zero or partially
    populated CameraInfo carries no usable geometry, and the caller
    treats None as "wait for valid intrinsics", never as "use a default".
    The except is narrow on purpose: a genuinely unexpected error should
    surface, not be swallowed as if intrinsics were merely missing."""
    try:
        # `k` is the row-major 3x3 intrinsics. rclpy delivers it as a numpy
        # ndarray, so DON'T use `a or b` to pick the field — `bool(ndarray)`
        # on a multi-element array raises ValueError ("truth value ...
        # ambiguous"), which the except below would swallow as "no
        # intrinsics", stalling perception forever on a perfectly valid K.
        # Select the field with explicit None checks instead.
        k_field = getattr(msg, "k", None)
        if k_field is None:
            k_field = getattr(msg, "K", None)
        k = list(k_field) if k_field is not None else []
        w = int(getattr(msg, "width", 0))
        h = int(getattr(msg, "height", 0))
        if len(k) < 6 or min(k[0], k[4], k[2], k[5]) <= 0 or w <= 0 or h <= 0:
            return None
        return _CamIntrinsics(
            width=w,
            height=h,
            fx=float(k[0]),
            fy=float(k[4]),
            cx=float(k[2]),
            cy=float(k[5]),
        )
    except (AttributeError, TypeError, ValueError, IndexError):
        return None


async def _self_pose_loop(
    hub: SubscribersHub,
    self_tracker: "_SelfTracker",
    *,
    pose_max_age_s: float,
) -> None:
    """Feed SelfTracker the robot's world-frame pose, sourced through
    the `service/map/pose` (or fallback `service/map/odom`) atlas
    contract — i.e. whatever provider mapping/AMCL/mocap registered.

    Frame name comes from the message's `header.frame_id` (so the
    rest of scene's outputs stamp the same world frame the localizer
    is using), not a hardcoded `"map"` constant: a deployment
    publishing pose in `world` or `odom_combined`
    Just Works without scene caring.

    Samples without an explicit source frame are ignored. Scene must not
    invent a frame for spatial state.
    """
    missing_frame_warned = False
    stale_warned = False
    while True:
        x = y = z = yaw = None
        frame_id: Optional[str] = None

        # Path A: SLAM-corrected pose contract (preferred — bounded drift).
        if hub.has("pose"):
            msg, stamp_unix, _count = hub.latest("pose")
            if (
                msg is not None
                and stamp_unix > 0
                and time.time() - stamp_unix <= pose_max_age_s
            ):
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
            footprint = self_tracker.robot_geometry.current()
            child_frame = str(
                getattr(msg, "child_frame_id", "") or ""
            ).strip() if msg is not None else ""
            if (
                msg is not None
                and stamp_unix > 0
                and time.time() - stamp_unix <= pose_max_age_s
                and footprint is not None
                and child_frame == footprint.base_frame
            ):
                p = msg.pose.pose
                q = p.orientation
                x = float(p.position.x)
                y = float(p.position.y)
                z = float(p.position.z)
                yaw = _quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
                frame_id = (
                    getattr(getattr(msg, "header", None), "frame_id", None) or None
                )

        if x is not None and frame_id:
            self_tracker.world_frame_id = frame_id
            await self_tracker.on_pose(
                x, y, z, yaw  # pyright: ignore[reportArgumentType]
            )
            missing_frame_warned = False
            stale_warned = False
        elif x is not None and not missing_frame_warned:
            log.warning(
                "[scene] pose sample has no header.frame_id; spatial self state "
                "is withheld until the provider publishes its coordinate frame"
            )
            missing_frame_warned = True
        elif not stale_warned:
            latest_age = min(
                (
                    time.time() - stamp
                    for kind in ("pose", "odom")
                    if hub.has(kind)
                    for _msg, stamp, _count in [hub.latest(kind)]
                    if stamp > 0.0
                ),
                default=0.0,
            )
            if latest_age > pose_max_age_s:
                log.warning(
                    "[scene] latest pose/odometry sample is %.2fs old; "
                    "spatial self updates are withheld (limit %.2fs)",
                    latest_age,
                    pose_max_age_s,
                )
                stale_warned = True
        await asyncio.sleep(0.2)


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    import math

    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _camera_to_world_from_contracts(
    hub: SubscribersHub,
    *,
    base_frame: str,
    camera_frame: str,
    expected_world_frame: str,
    pose_max_age_s: float,
):
    """Compose camera-to-world only from one validated contract frame chain."""
    import numpy as np

    def matrix(message, *, transform: bool):
        value = message.transform if transform else (
            message.pose.pose
            if hasattr(message, "pose") and hasattr(message.pose, "pose")
            else message.pose
        )
        translation = value.translation if transform else value.position
        q = value.rotation if transform else value.orientation
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        norm = (x * x + y * y + z * z + w * w) ** 0.5
        if norm <= 1e-12:
            return None
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        out = np.array(
            [
                [
                    1 - 2 * (y * y + z * z),
                    2 * (x * y - z * w),
                    2 * (x * z + y * w),
                    float(translation.x),
                ],
                [
                    2 * (x * y + z * w),
                    1 - 2 * (x * x + z * z),
                    2 * (y * z - x * w),
                    float(translation.y),
                ],
                [
                    2 * (x * z - y * w),
                    2 * (y * z + x * w),
                    1 - 2 * (x * x + y * y),
                    float(translation.z),
                ],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        return out if np.all(np.isfinite(out)) else None

    base_frame = str(base_frame or "").strip()
    camera_frame = str(camera_frame or "").strip()
    expected_world_frame = str(expected_world_frame or "").strip()
    if not base_frame or not camera_frame or not expected_world_frame:
        return None

    pose_matrix = None
    world_frame = ""
    for kind in ("pose", "odom"):
        if not hub.has(kind):
            continue
        message, stamp_unix, _count = hub.latest(kind)
        if message is None or stamp_unix <= 0:
            continue
        if pose_max_age_s > 0.0 and time.time() - stamp_unix > pose_max_age_s:
            continue
        world_frame = str(
            getattr(getattr(message, "header", None), "frame_id", "") or ""
        ).strip()
        if world_frame != expected_world_frame:
            continue
        if kind == "odom":
            child_frame = str(
                getattr(message, "child_frame_id", "") or ""
            ).strip()
            if child_frame != base_frame:
                continue
        pose_matrix = matrix(message, transform=False)
        if pose_matrix is not None:
            break
    if pose_matrix is None or not hub.has("camera_extrinsics"):
        return None
    extrinsics, stamp_unix, _count = hub.latest("camera_extrinsics")
    if extrinsics is None or stamp_unix <= 0:
        return None
    parent_frame = str(
        getattr(getattr(extrinsics, "header", None), "frame_id", "") or ""
    ).strip()
    child_frame = str(getattr(extrinsics, "child_frame_id", "") or "").strip()
    if parent_frame != base_frame or child_frame != camera_frame:
        return None
    extrinsics_matrix = matrix(extrinsics, transform=True)
    if extrinsics_matrix is None:
        return None
    return pose_matrix @ extrinsics_matrix, world_frame


def _image_msg_to_jpeg(msg) -> Optional[bytes]:
    """sensor_msgs/Image -> JPEG bytes.

    Accepts common RGB/BGR encodings and returns None on unknown encodings
    rather than throwing; the VLM tick can skip that frame.
    """
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


def _log_bg_task_exit(task: "asyncio.Task") -> None:
    """Done-callback for scene's fire-and-forget background tasks: log any
    exception at ERROR the moment the task dies, instead of letting asyncio
    sit on it until interpreter shutdown."""
    if task.cancelled():
        return
    exc = task.exception()
    if exc is not None:
        log.error("[scene] background task %r died: %r", task.get_name(), exc)


async def _lifecycle_watch(
    hub: SubscribersHub,
    binding: MapBinding,
    anno_store: Optional[AnnotationStore] = None,
    *,
    registry: Optional[ObjectRegistry] = None,
    live_binding: Optional[dict] = None,
    ops_lock: Optional[asyncio.Lock] = None,
    semantic_hold: Optional[dict] = None,
    interval_s: float = 5.0,
) -> None:
    """Lifecycle linkage: track mapping's latched identity broadcast against
    the session's LIVE binding (`live_binding` — the same dict the web
    facade's Save/Load update, so a facade-initiated load never reads as
    drift) and REACT to a frame-epoch change instead of only warning:

    - generation bump on the SAME map (reset / mapping re-init underneath
      scene): every stored map-frame coordinate just went stale. Derived
      objects are flushed from the registry (re-observation rebuilds them
      in the new frame); room annotations are user assets — flagged stale
      for confirmation, never deleted. The live binding's generation is
      advanced so the response fires exactly once per epoch.
    - a DIFFERENT map_id (map loaded/switched outside scene's facade):
      the registry's objects belong to the previous frame and are flushed,
      but scene cannot restore the new map's semantic state from here —
      that stays a WARNING telling the operator to Load via the map UI
      (or restart scene). The live binding is left pointing at what scene
      last knowingly bound.

    Every reaction is best-effort: a failing marker write / flush must
    never kill the watcher — the drift detection itself is load-bearing.

    A static binding carries no generation, so the epoch reference is
    learned from the FIRST confirming broadcast: without that, `gen_ok`
    would stay vacuously true and runtime bumps would be invisible."""
    if live_binding is None:
        live_binding = {
            "map_id": binding.map_id,
            "mode": binding.mode,
            "generation": binding.generation,
            "source": binding.source,
        }
    if ops_lock is None:
        ops_lock = asyncio.Lock()
    confirmed_key: Optional[tuple] = None
    warned_ephemeral = False
    last_warned: Optional[tuple] = None

    async def _flush_registry(why: str) -> bool:
        """Flush the registry; a failure is loud and non-fatal, but the
        result is returned so callers can withhold whatever acknowledgement
        (epoch advance, warn-once latch) would otherwise stop the retry."""
        if registry is None:
            return True
        try:
            async with registry.lock():
                dropped = registry.clear_objects()
            log.warning("[scene] flushed %d mis-anchored object(s) %s",
                        dropped, why)
            return True
        except Exception as e:  # noqa: BLE001
            log.error(
                "[scene] object flush failed — mis-anchored objects remain "
                "until the flush is retried: %s", e,
            )
            return False

    async def _tick() -> None:
        nonlocal confirmed_key, warned_ephemeral, last_warned
        msg, _stamp, _count = hub.latest("map_lifecycle")
        if msg is None:
            return
        live_id, live_gen = str(msg.map_id), int(msg.generation)
        bound_id = str(live_binding.get("map_id") or "")
        ref_gen = live_binding.get("generation")
        if not live_id:
            # Ephemeral broadcast: no named identity to compare — but if
            # scene statically bound a NAMED map, that named partition is
            # being fed coordinates from a frame that resets every boot.
            # Say so once instead of guarding silently forever.
            if not warned_ephemeral and binding.source in ("config", "env"):
                log.warning(
                    "[scene] mapping broadcasts an EPHEMERAL session (empty "
                    "map_id) while scene is bound to %r (source=%s) — mapping "
                    "mode is using an unsaved live session; Save the current "
                    "mapping session as %r first, then Load that saved map or "
                    "restart in localization mode for a stable cross-boot "
                    "binding",
                    binding.map_id, binding.source, binding.map_id,
                )
                warned_ephemeral = True
            return
        if live_id == bound_id and (ref_gen is None or live_gen == ref_gen):
            if confirmed_key != (live_id, live_gen):
                log.info(
                    "[scene] map binding confirmed by mapping lifecycle: "
                    "id=%s gen=%d mode=%s", live_id, live_gen, str(msg.mode),
                )
                confirmed_key = (live_id, live_gen)
                # Learn the epoch (static bindings start without one); from
                # now on a bump IS detectable. The annotation store compares
                # the confirmed epoch against the one its file recorded.
                live_binding["generation"] = live_gen
                if anno_store is not None:
                    try:
                        anno_store.reconcile_generation(live_gen)
                    except Exception as e:  # noqa: BLE001
                        # Best-effort marker write (disk full / read-only
                        # dir); the watcher itself must survive it.
                        log.error(
                            "[scene-anno] generation reconcile failed "
                            "(annotation staleness may be outdated): %s", e,
                        )
            last_warned = None
            return
        if live_id == bound_id:
            # Same map, new frame epoch. Respond, don't just warn — under
            # the shared ops lock so the reaction can't interleave with a
            # facade Save/Load that is mid-flight.
            async with ops_lock:
                # Re-read under the lock: a facade op may have advanced the
                # binding while we waited — its update IS the new truth.
                ref_gen = live_binding.get("generation")
                if (str(live_binding.get("map_id") or "") != live_id
                        or live_gen == ref_gen):
                    return
                log.warning(
                    "[scene] map %s frame epoch changed under scene (gen "
                    "%s→%d, mode=%s) — flushing derived objects "
                    "(re-observation rebuilds them in the new frame); room "
                    "annotations are flagged stale for confirmation",
                    live_id, ref_gen, live_gen, str(msg.mode),
                )
                if not await _flush_registry("after epoch bump"):
                    # Do NOT acknowledge the new generation: with the
                    # mis-anchored objects still in the registry, advancing
                    # would make the next tick see no mismatch and never
                    # retry. Leaving state untouched re-triggers this whole
                    # reaction (flush included) on the next tick.
                    return
                if anno_store is not None:
                    try:
                        n = anno_store.mark_all_stale(
                            f"map generation {ref_gen}→{live_gen}",
                            new_generation=live_gen,
                        )
                        if n:
                            log.warning(
                                "[scene-anno] %d annotation(s) marked stale "
                                "— confirm or redraw them in the map UI", n,
                            )
                    except Exception as e:  # noqa: BLE001
                        # Losing the stale marker is recoverable (next
                        # boot's load-time epoch check re-judges); losing
                        # this watcher would silence ALL drift handling.
                        log.error(
                            "[scene-anno] stale marking failed (annotations "
                            "may show as fresh until restart): %s", e,
                        )
                # Advance the live epoch: the response fired once for this
                # bump and the next bump is measured against it.
                live_binding["generation"] = live_gen
                if str(msg.mode):
                    live_binding["mode"] = str(msg.mode)
                confirmed_key = (live_id, live_gen)
            return
        key = (live_id, live_gen)
        if key != last_warned:
            async with ops_lock:
                if str(live_binding.get("map_id") or "") == live_id:
                    # A facade Load bound this map while we waited — the
                    # mismatch we saw is already resolved, not drift.
                    return
                log.warning(
                    "[scene] mapping's live map identity (id=%s gen=%d "
                    "mode=%s) no longer matches scene's binding (id=%s "
                    "gen=%s source=%s) — flushing stale objects; Load the "
                    "map in the map UI (or restart scene) to bind its "
                    "semantic state.",
                    live_id, live_gen, str(msg.mode),
                    bound_id, ref_gen, live_binding.get("source"),
                )
                if semantic_hold is not None:
                    # Scene stays bound to the previous map while mapping
                    # runs another one: a Save would commit the flushed
                    # (or newly foreign-frame) registry as the OLD map's
                    # snapshot and purge its last valid one. Held until a
                    # facade Load completes and re-converges the binding.
                    semantic_hold["reason"] = (
                        f"mapping switched to map {live_id} outside scene"
                    )
                if not await _flush_registry("after external map switch"):
                    # Don't latch this key into `last_warned` — the next
                    # tick warns again and retries the flush.
                    return
            last_warned = key

    while True:
        await asyncio.sleep(interval_s)
        try:
            await _tick()
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001
            # The watcher is the session's only runtime epoch guard — one
            # malformed sample / failed reaction must never end it.
            log.error(
                "[scene] lifecycle watch tick failed (watch continues): %s", e,
            )


# ── main ───────────────────────────────────────────────────────────────────
async def _run() -> None:
    config = _load_config()

    # Hand the FastMCP app from mcp_tools to Capability so it owns the
    # HTTP server. Capability auto-allocates the port (was hand-set to
    # 50106) and atlas-routes consumers via QueryCapabilities. The 6
    # tools were already decorated with @mcp_contract on mcp_tools.mcp
    # at import time; we still need to declare each on atlas — Capability
    # only auto-declares tools registered via @scene.mcp(), and we kept
    # the @mcp_contract pattern in mcp_tools to avoid a cyclic-import
    # rewrite. Manual declare per tool:
    scene.use_mcp_app(mcp_tools.mcp)

    # Wire state.
    registry = ObjectRegistry(grace_period_s=5.0)
    robot_geometry = RobotGeometryState()
    self_tracker = _SelfTracker(registry, robot_geometry)

    # Object persistence. The store backs the map UI's Save/Load snapshots;
    # boot-time warm restore and builder-driven continuous writes exist only
    # under the legacy SCENE_RESTORE_ON_START mode. Created before
    # perception so a legacy restore repopulates the registry before the
    # first detections arrive; the embedder is wired in later.
    # Which SLAM map this scene session belongs to. This is the join key
    # against mapping (lifecycle watch) and, in legacy mode, the partition
    # key for scene's persistent state; the default mode partitions session
    # state under the constant `.live` label and persists object snapshots
    # under per-Save tokens instead.
    #
    # Binding precedence (choose_map_binding): mapping's latched lifecycle
    # broadcast — the authoritative map identity, probed briefly here —
    # then manifest `map_id`, then SCENE_MAP_ID env, then "default". The
    # static levers stay as fallback because the normal full-boot order
    # starts scene BEFORE mapping (probe cost then ≈ the wait window);
    # a scene restart while mapping runs binds from the broadcast alone.
    broadcast = _discover_map_binding(
        float(os.environ.get("SCENE_MAP_BINDING_WAIT_S", "3.0"))
    )
    binding = choose_map_binding(
        broadcast, config.get("map_id"), os.environ.get("SCENE_MAP_ID")
    )
    map_id = binding.map_id
    restore_on_start = os.environ.get("SCENE_RESTORE_ON_START", "false").lower() in ("true", "1", "yes")
    # A live (unsaved) session is NEVER persisted — objects only reach the
    # DB via an explicit Save snapshot (web facade) or, in the legacy
    # restore_on_start mode, the builder's continuous writes under the bound
    # map id. The constant `.live` name is just the in-memory partition label
    # for the annotation/scene-graph session state below.
    scene_state_map_id = map_id if restore_on_start else ".live"
    log.info(
        "[scene] map binding: id=%s gen=%s source=%s mode=%s restore_on_start=%s state_partition=%s",
        binding.map_id, binding.generation, binding.source, binding.mode,
        restore_on_start, scene_state_map_id,
    )
    # The session's LIVE binding view — one mutable dict shared by the web
    # facade (Save/Load update it), the /user page header (reads it), and
    # the lifecycle watcher (epoch bumps update it + flush derived state).
    # `binding` above stays the immutable startup record. `map_ops_lock`
    # serializes the facade's Save/Load/Delete with the watcher's epoch
    # response — both suspend at awaits mid-critical-section.
    map_ops_lock = asyncio.Lock()
    # Shared between the web facade (Save honors it, Load sets/clears it)
    # and the lifecycle watcher (an external map switch raises it): while
    # `reason` is non-None, scene's semantic state may not match the map
    # mapping runs, and Save is refused until a Load completes.
    semantic_hold: dict = {"reason": None}
    live_binding: dict = {
        "map_id": binding.map_id,
        "mode": binding.mode if restore_on_start else "",
        "generation": binding.generation if restore_on_start else None,
        "source": binding.source if restore_on_start else "default",
    }
    if broadcast is not None and not str(broadcast.get("map_id") or ""):
        # mapping is provably UP but running ephemeral (no map_id) — its
        # frame resets every boot, so the named partition scene just bound
        # statically will not re-anchor across boots until the operator saves
        # this live session and later loads it in localization mode.
        log.warning(
            "[scene] mapping broadcasts an EPHEMERAL session (empty map_id) "
            "while scene binds %r from %s — objects stored under this id "
            "won't re-anchor across boots; Save the current mapping session "
            "as %r first, then Load that saved map or restart in localization "
            "mode",
            binding.map_id, binding.source, binding.map_id,
        )

    obj_store = None
    if os.environ.get("SCENE_OBJECT_MEMORY_ENABLED", "true").lower() in ("true", "1", "yes"):
        from .persistence import ObjectStore

        db_path = os.environ.get(
            "SCENE_OBJECT_MEMORY_DB", "/data/robonix/scene_memory/objects.db"
        )
        try:
            obj_store = ObjectStore(db_path, map_id=scene_state_map_id)
            purged = obj_store.purge_live_partitions()
            if purged:
                log.info(
                    "[scene-persist] purged %d leftover live-session row(s) "
                    "from earlier boots", purged,
                )
            restored = obj_store.load_all() if restore_on_start else []
            if restored:
                async with registry.lock():
                    for o in restored:
                        registry.restore_object(o)
            log.info(
                "[scene-persist] object store ready: restored %d object(s) (partition=%s, restore_on_start=%s) from %s",
                len(restored), obj_store.map_id, restore_on_start, db_path,
            )
        except Exception as e:  # noqa: BLE001
            # Object memory is opted in (default on), so a failure here is not
            # benign: persistence is OFF for the whole session — no warm
            # restore now and no writes later. Log at error, not warning.
            log.error(
                "[scene-persist] object memory enabled but store init/restore "
                "failed — persistence OFF for this session (no restore, no "
                "writes): %s", e,
            )
            obj_store = None

    # User annotations (rooms / POIs) — user-authored semantics on the same
    # map_id partition rule as the object store; validity is additionally
    # tracked against mapping's generation epoch (recorded in the file —
    # objects get their epoch guarantee from snapshot partitions + the
    # watcher's flush instead). A failure here disables the annotation API
    # for the session (web answers 503) but never blocks scene itself.
    anno_dir = os.environ.get(
        "SCENE_ANNOTATIONS_DIR", "/data/robonix/scene_annotations"
    )
    if not restore_on_start:
        # Fresh live session: drop unsaved annotations left by previous
        # live sessions (constant `.live` partition) and the per-boot
        # files earlier builds leaked (`.live-<pid>-<ts>.json`). Saved
        # rooms were carried into their map's own file at Save time.
        # Best-effort in its OWN guard: a cleanup failure must not take
        # the whole annotation API down with a misattributed init error.
        try:
            for leftover in Path(anno_dir).expanduser().glob(".live*.json"):
                leftover.unlink(missing_ok=True)
        except Exception as e:  # noqa: BLE001
            log.warning(
                "[scene-anno] live-session file cleanup failed (leftovers "
                "may resurface next boot): %s", e,
            )
    anno_store: Optional[AnnotationStore] = None
    try:
        anno_store = AnnotationStore(
            anno_dir,
            map_id=scene_state_map_id,
            generation=binding.generation if restore_on_start else None,
        )
        anns = anno_store.list()
        log.info(
            "[scene-anno] annotation store ready: %d annotation(s), %d stale "
            "(map_id=%s) at %s",
            len(anns), sum(a.stale for a in anns), anno_store.map_id,
            anno_store.path,
        )
    except Exception as e:  # noqa: BLE001
        log.error(
            "[scene-anno] annotation store init failed — annotation API "
            "disabled for this session: %s", e,
        )

    # Epoch sidecar (map_id → object snapshot partition; see map_meta.py).
    # Defaults next to the annotation dir so docker (/data/robonix) and
    # native ($PKG/rbnx-build/data/robonix) deployments both get a writable
    # location without new script plumbing. Without it, Save/Load degrade
    # to annotations-only (loudly, in the web response).
    map_meta: Optional[MapMetaStore] = None
    try:
        meta_dir = os.environ.get("SCENE_MAP_META_DIR") or os.path.join(
            os.path.dirname(anno_dir.rstrip("/")) or ".", "scene_maps"
        )
        map_meta = MapMetaStore(meta_dir)
    except Exception as e:  # noqa: BLE001
        log.error(
            "[scene-mapmeta] sidecar store init failed — Save/Load will not "
            "snapshot/restore objects this session: %s", e,
        )
    # mcp_tools v0 only needs the registry + the ROS hub (the latter is
    # supplied later in _start_ros_ingest). The geometric relation loop +
    # scene-graph store are wired further down, once the registry is live.
    mcp_tools.attach_state(
        registry=registry,
        robot_geometry=robot_geometry,
    )
    mcp_tools.attach_annotation_store(anno_store)

    # Bring up atlas + lifecycle gRPC + MCP HTTP. Non-blocking; scene
    # keeps running its own asyncio event loop after this returns.
    scene.bootstrap()
    geometry_task = asyncio.create_task(
        reconcile_robot_geometry(robot_geometry),
        name="scene-robot-geometry",
    )

    # Declare each scene MCP tool on atlas. Each handler has
    # `_robonix_*` attrs stashed by @mcp_contract — re-use them so the
    # description / JSON schema stay in sync with the codegen types.
    for fn in (
        mcp_tools.list_objects,
        mcp_tools.list_regions,
        mcp_tools.get_robot_context,
        mcp_tools.goal_near,
        mcp_tools.goal_room,
        mcp_tools.get_scene_graph,
        mcp_tools.get_object_context,
        mcp_tools.list_relations,
    ):
        cid = getattr(fn, "_robonix_contract_id", None)
        if cid is None:
            log.warning(
                "scene tool %s missing _robonix_contract_id; skipping", fn.__name__
            )
            continue
        in_cls = getattr(fn, "_robonix_input_cls", None)
        schema = json.dumps(in_cls.json_schema()) if in_cls else "{}"
        scene.declare_mcp(
            cid,
            scene.mcp_endpoint,
            description=(fn.__doc__ or "").strip(),
            input_schema_json=schema,
        )
    log.info("scene declared 8 MCP tools at %s", scene.mcp_endpoint)

    # ROS2 ingest hub + downstream consumers (self-pose, perception).
    # _start_ros_ingest still wants a raw atlas stub for QueryCapabilities;
    # reach into ATLAS's wire client directly for that.
    stub = ATLAS._wire_stub
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
    mcp_tools.attach_state(
        registry=registry,
        hub=hub,
        robot_geometry=robot_geometry,
    )

    # Wire the persistence embedder to perception's loaded CLIP text encoder
    # (same 512-d space as the per-object image features). The VLM-fallback
    # detector has no `embed_text`; persistence then stores placeholder
    # vectors (scalar state still restores fine).
    if obj_store is not None:
        obj_store.set_embedder(getattr(perception, "embed_text", None))
    bg_tasks = [
        geometry_task,
        asyncio.create_task(_stale_tick(registry), name="scene-stale-tick"),
        # Object-level watchdog: polls the registry for NEW objects and
        # saves one image per object to memgraph.  Default on; set
        # SCENE_OBJECT_WATCHDOG=0 to disable.
        *([asyncio.create_task(
            ObjectWatchdog(
                registry=registry, hub=hub,
            ).run(),
            name="object-watchdog",
        )] if os.environ.get("SCENE_OBJECT_WATCHDOG", "1") in ("1", "true", "yes") else []),
        # P2 guard: warn when mapping's live map identity drifts from the
        # binding scene started with (P3 will act on it instead).
        asyncio.create_task(
            _lifecycle_watch(hub, binding, anno_store,
                             registry=registry, live_binding=live_binding,
                             ops_lock=map_ops_lock,
                             semantic_hold=semantic_hold),
            name="scene-lifecycle-watch",
        ),
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
    # These tasks are fire-and-forget: nothing awaits them, so an uncaught
    # exception would otherwise vanish until shutdown (the failure mode of
    # the failure-detectors themselves). Surface any death immediately.
    for _t in bg_tasks:
        _t.add_done_callback(_log_bg_task_exit)

    # ── Relation layer ───────────────────────────────────────────────
    # Fast geometric relations (contact/containment + reachable_by) are
    # cheap and must reach Pilot within seconds — and even without VLM
    # creds — so the store and the geometric loop run unconditionally.
    # SCENE_GRAPH_ENABLED gates only the slow LLM enrichment below.
    from .scene_graph.geometric_loop import GeometricRelationLoop
    from .scene_graph.store import SceneGraphStore

    sg_cache_dir = os.environ.get(
        "SCENE_GRAPH_CACHE_DIR", "/data/robonix/scene_graph/cache"
    )
    # Partition the scene-graph caches by the session state label (`.live`
    # by default, the bound map id in legacy mode). The web facade's Load
    # rebinds the annotation store and restores objects into the registry;
    # this cache is session state and stays on its label.
    sg_store = SceneGraphStore(cache_dir=sg_cache_dir, map_id=scene_state_map_id)
    log.info(
        "[scene-graph] cache base=%s partitioned by map_id=%s",
        sg_cache_dir, scene_state_map_id,
    )
    mcp_tools.attach_scene_graph_store(sg_store)
    geo_loop = GeometricRelationLoop(registry, sg_store)
    await geo_loop.start()

    # ── Scene Graph (optional LLM enrichment of the residual) ────────
    sg_stop: asyncio.Event | None = None
    if os.environ.get("SCENE_GRAPH_ENABLED", "true").lower() in ("true", "1", "yes"):
        from .scene_graph.builder import SceneGraphBuilder, SceneGraphConfig, scene_graph_loop
        from .scene_graph.captioner import NodeCaptioner
        from .scene_graph.llm_client import SceneGraphLLMClient
        from .scene_graph.relations import RelationInferer

        sg_cfg = SceneGraphConfig()
        sg_llm = SceneGraphLLMClient()
        sg_captioner = NodeCaptioner()
        sg_inferer = RelationInferer(sg_llm)
        sg_builder = SceneGraphBuilder(
            registry=registry,
            captioner=sg_captioner,
            relation_inferer=sg_inferer,
            store=sg_store,
            config=sg_cfg,
            # Live sessions are not persisted (objects reach the DB only via
            # an explicit Save snapshot); the builder's continuous writes are
            # the LEGACY warm-restore mode's mechanism and follow its switch.
            object_store=obj_store if restore_on_start else None,
            perception=perception,
        )
        sg_stop = asyncio.Event()
        bg_tasks.append(
            asyncio.create_task(
                scene_graph_loop(sg_builder, sg_stop),
                name="scene-graph-loop",
            )
        )
        log.info(
            "scene graph LLM enrichment enabled (interval=%.0fs, cache=%s)",
            sg_cfg.interval_sec,
            sg_cache_dir,
        )

    # Web debug UI on a separate port — top-down 2D canvas + objects
    # table + robot pose. Lives in the same asyncio loop as the rest
    # of scene so registry reads are local. Set `web_port: 0` in the
    # deploy-manifest scene block to disable. SCENE_WEB_PORT and
    # SCENE_WEB_HOST are environment fallbacks; an explicit Scene config file
    # can set web_host: 127.0.0.1 to keep this operator surface local-only.
    web_port = int(
        int(config.get("web_port") or "0")
        if config.get("web_port") is not None and config.get("web_port") != ""
        else int(os.environ.get("SCENE_WEB_PORT", "50107") or "")
    )
    web_task = None
    web_server: uvicorn.Server | None = None
    if web_port > 0:
        web_host = resolve_web_host(config)
        web_app = web_ui.make_app(
            registry=registry,
            hub=hub,
            detector=perception,
            sg_store=sg_store,
            anno_store=anno_store,
            object_store=obj_store,
            map_meta=map_meta,
            map_binding=live_binding,
            ops_lock=map_ops_lock,
            semantic_hold=semantic_hold,
            robot_geometry=robot_geometry,
        )
        web_uv = uvicorn.Config(
            app=web_app,
            host=web_host,
            port=web_port,
            log_level="warning",
        )
        web_server = uvicorn.Server(web_uv)
        web_task = asyncio.create_task(web_server.serve(), name="scene-web-http")
        log.info("web UI on http://%s:%d", web_host, web_port)

    log.info(
        "scene up; cap=%s mcp=%s observations=%d",
        scene.id,
        scene.mcp_endpoint,
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
    if sg_stop is not None:
        sg_stop.set()
    if perception is not None:
        with contextlib.suppress(Exception):
            await perception.stop()
    with contextlib.suppress(Exception):
        await hub.stop()
    await geo_loop.stop()
    for t in bg_tasks:
        t.cancel()
    if web_server is not None:
        web_server.should_exit = True

    # Capability owns the gRPC server, MCP HTTP, heartbeat — stop them.
    scene._teardown()

    # Release the milvus-lite lock so the next boot can re-open the .db.
    if obj_store is not None:
        with contextlib.suppress(Exception):
            obj_store.close()


def main() -> None:
    try:
        asyncio.run(_run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
