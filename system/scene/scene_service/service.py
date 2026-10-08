# SPDX-License-Identifier: MulanPSL-2.0
"""scene_service entrypoint: registry, ROS 2 ingest, perception, scene graph,
MCP tools and web UI. Atlas registration, the driver lifecycle and the MCP
HTTP server belong to `robonix_api.Service`."""

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

import uvicorn

# torch/open3d can segfault on driver mismatches; get a stack when they do.
faulthandler.enable(all_threads=True)


from robonix_api import ATLAS, Service  # noqa: E402
from robonix_api.atlas_types import Ros2Params, Transport  # noqa: E402

scene = Service(id="scene", namespace="robonix/system/scene")

from . import mcp_tools
from . import web as web_ui
from .annotations import AnnotationStore
from .ingest.capabilities import PERCEPTION_KEYS, perception_config, plan_perception, provider_for_kind
from .map_binding import MapBinding, choose_map_binding, read_latched_lifecycle
from .object_watchdog import ObjectWatchdog
from .map_meta import MapMetaStore
from .robot_geometry import RobotGeometryState, reconcile_robot_geometry
from .ingest.perception_concept_graphs import ConceptGraphsDetector
from .ingest.perception_dualmap import DualMapDetector
from .ingest.perception_vlm import VLMObjectDetector, _CamIntrinsics
from .ingest.ros_subscribers import (
    SubscribersHub,
    TopicSpec,
)
from .lifecycle_runtime import (
    SceneLifecycleRuntime,
    close_scene_runtime_resources,
)
from .state import (
    BBox3D,
    ObjectRegistry,
    Pose3D,
)
from .state.object_registry import now_unix
from .object_views import (
    bearing_of as _view_bearing,
    looks_like as _view_looks_like,
    store_from_env,
)
from .web_binding import resolve_web_host

_LOG_LEVEL = os.environ.get("SCENE_LOG_LEVEL", "INFO").upper()
# `force`: an imported library may already have configured the root logger.
logging.basicConfig(
    level=_LOG_LEVEL,
    format="[scene-service] %(levelname)s %(message)s",
    force=True,
)
log = logging.getLogger("scene-service")
# Scribe's bridge later resets the root logger's level and handlers, so this
# logger keeps its own level and, in a container (whose console rbnx pipes to
# scribe), its own console handler.
log.setLevel(_LOG_LEVEL)
if Path("/.dockerenv").exists():
    _console = logging.StreamHandler()
    _console.setFormatter(
        logging.Formatter("[scene-service] %(levelname)s %(message)s"))
    log.addHandler(_console)


_lifecycle = SceneLifecycleRuntime(log)


@scene.on_init
def _on_init(cfg: dict):
    """Receive the deployment's nested system.scene.config mapping."""
    return _lifecycle.on_init(cfg)


@scene.on_activate
def _on_activate():
    """Wait until Scene's async runtime is fully initialized."""
    return _lifecycle.on_activate()


@scene.on_shutdown
def _on_shutdown():
    """Wait until Scene's async runtime has released its resources."""
    return _lifecycle.on_shutdown()


@scene.on_deactivate
def _on_deactivate():
    """Keep Scene ACTIVE until a real pause/resume lifecycle is implemented."""
    return _lifecycle.on_deactivate()


async def _wait_for_lifecycle_event(event) -> bool:
    """Wait without blocking asyncio; return false if shutdown wins."""
    while not event.is_set():
        if _lifecycle.shutdown_requested.is_set():
            return False
        await asyncio.sleep(0.05)
    return True


# The contracts Scene consumes, as (kind, contract_id, msg_type); Atlas says
# which providers serve them. A new input is a row here plus its consumer.
_SCENE_CONTRACTS: list[tuple[str, str, str]] = [
    ("rgb", "robonix/primitive/camera/rgb", "Image"),
    ("depth", "robonix/primitive/camera/depth", "Image"),
    ("lidar2d", "robonix/primitive/lidar/lidar", "LaserScan"),
    ("lidar3d", "robonix/primitive/lidar/lidar3d", "PointCloud2"),
    ("camera_extrinsics", "robonix/primitive/camera/extrinsics", "TransformStamped"),
    ("intrinsics", "robonix/primitive/camera/intrinsics", "CameraInfo"),
    ("pose", "robonix/service/map/pose", "PoseWithCovarianceStamped"),
    ("odom", "robonix/primitive/chassis/odom", "Odometry"),
    ("occupancy_grid", "robonix/service/map/occupancy_grid", "OccupancyGrid"),
    # Mapping's latched {map_id, mode, generation}, for _lifecycle_watch; the
    # hub skips it when the generated `map` package is missing.
    ("map_lifecycle", "robonix/service/map/lifecycle", "MapLifecycle"),
]

# Kinds dropped even when Atlas advertises them.
_DEFAULT_DISABLED_KINDS: frozenset[str] = frozenset()


# Only the ROS 2 ingest path is wired; "grpc" is reserved.
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


# (transport, contract_id) -> last resolution, so rediscovery logs changes only.
_LAST_RESOLVED: dict[tuple, tuple] = {}


def _build_topic_specs(
    observations: list[dict],
    atlas_stub,
    transport: str,
    camera_provider_id: str = "",
    known_kinds: set[str] | None = None,
) -> list[TopicSpec]:
    """Topic specs from the manifest's `observations[]` when given, else from
    Atlas for every row of `_SCENE_CONTRACTS` not in `known_kinds`. A failing
    entry is skipped, never fatal."""
    pb_t = _resolve_pb_transport(transport)
    if observations:
        return _resolve_explicit(observations, atlas_stub, pb_t, camera_provider_id)
    return _resolve_auto(
        atlas_stub,
        pb_t,
        camera_provider_id,
        known_kinds=known_kinds,
    )


def _resolve_auto(
    _unused,
    pb_transport: int,
    camera_provider_id: str = "",
    *,
    known_kinds: set[str] | None = None,
) -> list[TopicSpec]:
    """Resolve each `_SCENE_CONTRACTS` row through Atlas, skipping kinds the
    hub already owns so a subscription keeps one channel."""
    transport = Transport(pb_transport)
    known = known_kinds or set()
    out: list[TopicSpec] = []
    for kind, contract_id, msg_type in _SCENE_CONTRACTS:
        if kind in _DEFAULT_DISABLED_KINDS or kind in known:
            continue
        spec = _resolve_one_contract(
            transport,
            kind,
            contract_id,
            msg_type,
            provider_id=provider_for_kind(kind, camera_provider_id),
        )
        if spec is not None:
            out.append(spec)
    return out


def _resolve_one_contract(
    transport: Transport,
    kind: str,
    contract_id: str,
    msg_type: str,
    *,
    provider_id: str = "",
) -> Optional[TopicSpec]:
    """The endpoint serving `contract_id`, or None when nobody provides it."""
    caps = ATLAS.find_capability(
        contract_id=contract_id,
        transport=transport,
        provider_id=provider_id,
    )
    if not caps:
        return None
    cap_view = caps[0]
    try:
        ch = scene.connect_capability(cap_view, contract_id, transport)
    except Exception as e:  # noqa: BLE001
        log.warning(
            "[scene] connect_capability(%s/%s) failed: %s",
            cap_view.provider_id,
            contract_id,
            e,
        )
        return None
    endpoint = (ch.endpoint or "").strip()
    if not endpoint:
        ch.close()
        return None
    qos_profile = ""
    if isinstance(ch.params, Ros2Params):
        qos_profile = ch.params.qos_profile or ""
    sig = (endpoint, msg_type, qos_profile or "default", cap_view.provider_id)
    prev = _LAST_RESOLVED.get((transport, contract_id))
    if prev != sig:
        log.info(
            "[scene] %r ← atlas: topic=%s msg=%s qos=%s contract=%s cap=%s",
            kind,
            endpoint,
            msg_type,
            qos_profile or "default",
            contract_id,
            cap_view.provider_id,
        )
        _LAST_RESOLVED[(transport, contract_id)] = sig
    return TopicSpec(
        kind=kind,
        topic=endpoint,
        msg_type=msg_type,
        qos_profile=qos_profile or "default",
    )


def _discover_map_binding(wait_s: float) -> Optional[dict]:
    """Mapping's latched {map_id, mode, generation}, or None.

    Waits up to `wait_s` for the contract (mapping usually boots after Scene)
    and 5 s for the sample; `wait_s <= 0` disables the probe."""
    if wait_s <= 0:
        return None
    deadline = time.monotonic() + wait_s
    while True:
        try:
            spec = _resolve_one_contract(
                Transport.ROS2,
                "map_lifecycle",
                "robonix/service/map/lifecycle",
                "MapLifecycle",
            )
        except Exception as e:  # noqa: BLE001
            log.warning(
                "[scene] lifecycle probe: atlas query failed (%s) — "
                "falling back to static map binding",
                e,
            )
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
    observations: list[dict],
    atlas_stub,
    pb_transport: int,
    camera_provider_id: str = "",
) -> list[TopicSpec]:
    """Resolve the manifest's `{kind, contract[, msg_type, provider_id]}`
    entries; msg_type defaults to the `_SCENE_CONTRACTS` row."""
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
        msg_type = (
            str(entry.get("msg_type", "")) or by_contract.get(contract, (kind, ""))[1]
        )
        if not msg_type:
            log.warning(
                "[scene] observation %r: no msg_type — add it to the entry "
                "or to _SCENE_CONTRACTS in service.py",
                entry,
            )
            continue
        provider_id = str(entry.get("provider_id") or "")
        if not provider_id:
            provider_id = provider_for_kind(kind, camera_provider_id)
        spec = _resolve_one_contract(
            Transport(pb_transport),
            kind,
            contract,
            msg_type,
            provider_id=provider_id,
        )
        if spec is not None:
            out.append(spec)
    return out


# ── Self-pose tracker ──────────────────────────────────────────────────────
class _SelfTracker:
    """Owns the `robot` object, created from the first map-frame pose and
    updated from then on; `latest_xy_yaw` feeds the VLM detector."""

    def __init__(
        self,
        registry: ObjectRegistry,
        robot_geometry: RobotGeometryState,
    ) -> None:
        self.registry = registry
        self.robot_geometry = robot_geometry
        self._latest: Optional[tuple[float, float, float, float]] = None
        self._object_id: Optional[str] = None
        # The localizer's own frame name, never assumed to be "map".
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
                    label="robot",
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


# ── Stale-tick: flip missing flag, and collapse duplicates ─────────────────

# Merging is a quadratic repair, not a normal step, so it runs rarely.
_MERGE_EVERY_N_TICKS = 10


def _merge_gate_from_env() -> tuple[float, float]:
    """Floor and height gates for the duplicate collapse, in metres; a zero
    floor gate turns the collapse off."""
    def _read(name: str, default: float) -> float:
        try:
            return float(os.environ.get(name, "").strip() or default)
        except ValueError:
            return default

    return (_read("SCENE_MERGE_XY_M", 0.35), _read("SCENE_MERGE_Z_M", 1.20))


def declarable_scene_tools() -> list:
    """Every `@mcp_contract` handler in mcp_tools, once each, by contract id.
    Walked rather than listed, so a new tool cannot be left undeclared."""
    seen: dict[str, Any] = {}
    for name in dir(mcp_tools):
        if name.startswith("_"):
            continue
        fn = getattr(mcp_tools, name, None)
        cid = getattr(fn, "_robonix_contract_id", None)
        if not cid or not callable(fn):
            continue
        seen.setdefault(str(cid), fn)
    return [seen[cid] for cid in sorted(seen)]


def _declare_tools() -> None:
    """Declare the MCP tools on Atlas with the metadata @mcp_contract stashed."""
    tools = declarable_scene_tools()
    for fn in tools:
        in_cls = getattr(fn, "_robonix_input_cls", None)
        scene.declare_mcp(
            fn._robonix_contract_id,
            scene.mcp_endpoint,
            description=(fn.__doc__ or "").strip(),
            input_schema_json=json.dumps(in_cls.json_schema()) if in_cls else "{}",
        )
    log.info("scene declared %d MCP tools at %s", len(tools), scene.mcp_endpoint)


async def _stale_tick(registry: ObjectRegistry, *, period_s: float = 1.0) -> None:
    merge_xy, merge_z = _merge_gate_from_env()
    tick = 0
    while True:
        tick += 1
        async with registry.lock():
            flipped = registry.mark_stale(now_unix())
            merged = []
            if merge_xy > 0.0 and tick % _MERGE_EVERY_N_TICKS == 0:
                merged = registry.merge_duplicates(
                    now_unix(), xy_m=merge_xy, z_m=merge_z)
        if flipped:
            log.debug("marked %d object(s) missing (grace expired)", flipped)
        for absorbed, survivor in merged:
            # Info: an id a caller may hold just stopped naming this object.
            log.info("merged duplicate %s into %s", absorbed, survivor)
        await asyncio.sleep(period_s)


async def _object_views_tick(
    store, detector, registry, map_binding, *,
    period_s: float = 4.0,
) -> None:
    """Offer the camera's current view of each visible object to the store.

    Slow on purpose: a new side of an object only appears once the robot
    moves. The frame is read without the perception lock; a one-tick skew
    moves a crop by centimetres.
    """
    from .scene_graph.image_relations import project_box, project_point

    while True:
        await asyncio.sleep(period_s)
        try:
            # The detector's own boxes when it keeps them: a crop from the box
            # the label was given to cannot show something else. Projecting
            # the fused map box is the fallback, and misses whenever that box
            # sits off the object.
            seen = getattr(detector, "latest_detection_views", lambda: None)()
            if seen is not None:
                rgb, boxes, cam_xy = seen
                height, width = int(rgb.shape[0]), int(rgb.shape[1])
                objects = await registry.snapshot()
                map_id = store.partition(map_binding)
                for oid, rect in boxes:
                    obj = objects.get(oid)
                    if obj is None or obj.attributes.get("is_robot"):
                        continue
                    await asyncio.to_thread(
                        store.offer,
                        map_id=map_id, object_id=oid, image_bgr=rgb, rect=rect,
                        bearing=_view_bearing(cam_xy, (float(obj.pose.x), float(obj.pose.y))),
                        img_w=width, img_h=height,
                    )
                continue
            bundle = detector.latest_frame_bundle()
            if bundle is None:
                continue
            rgb, K, T_cam_map = bundle
            height, width = int(rgb.shape[0]), int(rgb.shape[1])
            # Depth rejects crops of whatever occludes the object.
            depth_m = None
            try:
                depth_m = detector.latest_depth_metres()
            except Exception:  # noqa: BLE001
                depth_m = None
            # Camera-optical -> map pose: its translation is the camera position.
            cam_xy = (float(T_cam_map[0, 3]), float(T_cam_map[1, 3]))

            objects = await registry.snapshot()
            map_id = store.partition(map_binding)
            for obj in objects.values():
                if obj.missing or obj.attributes.get("is_robot"):
                    continue
                rect = project_box(
                    T_cam_map, K,
                    (obj.pose.x, obj.pose.y, obj.pose.z),
                    (obj.bbox.size_x, obj.bbox.size_y, obj.bbox.size_z),
                    obj.bbox.yaw, width, height,
                )
                if rect is None:
                    continue
                centre = project_point(
                    T_cam_map, K, (obj.pose.x, obj.pose.y, obj.pose.z))
                if centre is None or not _view_looks_like(depth_m, rect, centre[2]):
                    continue
                bearing = _view_bearing(
                    cam_xy, (float(obj.pose.x), float(obj.pose.y)))
                await asyncio.to_thread(
                    store.offer,
                    map_id=map_id, object_id=obj.object_id, image_bgr=rgb,
                    rect=rect, bearing=bearing, img_w=width, img_h=height,
                )
        except asyncio.CancelledError:
            raise
        except Exception:  # noqa: BLE001
            log.exception("[scene-views] capture failed")


async def _export_clouds(detector) -> tuple[dict, dict]:
    """Point clouds and camera colours per registry id, from perception."""
    clouds: dict[str, list] = {}
    colours: dict[str, list] = {}
    if detector is None or not hasattr(detector, "export_3d_snapshot"):
        return clouds, colours
    try:
        snapshot = await asyncio.to_thread(detector.export_3d_snapshot)
    except Exception:  # noqa: BLE001
        # Warning: an export that keeps failing leaves the map point-less.
        log.warning("[scene-rerun] the point-cloud export failed; objects "
                    "will have no points this tick", exc_info=True)
        return clouds, colours
    for entry in snapshot.get("objects") or []:
        # Registry id when the backend has one, else its own uuid.
        key = entry.get("object_id") or entry.get("id")
        if not key:
            continue
        clouds[key] = entry.get("points") or []
        if entry.get("point_colors"):
            colours[key] = entry["point_colors"]
    return clouds, colours


async def _rerun_tick(sink, registry, detector, hub, sg_store=None,
                      robot_geometry=None, *, period_s: float = 1.0) -> None:
    """Publish the settled semantic map to the viewer, once a page opened it."""
    from . import web as web_ui
    from .rerun_sink import relation_edges

    ticks = 0
    started = time.time()
    while True:
        try:
            if not sink.ready:  # the export is the cost; nobody is watching
                await asyncio.sleep(period_s)
                continue
            objects = await registry.snapshot()
            live = [o for o in objects.values() if o.settled]
            clouds, colours = await _export_clouds(detector)
            robot = next((o for o in objects.values()
                          if o.attributes.get("is_robot")), None)
            pose = (robot.pose.x, robot.pose.y, robot.pose.yaw) if robot else None

            def publish():
                # One thread: rerun's timeline is per thread, and PNG work
                # and per-point loops stay off the event loop.
                sink.set_time(time.time() - started)
                occupancy = web_ui.occupancy_payload(hub)
                sink.log_occupancy(occupancy)
                sink.log_objects(live, clouds, colours)
                edges = relation_edges(sg_store)
                sink.log_relations(
                    edges, {o.object_id: (o.pose.x, o.pose.y, o.pose.z) for o in live})
                # Soma's polygon; the registry only holds the robot's box.
                shape = robot_geometry.current() if robot_geometry else None
                footprint = [list(p) for p in shape.points] if shape else []
                if pose is not None:
                    sink.log_robot(pose, footprint)
                sink.log_map2d(occupancy, live, clouds, edges, pose, footprint)
                return edges

            edges = await asyncio.to_thread(publish)
            if ticks % 60 == 0:  # an unfed viewer looks like an empty map
                log.info(
                    "[scene-rerun] tick %d: %d objects, %d with points, "
                    "%d with colour, robot=%s, %d edges", ticks, len(live),
                    sum(1 for o in live if clouds.get(o.object_id)),
                    sum(1 for o in live if colours.get(o.object_id)),
                    pose is not None, len(edges))
            ticks += 1
        except Exception:  # noqa: BLE001
            log.exception("[scene-rerun] publish failed")
        await asyncio.sleep(period_s)


async def _auto_discover_loop(
    *,
    atlas_stub,
    hub,
    transport: str,
    explicit: list[dict],
    camera_provider_id: str = "",
    period_s: float = 5.0,
) -> None:
    """Subscribe to inputs that appear after start (mapping boots late).
    Explicit observations are static and skip this."""
    if explicit:
        return
    while True:
        try:
            await asyncio.sleep(period_s)
            current = hub.has_kinds()
            specs = _build_topic_specs(
                explicit,
                atlas_stub,
                transport,
                camera_provider_id,
                known_kinds=current,
            )
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
    """Start the rclpy hub and its consumers (self pose, perception), each
    its own task. Returns (hub, detector or None, background tasks).

    Waits until Atlas offers at least one input, then a reconciler adds the
    rest as they appear; explicit `observations` skip both.
    """
    explicit = config.get("observations") or []
    transport = str(config.get("transport") or "ros2")
    camera_provider_id = str(config.get("camera_provider_id") or "").strip()
    if camera_provider_id:
        log.info(
            "[scene] RGB-D camera pinned to provider %s",
            camera_provider_id,
        )
    specs = _build_topic_specs(explicit, atlas_stub, transport, camera_provider_id)
    if not specs and not explicit:
        attempt = 0
        while not specs:
            attempt += 1
            await asyncio.sleep(2.0)
            specs = _build_topic_specs(
                explicit, atlas_stub, transport, camera_provider_id
            )
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

    # Always started: pose/odom may appear after Scene does.
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

    # The camera often registers after the chassis; give it a bounded wait so
    # the RGB-D path is not lost to a startup race.
    perception_wait_s = float(os.environ.get("SCENE_PERCEPTION_WAIT_S", "30"))
    deadline = time.time() + perception_wait_s
    while time.time() < deadline and not (hub.has("rgb") and hub.has("depth")):
        new_specs = _build_topic_specs(
            explicit, atlas_stub, transport, camera_provider_id
        )
        for spec in new_specs:
            if spec.kind not in hub.has_kinds():
                hub.add_spec(spec)
        if hub.has("rgb") and hub.has("depth"):
            log.info("[scene] perception-wait: rgb+depth now available")
            break
        await asyncio.sleep(2.0)

    if camera_provider_id:
        missing = [kind for kind in ("rgb", "depth") if not hub.has(kind)]
        if missing:
            log.warning(
                "[scene] camera provider %s is missing required RGB-D "
                "contract(s): %s",
                camera_provider_id,
                ", ".join(missing),
            )

    # ── perception ─────────────────────────────────────────────────────────
    # The capability probe picks the tier: metric (RGB-D), visual (VLM) or
    # none; each degradation is logged.
    perception_cfg = perception_config(config)
    if perception_cfg.ignored_keys:
        log.warning(
            "[scene] perception config keys ignored (not implemented): %s; "
            "recognised keys are %s",
            ", ".join(perception_cfg.ignored_keys), ", ".join(sorted(PERCEPTION_KEYS)),
        )
    profile = perception_cfg.profile
    plan = plan_perception(hub, profile, perception_cfg.backend)
    log.info("[scene] perception plan: %s", plan.summary())
    intrinsics_fallback = _scene_intrinsics_fallback(config.get("intrinsics_fallback"))
    detector: Optional[Any] = None
    camera_frame = str(
        config.get("camera_frame")
        or os.environ.get("SCENE_CAMERA_FRAME")
        or ""
    ).strip()
    configured_base_frame = str(
        config.get("base_frame")
        or os.environ.get("SCENE_BASE_FRAME")
        or ""
    ).strip()
    def _latest(kind: str) -> Optional[Any]:
        msg, stamp, _ = hub.latest(kind)
        return None if msg is None or stamp == 0.0 else msg

    def _rgb_msg() -> Optional[Any]:
        return _latest("rgb")

    def _depth_msg() -> Optional[Any]:
        return _latest("depth")

    def _frame_of(msg: Any) -> str:
        return str(getattr(getattr(msg, "header", None), "frame_id", "") or "").strip()

    def _active_camera_frame() -> str:
        return camera_frame or _frame_of(_rgb_msg())

    def _active_base_frame() -> str:
        footprint = self_tracker.robot_geometry.current()
        return footprint.base_frame if footprint is not None else ""

    if plan.detector in ("concept_graphs", "dualmap"):

        # The intrinsics contract first; the configured `intrinsics_fallback`
        # only until a usable CameraInfo arrives.
        intrinsics_logged = {"ok": False, "bad": False, "fallback": False}

        def _cam_info() -> Optional[_CamIntrinsics]:
            if hub.has("intrinsics"):
                msg, stamp, _ = hub.latest("intrinsics")
                if msg is not None and stamp > 0.0:
                    info_frame = _frame_of(msg)
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
                                k.fx,
                                k.fy,
                                k.cx,
                                k.cy,
                                k.width,
                                k.height,
                            )
                            intrinsics_logged["ok"] = True
                        return k
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
                        source,
                        k.fx,
                        k.fy,
                        k.cx,
                        k.cy,
                        k.width,
                        k.height,
                    )
                    intrinsics_logged["fallback"] = True
                return k
            return None

        # Two interchangeable metric mappers, chosen by `perception.backend`.
        backend = perception_cfg.backend
        detector_cls = DualMapDetector if backend == "dualmap" else ConceptGraphsDetector
        backend_kwargs = {"dualmap_cfg": perception_cfg.dualmap or None} if backend == "dualmap" else {}
        detector = detector_cls(
            rgb_fetcher_msg=_rgb_msg,
            depth_fetcher_msg=_depth_msg,
            camera_info_fetcher=_cam_info,
            world_frame_fn=lambda: self_tracker.world_frame_id,
            on_detections=lambda dets: _ingest_detections(registry, dets),
            registry=registry,
            # camera->world from TF, with the pose + extrinsics contracts as
            # the fallback.
            robot_base_frame_fn=_active_base_frame,
            hub=hub,
            # Raise on a shared GPU (e.g. with speech); manifest > env > default.
            period_s=float(
                perception_cfg.period_s
                or os.environ.get("SCENE_DETECT_PERIOD_S", "")
                or 0.6
            ),
            confidence_threshold=float(
                perception_cfg.confidence_threshold
                or os.environ.get("SCENE_DETECT_CONFIDENCE", "")
                or 0.30
            ),
            max_detections=int(perception_cfg.max_detections or 30),
            cfg_overrides=perception_cfg.concept_graphs or None,
            profile=profile,
            pose_max_age_s=pose_max_age_s,
            camera_frame=camera_frame,
            base_frame=configured_base_frame or None,
            **backend_kwargs,
        )
        await detector.start()
        if getattr(detector, "_task", None) is None:
            log.error(
                "[scene] perception backend %s did not start (see warnings above); "
                "Scene is running WITHOUT object recognition", backend,
            )
        else:
            log.info("[scene] perception: %s (rgb+depth, backend=%s, profile=%s)",
                     detector_cls.__name__, backend, profile)
    elif plan.detector == "vlm":
        log.warning(
            "[scene] perception: no depth stream — falling back to "
            "VLMObjectDetector. Object positions will be approximate. "
            "Configure a depth topic to get metric-accurate poses."
        )

        def _rgb_jpeg() -> Optional[tuple[bytes, int]]:
            """Return the latest encoded frame with its delivery count."""
            msg, _, count = hub.latest("rgb")
            if msg is None or count == 0:
                return None
            jpeg = _image_msg_to_jpeg(msg)
            return (jpeg, count) if jpeg is not None else None

        # Resolved per call (CameraInfo often arrives late); never invents K.
        def _vlm_intrinsics() -> Optional[_CamIntrinsics]:
            if hub.has("intrinsics"):
                msg, stamp, _ = hub.latest("intrinsics")
                if msg is not None and stamp > 0.0:
                    info_frame = _frame_of(msg)
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
    elif profile == "annotate":
        log.info(
            "[scene] perception: profile=annotate — object recognition off by "
            "configuration; manual regions/annotations, occupancy_grid and "
            "goal_near remain available"
        )
    else:
        log.warning(
            "[scene] perception: geometric tier — no camera wired; object "
            "detection disabled (occupancy_grid + goal_near remain available)"
        )

    return hub, detector, bg_tasks


def _scene_intrinsics_fallback(raw: Any) -> Optional[tuple[str, _CamIntrinsics]]:
    """The configured intrinsics fallback, or None. Opt-in and complete only:
    a guessed K silently moves every object."""
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
    """sensor_msgs/CameraInfo to _CamIntrinsics, or None when K or the size is
    not positive (the caller then waits; it never uses a default)."""
    try:
        # rclpy hands K over as an ndarray: `k or K` would raise here.
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
    """Feed the tracker the robot's pose from the map pose contract, else
    odometry, in the frame the message names; frameless samples are dropped."""

    def read(msg) -> tuple:
        p = (msg.pose.pose if hasattr(msg, "pose") and hasattr(msg.pose, "pose")
             else msg.pose)
        q = p.orientation
        return (float(p.position.x), float(p.position.y), float(p.position.z),
                _quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w)),
                getattr(getattr(msg, "header", None), "frame_id", None) or None)

    missing_frame_warned = False
    stale_warned = False
    while True:
        x = y = z = yaw = None
        frame_id: Optional[str] = None

        if hub.has("pose"):  # SLAM-corrected, preferred
            msg, stamp_unix, _count = hub.latest("pose")
            if (
                msg is not None
                and stamp_unix > 0
                and time.time() - stamp_unix <= pose_max_age_s
            ):
                x, y, z, yaw, frame_id = read(msg)

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
                x, y, z, yaw, frame_id = read(msg)

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
    """sensor_msgs/Image to JPEG bytes; None for an unknown encoding."""
    try:
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
    """Associate detections with registry objects."""
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
    """Log a background task's exception when it dies, not at shutdown."""
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
    """React to map-frame epoch changes and external map switches."""
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
        if registry is None:
            return True
        try:
            async with registry.lock():
                dropped = registry.clear_objects()
            log.warning(
                "[scene] flushed %d mis-anchored object(s) %s", dropped, why
            )
            return True
        except Exception as e:  # noqa: BLE001
            log.error(
                "[scene] object flush failed — mis-anchored objects remain "
                "until the flush is retried: %s",
                e,
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
            if not warned_ephemeral and binding.source in ("config", "env"):
                log.warning(
                    "[scene] mapping broadcasts an EPHEMERAL session (empty "
                    "map_id) while scene is bound to %r (source=%s) — mapping "
                    "mode is using an unsaved live session; Save the current "
                    "mapping session as %r first, then Load that saved map or "
                    "restart in localization mode for a stable cross-boot "
                    "binding",
                    binding.map_id,
                    binding.source,
                    binding.map_id,
                )
                warned_ephemeral = True
            return
        if live_id == bound_id and (ref_gen is None or live_gen == ref_gen):
            if confirmed_key != (live_id, live_gen):
                log.info(
                    "[scene] map binding confirmed by mapping lifecycle: "
                    "id=%s gen=%d mode=%s",
                    live_id,
                    live_gen,
                    str(msg.mode),
                )
                confirmed_key = (live_id, live_gen)
                live_binding["generation"] = live_gen
                if anno_store is not None:
                    try:
                        anno_store.reconcile_generation(live_gen)
                    except Exception as e:  # noqa: BLE001
                        log.error(
                            "[scene-anno] generation reconcile failed "
                            "(annotation staleness may be outdated): %s",
                            e,
                        )
            last_warned = None
            return
        if live_id == bound_id:
            async with ops_lock:
                ref_gen = live_binding.get("generation")
                if (
                    str(live_binding.get("map_id") or "") != live_id
                    or live_gen == ref_gen
                ):
                    return
                log.warning(
                    "[scene] map %s frame epoch changed under scene (gen "
                    "%s→%d, mode=%s) — flushing derived objects "
                    "(re-observation rebuilds them in the new frame); room "
                    "annotations are flagged stale for confirmation",
                    live_id,
                    ref_gen,
                    live_gen,
                    str(msg.mode),
                )
                if not await _flush_registry("after epoch bump"):
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
                                "— confirm or redraw them in the map UI",
                                n,
                            )
                    except Exception as e:  # noqa: BLE001
                        log.error(
                            "[scene-anno] stale marking failed (annotations "
                            "may show as fresh until restart): %s",
                            e,
                        )
                live_binding["generation"] = live_gen
                if str(msg.mode):
                    live_binding["mode"] = str(msg.mode)
                confirmed_key = (live_id, live_gen)
            return
        key = (live_id, live_gen)
        if key != last_warned:
            async with ops_lock:
                if str(live_binding.get("map_id") or "") == live_id:
                    return
                log.warning(
                    "[scene] mapping's live map identity (id=%s gen=%d "
                    "mode=%s) no longer matches scene's binding (id=%s "
                    "gen=%s source=%s) — flushing stale objects; Load the "
                    "map in the map UI (or restart scene) to bind its "
                    "semantic state.",
                    live_id,
                    live_gen,
                    str(msg.mode),
                    bound_id,
                    ref_gen,
                    live_binding.get("source"),
                )
                if semantic_hold is not None:
                    semantic_hold["reason"] = (
                        f"mapping switched to map {live_id} outside scene"
                    )
                if not await _flush_registry("after external map switch"):
                    return
            last_warned = key

    while True:
        await asyncio.sleep(interval_s)
        try:
            await _tick()
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001
            log.error(
                "[scene] lifecycle watch tick failed (watch continues): %s", e
            )


# ── active runtime ─────────────────────────────────────────────────────────
def _env_flag(name: str, default: str) -> bool:
    return os.environ.get(name, default).lower() in ("true", "1", "yes")


def _web_port_from(config: dict) -> int:
    """The web UI's port, 0 when it is switched off."""
    raw = config.get("web_port")
    if raw is not None and raw != "":
        return int(int(raw) or 0)
    return int(os.environ.get("SCENE_WEB_PORT", "50107") or "")


async def _run_active(config: dict) -> None:
    """Start Scene resources after Driver(INIT) and Driver(ACTIVATE)."""
    # ── The map viewer ────────────────────────────────────────────────────
    rerun_sink = None
    viewer_choice = "off"
    web_host = ""
    web_port_early = _web_port_from(config)
    if web_port_early > 0:
        web_host = resolve_web_host(config)
        viewer_choice = str(
            config.get("web_viewer")
            or os.environ.get("SCENE_WEB_VIEWER", "rerun")
        ).strip().lower()
        if viewer_choice not in ("rerun", "off"):
            raise ValueError(
                f"scene web_viewer must be rerun or off, not {viewer_choice!r}")
        if viewer_choice != "off":
            from .rerun_sink import RerunSink

            # Started by the first page that opens it, not here.
            viewer_grpc_port = int(
                os.environ.get("SCENE_RERUN_GRPC_PORT", "9876"))
            rerun_sink = RerunSink(
                grpc_port=viewer_grpc_port,
                grpc_port_2d=viewer_grpc_port + 1,
            )

    # Wire state.
    registry = ObjectRegistry(grace_period_s=5.0)
    robot_geometry = RobotGeometryState()
    self_tracker = _SelfTracker(registry, robot_geometry)

    # The map this session belongs to partitions all persistent state.
    # Precedence: mapping's lifecycle broadcast, manifest `map_id`,
    # SCENE_MAP_ID, "default" (mapping usually boots after Scene).
    broadcast = _discover_map_binding(
        float(os.environ.get("SCENE_MAP_BINDING_WAIT_S", "3.0"))
    )
    binding = choose_map_binding(
        broadcast, config.get("map_id"), os.environ.get("SCENE_MAP_ID")
    )
    map_id = binding.map_id
    restore_on_start = _env_flag("SCENE_RESTORE_ON_START", "false")
    scene_state_map_id = map_id if restore_on_start else ".live"
    log.info(
        "[scene] map binding: id=%s gen=%s source=%s mode=%s restore_on_start=%s state_partition=%s",
        binding.map_id,
        binding.generation,
        binding.source,
        binding.mode,
        restore_on_start,
        scene_state_map_id,
    )
    map_ops_lock = asyncio.Lock()
    semantic_hold: dict = {"reason": None}
    live_binding: dict = {
        "map_id": binding.map_id,
        "mode": binding.mode if restore_on_start else "",
        "generation": binding.generation if restore_on_start else None,
        "source": binding.source if restore_on_start else "default",
    }
    if broadcast is not None and not str(broadcast.get("map_id") or ""):
        # An unsaved mapping session never re-anchors a named partition.
        log.warning(
            "[scene] mapping broadcasts an EPHEMERAL session (empty map_id) "
            "while scene binds %r from %s — objects stored under this id "
            "won't re-anchor across boots; Save the current mapping session "
            "as %r first, then Load that saved map or restart in "
            "localization mode",
            binding.map_id,
            binding.source,
            binding.map_id,
        )

    obj_store = None
    if _env_flag("SCENE_OBJECT_MEMORY_ENABLED", "true"):
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
                    "from earlier boots",
                    purged,
                )
            restored = obj_store.load_all() if restore_on_start else []
            if restored:
                async with registry.lock():
                    for o in restored:
                        registry.restore_object(o)
            log.info(
                "[scene-persist] object store ready: restored %d object(s) (partition=%s, restore_on_start=%s) from %s",
                len(restored),
                obj_store.map_id,
                restore_on_start,
                db_path,
            )
        except Exception as e:  # noqa: BLE001
            log.error(
                "[scene-persist] object memory enabled but store init/restore "
                "failed — persistence OFF for this session (no restore, no "
                "writes): %s",
                e,
            )
            obj_store = None

    # Regions and POIs, partitioned like the objects and checked against
    # mapping's generation; a failure disables the annotation API only.
    anno_dir = os.environ.get(
        "SCENE_ANNOTATIONS_DIR", "/data/robonix/scene_annotations"
    )
    if not restore_on_start:
        try:
            for leftover in Path(anno_dir).expanduser().glob(".live*.json"):
                leftover.unlink(missing_ok=True)
        except Exception as e:  # noqa: BLE001
            log.warning(
                "[scene-anno] live-session file cleanup failed (leftovers "
                "may resurface next boot): %s",
                e,
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
            len(anns),
            sum(a.stale for a in anns),
            anno_store.map_id,
            anno_store.path,
        )
    except Exception as e:  # noqa: BLE001
        log.error(
            "[scene-anno] annotation store init failed — annotation API "
            "disabled for this session: %s",
            e,
        )
    map_meta: Optional[MapMetaStore] = None
    try:
        meta_dir = os.environ.get("SCENE_MAP_META_DIR") or os.path.join(
            os.path.dirname(anno_dir.rstrip("/")) or ".", "scene_maps"
        )
        map_meta = MapMetaStore(meta_dir)
    except Exception as e:  # noqa: BLE001
        log.error(
            "[scene-mapmeta] sidecar store init failed — Save/Load will not "
            "snapshot/restore objects this session: %s",
            e,
        )
    mcp_tools.attach_state(
        registry=registry,
        robot_geometry=robot_geometry,
    )
    mcp_tools.attach_annotation_store(anno_store)

    geometry_task = asyncio.create_task(
        reconcile_robot_geometry(robot_geometry),
        name="scene-robot-geometry",
    )

    _declare_tools()

    stub = ATLAS._wire_stub
    hub, perception, ingest_bg = await _start_ros_ingest(
        atlas_stub=stub,
        registry=registry,
        self_tracker=self_tracker,
        config=config,
    )
    mcp_tools.attach_state(  # again, now with the hub for goal_near
        registry=registry,
        hub=hub,
        robot_geometry=robot_geometry,
    )

    # CLIP text embeddings for persistence; placeholders without them.
    if obj_store is not None:
        obj_store.set_embedder(getattr(perception, "embed_text", None))
    bg_tasks = [
        geometry_task,
        asyncio.create_task(_stale_tick(registry), name="scene-stale-tick"),
        # One image per new object to memgraph (SCENE_OBJECT_WATCHDOG=0: off).
        *([asyncio.create_task(
            ObjectWatchdog(
                registry=registry, hub=hub,
            ).run(),
            name="object-watchdog",
        )] if os.environ.get("SCENE_OBJECT_WATCHDOG", "1") in ("1", "true", "yes") else []),
        asyncio.create_task(
            _lifecycle_watch(
                hub,
                binding,
                anno_store,
                registry=registry,
                live_binding=live_binding,
                ops_lock=map_ops_lock,
                semantic_hold=semantic_hold,
            ),
            name="scene-lifecycle-watch",
        ),
        asyncio.create_task(
            _auto_discover_loop(
                atlas_stub=stub,
                hub=hub,
                transport=str(config.get("transport") or "ros2"),
                explicit=(config.get("observations") or []),
                camera_provider_id=str(config.get("camera_provider_id") or "").strip(),
            ),
            name="scene-auto-discover",
        ),
        *ingest_bg,
    ]
    for _t in bg_tasks:
        _t.add_done_callback(_log_bg_task_exit)

    # ── Relation layer ───────────────────────────────────────────────
    # Geometric relations always run; SCENE_GRAPH_ENABLED gates only the
    # LLM enrichment.
    from .scene_graph.geometric_loop import GeometricRelationLoop
    from .scene_graph.store import SceneGraphStore

    sg_cache_dir = os.environ.get(
        "SCENE_GRAPH_CACHE_DIR", "/data/robonix/scene_graph/cache"
    )
    sg_store = SceneGraphStore(cache_dir=sg_cache_dir, map_id=scene_state_map_id)
    log.info(
        "[scene-graph] cache base=%s partitioned by map_id=%s",
        sg_cache_dir,
        map_id,
    )
    mcp_tools.attach_scene_graph_store(sg_store)
    geo_loop = GeometricRelationLoop(registry, sg_store)
    await geo_loop.start()

    # ── Scene Graph (optional LLM enrichment of the residual) ────────
    sg_stop: asyncio.Event | None = None
    if _env_flag("SCENE_GRAPH_ENABLED", "true"):
        from .scene_graph.builder import (
            SceneGraphBuilder,
            SceneGraphConfig,
            scene_graph_loop,
        )
        from .scene_graph.llm_client import SceneGraphLLMClient
        from .scene_graph.relations import RelationInferer

        sg_cfg = SceneGraphConfig()
        sg_llm = SceneGraphLLMClient()
        sg_inferer = RelationInferer(sg_llm)
        sg_builder = SceneGraphBuilder(
            registry=registry,
            relation_inferer=sg_inferer,
            store=sg_store,
            config=sg_cfg,
            # Continuous writes belong to the legacy warm-restore mode only.
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

    # ── Operator corrections: the single writer for object mutations ──
    from .object_mutations import ObjectMutationCoordinator

    object_mutations = ObjectMutationCoordinator(
        registry=registry,
        detector=perception,
        scene_graph_store=sg_store,
        live_binding=live_binding,
        ops_lock=map_ops_lock,
        semantic_hold=semantic_hold,
        object_store=obj_store,
        map_meta=map_meta,
    )
    mcp_tools.attach_object_mutations(object_mutations)

    web_port = _web_port_from(config)
    web_task = None
    web_server: uvicorn.Server | None = None

    def spawn(coro, name: str) -> None:
        task = asyncio.create_task(coro, name=name)
        task.add_done_callback(_log_bg_task_exit)
        bg_tasks.append(task)

    if web_port > 0:
        if rerun_sink is not None:
            if not rerun_sink.available:  # the map pages say the same
                log.warning("[scene-rerun] %s", rerun_sink.detail)
            # Always running; each tick is a no-op until a page opens the viewer.
            try:
                viewer_period = float(
                    os.environ.get("SCENE_RERUN_PERIOD_S", "") or 1.0)
            except ValueError:
                log.warning(
                    "[scene-rerun] SCENE_RERUN_PERIOD_S=%r is not a number; "
                    "publishing once a second",
                    os.environ.get("SCENE_RERUN_PERIOD_S"))
                viewer_period = 1.0
            spawn(_rerun_tick(rerun_sink, registry, perception, hub,
                              sg_store, robot_geometry,
                              period_s=max(0.1, viewer_period)),
                  "scene-rerun")

        # Unset SCENE_OBJECT_VIEWS_DIR means no photographs, and no captions.
        object_views = store_from_env()
        ephemeral_session_id = "session-" + time.strftime(
            "%Y%m%dT%H%M%SZ", time.gmtime())
        if object_views is not None:
            object_views.session_id = ephemeral_session_id
            # Last run's unsaved photographs show a map this one never saw.
            dropped = object_views.forget_stale_sessions(ephemeral_session_id)
            if dropped:
                log.info("[scene-views] discarded %d unsaved session(s) from "
                         "a previous run", dropped)
        if object_views is not None and perception is not None:
            spawn(_object_views_tick(object_views, perception, registry,
                                     live_binding), "scene-views")
            log.info("[scene-views] storing object views under %s (max %d each)",
                     object_views.root, object_views.max_views)
            from .object_captions import caption_loop
            from .scene_graph.llm_client import SceneGraphLLMClient
            spawn(caption_loop(registry, object_views, live_binding,
                               SceneGraphLLMClient()), "scene-captions")
        else:
            log.info("[scene-views] not storing object views "
                     "(SCENE_OBJECT_VIEWS_DIR unset)")

        web_app = web_ui.make_app(
            registry=registry,
            hub=hub,
            detector=perception,
            rerun_sink=rerun_sink,
            sg_store=sg_store,
            anno_store=anno_store,
            object_store=obj_store,
            map_meta=map_meta,
            map_binding=live_binding,
            ops_lock=map_ops_lock,
            semantic_hold=semantic_hold,
            robot_geometry=robot_geometry,
            object_mutations=object_mutations,
            object_views=object_views,
        )
        web_uv = uvicorn.Config(
            app=web_app,
            host=web_host,
            port=web_port,
            log_level="warning",
        )
        web_server = uvicorn.Server(web_uv)
        web_task = asyncio.create_task(web_server.serve(), name="scene-web-http")
        web_task.add_done_callback(_log_bg_task_exit)
        log.info("web UI on http://%s:%d", web_host, web_port)
        # The UI has no authentication and its endpoints write map data.
        if web_host not in ("127.0.0.1", "::1", "localhost"):
            log.warning(
                "web UI is bound to %s, not loopback, and has no "
                "authentication: anyone who can reach %s:%d may read and "
                "modify map annotations.",
                web_host, web_host, web_port,
            )

    log.info(
        "scene up; cap=%s mcp=%s observations=%d",
        scene.id,
        scene.mcp_endpoint,
        len(config.get("observations", [])),
    )
    _lifecycle.mark_runtime_ready()

    await _wait_for_lifecycle_event(_lifecycle.shutdown_requested)
    log.info("shutdown requested; tearing down")

    # SHUTDOWN is acknowledged only once every task, socket and lock is gone.
    await close_scene_runtime_resources(
        background_tasks=bg_tasks,
        scene_graph_stop=sg_stop,
        perception=perception,
        hub=hub,
        geometric_loop=geo_loop,
        web_server=web_server,
        web_task=web_task,
        object_store=obj_store,
    )


async def _run() -> None:
    """Register Scene, receive lifecycle config, then run active resources."""
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        with contextlib.suppress(NotImplementedError):
            loop.add_signal_handler(sig, _lifecycle.request_process_shutdown)

    # Bootstrap first, so rbnx can deliver CMD_INIT.
    scene.use_mcp_app(mcp_tools.mcp)
    scene.bootstrap()

    _declare_tools()

    run_error: BaseException | None = None
    try:
        if not await _wait_for_lifecycle_event(_lifecycle.initialized):
            return
        if not await _wait_for_lifecycle_event(_lifecycle.activation_requested):
            return
        await _run_active(_lifecycle.config)
    except BaseException as exc:
        run_error = exc
        _lifecycle.mark_runtime_failed(exc)
        raise
    finally:
        # A driver SHUTDOWN tears the capability down after its response is
        # sent; a direct signal has no such callback, so it is done here.
        _lifecycle.mark_shutdown_complete(run_error)
        if _lifecycle.driver_shutdown_requested.is_set():
            stopped = await asyncio.to_thread(scene._stopping.wait, 3.0)
            if not stopped:
                log.warning(
                    "Driver(SHUTDOWN) response completion was not observed "
                    "before process exit"
                )
        else:
            scene._teardown()


def main() -> None:
    try:
        asyncio.run(_run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
