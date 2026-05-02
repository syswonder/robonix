# SPDX-License-Identifier: MulanPSL-2.0
"""scene_service entrypoint — wires the registry, ingest pollers,
relation engine, FastMCP server, and atlas registration together.

Lifecycle:

  1. Bootstrap codegen / robonix-py paths (mirrors system/memory).
  2. Read RBNX_CONFIG_FILE for the Soma adapter (which observation
     kinds to enable). Defaults make sense for Webots Tiago.
  3. Spin up the asyncio event loop:
       - registry.lock-protected ObjectRegistry
       - RelationEngine periodic task (1 Hz)
       - per-observation pollers (gated by Soma config + atlas
         availability — missing caps are silently skipped)
       - VLMObjectDetector that runs on the rgb stream
       - self-pose tracker that updates the `robot` SceneObject
  4. Register cap + DeclareInterface for each MCP tool.
  5. Start FastMCP HTTP server.
  6. Heartbeat to atlas every 15 s.
  7. SIGTERM/SIGINT → cancel ingest tasks, unregister cap, exit.
"""
from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import os
import signal
import sys
import time
from pathlib import Path
from typing import Any, Optional


# ── Codegen + robonix-py path bootstrap ────────────────────────────────────
def _ensure_proto_gen() -> None:
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "rbnx-build" / "codegen" / "proto_gen"
        if pg.is_dir() and (pg / "atlas_pb2.py").exists():
            if str(pg) not in sys.path:
                sys.path.insert(0, str(pg))
            return
        d = d.parent


def _ensure_mcp_types() -> None:
    d = Path(__file__).resolve().parent
    while d.parent != d:
        mt = d / "rbnx-build" / "codegen" / "robonix_mcp_types"
        if mt.is_dir() and (mt / "__init__.py").exists():
            if str(mt) not in sys.path:
                sys.path.insert(0, str(mt))
            return
        d = d.parent


def _ensure_robonix_py() -> None:
    d = Path(__file__).resolve().parent
    while d.parent != d:
        for cand in (d / "pylib" / "robonix-py", d / "robonix-py"):
            if cand.is_dir() and (cand / "robonix_py" / "__init__.py").exists():
                if str(cand) not in sys.path:
                    sys.path.insert(0, str(cand))
                return
        d = d.parent
    import subprocess
    try:
        out = subprocess.run(
            ["rbnx", "path", "robonix-py"],
            capture_output=True, text=True, timeout=5, check=False,
        )
        if out.returncode == 0:
            lib = Path(out.stdout.strip())
            if lib.is_dir() and str(lib) not in sys.path:
                sys.path.insert(0, str(lib))
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass


_ensure_proto_gen()
_ensure_mcp_types()
_ensure_robonix_py()


import grpc
import atlas_pb2 as pb  # type: ignore
import atlas_pb2_grpc as pb_grpc  # type: ignore

from . import mcp_tools
from . import web as web_ui
from .ingest.perception_vlm import VLMObjectDetector
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


# ── Defaults: Soma adapter for Webots Tiago ────────────────────────────────
# v2 — ROS2-topic-based ingest (scene runs in its own container with
# rclpy and shares the host DDS bus with the primitives). Each entry
# names a logical kind + ROS topic + sensor_msgs / nav_msgs class.
# Pollers (the v1 MCP-poll path) are gone: MCP is for pilot only,
# scene's own data fetching uses the fast direct-DDS path.
_DEFAULT_OBSERVATIONS = [
    {"kind": "rgb",     "topic": "/head_front_camera/rgb/image_raw",                 "msg_type": "Image"},
    {"kind": "depth",   "topic": "/head_front_camera/depth_registered/image_raw",    "msg_type": "Image"},
    {"kind": "lidar2d", "topic": "/scan",                                            "msg_type": "LaserScan"},
    {"kind": "pose",    "topic": "/amcl_pose",                                       "msg_type": "PoseWithCovarianceStamped"},
    {"kind": "odom",    "topic": "/odom",                                            "msg_type": "Odometry"},
]


def _load_config() -> dict:
    """Read RBNX_CONFIG_FILE if set; otherwise use defaults. The file
    contents are a YAML-or-JSON dict; missing keys fall back to
    defaults so partial overrides are easy."""
    path = os.environ.get("RBNX_CONFIG_FILE")
    if not path:
        return {"observations": _DEFAULT_OBSERVATIONS}
    try:
        text = Path(path).read_text()
        try:
            cfg = json.loads(text)
        except json.JSONDecodeError:
            import yaml  # PyYAML is in deps
            cfg = yaml.safe_load(text) or {}
    except Exception as e:  # noqa: BLE001
        log.warning("failed to read %s: %s; using defaults", path, e)
        return {"observations": _DEFAULT_OBSERVATIONS}
    if not isinstance(cfg, dict):
        cfg = {}
    cfg.setdefault("observations", _DEFAULT_OBSERVATIONS)
    return cfg


# Map of (contract leaf → logical scene kind). Primitives DeclareInterface
# their ROS2 topics with contract IDs like `robonix/primitive/camera/rgb`;
# scene auto-classifies by leaf so the manifest doesn't have to enumerate.
# A leaf not in this table is still subscribed (`kind = leaf`) — scene
# just doesn't have specialised consumers for it yet.
_CONTRACT_LEAF_TO_KIND: dict[str, str] = {
    "rgb": "rgb",
    "depth": "depth",
    "depth_registered": "depth",
    # `lidar/lidar` is the canonical 2D LaserScan contract; `lidar2d`
    # is also accepted in case a soma uses the explicit name.
    "lidar": "lidar2d",
    "lidar2d": "lidar2d",
    "scan": "lidar2d",
    "lidar3d": "lidar3d",
    "pointcloud": "lidar3d",
    "pose": "pose",
    "amcl_pose": "pose",
    "odom": "odom",
}

# Optional manifest opt-out: kinds listed here are dropped even if atlas
# advertises them. Useful when a deployment doesn't want scene burning
# CPU on, say, a high-rate depth stream.
_DEFAULT_DISABLED_KINDS: frozenset[str] = frozenset()


def _build_topic_specs(observations: list[dict], atlas_stub) -> list[TopicSpec]:
    """Two paths:

      A. Auto-discovery (default — when manifest's `observations[]` is
         empty / absent). Scene queries atlas for ALL caps + their
         ROS2-transport interfaces, classifies each by contract leaf
         via `_CONTRACT_LEAF_TO_KIND`, and subscribes. Zero per-deploy
         config: a soma plugs in a new sensor primitive that
         DeclareInterface's its topic, scene picks it up on next
         start without touching robonix_manifest.yaml.

      B. Explicit overrides (when manifest provides `observations[]`).
         Each entry is `{kind, contract}`; scene resolves the topic +
         msg_type + qos identically. Used to disable a kind for a
         specific deployment, or to point at a non-canonical contract.

    msg_type derivation in both paths: read `[io.msg].msg` from the
    contract TOML at `/capabilities/<contract_path>.v1.toml`. Split
    on `/`, take the leaf — that's the ROS Python class name. The
    /capabilities dir is bind-mounted into the scene container.

    Errors at any step skip that one entry but never fail bring-up:
    a partially-resolvable scene config is still useful (RGB works
    even if lidar2d primitive isn't running yet).
    """
    if observations:
        return _resolve_explicit(observations, atlas_stub)
    return _resolve_auto(atlas_stub)


def _resolve_auto(atlas_stub) -> list[TopicSpec]:
    """Path A: scan everything atlas knows about, take what looks
    subscribable. We pull a single QueryCapabilities with empty
    contract_id and transport=ROS2 — atlas returns every cap with
    at least one ROS2-transport interface."""
    try:
        resp = atlas_stub.QueryCapabilities(pb.QueryCapabilitiesRequest(
            contract_id="",
            transport=pb.TRANSPORT_ROS2,
        ))
    except Exception as e:  # noqa: BLE001
        log.warning("[scene] atlas QueryCapabilities(*,ROS2) failed: %s", e)
        return []

    seen_kinds: set[str] = set()
    out: list[TopicSpec] = []
    for rec in resp.records:
        for iface in rec.interfaces:
            if iface.transport != pb.TRANSPORT_ROS2:
                continue
            spec = _spec_from_iface(iface)
            if spec is None:
                continue
            if spec.kind in _DEFAULT_DISABLED_KINDS:
                log.info("[scene] auto-discover: skipping kind=%s (in _DEFAULT_DISABLED_KINDS)", spec.kind)
                continue
            if spec.kind in seen_kinds:
                # Multiple primitives advertising the same kind. Take
                # the first (deterministic via atlas's record order).
                log.info("[scene] auto-discover: kind=%s already taken; ignoring %s on %s",
                         spec.kind, iface.contract_id, iface.endpoint)
                continue
            seen_kinds.add(spec.kind)
            log.info("[scene] auto-discover %r ← atlas: topic=%s msg=%s qos=%s contract=%s",
                     spec.kind, spec.topic, spec.msg_type, spec.qos_profile, iface.contract_id)
            out.append(spec)
    return out


def _resolve_explicit(observations: list[dict], atlas_stub) -> list[TopicSpec]:
    """Path B: per-entry contract lookup. Only used when the manifest
    has an explicit `observations[]` list."""
    out: list[TopicSpec] = []
    for entry in observations:
        kind = str(entry.get("kind", "")).lower()
        contract = str(entry.get("contract", ""))
        if not kind or not contract:
            log.warning("[scene] observation %r: missing kind/contract; skipping", entry)
            continue
        try:
            resp = atlas_stub.QueryCapabilities(pb.QueryCapabilitiesRequest(
                contract_id=contract, transport=pb.TRANSPORT_ROS2,
            ))
        except Exception as e:  # noqa: BLE001
            log.warning("[scene] %r: atlas query for %s failed: %s — skipping", kind, contract, e)
            continue
        chosen: Optional[TopicSpec] = None
        for rec in resp.records:
            for iface in rec.interfaces:
                if iface.contract_id != contract or iface.transport != pb.TRANSPORT_ROS2:
                    continue
                chosen = _spec_from_iface(iface, kind_override=kind)
                if chosen:
                    break
            if chosen:
                break
        if chosen is None:
            log.warning(
                "[scene] %r: no atlas record for contract=%r over ROS2 — skipping. "
                "Have a primitive DeclareInterface(transport=ROS2) for it.",
                kind, contract,
            )
            continue
        log.info("[scene] override %r ← atlas: topic=%s msg=%s qos=%s",
                 chosen.kind, chosen.topic, chosen.msg_type, chosen.qos_profile)
        out.append(chosen)
    return out


def _spec_from_iface(iface, *, kind_override: Optional[str] = None) -> Optional[TopicSpec]:
    """Build a TopicSpec from one atlas Iface. Resolves msg_type from
    the contract TOML on disk. Returns None when the interface isn't
    usable (no [io.msg], unknown contract leaf, missing endpoint)."""
    topic = (iface.endpoint or "").strip()
    if not topic:
        return None
    contract = iface.contract_id
    msg_type = _msg_type_from_contract(contract)
    if not msg_type:
        # Contract is rpc/srv (no [io.msg]) — not subscribable.
        return None
    qos_profile = ""
    if hasattr(iface, "params") and hasattr(iface.params, "ros2"):
        qos_profile = iface.params.ros2.qos_profile or ""
    leaf = contract.rsplit("/", 1)[-1].lower()
    kind = kind_override or _CONTRACT_LEAF_TO_KIND.get(leaf, leaf)
    return TopicSpec(
        kind=kind,
        topic=topic,
        msg_type=msg_type,
        qos_profile=qos_profile or "default",
    )


# Lazy-cached reverse map: contract_id → "Image" / "LaserScan" / …
_CONTRACT_MSG_CACHE: dict[str, str] = {}


def _msg_type_from_contract(contract_id: str) -> str:
    """Read `[io.msg].msg` from `/capabilities/<contract_path>.v1.toml`
    and return the Python class name. Returns "" when the contract is
    RPC-only (no `[io.msg]` block) or the TOML can't be read.

    Cache because we call this once per observation at startup and
    contracts don't change at runtime; keep the implementation in one
    place rather than littering atlas-resolution callers with TOML
    parsing.
    """
    if contract_id in _CONTRACT_MSG_CACHE:
        return _CONTRACT_MSG_CACHE[contract_id]
    # contract_id like "robonix/primitive/camera/rgb" → file is at
    # /capabilities/primitive/camera/rgb.v1.toml. Strip the leading
    # "robonix/" namespace prefix.
    parts = contract_id.split("/")
    if not parts or parts[0] != "robonix":
        _CONTRACT_MSG_CACHE[contract_id] = ""
        return ""
    rel = "/".join(parts[1:]) + ".v1.toml"
    candidates = [
        Path("/capabilities") / rel,
        Path("/scene/.capabilities-mirror") / rel,  # alt mount path
    ]
    for path in candidates:
        if not path.is_file():
            continue
        try:
            try:
                import tomllib  # 3.11+
                data = tomllib.loads(path.read_text())
            except ImportError:
                import tomli  # noqa: F401  # 3.10
                data = __import__("tomli").loads(path.read_text())
        except Exception as e:  # noqa: BLE001
            log.debug("[scene] failed to parse %s: %s", path, e)
            _CONTRACT_MSG_CACHE[contract_id] = ""
            return ""
        msg_field = (data.get("io", {}) or {}).get("msg", {}) or {}
        msg = str(msg_field.get("msg", ""))
        if not msg:
            _CONTRACT_MSG_CACHE[contract_id] = ""
            return ""
        # "sensor_msgs/msg/Image" → "Image"
        leaf = msg.rsplit("/", 1)[-1]
        _CONTRACT_MSG_CACHE[contract_id] = leaf
        return leaf
    log.debug("[scene] no contract TOML found for %s in %s",
              contract_id, [str(p) for p in candidates])
    _CONTRACT_MSG_CACHE[contract_id] = ""
    return ""


# ── DeclareInterface helper (mirrors system/memory) ────────────────────────
def _decl_mcp(stub, cap_id: str, contract_id: str, port: int, fn) -> None:
    """Register one MCP tool with atlas. The schema comes from the
    @mcp_contract input class stashed on `fn`."""
    description = (fn.__doc__ or "").strip()
    input_cls = getattr(fn, "_robonix_input_cls", None)
    schema_json = json.dumps(
        input_cls.json_schema() if input_cls is not None
        else {"type": "object", "properties": {}, "required": []}
    )
    stub.DeclareInterface(pb.DeclareInterfaceRequest(
        capability_id=cap_id,
        contract_id=contract_id,
        transport=pb.TRANSPORT_MCP,
        endpoint=f"http://127.0.0.1:{port}/mcp/",
        params=pb.TransportParams(mcp=pb.McpParams(
            description=description,
            input_schema_json=schema_json,
        )),
    ))


# ── Self-pose tracker ──────────────────────────────────────────────────────
class _SelfTracker:
    """Owns the `robot` SceneObject — created on first chassis-state
    poll, then EMA-updated. Never goes `missing` (we just stop refreshing
    if chassis stops responding). Also exposes a sync `latest_xy_yaw`
    callback that the VLM detector consumes for camera-to-map projection.
    """

    def __init__(self, registry: ObjectRegistry) -> None:
        self.registry = registry
        self._latest: Optional[tuple[float, float, float, float]] = None
        self._object_id: Optional[str] = None

    def latest_xy_yaw(self) -> Optional[tuple[float, float, float, float]]:
        return self._latest

    async def on_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        self._latest = (x, y, z, yaw)
        async with self.registry.lock():
            if self._object_id is None or self._object_id not in self.registry._objects:  # noqa: SLF001
                obj = self.registry.insert_object(
                    cls="robot",
                    pose=Pose3D(x=x, y=y, z=z, yaw=yaw, frame_id="map"),
                    bbox=BBox3D(size_x=0.6, size_y=0.6, size_z=1.5, frame_id="map"),
                    confidence=1.0,
                    now=now_unix(),
                    is_robot=True,
                    source="self",
                )
                self._object_id = obj.object_id
                log.info("[self] registered self-object %s", self._object_id)
            else:
                obj = self.registry.get_object(self._object_id)
                if obj is not None:
                    self.registry.update_object_pose(
                        obj,
                        Pose3D(x=x, y=y, z=z, yaw=yaw, frame_id="map"),
                        new_confidence=1.0,
                        now=now_unix(),
                        ema_pose=1.0,  # robot's own pose: hard-overwrite
                        ema_conf=1.0,
                    )


# ── Heartbeat task ─────────────────────────────────────────────────────────
async def _heartbeat_loop(stub, cap_id: str, *, period_s: float = 15.0) -> None:
    """Periodic NodeHeartbeat → atlas. Without this atlas evicts our
    cap after the configured timeout (default 60s) and Pilot can no
    longer call our tools."""
    loop = asyncio.get_running_loop()
    while True:
        try:
            await loop.run_in_executor(
                None, lambda: stub.Heartbeat(pb.HeartbeatRequest(capability_id=cap_id))
            )
        except Exception as e:  # noqa: BLE001
            log.debug("heartbeat failed (will retry): %s", e)
        await asyncio.sleep(period_s)


# ── Stale-tick: flip missing flag after grace period ───────────────────────
async def _stale_tick(registry: ObjectRegistry, *, period_s: float = 1.0) -> None:
    while True:
        async with registry.lock():
            flipped = registry.mark_stale(now_unix())
        if flipped:
            log.debug("marked %d object(s) missing (grace expired)", flipped)
        await asyncio.sleep(period_s)


# ── Wire ROS subscribers + downstream consumers ────────────────────────────
async def _start_ros_ingest(
    *,
    atlas_stub,
    registry: ObjectRegistry,
    self_tracker: "_SelfTracker",
    config: dict,
) -> tuple[SubscribersHub, Optional[VLMObjectDetector], list[asyncio.Task]]:
    """Bring up the rclpy hub + the per-kind consumers (self-pose
    bridge, VLM perception). Returns (hub, vlm_or_None, bg_tasks_for_shutdown).

    Each consumer is its own asyncio task so a hung VLM call doesn't
    starve the pose updater, and vice versa."""
    specs = _build_topic_specs(config.get("observations") or [], atlas_stub)
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
    if hub.has("pose") or hub.has("odom"):
        bg_tasks.append(asyncio.create_task(
            _self_pose_loop(hub, self_tracker), name="scene-self-pose"
        ))

    # ── VLM perception ─────────────────────────────────────────────────────
    vlm: Optional[VLMObjectDetector] = None
    if hub.has("rgb"):
        # Convert sensor_msgs/Image to JPEG bytes lazily on each tick.
        # cv_bridge is too heavy to import at module-top; we only need
        # the encoder here.
        def _rgb_jpeg() -> Optional[bytes]:
            msg, stamp, _ = hub.latest("rgb")
            if msg is None or stamp == 0.0:
                return None
            return _image_msg_to_jpeg(msg)

        vlm = VLMObjectDetector(
            rgb_fetcher=_rgb_jpeg,
            chassis_pose_fn=self_tracker.latest_xy_yaw,
            on_detections=lambda dets: _ingest_detections(registry, dets),
            period_s=4.0,  # one VLM call every ~4 s; LLM calls aren't free
        )
        await vlm.start()

    return hub, vlm, bg_tasks


async def _self_pose_loop(hub: SubscribersHub, self_tracker: "_SelfTracker") -> None:
    """Read latest amcl_pose / odom from the hub, feed SelfTracker.
    /amcl_pose is preferred (post-localisation); /odom is the fallback
    early in a session before AMCL has converged."""
    last_count_pose = 0
    last_count_odom = 0
    while True:
        if hub.has("pose"):
            msg, stamp, count = hub.latest("pose")
            if msg is not None and count != last_count_pose:
                last_count_pose = count
                p = msg.pose.pose  # PoseWithCovarianceStamped
                x, y, z = p.position.x, p.position.y, p.position.z
                yaw = _quat_to_yaw(p.orientation.x, p.orientation.y,
                                   p.orientation.z, p.orientation.w)
                await self_tracker.on_pose(x, y, z, yaw)
        if hub.has("odom") and last_count_pose == 0:
            # Only use odom as backup until amcl publishes.
            msg, stamp, count = hub.latest("odom")
            if msg is not None and count != last_count_odom:
                last_count_odom = count
                p = msg.pose.pose
                yaw = _quat_to_yaw(p.orientation.x, p.orientation.y,
                                   p.orientation.z, p.orientation.w)
                await self_tracker.on_pose(p.position.x, p.position.y, p.position.z, yaw)
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
        log.info("[detect] %d matched, %d new — registry: %s",
                 len(matched), len(new), registry.stats())


# ── main ───────────────────────────────────────────────────────────────────
async def _run() -> None:
    config = _load_config()
    atlas_addr = os.environ.get("ROBONIX_ATLAS", "localhost:50051")
    # Port resolution: deploy-manifest config wins; env var is the
    # fallback so package devs can still override via shell. Defaults
    # are 50106 (MCP HTTP for pilot) and 50107 (debug web UI).
    port = int(config.get("mcp_port") or os.environ.get("SCENE_PORT", "50106"))
    cap_id = os.environ.get("ROBONIX_CAPABILITY_ID", "com.robonix.system.scene")

    channel = grpc.insecure_channel(atlas_addr)
    stub = pb_grpc.AtlasStub(channel)

    # Atlas registration.
    pkg_dir = os.environ.get("ROBONIX_PKG_HOST_DIR", "")
    md_path = f"{pkg_dir}/CAPABILITY.md" if pkg_dir else ""
    try:
        stub.RegisterCapability(pb.RegisterCapabilityRequest(
            capability_id=cap_id,
            namespace="robonix/system/scene",
            capability_md_path=md_path,
        ))
    except Exception as e:  # noqa: BLE001
        log.warning("RegisterCapability failed (will keep running): %s", e)

    # Wire state.
    registry = ObjectRegistry(grace_period_s=5.0)
    relations = RelationEngine(registry, period_s=1.0)
    await relations.start()
    self_tracker = _SelfTracker(registry)
    mcp_tools.attach_state(registry=registry, relations=relations, transform_to_map=None)

    # Declare MCP tools to atlas (must happen after RegisterCapability).
    try:
        for cid, fn in (
            ("robonix/system/scene/get_snapshot",       mcp_tools.get_snapshot),
            ("robonix/system/scene/query",              mcp_tools.query),
            ("robonix/system/scene/get_object",         mcp_tools.get_object),
            ("robonix/system/scene/get_semantic_map",   mcp_tools.get_semantic_map),
            ("robonix/system/scene/get_safety_context", mcp_tools.get_safety_context),
        ):
            _decl_mcp(stub, cap_id, cid, port, fn)
        log.info("registered cap %s with 5 MCP interfaces on port %d", cap_id, port)
    except Exception as e:  # noqa: BLE001
        log.warning("DeclareInterface failed: %s", e)

    # ROS2 ingest hub + downstream consumers (self-pose, VLM perception).
    hub, vlm, ingest_bg = await _start_ros_ingest(
        atlas_stub=stub, registry=registry, self_tracker=self_tracker, config=config,
    )
    bg_tasks = [
        asyncio.create_task(_heartbeat_loop(stub, cap_id), name="scene-heartbeat"),
        asyncio.create_task(_stale_tick(registry), name="scene-stale-tick"),
        *ingest_bg,
    ]

    # FastMCP HTTP server. Bind 0.0.0.0 (not 127.0.0.1) because scene
    # runs in a container with --network host; binding to loopback
    # would still work via host network but explicit 0.0.0.0 makes it
    # easier to swap to bridge networking later if we ever drop
    # --network host.
    import uvicorn
    config_uv = uvicorn.Config(
        app=mcp_tools.mcp.streamable_http_app(),
        host="0.0.0.0", port=port, log_level="warning",
    )
    server = uvicorn.Server(config_uv)
    server_task = asyncio.create_task(server.serve(), name="scene-mcp-http")

    # Web debug UI on a separate port — top-down 2D canvas + objects
    # table + robot pose. Lives in the same asyncio loop as the rest
    # of scene so registry reads are local. Set `web_port: 0` in the
    # deploy-manifest scene block to disable. SCENE_WEB_PORT env is
    # the override of last resort.
    web_port = int(config.get("web_port") if config.get("web_port") is not None
                   else os.environ.get("SCENE_WEB_PORT", "50107"))
    web_task = None
    if web_port > 0:
        web_app = web_ui.make_app(registry=registry, relations=relations)
        web_uv = uvicorn.Config(
            app=web_app, host="0.0.0.0", port=web_port, log_level="warning",
        )
        web_server = uvicorn.Server(web_uv)
        web_task = asyncio.create_task(web_server.serve(), name="scene-web-http")
        log.info("web UI on http://0.0.0.0:%d", web_port)

    log.info("scene up; cap=%s port=%d atlas=%s observations=%d",
             cap_id, port, atlas_addr, len(config.get("observations", [])))

    # Wait for SIGTERM/SIGINT.
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        with contextlib.suppress(NotImplementedError):
            loop.add_signal_handler(sig, stop.set)
    await stop.wait()
    log.info("shutdown signal received; tearing down")

    # Tear down ingest first so we stop mutating the registry…
    if vlm is not None:
        with contextlib.suppress(Exception):
            await vlm.stop()
    with contextlib.suppress(Exception):
        await hub.stop()
    await relations.stop()
    for t in bg_tasks:
        t.cancel()
    server.should_exit = True

    # Unregister from atlas (best effort).
    try:
        stub.UnregisterCapability(pb.UnregisterCapabilityRequest(capability_id=cap_id))
    except Exception:  # noqa: BLE001
        pass

    with contextlib.suppress(Exception):
        await asyncio.wait_for(server_task, timeout=5.0)


def main() -> None:
    try:
        asyncio.run(_run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
