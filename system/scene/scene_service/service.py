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


def _build_topic_specs(observations: list[dict], atlas_stub) -> list[TopicSpec]:
    """Resolve each observation entry to a ROS topic.

    Two-step lookup:
      1. If the entry has a `contract`, ask atlas for the cap that
         publishes it over the ROS2 transport. If found, use the
         endpoint atlas reports (= the actual ROS topic name) and
         derive msg_type from the contract's IDL metadata.
      2. Fallback: use the static `topic` + `msg_type` fields from
         the manifest entry.

    The atlas-first path means a soma can re-register its primitives
    on different topics without editing the deploy manifest. The
    static fallback covers contracts whose publisher hasn't been
    migrated to DeclareInterface(transport=ROS2) yet (today: most of
    them — Webots primitives publish to ROS but don't tell atlas
    about it). Once a primitive registers its topic with atlas, the
    fallback becomes unused for that entry.
    """
    out: list[TopicSpec] = []
    for entry in observations:
        kind = str(entry.get("kind", "")).lower()
        if not kind:
            continue

        contract = str(entry.get("contract", ""))
        topic = ""
        msg_type = str(entry.get("msg_type", ""))

        if contract and atlas_stub is not None:
            try:
                resp = atlas_stub.QueryCapabilities(pb.QueryCapabilitiesRequest(
                    contract_id=contract,
                    transport=pb.TRANSPORT_ROS2,
                ))
                for rec in resp.records:
                    for iface in rec.interfaces:
                        if iface.contract_id == contract and iface.transport == pb.TRANSPORT_ROS2:
                            topic = iface.endpoint or ""
                            # Atlas msg_type is on the ROS2 TransportParams
                            # if the publisher set it; otherwise inherits
                            # from the contract IDL — leave the manifest
                            # fallback for that.
                            if hasattr(iface, "params") and hasattr(iface.params, "ros2"):
                                ros2_params = iface.params.ros2
                                if ros2_params.message_type:
                                    msg_type = ros2_params.message_type
                            break
                    if topic:
                        break
            except Exception as e:  # noqa: BLE001
                log.debug("[scene] atlas QueryCapabilities(%s, ROS2) failed: %s", contract, e)

        # Fallback: static topic from manifest.
        if not topic:
            topic = str(entry.get("topic", ""))
        if not topic or not msg_type:
            log.warning(
                "[scene] observation %r: no atlas record for contract=%r and "
                "no static topic/msg_type fallback — skipping",
                kind, contract,
            )
            continue

        log.info("[scene] observation %r → topic=%s msg_type=%s%s",
                 kind, topic, msg_type,
                 f" (atlas:{contract})" if contract and entry.get("topic", "") != topic else "")
        out.append(TopicSpec(kind=kind, topic=topic, msg_type=msg_type))
    return out


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
    port = int(os.environ.get("SCENE_PORT", "50106"))
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
