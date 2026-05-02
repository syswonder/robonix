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
from .ingest.poll_primitive import ChassisStatePoller, PrimitivePoller
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
_DEFAULT_OBSERVATIONS = [
    {"kind": "rgb",     "contract": "robonix/primitive/camera/snapshot",       "period_s": 3.0},
    {"kind": "lidar2d", "contract": "robonix/primitive/lidar/snapshot",        "period_s": 2.0},
    {"kind": "odom",    "contract": "robonix/primitive/chassis/state",         "period_s": 1.0},
    # `lidar3d` and `depth` are ignored when no matching cap is on
    # atlas; uncomment + extend if a sensor profile actually has them.
    # {"kind": "lidar3d", "contract": "robonix/primitive/lidar/lidar3d",        "period_s": 1.0},
    # {"kind": "depth",   "contract": "robonix/primitive/camera/depth_snapshot","period_s": 3.0},
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


# ── Wire pollers per Soma config ───────────────────────────────────────────
async def _start_pollers(
    *,
    atlas_stub,
    registry: ObjectRegistry,
    self_tracker: _SelfTracker,
    config: dict,
) -> tuple[list[Any], Optional[VLMObjectDetector]]:
    """For each observation kind in config, attempt to start the
    corresponding poller. Missing caps → log + skip. Returns the list
    of started tasks for shutdown."""
    started: list[Any] = []
    rgb_poller: Optional[PrimitivePoller] = None
    vlm: Optional[VLMObjectDetector] = None

    for entry in config.get("observations") or []:
        kind = str(entry.get("kind", "")).lower()
        contract = str(entry.get("contract", ""))
        period_s = float(entry.get("period_s", 2.0))
        if not kind or not contract:
            continue
        try:
            if kind == "odom":
                p = ChassisStatePoller(
                    atlas_stub=atlas_stub, pb=pb,
                    period_s=period_s,
                    on_pose=self_tracker.on_pose,
                )
                await p.start()
                started.append(p)
            elif kind == "rgb":
                async def _save_rgb_payload(payload: dict) -> None:
                    rgb_poller_state["last"] = payload  # noqa: F821
                rgb_poller_state: dict[str, Any] = {"last": None}

                rgb_poller = PrimitivePoller(
                    atlas_stub=atlas_stub, pb=pb,
                    contract_id=contract,
                    period_s=period_s,
                    on_result=_save_rgb_payload,
                    name="rgb",
                )
                await rgb_poller.start()
                started.append(rgb_poller)

                # Wire the VLM detector against rgb_poller's last payload.
                async def _rgb_fetch() -> Optional[dict]:
                    return rgb_poller_state["last"]

                vlm = VLMObjectDetector(
                    rgb_fetcher=_rgb_fetch,
                    chassis_pose_fn=self_tracker.latest_xy_yaw,
                    on_detections=lambda dets: _ingest_detections(registry, dets),
                    period_s=max(period_s, 2.0),
                )
                await vlm.start()
                started.append(vlm)
            else:
                # lidar2d / lidar3d / depth: launched as raw pollers but
                # we don't yet have downstream consumers in v1. The
                # Soma adapter still spawns them so we know whether
                # caps are present (visible in scene logs) — the
                # ingest pipeline will start using them once the
                # geom layer learns to consume.
                async def _drop_payload(_: dict) -> None:
                    return None

                p = PrimitivePoller(
                    atlas_stub=atlas_stub, pb=pb,
                    contract_id=contract,
                    period_s=period_s,
                    on_result=_drop_payload,
                    name=kind,
                )
                await p.start()
                started.append(p)
        except Exception as e:  # noqa: BLE001
            log.warning("failed to start poller %s (%s): %s", kind, contract, e)

    return started, vlm


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

    # Spin pollers / detector / heartbeat / stale-tick.
    started, _vlm = await _start_pollers(
        atlas_stub=stub, registry=registry, self_tracker=self_tracker, config=config,
    )
    bg_tasks = [
        asyncio.create_task(_heartbeat_loop(stub, cap_id), name="scene-heartbeat"),
        asyncio.create_task(_stale_tick(registry), name="scene-stale-tick"),
    ]

    # FastMCP HTTP server. Run uvicorn inside an executor since it
    # blocks the loop otherwise. uvicorn 0.30+ has a programmatic
    # async API but we keep parity with system/memory's pattern.
    import uvicorn
    config_uv = uvicorn.Config(
        app=mcp_tools.mcp.streamable_http_app(),
        host="127.0.0.1", port=port, log_level="warning",
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
    for t in started:
        try:
            await t.stop()
        except Exception:  # noqa: BLE001
            pass
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
