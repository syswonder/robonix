# SPDX-License-Identifier: MulanPSL-2.0
"""Tiny live web UI for scene — top-down 2D canvas of objects + robot,
plus a side panel listing every tracked object. Single static HTML
served from `/`, JSON state at `/api/state` polled by the page at 5 Hz.

Bound on a separate port from FastMCP (default 50107) so the LLM /
pilot path and the human-debug path don't share a uvicorn — they have
different latency tolerance and stop semantics. Lives in the same
asyncio loop as the rest of scene though, so reading `_REGISTRY` is
cheap (no IPC).

What it does NOT show yet: 3D pose Z, surface bbox extents, mapping
service's occupancy grid (mapping isn't deployed yet — once it is,
the JSON will grow an `occupancy:` field that the canvas overlays).
"""
from __future__ import annotations

import asyncio
import math
import base64
import io
import json
import logging
import os
import re
import time
from pathlib import Path
from typing import Any, Optional

from starlette.applications import Starlette
from starlette.responses import HTMLResponse, JSONResponse, Response
from starlette.routing import Mount, Route
from starlette.staticfiles import StaticFiles

from robonix_api import ATLAS

from .annotations import validate_annotation_fields
from .map_binding import sanitize_map_id as _sanitize_map_id
from .map_meta import make_meta
from .state import ObjectRegistry

log = logging.getLogger(__name__)


_INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>scene — robonix system service</title>
  <style>
    :root { --fg:#e8eaed; --bg:#0e1015; --acc:#7aa7ff; --muted:#7d828b; }
    html, body { background: var(--bg); color: var(--fg); margin: 0; padding: 0;
      font-family: -apple-system, BlinkMacSystemFont, "SF Mono", "Segoe UI", sans-serif; height: 100%; }
    /* The 2D view embeds as an iframe inside the combined layout. The
       info side panel was fighting that iframe for width on small
       screens (MacBook Air etc.) — moved out to a floating top-level
       panel in `_COMBINED_HTML`. The canvas now claims the full iframe
       width unconditionally. */
    #wrap { display: block; height: 100vh; }
    #canvas-wrap { position: relative; width: 100%; height: 100%; }
    canvas { display: block; width: 100%; height: 100%; background: #14171f; }
    .legend { position: absolute; bottom: 8px; left: 12px; font-size: 11px;
      color: var(--muted); background: rgba(20,23,31,.85); padding: 4px 8px; border-radius: 4px; }
    .scene-status { position: absolute; top: 10px; left: 12px; z-index: 2;
      color: var(--muted); background: rgba(20,23,31,.92); border: 1px solid #33405a;
      padding: 5px 9px; border-radius: 5px; font-size: 12px; }
    .scene-status.error { color: #ff8b82; border-color: #6b3640; }
  </style>
</head>
<body data-ready="loading">
<div id="wrap">
  <div id="canvas-wrap">
    <canvas id="c"></canvas>
    <div class="scene-status" id="scene-status" role="status">loading Scene state…</div>
    <div class="legend">scene · 1 m grid · north = +x · 5 Hz</div>
  </div>
</div>
<script>
const c = document.getElementById('c');
const ctx = c.getContext('2d');
const sceneStatus = document.getElementById('scene-status');
let currentState = null;
function fit() { c.width = c.clientWidth; c.height = c.clientHeight; }
window.addEventListener('resize', () => {
    fit();
    if (currentState) draw(currentState);
});
document.addEventListener('visibilitychange', () => {
    if (!document.hidden && currentState) draw(currentState);
});
fit();

function setSceneStatus(text, kind = '') {
    sceneStatus.textContent = text || '';
    sceneStatus.className = 'scene-status' + (kind ? ' ' + kind : '');
    sceneStatus.style.display = text ? 'block' : 'none';
}
function setSceneReady(value) { document.body.dataset.ready = value; }

// World-to-pixel: center on robot if known, else (0,0). 1m = 40 px.
let center = [0, 0];
let pxPerM = 40;
function w2p(x, y) {
    const cx = c.width / 2, cy = c.height / 2;
    return [cx + (x - center[0]) * pxPerM, cy - (y - center[1]) * pxPerM];
}

const CLS_COLORS = {
    robot: '#7aa7ff', table: '#f0c674', chair: '#e9b06b', monitor: '#88c0d0',
    person: '#f55', cup: '#a3be8c', bottle: '#a3be8c', tray: '#d08770',
    door: '#bf616a', plant: '#a3be8c', cabinet: '#d08770',
    keyboard: '#88c0d0', book: '#88c0d0', light_fixture: '#ebcb8b',
};

function classColor(cls) { return CLS_COLORS[cls] || '#9aa0a6'; }

// Cache the decoded occupancy image so the canvas doesn't re-decode
// every 200 ms tick. Re-decode only when state.occupancy.stamp_ms changes.
let occImg = null;
let occMeta = null;
let occStamp = 0;
let occLoading = 0;
let occLoadToken = 0;

function isCurrentOccupancyLoad(token, stamp) {
    return token === occLoadToken && stamp === occLoading;
}

function draw(state) {
    fit();
    ctx.clearRect(0, 0, c.width, c.height);

    // re-center on the robot if there is one; fall back to last-known.
    const robot = (state.objects || []).find(o => o.cls === 'robot');
    if (robot) center = [robot.pose.x, robot.pose.y];

    // ── Occupancy map underlay ──────────────────────────────────────
    if (state.occupancy && state.occupancy.stamp_ms !== occStamp
        && state.occupancy.stamp_ms !== occLoading) {
        occLoading = state.occupancy.stamp_ms;
        const loadToken = ++occLoadToken;
        const meta = state.occupancy;
        const im = new Image();
        im.onload = () => {
            if (!isCurrentOccupancyLoad(loadToken, meta.stamp_ms)) return;
            occImg = im;
            occMeta = meta;
            occStamp = meta.stamp_ms;
            occLoading = 0;
            if (currentState) draw(currentState);
        };
        im.onerror = () => {
            if (!isCurrentOccupancyLoad(loadToken, meta.stamp_ms)) return;
            occLoading = 0;
            setSceneReady('error');
            setSceneStatus('Scene map image could not be decoded; retrying…', 'error');
            console.error('occupancy PNG decode failed');
        };
        im.src = 'data:image/png;base64,' + meta.png_b64;
    }
    if (occImg && occMeta) {
        // Map cell [0,0] is at world (origin_x, origin_y). Width/height
        // in cells; resolution in m/cell. Map y grows up but image y
        // grows down → flip via negative scale.
        const wMeters = occMeta.width * occMeta.resolution;
        const hMeters = occMeta.height * occMeta.resolution;
        const [x0, y0] = w2p(occMeta.origin_x, occMeta.origin_y + hMeters);
        const wPx = wMeters * pxPerM;
        const hPx = hMeters * pxPerM;
        ctx.globalAlpha = 0.85;
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(occImg, x0, y0, wPx, hPx);
        ctx.globalAlpha = 1.0;
    }

    // 1m grid — subtle dashed lines so they don't fight the occupancy.
    ctx.save();
    ctx.strokeStyle = 'rgba(255,255,255,0.06)';
    ctx.setLineDash([2, 6]);
    ctx.lineWidth = 1;
    const w = c.width, h = c.height;
    const stepPx = pxPerM; // 1 m
    const offsetX = ((c.width / 2) - center[0] * pxPerM) % stepPx;
    const offsetY = ((c.height / 2) + center[1] * pxPerM) % stepPx;
    for (let x = offsetX - stepPx; x < w; x += stepPx) {
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    }
    for (let y = offsetY - stepPx; y < h; y += stepPx) {
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
    }
    ctx.restore();

    // axes through robot — same subtle treatment, slightly stronger
    const [rxp, ryp] = w2p(center[0], center[1]);
    ctx.save();
    ctx.strokeStyle = 'rgba(255,255,255,0.10)';
    ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(0, ryp); ctx.lineTo(w, ryp); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(rxp, 0); ctx.lineTo(rxp, h); ctx.stroke();
    ctx.restore();

    // objects — only the cls label on map, with high-contrast outline
    // so it reads against any occupancy background (free / unknown /
    // occupied are all different shades).
    ctx.font = 'bold 12px ui-monospace, monospace';
    ctx.textBaseline = 'middle';
    for (const o of (state.objects || [])) {
        const [px, py] = w2p(o.pose.x, o.pose.y);
        const r = Math.max(4, Math.min(20, (o.bbox.size_x || 0.2) * pxPerM * 0.5));
        const color = classColor(o.cls);
        ctx.globalAlpha = o.missing ? 0.3 : 1.0;
        ctx.fillStyle = color;
        ctx.beginPath(); ctx.arc(px, py, r, 0, Math.PI * 2); ctx.fill();
        ctx.globalAlpha = 1;
        // Label = just the class. Black halo outline + class-coloured
        // fill keeps it legible on white free space, dark unknown,
        // and grey occupied alike.
        const text = o.cls;
        const tx = px + r + 5, ty = py;
        ctx.lineWidth = 3;
        ctx.strokeStyle = 'rgba(0,0,0,0.85)';
        ctx.strokeText(text, tx, ty);
        ctx.fillStyle = color;
        ctx.fillText(text, tx, ty);
    }

    // Relation edges are NOT drawn on the map any more. Overlapping
    // object dots + crossing dashed lines were unreadable once more than
    // a couple of edges existed; relations now read as an explicit text
    // list in the floating info panel (see `info-rels` in _COMBINED_HTML).

    // robot heading arrow
    if (robot) {
        const yaw = robot.pose.yaw || 0;
        const [rx, ry] = w2p(robot.pose.x, robot.pose.y);
        const len = 22;
        ctx.strokeStyle = classColor('robot'); ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(rx, ry);
        ctx.lineTo(rx + Math.cos(yaw) * len, ry - Math.sin(yaw) * len);
        ctx.stroke();
        ctx.lineWidth = 1;
    }

    const canvasVisible = c.clientWidth > 0 && c.clientHeight > 0;
    if (canvasVisible
        && (!state.occupancy || (occImg && occStamp === state.occupancy.stamp_ms))) {
        setSceneReady('true');
        setSceneStatus('');
    } else {
        setSceneReady('loading');
        setSceneStatus(canvasVisible ? 'loading occupancy map…' : 'waiting for visible canvas…');
    }
}

function fmt(n) { return Number(n).toFixed(2); }

async function tick() {
    try {
        const r = await fetch('/api/state', { cache: 'no-store' });
        if (!r.ok) {
            setSceneReady('error');
            setSceneStatus(`Scene state unavailable (HTTP ${r.status}); retrying…`, 'error');
            return;
        }
        currentState = await r.json();
        draw(currentState);
        // Info panel was moved to a top-level floating overlay in
        // _COMBINED_HTML; this iframe no longer has any DOM for it.
    } catch (error) {
        setSceneReady('error');
        setSceneStatus(`Scene state unavailable; retrying… (${error})`, 'error');
    }
}
// 2Hz tick. The map updates at 1 Hz from slam_toolbox; the robot
// pose is interpolated visually so 500ms is smooth enough without
// blasting the canvas with redraws every frame.
setInterval(tick, 500);
tick();
</script>
</body>
</html>
"""


def _shorten_id(object_id: str) -> str:
    # `scene.object.cup_001` → `cup_001` for the table.
    return object_id.split(".", 2)[-1]


# Cache the rendered PNG keyed by the hub's monotonically-increasing
# message count. mapping publishes /map at ~1 Hz; with 3 page pollers
# at 2-4 Hz each we'd otherwise re-encode the whole grid 6-12×/sec —
# pure waste. With this cache, re-encode is gated to "once per new
# message", so steady-state /api/state cost drops to a dict lookup.
_OCCUPANCY_CACHE: dict[str, Any] = {"count": -1, "payload": None}

# Camera previews are much larger than the occupancy thumbnail (a Go2 RGB
# frame is commonly 1920x1080), and `/cam` polls several times per second.
# Keep independent caches because RGB and depth advance at different rates.
# Cache ``None`` as well: an unsupported frame must not be re-encoded on every
# GET while its hub message count is unchanged.
_CAMERA_CACHE: dict[str, dict[str, Any]] = {
    "rgb": {"hub": None, "count": -1, "payload": None},
    "depth": {"hub": None, "count": -1, "payload": None},
}
_CAMERA_PREVIEW_MIN_INTERVAL_S = 0.4


def _occupancy_payload(hub: Any) -> Optional[dict]:
    """Encode the latest OccupancyGrid (from /map via hub) as a small
    PNG + metadata. Cached by hub message count — only re-encodes when
    a fresh /map arrives. Returns None when no map is available yet
    or rendering fails (e.g. numpy missing)."""
    if hub is None or not hub.has("occupancy_grid"):
        return None
    msg, stamp_unix, count = hub.latest("occupancy_grid")
    if msg is None or count == 0:
        return None
    if _OCCUPANCY_CACHE["count"] == count:
        return _OCCUPANCY_CACHE["payload"]
    try:
        import numpy as np
        from PIL import Image as PILImage
    except ImportError:
        log.debug("[web] occupancy: numpy/Pillow unavailable; skipping render")
        return None
    info = msg.info
    w, h = int(info.width), int(info.height)
    if w == 0 or h == 0:
        return None
    # nav_msgs/OccupancyGrid data is row-major bottom-up int8 in
    # [-1, 100]: -1 unknown, 0 free, 100 occupied. Render as grayscale:
    # unknown=128 (mid), free=240 (almost white), occupied=20 (almost black).
    arr = np.frombuffer(bytes(msg.data), dtype=np.int8).reshape(h, w)
    out = np.full((h, w), 128, dtype=np.uint8)
    out[arr == 0]   = 240
    out[arr == 100] = 20
    # nav_msgs y origin is bottom-left; PNG image y is top-left → flip.
    out = np.flipud(out)
    buf = io.BytesIO()
    PILImage.fromarray(out, mode="L").save(buf, format="PNG", optimize=False)
    payload = {
        "width": w,
        "height": h,
        "resolution": float(info.resolution),
        "origin_x": float(info.origin.position.x),
        "origin_y": float(info.origin.position.y),
        "stamp_ms": int(stamp_unix * 1000),
        "png_b64": base64.b64encode(buf.getvalue()).decode("ascii"),
    }
    _OCCUPANCY_CACHE["count"] = count
    _OCCUPANCY_CACHE["payload"] = payload
    return payload


def _image_to_png_b64(msg: Any, *, kind: str) -> Optional[dict]:
    """sensor_msgs/Image → {png_b64, w, h, encoding, stamp_ms}.

    `kind` is "rgb" or "depth"; depth is normalised per-frame to a
    grayscale visualisation (raw ranges aren't human-meaningful in a
    debug panel). Returns None if the encoding is unsupported or the
    PIL import fails.
    """
    try:
        import numpy as np
        from PIL import Image as PILImage
    except ImportError:
        return None
    h, w = int(msg.height), int(msg.width)
    if h == 0 or w == 0:
        return None
    enc = (msg.encoding or "").lower()
    arr: Any = None
    out_mode = "RGB"
    if kind == "rgb":
        if enc == "rgb8":
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w, 3)
        elif enc == "bgr8":
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w, 3)[:, :, ::-1]
        elif enc == "rgba8":
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
        elif enc == "bgra8":
            # webots head camera publishes BGRA8.
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w, 4)[:, :, :3][:, :, ::-1]
        elif enc == "mono8":
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w)
            out_mode = "L"
        else:
            return None
    else:  # depth
        if enc in ("32fc1", "32FC1"):
            raw = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(h, w)
        elif enc in ("16uc1", "16UC1"):
            # mm → m for display; the rendering normalises anyway.
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(h, w).astype(np.float32) / 1000.0
        else:
            return None
        # Per-frame normalise: clip to [near, far_p99] so a single
        # garbage pixel at 1e9 doesn't crush the dynamic range.
        finite = np.isfinite(raw) & (raw > 0)
        if not finite.any():
            arr = np.zeros((h, w), dtype=np.uint8)
        else:
            valid = raw[finite]
            near = float(np.maximum(valid.min(), 0.05))
            far = float(np.percentile(valid, 99))
            far = max(far, near + 0.1)
            norm = np.clip((raw - near) / (far - near), 0.0, 1.0)
            norm = np.where(finite, 1.0 - norm, 0.0)  # invert: nearer = brighter
            arr = (norm * 255).astype(np.uint8)
        out_mode = "L"

    if arr is None:
        return None
    buf = io.BytesIO()
    PILImage.fromarray(np.ascontiguousarray(arr), mode=out_mode).save(buf, format="PNG", optimize=False)
    stamp_unix = float(getattr(getattr(msg, "header", None), "stamp", None) and
                       (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) or 0.0)
    return {
        "width": w,
        "height": h,
        "encoding": enc,
        "stamp_ms": int(stamp_unix * 1000),
        "png_b64": base64.b64encode(buf.getvalue()).decode("ascii"),
    }


def _camera_channel_payload(hub: Any, kind: str) -> Optional[dict]:
    """Return one cached camera preview for the hub's latest channel message.

    The completed cache entry is replaced atomically so a worker-thread reader
    can observe either the old or new entry, never a partially updated key and
    payload. Per-app request single-flight is enforced by ``make_app``.
    """
    if not hub.has(kind):
        return None
    msg, stamp_unix, count = hub.latest(kind)
    if msg is None or count == 0:
        return None

    cache = _CAMERA_CACHE[kind]
    if cache["hub"] is hub and cache["count"] == count:
        return cache["payload"]

    payload = _image_to_png_b64(msg, kind=kind)
    if payload is not None and payload["stamp_ms"] == 0:
        # Prefer the ROS header stamp; fall back to ingest-time when unset.
        payload["stamp_ms"] = int(stamp_unix * 1000)
    _CAMERA_CACHE[kind] = {
        "hub": hub,
        "count": count,
        "payload": payload,
    }
    return payload


def _camera_payload(hub: Any) -> dict:
    """JSON for the /cam panel: latest RGB + depth as cached PNGs."""
    out: dict[str, Any] = {"rgb": None, "depth": None}
    if hub is None:
        return out
    out["rgb"] = _camera_channel_payload(hub, "rgb")
    out["depth"] = _camera_channel_payload(hub, "depth")
    return out


def _camera_json_bytes(hub: Any) -> bytes:
    """Encode one camera response completely inside the preview worker."""
    return json.dumps(
        _camera_payload(hub),
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    ).encode("utf-8")


def _state_payload(registry: ObjectRegistry,
                   hub: Any, sg_store: Any = None,
                   anno_store: Any = None,
                   map_binding: Optional[dict] = None) -> dict:
    """Serialise the registry + relations + map into the small JSON
    shape the page consumes. Done in one snapshot so the page never
    sees a half-updated registry. The "relations" field shows the fast
    geometric slice (``reachable_by`` only, under the VLM-primary graph);
    the "scene_graph" field shows the full composed graph (geometric +
    image-grounded relational/semantic edges). "annotations" carries the
    user-drawn rooms/POIs (map-frame meters, same coordinates as objects)
    and "map_binding" the identity scene is bound to — both consumed by
    the /user annotation page."""
    objs_dict, _surfaces = _sync_snapshot(registry)
    geo_edges = sg_store.get_geometric_edges() if sg_store is not None else []
    out_objects: list[dict[str, Any]] = []
    robot_pose: Optional[dict[str, float]] = None
    for o in objs_dict.values():
        out_objects.append({
            "id": o.object_id,
            "short_id": _shorten_id(o.object_id),
            "cls": o.cls,
            "pose": {"x": o.pose.x, "y": o.pose.y, "z": o.pose.z, "yaw": o.pose.yaw},
            "bbox": {
                "size_x": o.bbox.size_x, "size_y": o.bbox.size_y, "size_z": o.bbox.size_z,
                "yaw": o.bbox.yaw,
            },
            "confidence": o.confidence,
            "observation_count": o.observation_count,
            "missing": o.missing,
        })
        if o.attributes.get("is_robot"):
            robot_pose = {"x": o.pose.x, "y": o.pose.y, "z": o.pose.z, "yaw": o.pose.yaw}
    out_relations = [
        {"subject": e.source_id, "predicate": e.relation, "target": e.target_id}
        for e in geo_edges
    ]

    # Scene graph edges from the LLM-enhanced layer (if enabled).
    sg_payload: dict[str, Any] = {"edges": [], "updated_at": 0.0}
    if sg_store is not None:
        snap = sg_store.get_snapshot()
        if snap is not None:
            sg_payload = {
                "edges": [
                    {
                        "source_id": e.source_id,
                        "target_id": e.target_id,
                        "relation": e.relation,
                        "confidence": round(e.confidence, 2),
                    }
                    for e in snap.edges
                ],
                "updated_at": snap.updated_at,
            }

    return {
        "objects": out_objects,
        "relations": out_relations,
        "scene_graph": sg_payload,
        "robot": robot_pose,
        "occupancy": _occupancy_payload(hub),
        "annotations": anno_store.list_json() if anno_store is not None else [],
        "map_binding": map_binding,
        "stamp_unix": time.time(),
    }


def _sync_snapshot(registry: ObjectRegistry):
    """Lock-protected sync read. The web handler runs in the asyncio
    loop, but we re-enter the registry's lock by hand using a sync
    wrapper so we don't accidentally `await` inside Starlette's sync
    response path. ObjectRegistry doesn't expose this; we replicate
    the dict copies here."""
    # registry._lock is asyncio.Lock; if we tried to .acquire() in a
    # sync context we'd deadlock. Pragmatic compromise: read the
    # internal dicts atomically (Python dict.copy() is atomic
    # bytecode-wise for our usage). Misses the lock but the worst
    # consequence is one frame seeing a half-updated registry, which
    # the next 200ms tick fixes. Good enough for a debug UI.
    return dict(registry._objects), dict(registry._surfaces)  # noqa: SLF001


_MAP_RPC = {
    "list_maps": (
        "robonix/service/map/list_maps",
        "RobonixServiceMapListMapsStub",
        "ListMaps",
        "ListMaps_Request",
    ),
    "save_map": (
        "robonix/service/map/save_map",
        "RobonixServiceMapSaveMapStub",
        "SaveMap",
        "SaveMap_Request",
    ),
    "load_map": (
        "robonix/service/map/load_map",
        "RobonixServiceMapLoadMapStub",
        "LoadMap",
        "LoadMap_Request",
    ),
    "delete_map": (
        "robonix/service/map/delete_map",
        "RobonixServiceMapDeleteMapStub",
        "DeleteMap",
        "DeleteMap_Request",
    ),
    "pose_estimate": (
        "robonix/service/map/pose_estimate",
        "RobonixServiceMapPoseEstimateStub",
        "PoseEstimate",
        "PoseEstimate_Request",
    ),
    "get_pose": (
        "robonix/service/map/get_pose",
        "RobonixServiceMapGetPoseStub",
        "GetPose",
        "GetPose_Request",
    ),
    "get_mode": (
        "robonix/service/map/get_mode",
        "RobonixServiceMapGetModeStub",
        "GetMode",
        "GetMode_Request",
    ),
}


def _pb_to_dict(resp: Any) -> dict:
    out: dict[str, Any] = {}
    for field, value in resp.ListFields():
        out[field.name] = value
    return out


def _map_rpc(op: str, payload: Optional[dict] = None, timeout_s: float = 20.0) -> dict:
    """Call mapping through the standard robonix/service/map capability.

    The user page talks only to scene HTTP. Scene then resolves mapping via
    Atlas + gRPC so this does not depend on mapping's debug Web UI or shared
    container paths.
    """
    import grpc  # lazy: web debug imports should not fail without grpc
    import map_pb2  # type: ignore
    import robonix_contracts_pb2_grpc as contracts_grpc  # type: ignore

    contract_id, stub_name, method_name, req_name = _MAP_RPC[op]
    caps = ATLAS.find_capability(contract_id=contract_id, transport="grpc")
    if not caps:
        return {"ok": False, "detail": f"no provider for {contract_id}"}
    cap = caps[0]
    channel_ref = ATLAS.connect_capability(
        consumer_id="scene",
        provider_id=cap.provider_id,
        contract_id=contract_id,
        transport="grpc",
    )
    try:
        endpoint = (channel_ref.endpoint or "").strip()
        if not endpoint:
            return {"ok": False, "detail": f"{contract_id} resolved to an empty endpoint"}
        with grpc.insecure_channel(endpoint) as grpc_channel:
            stub = getattr(contracts_grpc, stub_name)(grpc_channel)
            req = getattr(map_pb2, req_name)(**(payload or {}))
            resp = getattr(stub, method_name)(req, timeout=timeout_s)
        return _pb_to_dict(resp)
    except grpc.RpcError as e:
        return {"ok": False, "detail": f"{contract_id} rpc failed: {e.code().name}: {e.details()}"}
    except Exception as e:  # noqa: BLE001
        log.exception("scene map rpc %s failed", contract_id)
        return {"ok": False, "detail": str(e)}
    finally:
        channel_ref.close()


def _maps_payload() -> dict:
    out = _map_rpc("list_maps", {})
    maps = []
    if out.get("ok"):
        try:
            maps = json.loads(str(out.get("maps_json") or "[]"))
            if not isinstance(maps, list):
                maps = []
        except Exception as e:  # noqa: BLE001
            out = {"ok": False, "detail": f"invalid maps_json from mapping: {e}"}
    return {"ok": bool(out.get("ok")), "detail": out.get("detail", ""), "maps": maps}


def make_app(*, registry: ObjectRegistry,
             hub: Any = None, detector: Any = None,
             sg_store: Any = None, anno_store: Any = None,
             object_store: Any = None, map_meta: Any = None,
             map_binding: Optional[dict] = None,
             ops_lock: Optional[asyncio.Lock] = None,
             semantic_hold: Optional[dict] = None) -> Starlette:
    """Build the Starlette ASGI app the entrypoint mounts on its own
    uvicorn server.

    Routes:
      GET /                — combined split layout (2D map · 3D · cam)
      GET /2d              — 2D top-down map (occupancy grid + objects)
      GET /3d              — 3D scene (point clouds + bbox; three.js)
      GET /cam             — camera stack (live RGB + depth)
      GET /user            — end-user map page (rooms: draw / rename /
                             confirm-stale / delete; light object overlay)
      GET /api/state       — JSON for the 2D map
      GET /api/objects3d   — JSON for the 3D viz (per-object pcd + bbox)
      GET /api/camera      — JSON: latest RGB + depth frames
      /api/annotations[..] — user annotation CRUD (see below)
      /api/maps[..]        — scene-owned map library façade over map capabilities

    `hub` is the SubscribersHub — passed so the JSON state can include
    the latest OccupancyGrid for the 2D canvas underlay.
    `detector` is the ConceptGraphsDetector — passed so the 3D endpoint
    can serialize its persistent MapObjectList. If None, the 3D page
    just shows an empty world.
    `anno_store` is the AnnotationStore backing the annotation CRUD; when
    None (store init failed / disabled) those routes answer 503.
    `object_store` is the scene-object snapshot store. Save writes the live
    registry into a fresh partition token; Load restores the token the
    sidecar (`map_meta`) points at. Neither touches the store's own binding.
    `map_meta` is the MapMetaStore sidecar pairing each saved map with its
    object partition; when None, Save/Load degrade to annotations-only.
    `map_binding` is a mutable {map_id, mode, generation, source} dict shown
    by the /user page header, updated by Save/Load actions and by the
    lifecycle watcher (service.py) on a runtime epoch bump.
    `ops_lock` serializes Save/Load/Delete with each other AND with the
    watcher's epoch response (service.py shares the same lock): the handlers
    suspend at RPC awaits mid-critical-section, and an interleaved flush or
    second Save would mix sessions/epochs in one snapshot.

    Annotation API contract (STABLE once shipped — any frontend builds on
    it; see system/scene/README.md):
      GET    /api/annotations       → {ok, annotations: [...]}
      POST   /api/annotations       body {kind, name, points, theta?}
                                    → {ok, annotation}
      PUT    /api/annotations/{id}  body: any of {name, points, theta,
                                    stale:false} → {ok, annotation}
      DELETE /api/annotations/{id}  → {ok}
    theta (heading, radians) is poi-only — a room carrying it is a 400.
    On PUT, theta null/absent means "keep"; a set heading cannot be
    cleared, only changed (deliberate until the poi UI exists).
    Errors: 400 invalid body/fields, 404 unknown id, 503 store unavailable;
    those carry {ok: false, detail}. A store write failure (disk full)
    deliberately escapes as a plain 500 — the edit was NOT saved and hiding
    that behind a tidy body would be worse. Coordinates are map-frame
    meters. Same trust domain as the rest of this LAN debug/UI server —
    no auth.
    """
    if map_binding is None:
        map_binding = {}
    if ops_lock is None:
        ops_lock = asyncio.Lock()
    # `semantic_hold["reason"]` is non-None while scene's semantic state
    # may not match the map mapping is running: a Load is in flight, its
    # last attempt did not complete, or (set by the lifecycle watcher,
    # which is why the dict can be passed in and shared) mapping switched
    # maps outside the facade. Save reads it under `ops_lock`:
    # snapshotting in that divergent window would commit the wrong
    # (possibly just-flushed, empty) registry as the map's snapshot and
    # move the sidecar off the last valid one. In-memory only — a scene
    # restart re-binds from scratch, which is its own convergence.
    if semantic_hold is None:
        semantic_hold = {"reason": None}

    # Camera PNG encoding is CPU-heavy for 1920x1080 frames. Keep this state
    # inside one ASGI app so concurrent `/api/camera` requests share a single
    # worker and a recently completed preview without blocking Scene's event
    # loop or evicting another app's response cache.
    camera_preview_lock = asyncio.Lock()
    camera_preview_hub: Any = None
    camera_preview_bytes: Optional[bytes] = None
    camera_preview_completed_s = 0.0

    async def _persist_scene_objects(partition: str) -> tuple[int, int]:
        """Snapshot the registry into `partition` — EVERY object, including
        `missing` ones (known but not currently observed): after a Load the
        whole restored set is `missing` until re-observed, and filtering it
        out would commit an empty snapshot over the previous one. Anything
        in the registry is anchored to the current frame (the watcher and
        Load flush on epoch changes), so all of it belongs in the snapshot.
        The ONE exclusion is the self-object (`is_robot`, same predicate as
        the builder's persist gate): the pose tracker re-creates it from the
        live pose stream every session, so a restored copy would just sit as
        a second, frozen robot marker at wherever the robot stood at Save.
        Returns (written, expected); written < expected means the milvus
        upsert failed and the snapshot is incomplete — the caller must NOT
        commit the sidecar to it. Never rebinds the shared store."""
        if object_store is None:
            return 0, 0
        objs, _surfs = await registry.snapshot()
        pairs = [
            (o, None) for o in objs.values()
            if not o.attributes.get("is_robot", False)
        ]
        if not pairs:
            return 0, 0
        written = int(await asyncio.to_thread(
            object_store.persist, pairs, partition=partition
        ))
        return written, len(pairs)

    async def _restore_scene_objects(partition: str) -> int:
        """Restore one snapshot partition into the live registry. Raises on
        a DB read error (strict) — the facade must distinguish "snapshot is
        empty" from "could not read the snapshot", or a transient error
        followed by a Save would silently commit an empty snapshot.
        Never rebinds the shared store."""
        if object_store is None:
            return 0
        restored = await asyncio.to_thread(
            object_store.load_all, partition=partition, strict=True
        )
        async with registry.lock():
            for o in restored:
                registry.restore_object(o)
        return len(restored)

    def _latched_lifecycle(expected_map_id: str) -> tuple[Optional[int], str]:
        """(generation, mode) from mapping's latched lifecycle broadcast, or
        (None, "") when mapping doesn't broadcast (upstream main), no sample
        arrived yet, or the sample names a DIFFERENT map — a latch lagging
        behind a Save/Load must not stamp the wrong map's epoch into the
        sidecar / live binding (the watcher would read the real broadcast as
        a bump and flush). Enrichment only — the epoch machinery must work
        without it."""
        if hub is None or not hub.has("map_lifecycle"):
            return None, ""
        try:
            msg, _stamp, count = hub.latest("map_lifecycle")
            if msg is None or count == 0:
                return None, ""
            if str(msg.map_id) != _sanitize_map_id(expected_map_id):
                return None, ""
            return int(msg.generation), str(msg.mode)
        except Exception:  # noqa: BLE001
            # Malformed/foreign sample — enrichment only, never a failure.
            return None, ""

    _RESERVED_MAP_ID = re.compile(r"^\.|__s\d+$")

    def _reserved_map_id_error(map_id: str) -> Optional[JSONResponse]:
        """400 for map ids colliding with scene-internal namespaces: leading
        `.` (live-session state, purged at boot) and the `__s<N>` snapshot
        suffix (a map named `foo__s1` would alias — and Save would purge —
        map `foo`'s committed snapshot partition)."""
        if _RESERVED_MAP_ID.search(_sanitize_map_id(map_id)):
            return _anno_error(400, (
                f"map_id {map_id!r} uses a reserved scene-internal form "
                "(leading '.' or '__s<N>' suffix) — pick another name"
            ))
        return None

    def _set_map_binding(map_id: str, mode: str, source: str,
                         generation: Optional[int] = None) -> None:
        map_binding.update({
            "map_id": map_id,
            "mode": mode,
            "generation": generation,
            "source": source,
        })

    async def index(_request) -> HTMLResponse:
        # Combined split layout: 2D map left, 3D viz right, each with
        # an expand-button that maximises the panel inside the page
        # (NOT browser-fullscreen). The two iframes embed the original
        # standalone /2d and /3d routes so they remain individually
        # bookmarkable / debuggable.
        return HTMLResponse(_COMBINED_HTML)

    async def index2d(_request) -> HTMLResponse:
        return HTMLResponse(_INDEX_HTML)

    async def state(_request) -> JSONResponse:
        return JSONResponse(
            _state_payload(registry, hub, sg_store, anno_store, map_binding)
        )

    # ── annotation CRUD ──────────────────────────────────────────────
    def _anno_error(status: int, detail: str) -> JSONResponse:
        return JSONResponse({"ok": False, "detail": detail}, status_code=status)

    async def _anno_body(request) -> Optional[dict]:
        """Parse the JSON request body; None (→ caller answers 400) when
        it is missing, malformed, or not an object."""
        try:
            body = await request.json()
        except Exception:  # noqa: BLE001 — malformed body is a client error
            return None
        return body if isinstance(body, dict) else None

    async def annotations_list(_request) -> JSONResponse:
        if anno_store is None:
            return _anno_error(503, "annotation store unavailable")
        return JSONResponse({"ok": True, "annotations": anno_store.list_json()})

    async def annotations_create(request) -> JSONResponse:
        """POST /api/annotations — validate {kind, name, points, theta?}
        and persist a new annotation (returned with its generated id)."""
        if anno_store is None:
            return _anno_error(503, "annotation store unavailable")
        body = await _anno_body(request)
        if body is None:
            return _anno_error(400, "request body must be a JSON object")
        kind = body.get("kind")
        name = body.get("name", "")
        points = body.get("points")
        theta = body.get("theta")
        err = validate_annotation_fields(kind, name, points, theta)
        if err:
            return _anno_error(400, err)
        ann = anno_store.create(kind=kind, name=name, points=points, theta=theta)
        return JSONResponse({"ok": True, "annotation": ann.to_json()})

    async def annotations_update(request) -> JSONResponse:
        """PUT /api/annotations/{id} — partial update: any of name /
        points / theta / stale:false (the user's "confirm still valid").
        Provided fields are validated against the annotation's kind."""
        if anno_store is None:
            return _anno_error(503, "annotation store unavailable")
        ann_id = request.path_params["annotation_id"]
        existing = anno_store.get(ann_id)
        if existing is None:
            return _anno_error(404, f"unknown annotation id {ann_id!r}")
        body = await _anno_body(request)
        if body is None:
            return _anno_error(400, "request body must be a JSON object")
        name = body.get("name")
        points = body.get("points")
        theta = body.get("theta")
        # Validate the would-be merged state so a partial update can never
        # store something a create would have rejected.
        err = validate_annotation_fields(
            existing.kind,
            name if name is not None else existing.name,
            points if points is not None else existing.points,
            theta if theta is not None else existing.theta,
        )
        if err:
            return _anno_error(400, err)
        clear_stale = body.get("stale") is False
        ann = anno_store.update(
            ann_id, name=name, points=points, theta=theta,
            clear_stale=clear_stale,
        )
        if ann is None:  # deleted between get and update — still a 404
            return _anno_error(404, f"unknown annotation id {ann_id!r}")
        return JSONResponse({"ok": True, "annotation": ann.to_json()})

    async def annotations_delete(request) -> JSONResponse:
        """DELETE /api/annotations/{id} — remove the annotation; 404 when
        the id is unknown (delete is the user's explicit action, so unlike
        staleness it IS allowed to drop a user asset)."""
        if anno_store is None:
            return _anno_error(503, "annotation store unavailable")
        ann_id = request.path_params["annotation_id"]
        if not anno_store.delete(ann_id):
            return _anno_error(404, f"unknown annotation id {ann_id!r}")
        return JSONResponse({"ok": True})

    async def maps_list(_request) -> JSONResponse:
        return JSONResponse(await asyncio.to_thread(_maps_payload))

    def _map_exists_in_payload(payload: dict, map_id: str) -> tuple[bool, dict]:
        for item in payload.get("maps", []):
            if item.get("map_id") == map_id and item.get("has_spatial_artifact"):
                return True, item
        return False, {}

    async def maps_save(request) -> JSONResponse:
        body = await _anno_body(request)
        if body is None:
            return _anno_error(400, "request body must be a JSON object")
        map_id = str(body.get("map_id") or "").strip()
        if not map_id:
            return _anno_error(400, "map_id is required")
        reserved = _reserved_map_id_error(map_id)
        if reserved is not None:
            return reserved
        async with ops_lock:
            return await _maps_save_locked(map_id, body)

    async def _maps_save_locked(map_id: str, body: dict) -> JSONResponse:
        """Save body — runs under `ops_lock` (see maps_save)."""
        if semantic_hold["reason"]:
            return _anno_error(409, (
                f"save blocked: {semantic_hold['reason']} — Load a map "
                "(any map) until it reports success, then Save"
            ))
        note = str(body.get("note") or "")
        before_payload = await asyncio.to_thread(_maps_payload)
        spatial_exists, existing_map = _map_exists_in_payload(before_payload, map_id)
        current_bound_id = str(map_binding.get("map_id") or "")
        current_mode = str(map_binding.get("mode") or "")
        if (anno_store is not None
                and _sanitize_map_id(map_id) != anno_store.map_id
                and anno_store.has_saved(map_id)):
            # The store never loaded this map's annotation file (it is bound
            # to another partition — typically the live session). Carrying
            # the live partition over the file would silently destroy
            # previously saved rooms; a startup binding that happens to name
            # this map (lifecycle broadcast / env) is NOT a load.
            return _anno_error(409, (
                f"map {map_id} already has saved room annotations; load it "
                "first (or delete the map) instead of overwriting them with "
                "this live session"
            ))
        if spatial_exists:
            if current_bound_id and current_bound_id != map_id:
                return _anno_error(409, f"spatial map {map_id} already exists; load it before updating scene annotations/objects")
            if current_mode == "mapping":
                # The artifact was snapshotted at the original Save while the
                # live frame kept evolving (loop closures) — writing today's
                # coordinates against that frozen artifact would mis-anchor
                # every object/room on the next Load.
                return _anno_error(409, (
                    f"map {map_id} was saved from this still-running mapping "
                    "session; its spatial artifact is immutable and the live "
                    "frame has kept drifting since. Load it in localization "
                    "mode to edit rooms/objects, or delete it and save anew."
                ))
            out = {
                "ok": True,
                "map_id": map_id,
                "detail": f"spatial map {map_id} already exists; saved scene annotations/objects only",
                "spatial_unchanged": True,
            }
        else:
            save_timeout_s = float(os.environ.get("SCENE_MAP_SAVE_TIMEOUT_S", "300"))
            out = await asyncio.to_thread(
                _map_rpc,
                "save_map",
                {"map_id": map_id, "note": note},
                save_timeout_s,
            )
        object_count = 0
        object_persist_error = None
        if out.get("ok"):
            gen, broadcast_mode = _latched_lifecycle(map_id)
            if anno_store is not None:
                anno_store.rebind(map_id, generation=gen, carry_current=True)
            # Object snapshot: write into a FRESH partition token, commit the
            # sidecar only once the write is verifiably complete, and only
            # then purge the previous snapshot (plus any legacy rows under
            # the bare map id — they belong to frames that no longer exist).
            # A failure at any point leaves the sidecar at the previous,
            # still-consistent snapshot.
            if map_meta is not None and object_store is not None:
                prev_meta = map_meta.read(map_id)
                partition, seq = map_meta.next_partition(map_id)
                try:
                    # The token repeats after a FAILED save attempt (the
                    # sidecar only advances on commit) — clear its debris so
                    # the snapshot holds exactly this attempt's rows.
                    await asyncio.to_thread(object_store.delete_map, partition)
                    written, expected = await _persist_scene_objects(partition)
                    if written < expected:
                        raise RuntimeError(
                            f"snapshot incomplete: {written}/{expected} rows written"
                        )
                    object_count = written
                    map_meta.write(make_meta(
                        map_id, partition, seq,
                        generation=gen, mode=broadcast_mode,
                    ))
                    out["objects"] = object_count
                    out["snapshot_partition"] = partition
                    stale_parts = {_sanitize_map_id(map_id)}
                    if prev_meta is not None:
                        stale_parts.add(prev_meta.object_partition)
                    stale_parts.discard(partition)
                    for part in stale_parts:
                        try:
                            await asyncio.to_thread(object_store.delete_map, part)
                        except Exception as e:  # noqa: BLE001
                            # Orphan rows cost disk, not correctness — the
                            # sidecar no longer points at them.
                            out["stale_partition_cleanup_error"] = str(e)
                except Exception as e:  # noqa: BLE001
                    object_persist_error = str(e)
                    out["object_persist_error"] = object_persist_error
            elif object_store is not None:
                object_persist_error = (
                    "map_meta sidecar unavailable — objects not snapshotted"
                )
                out["object_persist_error"] = object_persist_error
            mode_after_save = current_mode if spatial_exists and current_bound_id == map_id and current_mode else "mapping"
            _set_map_binding(map_id, mode_after_save, "ui_save", generation=gen)
            if object_persist_error is not None:
                # The public Save contract is geometry + rooms + objects as
                # ONE unit. With the object snapshot uncommitted the artifact
                # is incomplete, and a 200/ok here would let the operator
                # move on without the recovery step — the sidecar still
                # points at the previous snapshot (spatial geometry, when it
                # was written above, does remain on disk). The recovery text
                # must match what the guards above will actually allow: after
                # a FRESH mapping-session save the artifact is already
                # immutable, so "retry Save" would only bounce off the
                # mapping-mode 409 — deleting and re-saving is the real path.
                recovery = (
                    "retry Save" if mode_after_save != "mapping" else (
                        "delete the map and Save anew (this mapping "
                        "session's artifact is immutable, so a plain retry "
                        "would be refused)"
                    )
                )
                out["ok"] = False
                out["partial"] = "spatial_saved_object_snapshot_failed"
                out["detail"] = (
                    f"{out.get('detail') or 'saved'}; OBJECT SNAPSHOT FAILED: "
                    f"{object_persist_error} — the map's semantic snapshot "
                    f"was NOT updated; {recovery}"
                )

        annotations = anno_store.list_json() if anno_store is not None else []
        out.setdefault("annotations", len(annotations))
        if out.get("ok"):
            maps_payload = await asyncio.to_thread(_maps_payload)
            saved_map = None
            for item in maps_payload.get("maps", []):
                if item.get("map_id") == map_id:
                    saved_map = item
                    break
            validation = {
                "map_id": map_id,
                "spatial_ok": bool(saved_map and saved_map.get("has_spatial_artifact") and saved_map.get("spatial_ok") is not False),
                "artifact_detail": (saved_map or {}).get("artifact_detail") or maps_payload.get("detail") or "map not found after save",
                "artifact_size": (saved_map or {}).get("artifact_size"),
                "has_preview": bool(saved_map and saved_map.get("has_preview")),
                "updated": (saved_map or {}).get("updated"),
                "object_count": int(object_count or 0),
                "room_count": sum(1 for a in annotations if a.get("kind") == "room"),
                "annotation_count": len(annotations),
                "paths": {
                    "map_artifact": (saved_map or {}).get("artifact_path") or "not exposed by map provider",
                    "preview": (saved_map or {}).get("preview_path") or "not exposed by map provider",
                    "scene_annotations": str(anno_store.path) if anno_store is not None else "annotation store unavailable",
                },
                "log": [
                    f"save_map returned ok={bool(out.get('ok'))}: {out.get('detail') or ''}",
                    f"map list validation ok={bool(maps_payload.get('ok'))}",
                    f"spatial artifact check: {(saved_map or {}).get('artifact_detail') or maps_payload.get('detail') or 'unknown'}",
                ],
            }
            if object_persist_error:
                validation["log"].append(f"object persistence error: {object_persist_error}")
            out["validation"] = validation
        return JSONResponse(out, status_code=200 if out.get("ok") else 502)

    async def maps_load(request) -> JSONResponse:
        body = await _anno_body(request)
        if body is None:
            return _anno_error(400, "request body must be a JSON object")
        map_id = str(body.get("map_id") or "").strip()
        if not map_id:
            return _anno_error(400, "map_id is required")
        async with ops_lock:
            return await _maps_load_locked(map_id, body)

    async def _maps_load_locked(map_id: str, body: dict) -> JSONResponse:
        """Load body — runs under `ops_lock` (see maps_save)."""
        mode = str(body.get("mode") or "localization")
        load_timeout_s = float(os.environ.get("SCENE_MAP_LOAD_TIMEOUT_S", "240"))
        before_stamp = 0.0
        before_count = 0
        if hub is not None and hub.has("occupancy_grid"):
            _before_msg, before_stamp, before_count = hub.latest("occupancy_grid")
        out = await asyncio.to_thread(_map_rpc, "load_map", {
            "map_id": map_id,
            "mode": mode,
            "has_initial_pose": bool(body.get("has_initial_pose", False)),
            "x": float(body.get("x") or 0.0),
            "y": float(body.get("y") or 0.0),
            "theta": float(body.get("theta") or 0.0),
        }, load_timeout_s)
        if out.get("ok"):
            # Mapping now runs the target DB while scene's semantic state
            # still belongs to the previous map — from here until every
            # rebind below commits, a Save must not snapshot. The hold is
            # cleared only on full success; every early return leaves it
            # set, so an operator's next Save gets a 409 pointing back at
            # the Load retry.
            semantic_hold["reason"] = f"the last load of {map_id} did not complete"
            # Mapping's LoadMap response means RTAB-Map accepted the database
            # and PublishMap request. Do not restore semantic state until Scene
            # has actually observed the resulting occupancy grid; otherwise the
            # first click only switches the database and a second click appears
            # necessary to refresh rooms/objects against the loaded map.
            ready_timeout_s = float(os.environ.get("SCENE_MAP_READY_TIMEOUT_S", "20"))
            deadline = time.monotonic() + ready_timeout_s
            occupancy_ready = False
            while time.monotonic() < deadline:
                if hub is not None and hub.has("occupancy_grid"):
                    msg, stamp, count = hub.latest("occupancy_grid")
                    if (
                        msg is not None
                        and int(getattr(msg.info, "width", 0)) > 0
                        and int(getattr(msg.info, "height", 0)) > 0
                        and (count > before_count or stamp > before_stamp)
                    ):
                        occupancy_ready = True
                        out["occupancy"] = {
                            "count": int(count),
                            "stamp_unix": float(stamp),
                            "width": int(msg.info.width),
                            "height": int(msg.info.height),
                        }
                        break
                await asyncio.sleep(0.1)
            if not occupancy_ready:
                semantic_hold["reason"] = (
                    f"the last load of {map_id} failed before scene "
                    "observed its occupancy grid"
                )
                out = {
                    **out,
                    "ok": False,
                    "detail": (
                        f"mapping loaded {map_id}, but Scene did not observe a new "
                        f"occupancy grid within {ready_timeout_s:.1f}s"
                    ),
                }
                return JSONResponse(out, status_code=502)
            # The loaded artifact defines a NEW frame: everything observed
            # before this load is anchored to the dead pre-load frame — and
            # the watcher will not catch it (a facade load is deliberately
            # not drift). Flush first, so the registry ends up holding the
            # restored snapshot plus post-load observations only. A flush
            # failure aborts the load HERE — rebinding annotations/objects
            # over a registry that may still hold pre-load objects would
            # recreate the mixed-epoch state this ordering exists to
            # prevent.
            try:
                registry_lock = getattr(registry, "lock", None)
                if callable(registry_lock):
                    async with registry_lock():
                        flushed = registry.clear_objects()
                else:
                    # Minimal registry adapters used by embedded callers may
                    # already serialize access and expose only clear_objects.
                    flushed = registry.clear_objects()
                if flushed:
                    out["objects_flushed"] = flushed
            except Exception as e:  # noqa: BLE001
                semantic_hold["reason"] = (
                    f"the last load of {map_id} failed flushing "
                    "pre-load objects"
                )
                out = {
                    **out,
                    "ok": False,
                    "object_flush_error": str(e),
                    "detail": (
                        f"{out.get('detail') or 'loaded'}; REGISTRY FLUSH "
                        f"FAILED: {e} — aborted before rooms/objects were "
                        "rebound; retry the load"
                    ),
                }
                return JSONResponse(out, status_code=502)
            gen, _broadcast_mode = _latched_lifecycle(map_id)
            meta = map_meta.read(map_id) if map_meta is not None else None
            if meta is not None:
                # Restore BEFORE rebinding annotations or committing the
                # scene binding: on failure the previous binding (and the
                # previous map's annotation file) stay in place, and the
                # semantic hold above keeps Save from publishing the
                # flushed (possibly empty) registry as a fresh snapshot
                # over the last valid one.
                try:
                    out["objects_restored"] = await _restore_scene_objects(
                        meta.object_partition
                    )
                    out["snapshot_partition"] = meta.object_partition
                except Exception as e:  # noqa: BLE001
                    semantic_hold["reason"] = (
                        f"the last load of {map_id} failed restoring its "
                        "object snapshot"
                    )
                    out = {
                        **out,
                        "ok": False,
                        "object_restore_error": str(e),
                        "detail": (
                            f"{out.get('detail') or 'loaded'}; RESTORE "
                            f"FAILED: {e} — saved objects were NOT loaded "
                            "and the previous scene binding was kept; retry "
                            "the load before saving"
                        ),
                    }
                    return JSONResponse(out, status_code=502)
            if anno_store is not None:
                # Localization load keeps the saved map's frame epoch, so the
                # live broadcast (when present) carries the generation the
                # rooms were saved under; the sidecar's recorded value is the
                # fallback. Either lets rebind() re-judge staleness.
                anno_gen = gen if gen is not None else (
                    meta.mapping_generation if meta is not None else None
                )
                anno_store.rebind(map_id, generation=anno_gen, carry_current=False)
            if meta is None:
                # No sidecar → no snapshot known to match this artifact's
                # frame (pre-epoch save or foreign DB). Restoring the bare
                # partition would bring back rows from ANY earlier build of
                # this map — the off-map mis-anchored objects bug.
                out["objects_restored"] = 0
                out["semantic_snapshot"] = (
                    "absent — objects not restored (no epoch sidecar for "
                    "this map; re-save it to create one)"
                )
                # Surface it where the operator is looking (the map dialog
                # renders `detail`), not only in the log.
                out["detail"] = (
                    f"{out.get('detail') or 'loaded'}; no semantic snapshot "
                    "for this map — objects not restored (rooms still load; "
                    "re-save the map to snapshot objects)"
                )
                log.warning(
                    "[scene-maps] load %s: no semantic sidecar — restoring "
                    "no objects (rooms still load; re-save the map to "
                    "snapshot current objects)", map_id,
                )
            _set_map_binding(map_id, "localization", "ui_load", generation=gen)
            semantic_hold["reason"] = None
        return JSONResponse(out, status_code=200 if out.get("ok") else 502)

    async def maps_delete(request) -> JSONResponse:
        body = await _anno_body(request)
        if body is None:
            return _anno_error(400, "request body must be a JSON object")
        map_id = str(body.get("map_id") or "").strip()
        if not map_id:
            return _anno_error(400, "map_id is required")
        async with ops_lock:
            return await _maps_delete_locked(map_id)

    async def _maps_delete_locked(map_id: str) -> JSONResponse:
        """Delete body — runs under `ops_lock` (see maps_save)."""
        out = await asyncio.to_thread(_map_rpc, "delete_map", {"map_id": map_id})
        if out.get("ok"):
            annotations_deleted = False
            objects_deleted = 0
            cleanup_errors = []
            if anno_store is not None:
                try:
                    annotations_deleted = bool(anno_store.delete_map(map_id))
                except Exception as e:  # noqa: BLE001
                    cleanup_errors.append(f"annotations: {e}")
            meta = map_meta.read(map_id) if map_meta is not None else None
            snapshot_rows_orphaned = False
            if object_store is not None:
                # Both the sidecar-pointed snapshot partition and any legacy
                # rows under the bare map id belong to this map — remove all.
                parts = {_sanitize_map_id(map_id)}
                if meta is not None:
                    parts.add(meta.object_partition)
                for part in parts:
                    try:
                        objects_deleted += int(
                            await asyncio.to_thread(object_store.delete_map, part)
                        )
                    except Exception as e:  # noqa: BLE001
                        cleanup_errors.append(f"objects[{part}]: {e}")
                        if meta is not None and part == meta.object_partition:
                            snapshot_rows_orphaned = True
            if map_meta is not None:
                if snapshot_rows_orphaned:
                    # The sidecar is the only pointer to the rows that just
                    # failed to delete — keep it so a retried Delete still
                    # finds them (boot purge only matches `.live*`).
                    cleanup_errors.append(
                        "sidecar kept: snapshot rows not deleted — retry delete"
                    )
                else:
                    try:
                        map_meta.delete(map_id)
                    except Exception as e:  # noqa: BLE001
                        cleanup_errors.append(f"sidecar: {e}")
            out["scene_cleanup"] = {
                "annotations_deleted": annotations_deleted,
                "objects_deleted": objects_deleted,
            }
            if cleanup_errors:
                out["scene_cleanup_error"] = "; ".join(cleanup_errors)
        return JSONResponse(out, status_code=200 if out.get("ok") else 502)

    async def maps_pose_estimate(request) -> JSONResponse:
        body = await _anno_body(request)
        if body is None:
            return _anno_error(400, "request body must be a JSON object")
        try:
            payload = {
                "x": float(body.get("x")),
                "y": float(body.get("y")),
                "theta": float(body.get("theta") or 0.0),
                "cov_xy": float(body.get("cov_xy") or 0.0),
                "cov_theta": float(body.get("cov_theta") or 0.0),
            }
            if not all(math.isfinite(float(payload[k])) for k in ("x", "y", "theta", "cov_xy", "cov_theta")):
                raise ValueError("non-finite pose")
        except Exception:
            return _anno_error(400, "x/y/theta must be finite numbers")

        def _pose_delta(pose: dict) -> Optional[dict]:
            if not pose.get("ok"):
                return None
            dx = float(pose.get("x", 0.0)) - payload["x"]
            dy = float(pose.get("y", 0.0)) - payload["y"]
            dtheta = math.atan2(
                math.sin(float(pose.get("theta", 0.0)) - payload["theta"]),
                math.cos(float(pose.get("theta", 0.0)) - payload["theta"]),
            )
            return {
                "dx": dx,
                "dy": dy,
                "dtheta": dtheta,
                "distance_m": math.hypot(dx, dy),
                "abs_yaw_rad": abs(dtheta),
            }

        mode_info = await asyncio.to_thread(_map_rpc, "get_mode", {}, 3.0)
        before = await asyncio.to_thread(_map_rpc, "get_pose", {}, 3.0)
        out = await asyncio.to_thread(_map_rpc, "pose_estimate", payload)
        # RTAB-Map relocalization is not instantaneous. Give the pose adapter a
        # short beat so the UI can show an immediate convergence hint instead
        # of only "published".
        await asyncio.sleep(float(os.environ.get("SCENE_POSE_ESTIMATE_FEEDBACK_DELAY_S", "1.2")))
        after = await asyncio.to_thread(_map_rpc, "get_pose", {}, 3.0)
        out["requested_pose"] = payload
        out["mode"] = mode_info
        out["pose_before"] = before
        out["pose_after"] = after
        out["delta_before"] = _pose_delta(before)
        out["delta_after"] = _pose_delta(after)
        if out.get("ok"):
            if after.get("ok") and out["delta_after"]:
                d = out["delta_after"]
                mode = str(mode_info.get("mode") or "unknown") if mode_info.get("ok") else "unknown"
                suffix = "" if mode == "localization" else f"; mode={mode}, pose estimate may not relocalize until a map is loaded in localization mode"
                out["detail"] = (
                    f"pose estimate sent to ({payload['x']:.2f}, {payload['y']:.2f}, {payload['theta']:.2f}); "
                    f"current error {d['distance_m']:.2f} m, yaw {d['abs_yaw_rad']:.2f} rad"
                    f"{suffix}"
                )
            else:
                out["detail"] = (out.get("detail") or "pose estimate sent") + "; current pose not available yet"
        return JSONResponse(out, status_code=200 if out.get("ok") else 502)

    async def index3d(_request) -> HTMLResponse:
        return HTMLResponse(_INDEX_3D_HTML)

    async def objects3d(_request) -> JSONResponse:
        if detector is None or not hasattr(detector, "export_3d_snapshot"):
            return JSONResponse({"objects": [], "stamp_unix": 0.0})
        return JSONResponse(detector.export_3d_snapshot())

    async def cam(_request) -> HTMLResponse:
        return HTMLResponse(_INDEX_CAM_HTML)

    async def user_page(_request) -> HTMLResponse:
        return HTMLResponse(_USER_HTML)

    async def camera_state(_request) -> JSONResponse:
        """Return a rate-limited, single-flight preview off the event loop."""
        nonlocal camera_preview_hub
        nonlocal camera_preview_bytes
        nonlocal camera_preview_completed_s

        loop = asyncio.get_running_loop()
        now = loop.time()
        if (
            camera_preview_hub is hub
            and camera_preview_bytes is not None
            and now - camera_preview_completed_s < _CAMERA_PREVIEW_MIN_INTERVAL_S
        ):
            return Response(camera_preview_bytes, media_type="application/json")

        async with camera_preview_lock:
            now = loop.time()
            if (
                camera_preview_hub is hub
                and camera_preview_bytes is not None
                and now - camera_preview_completed_s
                < _CAMERA_PREVIEW_MIN_INTERVAL_S
            ):
                return Response(camera_preview_bytes, media_type="application/json")
            payload = await asyncio.to_thread(_camera_json_bytes, hub)
            camera_preview_hub = hub
            camera_preview_bytes = payload
            camera_preview_completed_s = loop.time()
            return Response(payload, media_type="application/json")

    # Static asset directory ships with scene_service; holds the
    # tiago URDF mesh assets (STL/DAE files lifted from PAL Robotics's
    # tiago_description + pmb2_description). Mounted under /static so
    # the 3D viz can fetch them via STLLoader without any extra wiring.
    static_dir = Path(__file__).parent / "static"

    routes = [
        Route("/", index, methods=["GET"]),
        Route("/2d", index2d, methods=["GET"]),
        Route("/3d", index3d, methods=["GET"]),
        Route("/cam", cam, methods=["GET"]),
        Route("/user", user_page, methods=["GET"]),
        Route("/api/state", state, methods=["GET"]),
        Route("/api/objects3d", objects3d, methods=["GET"]),
        Route("/api/camera", camera_state, methods=["GET"]),
        Route("/api/annotations", annotations_list, methods=["GET"]),
        Route("/api/annotations", annotations_create, methods=["POST"]),
        Route("/api/annotations/{annotation_id}", annotations_update,
              methods=["PUT"]),
        Route("/api/annotations/{annotation_id}", annotations_delete,
              methods=["DELETE"]),
        Route("/api/maps", maps_list, methods=["GET"]),
        Route("/api/maps/save", maps_save, methods=["POST"]),
        Route("/api/maps/load", maps_load, methods=["POST"]),
        Route("/api/maps/delete", maps_delete, methods=["POST"]),
        Route("/api/maps/pose_estimate", maps_pose_estimate, methods=["POST"]),
    ]
    if static_dir.is_dir():
        routes.append(Mount("/static", StaticFiles(directory=str(static_dir)), name="static"))
    return Starlette(routes=routes)


# ── Combined split layout (iframes for both panels) ─────────────────────────
# Two side-by-side iframes (2D left, 3D right) with a per-panel "⛶ expand"
# button. Click expand → that panel goes `position: fixed; inset: 0` and
# covers the page (in-page fullscreen, not the browser's F11). Click again
# to restore. URL hash (`#2d` / `#3d`) is updated so refresh preserves state.
_COMBINED_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>scene — robonix</title>
  <style>
    html, body { margin: 0; padding: 0; height: 100%;
                 background: #08090c; color: #d8dde6; overflow: hidden;
                 font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
    /* 3-column layout: 2D | 3D | cam (rgb stacked over depth). 2D and
       3D get the bulk of the width so they're usable on a 1440-wide
       laptop; cam is narrower since the camera frames aren't the
       headline view (you mostly check it when detections look wrong). */
    #grid { display: grid; grid-template-columns: 5fr 5fr 2fr;
            height: 100vh; gap: 1px; background: #1a1d24; }
    /* Windows-style chrome: titlebar is its OWN row at the top of the
       panel, NOT an overlay floating over the iframe. iframe gets the
       clean remainder. The previous absolute-positioned head + ⛶
       button always covered the iframe content beneath them no matter
       how small they were. */
    .panel { background: #08090c;
             min-width: 0; min-height: 0; overflow: hidden;
             display: flex; flex-direction: column; }
    .panel .titlebar {
      flex: 0 0 auto;
      display: flex; align-items: center; gap: 8px;
      padding: 0 6px 0 10px; height: 24px;
      background: #14171f; border-bottom: 1px solid #303542;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      font-size: 11px; line-height: 1; user-select: none;
    }
    .panel .titlebar .badge { color: #f0c050; font-weight: 600;
                              letter-spacing: 0.04em; }
    .panel .titlebar .desc { flex: 1; color: #6a6f7a;
                             white-space: nowrap; overflow: hidden;
                             text-overflow: ellipsis; }
    .panel button.expand {
      flex: 0 0 auto;
      width: 22px; height: 18px; padding: 0;
      background: transparent; color: #889;
      border: 1px solid #303542; border-radius: 3px;
      cursor: pointer; font-size: 11px; line-height: 1;
    }
    .panel button.expand:hover { color: #f0c050; border-color: #5a606e; }
    .panel iframe { flex: 1 1 auto;
                    width: 100%; min-height: 0;
                    border: 0; display: block; background: #08090c; }
    .panel.expanded {
      position: fixed; inset: 0; z-index: 99;
      grid-column: unset; grid-row: unset;
    }
    body.has-expanded #grid > .panel:not(.expanded) { display: none; }
    /* ── Floating info overlay ──
       imgui-style draggable panel. Sits in the top-left corner over
       the 2D map by default (small enough not to swallow the canvas).
       Click the header to collapse to a single bar; drag the header
       to move; click ✕ to dismiss for this session. State is
       remembered in localStorage so refresh keeps your layout. */
    #info-fp {
      position: fixed; top: 12px; left: 12px; z-index: 200;
      width: 320px; max-height: calc(100vh - 24px);
      background: rgba(14, 16, 21, 0.94);
      border: 1px solid #303542; border-radius: 6px;
      box-shadow: 0 4px 18px rgba(0, 0, 0, 0.55);
      display: flex; flex-direction: column;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      font-size: 12px; color: #d8dde6;
      backdrop-filter: blur(2px);
    }
    #info-fp.collapsed { max-height: 28px; }
    #info-fp.collapsed #info-body { display: none; }
    #info-fp.dismissed { display: none; }
    #info-head {
      display: flex; align-items: center; gap: 6px;
      padding: 5px 8px; cursor: move; user-select: none;
      border-bottom: 1px solid #2a2e38;
      font-size: 11px; color: #889;
    }
    #info-head .title { color: #f0c050; font-weight: 600;
                        letter-spacing: 0.04em; }
    #info-head .stamp { flex: 1; color: #6a6f7a; font-size: 10px;
                        white-space: nowrap; overflow: hidden;
                        text-overflow: ellipsis; }
    #info-head button {
      background: none; border: 1px solid #303542; color: #889;
      width: 22px; height: 20px; padding: 0; border-radius: 3px;
      cursor: pointer; font-size: 11px; line-height: 1;
    }
    #info-head button:hover { color: #f0c050; border-color: #5a606e; }
    #info-body { padding: 8px 10px 10px; overflow: auto; flex: 1; }
    #info-body h2 {
      margin: 8px 0 4px 0; font-size: 10px; font-weight: 600;
      text-transform: uppercase; letter-spacing: 0.06em; color: #6a6f7a;
    }
    #info-body h2:first-child { margin-top: 0; }
    #info-body .pose { color: #7aa7ff; }
    #info-body table { width: 100%; border-collapse: collapse;
                       font-size: 11px; }
    #info-body td { padding: 2px 4px; vertical-align: top;
                    border-bottom: 1px solid #1a1d24; }
    #info-body td.id { color: #7aa7ff; white-space: nowrap; }
    #info-body td.cls { color: #f0c674; white-space: nowrap; }
    #info-body td.pp { color: #6a6f7a; font-size: 10px; }
    #info-body td.miss { color: #555; }
    /* Relation list: one "<source> <predicate> <target>" row per edge,
       replacing the old on-canvas dashed lines. */
    #info-rels .rel { display: flex; gap: 6px; align-items: baseline;
                      padding: 2px 4px; border-bottom: 1px solid #1a1d24;
                      font-size: 11px; white-space: nowrap;
                      overflow: hidden; text-overflow: ellipsis; }
    #info-rels .rs { color: #7aa7ff; }
    #info-rels .rp { color: #f0c050; font-weight: 600; }
    #info-rels .rt { color: #f0c674; }
    /* "Show info" pill that appears once the panel is dismissed. */
    #info-show {
      position: fixed; top: 12px; left: 12px; z-index: 200;
      padding: 4px 10px; font-size: 11px;
      background: rgba(14, 16, 21, 0.94); border: 1px solid #303542;
      border-radius: 4px; color: #889; cursor: pointer;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      display: none;
    }
    #info-show:hover { color: #f0c050; border-color: #5a606e; }
    body.info-dismissed #info-show { display: block; }
  </style>
</head>
<body>
  <div id="grid">
    <div class="panel" id="panel-2d">
      <div class="titlebar">
        <span class="badge">2D</span>
        <span class="desc">map · occupancy grid + tracked objects · 5 Hz</span>
        <button class="expand" title="expand">⛶</button>
      </div>
      <iframe src="/2d" loading="eager"></iframe>
    </div>
    <div class="panel" id="panel-3d">
      <div class="titlebar">
        <span class="badge">3D</span>
        <span class="desc">ConceptGraphs · drag rotate · WASD fly · click pick</span>
        <button class="expand" title="expand">⛶</button>
      </div>
      <iframe src="/3d" loading="eager"></iframe>
    </div>
    <div class="panel" id="panel-cam">
      <div class="titlebar">
        <span class="badge">cam</span>
        <span class="desc">live RGB + depth · perception input</span>
        <button class="expand" title="expand">⛶</button>
      </div>
      <iframe src="/cam" loading="eager"></iframe>
    </div>
  </div>
  <div id="info-fp">
    <div id="info-head" title="drag to move; click title to collapse">
      <span class="title">scene</span>
      <span class="stamp" id="info-stamp">—</span>
      <button id="info-collapse" title="collapse / expand">_</button>
      <button id="info-dismiss" title="hide (click 'show info' to bring back)">×</button>
    </div>
    <div id="info-body">
      <h2>robot</h2>
      <div class="pose" id="info-pose">no fix yet</div>
      <h2>objects</h2>
      <table>
        <tbody id="info-objs"><tr><td colspan="3" style="color:#555">—</td></tr></tbody>
      </table>
      <h2>relations</h2>
      <div id="info-rels"><span style="color:#555">—</span></div>
    </div>
  </div>
  <button id="info-show" title="re-open the floating info panel">▸ show info</button>
  <script>
    function setExpanded(id) {
      document.querySelectorAll('.panel').forEach(p => p.classList.remove('expanded'));
      if (id) {
        const p = document.getElementById('panel-' + id);
        if (p) p.classList.add('expanded');
        document.body.classList.add('has-expanded');
        location.hash = '#' + id;
      } else {
        document.body.classList.remove('has-expanded');
        location.hash = '';
      }
    }
    document.querySelectorAll('button.expand').forEach(btn => {
      btn.addEventListener('click', e => {
        e.stopPropagation();
        const panel = btn.closest('.panel');
        const id = panel.id.replace('panel-', '');
        if (panel.classList.contains('expanded')) setExpanded(null);
        else setExpanded(id);
        btn.textContent = panel.classList.contains('expanded') ? '×' : '⛶';
      });
    });
    // Restore from hash
    const h = location.hash.replace('#', '');
    if (h === '2d' || h === '3d' || h === 'cam') {
      setExpanded(h);
      const btn = document.querySelector('#panel-' + h + ' button.expand');
      if (btn) btn.textContent = '×';
    }

    // ── Floating info overlay: drag, collapse, dismiss, fetch loop ──
    const fp = document.getElementById('info-fp');
    const fphead = document.getElementById('info-head');
    const fpcollapse = document.getElementById('info-collapse');
    const fpdismiss = document.getElementById('info-dismiss');
    const fpshow = document.getElementById('info-show');
    const LS_KEY = 'sceneInfoFp.v1';
    function fpSave() {
      try {
        localStorage.setItem(LS_KEY, JSON.stringify({
          x: fp.style.left, y: fp.style.top,
          collapsed: fp.classList.contains('collapsed'),
          dismissed: document.body.classList.contains('info-dismissed'),
        }));
      } catch (_) {}
    }
    function fpLoad() {
      try {
        const s = JSON.parse(localStorage.getItem(LS_KEY) || '{}');
        if (s.x) fp.style.left = s.x;
        if (s.y) fp.style.top = s.y;
        if (s.collapsed) fp.classList.add('collapsed');
        if (s.dismissed) document.body.classList.add('info-dismissed');
      } catch (_) {}
    }
    fpLoad();
    // Click title (not buttons) to toggle collapse.
    fphead.addEventListener('click', e => {
      if (e.target.tagName === 'BUTTON') return;
      // dragstart suppresses click via a flag; see drag logic.
      if (fphead._dragged) { fphead._dragged = false; return; }
      fp.classList.toggle('collapsed');
      fpSave();
    });
    fpcollapse.addEventListener('click', e => {
      e.stopPropagation();
      fp.classList.toggle('collapsed');
      fpSave();
    });
    fpdismiss.addEventListener('click', e => {
      e.stopPropagation();
      document.body.classList.add('info-dismissed');
      fpSave();
    });
    fpshow.addEventListener('click', () => {
      document.body.classList.remove('info-dismissed');
      fpSave();
    });
    // Drag — pointerdown on the header, follow until pointerup.
    let dragOff = null;
    fphead.addEventListener('pointerdown', e => {
      if (e.target.tagName === 'BUTTON') return;
      const r = fp.getBoundingClientRect();
      dragOff = { dx: e.clientX - r.left, dy: e.clientY - r.top };
      fphead.setPointerCapture(e.pointerId);
      fphead._dragged = false;
    });
    fphead.addEventListener('pointermove', e => {
      if (!dragOff) return;
      const x = e.clientX - dragOff.dx;
      const y = e.clientY - dragOff.dy;
      // Clamp to viewport so the header is always grabbable.
      const maxX = window.innerWidth  - fp.offsetWidth - 4;
      const maxY = window.innerHeight - 30;
      fp.style.left = Math.max(4, Math.min(x, maxX)) + 'px';
      fp.style.top  = Math.max(4, Math.min(y, maxY)) + 'px';
      fphead._dragged = true;
    });
    fphead.addEventListener('pointerup', e => {
      dragOff = null;
      try { fphead.releasePointerCapture(e.pointerId); } catch (_) {}
      if (fphead._dragged) fpSave();
    });

    // Fetch /api/state and populate the floating panel.
    const fmt = n => Number(n).toFixed(2);
    async function fpTick() {
      try {
        const r = await fetch('/api/state', { cache: 'no-store' });
        if (r.ok) {
          const s = await r.json();
          const objs = (s.objects || []).slice().sort(
            (a, b) => a.cls.localeCompare(b.cls));
          document.getElementById('info-stamp').textContent =
            `${objs.length} obj · ${(s.relations || []).length} rel · ${(s.scene_graph && s.scene_graph.edges || []).length} sg · t=${fmt(s.stamp_unix)}`;
          const robotEl = document.getElementById('info-pose');
          if (s.robot) {
            robotEl.textContent =
              `(${fmt(s.robot.x)}, ${fmt(s.robot.y)}, ${fmt(s.robot.z)}) yaw=${fmt(s.robot.yaw)}`;
          } else {
            robotEl.textContent = 'no fix yet';
          }
          const tbody = document.getElementById('info-objs');
          if (!objs.length) {
            tbody.innerHTML = '<tr><td colspan="3" style="color:#555">—</td></tr>';
          } else {
            tbody.innerHTML = objs.map(o => `
              <tr>
                <td class="id">${o.short_id}</td>
                <td class="cls">${o.cls}</td>
                <td class="pp ${o.missing ? 'miss' : ''}">
                  (${fmt(o.pose.x)}, ${fmt(o.pose.y)}) c=${fmt(o.confidence)}
                </td>
              </tr>
            `).join('');
          }
          // Relations as an explicit "<source> <predicate> <target>" list
          // (replaces the old on-canvas dashed lines). short_id = last
          // dotted segment of the object id, e.g. scene.object.cup_001 → cup_001.
          const shortId = id => String(id).split('.').pop();
          const edges = (s.scene_graph && s.scene_graph.edges) || [];
          const relsEl = document.getElementById('info-rels');
          if (!edges.length) {
            relsEl.innerHTML = '<span style="color:#555">none</span>';
          } else {
            relsEl.innerHTML = edges.map(e => `
              <div class="rel">
                <span class="rs">${shortId(e.source_id)}</span>
                <span class="rp">${e.relation}</span>
                <span class="rt">${shortId(e.target_id)}</span>
              </div>
            `).join('');
          }
        }
      } catch (_) { /* swallow; next tick will retry */ }
      setTimeout(fpTick, 500);
    }
    fpTick();
    // Esc to restore split view
    window.addEventListener('keydown', e => {
      if (e.key === 'Escape' && document.querySelector('.panel.expanded')) {
        setExpanded(null);
        document.querySelectorAll('button.expand').forEach(b => b.textContent = '⛶');
      }
    });
  </script>
</body>
</html>
"""


# ── 3D viewer ───────────────────────────────────────────────────────────────
# Self-contained HTML using three.js via importmap (CDN). Renders:
#   - per-object Points (downsampled point cloud, coloured by class hash)
#   - per-object 12-edge bounding-box LineSegments
#   - per-object Sprite label (class + obs count)
#   - axis gizmo + grid + lighting
# Controls:
#   - Drag = rotate (OrbitControls)
#   - Scroll = zoom
#   - Right-drag = pan
#   - W/A/S/D + Q/E = fly-mode forward/strafe/up-down (camera-relative)
#   - Click = raycast pick a bbox; sidebar shows class/conf/obs
# Polls /api/objects3d every 1s; rebuilds dirty meshes only.
_INDEX_CAM_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>scene cam — robonix</title>
  <style>
    html, body { margin: 0; padding: 0; height: 100%;
                 background: #08090c; color: #d8dde6;
                 font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
    /* Two stacked tiles, RGB on top of depth. Each tile takes half the
       viewport. Image fills the tile keeping aspect; the dark bg shows
       through letterboxing when the panel aspect doesn't match the
       camera's. */
    #stack { display: grid; grid-template-rows: 1fr 1fr;
             height: 100vh; gap: 1px; background: #1a1d24; }
    .tile { position: relative; background: #08090c;
            min-width: 0; min-height: 0; overflow: hidden;
            display: flex; align-items: center; justify-content: center; }
    .tile img { max-width: 100%; max-height: 100%;
                object-fit: contain; image-rendering: pixelated; }
    .tile .label { position: absolute; top: 6px; left: 8px;
                   z-index: 5; padding: 3px 8px;
                   background: rgba(10,12,16,0.78); border: 1px solid #303542;
                   border-radius: 4px; font-size: 11px; color: #889;
                   pointer-events: none; }
    .tile .label b { color: #f0c050; font-weight: 600; }
    .tile .stale { color: #ee7066; }
    .tile .empty { color: #555;
                   font-size: 12px; text-align: center; padding: 1em; }
  </style>
</head>
<body>
  <div id="stack">
    <div class="tile" id="rgb">
      <span class="label"><b>rgb</b> <span class="meta">—</span></span>
      <img id="rgb_img" alt="" />
      <div class="empty" id="rgb_empty">no rgb stream connected</div>
    </div>
    <div class="tile" id="depth">
      <span class="label"><b>depth</b> <span class="meta">—</span> <i style="color:#666">(near=bright)</i></span>
      <img id="depth_img" alt="" />
      <div class="empty" id="depth_empty">no depth frame available</div>
    </div>
  </div>
  <script>
    const POLL_MS = 200;  // 5 Hz
    const STALE_MS = 2000; // mark stale if no update in 2s

    let lastRgbStamp = 0, lastDepthStamp = 0;

    function fmtAge(stamp_ms) {
      if (!stamp_ms) return '—';
      const age = (Date.now() - stamp_ms) / 1000;
      if (age < 0 || age > 1e6) return 'stamp ' + Math.round(stamp_ms / 1000);
      return age.toFixed(1) + 's ago';
    }

    function applyTile(panelId, payload, lastStamp) {
      const tile = document.getElementById(panelId);
      const img  = document.getElementById(panelId + '_img');
      const empty = document.getElementById(panelId + '_empty');
      const meta = tile.querySelector('.meta');
      if (!payload || !payload.png_b64) {
        img.style.display = 'none';
        empty.style.display = '';
        meta.textContent = '—';
        return lastStamp;
      }
      empty.style.display = 'none';
      img.style.display = '';
      // Only re-render when stamp changes (avoid flicker on identical frames).
      if (payload.stamp_ms !== lastStamp) {
        img.src = 'data:image/png;base64,' + payload.png_b64;
      }
      const ageStr = fmtAge(payload.stamp_ms);
      const stale = (Date.now() - payload.stamp_ms) > STALE_MS;
      meta.textContent = `${payload.width}x${payload.height} ${payload.encoding} · ${ageStr}`;
      meta.classList.toggle('stale', stale);
      return payload.stamp_ms;
    }

    async function tick() {
      try {
        const r = await fetch('/api/camera', { cache: 'no-store' });
        const j = await r.json();
        lastRgbStamp = applyTile('rgb', j.rgb, lastRgbStamp);
        lastDepthStamp = applyTile('depth', j.depth, lastDepthStamp);
      } catch (_) { /* swallow — next tick retries */ }
      setTimeout(tick, POLL_MS);
    }
    tick();
  </script>
</body>
</html>
"""


_INDEX_3D_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>scene 3D — robonix</title>
  <style>
    html, body { margin: 0; padding: 0; height: 100%; background: #08090c;
                 color: #d8dde6; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
    #app { position: absolute; inset: 0; }
    canvas { display: block; }
    /* HUD: collapsed state is a tiny `[?]` pill in the top-left so it
       can NEVER overlap the inspector on a narrow iframe (combined-
       layout middle column on a MacBook Air). Expanded reveals title
       + keymap. Stats moved to a separate bottom-left bubble so the
       3D iframe never shows two boxes fighting at the top. */
    #hud { position: absolute; top: 8px; left: 8px; z-index: 5; }
    #hud .head {
      display: inline-flex; align-items: center; gap: 6px;
      padding: 4px 8px; background: rgba(10,12,16,0.85);
      border: 1px solid #303542; border-radius: 4px;
      cursor: pointer; user-select: none; font-size: 11px; color: #889;
    }
    #hud .head:hover { color: #f0c050; border-color: #5a606e; }
    #hud .head b { color: #f0c050; font-weight: 600;
                   display: none; }      /* hidden when collapsed */
    #hud.expanded .head b { display: inline; }
    #hud .toggle { color: #6a6f7a; font-size: 10px; }
    #hud .help {
      display: none; margin-top: 4px; padding: 6px 10px;
      background: rgba(10,12,16,0.85); border: 1px solid #303542;
      border-radius: 4px; max-width: 280px; font-size: 11px;
      line-height: 1.5; color: #aab;
    }
    #hud.expanded .help { display: block; }
    #hud .help .k, #hud .help .key { color: #5fc; }
    #hud-stats {
      position: absolute; bottom: 30px; left: 8px;
      padding: 3px 8px; background: rgba(10,12,16,0.7);
      border-radius: 3px; font-size: 10px; color: #889;
      pointer-events: none;
    }
    /* Inspector: top-right. Tiny dot when empty so it can't overflow.
       Click sets `.expanded` and fills in object details. */
    #panel { position: absolute; top: 8px; right: 8px; z-index: 5;
             padding: 6px 10px; background: rgba(10,12,16,0.85);
             border: 1px solid #303542; border-radius: 4px;
             font-size: 12px; line-height: 1.5; overflow-wrap: break-word; }
    #panel.empty { padding: 3px 8px; font-size: 10px;
                   color: #6a6f7a; font-style: italic; }
    #panel:not(.empty) { min-width: 200px; }
    #panel h3 { margin: 0 0 6px; font-size: 13px; color: #f0c050; }
    #panel .row { display: flex; justify-content: space-between; gap: 16px; }
    #panel .k { color: #889; }
    #foot { position: absolute; bottom: 8px; left: 8px; right: 8px;
            padding: 4px 10px; background: rgba(10,12,16,0.7);
            border-radius: 3px; font-size: 11px; color: #889; pointer-events: none; }
    a { color: #5fc; }
  </style>
  <script type="importmap">
  {
    "imports": {
      "three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
      "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/",
      "three/examples/jsm/loaders/STLLoader.js": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/loaders/STLLoader.js"
    }
  }
  </script>
</head>
<body>
  <div id="app"></div>
  <div id="hud">
    <div class="head" id="hud-head" title="click to expand / collapse">
      <span class="toggle" id="hud-toggle">▸</span>
      <b>scene 3D</b>
    </div>
    <div class="help">
      <div><span class="k">drag</span> rotate · <span class="k">scroll</span> zoom · <span class="k">right-drag</span> pan</div>
      <div><span class="key">W A S D</span> fly · <span class="key">Q E</span> down/up · <span class="key">Shift</span> 3× speed</div>
      <div><span class="key">click</span> pick object · <span class="key">R</span> reset · <span class="key">G</span> grid</div>
    </div>
  </div>
  <div id="hud-stats">objects: 0 · points: 0</div>
  <div id="panel" class="empty">▸</div>
  <div id="foot"><a href="/">← back to 2D map</a> · auto-refresh 1Hz · <span id="foot-time">—</span></div>

  <script type="module">
    import * as THREE from 'three';
    import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

    // ── Scene setup ────────────────────────────────────────────────────
    const app = document.getElementById('app');
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setClearColor(0x08090c, 1.0);
    app.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(60, window.innerWidth/window.innerHeight, 0.05, 200);
    // Robonix convention: map frame X-forward, Y-left, Z-up. Use a
    // perspective camera looking down at +Z so the user sees the
    // floor as the ground plane.
    camera.up.set(0, 0, 1);
    camera.position.set(4, 4, 4);
    camera.lookAt(0, 0, 0.5);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0.5);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;

    // ── Lights + grid + axes ───────────────────────────────────────────
    scene.add(new THREE.HemisphereLight(0x9ab8ff, 0x202028, 0.9));
    const dir = new THREE.DirectionalLight(0xffffff, 0.6);
    dir.position.set(2, 2, 5);
    scene.add(dir);

    let grid = new THREE.GridHelper(20, 40, 0x303542, 0x202428);
    grid.rotation.x = Math.PI / 2;  // GridHelper is in XZ plane; rotate to XY for Z-up
    scene.add(grid);
    const axes = new THREE.AxesHelper(0.6);
    scene.add(axes);

    // ── 2D occupancy underlay ──────────────────────────────────────────
    // The user reads the 3D scene by looking down at point clouds + bboxes.
    // Without a floor reference it's hard to tell where a wall actually
    // is or how far the robot has driven. Drop the slam_toolbox / rtabmap
    // occupancy PNG onto a horizontal plane at z = -0.01 (just below
    // origin so it doesn't fight axes/grid for z-buffer). The same PNG
    // the 2D panel uses is fetched via /api/state and re-textured each
    // tick.
    let mapPlane = null;          // current THREE.Mesh
    let mapPlaneStamp = 0;        // last occupancy.stamp_ms applied
    function decodeMapPng(b64) {
        return new Promise((resolve, reject) => {
            const img = new Image();
            img.onload = () => resolve(img);
            img.onerror = () => reject(new Error('decode'));
            img.src = 'data:image/png;base64,' + b64;
        });
    }
    async function refreshMapPlane(occ) {
        if (!occ || !occ.png_b64 || !occ.stamp_ms) return;
        if (occ.stamp_ms === mapPlaneStamp) return;
        const img = await decodeMapPng(occ.png_b64).catch(() => null);
        if (!img) return;
        const W = occ.width, H = occ.height, R = occ.resolution;
        // Repaint the PNG with darker free-space + hard-edged walls so
        // the underlay reads at a glance (the raw OccupancyGrid PNG is
        // black/grey/white; in 3D against the dark background that's
        // basically invisible). Map: white(unknown)→transparent,
        // black(wall)→solid #d8dde6, grey(free)→very dim #1a1d24.
        const cv = document.createElement('canvas');
        cv.width = W; cv.height = H;
        const cx = cv.getContext('2d');
        cx.drawImage(img, 0, 0);
        const id = cx.getImageData(0, 0, W, H);
        const px = id.data;
        for (let i = 0; i < px.length; i += 4) {
            const v = px[i];          // R; PNG is greyscale-ish
            const a = px[i + 3];
            if (a === 0 || v >= 220) {
                // unknown → transparent
                px[i + 3] = 0;
            } else if (v <= 60) {
                // wall / occupied → bright + opaque
                px[i] = 0xd8; px[i + 1] = 0xdd; px[i + 2] = 0xe6;
                px[i + 3] = 240;
            } else {
                // free space → dim background tint, semi-transparent
                px[i] = 0x4a; px[i + 1] = 0x52; px[i + 2] = 0x60;
                px[i + 3] = 90;
            }
        }
        cx.putImageData(id, 0, 0);

        const tex = new THREE.CanvasTexture(cv);
        tex.minFilter = THREE.LinearFilter;
        tex.magFilter = THREE.LinearFilter;
        tex.needsUpdate = true;

        const planeW = W * R, planeH = H * R;
        // The PNG's pixel (0,0) is at (origin_x, origin_y + planeH);
        // we'll set a Mesh at origin + half-extent so plane corners
        // line up with the OccupancyGrid origin.
        const geom = new THREE.PlaneGeometry(planeW, planeH);
        const mat = new THREE.MeshBasicMaterial({
            map: tex, transparent: true, depthWrite: false,
            side: THREE.DoubleSide,
        });
        if (mapPlane) scene.remove(mapPlane);
        mapPlane = new THREE.Mesh(geom, mat);
        // OccupancyGrid orientation: image y axis is down (row 0 is
        // top), but world y axis is up. Three.js PlaneGeometry is in
        // local +X +Y plane with +Z normal — by default flat in XY at
        // z=0 already; we just flip the texture's V to match the PNG.
        mat.map.flipY = true;
        mat.map.needsUpdate = true;
        mapPlane.position.set(
            occ.origin_x + planeW / 2,
            occ.origin_y + planeH / 2,
            -0.01,
        );
        scene.add(mapPlane);
        mapPlaneStamp = occ.stamp_ms;
    }
    async function pollMap() {
        try {
            const r = await fetch('/api/state', {cache: 'no-store'});
            const j = await r.json();
            await refreshMapPlane(j.occupancy);
        } catch (e) { /* swallow — pollRobot uses the same endpoint */ }
    }
    pollMap();
    // SLAM /map updates ~1 Hz; 2 Hz polling gives a tight cadence
    // without re-decoding the PNG on every frame.
    setInterval(pollMap, 500);

    // ── Object registry (id → mesh-set) ────────────────────────────────
    const objectMeshes = new Map();   // id → { points, bboxLines, label, data }
    const pickables = [];             // bbox edges as raycaster targets

    const raycaster = new THREE.Raycaster();
    raycaster.params.Line = { threshold: 0.05 };
    let highlighted = null;

    // ── Robot body — composite Tiago-like proxy ────────────────────────
    // Hierarchy of primitives that match Tiago's silhouette:
    //   mobile base (cylinder, 0.54 m dia)
    //   torso column (cylinder)
    //   shoulder block (box, where the arm mounts)
    //   neck stub + head (sphere with eye accent)
    //   forward camera bezel (the head's "face")
    // Everything is parented to a `THREE.Group` so updateRobotPose
    // moves the whole body atomically. Materials are translucent so the
    // ConceptGraphs object pcds rendered behind/around the robot stay
    // visible.
    //
    // (We had a brief attempt to load the real Tiago STL meshes from
    // PAL's tiago_description / pmb2_description packages; the 5
    // visual STLs are still under static/urdf/meshes/ for future use,
    // but loading them in three.js needs work — the meshes have
    // per-link local origins that don't compose into the right body
    // shape without the actual URDF joint chain. Reverted to the
    // proxy so the user can at least see the robot.)
    const robotGroup = new THREE.Group();

    // Visual style — slightly different shades per part so the
    // articulation reads at a glance.
    const robotMatBase = new THREE.MeshStandardMaterial({
        color: 0xffaa33, transparent: true, opacity: 0.65,
        emissive: 0x553300, emissiveIntensity: 0.20,
        metalness: 0.15, roughness: 0.55,
    });
    const robotMatTorso = new THREE.MeshStandardMaterial({
        color: 0xfff0d0, transparent: true, opacity: 0.55,
        emissive: 0x442200, emissiveIntensity: 0.15,
        metalness: 0.05, roughness: 0.65,
    });
    const robotMatHead = new THREE.MeshStandardMaterial({
        color: 0xfff0d0, transparent: true, opacity: 0.7,
        emissive: 0x442200, emissiveIntensity: 0.18,
        metalness: 0.10, roughness: 0.5,
    });
    const robotMatAccent = new THREE.MeshStandardMaterial({
        color: 0x222831, transparent: false,
        emissive: 0x000000, metalness: 0.30, roughness: 0.40,
    });

    // 1) Mobile base — Tiago is ~54 cm dia, 30 cm tall. Stand it up by
    //    rotating CylinderGeometry's natural Y-up axis to Z-up.
    const baseGeom = new THREE.CylinderGeometry(0.27, 0.27, 0.30, 24);
    baseGeom.rotateX(Math.PI / 2);
    const baseMesh = new THREE.Mesh(baseGeom, robotMatBase);
    baseMesh.position.set(0, 0, 0.15);
    robotGroup.add(baseMesh);

    // 2) Torso column — slimmer, sits on top of the base.
    const torsoGeom = new THREE.CylinderGeometry(0.13, 0.15, 0.55, 20);
    torsoGeom.rotateX(Math.PI / 2);
    const torsoMesh = new THREE.Mesh(torsoGeom, robotMatTorso);
    torsoMesh.position.set(0, 0, 0.30 + 0.275);
    robotGroup.add(torsoMesh);

    // 3) Shoulder block — wider plate where Tiago's arm mounts.
    const shoulderGeom = new THREE.BoxGeometry(0.32, 0.42, 0.18);
    const shoulderMesh = new THREE.Mesh(shoulderGeom, robotMatTorso);
    shoulderMesh.position.set(0.0, 0.0, 0.30 + 0.55 + 0.09);
    robotGroup.add(shoulderMesh);

    // 4) Neck stub.
    const neckGeom = new THREE.CylinderGeometry(0.05, 0.06, 0.08, 16);
    neckGeom.rotateX(Math.PI / 2);
    const neckMesh = new THREE.Mesh(neckGeom, robotMatHead);
    neckMesh.position.set(0.02, 0, 0.30 + 0.55 + 0.18 + 0.04);
    robotGroup.add(neckMesh);

    // 5) Head — sphere with darker face plate pointing +X.
    const headGeom = new THREE.SphereGeometry(0.12, 24, 16);
    const headMesh = new THREE.Mesh(headGeom, robotMatHead);
    headMesh.position.set(0.05, 0, 0.30 + 0.55 + 0.18 + 0.08 + 0.10);
    robotGroup.add(headMesh);

    const faceGeom = new THREE.BoxGeometry(0.04, 0.18, 0.07);
    const faceMesh = new THREE.Mesh(faceGeom, robotMatAccent);
    faceMesh.position.set(0.05 + 0.10, 0, 0.30 + 0.55 + 0.18 + 0.08 + 0.10);
    robotGroup.add(faceMesh);

    // 6) Stylised arm — single capsule angled forward+down.
    const armGeom = new THREE.CapsuleGeometry(0.045, 0.45, 6, 12);
    const armMesh = new THREE.Mesh(armGeom, robotMatBase);
    armMesh.position.set(0.05, -0.20, 0.30 + 0.55 + 0.18 + 0.05);
    armMesh.rotation.set(0, 0.55, 0);
    robotGroup.add(armMesh);

    scene.add(robotGroup);

    // Forward-pointing arrow so yaw is visible even when zoomed out.
    // Lives outside robotGroup because ArrowHelper has its own internal
    // rotation that we set per-frame from yaw.
    const arrowDir = new THREE.Vector3(1, 0, 0);
    const robotArrow = new THREE.ArrowHelper(
        arrowDir, new THREE.Vector3(0, 0, 0.55), 0.6, 0xff5522, 0.18, 0.10,
    );
    scene.add(robotArrow);

    // Robot label sprite (separate so it doesn't tilt with the group).
    const robotLabelCv = document.createElement('canvas');
    robotLabelCv.width = 200; robotLabelCv.height = 56;
    {
        const cx = robotLabelCv.getContext('2d');
        cx.fillStyle = 'rgba(8,9,12,0.92)'; cx.fillRect(0,0,robotLabelCv.width,robotLabelCv.height);
        cx.strokeStyle = '#ffaa33'; cx.lineWidth = 2;
        cx.strokeRect(1,1,robotLabelCv.width-2,robotLabelCv.height-2);
        cx.fillStyle = '#ffaa33'; cx.font = 'bold 22px ui-monospace, monospace';
        cx.textAlign = 'center'; cx.textBaseline = 'middle';
        cx.fillText('robot', robotLabelCv.width/2, robotLabelCv.height/2);
    }
    const robotLabel = new THREE.Sprite(new THREE.SpriteMaterial({
        map: new THREE.CanvasTexture(robotLabelCv), transparent: true, depthTest: false,
    }));
    robotLabel.scale.set(0.5, 0.14, 1);
    scene.add(robotLabel);

    function updateRobotPose(rx, ry, rz, ryaw) {
        // The whole composite body moves+rotates as one rigid frame.
        robotGroup.position.set(rx, ry, rz);
        robotGroup.rotation.set(0, 0, ryaw);
        // Arrow: rotate the forward unit-vec by yaw, then place at the
        // body's mid-height for visibility.
        const dir = new THREE.Vector3(Math.cos(ryaw), Math.sin(ryaw), 0);
        robotArrow.position.set(rx, ry, rz + 0.55);
        robotArrow.setDirection(dir);
        // Label hovers above the head.
        robotLabel.position.set(rx, ry, rz + 1.40);
    }

    async function pollRobot() {
        try {
            const r = await fetch('/api/state', {cache: 'no-store'});
            const j = await r.json();
            const rb = j.robot;
            if (rb && Number.isFinite(rb.x)) {
                updateRobotPose(rb.x || 0, rb.y || 0, rb.z || 0, rb.yaw || 0);
            }
        } catch (e) {}
    }
    pollRobot();
    setInterval(pollRobot, 500);  // 2 Hz — same as the 2D map / panel
                                  // pollers; matches /map publish rate
                                  // and avoids redundant /api/state hits

    // ── Scene Graph relation edges ────────────────────────────────────
    const sgEdgeGroup = new THREE.Group();
    sgEdgeGroup.name = 'scene-graph-edges';
    scene.add(sgEdgeGroup);
    const sgLabelGroup = new THREE.Group();
    sgLabelGroup.name = 'scene-graph-labels';
    scene.add(sgLabelGroup);

    const SG_EDGE_COLORS = {
      on_top_of: 0x4caf50,  // green
      under:     0x4caf50,
      inside:    0x2196f3,  // blue
      contains:  0x2196f3,
      near:      0x9e9e9e,  // gray
      attached_to: 0xff9800, // orange
      part_of:   0xff9800,
      same_object: 0xf44336, // red
    };

    function makeSGLabel(text) {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 48;
      const ctx = c.getContext('2d');
      ctx.font = 'bold 22px monospace';
      ctx.fillStyle = '#e8eaed';
      ctx.textAlign = 'center';
      ctx.fillText(text, 128, 32);
      const tex = new THREE.CanvasTexture(c);
      const mat = new THREE.SpriteMaterial({map: tex, transparent: true, depthTest: false});
      const s = new THREE.Sprite(mat);
      s.scale.set(0.6, 0.12, 1);
      return s;
    }

    function rebuildSGEdges(sgData) {
      // Clear old edges
      while (sgEdgeGroup.children.length) sgEdgeGroup.remove(sgEdgeGroup.children[0]);
      while (sgLabelGroup.children.length) sgLabelGroup.remove(sgLabelGroup.children[0]);
      if (!sgData || !sgData.edges || sgData.edges.length === 0) return;

      // Build position lookup from current objectMeshes
      const posMap = new Map();
      for (const [id, entry] of objectMeshes) {
        if (entry.data && entry.data.center) {
          posMap.set(id, entry.data.center);
        }
      }

      for (const edge of sgData.edges) {
        const posA = posMap.get(edge.source_id);
        const posB = posMap.get(edge.target_id);
        if (!posA || !posB) continue;

        const color = SG_EDGE_COLORS[edge.relation] || 0x757575;
        const pts = new Float32Array([
          posA[0], posA[1], posA[2],
          posB[0], posB[1], posB[2],
        ]);
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.BufferAttribute(pts, 3));
        const mat = new THREE.LineBasicMaterial({
          color, linewidth: 2, transparent: true, opacity: 0.7,
        });
        const line = new THREE.LineSegments(geom, mat);
        sgEdgeGroup.add(line);

        // Label at midpoint
        const mx = (posA[0] + posB[0]) / 2;
        const my = (posA[1] + posB[1]) / 2;
        const mz = (posA[2] + posB[2]) / 2 + 0.05;
        const label = makeSGLabel(edge.relation);
        label.position.set(mx, my, mz);
        sgLabelGroup.add(label);
      }
    }

    // Poll scene graph edges at 0.2 Hz (every 5s — it updates at 30s)
    async function pollSGEdges() {
      try {
        const r = await fetch('/api/state', {cache: 'no-store'});
        const j = await r.json();
        if (j.scene_graph) rebuildSGEdges(j.scene_graph);
      } catch (e) {}
    }
    pollSGEdges();
    setInterval(pollSGEdges, 5000);

    // ── Color palette (deterministic per class) ────────────────────────
    function classColor(cls) {
        let h = 0;
        for (let i = 0; i < cls.length; i++) h = (h * 131 + cls.charCodeAt(i)) & 0xffffff;
        const hue = (h % 360) / 360;
        const c = new THREE.Color();
        c.setHSL(hue, 0.65, 0.6);
        return c;
    }

    // ── Build/update object meshes from snapshot ───────────────────────
    function rebuild(snapshot) {
        const incomingIds = new Set();
        let totalPts = 0;
        for (const obj of snapshot.objects) {
            incomingIds.add(obj.id);
            totalPts += obj.points.length;
            const colour = classColor(obj.cls);
            let entry = objectMeshes.get(obj.id);
            if (!entry) {
                entry = { points: null, bboxLines: null, label: null, data: null, pickGroup: null };
                objectMeshes.set(obj.id, entry);
            }
            // ── Points ──
            const ptArr = new Float32Array(obj.points.length * 3);
            for (let i = 0; i < obj.points.length; i++) {
                ptArr[i*3+0] = obj.points[i][0];
                ptArr[i*3+1] = obj.points[i][1];
                ptArr[i*3+2] = obj.points[i][2];
            }
            if (entry.points) scene.remove(entry.points);
            const ptGeom = new THREE.BufferGeometry();
            ptGeom.setAttribute('position', new THREE.BufferAttribute(ptArr, 3));
            const ptMat = new THREE.PointsMaterial({
                color: colour, size: 0.025, sizeAttenuation: true,
                transparent: true, opacity: 0.85,
            });
            entry.points = new THREE.Points(ptGeom, ptMat);
            scene.add(entry.points);

            // ── Bounding box edges (yaw-rotated, vertical Z) ──
            // The Python side computed an OBB constrained to yaw-only
            // (perception_concept_graphs.py:export_3d_snapshot) and
            // gave us 8 corners that are already in world frame with
            // Z-axis vertical. We must NOT min/max them — that would
            // throw away the yaw rotation and give a grid-aligned box
            // (the bug the user kept catching).
            //
            // Open3D's get_box_points() returns corners in this order:
            //   0:(--+), 1:(+--), 2:(-+-), 3:(--+),
            //   4:(+++), 5:(-++), 6:(+-+), 7:(++-)
            // (signs along local x/y/z). 12 cube edges connect any
            // pair differing in exactly one coordinate sign.
            const c = obj.bbox_corners;
            const O3D_EDGES = [
                [0,1],[0,2],[0,3],[1,6],[1,7],[2,5],
                [2,7],[3,5],[3,6],[4,5],[4,6],[4,7],
            ];
            const edgePts = new Float32Array(O3D_EDGES.length * 2 * 3);
            for (let e = 0; e < O3D_EDGES.length; e++) {
                const [a, b] = O3D_EDGES[e];
                edgePts[e*6+0]=c[a][0]; edgePts[e*6+1]=c[a][1]; edgePts[e*6+2]=c[a][2];
                edgePts[e*6+3]=c[b][0]; edgePts[e*6+4]=c[b][1]; edgePts[e*6+5]=c[b][2];
            }
            const edgeGeom = new THREE.BufferGeometry();
            edgeGeom.setAttribute('position', new THREE.BufferAttribute(edgePts, 3));
            if (entry.bboxLines) scene.remove(entry.bboxLines);
            const edgeMat = new THREE.LineBasicMaterial({ color: colour, linewidth: 2 });
            entry.bboxLines = new THREE.LineSegments(edgeGeom, edgeMat);
            scene.add(entry.bboxLines);

            // ── ctr + sz (Vector3 each) — used by label + click target ──
            // Centroid of the 8 corners.
            const ctr = new THREE.Vector3();
            for (const p of c) ctr.add(new THREE.Vector3(p[0], p[1], p[2]));
            ctr.multiplyScalar(1 / 8);
            // Local axes from corner 0 — three edges of the box.
            const ax = new THREE.Vector3(c[1][0]-c[0][0], c[1][1]-c[0][1], c[1][2]-c[0][2]);
            const ay = new THREE.Vector3(c[2][0]-c[0][0], c[2][1]-c[0][1], c[2][2]-c[0][2]);
            const az = new THREE.Vector3(c[3][0]-c[0][0], c[3][1]-c[0][1], c[3][2]-c[0][2]);
            // sz keeps the Vector3 shape so `sz.x/sz.y/sz.z` work in
            // the label code below (we got bitten by this last time —
            // sz used to be a plain number and label position read
            // `sz.z` as undefined, so labels disappeared).
            const sz = new THREE.Vector3(ax.length(), ay.length(), az.length());
            ax.normalize(); ay.normalize(); az.normalize();
            const rotMat = new THREE.Matrix4().makeBasis(ax, ay, az);

            // ── Click target — translucent rotated mesh ──
            if (entry.pickGroup) scene.remove(entry.pickGroup);
            const pickGeom = new THREE.BoxGeometry(
                Math.max(sz.x, 0.01), Math.max(sz.y, 0.01), Math.max(sz.z, 0.01),
            );
            const pickMat = new THREE.MeshBasicMaterial({
                color: colour, transparent: true, opacity: 0.0, depthWrite: false,
            });
            const pickMesh = new THREE.Mesh(pickGeom, pickMat);
            pickMesh.position.copy(ctr);
            pickMesh.quaternion.setFromRotationMatrix(rotMat);
            pickMesh.userData = { id: obj.id, data: obj, bbox: entry.bboxLines };
            entry.pickGroup = pickMesh;
            scene.add(pickMesh);

            // ── Label sprite ──
            if (entry.label) scene.remove(entry.label);
            const labelText = `${obj.cls} ×${obj.num_detections}`;
            const cv = document.createElement('canvas');
            cv.width = 256; cv.height = 64;
            const ctx = cv.getContext('2d');
            ctx.fillStyle = 'rgba(8,9,12,0.9)'; ctx.fillRect(0,0,cv.width,cv.height);
            ctx.strokeStyle = '#'+colour.getHexString(); ctx.lineWidth = 2;
            ctx.strokeRect(1,1,cv.width-2,cv.height-2);
            ctx.fillStyle = '#'+colour.getHexString();
            ctx.font = 'bold 28px ui-monospace, monospace';
            ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
            ctx.fillText(labelText, cv.width/2, cv.height/2);
            const tex = new THREE.CanvasTexture(cv);
            const spMat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false });
            const sp = new THREE.Sprite(spMat);
            sp.position.set(ctr.x, ctr.y, ctr.z + sz.z * 0.55 + 0.10);
            sp.scale.set(0.6, 0.15, 1);
            entry.label = sp;
            scene.add(sp);

            entry.data = obj;
        }
        // GC removed objects
        for (const [id, entry] of [...objectMeshes.entries()]) {
            if (incomingIds.has(id)) continue;
            if (entry.points)    scene.remove(entry.points);
            if (entry.bboxLines) scene.remove(entry.bboxLines);
            if (entry.label)     scene.remove(entry.label);
            if (entry.pickGroup) scene.remove(entry.pickGroup);
            objectMeshes.delete(id);
        }
        // Rebuild pickables list
        pickables.length = 0;
        for (const e of objectMeshes.values()) if (e.pickGroup) pickables.push(e.pickGroup);
        document.getElementById('hud-stats').textContent =
            `objects: ${snapshot.objects.length} · points: ${totalPts}`;
        document.getElementById('foot-time').textContent =
            new Date(snapshot.stamp_unix * 1000).toLocaleTimeString();
    }

    async function poll() {
        try {
            const r = await fetch('/api/objects3d', {cache: 'no-store'});
            const j = await r.json();
            rebuild(j);
        } catch (e) { console.warn('poll failed', e); }
    }
    poll();
    setInterval(poll, 1000);

    // ── WASD fly mode (camera-relative motion) ─────────────────────────
    const keys = new Set();
    window.addEventListener('keydown', e => {
        if (e.target && e.target.tagName === 'INPUT') return;
        keys.add(e.key.toLowerCase());
        if (e.key === 'r' || e.key === 'R') resetView();
        if (e.key === 'g' || e.key === 'G') { grid.visible = !grid.visible; }
    });
    window.addEventListener('keyup', e => keys.delete(e.key.toLowerCase()));

    function resetView() {
        camera.position.set(4, 4, 4);
        controls.target.set(0, 0, 0.5);
        controls.update();
    }

    function tickFly(dt) {
        // Speed in m/s. Shift accelerates.
        let speed = 1.5 * dt;
        if (keys.has('shift')) speed *= 3;
        const fwd = new THREE.Vector3();
        camera.getWorldDirection(fwd);
        const right = new THREE.Vector3().crossVectors(fwd, camera.up).normalize();
        const up = camera.up.clone();
        const move = new THREE.Vector3();
        if (keys.has('w')) move.addScaledVector(fwd, speed);
        if (keys.has('s')) move.addScaledVector(fwd, -speed);
        if (keys.has('d')) move.addScaledVector(right, speed);
        if (keys.has('a')) move.addScaledVector(right, -speed);
        if (keys.has('e')) move.addScaledVector(up, speed);
        if (keys.has('q')) move.addScaledVector(up, -speed);
        if (move.lengthSq() > 0) {
            camera.position.add(move);
            controls.target.add(move);
        }
    }

    // ── Click pick ─────────────────────────────────────────────────────
    function onClick(ev) {
        const rect = renderer.domElement.getBoundingClientRect();
        const mouse = new THREE.Vector2(
            ((ev.clientX - rect.left) / rect.width) * 2 - 1,
            -((ev.clientY - rect.top) / rect.height) * 2 + 1,
        );
        raycaster.setFromCamera(mouse, camera);
        const hits = raycaster.intersectObjects(pickables, false);
        if (highlighted) {
            // Restore highlight
            const prev = objectMeshes.get(highlighted);
            if (prev && prev.bboxLines) prev.bboxLines.material.linewidth = 2;
            highlighted = null;
        }
        if (hits.length === 0) {
            const panel = document.getElementById('panel');
            panel.classList.add('empty');
            panel.innerHTML = 'click an object';
            return;
        }
        const top = hits[0].object;
        const data = top.userData.data;
        highlighted = data.id;
        const entry = objectMeshes.get(data.id);
        if (entry && entry.bboxLines) entry.bboxLines.material.linewidth = 4;
        const panel = document.getElementById('panel');
        panel.classList.remove('empty');
        const c = data.center;
        panel.innerHTML = `
          <h3>${data.cls}</h3>
          <div class="row"><span class="k">id</span><span>${data.id.slice(0,12)}…</span></div>
          <div class="row"><span class="k">center (m)</span><span>${c[0].toFixed(2)}, ${c[1].toFixed(2)}, ${c[2].toFixed(2)}</span></div>
          <div class="row"><span class="k">observations</span><span>${data.num_detections}</span></div>
          <div class="row"><span class="k">points</span><span>${data.n_points}</span></div>
          <div class="row"><span class="k">conf (mean)</span><span>${data.conf_mean.toFixed(2)}</span></div>
        `;
    }
    renderer.domElement.addEventListener('click', onClick);

    // ── Resize ─────────────────────────────────────────────────────────
    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });

    // ── HUD collapse toggle ───────────────────────────────────────────
    // Default: collapsed (just the ▸ arrow). Click to expand and reveal
    // title + keymap; the arrow flips to ▾. Persists in localStorage.
    const hudEl = document.getElementById('hud');
    const hudHead = document.getElementById('hud-head');
    const hudToggle = document.getElementById('hud-toggle');
    const HUD_KEY = 'scene3dHud.expanded';
    function setHud(expanded) {
      hudEl.classList.toggle('expanded', expanded);
      hudToggle.textContent = expanded ? '▾' : '▸';
      try { localStorage.setItem(HUD_KEY, expanded ? '1' : '0'); } catch (_) {}
    }
    setHud(localStorage.getItem(HUD_KEY) === '1');
    hudHead.addEventListener('click', () => setHud(!hudEl.classList.contains('expanded')));

    // ── Render loop ────────────────────────────────────────────────────
    let last = performance.now();
    function loop(now) {
        const dt = Math.min(0.1, (now - last) / 1000);
        last = now;
        tickFly(dt);
        controls.update();
        renderer.render(scene, camera);
        requestAnimationFrame(loop);
    }
    requestAnimationFrame(loop);
  </script>
</body>
</html>
"""


# ── User annotation page (/user) ─────────────────────────────────────────────
# The end-user map page: SLAM occupancy underlay + LIGHTWEIGHT object overlay
# + user-drawn room polygons, with a draw-a-room flow talking to the
# /api/annotations CRUD. Deliberately self-contained (own inline JS, no
# imports from the debug pages' scripts): the two pages evolve independently
# and a debug-UI tweak must never break the user page. Kept dependency-free
# like every other page here (no framework, no build step).
_USER_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>robonix — map & rooms</title>
  <style>
    :root { --fg:#e8eaed; --bg:#0e1015; --panel:#161a22; --acc:#7aa7ff;
            --muted:#7d828b; --warn:#e6c454; --danger:#e06c75; }
    html, body { background: var(--bg); color: var(--fg); margin: 0; height: 100%;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    #app { display: flex; flex-direction: column; height: 100vh; }
    header { display: flex; align-items: center; gap: 14px; padding: 10px 16px;
      background: var(--panel); border-bottom: 1px solid #232936; flex: none; }
    header h1 { font-size: 15px; margin: 0; font-weight: 600; }
    header .meta { font-size: 12px; color: var(--muted); }
    header .stale-alert { font-size: 12px; color: var(--warn); display: none; }
    #main { display: flex; flex: 1; min-height: 0; }
    #panel { width: 260px; flex: none; background: var(--panel);
      border-right: 1px solid #232936; display: flex; flex-direction: column; }
    #panel .actions { padding: 12px; border-bottom: 1px solid #232936; }
    #map-tools { padding: 10px 12px; border-bottom: 1px solid #232936; display: grid; gap: 8px; }
    #map-tools label { display: grid; gap: 4px; color: var(--muted); font-size: 11px; }
    #map-id { width: 100%; box-sizing: border-box; border: 1px solid #33405a; border-radius: 6px;
      background: #0f131b; color: var(--fg); padding: 6px 8px; font-size: 12px; }
    #map-actions, .map-btns { display: flex; gap: 6px; flex-wrap: wrap; }
    #map-status { min-height: 16px; color: var(--muted); font-size: 11px; overflow-wrap: anywhere; }
    #map-status.busy { color: var(--acc); }
    #map-status.ok { color: #8ef0b7; }
    #map-status.err { color: var(--danger); }
    #mode-pill { display: inline-block; margin-left: 8px; padding: 1px 6px; border-radius: 999px;
      border: 1px solid #33405a; color: var(--muted); font-size: 11px; }
    #mode-pill.localization { border-color: #3b633f; color: #8ef0b7; background: rgba(64,150,80,.12); }
    #mode-pill.mapping { border-color: #6a5630; color: var(--warn); background: rgba(230,196,84,.10); }
    #map-list { display: grid; gap: 5px; max-height: min(24vh, 190px); overflow-y: auto;
      overscroll-behavior: contain; scrollbar-gutter: stable; }
    .mapitem { border: 1px solid #232936; border-radius: 8px; padding: 7px 8px; font-size: 12px; background: #121721; cursor: pointer; }
    .mapitem.selected, .mapitem.busy { border-color: var(--acc); }
    .mapitem .name { font-weight: 700; }
    .mapitem .sub { color: var(--muted); font-size: 10px; margin: 2px 0 6px; overflow-wrap: anywhere; }
    .mapitem .map-status { color: var(--acc); font-size: 10px; min-height: 12px; margin-top: 4px; }
    button:disabled { opacity: .55; cursor: progress; }
    button { background: #222a3a; color: var(--fg); border: 1px solid #33405a;
      border-radius: 6px; padding: 6px 10px; font-size: 12px; cursor: pointer; }
    button:hover { background: #2a3550; }
    button.primary { background: #2b4a86; border-color: #3c62ad; }
    button.primary.active { background: var(--acc); color: #0e1015; }
    button.small { padding: 2px 7px; font-size: 11px; }
    button.danger { border-color: #6b3640; color: var(--danger); }
    #room-list { flex: 1; min-height: 0; overflow-y: auto; padding: 7px 10px;
      overscroll-behavior: contain; scrollbar-gutter: stable; }
    .room { --room-color: var(--acc); border: 1px solid #232936;
      border-left: 3px solid var(--room-color); border-radius: 7px; padding: 6px 8px;
      margin-bottom: 6px; font-size: 12px; }
    .room.selected { border-color: var(--acc); }
    .room .name { font-weight: 650; }
    .room .swatch { display: inline-block; width: 8px; height: 8px; margin-right: 6px;
      border-radius: 2px; background: var(--room-color); vertical-align: 1px; }
    .room .sub { color: var(--muted); font-size: 10px; margin: 2px 0 5px 14px; }
    .room .badge { color: #0e1015; background: var(--warn); border-radius: 4px;
      padding: 0 5px; font-size: 10px; font-weight: 700; margin-left: 6px; }
    .room .btns { display: flex; gap: 6px; }
    #canvas-wrap { position: relative; flex: 1; min-width: 0; }
    canvas { display: block; width: 100%; height: 100%; background: #14171f; }
    #hint { position: absolute; top: 10px; left: 50%; transform: translateX(-50%);
      background: rgba(20,23,31,.92); border: 1px solid #33405a; color: var(--fg);
      font-size: 12px; padding: 6px 12px; border-radius: 6px; display: none; }
    #toast { position: absolute; bottom: 14px; left: 50%; transform: translateX(-50%);
      background: rgba(20,23,31,.95); border: 1px solid #6b3640; color: var(--danger);
      font-size: 12px; padding: 6px 12px; border-radius: 6px; display: none; }
    #scene-status { position: absolute; top: 10px; right: 10px;
      background: rgba(20,23,31,.92); border: 1px solid #33405a; color: var(--muted);
      font-size: 12px; padding: 6px 10px; border-radius: 6px; }
    #scene-status.error { border-color: #6b3640; color: var(--danger); }
    .legend { position: absolute; bottom: 8px; left: 12px; font-size: 11px;
      color: var(--muted); background: rgba(20,23,31,.85); padding: 4px 8px;
      border-radius: 4px; }
    #empty { padding: 10px 2px; color: var(--muted); font-size: 12px; }
    dialog.modal { width: min(360px, calc(100vw - 32px)); border: 1px solid #33405a;
      border-radius: 10px; padding: 0; background: #151a23; color: var(--fg);
      box-shadow: 0 18px 70px rgba(0,0,0,.55); }
    dialog.modal::backdrop { background: rgba(0,0,0,.55); }
    .modal-body { padding: 16px; display: grid; gap: 12px; }
    .modal-title { font-weight: 700; font-size: 14px; }
    .modal-message { color: var(--muted); font-size: 12px; line-height: 1.45; }
    .modal-input { width: 100%; box-sizing: border-box; border: 1px solid #33405a;
      border-radius: 7px; background: #0f131b; color: var(--fg); padding: 8px 10px;
      font-size: 13px; outline: none; }
    .modal-actions { display: flex; justify-content: flex-end; gap: 8px; }
    dialog.report-modal { width: min(720px, calc(100vw - 32px)); }
    .save-report-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .save-report-preview { width: 100%; max-height: 300px; object-fit: contain; border: 1px solid #33405a; border-radius: 8px; background: #0e1015; }
    .save-report-kv { display: grid; grid-template-columns: 120px 1fr; gap: 5px 10px; font-size: 12px; align-content: start; }
    .save-report-kv .k { color: var(--muted); }
    .save-report-kv .v { overflow-wrap: anywhere; }
    .save-report-log { margin: 8px 0 0; padding: 8px; border: 1px solid #283243; border-radius: 8px; background: #0f131b; color: #c9d1d9; font: 11px/1.45 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; white-space: pre-wrap; max-height: 130px; overflow: auto; }
    dialog.operation-modal { width: min(460px, calc(100vw - 32px)); }
    .operation-head { display: flex; align-items: center; gap: 10px; }
    .operation-spinner { width: 16px; height: 16px; border: 2px solid #33405a;
      border-top-color: var(--acc); border-radius: 50%; animation: operation-spin .8s linear infinite; }
    .operation-modal.finished .operation-spinner { animation: none; border-color: #3b633f;
      background: #8ef0b7; box-shadow: inset 0 0 0 4px #151a23; }
    .operation-modal.failed .operation-spinner { animation: none; border-color: var(--danger);
      background: var(--danger); box-shadow: inset 0 0 0 4px #151a23; }
    @keyframes operation-spin { to { transform: rotate(360deg); } }
    .operation-steps { display: grid; gap: 7px; margin: 2px 0; }
    .operation-step { display: grid; grid-template-columns: 16px 1fr; gap: 8px;
      align-items: center; color: var(--muted); font-size: 12px; }
    .operation-step::before { content: '○'; color: #526078; }
    .operation-step.active { color: var(--fg); }
    .operation-step.active::before { content: '●'; color: var(--acc); }
    .operation-step.done { color: #8ef0b7; }
    .operation-step.done::before { content: '✓'; }
    .operation-detail { margin: 0; padding: 8px; border: 1px solid #283243;
      border-radius: 8px; background: #0f131b; color: #c9d1d9;
      font: 11px/1.45 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      white-space: pre-wrap; max-height: 120px; overflow: auto; display: none; }
  </style>
</head>
<body data-ready="loading">
<div id="app">
  <header>
    <h1>Map &amp; rooms</h1>
    <span class="meta" id="meta">map: —</span>
    <span class="stale-alert" id="stale-alert">⚠ map was rebuilt — review stale rooms</span>
  </header>
  <div id="main">
    <div id="panel">
      <div id="map-tools">
        <label>Map ID<input id="map-id" placeholder="apartment_demo" /></label>
        <div id="map-actions">
          <button class="primary" id="btn-save-map">Save current</button>
          <button id="btn-refresh-maps">Refresh</button>
          <button id="btn-pose-estimate">Pose estimate</button>
        </div>
        <div id="map-status"><span id="map-status-msg">Ready.</span><span class="mode-label">Map mode:</span><span id="mode-pill">unknown</span></div>
        <div id="map-list"><div id="empty">No saved maps listed yet.</div></div>
      </div>
      <div class="actions">
        <button class="primary" id="btn-draw">✏ Annotate room</button>
      </div>
      <div id="room-list"><div id="empty">No rooms yet. Click “Annotate room”,
        then click on the map to outline one (double-click or Enter to finish,
        Esc to cancel).</div></div>
    </div>
    <div id="canvas-wrap">
      <canvas id="c"></canvas>
      <div id="hint"></div>
      <div id="toast"></div>
      <div id="scene-status" role="status">loading Scene state…</div>
      <div class="legend">drag to pan · wheel to zoom · hover a dot for its label</div>
    </div>
  </div>
</div>
<dialog class="modal" id="room-modal">
  <form method="dialog" class="modal-body">
    <div class="modal-title" id="room-modal-title">Room</div>
    <div class="modal-message" id="room-modal-message"></div>
    <input class="modal-input" id="room-modal-input" autocomplete="off" />
    <div class="modal-actions">
      <button id="room-modal-cancel" value="cancel">Cancel</button>
      <button class="primary" id="room-modal-ok" value="ok">OK</button>
    </div>
  </form>
</dialog>
<dialog class="modal report-modal" id="save-modal">
  <form method="dialog" class="modal-body">
    <div class="modal-title" id="save-modal-title">Save report</div>
    <div class="save-report-grid">
      <img class="save-report-preview" id="save-report-preview" alt="saved map preview" />
      <div class="save-report-kv" id="save-report-kv"></div>
    </div>
    <pre class="save-report-log" id="save-report-log"></pre>
    <div class="modal-actions">
      <button class="primary" value="ok">OK</button>
    </div>
  </form>
</dialog>
<dialog class="modal operation-modal" id="map-operation-modal">
  <div class="modal-body">
    <div class="operation-head">
      <span class="operation-spinner" aria-hidden="true"></span>
      <div class="modal-title" id="map-operation-title">Working...</div>
    </div>
    <div class="modal-message" id="map-operation-message"></div>
    <div class="operation-steps" id="map-operation-steps"></div>
    <pre class="operation-detail" id="map-operation-detail"></pre>
    <div class="modal-actions">
      <button id="map-operation-close" type="button">Close</button>
      <button class="primary" id="map-operation-primary" type="button" disabled>Working...</button>
    </div>
  </div>
</dialog>
<script>
const c = document.getElementById('c');
const ctx = c.getContext('2d');
function fit() { c.width = c.clientWidth; c.height = c.clientHeight; }
fit();

// ── view transform (world meters ↔ canvas px) ────────────────────────────
// Unlike the debug page (which chases the robot) the editor keeps a STABLE
// user-controlled view: drawing needs the map to hold still under the mouse.
let center = [0, 0];
let pxPerM = 40;
let viewInit = false;
function w2p(x, y) {
    return [c.width / 2 + (x - center[0]) * pxPerM,
            c.height / 2 - (y - center[1]) * pxPerM];
}
function p2w(px, py) {
    return [center[0] + (px - c.width / 2) / pxPerM,
            center[1] - (py - c.height / 2) / pxPerM];
}

// ── state ────────────────────────────────────────────────────────────────
let state = null;          // last /api/state payload
let occImg = null, occMeta = null, occStamp = 0, occLoading = 0;
let occLoadToken = 0;
let drawMode = false;
let draft = [];            // in-progress polygon vertices (world coords)
let mouseWorld = null;     // live cursor position for the rubber-band edge
let hoverObj = null;
let selectedId = null;
let selectedMapId = null;
let savedMaps = [];
let mapBusy = false;
let poseEstimateMode = false;
let draftSubmitting = false;
let mapOperationRetry = null;
let mapOperationDone = null;

window.addEventListener('resize', () => { fit(); draw(); });
document.addEventListener('visibilitychange', () => {
    if (!document.hidden) draw();
});

const hintEl = document.getElementById('hint');
const sceneStatusEl = document.getElementById('scene-status');
function setHint(text) {
    hintEl.style.display = text ? 'block' : 'none';
    hintEl.textContent = text || '';
}
function setSceneStatus(text, kind = '') {
    sceneStatusEl.textContent = text || '';
    sceneStatusEl.className = kind;
    sceneStatusEl.style.display = text ? 'block' : 'none';
}
function setSceneReady(value) { document.body.dataset.ready = value; }
function isCurrentOccupancyLoad(token, stamp) {
    return token === occLoadToken && stamp === occLoading;
}
let toastTimer = null;
function toast(text) {
    const t = document.getElementById('toast');
    t.textContent = text; t.style.display = 'block';
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { t.style.display = 'none'; }, 4000);
}
function setMapStatus(text, kind = '') {
    const el = document.getElementById('map-status');
    const msg = document.getElementById('map-status-msg');
    if (!el || !msg) return;
    el.className = kind;
    msg.textContent = text || 'Ready.';
}
function setMapBusy(on, label = '') {
    mapBusy = on;
    document.querySelectorAll('#map-tools button, #map-tools input, .map-btns button, #btn-draw, #room-list button')
        .forEach(el => el.disabled = on);
    if (!on) {
        renderMaps();
        renderPanel();
    }
    if (label) setMapStatus(label, on ? 'busy' : '');
}
const mapOperationModal = document.getElementById('map-operation-modal');
const mapOperationTitle = document.getElementById('map-operation-title');
const mapOperationMessage = document.getElementById('map-operation-message');
const mapOperationSteps = document.getElementById('map-operation-steps');
const mapOperationDetail = document.getElementById('map-operation-detail');
const mapOperationClose = document.getElementById('map-operation-close');
const mapOperationPrimary = document.getElementById('map-operation-primary');

function beginMapOperation({ title, message, status, steps }) {
    mapOperationRetry = null;
    mapOperationDone = null;
    mapOperationModal.classList.remove('finished', 'failed');
    mapOperationTitle.textContent = title;
    mapOperationMessage.textContent = message;
    mapOperationSteps.innerHTML = (steps || []).map((step, index) =>
        `<div class="operation-step ${index === 0 ? 'active' : ''}">${esc(step)}</div>`).join('');
    mapOperationDetail.textContent = '';
    mapOperationDetail.style.display = 'none';
    mapOperationClose.hidden = true;
    mapOperationPrimary.disabled = true;
    mapOperationPrimary.textContent = 'Working...';
    setMapBusy(true, status || message);
    if (!mapOperationModal.open) {
        if (typeof mapOperationModal.showModal === 'function') mapOperationModal.showModal();
        else mapOperationModal.setAttribute('open', '');
    }
}

function finishMapOperation({ ok, title, message, detail = '', retry = null, done = null, primaryText = '' }) {
    setMapBusy(false);
    mapOperationModal.classList.toggle('finished', ok);
    mapOperationModal.classList.toggle('failed', !ok);
    mapOperationTitle.textContent = title;
    mapOperationMessage.textContent = message;
    mapOperationSteps.querySelectorAll('.operation-step').forEach(step => {
        step.classList.remove('active');
        if (ok) step.classList.add('done');
    });
    mapOperationDetail.textContent = detail;
    mapOperationDetail.style.display = detail ? 'block' : 'none';
    mapOperationRetry = ok ? null : retry;
    mapOperationDone = done;
    mapOperationClose.hidden = ok;
    mapOperationPrimary.disabled = false;
    mapOperationPrimary.textContent = primaryText || (ok ? 'Done' : 'Retry');
}

mapOperationModal.addEventListener('cancel', ev => {
    if (mapBusy) ev.preventDefault();
});
mapOperationClose.addEventListener('click', () => {
    if (!mapBusy) mapOperationModal.close();
});
mapOperationPrimary.addEventListener('click', () => {
    if (mapBusy) return;
    if (mapOperationRetry) {
        const retry = mapOperationRetry;
        mapOperationRetry = null;
        retry();
        return;
    }
    const done = mapOperationDone;
    mapOperationDone = null;
    mapOperationModal.close();
    if (done) done();
});
function mapItemFor(id) {
    return Array.from(document.querySelectorAll('.mapitem')).find(el => el.dataset.id === id);
}
function setMapItemStatus(id, text) {
    const item = mapItemFor(id);
    if (!item) return;
    item.classList.toggle('busy', Boolean(text));
    const st = item.querySelector('.map-status');
    if (st) st.textContent = text || '';
}

// ── rendering ────────────────────────────────────────────────────────────
function polyCentroid(pts) {
    // Area-weighted polygon centroid. Averaging vertices visibly shifts labels
    // for irregular rooms because densely sampled edges receive extra weight.
    let twiceArea = 0, sx = 0, sy = 0;
    for (let i = 0; i < pts.length; i++) {
        const [x0, y0] = pts[i];
        const [x1, y1] = pts[(i + 1) % pts.length];
        const cross = x0 * y1 - x1 * y0;
        twiceArea += cross;
        sx += (x0 + x1) * cross;
        sy += (y0 + y1) * cross;
    }
    if (Math.abs(twiceArea) < 1e-9) {
        const sum = pts.reduce(([x, y], p) => [x + p[0], y + p[1]], [0, 0]);
        return [sum[0] / pts.length, sum[1] / pts.length];
    }
    return [sx / (3 * twiceArea), sy / (3 * twiceArea)];
}

const ROOM_COLORS = [
    { stroke:'#63a7ff', fill:'rgba(99,167,255,.18)', label:'#d8e9ff' },
    { stroke:'#5fd6a7', fill:'rgba(95,214,167,.18)', label:'#d2f8e9' },
    { stroke:'#f0aa5d', fill:'rgba(240,170,93,.18)', label:'#ffe7cd' },
    { stroke:'#c58cff', fill:'rgba(197,140,255,.18)', label:'#eedcff' },
    { stroke:'#ef7f9c', fill:'rgba(239,127,156,.18)', label:'#ffd9e3' },
    { stroke:'#55c8dd', fill:'rgba(85,200,221,.18)', label:'#d4f7fc' },
    { stroke:'#d5c456', fill:'rgba(213,196,86,.18)', label:'#fff6bd' },
    { stroke:'#8fcf66', fill:'rgba(143,207,102,.18)', label:'#e3f8d5' },
];
function roomColor(annotation) {
    const key = String(annotation.annotation_id || annotation.name || 'room');
    let hash = 2166136261;
    for (let i = 0; i < key.length; i++) {
        hash ^= key.charCodeAt(i);
        hash = Math.imul(hash, 16777619);
    }
    return ROOM_COLORS[(hash >>> 0) % ROOM_COLORS.length];
}

function draw() {
    fit();
    ctx.clearRect(0, 0, c.width, c.height);
    if (!state) return;

    // occupancy underlay (same decode/anchor math as the debug 2D page:
    // cell [0,0] at world origin, y flipped because image y grows down).
    // occStamp only advances on successful decode, so a corrupt frame is
    // retried on the next poll instead of wedging the underlay.
    if (state.occupancy && state.occupancy.stamp_ms !== occStamp
        && state.occupancy.stamp_ms !== occLoading) {
        occLoading = state.occupancy.stamp_ms;
        const loadToken = ++occLoadToken;
        const meta = state.occupancy;
        const im = new Image();
        im.onload = () => {
            if (!isCurrentOccupancyLoad(loadToken, meta.stamp_ms)) return;
            occImg = im;
            occMeta = meta;
            occStamp = meta.stamp_ms;
            occLoading = 0;
            draw();
        };
        im.onerror = () => {
            if (!isCurrentOccupancyLoad(loadToken, meta.stamp_ms)) return;
            occLoading = 0;
            setSceneReady('error');
            setSceneStatus('Scene map image could not be decoded; retrying…', 'error');
            console.error('occupancy PNG decode failed');
        };
        im.src = 'data:image/png;base64,' + meta.png_b64;
    }
    if (occImg && occMeta) {
        if (!viewInit) {   // first grid: fit it into the viewport once
            const wM = occMeta.width * occMeta.resolution;
            const hM = occMeta.height * occMeta.resolution;
            center = [occMeta.origin_x + wM / 2, occMeta.origin_y + hM / 2];
            pxPerM = Math.min((c.width - 40) / wM, (c.height - 40) / hM, 120);
            pxPerM = Math.max(pxPerM, 5);
            viewInit = true;
        }
        const wM = occMeta.width * occMeta.resolution;
        const hM = occMeta.height * occMeta.resolution;
        const [x0, y0] = w2p(occMeta.origin_x, occMeta.origin_y + hM);
        ctx.globalAlpha = 0.9;
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(occImg, x0, y0, wM * pxPerM, hM * pxPerM);
        ctx.globalAlpha = 1.0;
    } else {
        ctx.fillStyle = '#7d828b'; ctx.font = '13px sans-serif';
        ctx.fillText('no map yet — the SLAM map appears here once mapping publishes it', 20, 30);
    }

    // saved rooms
    for (const a of (state.annotations || [])) {
        if (a.kind !== 'room' || a.points.length < 3) continue;
        const pts = a.points.map(p => w2p(p[0], p[1]));
        ctx.beginPath();
        pts.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y));
        ctx.closePath();
        const sel = a.annotation_id === selectedId;
        const color = roomColor(a);
        ctx.fillStyle = a.stale ? 'rgba(230,196,84,0.12)' : color.fill;
        ctx.fill();
        ctx.lineWidth = sel ? 2.5 : 1.5;
        ctx.strokeStyle = a.stale ? '#e6c454' : color.stroke;
        ctx.setLineDash(a.stale ? [6, 4] : []);
        ctx.stroke();
        ctx.setLineDash([]);
        const [cx, cy] = w2p(...polyCentroid(a.points));
        const label = (a.stale ? '⚠ ' : '') + (a.name || '(unnamed)');
        ctx.font = '700 15px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.lineWidth = 5; ctx.strokeStyle = 'rgba(0,0,0,0.92)';
        ctx.strokeText(label, cx, cy);
        ctx.fillStyle = a.stale ? '#f4dc78' : color.label;
        ctx.fillText(label, cx, cy);
        ctx.textAlign = 'start';
    }

    // objects — small translucent dots; label only on hover (readability:
    // the underlay must stay legible, objects are a light overlay).
    for (const o of (state.objects || [])) {
        if (o.cls === 'robot') continue;
        const [px, py] = w2p(o.pose.x, o.pose.y);
        ctx.globalAlpha = o.missing ? 0.25 : 0.6;
        ctx.fillStyle = '#9ab8e8';
        ctx.beginPath(); ctx.arc(px, py, 3.5, 0, Math.PI * 2); ctx.fill();
        ctx.globalAlpha = 1;
        ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(14,16,21,0.9)';
        ctx.stroke();
    }
    if (hoverObj) {
        const [px, py] = w2p(hoverObj.pose.x, hoverObj.pose.y);
        ctx.font = '12px ui-monospace, monospace'; ctx.textBaseline = 'middle';
        ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(0,0,0,0.85)';
        ctx.strokeText(hoverObj.cls, px + 8, py);
        ctx.fillStyle = '#cfe0ff';
        ctx.fillText(hoverObj.cls, px + 8, py);
    }

    // Robot marker: fixed-size, high-contrast body plus a long directional
    // nose. Keeping it in screen pixels makes the pose readable at every map
    // zoom level, including over dark occupied cells and pale free space.
    const robot = state.robot;
    if (robot) {
        const [rx, ry] = w2p(robot.x, robot.y);
        const yaw = robot.yaw || 0;
        const robotMarkerNose = 24;
        ctx.save();
        ctx.translate(rx, ry);
        ctx.rotate(-yaw);

        ctx.beginPath();
        ctx.arc(0, 0, 12, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(7, 10, 16, 0.92)';
        ctx.fill();
        ctx.lineWidth = 4;
        ctx.strokeStyle = '#ffffff';
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(robotMarkerNose, 0);
        ctx.lineTo(-7, -10);
        ctx.lineTo(-3, 0);
        ctx.lineTo(-7, 10);
        ctx.closePath();
        ctx.fillStyle = '#ff5a1f';
        ctx.fill();
        ctx.lineWidth = 3;
        ctx.strokeStyle = '#ffffff';
        ctx.stroke();

        ctx.beginPath();
        ctx.arc(0, 0, 4, 0, Math.PI * 2);
        ctx.fillStyle = '#ffffff';
        ctx.fill();
        ctx.restore();
    }

    // draft polygon (draw mode)
    if (drawMode && draft.length) {
        const pts = draft.map(p => w2p(p[0], p[1]));
        ctx.beginPath();
        pts.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y));
        if (mouseWorld) { const [mx, my] = w2p(...mouseWorld); ctx.lineTo(mx, my); }
        ctx.strokeStyle = '#8ef0b7'; ctx.lineWidth = 2; ctx.setLineDash([5, 4]);
        ctx.stroke(); ctx.setLineDash([]);
        ctx.fillStyle = '#8ef0b7';
        for (const [x, y] of pts) {
            ctx.beginPath(); ctx.arc(x, y, 3, 0, Math.PI * 2); ctx.fill();
        }
    }

    const canvasVisible = c.clientWidth > 0 && c.clientHeight > 0;
    if (canvasVisible
        && (!state.occupancy || (occImg && occStamp === state.occupancy.stamp_ms))) {
        setSceneReady('true');
        setSceneStatus('');
    } else {
        setSceneReady('loading');
        setSceneStatus(canvasVisible ? 'loading occupancy map…' : 'waiting for visible canvas…');
    }
}

function showSaveReport(out, previewDataUrl) {
    const validation = (out && out.validation) || {};
    const modal = document.getElementById('save-modal');
    const title = document.getElementById('save-modal-title');
    const img = document.getElementById('save-report-preview');
    const kv = document.getElementById('save-report-kv');
    const log = document.getElementById('save-report-log');
    const status = validation.spatial_ok ? 'OK' : 'FAILED';
    title.textContent = `Save report · ${status}`;
    img.src = previewDataUrl || '';
    const paths = validation.paths || {};
    const rows = [
        ['Map ID', validation.map_id || out.map_id || '-'],
        ['Spatial artifact', validation.spatial_ok ? 'ok' : (validation.artifact_detail || 'failed')],
        ['Artifact size', validation.artifact_size ? `${validation.artifact_size} bytes` : '-'],
        ['Objects', validation.object_count ?? out.objects ?? '-'],
        ['Rooms', validation.room_count ?? '-'],
        ['Annotations', validation.annotation_count ?? out.annotations ?? '-'],
        ['Updated', validation.updated || '-'],
        ['Artifact path', paths.map_artifact || '-'],
        ['Preview path', paths.preview || '-'],
        ['Room JSON', paths.scene_annotations || '-'],
    ];
    kv.innerHTML = rows.map(([k, v]) => `<div class="k">${esc(k)}</div><div class="v">${esc(v)}</div>`).join('');
    log.textContent = (validation.log || []).join('\n') || 'no save log returned';
    if (typeof modal.showModal === 'function') modal.showModal();
    else modal.setAttribute('open', '');
}

// ── map library ──────────────────────────────────────────────────────────
function preferredMapId() {
    const input = document.getElementById('map-id');
    const typed = input && input.value.trim();
    const bound = state && state.map_binding && state.map_binding.map_id;
    return typed || bound || 'default';
}
function renderMaps() {
    const list = document.getElementById('map-list');
    if (!list) return;
    if (!savedMaps.length) {
        list.innerHTML = '<div id="empty">No saved maps listed yet.</div>';
        return;
    }
    list.innerHTML = savedMaps.map(m => {
        const dbLabel = !m.has_spatial_artifact ? 'no spatial artifact' : (m.spatial_ok === false ? 'invalid artifact' : 'spatial artifact');
        const dbTitle = m.artifact_detail ? ` title="${esc(m.artifact_detail)}"` : '';
        const canLoad = m.has_spatial_artifact && m.spatial_ok !== false;
        return `
      <div class="mapitem ${m.map_id === selectedMapId ? 'selected' : ''} ${canLoad ? '' : 'bad'}" data-id="${esc(m.map_id)}">
        <div class="name">${esc(m.map_id)}</div>
        <div class="sub"${dbTitle}>${dbLabel} · ${m.has_preview ? 'preview' : 'no preview'} · ${m.updated || '-'}</div>
        <div class="map-btns">
          <button class="small" data-map-act="load" ${canLoad && !mapBusy ? '' : 'disabled'}>Load</button>
          <button class="small danger" data-map-act="delete" ${mapBusy ? 'disabled' : ''}>Delete</button>
        </div>
        <div class="map-status">${canLoad ? '' : esc(m.artifact_detail || 'not loadable')}</div>
      </div>`;
    }).join('');
}
async function loadMaps() {
    try {
        const r = await fetch('/api/maps', { cache: 'no-store' });
        const out = await r.json();
        if (!r.ok || !out.ok) { toast(out.detail || 'list maps failed'); return; }
        savedMaps = out.maps || [];
        renderMaps();
    } catch (e) { toast('list maps failed: ' + e); }
}
async function mapRequest(path, body) {
    try {
        const response = await fetch(path, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(body),
        });
        const out = await response.json().catch(() => null);
        if (!response.ok || !out || out.ok === false) {
            return {
                ok: false,
                out,
                detail: (out && out.detail) || `request failed (${response.status})`,
            };
        }
        await refresh();
        return { ok: true, out, detail: out.detail || '' };
    } catch (error) {
        return { ok: false, out: null, detail: `request failed: ${error}` };
    }
}
async function saveCurrentMap() {
    if (mapBusy) return;
    const id = preferredMapId();
    document.getElementById('map-id').value = id;
    selectedMapId = id; renderMaps();
    draw();
    const previewDataUrl = c.toDataURL('image/png');
    const existing = savedMaps.find(item => item.map_id === id && item.has_spatial_artifact);
    beginMapOperation({
        title: existing ? `Updating ${id}` : `Saving ${id}`,
        message: existing
            ? 'The spatial map already exists. Scene rooms and objects will be updated under the same Map ID.'
            : 'Keep this page open while the spatial map and Scene data are made reusable.',
        status: existing ? `Updating Scene data for ${id}...` : `Saving map ${id}...`,
        steps: existing
            ? ['Validate existing spatial artifact', 'Persist rooms and Scene objects', 'Verify reusable map entry']
            : ['Snapshot the live spatial map', 'Persist rooms and Scene objects', 'Verify artifact and preview'],
    });
    const result = await mapRequest('/api/maps/save', { map_id: id, note: 'saved from scene user page' });
    if (result.ok) {
        const out = result.out;
        const validation = out.validation || {};
        const ok = validation.spatial_ok !== false && validation.has_preview !== false;
        const semanticOnly = Boolean(out.spatial_unchanged);
        setMapStatus(`${semanticOnly ? 'Updated scene data for' : 'Saved'} ${out.map_id || id}; spatial artifact ${ok ? 'ok' : 'failed'}; rooms ${validation.room_count ?? '-'}.`, ok ? 'ok' : 'err');
        toast((semanticOnly ? 'updated scene data for ' : 'saved map ') + (out.map_id || id));
        await loadMaps();
        finishMapOperation({
            ok,
            title: ok ? (semanticOnly ? 'Scene data updated' : 'Map saved') : 'Save validation failed',
            message: ok
                ? `${out.map_id || id} is ready to load after a stack restart.`
                : `${out.map_id || id} was written, but artifact validation did not pass.`,
            detail: (validation.log || []).join('\n') || out.detail || '',
            retry: ok ? null : saveCurrentMap,
            done: ok ? () => showSaveReport(out, previewDataUrl) : null,
            primaryText: ok ? 'View report' : 'Retry',
        });
    }
    else {
        setMapStatus(`Save ${id} failed: ${result.detail}`, 'err');
        finishMapOperation({
            ok: false,
            title: 'Save failed',
            message: `Nothing was confirmed reusable for Map ID ${id}.`,
            detail: result.detail,
            retry: saveCurrentMap,
        });
    }
}
async function loadSelectedMap(id) {
    if (mapBusy) return;
    selectedMapId = id;
    document.getElementById('map-id').value = id;
    renderMaps();
    setMapItemStatus(id, 'loading...');
    beginMapOperation({
        title: `Loading ${id}`,
        message: 'The editor is locked until Mapping publishes the loaded occupancy grid and Scene restores the matching semantic data.',
        status: `Loading ${id}; switching Mapping to localization mode...`,
        steps: ['Validate saved spatial artifact', 'Switch Mapping to localization mode', 'Wait for a fresh occupancy grid', 'Restore rooms and Scene objects'],
    });
    const result = await mapRequest('/api/maps/load', { map_id: id, mode: 'localization' });
    setMapItemStatus(id, '');
    if (result.ok) {
        const out = result.out;
        setMapStatus(`Loaded ${id}. Mapping requested localization mode; use Pose estimate if the robot pose is off.`, 'ok');
        toast('loaded map ' + id + ' · localization mode requested');
        await loadMaps();
        const occupancy = out.occupancy || {};
        finishMapOperation({
            ok: true,
            title: 'Map loaded',
            message: `${id} is active in localization mode. Rooms and Scene objects now use the same Map ID.`,
            detail: [
                out.detail || '',
                occupancy.width ? `occupancy ${occupancy.width} × ${occupancy.height}` : '',
                Number.isFinite(Number(out.objects_restored)) ? `objects restored: ${out.objects_restored}` : '',
                out.object_restore_error ? `object restore warning: ${out.object_restore_error}` : '',
            ].filter(Boolean).join('\n'),
        });
    } else {
        setMapStatus(`Load ${id} failed: ${result.detail}`, 'err');
        finishMapOperation({
            ok: false,
            title: 'Load failed',
            message: `${id} was not confirmed ready. The editor has not switched its Scene binding.`,
            detail: result.detail,
            retry: () => loadSelectedMap(id),
        });
    }
}
async function deleteSelectedMap(id) {
    if (mapBusy) return;
    if (!(await askConfirm('Delete map', `Delete map “${id}”?`))) return;
    setMapItemStatus(id, 'deleting...');
    setMapBusy(true, `Deleting map ${id}...`);
    const out = await api('POST', '/api/maps/delete', { map_id: id });
    setMapBusy(false);
    if (out) { setMapStatus(`Deleted ${id}.`, 'ok'); toast('deleted map ' + id); await loadMaps(); }
    else { setMapItemStatus(id, ''); setMapStatus(`Delete ${id} failed.`, 'err'); }
}
function setPoseEstimateMode(on) {
    poseEstimateMode = on;
    const btn = document.getElementById('btn-pose-estimate');
    if (btn) {
        btn.classList.toggle('primary', on);
        btn.textContent = on ? 'Click pose on map' : 'Pose estimate';
    }
    setHint(on ? 'Click the robot position on the map to publish /initialpose. Heading defaults to 0; refine later with orientation UI.' : '');
    c.style.cursor = on ? 'crosshair' : (drawMode ? 'crosshair' : 'grab');
}
function poseDeltaText(delta) {
    if (!delta) return 'pose feedback unavailable';
    return `error ${Number(delta.distance_m || 0).toFixed(2)} m, yaw ${Number(delta.abs_yaw_rad || 0).toFixed(2)} rad`;
}
async function sendPoseEstimate(world) {
    if (mapBusy || !world) return;
    setPoseEstimateMode(false);
    const x = world[0], y = world[1], theta = 0.0;
    setMapBusy(true, `Publishing pose estimate (${x.toFixed(2)}, ${y.toFixed(2)}, ${theta.toFixed(2)})...`);
    toast(`pose estimate: ${x.toFixed(2)}, ${y.toFixed(2)}`);
    const out = await api('POST', '/api/maps/pose_estimate', { x, y, theta });
    setMapBusy(false);
    if (out) {
        const msg = `${out.detail || 'Pose estimate sent.'}` +
            (out.delta_before ? ` · before ${poseDeltaText(out.delta_before)}` : '') +
            (out.delta_after ? ` · after ${poseDeltaText(out.delta_after)}` : '');
        setMapStatus(msg, 'ok');
        toast(out.detail || 'pose estimate sent');
    } else setMapStatus(`Pose estimate failed for (${x.toFixed(2)}, ${y.toFixed(2)}).`, 'err');
}
document.getElementById('btn-save-map').addEventListener('click', saveCurrentMap);
document.getElementById('btn-refresh-maps').addEventListener('click', async () => { setMapStatus('Refreshing maps...', 'busy'); await loadMaps(); setMapStatus('Map list refreshed.', 'ok'); });
document.getElementById('btn-pose-estimate').addEventListener('click', () => setPoseEstimateMode(!poseEstimateMode));
document.getElementById('map-list').addEventListener('click', (ev) => {
    if (mapBusy) return;
    const item = ev.target.closest('.mapitem');
    if (!item) return;
    const id = item.dataset.id;
    const act = ev.target.dataset && ev.target.dataset.mapAct;
    if (!act) {
        selectedMapId = id;
        document.getElementById('map-id').value = id;
        renderMaps();
        setMapStatus(`Selected ${id}. Click Load to enter localization mode.`, 'ok');
        return;
    }
    if (act === 'load') {
        const m = savedMaps.find(x => x.map_id === id);
        if (m && (!m.has_spatial_artifact || m.spatial_ok === false)) {
            setMapStatus(`Map ${id} is not loadable: ${m.artifact_detail || 'invalid spatial artifact'}`, 'err');
            return;
        }
        loadSelectedMap(id);
    }
    if (act === 'delete') deleteSelectedMap(id);
});

// ── side panel ───────────────────────────────────────────────────────────
function esc(s) {
    return String(s).replace(/[&<>"']/g,
        ch => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));
}

const roomModal = document.getElementById('room-modal');
const roomModalTitle = document.getElementById('room-modal-title');
const roomModalMessage = document.getElementById('room-modal-message');
const roomModalInput = document.getElementById('room-modal-input');
const roomModalOk = document.getElementById('room-modal-ok');
const roomModalCancel = document.getElementById('room-modal-cancel');

function askModal({ title, message = '', defaultValue = '', input = true, okText = 'OK', danger = false }) {
    return new Promise((resolve) => {
        roomModalTitle.textContent = title;
        roomModalMessage.textContent = message;
        roomModalMessage.style.display = message ? 'block' : 'none';
        roomModalInput.value = defaultValue || '';
        roomModalInput.style.display = input ? 'block' : 'none';
        roomModalOk.textContent = okText;
        roomModalOk.classList.toggle('danger', danger);
        let done = false;
        const finish = (value) => {
            if (done) return;
            done = true;
            roomModal.removeEventListener('close', onClose);
            resolve(value);
        };
        const onClose = () => {
            if (roomModal.returnValue === 'ok') {
                finish(input ? roomModalInput.value.trim() : true);
            } else {
                finish(null);
            }
        };
        roomModal.addEventListener('close', onClose, { once: true });
        if (typeof roomModal.showModal === 'function') roomModal.showModal();
        else roomModal.setAttribute('open', '');
        if (input) {
            roomModalInput.focus();
            roomModalInput.select();
        } else {
            roomModalOk.focus();
        }
    });
}

function askRoomName(title, defaultValue = '') {
    return askModal({ title, defaultValue, input: true, okText: 'Save' });
}

function askConfirm(title, message, okText = 'Delete') {
    return askModal({ title, message, input: false, okText, danger: true });
}
function renderPanel() {
    const rooms = (state && state.annotations || []).filter(a => a.kind === 'room');
    const list = document.getElementById('room-list');
    const anyStale = rooms.some(a => a.stale);
    document.getElementById('stale-alert').style.display = anyStale ? 'inline' : 'none';
    if (!rooms.length) {
        list.innerHTML = '<div id="empty">No rooms yet. Click “Annotate room”, ' +
          'then click on the map to outline one (double-click or Enter to ' +
          'finish, Esc to cancel).</div>';
        return;
    }
    list.innerHTML = rooms.map(a => `
      <div class="room ${a.annotation_id === selectedId ? 'selected' : ''}"
           data-id="${a.annotation_id}" style="--room-color:${roomColor(a).stroke}">
        <span class="swatch" aria-hidden="true"></span><span class="name">${esc(a.name || '(unnamed)')}</span>
        ${a.stale ? '<span class="badge" title="' + esc(a.stale_reason) + '">STALE</span>' : ''}
        <div class="sub">${a.points.length} corners</div>
        <div class="btns">
          <button class="small" data-act="rename" ${mapBusy ? 'disabled' : ''}>Rename</button>
          ${a.stale ? `<button class="small" data-act="confirm" ${mapBusy ? 'disabled' : ''}>Still valid</button>` : ''}
          <button class="small danger" data-act="delete" ${mapBusy ? 'disabled' : ''}>Delete</button>
        </div>
      </div>`).join('');
}
document.getElementById('room-list').addEventListener('click', async (ev) => {
    if (mapBusy) return;
    const roomEl = ev.target.closest('.room');
    if (!roomEl) return;
    const id = roomEl.dataset.id;
    const act = ev.target.dataset && ev.target.dataset.act;
    if (!act) { selectedId = (selectedId === id) ? null : id; renderPanel(); draw(); return; }
    const room = (state.annotations || []).find(a => a.annotation_id === id);
    if (!room) return;
    if (act === 'rename') {
        const name = await askRoomName('Rename room', room.name);
        if (name !== null && name) await api('PUT', '/api/annotations/' + id, { name });
    } else if (act === 'confirm') {
        await api('PUT', '/api/annotations/' + id, { stale: false });
    } else if (act === 'delete') {
        if (await askConfirm('Delete room', `Delete room “${room.name}”?`))
            await api('DELETE', '/api/annotations/' + id);
    }
});

// ── draw mode ────────────────────────────────────────────────────────────
const btnDraw = document.getElementById('btn-draw');
function setDrawMode(on) {
    drawMode = on;
    draft = [];
    btnDraw.classList.toggle('active', on);
    btnDraw.textContent = on ? '✕ Cancel drawing' : '✏ Annotate room';
    c.style.cursor = on ? 'crosshair' : 'grab';
    setHint(on ? 'Click to add corners · double-click or Enter to finish (≥3) · Esc to cancel' : '');
    draw();
}
btnDraw.addEventListener('click', () => setDrawMode(!drawMode));

async function finishDraft() {
    if (mapBusy || draftSubmitting) return;
    if (draft.length < 3) { toast('A room needs at least 3 corners.'); return; }
    draftSubmitting = true;
    try {
        const points = draft.map(p => [p[0], p[1]]);
        const name = await askRoomName('Create room', '');
        if (name === null) return;          // keep drawing
        if (!name) { toast('Room name is required.'); return; }
        const body = { kind: 'room', name, points };
        draft = [];
        const created = await api('POST', '/api/annotations', body);
        if (created) setDrawMode(false);
        else { draft = points; draw(); }
    } finally {
        draftSubmitting = false;
    }
}

// ── canvas input: pan / zoom / vertex clicks / hover ─────────────────────
let dragging = false, dragMoved = false, lastPos = null;
c.style.cursor = 'grab';
c.addEventListener('mousedown', (ev) => {
    if (mapBusy) return;
    dragging = true; dragMoved = false; lastPos = [ev.offsetX, ev.offsetY];
});
window.addEventListener('mouseup', () => { dragging = false; });
c.addEventListener('mousemove', (ev) => {
    mouseWorld = p2w(ev.offsetX, ev.offsetY);
    if (dragging && lastPos) {
        const dx = ev.offsetX - lastPos[0], dy = ev.offsetY - lastPos[1];
        if (Math.abs(dx) + Math.abs(dy) > 3) dragMoved = true;
        if (dragMoved) {
            center = [center[0] - dx / pxPerM, center[1] + dy / pxPerM];
            lastPos = [ev.offsetX, ev.offsetY];
        }
    } else if (!drawMode && state) {
        hoverObj = null;
        for (const o of (state.objects || [])) {
            if (o.cls === 'robot') continue;
            const [px, py] = w2p(o.pose.x, o.pose.y);
            if ((px - ev.offsetX) ** 2 + (py - ev.offsetY) ** 2 < 100) { hoverObj = o; break; }
        }
    }
    draw();
});
c.addEventListener('click', (ev) => {
    if (mapBusy) return;
    if (dragMoved) return;              // that was a pan, not a click
    if (poseEstimateMode) { sendPoseEstimate(p2w(ev.offsetX, ev.offsetY)); return; }
    if (drawMode) { draft.push(p2w(ev.offsetX, ev.offsetY)); draw(); }
});
c.addEventListener('dblclick', (ev) => {
    if (mapBusy) return;
    ev.preventDefault();
    if (drawMode) {
        // the dblclick's two single clicks added two duplicate corners at
        // the same spot — drop one before closing.
        if (draft.length > 1) draft.pop();
        finishDraft();
    }
});
window.addEventListener('keydown', (ev) => {
    if (mapBusy) return;
    if (!drawMode) return;
    if (ev.key === 'Escape') setDrawMode(false);
    if (ev.key === 'Enter' && !ev.repeat) finishDraft();
});
c.addEventListener('wheel', (ev) => {
    ev.preventDefault();
    if (mapBusy) return;
    const factor = ev.deltaY < 0 ? 1.15 : 1 / 1.15;
    // zoom about the cursor so the point under the mouse stays put
    const [wx, wy] = p2w(ev.offsetX, ev.offsetY);
    pxPerM = Math.min(400, Math.max(3, pxPerM * factor));
    const [nx, ny] = p2w(ev.offsetX, ev.offsetY);
    center = [center[0] + (wx - nx), center[1] + (wy - ny)];
    draw();
}, { passive: false });

// ── API + polling ────────────────────────────────────────────────────────
async function api(method, path, body) {
    try {
        const r = await fetch(path, {
            method,
            headers: body !== undefined ? { 'Content-Type': 'application/json' } : {},
            body: body !== undefined ? JSON.stringify(body) : undefined,
        });
        const out = await r.json().catch(() => null);
        if (!r.ok || !out || out.ok === false) {
            toast((out && out.detail) || (method + ' failed (' + r.status + ')'));
            return null;
        }
        await refresh();
        return out;
    } catch (e) { toast('request failed: ' + e); return null; }
}

async function refresh() {
    // Only the network fetch/parse is try-guarded (transient by nature);
    // a bug thrown by the render functions must surface in the console,
    // not be swallowed once a second forever.
    let next = null;
    try {
        const r = await fetch('/api/state', { cache: 'no-store' });
        if (!r.ok) {
            setSceneReady('error');
            setSceneStatus(`Scene state unavailable (HTTP ${r.status}); retrying…`, 'error');
            return;
        }
        next = await r.json();
    } catch (error) {
        setSceneReady('error');
        setSceneStatus(`Scene state unavailable; retrying… (${error})`, 'error');
        return;
    }
    state = next;
    const mb = state.map_binding;
    const unsavedLive = mb && mb.source === 'default' && !mb.mode;
    document.getElementById('meta').textContent = mb
        ? (unsavedLive ? 'map: live session · unsaved' : `map: ${mb.map_id} · ${mb.mode || 'mode unknown'}`)
        : 'map: —';
    const modePill = document.getElementById('mode-pill');
    if (modePill) {
        const mode = unsavedLive ? 'unsaved live' : ((mb && mb.mode) || 'unknown');
        modePill.textContent = mode;
        modePill.className = unsavedLive ? 'mapping' : mode;
    }
    const msg = document.getElementById('map-status-msg');
    if (unsavedLive && msg && (msg.textContent === 'Ready.' || msg.textContent === 'Map list refreshed.')) {
        setMapStatus('Live mapping session is not saved yet. Enter a Map ID, then Save current.', 'busy');
    } else if (mb && mb.mode === 'localization' && msg && (msg.textContent === 'Ready.' || msg.textContent === 'Map list refreshed.')) {
        setMapStatus('Localization mode is active. Use Pose estimate if the robot pose is off.', 'ok');
    }
    const mapInput = document.getElementById('map-id');
    if (mapInput && !mapInput.value && mb && mb.map_id && !unsavedLive) mapInput.value = mb.map_id;
    renderPanel();
    draw();
}
// 1 Hz is plenty for an editor page (the debug page polls at 5 Hz).
setInterval(refresh, 1000);
refresh();
loadMaps();
</script>
</body>
</html>
"""
