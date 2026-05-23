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

import base64
import io
import logging
import time
from pathlib import Path
from typing import Any, Optional

from starlette.applications import Starlette
from starlette.responses import HTMLResponse, JSONResponse
from starlette.routing import Mount, Route
from starlette.staticfiles import StaticFiles

from .state import ObjectRegistry, RelationEngine

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
  </style>
</head>
<body>
<div id="wrap">
  <div id="canvas-wrap">
    <canvas id="c"></canvas>
    <div class="legend">scene · 1 m grid · north = +x · 5 Hz</div>
  </div>
</div>
<script>
const c = document.getElementById('c');
const ctx = c.getContext('2d');
function fit() { c.width = c.clientWidth; c.height = c.clientHeight; }
window.addEventListener('resize', fit); fit();

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

function draw(state) {
    fit();
    ctx.clearRect(0, 0, c.width, c.height);

    // re-center on the robot if there is one; fall back to last-known.
    const robot = (state.objects || []).find(o => o.cls === 'robot');
    if (robot) center = [robot.pose.x, robot.pose.y];

    // ── Occupancy map underlay ──────────────────────────────────────
    if (state.occupancy && state.occupancy.stamp_ms !== occStamp) {
        occStamp = state.occupancy.stamp_ms;
        occMeta = state.occupancy;
        const im = new Image();
        im.onload = () => { occImg = im; };
        im.src = 'data:image/png;base64,' + state.occupancy.png_b64;
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
        // Skip missing objects on the live canvas — the operator-facing
        // complaint was "I walked a phone through frame and it stayed
        // there". Fading to 30% alpha (previous behaviour) still looks
        // "on the map" to a person; hard-hide matches the intent.
        // Stale records still surface in the side table styled `miss`,
        // so they aren't silently lost from the registry view either.
        if (o.missing) continue;
        const [px, py] = w2p(o.pose.x, o.pose.y);
        const r = Math.max(4, Math.min(20, (o.bbox.size_x || 0.2) * pxPerM * 0.5));
        const color = classColor(o.cls);
        ctx.globalAlpha = 1.0;
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
}

function fmt(n) { return Number(n).toFixed(2); }

async function tick() {
    try {
        const r = await fetch('/api/state', { cache: 'no-store' });
        if (!r.ok) return;
        const state = await r.json();
        draw(state);
        // Info panel was moved to a top-level floating overlay in
        // _COMBINED_HTML; this iframe no longer has any DOM for it.
    } catch (_) { /* swallow; next tick will retry */ }
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


def _camera_payload(hub: Any) -> dict:
    """JSON for the /cam panel: latest RGB + depth as base64 PNGs."""
    out: dict[str, Any] = {"rgb": None, "depth": None}
    if hub is None:
        return out
    if hub.has("rgb"):
        msg, stamp_unix, count = hub.latest("rgb")
        if msg is not None and count > 0:
            enc = _image_to_png_b64(msg, kind="rgb")
            if enc is not None:
                # Prefer rclpy stamp; fall back to ingest-time if header is unset.
                if enc["stamp_ms"] == 0:
                    enc["stamp_ms"] = int(stamp_unix * 1000)
                out["rgb"] = enc
    if hub.has("depth"):
        msg, stamp_unix, count = hub.latest("depth")
        if msg is not None and count > 0:
            enc = _image_to_png_b64(msg, kind="depth")
            if enc is not None:
                if enc["stamp_ms"] == 0:
                    enc["stamp_ms"] = int(stamp_unix * 1000)
                out["depth"] = enc
    return out


def _state_payload(registry: ObjectRegistry, relations: RelationEngine,
                   hub: Any) -> dict:
    """Serialise the registry + relations + map into the small JSON
    shape the page consumes. Done in one snapshot so the page never
    sees a half-updated registry."""
    objs_dict, _surfaces = _sync_snapshot(registry)
    rels = relations.current()
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
        {"subject": r.subject_object_id, "predicate": r.predicate, "target": r.target_object_id}
        for r in rels
    ]
    return {
        "objects": out_objects,
        "relations": out_relations,
        "robot": robot_pose,
        "occupancy": _occupancy_payload(hub),
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


def make_app(*, registry: ObjectRegistry, relations: RelationEngine,
             hub: Any = None, detector: Any = None) -> Starlette:
    """Build the Starlette ASGI app the entrypoint mounts on its own
    uvicorn server.

    Routes:
      GET /                — 2D top-down map (occupancy grid + objects)
      GET /3d              — 3D scene (point clouds + bbox; three.js)
      GET /api/state       — JSON for the 2D map
      GET /api/objects3d   — JSON for the 3D viz (per-object pcd + bbox)

    `hub` is the SubscribersHub — passed so the JSON state can include
    the latest OccupancyGrid for the 2D canvas underlay.
    `detector` is the ConceptGraphsDetector — passed so the 3D endpoint
    can serialize its persistent MapObjectList. If None, the 3D page
    just shows an empty world.
    """

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
        return JSONResponse(_state_payload(registry, relations, hub))

    async def index3d(_request) -> HTMLResponse:
        return HTMLResponse(_INDEX_3D_HTML)

    async def objects3d(_request) -> JSONResponse:
        if detector is None or not hasattr(detector, "export_3d_snapshot"):
            return JSONResponse({"objects": [], "stamp_unix": 0.0})
        return JSONResponse(detector.export_3d_snapshot())

    async def cam(_request) -> HTMLResponse:
        return HTMLResponse(_INDEX_CAM_HTML)

    async def camera_state(_request) -> JSONResponse:
        return JSONResponse(_camera_payload(hub))

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
        Route("/api/state", state, methods=["GET"]),
        Route("/api/objects3d", objects3d, methods=["GET"]),
        Route("/api/camera", camera_state, methods=["GET"]),
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
            `${objs.length} obj · ${(s.relations || []).length} rel · t=${fmt(s.stamp_unix)}`;
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
      <div class="empty" id="depth_empty">no depth stream connected</div>
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
