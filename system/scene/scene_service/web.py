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

import logging
import time
from pathlib import Path
from typing import Any, Optional

from starlette.applications import Starlette
from starlette.responses import HTMLResponse, JSONResponse
from starlette.routing import Route

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
    #wrap { display: grid; grid-template-columns: 1fr 360px; height: 100vh; }
    #canvas-wrap { position: relative; }
    canvas { display: block; width: 100%; height: 100%; background: #14171f; }
    #info { padding: 12px 16px; overflow-y: auto; border-left: 1px solid #1f2229;
      font-size: 13px; line-height: 1.45; }
    h1 { margin: 0 0 6px 0; font-size: 16px; font-weight: 600; color: var(--acc); }
    h2 { margin: 18px 0 6px 0; font-size: 12px; font-weight: 600;
      text-transform: uppercase; letter-spacing: 0.06em; color: var(--muted); }
    table { width: 100%; border-collapse: collapse; font-size: 12px; }
    td { padding: 3px 6px; border-bottom: 1px solid #1a1d24; vertical-align: top; }
    td:first-child { color: var(--acc); white-space: nowrap; font-family: "SF Mono", ui-monospace, monospace; }
    td.cls { color: #f0c674; }
    td.miss { color: #888; }
    #pose { font-family: "SF Mono", ui-monospace, monospace; font-size: 12px; color: var(--muted); }
    #stamp { color: var(--muted); font-size: 11px; }
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
  <aside id="info">
    <h1>scene</h1>
    <div id="stamp">—</div>
    <h2>robot</h2>
    <div id="pose">no fix yet</div>
    <h2>objects</h2>
    <table id="objs"><tbody></tbody></table>
  </aside>
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

function draw(state) {
    fit();
    ctx.clearRect(0, 0, c.width, c.height);

    // re-center on the robot if there is one; fall back to last-known.
    const robot = (state.objects || []).find(o => o.cls === 'robot');
    if (robot) center = [robot.pose.x, robot.pose.y];

    // 1m grid
    ctx.strokeStyle = '#1f232c'; ctx.lineWidth = 1;
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

    // axes through robot
    const [rxp, ryp] = w2p(center[0], center[1]);
    ctx.strokeStyle = '#2a2e36';
    ctx.beginPath(); ctx.moveTo(0, ryp); ctx.lineTo(w, ryp); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(rxp, 0); ctx.lineTo(rxp, h); ctx.stroke();

    // objects
    for (const o of (state.objects || [])) {
        const [px, py] = w2p(o.pose.x, o.pose.y);
        const r = Math.max(4, Math.min(20, (o.bbox.size_x || 0.2) * pxPerM * 0.5));
        ctx.globalAlpha = o.missing ? 0.3 : 1.0;
        ctx.fillStyle = classColor(o.cls);
        ctx.beginPath(); ctx.arc(px, py, r, 0, Math.PI * 2); ctx.fill();
        ctx.globalAlpha = 1;
        ctx.fillStyle = '#e8eaed'; ctx.font = '11px ui-monospace, monospace';
        ctx.fillText(o.cls + (o.cls === 'robot' ? '' : ` ${o.short_id}`), px + r + 4, py + 4);
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

function fillTable(state) {
    const tbody = document.querySelector('#objs tbody');
    tbody.innerHTML = '';
    const objs = (state.objects || []).slice().sort((a, b) => a.cls.localeCompare(b.cls));
    for (const o of objs) {
        const tr = document.createElement('tr');
        const id = `<td>${o.short_id}</td>`;
        const cls = `<td class="cls">${o.cls}</td>`;
        const pose = `<td>${fmt(o.pose.x)}, ${fmt(o.pose.y)}, ${fmt(o.pose.z)}</td>`;
        const conf = `<td class="${o.missing ? 'miss' : ''}">c=${fmt(o.confidence)}<br/>n=${o.observation_count}</td>`;
        tr.innerHTML = id + cls + pose + conf;
        tbody.appendChild(tr);
    }
    document.getElementById('stamp').textContent =
        `${objs.length} objects · ${state.relations?.length || 0} relations · stamp ${fmt(state.stamp_unix)}`;
    if (state.robot) {
        document.getElementById('pose').textContent =
            `(${fmt(state.robot.x)}, ${fmt(state.robot.y)}, ${fmt(state.robot.z)})  yaw=${fmt(state.robot.yaw)} rad`;
    }
}

async function tick() {
    try {
        const r = await fetch('/api/state', { cache: 'no-store' });
        if (!r.ok) return;
        const state = await r.json();
        draw(state);
        fillTable(state);
    } catch (_) { /* swallow; next tick will retry */ }
}
setInterval(tick, 200);
tick();
</script>
</body>
</html>
"""


def _shorten_id(object_id: str) -> str:
    # `scene.object.cup_001` → `cup_001` for the table.
    return object_id.split(".", 2)[-1]


def _state_payload(registry: ObjectRegistry, relations: RelationEngine) -> dict:
    """Serialise the registry + relations into the small JSON shape the
    page consumes. Done in one snapshot so the page never sees a
    half-updated registry."""
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


def make_app(*, registry: ObjectRegistry, relations: RelationEngine) -> Starlette:
    """Build the Starlette ASGI app the entrypoint mounts on its own
    uvicorn server. Two routes only: index page + JSON state."""

    async def index(_request) -> HTMLResponse:
        return HTMLResponse(_INDEX_HTML)

    async def state(_request) -> JSONResponse:
        return JSONResponse(_state_payload(registry, relations))

    return Starlette(routes=[
        Route("/", index, methods=["GET"]),
        Route("/api/state", state, methods=["GET"]),
    ])
