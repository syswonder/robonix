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
import html
import json
import logging
import os
import re
import time
from pathlib import Path
from urllib.parse import quote
from typing import Any, Optional

from starlette.applications import Starlette
from starlette.exceptions import HTTPException
from starlette.staticfiles import StaticFiles
from starlette.responses import (HTMLResponse, JSONResponse,
                                 PlainTextResponse, RedirectResponse,
                                 Response, StreamingResponse)
from starlette.middleware import Middleware
from starlette.routing import Route, Mount

from robonix_api import ATLAS

from .message_shape import image_is_well_formed, occupancy_grid_is_well_formed

from . import geometry
from .annotations import normalize_kind, validate_annotation_fields
from .map_binding import sanitize_map_id as _sanitize_map_id
from .map_meta import make_meta
from .rerun_sink import RERUN_VIEWER_DIR
from .state import ObjectRegistry

# Page markup, styles and scripts, read once at import.
_ASSET_DIR = Path(__file__).resolve().parent / "web_assets"


class _ExtensionlessJs(StaticFiles):
    """StaticFiles that also answers `foo` with `foo.js`: the rerun viewer
    bundle imports `./re_viewer` without an extension, as bundlers allow."""

    async def get_response(self, path: str, scope):
        try:
            return await super().get_response(path, scope)
        except HTTPException as missing:
            if missing.status_code != 404 or Path(path).suffix:
                raise
            return await super().get_response(path + ".js", scope)


def _asset(name: str) -> str:
    return (_ASSET_DIR / name).read_text(encoding="utf-8")


log = logging.getLogger(__name__)

# Encoded previews, keyed by the hub's message count so each message is
# encoded once however many pages poll. A None payload is cached too.
_OCCUPANCY_CACHE: dict[str, Any] = {"count": -1, "payload": None}
_CAMERA_CACHE: dict[str, dict[str, Any]] = {
    "rgb": {"hub": None, "count": -1, "payload": None},
    "depth": {"hub": None, "count": -1, "payload": None},
}
_CAMERA_PREVIEW_MIN_INTERVAL_S = 0.4


def occupancy_payload(hub: Any) -> Optional[dict]:
    """The latest OccupancyGrid as a grayscale PNG plus its geometry, or None
    when there is no map yet or it cannot be rendered."""
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
    # A mis-sized grid is valid ROS 2 but would make reshape raise.
    if not occupancy_grid_is_well_formed(w, h, len(msg.data)):
        log.warning(
            "occupancy_grid dropped: %d cells for a %dx%d grid",
            len(msg.data), w, h,
        )
        return None
    # -1 unknown → 128, 0 free → 240, 100 occupied → 20; rows bottom-up.
    arr = np.frombuffer(bytes(msg.data), dtype=np.int8).reshape(h, w)
    out = np.full((h, w), 128, dtype=np.uint8)
    out[arr == 0]   = 240
    out[arr == 100] = 20
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
    """sensor_msgs/Image as a PNG; depth is normalised per frame to
    grayscale. None for an unsupported encoding or without Pillow."""
    try:
        import numpy as np
        from PIL import Image as PILImage
    except ImportError:
        return None
    h, w = int(msg.height), int(msg.width)
    enc = (msg.encoding or "").lower()
    if not image_is_well_formed(w, h, enc, len(msg.data)):
        log.warning(
            "%s image dropped: %d bytes for a %dx%d %s frame",
            kind, len(msg.data), w, h, enc,
        )
        return None
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
        # Clip to [near, p99] so one garbage pixel cannot flatten the range.
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
    """One channel's preview, cached per message. The entry is replaced
    whole, so a reader on another thread never sees half an update."""
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
        payload["stamp_ms"] = int(stamp_unix * 1000)
    _CAMERA_CACHE[kind] = {"hub": hub, "count": count, "payload": payload}
    return payload


def _camera_payload(hub: Any) -> dict:
    """The latest RGB and depth previews."""
    return {kind: _camera_channel_payload(hub, kind) if hub is not None
            else None for kind in ("rgb", "depth")}


def _camera_json_bytes(hub: Any) -> bytes:
    """The /api/camera body, encoded entirely in the calling worker."""
    return json.dumps(_camera_payload(hub), ensure_ascii=False,
                      allow_nan=False, separators=(",", ":")).encode("utf-8")


def _state_payload(registry: ObjectRegistry,
                   hub: Any, sg_store: Any = None,
                   anno_store: Any = None,
                   map_binding: Optional[dict] = None,
                   robot_geometry: Any = None) -> dict:
    """The /api/state body: objects, the geometric relations, the composed
    scene graph, robot, occupancy, regions and the map binding."""
    objs_dict = _sync_snapshot(registry)
    geo_edges = sg_store.get_geometric_edges() if sg_store is not None else []
    regions = anno_store.list_json() if anno_store is not None else []
    out_objects: list[dict[str, Any]] = []
    robot_pose: Optional[dict[str, float]] = None
    for o in objs_dict.values():
        pose = {"x": o.pose.x, "y": o.pose.y, "z": o.pose.z, "yaw": o.pose.yaw}
        out_objects.append({
            "id": o.object_id,
            "display_name": o.display_name,
            "caption": o.caption,
            "caption_source": o.caption_source,
            "label": o.label,
            # Deprecated: `cls` is `label`, `short_id` is the id's last part.
            "cls": o.label,
            "short_id": o.object_id.rsplit(".", 1)[-1],
            "region": geometry.region_of(
                float(o.pose.x), float(o.pose.y), regions),
            "pose": pose,
            "bbox": {
                "size_x": o.bbox.size_x, "size_y": o.bbox.size_y, "size_z": o.bbox.size_z,
                "yaw": o.bbox.yaw,
            },
            # Tracking state, for the panel's debug view.
            "settled": o.settled,
            "confidence": o.confidence,
            "observation_count": o.observation_count,
            "missing": o.missing,
            "provisional": bool(o.attributes.get("label_provisional", False)),
        })
        if o.attributes.get("is_robot"):
            robot_pose = pose
    out_relations = [
        {"subject": e.source_id, "predicate": e.relation, "target": e.target_id}
        for e in geo_edges
    ]

    sg_payload: dict[str, Any] = {"edges": [], "updated_at": 0.0}
    on_map = {o["id"] for o in out_objects if o["settled"]}
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
                    if e.shown_on_map()
                    and e.source_id in on_map and e.target_id in on_map
                ],
                "updated_at": snap.updated_at,
            }

    return {
        "objects": out_objects,
        "relations": out_relations,
        "scene_graph": sg_payload,
        "robot": robot_pose,
        "robot_footprint": (
            robot_geometry.current().to_json()
            if robot_geometry is not None and robot_geometry.current() is not None
            else None
        ),
        "occupancy": occupancy_payload(hub),
        "annotations": regions,
        "map_binding": map_binding,
        "stamp_unix": time.time(),
    }


def _sync_snapshot(registry: ObjectRegistry):
    """The registry's objects without its asyncio lock: the dict copy is
    atomic, and a page that sees one tick half-updated fixes itself next."""
    return dict(registry._objects)  # noqa: SLF001


# The robonix/service/map operations the map library calls.
_MAP_OPS = ("list_maps", "save_map", "load_map", "delete_map",
            "pose_estimate", "get_pose", "get_mode")


def _map_rpc(op: str, payload: Optional[dict] = None, timeout_s: float = 20.0) -> dict:
    """Call one robonix/service/map operation through Atlas and gRPC; the
    response's set fields as a dict, or {ok: False, detail}."""
    import grpc  # lazy: web debug imports should not fail without grpc
    import map_pb2  # type: ignore
    import robonix_contracts_pb2_grpc as contracts_grpc  # type: ignore

    assert op in _MAP_OPS, op
    method_name = "".join(part.title() for part in op.split("_"))
    contract_id = f"robonix/service/map/{op}"
    stub_name = f"RobonixServiceMap{method_name}Stub"
    req_name = f"{method_name}_Request"
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
        return {field.name: value for field, value in resp.ListFields()}
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


# Interface copy, one file per language (`strings_<lang>.json`), keyed.
_LANGS = ("en", "zh")


def _load_strings() -> dict[str, dict[str, str]]:
    """`{key: {lang: text}}`. English defines the keys; a missing
    translation falls back to English in the page."""
    per_lang = {lang: json.loads(_asset(f"strings_{lang}.json"))
                for lang in _LANGS}
    keys = per_lang["en"]
    return {key: {lang: table[key]
                  for lang, table in per_lang.items() if key in table}
            for key in keys}


_STRINGS: dict[str, dict[str, str]] = _load_strings()

_I18N_JS = _asset("i18n.js")


def _i18n_js() -> str:
    return _I18N_JS.replace("__TABLE__", json.dumps(_STRINGS, ensure_ascii=False))



_NAV_LINKS = (
    ("/maps", "maps", "nav.maps"),
    ("/", "3D", "nav.semantic"),
    ("/2d", "2D", "nav.2d"),
    ("/cam", "camera", "nav.cam"),
    ("/regions", "regions", "nav.regions"),
    ("/logs", "logs", "nav.logs"),
)

# Views of the live map, grouped under one heading in this order.
_NAV_LIVE_MAP = ("/", "/2d", "/regions")


_ICON = ('<svg class="ico" viewBox="0 0 24 24" fill="none" stroke="currentColor"'
         ' stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round">{}</svg>')

_NAV_ICONS = {
    "/maps": _ICON.format(
        '<path d="M4 7.5 12 4l8 3.5-8 3.5z"/><path d="m4 12 8 3.5 8-3.5"/>'
        '<path d="m4 16.5 8 3.5 8-3.5"/>'),
    "/": _ICON.format(
        '<path d="M21 16V8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16z"/>'
        '<path d="m3.3 7 8.7 5 8.7-5"/><path d="M12 22V12"/>'),
    "/2d": _ICON.format(
        '<path d="M15 6 9 3 3 6v15l6-3 6 3 6-3V3z"/><path d="M9 3v15"/><path d="M15 6v15"/>'),
    "/cam": _ICON.format(
        '<path d="M14.5 4h-5L7 7H4a2 2 0 0 0-2 2v9a2 2 0 0 0 2 2h16a2 2 0 0 0 2-2V9a2 2 0 0 0-2-2h-3z"/>'
        '<circle cx="12" cy="13" r="3.2"/>'),
    "/logs": _ICON.format(
        '<path d="M4 6.5h13M4 11h16M4 15.5h11M4 20h7"/>'),
    "/regions": _ICON.format(
        '<path d="M4 6.5 10 4l4 2.5L20 4v13.5L14 20l-4-2.5L4 20z"/>'
        '<circle cx="12" cy="10.5" r="1.6"/>'),
}


# ── Scribe log reader ──────────────────────────────────────────────────────
# One append-only JSON-lines file per tag, so a byte offset is a cursor.
_LOG_LEVELS = ("debug", "info", "warn", "error")
_LOG_LEVEL_INDEX = {name: i for i, name in enumerate(_LOG_LEVELS)}

_LOG_TAIL_BYTES = 120_000      # first load: the tail of each file
_LOG_MAX_NEW_BYTES = 400_000


def _scribe_dir() -> Optional[Path]:
    """Where rbnx put this deployment's logs (SCRIBE_LOG_DIR), or None."""
    raw = (os.environ.get("SCRIBE_LOG_DIR") or "").strip()
    if not raw:
        return None
    path = Path(raw)
    return path if path.is_dir() else None


_LOG_LEVEL_ALIASES = {"warning": "warn", "critical": "error", "fatal": "error",
                      "trace": "debug"}


def _normalise_level(value: str) -> str:
    name = str(value or "").strip().lower()
    name = _LOG_LEVEL_ALIASES.get(name, name)
    return name if name in _LOG_LEVEL_INDEX else "info"


def _read_log_slice(path: Path, offset: Optional[int]) -> tuple[list[dict], int]:
    """Lines appended since `offset`, and the offset to use next. No offset
    means the tail; an offset past the end means the file was rotated."""
    try:
        size = path.stat().st_size
    except OSError:
        return [], offset or 0

    start = offset
    if start is None or start > size:
        start = max(0, size - _LOG_TAIL_BYTES)
    if size - start > _LOG_MAX_NEW_BYTES:
        start = size - _LOG_MAX_NEW_BYTES

    try:
        with path.open("rb") as handle:
            handle.seek(start)
            blob = handle.read(size - start)
    except OSError:
        return [], size

    # Drop a leading partial line after a tail seek; leave a trailing
    # partial line for the next poll.
    consumed = start + len(blob)
    if blob and not blob.endswith(b"\n"):
        cut = blob.rfind(b"\n")
        if cut < 0:
            return [], start
        consumed = start + cut + 1
        blob = blob[:cut + 1]
    text = blob.decode("utf-8", "replace")
    lines = text.split("\n")
    if start > 0 and offset is None and lines:
        lines = lines[1:]

    out = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except ValueError:
            # Raw stderr (a traceback) is shown as is.
            out.append({"ts": "", "level": "info", "tag": path.stem,
                        "msg": line, "raw": True})
            continue
        out.append({
            "ts": str(row.get("ts") or ""),
            "level": _normalise_level(row.get("level")),
            "tag": str(row.get("tag") or path.stem),
            "msg": str(row.get("msg") or ""),
        })
    return out, consumed


def _nav(active: str) -> str:
    """The sidebar every page shares."""
    by_href = {href: (label, key) for href, label, key in _NAV_LINKS}

    def link(href: str) -> str:
        label, key = by_href[href]
        current = ' class="on"' if href == active else ""
        icon = _NAV_ICONS.get(href, "")
        return (f'<a href="{href}"{current}>{icon}'
                f'<span data-i18n="{key}">{label}</span></a>')

    items = ['<div class="nav-group"><div class="nav-head" '
             'data-i18n="nav.group.live">current map</div>']
    items += [link(href) for href in _NAV_LIVE_MAP]
    items.append("</div>")
    items += [link(href) for href, _, _ in _NAV_LINKS
              if href not in _NAV_LIVE_MAP]
    return "".join(items)


# ── The object panel docked beside the map pages, fed by /api/state ─────────
_DOCK_CSS = _asset("dock.css")

_DOCK_TABS = (
    # (id, label, title, svg path data)
    ("objects", "Objects", "what the registry holds",
     '<rect x="3.5" y="3.5" width="7" height="7" rx="1.4"/>'
     '<rect x="13.5" y="3.5" width="7" height="7" rx="1.4"/>'
     '<rect x="3.5" y="13.5" width="7" height="7" rx="1.4"/>'
     '<rect x="13.5" y="13.5" width="7" height="7" rx="1.4"/>'),
    ("relations", "Relations", "how they sit together",
     '<circle cx="5.5" cy="6" r="2.5"/><circle cx="18" cy="12" r="2.5"/>'
     '<circle cx="5.5" cy="18" r="2.5"/>'
     '<path d="M7.8 7.2 15.7 11M7.8 16.8 15.7 13.2"/>'),
    ("robot", "Robot", "where it thinks it is",
     '<rect x="4" y="8" width="16" height="11" rx="2.5"/>'
     '<path d="M12 8V4.5"/><circle cx="12" cy="3.2" r="1.3"/>'
     '<path d="M8.5 12.5v2M15.5 12.5v2"/>'),
)


def _dock_html() -> str:
    """The dock markup: a tab rail, a header, and one pane per tab."""
    tabs = "".join(
        f'<button data-tab="{tid}" data-i18n-title="dock.{tid}.hint"'
        f' data-i18n-aria="dock.{tid}"'
        f' title="{label} — {hint}" aria-label="{label}">'
        f'{_ICON.format(path)}</button>'
        for tid, label, hint, path in _DOCK_TABS
    )
    return f"""
  <aside class="dock" id="dock">
    <div class="grip" id="dock-grip" data-i18n-title="dock.resize"
         title="drag to resize"></div>
    <div class="tabs">{tabs}</div>
    <div class="col">
      <div class="head">
        <span class="name" id="dock-name" data-i18n="dock.objects">Objects</span>
        <span class="stamp" id="dock-stamp">—</span>
        <button class="btn small danger" id="dock-flush"
                data-i18n="dock.flush"
                data-i18n-title="dock.flushTitle"></button>
        <button class="shutbtn" id="dock-shut" data-i18n-title="dock.collapse"
                data-i18n-aria="dock.collapse" title="collapse the dock"
                aria-label="collapse the dock">»</button>
      </div>
      <div class="panes">
        <div class="pane on" data-pane="objects">
          <div class="objs-scroll">
            <div class="gone" id="dock-gone" hidden></div>
            <table class="objs"><tbody id="dock-objs">
              <tr><td class="empty">—</td></tr>
            </tbody></table>
          </div>
          <div class="split" id="dock-split" role="separator"
               aria-orientation="horizontal"
               data-i18n-title="dock.splitDrag"
               title="drag to resize"></div>
          <div id="dock-detail" hidden></div>
        </div>
        <div class="pane" data-pane="relations">
          <div id="dock-rels"><span class="empty">—</span></div>
        </div>
        <div class="pane" data-pane="robot">
          <div id="dock-robot"><span class="empty">no fix yet</span></div>
        </div>
      </div>
    </div>
  </aside>
"""


_DOCK_JS = _asset("dock.js")
_RERUN_HOST_HTML = _asset("rerun_host.html")
_SHELL_CSS = _asset("shell.css")
_CONTROLS_CSS = _asset("controls.css")
_LABELS_JS = _asset("labels.js")
_LOGS_BODY = _asset("logs.html")
_MAPS_BODY = _asset("maps_page.html")


# ── The status bar on every page: mapping mode, bound map, viewer ──────────
_STATUS_BAR = """
<div class="sbar" id="sbar">
  <div class="sb-block">
    <span class="sb-k" data-i18n="bar.mode"></span>
    <span class="sb-mode" id="sb-mode"><i class="dot"></i><span class="txt"></span></span>
  </div>
  <div class="sb-rule"></div>
  <div class="sb-block">
    <span class="sb-k" data-i18n="bar.map"></span>
    <span class="sb-map" id="sb-map">—</span>
  </div>
  <div class="sb-rule"></div>
  <div class="sb-block">
    <span class="sb-k" data-i18n="bar.viewer"></span>
    <span class="sb-viewer" id="sb-viewer">—</span>
  </div>
</div>
<script>
  (function () {
    const bar = document.getElementById('sbar');
    const modeEl = document.getElementById('sb-mode');
    const modeTxt = modeEl.querySelector('.txt');
    const mapEl = document.getElementById('sb-map');
    const viewEl = document.getElementById('sb-viewer');
    const set = (el, text) => { if (el.textContent !== text) el.textContent = text; };
    const cls = (el, name) => { if (el.className !== name) el.className = name; };

    async function beat() {
      try {
        const r = await fetch('/api/state', {cache: 'no-store'});
        if (!r.ok) throw new Error(r.status);
        const s = await r.json();
        const mb = s.map_binding || {};
        // An unnamed live session is mapping, with no saved map yet.
        const temp = mb.source === 'default' && !mb.mode;
        const mode = temp ? 'mapping' : (mb.mode || '');
        cls(bar, 'sbar');
        cls(modeEl, 'sb-mode ' + (mode === 'localization' ? 'loc'
                                : mode === 'mapping' ? 'map' : 'idle'));
        modeTxt.dataset.i18n = mode === 'localization' ? 'bar.localizing'
                             : mode === 'mapping' ? 'bar.mapping' : 'bar.idle';
        set(modeTxt, t(modeTxt.dataset.i18n));
        if (temp) {
          mapEl.dataset.i18n = 'bar.temporary';
          set(mapEl, t('bar.temporary'));
          cls(mapEl, 'sb-map temp');
        } else {
          mapEl.removeAttribute('data-i18n');
          set(mapEl, mb.map_id || '—');
          cls(mapEl, 'sb-map');
        }
        const v = s.viewer || {};
        const key = v.backend === 'rerun' ? 'bar.viewerRerun'
                                          : 'bar.viewerNone';
        viewEl.dataset.i18n = key;
        set(viewEl, t(key));
        cls(viewEl, 'sb-viewer' + (v.fell_back ? ' fell-back' : ''));
        viewEl.title = v.fell_back && v.detail
          ? t('bar.viewerFailed') + ' ' + v.detail : '';
      } catch (_) {
        cls(bar, 'sbar down');
        cls(modeEl, 'sb-mode down');
        modeTxt.dataset.i18n = 'bar.offline';
        set(modeTxt, t('bar.offline'));
        viewEl.removeAttribute('data-i18n');
        set(viewEl, '—');
        cls(viewEl, 'sb-viewer');
      }
      setTimeout(beat, 1000);
    }
    beat();
  })();
</script>
"""


def _shell_page(active: str, body: str, title: str,
                info_panel: bool = False) -> str:
    """Wrap page content in the shared sidebar; `info_panel` docks the
    object panel beside it."""
    css = _CONTROLS_CSS + _SHELL_CSS + (_DOCK_CSS if info_panel else "")
    dock = _dock_html() if info_panel else ""
    script = "<script>" + _i18n_js() + """
    applyLang(langGet());
    document.getElementById('lang-switch').addEventListener('click', () => {
      langSet(langGet() === 'zh' ? 'en' : 'zh');
    });
    document.querySelectorAll('iframe').forEach(f => f.addEventListener(
      'load', () => {
        try { f.contentWindow.postMessage({sceneLang: langGet()}, '*'); }
        catch (_) {}
      }));
    </script>"""
    script += f"<script>{_DOCK_JS}</script>" if info_panel else ""
    return (
        "<!doctype html><html lang=\"zh\"><head><meta charset=\"utf-8\">"
        f"<title>{title}</title><style>{css}</style></head><body>"
        f'<div class="wrap"><nav><div class="brand">scene</div>{_nav(active)}'
        '<div class="spacer"></div>'
        '<button class="lang" id="lang-switch"'
        ' data-i18n-other="nav.lang.self"'
        ' data-i18n-title="nav.lang.title">English</button></nav>'
        f"<main>{_STATUS_BAR}{body}</main>{dock}</div>{script}</body></html>"
    )


def _framed(path: str, title: str) -> str:
    """A standalone page embedded in the shell; `?bare=1` is the page itself."""
    return _shell_page(
        path, f'<iframe src="{path}?bare=1" title="{title}"></iframe>', title)


def _bare(request) -> bool:
    return request.query_params.get("bare") == "1"


def make_app(*, registry: ObjectRegistry,
             hub: Any = None, detector: Any = None,
             sg_store: Any = None, anno_store: Any = None,
             object_store: Any = None, map_meta: Any = None,
             map_binding: Optional[dict] = None,
             ops_lock: Optional[asyncio.Lock] = None,
             semantic_hold: Optional[dict] = None,
             robot_geometry: Any = None,
             object_mutations: Any = None,
             object_views: Any = None,
             rerun_sink: Any = None) -> Starlette:
    """The Scene web app: the pages, /api/state, region CRUD and the map
    library façade over robonix/service/map.

    Region API (see system/scene/README.md): GET/POST /api/regions,
    PUT/DELETE /api/regions/{id}. PUT merges any of name, points, theta and
    `stale: false`; theta is poi-only. Errors are {ok: false, detail} with 400
    (bad input), 404 (unknown id) or 503 (no store); a failed store write is a
    plain 500 on purpose. No authentication: this is a LAN debug server.

    `map_binding` is the mutable {map_id, mode, generation, source} shared with
    the lifecycle watcher, and `ops_lock` serialises Save/Load/Delete with it.
    `object_store` + `map_meta` hold each saved map's object snapshot; without
    `map_meta`, Save/Load cover regions only.
    """
    if map_binding is None:
        map_binding = {}
    if ops_lock is None:
        ops_lock = asyncio.Lock()
    # Set while Scene's semantic state may not match the map mapping runs (a
    # load in flight or failed, or a switch outside the façade); Save refuses.
    if semantic_hold is None:
        semantic_hold = {"reason": None}

    # /api/camera: one encode at a time, reused for a short interval.
    camera_preview_lock = asyncio.Lock()
    camera_preview_hub: Any = None
    camera_preview_bytes: Optional[bytes] = None
    camera_preview_completed_s = 0.0

    async def _persist_scene_objects(partition: str) -> tuple[int, int]:
        """Write every object except the robot (missing ones too: a fresh
        Load's objects are all missing) into `partition`. Returns
        (written, expected); fewer written means the snapshot is incomplete."""
        if object_store is None:
            return 0, 0
        objs = await registry.snapshot()
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
        """Restore one snapshot partition; raises on a read error, which must
        not look like an empty snapshot."""
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
        """(generation, mode) from mapping's latched lifecycle broadcast for
        this map, or (None, "") when there is none or it names another map."""
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
            return None, ""

    _RESERVED_MAP_ID = re.compile(r"^\.|__s\d+$")

    def _reserved_map_id_error(map_id: str) -> Optional[JSONResponse]:
        """400 for ids in Scene's own namespaces: a leading `.` (live session)
        or a `__s<N>` suffix (snapshot partitions)."""
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

    def _map_page(path: str, view: str, title: str) -> HTMLResponse:
        # `available`, not `ready`: the frame is what starts the viewer.
        if rerun_sink is None or not rerun_sink.available:
            why = rerun_sink.detail if rerun_sink is not None else (
                "the map viewer is disabled")
            body = ('<div class="msg">No map viewer: '
                    + html.escape(why) + '.</div>')
        else:
            body = (f'<iframe id="v" src="/rerun?view={view}" '
                    f'title="{title}"></iframe>')
        return HTMLResponse(_shell_page(path, body, title, info_panel=True))

    async def index(_request) -> HTMLResponse:
        return _map_page("/", "3d", "scene — semantic map")

    async def index2d(_request) -> HTMLResponse:
        return _map_page("/2d", "2d", "scene — 2D map")

    async def state(_request) -> JSONResponse:
        payload = _state_payload(
            registry,
            hub,
            sg_store,
            anno_store,
            map_binding,
            robot_geometry,
        )
        live = rerun_sink is not None and rerun_sink.ready
        payload["viewer"] = {
            "backend": "rerun" if live else "none",
            "fell_back": rerun_sink is not None and not live,
            "detail": "" if live else (
                rerun_sink.detail if rerun_sink is not None else ""),
        }
        return JSONResponse(payload)

    # ── annotation CRUD ──────────────────────────────────────────────
    def _anno_error(status: int, detail: str) -> JSONResponse:
        return JSONResponse({"ok": False, "detail": detail}, status_code=status)

    async def _anno_body(request) -> Optional[dict]:
        """The JSON object body, or None when it is not one."""
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
        """Create a region from {kind, name, points, theta?}."""
        if anno_store is None:
            return _anno_error(503, "annotation store unavailable")
        body = await _anno_body(request)
        if body is None:
            return _anno_error(400, "request body must be a JSON object")
        kind = normalize_kind(body.get("kind"))
        name = body.get("name", "")
        points = body.get("points")
        theta = body.get("theta")
        err = validate_annotation_fields(kind, name, points, theta)
        if err:
            return _anno_error(400, err)
        ann = anno_store.create(kind=kind, name=name, points=points, theta=theta)
        return JSONResponse({"ok": True, "annotation": ann.to_json()})

    async def annotations_update(request) -> JSONResponse:
        """Partial update; the merged result is validated as a create would be."""
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
        """Delete a region; 404 when the id is unknown."""
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

    async def _map_request(request, locked_op, *, reserved_check=False):
        """Parse `{map_id, ...}` and run `locked_op(map_id, body)` under
        `ops_lock`, which serialises Save/Load/Delete with the lifecycle
        watcher."""
        body = await _anno_body(request)
        if body is None:
            return _anno_error(400, "request body must be a JSON object")
        map_id = str(body.get("map_id") or "").strip()
        if not map_id:
            return _anno_error(400, "map_id is required")
        if reserved_check:
            reserved = _reserved_map_id_error(map_id)
            if reserved is not None:
                return reserved
        async with ops_lock:
            return await locked_op(map_id, body)

    async def maps_save(request) -> JSONResponse:
        return await _map_request(request, _maps_save_locked,
                                  reserved_check=True)

    async def _maps_save_locked(map_id: str, body: dict) -> JSONResponse:
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
            # Saving the live session over a map whose regions were never
            # loaded would overwrite them.
            return _anno_error(409, (
                f"map {map_id} already has saved regions; load it "
                "first (or delete the map) instead of overwriting them with "
                "this live session"
            ))
        if spatial_exists:
            if current_bound_id and current_bound_id != map_id:
                return _anno_error(409, f"spatial map {map_id} already exists; load it before updating scene annotations/objects")
            if current_mode == "mapping":
                # The live frame has drifted from the frozen artifact.
                return _anno_error(409, (
                    f"map {map_id} was saved from this still-running mapping "
                    "session; its spatial artifact is immutable and the live "
                    "frame has kept drifting since. Load it in localization "
                    "mode to edit regions/objects, or delete it and save anew."
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
            # Write a fresh partition, commit the sidecar only when complete,
            # then purge the previous one; a failure keeps the last snapshot.
            if map_meta is not None and object_store is not None:
                prev_meta = map_meta.read(map_id)
                partition, seq = map_meta.next_partition(map_id)
                try:
                    # Clear rows a failed attempt left under the same token.
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
                            out["stale_partition_cleanup_error"] = str(e)
                except Exception as e:  # noqa: BLE001
                    object_persist_error = str(e)
                    out["object_persist_error"] = object_persist_error
            elif object_store is not None:
                object_persist_error = (
                    "map_meta sidecar unavailable — objects not snapshotted"
                )
                out["object_persist_error"] = object_persist_error
            if map_meta is not None:
                # The library's thumbnail; a map without one still saved.
                try:
                    occupancy = occupancy_payload(hub)
                    if occupancy is not None:
                        await asyncio.to_thread(map_meta.write_preview, map_id, occupancy)
                except Exception as error:  # noqa: BLE001
                    log.warning("[scene-maps] no preview for %s: %s", map_id, error)
            mode_after_save = current_mode if spatial_exists and current_bound_id == map_id and current_mode else "mapping"
            photos_were = object_views.partition(map_binding) if object_views else None
            _set_map_binding(map_id, mode_after_save, "ui_save", generation=gen)
            if object_views is not None:
                # A saved session's pictures follow it to the map id.
                await asyncio.to_thread(object_views.adopt, photos_were,
                                        object_views.partition(map_binding))
            if object_persist_error is not None:
                # Save is geometry + regions + objects as one unit, so a
                # missing snapshot fails it. After a mapping-session save the
                # artifact is immutable and only delete-and-resave recovers.
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
            saved_map = next((item for item in maps_payload.get("maps", [])
                              if item.get("map_id") == map_id), None)
            validation = {
                "map_id": map_id,
                "spatial_ok": bool(saved_map and saved_map.get("has_spatial_artifact") and saved_map.get("spatial_ok") is not False),
                "artifact_detail": (saved_map or {}).get("artifact_detail") or maps_payload.get("detail") or "map not found after save",
                "artifact_size": (saved_map or {}).get("artifact_size"),
                "has_preview": bool(saved_map and saved_map.get("has_preview")),
                "updated": (saved_map or {}).get("updated"),
                "object_count": int(object_count or 0),
                "room_count": sum(1 for a in annotations if a.get("kind") == "region"),
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
        return await _map_request(request, _maps_load_locked)

    async def _maps_load_locked(map_id: str, body: dict) -> JSONResponse:
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
            # Until every rebind below commits, Save is held off; only full
            # success clears the hold.
            semantic_hold["reason"] = f"the last load of {map_id} did not complete"
            # Restore semantic state only once the loaded map's grid arrives.
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
            # The load starts a new frame: flush pre-load objects first, and
            # abort rather than rebind over a registry that still has them.
            try:
                registry_lock = getattr(registry, "lock", None)
                if callable(registry_lock):
                    async with registry_lock():
                        flushed = registry.clear_objects()
                else:
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
                        f"FAILED: {e} — aborted before regions/objects were "
                        "rebound; retry the load"
                    ),
                }
                return JSONResponse(out, status_code=502)
            gen, _broadcast_mode = _latched_lifecycle(map_id)
            meta = map_meta.read(map_id) if map_meta is not None else None
            if meta is not None:
                # Restore before rebinding, so a failure keeps the old binding.
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
                anno_gen = gen if gen is not None else (
                    meta.mapping_generation if meta is not None else None
                )
                anno_store.rebind(map_id, generation=anno_gen, carry_current=False)
            if meta is None:
                # No sidecar: no snapshot is known to match this frame.
                out["objects_restored"] = 0
                out["semantic_snapshot"] = (
                    "absent — objects not restored (no epoch sidecar for "
                    "this map; re-save it to create one)"
                )
                out["detail"] = (
                    f"{out.get('detail') or 'loaded'}; no semantic snapshot "
                    "for this map — objects not restored (regions still load; "
                    "re-save the map to snapshot objects)"
                )
                log.warning(
                    "[scene-maps] load %s: no semantic sidecar — restoring "
                    "no objects (regions still load; re-save the map to "
                    "snapshot current objects)", map_id,
                )
            _set_map_binding(map_id, "localization", "ui_load", generation=gen)
            semantic_hold["reason"] = None
        return JSONResponse(out, status_code=200 if out.get("ok") else 502)

    async def maps_delete(request) -> JSONResponse:
        return await _map_request(request, _maps_delete_locked)

    async def _maps_delete_locked(map_id: str, _body: dict) -> JSONResponse:
        if (map_binding.get("source") != "default"
                and _sanitize_map_id(map_binding.get("map_id") or "") == _sanitize_map_id(map_id)):
            # Mapping would keep localizing on it and Scene would still name it.
            return _anno_error(409, (
                f"map {map_id} is in use; load another map or start a new "
                "session before deleting it"))
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
            if object_views is not None:
                await asyncio.to_thread(object_views.forget_map, map_id)
            meta = map_meta.read(map_id) if map_meta is not None else None
            snapshot_rows_orphaned = False
            if object_store is not None:
                # The snapshot partition and legacy rows under the bare id.
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
                    # Keep the only pointer to rows a retry must still find.
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
            yaw = float(pose.get("theta", 0.0)) - payload["theta"]
            dtheta = math.atan2(math.sin(yaw), math.cos(yaw))
            return {"dx": dx, "dy": dy, "dtheta": dtheta,
                    "distance_m": math.hypot(dx, dy), "abs_yaw_rad": abs(dtheta)}

        mode_info = await asyncio.to_thread(_map_rpc, "get_mode", {}, 3.0)
        before = await asyncio.to_thread(_map_rpc, "get_pose", {}, 3.0)
        out = await asyncio.to_thread(_map_rpc, "pose_estimate", payload)
        # Give relocalisation a moment, so the reply can show convergence.
        await asyncio.sleep(float(os.environ.get("SCENE_POSE_ESTIMATE_FEEDBACK_DELAY_S", "1.2")))
        after = await asyncio.to_thread(_map_rpc, "get_pose", {}, 3.0)
        out.update(requested_pose=payload, mode=mode_info, pose_before=before,
                   pose_after=after, delta_before=_pose_delta(before),
                   delta_after=_pose_delta(after))
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

    # The viewer's data (gRPC-web over HTTP/1.1) is proxied through Scene's
    # own port, so a browser needs to reach only that one.
    _PROXY_SKIP = {"host", "content-length", "connection", "keep-alive",
                   "transfer-encoding", "upgrade"}

    async def _proxy(request, port: int, path: str):
        """Stream one request to a local rerun server and back."""
        import httpx

        url = f"http://127.0.0.1:{port}/{path.lstrip('/')}"
        if request.url.query:
            url = f"{url}?{request.url.query}"
        headers = {k: v for k, v in request.headers.items()
                   if k.lower() not in _PROXY_SKIP}
        # Loopback: an HTTP_PROXY from the environment must not apply.
        client = httpx.AsyncClient(timeout=None, trust_env=False)
        try:
            body = await request.body()
            upstream = client.build_request(
                request.method, url, headers=headers, content=body)
            response = await client.send(upstream, stream=True)
        except Exception as error:  # noqa: BLE001
            await client.aclose()
            return PlainTextResponse(
                f"the rerun viewer is not reachable on 127.0.0.1:{port}: "
                f"{error}", status_code=502)

        async def stream():
            try:
                async for chunk in response.aiter_raw():
                    yield chunk
            finally:
                await response.aclose()
                await client.aclose()

        out = {k: v for k, v in response.headers.items()
               if k.lower() not in _PROXY_SKIP}
        return StreamingResponse(stream(), status_code=response.status_code,
                                 headers=out)

    async def rerun_data(request):
        """One map's gRPC-web stream. rerun requires the source path to be
        exactly `/proxy` and sends its calls to the origin root, so both feeds
        share these paths; the asking page's `view=2d` (or `?feed=`) picks the
        2D feed, anything else the 3D one."""
        if rerun_sink is None or not rerun_sink.ready:
            return PlainTextResponse("no viewer", status_code=404)
        feed = request.query_params.get("feed")
        if feed not in ("2d", "3d"):
            referer = request.headers.get("referer") or ""
            feed = "2d" if "view=2d" in referer else "3d"
        return await _proxy(request, rerun_sink.data_port(feed),
                            request.url.path)

    async def rerun_grpc(request):
        """The viewer's gRPC-web calls, which arrive at the origin root."""
        service = request.path_params.get("service", "")
        if "MessageProxyService" not in service:
            return PlainTextResponse("not found", status_code=404)
        return await rerun_data(request)

    async def viewer_url(request) -> JSONResponse:
        """Each map page's data source, on the host the reader used; or why
        there is none."""
        if rerun_sink is None or not rerun_sink.ready:
            detail = (rerun_sink.detail if rerun_sink is not None
                      else "the map viewer is disabled")
            return JSONResponse({"url": "", "detail": detail})
        source = f"rerun+http://{request.headers.get('host') or '127.0.0.1'}/proxy"
        return JSONResponse({"url": source, "detail": ""})

    async def objects3d(_request) -> JSONResponse:
        if detector is None or not hasattr(detector, "export_3d_snapshot"):
            return JSONResponse({"objects": [], "stamp_unix": 0.0})
        return JSONResponse(detector.export_3d_snapshot())

    async def cam(request) -> HTMLResponse:
        if _bare(request):
            return HTMLResponse(_INDEX_CAM_HTML)
        return HTMLResponse(_framed("/cam", "scene — camera"))

    async def regions_page(request) -> HTMLResponse:
        if _bare(request):
            return HTMLResponse(_regions_html())
        return HTMLResponse(_framed("/regions", "scene — regions"))

    async def _correct(request, apply, reply):
        """One correction through the mutation coordinator.

        `apply(body, epoch)` returns `(result, persisted, map_id, generation)`,
        or a response to send instead; `reply(result)` gives the fields that
        are particular to the call. A coordinator refusal (a stale epoch, a
        missing object) is a 409 carrying its message.
        """
        if object_mutations is None:
            return JSONResponse(
                {"ok": False,
                 "detail": "this deployment has no object mutation coordinator"},
                status_code=503)
        body = (await _anno_body(request) or {})
        epoch = {"expected_map_id": str(body.get("expected_map_id") or ""),
                 "expected_generation": body.get("expected_generation")}
        try:
            out = await apply(body, epoch)
        except Exception as error:  # noqa: BLE001
            return JSONResponse({"ok": False, "detail": str(error)},
                                status_code=409)
        if isinstance(out, Response):
            return out
        result, persisted, map_id, generation = out
        return JSONResponse({"ok": True, **reply(result), "map_id": map_id,
                             "generation": generation, "persisted": persisted})

    async def object_label(request) -> JSONResponse:
        """Correct one object's class, or clear a previous correction."""
        async def apply(body, epoch):
            label = str(body.get("label") or "").strip()
            clear = bool(body.get("clear_override"))
            if not label and not clear:
                return JSONResponse(
                    {"ok": False, "detail": "label must not be empty"},
                    status_code=400)
            return await object_mutations.apply_label_correction(
                object_id=request.path_params["object_id"], label=label,
                clear_override=clear,
                note=str(body.get("note") or "renamed from the web UI"),
                **epoch)
        # An operator override is what the panel will now show.
        return await _correct(request, apply, lambda obj: {
            "label": str(obj.attributes.get("label_override") or obj.label)})

    async def object_caption(request) -> JSONResponse:
        """Write one object's caption; an empty one hands it back to the VLM."""
        async def apply(body, epoch):
            return await object_mutations.apply_caption(
                object_id=request.path_params["object_id"],
                caption=str(body.get("caption") or ""),
                note="captioned from the web UI", **epoch)
        return await _correct(request, apply, lambda obj: {
            "caption": obj.caption, "caption_source": obj.caption_source})

    async def object_delete(request) -> JSONResponse:
        """Remove one object, and its photographs."""
        async def apply(body, epoch):
            out = await object_mutations.remove_object(
                object_id=request.path_params["object_id"],
                note=str(body.get("note") or "deleted from the web UI"),
                **epoch)
            if object_views is not None:
                await asyncio.to_thread(object_views.forget,
                                        object_views.partition(map_binding),
                                        out[0])
            return out
        return await _correct(request, apply,
                              lambda deleted_id: {"deleted_id": deleted_id})

    async def map_preview(request):
        """A saved map's occupancy grid, as Scene kept it at Save."""
        image, _ = (map_meta.read_preview(request.path_params["map_id"])
                    if map_meta is not None else (None, None))
        if image is None:
            return PlainTextResponse("this map has no preview", status_code=404)
        return Response(image, media_type="image/png",
                        headers={"Cache-Control": "no-cache"})

    async def map_contents(request) -> JSONResponse:
        """One saved map's regions, objects and grid geometry, without
        loading it."""
        map_id = _sanitize_map_id(request.path_params["map_id"])

        def collect() -> dict:
            grid = map_meta.read_preview(map_id)[1] if map_meta is not None else None

            regions = anno_store.read_saved(map_id) if anno_store is not None else []

            objects: list = []
            if map_meta is not None and object_store is not None:
                meta = map_meta.read(map_id)
                if meta is not None:
                    try:
                        rows = object_store.load_all(
                            partition=meta.object_partition)
                    except Exception:  # noqa: BLE001
                        rows = []
                    for obj in rows or []:
                        pose = getattr(obj, "pose", None)
                        objects.append({
                            "id": getattr(obj, "object_id", ""),
                            "display_name": getattr(obj, "display_name", ""),
                            "label": getattr(obj, "label", "object"),
                            "x": float(getattr(pose, "x", 0.0) or 0.0),
                            "y": float(getattr(pose, "y", 0.0) or 0.0),
                            "confidence": float(getattr(obj, "confidence", 0.0) or 0.0),
                        })
            return {
                "ok": True, "map_id": map_id, "grid": grid,
                "regions": regions, "objects": objects,
            }

        return JSONResponse(await asyncio.to_thread(collect))

    async def _resolve_object(object_id: str):
        """`(object, live_id, departure)` for a possibly stale id. Inferred
        forwarding is not followed: a wrong picture is worse than none."""
        async with registry.lock():
            live_id, departure = registry.resolve_id(
                object_id, follow_inferred=False)
            obj = registry._objects.get(live_id) if live_id else None
        return obj, live_id, departure

    async def object_resolve(request) -> JSONResponse:
        """The live id standing for a possibly merged or replaced one, or ""."""
        async with registry.lock():
            live_id, _ = registry.resolve_id(request.path_params["object_id"])
        return JSONResponse({"ok": True, "id": live_id or ""})

    def _views_unavailable() -> JSONResponse:
        return JSONResponse(
            {"ok": False,
             "detail": "object views are not configured for this deployment; "
                       "set SCENE_OBJECT_VIEWS_DIR to store them"},
            status_code=503)

    async def object_views_list(request) -> JSONResponse:
        """What pictures exist of one object, best-looking first."""
        if object_views is None:
            return _views_unavailable()
        requested = request.path_params["object_id"]
        obj, live_id, departure = await _resolve_object(requested)
        if obj is None:
            return JSONResponse(
                {"ok": False, "object_id": requested,
                 "detail": (f"object is gone ({departure.get('reason')})"
                            if departure else "unknown object"),
                 "departure": departure},
                status_code=404)
        map_id = object_views.partition(map_binding)
        rows = await asyncio.to_thread(object_views.views, map_id, live_id)
        return JSONResponse({
            "ok": True,
            "object_id": live_id,
            "requested_id": requested,
            "views": [
                {**row,
                 "url": f"/api/objects/{quote(live_id, safe='')}"
                        f"/views/{int(row.get('index', 0))}.jpg"}
                for row in rows
            ],
        })

    async def object_view_image(request):
        """One stored view, as the JPEG it was written as."""
        if object_views is None:
            return PlainTextResponse("object views are not configured", 503)
        obj, live_id, _departure = await _resolve_object(
            request.path_params["object_id"])
        if obj is None:
            return PlainTextResponse("unknown object", status_code=404)
        try:
            index = int(request.path_params["index"])
        except (TypeError, ValueError):
            return PlainTextResponse("view index must be a number", 400)
        map_id = object_views.partition(map_binding)
        data = await asyncio.to_thread(object_views.read, map_id, live_id, index)
        if not data:
            return PlainTextResponse("no such view", status_code=404)
        # A better look at the same side replaces the file in place.
        return Response(data, media_type="image/jpeg",
                        headers={"Cache-Control": "no-cache"})

    async def objects_flush(request) -> JSONResponse:
        """Drop every perceived object, through the same coordinator call as
        the MCP tool. Persists by default: a flushed set should not return at
        the next boot."""
        async def apply(body, epoch):
            persist = body.get("persist_to_snapshot")
            return await object_mutations.flush_objects(
                persist_to_snapshot=True if persist is None else bool(persist),
                note=str(body.get("note") or "flushed from the web UI"),
                **epoch)
        return await _correct(request, apply,
                              lambda deleted: {"deleted": deleted})

    async def logs_api(request) -> JSONResponse:
        """What scribe appended since `cursor` ({tag: byte offset} from the
        previous response); without one, each file's tail."""
        directory = _scribe_dir()
        if directory is None:
            return JSONResponse({
                "ok": False,
                "detail": "SCRIBE_LOG_DIR is not set for this deployment",
                "entries": [], "cursor": {}, "tags": [],
            }, status_code=200)

        try:
            cursor = json.loads(request.query_params.get("cursor") or "{}")
            if not isinstance(cursor, dict):
                cursor = {}
        except ValueError:
            cursor = {}

        wanted = [t for t in (request.query_params.get("tags") or "").split(",") if t]

        def collect() -> dict:
            files = sorted(directory.glob("*.log"))
            entries: list[dict] = []
            next_cursor: dict[str, int] = {}
            tags: list[str] = []
            for path in files:
                tag = path.stem
                tags.append(tag)
                if wanted and tag not in wanted:
                    # Advance skipped tags too, or re-enabling one replays
                    # the gap.
                    try:
                        next_cursor[tag] = path.stat().st_size
                    except OSError:
                        pass
                    continue
                offset = cursor.get(tag)
                rows, consumed = _read_log_slice(
                    path, int(offset) if isinstance(offset, int) else None)
                entries.extend(rows)
                next_cursor[tag] = consumed
            entries.sort(key=lambda e: e.get("ts") or "")
            return {"ok": True, "entries": entries, "cursor": next_cursor,
                    "tags": sorted(tags), "dir": str(directory)}

        return JSONResponse(await asyncio.to_thread(collect))

    async def logs_page(request) -> HTMLResponse:
        return HTMLResponse(_shell_page("/logs", _LOGS_BODY, "scene — logs"))

    async def maps_page(request) -> HTMLResponse:
        return HTMLResponse(_shell_page("/maps", _MAPS_BODY, "scene — maps"))

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

    async def viewer_focus(request):
        """Point the viewer's camera at one object (rerun has no selection
        setter, so the blueprint is resent with the object as the target)."""
        if rerun_sink is None:
            return JSONResponse({"ok": False, "why": "no viewer"}, status_code=409)
        body = (await _anno_body(request) or {})
        object_id = str(body.get("object_id") or "")
        if not object_id:
            return JSONResponse({"ok": False, "why": "no object"}, status_code=400)
        objects = await registry.snapshot()
        obj = objects.get(object_id)
        if obj is None:
            return JSONResponse({"ok": False, "why": "unknown object"},
                                status_code=404)
        ok = rerun_sink.look_at(
            (float(obj.pose.x), float(obj.pose.y), float(obj.pose.z)))
        return JSONResponse({"ok": bool(ok)})

    async def rerun_host(_request) -> HTMLResponse:
        """The viewer page; opening it is what starts the viewer."""
        if rerun_sink is not None:
            await asyncio.to_thread(rerun_sink.ensure_started)
        return HTMLResponse(_RERUN_HOST_HTML)

    async def cost_page(_request):
        """The cost view on its own, so it can sit in the combined layout."""
        return HTMLResponse(_COST_HTML)

    async def cost(_request):
        """What this backend is costing right now, as one JSON object.

        A backend chosen for being light has to be able to show it. The
        numbers are the scene container's own: cgroup memory and CPU rather
        than the host's, the GPU memory this process has actually reserved
        rather than whatever else shares the card, and the perception loop's
        own last tick.
        """
        return JSONResponse(_cost_sample(detector))

    routes = [
        Mount("/assets/rerun",
              app=_ExtensionlessJs(directory=RERUN_VIEWER_DIR, check_dir=False),
              name="rerun-assets"),
        Route("/", index, methods=["GET"]),
        Route("/cost", cost_page, methods=["GET"]),
        Route("/api/cost", cost, methods=["GET"]),
        Route("/2d", index2d, methods=["GET"]),
        Route("/rerun", rerun_host, methods=["GET"]),
        Route("/cam", cam, methods=["GET"]),
        Route("/maps", maps_page, methods=["GET"]),
        Route("/logs", logs_page, methods=["GET"]),
        Route("/api/logs", logs_api, methods=["GET"]),
        Route("/api/maps/{map_id}/preview", map_preview, methods=["GET"]),
        Route("/api/maps/{map_id}/contents", map_contents, methods=["GET"]),
        Route("/api/objects/flush", objects_flush, methods=["POST"]),
        Route("/api/viewer/focus", viewer_focus, methods=["POST"]),
        Route("/api/objects/{object_id}/resolve", object_resolve,
              methods=["GET"]),
        Route("/api/objects/{object_id}/views", object_views_list,
              methods=["GET"]),
        Route("/api/objects/{object_id}/views/{index}.jpg",
              object_view_image, methods=["GET"]),
        Route("/api/objects/{object_id}/label", object_label,
              methods=["POST"]),
        Route("/api/objects/{object_id}/caption", object_caption,
              methods=["POST"]),
        Route("/api/objects/{object_id}", object_delete,
              methods=["DELETE"]),
        Route("/regions", regions_page, methods=["GET"]),
        # Deprecated aliases of /regions, / and /api/regions.
        Route("/user", regions_page, methods=["GET"]),
        Route("/3d", lambda _r: RedirectResponse("/"), methods=["GET"]),
        Route("/api/annotations", annotations_list, methods=["GET"]),
        Route("/api/annotations", annotations_create, methods=["POST"]),
        Route("/api/annotations/{annotation_id}", annotations_update,
              methods=["PUT"]),
        Route("/api/annotations/{annotation_id}", annotations_delete,
              methods=["DELETE"]),
        Route("/api/state", state, methods=["GET"]),
        Route("/api/objects3d", objects3d, methods=["GET"]),
        Route("/api/viewer", viewer_url, methods=["GET"]),
        Route("/proxy", rerun_data, methods=["GET", "POST", "OPTIONS"]),
        Route("/api/camera", camera_state, methods=["GET"]),
        Route("/api/regions", annotations_list, methods=["GET"]),
        Route("/api/regions", annotations_create, methods=["POST"]),
        Route("/api/regions/{annotation_id}", annotations_update,
              methods=["PUT"]),
        Route("/api/regions/{annotation_id}", annotations_delete,
              methods=["DELETE"]),
        Route("/api/maps", maps_list, methods=["GET"]),
        Route("/api/maps/save", maps_save, methods=["POST"]),
        Route("/api/maps/load", maps_load, methods=["POST"]),
        Route("/api/maps/delete", maps_delete, methods=["POST"]),
        Route("/api/maps/pose_estimate", maps_pose_estimate, methods=["POST"]),
        # The viewer's gRPC-web calls (`/rerun.sdk_comms.<version>...`).
        # Last: as a two-segment catch-all it would shadow POST /api/regions.
        Route("/{service}/{method}", rerun_grpc, methods=["POST", "OPTIONS"]),
    ]
    return Starlette(routes=routes, middleware=[Middleware(_JsonPostsOnly)])


class _JsonPostsOnly:
    """Refuse POSTs a cross-site page could send without a CORS preflight.

    Such a request can only carry a form or text/plain body; every POST this
    service accepts is JSON or gRPC-web.
    """

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] == "http" and scope["method"] == "POST":
            ctype = dict(scope["headers"]).get(b"content-type", b"")
            if not ctype.startswith((b"application/json", b"application/grpc")):
                await JSONResponse({"ok": False, "detail": "expected a JSON body"},
                                   status_code=415)(scope, receive, send)
                return
        await self.app(scope, receive, send)


# Rolling CPU accounting: cgroup gives cumulative microseconds, so a rate needs
# the previous reading. One sampler for the process, so every viewer of the page
# sees the same series rather than each computing its own from its own last poll.
_cost_prev = {"cpu_usec": 0.0, "at": 0.0}


def _read_first(path, cast=float, default=None):
    """First whitespace-separated field of a /sys or /proc file, or default."""
    try:
        with open(path, encoding="utf-8") as f:
            return cast(f.read().split()[0])
    except (OSError, ValueError, IndexError):
        return default


def _cgroup_cpu_usec():
    """Cumulative CPU microseconds for this container, cgroup v2 then v1."""
    try:
        with open("/sys/fs/cgroup/cpu.stat", encoding="utf-8") as f:
            for line in f:
                if line.startswith("usage_usec"):
                    return float(line.split()[1])
    except OSError:
        pass
    ns = _read_first("/sys/fs/cgroup/cpuacct/cpuacct.usage")
    return ns / 1000.0 if ns is not None else None


def _gpu_sample():
    """GPU memory this process reserved, and the card's utilisation.

    Torch knows what this process holds, which is the honest number for "what
    does this backend cost"; the card's total and utilisation come from NVML
    when it is available and are shared with whatever else runs on the GPU.
    """
    out = {}
    try:
        import torch
        if torch.cuda.is_available():
            out["gpu_reserved_mb"] = torch.cuda.memory_reserved() / 1e6
            out["gpu_allocated_mb"] = torch.cuda.memory_allocated() / 1e6
    except Exception:  # noqa: BLE001
        pass
    try:
        import pynvml
        pynvml.nvmlInit()
        h = pynvml.nvmlDeviceGetHandleByIndex(0)
        mem = pynvml.nvmlDeviceGetMemoryInfo(h)
        out["gpu_card_used_mb"] = mem.used / 1e6
        out["gpu_card_total_mb"] = mem.total / 1e6
        out["gpu_util_pct"] = pynvml.nvmlDeviceGetUtilizationRates(h).gpu
    except Exception:  # noqa: BLE001
        pass
    return out


def _cost_sample(detector):
    """One reading of what the running perception backend costs."""
    now = time.time()
    sample = {"at": now}

    backend = getattr(detector, "backend_name", None)
    if backend is None and detector is not None:
        backend = type(detector).__name__
    sample["backend"] = backend or "none"

    mem = _read_first("/sys/fs/cgroup/memory.current")
    if mem is None:
        mem = _read_first("/sys/fs/cgroup/memory/memory.usage_in_bytes")
    if mem is not None:
        sample["mem_mb"] = mem / 1e6
    limit = _read_first("/sys/fs/cgroup/memory.max", cast=str)
    if limit not in (None, "max"):
        try:
            sample["mem_limit_mb"] = float(limit) / 1e6
        except ValueError:
            pass

    usec = _cgroup_cpu_usec()
    if usec is not None:
        prev_usec, prev_at = _cost_prev["cpu_usec"], _cost_prev["at"]
        if prev_at and now > prev_at:
            sample["cpu_pct"] = (usec - prev_usec) / (now - prev_at) / 1e4
        _cost_prev.update(cpu_usec=usec, at=now)

    sample.update(_gpu_sample())
    for attr, key in (("last_tick_s", "tick_s"), ("_last_tick_s", "tick_s"),
                      ("_tick_idx", "ticks"), ("_skipped_frames", "frames_skipped")):
        v = getattr(detector, attr, None)
        if v is not None and key not in sample:
            sample[key] = v
    n = getattr(detector, "_map_objects", None)
    if n is not None:
        sample["objects"] = len(n)
    return sample


_COST_HTML = _asset("cost.html")
_INDEX_CAM_HTML = _asset("camera.html")


def _regions_html() -> str:
    """The regions page, with the shared controls and scripts filled in."""
    return (_REGIONS_HTML
            .replace("__CONTROLS__", _CONTROLS_CSS)
            .replace("__LABELS__", _LABELS_JS)
            .replace("__I18N_RUNTIME__", _i18n_js()))


_REGIONS_HTML = _asset("map_page.html")
