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
from starlette.responses import (HTMLResponse, JSONResponse,
                                 PlainTextResponse, Response,
                                 StreamingResponse)
from starlette.routing import Route

from robonix_api import ATLAS

from .annotations import validate_annotation_fields
from .map_binding import sanitize_map_id as _sanitize_map_id
from .map_meta import make_meta
from .state import ObjectRegistry

# ── Page assets ────────────────────────────────────────────────────────────
# The markup, stylesheets and scripts live in `web_assets/` as real .html,
# .css and .js files rather than as triple-quoted strings in this module.
# They were 61% of it, which cost an editor's highlighting, every linter, and
# a diff that could be read -- and made each patch depend on matching exact
# indentation inside a string literal.
#
# Read once at import: they do not change while the service runs, and a
# per-request read would put a filesystem call in the path of every page.
_ASSET_DIR = Path(__file__).resolve().parent / "web_assets"


def _asset(name: str) -> str:
    """One page asset, by file name, read from `web_assets/`."""
    return (_ASSET_DIR / name).read_text(encoding="utf-8")


log = logging.getLogger(__name__)


_INDEX_HTML = _asset("map_2d.html")


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


def occupancy_payload(hub: Any) -> Optional[dict]:
    """The occupancy snapshot the UI and the 3D viewer share.

    A public name for the private builder below, so the viewer does not reach
    into this module's internals to draw the floor the objects stand on.
    """
    return _occupancy_payload(hub)


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
            # Some camera providers publish BGRA8.
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
                   map_binding: Optional[dict] = None,
                   robot_geometry: Any = None) -> dict:
    """Serialise the registry + relations + map into the small JSON
    shape the page consumes. Done in one snapshot so the page never
    sees a half-updated registry. The "relations" field shows the fast
    geometric slice (``reachable_by`` only, under the VLM-primary graph);
    the "scene_graph" field shows the full composed graph (geometric +
    image-grounded relational/semantic edges). "annotations" carries the
    user-drawn regions/POIs (map-frame meters, same coordinates as objects)
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
        "robot_footprint": (
            robot_geometry.current().to_json()
            if robot_geometry is not None and robot_geometry.current() is not None
            else None
        ),
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


# Maps first: nothing else can be operated until one is bound, so it is the
# page a reader starts on and the one they come back to when switching.
# ── Interface language ─────────────────────────────────────────────────────
# One table, two documents: the shell and the view inside its iframe both read
# the same key, and the choice lives in localStorage so it survives a reload
# and is shared across the frame boundary without a server round trip.
#
# Keys, not English text. Matching on the English means a typo fix silently
# drops the translation, and it makes the table impossible to audit for gaps.
_STRINGS: dict[str, dict[str, str]] = {
    # the sidebar
    "nav.maps":        {"en": "maps",         "zh": "地图"},
    "nav.semantic":    {"en": "semantic map", "zh": "语义地图"},
    "nav.2d":          {"en": "2D map",       "zh": "平面图"},
    "nav.cam":         {"en": "camera",       "zh": "相机"},
    "nav.regions":     {"en": "regions",      "zh": "区域"},
    "nav.logs":        {"en": "logs",          "zh": "日志"},
    "nav.lang":        {"en": "中文",          "zh": "English"},
    "nav.lang.title":  {"en": "switch to Chinese", "zh": "switch to English"},
    # the dock
    "dock.objects":    {"en": "Objects",   "zh": "物体"},
    "dock.relations":  {"en": "Relations", "zh": "关系"},
    "dock.robot":      {"en": "Robot",     "zh": "机器人"},
    "dock.objects.hint":   {"en": "what the registry holds",
                            "zh": "registry 里有什么"},
    "dock.relations.hint": {"en": "how they sit together",
                            "zh": "它们彼此怎么摆"},
    "dock.robot.hint":     {"en": "where it thinks it is",
                            "zh": "机器人认为自己在哪"},
    "dock.collapse":   {"en": "collapse the dock", "zh": "折叠面板"},
    "dock.back":       {"en": "back to the list", "zh": "返回列表"},
    "dock.rename":     {"en": "Rename",  "zh": "重命名"},
    "dock.delete":     {"en": "Delete",  "zh": "删除"},
    "dock.class":      {"en": "class",        "zh": "类别"},
    "dock.id":         {"en": "id",           "zh": "标识"},
    "dock.conf":       {"en": "confidence",   "zh": "置信度"},
    "dock.obs":        {"en": "observations", "zh": "观测次数"},
    "dock.pos":        {"en": "position",     "zh": "位置"},
    "dock.missing":    {"en": "not seen recently", "zh": "近期未见"},
    "dock.renamePrompt": {"en": "New name for this object",
                          "zh": "给这个物体起个新名字"},
    "dock.deleteAsk":  {"en": "Delete this object from the map?",
                        "zh": "把这个物体从地图上删除？"},
    "dock.save":       {"en": "Save",    "zh": "保存"},
    "dock.cancel":     {"en": "Cancel",  "zh": "取消"},
    "dock.gone":       {"en": "this object is no longer in the map",
                        "zh": "这个物体已不在地图里"},
    "dock.resize":     {"en": "drag to resize",    "zh": "拖动改变宽度"},
    "dock.empty.objects":   {"en": "nothing in the registry yet",
                             "zh": "registry 里还没有东西"},
    "dock.empty.relations": {"en": "no relations inferred yet",
                             "zh": "还没有推断出关系"},
    "dock.empty.robot":     {"en": "no fix yet", "zh": "还没有定位"},
    # map binding
    "map.temporary":   {"en": "temporary, unsaved", "zh": "临时地图，未保存"},
    "note.title":      {"en": "Temporary map", "zh": "临时地图（未保存）"},
    "note.body":       {"en": "Marking and recognition work normally here. "
                              "Saving this session under a name keeps the "
                              "regions and objects with it; without that, "
                              "they end with the session.",
                        "zh": "标记区域和识别物体都照常可用。把本次会话命名保存后，"
                              "区域和物体会一起存进去；不保存则随会话结束丢失。"},
    "note.link":       {"en": "Go to maps", "zh": "前往地图管理"},
    # the map form
    "form.mapid":      {"en": "Map ID",        "zh": "地图 ID"},
    "form.save":       {"en": "Save current",  "zh": "保存当前"},
    "form.refresh":    {"en": "Refresh",       "zh": "刷新"},
    "form.pose":       {"en": "Pose estimate", "zh": "位姿估计"},
    "form.mode":       {"en": "Map mode:",     "zh": "地图模式："},
    "form.nomaps":     {"en": "No saved maps listed yet.",
                        "zh": "还没有已保存的地图。"},
    "status.ready":    {"en": "Ready.", "zh": "就绪。"},
    # region marking
    "btn.mark":        {"en": "✏ Mark region",     "zh": "✏ 标记区域"},
    "btn.cancelDraw":  {"en": "✕ Cancel drawing",  "zh": "✕ 取消绘制"},
    "regions.empty":   {"en": "No regions yet. Click “Mark region”, then click "
                              "on the map to outline one (double-click or "
                              "Enter to finish, Esc to cancel).",
                        "zh": "还没有区域。点「标记区域」，然后在地图上点击勾出"
                              "轮廓（双击或回车完成，Esc 取消）。"},
    # row actions and dialogs
    "btn.load":        {"en": "Load",        "zh": "加载"},
    "btn.delete":      {"en": "Delete",      "zh": "删除"},
    "btn.rename":      {"en": "Rename",      "zh": "重命名"},
    "btn.stillValid":  {"en": "Still valid", "zh": "仍然有效"},
    "btn.cancel":      {"en": "Cancel",      "zh": "取消"},
    "btn.ok":          {"en": "OK",          "zh": "确定"},
    "btn.close":       {"en": "Close",       "zh": "关闭"},
    # the log view
    "logs.title":      {"en": "Logs",     "zh": "日志"},
    "logs.live":       {"en": "Live",     "zh": "实时"},
    "logs.paused":     {"en": "Paused",   "zh": "已暂停"},
    "logs.clear":      {"en": "Clear",    "zh": "清空"},
    "logs.search":     {"en": "filter text…", "zh": "过滤文本…"},
    "logs.alltags":    {"en": "all tags", "zh": "全部来源"},
    "logs.waiting":    {"en": "waiting for lines…", "zh": "等待日志…"},
    "logs.nodir":      {"en": "This deployment did not set SCRIBE_LOG_DIR, so "
                              "there is no log directory to read.",
                        "zh": "本部署没有设置 SCRIBE_LOG_DIR，没有可读的日志目录。"},
    "logs.hint":       {"en": "levels are counted over what this page has "
                              "received, not the whole boot",
                        "zh": "级别计数只统计本页收到的部分，不是整次启动"},
    # status and progress text the map page writes as it works
    "st.saveValidate": {"en": 'Validate existing spatial artifact', "zh": '校验已有空间产物'},
    "st.savePersist": {"en": 'Persist regions and Scene objects', "zh": '保存区域与场景物体'},
    "st.saveVerify": {"en": 'Verify reusable map entry', "zh": '确认地图条目可复用'},
    "st.saveSnapshot": {"en": 'Snapshot the live spatial map', "zh": '快照当前空间地图'},
    "st.saveArtifact": {"en": 'Verify artifact and preview', "zh": '校验产物并生成预览'},
    "st.updatedFor": {"en": 'Updated scene data for', "zh": '已更新场景数据：'},
    "st.updated": {"en": 'Scene data updated', "zh": '场景数据已更新'},
    "st.saveFailed": {"en": 'Save validation failed', "zh": '保存校验失败'},
    "st.loadValidate": {"en": 'Validate saved spatial artifact', "zh": '校验已保存的空间产物'},
    "st.loadSwitch": {"en": 'Switch Mapping to localization mode', "zh": '切换建图为定位模式'},
    "st.loadGrid": {"en": 'Wait for a fresh occupancy grid', "zh": '等待新的占据栅格'},
    "st.loadRestore": {"en": 'Restore regions and Scene objects', "zh": '恢复区域与场景物体'},
    "st.poseClick": {"en": 'Click pose on map', "zh": '在地图上点击位姿'},
    "st.poseSent": {"en": 'Pose estimate sent.', "zh": '位姿估计已发送。'},
    "st.refreshing": {"en": 'Refreshing maps...', "zh": '正在刷新地图列表…'},
    "st.refreshed": {"en": 'Map list refreshed.', "zh": '地图列表已刷新。'},
    "st.drawHint": {"en": 'Click to add corners · double-click or Enter to finish (≥3) · Esc to cancel', "zh": '点击添加顶点 · 双击或回车完成（≥3）· Esc 取消'},
    "st.nameRequired": {"en": 'Region name is required.', "zh": '区域名称不能为空。'},
    "st.liveUnsaved": {"en": 'Live mapping session is not saved yet. Enter a Map ID, then Save current. ', "zh": '当前是未保存的建图会话。填写地图 ID 后点「保存当前」。'},
    # Sentences that carry a value. The placeholder stays inside the
    # translated sentence so each language can put it where it belongs;
    # Chinese does not order these the way English does.
    "regions.legend": {"en": 'drag to pan · wheel to zoom', "zh": '拖动平移 · 滚轮缩放'},
    "st.verbSave": {"en": 'Save', "zh": '保存'},
    "st.verbLoad": {"en": 'Load', "zh": '加载'},
    "st.opFailed": {"en": '{verb} {id} failed: {error}', "zh": '{verb}「{id}」失败：{error}'},
    "st.opFailedTitle": {"en": '{verb} failed', "zh": '{verb}失败'},
    "st.unexpected": {"en": 'Unexpected error; the editor has been unlocked.', "zh": '发生意外错误；编辑器已解锁。'},
    "st.saved": {"en": 'Saved', "zh": '已保存'},
    "st.mapSaved": {"en": 'Map saved', "zh": '地图已保存'},
    "st.savedReport": {"en": '{what} {id}; spatial artifact {artifact}; regions {regions}.', "zh": '{what}「{id}」；空间地图{artifact}；区域 {regions} 个。'},
    "st.artifactOk": {"en": 'ok', "zh": '正常'},
    "st.artifactBad": {"en": 'failed', "zh": '失败'},
    "st.saveFailedFor": {"en": 'Save {id} failed: {detail}', "zh": '保存「{id}」失败：{detail}'},
    "st.loadedHint": {"en": 'Loaded {id}. Mapping requested localization mode; use Pose estimate if the robot pose is off.', "zh": '已加载「{id}」。已请求建图切到定位模式；若机器人位姿不对，用「位姿估计」修正。'},
    "st.loadFailedFor": {"en": 'Load {id} failed: {detail}', "zh": '加载「{id}」失败：{detail}'},
    "st.deletedFor": {"en": 'Deleted {id}.', "zh": '已删除「{id}」。'},
    "st.deleteFailedFor": {"en": 'Delete {id} failed.', "zh": '删除「{id}」失败。'},
    "st.poseFailedAt": {"en": 'Pose estimate failed for ({x}, {y}).', "zh": '在 ({x}, {y}) 处位姿估计失败。'},
    "st.selectedHint": {"en": 'Selected {id}. Click Load to enter localization mode.', "zh": '已选中「{id}」。点击「加载」进入定位模式。'},
    "st.notLoadable": {"en": 'Map {id} is not loadable: {detail}', "zh": '地图「{id}」无法加载：{detail}'},
    "st.invalidArtifact": {"en": 'invalid spatial artifact', "zh": '空间地图无效'},
    "st.localization": {"en": 'Localization mode is active. Use Pose estimate if the robot pose is off.', "zh": '定位模式已启用。位姿不对时用「位姿估计」。'},
    # the status bar every page carries
    "bar.mapping":     {"en": "building the map", "zh": "正在建图"},
    "bar.localizing":  {"en": "localizing",       "zh": "定位中"},
    "bar.idle":        {"en": "mode unknown",     "zh": "模式未知"},
    "bar.mode":        {"en": "robot",            "zh": "机器人"},
    "bar.map":         {"en": "active map",       "zh": "当前地图"},
    "bar.temporary":   {"en": "temporary, not saved",
                        "zh": "临时会话，未保存"},
    "bar.viewer":       {"en": "viewer",   "zh": "可视化"},
    "bar.viewerRerun":  {"en": "rerun",    "zh": "rerun"},
    "bar.viewerBuiltin": {"en": "built-in", "zh": "内置"},
    "bar.viewerFailed": {"en": "rerun did not start:",
                         "zh": "rerun 未能启动："},
    "bar.offline":     {"en": "scene is not responding",
                        "zh": "scene 无响应"},
    # the maps library
    "maps.newName":    {"en": "Name for this session", "zh": "给本次会话命名"},
    "maps.save":       {"en": "Save current session",  "zh": "保存当前会话"},
    "maps.refresh":    {"en": "Refresh",   "zh": "刷新"},
    "maps.load":       {"en": "Load",      "zh": "加载"},
    "maps.delete":     {"en": "Delete",    "zh": "删除"},
    "maps.cancel":     {"en": "Cancel",    "zh": "取消"},
    "maps.saved":      {"en": "saved",     "zh": "保存于"},
    "maps.size":       {"en": "artifact size",   "zh": "产物大小"},
    "maps.inUse":      {"en": "in use",    "zh": "使用中"},
    "maps.broken":     {"en": "cannot be loaded", "zh": "无法加载"},
    "maps.noPreview":  {"en": "no preview", "zh": "没有预览图"},
    "maps.working":    {"en": "working…",   "zh": "处理中…"},
    "maps.saving":     {"en": "saving…",    "zh": "保存中…"},
    "maps.failed":     {"en": "failed",     "zh": "失败"},
    "maps.nameRequired": {"en": "Give this session a name first.",
                          "zh": "先给这次会话起个名字。"},
    "maps.details":    {"en": "Details", "zh": "详情"},
    "maps.back":       {"en": "Back",    "zh": "返回"},
    "maps.regions":    {"en": "regions", "zh": "区域"},
    "maps.objects":    {"en": "objects", "zh": "物体"},
    "maps.nothing":    {"en": "none",    "zh": "无"},
    "maps.health":     {"en": "artifact health", "zh": "产物状态"},
    "maps.healthy":    {"en": "integrity check passed", "zh": "完整性校验通过"},
    "maps.id":         {"en": "id",        "zh": "标识"},
    "maps.artifactPath": {"en": "artifact path", "zh": "产物路径"},
    "maps.previewPath":  {"en": "preview path",  "zh": "预览图路径"},
    "maps.empty":      {"en": "No saved maps yet. Explore, then save this "
                              "session under a name.",
                        "zh": "还没有保存过地图。先探索，再把这次会话命名保存。"},
    # page titles
    "page.maps":       {"en": "Maps",    "zh": "地图"},
    "page.regions":    {"en": "Regions", "zh": "区域"},
}

_I18N_JS = _asset("i18n.js")


def _i18n_js() -> str:
    """The runtime with the table baked in."""
    import json
    return _I18N_JS.replace("__TABLE__", json.dumps(_STRINGS, ensure_ascii=False))





_NAV_LINKS = (
    ("/maps", "maps", "nav.maps"),
    ("/", "semantic map", "nav.semantic"),
    ("/2d", "2D map", "nav.2d"),
    ("/cam", "camera", "nav.cam"),
    ("/regions", "regions", "nav.regions"),
    ("/logs", "logs", "nav.logs"),
)


# One mark per destination, inline. Stroked rather than filled so they sit at
# the same visual weight as the label beside them at 15px, and inherit the
# link's colour so the active state needs no second rule.
_ICON = ('<svg class="ico" viewBox="0 0 24 24" fill="none" stroke="currentColor"'
         ' stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round">{}</svg>')

_NAV_ICONS = {
    # stacked sheets: the saved maps, one of which is bound
    "/maps": _ICON.format(
        '<path d="M4 7.5 12 4l8 3.5-8 3.5z"/><path d="m4 12 8 3.5 8-3.5"/>'
        '<path d="m4 16.5 8 3.5 8-3.5"/>'),
    # a box in space: the semantic map
    "/": _ICON.format(
        '<path d="M21 16V8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16z"/>'
        '<path d="m3.3 7 8.7 5 8.7-5"/><path d="M12 22V12"/>'),
    # a folded plan: the 2D map
    "/2d": _ICON.format(
        '<path d="M15 6 9 3 3 6v15l6-3 6 3 6-3V3z"/><path d="M9 3v15"/><path d="M15 6v15"/>'),
    # a lens: the camera
    "/cam": _ICON.format(
        '<path d="M14.5 4h-5L7 7H4a2 2 0 0 0-2 2v9a2 2 0 0 0 2 2h16a2 2 0 0 0 2-2V9a2 2 0 0 0-2-2h-3z"/>'
        '<circle cx="12" cy="13" r="3.2"/>'),
    # stacked lines, one short: the log
    "/logs": _ICON.format(
        '<path d="M4 6.5h13M4 11h16M4 15.5h11M4 20h7"/>'),
    # an outlined area with a pin: regions
    "/regions": _ICON.format(
        '<path d="M4 6.5 10 4l4 2.5L20 4v13.5L14 20l-4-2.5L4 20z"/>'
        '<circle cx="12" cy="10.5" r="1.6"/>'),
}



# ── Scribe log reader ──────────────────────────────────────────────────────
# One file per tag, one JSON object per line, append-only. That last property
# is what makes a byte offset a valid cursor and the poll cheap.

# Ordered so "at least warning" is a comparison rather than a set membership.
_LOG_LEVELS = ("debug", "info", "warn", "error")
_LOG_LEVEL_INDEX = {name: i for i, name in enumerate(_LOG_LEVELS)}

# How much of each file to show on a first load. Enough to cover a boot's
# worth of interesting lines, small enough that opening the page is instant.
_LOG_TAIL_BYTES = 120_000
# A single poll's ceiling, so a service that suddenly floods cannot make the
# response unbounded.
_LOG_MAX_NEW_BYTES = 400_000


def _scribe_dir() -> Optional[Path]:
    """Where rbnx put this deployment's logs, or None if it did not say.

    A native run outside rbnx has no SCRIBE_LOG_DIR, and the honest answer
    then is that there is nothing to show -- not a guessed path that silently
    reads someone else's deployment.
    """
    raw = (os.environ.get("SCRIBE_LOG_DIR") or "").strip()
    if not raw:
        return None
    path = Path(raw)
    return path if path.is_dir() else None


def _normalise_level(value: str) -> str:
    name = str(value or "").strip().lower()
    if name in ("warning",):
        return "warn"
    if name in ("critical", "fatal"):
        return "error"
    if name in ("trace",):
        return "debug"
    return name if name in _LOG_LEVEL_INDEX else "info"


def _read_log_slice(path: Path, offset: Optional[int]) -> tuple[list[dict], int]:
    """Lines appended since `offset`, and the offset to use next.

    `offset` of None means "first look": take the tail rather than the file,
    which on a long-running deployment is the difference between a page that
    opens now and one that ships a hundred megabytes first.

    A file that shrank was rotated or replaced, so the old offset describes a
    file that no longer exists and the only correct thing is to start over.
    """
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

    # A read can land mid-line at both ends: drop a leading partial whenever
    # we did not start at a known line boundary, and keep a trailing partial
    # out of the cursor so the next poll picks the whole line up.
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
            # Not every line is scribe's: a package that writes a bare
            # traceback to stderr lands here too, and dropping it would hide
            # exactly the thing someone opened this page to find.
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


def _read_grid_meta(map_dir: Path) -> Optional[dict]:
    """The saved grid's geometry, from the metadata mapping writes beside it.

    A tiny `key: value` file with one list; parsed here rather than with a
    YAML dependency, because that is the whole grammar it uses and pulling in
    a parser for six keys is not a trade worth making.
    """
    path = map_dir / "meta.yaml"
    if not path.is_file():
        return None
    out: dict = {}
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            if ":" not in line:
                continue
            key, raw = line.split(":", 1)
            key, raw = key.strip(), raw.strip()
            if raw.startswith("[") and raw.endswith("]"):
                try:
                    out[key] = [float(x) for x in raw[1:-1].split(",")]
                except ValueError:
                    continue
            else:
                try:
                    out[key] = float(raw)
                except ValueError:
                    out[key] = raw
    except OSError:
        return None
    origin = out.get("origin") or [0.0, 0.0, 0.0]
    try:
        return {
            "resolution": float(out.get("resolution") or 0.0),
            "width": int(out.get("width") or 0),
            "height": int(out.get("height") or 0),
            "origin_x": float(origin[0]),
            "origin_y": float(origin[1]),
            "saved_at": out.get("saved_at") or "",
        }
    except (TypeError, ValueError, IndexError):
        return None


def _read_saved_regions(base_dir: Optional[str], map_id: str) -> list:
    """One map's regions, read from its own file.

    The live store is bound to whichever map the session is on; rebinding it
    to read a different one would move the session. The storage is a file per
    map, so the other map's file is simply read.
    """
    if not base_dir:
        return []
    path = Path(base_dir).expanduser() / f"{_sanitize_map_id(map_id)}.json"
    if not path.is_file():
        return []
    try:
        blob = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    rows = blob.get("annotations") if isinstance(blob, dict) else blob
    return rows if isinstance(rows, list) else []


def _nav(active: str) -> str:
    """The sidebar every page shares.

    Before this the pages had no links between them: reaching the annotation
    view from the map meant editing the address bar, and nothing on any page
    said the other views existed.
    """
    items = []
    for href, label, key in _NAV_LINKS:
        current = ' class="on"' if href == active else ""
        icon = _NAV_ICONS.get(href, "")
        # The English is in the markup so the page is readable before the
        # script runs and so a crawler or a screenshot of a dead page still
        # says something; the script replaces it with the chosen language.
        items.append(
            f'<a href="{href}"{current}>{icon}'
            f'<span data-i18n="{key}">{label}</span></a>'
        )
    return "".join(items)


# ── The object and relation list ───────────────────────────────────────────
# Which objects the registry holds, where they are, and which relations hold
# between them: the thing a reader checks the map against. It used to live on
# the combined layout, which stopped being the landing page when the viewer
# became rerun's -- and rerun draws the map but knows nothing about the
# registry behind it, so the list went with it. It is shared by both now: a
# floating panel over whatever view is underneath, fed by /api/state.
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
    icon = ('<svg viewBox="0 0 24 24" fill="none" stroke="currentColor"'
            ' stroke-width="1.7" stroke-linecap="round"'
            ' stroke-linejoin="round">{}</svg>')
    tabs = "".join(
        f'<button data-tab="{tid}" data-i18n-title="dock.{tid}.hint"'
        f' data-i18n-aria="dock.{tid}"'
        f' title="{label} — {hint}" aria-label="{label}">'
        f'{icon.format(path)}</button>'
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
        <button class="shutbtn" id="dock-shut" data-i18n-title="dock.collapse"
                data-i18n-aria="dock.collapse" title="collapse the dock"
                aria-label="collapse the dock">»</button>
      </div>
      <div class="panes">
        <div class="pane on" data-pane="objects">
          <table class="objs"><tbody id="dock-objs">
            <tr><td class="empty">—</td></tr>
          </tbody></table>
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


_SHELL_CSS = _asset("shell.css")
_CONTROLS_CSS = _asset("controls.css")
# Shared by both maps: the class palette and the label placer.
_LABELS_JS = _asset("labels.js")


# ── The log view ───────────────────────────────────────────────────────────
_LOGS_BODY = _asset("logs.html")
_MAPS_BODY = _asset("maps_page.html")



# ── The status bar ─────────────────────────────────────────────────────────
# Present on every page by construction: what the robot is doing, and which
# map it is doing it to. Those two facts decide what everything else on the
# screen means, and they used to be stated in one corner of one page.
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
  // The same poll the rest of the UI uses. A page that cannot reach scene
  // says so here: a stale map name is worse than an admitted disconnection.
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
        // An unnamed live session is mapping: there is no saved map to be
        // localising against yet.
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
                                          : 'bar.viewerBuiltin';
        viewEl.dataset.i18n = key;
        set(viewEl, t(key));
        // Amber only when rerun was expected and did not arrive: an install
        // that never had it is not in a degraded state.
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
    """Wrap page content in the shared sidebar.

    `info_panel` docks the object, relation and robot panels beside the
    page. The map pages carry it: rerun draws the map but knows nothing about
    the registry behind it, and "which objects does scene actually hold" is
    the question the map is opened to answer. It docks rather than floats so
    the view resizes around it instead of being covered by it.
    """
    css = _CONTROLS_CSS + _SHELL_CSS + (_DOCK_CSS if info_panel else "")
    dock = _dock_html() if info_panel else ""
    # The language runtime loads on every page, dock or not: the sidebar is
    # everywhere and the switch lives in it.
    script = "<script>" + _i18n_js() + """
    applyLang(langGet());
    document.getElementById('lang-switch').addEventListener('click', () => {
      langSet(langGet() === 'zh' ? 'en' : 'zh');
    });
    // A frame that loads later than the choice still needs telling.
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
        '<button class="lang" id="lang-switch" data-i18n="nav.lang"'
        ' data-i18n-title="nav.lang.title">中文</button></nav>'
        f"<main>{_STATUS_BAR}{body}</main>{dock}</div>{script}</body></html>"
    )



def _viewer_body(field: str, fallback: str) -> str:
    """Page body that fills an iframe from /api/viewer.

    The URL is fetched rather than baked in because the servers start after
    this module is imported, and because a viewer that failed to start has to
    say so: `fallback` is the page to link to instead, so a reader who cannot
    see the map is not left guessing which half is broken. The fetch itself is
    caught for the same reason — a rejected promise leaves the frame blank
    forever, which is the one failure that looks like the map simply being
    empty.
    """
    return (
        '<iframe id="v" title="scene viewer"></iframe>'
        '<script>'
        'function down(t){document.querySelector("main").innerHTML='
        '"<div class=\'msg\'>No viewer: " + t +'
        f'". <br><br>The built-in view is at <a href=\'{fallback}\'>'
        f'{fallback}</a>.</div>";}}'
        'fetch("/api/viewer").then(function(r){return r.json();})'
        '.then(function(d){'
        f'var u=d["{field}"];'
        'if(u){document.getElementById("v").src=u;}'
        'else{down(d.detail||"the viewer is not running");}'
        '}).catch(function(e){down(String(e));});'
        '</script>'
    )


def _framed(path: str, title: str) -> str:
    """A sub-page rendered inside the shared sidebar.

    The standalone pages are kept exactly as they are and embedded, so each one
    stays individually addressable for debugging while the sidebar is present
    everywhere. `?bare=1` is what stops the embedded copy from drawing a second
    sidebar inside itself.
    """
    return _shell_page(
        path, f'<iframe src="{path}?bare=1" title="{title}"></iframe>', title)


def _bare(request) -> bool:
    """True when the caller wants the page without the sidebar."""
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
             rerun_sink: Any = None) -> Starlette:
    """Build the Starlette ASGI app the entrypoint mounts on its own
    uvicorn server.

    Routes:
      GET /                — the semantic map, rendered by the embedded rerun
                             viewer, with the sidebar the other pages share
      GET /2d              — 2D top-down map (occupancy grid + objects)
      GET /3d              — 3D scene (point clouds + bbox; three.js)
      GET /cam             — camera stack (live RGB + depth)
      GET /user            — end-user map page (regions: draw / rename /
                             confirm-stale / delete; light object overlay)
      GET /api/state       — JSON for the 2D map
      GET /api/objects3d   — JSON for the 3D viz (per-object pcd + bbox)
      GET /api/camera      — JSON: latest RGB + depth frames
      /api/regions[..] — user annotation CRUD (see below)
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
      GET    /api/regions       → {ok, annotations: [...]}
      POST   /api/regions       body {kind, name, points, theta?}
                                    → {ok, annotation}
      PUT    /api/regions/{id}  body: any of {name, points, theta,
                                    stale:false} → {ok, annotation}
      DELETE /api/regions/{id}  → {ok}
    theta (heading, radians) is poi-only — a region carrying it is a 400.
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

    async def index(request) -> HTMLResponse:
        # Combined split layout: 2D map left, 3D viz right, each with
        # an expand-button that maximises the panel inside the page
        # (NOT browser-fullscreen). The two iframes embed the original
        # standalone /2d and /3d routes so they remain individually
        # bookmarkable / debuggable.
        #
        # The landing page is the semantic map: it is the view that answers
        # "is the perception any good", which is what this UI is opened for.
        # The other pages are one click away in the sidebar.
        #
        # A deployment whose viewer is the built-in one keeps the page it had.
        # Native installs do not ship rerun and must not be handed a broken
        # frame in place of a working layout.
        if _bare(request):
            # Served raw rather than through the shell, so the
            # shared controls travel with it the same way the
            # framed map page receives them.
            return HTMLResponse(
                _COMBINED_HTML.replace("__CONTROLS__", _CONTROLS_CSS))
        if rerun_sink is None or not rerun_sink.ready:
            return HTMLResponse(_framed("/", "scene — semantic map"))
        body = _viewer_body("url", "/3d")
        return HTMLResponse(_shell_page("/", body, "scene — semantic map",
                                        info_panel=True))

    async def index2d(request) -> HTMLResponse:
        # `?bare=1` is the built-in canvas map. It stays the page itself where
        # rerun is absent, and stays reachable everywhere: the combined layout
        # embeds it, and a native install has nothing else.
        if _bare(request):
            return HTMLResponse(
                _INDEX_HTML.replace("__LABELS__", _LABELS_JS))
        # Always the built-in renderer. rerun's top-down view is the 3D
        # recording seen from above, point clouds included, and from above a
        # point cloud hides the floor plan it is drawn over.
        return HTMLResponse(_shell_page(
            "/2d", '<iframe src="/2d?bare=1" title="2D map"></iframe>',
            "scene — 2D map", info_panel=True))

    async def state(_request) -> JSONResponse:
        payload = _state_payload(
            registry,
            hub,
            sg_store,
            anno_store,
            map_binding,
            robot_geometry,
        )
        # Which viewer is drawing. The page swaps to the built-in canvas on
        # its own when the sink is not up, and that swap was invisible: the
        # layout simply lost its docked panels with no reason given.
        live = rerun_sink is not None and rerun_sink.ready
        payload["viewer"] = {
            "backend": "rerun" if live else "builtin",
            # A deployment with no sink at all has not fallen back; the
            # built-in canvas is simply what it has.
            "fell_back": rerun_sink is not None and not live,
            "detail": "" if live else (
                rerun_sink.detail if rerun_sink is not None else ""),
        }
        return JSONResponse(payload)

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
        """POST /api/regions — validate {kind, name, points, theta?}
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
        """PUT /api/regions/{id} — partial update: any of name /
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
        """DELETE /api/regions/{id} — remove the annotation; 404 when
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
            # previously saved regions; a startup binding that happens to name
            # this map (lifecycle broadcast / env) is NOT a load.
            return _anno_error(409, (
                f"map {map_id} already has saved regions; load it "
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
                # every object/region on the next Load.
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
                # The public Save contract is geometry + regions + objects as
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
            # necessary to refresh regions/objects against the loaded map.
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
                        f"FAILED: {e} — aborted before regions/objects were "
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
                # regions were saved under; the sidecar's recorded value is the
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

    async def index3d(request) -> HTMLResponse:
        # The built-in three.js view. It stays reachable on its own path even
        # where rerun serves the landing page: a native install has no rerun,
        # and this is the only 3D view it has.
        if _bare(request):
            return HTMLResponse(_INDEX_3D_HTML)
        return HTMLResponse(_framed("/3d", "scene — built-in 3D"))

    # rerun serves the viewer application on one port and each page's log
    # stream on another, and the embedded frame pointed straight at them. That
    # is invisible on the robot and unusable off it: reading the map from a
    # laptop meant forwarding four ports, and forgetting one produced a blank
    # frame with no error. Scene proxies all of them under its own port, so the
    # whole UI — shell, viewer and data — travels over the one port an operator
    # already has to reach.
    #
    # The stream is gRPC-web over HTTP/1.1 (`application/grpc-web+proto`), not
    # HTTP/2 gRPC: status and trailers ride inside the body, so an ordinary
    # streaming reverse proxy carries it without special handling.
    _PROXY_SKIP = {"host", "content-length", "connection", "keep-alive",
                   "transfer-encoding", "upgrade"}

    async def _proxy(request, port: int, path: str, extra: dict = None):
        """Stream one request to a local rerun server and back."""
        import httpx

        url = f"http://127.0.0.1:{port}/{path.lstrip('/')}"
        if request.url.query:
            url = f"{url}?{request.url.query}"
        headers = {k: v for k, v in request.headers.items()
                   if k.lower() not in _PROXY_SKIP}
        # `trust_env=False` is load-bearing: httpx otherwise honours
        # HTTP_PROXY/ALL_PROXY from the environment, and both the robot and a
        # developer's machine usually have one set. The upstream here is
        # loopback on this very host, so a proxy in the path turns a working
        # viewer into a 502 that looks like the viewer being down.
        client = httpx.AsyncClient(timeout=None, trust_env=False)
        try:
            body = await request.body()
            upstream = client.build_request(
                request.method, url, headers=headers, content=body)
            response = await client.send(upstream, stream=True)
        except Exception as error:  # noqa: BLE001
            await client.aclose()
            # A viewer that is not running is the common case here, and a
            # bare 502 in the frame says nothing about which half is down.
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
        out.update(extra or {})
        return StreamingResponse(stream(), status_code=response.status_code,
                                 headers=out)

    # The viewer application is a 40 MB wasm bundle that rerun serves with no
    # caching headers at all, so every visit re-downloaded the whole thing --
    # seconds on a forwarded connection, every single time. The bundle only
    # changes when the installed rerun does, which cannot happen without
    # restarting this process, so an identity minted per process is enough to
    # let the browser keep its copy: the first visit pays for the download and
    # every later one revalidates in a round trip.
    _ASSET_ETAG = f'W/"rerun-{os.getpid()}"'
    _CACHEABLE = (".wasm", ".js", ".css", ".svg", ".ico", ".woff2")

    def _asset_cache_headers(path: str) -> dict:
        if not path.endswith(_CACHEABLE):
            return {}
        # `no-cache` is revalidate-every-time, not do-not-store: the browser
        # keeps the bundle and asks whether it is still current, which is the
        # behaviour that makes a second visit instant without ever serving a
        # stale viewer after an upgrade.
        return {"etag": _ASSET_ETAG, "cache-control": "no-cache"}

    async def rerun_app(request):
        """The viewer application, under a path that names its feed.

        The feed cannot travel in the query: rerun rewrites its own URL once
        it has parsed it, dropping anything it does not recognise, and the
        referrer on the data calls then names no feed at all -- which is how
        the 2D page came to show the 3D map. The path survives that rewrite.
        """
        if rerun_sink is None or not rerun_sink.ready:
            return PlainTextResponse("no viewer", status_code=404)
        if request.path_params.get("feed") not in ("2d", "3d"):
            return PlainTextResponse("not found", status_code=404)
        path = request.path_params.get("path", "")
        cache = _asset_cache_headers(path)
        if cache and request.headers.get("if-none-match") == _ASSET_ETAG:
            return Response(status_code=304, headers=cache)
        return await _proxy(request, rerun_sink.web_port, path, extra=cache)

    async def rerun_asset(request):
        """The viewer's own assets, which it requests from the site root."""
        if rerun_sink is None or not rerun_sink.ready:
            return PlainTextResponse("no viewer", status_code=404)
        path = request.url.path.lstrip("/")
        cache = _asset_cache_headers(path)
        if cache and request.headers.get("if-none-match") == _ASSET_ETAG:
            return Response(status_code=304, headers=cache)
        return await _proxy(request, rerun_sink.web_port, path, extra=cache)

    async def rerun_data(request):
        """One page's log stream, at the only path rerun will accept.

        rerun parses the data-source URL itself and takes the endpoint path
        to be exactly `/proxy`. Anything longer is rejected before a single
        request is made, silently: the viewer drops the source and shows its
        start page, which looks exactly like a map with nothing in it.

        `/proxy` in that URL names the endpoint; it is not where the traffic
        goes. The viewer speaks gRPC-web, so the requests land on the service
        path at the origin root -- `/rerun.sdk_comms.<version>.MessageProxy
        Service/ReadMessages` -- and this handler answers both.

        Both feeds therefore live at the same paths, and which one a request
        wants is read from the page that asked. The two feeds are two frames
        with different URLs, so the referrer separates them; `feed` in the
        query is accepted as well for anyone opening the endpoint by hand.
        A request that says neither gets the 3D feed, which is the page the
        UI opens on.
        """
        if rerun_sink is None or not rerun_sink.ready:
            return PlainTextResponse("no viewer", status_code=404)
        feed = request.query_params.get("feed")
        if feed not in ("2d", "3d"):
            referer = request.headers.get("referer") or ""
            feed = "2d" if "/rerun/2d/" in referer else "3d"
        return await _proxy(request, rerun_sink.data_port(feed),
                            request.url.path)

    async def rerun_grpc(request):
        """The viewer's gRPC-web calls, which arrive at the origin root."""
        service = request.path_params.get("service", "")
        if "MessageProxyService" not in service:
            return PlainTextResponse("not found", status_code=404)
        return await rerun_data(request)

    def _proxied_viewer(page: str, authority: str) -> str:
        """The viewer page for one feed, served under scene's own origin.

        rerun takes its data source from an absolute URL in the query string,
        so this has to name the host and port the reader actually reached —
        the Host header, not what scene bound. Behind a forwarded port those
        differ, and the address scene bound is unreachable from the browser.
        """
        import urllib.parse as _url

        source = f"rerun+http://{authority}/proxy"
        # The feed is in the path, not the query: see `rerun_app`.
        return (f"/rerun/{page}/?url=" + _url.quote(source, safe="")
                + "&theme=dark")

    async def viewer_url(request) -> JSONResponse:
        """Where the embedded viewers are served, and why they are not.

        The page needs to tell a reader whether the map is missing because
        nothing has been detected or because the viewer was never started; a
        blank frame cannot say which.
        """
        if rerun_sink is None:
            return JSONResponse({
                "url": "", "url_2d": "",
                "detail": "this deployment uses the built-in viewer "
                          "(scene web_viewer: builtin)",
            })
        if not rerun_sink.ready:
            return JSONResponse({
                "url": "", "url_2d": "", "detail": rerun_sink.detail,
            })
        # The viewer is reached from wherever this page was reached from. The
        # request's Host is the only thing that knows that; the address Scene
        # bound to does not.
        host = (request.headers.get("host") or "").split(":")[0] or "127.0.0.1"
        # Same-origin links: the frame, the viewer application and the log
        # stream all travel over the port the reader already reached scene on.
        # `viewer_url()` on the sink stays for an operator opening rerun
        # directly on the robot, where the ports are local anyway.
        authority = request.headers.get("host") or f"{host}:50107"
        return JSONResponse({"url": _proxied_viewer("3d", authority),
                             "url_2d": _proxied_viewer("2d", authority),
                             "detail": ""})

    async def objects3d(_request) -> JSONResponse:
        if detector is None or not hasattr(detector, "export_3d_snapshot"):
            return JSONResponse({"objects": [], "stamp_unix": 0.0})
        return JSONResponse(detector.export_3d_snapshot())

    async def cam(request) -> HTMLResponse:
        if _bare(request):
            return HTMLResponse(_INDEX_CAM_HTML)
        return HTMLResponse(_framed("/cam", "scene — camera"))

    async def user_page(request) -> HTMLResponse:
        if _bare(request):
            return HTMLResponse(_user_html("regions"))
        return HTMLResponse(_framed("/regions", "scene — regions"))

    def _mutations_or_none():
        """The coordinator, or None where this deployment has none.

        Reading the map never needed it; correcting does. Saying so beats a
        traceback that looks like the object was the problem."""
        return object_mutations

    async def _object_body(request) -> dict:
        try:
            body = await request.json()
        except Exception:  # noqa: BLE001
            return {}
        return body if isinstance(body, dict) else {}

    async def object_label(request) -> JSONResponse:
        """Rename one object, or clear a previous rename."""
        _mut = _mutations_or_none()
        if _mut is None:
            return JSONResponse(
                {"ok": False,
                 "detail": "this deployment has no object mutation coordinator"},
                status_code=503)
        body = await _object_body(request)
        label = str(body.get("label") or "").strip()
        clear = bool(body.get("clear_override"))
        if not label and not clear:
            return JSONResponse(
                {"ok": False, "detail": "label must not be empty"},
                status_code=400)
        try:
            obj, persisted, map_id, generation = (
                await _mut.apply_label_correction(
                    object_id=request.path_params["object_id"],
                    label=label,
                    clear_override=clear,
                    expected_map_id=str(body.get("expected_map_id") or ""),
                    expected_generation=body.get("expected_generation"),
                    note=str(body.get("note") or "renamed from the web UI"),
                ))
        except Exception as error:  # noqa: BLE001
            # A stale epoch and a missing object both arrive as the
            # coordinator's own message; passing it through beats inventing a
            # category for it here.
            return JSONResponse({"ok": False, "detail": str(error)},
                                status_code=409)
        return JSONResponse({
            # SceneObject carries the label as `cls`; an operator override
            # is recorded in `attributes` and is what should be echoed back
            # when there is one, so the panel shows what it will now read.
            "ok": True,
            "label": str(obj.attributes.get("label_override") or obj.cls),
            "map_id": map_id, "generation": generation, "persisted": persisted,
        })

    async def object_delete(request) -> JSONResponse:
        """Remove one object the perception layer should not have produced."""
        _mut = _mutations_or_none()
        if _mut is None:
            return JSONResponse(
                {"ok": False,
                 "detail": "this deployment has no object mutation coordinator"},
                status_code=503)
        body = await _object_body(request)
        try:
            deleted_id, persisted, map_id, generation = (
                await _mut.remove_object(
                    object_id=request.path_params["object_id"],
                    expected_map_id=str(body.get("expected_map_id") or ""),
                    expected_generation=body.get("expected_generation"),
                    note=str(body.get("note") or "deleted from the web UI"),
                ))
        except Exception as error:  # noqa: BLE001
            return JSONResponse({"ok": False, "detail": str(error)},
                                status_code=409)
        return JSONResponse({
            "ok": True, "deleted_id": deleted_id, "map_id": map_id,
            "generation": generation, "persisted": persisted,
        })

    async def map_preview(request):
        """The stored occupancy thumbnail for one saved map.

        Mapping writes it beside the map's spatial artifact on every save.
        The directory is named by SCENE_MAP_PREVIEW_DIR because it belongs to
        mapping's filesystem, not scene's, and a service should be told where
        another service's files are rather than deduce it.
        """
        root = (os.environ.get("SCENE_MAP_PREVIEW_DIR") or "").strip()
        if not root or not Path(root).is_dir():
            return PlainTextResponse(
                "SCENE_MAP_PREVIEW_DIR is not set for this deployment, so "
                "saved-map previews are not available here",
                status_code=404)
        base = Path(root).resolve()
        # Sanitised, then checked to land inside `base`: an id from the URL
        # joined to a path is where directory traversal lives.
        candidate = (base / _sanitize_map_id(request.path_params["map_id"])
                     / "occupancy.png")
        try:
            resolved = candidate.resolve()
            resolved.relative_to(base)
        except (OSError, ValueError):
            return PlainTextResponse("not found", status_code=404)
        if not resolved.is_file():
            return PlainTextResponse("this map has no preview", status_code=404)
        try:
            blob = resolved.read_bytes()
        except OSError as error:
            return PlainTextResponse(f"unreadable: {error}", status_code=404)
        return Response(blob, media_type="image/png", headers={
            # A saved map's preview changes only when the map is saved again,
            # and the grid asks for one per card.
            "Cache-Control": "public, max-age=60",
        })

    async def map_contents(request) -> JSONResponse:
        """One saved map's regions, objects and grid geometry.

        Answers "what is in this map" without loading it, which is what
        choosing between two of them requires.
        """
        map_id = _sanitize_map_id(request.path_params["map_id"])

        def collect() -> dict:
            root = (os.environ.get("SCENE_MAP_PREVIEW_DIR") or "").strip()
            grid = _read_grid_meta(Path(root) / map_id) if root else None

            regions = _read_saved_regions(
                str(anno_store.path.parent) if anno_store is not None else None,
                map_id)

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
                            "short_id": _shorten_id(getattr(obj, "object_id", "")),
                            "cls": getattr(obj, "cls", "object"),
                            "x": float(getattr(pose, "x", 0.0) or 0.0),
                            "y": float(getattr(pose, "y", 0.0) or 0.0),
                            "confidence": float(getattr(obj, "confidence", 0.0) or 0.0),
                        })
            # The provider publishes no paths, but scene opens these
            # files itself on every preview request, so it can say where
            # they are. Absent files are omitted rather than reported as
            # an empty string: the caller renders what it is given.
            paths: dict = {}
            if root:
                map_dir = Path(root) / map_id
                for field, name in (("artifact", "rtabmap.db"),
                                    ("preview", "occupancy.png")):
                    candidate = map_dir / name
                    try:
                        size = candidate.stat().st_size
                    except OSError:
                        continue
                    paths[field] = {"path": str(candidate), "bytes": size}
            return {
                "ok": True, "map_id": map_id, "grid": grid,
                "regions": regions, "objects": objects, "paths": paths,
            }

        return JSONResponse(await asyncio.to_thread(collect))

    async def logs_api(request) -> JSONResponse:
        """Whatever scribe appended since the caller's cursor.

        `cursor` is a JSON object of {tag: byte offset}, handed back from the
        previous response. Absent, each file is tailed instead: a log page is
        opened to see what just happened, and shipping a day of boot history
        first would only delay that.
        """
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
                    # Still advance the cursor for a tag being skipped, or
                    # re-enabling it would replay the whole gap at once.
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
            # Interleaved by time so the correlation this page exists for --
            # what mapping said while perception went quiet -- reads in order.
            entries.sort(key=lambda e: e.get("ts") or "")
            return {"ok": True, "entries": entries, "cursor": next_cursor,
                    "tags": sorted(tags), "dir": str(directory)}

        return JSONResponse(await asyncio.to_thread(collect))

    async def logs_page(request) -> HTMLResponse:
        """Scene's log view: scribe's files, tailed live."""
        return HTMLResponse(_shell_page("/logs", _LOGS_BODY, "scene — logs"))

    async def maps_page_old(request) -> HTMLResponse:
        """Map management: name and save the live session, load a saved map,
        delete one, or re-estimate the pose on the one that is loaded.

        Its own page because binding a map is the operation every other page
        depends on, and because it was previously four controls crowded above
        an unrelated drawing button, which said they were the same kind of
        thing.
        """
        if _bare(request):
            return HTMLResponse(_user_html("maps"))
        return HTMLResponse(_framed("/maps", "scene — maps"))

    async def maps_page(request) -> HTMLResponse:
        """The map library: a grid of what exists, and how to add to it.

        Rendered straight into the shell rather than an iframe -- it has no
        canvas and no viewer, so a frame would only cost it the stylesheet
        and the language runtime.
        """
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

    routes = [
        Route("/", index, methods=["GET"]),
        Route("/2d", index2d, methods=["GET"]),
        Route("/3d", index3d, methods=["GET"]),
        Route("/cam", cam, methods=["GET"]),
        Route("/maps", maps_page, methods=["GET"]),
        Route("/logs", logs_page, methods=["GET"]),
        Route("/api/logs", logs_api, methods=["GET"]),
        Route("/api/maps/{map_id}/preview", map_preview, methods=["GET"]),
        Route("/api/maps/{map_id}/contents", map_contents, methods=["GET"]),
        Route("/api/objects/{object_id}/label", object_label,
              methods=["POST"]),
        Route("/api/objects/{object_id}", object_delete,
              methods=["DELETE"]),
        Route("/regions", user_page, methods=["GET"]),
        Route("/api/state", state, methods=["GET"]),
        Route("/api/objects3d", objects3d, methods=["GET"]),
        Route("/api/viewer", viewer_url, methods=["GET"]),
        # Everything the embedded viewer needs, under scene's own origin.
        Route("/rerun/{feed}", rerun_app, methods=["GET"]),
        Route("/rerun/{feed}/{path:path}", rerun_app, methods=["GET"]),
        Route("/proxy", rerun_data, methods=["GET", "POST", "OPTIONS"]),
        # The gRPC-web service path, kept version-tolerant: the package name
        # carries rerun's own version and changes with it, so the route
        # matches any two-segment service call and the handler decides.
        Route("/re_viewer.js", rerun_asset, methods=["GET"]),
        Route("/re_viewer_bg.wasm", rerun_asset, methods=["GET"]),
        Route("/favicon.svg", rerun_asset, methods=["GET"]),
        Route("/sw.js", rerun_asset, methods=["GET"]),
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
        # Registered last on purpose: this is a two-segment catch-all, and
        # Starlette matches in order, so anywhere above here it swallows
        # every two-segment POST the service has -- POST /api/regions among
        # them, which returned the proxy's 404 instead of creating a region.
        Route("/{service}/{method}", rerun_grpc, methods=["POST", "OPTIONS"]),
    ]
    return Starlette(routes=routes)


# ── Combined split layout (iframes for both panels) ─────────────────────────
# Two side-by-side iframes (2D left, 3D right) with a per-panel "⛶ expand"
# button. Click expand → that panel goes `position: fixed; inset: 0` and
# covers the page (in-page fullscreen, not the browser's F11). Click again
# to restore. URL hash (`#2d` / `#3d`) is updated so refresh preserves state.
_COMBINED_HTML = _asset("combined.html")


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
_INDEX_CAM_HTML = _asset("camera.html")


_INDEX_3D_HTML = _asset("viewer_3d.html")


# ── User annotation page (/user) ─────────────────────────────────────────────
# The end-user map page: SLAM occupancy underlay + LIGHTWEIGHT object overlay
# + user-drawn region polygons, with a draw-a-region flow talking to the
# /api/regions CRUD. Deliberately self-contained (own inline JS, no
# imports from the debug pages' scripts): the two pages evolve independently
# and a debug-UI tweak must never break the user page. Kept dependency-free
# like every other page here (no framework, no build step).
def _user_html(page: str) -> str:
    """The shared template in one of its two modes.

    `page` is "maps" or "regions"; it lands on <body data-page> and the CSS
    decides which controls exist. The heading follows it, because a page whose
    title does not match its controls is the state this split was undoing.
    """
    title = "Maps" if page == "maps" else "Regions"
    key = "page.maps" if page == "maps" else "page.regions"
    return (_USER_HTML
            # A framed page loads none of the shell's CSS, so the shared
            # controls travel with it rather than being redefined in it.
            .replace("__CONTROLS__", _CONTROLS_CSS)
            .replace("__LABELS__", _LABELS_JS)
            .replace("__PAGE__", page)
            .replace("__I18N_RUNTIME__", _i18n_js())
            .replace('<h1 id="page-title" data-i18n="page.maps">Maps</h1>',
                     f'<h1 id="page-title" data-i18n="{key}">{title}</h1>'))


_USER_HTML = _asset("map_page.html")
