# SPDX-License-Identifier: MulanPSL-2.0
"""Publish the semantic map to a rerun viewer.

Two recordings, one per page, each served by a rerun process of its own:

    /map/floor, /map/objects/{rgb_pcd,sem_pcd,bbox}/<id>, /map/relations,
    /map/robot/{footprint,heading}                               (3D page)
    /map2d/grid, /map2d/objects/{cloud,marker}/<id>, /map2d/relations,
    /map2d/robot/{footprint,heading}                             (2D page)
"""
from __future__ import annotations

import atexit
import colorsys
import hashlib
import importlib.util
import logging
import math
import os
import signal
import subprocess
import threading
import time
from typing import Any, Callable, Iterable, Optional, Sequence

import numpy as np

log = logging.getLogger(__name__)

# Where the image installs the viewer's browser bundle (docker/Dockerfile).
RERUN_VIEWER_DIR = os.environ.get("SCENE_RERUN_VIEWER_DIR", "/opt/rerun-web-viewer")

# rerun's chrome is dark and cannot be changed, so the map is drawn dark too:
# unknown ground recedes, seen floor lifts off it, walls are brightest.
_VIEW_BACKGROUND = [16, 18, 24]
_COLOUR_OCCUPIED = (240, 243, 250)
_COLOUR_FREE = (104, 113, 133)
_COLOUR_UNKNOWN = (24, 27, 34)
_COLOUR_RELATION = (138, 166, 206, 170)  # context, kept faint
_RELATION_RADIUS_M = 0.004
_COLOUR_ROBOT = (255, 166, 54)
# Object colours vary in hue at fixed saturation, in two lightness bands so
# that two objects with close hues still differ.
_OBJECT_SATURATION = 0.52
_OBJECT_LIGHTNESS = (0.62, 0.76)

# How long a port may take to come free, and a server to start listening.
_PORT_WAIT_S = 8.0

_APP_3D = "robonix-scene"
_APP_2D = "robonix-scene-2d"


def _digest(*parts: Any) -> str:
    """Content hash, so an entity that did not change is not sent again."""
    hasher = hashlib.blake2s(digest_size=12)
    for part in parts:
        if part is None:
            hasher.update(b"\x00")
            continue
        buffer = None
        if isinstance(part, (list, tuple)) and part:
            try:
                buffer = np.asarray(part, dtype=np.float32).tobytes()
            except (TypeError, ValueError):  # labels and other non-numbers
                buffer = None
        hasher.update(buffer if buffer is not None
                      else repr(part).encode("utf-8", "replace"))
        hasher.update(b"\x1e")
    return hasher.hexdigest()


def map_label(obj: Any) -> str:
    """The object's class: a caption over every object hides the map; the
    panel shows the caption of the one selected."""
    return str(getattr(obj, "label", "") or getattr(obj, "display_name", ""))


def instance_colour(object_id: str) -> tuple[int, int, int]:
    """A colour derived from the id, stable across processes (not `hash()`)."""
    digest = hashlib.blake2s(object_id.encode("utf-8"), digest_size=3).digest()
    hue = ((digest[0] << 8) | digest[1]) / 65536.0
    lightness = _OBJECT_LIGHTNESS[digest[2] & 1]
    red, green, blue = colorsys.hls_to_rgb(hue, lightness, _OBJECT_SATURATION)
    return (round(red * 255), round(green * 255), round(blue * 255))


def _layout(view):
    """One view, side panels collapsed, no time panel (the map is live)."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        view,
        rrb.BlueprintPanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="hidden"),
    )


def _blueprint_3d(look_target=None):
    """The 3D page; `look_target` aims the camera, which is how the panel
    points the viewer at an object (rerun has no selection setter)."""
    import rerun.blueprint as rrb

    # First-person: dragging turns the view in place and WASD moves, crossing
    # a room in a couple of seconds; the orbit default spins around a far-away
    # centre.
    eye = rrb.EyeControls3D(
        kind=rrb.Eye3DKind.FirstPerson, eye_up=[0.0, 0.0, 1.0], speed=5.0,
        look_target=None if look_target is None
        else [float(v) for v in look_target])
    return _layout(rrb.Spatial3DView(
        origin="/map",
        name="semantic map",
        background=_VIEW_BACKGROUND,
        eye_controls=eye,
        line_grid=False,  # the occupancy grid is the floor
        # Boxes are perception's own extent guess; hidden unless checked.
        overrides={"/map/objects/bbox": rrb.EntityBehavior(visible=False)},
    ))


def _blueprint_2d():
    import rerun.blueprint as rrb

    return _layout(rrb.Spatial2DView(
        origin="/map2d", name="2D map", background=_VIEW_BACKGROUND))


def _grid_texture(occupancy: dict):
    """The occupancy PNG as an RGB image (map_server convention), or None."""
    encoded = (occupancy or {}).get("png_b64")
    if not encoded:
        return None
    import base64
    import io

    from PIL import Image

    try:
        image = Image.open(io.BytesIO(base64.b64decode(encoded))).convert("L")
    except Exception as error:  # noqa: BLE001
        log.warning("[scene-rerun] could not decode the occupancy grid: %s", error)
        return None
    grid = np.asarray(image)
    texture = np.empty((*grid.shape, 3), dtype=np.uint8)
    occupied = grid < 100
    free = grid > 200
    texture[occupied] = _COLOUR_OCCUPIED
    texture[free] = _COLOUR_FREE
    texture[~(occupied | free)] = _COLOUR_UNKNOWN
    return texture


def _to_pixels(occupancy: dict) -> Optional[Callable[[float, float], list]]:
    """Map-frame to grid-pixel converter; the PNG rows run opposite to y."""
    resolution = float(occupancy.get("resolution") or 0.0)
    if resolution <= 0.0:
        log.warning("[scene-rerun] the occupancy grid carries no resolution (%r)",
                    occupancy.get("resolution"))
        return None
    origin_x = float(occupancy.get("origin_x") or 0.0)
    origin_y = float(occupancy.get("origin_y") or 0.0)
    height = int(occupancy.get("height") or 0)

    def convert(x: float, y: float) -> list[float]:
        return [(x - origin_x) / resolution,
                height - (y - origin_y) / resolution]

    return convert


def relation_edges(sg_store) -> list[dict]:
    """Geometric and scene-graph edges, each drawn once."""
    if sg_store is None:
        return []
    edges: list[dict] = []
    seen: set[tuple[str, str, str]] = set()
    sources = list(sg_store.get_geometric_edges() or [])
    snapshot = sg_store.get_snapshot()
    if snapshot is not None:
        sources.extend(snapshot.edges)
    for edge in sources:
        key = (edge.source_id, edge.relation, edge.target_id)
        if edge.shown_on_map() and key not in seen:
            seen.add(key)
            edges.append({"subject": edge.source_id, "target": edge.target_id,
                          "predicate": edge.relation})
    return edges


def _edges(relations: Optional[Iterable[dict]], positions: dict):
    """(subject position, target position, predicate) for edges whose ends exist."""
    for relation in relations or ():
        subject = positions.get(relation["subject"])
        target = positions.get(relation["target"])
        if subject is not None and target is not None:
            yield subject, target, relation["predicate"]


def _footprint(pose, footprint) -> list[tuple[float, float]]:
    """The footprint polygon, closed, in the map frame."""
    x, y, yaw = pose
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return [(x + px * cos_yaw - py * sin_yaw, y + px * sin_yaw + py * cos_yaw)
            for px, py in list(footprint) + [footprint[0]]]


def _port_is_free(port: int) -> bool:
    """Nobody listens on `port` and a server could bind it.

    A rerun server on a taken port never serves and never says so. The bind
    uses SO_REUSEADDR, as the server does, so TIME_WAIT does not count.
    """
    import socket

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.settimeout(0.25)
        if probe.connect_ex(("127.0.0.1", port)) == 0:
            return False
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("0.0.0.0", port))
        except OSError:
            return False
    return True


def _wait(done: Callable[[], bool], failure: str,
          check: Callable[[], None] = lambda: None) -> None:
    deadline = time.monotonic() + _PORT_WAIT_S
    while not done():
        check()
        if time.monotonic() >= deadline:
            raise OSError(failure)
        time.sleep(0.1)


class _Feed:
    """One recording, served by a rerun process of its own.

    Out of process because an in-process `serve_grpc` holds the GIL while it
    starts, which on a busy machine froze every handler in Scene.
    """

    def __init__(self, grpc_port: int, app_id: str) -> None:
        self.grpc_port = grpc_port
        self.app_id = app_id
        self.recording: Any = None
        self.process: Optional[subprocess.Popen] = None

    def serve(self, rr: Any, blueprint: Any, memory_limit: str) -> None:
        port = self.grpc_port
        # A previous run may still be letting go of the port.
        _wait(lambda: _port_is_free(port), f"port {port} is in use")
        self.process = subprocess.Popen(
            ["rerun", "--serve-grpc", "--port", str(port),
             "--bind", "127.0.0.1",  # reached through Scene's web port
             "--server-memory-limit", memory_limit,
             "--newest-first"],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True)

        def exited() -> None:
            if self.process.poll() is not None:
                raise OSError(f"rerun server on {port} exited with "
                              f"{self.process.returncode}")

        _wait(lambda: not _port_is_free(port),
              f"rerun server on {port} did not start listening", exited)
        self.recording = rr.RecordingStream(self.app_id)
        self.recording.connect_grpc(f"rerun+http://127.0.0.1:{port}/proxy",
                                    default_blueprint=blueprint)

    def exited(self) -> bool:
        return self.process is not None and self.process.poll() is not None

    def stop(self) -> None:
        """Stop the server; `rerun` is a launcher, so signal its group."""
        if self.process is None:
            return
        if self.recording is not None:
            try:  # before the server goes, or its flush reports an error
                self.recording.disconnect()
            except Exception:  # noqa: BLE001
                pass
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(self.process.pid, sig)
                self.process.wait(timeout=5)
                return
            except ProcessLookupError:
                return
            except subprocess.TimeoutExpired:
                continue


class RerunSink:
    """The two recordings and their servers, started on first use."""

    def __init__(self, *, grpc_port: int = 9876, grpc_port_2d: int = 9877,
                 memory_limit: str = "512MiB") -> None:
        self._lock = threading.Lock()
        self._ready = False
        self._rr: Any = None
        self._memory_limit = memory_limit
        # A blueprint belongs to a recording, so each page has its own.
        self._map3d = _Feed(grpc_port, _APP_3D)
        self._map2d = _Feed(grpc_port_2d, _APP_2D)
        atexit.register(self._map3d.stop)
        atexit.register(self._map2d.stop)
        self._detail = "the viewer has not been started"
        self._available: bool | None = None
        # Ids drawn last tick, so objects that left get cleared.
        self._drawn: set[str] = set()
        self._drawn2d: set[str] = set()
        self._grid_payload: object = None
        self._grid_cache = None
        self._sent: dict[str, str] = {}  # entity path -> digest last sent
        # Perception exports clouds intermittently; a missing one is not gone.
        self._clouds: dict[str, list] = {}
        self._cloud_colours: dict[str, list] = {}
        # `latest` keeps one value per entity, so the browser's memory stays
        # bounded; `changes` keeps a timeline for debugging.
        self._history = (os.environ.get("SCENE_RERUN_HISTORY", "latest")
                         .strip().lower())

    # ── lifecycle ────────────────────────────────────────────────────────
    @property
    def ready(self) -> bool:
        return self._ready

    @property
    def available(self) -> bool:
        """Whether this deployment has a viewer (rerun and its bundle), not
        whether it is running: the first page that opens it starts it."""
        if self._available is None:
            bundle = os.path.join(RERUN_VIEWER_DIR, "index.js")
            self._available = (importlib.util.find_spec("rerun") is not None
                               and os.path.isfile(bundle))
            if not self._available:
                self._detail = (
                    f"no rerun viewer here (rerun-sdk and {bundle} are both "
                    "needed; the scene image installs them)")
        return self._available

    @property
    def detail(self) -> str:
        """Why there is no viewer, for the page to show."""
        return self._detail

    def ensure_started(self) -> bool:
        if self._ready and (self._map3d.exited() or self._map2d.exited()):
            log.warning("[scene-rerun] a viewer server exited; restarting both")
            with self._lock:
                self._map3d.stop()
                self._map2d.stop()
                self._ready = False
                # The new servers hold nothing, so everything is sent again.
                self._sent.clear()
                self._drawn.clear()
                self._drawn2d.clear()
        return self._ready or self.start()

    def start(self) -> bool:
        """Start both servers; False, with `detail` saying why, on failure."""
        with self._lock:
            if self._ready:
                return True
            try:
                import rerun as rr
            except ImportError:
                return self._fail("rerun-sdk is not installed in this "
                                  "environment; the scene docker image installs it")
            self._rr = rr
            try:
                self._map3d.serve(rr, _blueprint_3d(), self._memory_limit)
                self._map2d.serve(rr, _blueprint_2d(), self._memory_limit)
            except Exception as error:  # noqa: BLE001
                self._map3d.stop()
                self._map2d.stop()
                return self._fail(f"the viewer could not start: {error}")
            self._ready = True
            self._detail = ""
            log.info("[scene-rerun] serving the 3D and 2D maps on gRPC ports "
                     "%d and %d", self._map3d.grpc_port, self._map2d.grpc_port)
            return True

    def _fail(self, detail: str) -> bool:
        self._detail = detail
        log.warning("[scene-rerun] %s; the map pages will say so", detail)
        return False

    def look_at(self, point) -> bool:
        """Aim the 3D camera at a map point by resending the blueprint."""
        if not self._ready or self._map3d.recording is None:
            return False
        try:
            self._rr.send_blueprint(
                _blueprint_3d(point), make_active=True, make_default=False,
                recording=self._map3d.recording)
            return True
        except Exception as error:  # noqa: BLE001
            log.debug("[scene-rerun] look_at failed: %r", error)
            return False

    def data_port(self, page: str) -> int:
        """The gRPC port carrying one page's stream."""
        return (self._map2d if page == "2d" else self._map3d).grpc_port

    def set_time(self, seconds: float) -> None:
        if self._ready and not self._static:
            for feed in (self._map3d, self._map2d):
                feed.recording.set_time("scene", duration=float(seconds))

    # ── sending ──────────────────────────────────────────────────────────
    @property
    def _static(self) -> bool:
        return self._history == "latest"

    def _log(self, feed: _Feed, path: str, entity: Any, *,
             static: bool = False) -> None:
        self._rr.log(path, entity, static=static or self._static,
                     recording=feed.recording)

    def _send(self, feed: _Feed, path: str, signature: str,
              entity: Callable[[], Any], *, static: bool = False) -> None:
        """Log `entity()` at `path` unless the path already carries `signature`."""
        if self._sent.get(path) == signature:
            return
        self._sent[path] = signature
        self._log(feed, path, entity(), static=static)

    def _clear(self, feed: _Feed, path: str) -> None:
        # Forget the digest, or the same content coming back stays cleared.
        self._sent.pop(path, None)
        self._log(feed, path, self._rr.Clear(recursive=True))

    def _texture(self, occupancy: dict):
        """Decoded grid, cached on the payload object (cached upstream).

        Holding the payload, not its id(), keeps a new grid from reusing it.
        """
        if occupancy is not self._grid_payload:
            self._grid_cache = _grid_texture(occupancy)
            self._grid_payload = occupancy
        return self._grid_cache

    def _with_remembered(self, objects, clouds, colours):
        """Clouds for `objects`, falling back to the last one each exported."""
        clouds, colours = dict(clouds or {}), dict(colours or {})
        for obj in objects:
            oid = obj.object_id
            if clouds.get(oid):
                self._clouds[oid] = clouds[oid]
                if colours.get(oid):
                    self._cloud_colours[oid] = colours[oid]
            elif self._clouds.get(oid):
                clouds[oid] = self._clouds[oid]
                if self._cloud_colours.get(oid):
                    colours[oid] = self._cloud_colours[oid]
        return clouds, colours

    # ── the 3D page ──────────────────────────────────────────────────────
    def log_occupancy(self, occupancy: dict) -> None:
        """The grid as one textured plane at floor level."""
        if not self._ready or not occupancy:
            return
        signature = _digest(occupancy.get("png_b64"), occupancy.get("resolution"),
                            occupancy.get("origin_x"), occupancy.get("origin_y"))
        if self._sent.get("/map/floor") == signature:
            return
        texture = self._texture(occupancy)
        if texture is None:
            return
        self._sent["/map/floor"] = signature
        height, width = texture.shape[:2]
        resolution = float(occupancy.get("resolution") or 0.05)
        x0 = float(occupancy.get("origin_x") or 0.0)
        y0 = float(occupancy.get("origin_y") or 0.0)
        x1, y1 = x0 + width * resolution, y0 + height * resolution
        # Static in both modes: only the current floor plan matters.
        self._log(self._map3d, "/map/floor", self._rr.Mesh3D(
            vertex_positions=[[x0, y0, 0.0], [x1, y0, 0.0],
                              [x1, y1, 0.0], [x0, y1, 0.0]],
            triangle_indices=[[0, 1, 2], [0, 2, 3]],
            # Image row 0 is the top of the map, so v runs against y.
            vertex_texcoords=[[0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [0.0, 0.0]],
            albedo_texture=texture,
        ), static=True)

    def log_objects(self, objects: Iterable[Any],
                    clouds: Optional[dict[str, Sequence]] = None,
                    colours: Optional[dict[str, Sequence]] = None) -> None:
        """Per object: camera-coloured points, instance-coloured points with
        the label, and a faint box (the only label when there are no points)."""
        if not self._ready:
            return
        rr, feed = self._rr, self._map3d
        objects = list(objects)
        clouds, colours = self._with_remembered(objects, clouds, colours)
        drawn: set[str] = set()
        for obj in objects:
            oid, label = obj.object_id, map_label(obj)
            drawn.add(oid)
            instance = instance_colour(oid)
            points = clouds.get(oid)
            if points:
                positions = [p[:3] for p in points]
                rgb = colours.get(oid)
                if rgb and len(rgb) == len(positions):
                    self._send(feed, f"/map/objects/rgb_pcd/{oid}",
                               _digest(positions, rgb),
                               lambda: rr.Points3D(positions, colors=list(rgb)))
                self._send(feed, f"/map/objects/sem_pcd/{oid}",
                           _digest(positions, label),
                           lambda: rr.Points3D(
                               positions, colors=[instance] * len(positions),
                               labels=[label], show_labels=True))
            centre = [obj.pose.x, obj.pose.y, obj.pose.z]
            half = [obj.bbox.size_x / 2.0, obj.bbox.size_y / 2.0,
                    obj.bbox.size_z / 2.0]
            # The box's alpha also dims its label, so it is solid when it is
            # the only thing carrying one.
            alpha = 60 if points else 190
            self._send(feed, f"/map/objects/bbox/{oid}",
                       _digest(centre, half, label, bool(points)),
                       lambda: rr.Boxes3D(
                           centers=[centre], half_sizes=[half],
                           colors=[(*instance, alpha)], labels=[label],
                           show_labels=not points))
        for oid in self._drawn - drawn:
            self._clouds.pop(oid, None)
            self._cloud_colours.pop(oid, None)
            for group in ("rgb_pcd", "sem_pcd", "bbox"):
                self._clear(feed, f"/map/objects/{group}/{oid}")
        self._drawn = drawn

    def log_relations(self, relations: Iterable[Any],
                      positions: dict[str, tuple[float, float, float]]) -> None:
        """One line per relation; logged when empty too, to clear the last."""
        if not self._ready:
            return
        edges = list(_edges(relations, positions))
        strips = [[list(subject), list(target)] for subject, target, _ in edges]
        labels = [predicate for _, _, predicate in edges]
        self._send(self._map3d, "/map/relations", _digest(strips, labels),
                   lambda: self._rr.LineStrips3D(
                       strips, labels=labels,
                       colors=[_COLOUR_RELATION] * len(strips),
                       radii=_RELATION_RADIUS_M, show_labels=False))

    def log_robot(self, pose: tuple[float, float, float],
                  footprint: Sequence[Sequence[float]]) -> None:
        """Soma's footprint polygon and an arrow along yaw."""
        if not self._ready:
            return
        rr, feed = self._rr, self._map3d
        x, y, yaw = pose
        if footprint:
            self._log(feed, "/map/robot/footprint", rr.LineStrips3D(
                [[[px, py, 0.02] for px, py in _footprint(pose, footprint)]],
                colors=[_COLOUR_ROBOT], radii=0.01))
        self._log(feed, "/map/robot/heading", rr.Arrows3D(
            origins=[[x, y, 0.05]],
            vectors=[[0.45 * math.cos(yaw), 0.45 * math.sin(yaw), 0.0]],
            colors=[_COLOUR_ROBOT]))

    # ── the 2D page ──────────────────────────────────────────────────────
    def log_map2d(self, occupancy: dict, objects: Iterable[Any],
                  clouds: Optional[dict[str, Sequence]] = None,
                  relations: Optional[Iterable[Any]] = None,
                  robot: Optional[tuple[float, float, float]] = None,
                  footprint: Optional[Sequence[Sequence[float]]] = None) -> None:
        """The floor plan with the same annotations, in grid pixels."""
        if not self._ready or not occupancy:
            return
        texture = self._texture(occupancy)
        to_pixels = _to_pixels(occupancy)
        if texture is None or to_pixels is None:
            return
        rr, feed = self._rr, self._map2d
        self._send(feed, "/map2d/grid", _digest(occupancy.get("png_b64"),
                                                occupancy.get("resolution")),
                   lambda: rr.Image(texture), static=True)

        objects = list(objects)
        clouds, _ = self._with_remembered(objects, clouds, None)
        # One entity per object, so a click names it.
        centres: dict[str, tuple[float, float]] = {}
        drawn: set[str] = set()
        for obj in objects:
            oid, label = obj.object_id, map_label(obj)
            instance = instance_colour(oid)
            centres[oid] = (obj.pose.x, obj.pose.y)
            drawn.add(oid)
            cloud = [to_pixels(p[0], p[1]) for p in clouds.get(oid) or ()]
            self._send(feed, f"/map2d/objects/cloud/{oid}", _digest(cloud, instance),
                       lambda: rr.Points2D(cloud, colors=[instance], radii=0.7))
            centre = to_pixels(obj.pose.x, obj.pose.y)
            self._send(feed, f"/map2d/objects/marker/{oid}",
                       _digest(centre, label, instance),
                       lambda: rr.Points2D([centre], colors=[instance],
                                           labels=[label], radii=2.5))
        for oid in self._drawn2d - drawn:
            for group in ("cloud", "marker"):
                self._clear(feed, f"/map2d/objects/{group}/{oid}")
        self._drawn2d = drawn

        # Unlabelled here: from above the edges are short and labels cover objects.
        strips = [[to_pixels(*subject), to_pixels(*target)]
                  for subject, target, _ in _edges(relations, centres)]
        self._send(feed, "/map2d/relations", _digest(strips),
                   lambda: rr.LineStrips2D(
                       strips, colors=[_COLOUR_RELATION] * len(strips), radii=0.4))

        if robot is None:
            return
        x, y, yaw = robot
        if footprint:
            self._log(feed, "/map2d/robot/footprint", rr.LineStrips2D(
                [[to_pixels(px, py) for px, py in _footprint(robot, footprint)]],
                colors=[_COLOUR_ROBOT], radii=0.6))
        head = to_pixels(x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw))
        base = to_pixels(x, y)
        self._log(feed, "/map2d/robot/heading", rr.Arrows2D(
            origins=[base], vectors=[[head[0] - base[0], head[1] - base[1]]],
            colors=[_COLOUR_ROBOT]))
