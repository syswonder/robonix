# SPDX-License-Identifier: MulanPSL-2.0
"""Publish the semantic map to a rerun viewer.

Scene used to draw its own map: a three.js page that pulled the library from a
CDN and needed WebGL. On a headless robot neither is available, so the 3D view
rendered nothing there, and keeping it working was viewer maintenance that has
nothing to do with a scene graph.

rerun ships the viewer, an HTTP server for it and the wire protocol between
them. Scene logs entities; the UI links to the viewer. What is logged is the
annotated map, not raw sensor data: one coloured point cloud and one label per
object, an edge per relation, and the robot's own footprint and heading.

Entity paths group the map so the viewer's tree is navigable:

    /map/objects/<kind>/<id>         points, faint box, label
    /map/ground                      the occupancy grid as a textured plane
    /map/relations                   one line strip per relation
    /map/robot/footprint             the polygon Soma supplies
    /map/robot/heading               an arrow along yaw
"""
from __future__ import annotations

import colorsys
import hashlib
import logging
import math
import os
import threading
import urllib.parse
from typing import Any, Iterable, Optional, Sequence

log = logging.getLogger(__name__)

# One application id per page. A blueprint belongs to an application, so two
# recordings sharing an id share a layout: the second page then opens on the
# first page's views and shows nothing, because its entities are not under
# their origin.
# The viewer's own chrome is dark and rerun 0.37 gives no way to change it
# (the web backend ignores SetTheme). A light view inside that dark chrome
# leaves the page half lit and half dark, and every label rerun draws takes
# its colour from the entity it belongs to, so on a light ground the muted
# object colours that read well as point clouds are exactly the colours that
# make the text unreadable. The view is dark for the same reason the chrome
# is, and the map is drawn the way a dark-mode floor plan is drawn: unknown
# ground recedes into the background, seen floor lifts off it, and the walls
# are the brightest thing in the grid because they are what bounds the room.
# The three cell meanings have to separate at a glance, and on the 3D page
# they are seen through a lit surface that darkens them: the swept floor is
# what says where the robot has actually been, and at the first dark values
# tried it was indistinguishable from ground nobody has ever observed. The
# steps are widened accordingly — unknown sits on the background, seen floor
# lifts well clear of it, walls are near white.
_VIEW_BACKGROUND = [16, 18, 24]
_COLOUR_OCCUPIED = (240, 243, 250)   # walls: they bound the room
_COLOUR_FREE = (104, 113, 133)       # floor the robot has seen
_COLOUR_UNKNOWN = (24, 27, 34)       # never observed
# Relations are context, not evidence. They are drawn thin, muted and
# translucent so that the point clouds stay the thing the eye lands on.
_COLOUR_RELATION = (138, 166, 206, 170)
_RELATION_RADIUS_M = 0.004
_COLOUR_ROBOT = (255, 166, 54)

# How long to wait for the viewer to bind its three ports before
# giving up on it and serving the built-in pages.
_START_TIMEOUT_S = 20.0

_APP_3D = "robonix-scene"
_APP_2D = "robonix-scene-2d"


# Object colours vary in hue only. Three random bytes span the whole RGB cube,
# which puts neon magenta next to muddy brown and makes the map read as noise
# rather than as a legend; holding saturation and lightness fixed keeps every
# object distinguishable while the set stays coherent on a light background.
_OBJECT_SATURATION = 0.52
# Two lightness bands rather than one. Hue alone collides: thirty objects on a
# 360-degree wheel put some pair within a couple of degrees more often than
# not, and two near-identical clouds read as one object seen twice. A second
# axis makes such a pair a light and a dark version of the same hue.
# Both bands sit above the background's lightness: these colours carry the
# object labels as well as its points, and a 0.42 band that reads fine as a
# cloud is unreadable as text on a dark ground.
_OBJECT_LIGHTNESS = (0.62, 0.76)


def _digest(*parts: Any) -> str:
    """A short content hash of whatever was passed.

    The viewer's cost is not the size of the map but the number of times the
    map is sent: every publish used to append a full copy of the grid, every
    point cloud and every box to the timeline whether or not anything had
    moved, and the browser keeps what it is sent. A hash per entity is what
    lets a tick decide it has nothing to say.

    Point lists arrive as nested Python sequences, which are slow to hash as
    text; numpy flattens them into one buffer when it is available and the
    text form stays as the fallback for the environments that lack it.
    """
    hasher = hashlib.blake2s(digest_size=12)
    for part in parts:
        if part is None:
            hasher.update(b"\x00")
            continue
        buffer = None
        if isinstance(part, (list, tuple)) and part:
            try:
                import numpy as np

                buffer = np.asarray(part, dtype=np.float32).tobytes()
            except Exception:  # noqa: BLE001
                buffer = None
        hasher.update(buffer if buffer is not None
                      else repr(part).encode("utf-8", "replace"))
        hasher.update(b"\x1e")
    return hasher.hexdigest()


def instance_colour(object_id: str) -> tuple[int, int, int]:
    """A stable colour per object id.

    Derived from the identifier so an object keeps its colour across restarts;
    a random palette recolours the whole map on every boot and makes two
    screenshots impossible to compare. `hash()` cannot be used for this:
    Python salts string hashing per process, so it produces exactly the
    per-boot palette this exists to avoid.

    Only the hue comes from the identifier. See the constants above.
    """
    digest = hashlib.blake2s(object_id.encode("utf-8"), digest_size=3).digest()
    hue = ((digest[0] << 8) | digest[1]) / 65536.0
    lightness = _OBJECT_LIGHTNESS[digest[2] & 1]
    red, green, blue = colorsys.hls_to_rgb(hue, lightness, _OBJECT_SATURATION)
    return (round(red * 255), round(green * 255), round(blue * 255))


def _blueprint_3d():
    """One 3D view over `/map`, on a flat background.

    The viewer is one page of Scene's UI, not the UI: it draws the annotated
    map and nothing else. Camera frames stay on Scene's own /cam page, which
    already has them, rather than being pushed through a second pipeline.

    rerun's default 3D background is a green-to-orange gradient standing in for
    sky and ground. On a semantic map it competes with the object colours that
    carry the meaning, so the view is given a flat dark background and the map
    is made its origin: the entity tree then opens on the map rather than on
    the recording root.
    """
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="/map",
            name="semantic map",
            background=_VIEW_BACKGROUND,
            # The occupancy grid is drawn as a real floor, so rerun's own
            # infinite grid adds a second, larger floor at the same height and
            # the map reads as a small patch floating on it.
            line_grid=False,
            # The boxes are the perception layer's own extent estimate. They
            # are often wrong, and a wireframe around every object buries the
            # points that are the actual evidence. Kept for debugging, switched
            # on from the entity tree when a detection needs checking.
            overrides={"/map/objects/bbox": rrb.EntityBehavior(visible=False)},
        ),
        collapse_panels=True,
    )


def _blueprint_2d():
    """One 2D view over `/map2d`: the floor plan with the same annotations."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial2DView(
            origin="/map2d",
            name="2D map",
            background=_VIEW_BACKGROUND,
        ),
        collapse_panels=True,
    )


def _grid_texture(occupancy: dict):
    """Decode the occupancy PNG into an RGB image, or None.

    The three meanings follow the map_server convention the grid is written
    with: near-black is occupied, near-white is free, the value between is
    ground the robot has never observed. Both pages colour it here so the
    floor plane and the floor plan can never drift apart.
    """
    encoded = (occupancy or {}).get("png_b64")
    if not encoded:
        return None
    try:
        import base64
        import io

        import numpy as np
        from PIL import Image
    except ImportError:
        log.warning(
            "[scene-rerun] numpy and Pillow are needed to draw the occupancy "
            "grid; the map will show objects with no floor under them")
        return None
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


def _to_pixels(occupancy: dict):
    """Return a map-frame to grid-pixel converter for this grid.

    The PNG is stored top-down while the grid's origin is its bottom-left
    corner, so the row runs opposite to y. Returns None when the grid carries
    no usable resolution.
    """
    resolution = float(occupancy.get("resolution") or 0.0)
    if resolution <= 0.0:
        log.warning(
            "[scene-rerun] the occupancy grid carries no resolution (%r); "
            "the 2D page cannot place anything on it",
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
    """The relation edges the viewer draws, as ``subject``/``target`` pairs.

    Both layers of the graph are included: the geometric edges the inferer
    keeps continuously, and the image-grounded edges the scene graph adds when
    it is enabled. Duplicates between the two are dropped so an edge is drawn
    once. Returns an empty list when the store is absent.
    """
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
        if key in seen:
            continue
        seen.add(key)
        edges.append({
            "subject": edge.source_id,
            "target": edge.target_id,
            "predicate": edge.relation,
        })
    return edges


def _relation_end(relation: Any, *names: str) -> Optional[str]:
    """The id at one end of a relation, under whichever name it carries.

    The web payload names the ends `subject`/`target`; the registry objects
    name them `subject_id`/`object_id`. Reading only one name silently drew no
    edges at all, which looks exactly like a scene with no relations in it.
    """
    for name in names:
        value = relation.get(name)
        if value:
            return str(value)
    return None


def _port_is_free(port: int) -> bool:
    """Whether a TCP port can still be bound on this host.

    `serve_grpc` does not return an error when its port is taken; it blocks,
    and a blocked call inside lifecycle activation reads as Scene hanging on
    startup with nothing in the log. A previous Scene that has not finished
    exiting is exactly when that happens, so the port is checked first and the
    viewer declines to start rather than stopping the service.
    """
    import socket

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("0.0.0.0", port))
        except OSError:
            return False
    return True


class _Feed:
    """One recording and the gRPC server that streams it."""

    def __init__(self, grpc_port: int, app_id: str) -> None:
        self.grpc_port = grpc_port
        self.app_id = app_id
        self.recording: Any = None
        self.grpc_url = ""

    def serve(self, rr: Any, blueprint: Any, memory_limit: str) -> None:
        """Open the recording and its gRPC server."""
        self.recording = rr.RecordingStream(self.app_id)
        self.grpc_url = self.recording.serve_grpc(
            grpc_port=self.grpc_port,
            server_memory_limit=memory_limit,
            default_blueprint=blueprint,
        )


class RerunSink:
    """Owns the rerun recording and the servers that expose it."""

    def __init__(self, *, grpc_port: int = 9876, web_port: int = 9090,
                 grpc_port_2d: int = 9877,
                 web_host: str = "127.0.0.1",
                 memory_limit: str = "512MiB") -> None:
        self._lock = threading.Lock()
        self._ready = False
        self._rr: Any = None
        self._web_host = web_host
        self._memory_limit = memory_limit
        # Two recordings, two servers, one per page. A blueprint belongs to a
        # recording, and the viewer takes its data source from the query
        # string with no way to name a view inside it, so a single recording
        # could only ever open on one layout. Splitting them is what lets the
        # semantic map and the floor plan each be their own page.
        self._web_port = web_port
        self._map3d = _Feed(grpc_port, _APP_3D)
        self._map2d = _Feed(grpc_port_2d, _APP_2D)
        self._detail = "the viewer has not been started"
        # What was drawn last tick. An entity logged once stays in the
        # recording forever: without this an object the registry evicted, or a
        # relation that stopped holding, keeps being drawn and a stale map is
        # indistinguishable from a live one.
        self._drawn: set[str] = set()
        self._grid_signature: object = None
        self._grid_cache = None
        # What each entity last carried. An entity whose content has not
        # changed is not logged again: see `_digest`.
        self._sent: dict[str, str] = {}
        # `changes` keeps the timeline, so dragging it replays the map filling
        # in; `latest` logs everything as static, which holds exactly one value
        # per entity in the browser's store and is what a weak or forwarded
        # client needs. The grid is static in both modes regardless: it is a
        # current-state artefact that SLAM rewrites continuously, and keeping
        # its history is what fills a browser with megabytes of superseded
        # floor plans.
        self._history = (os.environ.get("SCENE_RERUN_HISTORY", "changes")
                         .strip().lower())

    @property
    def ready(self) -> bool:
        return self._ready

    @property
    def detail(self) -> str:
        """Why there is no viewer, in words a page can show a reader."""
        return self._detail

    def start(self) -> bool:
        """Start the recording and both servers. False when rerun is absent."""
        with self._lock:
            if self._ready:
                return True
            taken = [port for port in (self._map3d.grpc_port,
                                       self._map2d.grpc_port,
                                       self._web_port)
                     if not _port_is_free(port)]
            if taken:
                # Checked before the import: a busy port is a failure whether
                # or not rerun is installed, and naming the port is what tells
                # an operator this is a leftover process rather than a missing
                # package.
                self._detail = (
                    "the viewer's ports are already in use: "
                    + ", ".join(str(port) for port in taken))
                log.warning(
                    "[scene-rerun] %s; scene will serve the built-in pages "
                    "instead", self._detail)
                return False
            try:
                import rerun as rr
            except ImportError:
                self._detail = (
                    "rerun-sdk is not installed in this environment; install "
                    "the scene `viewer` extra to enable the map viewer")
                log.warning("[scene-rerun] %s", self._detail)
                return False
            self._rr = rr
            # Every call below binds a port, and none of them fails when the
            # port is taken: `serve_grpc` blocks forever instead, and a bind
            # check beforehand does not catch it -- a socket left in TIME_WAIT
            # by the previous run accepts the check and still wedges the
            # server. The whole bring-up therefore runs in a thread this waits
            # on with a deadline: the map service must not be held hostage by
            # its own debugging aid, and a viewer that never came up has to
            # say so rather than leave the web UI unreachable forever.
            outcome: dict[str, Any] = {}

            def bring_up() -> None:
                try:
                    self._map3d.serve(rr, _blueprint_3d(), self._memory_limit)
                    self._map2d.serve(rr, _blueprint_2d(), self._memory_limit)
                    # One web server for both pages. `serve_web_viewer` hands
                    # out the viewer application, not the data: which recording
                    # a page shows is decided by the `url` in its query string.
                    # Calling it a second time only fights the first for the
                    # port, and the bind error takes the service down with it.
                    rr.serve_web_viewer(
                        web_port=self._web_port, open_browser=False,
                        connect_to=self._map3d.grpc_url)
                    outcome["ok"] = True
                except Exception as error:  # noqa: BLE001
                    outcome["error"] = error

            worker = threading.Thread(target=bring_up, name="scene-rerun-up",
                                      daemon=True)
            worker.start()
            worker.join(_START_TIMEOUT_S)
            if worker.is_alive():
                self._detail = (
                    f"the viewer did not finish starting within "
                    f"{_START_TIMEOUT_S:.0f}s; one of its ports "
                    f"({self._map3d.grpc_port}, {self._map2d.grpc_port}, "
                    f"{self._web_port}) is still held, most likely by the "
                    "previous run")
                log.warning(
                    "[scene-rerun] %s; scene will serve the built-in pages "
                    "instead", self._detail)
                return False
            if "error" in outcome:
                # Usually a port already in use. The viewer is a debugging
                # aid, so under `auto` a deployment that cannot serve it falls
                # back to the built-in pages; letting this escape would stop
                # the map service itself over a busy port.
                self._detail = f"the viewer could not start: {outcome['error']}"
                log.warning(
                    "[scene-rerun] %s; scene will serve the built-in pages "
                    "instead", self._detail)
                return False
            self._ready = True
            self._detail = ""
            log.info("[scene-rerun] 3D viewer on %s",
                     self.viewer_url(self._web_host))
            log.info("[scene-rerun] 2D viewer on %s",
                     self.viewer_url_2d(self._web_host))
            return True

    @property
    def web_port(self) -> int:
        """Where rerun serves the viewer application itself."""
        return self._web_port

    def data_port(self, page: str) -> int:
        """The gRPC-web port carrying one page's log stream."""
        feed = self._map2d if page == "2d" else self._map3d
        return feed.grpc_port

    def viewer_url(self, host: str) -> str:
        """The 3D viewer page, as seen from `host`."""
        return self._viewer_link(host, self._map3d.grpc_url)

    def viewer_url_2d(self, host: str) -> str:
        """The 2D viewer page, as seen from `host`."""
        return self._viewer_link(host, self._map2d.grpc_url)

    def _viewer_link(self, host: str, grpc_url: str) -> str:
        """The viewer page for one recording, reachable from `host`.

        `host` is the name the reader's browser used to reach Scene, not the
        address Scene bound. An operator opening the UI on a robot across the
        network would otherwise be handed a viewer on their own machine's
        loopback: a blank frame, no error, and an API that reports success.

        The data source travels in the query string. Opening the bare host
        serves the viewer with `?url=` empty, which renders rerun's start page
        and no map: the other failure that looks like this working.
        """
        source = grpc_url.replace("127.0.0.1", host).replace("0.0.0.0", host)
        # `theme=dark` pins the viewer's chrome rather than letting it follow
        # the reader's OS preference: the map inside the view is drawn for a
        # dark ground, and a light chrome around it leaves half the page lit
        # and every label sitting on the wrong background.
        return (f"http://{host}:{self._web_port}/"
                f"?url={urllib.parse.quote(source, safe='')}"
                f"&theme=dark")

    def _log3d(self, path: str, entity: Any, *, static: bool = False) -> None:
        self._rr.log(path, entity, static=static or self._static,
                     recording=self._map3d.recording)

    def _log2d(self, path: str, entity: Any, *, static: bool = False) -> None:
        self._rr.log(path, entity, static=static or self._static,
                     recording=self._map2d.recording)

    @property
    def _static(self) -> bool:
        """True when this feed keeps only the newest value of each entity."""
        return self._history == "latest"

    def _unchanged(self, key: str, signature: str) -> bool:
        """True when `key` already carries exactly this content.

        Records the signature as a side effect, so a caller that asks is a
        caller that is about to log. Clearing an entity has to forget its
        signature, or the next identical value would be suppressed and the
        entity would stay cleared on screen.
        """
        if self._sent.get(key) == signature:
            return True
        self._sent[key] = signature
        return False

    def _forget(self, key: str) -> None:
        self._sent.pop(key, None)

    def set_time(self, seconds: float) -> None:
        """Place everything logged next at this point on the timeline.

        Without a time index every publish overwrites the last and the viewer
        shows only the newest state. With one, dragging the timeline replays
        the map filling in as the robot drove, which is how a reader tells a
        detection that persisted from one that appeared for a single frame.
        """
        if self._ready and not self._static:
            for feed in (self._map3d, self._map2d):
                feed.recording.set_time("scene", duration=float(seconds))

    def _texture(self, occupancy: dict):
        """The coloured grid for this payload, decoded at most once.

        Both pages draw the same grid every tick, and decoding a room-scale
        PNG twice a second on the event loop that also answers the gRPC and
        web handlers is time taken from them. The payload is already cached by
        the hub's message count upstream, so its identity is enough to tell a
        new grid from the same one seen again.
        """
        signature = id(occupancy)
        if signature != self._grid_signature or self._grid_cache is None:
            self._grid_cache = _grid_texture(occupancy)
            self._grid_signature = signature
        return self._grid_cache

    def log_occupancy(self, occupancy: dict) -> None:
        """Log the occupancy grid as a map, not as a cloud of points.

        Objects logged alone float in an empty volume and a reader cannot tell
        a chair in the room from a chair inside a wall. The grid is what makes
        the semantic map judgeable, so it is drawn the way a map is drawn: one
        textured plane at floor level with a colour per cell meaning, rather
        than a scatter of points that reads as noise.
        """
        if not self._ready or not occupancy:
            return
        # The grid is the heaviest thing published and the thing that repeats
        # most: a room-scale texture, re-sent on every tick whether or not
        # SLAM touched it. Hashing the encoded grid is what turns a stationary
        # robot's viewer feed into nothing at all.
        if self._unchanged("/map/floor", _digest(occupancy.get("png_b64"),
                                                 occupancy.get("resolution"),
                                                 occupancy.get("origin_x"),
                                                 occupancy.get("origin_y"))):
            return
        texture = self._texture(occupancy)
        if texture is None:
            self._forget("/map/floor")
            return
        height, width = texture.shape[:2]
        resolution = float(occupancy.get("resolution") or 0.05)
        origin_x = float(occupancy.get("origin_x") or 0.0)
        origin_y = float(occupancy.get("origin_y") or 0.0)
        span_x = width * resolution
        span_y = height * resolution
        # Two triangles at floor level, textured with the grid. Row 0 of the
        # image is the top of the map, which is the highest y, so v runs the
        # other way from y.
        corners = [
            [origin_x, origin_y, 0.0],
            [origin_x + span_x, origin_y, 0.0],
            [origin_x + span_x, origin_y + span_y, 0.0],
            [origin_x, origin_y + span_y, 0.0],
        ]
        # Static in both history modes: the floor plan a reader wants is the
        # one SLAM holds now, and every superseded version of it is weight the
        # browser carries for the rest of the session.
        self._log3d("/map/floor", self._rr.Mesh3D(
            vertex_positions=corners,
            triangle_indices=[[0, 1, 2], [0, 2, 3]],
            vertex_texcoords=[[0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [0.0, 0.0]],
            albedo_texture=texture,
        ), static=True)

    def log_objects(self, objects: Iterable[Any],
                    clouds: Optional[dict[str, Sequence]] = None,
                    colours: Optional[dict[str, Sequence]] = None) -> None:
        """Log each object as two point layers and a labelled box.

        The layout follows DualMap, whose viewer this is modelled on. Each
        object appears three times under separate entity paths so the viewer's
        tree can switch between them:

          rgb_pcd/<id>   the points in the colours the camera saw
          sem_pcd/<id>   the same points in one colour per instance
          bbox/<id>      the box, carrying the class label

        Keeping the photographic and the semantic colouring apart is what makes
        the map readable: the first shows whether the geometry is right, the
        second whether the segmentation is. One layer alone answers half.
        """
        if not self._ready:
            return
        rr = self._rr
        clouds = clouds or {}
        colours = colours or {}
        drawn: set[str] = set()
        for obj in objects:
            drawn.add(obj.object_id)
            instance = instance_colour(obj.object_id)
            points = clouds.get(obj.object_id)
            if points:
                positions = [p[:3] for p in points]
                rgb = colours.get(obj.object_id)
                # A cloud is republished on every tick even when perception
                # has not revisited the object, and each republication is a
                # full copy of every point. Hashing the points is what keeps
                # an unchanged object out of the feed.
                if rgb and len(rgb) == len(positions):
                    path = f"/map/objects/rgb_pcd/{obj.object_id}"
                    if not self._unchanged(path, _digest(positions, rgb)):
                        self._log3d(path, rr.Points3D(positions,
                                                      colors=list(rgb)))
                path = f"/map/objects/sem_pcd/{obj.object_id}"
                if not self._unchanged(path, _digest(positions, obj.cls)):
                    self._log3d(path, rr.Points3D(
                        positions, colors=[instance] * len(positions),
                        labels=[obj.cls], show_labels=True))
            # The box is drawn faintly and carries the label. It is the
            # perception layer's own estimate of extent, it is often wrong, and
            # a solid box that is wrong reads as a solid claim: the points are
            # the evidence and should keep the visual weight. Kept because a
            # box far from its own points is exactly what a reader is looking
            # for when checking a detection.
            path = f"/map/objects/bbox/{obj.object_id}"
            centre = [obj.pose.x, obj.pose.y, obj.pose.z]
            half = [obj.bbox.size_x / 2.0, obj.bbox.size_y / 2.0,
                    obj.bbox.size_z / 2.0]
            # rerun draws a label in the colour of the entity that carries it,
            # so the alpha that keeps the box faint also dims its text. When
            # the box is the only thing labelling an object — no cloud to put
            # the name on — it is drawn solid enough for the label to read.
            alpha = 60 if points else 190
            if not self._unchanged(path, _digest(centre, half, obj.cls,
                                                 bool(points))):
                self._log3d(path, rr.Boxes3D(
                    centers=[centre],
                    half_sizes=[half],
                    colors=[(*instance, alpha)],
                    labels=[obj.cls],
                    show_labels=not points,
                ))
        self._clear_gone(drawn)

    def log_map2d(self, occupancy: dict, objects: Iterable[Any],
                  clouds: Optional[dict[str, Sequence]] = None,
                  relations: Optional[Iterable[Any]] = None,
                  robot: Optional[tuple[float, float, float]] = None,
                  footprint: Optional[Sequence[Sequence[float]]] = None) -> None:
        """Draw the floor plan: the same annotations, seen from above.

        The 2D page answers a different question from the 3D one. Read from
        above, an object either sits in the room it is supposed to be in or it
        does not, and a relation either crosses a wall or it does not. The grid
        is the image and everything else is logged in its pixel coordinates, so
        the annotations line up with the map itself rather than with a second
        drawing of it.

        Objects with no point cloud are still drawn as a labelled marker: an
        object the registry holds but the viewer omits reads as a detection
        that was never made.
        """
        if not self._ready or not occupancy:
            return
        texture = self._texture(occupancy)
        to_pixels = _to_pixels(occupancy)
        if texture is None or to_pixels is None:
            return
        rr = self._rr
        if not self._unchanged("/map2d/grid", _digest(
                occupancy.get("png_b64"), occupancy.get("resolution"))):
            self._log2d("/map2d/grid", rr.Image(texture), static=True)

        clouds = clouds or {}
        centres_by_id: dict[str, tuple[float, float]] = {}
        cloud_points: list[list[float]] = []
        cloud_colours: list[tuple[int, int, int]] = []
        centres: list[list[float]] = []
        labels: list[str] = []
        centre_colours: list[tuple[int, int, int]] = []
        for obj in objects:
            instance = instance_colour(obj.object_id)
            for point in clouds.get(obj.object_id) or ():
                cloud_points.append(to_pixels(point[0], point[1]))
                cloud_colours.append(instance)
            centres.append(to_pixels(obj.pose.x, obj.pose.y))
            centres_by_id[obj.object_id] = (obj.pose.x, obj.pose.y)
            labels.append(obj.cls)
            centre_colours.append(instance)
        # Each layer is one entity, logged every tick even when empty: an
        # entity that stops being logged keeps its last value on screen, so
        # skipping would leave a map full of objects the registry has dropped.
        if not self._unchanged("/map2d/objects/points",
                               _digest(cloud_points, cloud_colours)):
            self._log2d("/map2d/objects/points", rr.Points2D(
                cloud_points, colors=cloud_colours, radii=0.7))
        if not self._unchanged("/map2d/objects/labels",
                               _digest(centres, labels, centre_colours)):
            self._log2d("/map2d/objects/labels", rr.Points2D(
                centres, colors=centre_colours, labels=labels, radii=2.5))

        strips = []
        for relation in relations or ():
            subject = _relation_end(relation, "subject", "subject_id")
            target = _relation_end(relation, "target", "object_id")
            ends = [centres_by_id.get(subject or ""),
                    centres_by_id.get(target or "")]
            if None in ends:
                continue
            strips.append([to_pixels(*ends[0]), to_pixels(*ends[1])])
        # Labelled in 3D, unlabelled here: seen from above the edges are short
        # and cluster, and a chip on each one covers the objects the edge is
        # drawn between.
        if not self._unchanged("/map2d/relations", _digest(strips)):
            self._log2d("/map2d/relations", rr.LineStrips2D(
                strips, colors=[_COLOUR_RELATION] * len(strips), radii=0.4))

        if robot is None:
            return
        x, y, yaw = robot
        if footprint:
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            ring = list(footprint) + [footprint[0]]
            self._log2d("/map2d/robot/footprint", rr.LineStrips2D(
                [[to_pixels(x + px * cos_yaw - py * sin_yaw,
                            y + px * sin_yaw + py * cos_yaw)
                  for px, py in ring]],
                colors=[_COLOUR_ROBOT], radii=0.6))
        head = to_pixels(x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw))
        base = to_pixels(x, y)
        self._log2d("/map2d/robot/heading", rr.Arrows2D(
            origins=[base],
            vectors=[[head[0] - base[0], head[1] - base[1]]],
            colors=[_COLOUR_ROBOT]))

    def _clear_gone(self, drawn: set) -> None:
        """Erase the objects that were drawn last tick and are gone now.

        Each object owns its own entity paths, so an object dropped from the
        registry is simply never logged again and rerun keeps showing its last
        state. Clearing is what makes the viewer show the registry as it is
        rather than everything it has ever held.
        """
        rr = self._rr
        for object_id in self._drawn - drawn:
            for group in ("rgb_pcd", "sem_pcd", "bbox"):
                path = f"/map/objects/{group}/{object_id}"
                self._forget(path)
                self._log3d(path, rr.Clear(recursive=True))
        self._drawn = drawn

    def log_relations(self, relations: Iterable[Any],
                      positions: dict[str, tuple[float, float, float]]) -> None:
        """Log one line per relation, skipping any endpoint that is gone.

        An edge to an object no longer in the registry would be drawn to the
        origin, which reads as a real relation to a point on the floor.
        """
        if not self._ready:
            return
        strips, labels = [], []
        for relation in relations:
            subject = positions.get(
                _relation_end(relation, "subject", "subject_id"))
            target = positions.get(
                _relation_end(relation, "target", "object_id"))
            if subject is None or target is None:
                continue
            strips.append([list(subject), list(target)])
            labels.append(str(relation.get("predicate", "")))
        # Logged even when empty: skipping leaves the previous edges on screen
        # after the last relation stops holding. Unchanged edges are still
        # skipped — the entity keeps its value, so re-sending it draws the
        # same picture at the cost of another copy.
        if self._unchanged("/map/relations", _digest(strips, labels)):
            return
        self._log3d("/map/relations", self._rr.LineStrips3D(
            strips, labels=labels, colors=[_COLOUR_RELATION] * len(strips),
            radii=_RELATION_RADIUS_M, show_labels=False,
        ))

    def log_robot(self, pose: tuple[float, float, float],
                  footprint: Sequence[Sequence[float]]) -> None:
        """Log the robot's own footprint polygon and a heading arrow.

        The footprint is Soma's real polygon, transformed into the map frame
        here. Without the arrow a symmetric footprint gives no way to see which
        way the robot faces, which is what a relocalization error looks like.
        """
        if not self._ready:
            return
        rr = self._rr
        x, y, yaw = pose
        if footprint:
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            ring = list(footprint) + [footprint[0]]
            self._log3d("/map/robot/footprint", rr.LineStrips3D(
                [[[x + px * cos_yaw - py * sin_yaw,
                   y + px * sin_yaw + py * cos_yaw,
                   0.02] for px, py in ring]],
                colors=[_COLOUR_ROBOT], radii=0.01,
            ))
        self._log3d("/map/robot/heading", rr.Arrows3D(
            origins=[[x, y, 0.05]],
            vectors=[[0.45 * math.cos(yaw), 0.45 * math.sin(yaw), 0.0]],
            colors=[_COLOUR_ROBOT],
        ))
