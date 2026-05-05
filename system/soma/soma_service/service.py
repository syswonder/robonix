# SPDX-License-Identifier: MulanPSL-2.0
"""soma — robot body description package entrypoint.

Loads URDF from disk, publishes `/robot_description` (latched), spawns
`robot_state_publisher` so /tf_static carries the kinematic chain, and
serves three MCP queries: description / footprint / sensor_extrinsics.
"""
import asyncio
import json
import logging
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

logging.basicConfig(level=os.environ.get("SOMA_LOG_LEVEL", "INFO"))
log = logging.getLogger("soma")


# ── PYTHONPATH bootstrap ─────────────────────────────────────────────────────
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
import atlas_pb2 as pb
import atlas_pb2_grpc as pb_grpc
from mcp.server.fastmcp import FastMCP
from robonix_py import mcp_contract

# MCP IO types — auto-generated from soma/srv/*.srv via rbnx codegen.
import soma_mcp
from soma_mcp import (
    GetDescription_Request, GetDescription_Response,
    GetFootprint_Request, GetFootprint_Response,
    GetSensorExtrinsics_Request, GetSensorExtrinsics_Response,
    SensorExtrinsic,
)
from geometry_msgs_mcp import Point, Transform, Vector3, Quaternion


# ── URDF loader ──────────────────────────────────────────────────────────────
class _Body:
    """Parsed URDF + derived geometry. Computed once at startup."""

    def __init__(self, urdf_xml: str, *, base_frame: str = "base_link",
                 model_name_override: Optional[str] = None,
                 footprint_override: Optional[list[tuple[float, float]]] = None):
        self.urdf_xml = urdf_xml
        self.base_frame = base_frame
        self._parse(model_name_override)
        self._footprint_override = footprint_override

    def _parse(self, model_name_override: Optional[str]) -> None:
        try:
            from urdf_parser_py.urdf import URDF
        except ImportError as e:
            log.warning("urdf_parser_py unavailable (%s); soma queries return raw URDF only", e)
            self.model_name = model_name_override or "(unknown)"
            self.mass_kg = -1.0
            self._urdf_obj = None
            return
        try:
            urdf = URDF.from_xml_string(self.urdf_xml)
        except Exception as e:  # noqa: BLE001
            log.warning("URDF parse failed: %s — model_name+mass will be defaults", e)
            self.model_name = model_name_override or "(parse failed)"
            self.mass_kg = -1.0
            self._urdf_obj = None
            return
        self._urdf_obj = urdf
        self.model_name = model_name_override or getattr(urdf, "name", None) or "(unnamed)"
        total = 0.0
        any_mass = False
        for link in getattr(urdf, "links", []) or []:
            inertial = getattr(link, "inertial", None)
            if inertial is not None and inertial.mass is not None:
                total += float(inertial.mass)
                any_mass = True
        self.mass_kg = total if any_mass else -1.0

    def footprint(self) -> tuple[list[tuple[float, float]], float, float]:
        """Returns (polygon, inscribed_radius_m, circumscribed_radius_m).
        Polygon is CCW in `base_frame`. Falls back to a 0.5 m × 0.5 m box
        when neither URDF parse nor override is available — gives nav
        a usable shape for bring-up; override in deploy config."""
        if self._footprint_override:
            pts = self._footprint_override
            return pts, _inscribed(pts), _circumscribed(pts)
        # URDF-derived hull is non-trivial without trimesh / open3d
        # available in this venv; for now we just project axis-aligned
        # bounding boxes from each link's <collision><geometry><box>
        # / <cylinder> primitives onto z=0 and take the AABB. Mesh
        # links are ignored.
        boxes: list[tuple[float, float, float, float]] = []  # (xmin,ymin,xmax,ymax)
        if self._urdf_obj is not None:
            for link in self._urdf_obj.links or []:
                for col in (link.collisions or []):
                    geom = getattr(col, "geometry", None)
                    if geom is None:
                        continue
                    origin = getattr(col, "origin", None)
                    ox = oy = 0.0
                    if origin is not None and origin.xyz is not None:
                        ox, oy = float(origin.xyz[0]), float(origin.xyz[1])
                    # Box: half-extents = size/2.
                    if hasattr(geom, "size") and geom.size is not None:
                        sx, sy, _sz = (float(s) for s in geom.size)
                        boxes.append((ox - sx/2, oy - sy/2, ox + sx/2, oy + sy/2))
                    elif hasattr(geom, "radius") and hasattr(geom, "length"):
                        r = float(geom.radius)
                        boxes.append((ox - r, oy - r, ox + r, oy + r))
                    elif hasattr(geom, "radius"):
                        r = float(geom.radius)
                        boxes.append((ox - r, oy - r, ox + r, oy + r))
        if boxes:
            xmin = min(b[0] for b in boxes)
            ymin = min(b[1] for b in boxes)
            xmax = max(b[2] for b in boxes)
            ymax = max(b[3] for b in boxes)
        else:
            log.info("no parseable collision shapes — falling back to 0.5×0.5 m default footprint")
            xmin, ymin, xmax, ymax = -0.25, -0.25, 0.25, 0.25
        pts = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
        return pts, _inscribed(pts), _circumscribed(pts)

    def sensor_extrinsics(self, kind: str) -> list[dict]:
        """List sensor static transforms (parent_frame → child_frame).
        Heuristic match by link/joint name: anything containing
        camera/lidar/imu/scanner. Returns base_frame → sensor_link
        transforms only; for chained mounts the consumer composes via
        tf_static (which we publish via robot_state_publisher).
        """
        if self._urdf_obj is None:
            return []
        kind_filter = (kind or "").strip().lower()
        kind_keywords = {
            "camera": ("camera", "rgb", "depth", "rgbd"),
            "lidar":  ("lidar", "scanner", "hokuyo", "velodyne", "livox"),
            "imu":    ("imu", "gyro", "accel"),
        }
        out = []
        for joint in (self._urdf_obj.joints or []):
            child_name = (joint.child or "").lower()
            if not child_name:
                continue
            matched_kind = None
            for k, kws in kind_keywords.items():
                if any(kw in child_name for kw in kws):
                    matched_kind = k
                    break
            if matched_kind is None:
                continue
            if kind_filter and matched_kind != kind_filter:
                continue
            origin = getattr(joint, "origin", None)
            tx = ty = tz = 0.0
            qx = qy = qz = 0.0
            qw = 1.0
            if origin is not None:
                if origin.xyz is not None:
                    tx, ty, tz = (float(v) for v in origin.xyz)
                if origin.rpy is not None:
                    qx, qy, qz, qw = _rpy_to_quat(*origin.rpy)
            out.append({
                "sensor_name": joint.child,
                "parent_frame": joint.parent,
                "child_frame": joint.child,
                "tx": tx, "ty": ty, "tz": tz,
                "qx": qx, "qy": qy, "qz": qz, "qw": qw,
            })
        return out


def _inscribed(pts: list[tuple[float, float]]) -> float:
    """Cheap lower bound: min dist from origin to any edge of the polygon
    (assumes origin is inside; true for sensible footprints)."""
    if len(pts) < 3:
        return 0.0
    best = float("inf")
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        dx, dy = x2 - x1, y2 - y1
        seg = max(1e-9, math.hypot(dx, dy))
        # Distance from origin to infinite line through (p1, p2).
        d = abs(dx * y1 - dy * x1) / seg
        best = min(best, d)
    return best


def _circumscribed(pts: list[tuple[float, float]]) -> float:
    return max(math.hypot(x, y) for x, y in pts) if pts else 0.0


def _rpy_to_quat(r: float, p: float, y: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


# ── ROS2 publisher + robot_state_publisher subprocess ───────────────────────
class _RosPublisher:
    """Background rclpy node publishing /robot_description (latched)
    and supervising a robot_state_publisher subprocess so /tf_static
    carries the rigid-body chain. Lazy-imports rclpy so unit tests
    don't drag ROS2 in."""

    def __init__(self, body: _Body) -> None:
        self.body = body
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._rsp: Optional[subprocess.Popen] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="soma-rclpy", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._rsp is not None:
            try:
                self._rsp.terminate()
                self._rsp.wait(timeout=2.0)
            except Exception:  # noqa: BLE001
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
            from std_msgs.msg import String
        except ImportError as e:
            log.warning("rclpy unavailable (%s); /robot_description will NOT be published", e)
            return
        rclpy.init(args=None)
        node = Node("soma")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        pub = node.create_publisher(String, "/robot_description", qos)
        msg = String()
        msg.data = self.body.urdf_xml
        pub.publish(msg)
        log.info("[soma] /robot_description published (%d bytes, latched)", len(msg.data))

        # Spawn robot_state_publisher subprocess. Pass the URDF as the
        # `robot_description` parameter directly; this avoids races
        # against our latched topic publisher.
        try:
            self._rsp = subprocess.Popen(
                ["ros2", "run", "robot_state_publisher", "robot_state_publisher",
                 "--ros-args",
                 "-p", f"robot_description:={self.body.urdf_xml}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            log.info("[soma] robot_state_publisher spawned (pid=%s)", self._rsp.pid)
        except Exception as e:  # noqa: BLE001
            log.warning("[soma] failed to spawn robot_state_publisher: %s", e)

        while not self._stop.is_set():
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except Exception:  # noqa: BLE001
                break
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


# ── MCP handlers (atlas-registered via @mcp_contract) ───────────────────────
mcp = FastMCP("soma")
_BODY: Optional[_Body] = None


@mcp_contract(mcp, contract_id="robonix/system/soma/description")
async def get_description(_msg: GetDescription_Request) -> GetDescription_Response:
    """Get the robot's URDF + high-level metadata.
    Contract: robonix/system/soma/description."""
    if _BODY is None:
        return GetDescription_Response(
            urdf_xml="", model_name="no_description", mass_kg=-1.0,
            base_frame="base_link",
        )
    return GetDescription_Response(
        urdf_xml=_BODY.urdf_xml,
        model_name=_BODY.model_name,
        mass_kg=float(_BODY.mass_kg),
        base_frame=_BODY.base_frame,
    )


@mcp_contract(mcp, contract_id="robonix/system/soma/footprint")
async def get_footprint(_msg: GetFootprint_Request) -> GetFootprint_Response:
    """Get the robot's 2D collision footprint polygon in `base_frame`.
    Contract: robonix/system/soma/footprint."""
    if _BODY is None:
        return GetFootprint_Response(
            points=[], base_frame="base_link",
            inscribed_radius_m=0.0, circumscribed_radius_m=0.0,
        )
    pts, inscribed, circumscribed = _BODY.footprint()
    return GetFootprint_Response(
        points=[Point(x=x, y=y, z=0.0) for (x, y) in pts],
        base_frame=_BODY.base_frame,
        inscribed_radius_m=float(inscribed),
        circumscribed_radius_m=float(circumscribed),
    )


@mcp_contract(mcp, contract_id="robonix/system/soma/sensor_extrinsics")
async def get_sensor_extrinsics(msg: GetSensorExtrinsics_Request) -> GetSensorExtrinsics_Response:
    """List static-mount transforms for sensors declared in the URDF.
    Filter via `sensor_kind` (camera / lidar / imu / "" = all).
    Contract: robonix/system/soma/sensor_extrinsics."""
    if _BODY is None:
        return GetSensorExtrinsics_Response(sensors=[])
    raw = _BODY.sensor_extrinsics(msg.sensor_kind or "")
    out = []
    for s in raw:
        out.append(SensorExtrinsic(
            sensor_name=s["sensor_name"],
            parent_frame=s["parent_frame"],
            child_frame=s["child_frame"],
            transform=Transform(
                translation=Vector3(x=s["tx"], y=s["ty"], z=s["tz"]),
                rotation=Quaternion(x=s["qx"], y=s["qy"], z=s["qz"], w=s["qw"]),
            ),
        ))
    return GetSensorExtrinsics_Response(sensors=out)


# ── Atlas registration boilerplate (mirrors memsearch_service) ──────────────
def _decl_mcp(stub, cap_id: str, contract_id: str, port: int, fn) -> None:
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
            description=description, input_schema_json=schema_json,
        )),
    ))


def _load_config() -> dict:
    """rbnx boot writes a per-instance JSON config to a path passed via
    RBNX_CONFIG_FILE. Falls back to env vars when run by hand."""
    cfg_path = os.environ.get("RBNX_CONFIG_FILE", "")
    if cfg_path and Path(cfg_path).is_file():
        try:
            return json.loads(Path(cfg_path).read_text())
        except Exception as e:  # noqa: BLE001
            log.warning("[soma] failed to read %s: %s", cfg_path, e)
    return {}


def main():
    cfg = _load_config()
    urdf_path = cfg.get("urdf_path") or os.environ.get("SOMA_URDF_PATH", "")
    base_frame = cfg.get("base_frame") or "base_link"
    model_name_override = cfg.get("model_name")
    footprint_override = cfg.get("footprint_xy_pts")
    if footprint_override is not None:
        try:
            footprint_override = [(float(x), float(y)) for (x, y) in footprint_override]
        except Exception as e:  # noqa: BLE001
            log.warning("invalid footprint_xy_pts in config (%s); ignoring", e)
            footprint_override = None

    global _BODY
    if not urdf_path:
        log.warning("[soma] no urdf_path in config — every query will return no_description")
    else:
        try:
            urdf_xml = Path(urdf_path).expanduser().read_text()
            _BODY = _Body(
                urdf_xml=urdf_xml, base_frame=base_frame,
                model_name_override=model_name_override,
                footprint_override=footprint_override,
            )
            log.info(
                "[soma] loaded URDF: model=%s mass=%.2f kg base=%s (%d bytes)",
                _BODY.model_name, _BODY.mass_kg, _BODY.base_frame, len(urdf_xml),
            )
        except Exception as e:  # noqa: BLE001
            log.error("[soma] failed to load URDF from %s: %s", urdf_path, e)

    # Start ROS2 publisher in a background thread.
    pub: Optional[_RosPublisher] = None
    if _BODY is not None:
        pub = _RosPublisher(_BODY)
        pub.start()

    # Atlas register + DeclareInterface for all 3 caps.
    channel = grpc.insecure_channel(os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051"))
    stub = pb_grpc.AtlasStub(channel)
    port = int(os.environ.get("SOMA_MCP_PORT", "50108"))
    cap_id = os.environ.get("ROBONIX_CAPABILITY_ID", "com.robonix.system.soma")

    try:
        pkg_dir = os.environ.get("ROBONIX_PKG_HOST_DIR", "")
        md_path = f"{pkg_dir}/CAPABILITY.md" if pkg_dir else ""
        stub.RegisterCapability(pb.RegisterCapabilityRequest(
            capability_id=cap_id,
            namespace="robonix/system/soma",
            capability_md_path=md_path,
        ))
        _decl_mcp(stub, cap_id, "robonix/system/soma/description",       port, get_description)
        _decl_mcp(stub, cap_id, "robonix/system/soma/footprint",         port, get_footprint)
        _decl_mcp(stub, cap_id, "robonix/system/soma/sensor_extrinsics", port, get_sensor_extrinsics)
        log.info("[soma] registered cap %s → 3 interfaces on port %d", cap_id, port)
    except Exception as e:  # noqa: BLE001
        log.warning("[soma] atlas registration failed: %s", e)

    log.info("[soma] MCP HTTP serving on 127.0.0.1:%d", port)
    import uvicorn
    uvicorn.run(mcp.streamable_http_app(), host="127.0.0.1", port=port, log_level="warning")


if __name__ == "__main__":
    main()
