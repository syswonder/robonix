from dataclasses import dataclass
import json
import math 

from .config import VerifierConfig, finite_number

NAVIGATE_CONTRACT = "robonix/service/navigation/navigate"

def require_object(value: object, field: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{field} must be an object")
    return value

def require_string(value: object, field: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be a string")
    return value.strip()

def require_bool(value: object, field: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{field} must be a boolean")
    return value

def _reject_constant(value: str): 
    raise ValueError(f"non-standard JSON number: {value}")

def _unique_object(pairs: list) -> dict:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result

def parse_json_object(raw: str, field: str) -> dict:
    raw = require_string(raw, "raw")
    try:
        return require_object(json.loads(raw, parse_constant=_reject_constant, object_pairs_hook=_unique_object), "raw")
    except ValueError as exc:
        raise ValueError(f"invalid {field}: {exc}") from exc

@dataclass(frozen=True)
class VerificationEnvelope:
    target_provider_id: str
    target_args: dict
    scene_provider_id: str
    check_yaw: bool
    expected_map_id: str | None


def parse_envelope(args_json: str) -> VerificationEnvelope:
    """Parse PR #242's target_args envelope, not a target_args_json field."""
    payload = parse_json_object(args_json, "args_json")
    contract = require_string(payload.get("target_contract_id"),
                              "target_contract_id")
    if contract != NAVIGATE_CONTRACT:
        raise ValueError(f"unsupported target contract: {contract}")
    provider = require_string(payload.get("target_provider_id"),
                              "target_provider_id")
    target_args = require_object(payload.get("target_args"), "target_args")
    options = require_object(payload.get("verifier_args", {}), "verifier_args")
    scene = require_string(options.get("scene_provider_id"), "scene_provider_id")
    check_yaw = require_bool(options.get("check_yaw", True), "check_yaw")
    expected_map = None
    if "expected_map_id" in options:
        expected_map = require_string(options["expected_map_id"], "expected_map_id")
    return VerificationEnvelope(provider, target_args, scene, check_yaw, expected_map)


@dataclass(frozen=True)
class NavigationGoal:
    frame_id: str
    x: float
    y: float
    yaw: float | None


def parse_goal(target_args: dict, check_yaw: bool) -> NavigationGoal:
    """Read a map-frame planar goal; heading is required only when checked.

    Match the inspected navigation providers' planar quaternion convention.
    Do not silently invent coordinates or normalize invalid quaternions.
    """
    goal = require_object(target_args.get("goal"), "goal")
    header = require_object(goal.get("header"), "goal.header")
    frame = require_string(header.get("frame_id"), "goal.header.frame_id")
    if frame != "map":
        raise ValueError(f"unsupported frame {frame!r}; only 'map' is supported")
    pose = require_object(goal.get("pose"), "goal.pose")
    position = require_object(pose.get("position"), "goal.pose.position")
    x = finite_number(position.get("x"), "goal.x")
    y = finite_number(position.get("y"), "goal.y")
    z = finite_number(position.get("z", 0.0), "goal.z")
    if abs(z) > 1e-6:
        raise ValueError("only planar navigation goals (z=0) are supported")
    yaw = None
    if check_yaw:
        q = require_object(pose.get("orientation"), "goal.pose.orientation")
        qx = finite_number(q.get("x", 0.0), "orientation.x")
        qy = finite_number(q.get("y", 0.0), "orientation.y")
        qz = finite_number(q.get("z"), "orientation.z")
        qw = finite_number(q.get("w"), "orientation.w")
        if abs(qx) > 1e-6 or abs(qy) > 1e-6:
            raise ValueError("only planar quaternions are supported")
        if abs(math.hypot(qz, qw) - 1.0) > 1e-3:
            raise ValueError("orientation must be a unit quaternion")
        yaw = 2.0 * math.atan2(qz, qw)
    return NavigationGoal(frame, x, y, yaw)


@dataclass(frozen=True)
class RobotContext:
    pose_known: bool
    stale: bool
    map_id: str
    x: float
    y: float
    yaw: float
    reason: str


def parse_robot_context(raw: dict) -> RobotContext:
    """Validate fields consumed from Scene; ignore unrelated context fields."""
    raw = require_object(raw, "Scene response")
    known = require_bool(raw.get("pose_known"), "Scene.pose_known")
    stale = require_bool(raw.get("stale"), "Scene.stale")
    map_id, reason = raw.get("map_id"), raw.get("reason")
    if not isinstance(map_id, str) or not isinstance(reason, str):
        raise ValueError("Scene.map_id and Scene.reason must be strings")
    return RobotContext(
        known, stale, map_id,
        finite_number(raw.get("x"), "Scene.x"),
        finite_number(raw.get("y"), "Scene.y"),
        finite_number(raw.get("yaw"), "Scene.yaw"), reason,
    )


def shortest_angle_error(actual: float, target: float) -> float:
    delta = target - actual
    return abs(math.atan2(math.sin(delta), math.cos(delta)))


def verify_navigation_result(
    goal: NavigationGoal,
    context: RobotContext,
    config: VerifierConfig,
    expected_map_id: str | None = None,
) -> tuple[bool, str]:
    """Return an observation-based verdict; never use navigation's output.

    map_id is a map identity, not a TF frame. A configured identity can be
    compared, but the current contract cannot prove target map provenance.
    """
    if not context.pose_known:
        return False, "Scene robot pose is unknown"
    if context.stale:
        return False, f"Scene robot pose is stale: {context.reason}"
    if not context.map_id.strip():
        return False, "Scene map identity is unavailable"
    if expected_map_id is not None and context.map_id != expected_map_id:
        return False, f"map mismatch: expected={expected_map_id}, actual={context.map_id}"
    distance = math.hypot(context.x - goal.x, context.y - goal.y)
    if not math.isfinite(distance) or distance >= config.distance_tolerance_m:
        return False, (f"position mismatch: distance={distance:.6f}m, "
                       f"tolerance={config.distance_tolerance_m:.6f}m")
    detail = f"map={context.map_id}, distance={distance:.6f}m"
    if goal.yaw is not None:
        error = shortest_angle_error(context.yaw, goal.yaw)
        if error >= config.yaw_tolerance_rad:
            return False, (f"yaw mismatch: error={error:.6f}rad, "
                           f"tolerance={config.yaw_tolerance_rad:.6f}rad")
        detail += f", yaw_error={error:.6f}rad"
    else:
        detail += ", yaw check disabled by rule"
    return True, detail