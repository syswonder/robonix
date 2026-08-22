#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Ground-truth helpers for the Webots Scene object-quality acceptance gate.

Runtime Scene code must remain deployment-neutral.  The simulator, however,
has an authoritative world file.  This module resolves a named robot and a
named fixture from that file, converts the fixture into the initial map frame,
and computes label/center/box/point-cloud metrics without copying world
coordinates into the verifier.
"""

from __future__ import annotations

import json
import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

_FLOAT = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


@dataclass(frozen=True)
class ObjectGroundTruth:
    """One simulator fixture expressed in Scene's initial map frame."""

    label: str
    center_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    yaw_rad: float
    association_radius_m: float
    node_type: str = ""
    name: str = ""
    evaluate_yaw: bool = True


@dataclass(frozen=True)
class GroundTruthMatch:
    """One position-associated Scene object and simulator target."""

    truth_index: int
    object_index: int
    truth: ObjectGroundTruth
    obj: dict[str, Any]
    center_error_m: float


@dataclass(frozen=True)
class SemanticObjectGroundTruth:
    """One WBT semantic asset used by the cross-world detection benchmark.

    Unlike :class:`ObjectGroundTruth`, many extents come from reviewed
    type-level WBT approximations. They support a cross-world geometry proxy;
    separately curated exact fixtures remain the stronger acceptance evidence.
    """

    identity: str
    label: str
    center_m: tuple[float, float, float]
    association_radius_m: float
    node_type: str
    name: str
    size_m: tuple[float, float, float]
    yaw_rad: float = 0.0
    evaluate_yaw: bool = True


def transform_planar_center_yaw(
    center_m: Sequence[float],
    yaw_rad: float,
    *,
    translation_m: Sequence[float],
    alignment_yaw_rad: float,
) -> tuple[tuple[float, float, float], float]:
    """Apply one ``T(target <- source)`` planar transform to a 3-D pose."""

    if len(center_m) != 3:
        raise ValueError("truth center_m must have three values")
    if len(translation_m) != 3:
        raise ValueError("truth alignment translation_m must have three values")
    center = tuple(float(value) for value in center_m)
    translation = tuple(float(value) for value in translation_m)
    yaw = float(yaw_rad)
    alignment_yaw = float(alignment_yaw_rad)
    if not all(
        math.isfinite(value)
        for value in (*center, *translation, yaw, alignment_yaw)
    ):
        raise ValueError("truth pose and alignment must contain only finite values")
    cosine = math.cos(alignment_yaw)
    sine = math.sin(alignment_yaw)
    return (
        (
            translation[0] + cosine * center[0] - sine * center[1],
            translation[1] + sine * center[0] + cosine * center[1],
            translation[2] + center[2],
        ),
        _normalise_angle(yaw + alignment_yaw),
    )


def transform_semantic_inventory_planar(
    truths: Sequence[SemanticObjectGroundTruth],
    *,
    translation_m: Sequence[float],
    yaw_rad: float,
) -> tuple[SemanticObjectGroundTruth, ...]:
    """Express robot-initial WBT truth in one live SLAM map frame.

    WBT inventory coordinates are deliberately stored relative to the robot's
    checked-in initial pose.  A fresh mapping session is free to choose a
    different map origin and heading, so a live benchmark must capture
    ``T(map <- initial_robot)`` before motion and apply it once.  Keeping this
    calibration in the evaluator avoids assuming that ``map`` starts at
    ``(0, 0, 0)``.
    """

    aligned = []
    for truth in truths:
        center, yaw = transform_planar_center_yaw(
            truth.center_m,
            truth.yaw_rad,
            translation_m=translation_m,
            alignment_yaw_rad=yaw_rad,
        )
        aligned.append(
            SemanticObjectGroundTruth(
                identity=truth.identity,
                label=truth.label,
                center_m=center,
                association_radius_m=truth.association_radius_m,
                node_type=truth.node_type,
                name=truth.name,
                size_m=truth.size_m,
                yaw_rad=yaw,
                evaluate_yaw=truth.evaluate_yaw,
            )
        )
    return tuple(aligned)


def _node_blocks(world_text: str, node_type: str) -> Iterable[str]:
    """Yield balanced ``NodeType { ... }`` blocks from a Webots world."""

    pattern = re.compile(rf"(?m)^\s*{re.escape(node_type)}\s*\{{")
    for match in pattern.finditer(world_text):
        brace = world_text.find("{", match.start())
        depth = 0
        quoted = False
        escaped = False
        for index in range(brace, len(world_text)):
            char = world_text[index]
            if quoted:
                if escaped:
                    escaped = False
                elif char == "\\":
                    escaped = True
                elif char == '"':
                    quoted = False
                continue
            if char == '"':
                quoted = True
            elif char == "{":
                depth += 1
            elif char == "}":
                depth -= 1
                if depth == 0:
                    yield world_text[match.start() : index + 1]
                    break


def _named_node(world_text: str, node_type: str, name: str) -> str:
    name_pattern = re.compile(rf'(?m)^\s*name\s+"{re.escape(name)}"\s*$')
    matches = [
        block
        for block in _node_blocks(world_text, node_type)
        if name_pattern.search(block)
    ]
    if len(matches) != 1:
        raise ValueError(
            f"expected one {node_type} named {name!r}, found {len(matches)}"
        )
    return matches[0]


def _vector_field(block: str, field: str, length: int) -> tuple[float, ...]:
    values = r"\s+".join(f"({_FLOAT})" for _ in range(length))
    match = re.search(rf"(?m)^\s*{re.escape(field)}\s+{values}\s*$", block)
    if match is None:
        raise ValueError(f"node is missing {field}")
    parsed = tuple(float(value) for value in match.groups())
    if not all(math.isfinite(value) for value in parsed):
        raise ValueError(f"{field} contains a non-finite value")
    return parsed


def _optional_vector_field(
    block: str,
    field: str,
    length: int,
) -> tuple[float, ...] | None:
    try:
        return _vector_field(block, field, length)
    except ValueError:
        return None


def _optional_scalar_field(block: str, field: str) -> float | None:
    match = re.search(
        rf"(?m)^\s*{re.escape(field)}\s+({_FLOAT})\s*$",
        block,
    )
    if match is None:
        return None
    value = float(match.group(1))
    if not math.isfinite(value):
        raise ValueError(f"{field} contains a non-finite value")
    return value


def _optional_float_list_field(
    block: str,
    field: str,
) -> tuple[float, ...] | None:
    """Read one WBT ``MFFloat`` field without consuming the next array.

    Webots accepts both comma-separated and whitespace-separated values, and
    checked-in worlds use both single-line and multi-line arrays.  Match the
    named field's first balanced-looking bracket only; semantic fixture fields
    contain numeric literals rather than nested arrays.
    """

    match = re.search(
        rf"(?ms)^\s*{re.escape(field)}\s*\[(.*?)\]",
        block,
    )
    if match is None:
        return None
    payload = re.sub(r"(?m)#.*$", "", match.group(1))
    values = tuple(float(value) for value in re.findall(_FLOAT, payload))
    if not values:
        raise ValueError(f"{field} must contain at least one value")
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{field} contains a non-finite value")
    return values


def _optional_bool_field(block: str, field: str) -> bool | None:
    match = re.search(
        rf"(?m)^\s*{re.escape(field)}\s+(TRUE|FALSE)\s*$",
        block,
    )
    if match is None:
        return None
    return match.group(1) == "TRUE"


def _planar_yaw(rotation: Sequence[float]) -> float:
    """Project a Webots axis-angle rotation onto the world XY plane.

    Real checked-in worlds are not guaranteed to encode a planar pose with an
    exact ``0 0 ±1`` axis.  In particular ``apartment.wbt`` carries a tiny
    roll/pitch component on the robot node.  Rejecting that representation
    makes the benchmark depend on serialization noise rather than the actual
    heading.  Build the first column of the 3-D rotation matrix and use its XY
    direction as the projected forward heading.
    """

    axis_x, axis_y, axis_z, angle = rotation
    norm = math.sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z)
    if norm <= 1e-12:
        return 0.0
    axis_x /= norm
    axis_y /= norm
    axis_z /= norm
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus_cosine = 1.0 - cosine
    forward_x = cosine + axis_x * axis_x * one_minus_cosine
    forward_y = axis_z * sine + axis_y * axis_x * one_minus_cosine
    if math.hypot(forward_x, forward_y) <= 1e-9:
        raise ValueError("rotation has no stable projection onto the XY plane")
    return math.atan2(forward_y, forward_x)


def _normalise_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _door_leaf_geometry(
    block: str,
    frame_size_m: Sequence[float],
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    float,
]:
    """Resolve the actual leaf box represented by Webots R2025a ``Door``.

    ``Door.size`` describes the surrounding wall/frame envelope, not the
    moving door leaf.  The PROTO places a thinner leaf off the node origin and
    optionally rotates it around a side hinge.  Using the frame envelope as
    object truth makes the review box visibly too thick/tall and shifts it
    away from the rendered leaf.
    """

    size_x, size_y, size_z = (float(value) for value in frame_size_m)
    frame_size = _optional_vector_field(block, "frameSize", 3) or (
        0.05,
        0.05,
        0.05,
    )
    door_thickness = _optional_scalar_field(block, "doorThickness")
    if door_thickness is None:
        door_thickness = 0.05
    frame_height = _optional_scalar_field(block, "frameHeight")
    if frame_height is None:
        frame_height = 2.0
    frame_height = min(frame_height, size_z)
    position = _optional_scalar_field(block, "position") or 0.0
    can_be_open = _optional_bool_field(block, "canBeOpen")
    if can_be_open is False:
        position = 0.0
    joint_at_left = _optional_bool_field(block, "jointAtLeft")
    mirror_factor = -1.0 if joint_at_left is not False else 1.0

    hinge_x = 0.5 * size_x + frame_size[0]
    hinge_y = mirror_factor * (0.5 * size_y - 0.5 * frame_size[1])
    closed_center = (
        hinge_x + 0.5 * door_thickness,
        0.0,
        0.5 * frame_height,
    )
    cosine = math.cos(position)
    sine = math.sin(position)
    relative_x = closed_center[0] - hinge_x
    relative_y = closed_center[1] - hinge_y
    center = (
        hinge_x + cosine * relative_x - sine * relative_y,
        hinge_y + sine * relative_x + cosine * relative_y,
        closed_center[2],
    )
    leaf_size = (
        door_thickness,
        size_y - frame_size[1],
        frame_height - 0.5 * frame_size[2],
    )
    if any(value <= 0.0 for value in (*center[2:], *leaf_size)):
        raise ValueError("Door fields resolve to invalid leaf geometry")
    return center, leaf_size, position


def _cabinet_geometry(
    block: str,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    float,
]:
    """Resolve the R2025a ``Cabinet`` frame envelope from its grid fields.

    ``Cabinet`` has no public ``size`` field.  Its PROTO derives the width and
    height from ``columnsWidths`` and ``rowsHeights`` and places the node origin
    on the back/bottom edge.  A type-level box therefore gives both a wrong
    extent and a wrong centre for almost every cabinet in the benchmark.
    """

    depth = _optional_scalar_field(block, "depth")
    outer_thickness = _optional_scalar_field(block, "outerThickness")
    depth = 0.5 if depth is None else depth
    outer_thickness = 0.03 if outer_thickness is None else outer_thickness
    rows = _optional_float_list_field(block, "rowsHeights") or (
        0.24,
        0.2,
        0.2,
        0.4,
        0.4,
    )
    columns = _optional_float_list_field(block, "columnsWidths") or (
        0.4,
        0.17,
        0.17,
    )
    if any(
        not math.isfinite(value) or value <= 0.0
        for value in (depth, outer_thickness, *rows, *columns)
    ):
        raise ValueError("Cabinet fields resolve to invalid frame geometry")
    width = 2.0 * outer_thickness + sum(columns)
    height = 2.0 * outer_thickness + sum(rows)
    size = (depth, width, height)
    center = (0.5 * depth, 0.0, 0.5 * height)
    return center, size, 0.0


def _window_glass_geometry(
    block: str,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    float,
]:
    """Resolve the R2025a ``Window`` glass opening, excluding wall panels."""

    default_size = (0.2, 0.8, 2.4)
    size = _optional_vector_field(block, "size", 3) or default_size
    if any(not math.isfinite(value) or value <= 0.0 for value in size):
        size = default_size
    window_thickness = _optional_scalar_field(block, "windowThickness")
    bottom_wall_height = _optional_scalar_field(block, "bottomWallHeight")
    window_height = _optional_scalar_field(block, "windowHeight")
    frame_size = _optional_vector_field(block, "frameSize", 3)
    window_thickness = (
        0.05
        if window_thickness is None or window_thickness <= 0.0
        else window_thickness
    )
    bottom_wall_height = (
        0.7
        if bottom_wall_height is None or bottom_wall_height < 0.0
        else bottom_wall_height
    )
    window_height = (
        1.4 if window_height is None or window_height <= 0.0 else window_height
    )
    if frame_size is None or any(value <= 0.0 for value in frame_size):
        frame_size = (0.05, 0.02, 0.05)

    # Window.proto resets this group together when their relationship cannot
    # form a valid opening.  Mirror that behavior so the evaluator describes
    # the geometry Webots actually instantiates.
    if (
        window_height <= 2.0 * frame_size[2]
        or window_height + bottom_wall_height > size[2]
        or size[1] <= 2.0 * frame_size[1]
    ):
        size = default_size
        bottom_wall_height = 0.7
        window_height = 1.4
        frame_size = (0.05, 0.02, 0.05)

    glass_size = (
        window_thickness,
        size[1] - 2.0 * frame_size[1],
        window_height - 2.0 * frame_size[2],
    )
    center = (
        0.0,
        0.0,
        bottom_wall_height + 0.5 * window_height,
    )
    if any(
        not math.isfinite(value) or value <= 0.0
        for value in (*glass_size, center[2])
    ):
        raise ValueError("Window fields resolve to invalid glass geometry")
    return center, glass_size, 0.0


def _target_size(target: str, target_spec: dict[str, Any]) -> tuple[float, ...]:
    if field := target_spec.get("bbox_size_field"):
        size_m = _vector_field(target, str(field), 3)
    else:
        try:
            size_m = tuple(float(value) for value in target_spec["bbox_size_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("target requires bbox_size_field or bbox_size_m") from exc
    if len(size_m) != 3 or any(
        not math.isfinite(value) or value <= 0.0 for value in size_m
    ):
        raise ValueError("target dimensions must contain three positive SI values")
    return size_m


def _target_bbox_center_offset(
    target_spec: dict[str, Any],
    size_m: Sequence[float],
) -> tuple[float, float, float]:
    """Return the bounding-box center in the target node's local frame."""

    raw_offset = target_spec.get("bbox_center_offset_m")
    if raw_offset is None:
        # Most floor-standing Webots assets place their origin at the bottom
        # centre.  Keep that convention as the fixture default, while allowing
        # compound PROTOs to describe their actual local bounding envelope.
        return (0.0, 0.0, float(size_m[2]) * 0.5)
    try:
        offset = tuple(float(value) for value in raw_offset)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "target.bbox_center_offset_m must contain three finite SI values"
        ) from exc
    if len(offset) != 3 or any(not math.isfinite(value) for value in offset):
        raise ValueError(
            "target.bbox_center_offset_m must contain three finite SI values"
        )
    return (offset[0], offset[1], offset[2])


def _resolve_target(
    world_text: str,
    robot: str,
    target_spec: dict[str, Any],
    *,
    map_ground_z_world_m: float,
) -> ObjectGroundTruth:
    target = _named_node(
        world_text,
        str(target_spec["node_type"]),
        str(target_spec["name"]),
    )

    robot_xyz = _vector_field(robot, "translation", 3)
    target_xyz = _vector_field(target, "translation", 3)
    robot_yaw = _planar_yaw(_vector_field(robot, "rotation", 4))
    target_yaw = _planar_yaw(_vector_field(target, "rotation", 4))
    size_m = _target_size(target, target_spec)
    local_center = _target_bbox_center_offset(target_spec, size_m)

    # Webots x/y are the ground plane and z is vertical.  The mapping stack
    # anchors map XY to the robot's initial pose, so rotate the world-space
    # displacement by the inverse initial robot yaw.  Compound PROTO bounding
    # objects do not necessarily share the node origin, so first rotate their
    # local bounding-envelope centre into world coordinates.
    target_cosine = math.cos(target_yaw)
    target_sine = math.sin(target_yaw)
    world_center_x = (
        target_xyz[0] + target_cosine * local_center[0] - target_sine * local_center[1]
    )
    world_center_y = (
        target_xyz[1] + target_sine * local_center[0] + target_cosine * local_center[1]
    )
    world_center_z = target_xyz[2] + local_center[2]
    dx = world_center_x - robot_xyz[0]
    dy = world_center_y - robot_xyz[1]
    cosine = math.cos(-robot_yaw)
    sine = math.sin(-robot_yaw)
    center_x = cosine * dx - sine * dy
    center_y = sine * dx + cosine * dy
    center_z = world_center_z - map_ground_z_world_m
    radius = float(target_spec["association_radius_m"])
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("target.association_radius_m must be positive")
    return ObjectGroundTruth(
        label=str(target_spec["label"]).strip().lower(),
        center_m=(center_x, center_y, center_z),
        size_m=(size_m[0], size_m[1], size_m[2]),
        yaw_rad=_normalise_angle(target_yaw - robot_yaw),
        association_radius_m=radius,
        node_type=str(target_spec["node_type"]),
        name=str(target_spec["name"]),
        evaluate_yaw=bool(target_spec.get("evaluate_yaw", True)),
    )


def _load_world(
    config_path: Path,
    repository_root: Path,
) -> tuple[dict[str, Any], str, str]:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    world_relative = str(config["world"])
    world_path = repository_root / world_relative
    if not world_path.is_file():
        raise ValueError(f"world file does not exist: {world_relative}")
    return config, world_path.read_text(encoding="utf-8"), world_relative


def load_ground_truth(
    config_path: Path,
    *,
    repository_root: Path,
) -> tuple[ObjectGroundTruth, dict[str, Any]]:
    """Load the legacy single-target fixture, resolving its pose from WBT."""

    config, world_text, _ = _load_world(config_path, repository_root)
    robot_spec = config["robot"]
    robot = _named_node(
        world_text,
        str(robot_spec["node_type"]),
        str(robot_spec["name"]),
    )
    truth = _resolve_target(
        world_text,
        robot,
        config["target"],
        map_ground_z_world_m=float(config.get("map_ground_z_world_m", 0.0)),
    )
    return truth, config


def load_ground_truths(
    config_path: Path,
    *,
    repository_root: Path,
) -> tuple[tuple[ObjectGroundTruth, ...], dict[str, Any]]:
    """Resolve every multi-object target from the configured WBT world."""

    config, world_text, _ = _load_world(config_path, repository_root)
    target_specs = config.get("targets")
    if not isinstance(target_specs, list) or not target_specs:
        # A single-target fixture remains a valid one-element scene fixture.
        target_specs = [config["target"]]
    robot_spec = config["robot"]
    robot = _named_node(
        world_text,
        str(robot_spec["node_type"]),
        str(robot_spec["name"]),
    )
    truths = tuple(
        _resolve_target(
            world_text,
            robot,
            target_spec,
            map_ground_z_world_m=float(config.get("map_ground_z_world_m", 0.0)),
        )
        for target_spec in target_specs
    )
    identities = {(truth.node_type, truth.name) for truth in truths}
    if len(identities) != len(truths):
        raise ValueError("targets must identify distinct WBT nodes")
    return truths, config


def load_semantic_inventory(
    benchmark_path: Path,
    *,
    world_id: str,
    repository_root: Path,
) -> tuple[tuple[SemanticObjectGroundTruth, ...], dict[str, Any]]:
    """Build a complete semantic inventory from a checked-in WBT world.

    The benchmark JSON owns only type taxonomy, approximate physical extents,
    and robot identity.  Positions and rotations always come from the WBT;
    no world coordinates are duplicated in fixtures.
    """

    benchmark = json.loads(benchmark_path.read_text(encoding="utf-8"))
    worlds = benchmark.get("worlds") or []
    world_spec = next(
        (item for item in worlds if str(item.get("id")) == world_id),
        None,
    )
    if world_spec is None:
        raise ValueError(f"unknown benchmark world: {world_id}")
    world_path = repository_root / str(world_spec["world"])
    if not world_path.is_file():
        raise ValueError(f"world file does not exist: {world_spec['world']}")
    world_text = world_path.read_text(encoding="utf-8")
    robot_spec = world_spec["robot"]
    robot = _named_node(
        world_text,
        str(robot_spec["node_type"]),
        str(robot_spec["name"]),
    )
    robot_xyz = _vector_field(robot, "translation", 3)
    robot_rotation = _optional_vector_field(robot, "rotation", 4)
    robot_yaw = _planar_yaw(robot_rotation or (0.0, 0.0, 1.0, 0.0))
    map_ground_z_world_m = float(world_spec.get("map_ground_z_world_m", 0.0))
    semantic_types = benchmark.get("semantic_types") or {}
    truths: list[SemanticObjectGroundTruth] = []

    for node_type, type_spec in semantic_types.items():
        blocks = list(_node_blocks(world_text, str(node_type)))
        for occurrence, block in enumerate(blocks, 1):
            translation = _optional_vector_field(block, "translation", 3)
            if translation is None:
                continue
            rotation = _optional_vector_field(block, "rotation", 4)
            target_yaw = _planar_yaw(rotation or (0.0, 0.0, 1.0, 0.0))
            explicit_size = _optional_vector_field(block, "size", 3)
            raw_size = explicit_size or tuple(type_spec["size_m"])
            if len(raw_size) != 3 or any(
                not math.isfinite(float(value)) or float(value) <= 0.0
                for value in raw_size
            ):
                raise ValueError(f"invalid semantic size for {node_type}")
            size_m = tuple(float(value) for value in raw_size)
            raw_offset = type_spec.get("center_offset_m")
            if raw_offset is None:
                center_offset = (0.0, 0.0, size_m[2] * 0.5)
            else:
                center_offset = tuple(float(value) for value in raw_offset)
            if len(center_offset) != 3:
                raise ValueError(f"invalid semantic center offset for {node_type}")
            local_yaw = 0.0
            if str(node_type) == "Door":
                center_offset, size_m, local_yaw = _door_leaf_geometry(
                    block,
                    size_m,
                )
            elif str(node_type) == "Cabinet":
                center_offset, size_m, local_yaw = _cabinet_geometry(block)
            elif str(node_type) == "Window":
                center_offset, size_m, local_yaw = _window_glass_geometry(block)
            cosine = math.cos(target_yaw)
            sine = math.sin(target_yaw)
            world_center_x = (
                translation[0] + cosine * center_offset[0] - sine * center_offset[1]
            )
            world_center_y = (
                translation[1] + sine * center_offset[0] + cosine * center_offset[1]
            )
            world_center_z = translation[2] + center_offset[2]
            dx = world_center_x - robot_xyz[0]
            dy = world_center_y - robot_xyz[1]
            map_cosine = math.cos(-robot_yaw)
            map_sine = math.sin(-robot_yaw)
            center_m = (
                map_cosine * dx - map_sine * dy,
                map_sine * dx + map_cosine * dy,
                world_center_z - map_ground_z_world_m,
            )
            name_match = re.search(
                r'(?m)^\s*name\s+"([^"]+)"\s*$',
                block,
            )
            name = (
                name_match.group(1)
                if name_match is not None
                else f"{node_type}#{occurrence}"
            )
            identity = f"{node_type}:{name}"
            radius = float(type_spec["association_radius_m"])
            if not math.isfinite(radius) or radius <= 0.0:
                raise ValueError(f"invalid semantic association radius for {node_type}")
            truths.append(
                SemanticObjectGroundTruth(
                    identity=identity,
                    label=str(type_spec["label"]).strip().lower(),
                    center_m=center_m,
                    association_radius_m=radius,
                    node_type=str(node_type),
                    name=name,
                    size_m=size_m,
                    yaw_rad=_normalise_angle(
                        target_yaw + local_yaw - robot_yaw
                    ),
                    evaluate_yaw=bool(type_spec.get("evaluate_yaw", True)),
                )
            )

    identities = {truth.identity for truth in truths}
    if len(identities) != len(truths):
        raise ValueError(f"{world_id} semantic identities are not unique")
    return tuple(truths), benchmark


def nearest_object(
    objects: Sequence[dict[str, Any]],
    truth: ObjectGroundTruth,
    *,
    center_key: str = "pose",
) -> dict[str, Any] | None:
    """Associate by metric position so an incorrect label remains measurable."""

    candidates: list[tuple[float, dict[str, Any]]] = []
    for obj in objects:
        if bool(obj.get("missing")):
            continue
        if str(obj.get("cls") or "").strip().lower() == "robot":
            continue
        center = obj.get(center_key) or {}
        if isinstance(center, (list, tuple)):
            if len(center) < 2:
                continue
            x, y = float(center[0]), float(center[1])
        else:
            try:
                x, y = float(center["x"]), float(center["y"])
            except (KeyError, TypeError, ValueError):
                continue
        distance = math.hypot(x - truth.center_m[0], y - truth.center_m[1])
        candidates.append((distance, obj))
    if not candidates:
        return None
    distance, obj = min(candidates, key=lambda item: item[0])
    return obj if distance <= truth.association_radius_m else None


def _object_center(
    obj: dict[str, Any],
    center_key: str,
) -> tuple[float, float, float] | None:
    center = obj.get(center_key) or {}
    try:
        if isinstance(center, (list, tuple)):
            if len(center) < 3:
                return None
            values = float(center[0]), float(center[1]), float(center[2])
        else:
            values = float(center["x"]), float(center["y"]), float(center["z"])
    except (KeyError, TypeError, ValueError):
        return None
    return values if all(math.isfinite(value) for value in values) else None


def _object_bbox_volume(obj: dict[str, Any]) -> float | None:
    bbox = obj.get("bbox") or {}
    try:
        sizes = (
            float(bbox["size_x"]),
            float(bbox["size_y"]),
            float(bbox["size_z"]),
        )
    except (KeyError, TypeError, ValueError):
        return None
    if any(not math.isfinite(value) or value <= 0.0 for value in sizes):
        return None
    return sizes[0] * sizes[1] * sizes[2]


def _semantic_size_bucket(truth: SemanticObjectGroundTruth) -> str:
    """Stratify by equivalent-cube edge, robust to long thin objects.

    Maximum dimension put every Office asset into the old "large" bucket
    because keyboards, lamps and windows all have one long axis. The cube root
    of physical volume preserves actual scale while remaining orientation and
    class independent. The fixed 0.30 m and 0.75 m boundaries yield populated
    small/medium/large strata in all five checked-in worlds.
    """

    volume_m3 = math.prod(float(value) for value in truth.size_m)
    equivalent_edge_m = volume_m3 ** (1.0 / 3.0)
    if equivalent_edge_m < 0.30:
        return "small_cube_edge_lt_0_30m"
    if equivalent_edge_m < 0.75:
        return "medium_cube_edge_0_30_to_0_75m"
    return "large_cube_edge_ge_0_75m"


def match_objects_one_to_one(
    objects: Sequence[dict[str, Any]],
    truths: Sequence[ObjectGroundTruth],
    *,
    center_key: str = "pose",
) -> tuple[GroundTruthMatch, ...]:
    """Associate unique objects and targets by 3D position only.

    Label is deliberately excluded from association so a geometrically correct
    detection with the wrong class contributes to label error instead of recall
    loss.  The assignment maximizes match count, then minimizes total center
    distance; each object and target can appear in at most one match.
    """

    eligible_objects: list[
        tuple[int, dict[str, Any], tuple[float, float, float]]
    ] = []
    for object_index, obj in enumerate(objects):
        if bool(obj.get("missing")):
            continue
        if str(obj.get("cls") or "").strip().lower() == "robot":
            continue
        center = _object_center(obj, center_key)
        if center is None:
            continue
        eligible_objects.append((object_index, obj, center))

    truth_count = len(truths)
    object_count = len(eligible_objects)
    if truth_count == 0 or object_count == 0:
        return ()

    # Add a dummy row/column for every object/truth so unmatched items remain
    # legal even when the two real sets have equal size.  Valid edges have
    # normalized cost <= 1.  The unmatched penalty exceeds the sum of every
    # possible valid-edge cost, making the Hungarian solution lexicographic:
    # first maximize match count, then minimize normalized center distance.
    matrix_size = truth_count + object_count
    unmatched_cost = float(matrix_size + 1)
    invalid_cost = unmatched_cost * 4.0
    costs = [[0.0] * matrix_size for _ in range(matrix_size)]
    raw_distances: dict[tuple[int, int], float] = {}
    for truth_index, truth in enumerate(truths):
        for eligible_index, (_, _, center) in enumerate(eligible_objects):
            distance = math.dist(center, truth.center_m)
            raw_distances[(truth_index, eligible_index)] = distance
            costs[truth_index][eligible_index] = (
                distance / truth.association_radius_m
                if distance <= truth.association_radius_m
                else invalid_cost
            )
        for dummy_column in range(object_count, matrix_size):
            costs[truth_index][dummy_column] = unmatched_cost
    for dummy_row in range(truth_count, matrix_size):
        for eligible_index in range(object_count):
            costs[dummy_row][eligible_index] = unmatched_cost

    assignment = _minimum_cost_assignment(costs)
    matched: list[tuple[int, int, dict[str, Any], float]] = []
    for truth_index in range(truth_count):
        eligible_index = assignment[truth_index]
        if not 0 <= eligible_index < object_count:
            continue
        distance = raw_distances[(truth_index, eligible_index)]
        if distance > truths[truth_index].association_radius_m:
            continue
        object_index, obj, _ = eligible_objects[eligible_index]
        matched.append((truth_index, object_index, obj, distance))

    return tuple(
        GroundTruthMatch(
            truth_index=truth_index,
            object_index=object_index,
            truth=truths[truth_index],
            obj=obj,
            center_error_m=distance,
        )
        for truth_index, object_index, obj, distance in matched
    )


def _minimum_cost_assignment(costs: Sequence[Sequence[float]]) -> list[int]:
    """Return the minimum-cost column for each row (square Hungarian).

    The semantic inventory can contain more than one hundred assets, so the
    bit-mask dynamic program used by the small exact-geometry fixture is not
    appropriate.  This implementation is deterministic, dependency-free, and
    O(n³), which is modest for the five checked-in Webots worlds.
    """

    n = len(costs)
    if n == 0:
        return []
    if any(len(row) != n for row in costs):
        raise ValueError("assignment cost matrix must be square")
    u = [0.0] * (n + 1)
    v = [0.0] * (n + 1)
    p = [0] * (n + 1)
    way = [0] * (n + 1)
    for row in range(1, n + 1):
        p[0] = row
        column0 = 0
        minimum = [float("inf")] * (n + 1)
        used = [False] * (n + 1)
        while True:
            used[column0] = True
            row0 = p[column0]
            delta = float("inf")
            column1 = 0
            for column in range(1, n + 1):
                if used[column]:
                    continue
                current = costs[row0 - 1][column - 1] - u[row0] - v[column]
                if current < minimum[column]:
                    minimum[column] = current
                    way[column] = column0
                if minimum[column] < delta:
                    delta = minimum[column]
                    column1 = column
            for column in range(n + 1):
                if used[column]:
                    u[p[column]] += delta
                    v[column] -= delta
                else:
                    minimum[column] -= delta
            column0 = column1
            if p[column0] == 0:
                break
        while True:
            column1 = way[column0]
            p[column0] = p[column1]
            column0 = column1
            if column0 == 0:
                break
    assignment = [-1] * n
    for column in range(1, n + 1):
        if p[column]:
            assignment[p[column] - 1] = column - 1
    return assignment


def evaluate_semantic_inventory(
    objects: Sequence[dict[str, Any]],
    truths: Sequence[SemanticObjectGroundTruth],
    *,
    visible_truth_ids: set[str] | None = None,
    association_min_volume_ratio: float = 0.0,
    association_prefer_semantic_labels: bool = False,
    semantic_equivalence_groups: Sequence[Sequence[str]] = (),
    semantic_label_judgments: Mapping[tuple[str, str], bool] | None = None,
) -> dict[str, Any]:
    """Measure detection and classification, including explicit FP and FN.

    Map-plane radius and bbox-volume compatibility are hard admissibility
    gates.  When semantic preference is enabled, one-to-one association first
    maximizes the number of admissible detections, then the number of exact or
    explicitly equivalent labels, and finally minimizes normalized XY error.
    Optional externally reviewed judgments are data, not an in-code synonym
    table; callers are responsible for recording their judge and policy.
    A wrong label therefore still counts as a detection when it is the only
    admissible candidate, while a closer wrong class cannot steal a truth from
    a valid correct-class prediction.  Its independent 3D center error still
    exposes bad height/depth geometry.

    Unmatched predictions close to any world truth are duplicate false
    positives; predictions with no nearby world truth are ghost false
    positives.  If visibility evidence is supplied, recall and classification
    are scored only against independently visible targets.  A prediction
    matched to a real but non-visible world target is reported as
    ``ignored_real`` and excluded from the precision denominator instead of
    being mislabeled as a ghost.
    """

    all_truths = list(truths)
    scoped_truths = [
        truth
        for truth in all_truths
        if visible_truth_ids is None or truth.identity in visible_truth_ids
    ]
    scoped_truth_id_set = {truth.identity for truth in scoped_truths}
    predictions = [
        obj
        for obj in objects
        if not bool(obj.get("missing"))
        and str(obj.get("cls") or "").strip().lower() != "robot"
        and _object_center(obj, "pose") is not None
    ]
    if not 0.0 <= association_min_volume_ratio <= 1.0:
        raise ValueError("association_min_volume_ratio must be in [0, 1]")

    equivalence_by_label: dict[str, frozenset[str]] = {}
    reviewed_label_judgments = {
        (
            str(expected).strip().lower(),
            str(observed).strip().lower(),
        ): bool(equivalent)
        for (expected, observed), equivalent in (
            semantic_label_judgments or {}
        ).items()
    }
    for raw_group in semantic_equivalence_groups:
        group = frozenset(
            str(label).strip().lower()
            for label in raw_group
            if str(label).strip()
        )
        if len(group) < 2:
            raise ValueError(
                "semantic equivalence groups require at least two labels"
            )
        overlap = set(group) & set(equivalence_by_label)
        if overlap:
            raise ValueError(
                "semantic equivalence groups must not overlap: "
                + ", ".join(sorted(overlap))
            )
        for label in group:
            equivalence_by_label[label] = group

    def labels_equivalent(observed: str, expected: str) -> bool:
        if observed == expected:
            return True
        group = equivalence_by_label.get(expected)
        if group is not None and observed in group:
            return True
        return reviewed_label_judgments.get(
            (expected, observed),
            False,
        )

    def volume_compatible(
        obj: dict[str, Any],
        truth: SemanticObjectGroundTruth,
    ) -> bool:
        if association_min_volume_ratio <= 0.0:
            return True
        predicted_volume = _object_bbox_volume(obj)
        truth_volume = math.prod(truth.size_m)
        if predicted_volume is None or truth_volume <= 0.0:
            return True
        ratio = min(predicted_volume, truth_volume) / max(
            predicted_volume, truth_volume
        )
        return ratio >= association_min_volume_ratio

    truth_count = len(all_truths)
    visible_truth_count = len(scoped_truths)
    prediction_count = len(predictions)
    # Add one explicit dummy row per prediction and one explicit dummy column
    # per truth.  This permits either side to remain unmatched even when the
    # real bipartite sets happen to have equal size.
    matrix_size = truth_count + prediction_count
    max_match_count = min(truth_count, prediction_count)
    semantic_penalty = float(max_match_count + 1)
    unmatched_cost = float(
        (max_match_count + 1) * (semantic_penalty + 1.0)
    )
    invalid_cost = unmatched_cost * float(matrix_size + 1) * 4.0
    costs = [[invalid_cost] * matrix_size for _ in range(matrix_size)]
    raw_xy_distances: dict[tuple[int, int], float] = {}
    raw_center_distances: dict[tuple[int, int], float] = {}
    for truth_index, truth in enumerate(all_truths):
        for object_index, obj in enumerate(predictions):
            center = _object_center(obj, "pose")
            assert center is not None
            xy_distance = math.hypot(
                center[0] - truth.center_m[0],
                center[1] - truth.center_m[1],
            )
            raw_xy_distances[(truth_index, object_index)] = xy_distance
            raw_center_distances[(truth_index, object_index)] = math.dist(
                center, truth.center_m
            )
            if xy_distance <= truth.association_radius_m and volume_compatible(
                obj, truth
            ):
                observed = str(obj.get("cls") or "").strip().lower()
                semantic_cost = (
                    0.0
                    if (
                        not association_prefer_semantic_labels
                        or labels_equivalent(observed, truth.label)
                    )
                    else semantic_penalty
                )
                costs[truth_index][object_index] = (
                    semantic_cost + xy_distance / truth.association_radius_m
                )
        costs[truth_index][prediction_count + truth_index] = unmatched_cost
    for object_index in range(prediction_count):
        dummy_row = truth_count + object_index
        costs[dummy_row][object_index] = unmatched_cost
        for dummy_column in range(prediction_count, matrix_size):
            costs[dummy_row][dummy_column] = 0.0

    assignment = _minimum_cost_assignment(costs)
    matches: list[tuple[int, int, float]] = []
    matched_truths: set[int] = set()
    matched_objects: set[int] = set()
    for truth_index in range(truth_count):
        object_index = assignment[truth_index] if assignment else -1
        if not (0 <= object_index < prediction_count):
            continue
        xy_distance = raw_xy_distances[(truth_index, object_index)]
        if xy_distance > all_truths[
            truth_index
        ].association_radius_m or not volume_compatible(
            predictions[object_index],
            all_truths[truth_index],
        ):
            continue
        matched_truths.add(truth_index)
        matched_objects.add(object_index)
        matches.append((truth_index, object_index, xy_distance))

    visible_matches = [
        match
        for match in matches
        if all_truths[match[0]].identity in scoped_truth_id_set
    ]
    ignored_matches = [
        match
        for match in matches
        if all_truths[match[0]].identity not in scoped_truth_id_set
    ]
    visible_match_by_identity = {
        all_truths[truth_index].identity: (truth_index, object_index, distance)
        for truth_index, object_index, distance in visible_matches
    }

    per_target: list[dict[str, Any]] = []
    label_correct_count = 0
    center_errors: list[float] = []
    center_xy_errors: list[float] = []
    confusion: dict[str, dict[str, int]] = {}
    for truth in scoped_truths:
        matched = visible_match_by_identity.get(truth.identity)
        entry: dict[str, Any] = {
            "identity": truth.identity,
            "node_type": truth.node_type,
            "name": truth.name,
            "expected_label": truth.label,
            "max_dimension_m": max(float(value) for value in truth.size_m),
            "size_bucket": _semantic_size_bucket(truth),
            "matched": matched is not None,
        }
        if matched is not None:
            truth_index, object_index, xy_distance = matched
            center_distance = raw_center_distances[(truth_index, object_index)]
            obj = predictions[object_index]
            observed = str(obj.get("cls") or "").strip().lower()
            correct = labels_equivalent(observed, truth.label)
            label_correct_count += int(correct)
            center_errors.append(center_distance)
            center_xy_errors.append(xy_distance)
            confusion.setdefault(truth.label, {})
            confusion[truth.label][observed] = (
                confusion[truth.label].get(observed, 0) + 1
            )
            entry.update(
                {
                    "object_id": str(obj.get("id") or ""),
                    "observed_label": observed,
                    "label_correct": correct,
                    "center_xy_error_m": round(xy_distance, 6),
                    "center_error_m": round(center_distance, 6),
                }
            )
        per_target.append(entry)

    duplicate_predictions: list[dict[str, Any]] = []
    ghost_predictions: list[dict[str, Any]] = []
    ignored_real_predictions: list[dict[str, Any]] = []
    for truth_index, object_index, xy_distance in ignored_matches:
        truth = all_truths[truth_index]
        obj = predictions[object_index]
        observed = str(obj.get("cls") or "").strip().lower()
        center_distance = raw_center_distances[(truth_index, object_index)]
        ignored_real_predictions.append(
            {
                "object_id": str(obj.get("id") or ""),
                "observed_label": observed,
                "nearest_truth": truth.identity,
                "nearest_truth_label": truth.label,
                "label_correct": labels_equivalent(observed, truth.label),
                "center_xy_error_m": round(xy_distance, 6),
                "center_error_m": round(center_distance, 6),
            }
        )
    for object_index, obj in enumerate(predictions):
        if object_index in matched_objects:
            continue
        center = _object_center(obj, "pose")
        assert center is not None
        nearby = [
            (
                math.hypot(
                    center[0] - truth.center_m[0],
                    center[1] - truth.center_m[1],
                ),
                truth,
            )
            for truth in all_truths
            if (
                math.hypot(
                    center[0] - truth.center_m[0],
                    center[1] - truth.center_m[1],
                )
                <= truth.association_radius_m
                and volume_compatible(obj, truth)
            )
        ]
        payload = {
            "object_id": str(obj.get("id") or ""),
            "observed_label": str(obj.get("cls") or "").strip().lower(),
            "center_m": [round(value, 6) for value in center],
        }
        if nearby:
            xy_distance, truth = min(nearby, key=lambda item: item[0])
            payload.update(
                {
                    "nearest_truth": truth.identity,
                    "nearest_truth_label": truth.label,
                    "center_xy_error_m": round(xy_distance, 6),
                    "center_error_m": round(math.dist(center, truth.center_m), 6),
                }
            )
            duplicate_predictions.append(payload)
        else:
            if all_truths:
                xy_distance, truth = min(
                    (
                        (
                            math.hypot(
                                center[0] - truth.center_m[0],
                                center[1] - truth.center_m[1],
                            ),
                            truth,
                        )
                        for truth in all_truths
                    ),
                    key=lambda item: (item[0], item[1].identity),
                )
                payload.update(
                    {
                        "nearest_truth": truth.identity,
                        "nearest_truth_label": truth.label,
                        "center_xy_error_m": round(xy_distance, 6),
                        "center_error_m": round(math.dist(center, truth.center_m), 6),
                    }
                )
            ghost_predictions.append(payload)

    tp = len(visible_matches)
    fn = visible_truth_count - tp
    fp = len(duplicate_predictions) + len(ghost_predictions)
    evaluated_prediction_count = tp + fp
    precision = tp / evaluated_prediction_count if evaluated_prediction_count else 0.0
    recall = tp / visible_truth_count if visible_truth_count else 0.0
    f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0
    label_accuracy = label_correct_count / tp if tp else 0.0
    class_labels = sorted({truth.label for truth in scoped_truths})
    per_class: dict[str, Any] = {}
    for label in class_labels:
        expected = sum(truth.label == label for truth in scoped_truths)
        matched_for_class = [
            entry
            for entry in per_target
            if entry["expected_label"] == label and entry["matched"]
        ]
        correct = sum(bool(entry.get("label_correct")) for entry in matched_for_class)
        per_class[label] = {
            "visible_truth_count": expected,
            "detected_count": len(matched_for_class),
            "correct_label_count": correct,
            "miss_count": expected - len(matched_for_class),
            "detection_recall": len(matched_for_class) / expected if expected else 0.0,
            "classification_accuracy": (
                correct / len(matched_for_class) if matched_for_class else 0.0
            ),
        }
    macro_recall = (
        statistics.mean(item["detection_recall"] for item in per_class.values())
        if per_class
        else 0.0
    )
    macro_classification_accuracy = (
        statistics.mean(item["classification_accuracy"] for item in per_class.values())
        if per_class
        else 0.0
    )
    per_size_bucket: dict[str, Any] = {}
    bucket_rules = {
        "small_cube_edge_lt_0_30m": "<0.30m",
        "medium_cube_edge_0_30_to_0_75m": "[0.30m, 0.75m)",
        "large_cube_edge_ge_0_75m": ">=0.75m",
    }
    for bucket, rule in bucket_rules.items():
        total_truth_count = sum(
            _semantic_size_bucket(truth) == bucket for truth in all_truths
        )
        covered_entries = [
            entry for entry in per_target if entry["size_bucket"] == bucket
        ]
        covered_truth_count = len(covered_entries)
        matched_entries = [
            entry for entry in covered_entries if entry["matched"]
        ]
        matched_count = len(matched_entries)
        correct_label_count = sum(
            bool(entry.get("label_correct")) for entry in matched_entries
        )
        per_size_bucket[bucket] = {
            "equivalent_cube_edge_rule": rule,
            "total_truth_count": total_truth_count,
            "covered_truth_count": covered_truth_count,
            "coverage": (
                covered_truth_count / total_truth_count
                if total_truth_count
                else 0.0
            ),
            "matched_count": matched_count,
            "correct_label_count": correct_label_count,
            "recall_at_covered": (
                matched_count / covered_truth_count
                if covered_truth_count
                else 0.0
            ),
            "label_accuracy_among_matched": (
                correct_label_count / matched_count if matched_count else 0.0
            ),
        }
    return {
        "truth_scope": "visible" if visible_truth_ids is not None else "all",
        "truth_count": truth_count,
        "visible_truth_count": visible_truth_count,
        "coverage": visible_truth_count / truth_count if truth_count else 0.0,
        "prediction_count": prediction_count,
        "evaluated_prediction_count": evaluated_prediction_count,
        "ignored_real_prediction_count": len(ignored_real_predictions),
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "recall": recall,
        "recall_at_covered": recall,
        "f1": f1,
        "label_accuracy": label_accuracy,
        "macro_detection_recall": macro_recall,
        "macro_classification_accuracy": macro_classification_accuracy,
        "median_center_xy_error_m": _median(center_xy_errors),
        "median_center_error_m": _median(center_errors),
        "duplicate_fp_count": len(duplicate_predictions),
        "ghost_fp_count": len(ghost_predictions),
        "duplicates": duplicate_predictions,
        "ghosts": ghost_predictions,
        "ignored_real_predictions": ignored_real_predictions,
        "confusion": confusion,
        "per_class": per_class,
        "per_size_bucket": per_size_bucket,
        "per_target": per_target,
    }


def _rectangle(
    center_x: float,
    center_y: float,
    size_x: float,
    size_y: float,
    yaw: float,
) -> list[tuple[float, float]]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    half_x, half_y = size_x * 0.5, size_y * 0.5
    return [
        (
            center_x + cosine * x - sine * y,
            center_y + sine * x + cosine * y,
        )
        for x, y in (
            (-half_x, -half_y),
            (half_x, -half_y),
            (half_x, half_y),
            (-half_x, half_y),
        )
    ]


def _polygon_area(polygon: Sequence[tuple[float, float]]) -> float:
    if len(polygon) < 3:
        return 0.0
    return (
        abs(
            sum(
                x0 * y1 - x1 * y0
                for (x0, y0), (x1, y1) in zip(
                    polygon,
                    [*polygon[1:], polygon[0]],
                )
            )
        )
        * 0.5
    )


def _clip_polygon(
    subject: Sequence[tuple[float, float]],
    clip: Sequence[tuple[float, float]],
) -> list[tuple[float, float]]:
    """Sutherland-Hodgman clipping for two counter-clockwise rectangles."""

    output = list(subject)
    for edge_start, edge_end in zip(clip, [*clip[1:], clip[0]]):
        input_points = output
        output = []
        if not input_points:
            break

        def inside(point: tuple[float, float]) -> bool:
            return (edge_end[0] - edge_start[0]) * (point[1] - edge_start[1]) - (
                edge_end[1] - edge_start[1]
            ) * (point[0] - edge_start[0]) >= -1e-12

        def intersection(
            first: tuple[float, float],
            second: tuple[float, float],
        ) -> tuple[float, float]:
            dx1, dy1 = second[0] - first[0], second[1] - first[1]
            dx2, dy2 = edge_end[0] - edge_start[0], edge_end[1] - edge_start[1]
            denominator = dx1 * dy2 - dy1 * dx2
            if abs(denominator) <= 1e-12:
                return second
            t = (
                (edge_start[0] - first[0]) * dy2 - (edge_start[1] - first[1]) * dx2
            ) / denominator
            return first[0] + t * dx1, first[1] + t * dy1

        previous = input_points[-1]
        previous_inside = inside(previous)
        for current in input_points:
            current_inside = inside(current)
            if current_inside:
                if not previous_inside:
                    output.append(intersection(previous, current))
                output.append(current)
            elif previous_inside:
                output.append(intersection(previous, current))
            previous = current
            previous_inside = current_inside
    return output


def bbox_iou_3d(obj: dict[str, Any], truth: ObjectGroundTruth) -> float:
    """Return yaw-oriented 3D IoU between a Scene box and simulator truth."""

    pose = obj.get("pose") or {}
    bbox = obj.get("bbox") or {}
    try:
        center = (float(pose["x"]), float(pose["y"]), float(pose["z"]))
        size = (
            float(bbox["size_x"]),
            float(bbox["size_y"]),
            float(bbox["size_z"]),
        )
        yaw = float(bbox.get("yaw") or pose.get("yaw") or 0.0)
    except (KeyError, TypeError, ValueError):
        return 0.0
    if any(not math.isfinite(value) or value <= 0.0 for value in size):
        return 0.0
    predicted_xy = _rectangle(center[0], center[1], size[0], size[1], yaw)
    truth_xy = _rectangle(
        truth.center_m[0],
        truth.center_m[1],
        truth.size_m[0],
        truth.size_m[1],
        truth.yaw_rad,
    )
    area = _polygon_area(_clip_polygon(predicted_xy, truth_xy))
    predicted_low = center[2] - size[2] * 0.5
    predicted_high = center[2] + size[2] * 0.5
    truth_low = truth.center_m[2] - truth.size_m[2] * 0.5
    truth_high = truth.center_m[2] + truth.size_m[2] * 0.5
    overlap_z = max(
        0.0, min(predicted_high, truth_high) - max(predicted_low, truth_low)
    )
    intersection = area * overlap_z
    predicted_volume = size[0] * size[1] * size[2]
    truth_volume = truth.size_m[0] * truth.size_m[1] * truth.size_m[2]
    union = predicted_volume + truth_volume - intersection
    return intersection / union if union > 0.0 else 0.0


def point_inlier_fraction(
    points: Sequence[Sequence[float]],
    truth: ObjectGroundTruth,
    *,
    margin_m: float,
) -> float | None:
    """Fraction of exported points inside the fixture box plus SI margin."""

    valid = [
        (float(point[0]), float(point[1]), float(point[2]))
        for point in points
        if len(point) >= 3 and all(math.isfinite(float(value)) for value in point[:3])
    ]
    if not valid:
        return None
    cosine = math.cos(-truth.yaw_rad)
    sine = math.sin(-truth.yaw_rad)
    half = tuple(value * 0.5 + margin_m for value in truth.size_m)
    inliers = 0
    for x, y, z in valid:
        dx, dy, dz = (
            x - truth.center_m[0],
            y - truth.center_m[1],
            z - truth.center_m[2],
        )
        local_x = cosine * dx - sine * dy
        local_y = sine * dx + cosine * dy
        if abs(local_x) <= half[0] and abs(local_y) <= half[1] and abs(dz) <= half[2]:
            inliers += 1
    return inliers / len(valid)


def _median(values: Sequence[float]) -> float | None:
    return float(statistics.median(values)) if values else None


def _maximum_pairwise_xy(samples: Sequence[dict[str, Any]]) -> float | None:
    centers = [
        (float(sample["pose"]["x"]), float(sample["pose"]["y"])) for sample in samples
    ]
    if not centers:
        return None
    return max(
        math.hypot(first[0] - second[0], first[1] - second[1])
        for first in centers
        for second in centers
    )


def evaluate_ground_truth(
    samples: Sequence[dict[str, Any]],
    point_samples: Sequence[dict[str, Any]],
    truth: ObjectGroundTruth,
    *,
    expected_samples: int,
    thresholds: dict[str, Any],
) -> dict[str, Any]:
    """Summarise and gate observations without hiding missing measurements."""

    xy_errors = [
        math.hypot(
            float(sample["pose"]["x"]) - truth.center_m[0],
            float(sample["pose"]["y"]) - truth.center_m[1],
        )
        for sample in samples
    ]
    z_errors = [
        abs(float(sample["pose"]["z"]) - truth.center_m[2]) for sample in samples
    ]
    ious = [bbox_iou_3d(sample, truth) for sample in samples]
    labels = [str(sample.get("cls") or "").strip().lower() for sample in samples]
    ids = {str(sample.get("id") or "") for sample in samples if sample.get("id")}
    navigation_grade_fraction = (
        sum(bool(sample.get("navigation_grade")) for sample in samples) / len(samples)
        if samples
        else 0.0
    )
    inlier_values = [
        value
        for sample in point_samples
        if (
            value := point_inlier_fraction(
                sample.get("points") or (),
                truth,
                margin_m=float(thresholds["point_margin_m"]),
            )
        )
        is not None
    ]
    point_counts = [
        int(sample.get("n_points") or len(sample.get("points") or ()))
        for sample in point_samples
    ]
    target_recall = len(samples) / max(1, expected_samples)
    label_accuracy = (
        sum(label == truth.label for label in labels) / len(labels) if labels else 0.0
    )
    metrics = {
        "sample_count": len(samples),
        "expected_samples": expected_samples,
        "target_recall": target_recall,
        "label_accuracy": label_accuracy,
        "stable_id_count": len(ids),
        "median_center_xy_error_m": _median(xy_errors),
        "max_center_xy_drift_m": _maximum_pairwise_xy(samples),
        "median_center_z_error_m": _median(z_errors),
        "median_bbox_iou_3d": _median(ious),
        "median_point_inlier_fraction": _median(inlier_values),
        "median_point_count": _median([float(value) for value in point_counts]),
        "navigation_grade_fraction": navigation_grade_fraction,
    }
    failures: list[str] = []

    def require_at_least(metric: str, threshold: str) -> None:
        value = metrics[metric]
        if value is None or float(value) < float(thresholds[threshold]):
            failures.append(f"{metric} below {thresholds[threshold]}")

    def require_at_most(metric: str, threshold: str) -> None:
        value = metrics[metric]
        if value is None or float(value) > float(thresholds[threshold]):
            failures.append(f"{metric} above {thresholds[threshold]}")

    require_at_least("target_recall", "min_target_recall")
    require_at_least("label_accuracy", "min_label_accuracy")
    if not ids or len(ids) > int(thresholds["max_stable_ids"]):
        failures.append(f"stable_id_count above {thresholds['max_stable_ids']}")
    require_at_most(
        "median_center_xy_error_m",
        "max_median_center_xy_error_m",
    )
    require_at_most("max_center_xy_drift_m", "max_center_xy_drift_m")
    require_at_most(
        "median_center_z_error_m",
        "max_median_center_z_error_m",
    )
    require_at_least("median_bbox_iou_3d", "min_median_bbox_iou_3d")
    require_at_least(
        "median_point_inlier_fraction",
        "min_median_point_inlier_fraction",
    )
    require_at_least("median_point_count", "min_median_point_count")
    require_at_least(
        "navigation_grade_fraction",
        "min_navigation_grade_fraction",
    )
    return {
        "ok": not failures,
        "failures": failures,
        "ground_truth": {
            "label": truth.label,
            "center_m": list(truth.center_m),
            "bbox_size_m": list(truth.size_m),
            "yaw_rad": truth.yaw_rad,
        },
        **metrics,
    }


def _mean(values: Sequence[float]) -> float | None:
    return float(statistics.fmean(values)) if values else None


def _size_and_yaw_errors(
    match: GroundTruthMatch,
) -> tuple[float | None, float | None, float | None]:
    pose = match.obj.get("pose") or {}
    bbox = match.obj.get("bbox") or {}
    try:
        predicted_size = (
            float(bbox["size_x"]),
            float(bbox["size_y"]),
            float(bbox["size_z"]),
        )
    except (KeyError, TypeError, ValueError):
        predicted_size = None
    if predicted_size is not None and not all(
        math.isfinite(value) and value > 0.0 for value in predicted_size
    ):
        predicted_size = None

    try:
        raw_yaw = bbox["yaw"] if "yaw" in bbox else pose["yaw"]
        predicted_yaw = float(raw_yaw)
        if not math.isfinite(predicted_yaw):
            predicted_yaw = None
    except (KeyError, TypeError, ValueError):
        predicted_yaw = None

    def errors_for(
        size: tuple[float, float, float],
        yaw: float | None,
    ) -> tuple[float, float, float | None]:
        deltas = tuple(
            predicted - expected
            for predicted, expected in zip(size, match.truth.size_m)
        )
        size_error = math.sqrt(sum(delta * delta for delta in deltas))
        relative_size_error = (
            sum(
                abs(delta) / expected
                for delta, expected in zip(deltas, match.truth.size_m)
            )
            / 3.0
        )
        if yaw is None:
            yaw_error = None
        else:
            # A geometric oriented box has pi-periodic yaw.
            delta = _normalise_angle(yaw - match.truth.yaw_rad)
            yaw_error = abs(0.5 * math.atan2(math.sin(2 * delta), math.cos(2 * delta)))
        return size_error, relative_size_error, yaw_error

    if predicted_size is None:
        if predicted_yaw is None or not match.truth.evaluate_yaw:
            return None, None, None
        _, _, yaw_error = errors_for(match.truth.size_m, predicted_yaw)
        return None, None, yaw_error

    evaluated_yaw = predicted_yaw if match.truth.evaluate_yaw else None
    direct = errors_for(predicted_size, evaluated_yaw)
    # (sx, sy, yaw) and (sy, sx, yaw - pi/2) describe the same XY box.
    swapped = errors_for(
        (predicted_size[1], predicted_size[0], predicted_size[2]),
        evaluated_yaw - math.pi * 0.5 if evaluated_yaw is not None else None,
    )

    def representation_cost(
        errors: tuple[float, float, float | None],
    ) -> tuple[float, float]:
        yaw_cost = errors[2] / (math.pi * 0.5) if errors[2] is not None else 0.0
        return errors[1] + yaw_cost, errors[0]

    return min((direct, swapped), key=representation_cost)


def evaluate_scene_ground_truth(
    objects: Sequence[dict[str, Any]],
    truths: Sequence[ObjectGroundTruth],
    *,
    thresholds: dict[str, Any] | None = None,
    center_key: str = "pose",
) -> dict[str, Any]:
    """Evaluate one Scene snapshot against several WBT-derived targets."""

    if not truths:
        raise ValueError("at least one ground-truth target is required")
    thresholds = thresholds or {}
    matches = match_objects_one_to_one(objects, truths, center_key=center_key)
    visible_indices = {
        index
        for index, obj in enumerate(objects)
        if not bool(obj.get("missing"))
        and str(obj.get("cls") or "").strip().lower() != "robot"
    }
    matched_indices = {match.object_index for match in matches}
    duplicate_indices: set[int] = set()
    for object_index in visible_indices - matched_indices:
        center = _object_center(objects[object_index], center_key)
        if center is not None and any(
            math.dist(center, truth.center_m) <= truth.association_radius_m
            for truth in truths
        ):
            duplicate_indices.add(object_index)

    center_errors = [match.center_error_m for match in matches]
    labels_correct = [
        str(match.obj.get("cls") or "").strip().lower() == match.truth.label
        for match in matches
    ]
    geometry_errors = [_size_and_yaw_errors(match) for match in matches]
    size_errors = [value[0] for value in geometry_errors if value[0] is not None]
    relative_size_errors = [
        value[1] for value in geometry_errors if value[1] is not None
    ]
    yaw_errors = [value[2] for value in geometry_errors if value[2] is not None]

    metrics: dict[str, Any] = {
        "object_count": len(visible_indices),
        "expected_object_count": len(truths),
        "matched_object_count": len(matches),
        "target_recall": len(matches) / len(truths),
        "duplicate_count": len(duplicate_indices),
        "unmatched_object_count": len(visible_indices - matched_indices),
        "ghost_count": len(visible_indices - matched_indices - duplicate_indices),
        "label_accuracy": (
            sum(labels_correct) / len(labels_correct) if labels_correct else 0.0
        ),
        "geometry_sample_count": len(size_errors),
        "yaw_sample_count": len(yaw_errors),
        "mean_center_error_m": _mean(center_errors),
        "median_center_error_m": _median(center_errors),
        "mean_size_error_m": _mean(size_errors),
        "median_size_error_m": _median(size_errors),
        "mean_size_relative_error": _mean(relative_size_errors),
        "median_size_relative_error": _median(relative_size_errors),
        "mean_yaw_error_rad": _mean(yaw_errors),
        "median_yaw_error_rad": _median(yaw_errors),
    }
    failures: list[str] = []

    def require_at_least(metric: str, threshold: str) -> None:
        if threshold not in thresholds:
            return
        value = metrics[metric]
        if value is None or float(value) < float(thresholds[threshold]):
            failures.append(f"{metric} below {thresholds[threshold]}")

    def require_at_most(metric: str, threshold: str) -> None:
        if threshold not in thresholds:
            return
        value = metrics[metric]
        if value is None or float(value) > float(thresholds[threshold]):
            failures.append(f"{metric} above {thresholds[threshold]}")

    require_at_least("target_recall", "min_target_recall")
    require_at_least("label_accuracy", "min_label_accuracy")
    require_at_most("duplicate_count", "max_duplicate_count")
    require_at_most("median_center_error_m", "max_median_center_error_m")
    require_at_most("median_size_error_m", "max_median_size_error_m")
    require_at_most(
        "median_size_relative_error",
        "max_median_size_relative_error",
    )
    require_at_most("median_yaw_error_rad", "max_median_yaw_error_rad")

    per_target = []
    by_truth = {match.truth_index: match for match in matches}
    for truth_index, truth in enumerate(truths):
        match = by_truth.get(truth_index)
        if match is None:
            per_target.append(
                {
                    "node_type": truth.node_type,
                    "name": truth.name,
                    "expected_label": truth.label,
                    "yaw_evaluated": truth.evaluate_yaw,
                    "matched": False,
                }
            )
            continue
        size_error, relative_size_error, yaw_error = _size_and_yaw_errors(match)
        per_target.append(
            {
                "node_type": truth.node_type,
                "name": truth.name,
                "expected_label": truth.label,
                "yaw_evaluated": truth.evaluate_yaw,
                "matched": True,
                "object_id": str(match.obj.get("id") or ""),
                "observed_label": str(match.obj.get("cls") or "").strip().lower(),
                "label_correct": (
                    str(match.obj.get("cls") or "").strip().lower() == truth.label
                ),
                "center_error_m": match.center_error_m,
                "size_error_m": size_error,
                "size_relative_error": relative_size_error,
                "yaw_error_rad": yaw_error,
            }
        )

    return {
        "ok": not failures,
        "failures": failures,
        "per_target": per_target,
        **metrics,
    }


# Descriptive alias for callers that use the inverse word order.
evaluate_multi_object_ground_truth = evaluate_scene_ground_truth
