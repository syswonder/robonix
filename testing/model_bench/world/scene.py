# SPDX-License-Identifier: MulanPSL-2.0
"""The office floor in fixtures/scene.yaml, answered in Scene's shapes
(system/scene/scene_service/mcp_tools.py, find.py)."""

from __future__ import annotations

import math
import re
from pathlib import Path

import yaml

SCENE = yaml.safe_load((Path(__file__).resolve().parents[1] / "fixtures/scene.yaml").read_text())
OBJECTS = {o["id"]: o for o in SCENE["objects"]}
REGIONS = {r["id"]: r for r in SCENE["regions"]}
WORD = re.compile(r"[^0-9a-z\u4e00-\u9fff]+")  # letters, digits and CJK ideographs


def region_of(x: float, y: float) -> str:
    for r in SCENE["regions"]:
        xs, ys = [p[0] for p in r["points"]], [p[1] for p in r["points"]]
        if min(xs) <= x < max(xs) and min(ys) <= y < max(ys):
            return r["name"]
    return ""


def obj(o: dict) -> dict:
    """mcp_tools._to_idl."""
    return {"id": o["id"], "label": o["label"], "x": o["x"], "y": o["y"], "z": o["z"], "yaw": 0.0,
            "size_x": 0.5, "size_y": 0.5, "size_z": 0.5, "frame_id": "map",
            "observation_count": 12, "last_seen_unix": 1791952190.0}


def region(r: dict) -> dict:
    return {"id": r["id"], "kind": "region", "name": r["name"], "points_xy": [c for p in r["points"] for c in p],
            "theta": 0.0, "stale": False, "stale_reason": "", "updated_at_unix": 1791900000.0}


def edges(relation: str = "") -> list[dict]:
    return [dict(e, confidence=0.9, reason="") for e in SCENE["relations"] if not relation or e["relation"] == relation]


def list_objects() -> dict:
    regions = [{"id": r["id"], "label": r["name"], "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "size_x": 0.0, "size_y": 0.0,
                "size_z": 0.0, "frame_id": "", "observation_count": 0, "last_seen_unix": 0.0} for r in SCENE["regions"]]
    return {"objects": [obj(o) for o in SCENE["objects"]] + regions, "stamp_unix": 1791952192.0,
            "map_id": ".live", "generation": 1, "generation_supported": True}


def find(label: str = "", region: str = "", relation: str = "", anchor: str = "", **_) -> dict:
    """find.py: each constraint filters; survivors come nearest the robot first."""
    pool = list(SCENE["objects"])
    words = set(WORD.split(label.lower())) - {""}
    if words:
        pool = [o for o in pool if words & set(WORD.split(f"{o['label']} {o['caption']}".lower()))]
        if not pool:
            return {"objects": [], "detail": f"no object matches {label!r}"}
    if region:
        pool = [o for o in pool if region_of(o["x"], o["y"]).lower() == region.lower()]
        if not pool:
            return {"objects": [], "detail": f"none of them is in region {region!r}"}
    if relation:
        pool = [o for o in pool if any(e["source_id"] == o["id"] and e["relation"] == relation and e["target_id"] == anchor
                                       for e in SCENE["relations"])]
        if not pool:
            return {"objects": [], "detail": f"none of them is {relation} {anchor!r}"}
    pool.sort(key=lambda o: math.hypot(o["x"], o["y"]))
    return {"objects": [obj(o) for o in pool], "detail": ""}


def approach(x: float, y: float, standoff: float = 0.6) -> dict:
    """A pose `standoff` metres short of (x, y) on the way from the robot, facing it."""
    d = math.hypot(x, y) or 1.0
    yaw = math.atan2(y, x)
    return {"x": round(x - standoff * x / d, 3), "y": round(y - standoff * y / d, 3), "yaw": round(yaw, 3)}


def goal_near(object_id: str, **_) -> dict:
    o = OBJECTS.get(object_id)
    if o is None:
        raise ValueError(f"unknown object_id {object_id!r}")
    return {"reachable": True, **approach(o["x"], o["y"]), "reason": ""}


def goal_region(region_id: str, **_) -> dict:
    r = REGIONS.get(region_id)
    if r is None:
        raise ValueError(f"unknown region_id {region_id!r}; available regions: "
                         + ", ".join(f"{x['name']!r} (id={x['id']})" for x in SCENE["regions"]))
    xs, ys = [p[0] for p in r["points"]], [p[1] for p in r["points"]]
    return {"reachable": True, "x": sum(xs) / len(xs), "y": sum(ys) / len(ys), "yaw": 0.0, "reason": ""}


def object_context(object_id: str, **_) -> dict:
    o = OBJECTS.get(object_id)
    if o is None:
        raise ValueError(f"unknown object_id {object_id!r}")
    rel = [e for e in edges() if object_id in (e["source_id"], e["target_id"])]
    return {"object": obj(o), "region": region_of(o["x"], o["y"]), "relations": rel}


def robot_context() -> dict:
    near = sorted(SCENE["objects"], key=lambda o: math.hypot(o["x"], o["y"]))[:5]
    return {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame_id": "map", "region_id": "scene.region.r1", "region_name": REGIONS["scene.region.r1"]["name"],
            "nearby_objects": [obj(o) for o in near]}


def scene_graph() -> dict:
    return {"objects": [obj(o) for o in SCENE["objects"]], "edges": edges(), "regions": [region(r) for r in SCENE["regions"]]}
