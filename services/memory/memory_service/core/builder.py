"""Memory Builder — structured memory data construction, YAML load/store, batch import.

Supports:
  1. Programmatic construction via helper functions
  2. YAML file loading (structured format)
  3. Semi-structured log file parsing (Robonix disk log format, per Scribe §6)
  4. Batch import into MemoryService
"""

from __future__ import annotations

import json
import logging
import re
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from .types import (
    LogRecord, SpatialContext, ObjectCoord, TagSet, MemoryNode, NodeType,
    RememberRequest,
)

log = logging.getLogger("scribe_mem")


# ── Structured memory constructors ─────────────────────────────────────

def make_log_record(ts: Optional[int] = None, level: str = "Info",
                    tag: str = "", msg: str = "") -> LogRecord:
    """Create a LogRecord with sensible defaults."""
    if ts is None:
        ts = time.time_ns()
    return LogRecord(ts=ts, level=level, tag=tag, msg=msg)


def make_spatial(objects: List[Tuple[str, str, float, float, float]],
                 origin: str = "") -> SpatialContext:
    """Create SpatialContext from a list of (obj_id, label, x, y, z) tuples."""
    return SpatialContext(
        objects=[ObjectCoord(obj_id=oid, label=label, x=x, y=y, z=z)
                 for oid, label, x, y, z in objects],
        origin=origin,
    )


def make_tags(scene_type: str = "", action_type: str = "",
              task_type: str = "", success: bool = True,
              objects: Optional[List[str]] = None,
              difficulty: str = "medium",
              tool: str = "") -> TagSet:
    """Create a TagSet with the most commonly used fields."""
    return TagSet(
        scene_type=scene_type,
        action_type=action_type,
        task_type=task_type,
        success=success,
        objects_present=objects or [],
        difficulty=difficulty,
        tool_used=[tool] if tool else [],
    )


def make_memory_node(node_id: int, summary: str,
                     log_record: Optional[LogRecord] = None,
                     spatial: Optional[SpatialContext] = None,
                     tags: Optional[TagSet] = None,
                     timestamp: Optional[int] = None,
                     causal_chain: Optional[List[int]] = None,
                     weight: float = 0.5,
                     node_type: NodeType = NodeType.SHORT_TERM) -> MemoryNode:
    """Create a fully-formed MemoryNode."""
    ts = timestamp or (log_record.ts if log_record else time.time_ns())
    return MemoryNode(
        node_id=node_id,
        summary=summary,
        raw_log=log_record,
        timestamp=ts,
        spatial_data=spatial,
        tags=tags,
        causal_chain=list(causal_chain) if causal_chain else [],
        weight=weight,
        node_type=node_type,
        created_at=ts,
        version=1,
    )


# ── YAML loader ─────────────────────────────────────────────────────────

def load_memories_from_yaml(path: str) -> List[Dict[str, Any]]:
    """Load a YAML file of memory records.

    Expected YAML shape:
        memories:
          - session_id: "sess-1"
            plan_id: "plan-1"
            level: Info
            tag: exec
            msg: "grasped red cup in the kitchen"
            scene_type: kitchen
            action_type: grasp
            task_type: fetch
            success: true
            difficulty: easy
            objects:
              - [scene.obj.cup_001, "red cup", 1.0, 2.0, 0.8]
            parent: null
          - ...
    """
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    if not data or "memories" not in data:
        raise ValueError(f"YAML file {path} missing top-level 'memories' key")
    return data["memories"]


async def import_yaml_to_service(service, yaml_path: str,
                                  clear_existing: bool = False) -> List[int]:
    """Load YAML memory records and import them into a MemoryService.

    Args:
        service: MemoryService instance.
        yaml_path: Path to the YAML file.
        clear_existing: If True, clear existing data before import.

    Returns:
        List of created node_ids.
    """
    records = load_memories_from_yaml(yaml_path)

    if clear_existing:
        # Remove all existing nodes and rebuild empty indices.
        for nid in list(service.graph.all_ids()):
            service.graph.remove_node(nid)
        service.tags.rebuild([])
        service.vectors.rebuild([])
        log.info("import_yaml: cleared existing data before import")

    node_ids = []
    for rec in records:
        nid = await _import_one_record(service, rec)
        node_ids.append(nid)

    log.info("import_yaml: %d records loaded from %s", len(node_ids), yaml_path)
    return node_ids


async def _import_one_record(service, rec: Dict[str, Any]) -> int:
    """Import a single YAML record into the service."""
    # Build LogRecord
    lr = LogRecord(
        ts=rec.get("ts", time.time_ns()),
        level=rec.get("level", "Info"),
        tag=rec.get("tag", ""),
        msg=rec.get("msg", ""),
    )

    # Build SpatialContext from objects list
    spatial = None
    raw_objects = rec.get("objects", [])
    if raw_objects:
        spatial = SpatialContext(
            objects=[_parse_object_entry(o) for o in raw_objects],
            origin=rec.get("spatial_origin", ""),
        )

    # Determine parent
    parent = rec.get("parent")
    kv = rec.get("kv", {})

    resp = await service.remember(
        session_id=rec.get("session_id", "imported"),
        plan_id=rec.get("plan_id", "imported"),
        log_record=lr,
        spatial=spatial,
        parent_node_id=parent,
        kv=kv,
    )
    return resp.node_id


def _parse_object_entry(entry) -> ObjectCoord:
    """Parse an object entry from YAML.

    Supports:
      - [obj_id, label, x, y, z]  (list)
      - {obj_id: ..., label: ..., x: ..., y: ..., z: ...}  (dict)
    """
    if isinstance(entry, list):
        return ObjectCoord(
            obj_id=str(entry[0]),
            label=str(entry[1]) if len(entry) > 1 else "",
            x=float(entry[2]) if len(entry) > 2 else 0.0,
            y=float(entry[3]) if len(entry) > 3 else 0.0,
            z=float(entry[4]) if len(entry) > 4 else 0.0,
        )
    elif isinstance(entry, dict):
        return ObjectCoord(
            obj_id=str(entry.get("obj_id", "")),
            label=str(entry.get("label", "")),
            x=float(entry.get("x", 0)),
            y=float(entry.get("y", 0)),
            z=float(entry.get("z", 0)),
        )
    else:
        raise TypeError(f"Cannot parse object entry: {type(entry)} {entry}")


# ── Semi-structured log parser ──────────────────────────────────────────

# Matches Scribe disk format (Scribe spec §6):
#   {"ts":1765...,"lvl":"I","tag":"scene_svc","msg":"...","sid":"...","pid":"...","kv":{...}}
_LOG_LINE_RE = re.compile(r'^\s*\{.*"ts":\d+.*\}')

# Level shorthand mapping
_LVL_MAP = {"D": "Debug", "I": "Info", "W": "Warn", "E": "Error"}


def parse_log_lines(filepath: str) -> List[Dict[str, Any]]:
    """Parse a semi-structured Robonix log file (JSON-lines format).

    Each line is a JSON object matching the Scribe disk format (§6):
      {"ts":1765432100123456789,"lvl":"I","tag":"scene_svc","msg":"...","sid":"...","pid":"...","kv":{...}}

    Returns a list of dicts suitable for import_yaml_to_service pattern.
    """
    records = []
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # Skip non-JSON lines (console-format lines)
            if not line.startswith("{"):
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue

            rec = {
                "session_id": obj.get("sid", ""),
                "plan_id": obj.get("pid", ""),
                "level": _LVL_MAP.get(obj.get("lvl", "I"), "Info"),
                "tag": obj.get("tag", ""),
                "msg": obj.get("msg", ""),
                "ts": obj.get("ts", 0),
                "kv": obj.get("kv", {}),
            }
            records.append(rec)

    log.info("parse_log_lines: %d records from %s", len(records), filepath)
    return records


# ── Programmatic batch generator ────────────────────────────────────────

def generate_demo_data() -> List[Dict[str, Any]]:
    """Generate a batch of structured demo memory data programmatically.

    Covers typical robot task scenarios: fetch, navigate, craft, observe, error.
    Used as fallback when no YAML file is available.
    """
    now = time.time_ns()
    memories: List[Dict[str, Any]] = []

    def add(session, plan, level, tag, msg, scene="", action="", task="",
            success=True, objects=None, difficulty="easy", ts_offset=0):
        memories.append({
            "session_id": session,
            "plan_id": plan,
            "level": level,
            "tag": tag,
            "msg": msg,
            "ts": now + ts_offset,
            "objects": objects or [],
            "success": success,
        })

    # Kitchen fetch tasks
    add("sess-day1", "plan-fetch-1", "Info", "exec",
        "robot grasped the red cup from the kitchen counter",
        task="fetch", success=True,
        objects=[["scene.obj.cup_001", "red cup", 1.0, 2.0, 0.8]],
        ts_offset=0)

    add("sess-day1", "plan-fetch-2", "Info", "exec",
        "robot placed the blue cup on the living room coffee table",
        task="fetch", success=True,
        objects=[["scene.obj.cup_002", "blue cup", 3.0, 4.0, 0.5]],
        ts_offset=1_000_000_000)

    add("sess-day1", "plan-fetch-3", "Error", "exec",
        "robot failed to grasp the slippery glass on the kitchen sink — gripper slipped",
        task="fetch", success=False,
        objects=[["scene.obj.glass_001", "glass", 1.2, 2.3, 1.0]],
        difficulty="hard", ts_offset=2_000_000_000)

    # Navigation tasks
    add("sess-day1", "plan-nav-1", "Info", "nav",
        "navigated from the living room to the kitchen through the hallway",
        task="explore", success=True,
        objects=[],
        ts_offset=3_000_000_000)

    add("sess-day2", "plan-nav-2", "Warn", "nav",
        "navigated to the outdoor garden — unexpected obstacle at the doorway",
        task="explore", success=True,
        objects=[["scene.obj.door_001", "door", 5.0, 0.0, 0.0]],
        difficulty="medium", ts_offset=10_000_000_000)

    # Craft tasks
    add("sess-day2", "plan-craft-1", "Info", "exec",
        "crafted a wooden plank from 4 logs at the crafting table in the workshop",
        task="build", success=True,
        objects=[["scene.obj.plank_001", "wooden plank", 7.0, 8.0, 1.0]],
        difficulty="medium", ts_offset=11_000_000_000)

    add("sess-day2", "plan-craft-2", "Info", "exec",
        "crafted a stone axe from 3 stones and 2 sticks in the workshop",
        task="build", success=True,
        objects=[["scene.obj.axe_001", "stone axe", 7.1, 8.1, 1.0]],
        difficulty="hard", ts_offset=12_000_000_000)

    # Observation tasks
    add("sess-day3", "plan-obs-1", "Info", "camera",
        "observed and scanned the living room — detected sofa, tv, and coffee table",
        task="explore", success=True,
        objects=[["scene.obj.sofa_001", "sofa", 3.0, 4.0, 0.0],
                 ["scene.obj.tv_001", "television", 3.5, 2.0, 1.5],
                 ["scene.obj.table_001", "coffee table", 3.0, 3.0, 0.5]],
        ts_offset=20_000_000_000)

    # Failed lesson
    add("sess-day3", "plan-fetch-4", "Error", "exec",
        "robot attempted to grasp the heavy boulder — exceeded payload capacity, task aborted",
        task="fetch", success=False,
        objects=[["scene.obj.boulder_001", "boulder", 9.0, 9.0, 0.0]],
        difficulty="hard", ts_offset=21_000_000_000)

    return memories


# ── YAML exporter ───────────────────────────────────────────────────────

def export_to_yaml(memories: List[Dict[str, Any]], path: str) -> None:
    """Write memory records to a YAML file."""
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.dump({"memories": memories}, f, allow_unicode=True,
                  default_flow_style=False, sort_keys=False)
    log.info("export_to_yaml: %d records → %s", len(memories), path)
