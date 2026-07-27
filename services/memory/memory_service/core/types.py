"""Scribe Mem core data types.

All core data structures defined in Scribe-Mem-struct.md §2.
Phase1 uses Python dataclasses with to_dict/from_dict for JSON serialization
over the contract wire.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from enum import Enum, auto
from typing import Any, Dict, List, Optional, Set, Tuple


# ── Enums ────────────────────────────────────────────────────────────────


class NodeType(Enum):
    """Memory node classification; determines ID range and forget policy."""
    SHORT_TERM = "short_term"   # 0-999, FIFO eviction
    LONG_TERM = "long_term"     # 1000-8999, normal forget scoring
    SKILL = "skill"             # 9000-9998, protected (unless success rate drops)
    FIXED = "fixed"             # 9999 reserved, permanently protected
    LESSON = "lesson"           # long_term range, forget-protected


class CausalRelation(Enum):
    """Type of causal dependency between two MemoryNodes."""
    ENABLES = "enables"        # A must finish before B can start (collect → craft)
    TRIGGERS = "triggers"      # A's result triggers B (obstacle_detected → detour)
    DEPENDS_ON = "depends_on"  # B's execution depends on A's output (perceive → grasp)
    PARALLEL_TO = "parallel_to"  # A and B can run concurrently


class ConditionType(Enum):
    """Pre/post condition category."""
    OBJECT = "object"
    CAPABILITY = "capability"
    ENVIRONMENT = "environment"
    STATE = "state"


class ForgetDecision(Enum):
    """Outcome of a forget evaluation."""
    RETAIN = "retain"
    DOWNGRADE = "downgrade"
    ARCHIVE = "archive"
    DELETE = "delete"


# ── Core data structures (§2.1–§2.6) ─────────────────────────────────────


@dataclass
class ObjectCoord:
    """3D position of an object in the scene. (§2.3)"""
    obj_id: str                     # "scene.object.cup_003"
    label: str                      # "red cup"
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "ObjectCoord":
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


@dataclass
class SpatialContext:
    """Spatial context from Scene.list_objects. (§2.3)"""
    objects: List[ObjectCoord] = field(default_factory=list)
    origin: str = ""

    def __post_init__(self) -> None:
        """Reject coordinates whose reference frame was not supplied."""
        self.origin = str(self.origin or "").strip()
        if self.objects and not self.origin:
            raise ValueError("spatial origin frame is required when objects are present")

    def to_dict(self) -> Dict[str, Any]:
        return {"objects": [o.to_dict() for o in self.objects], "origin": self.origin}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "SpatialContext":
        objects = [ObjectCoord.from_dict(o) for o in d.get("objects", [])]
        return cls(objects=objects, origin=d.get("origin", ""))


@dataclass
class TagSet:
    """Four-dimension tag set for inverted-index filtering. (§2.2)

    Spatial dimension  — answers "where"
    Behaviour dimension — answers "what action"
    Cognitive dimension — answers "what task"
    Optimisation dimension — for forget scoring, not retrieval filtering
    """
    # Spatial
    scene_type: str = ""            # kitchen / living_room / workshop / outdoor
    objects_present: List[str] = field(default_factory=list)  # ["cup","table"]
    region: str = ""                # north / south / near_window

    # Behaviour
    action_type: str = ""           # grasp / place / navigate / craft / observe
    success: bool = True
    tool_used: List[str] = field(default_factory=list)  # provider_id list

    # Cognitive
    task_type: str = ""             # fetch / build / explore / dialogue
    difficulty: str = "medium"     # easy / medium / hard
    intent: str = ""               # LLM intent short-label

    # Optimisation (forget scoring, not retrieval)
    frequency: int = 0
    last_access: int = 0            # epoch ns
    quality_score: float = 0.5      # 0.0–1.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "TagSet":
        valid = {k: v for k, v in d.items() if k in cls.__dataclass_fields__}
        # Ensure list fields aren't left as None
        for list_field in ("objects_present", "tool_used"):
            if list_field in valid and valid[list_field] is None:
                valid[list_field] = []
        return cls(**valid)


@dataclass
class LogRecord:
    """Scribe Log record — the input source for MemoryNode. (§1.1)"""
    ts: int = 0                     # nanosecond timestamp, chronos now()
    level: str = "Info"             # Debug / Info / Warn / Error
    tag: str = ""                   # provider_id of source component
    msg: str = ""                   # free-text log body

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "LogRecord":
        valid = {k: v for k, v in d.items() if k in cls.__dataclass_fields__}
        return cls(**valid)


@dataclass
class MemoryNode:
    """CKG basic unit — one event the robot experienced. (§2.1)"""
    # Identity
    node_id: int = 0

    # Content
    summary: str = ""               # LLM one-liner; retrieval ranking basis
    raw_log: Optional[LogRecord] = None

    # Spatiotemporal
    timestamp: int = 0              # chronos ns
    spatial_data: Optional[SpatialContext] = None

    # Tags (4-dim)
    tags: Optional[TagSet] = None

    # Causal
    causal_chain: List[int] = field(default_factory=list)  # parent node IDs

    # Retrieval weight
    weight: float = 0.5             # 0.0–1.0 composite quality score

    # Embedding (Phase1: text-only, d=384 from all-MiniLM-L6-v2)
    embedding: List[float] = field(default_factory=list)

    # Images (patrol / inspection demo)
    image_refs: List[str] = field(default_factory=list)  # paths relative to data/

    # Metadata
    node_type: NodeType = NodeType.SHORT_TERM
    created_at: int = 0
    last_access: int = 0
    access_count: int = 0
    version: int = 1                # optimistic lock

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {}
        d["node_id"] = self.node_id
        d["summary"] = self.summary
        d["raw_log"] = self.raw_log.to_dict() if self.raw_log else None
        d["timestamp"] = self.timestamp
        d["spatial_data"] = self.spatial_data.to_dict() if self.spatial_data else None
        d["tags"] = self.tags.to_dict() if self.tags else None
        d["causal_chain"] = list(self.causal_chain)
        d["weight"] = self.weight
        d["embedding"] = list(self.embedding)
        d["node_type"] = self.node_type.value
        d["created_at"] = self.created_at
        d["last_access"] = self.last_access
        d["access_count"] = self.access_count
        d["version"] = self.version
        d["image_refs"] = list(self.image_refs)
        return d

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "MemoryNode":
        raw_log = LogRecord.from_dict(d["raw_log"]) if d.get("raw_log") else None
        spatial = SpatialContext.from_dict(d["spatial_data"]) if d.get("spatial_data") else None
        tags = TagSet.from_dict(d["tags"]) if d.get("tags") else None
        node_type = NodeType(d.get("node_type", "short_term"))
        return cls(
            node_id=d.get("node_id", 0),
            summary=d.get("summary", ""),
            raw_log=raw_log,
            timestamp=d.get("timestamp", 0),
            spatial_data=spatial,
            tags=tags,
            causal_chain=d.get("causal_chain", []),
            weight=d.get("weight", 0.5),
            embedding=d.get("embedding", []),
            node_type=node_type,
            created_at=d.get("created_at", 0),
            last_access=d.get("last_access", 0),
            access_count=d.get("access_count", 0),
            version=d.get("version", 1),
            image_refs=d.get("image_refs", []),
        )


@dataclass
class CausalEdge:
    """Directed edge in the CKG. (§2.4)"""
    from_node_id: int
    to_node_id: int
    relation: CausalRelation = CausalRelation.ENABLES

    def to_dict(self) -> Dict[str, Any]:
        return {"from_node_id": self.from_node_id, "to_node_id": self.to_node_id,
                "relation": self.relation.value}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "CausalEdge":
        return cls(from_node_id=d["from_node_id"], to_node_id=d["to_node_id"],
                   relation=CausalRelation(d.get("relation", "enables")))


@dataclass
class TimeRange:
    """Time range filter for search/history queries."""
    start_ts: int = 0
    end_ts: int = 0                 # 0 = no upper bound (use current time)


@dataclass
class TagFilter:
    """Tag filter for search queries. All fields optional — AND semantics. (§4.1)"""
    scene_type: Optional[str] = None
    objects: Optional[List[str]] = None
    action_type: Optional[str] = None
    success: Optional[bool] = None
    task_type: Optional[str] = None
    difficulty_max: Optional[str] = None  # "easy" / "medium" (filter <= this level)

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        if self.scene_type is not None:
            result["scene_type"] = self.scene_type
        if self.objects is not None:
            result["objects"] = self.objects
        if self.action_type is not None:
            result["action_type"] = self.action_type
        if self.success is not None:
            result["success"] = self.success
        if self.task_type is not None:
            result["task_type"] = self.task_type
        if self.difficulty_max is not None:
            result["difficulty_max"] = self.difficulty_max
        return result

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "TagFilter":
        return cls(
            scene_type=d.get("scene_type"),
            objects=d.get("objects"),
            action_type=d.get("action_type"),
            success=d.get("success"),
            task_type=d.get("task_type"),
            difficulty_max=d.get("difficulty_max"),
        )

    def is_empty(self) -> bool:
        """True if no filter condition is set."""
        return (self.scene_type is None and self.objects is None
                and self.action_type is None and self.success is None
                and self.task_type is None and self.difficulty_max is None)


# ── Forgetting (§2.6) ────────────────────────────────────────────────────


@dataclass
class ForgetRisk:
    """Forget risk score for a single node."""
    node_id: int
    frequency_score: float = 0.0    # 0-1, lower access = higher
    recency_score: float = 0.0      # 0-1, older = higher
    quality_score: float = 0.0      # 0-1, lower quality = higher
    redundancy_score: float = 0.0   # 0-1, more redundant = higher
    total_risk: float = 0.0
    is_protected: bool = False
    recent_hit: bool = False
    in_causal_chain: bool = False


@dataclass
class ForgetAuditEntry:
    """Audit log entry for a forget action."""
    timestamp: int
    node_id: int
    decision: str                   # ForgetDecision value
    risk_total: float
    operator: str = "system"


# ── Request / Response types for contracts (§4) ──────────────────────────


@dataclass
class RememberRequest:
    """Input for `robonix/service/memory/remember`. (§4.1)"""
    session_id: str
    plan_id: str
    log_record: LogRecord
    spatial: Optional[SpatialContext] = None
    parent_node_id: Optional[int] = None
    image_base64: str = ""           # top-level: camera frame as base64
    kv: Dict[str, str] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "session_id": self.session_id,
            "plan_id": self.plan_id,
            "log_record": self.log_record.to_dict(),
            "spatial": self.spatial.to_dict() if self.spatial else None,
            "parent_node_id": self.parent_node_id,
            "image_base64": self.image_base64,
            "kv": self.kv,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "RememberRequest":
        spatial = SpatialContext.from_dict(d["spatial"]) if d.get("spatial") else None
        return cls(
            session_id=d.get("session_id", ""),
            plan_id=d.get("plan_id", ""),
            log_record=LogRecord.from_dict(d.get("log_record", {})),
            spatial=spatial,
            parent_node_id=d.get("parent_node_id"),
            kv=d.get("kv", {}),
        )


@dataclass
class RememberResponse:
    """Output for `robonix/service/memory/remember`."""
    node_id: int
    message: str = ""


@dataclass
class SearchRequest:
    """Input for `robonix/service/memory/hybrid_search`. (§4.1)"""
    query: str
    tags: Optional[TagFilter] = None
    top_k: int = 5
    alpha: Optional[float] = None  # BM25 weight; None → use default 0.3
    time_range: Optional[TimeRange] = None
    require_executable: bool = False
    vlm_qa: bool = False            # if True, include VLM answer from node images

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {"query": self.query, "top_k": self.top_k}
        if self.tags is not None:
            d["tags"] = self.tags.to_dict()
        if self.alpha is not None:
            d["alpha"] = self.alpha
        if self.time_range is not None:
            d["time_range"] = {"start_ts": self.time_range.start_ts,
                               "end_ts": self.time_range.end_ts}
        d["require_executable"] = self.require_executable
        if self.vlm_qa:
            d["vlm_qa"] = self.vlm_qa
        return d

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "SearchRequest":
        tags = TagFilter.from_dict(d["tags"]) if d.get("tags") else None
        tr = None
        if d.get("time_range"):
            tr = TimeRange(start_ts=d["time_range"].get("start_ts", 0),
                           end_ts=d["time_range"].get("end_ts", 0))
        return cls(
            query=d.get("query", ""),
            tags=tags,
            top_k=d.get("top_k", 5),
            alpha=d.get("alpha"),
            time_range=tr,
            require_executable=d.get("require_executable", False),
            vlm_qa=d.get("vlm_qa", False),
        )


@dataclass
class SearchResponse:
    """Output for `robonix/service/memory/hybrid_search`."""
    nodes: List[MemoryNode] = field(default_factory=list)
    vlm_answer: str = ""            # VLM QA result when vlm_qa=true


@dataclass
class CompactResponse:
    """Output for `robonix/service/memory/promote`."""
    summary: str = ""
    nodes_compacted: int = 0


# ── Skill types (§2.5, Phase2 placeholder) ───────────────────────────────


@dataclass
class Condition:
    """Pre/post condition for a skill step."""
    type: str = ""                  # ConditionType value
    predicate: str = ""             # "robot.has(item=log, count>=1)"


@dataclass
class StepTemplate:
    """One step in a skill's causal chain."""
    order: int = 0
    capability_id: str = ""
    action_desc: str = ""
    parallel_group: Optional[int] = None
    depends_on_step: List[int] = field(default_factory=list)


@dataclass
class Constraint:
    """Execution constraint (safety/capability/environment)."""
    constraint_type: str = ""
    description: str = ""
    check_expr: str = ""


@dataclass
class SkillTemplate:
    """Extracted skill from N≥3 successful experiences. (§2.5)"""
    skill_id: int = 0
    skill_name: str = ""
    version: int = 1
    goal_template: str = ""
    goal_params: List[str] = field(default_factory=list)
    causal_template: List[StepTemplate] = field(default_factory=list)
    pre_conditions: List[Condition] = field(default_factory=list)
    post_conditions: List[Condition] = field(default_factory=list)
    constraints: List[Constraint] = field(default_factory=list)
    success_rate: float = 0.0
    sample_count: int = 0
    source_nodes: List[int] = field(default_factory=list)
    created_at: int = 0
    last_updated: int = 0
    deprecated: bool = False


@dataclass
class SkillSummary:
    """Lightweight skill listing entry."""
    skill_id: int
    skill_name: str
    version: int
    success_rate: float
    sample_count: int
    deprecated: bool = False


# ── Difficulty ordering helper ───────────────────────────────────────────

_DIFFICULTY_ORDER: Dict[str, int] = {"easy": 1, "medium": 2, "hard": 3}


def difficulty_leq(a: str, b: str) -> bool:
    """Check if difficulty a ≤ b using ordinal values."""
    return _DIFFICULTY_ORDER.get(a, 2) <= _DIFFICULTY_ORDER.get(b, 2)
