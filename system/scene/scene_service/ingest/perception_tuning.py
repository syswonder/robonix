# SPDX-License-Identifier: MulanPSL-2.0
"""Validated perception tuning for Scene's ConceptGraphs detector.

`PerceptionTuning` is the single home for every knob the detector reads out of
the Driver `CMD_INIT` `perception` config section.  Previously each knob was
spelled three times — extracted and range-checked in `service.py`, forwarded as
a keyword argument, then re-clamped in `ConceptGraphsDetector.__init__` — which
made the three copies free to disagree.  Here a knob is declared once in
`_SCALARS`, validated once in `__post_init__`, and read once by the detector.

Validation *rejects* rather than clamps: an out-of-range value in a deployment
config is an operator mistake worth surfacing at boot, not something to silently
round into range.  Error messages quote the full `perception.…` config path so
the operator can find the offending key.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, Mapping, Optional


# ── Open-vocab class list ────────────────────────────────────────────────
# One build-owned vocabulary drives both runtime class resolution and the
# checkpoint baker. Changing it therefore requires a package rebuild instead
# of allocating YOLO-World's CLIP text tower during robot boot.
_YOLO_CLASSES_PATH = (
    Path(__file__).resolve().parents[2] / "docker" / "yolo_world_classes.json"
)
DEFAULT_OPEN_VOCAB: list[str] = json.loads(
    _YOLO_CLASSES_PATH.read_text(encoding="utf-8")
)


def resolved_classes(
    configured: Optional[list[str]] = None,
    additional: Optional[list[str]] = None,
) -> list[str]:
    """Resolve the open-vocabulary prompt list with stable de-duplication.

    Normal deployments use Driver config. ``SCENE_OPEN_VOCAB_CLASSES`` remains
    a legacy full override only when no configured base list is supplied.
    """
    raw = (
        os.environ.get("SCENE_OPEN_VOCAB_CLASSES", "").strip()
        if configured is None and additional is None
        else ""
    )
    if raw:
        candidates = [s.strip() for s in raw.split(",") if s.strip()]
    else:
        candidates = list(configured or DEFAULT_OPEN_VOCAB)
        candidates.extend(additional or ())
    out: list[str] = []
    seen: set[str] = set()
    for candidate in candidates:
        label = str(candidate or "").strip().lower()
        if label and label not in seen:
            seen.add(label)
            out.append(label)
    return out


# ── Candidate scopes for the label-correction features ───────────────────
# Three mechanisms below — CLIP label rerank, planar surface snapping, and
# stationary tile refinement — only run once an operator scopes them, and they
# ship scoped to nothing. That is deliberate: the F1 / label-accuracy numbers
# measured for this pipeline were taken with them off, so switching any of them
# on by default would invalidate those measurements.
#
# These constants are the proposed starting scopes, derived from the shipped
# vocabulary and from the confusions actually observed in Frozen Office runs.
# They are validated against the vocabulary by tests/test_perception_profiles.py
# so a vocabulary edit cannot silently invalidate them, but nothing reads them
# at runtime. To adopt one, copy it into the Driver `perception` config (see
# "Tuning label correction" in the Scene README) and re-measure before making it
# the default here.
#
# Each group is a symmetric confusable set: CLIP rescores a detection only among
# its own group, and only switches when it clears `min_score` and `min_margin`.
# A label may appear in at most one group.
CANDIDATE_CLIP_RERANK_GROUPS: tuple[tuple[str, ...], ...] = (
    # Storage furniture: near-identical rectangular fronts at room distance.
    ("cabinet", "drawer", "dresser", "wardrobe"),
    # Flat work surfaces: separated mostly by height and context, not shape.
    ("table", "desk", "workbench", "counter"),
    ("monitor", "television"),
    ("bookshelf", "shelf"),
    ("door", "doorway"),
    # Small tabletop objects, where the mask is only a few hundred points and
    # the geometric gates cannot separate the classes at all.
    ("cup", "mug", "glass", "paper cup"),
    ("bottle", "water bottle", "thermos", "carafe"),
    ("bowl", "plate", "tray"),
    ("box", "cardboard box", "crate"),
    ("book", "notebook"),
    ("phone", "tablet"),
)

# Pure synonyms in the shipped vocabulary. An alias is a rename, not a rerank:
# it collapses the pair before voting so the two spellings stop splitting the
# label history of one object.
CANDIDATE_LABEL_ALIASES: dict[str, str] = {"sofa": "couch"}

# Large wall-adjacent furniture, whose depth cloud is usually a partial front
# face: the centroid drifts toward the camera as the fused cloud grows. Snapping
# the cloud onto the occupancy-grid surface behind it is what this is for.
# Unlike the rerank groups above, this set is a hypothesis about which classes
# benefit — it has not been measured per label.
CANDIDATE_SURFACE_SNAP_LABELS: tuple[str, ...] = (
    "cabinet",
    "wardrobe",
    "dresser",
    "bookshelf",
    "shelf",
    "refrigerator",
    "whiteboard",
    "radiator",
)


# ── Scalar knob table ────────────────────────────────────────────────────
# (attribute, config section under `perception`, interval, config key).
# The key defaults to the attribute name; only knobs whose config spelling
# differs from the attribute spell it out. The interval uses ordinary maths
# notation — "[1,)" is "at least one", "(0,1]" is "greater than zero and at
# most one", "" is unbounded — and drives both the range check and the message
# shown when the check fails. The cast comes from the dataclass annotation, so
# a field cannot drift from the type its config value is coerced to.
_SCALARS: tuple[tuple[str, str, str] | tuple[str, str, str, str], ...] = (
    # perception.*
    ("visible_miss_threshold", "", "[1,)", "visible_miss_frames"),
    ("visibility_depth_margin_m", "", "[0,)"),
    ("visibility_min_clear_samples", "", "[1,)"),
    ("visibility_min_clear_fraction", "", "(0,1]"),
    ("visibility_max_projected_samples", "", "[1,)"),
    ("object_ttl_s", "", "[0,)"),
    ("confirmation_min_unique_frames", "", "[1,)"),
    ("confirmation_singleton_min_mean_confidence", "", "[0,1]"),
    ("confidence_threshold", "", "(0,1]"),
    # perception.geometry.*
    ("mask_erosion_px", "geometry", "[0,)"),
    ("min_depth_m", "geometry", "[0,)"),
    ("max_depth_m", "geometry", "(0,)"),
    ("depth_mad_scale", "geometry", "[0,)"),
    ("depth_min_band_m", "geometry", "[0,)"),
    ("frame_dbscan", "geometry", ""),
    ("require_occupancy_bounds", "geometry", ""),
    ("map_bounds_margin_m", "geometry", "[0,)"),
    ("map_max_outside_fraction", "geometry", "[0,1]"),
    ("bbox_low_percentile", "geometry", "[0,100)"),
    ("bbox_high_percentile", "geometry", "(0,100]"),
    ("max_bbox_extent_m", "geometry", "(0,)"),
    # perception.geometry.scale_aware.*
    ("scale_aware_geometry", "geometry.scale_aware", "", "enabled"),
    ("scale_min_voxel_size_m", "geometry.scale_aware", "(0,)", "min_voxel_size_m"),
    ("scale_min_points_floor", "geometry.scale_aware", "[1,)", "min_points_floor"),
    ("scale_transition_extent_m", "geometry.scale_aware", "(0,)", "transition_extent_m"),
    ("scale_voxel_extent_factor", "geometry.scale_aware", "(0,)", "voxel_extent_factor"),
    # perception.geometry.surface_snap.* (key = attribute minus the prefix)
    ("surface_snap_max_distance_m", "geometry.surface_snap", "(0,)", "max_distance_m"),
    ("surface_snap_tangent_padding_m", "geometry.surface_snap", "[0,)", "tangent_padding_m"),
    ("surface_snap_min_shift_m", "geometry.surface_snap", "[0,)", "min_shift_m"),
    ("surface_snap_min_support_cells", "geometry.surface_snap", "[1,)", "min_support_cells"),
    ("surface_snap_min_dominant_share", "geometry.surface_snap", "(0,1]", "min_dominant_share"),
    ("surface_snap_min_tangent_coverage", "geometry.surface_snap", "(0,1]", "min_tangent_coverage"),
    ("surface_snap_occupancy_threshold", "geometry.surface_snap", "[1,100]", "occupancy_threshold"),
    # perception.stationary_refinement.* (key = attribute minus the prefix)
    ("stationary_refinement", "stationary_refinement", "", "enabled"),
    ("stationary_refinement_min_stationary_s", "stationary_refinement", "[0,)", "min_stationary_s"),
    ("stationary_refinement_interval_s", "stationary_refinement", "(0,)", "interval_s"),
    ("stationary_refinement_translation_m", "stationary_refinement", "[0,)", "translation_threshold_m"),
    ("stationary_refinement_rotation_deg", "stationary_refinement", "[0,)", "rotation_threshold_deg"),
    ("stationary_refinement_grid_size", "stationary_refinement", "[1,)", "grid_size"),
    ("stationary_refinement_overlap_fraction", "stationary_refinement", "[0,0.8]", "overlap_fraction"),
    ("stationary_refinement_edge_margin_px", "stationary_refinement", "[0,)", "edge_margin_px"),
    ("stationary_refinement_duplicate_iou", "stationary_refinement", "[0,1]", "duplicate_iou"),
    # perception.label.* (key = attribute minus the `label_` prefix)
    ("label_history_size", "label", "[1,)", "history_size"),
    ("label_min_switch_observations", "label", "[1,)", "min_switch_observations"),
    ("label_min_winner_share", "label", "[0,1]", "min_winner_share"),
    ("label_switch_margin", "label", "[0,1]", "switch_margin"),
    # perception.label.clip_rerank.*
    ("clip_rerank_min_score", "label.clip_rerank", "[-1,1]", "min_score"),
    ("clip_rerank_min_margin", "label.clip_rerank", "[0,2]", "min_margin"),
    (
        "clip_rerank_geometry_bonus",
        "label.clip_rerank.persistent_geometry",
        "[0,2]",
        "score_bonus",
    ),
    # perception.association.*
    ("association_min_spatial_similarity", "association", "[0,1]", "min_spatial_similarity"),
    ("association_min_visual_similarity", "association", "[-1,1]", "min_visual_similarity"),
    ("association_max_extent_ratio", "association", "[1,)", "max_extent_ratio"),
    ("association_adaptive_extent_scale", "association", "[0,)", "adaptive_extent_scale"),
    ("association_cleanup_interval_ticks", "association", "[1,)", "cleanup_interval_ticks"),
    ("exact_duplicate_centroid_max_m", "association", "[0,)"),
    ("exact_duplicate_min_voxel_coverage", "association", "[0,1]"),
    ("exact_duplicate_max_extent_ratio", "association", "[1,)"),
    ("coobserved_duplicate_min_shared_frames", "association", "[1,)"),
    ("coobserved_duplicate_min_median_iou", "association", "[0,1]"),
    ("coobserved_duplicate_max_extent_ratio", "association", "[1,)"),
    ("coobserved_duplicate_min_visual_similarity", "association", "[-1,1]"),
    ("identity_rebind_max_distance_m", "association", "[0,)"),
)


def _knob(entry: tuple[str, ...]) -> tuple[str, str, str, str, str]:
    """Expand one `_SCALARS` row into (attribute, section, key, interval, path)."""
    name, section, interval = entry[0], entry[1], entry[2]
    key = entry[3] if len(entry) > 3 else name
    path = ".".join(filter(None, ("perception", section, key)))
    return name, section, key, interval, path


# Knobs that are deployment-shaped rather than perception-shaped: they come
# from the resolved profile or an environment override, so `from_config`
# resolves them by hand after the table pass.
_PROFILE_KEYS = frozenset(
    {
        "period_s",
        "input_size",
        "max_detections",
        "inference_precision",
        "tensor_rt_mode",
        "yolo_weights_path",
        "stationary_refinement_input_size",
        "stationary_refinement_max_detections",
    }
)

# Class-gated identity knobs Scene no longer honours. Accepting them keeps old
# manifests booting; the caller logs one deprecation line per key found.
DEPRECATED_ASSOCIATION_KEYS = (
    "allow_cross_class_merge",
    "confusable_class_groups",
    "cross_class_centroid_max_m",
    "cross_class_iou_thresh",
    "cross_class_overlap_thresh",
    "same_class_centroid_max_m",
    "same_class_min_voxel_coverage",
    "same_class_max_extent_ratio",
    "same_class_disjoint_min_unique_frames",
    "same_class_disjoint_max_frame_gap",
    "same_class_disjoint_max_center_major_extent_ratio",
    "same_class_disjoint_min_visual_similarity",
    "same_class_merge_interval_ticks",
)

_CASTS: dict[str, type] = {"int": int, "float": float, "bool": bool}

_GEOMETRY_CONSTRAINT_KEYS = frozenset(
    {"source_labels", "min_horizontal_extent_m", "max_horizontal_extent_m"}
)


def _describe(interval: str) -> str:
    """Render an interval spec as the phrase used in validation messages."""
    low, high = interval[1:-1].split(",")
    low_word = "greater than" if interval[0] == "(" else "at least"
    high_word = "less than" if interval[-1] == ")" else "at most"
    parts = []
    if low:
        parts.append(f"{low_word} {low}")
    if high:
        parts.append(f"{high_word} {high}")
    return " and ".join(parts)


def _check_interval(path: str, value: float, interval: str) -> None:
    """Raise ValueError unless `value` falls inside the interval spec."""
    if not interval:
        return
    if not math.isfinite(value):
        raise ValueError(f"{path} must be a finite number")
    low, high = interval[1:-1].split(",")
    if low:
        bound = float(low)
        if value < bound or (interval[0] == "(" and value == bound):
            raise ValueError(f"{path} must be {_describe(interval)}")
    if high:
        bound = float(high)
        if value > bound or (interval[-1] == ")" and value == bound):
            raise ValueError(f"{path} must be {_describe(interval)}")


def _coerce(path: str, raw: Any, cast: type) -> Any:
    """Cast one config value, rejecting shapes the cast would silently mangle."""
    if cast is bool:
        if isinstance(raw, bool):
            return raw
        if isinstance(raw, str) and raw.strip().lower() in {
            "true", "false", "1", "0", "yes", "no",
        }:
            return raw.strip().lower() in {"true", "1", "yes"}
        raise ValueError(f"{path} must be a boolean")
    if isinstance(raw, bool) or isinstance(raw, (list, dict, tuple, set)):
        raise ValueError(f"{path} must be a number")
    try:
        return cast(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path} must be a number") from exc


def _section(config: Mapping[str, Any], path: str) -> Mapping[str, Any]:
    """Walk a dotted section path under `perception`, rejecting non-mappings."""
    current: Mapping[str, Any] = config
    walked = "perception"
    for part in filter(None, path.split(".")):
        value = current.get(part) or {}
        walked = f"{walked}.{part}"
        if not isinstance(value, dict):
            raise ValueError(f"{walked} must be a mapping")
        current = value
    return current


def _string_list(
    mapping: Mapping[str, Any],
    prefix: str,
    key: str,
    *,
    allow_empty: bool,
) -> Optional[list[str]]:
    """Normalise a list-of-strings config value to lowercase, or None if unset."""
    if key not in mapping:
        return None
    value = mapping[key]
    if not isinstance(value, list) or any(
        not isinstance(item, str) for item in value
    ):
        raise ValueError(f"{prefix}.{key} must be a list of strings")
    normalized = [item.strip().lower() for item in value if item.strip()]
    if not normalized and not allow_empty:
        raise ValueError(f"{prefix}.{key} must not be empty")
    return normalized


def _rerank_scope(
    raw: Any,
    path: str,
    *,
    minimum: int = 2,
) -> tuple[list[str], Optional[float]]:
    """Parse one rerank group/route entry into labels plus its margin override.

    Accepts either a bare string list or a ``{labels, min_margin}`` mapping so a
    single confusable set can carry a stricter switch margin than the global one.
    """
    margin: Optional[float] = None
    if isinstance(raw, list):
        labels = raw
    elif isinstance(raw, dict):
        unknown = set(raw) - {"labels", "min_margin"}
        if unknown:
            raise ValueError(
                f"{path} mappings contain unknown keys: "
                + ", ".join(sorted(unknown))
            )
        labels = raw.get("labels")
        if "min_margin" in raw:
            margin = _coerce(f"{path}.min_margin", raw["min_margin"], float)
            _check_interval(f"{path}.min_margin", margin, "[0,2]")
    else:
        labels = None
    if (
        not isinstance(labels, list)
        or any(not isinstance(item, str) for item in labels)
    ):
        raise ValueError(
            f"{path} must be a string list or a {{labels, min_margin}} mapping"
        )
    normalized = list(
        dict.fromkeys(item.strip().lower() for item in labels if item.strip())
    )
    if len(normalized) < minimum:
        raise ValueError(
            f"{path} must contain at least {minimum} distinct non-empty labels"
        )
    return normalized, margin


@dataclass
class PerceptionTuning:
    """Every validated knob `ConceptGraphsDetector` reads from Driver config.

    Construct via :meth:`from_config` for a deployment, or directly with keyword
    overrides in tests. `__post_init__` runs the same validation either way, so a
    directly-constructed instance cannot carry a range a config could not.
    """

    # ── detection cadence and deployment profile ──
    profile: str = "full"
    period_s: float = 0.6
    input_size: int = 640
    max_detections: int = 30
    confidence_threshold: float = 0.30
    inference_precision: str = "auto"
    tensor_rt_mode: str = "off"
    tensor_rt_cache_dir: str = "/opt/models/runtime-cache"
    yolo_weights_path: Optional[str] = None
    sam_weights_path: Optional[str] = None
    clip_model_name: Optional[str] = None
    clip_pretrained: Optional[str] = None

    # ── absence evidence (an object is deleted only on positive evidence) ──
    visible_miss_threshold: int = 3
    visibility_depth_margin_m: float = 0.10
    visibility_min_clear_samples: int = 3
    visibility_min_clear_fraction: float = 0.60
    visibility_max_projected_samples: int = 25
    visibility_upper_sample_fraction: float = 0.70
    visibility_min_depth_margin_m: float = 0.015
    visibility_extent_margin_scale: float = 0.30
    object_ttl_s: float = 30.0
    confirmation_min_unique_frames: int = 2
    confirmation_singleton_min_mean_confidence: float = 0.0

    # ── per-frame geometry ──
    mask_erosion_px: int = 1
    min_depth_m: float = 0.15
    max_depth_m: float = 6.0
    depth_mad_scale: float = 3.5
    depth_min_band_m: float = 0.12
    frame_dbscan: bool = True
    scale_aware_geometry: bool = True
    scale_min_voxel_size_m: float = 0.01
    scale_min_points_floor: int = 8
    scale_transition_extent_m: float = 0.30
    scale_voxel_extent_factor: float = 0.30
    require_occupancy_bounds: bool = True
    map_bounds_margin_m: float = 0.25
    map_max_outside_fraction: float = 0.20
    bbox_low_percentile: float = 5.0
    bbox_high_percentile: float = 95.0
    max_bbox_extent_m: float = 3.0

    # ── stationary tile refinement ──
    stationary_refinement: bool = False
    stationary_refinement_min_stationary_s: float = 2.0
    stationary_refinement_interval_s: float = 30.0
    stationary_refinement_translation_m: float = 0.03
    stationary_refinement_rotation_deg: float = 2.0
    stationary_refinement_grid_size: int = 2
    stationary_refinement_overlap_fraction: float = 0.20
    stationary_refinement_input_size: int = 640
    stationary_refinement_edge_margin_px: int = 4
    stationary_refinement_duplicate_iou: float = 0.50
    stationary_refinement_max_detections: int = 8

    # ── planar surface snapping ──
    surface_snap_labels: list[str] = field(default_factory=list)
    surface_snap_max_distance_m: float = 0.60
    surface_snap_tangent_padding_m: float = 0.25
    surface_snap_min_shift_m: float = 0.05
    surface_snap_min_support_cells: int = 30
    surface_snap_min_dominant_share: float = 0.55
    surface_snap_min_tangent_coverage: float = 0.50
    surface_snap_occupancy_threshold: int = 50

    # ── vocabulary and label stabilisation ──
    classes: Optional[list[str]] = None
    additional_classes: Optional[list[str]] = None
    label_history_size: int = 20
    label_min_switch_observations: int = 2
    label_min_winner_share: float = 0.55
    label_switch_margin: float = 0.15
    label_aliases: dict[str, str] = field(default_factory=dict)

    # ── CLIP label rerank ──
    clip_rerank_groups: list[list[str]] = field(default_factory=list)
    clip_rerank_routes: dict[str, list[str]] = field(default_factory=dict)
    clip_rerank_prompts: dict[str, list[str]] = field(default_factory=dict)
    clip_rerank_min_score: float = 0.20
    clip_rerank_min_margin: float = 0.04
    clip_rerank_min_margin_by_label: dict[str, float] = field(default_factory=dict)
    clip_rerank_geometry_bonus: float = 0.0
    clip_rerank_geometry_constraints: dict[str, dict[str, Any]] = field(
        default_factory=dict
    )

    # ── association and duplicate collapse ──
    exact_duplicate_centroid_max_m: float = 0.15
    exact_duplicate_min_voxel_coverage: float = 0.40
    exact_duplicate_max_extent_ratio: float = 1.50
    coobserved_duplicate_min_shared_frames: int = 3
    coobserved_duplicate_min_median_iou: float = 0.85
    coobserved_duplicate_max_extent_ratio: float = 2.0
    coobserved_duplicate_min_visual_similarity: float = 0.90
    identity_rebind_max_distance_m: float = 0.45
    association_min_spatial_similarity: float = 0.03
    association_min_visual_similarity: float = 0.65
    association_max_extent_ratio: float = 8.0
    association_adaptive_extent_scale: float = 0.45
    association_cleanup_interval_ticks: int = 10

    # Resolved open-vocabulary prompt list; derived, never supplied directly.
    resolved_classes: list[str] = field(default_factory=list, init=False)

    def __post_init__(self) -> None:
        """Range-check every knob and resolve the derived vocabulary.

        Runs for config-built and test-built instances alike. Raises ValueError
        naming the offending `perception.…` path; leaves the instance otherwise
        untouched so the detector can read fields without re-clamping.
        """
        self.profile = str(self.profile).strip().lower()
        self.inference_precision = str(self.inference_precision).strip().lower()
        self.tensor_rt_mode = str(self.tensor_rt_mode).strip().lower()
        self.tensor_rt_cache_dir = str(self.tensor_rt_cache_dir).strip()
        for entry in _SCALARS:
            name, _section, _key, interval, path = _knob(entry)
            if interval:
                _check_interval(path, float(getattr(self, name)), interval)
        self._validate_cross_field()
        self.resolved_classes = resolved_classes(
            self.classes,
            self.additional_classes,
        )
        if not self.resolved_classes:
            raise ValueError("open-vocabulary class list must not be empty")
        self._validate_vocabulary_scoped()
        # The refinement pass is driven from the detection loop, so it can never
        # run more often than the loop itself ticks.
        self.stationary_refinement_interval_s = max(
            self.period_s,
            self.stationary_refinement_interval_s,
        )

    def _validate_cross_field(self) -> None:
        """Check the ranges that only make sense relative to another knob."""
        if self.period_s <= 0.0:
            raise ValueError("perception.period_s must be greater than zero")
        if self.inference_precision not in {"auto", "fp16", "fp32"}:
            raise ValueError(
                "perception.precision must be one of auto, fp16, or fp32"
            )
        if self.tensor_rt_mode not in {"off", "auto", "required"}:
            raise ValueError(
                "perception.tensor_rt must be one of off, auto, or required"
            )
        if self.max_depth_m <= self.min_depth_m:
            raise ValueError(
                "perception.geometry depth range must satisfy "
                "0 <= min_depth_m < max_depth_m"
            )
        if self.bbox_low_percentile >= self.bbox_high_percentile:
            raise ValueError(
                "perception.geometry bbox percentiles must satisfy "
                "0 <= low < high <= 100"
            )
        if self.visibility_max_projected_samples < self.visibility_min_clear_samples:
            raise ValueError(
                "perception.visibility_max_projected_samples must be at least "
                "perception.visibility_min_clear_samples"
            )
        if self.surface_snap_min_shift_m > self.surface_snap_max_distance_m:
            raise ValueError(
                "perception.geometry.surface_snap.min_shift_m must be in "
                "[0, max_distance_m]"
            )
        if self.stationary_refinement_input_size < 320:
            raise ValueError(
                "perception.stationary_refinement.input_size must be at least 320"
            )
        if self.stationary_refinement_max_detections < 0:
            raise ValueError(
                "perception.stationary_refinement.max_supplemental_detections "
                "must not be negative"
            )
        # A lite profile ships no detector, so its zeroed model knobs are not
        # an operator error the way they would be under full/jetson.
        if self.profile != "lite":
            if self.max_detections < 1:
                raise ValueError("perception.max_detections must be at least one")
            if self.input_size < 320:
                raise ValueError("perception.input_size must be at least 320")

    def _validate_vocabulary_scoped(self) -> None:
        """Check every knob that names a label against the resolved vocabulary.

        A typo in a rerank group or alias would otherwise sit inert forever: the
        label simply never matches a detection, so the operator sees no rerank
        and no error. Failing at boot makes the typo obvious.
        """
        vocabulary = set(self.resolved_classes)

        unknown = set(self.surface_snap_labels) - vocabulary
        if unknown:
            raise ValueError(
                "perception.geometry.surface_snap.labels reference labels "
                "outside the resolved vocabulary: " + ", ".join(sorted(unknown))
            )

        aliases = set(self.label_aliases) | set(self.label_aliases.values())
        unknown = aliases - vocabulary
        if unknown:
            raise ValueError(
                "perception.label.aliases reference labels outside the "
                "resolved vocabulary: " + ", ".join(sorted(unknown))
            )
        chained = set(self.label_aliases.values()) & set(self.label_aliases)
        if chained:
            raise ValueError(
                "perception.label.aliases targets must be canonical, not "
                "another alias: " + ", ".join(sorted(chained))
            )

        flattened = [label for group in self.clip_rerank_groups for label in group]
        repeated = {label for label in flattened if flattened.count(label) > 1}
        if repeated:
            raise ValueError(
                "perception.label.clip_rerank.groups labels may appear in only "
                "one group: " + ", ".join(sorted(repeated))
            )
        invalid = {
            source
            for source, labels in self.clip_rerank_routes.items()
            if source not in labels
        }
        if invalid:
            raise ValueError(
                "each perception.label.clip_rerank.routes entry must contain "
                "its source label: " + ", ".join(sorted(invalid))
            )
        ambiguous = set(self.clip_rerank_routes) & set(flattened)
        if ambiguous:
            raise ValueError(
                "perception.label.clip_rerank.routes sources must not also "
                "belong to a symmetric group: " + ", ".join(sorted(ambiguous))
            )
        scoped = set(flattened) | {
            label for labels in self.clip_rerank_routes.values() for label in labels
        }
        unknown = scoped - vocabulary
        if unknown:
            raise ValueError(
                "perception.label.clip_rerank scopes reference labels outside "
                "the resolved vocabulary: " + ", ".join(sorted(unknown))
            )
        unknown = set(self.clip_rerank_prompts) - scoped
        if unknown:
            raise ValueError(
                "perception.label.clip_rerank.prompts reference labels outside "
                "the configured groups or routes: " + ", ".join(sorted(unknown))
            )
        unknown = set(self.clip_rerank_min_margin_by_label) - scoped
        if unknown:
            raise ValueError(
                "perception.label.clip_rerank margin overrides reference "
                "labels outside the configured groups or routes: "
                + ", ".join(sorted(unknown))
            )
        self._validate_geometry_constraints(scoped)

    def _validate_geometry_constraints(self, scoped: set[str]) -> None:
        """Check the persistent-geometry tiebreakers used by the CLIP rerank.

        Each entry says "prefer candidate L when the object's measured
        horizontal extent falls in this band". A constraint whose source label
        cannot actually route to L would never fire, so reject it rather than
        let it look configured.
        """
        candidates = {label: group for group in self.clip_rerank_groups for label in group}
        reachable = {**candidates, **self.clip_rerank_routes}
        unknown = set(self.clip_rerank_geometry_constraints) - scoped
        if unknown:
            raise ValueError(
                "perception.label.clip_rerank.persistent_geometry.labels "
                "reference labels outside the configured groups or routes: "
                + ", ".join(sorted(unknown))
            )
        for label, constraints in self.clip_rerank_geometry_constraints.items():
            path = (
                "perception.label.clip_rerank.persistent_geometry.labels."
                f"{label}"
            )
            unknown_keys = set(constraints) - _GEOMETRY_CONSTRAINT_KEYS
            if unknown_keys:
                raise ValueError(
                    f"{path} contains unknown keys: "
                    + ", ".join(sorted(unknown_keys))
                )
            sources = constraints.get("source_labels")
            if sources is not None:
                if not isinstance(sources, list) or not sources:
                    raise ValueError(
                        f"{path}.source_labels must be a non-empty list"
                    )
                unreachable = sorted(
                    source
                    for source in sources
                    if label not in reachable.get(source, ())
                )
                if unreachable:
                    raise ValueError(
                        f"{path}.source_labels cannot reach candidate "
                        f"{label!r}: " + ", ".join(unreachable)
                    )
            extents = {
                key: value
                for key, value in constraints.items()
                if key != "source_labels"
            }
            if not extents or any(
                not math.isfinite(value) or value <= 0.0
                for value in extents.values()
            ):
                raise ValueError(
                    f"{path} extents must be finite positive metres"
                )
            minimum = constraints.get("min_horizontal_extent_m")
            maximum = constraints.get("max_horizontal_extent_m")
            if minimum is not None and maximum is not None and minimum > maximum:
                raise ValueError(
                    f"{path}.min_horizontal_extent_m must not exceed "
                    "max_horizontal_extent_m"
                )

    @classmethod
    def from_config(
        cls,
        perception_config: Mapping[str, Any],
        *,
        profile: Mapping[str, Any],
        detect_period_env: str = "",
        yolo_weights_env: str = "",
        tensor_rt_cache_dir: str = "",
        inference_precision_env: str = "",
        tensor_rt_env: str = "",
    ) -> "PerceptionTuning":
        """Build a validated instance from a `perception` config section.

        `profile` is the resolved full/jetson/lite profile: it supplies the
        model-shaped defaults (cadence, input size, weights) that the config may
        still override key by key. The `*_env` arguments carry the environment
        escape hatches `service.py` honours, applied only where the config is
        silent.
        """
        if not isinstance(perception_config, dict):
            raise ValueError("perception must be a mapping")
        # `from __future__ import annotations` leaves field types as strings,
        # which is exactly the lookup key we want: every scalar knob must be
        # one of the three primitives a TOML value can be coerced to.
        casts = {
            f.name: _CASTS[f.type] for f in fields(cls) if f.type in _CASTS
        }
        values: dict[str, Any] = {}
        for entry in _SCALARS:
            name, section, key, _interval, path = _knob(entry)
            mapping = _section(perception_config, section)
            if key not in mapping or mapping[key] is None:
                continue
            values[name] = _coerce(path, mapping[key], casts[name])

        root = perception_config
        stationary = _section(perception_config, "stationary_refinement")
        values["profile"] = str(profile["name"])
        values["period_s"] = float(
            root["period_s"]
            if root.get("period_s") is not None
            else (detect_period_env or profile["period_s"] or 0.6)
        )
        values["input_size"] = int(
            root["input_size"]
            if root.get("input_size") is not None
            else (profile["input_size"] or 640)
        )
        values["max_detections"] = int(
            root["max_detections"]
            if root.get("max_detections") is not None
            else profile["max_detections"]
        )
        values["inference_precision"] = str(
            root.get("precision")
            or inference_precision_env
            or profile["inference_precision"]
        ).strip().lower()
        values["tensor_rt_mode"] = str(
            root.get("tensor_rt") or tensor_rt_env or profile["tensor_rt"]
        ).strip().lower()
        values["tensor_rt_cache_dir"] = (
            tensor_rt_cache_dir or "/opt/models/runtime-cache"
        )
        values["yolo_weights_path"] = str(
            yolo_weights_env or profile["yolo_weights_path"] or ""
        )
        values["stationary_refinement_input_size"] = int(
            stationary["input_size"]
            if stationary.get("input_size") is not None
            else profile["stationary_input_size"] or 640
        )
        values["stationary_refinement_max_detections"] = int(
            stationary["max_supplemental_detections"]
            if stationary.get("max_supplemental_detections") is not None
            else profile["stationary_max_detections"]
        )

        values.update(cls._label_values(perception_config))
        values["surface_snap_labels"] = (
            _string_list(
                _section(perception_config, "geometry.surface_snap"),
                "perception.geometry.surface_snap",
                "labels",
                allow_empty=True,
            )
            or []
        )
        vocabulary = _section(perception_config, "vocabulary")
        values["classes"] = _string_list(
            vocabulary, "perception.vocabulary", "classes", allow_empty=False
        )
        values["additional_classes"] = _string_list(
            vocabulary,
            "perception.vocabulary",
            "additional_classes",
            allow_empty=True,
        )
        return cls(**values)

    @staticmethod
    def _label_values(perception_config: Mapping[str, Any]) -> dict[str, Any]:
        """Parse the irregular label sections: aliases, rerank scopes, prompts."""
        label = _section(perception_config, "label")
        rerank = _section(perception_config, "label.clip_rerank")
        values: dict[str, Any] = {}

        raw_aliases = label.get("aliases") or {}
        if not isinstance(raw_aliases, dict) or any(
            not isinstance(alias, str) or not isinstance(canonical, str)
            for alias, canonical in raw_aliases.items()
        ):
            raise ValueError(
                "perception.label.aliases must be a string-to-string mapping"
            )
        aliases = {
            alias.strip().lower(): canonical.strip().lower()
            for alias, canonical in raw_aliases.items()
            if alias.strip() and canonical.strip()
            and alias.strip().lower() != canonical.strip().lower()
        }
        if len(aliases) != len(raw_aliases):
            raise ValueError(
                "perception.label.aliases keys and values must not be empty "
                "or self-referential"
            )
        values["label_aliases"] = aliases

        margins: dict[str, float] = {}
        groups: list[list[str]] = []
        raw_groups = rerank.get("groups") or []
        if not isinstance(raw_groups, list):
            raise ValueError(
                "perception.label.clip_rerank.groups must be a list"
            )
        for index, raw_group in enumerate(raw_groups):
            labels, margin = _rerank_scope(
                raw_group,
                f"perception.label.clip_rerank.groups[{index}]",
            )
            groups.append(labels)
            if margin is not None:
                margins.update({label: margin for label in labels})
        values["clip_rerank_groups"] = groups

        routes: dict[str, list[str]] = {}
        raw_routes = rerank.get("routes") or {}
        if not isinstance(raw_routes, dict):
            raise ValueError(
                "perception.label.clip_rerank.routes must map source labels "
                "to label lists"
            )
        for raw_source, raw_route in raw_routes.items():
            if not isinstance(raw_source, str) or not raw_source.strip():
                raise ValueError(
                    "perception.label.clip_rerank.routes source labels must "
                    "be non-empty strings"
                )
            source = raw_source.strip().lower()
            labels, margin = _rerank_scope(
                raw_route,
                f"perception.label.clip_rerank.routes.{source}",
            )
            routes[source] = labels
            if margin is not None:
                margins[source] = margin
        values["clip_rerank_routes"] = routes
        values["clip_rerank_min_margin_by_label"] = margins

        raw_prompts = rerank.get("prompts") or {}
        if not isinstance(raw_prompts, dict) or any(
            not isinstance(label, str)
            or not label.strip()
            or not isinstance(prompts, list)
            or not prompts
            or any(
                not isinstance(prompt, str) or not prompt.strip()
                for prompt in prompts
            )
            for label, prompts in raw_prompts.items()
        ):
            raise ValueError(
                "perception.label.clip_rerank.prompts must map non-empty "
                "labels to non-empty lists of non-empty strings"
            )
        values["clip_rerank_prompts"] = {
            label.strip().lower(): [prompt.strip() for prompt in prompts]
            for label, prompts in raw_prompts.items()
        }

        geometry = _section(
            perception_config, "label.clip_rerank.persistent_geometry"
        )
        unknown = set(geometry) - {"score_bonus", "labels"}
        if unknown:
            raise ValueError(
                "perception.label.clip_rerank.persistent_geometry contains "
                "unknown keys: " + ", ".join(sorted(unknown))
            )
        raw_constraints = geometry.get("labels") or {}
        if not isinstance(raw_constraints, dict):
            raise ValueError(
                "perception.label.clip_rerank.persistent_geometry.labels must "
                "map labels to constraint mappings"
            )
        constraints: dict[str, dict[str, Any]] = {}
        for raw_label, raw_entry in raw_constraints.items():
            if not isinstance(raw_label, str) or not raw_label.strip():
                raise ValueError(
                    "persistent geometry constraint labels must be non-empty "
                    "strings"
                )
            if not isinstance(raw_entry, dict) or not raw_entry:
                raise ValueError(
                    "each persistent geometry label must contain at least one "
                    "constraint"
                )
            entry: dict[str, Any] = {
                key: float(value)
                for key, value in raw_entry.items()
                if key != "source_labels"
            }
            sources = raw_entry.get("source_labels")
            if sources is not None:
                if not isinstance(sources, list) or not sources or any(
                    not isinstance(source, str) or not source.strip()
                    for source in sources
                ):
                    raise ValueError(
                        "persistent geometry source_labels must be a "
                        "non-empty list of non-empty strings"
                    )
                entry["source_labels"] = list(
                    dict.fromkeys(source.strip().lower() for source in sources)
                )
            constraints[raw_label.strip().lower()] = entry
        values["clip_rerank_geometry_constraints"] = constraints
        return values
