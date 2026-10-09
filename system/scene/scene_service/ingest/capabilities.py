# SPDX-License-Identifier: MulanPSL-2.0
"""Perception routing: which tier and detector Scene runs, from the wired inputs.

  metric     RGB + depth -> an open-vocabulary 3D mapper (ConceptGraphs or DualMap)
  visual     RGB only -> VLM detections with coarse positions
  geometric  no camera -> no detector; occupancy queries still work

Only the camera streams decide the tier; intrinsics and pose affect grounding.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Literal, Optional

Tier = Literal["metric", "visual", "geometric"]
Detector = Optional[Literal["concept_graphs", "dualmap", "vlm"]]

# Compute budget: lite (small models, ~4 GB VRAM), full (SAM-L + CLIP ViT-H-14,
# 16 GB+), annotate (no recognition, for boards without a usable GPU).
Profile = Literal["lite", "full", "annotate"]
PROFILES: tuple[str, ...] = ("lite", "full", "annotate")
DEFAULT_PROFILE: Profile = "lite"

# The metric-tier mapper.
Backend = Literal["concept_graphs", "dualmap"]
BACKENDS: tuple[str, ...] = ("concept_graphs", "dualmap")

# With no backend configured, DualMap is used when the image carries it
# (docker/Dockerfile.dualmap), otherwise ConceptGraphs.
FALLBACK_BACKEND: Backend = "concept_graphs"
PREFERRED_BACKEND: Backend = "dualmap"
DEFAULT_BACKEND: Backend = FALLBACK_BACKEND


def dualmap_available() -> bool:
    """Whether the image has a DualMap checkout; checked by path, since importing it loads torch."""
    root = os.environ.get("SCENE_DUALMAP_ROOT") or "/opt/dualmap"
    return os.path.isdir(os.path.join(root, "utils"))


def default_backend() -> Backend:
    """The backend to use when config and environment both stay silent."""
    return PREFERRED_BACKEND if dualmap_available() else FALLBACK_BACKEND


# Keys accepted under `scene.config.perception.dualmap`; documented in config.spec.
DUALMAP_KEYS: frozenset[str] = frozenset({
    "classes", "keep_unknown", "use_fastsam", "device",
    "keyframe_translation_m", "keyframe_rotation_deg", "keyframe_time_s",
    "merge_every_keyframes", "stable_only", "min_observations",
    "floor_gate", "floor_z_m",
    "stable_num", "active_window_size", "max_pending_count",
    "downsample_voxel_size", "sim_threshold", "merge_sim_threshold",
    "global_map",
})


def resolve_backend(value: Any) -> Backend:
    """Validate a backend name; blank means default_backend(), unknown raises."""
    name = str(value or "").strip().lower()
    if not name:
        return default_backend()
    if name not in BACKENDS:
        raise ValueError(
            f"unknown perception backend {value!r}; expected one of {', '.join(BACKENDS)}"
        )
    return name  # type: ignore[return-value]


def resolve_profile(value: Any) -> Profile:
    """Validate a profile name; blank means lite, unknown raises."""
    name = str(value or "").strip().lower() or DEFAULT_PROFILE
    if name not in PROFILES:
        raise ValueError(
            f"unknown perception profile {name!r}; expected one of {', '.join(PROFILES)}"
        )
    return name  # type: ignore[return-value]


CAMERA_KINDS = frozenset({"rgb", "depth", "intrinsics", "camera_extrinsics"})


def provider_for_kind(kind: str, camera_provider_id: str = "") -> str:
    """The camera inputs share one pinned provider; other kinds are discovered freely."""
    return camera_provider_id if kind in CAMERA_KINDS else ""


@dataclass(frozen=True)
class PerceptionPlan:
    """Resolved routing; ``detector`` None means no object detection. The
    booleans record the wired inputs for the startup log."""

    tier: Tier
    detector: Detector
    has_rgb: bool
    has_depth: bool
    has_intrinsics: bool
    has_pose: bool
    has_extrinsics: bool
    profile: Profile = DEFAULT_PROFILE

    @property
    def grounding(self) -> Literal["metric", "degraded", "n/a"]:
        if self.detector != "concept_graphs":
            return "n/a"
        return "metric" if (self.has_intrinsics and self.has_pose) else "degraded"

    def summary(self) -> str:
        inputs = ",".join(
            name
            for name, present in (
                ("rgb", self.has_rgb),
                ("depth", self.has_depth),
                ("intrinsics", self.has_intrinsics),
                ("pose", self.has_pose),
                ("extrinsics", self.has_extrinsics),
            )
            if present
        ) or "none"
        return (
            f"profile={self.profile} tier={self.tier} "
            f"detector={self.detector or 'none'} "
            f"grounding={self.grounding} inputs=[{inputs}]"
        )


def plan_perception(
    hub: Any, profile: Profile = DEFAULT_PROFILE, backend: Backend = DEFAULT_BACKEND,
) -> PerceptionPlan:
    """Pick the tier from the wired camera streams; `annotate` forces geometric."""
    has_rgb = hub.has("rgb")
    has_depth = hub.has("depth")
    has_intrinsics = hub.has("intrinsics")
    has_pose = hub.has("pose")
    has_extrinsics = hub.has("camera_extrinsics")

    if profile == "annotate":
        tier: Tier = "geometric"
        detector: Detector = None
    elif has_rgb and has_depth:
        tier, detector = "metric", backend
    elif has_rgb:
        tier, detector = "visual", "vlm"
    else:
        tier, detector = "geometric", None

    return PerceptionPlan(
        tier=tier,
        detector=detector,
        has_rgb=has_rgb,
        has_depth=has_depth,
        has_intrinsics=has_intrinsics,
        has_pose=has_pose,
        has_extrinsics=has_extrinsics,
        profile=profile,
    )


# Mirrors `_CFG_DEFAULTS` in perception_concept_graphs without importing torch;
# test_capabilities keeps the two in sync.
CONCEPT_GRAPHS_KEYS: frozenset[str] = frozenset({
    "assoc_feat_weight",
    "assoc_geo_weight",
    "assoc_threshold",
    "assoc_voxel_size_m",
    "association",
    "cross_class_centroid_max_m",
    "cross_class_iou_thresh",
    "cross_class_merge_interval_ticks",
    "cross_class_overlap_thresh",
    "dbscan_eps",
    "dbscan_min_points",
    "dbscan_remove_noise",
    "denoise_interval_ticks",
    "downsample_voxel_size",
    "feature_area_ratio",
    "feature_bank_size",
    "floor_z_m",
    "label_vote",
    "match_method",
    "max_merge_dist_m",
    "merge_overlap_interval_ticks",
    "merge_overlap_thresh",
    "merge_text_sim_thresh",
    "merge_threshold",
    "merge_visual_sim_thresh",
    "min_points_threshold",
    "obj_min_detections",
    "obj_min_points",
    "obj_pcd_max_points",
    "per_detection_dbscan",
    "phys_bias",
    "representative_by_text",
    "same_class_merge_dist_m",
    "same_class_merge_interval_ticks",
    "spatial_sim_type",
    "visibility_depth_margin_m",
    "visibility_min_clear_fraction",
    "visibility_min_clear_samples",
    "visibility_miss_ticks",
})


PERCEPTION_KEYS: frozenset[str] = frozenset({
    "profile", "backend", "period_s", "confidence_threshold", "max_detections",
    "concept_graphs", "dualmap",
})


@dataclass(frozen=True)
class PerceptionConfig:
    """The validated `scene.config.perception` block; `ignored_keys` are logged."""

    profile: Profile
    backend: Backend = DEFAULT_BACKEND
    period_s: Optional[float] = None
    confidence_threshold: Optional[float] = None
    max_detections: Optional[int] = None
    concept_graphs: dict = field(default_factory=dict)
    dualmap: dict = field(default_factory=dict)
    ignored_keys: tuple[str, ...] = ()


def perception_config(config: dict, env: Optional[dict] = None) -> PerceptionConfig:
    """Parse and validate `config["perception"]`. Profile and backend fall back to
    `SCENE_PROFILE` / `SCENE_PERCEPTION_BACKEND` in `env`; unknown backend keys
    and non-numeric numbers raise at boot."""
    env = os.environ if env is None else env
    raw = config.get("perception") or {}
    if not isinstance(raw, dict):
        raise ValueError(f"scene.config.perception must be a mapping, got {type(raw).__name__}")
    cg = _backend_block(raw, "concept_graphs", CONCEPT_GRAPHS_KEYS)
    dm = _backend_block(raw, "dualmap", DUALMAP_KEYS)

    def _num(key: str, cast):
        value = raw.get(key)
        if value is None or value == "":
            return None
        try:
            return cast(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"scene.config.perception.{key}={value!r} is not a number") from exc

    return PerceptionConfig(
        profile=resolve_profile(raw.get("profile") or env.get("SCENE_PROFILE") or ""),
        backend=resolve_backend(raw.get("backend") or env.get("SCENE_PERCEPTION_BACKEND") or ""),
        period_s=_num("period_s", float),
        confidence_threshold=_num("confidence_threshold", float),
        max_detections=_num("max_detections", int),
        concept_graphs=dict(cg),
        dualmap=dict(dm),
        ignored_keys=tuple(sorted(k for k in raw if k not in PERCEPTION_KEYS)),
    )


def _backend_block(raw: dict, name: str, keys: frozenset[str]) -> dict:
    """One backend's override mapping, rejecting keys it does not read."""
    block = raw.get(name) or {}
    if not isinstance(block, dict):
        raise ValueError(f"scene.config.perception.{name} must be a mapping")
    unknown = sorted(k for k in block if k not in keys)
    if unknown:
        raise ValueError(
            f"scene.config.perception.{name} has unknown keys {unknown}; "
            f"accepted: {', '.join(sorted(keys))}"
        )
    return block
