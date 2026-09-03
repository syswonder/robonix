# SPDX-License-Identifier: MulanPSL-2.0
"""Hardware-capability probe → perception tier selection.

Scene supports three hardware classes, each with a different semantic
ceiling. This module makes that routing *explicit and testable* instead
of an inline if/elif: it probes which observation kinds are wired (via
the subscribers hub) and reports the tier scene will run. `service.py`
dispatches the detector on the result and logs the plan once at startup.

The three tiers — named by outward semantic capability, not by which
detector happens to implement them:

  metric     RGB-D (+ intrinsics + pose) → ConceptGraphs: object-level
             3D semantics, spatial relations, open-vocabulary queries.
  visual     RGB only → VLM: approximate, region-level visual semantics;
             object positions are coarse (no metric depth back-project).
  geometric  no camera (LiDAR / 2D SLAM only) → no detector. The
             occupancy grid + `goal_near` BFS stay available, but the
             object/relation/scene-graph queries return empty.

Tier selection depends only on which *camera* streams are wired. The
intrinsics / pose / extrinsics slots are recorded for the startup log
and grounding-quality note, but they do NOT gate the tier. Their absence
degrades grounding rather than the tier: ConceptGraphs uses the TF tree when
available and falls back to pose + validated camera-extrinsics contracts. It
uses intrinsics from the live contract when available. Simulator deployments
may opt in to a reviewed
`intrinsics_fallback`; otherwise a missing intrinsics contract still stalls
object output instead of silently guessing K.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Literal, Optional

Tier = Literal["metric", "visual", "geometric"]
Detector = Optional[Literal["concept_graphs", "vlm"]]

# Perception profile: the operator's statement of how much compute this
# deployment gives Scene. It is orthogonal to the tier (which the wiring
# decides): the profile picks the model set and turns object recognition
# off entirely for machines that have no accelerator.
#
#   lite      small models (YOLO-World + MobileSAM + CLIP ViT-B-32), ~4 GB VRAM
#   full      paper-tier models (SAM-L + CLIP ViT-H-14), 16 GB+ VRAM or
#             Jetson Orin/Thor unified memory
#   annotate  no recognition at all; only manual regions/annotations and
#             the geometric queries (occupancy grid, goal_near) — the
#             profile for boards without a usable GPU.
Profile = Literal["lite", "full", "annotate"]
PROFILES: tuple[str, ...] = ("lite", "full", "annotate")
DEFAULT_PROFILE: Profile = "lite"


def resolve_profile(value: Any) -> Profile:
    """Validate a profile name from config/env; blank means the default.

    Raises ValueError naming the accepted values, so a typo in the manifest
    fails at boot instead of silently running the wrong model set.
    """
    name = str(value or "").strip().lower() or DEFAULT_PROFILE
    if name not in PROFILES:
        raise ValueError(
            f"unknown perception profile {name!r}; expected one of {', '.join(PROFILES)}"
        )
    return name  # type: ignore[return-value]


CAMERA_KINDS = frozenset({"rgb", "depth", "intrinsics", "camera_extrinsics"})


def provider_for_kind(kind: str, camera_provider_id: str = "") -> str:
    """Return the configured provider pin for a Scene input kind.

    RGB, depth, intrinsics and extrinsics describe one physical RGB-D camera
    and must therefore resolve from the same provider. Other inputs (mapping,
    lidar, and so on) remain independently discoverable.
    """
    return camera_provider_id if kind in CAMERA_KINDS else ""


@dataclass(frozen=True)
class PerceptionPlan:
    """Resolved perception routing for the current hardware snapshot.

    `tier` names the semantic ceiling; `detector` is which detector
    `service.py` should start (None = geometric, no object detection).
    The booleans record which inputs the probe found so the one-line
    startup log explains itself. `grounding` is a coarse quality label
    for the metric tier — "metric" when intrinsics + a pose contract are
    both wired, "degraded" when one is missing (ConceptGraphs then waits
    for intrinsics unless the deployment configured a reviewed fallback,
    and/or relies on TF); it is "n/a" off the metric tier. It
    reflects wiring at startup, not live data arrival — the detector's own
    logs are authoritative on whether K actually landed or fallback K was used."""

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
        """One-line, self-describing startup log: tier, chosen detector,
        grounding quality, and the raw input flags that drove it."""
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


def plan_perception(hub: Any, profile: Profile = DEFAULT_PROFILE) -> PerceptionPlan:
    """Probe `hub` for wired observation kinds and pick the perception tier.

    Pure with respect to the hub snapshot: reads `hub.has(kind)` only, no
    side effects. Tier is decided by camera availability alone —
    RGB+depth → metric (ConceptGraphs); RGB only → visual (VLM); neither
    → geometric (no detector). `hub.has(kind)` reflects whether a provider
    advertised the contract (the kind was subscribed), matching the gate
    `service.py` used before this module existed.

    `profile` is the operator's compute budget. `annotate` forces the
    geometric tier regardless of wiring: no detector is started, the
    cameras stay subscribed only for the web preview. `lite` and `full`
    do not change the routing; they change which model set the metric
    detector loads.
    """
    has_rgb = hub.has("rgb")
    has_depth = hub.has("depth")
    has_intrinsics = hub.has("intrinsics")
    has_pose = hub.has("pose")
    has_extrinsics = hub.has("camera_extrinsics")

    if profile == "annotate":
        tier: Tier = "geometric"
        detector: Detector = None
    elif has_rgb and has_depth:
        tier, detector = "metric", "concept_graphs"
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


PERCEPTION_KEYS: frozenset[str] = frozenset({
    "profile", "period_s", "confidence_threshold", "max_detections", "concept_graphs",
})


@dataclass(frozen=True)
class PerceptionConfig:
    """The validated `scene.config.perception` block.

    `ignored_keys` lists manifest keys nobody reads; the caller logs them so
    an operator can see that a knob is not wired rather than assume it is.
    """

    profile: Profile
    period_s: Optional[float] = None
    confidence_threshold: Optional[float] = None
    max_detections: Optional[int] = None
    concept_graphs: dict = field(default_factory=dict)
    ignored_keys: tuple[str, ...] = ()


def perception_config(config: dict, env: Optional[dict] = None) -> PerceptionConfig:
    """Parse and validate `config["perception"]`.

    Precedence for the profile: manifest value, then `SCENE_PROFILE` in
    `env` (the process environment by default), then `lite`. Numeric knobs
    are coerced so a manifest typo fails at boot. `concept_graphs` must be
    a mapping of detector knob overrides (same names as the SCENE_CG_*
    environment table) and is passed through untouched.
    """
    env = os.environ if env is None else env
    raw = config.get("perception") or {}
    if not isinstance(raw, dict):
        raise ValueError(f"scene.config.perception must be a mapping, got {type(raw).__name__}")
    cg = raw.get("concept_graphs") or {}
    if not isinstance(cg, dict):
        raise ValueError("scene.config.perception.concept_graphs must be a mapping")

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
        period_s=_num("period_s", float),
        confidence_threshold=_num("confidence_threshold", float),
        max_detections=_num("max_detections", int),
        concept_graphs=dict(cg),
        ignored_keys=tuple(sorted(k for k in raw if k not in PERCEPTION_KEYS)),
    )
