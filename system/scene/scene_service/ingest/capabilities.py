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

from dataclasses import dataclass
from typing import Any, Literal, Optional

Tier = Literal["metric", "visual", "geometric"]
Detector = Optional[Literal["concept_graphs", "vlm"]]


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
            f"tier={self.tier} detector={self.detector or 'none'} "
            f"grounding={self.grounding} inputs=[{inputs}]"
        )


def plan_perception(hub: Any) -> PerceptionPlan:
    """Probe `hub` for wired observation kinds and pick the perception tier.

    Pure with respect to the hub snapshot: reads `hub.has(kind)` only, no
    side effects. Tier is decided by camera availability alone —
    RGB+depth → metric (ConceptGraphs); RGB only → visual (VLM); neither
    → geometric (no detector). `hub.has(kind)` reflects whether a provider
    advertised the contract (the kind was subscribed), matching the gate
    `service.py` used before this module existed.
    """
    has_rgb = hub.has("rgb")
    has_depth = hub.has("depth")
    has_intrinsics = hub.has("intrinsics")
    has_pose = hub.has("pose")
    has_extrinsics = hub.has("camera_extrinsics")

    if has_rgb and has_depth:
        tier: Tier = "metric"
        detector: Detector = "concept_graphs"
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
    )
