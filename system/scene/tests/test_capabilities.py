# SPDX-License-Identifier: MulanPSL-2.0
"""Unit tests for the perception-tier capability probe.

Pure Python — no ROS2 / atlas / hub. A fake hub stands in for
SubscribersHub: the probe only ever calls `hub.has(kind)`.
"""
import importlib.util
import os
import sys

# Load capabilities.py directly by path, bypassing scene_service.ingest's
# __init__ — that eagerly imports the numpy-heavy detectors, which the
# probe itself doesn't need. Keeps this test pure Python (no deps).
_CAP_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scene_service", "ingest", "capabilities.py"
)
_spec = importlib.util.spec_from_file_location("scene_capabilities", _CAP_PATH)
_mod = importlib.util.module_from_spec(_spec)
# Register before exec so dataclass string-annotation resolution can find
# the module via sys.modules (required on Python 3.9; harmless on 3.10+).
sys.modules["scene_capabilities"] = _mod
_spec.loader.exec_module(_mod)
plan_perception = _mod.plan_perception
provider_for_kind = _mod.provider_for_kind


class _FakeHub:
    """Minimal hub stub: `has(kind)` is True iff kind is in the set."""

    def __init__(self, kinds):
        self._kinds = set(kinds)

    def has(self, kind):
        return kind in self._kinds


def test_metric_tier_full():
    # RGB-D + intrinsics + pose → metric tier, ConceptGraphs, metric grounding.
    plan = plan_perception(_FakeHub(
        ["rgb", "depth", "intrinsics", "pose", "camera_extrinsics"]
    ))
    assert plan.tier == "metric"
    assert plan.detector == "concept_graphs"
    assert plan.grounding == "metric"
    print("  [PASS] test_metric_tier_full")


def test_metric_tier_degraded():
    # RGB-D but no intrinsics/pose contract → still metric tier (depth is
    # present), but grounding is flagged degraded (waiting for K / tf2 pose).
    plan = plan_perception(_FakeHub(["rgb", "depth"]))
    assert plan.tier == "metric"
    assert plan.detector == "concept_graphs"
    assert plan.grounding == "degraded"
    print("  [PASS] test_metric_tier_degraded")


def test_visual_tier():
    # RGB only → visual tier, VLM detector, grounding n/a (not the metric path).
    plan = plan_perception(_FakeHub(["rgb", "pose"]))
    assert plan.tier == "visual"
    assert plan.detector == "vlm"
    assert plan.grounding == "n/a"
    print("  [PASS] test_visual_tier")


def test_geometric_tier():
    # No camera (lidar / occupancy only) → geometric tier, no detector.
    plan = plan_perception(_FakeHub(["lidar2d", "occupancy_grid", "pose"]))
    assert plan.tier == "geometric"
    assert plan.detector is None
    assert plan.grounding == "n/a"
    print("  [PASS] test_geometric_tier")


def test_summary_is_self_describing():
    # The startup log line names tier, detector, grounding, and inputs.
    s = plan_perception(_FakeHub(["rgb", "depth", "intrinsics", "pose"])).summary()
    assert "tier=metric" in s
    assert "detector=concept_graphs" in s
    assert "grounding=metric" in s
    assert "rgb" in s and "depth" in s and "intrinsics" in s
    # Empty-input case reads "none", not an empty bracket.
    assert "inputs=[none]" in plan_perception(_FakeHub([])).summary()
    print("  [PASS] test_summary_is_self_describing")


def test_rgbd_camera_provider_is_bound_as_one_group():
    provider = "front_rgbd"
    for kind in ("rgb", "depth", "intrinsics", "camera_extrinsics"):
        assert provider_for_kind(kind, provider) == provider
    for kind in ("pose", "odom", "occupancy_grid", "lidar3d"):
        assert provider_for_kind(kind, provider) == ""
    print("  [PASS] test_rgbd_camera_provider_is_bound_as_one_group")


if __name__ == "__main__":
    print("Running capability-probe unit tests...\n")
    test_metric_tier_full()
    test_metric_tier_degraded()
    test_visual_tier()
    test_geometric_tier()
    test_summary_is_self_describing()
    test_rgbd_camera_provider_is_bound_as_one_group()
    print("\nAll tests passed!")
