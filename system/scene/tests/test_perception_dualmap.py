# SPDX-License-Identifier: MulanPSL-2.0
"""DualMap backend: the pure parts that need neither DualMap nor a GPU.

Covers the LocalObject -> map-entry shaping (vocabulary lookup, unknown
filtering, point floor), the manifest vocabulary file, and the perception
plan/config routing that picks the backend.
"""
from __future__ import annotations

import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scene_service.ingest.capabilities import perception_config, plan_perception  # noqa: E402
from scene_service.ingest.perception_dualmap import DualMapDetector  # noqa: E402


class _Pcd:
    def __init__(self, n):
        self.points = [(0.0, 0.0, 0.0)] * n
        self.colors = []

    def has_colors(self):
        return False

    def get_axis_aligned_bounding_box(self):
        return "aabb"


def _obj(uid, class_id, n=10, prob=0.8, seen=3):
    return types.SimpleNamespace(uid=uid, class_id=class_id, pcd=_Pcd(n), bbox=None,
                                 max_prob=prob, observed_num=seen, clip_ft=[0.1, 0.2])


def _detector(**cfg) -> DualMapDetector:
    # Constructor only records callables; nothing is loaded until start().
    return DualMapDetector(
        rgb_fetcher_msg=lambda: None, depth_fetcher_msg=lambda: None,
        camera_info_fetcher=lambda: None, on_detections=None, registry=None,
        dualmap_cfg=cfg or None,
    )


def test_to_map_object_shapes_local_objects():
    d = _detector()
    d._dm_names = ["chair", "table", "unknown"]
    m = d._to_map_object(_obj("u1", 0))
    assert m["id"] == "u1" and m["class_name"] == "chair" and m["bbox"] == "aabb"
    assert m["conf"] == [0.8] and m["num_detections"] == 3 and m["n_points"] == 10
    assert d._to_map_object(_obj("u2", 2)) is None            # unknown dropped by default
    assert d._to_map_object(_obj("u3", 7)) is None            # out-of-range class id -> unknown
    assert d._to_map_object(_obj("u4", 0, n=3)) is None       # below the point floor
    keep = _detector(keep_unknown=True)
    keep._dm_names = d._dm_names
    assert keep._to_map_object(_obj("u2", 2))["class_name"] == "unknown"
    print("  [PASS] test_to_map_object_shapes_local_objects")


def test_classes_file_from_manifest():
    d = _detector(classes=["chair", " table ", ""])
    path = d._write_classes_file()
    try:
        assert open(path).read().split() == ["chair", "table"]
        assert d._classes_file_is_temp
    finally:
        os.remove(path)
    assert _detector()._write_classes_file() is None
    try:
        _detector(classes="chair")._write_classes_file()
    except ValueError:
        pass
    else:
        raise AssertionError("string vocabulary accepted")
    print("  [PASS] test_classes_file_from_manifest")


def test_unknown_dualmap_keys_fail_at_construction():
    for bad in ({"use_fastSAM": True}, {"vocabulary": []}):
        try:
            _detector(**bad)
        except ValueError as e:
            assert "unknown keys" in str(e)
        else:
            raise AssertionError(f"accepted {bad}")
    print("  [PASS] test_unknown_dualmap_keys_fail_at_construction")


def test_plan_routes_metric_tier_to_backend():
    class Hub:
        def has(self, kind):
            return kind in {"rgb", "depth", "intrinsics", "pose"}

    assert plan_perception(Hub()).detector == "concept_graphs"
    assert plan_perception(Hub(), "lite", "dualmap").detector == "dualmap"
    assert "detector=dualmap" in plan_perception(Hub(), "lite", "dualmap").summary()
    assert plan_perception(Hub(), "annotate", "dualmap").detector is None
    try:
        perception_config({"perception": {"backend": "dualmap", "dualmap": {"use_fastSAM": True}}}, env={})
    except ValueError as e:
        assert "unknown keys" in str(e)
    else:
        raise AssertionError("typo under perception.dualmap accepted")
    print("  [PASS] test_plan_routes_metric_tier_to_backend")


if __name__ == "__main__":
    test_to_map_object_shapes_local_objects()
    test_classes_file_from_manifest()
    test_unknown_dualmap_keys_fail_at_construction()
    test_plan_routes_metric_tier_to_backend()
    test_keyframe_gate_follows_dualmap_rule()
    test_merged_objects_keep_the_dominant_uid()


def test_keyframe_gate_follows_dualmap_rule():
    import numpy as np
    d = _detector(keyframe_translation_m=0.1, keyframe_rotation_deg=3.0, keyframe_time_s=1000.0)
    msg = types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1, nanosec=0)))
    pose = np.eye(4)
    assert d._is_keyframe(msg, pose)                       # first frame always maps
    assert not d._is_keyframe(msg, pose)                   # same message again: skipped
    msg2 = types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=2, nanosec=0)))
    near = np.eye(4); near[0, 3] = 0.05
    assert not d._is_keyframe(msg2, near)                  # 5 cm, no rotation: not a keyframe
    far = np.eye(4); far[0, 3] = 0.2
    assert d._is_keyframe(msg2, far)                       # 20 cm: keyframe
    msg3 = types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=3, nanosec=0)))
    turned = far.copy(); a = np.radians(5.0)
    turned[:3, :3] = [[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]
    assert d._is_keyframe(msg3, turned)                    # 5 degrees: keyframe
    assert d._skipped_frames == 1
    print("  [PASS] test_keyframe_gate_follows_dualmap_rule")


def test_merged_objects_keep_the_dominant_uid():
    d = _detector()
    a = types.SimpleNamespace(uid="A", observations=[object(), object(), object()], is_merged=False)
    b = types.SimpleNamespace(uid="B", observations=[object()], is_merged=False)
    merged = types.SimpleNamespace(uid="NEW", observations=a.observations + b.observations, is_merged=True)
    untouched = types.SimpleNamespace(uid="C", observations=[object()], is_merged=False)
    d._lm = types.SimpleNamespace(local_map=[merged, untouched])
    d._keep_merged_uids([a, b, untouched])
    assert merged.uid == "A" and merged.is_merged is False and untouched.uid == "C"
    print("  [PASS] test_merged_objects_keep_the_dominant_uid")
