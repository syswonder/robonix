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
        # Above the floor: the floor gate drops tracks whose points lie on it.
        self.points = [(0.0, 0.0, 0.8)] * n
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


def _box(lo, hi):
    class B:
        def get_min_bound(self):
            return lo

        def get_max_bound(self):
            return hi
    return B()


def _entry(name, lo, hi, det=1, pts=100):
    return {"class_name": name, "bbox": _box(lo, hi), "num_detections": det, "n_points": pts}


def test_overlapping_tracks_collapse_across_classes():
    d = _detector()
    cabinet = _entry("cabinet", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0), det=3, pts=500)
    # A small "sink" wholly inside the cabinet: IoU is 0.008, containment 1.0.
    sink = _entry("sink", (0.4, 0.4, 0.4), (0.6, 0.6, 0.6))
    chair = _entry("chair", (3.0, 3.0, 0.0), (3.8, 3.8, 1.0))
    assert d._overlap_ratio(cabinet, sink) == 1.0
    assert d._overlap_ratio(cabinet, chair) == 0.0
    dropped = {"overlap": 0}
    kept = d._suppress_overlaps([cabinet, sink, chair], dropped)
    assert [k["class_name"] for k in kept] == ["cabinet", "chair"]
    assert dropped["overlap"] == 1
    # The better-observed track survives regardless of input order.
    kept = d._suppress_overlaps([sink, cabinet], {"overlap": 0})
    assert [k["class_name"] for k in kept] == ["cabinet"]
    assert _detector(dedup_overlap=0)._suppress_overlaps([cabinet, sink]) == [cabinet, sink]
    print("  [PASS] test_overlapping_tracks_collapse_across_classes")


def test_tracks_lying_on_the_floor_are_dropped():
    import numpy as np
    d = _detector()
    d._dm_names = ["bed"]

    def obj(z):
        o = _obj("u", 0, n=200)
        o.pcd.points = [(0.0, 0.0, float(v)) for v in z]
        return o

    dropped = {"points": 0, "unknown": 0, "floor": 0}
    assert d._to_map_object(obj(np.full(200, -0.09)), dropped) is None      # under the floor
    assert d._to_map_object(obj(np.full(200, 0.02)), dropped) is None       # on the floor
    assert dropped["floor"] == 2
    assert d._to_map_object(obj(np.linspace(0.0, 0.8, 200)), dropped) is not None
    raised = _detector(floor_z_m=-1.5)                                     # Replica: floor at -1.5
    raised._dm_names = d._dm_names
    assert raised._to_map_object(obj(np.full(200, -1.48)), dropped) is None
    assert raised._to_map_object(obj(np.linspace(-1.5, -0.7, 200)), dropped) is not None
    print("  [PASS] test_tracks_lying_on_the_floor_are_dropped")




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


def test_same_class_smear_collapses_into_one_object():
    # A depth-smeared trail: one bin re-registered along the camera ray, each
    # copy smaller and a little further on. None of them contains another.
    d = _detector()
    trail = [_entry("bin", (x, x, 0.4), (x + s, x + s, 0.4 + s), pts=300 - i * 20)
             for i, (x, s) in enumerate([(0.0, 0.30), (0.25, 0.22), (0.45, 0.15), (0.60, 0.08)])]
    assert d._overlap_ratio(trail[0], trail[2]) == 0.0     # no containment at all
    kept = d._suppress_overlaps(list(trail), {"overlap": 0})
    assert len(kept) == 1 and kept[0] is trail[0]          # the best-supported one
    # Two genuinely separate objects of the same class stay separate.
    far = [_entry("chair", (0.0, 0.0, 0.0), (0.5, 0.5, 0.5)),
           _entry("chair", (1.4, 0.0, 0.0), (1.9, 0.5, 0.5))]
    assert len(d._suppress_overlaps(far, {"overlap": 0})) == 2
    # Different classes are left to the containment rule alone.
    d2 = _detector(dedup_overlap=0)
    mixed = [_entry("bin", (0.0, 0.0, 0.4), (0.3, 0.3, 0.7)),
             _entry("vase", (0.05, 0.05, 0.45), (0.2, 0.2, 0.6))]
    assert len(d2._suppress_overlaps(mixed, {"overlap": 0})) == 2
    assert len(_detector(dedup_same_class=0)._suppress_overlaps(list(trail), {"overlap": 0})) == 4
    print("  [PASS] test_same_class_smear_collapses_into_one_object")


def test_boxes_smaller_than_any_real_object_are_dropped():
    d = _detector(min_extent_m=0.08)
    d._dm_names = ["bin"]

    class Box:
        def __init__(self, e):
            self.e = e

        def get_min_bound(self):
            return (0.0, 0.0, 0.0)

        def get_max_bound(self):
            return self.e

    o = _obj("u", 0, n=50)
    o.bbox = Box((0.02, 0.02, 0.07))
    dropped = {"points": 0, "unknown": 0, "floor": 0, "too_small": 0}
    assert d._to_map_object(o, dropped) is None and dropped["too_small"] == 1
    o.bbox = Box((0.02, 0.02, 0.45))          # thin but long: a real object
    assert d._to_map_object(o, dropped) is not None
    print("  [PASS] test_boxes_smaller_than_any_real_object_are_dropped")


def test_lifecycle_overrides_reach_dualmaps_config():
    d = _detector(stable_num=4, active_window_size=30)
    assert d._lifecycle_cfg == {"stable_num": 4, "active_window_size": 30}
    assert _detector()._lifecycle_cfg == {}       # unset: DualMap's own defaults
    print("  [PASS] test_lifecycle_overrides_reach_dualmaps_config")


if __name__ == "__main__":
    test_to_map_object_shapes_local_objects()
    test_classes_file_from_manifest()
    test_unknown_dualmap_keys_fail_at_construction()
    test_plan_routes_metric_tier_to_backend()
    test_keyframe_gate_follows_dualmap_rule()
    test_merged_objects_keep_the_dominant_uid()
    test_overlapping_tracks_collapse_across_classes()
    test_tracks_lying_on_the_floor_are_dropped()
    test_same_class_smear_collapses_into_one_object()
    test_boxes_smaller_than_any_real_object_are_dropped()
    test_lifecycle_overrides_reach_dualmaps_config()
