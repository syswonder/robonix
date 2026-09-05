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


def test_lifecycle_overrides_reach_dualmaps_config():
    d = _detector(stable_num=4, active_window_size=30, downsample_voxel_size=0.05)
    assert d._lifecycle_cfg == {"stable_num": 4, "active_window_size": 30,
                                "downsample_voxel_size": 0.05}
    assert _detector()._lifecycle_cfg == {}       # unset: DualMap's own defaults
    print("  [PASS] test_lifecycle_overrides_reach_dualmaps_config")


def test_floor_height_comes_from_the_shared_setting():
    # Replica's floor is at -1.51 m and reaches Scene as the shared
    # perception setting; a backend-private default of 0 dropped every object
    # below the world origin as "floor noise".
    import numpy as np
    d = _detector()
    d._dm_names = ["chair"]
    d.cfg = {"floor_z_m": -1.51}
    assert d._floor_z_m == -1.51
    o = _obj("u", 0, n=200)
    o.pcd.points = [(0.0, 0.0, float(v)) for v in np.linspace(-1.5, -0.9, 200)]
    assert d._to_map_object(o) is not None            # a chair standing on that floor
    explicit = _detector(floor_z_m=0.0)
    explicit.cfg = {"floor_z_m": -1.51}
    assert explicit._floor_z_m == 0.0                 # the backend's own value wins
    print("  [PASS] test_floor_height_comes_from_the_shared_setting")


def test_global_objects_shape_like_local_ones():
    # A GlobalObject carries uid / class_id / pcd / bbox / clip_ft but none of
    # the local-map bookkeeping (max_prob, observed_num, is_stable), so the map
    # entry has to survive their absence — both maps are exported together.
    d = _detector()
    d._dm_names = ["chair"]
    g = types.SimpleNamespace(uid="g1", class_id=0, pcd=_Pcd(40), bbox=None, clip_ft=[0.3])
    m = d._to_map_object(g)
    assert m["id"] == "g1" and m["class_name"] == "chair"
    assert m["num_detections"] == 1 and m["conf"] == [0.5] and m["stable"] is True
    assert m["bbox"] == "aabb"          # fell back to the point cloud's own box
    print("  [PASS] test_global_objects_shape_like_local_ones")


if __name__ == "__main__":
    test_to_map_object_shapes_local_objects()
    test_classes_file_from_manifest()
    test_unknown_dualmap_keys_fail_at_construction()
    test_plan_routes_metric_tier_to_backend()
    test_keyframe_gate_follows_dualmap_rule()
    test_merged_objects_keep_the_dominant_uid()
    test_tracks_lying_on_the_floor_are_dropped()
    test_lifecycle_overrides_reach_dualmaps_config()
    test_global_objects_shape_like_local_ones()
    test_floor_height_comes_from_the_shared_setting()
