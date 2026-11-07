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


def test_keyframe_indices_and_periodic_merge_publication(monkeypatch):
    import numpy as np
    from scene_service.ingest import perception_dualmap

    monkeypatch.setattr(perception_dualmap, "_image_msg_to_bgr", lambda _: np.zeros((2, 2, 3), dtype=np.uint8))
    monkeypatch.setattr(perception_dualmap, "_depth_msg_to_metres", lambda _: np.ones((2, 2), dtype=np.float32))
    for populated, keyframes, expected_merges in ((False, 19, 0), (True, 19, 1), (True, 18, 0)):
        detector = _detector()
        detector._world_frame_fn = lambda: "map"
        detector._map_frame = "map"
        detector._rgb_msg = detector._depth_msg = lambda: object()
        detector._cam_info = lambda: types.SimpleNamespace(fx=1., fy=1., cx=0., cy=0.)
        detector._build_camera_to_map_transform = lambda **_: np.eye(4)
        detector._is_keyframe = lambda *_: True
        indices = []
        detector._dm_data_input = lambda **data: indices.append(data["idx"])
        detector._dm = types.SimpleNamespace(
            set_data_input=lambda _: None, process_detections=lambda: None,
            calculate_observations=lambda: None, get_curr_observations=lambda: [],
            update_state=lambda: None, update_data=lambda: None,
        )
        merges, publications = [], []
        tracks = [_obj("track", 0, n=3)] if populated else []
        def merge():
            if not tracks:
                raise RuntimeError("stack expects a non-empty TensorList")
            merges.append(True)
        detector._lm = types.SimpleNamespace(
            local_map=tracks, set_curr_idx=indices.append,
            process_observations=lambda _: None, get_global_observations=lambda: [],
            clear_global_observations=lambda: None, merge_local_map=merge,
        )
        detector._gm = None
        detector._dm_names = ["chair"]
        detector._keyframes = keyframes
        # Skipped frames must not age tracks in DualMap's active window.
        detector._tick_idx = keyframes * 8
        detector._project_to_registry = lambda: publications.append(True)
        detector._tick_locked()
        assert detector._consecutive_failures == 0
        assert len(merges) == expected_merges
        assert publications == [True]
        assert indices == [keyframes, keyframes]


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
    d = _detector(floor_gate=True)
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
    raised = _detector(floor_gate=True, floor_z_m=-1.5)                    # Replica: floor at -1.5
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


def test_world_frame_change_discards_old_tracks_before_transform_recovers(monkeypatch):
    import asyncio
    import numpy as np
    from scene_service.ingest import perception_dualmap
    from scene_service.state.object_registry import ObjectRegistry

    registry = ObjectRegistry()
    frame = ["odom"]
    detector = _detector()
    detector._registry = registry
    detector._world_frame_fn = lambda: frame[0]
    detector._map_frame = "odom"
    loop = asyncio.new_event_loop()
    detector._asyncio_loop = loop
    snapshot = {
        "uuid": "old-track", "cls": "cabinet", "x": 1.0, "y": 2.0,
        "z": 0.5, "yaw": 0.0, "size_x": 0.3, "size_y": 0.3,
        "size_z": 0.3, "confidence": 0.9,
    }
    try:
        loop.run_until_complete(detector._apply_snapshot([snapshot]))
        (cabinet,) = registry.all_objects()
        assert cabinet.pose.frame_id == "odom"
        detector._lm = types.SimpleNamespace(local_map=["old-track"])
        detector._gm = types.SimpleNamespace(global_map=["old-track"])
        detector._map_objects = ["old-track"]
        detector._all_objects = ["old-track"]
        detector._held_uuids = {"old-track"}
        detector._last_kf_pose = np.eye(4)
        detector._last_frame_key = (1, 0)
        detector._rgb_msg = lambda: object()
        detector._depth_msg = lambda: object()
        detector._cam_info = lambda: types.SimpleNamespace(fx=1.0, fy=1.0)
        detector._build_camera_to_map_transform = lambda **kwargs: None
        monkeypatch.setattr(perception_dualmap, "_image_msg_to_bgr",
                            lambda _: np.zeros((1, 1, 3), dtype=np.uint8))
        monkeypatch.setattr(perception_dualmap, "_depth_msg_to_metres",
                            lambda _: np.ones((1, 1), dtype=np.float32))

        detector._tick_locked()
        loop.run_until_complete(asyncio.sleep(0))
        assert detector._lm.local_map == ["old-track"]
        assert detector._gm.global_map == ["old-track"]
        assert detector._held_uuids == {"old-track"} and not cabinet.missing

        frame[0] = "map"
        detector._tick_locked()
        loop.run_until_complete(asyncio.sleep(0))

        assert detector._lm.local_map == []
        assert detector._gm.global_map == []
        assert detector._map_objects == [] and detector._all_objects == []
        assert not detector._held_uuids
        assert detector._last_kf_pose is None and detector._last_frame_key is None
        assert cabinet.missing
        assert cabinet.pose.frame_id == "odom"

        loop.run_until_complete(detector._apply_snapshot([
            {**snapshot, "uuid": "new-track"},
        ]))
        assert list(registry.all_objects()) == [cabinet]
        assert not cabinet.missing and cabinet.pose.frame_id == "map"
    finally:
        loop.close()


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


def test_queued_snapshot_cannot_relabel_old_coordinates_after_frame_change():
    import asyncio
    import numpy as np
    from scene_service.state.object_registry import ObjectRegistry

    detector = _detector()
    detector._registry = ObjectRegistry()
    frame = ["odom"]
    detector._world_frame_fn = lambda: frame[0]
    detector._map_frame = "odom"
    detector._map_objects = [{
        "id": "track", "class_name": "cabinet", "conf": [0.9],
        "pcd": types.SimpleNamespace(points=np.array([
            [0.0, 0.0, 0.5], [0.0, 0.3, 0.5],
            [0.3, 0.0, 0.5], [0.3, 0.3, 0.5],
        ])),
    }]
    loop = asyncio.new_event_loop()
    detector._asyncio_loop = loop
    try:
        detector._project_to_registry()
        frame[0] = "map"
        loop.run_until_complete(asyncio.sleep(0))
        assert not list(detector._registry.all_objects())

        detector._map_frame = "map"
        detector._project_to_registry()
        loop.run_until_complete(asyncio.sleep(0))
        (cabinet,) = detector._registry.all_objects()
        assert cabinet.pose.frame_id == "map" and cabinet.bbox.frame_id == "map"
    finally:
        loop.close()


def test_lifecycle_overrides_reach_dualmaps_config():
    d = _detector(stable_num=4, active_window_size=30, downsample_voxel_size=0.05,
                  merge_sim_threshold=0.3)
    assert d._lifecycle_cfg == {"stable_num": 4, "active_window_size": 30,
                                "downsample_voxel_size": 0.05, "merge_sim_threshold": 0.3}
    assert _detector()._lifecycle_cfg == {}       # unset: DualMap's own defaults
    print("  [PASS] test_lifecycle_overrides_reach_dualmaps_config")


def test_floor_height_comes_from_the_shared_setting():
    # Replica's floor is at -1.51 m and reaches Scene as the shared
    # perception setting; a backend-private default of 0 dropped every object
    # below the world origin as "floor noise".
    import numpy as np
    d = _detector(floor_gate=True)
    d._dm_names = ["chair"]
    d.cfg = {"floor_z_m": -1.51}
    assert d._floor_z_m == -1.51
    o = _obj("u", 0, n=200)
    o.pcd.points = [(0.0, 0.0, float(v)) for v in np.linspace(-1.5, -0.9, 200)]
    assert d._to_map_object(o) is not None            # a chair standing on that floor
    explicit = _detector(floor_gate=True, floor_z_m=0.0)
    explicit.cfg = {"floor_z_m": -1.51}
    assert explicit._floor_z_m == 0.0                 # the backend's own value wins
    print("  [PASS] test_floor_height_comes_from_the_shared_setting")


def test_floor_gate_is_off_unless_asked():
    d = _detector()
    d._dm_names = ["rug"]
    o = _obj("u", 0, n=200)
    o.pcd.points = [(0.0, 0.0, 0.02)] * 200        # a rug: 2 cm above the floor
    assert d._to_map_object(o) is not None
    assert _detector(floor_gate=True)._floor_gate is True
    print("  [PASS] test_floor_gate_is_off_unless_asked")


def test_global_map_is_opt_in():
    assert _detector()._global_map is False
    assert _detector(global_map=True)._global_map is True
    print("  [PASS] test_global_map_is_opt_in")


def test_global_objects_shape_like_local_ones():
    # A GlobalObject carries uid / class_id / pcd / bbox / clip_ft but none of
    # the local-map bookkeeping (max_prob, observed_num, is_stable), so the map
    # entry has to survive their absence — both maps are exported together.
    d = _detector()
    d._dm_names = ["chair"]
    g = types.SimpleNamespace(uid="g1", class_id=0, pcd=_Pcd(40), bbox=None, clip_ft=[0.3])
    m = d._to_map_object(g)
    assert m["id"] == "g1" and m["class_name"] == "chair"
    # 0.0, not the old 0.5 fallback: a GlobalObject carries no `max_prob`,
    # no `class_probs` and no observation score, so nothing has said
    # anything about this label yet and 0.0 is the honest answer. The
    # fallback changed with the confidence fix; this assertion did not.
    assert m["num_detections"] == 1 and m["conf"] == [0.0] and m["stable"] is True
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
    test_global_map_is_opt_in()
    test_floor_gate_is_off_unless_asked()
    test_floor_height_comes_from_the_shared_setting()


def _track(cls, lo, hi, n):
    """One map entry shaped like _to_map_object's output, spanning lo..hi."""
    import numpy as np
    o = _obj("u", 0, n=n)
    xs = np.linspace(lo[0], hi[0], n)
    ys = np.linspace(lo[1], hi[1], n)
    zs = np.linspace(lo[2], hi[2], n)
    o.pcd.points = [(float(a), float(b), float(c)) for a, b, c in zip(xs, ys, zs)]
    return {"class_name": cls, "n_points": n, "pcd": o.pcd}


def test_slices_of_the_floor_are_not_furniture():
    # The floor arrives as a millimetre-thick slab carrying more points than any
    # real object, so nothing that ranks by evidence can reject it.
    d = _detector()
    dropped = {"ground_slab": 0}
    kept = d._drop_ground_slabs([
        _track("desk", (-1, -1, -0.090), (1, 1, -0.089), 1900),   # the floor
        _track("keyboard", (0, 0, 0.640), (0.4, 0.2, 0.653), 50),  # flat, but on a desk
        _track("desk", (-1, -1, 0.61), (1, 1, 0.65), 800),         # a table top
    ], dropped)
    assert dropped["ground_slab"] == 1
    assert [t["class_name"] for t in kept] == ["keyboard", "desk"]
    # The floor plus some table legs: 30 cm tall, so not a slab, but most of
    # it sits under the floor. A rug (median above the floor) stays.
    dropped = {"ground_slab": 0}
    floor_blob = _track("desk", (-2, -2, -0.09), (1.5, 2.5, 0.20), 3765)
    # nine points in ten lie on the floor; the rest climb a table leg
    floor_blob["pcd"].points = [(x, y, -0.09 if i % 10 else 0.20)
                                for i, (x, y, _) in enumerate(floor_blob["pcd"].points)]
    kept = d._drop_ground_slabs([
        floor_blob,
        _track("rug", (0, 0, 0.00), (1.5, 1.0, 0.03), 400),
    ], dropped)
    assert dropped["ground_slab"] == 1 and [t["class_name"] for t in kept] == ["rug"]
    print("  [PASS] test_slices_of_the_floor_are_not_furniture")


def test_a_fragment_is_not_a_second_object_of_its_class():
    # Scale is judged against the class's own members, not a table of sizes.
    d = _detector()
    dropped = {"class_outlier": 0}
    group = [_track("desk", (0, 0, 0.6), (1.6, 1.5, 0.65), 2300),
             _track("desk", (4, 4, 0.6), (5.5, 5.4, 0.65), 1500),
             _track("desk", (9, 9, 0.2), (9.11, 9.10, 0.45), 20)]   # a smear copy
    kept = d._drop_class_outliers(list(group), dropped)
    assert dropped["class_outlier"] == 1 and len(kept) == 2
    # Two members are not a scale: with nothing to judge against, both stay.
    dropped = {"class_outlier": 0}
    assert len(d._drop_class_outliers(group[:1] + group[2:], dropped)) == 2
    print("  [PASS] test_a_fragment_is_not_a_second_object_of_its_class")


def test_one_table_in_pieces_merges_but_two_tables_do_not():
    d = _detector()
    dropped = {"overlapping": 0}
    # Same table from two sides: the smaller box is almost entirely inside.
    kept = d._absorb_overlapping([
        _track("desk", (-5.19, -3.31, 0.61), (-3.83, -1.47, 0.65), 2334),
        _track("desk", (-5.08, -3.23, 0.61), (-3.80, -1.64, 0.65), 1545),
    ], dropped)
    assert dropped["overlapping"] == 1 and len(kept) == 1
    # Two tables pushed together: they touch, so neither absorbs the other.
    dropped = {"overlapping": 0}
    kept = d._absorb_overlapping([
        _track("desk", (-5.19, -3.31, 0.61), (-3.83, -1.47, 0.65), 2334),
        _track("desk", (-4.22, -2.19, 0.61), (-2.62, -0.70, 0.65), 1536),
    ], dropped)
    assert dropped["overlapping"] == 0 and len(kept) == 2
    print("  [PASS] test_one_table_in_pieces_merges_but_two_tables_do_not")


def test_a_long_thin_copy_is_absorbed_even_on_little_overlap():
    # The two tests are alternatives, not a single rule: a smear copy of a
    # narrow object sits inside the original while overlapping well under half
    # of it. Requiring overlap alone let these back in (duplicates 7 -> 16).
    d = _detector()
    dropped = {"overlapping": 0}
    kept = d._absorb_overlapping([
        _track("plant", (0.0, 0.0, 0.0), (0.60, 0.60, 1.60), 776),
        _track("plant", (0.28, 0.28, 0.70), (0.34, 0.34, 0.90), 14),
    ], dropped)
    assert dropped["overlapping"] == 1 and len(kept) == 1
    print("  [PASS] test_a_long_thin_copy_is_absorbed_even_on_little_overlap")


# ── operator hooks reach both maps ───────────────────────────────────────────
# DualMap keeps two: the local map holds what the robot is looking at, and a
# track that becomes stable is *promoted* to the global map, keeping its uid.
# `_tick_locked` rebuilds the scene from the union. So an operator hook that
# edits only the local map is a no-op for anything already promoted: delete an
# object and the next tick reinstates it from the global map; flush and the
# promoted objects stay. `global_map` is an opt-in setting, so this is only
# reachable on a deployment that turned it on -- which is exactly the one where
# nobody would think to look.

class _Map:
    def __init__(self, objs):
        self.local_map = list(objs)
        self.global_map = list(objs)


def _promoted_detector(uids):
    p = _detector()
    objs = [_obj(u, 0) for u in uids]
    p._lm = _Map(objs)
    p._gm = _Map(objs)
    p._map_objects = [{"id": u} for u in uids]
    p._uuid_to_oid = {u: f"obj_{u}" for u in uids}
    return p


def _run(coro):
    import asyncio
    return asyncio.run(coro)


def test_delete_reaches_the_global_map():
    p = _promoted_detector(["a", "b"])
    _run(p.delete_object("obj_a"))
    assert [o.uid for o in p._lm.local_map] == ["b"]
    assert [o.uid for o in p._gm.global_map] == ["b"], (
        "a promoted track left in the global map is reinstated on the next "
        "tick, so the delete does not stick")
    assert [o["id"] for o in p._map_objects] == ["b"]


def test_flush_empties_the_global_map_too():
    p = _promoted_detector(["a", "b"])
    _run(p.reset_derived_state())
    assert p._lm.local_map == []
    assert p._gm.global_map == [], "a flush that keeps objects is not a flush"
    assert p._map_objects == []


def test_operator_hooks_work_without_a_global_map():
    """The default: `global_map` off, `_gm` is None, nothing to clear."""
    p = _promoted_detector(["a"])
    p._gm = None
    _run(p.delete_object("obj_a"))
    assert p._lm.local_map == []
    _run(p.reset_derived_state())
    assert p._lm.local_map == []


def test_stop_releases_injected_inference_after_export_failure():
    import asyncio
    from unittest.mock import Mock
    import pytest

    detector = _detector()
    provider = Mock()
    detector._engines = provider
    detector._dm = types.SimpleNamespace(inference=provider)
    detector._export = Mock(side_effect=RuntimeError('export failed'))
    with pytest.raises(RuntimeError, match='export failed'):
        asyncio.run(detector.stop())
    provider.close.assert_called_once()
    assert detector._dm.inference is None
    assert detector._engines is None


def test_replica_encoder_is_torch_free_and_uses_its_own_model_space(tmp_path, monkeypatch):
    import numpy as np
    import pytest
    import builtins
    import json
    from unittest.mock import Mock
    from scene_service.ingest import perception_engines

    weights = tmp_path / 'replica.bin'
    weights.touch()
    (tmp_path / 'model-config.json').write_text(json.dumps(dict(
        replica_clip_model='ViT-B-32', replica_clip_pretrained=str(weights.resolve()), replica_context_length=77)))
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'tensorrt')
    monkeypatch.setenv('SCENE_ONNX_DIR', str(tmp_path))
    run = Mock(side_effect=lambda feeds: {'text_feats': np.ones((len(feeds['tokens']), 512), np.float32)})
    monkeypatch.setattr(perception_engines, 'OrtSess', lambda path: types.SimpleNamespace(run=run))
    original = builtins.__import__
    def no_torch(name, *args, **kwargs):
        if name.split('.')[0] in ('torch', 'open_clip', 'ultralytics'):
            raise AssertionError(name)
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', no_torch)
    detector = _detector()
    encoder = detector._text_encoder('ViT-B-32', str(weights))
    first = detector._label_feature(encoder, 'chair')
    np.testing.assert_allclose(np.linalg.norm(first), 1., atol=1e-6)
    np.testing.assert_array_equal(first, detector._label_feature(encoder, 'chair'))
    assert run.call_count == 1
    assert detector._text_encoder('ViT-B-32', str(weights)) is encoder
    detector._export_encoder = None
    with pytest.raises(ValueError, match='differs'):
        detector._text_encoder('different-model', str(weights))


def test_replica_write_error_is_reported_instead_of_succeeding(tmp_path, monkeypatch):
    import numpy as np
    import pytest
    from unittest.mock import Mock
    from scene_service.ingest import perception_dualmap

    monkeypatch.setenv('SCENE_EXPORT_CG_PICKLE', str(tmp_path))
    detector = _detector()
    detector._text_encoder = Mock(return_value=lambda texts: np.ones((len(texts), 512), np.float32))
    monkeypatch.setattr(perception_dualmap, 'export_map_objects', Mock(side_effect=PermissionError('read only')))
    with pytest.raises(PermissionError, match='read only'):
        detector._export()
