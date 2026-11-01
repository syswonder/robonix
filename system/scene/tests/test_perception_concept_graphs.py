# SPDX-License-Identifier: MulanPSL-2.0
"""Association, cleanup publication and representative-feature refresh."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from scene_service.ingest import perception_concept_graphs as cg
from scene_service.state.object_registry import ObjectRegistry


@pytest.fixture
def detector(monkeypatch):
    frame = SimpleNamespace(header=SimpleNamespace(stamp=None))
    intrinsics = SimpleNamespace(fx=100., fy=100., cx=2., cy=2.)
    d = cg.ConceptGraphsDetector(
        rgb_fetcher_msg=lambda: frame, depth_fetcher_msg=lambda: frame,
        camera_info_fetcher=lambda: intrinsics, on_detections=Mock(),
        registry=ObjectRegistry(), world_frame_fn=lambda: "map")
    monkeypatch.setattr(cg, "_image_msg_to_bgr", lambda msg: np.zeros((4, 4, 3), np.uint8))
    monkeypatch.setattr(cg, "_depth_msg_to_metres", lambda msg: np.ones((4, 4), np.float32))
    monkeypatch.setattr(d, "_build_camera_to_map_transform", lambda **kwargs: np.eye(4))
    cloud = SimpleNamespace(points=np.array([[0, 0, 1], [1, 0, 1], [0, 1, 1], [0, 0, 2.]]))
    d._classes = ["chair"]
    d._engines = SimpleNamespace(
        detect=Mock(return_value=np.array([[0, 0, 3, 3, .9, 0]], np.float32)),
        segment=lambda *args: np.ones((1, 4, 4), bool),
        clip_feats=lambda *args: np.ones((1, 512), np.float32))
    d._map_objects = [{"id": "track", "class_name": "chair", "pcd": cloud}]
    def merge(**kwargs):
        for detection, scores in zip(kwargs["detection_list"], kwargs["agg_sim"]):
            if np.isneginf(scores).all():
                kwargs["objects"].append(detection)
        return kwargs["objects"]

    d._cg = SimpleNamespace(
        DetectionList=list,
        detections_to_obj_pcd_and_bbox=lambda **kwargs: [{"pcd": cloud, "bbox": None}],
        compute_visual_similarities=Mock(return_value=np.array([[1.]], np.float32)),
        aggregate_similarities=Mock(return_value=np.array([[1.]], np.float32)),
        merge_detections_to_objects=Mock(side_effect=merge),
        denoise_objects=lambda **kwargs: kwargs["objects"],
        filter_objects=lambda **kwargs: kwargs["objects"],
        merge_overlap_objects=Mock(side_effect=lambda **kwargs: (kwargs["objects"], [])))
    monkeypatch.setattr(d, "_voxel_vote_similarity", Mock(return_value=np.array([[.8]], np.float32)))
    monkeypatch.setattr(d, "_voxel_pcd_overlap_matrix", Mock(return_value=np.array([[1.]], np.float32)))
    monkeypatch.setattr(d, "_record_observation", Mock())
    monkeypatch.setattr(d, "_refresh_object", Mock())
    monkeypatch.setattr(d, "_update_visible_absence", Mock())
    monkeypatch.setattr(d, "_cross_class_geometric_collapse", lambda objects: objects)
    monkeypatch.setattr(d, "_same_class_proximity_collapse", lambda objects: objects)
    monkeypatch.setattr(d, "_project_to_registry", Mock())
    monkeypatch.setattr(cg, "export_from_env", Mock())
    return d


@pytest.mark.parametrize("association", ["voxel_vote", "cg"])
def test_association_runs_only_selected_scoring_path(detector, association):
    d = detector
    d.cfg["association"] = association
    d._tick_once()
    assert d._cg.merge_detections_to_objects.call_count == 1
    if association == "voxel_vote":
        d._cg.compute_visual_similarities.assert_not_called()
        d._cg.aggregate_similarities.assert_not_called()
        d._voxel_pcd_overlap_matrix.assert_not_called()
        d._voxel_vote_similarity.assert_called_once()
    else:
        d._cg.compute_visual_similarities.assert_called_once()
        d._cg.aggregate_similarities.assert_called_once()
        d._voxel_pcd_overlap_matrix.assert_called_once()
        d._voxel_vote_similarity.assert_not_called()
    scores = d._cg.merge_detections_to_objects.call_args.kwargs["agg_sim"]
    np.testing.assert_allclose(scores, [[.8 if association == "voxel_vote" else 1.]])


@pytest.mark.parametrize("observed", [False, True])
@pytest.mark.parametrize("cleanup_due", [False, True])
def test_tick_publishes_once_for_observation_or_cleanup(detector, observed, cleanup_due):
    d = detector
    d._tick_idx = 9 if cleanup_due else 1
    if not observed:
        d._engines.detect.return_value = np.empty((0, 6), np.float32)
    d._tick_once()
    assert d._project_to_registry.call_count == int(observed or cleanup_due)
    assert d._cg.merge_overlap_objects.call_count == int(cleanup_due)


@pytest.mark.parametrize("association", ["voxel_vote", "cg"])
def test_cg_distance_gate_stays_with_cg_scoring(detector, association):
    d = detector
    d.cfg["association"] = association
    d._map_objects[0]["pcd"] = SimpleNamespace(
        points=d._map_objects[0]["pcd"].points + [10, 0, 0])
    d._tick_once()
    scores = d._cg.merge_detections_to_objects.call_args.kwargs["agg_sim"]
    if association == "cg":
        assert np.isneginf(scores[0, 0])
    else:
        np.testing.assert_allclose(scores, [[.8]])


def test_optional_export_failure_does_not_prevent_publication(detector, monkeypatch):
    detector._tick_idx = 9
    monkeypatch.setattr(cg, "export_from_env", Mock(side_effect=OSError("disk full")))
    with pytest.raises(OSError, match="disk full"):
        detector._tick_once()
    detector._project_to_registry.assert_called_once()


@pytest.fixture
def feature_tracks(detector, monkeypatch):
    d = detector
    monkeypatch.setattr(d, "_refresh_object", cg.ConceptGraphsDetector._refresh_object.__get__(d))
    monkeypatch.setattr(d, "_record_observation", cg.ConceptGraphsDetector._record_observation.__get__(d))
    tracks = [{"id": name, "num_detections": 1, "class_name": "chair",
               "clip_ft": np.zeros(2, np.float32)} for name in ("a", "b")]
    d._map_objects = tracks
    d._bank = {o["id"]: [(np.array([1., 0.], np.float32), 4.),
                         (np.array([0., 1.], np.float32), 8.)] for o in tracks}
    monkeypatch.setattr(d, "_label_text_feat", Mock(return_value=np.array([1., 0.], np.float32)))
    return d, tracks


def test_unchanged_tracks_skip_selection_and_cleanup_merges_invalidate(feature_tracks):
    d, (changed, untouched) = feature_tracks
    for obj in d._map_objects:
        d._refresh_object(obj)
    previous = untouched["clip_ft"]
    d._label_text_feat.reset_mock()
    # A cleanup merge changes the count without recording a new detection.
    changed["num_detections"] += 1
    changed["clip_ft"] = np.array([.5, .5], np.float32)
    for obj in d._map_objects:
        d._refresh_object(obj)
    d._label_text_feat.assert_called_once()
    np.testing.assert_array_equal(changed["clip_ft"], [1., 0.])
    assert untouched["clip_ft"] is previous
    d._map_objects = [untouched]
    d._reconcile_side_tables()
    assert set(d._refreshed_counts) == {untouched["id"]}


def test_new_observation_invalidates_selection_even_at_same_count(feature_tracks):
    d, (obj, _) = feature_tracks
    d._refresh_object(obj)
    observation = {"class_name": "table", "conf": [5.], "mask": [np.ones((4, 4), bool)],
                   "clip_ft": np.array([0., 1.], np.float32),
                   "pcd": SimpleNamespace(points=np.empty((0, 3)))}
    d._record_observation(obj["id"], observation)
    d._label_text_feat.return_value = np.array([0., 1.], np.float32)
    d._refresh_object(obj)
    assert obj["class_name"] == "table"
    np.testing.assert_array_equal(obj["clip_ft"], [0., 1.])


def test_text_fallback_and_failed_refresh_retry(feature_tracks):
    d, (obj, _) = feature_tracks
    d._label_text_feat.side_effect = [None, RuntimeError("temporary failure"),
                                    np.array([1., 0.], np.float32)]
    d._refresh_object(obj)
    np.testing.assert_array_equal(obj["clip_ft"], [0., 1.])
    d._refresh_object(obj)
    d._refresh_object(obj)
    np.testing.assert_array_equal(obj["clip_ft"], [1., 0.])
    assert d._label_text_feat.call_count == 3


@pytest.mark.parametrize("matched", [False, True])
def test_observations_use_the_matched_or_new_track_identity(detector, matched):
    d = detector
    d._voxel_vote_similarity.return_value = np.array([[.8 if matched else 0.]], np.float32)
    d._tick_once()
    track_id, detection = d._record_observation.call_args.args
    if matched:
        assert track_id == "track"
        assert len(d._map_objects) == 1
    else:
        assert track_id == detection["id"]
        assert d._map_objects[-1] is detection


def test_selected_tensorrt_loader_does_not_import_torch(monkeypatch, tmp_path):
    import builtins
    from scene_service.ingest import perception_concept_graphs as cg
    from scene_service.ingest import perception_engines

    monkeypatch.setenv('SCENE_CG_INFERENCE', 'tensorrt')
    monkeypatch.setenv('SCENE_ONNX_DIR', str(tmp_path))
    monkeypatch.delenv('SCENE_CG_FORCE_CPU', raising=False)
    monkeypatch.delenv('SCENE_CLIP_MODEL', raising=False)
    monkeypatch.setenv('SCENE_PERCEPTION_BACKEND', 'concept_graphs')
    sentinel = object()
    monkeypatch.setattr(perception_engines, 'load_engines', lambda *a, **k: sentinel)
    original = builtins.__import__

    def without_torch(name, *args, **kwargs):
        if name.split('.')[0] in ('torch', 'ultralytics', 'open_clip'):
            raise AssertionError(f'Unexpected runtime import: {name}')
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', without_torch)
    assert cg._load_engine_bundle() == (sentinel, 'cuda')


def test_missing_torch_models_preserve_geometric_service(monkeypatch, caplog):
    from scene_service.ingest import perception_concept_graphs as cg
    from scene_service.ingest import perception_torch

    monkeypatch.delenv('SCENE_CG_INFERENCE', raising=False)

    def missing(*args):
        raise FileNotFoundError('missing source weights')

    monkeypatch.setattr(perception_torch, 'TorchPerception', missing)
    assert cg._load_engine_bundle() == (None, 'cpu')
    assert 'perception disabled' in caplog.text
