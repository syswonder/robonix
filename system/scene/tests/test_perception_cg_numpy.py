# SPDX-License-Identifier: MulanPSL-2.0
"""Object merging preserves observations and exported masks."""
import numpy as np
import pytest

pytest.importorskip("open3d")
from scene_service.ingest import perception_cg_numpy as cg


def test_merge_retains_only_first_mask(monkeypatch):
    class Cloud:
        points = np.ones((4, 3))

        def __iadd__(self, other):
            return self

    monkeypatch.setattr(cg, "process_pcd", lambda cloud, *args: cloud)
    monkeypatch.setattr(cg, "get_bounding_box", lambda *args: type("Box", (), {})())
    first = np.ones((4, 4), bool)
    obj = {"mask": [first], "num_detections": 1, "pcd": Cloud(),
           "clip_ft": np.ones(512, np.float32)}
    for _ in range(100):
        other = {"mask": [np.zeros((4, 4), bool)], "num_detections": 1,
                 "pcd": Cloud(), "clip_ft": np.ones(512, np.float32)}
        cg.merge_obj2_into_obj1(obj, other, .02, False, .02, 5, "axis_aligned")
    assert len(obj["mask"]) == 1
    assert obj["mask"][0] is first
    assert obj["num_detections"] == 101


def test_retained_mask_survives_map_export(tmp_path):
    import gzip
    import pickle
    import open3d as o3d
    from scene_service.ingest.cg_export import export_map_objects

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector([[0, 0, 0], [1, 1, 1]])
    mask = np.ones((4, 4), bool)
    obj = {"mask": [mask], "pcd": cloud, "bbox": cloud.get_axis_aligned_bounding_box(),
           "clip_ft": np.ones(512, np.float32), "num_detections": 3}
    path = export_map_objects(cg.MapObjectList([obj]), "test", tmp_path, "ViT-B-32", "local")
    with gzip.open(path, "rb") as source:
        payload = pickle.load(source)
    exported = payload["objects"][0]
    assert len(exported["mask"]) == 1
    np.testing.assert_array_equal(exported["mask"][0], mask)
    assert exported["num_detections"] == 3
    assert "pcd_np" in exported


def test_overlap_merge_counts_each_source_once():
    import open3d as o3d

    objects = cg.MapObjectList()
    for i in range(3):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(
            [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
        objects.append({"pcd": cloud, "clip_ft": np.array([1., 0.], np.float32),
                        "num_detections": 1, "image_idx": [i]})
    overlap = np.ones((3, 3), np.float32) - np.eye(3, dtype=np.float32)
    merged, indices = cg.merge_overlap_objects(
        .5, .5, 0., objects, overlap, .025, False, .02, 5, "axis_aligned")
    assert len(merged) == 1
    assert merged[0]["num_detections"] == 3
    assert sorted(merged[0]["image_idx"]) == [0, 1, 2]
    assert sorted(i for i in indices if i is not None) == [0]


def test_unmatched_detection_is_appended_without_copying():
    detection = {"id": "new-track"}
    objects = cg.MapObjectList([{"id": "existing"}])
    result = cg.merge_detections_to_objects(
        .025, False, .02, 5, "axis_aligned",
        detection_list=cg.DetectionList([detection]), objects=objects,
        agg_sim=np.array([[-np.inf]], np.float32))
    assert result is objects
    assert result[-1] is detection
