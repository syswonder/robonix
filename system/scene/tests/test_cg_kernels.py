# SPDX-License-Identifier: MulanPSL-2.0
"""Behavior and storage contracts for Scene-owned object-map kernels."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

# Torch is absent from lite profiles by design, and the kernels only reach for
# it on the CLIP-feature paths exercised here.  Skip rather than fail collection
# so the rest of the suite still runs on a lite host.
torch = pytest.importorskip("torch")

from scene_service.ingest.cg_kernels import (
    DetectionList,
    MapObjectList,
    PointCloud,
    aggregate_similarities,
    detections_to_obj_pcd_and_bbox,
    merge_detections_to_objects,
    merge_obj2_into_obj1,
    merge_overlap_objects,
    process_pcd,
)


def _object(identifier: str, offset: float = 0.0) -> dict:
    points = np.asarray(
        [
            [0.00 + offset, 0.00, 0.00],
            [0.05 + offset, 0.00, 0.00],
            [0.00 + offset, 0.05, 0.00],
            [0.05 + offset, 0.05, 0.05],
        ],
        dtype=np.float64,
    )
    return {
        "id": identifier,
        "class_name": "cup",
        "class_id": [0],
        "conf": [0.9],
        "image_idx": [1],
        "xyxy": [[0.0, 0.0, 4.0, 4.0]],
        "num_detections": 1,
        "n_points": len(points),
        "pcd": PointCloud(points, np.ones_like(points)),
        "clip_ft": torch.tensor([1.0, 0.0]),
    }


def test_point_cloud_storage_is_compact_and_voxel_downsampled() -> None:
    points = np.asarray(
        [[0.001, 0.0, 0.0], [0.002, 0.0, 0.0], [0.101, 0.0, 0.0]],
        dtype=np.float64,
    )
    colors = np.asarray([[1.0, 0.0, 0.0]] * 3, dtype=np.float64)
    cloud = PointCloud(points, colors)
    reduced = cloud.voxel_down_sample(0.05)
    assert cloud.points.dtype == np.float32
    assert cloud.colors.dtype == np.uint8
    assert cloud.points.flags.c_contiguous
    assert cloud.colors.flags.c_contiguous
    assert cloud.colors[0].tolist() == [255, 0, 0]
    assert len(reduced) == 2
    assert np.isclose(reduced.points[0, 0], 0.0015)


def test_process_pcd_keeps_largest_density_component() -> None:
    cluster = np.asarray(
        [[x, y, 0.0] for x in (0.0, 0.01, 0.02) for y in (0.0, 0.01)],
        dtype=np.float32,
    )
    noise = np.asarray([[2.0, 2.0, 2.0], [3.0, 3.0, 3.0]], dtype=np.float32)
    filtered = process_pcd(
        PointCloud(np.vstack((cluster, noise))),
        downsample_voxel_size=0.001,
        dbscan_remove_noise=True,
        dbscan_eps=0.025,
        dbscan_min_points=3,
        run_dbscan=True,
    )
    assert len(filtered) == len(cluster)
    assert float(np.max(filtered.points[:, 0])) < 1.0


def test_depth_masks_convert_to_transformed_compact_cloud() -> None:
    depth = np.asarray(
        [[2.0, 2.1, 2.0], [2.2, 2.3, 2.0], [2.0, 2.0, 2.0]],
        dtype=np.float32,
    )
    masks = np.zeros((1, 3, 3), dtype=bool)
    masks[0, :2, :2] = True
    image = np.full((3, 3, 3), [10, 20, 30], dtype=np.uint8)
    transform = np.eye(4, dtype=np.float64)
    transform[0, 3] = 1.0
    result = detections_to_obj_pcd_and_bbox(
        depth,
        masks,
        np.asarray([[2.0, 0.0, 1.0], [0.0, 2.0, 1.0], [0.0, 0.0, 1.0]]),
        image_rgb=image,
        trans_pose=transform,
        min_points_threshold=4,
        obj_pcd_max_points=4,
    )[0]
    assert result is not None
    assert isinstance(result["pcd"], PointCloud)
    assert result["pcd"].points.dtype == np.float32
    assert result["pcd"].colors.dtype == np.uint8
    assert np.isclose(float(np.mean(result["pcd"].points[:, 0])), 0.475)
    assert result["bbox"].volume() > 0.0


def test_merge_preserves_identity_and_operator_evidence_with_bounded_history() -> None:
    left = _object("survivor")
    right = _object("removed", offset=0.002)
    left["operator_label"] = "my cup"
    right["image_idx"] = list(range(300))
    right["class_id"] = [0] * 300
    right["conf"] = [0.8] * 300
    right["num_detections"] = 300
    merged = merge_obj2_into_obj1(
        left,
        right,
        downsample_voxel_size=0.001,
        dbscan_remove_noise=False,
        dbscan_eps=0.02,
        dbscan_min_points=3,
        spatial_sim_type="overlap",
        device="cpu",
        run_dbscan=False,
    )
    assert merged["id"] == "survivor"
    assert merged["operator_label"] == "my cup"
    assert merged["num_detections"] == 301
    assert len(merged["image_idx"]) == 256
    assert merged["n_points"] > 4
    assert np.isclose(float(torch.linalg.vector_norm(merged["clip_ft"])), 1.0)


def test_detection_and_periodic_merge_interfaces_match_runtime_contract() -> None:
    persistent = MapObjectList([_object("persistent")])
    detection = _object("detection", offset=0.002)
    merged = merge_detections_to_objects(
        downsample_voxel_size=0.001,
        dbscan_remove_noise=False,
        dbscan_eps=0.02,
        dbscan_min_points=3,
        spatial_sim_type="overlap",
        device="cpu",
        match_method="sim_sum",
        phys_bias=0.0,
        detection_list=DetectionList([detection]),
        objects=persistent,
        agg_sim=torch.tensor([[1.0]]),
    )
    assert len(merged) == 1
    assert merged[0]["num_detections"] == 2

    second = _object("second", offset=0.003)
    objects, updates = merge_overlap_objects(
        merge_overlap_thresh=0.5,
        merge_visual_sim_thresh=0.5,
        merge_text_sim_thresh=0.5,
        objects=MapObjectList([merged[0], second]),
        overlap_matrix=np.asarray([[0.0, 0.9], [0.9, 0.0]]),
        downsample_voxel_size=0.001,
        dbscan_remove_noise=False,
        dbscan_eps=0.02,
        dbscan_min_points=3,
        spatial_sim_type="overlap",
        device="cpu",
    )
    assert len(objects) == 1
    assert updates.count(None) == 1


def test_similarity_aggregation_retains_conceptgraphs_weighting() -> None:
    spatial = torch.tensor([[0.2, 0.5]])
    visual = torch.tensor([[0.8, 0.1]])
    result = aggregate_similarities("sim_sum", 0.25, spatial, visual)
    expected = 1.25 * spatial + 0.75 * visual
    assert torch.allclose(result, expected)
