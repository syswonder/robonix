# SPDX-License-Identifier: MulanPSL-2.0
"""Small, deployment-owned kernels for Scene's object-map pipeline.

The public call shapes intentionally match the subset of ConceptGraphs that
Scene used.  Geometry is represented by contiguous NumPy buffers rather than
Open3D objects: XYZ is float32 and RGB is uint8.  This keeps the perception
algorithm self-contained and gives x86 and aarch64 the same implementation.
"""

from __future__ import annotations

import copy
import math
from collections.abc import Iterable
from typing import Any, Optional

import numpy as np


def _points(value: Any) -> np.ndarray:
    raw = getattr(value, "points", value)
    array = np.asarray(raw, dtype=np.float32)
    if array.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    return np.ascontiguousarray(array.reshape((-1, 3)), dtype=np.float32)


def _colors(value: Any, count: int) -> np.ndarray:
    raw = getattr(value, "colors", value)
    if raw is None:
        return np.zeros((count, 3), dtype=np.uint8)
    array = np.asarray(raw)
    if array.size == 0:
        return np.zeros((count, 3), dtype=np.uint8)
    array = array.reshape((-1, 3))
    if array.shape[0] != count:
        return np.zeros((count, 3), dtype=np.uint8)
    if np.issubdtype(array.dtype, np.floating):
        finite = np.nan_to_num(array, nan=0.0, posinf=1.0, neginf=0.0)
        if finite.size and float(np.max(finite)) <= 1.0:
            finite = finite * 255.0
        array = finite
    return np.ascontiguousarray(
        np.clip(np.rint(array), 0, 255).astype(np.uint8, copy=False)
    )


def _box_corners(lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(
        [
            [lower[0], lower[1], lower[2]],
            [upper[0], lower[1], lower[2]],
            [lower[0], upper[1], lower[2]],
            [lower[0], lower[1], upper[2]],
            [upper[0], upper[1], upper[2]],
            [lower[0], upper[1], upper[2]],
            [upper[0], lower[1], upper[2]],
            [upper[0], upper[1], lower[2]],
        ],
        dtype=np.float32,
    )


class BoundingBox:
    """Minimal bbox compatibility object backed by eight float32 corners."""

    def __init__(self, corners: Any = None):
        array = np.asarray(
            corners if corners is not None else np.zeros((8, 3)),
            dtype=np.float32,
        )
        self._corners = np.ascontiguousarray(array.reshape((8, 3)))
        self.color = [0.0, 1.0, 0.0]

    @classmethod
    def from_points(cls, points: Any) -> "BoundingBox":
        array = _points(points)
        if not len(array):
            return cls()
        finite = array[np.all(np.isfinite(array), axis=1)]
        if not len(finite):
            return cls()
        return cls(_box_corners(np.min(finite, axis=0), np.max(finite, axis=0)))

    def get_box_points(self) -> np.ndarray:
        return self._corners

    def volume(self) -> float:
        extent = np.ptp(self._corners, axis=0)
        return float(np.prod(np.maximum(extent, 0.0)))

    def transform(self, matrix: Any) -> "BoundingBox":
        transform = np.asarray(matrix, dtype=np.float64)
        if transform.shape != (4, 4):
            raise ValueError("point-cloud transform must be 4x4")
        homogeneous = np.column_stack(
            (self._corners.astype(np.float64), np.ones(8, dtype=np.float64))
        )
        self._corners = np.ascontiguousarray(
            (transform @ homogeneous.T).T[:, :3],
            dtype=np.float32,
        )
        return self


def _packed_voxel_groups(
    points: np.ndarray,
    voxel_size: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return stable sort order, group starts, and counts for packed cells."""

    cell_size = max(1e-6, float(voxel_size))
    cells = np.floor(points / cell_size).astype(np.int64)
    lower = np.min(cells, axis=0)
    shifted = cells - lower
    span = np.max(shifted, axis=0) + 1
    limit = np.iinfo(np.int64).max
    can_pack = bool(
        int(span[0]) <= limit // max(1, int(span[1]))
        and int(span[0]) * int(span[1])
        <= limit // max(1, int(span[2]))
    )
    if can_pack:
        keys = (
            (shifted[:, 0] * span[1] + shifted[:, 1]) * span[2]
            + shifted[:, 2]
        )
        order = np.argsort(keys, kind="stable")
        sorted_keys = keys[order]
        starts = np.flatnonzero(
            np.r_[True, sorted_keys[1:] != sorted_keys[:-1]]
        )
    else:
        order = np.lexsort((cells[:, 2], cells[:, 1], cells[:, 0]))
        sorted_cells = cells[order]
        starts = np.flatnonzero(
            np.r_[True, np.any(sorted_cells[1:] != sorted_cells[:-1], axis=1)]
        )
    counts = np.diff(np.r_[starts, len(points)])
    return order, starts, counts


def _voxel_downsample(
    points: np.ndarray,
    colors: np.ndarray,
    voxel_size: float,
) -> tuple[np.ndarray, np.ndarray]:
    if not len(points) or float(voxel_size) <= 0.0:
        return points.copy(), colors.copy()
    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    colors = colors[finite]
    if not len(points):
        return points, colors
    order, starts, counts = _packed_voxel_groups(points, voxel_size)
    ordered_points = points[order].astype(np.float64)
    ordered_colors = colors[order].astype(np.float64)
    point_sums = np.add.reduceat(ordered_points, starts, axis=0)
    color_sums = np.add.reduceat(ordered_colors, starts, axis=0)
    return (
        np.ascontiguousarray(
            point_sums / counts[:, None],
            dtype=np.float32,
        ),
        np.ascontiguousarray(
            np.clip(np.rint(color_sums / counts[:, None]), 0, 255),
            dtype=np.uint8,
        ),
    )


def voxel_centres(points: Any, voxel_size: float) -> np.ndarray:
    """Return unique voxel centres using the packed-int64 grouping path."""

    array = _points(points)
    array = array[np.all(np.isfinite(array), axis=1)]
    if not len(array):
        return np.empty((0, 3), dtype=np.float32)
    size = max(1e-6, float(voxel_size))
    cells = np.floor(array / size).astype(np.int64)
    order, starts, _counts = _packed_voxel_groups(array, size)
    unique_cells = cells[order[starts]]
    return np.ascontiguousarray(
        (unique_cells.astype(np.float32) + 0.5) * size,
        dtype=np.float32,
    )


def _largest_dbscan_cluster(
    points: np.ndarray,
    *,
    eps: float,
    min_points: int,
) -> np.ndarray:
    """Return the largest DBSCAN component using cKDTree plus union-find."""

    if len(points) < max(2, int(min_points)):
        return np.ones(len(points), dtype=bool)
    from scipy.spatial import cKDTree

    neighbours = cKDTree(points).query_ball_point(
        points,
        r=max(1e-6, float(eps)),
        workers=-1,
    )
    core = np.fromiter(
        (len(row) >= max(1, int(min_points)) for row in neighbours),
        dtype=bool,
        count=len(points),
    )
    if not np.any(core):
        return np.ones(len(points), dtype=bool)
    parent = np.arange(len(points), dtype=np.int64)

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = int(parent[index])
        return index

    def union(left: int, right: int) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    for index in np.flatnonzero(core):
        for other in neighbours[int(index)]:
            if core[int(other)]:
                union(int(index), int(other))
    labels = np.full(len(points), -1, dtype=np.int64)
    root_to_label: dict[int, int] = {}
    for index in np.flatnonzero(core):
        root = find(int(index))
        labels[index] = root_to_label.setdefault(root, len(root_to_label))
    for index in np.flatnonzero(~core):
        adjacent = [labels[other] for other in neighbours[int(index)] if core[other]]
        if adjacent:
            labels[index] = int(adjacent[0])
    valid = labels >= 0
    if not np.any(valid):
        return np.ones(len(points), dtype=bool)
    winner = int(np.bincount(labels[valid]).argmax())
    selected = labels == winner
    return selected if int(np.count_nonzero(selected)) >= 5 else np.ones(
        len(points), dtype=bool
    )


class PointCloud:
    """Compact float32 XYZ plus uint8 RGB point cloud."""

    def __init__(self, points: Any = None, colors: Any = None):
        self._points = _points(
            np.empty((0, 3), dtype=np.float32) if points is None else points
        )
        self._colors = (
            _colors(colors, len(self._points))
            if colors is not None
            else np.zeros((len(self._points), 3), dtype=np.uint8)
        )

    @property
    def points(self) -> np.ndarray:
        return self._points

    @points.setter
    def points(self, value: Any) -> None:
        self._points = _points(value)
        if getattr(self, "_colors", np.empty((0, 3))).shape[0] != len(
            self._points
        ):
            self._colors = np.zeros((len(self._points), 3), dtype=np.uint8)

    @property
    def colors(self) -> np.ndarray:
        return self._colors

    @colors.setter
    def colors(self, value: Any) -> None:
        self._colors = _colors(value, len(self._points))

    def __deepcopy__(self, memo: dict[int, Any]) -> "PointCloud":
        copied = type(self)(self._points.copy(), self._colors.copy())
        memo[id(self)] = copied
        return copied

    def __iadd__(self, other: Any) -> "PointCloud":
        other_points = _points(other)
        other_colors = _colors(other, len(other_points))
        self._points = np.ascontiguousarray(
            np.concatenate((self._points, other_points), axis=0),
            dtype=np.float32,
        )
        self._colors = np.ascontiguousarray(
            np.concatenate((self._colors, other_colors), axis=0),
            dtype=np.uint8,
        )
        return self

    def __add__(self, other: Any) -> "PointCloud":
        result = copy.deepcopy(self)
        result += other
        return result

    def voxel_down_sample(self, voxel_size: float) -> "PointCloud":
        points, colors = _voxel_downsample(
            self._points,
            self._colors,
            voxel_size,
        )
        return type(self)(points, colors)

    def transform(self, matrix: Any) -> "PointCloud":
        transform = np.asarray(matrix, dtype=np.float64)
        if transform.shape != (4, 4):
            raise ValueError("point-cloud transform must be 4x4")
        homogeneous = np.column_stack(
            (
                self._points.astype(np.float64),
                np.ones(len(self._points), dtype=np.float64),
            )
        )
        self._points = np.ascontiguousarray(
            (transform @ homogeneous.T).T[:, :3],
            dtype=np.float32,
        )
        return self

    def paint_uniform_color(self, color: Any) -> None:
        normalized = _colors(
            np.broadcast_to(np.asarray(color).reshape((1, 3)), (len(self), 3)),
            len(self),
        )
        self._colors = normalized

    def get_axis_aligned_bounding_box(self) -> BoundingBox:
        return BoundingBox.from_points(self._points)

    def get_oriented_bounding_box(self, robust: bool = True) -> BoundingBox:
        del robust
        return self.get_axis_aligned_bounding_box()

    def __len__(self) -> int:
        return int(self._points.shape[0])


class DetectionList(list):
    def get_values(self, key: str, idx: Optional[int] = None) -> list[Any]:
        return [value[key] if idx is None else value[key][idx] for value in self]

    def get_stacked_values_torch(self, key: str, idx: Optional[int] = None):
        import torch

        values = []
        for detection in self:
            value = detection[key] if idx is None else detection[key][idx]
            if isinstance(value, BoundingBox):
                value = value.get_box_points()
            if isinstance(value, np.ndarray):
                value = torch.from_numpy(value)
            values.append(value)
        return torch.stack(values, dim=0)

    def get_stacked_values_numpy(self, key: str, idx: Optional[int] = None):
        return self.get_stacked_values_torch(key, idx).detach().cpu().numpy()

    def slice_by_indices(self, indices: Iterable[int]):
        return type(self)(self[index] for index in indices)

    def slice_by_mask(self, mask: Iterable[bool]):
        return type(self)(value for value, keep in zip(self, mask) if keep)


class MapObjectList(DetectionList):
    pass


def get_bounding_box(spatial_sim_type: str, pcd: Any) -> BoundingBox:
    del spatial_sim_type
    return BoundingBox.from_points(_points(pcd))


def process_pcd(
    pcd: Any,
    downsample_voxel_size: float,
    dbscan_remove_noise: bool,
    dbscan_eps: float,
    dbscan_min_points: int,
    run_dbscan: bool = True,
) -> PointCloud:
    source_points = _points(pcd)
    cloud = pcd if isinstance(pcd, PointCloud) else PointCloud(
        source_points,
        _colors(pcd, len(source_points)),
    )
    cloud = cloud.voxel_down_sample(float(downsample_voxel_size))
    if dbscan_remove_noise and run_dbscan and len(cloud) > 1:
        selected = _largest_dbscan_cluster(
            cloud.points,
            eps=float(dbscan_eps),
            min_points=int(dbscan_min_points),
        )
        cloud = PointCloud(cloud.points[selected], cloud.colors[selected])
    return cloud


def detections_to_obj_pcd_and_bbox(
    depth_array,
    masks,
    cam_K,
    image_rgb=None,
    trans_pose=None,
    min_points_threshold=5,
    spatial_sim_type="axis_aligned",
    obj_pcd_max_points=None,
    downsample_voxel_size=None,
    dbscan_remove_noise=None,
    dbscan_eps=None,
    dbscan_min_points=None,
    run_dbscan=None,
    device="cuda",
):
    del (
        downsample_voxel_size,
        dbscan_remove_noise,
        dbscan_eps,
        dbscan_min_points,
        run_dbscan,
        device,
    )
    depth = np.asarray(depth_array, dtype=np.float32)
    mask_array = np.asarray(masks, dtype=bool)
    intrinsics = np.asarray(cam_K, dtype=np.float64)
    colors = None if image_rgb is None else np.asarray(image_rgb)
    fx, fy = float(intrinsics[0, 0]), float(intrinsics[1, 1])
    cx, cy = float(intrinsics[0, 2]), float(intrinsics[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    transform = None if trans_pose is None else np.asarray(
        trans_pose, dtype=np.float64
    )
    output: list[Optional[dict[str, Any]]] = [None] * len(mask_array)
    minimum = max(1, int(min_points_threshold))
    target = -1 if obj_pcd_max_points is None else int(obj_pcd_max_points)
    for index, mask in enumerate(mask_array):
        valid = mask & np.isfinite(depth) & (depth > 0.0)
        ys, xs = np.nonzero(valid)
        if len(xs) < minimum:
            continue
        z = depth[ys, xs]
        points = np.column_stack(
            ((xs - cx) * z / fx, (ys - cy) * z / fy, z)
        ).astype(np.float32, copy=False)
        point_colors = (
            np.zeros((len(points), 3), dtype=np.uint8)
            if colors is None
            else _colors(colors[ys, xs], len(points))
        )
        if target > 0 and len(points) > target:
            selected = np.linspace(0, len(points) - 1, target, dtype=np.int64)
            points = points[selected]
            point_colors = point_colors[selected]
        cloud = PointCloud(points, point_colors)
        if transform is not None:
            cloud.transform(transform)
        bbox = get_bounding_box(spatial_sim_type, cloud)
        if bbox.volume() < 1e-6:
            continue
        output[index] = {"pcd": cloud, "bbox": bbox}
    return output


def compute_clip_features_batched(
    image,
    detections,
    clip_model,
    clip_preprocess,
    clip_tokenizer,
    classes,
    device,
):
    del clip_tokenizer, classes
    import torch
    from PIL import Image

    source = Image.fromarray(np.asarray(image, dtype=np.uint8))
    width, height = source.size
    crops = []
    tensors = []
    for raw in np.asarray(detections.xyxy, dtype=np.float32):
        x0, y0, x1, y1 = (float(value) for value in raw)
        x0, y0 = max(0.0, x0 - 20.0), max(0.0, y0 - 20.0)
        x1, y1 = min(float(width), x1 + 20.0), min(float(height), y1 + 20.0)
        if x1 <= x0 or y1 <= y0:
            crop = source.crop((0, 0, 1, 1))
        else:
            crop = source.crop((x0, y0, x1, y1))
        crops.append(crop)
        tensors.append(clip_preprocess(crop).unsqueeze(0))
    if not tensors:
        return [], np.empty((0, 0), dtype=np.float32), []
    batch = torch.cat(tensors, dim=0).to(device)
    with torch.no_grad():
        features = clip_model.encode_image(batch)
        features = features / features.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    return crops, features.float().cpu().numpy(), []


def compute_visual_similarities(detection_list, objects):
    import torch
    import torch.nn.functional as functional

    detections = detection_list.get_stacked_values_torch("clip_ft").float()
    persistent = objects.get_stacked_values_torch("clip_ft").float()
    return functional.normalize(detections, dim=1) @ functional.normalize(
        persistent, dim=1
    ).T


def aggregate_similarities(match_method, phys_bias, spatial_sim, visual_sim):
    if str(match_method) != "sim_sum":
        raise ValueError(f"unknown match method: {match_method}")
    bias = float(phys_bias)
    return (1.0 + bias) * spatial_sim + (1.0 - bias) * visual_sim


_HISTORY_ATTRIBUTES = (
    "image_idx",
    "mask_idx",
    "color_path",
    "class_id",
    "xyxy",
    "conf",
    "contain_number",
    "captions",
)


def _normalized_feature(value: Any):
    try:
        import torch
        import torch.nn.functional as functional

        if isinstance(value, torch.Tensor):
            return functional.normalize(value.float(), dim=0)
    except Exception:  # pragma: no cover - torch is optional to lite
        pass
    array = np.asarray(value, dtype=np.float32)
    norm = float(np.linalg.norm(array))
    return array / norm if norm > 1e-12 else array


def merge_obj2_into_obj1(
    obj1,
    obj2,
    downsample_voxel_size,
    dbscan_remove_noise,
    dbscan_eps,
    dbscan_min_points,
    spatial_sim_type,
    device,
    run_dbscan=True,
):
    del device
    left_count = max(1, int(obj1.get("num_detections", 1) or 1))
    right_count = max(1, int(obj2.get("num_detections", 1) or 1))
    for attribute in _HISTORY_ATTRIBUTES:
        if attribute in obj2:
            obj1.setdefault(attribute, [])
            obj1[attribute].extend(list(obj2.get(attribute, ()) or ()))
            del obj1[attribute][:-256]
    for attribute in ("num_detections", "num_obj_in_class"):
        if attribute in obj2:
            obj1[attribute] = int(obj1.get(attribute, 0) or 0) + int(
                obj2.get(attribute, 0) or 0
            )
    left_cloud = obj1.get("pcd")
    if not isinstance(left_cloud, PointCloud):
        left_points = _points(left_cloud)
        left_cloud = PointCloud(
            left_points,
            _colors(left_cloud, len(left_points)),
        )
    left_cloud += obj2.get("pcd")
    left_cloud = process_pcd(
        left_cloud,
        downsample_voxel_size,
        dbscan_remove_noise,
        dbscan_eps,
        dbscan_min_points,
        run_dbscan=run_dbscan,
    )
    obj1["pcd"] = left_cloud
    obj1["bbox"] = get_bounding_box(spatial_sim_type, left_cloud)
    obj1["n_points"] = len(left_cloud)
    if "clip_ft" in obj1 and "clip_ft" in obj2:
        combined = (
            obj1["clip_ft"] * left_count + obj2["clip_ft"] * right_count
        ) / float(left_count + right_count)
        obj1["clip_ft"] = _normalized_feature(combined)
    return obj1


def merge_detections_to_objects(
    downsample_voxel_size,
    dbscan_remove_noise,
    dbscan_eps,
    dbscan_min_points,
    spatial_sim_type,
    device,
    match_method,
    phys_bias,
    detection_list,
    objects,
    agg_sim,
):
    del match_method, phys_bias
    for detection_index in range(int(agg_sim.shape[0])):
        row = agg_sim[detection_index]
        if bool(np.isneginf(float(row.max().item()))):
            objects.append(detection_list[detection_index])
            continue
        object_index = int(row.argmax().item())
        objects[object_index] = merge_obj2_into_obj1(
            objects[object_index],
            detection_list[detection_index],
            downsample_voxel_size,
            dbscan_remove_noise,
            dbscan_eps,
            dbscan_min_points,
            spatial_sim_type,
            device,
            run_dbscan=False,
        )
    return objects


def denoise_objects(
    downsample_voxel_size,
    dbscan_remove_noise,
    dbscan_eps,
    dbscan_min_points,
    spatial_sim_type,
    device,
    objects,
):
    del device
    for obj in objects:
        original = obj["pcd"]
        filtered = process_pcd(
            original,
            downsample_voxel_size,
            dbscan_remove_noise,
            dbscan_eps,
            dbscan_min_points,
            run_dbscan=True,
        )
        if len(filtered) >= 4:
            obj["pcd"] = filtered
        obj["bbox"] = get_bounding_box(spatial_sim_type, obj["pcd"])
        obj["n_points"] = len(obj["pcd"])
    return objects


def filter_objects(obj_min_points, obj_min_detections, objects, map_edges=None):
    del map_edges
    return MapObjectList(
        obj
        for obj in objects
        if obj.get("operator_label")
        or (
            len(obj["pcd"].points) >= int(obj_min_points)
            and int(obj.get("num_detections", 0) or 0)
            >= int(obj_min_detections)
        )
    )


def _cosine(left: Any, right: Any) -> float:
    if hasattr(left, "detach"):
        left = left.detach().float().cpu().numpy()
    if hasattr(right, "detach"):
        right = right.detach().float().cpu().numpy()
    a = np.asarray(left, dtype=np.float32).reshape(-1)
    b = np.asarray(right, dtype=np.float32).reshape(-1)
    denominator = float(np.linalg.norm(a) * np.linalg.norm(b))
    return float(np.dot(a, b) / denominator) if denominator > 1e-12 else -1.0


def merge_overlap_objects(
    merge_overlap_thresh,
    merge_visual_sim_thresh,
    merge_text_sim_thresh,
    objects,
    overlap_matrix,
    downsample_voxel_size,
    dbscan_remove_noise,
    dbscan_eps,
    dbscan_min_points,
    spatial_sim_type,
    device,
    map_edges=None,
):
    del map_edges
    matrix = np.asarray(overlap_matrix, dtype=np.float32)
    pairs = [
        (float(max(matrix[i, j], matrix[j, i])), i, j)
        for i in range(len(objects))
        for j in range(i + 1, len(objects))
        if max(matrix[i, j], matrix[j, i]) > float(merge_overlap_thresh)
    ]
    pairs.sort(reverse=True)
    kept = np.ones(len(objects), dtype=bool)
    for _score, left, right in pairs:
        if not kept[left] or not kept[right]:
            continue
        visual = _cosine(objects[left].get("clip_ft"), objects[right].get("clip_ft"))
        if visual <= float(merge_visual_sim_thresh) or visual <= float(
            merge_text_sim_thresh
        ):
            continue
        objects[right] = merge_obj2_into_obj1(
            objects[right],
            objects[left],
            downsample_voxel_size,
            dbscan_remove_noise,
            dbscan_eps,
            dbscan_min_points,
            spatial_sim_type,
            device,
            run_dbscan=True,
        )
        kept[left] = False
    index_updates: list[Optional[int]] = [None] * len(objects)
    output = MapObjectList()
    for index, obj in enumerate(objects):
        if kept[index]:
            index_updates[index] = len(output)
            output.append(obj)
    return output, index_updates


__all__ = [
    "BoundingBox",
    "DetectionList",
    "MapObjectList",
    "PointCloud",
    "aggregate_similarities",
    "compute_clip_features_batched",
    "compute_visual_similarities",
    "denoise_objects",
    "detections_to_obj_pcd_and_bbox",
    "filter_objects",
    "get_bounding_box",
    "merge_detections_to_objects",
    "merge_obj2_into_obj1",
    "merge_overlap_objects",
    "process_pcd",
    "voxel_centres",
]
