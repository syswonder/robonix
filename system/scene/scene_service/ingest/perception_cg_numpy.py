"""Torch-free ConceptGraphs geometry and object merging for Scene.

Adapted from conceptgraph/slam/{utils,mapping,slam_classes} and
utils/model_utils.py on the ali-dev branch. Scene omits upstream tracker
logging, retains one initial mask, and merges only surviving objects.
"""
from __future__ import annotations

import copy
import logging
from collections import Counter

import numpy as np
import open3d as o3d

log = logging.getLogger(__name__)

# F.normalize's default eps (torch.nn.functional) — keeps the zero-vector
# behaviour identical to the torch path.
_FEPS = 1e-12


def _unit(v: np.ndarray) -> np.ndarray:
    """F.normalize(v, dim=0) for a 1-D feature: v / max(||v||, 1e-12)."""
    v = np.asarray(v, dtype=np.float32)
    n = float(np.linalg.norm(v))
    return v / max(n, _FEPS)


class DetectionList(list):
    pass


class MapObjectList(list):
    """Container for merged map objects; mirrors the upstream list subclass."""

    def to_serializable(self):
        s_obj_list = []
        for obj in self:
            s_obj_dict = copy.deepcopy(obj)
            s_obj_dict["clip_ft"] = np.asarray(s_obj_dict["clip_ft"], dtype=np.float32)
            s_obj_dict["pcd_np"] = np.asarray(s_obj_dict["pcd"].points)
            s_obj_dict["bbox_np"] = np.asarray(s_obj_dict["bbox"].get_box_points())
            s_obj_dict["pcd_color_np"] = np.asarray(s_obj_dict["pcd"].colors)
            del s_obj_dict["pcd"]
            del s_obj_dict["bbox"]
            s_obj_list.append(s_obj_dict)
        return s_obj_list


def process_pcd(
    pcd: o3d.geometry.PointCloud,
    downsample_voxel_size: float,
    dbscan_remove_noise: bool,
    dbscan_eps: float,
    dbscan_min_points: int,
    run_dbscan: bool = True,
) -> o3d.geometry.PointCloud:
    pcd = pcd.voxel_down_sample(voxel_size=downsample_voxel_size)
    if dbscan_remove_noise and run_dbscan:
        pcd = pcd_denoise_dbscan(pcd, eps=dbscan_eps, min_points=dbscan_min_points)
    return pcd


def pcd_denoise_dbscan(
    pcd: o3d.geometry.PointCloud, eps: float = 0.02, min_points: int = 10
) -> o3d.geometry.PointCloud:
    pcd_clusters = pcd.cluster_dbscan(eps=eps, min_points=min_points)
    obj_points = np.asarray(pcd.points)
    obj_colors = np.asarray(pcd.colors)
    pcd_clusters = np.array(pcd_clusters)

    counter = Counter(pcd_clusters)
    if counter and (-1 in counter):
        del counter[-1]

    if counter:
        most_common_label, _ = counter.most_common(1)[0]
        largest_mask = pcd_clusters == most_common_label
        largest_cluster_points = obj_points[largest_mask]
        largest_cluster_colors = obj_colors[largest_mask]
        # Upstream keeps the original cloud when the largest cluster is tiny.
        if len(largest_cluster_points) < 5:
            return pcd
        largest_cluster_pcd = o3d.geometry.PointCloud()
        largest_cluster_pcd.points = o3d.utility.Vector3dVector(largest_cluster_points)
        largest_cluster_pcd.colors = o3d.utility.Vector3dVector(largest_cluster_colors)
        pcd = largest_cluster_pcd
    return pcd


def get_bounding_box(spatial_sim_type: str, pcd: o3d.geometry.PointCloud):
    if ("accurate" in spatial_sim_type or "overlap" in spatial_sim_type) and len(pcd.points) >= 4:
        try:
            return pcd.get_oriented_bounding_box(robust=True)
        except RuntimeError as e:
            log.info("oriented bbox failed (%s); axis-aligned instead", e)
            return pcd.get_axis_aligned_bounding_box()
    else:
        return pcd.get_axis_aligned_bounding_box()


def merge_obj2_into_obj1(
    obj1: dict,
    obj2: dict,
    downsample_voxel_size: float,
    dbscan_remove_noise: bool,
    dbscan_eps: float,
    dbscan_min_points: int,
    spatial_sim_type: str,
    device: str = "",  # accepted for signature parity; unused without torch
    run_dbscan: bool = True,
) -> dict:
    extend_attributes = [
        "image_idx", "mask_idx", "color_path", "class_id",
        "xyxy", "conf", "contain_number", "captions",
    ]
    add_attributes = ["num_detections", "num_obj_in_class"]
    skip_attributes = ["id", "class_name", "is_background", "new_counter", "curr_obj_num", "inst_color", "mask"]
    custom_handled = ["pcd", "bbox", "clip_ft", "text_ft", "n_points"]

    all_handled_keys = set(extend_attributes + add_attributes + skip_attributes + custom_handled)
    unhandled_keys = set(obj2.keys()) - all_handled_keys
    if unhandled_keys:
        raise ValueError(
            f"Unhandled keys detected in obj2: {unhandled_keys}. "
            "Please update the merge function to handle these attributes."
        )

    n_obj1_det = obj1["num_detections"]
    n_obj2_det = obj2["num_detections"]

    for attr in extend_attributes:
        if attr in obj1 and attr in obj2:
            obj1[attr].extend(obj2[attr])

    for attr in add_attributes:
        if attr in obj1 and attr in obj2:
            obj1[attr] += obj2[attr]

    if "caption" in obj1 and "caption" in obj2:
        for key, value in obj2["caption"].items():
            obj1["caption"][key + n_obj1_det] = value

    obj1["pcd"] += obj2["pcd"]
    obj1["pcd"] = process_pcd(
        obj1["pcd"], downsample_voxel_size, dbscan_remove_noise,
        dbscan_eps, dbscan_min_points, run_dbscan,
    )
    obj1["n_points"] = len(np.asarray(obj1["pcd"].points))

    obj1["bbox"] = get_bounding_box(spatial_sim_type, obj1["pcd"])
    obj1["bbox"].color = [0, 1, 0]

    obj1["clip_ft"] = _unit(
        (np.asarray(obj1["clip_ft"], np.float32) * n_obj1_det
         + np.asarray(obj2["clip_ft"], np.float32) * n_obj2_det)
        / (n_obj1_det + n_obj2_det)
    )
    return obj1


def compute_visual_similarities(detection_list: DetectionList, objects: MapObjectList) -> np.ndarray:
    """(M, N) cosine matrix between detection and object clip features.

    Upstream computes F.cosine_similarity directly; features here are always
    unit-norm (CLIP output and every merge re-normalizes), so the dot product
    of normalized vectors equals the cosine. The max(norm, eps) clamp keeps
    the degenerate zero-vector case at 0 like F.cosine_similarity.
    """
    def _stack(items):
        if not items:
            return np.zeros((0, 1), dtype=np.float32)
        return np.stack([np.asarray(x["clip_ft"], dtype=np.float32).ravel() for x in items])

    det = _stack(detection_list)
    obj = _stack(objects)
    det = det / np.maximum(np.linalg.norm(det, axis=1, keepdims=True), _FEPS)
    obj = obj / np.maximum(np.linalg.norm(obj, axis=1, keepdims=True), _FEPS)
    return (det @ obj.T).astype(np.float32)


def aggregate_similarities(
    match_method: str, phys_bias: float,
    spatial_sim: np.ndarray, visual_sim: np.ndarray,
) -> np.ndarray:
    if match_method == "sim_sum":
        return (1 + phys_bias) * spatial_sim + (1 - phys_bias) * visual_sim
    raise ValueError(f"Unknown matching method: {match_method}")


def merge_detections_to_objects(
    downsample_voxel_size: float, dbscan_remove_noise: bool, dbscan_eps: float,
    dbscan_min_points: int, spatial_sim_type: str, device: str = "",
    match_method: str = "sim_sum", phys_bias: float = 0.5,
    detection_list: DetectionList = None, objects: MapObjectList = None,
    agg_sim: np.ndarray = None,
) -> MapObjectList:
    for detected_obj_idx in range(agg_sim.shape[0]):
        if agg_sim[detected_obj_idx].max() == float("-inf"):
            objects.append(detection_list[detected_obj_idx])
        else:
            existing_obj_match_idx = int(np.argmax(agg_sim[detected_obj_idx]))
            detected_obj = detection_list[detected_obj_idx]
            matched_obj = objects[existing_obj_match_idx]
            merged_obj = merge_obj2_into_obj1(
                obj1=matched_obj, obj2=detected_obj,
                downsample_voxel_size=downsample_voxel_size,
                dbscan_remove_noise=dbscan_remove_noise,
                dbscan_eps=dbscan_eps, dbscan_min_points=dbscan_min_points,
                spatial_sim_type=spatial_sim_type, device=device,
                run_dbscan=False,
            )
            objects[existing_obj_match_idx] = merged_obj
    return objects


def merge_overlap_objects(
    merge_overlap_thresh: float,
    merge_visual_sim_thresh: float,
    merge_text_sim_thresh: float,
    objects: MapObjectList,
    overlap_matrix: np.ndarray,
    downsample_voxel_size: float,
    dbscan_remove_noise: bool,
    dbscan_eps: float,
    dbscan_min_points: int,
    spatial_sim_type: str,
    device: str = "",
    map_edges=None,
):
    x, y = overlap_matrix.nonzero()
    overlap_ratio = overlap_matrix[x, y]

    sort = np.argsort(overlap_ratio)[::-1]
    x = x[sort]
    y = y[sort]
    overlap_ratio = overlap_ratio[sort]

    kept_objects = np.ones(len(objects), dtype=bool)
    index_updates = [None] * len(objects)

    for i, j, ratio in zip(x, y, overlap_ratio):
        if not (kept_objects[i] and kept_objects[j]):
            continue
        if ratio > merge_overlap_thresh:
            visual_sim = float(
                np.dot(objects[i]["clip_ft"], objects[j]["clip_ft"])
            )
            # Upstream keeps text_sim == visual_sim.
            text_sim = visual_sim
            if (visual_sim > merge_visual_sim_thresh) and (text_sim > merge_text_sim_thresh):
                objects[j] = merge_obj2_into_obj1(
                    objects[j], objects[i],
                    downsample_voxel_size, dbscan_remove_noise,
                    dbscan_eps, dbscan_min_points, spatial_sim_type,
                    device, run_dbscan=True,
                )
                kept_objects[i] = False
        else:
            break

    current_index = 0
    for original_index, is_kept in enumerate(kept_objects):
        if is_kept:
            index_updates[original_index] = current_index
            current_index += 1

    return MapObjectList(obj for obj, keep in zip(objects, kept_objects) if keep), index_updates


def denoise_objects(
    downsample_voxel_size: float,
    dbscan_remove_noise: bool,
    dbscan_eps: float,
    dbscan_min_points: int,
    spatial_sim_type: str,
    device: str = "",
    objects: MapObjectList = None,
) -> MapObjectList:
    for i in range(len(objects)):
        og_object_pcd = objects[i]["pcd"]
        if len(og_object_pcd.points) <= 1:
            objects[i]["pcd"] = og_object_pcd
        else:
            objects[i]["pcd"] = process_pcd(
                objects[i]["pcd"], downsample_voxel_size, dbscan_remove_noise,
                dbscan_eps, dbscan_min_points, run_dbscan=True,
            )
            if len(objects[i]["pcd"].points) < 4:
                objects[i]["pcd"] = og_object_pcd
        objects[i]["bbox"] = get_bounding_box(spatial_sim_type, objects[i]["pcd"])
        objects[i]["bbox"].color = [0, 1, 0]
    return objects


def filter_objects(
    obj_min_points: int,
    obj_min_detections: int,
    objects: MapObjectList,
    map_edges=None,
) -> MapObjectList:
    return MapObjectList(
        obj for obj in objects
        if len(obj["pcd"].points) >= obj_min_points
        and obj["num_detections"] >= obj_min_detections
    )


def dynamic_downsample(points: np.ndarray, colors: np.ndarray = None, target: int = 5000):
    if target == -1 or target is None:
        return points, colors
    num_points = points.shape[0]
    if num_points <= target:
        return points, colors
    downsample_factor = max(1, num_points // target)
    downsampled_points = points[::downsample_factor]
    downsampled_colors = colors[::downsample_factor] if colors is not None else None
    return downsampled_points, downsampled_colors


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
    device="",  # unused without torch; numpy path runs on CPU
):
    """Batched depth back-projection per mask, mirroring the upstream
    torch pipeline (batch_mask_depth_to_points_colors + dynamic_downsample):
    z = depth·mask, valid = z > 0, ray-scaled pixel grid, colors /255, points
    kept in raster order, equal-stride downsample, transform, volume-gated."""
    N, H, W = masks.shape
    depth = np.asarray(depth_array, dtype=np.float32)
    fx, fy, cx, cy = float(cam_K[0, 0]), float(cam_K[1, 1]), float(cam_K[0, 2]), float(cam_K[1, 2])

    y_grid, x_grid = np.meshgrid(np.arange(H, dtype=np.float32),
                                 np.arange(W, dtype=np.float32), indexing="ij")
    ray_x = (x_grid - cx) / fx
    ray_y = (y_grid - cy) / fy

    rgb_flat = None
    if image_rgb is not None:
        rgb_flat = np.asarray(image_rgb, dtype=np.float32).reshape(-1, 3) / 255.0

    processed_objects: list = [None] * N
    for i in range(N):
        z = depth * masks[i]
        valid = z > 0
        if int(valid.sum()) < min_points_threshold:
            continue

        zz = z[valid]
        pts = np.stack([ray_x[valid] * zz, ray_y[valid] * zz, zz], axis=-1)
        cols = rgb_flat[valid.reshape(-1)] if rgb_flat is not None else None

        pts, cols = dynamic_downsample(pts, colors=cols, target=obj_pcd_max_points)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
        if cols is not None:
            pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64))

        if trans_pose is not None:
            pcd.transform(trans_pose)

        bbox = get_bounding_box(spatial_sim_type, pcd)
        if bbox.volume() < 1e-6:
            continue

        processed_objects[i] = {"pcd": pcd, "bbox": bbox}

    return processed_objects
