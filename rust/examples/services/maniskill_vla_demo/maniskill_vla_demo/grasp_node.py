#!/usr/bin/env python3
"""Grasp manipulation node — Open3D geometric grasping + SAM2 segmentation.

No C++ compilation required.

Backends (GRASP_BACKEND env var):
  open3d    (default)  YOLOE bbox → SAM2 segmentation → clean object
                       point cloud → Open3D surface-normal antipodal grasp
                       sampling. SAM2 model auto-downloaded from HuggingFace
                       on first run (~340 MB, tiny variant).
                       Override model with SAM2_MODEL env var.
  heuristic            Top-down geometric fallback, no model required.

MCP tools (registered via @mcp.tool()):
  predict_grasps(text_prompt, max_grasps, score_threshold)
  execute_grasp(grasp_id, lift_height, approach_steps, grasp_steps)
  pick_object(object_label, lift_height)

Registration:
  node_id:     com.robonix.demo.grasp
  namespace:   robonix/skill/manipulation
  kind:        skill
  contract_id: robonix/skill/manipulation/tools
"""

import json
import logging
import os
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

for _n in (
    "mcp", "mcp.server", "mcp.server.streamable_http",
    "mcp.server.streamable_http_manager",
    "httpx", "httpcore", "uvicorn", "uvicorn.access",
):
    logging.getLogger(_n).setLevel(logging.WARNING)


# ── Path helpers ──────────────────────────────────────────────────────────────

def _ensure_proto_paths() -> None:
    pkg = Path(__file__).resolve().parent
    sys.path.insert(0, str(pkg))
    d = pkg
    while d.parent != d:
        pg = d / "proto_gen"
        if pg.is_dir() and (pg / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent


_ensure_proto_paths()

from mcp.server.fastmcp import FastMCP  # noqa: E402
import grpc  # noqa: E402
import robonix_runtime_pb2 as pb  # noqa: E402
import robonix_runtime_pb2_grpc as pb_grpc  # noqa: E402
import maniskill_env_pb2 as env_pb  # noqa: E402
import maniskill_env_pb2_grpc as env_pb_grpc  # noqa: E402

# ── Global state ──────────────────────────────────────────────────────────────

mcp = FastMCP("grasp")

_env_stub: env_pb_grpc.EnvDataServiceStub | None = None
_backend: str = "heuristic"
_sam2_predictor = None       # SAM2ImagePredictor, loaded on demand
_sam2_device: str = "cpu"
_yolo_model = None           # YOLOE for bbox detection
_yolo_device: str | int = "cpu"
_grasp_cache: list[dict] = []
_rerun_enabled: bool = False
_require_detection = os.environ.get("GRASP_REQUIRE_DETECTION", "1").strip().lower() not in ("0", "false", "no")


def _log(msg: str) -> None:
    print(f"[grasp] {msg}", file=sys.stderr)


# ── Rotation helpers ──────────────────────────────────────────────────────────

def _scipy_rotation():
    from scipy.spatial.transform import Rotation
    return Rotation


def _quat_delta_rotvec(q_cur_xyzw: np.ndarray, q_tar_xyzw: np.ndarray) -> np.ndarray:
    R = _scipy_rotation()
    delta = R.from_quat(q_tar_xyzw) * R.from_quat(q_cur_xyzw).inv()
    return delta.as_rotvec()


def _rotmat_to_quat_xyzw(mat: np.ndarray) -> np.ndarray:
    R = _scipy_rotation()
    return R.from_matrix(mat).as_quat()


def _top_down_rotmat() -> np.ndarray:
    """Rotation matrix for a top-down grasp (EE Z-axis pointing world -Z)."""
    return np.array([
        [1.0,  0.0,  0.0],
        [0.0, -1.0,  0.0],
        [0.0,  0.0, -1.0],
    ])


# ── Point cloud helpers ───────────────────────────────────────────────────────

def _build_pointcloud(
    rgb: np.ndarray,
    depth: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
    depth_min: float = 0.1,
    depth_max: float = 3.0,
    mask: np.ndarray | None = None,
    max_pts: int = 20000,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (Nx3 pts, Nx3 colors) in camera frame."""
    h, w = depth.shape
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    valid = (depth > depth_min) & (depth < depth_max)
    if mask is not None:
        valid &= mask
    z = depth[valid]
    x = (us[valid] - cx) / fx * z
    y = (vs[valid] - cy) / fy * z
    pts = np.stack([x, y, z], axis=1).astype(np.float32)
    clr = rgb[valid].astype(np.float32) / 255.0
    if len(pts) > max_pts:
        idx = np.random.choice(len(pts), max_pts, replace=False)
        pts, clr = pts[idx], clr[idx]
    return pts, clr


def _transform_to_world(pts_cam: np.ndarray, camera_pose_flat: list[float]) -> np.ndarray:
    T = np.array(camera_pose_flat, dtype=np.float64).reshape(4, 4)
    n = pts_cam.shape[0]
    hom = np.hstack([pts_cam.astype(np.float64), np.ones((n, 1))])
    return (T @ hom.T).T[:, :3].astype(np.float32)


# ── YOLOE bbox detection ──────────────────────────────────────────────────────

def _detect_bbox(
    rgb: np.ndarray,
    text_prompt: str,
    threshold: float = 0.25,
) -> tuple[tuple[int, int, int, int] | None, str]:
    """Return (bbox_xyxy, label) of best YOLOE detection, or (None, prompt)."""
    if _yolo_model is None:
        return None, text_prompt

    raw_classes = [c.strip() for c in text_prompt.replace(",", ".").split(".") if c.strip()] or ["object"]
    classes: list[str] = []
    for item in raw_classes:
        for candidate in (item, item.split()[-1].strip(), "object"):
            if candidate and candidate not in classes:
                classes.append(candidate)
    if _yolo_device != "cpu":
        _log("forcing YOLOE to CPU for dynamic class updates")
        _yolo_model.to("cpu")
        globals()["_yolo_device"] = "cpu"
    _yolo_model.set_classes(classes)
    try:
        results = _yolo_model.predict(
            rgb, conf=threshold, verbose=False, device=_yolo_device, half=False
        )
    except RuntimeError as exc:
        msg = str(exc).lower()
        if ("same device" in msg or ("cuda" in msg and "cpu" in msg)) and _yolo_device != "cpu":
            print(
                "[grasp] YOLOE CUDA/CPU mismatch — falling back to CPU for "
                "dynamic open-vocabulary detection",
                file=sys.stderr,
            )
            _yolo_model.to("cpu")
            globals()["_yolo_device"] = "cpu"
            _yolo_model.set_classes(classes)
            results = _yolo_model.predict(
                rgb, conf=threshold, verbose=False, device="cpu", half=False
            )
        else:
            raise
    r = results[0]
    if r.boxes is None or len(r.boxes) == 0:
        _log(f"bbox detect miss for prompt={text_prompt!r}")
        return None, text_prompt

    best_i = int(r.boxes.conf.argmax())
    box = r.boxes.xyxy[best_i].cpu().tolist()
    ci = int(r.boxes.cls[best_i].cpu())
    label = classes[ci] if 0 <= ci < len(classes) else text_prompt
    _log(
        f"bbox detect hit label={label!r} score={float(r.boxes.conf[best_i].cpu()):.3f} "
        f"box={[round(float(v), 1) for v in box]}"
    )
    return (int(box[0]), int(box[1]), int(box[2]), int(box[3])), label


# ── SAM2 segmentation ─────────────────────────────────────────────────────────

def _load_sam2() -> bool:
    """Load SAM2ImagePredictor from HuggingFace. Returns True on success."""
    global _sam2_predictor, _sam2_device
    try:
        import torch
        print(
            f"[grasp] python={sys.executable} CUDA_VISIBLE_DEVICES={os.environ.get('CUDA_VISIBLE_DEVICES', '<unset>')}",
            file=sys.stderr,
        )
        try:
            print(
                f"[grasp] pre-sam2 torch cuda_available={bool(torch.cuda.is_available())} "
                f"device_count={int(torch.cuda.device_count())}",
                file=sys.stderr,
            )
        except Exception as e:
            print(f"[grasp] pre-sam2 torch CUDA probe failed: {e}", file=sys.stderr)
        from sam2.sam2_image_predictor import SAM2ImagePredictor
    except ImportError as e:
        print(f"[grasp] SAM2 not available: {e}", file=sys.stderr)
        print("[grasp]   Install in package env: uv sync", file=sys.stderr)
        print(
            "[grasp]   Verify with: uv run python -c \"from sam2.sam2_image_predictor import SAM2ImagePredictor; print('ok')\"",
            file=sys.stderr,
        )
        return False

    try:
        cuda_available = bool(torch.cuda.is_available())
        device_count = int(torch.cuda.device_count())
        device_name = torch.cuda.get_device_name(0) if cuda_available and device_count > 0 else "n/a"
        print(
            f"[grasp] torch cuda_available={cuda_available} device_count={device_count} "
            f"device_name={device_name}",
            file=sys.stderr,
        )
    except Exception as e:
        print(f"[grasp] torch CUDA probe failed: {e}", file=sys.stderr)

    model_id = os.environ.get("SAM2_MODEL", "facebook/sam2.1-hiera-tiny")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    _sam2_device = device

    if os.environ.get("SAM2_FORCE_CPU", "").strip() in ("1", "true", "yes"):
        device = "cpu"
        _sam2_device = device

    print(f"[grasp] loading SAM2 ({model_id}) on {device}…", file=sys.stderr)
    try:
        pred = SAM2ImagePredictor.from_pretrained(model_id, device=device)
    except Exception as e:
        print(f"[grasp] SAM2 load failed: {e}", file=sys.stderr)
        print(
            "[grasp]   Verify package env/network/model cache with: "
            "uv run python -c \"from sam2.sam2_image_predictor import SAM2ImagePredictor; "
            "SAM2ImagePredictor.from_pretrained('" + model_id + "', device='" + device + "'); print('ok')\"",
            file=sys.stderr,
        )
        return False

    def _warmup_on(target_device: str) -> None:
        pred.model.to(target_device)
        dummy = np.zeros((64, 64, 3), dtype=np.uint8)
        with torch.inference_mode():
            pred.set_image(dummy)

    try:
        _warmup_on(device)
    except Exception as e:
        msg = str(e).lower()
        if device == "cuda" and (
            "no cuda gpus are available" in msg
            or "cuda" in msg
            or "device" in msg
        ):
            print(
                f"[grasp] SAM2 CUDA init failed ({e}) — retrying on CPU",
                file=sys.stderr,
            )
            device = "cpu"
            _sam2_device = "cpu"
            try:
                _warmup_on("cpu")
            except Exception as e2:
                print(f"[grasp] SAM2 load failed after CPU fallback: {e2}", file=sys.stderr)
                return False
        else:
            print(f"[grasp] SAM2 load failed: {e}", file=sys.stderr)
            return False

    _sam2_predictor = pred
    print(f"[grasp] SAM2 ready on {device}", file=sys.stderr)
    return True


def _sam2_segment(rgb: np.ndarray, bbox: tuple[int, int, int, int]) -> np.ndarray | None:
    """Segment object with SAM2 using bbox prompt. Returns bool mask (H×W) or None."""
    global _sam2_device
    if _sam2_predictor is None:
        return None
    import torch

    def _run() -> np.ndarray:
        box_arr = np.array(list(bbox), dtype=np.float32)
        with torch.inference_mode():
            _sam2_predictor.set_image(rgb)
            masks, _, _ = _sam2_predictor.predict(
                point_coords=None,
                point_labels=None,
                box=box_arr,
                multimask_output=False,
            )
        return masks[0].astype(bool)

    try:
        mask = _run()
        _log(f"SAM2 mask ready pixels={int(mask.sum())}")
        return mask
    except RuntimeError as e:
        msg = str(e).lower()
        if (
            "same device" in msg
            or ("cuda" in msg and "cpu" in msg)
        ) and _sam2_device == "cuda":
            print(
                "[grasp] SAM2 CUDA/CPU mismatch — moving model to CPU and retrying "
                "(or set SAM2_FORCE_CPU=1 to always use CPU).",
                file=sys.stderr,
            )
            _sam2_predictor.model.to("cpu")
            _sam2_device = "cpu"
            try:
                return _run()
            except Exception as e2:
                print(f"[grasp] SAM2 segment failed after CPU fallback: {e2}", file=sys.stderr)
                return None
        print(f"[grasp] SAM2 segment failed: {e}", file=sys.stderr)
        return None
    except Exception as e:
        print(f"[grasp] SAM2 segment failed: {e}", file=sys.stderr)
        return None


# ── Open3D surface-normal antipodal grasp sampling ────────────────────────────

def _open3d_grasp_from_cloud(
    pts_cam: np.ndarray,
    camera_pose_flat: list[float],
    label: str,
    gripper_half_width: float = 0.045,
    finger_depth: float = 0.05,
    finger_thickness: float = 0.012,
    n_surface_samples: int = 80,
    n_rotation_samples: int = 12,
    max_grasps: int = 10,
    score_threshold: float = 0.0,
    nms_dist: float = 0.02,
) -> list[dict]:
    """6-DoF grasp candidates via Open3D surface normals + antipodal scoring."""
    try:
        import open3d as o3d
    except ImportError:
        print("[grasp] open3d not available", file=sys.stderr)
        return []

    if len(pts_cam) < 20:
        return []

    T = np.array(camera_pose_flat, dtype=np.float64).reshape(4, 4)
    R_wc = T[:3, :3]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_cam.astype(np.float64))
    pcd = pcd.voxel_down_sample(voxel_size=0.005)
    pts = np.asarray(pcd.points, dtype=np.float32)
    if len(pts) < 10:
        return []

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.025, max_nn=30)
    )
    pcd.orient_normals_towards_camera_location(camera_location=np.array([0.0, 0.0, 0.0]))
    normals = np.asarray(pcd.normals, dtype=np.float32)

    sample_idx = np.random.choice(len(pts), min(len(pts), n_surface_samples), replace=False)
    candidates: list[dict] = []

    for si in sample_idx:
        pt = pts[si]
        n = normals[si]
        if np.linalg.norm(n) < 0.5:
            continue
        n = n / np.linalg.norm(n)

        # Approach = inward normal (gripper moves toward object surface)
        approach = -n

        ref = np.array([0.0, 0.0, 1.0]) if abs(approach[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        closing0 = np.cross(approach, ref)
        norm_c = np.linalg.norm(closing0)
        if norm_c < 1e-6:
            continue
        closing0 /= norm_c
        lateral0 = np.cross(approach, closing0)

        # Grasp center: surface point pulled back half a finger depth
        t_cam = pt - approach * (finger_depth * 0.5)
        diff = pts - t_cam  # Nx3

        for k in range(n_rotation_samples):
            angle = k * np.pi / n_rotation_samples
            c_a, s_a = float(np.cos(angle)), float(np.sin(angle))

            def _rot(v: np.ndarray) -> np.ndarray:
                return c_a * v + s_a * np.cross(approach, v) + (1 - c_a) * np.dot(approach, v) * approach

            closing = _rot(closing0)
            lateral = _rot(lateral0)

            d_close = diff @ closing
            d_lat   = diff @ lateral
            d_app   = diff @ approach

            in_bite = (
                (np.abs(d_close) <= gripper_half_width) &
                (np.abs(d_lat)   <= finger_thickness) &
                (d_app >= 0.0) & (d_app <= finger_depth)
            )
            n_bite = int(np.sum(in_bite))
            if n_bite < 2:
                continue

            jaw_band = 0.010
            n_l = int(np.sum(in_bite & (d_close >  gripper_half_width - jaw_band)))
            n_r = int(np.sum(in_bite & (d_close < -gripper_half_width + jaw_band)))
            antipodal = 2.0 * min(n_l, n_r) / max(1, n_l + n_r)

            blocked = (
                (np.abs(d_close) <= gripper_half_width + 0.005) &
                (np.abs(d_lat)   <= finger_thickness + 0.005) &
                (d_app < 0.0) & (d_app > -0.08)
            )
            clearance = max(0.0, 1.0 - int(np.sum(blocked)) * 0.15)

            score = float(antipodal * min(1.0, n_bite / 20.0) * clearance)
            if score < score_threshold:
                continue

            # rotation_world cols: [closing, lateral, approach] in world frame
            R_cam   = np.stack([closing, lateral, approach], axis=1)
            R_world = R_wc @ R_cam
            t_world = (T @ np.array([*t_cam.astype(np.float64), 1.0]))[:3]

            candidates.append({
                "score": score,
                "translation_world": t_world.tolist(),
                "rotation_world": R_world.flatten().tolist(),
                "approach_world": R_world[:, 2].tolist(),
                "width": round(gripper_half_width * 2, 4),
                "label": label,
                "backend": "open3d",
            })

    candidates.sort(key=lambda x: -x["score"])

    selected: list[dict] = []
    for c in candidates:
        if len(selected) >= max_grasps:
            break
        t = np.array(c["translation_world"])
        if any(np.linalg.norm(t - np.array(s["translation_world"])) < nms_dist
               for s in selected):
            continue
        selected.append(c)

    for i, g in enumerate(selected):
        g["id"] = i

    return selected


# ── Backend: heuristic (top-down fallback) ────────────────────────────────────

def _heuristic_grasps(
    rgb: np.ndarray,
    depth: np.ndarray,
    fx: float, fy: float, cx_px: float, cy_px: float,
    camera_pose_flat: list[float],
    text_prompt: str,
    max_grasps: int,
    score_threshold: float,
) -> list[dict]:
    bbox, label = _detect_bbox(rgb, text_prompt)
    _log_grasp_vision_rerun(rgb, bbox, None, label)

    if bbox is not None:
        x1, y1, x2, y2 = bbox
        u = (x1 + x2) // 2
        v = (y1 + y2) // 2
        h, w = depth.shape
        u, v = max(0, min(u, w - 1)), max(0, min(v, h - 1))
        z = float(depth[v, u])
        if not (0.05 < z < 5.0):
            patch = depth[max(0,y1):min(h,y2), max(0,x1):min(w,x2)]
            valid_d = patch[(patch > 0.05) & (patch < 5.0)]
            z = float(np.median(valid_d)) if len(valid_d) > 0 else 1.0
        px = (u - cx_px) / fx * z
        py = (v - cy_px) / fy * z
        center_cam = np.array([px, py, z], dtype=np.float32)
    else:
        valid = (depth > 0.3) & (depth < 2.0)
        z_med = float(np.median(depth[valid])) if np.any(valid) else 1.0
        center_cam = np.array([0.0, 0.0, z_med], dtype=np.float32)
        label = text_prompt

    T = np.array(camera_pose_flat, dtype=np.float64).reshape(4, 4)
    center_world = (T @ np.array([*center_cam, 1.0], dtype=np.float64))[:3].astype(np.float32)
    rot_world = _top_down_rotmat()
    _log(
        f"heuristic grasp seed label={label!r} bbox={bbox} "
        f"center_world={np.round(center_world.astype(np.float64), 4).tolist()}"
    )

    grasps = []
    angles = np.linspace(0, np.pi / 2, min(max_grasps, 4), endpoint=False)
    for i, angle in enumerate(angles):
        R_z = np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle),  np.cos(angle), 0],
            [0, 0, 1],
        ])
        rot_i = R_z @ rot_world
        score = round(1.0 - i * 0.05, 4)
        if score < score_threshold:
            break
        grasps.append({
            "id": i,
            "label": label,
            "score": score,
            "translation_world": center_world.tolist(),
            "rotation_world": rot_i.flatten().tolist(),
            "approach_world": rot_i[:, 2].tolist(),
            "width": 0.08,
            "backend": "heuristic",
        })
    return grasps[:max_grasps]


def _masked_topdown_grasps(
    rgb: np.ndarray,
    depth: np.ndarray,
    fx: float, fy: float, cx_px: float, cy_px: float,
    camera_pose_flat: list[float],
    text_prompt: str,
    max_grasps: int,
    score_threshold: float,
    bbox: tuple[int, int, int, int] | None,
    mask: np.ndarray | None,
    label: str,
) -> list[dict]:
    h, w = depth.shape
    if mask is not None and bool(np.any(mask)):
        ys, xs = np.nonzero(mask)
        u = int(np.round(xs.mean()))
        v = int(np.round(ys.mean()))
        valid_d = depth[mask & (depth > 0.05) & (depth < 5.0)]
        z = float(np.median(valid_d)) if len(valid_d) > 0 else float(depth[v, u])
    elif bbox is not None:
        x1, y1, x2, y2 = bbox
        u = int(np.round((x1 + x2) * 0.5))
        v = int(np.round((y1 + y2) * 0.5))
        patch = depth[max(0, y1):min(h, y2), max(0, x1):min(w, x2)]
        valid_d = patch[(patch > 0.05) & (patch < 5.0)]
        z = float(np.median(valid_d)) if len(valid_d) > 0 else float(depth[v, u])
    else:
        return _heuristic_grasps(
            rgb, depth, fx, fy, cx_px, cy_px,
            camera_pose_flat, text_prompt, max_grasps, score_threshold,
        )

    u, v = max(0, min(u, w - 1)), max(0, min(v, h - 1))
    if not (0.05 < z < 5.0):
        return _heuristic_grasps(
            rgb, depth, fx, fy, cx_px, cy_px,
            camera_pose_flat, text_prompt, max_grasps, score_threshold,
        )

    px = (u - cx_px) / fx * z
    py = (v - cy_px) / fy * z
    center_cam = np.array([px, py, z], dtype=np.float32)
    T = np.array(camera_pose_flat, dtype=np.float64).reshape(4, 4)
    center_world = (T @ np.array([*center_cam, 1.0], dtype=np.float64))[:3].astype(np.float32)
    center_world[2] = max(center_world[2], 0.01)
    rot_world = _top_down_rotmat()
    _log(
        f"topdown grasp seed label={label!r} uv=({u},{v}) "
        f"center_world={np.round(center_world.astype(np.float64), 4).tolist()}"
    )

    grasps = []
    angles = np.linspace(0, np.pi / 2, min(max_grasps, 4), endpoint=False)
    for i, angle in enumerate(angles):
        R_z = np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle),  np.cos(angle), 0],
            [0, 0, 1],
        ])
        rot_i = R_z @ rot_world
        score = round(1.0 - i * 0.05, 4)
        if score < score_threshold:
            break
        grasps.append({
            "id": i,
            "label": label,
            "score": score,
            "translation_world": center_world.tolist(),
            "rotation_world": rot_i.flatten().tolist(),
            "approach_world": rot_i[:, 2].tolist(),
            "width": 0.08,
            "backend": "topdown",
        })
    return grasps[:max_grasps]


# ── Backend: Open3D + SAM2 ────────────────────────────────────────────────────

def _open3d_predict(
    rgb: np.ndarray,
    depth: np.ndarray,
    fx: float, fy: float, cx_px: float, cy_px: float,
    camera_pose_flat: list[float],
    text_prompt: str,
    max_grasps: int,
    score_threshold: float,
) -> list[dict]:
    bbox, label = _detect_bbox(rgb, text_prompt)

    if bbox is not None:
        mask = _sam2_segment(rgb, bbox)
        if mask is None:
            # bbox rectangle fallback if SAM2 fails
            x1, y1, x2, y2 = bbox
            h, w = depth.shape
            mask = np.zeros((h, w), dtype=bool)
            mask[max(0,y1):min(h,y2), max(0,x1):min(w,x2)] = True
    else:
        if _require_detection:
            _log(f"target detection required; aborting grasp prediction for prompt={text_prompt!r}")
            _log_grasp_vision_rerun(rgb, None, None, text_prompt)
            return []
        mask = None
        label = text_prompt

    _log_grasp_vision_rerun(rgb, bbox, mask, label)

    prefer_topdown = (
        os.environ.get("GRASP_PREFER_TOPDOWN", "1").strip().lower() not in ("0", "false", "no")
        and any(tok in (text_prompt or "").lower() for tok in ("cube", "block"))
    )
    if prefer_topdown:
        grasps = _masked_topdown_grasps(
            rgb, depth, fx, fy, cx_px, cy_px,
            camera_pose_flat, text_prompt, max_grasps, score_threshold,
            bbox, mask, label,
        )
        if grasps:
            best = grasps[0]
            _log(
                f"predicted {len(grasps)} topdown grasp(s), best label={best.get('label')!r} "
                f"score={float(best.get('score', 0.0)):.3f} "
                f"t={np.round(np.array(best.get('translation_world', []), dtype=np.float64), 4).tolist()}"
            )
            return grasps

    pts_cam, _ = _build_pointcloud(
        rgb, depth, fx, fy, cx_px, cy_px, mask=mask, max_pts=20000
    )
    _log(
        f"grasp point cloud label={label!r} bbox={bbox} mask_pixels="
        f"{int(mask.sum()) if mask is not None else 0} points={len(pts_cam)}"
    )

    if len(pts_cam) < 20:
        print("[grasp] too few object points, falling back to heuristic", file=sys.stderr)
        return _heuristic_grasps(rgb, depth, fx, fy, cx_px, cy_px,
                                  camera_pose_flat, text_prompt, max_grasps, score_threshold)

    grasps = _open3d_grasp_from_cloud(
        pts_cam, camera_pose_flat, label,
        max_grasps=max_grasps,
        score_threshold=score_threshold,
    )

    if not grasps:
        print("[grasp] Open3D found no valid grasps, falling back to heuristic", file=sys.stderr)
        return _heuristic_grasps(rgb, depth, fx, fy, cx_px, cy_px,
                                  camera_pose_flat, text_prompt, max_grasps, score_threshold)
    best = grasps[0]
    _log(
        f"predicted {len(grasps)} grasp(s), best label={best.get('label')!r} "
        f"score={float(best.get('score', 0.0)):.3f} "
        f"t={np.round(np.array(best.get('translation_world', []), dtype=np.float64), 4).tolist()}"
    )
    return grasps


# ── Rerun visualization ───────────────────────────────────────────────────────

def _log_grasps_rerun(grasps: list[dict], executing_id: int | None = None) -> None:
    if not _rerun_enabled or not grasps:
        return
    try:
        import rerun as rr

        positions = np.array([g["translation_world"] for g in grasps], dtype=np.float32)
        colors = []
        for i, g in enumerate(grasps):
            if i == executing_id:
                colors.append([30, 120, 255, 255])
            else:
                s = float(g["score"])
                colors.append([int((1 - s) * 220), int(s * 220), 0, 200])

        rr.log("world/grasps/positions", rr.Points3D(
            positions=positions,
            colors=colors,
            radii=0.015,
            labels=[f"#{g['id']} {g['label']} {g['score']:.3f}" for g in grasps],
        ))

        vectors = np.array([g["approach_world"] for g in grasps], dtype=np.float32) * 0.10
        rr.log("world/grasps/approaches", rr.Arrows3D(
            origins=positions, vectors=vectors,
            colors=[[0, 200, 100, 200]] * len(grasps),
        ))

        strips, seg_colors = [], []
        for i, g in enumerate(grasps):
            t   = np.array(g["translation_world"], dtype=np.float32)
            rot = np.array(g["rotation_world"],    dtype=np.float32).reshape(3, 3)
            w   = float(g.get("width", 0.09))
            closing = rot[:, 0]
            strips.append([(t - closing * w / 2).tolist(), (t + closing * w / 2).tolist()])
            seg_colors.append([30, 120, 255, 220] if i == executing_id else [220, 200, 0, 150])

        rr.log("world/grasps/gripper_widths", rr.LineStrips3D(strips=strips, colors=seg_colors))
        _log(f"rerun logged {len(grasps)} grasp pose(s)")
    except Exception as e:
        print(f"[grasp] rerun log error: {e}", file=sys.stderr)


def _log_grasp_vision_rerun(
    rgb: np.ndarray,
    bbox: tuple[int, int, int, int] | None,
    mask: np.ndarray | None,
    label: str,
) -> None:
    """YOLO box + SAM2 mask on image; complements 3D paths under ``world/grasps/*``."""
    if not _rerun_enabled:
        return
    try:
        import rerun as rr

        rr.log("grasp/camera/rgb", rr.Image(rgb, color_model="RGB"))
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            rr.log(
                "grasp/camera/yolo_box",
                rr.Boxes2D(
                    mins=np.array([[float(x1), float(y1)]], dtype=np.float32),
                    sizes=np.array([[float(x2 - x1), float(y2 - y1)]], dtype=np.float32),
                    labels=[label],
                    colors=[[255, 140, 0, 255]],
                ),
            )
        else:
            rr.log("grasp/camera/yolo_box", rr.Clear(recursive=False))

        if mask is not None and mask.shape[:2] == rgb.shape[:2]:
            overlay = rgb.copy()
            m = mask
            overlay[m] = (overlay[m].astype(np.float32) * 0.45 + np.array([0, 255, 100], dtype=np.float32) * 0.55).astype(
                np.uint8
            )
            rr.log("grasp/camera/sam2_overlay", rr.Image(overlay, color_model="RGB"))
            rr.log(
                "grasp/camera/sam2_mask",
                rr.SegmentationImage(np.where(mask, np.uint16(1), np.uint16(0))),
            )
        else:
            rr.log("grasp/camera/sam2_overlay", rr.Clear(recursive=False))
            rr.log("grasp/camera/sam2_mask", rr.Clear(recursive=False))
        _log(
            f"rerun vision logged bbox={'yes' if bbox is not None else 'no'} "
            f"sam2_mask={'yes' if mask is not None else 'no'}"
        )
    except Exception as e:
        print(f"[grasp] rerun vision log error: {e}", file=sys.stderr)


# ── Grasp execution ───────────────────────────────────────────────────────────

def _get_obs() -> env_pb.Observation:
    return _env_stub.GetObs(env_pb.Empty())


def _step(action_12: list[float]) -> env_pb.StepResult:
    while len(action_12) < 13:
        action_12.append(0.0)
    repeat = max(1, min(int(os.environ.get("GRASP_ACTION_REPEAT", "4")), 20))
    result = None
    for _ in range(repeat):
        result = _env_stub.Step(env_pb.Action(values=action_12[:13]))
        if bool(result.done):
            break
    return result


def _make_action(
    arm_delta: list[float] | np.ndarray,
    gripper_cmd: float,
    body: list[float] | None = None,
    base: list[float] | None = None,
) -> list[float]:
    arm = [float(v) for v in list(arm_delta)[:6]]
    while len(arm) < 6:
        arm.append(0.0)
    body_vals = [0.0, 0.0, 0.0] if body is None else [float(v) for v in list(body)[:3]]
    while len(body_vals) < 3:
        body_vals.append(0.0)
    base_vals = [0.0, 0.0] if base is None else [float(v) for v in list(base)[:2]]
    while len(base_vals) < 2:
        base_vals.append(0.0)
    # Fetch pd_ee_delta_pose actual layout:
    # arm(6) -> gripper(2) -> body(3) -> base(2)
    return arm + [float(gripper_cmd), float(gripper_cmd)] + body_vals + base_vals


def _reset_env_if_done() -> bool:
    obs = _get_obs()
    if not bool(getattr(obs, "done", False)):
        return False
    _log("environment is done; resetting before grasp")
    _env_stub.Reset(env_pb.Empty())
    return True


def _parse_info_json(info_json: str) -> dict:
    try:
        data = json.loads(info_json or "{}")
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _tcp_position() -> np.ndarray | None:
    obs = _get_obs()
    if len(obs.tcp_pose) >= 3:
        return np.array(obs.tcp_pose[:3], dtype=np.float64)
    return None


def _drive_to_position(
    target_pos: np.ndarray,
    gripper_cmd: float,
    max_steps: int,
    pos_tol: float = 0.018,
    pos_gain: float = 2.5,
    max_delta: float = 0.06,
    target_rot_mat: np.ndarray | None = None,
    rot_gain: float = 1.0,
    max_rot_delta: float = 0.25,
) -> tuple[float, bool, dict]:
    dist = 999.0
    done = False
    last_info: dict = {}
    for _ in range(max_steps):
        obs = _get_obs()
        done = bool(obs.done)
        if len(obs.tcp_pose) >= 3:
            cur_pos = np.array(obs.tcp_pose[:3], dtype=np.float64)
            err = target_pos.astype(np.float64) - cur_pos
            dist = float(np.linalg.norm(err))
            if dist < pos_tol:
                break
            step_size = min(dist * pos_gain * 0.5, max_delta)
            pos_delta = (err / dist) * step_size
        else:
            pos_delta = np.zeros(3)
            dist = 0.0

        rot_delta = np.zeros(3)
        if target_rot_mat is not None and len(obs.tcp_pose) >= 7:
            try:
                cur_quat = np.array(obs.tcp_pose[3:7], dtype=np.float64)
                tar_quat = _rotmat_to_quat_xyzw(target_rot_mat)
                rv = _quat_delta_rotvec(cur_quat, tar_quat)
                rot_delta = np.clip(rv * rot_gain, -max_rot_delta, max_rot_delta)
            except Exception:
                pass

        action = _make_action(list(pos_delta) + list(rot_delta), gripper_cmd)
        result = _step(action)
        done = bool(result.done)
        last_info = _parse_info_json(result.info_json)
        if done:
            break

    return dist, done, last_info


def _hold_gripper(gripper_cmd: float, steps: int) -> bool:
    for _ in range(steps):
        result = _step(_make_action([0.0] * 6, gripper_cmd))
        if bool(result.done):
            return True
    return False


def _execute_grasp_motion(
    grasp: dict,
    lift_height: float,
    approach_steps: int,
    grasp_steps: int,
) -> dict:
    t_world = np.array(grasp["translation_world"], dtype=np.float64)
    rot_mat = _top_down_rotmat()
    cur_tcp = _tcp_position()
    z_offset = float(os.environ.get("GRASP_TARGET_Z_OFFSET", "0.035"))
    hover_offset = float(os.environ.get("GRASP_HOVER_Z_OFFSET", "0.16"))
    descend_clearance = float(os.environ.get("GRASP_DESCEND_CLEARANCE", "0.05"))
    target_z = max(float(t_world[2]) + z_offset, 0.035)
    if cur_tcp is not None:
        hover_z = max(target_z + hover_offset, float(cur_tcp[2]) + 0.04, 0.22)
    else:
        hover_z = max(target_z + hover_offset, 0.22)
    hover_target = np.array([float(t_world[0]), float(t_world[1]), hover_z], dtype=np.float64)
    descend_target = np.array([float(t_world[0]), float(t_world[1]), max(target_z + descend_clearance, target_z)], dtype=np.float64)
    grasp_target = np.array([float(t_world[0]), float(t_world[1]), target_z], dtype=np.float64)
    total_reward = 0.0
    steps_taken = 0
    _log(
        "execute grasp topdown "
        f"seed_target={np.round(t_world, 4).tolist()} "
        f"hover={np.round(hover_target, 4).tolist()} "
        f"descend={np.round(descend_target, 4).tolist()} "
        f"grasp={np.round(grasp_target, 4).tolist()} "
        f"lift={lift_height:.3f}"
    )

    if _hold_gripper(+1.0, 10):
        steps_taken += 10
        _log("episode ended while opening gripper")
        return {"status": "done_early", "phase": "open_gripper", "total_reward": total_reward, "steps": steps_taken}
    steps_taken += 10

    align_steps = max(20, approach_steps)
    descend_steps = max(20, approach_steps)

    dist, done, info = _drive_to_position(
        hover_target,
        +1.0,
        align_steps,
        pos_tol=0.02,
        target_rot_mat=rot_mat,
        pos_gain=2.0,
        max_delta=0.04,
    )
    steps_taken += align_steps
    _log(f"hover phase done={done} dist={dist:.4f} info={info}")
    if done:
        return {
            "status": "done_early",
            "phase": "hover",
            "distance_to_target": round(dist, 4),
            "info": info,
            "total_reward": total_reward,
            "steps": steps_taken,
        }

    dist, done, info = _drive_to_position(
        descend_target,
        +1.0,
        descend_steps // 2,
        pos_tol=0.02,
        target_rot_mat=rot_mat,
        pos_gain=1.5,
        max_delta=0.025,
    )
    steps_taken += descend_steps // 2
    _log(f"pre_grasp phase done={done} dist={dist:.4f} info={info}")
    if done:
        return {
            "status": "done_early",
            "phase": "pre_grasp",
            "distance_to_target": round(dist, 4),
            "info": info,
            "total_reward": total_reward,
            "steps": steps_taken,
        }

    dist, done, info = _drive_to_position(
        grasp_target,
        +1.0,
        descend_steps // 2,
        pos_tol=0.012,
        target_rot_mat=rot_mat,
        pos_gain=1.2,
        max_delta=0.015,
    )
    steps_taken += descend_steps // 2
    _log(f"approach phase done={done} dist={dist:.4f} info={info}")
    if done:
        return {
            "status": "done_early",
            "phase": "approach",
            "distance_to_target": round(dist, 4),
            "info": info,
            "total_reward": total_reward,
            "steps": steps_taken,
        }

    if _hold_gripper(-1.0, grasp_steps):
        steps_taken += grasp_steps
        _log("episode ended while closing gripper")
        return {"status": "done_early", "phase": "close_gripper", "total_reward": total_reward, "steps": steps_taken}
    steps_taken += grasp_steps

    result = _step(_make_action([0.0] * 6, -1.0))
    info = _parse_info_json(result.info_json)
    is_grasped = bool(info.get("success", False))
    total_reward += float(result.reward);  steps_taken += 1
    _log(f"post_close reward={float(result.reward):.4f} done={bool(result.done)} info={info}")

    lift_target = np.array([float(grasp_target[0]), float(grasp_target[1]), hover_z + lift_height], dtype=np.float64)
    dist, done, info = _drive_to_position(
        lift_target,
        -1.0,
        30,
        pos_tol=0.02,
        target_rot_mat=rot_mat,
    )
    steps_taken += 30
    _log(f"lift phase done={done} dist={dist:.4f} info={info}")
    if done:
        return {
            "status": "done_early",
            "phase": "lift",
            "distance_to_target": round(dist, 4),
            "info": info,
            "is_grasped": is_grasped,
            "total_reward": round(total_reward, 4),
            "steps": steps_taken,
        }

    result = _step(_make_action([0.0] * 6, -1.0))
    info = _parse_info_json(result.info_json)
    is_grasped = is_grasped or bool(info.get("success", False))
    total_reward += float(result.reward);  steps_taken += 1
    _log(
        f"final hold reward={float(result.reward):.4f} done={bool(result.done)} "
        f"info={info} is_grasped={is_grasped}"
    )

    return {
        "status": "success" if is_grasped else "attempted",
        "is_grasped": is_grasped,
        "total_reward": round(total_reward, 4),
        "steps": steps_taken,
    }


# ── MCP tools ─────────────────────────────────────────────────────────────────

@mcp.tool()
def predict_grasps(
    text_prompt: str = "object",
    max_grasps: int = 5,
    score_threshold: float = 0.0,
) -> str:
    """Predict 6-DoF grasp candidates for the target object.

    Uses SAM2 segmentation + Open3D surface-normal antipodal sampling (default).
    Falls back to top-down heuristic if models are unavailable.

    Args:
        text_prompt: Open-vocabulary object description, e.g. "red cube" or "cup . bottle".
        max_grasps: Maximum candidates to return (default 5).
        score_threshold: Minimum score to include a candidate (default 0.0).

    Returns:
        JSON: { "grasps": [...], "count": int, "backend": str }
        Each grasp: { "id", "label", "score", "translation_world"[3],
                      "rotation_world"[9], "approach_world"[3], "width", "backend" }
    """
    global _grasp_cache

    if _env_stub is None:
        return json.dumps({"error": "env gRPC not connected"})
    _reset_env_if_done()
    try:
        obs = _get_obs()
    except Exception as e:
        return json.dumps({"error": f"GetObs failed: {e}"})

    if not obs.rgb:
        return json.dumps({"error": "no observation available"})
    if not obs.depth:
        return json.dumps({"error": "depth not available — set obs_mode=rgbd"})

    rgb   = np.frombuffer(obs.rgb,   dtype=np.uint8).reshape(obs.height, obs.width, 3)
    depth = np.frombuffer(obs.depth, dtype=np.float32).reshape(obs.height, obs.width)
    fx    = float(obs.fx) if obs.fx > 0 else float(obs.width)  / 2.0
    fy    = float(obs.fy) if obs.fy > 0 else float(obs.height) / 2.0
    cx_px = float(obs.cx) if obs.cx > 0 else float(obs.width)  / 2.0
    cy_px = float(obs.cy) if obs.cy > 0 else float(obs.height) / 2.0
    camera_pose = list(obs.camera_pose)

    if not camera_pose:
        return json.dumps({"error": "camera_pose not in observation"})

    if _backend == "open3d":
        grasps = _open3d_predict(
            rgb, depth, fx, fy, cx_px, cy_px,
            camera_pose, text_prompt, max_grasps, score_threshold,
        )
    else:
        grasps = _heuristic_grasps(
            rgb, depth, fx, fy, cx_px, cy_px,
            camera_pose, text_prompt, max_grasps, score_threshold,
        )

    for i, g in enumerate(grasps):
        g["id"] = i
    _grasp_cache = grasps

    _log_grasps_rerun(grasps)
    _log(
        f"predict_grasps prompt={text_prompt!r} backend={_backend} "
        f"count={len(grasps)}"
    )
    return json.dumps({"grasps": grasps, "count": len(grasps), "backend": _backend})


@mcp.tool()
def execute_grasp(
    grasp_id: int = 0,
    lift_height: float = 0.15,
    approach_steps: int = 40,
    grasp_steps: int = 15,
) -> str:
    """Execute a grasp candidate from the last predict_grasps call.

    Motion: open gripper → pre-grasp → approach → close → lift.

    Args:
        grasp_id: Index from predict_grasps output (default 0 = best).
        lift_height: Metres to lift after grasping (default 0.15).
        approach_steps: Control steps for the approach phase (default 40).
        grasp_steps: Steps to hold gripper closed (default 15).

    Returns:
        JSON: { "status", "is_grasped", "total_reward", "steps", "grasp" }
    """
    if _env_stub is None:
        return json.dumps({"error": "env gRPC not connected"})
    _reset_env_if_done()
    if not _grasp_cache:
        return json.dumps({"error": "no grasps cached — call predict_grasps first"})
    if not (0 <= grasp_id < len(_grasp_cache)):
        return json.dumps({"error": f"grasp_id {grasp_id} out of range (have {len(_grasp_cache)})"})

    grasp          = _grasp_cache[grasp_id]
    lift_height    = max(0.0, min(float(lift_height),    0.5))
    approach_steps = max(4,   min(int(approach_steps),  200))
    grasp_steps    = max(4,   min(int(grasp_steps),     100))

    _log_grasps_rerun(_grasp_cache, executing_id=grasp_id)
    _log(
        f"execute_grasp grasp_id={grasp_id} label={grasp.get('label')!r} "
        f"score={float(grasp.get('score', 0.0)):.3f}"
    )

    result = _execute_grasp_motion(grasp, lift_height, approach_steps, grasp_steps)
    result["grasp"] = {k: v for k, v in grasp.items() if k != "rotation_world"}
    return json.dumps(result)


@mcp.tool()
def pick_object(
    object_label: str = "object",
    lift_height: float = 0.15,
) -> str:
    """Detect object, predict grasps, and execute the best one.

    Equivalent to: predict_grasps(object_label) → execute_grasp(0).

    Args:
        object_label: Open-vocabulary object description (e.g. "red cube").
        lift_height: Lift height in metres after grasping (default 0.15).

    Returns:
        JSON execution result (same as execute_grasp).
    """
    pred = json.loads(predict_grasps(text_prompt=object_label, max_grasps=5))
    if "error" in pred:
        return json.dumps(pred)
    if pred["count"] == 0:
        return json.dumps({"error": f"no grasps found for '{object_label}'"})
    return execute_grasp(grasp_id=0, lift_height=lift_height)


# ── Boilerplate ───────────────────────────────────────────────────────────────

def _pick_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _mcp_tools_meta() -> str:
    import asyncio
    async def _list():
        return await mcp.list_tools()
    tools = asyncio.run(_list())
    return json.dumps({
        "tools": [
            {"name": t.name, "description": t.description or "",
             "input_schema": dict(t.inputSchema)}
            for t in tools
        ]
    })


def _start_mcp_http(port: int) -> None:
    import uvicorn
    app = mcp.streamable_http_app()
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


def _heartbeat_loop(stub, node_id: str) -> None:
    while True:
        time.sleep(15.0)
        try:
            stub.NodeHeartbeat(pb.NodeHeartbeatRequest(node_id=node_id))
        except Exception as e:
            print(f"[grasp] heartbeat failed: {e}", file=sys.stderr)


def _discover_env_grpc(stub, node_id: str) -> str:
    for attempt in range(30):
        try:
            resp = stub.NegotiateChannel(pb.NegotiateChannelRequest(
                consumer_id=node_id,
                provider_node_id="com.robonix.demo.maniskill",
                interface_name="env_data",
                transport="grpc",
            ))
            return resp.endpoint
        except grpc.RpcError:
            if attempt < 29:
                time.sleep(2)
            else:
                raise
    raise RuntimeError("could not discover env gRPC endpoint")


def _load_yolo() -> None:
    global _yolo_model, _yolo_device
    try:
        import torch
        from ultralytics import YOLOE
        from maniskill_vla_demo.yolo_weights import resolve_open_vocab_weights
    except ImportError:
        return

    weights = resolve_open_vocab_weights(
        os.environ.get("GRASP_YOLOE_WEIGHTS") or os.environ.get("GRASP_YOLO_WEIGHTS")
    )
    # Default to CPU because this node changes classes dynamically.
    device_req = os.environ.get("GRASP_YOLO_DEVICE", "cpu").strip().lower()
    if device_req == "cuda" and torch.cuda.is_available():
        device = "cuda"
    elif device_req == "auto" and torch.cuda.is_available():
        device = "cpu"
    else:
        device = "cpu"
    try:
        _yolo_model = YOLOE(weights)
        _yolo_model.set_classes(["object"])
        from PIL import Image as PILImage
        dummy = PILImage.new("RGB", (320, 240))
        _yolo_model.predict(dummy, conf=0.5, verbose=False,
                            device=0 if device == "cuda" else "cpu", half=False)
        _yolo_device = 0 if device == "cuda" else "cpu"
        print(f"[grasp] YOLOE ready (device={device})", file=sys.stderr)
    except Exception as e:
        print(f"[grasp] YOLOE unavailable ({e}), detection disabled", file=sys.stderr)
        _yolo_model = None


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    global _env_stub, _backend, _rerun_enabled

    _backend = os.environ.get("GRASP_BACKEND", "open3d").strip().lower()
    if _backend not in ("open3d", "heuristic"):
        print(f"[grasp] unknown backend '{_backend}', using open3d", file=sys.stderr)
        _backend = "open3d"

    _load_yolo()

    if _backend == "open3d":
        strict_sam2 = os.environ.get("GRASP_REQUIRE_SAM2", "1").strip().lower() not in ("0", "false", "no")
        if not _load_sam2():
            if strict_sam2:
                raise RuntimeError(
                    "SAM2 is required for GRASP_BACKEND=open3d but failed to load. "
                    "Fix the package environment or set GRASP_REQUIRE_SAM2=0 to allow fallback."
                )
            print(
                "[grasp] SAM2 unavailable — keeping open3d backend with bbox/full-scene fallback "
                "(GRASP_REQUIRE_SAM2=0)",
                file=sys.stderr,
            )

    print(f"[grasp] backend={_backend}", file=sys.stderr)

    server_addr = os.environ.get("ROBONIX_ATLAS", "localhost:50051")
    channel = grpc.insecure_channel(server_addr)
    stub = pb_grpc.RobonixRuntimeStub(channel)

    node_id = "com.robonix.demo.grasp"
    stub.RegisterNode(pb.RegisterNodeRequest(
        node_id=node_id,
        namespace="robonix/skill/manipulation",
        kind="skill",
    ))

    mcp_port = _pick_port()
    stub.DeclareInterface(pb.DeclareInterfaceRequest(
        node_id=node_id,
        name="grasp_tools",
        supported_transports=["mcp"],
        metadata_json=_mcp_tools_meta(),
        listen_port=mcp_port,
        contract_id="robonix/skill/manipulation/tools",
    ))

    threading.Thread(target=_heartbeat_loop, args=(stub, node_id), daemon=True).start()
    threading.Thread(target=_start_mcp_http, args=(mcp_port,), daemon=True).start()
    print(f"[grasp] MCP :{mcp_port} (serving)", file=sys.stderr)

    print("[grasp] discovering env gRPC endpoint…", file=sys.stderr)
    env_endpoint = _discover_env_grpc(stub, node_id)
    _env_stub = env_pb_grpc.EnvDataServiceStub(grpc.insecure_channel(env_endpoint))
    print(f"[grasp] connected to env gRPC at {env_endpoint}", file=sys.stderr)

    rerun_port = int(os.environ.get("RERUN_GRPC_PORT", "9877"))
    try:
        import rerun as rr
        rr.init("maniskill_vla_demo", recording_id="maniskill_demo", spawn=False)
        rr.connect_grpc(f"rerun+http://localhost:{rerun_port}/proxy")
        _rerun_enabled = True
        print(f"[grasp] Rerun connected (port {rerun_port})", file=sys.stderr)
    except Exception as e:
        print(f"[grasp] Rerun unavailable ({e}), visualization disabled", file=sys.stderr)

    print("[grasp] ready", file=sys.stderr)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
