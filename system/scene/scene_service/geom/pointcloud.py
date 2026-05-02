# SPDX-License-Identifier: MulanPSL-2.0
"""PointCloud2 ↔ numpy ↔ Open3D conversions. We never iterate raw
points in pure Python; all transforms vectorize through numpy and the
heavy ops (segment_plane, voxel_down) run on Open3D."""
from __future__ import annotations

import logging
import struct
from typing import Optional

import numpy as np

log = logging.getLogger(__name__)


# Open3D is optional. When unavailable we still expose `pc2_to_xyz`
# (numpy-only) so the lidar2D / depth-projection code paths keep
# working — only plane_extract is skipped.
try:
    import open3d as o3d  # type: ignore
    HAVE_OPEN3D = True
except Exception as e:  # noqa: BLE001
    o3d = None  # type: ignore
    HAVE_OPEN3D = False
    log.warning("[scene-geom] Open3D unavailable (%s); plane extraction will no-op", e)


# Standard PointField datatype constants (matches sensor_msgs/PointField).
_PF_FLOAT32 = 7


def pc2_to_xyz(
    *,
    data: bytes,
    point_step: int,
    row_step: int,
    width: int,
    height: int,
    fields: list[dict],
    is_dense: bool = True,
) -> Optional[np.ndarray]:
    """Decode a PointCloud2 buffer into an (N, 3) float32 array of xyz.
    Returns None if the message has no x/y/z fields or the buffer is
    truncated. Tolerates the common Webots / Gazebo layouts (xyzrgb,
    xyzi, xyz_padding).

    `fields` is a list of dicts with keys (name, offset, datatype,
    count) — matching sensor_msgs/PointField. We only look up x/y/z
    floats and ignore everything else."""
    name_to_offset: dict[str, int] = {}
    for f in fields:
        if f.get("datatype") == _PF_FLOAT32 and f.get("name") in ("x", "y", "z"):
            name_to_offset[f["name"]] = int(f["offset"])
    if {"x", "y", "z"} - name_to_offset.keys():
        return None

    n = width * height
    if n == 0 or len(data) < point_step * n:
        return None

    raw = np.frombuffer(data[: point_step * n], dtype=np.uint8).reshape(n, point_step)
    out = np.empty((n, 3), dtype=np.float32)
    for i, axis in enumerate(("x", "y", "z")):
        off = name_to_offset[axis]
        # struct-unpack-via-numpy is faster than per-point decoding
        # because we just slice bytes and reinterpret as float32.
        col = raw[:, off : off + 4].copy()
        out[:, i] = col.view(np.float32).reshape(n)

    if not is_dense:
        # Drop NaN rows (common in Webots when a ray didn't hit anything).
        finite = np.isfinite(out).all(axis=1)
        out = out[finite]
    return out


def transform_xyz(xyz: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """`xyz` (N,3) → R @ xyz + t. R is (3,3), t is (3,)."""
    return xyz @ R.T + t[None, :]


def voxel_downsample(xyz: np.ndarray, voxel_size: float = 0.05) -> np.ndarray:
    """Downsample to one point per `voxel_size` cube. No-op when
    Open3D is missing — caller should still be able to feed the raw
    cloud to plane extraction (just slower)."""
    if not HAVE_OPEN3D or xyz.shape[0] == 0:
        return xyz
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(pcd.points, dtype=np.float32)


# ── 2D laserscan helpers ────────────────────────────────────────────────────
def laserscan_to_xy(
    *,
    angle_min: float,
    angle_increment: float,
    ranges: np.ndarray,
    range_min: float,
    range_max: float,
) -> np.ndarray:
    """sensor_msgs/LaserScan → (N, 2) numpy array of XY hits in the
    sensor frame. Used by the lidar2d ingest path on Webots Tiago."""
    n = ranges.shape[0]
    angles = angle_min + np.arange(n, dtype=np.float32) * angle_increment
    valid = (ranges > range_min) & (ranges < range_max) & np.isfinite(ranges)
    angles = angles[valid]
    rs = ranges[valid].astype(np.float32)
    return np.stack([rs * np.cos(angles), rs * np.sin(angles)], axis=1)
