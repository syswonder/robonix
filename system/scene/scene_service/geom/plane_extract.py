# SPDX-License-Identifier: MulanPSL-2.0
"""Plane / surface extraction from point clouds. Used to seed the
`Surface` layer of the registry so `on(cup, table)` can resolve when
the perception layer didn't directly classify a "table" object.

v1: Open3D RANSAC plane segmentation, restricted to nearly-horizontal
planes (normal close to z-up). Multiple planes are extracted by
peeling: segment → record inliers → remove → repeat. Capped at 6
planes per call to keep the cost bounded.

When Open3D is missing this whole module no-ops (extract_planes
returns empty). Scene still functions; just no surface registration.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .pointcloud import HAVE_OPEN3D, voxel_downsample

if HAVE_OPEN3D:
    import open3d as o3d  # type: ignore

log = logging.getLogger(__name__)


@dataclass
class ExtractedPlane:
    """One planar segment, in the input frame (caller transforms to map)."""
    centroid: tuple[float, float, float]
    normal: tuple[float, float, float]
    extent_x: float       # along principal direction in plane
    extent_y: float       # along the other in-plane direction
    inlier_count: int


_HORIZONTAL_TOL_RAD = math.radians(15.0)  # |normal · z| > cos(tol)
_RANSAC_DIST_M = 0.02
_RANSAC_RANSAC_N = 3
_RANSAC_NUM_ITERS = 200
_MIN_INLIERS = 200       # below this, ignore the segment
_MAX_PLANES = 6          # peel up to N planes per call
_MIN_AREA = 0.10         # m² — reject tiny patches


def _plane_normal_horizontal(n: np.ndarray) -> bool:
    if not np.isfinite(n).all():
        return False
    nrm = np.linalg.norm(n)
    if nrm < 1e-6:
        return False
    nz = abs(n[2] / nrm)
    return nz >= math.cos(_HORIZONTAL_TOL_RAD)


def extract_planes(xyz: np.ndarray, *, voxel_size: float = 0.05) -> list[ExtractedPlane]:
    """RANSAC peel — return up to _MAX_PLANES horizontal planar
    segments. Caller is responsible for transforming results to map
    frame; we operate in whatever frame `xyz` is provided in."""
    if not HAVE_OPEN3D or xyz.shape[0] < _MIN_INLIERS:
        return []
    pts = voxel_downsample(xyz, voxel_size=voxel_size)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))

    out: list[ExtractedPlane] = []
    remaining = pcd
    for _ in range(_MAX_PLANES):
        if len(remaining.points) < _MIN_INLIERS:
            break
        try:
            model, inlier_idx = remaining.segment_plane(
                distance_threshold=_RANSAC_DIST_M,
                ransac_n=_RANSAC_RANSAC_N,
                num_iterations=_RANSAC_NUM_ITERS,
            )
        except Exception as e:  # noqa: BLE001
            log.warning("[scene-geom] segment_plane failed: %s", e)
            break
        if len(inlier_idx) < _MIN_INLIERS:
            break

        plane_pts = np.asarray(remaining.select_by_index(inlier_idx).points)
        a, b, c, _d = model
        normal = np.array([a, b, c], dtype=np.float64)
        if _plane_normal_horizontal(normal):
            centroid = plane_pts.mean(axis=0)
            ex, ey = _principal_extents(plane_pts - centroid[None, :])
            if ex * ey >= _MIN_AREA:
                normal_unit = normal / max(np.linalg.norm(normal), 1e-9)
                out.append(ExtractedPlane(
                    centroid=tuple(centroid.tolist()),  # type: ignore[arg-type]
                    normal=tuple(normal_unit.tolist()),  # type: ignore[arg-type]
                    extent_x=float(ex),
                    extent_y=float(ey),
                    inlier_count=int(len(inlier_idx)),
                ))
        # Peel and continue.
        remaining = remaining.select_by_index(inlier_idx, invert=True)

    return out


def _principal_extents(centered: np.ndarray) -> tuple[float, float]:
    """Approximate in-plane extent via PCA on centered points. We only
    care about the two largest eigen-directions; their range gives a
    bbox-style size."""
    if centered.shape[0] < 3:
        return 0.0, 0.0
    cov = np.cov(centered.T)
    try:
        evals, evecs = np.linalg.eigh(cov)
    except np.linalg.LinAlgError:
        return 0.0, 0.0
    # eigh sorts ascending; take the two largest dirs.
    order = np.argsort(evals)[::-1][:2]
    e0, e1 = evecs[:, order[0]], evecs[:, order[1]]
    proj0 = centered @ e0
    proj1 = centered @ e1
    return float(proj0.ptp()), float(proj1.ptp())
