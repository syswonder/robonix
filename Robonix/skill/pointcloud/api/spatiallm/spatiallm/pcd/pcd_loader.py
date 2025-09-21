import logging
import numpy as np
import open3d as o3d

log = logging.getLogger(__name__)


# Load a point cloud from a file
def load_o3d_pcd(file_path: str):
    return o3d.io.read_point_cloud(file_path)


# Get points and colors from a Open3D point cloud
def get_points_and_colors(pcd: o3d.geometry.PointCloud):
    points = np.asarray(pcd.points)
    colors = np.zeros_like(points, dtype=np.uint8)
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        if colors.shape[1] == 4:
            colors = colors[:, :3]
        if colors.max() < 1.1:
            colors = (colors * 255).astype(np.uint8)
    return points, colors


# Preprocess a point cloud
def cleanup_pcd(
    pcd: o3d.geometry.PointCloud,
    voxel_size: float,
    num_nb: int = 15,
    std_ratio: float = 2.0,
):
    # voxelize the point cloud
    pcd = pcd.voxel_down_sample(voxel_size)
    # remove outliers
    pcd, _ = pcd.remove_statistical_outlier(num_nb, std_ratio=std_ratio)
    return pcd
