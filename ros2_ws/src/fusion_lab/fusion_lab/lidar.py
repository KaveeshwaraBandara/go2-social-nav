"""
lidar.py — Phase 2: LiDAR-only object proposals.

Ground-plane removal (RANSAC) + Euclidean/DBSCAN clustering to turn a raw
point cloud into a set of candidate object clusters, each with a 3D centroid
and an axis-aligned bounding box.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Cluster:
    points: np.ndarray        # (n, 3)
    centroid: np.ndarray      # (3,)
    bbox_min: np.ndarray      # (3,)
    bbox_max: np.ndarray      # (3,)

    @property
    def size(self) -> int:
        return self.points.shape[0]


def remove_ground_ransac(
    points: np.ndarray,
    distance_threshold: float = 0.2,
    num_iterations: int = 200,
) -> tuple[np.ndarray, np.ndarray]:
    """Segment the dominant plane (ground) via RANSAC using Open3D.

    Returns (non_ground_points, ground_mask) where ground_mask indexes the
    input array.
    """
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    _, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=num_iterations,
    )
    ground_mask = np.zeros(points.shape[0], dtype=bool)
    ground_mask[inliers] = True
    return points[~ground_mask], ground_mask


def cluster_dbscan(
    points: np.ndarray,
    eps: float = 0.5,
    min_points: int = 10,
) -> list[Cluster]:
    """Cluster points with Open3D's DBSCAN. Returns a list of Cluster objects."""
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    labels = np.array(
        pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False)
    )

    clusters: list[Cluster] = []
    for label in range(labels.max() + 1):
        pts = points[labels == label][:, :3]
        if pts.shape[0] == 0:
            continue
        clusters.append(
            Cluster(
                points=pts,
                centroid=pts.mean(axis=0),
                bbox_min=pts.min(axis=0),
                bbox_max=pts.max(axis=0),
            )
        )
    return clusters


def crop_range(
    points: np.ndarray,
    x_range: tuple[float, float] = (0.0, 40.0),
    y_range: tuple[float, float] = (-20.0, 20.0),
    z_range: tuple[float, float] = (-3.0, 3.0),
) -> np.ndarray:
    """Keep only points within a region of interest (speeds up clustering)."""
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    m = (
        (x >= x_range[0]) & (x <= x_range[1])
        & (y >= y_range[0]) & (y <= y_range[1])
        & (z >= z_range[0]) & (z <= z_range[1])
    )
    return points[m]
