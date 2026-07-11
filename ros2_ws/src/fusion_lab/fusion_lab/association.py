"""
association.py — Phase 4: associate 2D detections with 3D LiDAR points.

Two strategies live here:

  (A) frustum_association
      Project every LiDAR point into the image. For each 2D detection box,
      take the points whose projection lands inside the box, cluster them in
      3D to reject background bleed-through, and report the dominant cluster's
      centroid as the object's 3D position. Simple, robust, the place to start.

  (B) match_by_iou
      Given INDEPENDENT 3D boxes (from lidar.cluster_dbscan) and 2D boxes,
      project the 3D boxes to 2D and match via IoU + Hungarian assignment.
      Teaches the data-association view used by trackers.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .association_utils import hungarian_match, iou_2d
from .detection import Detection


@dataclass
class FusedObject:
    """An object carrying BOTH a semantic label and a 3D position."""

    label: str
    score: float
    centroid_velo: np.ndarray     # (3,) in LiDAR frame
    num_points: int
    detection: Detection


def frustum_association(
    points_velo: np.ndarray,
    detections: list[Detection],
    calib,
    img_width: int,
    img_height: int,
    cluster_eps: float = 0.6,
    min_cluster_pts: int = 8,
) -> list[FusedObject]:
    """Strategy A: project-into-box then cluster.

    Returns one FusedObject per detection that had enough supporting 3D points.
    """
    from .lidar import cluster_dbscan

    uv, depth, mask = calib.points_in_image_fov(points_velo, img_width, img_height)
    pts_in_fov = points_velo[mask][:, :3]

    fused: list[FusedObject] = []
    for det in detections:
        inside = det.contains(uv)
        if inside.sum() < min_cluster_pts:
            continue

        frustum_pts = pts_in_fov[inside]
        # Cluster to separate the actual object from background points that
        # happen to fall within the 2D box but lie far behind it.
        clusters = cluster_dbscan(frustum_pts, eps=cluster_eps,
                                  min_points=min_cluster_pts)
        if not clusters:
            # Fall back to nearest-depth median if clustering finds nothing.
            centroid = frustum_pts[np.argsort(frustum_pts[:, 0])[: max(1, len(frustum_pts)//2)]].mean(axis=0)
            n = frustum_pts.shape[0]
        else:
            # Pick the cluster closest to the sensor (objects occlude background).
            best = min(clusters, key=lambda c: np.linalg.norm(c.centroid))
            centroid = best.centroid
            n = best.size

        fused.append(
            FusedObject(
                label=det.label,
                score=det.score,
                centroid_velo=centroid,
                num_points=n,
                detection=det,
            )
        )
    return fused


def match_by_iou(
    boxes_2d_from_lidar: np.ndarray,   # (M, 4) projected 3D-cluster boxes
    detections: list[Detection],       # N camera detections
    iou_threshold: float = 0.3,
) -> list[tuple[int, int]]:
    """Strategy B: Hungarian assignment between projected-LiDAR and camera boxes.

    Returns list of (lidar_idx, detection_idx) matched pairs above threshold.
    """
    if len(boxes_2d_from_lidar) == 0 or len(detections) == 0:
        return []

    det_boxes = np.array([d.bbox for d in detections])
    cost = np.zeros((len(boxes_2d_from_lidar), len(det_boxes)))
    for i, lb in enumerate(boxes_2d_from_lidar):
        for j, db in enumerate(det_boxes):
            cost[i, j] = 1.0 - iou_2d(lb, db)  # minimise cost == maximise IoU

    rows, cols = hungarian_match(cost)
    pairs = []
    for r, c in zip(rows, cols):
        if (1.0 - cost[r, c]) >= iou_threshold:
            pairs.append((int(r), int(c)))
    return pairs
