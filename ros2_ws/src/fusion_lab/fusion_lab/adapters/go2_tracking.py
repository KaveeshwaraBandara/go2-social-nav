"""
go2_tracking.py — bridge from Go2 fusion output into the tracking layer.

This is glue, and it is deliberately the ONLY place the two worlds meet:

    fusion_lab (FusedObject, pseudo-velo, untimed)
        |
        |  <- this module
        v
    tracking (Detection3D, odom, stamped)

The dependency runs one way, fusion -> tracking. `tracking/` never imports
`fusion_lab`, which is what lets that package move into `go2-social-nav`
wholesale. This bridge stays behind: a different detection source there gets a
different bridge, and the Detection3D contract absorbs the difference.

Three things happen here, and nowhere else:

  1. FRAME. `FusedObject.centroid_velo` is in the camera-centred pseudo-velo
     frame, NOT odom, despite the raw cloud arriving as `frame_id: odom` —
     `Go2BagLoader` transforms it out before fusion runs. We invert that.

  2. TIME. `FusedObject` carries no timestamp. We stamp each detection with the
     real sensor time of the color image the YOLO box came from.

  3. DEDUPLICATION. YOLO sometimes puts two boxes on one person; both frusta
     then cluster onto the same body and produce two 3D centroids centimetres
     apart. That is a lie about the world, and it belongs here rather than in
     the tracker: the tracker's job is to follow objects, not to decide that two
     "objects" 9 cm apart are really one. Left in, each spawns its own track id.
"""
from __future__ import annotations

import numpy as np

# `tracking` is a sibling top-level package on sys.path, not a submodule of
# fusion_lab. It brings in no ROS and no fusion_lab.
from tracking import Detection3D

#: Two same-class centroids closer than this are the same physical object.
#:
#: A person is ~0.5 m across, so two genuinely distinct people never have
#: centroids this close; two boxes on one person land 0.01-0.10 m apart in the
#: validation bag. 0.35 m sits well inside that gap.
DEFAULT_DEDUP_RADIUS_M = 0.35


def odom_from_velo(rig) -> np.ndarray:
    """The (4, 4) inverse of the rig's odom->velo transform.

    Constant for a stationary robot, which every validation bag is. Under robot
    motion this would have to be re-evaluated per frame from odometry.
    """
    return np.linalg.inv(rig.T_velo_from_odom)


def deduplicate(detections: list[Detection3D],
                radius_m: float = DEFAULT_DEDUP_RADIUS_M) -> list[Detection3D]:
    """Collapse same-class detections whose centroids coincide in the xy plane.

    Greedy 3D non-max suppression. Detections are considered strongest-first by
    LiDAR support, not by YOLO confidence: the question being settled is which
    3D position to trust, and that is answered by how many points backed it, not
    by how sure the image classifier was that it saw a person.

    Distance is planar for the same reason gating is — two people do not stack
    vertically, and folding z in would only let centroid-height noise separate
    two detections that are plainly the same person.
    """
    order = sorted(range(len(detections)),
                   key=lambda i: detections[i].support, reverse=True)
    kept: list[int] = []
    for i in order:
        d = detections[i]
        if any(detections[k].label == d.label
               and float(np.hypot(d.x - detections[k].x, d.y - detections[k].y)) < radius_m
               for k in kept):
            continue
        kept.append(i)
    return [detections[i] for i in sorted(kept)]   # restore input order


def fused_to_detections(
    fused,
    T_odom_from_velo: np.ndarray,
    timestamp: float,
    labels: set[str] | None = None,
    dedup_radius_m: float = DEFAULT_DEDUP_RADIUS_M,
) -> list[Detection3D]:
    """Convert one frame's `list[FusedObject]` into stamped odom Detection3Ds.

    `timestamp` should be the real image sensor stamp (`Go2Frame.t_img`), not
    the loader's synthetic `Go2Frame.stamp`. The image is the right clock: the
    YOLO box that defines the detection came from that exact image. The LiDAR
    points backing its 3D position are smeared over the aggregation window, so
    no single cloud stamp describes the detection anyway.

    `labels` optionally restricts to classes of interest, e.g. {"person"}.
    Pass `dedup_radius_m=0.0` to disable deduplication.
    """
    detections: list[Detection3D] = []
    for obj in fused:
        if labels is not None and obj.label not in labels:
            continue
        p_velo = np.asarray(obj.centroid_velo, dtype=float)
        x, y, z = (T_odom_from_velo @ np.array([p_velo[0], p_velo[1], p_velo[2], 1.0]))[:3]
        detections.append(
            Detection3D(
                x=float(x),
                y=float(y),
                z=float(z),
                label=obj.label,
                confidence=float(obj.score),
                timestamp=float(timestamp),
                support=int(obj.num_points),
                frame_id="odom",
            )
        )
    if dedup_radius_m > 0.0:
        detections = deduplicate(detections, dedup_radius_m)
    return detections
