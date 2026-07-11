"""
detection3d.py — the tracker's INPUT contract.

Not the same thing as `track.py`. Track is the frozen seam shared with the
simulator and the controller; Detection3D is what feeds the tracker on the
hardware path only. HuNavSim never produces one — it emits ground-truth Tracks
directly, bypassing the tracker entirely.

Why this exists rather than passing `fusion_lab.association.FusedObject`
straight in:

  1. FusedObject carries no timestamp. The Kalman filter needs one.
  2. FusedObject.centroid_velo is in the camera-centred pseudo-velo frame, not
     odom. (Verified: the two differ by 1.86 m on a real frame.)
  3. Importing FusedObject would couple the tracker to fusion_lab, and the
     tracker has to survive being moved into `go2-social-nav`, where fusion_lab
     does not exist.

The velo->odom transform and the timestamp attachment happen in the adapter that
builds these, OUTSIDE this package. By the time a Detection3D exists, it is
already in odom and already stamped.

Stdlib only. No numpy, no ROS, no fusion_lab.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Detection3D:
    """One per-frame observation of an object. No identity — that is the
    tracker's job. Two Detection3Ds from consecutive frames carry no indication
    of whether they are the same person."""

    x: float
    y: float
    z: float
    """Position in `frame_id`. Ground-plane x/y, z up from the floor."""

    label: str
    """Detector class name, e.g. "person"."""

    confidence: float
    """Detector confidence in [0, 1]. On the Go2 path this is the YOLO box
    score, propagated unchanged by the fusion stage."""

    timestamp: float
    """Epoch seconds. Real sensor time, not a synthetic frame index — the filter
    derives its dt from the difference between consecutive values."""

    support: int = 0
    """How many LiDAR points backed this detection's 3D position (the DBSCAN
    cluster size). A fusion-specific quality signal, which is exactly why it
    lives here and not on Track. Low support means the position is thin
    evidence; track birth can gate on it to suppress phantoms. Producers with no
    notion of point support emit 0."""

    frame_id: str = "odom"
    """Must match the frame the tracker publishes in. Carried explicitly so a
    mismatch is catchable rather than silent."""
