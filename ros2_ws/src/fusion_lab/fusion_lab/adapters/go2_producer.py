"""
go2_producer.py — the REAL-PERCEPTION PRODUCER.

One of three interchangeable producers behind the canonical track contract:

    [oracle: /people + noise]  ─┐
    [HuNavSim (sim)]  ─────────-┤──> Track ──> IT2-FLS controller
    [THIS: bag → fusion → tracker] ┘

It closes the loop from a raw sensor frame to `list[Track]` and emits NOTHING
else. The controller may not reach in here; this may not reach into the
controller. `Track` is the only thing that crosses.

    Go2Frame (image + odom cloud)
        |  YOLO             detector.detect()          -> list[Detection]
        |  FUSE             frustum_association()      -> list[FusedObject]   (pseudo-velo)
        |  BRIDGE           fused_to_detections()      -> list[Detection3D]   (odom, stamped, deduped)
        |  TRACK            Tracker.update()           -> list[Track]         (id + velocity + covariance)
        v
    list[Track]

PURE NUMPY / PYTHON — no rclpy. That is deliberate and load-bearing: the robot
is Foxy, dev is Humble, the lab was Jazzy, and this core has to survive all
three untouched. Only the (later) node wrapper and the bag reader are allowed to
know ROS exists.

The detector is INJECTED rather than constructed, for two reasons: `ultralytics`
is a heavy optional dependency that a unit test should not need, and a different
detector (or a recorded set of boxes) can then drive the same pipeline.

KNOWN DATA FACTS — deliberately NOT compensated here
----------------------------------------------------
- The deskewed cloud aggregates returns over the preceding few hundred ms, so an
  approaching person reads ~0.2-0.4 m BEHIND true position. It is a sensor
  artifact, not a filter defect. Correcting it here would bake a bag-specific
  fudge into the producer. Left visible.
- odom-frame aggregation assumes a STATIONARY robot (every validation bag is).
  `T_odom_from_velo` is therefore computed ONCE. Under robot motion it must be
  re-evaluated per frame from odometry; `assumes_stationary_robot` says so out
  loud rather than failing quietly.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..association import frustum_association
from .go2 import Go2Rig
from .go2_tracking import DEFAULT_DEDUP_RADIUS_M, fused_to_detections, odom_from_velo

# `tracking` is a sibling top-level package: the canonical contract. The
# dependency runs ONE WAY (fusion_lab -> tracking); tracking imports nothing.
from tracking import Detection3D, Track, Tracker

#: Only people matter to a social navigation controller. Widen if the IT2-FLS
#: ever reasons about other classes (the contract already allows any label).
DEFAULT_LABELS = frozenset({"person"})


@dataclass
class Go2PerceptionProducer:
    """Turns Go2 sensor frames into canonical `Track`s. Stateful across frames.

    Stateful because tracking IS state: identities, Kalman covariance and
    lifecycle only exist across time. Feed frames in order; one producer per run.
    """

    rig: Go2Rig
    detector: object | None = None       # any .detect(image) -> list[Detection]
    tracker: Tracker = field(default_factory=Tracker)
    labels: frozenset[str] | None = DEFAULT_LABELS
    cluster_eps: float = 0.6
    min_cluster_pts: int = 8
    dedup_radius_m: float = DEFAULT_DEDUP_RADIUS_M

    #: Stated, not assumed: see the module docstring.
    assumes_stationary_robot: bool = True

    def __post_init__(self) -> None:
        # Constant for a stationary robot; recomputing per frame would be a lie
        # dressed up as diligence.
        self._T_odom_from_velo = odom_from_velo(self.rig)

    @classmethod
    def from_calib(cls, calib_npz: str | Path, **kwargs) -> "Go2PerceptionProducer":
        """Build from the rig calibration produced by `scripts/calibrate_go2.py`."""
        return cls(rig=Go2Rig.from_npz(calib_npz), **kwargs)

    # --- stage 1: frame -> stamped odom detections (no time, no identity) ----
    def detect(self, frame) -> list[Detection3D]:
        """Fuse ONE frame into odom-frame `Detection3D`s. Stateless.

        `frame` duck-types `Go2Frame`: `.image`, `.points` (pseudo-velo),
        `.t_img` (the REAL image sensor stamp — the right clock, because the YOLO
        box that defines the detection came from that exact image).
        """
        if self.detector is None:
            raise RuntimeError(
                "Go2PerceptionProducer has no detector. Inject one "
                "(e.g. fusion_lab.YoloDetector()) or call track() with detections."
            )
        boxes = self.detector.detect(frame.image)
        fused = frustum_association(
            frame.points,
            boxes,
            self.rig.calib,
            self.rig.img_width,
            self.rig.img_height,
            cluster_eps=self.cluster_eps,
            min_cluster_pts=self.min_cluster_pts,
        )
        return fused_to_detections(
            fused,
            self._T_odom_from_velo,
            timestamp=float(frame.t_img),
            labels=set(self.labels) if self.labels is not None else None,
            dedup_radius_m=self.dedup_radius_m,
        )

    # --- stage 2: detections -> tracks (identity, velocity, covariance) ------
    def track(self, detections: list[Detection3D], timestamp: float) -> list[Track]:
        """Advance the tracker by one frame. The seam is crossed HERE and only here."""
        return self.tracker.update(detections, timestamp)

    # --- the whole producer, one frame in, canonical tracks out --------------
    def step(self, frame) -> list[Track]:
        """`Go2Frame` -> `list[Track]`. The producer's entire public purpose."""
        detections = self.detect(frame)
        return self.track(detections, float(frame.t_img))
