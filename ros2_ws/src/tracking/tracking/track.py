"""
track.py — THE FROZEN TRACK CONTRACT.

This file is the seam between three codebases:

    HuNavSim (simulated humans)  ─┐
                                  ├─> Track ──> IT2FL controller (go2-social-nav)
    Go2 LiDAR+camera fusion  ─────┘

All three must agree on this type exactly, so this module depends on NOTHING:
not on fusion_lab, not on ROS, not even on numpy. Standard library only, plain
floats. It is copied verbatim into `go2-social-nav`; keep it that way.

Design rule that decided the field list
---------------------------------------
A field belongs here only if EVERY producer can emit it. HuNavSim knows nothing
about DBSCAN cluster sizes or YOLO bounding boxes, so `num_points` and `bbox`
are deliberately absent — they are fusion implementation details and live on
Detection3D instead. Uncertainty (`pos_std`, `vel_std`) IS present: the sim can
emit zeros for ground truth, and the interval type-2 fuzzy controller consumes
uncertainty directly, so throwing it away here would be lossy.

Frame convention
----------------
Positions and velocities are in `odom`: a fixed world frame, x/y on the ground
plane, z up from the floor. A standing person's `z` is their centroid height
(~0.8 m), NOT a camera-relative offset. The Go2 fusion core internally works in
a camera-centred "pseudo-velo" frame; converting out of it is the adapter's job,
and it happens before a Detection3D is ever constructed.

Units: metres, metres/second, radians, seconds (epoch).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

#: Bump when a field is added, removed, or its meaning changes. Producers and
#: consumers in other repos check this to fail loudly on a stale copy.
TRACK_SCHEMA_VERSION = 1

#: Below this speed the velocity direction is dominated by sensor noise, so
#: `heading` is held at its last confident value rather than allowed to spin.
HEADING_HOLD_SPEED = 0.15  # m/s


@dataclass(frozen=True)
class Track:
    """One tracked object, with a stable identity across frames.

    Immutable by design: a Track is a snapshot published at one instant, not a
    handle the consumer can mutate. The tracker keeps its own mutable state
    internally and emits fresh Tracks each frame.
    """

    # --- identity ----------------------------------------------------------
    id: int
    """Stable across frames for as long as the object is tracked. Never reused
    within a run. An id change means the tracker lost and re-acquired the
    object; the controller must treat that as a new person."""

    label: str
    """Semantic class, e.g. "person". A YOLO class name on the hardware path.
    Not restricted to persons — the contract is extensible to other classes."""

    # --- kinematics, all in `frame_id` -------------------------------------
    x: float
    y: float
    z: float
    """Filtered position. z is carried because the detection has it, but the
    motion model is planar: nothing predicts z, it is passed through."""

    vx: float
    vy: float
    """Filtered ground-plane velocity. There is deliberately no `vz`: people do
    not move vertically, and estimating it would only fit noise.

    NOTE (known bias, do not "fix" downstream): the deskewed LiDAR cloud
    aggregates returns over the preceding few hundred ms, so an approaching
    person reads ~0.2-0.4 m behind true position. Velocities therefore carry a
    small systematic lag. It is a sensor artifact, not a filter defect."""

    heading: float
    """Direction of travel in radians, atan2(vy, vx), in [-pi, pi].

    Stored rather than computed on demand so it can be HELD across moments when
    the object is too slow for velocity direction to mean anything (see
    HEADING_HOLD_SPEED). A person standing still still faces a direction, and a
    social controller wants a stable heading, not atan2 of noise. Check `speed`
    or `is_confident_heading` before trusting it for a near-stationary track."""

    # --- uncertainty (the controller's interval bounds come from here) ------
    pos_std: float
    """1-sigma position uncertainty in metres, isotropic in the xy plane."""

    vel_std: float
    """1-sigma velocity uncertainty in m/s, isotropic in the xy plane."""

    # --- provenance --------------------------------------------------------
    confidence: float
    """Detector confidence of the most recently ASSOCIATED detection, in [0, 1].
    It does not decay while the track coasts — use `time_since_update` for
    staleness, not this. Ground-truth producers emit 1.0."""

    timestamp: float
    """Epoch seconds of the frame this snapshot describes. For a coasting track
    this is the current frame time, not the time of the last detection."""

    time_since_update: float
    """Seconds since a detection was last associated with this track. 0.0 means
    the position is measured; > 0 means it is PREDICTED (the object is occluded
    or missed). Safety-relevant: a controller should widen its margins around a
    coasting track rather than trusting a dead-reckoned position."""

    age: int
    """Number of frames since this track was born. Young tracks are less
    trustworthy; the controller may discount them."""

    frame_id: str = "odom"
    """The frame `x, y, z, vx, vy, heading` are expressed in. Defaulted because
    every current producer emits odom, but carried explicitly so a frame
    mismatch between sim and hardware surfaces as a visible field rather than as
    silently wrong headings."""

    # --- derived, never stored ---------------------------------------------
    @property
    def speed(self) -> float:
        """Ground-plane speed in m/s."""
        return math.hypot(self.vx, self.vy)

    @property
    def is_coasting(self) -> bool:
        """True when this snapshot's position is predicted, not measured."""
        return self.time_since_update > 0.0

    @property
    def is_confident_heading(self) -> bool:
        """True when the object moves fast enough for `heading` to be meaningful
        right now, rather than a value held over from earlier motion."""
        return self.speed >= HEADING_HOLD_SPEED
