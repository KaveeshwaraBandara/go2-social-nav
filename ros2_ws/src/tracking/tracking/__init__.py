"""
tracking — detection-to-track layer for go2-social-nav.

Self-contained by construction: this package imports nothing from `fusion_lab`
and nothing from ROS. It consumes Detection3D, it emits Track. That is the whole
surface. The directory moves wholesale into `go2-social-nav` when it is ready.

    Detection3D (per frame, no identity)
        -> [association -> Kalman -> lifecycle]
        -> Track (stable id, velocity, heading)
"""
from .association import Association, greedy_associate
from .detection3d import Detection3D
from .kalman import (
    DEFAULT_INITIAL_VEL_STD,
    DEFAULT_MEASUREMENT_STD,
    DEFAULT_POSITION_BIAS_STD,
    DEFAULT_PROCESS_NOISE,
    ConstantVelocityFilter,
)
from .lifecycle import (
    DEFAULT_MAX_COAST_S,
    DEFAULT_MAX_TENTATIVE_MISSES,
    DEFAULT_MIN_BIRTH_SUPPORT,
    DEFAULT_MIN_HITS,
    Lifecycle,
)
from .track import HEADING_HOLD_SPEED, TRACK_SCHEMA_VERSION, Track
from .tracker import (
    DEFAULT_GATE_SLACK_M,
    DEFAULT_MAX_GATE_M,
    DEFAULT_MAX_SPEED_MPS,
    Tracker,
)

__all__ = [
    "Detection3D",
    "Track",
    "Tracker",
    "Association",
    "greedy_associate",
    "ConstantVelocityFilter",
    "TRACK_SCHEMA_VERSION",
    "HEADING_HOLD_SPEED",
    "DEFAULT_MAX_SPEED_MPS",
    "DEFAULT_GATE_SLACK_M",
    "DEFAULT_MAX_GATE_M",
    "Lifecycle",
    "DEFAULT_MIN_HITS",
    "DEFAULT_MAX_COAST_S",
    "DEFAULT_MAX_TENTATIVE_MISSES",
    "DEFAULT_MIN_BIRTH_SUPPORT",
    "DEFAULT_MEASUREMENT_STD",
    "DEFAULT_INITIAL_VEL_STD",
    "DEFAULT_PROCESS_NOISE",
    "DEFAULT_POSITION_BIAS_STD",
]
