"""
tracker.py — Detection3D in, Track out.

Association (M2) + a constant-velocity Kalman filter driven by the REAL
inter-frame dt (M3) + a birth/confirm/death policy (M4).

Each published Track carries a filtered position, a velocity, a heading, and
honest uncertainty, and has survived confirmation. Tentative tracks are tracked
internally but never published: a consumer must not be told about a person who
might be a one-frame fusion artifact.

The three concerns stay separable on purpose:
  association.py  — which detection continues which track
  kalman.py       — where a track is and how fast it is going
  lifecycle.py    — whether a track exists at all

Stdlib only. No numpy, no ROS, no fusion_lab.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

from .association import greedy_associate
from .detection3d import Detection3D
from .kalman import (
    DEFAULT_INITIAL_VEL_STD,
    DEFAULT_MEASUREMENT_STD,
    DEFAULT_POSITION_BIAS_STD,
    DEFAULT_PROCESS_NOISE,
    ConstantVelocityFilter,
)
from .lifecycle import Lifecycle
from .track import HEADING_HOLD_SPEED, Track

#: Fastest a person is assumed to move. A brisk walk is ~1.5 m/s; 2.0 leaves
#: headroom without opening the gate wide enough to jump onto a neighbour.
DEFAULT_MAX_SPEED_MPS = 2.0

#: Constant slack added to every gate, absorbing centroid noise and the known
#: ~0.2-0.4 m radial bias of the deskewed cloud. Without it, a stationary
#: person's jittering centroid could fall outside a dt-scaled gate.
DEFAULT_GATE_SLACK_M = 0.25

#: Ceiling on the gate however long a track has coasted. A track unseen for
#: seconds must not be allowed to claim a detection across the whole room.
DEFAULT_MAX_GATE_M = 1.2


@dataclass
class _TrackState:
    """The tracker's private, mutable bookkeeping for one object.

    Distinct from `Track`, which is the immutable snapshot published outward.
    Keeping them separate is what stops lifecycle counters and filter internals
    from leaking into the frozen contract.
    """

    id: int
    label: str
    kf: ConstantVelocityFilter
    confidence: float
    last_update_t: float          # last time a detection was ASSOCIATED
    last_predict_t: float         # last time the filter was propagated
    heading: float = 0.0          # held when too slow to trust velocity
    age: int = 0                  # frames since birth
    hits: int = 1                 # detections ever associated (birth counts)
    missed: int = 0               # CONSECUTIVE frames without a detection
    confirmed: bool = False       # published only once True

    # `greedy_associate` reads .x/.y/.label off whatever it is given, and the
    # filter's estimate is what should be matched against — not the last
    # measurement. A coasting track is matched where it is PREDICTED to be.
    @property
    def x(self) -> float:
        return self.kf.x

    @property
    def y(self) -> float:
        return self.kf.y


class Tracker:
    """Multi-object tracker. Feed it one frame of detections at a time."""

    def __init__(self, max_speed_mps: float = DEFAULT_MAX_SPEED_MPS,
                 gate_slack_m: float = DEFAULT_GATE_SLACK_M,
                 max_gate_m: float = DEFAULT_MAX_GATE_M,
                 lifecycle: Lifecycle | None = None,
                 measurement_std: float = DEFAULT_MEASUREMENT_STD,
                 initial_vel_std: float = DEFAULT_INITIAL_VEL_STD,
                 process_noise: float = DEFAULT_PROCESS_NOISE,
                 position_bias_std: float = DEFAULT_POSITION_BIAS_STD) -> None:
        self.max_speed_mps = max_speed_mps
        self.gate_slack_m = gate_slack_m
        self.max_gate_m = max_gate_m
        self.lifecycle = lifecycle or Lifecycle()
        self.measurement_std = measurement_std
        self.initial_vel_std = initial_vel_std
        self.process_noise = process_noise
        self.position_bias_std = position_bias_std
        self._states: list[_TrackState] = []
        self._ids = itertools.count(1)  # never reused within a run

    def gate_for(self, st: _TrackState, timestamp: float) -> float:
        """How far this particular track is allowed to have moved.

        Scaled by the time actually elapsed since the track was last MEASURED,
        not since the last frame: a coasting track's estimate has had longer to
        drift from the truth. This is the reason Detection3D insists on a real
        sensor clock — with the loader's synthetic uniform stamp every gate would
        collapse to the same constant, and a track coasting through an occlusion
        would be strangled by it.

        Now that a coasting track predicts FORWARD, this bound is conservative:
        the residual it must cover is prediction error, not the full distance
        walked. Kept as a plain geometric bound rather than a Mahalanobis gate
        because it fails safe and is trivial to reason about at 3 a.m.
        """
        elapsed = max(0.0, timestamp - st.last_update_t)
        return min(self.max_gate_m,
                   self.max_speed_mps * elapsed + self.gate_slack_m)

    def update(self, detections: list[Detection3D], timestamp: float) -> list[Track]:
        """Advance the tracker by one frame and publish the live tracks.

        Returns only CONFIRMED tracks. Tentative ones are still associated
        against and still filtered — they just are not announced.

        `timestamp` is the frame time. It is passed explicitly rather than read
        off `detections[0]` because a frame with zero detections still advances
        the clock — that is exactly when tracks coast, drift, and die.
        """
        # 1. Propagate every track to the current instant, using its own real
        #    elapsed time. Association then compares detections against
        #    PREDICTED positions, not stale ones.
        for st in self._states:
            st.kf.predict(timestamp - st.last_predict_t)
            st.last_predict_t = timestamp

        gates = [self.gate_for(st, timestamp) for st in self._states]
        assoc = greedy_associate(self._states, detections, gates)

        for ti, di in assoc.matches:
            self._correct(self._states[ti], detections[di])

        for ti in assoc.unmatched_tracks:
            st = self._states[ti]
            st.missed += 1
            st.age += 1
            # The filter already coasted in step 1; its covariance grew with it,
            # so a coasting track reports honestly widening uncertainty.

        for di in assoc.unmatched_detections:
            det = detections[di]
            if self.lifecycle.may_start_track(det.support):
                self._states.append(self._birth(det, timestamp))

        self._states = [st for st in self._states
                        if not self._should_delete(st, timestamp)]
        return [self._publish(st, timestamp) for st in self._states
                if st.confirmed]

    def _should_delete(self, st: _TrackState, timestamp: float) -> bool:
        return self.lifecycle.should_delete(
            confirmed=st.confirmed,
            missed=st.missed,
            time_since_update=max(0.0, timestamp - st.last_update_t),
        )

    # -- internals ----------------------------------------------------------
    def _birth(self, det: Detection3D, timestamp: float) -> _TrackState:
        return _TrackState(
            id=next(self._ids),
            label=det.label,
            kf=ConstantVelocityFilter(
                det.x, det.y, det.z,
                measurement_std=self.measurement_std,
                initial_vel_std=self.initial_vel_std,
                process_noise=self.process_noise,
                position_bias_std=self.position_bias_std,
            ),
            confidence=det.confidence,
            last_update_t=det.timestamp,
            last_predict_t=timestamp,
            # A birth is already one hit. With min_hits=1 the track is confirmed
            # immediately; evaluating this only in _correct would mean such a
            # track never published at all.
            confirmed=self.lifecycle.is_confirmed(1),
        )

    def _correct(self, st: _TrackState, det: Detection3D) -> None:
        st.kf.update(det.x, det.y, det.z)
        st.confidence = det.confidence
        st.last_update_t = det.timestamp
        st.age += 1
        st.hits += 1
        st.missed = 0
        if self.lifecycle.is_confirmed(st.hits):
            st.confirmed = True
        self._refresh_heading(st)

    def _refresh_heading(self, st: _TrackState) -> None:
        """Recompute heading only when the object is moving fast enough to mean
        it. Below the threshold the previous heading is HELD: a person standing
        still still faces a direction, and atan2 of noise would spin."""
        if st.kf.speed >= HEADING_HOLD_SPEED:
            st.heading = math.atan2(st.kf.vy, st.kf.vx)

    def _publish(self, st: _TrackState, timestamp: float) -> Track:
        kf = st.kf
        return Track(
            id=st.id, label=st.label,
            x=kf.x, y=kf.y, z=kf.z,
            vx=kf.vx, vy=kf.vy, heading=st.heading,
            pos_std=kf.pos_std, vel_std=kf.vel_std,
            confidence=st.confidence,
            timestamp=timestamp,
            time_since_update=max(0.0, timestamp - st.last_update_t),
            age=st.age,
        )
