"""
lifecycle.py — when a track is born, when it is published, when it dies.

Three states, not two:

    TENTATIVE ──(min_hits measurements)──> CONFIRMED ──(coasts too long)──> dead
        │
        └──(misses even once)──> dead

A detection with no matching track does NOT immediately become a published
track. It becomes a *tentative* one, invisible to the consumer, and has to earn
confirmation by being seen again. This is what keeps a one-frame fusion artifact
from being announced to a controller as a person.

Why tentative and confirmed die by different rules
--------------------------------------------------
They are answering different questions.

A tentative track is asking "are you real?". Noise answers no by not showing up
again, so a single miss is enough to discard it. Waiting longer would only let
two unrelated artifacts a second apart confirm each other.

A confirmed track is asking "are you gone?". A real person disappears from the
detector constantly — 21.6% of YOLO person boxes in the validation bag never get
a 3D position, producing 1-3 s dropouts mid-walk while the person is plainly
still there. Killing a confirmed track on one miss would shred it into fragments.
So a confirmed track is allowed to coast, and it coasts in SECONDS of real
elapsed time rather than in frames.

Seconds, not frames, because everything downstream of the sensor clock is in
seconds: the same `max_coast_s` means the same physical tolerance whether the
pipeline runs at 5 Hz offline or 15 Hz on the robot. A frame count would quietly
change meaning with the frame rate.

Stdlib only.
"""
from __future__ import annotations

#: Measurements a tentative track must accumulate before it is published.
#:
#: 2 would confirm on any two consecutive fusion artifacts. 3 is the smallest
#: number that demands the object persist across a genuine span of time. It also
#: suppresses the newborn velocity transient: a track's first correction can
#: swing velocity by a metre per second, and by the third the filter has settled.
DEFAULT_MIN_HITS = 3

#: How long a CONFIRMED track may coast, unseen, before it is deleted.
#:
#: Must exceed the detector's dropout length or real people get fragmented; must
#: not exceed the time in which a person can leave and a different person arrive
#: in the same place. Tuned on the validation bag, whose dropouts run 1-3 s.
DEFAULT_MAX_COAST_S = 1.5

#: Consecutive misses a TENTATIVE track tolerates. Zero tolerance: it has not
#: earned the benefit of the doubt.
DEFAULT_MAX_TENTATIVE_MISSES = 1

#: Minimum LiDAR points backing a detection before it may START a track.
#:
#: Birth is the one moment where a bad detection costs an id, so it is the one
#: moment worth being fussy. Continuing an existing track has no such threshold:
#: a person walking away legitimately thins out, and refusing their sparse
#: detections would kill the track precisely when it is still correct.
DEFAULT_MIN_BIRTH_SUPPORT = 8


class Lifecycle:
    """The birth/confirm/death policy, separated from the filtering.

    Split out from `Tracker` because these are the knobs that get retuned per
    deployment, and because the rules are easier to reason about — and to test —
    when they are not tangled with Kalman bookkeeping.
    """

    def __init__(self, min_hits: int = DEFAULT_MIN_HITS,
                 max_coast_s: float = DEFAULT_MAX_COAST_S,
                 max_tentative_misses: int = DEFAULT_MAX_TENTATIVE_MISSES,
                 min_birth_support: int = DEFAULT_MIN_BIRTH_SUPPORT) -> None:
        self.min_hits = min_hits
        self.max_coast_s = max_coast_s
        self.max_tentative_misses = max_tentative_misses
        self.min_birth_support = min_birth_support

    def may_start_track(self, support: int) -> bool:
        """Is this detection solid enough to spend an id on?

        `support == 0` means the producer has no notion of point support (a
        simulator, say), and is trusted rather than rejected.
        """
        return support == 0 or support >= self.min_birth_support

    def is_confirmed(self, hits: int) -> bool:
        return hits >= self.min_hits

    def should_delete(self, *, confirmed: bool, missed: int,
                      time_since_update: float) -> bool:
        if not confirmed:
            return missed >= self.max_tentative_misses
        return time_since_update > self.max_coast_s
