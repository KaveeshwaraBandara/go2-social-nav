"""
intent.py — THE FROZEN GESTURE-INTENT CONTRACT.

The second seam into the IT2-FLS controller, and the exact sibling of
`tracking/track.py`:

    [vision: pose-locked operator -> RandomForest]  ─┐
    [oracle: scripted gesture in sim / human study]  ─┤─> GestureIntent ──┐
                                                                          ├─> IT2-FLS ─> /cmd_vel
    [HuNavSim] / [bag -> fusion -> tracker]  ─────────> Track ────────────┘

Like `Track`, this module depends on NOTHING: not on gesture_lab, not on
mediapipe, not on ROS, not even on numpy. Standard library only, plain floats.
It is the shared type between `go2-social-nav` and Lasan's `go2-gesture-control`;
keep it byte-identical across the two repos.

THE ONE RULE
------------
A gesture producer emits `GestureIntent`. It NEVER publishes velocity. The
IT2-FLS is the sole publisher to `/cmd_vel`.

A gesture node that emits `geometry_msgs/Twist` is teleoperation — a fixed
lookup table from gesture to motion. The entire point of the fuzzy layer is that
COME in an open corridor and COME in a crowd produce different motion. Baking a
velocity in here would delete that difference before the controller ever saw it.
(`go2_gesture` — Phase 9 — deliberately IS such a teleop node. It is a manual
driving tool, not a producer behind this contract. See CLAUDE.md.)

WHY CONFIDENCES ARE FIELDS AND NOT `if` STATEMENTS
--------------------------------------------------
`gesture_confidence` and `operator_confidence` travel downstream as NUMBERS.
A producer must not collapse them to a yes/no before publishing.

This is the same argument that put `pos_std`/`vel_std` on `Track`: the INTERVAL
type-2 fuzzifier sizes its footprint of uncertainty from them. A producer that
applied `if confidence > 0.8: emit` would hand the controller a constant, and
an IT2 system fed constant uncertainty is a type-1 system with extra steps —
forfeiting the justification for the whole research contribution.

The two guards inside the vision producer (CONFIDENCE_THRESHOLD, CONSECUTIVE_
AGREE) are not a contradiction: they decide WHETHER A WINDOW IS A GESTURE AT
ALL, upstream of the label. Once a label exists, its confidence is passed
through untouched.

Frame convention
----------------
`operator_bearing` is in `frame_id`, a camera-fixed frame: +x forward, yaw
positive to the LEFT (REP-103). It is NOT in odom, because the gesture producer
has no odometry — resolving it into the world is the consumer's job, using the
rig extrinsic it already owns.

Units: radians, seconds (epoch). Confidences are dimensionless in [0, 1].
"""
from __future__ import annotations

from dataclasses import dataclass

#: Bump when a field is added, removed, or its meaning changes. Producers and
#: consumers in other repos check this to fail loudly on a stale copy.
GESTURE_INTENT_SCHEMA_VERSION = 1

#: The canonical vocabulary. These strings are load-bearing: they are the exact
#: class labels the trained RandomForest emits (gesture_lab/gesture_common.py,
#: MOVEMENT_CLASSES), so a "tidy-up" here — BACK_OFF for BACK OFF, lowercase,
#: an enum — silently stops matching the model and every command becomes
#: unrecognised. Change them only together with a retrain.
COME = "COME"
FOLLOW = "FOLLOW"
STOP = "STOP"
STAY = "STAY"
BACK_OFF = "BACK OFF"
RELEASE = "RELEASE"
NONE = "NONE"

#: Every label a producer may emit. NONE is included deliberately: "the operator
#: is visible and is not commanding" is a real, useful state — it is not the
#: same as no intent at all, and the controller may want to distinguish an idle
#: operator from an absent one.
GESTURE_LABELS = (COME, FOLLOW, STOP, STAY, BACK_OFF, RELEASE, NONE)

#: STOP is the one context-FREE command. A safety command whose meaning varies
#: with context is not a safety command; only the deceleration PROFILE may vary.
#: The controller is expected to branch on `is_safety_command`, never to treat
#: STOP as one more input to the rule base.
SAFETY_LABELS = frozenset({STOP})

#: Below this the operator association is too weak to act on — the hand was near
#: the edge of the match radius, or no pose lock existed at all. Exposed as a
#: NAMED CONSTANT the consumer may consult, NOT applied by the producer: the
#: fuzzy layer decides what a weak association means, the same way it decides
#: what a coasting Track means.
OPERATOR_CONFIDENCE_FLOOR = 0.25


@dataclass(frozen=True)
class GestureIntent:
    """One recognised command from one enrolled operator, at one instant.

    Immutable for the same reason `Track` is: this is a snapshot emitted by the
    producer, not a handle the consumer may edit. The producer keeps its own
    mutable state (frame window, arm state, operator lock) internally.
    """

    # --- what was said -----------------------------------------------------
    label: str
    """One of GESTURE_LABELS. Semantic INTENT, never a motion command: the
    controller decides what COME means here, now, with these people around."""

    gesture_confidence: float
    """Classifier probability for `label`, in [0, 1]. Passed downstream as a
    number — see the module docstring. Note the deliberate asymmetry upstream:
    STOP is emitted on a much lower probability than any other label, because a
    missed STOP on a walking 15 kg robot is far worse than a spurious one. So a
    STOP may legitimately arrive with a LOW confidence, and the controller must
    not read low confidence as "probably not a stop"."""

    # --- who said it -------------------------------------------------------
    operator_id: int
    """Stable for as long as the operator lock holds; -1 when nobody is
    enrolled. Same semantics as `Track.id`: a change means the lock was lost and
    re-acquired, i.e. this may be a DIFFERENT PERSON, and any latched mode
    (FOLLOW, STAY) should be treated as belonging to the previous operator."""

    operator_confidence: float
    """How sure the producer is that the classified hand belongs to the enrolled
    operator, in [0, 1]. Derived from how far the hand sat from that operator's
    wrist relative to the match radius: 1.0 on the wrist, falling to 0.0 at the
    radius. 0.0 also means "no pose lock", so a bystander cannot inherit it.

    This is the field that makes single-operator control real rather than
    aspirational, and it is the second FOU input alongside gesture_confidence.
    Consult OPERATOR_CONFIDENCE_FLOOR; do not hard-gate on it here."""

    operator_bearing: float
    """Direction to the operator in `frame_id`, radians, positive to the LEFT.
    0.0 is straight ahead.

    There is deliberately NO `operator_distance`. Metric range is the perception
    producer's job — the controller already receives the operator as a `Track`
    with real metres and real uncertainty. A vision-only gesture pipeline can
    offer nothing better than a palm-size proxy, and the field-list rule
    inherited from `Track` is that a field belongs here only if every producer
    can honestly emit it. Bearing survives that test (a sim oracle knows it
    exactly); range does not. The consumer associates intent to a Track by
    bearing."""

    armed: bool
    """Whether the producer was LISTENING when this intent was formed.

    Enrolment and arming are separate states upstream: disarming does not make
    the system forget the operator, and losing the operator does not leave it
    armed for whoever walks in next. An intent with `armed=False` is
    informational only — it must not drive motion."""

    # --- provenance --------------------------------------------------------
    timestamp: float
    """Epoch seconds of the camera frame this intent was formed from — not the
    time it was published. The gesture window spans roughly WINDOW_FRAMES frames
    BEFORE this instant, so an intent is inherently a little retrospective; the
    controller should not treat it as a measurement of "now"."""

    frame_id: str = "camera"
    """The frame `operator_bearing` is expressed in. Carried explicitly, exactly
    as on `Track`, so a sim/hardware frame mismatch surfaces as a visible field
    rather than as a silently mirrored bearing."""

    # --- derived, never stored ---------------------------------------------
    @property
    def is_safety_command(self) -> bool:
        """True for the context-free safety commands (STOP).

        The controller branches on this INSTEAD of running the fuzzy rule base:
        a stop that a crowd can talk you out of is not a stop."""
        return self.label in SAFETY_LABELS

    @property
    def has_operator(self) -> bool:
        """True when an operator was enrolled at this instant."""
        return self.operator_id >= 0

    @property
    def is_idle(self) -> bool:
        """True when the operator is present but not commanding.

        Distinct from `not has_operator`: an idle operator is still there, still
        locked, and may command in the next frame."""
        return self.label == NONE
