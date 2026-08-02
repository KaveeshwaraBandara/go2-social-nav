"""
intent_adapter.py — the seam between the vision pipeline and the contract.

This is the ONLY file in `gesture_lab` that constructs a `GestureIntent`, and it
is deliberately the only one that can be tested without a webcam, without
mediapipe, and without a trained model: pure stdlib in, frozen dataclass out.

    (label, probability, arm state, palm column, wrist distance)
        -> [validate] [un-mirror] [pinhole] [associate]
        -> GestureIntent

Everything upstream of here is Lasan-Perera/go2-gesture-control (pinned at
a0c9bdc); everything downstream is the IT2-FLS. Keeping the conversion in one
pure function is what lets the vision loop stay a near-verbatim vendor copy
while the contract stays ours.

WHY THE LABEL IS VALIDATED HERE, LOUDLY
---------------------------------------
Upstream falls back to `classify_gesture_rules()` when no trained model is on
disk, so the project runs end-to-end before any data exists. That fallback emits
a DIFFERENT, OLDER vocabulary — "TURN RIGHT", "MOVE FORWARD", "MOVE BACKWARD" —
which is not in GESTURE_LABELS and which the controller has no rule for.

Silently forwarding those would be the worst possible failure: plausible-looking
intents, wrong meanings, no error. So `make_intent` raises on an off-contract
label, and `gesture_control.run()` refuses to start without a model at all.
"""
from __future__ import annotations

import math

from gesture_intent import GESTURE_LABELS, GestureIntent

from . import config

#: Returned as `operator_id` when nobody is enrolled. Matches the "-1 means no
#: operator" convention documented on the contract.
NO_OPERATOR = -1


def bearing_from_image_x(
    palm_x: float,
    hfov_deg: float | None = None,
    mirrored: bool | None = None,
) -> float:
    """Image column (0..1, left to right AS DISPLAYED) -> bearing in radians.

    Positive is to the robot's LEFT (REP-103 yaw), 0.0 is straight ahead.

    Two things are easy to get wrong here and both are silent:

    1. THE MIRROR. `gesture_control` flips every frame so the operator sees a
       natural selfie view. That is a display convenience, but it inverts the
       sign of every horizontal measurement taken from the image. An unhandled
       flip makes the robot turn toward an operator standing on its other side,
       and nothing in the pipeline complains. Handled once, here, gated on
       `config.FRAME_IS_MIRRORED`.

    2. THE PROJECTION. A linear map from column to angle is off by several
       degrees near the frame edge at a 69 deg FOV — exactly where an operator
       stands when they are about to walk out of view. This uses the pinhole
       relation instead, which costs one atan.
    """
    if hfov_deg is None:
        hfov_deg = config.CAMERA_HFOV_DEG
    if mirrored is None:
        mirrored = config.FRAME_IS_MIRRORED

    # Un-mirror into true image coordinates, where x grows to the scene's right.
    x_true = (1.0 - palm_x) if mirrored else palm_x

    # Pinhole: offset from the principal column, in units of half-width, scaled
    # by the half-angle tangent.
    half_angle = math.radians(hfov_deg) / 2.0
    offset = (0.5 - x_true) * 2.0 * math.tan(half_angle)
    return math.atan(offset)


def operator_confidence_from_distance(
    wrist_distance: float | None,
    max_distance: float | None = None,
) -> float:
    """Hand-to-wrist distance -> confidence that this hand is the operator's.

    `wrist_distance` is what the operator lock already computes: the distance in
    frame-width fractions from the palm centre to the NEAREST wrist of the
    enrolled operator's pose. `None` means there was no pose lock, which is not
    low confidence but zero — the point of the lock is that an unassociated hand
    is nobody's, so a bystander can never inherit an operator's authority.

    The map is linear from 1.0 on the wrist to 0.0 at the match radius. That is
    a deliberate, uncalibrated choice: there is no ground truth for "how likely
    is this hand attached to that arm", and a fabricated exponential would look
    principled while meaning no more than this does. The IT2-FLS absorbs the
    imprecision — that is what the footprint of uncertainty is for. Revisit only
    with data from the bystander test.
    """
    if max_distance is None:
        max_distance = config.HAND_WRIST_MAX_DIST
    if wrist_distance is None or max_distance <= 0.0:
        return 0.0
    return _clamp01(1.0 - (wrist_distance / max_distance))


def make_intent(
    label: str,
    gesture_confidence: float,
    *,
    timestamp: float,
    armed: bool,
    operator_id: int = NO_OPERATOR,
    palm_x: float | None = None,
    wrist_distance: float | None = None,
    hfov_deg: float | None = None,
    mirrored: bool | None = None,
    frame_id: str = "camera",
) -> GestureIntent:
    """Build the frozen contract object from live pipeline state.

    Raises ValueError on a label outside GESTURE_LABELS — see the module
    docstring. `palm_x` of None (hand not visible this frame) yields a bearing of
    0.0; the consumer should read `operator_confidence` to know whether the
    bearing means anything, exactly as it reads `Track.is_coasting` before
    trusting a coasting position.
    """
    if label not in GESTURE_LABELS:
        raise ValueError(
            f"{label!r} is not in the gesture contract. Valid labels: "
            f"{list(GESTURE_LABELS)}. This usually means the rule-based "
            f"fallback classifier ran because gesture_model.pkl is missing — "
            f"it emits an older, off-contract vocabulary. See CLAUDE.md, "
            f"'Phase 11'."
        )

    bearing = 0.0 if palm_x is None else bearing_from_image_x(palm_x, hfov_deg, mirrored)

    return GestureIntent(
        label=label,
        gesture_confidence=_clamp01(gesture_confidence),
        operator_id=int(operator_id),
        operator_confidence=operator_confidence_from_distance(wrist_distance),
        operator_bearing=bearing,
        armed=bool(armed),
        timestamp=float(timestamp),
        frame_id=frame_id,
    )


def _clamp01(value: float) -> float:
    """Confidences are consumed as interval bounds; out-of-range is nonsense."""
    return max(0.0, min(1.0, float(value)))
