"""
gesture_intent — the canonical gesture-intent contract for go2-social-nav.

Self-contained by construction: this package imports nothing from `gesture_lab`,
nothing from mediapipe or sklearn, and nothing from ROS. A producer emits
`GestureIntent`; the IT2-FLS consumes it. That is the whole surface.

    camera -> [pose lock -> window -> classifier]  -> GestureIntent
    sim     -> [scripted operator]                 -> GestureIntent

Sibling of the `tracking` package, which does the same job for perception. The
two together are the complete input surface of the Phase-7 controller.
"""
from .intent import (
    BACK_OFF,
    COME,
    FOLLOW,
    GESTURE_INTENT_SCHEMA_VERSION,
    GESTURE_LABELS,
    NONE,
    OPERATOR_CONFIDENCE_FLOOR,
    RELEASE,
    SAFETY_LABELS,
    STAY,
    STOP,
    GestureIntent,
)

__all__ = [
    "GestureIntent",
    "GESTURE_INTENT_SCHEMA_VERSION",
    "GESTURE_LABELS",
    "SAFETY_LABELS",
    "OPERATOR_CONFIDENCE_FLOOR",
    "COME",
    "FOLLOW",
    "STOP",
    "STAY",
    "BACK_OFF",
    "RELEASE",
    "NONE",
]
