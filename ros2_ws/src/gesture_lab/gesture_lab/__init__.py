"""
gesture_lab — THE GESTURE-INTENT PRODUCER.

One of the producers behind the `gesture_intent` contract:

    [THIS: camera -> pose lock -> window -> RandomForest] ─┐
    [oracle: scripted operator in sim / human study]  ─────┴─> GestureIntent ─> IT2-FLS

It emits `GestureIntent` and NOTHING else. It never publishes velocity, never
imports the controller, and the controller never reaches in here. Same one-way
dependency discipline as fusion_lab -> tracking.

VENDORED, NOT FORKED
--------------------
config.py, gesture_common.py, gesture_model.py, hand_landmarker.py and
gesture_control.py come from Lasan-Perera/go2-gesture-control, pinned at
commit a0c9bdc. They are kept as close to upstream as possible so a re-sync is
a mechanical diff. Every deliberate divergence is marked with a
"DOWNSTREAM CHANGE" comment; there are four:

  1. absolute -> relative imports (they are a package here, not loose scripts)
  2. MODEL_PATH and the .task bundle paths resolve from the environment
     (/opt/models), because an installed package has no useful CWD
  3. run() emits GestureIntent via intent_adapter instead of (str, float)
  4. run() REFUSES to start without a trained model instead of falling back to
     the rule-based classifier, whose vocabulary is not in the contract

intent_adapter.py is ours, not upstream's.

WHY THIS __init__ IS NEARLY EMPTY
---------------------------------
`gesture_control` and `hand_landmarker` import cv2 and mediapipe at module
level. Re-exporting them here would make `import gesture_lab` — and therefore
the contract self-test, and any unit test of the adapter — depend on a working
OpenCV/MediaPipe install and, in practice, on a display. The light, pure layer
is exported eagerly; the heavy live loop is imported explicitly by the caller:

    from gesture_lab.gesture_control import run    # needs cv2 + mediapipe
"""
from .intent_adapter import (
    NO_OPERATOR,
    bearing_from_image_x,
    make_intent,
    operator_confidence_from_distance,
)

__version__ = "0.1.0"

#: Upstream commit these vendored sources track. Bump together with a re-sync.
UPSTREAM_COMMIT = "a0c9bdc"

__all__ = [
    "make_intent",
    "bearing_from_image_x",
    "operator_confidence_from_distance",
    "NO_OPERATOR",
    "UPSTREAM_COMMIT",
]
