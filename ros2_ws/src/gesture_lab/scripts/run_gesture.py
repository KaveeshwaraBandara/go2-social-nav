#!/usr/bin/env python3
"""
run_gesture.py — THE OFFLINE DRIVER for the gesture-intent producer.

What replay_bag.py is to the perception producer, this is to the gesture one:
a way to run the whole chain at the desk and watch the contract come out.

    webcam -> pose lock -> 12-frame window -> RandomForest -> GestureIntent

It is a DRIVER, NOT A NODE. It publishes nothing and subscribes to nothing;
there is no rclpy anywhere in this package. The live ROS node is deliberately
deferred - exactly as the perception producer's was - because the Foxy(robot) /
Humble(dev) / Jazzy(lab) transport question is not this phase's problem, and the
producer core is framework-agnostic precisely so that node is a thin wrapper
when it comes.

    # the real thing (needs a camera, a display, and gesture_model.pkl)
    ros2 run gesture_lab run_gesture.py

    # contract + adapter only: no camera, no model, no mediapipe, no sklearn
    ros2 run gesture_lab run_gesture.py --selftest

    # the trained model + the inference path: no camera, no mediapipe
    ros2 run gesture_lab run_gesture.py --check-model
"""
import argparse
import math
import sys


def selftest() -> int:
    """Exercise the contract and the adapter with synthetic pipeline state.

    This is the phase's verify gate while the trained model is unavailable. It
    covers precisely the code that is OURS - the contract type and the
    conversion into it - and deliberately touches none of the vendored vision
    code, so it needs no camera, no display, and no heavy dependency.

    What it is NOT: evidence that gesture RECOGNITION works. That needs the
    model and a human in front of a webcam.
    """
    from gesture_intent import (
        GESTURE_INTENT_SCHEMA_VERSION,
        GESTURE_LABELS,
        STOP,
        GestureIntent,
    )
    from gesture_lab import bearing_from_image_x, make_intent
    from gesture_lab.intent_adapter import NO_OPERATOR, operator_confidence_from_distance

    failures = []

    def check(name, got, want, tol=1e-9):
        ok = abs(got - want) <= tol if isinstance(want, float) else got == want
        print(f"  {'ok  ' if ok else 'FAIL'}  {name}: {got!r}"
              + ("" if ok else f"   (expected {want!r})"))
        if not ok:
            failures.append(name)

    print(f"gesture_intent schema version {GESTURE_INTENT_SCHEMA_VERSION}")
    print(f"vocabulary: {', '.join(GESTURE_LABELS)}\n")

    # --- bearing: the sign convention, which is the easiest thing to get wrong
    print("bearing (frames are MIRRORED; +bearing = robot's left)")
    check("centre of frame -> straight ahead", bearing_from_image_x(0.5), 0.0)
    # Mirrored display: an operator on the robot's LEFT appears on the RIGHT of
    # the flipped image, so a high column must yield a POSITIVE (left) bearing.
    left = bearing_from_image_x(0.9)
    right = bearing_from_image_x(0.1)
    check("operator drawn right-of-frame -> left of robot", left > 0.0, True)
    check("operator drawn left-of-frame  -> right of robot", right < 0.0, True)
    check("symmetric about centre", left, -right, tol=1e-12)
    # Pinhole, not linear: the edge of a 69 deg FOV is half the FOV off-axis.
    # Checked UNMIRRORED so this pins the projection maths, not the flip: in a
    # true image, column 0 is the scene's left, hence a positive (left) bearing.
    left_edge = math.degrees(bearing_from_image_x(0.0, hfov_deg=69.0, mirrored=False))
    right_edge = math.degrees(bearing_from_image_x(1.0, hfov_deg=69.0, mirrored=False))
    check("unmirrored left edge of a 69 deg FOV -> +34.5 deg", left_edge, 34.5, tol=0.01)
    check("unmirrored right edge of a 69 deg FOV -> -34.5 deg", right_edge, -34.5, tol=0.01)

    # --- operator confidence: the second FOU input
    print("\noperator confidence (from hand-to-wrist distance)")
    check("hand on the wrist", operator_confidence_from_distance(0.0, 0.25), 1.0)
    check("hand at the match radius", operator_confidence_from_distance(0.25, 0.25), 0.0)
    check("halfway", operator_confidence_from_distance(0.125, 0.25), 0.5)
    check("no pose lock at all", operator_confidence_from_distance(None, 0.25), 0.0)

    # --- the contract object
    print("\nGestureIntent")
    intent = make_intent(
        STOP, 0.31, timestamp=1234.5, armed=True,
        operator_id=7, palm_x=0.5, wrist_distance=0.05,
    )
    check("frozen", isinstance(intent, GestureIntent), True)
    check("label survives", intent.label, STOP)
    check("STOP is a safety command", intent.is_safety_command, True)
    check("low-confidence STOP is still a STOP", intent.gesture_confidence, 0.31)
    check("has_operator", intent.has_operator, True)
    try:
        intent.label = "COME"  # type: ignore[misc]
        check("immutable", False, True)
    except Exception:
        check("immutable", True, True)

    unenrolled = make_intent(
        "NONE", 0.9, timestamp=1.0, armed=False, operator_id=NO_OPERATOR,
    )
    check("unenrolled -> no operator", unenrolled.has_operator, False)
    check("unenrolled -> zero operator confidence", unenrolled.operator_confidence, 0.0)
    check("unenrolled -> idle", unenrolled.is_idle, True)

    # --- the guard that stops the rules fallback leaking into the contract
    print("\noff-contract labels are rejected")
    try:
        make_intent("MOVE FORWARD", 0.9, timestamp=1.0, armed=True)
        check("rules-fallback label raises", False, True)
    except ValueError:
        check("rules-fallback label raises", True, True)

    print()
    if failures:
        print(f"SELFTEST FAILED: {len(failures)} check(s): {', '.join(failures)}")
        return 1
    print("SELFTEST PASSED — contract + adapter are sound.")
    print("NOTE: this says nothing about recognition accuracy. That needs")
    print("      gesture_model.pkl and a human in front of the camera.")
    return 0


def check_model() -> int:
    """Load the trained bundle and drive the real inference path — no camera.

    Covers everything between the feature vector and the dispatched label, which
    --selftest deliberately does not touch: the joblib load, the window/feature
    layout agreement, that the model's classes ARE the contract's vocabulary,
    and that decide() applies the STOP override and the NONE suppression.

    Needs numpy + scikit-learn + joblib, but NOT cv2, mediapipe, or a display.
    """
    import numpy as np

    from gesture_intent import GESTURE_LABELS, NONE, STOP
    from gesture_lab import config
    from gesture_lab.gesture_common import (
        COL_CX, COL_CY, COL_NFING, COL_SIZE, COL_THUMB, FEATURE_DIM, ROW_WIDTH,
        extract_features,
    )
    from gesture_lab.gesture_model import decide, load_model, predict_gesture

    failures = []

    def check(name, ok, detail=""):
        print(f"  {'ok  ' if ok else 'FAIL'}  {name}" + (f": {detail}" if detail else ""))
        if not ok:
            failures.append(name)

    print(f"model path: {config.MODEL_PATH}")
    bundle = load_model()
    if bundle is None:
        print("\nFAILED: no usable model. See gesture_lab/model/README.md.")
        return 1

    print(f"\nbundle: {bundle['n_samples']} windows -> {bundle['n_training_rows']} rows, "
          f"cv macro F1 {bundle.get('cv_macro_f1'):.3f}")
    model = bundle["model"]
    model_classes = [str(c) for c in model.classes_]

    print("\nlayout")
    check("window_frames agrees", bundle["window_frames"] == config.WINDOW_FRAMES,
          f"{bundle['window_frames']}")
    check("feature_dim agrees", bundle["feature_dim"] == FEATURE_DIM, f"{FEATURE_DIM}")
    check("model expects FEATURE_DIM inputs",
          getattr(model, "n_features_in_", FEATURE_DIM) == FEATURE_DIM)

    print("\nvocabulary")
    check("model classes == the contract's vocabulary",
          set(model_classes) == set(GESTURE_LABELS),
          f"{model_classes}")
    missing = set(GESTURE_LABELS) - set(model_classes)
    extra = set(model_classes) - set(GESTURE_LABELS)
    if missing or extra:
        print(f"        missing from model: {sorted(missing)}")
        print(f"        not in contract   : {sorted(extra)}")

    # A synthetic window in the real row format. Not a real gesture — the point
    # is that the whole path runs and returns something well-formed, not that
    # the model is accurate. Accuracy needs a human in front of a camera.
    print("\ninference path (synthetic windows — shape, not accuracy)")
    n = config.WINDOW_FRAMES

    def window(nfing_start, nfing_end, grow=1.0):
        w = np.zeros((n, ROW_WIDTH))
        for i in range(n):
            f = i / (n - 1)
            w[i, COL_CX] = 0.5
            w[i, COL_CY] = 0.5
            w[i, COL_SIZE] = 0.08 * (1.0 + (grow - 1.0) * f)
            w[i, COL_NFING] = round(nfing_start + (nfing_end - nfing_start) * f)
            w[i, COL_THUMB] = 0.0
        return w

    feats = extract_features(window(0, 4, grow=1.4))
    check("extract_features returns FEATURE_DIM",
          feats is not None and feats.shape == (FEATURE_DIM,),
          f"{None if feats is None else feats.shape}")

    label, conf = predict_gesture(list(window(0, 4, grow=1.4)), bundle)
    check("predict_gesture returns a contract label or None",
          label is None or label in GESTURE_LABELS, f"{label!r} @ {conf:.2f}")

    # decide() is the live decision rule; drive it directly at known probabilities.
    probs_stop_low = np.zeros(len(model_classes))
    probs_stop_low[model_classes.index(STOP)] = config.STOP_CONFIDENCE_THRESHOLD + 0.01
    probs_stop_low[model_classes.index("COME")] = 0.9
    lbl, _ = decide(probs_stop_low, model_classes)
    check("STOP override beats a higher-scoring label", lbl == STOP, f"-> {lbl!r}")

    probs_none = np.zeros(len(model_classes))
    probs_none[model_classes.index(NONE)] = 1.0
    lbl, _ = decide(probs_none, model_classes)
    check("NONE is suppressed into no-command", lbl is None, f"-> {lbl!r}")

    probs_weak = np.zeros(len(model_classes))
    probs_weak[model_classes.index("COME")] = config.CONFIDENCE_THRESHOLD - 0.05
    probs_weak[model_classes.index(NONE)] = 0.05
    lbl, _ = decide(probs_weak, model_classes)
    check("below-threshold non-STOP is dropped", lbl is None, f"-> {lbl!r}")

    print()
    if failures:
        print(f"MODEL CHECK FAILED: {len(failures)} check(s): {', '.join(failures)}")
        return 1
    print("MODEL CHECK PASSED — the trained model matches this checkout and the")
    print("inference path returns contract-valid labels.")
    print("NOTE: still says nothing about recognition ACCURACY. That needs a")
    print("      camera and a human performing the gestures.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument(
        "--selftest", action="store_true",
        help="verify the contract and adapter without a camera or a model",
    )
    parser.add_argument(
        "--check-model", action="store_true",
        help="load the trained model and drive the inference path; no camera",
    )
    parser.add_argument(
        "--camera", type=int, default=0,
        help="camera index (/dev/videoN); default 0",
    )
    args = parser.parse_args()

    if args.selftest:
        return selftest()
    if args.check_model:
        return check_model()

    # Imported HERE, not at module scope, so --selftest stays runnable on a
    # machine with no cv2/mediapipe/sklearn - which is the whole point of it
    # existing while the model is missing.
    from gesture_lab.gesture_control import print_dispatch, run

    try:
        run(dispatch=print_dispatch, camera_index=args.camera)
    except RuntimeError as exc:
        print(f"\n{exc}\n", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
