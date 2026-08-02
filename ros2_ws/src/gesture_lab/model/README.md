# `gesture_lab/model/` — the trained gesture classifier

**Expected file: `gesture_model.pkl`. It is not here yet — this is the one
thing blocking Phase 11 from running end-to-end.**

## What it is

A joblib bundle written by upstream's `train_gestures.py`:

```python
{
  "model":           RandomForestClassifier,   # fitted
  "classes":         ["BACK OFF", "COME", "FOLLOW", "NONE", "RELEASE", "STAY", "STOP"],
  "window_frames":   12,      # must equal config.WINDOW_FRAMES
  "feature_dim":     101,     # must equal gesture_common.FEATURE_DIM
  "n_samples":       5087,
  "cv_macro_f1":     0.864,
}
```

`gesture_model.load_model()` refuses a bundle whose `window_frames` or
`feature_dim` disagrees with this checkout, rather than predicting garbage from
a mismatched feature layout.

## Why it is missing

Upstream (`Lasan-Perera/go2-gesture-control`) has `*.pkl` and `*.npy` in its
`.gitignore`, so neither the trained model nor the extracted dataset windows are
in that repo. The `.task` MediaPipe bundles *are* committed there, so excluding
the forest looks like an oversight rather than a decision.

## How to get it

**Preferred — ask for the artifact.** Request `gesture_model.pkl` from the
gesture repo owner, along with **the exact scikit-learn version it was trained
with**. A joblib pickle loaded under a materially different sklearn version
either warns and mispredicts or fails outright, so that version pins the
Phase-11 Dockerfile layer (`SKLEARN_SPEC`).

Worth asking them to commit it upstream via a `!gesture_model.pkl` negation or a
GitHub release asset — it is a few MB and it is the reproducible artifact behind
the reported macro F1.

**Fallback — retrain.** Needs the two source datasets (Zenodo-27 for the six
commands, IPN-Hand for the NONE class), roughly an hour of extraction, and
upstream's `extract_dataset.py` / `extract_ipn_none.py` / `train_gestures.py`,
none of which are vendored here — this package is the *inference* half only.

## Where it goes

Drop it in this directory. `ros2_ws` is bind-mounted into the container, so no
rebuild is needed:

```
ros2_ws/src/gesture_lab/model/gesture_model.pkl
```

Override with `$GO2_GESTURE_MODEL` to load it from anywhere else.

## Until then

Everything that does not depend on the model is testable now:

```
ros2 run gesture_lab run_gesture.py --selftest
```

That covers the contract and the intent adapter — the code this repo owns. It
proves nothing about recognition accuracy, which needs the model and a human in
front of a camera.
