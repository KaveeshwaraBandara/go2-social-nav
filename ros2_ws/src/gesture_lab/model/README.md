# `gesture_lab/model/` — the trained gesture classifier

**`gesture_model.pkl` is present on this bench but is NOT in git** (`.gitignore`:
`*.pkl`). It is 90 MB — a 300-tree, unlimited-depth RandomForest — which is two
orders of magnitude past the "small and code-adjacent" test that keeps
`fusion_lab/config/go2_calib.npz` (2 kB) versioned. It lives here and is
bind-mounted into the container with `ros2_ws`, exactly like the rosbags.

**Consequence, on purpose:** a fresh clone does not get the model. It gets the
code and a loud error from `load_model()` pointing back at this file — not a
silent wrong answer. Ask the gesture repo owner for the artifact.

## What is in the bundle

Verified by reading the pickle directly:

| key | value |
|---|---|
| `classes` | `BACK OFF, COME, FOLLOW, NONE, RELEASE, STAY, STOP` |
| `window_frames` | 12 — matches `config.WINDOW_FRAMES` |
| `feature_dim` | 101 — matches `gesture_common.FEATURE_DIM` (7×12 + 17) |
| `n_samples` | 4583 windows → 27498 rows after augmentation |
| `cv_macro_f1` | 0.8637 |
| model | `RandomForestClassifier`, 300 trees, `max_depth=None`, `class_weight="balanced"` |
| dumped by | **scikit-learn 1.9.0** |

The class list is set-identical to `gesture_intent.GESTURE_LABELS`, including
`"BACK OFF"` with a space. Those strings are load-bearing — "tidying" them
breaks label matching silently.

> Upstream's README reports 5,087 windows; this artifact records 4,583. Harmless
> (a different extraction run), but it means the README's per-class table is not
> exactly this model. The 0.864 macro F1 does match.

## The scikit-learn version gap — measured, not assumed

This is the one non-obvious thing about running the bundle here:

- it was dumped by **scikit-learn 1.9.0**, which requires **Python ≥ 3.11**
- this container is ROS 2 Humble = Ubuntu 22.04 = **Python 3.10**
- the newest scikit-learn with a cp310 wheel is **1.7.2**

So the training and runtime versions *cannot* match in this image. That is
normally the setup for silent misprediction, so it was measured: loading the
1.9.0 bundle under 1.7.2 and comparing `predict_proba` over 500 random 101-d
feature vectors gives a **max absolute difference of exactly 0.0**, with
identical argmax on all 500 rows. The forest/tree pickle internals did not
change between those versions.

`docker/Dockerfile` therefore pins `SKLEARN_SPEC="scikit-learn==1.7.2"`.
`load_model()` collapses sklearn's 300 per-tree `InconsistentVersionWarning`s
into one visible NOTE.

**Re-measure after any retrain:**

```bash
ros2 run gesture_lab run_gesture.py --check-model
```

If a future bundle does diverge, ask for one trained on 1.7.2 rather than
suppressing the difference.

## Replacing it

Drop the new `.pkl` in this directory — `ros2_ws` is bind-mounted, so no rebuild.
Override the path with `$GO2_GESTURE_MODEL` to load from anywhere else.
`load_model()` refuses any bundle whose `window_frames` or `feature_dim`
disagrees with this checkout rather than predicting garbage from a mismatched
feature layout.

Retraining from scratch needs the two source datasets (Zenodo-27 for the six
commands, IPN-Hand for NONE), roughly an hour of extraction, and upstream's
`extract_dataset.py` / `extract_ipn_none.py` / `train_gestures.py` — none of
which are vendored here. This package is the *inference* half only.
