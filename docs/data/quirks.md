# Quirks of the real Go2 data

Things that will silently corrupt your results if you take the messages at face
value. Each one below is **confirmed against the bags**, not folklore.

---

## 1. Every LiDAR cloud is 89% zero-padding

`/utlidar/cloud_deskewed` prepends a block of **exactly 10,000 zero points** to
every single message. Real returns are only ~1,150–1,600 points.

Measured across the first 8 clouds of the fusion bag:

| `width` | zero pts | real pts |
|---|---|---|
| 11,339 | 10,000 | 1,339 |
| 11,353 | 10,000 | 1,353 |
| 11,455 | 10,000 | 1,455 |
| 11,270 | 10,000 | 1,270 |
| 11,567 | 10,000 | 1,567 |
| 11,166 | 10,000 | 1,166 |
| 11,578 | 10,000 | 1,578 |
| 11,273 | 10,000 | 1,273 |

The count is exactly 10,000 every time — a fixed-size buffer, not noise.
<!-- src: captures/bags/social_fusion_20260703_161444/*.db3 — PointCloud2 payload -->

**The padding is a contiguous LEADING block.** Zero points occupy indices
`0 … 9999`; real data starts at index `10000` and runs to `width-1`. So:

```python
pts = cloud[10000:]        # correct — drops the pad block exactly
```

A `norm(xyz) > 0` filter also works and is safer if the 10,000 ever changes.
What does **not** work is trusting `width`: a consumer that iterates all points
processes ~89% origin points. Note `is_dense: True`, so the pad is *not* flagged
as invalid — nothing in the message advertises it.

Fields are `x, y, z, intensity`, `point_step: 32`.

---

## 2. We recorded the *wrong cloud topic* — and better ones exist

`/utlidar/cloud_deskewed` has `frame_id: odom`.
<!-- src: captures/bags/*/*.db3 — PointCloud2 header, both bags -->

The deskew step has already pushed points into the world frame, so **the cloud
inherits odometry drift**. `fusion_lab` works around this by defining a
pseudo-`velo` frame and pre-transforming `odom → velo` in the loader.
<!-- src: ros2_ws/src/fusion_lab/fusion_lab/adapters/go2.py:14-20 -->

Worse, anything *calibrated against* this cloud is calibrated against a drifting
frame — which is exactly what happened (quirk 3).

**But the robot publishes two other clouds that we never recorded:**

| Topic | Likely frame | Why it matters |
|---|---|---|
| `/utlidar/cloud` | sensor | raw, un-deskewed |
| **`/utlidar/cloud_base`** | **`base_link`** | **deskewed but robot-relative — no odom drift** |
| `/utlidar/cloud_deskewed` | `odom` | the one we used |
<!-- src: captures/harvest/ros_env_topics_20260713.txt -->

`/utlidar/cloud_base` is, on its name, precisely the topic this project should
have been consuming all along: motion-compensated like `cloud_deskewed`, but
expressed in a **robot-fixed frame**. Against that topic, the camera extrinsic
becomes a genuine **constant** `base_link → camera_link` — which is what an
extrinsic is supposed to be — instead of the drift-coupled `T_cam_from_odom`
hack, and the whole pseudo-`velo` workaround in `go2.py` becomes unnecessary.

TODO(unverified): confirm `cloud_base`'s actual `frame_id` and rate — the name is
strong evidence but no capture contains a message from it. **Record a short bag
with `/utlidar/cloud`, `/utlidar/cloud_base`, `/tf`, `/tf_static` and the camera
topics.** That single recording would likely retire quirks 2 and 3 together.

---

## 3. The committed extrinsic is drift-coupled, weakly-fit, and unreproducible

`ros2_ws/src/fusion_lab/config/go2_calib.npz` is tracked in git and is the
extrinsic the whole fusion pipeline depends on. Three separate problems.

**It is anchored to a drifting frame.** The stored matrix is
`T_cam_from_odom` — the camera's pose in `odom`. A rigidly-mounted camera has a
*constant* pose in `base_link` but a *time-varying* one in `odom`. Storing it as
a constant is only valid near the epoch it was solved at.

**Its own holdout metrics look poor.** The npz carries these arrays:

```
holdout_median_cm    = [286.1, 285.1, 286.0]
holdout_within_20cm  = [0.209, 0.170, 0.195]
```
<!-- src: ros2_ws/src/fusion_lab/config/go2_calib.npz -->

Read at face value: a **~2.86 m median holdout error**, with only **17–21% of
held-out points landing within 20 cm**. Three near-identical values suggest
three folds/scenes agreeing that the fit is bad.

**Caveat — the metric definition is unverifiable.** The script named as their
producer, `scripts/calibrate_go2.py`, **does not exist** in the working tree or
anywhere in git history; only the docstring reference to it survives.
<!-- src: ros2_ws/src/fusion_lab/fusion_lab/adapters/go2.py:12 (references the missing script) -->

So the `.npz` is a **tracked orphan artifact**: the calibration cannot be
regenerated, re-fit, or even audited, because the code that made it is gone.

TODO(unverified): recover or rewrite `calibrate_go2.py`, then confirm whether
`holdout_median_cm` means what it is named. If it does, the fusion results are
resting on a ~3 m extrinsic error and the LiDAR↔camera association is closer to
luck than geometry.

---

## 4. The fusion bag may be lossy

Go2-native topics run 4–6× slower in the fusion bag than the walk bag while the
LiDAR rate is unchanged. Most likely rosbag2 dropping under a ~71 MB/s write
load. Unconfirmed — see [topics.md](../interfaces/topics.md#rate-anomaly-between-the-two-bags--unresolved).

---

## 5. Colour and depth are different resolutions

Colour is 1280×720 `rgb8`; depth is 848×480 `16UC1`. There is no naive pixel
correspondence — use the one-shot
`/camera/camera/extrinsics/depth_to_color` message.
<!-- src: captures/bags/social_fusion_20260703_161444/*.db3 -->
