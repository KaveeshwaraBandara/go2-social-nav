# Frames & TF — and the hole in the middle of it

**Headline: there is no captured transform chain from the robot to the camera.**
Nothing in `captures/` connects `base_link` to `camera_color_optical_frame`.
The fusion pipeline works today only because it bypasses TF entirely.

## What the captures actually contain

### `/tf` and `/tf_static` were never recorded

Neither bag contains a `/tf` or `/tf_static` topic.
<!-- src: captures/bags/*/metadata.yaml — topic list, both bags -->

The standalone TF harvest also **failed**. `captures/harvest/tf_static_dump.txt`
is not a TF tree; its entire contents are:

```
WARNING: topic [/tf_static] does not appear to be published yet
```
<!-- src: captures/harvest/tf_static_dump.txt -->

On its own that warning would prove nothing — `/tf_static` is latched
(`TRANSIENT_LOCAL`), and `ros2 topic echo` routinely reports nothing even when a
publisher exists.

**But a live `ros2 topic list` on the robot shows no `/tf` and no `/tf_static`
either.**
<!-- src: captures/harvest/ros_env_topics_20260713.txt -->

Topic *listing* does not depend on QoS matching, so this is real evidence: at
harvest time **nothing on the robot was publishing TF at all.** The factory
Unitree stack does not run a `robot_state_publisher`; the URDF sits on disk
unused. The Go2 ships odometry and clouds, and leaves the transform tree to you.

That reframes the problem. It is not "the TF tree is hard to capture" — it is
**"there is no TF tree, and we have to publish one."**

### Frames observed in message headers

These are the only frame names that appear anywhere in the recorded data:

| Frame | Seen on | Source |
|---|---|---|
| `odom` | `/utlidar/robot_odom` (`header.frame_id`), `/utlidar/cloud_deskewed` | bag |
| `base_link` | `/utlidar/robot_odom` (`child_frame_id`) | bag |
| `utlidar_imu` | `/utlidar/imu` | bag |
| `camera_color_optical_frame` | `/camera/camera/color/*` | bag |
| `camera_depth_optical_frame` | `/camera/camera/depth/*` | bag |
<!-- src: captures/bags/*/*.db3 — deserialized headers -->

### Frames declared by the URDF

`captures/harvest/go2_description_FULL.urdf` declares 29 links: `base`, `imu`,
`radar`, and the 24 leg links (`{FL,FR,RL,RR}_{hip,thigh,calf,calflower,calflower1,foot}`),
plus `Head_upper` / `Head_lower`.
<!-- src: captures/harvest/go2_description_FULL.urdf -->

Two mismatches fall straight out of that:

1. **The URDF root is `base`, but odometry's child is `base_link`.** These are
   different strings. Whatever bridges them is not in the captures.
2. **The URDF contains no camera frames whatsoever.** No `camera_link`, no
   optical frames. It also has no `utlidar_*` frame — the LiDAR appears to be
   the link named `radar`, but nothing captured confirms that identification.

TODO(unverified): is the `radar` link the L1 LiDAR? Is there a second URDF /
xacro on the robot that adds the D435i mount? `captures/harvest/urdf_files.txt`
lists a `go2_description.urdf` under `~/go2-slam-navigation/` on the Jetson, but
only the file *list* was harvested, not the sensor-bearing variants.

## The camera mount is unmeasured — and the code knows it

Because there is no TF path to the camera, `fusion_lab` does not use TF. It
loads a hand-recovered 4×4 extrinsic from `config/go2_calib.npz`:

```
T_cam_from_odom   # camera pose expressed in the ODOM frame
```
<!-- src: ros2_ws/src/fusion_lab/fusion_lab/adapters/go2.py:10-20 -->

The adapter's own docstring states this is *"recovered offline by ICP-aligning a
RealSense depth back-projection against the LiDAR cloud"*.

This has a structural consequence that matters more than the calibration
quality: **the extrinsic is anchored to `odom`, which drifts.** A camera bolted
rigidly to the robot has a *constant* pose in `base_link` and a *time-varying*
pose in `odom`. Storing it as a constant `T_cam_from_odom` is only valid near
the epoch it was solved at, and silently degrades as odometry drifts away from
that epoch. See [quirks.md](../data/quirks.md) for the recorded holdout numbers,
which are consistent with a poorly-constrained fit.

**The correct fix is a `base_link → camera_*_optical_frame` static transform**,
measured or calibrated once. That is the single highest-value thing to recover
from the physical robot, and it is a hardware-access task — it cannot be
derived from the bags.

## Target tree (what *should* exist)

```
map                     ← MISSING: nothing in captures publishes map
 └─ odom                ← published implicitly by /utlidar/robot_odom
     └─ base_link       ← odom child_frame_id  (URDF calls this `base`)
         ├─ utlidar_imu           ← IMU frame (bag) — URDF link is `imu`
         ├─ radar / utlidar       ← LiDAR (URDF `radar`; identification unconfirmed)
         └─ camera_link           ← ★ MISSING ENTIRELY ★
             ├─ camera_color_optical_frame
             └─ camera_depth_optical_frame
```

Two gaps, in priority order:

1. **`base_link → camera_link`** — blocks principled LiDAR/camera fusion. Today
   substituted by a drift-coupled ICP matrix. *Highest priority*, and it is a
   physical-measurement task: nothing on the robot can tell you this number.

2. **`map → odom`** — **a localisation source does exist, and we weren't using it.**
   The robot publishes a full SLAM stack:
   `/uslam/localization/odom`, `/uslam/localization/cloud_world`,
   `/uslam/frontend/odom`, `/uslam/cloud_map`, `/uslam/navigation/global_path`,
   and `/lio_sam_ros2/mapping/odometry`.
   <!-- src: captures/harvest/ros_env_topics_20260713.txt -->

   So the "no global frame" conclusion was wrong — it was drawn from bags that
   simply never recorded these topics. `/uslam/localization/odom` is a
   map-corrected pose and is the natural `map → odom` source. (`~/maps/` and
   `~/SLAM/` on the Jetson corroborate that this has been used.)
   <!-- src: captures/harvest/runtime_20260713.txt -->

   TODO(unverified): confirm `/uslam/localization/odom`'s frame_ids and whether
   the uSLAM service is running by default or must be started.

## Publishing the tree: what to actually do

Since the robot publishes no TF, the container must. Minimum viable tree:

| Edge | Source |
|---|---|
| `map → odom` | uSLAM localisation (above), or omit and work odom-relative |
| `odom → base_link` | bridge from `/utlidar/robot_odom` (already correct frame ids) |
| `base_link → {imu, lidar}` | `robot_state_publisher` on the URDF — **but note the URDF root is `base`, not `base_link`**, so it needs a fix or a link |
| **`base_link → camera_link`** | **★ must be measured — does not exist anywhere ★** |

Fixing the `base` / `base_link` naming mismatch is a prerequisite for using
`robot_state_publisher` at all.
