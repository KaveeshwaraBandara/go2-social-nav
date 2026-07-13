# Sensors

Partially verified. Everything about **what the sensors emit** is confirmed from
the bags; everything about **which physical units they are** (serials, firmware,
mounting) is not captured and is marked TODO.

## RealSense D435i

| Property | Value | Source |
|---|---|---|
| Colour stream | 1280×720, `rgb8` | bag <!-- src: captures/bags/social_fusion_20260703_161444/*.db3 --> |
| Depth stream | 848×480, `16UC1` | bag |
| Achieved rate | 19.7 Hz (both streams, avg over 75.8 s) | bag |
| Colour frame | `camera_color_optical_frame` | bag |
| Depth frame | `camera_depth_optical_frame` | bag |
| Intrinsics `K` | fx 911.99, fy 911.48, cx 645.01, cy 379.08 | `go2_calib.npz` <!-- src: ros2_ws/src/fusion_lab/config/go2_calib.npz --> |
| Distortion | treated as zero | <!-- src: ros2_ws/src/fusion_lab/fusion_lab/adapters/go2.py:9 --> |
| Depth→colour extrinsic | published once, latched, on `/camera/camera/extrinsics/depth_to_color` | bag |
| ROS driver | `realsense-ros`, at `~/realsense_ws/` on the Jetson | <!-- src: captures/harvest/realsense_launch_files.txt --> |

The intrinsics are consistent with the 1280×720 colour stream (cx/cy sit near
the image centre), so `K` belongs to the colour camera.

- TODO(unverified): **serial number and firmware version.** Nothing captured
  records them. Run `rs-enumerate-devices -s` (in `scripts/harvest_gaps.sh`).
- TODO(unverified): **19.7 Hz — configured or achieved?** The driver is likely
  set to 30 Hz and under-delivering, but no capture records the requested rate.
  Check `~/realsense_ws/src/realsense-ros/.../config/config.yaml`, which the
  harvest listed but did not read.
- TODO(unverified): **mounting pose.** Not in any URDF. This is the central gap
  — see [frames-tf.md](../interfaces/frames-tf.md).

## LiDAR (Unitree L1 / `utlidar`)

| Property | Value | Source |
|---|---|---|
| Cloud topic | `/utlidar/cloud_deskewed` | bag |
| Rate | 15.1–15.4 Hz (consistent across both bags) | bag |
| Frame | **`odom`** — already deskewed into the world frame | bag |
| Fields | `x, y, z, intensity`; `point_step` 32 | bag |
| Real points/scan | ~1,150–1,600 | bag |
| Padding | exactly 10,000 leading zero points, every message | bag <!-- src: quirks.md#1 --> |
| IMU topic | `/utlidar/imu`, frame `utlidar_imu` | bag |
| IMU rate | 250 Hz (walk bag; 48 Hz in the lossy fusion bag) | bag |

The 15 Hz cloud rate is the **stable** figure — it is essentially identical in
both bags, unlike the Go2-native topics. Treat 15 Hz as the real perception
budget: it is the slowest link in the fusion chain and therefore sets the
pipeline's tick.

### The XT16 is not publishing

The Jetson has an **`xt16_ws/`** workspace (Hesai XT16), which raised the worry
that fusion had been calibrated against the wrong LiDAR.

**It hasn't.** A live `ros2 topic list` shows **no Hesai/XT16 topic of any kind** —
every cloud on the robot is `/utlidar/*`, the built-in Unitree L1.
<!-- src: captures/harvest/ros_env_topics_20260713.txt -->

So `xt16_ws/` is a dormant or abandoned workspace. The L1 is the operative
LiDAR, and the fusion work is against the right sensor.

- TODO(unverified): the XT16 could still be *physically mounted but unlaunched*.
  Worth a glance at the robot before assuming it isn't there.

### Three cloud topics exist — we recorded the least useful one

| Topic | Notes |
|---|---|
| `/utlidar/cloud` | raw, un-deskewed |
| **`/utlidar/cloud_base`** | **deskewed, robot-relative — the one we should be using** |
| `/utlidar/cloud_deskewed` | `odom` frame; the only one in our bags |
<!-- src: captures/harvest/ros_env_topics_20260713.txt -->

`/utlidar/cloud_base` very likely removes the odom-drift coupling that poisons the
current calibration. This is the highest-leverage change available to the
perception stack — see [quirks 2](../data/quirks.md).

The robot also publishes derived products the project currently ignores:
`/utlidar/height_map`, `/utlidar/grid_map`, `/utlidar/voxel_map`,
`/utlidar/range_map`, and `/utlidar/robot_pose`.

- TODO(unverified): confirm which link the URDF's `radar` refers to.

## Microphone — present, but never on a topic

**No audio topic exists in either bag.** `unitree_go/msg/AudioData` is defined in
the message set (`time_frame`, `uint8[] data`), so the interface exists.
<!-- src: captures/harvest/msg_defs.txt -->

A **USB microphone exists and has been captured from off-ROS**: the Jetson home
directory holds `test_go2_mic.py`, `mic_grab.py`, `mic_fmt.py`, `audio_test/`,
and the recordings `usbmic.wav`, `mic_capture.wav`, `grab48k.wav` (name implies
48 kHz).
<!-- src: captures/harvest/runtime_20260713.txt — ls ~/ -->

**And the robot does expose audio on ROS** — `/audioreceiver`, `/audiosender`,
`/audiohub/player/state`, plus the `/api/audiohub/{request,response}` and
`/api/vui/*` service topics.
<!-- src: captures/harvest/ros_env_topics_20260713.txt -->

So there are two audio paths — the Unitree audio hub over ROS, and direct ALSA
grabs from a USB mic — and the local `.wav` scripts suggest the ALSA route is the
one actually being used. Worth deciding which is canonical before the container
needs mic access (ROS topic vs. `--device` passthrough are very different
Dockerfile requirements).

- TODO(unverified): `arecord -l` — which card, built-in array or external USB?
- TODO(unverified): what is actually published on `/audioreceiver` (type, rate)?

## Gesture — the robot already has an API

`/api/gesture/request` and `/gesture/result` exist on the robot.
<!-- src: captures/harvest/ros_env_topics_20260713.txt -->

`go2_gesture` in this repo implements gesture recognition with MediaPipe +
TensorFlow — a heavy `aarch64` dependency pair
([constraints](../deployment/constraints.md)). Worth checking whether Unitree's
onboard gesture service does the job before shipping that stack in the image.

## Go2 body sensors

| Topic | Rate (walk bag) | Notes |
|---|---|---|
| `/sportmodestate` | 295 Hz | `unitree_go/msg/SportModeState` |
| `/lf/lowstate` | 20 Hz | `unitree_go/msg/LowState`; **absent from the fusion bag** |
| `/utlidar/robot_odom` | 150 Hz | `odom` → `base_link`, dead-reckoned |

Odometry has **no `map` frame and no correction source** anywhere in the
captures — it is unbounded dead reckoning.
