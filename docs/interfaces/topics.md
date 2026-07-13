# Topics — as recorded on the real Go2

Every row below is read out of the bags themselves (topic name, type, message
count, and `frame_id` deserialized from the first message on each topic).
Rates are `message_count / bag_duration`, so they are **average achieved rates
during that recording**, not configured publisher rates.

<!-- src: captures/bags/*/metadata.yaml (name, type, count, duration) -->
<!-- src: captures/bags/*/*.db3 (frame_id, resolution, encoding — deserialized) -->

## `social_fusion_20260703_161444` — 75.8 s, 17,507 msgs

The only bag with camera data. This is the one the fusion pipeline replays.

| Topic | Type | Hz (avg) | `frame_id` | Notes |
|---|---|---|---|---|
| `/sportmodestate` | `unitree_go/msg/SportModeState` | 53.0 | — | Go2 native |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | 47.9 | `utlidar_imu` | |
| `/utlidar/robot_odom` | `nav_msgs/msg/Odometry` | 36.1 | `odom` → `base_link` | `child_frame_id: base_link` |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 19.7 | `camera_depth_optical_frame` | 848×480 |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 19.7 | `camera_color_optical_frame` | 1280×720 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 19.7 | `camera_depth_optical_frame` | 848×480, `16UC1` |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 19.7 | `camera_color_optical_frame` | 1280×720, `rgb8` |
| `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | 15.1 | **`odom`** | see [quirks](../data/quirks.md) |
| `/camera/camera/extrinsics/depth_to_color` | `realsense2_camera_msgs/msg/Extrinsics` | one-shot | — | 1 msg, latched |

The colour stream is 1280×720 and the depth stream is 848×480 — **different
resolutions**, so depth↔colour pixel correspondence requires the RealSense
extrinsic, not a naive index map.

## `social_walk_20260703_133543` — 64.2 s, 46,890 msgs

No camera topics at all. Locomotion/LiDAR only.

| Topic | Type | Hz (avg) | `frame_id` |
|---|---|---|---|
| `/sportmodestate` | `unitree_go/msg/SportModeState` | 295.2 | — |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | 249.8 | `utlidar_imu` |
| `/utlidar/robot_odom` | `nav_msgs/msg/Odometry` | 149.6 | `odom` → `base_link` |
| `/lf/lowstate` | `unitree_go/msg/LowState` | 20.0 | — |
| `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | 15.4 | `odom` |

## Rate anomaly between the two bags — unresolved

The Go2-native topics run **~4–6× slower in the fusion bag** than in the walk
bag (`/sportmodestate` 53 vs 295 Hz, `/utlidar/imu` 48 vs 250 Hz,
`/utlidar/robot_odom` 36 vs 150 Hz) — while `/utlidar/cloud_deskewed` is
essentially unchanged (15.1 vs 15.4 Hz).

The captures do not explain this. The leading hypothesis is **recorder drop
under I/O load**: the fusion bag writes 5.4 GB in 75.8 s (≈71 MB/s, dominated
by 1280×720 RGB at ~20 Hz), which could starve the rosbag2 writer and shed the
highest-rate topics first. That is consistent with the low-rate cloud topic
being unaffected, but it is **not verified** — nothing captured records dropped
message counts.

TODO(unverified): confirm whether the fusion bag is lossy. If it is, the
`/sportmodestate` and `/utlidar/imu` streams in that bag must not be treated as
representative of on-robot rates, and any downstream timing assumption derived
from them is suspect.

## Not present in any capture

- `/tf`, `/tf_static` — **absent from both bags.** See [frames-tf.md](frames-tf.md).
- Audio / microphone — no topic in either bag.
- `/cmd_vel` — not recorded (these are observation bags; nothing was driving).
