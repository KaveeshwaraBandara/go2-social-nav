# Bags — the evidence base

Two recordings from the real Go2 EDU, made 2026-07-03. They are the **only**
ground truth this project has about on-robot behaviour.

## Where they live (and why not here)

Bags are **gitignored** (`captures/` in `.gitignore`) and are never committed —
they total ~5.8 GB. They live on the fusion-lab bench and are bind-mounted into
the container read-only:

```yaml
- ${GO2_BAGS:-$HOME/lidar-cam-fusion-lab/data/go2_bags}:/home/dev/bags:ro
```
<!-- src: docker-compose.yml -->

This repo holds the *replay capability*, not the bag storage. Override the host
path with `GO2_BAGS=/some/where`.

Format: rosbag2 `sqlite3`, metadata v4.
<!-- src: captures/bags/*/metadata.yaml -->

## `social_fusion_20260703_161444` — 5.4 GB

| | |
|---|---|
| Duration | 75.8 s |
| Messages | 17,507 |
| Topics | 9 |
| Camera | **yes** — colour + depth + camera_info + extrinsics |

**What it validates:** the entire LiDAR↔camera fusion path. This is the only bag
with RealSense data, so it is the sole input to the perception producer and the
replay visualizer. Every fusion result the project has ever produced traces to
this 75.8 s window.

**What it cannot validate:** anything TF-based (no `/tf`), anything map-relative
(no `map` frame), audio, or `/cmd_vel` response — the robot was observed, not
driven. Rates on the Go2-native topics are also suspect here (see
[quirks 4](quirks.md#4-the-fusion-bag-may-be-lossy)).

## `social_walk_20260703_133543` — 373 MB

| | |
|---|---|
| Duration | 64.2 s |
| Messages | 46,890 |
| Topics | 5 |
| Camera | **no** |

**What it validates:** locomotion and LiDAR-only behaviour at full native rates
— `/sportmodestate` at 295 Hz, `/utlidar/imu` at 250 Hz, `/utlidar/robot_odom`
at 150 Hz. Because it is 14× smaller, it is the more trustworthy reference for
*what rates the Go2 actually publishes at*. It also carries `/lf/lowstate`
(20 Hz), which the fusion bag lacks entirely.

**What it cannot validate:** anything involving the camera.

## Replaying

```bash
ros2 run fusion_lab replay_bag.py         # drives the perception producer
ros2 run fusion_lab visualize_replay.py   # renders annotated PNGs + mp4 -> outputs/
```
<!-- src: ros2_ws/src/fusion_lab/scripts/ -->

Rendered replays land in `outputs/`, which is also gitignored — they are
reproducible artifacts, not source.

## A note on the `.db3-wal` / `.db3-shm` files

Both bag directories carry SQLite write-ahead-log and shared-memory sidecars,
which means they were **copied while the database was still open**. The bags
read back cleanly, so no data appears lost, but they are not cleanly-closed
archives. Re-copy with the recorder stopped if you ever need a canonical archive.
<!-- src: captures/bags/*/ — directory listing -->
