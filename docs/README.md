# docs/ — what we actually know about the real robot

These docs are rebuilt from **real Jetson captures**, under one rule:

> Every factual claim traces to a file under `captures/`. Anything not backed by
> a capture is marked **`TODO(unverified)`** — not inferred, not filled in from
> prior knowledge.

Citations appear inline as `<!-- src: ... -->`. If you add a claim, cite it.

`captures/` is gitignored (~5.8 GB of bags). It lives on the fusion-lab bench.

## Index

### Hardware
- [runtime.md](hardware/runtime.md) — JetPack 5.1.1 / L4T R35.3.1 / CUDA 11.4 /
  MAXN **confirmed on-board**. Python, ROS distro and the actual module still TODO.
- [sensors.md](hardware/sensors.md) — D435i + L1 LiDAR: streams, rates and frames
  confirmed from bags. **An XT16 workspace exists on the board that no bag shows.**
- [network-dds.md](hardware/network-dds.md) — CycloneDDS pinned to `eth0` via a
  hand-written XML. Contents of that XML still TODO, and it matters.

### Interfaces
- [topics.md](interfaces/topics.md) — every topic, type, rate and `frame_id`,
  read out of the bags.
- [frames-tf.md](interfaces/frames-tf.md) — **the TF chain to the camera does not
  exist.** Read this one.

### Data
- [bags.md](data/bags.md) — the two recordings, what each validates, what it can't.
- [quirks.md](data/quirks.md) — the 10,000-point pad, the odom-frame cloud, the
  orphaned calibration. **Read before touching the data.**

### Deployment
- [topology.md](deployment/topology.md) — three-layer stack and the `/cmd_vel` seam.
- [constraints.md](deployment/constraints.md) — **the handoff artifact.** What must
  be resolved before a Jetson Dockerfile can be written honestly.

## The five things that matter most

1. **We recorded the wrong cloud topic.** The robot publishes `/utlidar/cloud_base`
   (robot-relative) and `/utlidar/cloud` (raw), but our bags only captured
   `/utlidar/cloud_deskewed`, which lives in the **drifting `odom` frame**.
   Switching to `cloud_base` likely retires the whole calibration mess below.
   → [quirks 2](data/quirks.md)
2. **The camera extrinsic is anchored to that drifting frame, fits badly (~2.9 m
   median holdout error), and cannot be regenerated** — `calibrate_go2.py` is not
   in the repo or its history. → [quirks 3](data/quirks.md)
3. **The robot publishes no TF at all.** No `robot_state_publisher` runs; the URDF
   is unused and has no camera link. The container must own the tree, and
   `base_link → camera_link` **has never been measured**.
   → [frames-tf.md](interfaces/frames-tf.md)
4. **Every LiDAR cloud is 89% zero-padding** — exactly 10,000 leading zero points.
   → [quirks 1](data/quirks.md)
5. **The DDS config contradicts itself** — `setup.sh` binds Cyclone to `enp3s0`
   (a desktop NIC), `cyclone_eth0.xml` binds to `eth0`. Sort this out before
   debugging why the container can't see the robot.
   → [network-dds.md](hardware/network-dds.md)

Two pieces of good news: **`/cmd_vel` already exists** on the robot (the SDK bridge
may be unnecessary), and **a uSLAM localisation stack is running** (`map → odom`
is available after all).

## Filling the gaps

`scripts/harvest_gaps.sh` is a read-only script that collects exactly the
missing evidence. Run it on the board, copy `harvest_out/` back into
`captures/`, and the `⚠ unverified` pages above can be completed.

It deliberately avoids `ros2 topic echo` for TF — that is the flaky-discovery
path that produced the empty `tf_static_dump.txt` in the first place.

## Superseded

[deployment.md](deployment.md) predates this restructure. Its *"Confirmed target
(locked)"* table is **not capture-verified**; those values are carried into
[runtime.md](hardware/runtime.md) as prior assertions to be checked, not relied on.
