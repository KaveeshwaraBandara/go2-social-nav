# Deployment topology

The three-layer stack and the contract seam that makes sim code transfer to the
robot unchanged.

> Much of this page describes an **intended** design, not a verified one. The
> layers below the `/cmd_vel` seam have never been exercised on the real board in
> anything the captures record. Claims sourced to `docs/deployment.md` are prior
> assertions, not capture evidence.

## The three layers

```
┌─ Jetson Orin NX ─────────────────────────────────────────────────┐
│                                                                   │
│  [ HUMBLE CONTAINER ]  ← our research code, identical to sim      │
│    fusion_lab      LiDAR+camera → tracking/Track                  │
│    go2_brain       social nav policy                              │
│    go2_gesture     MediaPipe gesture recognition                  │
│         │                                                         │
│         │  ↓ /cmd_vel  (geometry_msgs/Twist)                      │
│         │  ↑ /odom (nav_msgs/Odometry) + TF                       │
│         │        ══ THE SEAM ══  DDS/RTPS, network_mode: host     │
│         ▼                                                         │
│  [ FOXY HOST (native, Ubuntu 20.04) ]                             │
│    cmd_vel → SDK bridge node                                      │
│    Unitree ROS 2 SDK (Foxy)                                       │
│         │  ↓ SDK sport-mode calls    ↑ robot state                │
│         ▼                                                         │
│  [ GO2 FIRMWARE ]  Unitree onboard locomotion — the legs          │
└───────────────────────────────────────────────────────────────────┘
```
<!-- src: docs/deployment.md — PRIOR ASSERTION -->

## The seam

The contract is deliberately narrow:

- **In (down):** `/cmd_vel` — `geometry_msgs/Twist`.
- **Out (up):** `/odom` — `nav_msgs/Odometry` — plus TF.

Everything above the seam is written and validated in sim and does not change on
the robot. Below the seam, sim uses `planar_move`; the robot uses the SDK
bridge. That substitution is the entire point of the discipline.

## What the captures confirm about the seam

**The "out" half is real and matches.** The robot genuinely publishes
`nav_msgs/Odometry` with `odom` → `base_link` — on `/utlidar/robot_odom`, not
`/odom`, so the bridge must remap.
<!-- src: captures/bags/*/*.db3 -->

**The TF half of "out" does not exist — the robot publishes no TF at all**, so
the container has to publish the tree itself. This is more work than the diagram
implies. See [frames-tf.md](../interfaces/frames-tf.md).

**`/cmd_vel` already exists on the robot.** It appears in a live `ros2 topic
list`.
<!-- src: captures/harvest/ros_env_topics_20260713.txt -->

This is the best news in the whole harvest: the seam's "in" half may need **no
custom SDK bridge at all**. If the factory stack already subscribes `/cmd_vel`
and drives sport-mode, then the container publishes `/cmd_vel` over DDS and the
robot simply moves — exactly as in sim.

TODO(unverified): confirm `/cmd_vel` has a *subscriber* (`ros2 topic info
/cmd_vel`), not just a publisher, and that a `Twist` on it actually moves the
robot. Note `/api/sport/request` and `/api/sport_lease/request` also exist —
motion may require **acquiring the sport lease** first, and `/api/motion_switcher`
may need the right mode selected. That is the likely catch.

## Layer ownership

| Layer | Runs on | Ours? | Verified on hardware? |
|---|---|---|---|
| Research nodes | Humble container | yes | perception only, via bag replay |
| SDK bridge | Foxy host, native | **possibly unnecessary** — `/cmd_vel` exists | no |
| TF publisher | container | **not written — nothing publishes TF** | no |
| Locomotion | Go2 firmware | no (Unitree) | n/a |

The bridge layer shrinks and a **TF layer appears** — the opposite of what the
original plan assumed.

## Why a container rather than native

The research code targets `requires-python >= 3.10`.
<!-- src: ros2_ws/src/fusion_lab/pyproject.toml -->

If the host really is Ubuntu 20.04 / Foxy (Python 3.8), the code **cannot run
natively at all** — this is a hard incompatibility, not a preference, and it is
the real justification for the container. Confirm the host Python version before
building anything: see [runtime.md](../hardware/runtime.md) and
[constraints.md](constraints.md).
