# Deployment Architecture — Go2 EDU (Jetson Orin NX)

Target hardware deployment plan for `go2-social-nav`. This is the spec the Jetson
Dockerfile and the `/cmd_vel`→SDK bridge are built FROM. Sim code above the `/cmd_vel`
line transfers unchanged; only the locomotion layer below it swaps.

Status: PLANNING. No Jetson Dockerfile is written yet — see "Why not yet" below.
Companion docs: README.md (human-facing), CLAUDE.md (machine-facing project state).

---

## Confirmed target (locked)

| Item | Value | Notes |
|------|-------|-------|
| Robot | Unitree Go2 EDU **Plus** | research variant |
| Compute | **Jetson Orin NX, 100 TOPS** | `aarch64` / Tegra; headroom for full pipeline + vision concurrently |
| OS stack | **JetPack 5.1.1** → Ubuntu 20.04, L4T r35.x, CUDA 11.4, ROS 2 **Foxy** | factory image, kept UNTOUCHED |
| Reflash | **None** (stay on 5.1.1) | JP6 upgrade is irreversible (UEFI/QSPI rewrite, JP5 no longer boots), host-OS-constrained, and the unit is WSO2-owned — not worth the risk |
| Topology | Humble **container** (research nodes) ↔ Foxy **host** (SDK + bridge) over **DDS/RTPS** | matches sim code exactly |
| Sensors | Hesai/L1 LiDAR + RealSense D435i | mounting/config to confirm with WSO2 |

---

## The three-layer topology

```
┌─ Jetson Orin NX ────────────────────────────────────────────────┐
│                                                                  │
│  [ HUMBLE CONTAINER ]  ← our research code, identical to sim     │
│    perception (LiDAR/RealSense trackers → /people)               │
│    brain node (stub_brain now; IT2-FLS later)                    │
│    gesture recognition (Agra, MediaPipe)                         │
│         │                                                        │
│         │  /cmd_vel (geometry_msgs/Twist)   ↓                    │
│         │  /odom (nav_msgs/Odometry) + TF   ↑                    │
│         │        ── DDS / RTPS, --network host ──                │
│         ▼                                                        │
│  [ FOXY HOST (native, Ubuntu 20.04) ]                            │
│    cmd_vel → SDK bridge node                                     │
│    Unitree ROS 2 SDK (Foxy)                                      │
│         │  SDK sport-mode calls  ↓     robot state  ↑            │
│         ▼                                                        │
│  [ GO2 FIRMWARE ]  Unitree onboard locomotion — the legs        │
└──────────────────────────────────────────────────────────────────┘
```

**The `/cmd_vel` seam** (between the Humble container and the Foxy host) is the same
contract used in sim. In sim, below the seam is `planar_move` (or CHAMP); on the robot,
below the seam is the SDK bridge + Unitree firmware. The research code above the seam does
not change between sim and robot. That is the entire point of the contract discipline.

---

## Why a container on the Jetson (not native Foxy for everything)

- Our research code is written and validated against **ROS 2 Humble** in sim. Running it in
  a Humble container on the Jetson keeps it **byte-for-byte identical** to sim — no Foxy
  port of the research code, no message/API drift.
- The **Unitree SDK is Foxy/Ubuntu-20.04 native**, so the bridge that calls it runs
  **natively on the host**, alongside the SDK, not in the container.
- The two layers talk over **DDS/RTPS with `--network host`**. Standard message types
  (`Twist`, `Odometry`, `LaserScan`, TF, etc.) need **no translation** across Humble↔Foxy —
  same RMW wire protocol — which is why this split works without custom bridges. (Avoid
  custom/non-standard messages across the seam for exactly this reason.)

---

## Two Dockerfiles, one shared `src/`

| | Workstation image (exists) | Jetson image (planned) |
|---|---|---|
| Arch | `x86_64` | `aarch64` (Tegra) |
| Base | `osrf/ros:humble-*` | **L4T r35.x ROS 2 Humble** (build via `dusty-nv/jetson-containers`) |
| Purpose | sim + research + benchmarking | **deployment only** |
| Includes | Gazebo Classic, HuNavSim, Nav2 sim, CHAMP, harness, planar_move, X11/GUI | perception, brain, gesture, DDS config |
| Excludes | — | **all of sim**: no Gazebo, no HuNavSim, no Nav2 scene, no CHAMP, no harness, no GUI |
| ML deps | x86 PyPI wheels | **Jetson/aarch64 builds** matched to JP5.1.1 / CUDA 11.4 (NOT standard pip wheels) |

Both Dockerfiles mount the **same `ros2_ws/src/`**. The Jetson image is the *leaner* of the
two — it is the runtime image, not the development image.

### Known hard parts of the Jetson image (anticipate these)
1. **Base image / CUDA binding** — must match **L4T r35.x (JP 5.1.1)** exactly. Wrong L4T
   version = CUDA silently fails to bind. `jetson-containers` is the tool that gets this right.
2. **ARM64 + Jetson ML builds** — MediaPipe / TensorFlow (gesture) and later PyTorch must be
   the **Jetson-specific** aarch64 builds for JP5.1.1/CUDA 11.4. This is where time goes.
3. **GPU runtime** — Jetson uses the **NVIDIA Container Runtime** (`--runtime nvidia` / default
   runtime in `/etc/docker/daemon.json`), which differs from desktop `nvidia-container-toolkit`
   invocation.

---

## The `/cmd_vel` → SDK bridge (host-native, design pending SDK confirmation)

Runs **natively on the Foxy host**, next to the Unitree SDK. Responsibilities:
- Subscribe `/cmd_vel` (`geometry_msgs/Twist`) → translate to Unitree **high-level
  (sport-mode) velocity** command via the SDK.
- Publish `/odom` (`nav_msgs/Odometry`) + TF from the SDK's reported robot state.
- **Safety on comms loss:** if `/cmd_vel` stops arriving (timeout), command zero velocity /
  stop — never latch the last command. Mirror the sim safety floor (hard velocity cap,
  stop-if-too-close stays upstream in the brain).


---

## Sim → robot: what changes vs what transfers

| Layer | Sim | Robot | Change? |
|-------|-----|-------|---------|
| Perception input | ground-truth `/people` from HuNavSim | real LiDAR/RealSense trackers → `/people` | **new nodes**, same topic |
| Brain (stub → IT2-FLS) | publishes `/cmd_vel` | publishes `/cmd_vel` | **none** |
| `/cmd_vel` contract | — | — | **none (permanent)** |
| Below the seam | `planar_move` / CHAMP | SDK bridge + Unitree firmware | **swap the base** |
| `/odom` source | perfect sim odom | SDK/state-estimator odom (drifts, noisier) | same topic, **noisier data** |
| Tunables | sim speed/accel caps | real-robot speed/accel/latency limits | **retune params, not architecture** |

The research contribution is platform-agnostic. "Going to hardware" = write one bridge node
+ add real perception + retune a handful of limits. Not a rewrite.
