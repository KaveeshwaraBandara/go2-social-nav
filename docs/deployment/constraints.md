# Constraints — the Jetson Dockerfile handoff

**This is the handoff artifact.** No Jetson Dockerfile exists yet, deliberately.
This page is what must be resolved before one can be written honestly.

## Blocker 0 — RESOLVED: the target runtime is known

Harvested on-board 2026-07-13:
<!-- src: captures/harvest/runtime_20260713.txt -->

> **L4T R35.3.1 → JetPack 5.1.1 · CUDA 11.4 · aarch64 · power mode MAXN**

This is the fixed point every wheel below must match. Two riders:

- **The module is still unconfirmed.** `BOARD: t186ref` is a generic Tegra board
  string, *not* proof of an Orin NX. Thermal/TOPS budget claims remain unverified.
- **Python and ROS versions are still unconfirmed** — and Python is what
  Blocker 1 turns on.

MAXN also means there is **no performance headroom to unlock later**: the board
is already at its maximum power mode, so whatever the pipeline costs, that is
the budget.

## What the code actually needs

Verified by import-scanning the workspace:
<!-- src: ros2_ws/src — grep of import statements -->

| Package | Used by | `aarch64` risk |
|---|---|---|
| `numpy` (**pinned `<2.0`**) | everything | low — but the pin constrains all the rest <!-- src: ros2_ws/src/fusion_lab/pyproject.toml --> |
| `scipy` | fusion_lab | low |
| `open3d` | perception | **high** |
| `ultralytics` (→ **torch**) | detection | **high** |
| `tensorflow` | gesture | **high** |
| `mediapipe` | go2_gesture | **high** |
| `opencv` (`cv2`) | perception, viz | medium |
| `pandas`, `PIL` | tooling, viz | low |

Note the declared dependencies in `pyproject.toml` list **only `numpy` and
`scipy`**. The heavy ML stack — `open3d`, `ultralytics`, `tensorflow`,
`mediapipe` — is imported but **never declared anywhere**. That is its own
problem: there is no manifest to build a Dockerfile from. Producing one is step 1.

## The four hard ones

**`torch` (via `ultralytics`).** PyPI has no CUDA-enabled `aarch64` wheel — `pip
install torch` on this board yields a **CPU-only build that runs YOLO slowly and
never warns you**. Torch must come from **NVIDIA's JetPack-5 / CUDA-11.4 wheel
index**, matched to JetPack 5.1.1. This also caps the usable torch version: the
JP5 line tops out well below current PyPI torch, so `ultralytics` must be pinned
to a release compatible with that older torch — a modern `pip install
ultralytics` will try to pull a torch that does not exist for this platform. The
`numpy<2.0` pin narrows it further.

**Verify CUDA is actually live** before trusting any benchmark:
`python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"`.
If that prints `False`, everything downstream is a CPU measurement.

**`open3d`.** No official `aarch64` wheels for the relevant era; historically a
from-source build on Jetson (long, and fussy about CMake/CUDA). Check whether it
is genuinely needed at runtime or only in offline tooling — if it is only used
for visualisation, **dropping it from the runtime image is the cheapest win
available.**

**`tensorflow`.** Needs NVIDIA's Jetson-specific TF wheel. Having *both* TF and
torch CUDA runtimes in one image is heavy and a real source of CUDA/cuDNN
version conflicts. Ask whether the gesture path must ship in the same image as
perception.

**`mediapipe`.** `aarch64` wheels are patchy and often lag; may need a community
build or source compile.

## Blocker 1 — Python incompatibility (essentially confirmed)

**ROS Foxy is confirmed on the host** (`/opt/ros/foxy/`), and Foxy runs on
Ubuntu 20.04 → **Python 3.8**. `fusion_lab` requires **Python ≥ 3.10**.
<!-- src: captures/harvest/ros_env_topics_20260713.txt; ros2_ws/src/fusion_lab/pyproject.toml -->

The research code therefore **cannot run natively on the Jetson host.** The
Humble container is a **requirement**, not a convenience — this is now settled
rather than assumed. (`python3 --version` would make it airtight.)

## Blocker 2 — the DDS config on the box is self-contradictory

The host is **CycloneDDS** (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`), which the
container can match. But there are **two conflicting Cyclone configs**:
`unitree_ros2/setup.sh` binds to **`enp3s0`** (a desktop NIC name, straight out
of Unitree's PC-side instructions), while `~/cyclone_eth0.xml` binds to
**`eth0`** and adds `AllowMulticast spdp`.

If `enp3s0` doesn't exist on the Jetson, sourcing `setup.sh` points Cyclone at a
**nonexistent interface**. Any container that inherits that env inherits the bug.
Resolve which config is live *before* debugging container↔host discovery — you
will otherwise lose a day to it. See [network-dds.md](../hardware/network-dds.md).

## Blocker 3 — the calibration cannot be rebuilt

`config/go2_calib.npz` is committed, but `scripts/calibrate_go2.py` — the script
that produced it — **does not exist in the repo or its history**. The extrinsic
the fusion pipeline depends on cannot be regenerated, and its own holdout
metrics suggest a ~2.9 m median error.
See [quirks 3](../data/quirks.md#3-the-committed-extrinsic-is-drift-coupled-weakly-fit-and-unreproducible).

Shipping a Dockerfile around an unreproducible, likely-bad calibration would bake
the problem in. Recover the script, or re-derive the extrinsic from a proper
`base_link → camera_link` static transform measured on the robot.

## Blocker 4 — the SDK is not a package

`dpkg -l | grep -i unitree` returns **nothing**. The SDK exists only as a source
tree (`~/unitree_ros2/`) and a service install (`/unitree/{lib,module,services}`).
<!-- src: captures/harvest/runtime_20260713.txt -->

There is no apt package to `RUN apt install` in a Dockerfile. The SDK must be
vendored from source, and `/unitree/lib` likely has to be bind-mounted or copied
into any image that links against it.

## Blocker 5 — nothing publishes TF, so the container must

The robot publishes **no `/tf` and no `/tf_static`** — the factory stack runs no
`robot_state_publisher`. The container therefore has to own the transform tree,
including a `base_link → camera_link` edge **that has never been measured**, and
the URDF's root link is `base`, not `base_link`, so it can't be used as-is.
See [frames-tf.md](../interfaces/frames-tf.md).

This is scope the original three-layer plan did not account for.

## Suggested order

1. ~~Run the runtime harvest~~ — **done**: JetPack 5.1.1 / CUDA 11.4 / MAXN / Foxy / Cyclone.
2. ~~Settle the XT16-vs-L1 question~~ — **done**: no XT16 topic; the L1 is operative.
3. **Record a new bag** with `/utlidar/cloud_base`, `/utlidar/cloud`, `/tf`,
   `/tf_static`, `/cmd_vel` and the camera topics. `cloud_base` is robot-relative
   and likely retires the drift-coupled calibration entirely
   ([quirks 2](../data/quirks.md)). *Highest leverage single action.*
4. Resolve the **`enp3s0` vs `eth0`** Cyclone conflict (Blocker 2) and confirm
   `python3 --version` (Blocker 1).
5. Check whether `/cmd_vel` already drives the robot (sport lease?) — it may
   remove the SDK-bridge layer entirely. See [topology.md](topology.md).
6. Measure `base_link → camera_link` on the physical robot.
7. Write a real dependency manifest, torch pinned to the **JetPack-5 / CUDA-11.4**
   wheel index.
8. Decide whether `open3d` / `tensorflow` / `mediapipe` belong in the runtime image
   — the robot already exposes a gesture API and an audio hub.
9. *Then* write the Dockerfile.
