# Runtime (Jetson)

Partially verified from an on-board harvest, 2026-07-13.
<!-- src: captures/harvest/runtime_20260713.txt -->

## Verified

| Field | Value | Source |
|---|---|---|
| L4T revision | **R35.3.1** (GCID 32827747, 2023-03-19) | `/etc/nv_tegra_release` |
| Architecture | **aarch64** (EABI) | `/etc/nv_tegra_release` |
| CUDA SDK | **11.4.19** (cudart 11.4.298, nvcc 11.4.315) | `/usr/local/cuda/version.json` |
| cuBLAS / cuFFT / cuSOLVER | 11.6.6.84 / 10.6.0.202 / 11.2.0.297 | same |
| Nsight Compute | 2021.2.8.1 | same |
| Power mode | **MAXN** (nvpmodel 0) — no throttling headroom to reclaim | `nvpmodel -q` |
| Board string | `t186ref` | `/etc/nv_tegra_release` |
| **ROS distro** | **Foxy** (`/opt/ros/foxy/setup.bash`) | `unitree_ros2/setup.sh` <!-- src: captures/harvest/ros_env_topics_20260713.txt --> |
| RMW | `rmw_cyclonedds_cpp` | same |

**JetPack = 5.1.1**, derived from L4T R35.3.1 via NVIDIA's standard L4T↔JetPack
mapping. Note this is a *derivation*, not a string read off the board: the
`nvidia-jetpack` meta-package is **not installed** (`dpkg -l | grep
nvidia-jetpack` returns nothing), so no on-board package version states it
directly. The CUDA 11.4 toolkit is consistent with JetPack 5.1.x.

This **confirms** the JetPack 5.1.1 / L4T r35.x / CUDA 11.4 line previously
asserted in [docs/deployment.md](../deployment.md). Those three are no longer
guesses.

## Still unverified — and one of them is a trap

**`BOARD: t186ref` does NOT confirm an Orin NX.** That string is a generic L4T
board-reference tag shared across Tegra platforms; it is not a module
identifier. The "Orin NX, 100 TOPS" claim in `deployment.md` therefore remains
**unverified**, despite the rest of the row now checking out. Don't let the
confirmed JetPack version lend it false credibility.

| Field | Command | Status |
|---|---|---|
| **Module (Orin NX?)** | `cat /proc/device-tree/model; cat /sys/firmware/devicetree/base/model` | TODO(unverified) |
| OS / Ubuntu release | `cat /etc/os-release` | TODO(unverified) — but ROS Foxy is only supported on **20.04 focal**, so this is now near-certain |
| Kernel | `uname -a` | TODO(unverified) |
| **Host Python** | `python3 --version` | TODO(unverified) — **load-bearing, see below** |
| RAM / cores | `free -h; nproc` | TODO(unverified) |
| cuDNN / TensorRT | `dpkg -l \| grep -Ei 'cudnn\|tensorrt'` | TODO(unverified) |

## What the home directory reveals

`ls ~/` is not a version string, but it is strong evidence about how this board
is actually used. Several entries change the picture elsewhere in these docs:

- **`cyclonedds/`, `cyclonedds_ws/`, `cyclone_eth0.xml`** — CycloneDDS is the RMW
  in use, with a hand-written config bound to `eth0`. → [network-dds.md](network-dds.md)
- **`xt16_ws/`** — a workspace for a **Hesai XT16** LiDAR, which is *not* the
  `/utlidar` L1 that appears in the bags. → [sensors.md](sensors.md)
- **`test_go2_mic.py`, `mic_grab.py`, `usbmic.wav`, `grab48k.wav`, `audio_test/`** —
  a USB microphone exists and has been captured from, even though no audio topic
  was ever recorded in a bag.
- **`qos_scan_reliable.yaml`, `scan_reliable_repub.py`** — someone has already
  fought the robot's QoS. Consistent with the `depth: 1` / `VOLATILE` profiles
  seen in the bags.
- **`unitree_ros2/`, `/unitree/{lib,module,services}`** — the SDK is present as a
  source tree and a service install, **not** as a dpkg package (`dpkg -l | grep
  -i unitree` is empty). Any Dockerfile must vendor it from source.
- **`yolov8n.pt`** — the same weights this repo bakes to `/opt/models/`.
- **`venvs/`** — Python work is already being done in virtualenvs, which hints the
  system Python was not sufficient. Worth understanding before choosing a
  container base.

## Why the Python version is the load-bearing question

**ROS Foxy is confirmed**, and Foxy targets **Ubuntu 20.04**, which ships
**Python 3.8**. `fusion_lab` declares `requires-python = ">=3.10"`.
<!-- src: ros2_ws/src/fusion_lab/pyproject.toml -->

So the research code **almost certainly cannot run natively on the Jetson host** —
a hard incompatibility, not a preference. This is now the confirmed justification
for the Humble-container topology, not a rationalisation of it.

Corroborating detail: the `ros2` CLI on the box emits a `pkg_resources is
deprecated` warning, which is characteristic of the older Python/setuptools on
focal. And `~/venvs/` shows this friction has already been felt.
<!-- src: captures/harvest/ros_env_topics_20260713.txt -->

One command closes it for good: `python3 --version`.

See [constraints.md](../deployment/constraints.md).
