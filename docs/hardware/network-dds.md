# Network & DDS

Verified on-board 2026-07-13.
<!-- src: captures/harvest/ros_env_topics_20260713.txt -->

## Verified

| Field | Value | Source |
|---|---|---|
| ROS distro | **Foxy** (`/opt/ros/foxy/`) | `unitree_ros2/setup.sh` |
| RMW | **`rmw_cyclonedds_cpp`** | `setup.sh` — exported explicitly |
| Cyclone build | vendored: `~/unitree_ros2/cyclonedds_ws/install/` | `setup.sh` |
| Domain ID | not set by `setup.sh` → **default 0** | `setup.sh` (absent) |

## ⚠ The two DDS configs disagree — and the active one names an interface that probably doesn't exist

There are **two** Cyclone configurations on the box, and they do not match.

**1. `~/unitree_ros2/setup.sh`** exports an *inline* `CYCLONEDDS_URI` binding to
**`enp3s0`**:

```xml
<NetworkInterface name="enp3s0" priority="default" multicast="default" />
```

**2. `~/cyclone_eth0.xml`** is a standalone file binding to **`eth0`**, and it
additionally sets `<AllowMulticast>spdp</AllowMulticast>` and `<Domain Id="any">`:

```xml
<Interfaces><NetworkInterface name="eth0" priority="default" multicast="default" /></Interfaces>
<AllowMulticast>spdp</AllowMulticast>
```

Three things follow, and they compound:

- **`enp3s0` is a predictable-name PCIe NIC — a desktop/laptop name.** It is the
  interface used in Unitree's own published setup instructions, which assume you
  are on a **development PC**, not on the robot. A Jetson's onboard NIC is
  conventionally **`eth0`**. If `enp3s0` does not exist on this board, then
  sourcing `setup.sh` binds Cyclone to a **nonexistent interface**, and DDS
  discovery fails or silently falls back.
- **`cyclone_eth0.xml` looks exactly like the fix somebody wrote for that.** It
  corrects the interface *and* adds `AllowMulticast spdp` — a classic remedy for
  broken multicast discovery. Nobody writes that file unless they hit the problem.
- **But `setup.sh` still ships the broken value**, so whether the fix is actually
  active depends entirely on whether something re-exports `CYCLONEDDS_URI` to
  point at the XML *after* `setup.sh` runs. **The inline export wins otherwise.**

TODO(unverified) — three one-liners settle it:

```bash
ip -br addr                     # does enp3s0 exist at all, or only eth0?
echo "$CYCLONEDDS_URI"          # which config is actually live in a fresh shell?
grep -rn CYCLONEDDS ~/.bashrc ~/.profile   # is the XML ever wired in?
```

**This is the first thing to check if the container can't see the robot's topics.**

## QoS: unforgiving, and they already know it

Every topic in both bags was offered with:

```
reliability: 1 (RELIABLE)
durability:  2 (VOLATILE)
history:     1 (KEEP_LAST)
depth:       1        (10 for the two camera_info topics)
```
<!-- src: captures/bags/*/metadata.yaml — offered_qos_profiles -->

**`depth: 1`** means a subscriber that stalls for one frame period drops data —
there is no buffer. Any node consuming `/utlidar/cloud_deskewed` or the camera
streams must keep its callback under the frame budget (~66 ms at 15 Hz).

**Everything is `VOLATILE`**, so a late-joining subscriber gets nothing until the
next publish — which is also why the `/tf_static` harvest came back empty.
See [frames-tf.md](../interfaces/frames-tf.md).

Corroboration that this bites in practice: `~/qos_scan_reliable.yaml` and
`~/scan_reliable_repub.py` exist on the board — an already-built QoS
republishing workaround.
<!-- src: captures/harvest/runtime_20260713.txt -->

## The container↔host bridge

The host runs **Foxy + CycloneDDS**. Humble also ships CycloneDDS, so the bridge
is viable — but the container must be configured deliberately:

- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on **both** sides.
- The **same `CYCLONEDDS_URI`**, naming an interface that actually exists.
- `ROS_DOMAIN_ID` matching (host appears to use the default, **0**).
- `network_mode: host`, which `docker-compose.yml` already sets.

TODO(unverified): nothing in the captures shows a Humble node ever talking to
this Foxy host. Foxy↔Humble share the RTPS wire protocol so it *should* work,
but **it has never been demonstrated on this hardware.** Prove it with a single
topic before building anything on top of it.
