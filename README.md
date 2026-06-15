# Go2 Social Navigation — Containerized Simulation Environment

Containerized **ROS 2 Humble + Gazebo Classic 11** simulation environment for
Unitree Go2 social navigation research.

Everything runs inside Docker. The host provides only Docker + GUI forwarding;
the host's own ROS installation is never used.

## Why containerized

- **Host:** Ubuntu 24.04 (ROS 2 Jazzy native — unused here).
- **Target robot:** Unitree Go2 EDU, Jetson Orin NX, JetPack 5.1.1 (Ubuntu
  20.04, ROS 2 Foxy native). We develop in **Humble** so the codebase ports to
  Foxy via a thin bridge node on the robot later.
- Gazebo Classic 11 is EOL on Ubuntu 24.04 — containerizing gives us the
  Humble + Gazebo Classic 11 combo cleanly on a modern host.

## Hardware / GPU notes

- **This laptop:** no NVIDIA GPU — only Intel Raptor Lake-P UHD Graphics. The
  default compose config uses **Intel iGPU** acceleration via `/dev/dri`.
- **Desktop PC / Orin (later):** have NVIDIA GPUs. Use the `nvidia` profile
  (`PROFILE=nvidia ./run.sh ...`), which requires `nvidia-container-toolkit`
  on that host.
- **Display:** the host runs a Wayland session; Gazebo (an X11 app) renders
  through Xwayland. `run.sh` handles X11 authorization automatically.

## Layout

```
docker/
  Dockerfile        # osrf/ros:humble-desktop + Gazebo Classic 11 + tools
  entrypoint.sh     # sources ROS + workspace overlay
ros2_ws/src/        # our ROS 2 packages (live bind-mount into the container)
docker-compose.yml  # service def: host net, X11, /dev/dri, bind-mount
run.sh              # wrapper: X11 grant/revoke + compose build/up/shell/down
```

## Quick start

```bash
# 1. Build the image
./run.sh build

# 2. Start the container (grants X11 access, drops you into a shell)
./run.sh up

# 3. Inside the container — confirm Gazebo Classic 11 opens on your display
gazebo

# Extra shells into the running container
./run.sh shell

# Stop everything (also revokes X11 access)
./run.sh down
```

### Troubleshooting GUI

- **No Gazebo window appears:** run `xhost +local:` on the host, retry. If still
  blank, the iGPU path may be the issue — restart with software rendering:
  `LIBGL_ALWAYS_SOFTWARE=1 ./run.sh up`.
- **First `gazebo` launch is slow / times out:** it downloads model meshes on
  first run. Use `gazebo --verbose` to watch progress.

### NVIDIA host (later)

```bash
PROFILE=nvidia ./run.sh build
PROFILE=nvidia ./run.sh up
```

## The Go2 model (Phase 1)

Package: `ros2_ws/src/go2_description`.

- **Source:** the official [`unitreerobotics/unitree_ros`](https://github.com/unitreerobotics/unitree_ros)
  `go2_description` (meshes + kinematics + inertials), **repackaged for ROS 2 /
  ament**. The upstream package is ROS 1 / catkin with ROS 1 Gazebo plugins; we
  reuse only the description and write our own Gazebo Classic config.
- **Phase-1 simplification (deliberate):** the Go2's 12 revolute leg joints are
  **locked into a fixed standing stance** (hip 0, thigh 0.8, calf −1.5 rad),
  collapsing the robot into a single rigid body. A
  `libgazebo_ros_planar_move.so` plugin drives that body holonomically over the
  ground plane. **This is a kinematic driving base, not gait control** — no leg
  motion, no balance. The leg-locking transform lives in
  `go2_description/scripts/lock_legs.py` (run against `urdf/go2_original.urdf`
  to regenerate `urdf/go2.urdf`).

### Control contract (permanent)

| Topic     | Type                      | Direction          |
|-----------|---------------------------|--------------------|
| `/cmd_vel`| `geometry_msgs/msg/Twist` | input  (drive base)|
| `/odom`   | `nav_msgs/msg/Odometry`   | output             |
| TF        | `odom → base → links`     | output             |

`/cmd_vel` is the **permanent interface** that every later "brain" node
(teleop now, the IT2-FLS controller later) publishes to. `planar_move` is
holonomic: `linear.x`, `linear.y`, and `angular.z` all take effect.

### Launch

```bash
# Inside the container:
cd ~/ros2_ws
colcon build --packages-select go2_description   # one-time (build is bind-mounted)
source install/setup.bash
ros2 launch go2_description spawn_go2.launch.py            # GUI
ros2 launch go2_description spawn_go2.launch.py gui:=false # headless
```

### Teleop (Phase 2)

Drive the base from the keyboard. Run in its **own terminal that has keyboard
focus** (it reads raw stdin):

```bash
./run.sh shell
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

It publishes `geometry_msgs/msg/Twist` on `/cmd_vel` (no remap needed). Keys:
`i`/`,` forward/back, `j`/`l` turn, `u`/`o`/`m`/`.` diagonals, `k` stop;
`q`/`z` change speed. `planar_move` is holonomic, so the strafe keys work too.

## HuNavSim (Phase 3)

HuNavSim is **integrated directly into this container** (not run as a separate
prebuilt image). We reuse the official robotics-upo stack but install only what
the cafe demo needs — **no PMB2 robot / nav2 stack**:

- **Image layer** (`docker/Dockerfile`, pinned commits): `lightsfm` (make-installed),
  `people_msgs` (source), and `hunav_sim` + `hunav_gazebo_wrapper` (v2.0) built
  into the overlay `/opt/hunav_ws`. `behaviortree_cpp` comes from apt (4.9, so —
  unlike upstream — no BehaviorTree source build). The cafe Gazebo models
  (`cafe`, `cafe_table`, `ground_plane`, `sun`) are vendored into
  `/opt/gazebo_models` so first run is offline and reproducible.
- **Our glue** (`ros2_ws/src/go2_hunav`): `cafe_isolated.launch.py`, a minimal
  launch reusing the upstream nodes/worlds/scenarios without PMB2/nav2.

**Simplification (documented):** the `HuNavPlugin` requires a Gazebo model
named after `robot_name` to exist (it tracks the robot as a `ROBOT` agent the
pedestrians react to). For learning HuNavSim in isolation we spawn a **static
placeholder robot** (a box, `go2_hunav/models/placeholder_robot.sdf`) — not the
heavy PMB2, and not the Go2 yet. **Phase 4 replaces the box with the Go2.**

### Run the cafe scenario

```bash
./run.sh up
source ~/ros2_ws/install/setup.bash
ros2 launch go2_hunav cafe_isolated.launch.py            # opens RViz; pedestrians walk
# In another shell (./run.sh shell):
ros2 topic echo /people                                  # live agent positions (people_msgs/People)
```

**Visualization:** Gazebo's `gzclient` crashes on the heavy cafe scene under
Xwayland + Intel iGPU (a Gazebo-Classic rendering bug), so the launch defaults
to **RViz** (`rviz:=true`) with a `/people` → `MarkerArray` bridge
(`people_markers.py`): each pedestrian is a cylinder + heading arrow. The
Gazebo server still runs headless. Pass `gui:=true` to also try the (fragile)
Gazebo GUI.

`/people` (`people_msgs/msg/People`, `frame_id: map`) publishes each agent's
position (yaw packed in `position.z`) and velocity at ~100 Hz. Headless-verified:
agents spawn, walk (agent1 moved ~2.6 m in 5 s), and `/people` streams live.

> Note: use plain `ros2 topic echo /people` — `--once` may miss it due to QoS.

## Project phases

- **Phase 0 — Docker skeleton. ✅ Done.** ROS 2 Humble + Gazebo Classic 11
  container with working GUI forwarding.
- **Phase 1 — Spawn the Go2. ✅ Done.** Leg-locked rigid Go2 driven via
  `/cmd_vel` (planar_move). Headless-verified: spawn, `/cmd_vel`→`/odom` motion,
  TF tree.
- **Phase 2 — Teleop. ✅ Done.** `teleop_twist_keyboard` → `/cmd_vel` → motion.
- **Phase 3 — HuNavSim cafe in isolation. ✅ Done.** Integrated into our
  container; cafe pedestrians walk, `/people` streams live (static placeholder
  robot stands in for the real robot).
- **Phase 4 — Go2 + HuNavSim. ✅ Done.** The Go2 is the robot the pedestrians
  track; drive it among them via `/cmd_vel`. Headless-verified: Go2 spawns +
  plugin detects it, `/people` streams, `/cmd_vel` drives it.

### Run Go2 in the HuNavSim cafe (Phase 4)

```bash
./run.sh up
source ~/ros2_ws/install/setup.bash
ros2 launch go2_hunav cafe_go2.launch.py          # RViz: Go2 + walking pedestrians
# Drive it (another shell, ./run.sh shell):
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The Go2 (leg-locked planar_move base from Phase 1) is spawned as `robot`, the
model the `HuNavPlugin` tracks, so pedestrians react to it. Same launch as
Phase 3 with `use_go2:=true` (via the `cafe_go2` wrapper).

**Two upstream fixes applied** (both reproducible):
- **HuNavPlugin null-deref crash:** the plugin cast each agent's model to
  `physics::Actor` and dereferenced it unguarded, so once the (non-actor) robot
  was in the agent list `gzserver` aborted (`Actor px != 0`). A null-guard patch
  is baked into the image (`docker/Dockerfile`).
- **Scenario:** we use a regular-behavior cafe (`go2_hunav/scenarios/agents_cafe_regular.yaml`);
  agents still avoid the Go2 via the social-force model.
