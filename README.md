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
- **Phase 5 — Stub brain (closed autonomy loop). ✅ Done.** New package
  `go2_brain` with one node, `stub_brain`, that **replaces teleop as the
  `/cmd_vel` source**: it consumes `/people` + `/odom`, runs a basic Social
  Force Model, and publishes `/cmd_vel` at 20 Hz with a hard speed cap and a
  stop-if-too-close safety floor.
- **Phase 6 — Benchmark harness + Nav2 DWA/TEB baselines. ✅ Done.** New package
  `go2_bench`: a reusable harness that runs **any** `/cmd_vel`-producing brain
  through fixed pedestrian scenarios and logs **identical** metrics. Nav2 DWA
  (DWB) and TEB (built from source) are the first baselines, measured against the
  Phase-5 stub; the IT2-FLS controller (Phase 7) becomes another harness client.
- **Phase 8 — CHAMP walking base (OPTIONAL, demo/study only). 🚧 8a integrated.**
  An opt-in *physically walking* Go2 base for demo videos and the human study. It
  does **not** replace planar_move and is **never** used for benchmarking (see below).

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

## Stub brain — closed autonomy loop (Phase 5)

Package: `ros2_ws/src/go2_brain`, node `stub_brain.py`.

This is the first autonomy loop: a single node that **replaces teleop** as the
producer of `/cmd_vel`. It subscribes to `/people` (the ground-truth perception
stub) and `/odom` (the robot's own pose), runs a deliberately simple **Social
Force Model**, and publishes `/cmd_vel` at **20 Hz**:

- **Attractive** force toward a goal (`goal_x`, `goal_y` launch args).
- **Repulsive** force from each nearby person, exponential falloff with distance.
- Sum → desired world velocity → rotated into the body frame and clamped.
- The robot also steers its heading toward its direction of travel.

**Safety floor (independent of the force math):** hard linear/angular speed caps,
plus *stop-if-too-close* — if any person is within `stop_distance` the node zeroes
all translation and only rotates away from the nearest person.

It swaps in at the **permanent `/cmd_vel` contract**, so the IT2-FLS controller
(Phase 7) will later replace this node and nothing around it changes. No
perception, no fuzzy logic, no Nav2 — that's all later.

**Frame assumption (documented):** `/people` is in `map`; `/odom` is in `odom`.
The scene publishes a static **identity** `map → odom` transform, so the node
uses the `/odom` pose directly as the robot's map-frame pose (no tf2 listener).
All tunables (gains, caps, `stop_distance`, rate) live in
`go2_brain/config/stub_brain.yaml`.

### Build

```bash
./run.sh up
cd ~/ros2_ws
colcon build --packages-select go2_brain
source install/setup.bash
```

### Run + verify

```bash
# 1. Launch the Phase-4 cafe + Go2 scene AND stub_brain (instead of teleop).
#    Default goal is (0.0, -4.0); override per run, e.g. goal_x:=2.0 goal_y:=-3.0
ros2 launch go2_brain cafe_go2_brain.launch.py
# RViz opens: watch the Go2 drive itself toward the goal, slowing / steering
# around the walking pedestrians (cyan cylinders) and halting if one gets close.

# 2. In another shell, watch the node's output:
./run.sh shell
source ~/ros2_ws/install/setup.bash
ros2 topic echo /cmd_vel          # ~20 Hz Twist; linear.x/y drive, angular.z steers

# 3. Confirm it's alive and reaching the goal:
ros2 node list | grep stub_brain  # node is up
# stub_brain logs "goal reached; holding position." when within goal_tolerance,
# and warns "person within ... m: halting translation" when the safety floor fires.
```

**Verify gate:** the Go2 autonomously heads to the goal, steers around the
pedestrians, `/cmd_vel` streams at ~20 Hz, the run is crash-free, and the robot
reaches the goal without hitting anyone.

> Logic was also validated headless by feeding the node synthetic `/odom` +
> `/people`: with a clear path it commands velocity toward the goal; with a
> person inside `stop_distance` it zeroes translation and rotates away.

## Benchmark harness + Nav2 baselines (Phase 6)

Package: `ros2_ws/src/go2_bench`. The harness runs **any** `/cmd_vel`-producing
controller through a fixed set of pedestrian scenarios and logs **identical**
metrics, so the social-nav comparison (stub vs DWA vs TEB vs the future IT2-FLS)
is valid. The harness + metrics are the real deliverable; the baselines are its
first clients.

**Pieces:**
- **Scenarios** (`go2_hunav/scenarios/agents_bench_*.yaml`): three fixed,
  repeatable cases — **head_on** (1 pedestrian approaching), **crossing** (1
  pedestrian crossing the path), **group** (3-pedestrian standing group to pass).
  Robot start fixed at (0,0), goal (0,4) up the table-free north aisle; only the
  pedestrian config differs.
- **Metrics:** we reuse HuNavSim's `hunav_evaluator` for the social/proxemic
  metrics (min distance, intimate/personal/social intrusions, path, time-to-goal,
  collisions, speeds — it reads `/robot_states` + `/human_states`, so it measures
  every controller identically). `bench_runner` adds **trajectory jerk** computed
  from `/odom` resampled to a fixed 50 Hz grid (one identical computation for all
  controllers) and owns run start/stop + success/timeout.
- **Nav2 baselines:** Nav2 is just another `/cmd_vel` producer. It has no lidar,
  so the ground-truth `/people` is republished as a `PointCloud2` (`people_to_cloud`)
  into the costmap's obstacle layer — same pedestrian info the brain gets, in
  costmap form. Pedestrians live in the **local** costmap only (global plan stays
  stable). DWA = DWB; TEB = `teb_local_planner` built from source (`/opt/teb_ws`).

**Run one benchmark episode** (records a row into `results/benchmark.csv`):

```bash
ros2 launch go2_bench benchmark.launch.py scenario:=head_on controller:=stub record:=true
# controller := stub | dwa | teb     scenario := head_on | crossing | group
# args: record:=true (log metrics), run_id:=N, timeout:=45.0, goal_x/goal_y, rviz:=true
```

**Run the full comparison sweep** (one fresh container per run — HuNav's agent
manager is flaky across rapid back-to-back runs):

```bash
for sc in head_on crossing group; do for c in stub dwa teb; do
  ros2 launch go2_bench benchmark.launch.py scenario:=$sc controller:=$c \
       record:=true rviz:=false gui:=false
done; done
ros2 run go2_bench compare.py        # writes results/comparison.md
```

**Outputs** (`ros2_ws/results/`): `benchmark.csv` (master table: our jerk/path/
time/success + all `eval_*` metrics per run), `comparison.md` (stub vs DWA vs TEB
table), per-run JSON in `runs/`, raw trajectories in `trajectories/`, and the
evaluator's own CSV + per-timestep `_steps` files.

**Verify gate:** each controller drives the Go2 to goal among pedestrians and a
metrics row is logged identically; the comparison table shows the stub's smooth
low-jerk motion vs the Nav2 baselines — the gap the IT2-FLS controller (Phase 7)
must beat.

## CHAMP walking base (Phase 8, optional — demo/study only)

By default the Go2 is a **leg-locked rigid body** slid over the ground by
`planar_move` (Phase 1). That is the correct abstraction of Unitree's onboard
locomotion black box, and it is **clean, repeatable, and never falls** — so it is
the **primary base for ALL benchmarking (vs DWA/TEB) and IT2-FLS evaluation**.

Phase 8 adds a **second, optional base** that makes the Go2 *physically walk* using
the community **CHAMP** framework, so demo videos and the human-participant study
show a walking dog instead of a sliding box. **It is never used for benchmarking** —
its gait differs from the real robot, needs tuning, and can wobble (which would add
noise to the proxemics/jerk metrics). Crucially it consumes the **same `/cmd_vel`**
and publishes the **same `/odom` + TF**, so the brain/teleop/Nav2 layers are
identical across both bases.

**Source:** `anujjain-dev/unitree-go2-ros2` (humble) — CHAMP framework + a Go2
config — vendored into the image overlay `/opt/champ_ws`, pinned by commit
(`CHAMP_COMMIT` in the Dockerfile). Uses Gazebo Classic via `gazebo_ros2_control`.

### 8a — walking Go2 in an empty world

```bash
./run.sh build        # one-time: rebuilds the image with the CHAMP overlay
./run.sh up
cd ~/ros2_ws && colcon build --packages-select go2_description && source install/setup.bash
ros2 launch go2_description spawn_go2_champ.launch.py     # RViz opens; gzserver headless
# Drive it (own focused terminal):
./run.sh shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

This brings up the CHAMP control stack (quadruped controller → gait, state
estimation, two robot_localization EKFs → `/odom` + `odom→base_footprint` TF) and a
`gazebo_ros2_control`-actuated Go2 in an empty world. Like the rest of the project it
**defaults to RViz** (`gui:=false`, `rviz:=true`) because gzclient crashes on this
iGPU/Xwayland setup — RViz shows the RobotModel + TF + `/odom` so you watch the legs
articulate and the body walk. **Verify gate:** teleop drives a *stably walking* Go2
(no tipping); `/odom` + TF are sane.

> The walking description package is renamed to **`go2_champ_description`** in the
> image so it does not clash with this repo's leg-locked `go2_description`; both
> bases can be sourced together. `base:=planar_move|champ` wiring for the cafe scene
> (8b) and the stub-brain-on-CHAMP check (8c) come next. Benchmarking stays on
> planar_move.
