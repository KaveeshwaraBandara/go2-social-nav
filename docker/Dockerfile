# ROS 2 Humble + Gazebo Classic 11 development image
# Base is pinned by tag; capture the digest with:  docker image inspect osrf/ros:humble-desktop
FROM osrf/ros:humble-desktop

# ---- Build-time arguments -------------------------------------------------
# Mapped to the host user so files created in the bind-mounted ros2_ws/ are
# owned by you on the host, not by root.
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
# Host group IDs that own the GPU render nodes (/dev/dri). Lets the container
# user access Intel iGPU acceleration. Override at build time if yours differ.
ARG RENDER_GID=992
ARG VIDEO_GID=44

ENV DEBIAN_FRONTEND=noninteractive

# ###########################################################################
# DEPENDENCIES FIRST.
# Everything below up to the "SOURCE BUILDS" banner is pure dependency
# installation (apt + pip + vendored data) — the stable, expensive-to-download
# part of the image. It is deliberately placed before any git clone / colcon
# build so that editing a build step (re-pinning a commit, tweaking a patch)
# never invalidates these layers: the heavy TensorFlow/MediaPipe and Nav2
# downloads stay cached and rebuilds are fast.
# ###########################################################################

# ---- apt (1/2): base ROS 2 + Gazebo Classic + build tooling ----------------
# Gazebo Classic 11 arrives via ros-humble-gazebo-ros-pkgs (the canonical
# Humble <-> Gazebo Classic pairing). teleop_twist_keyboard + xacro + tf2_tools
# are needed in later phases; build-essential/cmake/git serve every source
# build further down; sudo is for the non-root dev user.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        ros-humble-teleop-twist-keyboard \
        ros-humble-xacro \
        ros-humble-tf2-tools \
        ros-humble-tf2-ros \
        python3-colcon-common-extensions \
        build-essential \
        cmake \
        git \
        nano \
        mesa-utils \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# ---- Non-root user ---------------------------------------------------------
# Create the user/group, and ensure render/video groups exist with the host
# GIDs so /dev/dri passthrough is usable. groupadd may collide if a GID is
# already taken by a base-image group; the '|| true' tolerates that. (sudo is
# installed in the base apt layer above.)
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && (getent group ${RENDER_GID} || groupadd --gid ${RENDER_GID} hostrender) \
    && (getent group ${VIDEO_GID}  || groupadd --gid ${VIDEO_GID}  hostvideo) \
    && usermod -aG ${RENDER_GID},${VIDEO_GID} ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# ---- apt (2/2): all remaining ROS / system dependencies --------------------
# One consolidated layer for every source build's runtime + build dependencies,
# grouped by the phase that needs them:
#   HuNavSim (Phase 3): behaviortree_cpp (apt ships 4.9, so unlike upstream we do
#     NOT build BehaviorTree from source), nav2 bt/msgs/util, tf-transformations,
#     filters, gazebo-plugins, pandas, eigen.
#   Nav2 baselines (Phase 6): navigation2 + nav2_bringup (DWB local planner).
#   TEB build deps (Phase 6d): libg2o, SuiteSparse, OpenCV, cv_bridge.
#   CHAMP walking base (Phase 8): robot_localization (EKFs), ros2_controllers
#     + ros2controlcli (the `ros2 control load_controller` the gazebo launch
#     calls — without it the legs never actuate), velodyne-simulator.
#   Gesture recognition (Phase 9): libGL + libgthread (OpenCV GUI runtime .so
#     deps for the X11-forwarded camera window), pip.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-behaviortree-cpp \
        ros-humble-nav2-behavior-tree \
        ros-humble-nav2-msgs \
        ros-humble-nav2-util \
        ros-humble-tf-transformations \
        ros-humble-filters \
        ros-humble-gazebo-plugins \
        python3-pandas \
        libeigen3-dev \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-libg2o \
        libsuitesparse-dev \
        libopencv-dev \
        ros-humble-cv-bridge \
        ros-humble-robot-localization \
        ros-humble-ros2-controllers \
        ros-humble-ros2controlcli \
        ros-humble-velodyne-simulator \
        libgl1 \
        libglib2.0-0 \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ---- pip: gesture Python runtime deps (Phase 9) ----------------------------
# MediaPipe Tasks API (>=0.10) for HandLandmarker, opencv-python (NOT headless —
# the node shows a live camera window via the forwarded X11 display), and
# TensorFlow for tf.lite.Interpreter (KeyPointClassifier). No venv — the
# container IS the isolation, and this is the same Python 3.10 that owns rclpy.
# numpy<2 is pinned: the apt scipy/pandas above were compiled against NumPy 1.x
# and NumPy 2.x breaks their C extensions (ABI mismatch).
# This is the single largest download in the image; kept here in the deps
# section so no later source-build edit can force it to re-run.
RUN pip3 install --no-cache-dir \
        "numpy<2" \
        mediapipe \
        opencv-python \
        tensorflow

# ---- Vendored Gazebo models (cafe world data; stable) ----------------------
# cafe.world references model://cafe, model://cafe_table, model://ground_plane.
# These are standard Gazebo Classic models NOT shipped with the package; rather
# than rely on a (now-retired) online model DB at first run, we vendor them.
RUN git clone --depth 1 --filter=blob:none --sparse \
        https://github.com/osrf/gazebo_models.git /opt/gazebo_models \
    && cd /opt/gazebo_models \
    && git sparse-checkout set cafe cafe_table ground_plane sun \
    && rm -rf /opt/gazebo_models/.git
ENV GAZEBO_MODEL_PATH=/opt/gazebo_models

# ###########################################################################
# SOURCE BUILDS.
# Volatile, commit-pinned git clones + colcon/make builds. Each build's pinned
# commit ARG sits immediately before its RUN (not at the top of the file) so
# that re-pinning one commit invalidates only that build's layer and the ones
# after it — never the dependency layers above. All run as root; the USER
# switch is at the very end.
# ###########################################################################

# ---- lightsfm (Social Force Model library, make-installed system-wide) -----
# Required by the HuNavSim build below, so it comes first.
ARG LIGHTSFM_COMMIT=b30327cca189af2fb90443a5d0040cceb46d7195
RUN git clone https://github.com/robotics-upo/lightsfm.git /tmp/lightsfm \
    && cd /tmp/lightsfm && git checkout ${LIGHTSFM_COMMIT} \
    && make && make install \
    && rm -rf /tmp/lightsfm

# ---- HuNavSim packages prebuilt into an image overlay (/opt/hunav_ws) -------
# Only the packages required to run the cafe demo are built (agent_manager,
# evaluator, gazebo_wrapper + their deps incl. people_msgs). The Qt5 rviz panel
# and the openai-based behavior_tree_generator are intentionally skipped.
ARG PEOPLE_COMMIT=0ae47f6e0208cedd84d19d066743fdc1d05fcafa
ARG HUNAV_SIM_COMMIT=8ecd594d57e921ce735d330782a008b7850672c5
ARG HUNAV_GAZEBO_WRAPPER_COMMIT=70b11896a105cb77d7faad9a9d518d92bafd8076
RUN mkdir -p /opt/hunav_ws/src && cd /opt/hunav_ws/src \
    && git clone -b ros2 https://github.com/wg-perception/people.git \
       && git -C people checkout ${PEOPLE_COMMIT} \
    && git clone -b v2.0 https://github.com/robotics-upo/hunav_sim.git \
       && git -C hunav_sim checkout ${HUNAV_SIM_COMMIT} \
    && git clone -b v2.0 https://github.com/robotics-upo/hunav_gazebo_wrapper.git \
       && git -C hunav_gazebo_wrapper checkout ${HUNAV_GAZEBO_WRAPPER_COMMIT} \
    && cd /opt/hunav_ws \
    && . /opt/ros/humble/setup.sh \
    && colcon build \
        --packages-up-to hunav_agent_manager hunav_evaluator hunav_gazebo_wrapper \
    && rm -rf /opt/hunav_ws/build /opt/hunav_ws/log

# Patch a HuNavPlugin crash: it casts each agent's Gazebo model to physics::Actor
# and dereferences it without a null check, so when the (non-actor) robot model
# is in the agent list, gzserver aborts (Actor px != 0). Guard the deref.
RUN sed -i \
      "s#ignition::math::Pose3d actorPose = actor->WorldPose();#if (!actor) continue;  // skip non-actor models (e.g. the robot)\n    ignition::math::Pose3d actorPose = actor->WorldPose();#g" \
      /opt/hunav_ws/src/hunav_gazebo_wrapper/src/HuNavPlugin.cpp \
    && cd /opt/hunav_ws && . /opt/ros/humble/setup.sh \
    && colcon build --packages-select hunav_gazebo_wrapper \
    && rm -rf /opt/hunav_ws/build /opt/hunav_ws/log

# The hunav_gazebo_world_generator writes generatedWorld.world next to the base
# world file (inside this overlay). Hand the overlay to the dev user so that
# write succeeds at runtime.
RUN chown -R ${USER_UID}:${USER_GID} /opt/hunav_ws

# ---- TEB local planner (Phase 6d) built from source -------------------------
# No Humble binary exists for teb_local_planner / costmap_converter, so we build
# them from their upstream Humble branches into an overlay (/opt/teb_ws), pinned
# by commit. This gives the nav2 controller plugin
# `teb_local_planner::TebLocalPlannerROS` -- the TEB baseline. Its build deps
# (g2o, SuiteSparse, OpenCV, cv_bridge) are installed in the apt layer above.
ARG COSTMAP_CONVERTER_COMMIT=9565858432664e9cfe0b3556e1809336c4ab06d4
ARG TEB_COMMIT=630a22e88dc9fd45be726a762edbb5b776bef231
RUN mkdir -p /opt/teb_ws/src && cd /opt/teb_ws/src \
    && git clone -b humble https://github.com/rst-tu-dortmund/costmap_converter.git \
       && git -C costmap_converter checkout ${COSTMAP_CONVERTER_COMMIT} \
    && git clone -b humble-devel https://github.com/rst-tu-dortmund/teb_local_planner.git \
       && git -C teb_local_planner checkout ${TEB_COMMIT} \
    && cd /opt/teb_ws \
    && . /opt/ros/humble/setup.sh \
    && colcon build --packages-up-to teb_local_planner \
    && rm -rf /opt/teb_ws/build /opt/teb_ws/log

# ---- CHAMP walking base (Phase 8, OPTIONAL) ---------------------------------
# A SECOND, opt-in locomotion base for demo videos / the human-participant study
# only. planar_move stays the DEFAULT and the ONLY base for benchmarking and
# IT2-FLS eval (CLAUDE.md principle 2). CHAMP consumes the SAME /cmd_vel and
# publishes /odom + a consistent TF tree, so brain/teleop/Nav2 drive it unchanged.
#
# Source: anujjain-dev/unitree-go2-ros2 (humble) = the CHAMP framework
# (chvmp/champ) + a Go2 config, built into its own overlay (/opt/champ_ws).
# Gazebo Classic via gazebo_ros2_control. Its runtime deps (robot_localization,
# ros2_controllers, ros2controlcli, velodyne-simulator) are in the apt layer above.
#
# NAME-COLLISION FIX: the upstream repo ships its OWN `go2_description` package
# (the actuated walking xacro) whose name clashes with OUR leg-locked
# go2_description in ros2_ws. We rename the CHAMP one to `go2_champ_description`
# (package + every `$(find ...)` / FindPackageShare reference) so BOTH overlays
# can be sourced together without shadowing each other.
ARG CHAMP_COMMIT=fdac259518930c8f03e79189c4d0fdd00fc47483
RUN mkdir -p /opt/champ_ws/src && cd /opt/champ_ws/src \
    && git clone https://github.com/anujjain-dev/unitree-go2-ros2.git \
       && git -C unitree-go2-ros2 checkout ${CHAMP_COMMIT} \
    && cd unitree-go2-ros2 \
    && D=robots/descriptions/go2_description \
    && for f in $D/package.xml $D/CMakeLists.txt $D/launch/description.launch.py \
                $D/xacro/gazebo.xacro $D/xacro/leg.xacro $D/xacro/laser.xacro \
                $D/xacro/robot.xacro $D/xacro/robot_VLP.xacro $D/xacro/velodyne.xacro \
                robots/configs/go2_config/launch/gazebo.launch.py \
                robots/configs/go2_config/launch/gazebo_velodyne.launch.py; do \
         sed -i "s/go2_description/go2_champ_description/g" "$f" || true; \
       done \
    # Phase 8b traction fix: the upstream CHAMP feet (mu 0.6) SLIP on the cafe floor
    # — legs step but the body barely translates (forward worst; turning still works).
    # The cafe ground_plane is mu 100, so the foot is the limiter in Gazebo Classic's
    # min() friction combine; raising foot mu to 1.5 makes them grip and push off
    # (~0.016 -> ~0.26 m/s forward). 0.6 is unique to the foot links here.
    && sed -i "s|<mu1>0.6</mu1>|<mu1>1.5</mu1>|g; s|<mu2>0.6</mu2>|<mu2>1.5</mu2>|g" \
            $D/xacro/gazebo.xacro \
    # Phase 8b gait tuning: stock gait caps forward at 0.3 m/s and lifts the feet only
    # 0.04 m, so they SCUFF instead of planting — the robot crawls AND the leg odometry
    # under-counts ~4-5x (it sees scuffing, not clean strides), so it looks frozen in
    # RViz. swing_height 0.04->0.08 (clean plant-and-push) + max vel 0.3->0.6 give a
    # believable ~0.5 m/s walk whose /odom matches ground truth (~0.9 ratio).
    && sed -i "s/max_linear_velocity_x : 0.3/max_linear_velocity_x : 0.6/; \
               s/swing_height : 0.04/swing_height : 0.08/; \
               s/stance_depth : 0.01/stance_depth : 0.02/" \
            robots/configs/go2_config/config/gait/gait.yaml \
    && mv $D robots/descriptions/go2_champ_description \
    && cd /opt/champ_ws && . /opt/ros/humble/setup.sh \
    && colcon build \
    && rm -rf /opt/champ_ws/build /opt/champ_ws/log
RUN chown -R ${USER_UID}:${USER_GID} /opt/champ_ws

# ---- Entry point + interactive shell setup ---------------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws

# Convenience: auto-source ROS, the HuNavSim overlay, and our workspace in
# every interactive shell.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /opt/hunav_ws/install/setup.bash ] && source /opt/hunav_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /opt/teb_ws/install/setup.bash ] && source /opt/teb_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /opt/champ_ws/install/setup.bash ] && source /opt/champ_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /home/${USERNAME}/ros2_ws/install/setup.bash ] && source /home/${USERNAME}/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
