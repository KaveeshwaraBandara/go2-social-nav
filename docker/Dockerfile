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

# Pinned commits for the HuNavSim stack (Phase 3). Reproducible by hash.
ARG LIGHTSFM_COMMIT=b30327cca189af2fb90443a5d0040cceb46d7195
ARG PEOPLE_COMMIT=0ae47f6e0208cedd84d19d066743fdc1d05fcafa
ARG HUNAV_SIM_COMMIT=8ecd594d57e921ce735d330782a008b7850672c5
ARG HUNAV_GAZEBO_WRAPPER_COMMIT=70b11896a105cb77d7faad9a9d518d92bafd8076

ENV DEBIAN_FRONTEND=noninteractive

# ---- System + ROS/Gazebo packages -----------------------------------------
# Gazebo Classic 11 arrives via ros-humble-gazebo-ros-pkgs (the canonical
# Humble <-> Gazebo Classic pairing). teleop_twist_keyboard + xacro + tf2_tools
# are needed in later phases; installed now so the image is stable.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        ros-humble-teleop-twist-keyboard \
        ros-humble-xacro \
        ros-humble-tf2-tools \
        ros-humble-tf2-ros \
        python3-colcon-common-extensions \
        build-essential \
        git \
        nano \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# ---- Non-root user ---------------------------------------------------------
# Create the user/group, and ensure render/video groups exist with the host
# GIDs so /dev/dri passthrough is usable. groupadd may collide if a GID is
# already taken by a base-image group; the '|| true' tolerates that.
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && (getent group ${RENDER_GID} || groupadd --gid ${RENDER_GID} hostrender) \
    && (getent group ${VIDEO_GID}  || groupadd --gid ${VIDEO_GID}  hostvideo) \
    && usermod -aG ${RENDER_GID},${VIDEO_GID} ${USERNAME} \
    && apt-get update && apt-get install -y --no-install-recommends sudo \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

# ===========================================================================
# HuNavSim layer (Phase 3) — integrated into THIS container (Option B).
# We reuse the official robotics-upo HuNavSim stack but install only what the
# cafe demo needs (no PMB2 / nav2 robot stack). NOTE: apt now ships
# behaviortree_cpp 4.9, so unlike upstream we do NOT build BehaviorTree from
# source — only people_msgs (not packaged) and the hunav packages are built.
# ===========================================================================

# ---- HuNavSim apt dependencies --------------------------------------------
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
        cmake \
    && rm -rf /var/lib/apt/lists/*

# ---- lightsfm (Social Force Model library, make-installed system-wide) -----
RUN git clone https://github.com/robotics-upo/lightsfm.git /tmp/lightsfm \
    && cd /tmp/lightsfm && git checkout ${LIGHTSFM_COMMIT} \
    && make && make install \
    && rm -rf /tmp/lightsfm

# ---- HuNavSim packages prebuilt into an image overlay (/opt/hunav_ws) -------
# Only the packages required to run the cafe demo are built (agent_manager,
# evaluator, gazebo_wrapper + their deps incl. people_msgs). The Qt5 rviz panel
# and the openai-based behavior_tree_generator are intentionally skipped.
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

# ---- Bundle the Gazebo models the cafe world needs -------------------------
# cafe.world references model://cafe, model://cafe_table, model://ground_plane.
# These are standard Gazebo Classic models NOT shipped with the package; rather
# than rely on a (now-retired) online model DB at first run, we vendor them.
RUN git clone --depth 1 --filter=blob:none --sparse \
        https://github.com/osrf/gazebo_models.git /opt/gazebo_models \
    && cd /opt/gazebo_models \
    && git sparse-checkout set cafe cafe_table ground_plane sun \
    && rm -rf /opt/gazebo_models/.git
ENV GAZEBO_MODEL_PATH=/opt/gazebo_models

# ===========================================================================
# Nav2 stack (Phase 6) — the DWA/TEB baselines the IT2-FLS controller is
# measured against. Installed as its own layer so the expensive HuNavSim layers
# above stay cached. navigation2 + nav2_bringup give the controller/planner/
# bt_navigator/costmaps incl. the DWB local planner (Phase 6c DWA baseline).
# Nav2 is just another /cmd_vel producer for our harness — it drives the same
# planar_move Go2 over the same contract.
# NOTE: TEB (Phase 6d) is NOT apt-available for Humble (no ros-humble-teb-local-
# planner); it will be added separately (source build or a substitute controller).
# ===========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# ---- TEB local planner (Phase 6d) built from source -------------------------
# No Humble binary exists for teb_local_planner / costmap_converter, so we build
# them from their upstream Humble branches into an overlay (/opt/teb_ws), pinned by
# commit for reproducibility. This gives the nav2 controller plugin
# `teb_local_planner::TebLocalPlannerROS` -- the TEB baseline. g2o (optimizer),
# SuiteSparse and OpenCV are the build deps.
ARG COSTMAP_CONVERTER_COMMIT=9565858432664e9cfe0b3556e1809336c4ab06d4
ARG TEB_COMMIT=630a22e88dc9fd45be726a762edbb5b776bef231
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-libg2o \
        libsuitesparse-dev \
        libopencv-dev \
        ros-humble-cv-bridge \
    && mkdir -p /opt/teb_ws/src && cd /opt/teb_ws/src \
    && git clone -b humble https://github.com/rst-tu-dortmund/costmap_converter.git \
       && git -C costmap_converter checkout ${COSTMAP_CONVERTER_COMMIT} \
    && git clone -b humble-devel https://github.com/rst-tu-dortmund/teb_local_planner.git \
       && git -C teb_local_planner checkout ${TEB_COMMIT} \
    && cd /opt/teb_ws \
    && . /opt/ros/humble/setup.sh \
    && colcon build --packages-up-to teb_local_planner \
    && rm -rf /opt/teb_ws/build /opt/teb_ws/log \
    && rm -rf /var/lib/apt/lists/*

# ---- Entry point -----------------------------------------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws

# Convenience: auto-source ROS, the HuNavSim overlay, and our workspace in
# every interactive shell.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /opt/hunav_ws/install/setup.bash ] && source /opt/hunav_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /opt/teb_ws/install/setup.bash ] && source /opt/teb_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /home/${USERNAME}/ros2_ws/install/setup.bash ] && source /home/${USERNAME}/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
