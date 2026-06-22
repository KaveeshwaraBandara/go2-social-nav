#!/usr/bin/env bash
# Container entrypoint: source ROS 2 (and the workspace overlay if built),
# then exec whatever command was passed (defaults to bash).
set -e

source /opt/ros/humble/setup.bash

# HuNavSim overlay (prebuilt into the image at /opt/hunav_ws).
if [ -f /opt/hunav_ws/install/setup.bash ]; then
    source /opt/hunav_ws/install/setup.bash
fi

# TEB local planner overlay (built from source at /opt/teb_ws -- Phase 6d).
if [ -f /opt/teb_ws/install/setup.bash ]; then
    source /opt/teb_ws/install/setup.bash
fi

# CHAMP walking-base overlay (built into the image at /opt/champ_ws -- Phase 8,
# OPTIONAL). Provides the champ_* control stack + go2_config + go2_champ_description
# for the opt-in walking base. planar_move (in ros2_ws) stays the default.
if [ -f /opt/champ_ws/install/setup.bash ]; then
    source /opt/champ_ws/install/setup.bash
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

exec "$@"
