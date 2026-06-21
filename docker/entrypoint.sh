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

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

exec "$@"
