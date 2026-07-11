cmake_minimum_required(VERSION 3.8)
project(fusion_lab)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# A pure-Python library (fusion core + Go2 adapters), not a node. Installs the
# module so the producer is importable from the workspace overlay.
ament_python_install_package(${PROJECT_NAME})

# Rig calibration (K + T_cam_from_odom) recovered offline by
# scripts/calibrate_go2.py. Small, permanent, and code-adjacent: without it
# there is no fusion at all, so it is versioned WITH the code — unlike the
# rosbags, which are gigabytes and live outside the repo.
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
