cmake_minimum_required(VERSION 3.8)
project(go2_brain)

find_package(ament_cmake REQUIRED)

# The node is a plain rclpy script (same pattern as go2_hunav's people_markers.py):
# install it as an executable program, ship launch + config as data.
install(
  PROGRAMS scripts/stub_brain.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
