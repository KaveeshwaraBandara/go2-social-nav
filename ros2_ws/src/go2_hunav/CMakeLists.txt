cmake_minimum_required(VERSION 3.8)
project(go2_hunav)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch models rviz scenarios
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/people_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
