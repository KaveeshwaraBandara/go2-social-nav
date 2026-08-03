cmake_minimum_required(VERSION 3.8)
project(people_oracle)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# A pure-Python library, not a node: the consuming node owns the /people
# subscription and calls this. Installs the module so any controller can
# `import people_oracle` from the workspace overlay.
ament_python_install_package(${PROJECT_NAME})

# Desk verification: builds synthetic /people messages, runs them through the
# oracle, and checks the resulting Tracks. Not a node — it neither publishes nor
# subscribes, and needs no ROS graph, no Gazebo and no robot.
install(
  PROGRAMS
    scripts/check_oracle.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
