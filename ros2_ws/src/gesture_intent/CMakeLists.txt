cmake_minimum_required(VERSION 3.8)
project(gesture_intent)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# A pure-Python library, not a node: install the module so gesture producers and
# the controller can `import gesture_intent` from the workspace overlay. No
# PROGRAMS, no launch/config — this package is a type, not a process.
# (Same shape as `tracking`, deliberately.)
ament_python_install_package(${PROJECT_NAME})

ament_package()
