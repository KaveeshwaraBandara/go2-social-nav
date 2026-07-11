cmake_minimum_required(VERSION 3.8)
project(tracking)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# A pure-Python library, not a node: install the module so producers and the
# controller can `import tracking` from the workspace overlay. No PROGRAMS, no
# launch/config — this package is a type, not a process.
ament_python_install_package(${PROJECT_NAME})

ament_package()
