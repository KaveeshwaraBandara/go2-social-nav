cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

# Install model assets and launch files into the package share directory.
install(
  DIRECTORY dae urdf launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
