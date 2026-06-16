cmake_minimum_required(VERSION 3.8)
project(go2_bench)

find_package(ament_cmake REQUIRED)

# Harness launch + (later) metrics config. Logger node scripts are installed as
# executables in 6b.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
