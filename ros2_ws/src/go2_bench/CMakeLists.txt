cmake_minimum_required(VERSION 3.8)
project(go2_bench)

find_package(ament_cmake REQUIRED)

# bench_runner is a plain rclpy script (same pattern as go2_brain's stub_brain.py):
# install it as an executable program, ship launch + config as data.
install(
  PROGRAMS
    scripts/bench_runner.py
    scripts/people_to_cloud.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
