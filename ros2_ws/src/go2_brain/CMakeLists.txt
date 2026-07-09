cmake_minimum_required(VERSION 3.8)
project(go2_brain)

find_package(ament_cmake REQUIRED)

# The nodes are plain rclpy scripts (same pattern as go2_hunav's people_markers.py):
# install them as executable programs, ship launch + config as data.
#   stub_brain.py    -- Phase 5 Social Force Model placeholder
#   it2fls_brain.py  -- Phase 7 Interval Type-2 Fuzzy Logic controller
#   it2_fls.py       -- the ROS-free fuzzy engine imported by it2fls_brain.py
#                       (installed in the SAME dir so the import resolves)
install(
  PROGRAMS
    scripts/stub_brain.py
    scripts/it2fls_brain.py
    scripts/it2_fls.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
