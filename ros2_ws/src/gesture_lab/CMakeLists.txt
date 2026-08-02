cmake_minimum_required(VERSION 3.8)
project(gesture_lab)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# A pure-Python library (vendored vision core + our intent adapter), not a node.
# Installs the module so the producer is importable from the workspace overlay.
ament_python_install_package(${PROJECT_NAME})

# The OFFLINE DRIVER: webcam -> pose lock -> classifier -> GestureIntent. Not a
# node — it neither publishes nor subscribes; it is how the producer runs at the
# desk with no robot and no ROS graph. `--selftest` runs the contract and the
# adapter with no camera, no display and no trained model, which is the only
# part of this phase that is verifiable until gesture_model.pkl arrives.
install(
  PROGRAMS
    scripts/run_gesture.py
  DESTINATION lib/${PROJECT_NAME}
)

# The trained RandomForest bundle lives in model/ and is NOT in the repo yet —
# upstream's .gitignore excludes *.pkl, so it has to be fetched from the gesture
# repo owner. The directory ships regardless so the install path is stable and
# the missing file produces one clear error from load_model() rather than a
# packaging difference between machines.
install(
  DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
