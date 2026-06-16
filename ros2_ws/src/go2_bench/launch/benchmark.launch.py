"""Phase 6 benchmark harness launch.

The harness runs ANY /cmd_vel-producing brain through identical pedestrian
scenarios. This launch brings up:

  1. The fixed cafe scene + leg-locked Go2 (reuses go2_hunav/cafe_isolated.launch
     verbatim, use_go2:=true) with the chosen benchmark SCENARIO's agents.
  2. The chosen CONTROLLER as the /cmd_vel producer.

Robot start is fixed at (0,0) (hard-coded in cafe_isolated) and the goal defaults
to (0,4) -- a straight NORTH path up the table-free aisle -- so every controller
faces identical conditions; only the pedestrian config changes per scenario.

Usage (6a -- verify scenarios with the Phase-5 stub brain):
  ros2 launch go2_bench benchmark.launch.py scenario:=head_on  controller:=stub
  ros2 launch go2_bench benchmark.launch.py scenario:=crossing controller:=stub
  ros2 launch go2_bench benchmark.launch.py scenario:=group    controller:=stub

Args:
  scenario   : head_on | crossing | group   (selects the agents YAML)
  controller : stub                          (Nav2 dwa/teb added in 6c/6d)
  goal_x, goal_y : robot goal in the map frame (default 0.0, 4.0)
  rviz       : show RViz (default true)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Benchmark scenario name -> agents YAML in go2_hunav/scenarios.
SCENARIOS = {
    "head_on": "agents_bench_head_on.yaml",
    "crossing": "agents_bench_crossing.yaml",
    "group": "agents_bench_group.yaml",
}


def launch_setup(context, *args, **kwargs):
    scenario = LaunchConfiguration("scenario").perform(context)
    controller = LaunchConfiguration("controller").perform(context)
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    rviz = LaunchConfiguration("rviz")

    if scenario not in SCENARIOS:
        raise RuntimeError(
            f"Unknown scenario '{scenario}'. Choose one of: {', '.join(SCENARIOS)}"
        )
    config_file = SCENARIOS[scenario]

    # 1) The fixed cafe scene + Go2, with this scenario's agents. Reuse the
    #    Phase-3/4 launch as-is; only feed it the scenario file + rviz flag.
    cafe = os.path.join(
        get_package_share_directory("go2_hunav"), "launch", "cafe_isolated.launch.py"
    )
    scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cafe),
        launch_arguments={
            "use_go2": "true",
            "rviz": rviz,
            "configuration_file": config_file,
        }.items(),
    )

    actions = [scene]

    # 2) The controller under test -- the /cmd_vel producer.
    if controller == "stub":
        stub_params = os.path.join(
            get_package_share_directory("go2_brain"), "config", "stub_brain.yaml"
        )
        actions.append(Node(
            package="go2_brain",
            executable="stub_brain.py",
            name="stub_brain",
            output="screen",
            parameters=[
                stub_params,
                {"goal_x": goal_x, "goal_y": goal_y, "use_sim_time": True},
            ],
        ))
    else:
        # Nav2 DWA/TEB controllers are wired in Phase 6c/6d.
        raise RuntimeError(
            f"Unknown controller '{controller}'. Only 'stub' is wired so far "
            f"(Nav2 dwa/teb come in 6c/6d)."
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario", default_value="head_on",
            description="Benchmark scenario: head_on | crossing | group"),
        DeclareLaunchArgument(
            "controller", default_value="stub",
            description="Controller under test: stub (Nav2 dwa/teb in 6c/6d)"),
        DeclareLaunchArgument("goal_x", default_value="0.0"),
        DeclareLaunchArgument("goal_y", default_value="4.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
