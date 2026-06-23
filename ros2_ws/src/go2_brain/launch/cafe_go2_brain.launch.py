"""Phase 5: the closed autonomy loop — cafe + Go2 + stub_brain (no teleop).

This brings up the EXACT Phase-4 scene (go2_hunav's cafe_go2 = cafe pedestrians
+ the leg-locked Go2 the agents track) and then runs stub_brain as the /cmd_vel
producer INSTEAD of teleop. We reuse the existing scene launch verbatim and only
add one node, honoring "replace one component at a time".

Run:
  ros2 launch go2_brain cafe_go2_brain.launch.py
  ros2 launch go2_brain cafe_go2_brain.launch.py goal_x:=2.0 goal_y:=-3.0

The goal defaults to (0.0, -4.0): the robot spawns at the origin and heads south
through the cafe, crossing the pedestrians' walking lanes so the repulsion /
safety floor are actually exercised. Tune the goal with the launch args above.
All other knobs live in go2_brain/config/stub_brain.yaml.

Phase 8c: `base:=planar_move` (default) or `base:=champ` selects the base, passed
straight through to cafe_go2. stub_brain is UNCHANGED across both — it just
consumes /people + /odom and publishes /cmd_vel, proving the contract holds.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")

    # Reuse the Phase-4 scene launch as-is (cafe pedestrians + Go2 as robot).
    cafe_go2 = os.path.join(
        get_package_share_directory("go2_hunav"), "launch", "cafe_go2.launch.py"
    )
    params_file = os.path.join(
        get_package_share_directory("go2_brain"), "config", "stub_brain.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("goal_x", default_value="0.0"),
        DeclareLaunchArgument("goal_y", default_value="-4.0"),
        # base passthrough (Phase 8c): planar_move (default) | champ. Launch-only —
        # the stub_brain node below is identical regardless of the base.
        DeclareLaunchArgument("base", default_value="planar_move"),
        # lidar passthrough: only takes effect when base:=champ. Mounts the
        # Velodyne VLP-16 (default true); pass lidar:=false to omit it.
        DeclareLaunchArgument("lidar", default_value="true"),

        # The full Phase-4 scene (cafe + walking pedestrians + the selected base).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cafe_go2),
            launch_arguments={
                "base": LaunchConfiguration("base"),
                "lidar": LaunchConfiguration("lidar"),
            }.items(),
        ),

        # The autonomy brain: /people + /odom -> /cmd_vel @ 20 Hz. The goal comes
        # from the launch args; everything else from the params file. use_sim_time
        # so its clock matches Gazebo's.
        Node(
            package="go2_brain",
            executable="stub_brain.py",
            name="stub_brain",
            output="screen",
            parameters=[
                params_file,
                {"goal_x": goal_x, "goal_y": goal_y, "use_sim_time": True},
            ],
        ),
    ])
