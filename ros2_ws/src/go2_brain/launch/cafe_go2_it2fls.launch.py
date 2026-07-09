"""Phase 7: the IT2-FLS closed loop -- cafe + Go2 + it2fls_brain (no teleop).

Identical in structure to cafe_go2_brain.launch.py (Phase 5), but runs the
Interval Type-2 Fuzzy Logic controller as the /cmd_vel producer instead of the
stub Social Force Model. Everything around it -- the cafe scene, the pedestrians,
the Go2 base -- is unchanged, which is the whole point of the permanent /cmd_vel
contract.

Run:
  ros2 launch go2_brain cafe_go2_it2fls.launch.py
  ros2 launch go2_brain cafe_go2_it2fls.launch.py goal_x:=2.0 goal_y:=-3.0
  ros2 launch go2_brain cafe_go2_it2fls.launch.py base:=champ    # walking demo

All fuzzy knobs live in go2_brain/config/it2fls_brain.yaml; the membership
functions themselves live in go2_brain/scripts/it2_fls.py.
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

    cafe_go2 = os.path.join(
        get_package_share_directory("go2_hunav"), "launch", "cafe_go2.launch.py"
    )
    params_file = os.path.join(
        get_package_share_directory("go2_brain"), "config", "it2fls_brain.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("goal_x", default_value="0.0"),
        DeclareLaunchArgument("goal_y", default_value="-4.0"),
        DeclareLaunchArgument("base", default_value="planar_move"),
        DeclareLaunchArgument("lidar", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cafe_go2),
            launch_arguments={
                "base": LaunchConfiguration("base"),
                "lidar": LaunchConfiguration("lidar"),
            }.items(),
        ),

        Node(
            package="go2_brain",
            executable="it2fls_brain.py",
            name="it2fls_brain",
            output="screen",
            parameters=[
                params_file,
                {"goal_x": goal_x, "goal_y": goal_y, "use_sim_time": True},
            ],
        ),
    ])
