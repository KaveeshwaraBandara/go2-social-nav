"""Phase 4: HuNavSim cafe with the real Go2 as the tracked robot.

Thin wrapper over cafe_isolated.launch.py with use_go2:=true. The Go2 is spawned
as the robot the pedestrians react to; drive it with /cmd_vel (e.g.
teleop_twist_keyboard) among them.

Phase 8b: `base:=planar_move` (DEFAULT, the leg-locked Phase-1 base used for all
benchmarking) or `base:=champ` (OPTIONAL CHAMP walking base, demo/study only).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cafe = os.path.join(
        get_package_share_directory("go2_hunav"), "launch", "cafe_isolated.launch.py"
    )
    return LaunchDescription([
        DeclareLaunchArgument("base", default_value="planar_move"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cafe),
            launch_arguments={
                "use_go2": "true",
                "rviz": "true",
                "base": LaunchConfiguration("base"),
            }.items(),
        )
    ])
