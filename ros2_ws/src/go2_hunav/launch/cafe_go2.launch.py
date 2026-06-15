"""Phase 4: HuNavSim cafe with the real Go2 as the tracked robot.

Thin wrapper over cafe_isolated.launch.py with use_go2:=true. The Go2 (leg-locked
planar_move base from Phase 1) is spawned as the robot the pedestrians react to;
drive it with /cmd_vel (e.g. teleop_twist_keyboard) among them.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    cafe = os.path.join(
        get_package_share_directory("go2_hunav"), "launch", "cafe_isolated.launch.py"
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cafe),
            launch_arguments={"use_go2": "true", "rviz": "true"}.items(),
        )
    ])
