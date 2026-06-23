"""
Phase 1 launch: start Gazebo Classic, publish the Go2 description, and spawn
the (leg-locked) Go2 as a planar-move driving base.

Brings up:
  - gzserver (physics) + optionally gzclient (GUI)
  - robot_state_publisher  (publishes /robot_description and base->* TFs)
  - spawn_entity           (spawns the Go2 from /robot_description)

Args:
  gui:=true|false      Show the Gazebo GUI (default true). false = headless.
  world:=<path>        World file (default: this package's empty.world).
  x, y, z, yaw:=<f>    Spawn pose. z defaults to 0.33 (computed standing height).
  lidar:=true|false    Mount the Velodyne VLP-16 (default true). Needs
                        ros-humble-velodyne-simulator in the image.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_go2 = get_package_share_directory("go2_description")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    xacro_path = os.path.join(pkg_go2, "urdf", "go2.xacro")

    default_world = os.path.join(pkg_go2, "worlds", "empty.world")

    # Args
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    lidar = LaunchConfiguration("lidar")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_path, " lidar:=", lidar]), value_type=str
    )

    declare_args = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.33"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("lidar", default_value="true"),
    ]

    # Help Gazebo Classic resolve package:// mesh URIs by adding the directory
    # that CONTAINS the package share (i.e. .../share) to GAZEBO_MODEL_PATH.
    set_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.path.dirname(pkg_go2) + os.pathsep + os.environ.get("GAZEBO_MODEL_PATH", ""),
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world, "verbose": "true"}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "go2",
            "-x", x, "-y", y, "-z", z, "-Y", yaw,
        ],
    )

    return LaunchDescription(
        declare_args
        + [set_model_path, gzserver, gzclient, robot_state_publisher, spawn_entity]
    )
