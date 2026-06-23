"""Phase 8a (OPTIONAL): CHAMP walking Go2 in an EMPTY world, driven by /cmd_vel.

This is the SECOND, opt-in locomotion base (CLAUDE.md principle 2): a physically
WALKING Go2 (CHAMP framework) for demo videos / the human-participant study only.
planar_move (spawn_go2.launch.py) remains the DEFAULT and the ONLY benchmarking
base. CHAMP consumes the SAME /cmd_vel and publishes /odom + a consistent TF tree,
so every brain node above /cmd_vel (teleop, stub_brain, the Nav2 baselines) is
unchanged across both bases.

8a brings the walking Go2 up in ISOLATION (empty world) so its gait / foot
friction can be tuned before it goes into the cafe scene (8b). It composes the
upstream CHAMP launches (vendored + pinned in the image overlay /opt/champ_ws):

  champ_bringup/bringup.launch.py
    - quadruped_controller_node : /cmd_vel -> gait -> joint trajectories
    - state_estimation_node     : joint_states/contacts -> odom/raw
    - two robot_localization EKFs: -> /odom + the odom->base_footprint TF
    - robot_state_publisher     : /robot_description (+ base_footprint->links TF)
  champ_gazebo/gazebo.launch.py
    - gzserver on `world`, spawns the actuated Go2 from /robot_description,
      and loads the ros2_control controllers (joint_state_broadcaster +
      joint_trajectory_controller via `ros2 control load_controller`).

NOTE (name collision): the walking description package is `go2_champ_description`
(renamed in the image from upstream `go2_description`) so it does NOT clash with
OUR leg-locked `go2_description`. Joints/links/gait come from `go2_config`. Both
packages live in the /opt/champ_ws overlay, sourced alongside ros2_ws.

Drive it (own focused terminal that has keyboard focus):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Visualization: like the rest of this project, gzclient crashes on this iGPU /
Xwayland setup (Gazebo-Classic Camera bug), so this defaults to RViz (rviz:=true)
to WATCH the robot walk, with gzserver headless (gui:=false). Pass gui:=true to
also attempt the (fragile) Gazebo GUI.

Args:
  rviz:=true|false  Open RViz (default true) — RobotModel + TF + /odom in the odom frame.
  gui:=true|false   Attempt the Gazebo GUI (default false; gzclient is unstable here).
  world:=<path>     World file (default: this package's empty.world).
  robot_name:=<s>   Gazebo entity name (default go2).
  lidar:=true|false Mount the Velodyne VLP-16 lidar (default true). Picks
                     go2_champ_description's robot_VLP.xacro (lidar wired in,
                     upstream-stock) instead of plain robot.xacro when true.
                     Needs ros-humble-velodyne-simulator in the image.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    lidar = LaunchConfiguration("lidar")

    # Default world ships with OUR (leg-locked) go2_description package.
    default_world = os.path.join(
        get_package_share_directory("go2_description"), "worlds", "empty.world"
    )
    # The CHAMP Go2 config + (renamed) walking description, from the overlay.
    go2_cfg = get_package_share_directory("go2_config")
    go2_desc = get_package_share_directory("go2_champ_description")
    # lidar:=true (default) -> robot_VLP.xacro (stock upstream file that just
    # adds an <xacro:include> of velodyne.xacro on top of robot.xacro).
    # lidar:=false -> plain robot.xacro, no lidar link/sensor at all.
    model = PythonExpression([
        "'", os.path.join(go2_desc, "xacro", "robot_VLP.xacro"),
        "' if '", lidar, "' == 'true' else '",
        os.path.join(go2_desc, "xacro", "robot.xacro"), "'",
    ])
    joints = os.path.join(go2_cfg, "config", "joints", "joints.yaml")
    links = os.path.join(go2_cfg, "config", "links", "links.yaml")
    gait = os.path.join(go2_cfg, "config", "gait", "gait.yaml")

    champ_bringup = get_package_share_directory("champ_bringup")
    champ_gazebo = get_package_share_directory("champ_gazebo")

    declare_args = [
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("robot_name", default_value="go2"),
        DeclareLaunchArgument("lidar", default_value="true"),
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("go2_description"), "rviz", "champ_8a.rviz")],
        condition=IfCondition(use_rviz),
    )

    # champ_gazebo gates its gzclient on `headless` (evaluated as a Python bool
    # via PythonExpression), NOT on `gui`. Translate our gui flag into a
    # correctly-cased "True"/"False" so " not headless" is valid Python.
    headless = PythonExpression(["'False' if '", gui, "' == 'true' else 'True'"])

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(champ_bringup, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "description_path": model,
            "joints_map_path": joints,
            "links_map_path": links,
            "gait_config_path": gait,
            "use_sim_time": "true",
            "robot_name": robot_name,
            "gazebo": "true",
            "rviz": "false",
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(champ_gazebo, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "robot_name": robot_name,
            "world": world,
            "world_init_z": "0.275",
            "gui": gui,
            "headless": headless,
            "close_loop_odom": "true",
        }.items(),
    )

    # Scope the CHAMP includes: in Humble, IncludeLaunchDescription does NOT push a
    # launch-configuration scope, so champ_bringup's own DeclareLaunchArgument("rviz")
    # (we pass rviz:=false to it) would OVERWRITE our top-level `rviz` in the shared
    # scope and silently disable our RViz. A scoped GroupAction isolates them.
    champ_stack = GroupAction([bringup, gazebo], scoped=True)

    return LaunchDescription(declare_args + [champ_stack, rviz_node])
