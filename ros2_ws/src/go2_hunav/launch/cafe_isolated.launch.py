"""
Phase 3: HuNavSim cafe scenario, ROBOT-LESS, in isolation.

This is our own minimal launch (Option B: HuNavSim integrated into our
container). It reuses the upstream HuNavSim nodes/worlds/scenarios but does NOT
spawn the PMB2 robot or nav2 — the goal is to learn HuNavSim by itself:
pedestrians walk the cafe and /people publishes their live positions.

Pipeline (mirrors upstream simulation.launch.py, minus the robot):
  1. hunav_loader            - loads agents_cafe.yaml as ROS params
  2. hunav_gazebo_world_generator - injects agents + HuNavPlugin into cafe.world
                                    -> writes generatedWorld.world
  3. gzserver (+ gzclient)   - runs the generated world
  4. hunav_agent_manager     - the behavior brain; drives agents, publishes /people

Later phases add the Go2 as the robot the agents react to.
"""
import os

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wrapper_share = get_package_share_directory("hunav_gazebo_wrapper")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    # Where libHuNavPlugin.so lives (so gzserver can load the plugin).
    wrapper_plugin_dir = os.path.join(get_package_prefix("hunav_gazebo_wrapper"), "lib")
    # Actor skins (elegant_man.dae, ...) + media live here. gzclient needs this
    # on the Gazebo paths to render the pedestrians (gzserver does not).
    wrapper_models_dir = os.path.join(wrapper_share, "models")

    # --- Launch args -------------------------------------------------------
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("rviz")
    use_go2 = LaunchConfiguration("use_go2")
    base = LaunchConfiguration("base")
    configuration_file = LaunchConfiguration("configuration_file")
    environment_name = LaunchConfiguration("environment_name")
    gz_obs = LaunchConfiguration("use_gazebo_obs")
    rate = LaunchConfiguration("update_rate")
    robot_name = LaunchConfiguration("robot_name")
    global_frame = LaunchConfiguration("global_frame_to_publish")
    use_navgoal = LaunchConfiguration("use_navgoal_to_start")
    navgoal_topic = LaunchConfiguration("navgoal_topic")
    ignore_models = LaunchConfiguration("ignore_models")
    verbose = LaunchConfiguration("verbose")

    declare_args = [
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("configuration_file", default_value="agents_cafe_regular.yaml"),
        DeclareLaunchArgument("environment_name", default_value="cafe"),
        DeclareLaunchArgument("use_gazebo_obs", default_value="true"),
        DeclareLaunchArgument("update_rate", default_value="100.0"),
        # The HuNavPlugin requires a Gazebo model with this name to exist (it
        # tracks the robot as a ROBOT agent). In Phase 3 a static placeholder
        # box (below) provides it; Phase 4 swaps in the Go2.
        DeclareLaunchArgument("robot_name", default_value="robot"),
        DeclareLaunchArgument("global_frame_to_publish", default_value="map"),
        # Start agents immediately (don't wait for a robot navigation goal).
        DeclareLaunchArgument("use_navgoal_to_start", default_value="false"),
        DeclareLaunchArgument("navgoal_topic", default_value="goal_pose"),
        DeclareLaunchArgument("ignore_models", default_value="ground_plane cafe"),
        DeclareLaunchArgument("verbose", default_value="false"),
        # RViz is the robust way to watch the pedestrians on this iGPU/Xwayland
        # setup (gzclient crashes on the heavy cafe scene). rviz=true gives the
        # visualization; set gui:=true to also try the (fragile) Gazebo GUI.
        DeclareLaunchArgument("rviz", default_value="true"),
        # Phase 4: spawn the real Go2 as the tracked robot instead of the
        # static placeholder box. Drive it with /cmd_vel among the pedestrians.
        DeclareLaunchArgument("use_go2", default_value="false"),
        # Phase 8b (OPTIONAL): which locomotion base to spawn as the robot when
        # use_go2:=true. `planar_move` (DEFAULT) = the leg-locked Phase-1 base,
        # byte-for-byte the established behavior used for ALL benchmarking. `champ`
        # = the opt-in CHAMP walking base (demo/study only; needs /opt/champ_ws).
        # Both consume the same /cmd_vel and publish /odom + TF (CLAUDE.md principle 2).
        DeclareLaunchArgument("base", default_value="planar_move"),
    ]

    # --- Gazebo resource env (so the cafe models + HuNavPlugin resolve) -----
    # GAZEBO_MODEL_PATH already includes /opt/gazebo_models (image ENV); append
    # the wrapper's bundled media/models. Also expose the plugin dir.
    # Parent of go2_description's share dir, so package://go2_description/... Go2
    # meshes resolve in Gazebo (same trick as Phase 1's spawn launch).
    go2_share_parent = os.path.dirname(get_package_share_directory("go2_description"))
    set_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.pathsep.join([
            os.environ.get("GAZEBO_MODEL_PATH", ""), wrapper_models_dir, go2_share_parent,
        ]),
    )
    set_resource_path = SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=os.environ.get("GAZEBO_RESOURCE_PATH", "") + os.pathsep + wrapper_models_dir,
    )
    set_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=os.environ.get("GAZEBO_PLUGIN_PATH", "") + os.pathsep + wrapper_plugin_dir,
    )

    # --- Config paths ------------------------------------------------------
    # Scenario lives in THIS package (we use a regular-behavior cafe variant;
    # upstream's surprised/threatening behaviors crash gzserver via a null-actor
    # deref in HuNavPlugin when the robot is pulled into the pedestrian loop).
    agent_conf_file = PathJoinSubstitution(
        [FindPackageShare("go2_hunav"), "scenarios", configuration_file]
    )
    base_world_file = PathJoinSubstitution(
        [FindPackageShare("hunav_gazebo_wrapper"), "worlds",
         [environment_name, ".world"]]
    )
    generated_world = os.path.join(wrapper_share, "worlds", "generatedWorld.world")
    gz_params_file = os.path.join(wrapper_share, "launch", "params.yaml")
    placeholder_robot_sdf = os.path.join(
        get_package_share_directory("go2_hunav"), "models", "placeholder_robot.sdf"
    )

    # --- 1) hunav_loader: load the agents YAML as params -------------------
    hunav_loader_node = Node(
        package="hunav_agent_manager",
        executable="hunav_loader",
        output="screen",
        parameters=[agent_conf_file],
    )

    # --- 2) world generator: inject agents + plugin into the base world ----
    world_generator_node = Node(
        package="hunav_gazebo_wrapper",
        executable="hunav_gazebo_world_generator",
        output="screen",
        parameters=[
            {"base_world": base_world_file},
            {"use_gazebo_obs": gz_obs},
            {"update_rate": rate},
            {"robot_name": robot_name},
            {"global_frame_to_publish": global_frame},
            {"use_navgoal_to_start": use_navgoal},
            {"navgoal_topic": navgoal_topic},
            {"ignore_models": ignore_models},
        ],
    )

    # --- 3) Gazebo (server + optional client) on the generated world -------
    gzserver = ExecuteProcess(
        cmd=[
            "gzserver", generated_world,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            "--ros-args", "--params-file", gz_params_file,
        ],
        output="screen",
        shell=False,
        on_exit=Shutdown(),
    )
    # Use gazebo_ros' gzclient launch (it waits for the gzserver master rather
    # than racing it, which a bare `gzclient` process does).
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(gui),
    )

    # The HuNavPlugin tracks the Gazebo model named robot_name. Two options:
    #  - Phase 3 (use_go2:=false): a static placeholder box.
    #  - Phase 4 (use_go2:=true):  the real leg-locked Go2 (planar_move base).
    spawn_placeholder_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        condition=UnlessCondition(use_go2),
        arguments=[
            "-file", placeholder_robot_sdf,
            "-entity", robot_name,
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
    )

    # --- Base selection (Phase 8b) ----------------------------------------
    # When use_go2:=true, `base` picks which locomotion base is the tracked robot.
    # planar_move (default) = leg-locked Phase-1 base; champ = CHAMP walking base.
    planar_on = PythonExpression(
        ["'", use_go2, "' == 'true' and '", base, "' == 'planar_move'"])
    champ_on = PythonExpression(
        ["'", use_go2, "' == 'true' and '", base, "' == 'champ'"])

    # --- Go2 robot, planar_move base (Phase 4) ----------------------------
    go2_urdf = os.path.join(
        get_package_share_directory("go2_description"), "urdf", "go2.urdf"
    )
    with open(go2_urdf, "r") as f:
        go2_description = f.read()
    go2_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        condition=IfCondition(planar_on),
        parameters=[{"robot_description": go2_description, "use_sim_time": True}],
    )
    spawn_go2 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        condition=IfCondition(planar_on),
        arguments=[
            "-topic", "robot_description",
            "-entity", robot_name,
            "-x", "0.0", "-y", "0.0", "-z", "0.33",
        ],
    )

    # --- Go2 robot, CHAMP walking base (Phase 8b, OPTIONAL) ---------------
    # The CHAMP control stack (quadruped_controller -> gait, state estimation, two
    # robot_localization EKFs -> /odom + odom->base_footprint TF, and the
    # robot_state_publisher that publishes the ACTUATED /robot_description). Spawns
    # into the SAME cafe gzserver as the tracked robot (entity == robot_name), so
    # HuNav and the pedestrians react to it identically. Mutually exclusive with the
    # planar_move path above (only one /robot_description publisher at a time).
    #
    # Scoped: champ_bringup declares its OWN `rviz` / `robot_name` launch args; in
    # Humble IncludeLaunchDescription does NOT scope launch configs, so without a
    # scoped GroupAction those would leak and clobber the cafe scene's `rviz` (kill
    # RViz) and `robot_name` (break HuNav's robot tracking). See Phase-8a gotchas.
    champ_cfg = FindPackageShare("go2_config")
    champ_bringup_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare("champ_bringup"), "launch", "bringup.launch.py"])),
                launch_arguments={
                    "description_path": PathJoinSubstitution(
                        [FindPackageShare("go2_champ_description"), "xacro", "robot.xacro"]),
                    "joints_map_path": PathJoinSubstitution(
                        [champ_cfg, "config", "joints", "joints.yaml"]),
                    "links_map_path": PathJoinSubstitution(
                        [champ_cfg, "config", "links", "links.yaml"]),
                    "gait_config_path": PathJoinSubstitution(
                        [champ_cfg, "config", "gait", "gait.yaml"]),
                    "use_sim_time": "true",
                    "robot_name": "go2",
                    "gazebo": "true",
                    "rviz": "false",
                    "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
                    "hardware_connected": "false",
                    "publish_foot_contacts": "false",
                    "close_loop_odom": "true",
                }.items(),
            )
        ],
        scoped=True,
        condition=IfCondition(champ_on),
    )
    # Spawn the actuated CHAMP Go2 from /robot_description into the cafe gzserver.
    spawn_champ = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        condition=IfCondition(champ_on),
        arguments=[
            "-topic", "robot_description",
            "-entity", robot_name,
            "-x", "0.0", "-y", "0.0", "-z", "0.3",
        ],
    )
    # Activate the ros2_control controllers once gazebo_ros2_control (in the CHAMP
    # URDF) brings up the controller_manager. The CLI retries until the service is up.
    load_js_controller = ExecuteProcess(
        condition=IfCondition(champ_on),
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_states_controller"],
        output="screen",
    )
    load_effort_controller = ExecuteProcess(
        condition=IfCondition(champ_on),
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_group_effort_controller"],
        output="screen",
    )
    # Foot-contact sensor: publishes /foot_contacts from the Gazebo foot contacts.
    # REQUIRED for odometry — champ's state_estimation derives /odom/raw (and thus
    # /odom + the odom->base_footprint TF) from joint_states + /foot_contacts. Without
    # it /odom stays frozen at 0 and the robot looks stuck in RViz even though it
    # physically walks. (In 8a this came from champ_gazebo's launch; the cafe branch
    # spawns into HuNav's gzserver instead, so we add the node explicitly here.)
    champ_contact_sensor = Node(
        package="champ_gazebo",
        executable="contact_sensor",
        output="screen",
        condition=IfCondition(champ_on),
        parameters=[
            {"use_sim_time": True},
            PathJoinSubstitution([champ_cfg, "config", "links", "links.yaml"]),
        ],
    )

    # --- 4) hunav behavior manager: drives agents, publishes /people -------
    hunav_manager_node = Node(
        package="hunav_agent_manager",
        executable="hunav_agent_manager",
        name="hunav_agent_manager",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # /people -> MarkerArray bridge + RViz, so pedestrians are viewable.
    people_markers_node = Node(
        package="go2_hunav",
        executable="people_markers.py",
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("go2_hunav"), "rviz", "cafe.rviz")],
        condition=IfCondition(use_rviz),
    )

    # map -> odom so the agent frames have a root (no robot localization here).
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    # --- Ordering: loader -> (2s) world gen -> (2s) gazebo -----------------
    after_loader = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg="hunav_loader started; launching world generator in 2s..."),
                TimerAction(period=2.0, actions=[world_generator_node]),
            ],
        )
    )
    # By the time the world generator starts it has already read hunav_loader's
    # params, so the loader is guaranteed up -- safe to start the agent manager
    # here (avoids the startup race where the manager falls back to warehouse
    # defaults and crashes). Gazebo + robot follow 2s later.
    after_worldgen = RegisterEventHandler(
        OnProcessStart(
            target_action=world_generator_node,
            on_start=[
                LogInfo(msg="world generator started; starting agent manager, "
                            "then Gazebo in 2s..."),
                hunav_manager_node,
                TimerAction(period=2.0, actions=[
                    gzserver, gzclient, spawn_placeholder_robot, spawn_go2,
                    spawn_champ, load_js_controller, load_effort_controller]),
                # contact_sensor a bit later, once the robot model (and its Gazebo
                # foot contact sensors) exist, so /foot_contacts -> /odom works.
                TimerAction(period=6.0, actions=[champ_contact_sensor]),
            ],
        )
    )

    return LaunchDescription(
        declare_args
        + [set_model_path, set_resource_path, set_plugin_path,
           hunav_loader_node, after_loader, after_worldgen,
           static_tf_node, people_markers_node, rviz_node, go2_rsp,
           champ_bringup_group]
    )
